package testchipip.boot

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.tilelink._
import freechips.rocketchip.devices.tilelink._
import freechips.rocketchip.regmapper._
import freechips.rocketchip.subsystem._
import freechips.rocketchip.util.PlusArg

case class CustomBootPinParams(
  customBootAddress: BigInt = 0x80000000L, // Default is DRAM_BASE
  masterWhere: TLBusWrapperLocation = CBUS, // This needs to write to clint and bootaddrreg, which are on CBUS/PBUS
  customBootAddresses: Seq[BigInt] = Nil,
  autoBootFromReset: Boolean = false,
  /** Bound every step, read the boot address back, check every response, and export the outcome
    * as [[CustomBootStatus]]. Off keeps the stock sequence exactly. */
  hardened: Boolean = false,
  responseTimeoutCycles: Int = 1024,
  /** Verification only: +custom_boot_corrupt_address=1 flips bit 2 of the written boot address. The
    * hardware exists only when this is true, so no product build carries it. */
  enableFaultInjection: Boolean = false
) {
  val bootAddresses: Seq[BigInt] =
    if (customBootAddresses.nonEmpty) customBootAddresses else Seq(customBootAddress)
  val selectorWidth: Int = math.max(1, log2Ceil(bootAddresses.size))

  require(bootAddresses.nonEmpty && isPow2(bootAddresses.size),
    "custom-boot address count must be a non-empty power of two")
  require(bootAddresses.forall(a => a >= 0 && a.bitLength <= 32 && (a & 0x3) == 0),
    "every custom-boot address must be an aligned 32-bit address")
  require(!autoBootFromReset || bootAddresses.size == 4,
    "reset-time custom-boot selection requires exactly four Scala-parameter addresses")
  require(responseTimeoutCycles > 0, "custom-boot response timeout must be positive")
  require(!enableFaultInjection || hardened, "custom-boot fault injection needs the hardened sequence")
}

/** Outcome of the hardened custom-boot sequence in the current reset. `step` and `cause` are valid
  * while `failed`; `done` means every write, the read-back and the MSIP write completed cleanly. */
class CustomBootStatus extends Bundle {
  val done = Bool()
  val failed = Bool()
  val step = UInt(3.W)
  val cause = UInt(3.W)
}

object CustomBootStatus {
  val StepUnlock = 1
  val StepBootAddress = 2
  val StepReadback = 3
  val StepMsip = 4
  val CauseDenied = 1
  val CauseCorrupt = 2
  val CauseTimeout = 3
  val CauseMismatch = 4
}

case object CustomBootPinKey extends Field[Option[CustomBootPinParams]](None)

trait CanHavePeripheryCustomBootPin { this: BaseSubsystem =>
  private val customBootPinAndStatus = p(CustomBootPinKey).map { params =>
    require(p(BootAddrRegKey).isDefined, "CustomBootPin relies on existence of BootAddrReg")
    val tlbus = locateTLBusWrapper(params.masterWhere)
    val clientParams = TLMasterPortParameters.v1(
      clients = Seq(TLMasterParameters.v1(
        name = "custom-boot",
        sourceId = IdRange(0, 1),
      )),
      minLatency = 1
    )

    val statusSource =
      if (params.hardened) Some(tlbus { BundleBridgeSource(() => new CustomBootStatus) }) else None

    val inner_io = tlbus {
      val node = TLClientNode(Seq(clientParams))
      tlbus.coupleFrom(s"port_named_custom_boot_pin") ({ _ := node })

      InModuleBody {
        val custom_boot = IO(Input(UInt(params.selectorWidth.W))).suggestName("custom_boot")
        val (tl, edge) = node.out(0)
        val (inactive :: waiting_strap :: waiting_unlock_a :: waiting_unlock_d ::
          waiting_bootaddr_reg_a :: waiting_bootaddr_reg_d :: waiting_readback_a ::
          waiting_readback_d :: waiting_msip_a :: waiting_msip_d :: dead :: Nil) = Enum(11)
        val initialState = if (params.autoBootFromReset) waiting_strap else inactive
        val state = RegInit(initialState)
        val selectorLatched = RegInit(0.U(params.selectorWidth.W))

        // Product reset straps are asynchronous package inputs. Two flops provide the CDC barrier;
        // the board contract still requires both bits to be stable before reset is released.
        val selectorSync0 = RegNext(custom_boot, 0.U(params.selectorWidth.W))
        val selectorSync1 = RegNext(selectorSync0, 0.U(params.selectorWidth.W))
        val strapSettleCycles = 3
        val strapSettleCount = RegInit(0.U(log2Ceil(strapSettleCycles + 1).W))

        val bootRegParams = p(BootAddrRegKey).get
        val protection = bootRegParams.writeProtection
        val unlockAddress = protection
          .map(p => bootRegParams.bootRegAddress + p.unlockOffset)
          .getOrElse(bootRegParams.bootRegAddress)
        val unlockMagic = protection.map(_.magic).getOrElse(BigInt(0))
        val firstWriteState = if (protection.nonEmpty) waiting_unlock_a else waiting_bootaddr_reg_a
        val selectedBootAddress = if (params.bootAddresses.size == 1) {
          params.bootAddresses.head.U(32.W)
        } else {
          VecInit(params.bootAddresses.map(_.U(32.W)))(selectorLatched)
        }
        val responseFailed = tl.d.bits.denied || tl.d.bits.corrupt

        // Hardened sequence state. A failure stops the sequence without raising MSIP, as before, but
        // is now recorded with its step and cause for the owner of the status bundle to report.
        val done = RegInit(false.B)
        val failed = RegInit(false.B)
        val failStep = RegInit(0.U(3.W))
        val failCause = RegInit(0.U(3.W))
        val responseCause =
          Mux(tl.d.bits.denied, CustomBootStatus.CauseDenied.U(3.W), CustomBootStatus.CauseCorrupt.U(3.W))
        def fail(step: Int, cause: UInt): Unit = {
          failed := true.B
          failStep := step.U
          failCause := cause
          state := dead
        }
        val corruptAddress =
          if (params.enableFaultInjection) PlusArg("custom_boot_corrupt_address", width = 1).asBool
          else false.B
        val writtenBootAddress = selectedBootAddress ^ Mux(corruptAddress, 4.U(32.W), 0.U(32.W))
        // The read-back is a 4-byte Get of the boot-address word; pick its lane out of the beat.
        val readbackShift = ((bootRegParams.bootRegAddress % tlbus.beatBytes) * 8).toInt
        val readbackData = (tl.d.bits.data >> readbackShift)(31, 0)
        // Every waiting state is bounded: a lost ready or response ends the sequence as a failure.
        val stepOfState = Seq(
          waiting_unlock_a -> CustomBootStatus.StepUnlock,
          waiting_unlock_d -> CustomBootStatus.StepUnlock,
          waiting_bootaddr_reg_a -> CustomBootStatus.StepBootAddress,
          waiting_bootaddr_reg_d -> CustomBootStatus.StepBootAddress,
          waiting_readback_a -> CustomBootStatus.StepReadback,
          waiting_readback_d -> CustomBootStatus.StepReadback,
          waiting_msip_a -> CustomBootStatus.StepMsip,
          waiting_msip_d -> CustomBootStatus.StepMsip)
        val waiting = stepOfState.map(_._1 === state).reduce(_ || _)
        val timerWidth = log2Ceil(params.responseTimeoutCycles + 1)
        val timer = RegInit(0.U(timerWidth.W))
        val previousState = RegNext(state, initialState)
        when (state =/= previousState) {
          timer := 0.U
        }.elsewhen (waiting && timer =/= params.responseTimeoutCycles.U) {
          timer := timer + 1.U
        }
        val timedOut = params.hardened.B && waiting && timer === params.responseTimeoutCycles.U

        tl.a.valid := false.B
        tl.a.bits := DontCare
        tl.d.ready := true.B
        switch (state) {
          is (inactive) {
            when (custom_boot.orR) {
              selectorLatched := custom_boot
              state := firstWriteState
              done := false.B
              failed := false.B
              failStep := 0.U
              failCause := 0.U
            }
          }
          is (waiting_strap) {
            when (strapSettleCount === (strapSettleCycles - 1).U) {
              selectorLatched := selectorSync1
              state := firstWriteState
            }.otherwise {
              strapSettleCount := strapSettleCount + 1.U
            }
          }
          is (waiting_unlock_a) {
            tl.a.valid := true.B
            tl.a.bits := edge.Put(
              toAddress = unlockAddress.U,
              fromSource = 0.U,
              lgSize = 2.U,
              data = unlockMagic.U(32.W)
            )._2
            when (tl.a.fire) { state := waiting_unlock_d }
          }
          is (waiting_unlock_d) {
            when (tl.d.fire) {
              state := Mux(responseFailed, dead, waiting_bootaddr_reg_a)
              if (params.hardened) {
                when (responseFailed) { fail(CustomBootStatus.StepUnlock, responseCause) }
              }
            }
          }
          is (waiting_bootaddr_reg_a) {
            tl.a.valid := true.B
            tl.a.bits := edge.Put(
              toAddress = bootRegParams.bootRegAddress.U,
              fromSource = 0.U,
              lgSize = 2.U,
              data = writtenBootAddress
            )._2
            when (tl.a.fire) { state := waiting_bootaddr_reg_d }
          }
          is (waiting_bootaddr_reg_d) {
            when (tl.d.fire) {
              state := Mux(responseFailed, dead, if (params.hardened) waiting_readback_a else waiting_msip_a)
              if (params.hardened) {
                when (responseFailed) { fail(CustomBootStatus.StepBootAddress, responseCause) }
              }
            }
          }
          // The boot-address guard acknowledges an unauthorised write and ignores it, so only a
          // read-back shows that the selected address was really stored.
          is (waiting_readback_a) {
            tl.a.valid := true.B
            tl.a.bits := edge.Get(
              fromSource = 0.U,
              toAddress = bootRegParams.bootRegAddress.U,
              lgSize = 2.U
            )._2
            when (tl.a.fire) { state := waiting_readback_d }
          }
          is (waiting_readback_d) {
            when (tl.d.fire) {
              when (responseFailed) {
                fail(CustomBootStatus.StepReadback, responseCause)
              }.elsewhen (readbackData =/= selectedBootAddress) {
                fail(CustomBootStatus.StepReadback, CustomBootStatus.CauseMismatch.U(3.W))
              }.otherwise {
                state := waiting_msip_a
              }
            }
          }
          is (waiting_msip_a) {
            tl.a.valid := true.B
            tl.a.bits := edge.Put(
              toAddress = (p(CLINTKey).get.baseAddress + CLINTConsts.msipOffset(0)).U, // msip for hart0
              fromSource = 0.U,
              lgSize = log2Ceil(CLINTConsts.msipBytes).U,
              data = 1.U
            )._2
            when (tl.a.fire) { state := waiting_msip_d }
          }
          is (waiting_msip_d) {
            when (tl.d.fire) {
              state := dead
              if (params.hardened) {
                when (responseFailed) {
                  fail(CustomBootStatus.StepMsip, responseCause)
                }.otherwise {
                  done := true.B
                }
              }
            }
          }
          is (dead) {
            if (!params.autoBootFromReset) {
              when (!custom_boot.orR) { state := inactive }
            }
          }
        }
        // Last connect: a timeout overrides whatever the waiting state did this cycle.
        when (timedOut) {
          fail(0, CustomBootStatus.CauseTimeout.U(3.W))
          failStep := MuxLookup(state, 0.U(3.W))(stepOfState.map { case (st, step) => st -> step.U(3.W) })
        }
        statusSource.foreach { source =>
          source.bundle.done := done
          source.bundle.failed := failed
          source.bundle.step := failStep
          source.bundle.cause := failCause
        }
        custom_boot
      }
    }
    val outer_io = InModuleBody {
      val custom_boot = IO(Input(UInt(params.selectorWidth.W))).suggestName("custom_boot")
      inner_io := custom_boot
      custom_boot
    }
    (outer_io, statusSource)
  }
  val custom_boot_pin = customBootPinAndStatus.map(_._1)
  /** Outcome of the boot sequence; present only for a hardened master. */
  val custom_boot_status = customBootPinAndStatus.flatMap(_._2)
}
