package testchipip.boot

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.tilelink._
import freechips.rocketchip.devices.tilelink._
import freechips.rocketchip.regmapper._
import freechips.rocketchip.subsystem._

case class CustomBootPinParams(
  customBootAddress: BigInt = 0x80000000L, // Default is DRAM_BASE
  masterWhere: TLBusWrapperLocation = CBUS, // This needs to write to clint and bootaddrreg, which are on CBUS/PBUS
  customBootAddresses: Seq[BigInt] = Nil,
  autoBootFromReset: Boolean = false
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
}

case object CustomBootPinKey extends Field[Option[CustomBootPinParams]](None)

trait CanHavePeripheryCustomBootPin { this: BaseSubsystem =>
  val custom_boot_pin = p(CustomBootPinKey).map { params =>
    require(p(BootAddrRegKey).isDefined, "CustomBootPin relies on existence of BootAddrReg")
    val tlbus = locateTLBusWrapper(params.masterWhere)
    val clientParams = TLMasterPortParameters.v1(
      clients = Seq(TLMasterParameters.v1(
        name = "custom-boot",
        sourceId = IdRange(0, 1),
      )),
      minLatency = 1
    )

    val inner_io = tlbus {
      val node = TLClientNode(Seq(clientParams))
      tlbus.coupleFrom(s"port_named_custom_boot_pin") ({ _ := node })

      InModuleBody {
        val custom_boot = IO(Input(UInt(params.selectorWidth.W))).suggestName("custom_boot")
        val (tl, edge) = node.out(0)
        val (inactive :: waiting_strap :: waiting_unlock_a :: waiting_unlock_d ::
          waiting_bootaddr_reg_a :: waiting_bootaddr_reg_d :: waiting_msip_a ::
          waiting_msip_d :: dead :: Nil) = Enum(9)
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

        tl.a.valid := false.B
        tl.a.bits := DontCare
        tl.d.ready := true.B
        switch (state) {
          is (inactive) {
            when (custom_boot.orR) {
              selectorLatched := custom_boot
              state := firstWriteState
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
            }
          }
          is (waiting_bootaddr_reg_a) {
            tl.a.valid := true.B
            tl.a.bits := edge.Put(
              toAddress = bootRegParams.bootRegAddress.U,
              fromSource = 0.U,
              lgSize = 2.U,
              data = selectedBootAddress
            )._2
            when (tl.a.fire) { state := waiting_bootaddr_reg_d }
          }
          is (waiting_bootaddr_reg_d) {
            when (tl.d.fire) {
              state := Mux(responseFailed, dead, waiting_msip_a)
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
          is (waiting_msip_d) { when (tl.d.fire) { state := dead } }
          is (dead) {
            if (!params.autoBootFromReset) {
              when (!custom_boot.orR) { state := inactive }
            }
          }
        }
        custom_boot
      }
    }
    val outer_io = InModuleBody {
      val custom_boot = IO(Input(UInt(params.selectorWidth.W))).suggestName("custom_boot")
      inner_io := custom_boot
      custom_boot
    }
    outer_io
  }
}
