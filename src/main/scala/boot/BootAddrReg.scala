package testchipip.boot

import chisel3._
import chisel3.util.Cat
import org.chipsalliance.cde.config.{Parameters, Field}
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.tilelink._
import freechips.rocketchip.devices.tilelink._
import freechips.rocketchip.regmapper._
import freechips.rocketchip.subsystem._
import testchipip.soc.{SubsystemInjector}

case class BootAddrRegWriteProtectionParams(
  unlockOffset: Int = 0x8,
  magic: BigInt = 0x5a4c4254L // "ZLBT"; an accidental-write guard, not a security credential
) {
  require(unlockOffset >= 8 && unlockOffset < 4096 && (unlockOffset % 4 == 0),
    "boot-address unlock offset must be a 32-bit-aligned location after the 64-bit boot register")
  require(magic >= 0 && magic.bitLength <= 32, "boot-address unlock magic must fit in 32 bits")
}

case class BootAddrRegParams(
  defaultBootAddress: BigInt = 0x80000000L, // This should be DRAM_BASE
  bootRegAddress: BigInt = 0x1000,
  slaveWhere: TLBusWrapperLocation = PBUS,
  writeProtection: Option[BootAddrRegWriteProtectionParams] = None
) {
  require(defaultBootAddress >= 0 && defaultBootAddress.bitLength <= 64,
    "default boot address must fit in 64 bits")
}
case object BootAddrRegKey extends Field[Option[BootAddrRegParams]](None)

/** One-shot accidental-write guard used by protected BootAddrReg configurations.
  *
  * Byte masks are explicit so a partial magic write actively relocks the guard, and any write
  * attempt to the boot-address word consumes an existing authorization. The public magic protects
  * against accidental/random writes; it does not authenticate a malicious requester.
  */
class BootAddrRegWriteGuard(defaultBootAddress: BigInt, magic: BigInt) extends Module {
  require(defaultBootAddress >= 0 && defaultBootAddress.bitLength <= 64)
  require(magic >= 0 && magic.bitLength <= 32)

  val io = IO(new Bundle {
    val magicWriteMask = Input(UInt(4.W))
    val magicWriteData = Input(UInt(32.W))
    val bootWriteMask = Input(UInt(4.W))
    val bootWriteData = Input(UInt(32.W))
    val bootAddress = Output(UInt(64.W))
    val armed = Output(Bool())
  })

  private val bootAddress = RegInit(defaultBootAddress.U(64.W))
  private val armed = RegInit(false.B)

  private val magicAttempt = io.magicWriteMask.orR
  private val completeMagic = io.magicWriteMask.andR && io.magicWriteData === magic.U(32.W)
  private val bootWriteAttempt = io.bootWriteMask.orR
  private val completeAlignedBootWrite = io.bootWriteMask.andR && io.bootWriteData(1, 0) === 0.U

  when (magicAttempt) {
    armed := completeMagic
  }
  when (bootWriteAttempt) {
    armed := false.B
    when (armed && completeAlignedBootWrite) {
      bootAddress := Cat(0.U(32.W), io.bootWriteData)
    }
  }

  io.bootAddress := bootAddress
  io.armed := armed
}

case object BootAddrRegInjector extends SubsystemInjector((p, baseSubsystem) => {
  p(BootAddrRegKey).map { params =>
    implicit val q: Parameters = p
    val tlbus = baseSubsystem.locateTLBusWrapper(params.slaveWhere)
    val device = new SimpleDevice("boot-address-reg", Nil)

    tlbus {
      val node = TLRegisterNode(Seq(AddressSet(params.bootRegAddress, 4096-1)), device, "reg/control", beatBytes=tlbus.beatBytes)
      tlbus.coupleTo("boot-address-reg") { node := TLFragmenter(tlbus, Some("BootAddrReg")) := _ }
      InModuleBody {
        params.writeProtection match {
          case None =>
            // Preserve the stock testchipip behavior for every configuration that does not opt in.
            val bootAddrReg = RegInit(params.defaultBootAddress.U(64.W))
            node.regmap(0 -> RegField.bytes(bootAddrReg))

          case Some(protection) =>
            val guard = Module(new BootAddrRegWriteGuard(params.defaultBootAddress, protection.magic))
            val bootWriteMask = WireDefault(VecInit.fill(4)(false.B))
            val bootWriteData = WireDefault(VecInit.fill(4)(0.U(8.W)))
            val magicWriteMask = WireDefault(VecInit.fill(4)(false.B))
            val magicWriteData = WireDefault(VecInit.fill(4)(0.U(8.W)))

            def protectedBootByte(index: Int): RegField = {
              val high = 8 * (index + 1) - 1
              val low = 8 * index
              RegField(8, guard.io.bootAddress(high, low), RegWriteFn((fire, data) => {
                bootWriteMask(index) := fire
                when (fire) { bootWriteData(index) := data }
                true.B
              }))
            }

            def magicByte(index: Int): RegField = RegField(8, 0.U(8.W), RegWriteFn((fire, data) => {
              magicWriteMask(index) := fire
              when (fire) { magicWriteData(index) := data }
              true.B
            }))

            guard.io.bootWriteMask := bootWriteMask.asUInt
            guard.io.bootWriteData := bootWriteData.asUInt
            guard.io.magicWriteMask := magicWriteMask.asUInt
            guard.io.magicWriteData := magicWriteData.asUInt

            node.regmap(
              0 -> (Seq.tabulate(4)(protectedBootByte) ++
                Seq(RegField.r(32, guard.io.bootAddress(63, 32)))),
              protection.unlockOffset -> Seq.tabulate(4)(magicByte))
        }
      }
    }
  }
})
