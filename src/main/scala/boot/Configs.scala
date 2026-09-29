package testchipip.boot

import chisel3._
import org.chipsalliance.cde.config.{Parameters, Config}
import sifive.blocks.devices.uart.{UARTParams}
import testchipip.soc.{SubsystemInjectorKey}

//---------------------------
// Bringup/Boot Configs
//---------------------------

// Specify which Tiles will stay in reset, controlled by the TileResetCtrl block
class WithTilesStartInReset(harts: Int*) extends Config((site, here, up) => {
  case TileResetCtrlKey => up(TileResetCtrlKey, site).copy(initResetHarts = up(TileResetCtrlKey, site).initResetHarts ++ harts)
})

// Specify the parameters for the BootAddrReg
class WithBootAddrReg(params: BootAddrRegParams = BootAddrRegParams()) extends Config((site, here, up) => {
  case BootAddrRegKey => Some(params)
  case SubsystemInjectorKey => up(SubsystemInjectorKey) + BootAddrRegInjector
})

// Require a one-shot full-width magic write before the next boot-address write.
class WithBootAddrRegWriteProtection(
  protection: BootAddrRegWriteProtectionParams = BootAddrRegWriteProtectionParams())
    extends Config((site, here, up) => {
  case BootAddrRegKey => up(BootAddrRegKey, site).map(_.copy(writeProtection = Some(protection)))
})

// Remove the BootAddrReg from the syste. This will likely break the default bootrom
class WithNoBootAddrReg extends Config((site, here, up) => {
  case BootAddrRegKey => None
})

// Attach a boot-select pin to the system with given parameters
class WithCustomBootPin(params: CustomBootPinParams = CustomBootPinParams()) extends Config((site, here, up) => {
  case CustomBootPinKey => Some(params)
})

// Specify the alternate boot addres the custom boot pin will select
class WithCustomBootPinAltAddr(address: BigInt) extends Config((site, here, up) => {
  case CustomBootPinKey => up(CustomBootPinKey, site).map(p => p.copy(
    customBootAddress = address,
    customBootAddresses = Nil,
    autoBootFromReset = false))
})

// Replace the legacy trigger/address pair with a reset-time selector over four Scala parameters.
class WithCustomBootPinAddresses(addresses: Seq[BigInt]) extends Config((site, here, up) => {
  case CustomBootPinKey => up(CustomBootPinKey, site).map(p => p.copy(
    customBootAddresses = addresses,
    autoBootFromReset = true))
})

// Bound, verify and report the custom-boot sequence (CustomBootPinParams.hardened).
class WithCustomBootPinHardening(responseTimeoutCycles: Int = 1024) extends Config((site, here, up) => {
  case CustomBootPinKey => up(CustomBootPinKey, site).map(p => p.copy(
    hardened = true,
    responseTimeoutCycles = responseTimeoutCycles))
})

// Verification only: +custom_boot_corrupt_address=1 corrupts the written boot address.
class WithCustomBootPinFaultInjection extends Config((site, here, up) => {
  case CustomBootPinKey => up(CustomBootPinKey, site).map(_.copy(enableFaultInjection = true))
})

// Remove the boot-select pin from the system
class WithNoCustomBootPin extends Config((site, here, up) => {
  case CustomBootPinKey => None
})
