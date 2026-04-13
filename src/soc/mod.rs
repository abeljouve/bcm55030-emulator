//! BCM55030 SoC emulation — Peripheral models only.
//!
//! Session 1 layout: peripheral trait + bank + first wave of peripherals
//! (UART, PBC, SPI flash, SFP EEPROM, BSC I²C). Everything else in the
//! SYSREG range is still served by [`sysreg_shim::SysregShim`], which
//! will shrink as future sessions land dedicated peripheral modules.
//!
//! **No firmware hooks.** Per the contributor guide §1, the previous
//! `register_hooks()` function at 35 firmware-PC entries is gone — the
//! stdin replay path is replaced by the mpsc channel exposed by the
//! bank (`PeripheralBank::uart_rx_sender()`).

pub mod alarm;
pub mod bank;
pub mod boot_rom;
pub mod bsc_i2c;
pub mod default_store;
pub mod epon_mac;
pub mod mmio_blocks;
pub mod mmio_init;
pub mod pbc;
pub mod peripheral;
pub mod serdes;
pub mod sfp_eeprom;
pub mod spi_flash;
pub mod sysreg_shim;
pub mod uart;
