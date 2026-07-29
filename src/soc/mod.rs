//! BCM55030 SoC emulation — Peripheral models only.
//!
//! Peripheral trait + bank + the modelled peripherals (UART, PBC, SPI
//! flash, SFP EEPROM, BSC I²C, and the rest). Anything in the SYSREG
//! range not yet carved into its own module is served by
//! [`sysreg_shim::SysregShim`].
//!
//! **No firmware hooks.** The emulator models the hardware, not any
//! particular firmware image: there are no per-PC hooks. UART input
//! arrives through the mpsc channel exposed by the bank
//! (`PeripheralBank::uart_rx_sender()`).

pub mod alarm_events;
pub mod lane_bus;
pub mod analysis;
pub mod bank;
pub mod boot_rom;
pub mod bsc_i2c;
pub mod default_store;
pub mod dma;
pub mod efuse_udr;
pub mod epon_mac;
pub mod fatal_filter;
pub mod macsec;
pub mod mmio_blocks;
pub mod mmio_init;
pub mod mpcp;
pub mod mpcp_tssync;
pub mod nco;
pub mod olt;
pub mod pbc;
pub mod peripheral;
pub mod scenario;
pub mod serdes;
pub mod sfp_eeprom;
pub mod spi_flash;
pub mod sysreg_shim;
pub mod timer;
pub mod uart;
pub mod lue;
pub mod mac_filter;
