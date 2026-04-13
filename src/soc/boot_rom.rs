/// BCM55030 boot-ROM-related constants.
///
/// The BCM55030 has no "mask ROM" in the classical sense: hardware DMA
/// copies the first 64 KB of SPI flash into SRAM at reset and the CPU
/// starts executing from SRAM 0x0000. Every function we once called "boot
/// ROM" turned out to be regular firmware code loaded from flash (verified
/// via `mem/rm` on the live target vs. Ghidra `read_memory` byte-for-byte).
///
/// As of 2026-04-13 there is no `boot_rom_*` intercept left. The
/// bootloader reads TKF headers, programs the PBC DMA engine, copies the
/// selected firmware slot to SRAM at `FIRMWARE_BASE`, and jumps there — all
/// natively. The emulator only needs:
///   - `boot_from_flash` in `main.rs` to seed SRAM from the 64 KB HW DMA
///   - `PeripheralBusController` in `soc/pbc.rs` to model the flash →
///     SRAM DMA path the firmware programs
///
/// See `spi_dma_setup_transfer @ bootloader 0x4a68` and the caller chain
/// `FUN_00001ee0 → FUN_00001e40 → spi_flash_read → spi_dma_setup_transfer`
/// for the install path.

/// Runtime base where firmware is loaded in SRAM. Hardware-validated:
/// `mem/rm 0x32000` on the real BCM55030 returns the firmware IVT signature.
/// The bootloader stays in place at `0..0xA800`; firmware sits above it at
/// this base. All firmware PC values, hook addresses, and literal pool
/// absolutes encoded by the linker assume this base.
pub const FIRMWARE_BASE: u32 = 0x32000;
