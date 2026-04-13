/// BCM55030 Boot ROM intercepts.
///
/// The BCM55030 has a mask ROM (burned in silicon, not readable) that provides
/// startup functions called by the app firmware's IVT code. Since we can't
/// dump the ROM, we intercept these functions and emulate their behavior.

use crate::cpu::exception::Exception;
use crate::cpu::registers::CpuState;
use crate::hooks::HookAction;
use crate::memory::Memory;

/// Runtime base where firmware is loaded in ICCM/DCCM. Hardware-validated:
/// `mem/rm 0x32000` on the real BCM55030 returns the firmware IVT signature.
/// The bootloader stays in place at `0..0xA800`; firmware sits above it at this
/// base. All firmware PC values, hook addresses, literal pool absolutes encoded
/// by the linker assume this base.
pub const FIRMWARE_BASE: u32 = 0x32000;

/// Boot ROM "start app" — intercept at ICCM address 0x32000.
///
/// The bootloader stages firmware in DCCM at offset 0x32000 (copied from flash
/// via DMA). The boot ROM then selects the best compatible app from flash,
/// loads it to ICCM/DCCM, and starts execution.
///
/// The real boot ROM has its own compatibility check (ROM version field at
/// TKF header +0x0C). The bootloader may stage a different slot than what
/// the boot ROM ultimately selects. We emulate the boot ROM's selection:
/// scan all TKF slots and pick the one with the highest ROM version, which
/// matches the real BCM55030 boot ROM behavior.
///
/// TKF header format (0x27 bytes):
///   +0x00: u32 magic (0x00010004)
///   +0x04: u32 CRC
///   +0x0C: u32 ROM version (e.g. 0x02000700, 0x02000900)
///   +0x1E: u8  flags (bit 0 = compressed)
///   +0x20: u24 code size (big-endian, upper 3 bytes of u32)
///   +0x27: code payload starts
pub fn boot_rom_start_app(state: &mut CpuState, mem: &mut Memory) -> Result<HookAction, Exception> {
    // The hook is installed at `FIRMWARE_BASE` (= 0x32000). It fires twice:
    //  1. The bootloader does `jl 0x32000` to start firmware — first fire, we load
    //     firmware from flash and set PC=FIRMWARE_BASE.
    //  2. The CPU then steps to PC=FIRMWARE_BASE (now containing firmware's IVT slot 0,
    //     which is a J to the startup code) — second fire. We must let firmware run.
    if mem.app_size.is_some() {
        return Ok(HookAction::Continue);
    }

    let staging: usize = 0x32000;
    let firmware_signature: [u8; 4] = [0x21, 0x4A, 0x00, 0x00];

    // Check for firmware IVT signature in DCCM staging area
    {
        let b0 = mem.read_byte(mem.dccm_base + staging as u32).unwrap_or(0);
        let b1 = mem.read_byte(mem.dccm_base + staging as u32 + 1).unwrap_or(0);
        let b2 = mem.read_byte(mem.dccm_base + staging as u32 + 2).unwrap_or(0);
        let b3 = mem.read_byte(mem.dccm_base + staging as u32 + 3).unwrap_or(0);
        if [b0, b1, b2, b3] != firmware_signature {
            return Ok(HookAction::Continue); // No firmware found, let ICCM J_S [blink] handle it
        }
    }

    // Boot ROM app selection: scan all TKF slots and pick the best compatible one.
    // The real boot ROM selects based on ROM version compatibility (header +0x0C).
    // We pick the slot with the highest ROM version, matching real hardware behavior.
    let (app_code, app_size) = {
        let mmio = match mem.mmio() {
            Some(m) => m,
            None => return Ok(HookAction::Continue),
        };
        let flash = &mmio.pbc.flash.data;

        let tkf_magic: u32 = 0x00010004;
        let tkf_header_size: usize = 0x27;

        let mut best_slot: Option<(usize, u32, usize)> = None; // (header_off, rom_ver, code_size)

        for &header_off in &[0x120000usize, 0x1A0000, 0x270000] {
            if header_off + tkf_header_size >= flash.len() { continue; }

            // Check TKF magic
            let magic = u32::from_be_bytes([
                flash[header_off], flash[header_off + 1],
                flash[header_off + 2], flash[header_off + 3],
            ]);
            if magic != tkf_magic { continue; }

            // Check firmware signature at code start
            let code_off = header_off + tkf_header_size;
            if code_off + 4 > flash.len() { continue; }
            if flash[code_off..code_off + 4] != firmware_signature { continue; }

            // Read ROM version field at +0x0C
            let rom_ver = u32::from_be_bytes([
                flash[header_off + 0x0C], flash[header_off + 0x0D],
                flash[header_off + 0x0E], flash[header_off + 0x0F],
            ]);

            // Read code size from +0x20 (upper 3 bytes of u32)
            let code_size = ((flash[header_off + 0x20] as usize) << 16)
                | ((flash[header_off + 0x21] as usize) << 8)
                | (flash[header_off + 0x22] as usize);

            if code_size == 0 || code_off + code_size > flash.len() { continue; }

            crate::vlog!("[Boot ROM] TKF slot at 0x{:06X}: ROM version 0x{:08X}, code {} bytes",
                header_off, rom_ver, code_size);

            // Pick the slot with highest ROM version
            let dominated = match best_slot {
                Some((_, best_ver, _)) => rom_ver > best_ver,
                None => true,
            };
            if dominated {
                best_slot = Some((header_off, rom_ver, code_size));
            }
        }

        match best_slot {
            Some((header_off, rom_ver, code_size)) => {
                let code_off = header_off + tkf_header_size;
                let code = flash[code_off..code_off + code_size].to_vec();
                crate::vlog!("[Boot ROM] Selected slot at 0x{:06X} (ROM ver 0x{:08X}), {} bytes",
                    header_off, rom_ver, code_size);
                (code, code_size)
            }
            None => {
                crate::vlog!("[Boot ROM] No valid TKF firmware found in flash");
                return Ok(HookAction::Continue);
            }
        }
    };

    crate::vlog!(
        "[Boot ROM] Loading firmware: {} bytes (0x{:X}) at runtime base 0x{:X}",
        app_size, app_size, FIRMWARE_BASE
    );

    // Load firmware into unified SRAM at FIRMWARE_BASE.
    // The bootloader bytes at 0..0xA800 are preserved (matching real HW).
    mem.load_binary(FIRMWARE_BASE, &app_code);
    mem.app_size = Some(app_size);
    mem.app_load_base = FIRMWARE_BASE;

    // Firmware runs in the same physical address space as the bootloader
    mem.dccm_base = 0;

    // Reset CPU state for firmware.
    // Interrupts start DISABLED (E1=E2=false) — the firmware enables them via
    // irq_setup_vector_and_enable() after installing exception handlers.
    // IENABLE is set to all-enabled: the firmware never writes IENABLE itself,
    // it expects the boot ROM to have enabled all interrupt lines.
    //
    // SoC-integration fields that live in CpuState (e.g. timer1_irq wiring)
    // must survive the reset — they describe hardware, not CPU state.
    let saved_timer1_irq = state.timer1_irq;
    *state = CpuState::new();
    state.timer1_irq = saved_timer1_irq;
    state.core_regs[28] = 0x10800; // SP (firmware startup will overwrite to 0x32000)
    state.aux_ienable = 0xFFFFFFFF; // Boot ROM enables all interrupt lines
    state.pc = FIRMWARE_BASE;

    crate::vlog!(
        "[Boot ROM] SRAM base=0, firmware {} bytes, entry=0x{:05X}",
        app_size, FIRMWARE_BASE
    );
    Ok(HookAction::Skip)
}

// NOTE: previous helper functions removed (2026-04-10):
//   boot_rom_hw_init           — firmware does its own hw_init at runtime 0x79450
//   boot_rom_crt_main          — firmware does its own .data copy + BSS clear at 0x74E24
//   boot_rom_exception_handler — firmware installs its own exception handlers
//   boot_rom_log_write_callback — firmware has its own UART writer at 0x74BD8
//   read_tkf_data_size, find_data_lma, find_data_lma_by_gp, find_flash_code_start
//                              — only used by boot_rom_crt_main, no longer needed
//
// All these "boot ROM functions" we previously hooked turned out to be regular
// firmware functions inside firmware's binary (verified via `mem/rm` on real hardware
// vs. Ghidra `read_memory` byte-for-byte match). The real boot ROM is minimal —
// it just stages the binary into ICCM/DCCM (which `boot_rom_start_app` does)
// and jumps to FIRMWARE_BASE.
