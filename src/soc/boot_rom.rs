/// BCM55030 Boot ROM intercepts.
///
/// The BCM55030 has a mask ROM (burned in silicon, not readable) that provides
/// startup functions called by the app firmware's IVT code. Since we can't
/// dump the ROM, we intercept these functions and emulate their behavior.

use crate::cpu::exception::Exception;
use crate::cpu::registers::CpuState;
use crate::hooks::HookAction;
use crate::memory::{Memory, DCCM_SIZE, ICCM_SIZE};

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

    crate::vlog!("[Boot ROM] Loading firmware: {} bytes (0x{:X})", app_size, app_size);

    // Load firmware into ICCM (code) and DCCM (data)
    mem.load_iccm(0, &app_code);
    mem.load_binary(0, &app_code);
    mem.app_size = Some(app_size);

    // Fill remaining ICCM with J_S [blink] (0x7EE0)
    let fill_start = (app_size + 1) & !1;
    if fill_start < ICCM_SIZE {
        let mut fill = vec![0u8; ICCM_SIZE - fill_start];
        for chunk in fill.chunks_exact_mut(2) {
            chunk[0] = 0x7E;
            chunk[1] = 0xE0;
        }
        mem.load_iccm(fill_start as u32, &fill);
    }

    // Firmware runs at base 0
    mem.iccm_base = 0;
    mem.dccm_base = 0;

    // Install generic IRQ handlers at IVT entries 16-31 (external IRQs at offsets 0x80-0xF8).
    // The firmware's hw_irq_and_exception_init installs exception handlers (entries 1-15)
    // via hw_auxreg_write_entry → ICCM mirror. But external IRQ entries (16+) are NOT
    // written by the firmware — the boot ROM installs them before firmware starts.
    // The handler at 0x33BD0 is the firmware's generic exception/IRQ dispatch function
    // (read from literal pool at 0x1C04 by hw_auxreg_write_entry).
    // NOTE: IRQ vector stubs are installed in boot_rom_crt_main (not here) because
    // the firmware startup code at 0x60-0x98 overlaps the IVT IRQ entries (0x80-0x98).
    // The startup code must execute first before we overwrite those addresses.

    // Protect PCL-relative literal pool constants
    mem.protect_firmware_literals();

    // Reset CPU state for firmware.
    // Interrupts start DISABLED (E1=E2=false) — the firmware enables them via
    // irq_setup_vector_and_enable() after installing exception handlers.
    // IENABLE is set to all-enabled: the firmware never writes IENABLE itself,
    // it expects the boot ROM to have enabled all interrupt lines.
    *state = CpuState::new();
    state.core_regs[28] = 0x10800; // SP
    state.aux_ienable = 0xFFFFFFFF; // Boot ROM enables all interrupt lines
    state.pc = 0;

    crate::vlog!("[Boot ROM] ICCM/DCCM base=0, firmware {} bytes, entry=0x00000000", app_size);
    Ok(HookAction::Skip)
}

/// Boot ROM hw_init — stub (PLL, clocks, pin mux, SerDes).
/// The emulator doesn't need hardware initialization.
pub fn boot_rom_hw_init(state: &mut CpuState, _mem: &mut Memory) -> Result<HookAction, Exception> {
    crate::vlog!("[Boot ROM] 0x{:05X}: boot_rom_hw_init — early HW init (stub, return to 0x{:05X})",
        state.pc, state.core_regs[31]);
    state.pc = state.core_regs[31];
    state.instruction_count += 1;
    Ok(HookAction::Skip)
}

/// Boot ROM crt_main — copies .data section, clears BSS, jumps to firmware_main_loop.
///
/// The real boot ROM CRT performs C runtime initialization:
/// 1. Copy .data init values from ICCM (LMA) to their DCCM addresses (VMA)
/// 2. Zero the .bss section
/// 3. Jump to firmware_main_loop (0x20C)
///
/// The .data VMA occupies the top of DCCM: [DCCM_SIZE - data_size .. DCCM_SIZE).
/// The data_size is stored in the TKF header at +0x10.
/// The .data init values are packed in the binary at a constant offset (delta)
/// below their VMA addresses. We derive delta by finding a VMA pointer in a
/// literal pool and verifying the corresponding LMA contains valid data.
pub fn boot_rom_crt_main(state: &mut CpuState, mem: &mut Memory) -> Result<HookAction, Exception> {
    let app_size = mem.app_size.unwrap_or(0) as u32;
    let dccm_size = DCCM_SIZE as u32;

    // Read data_size from TKF header in flash (field +0x10).
    let data_size = read_tkf_data_size(mem, app_size);

    if data_size > 0 && data_size < dccm_size {
        let data_vma = dccm_size - data_size;

        // Find the flash code start offset for this app slot
        let flash_code_start = find_flash_code_start(mem, app_size);
        let binary_len = (app_size as usize).max(data_size as usize * 2);

        let data_lma = find_data_lma(mem, flash_code_start, binary_len, data_vma, data_size);

        crate::vlog!(
            "[Boot ROM] 0x{:05X}: boot_rom_crt_main — .data: flash[0x{:X}..0x{:X}] → DCCM[0x{:X}..0x{:X}] ({} bytes)",
            state.pc, data_lma, data_lma + data_size, data_vma, dccm_size, data_size
        );

        // Copy from flash to DCCM (the .data init values may extend beyond app_size in ICCM)
        {
            let mmio = mem.mmio().unwrap();
            let flash = &mmio.pbc.flash.data;
            let mut buf = vec![0u8; data_size as usize];
            for i in 0..data_size as usize {
                let flash_off = flash_code_start + data_lma as usize + i;
                if flash_off < flash.len() {
                    buf[i] = flash[flash_off];
                }
            }
            drop(mmio);
            mem.load_binary(data_vma, &buf);
        }
    }

    // Clear BSS: zero from app_size to data_vma
    let bss_end = if data_size > 0 { dccm_size - data_size } else { dccm_size };
    if app_size < bss_end {
        let zeros = vec![0u8; (bss_end - app_size) as usize];
        mem.load_binary(app_size, &zeros);
    }

    // NOTE: IVT entries 16-31 (external IRQs at offsets 0x80-0xF8) are NOT overwritten
    // with RTIE stubs in ICCM. The firmware has functions at these addresses (e.g.,
    // firmware_update_check_and_trigger at 0xF8) that are called during init.
    // Overwriting ICCM would destroy these functions.
    // Instead, IRQ dispatch is handled by hooks in soc/mod.rs that check whether
    // the CPU is in interrupt context (flag_a1/flag_a2) and perform RTIE in Rust.

    crate::vlog!(
        "[Boot ROM] 0x{:05X}: boot_rom_crt_main — BSS cleared 0x{:X}-0x{:X} ({} bytes), jumping to firmware_main_loop (0x20C)",
        state.pc, app_size, bss_end, bss_end - app_size
    );

    state.pc = 0x20C;

    // Set hardware ready flags (real boot ROM hw_init would set these)
    let _ = mem.write_byte(0x7E207, 1);

    Ok(HookAction::Skip)
}

/// Find the flash offset where this app's code starts (header_off + 0x27).
fn find_flash_code_start(mem: &Memory, app_size: u32) -> usize {
    let mmio = match mem.mmio() {
        Some(m) => m,
        None => return 0,
    };
    let flash = &mmio.pbc.flash.data;
    for &header_off in &[0x120000usize, 0x1A0000, 0x270000] {
        if header_off + 0x27 >= flash.len() { continue; }
        let code_size = ((flash[header_off + 0x20] as u32) << 16)
            | ((flash[header_off + 0x21] as u32) << 8)
            | (flash[header_off + 0x22] as u32);
        if code_size == app_size {
            return header_off + 0x27;
        }
    }
    0
}

/// Read the .data section size from the TKF header field +0x10 in flash.
fn read_tkf_data_size(mem: &mut Memory, app_size: u32) -> u32 {
    let mmio = match mem.mmio() {
        Some(m) => m,
        None => return 0,
    };
    let flash = &mmio.pbc.flash.data;
    for &header_off in &[0x120000usize, 0x1A0000, 0x270000] {
        if header_off + 0x27 >= flash.len() { continue; }
        let code_size = ((flash[header_off + 0x20] as u32) << 16)
            | ((flash[header_off + 0x21] as u32) << 8)
            | (flash[header_off + 0x22] as u32);
        if code_size == app_size {
            return u32::from_be_bytes([
                flash[header_off + 0x10], flash[header_off + 0x11],
                flash[header_off + 0x12], flash[header_off + 0x13],
            ]);
        }
    }
    0
}

/// Find the VMA-to-LMA delta for the .data section.
///
/// Searches the binary (in flash) for the format string "\n%s %X" which is
/// always present in the .data section (used by cli_print_firmware_version_info).
/// The VMA of this string is read from a literal pool at a known ICCM offset
/// (0x16118, the first literal pool of cli_print_firmware_version_info).
/// delta = VMA - LMA, and data_lma = data_vma - delta.
///
/// Returns data_lma (the offset in the binary where .data init values start).
fn find_data_lma(mem: &Memory, flash_code_start: usize, binary_len: usize, data_vma: u32, data_size: u32) -> u32 {
    let fallback = data_vma.saturating_sub(data_size);

    // Read the VMA pointer from the literal pool at ICCM 0x16118
    // This is the format string pointer used by cli_print_firmware_version_info
    let string_vma = mem.fetch_word(0x16118).unwrap_or(0);
    if string_vma < data_vma || string_vma >= data_vma + data_size {
        // Fallback: try to find VMA pointer by scanning literal pools
        return find_data_lma_by_gp(mem, flash_code_start, binary_len, data_vma, data_size);
    }

    // Search for "\n%s %X" in the flash binary
    let needle: &[u8] = &[0x0A, 0x25, 0x73, 0x20, 0x25, 0x58]; // "\n%s %X"
    let mmio = match mem.mmio() {
        Some(m) => m,
        None => return fallback,
    };
    let flash = &mmio.pbc.flash.data;

    for off in 0..binary_len.saturating_sub(needle.len()) {
        let flash_off = flash_code_start + off;
        if flash_off + needle.len() > flash.len() { break; }
        if flash[flash_off..flash_off + needle.len()] == *needle {
            let string_lma = off as u32;
            let delta = string_vma - string_lma;
            let data_lma = data_vma.saturating_sub(delta);
            crate::vlog!(
                "[Boot ROM] .data delta: string VMA=0x{:X} LMA=0x{:X} delta=0x{:X} → data_lma=0x{:X}",
                string_vma, string_lma, delta, data_lma
            );
            return data_lma;
        }
    }

    fallback
}

/// Fallback: find data_lma using GP register heuristic.
fn find_data_lma_by_gp(mem: &Memory, flash_code_start: usize, binary_len: usize, data_vma: u32, data_size: u32) -> u32 {
    let fallback = data_vma.saturating_sub(data_size);

    let gp = mem.fetch_word(0x7C).unwrap_or(0);
    if gp < data_vma || gp >= data_vma + data_size {
        return fallback;
    }

    let mmio = match mem.mmio() {
        Some(m) => m,
        None => return fallback,
    };
    let flash = &mmio.pbc.flash.data;
    let gp_offset_in_data = gp - data_vma;
    let data_end = data_vma + data_size;

    let mut best_score = 0u32;
    let mut best_lma = fallback;

    let lma_max = binary_len.saturating_sub(data_size as usize) as u32;
    let mut lma = 0u32;
    while lma <= lma_max {
        let gp_lma_flash = flash_code_start + lma as usize + gp_offset_in_data as usize;
        if gp_lma_flash + 128 > flash.len() { break; }

        let mut score = 0u32;
        for i in 0..32u32 {
            let off = gp_lma_flash + (i * 4) as usize;
            if off + 3 < flash.len() {
                let w = u32::from_be_bytes([flash[off], flash[off+1], flash[off+2], flash[off+3]]);
                if w >= data_vma && w < data_end {
                    score += 1;
                }
            }
        }
        if score > best_score {
            best_score = score;
            best_lma = lma;
        }
        lma += 4;
    }

    if best_score >= 4 { best_lma } else { fallback }
}

/// Boot ROM exception handler stubs.
pub fn boot_rom_exception_handler(state: &mut CpuState, _mem: &mut Memory) -> Result<HookAction, Exception> {
    crate::vlog!("[Boot ROM] 0x{:05X}: boot_rom_exception_handler (stub, return to 0x{:05X})",
        state.pc, state.core_regs[31]);
    state.pc = state.core_regs[31];
    state.instruction_count += 1;
    Ok(HookAction::Skip)
}
