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
/// via DMA). We detect the firmware signature (0x214A0000), find the matching
/// flash section, copy it to ICCM, and reset the CPU for firmware execution.
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

    // Determine app size by matching DCCM staging data against flash sections
    let app_size = {
        let mmio = match mem.mmio() {
            Some(m) => m,
            None => return Ok(HookAction::Continue),
        };
        let flash = &mmio.pbc.flash.data;

        let match_len = 64;
        let mut staging_bytes = vec![0u8; match_len];
        for i in 0..match_len {
            staging_bytes[i] = mem.read_byte(mem.dccm_base + (staging + i) as u32).unwrap_or(0);
        }

        let mut found_size = 0usize;
        for &header_off in &[0x120000usize, 0x1A0000, 0x270000] {
            let code_off = header_off + 0x27;
            if code_off + match_len > flash.len() { continue; }
            if flash[code_off..code_off + match_len] == staging_bytes[..] {
                let max_size = flash.len() - code_off;
                let mut size = 0;
                while size < max_size {
                    let block_end = (size + 256).min(max_size);
                    if flash[code_off + size..code_off + block_end].iter().all(|&b| b == 0xFF) {
                        break;
                    }
                    size += 256;
                }
                found_size = size;
                eprintln!("[Boot ROM] Matched flash section at 0x{:06X}, size {} bytes", header_off, size);
                break;
            }
        }
        found_size
    };

    if app_size == 0 {
        eprintln!("[Boot ROM] Firmware signature found in DCCM but not in flash");
        return Ok(HookAction::Continue);
    }

    eprintln!("[Boot ROM] Firmware detected in DCCM at 0x{:05X}, {} bytes from flash", staging, app_size);

    // Copy firmware from DCCM staging area
    let mut app_code = vec![0u8; app_size];
    for i in 0..app_size {
        app_code[i] = mem.read_byte(mem.dccm_base + (staging + i) as u32).unwrap_or(0);
    }

    eprintln!("[Boot ROM] Loading firmware: DCCM 0x{:05X}, {} bytes (0x{:X})", staging, app_size, app_size);

    // Load firmware into ICCM (overwriting bootloader)
    mem.load_iccm(0, &app_code);
    mem.app_size = Some(app_size);

    // Copy to DCCM at offset 0 if staged elsewhere
    if staging != 0 {
        mem.load_binary(0, &app_code);
    }

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

    // Protect PCL-relative literal pool constants
    mem.protect_firmware_literals();

    // Reset CPU state for firmware.
    // Interrupts start DISABLED — the firmware's IVT area (0x80-0xF8) contains
    // startup code, not interrupt handlers. The firmware installs proper handlers
    // via irq_setup_vector_and_enable() later in the init sequence.
    *state = CpuState::new();
    state.core_regs[28] = 0x10800; // SP
    state.pc = 0;

    eprintln!("[Boot ROM] ICCM/DCCM base=0, firmware {} bytes, entry=0x00000000", app_size);
    Ok(HookAction::Skip)
}

/// Boot ROM hw_init — stub (PLL, clocks, pin mux, SerDes).
/// The emulator doesn't need hardware initialization.
pub fn boot_rom_hw_init(state: &mut CpuState, _mem: &mut Memory) -> Result<HookAction, Exception> {
    eprintln!("[Boot ROM] 0x{:05X}: boot_rom_hw_init — early HW init (stub, return to 0x{:05X})",
        state.pc, state.core_regs[31]);
    state.pc = state.core_regs[31];
    state.instruction_count += 1;
    Ok(HookAction::Skip)
}

/// Boot ROM crt_main — clears BSS, jumps to firmware_main_loop (0x20C).
pub fn boot_rom_crt_main(state: &mut CpuState, mem: &mut Memory) -> Result<HookAction, Exception> {
    let bss_start = mem.app_size.unwrap_or(0) as u32;
    let bss_end = DCCM_SIZE as u32;
    if bss_start < bss_end {
        let zeros = vec![0u8; (bss_end - bss_start) as usize];
        mem.load_binary(bss_start, &zeros);
        eprintln!(
            "[Boot ROM] 0x{:05X}: boot_rom_crt_main — BSS cleared 0x{:X}-0x{:X} ({} bytes), jumping to firmware_main_loop (0x20C)",
            state.pc, bss_start, bss_end, bss_end - bss_start
        );
    } else {
        eprintln!("[Boot ROM] 0x{:05X}: boot_rom_crt_main — jumping to firmware_main_loop (0x20C)", state.pc);
    }

    state.pc = 0x20C;

    // Set hardware ready flags (real boot ROM hw_init would set these)
    let _ = mem.write_byte(0x7E207, 1);

    Ok(HookAction::Skip)
}

/// Boot ROM exception handler stubs.
pub fn boot_rom_exception_handler(state: &mut CpuState, _mem: &mut Memory) -> Result<HookAction, Exception> {
    eprintln!("[Boot ROM] 0x{:05X}: boot_rom_exception_handler (stub, return to 0x{:05X})",
        state.pc, state.core_regs[31]);
    state.pc = state.core_regs[31];
    state.instruction_count += 1;
    Ok(HookAction::Skip)
}
