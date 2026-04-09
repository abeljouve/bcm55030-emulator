/// BCM55030 SoC emulation.
///
/// This module contains all BCM55030-specific behavior: boot ROM intercepts,
/// SerDes register stubs, init milestones, and peripheral workarounds.
/// The core ARC700 CPU emulation (decoder, executor, memory) has no knowledge
/// of the BCM55030 — all SoC-specific behavior is injected via hooks.

pub mod boot_rom;
pub mod mmio;
pub mod pbc;
pub mod spi_flash;
pub mod uart;

use crate::cpu::exception::Exception;
use crate::cpu::registers::CpuState;
use crate::hooks::{Hook, HookAction, HookTable};
use crate::memory::Memory;

/// Register all BCM55030 hooks on the CPU.
/// Called once at startup, before execution begins.
pub fn register_hooks(hooks: &mut HookTable) {
    // ── Boot ROM intercepts ──────────────────────────────────────────────
    // These addresses are in the boot ROM area (filled with J_S [blink] stubs).
    // The real boot ROM has code here; we intercept and emulate.

    // "Start app" — bootloader jumps here after staging firmware in DCCM
    hooks.insert(0x32000, Hook::Custom(boot_rom::boot_rom_start_app));

    // boot_rom_hw_init (PLL, clocks, SerDes — safe to stub)
    // Pre-relocation: 0x79450 (Firmware), 0x79190 (Diag)
    hooks.insert(0x79190, Hook::Custom(boot_rom::boot_rom_hw_init));
    hooks.insert(0x79450, Hook::Custom(boot_rom::boot_rom_hw_init));

    // boot_rom_crt_main (CRITICAL — BSS clear + jump to firmware_main_loop)
    // Pre-relocation: 0x74E24 (Firmware), 0x74B60 (Diag)
    hooks.insert(0x74B60, Hook::Custom(boot_rom::boot_rom_crt_main));
    hooks.insert(0x74E24, Hook::Custom(boot_rom::boot_rom_crt_main));

    // boot_rom_exception_handlers (safe to stub)
    hooks.insert(0x78F54, Hook::Custom(boot_rom::boot_rom_exception_handler));
    hooks.insert(0x79214, Hook::Custom(boot_rom::boot_rom_exception_handler));
    hooks.insert(0x78F78, Hook::Custom(boot_rom::boot_rom_exception_handler));
    hooks.insert(0x79238, Hook::Custom(boot_rom::boot_rom_exception_handler));

    // ── Firmware firmware workarounds ────────────────────────────────────────
    // These hooks handle hardware features not yet emulated. They are at
    // addresses within the firmware binary, so they only fire after firmware loads.

    // Note: 0x98 is the Timer 0 (IRQ 3) vector in the firmware IVT.
    // Previously had a restart_guard here, but it intercepted Timer 0 interrupts
    // and caused firmware_main_loop to restart instead of handling the timer ISR.

    // irq_set_pending_bit_and_call_handler (0x33A50): the event dispatch calls handler
    // table addresses (e.g., 0x3B740) that are intentionally +4 past the function entry
    // (the binary stores these addresses in ICCM literal pools). Calling at +4 skips
    // the FP prologue (st.a fp,[sp,-4]), and the shared epilogue (ld.ab fp,[sp,4])
    // pops the caller's local variable into FP → SP corruption.
    // This appears to be a boot ROM trampoline mechanism that we don't emulate.
    hooks.insert(0x33A50, Hook::Custom(event_dispatch_stub));

    // serdes_reg_read_byte — return 0xFF for non-SPI bus types.
    // The BCM55030 has dedicated SerDes register buses (type 0x00) that the
    // emulator lacks. Returning 0xFF satisfies calibration/ready checks.
    hooks.insert(0x12CA4, Hook::Custom(serdes_reg_read_byte_stub));

    // serdes_reg_write — skip for non-SPI bus types.
    hooks.insert(0x12CD8, Hook::Custom(serdes_reg_write_stub));

    // serdes_hw_ready_flag — always return 1 (ready).
    // Real boot ROM hw_init sets this flag; we stub it.
    hooks.insert(0x1E6C, Hook::ReturnValue(1));

    // Boot ROM log_printf write callback at 0x74BD8.
    // The firmware's log_printf loads this address from an ICCM literal pool and passes
    // it to vprintf_core_format_engine as the character output callback.
    // On real hardware this is a boot ROM function that writes to UART.
    // Signature: callback(context: r0, data_ptr: r1, len: r2) → r0 = bytes written
    hooks.insert(0x74BD8, Hook::Custom(boot_rom_log_write_callback));

    // Boot ROM IRQ handlers — intercept external IRQ vectors (IVT entries 16-31).
    // The firmware has code at addresses 0x80-0xF8 that gets called during init,
    // so we can't overwrite ICCM with RTIE stubs. Instead, hooks check if the CPU
    // is in interrupt context (flag_a1/flag_a2). If yes: handle the IRQ and RTIE.
    // If no: let normal firmware code execute (Continue).
    // IRQ 5 (UART, entry 21, offset 0xA8) gets special handling: drain TX ring
    // buffer to stdout and fill RX ring buffer from stdin.
    for irq in 0..16u32 {
        let vector_offset = (16 + irq) * 8;
        hooks.insert(vector_offset, Hook::Custom(boot_rom_irq_handler));
    }

    // ── Firmware init milestones (debug tracing) ─────────────────────────────
    hooks.insert(0x0020C, Hook::Log("firmware_main_loop ENTRY"));
    hooks.insert(0x01B98, Hook::Log("serdes_hw_init_lanes_and_dma"));
    hooks.insert(0x16138, Hook::Log("cli_uart_init"));
    hooks.insert(0x3C224, Hook::Log("serdes_config_fds_init"));
    hooks.insert(0x128E8, Hook::Log("serdes_load_config_and_reinit"));
    hooks.insert(0x1366C, Hook::Log("  serdes_init_all_lanes_hw"));
    hooks.insert(0x14AB0, Hook::Log("  mpcp_send_RegisterReq_with_speed"));
    hooks.insert(0x13F94, Hook::Log("  serdes_lane2_init_pon_rx"));
    hooks.insert(0x14670, Hook::Log("  serdes_lane0_reinit_rate_change"));
    hooks.insert(0x000F8, Hook::Log("firmware_update_check_and_trigger"));
    hooks.insert(0x3BB30, Hook::Log("epon_link_init"));
    hooks.insert(0x046D0, Hook::Log("sfp_serial_bus_read_and_configure"));
    hooks.insert(0x09834, Hook::Log("epon_runtime_full_init"));
    hooks.insert(0x3573C, Hook::Log("hw_check_fatal_error_status"));
    hooks.insert(0x099CC, Hook::Log("system_shutdown_and_flush"));
    hooks.insert(0x06680, Hook::Log("epon_rx_flag_clear_init"));
    hooks.insert(0x1AE2C, Hook::Log("mpcp_slot_and_timing_init"));
    hooks.insert(0x20FD4, Hook::Log("hw_config_load_and_reset_init"));
    hooks.insert(0x3C4B4, Hook::Log("epon_llid_queue_table_init"));
    hooks.insert(0x16014, Hook::Log("cli_poll_and_process_input"));
    hooks.insert(0x02750, Hook::Log("irq_setup_vector_and_enable"));
    // Remaining init functions after irq_setup_vector_and_enable
    hooks.insert(0x2F800, Hook::Log("stats_counter_reset_all_init"));
    hooks.insert(0x07C1C, Hook::Log("mpcp_register_ack_init"));
    hooks.insert(0x19D74, Hook::Log("system_load_hw_config_from_fds"));
    hooks.insert(0x0BD14, Hook::Log("epon_llid_init_all_channels"));
    hooks.insert(0x09AD8, Hook::Log("macsec_hw_session_init"));
    hooks.insert(0x06B78, Hook::Log("epon_rx_and_mka_init"));
    hooks.insert(0x010CC, Hook::Log("mpcp_slot_config_init_from_fds"));
    hooks.insert(0x1C400, Hook::Log("dpoe_queue_config_init"));
    hooks.insert(0x0A2C8, Hook::Log("serdes_apply_pending_speed_change"));
    hooks.insert(0x01880, Hook::Log("llid_all_channels_init_and_deactivate"));
    hooks.insert(0x04138, Hook::Log("fds_init_default_hw_record_if_missing"));
}

// ── Hook implementations ─────────────────────────────────────────────────

/// Event dispatch stub — skip handler calls during firmware execution.
/// The firmware's event system stores handler table addresses (from ICCM literal pools)
/// that are +4 past the actual function entry, skipping the FP save prologue.
/// On real hardware, the boot ROM likely installs trampolines at these addresses.
fn event_dispatch_stub(state: &mut CpuState, mem: &mut Memory) -> Result<HookAction, Exception> {
    if mem.app_size.is_none() {
        return Ok(HookAction::Continue);
    }
    state.pc = state.core_regs[31]; // blink
    state.instruction_count += 1;
    Ok(HookAction::Skip)
}

/// Boot ROM log_printf write callback — emulates the boot ROM function at 0x74BD8.
/// Called by vprintf_core_format_engine with (r0=context, r1=data_ptr, r2=len).
/// Reads `len` bytes from DCCM at `data_ptr` and writes them to stdout.
/// Returns len in r0 (success).
fn boot_rom_log_write_callback(state: &mut CpuState, mem: &mut Memory) -> Result<HookAction, Exception> {
    if mem.app_size.is_none() {
        return Ok(HookAction::Continue);
    }
    let data_ptr = state.core_regs[1];
    let len = state.core_regs[2];
    if len > 0 && len < 0x10000 {
        let mut buf = Vec::with_capacity(len as usize);
        for i in 0..len {
            buf.push(mem.read_byte(data_ptr + i)?);
        }
        let stdout = std::io::stdout();
        let mut handle = stdout.lock();
        let _ = std::io::Write::write_all(&mut handle, &buf);
        let _ = std::io::Write::flush(&mut handle);
    }
    state.core_regs[0] = len as u32;
    state.pc = state.core_regs[31]; // blink
    state.instruction_count += 1;
    Ok(HookAction::Skip)
}

/// Boot ROM generic IRQ handler — intercepts external IRQ vectors (IVT entries 16-31).
///
/// These hooks fire at IVT addresses 0x80-0xF8 which overlap with firmware code.
/// We distinguish IRQ dispatch from normal code execution by checking flag_a1/flag_a2
/// (set by check_interrupts when entering an ISR).
///
/// For IRQ context:
///   - IRQ 5 (UART): drain TX ring buffer to stdout, fill RX ring buffer from stdin
///   - All IRQs: perform RTIE (restore STATUS32, PC = ilink) and return Skip
///
/// For normal code: return Continue (let firmware code execute from ICCM).
///
/// UART ring buffer struct at DCCM 0x7E204:
///   +0: rx_empty (0=has data), +1: rx_write_ptr, +2: rx_read_ptr
///   +3: tx_trigger, +4: tx_write_ptr, +5: tx_read_ptr
/// TX buffer at 0x348, RX buffer at 0x248 (both 256 bytes).
fn boot_rom_irq_handler(state: &mut CpuState, mem: &mut Memory) -> Result<HookAction, Exception> {
    // Only handle firmware IRQs
    if mem.app_size.is_none() {
        return Ok(HookAction::Continue);
    }

    // Only intercept if we're in interrupt context (not normal code execution)
    if !state.flag_a1 && !state.flag_a2 {
        return Ok(HookAction::Continue);
    }

    // Determine IRQ number from PC: PC = int_vector_base + (16 + irq) * 8
    let irq = (state.pc.wrapping_sub(state.aux_int_vector_base)) / 8;
    let irq = irq.saturating_sub(16);

    // IRQ 5 = UART: process TX/RX ring buffers
    if irq == 5 {
        uart_isr_process(mem)?;
    }

    // Perform RTIE: restore STATUS32 and PC from saved interrupt state
    if state.flag_a2 {
        let saved = state.aux_status32_l2;
        state.set_status32(saved);
        state.pc = state.core_regs[crate::cpu::registers::REG_ILINK2 as usize];
        state.aux_bta = state.aux_bta_l2;
    } else {
        let saved = state.aux_status32_l1;
        state.set_status32(saved);
        state.pc = state.core_regs[crate::cpu::registers::REG_ILINK1 as usize];
        state.aux_bta = state.aux_bta_l1;
    }
    state.pc_written = true;
    state.instruction_count += 1;
    Ok(HookAction::Skip)
}

/// Process UART TX/RX ring buffers (called from boot_rom_irq_handler for IRQ 5).
fn uart_isr_process(mem: &mut Memory) -> Result<(), Exception> {
    const UART_STRUCT: u32 = 0x7E204;
    const TX_BUF: u32 = 0x348;
    const RX_BUF: u32 = 0x248;

    // Phase 1: Read UART hardware state and drain rx_queue
    let (ier, rx_bytes) = {
        let mut mmio = mem.mmio().unwrap();
        let ier = mmio.uart.ier();
        let rx: Vec<u8> = mmio.uart.rx_queue.drain(..).collect();
        (ier, rx)
    };

    // Phase 2: TX — drain firmware's TX ring buffer to stdout
    if ier & 0x40 != 0 {
        let tx_write = mem.read_byte(UART_STRUCT + 4)?;
        let mut tx_read = mem.read_byte(UART_STRUCT + 5)?;

        if tx_read != tx_write {
            let stdout = std::io::stdout();
            let mut handle = stdout.lock();
            while tx_read != tx_write {
                let byte = mem.read_byte(TX_BUF + tx_read as u32)?;
                let _ = std::io::Write::write_all(&mut handle, &[byte]);
                tx_read = tx_read.wrapping_add(1);
            }
            let _ = std::io::Write::flush(&mut handle);
            mem.write_byte(UART_STRUCT + 5, tx_read)?;
        }

        // Buffer empty: clear TXIE and re-arm trigger for next enqueue batch
        {
            let mut mmio = mem.mmio().unwrap();
            mmio.uart.ier_clear(0x40);
        }
        mem.write_byte(UART_STRUCT + 3, 1)?; // re-arm tx_trigger
    }

    // Phase 3: RX — fill firmware's RX ring buffer from stdin
    if !rx_bytes.is_empty() {
        let mut pushed = 0usize;
        for &byte in &rx_bytes {
            let rx_write = mem.read_byte(UART_STRUCT + 1)?;
            let rx_read = mem.read_byte(UART_STRUCT + 2)?;

            if rx_write.wrapping_add(1) == rx_read {
                break; // Buffer full
            }

            mem.write_byte(RX_BUF + rx_write as u32, byte)?;
            mem.write_byte(UART_STRUCT + 1, rx_write.wrapping_add(1))?;
            mem.write_byte(UART_STRUCT + 0, 0)?; // not empty
            pushed += 1;
        }

        // Push back any bytes that didn't fit
        if pushed < rx_bytes.len() {
            let mut mmio = mem.mmio().unwrap();
            for &byte in &rx_bytes[pushed..] {
                mmio.uart.rx_queue.push_back(byte);
            }
        }
    }

    // Ensure RXIE (bit 2) is set so future stdin data triggers UART IRQ.
    // The firmware only sets RXIE after a successful dequeue, but the first
    // dequeue finds an empty buffer and never sets it — chicken-and-egg.
    // The boot ROM ISR enables RXIE as part of its processing.
    {
        let mut mmio = mem.mmio().unwrap();
        mmio.uart.ier_set(0x04);
    }

    Ok(())
}

/// serdes_reg_read_byte stub — returns 0xFF for non-SPI bus types.
fn serdes_reg_read_byte_stub(state: &mut CpuState, mem: &mut Memory) -> Result<HookAction, Exception> {
    if mem.app_size.is_none() {
        return Ok(HookAction::Continue);
    }
    let bus_type = (state.core_regs[0] >> 8) & 0xFF;
    if bus_type != 0x06 {
        state.core_regs[0] = 0xFF;
        state.pc = state.core_regs[31]; // blink
        state.instruction_count += 1;
        Ok(HookAction::Skip)
    } else {
        Ok(HookAction::Continue)
    }
}

/// serdes_reg_write stub — skip for non-SPI bus types.
fn serdes_reg_write_stub(state: &mut CpuState, mem: &mut Memory) -> Result<HookAction, Exception> {
    if mem.app_size.is_none() {
        return Ok(HookAction::Continue);
    }
    let bus_type = (state.core_regs[0] >> 8) & 0xFF;
    if bus_type != 0x06 {
        state.pc = state.core_regs[31];
        state.instruction_count += 1;
        Ok(HookAction::Skip)
    } else {
        Ok(HookAction::Continue)
    }
}
