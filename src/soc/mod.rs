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
