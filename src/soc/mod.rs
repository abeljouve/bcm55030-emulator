/// BCM55030 SoC emulation.
///
/// This module contains all BCM55030-specific behavior: boot ROM intercepts,
/// SerDes register stubs, init milestones, and peripheral workarounds.
/// The core ARC700 CPU emulation (decoder, executor, memory) has no knowledge
/// of the BCM55030 — all SoC-specific behavior is injected via hooks.

pub mod boot_rom;

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

    // Prevent firmware_main_loop from "returning" to startup code.
    // The function should loop forever, but some init error causes it to exit.
    hooks.insert(0x98, Hook::Custom(restart_guard));

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
    hooks.insert(0x02750, Hook::Log("irq_setup_vector_and_enable"));
}

// ── Hook implementations ─────────────────────────────────────────────────

/// Prevent firmware_main_loop from returning to startup code (0x98).
/// Redirects back to firmware_main_loop (0x20C) to retry the init.
fn restart_guard(state: &mut CpuState, mem: &mut Memory) -> Result<HookAction, Exception> {
    // Only active when firmware is loaded
    if mem.app_size.is_none() {
        return Ok(HookAction::Continue);
    }
    state.pc = 0x20C;
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
