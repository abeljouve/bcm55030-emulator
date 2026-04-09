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
use crate::soc::boot_rom::FIRMWARE_BASE;

/// Register all BCM55030 hooks on the CPU.
/// Called once at startup, before execution begins.
///
/// All firmware hook addresses are computed as `FIRMWARE_BASE + file_offset` so that
/// they match the runtime PCs after `boot_rom_start_app` loads firmware at
/// `0x32000`. The Ghidra "ram:" addresses for firmware (with image base
/// `0x20000000`) map to file offsets via `file_offset = the decompiler_addr - 0x20000000`.
pub fn register_hooks(hooks: &mut HookTable) {
    // ── Boot ROM minimal intercept ───────────────────────────────────────
    //
    // ★ IMPORTANT (2026-04-09): Hardware validation via `mem/rm` showed that
    // the addresses we previously called "boot ROM functions" (0x74B60,
    // 0x74E24, 0x74BD8, 0x79190, 0x79450, 0x78F54, 0x78F78, 0x79214, 0x79238)
    // are all INSIDE the firmware binary at runtime. They map to file offsets
    // 0x42xxx/0x47xxx, contain real firmware code, and execute natively on the
    // real BCM55030. The boot ROM is much simpler than we thought — it just
    // loads the binary from flash and jumps into it. ALL of `hw_init`,
    // `crt_main` (.data copy + BSS clear), exception handlers, and the log
    // write callback are firmware functions that run natively from firmware.
    //
    // We removed every `boot_rom_*` hook except `boot_rom_start_app`, which
    // emulates the mask ROM's job of staging the binary into ICCM/DCCM (we
    // don't model the DMA path the real boot ROM uses).
    //
    // Hardware proofs (level1/mem_rm_*.txt + Ghidra read_memory):
    //   mem/rm 0x74e24 → matches Ghidra ram:20042e24 byte-for-byte
    //   mem/rm 0x79450 → matches Ghidra ram:20047450 byte-for-byte
    //   mem/rm 0x74bd8 → matches Ghidra ram:20042bd8 byte-for-byte
    //   ...all "boot ROM" addresses are inside the loaded binary.

    // "Start app" — bootloader does `jl 0x32000` after staging firmware in DCCM.
    // The hook loads firmware from flash to ICCM/DCCM at FIRMWARE_BASE and sets
    // PC=FIRMWARE_BASE. The hook fires twice: once for the bootloader's `jl`
    // call, once for the subsequent step at PC=FIRMWARE_BASE. The second fire
    // returns Continue (see early return in boot_rom_start_app).
    hooks.insert(FIRMWARE_BASE, Hook::Custom(boot_rom::boot_rom_start_app));

    // ── Hardware feature stubs (real hardware behavior we don't emulate) ──

    // serdes_reg_read_byte (file 0x12CA4) — return 0xFF for non-SPI bus types.
    // BCM55030 has dedicated SerDes register buses (type 0x00) that we lack;
    // 0xFF satisfies calibration/ready checks.
    hooks.insert(FIRMWARE_BASE + 0x12CA4, Hook::Custom(serdes_reg_read_byte_stub));

    // serdes_reg_write (file 0x12CD8) — skip for non-SPI bus types.
    hooks.insert(FIRMWARE_BASE + 0x12CD8, Hook::Custom(serdes_reg_write_stub));

    // serdes_hw_ready_flag (file 0x1E6C) — always return 1 (ready).
    hooks.insert(FIRMWARE_BASE + 0x1E6C, Hook::ReturnValue(1));

    // ── Workarounds being re-evaluated post architectural fix ────────────
    //
    // event_dispatch_stub (file 0x33A50): the firmware's event system stores
    // handler addresses +4 past function entry. With the corrected base, this
    // may behave correctly natively — to be re-validated.
    hooks.insert(FIRMWARE_BASE + 0x33A50, Hook::Custom(event_dispatch_stub));

    // Boot ROM IRQ generic handlers (formerly at offsets 0x80-0xF8): removed.
    // The firmware installs its own ISR addresses into the IVT during init,
    // and the IVT mirror in memory.rs (range FIRMWARE_BASE..FIRMWARE_BASE+0x100)
    // propagates those data writes to ICCM. The CPU's check_interrupts path
    // jumps to the installed handler natively.

    // ── Firmware init milestones (debug tracing) ─────────────────────────────
    hooks.insert(FIRMWARE_BASE + 0x0020C, Hook::Log("firmware_main_loop ENTRY"));
    hooks.insert(FIRMWARE_BASE + 0x01B98, Hook::Log("serdes_hw_init_lanes_and_dma"));
    hooks.insert(FIRMWARE_BASE + 0x16138, Hook::Log("cli_uart_init"));
    hooks.insert(FIRMWARE_BASE + 0x3C224, Hook::Log("serdes_config_fds_init"));
    hooks.insert(FIRMWARE_BASE + 0x128E8, Hook::Log("serdes_load_config_and_reinit"));
    hooks.insert(FIRMWARE_BASE + 0x1366C, Hook::Log("  serdes_init_all_lanes_hw"));
    hooks.insert(FIRMWARE_BASE + 0x14AB0, Hook::Log("  mpcp_send_RegisterReq_with_speed"));
    hooks.insert(FIRMWARE_BASE + 0x13F94, Hook::Log("  serdes_lane2_init_pon_rx"));
    hooks.insert(FIRMWARE_BASE + 0x14670, Hook::Log("  serdes_lane0_reinit_rate_change"));
    hooks.insert(FIRMWARE_BASE + 0x000F8, Hook::Log("firmware_update_check_and_trigger"));
    hooks.insert(FIRMWARE_BASE + 0x3BB30, Hook::Log("epon_link_init"));
    hooks.insert(FIRMWARE_BASE + 0x046D0, Hook::Log("sfp_serial_bus_read_and_configure"));
    hooks.insert(FIRMWARE_BASE + 0x09834, Hook::Log("epon_runtime_full_init"));
    hooks.insert(FIRMWARE_BASE + 0x3573C, Hook::Log("hw_check_fatal_error_status"));
    hooks.insert(FIRMWARE_BASE + 0x099CC, Hook::Log("system_shutdown_and_flush"));
    hooks.insert(FIRMWARE_BASE + 0x06680, Hook::Log("epon_rx_flag_clear_init"));
    hooks.insert(FIRMWARE_BASE + 0x1AE2C, Hook::Log("mpcp_slot_and_timing_init"));
    hooks.insert(FIRMWARE_BASE + 0x20FD4, Hook::Log("hw_config_load_and_reset_init"));
    hooks.insert(FIRMWARE_BASE + 0x3C4B4, Hook::Log("epon_llid_queue_table_init"));
    hooks.insert(FIRMWARE_BASE + 0x16014, Hook::Log("cli_poll_and_process_input"));
    hooks.insert(FIRMWARE_BASE + 0x02750, Hook::Log("irq_setup_vector_and_enable"));
    // Remaining init functions after irq_setup_vector_and_enable
    hooks.insert(FIRMWARE_BASE + 0x2F800, Hook::Log("stats_counter_reset_all_init"));
    hooks.insert(FIRMWARE_BASE + 0x07C1C, Hook::Log("mpcp_register_ack_init"));
    hooks.insert(FIRMWARE_BASE + 0x19D74, Hook::Log("system_load_hw_config_from_fds"));
    hooks.insert(FIRMWARE_BASE + 0x0BD14, Hook::Log("epon_llid_init_all_channels"));
    hooks.insert(FIRMWARE_BASE + 0x09AD8, Hook::Log("macsec_hw_session_init"));
    hooks.insert(FIRMWARE_BASE + 0x06B78, Hook::Log("epon_rx_and_mka_init"));
    hooks.insert(FIRMWARE_BASE + 0x010CC, Hook::Log("mpcp_slot_config_init_from_fds"));
    hooks.insert(FIRMWARE_BASE + 0x1C400, Hook::Log("dpoe_queue_config_init"));
    hooks.insert(FIRMWARE_BASE + 0x0A2C8, Hook::Log("serdes_apply_pending_speed_change"));
    hooks.insert(FIRMWARE_BASE + 0x01880, Hook::Log("llid_all_channels_init_and_deactivate"));
    hooks.insert(FIRMWARE_BASE + 0x04138, Hook::Log("fds_init_default_hw_record_if_missing"));
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
