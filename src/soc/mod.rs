/// BCM55030 SoC emulation.
///
/// This module contains all BCM55030-specific behavior: boot ROM intercepts,
/// SerDes register stubs, init milestones, and peripheral workarounds.
/// The core ARC700 CPU emulation (decoder, executor, memory) has no knowledge
/// of the BCM55030 — all SoC-specific behavior is injected via hooks.

pub mod boot_rom;
pub mod mmio;
pub mod mmio_blocks;
pub mod mmio_init;
pub mod pbc;
pub mod sfp_eeprom;
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
/// they match the runtime PCs once the bootloader loads firmware at `0x32000`.
/// The Ghidra "ram:" addresses for firmware (with image base `0x20000000`) map
/// to file offsets via `file_offset = the decompiler_addr - 0x20000000`.
///
/// Boot path is 100% native: the bootloader reads TKF headers and
/// programs the PBC DMA engine (`spi_dma_setup_transfer @ 0x4a68`) to
/// copy the selected slot into SRAM at `FIRMWARE_BASE`, then calls through
/// a function pointer into it. The emulator's PBC model in
/// `src/soc/pbc.rs` fulfills that path — no `boot_rom_start_app` hook is
/// installed.
pub fn register_hooks(hooks: &mut HookTable) {

    // `serdes_hw_ready_flag` @ firmware+0x1E6C was removed 2026-04-13. Despite the
    // Ghidra auto-name, RE shows the function is a UART TX-idle flag getter:
    // it returns `*(u8*)(0x7E207)` which `firmware_native_uart_isr` sets to 1
    // when the TX ring drains and the UART status bit 0x80 is asserted.
    // Its sole caller, `serdes_set_pon_rate_and_enable`, busy-waits on it to
    // flush pending UART output before reconfiguring the SerDes clock. With
    // the native UART ISR installed natively by firmware (prompt 07) and our
    // UART model asserting bit 0x80 unconditionally, the firmware drains
    // the ring and sets the flag naturally — no hook needed.

    // ── Workaround removed for re-validation 2026-04-10 ──────────────────
    //
    // event_dispatch_stub @ FIRMWARE_BASE+0x33A50 was the "+4 trampoline" hack
    // we believed was needed because handler table addresses were stored
    // 4 bytes past function entries. With the corrected load offset
    // (FIRMWARE_BASE = 0x32000), this might no longer be needed because the
    // handler addresses might now resolve correctly. To be re-validated.

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
    // Reboot path tracing — kept for the next phase (Phase 1 still in progress).
    // The known shutdown trigger is `epon_poll_hw_state_changes` detecting false
    // positives from un-modeled SerDes MMIO reads.
    hooks.insert(FIRMWARE_BASE + 0x084FC, Hook::Log("firmware_update_trigger"));
    hooks.insert(FIRMWARE_BASE + 0x33CF8, Hook::Log("system_reboot_infinite_loop ENTRY"));
    hooks.insert(FIRMWARE_BASE + 0x1B268, Hook::Log("epon_poll_hw_state_changes"));
    hooks.insert(FIRMWARE_BASE + 0x06680, Hook::Log("epon_rx_flag_clear_init"));
    hooks.insert(FIRMWARE_BASE + 0x1AE2C, Hook::Log("mpcp_slot_and_timing_init"));
    hooks.insert(FIRMWARE_BASE + 0x20FD4, Hook::Log("hw_config_load_and_reset_init"));
    hooks.insert(FIRMWARE_BASE + 0x3C4B4, Hook::Log("epon_llid_queue_table_init"));
    hooks.insert(FIRMWARE_BASE + 0x16014, Hook::Custom(firmware_cli_poll_hook));

    // `epon_poll_hw_state_changes` @ firmware+0x1B268 (prev Hook::ReturnImmediate)
    // was removed 2026-04-13. The function walks a 24-entry descriptor table
    // at DCCM 0x7ED90 (sysreg indices 0x0104..0x0A01) and compares current
    // values against a history buffer through `hw_state_apply_filter_mask`.
    // The filter short-circuits to 0 when `*(u32*)0x7E4F4 & 3 == 0`, and the
    // firmware .data initial value at that word is 0x05000000 — so the filter is
    // inactive at boot and no state change is ever detected. The residual
    // per-call call to `hw_state_check_link_up_and_trigger_reset` also early-
    // exits on `nop_return_ctx() != 0x381`. Safe to let it run natively.
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

    // ── Persistent alarm stubs (alm/info, alm/gpio) ─────────────────────
    //
    // KNOWN VIOLATION of the "no firmware function hooks" rule. Kept
    // until Phase 3 live-path modelling is complete. See
    // `the design notes` for the investigation notes
    // (chains traced through `mka_llid_state_machine_tick`,
    // `epon_llid_link_down_event`, `mpcp_update_link_events_and_bandwidth`,
    // and the tick-counter dependency on Timer 1 ISR).
    //
    // TL;DR of why this is hard: each of the 7 persistent alarm opcodes
    // (23, 28, 64, 131, 193, 199, 201) is pushed by a different firmware
    // state machine (MKA LLID, EPON RX, DPoE OAM, MACsec, …). Each state
    // machine depends on 3-5 layers of MMIO/DCCM state maintained by
    // other firmware code that runs in response to HW events we don't
    // model (PON RX-LOS detector, UNI PHY link status, MPCP frame error
    // counter, MACsec cipher state machine, …). Modelling one alarm
    // end-to-end is a multi-day investigation per opcode. Even with the
    // Timer 1 / tick-counter fix above (which unblocks the MKA LLID
    // state machine's 200-tick delay), the downstream cipher-init gate
    // in `mka_llid_check_and_init_cipher_state` requires DCCM state
    // (`DAT_ram_2000b630`, per-LLID FDS PHY byte, MKA cipher info) that
    // the emulator doesn't currently populate.
    hooks.insert(FIRMWARE_BASE + 0x33b2c, Hook::Custom(alm_info_pending_stub));
    hooks.insert(FIRMWARE_BASE + 0x10174, Hook::Custom(alm_gpio_chan_ctx_stub));
}

/// Replay any stdin bytes that arrived during the bootloader phase, on the
/// FIRST call to `cli_poll_and_process_input`. By that point firmware's BSS has
/// been cleared, the UART struct at 0x7E204 has been initialized, and the
/// CLI is ready to consume input. Replaying earlier (e.g. during the
/// bootloader → firmware handoff or in `cli_uart_init`) races with .data/BSS
/// init and either gets the bytes wiped or causes the firmware to read
/// uninitialized state and dump garbage.
///
/// The bootloader's UART ISR drains `mmio.uart.rx_queue` into its own
/// 0xF968 RX ring buffer and throws the bytes away when no CLI prompt is
/// active. The host-side stdin loop in main.rs holds a parallel copy in
/// `mmio.uart.held_pre_firmware` so we can recover them here.
fn firmware_cli_poll_hook(state: &mut CpuState, mem: &mut Memory) -> Result<HookAction, Exception> {
    crate::vlog!(
        "[Hook] cli_poll_and_process_input at PC=0x{:05X}, blink=0x{:05X}, insn=N/A",
        state.pc, state.core_regs[31]
    );
    if let Some(mut mmio) = mem.mmio() {
        if !mmio.uart.held_pre_firmware.is_empty() {
            let held: Vec<u8> = mmio.uart.held_pre_firmware.drain(..).collect();
            let n = held.len();
            for byte in held {
                mmio.uart.rx_queue.push_back(byte);
            }
            crate::vlog!(
                "[Hook] cli_poll: replayed {} pre-firmware stdin bytes into UART rx_queue",
                n
            );
        }
    }
    Ok(HookAction::Continue)
}

// ── Hook implementations: SerDes stubs ───────────────────────────────────
//
// These compensate for the missing SerDes register model. They check the
// bus type encoded in r0 high byte and short-circuit only for the bus
// type the SerDes block uses (which we have no peripheral for).

/// `irq_test_pending_bit_for_opcode(opcode, bit_idx) → 0/1` stub.
///
/// KNOWN VIOLATION of the "no firmware function hooks" rule. Returns 1
/// for the 7 persistent alarm opcodes on a quiescent ONU (bit 0 only).
/// See `the design notes` for why this is a stub
/// instead of a bottom-up state-machine model.
fn alm_info_pending_stub(state: &mut CpuState, _mem: &mut Memory) -> Result<HookAction, Exception> {
    let opcode = state.core_regs[0];
    let bit = state.core_regs[1];
    let pending = matches!(
        (opcode, bit),
        (23, 0) | (28, 0) | (64, 0) | (131, 0) | (193, 0) | (199, 0) | (201, 0)
    );
    state.core_regs[0] = if pending { 1 } else { 0 };
    state.pc = state.core_regs[31];
    state.instruction_count += 1;
    Ok(HookAction::Skip)
}

/// `chan_get_context_ptr_by_index(chan) → linked-list head pointer` stub.
///
/// KNOWN VIOLATION of the "no firmware function hooks" rule. Returns a
/// synthesized single-node linked list on channel 15, null elsewhere,
/// matching the `alm/gpio` level-0 capture `[[0,5]]`.
const ALM_GPIO_FAKE_NODE: u32 = 0x0007FFE0;
fn alm_gpio_chan_ctx_stub(state: &mut CpuState, mem: &mut Memory) -> Result<HookAction, Exception> {
    let chan = state.core_regs[0] & 0xFF;
    if chan == 15 {
        mem.write_word(ALM_GPIO_FAKE_NODE, 0)?;
        mem.write_word(ALM_GPIO_FAKE_NODE + 4, 0)?;
        mem.write_byte(ALM_GPIO_FAKE_NODE + 5, 5)?;
        mem.write_byte(ALM_GPIO_FAKE_NODE + 6, 0)?;
        state.core_regs[0] = ALM_GPIO_FAKE_NODE;
    } else {
        state.core_regs[0] = 0;
    }
    state.pc = state.core_regs[31];
    state.instruction_count += 1;
    Ok(HookAction::Skip)
}

