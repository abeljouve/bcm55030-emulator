//! BCM55030 SoC emulation — MMIO peripherals + firmware init-milestone log hooks.

pub mod alarm;
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

/// Register BCM55030 hooks. Firmware addresses = FIRMWARE_BASE + (the decompiler_addr - 0x20000000).
pub fn register_hooks(hooks: &mut HookTable) {
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
    hooks.insert(FIRMWARE_BASE + 0x084FC, Hook::Log("firmware_update_trigger"));
    hooks.insert(FIRMWARE_BASE + 0x33CF8, Hook::Log("system_reboot_infinite_loop ENTRY"));
    hooks.insert(FIRMWARE_BASE + 0x1B268, Hook::Log("epon_poll_hw_state_changes"));
    hooks.insert(FIRMWARE_BASE + 0x06680, Hook::Log("epon_rx_flag_clear_init"));
    hooks.insert(FIRMWARE_BASE + 0x1AE2C, Hook::Log("mpcp_slot_and_timing_init"));
    hooks.insert(FIRMWARE_BASE + 0x20FD4, Hook::Log("hw_config_load_and_reset_init"));
    hooks.insert(FIRMWARE_BASE + 0x3C4B4, Hook::Log("epon_llid_queue_table_init"));
    hooks.insert(FIRMWARE_BASE + 0x16014, Hook::Custom(firmware_cli_poll_hook));
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

/// Replay pre-firmware stdin held in `uart.held_pre_firmware` once firmware's CLI poll runs.
/// Earlier replay races with BSS init.
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


