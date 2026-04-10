/// BCM55030 SoC emulation.
///
/// This module contains all BCM55030-specific behavior: boot ROM intercepts,
/// SerDes register stubs, init milestones, and peripheral workarounds.
/// The core ARC700 CPU emulation (decoder, executor, memory) has no knowledge
/// of the BCM55030 — all SoC-specific behavior is injected via hooks.

pub mod boot_rom;
pub mod mmio;
pub mod pbc;
pub mod sfp_eeprom;
pub mod spi_flash;
pub mod uart;

use crate::cpu::exception::Exception;
use crate::cpu::registers::CpuState;
use crate::cpu::registers::REG_ILINK1;
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

    // ── Missing-hardware stubs (NOT firmware bypasses) ──────────────────
    //
    // These hooks compensate for hardware blocks we don't model at all
    // (SerDes lanes). Without them, the firmware enters retry loops on
    // SerDes register I/O during `serdes_init_all_lanes_hw`. They are NOT
    // bypassing buggy firmware logic — they're providing a synthetic
    // "ready/empty" answer for a hardware block that's silent in our
    // emulator. To remove them, we'd need a real SerDes register model
    // (~50 registers per lane × 4 lanes + PLL/common); see Phase 3 of the
    // roadmap.
    //
    // Re-validated 2026-04-10: tested without these stubs → firmware stops in
    // `serdes_init_all_lanes_hw` at insn 11.7M. With them → firmware reaches
    // `cli_poll_and_process_input`.

    // serdes_reg_read_byte (file 0x12CA4) — return 0xFF for non-SPI bus types.
    hooks.insert(FIRMWARE_BASE + 0x12CA4, Hook::Custom(serdes_reg_read_byte_stub));

    // serdes_reg_write (file 0x12CD8) — skip for non-SPI bus types.
    hooks.insert(FIRMWARE_BASE + 0x12CD8, Hook::Custom(serdes_reg_write_stub));

    // serdes_hw_ready_flag (file 0x1E6C) — always return 1 (ready).
    hooks.insert(FIRMWARE_BASE + 0x1E6C, Hook::ReturnValue(1));

    // ── Workaround removed for re-validation 2026-04-10 ──────────────────
    //
    // event_dispatch_stub @ FIRMWARE_BASE+0x33A50 was the "+4 trampoline" hack
    // we believed was needed because handler table addresses were stored
    // 4 bytes past function entries. With the corrected load offset
    // (FIRMWARE_BASE = 0x32000), this might no longer be needed because the
    // handler addresses might now resolve correctly. To be re-validated.

    // ── Firmware UART ISR (synthetic) ────────────────────────────────────────
    //
    // Firmware's `uart_putchar` enqueues bytes into a TX ring buffer at DCCM
    // 0x348 (struct at 0x7E204) and sets bit 0x40 in the UART IER. The
    // firmware expects an ISR to drain that buffer, but our IRQ 5 vector
    // dispatch lands at the bootloader's IVT (since AUX_INT_VECTOR_BASE
    // stays at 0), and the bootloader's ISR drains a DIFFERENT ring
    // buffer. Result: firmware's TX buffer fills up and `uart_putchar` loops
    // forever in `serdes_tx_queue_enqueue`.
    //
    // The IVT entry at runtime 0x320A8 contains 8 bytes that we don't yet
    // decode/understand (`26 4a 70 00 26 4a 70 00`). Until we figure out
    // the real mechanism (Phase 2 — could be polling, AUX vector base
    // shift, or a custom ISR encoding), install a synthetic ISR at the
    // bootloader's IVT entry 21 (PC 0xA8) that does the right thing for
    // firmware only (gated on `mem.app_size.is_some()`).
    //
    // This is NOT a firmware function bypass — there's no firmware code
    // at this address (it's a hardware vector slot). It's filling in a
    // missing peripheral driver.
    hooks.insert(0xA8, Hook::Custom(firmware_uart_isr));

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

    // ── Stub: epon_poll_hw_state_changes ────────────────────────────────
    //
    // This function (file 0x1B268) walks a 24-entry table at .data 0x7ED90
    // and reads sysreg[idx] for each entry, comparing against a stored
    // previous value. On any (current != 0 && current != prev) it calls
    // log_printf to dump a "state change" message — and on certain bit
    // transitions it triggers `system_shutdown_and_flush`.
    //
    // Real HW returns 0 for ALL these registers on a quiescent ONU. Our
    // store-and-return default leaves residual values from prior writes
    // (e.g. 0xFFFF0 mask clears), which the firmware misreads as state
    // changes — dumping the entire .rodata string region to UART or
    // entering the shutdown path.
    //
    // We can't blindly return 0 for these specific sysreg offsets because
    // some of them are also used as configuration/state registers by other
    // firmware code (returning 0 there breaks init). The cleanest fix is
    // to stub the polling function itself to a no-op return — until we
    // have real HW models for the SerDes/MPCP/EPON status registers.
    hooks.insert(FIRMWARE_BASE + 0x1B268, Hook::ReturnImmediate);
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

/// Synthetic UART ISR for firmware (IRQ 5).
///
/// Drains firmware's TX ring buffer at DCCM 0x348 to the UART data register.
/// Reads stdin into firmware's RX ring buffer at DCCM 0x248. Performs RTIE.
///
/// Triggered by the IRQ 5 vector at PC 0xA8. Only fires when:
///   1. Firmware is loaded (mem.app_size is Some)
///   2. The CPU is in interrupt context (flag_a1 set, set by check_interrupts)
///
/// Otherwise (bootloader running, or normal code execution at PC 0xA8),
/// returns Continue and lets the bootloader's J 0x4348 ISR run.
///
/// Firmware UART struct layout at DCCM 0x7E204:
///   +0: rx_empty flag (0 = has data)
///   +1: rx_write index
///   +2: rx_read  index
///   +3: tx_trigger flag (set by ISR after drain to re-arm next batch)
///   +4: tx_write index
///   +5: tx_read  index
///
/// TX buffer at DCCM 0x348, RX buffer at DCCM 0x248 (256 bytes each).
fn firmware_uart_isr(state: &mut CpuState, mem: &mut Memory) -> Result<HookAction, Exception> {
    // Only intercept when firmware is loaded
    if mem.app_size.is_none() {
        return Ok(HookAction::Continue);
    }

    // Only intercept on actual IRQ entry (not when normal code happens to PC=0xA8)
    if !state.flag_a1 && !state.flag_a2 {
        return Ok(HookAction::Continue);
    }

    const UART_STRUCT: u32 = 0x7E204;
    const TX_BUF: u32 = 0x348;
    const RX_BUF: u32 = 0x248;

    // Phase 1: snapshot UART HW state and drain incoming RX queue
    let (ier, rx_bytes) = {
        let mut mmio = mem.mmio().unwrap();
        let ier = mmio.uart.ier();
        let rx: Vec<u8> = mmio.uart.rx_queue.drain(..).collect();
        (ier, rx)
    };

    // Phase 2: TX — drain firmware's TX ring buffer to stdout (via UART HW)
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

        // Buffer drained: clear TXIE and re-arm trigger for next batch
        {
            let mut mmio = mem.mmio().unwrap();
            mmio.uart.ier_clear(0x40);
        }
        mem.write_byte(UART_STRUCT + 3, 1)?;
    }

    // Phase 3: RX — fill firmware's RX ring buffer from stdin
    if !rx_bytes.is_empty() {
        let mut pushed = 0usize;
        for &byte in &rx_bytes {
            let rx_write = mem.read_byte(UART_STRUCT + 1)?;
            let rx_read = mem.read_byte(UART_STRUCT + 2)?;

            if rx_write.wrapping_add(1) == rx_read {
                break; // Ring buffer full
            }

            mem.write_byte(RX_BUF + rx_write as u32, byte)?;
            mem.write_byte(UART_STRUCT + 1, rx_write.wrapping_add(1))?;
            mem.write_byte(UART_STRUCT + 0, 0)?;
            pushed += 1;
        }

        if pushed < rx_bytes.len() {
            let mut mmio = mem.mmio().unwrap();
            for &byte in &rx_bytes[pushed..] {
                mmio.uart.rx_queue.push_back(byte);
            }
        }
    }

    // Ensure RXIE (bit 2) is set so future stdin data triggers UART IRQ
    {
        let mut mmio = mem.mmio().unwrap();
        mmio.uart.ier_set(0x04);
    }

    // RTIE: restore STATUS32 and PC from saved interrupt state.
    // For level-1 IRQ, also restore r0..r3 from the fast-IRQ shadow set
    // saved by check_interrupts. The bootloader's IRQ handler at 0xA800
    // (which we may bypass via this hook) freely clobbers r0..r3 — real
    // HW shadow registers protect the firmware's GP state.
    if state.flag_a2 {
        let saved = state.aux_status32_l2;
        state.set_status32(saved);
        state.pc = state.core_regs[crate::cpu::registers::REG_ILINK2 as usize];
        state.aux_bta = state.aux_bta_l2;
    } else {
        let saved = state.aux_status32_l1;
        state.set_status32(saved);
        state.pc = state.core_regs[REG_ILINK1 as usize];
        state.aux_bta = state.aux_bta_l1;
        state.core_regs[0] = state.irq_shadow_r0_r3[0];
        state.core_regs[1] = state.irq_shadow_r0_r3[1];
        state.core_regs[2] = state.irq_shadow_r0_r3[2];
        state.core_regs[3] = state.irq_shadow_r0_r3[3];
    }
    state.pc_written = true;
    state.instruction_count += 1;
    Ok(HookAction::Skip)
}

/// Replay any stdin bytes that arrived during the bootloader phase, on the
/// FIRST call to `cli_poll_and_process_input`. By that point firmware's BSS has
/// been cleared, the UART struct at 0x7E204 has been initialized, and the
/// CLI is ready to consume input. Replaying earlier (e.g. in
/// `boot_rom_start_app` or in `cli_uart_init`) races with .data/BSS init and
/// either gets the bytes wiped or causes the firmware to read uninitialized
/// state and dump garbage.
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

/// `serdes_reg_read_byte` stub: returns 0xFF for the missing-HW bus type.
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

/// `serdes_reg_write` stub: silently swallows writes for the missing-HW bus type.
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
