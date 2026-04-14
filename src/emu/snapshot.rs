//! Cross-thread snapshot types published by the CPU worker and
//! consumed by the UI (60 Hz) and MCP (on-demand) threads.
//!
//! `EmulatorSnapshot` is the cheap part: CPU registers, flags,
//! peripheral display state — small enough to clone every publish
//! tick. SRAM (512 KB) and the D-cache dump (4 KB) are **not** in
//! the frame snapshot; the worker answers explicit
//! `CpuCommand::RequestSram` / `RequestDcache` on demand.

use std::time::Instant;

use crate::cache::DCacheLineInfo;
use crate::cpu::registers::{CpuState, DelayState, PauseReason};
use crate::memory::Watchpoint;
use crate::soc::bank::BootMode;
use crate::soc::peripheral::PeripheralSnapshot;

/// Headline run state for the status bar. Drives the spinner /
/// stop-sign indicator and the "Running / Paused / Halted" label.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RunState {
    Running,
    Paused,
    Halted,
    Sleeping,
    /// Sub-state of Paused — pause was triggered by a breakpoint
    /// hook at the current PC. The UI uses this to highlight the
    /// disassembly gutter marker.
    Breakpoint,
}

/// STATUS32 flag decomposition. Mirrors `CpuState::flag_*` fields,
/// plus the raw packed value for convenience.
#[derive(Clone, Copy, Debug, Default)]
pub struct FlagsSnapshot {
    pub z: bool,
    pub n: bool,
    pub c: bool,
    pub v: bool,
    pub e1: bool,
    pub e2: bool,
    pub a1: bool,
    pub a2: bool,
    pub ae: bool,
    pub de: bool,
    pub u: bool,
    pub l: bool,
    pub h: bool,
    pub status32: u32,
}

/// Snapshot of the auxiliary-register surface the UI renders on the
/// "Aux" sub-tab. Pure data — no behaviour.
#[derive(Clone, Copy, Debug, Default)]
pub struct AuxSnapshot {
    /// BCM55030 IDENTITY = `0x00B40124` (ARCVER 0x24). Hard-coded
    /// here so the UI does not need to peek an aux register.
    pub identity: u32,
    pub lp_start: u32,
    pub lp_end: u32,
    pub int_vector_base: u32,
    pub ienable: u32,
    pub ipending: u32,
    /// Timer 0: COUNT, CONTROL, LIMIT (IRQ 3).
    pub count0: u32,
    pub control0: u32,
    pub limit0: u32,
    /// Timer 1: COUNT, CONTROL, LIMIT. BCM55030 wires this to IRQ 7
    /// instead of the ARC 700 default IRQ 4 — `timer1_irq` carries
    /// the live line number.
    pub count1: u32,
    pub control1: u32,
    pub limit1: u32,
    pub timer1_irq: u32,
    pub dc_ctrl: u32,
}

/// CPU-side cheap state. Published by the worker after every
/// publish trigger.
#[derive(Clone, Debug)]
pub struct CpuSnapshot {
    pub core_regs: [u32; 64],
    pub pc: u32,
    pub flags: FlagsSnapshot,
    pub aux: AuxSnapshot,
    pub delay_state: DelayState,
    pub halted: bool,
    pub sleeping: bool,
    pub paused: bool,
    pub instruction_count: u64,
}

impl CpuSnapshot {
    /// Build from a live `CpuState`. Called under the CPU worker's
    /// publish path.
    pub fn from_state(state: &CpuState) -> Self {
        let aux = AuxSnapshot {
            identity: 0x00B40124,
            lp_start: state.aux_lp_start,
            lp_end: state.aux_lp_end,
            int_vector_base: state.aux_int_vector_base,
            ienable: state.aux_ienable,
            ipending: state.aux_irq_pending,
            count0: state.aux_count0,
            control0: state.aux_control0,
            limit0: state.aux_limit0,
            count1: state.aux_count1,
            control1: state.aux_control1,
            limit1: state.aux_limit1,
            timer1_irq: state.timer1_irq,
            dc_ctrl: state.aux_dc_ctrl,
        };
        let flags = FlagsSnapshot {
            z: state.flag_z,
            n: state.flag_n,
            c: state.flag_c,
            v: state.flag_v,
            e1: state.flag_e1,
            e2: state.flag_e2,
            a1: state.flag_a1,
            a2: state.flag_a2,
            ae: state.flag_ae,
            de: state.flag_de,
            u: state.flag_u,
            l: state.flag_l,
            h: state.flag_h,
            status32: state.status32(),
        };
        Self {
            core_regs: state.core_regs,
            pc: state.pc,
            flags,
            aux,
            delay_state: state.delay_state,
            halted: state.halted,
            sleeping: state.sleeping,
            paused: state.paused,
            instruction_count: state.instruction_count,
        }
    }
}

/// Per-frame snapshot bundle. Reasonably cheap to clone — core
/// regs + flags + aux + a small `Vec<PeripheralSnapshot>` and a
/// short breakpoints list.
#[derive(Clone, Debug)]
pub struct EmulatorSnapshot {
    pub cpu: CpuSnapshot,
    pub peripherals: Vec<PeripheralSnapshot>,
    pub run_state: RunState,
    pub boot_mode: BootMode,
    pub bank_tick_accumulator: u64,
    pub insns_per_sec: u32,
    pub breakpoints: Vec<u32>,
    pub watchpoints: Vec<Watchpoint>,
    pub pause_reason: PauseReason,
    /// Wall-clock time the CPU worker published this snapshot.
    /// Used by the UI to detect stale vs fresh snapshots (e.g.
    /// to gate register-change highlighting).
    pub timestamp: Instant,
}

impl EmulatorSnapshot {
    /// Initial placeholder before the worker publishes its first
    /// real snapshot. `run_state = Paused`, zero counters,
    /// everything else default.
    pub fn placeholder(boot_mode: BootMode) -> Self {
        Self {
            cpu: CpuSnapshot::from_state(&CpuState::new()),
            peripherals: Vec::new(),
            run_state: RunState::Paused,
            boot_mode,
            bank_tick_accumulator: 0,
            insns_per_sec: 0,
            breakpoints: Vec::new(),
            watchpoints: Vec::new(),
            pause_reason: PauseReason::None,
            timestamp: Instant::now(),
        }
    }
}

/// Large-payload snapshots fetched on demand — not embedded in
/// `EmulatorSnapshot` to avoid cloning 512 KB every 16 ms.

#[derive(Clone, Debug)]
pub struct SramSnapshot {
    pub bytes: Vec<u8>,
    pub timestamp: Instant,
}

#[derive(Clone, Debug)]
pub struct DcacheSnapshot {
    pub lines: Vec<DCacheLineInfo>,
    pub ctrl_raw: u32,
    pub timestamp: Instant,
}
