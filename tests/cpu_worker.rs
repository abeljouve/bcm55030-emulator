//! Phase 2 — CPU worker thread + snapshot bus.
//!
//! Verifies end-to-end: a caller spawns the CPU worker, sends
//! `CpuCommand::StepN`, polls the shared snapshot, and observes
//! `instruction_count` advancing as expected. A second case
//! validates the breakpoint → pause path.

use std::sync::mpsc;
use std::sync::Arc;
use std::thread;
use std::time::{Duration, Instant};

use parking_lot::{Mutex, RwLock};

use bcm55030_emulator::cpu::registers::PauseReason;
use bcm55030_emulator::cpu::Cpu;
use bcm55030_emulator::emu::command::{oneshot, CpuCommand};
use bcm55030_emulator::emu::{
    Annotations, EmulatorHandle, EmulatorSnapshot, EventLog, McpStatus, RunState,
};
use bcm55030_emulator::soc::bank::{BootMode, PeripheralBank};

const NOP_S: [u8; 2] = [0x78, 0xE0];

/// Load a flat-mode Cpu with `n` copies of NOP_S at address 0,
/// PC pre-seeded to 0.
fn flat_cpu_with_nops(n: usize) -> Cpu {
    let mut cpu = Cpu::new(64 * 1024);
    let mut blob = Vec::with_capacity(n * 2);
    for _ in 0..n {
        blob.extend_from_slice(&NOP_S);
    }
    cpu.mem.load_binary(0, &blob);
    cpu.state.pc = 0;
    cpu
}

/// Build an `EmulatorHandle` with all inner state freshly
/// allocated. The `bank` field is populated with a new
/// `PeripheralBank` even in flat-Cpu tests because the handle's
/// type demands one — the test never issues MMIO ops so the bank
/// stays untouched.
fn make_handle(cpu_cmd: mpsc::Sender<CpuCommand>) -> EmulatorHandle {
    let bank = Arc::new(RwLock::new(PeripheralBank::new(BootMode::Cold)));
    let (uart_tx, _uart_rx) = mpsc::channel::<u8>();
    EmulatorHandle {
        bank,
        snapshot: Arc::new(Mutex::new(EmulatorSnapshot::placeholder(BootMode::Cold))),
        cpu_cmd,
        uart_tx,
        annotations: Arc::new(RwLock::new(Annotations::new())),
        event_log: Arc::new(Mutex::new(EventLog::default())),
        mcp_status: Arc::new(Mutex::new(McpStatus::default())),
        firmware_info: Arc::new(Mutex::new(None)),
    }
}

/// Poll the shared snapshot until `predicate` returns true or
/// `timeout` elapses.
fn wait_until<F>(handle: &EmulatorHandle, timeout: Duration, predicate: F) -> bool
where
    F: Fn(&EmulatorSnapshot) -> bool,
{
    let start = Instant::now();
    while start.elapsed() < timeout {
        {
            let snap = handle.snapshot.lock();
            if predicate(&snap) {
                return true;
            }
        }
        thread::sleep(Duration::from_millis(5));
    }
    false
}

/// Round-trip: StepN(10) executes exactly 10 NOP_S instructions
/// and the worker publishes a snapshot with `instruction_count == 10`
/// and `run_state == Paused`.
#[test]
fn worker_executes_step_n() {
    let cpu = flat_cpu_with_nops(16);
    let (cmd_tx, cmd_rx) = mpsc::channel::<CpuCommand>();
    let handle = make_handle(cmd_tx.clone());

    let handle_for_worker = handle.clone();
    let worker = thread::spawn(move || {
        bcm55030_emulator::emu::cpu_worker::run(
            cpu,
            handle_for_worker,
            cmd_rx,
            Box::new(|_| Cpu::new(64 * 1024)),
        );
    });

    cmd_tx.send(CpuCommand::StepN(10)).expect("send StepN");

    let ok = wait_until(&handle, Duration::from_secs(2), |snap| {
        snap.cpu.instruction_count >= 10 && snap.run_state != RunState::Running
    });
    assert!(ok, "worker must finish StepN(10) within 2 s");

    {
        let snap = handle.snapshot.lock();
        assert_eq!(snap.cpu.instruction_count, 10);
        assert_eq!(snap.cpu.pc, 20, "10 NOP_S = 20 bytes");
        assert!(snap.cpu.paused);
        assert!(matches!(
            snap.run_state,
            RunState::Paused | RunState::Breakpoint
        ));
    }

    cmd_tx.send(CpuCommand::Shutdown).expect("send Shutdown");
    worker.join().expect("worker thread join");
}

/// Breakpoint at PC=8 pauses the CPU after exactly 4 NOP_S
/// instructions. `pause_reason` must be `Breakpoint(8)`.
#[test]
fn worker_pauses_on_breakpoint() {
    let cpu = flat_cpu_with_nops(16);
    let (cmd_tx, cmd_rx) = mpsc::channel::<CpuCommand>();
    let handle = make_handle(cmd_tx.clone());

    let handle_for_worker = handle.clone();
    let worker = thread::spawn(move || {
        bcm55030_emulator::emu::cpu_worker::run(
            cpu,
            handle_for_worker,
            cmd_rx,
            Box::new(|_| Cpu::new(64 * 1024)),
        );
    });

    // Install a breakpoint at PC = 8 (after 4 NOP_S).
    let (bp_tx, bp_rx) = oneshot::<usize>();
    cmd_tx
        .send(CpuCommand::SetBreakpoint {
            address: 8,
            response: bp_tx,
        })
        .expect("send SetBreakpoint");
    let _ = bp_rx.recv_timeout(Duration::from_secs(1)).expect("bp response");

    // Unbounded run — breakpoint must fire.
    cmd_tx
        .send(CpuCommand::Run { max_insns: None })
        .expect("send Run");

    let ok = wait_until(&handle, Duration::from_secs(2), |snap| {
        matches!(snap.pause_reason, PauseReason::Breakpoint(8))
    });
    assert!(ok, "worker must hit breakpoint within 2 s");

    {
        let snap = handle.snapshot.lock();
        assert_eq!(snap.cpu.pc, 8, "paused at the breakpoint PC");
        assert_eq!(snap.cpu.instruction_count, 4);
        assert_eq!(snap.pause_reason, PauseReason::Breakpoint(8));
        assert_eq!(snap.run_state, RunState::Breakpoint);
        assert!(snap.breakpoints.contains(&8));
    }

    cmd_tx.send(CpuCommand::Shutdown).expect("send Shutdown");
    worker.join().expect("worker thread join");
}

/// Pause interrupts a running CPU and reports `UserPause`.
#[test]
fn worker_pause_interrupts_run() {
    let cpu = flat_cpu_with_nops(16);
    let (cmd_tx, cmd_rx) = mpsc::channel::<CpuCommand>();
    let handle = make_handle(cmd_tx.clone());

    let handle_for_worker = handle.clone();
    let worker = thread::spawn(move || {
        bcm55030_emulator::emu::cpu_worker::run(
            cpu,
            handle_for_worker,
            cmd_rx,
            Box::new(|_| Cpu::new(64 * 1024)),
        );
    });

    cmd_tx
        .send(CpuCommand::Run { max_insns: Some(1_000_000) })
        .expect("send Run");
    thread::sleep(Duration::from_millis(20));
    cmd_tx.send(CpuCommand::Pause).expect("send Pause");

    let ok = wait_until(&handle, Duration::from_secs(1), |snap| {
        snap.run_state == RunState::Paused && snap.cpu.paused
    });
    assert!(ok, "worker must pause within 1 s");

    cmd_tx.send(CpuCommand::Shutdown).expect("send Shutdown");
    worker.join().expect("worker thread join");
}
