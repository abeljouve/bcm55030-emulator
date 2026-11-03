//! A named snapshot taken while the D-cache holds dirty lines must restore
//! the machine as it was: the snapshot's dirty data comes back, and the dirty
//! lines current at restore time are not written over the restored SRAM.

use std::sync::mpsc;
use std::sync::Arc;
use std::thread;
use std::time::{Duration, Instant};

use parking_lot::{Mutex, RwLock};

use bcm55030_emulator::cpu::Cpu;
use bcm55030_emulator::emu::command::{oneshot, CpuCommand};
use bcm55030_emulator::emu::{
    Annotations, EmulatorHandle, EmulatorSnapshot, EventLog, McpStatus, RunState,
};
use bcm55030_emulator::soc::bank::BootMode;

const X: u32 = 0x4000;
const A: u32 = 0xAAAA_0001;
const B: u32 = 0xBBBB_0001;

/// `st r1,[r0]`, `ld r3,[r0]`, `st r2,[r0]`, `ld r3,[r0]`.
const ST_R1_R0: u32 = 0x1800_0040;
const ST_R2_R0: u32 = 0x1800_0080;
const LD_R3_R0: u32 = 0x1000_0003;

fn handle_for(cpu: &Cpu, cpu_cmd: mpsc::Sender<CpuCommand>) -> EmulatorHandle {
    let (uart_tx, _uart_rx) = mpsc::channel::<u8>();
    EmulatorHandle {
        bank: Arc::clone(cpu.bank().expect("SoC mode")),
        snapshot: Arc::new(Mutex::new(EmulatorSnapshot::placeholder(BootMode::Warm))),
        cpu_cmd,
        uart_tx,
        annotations: Arc::new(RwLock::new(Annotations::new())),
        event_log: Arc::new(Mutex::new(EventLog::default())),
        mcp_status: Arc::new(Mutex::new(McpStatus::default())),
        firmware_info: Arc::new(Mutex::new(None)),
    }
}

fn step_and_wait(tx: &mpsc::Sender<CpuCommand>, h: &EmulatorHandle, n: u32) {
    let target = h.snapshot.lock().cpu.instruction_count + u64::from(n);
    tx.send(CpuCommand::StepN(n)).unwrap();
    let start = Instant::now();
    while start.elapsed() < Duration::from_secs(5) {
        {
            let s = h.snapshot.lock();
            if s.cpu.instruction_count >= target && s.run_state != RunState::Running {
                return;
            }
        }
        thread::sleep(Duration::from_millis(2));
    }
    panic!("StepN({n}) did not finish");
}

#[test]
fn restore_brings_back_the_snapshot_dirty_lines() {
    let mut cpu = Cpu::new_bcm55030(BootMode::Warm);
    let mut code = Vec::new();
    for w in [ST_R1_R0, LD_R3_R0, ST_R2_R0, LD_R3_R0] {
        code.extend_from_slice(&w.to_be_bytes());
    }
    cpu.mem.load_binary(0, &code);
    cpu.state.pc = 0;
    cpu.state.core_regs[0] = X;
    cpu.state.core_regs[1] = A;
    cpu.state.core_regs[2] = B;

    let (tx, rx) = mpsc::channel::<CpuCommand>();
    let h = handle_for(&cpu, tx.clone());
    let hw = h.clone();
    let worker = thread::spawn(move || {
        bcm55030_emulator::emu::cpu_worker::run(cpu, hw, rx, Box::new(|_| Cpu::new(64 * 1024)));
    });

    step_and_wait(&tx, &h, 1); // st A: a dirty line, SRAM still 0
    let (r, w) = oneshot();
    tx.send(CpuCommand::SaveSnapshot { name: "s".into(), response: r }).unwrap();
    w.recv_timeout(Duration::from_secs(5)).unwrap().unwrap();

    step_and_wait(&tx, &h, 3); // ld A, st B (same line, dirty), ld B
    assert_eq!(h.snapshot.lock().cpu.core_regs[3], B);

    let (r, w) = oneshot();
    tx.send(CpuCommand::RestoreSnapshot { name: "s".into(), response: r }).unwrap();
    w.recv_timeout(Duration::from_secs(5)).unwrap().unwrap();

    step_and_wait(&tx, &h, 1); // ld r3,[r0] at the snapshot PC
    assert_eq!(
        h.snapshot.lock().cpu.core_regs[3],
        A,
        "the value stored before the snapshot, not the later dirty line"
    );

    tx.send(CpuCommand::Shutdown).unwrap();
    worker.join().unwrap();
}
