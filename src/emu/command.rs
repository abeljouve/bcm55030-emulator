//! `CpuCommand` — messages the UI / MCP threads send to the CPU
//! worker thread, plus a minimal oneshot wrapper over `std::sync::
//! mpsc::sync_channel(1)` for the response side.

use std::path::PathBuf;
use std::sync::mpsc::{self, RecvError, RecvTimeoutError, SendError, SyncSender};
use std::time::Duration;

use crate::emu::snapshot::{DcacheSnapshot, EmulatorSnapshot, SramSnapshot};
use crate::memory::WatchMode;
use crate::soc::bank::BootMode;

/// Single-use sender for a command response. Callers consume it
/// by value so the sender is dropped after `send`; the receiver
/// unblocks on either `send` or drop.
#[derive(Debug)]
pub struct OneshotSender<T>(SyncSender<T>);

/// Single-use receiver paired with `OneshotSender`.
#[derive(Debug)]
pub struct OneshotReceiver<T>(mpsc::Receiver<T>);

/// Build a fresh one-slot channel. The capacity is 1 so the
/// worker thread never blocks when writing its response — the
/// caller has always already parked in `recv`.
pub fn oneshot<T>() -> (OneshotSender<T>, OneshotReceiver<T>) {
    let (tx, rx) = mpsc::sync_channel(1);
    (OneshotSender(tx), OneshotReceiver(rx))
}

impl<T> OneshotSender<T> {
    pub fn send(self, val: T) -> Result<(), SendError<T>> {
        self.0.send(val)
    }
}

impl<T> OneshotReceiver<T> {
    pub fn recv(self) -> Result<T, RecvError> {
        self.0.recv()
    }

    pub fn recv_timeout(self, timeout: Duration) -> Result<T, RecvTimeoutError> {
        self.0.recv_timeout(timeout)
    }
}

/// Execution-speed cap applied by the CPU worker. `Unlimited`
/// runs flat out (the default, same throughput as the headless
/// CLI); `Ips(n)` caps the worker to roughly `n` instructions
/// per wall-clock second via a sleep/step budget per 10 ms
/// window.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SpeedLimit {
    Unlimited,
    Ips(u32),
}

impl SpeedLimit {
    pub fn as_ips(self) -> Option<u32> {
        match self {
            SpeedLimit::Unlimited => None,
            SpeedLimit::Ips(n) => Some(n),
        }
    }
}

impl Default for SpeedLimit {
    fn default() -> Self {
        SpeedLimit::Unlimited
    }
}

/// Firmware loading mode passed to `CpuCommand::LoadFirmware`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FirmwareMode {
    /// Flat: raw binary at address 0, no peripheral bank. Used by
    /// `tests/integration_tests.rs`.
    Flat,
    /// SoC: BCM55030 unified 512 KB SRAM, peripheral bank wired,
    /// hardware DMA copies the first 64 KB of the flash image.
    Soc,
}

/// Payload returned by a successful `LoadFirmware`.
#[derive(Clone, Debug)]
pub struct LoadFirmwareResult {
    pub loaded_bytes: usize,
    pub entry_point: u32,
    pub flash_bytes: usize,
}

/// Commands the worker thread accepts. Everything the UI and MCP
/// do at the CPU level flows through this enum.
#[derive(Debug)]
pub enum CpuCommand {
    Run {
        max_insns: Option<u64>,
    },
    RunTo {
        address: u32,
    },
    Pause,
    StepOne,
    StepN(u32),
    StepOver,
    Reset {
        boot_mode: BootMode,
        keep_breakpoints: bool,
    },
    LoadFirmware {
        path: PathBuf,
        mode: FirmwareMode,
        boot_mode: BootMode,
        flash_path: Option<PathBuf>,
        entry_point: u32,
        keep_breakpoints: bool,
        response: OneshotSender<Result<LoadFirmwareResult, String>>,
    },
    SetBreakpoint {
        address: u32,
        response: OneshotSender<usize>,
    },
    RemoveBreakpoint {
        address: u32,
    },
    SetWatchpoint {
        addr: u32,
        size: u32,
        mode: WatchMode,
        response: OneshotSender<usize>,
    },
    RemoveWatchpoint {
        index: usize,
    },
    WriteRegister {
        name: String,
        value: u32,
        response: OneshotSender<Result<(), String>>,
    },
    WriteSram {
        addr: u32,
        bytes: Vec<u8>,
        response: OneshotSender<Result<(), String>>,
    },
    RequestSram {
        response: OneshotSender<SramSnapshot>,
    },
    RequestDcache {
        response: OneshotSender<DcacheSnapshot>,
    },
    Snapshot {
        response: OneshotSender<EmulatorSnapshot>,
    },
    /// Apply a new execution-speed cap to the worker. Takes
    /// effect on the next `step()` — the worker recomputes its
    /// budget window against wall clock.
    SetSpeed {
        limit: SpeedLimit,
    },
    /// Shutdown signal — the worker drops its `Cpu`, sends a last
    /// snapshot (run_state=Halted) and exits.
    Shutdown,
}
