//! `EmulatorHandle` — clone-cheap struct of `Arc`s shared by the
//! UI thread, the MCP server thread, and the CPU worker thread.

use std::path::PathBuf;
use std::sync::mpsc::Sender;
use std::sync::Arc;

use parking_lot::{Mutex, RwLock};

use crate::emu::annotations::Annotations;
use crate::emu::command::{CpuCommand, FirmwareMode};
use crate::emu::event_log::EventLog;
use crate::emu::snapshot::EmulatorSnapshot;
use crate::soc::bank::{BootMode, PeripheralBank};

/// Description of the currently loaded firmware image. `None`
/// until `CpuCommand::LoadFirmware` lands.
#[derive(Clone, Debug)]
pub struct FirmwareInfo {
    pub path: PathBuf,
    pub mode: FirmwareMode,
    pub boot_mode: BootMode,
    pub entry_point: u32,
    pub flash_size: usize,
    pub flash_loaded: bool,
}

/// Live MCP server status — listening address + connected client
/// count. The MCP worker updates this; the UI status bar reads it.
#[derive(Clone, Debug, Default)]
pub struct McpStatus {
    pub listening: Option<String>,
    pub connected_clients: u32,
}

/// The handle tying all shared state together. Every thread holds
/// a clone; every field is either an `Arc<Mutex<T>>`, an
/// `Arc<RwLock<T>>`, or an `mpsc::Sender` — all cheap to clone.
#[derive(Clone)]
pub struct EmulatorHandle {
    /// Shared peripheral bank. Same `Arc` the `Cpu` holds, so
    /// CPU-side MMIO writes and UI-side `inject_event` calls
    /// observe the same state.
    pub bank: Arc<RwLock<PeripheralBank>>,

    /// Latest cheap snapshot published by the CPU worker.
    pub snapshot: Arc<Mutex<EmulatorSnapshot>>,

    /// CPU command channel. All run / pause / step / reset /
    /// breakpoint operations go through here.
    pub cpu_cmd: Sender<CpuCommand>,

    /// UART RX mpsc sender. Identical to the channel `main.rs`
    /// currently feeds from stdin — the UI and MCP clients push
    /// user-typed bytes through this path too.
    pub uart_tx: Sender<u8>,

    /// User-loaded symbols / comments / regions. Firmware-agnostic
    /// (the contributor guide).
    pub annotations: Arc<RwLock<Annotations>>,

    /// Bounded MCP activity log.
    pub event_log: Arc<Mutex<EventLog>>,

    /// Live MCP server status.
    pub mcp_status: Arc<Mutex<McpStatus>>,

    /// Description of the currently-loaded firmware, if any.
    pub firmware_info: Arc<Mutex<Option<FirmwareInfo>>>,
}
