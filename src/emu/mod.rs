//! Cross-thread emulator plumbing for the UI and MCP server.
//!
//! The modules in `src/emu/` wrap the existing `Cpu` + `Memory` +
//! `PeripheralBank` stack in the state-sharing primitives the egui
//! GUI and the rmcp HTTP server both need:
//!
//! - `command` — `CpuCommand` enum + oneshot response wiring.
//! - `snapshot` — `EmulatorSnapshot` / `CpuSnapshot` / support types.
//! - `annotations` — user-loaded symbols / comments / regions.
//! - `event_log` — bounded MCP activity log.
//! - `handle` — `EmulatorHandle` clone-struct tying everything
//!   together.
//! - `cpu_worker` — worker-thread body that owns `Cpu` and drains
//!   the command channel.
//!
//! Everything in this module is plain data and lives in the
//! default build — no feature gate. Only the eframe and rmcp
//! integrations sitting on top are gated behind `ui` / `mcp`.

pub mod annotations;
pub mod command;
pub mod cpu_worker;
pub mod event_log;
pub mod handle;
#[cfg(feature = "ui")]
pub mod session;
pub mod snapshot;

pub use annotations::Annotations;
pub use command::{CpuCommand, FirmwareMode, LoadFirmwareResult, OneshotSender};
pub use event_log::{Direction, EventEntry, EventLog};
pub use handle::{EmulatorHandle, FirmwareInfo, McpStatus};
pub use snapshot::{
    AuxSnapshot, CpuSnapshot, DcacheSnapshot, EmulatorSnapshot, FlagsSnapshot, RunState,
    SramSnapshot,
};
