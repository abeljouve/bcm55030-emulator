//! egui/eframe GUI for the ARC700 emulator. Gated behind the
//! `ui` cargo feature.
//!
//! The module layout follows the phase-6 plan
//! (`the design plan`):
//!
//! - `app` — `EmulatorApp` struct + `impl eframe::App`.
//! - `theme` — colour tokens used across panels.
//! - `panels/*` — one file per dockable panel
//!   (toolbar, disassembly, registers, memory, uart_terminal,
//!   status_bar). Peripheral inspector, MCP activity log, and
//!   annotation dialogs land in phases 7 and 8.
//!
//! Entry point: `ui::run(handle)` — call this from `main.rs`
//! after the CPU worker + optional MCP server have been spawned.

pub mod app;
pub mod panels;
pub mod theme;

pub use app::{run, EmulatorApp};
