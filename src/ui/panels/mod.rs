//! UI panel submodules. Each file owns one egui panel.

pub mod debug_panel;
pub mod disassembly;
pub mod mcp_log;
pub mod memory;
pub mod packets;
pub mod peripherals;
pub mod registers;
pub mod status_bar;
pub mod strings;
pub mod toolbar;
pub mod uart_terminal;

/// Central pane tab selection: memory viewer, peripheral
/// inspector, or strings extractor.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CentralTab {
    Memory,
    Peripherals,
    Strings,
    /// Frames exchanged with the EPON peer.
    Packets,
}

/// Bottom panel tab selection: UART terminal, MCP activity log,
/// or the Debug panel (breakpoints + watchpoints + call stack).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BottomTab {
    Uart,
    McpLog,
    Debug,
}
