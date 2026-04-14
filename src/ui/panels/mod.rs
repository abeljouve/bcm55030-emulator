//! UI panel submodules. Each file owns one egui panel.

pub mod debug_panel;
pub mod disassembly;
pub mod mcp_log;
pub mod memory;
pub mod peripherals;
pub mod registers;
pub mod status_bar;
pub mod toolbar;
pub mod uart_terminal;

/// Central pane tab selection: memory viewer vs peripheral
/// inspector.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CentralTab {
    Memory,
    Peripherals,
}

/// Bottom panel tab selection: UART terminal, MCP activity log,
/// or the Debug panel (breakpoints + watchpoints + call stack).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BottomTab {
    Uart,
    McpLog,
    Debug,
}
