//! UI panel submodules. Each file owns one egui panel.

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

/// Bottom-left panel tab selection: UART terminal vs MCP activity
/// log. Phase 8 added the MCP log next to the existing UART.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BottomTab {
    Uart,
    McpLog,
}
