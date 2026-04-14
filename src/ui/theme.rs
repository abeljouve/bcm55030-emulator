//! Colour tokens shared across panels. Keeping them in one file
//! makes it easy to tune a global colour palette later.

use egui::Color32;

/// Breakpoint marker in the disassembly gutter.
pub const BREAKPOINT: Color32 = Color32::from_rgb(220, 50, 50);

/// Current-PC highlight row.
pub const PC_HIGHLIGHT: Color32 = Color32::from_rgb(60, 60, 30);

/// Delay-slot instruction row.
pub const DELAY_SLOT: Color32 = Color32::from_rgb(90, 30, 90);

/// Zero-overhead loop range.
pub const LP_RANGE: Color32 = Color32::from_rgb(20, 60, 30);

/// Recently-changed register cell (orange fade).
pub const CHANGED_REG: Color32 = Color32::from_rgb(255, 140, 0);

/// Stack region highlight in the memory viewer.
pub const STACK: Color32 = Color32::from_rgb(30, 60, 90);

/// UART terminal foreground / background.
pub const TERMINAL_FG: Color32 = Color32::from_rgb(0, 220, 60);
pub const TERMINAL_BG: Color32 = Color32::from_rgb(0, 0, 0);

/// Mutation-row colour in the MCP activity log.
pub const MUTATION: Color32 = Color32::from_rgb(240, 170, 60);

/// Muted label / secondary text.
pub const MUTED: Color32 = Color32::from_rgb(160, 160, 160);
