//! Bottom status bar: run state, PC, instruction count,
//! instructions/sec, delay-slot / LP indicators, boot mode,
//! MCP server status.

use eframe::egui;

use crate::cpu::registers::DelayState;
use crate::emu::snapshot::RunState;
use crate::ui::theme;
use crate::ui::EmulatorApp;

pub fn draw(ui: &mut egui::Ui, app: &mut EmulatorApp) {
    ui.horizontal(|ui| {
        let run_label = match app.snapshot.run_state {
            RunState::Running => "Running",
            RunState::Paused => "Paused",
            RunState::Halted => "Halted",
            RunState::Sleeping => "Sleeping",
            RunState::Breakpoint => "Breakpoint",
        };
        ui.strong(run_label);
        ui.separator();
        ui.label(format!("PC 0x{:08X}", app.snapshot.cpu.pc));
        ui.separator();
        ui.label(format!("insn {}", app.snapshot.cpu.instruction_count));
        ui.separator();
        ui.label(format!("{} ins/s", app.snapshot.insns_per_sec));
        ui.separator();

        let delay_active = !matches!(app.snapshot.cpu.delay_state, DelayState::None);
        ui.colored_label(
            if delay_active {
                theme::DELAY_SLOT
            } else {
                theme::MUTED
            },
            "DS",
        );
        let lp_active = app.snapshot.cpu.aux.lp_start != 0
            && app.snapshot.cpu.pc >= app.snapshot.cpu.aux.lp_start
            && app.snapshot.cpu.pc <= app.snapshot.cpu.aux.lp_end;
        ui.colored_label(
            if lp_active {
                theme::LP_RANGE
            } else {
                theme::MUTED
            },
            "LP",
        );

        ui.separator();
        ui.label(format!("{:?}", app.snapshot.boot_mode));

        ui.separator();
        let mcp_label = match app.handle.mcp_status.lock().listening.as_deref() {
            Some(addr) => format!("MCP {}", addr),
            None => "MCP off".to_string(),
        };
        ui.label(mcp_label);
    });
}
