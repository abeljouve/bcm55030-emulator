//! Disassembly panel: decode instructions from the on-demand
//! SRAM snapshot, render them in a scrolling table, highlight
//! the current PC row, and expose gutter-click breakpoint
//! toggling.

use eframe::egui;

use crate::decoder;
use crate::decoder::format::{format_line, FormattedLine};
use crate::emu::command::CpuCommand;
use crate::ui::theme;
use crate::ui::EmulatorApp;

const ROWS_ABOVE_PC: u32 = 12;
const ROWS_TOTAL: u32 = 48;

pub fn draw(ui: &mut egui::Ui, app: &mut EmulatorApp) {
    ui.horizontal(|ui| {
        if ui.button("Center on PC").clicked() {
            app.disasm_cursor = app.snapshot.cpu.pc;
        }
        ui.label(format!("@ 0x{:08X}", app.disasm_cursor));
    });
    ui.separator();

    let Some(sram) = app.sram.as_ref() else {
        ui.colored_label(theme::MUTED, "Waiting for SRAM snapshot…");
        return;
    };

    let base_pc = app
        .snapshot
        .cpu
        .pc
        .saturating_sub(ROWS_ABOVE_PC * 4);

    let lines = decode_window(&sram.bytes, base_pc, ROWS_TOTAL);

    egui::ScrollArea::vertical()
        .id_salt("disasm_scroll")
        .max_height(ui.available_height())
        .show(ui, |ui| {
            for line in &lines {
                let row_bg = if line.address == app.snapshot.cpu.pc {
                    Some(theme::PC_HIGHLIGHT)
                } else if line.is_delay_slot_carrier {
                    Some(theme::DELAY_SLOT)
                } else if line.address >= app.snapshot.cpu.aux.lp_start
                    && line.address <= app.snapshot.cpu.aux.lp_end
                    && app.snapshot.cpu.aux.lp_start != 0
                {
                    Some(theme::LP_RANGE)
                } else {
                    None
                };
                let mut frame = egui::Frame::default();
                if let Some(bg) = row_bg {
                    frame = frame.fill(bg);
                }
                frame.show(ui, |ui| {
                    ui.horizontal(|ui| {
                        // Breakpoint gutter — clickable circle.
                        let has_bp = app.snapshot.breakpoints.contains(&line.address);
                        let gutter_label = if has_bp { "●" } else { "○" };
                        let color = if has_bp { theme::BREAKPOINT } else { theme::MUTED };
                        if ui
                            .colored_label(color, gutter_label)
                            .interact(egui::Sense::click())
                            .clicked()
                        {
                            toggle_breakpoint(app, line.address, has_bp);
                        }

                        ui.monospace(format!("0x{:08X}", line.address));
                        ui.monospace(format!("{:<9}", line.hex_bytes));
                        let mnemonic = egui::RichText::new(format!("{:<8}", line.mnemonic))
                            .monospace();
                        ui.label(mnemonic);
                        ui.monospace(&line.operands);
                    });
                });
                let resp = ui.interact(
                    ui.min_rect(),
                    egui::Id::new(("disasm_row", line.address)),
                    egui::Sense::click(),
                );
                if resp.clicked() {
                    app.disasm_cursor = line.address;
                }
            }
        });
}

fn decode_window(bytes: &[u8], base: u32, rows: u32) -> Vec<FormattedLine> {
    let mut out = Vec::with_capacity(rows as usize);
    let mut pc = base;
    let mut i = 0u32;
    while i < rows {
        let Ok(dec) = decoder::decode_bytes(pc, bytes, 0) else {
            out.push(FormattedLine {
                address: pc,
                size: 2,
                hex_bytes: String::new(),
                mnemonic: "????".into(),
                operands: String::new(),
                branch_target: None,
                is_delay_slot_carrier: false,
            });
            pc = pc.wrapping_add(2);
            i += 1;
            continue;
        };
        let total = dec.total_size();
        let raw_start = pc as usize;
        let raw_end = (raw_start + total as usize).min(bytes.len());
        let raw = if raw_start < bytes.len() {
            &bytes[raw_start..raw_end]
        } else {
            &[][..]
        };
        let line = format_line(&dec, raw);
        pc = pc.wrapping_add(total);
        out.push(line);
        i += 1;
    }
    out
}

fn toggle_breakpoint(app: &EmulatorApp, address: u32, was_set: bool) {
    let cmd = if was_set {
        CpuCommand::RemoveBreakpoint { address }
    } else {
        let (tx, _rx) = crate::emu::command::oneshot::<usize>();
        CpuCommand::SetBreakpoint { address, response: tx }
    };
    let _ = app.handle.cpu_cmd.send(cmd);
}
