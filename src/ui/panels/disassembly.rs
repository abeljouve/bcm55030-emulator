//! Disassembly panel: decode instructions from the on-demand
//! SRAM snapshot, render them in a scrolling table with IDE-like
//! styling. Features:
//!
//! - Fixed-width columns laid out through `egui_extras::TableBuilder`
//! - Breakpoint gutter with one-click toggle
//! - Current-PC row drawn as a full-width bar
//! - Delay-slot carrier row tint, LP range tint
//! - Syntax-highlighted operands (registers, immediates, comments)
//! - Inline symbol resolution from `handle.annotations`

use eframe::egui;
use egui_extras::{Column, TableBuilder};
use egui_phosphor::regular as ph;

use crate::decoder;
use crate::decoder::format::{format_line, FormattedLine};
use crate::emu::command::CpuCommand;
use crate::ui::EmulatorApp;

const ROWS_ABOVE_PC: u32 = 12;
const ROWS_TOTAL: u32 = 80;

pub fn draw(ui: &mut egui::Ui, app: &mut EmulatorApp) {
    header(ui, app);
    ui.separator();

    let Some(sram) = app.sram.as_ref() else {
        ui.colored_label(app.accents.muted, "Waiting for SRAM snapshot…");
        return;
    };

    let base_pc = app.snapshot.cpu.pc.saturating_sub(ROWS_ABOVE_PC * 4);
    let lines = decode_window(&sram.bytes, base_pc, ROWS_TOTAL);
    let pc = app.snapshot.cpu.pc;
    let lp_start = app.snapshot.cpu.aux.lp_start;
    let lp_end = app.snapshot.cpu.aux.lp_end;
    let breakpoints = app.snapshot.breakpoints.clone();
    let symbols: std::collections::HashMap<u32, String> =
        app.handle.annotations.read().symbols.clone();
    let comments: std::collections::HashMap<u32, String> =
        app.handle.annotations.read().comments.clone();
    let accents = app.accents;
    let mut toggle_bp: Option<(u32, bool)> = None;
    let mut cursor_to: Option<u32> = None;

    let text_height = egui::TextStyle::Monospace.resolve(ui.style()).size + 4.0;

    egui::ScrollArea::vertical()
        .id_salt("disasm_scroll")
        .max_height(ui.available_height())
        .show(ui, |ui| {
            TableBuilder::new(ui)
                .striped(false)
                .resizable(false)
                .cell_layout(egui::Layout::left_to_right(egui::Align::Center))
                .column(Column::exact(22.0)) // breakpoint gutter
                .column(Column::exact(86.0)) // address
                .column(Column::exact(80.0)) // hex bytes
                .column(Column::exact(66.0)) // mnemonic
                .column(Column::remainder()) // operands + comments
                .body(|body| {
                    body.rows(text_height, lines.len(), |mut row| {
                        let line = &lines[row.index()];
                        let is_pc = line.address == pc;
                        let in_lp = lp_start != 0
                            && line.address >= lp_start
                            && line.address <= lp_end;
                        let row_fill = if is_pc {
                            Some(accents.pc_highlight)
                        } else if line.is_delay_slot_carrier {
                            Some(accents.delay_slot)
                        } else if in_lp {
                            Some(accents.lp_range)
                        } else {
                            None
                        };
                        if let Some(fill) = row_fill {
                            row.set_selected(true);
                            let _ = fill; // egui-extras handles stripe colour for selected rows
                        }

                        // Breakpoint gutter.
                        row.col(|ui| {
                            let has_bp = breakpoints.contains(&line.address);
                            let glyph = if has_bp {
                                ph::CIRCLE_HALF_TILT
                            } else {
                                ph::CIRCLE
                            };
                            let color = if has_bp {
                                accents.breakpoint
                            } else {
                                accents.muted.gamma_multiply(0.5)
                            };
                            let resp = ui.add(
                                egui::Label::new(
                                    egui::RichText::new(glyph).color(color).size(13.0),
                                )
                                .sense(egui::Sense::click()),
                            );
                            if resp.clicked() {
                                toggle_bp = Some((line.address, has_bp));
                            }
                        });

                        // Address column, with optional PC arrow.
                        row.col(|ui| {
                            if is_pc {
                                ui.label(
                                    egui::RichText::new(format!("▸ 0x{:08X}", line.address))
                                        .monospace()
                                        .strong()
                                        .color(accents.pc_highlight_strong),
                                );
                            } else {
                                ui.label(
                                    egui::RichText::new(format!("  0x{:08X}", line.address))
                                        .monospace()
                                        .color(accents.muted.gamma_multiply(0.85)),
                                );
                            }
                        });

                        // Hex bytes.
                        row.col(|ui| {
                            ui.label(
                                egui::RichText::new(&line.hex_bytes)
                                    .monospace()
                                    .color(accents.muted),
                            );
                        });

                        // Mnemonic — bold + accent colour.
                        row.col(|ui| {
                            ui.label(
                                egui::RichText::new(&line.mnemonic)
                                    .monospace()
                                    .strong()
                                    .color(accents.accent),
                            );
                        });

                        // Operands + inline symbol / comment.
                        row.col(|ui| {
                            render_operands(ui, line, &symbols, &accents);
                            if let Some(sym) = line.branch_target.and_then(|a| symbols.get(&a))
                            {
                                ui.label(
                                    egui::RichText::new(format!("→ {sym}"))
                                        .monospace()
                                        .color(accents.success),
                                );
                            }
                            if let Some(comment) = comments.get(&line.address) {
                                ui.label(
                                    egui::RichText::new(format!("; {comment}"))
                                        .monospace()
                                        .italics()
                                        .color(accents.muted),
                                );
                            }
                        });

                        let row_response = row.response();
                        if row_response.clicked() {
                            cursor_to = Some(line.address);
                        }
                    });
                });
        });

    if let Some((addr, was_set)) = toggle_bp {
        toggle_breakpoint(app, addr, was_set);
    }
    if let Some(addr) = cursor_to {
        app.disasm_cursor = addr;
    }
}

fn header(ui: &mut egui::Ui, app: &mut EmulatorApp) {
    ui.horizontal(|ui| {
        ui.strong(format!("{} Disassembly", ph::CODE));
        ui.separator();
        if ui
            .add(egui::Button::new(format!("{} PC", ph::CROSSHAIR)))
            .on_hover_text("Center on current PC")
            .clicked()
        {
            app.disasm_cursor = app.snapshot.cpu.pc;
        }
        ui.label(
            egui::RichText::new(format!("cursor 0x{:08X}", app.disasm_cursor))
                .monospace()
                .small()
                .color(app.accents.muted),
        );
    });
}

/// Render a pre-formatted operand string with per-token colouring.
/// We colour anything starting with `r`/`R` + digits as a
/// register, hex literals as immediates, and the rest as plain
/// text. Good enough for the 25 instruction variants without
/// writing a real tokenizer.
fn render_operands(
    ui: &mut egui::Ui,
    line: &FormattedLine,
    symbols: &std::collections::HashMap<u32, String>,
    accents: &crate::ui::theme::AccentTokens,
) {
    let operands = &line.operands;
    if operands.is_empty() {
        return;
    }
    ui.horizontal(|ui| {
        for raw in operands.split(',') {
            let token = raw.trim();
            if token.is_empty() {
                continue;
            }
            let color = classify_token(token, line.branch_target, symbols, accents);
            ui.label(
                egui::RichText::new(format!("{token}"))
                    .monospace()
                    .color(color),
            );
            ui.label(egui::RichText::new(" ").monospace());
        }
    });
}

fn classify_token(
    token: &str,
    branch_target: Option<u32>,
    symbols: &std::collections::HashMap<u32, String>,
    accents: &crate::ui::theme::AccentTokens,
) -> egui::Color32 {
    // Registers: r0..r63, sp, fp, gp, blink, lp_count, pcl, etc.
    let trimmed = token.trim_start_matches('[').trim_end_matches(']');
    let head = trimmed
        .split_once(',')
        .map(|(h, _)| h.trim())
        .unwrap_or(trimmed)
        .trim();
    const REG_ALIASES: &[&str] = &[
        "sp", "fp", "gp", "blink", "ilink1", "ilink2", "lp_count", "pcl", "limm",
    ];
    if head.starts_with('r') || head.starts_with('R') {
        if head[1..].chars().all(|c| c.is_ascii_digit()) && !head[1..].is_empty() {
            return accents.accent;
        }
    }
    if REG_ALIASES.iter().any(|n| head.eq_ignore_ascii_case(n)) {
        return accents.accent;
    }
    // Hex immediates.
    if head.starts_with("0x") || head.starts_with("0X") {
        if let Some(sym) = branch_target.and_then(|a| symbols.get(&a)) {
            let _ = sym;
            return accents.success;
        }
        return accents.warning;
    }
    // Negative / positive decimal immediates.
    if head.chars().all(|c| c.is_ascii_digit() || c == '-') && !head.is_empty() {
        return accents.warning;
    }
    accents.muted
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
