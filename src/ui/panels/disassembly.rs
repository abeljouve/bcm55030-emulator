//! Disassembly panel: Ghidra-like linear sweep with free
//! scrolling, branch arcs in the left gutter, PC highlight, and
//! syntax-coloured operands.
//!
//! Core model:
//! - `disasm_view_base` is the top-of-view address the panel
//!   renders from. It is independent of the CPU PC.
//! - `disasm_follow_pc` (default true) snaps the view to track
//!   the current PC each frame. Any manual scroll / navigation
//!   clears the flag; the toolbar "Center on PC" button sets it
//!   again.
//! - Scroll wheel, Page Up/Down, arrow keys, Home/End all drive
//!   `disasm_view_base` directly, letting the user sweep the
//!   whole 512 KB SRAM one instruction at a time.
//!
//! Branch arc gutter: after decoding the visible window, we
//! collect every `branch_target` and draw a vertical segment
//! connecting the source row to the destination row. Arcs whose
//! target lands off-screen render as an arrow pointing out of
//! the viewport.

use eframe::egui;
use egui_phosphor::regular as ph;

use crate::decoder;
use crate::decoder::format::{format_line, FormattedLine};
use crate::emu::command::CpuCommand;
use crate::ui::EmulatorApp;

const ARC_GUTTER_WIDTH: f32 = 72.0;

pub fn draw(ui: &mut egui::Ui, app: &mut EmulatorApp) {
    header(ui, app);
    ui.separator();

    let Some(sram) = app.sram.clone() else {
        ui.colored_label(app.accents.muted, "Waiting for SRAM snapshot…");
        return;
    };

    let sram_bytes: &[u8] = &sram.bytes;

    // 1. Keep the view base in sync with Follow PC mode.
    if app.disasm_follow_pc {
        app.disasm_view_base = align_view_base(app.snapshot.cpu.pc, sram_bytes);
    }

    // 2. Compute visible row count from the panel height.
    let text_height = egui::TextStyle::Monospace.resolve(ui.style()).size + 5.0;
    let avail_h = ui.available_height();
    let visible_rows = ((avail_h / text_height).floor() as u32).max(8).min(256);
    app.disasm_visible_rows = visible_rows;

    // 3. Handle navigation input — scroll wheel, keys. This may
    //    update `disasm_view_base` and flip follow_pc off.
    let base = handle_navigation(ui, app, sram_bytes, visible_rows);

    // 4. Decode the visible window at the current base.
    let lines = decode_window(sram_bytes, base, visible_rows);

    // 5. Render the gutter + rows. We use a single horizontal
    //    layout per row and track the rects so the arc painter
    //    can find row centres afterwards.
    let pc = app.snapshot.cpu.pc;
    let lp_start = app.snapshot.cpu.aux.lp_start;
    let lp_end = app.snapshot.cpu.aux.lp_end;
    let breakpoints = app.snapshot.breakpoints.clone();
    let annotations = app.handle.annotations.read().clone();
    let symbols = &annotations.symbols;
    let comments = &annotations.comments;
    let accents = app.accents;

    let mut toggle_bp: Option<(u32, bool)> = None;
    let mut cursor_to: Option<u32> = None;
    let mut row_rects: Vec<(u32, egui::Rect)> = Vec::with_capacity(lines.len() as usize);

    let panel_rect = ui
        .available_rect_before_wrap();
    let panel_painter = ui.painter_at(panel_rect);

    ui.vertical(|ui| {
        ui.spacing_mut().item_spacing.y = 1.0;
        for line in &lines {
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

            let inner = ui
                .allocate_ui_with_layout(
                    egui::vec2(ui.available_width(), text_height),
                    egui::Layout::left_to_right(egui::Align::Center),
                    |ui| {
                        // Row-wide background.
                        let full_rect = ui.available_rect_before_wrap();
                        if let Some(fill) = row_fill {
                            ui.painter().rect_filled(
                                full_rect,
                                egui::CornerRadius::same(2),
                                fill,
                            );
                        }

                        // Arc gutter reservation — empty space,
                        // filled later by the arc painter.
                        ui.add_space(ARC_GUTTER_WIDTH);

                        // Breakpoint gutter.
                        let has_bp = breakpoints.contains(&line.address);
                        let glyph = if has_bp {
                            ph::CIRCLE_HALF_TILT
                        } else {
                            ph::CIRCLE
                        };
                        let color = if has_bp {
                            accents.breakpoint
                        } else {
                            accents.muted.gamma_multiply(0.4)
                        };
                        let resp = ui.add(
                            egui::Label::new(
                                egui::RichText::new(glyph)
                                    .color(color)
                                    .size(12.0),
                            )
                            .sense(egui::Sense::click()),
                        );
                        if resp.clicked() {
                            toggle_bp = Some((line.address, has_bp));
                        }

                        // Address.
                        if is_pc {
                            ui.label(
                                egui::RichText::new(format!(
                                    "▸ 0x{:08X}",
                                    line.address
                                ))
                                .monospace()
                                .strong()
                                .color(accents.pc_highlight_strong),
                            );
                        } else {
                            ui.label(
                                egui::RichText::new(format!(
                                    "  0x{:08X}",
                                    line.address
                                ))
                                .monospace()
                                .color(accents.muted.gamma_multiply(0.85)),
                            );
                        }
                        ui.add_space(6.0);

                        // Hex bytes — fixed width column.
                        ui.add_sized(
                            [80.0, text_height],
                            egui::Label::new(
                                egui::RichText::new(&line.hex_bytes)
                                    .monospace()
                                    .color(accents.muted),
                            ),
                        );

                        // Mnemonic — bold accent.
                        ui.add_sized(
                            [64.0, text_height],
                            egui::Label::new(
                                egui::RichText::new(&line.mnemonic)
                                    .monospace()
                                    .strong()
                                    .color(accents.accent),
                            ),
                        );

                        // Operands + inline symbol / comment.
                        render_operands(ui, line, symbols, &accents);
                        if let Some(sym) =
                            line.branch_target.and_then(|a| symbols.get(&a))
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
                    },
                );

            let row_rect = inner.response.rect;
            row_rects.push((line.address, row_rect));

            // Full-row click handling — a click anywhere outside
            // the gutter / breakpoint dot moves the cursor.
            let row_resp = ui.interact(
                row_rect,
                egui::Id::new(("disasm_row", line.address)),
                egui::Sense::click(),
            );
            if row_resp.clicked() {
                cursor_to = Some(line.address);
            }
        }
    });

    // 6. Paint the branch arcs on top of the reserved gutter.
    paint_branch_arcs(&panel_painter, &lines, &row_rects, &accents);

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

        let mut follow = app.disasm_follow_pc;
        if ui
            .add(egui::Button::new(format!("{} Follow PC", ph::CROSSHAIR)).selected(follow))
            .on_hover_text("Keep the viewport centred on the current PC")
            .clicked()
        {
            follow = !follow;
            app.disasm_follow_pc = follow;
            if follow {
                app.disasm_view_base = app.snapshot.cpu.pc;
            }
        }

        if ui
            .add(egui::Button::new(format!("{} Center", ph::ARROW_LINE_LEFT)))
            .on_hover_text("Jump to the current PC (one-shot)")
            .clicked()
        {
            app.disasm_view_base = app.snapshot.cpu.pc;
            app.disasm_follow_pc = true;
        }

        ui.separator();
        ui.label(
            egui::RichText::new("Go to")
                .small()
                .color(app.accents.muted),
        );
        let mut buf = format!("0x{:08X}", app.disasm_view_base);
        let resp = ui.add(egui::TextEdit::singleline(&mut buf).desired_width(110.0));
        if resp.changed() {
            let trimmed = buf.trim_start_matches("0x").trim_start_matches("0X");
            if let Ok(addr) = u32::from_str_radix(trimmed, 16) {
                app.disasm_view_base = addr;
                app.disasm_follow_pc = false;
            }
        }

        ui.label(
            egui::RichText::new(format!(
                "cursor 0x{:08X}",
                app.disasm_cursor
            ))
            .monospace()
            .small()
            .color(app.accents.muted),
        );
    });
}

/// Process scroll / keyboard navigation. Returns the view base
/// to use for this frame's draw (which is usually the updated
/// `app.disasm_view_base`). Clears `follow_pc` on any user
/// interaction.
fn handle_navigation(
    ui: &mut egui::Ui,
    app: &mut EmulatorApp,
    bytes: &[u8],
    visible_rows: u32,
) -> u32 {
    let response_id = egui::Id::new("disasm_nav_layer");
    let hover_rect = ui.available_rect_before_wrap();
    let hover_resp =
        ui.interact(hover_rect, response_id, egui::Sense::click_and_drag());

    if hover_resp.hovered() {
        let scroll = ui.input(|i| i.smooth_scroll_delta.y);
        if scroll.abs() > 1.0 {
            // egui reports positive scroll = content moves
            // down = we show earlier addresses. The panel
            // mirrors a regular text editor.
            let text_height =
                egui::TextStyle::Monospace.resolve(ui.style()).size + 5.0;
            let rows = (scroll / text_height).round() as i32;
            if rows != 0 {
                shift_view(app, bytes, -rows);
            }
        }
    }

    // Keyboard navigation — only if the panel is hovered so
    // typing in a peripheral widget elsewhere is not captured.
    if hover_resp.hovered() || hover_resp.has_focus() {
        ui.input_mut(|i| {
            if i.consume_key(egui::Modifiers::NONE, egui::Key::ArrowDown) {
                shift_view_bytes(app, bytes, 1);
            }
            if i.consume_key(egui::Modifiers::NONE, egui::Key::ArrowUp) {
                shift_view_bytes(app, bytes, -1);
            }
            if i.consume_key(egui::Modifiers::NONE, egui::Key::PageDown) {
                shift_view(app, bytes, visible_rows as i32);
            }
            if i.consume_key(egui::Modifiers::NONE, egui::Key::PageUp) {
                shift_view(app, bytes, -(visible_rows as i32));
            }
            if i.consume_key(egui::Modifiers::NONE, egui::Key::Home) {
                app.disasm_view_base = 0;
                app.disasm_follow_pc = false;
            }
            if i.consume_key(egui::Modifiers::NONE, egui::Key::End) {
                let last = bytes.len().saturating_sub(1) as u32;
                app.disasm_view_base = last & !1;
                app.disasm_follow_pc = false;
            }
        });
    }

    app.disasm_view_base
}

/// Shift the view by `rows` instructions (positive = forward,
/// negative = backward). Crosses variable-length boundaries via
/// `step_forward` and `step_back`.
fn shift_view(app: &mut EmulatorApp, bytes: &[u8], rows: i32) {
    let mut base = app.disasm_view_base;
    if rows > 0 {
        for _ in 0..rows {
            base = step_forward(bytes, base);
        }
    } else {
        for _ in 0..(-rows) {
            base = step_back(bytes, base);
        }
    }
    app.disasm_view_base = base;
    app.disasm_follow_pc = false;
}

/// Arrow-key variant: shift by one instruction unit.
fn shift_view_bytes(app: &mut EmulatorApp, bytes: &[u8], rows: i32) {
    shift_view(app, bytes, rows);
}

/// Decode at `base` and advance past the instruction. Falls
/// back to +2 on decode failure so we never get stuck.
fn step_forward(bytes: &[u8], base: u32) -> u32 {
    match decoder::decode_bytes(base, bytes, 0) {
        Ok(dec) => base.wrapping_add(dec.total_size() as u32),
        Err(_) => base.wrapping_add(2),
    }
}

/// Heuristic backward step: scan backwards in 2-byte steps up
/// to 8 bytes and pick the candidate whose forward decode lands
/// exactly on `base`. Falls back to `base - 2` if nothing
/// matches (data region or start of SRAM).
fn step_back(bytes: &[u8], base: u32) -> u32 {
    if base == 0 {
        return 0;
    }
    for delta in (2u32..=8).step_by(2) {
        if let Some(candidate) = base.checked_sub(delta) {
            if let Ok(dec) = decoder::decode_bytes(candidate, bytes, 0) {
                if candidate.wrapping_add(dec.total_size() as u32) == base {
                    return candidate;
                }
            }
        }
    }
    base.saturating_sub(2)
}

/// Align a view base to the start of the instruction containing
/// `addr`. Used by Follow PC so the PC is always the 12th visible
/// row (rough centring).
fn align_view_base(addr: u32, bytes: &[u8]) -> u32 {
    let mut base = addr;
    for _ in 0..12 {
        base = step_back(bytes, base);
    }
    base
}

/// Render a pre-formatted operand string with per-token colouring.
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
                egui::RichText::new(token.to_string())
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
    if head.starts_with("0x") || head.starts_with("0X") {
        if branch_target.and_then(|a| symbols.get(&a)).is_some() {
            return accents.success;
        }
        return accents.warning;
    }
    if head.chars().all(|c| c.is_ascii_digit() || c == '-') && !head.is_empty() {
        return accents.warning;
    }
    accents.muted
}

/// Draw Ghidra-style branch arcs inside the left gutter. Every
/// `branch_target` lights up a vertical segment at a depth
/// proportional to the index of the arc (so longer arcs sit
/// further left). Arcs whose target is off-screen terminate in
/// a small arrow glyph at the top / bottom of the gutter.
fn paint_branch_arcs(
    painter: &egui::Painter,
    lines: &[FormattedLine],
    row_rects: &[(u32, egui::Rect)],
    accents: &crate::ui::theme::AccentTokens,
) {
    if row_rects.is_empty() {
        return;
    }
    // Build a quick address → row-center-y lookup.
    let row_centre =
        |addr: u32| row_rects.iter().find(|(a, _)| *a == addr).map(|(_, r)| r.center().y);

    // Gutter occupies the leftmost ARC_GUTTER_WIDTH of the
    // panel. Depth layers are 10 px wide each.
    let gutter_left = row_rects[0].1.left();
    let gutter_right = gutter_left + ARC_GUTTER_WIDTH - 4.0;
    let lane_width = 10.0;
    let max_lanes = ((ARC_GUTTER_WIDTH - 4.0) / lane_width) as usize;

    // Collect the arcs we actually want to draw.
    struct Arc {
        src_y: f32,
        dst_y: Option<f32>, // None = off-screen
        color: egui::Color32,
        dst_above: bool, // target is above the source?
    }
    let mut arcs: Vec<Arc> = Vec::new();
    for line in lines {
        let Some(target) = line.branch_target else { continue };
        let Some(src_y) = row_centre(line.address) else { continue };
        let dst_y_opt = row_centre(target);
        let dst_above = target < line.address;
        arcs.push(Arc {
            src_y,
            dst_y: dst_y_opt,
            color: if dst_above {
                accents.accent
            } else {
                accents.success
            },
            dst_above,
        });
    }
    if arcs.is_empty() {
        return;
    }

    // Sort arcs by vertical span (shorter first) so short arcs
    // take the rightmost lane — less clutter for tight loops.
    arcs.sort_by(|a, b| {
        let aa = a.dst_y.unwrap_or(0.0) - a.src_y;
        let bb = b.dst_y.unwrap_or(0.0) - b.src_y;
        aa.abs().partial_cmp(&bb.abs()).unwrap_or(std::cmp::Ordering::Equal)
    });

    let stroke_w = 1.4;
    for (idx, arc) in arcs.iter().enumerate() {
        let lane = idx % max_lanes;
        let x = gutter_right - (lane as f32 + 1.0) * lane_width;
        let connector_len = 6.0;

        match arc.dst_y {
            Some(dst_y) => {
                // Horizontal nub at source.
                painter.line_segment(
                    [egui::pos2(gutter_right, arc.src_y), egui::pos2(x, arc.src_y)],
                    egui::Stroke::new(stroke_w, arc.color),
                );
                // Vertical line between source and destination.
                painter.line_segment(
                    [egui::pos2(x, arc.src_y), egui::pos2(x, dst_y)],
                    egui::Stroke::new(stroke_w, arc.color),
                );
                // Horizontal arrow at destination.
                painter.line_segment(
                    [egui::pos2(x, dst_y), egui::pos2(gutter_right, dst_y)],
                    egui::Stroke::new(stroke_w, arc.color),
                );
                // Arrowhead pointing right into the row.
                arrow_head(
                    painter,
                    egui::pos2(gutter_right, dst_y),
                    arc.color,
                    4.0,
                );
                let _ = connector_len;
            }
            None => {
                // Target off-screen — draw a vertical going
                // to the panel edge with an arrow.
                let top = row_rects.first().map(|(_, r)| r.top()).unwrap_or(0.0);
                let bottom = row_rects.last().map(|(_, r)| r.bottom()).unwrap_or(0.0);
                let end_y = if arc.dst_above { top } else { bottom };
                painter.line_segment(
                    [egui::pos2(gutter_right, arc.src_y), egui::pos2(x, arc.src_y)],
                    egui::Stroke::new(stroke_w, arc.color),
                );
                painter.line_segment(
                    [egui::pos2(x, arc.src_y), egui::pos2(x, end_y)],
                    egui::Stroke::new(stroke_w, arc.color),
                );
                arrow_head_vertical(
                    painter,
                    egui::pos2(x, end_y),
                    arc.color,
                    4.0,
                    arc.dst_above,
                );
            }
        }
    }
}

fn arrow_head(painter: &egui::Painter, tip: egui::Pos2, color: egui::Color32, size: f32) {
    let stroke = egui::Stroke::new(1.4, color);
    painter.line_segment([tip, tip + egui::vec2(-size, -size * 0.6)], stroke);
    painter.line_segment([tip, tip + egui::vec2(-size, size * 0.6)], stroke);
}

fn arrow_head_vertical(
    painter: &egui::Painter,
    tip: egui::Pos2,
    color: egui::Color32,
    size: f32,
    pointing_up: bool,
) {
    let stroke = egui::Stroke::new(1.4, color);
    let dy = if pointing_up { size } else { -size };
    painter.line_segment([tip, tip + egui::vec2(-size * 0.6, dy)], stroke);
    painter.line_segment([tip, tip + egui::vec2(size * 0.6, dy)], stroke);
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
        CpuCommand::SetBreakpoint {
            address,
            response: tx,
        }
    };
    let _ = app.handle.cpu_cmd.send(cmd);
}
