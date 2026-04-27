//! Memory viewer: hex + ASCII grid with SRAM / Flash / D-cache
//! sub-tabs. Bytes are coloured by value (zeroes muted, printable
//! ASCII tinted green), hex ↔ ASCII hover is synchronised, and
//! long runs of all-zero rows are folded into an ellipsis marker
//! à la `xxd -a` so the viewer stays readable on sparse images.

use eframe::egui;
use egui_phosphor::regular as ph;

use crate::ui::EmulatorApp;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Tab {
    Sram,
    Flash,
    Dcache,
}

const BYTES_PER_ROW: usize = 16;
/// Minimum number of consecutive all-zero rows that triggers
/// collapse into a single ellipsis marker. 4 rows = 64 bytes —
/// below that threshold it is cheaper to keep the rows visible.
const ELIDE_THRESHOLD: usize = 4;

/// A logical row in the hex grid after compaction. Either a real
/// 16-byte row at a given memory offset, or an elided run.
#[derive(Clone, Copy, Debug)]
enum Row {
    Data { offset: usize },
    Elided { start: usize, end: usize },
}

pub fn draw(ui: &mut egui::Ui, app: &mut EmulatorApp) {
    ui.horizontal(|ui| {
        ui.strong(format!("{} Memory", ph::MEMORY));
        ui.separator();
        ui.selectable_value(&mut app.memory_tab, Tab::Sram, "SRAM");
        ui.selectable_value(&mut app.memory_tab, Tab::Flash, "Flash");
        ui.selectable_value(&mut app.memory_tab, Tab::Dcache, "D-cache");
        ui.separator();
        ui.label(
            egui::RichText::new("Go to")
                .small()
                .color(app.accents.muted),
        );
        let mut buf = format!("0x{:08X}", app.memory_cursor);
        if ui
            .add(egui::TextEdit::singleline(&mut buf).desired_width(110.0))
            .changed()
        {
            let trimmed = buf.trim_start_matches("0x").trim_start_matches("0X");
            if let Ok(addr) = u32::from_str_radix(trimmed, 16) {
                app.memory_cursor = addr;
                app.memory_cursor_dirty = true;
            }
        }
    });
    ui.separator();

    match app.memory_tab {
        Tab::Sram => draw_sram(ui, app),
        Tab::Flash => draw_flash(ui, app),
        Tab::Dcache => draw_dcache(ui, app),
    }
}

fn draw_sram(ui: &mut egui::Ui, app: &mut EmulatorApp) {
    let Some(sram) = app.sram.clone() else {
        ui.colored_label(app.accents.muted, "Waiting for SRAM snapshot…");
        return;
    };
    draw_hex_grid(ui, app, &sram.bytes, None, "sram");
}

fn draw_flash(ui: &mut egui::Ui, app: &mut EmulatorApp) {
    // Clone the flash + baseline under a short read lock so we
    // don't hold it across the scroll-area draw.
    let (flash, baseline) = {
        let guard = app.handle.bank.read();
        (
            guard.pbc.flash.data.clone(),
            guard.pbc.flash.baseline.clone(),
        )
    };
    if let Some(ref base) = baseline {
        let dirty_bytes = flash
            .iter()
            .zip(base.iter())
            .filter(|(a, b)| a != b)
            .count();
        ui.horizontal(|ui| {
            ui.label(
                egui::RichText::new(format!(
                    "{} bytes modified since load",
                    dirty_bytes
                ))
                .small()
                .color(if dirty_bytes == 0 {
                    app.accents.muted
                } else {
                    app.accents.warning
                }),
            );
            ui.separator();
            let has_dirty = dirty_bytes > 0;
            if ui
                .add_enabled(
                    has_dirty,
                    egui::Button::new(format!("{} Prev dirty", ph::CARET_LEFT)),
                )
                .on_hover_text("Jump to the previous row with modified bytes")
                .clicked()
            {
                if let Some(off) = find_dirty_row(&flash, base, app.memory_cursor, false)
                {
                    app.memory_cursor = off;
                    app.memory_cursor_dirty = true;
                }
            }
            if ui
                .add_enabled(
                    has_dirty,
                    egui::Button::new(format!("Next dirty {}", ph::CARET_RIGHT)),
                )
                .on_hover_text("Jump to the next row with modified bytes")
                .clicked()
            {
                if let Some(off) = find_dirty_row(&flash, base, app.memory_cursor, true) {
                    app.memory_cursor = off;
                    app.memory_cursor_dirty = true;
                }
            }
        });
        ui.separator();
    }
    draw_hex_grid(ui, app, &flash, baseline.as_deref(), "flash");
}

fn draw_hex_grid(
    ui: &mut egui::Ui,
    app: &mut EmulatorApp,
    bytes: &[u8],
    baseline: Option<&[u8]>,
    id: &str,
) {
    let accents = app.accents;
    let zero_color = accents.muted.gamma_multiply(0.4);
    let ascii_color = accents.success;
    let text_color = ui.visuals().text_color();

    let rows = compact_rows(bytes, baseline);
    let row_height = egui::TextStyle::Monospace.resolve(ui.style()).size + 3.0;

    // Build a byte-offset → compacted-row-index lookup so the
    // "Go to" + "Prev / Next dirty" buttons can scroll to the
    // right place even after compaction.
    let cursor_off = app.memory_cursor as usize;
    let jumped_this_frame = app.memory_cursor_dirty;
    app.memory_cursor_dirty = false;
    let cursor_row_idx = rows.iter().position(|r| match r {
        Row::Data { offset } => *offset <= cursor_off && cursor_off < *offset + BYTES_PER_ROW,
        Row::Elided { start, end } => *start <= cursor_off && cursor_off < *end,
    });

    let mut scroll_area = egui::ScrollArea::vertical()
        .id_salt(id)
        .auto_shrink([false, false])
        .max_height(ui.available_height());
    if jumped_this_frame {
        if let Some(row_idx) = cursor_row_idx {
            scroll_area =
                scroll_area.vertical_scroll_offset(row_idx as f32 * row_height);
        }
    }
    scroll_area.show_rows(ui, row_height, rows.len(), |ui, row_range| {
        ui.spacing_mut().item_spacing.y = 1.0;
        for row_idx in row_range {
            let row = rows[row_idx];
            match row {
                Row::Data { offset } => draw_data_row(
                    ui,
                    bytes,
                    baseline,
                    offset,
                    zero_color,
                    ascii_color,
                    text_color,
                    &accents,
                ),
                Row::Elided { start, end } => draw_elided_row(ui, &accents, start, end),
            }
        }
    });
}

fn draw_data_row(
    ui: &mut egui::Ui,
    bytes: &[u8],
    baseline: Option<&[u8]>,
    off: usize,
    zero_color: egui::Color32,
    ascii_color: egui::Color32,
    text_color: egui::Color32,
    accents: &crate::ui::theme::AccentTokens,
) {
    let row_end = (off + BYTES_PER_ROW).min(bytes.len());
    let slice = &bytes[off..row_end];

    ui.horizontal(|ui| {
        // Offset.
        ui.label(
            egui::RichText::new(format!("{:08X}", off))
                .monospace()
                .color(accents.muted),
        );
        ui.add_space(10.0);

        // Hex bytes with hover ID + baseline diff highlight.
        let mut hover_idx: Option<usize> = None;
        for (i, b) in slice.iter().enumerate() {
            let abs = off + i;
            let modified = baseline
                .map(|base| abs < base.len() && *b != base[abs])
                .unwrap_or(false);
            let color = if modified {
                accents.warning
            } else if *b == 0 {
                zero_color
            } else if b.is_ascii_graphic() || *b == b' ' {
                ascii_color
            } else {
                text_color
            };
            let mut rich = egui::RichText::new(format!("{:02X}", b))
                .monospace()
                .color(color);
            if modified {
                rich = rich
                    .strong()
                    .background_color(accents.warning.gamma_multiply(0.22));
            }
            let resp = ui.add(egui::Label::new(rich).sense(egui::Sense::hover()));
            if resp.hovered() {
                hover_idx = Some(i);
            }
            if i == 7 {
                ui.add_space(4.0);
            }
        }

        ui.add_space(10.0);
        ui.label(egui::RichText::new("│").monospace().color(accents.muted));

        // ASCII column — highlight the hovered index.
        for (i, b) in slice.iter().enumerate() {
            let ch = if b.is_ascii_graphic() || *b == b' ' {
                *b as char
            } else {
                '.'
            };
            let mut rich = egui::RichText::new(ch.to_string()).monospace();
            rich = if Some(i) == hover_idx {
                rich.background_color(accents.accent.gamma_multiply(0.35))
                    .color(ui.visuals().text_color())
            } else if *b == 0 {
                rich.color(zero_color)
            } else {
                rich.color(ascii_color)
            };
            ui.label(rich);
        }
    });
}

fn draw_elided_row(
    ui: &mut egui::Ui,
    accents: &crate::ui::theme::AccentTokens,
    start: usize,
    end: usize,
) {
    let span = end - start;
    ui.horizontal(|ui| {
        ui.label(
            egui::RichText::new(format!("{:08X}", start))
                .monospace()
                .color(accents.muted.gamma_multiply(0.6)),
        );
        ui.add_space(10.0);
        ui.label(
            egui::RichText::new(format!(
                "… {} zero bytes ({} rows) elided …",
                span,
                span / BYTES_PER_ROW
            ))
            .monospace()
            .italics()
            .color(accents.muted.gamma_multiply(0.7)),
        );
    });
}

/// Build the compacted row list for a byte slice. Runs of
/// `ELIDE_THRESHOLD` or more consecutive all-zero rows (in both
/// `bytes` and `baseline` when a baseline is present) are folded
/// into a single `Row::Elided` marker, with the first and last
/// zero rows kept as `Row::Data` anchors so the user still sees
/// the run boundaries.
fn compact_rows(bytes: &[u8], baseline: Option<&[u8]>) -> Vec<Row> {
    let mut out = Vec::new();
    let mut off = 0usize;
    while off < bytes.len() {
        let end = (off + BYTES_PER_ROW).min(bytes.len());
        if !row_is_blank(bytes, baseline, off, end) {
            out.push(Row::Data { offset: off });
            off = end;
            continue;
        }
        // Count consecutive blank rows.
        let run_start = off;
        let mut cursor = off;
        while cursor < bytes.len() {
            let row_end = (cursor + BYTES_PER_ROW).min(bytes.len());
            if row_is_blank(bytes, baseline, cursor, row_end) {
                cursor += BYTES_PER_ROW;
            } else {
                break;
            }
        }
        let run_bytes = cursor.min(bytes.len()) - run_start;
        let run_rows = (run_bytes + BYTES_PER_ROW - 1) / BYTES_PER_ROW;
        if run_rows >= ELIDE_THRESHOLD {
            // Keep the first and last zero rows visible as anchors.
            out.push(Row::Data { offset: run_start });
            out.push(Row::Elided {
                start: run_start + BYTES_PER_ROW,
                end: run_start + (run_rows - 1) * BYTES_PER_ROW,
            });
            out.push(Row::Data {
                offset: run_start + (run_rows - 1) * BYTES_PER_ROW,
            });
        } else {
            for i in 0..run_rows {
                out.push(Row::Data {
                    offset: run_start + i * BYTES_PER_ROW,
                });
            }
        }
        off = cursor;
    }
    out
}

#[inline]
fn row_is_blank(bytes: &[u8], baseline: Option<&[u8]>, off: usize, end: usize) -> bool {
    let live_zero = bytes[off..end].iter().all(|b| *b == 0);
    if !live_zero {
        return false;
    }
    // For the flash tab, also require the baseline row to be
    // all-zero — we do not want to hide a region that went from
    // non-zero in the baseline to zero in the live image.
    match baseline {
        Some(base) => {
            let b_end = end.min(base.len());
            off >= base.len() || base[off..b_end].iter().all(|b| *b == 0)
        }
        None => true,
    }
}

/// Find the next (or previous) 16-byte-aligned row that contains
/// at least one byte differing from the baseline. Wraps at the
/// extremities so the user can sweep the whole flash forever.
fn find_dirty_row(
    flash: &[u8],
    baseline: &[u8],
    current_addr: u32,
    forward: bool,
) -> Option<u32> {
    let len = flash.len().min(baseline.len());
    if len == 0 {
        return None;
    }
    let total_rows = (len + BYTES_PER_ROW - 1) / BYTES_PER_ROW;
    let cur_row = (current_addr as usize / BYTES_PER_ROW).min(total_rows.saturating_sub(1));
    for step in 1..=total_rows {
        let idx = if forward {
            (cur_row + step) % total_rows
        } else {
            (cur_row + total_rows - step) % total_rows
        };
        let off = idx * BYTES_PER_ROW;
        let end = (off + BYTES_PER_ROW).min(len);
        let modified = (off..end).any(|i| flash[i] != baseline[i]);
        if modified {
            return Some(off as u32);
        }
    }
    None
}

fn draw_dcache(ui: &mut egui::Ui, app: &EmulatorApp) {
    let Some(dc) = app.dcache.as_ref() else {
        ui.colored_label(
            app.accents.muted,
            "D-cache snapshot not requested yet.",
        );
        return;
    };
    ui.label(
        egui::RichText::new(format!("DC_CTRL 0x{:08X}", dc.ctrl_raw))
            .monospace()
            .color(app.accents.muted),
    );
    ui.separator();
    egui::ScrollArea::vertical()
        .id_salt("dcache")
        .max_height(ui.available_height())
        .show(ui, |ui| {
            for line in &dc.lines {
                if !line.valid {
                    continue;
                }
                let color = if line.dirty {
                    app.accents.warning
                } else {
                    ui.visuals().text_color()
                };
                ui.label(
                    egui::RichText::new(format!(
                        "set {:02} way {}  tag 0x{:08X}  base 0x{:08X} {}",
                        line.set,
                        line.way,
                        line.tag,
                        line.base_addr,
                        if line.dirty { "[dirty]" } else { "" },
                    ))
                    .monospace()
                    .color(color),
                );
            }
        });
}
