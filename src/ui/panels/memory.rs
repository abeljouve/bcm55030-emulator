//! Memory viewer: hex + ASCII grid with SRAM / Flash / D-cache
//! sub-tabs. Bytes are coloured by value (zeroes muted, printable
//! ASCII tinted green) and hex ↔ ASCII hover is synchronised so a
//! pointer over one column highlights the other.

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
const ROWS_VISIBLE: usize = 40;

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

fn draw_sram(ui: &mut egui::Ui, app: &EmulatorApp) {
    let Some(sram) = app.sram.as_ref() else {
        ui.colored_label(app.accents.muted, "Waiting for SRAM snapshot…");
        return;
    };
    let start = (app.memory_cursor as usize) & !(BYTES_PER_ROW - 1);
    draw_hex_grid(ui, app, &sram.bytes, None, start, "sram");
}

fn draw_flash(ui: &mut egui::Ui, app: &EmulatorApp) {
    // Clone the flash + baseline under a short read lock so we
    // don't hold it across the scroll-area draw (which can
    // allocate and trigger repaint requests).
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
        });
        ui.separator();
    }
    let start = (app.memory_cursor as usize) & !(BYTES_PER_ROW - 1);
    draw_hex_grid(ui, app, &flash, baseline.as_deref(), start, "flash");
}

fn draw_hex_grid(
    ui: &mut egui::Ui,
    app: &EmulatorApp,
    bytes: &[u8],
    baseline: Option<&[u8]>,
    start: usize,
    id: &str,
) {
    let accents = app.accents;
    let zero_color = accents.muted.gamma_multiply(0.4);
    let ascii_color = accents.success;
    let text_color = ui.visuals().text_color();

    egui::ScrollArea::vertical()
        .id_salt(id)
        .max_height(ui.available_height())
        .show(ui, |ui| {
            ui.spacing_mut().item_spacing.y = 1.0;
            for row in 0..ROWS_VISIBLE {
                let off = start + row * BYTES_PER_ROW;
                if off >= bytes.len() {
                    break;
                }
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

                    // Hex bytes with hover ID per byte. Bytes
                    // that differ from the baseline (flash only)
                    // get a warm accent tint + coloured background.
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
                        let resp = ui
                            .add(egui::Label::new(rich).sense(egui::Sense::hover()));
                        if resp.hovered() {
                            hover_idx = Some(i);
                        }
                        if i == 7 {
                            ui.add_space(4.0);
                        }
                    }

                    ui.add_space(10.0);
                    ui.label(
                        egui::RichText::new("│").monospace().color(accents.muted),
                    );

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
        });
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
