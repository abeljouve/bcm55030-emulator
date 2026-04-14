//! Memory viewer: hex + ASCII grid with SRAM / Flash / D-cache
//! sub-tabs. Reads from the on-demand `SramSnapshot` (disassembly
//! panel drives the refresh) and from `handle.bank` for flash.

use eframe::egui;

use crate::ui::theme;
use crate::ui::EmulatorApp;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Tab {
    Sram,
    Flash,
    Dcache,
}

const BYTES_PER_ROW: usize = 16;
const ROWS_VISIBLE: usize = 32;

pub fn draw(ui: &mut egui::Ui, app: &mut EmulatorApp) {
    ui.horizontal(|ui| {
        ui.selectable_value(&mut app.memory_tab, Tab::Sram, "SRAM");
        ui.selectable_value(&mut app.memory_tab, Tab::Flash, "Flash");
        ui.selectable_value(&mut app.memory_tab, Tab::Dcache, "D-cache");
        ui.separator();
        ui.label("Go to:");
        let mut buf = format!("0x{:08X}", app.memory_cursor);
        if ui
            .add(egui::TextEdit::singleline(&mut buf).desired_width(120.0))
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
        ui.colored_label(theme::MUTED, "Waiting for SRAM snapshot…");
        return;
    };
    let start = (app.memory_cursor as usize) & !(BYTES_PER_ROW - 1);
    draw_hex_grid(ui, &sram.bytes, start, "sram");
}

fn draw_flash(ui: &mut egui::Ui, app: &EmulatorApp) {
    let guard = app.handle.bank.read();
    let flash = &guard.pbc.flash.data;
    let start = (app.memory_cursor as usize) & !(BYTES_PER_ROW - 1);
    draw_hex_grid(ui, flash, start, "flash");
}

fn draw_hex_grid(ui: &mut egui::Ui, bytes: &[u8], start: usize, id: &str) {
    egui::ScrollArea::vertical()
        .id_salt(id)
        .max_height(ui.available_height())
        .show(ui, |ui| {
            for row in 0..ROWS_VISIBLE {
                let off = start + row * BYTES_PER_ROW;
                if off >= bytes.len() {
                    break;
                }
                let row_end = (off + BYTES_PER_ROW).min(bytes.len());
                let slice = &bytes[off..row_end];
                let hex = slice
                    .iter()
                    .map(|b| format!("{:02X}", b))
                    .collect::<Vec<_>>()
                    .join(" ");
                let ascii = slice
                    .iter()
                    .map(|b| {
                        if b.is_ascii_graphic() || *b == b' ' {
                            *b as char
                        } else {
                            '.'
                        }
                    })
                    .collect::<String>();
                ui.monospace(format!(
                    "{:08X}  {:<47}  |{}|",
                    off, hex, ascii
                ));
            }
        });
}

fn draw_dcache(ui: &mut egui::Ui, app: &EmulatorApp) {
    let Some(dc) = app.dcache.as_ref() else {
        ui.colored_label(
            theme::MUTED,
            "D-cache snapshot not requested yet (phase 6 refresh path pending).",
        );
        return;
    };
    ui.label(format!("DC_CTRL raw: 0x{:08X}", dc.ctrl_raw));
    ui.separator();
    egui::ScrollArea::vertical()
        .id_salt("dcache")
        .max_height(ui.available_height())
        .show(ui, |ui| {
            for line in &dc.lines {
                if !line.valid {
                    continue;
                }
                ui.monospace(format!(
                    "set {:02} way {}  tag 0x{:08X}  base 0x{:08X} {}",
                    line.set,
                    line.way,
                    line.tag,
                    line.base_addr,
                    if line.dirty { "[dirty]" } else { "" },
                ));
            }
        });
}
