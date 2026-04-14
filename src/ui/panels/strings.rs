//! Strings panel: extract printable-ASCII strings (length ≥
//! `min_len`) from the SRAM or flash region and list them in a
//! scrollable, filterable table. Clicking an entry jumps to the
//! matching offset in the memory viewer.

use eframe::egui;
use egui_phosphor::regular as ph;

use crate::ui::app::{StringHit, StringsCache, StringsSource};
use crate::ui::EmulatorApp;

pub fn draw(ui: &mut egui::Ui, app: &mut EmulatorApp) {
    header(ui, app);
    ui.separator();

    // Rebuild the cache if the selected source / min_len has
    // changed — or if it's missing.
    let needs_rebuild = app
        .strings_cache
        .as_ref()
        .map(|c| c.source != app.strings_tab || c.min_len != app.strings_min_len)
        .unwrap_or(true);
    if needs_rebuild {
        rebuild_cache(app);
    }

    let accents = app.accents;
    let filter = app.strings_filter.to_lowercase();
    let Some(cache) = app.strings_cache.as_ref() else {
        ui.colored_label(accents.muted, "No strings available.");
        return;
    };

    let total = cache.entries.len();
    let matching: Vec<&StringHit> = cache
        .entries
        .iter()
        .filter(|s| {
            filter.is_empty() || s.content.to_lowercase().contains(&filter)
        })
        .collect();

    ui.label(
        egui::RichText::new(format!(
            "{} strings ({} match filter)",
            total,
            matching.len()
        ))
        .small()
        .color(accents.muted),
    );
    ui.separator();

    let row_height =
        egui::TextStyle::Monospace.resolve(ui.style()).size + 3.0;
    let mut reveal: Option<u32> = None;
    egui::ScrollArea::vertical()
        .id_salt("strings_scroll")
        .auto_shrink([false, false])
        .max_height(ui.available_height())
        .show_rows(ui, row_height, matching.len(), |ui, row_range| {
            ui.spacing_mut().item_spacing.y = 1.0;
            for idx in row_range {
                let hit = matching[idx];
                ui.horizontal(|ui| {
                    ui.label(
                        egui::RichText::new(format!("0x{:08X}", hit.addr))
                            .monospace()
                            .color(accents.muted),
                    );
                    ui.label(
                        egui::RichText::new(format!("[{:>3}]", hit.content.len()))
                            .small()
                            .monospace()
                            .color(accents.muted.gamma_multiply(0.7)),
                    );
                    let label = egui::Label::new(
                        egui::RichText::new(format!("\"{}\"", escape(&hit.content)))
                            .monospace()
                            .color(accents.success),
                    )
                    .sense(egui::Sense::click());
                    if ui
                        .add(label)
                        .on_hover_text("Click to reveal in the memory viewer")
                        .clicked()
                    {
                        reveal = Some(hit.addr);
                    }
                });
            }
        });

    if let Some(addr) = reveal {
        app.central_tab = crate::ui::panels::CentralTab::Memory;
        app.memory_tab = match app.strings_tab {
            StringsSource::Sram => crate::ui::panels::memory::Tab::Sram,
            StringsSource::Flash => crate::ui::panels::memory::Tab::Flash,
        };
        app.memory_cursor = addr;
        app.memory_cursor_dirty = true;
    }
}

fn header(ui: &mut egui::Ui, app: &mut EmulatorApp) {
    ui.horizontal(|ui| {
        ui.strong(format!("{} Strings", ph::TEXT_AA));
        ui.separator();
        ui.selectable_value(&mut app.strings_tab, StringsSource::Sram, "SRAM");
        ui.selectable_value(&mut app.strings_tab, StringsSource::Flash, "Flash");
        ui.separator();
        ui.label(
            egui::RichText::new("min length")
                .small()
                .color(app.accents.muted),
        );
        ui.add(
            egui::DragValue::new(&mut app.strings_min_len)
                .range(1..=64)
                .speed(1.0),
        );
        if ui
            .button(format!("{} Rescan", ph::ARROW_CLOCKWISE))
            .clicked()
        {
            rebuild_cache(app);
        }
        ui.separator();
        ui.add(
            egui::TextEdit::singleline(&mut app.strings_filter)
                .desired_width(180.0)
                .hint_text("filter…"),
        );
        if !app.strings_filter.is_empty()
            && ui.small_button(ph::X).clicked()
        {
            app.strings_filter.clear();
        }
    });
}

fn rebuild_cache(app: &mut EmulatorApp) {
    let min_len = app.strings_min_len.max(2);
    let source = app.strings_tab;
    let entries = match source {
        StringsSource::Sram => app
            .sram
            .as_ref()
            .map(|s| extract_strings(&s.bytes, min_len))
            .unwrap_or_default(),
        StringsSource::Flash => {
            let bytes = app.handle.bank.read().pbc.flash.data.clone();
            extract_strings(&bytes, min_len)
        }
    };
    app.strings_cache = Some(StringsCache {
        source,
        min_len,
        entries,
    });
}

/// Scan `bytes` for runs of printable ASCII (0x20..=0x7E) of
/// length `≥ min_len`. Terminators (`\0` or any non-printable
/// byte) split runs. Returns a flat vector of
/// `(offset, content)` hits.
fn extract_strings(bytes: &[u8], min_len: usize) -> Vec<StringHit> {
    let mut out = Vec::new();
    let mut run_start: Option<usize> = None;
    for (i, b) in bytes.iter().enumerate() {
        let printable = (0x20..=0x7E).contains(b);
        match (printable, run_start) {
            (true, None) => run_start = Some(i),
            (false, Some(start)) => {
                if i - start >= min_len {
                    let slice = &bytes[start..i];
                    if let Ok(s) = std::str::from_utf8(slice) {
                        out.push(StringHit {
                            addr: start as u32,
                            content: s.to_string(),
                        });
                    }
                }
                run_start = None;
            }
            _ => {}
        }
    }
    if let Some(start) = run_start {
        let i = bytes.len();
        if i - start >= min_len {
            if let Ok(s) = std::str::from_utf8(&bytes[start..i]) {
                out.push(StringHit {
                    addr: start as u32,
                    content: s.to_string(),
                });
            }
        }
    }
    out
}

fn escape(s: &str) -> String {
    let mut out = String::with_capacity(s.len());
    for c in s.chars() {
        match c {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            _ => out.push(c),
        }
    }
    out
}
