//! Packet view for the EPON link: a list of frames, the dissection of the
//! selected one, and its bytes with the selected field highlighted.

use eframe::egui;

use crate::soc::olt::decode::{self, Dissection, Field};
use crate::ui::theme;
use crate::ui::EmulatorApp;

/// A frame lifted out of the model, ready to render.
struct Row {
    /// Downstream frames go to the ONU; upstream come from it.
    downstream: bool,
    /// Instant on the link clock, which is what the peer's timers run on.
    at: epon_olt::WireInstant,
    data: Vec<u8>,
    label: String,
    dissection: Dissection,
}

/// Selection and filter state, owned by the app.
#[derive(Default)]
pub struct PacketScratch {
    pub selected: Option<usize>,
    pub selected_field: Option<usize>,
    pub filter: String,
    pub follow: bool,
}

pub fn draw(ui: &mut egui::Ui, app: &mut EmulatorApp) {
    let rows = collect(app);
    let filter = app.packet_scratch.filter.to_ascii_lowercase();
    let visible: Vec<usize> = rows
        .iter()
        .enumerate()
        .filter(|(_, r)| matches(r, &filter))
        .map(|(i, _)| i)
        .collect();

    ui.horizontal(|ui| {
        ui.label(format!("{} frames", rows.len()));
        if visible.len() != rows.len() {
            ui.label(theme_muted(format!("{} shown", visible.len())));
        }
        ui.separator();
        ui.label("Filter");
        ui.add(
            egui::TextEdit::singleline(&mut app.packet_scratch.filter)
                .hint_text("mpcp, oam, register…")
                .desired_width(160.0),
        );
        ui.checkbox(&mut app.packet_scratch.follow, "Follow");
        if ui.button("Clear selection").clicked() {
            app.packet_scratch.selected = None;
            app.packet_scratch.selected_field = None;
        }
    });
    ui.separator();

    if rows.is_empty() {
        ui.label(theme_muted(
            "No frames yet. They appear once the link comes up.",
        ));
        return;
    }

    if app.packet_scratch.follow {
        app.packet_scratch.selected = visible.last().copied();
    }

    let list_height = (ui.available_height() * 0.45).max(120.0);
    draw_list(ui, app, &rows, &visible, list_height);
    ui.separator();

    let Some(sel) = app.packet_scratch.selected.filter(|i| *i < rows.len()) else {
        ui.label(theme_muted("Select a frame to see its dissection."));
        return;
    };
    let row = &rows[sel];

    let remaining = ui.available_height();
    egui::Panel::left("packet_tree")
        .resizable(true)
        .default_size(ui.available_width() * 0.55)
        .show_inside(ui, |ui| draw_tree(ui, app, row, remaining));
    egui::CentralPanel::default().show_inside(ui, |ui| draw_bytes(ui, app, row));
}

fn collect(app: &EmulatorApp) -> Vec<Row> {
    let bank = app.handle.bank.read();
    let olt = &bank.olt;
    let mut rows: Vec<Row> = olt
        .tx_log()
        .iter()
        .map(|f| (false, f))
        .chain(olt.rx_log().iter().map(|f| (true, f)))
        .map(|(downstream, f)| Row {
            downstream,
            at: f.at,
            data: f.data.clone(),
            label: f.description.clone(),
            dissection: decode::dissect(&f.data),
        })
        .collect();
    rows.sort_by_key(|r| r.at);
    rows
}

fn matches(row: &Row, filter: &str) -> bool {
    if filter.is_empty() {
        return true;
    }
    let d = &row.dissection;
    d.protocol.to_ascii_lowercase().contains(filter)
        || d.summary.to_ascii_lowercase().contains(filter)
        || row.label.to_ascii_lowercase().contains(filter)
}

fn draw_list(
    ui: &mut egui::Ui,
    app: &mut EmulatorApp,
    rows: &[Row],
    visible: &[usize],
    height: f32,
) {
    egui::ScrollArea::vertical()
        .id_salt("packet_list")
        .max_height(height)
        .stick_to_bottom(app.packet_scratch.follow)
        .show(ui, |ui| {
            egui::Grid::new("packet_grid")
                .num_columns(6)
                .striped(true)
                .spacing([12.0, 2.0])
                .show(ui, |ui| {
                    ui.label(theme_muted("Link time"));
                    ui.label(theme_muted("Dir"));
                    ui.label(theme_muted("Source"));
                    ui.label(theme_muted("Destination"));
                    ui.label(theme_muted("Protocol"));
                    ui.label(theme_muted("Summary"));
                    ui.end_row();

                    for &i in visible {
                        let row = &rows[i];
                        let selected = app.packet_scratch.selected == Some(i);
                        // Direction reads at a glance: downstream is the
                        // peer talking, upstream is the firmware answering.
                        let (arrow, colour) = if row.downstream {
                            ("↓", theme::MUTED)
                        } else {
                            ("↑", theme::TERMINAL_FG)
                        };
                        let mut clicked = ui
                            .selectable_label(selected, format!("{:.3}", row.at.as_ps() as f64 / 1e9))
                            .clicked();
                        clicked |= ui
                            .selectable_label(selected, egui::RichText::new(arrow).color(colour))
                            .clicked();
                        clicked |= ui.selectable_label(selected, &row.dissection.src).clicked();
                        clicked |= ui.selectable_label(selected, &row.dissection.dst).clicked();
                        clicked |= ui
                            .selectable_label(
                                selected,
                                egui::RichText::new(&row.dissection.protocol).color(colour),
                            )
                            .clicked();
                        clicked |= ui
                            .selectable_label(selected, &row.dissection.summary)
                            .clicked();
                        if clicked {
                            app.packet_scratch.selected = Some(i);
                            app.packet_scratch.selected_field = None;
                            app.packet_scratch.follow = false;
                        }
                        ui.end_row();
                    }
                });
        });
}

fn draw_tree(ui: &mut egui::Ui, app: &mut EmulatorApp, row: &Row, height: f32) {
    ui.label(egui::RichText::new(&row.label).color(theme::MUTATION));
    egui::ScrollArea::vertical()
        .id_salt("packet_tree_scroll")
        .max_height(height)
        .show(ui, |ui| {
            for (i, f) in row.dissection.fields.iter().enumerate() {
                let selected = app.packet_scratch.selected_field == Some(i);
                let indent = "    ".repeat(f.depth as usize);
                let text = if f.value.is_empty() {
                    format!("{indent}{}", f.name)
                } else {
                    format!("{indent}{}: {}", f.name, f.value)
                };
                let mut label = egui::RichText::new(text).monospace();
                if f.depth == 0 {
                    label = label.color(theme::MUTATION).strong();
                }
                if ui.selectable_label(selected, label).clicked() {
                    app.packet_scratch.selected_field = Some(i);
                }
            }
        });
}

fn draw_bytes(ui: &mut egui::Ui, app: &EmulatorApp, row: &Row) {
    let highlight = app
        .packet_scratch
        .selected_field
        .and_then(|i| row.dissection.fields.get(i))
        .and_then(field_range);

    egui::Frame::default().fill(theme::TERMINAL_BG).show(ui, |ui| {
        egui::ScrollArea::vertical()
            .id_salt("packet_bytes")
            .show(ui, |ui| {
                for (line, chunk) in row.data.chunks(16).enumerate() {
                    let base = line * 16;
                    ui.horizontal(|ui| {
                        ui.spacing_mut().item_spacing.x = 4.0;
                        ui.label(
                            egui::RichText::new(format!("{base:04X}"))
                                .monospace()
                                .color(theme::MUTED),
                        );
                        for (i, b) in chunk.iter().enumerate() {
                            let inside = highlight
                                .as_ref()
                                .is_some_and(|r| r.contains(&(base + i)));
                            let mut t = egui::RichText::new(format!("{b:02X}")).monospace();
                            t = if inside {
                                t.color(theme::TERMINAL_BG).background_color(theme::MUTATION)
                            } else {
                                t.color(theme::TERMINAL_FG)
                            };
                            ui.label(t);
                        }
                        ui.label(
                            egui::RichText::new(ascii(chunk))
                                .monospace()
                                .color(theme::MUTED),
                        );
                    });
                }
            });
    });
}

fn field_range(f: &Field) -> Option<std::ops::Range<usize>> {
    let off = f.offset?;
    (f.len > 0).then(|| off..off + f.len)
}

fn ascii(bytes: &[u8]) -> String {
    bytes
        .iter()
        .map(|&b| if (0x20..0x7F).contains(&b) { b as char } else { '.' })
        .collect()
}

fn theme_muted(text: impl Into<String>) -> egui::RichText {
    egui::RichText::new(text.into()).color(theme::MUTED)
}
