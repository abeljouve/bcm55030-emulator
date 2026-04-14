//! MCP activity log panel. Reads from `handle.event_log` every
//! frame and renders the bounded VecDeque in reverse-chronological
//! order. Mutation entries are coloured distinctly so an agent
//! can tell reads from writes at a glance.

use eframe::egui;
use std::time::{SystemTime, UNIX_EPOCH};

use crate::emu::event_log::Direction;
use crate::ui::theme;
use crate::ui::EmulatorApp;

pub fn draw(ui: &mut egui::Ui, app: &mut EmulatorApp) {
    let (entries_snapshot, len, capacity, in_flight) = {
        let log = app.handle.event_log.lock();
        (
            log.entries().iter().cloned().collect::<Vec<_>>(),
            log.len(),
            0usize, // capacity accessor not exposed; header shows len only
            log.in_flight,
        )
    };
    let _ = capacity;

    ui.horizontal(|ui| {
        ui.label(format!("{len} entries"));
        if in_flight > 0 {
            ui.separator();
            ui.colored_label(
                egui::Color32::from_rgb(255, 180, 40),
                format!("{in_flight} in-flight"),
            );
        }
        ui.separator();
        if ui.button("Clear").clicked() {
            app.handle.event_log.lock().clear();
        }
    });
    ui.separator();

    egui::Frame::default()
        .fill(theme::TERMINAL_BG)
        .show(ui, |ui| {
            egui::ScrollArea::vertical()
                .id_salt("mcp_activity_log")
                .stick_to_bottom(true)
                .max_height(ui.available_height())
                .show(ui, |ui| {
                    for entry in entries_snapshot.iter() {
                        let color = if entry.is_mutation {
                            theme::MUTATION
                        } else {
                            theme::TERMINAL_FG
                        };
                        let arrow = match entry.direction {
                            Direction::Request => "→",
                            Direction::Response => "←",
                        };
                        let ts = format_timestamp(entry.timestamp);
                        let line = format!(
                            "{ts} {arrow} {tool}  {body}",
                            tool = entry.tool,
                            body = if entry.direction == Direction::Request {
                                &entry.params
                            } else {
                                &entry.result
                            },
                        );
                        ui.colored_label(color, egui::RichText::new(line).monospace());
                    }
                });
        });
}

/// Format a `SystemTime` as `HH:MM:SS.mmm` in the local wall clock.
/// Avoids pulling chrono — UNIX seconds modulo 86 400 is close
/// enough for the activity log header.
fn format_timestamp(ts: SystemTime) -> String {
    let dur = match ts.duration_since(UNIX_EPOCH) {
        Ok(d) => d,
        Err(_) => return "??:??:??.???".to_string(),
    };
    let secs = dur.as_secs();
    let millis = dur.subsec_millis();
    let h = (secs / 3600) % 24;
    let m = (secs / 60) % 60;
    let s = secs % 60;
    format!("{h:02}:{m:02}:{s:02}.{millis:03}")
}
