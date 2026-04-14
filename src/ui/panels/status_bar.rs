//! Bottom status bar: run state pill, PC badge, insn badge,
//! IPS sparkline, delay-slot / LP dots, boot mode, MCP status.
//! Badges are clickable — left-click copies the rendered text to
//! the system clipboard.

use eframe::egui;
use egui_phosphor::regular as ph;

use crate::cpu::registers::DelayState;
use crate::emu::snapshot::RunState;
use crate::ui::EmulatorApp;

pub fn draw(ui: &mut egui::Ui, app: &mut EmulatorApp) {
    ui.horizontal_centered(|ui| {
        ui.add_space(6.0);

        // Run-state pill with tinted background.
        let (label, glyph, color) = match app.snapshot.run_state {
            RunState::Running => ("Running", ph::PLAY, app.accents.success),
            RunState::Paused => ("Paused", ph::PAUSE, app.accents.warning),
            RunState::Halted => ("Halted", ph::STOP_CIRCLE, app.accents.danger),
            RunState::Sleeping => ("Sleeping", ph::MOON, app.accents.muted),
            RunState::Breakpoint => ("Breakpoint", ph::CIRCLE, app.accents.danger),
        };
        pill(ui, &format!("{glyph} {label}"), color);

        ui.add_space(6.0);

        // PC badge (click → copy).
        let pc_text = format!("PC 0x{:08X}", app.snapshot.cpu.pc);
        if badge(ui, &pc_text, app.accents.muted).clicked() {
            ui.ctx().copy_text(format!("0x{:08X}", app.snapshot.cpu.pc));
        }

        let insn_text = format!("insn {}", app.snapshot.cpu.instruction_count);
        if badge(ui, &insn_text, app.accents.muted).clicked() {
            ui.ctx()
                .copy_text(app.snapshot.cpu.instruction_count.to_string());
        }

        // IPS number + inline sparkline.
        let ips_text = format!("{} ips", app.snapshot.insns_per_sec);
        badge(ui, &ips_text, app.accents.muted);
        sparkline(ui, &app.ips_history, app.accents.accent);

        ui.add_space(6.0);

        // Delay slot / LP range dots.
        let delay_active = !matches!(app.snapshot.cpu.delay_state, DelayState::None);
        dot(ui, "DS", delay_active, app.accents.accent, app.accents.muted);
        let lp_active = app.snapshot.cpu.aux.lp_start != 0
            && app.snapshot.cpu.pc >= app.snapshot.cpu.aux.lp_start
            && app.snapshot.cpu.pc <= app.snapshot.cpu.aux.lp_end;
        dot(ui, "LP", lp_active, app.accents.success, app.accents.muted);

        // Push MCP status + boot mode to the right edge.
        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
            ui.add_space(6.0);
            let mcp_label = match app.handle.mcp_status.lock().listening.as_deref() {
                Some(addr) => format!("{} MCP {addr}", ph::PLUGS_CONNECTED),
                None => format!("{} MCP off", ph::PLUGS),
            };
            ui.label(egui::RichText::new(mcp_label).small().color(app.accents.muted));
            ui.separator();
            ui.label(
                egui::RichText::new(format!("{:?}", app.snapshot.boot_mode))
                    .small()
                    .color(app.accents.muted),
            );
        });
    });
}

/// Small coloured pill with an icon + label.
fn pill(ui: &mut egui::Ui, text: &str, color: egui::Color32) {
    egui::Frame::default()
        .fill(color.gamma_multiply(0.22))
        .stroke(egui::Stroke::new(1.0, color))
        .corner_radius(egui::CornerRadius::same(10))
        .inner_margin(egui::Margin::symmetric(8, 2))
        .show(ui, |ui| {
            ui.label(
                egui::RichText::new(text)
                    .small()
                    .strong()
                    .color(color),
            );
        });
}

/// Monospace numeric badge with muted background.
fn badge(ui: &mut egui::Ui, text: &str, color: egui::Color32) -> egui::Response {
    let frame_color = ui.visuals().extreme_bg_color.gamma_multiply(0.45);
    egui::Frame::default()
        .fill(frame_color)
        .corner_radius(egui::CornerRadius::same(4))
        .inner_margin(egui::Margin::symmetric(6, 2))
        .show(ui, |ui| {
            ui.label(
                egui::RichText::new(text)
                    .small()
                    .monospace()
                    .color(color),
            )
            .on_hover_text("Click to copy")
        })
        .inner
}

/// Filled / hollow dot indicator (delay slot, loop range).
fn dot(ui: &mut egui::Ui, label: &str, active: bool, on: egui::Color32, off: egui::Color32) {
    let color = if active { on } else { off };
    ui.horizontal(|ui| {
        let (rect, _) = ui.allocate_exact_size(egui::vec2(8.0, 8.0), egui::Sense::hover());
        let painter = ui.painter();
        if active {
            painter.circle_filled(rect.center(), 4.0, color);
        } else {
            painter.circle_stroke(rect.center(), 4.0, egui::Stroke::new(1.0, color));
        }
        ui.label(egui::RichText::new(label).small().color(color));
    });
}

/// Tiny inline sparkline rendered from the IPS ring buffer.
fn sparkline(
    ui: &mut egui::Ui,
    samples: &std::collections::VecDeque<u32>,
    color: egui::Color32,
) {
    let width = 80.0;
    let height = 14.0;
    let (rect, _) =
        ui.allocate_exact_size(egui::vec2(width, height), egui::Sense::hover());
    let painter = ui.painter();
    let bg = ui.visuals().extreme_bg_color.gamma_multiply(0.6);
    painter.rect_filled(rect, egui::CornerRadius::same(3), bg);
    if samples.len() < 2 {
        return;
    }
    let max = samples.iter().copied().max().unwrap_or(1).max(1) as f32;
    let step = width / (samples.len() as f32 - 1.0);
    let mut points: Vec<egui::Pos2> = Vec::with_capacity(samples.len());
    for (i, s) in samples.iter().enumerate() {
        let x = rect.left() + i as f32 * step;
        let y = rect.bottom() - (*s as f32 / max) * (height - 2.0) - 1.0;
        points.push(egui::pos2(x, y));
    }
    painter.add(egui::Shape::line(points, egui::Stroke::new(1.2, color)));
}
