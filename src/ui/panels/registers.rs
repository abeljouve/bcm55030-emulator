//! Registers panel: Core / Aux tabs, each showing a grid of
//! card-style cells. Recently-changed registers get a coloured
//! border fade driven by `egui::Context::animate_value_with_time`.

use eframe::egui;
use egui_phosphor::regular as ph;

use crate::ui::EmulatorApp;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Tab {
    Core,
    Aux,
}

pub fn draw(ui: &mut egui::Ui, app: &mut EmulatorApp) {
    ui.vertical(|ui| {
        ui.horizontal(|ui| {
            ui.strong(format!("{} Registers", ph::CPU));
            ui.separator();
            ui.selectable_value(&mut app.registers_tab, Tab::Core, "Core");
            ui.selectable_value(&mut app.registers_tab, Tab::Aux, "Aux");
        });
        ui.separator();
        egui::ScrollArea::vertical()
            .id_salt("registers_scroll")
            .show(ui, |ui| match app.registers_tab {
                Tab::Core => draw_core(ui, app),
                Tab::Aux => draw_aux(ui, app),
            });
    });
}

fn draw_core(ui: &mut egui::Ui, app: &EmulatorApp) {
    // Flags strip: pill per STATUS32 bit.
    egui::Frame::default()
        .fill(ui.visuals().extreme_bg_color.gamma_multiply(0.45))
        .corner_radius(egui::CornerRadius::same(6))
        .inner_margin(egui::Margin::symmetric(8, 4))
        .show(ui, |ui| {
            ui.horizontal_wrapped(|ui| {
                ui.label(egui::RichText::new("STATUS32").small().color(app.accents.muted));
                ui.separator();
                flag_pill(ui, app, "Z", app.snapshot.cpu.flags.z);
                flag_pill(ui, app, "N", app.snapshot.cpu.flags.n);
                flag_pill(ui, app, "C", app.snapshot.cpu.flags.c);
                flag_pill(ui, app, "V", app.snapshot.cpu.flags.v);
                flag_pill(ui, app, "E1", app.snapshot.cpu.flags.e1);
                flag_pill(ui, app, "E2", app.snapshot.cpu.flags.e2);
                flag_pill(ui, app, "U", app.snapshot.cpu.flags.u);
                flag_pill(ui, app, "H", app.snapshot.cpu.flags.h);
                flag_pill(ui, app, "L", app.snapshot.cpu.flags.l);
                ui.separator();
                ui.label(
                    egui::RichText::new(format!(
                        "0x{:08X}",
                        app.snapshot.cpu.flags.status32
                    ))
                    .monospace()
                    .small()
                    .color(app.accents.muted),
                );
            });
        });
    ui.add_space(6.0);

    // r0..r31 as a 4-column card grid.
    let cols = 4usize;
    let card_w = (ui.available_width() / cols as f32).floor() - 6.0;
    egui::Grid::new("core_reg_grid")
        .num_columns(cols)
        .spacing([6.0, 6.0])
        .show(ui, |ui| {
            for i in 0..32usize {
                register_card(
                    ui,
                    app,
                    &reg_name(i as u8),
                    app.snapshot.cpu.core_regs[i],
                    app.changed_regs.contains(&(i as u8)),
                    card_w,
                );
                if i % cols == cols - 1 {
                    ui.end_row();
                }
            }
        });
    ui.add_space(6.0);

    // Special registers — always shown separately so they are easy to find.
    ui.label(egui::RichText::new("Special").small().color(app.accents.muted));
    egui::Grid::new("core_reg_extras")
        .num_columns(cols)
        .spacing([6.0, 6.0])
        .show(ui, |ui| {
            let special: [(&str, u32); 6] = [
                ("pc", app.snapshot.cpu.pc),
                ("blink", app.snapshot.cpu.core_regs[31]),
                ("sp", app.snapshot.cpu.core_regs[28]),
                ("fp", app.snapshot.cpu.core_regs[27]),
                ("gp", app.snapshot.cpu.core_regs[26]),
                ("lp_cnt", app.snapshot.cpu.core_regs[60]),
            ];
            for (idx, (name, val)) in special.iter().enumerate() {
                register_card(ui, app, name, *val, false, card_w);
                if idx % cols == cols - 1 {
                    ui.end_row();
                }
            }
        });
}

fn draw_aux(ui: &mut egui::Ui, app: &EmulatorApp) {
    let aux = &app.snapshot.cpu.aux;
    // No `ienable` row: aux 0x40C is unimplemented on this silicon
    // (DATASHEET §6.1). Showing an enable mask that does not exist is how a
    // reader concludes a line is disabled when the hardware will deliver it.
    let rows: [(&str, u32); 14] = [
        ("identity", aux.identity),
        ("status32", app.snapshot.cpu.flags.status32),
        ("lp_start", aux.lp_start),
        ("lp_end", aux.lp_end),
        ("int_vbase", aux.int_vector_base),
        ("ipending", aux.ipending),
        ("count0", aux.count0),
        ("control0", aux.control0),
        ("limit0", aux.limit0),
        ("count1", aux.count1),
        ("control1", aux.control1),
        ("limit1", aux.limit1),
        ("timer1_irq", aux.timer1_irq),
        ("dc_ctrl", aux.dc_ctrl),
    ];
    let cols = 2usize;
    let card_w = (ui.available_width() / cols as f32).floor() - 6.0;
    egui::Grid::new("aux_reg_grid")
        .num_columns(cols)
        .spacing([6.0, 6.0])
        .show(ui, |ui| {
            for (idx, (name, val)) in rows.iter().enumerate() {
                register_card(ui, app, name, *val, false, card_w);
                if idx % cols == cols - 1 {
                    ui.end_row();
                }
            }
        });
}

/// Paint one register "card": name on the first line, hex + dec +
/// ASCII on the second. Changed registers get an animated accent
/// border that fades over ~0.8 s thanks to
/// `Context::animate_value_with_time`.
fn register_card(
    ui: &mut egui::Ui,
    app: &EmulatorApp,
    name: &str,
    value: u32,
    changed: bool,
    width: f32,
) {
    let id = egui::Id::new(("reg_card", name));
    let target = if changed { 1.0 } else { 0.0 };
    let t = ui.ctx().animate_value_with_time(id, target, 0.8);
    let border = app
        .accents
        .changed_reg
        .gamma_multiply(t.clamp(0.0, 1.0));
    let bg = ui.visuals().widgets.noninteractive.bg_fill;
    let (rect, resp) = ui.allocate_exact_size(
        egui::vec2(width, 40.0),
        egui::Sense::click(),
    );
    let painter = ui.painter_at(rect);
    painter.rect_filled(rect, egui::CornerRadius::same(6), bg);
    if t > 0.01 {
        painter.rect_stroke(
            rect,
            egui::CornerRadius::same(6),
            egui::Stroke::new(1.5, border),
            egui::StrokeKind::Inside,
        );
    } else {
        painter.rect_stroke(
            rect,
            egui::CornerRadius::same(6),
            egui::Stroke::new(1.0, app.accents.muted.gamma_multiply(0.4)),
            egui::StrokeKind::Inside,
        );
    }

    // First row: name.
    let name_pos = rect.left_top() + egui::vec2(8.0, 4.0);
    painter.text(
        name_pos,
        egui::Align2::LEFT_TOP,
        name,
        egui::FontId::proportional(11.0),
        app.accents.muted,
    );

    // Second row: hex value + decimal in brackets + ASCII.
    let hex = format!("0x{value:08X}");
    let dec = format!("{value}");
    let ascii = value_to_ascii(value);
    let hex_pos = rect.left_bottom() + egui::vec2(8.0, -18.0);
    painter.text(
        hex_pos,
        egui::Align2::LEFT_TOP,
        &hex,
        egui::FontId::monospace(13.0),
        ui.visuals().text_color(),
    );
    let dec_pos = rect.right_bottom() + egui::vec2(-8.0, -18.0);
    painter.text(
        dec_pos,
        egui::Align2::RIGHT_TOP,
        format!("{dec}  {ascii}"),
        egui::FontId::proportional(10.0),
        app.accents.muted,
    );

    if resp.clicked() {
        ui.ctx().copy_text(hex);
    }
    resp.on_hover_text(format!("{name} = {value} (0x{value:08X}) — click to copy"));
}

fn flag_pill(ui: &mut egui::Ui, app: &EmulatorApp, label: &str, set: bool) {
    let color = if set {
        app.accents.success
    } else {
        app.accents.muted
    };
    egui::Frame::default()
        .fill(color.gamma_multiply(if set { 0.25 } else { 0.1 }))
        .stroke(egui::Stroke::new(1.0, color))
        .corner_radius(egui::CornerRadius::same(4))
        .inner_margin(egui::Margin::symmetric(4, 1))
        .show(ui, |ui| {
            ui.label(
                egui::RichText::new(label)
                    .small()
                    .monospace()
                    .strong()
                    .color(color),
            );
        });
}

fn reg_name(idx: u8) -> String {
    match idx {
        26 => "gp".into(),
        27 => "fp".into(),
        28 => "sp".into(),
        29 => "ilink1".into(),
        30 => "ilink2".into(),
        31 => "blink".into(),
        _ => format!("r{}", idx),
    }
}

fn value_to_ascii(v: u32) -> String {
    let bytes = v.to_be_bytes();
    let mut s = String::with_capacity(4);
    for b in bytes {
        if b.is_ascii_graphic() || b == b' ' {
            s.push(b as char);
        } else {
            s.push('·');
        }
    }
    s
}
