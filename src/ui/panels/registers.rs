//! Registers panel: core + aux sub-tabs, change highlighting.

use eframe::egui;

use crate::ui::theme;
use crate::ui::EmulatorApp;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Tab {
    Core,
    Aux,
}

pub fn draw(ui: &mut egui::Ui, app: &mut EmulatorApp) {
    ui.vertical(|ui| {
        ui.horizontal(|ui| {
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
    // Flags strip.
    ui.horizontal(|ui| {
        ui.label("Flags:");
        flag_pill(ui, "Z", app.snapshot.cpu.flags.z);
        flag_pill(ui, "N", app.snapshot.cpu.flags.n);
        flag_pill(ui, "C", app.snapshot.cpu.flags.c);
        flag_pill(ui, "V", app.snapshot.cpu.flags.v);
        flag_pill(ui, "E1", app.snapshot.cpu.flags.e1);
        flag_pill(ui, "E2", app.snapshot.cpu.flags.e2);
        flag_pill(ui, "U", app.snapshot.cpu.flags.u);
        flag_pill(ui, "H", app.snapshot.cpu.flags.h);
        flag_pill(ui, "L", app.snapshot.cpu.flags.l);
    });
    ui.label(format!("status32: 0x{:08X}", app.snapshot.cpu.flags.status32));
    ui.separator();

    // r0..r31 grid.
    egui::Grid::new("core_reg_grid")
        .num_columns(2)
        .spacing([8.0, 2.0])
        .show(ui, |ui| {
            for i in 0..32usize {
                let name = reg_name(i as u8);
                let val = app.snapshot.cpu.core_regs[i];
                let changed = app.changed_regs.contains(&(i as u8));
                register_cell(ui, &name, val, changed);
                if i % 2 == 1 {
                    ui.end_row();
                }
            }
        });
    ui.separator();

    egui::Grid::new("core_reg_extras")
        .num_columns(2)
        .spacing([8.0, 2.0])
        .show(ui, |ui| {
            let special = [
                ("pc", app.snapshot.cpu.pc),
                ("blink", app.snapshot.cpu.core_regs[31]),
                ("sp", app.snapshot.cpu.core_regs[28]),
                ("fp", app.snapshot.cpu.core_regs[27]),
                ("gp", app.snapshot.cpu.core_regs[26]),
                ("lp_count", app.snapshot.cpu.core_regs[60]),
            ];
            for (i, (name, val)) in special.iter().enumerate() {
                register_cell(ui, name, *val, false);
                if i % 2 == 1 {
                    ui.end_row();
                }
            }
        });
}

fn draw_aux(ui: &mut egui::Ui, app: &EmulatorApp) {
    let aux = &app.snapshot.cpu.aux;
    egui::Grid::new("aux_reg_grid")
        .num_columns(2)
        .spacing([8.0, 2.0])
        .show(ui, |ui| {
            aux_row(ui, "identity", aux.identity);
            aux_row(ui, "status32", app.snapshot.cpu.flags.status32);
            aux_row(ui, "lp_start", aux.lp_start);
            aux_row(ui, "lp_end", aux.lp_end);
            aux_row(ui, "int_vector_base", aux.int_vector_base);
            aux_row(ui, "ienable", aux.ienable);
            aux_row(ui, "ipending", aux.ipending);
            aux_row(ui, "count0", aux.count0);
            aux_row(ui, "control0", aux.control0);
            aux_row(ui, "limit0", aux.limit0);
            aux_row(ui, "count1", aux.count1);
            aux_row(ui, "control1", aux.control1);
            aux_row(ui, "limit1", aux.limit1);
            aux_row(ui, "timer1_irq", aux.timer1_irq);
            aux_row(ui, "dc_ctrl", aux.dc_ctrl);
        });
}

fn flag_pill(ui: &mut egui::Ui, label: &str, set: bool) {
    let color = if set {
        egui::Color32::from_rgb(40, 200, 80)
    } else {
        theme::MUTED
    };
    ui.colored_label(color, label);
}

fn register_cell(ui: &mut egui::Ui, name: &str, val: u32, changed: bool) {
    ui.horizontal(|ui| {
        ui.label(egui::RichText::new(format!("{:>9}", name)).monospace());
        let text = egui::RichText::new(format!("0x{:08X}", val)).monospace();
        if changed {
            ui.colored_label(theme::CHANGED_REG, text);
        } else {
            ui.label(text);
        }
    });
}

fn aux_row(ui: &mut egui::Ui, name: &str, val: u32) {
    ui.label(egui::RichText::new(format!("{:>16}", name)).monospace());
    ui.label(egui::RichText::new(format!("0x{:08X}", val)).monospace());
    ui.end_row();
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
