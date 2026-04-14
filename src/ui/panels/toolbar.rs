//! Top toolbar: grouped execution controls, firmware / annotation
//! menu, boot mode toggle, palette selector. Emits `CpuCommand`s
//! via `handle.cpu_cmd` and toggles `EmulatorApp` state directly.

use eframe::egui;
use egui_phosphor::regular as ph;

use crate::emu::command::CpuCommand;
use crate::emu::snapshot::RunState;
use crate::soc::bank::BootMode;
use crate::ui::theme::Palette;
use crate::ui::EmulatorApp;

pub fn draw(ui: &mut egui::Ui, app: &mut EmulatorApp) {
    ui.horizontal_centered(|ui| {
        ui.add_space(4.0);

        // Primary run/pause button — coloured wide button that
        // flips between green Run and red Pause depending on the
        // worker state. Gains a subtle opacity pulse when the
        // CPU is paused to draw the eye.
        let running = matches!(app.snapshot.run_state, RunState::Running);
        let (label, base_fill, stroke) = if running {
            (
                format!("{} Pause", ph::PAUSE),
                app.accents.danger,
                app.accents.danger,
            )
        } else {
            (
                format!("{} Run", ph::PLAY),
                app.accents.success,
                app.accents.success,
            )
        };

        let pulse = if running {
            0.35
        } else {
            let t = ui.input(|i| i.time) as f32;
            0.25 + ((t * 2.0).sin() * 0.5 + 0.5) * 0.25
        };
        let fill = base_fill.gamma_multiply(pulse);
        if !running {
            ui.ctx().request_repaint_after(std::time::Duration::from_millis(32));
        }
        let run_btn = ui.add(
            egui::Button::new(egui::RichText::new(label).strong())
                .min_size(egui::vec2(96.0, 28.0))
                .fill(fill)
                .stroke(egui::Stroke::new(1.5, stroke))
                .corner_radius(egui::CornerRadius::same(6)),
        );
        if run_btn.clicked() {
            let cmd = if running {
                CpuCommand::Pause
            } else {
                CpuCommand::Run { max_insns: None }
            };
            let _ = app.handle.cpu_cmd.send(cmd);
        }

        // Stepping group: single-step / step-over / run-to-cursor.
        group(ui, |ui| {
            if icon_button(ui, ph::STEPS, "Step one instruction").clicked() {
                let _ = app.handle.cpu_cmd.send(CpuCommand::StepOne);
            }
            if icon_button(ui, ph::SKIP_FORWARD, "Step over").clicked() {
                let _ = app.handle.cpu_cmd.send(CpuCommand::StepOver);
            }
            if icon_button(ui, ph::ARROW_LINE_RIGHT, "Run to cursor").clicked() {
                let _ = app.handle.cpu_cmd.send(CpuCommand::RunTo {
                    address: app.disasm_cursor,
                });
            }
            if icon_button(ui, ph::ARROW_COUNTER_CLOCKWISE, "Reset CPU").clicked() {
                let _ = app.handle.cpu_cmd.send(CpuCommand::Reset {
                    boot_mode: app.snapshot.boot_mode,
                    keep_breakpoints: true,
                });
            }
        });

        // Firmware + annotation group.
        group(ui, |ui| {
            if ui
                .button(format!("{} Firmware…", ph::FILE_ARROW_UP))
                .on_hover_text("Load a flash image into the PBC SPI flash peripheral")
                .clicked()
            {
                load_firmware(app);
            }
            if ui
                .button(format!("{} Load annot.", ph::BOOKMARKS))
                .on_hover_text("Load user annotations JSON")
                .clicked()
            {
                load_annotations(app);
            }
            if ui
                .button(format!("{} Save annot.", ph::FLOPPY_DISK))
                .on_hover_text("Save user annotations JSON")
                .clicked()
            {
                save_annotations(app);
            }
        });

        // Boot mode toggle.
        group(ui, |ui| {
            let mut mode = app.snapshot.boot_mode;
            let warm_changed = ui
                .radio_value(&mut mode, BootMode::Warm, "Warm")
                .changed();
            let cold_changed = ui
                .radio_value(&mut mode, BootMode::Cold, "Cold")
                .changed();
            if warm_changed || cold_changed {
                app.handle.snapshot.lock().boot_mode = mode;
            }
        });

        // Push remaining items to the right edge of the toolbar.
        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
            ui.add_space(4.0);
            palette_selector(ui, app);
            ui.separator();
            // Live status badge: PC + insns/sec + bank tick.
            let badge_text = format!(
                "PC 0x{:08X}   {:>6} ips",
                app.snapshot.cpu.pc, app.snapshot.insns_per_sec,
            );
            ui.label(
                egui::RichText::new(badge_text)
                    .monospace()
                    .color(app.accents.muted),
            );
        });
    });
}

/// Grouped-button frame with a subtle rounded background so the
/// toolbar reads as a row of sections rather than a blur of
/// individual buttons.
fn group<R>(ui: &mut egui::Ui, add_contents: impl FnOnce(&mut egui::Ui) -> R) {
    egui::Frame::default()
        .fill(ui.visuals().extreme_bg_color.gamma_multiply(0.45))
        .corner_radius(egui::CornerRadius::same(6))
        .inner_margin(egui::Margin::symmetric(6, 3))
        .show(ui, |ui| {
            ui.horizontal(|ui| {
                add_contents(ui);
            });
        });
    ui.add_space(4.0);
}

/// Icon-only button with hover tooltip. Uses Phosphor regular.
fn icon_button(ui: &mut egui::Ui, glyph: &str, tooltip: &str) -> egui::Response {
    ui.add(egui::Button::new(egui::RichText::new(glyph).size(16.0)).min_size(egui::vec2(28.0, 24.0)))
        .on_hover_text(tooltip)
}

fn palette_selector(ui: &mut egui::Ui, app: &mut EmulatorApp) {
    let current = app.palette;
    egui::ComboBox::from_id_salt("palette_selector")
        .selected_text(format!("{} {}", ph::PAINT_BRUSH, current.label()))
        .width(130.0)
        .show_ui(ui, |ui| {
            for p in Palette::ALL {
                let response = ui.selectable_label(p == current, p.label());
                if response.clicked() && p != current {
                    let ctx = ui.ctx().clone();
                    app.set_palette(&ctx, p);
                }
            }
        });
}

fn load_firmware(app: &mut EmulatorApp) {
    let Some(path) = rfd::FileDialog::new()
        .add_filter("BCM55030 flash", &["bin"])
        .pick_file()
    else {
        return;
    };
    let (tx, rx) = crate::emu::command::oneshot();
    let cmd = CpuCommand::LoadFirmware {
        path,
        mode: crate::emu::command::FirmwareMode::Soc,
        boot_mode: app.snapshot.boot_mode,
        flash_path: None,
        entry_point: 0,
        keep_breakpoints: true,
        response: tx,
    };
    if app.handle.cpu_cmd.send(cmd).is_ok() {
        match rx.recv_timeout(std::time::Duration::from_secs(5)) {
            Ok(Ok(r)) => eprintln!(
                "[ui] loaded firmware: {} bytes into {}-byte flash, entry 0x{:08X}",
                r.loaded_bytes, r.flash_bytes, r.entry_point
            ),
            Ok(Err(e)) => eprintln!("[ui] load_firmware failed: {e}"),
            Err(_) => eprintln!("[ui] load_firmware timed out"),
        }
    }
}

fn load_annotations(app: &EmulatorApp) {
    let Some(path) = rfd::FileDialog::new()
        .add_filter("json", &["json"])
        .pick_file()
    else {
        return;
    };
    match std::fs::read_to_string(&path) {
        Ok(body) => match crate::emu::annotations::Annotations::from_json_str(&body) {
            Ok(parsed) => *app.handle.annotations.write() = parsed,
            Err(e) => eprintln!("[UI] annotations parse failed: {e}"),
        },
        Err(e) => eprintln!("[UI] annotations read failed: {e}"),
    }
}

fn save_annotations(app: &EmulatorApp) {
    let Some(path) = rfd::FileDialog::new()
        .add_filter("json", &["json"])
        .set_file_name("annotations.json")
        .save_file()
    else {
        return;
    };
    let body = app.handle.annotations.read().to_json_string();
    if let Err(e) = std::fs::write(&path, body) {
        eprintln!("[UI] annotations write failed: {e}");
    }
}
