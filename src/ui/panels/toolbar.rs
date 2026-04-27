//! Top toolbar: grouped execution controls, firmware / annotation
//! menu, boot mode toggle, palette selector. Emits `CpuCommand`s
//! via `handle.cpu_cmd` and toggles `EmulatorApp` state directly.

use eframe::egui;
use egui_phosphor::regular as ph;

use crate::emu::command::{CpuCommand, SpeedLimit};
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

        // Speed control: log-scale slider + unlimited toggle.
        group(ui, |ui| {
            speed_control(ui, app);
        });

        // Firmware group.
        group(ui, |ui| {
            if ui
                .button(format!("{} Load", ph::FILE_ARROW_UP))
                .on_hover_text("Load a flash image into the PBC SPI flash peripheral (Ctrl-L)")
                .clicked()
            {
                load_firmware(app);
            }
            recent_firmwares_menu(ui, app);

            let (has_fw, is_dirty, dirty_bytes) = {
                let fi = app.handle.firmware_info.lock();
                let (d, cnt) = {
                    let guard = app.handle.bank.read();
                    let flash = &guard.pbc.flash;
                    let count = match flash.baseline.as_ref() {
                        Some(base) => flash
                            .data
                            .iter()
                            .zip(base.iter())
                            .filter(|(a, b)| a != b)
                            .count(),
                        None => 0,
                    };
                    (flash.dirty || count > 0, count)
                };
                (fi.is_some(), d, cnt)
            };

            let persist_label = if is_dirty {
                format!("{} Persist*", ph::FLOPPY_DISK)
            } else {
                format!("{} Persist", ph::FLOPPY_DISK)
            };
            let persist_btn = ui
                .add_enabled(has_fw, egui::Button::new(persist_label))
                .on_hover_text(
                    "Write the modified flash to <firmware>.persist next \
                     to the loaded image (same as the CLI --persist-flash flag)",
                );
            if persist_btn.clicked() {
                persist_flash(app);
            }
            if ui
                .add_enabled(
                    has_fw,
                    egui::Button::new(format!("{} Save as…", ph::EXPORT)),
                )
                .on_hover_text("Save the current flash to a file of your choosing")
                .clicked()
            {
                save_flash_as(app);
            }

            if is_dirty {
                ui.label(
                    egui::RichText::new(format!("{dirty_bytes} B dirty"))
                        .small()
                        .color(app.accents.warning),
                );
            }
        });

        // Annotations + session group.
        group(ui, |ui| {
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
            ui.separator();
            if ui
                .button(format!("{} Load session", ph::FOLDER_OPEN))
                .on_hover_text(
                    "Restore breakpoints, watchpoints, annotations, \
                     view state and palette from a .arc700-session.json",
                )
                .clicked()
            {
                load_session(app);
            }
            if ui
                .button(format!("{} Save session", ph::FLOPPY_DISK_BACK))
                .on_hover_text("Save the current session to a JSON file")
                .clicked()
            {
                save_session(app);
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

/// Execution-speed control group. Shows a log-scale slider
/// (1 kips → 50 Mips) plus an "∞" toggle for the default
/// `Unlimited` setting. The slider stores its live value in
/// `EmulatorApp.speed_slider_log10` so drag-gestures stay smooth
/// across frames; the actual dispatch to the worker happens on
/// drag release (`drag_stopped`) or when the `∞` button flips.
fn speed_control(ui: &mut egui::Ui, app: &mut EmulatorApp) {
    ui.label(
        egui::RichText::new(format!("{} Speed", ph::GAUGE))
            .small()
            .color(app.accents.muted),
    );

    // Sync slider log from worker state if the user hasn't been
    // dragging this frame.
    if !app.speed_slider_dragging {
        app.speed_slider_log10 = match app.snapshot.speed_limit {
            SpeedLimit::Unlimited => SPEED_LOG_MAX,
            SpeedLimit::Ips(n) => (n.max(1) as f32).log10(),
        };
    }

    let min = SPEED_LOG_MIN;
    let max = SPEED_LOG_MAX;
    let resp = ui.add(
        egui::Slider::new(&mut app.speed_slider_log10, min..=max)
            .show_value(false)
            .handle_shape(egui::style::HandleShape::Circle)
            .trailing_fill(true),
    );
    if resp.dragged() {
        app.speed_slider_dragging = true;
    }
    if resp.drag_stopped() || resp.changed() && !resp.dragged() {
        let target = slider_to_limit(app.speed_slider_log10);
        let _ = app.handle.cpu_cmd.send(CpuCommand::SetSpeed { limit: target });
        app.speed_slider_dragging = false;
    }

    // Live label: follow the slider while dragging, worker state
    // otherwise. Keeps the readout in sync with the handle under
    // the user's finger.
    let label_limit = if app.speed_slider_dragging {
        slider_to_limit(app.speed_slider_log10)
    } else {
        app.snapshot.speed_limit
    };
    ui.label(
        egui::RichText::new(format_speed(label_limit))
            .small()
            .monospace()
            .color(app.accents.muted),
    );

    let is_unlimited = matches!(app.snapshot.speed_limit, SpeedLimit::Unlimited);
    if ui
        .add(
            egui::Button::new(if is_unlimited { "∞" } else { "Max" })
                .min_size(egui::vec2(28.0, 20.0))
                .selected(is_unlimited),
        )
        .on_hover_text("Remove the speed cap")
        .clicked()
    {
        let _ = app
            .handle
            .cpu_cmd
            .send(CpuCommand::SetSpeed { limit: SpeedLimit::Unlimited });
        app.speed_slider_log10 = SPEED_LOG_MAX;
    }
}

/// Slider bounds — log10 of the desired ips. 0.0 = 1 ips,
/// 8.0 = 100 Mips, and the top slot represents "unlimited".
const SPEED_LOG_MIN: f32 = 0.0;
const SPEED_LOG_MAX: f32 = 8.0;

fn slider_to_limit(log10: f32) -> SpeedLimit {
    if log10 >= SPEED_LOG_MAX - 0.05 {
        return SpeedLimit::Unlimited;
    }
    let ips = 10f32.powf(log10).round() as u32;
    SpeedLimit::Ips(ips.max(1))
}

fn format_speed(limit: SpeedLimit) -> String {
    match limit {
        SpeedLimit::Unlimited => "∞ ips".to_string(),
        SpeedLimit::Ips(n) if n >= 1_000_000 => format!("{:.1} Mips", n as f32 / 1e6),
        SpeedLimit::Ips(n) if n >= 1_000 => format!("{:.1} kips", n as f32 / 1e3),
        SpeedLimit::Ips(n) => format!("{n} ips"),
    }
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
    if let Some(path) = rfd::FileDialog::new()
        .add_filter("BCM55030 flash", &["bin"])
        .pick_file()
    {
        app.load_firmware_path(path);
    }
}

/// Dropdown menu showing up to 8 recently loaded firmwares.
/// Disabled when the list is empty.
fn recent_firmwares_menu(ui: &mut egui::Ui, app: &mut EmulatorApp) {
    let recents = app.recent_firmwares.clone();
    let empty = recents.is_empty();
    let label = format!("{}", ph::CLOCK_COUNTER_CLOCKWISE);
    ui.add_enabled_ui(!empty, |ui| {
        egui::ComboBox::from_id_salt("recent_firmwares")
            .selected_text(label)
            .width(28.0)
            .show_ui(ui, |ui| {
                for path in recents {
                    let display = path
                        .file_name()
                        .map(|n| n.to_string_lossy().into_owned())
                        .unwrap_or_else(|| path.to_string_lossy().into_owned());
                    if ui.selectable_label(false, display).clicked() {
                        app.load_firmware_path(path);
                    }
                }
            })
            .response
            .on_hover_text("Recently loaded firmwares");
    });
}

/// Write the current flash contents to `<firmware>.persist`
/// alongside the originally-loaded image. Mirrors the CLI
/// `--persist-flash` on-exit path. Refreshes the in-memory
/// baseline so follow-up writes are measured from the persisted
/// image.
fn persist_flash(app: &EmulatorApp) {
    let Some(info) = app.handle.firmware_info.lock().clone() else {
        eprintln!("[ui] persist_flash: no firmware loaded");
        return;
    };
    let mut path = info.path.clone();
    let file_name = path
        .file_name()
        .map(|n| n.to_os_string())
        .unwrap_or_default();
    let mut persist_name = file_name;
    persist_name.push(".persist");
    path.set_file_name(persist_name);

    let data = {
        let mut guard = app.handle.bank.write();
        let snapshot = guard.pbc.flash.data.clone();
        // Update the baseline so the memory viewer clears its
        // highlights after the save.
        guard.pbc.flash.baseline = Some(snapshot.clone());
        guard.pbc.flash.dirty = false;
        snapshot
    };
    match std::fs::write(&path, &data) {
        Ok(_) => eprintln!(
            "[ui] flash persisted to {} ({} bytes)",
            path.display(),
            data.len()
        ),
        Err(e) => eprintln!("[ui] persist_flash failed: {e}"),
    }
}

/// Save the current flash to a user-chosen file via rfd. Does
/// not touch the baseline — use "Persist" for the drop-in
/// roundtrip that CLI users expect.
fn save_flash_as(app: &EmulatorApp) {
    let Some(path) = rfd::FileDialog::new()
        .add_filter("BCM55030 flash", &["bin"])
        .set_file_name("flash.bin")
        .save_file()
    else {
        return;
    };
    let data = app.handle.bank.read().pbc.flash.data.clone();
    match std::fs::write(&path, &data) {
        Ok(_) => eprintln!(
            "[ui] flash saved to {} ({} bytes)",
            path.display(),
            data.len()
        ),
        Err(e) => eprintln!("[ui] save_flash_as failed: {e}"),
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

/// Collect the current app state into a `Session` and write it
/// through a save dialog. Watchpoints come from the live
/// snapshot; annotations from `handle.annotations`.
fn save_session(app: &EmulatorApp) {
    let Some(path) = rfd::FileDialog::new()
        .add_filter("json", &["json"])
        .set_file_name("session.arc700.json")
        .save_file()
    else {
        return;
    };

    let firmware_path = app
        .handle
        .firmware_info
        .lock()
        .as_ref()
        .map(|f| f.path.clone());
    let annotations = app.handle.annotations.read().clone();

    let session = crate::emu::session::Session {
        firmware_path,
        palette: Some(app.palette.as_str().to_string()),
        breakpoints: app.snapshot.breakpoints.clone(),
        watchpoints: app.snapshot.watchpoints.clone(),
        symbols: annotations.symbols.clone(),
        comments: annotations.comments.clone(),
        regions: annotations.regions.clone(),
        disasm_view_base: Some(app.disasm_view_base),
        disasm_follow_pc: Some(app.disasm_follow_pc),
        memory_cursor: Some(app.memory_cursor),
    };

    match std::fs::write(&path, session.to_json_string()) {
        Ok(_) => eprintln!("[ui] session saved to {}", path.display()),
        Err(e) => eprintln!("[ui] session save failed: {e}"),
    }
}

/// Read a session JSON and replay it through `cpu_cmd`s and the
/// handle's annotations. Loads the firmware first (if the path
/// still exists), then applies breakpoints, watchpoints, and
/// finally view state.
fn load_session(app: &mut EmulatorApp) {
    let Some(path) = rfd::FileDialog::new()
        .add_filter("json", &["json"])
        .pick_file()
    else {
        return;
    };
    let body = match std::fs::read_to_string(&path) {
        Ok(b) => b,
        Err(e) => {
            eprintln!("[ui] session read failed: {e}");
            return;
        }
    };
    let session = match crate::emu::session::Session::from_json_str(&body) {
        Ok(s) => s,
        Err(e) => {
            eprintln!("[ui] session parse failed: {e}");
            return;
        }
    };

    // Firmware first — the worker rebuilds its Cpu which also
    // wipes the breakpoint table.
    if let Some(fw_path) = session.firmware_path.clone() {
        if fw_path.exists() {
            app.load_firmware_path(fw_path);
        } else {
            eprintln!(
                "[ui] session firmware not found: {}",
                fw_path.display()
            );
        }
    }

    // Palette.
    if let Some(palette_name) = session.palette.as_deref() {
        if let Some(p) = crate::ui::theme::Palette::from_str(palette_name) {
            app.palette = p;
            app.accents = crate::ui::theme::AccentTokens::from_palette(p);
        }
    }

    // Annotations — replace wholesale.
    {
        let mut guard = app.handle.annotations.write();
        guard.symbols = session.symbols.clone();
        guard.comments = session.comments.clone();
        guard.regions = session.regions.clone();
    }

    // Breakpoints.
    for addr in &session.breakpoints {
        let (tx, _rx) = crate::emu::command::oneshot::<usize>();
        let _ = app
            .handle
            .cpu_cmd
            .send(CpuCommand::SetBreakpoint { address: *addr, response: tx });
    }

    // Watchpoints.
    for wp in &session.watchpoints {
        let (tx, _rx) = crate::emu::command::oneshot::<usize>();
        let _ = app.handle.cpu_cmd.send(CpuCommand::SetWatchpoint {
            addr: wp.addr,
            size: wp.size,
            mode: wp.mode,
            response: tx,
        });
    }

    // View state.
    if let Some(base) = session.disasm_view_base {
        app.disasm_view_base = base;
    }
    if let Some(follow) = session.disasm_follow_pc {
        app.disasm_follow_pc = follow;
    }
    if let Some(cursor) = session.memory_cursor {
        app.memory_cursor = cursor;
        app.memory_cursor_dirty = true;
    }

    eprintln!("[ui] session loaded from {}", path.display());
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
