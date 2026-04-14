//! UART terminal: TX log (output) + input text field that feeds
//! the bank's UART RX mpsc sender.

use eframe::egui;

use crate::soc::peripheral::{PeripheralEvent, UartEvent};
use crate::ui::theme;
use crate::ui::EmulatorApp;

pub fn draw(ui: &mut egui::Ui, app: &mut EmulatorApp) {
    ui.horizontal(|ui| {
        ui.label("UART");
        if ui.button("Clear").clicked() {
            let event = PeripheralEvent::Uart(UartEvent::ClearTxLog);
            app.handle.bank.write().inject_event(&event);
            app.uart_log_written = 0;
        }
        ui.separator();
        let logging = app.uart_log_file.is_some();
        let label = if logging {
            format!(
                "📝 Logging → {}",
                app.uart_log_path
                    .as_ref()
                    .and_then(|p| p.file_name().map(|n| n.to_string_lossy().into_owned()))
                    .unwrap_or_else(|| "?".to_string())
            )
        } else {
            "📝 Log to file…".to_string()
        };
        if ui
            .button(label)
            .on_hover_text("Tee every UART TX byte to a file on disk")
            .clicked()
        {
            if logging {
                app.uart_log_file = None;
                app.uart_log_path = None;
                app.uart_log_written = 0;
            } else if let Some(path) = rfd::FileDialog::new()
                .add_filter("log", &["log", "txt"])
                .set_file_name("uart.log")
                .save_file()
            {
                match std::fs::OpenOptions::new()
                    .create(true)
                    .append(true)
                    .open(&path)
                {
                    Ok(f) => {
                        app.uart_log_file = Some(f);
                        app.uart_log_path = Some(path);
                        app.uart_log_written = 0;
                    }
                    Err(e) => eprintln!("[ui] uart log open failed: {e}"),
                }
            }
        }
        ui.separator();
        let ienable = app.snapshot.cpu.aux.ienable;
        let rx_irq = (ienable >> 5) & 1 == 1;
        ui.colored_label(
            if rx_irq {
                egui::Color32::from_rgb(40, 200, 80)
            } else {
                theme::MUTED
            },
            "IRQ5",
        );
    });
    ui.separator();

    // Output: read the bank's TX log, render in a fixed-font
    // scroll area with terminal colours.
    let bytes = app.handle.bank.read().uart.tx_log_bytes();
    let text = String::from_utf8_lossy(&bytes).into_owned();
    egui::Frame::default()
        .fill(theme::TERMINAL_BG)
        .show(ui, |ui| {
            egui::ScrollArea::vertical()
                .id_salt("uart_output")
                .stick_to_bottom(true)
                .max_height(ui.available_height() - 28.0)
                .show(ui, |ui| {
                    ui.colored_label(
                        theme::TERMINAL_FG,
                        egui::RichText::new(text).monospace(),
                    );
                });
        });

    // Input: Enter submits. Bytes go straight to the bank's
    // UART RX channel.
    ui.horizontal(|ui| {
        let resp = ui.add(
            egui::TextEdit::singleline(&mut app.uart_input)
                .desired_width(ui.available_width() - 80.0)
                .hint_text("type here + Enter"),
        );
        let enter_hit =
            resp.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter));
        if enter_hit || ui.button("Send").clicked() {
            for b in app.uart_input.bytes() {
                let _ = app.handle.uart_tx.send(b);
            }
            let _ = app.handle.uart_tx.send(b'\r');
            app.uart_input.clear();
            resp.request_focus();
        }
    });
}
