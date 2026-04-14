//! Top toolbar: execution controls, boot mode toggle, speed
//! slider. Emits `CpuCommand`s via `handle.cpu_cmd`.

use eframe::egui;

use crate::emu::command::CpuCommand;
use crate::soc::bank::BootMode;
use crate::ui::EmulatorApp;

pub fn draw(ui: &mut egui::Ui, app: &mut EmulatorApp) {
    ui.horizontal(|ui| {
        let running = matches!(
            app.snapshot.run_state,
            crate::emu::snapshot::RunState::Running
        );

        if ui
            .add(egui::Button::new(if running { "⏸ Pause" } else { "▶ Run" }))
            .clicked()
        {
            let cmd = if running {
                CpuCommand::Pause
            } else {
                CpuCommand::Run { max_insns: None }
            };
            let _ = app.handle.cpu_cmd.send(cmd);
        }

        if ui.button("⟶ Step").clicked() {
            let _ = app.handle.cpu_cmd.send(CpuCommand::StepOne);
        }
        if ui.button("⇒ Step Over").clicked() {
            let _ = app.handle.cpu_cmd.send(CpuCommand::StepOver);
        }
        if ui.button("↷ Run to cursor").clicked() {
            let _ = app.handle.cpu_cmd.send(CpuCommand::RunTo {
                address: app.disasm_cursor,
            });
        }
        if ui.button("↺ Reset").clicked() {
            let _ = app.handle.cpu_cmd.send(CpuCommand::Reset {
                boot_mode: app.snapshot.boot_mode,
                keep_breakpoints: true,
            });
        }

        ui.separator();

        // Boot mode radio: stateful — Reset sends whichever is selected.
        let mut mode = app.snapshot.boot_mode;
        if ui
            .radio_value(&mut mode, BootMode::Warm, "Warm")
            .clicked()
            || ui.radio_value(&mut mode, BootMode::Cold, "Cold").clicked()
        {
            // No immediate dispatch — applied on next Reset click.
            // Persist intent on the snapshot by locking + writing.
            app.handle.snapshot.lock().boot_mode = mode;
        }

        ui.separator();

        ui.label(format!(
            "PC 0x{:08X}  insn {}  bank={}/64",
            app.snapshot.cpu.pc,
            app.snapshot.cpu.instruction_count,
            app.snapshot.bank_tick_accumulator % 64,
        ));
    });
}
