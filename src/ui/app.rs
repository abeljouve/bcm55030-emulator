//! Top-level `EmulatorApp` + `eframe::App` implementation.
//!
//! Owns a clone of the shared `EmulatorHandle`, polls the latest
//! `EmulatorSnapshot` once per frame, and fans it out to each
//! panel's `draw` function. Every panel reads from the snapshot
//! / bank and emits `CpuCommand`s through `handle.cpu_cmd` or
//! `PeripheralEvent`s through `handle.bank.write().inject_event`
//! — the UI never touches `Cpu` directly.

use std::time::Duration;

use eframe::egui;

use crate::emu::snapshot::{DcacheSnapshot, SramSnapshot};
use crate::emu::{EmulatorHandle, EmulatorSnapshot};
use crate::ui::panels;

/// Shared UI state. Cheap to clone (everything lives behind
/// `Arc`s or is plain data).
pub struct EmulatorApp {
    pub handle: EmulatorHandle,

    /// Most recent snapshot polled from `handle.snapshot`. Stored
    /// so panels can reuse the same data and so diff-driven
    /// highlights (register change fade) have a stable baseline.
    pub snapshot: EmulatorSnapshot,

    /// Previous frame's core-register set, used to detect
    /// value changes and drive the orange fade in the
    /// register panel.
    pub prev_core_regs: [u32; 64],
    /// Frame-local change markers: which register indices
    /// changed most recently. Reset every frame by `update`.
    pub changed_regs: Vec<u8>,

    /// On-demand SRAM snapshot for the disassembly /memory
    /// viewer. Refreshed via `CpuCommand::RequestSram` at most
    /// every `SRAM_REFRESH` wall-clock interval.
    pub sram: Option<SramSnapshot>,
    /// Last wall-clock time an SRAM refresh was issued.
    pub last_sram_fetch: Option<std::time::Instant>,

    /// On-demand D-cache snapshot for the "Cache state" sub-tab.
    pub dcache: Option<DcacheSnapshot>,

    // Per-panel UI state.
    pub disasm_cursor: u32,
    pub memory_cursor: u32,
    pub memory_tab: panels::memory::Tab,
    pub registers_tab: panels::registers::Tab,
    pub uart_input: String,

    /// Central pane tab selection: memory viewer vs peripheral
    /// inspector. Switched via a small horizontal selector at the
    /// top of the central panel.
    pub central_tab: panels::CentralTab,
    /// Currently-selected peripheral in the inspector.
    pub peripheral_tab: panels::peripherals::PeripheralTab,
    /// Bottom-left panel tab selection: UART terminal vs MCP
    /// activity log.
    pub bottom_tab: panels::BottomTab,
    /// Scratch buffers for peripheral inspector input widgets.
    pub periph_scratch: panels::peripherals::PeripheralScratch,

    /// Debounce flag: set to true on the very first frame so
    /// the app can perform one-shot initialisation (turning
    /// off `uart.stdout_passthrough`, etc.).
    pub first_frame: bool,
}

const SRAM_REFRESH: Duration = Duration::from_millis(100);

impl EmulatorApp {
    pub fn new(handle: EmulatorHandle) -> Self {
        let snapshot = handle.snapshot.lock().clone();
        let prev_core_regs = snapshot.cpu.core_regs;
        let disasm_cursor = snapshot.cpu.pc;
        Self {
            handle,
            snapshot,
            prev_core_regs,
            changed_regs: Vec::new(),
            sram: None,
            last_sram_fetch: None,
            dcache: None,
            disasm_cursor,
            memory_cursor: 0,
            memory_tab: panels::memory::Tab::Sram,
            registers_tab: panels::registers::Tab::Core,
            uart_input: String::new(),
            central_tab: panels::CentralTab::Memory,
            peripheral_tab: panels::peripherals::PeripheralTab::Uart,
            bottom_tab: panels::BottomTab::Uart,
            periph_scratch: panels::peripherals::PeripheralScratch::default(),
            first_frame: true,
        }
    }

    fn refresh_snapshot(&mut self) {
        let fresh = self.handle.snapshot.lock().clone();
        self.changed_regs.clear();
        for i in 0..64usize {
            if fresh.cpu.core_regs[i] != self.prev_core_regs[i] {
                self.changed_regs.push(i as u8);
            }
        }
        self.prev_core_regs = fresh.cpu.core_regs;
        self.snapshot = fresh;
    }

    /// Fire an SRAM request if one hasn't landed within the
    /// `SRAM_REFRESH` window. The worker answers asynchronously
    /// via a oneshot — we grab it on the next frame.
    fn maybe_refresh_sram(&mut self) {
        let now = std::time::Instant::now();
        if let Some(last) = self.last_sram_fetch {
            if now.duration_since(last) < SRAM_REFRESH {
                return;
            }
        }
        self.last_sram_fetch = Some(now);
        let (tx, rx) = crate::emu::command::oneshot::<SramSnapshot>();
        if self
            .handle
            .cpu_cmd
            .send(crate::emu::command::CpuCommand::RequestSram { response: tx })
            .is_ok()
        {
            // Block briefly (≤ 5 ms) — worker responds on its
            // next scheduling opportunity. If it's deep in a
            // hot loop the first attempt may time out; the
            // next frame retries.
            if let Ok(snap) = rx.recv_timeout(Duration::from_millis(5)) {
                self.sram = Some(snap);
            }
        }
    }
}

impl eframe::App for EmulatorApp {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        if self.first_frame {
            self.handle.bank.write().uart.stdout_passthrough = false;
            self.first_frame = false;
        }
        self.refresh_snapshot();
        self.maybe_refresh_sram();

        egui::Panel::top("toolbar")
            .exact_size(36.0)
            .show_inside(ui, |ui| panels::toolbar::draw(ui, self));
        egui::Panel::bottom("status_bar")
            .exact_size(22.0)
            .show_inside(ui, |ui| panels::status_bar::draw(ui, self));
        egui::Panel::bottom("bottom_tabs")
            .resizable(true)
            .default_size(200.0)
            .show_inside(ui, |ui| {
                ui.horizontal(|ui| {
                    ui.selectable_value(
                        &mut self.bottom_tab,
                        panels::BottomTab::Uart,
                        "UART",
                    );
                    ui.selectable_value(
                        &mut self.bottom_tab,
                        panels::BottomTab::McpLog,
                        "MCP Activity",
                    );
                });
                ui.separator();
                match self.bottom_tab {
                    panels::BottomTab::Uart => panels::uart_terminal::draw(ui, self),
                    panels::BottomTab::McpLog => panels::mcp_log::draw(ui, self),
                }
            });
        egui::Panel::left("disassembly")
            .resizable(true)
            .default_size(420.0)
            .show_inside(ui, |ui| panels::disassembly::draw(ui, self));
        egui::Panel::right("registers")
            .resizable(true)
            .default_size(320.0)
            .show_inside(ui, |ui| panels::registers::draw(ui, self));
        egui::CentralPanel::default().show_inside(ui, |ui| {
            ui.horizontal(|ui| {
                ui.selectable_value(
                    &mut self.central_tab,
                    panels::CentralTab::Memory,
                    "Memory",
                );
                ui.selectable_value(
                    &mut self.central_tab,
                    panels::CentralTab::Peripherals,
                    "Peripherals",
                );
            });
            ui.separator();
            match self.central_tab {
                panels::CentralTab::Memory => panels::memory::draw(ui, self),
                panels::CentralTab::Peripherals => panels::peripherals::draw(ui, self),
            }
        });

        ui.ctx().request_repaint_after(Duration::from_millis(16));
    }
}

/// Entry point invoked from `main.rs`. Blocks the calling thread
/// on `eframe::run_native` — callers should spawn the CPU worker
/// and the MCP server thread *before* calling `run`.
pub fn run(handle: EmulatorHandle) -> eframe::Result<()> {
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_title("ARC700 Emulator — BCM55030")
            .with_inner_size([1440.0, 900.0])
            .with_min_inner_size([960.0, 600.0]),
        ..Default::default()
    };
    eframe::run_native(
        "arc700-emulator",
        options,
        Box::new(move |_cc| Ok(Box::new(EmulatorApp::new(handle)))),
    )
}
