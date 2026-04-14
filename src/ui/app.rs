//! Top-level `EmulatorApp` + `eframe::App` implementation.
//!
//! Owns a clone of the shared `EmulatorHandle`, polls the latest
//! `EmulatorSnapshot` once per frame, and fans it out to each
//! panel's `draw` function. Every panel reads from the snapshot
//! / bank and emits `CpuCommand`s through `handle.cpu_cmd` or
//! `PeripheralEvent`s through `handle.bank.write().inject_event`
//! — the UI never touches `Cpu` directly.

use std::sync::Arc;
use std::time::Duration;

use eframe::egui;

use crate::emu::snapshot::{DcacheSnapshot, SramSnapshot};
use crate::emu::{EmulatorHandle, EmulatorSnapshot};
use crate::ui::panels;
use crate::ui::theme::{AccentTokens, Palette};

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
    /// Top-of-view address for the disassembly panel. Separate
    /// from `disasm_cursor` (which tracks the user's click
    /// selection) and from the CPU PC — it is the *viewport* the
    /// panel renders around. Updated by the `Follow PC` mode, by
    /// scroll events, by Page Up/Down, and by the goto box.
    pub disasm_view_base: u32,
    /// When true, the panel snaps `disasm_view_base` to the CPU
    /// PC every frame. Cleared on any user-driven scroll or
    /// navigation; restored by clicking the "Follow PC" button.
    pub disasm_follow_pc: bool,
    /// Number of visible rows in the disassembly panel, computed
    /// per frame from the available height. Cached here so
    /// keyboard navigation (Page Up/Down) can reuse it.
    pub disasm_visible_rows: u32,
    pub memory_cursor: u32,
    /// One-shot flag set when the user types a new address in
    /// the memory viewer "Go to" box. The next frame's scroll
    /// area snaps to the corresponding row and clears the flag.
    pub memory_cursor_dirty: bool,
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

    /// Active Catppuccin flavour. Persisted via eframe storage.
    pub palette: Palette,
    /// Accent tokens derived from `palette`. Rebuilt whenever the
    /// palette changes.
    pub accents: AccentTokens,

    /// Ring buffer of recent instructions-per-second samples.
    /// Feeds the status-bar sparkline. Capacity = 120 samples ≈
    /// two minutes at 1 Hz sampling.
    pub ips_history: std::collections::VecDeque<u32>,
    /// Last wall-clock time we pushed an IPS sample.
    pub last_ips_sample: Option<std::time::Instant>,

    /// Log-scale value currently reflected by the toolbar speed
    /// slider. Separate from the worker state so the slider feels
    /// smooth during a drag — the actual `SetSpeed` command is
    /// only dispatched on drag-release.
    pub speed_slider_log10: f32,
    /// `true` while the user holds the speed slider, used to
    /// suppress the live re-sync from `snapshot.speed_limit`.
    pub speed_slider_dragging: bool,
}

const SRAM_REFRESH: Duration = Duration::from_millis(100);
const STORAGE_PALETTE_KEY: &str = "ui.palette";

impl EmulatorApp {
    pub fn new(handle: EmulatorHandle, palette: Palette) -> Self {
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
            disasm_view_base: disasm_cursor,
            disasm_follow_pc: true,
            disasm_visible_rows: 64,
            memory_cursor: 0,
            memory_cursor_dirty: false,
            memory_tab: panels::memory::Tab::Sram,
            registers_tab: panels::registers::Tab::Core,
            uart_input: String::new(),
            central_tab: panels::CentralTab::Memory,
            peripheral_tab: panels::peripherals::PeripheralTab::Uart,
            bottom_tab: panels::BottomTab::Uart,
            periph_scratch: panels::peripherals::PeripheralScratch::default(),
            palette,
            accents: AccentTokens::from_palette(palette),
            ips_history: std::collections::VecDeque::with_capacity(120),
            last_ips_sample: None,
            speed_slider_log10: 8.0,
            speed_slider_dragging: false,
        }
    }

    /// Push an IPS sample if at least a second has elapsed since
    /// the previous one. Called once per frame from `refresh_snapshot`.
    fn push_ips_sample(&mut self) {
        let now = std::time::Instant::now();
        let should = match self.last_ips_sample {
            None => true,
            Some(prev) => now.duration_since(prev) >= Duration::from_millis(500),
        };
        if !should {
            return;
        }
        self.last_ips_sample = Some(now);
        if self.ips_history.len() == 120 {
            self.ips_history.pop_front();
        }
        self.ips_history.push_back(self.snapshot.insns_per_sec);
    }

    /// Swap the active palette, rebuilding accents and pushing
    /// the new visuals into the egui context.
    pub fn set_palette(&mut self, ctx: &egui::Context, palette: Palette) {
        self.palette = palette;
        self.accents = AccentTokens::from_palette(palette);
        ctx.set_visuals(crate::ui::theme::visuals_for(palette));
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
        self.push_ips_sample();
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
            if let Ok(snap) = rx.recv_timeout(Duration::from_millis(5)) {
                self.sram = Some(snap);
            }
        }
    }
}

impl eframe::App for EmulatorApp {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        self.refresh_snapshot();
        self.maybe_refresh_sram();

        // One-shot fade-in covering the entire window on first
        // display. `animate_value_with_time` drives a 0→1 linear
        // ramp across the first ~500 ms. We use the resulting
        // alpha to gamma-multiply the whole UI via a global
        // opacity override.
        let fade = ui
            .ctx()
            .animate_value_with_time(egui::Id::new("app.fade_in"), 1.0, 0.5);
        ui.set_opacity(fade.clamp(0.0, 1.0));

        egui::Panel::top("toolbar")
            .exact_size(40.0)
            .show_inside(ui, |ui| panels::toolbar::draw(ui, self));
        egui::Panel::bottom("status_bar")
            .exact_size(24.0)
            .show_inside(ui, |ui| panels::status_bar::draw(ui, self));
        egui::Panel::bottom("bottom_tabs")
            .resizable(true)
            .default_size(200.0)
            .show_inside(ui, |ui| {
                ui.horizontal(|ui| {
                    ui.selectable_value(
                        &mut self.bottom_tab,
                        panels::BottomTab::Uart,
                        format!("{} UART", egui_phosphor::regular::TERMINAL),
                    );
                    ui.selectable_value(
                        &mut self.bottom_tab,
                        panels::BottomTab::McpLog,
                        format!("{} MCP Activity", egui_phosphor::regular::PLUGS_CONNECTED),
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
            .default_size(460.0)
            .show_inside(ui, |ui| panels::disassembly::draw(ui, self));
        egui::Panel::right("registers")
            .resizable(true)
            .default_size(340.0)
            .show_inside(ui, |ui| panels::registers::draw(ui, self));
        egui::CentralPanel::default().show_inside(ui, |ui| {
            ui.horizontal(|ui| {
                ui.selectable_value(
                    &mut self.central_tab,
                    panels::CentralTab::Memory,
                    format!("{} Memory", egui_phosphor::regular::MEMORY),
                );
                ui.selectable_value(
                    &mut self.central_tab,
                    panels::CentralTab::Peripherals,
                    format!("{} Peripherals", egui_phosphor::regular::CIRCUITRY),
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

    fn save(&mut self, storage: &mut dyn eframe::Storage) {
        storage.set_string(STORAGE_PALETTE_KEY, self.palette.as_str().to_string());
    }
}

/// Load a Palette from eframe storage, defaulting to Mocha.
fn load_palette(cc: &eframe::CreationContext<'_>) -> Palette {
    cc.storage
        .and_then(|s| s.get_string(STORAGE_PALETTE_KEY))
        .and_then(|s| Palette::from_str(&s))
        .unwrap_or(Palette::Mocha)
}

/// Install JetBrains Mono as the primary monospace font and add
/// the Phosphor icon set to both families so we can drop icon
/// glyphs into any label.
fn install_fonts(ctx: &egui::Context) {
    let mut fonts = egui::FontDefinitions::default();

    // Vendored JetBrains Mono Regular (OFL). Becomes the first
    // choice for the monospace family; Hack / Ubuntu stay as
    // fallbacks for glyphs JetBrains Mono does not cover.
    const JETBRAINS_MONO: &[u8] =
        include_bytes!("../../assets/fonts/JetBrainsMono-Regular.ttf");
    fonts.font_data.insert(
        "jetbrains_mono".to_owned(),
        Arc::new(egui::FontData::from_static(JETBRAINS_MONO)),
    );
    if let Some(family) = fonts.families.get_mut(&egui::FontFamily::Monospace) {
        family.insert(0, "jetbrains_mono".to_owned());
    }

    egui_phosphor::add_to_fonts(&mut fonts, egui_phosphor::Variant::Regular);
    ctx.set_fonts(fonts);
}

/// Entry point invoked from the GUI binary. Blocks the calling
/// thread on `eframe::run_native` — callers should spawn the CPU
/// worker and the MCP server thread *before* calling `run`.
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
        Box::new(move |cc| {
            let palette = load_palette(cc);
            install_fonts(&cc.egui_ctx);
            cc.egui_ctx.set_visuals(crate::ui::theme::visuals_for(palette));
            cc.egui_ctx.all_styles_mut(|s| crate::ui::theme::configure_style(s));
            Ok(Box::new(EmulatorApp::new(handle, palette)))
        }),
    )
}
