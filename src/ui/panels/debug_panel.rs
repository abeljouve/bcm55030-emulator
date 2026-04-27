//! Debug panel — breakpoints, watchpoints and a best-effort call
//! stack in a single bottom tab. Reads state from the live
//! snapshot and dispatches mutations through `cpu_cmd`.

use eframe::egui;
use egui_phosphor::regular as ph;

use crate::emu::command::{oneshot, CpuCommand};
use crate::memory::{WatchMode, Watchpoint};
use crate::ui::EmulatorApp;

/// Maximum depth we scan when reconstructing the call stack.
const STACK_SCAN_SLOTS: usize = 64;
const STACK_SCAN_WINDOW_BYTES: u32 = 0x400;

pub fn draw(ui: &mut egui::Ui, app: &mut EmulatorApp) {
    ui.horizontal(|ui| {
        ui.strong(format!("{} Debug", ph::BUG));
        ui.separator();
        ui.selectable_value(&mut app.debug_tab, DebugTab::Breakpoints, "Breakpoints");
        ui.selectable_value(&mut app.debug_tab, DebugTab::Watchpoints, "Watchpoints");
        ui.selectable_value(&mut app.debug_tab, DebugTab::CallStack, "Call stack");
    });
    ui.separator();

    egui::ScrollArea::vertical()
        .id_salt("debug_panel")
        .auto_shrink([false, false])
        .show(ui, |ui| match app.debug_tab {
            DebugTab::Breakpoints => draw_breakpoints(ui, app),
            DebugTab::Watchpoints => draw_watchpoints(ui, app),
            DebugTab::CallStack => draw_call_stack(ui, app),
        });
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DebugTab {
    Breakpoints,
    Watchpoints,
    CallStack,
}

// ---------------------------------------------------------------
// Breakpoints
// ---------------------------------------------------------------

fn draw_breakpoints(ui: &mut egui::Ui, app: &mut EmulatorApp) {
    ui.horizontal(|ui| {
        ui.label(
            egui::RichText::new("Add at")
                .small()
                .color(app.accents.muted),
        );
        let resp = ui.add(
            egui::TextEdit::singleline(&mut app.debug_scratch.bp_addr)
                .desired_width(110.0)
                .hint_text("0x00000150"),
        );
        let enter_hit =
            resp.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter));
        if ui.button(format!("{} Add", ph::PLUS)).clicked() || enter_hit {
            if let Some(addr) = parse_hex_u32(&app.debug_scratch.bp_addr) {
                let (tx, _rx) = oneshot::<usize>();
                let _ = app
                    .handle
                    .cpu_cmd
                    .send(CpuCommand::SetBreakpoint { address: addr, response: tx });
                app.debug_scratch.bp_addr.clear();
            }
        }
        ui.separator();
        ui.label(
            egui::RichText::new(format!("{} active", app.snapshot.breakpoints.len()))
                .small()
                .color(app.accents.muted),
        );
    });
    ui.separator();

    let symbols = app.handle.annotations.read().symbols.clone();
    if app.snapshot.breakpoints.is_empty() {
        ui.colored_label(app.accents.muted, "No breakpoints.");
        return;
    }
    let accents = app.accents;
    let mut remove: Option<u32> = None;
    for addr in &app.snapshot.breakpoints {
        ui.horizontal(|ui| {
            ui.label(
                egui::RichText::new(ph::CIRCLE_HALF_TILT)
                    .color(accents.breakpoint)
                    .size(12.0),
            );
            ui.monospace(format!("0x{:08X}", addr));
            if let Some(sym) = symbols.get(addr) {
                ui.label(
                    egui::RichText::new(format!("— {sym}"))
                        .small()
                        .italics()
                        .color(accents.success),
                );
            }
            if ui
                .small_button(ph::X)
                .on_hover_text("Remove breakpoint")
                .clicked()
            {
                remove = Some(*addr);
            }
            if ui
                .small_button(format!("{} Reveal", ph::MAGNIFYING_GLASS))
                .on_hover_text("Show in the disassembly panel")
                .clicked()
            {
                app.disasm_view_base = *addr;
                app.disasm_follow_pc = false;
            }
        });
    }
    if let Some(addr) = remove {
        let _ = app
            .handle
            .cpu_cmd
            .send(CpuCommand::RemoveBreakpoint { address: addr });
    }
}

// ---------------------------------------------------------------
// Watchpoints
// ---------------------------------------------------------------

fn draw_watchpoints(ui: &mut egui::Ui, app: &mut EmulatorApp) {
    ui.horizontal(|ui| {
        ui.label(
            egui::RichText::new("Add at")
                .small()
                .color(app.accents.muted),
        );
        ui.add(
            egui::TextEdit::singleline(&mut app.debug_scratch.wp_addr)
                .desired_width(110.0)
                .hint_text("0x00010800"),
        );
        ui.label("size");
        ui.add(
            egui::TextEdit::singleline(&mut app.debug_scratch.wp_size)
                .desired_width(40.0)
                .hint_text("4"),
        );
        egui::ComboBox::from_id_salt("wp_mode_combo")
            .selected_text(mode_label(app.debug_scratch.wp_mode))
            .show_ui(ui, |ui| {
                ui.selectable_value(
                    &mut app.debug_scratch.wp_mode,
                    WatchMode::Read,
                    "Read",
                );
                ui.selectable_value(
                    &mut app.debug_scratch.wp_mode,
                    WatchMode::Write,
                    "Write",
                );
                ui.selectable_value(
                    &mut app.debug_scratch.wp_mode,
                    WatchMode::ReadWrite,
                    "Read/Write",
                );
            });
        if ui.button(format!("{} Add", ph::PLUS)).clicked() {
            if let (Some(addr), Ok(size)) = (
                parse_hex_u32(&app.debug_scratch.wp_addr),
                app.debug_scratch.wp_size.parse::<u32>(),
            ) {
                let (tx, _rx) = oneshot::<usize>();
                let _ = app.handle.cpu_cmd.send(CpuCommand::SetWatchpoint {
                    addr,
                    size: size.max(1),
                    mode: app.debug_scratch.wp_mode,
                    response: tx,
                });
                app.debug_scratch.wp_addr.clear();
            }
        }
        ui.separator();
        ui.label(
            egui::RichText::new(format!("{} active", app.snapshot.watchpoints.len()))
                .small()
                .color(app.accents.muted),
        );
    });
    ui.separator();

    if app.snapshot.watchpoints.is_empty() {
        ui.colored_label(app.accents.muted, "No watchpoints.");
        return;
    }
    let accents = app.accents;
    let entries: Vec<Watchpoint> = app.snapshot.watchpoints.clone();
    let mut remove: Option<usize> = None;
    for (idx, wp) in entries.iter().enumerate() {
        ui.horizontal(|ui| {
            ui.label(
                egui::RichText::new(ph::EYE)
                    .color(accents.warning)
                    .size(12.0),
            );
            ui.monospace(format!(
                "0x{:08X}  {:>2} B  {}",
                wp.addr,
                wp.size,
                mode_label(wp.mode),
            ));
            if ui
                .small_button(ph::X)
                .on_hover_text("Remove watchpoint")
                .clicked()
            {
                remove = Some(idx);
            }
            if ui
                .small_button(format!("{} Go to", ph::MAGNIFYING_GLASS))
                .on_hover_text("Show in the memory viewer")
                .clicked()
            {
                app.memory_cursor = wp.addr;
                app.memory_cursor_dirty = true;
            }
        });
    }
    if let Some(idx) = remove {
        let _ = app
            .handle
            .cpu_cmd
            .send(CpuCommand::RemoveWatchpoint { index: idx });
    }
}

fn mode_label(mode: WatchMode) -> &'static str {
    match mode {
        WatchMode::Read => "Read",
        WatchMode::Write => "Write",
        WatchMode::ReadWrite => "Read/Write",
    }
}

// ---------------------------------------------------------------
// Call stack
// ---------------------------------------------------------------

fn draw_call_stack(ui: &mut egui::Ui, app: &mut EmulatorApp) {
    let pc = app.snapshot.cpu.pc;
    let blink = app.snapshot.cpu.core_regs[31];
    let sp = app.snapshot.cpu.core_regs[28];
    let symbols = app.handle.annotations.read().symbols.clone();
    let accents = app.accents;

    // Frame #0 — current PC.
    frame_row(ui, &accents, 0, pc, "PC", &symbols, &mut |target| {
        app.disasm_view_base = target;
        app.disasm_follow_pc = false;
    });

    // Frame #1 — blink (r31). Convention: ARC callers leave the
    // return address in `blink`; leaf functions never push it.
    if blink != 0 && blink != pc {
        frame_row(ui, &accents, 1, blink, "blink (r31)", &symbols, &mut |target| {
            app.disasm_view_base = target;
            app.disasm_follow_pc = false;
        });
    }

    ui.separator();
    ui.label(
        egui::RichText::new("Stack scan — likely return addresses")
            .small()
            .color(accents.muted),
    );

    // Walk the stack around `sp` looking for 4-byte-aligned
    // words that look like valid code addresses in SRAM. This is
    // a best-effort heuristic — it surfaces saved-blink slots
    // without knowing the exact prologue of each function.
    let Some(sram) = app.sram.as_ref() else {
        ui.colored_label(accents.muted, "Waiting for SRAM snapshot…");
        return;
    };
    let candidates = scan_stack_for_return_addresses(
        &sram.bytes,
        sp,
        STACK_SCAN_WINDOW_BYTES,
        STACK_SCAN_SLOTS,
    );
    if candidates.is_empty() {
        ui.colored_label(accents.muted, "No call-like slots found.");
        return;
    }
    let mut frame_idx = 2usize;
    for (slot_addr, target) in candidates {
        if target == pc || target == blink {
            continue;
        }
        frame_row(
            ui,
            &accents,
            frame_idx,
            target,
            &format!("[0x{:08X}]", slot_addr),
            &symbols,
            &mut |target| {
                app.disasm_view_base = target;
                app.disasm_follow_pc = false;
            },
        );
        frame_idx += 1;
    }
}

fn frame_row(
    ui: &mut egui::Ui,
    accents: &crate::ui::theme::AccentTokens,
    depth: usize,
    target: u32,
    origin: &str,
    symbols: &std::collections::HashMap<u32, String>,
    reveal: &mut dyn FnMut(u32),
) {
    ui.horizontal(|ui| {
        ui.label(
            egui::RichText::new(format!("#{depth}"))
                .monospace()
                .small()
                .color(accents.muted),
        );
        ui.label(
            egui::RichText::new(format!("0x{:08X}", target))
                .monospace()
                .color(if depth == 0 {
                    accents.pc_highlight_strong
                } else {
                    accents.accent
                }),
        );
        if let Some(sym) = symbols.get(&target) {
            ui.label(
                egui::RichText::new(sym.as_str())
                    .small()
                    .italics()
                    .color(accents.success),
            );
        }
        ui.label(
            egui::RichText::new(origin)
                .small()
                .color(accents.muted),
        );
        if ui
            .small_button(format!("{} Reveal", ph::MAGNIFYING_GLASS))
            .clicked()
        {
            reveal(target);
        }
    });
}

/// Walk `sp..sp+window` in 4-byte steps reading the big-endian
/// word at each slot. A word is considered a plausible return
/// address if it is 2-byte aligned and falls inside the SRAM
/// region. Returns `(slot_addr, target)` pairs.
fn scan_stack_for_return_addresses(
    sram: &[u8],
    sp: u32,
    window: u32,
    max_slots: usize,
) -> Vec<(u32, u32)> {
    let mut out = Vec::new();
    let start = sp & !3;
    let end = start.saturating_add(window);
    let mut addr = start;
    while addr + 4 <= end && out.len() < max_slots {
        let off = addr as usize;
        if off + 4 > sram.len() {
            break;
        }
        let w = ((sram[off] as u32) << 24)
            | ((sram[off + 1] as u32) << 16)
            | ((sram[off + 2] as u32) << 8)
            | (sram[off + 3] as u32);
        // Plausible? 2-byte aligned, inside SRAM, not zero.
        if w != 0 && (w & 1) == 0 && (w as usize) < sram.len() {
            out.push((addr, w));
        }
        addr = addr.wrapping_add(4);
    }
    out
}

// ---------------------------------------------------------------
// Scratch state
// ---------------------------------------------------------------

/// Form input state for the debug panel. Lives on `EmulatorApp`
/// so text edits survive repaints.
pub struct DebugScratch {
    pub bp_addr: String,
    pub wp_addr: String,
    pub wp_size: String,
    pub wp_mode: WatchMode,
}

impl Default for DebugScratch {
    fn default() -> Self {
        Self {
            bp_addr: String::new(),
            wp_addr: String::new(),
            wp_size: "4".to_string(),
            wp_mode: WatchMode::Write,
        }
    }
}

fn parse_hex_u32(s: &str) -> Option<u32> {
    let t = s.trim().trim_start_matches("0x").trim_start_matches("0X");
    u32::from_str_radix(t, 16).ok()
}
