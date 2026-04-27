//! Peripheral inspector: 14 tabs, one per BCM55030 MMIO subsystem.
//!
//! Reads snapshots from `app.snapshot.peripherals` (refreshed by
//! the CPU worker) and routes UI-driven mutations through
//! `handle.bank.write().inject_event(...)`. Peripherals without a
//! typed snapshot (mpcp, nco) fall back to `peek_word` hex dumps.

use eframe::egui;
use egui_phosphor::regular as ph;

use crate::soc::peripheral::{
    AlarmEvent, BscEvent, DmaEvent, EfuseEvent, EponEvent, FatalFilterEvent,
    LaneSpeed, MacsecEvent, PbcEvent, PeripheralEvent, PeripheralSnapshot,
    SerDesEvent, SfpEvent, TimerEvent, UartEvent,
};
use crate::ui::EmulatorApp;

/// Selected peripheral tab inside the inspector.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PeripheralTab {
    Uart,
    Pbc,
    Bsc,
    Sfp,
    SerDes,
    Epon,
    Macsec,
    Dma,
    Alarm,
    Timer,
    Efuse,
    Fatal,
    Mpcp,
    Nco,
}

/// Scratch input state for the peripheral inspector. Lives on
/// `EmulatorApp` so field text survives repaints.
#[derive(Default)]
pub struct PeripheralScratch {
    pub sfp_temp_c256: Option<i16>,
    pub sfp_vcc_uv: Option<u32>,
    pub sfp_tx_bias_ua: Option<u32>,
    pub sfp_tx_power_uw: Option<u32>,
    pub sfp_rx_power_uw: Option<u32>,
    pub alarm_opcode: String,
    pub timer_prescaler: String,
    pub timer_counter: String,
    pub dma_channel: u8,
    pub macsec_sa_slot: u8,
    pub epon_llid: u8,
    pub fatal_inject_mask: String,
    pub fatal_link_idx: u8,
    pub fatal_link_up: bool,
}

pub fn draw(ui: &mut egui::Ui, app: &mut EmulatorApp) {
    egui::Panel::left("peripheral_sidebar")
        .resizable(false)
        .exact_size(160.0)
        .frame(
            egui::Frame::default()
                .fill(ui.visuals().extreme_bg_color.gamma_multiply(0.4))
                .inner_margin(egui::Margin::same(6)),
        )
        .show_inside(ui, |ui| {
            ui.label(
                egui::RichText::new("Peripherals")
                    .small()
                    .color(app.accents.muted),
            );
            ui.add_space(4.0);
            for (tab, icon, label) in TABS {
                let selected = app.peripheral_tab == *tab;
                let text = egui::RichText::new(format!("{icon}  {label}"))
                    .monospace()
                    .size(13.0);
                let resp = ui.add_sized(
                    [ui.available_width(), 24.0],
                    egui::Button::selectable(selected, text),
                );
                if resp.clicked() {
                    app.peripheral_tab = *tab;
                }
            }
        });

    egui::CentralPanel::default()
        .frame(egui::Frame::default().inner_margin(egui::Margin::same(10)))
        .show_inside(ui, |ui| {
            ui.horizontal(|ui| {
                let (icon, title) = header_for(app.peripheral_tab);
                ui.strong(format!("{icon}  {title}"));
            });
            ui.separator();
            egui::ScrollArea::vertical()
                .id_salt("peripheral_inspector")
                .show(ui, |ui| match app.peripheral_tab {
                    PeripheralTab::Uart => draw_uart(ui, app),
                    PeripheralTab::Pbc => draw_pbc(ui, app),
                    PeripheralTab::Bsc => draw_bsc(ui, app),
                    PeripheralTab::Sfp => draw_sfp(ui, app),
                    PeripheralTab::SerDes => draw_serdes(ui, app),
                    PeripheralTab::Epon => draw_epon(ui, app),
                    PeripheralTab::Macsec => draw_macsec(ui, app),
                    PeripheralTab::Dma => draw_dma(ui, app),
                    PeripheralTab::Alarm => draw_alarm(ui, app),
                    PeripheralTab::Timer => draw_timer(ui, app),
                    PeripheralTab::Efuse => draw_efuse(ui, app),
                    PeripheralTab::Fatal => draw_fatal(ui, app),
                    PeripheralTab::Mpcp => draw_mpcp(ui, app),
                    PeripheralTab::Nco => draw_nco(ui, app),
                });
        });
}

const TABS: &[(PeripheralTab, &str, &str)] = &[
    (PeripheralTab::Uart, ph::TERMINAL, "UART"),
    (PeripheralTab::Pbc, ph::HARD_DRIVES, "PBC"),
    (PeripheralTab::Bsc, ph::PLUGS, "BSC/I2C"),
    (PeripheralTab::Sfp, ph::LIGHTNING, "SFP"),
    (PeripheralTab::SerDes, ph::WAVEFORM, "SerDes"),
    (PeripheralTab::Epon, ph::NETWORK, "EPON"),
    (PeripheralTab::Macsec, ph::LOCK_KEY, "MACsec"),
    (PeripheralTab::Dma, ph::ARROWS_LEFT_RIGHT, "DMA"),
    (PeripheralTab::Alarm, ph::BELL, "Alarm"),
    (PeripheralTab::Timer, ph::CLOCK, "Timer"),
    (PeripheralTab::Efuse, ph::FINGERPRINT, "eFuse"),
    (PeripheralTab::Fatal, ph::WARNING, "Fatal"),
    (PeripheralTab::Mpcp, ph::TREE_STRUCTURE, "MPCP"),
    (PeripheralTab::Nco, ph::WAVE_SINE, "NCO"),
];

fn header_for(tab: PeripheralTab) -> (&'static str, &'static str) {
    for (t, icon, label) in TABS {
        if *t == tab {
            return (icon, label);
        }
    }
    ("", "Peripheral")
}

fn find_snapshot<'a>(
    app: &'a EmulatorApp,
    matcher: fn(&PeripheralSnapshot) -> bool,
) -> Option<&'a PeripheralSnapshot> {
    app.snapshot.peripherals.iter().find(|p| matcher(p))
}

fn inject(app: &EmulatorApp, event: PeripheralEvent) {
    app.handle.bank.write().inject_event(&event);
}

fn missing(ui: &mut egui::Ui, _app: &EmulatorApp, what: &str) {
    ui.colored_label(
        ui.visuals().weak_text_color(),
        format!("{what}: snapshot not available yet."),
    );
}

// ---------------------------------------------------------------------------
// UART
// ---------------------------------------------------------------------------

fn draw_uart(ui: &mut egui::Ui, app: &EmulatorApp) {
    let Some(PeripheralSnapshot::Uart(snap)) =
        find_snapshot(app, |p| matches!(p, PeripheralSnapshot::Uart(_)))
    else {
        missing(ui, app, "UART");
        return;
    };
    kv(ui, "IER (IRQ enable)", format!("0x{:02X}", snap.ier));
    kv(ui, "Baud divisor", format!("{}", snap.baud_divisor));
    kv(ui, "RX queue depth", format!("{}", snap.rx_queue_len));
    kv(ui, "TX log tail bytes", format!("{}", snap.tx_log_tail.len()));
    ui.separator();
    if ui.button("Send Break").clicked() {
        inject(app, PeripheralEvent::Uart(UartEvent::Break));
    }
    if ui.button("Clear TX log").clicked() {
        inject(app, PeripheralEvent::Uart(UartEvent::ClearTxLog));
    }
}

// ---------------------------------------------------------------------------
// PBC
// ---------------------------------------------------------------------------

fn draw_pbc(ui: &mut egui::Ui, app: &EmulatorApp) {
    let Some(PeripheralSnapshot::Pbc(snap)) =
        find_snapshot(app, |p| matches!(p, PeripheralSnapshot::Pbc(_)))
    else {
        missing(ui, app, "PBC");
        return;
    };
    kv(ui, "Flash size", format!("{} bytes", snap.flash_size));
    kv(
        ui,
        "Flash dirty",
        if snap.flash_dirty { "yes" } else { "no" }.to_string(),
    );
    kv(ui, "SPI control", format!("0x{:08X}", snap.spi_control));
    kv(ui, "DMA flash addr", format!("0x{:08X}", snap.dma_flash_addr));
    kv(ui, "DMA SRAM addr", format!("0x{:08X}", snap.dma_data_addr));
    ui.separator();
    if ui.button("Load flash from file…").clicked() {
        if let Some(path) = rfd::FileDialog::new().pick_file() {
            inject(app, PeripheralEvent::Pbc(PbcEvent::LoadFlashFromFile(path)));
        }
    }
    if ui.button("Dump flash to file…").clicked() {
        if let Some(path) = rfd::FileDialog::new().save_file() {
            inject(app, PeripheralEvent::Pbc(PbcEvent::DumpFlashToFile(path)));
        }
    }
}

// ---------------------------------------------------------------------------
// BSC / I2C
// ---------------------------------------------------------------------------

fn draw_bsc(ui: &mut egui::Ui, app: &EmulatorApp) {
    let Some(PeripheralSnapshot::Bsc(snap)) =
        find_snapshot(app, |p| matches!(p, PeripheralSnapshot::Bsc(_)))
    else {
        missing(ui, app, "BSC/I2C");
        return;
    };
    kv(ui, "Busy", if snap.busy { "yes" } else { "no" }.to_string());
    kv(
        ui,
        "Last device addr",
        format!("0x{:02X}", snap.last_device_addr),
    );
    kv(
        ui,
        "Last word addr",
        format!("0x{:04X}", snap.last_word_addr),
    );
    ui.separator();
    if ui.button("Force NACK").clicked() {
        inject(app, PeripheralEvent::Bsc(BscEvent::ForceNack));
    }
    if ui.button("Reset controller").clicked() {
        inject(app, PeripheralEvent::Bsc(BscEvent::Reset));
    }
}

// ---------------------------------------------------------------------------
// SFP
// ---------------------------------------------------------------------------

fn draw_sfp(ui: &mut egui::Ui, app: &mut EmulatorApp) {
    let snap = match find_snapshot(app, |p| matches!(p, PeripheralSnapshot::Sfp(_))) {
        Some(PeripheralSnapshot::Sfp(snap)) => snap.clone(),
        _ => {
            missing(ui, app, "SFP");
            return;
        }
    };
    let scratch = &mut app.periph_scratch;
    if scratch.sfp_temp_c256.is_none() {
        scratch.sfp_temp_c256 = Some(snap.temperature_c256);
        scratch.sfp_vcc_uv = Some(snap.vcc_uv);
        scratch.sfp_tx_bias_ua = Some(snap.tx_bias_ua);
        scratch.sfp_tx_power_uw = Some(snap.tx_power_uw);
        scratch.sfp_rx_power_uw = Some(snap.rx_power_uw);
    }

    kv(ui, "Vendor", snap.vendor.clone());
    kv(ui, "Serial", snap.serial.clone());
    kv(ui, "Part number", snap.part_number.clone());
    ui.separator();

    let mut temp = scratch.sfp_temp_c256.unwrap_or(snap.temperature_c256);
    let mut vcc = scratch.sfp_vcc_uv.unwrap_or(snap.vcc_uv);
    let mut tx_bias = scratch.sfp_tx_bias_ua.unwrap_or(snap.tx_bias_ua);
    let mut tx_pow = scratch.sfp_tx_power_uw.unwrap_or(snap.tx_power_uw);
    let mut rx_pow = scratch.sfp_rx_power_uw.unwrap_or(snap.rx_power_uw);

    ui.horizontal(|ui| {
        ui.label("Temperature (1/256 °C):");
        ui.add(egui::DragValue::new(&mut temp));
    });
    ui.horizontal(|ui| {
        ui.label("Vcc (µV):");
        ui.add(egui::DragValue::new(&mut vcc));
    });
    ui.horizontal(|ui| {
        ui.label("TX bias (µA):");
        ui.add(egui::DragValue::new(&mut tx_bias));
    });
    ui.horizontal(|ui| {
        ui.label("TX power (µW):");
        ui.add(egui::DragValue::new(&mut tx_pow));
    });
    ui.horizontal(|ui| {
        ui.label("RX power (µW):");
        ui.add(egui::DragValue::new(&mut rx_pow));
    });
    scratch.sfp_temp_c256 = Some(temp);
    scratch.sfp_vcc_uv = Some(vcc);
    scratch.sfp_tx_bias_ua = Some(tx_bias);
    scratch.sfp_tx_power_uw = Some(tx_pow);
    scratch.sfp_rx_power_uw = Some(rx_pow);

    if ui.button("Apply DDM").clicked() {
        inject(app, PeripheralEvent::Sfp(SfpEvent::SetTemperatureC256(temp)));
        inject(app, PeripheralEvent::Sfp(SfpEvent::SetVccUv(vcc)));
        inject(app, PeripheralEvent::Sfp(SfpEvent::SetTxBiasUa(tx_bias)));
        inject(app, PeripheralEvent::Sfp(SfpEvent::SetTxPowerUw(tx_pow)));
        inject(app, PeripheralEvent::Sfp(SfpEvent::SetRxPowerUw(rx_pow)));
    }
}

// ---------------------------------------------------------------------------
// SerDes
// ---------------------------------------------------------------------------

fn draw_serdes(ui: &mut egui::Ui, app: &EmulatorApp) {
    let Some(PeripheralSnapshot::SerDes(snap)) =
        find_snapshot(app, |p| matches!(p, PeripheralSnapshot::SerDes(_)))
    else {
        missing(ui, app, "SerDes");
        return;
    };
    kv(ui, "Error status", format!("0x{:08X}", snap.error_status));
    ui.separator();
    for (idx, lane) in snap.lanes.iter().enumerate() {
        egui::Frame::group(ui.style()).show(ui, |ui| {
            ui.horizontal(|ui| {
                ui.strong(format!("Lane {idx}"));
                ui.separator();
                ui.label(format!(
                    "{}",
                    match lane.speed {
                        LaneSpeed::OneGigabit => "1G",
                        LaneSpeed::TenGigabit => "10G",
                        LaneSpeed::Pon1G => "PON-1G",
                        LaneSpeed::Pon10G => "PON-10G",
                    }
                ));
            });
            ui.horizontal(|ui| {
                let mut enabled = lane.enabled;
                if ui.checkbox(&mut enabled, "Enabled").changed() {
                    inject(
                        app,
                        PeripheralEvent::SerDes(SerDesEvent::SetLaneEnabled(idx as u8, enabled)),
                    );
                }
                let mut locked = lane.locked;
                if ui.checkbox(&mut locked, "Locked").changed() {
                    inject(
                        app,
                        PeripheralEvent::SerDes(SerDesEvent::SetLinkLocked(idx as u8, locked)),
                    );
                }
                let mut los = lane.rx_los;
                if ui.checkbox(&mut los, "RX LOS").changed() {
                    inject(
                        app,
                        PeripheralEvent::SerDes(SerDesEvent::InjectRxLos(idx as u8, los)),
                    );
                }
            });
        });
    }
    if ui.button("Clear error status").clicked() {
        inject(app, PeripheralEvent::SerDes(SerDesEvent::ClearErrorStatus));
    }
}

// ---------------------------------------------------------------------------
// EPON MAC
// ---------------------------------------------------------------------------

fn draw_epon(ui: &mut egui::Ui, app: &mut EmulatorApp) {
    let snap = match find_snapshot(app, |p| matches!(p, PeripheralSnapshot::EponMac(_))) {
        Some(PeripheralSnapshot::EponMac(snap)) => snap.clone(),
        _ => {
            missing(ui, app, "EPON MAC");
            return;
        }
    };
    kv(ui, "CHIP_ID", format!("0x{:08X}", snap.chip_id));
    kv(ui, "CHIP_REV", format!("0x{:08X}", snap.chip_rev));
    kv(
        ui,
        "LLID active bitmap",
        format!("0x{:08X}", snap.llid_active_bitmap),
    );
    kv(ui, "RX grant mask", format!("0x{:08X}", snap.rx_grant_mask));
    kv(ui, "TX grant mask", format!("0x{:08X}", snap.tx_grant_mask));
    kv(ui, "IRQ mask", format!("0x{:08X}", snap.irq_mask));
    ui.separator();
    ui.horizontal(|ui| {
        ui.label("LLID:");
        ui.add(egui::DragValue::new(&mut app.periph_scratch.epon_llid).range(0..=31));
        let bit = 1u32 << app.periph_scratch.epon_llid;
        let active = snap.llid_active_bitmap & bit != 0;
        if ui.button(if active { "Deactivate" } else { "Activate" }).clicked() {
            inject(
                app,
                PeripheralEvent::Epon(EponEvent::SetLlidActive(app.periph_scratch.epon_llid, !active)),
            );
        }
        if ui.button("Inject IRQ").clicked() {
            inject(
                app,
                PeripheralEvent::Epon(EponEvent::InjectLlidInterrupt(app.periph_scratch.epon_llid)),
            );
        }
    });
    if ui.button("Reset counters").clicked() {
        inject(app, PeripheralEvent::Epon(EponEvent::ResetCounters));
    }
}

// ---------------------------------------------------------------------------
// MACsec
// ---------------------------------------------------------------------------

fn draw_macsec(ui: &mut egui::Ui, app: &mut EmulatorApp) {
    let Some(PeripheralSnapshot::Macsec(snap)) =
        find_snapshot(app, |p| matches!(p, PeripheralSnapshot::Macsec(_)))
    else {
        missing(ui, app, "MACsec");
        return;
    };
    let snap = snap.clone();
    kv(ui, "Control", format!("0x{:08X}", snap.control));
    kv(ui, "Enable mode", format!("0x{:08X}", snap.enable_mode));
    kv(ui, "Key engine busy", snap.key_engine_busy.to_string());
    kv(ui, "PN threshold busy", snap.pn_threshold_busy.to_string());
    kv(ui, "SA slots programmed", snap.sa_slots_programmed.to_string());
    kv(
        ui,
        "PN overflow mask",
        format!("0x{:08X}", snap.pn_overflow_mask),
    );
    ui.separator();
    ui.horizontal(|ui| {
        ui.label("SA slot:");
        ui.add(egui::DragValue::new(&mut app.periph_scratch.macsec_sa_slot).range(0..=31));
        if ui.button("Inject PN overflow").clicked() {
            inject(
                app,
                PeripheralEvent::Macsec(MacsecEvent::InjectPnOverflow(
                    app.periph_scratch.macsec_sa_slot,
                )),
            );
        }
    });
    if ui.button("Reset SA table").clicked() {
        inject(app, PeripheralEvent::Macsec(MacsecEvent::ResetSaTable));
    }
}

// ---------------------------------------------------------------------------
// DMA
// ---------------------------------------------------------------------------

fn draw_dma(ui: &mut egui::Ui, app: &mut EmulatorApp) {
    let Some(PeripheralSnapshot::Dma(snap)) =
        find_snapshot(app, |p| matches!(p, PeripheralSnapshot::Dma(_)))
    else {
        missing(ui, app, "DMA");
        return;
    };
    let snap = snap.clone();
    kv(ui, "Channels enabled", format!("0x{:08X}", snap.channels_enabled));
    kv(ui, "Channels busy", format!("0x{:08X}", snap.channels_busy));
    kv(
        ui,
        "IRQ pending bitmap",
        format!("0x{:08X}", snap.irq_pending_bitmap),
    );
    ui.separator();
    ui.horizontal(|ui| {
        ui.label("Channel:");
        ui.add(egui::DragValue::new(&mut app.periph_scratch.dma_channel).range(0..=31));
        if ui.button("Inject queue entry").clicked() {
            inject(
                app,
                PeripheralEvent::Dma(DmaEvent::InjectQueueEntry(app.periph_scratch.dma_channel)),
            );
        }
        if ui.button("Inject bus error").clicked() {
            inject(
                app,
                PeripheralEvent::Dma(DmaEvent::InjectBusError(app.periph_scratch.dma_channel)),
            );
        }
    });
}

// ---------------------------------------------------------------------------
// Alarm
// ---------------------------------------------------------------------------

fn draw_alarm(ui: &mut egui::Ui, app: &mut EmulatorApp) {
    let Some(PeripheralSnapshot::Alarm(snap)) =
        find_snapshot(app, |p| matches!(p, PeripheralSnapshot::Alarm(_)))
    else {
        missing(ui, app, "Alarm");
        return;
    };
    let snap = snap.clone();
    ui.label(format!(
        "Forced opcodes: {}",
        snap.forced_opcodes
            .iter()
            .map(|o| format!("0x{:04X}", o))
            .collect::<Vec<_>>()
            .join(", "),
    ));
    ui.label(format!(
        "Live opcodes: {}",
        snap.live_opcodes
            .iter()
            .map(|o| format!("0x{:04X}", o))
            .collect::<Vec<_>>()
            .join(", "),
    ));
    ui.separator();
    ui.horizontal(|ui| {
        ui.label("Opcode (hex):");
        ui.add(
            egui::TextEdit::singleline(&mut app.periph_scratch.alarm_opcode)
                .desired_width(80.0)
                .hint_text("0x0042"),
        );
        let parsed = parse_hex_u16(&app.periph_scratch.alarm_opcode);
        if ui.button("Force pending").clicked() {
            if let Some(op) = parsed {
                inject(app, PeripheralEvent::Alarm(AlarmEvent::ForcePending(op)));
            }
        }
        if ui.button("Clear pending").clicked() {
            if let Some(op) = parsed {
                inject(app, PeripheralEvent::Alarm(AlarmEvent::ClearPending(op)));
            }
        }
    });
    if ui.button("Clear all forced").clicked() {
        inject(app, PeripheralEvent::Alarm(AlarmEvent::ClearAll));
    }
}

// ---------------------------------------------------------------------------
// Timer
// ---------------------------------------------------------------------------

fn draw_timer(ui: &mut egui::Ui, app: &mut EmulatorApp) {
    let Some(PeripheralSnapshot::Timer(snap)) =
        find_snapshot(app, |p| matches!(p, PeripheralSnapshot::Timer(_)))
    else {
        missing(ui, app, "Timer");
        return;
    };
    let snap = snap.clone();
    kv(ui, "EPON counter", format!("0x{:08X}", snap.counter));
    kv(ui, "Prescaler", snap.prescaler.to_string());
    ui.separator();
    ui.horizontal(|ui| {
        ui.label("Prescaler:");
        ui.add(
            egui::TextEdit::singleline(&mut app.periph_scratch.timer_prescaler)
                .desired_width(80.0),
        );
        if ui.button("Apply").clicked() {
            if let Ok(v) = app.periph_scratch.timer_prescaler.parse::<u32>() {
                inject(app, PeripheralEvent::Timer(TimerEvent::SetPrescaler(v)));
            }
        }
    });
    ui.horizontal(|ui| {
        ui.label("Counter:");
        ui.add(
            egui::TextEdit::singleline(&mut app.periph_scratch.timer_counter)
                .desired_width(100.0)
                .hint_text("0x00000000"),
        );
        if ui.button("Apply").clicked() {
            if let Some(v) = parse_hex_u32(&app.periph_scratch.timer_counter) {
                inject(app, PeripheralEvent::Timer(TimerEvent::SetCounter(v)));
            }
        }
    });
}

// ---------------------------------------------------------------------------
// eFuse / UDR
// ---------------------------------------------------------------------------

fn draw_efuse(ui: &mut egui::Ui, app: &EmulatorApp) {
    let Some(PeripheralSnapshot::Efuse(snap)) =
        find_snapshot(app, |p| matches!(p, PeripheralSnapshot::Efuse(_)))
    else {
        missing(ui, app, "eFuse/UDR");
        return;
    };
    kv(ui, "UDR status", format!("0x{:08X}", snap.udr_status));
    kv(ui, "Clock toggles", snap.clock_toggles.to_string());
    ui.separator();
    if ui.button("Load 80-byte snapshot from file…").clicked() {
        if let Some(path) = rfd::FileDialog::new().pick_file() {
            if let Ok(bytes) = std::fs::read(&path) {
                inject(app, PeripheralEvent::Efuse(EfuseEvent::SetSnapshot(bytes)));
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Fatal filter
// ---------------------------------------------------------------------------

fn draw_fatal(ui: &mut egui::Ui, app: &mut EmulatorApp) {
    let Some(PeripheralSnapshot::FatalFilter(snap)) =
        find_snapshot(app, |p| matches!(p, PeripheralSnapshot::FatalFilter(_)))
    else {
        missing(ui, app, "Fatal filter");
        return;
    };
    let snap = snap.clone();
    kv(ui, "Fatal status", format!("0x{:08X}", snap.fatal_status));
    kv(
        ui,
        "Link-up bitmap",
        format!("0x{:08X}", snap.link_up_bitmap),
    );
    ui.separator();
    ui.horizontal(|ui| {
        ui.label("Inject mask (hex):");
        ui.add(
            egui::TextEdit::singleline(&mut app.periph_scratch.fatal_inject_mask)
                .desired_width(120.0)
                .hint_text("0x00000001"),
        );
        if ui.button("Inject fatal").clicked() {
            if let Some(m) = parse_hex_u32(&app.periph_scratch.fatal_inject_mask) {
                inject(
                    app,
                    PeripheralEvent::FatalFilter(FatalFilterEvent::InjectFatal(m)),
                );
            }
        }
    });
    if ui.button("Clear fatal").clicked() {
        inject(
            app,
            PeripheralEvent::FatalFilter(FatalFilterEvent::ClearFatal),
        );
    }
    ui.horizontal(|ui| {
        ui.label("PHY link:");
        ui.add(egui::DragValue::new(&mut app.periph_scratch.fatal_link_idx).range(0..=31));
        ui.checkbox(&mut app.periph_scratch.fatal_link_up, "up");
        if ui.button("Apply").clicked() {
            inject(
                app,
                PeripheralEvent::FatalFilter(FatalFilterEvent::SetLinkUp(
                    app.periph_scratch.fatal_link_idx,
                    app.periph_scratch.fatal_link_up,
                )),
            );
        }
    });
}

// ---------------------------------------------------------------------------
// MPCP / NCO (peek fallback — no typed snapshot)
// ---------------------------------------------------------------------------

fn draw_mpcp(ui: &mut egui::Ui, app: &EmulatorApp) {
    ui.label("MPCP claim regions (peek_word hex dump):");
    ui.separator();
    const REGIONS: &[(u32, u32)] = &[
        (0x0100_0120, 0x0100_0140),
        (0x0100_0320, 0x0100_0328),
        (0x0100_1180, 0x0100_11C0),
        (0x0100_1268, 0x0100_1390),
    ];
    let bank = app.handle.bank.read();
    for (start, end) in REGIONS {
        ui.monospace(format!("-- 0x{:08X}..0x{:08X} --", start, end));
        let mut addr = *start;
        while addr < *end {
            let v = bank.peek_word(addr).unwrap_or(0);
            ui.monospace(format!("  0x{:08X}: 0x{:08X}", addr, v));
            addr += 4;
        }
    }
}

fn draw_nco(ui: &mut egui::Ui, app: &EmulatorApp) {
    ui.label("NCO TX mode register (peek_word):");
    ui.separator();
    let bank = app.handle.bank.read();
    let addr = 0x0100_0F80u32;
    let v = bank.peek_word(addr).unwrap_or(0);
    ui.monospace(format!("0x{:08X}: 0x{:08X}", addr, v));
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

fn kv(ui: &mut egui::Ui, label: &str, value: String) {
    ui.horizontal(|ui| {
        ui.label(egui::RichText::new(format!("{label}:")).monospace());
        ui.monospace(value);
    });
}

fn parse_hex_u16(s: &str) -> Option<u16> {
    let t = s.trim().trim_start_matches("0x").trim_start_matches("0X");
    u16::from_str_radix(t, 16).ok()
}

fn parse_hex_u32(s: &str) -> Option<u32> {
    let t = s.trim().trim_start_matches("0x").trim_start_matches("0X");
    u32::from_str_radix(t, 16).ok()
}
