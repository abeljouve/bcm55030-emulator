//! Peripheral bank — the central MMIO routing and tick orchestrator.
//!
//! The bank owns the full set of BCM55030 peripheral models by value and
//! routes every MMIO load / store from the CPU to the peripheral that
//! claims the target address. It also exposes the mpsc channel that UI
//! and `main.rs` use to feed bytes into the UART receive path.
//!
//! The bank is designed to live behind `Arc<parking_lot::RwLock<...>>`
//! so the UI thread (read lock at 60 Hz) and the CPU thread (write lock
//! per MMIO access) can share it without contention on the SRAM fast
//! path — which does not touch the bank at all.

use std::collections::{HashMap, VecDeque};
use std::io::{BufWriter, Write};
use std::sync::mpsc;

use parking_lot::Mutex;

use crate::cpu::exception::Exception;
use crate::soc::alarm_events::AlarmEvents;
use crate::soc::bsc_i2c::BscI2c;
use crate::soc::lane_bus::LaneBus;
use crate::soc::dma::DmaChannelController;
use crate::soc::efuse_udr::EfuseUdr;
use crate::soc::epon_mac::EponMac;
use crate::soc::fatal_filter::FatalFilter;
use crate::soc::macsec::Macsec;
use crate::soc::mpcp::Mpcp;
use crate::soc::nco::Nco;
use crate::soc::olt::Olt;
use crate::soc::pbc::Pbc;
use crate::soc::peripheral::{
    DatapathOp, Peripheral, PeripheralEvent, PeripheralId, PeripheralSnapshot,
};
use crate::soc::scenario::ScenarioEngine;
use crate::soc::serdes::SerDes;
use crate::soc::sysreg_shim::SysregShim;
use crate::soc::timer::EponTimer;
use crate::soc::uart::Uart;
use crate::soc::vlan_lue::VlanLue;

/// CPU instruction ticks between bank tick invocations. Higher = less
/// contention but coarser peripheral advancement. The EPON free-running
/// counter assumes 64 — keep the default here.
pub const BANK_TICK_PRESCALER: u64 = 64;

/// Per-address last-access record for the `explain_mmio` MCP tool.
#[derive(Clone, Debug)]
pub struct LastAccessInfo {
    pub pc: u32,
    pub blink: u32,
    pub insn: u64,
    pub direction: &'static str,
    pub value: u32,
}

const LAST_ACCESS_MAX: usize = 4096;

/// Per-access MMIO history entry for the ring buffer.
#[derive(Clone, Debug)]
pub struct MmioHistoryEntry {
    pub insn: u64,
    pub pc: u32,
    pub blink: u32,
    pub address: u32,
    pub value: u32,
    pub direction: &'static str,
    pub width: &'static str,
    pub peripheral: &'static str,
}

pub const DEFAULT_MMIO_HISTORY_SIZE: usize = 8192;

/// Aggregated MMIO trace entry (replaces the one previously owned by
/// `mmio::MmioController`). Used by `--dump-mmio-trace` / cold-boot
/// trace capture.
#[derive(Default, Clone, Debug)]
pub struct MmioTraceEntry {
    pub peripheral: &'static str,
    pub reads: u64,
    pub writes: u64,
    pub last_read_value: u32,
    pub last_write_value: u32,
    pub first_pc: u32,
    pub first_insn: u64,
    /// bit 0 = byte access, bit 1 = half, bit 2 = word
    pub access_widths: u8,
}

/// Per-access sequential MMIO trace (for `--trace-mmio-seq`).
///
/// Holds an open file handle (wrapped in a `BufWriter`) and an optional
/// set of address ranges. When ranges is non-empty, only accesses whose
/// address falls inside at least one range are written. When the `Vec` is
/// empty every access is recorded.
pub struct SeqTrace {
    writer: BufWriter<std::fs::File>,
    /// Filter ranges `[start, end)`. Empty = record everything.
    ranges: Vec<(u32, u32)>,
}

impl SeqTrace {
    /// Open `path` for writing and attach the optional filter ranges.
    pub fn open(path: &str, ranges: Vec<(u32, u32)>) -> std::io::Result<Self> {
        let file = std::fs::File::create(path)?;
        Ok(Self {
            writer: BufWriter::with_capacity(64 * 1024, file),
            ranges,
        })
    }

    /// Returns `true` when `addr` passes the configured filter.
    #[inline]
    pub fn addr_matches(&self, addr: u32) -> bool {
        if self.ranges.is_empty() {
            return true;
        }
        self.ranges.iter().any(|(start, end)| addr >= *start && addr < *end)
    }

    /// Write one JSON Lines entry. Never panics — silently ignores write errors.
    #[inline]
    pub fn emit(&mut self, cycle: u64, pc: u32, addr: u32, value: u32, rw: &str, width: &str, periph: &str) {
        let _ = writeln!(
            self.writer,
            r#"{{"cycle":{cycle},"pc":{pc},"addr":{addr},"value":{value},"rw":"{rw}","width":"{width}","periph":"{periph}"}}"#,
        );
    }

    /// Flush the internal buffer. Call at clean exit.
    pub fn flush(&mut self) {
        let _ = self.writer.flush();
    }
}

/// Boot mode — controls whether peripherals apply the post-boot snapshot
/// in their `reset_warm()` path or start at cold-reset values.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BootMode {
    Cold,
    Warm,
}

pub struct PeripheralBank {
    pub uart: Uart,
    pub pbc: Pbc,
    pub bsc_i2c: BscI2c,
    /// Lane 8 indirect bus — MPCP SerDes register file.
    pub mpcp_bus: LaneBus,
    pub serdes: SerDes,
    pub epon_mac: EponMac,
    pub macsec: Macsec,
    pub dma: DmaChannelController,
    pub alarm_events: AlarmEvents,
    pub timer: EponTimer,
    pub efuse_udr: EfuseUdr,
    pub fatal_filter: FatalFilter,
    pub mpcp: Mpcp,
    pub nco: Nco,
    pub vlan_lue: VlanLue,
    pub olt: Olt,

    pending_cache_inv: Vec<DatapathOp>,

    /// Scenario engine — MMIO overrides, scheduled events, and MMIO
    /// watchpoints.  Overrides are checked BEFORE the peripheral
    /// dispatch chain.  All stimulus is HW-level, never firmware.
    pub scenario: ScenarioEngine,

    /// Temporary residual-plus-legacy-arms for the SYSREG range
    /// (`0x01000000..0x01003800`). Hosts every stub that has not yet been
    /// carved into its own peripheral file — CHIP_ID, LLID IRQ forced 0,
    /// SerDes forced bits, I²C UDR bit-bang, DMA queue drain, alarm
    /// counters, etc. This struct shrinks as Sessions 2–7 land their
    /// dedicated peripherals.
    pub sysreg: SysregShim,

    uart_rx_sender: mpsc::Sender<u8>,
    /// Wrapped in `parking_lot::Mutex` so `PeripheralBank` is
    /// `Sync`. `mpsc::Receiver` is `Send` but not `Sync`, which
    /// would prevent `Arc<RwLock<PeripheralBank>>` from crossing
    /// thread boundaries in the UI / MCP workers. Contention is
    /// zero — only the bank's own `tick()` drains it, always
    /// while the bank is write-locked — so the mutex acquisition
    /// stays in parking_lot's fast path (no syscall, no
    /// poisoning).
    uart_rx_receiver: Mutex<mpsc::Receiver<u8>>,

    /// Current CPU context for trace entries and watchpoint messages.
    pub current_pc: u32,
    pub current_blink: u32,
    pub current_insn: u64,

    /// Optional aggregated MMIO trace for `--dump-mmio-trace`.
    pub mmio_trace: Option<HashMap<u32, MmioTraceEntry>>,

    /// Verbose trace of every MMIO access (unbounded — only enable via
    /// `--trace-mmio`). Mirrors the old `MmioController::trace` flag.
    pub trace: bool,

    /// Aggregated IRQ pending mask from peripherals. `Cpu::step()` ORs
    /// this into `aux_irq_pending` after each bank tick. Session 1 only
    /// UART contributes; other peripherals will add their bits once they
    /// land.
    pub irq_pending: u32,

    /// Boot mode — peripherals snapshot this during construction.
    pub boot_mode: BootMode,

    /// Audit 2.2: when `true`, MMIO accesses that no peripheral
    /// claims return [`Exception::MemoryError`] instead of silently
    /// reading as zero. Off by default; opt in via
    /// `--unmapped-exception` on the main binary to surface
    /// unmodelled firmware probes.
    pub unmapped_exception: bool,

    /// Per-address last-access cache for `explain_mmio`. Bounded to
    /// `LAST_ACCESS_MAX` entries — evicts oldest when full.
    pub last_access: HashMap<u32, LastAccessInfo>,

    /// Ring buffer of recent MMIO accesses. Bounded to
    /// `mmio_history_max` entries. Zero overhead when empty (size=0).
    pub mmio_history: VecDeque<MmioHistoryEntry>,
    pub mmio_history_max: usize,

    /// Per-access sequential MMIO trace (`--trace-mmio-seq`). `None`
    /// by default — zero overhead when disabled. When present, every
    /// successful MMIO access appends a JSON Lines entry.
    pub seq_trace: Option<SeqTrace>,
}

impl PeripheralBank {
    pub fn new(boot_mode: BootMode) -> Self {
        let (tx, rx) = mpsc::channel::<u8>();
        let mut bank = Self {
            uart: Uart::new(),
            pbc: Pbc::new(),
            bsc_i2c: BscI2c::new(),
            mpcp_bus: {
                let mut bus = LaneBus::new(0x0100_0118, 2);
                bus.apply_init(super::mmio_init::SYSREG_INIT_VALUES);
                bus
            },
            serdes: SerDes::new(),
            epon_mac: EponMac::new(),
            macsec: Macsec::new(),
            dma: DmaChannelController::new(),
            alarm_events: AlarmEvents::new(),
            timer: EponTimer::new(),
            efuse_udr: EfuseUdr::new(),
            fatal_filter: FatalFilter::new(),
            mpcp: Mpcp::new(),
            nco: Nco::new(),
            vlan_lue: VlanLue::new(),
            olt: Olt::new(),
            pending_cache_inv: Vec::new(),
            scenario: ScenarioEngine::new(),
            sysreg: SysregShim::new(),
            uart_rx_sender: tx,
            uart_rx_receiver: Mutex::new(rx),
            current_pc: 0,
            current_blink: 0,
            current_insn: 0,
            mmio_trace: None,
            trace: false,
            irq_pending: 0,
            boot_mode,
            unmapped_exception: false,
            last_access: HashMap::new(),
            mmio_history: VecDeque::with_capacity(DEFAULT_MMIO_HISTORY_SIZE),
            mmio_history_max: DEFAULT_MMIO_HISTORY_SIZE,
            seq_trace: None,
        };
        // Apply the requested reset flavour.
        match boot_mode {
            BootMode::Cold => bank.reset_cold(),
            BootMode::Warm => bank.reset_warm(),
        }
        bank
    }

    /// Return a cloneable sender that pushes bytes into the UART receive
    /// path. The main loop (`main.rs`) and the future UI both acquire a
    /// clone at startup and then feed bytes without touching the bank
    /// directly.
    pub fn uart_rx_sender(&self) -> mpsc::Sender<u8> {
        self.uart_rx_sender.clone()
    }

    /// Drain the mpsc channel into the UART receive queue. Called from
    /// [`tick`] before other peripherals tick, so UART IRQ bits that
    /// arise from newly-received bytes are visible in the same pass.
    fn drain_uart_channel(&mut self) {
        while let Ok(b) = self.uart_rx_receiver.lock().try_recv() {
            self.uart.push_rx_byte(b);
        }
    }

    /// Advance all peripherals by `cpu_instructions` CPU cycles.
    /// Called from `Cpu::step()` once per `BANK_TICK_PRESCALER`.
    pub fn tick(&mut self, cpu_instructions: u64) {
        self.drain_uart_channel();
        self.uart.tick(cpu_instructions);
        self.pbc.tick(cpu_instructions);
        self.bsc_i2c.tick(cpu_instructions);
        self.mpcp_bus.tick();
        self.serdes.tick(cpu_instructions);
        self.epon_mac.tick(cpu_instructions);
        self.macsec.tick(cpu_instructions);
        self.dma.tick(cpu_instructions);
        self.alarm_events.tick(cpu_instructions);
        self.timer.tick(cpu_instructions);
        self.efuse_udr.tick(cpu_instructions);
        self.fatal_filter.tick(cpu_instructions);
        self.mpcp.tick(cpu_instructions);
        self.nco.tick(cpu_instructions);
        self.vlan_lue.tick(cpu_instructions);
        self.olt.tick(cpu_instructions);
        // Drain OLT RX frames into the mailbox engine. We load into
        // word_index=0 (bitmap at 0x01001438) since that's the most
        // common queue. The frames are also available from any
        // word_index via the CMD/STATUS FIFO intercept.
        self.olt.load_frames_into_mailbox(0);
        // Sync OLT bitmaps into the epon_mac LLID backing store.
        // Only set when mailbox_pending has unread frames. Once the
        // firmware issues a CMD write and the frame moves to the FIFO,
        // the bitmap clears so the firmware doesn't try to read a
        // second non-existent frame.
        if self.olt.link_change_pending {
            self.olt.link_change_pending = false;
            self.epon_mac.set_1g_link_change_bit();
            self.epon_mac.set_phy_link_status_bit();
            self.pending_cache_inv.push(
                DatapathOp::CacheInvalidate { addr: 0x0100_0410 },
            );
            self.pending_cache_inv.push(
                DatapathOp::CacheInvalidate { addr: 0x0100_0E04 },
            );
        }
        if self.olt.config.enabled && self.olt.link_up {
            self.epon_mac.set_discovery_status_bit();
            if self.pending_cache_inv.is_empty() {
                self.pending_cache_inv.push(
                    DatapathOp::CacheInvalidate { addr: 0x0100_1040 },
                );
            }
        }
        if self.olt.config.enabled {
            let bmp = if !self.olt.mailbox_pending.is_empty() {
                0xFFFF_FFFFu32
            } else {
                0
            };
            for wi in 0..6u32 {
                let addr = 0x0100_1438 + wi * 0x200;
                if addr < 0x0100_2000 {
                    self.epon_mac.poke_llid_store(addr, bmp);
                }
            }
        }
        self.sysreg.tick(cpu_instructions);
        self.scenario.tick(cpu_instructions);

        // Aggregate IRQ pending bits. UART is the only v1 contributor
        // (IRQ 5, level 1). Other peripherals will add their bits once
        // they land.
        self.irq_pending = self.uart.irq_pending();
    }

    /// Process deferred writes from scenario effects.
    fn drain_scenario_deferred(&mut self) {
        let writes = self.scenario.take_deferred_writes();
        for w in writes {
            let _ = self.write_word_inner(w.address, w.value);
        }
    }

    /// Check if the scenario engine requested a pause.
    pub fn scenario_pause_requested(&mut self) -> bool {
        self.scenario.take_pause()
    }

    /// Cold reset — zeros volatile state in all peripherals. Called on
    /// FLAG 1 reboot and at startup in `--cold-boot` mode.
    pub fn reset_cold(&mut self) {
        self.uart.reset_cold();
        self.pbc.reset_cold();
        self.bsc_i2c.reset_cold();
        self.mpcp_bus.reset();
        self.mpcp_bus.apply_init(super::mmio_init::SYSREG_INIT_VALUES);
        self.serdes.reset_cold();
        self.epon_mac.reset_cold();
        self.macsec.reset_cold();
        self.dma.reset_cold();
        self.alarm_events.reset_cold();
        self.timer.reset_cold();
        self.efuse_udr.reset_cold();
        self.fatal_filter.reset_cold();
        self.mpcp.reset_cold();
        self.nco.reset_cold();
        self.vlan_lue.reset_cold();
        self.olt.reset_cold();
        self.sysreg.reset_cold();
        self.irq_pending = 0;
        self.current_pc = 0;
        self.current_blink = 0;
        self.current_insn = 0;
    }

    /// Warm reset — loads the post-boot snapshot on top of cold reset.
    pub fn reset_warm(&mut self) {
        self.uart.reset_warm();
        self.pbc.reset_warm();
        self.bsc_i2c.reset_warm();
        self.mpcp_bus.reset();
        self.mpcp_bus.apply_init(super::mmio_init::SYSREG_INIT_VALUES);
        self.serdes.reset_warm();
        self.epon_mac.reset_warm();
        self.macsec.reset_warm();
        self.dma.reset_warm();
        self.alarm_events.reset_warm();
        self.timer.reset_warm();
        self.efuse_udr.reset_warm();
        self.fatal_filter.reset_warm();
        self.mpcp.reset_warm();
        self.nco.reset_warm();
        self.vlan_lue.reset_warm();
        self.olt.reset_warm();
        self.sysreg.reset_warm();
        self.irq_pending = 0;
        self.current_pc = 0;
        self.current_blink = 0;
        self.current_insn = 0;
    }

    /// Burst controller trigger: the firmware pulsed bit 11 of
    /// PON_LANE_CONFIG_GRP1 (0x010001B0). On real silicon this fires
    /// whatever MPCP frame is programmed in the TX registers. The
    /// emulator synthesizes the frame and delivers it to the OLT model.
    /// Burst controller trigger: the firmware pulsed bit 11 of
    /// PON_LANE_CONFIG_GRP1 (0x010001B0). Build the MPCP frame from
    /// the lane 8 indirect register file and deliver to the OLT.
    fn handle_mpcp_burst_trigger(&mut self) {
        if !self.olt.link_up {
            return;
        }
        // Read MPCP TX config from lane 8 indirect register file.
        // Regs 0x50-0x53 are programmed by mpcp_build_register_frame.
        let reg50 = self.mpcp_bus.reg(0x50);
        let reg51 = self.mpcp_bus.reg(0x51);
        let _reg52 = self.mpcp_bus.reg(0x52);

        // Determine MPCP opcode from reg 0x51 grant flags.
        // Bit 4 = discovery_info present → REGISTER_REQ (opcode 4).
        // Otherwise default to REGISTER_REQ for now.
        let opcode: u16 = if reg51 & 0x10 != 0 || reg50 == 0 { 0x0004 } else { 0x0004 };

        let onu_mac = self.olt.config.onu_mac_override
            .unwrap_or([0x02, 0x00, 0x00, 0x01, 0x02, 0x03]);
        let ts = self.olt.mpcp_timestamp;

        let mut frame = Vec::with_capacity(64);
        // DA: slow protocol multicast (IEEE 802.3ah)
        frame.extend_from_slice(&[0x01, 0x80, 0xC2, 0x00, 0x00, 0x01]);
        // SA: ONU MAC
        frame.extend_from_slice(&onu_mac);
        // EtherType: 0x8808 (MPCP)
        frame.extend_from_slice(&[0x88, 0x08]);
        // Opcode
        frame.extend_from_slice(&opcode.to_be_bytes());
        // Timestamp (4 bytes BE)
        frame.extend_from_slice(&ts.to_be_bytes());
        // Pending grants + flags + padding to 64 bytes
        while frame.len() < 64 {
            frame.push(0x00);
        }

        eprintln!(
            "[BCM55030] MPCP burst trigger — opcode 0x{:04X}, reg50=0x{:02X} reg51=0x{:02X}, ONU MAC {:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X}",
            opcode, reg50, reg51,
            onu_mac[0], onu_mac[1], onu_mac[2], onu_mac[3], onu_mac[4], onu_mac[5]
        );
        self.olt.handle_tx_frame(&frame);
    }

    /// Update the CPU context fields in one shot. Called by `Cpu::step()`
    /// before every MMIO access so trace entries can show the touching
    /// PC / blink / insn count.
    pub fn update_cpu_context(&mut self, pc: u32, blink: u32, insn: u64) {
        self.current_pc = pc;
        self.current_blink = blink;
        self.current_insn = insn;
        self.sysreg.update_cpu_context(pc, insn);
    }

    #[inline]
    fn record_history(&mut self, addr: u32, value: u32, direction: &'static str, width: &'static str, peripheral: &'static str) {
        if self.mmio_history_max == 0 { return; }
        if self.mmio_history.len() >= self.mmio_history_max {
            self.mmio_history.pop_front();
        }
        self.mmio_history.push_back(MmioHistoryEntry {
            insn: self.current_insn,
            pc: self.current_pc,
            blink: self.current_blink,
            address: addr,
            value,
            direction,
            width,
            peripheral,
        });
    }

    #[inline]
    fn record_last_access(&mut self, addr: u32, value: u32, direction: &'static str) {
        if self.last_access.len() >= LAST_ACCESS_MAX && !self.last_access.contains_key(&addr) {
            if let Some(&oldest) = self.last_access.keys().next() {
                self.last_access.remove(&oldest);
            }
        }
        self.last_access.insert(addr, LastAccessInfo {
            pc: self.current_pc,
            blink: self.current_blink,
            insn: self.current_insn,
            direction,
            value,
        });
    }

    /// Emit one entry to the sequential MMIO trace if enabled and the
    /// address matches the configured filter. Inlined so the branch
    /// folds away at the call site when `seq_trace` is `None`.
    #[inline]
    fn seq_emit(&mut self, addr: u32, value: u32, rw: &str, width: &str, periph: &str) {
        if let Some(ref mut st) = self.seq_trace {
            if st.addr_matches(addr) {
                st.emit(self.current_insn, self.current_pc, addr, value, rw, width, periph);
            }
        }
    }

    /// Snapshot each peripheral for UI display. Cheap (shallow clones)
    /// so the UI can call it every frame under a short read lock.
    pub fn snapshot_all(&self) -> Vec<PeripheralSnapshot> {
        vec![
            self.uart.snapshot(),
            self.pbc.snapshot(),
            self.bsc_i2c.snapshot(),
            self.bsc_i2c.sfp_snapshot(),
            self.serdes.snapshot(),
            self.epon_mac.snapshot(),
            self.macsec.snapshot(),
            self.dma.snapshot(),
            self.alarm_events.snapshot(),
            self.timer.snapshot(),
            self.efuse_udr.snapshot(),
            self.fatal_filter.snapshot(),
            self.olt.snapshot(),
        ]
    }

    /// Dispatch a UI event to the peripheral that understands it.
    pub fn inject_event(&mut self, event: &PeripheralEvent) -> bool {
        let targets: [&mut dyn Peripheral; 14] = [
            &mut self.uart,
            &mut self.pbc,
            &mut self.bsc_i2c,
            &mut self.serdes,
            &mut self.epon_mac,
            &mut self.macsec,
            &mut self.dma,
            &mut self.alarm_events,
            &mut self.timer,
            &mut self.efuse_udr,
            &mut self.fatal_filter,
            &mut self.mpcp,
            &mut self.nco,
            &mut self.olt,
        ];
        for p in targets {
            if p.inject_event(event).is_ok() {
                return true;
            }
        }
        false
    }

    /// Drain pending datapath operations from every peripheral. The
    /// caller (`Memory::apply_datapath`) applies them to SRAM / flash
    /// after releasing the bank write lock.
    pub fn take_pending_datapath(&mut self) -> Vec<DatapathOp> {
        let mut ops = Vec::new();
        ops.append(&mut self.pbc.take_pending_datapath());
        ops.append(&mut self.pending_cache_inv);
        ops
    }

    /// PBC flash-write callback. `Memory::apply_datapath` collects the
    /// SRAM bytes and hands them back via this method for peripherals
    /// that emitted a `DatapathOp::FlashWrite`.
    pub fn complete_flash_write(
        &mut self,
        peripheral: PeripheralId,
        flash_addr: u32,
        data: &[u8],
    ) {
        match peripheral {
            PeripheralId::Pbc => self.pbc.complete_flash_write(flash_addr, data),
        }
    }

    /// Return the name of the peripheral that claims `addr`, if any.
    pub fn peripheral_for(&self, addr: u32) -> Option<&'static str> {
        if self.uart.claims(addr) { return Some("uart"); }
        if self.pbc.claims(addr) { return Some("pbc"); }
        if self.bsc_i2c.claims(addr) { return Some("bsc_i2c"); }
        if self.serdes.claims(addr) { return Some("serdes"); }
        if self.epon_mac.claims(addr) { return Some("epon_mac"); }
        if self.macsec.claims(addr) { return Some("macsec"); }
        if self.dma.claims(addr) { return Some("dma"); }
        if self.timer.claims(addr) { return Some("timer"); }
        if self.efuse_udr.claims(addr) { return Some("efuse_udr"); }
        if self.fatal_filter.claims(addr) { return Some("fatal_filter"); }
        if self.mpcp.claims(addr) { return Some("mpcp"); }
        if self.nco.claims(addr) { return Some("nco"); }
        if self.vlan_lue.claims(addr) { return Some("vlan_lue"); }
        if self.sysreg.claims(addr) { return Some("sysreg"); }
        None
    }

    pub fn capture_for_snapshot(&self) -> crate::emu::named_snapshot::PeripheralBankSaveState {
        crate::emu::named_snapshot::PeripheralBankSaveState {
            uart: self.uart.clone(),
            pbc: self.pbc.clone(),
            bsc_i2c: self.bsc_i2c.clone(),
            mpcp_bus: self.mpcp_bus.clone(),
            serdes: self.serdes.clone(),
            epon_mac: self.epon_mac.clone(),
            macsec: self.macsec.clone(),
            dma: self.dma.clone(),
            alarm_events: self.alarm_events.clone(),
            timer: self.timer.clone(),
            efuse_udr: self.efuse_udr.clone(),
            fatal_filter: self.fatal_filter.clone(),
            mpcp: self.mpcp.clone(),
            nco: self.nco.clone(),
            vlan_lue: self.vlan_lue.clone(),
            olt: self.olt.clone(),
            scenario: self.scenario.clone(),
            sysreg: self.sysreg.clone(),
        }
    }

    pub fn restore_from_snapshot(&mut self, state: crate::emu::named_snapshot::PeripheralBankSaveState) {
        self.uart = state.uart;
        self.pbc = state.pbc;
        self.bsc_i2c = state.bsc_i2c;
        self.mpcp_bus = state.mpcp_bus;
        self.serdes = state.serdes;
        self.epon_mac = state.epon_mac;
        self.macsec = state.macsec;
        self.dma = state.dma;
        self.alarm_events = state.alarm_events;
        self.timer = state.timer;
        self.efuse_udr = state.efuse_udr;
        self.fatal_filter = state.fatal_filter;
        self.mpcp = state.mpcp;
        self.nco = state.nco;
        self.vlan_lue = state.vlan_lue;
        self.olt = state.olt;
        self.scenario = state.scenario;
        self.sysreg = state.sysreg;
        self.irq_pending = 0;
        self.current_pc = 0;
        self.current_blink = 0;
        self.current_insn = 0;
    }

    // -------- MMIO routing --------

    /// Side-effect-free probe of `addr`, used by the MCP
    /// `peek_mmio` tool and by the future peripheral inspector's
    /// continuous polling. Falls back to the peripheral trait's
    /// default (`Ok(0)`) when no override exists — each peripheral
    /// adds real peek support incrementally.
    pub fn peek_word(&self, addr: u32) -> Result<u32, Exception> {
        if !self.scenario.overrides.is_empty() {
            if let Some(v) = self.scenario.overrides.peek_read(addr) {
                return Ok(v);
            }
        }
        if self.uart.claims(addr) {
            return self.uart.peek_word(addr);
        }
        if self.pbc.claims(addr) {
            return self.pbc.peek_word(addr);
        }
        if self.bsc_i2c.claims(addr) {
            return self.bsc_i2c.peek_word(addr);
        }
        if self.mpcp_bus.claims(addr) {
            let v = if addr == self.mpcp_bus.cmd_addr {
                self.mpcp_bus.peek_cmd()
            } else if addr == self.mpcp_bus.data_addr {
                self.mpcp_bus.read_data()
            } else {
                self.mpcp_bus.peek_stat()
            };
            return Ok(v);
        }
        if self.serdes.claims(addr) {
            return self.serdes.peek_word(addr);
        }
        if self.epon_mac.claims(addr) {
            return self.epon_mac.peek_word(addr);
        }
        if self.macsec.claims(addr) {
            return self.macsec.peek_word(addr);
        }
        if self.dma.claims(addr) {
            return self.dma.peek_word(addr);
        }
        if self.timer.claims(addr) {
            return self.timer.peek_word(addr);
        }
        if self.efuse_udr.claims(addr) {
            return self.efuse_udr.peek_word(addr);
        }
        if self.fatal_filter.claims(addr) {
            return self.fatal_filter.peek_word(addr);
        }
        if self.mpcp.claims(addr) {
            return self.mpcp.peek_word(addr);
        }
        if self.nco.claims(addr) {
            return self.nco.peek_word(addr);
        }
        if self.vlan_lue.claims(addr) {
            return self.vlan_lue.peek_word(addr);
        }
        // SysregShim residual has no peek path yet.
        Ok(0)
    }

    pub fn read_word(&mut self, addr: u32) -> Result<u32, Exception> {
        let result = self.read_word_inner(addr);
        let value = *result.as_ref().unwrap_or(&0);
        if result.is_ok() {
            self.record_last_access(addr, value, "read");
        }
        self.scenario.on_mmio_read(addr, value);
        self.drain_scenario_deferred();
        result
    }

    fn read_word_inner(&mut self, addr: u32) -> Result<u32, Exception> {
        if !self.scenario.overrides.is_empty() {
            if let Some(v) = self.scenario.overrides.try_read(addr) {
                self.record_history(addr, v, "read", "word", "override");
                self.seq_emit(addr, v, "r", "word", "override");
                return Ok(v);
            }
        }
        macro_rules! dispatch_rw {
            ($periph:expr, $name:expr) => {{
                let v = $periph.read_word(addr)?;
                self.record_history(addr, v, "read", "word", $name);
                self.seq_emit(addr, v, "r", "word", $name);
                return Ok(v);
            }};
        }
        // OLT mailbox CMD_STATUS/DATA reads must be checked BEFORE
        // epon_mac because the mailbox registers (0x010015C0+) fall
        // inside the epon_mac LLID address range (0x01001400-0x01002000).
        // The bitmap reads go through epon_mac normally (we poke the
        // llid_store in tick), but CMD_STATUS and DATA need the OLT's
        // FIFO side effects (pop-on-read).
        if crate::soc::olt::Olt::claims_mailbox(addr) {
            if let Some(v) = self.olt.read_cmd_status(addr) {
                self.record_history(addr, v, "read", "word", "olt_status");
                self.seq_emit(addr, v, "r", "word", "olt_status");
                return Ok(v);
            }
            if let Some(v) = self.olt.read_data(addr) {
                self.record_history(addr, v, "read", "word", "olt_data");
                self.seq_emit(addr, v, "r", "word", "olt_data");
                return Ok(v);
            }
        }
        if self.uart.claims(addr) { dispatch_rw!(self.uart, "uart"); }
        if self.pbc.claims(addr) { dispatch_rw!(self.pbc, "pbc"); }
        if self.bsc_i2c.claims(addr) { dispatch_rw!(self.bsc_i2c, "bsc_i2c"); }
        if self.mpcp_bus.claims(addr) {
            let v = if addr == self.mpcp_bus.cmd_addr {
                self.mpcp_bus.read_cmd()
            } else if addr == self.mpcp_bus.data_addr {
                self.mpcp_bus.read_data()
            } else {
                self.mpcp_bus.read_stat()
            };
            self.record_history(addr, v, "read", "word", "mpcp_bus");
            self.seq_emit(addr, v, "r", "word", "mpcp_bus");
            return Ok(v);
        }
        if self.serdes.claims(addr) { dispatch_rw!(self.serdes, "serdes"); }
        if self.epon_mac.claims(addr) { dispatch_rw!(self.epon_mac, "epon_mac"); }
        if self.macsec.claims(addr) { dispatch_rw!(self.macsec, "macsec"); }
        if self.dma.claims(addr) { dispatch_rw!(self.dma, "dma"); }
        if self.timer.claims(addr) { dispatch_rw!(self.timer, "timer"); }
        if self.efuse_udr.claims(addr) { dispatch_rw!(self.efuse_udr, "efuse_udr"); }
        if self.fatal_filter.claims(addr) { dispatch_rw!(self.fatal_filter, "fatal_filter"); }
        if self.mpcp.claims(addr) { dispatch_rw!(self.mpcp, "mpcp"); }
        if self.nco.claims(addr) { dispatch_rw!(self.nco, "nco"); }
        if self.vlan_lue.claims(addr) {
            let v = self.vlan_lue.read_word(addr)?;
            self.record_history(addr, v, "read", "word", "vlan_lue");
            self.seq_emit(addr, v, "r", "word", "vlan_lue");
            return Ok(v);
        }
        if self.sysreg.claims(addr) { dispatch_rw!(self.sysreg, "sysreg"); }
        if self.trace {
            eprintln!("[MMIO] read  word  0x{:08X} → 0x00000000 (unmapped)", addr);
        }
        self.seq_emit(addr, 0, "r", "word", "unmapped");
        if self.unmapped_exception {
            return Err(Exception::MemoryError { address: addr, is_write: false });
        }
        Ok(0)
    }

    pub fn write_word(&mut self, addr: u32, val: u32) -> Result<(), Exception> {
        let result = self.write_word_inner(addr, val);
        if result.is_ok() {
            self.record_last_access(addr, val, "write");
        }
        self.scenario.on_mmio_write(addr, val);
        self.drain_scenario_deferred();
        result
    }

    fn write_word_inner(&mut self, addr: u32, val: u32) -> Result<(), Exception> {
        let val = if !self.scenario.overrides.is_empty() {
            if let Some(masked) = self.scenario.overrides.filter_write(addr, val) {
                self.record_history(addr, masked, "write", "word", "override-mask");
                self.seq_emit(addr, masked, "w", "word", "override-mask");
                masked
            } else {
                val
            }
        } else {
            val
        };
        macro_rules! dispatch_ww {
            ($periph:expr, $name:expr) => {{
                $periph.write_word(addr, val)?;
                self.record_history(addr, val, "write", "word", $name);
                self.seq_emit(addr, val, "w", "word", $name);
                return Ok(());
            }};
        }
        // OLT CMD write intercept — must be before epon_mac.
        if crate::soc::olt::Olt::claims_mailbox(addr) && self.olt.write_cmd(addr, val) {
            // Immediately sync bitmap into epon_mac so the firmware
            // doesn't see stale 0xFFFFFFFF on the next bitmap check
            // within the same tick window.
            if self.olt.config.enabled {
                let bmp = if !self.olt.mailbox_pending.is_empty() {
                    0xFFFF_FFFFu32
                } else {
                    0
                };
                for wi in 0..6u32 {
                    let a = 0x0100_1438 + wi * 0x200;
                    if a < 0x0100_2000 {
                        self.epon_mac.poke_llid_store(a, bmp);
                    }
                }
            }
            self.record_history(addr, val, "write", "word", "olt_cmd");
            self.seq_emit(addr, val, "w", "word", "olt_cmd");
            return Ok(());
        }
        if self.uart.claims(addr) { dispatch_ww!(self.uart, "uart"); }
        if self.pbc.claims(addr) {
            self.pbc.write_word(addr, val)?;
            if let Some((tx, rx_len)) = self.pbc.take_pending_spi_serdes() {
                let rx = self.serdes.spi_command(&tx, rx_len);
                self.pbc.complete_spi_serdes(&rx);
            }
            self.record_history(addr, val, "write", "word", "pbc");
            self.seq_emit(addr, val, "w", "word", "pbc");
            return Ok(());
        }
        if self.bsc_i2c.claims(addr) { dispatch_ww!(self.bsc_i2c, "bsc_i2c"); }
        if self.mpcp_bus.claims(addr) {
            if addr == self.mpcp_bus.cmd_addr {
                self.mpcp_bus.write_cmd(val);
            } else if addr == self.mpcp_bus.data_addr {
                self.mpcp_bus.write_data(val);
            } else {
                self.mpcp_bus.write_stat(val);
            }
            self.record_history(addr, val, "write", "word", "mpcp_bus");
            self.seq_emit(addr, val, "w", "word", "mpcp_bus");
            return Ok(());
        }
        if self.serdes.claims(addr) {
            if addr == 0x0100_01B0 && self.olt.config.enabled {
                let old = self.serdes.peek_word(addr).unwrap_or(0);
                self.serdes.write_word(addr, val)?;
                if (val & 0x800) != 0 && (old & 0x800) == 0 {
                    self.handle_mpcp_burst_trigger();
                }
            } else {
                self.serdes.write_word(addr, val)?;
            }
            self.record_history(addr, val, "write", "word", "serdes");
            self.seq_emit(addr, val, "w", "word", "serdes");
            return Ok(());
        }
        if self.epon_mac.claims(addr) { dispatch_ww!(self.epon_mac, "epon_mac"); }
        if self.macsec.claims(addr) { dispatch_ww!(self.macsec, "macsec"); }
        if self.dma.claims(addr) { dispatch_ww!(self.dma, "dma"); }
        if self.timer.claims(addr) { dispatch_ww!(self.timer, "timer"); }
        if self.efuse_udr.claims(addr) { dispatch_ww!(self.efuse_udr, "efuse_udr"); }
        if self.fatal_filter.claims(addr) { dispatch_ww!(self.fatal_filter, "fatal_filter"); }
        if self.mpcp.claims(addr) { dispatch_ww!(self.mpcp, "mpcp"); }
        if self.nco.claims(addr) { dispatch_ww!(self.nco, "nco"); }
        if self.vlan_lue.claims(addr) {
            self.vlan_lue.write_word(addr, val)?;
            self.record_history(addr, val, "write", "word", "vlan_lue");
            self.seq_emit(addr, val, "w", "word", "vlan_lue");
            return Ok(());
        }
        if self.sysreg.claims(addr) { dispatch_ww!(self.sysreg, "sysreg"); }
        if self.trace {
            eprintln!("[MMIO] write word  0x{:08X} = 0x{:08X} (unmapped)", addr, val);
        }
        self.seq_emit(addr, val, "w", "word", "unmapped");
        if self.unmapped_exception {
            return Err(Exception::MemoryError { address: addr, is_write: true });
        }
        Ok(())
    }

    pub fn read_half(&mut self, addr: u32) -> Result<u16, Exception> {
        if !self.scenario.overrides.is_empty() {
            let aligned = addr & !3;
            if let Some(word) = self.scenario.overrides.try_read(aligned) {
                let shift = (2 - (addr & 2)) * 8;
                let v = ((word >> shift) & 0xFFFF) as u16;
                self.record_history(addr, v as u32, "read", "half", "override");
                self.seq_emit(addr, v as u32, "r", "half", "override");
                return Ok(v);
            }
        }
        macro_rules! dispatch_rh {
            ($periph:expr, $name:expr) => {{
                let v = $periph.read_half(addr)?;
                self.seq_emit(addr, v as u32, "r", "half", $name);
                return Ok(v);
            }};
        }
        if self.uart.claims(addr) { dispatch_rh!(self.uart, "uart"); }
        if self.pbc.claims(addr) { dispatch_rh!(self.pbc, "pbc"); }
        if self.bsc_i2c.claims(addr) { dispatch_rh!(self.bsc_i2c, "bsc_i2c"); }
        if self.serdes.claims(addr) { dispatch_rh!(self.serdes, "serdes"); }
        if self.epon_mac.claims(addr) { dispatch_rh!(self.epon_mac, "epon_mac"); }
        if self.macsec.claims(addr) { dispatch_rh!(self.macsec, "macsec"); }
        if self.dma.claims(addr) { dispatch_rh!(self.dma, "dma"); }
        if self.timer.claims(addr) { dispatch_rh!(self.timer, "timer"); }
        if self.efuse_udr.claims(addr) { dispatch_rh!(self.efuse_udr, "efuse_udr"); }
        if self.fatal_filter.claims(addr) { dispatch_rh!(self.fatal_filter, "fatal_filter"); }
        if self.mpcp.claims(addr) { dispatch_rh!(self.mpcp, "mpcp"); }
        if self.nco.claims(addr) { dispatch_rh!(self.nco, "nco"); }
        if self.vlan_lue.claims(addr) {
            let v = self.vlan_lue.read_half(addr)?;
            self.seq_emit(addr, v as u32, "r", "half", "vlan_lue");
            return Ok(v);
        }
        if self.sysreg.claims(addr) { dispatch_rh!(self.sysreg, "sysreg"); }
        self.seq_emit(addr, 0, "r", "half", "unmapped");
        if self.unmapped_exception {
            return Err(Exception::MemoryError { address: addr, is_write: false });
        }
        Ok(0)
    }

    pub fn write_half(&mut self, addr: u32, val: u16) -> Result<(), Exception> {
        macro_rules! dispatch_wh {
            ($periph:expr, $name:expr) => {{
                $periph.write_half(addr, val)?;
                self.seq_emit(addr, val as u32, "w", "half", $name);
                return Ok(());
            }};
        }
        if self.uart.claims(addr) { dispatch_wh!(self.uart, "uart"); }
        if self.pbc.claims(addr) { dispatch_wh!(self.pbc, "pbc"); }
        if self.bsc_i2c.claims(addr) { dispatch_wh!(self.bsc_i2c, "bsc_i2c"); }
        if self.serdes.claims(addr) { dispatch_wh!(self.serdes, "serdes"); }
        if self.epon_mac.claims(addr) { dispatch_wh!(self.epon_mac, "epon_mac"); }
        if self.macsec.claims(addr) { dispatch_wh!(self.macsec, "macsec"); }
        if self.dma.claims(addr) { dispatch_wh!(self.dma, "dma"); }
        if self.timer.claims(addr) { dispatch_wh!(self.timer, "timer"); }
        if self.efuse_udr.claims(addr) { dispatch_wh!(self.efuse_udr, "efuse_udr"); }
        if self.fatal_filter.claims(addr) { dispatch_wh!(self.fatal_filter, "fatal_filter"); }
        if self.mpcp.claims(addr) { dispatch_wh!(self.mpcp, "mpcp"); }
        if self.nco.claims(addr) { dispatch_wh!(self.nco, "nco"); }
        if self.vlan_lue.claims(addr) {
            self.vlan_lue.write_half(addr, val)?;
            self.seq_emit(addr, val as u32, "w", "half", "vlan_lue");
            return Ok(());
        }
        if self.sysreg.claims(addr) { dispatch_wh!(self.sysreg, "sysreg"); }
        self.seq_emit(addr, val as u32, "w", "half", "unmapped");
        if self.unmapped_exception {
            return Err(Exception::MemoryError { address: addr, is_write: true });
        }
        Ok(())
    }

    pub fn read_byte(&mut self, addr: u32) -> Result<u8, Exception> {
        if !self.scenario.overrides.is_empty() {
            let aligned = addr & !3;
            if let Some(word) = self.scenario.overrides.try_read(aligned) {
                let shift = (3 - (addr & 3)) * 8;
                let v = ((word >> shift) & 0xFF) as u8;
                self.record_history(addr, v as u32, "read", "byte", "override");
                self.seq_emit(addr, v as u32, "r", "byte", "override");
                return Ok(v);
            }
        }
        macro_rules! dispatch_rb {
            ($periph:expr, $name:expr) => {{
                let v = $periph.read_byte(addr)?;
                self.seq_emit(addr, v as u32, "r", "byte", $name);
                return Ok(v);
            }};
        }
        if self.uart.claims(addr) { dispatch_rb!(self.uart, "uart"); }
        if self.pbc.claims(addr) { dispatch_rb!(self.pbc, "pbc"); }
        if self.bsc_i2c.claims(addr) { dispatch_rb!(self.bsc_i2c, "bsc_i2c"); }
        if self.serdes.claims(addr) { dispatch_rb!(self.serdes, "serdes"); }
        if self.epon_mac.claims(addr) { dispatch_rb!(self.epon_mac, "epon_mac"); }
        if self.macsec.claims(addr) { dispatch_rb!(self.macsec, "macsec"); }
        if self.dma.claims(addr) { dispatch_rb!(self.dma, "dma"); }
        if self.timer.claims(addr) { dispatch_rb!(self.timer, "timer"); }
        if self.efuse_udr.claims(addr) { dispatch_rb!(self.efuse_udr, "efuse_udr"); }
        if self.fatal_filter.claims(addr) { dispatch_rb!(self.fatal_filter, "fatal_filter"); }
        if self.mpcp.claims(addr) { dispatch_rb!(self.mpcp, "mpcp"); }
        if self.nco.claims(addr) { dispatch_rb!(self.nco, "nco"); }
        if self.vlan_lue.claims(addr) {
            let v = self.vlan_lue.read_byte(addr)?;
            self.seq_emit(addr, v as u32, "r", "byte", "vlan_lue");
            return Ok(v);
        }
        if self.sysreg.claims(addr) { dispatch_rb!(self.sysreg, "sysreg"); }
        self.seq_emit(addr, 0, "r", "byte", "unmapped");
        if self.unmapped_exception {
            return Err(Exception::MemoryError { address: addr, is_write: false });
        }
        Ok(0)
    }

    pub fn write_byte(&mut self, addr: u32, val: u8) -> Result<(), Exception> {
        macro_rules! dispatch_wb {
            ($periph:expr, $name:expr) => {{
                $periph.write_byte(addr, val)?;
                self.seq_emit(addr, val as u32, "w", "byte", $name);
                return Ok(());
            }};
        }
        if self.uart.claims(addr) { dispatch_wb!(self.uart, "uart"); }
        if self.pbc.claims(addr) { dispatch_wb!(self.pbc, "pbc"); }
        if self.bsc_i2c.claims(addr) { dispatch_wb!(self.bsc_i2c, "bsc_i2c"); }
        if self.serdes.claims(addr) { dispatch_wb!(self.serdes, "serdes"); }
        if self.epon_mac.claims(addr) { dispatch_wb!(self.epon_mac, "epon_mac"); }
        if self.macsec.claims(addr) { dispatch_wb!(self.macsec, "macsec"); }
        if self.dma.claims(addr) { dispatch_wb!(self.dma, "dma"); }
        if self.timer.claims(addr) { dispatch_wb!(self.timer, "timer"); }
        if self.efuse_udr.claims(addr) { dispatch_wb!(self.efuse_udr, "efuse_udr"); }
        if self.fatal_filter.claims(addr) { dispatch_wb!(self.fatal_filter, "fatal_filter"); }
        if self.mpcp.claims(addr) { dispatch_wb!(self.mpcp, "mpcp"); }
        if self.nco.claims(addr) { dispatch_wb!(self.nco, "nco"); }
        if self.vlan_lue.claims(addr) {
            self.vlan_lue.write_byte(addr, val)?;
            self.seq_emit(addr, val as u32, "w", "byte", "vlan_lue");
            return Ok(());
        }
        if self.sysreg.claims(addr) { dispatch_wb!(self.sysreg, "sysreg"); }
        self.seq_emit(addr, val as u32, "w", "byte", "unmapped");
        if self.unmapped_exception {
            return Err(Exception::MemoryError { address: addr, is_write: true });
        }
        Ok(())
    }
}
