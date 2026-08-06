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
use crate::soc::lue::Lue;
use crate::soc::dma::DmaChannelController;
use crate::soc::efuse_udr::EfuseUdr;
use crate::soc::epon_mac::EponMac;
use crate::soc::fatal_filter::FatalFilter;
use crate::soc::macsec::Macsec;
use crate::soc::mpcp::Mpcp;
use crate::soc::mpcp_tssync::MpcpTsSync;
use crate::soc::nco::Nco;
use crate::soc::olt::{self, Olt};
use crate::soc::pbc::Pbc;
use crate::soc::peripheral::{
    DatapathOp, Peripheral, PeripheralEvent, PeripheralId, PeripheralSnapshot,
};
use crate::soc::scenario::ScenarioEngine;
use crate::soc::serdes::SerDes;
use crate::soc::sysreg_shim::SysregShim;
use crate::soc::timer::EponTimer;
use crate::soc::uart::Uart;
use crate::soc::mac_filter::MacFilter;

/// CPU instructions between bank tick invocations. Higher = less
/// contention but coarser peripheral advancement. The EPON timer
/// uses per-read advancement independently of this prescaler.
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

/// Source address of a synthesized uplink frame, used until the firmware's
/// own transmission supplies the real one.
const SYNTHETIC_ONU_MAC: olt::types::MacAddr =
    olt::types::MacAddr::new([0x02, 0x00, 0x00, 0x01, 0x02, 0x03]);
/// Discovery windows the synthesized frame claims: the 10/1 Gbit/s row.
const SYNTHETIC_DISCOVERY_INFORMATION: u16 = 0x0011;
/// Laser on and off times the synthesized frame declares.
const SYNTHETIC_LASER_TQ: u8 = 32;

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
    /// `true` when the access used a `.di` (cache bypass) instruction.
    /// `false` for regular `st`/`ld` (cached) or non-CPU-initiated accesses.
    pub di: bool,
}

/// AUX register write history entry (from `sr` instructions).
#[derive(Clone, Debug)]
pub struct AuxWriteHistoryEntry {
    pub insn: u64,
    pub pc: u32,
    pub aux_num: u32,
    pub value: u32,
}

pub const DEFAULT_AUX_WRITE_HISTORY_SIZE: usize = 8192;

pub const DEFAULT_MMIO_HISTORY_SIZE: usize = 8192;

/// Aggregated per-address MMIO write summary for `save_mmio_summary` /
/// `diff_mmio_summaries`.
#[derive(Clone, Debug)]
pub struct MmioSummaryEntry {
    pub write_count: u64,
    pub last_value: u32,
    pub first_insn: u64,
    pub last_insn: u64,
}

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
    pub fn emit(&mut self, cycle: u64, pc: u32, addr: u32, value: u32, rw: &str, width: &str, periph: &str, di: bool) {
        let _ = writeln!(
            self.writer,
            r#"{{"cycle":{cycle},"pc":{pc},"addr":{addr},"value":{value},"rw":"{rw}","width":"{width}","periph":"{periph}","di":{di}}}"#,
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
    /// Packet classifier: two latched indirect ports into the rule
    /// tables. Ten addresses inside the SerDes lane window, so it is
    /// tested before the SerDes on every dispatch chain.
    pub lue: Lue,
    /// Whether the classifier decides downstream routing.
    ///
    /// Shut by default. The step from a rule verdict to a mailbox queue
    /// is not established, and an open gate would put that guess on the
    /// only path every downstream frame takes.
    pub use_classifier: bool,
    pub classifier_binding: crate::soc::lue::ClassifierBinding,
    pub serdes: SerDes,
    pub epon_mac: EponMac,
    pub macsec: Macsec,
    pub dma: DmaChannelController,
    pub alarm_events: AlarmEvents,
    pub timer: EponTimer,
    pub efuse_udr: EfuseUdr,
    pub fatal_filter: FatalFilter,
    pub mpcp: Mpcp,
    /// MPCP TS-sync register block (0x01000300/04/14/18/1C, 0x010000B4,
    /// 0x01000D88/D8C). OLT-gated: behaves as a plain seeded store when
    /// the OLT model is disabled. See `soc/mpcp_tssync.rs` (G1+G5).
    pub mpcp_tssync: MpcpTsSync,
    pub nco: Nco,
    pub mac_filter: MacFilter,
    pub olt: Olt,

    /// Monotonic local NCO TX timestamp counter (bank-tick domain), used
    /// to drive 0x01000D8C when the OLT is enabled (G5). Advances in the
    /// same domain as the EPON free-running counter.
    nco_tx_ts_counter: u32,

    pending_cache_inv: Vec<DatapathOp>,

    /// Scenario engine — MMIO overrides, scheduled events, and MMIO
    /// watchpoints.  Overrides are checked BEFORE the peripheral
    /// dispatch chain.  All stimulus is HW-level, never firmware.
    pub scenario: ScenarioEngine,

    /// Transitional backing store for the SYSREG range
    /// (`0x01000000..0x01003800`). Hosts every register that has not yet
    /// been carved into its own peripheral file — CHIP_ID, LLID IRQ
    /// forced 0, SerDes forced bits, I²C UDR bit-bang, DMA queue drain,
    /// alarm counters, etc. This struct shrinks as subsystems gain
    /// dedicated peripheral modules.
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
    /// Set by `Cpu::step()` before execute: `true` when the current
    /// instruction uses `.di` (cache bypass). Used by `record_history`.
    pub current_di: bool,

    /// Ring buffer of recent AUX register writes (from `sr` instructions).
    pub aux_write_history: VecDeque<AuxWriteHistoryEntry>,
    pub aux_write_history_max: usize,

    /// Optional aggregated MMIO trace for `--dump-mmio-trace`.
    pub mmio_trace: Option<HashMap<u32, MmioTraceEntry>>,

    /// Verbose trace of every MMIO access (unbounded — only enable via
    /// `--trace-mmio`). Mirrors the old `MmioController::trace` flag.
    pub trace: bool,

    /// Aggregated IRQ pending mask from peripherals. `Cpu::step()` ORs
    /// this into `aux_irq_pending` after each bank tick.
    pub irq_pending: u32,

    /// DMA mailbox registers.
    /// master[0/1] (0x10/0x18) = W1C status (ISR clears by writing 1).
    /// channel[0/1] (0x14/0x1C) = R/W mask (firmware writes 0xFFFFFFFF
    ///   to mask all channels). ISR checks `master & ~mask`.
    dma_master_status: [u32; 2],
    dma_channel_mask: [u32; 2],

    /// Boot mode — peripherals snapshot this during construction.
    pub boot_mode: BootMode,

    /// When `true`, MMIO accesses that no peripheral claims return
    /// [`Exception::MemoryError`] instead of silently reading as zero.
    /// Off by default; opt in via `--unmapped-exception` on the main
    /// binary to surface unmodelled firmware probes.
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

    /// Named MMIO write summaries for `diff_mmio_summaries`.
    pub mmio_summaries: HashMap<String, HashMap<u32, MmioSummaryEntry>>,

    /// Countdown (in bank ticks) before clearing the firmware's Phase 1
    /// guard struct at SRAM 0x7E3CA. On real BCM55030, the DMA engine
    /// clears this after frame consumption. Delayed by a few ticks so the
    /// firmware's store instruction has executed before we invalidate the
    /// D-cache line.
    guard_clear_countdown: u32,
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
                // G4: SerDes CDR cold-cal-done bits (lane2 0xBB / lane3
                // 0xDB, reg-file index 0x100+reg) are NO LONGER force-
                // seeded here. Cal convergence is an analog physics effect
                // the emulator must NOT fake (DO-NOT-FAKE). It is now a
                // scenario input — default not-converged (bit7=0). A
                // scenario opts in via `set_serdes_cal_converged`, and the
                // bank tick then seeds bit7 (see `apply_serdes_cal_seed`).
                bus
            },
            serdes: SerDes::new(),
            epon_mac: EponMac::new(),
            macsec: Macsec::new(),
            dma: DmaChannelController::new(),
            alarm_events: AlarmEvents::new(),
            timer: EponTimer::new(),
            lue: Lue::new(),
            use_classifier: false,
            classifier_binding: Default::default(),
            efuse_udr: EfuseUdr::new(),
            fatal_filter: FatalFilter::new(),
            mpcp: Mpcp::new(),
            mpcp_tssync: MpcpTsSync::new(),
            nco: Nco::new(),
            mac_filter: MacFilter::new(),
            olt: Olt::new(),
            nco_tx_ts_counter: 0,
            pending_cache_inv: Vec::new(),
            scenario: ScenarioEngine::new(),
            sysreg: SysregShim::new(),
            uart_rx_sender: tx,
            uart_rx_receiver: Mutex::new(rx),
            current_pc: 0,
            current_blink: 0,
            current_insn: 0,
            current_di: false,
            aux_write_history: VecDeque::with_capacity(DEFAULT_AUX_WRITE_HISTORY_SIZE),
            aux_write_history_max: DEFAULT_AUX_WRITE_HISTORY_SIZE,
            mmio_trace: None,
            trace: false,
            irq_pending: 0,
            dma_master_status: [0; 2],
            dma_channel_mask: [0; 2],
            boot_mode,
            unmapped_exception: false,
            last_access: HashMap::new(),
            mmio_history: VecDeque::with_capacity(DEFAULT_MMIO_HISTORY_SIZE),
            mmio_history_max: DEFAULT_MMIO_HISTORY_SIZE,
            seq_trace: None,
            mmio_summaries: HashMap::new(),
            guard_clear_countdown: 0,
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
        self.mpcp_tssync.tick(cpu_instructions);
        self.nco.tick(cpu_instructions);
        self.mac_filter.tick(cpu_instructions);
        self.olt.tick(cpu_instructions);
        // Drain OLT RX frames into per-slot mailbox FIFOs. Each frame
        // is routed by EtherType: MPCP→slot 0x10, OAM→slot 0x0F, etc.
        let mbox_before = self.olt.total_pending_count();
        // The classifier decides only when its gate is open. Shut, the
        // routing is byte-for-byte what it was before it existed.
        let classifier = self
            .use_classifier
            .then_some((&self.lue, self.classifier_binding));
        self.olt.load_frames_into_mailbox(classifier);
        // The MAC holds the counters software latches; the OLT is where a
        // frame is known to have landed. Carrying the number across is the
        // datapath's job, and it is the bank that owns the datapath.
        for (slot, frames) in self.olt.drain_arrivals() {
            self.epon_mac.record_queue_arrivals(slot, frames);
        }
        if self.olt.total_pending_count() > mbox_before {
            self.dma_master_status[0] |= 1 << 27;
            self.dma_channel_mask[0] |= 1;
            // Invalidate D-cache line for DMA master status so the
            // firmware's ISR reads the real value on the next IRQ 6.
            self.pending_cache_inv.push(
                DatapathOp::CacheInvalidate { addr: 0x0100_0010 },
            );
        }
        // Sync OLT bitmaps into the epon_mac LLID backing store.
        // Only set when mailbox_pending has unread frames. Once the
        // firmware issues a CMD write and the frame moves to the FIFO,
        // the bitmap clears so the firmware doesn't try to read a
        // second non-existent frame.
        // When the OLT assigns an LLID (REGISTER frame), update the
        // EPON MAC LLID match table so the firmware can find the slot.
        if let Some(llid) = self.olt.pending_llid_update.take() {
            // Update BOTH 1G and 10G LLID match tables (slot 0).
            for &base in &[0x0100_043Cu32, 0x0100_0D00u32] {
                let old_val = self.epon_mac.read_word(base).unwrap_or(0x0001_7FFE);
                let new_val = (old_val & 0xFFFF_0000) | (llid as u32);
                self.epon_mac.poke_table_store(base, new_val);
                self.pending_cache_inv.push(
                    DatapathOp::CacheInvalidate { addr: base },
                );
            }
        }
        if self.olt.link_change_pending {
            self.olt.link_change_pending = false;
            self.epon_mac.set_1g_link_change_bit();
            // G2: bit6 (0x01000E04) is NO LONGER set on the link-change
            // edge. It is a 10G PCS block-lock LEVEL driven per-tick below
            // only once the OLT has broadcast a downstream GATE (a real
            // stream to lock to) — not merely because the PHY link came up.
            self.epon_mac.set_discovery_status_bit();
            for &addr in &[0x0100_0410u32, 0x0100_0E04, 0x0100_1040] {
                self.pending_cache_inv.push(
                    DatapathOp::CacheInvalidate { addr },
                );
            }
        }
        if self.olt.link_up {
            self.epon_mac.set_discovery_status_bit();
        }
        // ── G2: bit6 (0x01000E04) 10G PCS block-lock as a sticky LEVEL ─
        // While the OLT model broadcasts a valid downstream (enabled +
        // link up) and the lane-3 RX path is up, re-assert bit6 every
        // tick so it behaves as a continuous block-lock level (the
        // firmware W1C-clears it and checks re-latch — mpcp_sm.rs:495).
        // When the downstream is gone, the level drops. This is a NO-OP
        // when the OLT is disabled (bit6 stays 0, as before). DO-NOT-FAKE:
        // the value tracks the modelled downstream-present condition.
        // "lane-3 RX up + valid downstream present" is modelled by the
        // OLT having broadcast >=1 downstream GATE (a real stream the
        // PCS can block-lock to). We do NOT model analog cal physics
        // here (that is a scenario input — G4); bit6 reflects only the
        // modelled downstream-present condition.
        if self.olt.link_up && self.olt.has_broadcast_gate() {
            self.epon_mac.set_phy_link_status_bit();
            self.pending_cache_inv.push(
                DatapathOp::CacheInvalidate { addr: 0x0100_0E04 },
            );
        } else if !self.olt.link_up {
            // Downstream stream gone -> PCS block-lock drops.
            self.epon_mac.clear_phy_link_status_bit();
            self.pending_cache_inv.push(
                DatapathOp::CacheInvalidate { addr: 0x0100_0E04 },
            );
        }
        // ── G1 + G5: MPCP TS-sync register drive (OLT-gated) ──────────
        // When the OLT model is broadcasting a downstream (enabled +
        // link up), drive the HW-status TS-sync registers from the OLT
        // model's advancing timestamp and a local monotonic NCO counter.
        // This whole block is a NO-OP when the OLT is disabled, so the
        // boot path is byte-identical in that case.
        if self.olt.link_up {
            // 0x010000B4 = live OLT MPCP timestamp (HW captured). On
            // silicon the firmware treats 0xFFFFFFFF as "block not
            // producing"; a present downstream gives a real value.
            let olt_ts = self.olt.mpcp_timestamp;
            self.mpcp_tssync.drive_captured_ts(olt_ts);
            // 0x01000D8C = local NCO TX timestamp, monotonic in the
            // bank-tick domain (G5).
            self.nco_tx_ts_counter = self.nco_tx_ts_counter.wrapping_add(1);
            self.mpcp_tssync.drive_nco_tx_ts(self.nco_tx_ts_counter);
            // 0x01000320 = HW-captured OLT timestamp (block 52, owned by
            // `mpcp`). Mirror the OLT model's advancing timestamp so the
            // firmware's registration-independent RX-decode proof
            // (mpcp_sm.rs:490) advances. Poke the mpcp store directly
            // (HW-faithful: this is the EPON MAC latching the OLT TS, not
            // a firmware write).
            self.mpcp.poke_tx_rate(0x0100_0320, olt_ts);
            // 0x01000304 bit0 = HW OLT-lock. Set ONLY once the OLT model
            // has broadcast >=1 downstream GATE — models the HW timestamp
            // lock off the recovered downstream. DO-NOT-FAKE: never set
            // from a firmware write to 0x01000300.
            self.mpcp_tssync.set_hw_lock(self.olt.has_broadcast_gate());
            for &addr in &[0x0100_00B4u32, 0x0100_0304, 0x0100_0320, 0x0100_0D8C] {
                self.pending_cache_inv.push(
                    DatapathOp::CacheInvalidate { addr },
                );
            }
        }
        // LLID OAM state initialization — models the EPON MAC's
        // internal state setup after registration completes. On real
        // BCM55030, event 0xC1 (pushed by epon_llid_teardown) sets
        // the OAM state byte. The emulator doesn't model the full
        // event dispatch chain, so we set it directly with a delay
        // to let the firmware's teardown/setup cycle finish first.
        if self.olt.registration_complete {
            self.olt.registration_complete = false;
            self.olt.llid_state_init_countdown = 2000;
        }
        if self.olt.llid_state_init_countdown > 0 {
            self.olt.llid_state_init_countdown -= 1;
            if self.olt.llid_state_init_countdown == 0 {
                self.pending_cache_inv.push(
                    DatapathOp::SramWrite {
                        sram_addr: 0x0000_461C,
                        data: vec![0x01],
                    },
                );
            }
        }
        for wi in 0..6u32 {
            let addr = 0x0100_1438 + wi * 0x200;
            if addr < 0x0100_2000 {
                let bmp = self.olt.mailbox_bitmap[wi as usize];
                self.epon_mac.poke_llid_store(addr, bmp);
                self.pending_cache_inv.push(
                    DatapathOp::CacheInvalidate { addr },
                );
            }
        }
        // Phase 1 guard struct clear — models DMA completion.
        // On real BCM55030, the DMA engine clears the frame descriptor
        // at SRAM 0x7E3CA after the firmware consumes a mailbox frame.
        // The firmware's `epon_rx_packet_dispatch_handler` stores the
        // frame length at 0x7E3CA-0x7E3CB; the guard byte at 0x7E3CB
        // gates Phase 1: non-zero → permanent flush mode.
        //
        // The firmware has TWO frame consumption paths:
        // - Process path: reads CMD/DATA → OLT intercepts → frame_consumed
        // - Flush path: writes DRAIN registers → not intercepted by OLT
        //
        // We clear the guard whenever the mailbox is empty (no pending
        // frames, FIFO drained). This covers both paths and models the
        // DMA engine's "no frames pending" signal. The 3-tick delay
        // ensures the firmware's store to 0x7E3CA has executed before
        // the D-cache invalidation discards it.
        let mailbox_idle = !self.olt.has_any_pending()
            && self.olt.mailbox_fifo.is_empty();
        if self.olt.frame_consumed || mailbox_idle {
            self.olt.frame_consumed = false;
            if self.guard_clear_countdown == 0 {
                self.guard_clear_countdown = 3;
            }
        }
        if self.guard_clear_countdown > 0 {
            self.guard_clear_countdown -= 1;
            if self.guard_clear_countdown == 0 {
                self.pending_cache_inv.push(
                    DatapathOp::SramWrite {
                        sram_addr: 0x0007_E3CA,
                        data: vec![0x00, 0x00],
                    },
                );
            }
        }

        self.sysreg.tick(cpu_instructions);
        self.scenario.tick(cpu_instructions);

        // The RX calibration is no longer seeded from here. It converges as
        // a consequence of the sequence software programs — arming the
        // control register starts it, and it completes after the measured
        // number of status reads (`LaneBus::arm_transaction`). A scenario
        // that opts into a converged analog state now means "skip the
        // wait", not "fabricate the bit".
        if self.scenario.serdes_cal_converged {
            self.mpcp_bus.set_cal_immediate(true);
        }

        // Aggregate IRQ pending bits from all peripherals.
        self.irq_pending = self.uart.irq_pending();
        // DMA mailbox IRQ 6: NOT generated. The firmware masks all
        // channels (0xFFFFFFFF in 0x14/0x1C), so the ISR never
        // dispatches handlers. Frame RX goes through the main loop
        // polling path (epon_rx_poll_and_dispatch_queues). Generating
        // IRQ 6 causes an ISR storm because the ISR reads 0x10 via
        // D-cache (stale value) and the W1C write never reaches MMIO.
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
        // G4: cal-done bit7 (0x1BB/0x1DB) NOT force-seeded on reset —
        // default not-converged. Scenario opt-in applies it in tick().
        self.serdes.reset_cold();
        self.epon_mac.reset_cold();
        self.macsec.reset_cold();
        self.dma.reset_cold();
        self.alarm_events.reset_cold();
        self.timer.reset_cold();
        self.lue.reset_cold();
        self.efuse_udr.reset_cold();
        self.fatal_filter.reset_cold();
        self.mpcp.reset_cold();
        self.mpcp_tssync.reset_cold();
        self.nco.reset_cold();
        self.mac_filter.reset_cold();
        self.olt.reset_cold();
        self.sysreg.reset_cold();
        self.nco_tx_ts_counter = 0;
        self.irq_pending = 0;
        self.dma_master_status = [0; 2];
        self.dma_channel_mask = [0; 2];
        self.current_pc = 0;
        self.current_blink = 0;
        self.current_insn = 0;
        self.current_di = false;
    }

    /// Warm reset — loads the post-boot snapshot on top of cold reset.
    pub fn reset_warm(&mut self) {
        self.uart.reset_warm();
        self.pbc.reset_warm();
        self.bsc_i2c.reset_warm();
        self.mpcp_bus.reset();
        self.mpcp_bus.apply_init(super::mmio_init::SYSREG_INIT_VALUES);
        // G4: cal-done bit7 (0x1BB/0x1DB) NOT force-seeded on reset —
        // default not-converged. Scenario opt-in applies it in tick().
        self.serdes.reset_warm();
        self.epon_mac.reset_warm();
        self.macsec.reset_warm();
        self.dma.reset_warm();
        self.alarm_events.reset_warm();
        self.timer.reset_warm();
        self.lue.reset_warm();
        self.efuse_udr.reset_warm();
        self.fatal_filter.reset_warm();
        self.mpcp.reset_warm();
        self.mpcp_tssync.reset_warm();
        self.nco.reset_warm();
        self.mac_filter.reset_warm();
        self.olt.reset_warm();
        self.sysreg.reset_warm();
        self.nco_tx_ts_counter = 0;
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
        // The firmware pushes its own frames through the transmit port. Once
        // one has been captured there, this synthetic frame is not a stand-in
        // any more — it is a second, competing source for the same state
        // machine, and it reports a registration the firmware never earned.
        if self.olt.real_tx_seen() {
            return;
        }
        let onu_mac = self
            .olt
            .config
            .onu_mac_override
            .unwrap_or(SYNTHETIC_ONU_MAC);
        let frame = olt::mpcp::register_req(
            olt::mpcp::Header {
                dst: olt::types::MacAddr::MPCP_MULTICAST,
                src: onu_mac,
                opcode: olt::mpcp::Opcode::RegisterReq,
                timestamp: self.olt.mpcp_timestamp,
            },
            olt::mpcp::RegisterReqBody {
                flag: olt::mpcp::RegisterReqFlag::Register,
                pending_grants: 0,
                discovery_information: SYNTHETIC_DISCOVERY_INFORMATION,
                laser_on: SYNTHETIC_LASER_TQ,
                laser_off: SYNTHETIC_LASER_TQ,
            },
        );
        eprintln!("[BCM55030] MPCP burst trigger — synthetic REGISTER_REQ from {onu_mac}");
        self.olt.handle_tx_frame(&frame);
    }


    /// Update the CPU context fields in one shot. Called by `Cpu::step()`
    /// before every MMIO access so trace entries can show the touching
    /// PC / blink / insn count.
    pub fn update_cpu_context(&mut self, pc: u32, blink: u32, insn: u64) {
        self.current_pc = pc;
        self.current_blink = blink;
        self.current_insn = insn;
        self.current_di = false;
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
            di: self.current_di,
        });
    }

    /// Record an AUX register write (from `sr` instruction).
    pub fn record_aux_write(&mut self, aux_num: u32, value: u32) {
        if self.aux_write_history_max == 0 { return; }
        if self.aux_write_history.len() >= self.aux_write_history_max {
            self.aux_write_history.pop_front();
        }
        self.aux_write_history.push_back(AuxWriteHistoryEntry {
            insn: self.current_insn,
            pc: self.current_pc,
            aux_num,
            value,
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
                st.emit(self.current_insn, self.current_pc, addr, value, rw, width, periph, self.current_di);
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
        if self.lue.claims(addr) { return Some("lue"); }
        if self.serdes.claims(addr) { return Some("serdes"); }
        // mpcp_tssync claims 8 exact addresses, two of which (0x0D88/
        // 0x0D8C) fall inside epon_mac's table window — check it FIRST.
        if self.mpcp_tssync.claims(addr) { return Some("mpcp_tssync"); }
        if self.epon_mac.claims(addr) { return Some("epon_mac"); }
        if self.macsec.claims(addr) { return Some("macsec"); }
        if self.dma.claims(addr) { return Some("dma"); }
        if self.timer.claims(addr) { return Some("timer"); }
        if self.efuse_udr.claims(addr) { return Some("efuse_udr"); }
        if self.fatal_filter.claims(addr) { return Some("fatal_filter"); }
        if self.mpcp.claims(addr) { return Some("mpcp"); }
        if self.nco.claims(addr) { return Some("nco"); }
        if self.mac_filter.claims(addr) { return Some("mac_filter"); }
        if self.sysreg.claims(addr) { return Some("sysreg"); }
        None
    }

    pub fn capture_for_snapshot(&self) -> crate::emu::named_snapshot::PeripheralBankSaveState {
        crate::emu::named_snapshot::PeripheralBankSaveState {
            uart: self.uart.clone(),
            pbc: self.pbc.clone(),
            bsc_i2c: self.bsc_i2c.clone(),
            mpcp_bus: self.mpcp_bus.clone(),
            lue: self.lue.clone(),
            use_classifier: self.use_classifier,
            classifier_binding: self.classifier_binding,
            serdes: self.serdes.clone(),
            epon_mac: self.epon_mac.clone(),
            macsec: self.macsec.clone(),
            dma: self.dma.clone(),
            alarm_events: self.alarm_events.clone(),
            timer: self.timer.clone(),
            efuse_udr: self.efuse_udr.clone(),
            fatal_filter: self.fatal_filter.clone(),
            mpcp: self.mpcp.clone(),
            mpcp_tssync: self.mpcp_tssync.clone(),
            nco: self.nco.clone(),
            mac_filter: self.mac_filter.clone(),
            olt: self.olt.clone(),
            scenario: self.scenario.clone(),
            sysreg: self.sysreg.clone(),
            pending_cache_inv: self.pending_cache_inv.clone(),
            dma_master_status: self.dma_master_status,
            dma_channel_mask: self.dma_channel_mask,
            guard_clear_countdown: self.guard_clear_countdown,
            nco_tx_ts_counter: self.nco_tx_ts_counter,
        }
    }

    pub fn restore_from_snapshot(&mut self, state: crate::emu::named_snapshot::PeripheralBankSaveState) {
        self.uart = state.uart;
        self.pbc = state.pbc;
        self.bsc_i2c = state.bsc_i2c;
        self.mpcp_bus = state.mpcp_bus;
        self.lue = state.lue;
        self.use_classifier = state.use_classifier;
        self.classifier_binding = state.classifier_binding;
        self.serdes = state.serdes;
        self.epon_mac = state.epon_mac;
        self.macsec = state.macsec;
        self.dma = state.dma;
        self.alarm_events = state.alarm_events;
        self.timer = state.timer;
        self.efuse_udr = state.efuse_udr;
        self.fatal_filter = state.fatal_filter;
        self.mpcp = state.mpcp;
        self.mpcp_tssync = state.mpcp_tssync;
        self.nco = state.nco;
        self.mac_filter = state.mac_filter;
        self.olt = state.olt;
        self.scenario = state.scenario;
        self.sysreg = state.sysreg;
        self.pending_cache_inv = state.pending_cache_inv;
        self.dma_master_status = state.dma_master_status;
        self.dma_channel_mask = state.dma_channel_mask;
        self.guard_clear_countdown = state.guard_clear_countdown;
        self.nco_tx_ts_counter = state.nco_tx_ts_counter;
        self.irq_pending = 0;
        self.current_pc = 0;
        self.current_blink = 0;
        self.current_insn = 0;
        self.current_di = false;
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
        if self.lue.claims(addr) {
            return Ok(self.lue.peek_word(addr).unwrap_or(0));
        }
        if self.serdes.claims(addr) {
            return self.serdes.peek_word(addr);
        }
        // mpcp_tssync first — 0x0D88/0x0D8C overlap epon_mac's table.
        if self.mpcp_tssync.claims(addr) {
            return self.mpcp_tssync.peek_word(addr);
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
        if self.mac_filter.claims(addr) {
            return self.mac_filter.peek_word(addr);
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
        if self.lue.claims(addr) { dispatch_rw!(self.lue, "lue"); }
        if self.serdes.claims(addr) {
            let v = self.serdes.read_word(addr)?;
            let tag = SerDes::mdio_peripheral_tag(addr);
            self.record_history(addr, v, "read", "word", tag);
            self.seq_emit(addr, v, "r", "word", tag);
            return Ok(v);
        }
        // mpcp_tssync first — 0x0D88/0x0D8C overlap epon_mac's table.
        if self.mpcp_tssync.claims(addr) { dispatch_rw!(self.mpcp_tssync, "mpcp_tssync"); }
        if self.epon_mac.claims(addr) { dispatch_rw!(self.epon_mac, "epon_mac"); }
        if self.macsec.claims(addr) { dispatch_rw!(self.macsec, "macsec"); }
        if self.dma.claims(addr) { dispatch_rw!(self.dma, "dma"); }
        if self.timer.claims(addr) { dispatch_rw!(self.timer, "timer"); }
        if self.efuse_udr.claims(addr) { dispatch_rw!(self.efuse_udr, "efuse_udr"); }
        if self.fatal_filter.claims(addr) { dispatch_rw!(self.fatal_filter, "fatal_filter"); }
        if self.mpcp.claims(addr) { dispatch_rw!(self.mpcp, "mpcp"); }
        if self.nco.claims(addr) { dispatch_rw!(self.nco, "nco"); }
        if self.mac_filter.claims(addr) {
            let v = self.mac_filter.read_word(addr)?;
            self.record_history(addr, v, "read", "word", "mac_filter");
            self.seq_emit(addr, v, "r", "word", "mac_filter");
            return Ok(v);
        }
        // DMA mailbox status registers — intercept before sysreg.
        match addr {
            0x0100_0010 => {
                let v = self.dma_master_status[0];
                self.record_history(addr, v, "read", "word", "dma_mbox");
                self.seq_emit(addr, v, "r", "word", "dma_mbox");
                return Ok(v);
            }
            0x0100_0014 => {
                let v = self.dma_channel_mask[0];
                self.record_history(addr, v, "read", "word", "dma_mbox");
                self.seq_emit(addr, v, "r", "word", "dma_mbox");
                return Ok(v);
            }
            0x0100_0018 => {
                let v = self.dma_master_status[1];
                self.record_history(addr, v, "read", "word", "dma_mbox");
                self.seq_emit(addr, v, "r", "word", "dma_mbox");
                return Ok(v);
            }
            0x0100_001C => {
                let v = self.dma_channel_mask[1];
                self.record_history(addr, v, "read", "word", "dma_mbox");
                self.seq_emit(addr, v, "r", "word", "dma_mbox");
                return Ok(v);
            }
            _ => {}
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
        // The MAC's report template. Observational: the write still reaches
        // the table store that serves it back.
        self.olt.observe_report_template(addr, val);
        // Uplink capture. Observational only: it consumes nothing and
        // changes no read-back, so the write still reaches whichever
        // peripheral claims it below.
        if crate::soc::olt::Olt::claims_mailbox(addr) {
            self.olt.observe_write(addr, val);
        }
        // OLT CMD write intercept — must be before epon_mac.
        if crate::soc::olt::Olt::claims_mailbox(addr) && self.olt.write_cmd(addr, val) {
            // Immediately sync per-slot bitmap into epon_mac so the
            // firmware's next bitmap read sees the updated value.
            for wi in 0..6u32 {
                let a = 0x0100_1438 + wi * 0x200;
                if a < 0x0100_2000 {
                    self.epon_mac.poke_llid_store(a, self.olt.mailbox_bitmap[wi as usize]);
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
            let tag = self.pbc.last_dma_tag;
            self.record_history(addr, val, "write", "word", tag);
            self.seq_emit(addr, val, "w", "word", tag);
            return Ok(());
        }
        if self.bsc_i2c.claims(addr) { dispatch_ww!(self.bsc_i2c, "bsc_i2c"); }
        if self.mpcp_bus.claims(addr) {
            if addr == self.mpcp_bus.cmd_addr {
                self.mpcp_bus.write_cmd(val);
                self.mpcp_bus.clear_cmd_bits_now();
                self.pending_cache_inv.push(
                    DatapathOp::CacheInvalidate { addr },
                );
            } else if addr == self.mpcp_bus.data_addr {
                self.mpcp_bus.write_data(val);
            } else {
                self.mpcp_bus.write_stat(val);
                // A STAT write is what arms a lane-register transaction:
                // the operand is already staged in the reg file and this
                // word says whether to read it, write it, or only move the
                // pointer. `arm_transaction` carries out the access against
                // the lane registers, which is where the RX calibration
                // engine lives.
                self.mpcp_bus.arm_transaction(val);
            }
            self.record_history(addr, val, "write", "word", "mpcp_bus");
            self.seq_emit(addr, val, "w", "word", "mpcp_bus");
            return Ok(());
        }
        if self.lue.claims(addr) { dispatch_ww!(self.lue, "lue"); }
        if self.serdes.claims(addr) {
            if addr == 0x0100_01B0 {
                let old = self.serdes.peek_word(addr).unwrap_or(0);
                self.serdes.write_word(addr, val)?;
                if (val & 0x800) != 0 && (old & 0x800) == 0 {
                    self.handle_mpcp_burst_trigger();
                }
            } else {
                self.serdes.write_word(addr, val)?;
            }
            let tag = SerDes::mdio_peripheral_tag(addr);
            self.record_history(addr, val, "write", "word", tag);
            self.seq_emit(addr, val, "w", "word", tag);
            return Ok(());
        }
        // mpcp_tssync first — 0x0D88/0x0D8C overlap epon_mac's table.
        if self.mpcp_tssync.claims(addr) { dispatch_ww!(self.mpcp_tssync, "mpcp_tssync"); }
        if self.epon_mac.claims(addr) {
            // When the OLT has a registered LLID, enforce it in the
            // LLID match table. The firmware reinitializes these entries
            // during MPCP processing, overwriting the OLT's assignment.
            // On real BCM55030, the EPON MAC hardware maintains the
            // registered LLID — the firmware's table writes program
            // the match value, but slot 0 always reflects the OLT's
            // assigned LLID once registration completes.
            if self.olt.mpcp_state() != crate::soc::olt::OltMpcpState::Idle
                && (addr == 0x0100_043C || addr == 0x0100_0D00)
                && val != 0
            {
                let llid = self.olt.assigned_llid() as u32;
                let forced = (val & 0xFFFF_0000) | llid;
                self.epon_mac.poke_table_store(addr, forced);
                self.record_history(addr, forced, "write", "word", "epon_mac");
                self.seq_emit(addr, forced, "w", "word", "epon_mac");
                return Ok(());
            }
            dispatch_ww!(self.epon_mac, "epon_mac");
        }
        if self.macsec.claims(addr) { dispatch_ww!(self.macsec, "macsec"); }
        if self.dma.claims(addr) { dispatch_ww!(self.dma, "dma"); }
        if self.timer.claims(addr) { dispatch_ww!(self.timer, "timer"); }
        if self.efuse_udr.claims(addr) {
            // The lane register file has one instance per context, and
            // which one a transaction reaches is decided *here*, not on
            // the lane bus: software re-arms bits [13:12] of this word
            // before every transaction. The bus cannot see the selector,
            // so route it across explicitly — without this, both contexts
            // share one file and each overwrites the other's registers.
            if addr == crate::soc::efuse_udr::REG_I2C_UDR_CLK_RESET {
                self.mpcp_bus.set_page(((val >> 12) & 3) as usize);
            }
            dispatch_ww!(self.efuse_udr, "efuse_udr");
        }
        if self.fatal_filter.claims(addr) { dispatch_ww!(self.fatal_filter, "fatal_filter"); }
        if self.mpcp.claims(addr) { dispatch_ww!(self.mpcp, "mpcp"); }
        if self.nco.claims(addr) { dispatch_ww!(self.nco, "nco"); }
        if self.mac_filter.claims(addr) {
            self.mac_filter.write_word(addr, val)?;
            self.record_history(addr, val, "write", "word", "mac_filter");
            self.seq_emit(addr, val, "w", "word", "mac_filter");
            return Ok(());
        }
        // DMA mailbox registers — W1C semantics.
        match addr {
            0x0100_0010 => {
                self.dma_master_status[0] &= !val;
                self.record_history(addr, val, "write", "word", "dma_mbox");
                self.seq_emit(addr, val, "w", "word", "dma_mbox");
                return Ok(());
            }
            0x0100_0014 => {
                self.dma_channel_mask[0] = val;
                self.record_history(addr, val, "write", "word", "dma_mbox");
                self.seq_emit(addr, val, "w", "word", "dma_mbox");
                return Ok(());
            }
            0x0100_0018 => {
                self.dma_master_status[1] &= !val;
                self.record_history(addr, val, "write", "word", "dma_mbox");
                self.seq_emit(addr, val, "w", "word", "dma_mbox");
                return Ok(());
            }
            0x0100_001C => {
                self.dma_channel_mask[1] = val;
                self.record_history(addr, val, "write", "word", "dma_mbox");
                self.seq_emit(addr, val, "w", "word", "dma_mbox");
                return Ok(());
            }
            _ => {}
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
        if self.lue.claims(addr) { dispatch_rh!(self.lue, "lue"); }
        if self.serdes.claims(addr) { dispatch_rh!(self.serdes, "serdes"); }
        if self.mpcp_tssync.claims(addr) { dispatch_rh!(self.mpcp_tssync, "mpcp_tssync"); }
        if self.epon_mac.claims(addr) { dispatch_rh!(self.epon_mac, "epon_mac"); }
        if self.macsec.claims(addr) { dispatch_rh!(self.macsec, "macsec"); }
        if self.dma.claims(addr) { dispatch_rh!(self.dma, "dma"); }
        if self.timer.claims(addr) { dispatch_rh!(self.timer, "timer"); }
        if self.efuse_udr.claims(addr) { dispatch_rh!(self.efuse_udr, "efuse_udr"); }
        if self.fatal_filter.claims(addr) { dispatch_rh!(self.fatal_filter, "fatal_filter"); }
        if self.mpcp.claims(addr) { dispatch_rh!(self.mpcp, "mpcp"); }
        if self.nco.claims(addr) { dispatch_rh!(self.nco, "nco"); }
        if self.mac_filter.claims(addr) {
            let v = self.mac_filter.read_half(addr)?;
            self.seq_emit(addr, v as u32, "r", "half", "mac_filter");
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
        if self.lue.claims(addr) { dispatch_wh!(self.lue, "lue"); }
        if self.serdes.claims(addr) { dispatch_wh!(self.serdes, "serdes"); }
        if self.mpcp_tssync.claims(addr) { dispatch_wh!(self.mpcp_tssync, "mpcp_tssync"); }
        if self.epon_mac.claims(addr) { dispatch_wh!(self.epon_mac, "epon_mac"); }
        if self.macsec.claims(addr) { dispatch_wh!(self.macsec, "macsec"); }
        if self.dma.claims(addr) { dispatch_wh!(self.dma, "dma"); }
        if self.timer.claims(addr) { dispatch_wh!(self.timer, "timer"); }
        if self.efuse_udr.claims(addr) { dispatch_wh!(self.efuse_udr, "efuse_udr"); }
        if self.fatal_filter.claims(addr) { dispatch_wh!(self.fatal_filter, "fatal_filter"); }
        if self.mpcp.claims(addr) { dispatch_wh!(self.mpcp, "mpcp"); }
        if self.nco.claims(addr) { dispatch_wh!(self.nco, "nco"); }
        if self.mac_filter.claims(addr) {
            self.mac_filter.write_half(addr, val)?;
            self.seq_emit(addr, val as u32, "w", "half", "mac_filter");
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
        if self.lue.claims(addr) { dispatch_rb!(self.lue, "lue"); }
        if self.serdes.claims(addr) { dispatch_rb!(self.serdes, "serdes"); }
        if self.mpcp_tssync.claims(addr) { dispatch_rb!(self.mpcp_tssync, "mpcp_tssync"); }
        if self.epon_mac.claims(addr) { dispatch_rb!(self.epon_mac, "epon_mac"); }
        if self.macsec.claims(addr) { dispatch_rb!(self.macsec, "macsec"); }
        if self.dma.claims(addr) { dispatch_rb!(self.dma, "dma"); }
        if self.timer.claims(addr) { dispatch_rb!(self.timer, "timer"); }
        if self.efuse_udr.claims(addr) { dispatch_rb!(self.efuse_udr, "efuse_udr"); }
        if self.fatal_filter.claims(addr) { dispatch_rb!(self.fatal_filter, "fatal_filter"); }
        if self.mpcp.claims(addr) { dispatch_rb!(self.mpcp, "mpcp"); }
        if self.nco.claims(addr) { dispatch_rb!(self.nco, "nco"); }
        if self.mac_filter.claims(addr) {
            let v = self.mac_filter.read_byte(addr)?;
            self.seq_emit(addr, v as u32, "r", "byte", "mac_filter");
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
        if self.lue.claims(addr) { dispatch_wb!(self.lue, "lue"); }
        if self.serdes.claims(addr) { dispatch_wb!(self.serdes, "serdes"); }
        if self.mpcp_tssync.claims(addr) { dispatch_wb!(self.mpcp_tssync, "mpcp_tssync"); }
        if self.epon_mac.claims(addr) { dispatch_wb!(self.epon_mac, "epon_mac"); }
        if self.macsec.claims(addr) { dispatch_wb!(self.macsec, "macsec"); }
        if self.dma.claims(addr) { dispatch_wb!(self.dma, "dma"); }
        if self.timer.claims(addr) { dispatch_wb!(self.timer, "timer"); }
        if self.efuse_udr.claims(addr) { dispatch_wb!(self.efuse_udr, "efuse_udr"); }
        if self.fatal_filter.claims(addr) { dispatch_wb!(self.fatal_filter, "fatal_filter"); }
        if self.mpcp.claims(addr) { dispatch_wb!(self.mpcp, "mpcp"); }
        if self.nco.claims(addr) { dispatch_wb!(self.nco, "nco"); }
        if self.mac_filter.claims(addr) {
            self.mac_filter.write_byte(addr, val)?;
            self.seq_emit(addr, val as u32, "w", "byte", "mac_filter");
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
