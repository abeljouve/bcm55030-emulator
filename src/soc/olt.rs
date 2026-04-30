//! BCM55030 OLT Emulator — EPON OLT protocol engine.
//!
//! Emulates an EPON Optical Line Terminal from the ONU firmware's
//! perspective. The OLT captures TX frames from the ONU, runs
//! MPCP registration and OAM keepalive state machines, and injects
//! RX response frames back through the firmware's DMA RX path.
//!
//! This is a **hardware-level** peripheral — it communicates with
//! the firmware exclusively through MMIO registers, never by
//! intercepting firmware functions. The OLT sits alongside the
//! EPON MAC and MACsec peripherals and coordinates frame injection
//! through the LLID bitmap and mailbox read engine.
//!
//! # Architecture
//!
//! ```text
//!   ONU firmware (firmware)
//!       ↑ RX frames        ↓ TX frames
//!   ┌───┴──────────────────┴───┐
//!   │   EPON MAC + MACsec      │  MMIO registers
//!   │   (LLID bitmap, mailbox) │
//!   └───┬──────────────────┬───┘
//!       ↑ inject frames    ↓ capture TX
//!   ┌───┴──────────────────┴───┐
//!   │   OLT Emulator           │  This module
//!   │   - MPCP state machine   │
//!   │   - OAM keepalive        │
//!   │   - Frame TX/RX logs     │
//!   └──────────────────────────┘
//! ```
//!
//! # Frame injection path
//!
//! The firmware reads RX frames through a mailbox-style engine:
//!
//! 1. Firmware calls `epon_llid_bitmap_check(pin)` which reads
//!    `*(bitmap_base + (pin >> 5) * 0x200)` and checks bit
//!    `(pin & 0x1f)`.
//! 2. If the bit is set, firmware calls
//!    `macsec_hw_read_sa_data_block(pin, buffer, 0x605)` which:
//!    a. Writes a command word to the SA programming register
//!    b. Polls `hw_mailbox_poll_read_response(pin)` to read
//!       frame words one at a time
//! 3. The frame is dispatched by EtherType:
//!    - `0x8808` → `mpcp_rx_frame_dispatch`
//!    - `0x8809` → OAM processing
//!    - `0x888E` → EAP key exchange
//!
//! The OLT injects frames by:
//! - Setting bits in the LLID bitmap register
//! - Queueing frame data words for the mailbox read engine
//!
//! # Protocol layers
//!
//! - **MPCP** (IEEE 802.3 Clause 64): Discovery, registration,
//!   GATE grant assignment, timestamp sync
//! - **OAM** (IEEE 802.3 Clause 57): Info PDU keepalive,
//!   DPoE OAM extensions
//!
//! # Configuration
//!
//! The OLT is disabled by default (backward compatible). Enable
//! via `--olt-enable` CLI flag or `olt_enable` MCP tool.

use std::collections::{HashMap, VecDeque};

use crate::soc::peripheral::{
    AddressRange, Peripheral, PeripheralError, PeripheralEvent, PeripheralSnapshot,
};
use crate::cpu::exception::Exception;

// ── Constants ────────────────────────────────────────────────────────

/// OLT MAC address (configurable, default is a plausible Broadcom OLT).
const DEFAULT_OLT_MAC: [u8; 6] = [0x02, 0x00, 0x00, 0x01, 0x00, 0x01];

/// Standard EPON multicast destination for MPCP frames.
#[cfg(test)]
const MPCP_MULTICAST_DA: [u8; 6] = [0x01, 0x80, 0xC2, 0x00, 0x00, 0x01];

/// Standard slow-protocols multicast for OAM.
const OAM_MULTICAST_DA: [u8; 6] = [0x01, 0x80, 0xC2, 0x00, 0x00, 0x02];

/// EtherType: MPCP (Multi-Point Control Protocol).
const ETHERTYPE_MPCP: u16 = 0x8808;
/// EtherType: Slow Protocols (OAM).
const ETHERTYPE_OAM: u16 = 0x8809;

/// MPCP opcodes (IEEE 802.3ah / Clause 64).
const MPCP_OPCODE_GATE: u16 = 2;
const MPCP_OPCODE_REPORT: u16 = 3;
const MPCP_OPCODE_REGISTER_REQ: u16 = 4;
const MPCP_OPCODE_REGISTER: u16 = 5;
const MPCP_OPCODE_REGISTER_ACK: u16 = 6;

/// OAM subtype for slow-protocols.
const OAM_SUBTYPE: u8 = 0x03;
/// OAM code: Information.
const OAM_CODE_INFO: u8 = 0x00;

/// Default LLID assigned to the first ONU.
const DEFAULT_LLID: u16 = 1;

/// Ticks between OAM keepalive transmissions (in bank tick units).
/// At BANK_TICK_PRESCALER=64 instructions/tick, and ~89 MIPS,
/// 64 ticks ≈ 4096 instructions ≈ 46 µs. We want ~1 second
/// between keepalives, so ~21700 ticks.
const OAM_KEEPALIVE_INTERVAL_TICKS: u64 = 20_000;

/// Ticks between periodic GATE frames.
const GATE_INTERVAL_TICKS: u64 = 30_000;

/// Delay before link_up activates after olt_enable. Lets the firmware
/// complete boot and SerDes init before seeing PHY link-change events.
/// ~900K ticks × 64 insns/tick ≈ 57M insns (just after boot).
const LINK_UP_DELAY_TICKS: u64 = 900_000;

/// Delay (in ticks) before responding to a REGISTER_REQ with REGISTER.
const REGISTER_RESPONSE_DELAY_TICKS: u64 = 100;

/// Maximum number of frames to log per direction.
const MAX_FRAME_LOG: usize = 256;

/// Maximum frame size (bytes).
const MAX_FRAME_SIZE: usize = 1600;

// ── Per-slot mailbox routing (from firmware RE) ─────────────────────

/// Mailbox slot for MPCP frames (EtherType 0x8808).
/// Firmware reads with CMD_STATUS = 0x400010, pin = 0x10.
const SLOT_MPCP: u8 = 0x10;
/// Mailbox slot for OAM slow-protocol frames (EtherType 0x8809).
/// Firmware reads with CMD_STATUS = 0x40000F, pin = 0x0F.
const SLOT_OAM: u8 = 0x0F;
/// Mailbox slot for MACsec/EAP frames (EtherType 0x888E).
/// Firmware reads with CMD_STATUS = 0x400000, pin = 0x00.
const SLOT_MACSEC: u8 = 0x00;

// ── DMA mailbox constants (reverse-engineered: the frame-protocol notes) ──────

/// LLID bitmap base — firmware reads `*(BITMAP_BASE + word_idx * 0x200)`.
const BITMAP_BASE: u32 = 0x0100_1438;
/// Command/status register base (dual-purpose R/W).
/// Write: `0x400000 | slot` to start frame read.
/// Read: bit 9 = data ready.
const CMD_STATUS_BASE: u32 = 0x0100_15C0;
/// Data register base — firmware reads 32-bit frame words sequentially.
const DATA_BASE: u32 = 0x0100_15C4;
/// Stride between word_index groups.
const MAILBOX_STRIDE: u32 = 0x200;

// ── OLT State Machine ───────────────────────────────────────────────

/// MPCP registration state from the OLT's perspective.
#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize)]
pub enum OltMpcpState {
    /// OLT is idle, waiting for ONU to send REGISTER_REQ.
    Idle,
    /// ONU sent REGISTER_REQ; OLT is preparing REGISTER response.
    Discovery,
    /// OLT sent REGISTER flags=1 (slot assignment), waiting for ONU REGISTER_ACK.
    WaitAck,
    /// OLT received first REGISTER_ACK, sent REGISTER flags=3 (confirm).
    WaitFinalAck,
    /// ONU acknowledged registration. MPCP is up.
    Registered,
}

impl std::fmt::Display for OltMpcpState {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            OltMpcpState::Idle => write!(f, "idle"),
            OltMpcpState::Discovery => write!(f, "discovery"),
            OltMpcpState::WaitAck => write!(f, "wait_ack"),
            OltMpcpState::WaitFinalAck => write!(f, "wait_final_ack"),
            OltMpcpState::Registered => write!(f, "registered"),
        }
    }
}

/// A captured or generated Ethernet frame.
#[derive(Clone, Debug)]
pub struct OltFrame {
    /// Raw frame bytes (dst_mac + src_mac + ethertype + payload).
    pub data: Vec<u8>,
    /// Bank tick when the frame was captured/generated.
    pub tick: u64,
    /// Human-readable description.
    pub description: String,
}

/// OLT configuration.
#[derive(Clone, Debug)]
pub struct OltConfig {
    /// Whether the OLT emulation is active.
    pub enabled: bool,
    /// OLT MAC address.
    pub mac: [u8; 6],
    /// Starting LLID for ONU registration.
    pub llid_start: u16,
    /// OAM keepalive interval in bank ticks.
    pub oam_interval_ticks: u64,
    /// GATE interval in bank ticks.
    pub gate_interval_ticks: u64,
    /// ONU MAC override for synthesized TX frames.
    pub onu_mac_override: Option<[u8; 6]>,
}

impl Default for OltConfig {
    fn default() -> Self {
        Self {
            enabled: false,
            mac: DEFAULT_OLT_MAC,
            llid_start: DEFAULT_LLID,
            oam_interval_ticks: OAM_KEEPALIVE_INTERVAL_TICKS,
            gate_interval_ticks: GATE_INTERVAL_TICKS,
            onu_mac_override: None,
        }
    }
}

/// Snapshot of OLT state for MCP/UI display.
#[derive(Clone, Debug, serde::Serialize)]
pub struct OltSnapshot {
    pub enabled: bool,
    pub mpcp_state: String,
    pub olt_mac: String,
    pub onu_mac: String,
    pub assigned_llid: u16,
    pub mpcp_timestamp: u32,
    pub tx_frame_count: usize,
    pub rx_frame_count: usize,
    pub oam_keepalive_count: u64,
    pub gate_count: u64,
}

// ── Main OLT Peripheral ─────────────────────────────────────────────

/// OLT emulator peripheral. Runs the EPON OLT protocol stack and
/// injects/captures frames through the EPON MAC DMA path.
#[derive(Clone)]
pub struct Olt {
    pub config: OltConfig,

    /// Current MPCP state.
    mpcp_state: OltMpcpState,
    /// ONU MAC address (learned from REGISTER_REQ).
    onu_mac: [u8; 6],
    /// Assigned LLID for the registered ONU.
    assigned_llid: u16,
    /// Running MPCP timestamp (incremented each tick).
    pub mpcp_timestamp: u32,

    /// Tick counter since the OLT was enabled.
    ticks_elapsed: u64,
    /// Tick of last OAM keepalive sent.
    last_oam_tick: u64,
    /// Tick of last GATE sent.
    last_gate_tick: u64,
    /// Tick when REGISTER_REQ was received (for delayed response).
    register_req_tick: Option<u64>,

    /// Number of OAM keepalives sent.
    oam_keepalive_count: u64,
    /// Number of GATE frames sent.
    gate_count: u64,

    /// Frames queued for injection into the firmware's RX path.
    /// The bank's tick handler drains these into the EPON MAC
    /// mailbox read engine.
    rx_inject_queue: VecDeque<Vec<u8>>,

    /// Log of frames captured from ONU TX (for MCP introspection).
    tx_log: VecDeque<OltFrame>,
    /// Log of frames injected to ONU RX (for MCP introspection).
    rx_log: VecDeque<OltFrame>,

    /// Whether the PHY link has been detected as up. The OLT
    /// auto-starts MPCP discovery once the link is up.
    pub link_up: bool,
    /// One-shot flag: set when link_up transitions to true.
    pub link_change_pending: bool,
    /// Countdown before link_up activates (0 = immediate or already fired).
    pub link_up_delay: u64,

    // ── DMA mailbox state ──────────────────────────────────────────
    /// Per-word_index bitmap values. When non-zero, the firmware's
    /// `epon_llid_bitmap_check` will see frames available.
    pub mailbox_bitmap: [u32; 8],
    /// FIFO of 32-bit words for the frame currently being read.
    /// Populated when the firmware writes a read command to CMD_STATUS.
    pub mailbox_fifo: VecDeque<u32>,
    /// Per-slot FIFOs of encoded frame words waiting for firmware reads.
    /// Key = slot number (pin), value = queue of encoded frames.
    /// Firmware extracts slot from CMD_STATUS write: `val & 0xFF`.
    pub mailbox_pending: HashMap<u8, VecDeque<Vec<u32>>>,

    pub trace: bool,

    /// When Some(llid), the bank should write this LLID to the EPON MAC
    /// match table at slot 0 (0x0100043C). Set when the OLT sends a
    /// REGISTER frame. Consumed by bank tick.
    pub pending_llid_update: Option<u16>,

    /// Set when OLT sends REGISTER flags=3. Bank generates a second
    /// auto-REGISTER_ACK to complete the handshake.
    pub pending_final_ack: bool,

    /// Set when `read_data()` pops the last word from the mailbox FIFO.
    /// The bank uses this to schedule a DMA-style clear of the firmware's
    /// Phase 1 guard struct at SRAM 0x7E3CA.
    pub frame_consumed: bool,

    /// One-shot flag: set when the OLT transitions to Registered.
    /// Bank uses this to initialize the LLID OAM state byte in SRAM,
    /// modelling the EPON MAC's internal state initialization that
    /// the firmware's ISR-driven path normally handles.
    pub registration_complete: bool,
    /// Countdown before writing the LLID OAM state byte. Delayed so
    /// the firmware's teardown/setup cycle (which clears the byte)
    /// finishes first.
    pub llid_state_init_countdown: u32,
}

impl Olt {
    pub fn new() -> Self {
        Self {
            config: OltConfig::default(),
            mpcp_state: OltMpcpState::Idle,
            onu_mac: [0; 6],
            assigned_llid: DEFAULT_LLID,
            mpcp_timestamp: 0,
            ticks_elapsed: 0,
            last_oam_tick: 0,
            last_gate_tick: 0,
            register_req_tick: None,
            oam_keepalive_count: 0,
            gate_count: 0,
            rx_inject_queue: VecDeque::new(),
            tx_log: VecDeque::new(),
            rx_log: VecDeque::new(),
            link_up: false,
            link_change_pending: false,
            link_up_delay: 0,
            mailbox_bitmap: [0; 8],
            mailbox_fifo: VecDeque::new(),
            mailbox_pending: HashMap::new(),
            trace: false,
            pending_llid_update: None,
            pending_final_ack: false,
            frame_consumed: false,
            registration_complete: false,
            llid_state_init_countdown: 0,
        }
    }

    /// Enable or disable OLT emulation.
    pub fn set_enabled(&mut self, enabled: bool) {
        self.config.enabled = enabled;
        if enabled {
            eprintln!("[OLT] Enabled — OLT MAC {:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X}",
                self.config.mac[0], self.config.mac[1], self.config.mac[2],
                self.config.mac[3], self.config.mac[4], self.config.mac[5]);
            eprintln!("[OLT] link_up deferred — fires after {} ticks", LINK_UP_DELAY_TICKS);
            self.link_up_delay = LINK_UP_DELAY_TICKS;
        } else {
            self.set_link_up(false);
            self.link_up_delay = 0;
            eprintln!("[OLT] Disabled");
        }
    }

    /// Return the current MPCP state.
    pub fn mpcp_state(&self) -> OltMpcpState {
        self.mpcp_state
    }

    pub fn get_onu_mac(&self) -> [u8; 6] {
        self.onu_mac
    }

    pub fn assigned_llid(&self) -> u16 {
        self.assigned_llid
    }

    /// Return a reference to the TX log (ONU → OLT frames).
    pub fn tx_log(&self) -> &VecDeque<OltFrame> {
        &self.tx_log
    }

    /// Return a reference to the RX log (OLT → ONU frames).
    pub fn rx_log(&self) -> &VecDeque<OltFrame> {
        &self.rx_log
    }

    /// Inject a raw frame into the ONU's RX path. The frame will
    /// be queued and delivered on the next tick.
    pub fn inject_raw_frame(&mut self, frame: Vec<u8>) {
        if frame.len() < 14 {
            return; // Too short for an Ethernet frame
        }
        self.log_rx_frame(&frame, "manual inject");
        self.rx_inject_queue.push_back(frame);
    }

    /// Take all pending RX frames for injection into the EPON MAC.
    /// Called by the peripheral bank after tick.
    pub fn take_rx_frames(&mut self) -> Vec<Vec<u8>> {
        self.rx_inject_queue.drain(..).collect()
    }

    // ── DMA mailbox integration ────────────────────────────────────

    /// Encode a raw Ethernet frame into the mailbox word sequence
    /// that `hw_mailbox_poll_read_response` returns to the firmware.
    ///
    /// Format (from RE of `macsec_hw_read_sa_data_block`):
    /// - Word 0: header (status bytes, low 16 bits)
    /// - Word 1: bits[26:16] = payload_len + 6, bits[15:0] = first 2 bytes
    /// - Words 2..N: remaining payload, 4 bytes/word, big-endian
    fn encode_frame_to_mailbox_words(frame: &[u8]) -> Vec<u32> {
        let mut words = Vec::new();
        // Word 0: header — 0 status (no errors)
        words.push(0u32);
        // Word 1: length field + first 2 bytes of frame
        let payload_len = frame.len();
        let len_field = ((payload_len + 6) as u32 & 0x7FF) << 16;
        let first2 = if frame.len() >= 2 {
            ((frame[0] as u32) << 8) | (frame[1] as u32)
        } else if frame.len() == 1 {
            (frame[0] as u32) << 8
        } else {
            0
        };
        words.push(len_field | first2);
        // Words 2..N: remaining bytes starting from offset 2
        let remaining = if frame.len() > 2 { &frame[2..] } else { &[] };
        for chunk in remaining.chunks(4) {
            let mut word = 0u32;
            for (i, &b) in chunk.iter().enumerate() {
                word |= (b as u32) << (24 - i * 8);
            }
            words.push(word);
        }
        words
    }

    /// Determine the mailbox slot for a frame based on EtherType.
    /// On real BCM55030, both MPCP (0x8808) and OAM (0x8809) are
    /// control-plane frames routed to the same mailbox slot. The
    /// firmware's Phase 1 dispatch loop processes the MPCP queue
    /// first; routing OAM here ensures it's handled in that loop
    /// rather than requiring a separate data-queue check.
    fn slot_for_frame(frame: &[u8]) -> u8 {
        if frame.len() >= 14 {
            let ethertype = u16::from_be_bytes([frame[12], frame[13]]);
            match ethertype {
                ETHERTYPE_MPCP | ETHERTYPE_OAM => SLOT_MPCP,
                0x888E => SLOT_MACSEC,
                _ => SLOT_OAM,
            }
        } else {
            SLOT_OAM
        }
    }

    /// Load pending RX frames into per-slot mailbox FIFOs. Called from
    /// bank.tick() after the OLT's own tick. Routes each frame to the
    /// correct slot based on EtherType.
    pub fn load_frames_into_mailbox(&mut self) {
        let frames = self.take_rx_frames();
        for frame in frames {
            let slot = Self::slot_for_frame(&frame);
            let words = Self::encode_frame_to_mailbox_words(&frame);
            self.mailbox_pending
                .entry(slot)
                .or_default()
                .push_back(words);
        }
        self.refresh_bitmap();
    }

    /// Recompute the bitmap array from the per-slot pending queues.
    pub fn refresh_bitmap(&mut self) {
        self.mailbox_bitmap = [0; 8];
        for (&slot, queue) in &self.mailbox_pending {
            if !queue.is_empty() {
                let wi = (slot >> 5) as usize;
                let bit = slot & 0x1F;
                if wi < self.mailbox_bitmap.len() {
                    self.mailbox_bitmap[wi] |= 1 << bit;
                }
            }
        }
    }

    /// Check if any slot has pending frames.
    pub fn has_any_pending(&self) -> bool {
        self.mailbox_pending.values().any(|q| !q.is_empty())
    }

    /// Total number of pending frames across all slots.
    pub fn total_pending_count(&self) -> usize {
        self.mailbox_pending.values().map(|q| q.len()).sum()
    }

    /// Handle a read of the LLID bitmap register.
    /// Returns `Some(bitmap)` for any valid word index when the OLT is
    /// enabled — including 0 — so the read never falls through to
    /// epon_mac's LLID backing store, which may contain stale bits.
    pub fn read_bitmap(&self, addr: u32) -> Option<u32> {
        if !self.config.enabled { return None; }
        let offset = addr.wrapping_sub(BITMAP_BASE);
        if offset % MAILBOX_STRIDE != 0 { return None; }
        let wi = (offset / MAILBOX_STRIDE) as usize;
        if wi < self.mailbox_bitmap.len() {
            Some(self.mailbox_bitmap[wi])
        } else {
            None
        }
    }

    /// Handle a read of the CMD/STATUS register (bit 9 = data ready).
    /// Returns `Some(0)` when enabled but FIFO is empty, preventing
    /// fallthrough to epon_mac which could return stale values.
    pub fn read_cmd_status(&self, addr: u32) -> Option<u32> {
        if !self.config.enabled { return None; }
        let offset = addr.wrapping_sub(CMD_STATUS_BASE);
        if offset % MAILBOX_STRIDE != 0 { return None; }
        if !self.mailbox_fifo.is_empty() {
            Some(0x200) // bit 9 = data ready
        } else {
            Some(0)
        }
    }

    /// Handle a read of the DATA register (pop next word).
    /// Returns `Some(0)` when enabled but FIFO is empty, preventing
    /// fallthrough to epon_mac. On real HW the mailbox returns 0
    /// when no data is pending.
    pub fn read_data(&mut self, addr: u32) -> Option<u32> {
        if !self.config.enabled { return None; }
        let offset = addr.wrapping_sub(DATA_BASE);
        if offset % MAILBOX_STRIDE != 0 { return None; }
        if let Some(word) = self.mailbox_fifo.pop_front() {
            if self.mailbox_fifo.is_empty() {
                self.frame_consumed = true;
            }
            Some(word)
        } else {
            Some(0)
        }
    }

    /// Handle a write to the CMD/STATUS register.
    /// Value `0x400000 | slot` = start reading a frame from slot's FIFO.
    pub fn write_cmd(&mut self, addr: u32, val: u32) -> bool {
        if !self.config.enabled { return false; }
        let offset = addr.wrapping_sub(CMD_STATUS_BASE);
        if offset % MAILBOX_STRIDE != 0 { return false; }
        if val & 0x400000 != 0 {
            let slot = (val & 0xFF) as u8;
            self.mailbox_fifo.clear();
            if let Some(queue) = self.mailbox_pending.get_mut(&slot) {
                if let Some(words) = queue.pop_front() {
                    self.mailbox_fifo.extend(words);
                    if self.trace {
                        eprintln!("[OLT] mailbox: loaded {} words for slot 0x{:02X} (cmd @ +0x{:X})",
                            self.mailbox_fifo.len(), slot, offset);
                    }
                }
                if queue.is_empty() {
                    self.mailbox_pending.remove(&slot);
                }
            }
            self.refresh_bitmap();
            return true;
        }
        false
    }

    /// Check if an address is in the OLT mailbox range.
    pub fn claims_mailbox(addr: u32) -> bool {
        // Bitmap range: BITMAP_BASE + [0..7] * STRIDE
        // CMD/STATUS range: CMD_STATUS_BASE + [0..7] * STRIDE
        // DATA range: DATA_BASE + [0..7] * STRIDE
        let max_offset = 7 * MAILBOX_STRIDE;
        let in_bitmap = addr >= BITMAP_BASE && addr <= BITMAP_BASE + max_offset
            && (addr - BITMAP_BASE) % MAILBOX_STRIDE == 0;
        let in_cmd = addr >= CMD_STATUS_BASE && addr <= CMD_STATUS_BASE + max_offset
            && (addr - CMD_STATUS_BASE) % MAILBOX_STRIDE == 0;
        let in_data = addr >= DATA_BASE && addr <= DATA_BASE + max_offset
            && (addr - DATA_BASE) % MAILBOX_STRIDE == 0;
        in_bitmap || in_cmd || in_data
    }

    /// Notify the OLT that a TX frame was sent by the ONU firmware.
    /// This is called when the bank detects a DMA TX completion or
    /// MPCP command latch write.
    pub fn on_tx_frame(&mut self, frame: &[u8]) {
        if !self.config.enabled || frame.len() < 14 {
            return;
        }

        // Parse EtherType (offset 12-13).
        let ethertype = u16::from_be_bytes([frame[12], frame[13]]);

        let desc = match ethertype {
            ETHERTYPE_MPCP => self.handle_mpcp_tx(frame),
            ETHERTYPE_OAM => self.handle_oam_tx(frame),
            _ => format!("unknown ethertype 0x{:04X} ({} bytes)", ethertype, frame.len()),
        };

        self.log_tx_frame(frame, &desc);
    }

    /// Notify the OLT that the PHY link state changed.
    pub fn set_link_up(&mut self, up: bool) {
        if self.link_up != up {
            self.link_up = up;
            self.link_change_pending = true;
            if up && self.config.enabled {
                eprintln!("[OLT] PHY link up — starting MPCP discovery");
                self.mpcp_state = OltMpcpState::Idle;
            }
        }
    }

    // ── MPCP TX handling (ONU → OLT) ────────────────────────────────

    /// Process a frame transmitted by the firmware's burst controller.
    pub fn handle_tx_frame(&mut self, frame: &[u8]) {
        if frame.len() < 14 {
            return;
        }
        let ethertype = u16::from_be_bytes([frame[12], frame[13]]);
        if ethertype == 0x8808 {
            let desc = self.handle_mpcp_tx(frame);
            self.log_tx_frame(frame, &desc);
        } else {
            self.log_tx_frame(frame, "non-MPCP TX");
        }
    }

    fn handle_mpcp_tx(&mut self, frame: &[u8]) -> String {
        if frame.len() < 16 {
            return "MPCP frame too short".into();
        }

        // MPCP opcode at offset 14-15.
        let opcode = u16::from_be_bytes([frame[14], frame[15]]);

        match opcode {
            MPCP_OPCODE_REGISTER_REQ => {
                // Learn the ONU's source MAC from the frame.
                self.onu_mac.copy_from_slice(&frame[6..12]);
                eprintln!(
                    "[OLT] Received REGISTER_REQ from {:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X}",
                    self.onu_mac[0], self.onu_mac[1], self.onu_mac[2],
                    self.onu_mac[3], self.onu_mac[4], self.onu_mac[5]
                );
                // Only restart discovery if idle — ignore duplicate REQs
                // while the handshake is in progress.
                if self.mpcp_state == OltMpcpState::Idle {
                    self.mpcp_state = OltMpcpState::Discovery;
                    self.register_req_tick = Some(self.ticks_elapsed);
                }
                format!(
                    "REGISTER_REQ from {:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X}",
                    self.onu_mac[0], self.onu_mac[1], self.onu_mac[2],
                    self.onu_mac[3], self.onu_mac[4], self.onu_mac[5]
                )
            }
            MPCP_OPCODE_REGISTER_ACK => {
                if self.mpcp_state == OltMpcpState::WaitAck {
                    eprintln!(
                        "[OLT] REGISTER_ACK received — sending REGISTER flags=3 (LLID={})",
                        self.assigned_llid
                    );
                    let confirm_frame = self.build_mpcp_register(0x03);
                    self.log_rx_frame(&confirm_frame, "REGISTER flags=3");
                    self.rx_inject_queue.push_back(confirm_frame);
                    self.pending_final_ack = true;
                    self.mpcp_state = OltMpcpState::WaitFinalAck;
                } else if self.mpcp_state == OltMpcpState::WaitFinalAck {
                    self.mpcp_state = OltMpcpState::Registered;
                    self.registration_complete = true;
                    eprintln!(
                        "[OLT] Final REGISTER_ACK received — ONU registered (LLID={})",
                        self.assigned_llid
                    );
                }
                format!("REGISTER_ACK (LLID={})", self.assigned_llid)
            }
            5 => {
                // REPORT (opcode 5) — ONU bandwidth request, no action needed.
                "REPORT".into()
            }
            _ => format!("MPCP opcode {} ({} bytes)", opcode, frame.len()),
        }
    }

    fn handle_oam_tx(&mut self, frame: &[u8]) -> String {
        if frame.len() < 18 {
            return "OAM frame too short".into();
        }
        let subtype = frame[14];
        let flags = u16::from_be_bytes([frame[15], frame[16]]);
        let code = frame[17];
        format!(
            "OAM subtype={} flags=0x{:04X} code={} ({} bytes)",
            subtype, flags, code, frame.len()
        )
    }

    // ── Frame builders ──────────────────────────────────────────────

    /// Build an MPCP REGISTER frame (opcode 5).
    /// `flags`: 0x01 = register (slot assignment), 0x03 = register+ack (confirm).
    fn build_mpcp_register(&self, flags: u8) -> Vec<u8> {
        let mut frame = Vec::with_capacity(64);

        // Destination: ONU MAC (learned from REGISTER_REQ).
        frame.extend_from_slice(&self.onu_mac);
        // Source: OLT MAC.
        frame.extend_from_slice(&self.config.mac);
        // EtherType: MPCP.
        frame.extend_from_slice(&ETHERTYPE_MPCP.to_be_bytes());
        // Opcode: REGISTER (5).
        frame.extend_from_slice(&MPCP_OPCODE_REGISTER.to_be_bytes());
        // Timestamp (4 bytes).
        frame.extend_from_slice(&self.mpcp_timestamp.to_be_bytes());
        // Assigned LLID (2 bytes) — port + LLID.
        frame.extend_from_slice(&self.assigned_llid.to_be_bytes());
        // Flags.
        frame.push(flags);
        // Sync time (2 bytes) — typical value.
        frame.extend_from_slice(&0x0032u16.to_be_bytes());
        // Pad to minimum Ethernet frame size.
        while frame.len() < 60 {
            frame.push(0x00);
        }
        frame
    }

    /// Build an MPCP GATE frame (opcode 2) for periodic grant
    /// assignment and timestamp synchronization.
    fn build_mpcp_gate(&self) -> Vec<u8> {
        let mut frame = Vec::with_capacity(64);

        // Destination: ONU MAC.
        frame.extend_from_slice(&self.onu_mac);
        // Source: OLT MAC.
        frame.extend_from_slice(&self.config.mac);
        // EtherType: MPCP.
        frame.extend_from_slice(&ETHERTYPE_MPCP.to_be_bytes());
        // Opcode: GATE (2).
        frame.extend_from_slice(&MPCP_OPCODE_GATE.to_be_bytes());
        // Timestamp (4 bytes).
        frame.extend_from_slice(&self.mpcp_timestamp.to_be_bytes());
        // Number of grants (1 byte) — 1 grant.
        frame.push(0x01);
        // Grant start time (4 bytes).
        frame.extend_from_slice(&self.mpcp_timestamp.to_be_bytes());
        // Grant length (2 bytes) — maximum grant window.
        frame.extend_from_slice(&0xFFFFu16.to_be_bytes());
        // Force report flag.
        frame.push(0x01);
        // Pad to minimum Ethernet frame size.
        while frame.len() < 60 {
            frame.push(0x00);
        }
        frame
    }

    /// Build an OAM Information PDU (code 0x00) for keepalive.
    fn build_oam_info(&self) -> Vec<u8> {
        let mut frame = Vec::with_capacity(64);

        // Destination: ONU MAC or OAM multicast.
        if self.onu_mac != [0; 6] {
            frame.extend_from_slice(&self.onu_mac);
        } else {
            frame.extend_from_slice(&OAM_MULTICAST_DA);
        }
        // Source: OLT MAC.
        frame.extend_from_slice(&self.config.mac);
        // EtherType: Slow Protocols.
        frame.extend_from_slice(&ETHERTYPE_OAM.to_be_bytes());
        // Subtype: OAM (0x03).
        frame.push(OAM_SUBTYPE);
        // Flags: link stable, local information valid.
        // Bits: [15:8] = 0x00, [7:0] = 0x08 (local_info_valid).
        frame.extend_from_slice(&0x0008u16.to_be_bytes());
        // Code: Information (0x00).
        frame.push(OAM_CODE_INFO);
        // Local Information TLV.
        frame.push(0x01); // Type: Local Information
        frame.push(0x10); // Length: 16 bytes
        // OAM version (1 byte).
        frame.push(0x01);
        // Revision (2 bytes).
        frame.extend_from_slice(&0x0001u16.to_be_bytes());
        // State (1 byte): MUX action = forward, parser action = forward.
        frame.push(0x00);
        // OAM configuration (1 byte): active mode, unidirectional support.
        frame.push(0x05);
        // Max OAMPDU size (2 bytes).
        frame.extend_from_slice(&0x05DCu16.to_be_bytes());
        // OUI (3 bytes) — Broadcom.
        frame.extend_from_slice(&[0x00, 0x10, 0x18]);
        // Vendor-specific (4 bytes).
        frame.extend_from_slice(&[0x00, 0x00, 0x00, 0x00]);
        // End marker TLV.
        frame.push(0x00); // Type: End
        frame.push(0x00); // Length: 0
        // Pad to minimum frame size.
        while frame.len() < 60 {
            frame.push(0x00);
        }
        frame
    }

    // ── Tick / state machine advancement ────────────────────────────

    /// Advance the OLT state machine. Called from the peripheral
    /// bank's tick handler.
    fn advance(&mut self) {
        if !self.config.enabled {
            return;
        }

        self.ticks_elapsed += 1;
        self.mpcp_timestamp = self.mpcp_timestamp.wrapping_add(1);

        // Handle delayed REGISTER response.
        if let Some(req_tick) = self.register_req_tick {
            if self.mpcp_state == OltMpcpState::Discovery
                && self.ticks_elapsed >= req_tick + REGISTER_RESPONSE_DELAY_TICKS
            {
                self.register_req_tick = None;
                let register_frame = self.build_mpcp_register(0x01);
                eprintln!(
                    "[OLT] Sending REGISTER flags=1 (LLID={}) to {:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X}",
                    self.assigned_llid,
                    self.onu_mac[0], self.onu_mac[1], self.onu_mac[2],
                    self.onu_mac[3], self.onu_mac[4], self.onu_mac[5]
                );
                self.log_rx_frame(&register_frame, "REGISTER flags=1");
                self.rx_inject_queue.push_back(register_frame);
                self.pending_llid_update = Some(self.assigned_llid);
                self.mpcp_state = OltMpcpState::WaitAck;
            }
        }

        // Periodic GATE frames once registered.
        if self.mpcp_state == OltMpcpState::Registered
            && self.ticks_elapsed >= self.last_gate_tick + self.config.gate_interval_ticks
        {
            self.last_gate_tick = self.ticks_elapsed;
            let gate_frame = self.build_mpcp_gate();
            self.log_rx_frame(&gate_frame, "GATE");
            self.rx_inject_queue.push_back(gate_frame);
            self.gate_count += 1;
        }

        // Periodic OAM keepalive — sent whenever link is up, regardless
        // of MPCP state. On real hardware the OLT sends OAM Info PDUs
        // before MPCP registration to prevent the ONU's keepalive
        // watchdog from timing out.
        if self.link_up
            && self.ticks_elapsed >= self.last_oam_tick + self.config.oam_interval_ticks
        {
            self.last_oam_tick = self.ticks_elapsed;
            let oam_frame = self.build_oam_info();
            self.log_rx_frame(&oam_frame, "OAM Info keepalive");
            self.rx_inject_queue.push_back(oam_frame);
            self.oam_keepalive_count += 1;
        }
    }

    // ── Logging ─────────────────────────────────────────────────────

    fn log_tx_frame(&mut self, frame: &[u8], description: &str) {
        if self.tx_log.len() >= MAX_FRAME_LOG {
            self.tx_log.pop_front();
        }
        self.tx_log.push_back(OltFrame {
            data: frame[..frame.len().min(MAX_FRAME_SIZE)].to_vec(),
            tick: self.ticks_elapsed,
            description: description.to_string(),
        });
        if self.trace {
            eprintln!(
                "[OLT TX] tick={} {} ({} bytes)",
                self.ticks_elapsed, description, frame.len()
            );
        }
    }

    fn log_rx_frame(&mut self, frame: &[u8], description: &str) {
        if self.rx_log.len() >= MAX_FRAME_LOG {
            self.rx_log.pop_front();
        }
        self.rx_log.push_back(OltFrame {
            data: frame[..frame.len().min(MAX_FRAME_SIZE)].to_vec(),
            tick: self.ticks_elapsed,
            description: description.to_string(),
        });
        if self.trace {
            eprintln!(
                "[OLT RX] tick={} {} ({} bytes)",
                self.ticks_elapsed, description, frame.len()
            );
        }
    }

    fn mac_to_string(mac: &[u8; 6]) -> String {
        format!(
            "{:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X}",
            mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]
        )
    }
}

// ── Peripheral trait ─────────────────────────────────────────────────

/// The OLT does not own any MMIO addresses itself — it piggybacks on
/// the EPON MAC and MACsec peripherals for frame injection. The
/// peripheral trait is implemented for integration with the bank's
/// tick/reset/snapshot infrastructure.

impl Peripheral for Olt {
    fn name(&self) -> &'static str {
        "olt"
    }

    fn address_ranges(&self) -> &'static [AddressRange] {
        // The OLT does not claim any MMIO ranges — it works by
        // manipulating other peripherals' state through the bank.
        const EMPTY: &[AddressRange] = &[];
        EMPTY
    }

    fn read_word(&mut self, _addr: u32) -> Result<u32, Exception> {
        Ok(0)
    }

    fn write_word(&mut self, _addr: u32, _val: u32) -> Result<(), Exception> {
        Ok(())
    }

    fn tick(&mut self, _cpu_instructions: u64) {
        if self.link_up_delay > 0 {
            self.link_up_delay -= 1;
            if self.link_up_delay == 0 {
                self.set_link_up(true);
            }
        }
        self.advance();
    }

    fn reset_cold(&mut self) {
        let config = self.config.clone();
        *self = Self::new();
        self.config = config;
    }

    fn reset_warm(&mut self) {
        self.reset_cold();
    }

    fn snapshot(&self) -> PeripheralSnapshot {
        PeripheralSnapshot::Olt(OltSnapshot {
            enabled: self.config.enabled,
            mpcp_state: format!("{}", self.mpcp_state),
            olt_mac: Self::mac_to_string(&self.config.mac),
            onu_mac: Self::mac_to_string(&self.onu_mac),
            assigned_llid: self.assigned_llid,
            mpcp_timestamp: self.mpcp_timestamp,
            tx_frame_count: self.tx_log.len(),
            rx_frame_count: self.rx_log.len(),
            oam_keepalive_count: self.oam_keepalive_count,
            gate_count: self.gate_count,
        })
    }

    fn inject_event(&mut self, event: &PeripheralEvent) -> Result<(), PeripheralError> {
        match event {
            PeripheralEvent::Olt(ev) => match ev {
                OltEvent::Enable(enabled) => {
                    self.set_enabled(*enabled);
                    Ok(())
                }
                OltEvent::SetMac(mac) => {
                    self.config.mac = *mac;
                    Ok(())
                }
                OltEvent::InjectFrame(frame) => {
                    self.inject_raw_frame(frame.clone());
                    Ok(())
                }
                OltEvent::SetLinkUp(up) => {
                    self.set_link_up(*up);
                    Ok(())
                }
            },
            _ => Err(PeripheralError::UnsupportedEvent),
        }
    }
}

// ── OLT Events ──────────────────────────────────────────────────────

/// UI/MCP-driven mutations for the OLT peripheral.
#[derive(Clone, Debug)]
pub enum OltEvent {
    /// Enable or disable OLT emulation.
    Enable(bool),
    /// Set the OLT's MAC address.
    SetMac([u8; 6]),
    /// Inject a raw frame into the ONU's RX path.
    InjectFrame(Vec<u8>),
    /// Set the PHY link state.
    SetLinkUp(bool),
}

// ── Tests ───────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_olt_is_disabled() {
        let olt = Olt::new();
        assert!(!olt.config.enabled);
        assert_eq!(olt.mpcp_state, OltMpcpState::Idle);
    }

    #[test]
    fn enable_disable_toggle() {
        let mut olt = Olt::new();
        olt.set_enabled(true);
        assert!(olt.config.enabled);
        olt.set_enabled(false);
        assert!(!olt.config.enabled);
    }

    #[test]
    fn register_req_triggers_discovery() {
        let mut olt = Olt::new();
        olt.set_enabled(true);
        olt.set_link_up(true);

        // Build a minimal REGISTER_REQ frame.
        let mut frame = vec![0u8; 64];
        // dst: MPCP multicast
        frame[0..6].copy_from_slice(&MPCP_MULTICAST_DA);
        // src: fake ONU MAC
        frame[6..12].copy_from_slice(&[0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF]);
        // EtherType: MPCP
        frame[12..14].copy_from_slice(&ETHERTYPE_MPCP.to_be_bytes());
        // Opcode: REGISTER_REQ (4)
        frame[14..16].copy_from_slice(&MPCP_OPCODE_REGISTER_REQ.to_be_bytes());

        olt.on_tx_frame(&frame);
        assert_eq!(olt.mpcp_state, OltMpcpState::Discovery);
        assert_eq!(olt.onu_mac, [0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF]);
    }

    #[test]
    fn register_response_after_delay() {
        let mut olt = Olt::new();
        olt.set_enabled(true);
        olt.set_link_up(true);

        // Send REGISTER_REQ.
        let mut frame = vec![0u8; 64];
        frame[0..6].copy_from_slice(&MPCP_MULTICAST_DA);
        frame[6..12].copy_from_slice(&[0x11, 0x22, 0x33, 0x44, 0x55, 0x66]);
        frame[12..14].copy_from_slice(&ETHERTYPE_MPCP.to_be_bytes());
        frame[14..16].copy_from_slice(&MPCP_OPCODE_REGISTER_REQ.to_be_bytes());
        olt.on_tx_frame(&frame);

        // Tick past the response delay.
        for _ in 0..=REGISTER_RESPONSE_DELAY_TICKS {
            olt.advance();
        }

        assert_eq!(olt.mpcp_state, OltMpcpState::WaitAck);
        assert!(!olt.rx_inject_queue.is_empty());

        // Check the injected REGISTER frame.
        let register = olt.rx_inject_queue.pop_front().unwrap();
        // EtherType should be MPCP.
        assert_eq!(
            u16::from_be_bytes([register[12], register[13]]),
            ETHERTYPE_MPCP
        );
        // Opcode should be REGISTER (3).
        assert_eq!(
            u16::from_be_bytes([register[14], register[15]]),
            MPCP_OPCODE_REGISTER
        );
    }

    #[test]
    fn register_ack_triggers_two_step_flow() {
        let mut olt = Olt::new();
        olt.set_enabled(true);
        olt.mpcp_state = OltMpcpState::WaitAck;

        let mut frame = vec![0u8; 64];
        frame[12..14].copy_from_slice(&ETHERTYPE_MPCP.to_be_bytes());
        frame[14..16].copy_from_slice(&MPCP_OPCODE_REGISTER_ACK.to_be_bytes());

        // First ACK → sends REGISTER flags=3, transitions to WaitFinalAck.
        olt.on_tx_frame(&frame);
        assert_eq!(olt.mpcp_state, OltMpcpState::WaitFinalAck);
        assert!(!olt.rx_inject_queue.is_empty());

        // Second ACK → transitions to Registered.
        olt.on_tx_frame(&frame);
        assert_eq!(olt.mpcp_state, OltMpcpState::Registered);
    }

    #[test]
    fn oam_keepalive_sent_periodically() {
        let mut olt = Olt::new();
        olt.set_enabled(true);
        olt.set_link_up(true);
        olt.mpcp_state = OltMpcpState::Registered;
        olt.onu_mac = [0x11, 0x22, 0x33, 0x44, 0x55, 0x66];
        olt.config.oam_interval_ticks = 100;

        for _ in 0..=100 {
            olt.advance();
        }

        assert!(olt.oam_keepalive_count >= 1);
        assert!(!olt.rx_inject_queue.is_empty());
    }

    #[test]
    fn gate_sent_periodically() {
        let mut olt = Olt::new();
        olt.set_enabled(true);
        olt.mpcp_state = OltMpcpState::Registered;
        olt.onu_mac = [0x11, 0x22, 0x33, 0x44, 0x55, 0x66];
        olt.config.gate_interval_ticks = 50;

        for _ in 0..=50 {
            olt.advance();
        }

        assert!(olt.gate_count >= 1);
    }

    #[test]
    fn inject_raw_frame() {
        let mut olt = Olt::new();
        let frame = vec![0xFFu8; 64];
        olt.inject_raw_frame(frame.clone());
        assert_eq!(olt.rx_inject_queue.len(), 1);
        assert_eq!(olt.rx_log.len(), 1);
    }

    #[test]
    fn cold_reset_preserves_config() {
        let mut olt = Olt::new();
        olt.config.enabled = true;
        olt.config.mac = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06];
        olt.mpcp_state = OltMpcpState::Registered;
        olt.oam_keepalive_count = 42;

        olt.reset_cold();

        assert!(olt.config.enabled);
        assert_eq!(olt.config.mac, [0x01, 0x02, 0x03, 0x04, 0x05, 0x06]);
        assert_eq!(olt.mpcp_state, OltMpcpState::Idle);
        assert_eq!(olt.oam_keepalive_count, 0);
    }

    #[test]
    fn frame_too_short_ignored() {
        let mut olt = Olt::new();
        olt.set_enabled(true);
        olt.on_tx_frame(&[0x00; 10]); // Too short
        assert_eq!(olt.tx_log.len(), 0);
    }

    #[test]
    fn per_slot_fifo_routes_by_ethertype() {
        let mut olt = Olt::new();
        olt.set_enabled(true);

        // OAM (0x8809) and MPCP (0x8808) share SLOT_MPCP (control plane).
        for _ in 0..3 {
            let mut oam = vec![0u8; 60];
            oam[12..14].copy_from_slice(&ETHERTYPE_OAM.to_be_bytes());
            olt.inject_raw_frame(oam);
        }
        let mut mpcp = vec![0u8; 60];
        mpcp[12..14].copy_from_slice(&ETHERTYPE_MPCP.to_be_bytes());
        olt.inject_raw_frame(mpcp);

        olt.load_frames_into_mailbox();

        assert_eq!(
            olt.mailbox_pending.get(&SLOT_MPCP).map(|q| q.len()),
            Some(4),
            "MPCP slot should have 4 frames (3 OAM + 1 MPCP)"
        );

        // Read first frame (OAM)
        assert!(olt.write_cmd(CMD_STATUS_BASE, 0x400000 | SLOT_MPCP as u32));
        assert!(!olt.mailbox_fifo.is_empty());
        assert_eq!(olt.mailbox_pending.get(&SLOT_MPCP).map(|q| q.len()), Some(3));
    }

    #[test]
    fn per_slot_bitmap_sets_correct_bits() {
        let mut olt = Olt::new();
        olt.set_enabled(true);

        // OAM and MPCP share SLOT_MPCP — only bit 16 should be set.
        let mut oam = vec![0u8; 60];
        oam[12..14].copy_from_slice(&ETHERTYPE_OAM.to_be_bytes());
        olt.inject_raw_frame(oam);

        let mut mpcp = vec![0u8; 60];
        mpcp[12..14].copy_from_slice(&ETHERTYPE_MPCP.to_be_bytes());
        olt.inject_raw_frame(mpcp);

        olt.load_frames_into_mailbox();

        assert_ne!(olt.mailbox_bitmap[0] & (1 << SLOT_MPCP), 0, "MPCP bit");
        assert_eq!(olt.mailbox_bitmap[0], 1 << SLOT_MPCP);

        // After reading first frame, still 1 pending → bit stays set
        olt.write_cmd(CMD_STATUS_BASE, 0x400000 | SLOT_MPCP as u32);
        assert_ne!(olt.mailbox_bitmap[0] & (1 << SLOT_MPCP), 0);

        // After reading second frame, queue empty → bit clears
        olt.write_cmd(CMD_STATUS_BASE, 0x400000 | SLOT_MPCP as u32);
        assert_eq!(olt.mailbox_bitmap[0], 0);
    }

    #[test]
    fn write_cmd_empty_slot_returns_true_clears_fifo() {
        let mut olt = Olt::new();
        olt.set_enabled(true);

        // Load MACsec frame only (uses SLOT_MACSEC = 0x00)
        let mut macsec = vec![0u8; 60];
        macsec[12..14].copy_from_slice(&0x888Eu16.to_be_bytes());
        olt.inject_raw_frame(macsec);
        olt.load_frames_into_mailbox();

        // Read from MPCP slot — no frame there, but CMD still intercepted
        assert!(olt.write_cmd(CMD_STATUS_BASE, 0x400000 | SLOT_MPCP as u32));
        assert!(olt.mailbox_fifo.is_empty(), "No MPCP frame to load");

        // CMD_STATUS read returns 0 (no data ready, but OLT intercepts)
        assert_eq!(olt.read_cmd_status(CMD_STATUS_BASE), Some(0));

        // MACsec slot still has its frame
        assert_eq!(olt.mailbox_pending.get(&SLOT_MACSEC).map(|q| q.len()), Some(1));
    }
}
