//! The link to the OLT, as this SoC sees it.
//!
//! The peer itself lives in the [`epon_olt`] crate and runs its own loop on
//! its own clock. What is left here is the join: the frame-queue mailbox the
//! firmware reads and writes, and the conversion between the two time bases —
//! CPU instructions on this side, picoseconds of link time on the other.
//!
//! That split is the point. The peer does not advance because the CPU executed
//! instructions; it advances because link time passed, and this module is what
//! says how much link time one tick is worth. Everything about the far end —
//! what it sends, when it decides to, how long a registration lasts — is
//! decided over there, by a model that can also be run with no CPU present at
//! all (`cargo run -p epon-olt --bin olt`).
//!
//! - [`mailbox`] MMIO decoding, uplink assembly, downlink word encoding

pub mod mailbox;
pub mod report;

use std::collections::{HashMap, VecDeque};

use crate::cpu::exception::Exception;
use crate::soc::peripheral::{
    AddressRange, Peripheral, PeripheralError, PeripheralEvent, PeripheralSnapshot,
};

use epon_olt::clock::{WireDuration, WireInstant};
use epon_olt::fibre::FibreConfig;
use epon_olt::link::Link;
use epon_olt::peer::PeerConfig;

use crate::soc::lue::{ClassifierBinding, Lue, Verdict};
use mailbox::{Command, Slot, TxAssembler};

// The protocol modules are re-exported so consumers keep addressing them
// through this module: they are the same types the peer works with.
pub use epon_olt::{decode, extended, mpcp, oam, state, types};
pub use epon_olt::{Counters as OltCounters, LoggedFrame as OltFrame};

pub use epon_olt::state::MpcpState as OltMpcpState;

use types::{Llid, MacAddr};

/// Duration of one bank tick, in picoseconds.
///
/// A tick is `BANK_TICK_PRESCALER` instructions of CPU time, which is what the
/// bank counts. It is the only bridge between the two clocks.
const TICK_PS: u64 = 720_900;

/// How much faster than the wire the link runs.
///
/// The host advances on instructions, so a run long enough to hold a full
/// exchange at the real cadence is expensive: one second of link time costs
/// about 89 million instructions. This multiplies how much link time one tick
/// is worth, which buys that back.
///
/// Because it scales the clock and not the intervals, everything stays
/// self-consistent: a duration the far end derives from two timestamps still
/// equals the interval that produced them. What it does not survive is any
/// comparison between link time and the CPU's own time — at scale N the far
/// end sees the link running N times faster than its own oscillator.
const DEFAULT_TIME_SCALE: u64 = 32;

/// Ticks before the link comes up, so the firmware finishes boot and SerDes
/// init before it sees a link-change edge.
///
/// This one is in ticks, not link time, and deliberately: what it waits for is
/// the CPU getting through its boot, which has nothing to do with the fibre.
const LINK_UP_DELAY_TICKS: u64 = 900_000;

/// Frames the receive queue holds before the line behind it starts backing up.
///
/// A hardware receive FIFO is a few frames deep. What does not fit stays on
/// the fibre, and what does not fit there is lost — which is what a downstream
/// nothing is draining actually does.
const MAILBOX_DEPTH: usize = 8;

#[derive(Clone, Debug)]
pub struct OltConfig {
    pub mac: MacAddr,
    pub llid_start: Llid,
    pub oam_interval_ms: u64,
    pub gate_interval_ms: u64,
    /// Address used for a synthesized uplink frame, before the real one
    /// teaches the model what the firmware actually sends.
    pub onu_mac_override: Option<MacAddr>,
    /// Multiplies how much link time a tick is worth. 1 runs at the wire's
    /// cadence.
    pub time_scale: u64,
    /// Attribute leaves read in rotation. Widen it to survey what the ONU
    /// answers to.
    pub polled_attributes: Vec<u16>,
}

impl Default for OltConfig {
    fn default() -> Self {
        let peer = PeerConfig::default();
        Self {
            mac: peer.mac,
            llid_start: peer.llid_start,
            oam_interval_ms: peer.oam_interval.as_ms(),
            gate_interval_ms: peer.gate_interval.as_ms(),
            onu_mac_override: None,
            time_scale: DEFAULT_TIME_SCALE,
            polled_attributes: peer.polled_attributes,
        }
    }
}

#[derive(Clone, Debug, serde::Serialize)]
pub struct OltSnapshot {
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

/// UI and MCP-driven mutations.
#[derive(Clone, Debug)]
pub enum OltEvent {
    SetMac([u8; 6]),
    InjectFrame(Vec<u8>),
    SetLinkUp(bool),
}

/// Where the downstream routing decision came from.
///
/// This is the SoC's own tally, deliberately **not** the peer's: the
/// peer's ledger accounts for frames on the link, and a decision taken
/// by silicon has no business in it. Same pattern, separate object.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct ClassifierRouting {
    /// Frames routed with no classifier consulted — the gate was shut.
    pub no_classifier: u64,
    /// Frames the classifier was asked about.
    pub classified: u64,
    /// Of those, the ones a rule matched and whose action named a queue.
    pub matched: u64,
    /// Matched, but the action does not name a queue: routing fell back.
    pub match_without_a_queue: u64,
    /// No rule matched; the specification calls this a drop, and the
    /// fallback routing it anyway is a known, counted divergence.
    pub no_match: u64,
    /// The rules could not decide. Counted separately from a miss,
    /// because they are different facts.
    pub undecidable: u64,
    /// Why the rules could not decide, summed over every frame. A bare
    /// `undecidable` says the classifier is blocked; this says on what,
    /// which is the difference between a number and a direction.
    pub refusals: crate::soc::lue::EngineCounters,
    /// Of the matched frames, the ones the EtherType fallback would have
    /// sent to the same queue.
    pub matched_agrees_with_fallback: u64,
    /// And the ones it would not. Two independent readings of where a
    /// frame belongs; counting only the agreements would make the check
    /// unable to fail.
    pub matched_differs_from_fallback: u64,
}

impl ClassifierRouting {
    /// Every frame handed to the classifier left exactly one tally.
    pub fn decisions_accounted_for(&self) -> bool {
        self.classified
            == self.matched + self.match_without_a_queue + self.no_match + self.undecidable
    }
}

#[derive(Clone)]
pub struct Olt {
    pub classifier_counters: ClassifierRouting,
    /// Frames landed per queue since the last drain, for the datapath to
    /// hand to the MAC's counter port. Kept here because this is where a
    /// frame is known to have landed, and drained rather than read so a
    /// count is reported exactly once.
    arrivals: HashMap<u8, u64>,
    pub config: OltConfig,
    /// The peer and the fibre either side of it.
    link: Link,

    /// Link time elapsed, which is what the peer runs on.
    wire_now: WireInstant,
    /// Bank ticks elapsed, which is what the host runs on.
    ticks_elapsed: u64,
    /// Mirror of the peer's timestamp, for the datapath registers that latch
    /// it. Reading it from here saves every consumer a conversion.
    pub mpcp_timestamp: u32,

    pub link_up: bool,
    pub link_change_pending: bool,
    pub link_up_delay: u64,

    /// One bitmap word per block; a set bit means that slot has a frame.
    pub mailbox_bitmap: [u32; 8],
    /// Words of the frame currently being read out.
    pub mailbox_fifo: VecDeque<u32>,
    /// Encoded frames waiting per slot.
    pub mailbox_pending: HashMap<u8, VecDeque<Vec<u32>>>,
    /// The MAC's report template, as the firmware programmed it.
    report_template: report::Template,
    /// Ordinary grants already answered, so each one is answered once.
    grants_answered: u64,
    /// Reassembles frames the firmware transmits.
    tx_assembler: TxAssembler,
    real_tx_seen: bool,

    pub trace: bool,

    /// LLID the bank should write into the MAC match table.
    pub pending_llid_update: Option<u16>,
    /// The last word of a frame was just read out.
    pub frame_consumed: bool,
    /// Registration completed; the bank initializes the LLID OAM state byte.
    pub registration_complete: bool,
    pub llid_state_init_countdown: u32,
}

impl Default for Olt {
    fn default() -> Self {
        Self::new()
    }
}

impl Olt {
    pub fn new() -> Self {
        let config = OltConfig::default();
        Self {
            classifier_counters: ClassifierRouting::default(),
            arrivals: HashMap::new(),
            link: Link::new(
                peer_config(&config),
                FibreConfig::downstream(),
                FibreConfig::upstream(),
            ),
            config,
            wire_now: WireInstant::ZERO,
            ticks_elapsed: 0,
            mpcp_timestamp: 0,
            link_up: false,
            link_change_pending: false,
            // The line is live from the start; the delay lets the firmware
            // finish boot before it sees the link-change edge.
            link_up_delay: LINK_UP_DELAY_TICKS,
            mailbox_bitmap: [0; 8],
            mailbox_fifo: VecDeque::new(),
            mailbox_pending: HashMap::new(),
            report_template: report::Template::default(),
            grants_answered: 0,
            tx_assembler: TxAssembler::default(),
            real_tx_seen: false,
            trace: false,
            pending_llid_update: None,
            frame_consumed: false,
            registration_complete: false,
            llid_state_init_countdown: 0,
        }
    }

    // ── Accessors ───────────────────────────────────────────────────

    pub fn mpcp_state(&self) -> OltMpcpState {
        self.link.peer.mpcp_state()
    }

    pub fn get_onu_mac(&self) -> [u8; 6] {
        self.link.peer.onu_mac().octets()
    }

    pub fn assigned_llid(&self) -> u16 {
        self.link.peer.assigned_llid().as_u16()
    }

    /// What the peer did with everything it was handed.
    pub fn counters(&self) -> &OltCounters {
        &self.link.peer.counters
    }

    /// Frames lost because a direction of the fibre was full.
    pub fn dropped_downstream(&self) -> u64 {
        self.link.downstream.dropped_full
    }

    pub fn dropped_upstream(&self) -> u64 {
        self.link.upstream.dropped_full
    }

    /// True once a frame has been reassembled from the transmit port.
    pub fn real_tx_seen(&self) -> bool {
        self.real_tx_seen
    }

    /// Submissions abandoned before completing.
    pub fn tx_dropped(&self) -> u64 {
        self.tx_assembler.dropped
    }

    /// Grant `llid` to the next ONU that registers.
    pub fn set_assigned_llid(&mut self, llid: Llid) {
        self.link.peer.set_assigned_llid(llid);
        self.config.llid_start = llid;
    }

    /// Discovery, which decides whether extended OAMPDUs may be exchanged.
    pub fn discovery(&self) -> oam::Discovery {
        self.link.peer.discovery()
    }

    /// Bank ticks elapsed.
    pub fn ticks_elapsed(&self) -> u64 {
        self.ticks_elapsed
    }

    /// Link time elapsed, which is what the peer's own timers run on.
    pub fn wire_now(&self) -> WireInstant {
        self.wire_now
    }

    /// True while an upstream submission is waiting for more words.
    pub fn tx_assembling(&self) -> bool {
        self.tx_assembler.is_assembling()
    }

    /// Build and queue an extended OAMPDU carrying `payload` after the vendor
    /// opcode. Returns the frame as it was queued.
    pub fn send_extended(
        &mut self,
        oui: oam::Oui,
        opcode: extended::Opcode,
        payload: &[u8],
    ) -> Vec<u8> {
        let frame = self.link.peer.send_extended(oui, opcode, payload);
        self.carry_downstream();
        frame
    }

    /// Read one attribute now, rather than waiting for the rotation.
    pub fn request_attribute(&mut self, leaf: u16) -> Vec<u8> {
        let frame = self.link.peer.request_attribute(leaf);
        self.carry_downstream();
        frame
    }

    /// Discovery state of the extended-OAM gate.
    pub fn discovery_state(&self) -> oam::DiscoveryState {
        self.link.peer.discovery().state()
    }

    /// Variables the peer has answered with.
    pub fn attribute_replies(&self) -> &VecDeque<extended::Container> {
        self.link.peer.attribute_replies()
    }

    /// True once discovery has started, which is when a GATE has gone out.
    pub fn has_broadcast_gate(&self) -> bool {
        self.link.peer.counters.gates_sent > 0
    }

    pub fn tx_log(&self) -> &VecDeque<OltFrame> {
        self.link.peer.tx_log()
    }

    pub fn rx_log(&self) -> &VecDeque<OltFrame> {
        self.link.peer.rx_log()
    }

    pub fn set_link_up(&mut self, up: bool) {
        if self.link_up == up {
            return;
        }
        self.link_up = up;
        self.link_change_pending = true;
        self.link.set_link(up, self.wire_now);
        if !up {
            self.mailbox_pending.clear();
            self.mailbox_fifo.clear();
            self.refresh_bitmap();
        }
        self.drain_peer_log();
    }

    /// Queue a frame for delivery to the firmware.
    pub fn inject_raw_frame(&mut self, frame: Vec<u8>) {
        self.link.peer.inject(frame);
        self.carry_downstream();
    }

    /// Move what the peer just decided to send onto the fibre, so a frame
    /// built outside a tick does not wait for the next one to start crossing.
    fn carry_downstream(&mut self) {
        for out in self.link.peer.take_downstream() {
            self.link.downstream.send(out.frame, out.at);
        }
    }

    fn drain_peer_log(&mut self) {
        for (at, line) in self.link.peer.take_log_lines() {
            eprintln!("[OLT] {at} {line}");
        }
    }

    // ── Mailbox ─────────────────────────────────────────────────────

    /// True when `addr` belongs to the mailbox ranges.
    pub fn claims_mailbox(addr: u32) -> bool {
        mailbox::claims(addr)
    }

    /// Take frames that have reached this end and put them in their slots.
    ///
    /// Only up to the depth of the queue: what does not fit stays on the
    /// fibre, where it either gets its turn or is dropped. Storing an
    /// unbounded backlog would deliver a burst of stale frames long after
    /// their windows closed.
    /// `classifier` is consulted only when the SoC has one and its gate
    /// is open. With no classifier, or with one that cannot decide, the
    /// EtherType fallback picks the queue — and says so in a counter,
    /// because a fallback nobody counts is indistinguishable from a
    /// classifier that worked.
    pub fn load_frames_into_mailbox(&mut self, classifier: Option<(&Lue, ClassifierBinding)>) {
        while self.total_pending_count() < MAILBOX_DEPTH {
            let Some(landed) = self.link.poll_downstream(self.wire_now) else {
                break;
            };
            let slot = self.choose_slot(classifier, &landed.frame);
            let words = mailbox::encode_frame(&landed.frame);
            self.mailbox_pending.entry(slot.0).or_default().push_back(words);
            *self.arrivals.entry(slot.0).or_insert(0) += 1;
        }
        self.refresh_bitmap();
    }

    /// Which queue a frame goes to, and why.
    ///
    /// Only one action result names a queue, and
    /// [`crate::soc::lue::Action::destination_queue`] is what says which.
    /// A rule that matches on any other result has decided something —
    /// just not where the frame goes — and that is counted separately
    /// from a miss, because they are different facts.
    fn choose_slot(
        &mut self,
        classifier: Option<(&Lue, ClassifierBinding)>,
        frame: &[u8],
    ) -> Slot {
        let Some((lue, binding)) = classifier else {
            self.classifier_counters.no_classifier += 1;
            return Slot::for_frame(frame);
        };
        self.classifier_counters.classified += 1;
        let (verdict, refusals) = lue.classify(binding, frame);
        self.classifier_counters.refusals.add(&refusals);
        match verdict {
            Verdict::Match { actions, .. } => match actions.iter().find_map(|a| a.destination_queue())
            {
                Some(queue) => {
                    self.classifier_counters.matched += 1;
                    if Slot(queue) == Slot::for_frame(frame) {
                        self.classifier_counters.matched_agrees_with_fallback += 1;
                    } else {
                        self.classifier_counters.matched_differs_from_fallback += 1;
                    }
                    Slot(queue)
                }
                None => {
                    // A match whose action names no queue decides
                    // nothing about routing.
                    self.classifier_counters.match_without_a_queue += 1;
                    Slot::for_frame(frame)
                }
            },
            Verdict::NoMatch => {
                self.classifier_counters.no_match += 1;
                Slot::for_frame(frame)
            }
            Verdict::Undecidable { .. } => {
                self.classifier_counters.undecidable += 1;
                Slot::for_frame(frame)
            }
        }
    }

    /// Take the arrivals recorded since the last call. Draining rather
    /// than reading means a frame is counted once and only once, however
    /// often the datapath asks.
    pub fn drain_arrivals(&mut self) -> Vec<(u8, u64)> {
        if self.arrivals.is_empty() {
            return Vec::new();
        }
        self.arrivals.drain().collect()
    }

    pub fn refresh_bitmap(&mut self) {
        self.mailbox_bitmap = [0; 8];
        for (&slot, queue) in &self.mailbox_pending {
            if queue.is_empty() {
                continue;
            }
            let (index, bit) = Slot(slot).bitmap_position();
            if let Some(word) = self.mailbox_bitmap.get_mut(index) {
                *word |= 1 << bit;
            }
        }
    }

    pub fn has_any_pending(&self) -> bool {
        self.mailbox_pending.values().any(|q| !q.is_empty())
    }

    pub fn total_pending_count(&self) -> usize {
        self.mailbox_pending.values().map(|q| q.len()).sum()
    }

    /// Read the LLID bitmap. Answers for every valid index, so the read never
    /// falls through to a stale backing store.
    pub fn read_bitmap(&self, addr: u32) -> Option<u32> {
        let index = mailbox::block_index(addr, mailbox::BITMAP_BASE)? as usize;
        self.mailbox_bitmap.get(index).copied()
    }

    /// Read the command/status port: data-ready when a word is queued.
    pub fn read_cmd_status(&self, addr: u32) -> Option<u32> {
        mailbox::block_index(addr, mailbox::CMD_STATUS_BASE)?;
        Some(if self.mailbox_fifo.is_empty() {
            0
        } else {
            mailbox::STATUS_DATA_READY
        })
    }

    /// Pop the next word from the data port.
    pub fn read_data(&mut self, addr: u32) -> Option<u32> {
        mailbox::block_index(addr, mailbox::DATA_BASE)?;
        match self.mailbox_fifo.pop_front() {
            Some(word) => {
                if self.mailbox_fifo.is_empty() {
                    self.frame_consumed = true;
                }
                Some(word)
            }
            None => Some(0),
        }
    }

    /// Handle a read command. Returns true when the write was consumed.
    pub fn write_cmd(&mut self, addr: u32, val: u32) -> bool {
        if mailbox::block_index(addr, mailbox::CMD_STATUS_BASE).is_none() {
            return false;
        }
        let Some(Command::Read { slot }) = Command::decode(val) else {
            return false;
        };
        self.mailbox_fifo.clear();
        if let Some(queue) = self.mailbox_pending.get_mut(&slot.0) {
            if let Some(words) = queue.pop_front() {
                self.mailbox_fifo.extend(words);
                if self.trace {
                    eprintln!(
                        "[OLT] mailbox: {} words for slot 0x{:02X}",
                        self.mailbox_fifo.len(),
                        slot.0
                    );
                }
            }
            if queue.is_empty() {
                self.mailbox_pending.remove(&slot.0);
            }
        }
        self.refresh_bitmap();
        true
    }

    /// Observe a write to the mailbox range and reassemble transmitted
    /// frames. Consumes nothing and changes no read-back: the write still
    /// reaches whichever peripheral claims it.
    /// True once the firmware has programmed a template the MAC could send.
    pub fn report_armed(&self) -> bool {
        self.report_template.is_armed()
    }

    /// Observe a write to the report template block. Separate from the
    /// mailbox: the template lives in the MAC's table space, not in the
    /// frame queue.
    pub fn observe_report_template(&mut self, addr: u32, val: u32) -> bool {
        self.report_template.observe_write(addr, val)
    }

    /// Answer any ordinary grant that has opened since the last check.
    ///
    /// This is the MAC's own doing: the CPU never builds an opcode-3 frame,
    /// so a REPORT can only come from here. One grant, one report.
    fn answer_grants(&mut self) {
        let granted = self.link.peer.counters.gates_normal_sent;
        if granted <= self.grants_answered || !self.report_template.is_armed() {
            self.grants_answered = granted;
            return;
        }
        self.grants_answered = granted;
        self.emit_report();
    }

    /// Emit a REPORT from the template, as the MAC does when a grant opens.
    ///
    /// Returns the frame if one went out. Nothing happens while the
    /// template is unprogrammed or no address is known — a MAC with
    /// nothing to send stays quiet rather than inventing a frame.
    pub fn emit_report(&mut self) -> Option<Vec<u8>> {
        let onu = self.link.peer.onu_mac();
        if onu.is_zero() {
            return None;
        }
        let frame = self.report_template.build(
            onu,
            self.link.peer.config.mac,
            self.wire_now.mpcp_timestamp(),
        )?;
        self.on_tx_frame(&frame);
        Some(frame)
    }

    pub fn observe_write(&mut self, addr: u32, val: u32) {
        if mailbox::block_index(addr, mailbox::CMD_STATUS_BASE).is_some() {
            if let Some(Command::Write { len, .. }) = Command::decode(val) {
                self.tx_assembler.begin(addr, len);
            }
            return;
        }
        if let Some(frame) = self.tx_assembler.push_word(addr, val) {
            self.on_captured_tx(frame);
        }
    }

    fn on_captured_tx(&mut self, frame: Vec<u8>) {
        if !self.real_tx_seen {
            self.real_tx_seen = true;
            eprintln!("[OLT] uplink capture live — {} byte frame", frame.len());
            // Whatever the synthesized path reached, it reached without a
            // frame ever leaving the firmware. Rewind so the first real
            // request meets the first real response.
            self.link.peer.rewind_handshake();
            self.registration_complete = false;
        }
        self.on_tx_frame(&frame);
    }

    // ── Uplink ──────────────────────────────────────────────────────

    /// Put a frame the firmware transmitted on the line.
    pub fn on_tx_frame(&mut self, frame: &[u8]) {
        self.link.send_upstream(frame.to_vec(), self.wire_now);
    }

    /// Put a frame the bank synthesized on the line.
    pub fn handle_tx_frame(&mut self, frame: &[u8]) {
        self.on_tx_frame(frame);
    }

    // ── Tick ────────────────────────────────────────────────────────

    /// Link time one tick is worth, at the configured scale.
    fn tick_duration(&self) -> WireDuration {
        WireDuration::from_ps(TICK_PS * self.config.time_scale.max(1))
    }

    fn advance(&mut self) {
        self.ticks_elapsed += 1;
        self.wire_now += self.tick_duration();
        self.link.advance_to(self.wire_now);
        self.mpcp_timestamp = self.wire_now.mpcp_timestamp();
        if let Some(llid) = self.link.peer.take_llid_assignment() {
            self.pending_llid_update = Some(llid);
        }
        if self.link.peer.take_registration_complete() {
            self.registration_complete = true;
        }
        self.drain_peer_log();
    }

    /// Push the host-facing configuration into the peer.
    fn apply_config(&mut self) {
        let peer = peer_config(&self.config);
        self.link.peer.set_assigned_llid(peer.llid_start);
        self.link.peer.config = peer;
    }
}

/// Translate the host-facing configuration into the peer's own.
fn peer_config(config: &OltConfig) -> PeerConfig {
    PeerConfig {
        mac: config.mac,
        llid_start: config.llid_start,
        oam_interval: WireDuration::from_ms(config.oam_interval_ms.max(1)),
        gate_interval: WireDuration::from_ms(config.gate_interval_ms.max(1)),
        polled_attributes: config.polled_attributes.clone(),
        ..PeerConfig::default()
    }
}

impl Peripheral for Olt {
    fn name(&self) -> &'static str {
        "olt"
    }

    /// The model claims no range of its own; the bank routes mailbox
    /// accesses to it explicitly.
    fn address_ranges(&self) -> &'static [AddressRange] {
        &[]
    }

    fn read_word(&mut self, _addr: u32) -> Result<u32, Exception> {
        Ok(0)
    }

    fn write_word(&mut self, _addr: u32, _val: u32) -> Result<(), Exception> {
        Ok(())
    }

    fn tick(&mut self, _cpu_instructions: u64) {
        // Configuration reaches the peer here rather than at every setter, so
        // a change made through the MCP or the UI lands on a tick boundary.
        self.apply_config();
        if self.link_up_delay > 0 {
            self.link_up_delay -= 1;
            if self.link_up_delay == 0 {
                self.set_link_up(true);
            }
        }
        self.advance();
        // A grant that has opened is a grant the MAC answers.
        self.answer_grants();
    }

    /// Reset the session, not the peer. The far end does not reboot because
    /// this SoC does, so config, trace and the fact that the line is live
    /// survive; MPCP state, queues, logs and mailbox do not.
    fn reset_cold(&mut self) {
        let config = self.config.clone();
        let trace = self.trace;
        // Live means up now or armed to come up; an explicit down stays down.
        let line_is_live = self.link_up || self.link_up_delay > 0;
        *self = Self::new();
        self.config = config;
        self.trace = trace;
        self.apply_config();
        // A fresh model arms the line; an explicit link-down is an
        // instruction, so it must survive the reset.
        self.link_up_delay = 0;
        if line_is_live {
            // Re-arm through the same deferral a fresh model uses: the
            // firmware coming up after this reset has never seen the link
            // rise, and needs the edge in the same order.
            self.link_up_delay = LINK_UP_DELAY_TICKS;
        }
    }

    fn reset_warm(&mut self) {
        self.reset_cold();
    }

    fn snapshot(&self) -> PeripheralSnapshot {
        let counters = self.counters();
        PeripheralSnapshot::Olt(OltSnapshot {
            mpcp_state: self.mpcp_state().to_string(),
            olt_mac: self.config.mac.to_string(),
            onu_mac: self.link.peer.onu_mac().to_string(),
            assigned_llid: self.assigned_llid(),
            mpcp_timestamp: self.mpcp_timestamp,
            tx_frame_count: self.tx_log().len(),
            rx_frame_count: self.rx_log().len(),
            oam_keepalive_count: counters.oam_keepalives_sent,
            gate_count: counters.gates_sent,
        })
    }

    fn inject_event(&mut self, event: &PeripheralEvent) -> Result<(), PeripheralError> {
        let PeripheralEvent::Olt(event) = event else {
            return Err(PeripheralError::UnsupportedEvent);
        };
        match event {
            OltEvent::SetMac(mac) => self.config.mac = MacAddr::new(*mac),
            OltEvent::InjectFrame(frame) => self.inject_raw_frame(frame.clone()),
            OltEvent::SetLinkUp(up) => self.set_link_up(*up),
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests;
