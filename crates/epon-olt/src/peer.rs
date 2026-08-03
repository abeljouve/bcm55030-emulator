//! The peer: an OLT that runs its own loop.
//!
//! It owns a clock, a scheduler and a protocol state machine, and nothing
//! else. It does not know what is on the other end of the fibre, how frames
//! reach it, or what advances its clock — which is what lets the same engine
//! sit beside a simulated ONU, drive a socket to real equipment, or run as a
//! protocol test harness with no ONU at all.
//!
//! The loop is three calls:
//!
//! ```text
//! peer.advance_to(now)              // fire whatever is due
//! peer.deliver(frame, arrived_at)   // hand it what arrived
//! peer.take_downstream()            // collect what it decided to send
//! ```
//!
//! Every duration it works with is a duration of link time, and every
//! decision it takes is stamped with the instant it was due — not with the
//! instant the host got round to it. A host that calls `advance_to` once per
//! second and one that calls it a million times a second see the same peer.

use std::collections::VecDeque;

use crate::clock::{WireDuration, WireInstant};
use crate::extended;
use crate::mpcp::{self, GateFlags, Grant, RegisterFlag, RegisterReqFlag};
use crate::oam;
use crate::sched::Scheduler;
use crate::state::MpcpState;
use crate::types::{EtherType, Llid, MacAddr};

/// Default peer address, from the locally-administered range.
pub const DEFAULT_OLT_MAC: MacAddr = MacAddr::new([0x02, 0x00, 0x00, 0x00, 0x00, 0x01]);
/// LLID handed to the first ONU that registers.
pub const DEFAULT_LLID: Llid = Llid(1);

/// Interval between OAM keepalives.
pub const OAM_KEEPALIVE_INTERVAL: WireDuration = WireDuration::from_ms(804);
/// Interval between periodic GATEs.
pub const GATE_INTERVAL: WireDuration = WireDuration::from_ms(1000);
/// Interval between attribute reads.
pub const ATTRIBUTE_POLL_INTERVAL: WireDuration = WireDuration::from_ms(7600);
/// How long a registration lasts before the peer tears it down.
///
/// The peer's own timer is a round minute. What the ONU observes is this plus
/// however long the exchange takes to cross the fibre and be acted on, which
/// is the delay model's business — writing the observed figure here would bake
/// a symptom into the one part of the model meant to explain it.
pub const REGISTRATION_LIFETIME: WireDuration = WireDuration::from_ms(60_000);
/// Pause between a teardown and the discovery that follows it.
pub const REDISCOVERY_DELAY: WireDuration = WireDuration::from_ms(693);
/// Turnaround between a request landing and its answer being on the line.
///
/// INFERRED: an OLT admitting an ONU has to place the grant in its schedule
/// before it can answer, so the answer is not immediate. The figure is a
/// plausible order of magnitude, not a measurement.
pub const PROCESSING_DELAY: WireDuration = WireDuration::from_ms(1);

/// Requests ignored before one is answered. The peer lets the first discovery
/// window pass and registers on the second.
pub const IGNORED_REGISTER_REQUESTS: u32 = 1;

// Window parameters, in MPCP time quanta. These travel inside the frame, so
// they do not depend on how fast the clock driving the peer runs.

/// Sync time advertised in the discovery GATE and echoed in REGISTER.
pub const SYNC_TIME_TQ: u16 = 32;
/// Laser on and off times echoed back in REGISTER.
pub const ECHOED_LASER_ON_TQ: u8 = 32;
pub const ECHOED_LASER_OFF_TQ: u8 = 32;
/// Width of the discovery grant window.
pub const DISCOVERY_GRANT_LENGTH_TQ: u16 = 8244;
/// How far ahead of the timestamp the discovery grant opens.
pub const DISCOVERY_GRANT_OFFSET_TQ: u32 = 22_806;
/// Discovery information field: which upstream and downstream rates the
/// window is open for.
///
/// The receiver ANDs it against its own mask and drops the GATE, before
/// logging anything, if nothing remains. That makes it tempting to set both
/// window bits and be done — but no row of the rate table has a window offered
/// at a rate the field does not also claim as a capability, so that value is
/// not a value the field can hold. This one is the 10/1 Gbit/s row, which is
/// what an ONU on this link answers with.
pub const DISCOVERY_INFORMATION: u16 = 0x0011;
/// Both window bits with no capability claimed. Not a row of the rate table;
/// kept because a receiver's mask test admits it and some equipment sends it.
pub const DISCOVERY_INFORMATION_BOTH_WINDOWS: u16 = 0x0030;
/// Grants carried by a discovery GATE.
pub const GATE_GRANT_COUNT: u8 = 1;

/// Interval between normal (non-discovery) GATEs once an ONU is registered.
///
/// INFERRED (802.3 clause 64), and **no firmware witness is available**: the
/// firmware discards a non-discovery GATE at its first test, without a counter or an
/// alternative branch, so nothing observable says how often one arrives or
/// which one asks for a report. This is a plausible service interval, not a
/// measurement.
pub const NORMAL_GATE_INTERVAL: WireDuration = WireDuration::from_ms(1000);

/// How far ahead of the timestamp a normal grant opens, and how wide it is.
///
/// INFERRED, same standing as the interval above.
pub const NORMAL_GRANT_OFFSET_TQ: u32 = 4_096;
pub const NORMAL_GRANT_LENGTH_TQ: u16 = 8_244;
/// Maximum OAMPDU size advertised in the local information TLV.
pub const MAX_OAM_PDU_SIZE: u16 = 0x0600;

/// Frames retained per direction.
pub const MAX_FRAME_LOG: usize = 256;
/// Bytes retained per logged frame.
pub const MAX_FRAME_SIZE: usize = 1600;
/// Log lines retained for the host to drain.
const MAX_LOG_LINES: usize = 256;

/// Attributes read in rotation by default: the ones the peer is known to ask
/// for. One read per request, so a reply can be attributed to it.
pub fn default_polled_attributes() -> Vec<u16> {
    vec![
        extended::leaf::FIRMWARE_INFO,
        extended::leaf::MANUFACTURER_INFO,
        extended::leaf::MANUFACTURER_ORG_NAME,
    ]
}

/// What the peer schedules for itself.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Timer {
    /// Broadcast the next discovery GATE.
    Gate,
    /// Send the next OAM Information keepalive.
    Keepalive,
    /// Read the next attribute in the rotation.
    AttributePoll,
    /// Answer a registration request that was accepted.
    RegisterResponse,
    /// Tear down a registration that has run its course.
    Lifetime,
    /// Resume discovery after a teardown.
    Rediscovery,
    /// Issue the next normal GATE. Only armed while registered — a grant
    /// window means nothing to an ONU that has no LLID.
    NormalGate,
}

/// How the peer behaves. Every interval is link time.
#[derive(Clone, Debug)]
pub struct PeerConfig {
    pub mac: MacAddr,
    pub llid_start: Llid,
    pub oam_interval: WireDuration,
    pub gate_interval: WireDuration,
    pub attribute_interval: WireDuration,
    pub registration_lifetime: WireDuration,
    pub rediscovery_delay: WireDuration,
    pub processing_delay: WireDuration,
    pub ignored_register_requests: u32,
    /// Interval between normal GATEs while registered.
    pub normal_gate_interval: WireDuration,
    /// Attribute leaves read in rotation. Widen it to survey what the ONU
    /// answers to.
    pub polled_attributes: Vec<u16>,
    /// Discovery information advertised in the GATE. A receiver whose mask
    /// shares no bit with it drops the frame without logging anything, so a
    /// wrong value here reads as silence, not as an error.
    pub discovery_information: u16,
    /// Address used for OAMPDUs before the ONU's own is known. Clause 57
    /// addresses them to the slow-protocol multicast throughout; sending them
    /// unicast once the address is known is what some equipment does.
    pub unicast_oam: bool,
    /// Width of an ordinary grant, in time quanta.
    pub normal_grant_length_tq: u16,
}

impl Default for PeerConfig {
    fn default() -> Self {
        Self {
            mac: DEFAULT_OLT_MAC,
            llid_start: DEFAULT_LLID,
            oam_interval: OAM_KEEPALIVE_INTERVAL,
            gate_interval: GATE_INTERVAL,
            attribute_interval: ATTRIBUTE_POLL_INTERVAL,
            registration_lifetime: REGISTRATION_LIFETIME,
            rediscovery_delay: REDISCOVERY_DELAY,
            processing_delay: PROCESSING_DELAY,
            ignored_register_requests: IGNORED_REGISTER_REQUESTS,
            normal_gate_interval: NORMAL_GATE_INTERVAL,
            polled_attributes: default_polled_attributes(),
            discovery_information: DISCOVERY_INFORMATION,
            normal_grant_length_tq: NORMAL_GRANT_LENGTH_TQ,
            unicast_oam: true,
        }
    }
}

/// What the peer did with what it was handed.
///
/// Every path that discards an upstream frame increments one of these, so the
/// question "where did the other requests go" has an answer that is not "read
/// the source". The invariant the host can assert is that every REGISTER_REQ
/// seen is accounted for exactly once.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, serde::Serialize, schemars::JsonSchema)]
pub struct Counters {
    pub frames_received: u64,
    pub frames_sent: u64,
    /// Received frames whose EtherType the peer does not route.
    pub frames_unrecognized: u64,

    pub register_req_seen: u64,
    pub register_req_accepted: u64,
    /// Refused because the link was between registrations.
    pub register_req_link_settling: u64,
    /// Refused because a handshake was already in flight.
    pub register_req_in_flight: u64,
    /// Refused because the peer was letting a discovery window pass.
    pub register_req_window_passed: u64,
    /// Refused because the frame did not parse as an MPCPDU.
    pub register_req_malformed: u64,
    /// The request asked to be torn down, not admitted.
    pub register_req_deregister: u64,

    pub register_ack_seen: u64,
    /// Ignored because no handshake was waiting for one.
    pub register_ack_unexpected: u64,
    /// The acknowledgement did not echo back what was granted, or refused it.
    pub register_ack_rejected: u64,

    /// Upstream MPCPDUs, by what they were.
    ///
    /// The report engine is the one thing the far end cannot be asked
    /// about: the CPU never builds an opcode-3 frame, so a report can
    /// only come from the MAC. Until this counter exists, "no report in
    /// N frames" has no denominator and is not a measurement.
    /// Every upstream MPCPDU, whatever it turned out to be.
    pub mpcp_upstream_seen: u64,
    pub reports_seen: u64,
    /// Upstream MPCPDUs of an opcode the peer has no branch for. Every
    /// upstream frame lands in exactly one of the counted buckets.
    pub mpcp_upstream_unhandled: u64,
    /// Upstream MPCPDUs carrying an opcode only the far end may send.
    pub mpcp_upstream_wrong_direction: u64,
    /// MPCPDUs the peer could not parse at all.
    pub mpcp_malformed: u64,

    pub gates_sent: u64,
    /// Gates sent without the discovery flag. The firmware discards
    /// those at its first test, without a counter or an alternative
    /// branch — so this is the only place their number exists.
    pub gates_normal_sent: u64,
    pub oam_keepalives_sent: u64,
    pub attribute_requests_sent: u64,
    pub attribute_replies_seen: u64,
    pub registrations: u64,
    pub deregistrations: u64,
}

impl Counters {
    /// Every upstream MPCPDU was disposed of exactly once. Without
    /// this, a frame can be logged and never counted, which is how a
    /// silence gets read as a clean link.
    pub fn upstream_mpcpdus_accounted_for(&self) -> bool {
        self.mpcp_upstream_seen
            == self.register_req_seen
                + self.register_ack_seen
                + self.reports_seen
                + self.mpcp_upstream_unhandled
                + self.mpcp_upstream_wrong_direction
                + self.mpcp_malformed
    }

    /// Every registration request seen was disposed of exactly once.
    pub fn register_requests_accounted_for(&self) -> bool {
        self.register_req_seen
            == self.register_req_accepted
                + self.register_req_link_settling
                + self.register_req_in_flight
                + self.register_req_window_passed
                + self.register_req_malformed
                + self.register_req_deregister
    }
}

/// A frame the peer logged, in either direction.
#[derive(Clone, Debug)]
pub struct LoggedFrame {
    pub data: Vec<u8>,
    /// Instant on the wire clock: when it was built, or when it arrived.
    pub at: WireInstant,
    pub description: String,
}

/// A frame the peer wants on the line.
#[derive(Clone, Debug)]
pub struct Emitted {
    /// Instant the peer decided to send it. What happens between here and the
    /// far end belongs to the fibre, not to the peer.
    pub at: WireInstant,
    pub frame: Vec<u8>,
    pub description: String,
}

#[derive(Clone)]
pub struct Peer {
    pub config: PeerConfig,

    now: WireInstant,
    sched: Scheduler<Timer>,

    mpcp_state: MpcpState,
    onu_mac: MacAddr,
    assigned_llid: Llid,
    link_up: bool,

    discovery: oam::Discovery,
    /// Requests seen since discovery last restarted.
    register_requests_seen: u32,
    /// Instant the current registration began at.
    registered_at: Option<WireInstant>,
    /// True while the peer is silent between a teardown and the next window.
    settling: bool,
    /// What the last request declared, to echo back in the REGISTER. Echoing
    /// a constant means the ONU can never tell it was heard.
    last_request: Option<mpcp::RegisterReqBody>,

    /// Next attribute to read, as an index into the polled set.
    next_attribute: usize,
    attribute_replies: VecDeque<extended::Container>,

    downstream: VecDeque<Emitted>,
    tx_log: VecDeque<LoggedFrame>,
    rx_log: VecDeque<LoggedFrame>,
    log_lines: VecDeque<(WireInstant, String)>,

    pub counters: Counters,
    /// LLID the host should publish, once, after a REGISTER goes out.
    llid_assignment: Option<u16>,
    /// A registration completed and the host has not been told yet.
    registration_complete: bool,
}

impl Default for Peer {
    fn default() -> Self {
        Self::new(PeerConfig::default())
    }
}

impl Peer {
    pub fn new(config: PeerConfig) -> Self {
        let assigned_llid = config.llid_start;
        Self {
            config,
            now: WireInstant::ZERO,
            sched: Scheduler::new(),
            mpcp_state: MpcpState::Idle,
            onu_mac: MacAddr::ZERO,
            assigned_llid,
            link_up: false,
            discovery: oam::Discovery::default(),
            register_requests_seen: 0,
            registered_at: None,
            settling: false,
            last_request: None,
            next_attribute: 0,
            attribute_replies: VecDeque::new(),
            downstream: VecDeque::new(),
            tx_log: VecDeque::new(),
            rx_log: VecDeque::new(),
            log_lines: VecDeque::new(),
            counters: Counters::default(),
            llid_assignment: None,
            registration_complete: false,
        }
    }

    // ── The loop ────────────────────────────────────────────────────

    pub fn now(&self) -> WireInstant {
        self.now
    }

    /// Run every event due at or before `now`.
    ///
    /// Each fires at the instant it was scheduled for, so a host that advances
    /// in coarse steps gets the same frames, with the same stamps, as one that
    /// advances in fine ones — only later.
    pub fn advance_to(&mut self, now: WireInstant) {
        while let Some((at, timer)) = self.sched.pop_due(now) {
            self.now = at;
            self.fire(timer);
        }
        self.now = self.now.max(now);
    }

    /// Instant the peer next has something to do, if anything.
    pub fn next_due(&mut self) -> Option<WireInstant> {
        self.sched.next_due()
    }

    /// Take the frames the peer has decided to send.
    pub fn take_downstream(&mut self) -> Vec<Emitted> {
        self.downstream.drain(..).collect()
    }

    /// Take the lines the peer wants said about what it did, each with the
    /// instant on the link clock it happened at.
    pub fn take_log_lines(&mut self) -> Vec<(WireInstant, String)> {
        self.log_lines.drain(..).collect()
    }

    fn fire(&mut self, timer: Timer) {
        match timer {
            Timer::Gate => {
                let frame = self.build_gate();
                // Counted from what the frame carries, not from what the
                // builder was meant to put there — the difference is the
                // whole point of the counter.
                if mpcp::Pdu::parse(&frame)
                    .is_some_and(|p| matches!(p.body, mpcp::Body::Gate { flags, .. } if !flags.discovery))
                {
                    self.counters.gates_normal_sent += 1;
                }
                self.emit(frame, "GATE");
                self.counters.gates_sent += 1;
                self.sched.arm_in(Timer::Gate, self.now, self.config.gate_interval);
            }
            Timer::NormalGate => {
                // Only while registered: the window is addressed to an LLID.
                if self.mpcp_state == MpcpState::Registered {
                    let frame = self.build_normal_gate();
                    self.emit(frame, "GATE (normal)");
                    self.counters.gates_sent += 1;
                    self.counters.gates_normal_sent += 1;
                    self.sched.arm_in(Timer::NormalGate, self.now, self.config.normal_gate_interval);
                }
            }
            Timer::Keepalive => {
                let frame = self.build_oam_info();
                self.emit(frame, "OAM information");
                self.counters.oam_keepalives_sent += 1;
                let was = self.discovery.state();
                self.discovery.on_sent();
                self.note_discovery_change(was);
                self.sched.arm_in(Timer::Keepalive, self.now, self.config.oam_interval);
            }
            Timer::AttributePoll => {
                self.poll_next_attribute();
                self.sched.arm_in(Timer::AttributePoll, self.now, self.config.attribute_interval);
            }
            Timer::RegisterResponse => self.answer_registration(),
            Timer::Lifetime => self.tear_down_registration(),
            Timer::Rediscovery => {
                self.settling = false;
                self.register_requests_seen = 0;
            }
        }
    }

    // ── Link ────────────────────────────────────────────────────────

    pub fn link_up(&self) -> bool {
        self.link_up
    }

    /// Bring the link up or down at `at`.
    ///
    /// Up starts the periodic traffic immediately: an ONU slaves its clock to
    /// the first GATE it sees, so there is nothing to wait for. Down stops it
    /// and takes discovery back to the beginning, because the peer has no
    /// evidence about an end it can no longer hear.
    pub fn set_link(&mut self, up: bool, at: WireInstant) {
        if self.link_up == up {
            return;
        }
        self.now = self.now.max(at);
        self.link_up = up;
        if up {
            self.log("link up — starting MPCP discovery".to_string());
            self.mpcp_state = MpcpState::Idle;
            self.sched.arm_at(Timer::Gate, self.now);
            self.sched.arm_at(Timer::Keepalive, self.now);
        } else {
            self.log("link down".to_string());
            self.sched.clear();
            self.mpcp_state = MpcpState::Idle;
            self.registered_at = None;
            self.settling = false;
            self.register_requests_seen = 0;
            self.discovery.reset();
        }
    }

    // ── Uplink ──────────────────────────────────────────────────────

    /// Hand the peer a frame that arrived at `at`.
    pub fn deliver(&mut self, frame: &[u8], at: WireInstant) {
        self.now = self.now.max(at);
        self.counters.frames_received += 1;
        let description = match EtherType::of_frame(frame) {
            Some(EtherType::Mpcp) => self.on_mpcp(frame),
            Some(EtherType::SlowProtocol) => self.on_oam(frame),
            Some(other) => format!("{other} ({} bytes)", frame.len()),
            None => {
                self.counters.frames_unrecognized += 1;
                return;
            }
        };
        let entry = self.frame_entry(frame, &description);
        push_capped(&mut self.tx_log, entry);
    }

    fn on_mpcp(&mut self, frame: &[u8]) -> String {
        self.counters.mpcp_upstream_seen += 1;
        let Some(pdu) = mpcp::Pdu::parse(frame) else {
            // A frame that did not parse is not a registration request
            // that failed: counting it as one made the registration
            // ledger disagree with itself.
            self.counters.mpcp_malformed += 1;
            return "malformed MPCPDU".into();
        };
        match pdu.header.opcode {
            mpcp::Opcode::RegisterReq => {
                self.onu_mac = pdu.header.src;
                self.counters.register_req_seen += 1;
                let mpcp::Body::RegisterReq(body) = pdu.body else {
                    self.counters.register_req_malformed += 1;
                    return "REGISTER_REQ without a body".into();
                };
                // Table 64-3: the same code point carries a request to be
                // torn down. Treating it as a registration is how a model
                // registers an ONU that asked to leave.
                if body.flag == RegisterReqFlag::Deregister {
                    self.counters.register_req_deregister += 1;
                    self.tear_down_registration();
                    return format!("REGISTER_REQ from {} (asking to deregister)", self.onu_mac);
                }
                self.last_request = Some(body);
                if self.settling {
                    // Between a teardown and the next window the peer is
                    // silent. Counted, so the silence is visible.
                    self.counters.register_req_link_settling += 1;
                    return format!("REGISTER_REQ from {} (link settling)", self.onu_mac);
                }
                if self.mpcp_state != MpcpState::Idle {
                    self.counters.register_req_in_flight += 1;
                    return format!("REGISTER_REQ from {} (in flight)", self.onu_mac);
                }
                self.register_requests_seen += 1;
                if self.register_requests_seen <= self.config.ignored_register_requests {
                    self.counters.register_req_window_passed += 1;
                    return format!(
                        "REGISTER_REQ from {} (window {} passed over)",
                        self.onu_mac, self.register_requests_seen
                    );
                }
                self.counters.register_req_accepted += 1;
                self.log(format!("REGISTER_REQ from {}", self.onu_mac));
                self.mpcp_state = MpcpState::Discovery;
                self.sched.arm_in(Timer::RegisterResponse, self.now, self.config.processing_delay);
                format!("REGISTER_REQ from {}", self.onu_mac)
            }
            mpcp::Opcode::RegisterAck => {
                self.counters.register_ack_seen += 1;
                let mpcp::Body::RegisterAck { flag, echoed_port, echoed_sync_time } = pdu.body
                else {
                    self.counters.register_ack_rejected += 1;
                    return "REGISTER_ACK without a body".into();
                };
                // An acknowledgement that echoes something else acknowledges
                // something else. Accepting it registers a link neither end
                // agreed on.
                let echoes_the_grant = flag.acknowledges()
                    && echoed_port == self.assigned_llid
                    && echoed_sync_time == SYNC_TIME_TQ;
                if !echoes_the_grant {
                    self.counters.register_ack_rejected += 1;
                    self.log(format!(
                        "REGISTER_ACK refused: {flag:?}, port {echoed_port}, sync {echoed_sync_time} \
                         (granted port {}, sync {SYNC_TIME_TQ})",
                        self.assigned_llid
                    ));
                    return format!("REGISTER_ACK refused ({flag:?}, port {echoed_port})");
                }
                self.on_register_ack();
                format!("REGISTER_ACK (LLID={})", self.assigned_llid)
            }
            mpcp::Opcode::Report => {
                self.counters.reports_seen += 1;
                format!("REPORT ({} bytes)", frame.len())
            }
            opcode if opcode.is_downstream_only() => {
                self.counters.mpcp_upstream_wrong_direction += 1;
                format!("{opcode} from the ONU, which cannot originate it")
            }
            opcode => {
                self.counters.mpcp_upstream_unhandled += 1;
                format!("{opcode} ({} bytes)", frame.len())
            }
        }
    }

    /// The REGISTER_ACK closes the handshake. The REGISTER that opened it
    /// already carried the granting flag, so there is nothing left to confirm.
    fn on_register_ack(&mut self) {
        if self.mpcp_state != MpcpState::WaitAck {
            self.counters.register_ack_unexpected += 1;
            return;
        }
        self.mpcp_state = MpcpState::Registered;
        self.registration_complete = true;
        self.registered_at = Some(self.now);
        self.counters.registrations += 1;
        self.sched.arm_in(Timer::Lifetime, self.now, self.config.registration_lifetime);
        // The grant windows start once there is an LLID to address them to.
        self.sched.arm_in(Timer::NormalGate, self.now, self.config.normal_gate_interval);
        let c = self.counters;
        self.log(format!(
            "registered — LLID={} ({} REGISTER_REQ seen: {} accepted, {} window passed over, \
             {} in flight, {} link settling)",
            self.assigned_llid,
            c.register_req_seen,
            c.register_req_accepted,
            c.register_req_window_passed,
            c.register_req_in_flight,
            c.register_req_link_settling,
        ));
    }

    fn on_oam(&mut self, frame: &[u8]) -> String {
        if let Some(pdu) = extended::Pdu::parse(frame) {
            return self.on_extended(pdu);
        }
        let Some(pdu) = oam::Pdu::parse(frame) else {
            return "malformed OAMPDU".into();
        };
        let was = self.discovery.state();
        // An Information PDU is the peer describing itself, which is the only
        // thing this end can have evaluated; anything else carries flags but
        // no description.
        if pdu.code == oam::Code::Information {
            self.discovery.on_peer_information(pdu.flags);
        } else {
            self.discovery.on_peer_flags(pdu.flags);
        }
        if self.discovery.state() != was {
            self.log(format!(
                "discovery {:?} -> {:?} (peer flags 0x{:04X})",
                was,
                self.discovery.state(),
                pdu.flags.as_u16()
            ));
            self.on_discovery_entered(self.discovery.state());
        }
        format!("OAM {} flags=0x{:04X} ({} bytes)", pdu.code, pdu.flags.as_u16(), frame.len())
    }

    fn on_extended(&mut self, pdu: extended::Pdu) -> String {
        for c in pdu.containers() {
            self.counters.attribute_replies_seen += 1;
            self.log(format!(
                "{} {} -> {:?} ({} bytes)",
                pdu.opcode,
                c.descriptor,
                c.length,
                c.value.len()
            ));
            if self.attribute_replies.len() >= MAX_FRAME_LOG {
                self.attribute_replies.pop_front();
            }
            self.attribute_replies.push_back(c.clone());
        }
        format!("{} {} ({} variables)", pdu.oui, pdu.opcode, pdu.variables.len())
    }

    // ── Registration ────────────────────────────────────────────────

    fn answer_registration(&mut self) {
        if self.mpcp_state != MpcpState::Discovery || self.settling {
            return;
        }
        // Ack is the value that grants an LLID; 1 and 2 deregister.
        let frame = self.build_register(RegisterFlag::Ack);
        self.log(format!("REGISTER LLID={} to {}", self.assigned_llid, self.onu_mac));
        self.emit(frame, "REGISTER");
        self.llid_assignment = Some(self.assigned_llid.as_u16());
        self.mpcp_state = MpcpState::WaitAck;
    }

    fn tear_down_registration(&mut self) {
        if self.registered_at.is_none() {
            return;
        }
        let frame = self.build_register(RegisterFlag::Deregister);
        self.log(format!("deregistering LLID={}", self.assigned_llid));
        self.emit(frame, "REGISTER deregister");
        self.counters.deregistrations += 1;
        self.registered_at = None;
        self.registration_complete = false;
        self.mpcp_state = MpcpState::Idle;
        // No LLID, no window to grant. The timer is re-armed by the next
        // registration rather than left firing into nothing.
        self.sched.cancel(Timer::NormalGate);
        let was = self.discovery.state();
        self.discovery.reset();
        self.note_discovery_change(was);
        self.sched.cancel(Timer::AttributePoll);
        self.settling = true;
        self.sched.arm_in(Timer::Rediscovery, self.now, self.config.rediscovery_delay);
    }

    fn note_discovery_change(&mut self, was: oam::DiscoveryState) {
        let state = self.discovery.state();
        if state == was {
            return;
        }
        self.log(format!("discovery {was:?} -> {state:?}"));
        self.on_discovery_entered(state);
    }

    /// Extended OAMPDUs only cross the sublayer once both ends are stable, so
    /// the attribute rotation is armed by convergence and disarmed by losing
    /// it — rather than being tried every time round the loop.
    fn on_discovery_entered(&mut self, state: oam::DiscoveryState) {
        if state == oam::DiscoveryState::Converged {
            self.sched.arm_at(Timer::AttributePoll, self.now);
        } else {
            self.sched.cancel(Timer::AttributePoll);
        }
    }

    fn poll_next_attribute(&mut self) {
        if self.onu_mac.is_zero() {
            return;
        }
        let leaves = &self.config.polled_attributes;
        if leaves.is_empty() {
            return;
        }
        let leaf = leaves[self.next_attribute % leaves.len()];
        self.next_attribute = self.next_attribute.wrapping_add(1);
        let frame = self.build_attribute_request(leaf);
        self.emit(frame, "attribute request");
        self.counters.attribute_requests_sent += 1;
        self.log(format!("reading attribute 0x{leaf:04X}"));
    }

    // ── Host-facing ─────────────────────────────────────────────────

    pub fn mpcp_state(&self) -> MpcpState {
        self.mpcp_state
    }

    pub fn onu_mac(&self) -> MacAddr {
        self.onu_mac
    }

    pub fn assigned_llid(&self) -> Llid {
        self.assigned_llid
    }

    /// Grant `llid` to the next ONU that registers.
    pub fn set_assigned_llid(&mut self, llid: Llid) {
        self.assigned_llid = llid;
        self.config.llid_start = llid;
    }

    pub fn discovery(&self) -> oam::Discovery {
        self.discovery
    }

    pub fn attribute_replies(&self) -> &VecDeque<extended::Container> {
        &self.attribute_replies
    }

    pub fn tx_log(&self) -> &VecDeque<LoggedFrame> {
        &self.tx_log
    }

    pub fn rx_log(&self) -> &VecDeque<LoggedFrame> {
        &self.rx_log
    }

    /// The LLID a REGISTER just granted, once.
    pub fn take_llid_assignment(&mut self) -> Option<u16> {
        self.llid_assignment.take()
    }

    /// True while a completed registration has not been acknowledged by the
    /// host. Taking it clears it.
    pub fn take_registration_complete(&mut self) -> bool {
        std::mem::take(&mut self.registration_complete)
    }

    pub fn registration_complete(&self) -> bool {
        self.registration_complete
    }

    /// Instant the current registration began at.
    pub fn registered_at(&self) -> Option<WireInstant> {
        self.registered_at
    }

    /// Send an extended OAMPDU carrying `payload` after the vendor opcode.
    /// Returns the frame as it was queued.
    pub fn send_extended(
        &mut self,
        oui: oam::Oui,
        opcode: extended::Opcode,
        payload: &[u8],
    ) -> Vec<u8> {
        let mut w = crate::types::FrameWriter::ethernet(
            self.oam_destination(),
            self.config.mac,
            EtherType::SlowProtocol,
        );
        w.u8(oam::SUBTYPE_OAM)
            .u16(self.discovery.flags().as_u16())
            .u8(oam::Code::OrganizationSpecific.as_u8())
            .bytes(&oui.0)
            .u8(opcode.as_u8())
            .bytes(payload);
        let frame = w.pad_to(crate::types::MIN_FRAME_LEN);
        self.emit(frame.clone(), "extended request");
        frame
    }

    /// Read one attribute now, rather than waiting for the rotation.
    pub fn request_attribute(&mut self, leaf: u16) -> Vec<u8> {
        let frame = self.build_attribute_request(leaf);
        self.emit(frame.clone(), "attribute request");
        self.counters.attribute_requests_sent += 1;
        frame
    }

    /// Put a frame of the host's choosing on the line.
    pub fn inject(&mut self, frame: Vec<u8>) {
        self.emit(frame, "injected");
    }

    /// Take the state machine back to the beginning without touching the
    /// link, the clock or the counters.
    ///
    /// A host that fed the peer synthesized traffic before the real end was
    /// alive uses this so the first real request meets the first real answer.
    pub fn rewind_handshake(&mut self) {
        self.mpcp_state = MpcpState::Idle;
        self.registration_complete = false;
        self.registered_at = None;
        // Discovery starts over, so the window count does too: leaving it
        // spent means the first real request is answered on a window the
        // synthesized traffic used up.
        self.register_requests_seen = 0;
        self.sched.cancel(Timer::RegisterResponse);
        self.sched.cancel(Timer::Lifetime);
    }

    // ── Downlink builders ───────────────────────────────────────────

    fn mpcp_header(&self, opcode: mpcp::Opcode, dst: MacAddr) -> mpcp::Header {
        mpcp::Header { dst, src: self.config.mac, opcode, timestamp: self.now.mpcp_timestamp() }
    }

    fn oam_destination(&self) -> MacAddr {
        if self.config.unicast_oam && !self.onu_mac.is_zero() {
            self.onu_mac
        } else {
            MacAddr::SLOW_PROTOCOL_MULTICAST
        }
    }

    fn build_register(&self, flag: RegisterFlag) -> Vec<u8> {
        let request = self.last_request.unwrap_or(mpcp::RegisterReqBody {
            flag: RegisterReqFlag::Register,
            pending_grants: 0,
            discovery_information: 0,
            laser_on: ECHOED_LASER_ON_TQ,
            laser_off: ECHOED_LASER_OFF_TQ,
        });
        mpcp::register(
            self.mpcp_header(mpcp::Opcode::Register, self.onu_mac),
            self.assigned_llid,
            flag,
            mpcp::RegisterBody {
                sync_time: SYNC_TIME_TQ,
                // Echoed, which is what "echoed" means: these are the values
                // the ONU declared, handed back so it can check they landed.
                echoed_pending_grants: request.pending_grants,
                echoed_laser_on: request.laser_on,
                echoed_laser_off: request.laser_off,
            },
        )
    }

    /// Discovery GATEs are broadcast: they are how an unregistered ONU is
    /// reached, before any address is known.
    fn build_gate(&self) -> Vec<u8> {
        mpcp::gate(
            self.mpcp_header(mpcp::Opcode::Gate, MacAddr::MPCP_MULTICAST),
            GateFlags { grant_count: GATE_GRANT_COUNT, discovery: true, force_report: 0 },
            Some(Grant {
                start_time: self.now.mpcp_timestamp().wrapping_add(DISCOVERY_GRANT_OFFSET_TQ),
                length: DISCOVERY_GRANT_LENGTH_TQ,
            }),
            Some(mpcp::DiscoveryWindow {
                sync_time: SYNC_TIME_TQ,
                information: self.config.discovery_information,
            }),
        )
    }

    /// A normal GATE: addressed to the ONU, no discovery flag, no trailer.
    ///
    /// ⛔ **No firmware-observable oracle can validate this frame.** The
    /// receiving firmware tests the discovery bit first and returns if it is
    /// clear — no log, no counter, no second branch — so the only thing that
    /// can be checked from outside is that sending it changes nothing else.
    /// What it buys is the grant window the upstream side needs.
    fn build_normal_gate(&self) -> Vec<u8> {
        mpcp::gate(
            self.mpcp_header(mpcp::Opcode::Gate, self.onu_mac),
            GateFlags { grant_count: 1, discovery: false, force_report: 1 },
            Some(Grant {
                start_time: self.now.mpcp_timestamp().wrapping_add(NORMAL_GRANT_OFFSET_TQ),
                length: NORMAL_GRANT_LENGTH_TQ,
            }),
            None,
        )
    }

    fn build_oam_info(&self) -> Vec<u8> {
        oam::information(
            self.oam_destination(),
            self.config.mac,
            self.discovery.flags(),
            oam::InfoTlv {
                is_local: true,
                oam_version: 0x01,
                revision: 0x0001,
                state: 0x00,
                configuration: 0x05,
                max_pdu_size: MAX_OAM_PDU_SIZE,
                oui: oam::Oui::DPOE,
                vendor_specific: [0; 4],
            },
        )
    }

    fn build_attribute_request(&self, leaf: u16) -> Vec<u8> {
        extended::get_request(
            extended::Header {
                dst: self.oam_destination(),
                src: self.config.mac,
                flags: self.discovery.flags(),
                oui: oam::Oui::DPOE,
                opcode: extended::Opcode::GetRequest,
            },
            &[extended::Descriptor::attribute(leaf)],
        )
    }

    // ── Bookkeeping ─────────────────────────────────────────────────

    fn emit(&mut self, frame: Vec<u8>, description: &str) {
        let entry = self.frame_entry(&frame, description);
        push_capped(&mut self.rx_log, entry);
        self.counters.frames_sent += 1;
        self.downstream.push_back(Emitted {
            at: self.now,
            frame,
            description: description.to_string(),
        });
    }

    fn frame_entry(&self, frame: &[u8], description: &str) -> LoggedFrame {
        LoggedFrame {
            data: frame[..frame.len().min(MAX_FRAME_SIZE)].to_vec(),
            at: self.now,
            description: description.to_string(),
        }
    }

    fn log(&mut self, line: String) {
        if self.log_lines.len() >= MAX_LOG_LINES {
            self.log_lines.pop_front();
        }
        self.log_lines.push_back((self.now, line));
    }
}

fn push_capped(log: &mut VecDeque<LoggedFrame>, entry: LoggedFrame) {
    if log.len() >= MAX_FRAME_LOG {
        log.pop_front();
    }
    log.push_back(entry);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mpcp::RegisterAckFlag;

    fn at(ms: u64) -> WireInstant {
        WireInstant::from_ps(WireDuration::from_ms(ms).as_ps())
    }

    /// A peer with the link up, wound forward to `ms`.
    fn live_peer() -> Peer {
        let mut peer = Peer::default();
        peer.set_link(true, at(0));
        peer
    }

    fn register_req(src: MacAddr) -> Vec<u8> {
        mpcp::register_req(
            mpcp::Header {
                dst: MacAddr::MPCP_MULTICAST,
                src,
                opcode: mpcp::Opcode::RegisterReq,
                timestamp: 0,
            },
            mpcp::RegisterReqBody {
                flag: mpcp::RegisterReqFlag::Register,
                pending_grants: PENDING_GRANTS,
                discovery_information: 0x0011,
                laser_on: 32,
                laser_off: 32,
            },
        )
    }

    /// A REPORT, header only: its body is not modelled anywhere, and a
    /// made-up one would be a guess standing in for a measurement.
    fn report_frame() -> Vec<u8> {
        mpcp::bare(mpcp::Header {
            dst: DEFAULT_OLT_MAC,
            src: ONU,
            opcode: mpcp::Opcode::Report,
            timestamp: 0,
        })
    }

    /// A GATE sent upstream — an opcode only the far end may originate.
    fn gate_from_the_onu() -> Vec<u8> {
        mpcp::gate(
            mpcp::Header {
                dst: DEFAULT_OLT_MAC,
                src: ONU,
                opcode: mpcp::Opcode::Gate,
                timestamp: 0,
            },
            GateFlags { grant_count: 0, discovery: false, force_report: 0 },
            None,
            None,
        )
    }

    /// An MPCP EtherType with nothing behind it.
    fn truncated_mpcpdu() -> Vec<u8> {
        crate::types::FrameWriter::ethernet(DEFAULT_OLT_MAC, ONU, EtherType::Mpcp).finish()
    }

    fn register_ack_with(flag: mpcp::RegisterAckFlag, port: Llid, sync: u16) -> Vec<u8> {
        mpcp::register_ack(
            mpcp::Header {
                dst: MacAddr::MPCP_MULTICAST,
                src: ONU,
                opcode: mpcp::Opcode::RegisterAck,
                timestamp: 0,
            },
            flag,
            port,
            sync,
        )
    }

    fn register_ack(src: MacAddr, llid: Llid) -> Vec<u8> {
        mpcp::register_ack(
            mpcp::Header {
                dst: MacAddr::MPCP_MULTICAST,
                src,
                opcode: mpcp::Opcode::RegisterAck,
                timestamp: 0,
            },
            mpcp::RegisterAckFlag::Ack,
            llid,
            SYNC_TIME_TQ,
        )
    }

    const ONU: MacAddr = MacAddr::new([0x02, 0x00, 0x00, 0x01, 0x02, 0x03]);
    /// Grants the test ONU declares outstanding, so the echo has something
    /// distinguishable to carry.
    const PENDING_GRANTS: u8 = 16;

    /// Drive a registration to completion and return the peer and the instant
    /// the ACK landed.
    fn register(peer: &mut Peer, mut t: u64) -> u64 {
        // The first request is passed over, the second is answered.
        for _ in 0..=IGNORED_REGISTER_REQUESTS {
            peer.deliver(&register_req(ONU), at(t));
            t += 10;
            peer.advance_to(at(t));
        }
        t += PROCESSING_DELAY.as_ms() + 1;
        peer.advance_to(at(t));
        peer.deliver(&register_ack(ONU, peer.assigned_llid()), at(t));
        t
    }

    #[test]
    fn the_link_coming_up_starts_the_periodic_traffic() {
        let mut peer = live_peer();
        peer.advance_to(at(0));
        assert_eq!(peer.counters.gates_sent, 1);
        assert_eq!(peer.counters.oam_keepalives_sent, 1);
        peer.advance_to(at(1000));
        assert_eq!(peer.counters.gates_sent, 2);
        assert_eq!(peer.counters.oam_keepalives_sent, 2);
    }

    #[test]
    fn advancing_in_one_step_or_many_gives_the_same_frames() {
        let mut coarse = live_peer();
        coarse.advance_to(at(10_000));

        let mut fine = live_peer();
        for ms in 0..=10_000 {
            fine.advance_to(at(ms));
        }
        assert_eq!(coarse.counters.gates_sent, fine.counters.gates_sent);
        assert_eq!(coarse.counters.oam_keepalives_sent, fine.counters.oam_keepalives_sent);
        // And the stamps agree, not just the counts: an event fires at the
        // instant it was due, not when the host got round to it.
        let stamps = |p: &Peer| p.rx_log().iter().map(|f| f.at).collect::<Vec<_>>();
        assert_eq!(stamps(&coarse), stamps(&fine));
    }

    #[test]
    fn a_gate_carries_the_window_the_receiver_checks() {
        let mut peer = live_peer();
        peer.advance_to(at(0));
        let gate = peer.take_downstream().into_iter().next().expect("a GATE went out");
        let pdu = mpcp::Pdu::parse(&gate.frame).expect("parses");
        assert_eq!(pdu.header.opcode, mpcp::Opcode::Gate);
        assert_eq!(pdu.header.dst, MacAddr::MPCP_MULTICAST);
        match pdu.body {
            mpcp::Body::Gate { flags, .. } => assert!(flags.discovery),
            other => panic!("unexpected body {other:?}"),
        }
    }

    #[test]
    fn the_first_discovery_window_is_passed_over_and_counted() {
        let mut peer = live_peer();
        peer.advance_to(at(0));
        peer.deliver(&register_req(ONU), at(1));
        assert_eq!(peer.counters.register_req_window_passed, 1);
        assert_eq!(peer.mpcp_state(), MpcpState::Idle);

        peer.deliver(&register_req(ONU), at(2));
        assert_eq!(peer.counters.register_req_accepted, 1);
        assert_eq!(peer.mpcp_state(), MpcpState::Discovery);
        assert!(peer.counters.register_requests_accounted_for());
    }

    #[test]
    fn every_refused_request_is_accounted_for() {
        let mut peer = live_peer();
        peer.advance_to(at(0));
        for ms in 1..=20 {
            peer.deliver(&register_req(ONU), at(ms));
            peer.advance_to(at(ms));
        }
        assert_eq!(peer.counters.register_req_seen, 20);
        assert!(peer.counters.register_req_in_flight > 0, "duplicates are refused");
        assert!(peer.counters.register_requests_accounted_for());
    }

    /// Every upstream MPCPDU lands in exactly one bucket. Without this,
    /// a REPORT could arrive, be logged, and leave no number behind —
    /// and "no report in N frames" would have no denominator.
    #[test]
    fn every_upstream_mpcpdu_is_accounted_for() {
        let mut peer = live_peer();
        peer.advance_to(at(0));

        for ms in 1..=6 {
            peer.deliver(&register_req(ONU), at(ms));
            peer.advance_to(at(ms));
        }
        // An opcode only the far end may originate, and a frame that is
        // not an MPCPDU at all.
        peer.deliver(&report_frame(), at(7));
        peer.deliver(&gate_from_the_onu(), at(8));
        peer.deliver(&truncated_mpcpdu(), at(9));

        let c = peer.counters;
        assert_eq!(c.reports_seen, 1, "a REPORT must leave a number behind");
        assert_eq!(c.mpcp_upstream_wrong_direction, 1);
        assert_eq!(c.mpcp_malformed, 1);
        assert!(c.upstream_mpcpdus_accounted_for(), "{c:?}");
        // The registration ledger stays consistent too: a frame that did
        // not parse is not a request that failed.
        assert!(c.register_requests_accounted_for(), "{c:?}");
    }

    /// The peer sends discovery GATEs and only those today. The counter
    /// reads the flag off the frame, so it stays right when that changes.
    #[test]
    fn no_gate_sent_so_far_is_a_non_discovery_one() {
        let mut peer = live_peer();
        peer.advance_to(at(3000));
        assert!(peer.counters.gates_sent > 0);
        assert_eq!(peer.counters.gates_normal_sent, 0);
    }

    #[test]
    fn a_registration_completes_and_grants_the_llid() {
        let mut peer = live_peer();
        peer.advance_to(at(0));
        register(&mut peer, 1);
        assert_eq!(peer.mpcp_state(), MpcpState::Registered);
        assert_eq!(peer.counters.registrations, 1);
        assert_eq!(peer.take_llid_assignment(), Some(DEFAULT_LLID.as_u16()));
        assert!(peer.take_registration_complete());
        assert!(!peer.take_registration_complete(), "taking it clears it");
    }

    #[test]
    fn the_registration_lifetime_is_a_round_minute_from_the_ack() {
        let mut peer = live_peer();
        peer.advance_to(at(0));
        let acked = register(&mut peer, 1);
        peer.take_downstream();

        peer.advance_to(at(acked + REGISTRATION_LIFETIME.as_ms() - 1));
        assert_eq!(peer.counters.deregistrations, 0, "not a millisecond early");

        peer.advance_to(at(acked + REGISTRATION_LIFETIME.as_ms()));
        assert_eq!(peer.counters.deregistrations, 1);
        let teardown = peer
            .take_downstream()
            .into_iter()
            .find(|e| e.description.contains("deregister"))
            .expect("a teardown went out");
        // The peer's own timer is exactly a minute: whatever the far end
        // observes on top of that came from the fibre, not from here.
        assert_eq!(teardown.at - at(acked), REGISTRATION_LIFETIME);
        assert_eq!(peer.mpcp_state(), MpcpState::Idle);
    }

    #[test]
    fn a_teardown_silences_the_peer_until_the_next_window() {
        let mut peer = live_peer();
        peer.advance_to(at(0));
        let acked = register(&mut peer, 1);
        let torn = acked + REGISTRATION_LIFETIME.as_ms();
        peer.advance_to(at(torn));

        peer.deliver(&register_req(ONU), at(torn + 1));
        assert_eq!(peer.counters.register_req_link_settling, 1);

        peer.advance_to(at(torn + REDISCOVERY_DELAY.as_ms()));
        peer.deliver(&register_req(ONU), at(torn + REDISCOVERY_DELAY.as_ms() + 1));
        // The count restarted with discovery, so this one is passed over
        // rather than refused.
        assert_eq!(peer.counters.register_req_window_passed, 2);
        assert!(peer.counters.register_requests_accounted_for());
    }

    /// What the peer hands back has to be what it was told, or the ONU has
    /// no way to know it was heard.
    #[test]
    fn the_grant_echoes_what_the_request_declared() {
        let mut peer = live_peer();
        peer.advance_to(at(0));
        for _ in 0..=IGNORED_REGISTER_REQUESTS {
            peer.deliver(&register_req(ONU), at(1));
        }
        peer.advance_to(at(PROCESSING_DELAY.as_ms() + 2));
        let register = peer
            .take_downstream()
            .into_iter()
            .find(|e| e.description == "REGISTER")
            .expect("a REGISTER went out");
        // Echoed pending grants sit past the flags and sync time.
        assert_eq!(register.frame[25], PENDING_GRANTS);
        assert_eq!(register.frame[26], 32);
        assert_eq!(register.frame[27], 32);
    }

    /// The same code point carries "let me in" and "let me out".
    #[test]
    fn a_request_to_deregister_is_not_a_registration() {
        let mut peer = live_peer();
        peer.advance_to(at(0));
        register(&mut peer, 1);
        assert_eq!(peer.mpcp_state(), MpcpState::Registered);

        let leaving = mpcp::register_req(
            mpcp::Header {
                dst: MacAddr::MPCP_MULTICAST,
                src: ONU,
                opcode: mpcp::Opcode::RegisterReq,
                timestamp: 0,
            },
            mpcp::RegisterReqBody {
                flag: RegisterReqFlag::Deregister,
                pending_grants: 0,
                discovery_information: 0x0011,
                laser_on: 32,
                laser_off: 32,
            },
        );
        peer.deliver(&leaving, at(100));
        assert_eq!(peer.counters.register_req_deregister, 1);
        assert_eq!(peer.counters.deregistrations, 1);
        assert_eq!(peer.mpcp_state(), MpcpState::Idle);
        assert!(peer.counters.register_requests_accounted_for());
    }

    /// An acknowledgement that echoes something else acknowledges something
    /// else. Before this, nothing in it was read at all.
    #[test]
    fn an_ack_that_echoes_the_wrong_grant_is_refused() {
        for wrong in [
            register_ack_with(RegisterAckFlag::Ack, Llid(0x1234), SYNC_TIME_TQ),
            register_ack_with(RegisterAckFlag::Ack, DEFAULT_LLID, SYNC_TIME_TQ + 1),
            register_ack_with(RegisterAckFlag::Nack, DEFAULT_LLID, SYNC_TIME_TQ),
        ] {
            let mut peer = live_peer();
            peer.advance_to(at(0));
            for _ in 0..=IGNORED_REGISTER_REQUESTS {
                peer.deliver(&register_req(ONU), at(1));
            }
            peer.advance_to(at(PROCESSING_DELAY.as_ms() + 2));
            assert_eq!(peer.mpcp_state(), MpcpState::WaitAck);

            peer.deliver(&wrong, at(PROCESSING_DELAY.as_ms() + 3));
            assert_eq!(peer.counters.register_ack_rejected, 1);
            assert_eq!(peer.counters.registrations, 0);
            assert_eq!(peer.mpcp_state(), MpcpState::WaitAck, "still waiting");
        }
    }

    #[test]
    fn an_unexpected_ack_is_counted_rather_than_acted_on() {
        let mut peer = live_peer();
        peer.advance_to(at(0));
        peer.deliver(&register_ack(ONU, DEFAULT_LLID), at(1));
        assert_eq!(peer.counters.register_ack_unexpected, 1);
        assert_eq!(peer.mpcp_state(), MpcpState::Idle);
        assert_eq!(peer.counters.registrations, 0);
    }

    #[test]
    fn the_link_going_down_stops_everything_and_forgets_discovery() {
        let mut peer = live_peer();
        peer.advance_to(at(0));
        register(&mut peer, 1);
        peer.set_link(false, at(100));
        assert_eq!(peer.mpcp_state(), MpcpState::Idle);
        assert_eq!(peer.next_due(), None);

        let gates = peer.counters.gates_sent;
        peer.advance_to(at(100_000));
        assert_eq!(peer.counters.gates_sent, gates, "a dead link sends nothing");
        assert_eq!(peer.counters.deregistrations, 0, "and tears nothing down");
    }

    #[test]
    fn attributes_are_only_read_once_discovery_has_converged() {
        let mut peer = live_peer();
        peer.advance_to(at(0));
        register(&mut peer, 1);
        peer.advance_to(at(30_000));
        assert_eq!(peer.counters.attribute_requests_sent, 0, "nothing has converged");

        // Six keepalives take the peer to stable; a stable peer answering
        // converges it.
        let stable = oam::information(
            peer.config.mac,
            ONU,
            oam::Flags::local_stable(),
            oam::InfoTlv {
                is_local: true,
                oam_version: 1,
                revision: 0,
                state: 0,
                configuration: 0x14,
                max_pdu_size: 0x0600,
                oui: oam::Oui::DPOE,
                vendor_specific: [0; 4],
            },
        );
        peer.deliver(&stable, at(30_001));
        assert!(peer.discovery().converged());
        peer.advance_to(at(30_002));
        assert_eq!(peer.counters.attribute_requests_sent, 1);
    }
}
