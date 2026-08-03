//! OAMPDU encoding and decoding (IEEE 802.3 clause 57).

use std::fmt;

use crate::types::{EtherType, FrameWriter, MacAddr, MIN_FRAME_LEN};

/// Offset of the slow-protocol subtype, right after the Ethernet header.
const SUBTYPE_OFFSET: usize = 14;
/// Offset of the flags field.
const FLAGS_OFFSET: usize = 15;
/// Offset of the OAMPDU code.
const CODE_OFFSET: usize = 17;
/// Offset of the first TLV.
const TLV_OFFSET: usize = 18;

/// Slow-protocol subtype carrying OAM.
pub const SUBTYPE_OAM: u8 = 0x03;

/// OAMPDU codes (clause 57.4.3.2).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Code {
    Information,
    EventNotification,
    VariableRequest,
    VariableResponse,
    LoopbackControl,
    OrganizationSpecific,
    Other(u8),
}

impl Code {
    pub const fn as_u8(self) -> u8 {
        match self {
            Self::Information => 0x00,
            Self::EventNotification => 0x01,
            Self::VariableRequest => 0x02,
            Self::VariableResponse => 0x03,
            Self::LoopbackControl => 0x04,
            Self::OrganizationSpecific => 0xFE,
            Self::Other(v) => v,
        }
    }
}

impl From<u8> for Code {
    fn from(v: u8) -> Self {
        match v {
            0x00 => Self::Information,
            0x01 => Self::EventNotification,
            0x02 => Self::VariableRequest,
            0x03 => Self::VariableResponse,
            0x04 => Self::LoopbackControl,
            0xFE => Self::OrganizationSpecific,
            other => Self::Other(other),
        }
    }
}

impl fmt::Display for Code {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Information => write!(f, "information"),
            Self::EventNotification => write!(f, "event"),
            Self::VariableRequest => write!(f, "var-request"),
            Self::VariableResponse => write!(f, "var-response"),
            Self::LoopbackControl => write!(f, "loopback"),
            Self::OrganizationSpecific => write!(f, "org-specific"),
            Self::Other(v) => write!(f, "code 0x{v:02X}"),
        }
    }
}

/// The OAMPDU flags field (clause 57.4.2.1). The discovery state machine
/// converges by exchanging the four evaluating/stable bits.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Flags {
    pub link_fault: bool,
    pub dying_gasp: bool,
    pub critical_event: bool,
    pub local_evaluating: bool,
    pub local_stable: bool,
    pub remote_evaluating: bool,
    pub remote_stable: bool,
}

impl Flags {
    const LINK_FAULT: u16 = 0x0001;
    const DYING_GASP: u16 = 0x0002;
    const CRITICAL_EVENT: u16 = 0x0004;
    const LOCAL_EVALUATING: u16 = 0x0008;
    const LOCAL_STABLE: u16 = 0x0010;
    const REMOTE_EVALUATING: u16 = 0x0020;
    const REMOTE_STABLE: u16 = 0x0040;

    /// Local information is valid and settled, the remote is not yet known.
    pub const fn local_stable() -> Self {
        Self {
            link_fault: false,
            dying_gasp: false,
            critical_event: false,
            local_evaluating: false,
            local_stable: true,
            remote_evaluating: false,
            remote_stable: false,
        }
    }

    /// Both ends have settled: the state a converged discovery reports.
    pub const fn converged() -> Self {
        Self {
            link_fault: false,
            dying_gasp: false,
            critical_event: false,
            local_evaluating: false,
            local_stable: true,
            remote_evaluating: false,
            remote_stable: true,
        }
    }

    pub fn as_u16(self) -> u16 {
        let mut v = 0;
        if self.link_fault { v |= Self::LINK_FAULT }
        if self.dying_gasp { v |= Self::DYING_GASP }
        if self.critical_event { v |= Self::CRITICAL_EVENT }
        if self.local_evaluating { v |= Self::LOCAL_EVALUATING }
        if self.local_stable { v |= Self::LOCAL_STABLE }
        if self.remote_evaluating { v |= Self::REMOTE_EVALUATING }
        if self.remote_stable { v |= Self::REMOTE_STABLE }
        v
    }

    pub fn from_u16(v: u16) -> Self {
        Self {
            link_fault: v & Self::LINK_FAULT != 0,
            dying_gasp: v & Self::DYING_GASP != 0,
            critical_event: v & Self::CRITICAL_EVENT != 0,
            local_evaluating: v & Self::LOCAL_EVALUATING != 0,
            local_stable: v & Self::LOCAL_STABLE != 0,
            remote_evaluating: v & Self::REMOTE_EVALUATING != 0,
            remote_stable: v & Self::REMOTE_STABLE != 0,
        }
    }
}

/// Discovery, from the local end's point of view (clause 57.3.2.1).
///
/// Extended OAMPDUs are gated on this converging: until both ends report
/// stable, the sublayer discards them in both directions.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum DiscoveryState {
    /// Advertising local information, not yet satisfied with the peer's.
    #[default]
    LocalEvaluating,
    /// Local information settled; the peer has not confirmed it yet.
    LocalStable,
    /// Both ends settled. Extended OAMPDUs may flow.
    Converged,
}

/// Drives [`DiscoveryState`] from the PDUs sent and received.
///
/// Two rules keep it honest, and both are load-bearing:
///
/// - **Stability is claimed against evidence.** The local end does not settle
///   until it has heard the remote describe itself. A state machine that
///   promotes itself on a count of PDUs it sent declares the link discovered
///   before anything has answered.
/// - **The remote bits are a copy, not a summary** (clause 57.4.2.1). What
///   goes out in bits 5 and 6 is what the peer said about *itself* in bits 3
///   and 4. Deriving them from the local state instead makes two ends that
///   both do it knock each other back to evaluation forever: each reads the
///   other's "I am evaluating" as a reason to restart, and neither ever gets
///   far enough to say anything else.
#[derive(Clone, Copy, Debug, Default)]
pub struct Discovery {
    state: DiscoveryState,
    /// Information PDUs sent in the current state.
    sent: u32,
    /// Flags last seen from the peer, once it has told us anything.
    peer: Option<Flags>,
    /// The peer has described itself in an Information PDU. Until then there
    /// is nothing to have evaluated.
    peer_described_itself: bool,
}

impl Discovery {
    /// Information PDUs advertised as evaluating before claiming stability.
    /// The local end must give the peer time to latch the advertisement.
    const EVALUATING_PDUS: u32 = 6;

    pub fn state(&self) -> DiscoveryState {
        self.state
    }

    /// True once extended OAMPDUs may be exchanged.
    pub fn converged(&self) -> bool {
        self.state == DiscoveryState::Converged
    }

    /// True once the peer has described itself at least once.
    pub fn peer_described_itself(&self) -> bool {
        self.peer_described_itself
    }

    /// Flags to advertise in the next Information PDU.
    pub fn flags(&self) -> Flags {
        let mut flags = match self.state {
            DiscoveryState::LocalEvaluating => Flags {
                local_evaluating: true,
                ..Flags::default()
            },
            DiscoveryState::LocalStable | DiscoveryState::Converged => Flags::local_stable(),
        };
        // Echo what the peer said about itself, rather than restating our own
        // state in two places.
        if let Some(peer) = self.peer {
            flags.remote_evaluating = peer.local_evaluating;
            flags.remote_stable = peer.local_stable;
        }
        flags
    }

    /// Record an Information PDU going out.
    ///
    /// Sending is what lets evaluation finish, but only once the peer has
    /// given this end something to evaluate.
    pub fn on_sent(&mut self) {
        self.sent = self.sent.saturating_add(1);
        self.settle_if_evaluated();
    }

    /// Leave evaluation once both halves of the condition hold, whichever
    /// arrived last: enough advertisements out, and something heard back.
    fn settle_if_evaluated(&mut self) {
        if self.state == DiscoveryState::LocalEvaluating
            && self.peer_described_itself
            && self.sent >= Self::EVALUATING_PDUS
        {
            self.enter(DiscoveryState::LocalStable);
        }
    }

    /// Record an Information PDU from the peer: it carries the peer's own
    /// description, which is what this end evaluates.
    pub fn on_peer_information(&mut self, flags: Flags) {
        self.peer_described_itself = true;
        self.on_peer_flags(flags);
    }

    /// Record the peer's flags, from any OAMPDU.
    pub fn on_peer_flags(&mut self, flags: Flags) {
        self.peer = Some(flags);
        // A fault is the one thing that takes this end back to the start:
        // there is no link left to have evaluated.
        if flags.link_fault {
            // Nothing survives: whatever was evaluated was evaluated about a
            // link that is no longer there.
            *self = Self::default();
            return;
        }
        self.settle_if_evaluated();
        match self.state {
            // The peer restarting does not unsettle this end's own
            // description of itself — it only means discovery is not
            // complete, which is what leaving Converged says.
            DiscoveryState::Converged if flags.local_evaluating => {
                self.enter(DiscoveryState::LocalStable)
            }
            DiscoveryState::LocalStable if flags.local_stable => {
                self.enter(DiscoveryState::Converged)
            }
            _ => {}
        }
    }

    pub fn peer_flags(&self) -> Option<Flags> {
        self.peer
    }

    fn enter(&mut self, state: DiscoveryState) {
        if self.state != state {
            self.state = state;
            self.sent = 0;
        }
    }

    pub fn reset(&mut self) {
        *self = Self::default();
    }
}

/// A 24-bit organizationally unique identifier.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Oui(pub [u8; 3]);

impl Oui {
    /// The OUI the extended-OAM channel is carried under.
    pub const DPOE: Self = Self([0x00, 0x10, 0x00]);
}

impl fmt::Display for Oui {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let [a, b, c] = self.0;
        write!(f, "{a:02X}-{b:02X}-{c:02X}")
    }
}

/// Local or remote information TLV (clause 57.5.2.1).
#[derive(Clone, Copy, Debug)]
pub struct InfoTlv {
    pub is_local: bool,
    pub oam_version: u8,
    pub revision: u16,
    /// Mux and parser action state.
    pub state: u8,
    /// Advertised OAM capabilities.
    pub configuration: u8,
    pub max_pdu_size: u16,
    pub oui: Oui,
    pub vendor_specific: [u8; 4],
}

impl InfoTlv {
    const LOCAL_TYPE: u8 = 0x01;
    const REMOTE_TYPE: u8 = 0x02;
    /// Type and length included, per clause 57.5.2.1.
    const LENGTH: u8 = 0x10;

    fn write(&self, w: &mut FrameWriter) {
        w.u8(if self.is_local { Self::LOCAL_TYPE } else { Self::REMOTE_TYPE })
            .u8(Self::LENGTH)
            .u8(self.oam_version)
            .u16(self.revision)
            .u8(self.state)
            .u8(self.configuration)
            .u16(self.max_pdu_size)
            .bytes(&self.oui.0)
            .bytes(&self.vendor_specific);
    }
}

/// A parsed OAMPDU header.
#[derive(Clone, Copy, Debug)]
pub struct Pdu {
    pub dst: MacAddr,
    pub src: MacAddr,
    pub subtype: u8,
    pub flags: Flags,
    pub code: Code,
}

impl Pdu {
    /// Parse an OAMPDU. Returns `None` unless the frame is a slow-protocol
    /// frame long enough to hold a header.
    pub fn parse(frame: &[u8]) -> Option<Self> {
        if EtherType::of_frame(frame)? != EtherType::SlowProtocol {
            return None;
        }
        Some(Self {
            dst: MacAddr::from_slice(frame)?,
            src: MacAddr::from_slice(&frame[6..])?,
            subtype: *frame.get(SUBTYPE_OFFSET)?,
            flags: Flags::from_u16(u16::from_be_bytes([
                *frame.get(FLAGS_OFFSET)?,
                *frame.get(FLAGS_OFFSET + 1)?,
            ])),
            code: Code::from(*frame.get(CODE_OFFSET)?),
        })
    }

    /// Bytes following the OAMPDU header, where TLVs begin.
    pub fn tlv_bytes(frame: &[u8]) -> &[u8] {
        frame.get(TLV_OFFSET..).unwrap_or(&[])
    }
}

/// Build an Information OAMPDU carrying one local-information TLV.
pub fn information(dst: MacAddr, src: MacAddr, flags: Flags, info: InfoTlv) -> Vec<u8> {
    let mut w = FrameWriter::ethernet(dst, src, EtherType::SlowProtocol);
    w.u8(SUBTYPE_OAM)
        .u16(flags.as_u16())
        .u8(Code::Information.as_u8());
    info.write(&mut w);
    // End-of-TLV marker.
    w.u8(0x00).u8(0x00);
    w.pad_to(MIN_FRAME_LEN)
}

#[cfg(test)]
mod tests {
    use super::*;

    const SRC: MacAddr = MacAddr::new([0x02, 0x00, 0x00, 0x00, 0x00, 0x01]);

    fn info_tlv() -> InfoTlv {
        InfoTlv {
            is_local: true,
            oam_version: 0x01,
            revision: 0x0001,
            state: 0x00,
            configuration: 0x05,
            max_pdu_size: 0x05DC,
            oui: Oui([0x00, 0x10, 0x00]),
            vendor_specific: [0; 4],
        }
    }

    #[test]
    fn flags_round_trip_bit_by_bit() {
        for bit in 0..7u16 {
            let raw = 1 << bit;
            assert_eq!(Flags::from_u16(raw).as_u16(), raw, "bit {bit}");
        }
    }

    #[test]
    fn converged_flags_carry_both_stable_bits() {
        let f = Flags::converged();
        assert!(f.local_stable && f.remote_stable);
        assert_eq!(f.as_u16(), 0x0050);
        assert_eq!(Flags::local_stable().as_u16(), 0x0010);
    }

    #[test]
    fn information_round_trips_through_parse() {
        let frame = information(
            MacAddr::SLOW_PROTOCOL_MULTICAST,
            SRC,
            Flags::local_stable(),
            info_tlv(),
        );
        assert_eq!(frame.len(), MIN_FRAME_LEN);
        let pdu = Pdu::parse(&frame).expect("parses");
        assert_eq!(pdu.subtype, SUBTYPE_OAM);
        assert_eq!(pdu.code, Code::Information);
        assert!(pdu.flags.local_stable);
        assert_eq!(pdu.src, SRC);
        // The TLV starts with its type and declared length.
        assert_eq!(Pdu::tlv_bytes(&frame)[..2], [0x01, InfoTlv::LENGTH]);
    }

    #[test]
    fn codes_round_trip() {
        for code in [
            Code::Information,
            Code::EventNotification,
            Code::VariableRequest,
            Code::VariableResponse,
            Code::LoopbackControl,
            Code::OrganizationSpecific,
        ] {
            assert_eq!(Code::from(code.as_u8()), code);
        }
    }

    /// Drive a discovery to convergence against a peer that describes itself
    /// as stable.
    fn converge() -> Discovery {
        let mut d = Discovery::default();
        d.on_peer_information(Flags::local_stable());
        for _ in 0..Discovery::EVALUATING_PDUS {
            d.on_sent();
        }
        d.on_peer_flags(Flags::local_stable());
        assert!(d.converged());
        d
    }

    #[test]
    fn discovery_advertises_evaluating_before_it_claims_stability() {
        let mut d = Discovery::default();
        assert_eq!(d.state(), DiscoveryState::LocalEvaluating);
        assert!(d.flags().local_evaluating);
        assert!(!d.flags().local_stable);

        d.on_peer_information(Flags { local_evaluating: true, ..Flags::default() });
        for _ in 0..Discovery::EVALUATING_PDUS - 1 {
            d.on_sent();
            assert_eq!(d.state(), DiscoveryState::LocalEvaluating);
        }
        d.on_sent();
        assert_eq!(d.state(), DiscoveryState::LocalStable);
    }

    #[test]
    fn stability_is_not_claimed_before_the_peer_has_said_anything() {
        let mut d = Discovery::default();
        for _ in 0..Discovery::EVALUATING_PDUS * 10 {
            d.on_sent();
        }
        assert_eq!(
            d.state(),
            DiscoveryState::LocalEvaluating,
            "sending cannot be evidence about the other end"
        );
        // Hearing the peer is the missing half. It arrives last here, so it
        // is what settles the evaluation — and since the peer announced
        // itself stable, the same PDU completes discovery.
        d.on_peer_information(Flags::local_stable());
        assert!(d.converged());
    }

    #[test]
    fn convergence_needs_both_ends_settled() {
        let mut d = Discovery::default();
        // A stable peer while still evaluating is not convergence.
        d.on_peer_information(Flags::local_stable());
        assert_eq!(d.state(), DiscoveryState::LocalEvaluating);
        let d = converge();
        assert_eq!(d.flags().as_u16(), 0x0050);
    }

    #[test]
    fn the_remote_bits_are_a_copy_of_what_the_peer_said_about_itself() {
        let mut d = Discovery::default();
        // Still evaluating, but the peer has announced itself stable: bit 6
        // must already carry that, or the peer never learns it was heard.
        d.on_peer_information(Flags::local_stable());
        assert_eq!(d.flags().as_u16(), 0x0048);

        // A peer that is evaluating shows up in bit 5, not in the local bits.
        let mut d = Discovery::default();
        d.on_peer_information(Flags { local_evaluating: true, ..Flags::default() });
        assert_eq!(d.flags().as_u16(), 0x0028);
    }

    #[test]
    fn a_peer_restarting_costs_convergence_but_not_the_local_evaluation() {
        let mut d = converge();
        d.on_peer_flags(Flags { local_evaluating: true, ..Flags::default() });
        assert_eq!(d.state(), DiscoveryState::LocalStable);
        assert!(!d.converged());
        // And it converges again as soon as the peer settles, without having
        // to re-run six PDUs' worth of evaluation.
        d.on_peer_flags(Flags::local_stable());
        assert!(d.converged());
    }

    #[test]
    fn a_fault_takes_everything_back_to_the_start() {
        let mut d = converge();
        d.on_peer_flags(Flags { link_fault: true, ..Flags::local_stable() });
        assert_eq!(d.state(), DiscoveryState::LocalEvaluating);
        assert!(!d.converged());
        assert!(!d.peer_described_itself(), "a faulted link described nothing");
        assert_eq!(d.flags().as_u16(), 0x0008);
    }

    /// Two ends of this state machine, facing each other, must converge.
    /// They did not when each read the other's "evaluating" as a reason to
    /// restart: both settled, both were knocked back, forever.
    #[test]
    fn two_of_these_facing_each_other_converge() {
        let mut a = Discovery::default();
        let mut b = Discovery::default();
        for _ in 0..64 {
            let (fa, fb) = (a.flags(), b.flags());
            a.on_sent();
            b.on_sent();
            a.on_peer_information(fb);
            b.on_peer_information(fa);
            if a.converged() && b.converged() {
                return;
            }
        }
        panic!("two ends never converged: {:?} / {:?}", a.state(), b.state());
    }

    #[test]
    fn the_stable_flags_clear_the_gate_extended_oam_is_checked_against() {
        // A receiver admits extended OAMPDUs only when the evaluating and
        // stable bits read exactly "stable".
        for flags in [Flags::local_stable(), Flags::converged()] {
            assert_eq!(flags.as_u16() & 0x18, 0x10);
        }
        assert_ne!(
            Flags { local_evaluating: true, ..Flags::default() }.as_u16() & 0x18,
            0x10
        );
    }

    #[test]
    fn parse_rejects_a_non_slow_protocol_frame() {
        let frame = FrameWriter::ethernet(MacAddr::ZERO, SRC, EtherType::Mpcp)
            .pad_to(MIN_FRAME_LEN);
        assert!(Pdu::parse(&frame).is_none());
    }
}
