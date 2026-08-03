//! MPCPDU encoding and decoding (IEEE 802.3 clause 64).

use std::fmt;

use crate::types::{EtherType, FrameWriter, Llid, MacAddr, MIN_FRAME_LEN};

/// Offset of the opcode field, right after the Ethernet header.
const OPCODE_OFFSET: usize = 14;
/// Offset of the timestamp field.
const TIMESTAMP_OFFSET: usize = 16;
/// Offset of the MPCPDU body, after opcode and timestamp.
const BODY_OFFSET: usize = 20;

/// MPCP opcodes. Only the discovery and gating set is modelled.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Opcode {
    Gate,
    Report,
    RegisterReq,
    Register,
    RegisterAck,
    Other(u16),
}

impl Opcode {
    pub const fn as_u16(self) -> u16 {
        match self {
            Self::Gate => 2,
            Self::Report => 3,
            Self::RegisterReq => 4,
            Self::Register => 5,
            Self::RegisterAck => 6,
            Self::Other(v) => v,
        }
    }

    /// True for opcodes only an OLT may originate.
    pub const fn is_downstream_only(self) -> bool {
        matches!(self, Self::Gate | Self::Register)
    }
}

impl From<u16> for Opcode {
    fn from(v: u16) -> Self {
        match v {
            2 => Self::Gate,
            3 => Self::Report,
            4 => Self::RegisterReq,
            5 => Self::Register,
            6 => Self::RegisterAck,
            other => Self::Other(other),
        }
    }
}

impl fmt::Display for Opcode {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Gate => write!(f, "GATE"),
            Self::Report => write!(f, "REPORT"),
            Self::RegisterReq => write!(f, "REGISTER_REQ"),
            Self::Register => write!(f, "REGISTER"),
            Self::RegisterAck => write!(f, "REGISTER_ACK"),
            Self::Other(v) => write!(f, "opcode {v}"),
        }
    }
}

/// Flags octet of a REGISTER MPCPDU (clause 64.3.6.3, Table 64-4).
///
/// There is no "register" value: [`Self::Ack`] is what grants an LLID. A
/// receiver treats 1 and 2 as a deregistration.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RegisterFlag {
    /// Tear down and re-run discovery for an already-registered ONU.
    Reregister,
    /// Deregister the ONU.
    Deregister,
    /// Grant the LLID: the only value that registers.
    Ack,
    /// Refuse the request.
    Nack,
}

impl RegisterFlag {
    pub const fn as_u8(self) -> u8 {
        match self {
            Self::Reregister => 1,
            Self::Deregister => 2,
            Self::Ack => 3,
            Self::Nack => 4,
        }
    }

    pub const fn from_u8(v: u8) -> Option<Self> {
        match v {
            1 => Some(Self::Reregister),
            2 => Some(Self::Deregister),
            3 => Some(Self::Ack),
            4 => Some(Self::Nack),
            _ => None,
        }
    }

    /// True for the value that grants an LLID.
    pub const fn registers(self) -> bool {
        matches!(self, Self::Ack)
    }
}

/// Flags octet of a REGISTER_REQ MPCPDU (clause 64.3.6.2, Table 64-3).
///
/// A different table from the REGISTER flags, on a different code point: 3
/// here is a request to be torn down, while 3 in a REGISTER is what grants the
/// LLID. Sharing one enum between the two turns that into a silent inversion.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RegisterReqFlag {
    /// Asking to be admitted.
    Register,
    /// Asking to be torn down.
    Deregister,
    Other(u8),
}

impl RegisterReqFlag {
    pub const fn as_u8(self) -> u8 {
        match self {
            Self::Register => 1,
            Self::Deregister => 3,
            Self::Other(v) => v,
        }
    }

    pub const fn from_u8(v: u8) -> Self {
        match v {
            1 => Self::Register,
            3 => Self::Deregister,
            other => Self::Other(other),
        }
    }
}

/// Flags octet of a REGISTER_ACK MPCPDU (clause 64.3.6.4, Table 64-5).
///
/// Its own table again, and its own numbering: 1 is the acknowledgement here.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RegisterAckFlag {
    /// The ONU refuses what was granted.
    Nack,
    /// The ONU accepts what was granted.
    Ack,
    Other(u8),
}

impl RegisterAckFlag {
    pub const fn as_u8(self) -> u8 {
        match self {
            Self::Nack => 0,
            Self::Ack => 1,
            Self::Other(v) => v,
        }
    }

    pub const fn from_u8(v: u8) -> Self {
        match v {
            0 => Self::Nack,
            1 => Self::Ack,
            other => Self::Other(other),
        }
    }

    pub const fn acknowledges(self) -> bool {
        matches!(self, Self::Ack)
    }
}

/// "Number of grants / flags" octet of a GATE MPCPDU (clause 64.3.5.1):
/// bits [2:0] grant count, bit 3 discovery, bits [7:4] force-report.
#[derive(Clone, Copy, Debug, Default)]
pub struct GateFlags {
    pub grant_count: u8,
    pub discovery: bool,
    pub force_report: u8,
}

impl GateFlags {
    const GRANT_COUNT_MASK: u8 = 0x07;
    const DISCOVERY: u8 = 0x08;
    const FORCE_REPORT_SHIFT: u8 = 4;

    pub fn as_u8(self) -> u8 {
        (self.grant_count & Self::GRANT_COUNT_MASK)
            | if self.discovery { Self::DISCOVERY } else { 0 }
            | (self.force_report << Self::FORCE_REPORT_SHIFT)
    }

    pub fn from_u8(v: u8) -> Self {
        Self {
            grant_count: v & Self::GRANT_COUNT_MASK,
            discovery: v & Self::DISCOVERY != 0,
            force_report: v >> Self::FORCE_REPORT_SHIFT,
        }
    }
}

/// A single upstream transmission grant.
#[derive(Clone, Copy, Debug)]
pub struct Grant {
    pub start_time: u32,
    pub length: u16,
}

/// The fields shared by every MPCPDU.
#[derive(Clone, Copy, Debug)]
pub struct Header {
    pub dst: MacAddr,
    pub src: MacAddr,
    pub opcode: Opcode,
    pub timestamp: u32,
}

impl Header {
    fn writer(&self) -> FrameWriter {
        let mut w = FrameWriter::ethernet(self.dst, self.src, EtherType::Mpcp);
        w.u16(self.opcode.as_u16()).u32(self.timestamp);
        w
    }
}

/// A parsed MPCPDU: the header plus whatever the opcode carries.
#[derive(Clone, Copy, Debug)]
pub struct Pdu {
    pub header: Header,
    pub body: Body,
}

/// Opcode-specific contents.
#[derive(Clone, Copy, Debug)]
pub enum Body {
    Gate { flags: GateFlags, grant: Option<Grant> },
    Register { llid: Llid, flag: u8, sync_time: u16 },
    /// What an ONU asks for, and what it echoes back of the window it was
    /// offered. Reading these is the only way to refuse anything: a receiver
    /// that parses nothing accepts everything.
    RegisterReq(RegisterReqBody),
    /// Clause 64.3.6.4: flags, then the port and sync time being echoed. The
    /// LLID is *not* at the front — putting it there reads the flags octet as
    /// the low half of an LLID, which agrees with the truth only while the
    /// LLID is small enough for its high byte to be zero.
    RegisterAck { flag: RegisterAckFlag, echoed_port: Llid, echoed_sync_time: u16 },
    /// Carries no field the model reads.
    Empty,
}

/// Body of a REGISTER_REQ MPCPDU.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RegisterReqBody {
    pub flag: RegisterReqFlag,
    /// How many grants the ONU already has outstanding.
    pub pending_grants: u8,
    /// Which discovery windows the ONU can use, echoed from the GATE's own
    /// field ANDed with what it supports.
    pub discovery_information: u16,
    pub laser_on: u8,
    pub laser_off: u8,
}

impl Pdu {
    /// Parse an MPCPDU. Returns `None` unless the frame is an MPCP frame
    /// long enough to hold a header.
    pub fn parse(frame: &[u8]) -> Option<Self> {
        if EtherType::of_frame(frame)? != EtherType::Mpcp {
            return None;
        }
        let opcode = Opcode::from(u16::from_be_bytes([
            *frame.get(OPCODE_OFFSET)?,
            *frame.get(OPCODE_OFFSET + 1)?,
        ]));
        let timestamp = u32::from_be_bytes([
            *frame.get(TIMESTAMP_OFFSET)?,
            *frame.get(TIMESTAMP_OFFSET + 1)?,
            *frame.get(TIMESTAMP_OFFSET + 2)?,
            *frame.get(TIMESTAMP_OFFSET + 3)?,
        ]);
        let header = Header {
            dst: MacAddr::from_slice(frame)?,
            src: MacAddr::from_slice(&frame[6..])?,
            opcode,
            timestamp,
        };
        let body = frame.get(BODY_OFFSET..).unwrap_or(&[]);
        let at = |i: usize| body.get(i).copied().unwrap_or(0);
        let u16_at = |i: usize| u16::from_be_bytes([at(i), at(i + 1)]);
        let body = match opcode {
            Opcode::Gate => {
                let flags = GateFlags::from_u8(at(0));
                // A grant is only there when the flags say one is.
                let grant = (flags.grant_count > 0 && body.len() >= 7).then(|| Grant {
                    start_time: u32::from_be_bytes([at(1), at(2), at(3), at(4)]),
                    length: u16_at(5),
                });
                Body::Gate { flags, grant }
            }
            Opcode::Register => Body::Register {
                llid: Llid(u16_at(0)),
                flag: at(2),
                sync_time: u16_at(3),
            },
            Opcode::RegisterAck => Body::RegisterAck {
                flag: RegisterAckFlag::from_u8(at(0)),
                echoed_port: Llid(u16_at(1)),
                echoed_sync_time: u16_at(3),
            },
            Opcode::RegisterReq => Body::RegisterReq(RegisterReqBody {
                flag: RegisterReqFlag::from_u8(at(0)),
                pending_grants: at(1),
                discovery_information: u16_at(2),
                laser_on: at(4),
                laser_off: at(5),
            }),
            _ => Body::Empty,
        };
        Some(Self { header, body })
    }
}

/// Body of a REGISTER MPCPDU, past the assigned port and flags.
#[derive(Clone, Copy, Debug)]
pub struct RegisterBody {
    pub sync_time: u16,
    pub echoed_pending_grants: u8,
    pub echoed_laser_on: u8,
    pub echoed_laser_off: u8,
}

/// Build a REGISTER MPCPDU assigning `llid`.
pub fn register(header: Header, llid: Llid, flag: RegisterFlag, body: RegisterBody) -> Vec<u8> {
    let mut w = header.writer();
    w.u16(llid.as_u16())
        .u8(flag.as_u8())
        .u16(body.sync_time)
        .u8(body.echoed_pending_grants)
        .u8(body.echoed_laser_on)
        .u8(body.echoed_laser_off);
    w.pad_to(MIN_FRAME_LEN)
}

/// Build a REGISTER_ACK MPCPDU: the flags first, then what is being echoed.
pub fn register_ack(
    header: Header,
    flag: RegisterAckFlag,
    echoed_port: Llid,
    echoed_sync_time: u16,
) -> Vec<u8> {
    let mut w = header.writer();
    w.u8(flag.as_u8()).u16(echoed_port.as_u16()).u16(echoed_sync_time);
    w.pad_to(MIN_FRAME_LEN)
}

/// Trailer of a discovery GATE: the window parameters the ONU echoes back in
/// its REGISTER_REQ, and the field it gates the whole frame on.
#[derive(Clone, Copy, Debug)]
pub struct DiscoveryWindow {
    pub sync_time: u16,
    /// The receiver ANDs this against its accepted-window mask and drops the
    /// GATE when nothing remains, before logging anything.
    pub information: u16,
}

/// Build a GATE MPCPDU carrying at most one grant.
pub fn gate(
    header: Header,
    flags: GateFlags,
    grant: Option<Grant>,
    discovery: Option<DiscoveryWindow>,
) -> Vec<u8> {
    let mut w = header.writer();
    w.u8(flags.as_u8());
    if let Some(g) = grant {
        w.u32(g.start_time).u16(g.length);
    }
    if let Some(d) = discovery {
        w.u16(d.sync_time).u16(d.information);
    }
    w.pad_to(MIN_FRAME_LEN)
}

/// Build an MPCPDU carrying nothing but its header, padded to the
/// minimum frame length.
///
/// For opcodes whose body this model does not represent. The REPORT is
/// one: its layout is not established, and inventing one would put a
/// guess on the wire where a measurement belongs.
pub fn bare(header: Header) -> Vec<u8> {
    header.writer().pad_to(MIN_FRAME_LEN)
}

/// Build a REGISTER_REQ MPCPDU. Its flags octet is the register/deregister
/// request code, not the REGISTER response code.
/// Build a REPORT carrying one queue length.
///
/// ⛔ The rest of the body stays absent for the reason above. This one
/// field is here because it is the only one that can be **checked**: the
/// length is a number the firmware computed from the registration
/// exchange and wrote into the MAC's template, so a report carrying it
/// can be told apart from a report the model made up. The queue-set count
/// and bitmap are the single-set case, INFERRED from clause 64 with no
/// witness here.
pub fn report_with_length(header: Header, queue_length: u16) -> Vec<u8> {
    let mut w = header.writer();
    w.u8(1).u8(1).u16(queue_length);
    w.pad_to(MIN_FRAME_LEN)
}

pub fn register_req(header: Header, body: RegisterReqBody) -> Vec<u8> {
    let mut w = header.writer();
    w.u8(body.flag.as_u8())
        .u8(body.pending_grants)
        .u16(body.discovery_information)
        .u8(body.laser_on)
        .u8(body.laser_off);
    w.pad_to(MIN_FRAME_LEN)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn header(opcode: Opcode) -> Header {
        Header {
            dst: MacAddr::MPCP_MULTICAST,
            src: MacAddr::new([0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF]),
            opcode,
            timestamp: 0x1234_5678,
        }
    }

    #[test]
    fn opcodes_round_trip_and_five_is_not_report() {
        for op in [
            Opcode::Gate,
            Opcode::Report,
            Opcode::RegisterReq,
            Opcode::Register,
            Opcode::RegisterAck,
        ] {
            assert_eq!(Opcode::from(op.as_u16()), op);
        }
        assert_eq!(Opcode::from(3), Opcode::Report);
        assert_eq!(Opcode::from(5), Opcode::Register);
        assert!(Opcode::Register.is_downstream_only());
        assert!(!Opcode::Report.is_downstream_only());
    }

    #[test]
    fn gate_flags_round_trip() {
        let flags = GateFlags { grant_count: 1, discovery: true, force_report: 0 };
        assert_eq!(GateFlags::from_u8(flags.as_u8()).grant_count, 1);
        assert!(GateFlags::from_u8(flags.as_u8()).discovery);
        assert!(!GateFlags::from_u8(0x01).discovery);
    }

    #[test]
    fn register_round_trips_through_parse() {
        let body = RegisterBody {
            sync_time: 32,
            echoed_pending_grants: 0,
            echoed_laser_on: 32,
            echoed_laser_off: 32,
        };
        let frame = register(header(Opcode::Register), Llid(7), RegisterFlag::Ack, body);
        assert_eq!(frame.len(), MIN_FRAME_LEN);
        let pdu = Pdu::parse(&frame).expect("parses");
        assert_eq!(pdu.header.opcode, Opcode::Register);
        assert_eq!(pdu.header.timestamp, 0x1234_5678);
        match pdu.body {
            Body::Register { llid, flag, sync_time } => {
                assert_eq!(llid, Llid(7));
                assert_eq!(flag, RegisterFlag::Ack.as_u8());
                assert_eq!(sync_time, 32);
            }
            other => panic!("unexpected body {other:?}"),
        }
        // Laser on and off sit past the echoed pending grants, at MPCPDU 12
        // and 13. A receiver echoes them back verbatim.
        assert_eq!(frame[BODY_OFFSET + 6], 32);
        assert_eq!(frame[BODY_OFFSET + 7], 32);
        match Pdu::parse(&frame).expect("parses").body {
            Body::Register { .. } => {}
            other => panic!("unexpected body {other:?}"),
        }
    }

    #[test]
    fn gate_round_trips_with_its_discovery_bit() {
        let flags = GateFlags { grant_count: 1, discovery: true, force_report: 0 };
        let frame = gate(
            header(Opcode::Gate),
            flags,
            Some(Grant { start_time: 9, length: 8244 }),
            Some(DiscoveryWindow { sync_time: 32, information: 0x0030 }),
        );
        // The window parameters follow the grant: sync time at MPCPDU 13..14,
        // discovery information at 15..16. Putting anything else at 13 lands
        // in the high half of the sync time.
        assert_eq!(&frame[BODY_OFFSET + 7..BODY_OFFSET + 9], &32u16.to_be_bytes());
        assert_eq!(&frame[BODY_OFFSET + 9..BODY_OFFSET + 11], &0x0030u16.to_be_bytes());
        match Pdu::parse(&frame).expect("parses").body {
            Body::Gate { flags, .. } => {
                assert!(flags.discovery);
                assert_eq!(flags.grant_count, 1);
            }
            other => panic!("unexpected body {other:?}"),
        }
    }

    /// The acknowledgement's layout, against the octets an ONU actually
    /// sends: flags, then the echoed port, then the echoed sync time. Reading
    /// an LLID off the front instead happens to agree while the LLID is one
    /// byte wide, and stops agreeing the moment it is not.
    #[test]
    fn register_ack_puts_the_flags_first_not_the_llid() {
        let frame = register_ack(
            header(Opcode::RegisterAck),
            RegisterAckFlag::Ack,
            Llid(0x3C67),
            32,
        );
        assert_eq!(&frame[BODY_OFFSET..BODY_OFFSET + 5], &[0x01, 0x3C, 0x67, 0x00, 0x20]);
        match Pdu::parse(&frame).expect("parses").body {
            Body::RegisterAck { flag, echoed_port, echoed_sync_time } => {
                assert_eq!(flag, RegisterAckFlag::Ack);
                assert_eq!(echoed_port, Llid(0x3C67));
                assert_eq!(echoed_sync_time, 32);
            }
            other => panic!("unexpected body {other:?}"),
        }
    }

    /// Three tables, three numberings, on three code points. 3 asks to be torn
    /// down in a request and grants the LLID in a response; 1 acknowledges in
    /// an acknowledgement and tears down in a response.
    #[test]
    fn the_three_flag_tables_do_not_share_a_numbering() {
        assert_eq!(RegisterReqFlag::Deregister.as_u8(), 3);
        assert_eq!(RegisterFlag::Ack.as_u8(), 3);
        assert_eq!(RegisterAckFlag::Ack.as_u8(), 1);
        assert_eq!(RegisterFlag::Reregister.as_u8(), 1);
    }

    #[test]
    fn register_req_round_trips_its_whole_body() {
        let body = RegisterReqBody {
            flag: RegisterReqFlag::Register,
            pending_grants: 16,
            discovery_information: 0x0011,
            laser_on: 32,
            laser_off: 32,
        };
        let frame = register_req(header(Opcode::RegisterReq), body);
        assert_eq!(
            &frame[BODY_OFFSET..BODY_OFFSET + 6],
            &[0x01, 0x10, 0x00, 0x11, 0x20, 0x20]
        );
        match Pdu::parse(&frame).expect("parses").body {
            Body::RegisterReq(parsed) => assert_eq!(parsed, body),
            other => panic!("unexpected body {other:?}"),
        }
    }

    #[test]
    fn a_request_to_deregister_is_not_a_request_to_register() {
        let frame = register_req(
            header(Opcode::RegisterReq),
            RegisterReqBody {
                flag: RegisterReqFlag::Deregister,
                pending_grants: 0,
                discovery_information: 0,
                laser_on: 0,
                laser_off: 0,
            },
        );
        match Pdu::parse(&frame).expect("parses").body {
            Body::RegisterReq(body) => assert_eq!(body.flag, RegisterReqFlag::Deregister),
            other => panic!("unexpected body {other:?}"),
        }
    }

    #[test]
    fn a_gate_grant_is_parsed_back_out() {
        let frame = gate(
            header(Opcode::Gate),
            GateFlags { grant_count: 1, discovery: true, force_report: 0 },
            Some(Grant { start_time: 0x026A_D4B1, length: 8244 }),
            Some(DiscoveryWindow { sync_time: 32, information: 0x0011 }),
        );
        match Pdu::parse(&frame).expect("parses").body {
            Body::Gate { grant: Some(g), .. } => {
                assert_eq!(g.start_time, 0x026A_D4B1);
                assert_eq!(g.length, 8244);
            }
            other => panic!("unexpected body {other:?}"),
        }
        // No grants announced means no grant to read, whatever follows.
        let empty = gate(
            header(Opcode::Gate),
            GateFlags { grant_count: 0, discovery: true, force_report: 0 },
            None,
            None,
        );
        match Pdu::parse(&empty).expect("parses").body {
            Body::Gate { grant, .. } => assert!(grant.is_none()),
            other => panic!("unexpected body {other:?}"),
        }
    }

    #[test]
    fn parse_rejects_a_non_mpcp_frame() {
        let frame = FrameWriter::ethernet(
            MacAddr::ZERO,
            MacAddr::ZERO,
            EtherType::SlowProtocol,
        )
        .pad_to(MIN_FRAME_LEN);
        assert!(Pdu::parse(&frame).is_none());
    }

    #[test]
    fn parse_rejects_a_truncated_frame() {
        let frame = register(
            header(Opcode::Register),
            Llid(1),
            RegisterFlag::Ack,
            RegisterBody {
                sync_time: 0,
                echoed_pending_grants: 0,
                echoed_laser_on: 0,
                echoed_laser_off: 0,
            },
        );
        assert!(Pdu::parse(&frame[..OPCODE_OFFSET + 1]).is_none());
    }
}
