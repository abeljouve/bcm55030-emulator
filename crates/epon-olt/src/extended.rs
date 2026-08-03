//! Extended OAM: organization-specific OAMPDUs and their variable encoding.
//!
//! An extended OAMPDU is an Information-class frame with code
//! [`Code::OrganizationSpecific`], an OUI, a vendor opcode, and a list of
//! variables. Variables are addressed by a branch and a leaf; a request
//! carries the address alone, a response carries a length and a value.

use std::fmt;

use crate::oam::{self, Code, Flags, Oui, SUBTYPE_OAM};
use crate::types::{EtherType, FrameWriter, MacAddr, MIN_FRAME_LEN};

/// Offset of the OUI, right after the OAMPDU code.
const OUI_OFFSET: usize = 18;
/// Offset of the vendor opcode.
const OPCODE_OFFSET: usize = 21;
/// Offset of the first variable.
const VARIABLES_OFFSET: usize = 22;

/// Vendor opcodes carried under the OUI.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Opcode {
    GetRequest,
    GetResponse,
    SetRequest,
    SetResponse,
    /// Link events the peer collects.
    LinkEvents,
    /// Multipart transfers, including firmware download.
    Multipart,
    Other(u8),
}

impl Opcode {
    /// How the variable list that follows is laid out.
    ///
    /// A request names what it wants; a response carries what was found. The
    /// two are not distinguishable from the bytes alone, so reading a list
    /// without knowing the opcode means reading a request's next address as
    /// the previous one's length — which yields a plausible container made of
    /// the addresses that came after it.
    pub const fn list_shape(self) -> ListShape {
        match self {
            Self::GetRequest => ListShape::Addresses,
            _ => ListShape::Containers,
        }
    }

    pub const fn as_u8(self) -> u8 {
        match self {
            Self::GetRequest => 0x01,
            Self::GetResponse => 0x02,
            Self::SetRequest => 0x03,
            Self::SetResponse => 0x04,
            Self::LinkEvents => 0x08,
            Self::Multipart => 0x09,
            Self::Other(v) => v,
        }
    }
}

impl From<u8> for Opcode {
    fn from(v: u8) -> Self {
        match v {
            0x01 => Self::GetRequest,
            0x02 => Self::GetResponse,
            0x03 => Self::SetRequest,
            0x04 => Self::SetResponse,
            0x08 => Self::LinkEvents,
            0x09 => Self::Multipart,
            other => Self::Other(other),
        }
    }
}

impl fmt::Display for Opcode {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::GetRequest => write!(f, "get-request"),
            Self::GetResponse => write!(f, "get-response"),
            Self::SetRequest => write!(f, "set-request"),
            Self::SetResponse => write!(f, "set-response"),
            Self::LinkEvents => write!(f, "link-events"),
            Self::Multipart => write!(f, "multipart"),
            Self::Other(v) => write!(f, "opcode 0x{v:02X}"),
        }
    }
}

/// What the entries of a variable list are.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ListShape {
    /// Addresses alone, plus whatever parameters an address takes.
    Addresses,
    /// Addresses with a length and a value.
    Containers,
}

/// The first octet of a variable address.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Branch {
    /// Ends a variable list. Its leaf and length are zero too.
    Terminator,
    /// Selects the object subsequent variables apply to.
    ObjectContext,
    /// Device attributes.
    Attribute,
    /// Statistics.
    Counter,
    /// Operations rather than values.
    Action,
    Other(u8),
}

impl Branch {
    pub const fn as_u8(self) -> u8 {
        match self {
            Self::Terminator => 0x00,
            Self::ObjectContext => 0xD6,
            Self::Attribute => 0xD7,
            Self::Counter => 0xD8,
            Self::Action => 0xD9,
            Self::Other(v) => v,
        }
    }
}

impl From<u8> for Branch {
    fn from(v: u8) -> Self {
        match v {
            0x00 => Self::Terminator,
            0xD6 => Self::ObjectContext,
            0xD7 => Self::Attribute,
            0xD8 => Self::Counter,
            0xD9 => Self::Action,
            other => Self::Other(other),
        }
    }
}

impl fmt::Display for Branch {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Terminator => write!(f, "end"),
            Self::ObjectContext => write!(f, "context"),
            Self::Attribute => write!(f, "attribute"),
            Self::Counter => write!(f, "counter"),
            Self::Action => write!(f, "action"),
            Self::Other(v) => write!(f, "branch 0x{v:02X}"),
        }
    }
}

/// Attribute leaves seen on this link.
pub mod leaf {
    pub const DEVICE_ID: u16 = 0x0002;
    pub const FIRMWARE_INFO: u16 = 0x0003;
    pub const MANUFACTURER_INFO: u16 = 0x0006;
    pub const MANUFACTURER_ORG_NAME: u16 = 0x000E;
}

/// The length octet of a variable, which is not a plain count.
///
/// Zero means a full-size value, and anything with the top bit set is a
/// status code carrying no data at all. Emitting a plain zero for "empty"
/// therefore announces 128 bytes and walks a reader off the end of the list.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Length {
    /// A value occupying the full width, encoded as zero.
    Full,
    /// A value of 1 to 127 bytes.
    Bytes(u8),
    /// A status code. No value follows.
    Status(u8),
}

impl Length {
    /// Width a full-size value occupies.
    pub const FULL_WIDTH: usize = 128;

    pub const fn as_u8(self) -> u8 {
        match self {
            Self::Full => 0,
            Self::Bytes(n) => n,
            Self::Status(s) => s,
        }
    }

    pub const fn from_u8(v: u8) -> Self {
        match v {
            0 => Self::Full,
            1..=0x7F => Self::Bytes(v),
            _ => Self::Status(v),
        }
    }

    /// Bytes of value that follow this length octet.
    pub const fn data_len(self) -> usize {
        match self {
            Self::Full => Self::FULL_WIDTH,
            Self::Bytes(n) => n as usize,
            Self::Status(_) => 0,
        }
    }

    /// Encode a value of `n` bytes, refusing the encodings that would be
    /// read as something else.
    pub fn of_value(n: usize) -> Option<Self> {
        match n {
            0 => None,
            Self::FULL_WIDTH => Some(Self::Full),
            1..=0x7F => Some(Self::Bytes(n as u8)),
            _ => None,
        }
    }
}

/// A variable address: what to read, and on which object.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Descriptor {
    pub branch: Branch,
    pub leaf: u16,
}

impl Descriptor {
    pub const fn attribute(leaf: u16) -> Self {
        Self { branch: Branch::Attribute, leaf }
    }

    fn write(&self, w: &mut FrameWriter) {
        w.u8(self.branch.as_u8()).u16(self.leaf);
    }
}

impl fmt::Display for Descriptor {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}/0x{:04X}", self.branch, self.leaf)
    }
}

/// A variable address together with its length and value.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Container {
    pub descriptor: Descriptor,
    pub length: Length,
    pub value: Vec<u8>,
}

impl Container {
    /// True when the peer answered with a status instead of a value.
    pub fn is_status(&self) -> bool {
        matches!(self.length, Length::Status(_))
    }
}

/// A parsed extended OAMPDU.
#[derive(Clone, Debug)]
pub struct Pdu {
    pub oui: Oui,
    pub opcode: Opcode,
    pub variables: Vec<Variable>,
}

impl Pdu {
    /// Parse an extended OAMPDU. Returns `None` unless the frame is a
    /// slow-protocol frame carrying an organization-specific OAMPDU.
    pub fn parse(frame: &[u8]) -> Option<Self> {
        let header = oam::Pdu::parse(frame)?;
        if header.subtype != SUBTYPE_OAM || header.code != Code::OrganizationSpecific {
            return None;
        }
        let oui = Oui([
            *frame.get(OUI_OFFSET)?,
            *frame.get(OUI_OFFSET + 1)?,
            *frame.get(OUI_OFFSET + 2)?,
        ]);
        let opcode = Opcode::from(*frame.get(OPCODE_OFFSET)?);
        Some(Self {
            oui,
            opcode,
            variables: parse_variables(
                frame.get(VARIABLES_OFFSET..).unwrap_or(&[]),
                opcode.list_shape(),
            ),
        })
    }

    /// The entries that carry a value. A request has none.
    pub fn containers(&self) -> impl Iterator<Item = &Container> {
        self.variables.iter().filter_map(Variable::as_container)
    }
}

/// One entry of a variable list.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Variable {
    /// An address a request is asking about, with any parameters it takes.
    Address { descriptor: Descriptor, parameters: Vec<u8> },
    /// An address with the length and value a response carries for it.
    Container(Container),
}

impl Variable {
    pub fn descriptor(&self) -> Descriptor {
        match self {
            Self::Address { descriptor, .. } => *descriptor,
            Self::Container(c) => c.descriptor,
        }
    }

    pub fn as_container(&self) -> Option<&Container> {
        match self {
            Self::Container(c) => Some(c),
            Self::Address { .. } => None,
        }
    }
}

/// Parameter bytes an address carries inside a request, past its three-byte
/// address.
///
/// INFERRED from what the receiving end steps over: the statistics leaves take
/// a three-byte parameter, and an object-context selector is written out in
/// full even in a request.
fn request_parameter_len(descriptor: Descriptor) -> Option<usize> {
    const STATS_BY_OPCODE: [u16; 2] = [0x0301, 0x0302];
    match descriptor.branch {
        // A context selector is a whole container wherever it appears.
        Branch::ObjectContext => None,
        Branch::Attribute if STATS_BY_OPCODE.contains(&descriptor.leaf) => Some(3),
        _ => Some(0),
    }
}

/// Walk a variable list until its terminator or its end.
///
/// `shape` comes from the opcode: see [`Opcode::list_shape`].
pub fn parse_variables(mut bytes: &[u8], shape: ListShape) -> Vec<Variable> {
    let mut out = Vec::new();
    while bytes.len() >= 3 {
        let branch = Branch::from(bytes[0]);
        let leaf = u16::from_be_bytes([bytes[1], bytes[2]]);
        if branch == Branch::Terminator && leaf == 0 {
            break;
        }
        let descriptor = Descriptor { branch, leaf };
        if shape == ListShape::Addresses {
            if let Some(n) = request_parameter_len(descriptor) {
                let take = n.min(bytes.len() - 3);
                out.push(Variable::Address {
                    descriptor,
                    parameters: bytes[3..3 + take].to_vec(),
                });
                bytes = &bytes[3 + take..];
                continue;
            }
        }
        // A descriptor with no length octet ends the list too.
        let Some(&raw_len) = bytes.get(3) else { break };
        let length = Length::from_u8(raw_len);
        let take = length.data_len().min(bytes.len().saturating_sub(4));
        out.push(Variable::Container(Container {
            descriptor,
            length,
            value: bytes[4..4 + take].to_vec(),
        }));
        bytes = &bytes[4 + take..];
    }
    out
}

/// Frame header shared by every extended OAMPDU we build.
#[derive(Clone, Copy, Debug)]
pub struct Header {
    pub dst: MacAddr,
    pub src: MacAddr,
    pub flags: Flags,
    pub oui: Oui,
    pub opcode: Opcode,
}

impl Header {
    fn writer(&self) -> FrameWriter {
        let mut w = FrameWriter::ethernet(self.dst, self.src, EtherType::SlowProtocol);
        w.u8(SUBTYPE_OAM)
            .u16(self.flags.as_u16())
            .u8(Code::OrganizationSpecific.as_u8())
            .bytes(&self.oui.0)
            .u8(self.opcode.as_u8());
        w
    }
}

/// Build a request reading `descriptors`, terminated so a reader stops
/// where we intend it to.
pub fn get_request(header: Header, descriptors: &[Descriptor]) -> Vec<u8> {
    let mut w = header.writer();
    for d in descriptors {
        d.write(&mut w);
    }
    // Terminator: branch, leaf and length all zero.
    w.u8(Branch::Terminator.as_u8()).u16(0).u8(0);
    w.pad_to(MIN_FRAME_LEN)
}

/// Build a response carrying `containers`.
pub fn get_response(header: Header, containers: &[Container]) -> Vec<u8> {
    let mut w = header.writer();
    for c in containers {
        c.descriptor.write(&mut w);
        w.u8(c.length.as_u8());
        w.bytes(&c.value);
    }
    w.u8(Branch::Terminator.as_u8()).u16(0).u8(0);
    w.pad_to(MIN_FRAME_LEN)
}

#[cfg(test)]
mod tests {
    use super::*;

    const PEER: MacAddr = MacAddr::new([0x02, 0x00, 0x00, 0x00, 0x00, 0x01]);
    const ONU: MacAddr = MacAddr::new([0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF]);

    fn header(opcode: Opcode) -> Header {
        Header {
            dst: ONU,
            src: PEER,
            flags: Flags::converged(),
            oui: Oui::DPOE,
            opcode,
        }
    }

    #[test]
    fn opcodes_and_branches_round_trip() {
        for op in [
            Opcode::GetRequest,
            Opcode::GetResponse,
            Opcode::SetRequest,
            Opcode::SetResponse,
            Opcode::LinkEvents,
            Opcode::Multipart,
        ] {
            assert_eq!(Opcode::from(op.as_u8()), op);
        }
        for b in [
            Branch::Terminator,
            Branch::ObjectContext,
            Branch::Attribute,
            Branch::Counter,
            Branch::Action,
        ] {
            assert_eq!(Branch::from(b.as_u8()), b);
        }
    }

    #[test]
    fn zero_length_means_a_full_value_not_an_empty_one() {
        assert_eq!(Length::from_u8(0), Length::Full);
        assert_eq!(Length::Full.data_len(), 128);
        assert_eq!(Length::from_u8(0x0C).data_len(), 12);
        // The top bit turns the octet into a status with no value.
        assert_eq!(Length::from_u8(0xA1), Length::Status(0xA1));
        assert_eq!(Length::Status(0xA1).data_len(), 0);
    }

    #[test]
    fn of_value_refuses_the_encodings_that_would_be_misread() {
        // Empty has no encoding: zero would announce 128 bytes.
        assert_eq!(Length::of_value(0), None);
        assert_eq!(Length::of_value(1), Some(Length::Bytes(1)));
        assert_eq!(Length::of_value(127), Some(Length::Bytes(127)));
        // 128 has one, and it is not 128.
        assert_eq!(Length::of_value(128), Some(Length::Full));
        assert_eq!(Length::of_value(128).map(Length::as_u8), Some(0));
        assert_eq!(Length::of_value(129), None);
    }

    #[test]
    fn a_request_round_trips_through_parse() {
        let wanted = [
            Descriptor::attribute(leaf::FIRMWARE_INFO),
            Descriptor::attribute(leaf::MANUFACTURER_INFO),
        ];
        let frame = get_request(header(Opcode::GetRequest), &wanted);
        let pdu = Pdu::parse(&frame).expect("parses");
        assert_eq!(pdu.opcode, Opcode::GetRequest);
        assert_eq!(pdu.oui, Oui::DPOE);
        // A request names addresses and carries no values. Reading it as a
        // list of containers used to yield one fabricated container whose
        // length and value were the addresses that followed it.
        assert_eq!(pdu.containers().count(), 0);
        assert_eq!(
            pdu.variables,
            wanted
                .iter()
                .map(|d| Variable::Address { descriptor: *d, parameters: Vec::new() })
                .collect::<Vec<_>>()
        );
    }

    /// Two leaves take a parameter inside a request, so the address that
    /// follows them starts three bytes further on.
    #[test]
    fn a_statistics_request_steps_over_its_parameter() {
        let bytes = [
            Branch::Attribute.as_u8(), 0x03, 0x01, 0xAA, 0xBB, 0xCC,
            Branch::Attribute.as_u8(), 0x00, 0x03,
            Branch::Terminator.as_u8(), 0x00, 0x00,
        ];
        let vars = parse_variables(&bytes, ListShape::Addresses);
        assert_eq!(
            vars,
            vec![
                Variable::Address {
                    descriptor: Descriptor::attribute(0x0301),
                    parameters: vec![0xAA, 0xBB, 0xCC],
                },
                Variable::Address {
                    descriptor: Descriptor::attribute(leaf::FIRMWARE_INFO),
                    parameters: Vec::new(),
                },
            ]
        );
    }

    /// A context selector is written out in full wherever it appears, so it
    /// is a container even in a request.
    #[test]
    fn an_object_context_is_a_container_even_in_a_request() {
        let bytes = [
            Branch::ObjectContext.as_u8(), 0x00, 0x02, 0x02, 0x00, 0x00,
            Branch::Attribute.as_u8(), 0x00, 0x03,
            Branch::Terminator.as_u8(), 0x00, 0x00,
        ];
        let vars = parse_variables(&bytes, ListShape::Addresses);
        assert_eq!(vars.len(), 2);
        assert_eq!(
            vars[0],
            Variable::Container(Container {
                descriptor: Descriptor { branch: Branch::ObjectContext, leaf: 0x0002 },
                length: Length::Bytes(2),
                value: vec![0x00, 0x00],
            })
        );
        assert_eq!(vars[1].descriptor(), Descriptor::attribute(leaf::FIRMWARE_INFO));
    }

    #[test]
    fn a_response_round_trips_through_parse() {
        let containers = vec![
            Container {
                descriptor: Descriptor::attribute(leaf::FIRMWARE_INFO),
                length: Length::Bytes(12),
                value: vec![0xAB; 12],
            },
            Container {
                descriptor: Descriptor::attribute(leaf::MANUFACTURER_INFO),
                length: Length::Bytes(16),
                value: vec![0xCD; 16],
            },
        ];
        let frame = get_response(header(Opcode::GetResponse), &containers);
        let pdu = Pdu::parse(&frame).expect("parses");
        assert_eq!(pdu.opcode, Opcode::GetResponse);
        assert_eq!(pdu.containers().cloned().collect::<Vec<_>>(), containers);
    }

    #[test]
    fn a_status_container_carries_no_value() {
        let containers = vec![Container {
            descriptor: Descriptor::attribute(0x00AA),
            length: Length::Status(0xA1),
            value: Vec::new(),
        }];
        let frame = get_response(header(Opcode::GetResponse), &containers);
        let pdu = Pdu::parse(&frame).expect("parses");
        let parsed: Vec<_> = pdu.containers().collect();
        assert_eq!(parsed.len(), 1);
        assert!(parsed[0].is_status());
        assert!(parsed[0].value.is_empty());
    }

    #[test]
    fn the_walker_stops_at_the_terminator() {
        // Two variables, a terminator, then bytes that must not be read.
        let mut bytes = vec![
            Branch::Attribute.as_u8(), 0x00, 0x03, 0x02, 0x11, 0x22,
            Branch::Terminator.as_u8(), 0x00, 0x00, 0x00,
        ];
        bytes.extend_from_slice(&[Branch::Attribute.as_u8(), 0x00, 0x06, 0x04, 1, 2, 3, 4]);
        let vars = parse_variables(&bytes, ListShape::Containers);
        assert_eq!(vars.len(), 1);
        assert_eq!(
            vars[0].as_container().expect("a container").value,
            vec![0x11, 0x22]
        );
    }

    #[test]
    fn a_truncated_value_does_not_run_past_the_buffer() {
        // Announces 128 bytes but carries four.
        let bytes = [Branch::Attribute.as_u8(), 0x00, 0x03, 0x00, 1, 2, 3, 4];
        let vars = parse_variables(&bytes, ListShape::Containers);
        assert_eq!(vars.len(), 1);
        let c = vars[0].as_container().expect("a container");
        assert_eq!(c.length, Length::Full);
        assert_eq!(c.value, vec![1, 2, 3, 4]);
    }

    #[test]
    fn parse_rejects_a_plain_information_pdu() {
        let frame = oam::information(
            ONU,
            PEER,
            Flags::converged(),
            oam::InfoTlv {
                is_local: true,
                oam_version: 1,
                revision: 1,
                state: 0,
                configuration: 5,
                max_pdu_size: 0x0600,
                oui: Oui::DPOE,
                vendor_specific: [0; 4],
            },
        );
        assert!(Pdu::parse(&frame).is_none());
    }
}
