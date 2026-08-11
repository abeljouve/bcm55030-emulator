//! Extended OAM: organization-specific OAMPDUs and their variable encoding.
//!
//! An extended OAMPDU is an Information-class frame with code
//! [`Code::OrganizationSpecific`], an OUI, one octet under that OUI, and a
//! body. Variables are addressed by a branch and a leaf; a request carries
//! the address alone, a response carries a length and a value.
//!
//! **The octet after the OUI only means anything under its OUI.** A receiver
//! switches on the 24-bit identifier first and reaches a different table for
//! each one, so the pair travels together as a [`Channel`] and the octet is
//! never read on its own. Reading it without the OUI is what makes one
//! container's message announce itself as another's, and makes a body that
//! carries no variable list get walked as one — which fabricates containers
//! out of whatever bytes followed.

use std::fmt;

use crate::oam::{self, Code, Flags, Oui, SUBTYPE_OAM};
use crate::types::{EtherType, FrameWriter, MacAddr, MIN_FRAME_LEN};

/// Offset of the OUI, right after the OAMPDU code.
pub const OUI_OFFSET: usize = 18;
/// Offset of the octet that follows the OUI.
pub const CODE_OFFSET: usize = 21;
/// Offset of the body.
pub const BODY_OFFSET: usize = 22;

/// Messages of the DPoE variable channel, under [`Oui::DPOE`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DpoeOpcode {
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

impl DpoeOpcode {
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

    pub const fn from_u8(v: u8) -> Self {
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

impl fmt::Display for DpoeOpcode {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::GetRequest => write!(f, "get-request"),
            Self::GetResponse => write!(f, "get-response"),
            Self::SetRequest => write!(f, "set-request"),
            Self::SetResponse => write!(f, "set-response"),
            Self::LinkEvents => write!(f, "link-events"),
            Self::Multipart => write!(f, "multipart"),
            Self::Other(v) => write!(f, "code 0x{v:02X}"),
        }
    }
}

/// Messages of the second container, under [`Oui::VENDOR_EXTENSION`].
///
/// Only one of its codes has been observed to carry a variable list. The rest
/// are real messages this link exchanges, but their bodies are something else,
/// so they get a code and no shape rather than a name and a wrong one.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ExtensionMessage {
    /// The one code whose body is a variable list.
    Variables,
    Other(u8),
}

impl ExtensionMessage {
    /// The code the variable-list message travels under.
    pub const VARIABLES_CODE: u8 = 0x03;

    pub const fn as_u8(self) -> u8 {
        match self {
            Self::Variables => Self::VARIABLES_CODE,
            Self::Other(v) => v,
        }
    }

    pub const fn from_u8(v: u8) -> Self {
        match v {
            Self::VARIABLES_CODE => Self::Variables,
            other => Self::Other(other),
        }
    }
}

impl fmt::Display for ExtensionMessage {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Variables => write!(f, "variables"),
            Self::Other(v) => write!(f, "code 0x{v:02X}"),
        }
    }
}

/// The OUI and the octet under it, as one value.
///
/// The octet is only meaningful under its OUI, which is why the two are never
/// apart here: sending a code from one container under the other's OUI is not
/// something this type can express.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Channel {
    Dpoe(DpoeOpcode),
    Extension(ExtensionMessage),
    Other { oui: Oui, code: u8 },
}

impl Channel {
    pub const fn oui(self) -> Oui {
        match self {
            Self::Dpoe(_) => Oui::DPOE,
            Self::Extension(_) => Oui::VENDOR_EXTENSION,
            Self::Other { oui, .. } => oui,
        }
    }

    pub const fn code(self) -> u8 {
        match self {
            Self::Dpoe(op) => op.as_u8(),
            Self::Extension(m) => m.as_u8(),
            Self::Other { code, .. } => code,
        }
    }

    /// Read a channel from the two fields as they arrive: identifier first,
    /// octet second.
    pub fn decode(oui: Oui, code: u8) -> Self {
        match oui {
            Oui::DPOE => Self::Dpoe(DpoeOpcode::from_u8(code)),
            Oui::VENDOR_EXTENSION => Self::Extension(ExtensionMessage::from_u8(code)),
            other => Self::Other { oui: other, code },
        }
    }

    /// How the body that follows is laid out.
    ///
    /// A request names what it wants; a response carries what was found. The
    /// two are not distinguishable from the bytes alone, so reading a list
    /// without knowing the message means reading a request's next address as
    /// the previous one's length — which yields a plausible container made of
    /// the addresses that came after it.
    pub const fn body_shape(self) -> BodyShape {
        match self {
            Self::Dpoe(DpoeOpcode::GetRequest) => BodyShape::Addresses,
            Self::Dpoe(_) => BodyShape::Containers,
            Self::Extension(ExtensionMessage::Variables) => BodyShape::Containers,
            // Every other code of the second container, and every unknown
            // container: nothing says these bodies are variable lists.
            Self::Extension(ExtensionMessage::Other(_)) | Self::Other { .. } => BodyShape::Opaque,
        }
    }
}

impl fmt::Display for Channel {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Dpoe(op) => write!(f, "{op}"),
            Self::Extension(m) => write!(f, "{m}"),
            Self::Other { code, .. } => write!(f, "code 0x{code:02X}"),
        }
    }
}

/// What the body of an extended OAMPDU is.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BodyShape {
    /// Addresses alone, plus whatever parameters they take.
    Addresses,
    /// Addresses with a length and a value.
    Containers,
    /// Not a variable list. Left undecoded rather than walked as one: only
    /// one code of the extension container has been observed to carry a
    /// list, and walking the rest fabricates containers.
    Opaque,
}

/// A byte range of the frame a parsed thing came from.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Span {
    pub start: usize,
    pub len: usize,
}

impl Span {
    pub const fn new(start: usize, len: usize) -> Self {
        Self { start, len }
    }

    pub const fn end(self) -> usize {
        self.start + self.len
    }
}

/// A variable together with the bytes it was read from.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Located {
    pub variable: Variable,
    pub span: Span,
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

/// Leaves seen on this link.
pub mod leaf {
    pub const DEVICE_ID: u16 = 0x0002;
    pub const FIRMWARE_INFO: u16 = 0x0003;
    pub const MANUFACTURER_INFO: u16 = 0x0006;
    pub const MANUFACTURER_ORG_NAME: u16 = 0x000E;
    /// Carries one entry of a classifier rule list. Under the attribute
    /// branch; see [`crate::rule_list`].
    pub const RULE_LIST: u16 = 0x0501;
    /// Under the action branch: put the accumulated entries into effect.
    pub const RULE_LIST_APPLY: u16 = 0x0502;
    /// Queue configuration, as one container.
    pub const QUEUE_CONFIG: u16 = 0x7FF1;
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

    pub const fn action(leaf: u16) -> Self {
        Self { branch: Branch::Action, leaf }
    }

    pub const fn object_context(leaf: u16) -> Self {
        Self { branch: Branch::ObjectContext, leaf }
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
    pub channel: Channel,
    /// Entries of the variable list, each with the bytes it came from. Empty
    /// when the body is opaque — a body nothing says is a list is kept, not
    /// walked.
    pub variables: Vec<Located>,
    /// Set when [`Channel::body_shape`] is [`BodyShape::Opaque`]: the body is
    /// recorded so it can be shown and re-sent, and deliberately not decoded.
    pub opaque_body: Option<Span>,
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
        // The identifier decides what the next octet means, so it is read
        // first and the octet is never interpreted without it.
        let channel = Channel::decode(oui, *frame.get(CODE_OFFSET)?);
        let body = frame.get(BODY_OFFSET..).unwrap_or(&[]);
        let shape = channel.body_shape();
        let (variables, opaque_body) = match shape {
            BodyShape::Opaque => (Vec::new(), Some(Span::new(BODY_OFFSET, body.len()))),
            shape => (parse_variables(body, shape, BODY_OFFSET), None),
        };
        Some(Self { oui, channel, variables, opaque_body })
    }

    /// The shape the body was read with.
    pub fn body_shape(&self) -> BodyShape {
        self.channel.body_shape()
    }

    /// The variables, without their spans.
    pub fn variables(&self) -> impl Iterator<Item = &Variable> {
        self.variables.iter().map(|l| &l.variable)
    }

    /// The entries that carry a value. A request has none, and so does a
    /// body that was never parsed as a list.
    pub fn containers(&self) -> impl Iterator<Item = &Container> {
        self.variables.iter().filter_map(|l| l.variable.as_container())
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
/// `shape` comes from the channel: see [`Channel::body_shape`]. `base` is the
/// offset of `bytes` inside the frame, so every entry can carry the range it
/// was read from — a dissection whose fields have no range cannot highlight
/// the bytes behind a selection, which is the whole contract of the tree.
///
/// [`BodyShape::Opaque`] never reaches here: a caller that cannot say the body
/// is a list must not have it walked as one.
pub fn parse_variables(bytes: &[u8], shape: BodyShape, base: usize) -> Vec<Located> {
    let mut out = Vec::new();
    if shape == BodyShape::Opaque {
        return out;
    }
    let mut at = 0usize;
    while bytes.len() - at >= 3 {
        let rest = &bytes[at..];
        let branch = Branch::from(rest[0]);
        let leaf = u16::from_be_bytes([rest[1], rest[2]]);
        if branch == Branch::Terminator && leaf == 0 {
            break;
        }
        let descriptor = Descriptor { branch, leaf };
        if shape == BodyShape::Addresses {
            if let Some(n) = request_parameter_len(descriptor) {
                let take = n.min(rest.len() - 3);
                out.push(Located {
                    variable: Variable::Address {
                        descriptor,
                        parameters: rest[3..3 + take].to_vec(),
                    },
                    span: Span::new(base + at, 3 + take),
                });
                at += 3 + take;
                continue;
            }
        }
        // A descriptor with no length octet ends the list too.
        let Some(&raw_len) = rest.get(3) else { break };
        let length = Length::from_u8(raw_len);
        let take = length.data_len().min(rest.len().saturating_sub(4));
        out.push(Located {
            variable: Variable::Container(Container {
                descriptor,
                length,
                value: rest[4..4 + take].to_vec(),
            }),
            span: Span::new(base + at, 4 + take),
        });
        at += 4 + take;
    }
    out
}

/// Frame header shared by every extended OAMPDU we build.
///
/// The identifier and the octet under it are one field here, so a code from
/// one container cannot be written out under the other's identifier.
#[derive(Clone, Copy, Debug)]
pub struct Header {
    pub dst: MacAddr,
    pub src: MacAddr,
    pub flags: Flags,
    pub channel: Channel,
}

impl Header {
    /// The identifier this header will write.
    pub const fn oui(&self) -> Oui {
        self.channel.oui()
    }

    pub fn writer(&self) -> FrameWriter {
        let mut w = FrameWriter::ethernet(self.dst, self.src, EtherType::SlowProtocol);
        w.u8(SUBTYPE_OAM)
            .u16(self.flags.as_u16())
            .u8(Code::OrganizationSpecific.as_u8())
            .bytes(&self.channel.oui().0)
            .u8(self.channel.code());
        w
    }
}

/// Write one variable, whichever kind it is.
fn write_variable(w: &mut FrameWriter, v: &Variable) {
    match v {
        Variable::Address { descriptor, parameters } => {
            descriptor.write(w);
            w.bytes(parameters);
        }
        Variable::Container(c) => {
            c.descriptor.write(w);
            w.u8(c.length.as_u8());
            w.bytes(&c.value);
        }
    }
}

/// The three zero octets that end a variable list: branch, leaf and length.
fn write_terminator(w: &mut FrameWriter) {
    w.u8(Branch::Terminator.as_u8()).u16(0).u8(0);
}

/// Build a message whose body is `variables`, terminated so a reader stops
/// where we intend it to.
pub fn variables_message(header: Header, variables: &[Variable]) -> Vec<u8> {
    let mut w = header.writer();
    for v in variables {
        write_variable(&mut w, v);
    }
    write_terminator(&mut w);
    w.pad_to(MIN_FRAME_LEN)
}

/// Build a request reading `descriptors`.
pub fn get_request(header: Header, descriptors: &[Descriptor]) -> Vec<u8> {
    let mut w = header.writer();
    for d in descriptors {
        d.write(&mut w);
    }
    write_terminator(&mut w);
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
    write_terminator(&mut w);
    w.pad_to(MIN_FRAME_LEN)
}

/// Build a message whose body is handed over verbatim.
///
/// The escape hatch that keeps a refusal measurable: a receiver's answer to a
/// malformed message is an observation, and normalising the bytes on the way
/// out destroys it.
pub fn raw_message(header: Header, body: &[u8]) -> Vec<u8> {
    let mut w = header.writer();
    w.bytes(body);
    w.pad_to(MIN_FRAME_LEN)
}

#[cfg(test)]
mod tests {
    use super::*;

    const PEER: MacAddr = MacAddr::new([0x02, 0x00, 0x00, 0x00, 0x00, 0x01]);
    const ONU: MacAddr = MacAddr::new([0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF]);

    fn header(channel: Channel) -> Header {
        Header { dst: ONU, src: PEER, flags: Flags::converged(), channel }
    }

    /// Codes of the second container seen carrying traffic on this link.
    /// A fixture, not a table the model consults: what the model needs to
    /// know about them is that their bodies are not variable lists.
    const EXTENSION_CODES_SEEN: [u8; 12] =
        [0x00, 0x01, 0x03, 0x06, 0x09, 0x0A, 0x0B, 0x0C, 0x11, 0x13, 0x15, 0x17];

    #[test]
    fn codes_and_branches_round_trip_under_their_own_identifier() {
        for op in [
            DpoeOpcode::GetRequest,
            DpoeOpcode::GetResponse,
            DpoeOpcode::SetRequest,
            DpoeOpcode::SetResponse,
            DpoeOpcode::LinkEvents,
            DpoeOpcode::Multipart,
            DpoeOpcode::Other(0x7F),
        ] {
            assert_eq!(DpoeOpcode::from_u8(op.as_u8()), op);
            let c = Channel::Dpoe(op);
            assert_eq!(Channel::decode(c.oui(), c.code()), c);
        }
        for code in EXTENSION_CODES_SEEN {
            let m = ExtensionMessage::from_u8(code);
            assert_eq!(m.as_u8(), code);
            let c = Channel::Extension(m);
            assert_eq!(Channel::decode(Oui::VENDOR_EXTENSION, code), c);
        }
        let unknown = Oui([0xAA, 0xBB, 0xCC]);
        assert_eq!(
            Channel::decode(unknown, 3),
            Channel::Other { oui: unknown, code: 3 }
        );
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

    /// The same octet under two identifiers is two different messages, and
    /// the bodies behind them are laid out differently. Reading the octet
    /// without its identifier is what made one container's message announce
    /// itself as the other's.
    #[test]
    fn the_same_octet_is_a_different_code_under_each_oui() {
        let code = ExtensionMessage::VARIABLES_CODE;
        let dpoe = Channel::decode(Oui::DPOE, code);
        let ext = Channel::decode(Oui::VENDOR_EXTENSION, code);
        assert_ne!(dpoe, ext);
        assert_eq!(dpoe, Channel::Dpoe(DpoeOpcode::SetRequest));
        assert_eq!(ext, Channel::Extension(ExtensionMessage::Variables));
        // Same octet on the wire, same shape here — but the code 1 that
        // reads as a read request under one identifier carries no list at
        // all under the other.
        assert_eq!(dpoe.code(), ext.code());
        assert_eq!(
            Channel::decode(Oui::DPOE, 1).body_shape(),
            BodyShape::Addresses
        );
        assert_eq!(
            Channel::decode(Oui::VENDOR_EXTENSION, 1).body_shape(),
            BodyShape::Opaque
        );
    }

    #[test]
    fn only_one_code_of_the_extension_container_carries_a_list() {
        let carrying: Vec<u8> = EXTENSION_CODES_SEEN
            .into_iter()
            .filter(|&c| {
                Channel::decode(Oui::VENDOR_EXTENSION, c).body_shape() != BodyShape::Opaque
            })
            .collect();
        assert_eq!(carrying, vec![ExtensionMessage::VARIABLES_CODE]);
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
        let frame = get_request(header(Channel::Dpoe(DpoeOpcode::GetRequest)), &wanted);
        let pdu = Pdu::parse(&frame).expect("parses");
        assert_eq!(pdu.channel, Channel::Dpoe(DpoeOpcode::GetRequest));
        assert_eq!(pdu.oui, Oui::DPOE);
        // A request names addresses and carries no values. Reading it as a
        // list of containers used to yield one fabricated container whose
        // length and value were the addresses that followed it.
        assert_eq!(pdu.containers().count(), 0);
        assert_eq!(
            pdu.variables().cloned().collect::<Vec<_>>(),
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
        let vars = parse_variables(&bytes, BodyShape::Addresses, 0);
        assert_eq!(
            vars.iter().map(|l| l.variable.clone()).collect::<Vec<_>>(),
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
        // And each one says which bytes it came from.
        assert_eq!(vars[0].span, Span::new(0, 6));
        assert_eq!(vars[1].span, Span::new(6, 3));
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
        let vars = parse_variables(&bytes, BodyShape::Addresses, 0);
        assert_eq!(vars.len(), 2);
        assert_eq!(
            vars[0].variable,
            Variable::Container(Container {
                descriptor: Descriptor::object_context(0x0002),
                length: Length::Bytes(2),
                value: vec![0x00, 0x00],
            })
        );
        assert_eq!(vars[1].variable.descriptor(), Descriptor::attribute(leaf::FIRMWARE_INFO));
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
        let frame = get_response(header(Channel::Dpoe(DpoeOpcode::GetResponse)), &containers);
        let pdu = Pdu::parse(&frame).expect("parses");
        assert_eq!(pdu.channel, Channel::Dpoe(DpoeOpcode::GetResponse));
        assert_eq!(pdu.containers().cloned().collect::<Vec<_>>(), containers);
    }

    #[test]
    fn a_status_container_carries_no_value() {
        let containers = vec![Container {
            descriptor: Descriptor::attribute(0x00AA),
            length: Length::Status(0xA1),
            value: Vec::new(),
        }];
        let frame = get_response(header(Channel::Dpoe(DpoeOpcode::GetResponse)), &containers);
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
        let vars = parse_variables(&bytes, BodyShape::Containers, 0);
        assert_eq!(vars.len(), 1);
        assert_eq!(
            vars[0].variable.as_container().expect("a container").value,
            vec![0x11, 0x22]
        );
    }

    #[test]
    fn a_truncated_value_does_not_run_past_the_buffer() {
        // Announces 128 bytes but carries four.
        let bytes = [Branch::Attribute.as_u8(), 0x00, 0x03, 0x00, 1, 2, 3, 4];
        let vars = parse_variables(&bytes, BodyShape::Containers, 0);
        assert_eq!(vars.len(), 1);
        let c = vars[0].variable.as_container().expect("a container");
        assert_eq!(c.length, Length::Full);
        assert_eq!(c.value, vec![1, 2, 3, 4]);
        // The span stops at the buffer too, or a view would highlight
        // bytes the frame does not have.
        assert_eq!(vars[0].span, Span::new(0, bytes.len()));
    }

    /// A body nothing says is a variable list is kept and not walked. Walking
    /// it produced containers made of whatever bytes followed, and each one
    /// was counted as a reply.
    #[test]
    fn an_opaque_body_is_not_walked_as_a_variable_list() {
        let body = [0xD7, 0x05, 0x01, 0x04, 0xDE, 0xAD, 0xBE, 0xEF];
        let frame = raw_message(
            header(Channel::Extension(ExtensionMessage::Other(0x0A))),
            &body,
        );
        let pdu = Pdu::parse(&frame).expect("parses");
        assert_eq!(pdu.body_shape(), BodyShape::Opaque);
        assert!(pdu.variables.is_empty());
        assert_eq!(pdu.containers().count(), 0);
        let span = pdu.opaque_body.expect("the body is kept");
        assert_eq!(span.start, BODY_OFFSET);
        assert_eq!(&frame[span.start..span.start + body.len()], &body);
        assert!(span.end() <= frame.len());
    }

    #[test]
    fn a_raw_body_survives_untouched() {
        // Deliberately malformed: a length octet promising more than is
        // there, under a channel whose body is a list. Its bytes must reach
        // the wire as given, because the receiver's refusal is the
        // measurement.
        let body = [0xD7, 0x05, 0x01, 0x7F, 0x01, 0x02];
        let frame = raw_message(
            header(Channel::Extension(ExtensionMessage::Variables)),
            &body,
        );
        assert_eq!(&frame[BODY_OFFSET..BODY_OFFSET + body.len()], &body);
    }

    #[test]
    fn every_variable_carries_the_span_it_was_read_from() {
        let containers = vec![
            Container {
                descriptor: Descriptor::attribute(leaf::FIRMWARE_INFO),
                length: Length::Bytes(4),
                value: vec![1, 2, 3, 4],
            },
            Container {
                descriptor: Descriptor::attribute(leaf::DEVICE_ID),
                length: Length::Bytes(2),
                value: vec![9, 9],
            },
        ];
        let frame = get_response(header(Channel::Dpoe(DpoeOpcode::GetResponse)), &containers);
        let pdu = Pdu::parse(&frame).expect("parses");
        assert_eq!(pdu.variables.len(), 2);
        let mut expected = BODY_OFFSET;
        for (located, c) in pdu.variables.iter().zip(&containers) {
            assert_eq!(located.span.start, expected);
            assert_eq!(located.span.len, 4 + c.value.len());
            assert!(located.span.end() <= frame.len());
            expected += located.span.len;
        }
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
