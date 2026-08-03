//! Shared Ethernet primitives for the OLT protocol modules.

use std::fmt;

/// Minimum Ethernet frame size excluding the FCS.
pub const MIN_FRAME_LEN: usize = 60;

/// Offset of the EtherType field in an Ethernet frame.
pub const ETHERTYPE_OFFSET: usize = 12;

/// Length of the Ethernet header: two addresses plus the EtherType.
pub const ETHERNET_HEADER_LEN: usize = 14;

/// A 48-bit Ethernet address.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Default)]
pub struct MacAddr(pub [u8; 6]);

impl MacAddr {
    pub const ZERO: Self = Self([0; 6]);
    /// Destination of MPCP control frames (IEEE 802.3 annex 31A).
    pub const MPCP_MULTICAST: Self = Self([0x01, 0x80, 0xC2, 0x00, 0x00, 0x01]);
    /// Destination of slow-protocol frames.
    pub const SLOW_PROTOCOL_MULTICAST: Self = Self([0x01, 0x80, 0xC2, 0x00, 0x00, 0x02]);

    pub const fn new(octets: [u8; 6]) -> Self {
        Self(octets)
    }

    pub const fn octets(&self) -> [u8; 6] {
        self.0
    }

    pub fn is_zero(&self) -> bool {
        self.0 == [0; 6]
    }

    /// Read an address from `bytes`, which must be at least six long.
    pub fn from_slice(bytes: &[u8]) -> Option<Self> {
        bytes.get(..6).map(|b| {
            let mut octets = [0u8; 6];
            octets.copy_from_slice(b);
            Self(octets)
        })
    }
}

impl fmt::Display for MacAddr {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let [a, b, c, d, e, g] = self.0;
        write!(f, "{a:02X}:{b:02X}:{c:02X}:{d:02X}:{e:02X}:{g:02X}")
    }
}

impl From<[u8; 6]> for MacAddr {
    fn from(octets: [u8; 6]) -> Self {
        Self(octets)
    }
}

impl From<MacAddr> for [u8; 6] {
    fn from(mac: MacAddr) -> Self {
        mac.0
    }
}

/// EtherTypes the model routes on.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EtherType {
    /// Multi-point control protocol (IEEE 802.3 clause 64).
    Mpcp,
    /// Slow protocols, which carry OAM (IEEE 802.3 clause 57).
    SlowProtocol,
    /// EAP over LAN.
    Eapol,
    Other(u16),
}

impl EtherType {
    pub const fn as_u16(self) -> u16 {
        match self {
            Self::Mpcp => 0x8808,
            Self::SlowProtocol => 0x8809,
            Self::Eapol => 0x888E,
            Self::Other(v) => v,
        }
    }

    /// Read the EtherType of `frame`, if it is long enough to have one.
    pub fn of_frame(frame: &[u8]) -> Option<Self> {
        let hi = *frame.get(ETHERTYPE_OFFSET)?;
        let lo = *frame.get(ETHERTYPE_OFFSET + 1)?;
        Some(Self::from(u16::from_be_bytes([hi, lo])))
    }
}

impl From<u16> for EtherType {
    fn from(v: u16) -> Self {
        match v {
            0x8808 => Self::Mpcp,
            0x8809 => Self::SlowProtocol,
            0x888E => Self::Eapol,
            other => Self::Other(other),
        }
    }
}

impl fmt::Display for EtherType {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Mpcp => write!(f, "MPCP"),
            Self::SlowProtocol => write!(f, "slow-protocol"),
            Self::Eapol => write!(f, "EAPOL"),
            Self::Other(v) => write!(f, "0x{v:04X}"),
        }
    }
}

/// A logical link identifier.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Default)]
pub struct Llid(pub u16);

impl Llid {
    pub const fn as_u16(self) -> u16 {
        self.0
    }
}

impl fmt::Display for Llid {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.0)
    }
}

/// Appends big-endian fields to a frame under construction, so protocol
/// modules describe layout instead of juggling byte slices.
#[derive(Debug, Default)]
pub struct FrameWriter {
    bytes: Vec<u8>,
}

impl FrameWriter {
    pub fn with_capacity(n: usize) -> Self {
        Self { bytes: Vec::with_capacity(n) }
    }

    /// Start a frame with its destination, source and EtherType.
    pub fn ethernet(dst: MacAddr, src: MacAddr, ethertype: EtherType) -> Self {
        let mut w = Self::with_capacity(MIN_FRAME_LEN);
        w.mac(dst);
        w.mac(src);
        w.u16(ethertype.as_u16());
        w
    }

    pub fn u8(&mut self, v: u8) -> &mut Self {
        self.bytes.push(v);
        self
    }

    pub fn u16(&mut self, v: u16) -> &mut Self {
        self.bytes.extend_from_slice(&v.to_be_bytes());
        self
    }

    pub fn u32(&mut self, v: u32) -> &mut Self {
        self.bytes.extend_from_slice(&v.to_be_bytes());
        self
    }

    pub fn mac(&mut self, v: MacAddr) -> &mut Self {
        self.bytes.extend_from_slice(&v.0);
        self
    }

    pub fn bytes(&mut self, v: &[u8]) -> &mut Self {
        self.bytes.extend_from_slice(v);
        self
    }

    pub fn len(&self) -> usize {
        self.bytes.len()
    }

    pub fn is_empty(&self) -> bool {
        self.bytes.is_empty()
    }

    /// Zero-pad up to `len` and return the frame.
    pub fn pad_to(mut self, len: usize) -> Vec<u8> {
        if self.bytes.len() < len {
            self.bytes.resize(len, 0);
        }
        self.bytes
    }

    pub fn finish(self) -> Vec<u8> {
        self.bytes
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mac_round_trips_through_a_slice() {
        let mac = MacAddr::new([0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF]);
        assert_eq!(MacAddr::from_slice(&mac.octets()), Some(mac));
        assert_eq!(mac.to_string(), "AA:BB:CC:DD:EE:FF");
        assert!(MacAddr::from_slice(&[0, 1, 2]).is_none());
    }

    #[test]
    fn ethertype_round_trips() {
        for et in [EtherType::Mpcp, EtherType::SlowProtocol, EtherType::Eapol] {
            assert_eq!(EtherType::from(et.as_u16()), et);
        }
        assert_eq!(EtherType::from(0x0800), EtherType::Other(0x0800));
    }

    #[test]
    fn ethertype_of_frame_needs_a_full_header() {
        let frame = FrameWriter::ethernet(
            MacAddr::MPCP_MULTICAST,
            MacAddr::ZERO,
            EtherType::Mpcp,
        )
        .finish();
        assert_eq!(frame.len(), ETHERNET_HEADER_LEN);
        assert_eq!(EtherType::of_frame(&frame), Some(EtherType::Mpcp));
        assert_eq!(EtherType::of_frame(&frame[..13]), None);
    }

    #[test]
    fn pad_to_never_truncates() {
        let mut w = FrameWriter::with_capacity(4);
        w.u32(0xDEADBEEF);
        assert_eq!(w.pad_to(2).len(), 4);
    }
}
