//! The frame check sequence an Ethernet frame carries on the wire.
//!
//! IEEE 802.3 clause 3.2.9: a CRC-32 over everything from the destination
//! address to the end of the payload, transmitted least-significant octet
//! first. It is part of the frame, not a property of the transmitter: a
//! receiving MAC deposits it in its buffer along with the rest, and software
//! reading that buffer finds four octets there.
//!
//! Modelling the wire without it is not a harmless simplification. A receiver
//! that expects an FCS neutralises the last four octets of what it was handed
//! and shortens the length it reports by four -- so a frame built without one
//! loses four octets of payload at the far end, silently, and reports a length
//! four too large. That is a difference the far end can measure, which makes
//! it a difference this model has to have.

/// Octets the FCS occupies. A frame on the wire is this much longer than the
/// PDU that was built.
pub const FCS_LEN: usize = 4;

/// The CRC-32 generator of IEEE 802.3, reflected.
///
/// The standard writes the polynomial as `0x04C11DB7` most-significant bit
/// first; processing octets least-significant bit first -- which is the order
/// they go on the wire -- turns it into this.
const POLY_REFLECTED: u32 = 0xEDB8_8320;

/// The FCS of `bytes`, as the value to append.
pub fn crc32(bytes: &[u8]) -> u32 {
    let mut crc = 0xFFFF_FFFFu32;
    for &b in bytes {
        crc ^= b as u32;
        for _ in 0..8 {
            let lsb = crc & 1;
            crc >>= 1;
            if lsb != 0 {
                crc ^= POLY_REFLECTED;
            }
        }
    }
    !crc
}

/// The frame as it goes on the wire: the PDU with its FCS appended.
pub fn appended(mut frame: Vec<u8>) -> Vec<u8> {
    let fcs = crc32(&frame);
    frame.extend_from_slice(&fcs.to_le_bytes());
    frame
}

/// Whether `frame` ends in an FCS that matches the rest of it.
///
/// Nothing in the model rejects a frame on this, and nothing should start to
/// without a reason to: the point of having it is that the check can be made
/// at all, which is what tells a real FCS from four octets called one.
pub fn is_valid(frame: &[u8]) -> bool {
    let Some(split) = frame.len().checked_sub(FCS_LEN) else {
        return false;
    };
    let (body, tail) = frame.split_at(split);
    crc32(body).to_le_bytes() == tail
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The check value every CRC-32 catalogue lists for this parameter set.
    #[test]
    fn matches_the_published_check_value() {
        assert_eq!(crc32(b"123456789"), 0xCBF4_3926);
    }

    #[test]
    fn an_appended_fcs_checks_out() {
        let frame = appended(b"a frame of no particular shape".to_vec());
        assert!(is_valid(&frame));
    }

    #[test]
    fn appending_grows_the_frame_by_the_fcs() {
        let pdu = vec![0u8; 60];
        assert_eq!(appended(pdu).len(), 60 + FCS_LEN);
    }

    /// One flipped octet has to fail, or the check is measuring nothing.
    #[test]
    fn a_corrupted_frame_does_not_check_out() {
        let mut frame = appended(vec![0x11, 0x22, 0x33, 0x44, 0x55]);
        frame[2] ^= 0x01;
        assert!(!is_valid(&frame));
    }

    #[test]
    fn a_frame_too_short_to_hold_one_does_not_check_out() {
        assert!(!is_valid(&[0x01, 0x02, 0x03]));
    }
}
