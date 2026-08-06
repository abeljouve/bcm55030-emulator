//! The MAC's report engine.
//!
//! A REPORT is the one upstream MPCPDU the CPU never builds. The firmware
//! programs a template block and the MAC emits from it, in the window a
//! GATE granted. Without this, the template is written and nothing ever
//! comes of it — and "no report seen" measures the emulator, not the
//! firmware.

use epon_olt::mpcp;
use epon_olt::types::MacAddr;

/// Base of the template block the firmware programs.
const TEMPLATE_BASE: u32 = 0x0100_04D8;
/// Length and time offset: length in bits [31:16], offset in bits [7:0].
const OFF_LENGTH_AND_OFFSET: u32 = 0x00;
/// Total size the MAC sends, four bytes past the announced length.
const OFF_TOTAL: u32 = 0x54;
/// The time offset carries this bit set; it is not part of the value.
const OFFSET_MARKER: u8 = 0x80;
/// Shortest frame that still carries an Ethernet header, an opcode and a
/// timestamp. A template announcing less describes no MPCPDU.
const MIN_MPCPDU_LEN: u16 = 20;

/// The template as the firmware left it.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Template {
    /// Frame length the firmware announced, in bytes.
    pub length: u16,
    /// Offset from the granted start, with the marker bit stripped.
    pub time_offset: u8,
    /// Total the MAC sends, read back from the block.
    pub total: u16,
    /// True once both words have been written.
    programmed: bool,
}

impl Template {
    /// True when the firmware has programmed a template the MAC could send.
    pub fn is_armed(&self) -> bool {
        self.programmed && self.length >= MIN_MPCPDU_LEN
    }

    /// Observe a write to the template block. Returns true when the address
    /// belonged to it.
    pub fn observe_write(&mut self, addr: u32, val: u32) -> bool {
        match addr.checked_sub(TEMPLATE_BASE) {
            Some(OFF_LENGTH_AND_OFFSET) => {
                self.length = (val >> 16) as u16;
                self.time_offset = (val as u8) & !OFFSET_MARKER;
                self.programmed = true;
                true
            }
            Some(OFF_TOTAL) => {
                self.total = val as u16;
                true
            }
            _ => false,
        }
    }

    /// Build the REPORT this template describes.
    ///
    /// Header only. The body — queue count, thresholds — is not modelled
    /// anywhere, and inventing one would put a guess where a measurement
    /// belongs. What the far end checks first is the opcode, and that is
    /// carried here.
    pub fn build(&self, src: MacAddr, dst: MacAddr, timestamp: u32) -> Option<Vec<u8>> {
        if !self.is_armed() {
            return None;
        }
        // The length the firmware announced is the one field of a REPORT
        // that has a witness: it computed it and wrote it here.
        let mut frame = mpcp::report_with_length(
            mpcp::Header { dst, src, opcode: mpcp::Opcode::Report, timestamp },
            self.length,
        );
        frame.resize(frame.len().max(self.length as usize), 0);
        Some(frame)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The values a firmware boot actually writes: length 64, offset 0x1B
    /// behind the marker bit, total 68.
    const OBSERVED_WORD: u32 = 0x0040_009B;
    const OBSERVED_TOTAL: u32 = 0x44;

    #[test]
    fn the_template_decodes_what_a_boot_writes() {
        let mut t = Template::default();
        assert!(t.observe_write(TEMPLATE_BASE, OBSERVED_WORD));
        assert!(t.observe_write(TEMPLATE_BASE + OFF_TOTAL, OBSERVED_TOTAL));

        assert_eq!(t.length, 64);
        assert_eq!(t.time_offset, 0x1B, "the marker bit is not part of the value");
        assert_eq!(t.total, 68);
        assert_eq!(t.total, t.length + 4, "the total runs four past the length");
        assert!(t.is_armed());
    }

    #[test]
    fn an_unprogrammed_template_sends_nothing() {
        let t = Template::default();
        assert!(!t.is_armed());
        assert!(t.build(MacAddr::ZERO, MacAddr::ZERO, 0).is_none());
    }

    #[test]
    fn a_template_too_short_to_be_an_mpcpdu_sends_nothing() {
        let mut t = Template::default();
        t.observe_write(TEMPLATE_BASE, 0x0004_0080);
        assert_eq!(t.length, 4);
        assert!(!t.is_armed());
        assert!(t.build(MacAddr::ZERO, MacAddr::ZERO, 0).is_none());
    }

    #[test]
    fn addresses_outside_the_block_are_left_alone() {
        let mut t = Template::default();
        assert!(!t.observe_write(TEMPLATE_BASE - 4, 0xFFFF_FFFF));
        assert!(!t.observe_write(TEMPLATE_BASE + 0x08, 0xFFFF_FFFF));
        assert!(!t.observe_write(0, 0xFFFF_FFFF));
        assert!(!t.is_armed());
    }

    #[test]
    fn the_built_frame_is_a_report_of_the_announced_length() {
        let onu = MacAddr::new([0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF]);
        let olt = MacAddr::new([0x02, 0, 0, 0, 0, 1]);
        let mut t = Template::default();
        t.observe_write(TEMPLATE_BASE, OBSERVED_WORD);

        let frame = t.build(onu, olt, 0x1234_5678).expect("armed");
        assert_eq!(frame.len(), 64);
        let pdu = mpcp::Pdu::parse(&frame).expect("parses");
        assert_eq!(pdu.header.opcode, mpcp::Opcode::Report);
        assert_eq!(pdu.header.src, onu, "a REPORT travels upstream");
        assert_eq!(pdu.header.timestamp, 0x1234_5678);
    }
}
