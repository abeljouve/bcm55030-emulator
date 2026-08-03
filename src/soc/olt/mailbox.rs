//! The frame-queue mailbox: MMIO decoding, uplink assembly, downlink words.
//!
//! The command/status port is bidirectional. Bit 22 starts a read of a queued
//! downstream frame; bit 21 announces an upstream frame the firmware then
//! pushes word by word through the data port at the same stride.

use epon_olt::types::{EtherType, ETHERNET_HEADER_LEN};

/// LLID bitmap base; one word per stride.
pub const BITMAP_BASE: u32 = 0x0100_1438;
/// Command and status port base.
pub const CMD_STATUS_BASE: u32 = 0x0100_15C0;
/// Data port base.
pub const DATA_BASE: u32 = 0x0100_15C4;
/// Stride between channel blocks.
pub const STRIDE: u32 = 0x200;
/// Number of addressable channel blocks.
pub const BLOCK_COUNT: u32 = 8;

/// Status bit 9: a word is waiting on the data port.
pub const STATUS_DATA_READY: u32 = 0x200;

/// A mailbox queue, addressed by the low byte of a read command.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct Slot(pub u8);

impl Slot {
    /// Control-plane queue: MPCP and OAM share it.
    pub const CONTROL: Self = Self(0x10);
    /// Queue used for frames that are neither control-plane nor EAPOL.
    pub const DATA: Self = Self(0x0F);
    /// EAPOL queue.
    pub const EAPOL: Self = Self(0x00);

    /// Queue a frame is delivered through, chosen by EtherType.
    pub fn for_frame(frame: &[u8]) -> Self {
        match EtherType::of_frame(frame) {
            Some(EtherType::Mpcp | EtherType::SlowProtocol) => Self::CONTROL,
            Some(EtherType::Eapol) => Self::EAPOL,
            _ => Self::DATA,
        }
    }

    /// Index and bit position of this slot in the LLID bitmap.
    pub fn bitmap_position(self) -> (usize, u32) {
        ((self.0 >> 5) as usize, (self.0 & 0x1F) as u32)
    }
}

/// A command written to the command/status port.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Command {
    /// Dequeue a downstream frame from `slot`.
    Read { slot: Slot },
    /// Announce an upstream frame of `len` bytes on `channel`.
    ///
    /// `mpcp` is bit 5. Control frames set it, slow-protocol frames do not,
    /// so it selects the queue rather than the direction: bit 21 alone is
    /// what marks a submission.
    Write { channel: u8, len: usize, mpcp: bool },
}

impl Command {
    const READ_BIT: u32 = 0x0040_0000;
    const WRITE_BIT: u32 = 0x0020_0000;
    /// Bit 5 marks a control-plane submission.
    const MPCP_QUEUE: u32 = 0x20;
    const LEN_SHIFT: u32 = 10;
    const LEN_MASK: u32 = 0x7FF;
    const CHANNEL_MASK: u32 = 0x1F;
    const SLOT_MASK: u32 = 0xFF;

    /// Encode a command word, the inverse of [`Self::decode`].
    pub fn encode(self) -> u32 {
        match self {
            Self::Read { slot } => Self::READ_BIT | (slot.0 as u32 & Self::SLOT_MASK),
            Self::Write { channel, len, mpcp } => {
                Self::WRITE_BIT
                    | if mpcp { Self::MPCP_QUEUE } else { 0 }
                    | (channel as u32 & Self::CHANNEL_MASK)
                    | ((len as u32 & Self::LEN_MASK) << Self::LEN_SHIFT)
            }
        }
    }

    /// Decode a command word. Read takes precedence: it is the only bit the
    /// downstream path acts on, and the two are never set together.
    pub fn decode(val: u32) -> Option<Self> {
        if val & Self::READ_BIT != 0 {
            return Some(Self::Read { slot: Slot((val & Self::SLOT_MASK) as u8) });
        }
        if val & Self::WRITE_BIT != 0 {
            return Some(Self::Write {
                channel: (val & Self::CHANNEL_MASK) as u8,
                len: ((val >> Self::LEN_SHIFT) & Self::LEN_MASK) as usize,
                mpcp: val & Self::MPCP_QUEUE != 0,
            });
        }
        None
    }
}

/// Block index for `addr` within the bank rooted at `base`, when it lands
/// exactly on a stride boundary.
pub fn block_index(addr: u32, base: u32) -> Option<u32> {
    let delta = addr.checked_sub(base)?;
    let idx = delta / STRIDE;
    (delta % STRIDE == 0 && idx < BLOCK_COUNT).then_some(idx)
}

/// True when `addr` belongs to the bitmap, command or data ranges.
pub fn claims(addr: u32) -> bool {
    [BITMAP_BASE, CMD_STATUS_BASE, DATA_BASE]
        .iter()
        .any(|&base| block_index(addr, base).is_some())
}

/// Number of data-port words a frame of `len` bytes is pushed as: the header
/// word on its own, then the remainder rounded up.
pub fn word_count(len: usize) -> usize {
    let total = len + 6;
    let mut n = (total / 4).saturating_sub(2);
    if total % 4 != 0 {
        n += 1;
    }
    1 + n
}

/// Words dropped because a submission never completed.
pub type DropCount = u64;

/// Reassembles an upstream frame from the words the firmware pushes.
#[derive(Clone, Debug, Default)]
pub struct TxAssembler {
    pending: Option<Pending>,
    pub dropped: DropCount,
}

#[derive(Clone, Debug)]
struct Pending {
    data_addr: u32,
    len: usize,
    words_expected: usize,
    words: Vec<u32>,
}

/// Two bytes of alignment padding precede the destination address.
pub const ALIGN_PAD: usize = 2;
/// Abandon a submission that runs past this, rather than growing without end.
const MAX_WORDS: usize = 512;

impl TxAssembler {
    /// Begin a submission announced by `cmd` at command address `addr`.
    /// Frames too short to carry an Ethernet header are ignored.
    pub fn begin(&mut self, addr: u32, len: usize) {
        if len < ETHERNET_HEADER_LEN {
            return;
        }
        let Some(block) = block_index(addr, CMD_STATUS_BASE) else {
            return;
        };
        if self.pending.is_some() {
            self.dropped += 1;
        }
        let words_expected = word_count(len);
        self.pending = Some(Pending {
            data_addr: DATA_BASE + block * STRIDE,
            len,
            words_expected,
            words: Vec::with_capacity(words_expected),
        });
    }

    /// Feed a data-port word. Returns the frame once the submission completes.
    pub fn push_word(&mut self, addr: u32, val: u32) -> Option<Vec<u8>> {
        let pending = self.pending.as_mut().filter(|p| p.data_addr == addr)?;
        pending.words.push(val);
        if pending.words.len() > MAX_WORDS {
            self.pending = None;
            self.dropped += 1;
            return None;
        }
        if pending.words.len() < pending.words_expected {
            return None;
        }
        let pending = self.pending.take()?;
        let raw: Vec<u8> = pending.words.iter().flat_map(|w| w.to_be_bytes()).collect();
        match raw.get(ALIGN_PAD..ALIGN_PAD + pending.len) {
            Some(frame) => Some(frame.to_vec()),
            None => {
                self.dropped += 1;
                None
            }
        }
    }

    /// True while a submission is waiting for more words.
    pub fn is_assembling(&self) -> bool {
        self.pending.is_some()
    }

    pub fn reset(&mut self) {
        self.pending = None;
    }
}

/// Encode a downstream frame into the words the firmware reads back:
/// a status word, then a length field paired with the first two bytes,
/// then the rest four bytes at a time.
pub fn encode_frame(frame: &[u8]) -> Vec<u32> {
    let mut words = Vec::with_capacity(2 + frame.len() / 4);
    words.push(0);
    let len_field = ((frame.len() + 6) as u32 & 0x7FF) << 16;
    let head = frame
        .iter()
        .take(2)
        .fold(0u32, |acc, &b| (acc << 8) | b as u32);
    let head = if frame.len() == 1 { head << 8 } else { head };
    words.push(len_field | head);
    for chunk in frame.get(2..).unwrap_or(&[]).chunks(4) {
        let mut word = 0u32;
        for (i, &b) in chunk.iter().enumerate() {
            word |= (b as u32) << (24 - i * 8);
        }
        words.push(word);
    }
    words
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn read_and_write_commands_decode_apart() {
        assert_eq!(
            Command::decode(0x0040_0010),
            Some(Command::Read { slot: Slot::CONTROL })
        );
        assert_eq!(
            Command::decode(0x0021_0020),
            Some(Command::Write { channel: 0, len: 64, mpcp: true })
        );
        // Bit 5 selects the queue, so a submission without it is still a
        // submission: slow-protocol frames go out that way.
        assert_eq!(
            Command::decode(0x0021_0000),
            Some(Command::Write { channel: 0, len: 64, mpcp: false })
        );
        assert_eq!(Command::decode(0), None);
    }

    #[test]
    fn commands_round_trip_through_encode() {
        for cmd in [
            Command::Read { slot: Slot::CONTROL },
            Command::Read { slot: Slot::EAPOL },
            Command::Write { channel: 0, len: 64, mpcp: true },
            Command::Write { channel: 3, len: 1518, mpcp: false },
        ] {
            assert_eq!(Command::decode(cmd.encode()), Some(cmd));
        }
    }

    #[test]
    fn addresses_decode_only_on_stride_boundaries() {
        assert_eq!(block_index(CMD_STATUS_BASE, CMD_STATUS_BASE), Some(0));
        assert_eq!(block_index(CMD_STATUS_BASE + STRIDE, CMD_STATUS_BASE), Some(1));
        assert_eq!(block_index(CMD_STATUS_BASE + 4, CMD_STATUS_BASE), None);
        assert_eq!(block_index(CMD_STATUS_BASE - 4, CMD_STATUS_BASE), None);
        assert_eq!(
            block_index(CMD_STATUS_BASE + BLOCK_COUNT * STRIDE, CMD_STATUS_BASE),
            None
        );
        assert!(claims(DATA_BASE + STRIDE));
        assert!(!claims(DATA_BASE + 8));
    }

    #[test]
    fn word_count_matches_a_sixty_four_byte_frame() {
        assert_eq!(word_count(64), 17);
    }

    #[test]
    fn assembler_yields_the_frame_without_its_padding() {
        let mut asm = TxAssembler::default();
        let cmd_addr = CMD_STATUS_BASE + STRIDE;
        let data_addr = DATA_BASE + STRIDE;
        asm.begin(cmd_addr, 64);
        assert!(asm.is_assembling());

        let mut out = None;
        for i in 0..word_count(64) {
            // Distinguishable payload: the first word holds the two padding
            // bytes, so the frame must start at the second halfword.
            out = asm.push_word(data_addr, 0x0000_0100 + i as u32);
        }
        let frame = out.expect("completes on the last word");
        assert_eq!(frame.len(), 64);
        assert_eq!(&frame[..2], &[0x01, 0x00]);
        assert_eq!(asm.dropped, 0);
        assert!(!asm.is_assembling());
    }

    #[test]
    fn a_frame_shorter_than_a_header_is_not_assembled() {
        let mut asm = TxAssembler::default();
        asm.begin(CMD_STATUS_BASE, ETHERNET_HEADER_LEN - 1);
        assert!(!asm.is_assembling());
    }

    #[test]
    fn restarting_a_submission_counts_the_abandoned_one() {
        let mut asm = TxAssembler::default();
        asm.begin(CMD_STATUS_BASE, 64);
        asm.push_word(DATA_BASE, 0);
        asm.begin(CMD_STATUS_BASE, 64);
        assert_eq!(asm.dropped, 1);
    }

    #[test]
    fn words_for_another_channel_are_ignored() {
        let mut asm = TxAssembler::default();
        asm.begin(CMD_STATUS_BASE, 64);
        assert!(asm.push_word(DATA_BASE + STRIDE, 0xFFFF_FFFF).is_none());
        assert!(asm.is_assembling());
    }

    #[test]
    fn slots_route_by_ethertype() {
        let mut mpcp = vec![0u8; ETHERNET_HEADER_LEN];
        mpcp[12..14].copy_from_slice(&EtherType::Mpcp.as_u16().to_be_bytes());
        assert_eq!(Slot::for_frame(&mpcp), Slot::CONTROL);

        let mut eapol = mpcp.clone();
        eapol[12..14].copy_from_slice(&EtherType::Eapol.as_u16().to_be_bytes());
        assert_eq!(Slot::for_frame(&eapol), Slot::EAPOL);

        assert_eq!(Slot::for_frame(&[]), Slot::DATA);
        assert_eq!(Slot::CONTROL.bitmap_position(), (0, 16));
    }

    #[test]
    fn encode_frame_carries_the_length_and_the_first_two_bytes() {
        let frame: Vec<u8> = (0..8).collect();
        let words = encode_frame(&frame);
        assert_eq!(words[0], 0);
        assert_eq!(words[1] >> 16, (frame.len() + 6) as u32);
        assert_eq!(words[1] & 0xFFFF, 0x0001);
        assert_eq!(words[2], 0x0203_0405);
    }
}
