//! The user-network-interface Ethernet port: one contiguous block at
//! `0x01002D00..0x01002D40`, sixteen words wide.
//!
//! This is a *port*, not a SerDes lane. The lane that carries it lives in
//! `soc/serdes.rs` at `0x010001C0`; the block modelled here is disjoint
//! from it, and two of its words move with what arrives on the receive
//! pair — a link partner's line coding and whether that partner runs
//! clause 37 auto-negotiation. A lane-mode controller has no business
//! knowing either, which is why this is its own file.
//!
//! Which words are stored and which are produced is not a judgement
//! call: a boot traced over the whole block says who writes what. Of the
//! seven words that move when the port comes up, `0x2D00`, `0x2D04` and
//! `0x2D18` are written by software and are plain store here. The rest
//! are never given their value by a write, so the model produces them:
//!
//!   * [`REG_RX_STATUS`] — never read and never written by the firmware
//!     traced, and it moves with the far end's mode. Derived from the
//!     receive signal alone, with nothing gating it on the port state.
//!   * [`REG_PORT_STATUS`] — read, never written, in a boot that brought
//!     the port up. Its armed bit is therefore an acknowledgement from
//!     the hardware, which is what the model makes it.
//!   * [`REG_LINK_MODE`] — see [`link_mode_word`].
//!   * [`REG_STAT_CMD`] bit 31 — see [`BUSY_BIT`].
//!   * [`REG_STAT_DATA`] — the read side of the statistics window below.
//!
//! ## The statistics window
//!
//! Two of these words are not independent registers: they are the
//! address and data halves of one indexed window onto a counter file.
//! Software writes an index to [`REG_STAT_CMD`], waits for its bit 31 to
//! clear, and takes the value out of [`REG_STAT_DATA`]. Over one traced
//! boot it wrote the command port twenty-one thousand times and read the
//! data port very nearly as often, in that order — which is what says
//! the pair is a window and not two registers that happen to be adjacent.
//!
//! Modelling it as two plain words is what this file used to do, and it
//! makes the data port answer zero for a reason that has nothing to do
//! with the hardware. The window is modelled here so the zero has a
//! cause: nothing in this emulator feeds the counter file, and
//! [`UniEvent::SetStatCounter`] is how a test puts something in it.
//!
//! ⛔ Three stated gaps remain:
//!
//!   * `0x01002D0C` reads `0x00008807` on silicon once the port is up,
//!     from a pre-arm value of `0x00000003`.
//!   * `0x01002D34` reads `0x000041A0` once the port is up, from zero
//!     before — a word the reset table does not name at all.
//!
//!     Both are plain stores here, so both stay where they started and
//!     diverge from silicon on an armed port. They are **not** derived
//!     the way [`REG_LINK_MODE`] is, and the difference is evidence, not
//!     taste: that word has three measured write refusals behind it, so
//!     a reading it produces cannot be one software put there. These two
//!     have no such test, so "the hardware produces it" and "software
//!     wrote it somewhere the trace did not cover" are not yet
//!     separable. What separates them is one experiment — write each on
//!     hardware and read it back — and until it is run, a rule here
//!     would be a guess wearing the same clothes as the rest of the file.
//!
//!   * **Nothing increments the counter file.** A frame crossing this
//!     port moves the harness counters below and no entry of the counter
//!     file, because no measurement names one: nothing observed says
//!     which index counts frames, octets, or anything else. Wiring the
//!     ingress to an index would put a number where a guess is, and
//!     `stats`-style firmware would then print it as a reading. So the
//!     file stays fed only by [`UniEvent::SetStatCounter`], and the
//!     falsifier is one measurement naming one index.
//!
//! ## The receive datapath
//!
//! A frame arrives as [`UniEvent::ReceiveFrame`] and gets past this
//! block only through gates that were measured:
//!
//!   * the port must be **armed** — unarmed, nothing about this port
//!     works, which is the one thing every reading of it agrees on;
//!   * something must be **on the receive pair** — a frame arriving with
//!     [`UniRxSignal::None`] contradicts its own premise, so it is
//!     dropped and counted rather than quietly accepted.
//!
//! What happens after that is **not** this block's decision. The frame
//! goes to the classifier, and only a rule that names a queue sends it
//! anywhere. That is the whole point of routing it rather than
//! delivering it: on hardware, frames pushed into this port were never
//! seen by the firmware, and a model that delivered them anyway would
//! be contradicting silicon to make the emulator look livelier.
//!
//! ⛔ Which classifier instance sees this ingress is **not established**
//! — see `Bank::uni_classifier_binding`, which carries the default and
//! the falsifier.
//!
//! **No firmware hook.** What arms the port and what arrives on the
//! receive pair are inputs ([`UniEvent`]), not something this module
//! infers from a firmware variable. The equivalent shortcut — writing a
//! known SRAM address to push a firmware down a path — has been found in
//! this emulator once before and removed.

use crate::cpu::exception::Exception;
use crate::soc::peripheral::{
    AddressRange, Peripheral, PeripheralError, PeripheralEvent, PeripheralSnapshot,
};

/// The port's MMIO block.
pub const UNI_BASE: u32 = 0x0100_2D00;
pub const UNI_END: u32 = 0x0100_2D40;

const UNI_WORDS: usize = ((UNI_END - UNI_BASE) / 4) as usize;

const UNI_RANGES: &[AddressRange] = &[AddressRange::new(UNI_BASE, UNI_END)];

/// The word that follows what arrives on the receive pair.
pub const REG_RX_STATUS: u32 = 0x0100_2D14;

/// The word that carries both the port state and part of the receive
/// signal.
pub const REG_PORT_STATUS: u32 = 0x0100_2D2C;

/// The statistics window's command port: software writes the index of
/// the counter it wants here, and bit 31 is the hardware's busy flag.
///
/// The firmware spins on that bit — with a timeout, and returning a hard
/// zero without re-reading the result when the wait expires — on both
/// sides of the write.
pub const REG_STAT_CMD: u32 = 0x0100_2D1C;

/// The statistics window's data port: the counter [`REG_STAT_CMD`]
/// selected reads out here.
pub const REG_STAT_DATA: u32 = 0x0100_2D20;

/// The word that carries the port's link mode. Two readings exist, one
/// on each side of arming, and three writes to it were refused.
pub const REG_LINK_MODE: u32 = 0x0100_2D28;

// ---------------------------------------------------------------------------
// Bit layout — one place, next to the encode / decode pair it serves.
// ---------------------------------------------------------------------------

/// Bit 0 of [`REG_RX_STATUS`] is set in every reading taken, including
/// the one with no signal on the receive pair at all. It says nothing
/// about the far end; it is reproduced because it is there.
const RX_PRESENT_ALWAYS: u32 = 1 << 0;

/// Bit 14 of [`REG_RX_STATUS`] follows the presence of clause 37
/// auto-negotiation on the receive pair, and nothing else: it is the one
/// bit that separates the two 1000BASE-X readings from each other.
const RX_AN_DETECTED: u32 = 1 << 14;

/// Four bits of [`REG_RX_STATUS`] that move together when the far end
/// switches to 10G line coding. Which of the four means what is **not**
/// established — no reading splits them — so they are kept as one
/// signature rather than given four names.
const RX_TEN_GIG_CODING: u32 = 0x9C00;

/// Bit 8 of [`REG_PORT_STATUS`] is set in every reading of that word,
/// armed or not.
const PORT_ALWAYS: u32 = 1 << 8;

/// Bit 3 of [`REG_PORT_STATUS`]. Clear in the one reading taken **before
/// the port was armed**, set in all four taken after — the only bit of
/// this word that tracks the port rather than the signal.
///
/// It is the hardware's: over a boot that brought the port up, software
/// read this word and never once wrote it, so no store it makes can be
/// where the bit comes from.
///
/// ⚠ That reading is **post-boot**: this block sits outside the two
/// ranges [`super::mmio_init`] brute-scanned, so its seeds are pre-arm
/// observations, not power-on values. What is measured is *before
/// arming*; nothing here separates that from a value software wrote
/// earlier in the boot.
const PORT_ARMED: u32 = 1 << 3;

/// Bit 4 of [`REG_PORT_STATUS`]. Set with no signal on the receive pair
/// and with 10G line coding at the far end; clear under either
/// 1000BASE-X reading. That is the shape of "no 1G synchronisation",
/// and it is what makes the two readings whose [`REG_RX_STATUS`] words
/// are identical tell each other apart.
const PORT_NO_1G_SYNC: u32 = 1 << 4;

/// Bits 2 and 5 of [`REG_PORT_STATUS`], which appear together in the one
/// reading taken with clause 37 auto-negotiation running and in no
/// other. Two bits, one observation: kept as a pair.
const PORT_AN_STATE: u32 = (1 << 2) | (1 << 5);

/// Bit 31 of [`REG_STAT_CMD`].
///
/// Two readings exist and they disagree. The value the reset table holds
/// has it set, and it stays set across the first hundred reads of a
/// boot. The reading taken after the port was armed has it clear.
///
/// Software cannot be what sets it: over one boot it wrote this word
/// sixty-three thousand times and not once with bit 31 among the bits it
/// wrote. So a read that comes back with the bit set is the hardware
/// answering, and a model that let the word round-trip would end the
/// firmware's wait on the firmware's own store.
///
/// So the model gives the bit to the hardware — no write clears it —
/// and ties the one transition it has to the one event measured
/// alongside it. INFERRED: arming is *when* the bit drops; that it is
/// *why* is not established. Falsifier: a reading of this word taken
/// with the port unarmed and the bit already clear.
const BUSY_BIT: u32 = 1 << 31;

/// Bit 17 of [`REG_STAT_CMD`] selects which of the two counter banks the
/// index addresses.
///
/// OBSERVED in the indices software writes: it writes `0..=0x1E` with
/// this bit clear and then restarts from zero with it set, which is the
/// shape of two banks and not one run of forty-nine.
const STAT_BANK_BIT: u32 = 1 << 17;

/// The index field of [`REG_STAT_CMD`], below the bank bit.
const STAT_INDEX_MASK: u32 = STAT_BANK_BIT - 1;

/// How many counters each bank holds, in bank order.
///
/// OBSERVED from where software restarts its index: it walks `0..=0x1E`
/// in bank 0 and `0..=0x11` in bank 1. Thirty-one then eighteen.
pub const STAT_BANK_LEN: [usize; 2] = [31, 18];

const STAT_COUNT: usize = STAT_BANK_LEN[0] + STAT_BANK_LEN[1];

/// [`REG_LINK_MODE`] with the port unarmed — the reading the reset table
/// holds.
const LINK_MODE_UNARMED: u32 = 0x0000_8000;

/// [`REG_LINK_MODE`] with the port armed.
///
/// ⚠ One armed reading, taken with clause 37 auto-negotiation on the
/// receive pair. Whether this word moves with the signal the way
/// [`REG_PORT_STATUS`] does is **not** established — no reading splits
/// it — so the model ties it to arming alone. A second armed reading of
/// `0x0000_0140` exists, taken after a different arming path; this model
/// does not reproduce it, and what separates the two is unknown.
const LINK_MODE_ARMED: u32 = 0x0000_1140;

/// Which counter a command word selects.
///
/// Out-of-range indices get a variant of their own rather than being
/// clamped: what the hardware answers for one is not established, and a
/// silent clamp would hand back a neighbour's count as if it were real.
#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize)]
pub enum StatSelect {
    Counter { bank: usize, index: usize },
    OutOfRange,
}

impl StatSelect {
    /// Decode a command word. Bit 31 is the hardware's and carries no
    /// selection, so it is masked off before decoding.
    pub fn decode(cmd: u32) -> Self {
        let bank = usize::from(cmd & STAT_BANK_BIT != 0);
        let index = (cmd & STAT_INDEX_MASK) as usize;
        if index < STAT_BANK_LEN[bank] {
            Self::Counter { bank, index }
        } else {
            Self::OutOfRange
        }
    }

    /// True when this names a counter the window actually has. A bank
    /// number from outside the model is checked here too, so a caller
    /// cannot index the length table out of bounds.
    fn in_range(self) -> bool {
        match self {
            Self::Counter { bank, index } => {
                STAT_BANK_LEN.get(bank).is_some_and(|&len| index < len)
            }
            Self::OutOfRange => false,
        }
    }

    /// The command word that selects this counter, as software writes it.
    pub fn encode(self) -> Option<u32> {
        match self {
            Self::Counter { bank, index } if self.in_range() => {
                Some(index as u32 | if bank == 1 { STAT_BANK_BIT } else { 0 })
            }
            _ => None,
        }
    }

    /// Position in one flat counter file, so the two banks can share a
    /// backing array without either one's length leaking into the caller.
    fn flat(self) -> Option<usize> {
        match self {
            Self::Counter { bank, index } if self.in_range() => {
                Some(bank * STAT_BANK_LEN[0] + index)
            }
            _ => None,
        }
    }
}

/// Assemble the link-mode word. Two readings, one on each side of
/// arming, and nothing else measured to gate it on.
fn link_mode_word(armed: bool) -> u32 {
    if armed {
        LINK_MODE_ARMED
    } else {
        LINK_MODE_UNARMED
    }
}

// ---------------------------------------------------------------------------
// The receive signal
// ---------------------------------------------------------------------------

/// What is arriving on the receive pair.
///
/// Four cases, because four were measured. There is no fifth because
/// none was observed, and no variant carries a speed or a duplex — the
/// readings constrain the line coding and the presence of
/// auto-negotiation, and nothing else.
#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize)]
pub enum UniRxSignal {
    /// Nothing on the receive pair.
    None,
    /// 1000BASE-X, with or without clause 37 auto-negotiation.
    BaseX { autoneg: bool },
    /// 10G line coding at the far end.
    TenGigCoding,
}

impl UniRxSignal {
    /// Every inhabitant, in the order the readings were taken.
    pub const ALL: [Self; 4] = [
        Self::None,
        Self::BaseX { autoneg: true },
        Self::BaseX { autoneg: false },
        Self::TenGigCoding,
    ];

    /// The receive-status word this signal produces.
    pub fn rx_status_word(self) -> u32 {
        RX_PRESENT_ALWAYS
            | match self {
                Self::None => 0,
                Self::BaseX { autoneg: false } => 0,
                Self::BaseX { autoneg: true } => RX_AN_DETECTED,
                Self::TenGigCoding => RX_TEN_GIG_CODING,
            }
    }

    /// The signal-dependent part of the port-status word.
    fn port_status_bits(self) -> u32 {
        match self {
            Self::None => PORT_NO_1G_SYNC,
            Self::BaseX { autoneg: false } => 0,
            Self::BaseX { autoneg: true } => PORT_AN_STATE,
            Self::TenGigCoding => PORT_NO_1G_SYNC,
        }
    }

    /// Recover the signal from a pair of words read off an **armed**
    /// port. Returns `None` when the pair is not one this model
    /// produces.
    ///
    /// The pair is needed, not just the receive-status word: two of the
    /// four readings share that word and are separated only by
    /// [`PORT_NO_1G_SYNC`]. Unarmed the port word carries no signal bits
    /// at all, so no decode is offered for it — there is nothing in an
    /// unarmed pair to decode.
    pub fn from_armed_status_words(rx_status: u32, port_status: u32) -> Option<Self> {
        Self::ALL.into_iter().find(|candidate| {
            candidate.rx_status_word() == rx_status
                && port_status_word(*candidate, true) == port_status
        })
    }
}

/// Assemble the port-status word.
///
/// Unarmed the word carries [`PORT_ALWAYS`] alone. That is not a guess
/// about the other bits: the single reading of this word taken with the
/// port unarmed is the one the reset table holds, and that reading is
/// post-boot and pre-arm — see [`super::mmio_init`]. The model
/// reproduces the reading it has and derives the rest only once the port
/// is armed, which is the state all four signal readings were taken in.
fn port_status_word(rx: UniRxSignal, armed: bool) -> u32 {
    if !armed {
        return PORT_ALWAYS;
    }
    PORT_ALWAYS | PORT_ARMED | rx.port_status_bits()
}

/// Every reading taken of the two derived words, with the port armed:
/// `(signal at the far end, REG_RX_STATUS, REG_PORT_STATUS)`.
///
/// These are hardware readings pinned next to the type they constrain,
/// not a fixture: the encode / decode pair above has to reproduce each
/// row in both directions, and the round-trip test is what says so.
pub const MEASURED_ARMED_READINGS: [(UniRxSignal, u32, u32); 4] = [
    (UniRxSignal::None, 0x0000_0001, 0x0000_0118),
    (UniRxSignal::BaseX { autoneg: true }, 0x0000_4001, 0x0000_012C),
    (UniRxSignal::BaseX { autoneg: false }, 0x0000_0001, 0x0000_0108),
    (UniRxSignal::TenGigCoding, 0x0000_9C01, 0x0000_0118),
];

// ---------------------------------------------------------------------------
// Events, counters, snapshot
// ---------------------------------------------------------------------------

/// Inputs to the port. Both are things happening outside the SoC —
/// a link partner, and whatever brings the port out of reset — so both
/// arrive as injected state rather than being inferred from firmware.
#[derive(Clone, Debug)]
pub enum UniEvent {
    SetRxSignal(UniRxSignal),
    SetArmed(bool),
    /// Put a value in the counter file. Nothing in this emulator feeds
    /// it, so this is the only way a read of the data port comes back
    /// non-zero — and having it means a test can tell "the window works
    /// and the counter is zero" from "the window is not modelled".
    SetStatCounter { bank: usize, index: usize, value: u32 },
    /// A frame arrives on the receive pair.
    ReceiveFrame(Vec<u8>),
}

/// How deep the port holds frames the datapath has not taken yet.
///
/// Finite on purpose: a queue that never drops turns a burst into a
/// backlog delivered late, and the loss is counted so a full queue
/// cannot be read as a link that received nothing.
const RX_QUEUE_DEPTH: usize = 8;

#[derive(Clone, Copy, Debug, Default, serde::Serialize)]
pub struct UniCounters {
    /// Reads of [`REG_STAT_CMD`] that found bit 31 still set. The
    /// firmware's wait on that bit has a timeout and gives up by
    /// returning zero, so without this number a boot that spins here and
    /// a boot that never looks read the same.
    pub port_reads_while_busy: u64,
    /// Writes to [`REG_STAT_CMD`] made while the hardware held bit 31.
    ///
    /// The bit is the hardware's and no write takes it away. What the
    /// write *does* carry is a counter index, so this is not a refused
    /// attempt to clear the flag — it is software selecting a counter
    /// before the window said it was ready.
    pub stat_cmd_writes_while_busy: u64,
    /// Writes to [`REG_STAT_CMD`] that named a counter in range.
    pub stat_selects: u64,
    /// Writes to [`REG_STAT_CMD`] whose index is past the end of its
    /// bank. A window driven with an index nobody implements answers
    /// zero, which reads exactly like a counter that never moved.
    pub stat_selects_out_of_range: u64,
    /// Reads of [`REG_STAT_DATA`] taken with no counter selected.
    pub stat_reads_unselected: u64,
    /// Writes to [`REG_LINK_MODE`], which the hardware refuses.
    pub link_mode_writes_refused: u64,

    // The receive datapath. Every frame offered to the port lands in
    // exactly one of these, so the four always add up to `rx_offered`
    // minus what is still queued — a frame that vanishes without a
    // counter is the fault this split exists to make visible.
    /// Frames offered to the port, whatever became of them.
    pub rx_offered: u64,
    /// Dropped because the port was not armed.
    pub rx_dropped_unarmed: u64,
    /// Dropped because nothing was on the receive pair. A frame and
    /// [`UniRxSignal::None`] cannot both be true.
    pub rx_dropped_no_signal: u64,
    /// Dropped because the port's queue was already full.
    pub rx_dropped_queue_full: u64,
    /// Taken past the port's own gates and handed to the datapath.
    pub rx_accepted: u64,
}

#[derive(Clone, Debug, serde::Serialize)]
pub struct UniSnapshot {
    pub rx: UniRxSignal,
    pub armed: bool,
    pub rx_status: u32,
    pub port_status: u32,
    pub link_mode: u32,
    pub port_busy: bool,
    /// What the command port currently selects, if anything.
    pub stat_selected: Option<StatSelect>,
    /// Frames the port has accepted and the datapath has not taken.
    pub rx_queued: usize,
    pub counters: UniCounters,
}

// ---------------------------------------------------------------------------
// The peripheral
// ---------------------------------------------------------------------------

#[derive(Clone)]
pub struct Uni {
    /// Backing store for the fourteen words that are not derived. The
    /// two that are still have a slot: a write lands there, it simply
    /// does not come back out.
    store: [u32; UNI_WORDS],
    rx: UniRxSignal,
    armed: bool,
    /// Bit 31 of [`REG_STAT_CMD`], owned by the hardware. Seeded from
    /// the reset table rather than from a literal, so the two cannot
    /// drift.
    port_busy: bool,
    /// The counter file behind the statistics window. Nothing in this
    /// emulator increments it; see the module note.
    stat_counters: [u32; STAT_COUNT],
    /// What the last command-port write selected.
    stat_selected: Option<StatSelect>,
    /// Frames past the port's gates, waiting for the datapath.
    rx_pending: std::collections::VecDeque<Vec<u8>>,
    pub counters: UniCounters,
}

impl Default for Uni {
    fn default() -> Self {
        Self::new()
    }
}

impl Uni {
    pub fn new() -> Self {
        let mut uni = Self {
            store: [0u32; UNI_WORDS],
            rx: UniRxSignal::None,
            armed: false,
            port_busy: false,
            stat_counters: [0u32; STAT_COUNT],
            stat_selected: None,
            rx_pending: std::collections::VecDeque::new(),
            counters: UniCounters::default(),
        };
        uni.apply_silicon_power_on();
        uni
    }

    #[inline]
    pub fn claims(&self, addr: u32) -> bool {
        (UNI_BASE..UNI_END).contains(&(addr & !3))
    }

    #[inline]
    fn idx(addr: u32) -> usize {
        (((addr & !3) - UNI_BASE) / 4) as usize
    }

    pub fn rx_signal(&self) -> UniRxSignal {
        self.rx
    }

    pub fn armed(&self) -> bool {
        self.armed
    }

    pub fn port_busy(&self) -> bool {
        self.port_busy
    }

    /// The receive-status word as it reads right now.
    pub fn rx_status(&self) -> u32 {
        self.rx.rx_status_word()
    }

    /// The port-status word as it reads right now.
    pub fn port_status(&self) -> u32 {
        port_status_word(self.rx, self.armed)
    }

    /// The link-mode word as it reads right now.
    pub fn link_mode(&self) -> u32 {
        link_mode_word(self.armed)
    }

    /// What the command port currently selects, if anything.
    pub fn stat_selected(&self) -> Option<StatSelect> {
        self.stat_selected
    }

    /// The counter file, in flat order: bank 0 then bank 1.
    pub fn stat_counters(&self) -> &[u32] {
        &self.stat_counters
    }

    /// How many accepted frames are waiting for the datapath.
    pub fn rx_queued(&self) -> usize {
        self.rx_pending.len()
    }

    /// Offer a frame to the receive pair.
    ///
    /// The two gates are the port state and the signal, because those
    /// are the two things about this port that were measured. Nothing
    /// here inspects the frame: what a frame is worth is the
    /// classifier's decision, not the port's.
    fn receive_frame(&mut self, frame: &[u8]) {
        self.counters.rx_offered += 1;
        if !self.armed {
            self.counters.rx_dropped_unarmed += 1;
            return;
        }
        if self.rx == UniRxSignal::None {
            self.counters.rx_dropped_no_signal += 1;
            return;
        }
        if self.rx_pending.len() >= RX_QUEUE_DEPTH {
            self.counters.rx_dropped_queue_full += 1;
            return;
        }
        self.counters.rx_accepted += 1;
        self.rx_pending.push_back(frame.to_vec());
    }

    /// Take the frames the port has accepted.
    ///
    /// Draining rather than reading means a frame leaves this block
    /// once, however often the datapath asks.
    pub fn take_rx_frames(&mut self) -> Vec<Vec<u8>> {
        self.rx_pending.drain(..).collect()
    }

    fn stat_cmd_word(&self) -> u32 {
        let stored = self.store[Self::idx(REG_STAT_CMD)] & !BUSY_BIT;
        if self.port_busy {
            stored | BUSY_BIT
        } else {
            stored
        }
    }

    /// The data port. A selection that names no counter answers zero —
    /// which is what an unimplemented index looks like, and why the read
    /// is counted rather than left silent.
    fn stat_data_word(&self) -> u32 {
        self.stat_selected
            .and_then(StatSelect::flat)
            .map(|i| self.stat_counters[i])
            .unwrap_or(0)
    }

    /// Seed the block from the reset table.
    ///
    /// The method keeps the name its siblings in the other peripherals
    /// carry, but for *this* block the table is not a power-on scan: the
    /// rows are a post-boot, pre-arm reading (see [`super::mmio_init`]).
    fn apply_silicon_power_on(&mut self) {
        self.store.fill(0);
        for &(off, val) in super::mmio_init::SYSREG_INIT_VALUES {
            let abs = 0x0100_0000 + off;
            if (UNI_BASE..UNI_END).contains(&abs) {
                self.store[Self::idx(abs)] = val;
            }
        }
        // The busy bit is the hardware's, so take its seed from the same
        // table the rest of the block comes from and keep it out of the
        // store.
        let seed = self.store[Self::idx(REG_STAT_CMD)];
        self.port_busy = (seed & BUSY_BIT) != 0;
        self.store[Self::idx(REG_STAT_CMD)] = seed & !BUSY_BIT;
        // A reset leaves the window pointing at nothing: the seeded
        // command word is a reading, not a selection software made.
        self.stat_selected = None;
        self.stat_counters = [0u32; STAT_COUNT];
    }

    /// Side-effect-free read. Shared with [`Peripheral::peek_word`];
    /// the only side effects a real read has are the counters.
    fn value_at(&self, addr: u32) -> u32 {
        match addr & !3 {
            REG_RX_STATUS => self.rx_status(),
            REG_PORT_STATUS => self.port_status(),
            REG_LINK_MODE => self.link_mode(),
            REG_STAT_CMD => self.stat_cmd_word(),
            REG_STAT_DATA => self.stat_data_word(),
            word => self.store[Self::idx(word)],
        }
    }
}

impl Peripheral for Uni {
    fn name(&self) -> &'static str {
        "uni"
    }

    fn address_ranges(&self) -> &'static [AddressRange] {
        UNI_RANGES
    }

    fn read_word(&mut self, addr: u32) -> Result<u32, Exception> {
        match addr & !3 {
            REG_STAT_CMD if self.port_busy => self.counters.port_reads_while_busy += 1,
            REG_STAT_DATA if self.stat_selected.and_then(StatSelect::flat).is_none() => {
                self.counters.stat_reads_unselected += 1
            }
            _ => {}
        }
        Ok(self.value_at(addr))
    }

    fn peek_word(&self, addr: u32) -> Result<u32, Exception> {
        Ok(self.value_at(addr))
    }

    fn write_word(&mut self, addr: u32, val: u32) -> Result<(), Exception> {
        let word = addr & !3;
        if word == REG_STAT_CMD {
            if self.port_busy {
                self.counters.stat_cmd_writes_while_busy += 1;
            }
            let select = StatSelect::decode(val);
            match select {
                StatSelect::Counter { .. } => self.counters.stat_selects += 1,
                StatSelect::OutOfRange => self.counters.stat_selects_out_of_range += 1,
            }
            self.stat_selected = Some(select);
            self.store[Self::idx(word)] = val & !BUSY_BIT;
            return Ok(());
        }
        if word == REG_LINK_MODE {
            // Three writes to this word were refused on hardware and
            // none was accepted, so the model refuses all of them. The
            // slot still takes the value, for the reader of a dump.
            self.counters.link_mode_writes_refused += 1;
        }
        // The derived words keep a slot so a write is not lost to the
        // reader of a dump, but the read path does not consult it: what
        // those words report is the signal, the port state, and — for
        // the data port — the counter the window selects.
        self.store[Self::idx(word)] = val;
        Ok(())
    }

    fn tick(&mut self, _cpu_instructions: u64) {}

    fn reset_cold(&mut self) {
        self.rx = UniRxSignal::None;
        self.armed = false;
        self.rx_pending.clear();
        self.counters = UniCounters::default();
        self.apply_silicon_power_on();
    }

    fn reset_warm(&mut self) {
        // The reset table holds one reading for this block; there is no
        // second set of values for a warm start.
        self.reset_cold();
    }

    fn snapshot(&self) -> PeripheralSnapshot {
        PeripheralSnapshot::Uni(UniSnapshot {
            rx: self.rx,
            armed: self.armed,
            rx_status: self.rx_status(),
            port_status: self.port_status(),
            link_mode: self.link_mode(),
            port_busy: self.port_busy,
            stat_selected: self.stat_selected,
            rx_queued: self.rx_pending.len(),
            counters: self.counters,
        })
    }

    fn inject_event(&mut self, event: &PeripheralEvent) -> Result<(), PeripheralError> {
        match event {
            PeripheralEvent::Uni(ev) => match ev {
                UniEvent::SetRxSignal(rx) => {
                    self.rx = *rx;
                    Ok(())
                }
                UniEvent::SetArmed(armed) => {
                    self.armed = *armed;
                    if *armed {
                        self.port_busy = false;
                    } else {
                        // A port taken out of service does not keep the
                        // frames it had in hand; they are counted lost
                        // where they were accepted.
                        self.rx_pending.clear();
                    }
                    Ok(())
                }
                UniEvent::ReceiveFrame(frame) => {
                    self.receive_frame(frame);
                    Ok(())
                }
                UniEvent::SetStatCounter { bank, index, value } => {
                    match (StatSelect::Counter {
                        bank: *bank,
                        index: *index,
                    })
                    .flat()
                    {
                        Some(i) => {
                            self.stat_counters[i] = *value;
                            Ok(())
                        }
                        None => Err(PeripheralError::UnsupportedEvent),
                    }
                }
            },
            _ => Err(PeripheralError::UnsupportedEvent),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn armed_with(rx: UniRxSignal) -> Uni {
        let mut u = Uni::new();
        u.reset_warm();
        u.inject_event(&PeripheralEvent::Uni(UniEvent::SetArmed(true)))
            .unwrap();
        u.inject_event(&PeripheralEvent::Uni(UniEvent::SetRxSignal(rx)))
            .unwrap();
        u
    }

    /// Every measured row, in both directions: the encoder reproduces the
    /// two words, and the decoder gets back the variant they came from.
    #[test]
    fn the_measured_signal_table_is_reproduced_both_ways() {
        for (rx, rx_status, port_status) in MEASURED_ARMED_READINGS {
            let mut u = armed_with(rx);
            assert_eq!(
                u.read_word(REG_RX_STATUS).unwrap(),
                rx_status,
                "receive status for {rx:?}"
            );
            assert_eq!(
                u.read_word(REG_PORT_STATUS).unwrap(),
                port_status,
                "port status for {rx:?}"
            );
            assert_eq!(
                UniRxSignal::from_armed_status_words(rx_status, port_status),
                Some(rx),
                "decoding the pair back"
            );
        }
    }

    /// The negative control the table needs: two of the four rows carry
    /// the same receive-status word, so if the port word did not move
    /// with the signal the model would be reporting one state for two.
    #[test]
    fn no_signal_and_basex_without_autoneg_do_not_read_the_same() {
        let mut none = armed_with(UniRxSignal::None);
        let mut basex = armed_with(UniRxSignal::BaseX { autoneg: false });
        assert_eq!(
            none.read_word(REG_RX_STATUS).unwrap(),
            basex.read_word(REG_RX_STATUS).unwrap(),
            "the receive-status word alone cannot tell them apart"
        );
        assert_ne!(
            none.read_word(REG_PORT_STATUS).unwrap(),
            basex.read_word(REG_PORT_STATUS).unwrap(),
            "and the port-status word is what does"
        );
        assert_eq!(none.read_word(REG_PORT_STATUS).unwrap(), 0x0000_0118);
        assert_eq!(basex.read_word(REG_PORT_STATUS).unwrap(), 0x0000_0108);
    }

    /// The bit that says the port is armed is absent from the one
    /// reading taken before it was.
    #[test]
    fn the_armed_bit_is_absent_from_the_reading_taken_before_arming() {
        let mut u = Uni::new();
        u.reset_warm();
        assert_eq!(u.read_word(REG_PORT_STATUS).unwrap(), PORT_ALWAYS);
        assert_eq!(u.read_word(REG_PORT_STATUS).unwrap() & PORT_ARMED, 0);
        for (rx, _, port_status) in MEASURED_ARMED_READINGS {
            assert_ne!(
                port_status & PORT_ARMED,
                0,
                "{rx:?} was read with the port armed"
            );
        }
    }

    /// A write cannot take the busy bit away from the hardware — and the
    /// reads that find it still set are counted, because the firmware's
    /// wait ends by returning zero either way.
    #[test]
    fn the_command_port_busy_bit_is_not_cleared_by_a_write() {
        let mut u = Uni::new();
        u.reset_warm();
        assert_eq!(u.read_word(REG_STAT_CMD).unwrap(), 0x8000_001E);
        u.write_word(REG_STAT_CMD, 0x0000_001E).unwrap();
        assert_eq!(
            u.read_word(REG_STAT_CMD).unwrap(),
            0x8000_001E,
            "bit 31 belongs to the hardware"
        );
        assert_eq!(u.counters.stat_cmd_writes_while_busy, 1);
        assert_eq!(u.counters.port_reads_while_busy, 2);
        // The rest of the word is the index software wrote, and it
        // round-trips.
        u.write_word(REG_STAT_CMD, 0x0000_0005).unwrap();
        assert_eq!(u.read_word(REG_STAT_CMD).unwrap(), 0x8000_0005);
    }

    /// Arming is the one event measured alongside the bit dropping, and
    /// it is the only thing in the model that drops it.
    #[test]
    fn arming_the_port_is_what_releases_the_busy_bit() {
        let mut u = Uni::new();
        u.reset_warm();
        assert!(u.port_busy());
        u.inject_event(&PeripheralEvent::Uni(UniEvent::SetArmed(true)))
            .unwrap();
        assert_eq!(u.read_word(REG_STAT_CMD).unwrap(), 0x0000_001E);
        assert_eq!(u.counters.port_reads_while_busy, 0);
    }

    // -----------------------------------------------------------------
    // The statistics window
    // -----------------------------------------------------------------

    /// Every counter the window can name, encoded and decoded back.
    #[test]
    fn a_selection_round_trips_through_the_command_word() {
        for (bank, &len) in STAT_BANK_LEN.iter().enumerate() {
            for index in 0..len {
                let sel = StatSelect::Counter { bank, index };
                let word = sel.encode().expect("in range");
                assert_eq!(StatSelect::decode(word), sel, "bank {bank} index {index}");
            }
            // One past the end of each bank is not a counter, and the
            // encoder refuses to name it.
            let past = StatSelect::Counter { bank, index: len };
            assert_eq!(past.encode(), None);
            assert_eq!(
                StatSelect::decode(len as u32 | if bank == 1 { STAT_BANK_BIT } else { 0 }),
                StatSelect::OutOfRange
            );
        }
    }

    /// The bank bit is what separates the two runs: index 0 of bank 1 is
    /// not index 0 of bank 0.
    #[test]
    fn the_bank_bit_selects_a_different_counter_at_the_same_index() {
        let mut u = armed_with(UniRxSignal::BaseX { autoneg: true });
        for (bank, value) in [(0usize, 0x1111_1111u32), (1, 0x2222_2222)] {
            u.inject_event(&PeripheralEvent::Uni(UniEvent::SetStatCounter {
                bank,
                index: 0,
                value,
            }))
            .unwrap();
        }
        for (bank, expected) in [(0usize, 0x1111_1111u32), (1, 0x2222_2222)] {
            let cmd = StatSelect::Counter { bank, index: 0 }.encode().unwrap();
            u.write_word(REG_STAT_CMD, cmd).unwrap();
            assert_eq!(u.read_word(REG_STAT_DATA).unwrap(), expected, "bank {bank}");
        }
    }

    /// The whole point of modelling the window: the data port answers
    /// with the counter the command port named, so a zero there means
    /// the counter is zero and not that the window is missing.
    #[test]
    fn the_data_port_answers_with_the_selected_counter() {
        let mut u = armed_with(UniRxSignal::BaseX { autoneg: true });
        u.inject_event(&PeripheralEvent::Uni(UniEvent::SetStatCounter {
            bank: 0,
            index: 7,
            value: 0xDEAD_BEEF,
        }))
        .unwrap();

        // Nothing selected yet: zero, and the read is counted so it
        // cannot be mistaken for a counter that never moved.
        assert_eq!(u.read_word(REG_STAT_DATA).unwrap(), 0);
        assert_eq!(u.counters.stat_reads_unselected, 1);

        u.write_word(REG_STAT_CMD, StatSelect::Counter { bank: 0, index: 7 }.encode().unwrap())
            .unwrap();
        assert_eq!(u.read_word(REG_STAT_DATA).unwrap(), 0xDEAD_BEEF);
        assert_eq!(u.counters.stat_selects, 1);
        assert_eq!(u.counters.stat_reads_unselected, 1, "that read was selected");

        // A neighbour the test never wrote still reads zero, so the
        // window is not just handing back the last value it saw.
        u.write_word(REG_STAT_CMD, StatSelect::Counter { bank: 0, index: 8 }.encode().unwrap())
            .unwrap();
        assert_eq!(u.read_word(REG_STAT_DATA).unwrap(), 0);
    }

    /// An index past the end of its bank answers zero — which is what a
    /// counter that never moved answers too, so it is counted apart.
    #[test]
    fn an_index_past_the_end_of_its_bank_is_counted_not_clamped() {
        let mut u = armed_with(UniRxSignal::BaseX { autoneg: true });
        u.inject_event(&PeripheralEvent::Uni(UniEvent::SetStatCounter {
            bank: 0,
            index: STAT_BANK_LEN[0] - 1,
            value: 0x5555_5555,
        }))
        .unwrap();
        u.write_word(REG_STAT_CMD, STAT_BANK_LEN[0] as u32).unwrap();
        assert_eq!(u.stat_selected(), Some(StatSelect::OutOfRange));
        assert_eq!(
            u.read_word(REG_STAT_DATA).unwrap(),
            0,
            "it must not fall back on the last counter of the bank"
        );
        assert_eq!(u.counters.stat_selects_out_of_range, 1);
        assert_eq!(u.counters.stat_selects, 0);
        assert_eq!(u.counters.stat_reads_unselected, 1);
    }

    /// Injecting into a bank the model does not have is refused rather
    /// than panicking on the length table.
    #[test]
    fn injecting_outside_the_counter_file_is_refused() {
        let mut u = Uni::new();
        for (bank, index) in [(2usize, 0usize), (0, STAT_BANK_LEN[0]), (1, STAT_BANK_LEN[1])] {
            assert!(u
                .inject_event(&PeripheralEvent::Uni(UniEvent::SetStatCounter {
                    bank,
                    index,
                    value: 1,
                }))
                .is_err());
        }
    }

    /// A reset leaves the window pointing at nothing and the file empty,
    /// so a counter injected by one test cannot survive into the next.
    #[test]
    fn a_reset_empties_the_counter_file_and_the_selection() {
        let mut u = armed_with(UniRxSignal::None);
        u.inject_event(&PeripheralEvent::Uni(UniEvent::SetStatCounter {
            bank: 1,
            index: 3,
            value: 0x1234_5678,
        }))
        .unwrap();
        u.write_word(REG_STAT_CMD, StatSelect::Counter { bank: 1, index: 3 }.encode().unwrap())
            .unwrap();
        assert_eq!(u.read_word(REG_STAT_DATA).unwrap(), 0x1234_5678);
        u.reset_cold();
        assert_eq!(u.stat_selected(), None);
        assert!(u.stat_counters().iter().all(|&c| c == 0));
        assert_eq!(u.read_word(REG_STAT_DATA).unwrap(), 0);
    }

    // -----------------------------------------------------------------
    // The receive datapath
    // -----------------------------------------------------------------

    fn frame(len: usize) -> Vec<u8> {
        (0..len).map(|i| i as u8).collect()
    }

    fn offer(u: &mut Uni, f: Vec<u8>) {
        u.inject_event(&PeripheralEvent::Uni(UniEvent::ReceiveFrame(f)))
            .unwrap();
    }

    /// The two gates, each on its own, and the accepted case that says
    /// the gates are gates and not a closed door.
    #[test]
    fn a_frame_needs_an_armed_port_and_something_on_the_pair() {
        // Unarmed, with a signal: refused on the port state.
        let mut u = Uni::new();
        u.inject_event(&PeripheralEvent::Uni(UniEvent::SetRxSignal(
            UniRxSignal::BaseX { autoneg: true },
        )))
        .unwrap();
        offer(&mut u, frame(64));
        assert_eq!(u.counters.rx_dropped_unarmed, 1);
        assert_eq!(u.rx_queued(), 0);

        // Armed, with nothing on the pair: a frame contradicts its own
        // premise, so it is refused on the signal.
        let mut u = armed_with(UniRxSignal::None);
        offer(&mut u, frame(64));
        assert_eq!(u.counters.rx_dropped_no_signal, 1);
        assert_eq!(u.rx_queued(), 0);

        // Both gates open: taken.
        let mut u = armed_with(UniRxSignal::BaseX { autoneg: true });
        offer(&mut u, frame(64));
        assert_eq!(u.counters.rx_accepted, 1);
        assert_eq!(u.rx_queued(), 1);
        assert_eq!(u.take_rx_frames(), vec![frame(64)]);
        assert_eq!(u.rx_queued(), 0, "draining takes the frame once");
    }

    /// Every frame offered lands in exactly one counter. A frame that
    /// vanishes without one is the fault this test exists to catch.
    #[test]
    fn every_offered_frame_is_accounted_for() {
        let mut u = armed_with(UniRxSignal::BaseX { autoneg: false });
        for _ in 0..RX_QUEUE_DEPTH + 3 {
            offer(&mut u, frame(64));
        }
        u.inject_event(&PeripheralEvent::Uni(UniEvent::SetRxSignal(
            UniRxSignal::None,
        )))
        .unwrap();
        offer(&mut u, frame(64));

        let c = u.counters;
        assert_eq!(c.rx_offered, RX_QUEUE_DEPTH as u64 + 4);
        assert_eq!(
            c.rx_accepted + c.rx_dropped_unarmed + c.rx_dropped_no_signal + c.rx_dropped_queue_full,
            c.rx_offered,
            "the four outcomes must add up to what was offered"
        );
        assert_eq!(c.rx_dropped_queue_full, 3);
        assert_eq!(c.rx_dropped_no_signal, 1);
        assert_eq!(u.rx_queued(), RX_QUEUE_DEPTH);
    }

    /// A port taken out of service does not keep what it was holding.
    #[test]
    fn disarming_the_port_lets_go_of_the_queued_frames() {
        let mut u = armed_with(UniRxSignal::BaseX { autoneg: true });
        offer(&mut u, frame(64));
        assert_eq!(u.rx_queued(), 1);
        u.inject_event(&PeripheralEvent::Uni(UniEvent::SetArmed(false)))
            .unwrap();
        assert_eq!(u.rx_queued(), 0);
        assert_eq!(u.take_rx_frames(), Vec::<Vec<u8>>::new());
    }

    /// ⭐ The receive path moves no entry of the counter file. Nothing
    /// measured says which index counts a frame, so putting one there
    /// would be a guess that firmware would print as a reading.
    #[test]
    fn a_received_frame_moves_no_entry_of_the_counter_file() {
        let mut u = armed_with(UniRxSignal::BaseX { autoneg: true });
        for _ in 0..5 {
            offer(&mut u, frame(64));
        }
        let _ = u.take_rx_frames();
        assert!(
            u.stat_counters().iter().all(|&c| c == 0),
            "no index is known to count frames, so none may be made to"
        );
        assert_eq!(u.counters.rx_accepted, 5, "the harness counter is what moved");
    }

    // -----------------------------------------------------------------
    // The link-mode word
    // -----------------------------------------------------------------

    /// Both readings, and the refusal that says the word is not a store.
    #[test]
    fn the_link_mode_word_follows_arming_and_refuses_writes() {
        let mut u = Uni::new();
        u.reset_warm();
        assert_eq!(u.read_word(REG_LINK_MODE).unwrap(), LINK_MODE_UNARMED);

        u.write_word(REG_LINK_MODE, 0x0000_5140).unwrap();
        assert_eq!(
            u.read_word(REG_LINK_MODE).unwrap(),
            LINK_MODE_UNARMED,
            "three writes to this word were refused on hardware"
        );
        assert_eq!(u.counters.link_mode_writes_refused, 1);

        u.inject_event(&PeripheralEvent::Uni(UniEvent::SetArmed(true)))
            .unwrap();
        assert_eq!(u.read_word(REG_LINK_MODE).unwrap(), LINK_MODE_ARMED);
        u.write_word(REG_LINK_MODE, 0x0000_0000).unwrap();
        assert_eq!(u.read_word(REG_LINK_MODE).unwrap(), LINK_MODE_ARMED);
        assert_eq!(u.counters.link_mode_writes_refused, 2);
    }

    /// The unarmed reading is the table's, not a literal that can drift
    /// away from it.
    #[test]
    fn the_unarmed_link_mode_is_the_seeded_reading() {
        assert_eq!(LINK_MODE_UNARMED, seed(REG_LINK_MODE));
    }

    /// Every word of this block the reset table names, in address order.
    fn seeded_words() -> Vec<(u32, u32)> {
        crate::soc::mmio_init::SYSREG_INIT_VALUES
            .iter()
            .map(|&(off, val)| (0x0100_0000 + off, val))
            .filter(|(abs, _)| (UNI_BASE..UNI_END).contains(abs))
            .collect()
    }

    /// The reading the table holds for one word of this block.
    fn seed(addr: u32) -> u32 {
        seeded_words()
            .into_iter()
            .find(|&(abs, _)| abs == addr)
            .map(|(_, val)| val)
            .unwrap_or_else(|| panic!("the reset table names no {addr:#010X}"))
    }

    /// The derived words reproduce their seed **without storing it** —
    /// which is the claim, and the only part of the eleven that is not
    /// the table read back against itself.
    #[test]
    fn the_derived_words_reproduce_their_seed_without_storing_it() {
        let mut u = Uni::new();
        u.reset_warm();
        // These are produced whole, so a marker cannot displace them:
        // what comes back is still the measured reading.
        for addr in [REG_RX_STATUS, REG_PORT_STATUS, REG_LINK_MODE] {
            assert_eq!(
                u.read_word(addr).unwrap(),
                seed(addr),
                "seed at {addr:#010X}"
            );
            u.write_word(addr, 0xFFFF_FFFF).unwrap();
            assert_eq!(
                u.read_word(addr).unwrap(),
                seed(addr),
                "a write cannot become the reading at {addr:#010X}"
            );
        }
        // The command port is produced one bit at a time: bit 31
        // survives a write that clears it, and the index bits around it
        // — which are software's — do not.
        assert_eq!(u.read_word(REG_STAT_CMD).unwrap(), seed(REG_STAT_CMD));
        u.write_word(REG_STAT_CMD, 0x0000_0000).unwrap();
        assert_eq!(
            u.read_word(REG_STAT_CMD).unwrap(),
            BUSY_BIT,
            "bit 31 is produced, the index is stored"
        );
    }

    /// Which words are derived and which are stored, swept rather than
    /// asserted: a marker written to each of the sixteen comes back out
    /// of every word the model does not produce.
    #[test]
    fn exactly_the_documented_words_are_derived_and_the_rest_are_stored() {
        // Bit 31 clear, so the command word can show which bit it keeps,
        // and an index field that names no counter, so the data port
        // stays unselected.
        const MARKER: u32 = 0x5A5A_A5A5;
        for word in (UNI_BASE..UNI_END).step_by(4) {
            let mut u = Uni::new();
            u.reset_warm();
            u.write_word(word, MARKER).unwrap();
            let back = u.read_word(word).unwrap();
            match word {
                REG_RX_STATUS | REG_PORT_STATUS | REG_LINK_MODE => assert_eq!(
                    back,
                    seed(word),
                    "{word:#010X} is derived: a write must not reach the reading"
                ),
                REG_STAT_CMD => assert_eq!(
                    back,
                    MARKER | BUSY_BIT,
                    "{word:#010X} stores the index but not the bit the hardware owns"
                ),
                REG_STAT_DATA => assert_eq!(
                    back, 0,
                    "{word:#010X} is the read side of the window, never a store"
                ),
                _ => assert_eq!(back, MARKER, "{word:#010X} is a plain store"),
            }
        }
    }

    /// Coverage, not values: every row the reset table holds for this
    /// block reaches the block, and no word outside those rows is
    /// touched.
    ///
    /// ⚠ Read this for what it is. Comparing a seeded word against the
    /// table it was seeded from says **nothing** about whether the value
    /// is the silicon's — that claim rests on the hardware reading
    /// [`super::mmio_init`] records, not on this test. What it does catch
    /// is a seeding path that silently drops or misplaces rows: skip one
    /// row of the eight plain-storage words and every other test in this
    /// file still passes, because the three derived words produce their
    /// readings without consulting the store.
    #[test]
    fn the_reset_table_seeds_eleven_words_of_this_block_and_no_others() {
        let mut u = Uni::new();
        u.reset_warm();
        let seeded = seeded_words();
        assert_eq!(
            seeded.len(),
            11,
            "the reset table seeds eleven words of this block"
        );
        for word in (UNI_BASE..UNI_END).step_by(4) {
            match seeded.iter().find(|&&(abs, _)| abs == word) {
                // The derived words answer from a rule, not from the
                // store, so they cannot witness the seeding loop; their
                // own test does that.
                Some(_)
                    if matches!(
                        word,
                        REG_RX_STATUS | REG_PORT_STATUS | REG_LINK_MODE | REG_STAT_CMD
                    ) => {}
                Some(&(_, val)) => assert_eq!(
                    u.read_word(word).unwrap(),
                    val,
                    "the seeding loop skipped the row for {word:#010X}"
                ),
                None => assert_eq!(
                    u.read_word(word).unwrap(),
                    0,
                    "word the table does not name: {word:#010X}"
                ),
            }
        }
    }

    /// The derived words are derived, not stored: a write to either
    /// changes nothing about what comes back.
    #[test]
    fn a_write_to_a_derived_status_word_changes_no_reading() {
        let mut u = armed_with(UniRxSignal::BaseX { autoneg: true });
        u.write_word(REG_RX_STATUS, 0xFFFF_FFFF).unwrap();
        u.write_word(REG_PORT_STATUS, 0xFFFF_FFFF).unwrap();
        assert_eq!(u.read_word(REG_RX_STATUS).unwrap(), 0x0000_4001);
        assert_eq!(u.read_word(REG_PORT_STATUS).unwrap(), 0x0000_012C);
    }

    /// A word this block does not derive is a plain store, at every
    /// access width the trait offers.
    #[test]
    fn the_plain_store_round_trips_at_every_width() {
        let mut u = Uni::new();
        u.reset_cold();
        u.write_word(0x0100_2D3C, 0x1122_3344).unwrap();
        assert_eq!(u.read_word(0x0100_2D3C).unwrap(), 0x1122_3344);
        assert_eq!(u.read_half(0x0100_2D3C).unwrap(), 0x1122);
        assert_eq!(u.read_half(0x0100_2D3E).unwrap(), 0x3344);
        assert_eq!(u.read_byte(0x0100_2D3F).unwrap(), 0x44);
        u.write_byte(0x0100_2D3C, 0xAA).unwrap();
        assert_eq!(u.read_word(0x0100_2D3C).unwrap(), 0xAA22_3344);
        u.write_half(0x0100_2D3E, 0xBEEF).unwrap();
        assert_eq!(u.read_word(0x0100_2D3C).unwrap(), 0xAA22_BEEF);
    }

    #[test]
    fn claims_cover_the_block_and_nothing_else() {
        let u = Uni::new();
        assert!(u.claims(UNI_BASE));
        assert!(u.claims(0x0100_2D3F));
        assert!(!u.claims(0x0100_2CFC));
        assert!(!u.claims(UNI_END));
    }

    #[test]
    fn an_unknown_status_pair_does_not_decode() {
        assert_eq!(
            UniRxSignal::from_armed_status_words(0x0000_0001, 0x0000_0100),
            None,
            "an unarmed port word carries no signal to recover"
        );
        assert_eq!(
            UniRxSignal::from_armed_status_words(0xDEAD_BEEF, 0x0000_0118),
            None
        );
    }

    #[test]
    fn a_cold_reset_puts_the_port_back_where_it_started() {
        let mut u = armed_with(UniRxSignal::TenGigCoding);
        u.write_word(0x0100_2D3C, 0xFFFF_FFFF).unwrap();
        u.reset_cold();
        assert_eq!(u.rx_signal(), UniRxSignal::None);
        assert!(!u.armed());
        assert!(u.port_busy());
        assert_eq!(u.read_word(0x0100_2D3C).unwrap(), 0);
        assert_eq!(u.read_word(REG_PORT_STATUS).unwrap(), PORT_ALWAYS);
    }
}
