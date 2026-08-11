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
//! seven words that move when the port comes up, `0x2D00`, `0x2D04`,
//! `0x2D18` and `0x2D28` are written by software and are plain store
//! here. The other three are never given their value by a write, so the
//! model has to produce them:
//!
//!   * [`REG_RX_STATUS`] — never read and never written by the firmware
//!     traced, and it moves with the far end's mode. Derived from the
//!     receive signal alone, with nothing gating it on the port state.
//!   * [`REG_PORT_STATUS`] — read, never written, in a boot that brought
//!     the port up. Its armed bit is therefore an acknowledgement from
//!     the hardware, which is what the model makes it.
//!   * [`REG_BUSY`] bit 31 — see [`BUSY_BIT`].
//!
//! ⛔ One word of that list is still missing here: `0x01002D0C` reads
//! `0x00008807` on silicon once the port is up, from a pre-arm value of
//! `0x00000003`, and no write produces it. It is a plain store in this
//! model, so it stays at the seeded value. What the value means is not
//! established — it looks like an EtherType and is not one that is
//! known — so it is left as a stated gap rather than a guessed rule.
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

/// The word whose bit 31 the firmware spins on — with a timeout, and
/// returning a hard zero without re-reading the result when the wait
/// expires — before taking a statistics count out of this block.
pub const REG_BUSY: u32 = 0x0100_2D1C;

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

/// Bit 31 of [`REG_BUSY`].
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
}

#[derive(Clone, Copy, Debug, Default, serde::Serialize)]
pub struct UniCounters {
    /// Reads of [`REG_BUSY`] that found bit 31 still set. The firmware's
    /// wait on that bit has a timeout and gives up by returning zero, so
    /// without this number a boot that spins here and a boot that never
    /// looks read the same.
    pub port_reads_while_busy: u64,
    /// Writes to [`REG_BUSY`] that carried bit 31 clear while the
    /// hardware held it set. A refusal nobody counts cannot be told from
    /// a write that was never made.
    pub busy_bit_writes_refused: u64,
}

#[derive(Clone, Debug, serde::Serialize)]
pub struct UniSnapshot {
    pub rx: UniRxSignal,
    pub armed: bool,
    pub rx_status: u32,
    pub port_status: u32,
    pub port_busy: bool,
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
    /// Bit 31 of [`REG_BUSY`], owned by the hardware. Seeded from the
    /// reset table rather than from a literal, so the two cannot drift.
    port_busy: bool,
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

    fn busy_word(&self) -> u32 {
        let stored = self.store[Self::idx(REG_BUSY)] & !BUSY_BIT;
        if self.port_busy {
            stored | BUSY_BIT
        } else {
            stored
        }
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
        let seed = self.store[Self::idx(REG_BUSY)];
        self.port_busy = (seed & BUSY_BIT) != 0;
        self.store[Self::idx(REG_BUSY)] = seed & !BUSY_BIT;
    }

    /// Side-effect-free read. Shared with [`Peripheral::peek_word`];
    /// the only side effect a real read has is the busy counter.
    fn value_at(&self, addr: u32) -> u32 {
        match addr & !3 {
            REG_RX_STATUS => self.rx_status(),
            REG_PORT_STATUS => self.port_status(),
            REG_BUSY => self.busy_word(),
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
        if addr & !3 == REG_BUSY && self.port_busy {
            self.counters.port_reads_while_busy += 1;
        }
        Ok(self.value_at(addr))
    }

    fn peek_word(&self, addr: u32) -> Result<u32, Exception> {
        Ok(self.value_at(addr))
    }

    fn write_word(&mut self, addr: u32, val: u32) -> Result<(), Exception> {
        let word = addr & !3;
        if word == REG_BUSY {
            if self.port_busy && (val & BUSY_BIT) == 0 {
                self.counters.busy_bit_writes_refused += 1;
            }
            self.store[Self::idx(word)] = val & !BUSY_BIT;
            return Ok(());
        }
        // The two derived words keep a slot so a write is not lost to
        // the reader of a dump, but the read path does not consult it:
        // what those words report is the signal and the port state.
        self.store[Self::idx(word)] = val;
        Ok(())
    }

    fn tick(&mut self, _cpu_instructions: u64) {}

    fn reset_cold(&mut self) {
        self.rx = UniRxSignal::None;
        self.armed = false;
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
            port_busy: self.port_busy,
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
                    }
                    Ok(())
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
    fn the_counter_port_busy_bit_is_not_cleared_by_a_write() {
        let mut u = Uni::new();
        u.reset_warm();
        assert_eq!(u.read_word(REG_BUSY).unwrap(), 0x8000_001E);
        u.write_word(REG_BUSY, 0x0000_001E).unwrap();
        assert_eq!(
            u.read_word(REG_BUSY).unwrap(),
            0x8000_001E,
            "bit 31 belongs to the hardware"
        );
        assert_eq!(u.counters.busy_bit_writes_refused, 1);
        assert_eq!(u.counters.port_reads_while_busy, 2);
        // The rest of the word is software's own and round-trips.
        u.write_word(REG_BUSY, 0x0000_00AB).unwrap();
        assert_eq!(u.read_word(REG_BUSY).unwrap(), 0x8000_00AB);
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
        assert_eq!(u.read_word(REG_BUSY).unwrap(), 0x0000_001E);
        assert_eq!(u.counters.port_reads_while_busy, 0);
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

    /// The three derived words reproduce their seed **without storing
    /// it** — which is the claim, and the only part of the eleven that is
    /// not the table read back against itself.
    #[test]
    fn the_derived_words_reproduce_their_seed_without_storing_it() {
        let mut u = Uni::new();
        u.reset_warm();
        // Two of the three are produced whole, so a marker cannot
        // displace them: what comes back is still the measured reading.
        for addr in [REG_RX_STATUS, REG_PORT_STATUS] {
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
        // The third is produced one bit at a time: bit 31 survives a
        // write that clears it, and the bits around it — which are
        // software's — do not.
        assert_eq!(u.read_word(REG_BUSY).unwrap(), seed(REG_BUSY));
        u.write_word(REG_BUSY, 0x0000_0000).unwrap();
        assert_eq!(
            u.read_word(REG_BUSY).unwrap(),
            BUSY_BIT,
            "bit 31 is produced, the rest of the word is stored"
        );
    }

    /// Which words are derived and which are stored, swept rather than
    /// asserted: a marker written to each of the sixteen comes back out
    /// of every word except the two that are produced, and out of every
    /// bit of the busy word except bit 31.
    #[test]
    fn exactly_the_documented_words_are_derived_and_the_rest_are_stored() {
        const MARKER: u32 = 0x5A5A_A5A5; // bit 31 clear, so the busy word
                                         // can show which bit it keeps
        for word in (UNI_BASE..UNI_END).step_by(4) {
            let mut u = Uni::new();
            u.reset_warm();
            u.write_word(word, MARKER).unwrap();
            let back = u.read_word(word).unwrap();
            match word {
                REG_RX_STATUS | REG_PORT_STATUS => assert_eq!(
                    back,
                    seed(word),
                    "{word:#010X} is derived: a write must not reach the reading"
                ),
                REG_BUSY => assert_eq!(
                    back,
                    MARKER | BUSY_BIT,
                    "{word:#010X} stores every bit but the one the hardware owns"
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
                // The derived three answer from a rule, not from the
                // store, so they cannot witness the seeding loop; their
                // own test does that.
                Some(_) if matches!(word, REG_RX_STATUS | REG_PORT_STATUS | REG_BUSY) => {}
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
