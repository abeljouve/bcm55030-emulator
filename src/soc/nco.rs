//! The 16-slot low-memory table that doubles as the ARC interrupt
//! vector table, plus one word at `0x01000F80`.
//!
//! ⛔ The module keeps its `nco` name for its consumers, but the name is
//! the weakest thing in it. The table this module's slots model holds
//! `j @<limm>` instruction pairs — that reading is anchored on live
//! silicon, where slot 0 reads `2020 0f80 0000 0150`, an ARCompact jump
//! to `0x150`. An oscillator's frequency word and a jump target cannot
//! be the same four bytes, so the per-field oscillator names this file
//! used to carry (`CONFIG` / `PRESCALE` / `FREQ_HI` / `FREQ_LO`) are
//! not describing what is stored. They are recorded here only as the
//! names the register DB uses for the same slot, which calls it an NCO
//! channel array — and separately notes that reading as mislabelled.
//!
//! ⛔ `0x01000F80` has **three** claimants in the register DB and no
//! measurement separates them: a MACsec channel-enable word (block 15),
//! a secondary TX-mode word with a bit 9 (block 23), and a lane-busy
//! status bit 3 (block 25). This module serves it as a plain backing
//! store, which is the only behaviour all three readings agree on. The
//! bit table this doc used to print asserted the second reading as
//! fact; the constant is `REG_NCO_TX_MODE` for its call sites, not
//! because the name is established.
//!
//! `CLK_READY_FLAG` at `0x00FC1017` is not claimed here because the warm
//! boot path never touches it; it can migrate in when a firmware path
//! that reads it is identified.

use crate::cpu::exception::Exception;
use crate::soc::peripheral::{AddressRange, Peripheral, PeripheralSnapshot};

/// The one MMIO word this peripheral claims. ⛔ Three blocks of the
/// register DB name it differently and nothing measured separates them
/// — see the module doc. The constant keeps its name for its call
/// sites; served as a plain backing store, which is what all three
/// readings have in common.
pub const REG_NCO_TX_MODE: u32 = 0x0100_0F80;

const NCO_RANGES: &[AddressRange] =
    &[AddressRange::new(REG_NCO_TX_MODE, REG_NCO_TX_MODE + 4)];

/// Number of NCO / ARC interrupt-vector channels.
pub const NCO_CHANNELS: usize = 16;

/// Each channel occupies 8 bytes in the low-memory aperture.
pub const NCO_SLOT_BYTES: u32 = 8;

/// Top of the NCO/IVT aperture when `AUX_INT_VECTOR_BASE` (AUX 0x25)
/// is 0 — the only base ever observed on this silicon (the bootloader
/// and the firmware never write AUX 0x25). `0..0x80` = 16 channels × 8
/// bytes.
pub const NCO_IVT_LIMIT: u32 = (NCO_CHANNELS as u32) * NCO_SLOT_BYTES;

/// The ARCompact `j @<limm>` opcode word that prefixes every
/// programmed slot (big-endian on-chip = bytes `20 20 0f 80`). When a
/// slot's first four bytes equal this, the channel is an installed
/// interrupt vector and the trailing 32-bit word is the absolute jump
/// target. Live-silicon anchor: slot 0 = `2020 0f80 0000 0150`,
/// i.e. `j @0x150`. This is the measured half of the module; the
/// oscillator reading of the same bytes is the unmeasured half.
pub const NCO_J_LIMM_OPCODE: u32 = 0x2020_0F80;

/// One slot of the 16-entry table. The hardware aliases it over the
/// low-memory ARC interrupt-vector range on a separate physical bus:
/// `.di` (uncached) stores land here, while plain CPU reads and
/// instruction fetch see SRAM. The ARC interrupt unit fetches its
/// vector from this table, not from SRAM — live-silicon slot 0 reads
/// `j @0x150`.
#[derive(Clone, Copy)]
pub struct NcoSlot {
    /// Raw 8 bytes exactly as written by the four `stw.di` halfword
    /// stores: `+0 opcode_hi`, `+2 opcode_lo`, `+4 target_hi`,
    /// `+6 target_lo`. ⛔ The register DB names the same four fields
    /// `CONFIG` / `PRESCALE` / `FREQ_HI` / `FREQ_LO`, i.e. an
    /// oscillator's configuration and frequency. Those names cannot be
    /// right for a slot that decodes as a jump, and nothing in this
    /// module treats the bytes as a frequency.
    pub raw: [u8; 8],
    /// Any `.di` field has been written to this slot since reset.
    pub written: bool,
}

impl NcoSlot {
    const fn empty() -> Self {
        Self { raw: [0; 8], written: false }
    }

    /// High halfword of the opcode pair (`raw[0..2]`, big-endian) — the
    /// register DB calls this field `CONFIG`.
    #[inline]
    pub fn config(&self) -> u16 {
        ((self.raw[0] as u16) << 8) | self.raw[1] as u16
    }

    /// The absolute 32-bit jump target (`raw[4..8]`, big-endian) — the
    /// long-immediate operand of the `j @<limm>` in the first four
    /// bytes. ⛔ The register DB reads the same two halfwords as a
    /// frequency word split high/low.
    #[inline]
    pub fn target(&self) -> u32 {
        ((self.raw[4] as u32) << 24)
            | ((self.raw[5] as u32) << 16)
            | ((self.raw[6] as u32) << 8)
            | self.raw[7] as u32
    }

    /// True when this channel holds an installed ARC interrupt
    /// vector: the slot has been programmed and its opcode prefix is
    /// the `j @<limm>` pair. Channels left as their reset zero (or
    /// programmed only as clock NCOs without the vector opcode) are
    /// not valid interrupt vectors.
    #[inline]
    pub fn is_installed_vector(&self) -> bool {
        self.written
            && (((self.config() as u32) << 16)
                | (((self.raw[2] as u32) << 8) | self.raw[3] as u32))
                == NCO_J_LIMM_OPCODE
    }
}

#[derive(Clone)]
pub struct Nco {
    tx_mode: u32,
    /// 16-channel NCO table aliased over the ARC interrupt-vector
    /// range. Written only by `.di` stores routed here from the
    /// low-memory aperture (see `Memory::write_*_data`).
    slots: [NcoSlot; NCO_CHANNELS],
    pub trace: bool,
}

impl Nco {
    pub fn new() -> Self {
        Self {
            tx_mode: 0,
            slots: [NcoSlot::empty(); NCO_CHANNELS],
            trace: false,
        }
    }

    #[inline]
    pub fn claims(&self, addr: u32) -> bool {
        (REG_NCO_TX_MODE..REG_NCO_TX_MODE + 4).contains(&addr)
    }

    /// Is `addr` inside the NCO/IVT low-memory aperture
    /// (`0..NCO_IVT_LIMIT`, base AUX 0x25 = 0)?
    #[inline]
    pub fn in_ivt_aperture(addr: u32) -> bool {
        addr < NCO_IVT_LIMIT
    }

    /// Route a `.di` (uncached) store in the IVT aperture to the NCO
    /// table. `addr` is the CPU address (`0..0x80`); `size` is 1/2/4.
    /// Writes are big-endian, matching the on-chip byte order and the
    /// `stw.di` field order the firmware uses.
    pub fn ivt_di_store(&mut self, addr: u32, val: u32, size: u8) {
        let chan = (addr / NCO_SLOT_BYTES) as usize;
        if chan >= NCO_CHANNELS {
            return;
        }
        let off = (addr % NCO_SLOT_BYTES) as usize;
        let slot = &mut self.slots[chan];
        // Per-byte bounds guard: a multi-byte store starting near the slot end
        // (off+size > NCO_SLOT_BYTES) would overrun slot.raw. Real firmware only
        // issues slot-aligned writes, but a fuzzed input can drive a size-2/4
        // store at off=5..7; drop the out-of-slot bytes rather than panic. No
        // cross-slot spill is modelled (unproven on silicon).
        match size {
            1 => {
                slot.raw[off] = val as u8;
            }
            2 => {
                let n = NCO_SLOT_BYTES as usize;
                slot.raw[off] = (val >> 8) as u8;
                if off + 1 < n { slot.raw[off + 1] = val as u8; }
            }
            _ => {
                let n = NCO_SLOT_BYTES as usize;
                slot.raw[off] = (val >> 24) as u8;
                if off + 1 < n { slot.raw[off + 1] = (val >> 16) as u8; }
                if off + 2 < n { slot.raw[off + 2] = (val >> 8) as u8; }
                if off + 3 < n { slot.raw[off + 3] = val as u8; }
            }
        }
        slot.written = true;
        if self.trace {
            eprintln!(
                "[NCO] .di store ch{} +{:#x} = {:#0width$x} (size {}) -> slot {:02x?}",
                chan, off, val, size, slot.raw,
                width = (size as usize) * 2 + 2
            );
        }
    }

    /// Read the raw `count` bytes of the IVT aperture at `addr` — used
    /// by the dual-aperture readback path. Returns `None` when the
    /// channel was never programmed (the caller then falls back to
    /// SRAM, the other half of the aperture).
    pub fn ivt_read(&self, addr: u32, count: usize) -> Option<Vec<u8>> {
        let chan = (addr / NCO_SLOT_BYTES) as usize;
        if chan >= NCO_CHANNELS || !self.slots[chan].written {
            return None;
        }
        let off = (addr % NCO_SLOT_BYTES) as usize;
        let mut out = Vec::with_capacity(count);
        for i in 0..count {
            let a = addr + i as u32;
            let c = (a / NCO_SLOT_BYTES) as usize;
            let o = (a % NCO_SLOT_BYTES) as usize;
            if c >= NCO_CHANNELS || !self.slots[c].written {
                return None;
            }
            let _ = off;
            out.push(self.slots[c].raw[o]);
        }
        Some(out)
    }

    /// The installed ARC interrupt vector for channel `n`, i.e. the
    /// absolute address the `j @<limm>` slot jumps to. `None` when the
    /// channel is not a programmed `j @limm` vector.
    pub fn interrupt_vector(&self, n: u8) -> Option<u32> {
        let s = self.slots.get(n as usize)?;
        if s.is_installed_vector() {
            Some(s.target())
        } else {
            None
        }
    }

    /// Snapshot of every channel for the UI / introspection.
    pub fn ivt_slots(&self) -> &[NcoSlot; NCO_CHANNELS] {
        &self.slots
    }
}

impl Peripheral for Nco {
    fn name(&self) -> &'static str {
        "nco"
    }

    fn address_ranges(&self) -> &'static [AddressRange] {
        NCO_RANGES
    }

    fn read_word(&mut self, _addr: u32) -> Result<u32, Exception> {
        Ok(self.tx_mode)
    }

    fn write_word(&mut self, _addr: u32, val: u32) -> Result<(), Exception> {
        self.tx_mode = val;
        Ok(())
    }

    fn tick(&mut self, _cpu_instructions: u64) {}

    fn reset_cold(&mut self) {
        self.tx_mode = 0;
        // A reset clears the NCO/IVT table — silicon has no installed
        // vectors until the firmware reprograms them via `.di`.
        self.slots = [NcoSlot::empty(); NCO_CHANNELS];
    }

    fn snapshot(&self) -> PeripheralSnapshot {
        PeripheralSnapshot::empty(self.name())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn round_trip_tx_mode() {
        let mut n = Nco::new();
        assert_eq!(n.read_word(REG_NCO_TX_MODE).unwrap(), 0);
        n.write_word(REG_NCO_TX_MODE, 0x0000_4000).unwrap();
        assert_eq!(n.read_word(REG_NCO_TX_MODE).unwrap(), 0x0000_4000);
    }

    #[test]
    fn di_store_builds_j_limm_vector_slot() {
        let mut n = Nco::new();
        // Reproduce a channel-5 vector install (target 0x0004_8120):
        // four 16-bit `stw.di` fields — CONFIG, PRESCALE, FREQ_HI,
        // FREQ_LO — at channel 5 (CPU addr 5*8 = 0x28).
        let base = 5 * NCO_SLOT_BYTES;
        n.ivt_di_store(base, 0x2020, 2);
        n.ivt_di_store(base + 2, 0x0F80, 2);
        n.ivt_di_store(base + 4, 0x0004, 2); // FREQ_HI
        n.ivt_di_store(base + 6, 0x8120, 2); // FREQ_LO

        assert!(n.ivt_slots()[5].is_installed_vector());
        assert_eq!(n.interrupt_vector(5), Some(0x0004_8120));
        // Unprogrammed channels are not vectors.
        assert_eq!(n.interrupt_vector(4), None);
        assert_eq!(n.interrupt_vector(6), None);
    }

    #[test]
    fn slot_without_j_limm_opcode_is_not_a_vector() {
        let mut n = Nco::new();
        // A clock-only NCO channel (no `2020 0f80` opcode prefix)
        // must not be treated as an installed interrupt vector.
        n.ivt_di_store(8 * NCO_SLOT_BYTES + 4, 0x0000_0228, 4);
        assert!(!n.ivt_slots()[8].is_installed_vector());
        assert_eq!(n.interrupt_vector(8), None);
    }

    #[test]
    fn word_di_store_full_slot_is_a_vector() {
        let mut n = Nco::new();
        // Live-silicon slot0 anchor: `2020 0f80 0000 0150` = j @0x150.
        n.ivt_di_store(0, 0x2020_0F80, 4);
        n.ivt_di_store(4, 0x0000_0150, 4);
        assert_eq!(n.interrupt_vector(0), Some(0x0000_0150));
    }

    #[test]
    fn reset_clears_installed_vectors() {
        let mut n = Nco::new();
        n.ivt_di_store(0, 0x2020_0F80, 4);
        n.ivt_di_store(4, 0x0000_0150, 4);
        assert!(n.interrupt_vector(0).is_some());
        n.reset_cold();
        assert_eq!(n.interrupt_vector(0), None);
    }

    #[test]
    fn claims_covers_word_only() {
        let n = Nco::new();
        assert!(n.claims(0x0100_0F80));
        assert!(n.claims(0x0100_0F83));
        assert!(!n.claims(0x0100_0F84));
        assert!(!n.claims(0x0100_0F7C));
    }
}
