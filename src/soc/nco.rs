//! BCM55030 Numerically Controlled Oscillator (NCO) — D1.
//!
//! Owns the single `NCO_TX_MODE_SECONDARY` register at
//! `0x01000F80`, identified by dumping SRAM after a full boot and
//! reading the runtime-initialised `DAT_ram_2003e924` pointer
//! (hwregs block 23). The firmware uses this register to toggle
//! dual-TX mode on the SerDes TX path:
//!
//! | Bit | Name                     | Semantics                   |
//! |:----|:-------------------------|:----------------------------|
//! | `9` | `NCO_TX_MODE_SECONDARY`  | Set in mode 2               |
//! | `14`| `NCO_DUAL_TX_ENABLE`     | Enable dual-TX mode         |
//!
//! v1 is a plain backing store. The future `CLK_READY_FLAG` at
//! `0x00FC1017` mentioned in the design notes section 12 is not
//! claimed here because the boot trace shows it is not touched
//! by the warm boot path; it will migrate in when a firmware
//! path that reads it is identified.

use crate::cpu::exception::Exception;
use crate::soc::peripheral::{AddressRange, Peripheral, PeripheralSnapshot};

pub const REG_NCO_TX_MODE: u32 = 0x0100_0F80;

const NCO_RANGES: &[AddressRange] =
    &[AddressRange::new(REG_NCO_TX_MODE, REG_NCO_TX_MODE + 4)];

/// Number of NCO / ARC interrupt-vector channels.
pub const NCO_CHANNELS: usize = 16;

/// Each channel occupies 8 bytes in the low-memory aperture.
pub const NCO_SLOT_BYTES: u32 = 8;

/// Top of the NCO/IVT aperture when `AUX_INT_VECTOR_BASE` (AUX 0x25)
/// is 0 — the only base ever observed on this silicon (the bootloader
/// and the firmware never write AUX 0x25; verified in the firmware-silent
/// session logs). `0..0x80` = 16 channels × 8 bytes.
pub const NCO_IVT_LIMIT: u32 = (NCO_CHANNELS as u32) * NCO_SLOT_BYTES;

/// The ARCompact `j @<limm>` opcode word that prefixes every
/// programmed NCO/IVT slot (`CONFIG=0x2020`, `PRESCALE=0x0F80`,
/// big-endian on-chip = bytes `20 20 0f 80`). When a slot's first
/// four bytes equal this, the channel is an installed interrupt
/// vector and the trailing 32-bit word is the absolute jump target.
/// Live-silicon anchor: slot0 = `2020 0f80 0000 0150` = `j @0x150`.
pub const NCO_J_LIMM_OPCODE: u32 = 0x2020_0F80;

/// One NCO channel / ARC interrupt-vector slot. The hardware aliases
/// the 16-channel NCO table over the low-memory ARC interrupt-vector
/// range on a separate physical bus: `.di` (uncached) stores land
/// here, while plain CPU reads / instruction fetch see SRAM. The ARC
/// interrupt unit fetches its vector from this table, not from SRAM.
/// (Evidence: Ghidra `nco_write_channel` @0x5a18 /
/// `hw_install_irq_vector_2` @0x20042d00 plate comments;
/// "NCO table IS the ARC IVT" RE swarm, live slot0 = `j @0x150`.)
#[derive(Clone, Copy)]
pub struct NcoSlot {
    /// Raw 8 bytes exactly as written by the `stw.di` field stores
    /// (`+0 CONFIG`, `+2 PRESCALE`, `+4 FREQ_HI`, `+6 FREQ_LO`).
    pub raw: [u8; 8],
    /// Any `.di` field has been written to this slot since reset.
    pub written: bool,
}

impl NcoSlot {
    const fn empty() -> Self {
        Self { raw: [0; 8], written: false }
    }

    /// `CONFIG` halfword (`raw[0..2]`, big-endian).
    #[inline]
    pub fn config(&self) -> u16 {
        ((self.raw[0] as u16) << 8) | self.raw[1] as u16
    }

    /// The absolute 32-bit jump target (`FREQ_HI:FREQ_LO`,
    /// `raw[4..8]`, big-endian) — the operand of the `j @<limm>`.
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
    /// `stw.di` field order used by `nco_write_channel`.
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
        // Reproduce `nco_write_channel(5, 0x0004_8120)`: four 16-bit
        // `stw.di` fields — CONFIG, PRESCALE, FREQ_HI, FREQ_LO — at
        // channel 5 (CPU addr 5*8 = 0x28).
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
