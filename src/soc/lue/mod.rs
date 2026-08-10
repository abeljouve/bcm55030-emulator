//! Packet classifier — the latched indirect port into its rule tables.
//!
//! Two instances live inside the lane register file, `0x400` apart. Each
//! is five words: a command register and four data registers.
//!
//! ```text
//! instance 0   CMD 0x0100240C   DATA 0x01002410 .. 0x0100241C
//! instance 1   CMD 0x0100280C   DATA 0x01002810 .. 0x0100281C
//! ```
//!
//! Ten addresses, listed one by one rather than taken as a range: the
//! words either side of them are live and belong to the SerDes — lane
//! reset at `+0x00`, a status word at `+0x04` that the hardware state
//! poller reads a hundred thousand times a boot, the VLAN EtherType at
//! `+0x20`. Widening this claim to a range would take all of them.
//!
//! Software drives the port by writing the data registers (for a write),
//! then the command word with bit 31 set, then spinning on the command
//! word until that bit clears. **That spin has no timeout**, which sets
//! the one hard rule here: the go bit clears on every command, whatever
//! it asks for. See [`port::LuePort::write_cmd`].
//!
//! Which instance a command reaches matters: the two hold different
//! tables, and the readback path that prints the rules addresses the
//! second one. Treating either as a spare would make an empty answer
//! look like a refutation of the model.

pub mod engine;
pub mod port;
pub mod rule;

pub use engine::{Engine, EngineCounters, RuleStart, Verdict};
pub use rule::{Action, ActionPayload, Clause, Entry, Field, Link, Op, Rule, RuleError, RuleRead};
pub use port::{Cmd, LuePort, PortCounters, Quad, CMD_ERROR, CMD_GO, CMD_WRITE};

use crate::cpu::exception::Exception;

/// Command register of each instance. The four data registers follow at
/// `+4`, `+8`, `+0xC`, `+0x10`.
pub const LUE_CMD_ADDR: [u32; 2] = [0x0100_240C, 0x0100_280C];

/// Words claimed per instance: the command word plus four data words.
const WORDS_PER_INSTANCE: u32 = 5;

/// Which port and which table the downstream path consults.
///
/// ⛔ **Still not established, and the shape is probably wrong too.**
/// Nothing in the firmware ever reads a classification result back, so no
/// observation says which port and table decide where a downstream frame
/// goes. Worse, the table index software uses when it *writes* a rule is
/// derived per queue descriptor rather than fixed — a queue of type 0
/// goes to table 3, type 1 to table 1, type 2 to table 2, and anything
/// else to `queue_count + 4` — so a single pair may not be able to
/// express what the hardware does. The binding stays a named setting for
/// exactly that reason: changing it is one line, and refuting it does not
/// mean rewriting the engine.
///
/// The default names the pair that holds the rules which name a queue.
/// Read back over a boot, instance 1 table 8 holds five rules and every
/// one of them ends in a queue result; the pair the readback path prints
/// (instance 1, table 3) holds five rules and **not one** queue result —
/// it is the least informative table of the fourteen software programs.
/// The old default named it only because it was the one that could be
/// read off a running device.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ClassifierBinding {
    pub instance: usize,
    pub table: u8,
}

impl Default for ClassifierBinding {
    fn default() -> Self {
        Self { instance: 1, table: 8 }
    }
}

/// The classifier's two indirect ports.
#[derive(Clone, Debug, Default)]
pub struct Lue {
    pub inst: [LuePort; 2],
}

impl Lue {
    pub fn new() -> Self {
        Self { inst: [LuePort::new(), LuePort::new()] }
    }

    /// Which instance and which word an address names, if any.
    ///
    /// Slot 0 is the command register, slots 1..4 are the data
    /// registers in address order.
    #[inline]
    fn locate(addr: u32) -> Option<(usize, usize)> {
        let word = addr & !3;
        for (inst, base) in LUE_CMD_ADDR.iter().enumerate() {
            if word >= *base && word < base + WORDS_PER_INSTANCE * 4 {
                return Some((inst, ((word - base) / 4) as usize));
            }
        }
        None
    }

    #[inline]
    pub fn claims(&self, addr: u32) -> bool {
        Self::locate(addr).is_some()
    }

    /// Read a word with no side effect of any kind.
    ///
    /// The go bit is cleared when the command is accepted, not when it
    /// is read back, so reading is free — which is what lets the same
    /// body serve `peek_word`. A port that cleared state on read would
    /// corrupt itself through the bank's peek-before-write path.
    pub fn peek_word(&self, addr: u32) -> Option<u32> {
        let (inst, slot) = Self::locate(addr)?;
        Some(match slot {
            0 => self.inst[inst].read_cmd(),
            n => self.inst[inst].read_data(n - 1),
        })
    }

    pub fn read_word(&mut self, addr: u32) -> Result<u32, Exception> {
        Ok(self.peek_word(addr).unwrap_or(0))
    }

    pub fn write_word(&mut self, addr: u32, val: u32) -> Result<(), Exception> {
        if let Some((inst, slot)) = Self::locate(addr) {
            match slot {
                0 => self.inst[inst].write_cmd(val),
                n => self.inst[inst].write_data(n - 1, val),
            }
        }
        Ok(())
    }

    pub fn read_half(&mut self, addr: u32) -> Result<u16, Exception> {
        let word = self.read_word(addr & !3)?;
        Ok(if addr & 2 == 0 { (word >> 16) as u16 } else { word as u16 })
    }

    pub fn read_byte(&mut self, addr: u32) -> Result<u8, Exception> {
        let word = self.read_word(addr & !3)?;
        Ok((word >> (24 - (addr & 3) * 8)) as u8)
    }

    pub fn write_half(&mut self, addr: u32, val: u16) -> Result<(), Exception> {
        let word = self.read_word(addr & !3)?;
        let merged = if addr & 2 == 0 {
            (word & 0x0000_FFFF) | ((val as u32) << 16)
        } else {
            (word & 0xFFFF_0000) | val as u32
        };
        self.write_word(addr & !3, merged)
    }

    pub fn write_byte(&mut self, addr: u32, val: u8) -> Result<(), Exception> {
        let word = self.read_word(addr & !3)?;
        let shift = 24 - (addr & 3) * 8;
        let merged = (word & !(0xFFu32 << shift)) | ((val as u32) << shift);
        self.write_word(addr & !3, merged)
    }

    /// Classify a frame against the bound port and table.
    ///
    /// Returns what the rules say, including "cannot say". Translating a
    /// verdict into a queue is **not** done here: that step is the
    /// unestablished link, and keeping it in a separate, named function
    /// is what lets it be refuted without touching the engine.
    /// The counters come back with the verdict rather than being dropped:
    /// "undecidable" is a number, "undecidable because a field code has
    /// no frame field behind it" is a direction, and only the second one
    /// says what to work on next.
    pub fn classify(
        &self,
        binding: ClassifierBinding,
        frame: &[u8],
    ) -> (Verdict, EngineCounters) {
        let port = &self.inst[binding.instance & 1];
        let starts: Vec<RuleStart> = port
            .rule_starts(binding.table)
            .into_iter()
            .map(|index| RuleStart { table: binding.table, index })
            .collect();
        let mut counters = EngineCounters::default();
        if starts.is_empty() {
            // An empty table has nothing to say. It is not a miss, and
            // the caller must not read it as one.
            counters.frames_classified += 1;
            counters.no_rules += 1;
            return (Verdict::Undecidable { reason: "no rules programmed" }, counters);
        }
        let verdict = Engine::classify(port, &starts, frame, &mut counters);
        (verdict, counters)
    }

    /// Power-on state: empty tables, zero registers.
    ///
    /// The command registers are seeded non-zero on real hardware, and the
    /// firmware issues two byte reads of them per boot whose consumer is not
    /// identified. Starting empty makes those reads return zero instead — a
    /// known, deliberate difference, not an oversight.
    pub fn reset_cold(&mut self) {
        for p in &mut self.inst {
            p.reset();
        }
    }

    pub fn reset_warm(&mut self) {
        self.reset_cold();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Ten addresses, and only those ten. The neighbours are live: lane
    /// reset, the status word the state poller hammers, the VLAN
    /// EtherType, and the second command engine.
    #[test]
    fn exactly_ten_addresses_are_claimed() {
        let lue = Lue::new();
        for addr in [
            0x0100_240C, 0x0100_2410, 0x0100_2414, 0x0100_2418, 0x0100_241C,
            0x0100_280C, 0x0100_2810, 0x0100_2814, 0x0100_2818, 0x0100_281C,
        ] {
            assert!(lue.claims(addr), "{addr:#010x} should be claimed");
        }
        for addr in [
            0x0100_2400, 0x0100_2404, 0x0100_2408, 0x0100_2420, 0x0100_2424,
            0x0100_2644, 0x0100_2800, 0x0100_2804, 0x0100_2820, 0x0100_2A44,
        ] {
            assert!(!lue.claims(addr), "{addr:#010x} must stay with its owner");
        }
    }

    /// Reading must not change anything: the bank peeks a word before
    /// writing it, so a port that consumed state on read would corrupt
    /// itself through its own write path.
    #[test]
    fn reading_the_port_has_no_side_effect() {
        let mut lue = Lue::new();
        lue.write_word(0x0100_2418, 0x0000_8809).unwrap();
        lue.write_word(0x0100_240C, CMD_GO | CMD_WRITE | (3 << 12) | 0x40).unwrap();

        let before = lue.clone();
        for _ in 0..3 {
            for addr in [0x0100_240Cu32, 0x0100_2410, 0x0100_2414, 0x0100_2418, 0x0100_241C] {
                let r = lue.read_word(addr).unwrap();
                assert_eq!(r, lue.peek_word(addr).unwrap(), "peek and read disagree");
            }
        }
        assert_eq!(format!("{:?}", lue.inst[0]), format!("{:?}", before.inst[0]));
    }

    /// The instance is part of the address, not a detail: the two hold
    /// different tables and the readback path uses the second one.
    #[test]
    fn the_two_instances_do_not_share_a_table() {
        let mut lue = Lue::new();
        lue.write_word(0x0100_2418, 0x0000_8809).unwrap();
        lue.write_word(0x0100_240C, CMD_GO | CMD_WRITE | (3 << 12) | 0x40).unwrap();

        // Same table, same index, other instance.
        lue.write_word(0x0100_280C, CMD_GO | (3 << 12) | 0x40).unwrap();
        for addr in [0x0100_2810u32, 0x0100_2814, 0x0100_2818, 0x0100_281C] {
            assert_eq!(lue.read_word(addr).unwrap(), 0);
        }
        assert_eq!(lue.inst[1].counters.reads_of_empty, 1);

        // And the first instance still holds it.
        lue.write_word(0x0100_240C, CMD_GO | (3 << 12) | 0x40).unwrap();
        assert_eq!(lue.read_word(0x0100_2418).unwrap(), 0x0000_8809);
    }

    /// The word order is the whole point of `Quad`: the header is at the
    /// highest address, and the entry reads back with it first.
    #[test]
    fn an_entry_written_through_mmio_keeps_its_word_order() {
        let mut lue = Lue::new();
        let q = Quad::new([0xF030_0021, 0x0000_0000, 0x0000_8809, 0x1000_0000]);
        for (i, w) in q.to_regs().iter().enumerate() {
            lue.write_word(0x0100_2410 + i as u32 * 4, *w).unwrap();
        }
        lue.write_word(0x0100_240C, CMD_GO | CMD_WRITE | (3 << 12) | 0x2E).unwrap();
        assert_eq!(lue.inst[0].entry(3, 0x2E), Some(q));
        // The header is at the highest of the four addresses, and the
        // last entry word at the lowest.
        assert_eq!(lue.read_word(0x0100_241C).unwrap(), 0xF030_0021);
        assert_eq!(lue.read_word(0x0100_2410).unwrap(), 0x1000_0000);
    }

    /// A byte read of the command register is something the boot path
    /// does; it must answer from the word without disturbing it.
    #[test]
    fn narrow_accesses_reach_the_same_registers() {
        let mut lue = Lue::new();
        lue.write_word(0x0100_240C, CMD_GO | CMD_WRITE | (1 << 16) | (3 << 12) | 0x40).unwrap();
        assert_eq!(lue.read_byte(0x0100_240C).unwrap(), 0x40);
        assert_eq!(lue.read_half(0x0100_240C).unwrap(), 0x4001);
        assert_eq!(lue.read_half(0x0100_240E).unwrap(), 0x3040);
        assert_eq!(lue.inst[0].counters.commands, 1);

        lue.write_half(0x0100_2412, 0xBEEF).unwrap();
        assert_eq!(lue.read_word(0x0100_2410).unwrap(), 0x0000_BEEF);
        lue.write_byte(0x0100_2410, 0xAB).unwrap();
        assert_eq!(lue.read_word(0x0100_2410).unwrap(), 0xAB00_BEEF);
    }

    #[test]
    fn a_cold_reset_empties_both_instances() {
        let mut lue = Lue::new();
        lue.write_word(0x0100_2418, 0x1234).unwrap();
        lue.write_word(0x0100_240C, CMD_GO | CMD_WRITE | (3 << 12) | 0x40).unwrap();
        lue.reset_cold();
        assert_eq!(lue.read_word(0x0100_240C).unwrap(), 0);
        assert_eq!(lue.inst[0].entry_count(3), 0);
        assert_eq!(lue.inst[0].counters, PortCounters::default());
    }
}
