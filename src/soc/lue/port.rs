//! The latched indirect port: command word, four data words, and the
//! sparse table file behind them.

use std::collections::HashMap;

/// Bit 31 of the command word: software sets it to start an operation
/// and spins on the same address until it reads back clear.
pub const CMD_GO: u32 = 0x8000_0000;

/// Bit 30: the direction. It does **not** come from the opcode — the
/// hardware takes it from a separate lookup, and two opcodes that map to
/// the same operation field are told apart by this bit alone.
pub const CMD_WRITE: u32 = 0x4000_0000;

/// Bit 29: the error flag software tests after the spin.
///
/// The model never raises it. Neither the value silicon returns nor the
/// conditions that produce it are established, and it changes which
/// branch software takes — so an invented one would steer the firmware
/// down a path nothing here can justify.
pub const CMD_ERROR: u32 = 0x2000_0000;

/// How many tables the four table-select bits address.
pub const TABLE_COUNT: usize = 16;

/// The index field is twelve bits wide.
pub const INDEX_MASK: u16 = 0x0FFF;

/// A decoded command word.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Cmd {
    pub write: bool,
    /// Operation field, bits [23:20]. Software derives it from an opcode
    /// that the hardware never sees: opcodes at or above `0x0F` are sent
    /// as zero, and only the direction bit distinguishes them.
    pub op_field: u8,
    /// Bits [18:16]. Selects which view of a table entry is addressed;
    /// its meaning is not established.
    pub arg2: u8,
    /// Bits [15:12].
    pub table: u8,
    /// Bits [11:0].
    pub index: u16,
}

impl Cmd {
    pub fn decode(word: u32) -> Self {
        Self {
            write: word & CMD_WRITE != 0,
            op_field: ((word >> 20) & 0xF) as u8,
            arg2: ((word >> 16) & 7) as u8,
            table: ((word >> 12) & 0xF) as u8,
            index: (word as u16) & INDEX_MASK,
        }
    }

    /// The command word this describes, without the go bit — the value
    /// the port holds once the operation is over.
    pub fn encode(&self) -> u32 {
        (if self.write { CMD_WRITE } else { 0 })
            | ((self.op_field as u32 & 0xF) << 20)
            | ((self.arg2 as u32 & 7) << 16)
            | ((self.table as u32 & 0xF) << 12)
            | (self.index & INDEX_MASK) as u32
    }
}

/// A table entry: four words, indexed the way software indexes them.
///
/// `words[0]` is the clause header. The register file runs the other
/// way — the header sits at the **highest** of the four addresses — so
/// the two orders are inverted with respect to each other. That
/// inversion lives here, in `to_regs` / `from_regs`, and nowhere else.
///
/// ⛔ **Four words is what this models, and a device prints five.** Its
/// rule listing puts one byte between the entry index and the four words
/// — `003F 01 F0300021 …` — and what that byte is has not been
/// established. It is not modelled here rather than being guessed at; a
/// fifth field invented to fill the gap would round-trip against itself
/// and nothing else.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Quad {
    pub words: [u32; 4],
}

impl Quad {
    pub fn new(words: [u32; 4]) -> Self {
        Self { words }
    }

    /// The four data registers in address order (lowest first).
    pub fn to_regs(&self) -> [u32; 4] {
        [self.words[3], self.words[2], self.words[1], self.words[0]]
    }

    /// Rebuild an entry from the four data registers in address order.
    pub fn from_regs(regs: [u32; 4]) -> Self {
        Self { words: [regs[3], regs[2], regs[1], regs[0]] }
    }

    pub fn is_zero(&self) -> bool {
        self.words == [0; 4]
    }
}

/// What the port refused, and why. A path that drops a command without
/// incrementing something reads as a healthy port.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct PortCounters {
    /// Commands accepted with the go bit set.
    pub commands: u64,
    pub writes: u64,
    pub reads: u64,
    /// Commands whose operation field is not one this model implements.
    /// The go bit still clears — the firmware's spin has no timeout —
    /// but no data moves.
    pub unknown_op: u64,
    /// Command-word writes without the go bit: nothing was asked for.
    pub no_go: u64,
    /// Reads of an entry nothing ever wrote. Not an error: the tables
    /// are scanned exhaustively at start-up. Without this number, "the
    /// table answered zero" cannot be told from "the table is empty".
    pub reads_of_empty: u64,
    /// Writes that stored an all-zero entry, i.e. cleared a slot.
    pub writes_of_zero: u64,
}

impl PortCounters {
    /// Every command with the go bit set is accounted for exactly once.
    pub fn commands_accounted_for(&self) -> bool {
        self.commands == self.writes + self.reads + self.unknown_op
    }
}

/// What happened to one slot of one table, over the run.
///
/// The port's aggregate counters answer "how many entries were cleared";
/// they cannot answer "which one, and when". That difference is not
/// academic: a table read at the end of a boot held five rules where a
/// device held six, and the sixth had been written and then cleared —
/// one of a few hundred zero writes, indistinguishable from the padding
/// the rest of them are without this.
///
/// ⚠ `at` fields are **command ordinals**, not CPU cycles: the value of
/// `PortCounters::commands` at the moment of the write. The port has no
/// clock of its own, and inventing one from the caller's would put a host
/// quantity in a peripheral.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct EntryLog {
    /// Ordinal of the first write to this slot.
    pub first_write_at: u64,
    /// Ordinal of the most recent write, cleared or not.
    pub last_write_at: u64,
    /// Writes to this slot, including the ones that cleared it.
    pub writes: u32,
    /// Writes that stored all zeros, i.e. emptied the slot.
    pub clears: u32,
    /// Ordinal of the most recent clear, if there was one.
    pub last_clear_at: Option<u64>,
    /// What the slot held immediately before its most recent clear. This
    /// is the word that would otherwise be gone: a cleared entry reads
    /// back exactly like one nothing ever wrote.
    pub cleared_content: Option<Quad>,
}

/// One slot's journal, with the slot it belongs to.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct LoggedSlot {
    pub table: u8,
    pub index: u16,
    pub log: EntryLog,
}

/// One instance of the indirect port.
#[derive(Clone, Debug)]
pub struct LuePort {
    /// Sparse: an entry that was never written is absent, and the
    /// start-up scan walks all 4096 indices of two tables several times
    /// per boot. Dense storage would be a megabyte per instance, copied
    /// on every snapshot.
    tables: [HashMap<u16, Quad>; TABLE_COUNT],
    /// The four data registers, in address order.
    regs: [u32; 4],
    /// The last command word, go bit already cleared.
    last_cmd: u32,
    /// Per-slot write history. Sparse for the same reason `tables` is,
    /// and it survives a clear — which is the whole point of it.
    journal: HashMap<(u8, u16), EntryLog>,
    pub counters: PortCounters,
}

impl Default for LuePort {
    fn default() -> Self {
        Self::new()
    }
}

impl LuePort {
    pub fn new() -> Self {
        Self {
            tables: std::array::from_fn(|_| HashMap::new()),
            regs: [0; 4],
            last_cmd: 0,
            journal: HashMap::new(),
            counters: PortCounters::default(),
        }
    }

    pub fn reset(&mut self) {
        for t in &mut self.tables {
            t.clear();
        }
        self.regs = [0; 4];
        self.last_cmd = 0;
        self.journal.clear();
        self.counters = PortCounters::default();
    }

    pub fn read_cmd(&self) -> u32 {
        self.last_cmd
    }

    pub fn read_data(&self, slot: usize) -> u32 {
        self.regs[slot]
    }

    pub fn write_data(&mut self, slot: usize, val: u32) {
        self.regs[slot] = val;
    }

    /// Look an entry up without touching the port — for tests and for
    /// anything that inspects the tables rather than driving them.
    pub fn entry(&self, table: u8, index: u16) -> Option<Quad> {
        self.tables
            .get(table as usize & (TABLE_COUNT - 1))
            .and_then(|t| t.get(&(index & INDEX_MASK)))
            .copied()
    }

    /// Where each rule in a table begins.
    ///
    /// A rule occupies a run of consecutive indices — its clauses, then
    /// the entries its action list spills into. Both are found by reading
    /// the rules forward from the lowest index and skipping what each one
    /// covers, because software keeps its own list of rules in memory and
    /// never writes it here.
    ///
    /// ⚠ Counting an index as a start whenever the entry below it is
    /// terminal is **not** the same thing: it reports every spill entry as
    /// a rule of its own. Over a boot that turns five rules into ten, and
    /// the five phantoms are exactly the entries holding the queue result.
    pub fn rule_starts(&self, table: u8) -> Vec<u16> {
        let t = &self.tables[table as usize & (TABLE_COUNT - 1)];
        let mut keys: Vec<u16> = t.keys().copied().collect();
        keys.sort_unstable();

        let mut starts = Vec::new();
        let mut covered_through: Option<u16> = None;
        for index in keys {
            if covered_through.is_some_and(|end| index <= end) {
                continue;
            }
            starts.push(index);
            let span = super::rule::Rule::decode(|i| t.get(&i).copied(), index).span.max(1);
            covered_through = Some(index.saturating_add(span - 1));
        }
        starts
    }

    /// Read the rule that starts at `index`, spill entries included.
    pub fn rule(&self, table: u8, index: u16) -> super::rule::RuleRead {
        let t = &self.tables[table as usize & (TABLE_COUNT - 1)];
        super::rule::Rule::decode(|i| t.get(&i).copied(), index)
    }

    /// Number of entries held in a table.
    pub fn entry_count(&self, table: u8) -> usize {
        self.tables[table as usize & (TABLE_COUNT - 1)].len()
    }

    /// What was written to one slot, and when.
    pub fn entry_log(&self, table: u8, index: u16) -> Option<&EntryLog> {
        self.journal
            .get(&(table & (TABLE_COUNT as u8 - 1), index & INDEX_MASK))
    }

    /// Every slot the port has ever been written to, in table then index
    /// order — including the ones that are empty now.
    pub fn entry_logs(&self) -> Vec<LoggedSlot> {
        let mut all: Vec<LoggedSlot> = self
            .journal
            .iter()
            .map(|(&(table, index), &log)| LoggedSlot { table, index, log })
            .collect();
        all.sort_unstable_by_key(|s| (s.table, s.index));
        all
    }

    /// The slots that were written and then emptied, with what they held.
    ///
    /// This is the instrument the aggregate `writes_of_zero` could not
    /// be: most of those zero writes are the padding the start-up path
    /// emits, and a handful are entries that carried a rule. Only the
    /// second kind has a non-`None` `cleared_content`.
    pub fn cleared_slots(&self) -> Vec<LoggedSlot> {
        self.entry_logs()
            .into_iter()
            .filter(|s| s.log.cleared_content.is_some())
            .collect()
    }

    /// Accept a command word.
    ///
    /// The go bit always clears, on every combination — index beyond any
    /// depth, table never used, operation field never seen. The firmware
    /// spins on this word with no timeout, so a command the model does
    /// not understand must still complete; it just moves no data.
    pub fn write_cmd(&mut self, word: u32) {
        let cmd = Cmd::decode(word);
        // Never raise the error bit, and never leave the go bit set.
        self.last_cmd = cmd.encode();

        if word & CMD_GO == 0 {
            self.counters.no_go += 1;
            return;
        }
        self.counters.commands += 1;

        // Only operation field zero is exercised by anything observed:
        // it is the plain entry read and entry write. Everything else
        // completes without moving data rather than guessing.
        if cmd.op_field != 0 {
            self.counters.unknown_op += 1;
            return;
        }

        let table = &mut self.tables[cmd.table as usize];
        if cmd.write {
            self.counters.writes += 1;
            let quad = Quad::from_regs(self.regs);
            let at = self.counters.commands;
            let log = self
                .journal
                .entry((cmd.table, cmd.index))
                .or_insert(EntryLog { first_write_at: at, ..EntryLog::default() });
            log.last_write_at = at;
            log.writes += 1;
            if quad.is_zero() {
                // Storing zero and never storing anything read back the
                // same; keep the table sparse and count the difference.
                self.counters.writes_of_zero += 1;
                log.clears += 1;
                log.last_clear_at = Some(at);
                // What the slot held is about to become unreadable. Keep
                // it: a cleared rule and a slot that never held one are
                // the same table afterwards, and they are not the same
                // event.
                if let Some(previous) = table.remove(&cmd.index) {
                    log.cleared_content = Some(previous);
                }
            } else {
                table.insert(cmd.index, quad);
            }
        } else {
            self.counters.reads += 1;
            match table.get(&cmd.index) {
                Some(q) => self.regs = q.to_regs(),
                None => {
                    self.counters.reads_of_empty += 1;
                    self.regs = [0; 4];
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_command_word_round_trips_through_its_fields() {
        let cmd = Cmd { write: true, op_field: 0, arg2: 1, table: 3, index: 0x040 };
        assert_eq!(Cmd::decode(cmd.encode() | CMD_GO), cmd);

        let read = Cmd { write: false, op_field: 0, arg2: 2, table: 1, index: 0x005 };
        assert_eq!(Cmd::decode(read.encode() | CMD_GO), read);
    }

    /// The index field is twelve bits, not eleven: an index of `0x800`
    /// must survive, and one above the field must not bleed into the
    /// table select.
    #[test]
    fn the_index_field_is_twelve_bits_wide() {
        let cmd = Cmd::decode(CMD_GO | (3 << 12) | 0x0800);
        assert_eq!(cmd.index, 0x800);
        assert_eq!(cmd.table, 3);
    }

    /// An entry is indexed one way by software and laid out the other
    /// way in the register file. Only this pair knows that.
    #[test]
    fn entry_words_and_data_registers_are_inverted() {
        let q = Quad::new([0xF030_0021, 0x0000_0000, 0x0000_8809, 0x1000_0000]);
        let regs = q.to_regs();
        assert_eq!(regs[0], 0x1000_0000, "lowest address holds words[3]");
        assert_eq!(regs[3], 0xF030_0021, "highest address holds the header");
        assert_eq!(Quad::from_regs(regs), q);
    }

    #[test]
    fn a_write_then_a_read_returns_the_same_entry() {
        let mut p = LuePort::new();
        let q = Quad::new([0xF038_00F1, 0x0000_0001, 0x0000_0010, 0x0F60_0000]);
        for (i, w) in q.to_regs().iter().enumerate() {
            p.write_data(i, *w);
        }
        p.write_cmd(CMD_GO | CMD_WRITE | (3 << 12) | 0x40);

        p.write_data(0, 0);
        p.write_data(1, 0);
        p.write_data(2, 0);
        p.write_data(3, 0);
        p.write_cmd(CMD_GO | (3 << 12) | 0x40);
        assert_eq!(Quad::from_regs([
            p.read_data(0),
            p.read_data(1),
            p.read_data(2),
            p.read_data(3),
        ]), q);
    }

    /// The three negative controls: a different table, a neighbouring
    /// index, and — checked one level up — the other instance.
    #[test]
    fn a_neighbouring_table_or_index_holds_nothing() {
        let mut p = LuePort::new();
        for (i, w) in Quad::new([1, 2, 3, 4]).to_regs().iter().enumerate() {
            p.write_data(i, *w);
        }
        p.write_cmd(CMD_GO | CMD_WRITE | (3 << 12) | 0x40);

        for cmd in [CMD_GO | (1 << 12) | 0x40, CMD_GO | (3 << 12) | 0x41] {
            p.write_cmd(cmd);
            assert_eq!([p.read_data(0), p.read_data(1), p.read_data(2), p.read_data(3)], [0; 4]);
        }
    }

    /// The spin has no timeout: whatever software asks for, the go bit
    /// must be gone by the time it looks.
    #[test]
    fn the_go_bit_clears_on_every_combination() {
        let mut p = LuePort::new();
        for word in [
            CMD_GO,
            CMD_GO | CMD_WRITE | 0x0FFF,
            CMD_GO | (0xF << 20) | (7 << 16) | (0xF << 12) | 0x0FFF,
            CMD_GO | CMD_WRITE | (9 << 20),
        ] {
            p.write_cmd(word);
            assert_eq!(p.read_cmd() & CMD_GO, 0, "go bit left set for {word:#010x}");
            assert_eq!(p.read_cmd() & CMD_ERROR, 0, "error bit invented for {word:#010x}");
        }
    }

    /// An operation this model does not implement completes without
    /// moving data — and is counted, so "nothing happened" can be told
    /// from "nothing was asked".
    #[test]
    fn an_unknown_operation_leaves_the_data_alone_and_is_counted() {
        let mut p = LuePort::new();
        p.write_data(0, 0xDEAD_BEEF);
        p.write_cmd(CMD_GO | (9 << 20) | (3 << 12) | 0x40);
        assert_eq!(p.read_data(0), 0xDEAD_BEEF);
        assert_eq!(p.counters.unknown_op, 1);
        assert!(p.counters.commands_accounted_for());
    }

    #[test]
    fn a_command_without_the_go_bit_asks_for_nothing() {
        let mut p = LuePort::new();
        p.write_cmd(CMD_WRITE | (3 << 12) | 0x40);
        assert_eq!(p.counters.no_go, 1);
        assert_eq!(p.counters.commands, 0);
    }

    /// Reading an index nothing wrote answers zero — and says so, so a
    /// zero can be told from an answer.
    #[test]
    fn a_read_of_an_empty_entry_is_counted() {
        let mut p = LuePort::new();
        p.write_cmd(CMD_GO | (5 << 12) | 0x123);
        assert_eq!(p.counters.reads_of_empty, 1);
        assert_eq!(p.entry_count(5), 0);
        assert!(p.counters.commands_accounted_for());
    }

    /// Writing zero clears a slot rather than filling the table with
    /// padding: the start-up path pads entries out to a multiple of four.
    #[test]
    fn writing_zero_clears_the_slot_and_keeps_the_table_sparse() {
        let mut p = LuePort::new();
        p.write_data(3, 0xF030_0021);
        p.write_cmd(CMD_GO | CMD_WRITE | (3 << 12) | 0x40);
        assert_eq!(p.entry_count(3), 1);

        p.write_data(3, 0);
        p.write_cmd(CMD_GO | CMD_WRITE | (3 << 12) | 0x40);
        assert_eq!(p.entry_count(3), 0);
        assert_eq!(p.counters.writes_of_zero, 1);
    }

    /// A cleared slot reads back exactly like one nothing ever wrote. The
    /// journal is what tells them apart — and it names the slot, the
    /// command that cleared it, and what it held.
    #[test]
    fn an_entry_journal_names_the_slot_that_was_cleared() {
        let mut p = LuePort::new();
        let rule = Quad::new([0xF038_00F1, 0x0000_0001, 0x0000_0010, 0x0F60_0000]);

        // A slot that is written and stays written.
        for (i, w) in rule.to_regs().iter().enumerate() {
            p.write_data(i, *w);
        }
        p.write_cmd(CMD_GO | CMD_WRITE | (8 << 12) | 0x34);

        // A slot that is written, then padded over with zeros.
        for (i, w) in rule.to_regs().iter().enumerate() {
            p.write_data(i, *w);
        }
        p.write_cmd(CMD_GO | CMD_WRITE | (8 << 12) | 0x2E);
        for i in 0..4 {
            p.write_data(i, 0);
        }
        p.write_cmd(CMD_GO | CMD_WRITE | (8 << 12) | 0x2E);

        // A slot only ever written with zeros: this is the padding the
        // start-up path emits, and it must not look like a lost rule.
        p.write_cmd(CMD_GO | CMD_WRITE | (8 << 12) | 0x2F);

        // The table cannot tell the last two apart.
        assert_eq!(p.entry(8, 0x2E), None);
        assert_eq!(p.entry(8, 0x2F), None);
        assert_eq!(p.entry_count(8), 1);
        assert_eq!(p.counters.writes_of_zero, 2, "two zero writes, one rule lost");

        // The journal can.
        let cleared = p.cleared_slots();
        assert_eq!(cleared.len(), 1, "only one of the two zero writes lost a rule");
        assert_eq!(cleared[0].table, 8);
        assert_eq!(cleared[0].index, 0x2E);
        assert_eq!(cleared[0].log.cleared_content, Some(rule));
        assert_eq!(cleared[0].log.writes, 2);
        assert_eq!(cleared[0].log.clears, 1);
        assert_eq!(cleared[0].log.first_write_at, 2, "the second command of the run");
        assert_eq!(cleared[0].log.last_clear_at, Some(3));

        // The surviving slot is logged too, and was never cleared.
        let live = p.entry_log(8, 0x34).expect("a written slot is logged");
        assert_eq!(live.writes, 1);
        assert_eq!(live.clears, 0);
        assert_eq!(live.cleared_content, None);
        assert_eq!(live.first_write_at, 1);

        // Reading does not disturb it, and a slot nothing wrote has none.
        p.write_cmd(CMD_GO | (8 << 12) | 0x34);
        assert_eq!(p.entry_log(8, 0x34).map(|l| l.writes), Some(1));
        assert_eq!(p.entry_log(8, 0x99), None);
        assert_eq!(p.entry_logs().len(), 3);

        // And a reset takes the journal with it.
        p.reset();
        assert!(p.entry_logs().is_empty());
    }
}
