//! Generic lane indirect bus — shared CMD/DATA/STAT protocol.
//!
//! The BCM55030 lane state table at base `0x010000F8` provides
//! multiple indirect register buses, each with the same 3-register
//! protocol:
//!
//!   * **CMD**  (`lane_base + 0x48`) — command + address. Bits\[31:27\]
//!     are command flags that auto-clear (deferred: on next read).
//!   * **DATA** (`lane_base + 0x54`) — write: input data.
//!     Read: result of last command.
//!   * **STAT** (`lane_base + 0x58`) — status. Bit 31 = busy overlay.
//!
//! Known lanes:
//!   * Lane 0 → BSC I2C (SFP EEPROM at slave 0x50/0x51)
//!   * Lane 8 → MPCP indirect (SerDes internal register file)

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum BusState {
    Idle,
    Busy,
    Done,
}

/// Indirect register file capacity (512 entries, 9-bit index).
const INDIRECT_REG_COUNT: usize = 512;

// ── Lane register file and the RX calibration it carries ────────────────
//
// The three MMIO words above are only the *transport*. What software
// actually addresses is a file of 8-bit lane registers (`0xA0`, `0xBA`,
// `0xBB`, …) reached one byte per transaction. A logical read of `0xBB`
// costs ten MMIO accesses; the register number rides in the staged word
// (`regs[0]`) and the result comes back in `regs[0x100]`.
//
// -- OBSERVED, disassembly of the lane bring-up path plus a full-boot MMIO
// trace of the transport: the staged word is `reg` for a read and
// `(value << 8) | reg` for a write, and the arming word distinguishes the
// two (see `arm_transaction`).

/// Lane register file size — the register number is one byte.
const LANE_REG_COUNT: usize = 256;

/// How many register files the transport addresses.
///
/// The register number is one byte, but it is not the whole address: a
/// two-bit mode selector picks *which* file the byte lands in. The
/// selector is not carried on this bus — software latches it in a
/// separate MMIO word and re-arms it before every transaction, so the
/// bus has to be told (`set_page`).
///
/// -- OBSERVED: the same indirect register numbers appear twice in a
/// silicon register dump, once per context, and 59 of the 138 numbers
/// present in both hold **different** values. One file cannot represent
/// that. Two of the four selector values carry traffic; the third is the
/// transient a read-modify-write of the selector word passes through.
pub const LANE_PAGE_COUNT: usize = 4;

/// RX calibration control register. Even lanes use `0xBA`, odd `0xDA`.
/// Bit 0 = enable, bit 1 = direction, bit 2 = select the cold algorithm.
/// -- OBSERVED: `0x00 -> 0x07` on this register is what starts a cold RX
/// calibration (enable + direction + cold).
const CAL_CTRL_EVEN: u8 = 0xBA;
const CAL_CTRL_ODD: u8 = 0xDA;
/// RX calibration status register, paired with the control register above.
/// Bit 7 = calibration done, active high.
/// -- OBSERVED: hardware sets it, a read does not clear it, and no firmware
/// ever writes it. It is a sticky status bit.
const CAL_STAT_EVEN: u8 = 0xBB;
const CAL_STAT_ODD: u8 = 0xDB;

const CAL_ENABLE_BIT: u8 = 0x01;
const CAL_COLD_BIT: u8 = 0x04;
const CAL_DONE_BIT: u8 = 0x80;

/// How many reads of the status register a cold calibration takes to
/// converge.
///
/// -- OBSERVED on silicon: the count varies widely between runs (a
/// spread of roughly 8x) and its cause is not established. A model
/// cannot reproduce a distribution it does not understand, so this
/// picks the fastest measured convergence: it bounds a wait
/// conservatively, since a firmware whose timeout survives here would
/// still have to survive the slow end on hardware.
/// `set_cal_reads_to_done` moves it anywhere else in the measured range.
pub const DEFAULT_CAL_READS_TO_DONE: u32 = 14_693;

/// Per-lane-parity calibration state. There are two independent engines on
/// a bus, addressed through the even (`0xBA`/`0xBB`) and odd (`0xDA`/`0xDB`)
/// register pairs.
#[derive(Clone, Debug, PartialEq, Eq)]
enum CalState {
    /// Not armed. The status bit is whatever it was.
    Idle,
    /// Armed and converging: `reads_left` status reads remain.
    Running { reads_left: u32 },
    /// Converged — the status bit is set and stays set until software
    /// clears the enable bit.
    Done,
}

#[derive(Clone, Debug)]
pub struct LaneBus {
    pub cmd_addr: u32,
    pub data_addr: u32,
    pub stat_addr: u32,

    cmd: u32,
    data: u32,
    stat: u32,

    pub state: BusState,
    busy_counter: u8,
    busy_ticks: u8,
    cmd_pending_clear: u32,

    /// Indirect register file. CMD writes with cmd=2 (bit 28) store
    /// DATA into `regs[reg_idx]`. CMD writes with cmd=1 (bit 27) load
    /// `regs[reg_idx]` into DATA.
    regs: Vec<u32>,

    /// The lane registers themselves — what the transport above carries.
    /// One file per page, laid out `page * LANE_REG_COUNT + reg`.
    lane_regs: Vec<u8>,
    /// Which register file the next transaction addresses. Set from the
    /// mode selector, which lives outside this bus.
    page: usize,
    /// Calibration engines, per page: `[0]` = even pair (`0xBA`/`0xBB`),
    /// `[1]` = odd pair (`0xDA`/`0xDB`).
    ///
    /// Per page, not global: each context has its own pair of engines.
    /// Sharing them let one context see the other's done bit — which
    /// happened to be harmless only because every arming sequence starts
    /// by writing zero to the control register, and the model reads that
    /// as "disarm and drop the done bit". A sequence that polled without
    /// disarming first would have read DONE immediately.
    cal: [[CalState; 2]; LANE_PAGE_COUNT],
    /// Status reads a cold calibration takes to converge.
    cal_reads_to_done: u32,
    /// When set, an armed calibration converges on its first status read
    /// instead of after `cal_reads_to_done`. For harnesses that need to get
    /// past the wait rather than measure it; never the default.
    cal_immediate: bool,
}

impl LaneBus {
    pub fn new(base: u32, busy_ticks: u8) -> Self {
        Self {
            cmd_addr: base + 0x48,
            data_addr: base + 0x54,
            stat_addr: base + 0x58,
            cmd: 0,
            data: 0,
            stat: 0,
            state: BusState::Idle,
            busy_counter: 0,
            busy_ticks,
            cmd_pending_clear: 0,
            regs: vec![0u32; INDIRECT_REG_COUNT],
            lane_regs: vec![0u8; LANE_REG_COUNT * LANE_PAGE_COUNT],
            page: 0,
            cal: [const { [CalState::Idle, CalState::Idle] }; LANE_PAGE_COUNT],
            cal_reads_to_done: DEFAULT_CAL_READS_TO_DONE,
            cal_immediate: false,
        }
    }

    /// Move the modelled convergence point within the measured range, or
    /// past it — a scenario reproducing a lane that never converges sets
    /// this to `u32::MAX`.
    pub fn set_cal_reads_to_done(&mut self, reads: u32) {
        self.cal_reads_to_done = reads;
    }

    /// Converge on the first status read after arming. Harness escape
    /// hatch, off by default.
    pub fn set_cal_immediate(&mut self, on: bool) {
        self.cal_immediate = on;
    }

    /// Select which register file subsequent transactions address.
    ///
    /// Out-of-range values are ignored rather than wrapped: the selector
    /// is two bits wide at its source, so a wider value means the caller
    /// decoded the wrong field, and silently aliasing it onto a live page
    /// would be the very fault this split exists to remove.
    pub fn set_page(&mut self, page: usize) {
        if page < LANE_PAGE_COUNT {
            self.page = page;
        }
    }

    pub fn page(&self) -> usize {
        self.page
    }

    #[inline]
    fn lane_slot(page: usize, reg: u8) -> usize {
        page * LANE_REG_COUNT + reg as usize
    }

    /// Read a lane register on the current page, without side effects.
    pub fn lane_reg(&self, reg: u8) -> u8 {
        self.lane_regs[Self::lane_slot(self.page, reg)]
    }

    /// Read a lane register on a named page — for tests and inspection
    /// that need to look at the file the bus is not pointing at.
    pub fn lane_reg_on_page(&self, page: usize, reg: u8) -> u8 {
        self.lane_regs[Self::lane_slot(page.min(LANE_PAGE_COUNT - 1), reg)]
    }

    pub fn claims(&self, addr: u32) -> bool {
        addr == self.cmd_addr || addr == self.data_addr || addr == self.stat_addr
    }

    // ── CMD ─────────────────────────────────────────────────────────

    pub fn write_cmd(&mut self, val: u32) {
        self.cmd = val;
        let cmd_bits = val & 0xF800_0000;
        if cmd_bits != 0 {
            self.cmd_pending_clear = cmd_bits;
        }
        self.execute_indirect(val);
    }

    /// Execute an indirect register operation from the CMD value.
    /// CMD encoding: `(reg_idx & 0x1FF) << 18 | (mode & 3) << 16 |
    /// data_low7 | (cmd << 27)`.
    /// cmd bit 28 = write DATA → regs[reg_idx].
    /// cmd bit 27 = read regs[reg_idx] → DATA.
    fn execute_indirect(&mut self, val: u32) {
        let cmd_hi = (val >> 27) & 0x1F;
        let reg_idx = ((val >> 18) & 0x1FF) as usize;
        if reg_idx >= INDIRECT_REG_COUNT {
            return;
        }
        if cmd_hi & 2 != 0 {
            self.regs[reg_idx] = self.data;
        }
        if cmd_hi & 1 != 0 {
            self.data = self.regs[reg_idx];
        }
    }

    /// Clear command bits immediately (BSC does this after processing).
    pub fn clear_cmd_bits_now(&mut self) {
        self.cmd &= 0x07FF_FFFF;
        self.cmd_pending_clear = 0;
    }

    pub fn read_cmd(&mut self) -> u32 {
        let val = self.cmd;
        if self.cmd_pending_clear != 0 {
            self.cmd &= !self.cmd_pending_clear;
            self.cmd_pending_clear = 0;
        }
        val
    }

    pub fn peek_cmd(&self) -> u32 {
        self.cmd
    }

    pub fn cmd_hi(&self) -> u8 {
        ((self.cmd >> 27) & 0x1F) as u8
    }

    // ── DATA ────────────────────────────────────────────────────────

    pub fn write_data(&mut self, val: u32) {
        self.data = val;
    }

    pub fn read_data(&self) -> u32 {
        self.data
    }

    pub fn set_data(&mut self, val: u32) {
        self.data = val;
    }

    // ── STAT ────────────────────────────────────────────────────────

    pub fn write_stat(&mut self, val: u32) {
        self.stat = val & !1;
    }

    pub fn read_stat(&self) -> u32 {
        let mut val = self.stat;
        if matches!(self.state, BusState::Busy) {
            val |= 0x8000_0000;
        } else {
            val &= !0x8000_0000;
        }
        val
    }

    pub fn peek_stat(&self) -> u32 {
        self.read_stat()
    }

    pub fn raw_stat(&self) -> u32 {
        self.stat
    }

    pub fn set_raw_stat(&mut self, val: u32) {
        self.stat = val;
    }

    // ── Indirect register file ────────────────────────────────────

    pub fn reg(&self, idx: usize) -> u32 {
        self.regs.get(idx).copied().unwrap_or(0)
    }

    pub fn set_reg(&mut self, idx: usize, val: u32) {
        if idx < self.regs.len() {
            self.regs[idx] = val;
        }
    }

    pub fn regs_slice(&self, start: usize, len: usize) -> &[u32] {
        let end = (start + len).min(self.regs.len());
        &self.regs[start..end]
    }

    // ── Lane registers: the logical transaction ─────────────────────

    /// Execute the lane-register transaction that the arming word starts.
    ///
    /// The transport has already staged the operand in `regs[0]` (a CMD
    /// write with the store flag and index 0). This word says what to do
    /// with it. -- OBSERVED, full-boot MMIO trace of the three arming words
    /// the bring-up path emits, cross-checked against the disassembly:
    ///
    /// | arming word  | meaning                                     |
    /// |--------------|---------------------------------------------|
    /// | `0x0000000B` | set the read pointer — no data moves        |
    /// | `0x00400009` | execute the read; result lands in `regs[0x100]` |
    /// | `0x00000013` | execute the write                           |
    ///
    /// The read is distinguished by bit 22 (the `0x100` command field
    /// shifted into place); the write by the operand-length field
    /// `bits[13:3] == 2`, one byte of data on top of the register number.
    pub fn arm_transaction(&mut self, arm: u32) {
        let staged = self.regs[0];
        let reg = (staged & 0xFF) as u8;
        if arm & (1 << 22) != 0 {
            let val = self.lane_read(reg);
            self.regs[0x100] = val as u32;
        } else if (arm >> 3) & 0x7FF == 2 {
            let val = ((staged >> 8) & 0xFF) as u8;
            self.lane_write(reg, val);
        }
    }

    /// A lane-register write, with the side effects the hardware has.
    fn lane_write(&mut self, reg: u8, val: u8) {
        let page = self.page;
        self.lane_regs[Self::lane_slot(page, reg)] = val;

        // Arming a cold calibration: the control register takes
        // enable + cold together. -- OBSERVED: `0x00 -> 0x07` on `0xBA`
        // starts one, and one pass is enough (a reading that suggested the
        // command had to be re-issued was a guard cutting in early).
        let parity = match reg {
            CAL_CTRL_EVEN => Some(0usize),
            CAL_CTRL_ODD => Some(1usize),
            _ => None,
        };
        if let Some(p) = parity {
            let stat_reg = if p == 0 { CAL_STAT_EVEN } else { CAL_STAT_ODD };
            if val & CAL_ENABLE_BIT == 0 {
                // Disabled: the engine stops and its status bit goes away.
                self.cal[page][p] = CalState::Idle;
                self.lane_regs[Self::lane_slot(page, stat_reg)] &= !CAL_DONE_BIT;
            } else if val & CAL_COLD_BIT != 0 && self.cal[page][p] != CalState::Done {
                self.cal[page][p] = CalState::Running {
                    reads_left: if self.cal_immediate { 1 } else { self.cal_reads_to_done },
                };
            }
            // Note what is deliberately absent: clearing bit 2 does NOT end
            // a calibration, and nothing here clears bit 2 either.
            // -- OBSERVED: the control register reads `0x07` for as long as
            // the done bit is missing and `0x03` only after firmware clears
            // bit 2 itself. The hardware never touches it.
        }
    }

    /// A lane-register read, with the side effects the hardware has.
    fn lane_read(&mut self, reg: u8) -> u8 {
        let page = self.page;
        let parity = match reg {
            CAL_STAT_EVEN => Some(0usize),
            CAL_STAT_ODD => Some(1usize),
            _ => None,
        };
        if let Some(p) = parity {
            if let CalState::Running { reads_left } = self.cal[page][p] {
                let left = reads_left.saturating_sub(1);
                if left == 0 {
                    self.cal[page][p] = CalState::Done;
                    self.lane_regs[Self::lane_slot(page, reg)] |= CAL_DONE_BIT;
                } else {
                    self.cal[page][p] = CalState::Running { reads_left: left };
                }
            }
        }
        self.lane_regs[Self::lane_slot(page, reg)]
    }

    // ── Bus state ───────────────────────────────────────────────────

    pub fn go_busy(&mut self) {
        self.state = BusState::Busy;
        self.busy_counter = self.busy_ticks;
    }

    pub fn is_busy(&self) -> bool {
        matches!(self.state, BusState::Busy)
    }

    pub fn is_done(&self) -> bool {
        matches!(self.state, BusState::Done)
    }

    pub fn set_idle(&mut self) {
        self.state = BusState::Idle;
    }

    pub fn tick(&mut self) {
        if matches!(self.state, BusState::Busy) && self.busy_counter > 0 {
            self.busy_counter -= 1;
            if self.busy_counter == 0 {
                self.state = BusState::Done;
            }
        }
    }

    pub fn reset(&mut self) {
        self.cmd = 0;
        self.data = 0;
        self.stat = 0;
        self.state = BusState::Idle;
        self.busy_counter = 0;
        self.regs.fill(0);
        self.cmd_pending_clear = 0;
        self.lane_regs.fill(0);
        self.page = 0;
        self.cal = [const { [CalState::Idle, CalState::Idle] }; LANE_PAGE_COUNT];
    }

    /// Apply silicon power-on values for the CMD/DATA/STAT registers.
    pub fn apply_init(&mut self, init_values: &[(u32, u32)]) {
        let base = self.cmd_addr & 0xFFFF_FF00;
        for &(off, val) in init_values {
            let abs = base + off;
            if abs == self.cmd_addr {
                self.cmd = val;
            } else if abs == self.data_addr {
                self.data = val;
            } else if abs == self.stat_addr {
                self.stat = val;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Drive one logical lane-register transaction the way the bring-up
    /// path does: stage the operand through the transport, then arm.
    fn stage(bus: &mut LaneBus, operand: u32) {
        bus.write_data(operand);
        bus.write_cmd(0x1002_002A); // store flag, index 0
    }

    fn lane_write_txn(bus: &mut LaneBus, reg: u8, val: u8) {
        stage(bus, ((val as u32) << 8) | reg as u32);
        bus.arm_transaction(0x0000_0013);
    }

    fn lane_read_txn(bus: &mut LaneBus, reg: u8) -> u8 {
        stage(bus, reg as u32);
        bus.arm_transaction(0x0000_000B); // pointer set: moves nothing
        bus.arm_transaction(0x0040_0009); // execute
        bus.write_cmd(0x0C02_002A); // load flag, index 0x100
        bus.read_data() as u8
    }

    #[test]
    fn lane_registers_round_trip_through_the_transport() {
        let mut bus = LaneBus::new(0x0100_0118, 2);
        lane_write_txn(&mut bus, 0xA1, 0x07);
        assert_eq!(lane_read_txn(&mut bus, 0xA1), 0x07);
        // A register nobody wrote reads zero, not the last operand.
        assert_eq!(lane_read_txn(&mut bus, 0xA2), 0x00);
    }

    /// The two contexts hold different values at the same register
    /// numbers — a silicon dump shows 59 of 138 shared numbers differing.
    /// One file cannot represent that, and the failure is not symmetric:
    /// the second writer wins and the first reads back a value it never
    /// wrote, then read-modify-writes it back into its own configuration.
    #[test]
    fn each_page_keeps_its_own_register_file() {
        let mut bus = LaneBus::new(0x0100_0118, 2);

        bus.set_page(2);
        lane_write_txn(&mut bus, 0x16, 0x0E);
        bus.set_page(1);
        lane_write_txn(&mut bus, 0x16, 0x00);

        bus.set_page(2);
        assert_eq!(lane_read_txn(&mut bus, 0x16), 0x0E);
        bus.set_page(1);
        assert_eq!(lane_read_txn(&mut bus, 0x16), 0x00);
    }

    /// A register the other page wrote must read as untouched here, not
    /// as that value: this is the read that produced a wrong
    /// read-modify-write on the live configuration.
    #[test]
    fn a_page_does_not_see_what_another_page_wrote() {
        let mut bus = LaneBus::new(0x0100_0118, 2);
        bus.set_page(1);
        lane_write_txn(&mut bus, 0x16, 0x0E);
        bus.set_page(2);
        assert_eq!(lane_read_txn(&mut bus, 0x16), 0x00);
        assert_eq!(bus.lane_reg_on_page(1, 0x16), 0x0E);
    }

    /// Each page has its own calibration engines. Sharing them let a
    /// page read a done bit it never earned, as soon as another page had
    /// converged — which stayed invisible only because every arming
    /// sequence happens to disarm first.
    #[test]
    fn a_converged_calibration_does_not_leak_to_another_page() {
        let mut bus = LaneBus::new(0x0100_0118, 2);
        bus.set_cal_immediate(true);

        bus.set_page(2);
        lane_write_txn(&mut bus, CAL_CTRL_EVEN, 0x07);
        assert_eq!(lane_read_txn(&mut bus, CAL_STAT_EVEN) & CAL_DONE_BIT, CAL_DONE_BIT);

        // Same engine number, other page: nothing was armed there.
        bus.set_page(1);
        assert_eq!(lane_read_txn(&mut bus, CAL_STAT_EVEN) & CAL_DONE_BIT, 0);
    }

    /// The selector is two bits at its source. A wider value means the
    /// caller decoded the wrong field; aliasing it onto a live page is
    /// exactly the fault the split removes.
    #[test]
    fn an_out_of_range_page_is_refused_not_wrapped() {
        let mut bus = LaneBus::new(0x0100_0118, 2);
        bus.set_page(2);
        bus.set_page(LANE_PAGE_COUNT + 2);
        assert_eq!(bus.page(), 2);
    }

    #[test]
    fn a_pointer_set_moves_no_data() {
        let mut bus = LaneBus::new(0x0100_0118, 2);
        lane_write_txn(&mut bus, 0xA1, 0x07);
        stage(&mut bus, 0xA1);
        bus.arm_transaction(0x0000_000B);
        // Only the execute word fills the result slot.
        assert_eq!(bus.reg(0x100), 0);
    }

    /// The status bit is not there for the asking: it appears only after
    /// software arms a calibration, and only once it has converged.
    #[test]
    fn calibration_done_requires_arming_and_converging() {
        let mut bus = LaneBus::new(0x0100_0118, 2);
        bus.set_cal_reads_to_done(4);

        // Un-armed: reading the status register forever changes nothing.
        for _ in 0..16 {
            assert_eq!(lane_read_txn(&mut bus, CAL_STAT_EVEN) & CAL_DONE_BIT, 0);
        }

        // Arm a cold calibration: enable + direction + cold.
        lane_write_txn(&mut bus, CAL_CTRL_EVEN, 0x07);
        for i in 1..4 {
            assert_eq!(
                lane_read_txn(&mut bus, CAL_STAT_EVEN) & CAL_DONE_BIT,
                0,
                "converged early, on read {i}"
            );
        }
        assert_eq!(lane_read_txn(&mut bus, CAL_STAT_EVEN) & CAL_DONE_BIT, CAL_DONE_BIT);
        // Sticky: it stays across further reads.
        assert_eq!(lane_read_txn(&mut bus, CAL_STAT_EVEN) & CAL_DONE_BIT, CAL_DONE_BIT);
    }

    /// Enable alone does not start one — the cold bit selects the
    /// algorithm that produces the status bit.
    #[test]
    fn enable_without_the_cold_bit_does_not_converge() {
        let mut bus = LaneBus::new(0x0100_0118, 2);
        bus.set_cal_reads_to_done(2);
        lane_write_txn(&mut bus, CAL_CTRL_EVEN, 0x03); // enable + direction
        for _ in 0..8 {
            assert_eq!(lane_read_txn(&mut bus, CAL_STAT_EVEN) & CAL_DONE_BIT, 0);
        }
    }

    /// The even and odd engines are independent: arming one leaves the
    /// other's status alone.
    #[test]
    fn the_two_parities_are_independent() {
        let mut bus = LaneBus::new(0x0100_0118, 2);
        bus.set_cal_reads_to_done(1);
        lane_write_txn(&mut bus, CAL_CTRL_ODD, 0x07);
        assert_eq!(lane_read_txn(&mut bus, CAL_STAT_ODD) & CAL_DONE_BIT, CAL_DONE_BIT);
        assert_eq!(lane_read_txn(&mut bus, CAL_STAT_EVEN) & CAL_DONE_BIT, 0);
    }

    /// -- OBSERVED: the control register reads 0x07 for as long as the done
    /// bit is missing and 0x03 only after firmware clears bit 2 itself.
    /// Clearing it is not what ends the calibration, and the hardware never
    /// clears it.
    #[test]
    fn the_hardware_never_clears_the_cold_bit() {
        let mut bus = LaneBus::new(0x0100_0118, 2);
        bus.set_cal_reads_to_done(1);
        lane_write_txn(&mut bus, CAL_CTRL_EVEN, 0x07);
        assert_eq!(lane_read_txn(&mut bus, CAL_CTRL_EVEN), 0x07);
        lane_read_txn(&mut bus, CAL_STAT_EVEN); // converge
        assert_eq!(lane_read_txn(&mut bus, CAL_CTRL_EVEN), 0x07, "hardware cleared bit 2");
        // Software clears it, and only then does it read back 0x03.
        lane_write_txn(&mut bus, CAL_CTRL_EVEN, 0x03);
        assert_eq!(lane_read_txn(&mut bus, CAL_CTRL_EVEN), 0x03);
        // ...and the status bit survives that clear.
        assert_eq!(lane_read_txn(&mut bus, CAL_STAT_EVEN) & CAL_DONE_BIT, CAL_DONE_BIT);
    }

    /// Dropping the enable bit stops the engine and takes the status with
    /// it, so a re-armed lane starts from not-converged.
    #[test]
    fn clearing_enable_retracts_the_status_bit() {
        let mut bus = LaneBus::new(0x0100_0118, 2);
        bus.set_cal_reads_to_done(1);
        lane_write_txn(&mut bus, CAL_CTRL_EVEN, 0x07);
        lane_read_txn(&mut bus, CAL_STAT_EVEN);
        assert_eq!(lane_read_txn(&mut bus, CAL_STAT_EVEN) & CAL_DONE_BIT, CAL_DONE_BIT);
        lane_write_txn(&mut bus, CAL_CTRL_EVEN, 0x00);
        assert_eq!(lane_read_txn(&mut bus, CAL_STAT_EVEN) & CAL_DONE_BIT, 0);
    }

    #[test]
    fn reset_clears_the_lane_registers_and_the_engines() {
        let mut bus = LaneBus::new(0x0100_0118, 2);
        bus.set_cal_reads_to_done(1);
        lane_write_txn(&mut bus, CAL_CTRL_EVEN, 0x07);
        lane_read_txn(&mut bus, CAL_STAT_EVEN);
        bus.reset();
        assert_eq!(bus.lane_reg(CAL_CTRL_EVEN), 0);
        assert_eq!(lane_read_txn(&mut bus, CAL_STAT_EVEN) & CAL_DONE_BIT, 0);
    }
}
