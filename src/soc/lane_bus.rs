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
        }
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

    pub fn regs_slice(&self, start: usize, len: usize) -> &[u32] {
        let end = (start + len).min(self.regs.len());
        &self.regs[start..end]
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
