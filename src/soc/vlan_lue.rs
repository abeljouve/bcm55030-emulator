//! VLAN / EtherType / LUE indirect access table (block 8).
//!
//! Registers `0x01003000..0x01003020`:
//!   * `0x01003000` — VLAN_CTRL (RMW)
//!   * `0x0100300C` — CUSTOM_VLAN_ETHERTYPE (RMW)
//!   * `0x01003010` — INDIRECT_CMD (RW, busy/index/rw)
//!   * `0x01003014` — INDIRECT_DATA_2 (RW, MSB of 96-bit payload)
//!   * `0x01003018` — INDIRECT_DATA_1 (RW)
//!   * `0x0100301C` — INDIRECT_DATA_0 (RW, LSB of 96-bit payload)
//!
//! Pattern 3 (indirect access): write index + direction to CMD,
//! then read/write DATA registers. The firmware programs classifier
//! rules during init and reads them back via `lue/print`.

use crate::cpu::exception::Exception;

const BASE: u32 = 0x0100_3000;
const END: u32 = 0x0100_3020;
const OFF_VLAN_CTRL: u32 = 0x00;
const OFF_CUSTOM_ETHERTYPE: u32 = 0x0C;
const OFF_CMD: u32 = 0x10;
const OFF_DATA2: u32 = 0x14;
const OFF_DATA1: u32 = 0x18;
const OFF_DATA0: u32 = 0x1C;

const TABLE_SIZE: usize = 256;

#[derive(Clone, Copy, Default)]
struct TableEntry {
    data2: u32,
    data1: u32,
    data0: u32,
}

pub struct VlanLue {
    vlan_ctrl: u32,
    custom_ethertype: u32,
    cmd: u32,
    data2: u32,
    data1: u32,
    data0: u32,
    table: Vec<TableEntry>,
    regs_04_08: [u32; 2],
}

impl VlanLue {
    pub fn new() -> Self {
        Self {
            vlan_ctrl: 0,
            custom_ethertype: 0,
            cmd: 0,
            data2: 0,
            data1: 0,
            data0: 0,
            table: vec![TableEntry::default(); TABLE_SIZE],
            regs_04_08: [0; 2],
        }
    }

    #[inline]
    pub fn claims(&self, addr: u32) -> bool {
        (BASE..END).contains(&addr)
    }

    pub fn reset_cold(&mut self) {
        self.vlan_ctrl = 0;
        self.custom_ethertype = 0;
        self.cmd = 0;
        self.data2 = 0;
        self.data1 = 0;
        self.data0 = 0;
        for e in &mut self.table {
            *e = TableEntry::default();
        }
        self.regs_04_08 = [0; 2];
    }

    pub fn reset_warm(&mut self) {
        self.reset_cold();
        for &(off, val) in super::mmio_init::SYSREG_INIT_VALUES {
            let abs = 0x0100_0000 + off;
            if (BASE..END).contains(&abs) {
                self.apply_init(abs - BASE, val);
            }
        }
    }

    fn apply_init(&mut self, off: u32, val: u32) {
        match off {
            OFF_VLAN_CTRL => self.vlan_ctrl = val,
            OFF_CUSTOM_ETHERTYPE => self.custom_ethertype = val,
            OFF_CMD => self.cmd = val,
            OFF_DATA2 => self.data2 = val,
            OFF_DATA1 => self.data1 = val,
            OFF_DATA0 => self.data0 = val,
            0x04 => self.regs_04_08[0] = val,
            0x08 => self.regs_04_08[1] = val,
            _ => {}
        }
    }

    pub fn tick(&mut self, _cpu_instructions: u64) {}

    pub fn read_word(&mut self, addr: u32) -> Result<u32, Exception> {
        let off = addr - BASE;
        Ok(match off {
            OFF_VLAN_CTRL => self.vlan_ctrl,
            OFF_CUSTOM_ETHERTYPE => self.custom_ethertype,
            OFF_CMD => self.cmd & !0xC000_0000,
            OFF_DATA2 => self.data2,
            OFF_DATA1 => self.data1,
            OFF_DATA0 => self.data0,
            0x04 => self.regs_04_08[0],
            0x08 => self.regs_04_08[1],
            _ => 0,
        })
    }

    pub fn write_word(&mut self, addr: u32, val: u32) -> Result<(), Exception> {
        let off = addr - BASE;
        match off {
            OFF_VLAN_CTRL => self.vlan_ctrl = val,
            OFF_CUSTOM_ETHERTYPE => self.custom_ethertype = val,
            OFF_CMD => {
                let index = (val & 0xFF) as usize;
                let is_read = (val & 0x8000_0000) != 0;
                let is_write = (val & 0x4000_0000) != 0;

                if is_write && index < TABLE_SIZE {
                    self.table[index] = TableEntry {
                        data2: self.data2,
                        data1: self.data1,
                        data0: self.data0,
                    };
                } else if is_read && index < TABLE_SIZE {
                    let entry = self.table[index];
                    self.data2 = entry.data2;
                    self.data1 = entry.data1;
                    self.data0 = entry.data0;
                }
                self.cmd = val & !0xC000_0000;
            }
            OFF_DATA2 => self.data2 = val,
            OFF_DATA1 => self.data1 = val,
            OFF_DATA0 => self.data0 = val,
            0x04 => self.regs_04_08[0] = val,
            0x08 => self.regs_04_08[1] = val,
            _ => {}
        }
        Ok(())
    }

    pub fn read_half(&mut self, addr: u32) -> Result<u16, Exception> {
        let word = self.read_word(addr & !3)?;
        let half_idx = (addr >> 1) & 1;
        Ok((word >> (16 - half_idx * 16)) as u16)
    }

    pub fn write_half(&mut self, addr: u32, val: u16) -> Result<(), Exception> {
        let word_addr = addr & !3;
        let half_idx = (addr >> 1) & 1;
        let old = self.read_word(word_addr)?;
        let shift = 16 - half_idx * 16;
        let mask = !(0xFFFFu32 << shift);
        let new = (old & mask) | ((val as u32) << shift);
        self.write_word(word_addr, new)
    }

    pub fn read_byte(&mut self, addr: u32) -> Result<u8, Exception> {
        let word = self.read_word(addr & !3)?;
        let byte_idx = addr & 3;
        Ok((word >> (24 - byte_idx * 8)) as u8)
    }

    pub fn write_byte(&mut self, addr: u32, val: u8) -> Result<(), Exception> {
        let word_addr = addr & !3;
        let byte_idx = addr & 3;
        let old = self.read_word(word_addr)?;
        let shift = 24 - byte_idx * 8;
        let mask = !(0xFFu32 << shift);
        let new = (old & mask) | ((val as u32) << shift);
        self.write_word(word_addr, new)
    }

    pub fn peek_word(&self, addr: u32) -> Result<u32, Exception> {
        let off = addr - BASE;
        Ok(match off {
            OFF_VLAN_CTRL => self.vlan_ctrl,
            OFF_CUSTOM_ETHERTYPE => self.custom_ethertype,
            OFF_CMD => self.cmd & !0xC000_0000,
            OFF_DATA2 => self.data2,
            OFF_DATA1 => self.data1,
            OFF_DATA0 => self.data0,
            0x04 => self.regs_04_08[0],
            0x08 => self.regs_04_08[1],
            _ => 0,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn indirect_write_then_read() {
        let mut v = VlanLue::new();
        v.write_word(BASE + OFF_DATA2, 0xAAAA_BBBB).unwrap();
        v.write_word(BASE + OFF_DATA1, 0xCCCC_DDDD).unwrap();
        v.write_word(BASE + OFF_DATA0, 0xEEEE_FFFF).unwrap();
        // Write to index 5: bit 30 = write command
        v.write_word(BASE + OFF_CMD, 0x4000_0005).unwrap();

        v.write_word(BASE + OFF_DATA2, 0).unwrap();
        v.write_word(BASE + OFF_DATA1, 0).unwrap();
        v.write_word(BASE + OFF_DATA0, 0).unwrap();

        // Read from index 5: bit 31 = read command
        v.write_word(BASE + OFF_CMD, 0x8000_0005).unwrap();

        assert_eq!(v.read_word(BASE + OFF_DATA2).unwrap(), 0xAAAA_BBBB);
        assert_eq!(v.read_word(BASE + OFF_DATA1).unwrap(), 0xCCCC_DDDD);
        assert_eq!(v.read_word(BASE + OFF_DATA0).unwrap(), 0xEEEE_FFFF);
    }

    #[test]
    fn cmd_bits_clear_immediately() {
        let mut v = VlanLue::new();
        v.write_word(BASE + OFF_CMD, 0x8000_0000).unwrap();
        let cmd = v.read_word(BASE + OFF_CMD).unwrap();
        assert_eq!(cmd & 0xC000_0000, 0);
    }

    #[test]
    fn different_indices_independent() {
        let mut v = VlanLue::new();

        v.write_word(BASE + OFF_DATA0, 0x1111).unwrap();
        v.write_word(BASE + OFF_CMD, 0x4000_000A).unwrap();

        v.write_word(BASE + OFF_DATA0, 0x2222).unwrap();
        v.write_word(BASE + OFF_CMD, 0x4000_000B).unwrap();

        v.write_word(BASE + OFF_CMD, 0x8000_000A).unwrap();
        assert_eq!(v.read_word(BASE + OFF_DATA0).unwrap(), 0x1111);

        v.write_word(BASE + OFF_CMD, 0x8000_000B).unwrap();
        assert_eq!(v.read_word(BASE + OFF_DATA0).unwrap(), 0x2222);
    }
}
