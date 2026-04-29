//! BCM55030 Broadcom Serial Controller (BSC) I²C master.
//!
//! The BSC is the on-chip I²C master that talks to the SFP EEPROM
//! (devices `0xA0` and `0xA2`) at MMIO offsets:
//!
//!   * `0x01000140` — CMD register. Bit 28 = sub-addr latch, bit 27 =
//!     read trigger. Bits[26:18] = `(0x100 + word_idx)`. Bits[6:0] =
//!     I2C slave address (0x50 = A0h, 0x51 = A2h). Bits[31:27]
//!     auto-clear after the command is latched.
//!   * `0x0100014C` — DATA register. Write: sets EEPROM sub-address for
//!     the next CMD latch. Read: returns 4-byte result of last word read.
//!   * `0x01000150` — STAT register. Bit 31 = busy. A write with bit 22
//!     set arms a burst read; byte_count in bits[13:3].
//!
//! Protocol (matches silicon):
//!   1. DATA ← sub_addr
//!   2. CMD latch (bit 28) → latches sub_addr, goes busy
//!   3. STAT arm (bit 22, byte_count) → arms burst, sets EEPROM pointer
//!   4. CMD read triggers (bit 27) → sequential words from pointer
//!   5. After byte_count bytes, protocol returns to Idle
//!
//! Reads without proper init (latch + arm) return stale data.

use crate::cpu::exception::Exception;
use crate::soc::peripheral::{
    AddressRange, BscEvent, BscSnapshot, Peripheral, PeripheralError, PeripheralEvent,
    PeripheralSnapshot, SfpSnapshot,
};
use crate::soc::sfp_eeprom::SfpEeprom;

pub const BSC_BASE: u32 = 0x01000140;
pub const BSC_END: u32 = 0x01000158;

const REG_CMD: u32 = 0x140;
const REG_BASE_ADDR: u32 = 0x14C;
const REG_STATUS: u32 = 0x150;

const BSC_RANGES: &[AddressRange] = &[AddressRange::new(BSC_BASE, BSC_END)];

/// Number of bank ticks the busy bit stays asserted after a command.
const BSC_BUSY_TICKS: u8 = 2;

#[derive(Clone, Debug, PartialEq, Eq)]
enum BusState {
    Idle,
    Busy,
    Done,
}

#[derive(Clone, Debug, PartialEq, Eq)]
enum ProtocolState {
    Idle,
    SubAddrLatched,
    Armed,
}

#[derive(Clone)]
pub struct BscI2c {
    pub sfp: SfpEeprom,
    state: BusState,
    busy_counter: u8,
    protocol: ProtocolState,
    pending_sub_addr: u16,
    sub_addr: u16,
    eeprom_ptr: u16,
    byte_count: u16,
    bytes_read: u16,
    last_device: u8,
    pending_read_word: u32,
    force_nack: bool,
    raw_store: [u32; 6],
}

impl BscI2c {
    pub fn new() -> Self {
        Self {
            sfp: SfpEeprom::new_default(),
            state: BusState::Idle,
            busy_counter: 0,
            protocol: ProtocolState::Idle,
            pending_sub_addr: 0,
            sub_addr: 0,
            eeprom_ptr: 0,
            byte_count: 0,
            bytes_read: 0,
            last_device: 0,
            pending_read_word: 0,
            force_nack: false,
            raw_store: [0u32; 6],
        }
    }

    #[inline]
    pub fn claims(&self, addr: u32) -> bool {
        (BSC_BASE..BSC_END).contains(&addr)
    }

    fn store_idx(&self, offset: u32) -> Option<usize> {
        if offset >= 0x140 && offset < 0x158 {
            Some(((offset - 0x140) / 4) as usize)
        } else {
            None
        }
    }

    /// Build a snapshot of the owned SFP EEPROM's display state. The
    /// bank exposes this as a separate peripheral row so the UI can
    /// render an "SFP DDM" tab without reaching into BSC internals.
    pub fn sfp_snapshot(&self) -> PeripheralSnapshot {
        PeripheralSnapshot::Sfp(SfpSnapshot {
            vendor: self.sfp.vendor_name(),
            serial: self.sfp.serial_number(),
            part_number: self.sfp.part_number(),
            temperature_c256: self.sfp.ddm.temperature_c256,
            vcc_uv: (self.sfp.ddm.vcc_100uv as u32) * 100,
            tx_bias_ua: (self.sfp.ddm.tx_bias_2ua as u32) * 2,
            tx_power_uw: (self.sfp.ddm.tx_power_01uw as u32) / 10,
            rx_power_uw: (self.sfp.ddm.rx_power_01uw as u32) / 10,
        })
    }

    fn issue_command(&mut self, val: u32) {
        self.last_device = (val & 0x1) as u8;
        let cmd_hi = (val >> 27) & 0x1F;

        if cmd_hi & 2 != 0 {
            self.sub_addr = self.pending_sub_addr;
            self.protocol = ProtocolState::SubAddrLatched;
            self.state = BusState::Busy;
            self.busy_counter = BSC_BUSY_TICKS;
        } else if cmd_hi & 1 != 0 {
            if matches!(self.protocol, ProtocolState::Armed) && self.bytes_read < self.byte_count
            {
                self.pending_read_word = self.sfp.read_word(self.last_device, self.eeprom_ptr);
                if self.force_nack {
                    self.pending_read_word = 0xFFFF_FFFF;
                    self.force_nack = false;
                }
                self.eeprom_ptr = self.eeprom_ptr.wrapping_add(4);
                self.bytes_read += 4;
                if self.bytes_read >= self.byte_count {
                    self.protocol = ProtocolState::Idle;
                }
            }
            self.state = BusState::Busy;
            self.busy_counter = BSC_BUSY_TICKS;
        }

        if let Some(i) = self.store_idx(REG_CMD) {
            self.raw_store[i] = val & 0x07FF_FFFF;
        }
    }
}

impl Peripheral for BscI2c {
    fn name(&self) -> &'static str {
        "bsc_i2c"
    }

    fn address_ranges(&self) -> &'static [AddressRange] {
        BSC_RANGES
    }

    fn peek_word(&self, addr: u32) -> Result<u32, Exception> {
        let off = addr - 0x01000000;
        match off {
            REG_CMD => Ok(self
                .store_idx(REG_CMD)
                .map(|i| self.raw_store[i])
                .unwrap_or(0)),
            REG_BASE_ADDR => Ok(self.pending_read_word),
            REG_STATUS => {
                let mut val = self
                    .store_idx(REG_STATUS)
                    .map(|i| self.raw_store[i])
                    .unwrap_or(0);
                if matches!(self.state, BusState::Busy) {
                    val |= 0x8000_0000;
                } else {
                    val &= !0x8000_0000;
                }
                Ok(val)
            }
            _ => Ok(self
                .store_idx(off)
                .map(|i| self.raw_store[i])
                .unwrap_or(0)),
        }
    }

    fn read_word(&mut self, addr: u32) -> Result<u32, Exception> {
        let off = addr - 0x01000000;
        match off {
            REG_CMD => Ok(self
                .store_idx(REG_CMD)
                .map(|i| self.raw_store[i])
                .unwrap_or(0)),
            REG_BASE_ADDR => {
                if matches!(self.state, BusState::Done) {
                    self.state = BusState::Idle;
                }
                Ok(self.pending_read_word)
            }
            REG_STATUS => {
                let mut val = self
                    .store_idx(REG_STATUS)
                    .map(|i| self.raw_store[i])
                    .unwrap_or(0);
                if matches!(self.state, BusState::Busy) {
                    val |= 0x8000_0000;
                } else {
                    val &= !0x8000_0000;
                }
                Ok(val)
            }
            _ => Ok(self
                .store_idx(off)
                .map(|i| self.raw_store[i])
                .unwrap_or(0)),
        }
    }

    fn write_word(&mut self, addr: u32, val: u32) -> Result<(), Exception> {
        let off = addr - 0x01000000;
        if let Some(i) = self.store_idx(off) {
            self.raw_store[i] = val;
        }
        match off {
            REG_CMD => self.issue_command(val),
            REG_BASE_ADDR => self.pending_sub_addr = (val & 0xFFFF) as u16,
            REG_STATUS => {
                if let Some(i) = self.store_idx(REG_STATUS) {
                    self.raw_store[i] = val & !1;
                }
                if val & 0x0040_0000 != 0
                    && matches!(self.protocol, ProtocolState::SubAddrLatched)
                {
                    let bc = ((val >> 3) & 0x7FF) as u16;
                    if bc > 0 {
                        self.byte_count = bc;
                        self.eeprom_ptr = self.sub_addr;
                        self.bytes_read = 0;
                        self.protocol = ProtocolState::Armed;
                        self.state = BusState::Busy;
                        self.busy_counter = BSC_BUSY_TICKS;
                    }
                }
            }
            _ => {}
        }
        Ok(())
    }

    fn tick(&mut self, _cpu_instructions: u64) {
        if matches!(self.state, BusState::Busy) && self.busy_counter > 0 {
            self.busy_counter -= 1;
            if self.busy_counter == 0 {
                self.state = BusState::Done;
            }
        }
    }

    fn reset_cold(&mut self) {
        self.state = BusState::Idle;
        self.busy_counter = 0;
        self.protocol = ProtocolState::Idle;
        self.pending_sub_addr = 0;
        self.sub_addr = 0;
        self.eeprom_ptr = 0;
        self.byte_count = 0;
        self.bytes_read = 0;
        self.last_device = 0;
        self.pending_read_word = 0;
        self.force_nack = false;
        self.raw_store = [0u32; 6];
        self.sfp.reset_to_snapshot();
    }

    fn snapshot(&self) -> PeripheralSnapshot {
        PeripheralSnapshot::Bsc(BscSnapshot {
            busy: matches!(self.state, BusState::Busy),
            last_device_addr: if self.last_device == 0 { 0xA0 } else { 0xA2 },
            last_word_addr: self.eeprom_ptr,
        })
    }

    fn inject_event(&mut self, event: &PeripheralEvent) -> Result<(), PeripheralError> {
        match event {
            PeripheralEvent::Bsc(BscEvent::ForceNack) => {
                self.force_nack = true;
                Ok(())
            }
            PeripheralEvent::Bsc(BscEvent::Reset) => {
                self.state = BusState::Idle;
                self.busy_counter = 0;
                Ok(())
            }
            PeripheralEvent::Sfp(_) => {
                // Forward DDM/identity edits to the SFP sub-device.
                use crate::soc::peripheral::SfpEvent;
                if let PeripheralEvent::Sfp(ev) = event {
                    match ev {
                        SfpEvent::SetTemperatureC256(v) => self.sfp.ddm.temperature_c256 = *v,
                        SfpEvent::SetVccUv(uv) => {
                            self.sfp.ddm.vcc_100uv = (*uv / 100).min(0xFFFF) as u16;
                        }
                        SfpEvent::SetTxBiasUa(ua) => {
                            self.sfp.ddm.tx_bias_2ua = (*ua / 2).min(0xFFFF) as u16;
                        }
                        SfpEvent::SetTxPowerUw(uw) => {
                            self.sfp.ddm.tx_power_01uw = (*uw * 10).min(0xFFFF) as u16;
                        }
                        SfpEvent::SetRxPowerUw(uw) => {
                            self.sfp.ddm.rx_power_01uw = (*uw * 10).min(0xFFFF) as u16;
                        }
                        SfpEvent::SetVendorName(n) => self.sfp.set_vendor_name(*n),
                        SfpEvent::SetSerialNumber(s) => self.sfp.set_serial_number(*s),
                        SfpEvent::SetPartNumber(p) => self.sfp.set_part_number(*p),
                    }
                }
                Ok(())
            }
            _ => Err(PeripheralError::UnsupportedEvent),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn data_reg_returns_zero_when_idle() {
        let mut bsc = BscI2c::new();
        bsc.write_word(0x0100014C, 0x0000_000C).unwrap();
        let v = bsc.read_word(0x0100014C).unwrap();
        assert_eq!(v, 0, "DATA register must not echo back written value in Idle");
    }

    #[test]
    fn stat_bit0_auto_clears() {
        let mut bsc = BscI2c::new();
        bsc.write_word(0x01000150, 0x0B).unwrap();
        let v = bsc.read_word(0x01000150).unwrap();
        assert_eq!(v, 0x0A, "STAT bit 0 must auto-clear on write");
    }

    #[test]
    fn device_select_from_slave_addr_bit0() {
        let mut bsc = BscI2c::new();
        bsc.write_word(0x0100014C, 0x0000_0050).unwrap();
        // slave addr 0x51 (A2h) → bit 0 = 1 → device 1
        let cmd = (1u32 << 27) | (0x100u32 << 18) | 0x51;
        bsc.write_word(0x01000140, cmd).unwrap();
        assert_eq!(bsc.last_device, 1);

        // slave addr 0x50 (A0h) → bit 0 = 0 → device 0
        let cmd2 = (1u32 << 27) | (0x100u32 << 18) | 0x50;
        bsc.write_word(0x01000140, cmd2).unwrap();
        assert_eq!(bsc.last_device, 0);
    }

    #[test]
    fn busy_clears_after_ticks() {
        let mut bsc = BscI2c::new();
        bsc.write_word(0x0100014C, 0x0000_0050).unwrap();
        let cmd = (1u32 << 27) | (0x100u32 << 18) | 0x51;
        bsc.write_word(0x01000140, cmd).unwrap();
        let s = bsc.read_word(0x01000150).unwrap();
        assert_ne!(s & 0x8000_0000, 0);
        for _ in 0..BSC_BUSY_TICKS {
            bsc.tick(64);
        }
        let s = bsc.read_word(0x01000150).unwrap();
        assert_eq!(s & 0x8000_0000, 0);
    }

    fn do_init(bsc: &mut BscI2c, sub_addr: u16, byte_count: u16, slave: u32) {
        bsc.write_word(0x0100014C, sub_addr as u32).unwrap();
        bsc.write_word(0x01000140, (2u32 << 27) | slave).unwrap();
        for _ in 0..BSC_BUSY_TICKS {
            bsc.tick(1);
        }
        bsc.write_word(0x01000150, 0x0B).unwrap();
        bsc.write_word(0x01000140, 0x0400_0000 | slave).unwrap();
        let arm_val = 0x0040_0000 | ((byte_count as u32) << 3) | 1;
        bsc.write_word(0x01000150, arm_val).unwrap();
        for _ in 0..BSC_BUSY_TICKS {
            bsc.tick(1);
        }
    }

    #[test]
    fn reference_protocol_reads_correct_data() {
        let mut bsc = BscI2c::new();
        do_init(&mut bsc, 0, 16, 0x50);

        for word_idx in 0u32..4 {
            let cmd = (1u32 << 27) | ((0x100 + word_idx) << 18) | 0x50;
            bsc.write_word(0x01000140, cmd).unwrap();
            for _ in 0..BSC_BUSY_TICKS {
                bsc.tick(1);
            }
            let data = bsc.read_word(0x0100014C).unwrap();
            let expected = bsc.sfp.read_word(0, word_idx as u16 * 4);
            assert_eq!(data, expected, "word {} mismatch", word_idx);
        }
    }

    #[test]
    fn reads_without_init_return_stale() {
        let mut bsc = BscI2c::new();
        let cmd = (1u32 << 27) | (0x100u32 << 18) | 0x50;
        bsc.write_word(0x01000140, cmd).unwrap();
        for _ in 0..BSC_BUSY_TICKS {
            bsc.tick(1);
        }
        let data = bsc.read_word(0x0100014C).unwrap();
        assert_eq!(data, 0, "read without init must return stale (0)");
    }

    #[test]
    fn burst_exhaustion_returns_stale() {
        let mut bsc = BscI2c::new();
        do_init(&mut bsc, 0, 4, 0x50);

        let cmd = (1u32 << 27) | (0x100u32 << 18) | 0x50;
        bsc.write_word(0x01000140, cmd).unwrap();
        for _ in 0..BSC_BUSY_TICKS {
            bsc.tick(1);
        }
        let data1 = bsc.read_word(0x0100014C).unwrap();
        let expected = bsc.sfp.read_word(0, 0);
        assert_eq!(data1, expected);

        let cmd2 = (1u32 << 27) | (0x101u32 << 18) | 0x50;
        bsc.write_word(0x01000140, cmd2).unwrap();
        for _ in 0..BSC_BUSY_TICKS {
            bsc.tick(1);
        }
        let data2 = bsc.read_word(0x0100014C).unwrap();
        assert_eq!(data2, data1, "stale data after burst exhaustion");
    }

    #[test]
    fn a2h_device_select_with_init() {
        let mut bsc = BscI2c::new();
        do_init(&mut bsc, 0, 4, 0x51);

        let cmd = (1u32 << 27) | (0x100u32 << 18) | 0x51;
        bsc.write_word(0x01000140, cmd).unwrap();
        for _ in 0..BSC_BUSY_TICKS {
            bsc.tick(1);
        }
        let data = bsc.read_word(0x0100014C).unwrap();
        let expected = bsc.sfp.read_word(1, 0);
        assert_eq!(data, expected, "A2h word 0 mismatch");
    }

    #[test]
    fn sequential_reads_advance_pointer() {
        let mut bsc = BscI2c::new();
        do_init(&mut bsc, 8, 8, 0x50);

        for word_idx in 0u32..2 {
            let cmd = (1u32 << 27) | ((0x100 + word_idx) << 18) | 0x50;
            bsc.write_word(0x01000140, cmd).unwrap();
            for _ in 0..BSC_BUSY_TICKS {
                bsc.tick(1);
            }
            let data = bsc.read_word(0x0100014C).unwrap();
            let expected = bsc.sfp.read_word(0, 8 + word_idx as u16 * 4);
            assert_eq!(data, expected, "word {} from offset 8", word_idx);
        }
    }
}
