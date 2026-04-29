//! BCM55030 Broadcom Serial Controller (BSC) I²C master.
//!
//! Lane 0 of the lane state table indirect bus. Talks to the SFP
//! EEPROM (devices `0xA0` and `0xA2`) at MMIO offsets:
//!
//!   * `0x01000140` — CMD register (lane_base + 0x48)
//!   * `0x0100014C` — DATA register (lane_base + 0x54)
//!   * `0x01000150` — STAT register (lane_base + 0x58)
//!
//! Uses the shared `LaneBus` for CMD/DATA/STAT protocol.

use crate::cpu::exception::Exception;
use crate::soc::lane_bus::LaneBus;
use crate::soc::peripheral::{
    AddressRange, BscEvent, BscSnapshot, Peripheral, PeripheralError, PeripheralEvent,
    PeripheralSnapshot, SfpSnapshot,
};
use crate::soc::sfp_eeprom::SfpEeprom;

/// Lane 0 base in the lane state table.
const LANE0_BASE: u32 = 0x0100_00F8;

pub const BSC_BASE: u32 = 0x01000140;
pub const BSC_END: u32 = 0x01000158;

const BSC_RANGES: &[AddressRange] = &[AddressRange::new(BSC_BASE, BSC_END)];

const BSC_BUSY_TICKS: u8 = 2;

#[derive(Clone, Debug, PartialEq, Eq)]
enum ProtocolState {
    Idle,
    SubAddrLatched,
    Armed,
}

#[derive(Clone)]
pub struct BscI2c {
    pub sfp: SfpEeprom,
    pub bus: LaneBus,
    /// Backing store for intermediate addresses (0x144, 0x148, 0x154)
    /// that fall between CMD/DATA/STAT in the BSC range.
    aux_store: [u32; 3],
    protocol: ProtocolState,
    pending_sub_addr: u16,
    sub_addr: u16,
    eeprom_ptr: u16,
    byte_count: u16,
    bytes_read: u16,
    last_device: u8,
    pending_read_word: u32,
    force_nack: bool,
}

impl BscI2c {
    pub fn new() -> Self {
        Self {
            sfp: SfpEeprom::new_default(),
            bus: LaneBus::new(LANE0_BASE, BSC_BUSY_TICKS),
            aux_store: [0; 3],
            protocol: ProtocolState::Idle,
            pending_sub_addr: 0,
            sub_addr: 0,
            eeprom_ptr: 0,
            byte_count: 0,
            bytes_read: 0,
            last_device: 0,
            pending_read_word: 0,
            force_nack: false,
        }
    }

    #[inline]
    pub fn claims(&self, addr: u32) -> bool {
        (BSC_BASE..BSC_END).contains(&addr)
    }

    fn aux_idx(addr: u32) -> Option<usize> {
        match addr {
            0x0100_0144 => Some(0),
            0x0100_0148 => Some(1),
            0x0100_0154 => Some(2),
            _ => None,
        }
    }

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
            self.bus.go_busy();
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
            self.bus.go_busy();
        }

        // BSC clears cmd bits immediately (not deferred like other lanes).
        self.bus.clear_cmd_bits_now();
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
        if addr == self.bus.cmd_addr {
            Ok(self.bus.peek_cmd())
        } else if addr == self.bus.data_addr {
            Ok(self.pending_read_word)
        } else if addr == self.bus.stat_addr {
            Ok(self.bus.read_stat())
        } else if let Some(i) = Self::aux_idx(addr) {
            Ok(self.aux_store[i])
        } else {
            Ok(0)
        }
    }

    fn read_word(&mut self, addr: u32) -> Result<u32, Exception> {
        if addr == self.bus.cmd_addr {
            Ok(self.bus.read_cmd())
        } else if addr == self.bus.data_addr {
            if self.bus.is_done() {
                self.bus.set_idle();
            }
            Ok(self.pending_read_word)
        } else if addr == self.bus.stat_addr {
            Ok(self.bus.read_stat())
        } else if let Some(i) = Self::aux_idx(addr) {
            Ok(self.aux_store[i])
        } else {
            Ok(0)
        }
    }

    fn write_word(&mut self, addr: u32, val: u32) -> Result<(), Exception> {
        if let Some(i) = Self::aux_idx(addr) {
            self.aux_store[i] = val;
            return Ok(());
        }
        if addr == self.bus.cmd_addr {
            self.bus.write_cmd(val);
            self.issue_command(val);
        } else if addr == self.bus.data_addr {
            self.bus.write_data(val);
            self.pending_sub_addr = (val & 0xFFFF) as u16;
        } else if addr == self.bus.stat_addr {
            self.bus.write_stat(val);
            if val & 0x0040_0000 != 0
                && matches!(self.protocol, ProtocolState::SubAddrLatched)
            {
                let bc = ((val >> 3) & 0x7FF) as u16;
                if bc > 0 {
                    self.byte_count = bc;
                    self.eeprom_ptr = self.sub_addr;
                    self.bytes_read = 0;
                    self.protocol = ProtocolState::Armed;
                    self.bus.go_busy();
                }
            }
        }
        Ok(())
    }

    fn tick(&mut self, _cpu_instructions: u64) {
        self.bus.tick();
    }

    fn reset_cold(&mut self) {
        self.bus.reset();
        self.aux_store = [0; 3];
        self.protocol = ProtocolState::Idle;
        self.pending_sub_addr = 0;
        self.sub_addr = 0;
        self.eeprom_ptr = 0;
        self.byte_count = 0;
        self.bytes_read = 0;
        self.last_device = 0;
        self.pending_read_word = 0;
        self.force_nack = false;
        self.bus.apply_init(super::mmio_init::SYSREG_INIT_VALUES);
        self.sfp.reset_to_snapshot();
    }

    fn snapshot(&self) -> PeripheralSnapshot {
        PeripheralSnapshot::Bsc(BscSnapshot {
            busy: self.bus.is_busy(),
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
                self.bus.set_idle();
                Ok(())
            }
            PeripheralEvent::Sfp(_) => {
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
        let cmd = (1u32 << 27) | (0x100u32 << 18) | 0x51;
        bsc.write_word(0x01000140, cmd).unwrap();
        assert_eq!(bsc.last_device, 1);

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
