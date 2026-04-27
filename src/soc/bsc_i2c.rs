//! BCM55030 Broadcom Serial Controller (BSC) I²C master.
//!
//! The BSC is the on-chip I²C master that talks to the SFP EEPROM
//! (devices `0xA0` and `0xA2`) at MMIO offsets:
//!
//!   * `0x01000140` — command / start register. Bits[27..32] encode the
//!     command, bits[18..27] encode `(word_index + 0x100)`, bit 0
//!     selects A0h (0) or A2h (1).
//!   * `0x0100014C` — read data register. The firmware polls this for
//!     the four bytes of the most recent transaction.
//!   * `0x01000150` — busy/status register. Bit 31 = busy. Set on
//!     command, cleared on completion (after `BSC_BUSY_TICKS`).
//!
//! Session 1 implements a state machine sufficient for the firmware's
//! polling loop: any number of registers in the BSC range are stored
//! verbatim, but the three registers above drive a real micro state
//! machine (`Idle → Busy → Done`). The owned [`SfpEeprom`] supplies
//! the read data, including the live DDM overlay for A2h bytes 96–109.
//!
//! Audit 5.10 is resolved here.

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

pub struct BscI2c {
    pub sfp: SfpEeprom,
    state: BusState,
    busy_counter: u8,
    /// Last device select bit from REG_CMD (0 = A0h, 1 = A2h).
    last_device: u8,
    /// Word index (in 4-byte words) from REG_CMD bits[18..27].
    last_word_idx: u16,
    /// 16-bit base address from REG_BASE_ADDR low half.
    base_addr: u16,
    /// Pending read result (4 bytes packed LE).
    pending_read_word: u32,
    /// Force NACK on next transaction (UI fault injection).
    force_nack: bool,
    /// Verbatim register store for unmodelled BSC offsets in the range.
    raw_store: [u32; 6],
}

impl BscI2c {
    pub fn new() -> Self {
        Self {
            sfp: SfpEeprom::new_default(),
            state: BusState::Idle,
            busy_counter: 0,
            last_device: 0,
            last_word_idx: 0,
            base_addr: 0,
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
        let param_5 = (val >> 18) & 0x1FF;
        let cmd_hi = (val >> 27) & 0x1F;
        if (cmd_hi & 1) != 0 && param_5 >= 0x100 {
            self.last_word_idx = (param_5 - 0x100) as u16;
            let read_off = self.base_addr.wrapping_add(self.last_word_idx * 4);
            self.pending_read_word = self.sfp.read_word(self.last_device, read_off);
            if self.force_nack {
                self.pending_read_word = 0xFFFF_FFFF;
                self.force_nack = false;
            }
            self.state = BusState::Busy;
            self.busy_counter = BSC_BUSY_TICKS;
        }
        // Hardware behaviour: command bits (27-31) auto-clear once the
        // command is latched. The firmware polls REG_CMD waiting for
        // these bits to drop. Without this, the polling loop at
        // `epon_link_config_and_enable_all @ ram:20032CB0` spins
        // forever (regression caught during the Session 1 refactor).
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

    fn read_word(&mut self, addr: u32) -> Result<u32, Exception> {
        let off = addr - 0x01000000;
        match off {
            REG_CMD => Ok(self
                .store_idx(REG_CMD)
                .map(|i| self.raw_store[i])
                .unwrap_or(0)),
            REG_BASE_ADDR => {
                if matches!(self.state, BusState::Busy | BusState::Done) {
                    let val = self.pending_read_word;
                    if matches!(self.state, BusState::Done) {
                        self.state = BusState::Idle;
                    }
                    Ok(val)
                } else {
                    Ok(self.base_addr as u32)
                }
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
            REG_BASE_ADDR => self.base_addr = (val & 0xFFFF) as u16,
            REG_STATUS => {
                // Writing 1 to bit 31 normally clears it on real HW.
                // Mirror that behaviour for parity.
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
        self.last_device = 0;
        self.last_word_idx = 0;
        self.base_addr = 0;
        self.pending_read_word = 0;
        self.force_nack = false;
        self.raw_store = [0u32; 6];
        self.sfp.reset_to_snapshot();
    }

    fn snapshot(&self) -> PeripheralSnapshot {
        PeripheralSnapshot::Bsc(BscSnapshot {
            busy: matches!(self.state, BusState::Busy),
            last_device_addr: if self.last_device == 0 { 0xA0 } else { 0xA2 },
            last_word_addr: self.base_addr.wrapping_add(self.last_word_idx * 4),
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
    fn busy_clears_after_ticks() {
        let mut bsc = BscI2c::new();
        bsc.write_word(0x0100014C, 0x0000_0050).unwrap();
        // Issue a read of word_idx=0 on A2h (device=1)
        let cmd = (1u32 << 27) | (0x100u32 << 18) | 1;
        bsc.write_word(0x01000140, cmd).unwrap();
        // Status bit 31 set immediately
        let s = bsc.read_word(0x01000150).unwrap();
        assert_ne!(s & 0x8000_0000, 0);
        // After enough ticks, it clears
        for _ in 0..BSC_BUSY_TICKS {
            bsc.tick(64);
        }
        let s = bsc.read_word(0x01000150).unwrap();
        assert_eq!(s & 0x8000_0000, 0);
    }
}
