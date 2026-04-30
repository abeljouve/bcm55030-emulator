//! BCM55030 EPON free-running counter.
//!
//! Owns the single-register `EPON_FREE_COUNTER` at `0x01000050`.
//! The counter advances by 1 on each word-level read so that firmware
//! exact-match busy-waits (`serdes_busy_wait_epon_timer_ticks`) see
//! every consecutive value and never skip a target. Sub-word reads
//! (byte/half) peek without advancing — only the firmware's `ld.di`
//! word reads trigger progression.
//!
//! On real silicon the counter runs at 156.25 MHz independently of
//! reads, but exact-match correctness is required for firmware init
//! to complete.
//!
//! The ARC Timer 0 / Timer 1 live CPU-side in `src/cpu/mod.rs` and
//! are unrelated to this peripheral — they are aux registers, not
//! MMIO.

use crate::cpu::exception::Exception;
use crate::soc::peripheral::{
    AddressRange, Peripheral, PeripheralError, PeripheralEvent, PeripheralSnapshot, TimerEvent,
    TimerSnapshot,
};

pub const REG_EPON_FREE_COUNTER: u32 = 0x0100_0050;

const TIMER_RANGES: &[AddressRange] = &[AddressRange::new(
    REG_EPON_FREE_COUNTER,
    REG_EPON_FREE_COUNTER + 4,
)];

#[derive(Clone)]
pub struct EponTimer {
    counter: u32,
    prescaler: u32,
    pub trace: bool,
}

impl EponTimer {
    pub fn new() -> Self {
        Self {
            counter: 0,
            prescaler: 1,
            trace: false,
        }
    }

    #[inline]
    pub fn claims(&self, addr: u32) -> bool {
        (REG_EPON_FREE_COUNTER..REG_EPON_FREE_COUNTER + 4).contains(&addr)
    }
}

impl Peripheral for EponTimer {
    fn name(&self) -> &'static str {
        "timer"
    }

    fn address_ranges(&self) -> &'static [AddressRange] {
        TIMER_RANGES
    }

    fn read_word(&mut self, _addr: u32) -> Result<u32, Exception> {
        let val = self.counter;
        self.counter = self.counter.wrapping_add(self.prescaler);
        Ok(val)
    }

    fn read_byte(&mut self, addr: u32) -> Result<u8, Exception> {
        let byte_idx = addr & 3;
        Ok((self.counter >> (24 - byte_idx * 8)) as u8)
    }

    fn read_half(&mut self, addr: u32) -> Result<u16, Exception> {
        let half_idx = (addr >> 1) & 1;
        Ok((self.counter >> (16 - half_idx * 16)) as u16)
    }

    fn write_word(&mut self, _addr: u32, val: u32) -> Result<(), Exception> {
        self.counter = val;
        Ok(())
    }

    fn tick(&mut self, _cpu_instructions: u64) {}

    fn reset_cold(&mut self) {
        self.counter = 0;
        self.prescaler = 1;
    }

    fn snapshot(&self) -> PeripheralSnapshot {
        PeripheralSnapshot::Timer(TimerSnapshot {
            counter: self.counter,
            prescaler: self.prescaler,
        })
    }

    fn inject_event(&mut self, event: &PeripheralEvent) -> Result<(), PeripheralError> {
        match event {
            PeripheralEvent::Timer(ev) => match ev {
                TimerEvent::SetPrescaler(p) => {
                    if *p == 0 {
                        return Err(PeripheralError::InvalidParameter("prescaler 0"));
                    }
                    self.prescaler = *p;
                    Ok(())
                }
                TimerEvent::SetCounter(v) => {
                    self.counter = *v;
                    Ok(())
                }
            },
            _ => Err(PeripheralError::UnsupportedEvent),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn counter_increments_on_word_read() {
        let mut t = EponTimer::new();
        assert_eq!(t.read_word(REG_EPON_FREE_COUNTER).unwrap(), 0);
        assert_eq!(t.read_word(REG_EPON_FREE_COUNTER).unwrap(), 1);
        assert_eq!(t.read_word(REG_EPON_FREE_COUNTER).unwrap(), 2);
    }

    #[test]
    fn byte_read_does_not_advance() {
        let mut t = EponTimer::new();
        t.write_word(REG_EPON_FREE_COUNTER, 0x11223344).unwrap();
        assert_eq!(t.read_byte(REG_EPON_FREE_COUNTER).unwrap(), 0x11);
        assert_eq!(t.read_byte(REG_EPON_FREE_COUNTER + 3).unwrap(), 0x44);
        assert_eq!(t.read_word(REG_EPON_FREE_COUNTER).unwrap(), 0x11223344);
    }

    #[test]
    fn prescaler_controls_increment() {
        let mut t = EponTimer::new();
        t.inject_event(&PeripheralEvent::Timer(TimerEvent::SetPrescaler(10)))
            .unwrap();
        assert_eq!(t.read_word(REG_EPON_FREE_COUNTER).unwrap(), 0);
        assert_eq!(t.read_word(REG_EPON_FREE_COUNTER).unwrap(), 10);
        assert_eq!(t.read_word(REG_EPON_FREE_COUNTER).unwrap(), 20);
    }

    #[test]
    fn software_write_overrides_counter() {
        let mut t = EponTimer::new();
        t.write_word(REG_EPON_FREE_COUNTER, 0xDEAD_BEEF).unwrap();
        assert_eq!(t.read_word(REG_EPON_FREE_COUNTER).unwrap(), 0xDEAD_BEEF);
    }

    #[test]
    fn busy_wait_target_always_hit() {
        let mut t = EponTimer::new();
        let start = t.read_word(REG_EPON_FREE_COUNTER).unwrap() & 0x7FFF;
        let target = (start + 10) & 0x7FFF;
        let mut found = false;
        for _ in 0..200 {
            if t.read_word(REG_EPON_FREE_COUNTER).unwrap() & 0x7FFF == target {
                found = true;
                break;
            }
        }
        assert!(found, "exact-match target must be reachable");
    }
}
