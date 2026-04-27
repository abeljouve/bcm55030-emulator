//! BCM55030 EPON free-running counter — Session 6.
//!
//! Owns the single-register `EPON_FREE_COUNTER` at `0x01000050`,
//! previously served by `sysreg_shim::timer_counter`. The counter
//! increments once per bank tick scaled by a user-configurable
//! prescaler (default `1` — every bank tick advances the counter).
//!
//! Resolves audit 3.1 — the prescaler is tunable via
//! [`TimerEvent::SetPrescaler`] and the counter value via
//! [`TimerEvent::SetCounter`].
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
    tick_accumulator: u32,
    prescaler: u32,
    pub trace: bool,
}

impl EponTimer {
    pub fn new() -> Self {
        Self {
            counter: 0,
            tick_accumulator: 0,
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
        Ok(self.counter)
    }

    fn write_word(&mut self, _addr: u32, val: u32) -> Result<(), Exception> {
        // Counter is writable by software — used for calibration
        // tests. Not a HW-verified behaviour, but it matches the
        // old `sysreg_store` residual fallback.
        self.counter = val;
        Ok(())
    }

    fn tick(&mut self, _cpu_instructions: u64) {
        self.tick_accumulator = self.tick_accumulator.wrapping_add(1);
        if self.tick_accumulator >= self.prescaler {
            self.tick_accumulator = 0;
            self.counter = self.counter.wrapping_add(1);
        }
    }

    fn reset_cold(&mut self) {
        self.counter = 0;
        self.tick_accumulator = 0;
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
                    self.tick_accumulator = 0;
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
    fn counter_increments_on_each_tick() {
        let mut t = EponTimer::new();
        t.tick(64);
        t.tick(64);
        t.tick(64);
        assert_eq!(t.read_word(REG_EPON_FREE_COUNTER).unwrap(), 3);
    }

    #[test]
    fn prescaler_divides_tick_rate() {
        let mut t = EponTimer::new();
        t.inject_event(&PeripheralEvent::Timer(TimerEvent::SetPrescaler(4)))
            .unwrap();
        for _ in 0..12 {
            t.tick(64);
        }
        assert_eq!(t.read_word(REG_EPON_FREE_COUNTER).unwrap(), 3);
    }

    #[test]
    fn software_write_overrides_counter() {
        let mut t = EponTimer::new();
        t.write_word(REG_EPON_FREE_COUNTER, 0xDEAD_BEEF).unwrap();
        assert_eq!(t.read_word(REG_EPON_FREE_COUNTER).unwrap(), 0xDEAD_BEEF);
    }

    #[test]
    fn set_counter_event() {
        let mut t = EponTimer::new();
        t.inject_event(&PeripheralEvent::Timer(TimerEvent::SetCounter(42)))
            .unwrap();
        assert_eq!(t.read_word(REG_EPON_FREE_COUNTER).unwrap(), 42);
    }
}
