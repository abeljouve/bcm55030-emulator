//! Alarm / event dispatch — a UI test harness, not a hardware model.
//!
//! This peripheral **does not model hardware**. It claims no MMIO
//! range, has no decoder of its own, does nothing on `tick()`, and
//! never writes into memory. All it holds is a set of opcodes forced
//! from the UI through [`AlarmEvent::ForcePending`] /
//! [`AlarmEvent::ClearPending`], which the snapshot exposes.
//!
//! ⛔ This doc used to attribute specific opcode numbers to specific
//! upstream sources — a teardown group, a counter-overflow group, a
//! pin-change group. Those attributions were INFERRED, and a file
//! whose first line says it models no hardware is the last place a
//! reader should meet a numbered source map. They are removed rather
//! than annotated: keeping them would mean carrying a map of a
//! pipeline this module deliberately does not have.
//!
//! What remains true, and is the reason the harness exists: on a
//! quiescent emulator the alarm CLI commands show fewer persistent
//! opcodes than hardware would, because nothing in the model raises
//! any. Every opcode you see here was put there from the UI.

use crate::cpu::exception::Exception;
use crate::soc::peripheral::{
    AddressRange, AlarmEvent, AlarmSnapshot, Peripheral, PeripheralError, PeripheralEvent,
    PeripheralSnapshot,
};

/// Maximum number of distinct opcodes the test harness tracks.
///
/// A bound on a UI list, chosen so a snapshot stays small. ⛔ It is not
/// derived from anything on the chip; the handler-table size this
/// comment used to cite is a property of a piece of software, and this
/// module models no hardware to compare it against.
pub const FORCED_OPCODE_CAPACITY: usize = 64;

#[derive(Clone)]
pub struct AlarmEvents {
    forced: Vec<u16>,
    pub trace: bool,
}

impl AlarmEvents {
    pub fn new() -> Self {
        Self {
            forced: Vec::with_capacity(FORCED_OPCODE_CAPACITY),
            trace: false,
        }
    }

    #[inline]
    pub fn claims(&self, _addr: u32) -> bool {
        false
    }

    pub fn force_pending(&mut self, opcode: u16) {
        if !self.forced.contains(&opcode) && self.forced.len() < FORCED_OPCODE_CAPACITY {
            self.forced.push(opcode);
        }
    }

    pub fn clear_pending(&mut self, opcode: u16) {
        self.forced.retain(|op| *op != opcode);
    }

    pub fn clear_all(&mut self) {
        self.forced.clear();
    }

    pub fn forced_opcodes(&self) -> &[u16] {
        &self.forced
    }
}

impl Peripheral for AlarmEvents {
    fn name(&self) -> &'static str {
        "alarm_events"
    }

    fn address_ranges(&self) -> &'static [AddressRange] {
        &[]
    }

    fn read_word(&mut self, _addr: u32) -> Result<u32, Exception> {
        Ok(0)
    }

    fn write_word(&mut self, _addr: u32, _val: u32) -> Result<(), Exception> {
        Ok(())
    }

    fn tick(&mut self, _cpu_instructions: u64) {}

    fn reset_cold(&mut self) {
        self.forced.clear();
    }

    fn snapshot(&self) -> PeripheralSnapshot {
        PeripheralSnapshot::Alarm(AlarmSnapshot {
            forced_opcodes: self.forced.clone(),
            live_opcodes: Vec::new(),
        })
    }

    fn inject_event(&mut self, event: &PeripheralEvent) -> Result<(), PeripheralError> {
        match event {
            PeripheralEvent::Alarm(ev) => match ev {
                AlarmEvent::ForcePending(op) => {
                    self.force_pending(*op);
                    Ok(())
                }
                AlarmEvent::ClearPending(op) => {
                    self.clear_pending(*op);
                    Ok(())
                }
                AlarmEvent::ClearAll => {
                    self.clear_all();
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
    fn force_clear_roundtrip() {
        let mut a = AlarmEvents::new();
        a.force_pending(28);
        a.force_pending(131);
        a.force_pending(28); // duplicate — no-op
        assert_eq!(a.forced_opcodes(), &[28u16, 131u16]);
        a.clear_pending(28);
        assert_eq!(a.forced_opcodes(), &[131u16]);
        a.clear_all();
        assert!(a.forced_opcodes().is_empty());
    }

    #[test]
    fn capacity_is_bounded() {
        let mut a = AlarmEvents::new();
        for i in 0..200u16 {
            a.force_pending(i);
        }
        assert_eq!(a.forced_opcodes().len(), FORCED_OPCODE_CAPACITY);
    }

    #[test]
    fn claims_nothing() {
        let a = AlarmEvents::new();
        assert!(!a.claims(0x0100_0000));
        assert!(!a.claims(0x0000_0000));
    }

    #[test]
    fn event_injection_dispatches() {
        let mut a = AlarmEvents::new();
        a.inject_event(&PeripheralEvent::Alarm(AlarmEvent::ForcePending(193)))
            .unwrap();
        a.inject_event(&PeripheralEvent::Alarm(AlarmEvent::ForcePending(199)))
            .unwrap();
        assert_eq!(a.forced_opcodes().len(), 2);
        a.inject_event(&PeripheralEvent::Alarm(AlarmEvent::ClearAll))
            .unwrap();
        assert!(a.forced_opcodes().is_empty());
    }
}
