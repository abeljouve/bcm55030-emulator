//! BCM55030 alarm / event dispatch — Session 5 UI test harness.
//!
//! This peripheral **does not model hardware**. The real BCM55030
//! alarm pipeline is driven by upstream event sources:
//!
//!   * **LLID teardown events (opcodes 193, 199, 201)** — raised
//!     by the EPON MAC when a deregister or link-down happens.
//!   * **Stats counter overflow events (opcodes 23, 64)** — raised
//!     when a per-LLID counter in the stats file crosses its
//!     high-water threshold (block 75).
//!   * **GPIO / PMD pin-change events (opcodes 28, 131)** — raised
//!     by physical I/O events not yet modelled.
//!
//! Session 5 removes the old `alarm.rs` which synthesised
//! quiescent-ONU alarm state by writing directly into the
//! firmware's in-SRAM pending-bit mask table on every tick. That
//! behaviour violated the contributor guide (no firmware-mode emulation).
//! Its replacement is this peripheral, which:
//!
//!   1.  Claims no MMIO range — it has no HW decoder of its own.
//!   2.  Stores a **UI-forced** pending-opcode set that the user
//!       can poke via [`AlarmEvent::ForcePending`] /
//!       [`AlarmEvent::ClearPending`]. The UI snapshot exposes it.
//!   3.  Does nothing on `tick()` — the firmware now handles the
//!       alarm flow natively.
//!
//! **Known regression:** without the synthetic seeder, the
//! `alm/info` and `alm/gpio` CLI commands on a quiescent emulator
//! will show fewer persistent opcodes than a live ONU capture.
//! Opcodes 28 and 131 in particular have no current upstream
//! source in the model; they remain TODO and must be driven
//! explicitly from the UI through [`AlarmEvent::ForcePending`].
//! This is audit item 7.1 — remaining partial until every event
//! source is either modelled or explicitly injected.

use crate::cpu::exception::Exception;
use crate::soc::peripheral::{
    AddressRange, AlarmEvent, AlarmSnapshot, Peripheral, PeripheralError, PeripheralEvent,
    PeripheralSnapshot,
};

/// Maximum number of distinct opcodes the test harness tracks. The
/// firmware dispatch table has 147 handler slots; 64 covers the
/// working set without wasting snapshot bandwidth.
pub const FORCED_OPCODE_CAPACITY: usize = 64;

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
