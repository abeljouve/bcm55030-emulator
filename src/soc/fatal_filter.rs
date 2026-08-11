//! The four-word block at `0x01003600`–`0x01003610`.
//!
//! ⛔ The module keeps its `fatal_filter` name for its consumers, and
//! the name is not established. It was chosen when this block was read
//! as a packet filter with an error aggregator; the register DB has
//! since relabelled the block "SerDes IRQ / Lane Mask Controller" and
//! notes explicitly that it is **not** a packet filter. Two readings of
//! the block survive:
//!
//!   * a filter enable mask plus a latched fatal-error word, the one
//!     this module implements;
//!   * an IRQ / lane enable mask at `+0x00` whose low four bits are set
//!     together, paired with a status word at `+0x04` carrying an error
//!     field in bits `[19:4]` — four accessors share the pair, one of
//!     which only ORs bits in, which is not how a table pointer is used.
//!
//! Neither reading is measured. What the module does — the word at
//! `+0x04` reads back zero until something injects into it, and clears
//! written bits — is a **workaround** that keeps a polling loop moving,
//! not a measurement of the hardware.
//!
//! ⛔ The "fatal error status" this module's name evokes is a different
//! address, `0x01002804`, and **both** names ever given to that word
//! were fabricated and have been retracted; the register DB now
//! reads it as a classifier status / write-1-to-clear word. That
//! address is served elsewhere and the reformulation lives with the
//! peripheral that serves it — this module only points at it.
//!
//! ⛔ The block once cited here as a second fatal-monitor block no
//! longer exists: it was merged away, and the register entries it
//! contributed were dropped as misclassified.

use crate::cpu::exception::Exception;
use crate::soc::peripheral::{
    AddressRange, FatalFilterEvent, FatalFilterSnapshot, Peripheral, PeripheralError,
    PeripheralEvent, PeripheralSnapshot,
};

pub const FATAL_FILTER_BASE: u32 = 0x0100_3600;
pub const FATAL_FILTER_END: u32 = 0x0100_3614;

const REG_FILTER_STATUS: u32 = 0x0100_3600;
/// The second word of the block, named after its address because its
/// role is not established. ⛔ Two readings compete — a latched
/// fatal-error aggregator, and a status word with an error field in
/// bits `[19:4]` whose sibling at `+0x00` is an IRQ / lane mask — and
/// the first is traceable to a name the register DB has since
/// retracted. Reading it as zero until something injects is a
/// workaround for a polling loop, not a measurement.
///
/// Listed in `FIDELITY.md`, "Two register behaviours chosen to keep a
/// firmware moving" — keep the two in step.
const REG_3604: u32 = 0x0100_3604;
const REG_FILTER_ENABLE: u32 = 0x0100_3608;
const REG_FILTER_IRQ: u32 = 0x0100_3610;

const FATAL_FILTER_RANGES: &[AddressRange] =
    &[AddressRange::new(FATAL_FILTER_BASE, FATAL_FILTER_END)];

#[derive(Clone)]
pub struct FatalFilter {
    fatal_status: u32,
    filter_status: u32,
    filter_enable: u32,
    filter_irq: u32,
    link_up_bitmap: u32,
    pub trace: bool,
}

impl FatalFilter {
    pub fn new() -> Self {
        Self {
            fatal_status: 0,
            filter_status: 0,
            filter_enable: 0,
            filter_irq: 0,
            link_up_bitmap: 0,
            trace: false,
        }
    }

    #[inline]
    pub fn claims(&self, addr: u32) -> bool {
        (FATAL_FILTER_BASE..FATAL_FILTER_END).contains(&addr)
    }

    fn apply_silicon_power_on(&mut self) {
        for &(off, val) in super::mmio_init::SYSREG_INIT_VALUES {
            let abs = 0x0100_0000 + off;
            if !self.claims(abs) {
                continue;
            }
            match abs {
                REG_FILTER_STATUS => self.filter_status = val,
                REG_FILTER_ENABLE => self.filter_enable = val,
                REG_FILTER_IRQ => self.filter_irq = val,
                // Left at 0 rather than seeded: under either reading of
                // this word the power-on snapshot is a status value, and
                // seeding a status makes the firmware act on a condition
                // nothing produced.
                REG_3604 => {}
                _ => {}
            }
        }
    }
}

impl Peripheral for FatalFilter {
    fn name(&self) -> &'static str {
        "fatal_filter"
    }

    fn address_ranges(&self) -> &'static [AddressRange] {
        FATAL_FILTER_RANGES
    }

    fn read_word(&mut self, addr: u32) -> Result<u32, Exception> {
        match addr {
            REG_FILTER_STATUS => Ok(self.filter_status),
            REG_3604 => Ok(self.fatal_status),
            REG_FILTER_ENABLE => Ok(self.filter_enable),
            REG_FILTER_IRQ => Ok(self.filter_irq),
            _ => Ok(0),
        }
    }

    fn write_word(&mut self, addr: u32, val: u32) -> Result<(), Exception> {
        match addr {
            REG_FILTER_STATUS => self.filter_status = val,
            REG_3604 => {
                // Write-1-to-clear. ⛔ Under the competing reading this
                // word is written with a whole mask bitmap, which this
                // arm would silently turn into a clear. The two readings
                // are separated by whether a write of `0x000FFFF0` here
                // is followed by a read of the same value.
                self.fatal_status &= !val;
            }
            REG_FILTER_ENABLE => self.filter_enable = val,
            REG_FILTER_IRQ => self.filter_irq = val,
            _ => {}
        }
        Ok(())
    }

    fn tick(&mut self, _cpu_instructions: u64) {}

    fn reset_cold(&mut self) {
        self.fatal_status = 0;
        self.filter_status = 0;
        self.filter_enable = 0;
        self.filter_irq = 0;
        self.link_up_bitmap = 0;
        self.apply_silicon_power_on();
    }

    fn reset_warm(&mut self) {
        // Silicon power-on snapshot already applied in `reset_cold`.
        self.reset_cold();
    }

    fn snapshot(&self) -> PeripheralSnapshot {
        PeripheralSnapshot::FatalFilter(FatalFilterSnapshot {
            fatal_status: self.fatal_status,
            link_up_bitmap: self.link_up_bitmap,
        })
    }

    fn inject_event(&mut self, event: &PeripheralEvent) -> Result<(), PeripheralError> {
        match event {
            PeripheralEvent::FatalFilter(ev) => match ev {
                FatalFilterEvent::InjectFatal(bits) => {
                    self.fatal_status |= *bits;
                    Ok(())
                }
                FatalFilterEvent::ClearFatal => {
                    self.fatal_status = 0;
                    Ok(())
                }
                FatalFilterEvent::SetLinkUp(phy, up) => {
                    let bit = 1u32 << (*phy as u32 & 0x1F);
                    if *up {
                        self.link_up_bitmap |= bit;
                    } else {
                        self.link_up_bitmap &= !bit;
                    }
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
    fn fatal_reads_zero_until_injected() {
        let mut f = FatalFilter::new();
        assert_eq!(f.read_word(REG_3604).unwrap(), 0);
        f.inject_event(&PeripheralEvent::FatalFilter(FatalFilterEvent::InjectFatal(0x42)))
            .unwrap();
        assert_eq!(f.read_word(REG_3604).unwrap(), 0x42);
    }

    #[test]
    fn fatal_w1c_on_write() {
        let mut f = FatalFilter::new();
        f.inject_event(&PeripheralEvent::FatalFilter(FatalFilterEvent::InjectFatal(0xFF)))
            .unwrap();
        f.write_word(REG_3604, 0x0F).unwrap();
        assert_eq!(f.read_word(REG_3604).unwrap(), 0xF0);
    }

    #[test]
    fn claims_covers_0x3600_to_0x3613() {
        let f = FatalFilter::new();
        assert!(f.claims(0x0100_3600));
        assert!(f.claims(0x0100_3604));
        assert!(f.claims(0x0100_3608));
        assert!(f.claims(0x0100_3610));
        assert!(!f.claims(0x0100_3614));
        assert!(!f.claims(0x0100_35FC));
    }

    #[test]
    fn link_up_toggle() {
        let mut f = FatalFilter::new();
        f.inject_event(&PeripheralEvent::FatalFilter(FatalFilterEvent::SetLinkUp(0, true)))
            .unwrap();
        f.inject_event(&PeripheralEvent::FatalFilter(FatalFilterEvent::SetLinkUp(2, true)))
            .unwrap();
        assert_eq!(f.link_up_bitmap, 0b101);
        f.inject_event(&PeripheralEvent::FatalFilter(FatalFilterEvent::SetLinkUp(0, false)))
            .unwrap();
        assert_eq!(f.link_up_bitmap, 0b100);
    }
}
