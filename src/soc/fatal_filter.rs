//! BCM55030 filter / fatal error monitor — Session 6.
//!
//! Owns the filter error aggregator at `0x01003604`. The register
//! is read-as-zero by hardware when no fatal condition is latched
//! — the firmware polls it from its idle loop and triggers a
//! rollback boot on any non-zero value.
//!
//! This peripheral also owns the carveout at `0x01003600`–
//! `0x01003610` that was reserved by the DMA peripheral for
//! Session 7, consolidating the filter / fatal arms into a
//! single module. Additional fatal-monitor registers (block 5601
//! at runtime `0x010027B8`, block 5602 at `0x010027EC`) are
//! inside the MACsec range and continue to be served from there
//! — the firmware reads them through MACsec's backing store.
//!
//! Audit items resolved:
//!
//!   * **5.5 (finish)** — the `0x3604` hardcoded arm is gone from
//!     `sysreg_shim`; fatal reads are driven by real state.
//!   * **5.6** — filter / fatal window has a dedicated owner.
//!   * **5.12 (finish)** — no more residual sysreg special arms.

use crate::cpu::exception::Exception;
use crate::soc::peripheral::{
    AddressRange, FatalFilterEvent, FatalFilterSnapshot, Peripheral, PeripheralError,
    PeripheralEvent, PeripheralSnapshot,
};

pub const FATAL_FILTER_BASE: u32 = 0x0100_3600;
pub const FATAL_FILTER_END: u32 = 0x0100_3614;

const REG_FILTER_STATUS: u32 = 0x0100_3600;
const REG_FILTER_FATAL: u32 = 0x0100_3604;
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
                // Fatal aggregator stays at 0 — it's a latched-error
                // register, not a config register.
                REG_FILTER_FATAL => {}
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
            REG_FILTER_FATAL => Ok(self.fatal_status),
            REG_FILTER_ENABLE => Ok(self.filter_enable),
            REG_FILTER_IRQ => Ok(self.filter_irq),
            _ => Ok(0),
        }
    }

    fn write_word(&mut self, addr: u32, val: u32) -> Result<(), Exception> {
        match addr {
            REG_FILTER_STATUS => self.filter_status = val,
            REG_FILTER_FATAL => {
                // Write-1-to-clear on fatal bits.
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
        assert_eq!(f.read_word(REG_FILTER_FATAL).unwrap(), 0);
        f.inject_event(&PeripheralEvent::FatalFilter(FatalFilterEvent::InjectFatal(0x42)))
            .unwrap();
        assert_eq!(f.read_word(REG_FILTER_FATAL).unwrap(), 0x42);
    }

    #[test]
    fn fatal_w1c_on_write() {
        let mut f = FatalFilter::new();
        f.inject_event(&PeripheralEvent::FatalFilter(FatalFilterEvent::InjectFatal(0xFF)))
            .unwrap();
        f.write_word(REG_FILTER_FATAL, 0x0F).unwrap();
        assert_eq!(f.read_word(REG_FILTER_FATAL).unwrap(), 0xF0);
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
