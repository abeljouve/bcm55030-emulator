//! BCM55030 DMA channel controller — Session 4.
//!
//! Owns the DMA channel file at `0x01002000..0x01004000`. The file
//! is organised as a set of stride-`0x200` channel slots (one per
//! DMA channel) sharing a common sub-offset layout. The MACsec SA
//! programming windows at `0x01002400..0x01002D40` and
//! `0x01003500..0x01003540` sit inside this range but are claimed
//! first by [`Macsec`](super::macsec::Macsec) — DMA excludes those
//! addresses via its own [`claims`](DmaChannelController::claims)
//! predicate so the two peripherals do not fight over the same
//! routes.
//!
//! Each channel slot exposes, at minimum:
//!
//! | Sub-offset | Name              | Semantics                                  |
//! |:-----------|:------------------|:-------------------------------------------|
//! | `0x00`     | `SUBCHAN_INDEX`   | drain trigger + status (bit 8 = ready)     |
//! | `0x34`     | `IRQ_STATUS`      | per-sub-channel IRQ status bitmap          |
//! | `0x3C`     | `QUEUE_DRAIN`     | alias of drain status — bit 8 polled       |
//! | `0x4C`     | `IRQ_ENABLE`      | per-sub-channel IRQ enable mask            |
//! | `0x50`     | `IRQ_PENDING`     | per-sub-channel IRQ pending mask           |
//!
//! v1 models the queue-drain `bit 8 = ready` flag that every
//! firmware polling loop depends on: reads return bit 8 set, a
//! write with bit 8 set clears it, the next tick re-arms it. This
//! matches the residual arm previously in `sysreg_shim` (audit
//! 5.7 — already resolved inside `epon_mac.rs` for the LLID
//! sub-range `0x1400..0x2000`; this file resolves the rest).
//!
//! All other offsets route through a flat backing store with the
//! same bits `27..31` auto-clear semantic MACsec uses.
//!
//! Audit items resolved:
//!
//!   * **5.8 (finish)** — the generic sysreg auto-clear arm is
//!     fully removed. Each peripheral with a real command-bit
//!     semantic now owns the clear behaviour locally.
//!   * **5.7 (finish)** — DMA queue drain no longer served by a
//!     shim fallback.
//!   * **5.12 (part)** — residual `SYSREG_INIT_VALUES` shrink.

use crate::cpu::exception::Exception;
use crate::soc::peripheral::{
    AddressRange, DmaEvent, DmaSnapshot, Peripheral, PeripheralError, PeripheralEvent,
    PeripheralSnapshot,
};

pub const DMA_BASE: u32 = 0x0100_2000;
pub const DMA_END: u32 = 0x0100_4000;

/// Sub-ranges inside `DMA_BASE..DMA_END` that belong to other
/// peripherals (MACsec, filter/fatal) and must NOT be claimed by
/// this one. The bank router checks MACsec first, but the
/// exclusion keeps the predicate self-consistent if routing order
/// ever changes.
///
/// `0x01003604` is the filter / fatal error aggregator carved out
/// from the DMA window until Session 7 lands `fatal_filter.rs`;
/// it stays in `sysreg_shim` with a hardcoded `=> 0` arm.
const MACSEC_CARVEOUTS: &[(u32, u32)] = &[
    (0x0100_2400, 0x0100_2D40),
    (0x0100_3500, 0x0100_3540),
    (0x0100_3600, 0x0100_3614),
];

const DMA_STRIDE: u32 = 0x0200;
/// Sub-offset within a channel slot that carries the drain trigger
/// and its `ready` status (bit 8). This is the offset the firmware
/// polls across every channel (`base + N * 0x200 + 0x3C`).
const SUB_QUEUE_DRAIN: u32 = 0x003C;

const CMD_BIT_MASK: u32 = 0xF800_0000;

const DMA_RANGES: &[AddressRange] = &[AddressRange::new(DMA_BASE, DMA_END)];

pub struct DmaChannelController {
    store: Vec<u32>,
    pending_clear: Vec<u32>,
    /// Per-channel queue-drain "ready" flag. The base offset is
    /// `0x01001400` per `hwregs` block 74, but the LLID channels
    /// live in `epon_mac.rs`. The remaining channels start at
    /// `0x01002000` — `0x203C`, `0x223C`, ..., `0x3E3C`.
    drain_flag: Vec<bool>,
    pub trace: bool,
}

impl DmaChannelController {
    pub fn new() -> Self {
        let words = ((DMA_END - DMA_BASE) / 4) as usize;
        let channel_count = ((DMA_END - DMA_BASE) / DMA_STRIDE) as usize;
        Self {
            store: vec![0u32; words],
            pending_clear: vec![0u32; words],
            drain_flag: vec![true; channel_count],
            trace: false,
        }
    }

    #[inline]
    pub fn claims(&self, addr: u32) -> bool {
        if !(DMA_BASE..DMA_END).contains(&addr) {
            return false;
        }
        !MACSEC_CARVEOUTS
            .iter()
            .any(|(start, end)| addr >= *start && addr < *end)
    }

    fn idx(addr: u32) -> usize {
        ((addr - DMA_BASE) / 4) as usize
    }

    fn slot_of(addr: u32) -> (usize, u32) {
        let rel = addr - DMA_BASE;
        ((rel / DMA_STRIDE) as usize, rel % DMA_STRIDE)
    }

    fn apply_warm_snapshot(&mut self) {
        for &(off, val) in super::mmio_init::SYSREG_INIT_VALUES {
            let abs = 0x0100_0000 + off;
            if self.claims(abs) {
                let idx = Self::idx(abs);
                self.store[idx] = val;
            }
        }
    }
}

impl Peripheral for DmaChannelController {
    fn name(&self) -> &'static str {
        "dma"
    }

    fn address_ranges(&self) -> &'static [AddressRange] {
        DMA_RANGES
    }

    fn read_word(&mut self, addr: u32) -> Result<u32, Exception> {
        let idx = Self::idx(addr);
        let (slot, within) = Self::slot_of(addr);
        if within == SUB_QUEUE_DRAIN {
            // Queue drain register — bit 8 = "ready" flag driven by
            // the tick-rearmed per-channel state. Remaining bits
            // come from the backing store so the firmware config
            // bits round-trip.
            let base = self.store[idx] & !0x100;
            let set = if self.drain_flag[slot] { 0x100 } else { 0 };
            // Command bits in the backing store still auto-clear
            // on read so trigger-style commands work.
            let clear_mask = self.pending_clear[idx];
            if clear_mask != 0 {
                self.store[idx] = self.store[idx] & !clear_mask;
                self.pending_clear[idx] = 0;
            }
            return Ok(base | set);
        }
        let val = self.store[idx];
        let clear_mask = self.pending_clear[idx];
        if clear_mask != 0 {
            self.store[idx] = val & !clear_mask;
            self.pending_clear[idx] = 0;
        }
        Ok(val)
    }

    fn write_word(&mut self, addr: u32, val: u32) -> Result<(), Exception> {
        let idx = Self::idx(addr);
        let (slot, within) = Self::slot_of(addr);
        if within == SUB_QUEUE_DRAIN {
            if val & 0x100 != 0 {
                self.drain_flag[slot] = false;
            }
            self.store[idx] = val & !0x100;
            return Ok(());
        }
        self.store[idx] = val;
        let cmd_bits = val & CMD_BIT_MASK;
        if cmd_bits != 0 {
            self.pending_clear[idx] = cmd_bits;
        }
        Ok(())
    }

    fn tick(&mut self, _cpu_instructions: u64) {
        for flag in &mut self.drain_flag {
            *flag = true;
        }
    }

    fn reset_cold(&mut self) {
        self.store.fill(0);
        self.pending_clear.fill(0);
        for flag in &mut self.drain_flag {
            *flag = true;
        }
    }

    fn reset_warm(&mut self) {
        self.reset_cold();
        self.apply_warm_snapshot();
    }

    fn snapshot(&self) -> PeripheralSnapshot {
        PeripheralSnapshot::Dma(DmaSnapshot {
            channels_enabled: 0,
            channels_busy: self.drain_flag.iter().filter(|f| **f).count() as u32,
            irq_pending_bitmap: 0,
        })
    }

    fn inject_event(&mut self, event: &PeripheralEvent) -> Result<(), PeripheralError> {
        match event {
            PeripheralEvent::Dma(ev) => match ev {
                DmaEvent::InjectQueueEntry(chan) => {
                    let idx = (*chan as usize).min(self.drain_flag.len() - 1);
                    self.drain_flag[idx] = true;
                    Ok(())
                }
                DmaEvent::InjectBusError(_) => {
                    // Bus-error path not yet modelled; ack so the
                    // UI button wires up cleanly.
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
    fn queue_drain_bit8_cycles() {
        let mut d = DmaChannelController::new();
        let addr = 0x0100_203C;
        assert_eq!(d.read_word(addr).unwrap() & 0x100, 0x100);
        d.write_word(addr, 0x100).unwrap();
        assert_eq!(d.read_word(addr).unwrap() & 0x100, 0);
        d.tick(64);
        assert_eq!(d.read_word(addr).unwrap() & 0x100, 0x100);
    }

    #[test]
    fn command_bits_autoclear_on_read() {
        let mut d = DmaChannelController::new();
        d.write_word(0x0100_2034, 0x8000_0042).unwrap();
        assert_eq!(d.read_word(0x0100_2034).unwrap(), 0x8000_0042);
        assert_eq!(d.read_word(0x0100_2034).unwrap(), 0x0000_0042);
    }

    #[test]
    fn macsec_carveouts_excluded_from_claim() {
        let d = DmaChannelController::new();
        assert!(d.claims(0x0100_2000));
        assert!(d.claims(0x0100_23FC));
        assert!(!d.claims(0x0100_2400));
        assert!(!d.claims(0x0100_2D00));
        assert!(d.claims(0x0100_2D40));
        assert!(!d.claims(0x0100_3500));
        assert!(d.claims(0x0100_3540));
        assert!(!d.claims(0x0100_3604));
        assert!(!d.claims(0x0100_3610));
        assert!(d.claims(0x0100_3614));
        assert!(d.claims(0x0100_3E3C));
        assert!(!d.claims(0x0100_4000));
    }
}
