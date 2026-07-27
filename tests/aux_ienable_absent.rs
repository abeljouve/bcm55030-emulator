//! `AUX_IENABLE` (aux `0x40C`) is not implemented on the BCM55030.
//!
//! DATASHEET §6.1, measured on silicon: a read of `0x40C` returns the
//! `IDENTITY` value that every unimplemented auxiliary address returns, a
//! write is discarded (writing back the read value with bit 7 set and
//! re-reading returns the original), and there is **no per-line interrupt
//! enable mask** on this device. A line's level comes from `AUX_IRQ_LEV`
//! (`0x200`), delivery of that level is gated by `STATUS32.E1` / `E2`, and
//! individual sources are masked at the peripheral.
//!
//! The evidence that settles it: Timer 1 / IRQ 7 was delivered 19 001 times
//! during the §4.1.4 hardware-loop characterization, in a run whose own
//! `0x40C` readback showed bit 7 clear.
//!
//! The emulator used to `&`-mask delivery with an `aux_ienable` field
//! initialised to 0, and then preset it to `0xFFFFFFFF` in four places to
//! neutralise the mask it had invented. Firmware written against that model
//! would run here and wedge on silicon — this test pins the absence.

use bcm55030_emulator::cpu::registers::{CpuState, AUX_IENABLE, IDENTITY_VALUE};
use bcm55030_emulator::cpu::Cpu;
use bcm55030_emulator::soc::bank::BootMode;

#[test]
fn reading_aux_40c_returns_identity() {
    let state = CpuState::new();
    assert_eq!(
        state.read_aux_reg(AUX_IENABLE).unwrap(),
        IDENTITY_VALUE,
        "0x40C is absent, so it reads back as any absent aux does"
    );
}

#[test]
fn writing_aux_40c_has_no_observable_effect() {
    let mut state = CpuState::new();

    // The datasheet's own probe: write back the read value with bit 7 set.
    state.write_aux_reg(AUX_IENABLE, IDENTITY_VALUE | 0x80).unwrap();
    assert_eq!(state.read_aux_reg(AUX_IENABLE).unwrap(), IDENTITY_VALUE);

    // And the two extremes, which a firmware would use as a mask.
    state.write_aux_reg(AUX_IENABLE, 0x0000_0000).unwrap();
    assert_eq!(state.read_aux_reg(AUX_IENABLE).unwrap(), IDENTITY_VALUE);
    state.write_aux_reg(AUX_IENABLE, 0xFFFF_FFFF).unwrap();
    assert_eq!(state.read_aux_reg(AUX_IENABLE).unwrap(), IDENTITY_VALUE);
}

/// The load-bearing one: clearing every bit of `0x40C` must not stop a line
/// from being delivered. Under the old model this wrote 0 into the mask and
/// the interrupt was silently dropped.
#[test]
fn clearing_aux_40c_does_not_mask_a_pending_line() {
    let mut cpu = Cpu::new_bcm55030(BootMode::Cold);

    // A 16-bit NOP_S at PC=0; executing it then takes the pending IRQ.
    cpu.mem.load_binary(0x0, &[0x78, 0xE0]);
    // Level-1 vector 5 in the SRAM IVT (no NCO channel programmed).
    cpu.state.aux_int_vector_base = 0x1000;
    cpu.mem.load_binary(0x1000 + 5 * 8, &[0x20, 0x20, 0x0F, 0x80, 0x00, 0x03, 0xF1, 0x08]);

    cpu.state.pc = 0x0;
    cpu.state.aux_irq_lev = 0; // IRQ 5 = level 1
    cpu.state.flag_e1 = true;
    cpu.state.aux_irq_pending = 1 << 5;

    // Software "disables" every line the only way it can address them.
    cpu.state.write_aux_reg(AUX_IENABLE, 0).unwrap();

    cpu.step().unwrap();

    assert_eq!(
        cpu.state.pc,
        0x1000 + 5 * 8,
        "IRQ 5 must still be taken: 0x40C is not an enable mask"
    );
    assert_eq!(cpu.state.aux_icause1, 5);
}

/// The masks that *do* exist still work: `STATUS32.E1` gates level 1.
#[test]
fn status32_e1_still_gates_level_one() {
    let mut cpu = Cpu::new_bcm55030(BootMode::Cold);
    cpu.mem.load_binary(0x0, &[0x78, 0xE0]);
    cpu.state.aux_int_vector_base = 0x1000;
    cpu.state.pc = 0x0;
    cpu.state.aux_irq_lev = 0;
    cpu.state.flag_e1 = false; // interrupts globally disabled
    cpu.state.aux_irq_pending = 1 << 5;

    cpu.step().unwrap();

    assert_eq!(cpu.state.pc, 0x2, "E1=0 must hold the line off");
    assert_eq!(cpu.state.aux_irq_pending, 1 << 5, "and leave it pending");
}
