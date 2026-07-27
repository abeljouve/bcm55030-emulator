//! A stalled PC is not a reboot: the ARC timer `CONTROL.W` bit decides.
//!
//! The emulator used to warm-reset the SoC after 64 consecutive same-PC steps,
//! unconditionally. That is wrong twice over:
//!
//! 1. **Unconditional.** Real silicon resets on a spin only if a watchdog was
//!    armed. `docs/isa/05-registers.md` Figure 66 gives the ARC timer control
//!    layout — bit 0 `IE`, bit 1 `NH`, bit 2 `W` ("enable watchdog reset
//!    signal"), bit 3 `IP` — and the emulator implemented every bit of it
//!    EXCEPT `W`, then compensated with a detector that rebooted whether or
//!    not the firmware had asked for one.
//! 2. **Far too fast.** A firmware watchdog set for ~100 ms is ~15.6 M
//!    instructions at 156.25 MHz. Rebooting 64 instructions in makes the
//!    reboot cadence ~250 000x too fast, which is why the emulator showed a
//!    tight reboot loop where silicon showed a long silence.
//!
//! The cost was diagnostic, not cosmetic: an invented reboot wipes SRAM and
//! the register file, i.e. exactly the evidence a hang investigation needs.
//! Same failure shape as the icache zero-fill (D4) in the same tracker.

use bcm55030_emulator::cpu::registers::{PauseReason, AUX_CONTROL1, AUX_LIMIT1};
use bcm55030_emulator::cpu::{Cpu, SpinVerdict};
use bcm55030_emulator::soc::bank::BootMode;

/// `b .` — branch to self, 32-bit form (offset 0).
const BRANCH_TO_SELF: [u8; 4] = [0x00, 0x00, 0x00, 0x00];

/// Park the CPU on a branch-to-self at `pc`.
fn spinning_cpu() -> Cpu {
    let mut cpu = Cpu::new_bcm55030(BootMode::Warm);
    // A 32-bit all-zero word decodes as `b 0` = branch to self on ARCompact.
    let pc = 0x0004_0000u32;
    cpu.mem.load_binary(pc, &BRANCH_TO_SELF);
    cpu.state.pc = pc;
    cpu.state.halted = false;
    cpu.state.paused = false;
    cpu
}

/// Clear every interrupt path so a spin is genuinely unbreakable.
fn mask_all_interrupts(cpu: &mut Cpu) {
    cpu.state.flag_e1 = false;
    cpu.state.flag_e2 = false;
    cpu.state.aux_irq_pending = 0;
    cpu.state.aux_control0 = 0;
    cpu.state.aux_limit0 = 0;
    cpu.state.aux_control1 = 0;
    cpu.state.aux_limit1 = 0;
}

#[test]
fn a_spin_with_no_watchdog_and_no_interrupt_is_a_hang_not_a_reboot() {
    let mut cpu = spinning_cpu();
    mask_all_interrupts(&mut cpu);
    let pc = cpu.state.pc;
    // Leave a witness in SRAM: a fabricated reboot would erase it.
    cpu.mem.write_word(0x0003_0000, 0xDEAD_BEEF).unwrap();

    assert_eq!(cpu.classify_spin(), SpinVerdict::Hung);
    cpu.run(10_000).unwrap();

    assert!(cpu.state.halted, "an unbreakable spin must stop the run");
    assert_eq!(
        cpu.state.pause_reason,
        PauseReason::SpinNoWatchdog(pc),
        "the halt must NAME the mechanism and the spinning PC"
    );
    assert_eq!(cpu.state.pc, pc, "the PC must be preserved for the investigator");
    assert_eq!(
        cpu.mem.read_word(0x0003_0000).unwrap(),
        0xDEAD_BEEF,
        "SRAM must survive: destroying it is what made this class undiagnosable"
    );
}

#[test]
fn a_spin_with_an_armed_watchdog_does_reset_and_at_the_programmed_count() {
    let mut cpu = spinning_cpu();
    mask_all_interrupts(&mut cpu);
    // Arm Timer 1 as a watchdog: W (bit 2), no IE, ~100 ms worth of ticks.
    cpu.state.write_aux_reg(AUX_LIMIT1, 15_625_000).unwrap();
    cpu.state.write_aux_reg(AUX_CONTROL1, 0b100).unwrap();
    cpu.mem.write_word(0x0003_0000, 0xDEAD_BEEF).unwrap();

    assert_eq!(cpu.classify_spin(), SpinVerdict::WatchdogArmed);
    let _ = cpu.run(10_000); // post-reset the fixture has no flash image; the reset is the point

    assert!(!cpu.state.halted, "an armed watchdog resets the chip, it does not halt it");
    assert_ne!(
        cpu.mem.read_word(0x0003_0000).unwrap(),
        0xDEAD_BEEF,
        "a real watchdog reset DOES wipe SRAM -- that part was never in doubt"
    );
}

#[test]
fn the_watchdog_bit_is_what_resets_not_the_spin_detector() {
    // No spin at all: a running CPU whose watchdog expires must still reset.
    let mut cpu = Cpu::new_bcm55030(BootMode::Warm);
    let pc = 0x0004_0000u32;
    // 16 x `nop_s` (0x78E0) so the PC keeps moving and the spin detector
    // never engages.
    let nops: Vec<u8> = std::iter::repeat([0x78u8, 0xE0]).take(64).flatten().collect();
    cpu.mem.load_binary(pc, &nops);
    cpu.state.pc = pc;
    mask_all_interrupts(&mut cpu);
    cpu.state.write_aux_reg(AUX_LIMIT1, 8).unwrap();
    cpu.state.write_aux_reg(AUX_CONTROL1, 0b100).unwrap();
    cpu.mem.write_word(0x0003_0000, 0xDEAD_BEEF).unwrap();

    let _ = cpu.run(64);

    assert_ne!(
        cpu.mem.read_word(0x0003_0000).unwrap(),
        0xDEAD_BEEF,
        "CONTROL.W (docs/isa/05-registers.md Fig. 66) must reset the chip on limit"
    );
}

#[test]
fn a_timer_without_the_w_bit_only_interrupts_and_never_resets() {
    let mut cpu = Cpu::new_bcm55030(BootMode::Warm);
    let pc = 0x0004_0000u32;
    let nops: Vec<u8> = std::iter::repeat([0x78u8, 0xE0]).take(64).flatten().collect();
    cpu.mem.load_binary(pc, &nops);
    cpu.state.pc = pc;
    mask_all_interrupts(&mut cpu);
    cpu.state.write_aux_reg(AUX_LIMIT1, 8).unwrap();
    cpu.state.write_aux_reg(AUX_CONTROL1, 0b001).unwrap(); // IE only, no W
    cpu.mem.write_word(0x0003_0000, 0xDEAD_BEEF).unwrap();

    cpu.run(64).unwrap();

    assert_eq!(
        cpu.mem.read_word(0x0003_0000).unwrap(),
        0xDEAD_BEEF,
        "IE without W raises an interrupt; it must not reset the chip"
    );
    assert_ne!(cpu.state.aux_irq_pending, 0, "…but the interrupt line must be raised");
}

#[test]
fn a_spin_that_an_interrupt_can_still_break_keeps_running() {
    let mut cpu = spinning_cpu();
    mask_all_interrupts(&mut cpu);
    // Interrupts enabled + a timer source: the classic "wait here for the ISR"
    // idiom. Silicon sits in it as long as it likes; the emulator used to
    // reboot out of it after 64 instructions.
    cpu.state.flag_e1 = true;
    cpu.state.write_aux_reg(AUX_LIMIT1, 1_000_000).unwrap();
    cpu.state.write_aux_reg(AUX_CONTROL1, 0b001).unwrap();
    let pc = cpu.state.pc;
    cpu.mem.write_word(0x0003_0000, 0xDEAD_BEEF).unwrap();

    assert_eq!(cpu.classify_spin(), SpinVerdict::Interruptible);
    cpu.run(5_000).unwrap();

    assert!(!cpu.state.halted, "an interruptible wait is not a hang");
    assert_eq!(cpu.state.pc, pc, "still waiting, exactly where it was");
    assert_eq!(
        cpu.mem.read_word(0x0003_0000).unwrap(),
        0xDEAD_BEEF,
        "no reboot: nothing was wiped"
    );
}
