//! Synchronous exception entry, as measured on silicon: an Instruction Error
//! (unimplemented opcode, `rtie`), SWI and TRAP_S all enter slot 2 of the
//! vector table with ILINK2 = the address execution would have continued at,
//! STATUS32_L2 = STATUS32, STATUS32 unchanged and AUX_IRQ_LV12 bit 1 set.
//! `j.f [ilink2]` resumes.

use bcm55030_emulator::cpu::exception::Exception;
use bcm55030_emulator::cpu::registers::{AUX_IRQ_LV12, REG_BLINK, REG_ILINK1, REG_ILINK2};
use bcm55030_emulator::cpu::Cpu;
use bcm55030_emulator::decoder::instruction::{Instruction, ZeroOp};

const VBASE: u32 = 0x1000;
const SLOT2: u32 = VBASE + 2 * 8;
const CODE: u32 = 0x100;

const RTIE: u32 = 0x246F_003F;
const SWI: u32 = 0x226F_003F;
const TRAP_S_0: u16 = 0x781E;
const MPY_R8_R0_R1: u32 = 0x201A_0048;
const J_F_ILINK2: u32 = 0x2020_8780;
const NOP_S: u16 = 0x78E0;
/// `b.d` to PCL + 16.
const B_D_PLUS_16: u32 = 0x0010_0020;
/// Major opcode 6: no ARCompact encoding the core implements.
const UNDECODABLE: u32 = 0x3000_0000;

fn cpu_with(code: &[u8]) -> Cpu {
    let mut cpu = Cpu::new(0x4000);
    cpu.mem.load_binary(CODE, code);
    cpu.state.aux_int_vector_base = VBASE;
    cpu.state.pc = CODE;
    cpu.state.core_regs[REG_BLINK as usize] = 0xB111_0000;
    cpu.state.core_regs[28] = 0x3000; // sp
    cpu.state.core_regs[REG_ILINK1 as usize] = 0x5EED_0029;
    cpu.state.core_regs[REG_ILINK2 as usize] = 0x5EED_0030;
    cpu
}

fn be32(w: u32) -> [u8; 4] {
    w.to_be_bytes()
}

/// Run one faulting step and check the measured entry state.
fn assert_entry(cpu: &mut Cpu, fault_pc: u32, ilink2: u32) {
    cpu.state.flag_z = true;
    cpu.state.flag_c = true;
    let st32_before = cpu.state.status32();
    let lv12_before = cpu.state.read_aux_reg(AUX_IRQ_LV12).unwrap();
    cpu.step().expect("an Instruction Error/SWI/TRAP is vectored, not returned");
    assert_eq!(cpu.state.pc, SLOT2, "slot 2 of INT_VECTOR_BASE");
    assert_eq!(cpu.state.core_regs[REG_ILINK2 as usize], ilink2, "ILINK2");
    assert_eq!(cpu.state.aux_status32_l2, st32_before, "STATUS32_L2 = STATUS32");
    assert_eq!(cpu.state.status32(), st32_before, "STATUS32 unchanged");
    assert_eq!(cpu.state.read_aux_reg(AUX_IRQ_LV12).unwrap(), lv12_before | 2, "LV12 bit 1");
    assert_eq!(cpu.state.core_regs[REG_ILINK1 as usize], 0x5EED_0029, "ILINK1 untouched");
    assert_eq!(cpu.state.core_regs[REG_BLINK as usize], 0xB111_0000, "BLINK untouched");
    assert_eq!(cpu.state.core_regs[28], 0x3000, "SP untouched");
    assert_eq!(cpu.last_exception.as_ref().map(|(pc, _)| *pc), Some(fault_pc));
}

#[test]
fn rtie_enters_slot_2_with_next_pc() {
    let mut cpu = cpu_with(&be32(RTIE));
    assert_entry(&mut cpu, CODE, CODE + 4);
}

#[test]
fn unimplemented_multiply_enters_slot_2_without_writing() {
    let mut cpu = cpu_with(&be32(MPY_R8_R0_R1));
    cpu.state.core_regs[0] = 3;
    cpu.state.core_regs[1] = 7;
    assert_entry(&mut cpu, CODE, CODE + 4);
    assert_eq!(cpu.state.core_regs[8], 0);
}

#[test]
fn swi_and_trap_s_enter_slot_2() {
    let mut cpu = cpu_with(&be32(SWI));
    assert_entry(&mut cpu, CODE, CODE + 4);

    let mut cpu = cpu_with(&TRAP_S_0.to_be_bytes());
    let d = bcm55030_emulator::decoder::decode(CODE, &cpu.mem).unwrap();
    assert!(matches!(d.inst, Instruction::ZeroOp(ZeroOp::Trap { .. })), "{:?}", d.inst);
    assert_entry(&mut cpu, CODE, CODE + 2);
}

#[test]
fn undecodable_opcode_enters_slot_2() {
    let mut cpu = cpu_with(&be32(UNDECODABLE));
    assert!(bcm55030_emulator::decoder::decode(CODE, &cpu.mem).is_err());
    assert_entry(&mut cpu, CODE, CODE + 4);
}

#[test]
fn fault_in_a_delay_slot_saves_the_branch_target() {
    let mut code = be32(B_D_PLUS_16).to_vec();
    code.extend_from_slice(&be32(RTIE));
    let mut cpu = cpu_with(&code);
    cpu.step().unwrap(); // b.d
    assert_entry(&mut cpu, CODE + 4, CODE + 16);
}

#[test]
fn j_f_ilink2_resumes_with_the_saved_flags() {
    let mut code = be32(RTIE).to_vec();
    code.extend_from_slice(&NOP_S.to_be_bytes());
    let mut cpu = cpu_with(&code);
    cpu.mem.load_binary(SLOT2, &be32(J_F_ILINK2));
    cpu.state.flag_n = true;
    let st32 = cpu.state.status32();
    cpu.step().unwrap();
    // The handler clobbers the flags, then returns.
    cpu.state.flag_n = false;
    cpu.state.flag_v = true;
    cpu.step().unwrap();
    assert_eq!(cpu.state.pc, CODE + 4);
    assert_eq!(cpu.state.status32(), st32);
    cpu.step().unwrap();
    assert_eq!(cpu.state.pc, CODE + 6);
}

#[test]
fn a_second_exception_without_clearing_behaves_the_same() {
    let mut code = be32(RTIE).to_vec();
    code.extend_from_slice(&be32(SWI));
    let mut cpu = cpu_with(&code);
    cpu.mem.load_binary(SLOT2, &be32(J_F_ILINK2));
    assert_entry(&mut cpu, CODE, CODE + 4);
    cpu.step().unwrap(); // j.f [ilink2], LV12 bit 1 left set
    assert_entry(&mut cpu, CODE + 4, CODE + 8);
    assert_eq!(cpu.exceptions_taken, 2);
}

#[test]
fn stop_on_exception_returns_the_fault() {
    let mut cpu = cpu_with(&be32(RTIE));
    cpu.stop_on_exception = true;
    assert_eq!(cpu.step(), Err(Exception::InstructionError { address: CODE }));
    assert_eq!(cpu.state.pc, CODE);
}

#[test]
fn a_fault_on_the_vector_itself_stops() {
    let mut cpu = cpu_with(&be32(RTIE));
    cpu.mem.load_binary(SLOT2, &be32(RTIE));
    cpu.step().unwrap();
    assert_eq!(cpu.state.pc, SLOT2);
    assert!(cpu.step().is_err(), "re-entering slot 2 forever is reported, not looped");
}

#[test]
fn unmeasured_classes_still_stop() {
    // `mov r0, ilink1` in user mode is a privilege violation, whose entry on
    // this core is not measured: it is returned, not vectored.
    const MOV_R0_ILINK1: u32 = 0x200A_0740;
    let mut cpu = cpu_with(&be32(MOV_R0_ILINK1));
    cpu.state.flag_u = true;
    assert_eq!(cpu.step(), Err(Exception::PrivilegeViolation { address: CODE }));
    assert_eq!(cpu.state.pc, CODE);
    assert_eq!(cpu.exceptions_taken, 0);
}
