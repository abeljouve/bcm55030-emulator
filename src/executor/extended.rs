use crate::cpu::condition::ConditionCode;
use crate::cpu::exception::Exception;
use crate::cpu::registers::CpuState;
use crate::decoder::instruction::{ExtArithOp, Operand};

use super::{resolve_value, write_dest};

pub fn execute_ext_arith(
    op: ExtArithOp,
    dst: Operand,
    src1: Operand,
    src2: Operand,
    set_flags: bool,
    cc: Option<ConditionCode>,
    state: &mut CpuState,
) -> Result<(), Exception> {
    // The saturating/DSP group is not implemented on this core: it faults
    // whatever its condition. INFERRED for the false-condition case: only
    // unconditional forms were measured.
    if op.is_absent_on_core() {
        return Err(Exception::InstructionError { address: state.pc });
    }

    if let Some(cc) = cc {
        if !cc.evaluate(state.flag_z, state.flag_n, state.flag_c, state.flag_v) {
            return Ok(());
        }
    }

    let a = resolve_value(src1, state)?;
    let b = resolve_value(src2, state)?;

    match op {
        ExtArithOp::Asl | ExtArithOp::Lsr | ExtArithOp::Asr | ExtArithOp::Ror => {
            let (result, carry) = barrel_shift(op, a, b);
            write_dest(dst, result, state)?;
            if set_flags {
                state.flag_z = result == 0;
                state.flag_n = (result >> 31) != 0;
                state.flag_c = carry;
            }
        }
        ExtArithOp::Norm | ExtArithOp::Normw => {
            // The destination gets the normalisation count, but Z and N
            // describe the SOURCE (NORMW: its low half). C and V are kept.
            // Observed on silicon.
            let (result, src_zero, src_neg) = if op == ExtArithOp::Norm {
                (norm32(a), a == 0, (a >> 31) != 0)
            } else {
                let v16 = a as u16;
                (norm16(v16), v16 == 0, (v16 >> 15) != 0)
            };
            write_dest(dst, result, state)?;
            if set_flags {
                state.flag_z = src_zero;
                state.flag_n = src_neg;
            }
        }
        ExtArithOp::Swap => {
            let result = a.rotate_left(16);
            write_dest(dst, result, state)?;
            if set_flags {
                state.flag_z = result == 0;
                state.flag_n = (result >> 31) != 0;
            }
        }
        _ => unreachable!("{op:?} faults before it computes"),
    }

    Ok(())
}

/// Multi-bit shift through the barrel shifter. The count is taken modulo 32
/// (a count of 32 shifts by 0, 33 by 1). C is the last bit shifted out, and is
/// cleared when the masked count is 0. V is never written. Observed on silicon
/// for register counts 0..=63 on all four operations; the immediate (u6)
/// forms go through the same operation and are assumed to match (INFERRED).
fn barrel_shift(op: ExtArithOp, a: u32, count: u32) -> (u32, bool) {
    let s = count & 31;
    if s == 0 {
        return (a, false);
    }
    match op {
        ExtArithOp::Asl => (a << s, (a >> (32 - s)) & 1 != 0),
        ExtArithOp::Lsr => (a >> s, (a >> (s - 1)) & 1 != 0),
        ExtArithOp::Asr => (((a as i32) >> s) as u32, (a >> (s - 1)) & 1 != 0),
        ExtArithOp::Ror => (a.rotate_right(s), (a >> (s - 1)) & 1 != 0),
        _ => unreachable!(),
    }
}

/// Leading sign bits minus one: how far `v` can shift left before its sign
/// changes. 0 and all-ones give 31.
fn norm32(v: u32) -> u32 {
    if v >> 31 == 0 {
        v.leading_zeros().saturating_sub(1).min(31)
    } else {
        (!v).leading_zeros().saturating_sub(1).min(31)
    }
}

fn norm16(v: u16) -> u32 {
    if v >> 15 == 0 {
        v.leading_zeros().saturating_sub(1).min(15)
    } else {
        (!v).leading_zeros().saturating_sub(1).min(15)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn run(op: ExtArithOp, a: u32, b: u32, flags_in: (bool, bool, bool, bool)) -> (u32, [bool; 4]) {
        let mut st = CpuState::new();
        st.core_regs[0] = a;
        st.core_regs[1] = b;
        (st.flag_z, st.flag_n, st.flag_c, st.flag_v) = flags_in;
        execute_ext_arith(op, Operand::Reg(2), Operand::Reg(0), Operand::Reg(1), true, None, &mut st)
            .unwrap();
        (st.core_regs[2], [st.flag_z, st.flag_n, st.flag_c, st.flag_v])
    }

    #[test]
    fn shift_count_is_taken_modulo_32() {
        for count in [0u32, 1, 31, 32, 33, 63] {
            let s = count & 31;
            assert_eq!(run(ExtArithOp::Asl, 5, count, (false, false, false, false)).0, 5u32 << s);
            assert_eq!(run(ExtArithOp::Lsr, 0xA000_0000, count, (false, false, false, false)).0, 0xA000_0000u32 >> s);
            assert_eq!(
                run(ExtArithOp::Asr, 0x8000_0000, count, (false, false, false, false)).0,
                ((0x8000_0000u32 as i32) >> s) as u32
            );
            assert_eq!(run(ExtArithOp::Ror, 0x1234_5678, count, (false, false, false, false)).0, 0x1234_5678u32.rotate_right(s));
        }
    }

    #[test]
    fn shift_carry_is_last_bit_out_and_cleared_by_a_zero_count() {
        // A zero (or 32) count clears C even when it was set, and keeps V.
        for op in [ExtArithOp::Asl, ExtArithOp::Lsr, ExtArithOp::Asr, ExtArithOp::Ror] {
            for count in [0, 32] {
                let (_, f) = run(op, 0xFFFF_FFFF, count, (false, false, true, true));
                assert!(!f[2], "{op:?} count {count}: C must be cleared");
                assert!(f[3], "{op:?} count {count}: V must be kept");
            }
        }
        assert!(run(ExtArithOp::Asl, 0x8000_0000, 1, (false, false, false, false)).1[2]);
        assert!(!run(ExtArithOp::Asl, 0x0000_0001, 32 + 31, (false, false, false, false)).1[2]);
        assert!(run(ExtArithOp::Lsr, 0x0000_0002, 2, (false, false, false, false)).1[2]);
        assert!(run(ExtArithOp::Asr, 0x0000_0001, 33, (false, false, false, false)).1[2]);
        assert!(run(ExtArithOp::Ror, 0x0000_0001, 1, (false, false, false, false)).1[2]);
    }

    #[test]
    fn norm_flags_describe_the_source() {
        // (source, count, Z, N): C and V preset to 1 must survive.
        for (src, count, z, n) in [
            (0u32, 31, true, false),
            (0x8000_0000, 0, false, true),
            (0xFFFF_FFFF, 31, false, true),
            (1, 30, false, false),
        ] {
            let (r, f) = run(ExtArithOp::Norm, src, 0, (false, false, true, true));
            assert_eq!((r, f), (count, [z, n, true, true]), "norm.f {src:#x}");
        }
        for (src, count, z, n) in [
            (0xABCD_0000u32, 15, true, false),
            (0x0000_8000, 0, false, true),
            (0x0000_FFFF, 15, false, true),
            (0x0000_0001, 14, false, false),
        ] {
            let (r, f) = run(ExtArithOp::Normw, src, 0, (false, false, true, true));
            assert_eq!((r, f), (count, [z, n, true, true]), "normw.f {src:#x}");
        }
    }

    #[test]
    fn saturating_group_faults() {
        for op in [
            ExtArithOp::Adds, ExtArithOp::Subs, ExtArithOp::Divaw, ExtArithOp::Asls,
            ExtArithOp::Asrs, ExtArithOp::Addsdw, ExtArithOp::Subsdw, ExtArithOp::Sat16,
            ExtArithOp::Rnd16, ExtArithOp::Abss, ExtArithOp::Abssw, ExtArithOp::Negs,
            ExtArithOp::Negsw,
        ] {
            let mut st = CpuState::new();
            st.pc = 0x100;
            st.core_regs[2] = 0xDEAD;
            let r = execute_ext_arith(op, Operand::Reg(2), Operand::Reg(0), Operand::Reg(1), true, None, &mut st);
            assert_eq!(r, Err(Exception::InstructionError { address: 0x100 }), "{op:?}");
            assert_eq!(st.core_regs[2], 0xDEAD, "{op:?} must not write");
        }
    }
}
