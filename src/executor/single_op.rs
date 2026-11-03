use crate::cpu::condition::ConditionCode;
use crate::cpu::exception::Exception;
use crate::cpu::registers::CpuState;
use crate::decoder::fields;
use crate::decoder::instruction::{Operand, SingleOp};

use super::{resolve_value, write_dest};

pub fn execute_single_op(
    op: SingleOp,
    dst: Operand,
    src: Operand,
    set_flags: bool,
    cc: Option<ConditionCode>,
    state: &mut CpuState,
) -> Result<(), Exception> {
    // EX is not implemented on this core: it faults and memory is left
    // untouched. Observed on silicon; the false-condition case is INFERRED.
    if matches!(op, SingleOp::Ex) {
        return Err(Exception::InstructionError { address: state.pc });
    }

    if let Some(cc) = cc {
        if !cc.evaluate(state.flag_z, state.flag_n, state.flag_c, state.flag_v) {
            return Ok(());
        }
    }

    let val = resolve_value(src, state)?;
    let carry_in = state.flag_c;
    let mut overflow: Option<bool> = None;

    let (result, carry) = match op {
        // One-bit left shifts also write V = bit31 ^ bit30 of the source
        // (= N ^ C after the shift): observed on silicon for ASL and RLC.
        SingleOp::Asl => {
            let c = (val >> 31) != 0;
            overflow = Some(((val >> 31) ^ (val >> 30)) & 1 != 0);
            (val << 1, Some(c))
        }
        SingleOp::Asr => {
            let c = (val & 1) != 0;
            (((val as i32) >> 1) as u32, Some(c))
        }
        SingleOp::Lsr => {
            let c = (val & 1) != 0;
            (val >> 1, Some(c))
        }
        SingleOp::Ror => {
            let c = (val & 1) != 0;
            (val.rotate_right(1), Some(c))
        }
        SingleOp::Rrc => {
            let c = (val & 1) != 0;
            let result = (val >> 1) | ((carry_in as u32) << 31);
            (result, Some(c))
        }
        SingleOp::Sexb => (fields::sign_extend(val & 0xFF, 8) as u32, None),
        SingleOp::Sexw => (fields::sign_extend(val & 0xFFFF, 16) as u32, None),
        SingleOp::Extb => (val & 0xFF, None),
        SingleOp::Extw => (val & 0xFFFF, None),
        SingleOp::Abs => {
            let signed = val as i32;
            if signed == i32::MIN {
                overflow = Some(true);
                (0x80000000u32, Some(true))
            } else if signed < 0 {
                overflow = Some(false);
                ((-signed) as u32, Some(true))
            } else {
                overflow = Some(false);
                (val, Some(false))
            }
        }
        SingleOp::Not => (!val, None),
        SingleOp::Rlc => {
            let c = (val >> 31) != 0;
            overflow = Some(((val >> 31) ^ (val >> 30)) & 1 != 0);
            let result = (val << 1) | (carry_in as u32);
            (result, Some(c))
        }
        SingleOp::Ex => unreachable!("EX faults before it computes"),
    };

    write_dest(dst, result, state)?;

    if set_flags {
        state.flag_z = result == 0;
        state.flag_n = (result >> 31) != 0;
        if let Some(c) = carry {
            state.flag_c = c;
        }
        if let Some(v) = overflow {
            state.flag_v = v;
        }
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn run(op: SingleOp, src: u32, flags_in: (bool, bool, bool, bool)) -> (u32, [bool; 4]) {
        let mut st = CpuState::new();
        st.core_regs[0] = src;
        (st.flag_z, st.flag_n, st.flag_c, st.flag_v) = flags_in;
        execute_single_op(op, Operand::Reg(1), Operand::Reg(0), true, None, &mut st).unwrap();
        (st.core_regs[1], [st.flag_z, st.flag_n, st.flag_c, st.flag_v])
    }

    #[test]
    fn asl_one_bit_sets_v_on_sign_change() {
        // (source, result, N, C, V), whatever V was before.
        for (src, res, n, c, v) in [
            (0x4000_0000u32, 0x8000_0000u32, true, false, true),
            (0x7FFF_FFFF, 0xFFFF_FFFE, true, false, true),
            (0x8000_0000, 0x0000_0000, false, true, true),
            (0xC000_0000, 0x8000_0000, true, true, false),
            (0x0000_0001, 0x0000_0002, false, false, false),
        ] {
            for v_in in [false, true] {
                let (r, f) = run(SingleOp::Asl, src, (false, false, false, v_in));
                assert_eq!((r, f[1], f[2], f[3]), (res, n, c, v), "asl.f {src:#x} V_in={v_in}");
            }
        }
    }

    #[test]
    fn rlc_writes_v_like_asl() {
        for (src, v) in [(0x4000_0000u32, true), (0x8000_0000, true), (0xC000_0000, false), (1, false)] {
            for v_in in [false, true] {
                assert_eq!(run(SingleOp::Rlc, src, (false, false, true, v_in)).1[3], v, "rlc.f {src:#x}");
            }
        }
    }

    #[test]
    fn right_shifts_keep_v() {
        for op in [SingleOp::Lsr, SingleOp::Asr, SingleOp::Ror, SingleOp::Rrc] {
            assert!(run(op, 0x8000_0001, (false, false, false, true)).1[3], "{op:?}");
            assert!(!run(op, 0x8000_0001, (false, false, false, false)).1[3], "{op:?}");
        }
    }

    #[test]
    fn ex_faults_without_writing() {
        // The executor no longer reaches memory for EX, so memory is untouched.
        let mut st = CpuState::new();
        st.pc = 0x10;
        st.core_regs[0] = 0x40;
        st.core_regs[1] = 0xAABB_CCDD;
        let r = execute_single_op(SingleOp::Ex, Operand::Reg(1), Operand::Reg(0), false, None, &mut st);
        assert_eq!(r, Err(Exception::InstructionError { address: 0x10 }));
        assert_eq!(st.core_regs[1], 0xAABB_CCDD);
    }
}
