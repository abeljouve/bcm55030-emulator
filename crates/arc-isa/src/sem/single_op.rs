//! Single-operand operations (major 0x04, sub-opcode 0x2F, and their compact forms).

use crate::condition::ConditionCode;
use crate::decoder::instruction::{Operand, SingleOp};
use crate::dom::{Dom, Flag};
use crate::exception::Exception;

use super::regs::{cond_holds, operand, write_dest};

#[inline]
pub fn single_op<D: Dom>(
    d: &mut D,
    op: SingleOp,
    dst: Operand,
    src: Operand,
    set_flags: bool,
    cc: Option<ConditionCode>,
) -> Result<(), Exception> {
    // EX is not implemented on this core: it faults and memory is left
    // untouched. Observed on silicon; the false-condition case is INFERRED.
    if matches!(op, SingleOp::Ex) {
        return Err(Exception::InstructionError { address: d.pc() });
    }
    // One copy of the body, guarded: no condition is "always".
    let g = match cc {
        None => d.kb(true),
        Some(cc) => cond_holds(d, cc),
    };
    d.when(g, move |d| body(d, op, dst, src, set_flags))
}

#[inline(always)]
fn body<D: Dom>(
    d: &mut D,
    op: SingleOp,
    dst: Operand,
    src: Operand,
    set_flags: bool,
) -> Result<(), Exception> {
    let val = operand(d, src)?;
    let one = d.k(1);
    let (r, c, v) = match op {
        // One-bit left shifts also write V = bit31 ^ bit30 of the source
        // (= N ^ C after the shift): observed on silicon for ASL and RLC.
        SingleOp::Asl | SingleOp::Rlc => {
            let c = d.bit(val, 31);
            let b30 = d.bit(val, 30);
            let v = d.bxor(c, b30);
            let sh = d.shl(val, one);
            let r = if op == SingleOp::Rlc {
                let cin = d.flag(Flag::C);
                let ci = d.b2v(cin);
                d.or(sh, ci)
            } else {
                sh
            };
            (r, Some(c), Some(v))
        }
        SingleOp::Asr => {
            let c = d.bit(val, 0);
            (d.ashr(val, one), Some(c), None)
        }
        SingleOp::Lsr => {
            let c = d.bit(val, 0);
            (d.lshr(val, one), Some(c), None)
        }
        SingleOp::Ror | SingleOp::Rrc => {
            let c = d.bit(val, 0);
            let lo = d.lshr(val, one);
            let top = if op == SingleOp::Ror {
                val
            } else {
                let cin = d.flag(Flag::C);
                d.b2v(cin)
            };
            let k31 = d.k(31);
            let hi = d.shl(top, k31);
            (d.or(lo, hi), Some(c), None)
        }
        SingleOp::Sexb => (d.sext(val, 8), None, None),
        SingleOp::Sexw => (d.sext(val, 16), None, None),
        SingleOp::Extb => (d.zext(val, 8), None, None),
        SingleOp::Extw => (d.zext(val, 16), None, None),
        SingleOp::Abs => {
            // C: the source was negative; V: it was the one value whose
            // negation does not fit (0x80000000, which is returned as is).
            let zero = d.k(0);
            let negative = d.slt(val, zero);
            let n = d.neg(val);
            let r = d.ite(negative, n, val);
            let min = d.k(0x8000_0000);
            let v = d.eq(val, min);
            (r, Some(negative), Some(v))
        }
        SingleOp::Not => (d.not(val), None, None),
        SingleOp::Ex => unreachable!("EX faults before it computes"),
    };
    write_dest(d, dst, r)?;
    if set_flags {
        let zero = d.k(0);
        let z = d.eq(r, zero);
        let n = d.bit(r, 31);
        d.set_flag(Flag::Z, z);
        d.set_flag(Flag::N, n);
        if let Some(c) = c {
            d.set_flag(Flag::C, c);
        }
        if let Some(v) = v {
            d.set_flag(Flag::V, v);
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::flat::FlatMachine;

    fn run(op: SingleOp, src: u32, flags_in: (bool, bool, bool, bool)) -> (u32, [bool; 4]) {
        let mut m = FlatMachine::new(0);
        m.regs[0] = src;
        m.set_flag(Flag::Z, flags_in.0);
        m.set_flag(Flag::N, flags_in.1);
        m.set_flag(Flag::C, flags_in.2);
        m.set_flag(Flag::V, flags_in.3);
        single_op(&mut m, op, Operand::Reg(1), Operand::Reg(0), true, None).unwrap();
        (m.regs[1], [m.flag(Flag::Z), m.flag(Flag::N), m.flag(Flag::C), m.flag(Flag::V)])
    }

    #[test]
    fn asl_one_bit_sets_v_on_sign_change() {
        // (source, result, N, C, V), measured on silicon, whatever V was before.
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
        let mut m = FlatMachine::new(0x100);
        m.write(0x40, crate::decoder::instruction::DataSize::Word, 0x1122_3344).unwrap();
        m.pc = 0x10;
        m.regs[0] = 0x40;
        m.regs[1] = 0xAABB_CCDD;
        let r = single_op(&mut m, SingleOp::Ex, Operand::Reg(1), Operand::Reg(0), false, None);
        assert_eq!(r, Err(Exception::InstructionError { address: 0x10 }));
        assert_eq!(m.regs[1], 0xAABB_CCDD);
        assert_eq!(m.read(0x40, crate::decoder::instruction::DataSize::Word).unwrap(), 0x1122_3344);
    }
}
