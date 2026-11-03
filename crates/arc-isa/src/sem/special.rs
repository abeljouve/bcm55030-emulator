//! Zero-operand instructions, `LP` and `FLAG`.

use crate::condition::ConditionCode;
use crate::decoder::instruction::{Operand, ZeroOp};
use crate::dom::{AuxReg, Dom, Flag};
use crate::exception::Exception;

use super::regs::{cond_holds, operand, set_status32};

#[inline]
pub fn zero_op<D: Dom>(d: &mut D, zop: &ZeroOp) -> Result<(), Exception> {
    match *zop {
        ZeroOp::Nop | ZeroOp::Sync => Ok(()),
        ZeroOp::Sleep { u6 } => {
            // A non-zero operand sets E1 (bit 3) and E2 (bit 4) while sleeping.
            if u6 != 0 {
                let e1 = d.kb((u6 >> 3) & 1 != 0);
                let e2 = d.kb((u6 >> 4) & 1 != 0);
                d.set_flag(Flag::E1, e1);
                d.set_flag(Flag::E2, e2);
            }
            d.sleep();
            Ok(())
        }
        ZeroOp::Swi => Err(Exception::Trap { param: 0 }),
        ZeroOp::Brk => {
            d.halt();
            Ok(())
        }
        ZeroOp::Trap { param } => Err(Exception::Trap { param }),
        // Not implemented on this core: an Instruction Error, exactly as an
        // unknown opcode (observed on silicon). Returns use `j.f [ilinkN]`.
        ZeroOp::Rtie => Err(Exception::InstructionError { address: d.pc() }),
    }
}

/// `LP`: set up a zero-overhead loop ending at PCL + `offset`, or, when the
/// condition fails, branch there without setting one up.
#[inline]
pub fn lp<D: Dom>(
    d: &mut D,
    offset: u32,
    cc: Option<ConditionCode>,
    size: u32,
) -> Result<(), Exception> {
    let end = (d.pc() & !3).wrapping_add(offset);
    let start = d.pc().wrapping_add(size);
    let setup = |d: &mut D| -> Result<(), Exception> {
        let f = d.kb(false);
        d.set_flag(Flag::L, f);
        let s = d.k(start);
        d.set_aux(AuxReg::LpStart, s);
        let e = d.k(end);
        d.set_aux(AuxReg::LpEnd, e);
        Ok(())
    };
    match cc {
        None => setup(d),
        Some(cc) => {
            let g = cond_holds(d, cc);
            let ng = d.bnot(g);
            d.when(ng, |d| {
                d.lp_skip(end);
                Ok(())
            })?;
            d.when(g, setup)
        }
    }
}

/// `FLAG`: load STATUS32 from the operand. Bit 0 (H) halts, and is privileged;
/// in user mode only Z, N, C and V are written.
#[inline]
pub fn flag<D: Dom>(d: &mut D, src: Operand, cc: Option<ConditionCode>) -> Result<(), Exception> {
    let body = |d: &mut D| -> Result<(), Exception> {
        let val = operand(d, src)?;
        let h = d.bit(val, 0);
        let u = d.flag(Flag::U);
        d.when(h, |d| {
            d.fault_if(u, Exception::PrivilegeViolation { address: d.pc() })?;
            d.halt();
            set_status32(d, val);
            Ok(())
        })?;
        let nh = d.bnot(h);
        d.when(nh, |d| {
            d.when(u, |d| {
                for f in [Flag::Z, Flag::N, Flag::C, Flag::V] {
                    let b = d.bit(val, f.status32_bit() as u8);
                    d.set_flag(f, b);
                }
                Ok(())
            })?;
            let nu = d.bnot(u);
            d.when(nu, |d| {
                set_status32(d, val);
                Ok(())
            })
        })
    };
    match cc {
        None => body(d),
        Some(cc) => {
            let g = cond_holds(d, cc);
            d.when(g, body)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::flat::FlatMachine;
    use crate::regs::REG_ILINK1;

    #[test]
    fn rtie_faults_instruction_error_vec2() {
        // Observed on silicon: this core does not implement `rtie`; it raises
        // vector 2 (Instruction Error) and must not restore STATUS32 or the PC.
        let mut m = FlatMachine::new(0);
        m.pc = 0x1234;
        m.set_flag(Flag::A1, true);
        m.set_aux(AuxReg::Status32L1, 0xDEAD_BEEF);
        m.regs[REG_ILINK1 as usize] = 0x9999;
        let before = m.clone();
        assert_eq!(zero_op(&mut m, &ZeroOp::Rtie), Err(Exception::InstructionError { address: 0x1234 }));
        assert_eq!(Exception::InstructionError { address: 0x1234 }.vector_number(), 0x02);
        assert_eq!(m, before, "rtie must not change any state");
    }
}
