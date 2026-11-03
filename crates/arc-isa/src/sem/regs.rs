//! Core registers, STATUS32 and condition codes.

use crate::condition::ConditionCode;
use crate::decoder::instruction::Operand;
use crate::dom::{Dom, Flag};
use crate::exception::Exception;
use crate::regs::{REG_ILINK1, REG_ILINK2, REG_LIMM, REG_PCL};

/// Read core register `r` as an operand. ILINK1/2 fault in user mode, r61 is
/// reserved, r62 (the long-immediate marker) reads 0 once the decoder has
/// resolved it, r63 is PCL.
#[inline(always)]
pub fn read_core_reg<D: Dom>(d: &mut D, r: u8) -> Result<D::V, Exception> {
    // The ordinary registers first: the arm order shapes the generated code.
    match r {
        0..=28 | 31..=60 => Ok(d.reg(r)),
        REG_ILINK1 | REG_ILINK2 => {
            let u = d.flag(Flag::U);
            d.fault_if(u, Exception::PrivilegeViolation { address: d.pc() })?;
            Ok(d.reg(r))
        }
        REG_LIMM => Ok(d.k(0)),
        REG_PCL => {
            let pcl = d.pc() & !3;
            Ok(d.k(pcl))
        }
        _ => Err(Exception::InstructionError { address: d.pc() }),
    }
}

/// Write core register `r` as a destination. ILINK1/2 fault in user mode,
/// r61 and r63 are not writable, and a write to r62 is discarded.
#[inline(always)]
pub fn write_core_reg<D: Dom>(d: &mut D, r: u8, v: D::V) -> Result<(), Exception> {
    match r {
        0..=28 | 31..=60 => {
            d.set_reg(r, v);
            Ok(())
        }
        REG_ILINK1 | REG_ILINK2 => {
            let u = d.flag(Flag::U);
            d.fault_if(u, Exception::PrivilegeViolation { address: d.pc() })?;
            d.set_reg(r, v);
            Ok(())
        }
        REG_LIMM => Ok(()),
        _ => Err(Exception::InstructionError { address: d.pc() }),
    }
}

/// STATUS32 assembled from the flags.
#[inline]
pub fn status32<D: Dom>(d: &mut D) -> D::V {
    let mut acc = d.k(0);
    for f in Flag::ALL {
        let b = d.flag(f);
        let v = d.b2v(b);
        let n = d.k(f.status32_bit());
        let bit = d.shl(v, n);
        acc = d.or(acc, bit);
    }
    acc
}

/// Load every flag from the corresponding STATUS32 bit.
#[inline]
pub fn set_status32<D: Dom>(d: &mut D, v: D::V) {
    for f in Flag::ALL {
        let b = d.bit(v, f.status32_bit() as u8);
        d.set_flag(f, b);
    }
}

/// Whether condition `cc` holds on the current flags (ARCompact Table 50).
#[inline(always)]
pub fn cond_holds<D: Dom>(d: &mut D, cc: ConditionCode) -> D::B {
    let z = d.flag(Flag::Z);
    let n = d.flag(Flag::N);
    let c = d.flag(Flag::C);
    let v = d.flag(Flag::V);
    match cc {
        ConditionCode::AL => d.kb(true),
        ConditionCode::EQ => z,
        ConditionCode::NE => d.bnot(z),
        ConditionCode::PL => d.bnot(n),
        ConditionCode::MI => n,
        ConditionCode::CS => c,
        ConditionCode::CC => d.bnot(c),
        ConditionCode::VS => v,
        ConditionCode::VC => d.bnot(v),
        // GT: N == V and !Z
        ConditionCode::GT => {
            let nv = d.bxor(n, v);
            let same = d.bnot(nv);
            let nz = d.bnot(z);
            d.band(same, nz)
        }
        ConditionCode::GE => {
            let nv = d.bxor(n, v);
            d.bnot(nv)
        }
        ConditionCode::LT => d.bxor(n, v),
        ConditionCode::LE => {
            let nv = d.bxor(n, v);
            d.bor(z, nv)
        }
        // HI: !C and !Z
        ConditionCode::HI => {
            let nc = d.bnot(c);
            let nz = d.bnot(z);
            d.band(nc, nz)
        }
        ConditionCode::LS => d.bor(c, z),
        // PNZ: !N and !Z
        ConditionCode::PNZ => {
            let nn = d.bnot(n);
            let nz = d.bnot(z);
            d.band(nn, nz)
        }
    }
}

/// Value of a decoded operand: a register, an immediate, or nothing (0).
#[inline(always)]
pub fn operand<D: Dom>(d: &mut D, op: Operand) -> Result<D::V, Exception> {
    match op {
        Operand::Reg(r) => read_core_reg(d, r),
        Operand::Imm(v) => Ok(d.k(v)),
        Operand::None => Ok(d.k(0)),
    }
}

/// Write a result to a destination operand; anything but a register discards it.
#[inline(always)]
pub fn write_dest<D: Dom>(d: &mut D, op: Operand, v: D::V) -> Result<(), Exception> {
    match op {
        Operand::Reg(r) => write_core_reg(d, r, v),
        _ => Ok(()),
    }
}
