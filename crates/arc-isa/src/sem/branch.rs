//! Branches, compare-and-branch and jumps. Offsets are relative to PCL, the
//! word-aligned address of the branch.

use crate::condition::ConditionCode;
use crate::decoder::instruction::{BrCompareKind, DelayMode, Operand};
use crate::dom::{AuxReg, Dom};
use crate::exception::Exception;
use crate::regs::REG_ILINK1;

use super::regs::{cond_holds, operand, set_status32};

#[inline(always)]
fn pcl_target<D: Dom>(d: &D, offset: i32) -> u32 {
    (d.pc() & !3).wrapping_add(offset as u32)
}

/// Run `body` under an optional condition code.
#[inline(always)]
fn under<D: Dom>(
    d: &mut D,
    cc: Option<ConditionCode>,
    body: impl FnOnce(&mut D) -> Result<(), Exception>,
) -> Result<(), Exception> {
    match cc {
        None => body(d),
        Some(cc) => {
            let g = cond_holds(d, cc);
            d.when(g, body)
        }
    }
}

/// `B`, `Bcc`, `BL`, `BLcc`, with or without delay slot. `size` is the
/// branch's own length, for the return address.
#[inline]
pub fn branch<D: Dom>(
    d: &mut D,
    offset: i32,
    cc: Option<ConditionCode>,
    delay: DelayMode,
    link: bool,
    size: u32,
) -> Result<(), Exception> {
    under(d, cc, |d| {
        let target = pcl_target(d, offset);
        let ret = d.pc().wrapping_add(size);
        let t = d.k(target);
        d.set_aux(AuxReg::Bta, t);
        d.branch(t, delay, link.then_some(ret))
    })
}

/// `BRcc` and `BBIT0/1`: compare two operands and branch if the relation holds.
#[inline]
pub fn branch_compare<D: Dom>(
    d: &mut D,
    kind: BrCompareKind,
    src1: Operand,
    src2: Operand,
    offset: i32,
    delay: DelayMode,
) -> Result<(), Exception> {
    let a = operand(d, src1)?;
    let b = operand(d, src2)?;
    let taken = match kind {
        BrCompareKind::Breq => d.eq(a, b),
        BrCompareKind::Brne => {
            let e = d.eq(a, b);
            d.bnot(e)
        }
        BrCompareKind::Brlt => d.slt(a, b),
        BrCompareKind::Brge => {
            let lt = d.slt(a, b);
            d.bnot(lt)
        }
        BrCompareKind::Brlo => d.ult(a, b),
        BrCompareKind::Brhs => {
            let lo = d.ult(a, b);
            d.bnot(lo)
        }
        BrCompareKind::Bbit0 | BrCompareKind::Bbit1 => {
            let m31 = d.k(31);
            let n = d.and(b, m31);
            let s = d.lshr(a, n);
            let bit = d.bit(s, 0);
            if kind == BrCompareKind::Bbit1 { bit } else { d.bnot(bit) }
        }
    };
    d.when(taken, |d| {
        let target = pcl_target(d, offset);
        let t = d.k(target);
        d.set_aux(AuxReg::Bta, t);
        d.branch(t, delay, None)
    })
}

/// `J`, `Jcc`, `JL`, `JLcc`, and `J.F [ilinkN]`, which also restores STATUS32
/// and BTA from the level-1 (ILINK1) or level-2 (any other register) copies.
#[inline]
pub fn jump<D: Dom>(
    d: &mut D,
    target: Operand,
    cc: Option<ConditionCode>,
    delay: DelayMode,
    link: bool,
    flag_restore: bool,
    size: u32,
) -> Result<(), Exception> {
    under(d, cc, |d| {
        let t = operand(d, target)?;
        let ret = d.pc().wrapping_add(size);
        d.set_aux(AuxReg::Bta, t);
        if flag_restore {
            if let Operand::Reg(r) = target {
                let (st, bta) = if r == REG_ILINK1 {
                    (AuxReg::Status32L1, AuxReg::BtaL1)
                } else {
                    (AuxReg::Status32L2, AuxReg::BtaL2)
                };
                let s = d.aux(st);
                set_status32(d, s);
                let b = d.aux(bta);
                d.set_aux(AuxReg::Bta, b);
            }
        }
        d.branch(t, delay, link.then_some(ret))
    })
}
