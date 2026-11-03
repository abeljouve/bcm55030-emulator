pub mod emu_dom;

/// Frozen reference implementation for the differential tests.
#[cfg(test)]
pub(crate) mod legacy;
#[cfg(test)]
mod differential;

use crate::cpu::exception::Exception;
use crate::cpu::registers::CpuState;
use crate::decoder::instruction::*;
use crate::memory::Memory;

pub fn resolve_value(op: Operand, state: &CpuState) -> Result<u32, Exception> {
    match op {
        Operand::Reg(r) => state.read_core_reg(r),
        Operand::Imm(v) => Ok(v),
        Operand::None => Ok(0),
    }
}

pub fn write_dest(op: Operand, value: u32, state: &mut CpuState) -> Result<(), Exception> {
    match op {
        Operand::Reg(r) => state.write_core_reg(r, value),
        _ => Ok(()),
    }
}

pub fn execute(
    decoded: &DecodedInstruction,
    state: &mut CpuState,
    mem: &mut Memory,
) -> Result<(), Exception> {
    // One out-of-line function per instruction class, as before the semantics
    // went generic: inlined into one dispatcher they made it seven times
    // larger and measurably slower.
    match &decoded.inst {
        Instruction::Alu { op, dst, src1, src2, set_flags, cc } => {
            run::alu(state, mem, *op, *dst, *src1, *src2, *set_flags, *cc)
        }
        Instruction::SingleOp { op, dst, src, set_flags, cc } => {
            run::single_op(state, mem, *op, *dst, *src, *set_flags, *cc)
        }
        Instruction::ZeroOp(zop) => run::zero_op(state, mem, zop),
        Instruction::Branch { offset, cc, delay, link } => {
            run::branch(state, mem, *offset, *cc, *delay, *link, decoded.total_size())
        }
        Instruction::BranchCompare { kind, src1, src2, offset, delay } => {
            run::branch_compare(state, mem, *kind, *src1, *src2, *offset, *delay)
        }
        Instruction::Jump { target, cc, delay, link, flag_restore } => run::jump(
            state, mem, *target, *cc, *delay, *link, *flag_restore, decoded.total_size(),
        ),
        Instruction::Load { dst, base, offset, data_size, sign_extend, writeback, cache_bypass } => {
            run::load(state, mem, *dst, *base, *offset, *data_size, *sign_extend, *writeback,
                      *cache_bypass)
        }
        Instruction::Store { src, base, offset, data_size, writeback, cache_bypass } => {
            run::store(state, mem, *src, *base, *offset, *data_size, *writeback, *cache_bypass)
        }
        Instruction::Loop { offset, cc } => run::lp(state, mem, *offset, *cc, decoded.total_size()),
        Instruction::LoadAux { dst, addr } => run::lr(state, mem, *dst, *addr),
        Instruction::StoreAux { src, addr } => run::sr(state, mem, *src, *addr),
        Instruction::Flag { src, cc } => run::flag(state, mem, *src, *cc),
        Instruction::ExtArith { op, dst, src1, src2, set_flags, cc } => {
            run::ext_arith(state, mem, *op, *dst, *src1, *src2, *set_flags, *cc)
        }
        Instruction::Prefetch => Ok(()),
    }
}

/// The generic semantics instantiated on `EmuDom`, one function per class.
#[allow(clippy::too_many_arguments)]
mod run {
    use arc_isa::condition::ConditionCode;
    use arc_isa::sem;

    use super::emu_dom::EmuDom;
    use crate::cpu::exception::Exception;
    use crate::cpu::registers::CpuState;
    use crate::decoder::instruction::*;
    use crate::memory::Memory;

    type R = Result<(), Exception>;

    #[inline(never)]
    pub fn alu(s: &mut CpuState, m: &mut Memory, op: AluOp, dst: Operand, a: Operand, b: Operand,
               f: bool, cc: Option<ConditionCode>) -> R {
        sem::alu::alu(&mut EmuDom::new(s, m), op, dst, a, b, f, cc)
    }
    #[inline(never)]
    pub fn single_op(s: &mut CpuState, m: &mut Memory, op: SingleOp, dst: Operand, src: Operand,
                     f: bool, cc: Option<ConditionCode>) -> R {
        sem::single_op::single_op(&mut EmuDom::new(s, m), op, dst, src, f, cc)
    }
    #[inline(never)]
    pub fn ext_arith(s: &mut CpuState, m: &mut Memory, op: ExtArithOp, dst: Operand, a: Operand,
                     b: Operand, f: bool, cc: Option<ConditionCode>) -> R {
        sem::extended::ext_arith(&mut EmuDom::new(s, m), op, dst, a, b, f, cc)
    }
    #[inline(never)]
    pub fn zero_op(s: &mut CpuState, m: &mut Memory, zop: &ZeroOp) -> R {
        sem::special::zero_op(&mut EmuDom::new(s, m), zop)
    }
    #[inline(never)]
    pub fn branch(s: &mut CpuState, m: &mut Memory, offset: i32, cc: Option<ConditionCode>,
                  delay: DelayMode, link: bool, size: u32) -> R {
        sem::branch::branch(&mut EmuDom::new(s, m), offset, cc, delay, link, size)
    }
    #[inline(never)]
    pub fn branch_compare(s: &mut CpuState, m: &mut Memory, kind: BrCompareKind, a: Operand,
                          b: Operand, offset: i32, delay: DelayMode) -> R {
        sem::branch::branch_compare(&mut EmuDom::new(s, m), kind, a, b, offset, delay)
    }
    #[inline(never)]
    pub fn jump(s: &mut CpuState, m: &mut Memory, target: Operand, cc: Option<ConditionCode>,
                delay: DelayMode, link: bool, flag_restore: bool, size: u32) -> R {
        sem::branch::jump(&mut EmuDom::new(s, m), target, cc, delay, link, flag_restore, size)
    }
    #[inline(never)]
    pub fn load(s: &mut CpuState, m: &mut Memory, dst: Operand, base: Operand, off: Operand,
                size: DataSize, sx: bool, wb: WritebackMode, di: bool) -> R {
        sem::load_store::load(&mut EmuDom::new(s, m), dst, base, off, size, sx, wb, di)
    }
    #[inline(never)]
    pub fn store(s: &mut CpuState, m: &mut Memory, src: Operand, base: Operand, off: Operand,
                 size: DataSize, wb: WritebackMode, di: bool) -> R {
        sem::load_store::store(&mut EmuDom::new(s, m), src, base, off, size, wb, di)
    }
    #[inline(never)]
    pub fn lp(s: &mut CpuState, m: &mut Memory, offset: u32, cc: Option<ConditionCode>,
              size: u32) -> R {
        sem::special::lp(&mut EmuDom::new(s, m), offset, cc, size)
    }
    #[inline(never)]
    pub fn lr(s: &mut CpuState, m: &mut Memory, dst: Operand, addr: Operand) -> R {
        sem::aux::lr(&mut EmuDom::new(s, m), dst, addr)
    }
    #[inline(never)]
    pub fn sr(s: &mut CpuState, m: &mut Memory, src: Operand, addr: Operand) -> R {
        sem::aux::sr(&mut EmuDom::new(s, m), src, addr)
    }
    #[inline(never)]
    pub fn flag(s: &mut CpuState, m: &mut Memory, src: Operand, cc: Option<ConditionCode>) -> R {
        sem::special::flag(&mut EmuDom::new(s, m), src, cc)
    }
}
