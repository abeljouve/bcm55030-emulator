//! One decoded instruction, dispatched to its semantics.

use crate::decoder::instruction::{DecodedInstruction, Instruction};
use crate::dom::Dom;
use crate::exception::Exception;

use super::{alu, aux, branch, extended, load_store, single_op, special};

#[inline]
pub fn execute<D: Dom>(d: &mut D, decoded: &DecodedInstruction) -> Result<(), Exception> {
    let size = decoded.total_size();
    match decoded.inst {
        Instruction::Alu { op, dst, src1, src2, set_flags, cc } => {
            alu::alu(d, op, dst, src1, src2, set_flags, cc)
        }
        Instruction::SingleOp { op, dst, src, set_flags, cc } => {
            single_op::single_op(d, op, dst, src, set_flags, cc)
        }
        Instruction::ZeroOp(ref zop) => special::zero_op(d, zop),
        Instruction::Branch { offset, cc, delay, link } => {
            branch::branch(d, offset, cc, delay, link, size)
        }
        Instruction::BranchCompare { kind, src1, src2, offset, delay } => {
            branch::branch_compare(d, kind, src1, src2, offset, delay)
        }
        Instruction::Jump { target, cc, delay, link, flag_restore } => {
            branch::jump(d, target, cc, delay, link, flag_restore, size)
        }
        Instruction::Load { dst, base, offset, data_size, sign_extend, writeback, cache_bypass } => {
            load_store::load(d, dst, base, offset, data_size, sign_extend, writeback, cache_bypass)
        }
        Instruction::Store { src, base, offset, data_size, writeback, cache_bypass } => {
            load_store::store(d, src, base, offset, data_size, writeback, cache_bypass)
        }
        Instruction::Loop { offset, cc } => special::lp(d, offset, cc, size),
        Instruction::LoadAux { dst, addr } => aux::lr(d, dst, addr),
        Instruction::StoreAux { src, addr } => aux::sr(d, src, addr),
        Instruction::Flag { src, cc } => special::flag(d, src, cc),
        Instruction::ExtArith { op, dst, src1, src2, set_flags, cc } => {
            extended::ext_arith(d, op, dst, src1, src2, set_flags, cc)
        }
        Instruction::Prefetch => Ok(()),
    }
}
