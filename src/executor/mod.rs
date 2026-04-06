pub mod alu;
pub mod branch;
pub mod extended;
pub mod load_store;
pub mod multiply;
pub mod single_op;
pub mod special;

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
    match &decoded.inst {
        Instruction::Alu {
            op,
            dst,
            src1,
            src2,
            set_flags,
            cc,
        } => {
            alu::execute_alu(*op, *dst, *src1, *src2, *set_flags, *cc, state)?;
        }
        Instruction::SingleOp {
            op,
            dst,
            src,
            set_flags,
            cc,
        } => {
            single_op::execute_single_op(*op, *dst, *src, *set_flags, *cc, state, mem)?;
        }
        Instruction::ZeroOp(zop) => {
            special::execute_zero_op(zop, state)?;
        }
        Instruction::Branch {
            offset,
            cc,
            delay,
            link,
        } => {
            branch::execute_branch(decoded, *offset, *cc, *delay, *link, state)?;
        }
        Instruction::BranchCompare {
            kind,
            src1,
            src2,
            offset,
            delay,
        } => {
            branch::execute_branch_compare(
                *kind, *src1, *src2, *offset, *delay, decoded, state,
            )?;
        }
        Instruction::Jump {
            target,
            cc,
            delay,
            link,
            flag_restore,
        } => {
            branch::execute_jump(decoded, *target, *cc, *delay, *link, *flag_restore, state)?;
        }
        Instruction::Load {
            dst,
            base,
            offset,
            data_size,
            sign_extend: do_sign_ext,
            writeback,
            ..
        } => {
            load_store::execute_load(
                *dst,
                *base,
                *offset,
                *data_size,
                *do_sign_ext,
                *writeback,
                state,
                mem,
            )?;
        }
        Instruction::Store {
            src,
            base,
            offset,
            data_size,
            writeback,
            ..
        } => {
            load_store::execute_store(*src, *base, *offset, *data_size, *writeback, state, mem)?;
        }
        Instruction::Loop { offset, cc } => {
            special::execute_loop(*offset, *cc, decoded, state)?;
        }
        Instruction::LoadAux { dst, addr } => {
            load_store::execute_load_aux(*dst, *addr, state)?;
        }
        Instruction::StoreAux { src, addr } => {
            load_store::execute_store_aux(*src, *addr, state)?;
        }
        Instruction::Flag { src, cc } => {
            special::execute_flag(*src, *cc, state)?;
        }
        Instruction::ExtArith {
            op,
            dst,
            src1,
            src2,
            set_flags,
            cc,
        } => {
            extended::execute_ext_arith(*op, *dst, *src1, *src2, *set_flags, *cc, state)?;
        }
        Instruction::Prefetch => {}
    }
    Ok(())
}
