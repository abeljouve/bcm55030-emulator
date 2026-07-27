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
            cache_bypass,
        } => {
            load_store::execute_load(
                *dst,
                *base,
                *offset,
                *data_size,
                *do_sign_ext,
                *writeback,
                *cache_bypass,
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
            cache_bypass,
        } => {
            load_store::execute_store(
                *src, *base, *offset, *data_size, *writeback, *cache_bypass, state, mem,
            )?;
        }
        Instruction::Loop { offset, cc } => {
            special::execute_loop(*offset, *cc, decoded, state)?;
        }
        Instruction::LoadAux { dst, addr } => {
            let addr_val = resolve_value(*addr, state)?;
            // Intercept D-cache debug registers: DC_TAG (0x59) and DC_DATA (0x5B).
            // These return cache state based on DC_RAM_ADDR (0x58) previously set.
            match addr_val {
                0x59 => {
                    let val = mem.dcache_read_tag();
                    write_dest(*dst, val, state)?;
                }
                0x5B => {
                    let val = mem.dcache_read_data();
                    write_dest(*dst, val, state)?;
                }
                _ => {
                    load_store::execute_load_aux(*dst, *addr, state)?;
                }
            }
        }
        Instruction::StoreAux { src, addr } => {
            let addr_val = resolve_value(*addr, state)?;
            load_store::execute_store_aux(*src, *addr, state)?;
            // Post-process D-cache control writes
            match addr_val {
                0x47 => {
                    // DC_IVDC: invalidate entire D-cache
                    let val = resolve_value(*src, state)?;
                    if val & 1 != 0 {
                        mem.dcache_invalidate_all()?;
                    }
                }
                0x48 => {
                    // DC_CTRL: sync cache enable/IM/LM state
                    let val = state.read_aux_reg(0x48)?;
                    mem.dcache_sync_ctrl(val);
                }
                0x4A => {
                    // DC_IVDL: invalidate single cache line
                    let val = resolve_value(*src, state)?;
                    mem.dcache_invalidate_line(val)?;
                }
                // 0x4B (DC_FLSH): no handler needed — DC_FLSH is a no-op on
                // real BCM55030 (verified against real hardware). write_aux_reg
                // silently absorbs the write via the default fallthrough.
                0x58 => {
                    // DC_RAM_ADDR: set probe address for DC_TAG/DC_DATA reads
                    let val = resolve_value(*src, state)?;
                    mem.dcache_set_ram_addr(val);
                }
                0x10 => {
                    // IC_IVIC: invalidate entire I-cache on any write.
                    // Firmware may write 0 as the flush trigger, so gating on a
                    // bit value would miss the real flush path — any write flushes.
                    let _ = resolve_value(*src, state)?;
                    mem.icache_invalidate_all();
                }
                0x19 => {
                    // IC_IVIL: single-line I-cache invalidate. NO-OP on
                    // BCM55030 — verified against real hardware (DATASHEET §5.2).
                    // Only IC_IVIC (0x10) actually flushes. The address operand
                    // is still resolved/consumed so a side-effecting source
                    // register is read as on silicon.
                    let _ = resolve_value(*src, state)?;
                }
                _ => {}
            }
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
