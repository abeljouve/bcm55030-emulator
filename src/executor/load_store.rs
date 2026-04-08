use crate::cpu::exception::Exception;
use crate::cpu::registers::CpuState;
use crate::decoder::fields;
use crate::decoder::instruction::*;
use crate::memory::Memory;

use super::{resolve_value, write_dest};

pub fn execute_load(
    dst: Operand,
    base: Operand,
    offset: Operand,
    data_size: DataSize,
    do_sign_ext: bool,
    writeback: WritebackMode,
    state: &mut CpuState,
    mem: &Memory,
) -> Result<(), Exception> {
    // ARC 700: LD to extension regs (r32-r59) or LP_COUNT (r60) raises InstructionError
    if let Operand::Reg(r) = dst {
        if r >= 32 && r <= 60 {
            return Err(Exception::InstructionError { address: state.pc });
        }
    }

    let base_val = resolve_value(base, state)?;
    let offset_val = resolve_value(offset, state)?;

    let ea = compute_ea(base_val, offset_val, data_size, writeback);

    // ARC 700 Harvard architecture: PC-relative loads access the Code Space
    // (ICCM), not the Data Space (DCCM). From the ISA manual:
    //   "Code Space: accessible via instruction fetch and PC relative ops"
    //   "Data Space: accessible using load (LD) and store (ST) operations"
    // When base register is PCL (r63), use instruction fetch path (ICCM).
    let is_pcl_relative = matches!(base, Operand::Reg(63));

    let value = match data_size {
        DataSize::Word => {
            if is_pcl_relative {
                mem.fetch_word(ea)?
            } else {
                mem.read_word(ea)?
            }
        }
        DataSize::Byte => {
            let b = if is_pcl_relative {
                // fetch_half gets 2 bytes from ICCM; extract the target byte
                let h = mem.fetch_half(ea & !1)?;
                if ea & 1 == 0 { (h >> 8) as u8 } else { h as u8 }
            } else {
                mem.read_byte(ea)?
            } as u32;
            if do_sign_ext {
                fields::sign_extend(b, 8) as u32
            } else {
                b
            }
        }
        DataSize::HalfWord => {
            let h = if is_pcl_relative {
                mem.fetch_half(ea)?
            } else {
                mem.read_half(ea)?
            } as u32;
            if do_sign_ext {
                fields::sign_extend(h, 16) as u32
            } else {
                h
            }
        }
    };

    write_dest(dst, value, state)?;

    if matches!(writeback, WritebackMode::PreWrite | WritebackMode::PostWrite) {
        let wb_val = base_val.wrapping_add(offset_val);
        if let Operand::Reg(r) = base {
            state.write_core_reg(r, wb_val)?;
        }
    }

    Ok(())
}

pub fn execute_store(
    src: Operand,
    base: Operand,
    offset: Operand,
    data_size: DataSize,
    writeback: WritebackMode,
    state: &mut CpuState,
    mem: &mut Memory,
) -> Result<(), Exception> {
    let base_val = resolve_value(base, state)?;
    let offset_val = resolve_value(offset, state)?;
    let src_val = resolve_value(src, state)?;

    let ea = compute_ea(base_val, offset_val, data_size, writeback);

    match data_size {
        DataSize::Word => mem.write_word(ea, src_val)?,
        DataSize::Byte => mem.write_byte(ea, src_val as u8)?,
        DataSize::HalfWord => mem.write_half(ea, src_val as u16)?,
    }

    if matches!(writeback, WritebackMode::PreWrite | WritebackMode::PostWrite) {
        let wb_val = base_val.wrapping_add(offset_val);
        if let Operand::Reg(r) = base {
            state.write_core_reg(r, wb_val)?;
        }
    }

    Ok(())
}

fn compute_ea(base: u32, offset: u32, data_size: DataSize, writeback: WritebackMode) -> u32 {
    match writeback {
        WritebackMode::None | WritebackMode::PreWrite => base.wrapping_add(offset),
        WritebackMode::PostWrite => base,
        WritebackMode::Scaled => {
            let scale = match data_size {
                DataSize::Word => 4u32,
                DataSize::HalfWord => 2,
                DataSize::Byte => 1,
            };
            base.wrapping_add(offset.wrapping_mul(scale))
        }
    }
}

pub fn execute_load_aux(
    dst: Operand,
    addr: Operand,
    state: &mut CpuState,
) -> Result<(), Exception> {
    let addr_val = resolve_value(addr, state)?;
    let val = state.read_aux_reg(addr_val)?;
    write_dest(dst, val, state)?;
    Ok(())
}

pub fn execute_store_aux(
    src: Operand,
    addr: Operand,
    state: &mut CpuState,
) -> Result<(), Exception> {
    let addr_val = resolve_value(addr, state)?;
    let val = resolve_value(src, state)?;
    state.write_aux_reg(addr_val, val)?;
    Ok(())
}
