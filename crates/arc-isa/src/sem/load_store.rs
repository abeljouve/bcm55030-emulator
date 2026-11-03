//! Loads and stores: effective address, address write-back, sign extension.

use crate::decoder::instruction::{DataSize, Operand, WritebackMode};
use crate::dom::Dom;
use crate::exception::Exception;

use super::regs::{operand, write_core_reg, write_dest};

/// The address the instruction asks for. `.aw`/plain add the offset, `.ab`
/// uses the base as is (the sum is written back after), `.as` scales the
/// offset by the access size.
#[inline(always)]
fn effective_address<D: Dom>(d: &mut D, base: D::V, off: D::V, size: DataSize, wb: WritebackMode)
    -> D::V {
    match wb {
        WritebackMode::None | WritebackMode::PreWrite => d.add(base, off),
        WritebackMode::PostWrite => base,
        WritebackMode::Scaled => {
            let s = d.k(match size {
                DataSize::Word => 2,
                DataSize::HalfWord => 1,
                DataSize::Byte => 0,
            });
            let scaled = d.shl(off, s);
            d.add(base, scaled)
        }
    }
}

/// The address the core drives: the requested one with the low bits of the
/// access size cleared. A misaligned access is not a fault on this core; it
/// lands on the aligned word or half-word (observed on silicon).
#[inline(always)]
fn core_aligned<D: Dom>(d: &mut D, ea: D::V, size: DataSize) -> D::V {
    let m = d.k(match size {
        DataSize::Word => !3,
        DataSize::HalfWord => !1,
        DataSize::Byte => !0,
    });
    d.and(ea, m)
}

#[inline(always)]
fn write_back<D: Dom>(d: &mut D, base: Operand, bv: D::V, ov: D::V, wb: WritebackMode)
    -> Result<(), Exception> {
    if matches!(wb, WritebackMode::PreWrite | WritebackMode::PostWrite) {
        if let Operand::Reg(r) = base {
            let sum = d.add(bv, ov);
            write_core_reg(d, r, sum)?;
        }
    }
    Ok(())
}

#[inline]
#[allow(clippy::too_many_arguments)]
pub fn load<D: Dom>(
    d: &mut D,
    dst: Operand,
    base: Operand,
    offset: Operand,
    size: DataSize,
    sign_extend: bool,
    wb: WritebackMode,
    di: bool,
) -> Result<(), Exception> {
    // A load into an extension register (r32-r59) or LP_COUNT faults.
    if let Operand::Reg(r) = dst {
        if (32..=60).contains(&r) {
            return Err(Exception::InstructionError { address: d.pc() });
        }
    }
    let bv = operand(d, base)?;
    let ov = operand(d, offset)?;
    let ea = effective_address(d, bv, ov, size, wb);
    let aligned = core_aligned(d, ea, size);
    let v = d.load(aligned, ea, size, di)?;
    let v = match (size, sign_extend) {
        (DataSize::Byte, true) => d.sext(v, 8),
        (DataSize::HalfWord, true) => d.sext(v, 16),
        _ => v,
    };
    write_dest(d, dst, v)?;
    write_back(d, base, bv, ov, wb)
}

#[inline]
pub fn store<D: Dom>(
    d: &mut D,
    src: Operand,
    base: Operand,
    offset: Operand,
    size: DataSize,
    wb: WritebackMode,
    di: bool,
) -> Result<(), Exception> {
    let bv = operand(d, base)?;
    let ov = operand(d, offset)?;
    let sv = operand(d, src)?;
    let ea = effective_address(d, bv, ov, size, wb);
    let aligned = core_aligned(d, ea, size);
    d.store(aligned, ea, size, sv, di)?;
    write_back(d, base, bv, ov, wb)
}
