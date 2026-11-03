//! `StateDom`: the instruction semantics over a `CpuState` alone, for the
//! register-level API (`CpuState::read_aux_reg` and friends). There is no
//! memory behind it: a load or store is a bus error.

use arc_isa::decoder::instruction::{DataSize, DelayMode};
use arc_isa::dom::{AuxReg, Dom, Flag};

use crate::cpu::exception::Exception;
use crate::cpu::registers::{CpuState, AUX_DEBUG};

pub struct StateDom<'a>(pub &'a mut CpuState);

impl<'a> Dom for StateDom<'a> {
    arc_isa::concrete_primitives!();

    #[inline(always)]
    fn pc(&self) -> u32 {
        self.0.pc
    }
    #[inline(always)]
    fn reg(&mut self, r: u8) -> u32 {
        self.0.core_regs[r as usize]
    }
    #[inline(always)]
    fn set_reg(&mut self, r: u8, v: u32) {
        self.0.core_regs[r as usize] = v;
    }
    #[inline(always)]
    fn flag(&mut self, f: Flag) -> bool {
        *self.0.flag_mut(f)
    }
    #[inline(always)]
    fn set_flag(&mut self, f: Flag, b: bool) {
        *self.0.flag_mut(f) = b;
    }
    fn load(&mut self, _ea: u32, raw: u32, _size: DataSize, _di: bool) -> Result<u32, Exception> {
        Err(Exception::MemoryError { address: raw, is_write: false })
    }
    fn store(&mut self, _ea: u32, raw: u32, _size: DataSize, _v: u32, _di: bool)
        -> Result<(), Exception> {
        Err(Exception::MemoryError { address: raw, is_write: true })
    }
    #[inline(always)]
    fn aux(&mut self, a: AuxReg) -> u32 {
        *self.0.aux_mut(a)
    }
    #[inline(always)]
    fn set_aux(&mut self, a: AuxReg, v: u32) {
        *self.0.aux_mut(a) = v;
    }
    #[inline]
    fn aux_read_hook(&mut self, addr: u32) -> Option<Result<u32, Exception>> {
        (addr == AUX_DEBUG).then(|| Ok(self.0.debug_aux()))
    }
    #[inline]
    fn aux_write_hook(&mut self, _addr: u32, _v: u32) -> Result<(), Exception> {
        Ok(())
    }
    #[inline(always)]
    fn branch(&mut self, target: u32, delay: DelayMode, link: Option<u32>)
        -> Result<(), Exception> {
        self.0.take_branch(target, delay, link);
        Ok(())
    }
    #[inline(always)]
    fn lp_skip(&mut self, target: u32) {
        self.0.pc = target;
        self.0.pc_written = true;
    }
    #[inline(always)]
    fn halt(&mut self) {
        self.0.halted = true;
    }
    #[inline(always)]
    fn sleep(&mut self) {
        self.0.sleeping = true;
    }
}
