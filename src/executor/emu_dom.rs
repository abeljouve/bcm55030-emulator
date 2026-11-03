//! The emulator's concrete domain: the instruction semantics of `arc_isa` run
//! on the live `CpuState` and `Memory`, with the platform (D-cache probes and
//! control, I-cache invalidation, misaligned-access log, MMIO) behind the
//! memory and AUX hooks.

use arc_isa::decoder::instruction::{DataSize, DelayMode};
use arc_isa::dom::{AuxReg, Dom, Flag};

use crate::cpu::exception::Exception;
use crate::cpu::registers::{CpuState, AUX_DEBUG};
use crate::memory::Memory;

pub struct EmuDom<'a> {
    pub state: &'a mut CpuState,
    pub mem: &'a mut Memory,
}

impl<'a> EmuDom<'a> {
    #[inline(always)]
    pub fn new(state: &'a mut CpuState, mem: &'a mut Memory) -> Self {
        Self { state, mem }
    }
}

impl<'a> Dom for EmuDom<'a> {
    arc_isa::concrete_primitives!();

    #[inline(always)]
    fn pc(&self) -> u32 {
        self.state.pc
    }
    #[inline(always)]
    fn reg(&mut self, r: u8) -> u32 {
        self.state.core_regs[r as usize]
    }
    #[inline(always)]
    fn set_reg(&mut self, r: u8, v: u32) {
        self.state.core_regs[r as usize] = v;
    }
    #[inline(always)]
    fn flag(&mut self, f: Flag) -> bool {
        *self.state.flag_mut(f)
    }
    #[inline(always)]
    fn set_flag(&mut self, f: Flag, b: bool) {
        *self.state.flag_mut(f) = b;
    }

    /// `Memory` aligns and logs misaligned accesses itself, so it gets the
    /// raw address; masking twice is harmless.
    #[inline(always)]
    fn load(&mut self, _ea: u32, raw: u32, size: DataSize, di: bool) -> Result<u32, Exception> {
        Ok(match size {
            DataSize::Word => self.mem.read_word_data(raw, di)?,
            DataSize::HalfWord => self.mem.read_half_data(raw, di)? as u32,
            DataSize::Byte => self.mem.read_byte_data(raw, di)? as u32,
        })
    }
    #[inline(always)]
    fn store(&mut self, _ea: u32, raw: u32, size: DataSize, v: u32, di: bool)
        -> Result<(), Exception> {
        match size {
            DataSize::Word => self.mem.write_word_data(raw, v, di),
            DataSize::HalfWord => self.mem.write_half_data(raw, v as u16, di),
            DataSize::Byte => self.mem.write_byte_data(raw, v as u8, di),
        }
    }

    #[inline(always)]
    fn aux(&mut self, a: AuxReg) -> u32 {
        *self.state.aux_mut(a)
    }
    #[inline(always)]
    fn set_aux(&mut self, a: AuxReg, v: u32) {
        *self.state.aux_mut(a) = v;
    }

    /// D-cache probe registers DC_TAG (0x59) and DC_DATA (0x5B) answer from the
    /// cache model, for the address set in DC_RAM_ADDR (0x58); DEBUG carries
    /// the timing model's load-pending bit.
    #[inline]
    fn aux_read_hook(&mut self, addr: u32) -> Option<Result<u32, Exception>> {
        match addr {
            AUX_DEBUG => Some(Ok(self.state.debug_aux())),
            0x59 => Some(Ok(self.mem.dcache_read_tag())),
            0x5B => Some(Ok(self.mem.dcache_read_data())),
            _ => None,
        }
    }

    /// Cache side effects of `sr`, after the architectural write.
    #[inline]
    fn aux_write_hook(&mut self, addr: u32, v: u32) -> Result<(), Exception> {
        match addr {
            // DC_IVDC: invalidate the whole D-cache.
            0x47 => {
                if v & 1 != 0 {
                    self.mem.dcache_invalidate_all()?;
                }
            }
            // DC_CTRL: the cache follows the masked shadow.
            0x48 => self.mem.dcache_sync_ctrl(self.state.aux_dc_ctrl),
            // DC_IVDL: invalidate one line.
            0x4A => self.mem.dcache_invalidate_line(v)?,
            // DC_RAM_ADDR: probe address for DC_TAG / DC_DATA.
            0x58 => self.mem.dcache_set_ram_addr(v),
            // IC_IVIC: any write flushes the I-cache. IC_IVIL (0x19) and
            // DC_FLSH (0x4B) are no-ops on this core.
            0x10 => self.mem.icache_invalidate_all(),
            _ => {}
        }
        Ok(())
    }

    #[inline(always)]
    fn branch(&mut self, target: u32, delay: DelayMode, link: Option<u32>)
        -> Result<(), Exception> {
        self.state.take_branch(target, delay, link);
        Ok(())
    }
    #[inline(always)]
    fn lp_skip(&mut self, target: u32) {
        self.state.pc = target;
        self.state.pc_written = true;
    }
    #[inline(always)]
    fn halt(&mut self) {
        self.state.halted = true;
    }
    #[inline(always)]
    fn sleep(&mut self) {
        self.state.sleeping = true;
    }
    #[inline(always)]
    fn take_delay(&mut self) -> Option<(u32, bool)> {
        self.state.take_delay()
    }
    #[inline(always)]
    fn delay_pending(&mut self) -> bool {
        matches!(self.state.delay_state, crate::cpu::registers::DelayState::DelaySlot { .. })
    }
    #[inline(always)]
    fn begin_insn(&mut self) {
        self.state.pc_written = false;
        self.state.link_executed = false;
    }
    #[inline(always)]
    fn pc_written(&mut self) -> bool {
        self.state.pc_written
    }
    #[inline(always)]
    fn set_pc(&mut self, pc: u32) {
        self.state.pc = pc;
    }
}
