//! The emulator's concrete domain: the instruction semantics of `arc_isa` run
//! on the live `CpuState` and `Memory`, with the platform (D-cache probes and
//! control, I-cache invalidation, misaligned-access log, MMIO) behind the
//! memory and AUX hooks.

use arc_isa::decoder::instruction::{DataSize, DelayMode};
use arc_isa::dom::{AuxReg, Dom, Flag};

use crate::cpu::exception::Exception;
use crate::cpu::registers::{CpuState, DelayState, REG_BLINK};
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

    #[inline(always)]
    fn flag_mut(&mut self, f: Flag) -> &mut bool {
        let s = &mut *self.state;
        match f {
            Flag::Z => &mut s.flag_z,
            Flag::N => &mut s.flag_n,
            Flag::C => &mut s.flag_c,
            Flag::V => &mut s.flag_v,
            Flag::E1 => &mut s.flag_e1,
            Flag::E2 => &mut s.flag_e2,
            Flag::A1 => &mut s.flag_a1,
            Flag::A2 => &mut s.flag_a2,
            Flag::AE => &mut s.flag_ae,
            Flag::DE => &mut s.flag_de,
            Flag::U => &mut s.flag_u,
            Flag::L => &mut s.flag_l,
            Flag::H => &mut s.flag_h,
        }
    }

    #[inline(always)]
    fn aux_mut(&mut self, a: AuxReg) -> &mut u32 {
        let s = &mut *self.state;
        match a {
            AuxReg::Semaphore => &mut s.aux_semaphore,
            AuxReg::LpStart => &mut s.aux_lp_start,
            AuxReg::LpEnd => &mut s.aux_lp_end,
            AuxReg::Status32L1 => &mut s.aux_status32_l1,
            AuxReg::Status32L2 => &mut s.aux_status32_l2,
            AuxReg::Count0 => &mut s.aux_count0,
            AuxReg::Control0 => &mut s.aux_control0,
            AuxReg::Limit0 => &mut s.aux_limit0,
            AuxReg::IntVectorBase => &mut s.aux_int_vector_base,
            AuxReg::Mulhi => &mut s.aux_mulhi,
            AuxReg::Macmode => &mut s.aux_macmode,
            AuxReg::IrqLv12 => &mut s.aux_irq_lv12,
            AuxReg::Count1 => &mut s.aux_count1,
            AuxReg::Control1 => &mut s.aux_control1,
            AuxReg::Limit1 => &mut s.aux_limit1,
            AuxReg::IrqLev => &mut s.aux_irq_lev,
            AuxReg::IrqHint => &mut s.aux_irq_hint,
            AuxReg::Itrigger => &mut s.aux_itrigger,
            AuxReg::Xpu => &mut s.aux_xpu,
            AuxReg::IrqPulseCancel => &mut s.aux_irq_pulse_cancel,
            AuxReg::IrqPending => &mut s.aux_irq_pending,
            AuxReg::DcCtrl => &mut s.aux_dc_ctrl,
            AuxReg::Bta => &mut s.aux_bta,
            AuxReg::BtaL1 => &mut s.aux_bta_l1,
            AuxReg::BtaL2 => &mut s.aux_bta_l2,
        }
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
        *self.flag_mut(f)
    }
    #[inline(always)]
    fn set_flag(&mut self, f: Flag, b: bool) {
        *self.flag_mut(f) = b;
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
        *self.aux_mut(a)
    }
    #[inline(always)]
    fn set_aux(&mut self, a: AuxReg, v: u32) {
        *self.aux_mut(a) = v;
    }

    /// D-cache probe registers DC_TAG (0x59) and DC_DATA (0x5B) answer from the
    /// cache model, for the address set in DC_RAM_ADDR (0x58).
    #[inline]
    fn aux_read_hook(&mut self, addr: u32) -> Option<Result<u32, Exception>> {
        match addr {
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
        match delay {
            DelayMode::Delay => {
                self.state.delay_state = DelayState::DelaySlot { target, is_link: link.is_some() };
            }
            DelayMode::NoDelay => {
                if let Some(ret) = link {
                    self.state.core_regs[REG_BLINK as usize] = ret;
                    self.state.link_executed = true;
                }
                self.state.pc = target;
                self.state.pc_written = true;
            }
        }
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
}
