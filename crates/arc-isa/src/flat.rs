//! `FlatMachine`: a self-contained concrete domain. Architectural state, a
//! flat big-endian memory, no caches, no MMIO, no interrupts. It is the
//! reference the instruction set is checked on without an emulator around it.

use crate::decoder::instruction::{DataSize, DelayMode};
use crate::dom::{AuxReg, Dom, Flag};
use crate::exception::Exception;
use crate::regs::REG_BLINK;

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FlatMachine {
    pub regs: [u32; 64],
    pub flags: [bool; Flag::ALL.len()],
    pub aux: [u32; AuxReg::ALL.len()],
    /// Flat memory from address 0, big-endian.
    pub mem: Vec<u8>,
    pub pc: u32,
    /// The executed instruction wrote the PC (a taken branch without delay).
    pub pc_written: bool,
    /// A link branch without delay slot wrote BLINK.
    pub link_executed: bool,
    /// A delayed branch is pending: target, and whether it links.
    pub delay: Option<(u32, bool)>,
    pub halted: bool,
    pub sleeping: bool,
}

impl FlatMachine {
    pub fn new(mem_size: usize) -> Self {
        Self {
            regs: [0; 64],
            flags: [false; Flag::ALL.len()],
            aux: [0; AuxReg::ALL.len()],
            mem: vec![0; mem_size],
            pc: 0,
            pc_written: false,
            link_executed: false,
            delay: None,
            halted: false,
            sleeping: false,
        }
    }

    #[inline]
    fn flag_index(f: Flag) -> usize {
        Flag::ALL.iter().position(|&g| g == f).unwrap()
    }

    #[inline]
    fn aux_index(a: AuxReg) -> usize {
        AuxReg::ALL.iter().position(|&b| b == a).unwrap()
    }

    fn bytes(&self, addr: u32, n: usize, is_write: bool) -> Result<usize, Exception> {
        let a = addr as usize;
        if a.checked_add(n).is_none_or(|e| e > self.mem.len()) {
            return Err(Exception::MemoryError { address: addr, is_write });
        }
        Ok(a)
    }

    pub fn read(&self, addr: u32, size: DataSize) -> Result<u32, Exception> {
        let n = size_bytes(size);
        let a = self.bytes(addr, n, false)?;
        Ok(self.mem[a..a + n].iter().fold(0u32, |acc, &b| (acc << 8) | b as u32))
    }

    pub fn write(&mut self, addr: u32, size: DataSize, v: u32) -> Result<(), Exception> {
        let n = size_bytes(size);
        let a = self.bytes(addr, n, true)?;
        for i in 0..n {
            self.mem[a + i] = (v >> (8 * (n - 1 - i))) as u8;
        }
        Ok(())
    }
}

impl crate::decoder::InstructionFetch for FlatMachine {
    fn fetch_half(&self, addr: u32) -> Result<u16, Exception> {
        if addr & 1 != 0 {
            return Err(Exception::MisalignedAccess { address: addr });
        }
        Ok(self.read(addr, DataSize::HalfWord)? as u16)
    }
    fn fetch_word(&self, addr: u32) -> Result<u32, Exception> {
        if addr & 1 != 0 {
            return Err(Exception::MisalignedAccess { address: addr });
        }
        let hi = self.read(addr, DataSize::HalfWord)?;
        let lo = self.read(addr.wrapping_add(2), DataSize::HalfWord)?;
        Ok((hi << 16) | lo)
    }
}

impl FlatMachine {
    /// One architectural step: the zero-overhead loop and delay slot, decode,
    /// execute, the PC update, and the entry into a vectored exception (an
    /// Instruction Error, SWI or TRAP goes to slot 2; any other exception is
    /// returned, as is a fault on the vector itself). A halted or sleeping
    /// machine does nothing.
    pub fn step(&mut self) -> Result<(), Exception> {
        use crate::sem::step;
        if self.halted || self.sleeping {
            return Ok(());
        }
        let (delay, looped) = step::begin(self);
        if looped {
            return Ok(());
        }
        let pc = self.pc;
        let decoded = match crate::decoder::decode_with(pc, &*self) {
            Ok(d) => d,
            Err(e) => {
                // The major opcode still gives the length of an undecodable one.
                let size = match self.fetch_half_of(pc) {
                    Some(hw) if (hw >> 11) >= 0x0C => 2,
                    _ => 4,
                };
                let resume = delay.map_or(pc.wrapping_add(size), |(t, _)| t);
                return self.vector(e, resume);
            }
        };
        let next_pc = pc.wrapping_add(decoded.total_size());
        let resume = delay.map_or(next_pc, |(t, _)| t);
        if let Err(e) = step::link_slot(self, delay, next_pc) {
            return self.vector(e, resume);
        }
        self.begin_insn();
        if let Err(e) = crate::sem::execute(self, &decoded) {
            return self.vector(e, resume);
        }
        step::finish(self, delay, next_pc);
        Ok(())
    }

    fn fetch_half_of(&self, pc: u32) -> Option<u16> {
        self.read(pc, DataSize::HalfWord).ok().map(|v| v as u16)
    }

    fn vector(&mut self, e: Exception, resume: u32) -> Result<(), Exception> {
        if !e.is_vectored() {
            return Err(e);
        }
        let slot = e.vector_number() as u32;
        let target = self.aux(AuxReg::IntVectorBase).wrapping_add(slot * 8);
        if target == self.pc {
            return Err(e);
        }
        crate::sem::step::exception_entry(self, slot, resume);
        Ok(())
    }
}

fn size_bytes(size: DataSize) -> usize {
    match size {
        DataSize::Word => 4,
        DataSize::HalfWord => 2,
        DataSize::Byte => 1,
    }
}

impl Dom for FlatMachine {
    crate::concrete_primitives!();

    #[inline]
    fn pc(&self) -> u32 {
        self.pc
    }
    #[inline]
    fn reg(&mut self, r: u8) -> u32 {
        self.regs[r as usize]
    }
    #[inline]
    fn set_reg(&mut self, r: u8, v: u32) {
        self.regs[r as usize] = v;
    }
    #[inline]
    fn flag(&mut self, f: Flag) -> bool {
        self.flags[Self::flag_index(f)]
    }
    #[inline]
    fn set_flag(&mut self, f: Flag, b: bool) {
        self.flags[Self::flag_index(f)] = b;
    }

    fn load(&mut self, ea: u32, _raw: u32, size: DataSize, _di: bool) -> Result<u32, Exception> {
        self.read(ea, size)
    }
    fn store(&mut self, ea: u32, _raw: u32, size: DataSize, v: u32, _di: bool)
        -> Result<(), Exception> {
        self.write(ea, size, v)
    }
    #[inline]
    fn aux(&mut self, a: AuxReg) -> u32 {
        self.aux[Self::aux_index(a)]
    }
    #[inline]
    fn set_aux(&mut self, a: AuxReg, v: u32) {
        self.aux[Self::aux_index(a)] = v;
    }
    fn aux_read_hook(&mut self, _addr: u32) -> Option<Result<u32, Exception>> {
        None
    }
    fn aux_write_hook(&mut self, _addr: u32, _v: u32) -> Result<(), Exception> {
        Ok(())
    }

    fn branch(&mut self, target: u32, delay: DelayMode, link: Option<u32>)
        -> Result<(), Exception> {
        match delay {
            DelayMode::Delay => self.delay = Some((target, link.is_some())),
            DelayMode::NoDelay => {
                if let Some(ret) = link {
                    self.regs[REG_BLINK as usize] = ret;
                    self.link_executed = true;
                }
                self.pc = target;
                self.pc_written = true;
            }
        }
        Ok(())
    }
    fn lp_skip(&mut self, target: u32) {
        self.pc = target;
        self.pc_written = true;
    }
    fn halt(&mut self) {
        self.halted = true;
    }
    fn sleep(&mut self) {
        self.sleeping = true;
    }
    fn take_delay(&mut self) -> Option<(u32, bool)> {
        self.delay.take()
    }
    fn delay_pending(&mut self) -> bool {
        self.delay.is_some()
    }
    fn begin_insn(&mut self) {
        self.pc_written = false;
        self.link_executed = false;
    }
    fn pc_written(&mut self) -> bool {
        self.pc_written
    }
    fn set_pc(&mut self, pc: u32) {
        self.pc = pc;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::condition::ConditionCode;
    use crate::sem::regs::*;

    #[test]
    fn steps_a_zero_overhead_loop_and_takes_an_exception() {
        use crate::regs::REG_ILINK2;
        let alu = |sub: u32, b: u32, c: u32, a: u32, p: u32| {
            (0x04u32 << 27) | ((b & 7) << 24) | (p << 22) | (sub << 16) | (((b >> 3) & 7) << 12) | (c << 6) | a
        };
        let mut m = FlatMachine::new(0x4000);
        // mov lp_count, 3 ; lp (LP_END = 0x10C) ; add r4, r4, 1 ; rtie (Instruction Error)
        let prog = [alu(0x0A, 60, 3, 0, 1), alu(0x28, 0, 4, 0, 1), alu(0x00, 4, 1, 4, 1), 0x246F_003F];
        for (i, w) in prog.iter().enumerate() {
            m.write(0x100 + 4 * i as u32, DataSize::Word, *w).unwrap();
        }
        m.pc = 0x100;
        m.set_aux(AuxReg::IntVectorBase, 0x1000);
        while m.pc != 0x1010 {
            m.step().unwrap();
        }
        assert_eq!(m.regs[4], 3, "the loop body ran LP_COUNT times");
        assert_eq!(m.regs[REG_ILINK2 as usize], 0x110, "ILINK2 = the instruction after rtie");
        assert_eq!(m.aux(AuxReg::IrqLv12) & 2, 2);
    }

    #[test]
    fn memory_is_big_endian_and_bounded() {
        let mut m = FlatMachine::new(16);
        m.write(4, DataSize::Word, 0x1122_3344).unwrap();
        assert_eq!(m.mem[4..8], [0x11, 0x22, 0x33, 0x44]);
        assert_eq!(m.read(6, DataSize::HalfWord).unwrap(), 0x3344);
        assert_eq!(m.read(15, DataSize::Byte).unwrap(), 0);
        assert!(m.read(14, DataSize::Word).is_err());
    }

    #[test]
    fn status32_round_trips_through_the_flags() {
        let mut m = FlatMachine::new(0);
        let all: u32 = Flag::ALL.iter().map(|f| 1 << f.status32_bit()).sum();
        for v in [0, all, 0x0000_0A00, 0x0000_0806, 0xFFFF_FFFF & all] {
            set_status32(&mut m, v);
            assert_eq!(status32(&mut m), v);
        }
    }

    #[test]
    fn conditions_match_the_reference_table() {
        let mut m = FlatMachine::new(0);
        for q in 0..16u8 {
            let cc = ConditionCode::from_u8(q).unwrap();
            for bits in 0..16u8 {
                let (z, n, c, v) = (bits & 8 != 0, bits & 4 != 0, bits & 2 != 0, bits & 1 != 0);
                m.set_flag(Flag::Z, z);
                m.set_flag(Flag::N, n);
                m.set_flag(Flag::C, c);
                m.set_flag(Flag::V, v);
                assert_eq!(cond_holds(&mut m, cc), cc.evaluate(z, n, c, v), "{cc:?} {bits:04b}");
            }
        }
    }

    #[test]
    fn privileged_and_special_registers() {
        let mut m = FlatMachine::new(0);
        m.pc = 0x1002;
        m.regs[29] = 7;
        assert_eq!(read_core_reg(&mut m, 29), Ok(7));
        assert_eq!(read_core_reg(&mut m, 63), Ok(0x1000));
        assert_eq!(read_core_reg(&mut m, 62), Ok(0));
        assert_eq!(read_core_reg(&mut m, 61), Err(Exception::InstructionError { address: 0x1002 }));
        m.set_flag(Flag::U, true);
        assert_eq!(read_core_reg(&mut m, 30), Err(Exception::PrivilegeViolation { address: 0x1002 }));
        assert_eq!(write_core_reg(&mut m, 29, 1), Err(Exception::PrivilegeViolation { address: 0x1002 }));
        assert_eq!(write_core_reg(&mut m, 63, 1), Err(Exception::InstructionError { address: 0x1002 }));
        assert_eq!(write_core_reg(&mut m, 62, 1), Ok(()));
    }
}
