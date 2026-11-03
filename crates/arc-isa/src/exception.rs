#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Exception {
    /// Reset (vector 0x00)
    Reset,

    /// Memory error: bus error, protection violation (vector 0x01)
    MemoryError { address: u32, is_write: bool },

    /// Instruction error: illegal opcode, illegal register (vector 0x02)
    InstructionError { address: u32 },

    /// Machine check (vector 0x04)
    MachineCheck,

    /// Privilege violation: user mode accessing kernel resources (vector 0x06)
    PrivilegeViolation { address: u32 },

    /// Trap / SWI (slot 2 on this core)
    Trap { param: u8 },

    /// Extension exception (vector 0x08)
    Extension,

    /// Misaligned data access
    MisalignedAccess { address: u32 },

    /// Level 1 interrupt (vectors 16+)
    Interrupt1 { irq: u8 },

    /// Level 2 interrupt (vectors 16+)
    Interrupt2 { irq: u8 },

    /// CPU halted (BRK / FLAG halt)
    Halt,

    /// CPU sleeping (SLEEP instruction)
    Sleep,
}

impl Exception {
    #[inline]
    pub fn vector_number(&self) -> u8 {
        match self {
            Exception::Reset => 0x00,
            Exception::MemoryError { .. } => 0x01,
            Exception::InstructionError { .. } | Exception::MisalignedAccess { .. } => 0x02,
            Exception::MachineCheck => 0x04,
            Exception::PrivilegeViolation { .. } => 0x06,
            // SWI and TRAP_S enter slot 2 on this core, like an Instruction
            // Error, not the generic ARC 700 trap vector. -- OBSERVED on silicon.
            Exception::Trap { .. } => 0x02,
            Exception::Extension => 0x08,
            Exception::Interrupt1 { irq } => 16 + irq,
            Exception::Interrupt2 { irq } => 16 + irq,
            Exception::Halt | Exception::Sleep => 0x00,
        }
    }

    /// Whether the core takes this exception through its vector table. Only
    /// the classes whose entry was measured on silicon are vectored; the
    /// others (bus errors, misaligned or privileged accesses) still stop the
    /// emulation, because what this core does on them is not known.
    #[inline]
    pub fn is_vectored(&self) -> bool {
        matches!(self, Exception::InstructionError { .. } | Exception::Trap { .. })
    }
}
