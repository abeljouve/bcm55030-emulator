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

    /// Trap / SWI (vector 0x07 for TRAP, 0x07 for SWI)
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
    pub fn vector_number(&self) -> u8 {
        match self {
            Exception::Reset => 0x00,
            Exception::MemoryError { .. } => 0x01,
            Exception::InstructionError { .. } | Exception::MisalignedAccess { .. } => 0x02,
            Exception::MachineCheck => 0x04,
            Exception::PrivilegeViolation { .. } => 0x06,
            Exception::Trap { .. } => 0x07,
            Exception::Extension => 0x08,
            Exception::Interrupt1 { irq } => 16 + irq,
            Exception::Interrupt2 { irq } => 16 + irq,
            Exception::Halt | Exception::Sleep => 0x00,
        }
    }

    /// Encode ECR: [31:24] reserved | [23:16] vector | [15:8] cause | [7:0] param
    pub fn encode_ecr(&self) -> u32 {
        let vector = self.vector_number() as u32;
        let (cause, param) = match self {
            Exception::MemoryError { is_write, .. } => {
                (if *is_write { 0x02 } else { 0x01 }, 0x00)
            }
            Exception::InstructionError { .. } => (0x02, 0x00),
            Exception::MisalignedAccess { .. } => (0x01, 0x00),
            Exception::PrivilegeViolation { .. } => (0x01, 0x00),
            Exception::Trap { param } => (0x01, *param as u32),
            _ => (0x00, 0x00),
        };
        (vector << 16) | (cause << 8) | param
    }
}
