use crate::cpu::condition::ConditionCode;

/// Operand for a decoded instruction
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Operand {
    /// Core register (0-63)
    Reg(u8),
    /// Immediate value (already sign/zero extended)
    Imm(u32),
    /// No operand
    None,
}

/// ALU operation sub-opcodes (major opcode 0x04)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum AluOp {
    Add = 0x00,
    Adc = 0x01,
    Sub = 0x02,
    Sbc = 0x03,
    And = 0x04,
    Or = 0x05,
    Bic = 0x06,
    Xor = 0x07,
    Max = 0x08,
    Min = 0x09,
    Mov = 0x0A,
    Tst = 0x0B,
    Cmp = 0x0C,
    Rcmp = 0x0D,
    Rsub = 0x0E,
    Bset = 0x0F,
    Bclr = 0x10,
    Btst = 0x11,
    Bxor = 0x12,
    Bmsk = 0x13,
    Add1 = 0x14,
    Add2 = 0x15,
    Add3 = 0x16,
    Sub1 = 0x17,
    Sub2 = 0x18,
    Sub3 = 0x19,
    Mpy = 0x1A,
    Mpyh = 0x1B,
    Mpyhu = 0x1C,
    Mpyu = 0x1D,
}

impl AluOp {
    pub fn from_u8(val: u8) -> Option<Self> {
        match val {
            0x00 => Some(Self::Add),
            0x01 => Some(Self::Adc),
            0x02 => Some(Self::Sub),
            0x03 => Some(Self::Sbc),
            0x04 => Some(Self::And),
            0x05 => Some(Self::Or),
            0x06 => Some(Self::Bic),
            0x07 => Some(Self::Xor),
            0x08 => Some(Self::Max),
            0x09 => Some(Self::Min),
            0x0A => Some(Self::Mov),
            0x0B => Some(Self::Tst),
            0x0C => Some(Self::Cmp),
            0x0D => Some(Self::Rcmp),
            0x0E => Some(Self::Rsub),
            0x0F => Some(Self::Bset),
            0x10 => Some(Self::Bclr),
            0x11 => Some(Self::Btst),
            0x12 => Some(Self::Bxor),
            0x13 => Some(Self::Bmsk),
            0x14 => Some(Self::Add1),
            0x15 => Some(Self::Add2),
            0x16 => Some(Self::Add3),
            0x17 => Some(Self::Sub1),
            0x18 => Some(Self::Sub2),
            0x19 => Some(Self::Sub3),
            0x1A => Some(Self::Mpy),
            0x1B => Some(Self::Mpyh),
            0x1C => Some(Self::Mpyhu),
            0x1D => Some(Self::Mpyu),
            _ => None,
        }
    }

    /// Whether this op discards the result (only sets flags)
    pub fn is_test_only(self) -> bool {
        matches!(self, Self::Tst | Self::Cmp | Self::Rcmp | Self::Btst)
    }

    /// Whether this is a MOV (single source operand from C, not B)
    pub fn is_mov(self) -> bool {
        matches!(self, Self::Mov)
    }
}

/// Single-operand sub-sub-opcodes (major 0x04, sub-opcode 0x2F)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SingleOp {
    Asl,    // 0x00 - shift left by 1
    Lsr,    // 0x01 - logical shift right by 1
    Asr,    // 0x02 - arithmetic shift right by 1
    Ror,    // 0x03 - rotate right by 1
    Rrc,    // 0x04 - rotate right through carry
    Sexb,   // 0x05 - sign extend byte
    Sexw,   // 0x06 - sign extend word (halfword)
    Extb,   // 0x07 - zero extend byte
    Extw,   // 0x08 - zero extend word (halfword)
    Abs,    // 0x09 - absolute value
    Not,    // 0x0A - bitwise NOT
    Rlc,    // 0x0B - rotate left through carry
    Ex,     // 0x16 - atomic exchange
}

impl SingleOp {
    pub fn from_u8(val: u8) -> Option<Self> {
        match val {
            0x00 => Some(Self::Asl),
            0x01 => Some(Self::Lsr),
            0x02 => Some(Self::Asr),
            0x03 => Some(Self::Ror),
            0x04 => Some(Self::Rrc),
            0x05 => Some(Self::Sexb),
            0x06 => Some(Self::Sexw),
            0x07 => Some(Self::Extb),
            0x08 => Some(Self::Extw),
            0x09 => Some(Self::Abs),
            0x0A => Some(Self::Not),
            0x0B => Some(Self::Rlc),
            0x16 => Some(Self::Ex),
            _ => None,
        }
    }
}

/// Zero-operand instructions
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ZeroOp {
    Nop,
    Sleep { u6: u8 },
    Swi,
    Brk,
    Trap { param: u8 },
    Rtie,
    Sync,
}

/// Extended arithmetic operations (major 0x05)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ExtArithOp {
    // Dual-operand shifts (sub 0x00-0x03)
    Asl,      // 0x00 - multi-bit shift left
    Lsr,      // 0x01 - multi-bit logical shift right
    Asr,      // 0x02 - multi-bit arithmetic shift right
    Ror,      // 0x03 - multi-bit rotate right
    // Saturating arithmetic (sub 0x06-0x07)
    Adds,     // 0x06
    Subs,     // 0x07
    Divaw,    // 0x08
    Asls,     // 0x0A - shift left with saturation
    Asrs,     // 0x0B - shift right with saturation
    // 0x04 (Mul64) and 0x05 (Mulu64) not supported on ARC 700
    Addsdw,   // 0x28
    Subsdw,   // 0x29
    // Single-operand extensions (sub 0x2F, a field = sub-opcode2)
    Swap,     // a=0x00
    Norm,     // a=0x01
    Sat16,    // a=0x02
    Rnd16,    // a=0x03
    Abssw,    // a=0x04
    Abss,     // a=0x05
    Negsw,    // a=0x06
    Negs,     // a=0x07
    Normw,    // a=0x08
}

/// Data sizes for load/store
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DataSize {
    Word,     // 32-bit (ZZ=00)
    Byte,     // 8-bit  (ZZ=01)
    HalfWord, // 16-bit (ZZ=10)
}

/// Address writeback modes
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WritebackMode {
    None,       // AA=00
    PreWrite,   // AA=01 (.AW)
    PostWrite,  // AA=10 (.AB)
    Scaled,     // AA=11 (.AS)
}

/// Delay slot mode
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DelayMode {
    NoDelay,   // N=0 (ND): nullify delay slot if branch taken
    Delay,     // N=1 (D): always execute delay slot
}

/// BRcc compare kind
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BrCompareKind {
    Breq,   // 0x00
    Brne,   // 0x01
    Brlt,   // 0x02
    Brge,   // 0x03
    Brlo,   // 0x04
    Brhs,   // 0x05
    Bbit0,  // 0x0E
    Bbit1,  // 0x0F
}

impl BrCompareKind {
    pub fn from_u8(val: u8) -> Option<Self> {
        match val {
            0x00 => Some(Self::Breq),
            0x01 => Some(Self::Brne),
            0x02 => Some(Self::Brlt),
            0x03 => Some(Self::Brge),
            0x04 => Some(Self::Brlo),
            0x05 => Some(Self::Brhs),
            0x0E => Some(Self::Bbit0),
            0x0F => Some(Self::Bbit1),
            _ => None,
        }
    }
}

/// Decoded instruction
#[derive(Debug, Clone)]
pub enum Instruction {
    /// ALU dual-operand (major 0x04 sub-opcodes 0x00-0x1D, or 16-bit equivalents)
    Alu {
        op: AluOp,
        dst: Operand,
        src1: Operand,
        src2: Operand,
        set_flags: bool,
        cc: Option<ConditionCode>,
    },

    /// Single-operand (major 0x04, sub-opcode 0x2F, or 16-bit equivalents)
    SingleOp {
        op: SingleOp,
        dst: Operand,
        src: Operand,
        set_flags: bool,
        cc: Option<ConditionCode>,
    },

    /// Zero-operand
    ZeroOp(ZeroOp),

    /// Branch (B/Bcc/BL/BLcc)
    Branch {
        offset: i32,
        cc: Option<ConditionCode>,
        delay: DelayMode,
        link: bool,
    },

    /// Branch on compare (BRcc, BBIT0, BBIT1)
    BranchCompare {
        kind: BrCompareKind,
        src1: Operand,
        src2: Operand,
        offset: i32,
        delay: DelayMode,
    },

    /// Jump (J/JL)
    Jump {
        target: Operand,
        cc: Option<ConditionCode>,
        delay: DelayMode,
        link: bool,
        flag_restore: bool,
    },

    /// Load from data memory
    Load {
        dst: Operand,
        base: Operand,
        offset: Operand,
        data_size: DataSize,
        sign_extend: bool,
        writeback: WritebackMode,
        cache_bypass: bool,
    },

    /// Store to data memory
    Store {
        src: Operand,
        base: Operand,
        offset: Operand,
        data_size: DataSize,
        writeback: WritebackMode,
        cache_bypass: bool,
    },

    /// Zero-overhead loop setup (LPcc)
    Loop {
        offset: u32,
        cc: Option<ConditionCode>,
    },

    /// Load from auxiliary register (LR)
    LoadAux {
        dst: Operand,
        addr: Operand,
    },

    /// Store to auxiliary register (SR)
    StoreAux {
        src: Operand,
        addr: Operand,
    },

    /// FLAG instruction
    Flag {
        src: Operand,
        cc: Option<ConditionCode>,
    },

    /// Extended arithmetic (major 0x05)
    ExtArith {
        op: ExtArithOp,
        dst: Operand,
        src1: Operand,
        src2: Operand,
        set_flags: bool,
        cc: Option<ConditionCode>,
    },

    /// PREFETCH (treated as NOP in emulator)
    Prefetch,
}

/// A fully decoded instruction with metadata
#[derive(Debug, Clone)]
pub struct DecodedInstruction {
    pub inst: Instruction,
    /// Size of the instruction itself in bytes (2 or 4)
    pub size: u8,
    /// Whether a LIMM word follows (adds 4 bytes to total)
    pub has_limm: bool,
    /// PC of this instruction
    pub pc: u32,
}

impl DecodedInstruction {
    /// Total size in bytes including LIMM
    pub fn total_size(&self) -> u32 {
        self.size as u32 + if self.has_limm { 4 } else { 0 }
    }
}
