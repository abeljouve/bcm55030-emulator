//! Architectural register numbers, auxiliary register addresses and STATUS32
//! bit positions of this core.

// Special register indices
pub const REG_GP: u8 = 26;
pub const REG_FP: u8 = 27;
pub const REG_SP: u8 = 28;
pub const REG_ILINK1: u8 = 29;
pub const REG_ILINK2: u8 = 30;
pub const REG_BLINK: u8 = 31;
pub const REG_LP_COUNT: u8 = 60;
pub const REG_RESERVED: u8 = 61;
pub const REG_LIMM: u8 = 62;
pub const REG_PCL: u8 = 63;

// Auxiliary register addresses
pub const AUX_STATUS: u32 = 0x00;
pub const AUX_SEMAPHORE: u32 = 0x01;
pub const AUX_LP_START: u32 = 0x02;
pub const AUX_LP_END: u32 = 0x03;
pub const AUX_IDENTITY: u32 = 0x04;
pub const AUX_DEBUG: u32 = 0x05;
pub const AUX_PC: u32 = 0x06;
pub const AUX_STATUS32: u32 = 0x0A;
pub const AUX_STATUS32_L1: u32 = 0x0B;
pub const AUX_STATUS32_L2: u32 = 0x0C;
/// A plain read/write register; not IC_CTRL, whatever the generic ARC 700
/// map says. It reads 0x2 after reset and writes have no effect on the
/// I-cache. -- OBSERVED on silicon.
pub const AUX_MULHI: u32 = 0x11;
pub const AUX_COUNT0: u32 = 0x21;
pub const AUX_CONTROL0: u32 = 0x22;
pub const AUX_LIMIT0: u32 = 0x23;
pub const AUX_INT_VECTOR_BASE: u32 = 0x25;
pub const AUX_MACMODE: u32 = 0x41;
pub const AUX_IRQ_LV12: u32 = 0x43;
pub const AUX_COUNT1: u32 = 0x100;
pub const AUX_CONTROL1: u32 = 0x101;
pub const AUX_LIMIT1: u32 = 0x102;
pub const AUX_IRQ_LEV: u32 = 0x200;
pub const AUX_IRQ_HINT: u32 = 0x201;
// Exception-state registers of the generic ARC 700. ERET..EFA (0x400-0x404),
// ICAUSE1/2 and BTA, BTA_L1/L2 are NOT implemented on this integration: a read
// returns `IDENTITY_VALUE` and a write is discarded, like any absent aux. The
// names stay so the disassembler can label them. The core still keeps the
// interrupt cause and branch-target state internally (`aux_icause*`,
// `aux_bta*`), it just does not expose it. -- OBSERVED on silicon.
pub const AUX_ERET: u32 = 0x400;
pub const AUX_ERBTA: u32 = 0x401;
pub const AUX_ERSTATUS: u32 = 0x402;
pub const AUX_ECR: u32 = 0x403;
pub const AUX_EFA: u32 = 0x404;
pub const AUX_ICAUSE1: u32 = 0x40A;
pub const AUX_ICAUSE2: u32 = 0x40B;
/// `IENABLE` in the generic ARC 700 definition — **not implemented on this
/// integration**. Kept as a named address so the disassembler can label it and
/// so the absence is documented where a reader looks for the register. There is
/// deliberately no `aux_ienable` field and no read/write arm: the address falls
/// through to the generic absent-aux handling (read = `IDENTITY_VALUE`, write
/// discarded). -- OBSERVED, DATASHEET §6.1.
pub const AUX_IENABLE: u32 = 0x40C;
pub const AUX_ITRIGGER: u32 = 0x40D;
pub const AUX_XPU: u32 = 0x410;
pub const AUX_BTA: u32 = 0x412;
pub const AUX_BTA_L1: u32 = 0x413;
pub const AUX_BTA_L2: u32 = 0x414;
pub const AUX_IRQ_PULSE_CANCEL: u32 = 0x415;
pub const AUX_IRQ_PENDING: u32 = 0x416;

/// BCM55030 IDENTITY register value (verified against real hardware).
/// ARCVER=0x24, build=0xB4. Non-existent aux regs also return this value.
pub const IDENTITY_VALUE: u32 = 0x00B40124;

// NOTE: a disassembler label `Auxregs0028` = STATUS32 (AUX 0x0A * 4 = byte
// offset 0x28). There is NO BCM55030-custom AUX at number 0x28.

// BCR addresses
pub const AUX_BCR_VER: u32 = 0x60;
pub const AUX_BTA_LINK_BUILD: u32 = 0x63;
pub const AUX_EA_BUILD: u32 = 0x65;
pub const AUX_VECBASE_AC_BUILD: u32 = 0x68;
pub const AUX_RF_BUILD: u32 = 0x6E;
pub const AUX_TIMER_BUILD: u32 = 0x75;
pub const AUX_DCCM_BUILD: u32 = 0x74;
pub const AUX_ICCM_BUILD: u32 = 0x78;
pub const AUX_MULTIPLY_BUILD: u32 = 0x7B;
pub const AUX_SWAP_BUILD: u32 = 0x7C;
pub const AUX_NORM_BUILD: u32 = 0x7D;
pub const AUX_MINMAX_BUILD: u32 = 0x7E;
pub const AUX_BARREL_BUILD: u32 = 0x7F;
pub const AUX_D_CACHE_BUILD: u32 = 0x72;
pub const AUX_I_CACHE_BUILD: u32 = 0x77;

// STATUS32 bit positions
pub const S32_H: u32 = 0;
pub const S32_E1: u32 = 1;
pub const S32_E2: u32 = 2;
pub const S32_A1: u32 = 3;
pub const S32_A2: u32 = 4;
pub const S32_AE: u32 = 5;
pub const S32_DE: u32 = 6;
pub const S32_U: u32 = 7;
pub const S32_V: u32 = 8;
pub const S32_C: u32 = 9;
pub const S32_N: u32 = 10;
pub const S32_Z: u32 = 11;
pub const S32_L: u32 = 12;
