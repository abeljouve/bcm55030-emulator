//! The value domain the instruction semantics are written against.
//!
//! The semantics never branch on a value. They compute with the primitives
//! below, each of which maps onto one SMT-LIB QF_BV operator, and express
//! data-dependent effects with [`Dom::when`] and [`Dom::fault_if`]. A concrete
//! domain (`u32`/`bool`) runs them; a symbolic one builds terms from the same
//! code. The only way a value can steer the semantics statically is
//! [`Dom::as_const`], reserved for auxiliary register numbers.
//!
//! What varies with the platform (memory, caches, MMIO, interrupt plumbing)
//! sits behind the memory and AUX hooks; what the instruction set defines is
//! written once, in the semantics, on top of this trait.

use crate::decoder::instruction::{DataSize, DelayMode};
use crate::exception::Exception;

/// STATUS32 flags, plus the halt bit.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Flag {
    Z,
    N,
    C,
    V,
    E1,
    E2,
    A1,
    A2,
    AE,
    DE,
    U,
    L,
    H,
}

impl Flag {
    pub const ALL: [Flag; 13] = [
        Flag::Z, Flag::N, Flag::C, Flag::V, Flag::E1, Flag::E2, Flag::A1, Flag::A2, Flag::AE,
        Flag::DE, Flag::U, Flag::L, Flag::H,
    ];

    /// Bit position in STATUS32.
    #[inline]
    pub const fn status32_bit(self) -> u32 {
        use crate::regs::*;
        match self {
            Flag::H => S32_H,
            Flag::E1 => S32_E1,
            Flag::E2 => S32_E2,
            Flag::A1 => S32_A1,
            Flag::A2 => S32_A2,
            Flag::AE => S32_AE,
            Flag::DE => S32_DE,
            Flag::U => S32_U,
            Flag::V => S32_V,
            Flag::C => S32_C,
            Flag::N => S32_N,
            Flag::Z => S32_Z,
            Flag::L => S32_L,
        }
    }
}

/// Auxiliary registers with architectural storage.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum AuxReg {
    Semaphore,
    LpStart,
    LpEnd,
    Status32L1,
    Status32L2,
    Count0,
    Control0,
    Limit0,
    IntVectorBase,
    Mulhi,
    Macmode,
    IrqLv12,
    Count1,
    Control1,
    Limit1,
    IrqLev,
    IrqHint,
    Itrigger,
    Xpu,
    IrqPulseCancel,
    IrqPending,
    DcCtrl,
    /// Branch target of the last taken branch. Not readable by `lr` on this
    /// core; kept because `j.f [ilinkN]` restores it.
    Bta,
    BtaL1,
    BtaL2,
}

impl AuxReg {
    pub const ALL: [AuxReg; 25] = [
        AuxReg::Semaphore, AuxReg::LpStart, AuxReg::LpEnd, AuxReg::Status32L1,
        AuxReg::Status32L2, AuxReg::Count0, AuxReg::Control0, AuxReg::Limit0,
        AuxReg::IntVectorBase, AuxReg::Mulhi, AuxReg::Macmode, AuxReg::IrqLv12, AuxReg::Count1,
        AuxReg::Control1, AuxReg::Limit1, AuxReg::IrqLev, AuxReg::IrqHint, AuxReg::Itrigger,
        AuxReg::Xpu, AuxReg::IrqPulseCancel, AuxReg::IrqPending, AuxReg::DcCtrl, AuxReg::Bta,
        AuxReg::BtaL1, AuxReg::BtaL2,
    ];
}

pub trait Dom {
    /// A 32-bit word.
    type V: Copy;
    /// One bit.
    type B: Copy;

    // ---- primitives (one SMT-LIB QF_BV operator each) ----------------------

    fn k(&mut self, v: u32) -> Self::V;
    fn kb(&mut self, b: bool) -> Self::B;

    fn add(&mut self, a: Self::V, b: Self::V) -> Self::V;
    fn sub(&mut self, a: Self::V, b: Self::V) -> Self::V;
    fn and(&mut self, a: Self::V, b: Self::V) -> Self::V;
    fn or(&mut self, a: Self::V, b: Self::V) -> Self::V;
    fn xor(&mut self, a: Self::V, b: Self::V) -> Self::V;
    fn not(&mut self, a: Self::V) -> Self::V;
    fn neg(&mut self, a: Self::V) -> Self::V;
    /// `bvshl`: a count of 32 or more gives 0.
    fn shl(&mut self, a: Self::V, n: Self::V) -> Self::V;
    /// `bvlshr`: a count of 32 or more gives 0.
    fn lshr(&mut self, a: Self::V, n: Self::V) -> Self::V;
    /// `bvashr`: a count of 32 or more gives the sign fill.
    fn ashr(&mut self, a: Self::V, n: Self::V) -> Self::V;

    fn eq(&mut self, a: Self::V, b: Self::V) -> Self::B;
    fn ult(&mut self, a: Self::V, b: Self::V) -> Self::B;
    fn slt(&mut self, a: Self::V, b: Self::V) -> Self::B;

    fn ite(&mut self, c: Self::B, a: Self::V, b: Self::V) -> Self::V;
    fn ite_b(&mut self, c: Self::B, a: Self::B, b: Self::B) -> Self::B;
    fn band(&mut self, a: Self::B, b: Self::B) -> Self::B;
    fn bor(&mut self, a: Self::B, b: Self::B) -> Self::B;
    fn bxor(&mut self, a: Self::B, b: Self::B) -> Self::B;
    fn bnot(&mut self, a: Self::B) -> Self::B;

    /// Bit `i` (static) of `a`.
    fn bit(&mut self, a: Self::V, i: u8) -> Self::B;
    /// 0 or 1.
    fn b2v(&mut self, b: Self::B) -> Self::V;
    /// Sign-extend the low `bits` (8 or 16) bits.
    fn sext(&mut self, a: Self::V, bits: u8) -> Self::V;
    /// Zero-extend the low `bits` (8 or 16) bits.
    fn zext(&mut self, a: Self::V, bits: u8) -> Self::V;
    /// Count of leading zero bits (32 for 0).
    fn clz(&mut self, a: Self::V) -> Self::V;

    /// The value, when it is a known constant. The concrete domain always
    /// answers; a symbolic one only for a constant term. The semantics use it
    /// for one thing: the auxiliary register number of `lr`/`sr`.
    fn as_const(&self, v: Self::V) -> Option<u32>;

    // ---- architectural state ----------------------------------------------

    /// Address of the instruction being executed. Always concrete: code is
    /// lifted one instruction address at a time.
    fn pc(&self) -> u32;
    /// Raw core register storage, 0..=63. Privilege checks and the special
    /// numbers (61, 62, 63) are the semantics' business, not the domain's.
    fn reg(&mut self, r: u8) -> Self::V;
    fn set_reg(&mut self, r: u8, v: Self::V);
    fn flag(&mut self, f: Flag) -> Self::B;
    fn set_flag(&mut self, f: Flag, b: Self::B);

    // ---- data-dependent effects --------------------------------------------

    /// Run `body` when `g` holds. Concrete: a branch. Symbolic: `body` runs on
    /// a copy of the state, and every register, flag and effect it touched is
    /// merged back as `ite(g, new, old)`.
    fn when(
        &mut self,
        g: Self::B,
        body: impl FnOnce(&mut Self) -> Result<(), Exception>,
    ) -> Result<(), Exception>;
    /// Raise `e` when `c` holds.
    fn fault_if(&mut self, c: Self::B, e: Exception) -> Result<(), Exception>;

    // ---- memory and auxiliary registers (platform) -------------------------

    /// Load from the core-aligned address `ea`; `raw` is the address before
    /// alignment, for platforms that log misaligned accesses. The result is
    /// zero-extended; sign extension is the semantics' job.
    fn load(&mut self, ea: Self::V, raw: Self::V, size: DataSize, di: bool)
        -> Result<Self::V, Exception>;
    fn store(&mut self, ea: Self::V, raw: Self::V, size: DataSize, v: Self::V, di: bool)
        -> Result<(), Exception>;
    /// Raw storage of an auxiliary register the instruction set defines.
    /// Masks, read-only bits, privilege and absent addresses are applied by
    /// the semantics, not here.
    fn aux(&mut self, a: AuxReg) -> Self::V;
    fn set_aux(&mut self, a: AuxReg, v: Self::V);
    /// A platform register answering `lr` before the instruction set does
    /// (cache probes, debug status bits driven by a timing model). `None`
    /// leaves the read to the semantics.
    fn aux_read_hook(&mut self, addr: u32) -> Option<Result<Self::V, Exception>>;
    /// Platform side effects of `sr`, after the instruction set's own write
    /// (cache invalidation and control). `v` is the source value.
    fn aux_write_hook(&mut self, addr: u32, v: Self::V) -> Result<(), Exception>;
    /// `lr` / `sr` whose register number is not a known constant. A concrete
    /// domain never gets here (`as_const` always answers); a symbolic one
    /// records an unknown auxiliary access and marks the code uncertain.
    fn aux_read_dynamic(&mut self, addr: Self::V) -> Result<Self::V, Exception>;
    fn aux_write_dynamic(&mut self, addr: Self::V, v: Self::V) -> Result<(), Exception>;

    // ---- control effects ---------------------------------------------------

    /// A taken branch or jump to `target`. With `delay`, the next instruction
    /// executes first; `link` is the return address to save in BLINK.
    fn branch(&mut self, target: Self::V, delay: DelayMode, link: Option<u32>)
        -> Result<(), Exception>;
    /// A conditional LP whose condition failed: continue at `target`.
    fn lp_skip(&mut self, target: u32);
    fn halt(&mut self);
    fn sleep(&mut self);
}
