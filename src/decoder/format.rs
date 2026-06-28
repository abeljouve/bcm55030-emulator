//! Pretty-printer for [`DecodedInstruction`]. Pure function — no
//! state, no `Memory`, no annotations. User symbols are resolved
//! at the call site (UI disassembly panel / MCP `disassemble`
//! tool), not inside this module, so the formatter stays
//! firmware-agnostic (the contributor guide).
//!
//! Output conventions mirror the ARCompact mnemonic style used in
//! Synopsys assembler listings:
//!
//! ```text
//! add.f  r0, r1, r2
//! ld.di  r3, [r4, 0x40]
//! bl.d   0x00000124
//! lr     r5, [status32]
//! nop_s
//! ```
//!
//! A fall-through `format!("{:?}", inst)` prevents panics on any
//! variant the formatter doesn't yet handle — the UI sees a raw
//! debug string rather than a missing line.

use crate::cpu::condition::ConditionCode;
use crate::decoder::instruction::{
    AluOp, BrCompareKind, DataSize, DecodedInstruction, DelayMode, ExtArithOp, Instruction,
    Operand, SingleOp, WritebackMode, ZeroOp,
};

/// One rendered disassembly row, ready for the UI to place into a
/// table without any further formatting.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct FormattedLine {
    pub address: u32,
    /// Total instruction size in bytes (includes LIMM).
    pub size: u32,
    /// Big-endian hex dump of the encoded bytes, e.g. `"78E0"` or
    /// `"20418000 0000002A"` (LIMM separated by a space).
    pub hex_bytes: String,
    /// Mnemonic including optional `.f` / `.cc` / `.d` / `.di`
    /// suffixes. Examples: `"add"`, `"bl.d"`, `"ld.di"`.
    pub mnemonic: String,
    /// Rendered operand list, comma-separated.
    pub operands: String,
    /// Absolute target address for unconditional fixed-target
    /// branches / jumps, so the UI can rewrite it to a user
    /// symbol or draw a jump arrow.
    pub branch_target: Option<u32>,
    /// `true` when the instruction is a branch/jump with
    /// `DelayMode::Delay` — the following instruction executes
    /// in the delay slot.
    pub is_delay_slot_carrier: bool,
}

/// Format a decoded instruction into a `FormattedLine`. `raw` is
/// the on-disk byte sequence (for the `hex_bytes` column);
/// callers hand over `&sram[pc..pc + total_size]`.
pub fn format_line(dec: &DecodedInstruction, raw: &[u8]) -> FormattedLine {
    let (mnemonic, operands) = format_mnemonic_and_operands(dec);
    FormattedLine {
        address: dec.pc,
        size: dec.total_size(),
        hex_bytes: hex_dump(raw, dec.size),
        mnemonic,
        operands,
        branch_target: branch_target(dec),
        is_delay_slot_carrier: is_delay_slot_carrier(&dec.inst),
    }
}

/// Format just the mnemonic + operands, without the surrounding
/// table metadata. Useful for headless trace output.
pub fn format_instruction(dec: &DecodedInstruction) -> String {
    let (m, o) = format_mnemonic_and_operands(dec);
    if o.is_empty() { m } else { format!("{} {}", m, o) }
}

/// Canonical name of a core register. SoC/ISA-level only —
/// never carries firmware symbols.
pub fn format_core_reg(n: u8) -> String {
    match n {
        26 => "gp".to_string(),
        27 => "fp".to_string(),
        28 => "sp".to_string(),
        29 => "ilink1".to_string(),
        30 => "ilink2".to_string(),
        31 => "blink".to_string(),
        60 => "lp_count".to_string(),
        62 => "limm".to_string(),
        63 => "pcl".to_string(),
        n => format!("r{}", n),
    }
}

/// Render a single operand.
pub fn format_operand(op: Operand) -> String {
    match op {
        Operand::Reg(n) => format_core_reg(n),
        Operand::Imm(v) => format!("0x{:X}", v),
        Operand::None => String::new(),
    }
}

fn format_mnemonic_and_operands(dec: &DecodedInstruction) -> (String, String) {
    match &dec.inst {
        Instruction::Alu {
            op,
            dst,
            src1,
            src2,
            set_flags,
            cc,
        } => format_alu(*op, *dst, *src1, *src2, *set_flags, *cc),
        Instruction::SingleOp {
            op,
            dst,
            src,
            set_flags,
            cc,
        } => format_single_op(*op, *dst, *src, *set_flags, *cc),
        Instruction::ZeroOp(zop) => (format_zero_op(*zop), String::new()),
        Instruction::Branch {
            offset,
            cc,
            delay,
            link,
        } => format_branch(dec.pc, *offset, *cc, *delay, *link),
        Instruction::BranchCompare {
            kind,
            src1,
            src2,
            offset,
            delay,
        } => format_br_compare(dec.pc, *kind, *src1, *src2, *offset, *delay),
        Instruction::Jump {
            target,
            cc,
            delay,
            link,
            flag_restore: _,
        } => format_jump(*target, *cc, *delay, *link),
        Instruction::Load {
            dst,
            base,
            offset,
            data_size,
            sign_extend,
            writeback,
            cache_bypass,
        } => format_load(
            *dst,
            *base,
            *offset,
            *data_size,
            *sign_extend,
            *writeback,
            *cache_bypass,
        ),
        Instruction::Store {
            src,
            base,
            offset,
            data_size,
            writeback,
            cache_bypass,
        } => format_store(
            *src,
            *base,
            *offset,
            *data_size,
            *writeback,
            *cache_bypass,
        ),
        Instruction::Loop { offset, cc } => format_loop(dec.pc, *offset, *cc),
        Instruction::LoadAux { dst, addr } => (
            "lr".to_string(),
            format!("{}, [{}]", format_operand(*dst), format_aux_target(*addr)),
        ),
        Instruction::StoreAux { src, addr } => (
            "sr".to_string(),
            format!("{}, [{}]", format_operand(*src), format_aux_target(*addr)),
        ),
        Instruction::Flag { src, cc } => {
            let mut m = "flag".to_string();
            if let Some(c) = cc {
                m.push('.');
                m.push_str(format_cond(*c));
            }
            (m, format_operand(*src))
        }
        Instruction::ExtArith {
            op,
            dst,
            src1,
            src2,
            set_flags,
            cc,
        } => format_ext_arith(*op, *dst, *src1, *src2, *set_flags, *cc),
        Instruction::Prefetch => ("prefetch".to_string(), String::new()),
    }
}

// -------- Alu --------------------------------------------------------------

fn format_alu(
    op: AluOp,
    dst: Operand,
    src1: Operand,
    src2: Operand,
    set_flags: bool,
    cc: Option<ConditionCode>,
) -> (String, String) {
    let mnemonic = build_suffix(format_alu_op(op), set_flags, cc, false);
    let operands = if op.is_test_only() {
        // TST / CMP / RCMP / BTST — no destination.
        format!("{}, {}", format_operand(src1), format_operand(src2))
    } else if op.is_mov() {
        // MOV: single source from `src2` per ISA. Some decoders
        // store it in src1 instead; handle both.
        let src = if matches!(src2, Operand::None) { src1 } else { src2 };
        format!("{}, {}", format_operand(dst), format_operand(src))
    } else {
        format!(
            "{}, {}, {}",
            format_operand(dst),
            format_operand(src1),
            format_operand(src2)
        )
    };
    (mnemonic, operands)
}

fn format_alu_op(op: AluOp) -> &'static str {
    match op {
        AluOp::Add => "add",
        AluOp::Adc => "adc",
        AluOp::Sub => "sub",
        AluOp::Sbc => "sbc",
        AluOp::And => "and",
        AluOp::Or => "or",
        AluOp::Bic => "bic",
        AluOp::Xor => "xor",
        AluOp::Max => "max",
        AluOp::Min => "min",
        AluOp::Mov => "mov",
        AluOp::Tst => "tst",
        AluOp::Cmp => "cmp",
        AluOp::Rcmp => "rcmp",
        AluOp::Rsub => "rsub",
        AluOp::Bset => "bset",
        AluOp::Bclr => "bclr",
        AluOp::Btst => "btst",
        AluOp::Bxor => "bxor",
        AluOp::Bmsk => "bmsk",
        AluOp::Add1 => "add1",
        AluOp::Add2 => "add2",
        AluOp::Add3 => "add3",
        AluOp::Sub1 => "sub1",
        AluOp::Sub2 => "sub2",
        AluOp::Sub3 => "sub3",
        AluOp::Mpy => "mpy",
        AluOp::Mpyh => "mpyh",
        AluOp::Mpyhu => "mpyhu",
        AluOp::Mpyu => "mpyu",
    }
}

// -------- Single-operand ---------------------------------------------------

fn format_single_op(
    op: SingleOp,
    dst: Operand,
    src: Operand,
    set_flags: bool,
    cc: Option<ConditionCode>,
) -> (String, String) {
    let base = match op {
        SingleOp::Asl => "asl",
        SingleOp::Lsr => "lsr",
        SingleOp::Asr => "asr",
        SingleOp::Ror => "ror",
        SingleOp::Rrc => "rrc",
        SingleOp::Sexb => "sexb",
        SingleOp::Sexw => "sexw",
        SingleOp::Extb => "extb",
        SingleOp::Extw => "extw",
        SingleOp::Abs => "abs",
        SingleOp::Not => "not",
        SingleOp::Rlc => "rlc",
        SingleOp::Ex => "ex",
    };
    let mnemonic = build_suffix(base, set_flags, cc, false);
    (
        mnemonic,
        format!("{}, {}", format_operand(dst), format_operand(src)),
    )
}

// -------- Zero-operand -----------------------------------------------------

fn format_zero_op(op: ZeroOp) -> String {
    match op {
        ZeroOp::Nop => "nop".to_string(),
        ZeroOp::Sleep { u6 } => format!("sleep 0x{:02X}", u6),
        ZeroOp::Swi => "swi".to_string(),
        ZeroOp::Brk => "brk".to_string(),
        ZeroOp::Trap { param } => format!("trap 0x{:02X}", param),
        ZeroOp::Rtie => "rtie".to_string(),
        ZeroOp::Sync => "sync".to_string(),
    }
}

// -------- Branch / jump ----------------------------------------------------

fn format_branch(
    pc: u32,
    offset: i32,
    cc: Option<ConditionCode>,
    delay: DelayMode,
    link: bool,
) -> (String, String) {
    let base = if link { "bl" } else { "b" };
    let mut mnemonic = base.to_string();
    if let Some(c) = cc {
        mnemonic.push('.');
        mnemonic.push_str(format_cond(c));
    }
    if matches!(delay, DelayMode::Delay) {
        mnemonic.push_str(".d");
    }
    // ARC branch offsets are relative to PCL (PC & ~3), matching the executor
    // (executor/branch.rs). Using the raw pc shows targets +2 too high for
    // branches at 2-mod-4 addresses (a display-only bug that misled audits).
    let target = ((pc & !3) as i64).wrapping_add(offset as i64) as u32;
    (mnemonic, format!("0x{:08X}", target))
}

fn format_br_compare(
    pc: u32,
    kind: BrCompareKind,
    src1: Operand,
    src2: Operand,
    offset: i32,
    delay: DelayMode,
) -> (String, String) {
    let base = match kind {
        BrCompareKind::Breq => "breq",
        BrCompareKind::Brne => "brne",
        BrCompareKind::Brlt => "brlt",
        BrCompareKind::Brge => "brge",
        BrCompareKind::Brlo => "brlo",
        BrCompareKind::Brhs => "brhs",
        BrCompareKind::Bbit0 => "bbit0",
        BrCompareKind::Bbit1 => "bbit1",
    };
    let mut mnemonic = base.to_string();
    if matches!(delay, DelayMode::Delay) {
        mnemonic.push_str(".d");
    }
    let target = ((pc & !3) as i64).wrapping_add(offset as i64) as u32; // PCL-relative (see format_branch)
    (
        mnemonic,
        format!(
            "{}, {}, 0x{:08X}",
            format_operand(src1),
            format_operand(src2),
            target
        ),
    )
}

fn format_jump(
    target: Operand,
    cc: Option<ConditionCode>,
    delay: DelayMode,
    link: bool,
) -> (String, String) {
    let base = if link { "jl" } else { "j" };
    let mut mnemonic = base.to_string();
    if let Some(c) = cc {
        mnemonic.push('.');
        mnemonic.push_str(format_cond(c));
    }
    if matches!(delay, DelayMode::Delay) {
        mnemonic.push_str(".d");
    }
    let ops = match target {
        Operand::Reg(_) => format!("[{}]", format_operand(target)),
        Operand::Imm(v) => format!("0x{:08X}", v),
        Operand::None => String::new(),
    };
    (mnemonic, ops)
}

// -------- Load / store -----------------------------------------------------

fn format_load(
    dst: Operand,
    base: Operand,
    offset: Operand,
    data_size: DataSize,
    sign_extend: bool,
    writeback: WritebackMode,
    cache_bypass: bool,
) -> (String, String) {
    let mut m = String::from("ld");
    m.push_str(data_size_suffix(data_size));
    if sign_extend {
        m.push_str(".x");
    }
    m.push_str(writeback_suffix(writeback));
    if cache_bypass {
        m.push_str(".di");
    }
    (
        m,
        format!(
            "{}, {}",
            format_operand(dst),
            format_mem_operand(base, offset)
        ),
    )
}

fn format_store(
    src: Operand,
    base: Operand,
    offset: Operand,
    data_size: DataSize,
    writeback: WritebackMode,
    cache_bypass: bool,
) -> (String, String) {
    let mut m = String::from("st");
    m.push_str(data_size_suffix(data_size));
    m.push_str(writeback_suffix(writeback));
    if cache_bypass {
        m.push_str(".di");
    }
    (
        m,
        format!(
            "{}, {}",
            format_operand(src),
            format_mem_operand(base, offset)
        ),
    )
}

fn format_mem_operand(base: Operand, offset: Operand) -> String {
    match (base, offset) {
        (Operand::None, Operand::None) => "[0]".to_string(),
        (_, Operand::None) => format!("[{}]", format_operand(base)),
        (Operand::None, Operand::Imm(v)) => format!("[0x{:X}]", v),
        (b, Operand::Imm(v)) => format!("[{}, 0x{:X}]", format_operand(b), v),
        (b, o) => format!("[{}, {}]", format_operand(b), format_operand(o)),
    }
}

fn data_size_suffix(sz: DataSize) -> &'static str {
    match sz {
        DataSize::Word => "",
        DataSize::Byte => "b",
        DataSize::HalfWord => "w",
    }
}

fn writeback_suffix(wb: WritebackMode) -> &'static str {
    match wb {
        WritebackMode::None => "",
        WritebackMode::PreWrite => ".aw",
        WritebackMode::PostWrite => ".ab",
        WritebackMode::Scaled => ".as",
    }
}

// -------- Loop -------------------------------------------------------------

fn format_loop(pc: u32, offset: u32, cc: Option<ConditionCode>) -> (String, String) {
    let mut m = String::from("lp");
    if let Some(c) = cc {
        m.push('.');
        m.push_str(format_cond(c));
    }
    let target = (pc & !3).wrapping_add(offset); // PCL-relative (executor/special.rs uses pc & ~3)
    (m, format!("0x{:08X}", target))
}

// -------- Aux register targets --------------------------------------------

fn format_aux_target(op: Operand) -> String {
    match op {
        Operand::Imm(v) => aux_name(v)
            .map(|name| name.to_string())
            .unwrap_or_else(|| format!("0x{:X}", v)),
        other => format_operand(other),
    }
}

/// ARC 700 / BCM55030 SoC-level aux register name table. Hardware
/// documentation, not firmware — safe to ship. Unknown addresses
/// fall back to a hex literal.
fn aux_name(addr: u32) -> Option<&'static str> {
    match addr {
        0x000 => Some("status"),
        0x001 => Some("semaphore"),
        0x002 => Some("lp_start"),
        0x003 => Some("lp_end"),
        0x004 => Some("identity"),
        0x005 => Some("debug"),
        0x006 => Some("pc"),
        0x00A => Some("status32"),
        0x00B => Some("status32_l1"),
        0x00C => Some("status32_l2"),
        0x010 => Some("ic_ivic"),
        0x019 => Some("ic_ivil"),
        0x021 => Some("count0"),
        0x022 => Some("control0"),
        0x023 => Some("limit0"),
        0x025 => Some("int_vector_base"),
        0x037 => Some("aux_volatile"),
        0x043 => Some("aux_macmode"),
        0x047 => Some("dc_ivdc"),
        0x048 => Some("dc_ctrl"),
        0x04A => Some("dc_ivdl"),
        0x04B => Some("dc_flsh"),
        0x058 => Some("dc_ram_addr"),
        0x059 => Some("dc_tag"),
        0x05B => Some("dc_data"),
        0x100 => Some("count1"),
        0x101 => Some("control1"),
        0x102 => Some("limit1"),
        0x400 => Some("eret"),
        0x401 => Some("erbta"),
        0x402 => Some("erstatus"),
        0x403 => Some("ecr"),
        0x404 => Some("efa"),
        0x40A => Some("icause1"),
        0x40B => Some("icause2"),
        0x40C => Some("ienable"),
        0x40D => Some("itrigger"),
        0x412 => Some("bta"),
        0x413 => Some("bta_l1"),
        0x414 => Some("bta_l2"),
        0x415 => Some("irq_pulse_cancel"),
        0x416 => Some("irq_pending"),
        _ => None,
    }
}

// -------- ExtArith ---------------------------------------------------------

fn format_ext_arith(
    op: ExtArithOp,
    dst: Operand,
    src1: Operand,
    src2: Operand,
    set_flags: bool,
    cc: Option<ConditionCode>,
) -> (String, String) {
    let (base, single_src) = match op {
        ExtArithOp::Asl => ("asl", false),
        ExtArithOp::Lsr => ("lsr", false),
        ExtArithOp::Asr => ("asr", false),
        ExtArithOp::Ror => ("ror", false),
        ExtArithOp::Adds => ("adds", false),
        ExtArithOp::Subs => ("subs", false),
        ExtArithOp::Divaw => ("divaw", false),
        ExtArithOp::Asls => ("asls", false),
        ExtArithOp::Asrs => ("asrs", false),
        ExtArithOp::Addsdw => ("addsdw", false),
        ExtArithOp::Subsdw => ("subsdw", false),
        ExtArithOp::Swap => ("swap", true),
        ExtArithOp::Norm => ("norm", true),
        ExtArithOp::Sat16 => ("sat16", true),
        ExtArithOp::Rnd16 => ("rnd16", true),
        ExtArithOp::Abssw => ("abssw", true),
        ExtArithOp::Abss => ("abss", true),
        ExtArithOp::Negsw => ("negsw", true),
        ExtArithOp::Negs => ("negs", true),
        ExtArithOp::Normw => ("normw", true),
    };
    let mnemonic = build_suffix(base, set_flags, cc, false);
    let operands = if single_src {
        format!("{}, {}", format_operand(dst), format_operand(src1))
    } else {
        format!(
            "{}, {}, {}",
            format_operand(dst),
            format_operand(src1),
            format_operand(src2)
        )
    };
    (mnemonic, operands)
}

// -------- Suffix helpers ---------------------------------------------------

fn build_suffix(
    base: &str,
    set_flags: bool,
    cc: Option<ConditionCode>,
    delay: bool,
) -> String {
    let mut m = base.to_string();
    if let Some(c) = cc {
        m.push('.');
        m.push_str(format_cond(c));
    }
    if set_flags {
        m.push_str(".f");
    }
    if delay {
        m.push_str(".d");
    }
    m
}

fn format_cond(c: ConditionCode) -> &'static str {
    match c {
        ConditionCode::AL => "",
        ConditionCode::EQ => "eq",
        ConditionCode::NE => "ne",
        ConditionCode::PL => "pl",
        ConditionCode::MI => "mi",
        ConditionCode::CS => "cs",
        ConditionCode::CC => "cc",
        ConditionCode::VS => "vs",
        ConditionCode::VC => "vc",
        ConditionCode::GT => "gt",
        ConditionCode::GE => "ge",
        ConditionCode::LT => "lt",
        ConditionCode::LE => "le",
        ConditionCode::HI => "hi",
        ConditionCode::LS => "ls",
        ConditionCode::PNZ => "pnz",
    }
}

// -------- Metadata ---------------------------------------------------------

fn is_delay_slot_carrier(inst: &Instruction) -> bool {
    match inst {
        Instruction::Branch { delay, .. }
        | Instruction::BranchCompare { delay, .. }
        | Instruction::Jump { delay, .. } => matches!(delay, DelayMode::Delay),
        _ => false,
    }
}

fn branch_target(dec: &DecodedInstruction) -> Option<u32> {
    match &dec.inst {
        Instruction::Branch { offset, .. } => {
            Some(((dec.pc & !3) as i64).wrapping_add(*offset as i64) as u32)
        }
        Instruction::BranchCompare { offset, .. } => {
            Some(((dec.pc & !3) as i64).wrapping_add(*offset as i64) as u32)
        }
        Instruction::Jump {
            target: Operand::Imm(v),
            ..
        } => Some(*v),
        Instruction::Loop { offset, .. } => Some((dec.pc & !3).wrapping_add(*offset)),
        _ => None,
    }
}

fn hex_dump(raw: &[u8], insn_size: u8) -> String {
    let mut out = String::new();
    let split = insn_size as usize;
    for (i, b) in raw.iter().enumerate() {
        if i == split && split < raw.len() {
            out.push(' ');
        }
        out.push_str(&format!("{:02X}", b));
    }
    out
}
