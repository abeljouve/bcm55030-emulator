use crate::cpu::condition::ConditionCode;
use crate::cpu::exception::Exception;
use crate::decoder::fields::*;
use crate::decoder::instruction::*;
use crate::decoder::InstructionFetch;

/// Decode a 32-bit instruction word
pub fn decode_32bit(
    word: u32,
    pc: u32,
    mem: &dyn InstructionFetch,
) -> Result<DecodedInstruction, Exception> {
    let major = extract_bits(word, 31, 27) as u8;
    match major {
        0x00 => decode_branch(word, pc),
        0x01 => decode_branch_link_or_compare(word, pc, mem),
        0x02 => decode_load(word, pc, mem),
        0x03 => decode_store(word, pc, mem),
        0x04 => decode_general_ops(word, pc, mem),
        0x05 => decode_extension_ops(word, pc, mem),
        _ => Err(Exception::InstructionError { address: pc }),
    }
}

/// Check if any source register is LIMM (r62) and fetch it
fn resolve_limm(
    b_reg: u8,
    c_reg: u8,
    pc: u32,
    insn_size: u8,
    mem: &dyn InstructionFetch,
) -> Result<(Option<u32>, bool), Exception> {
    let needs_limm = b_reg == 62 || c_reg == 62;
    if needs_limm {
        let limm_addr = pc + insn_size as u32;
        let limm = mem.fetch_word(limm_addr)?;
        Ok((Some(limm), true))
    } else {
        Ok((None, false))
    }
}

/// Resolve an operand: if reg is 62, use LIMM; otherwise Reg
fn resolve_operand(reg: u8, limm: Option<u32>) -> Operand {
    if reg == 62 {
        Operand::Imm(limm.unwrap_or(0))
    } else {
        Operand::Reg(reg)
    }
}

// ============== Major 0x00: Branch ==============

fn decode_branch(word: u32, pc: u32) -> Result<DecodedInstruction, Exception> {
    let sub = (word >> 16) & 1; // bit[16]
    if sub == 0 {
        // Bcc: conditional branch, 21-bit signed offset
        let s_low = extract_bits(word, 26, 17);  // S[10:1]
        let s_high = extract_bits(word, 15, 6);  // S[20:11]
        let raw_offset = (s_high << 10) | s_low; // S[20:1]
        let offset = sign_extend(raw_offset << 1, 21); // 21-bit signed, already shifted by 1

        let n = extract_n_bit(word);
        let q = extract_condition_q(word);
        let cc = ConditionCode::from_u8(q)
            .ok_or(Exception::InstructionError { address: pc })?;

        Ok(DecodedInstruction {
            inst: Instruction::Branch {
                offset,
                cc: Some(cc),
                delay: if n { DelayMode::Delay } else { DelayMode::NoDelay },
                link: false,
            },
            size: 4,
            has_limm: false,
            pc,
        })
    } else {
        // B far: unconditional, 25-bit signed offset
        let s_low = extract_bits(word, 26, 17);  // S[10:1]
        let s_high = extract_bits(word, 15, 6);  // S[20:11]
        let t = extract_bits(word, 3, 0);         // S[24:21]
        let raw_offset = (t << 20) | (s_high << 10) | s_low; // S[24:1]
        let offset = sign_extend(raw_offset << 1, 25);

        let n = extract_n_bit(word);

        // bit[4] = R, must be 0
        if (word >> 4) & 1 != 0 {
            return Err(Exception::InstructionError { address: pc });
        }

        Ok(DecodedInstruction {
            inst: Instruction::Branch {
                offset,
                cc: None, // unconditional
                delay: if n { DelayMode::Delay } else { DelayMode::NoDelay },
                link: false,
            },
            size: 4,
            has_limm: false,
            pc,
        })
    }
}

// ============== Major 0x01: BL / BRcc ==============

fn decode_branch_link_or_compare(
    word: u32,
    pc: u32,
    mem: &dyn InstructionFetch,
) -> Result<DecodedInstruction, Exception> {
    let bit17 = (word >> 17) & 1;
    let bit16 = (word >> 16) & 1;

    if bit16 == 1 {
        // bit[16]=1: BRcc (branch on register compare)
        // bit[4] distinguishes reg-reg (0) vs reg-imm (1)
        decode_branch_compare(word, pc, mem)
    } else {
        // bit[16]=0: branch-and-link
        if bit17 == 0 {
            // BLcc: conditional, 21-bit offset (32-bit aligned target)
            // bit[17]=0, bit[16]=0
            // bits[26:18] = S[10:2] (9 bits), bits[15:6] = S[20:11] (10 bits)
            let s_low = extract_bits(word, 26, 18);   // S[10:2], 9 bits
            let s_high = extract_bits(word, 15, 6);    // S[20:11], 10 bits
            let raw = (s_high << 9) | s_low;
            let offset = sign_extend(raw << 2, 21);

            let n = extract_n_bit(word);
            let q = extract_condition_q(word);
            let cc = ConditionCode::from_u8(q)
                .ok_or(Exception::InstructionError { address: pc })?;

            Ok(DecodedInstruction {
                inst: Instruction::Branch {
                    offset,
                    cc: Some(cc),
                    delay: if n { DelayMode::Delay } else { DelayMode::NoDelay },
                    link: true,
                },
                size: 4,
                has_limm: false,
                pc,
            })
        } else {
            // BL far unconditional: 25-bit offset, 32-bit aligned target
            // bit[17]=1, bit[16]=0
            // bits[26:18] = S[10:2] (9 bits), bits[15:6] = S[20:11] (10 bits), bits[3:0] = S[24:21]
            let s_low = extract_bits(word, 26, 18);   // S[10:2], 9 bits
            let s_mid = extract_bits(word, 15, 6);     // S[20:11], 10 bits
            let t = extract_bits(word, 3, 0);           // S[24:21], 4 bits
            let raw = (t << 19) | (s_mid << 9) | s_low;
            let offset = sign_extend(raw << 2, 25);

            let n = extract_n_bit(word);

            Ok(DecodedInstruction {
                inst: Instruction::Branch {
                    offset,
                    cc: None,
                    delay: if n { DelayMode::Delay } else { DelayMode::NoDelay },
                    link: true,
                },
                size: 4,
                has_limm: false,
                pc,
            })
        }
    }
}

fn decode_branch_compare(
    word: u32,
    pc: u32,
    mem: &dyn InstructionFetch,
) -> Result<DecodedInstruction, Exception> {
    let b_reg = extract_b_reg(word);
    let n = extract_n_bit(word);
    let sub_i = extract_bits(word, 3, 0) as u8;
    let is_imm = (word >> 4) & 1 != 0; // bit[4]: 0=reg, 1=imm

    // Offset: S[7:1] at bits[23:17], S8 at bit[15]
    let s_low = extract_bits(word, 23, 17); // S[7:1]
    let s8 = extract_bits(word, 15, 15);    // S8
    let raw = (s8 << 8) | (s_low << 1);     // S[8:0], already *2 for 16-bit alignment
    let offset = sign_extend(raw, 9);

    let kind = BrCompareKind::from_u8(sub_i)
        .ok_or(Exception::InstructionError { address: pc })?;

    let src1 = Operand::Reg(b_reg);
    let (src2, has_limm) = if is_imm {
        let u6 = extract_bits(word, 11, 6);
        (Operand::Imm(u6), false)
    } else {
        let c_reg = extract_c_reg(word);
        if c_reg == 62 {
            let limm = mem.fetch_word(pc + 4)?;
            (Operand::Imm(limm), true)
        } else {
            (Operand::Reg(c_reg), false)
        }
    };

    // Also handle LIMM for B register
    let (src1, has_limm) = if b_reg == 62 {
        let limm = mem.fetch_word(pc + 4)?;
        (Operand::Imm(limm), true)
    } else {
        (src1, has_limm)
    };

    Ok(DecodedInstruction {
        inst: Instruction::BranchCompare {
            kind,
            src1,
            src2,
            offset,
            delay: if n { DelayMode::Delay } else { DelayMode::NoDelay },
        },
        size: 4,
        has_limm,
        pc,
    })
}

// ============== Major 0x02: Load ==============

fn decode_load(
    word: u32,
    pc: u32,
    mem: &dyn InstructionFetch,
) -> Result<DecodedInstruction, Exception> {
    let b_reg = extract_b_reg(word);
    let a_reg = extract_a_reg(word);
    let di = (word >> 11) & 1 != 0;
    let aa = extract_bits(word, 10, 9) as u8;
    let zz = extract_bits(word, 8, 7) as u8;
    let x = (word >> 6) & 1 != 0;

    // Signed 9-bit offset: S[7:0] at bits[23:16], S8 at bit[15]
    let s_low = extract_bits(word, 23, 16);
    let s8 = extract_bits(word, 15, 15);
    let raw = (s8 << 8) | s_low;
    let offset_val = sign_extend(raw, 9) as u32;

    if zz == 3 {
        return Err(Exception::InstructionError { address: pc });
    }

    let data_size = match zz {
        0 => DataSize::Word,
        1 => DataSize::Byte,
        2 => DataSize::HalfWord,
        _ => unreachable!(),
    };

    let writeback = match aa {
        0 => WritebackMode::None,
        1 => WritebackMode::PreWrite,
        2 => WritebackMode::PostWrite,
        3 => WritebackMode::Scaled,
        _ => unreachable!(),
    };

    // Check: incrementing modes with LIMM base are illegal
    if b_reg == 62 && (aa == 1 || aa == 2) {
        return Err(Exception::InstructionError { address: pc });
    }

    // Check: LP_COUNT as destination
    if a_reg == 60 {
        return Err(Exception::InstructionError { address: pc });
    }

    // Check: extension regs (32-59) or PCL (63) as destination
    if (a_reg >= 32 && a_reg <= 59) || a_reg == 63 {
        return Err(Exception::InstructionError { address: pc });
    }

    // LIMM for base register
    let (base, has_limm) = if b_reg == 62 {
        let limm = mem.fetch_word(pc + 4)?;
        (Operand::Imm(limm), true)
    } else {
        (Operand::Reg(b_reg), false)
    };

    let dst = if a_reg == 62 {
        Operand::None // PREFETCH (discard result)
    } else {
        Operand::Reg(a_reg)
    };

    Ok(DecodedInstruction {
        inst: Instruction::Load {
            dst,
            base,
            offset: Operand::Imm(offset_val),
            data_size,
            sign_extend: x,
            writeback,
            cache_bypass: di,
        },
        size: 4,
        has_limm,
        pc,
    })
}

// ============== Major 0x03: Store ==============

fn decode_store(
    word: u32,
    pc: u32,
    mem: &dyn InstructionFetch,
) -> Result<DecodedInstruction, Exception> {
    let b_reg = extract_b_reg(word);
    let c_reg = extract_c_reg(word);
    let di = (word >> 5) & 1 != 0;
    let aa = extract_bits(word, 4, 3) as u8;
    let zz = extract_bits(word, 2, 1) as u8;

    // Signed 9-bit offset: S[7:0] at bits[23:16], S8 at bit[15]
    let s_low = extract_bits(word, 23, 16);
    let s8 = extract_bits(word, 15, 15);
    let raw = (s8 << 8) | s_low;
    let offset_val = sign_extend(raw, 9) as u32;

    if zz == 3 {
        return Err(Exception::InstructionError { address: pc });
    }

    let data_size = match zz {
        0 => DataSize::Word,
        1 => DataSize::Byte,
        2 => DataSize::HalfWord,
        _ => unreachable!(),
    };

    let writeback = match aa {
        0 => WritebackMode::None,
        1 => WritebackMode::PreWrite,
        2 => WritebackMode::PostWrite,
        3 => WritebackMode::Scaled,
        _ => unreachable!(),
    };

    // Check: incrementing modes with LIMM base are illegal
    if b_reg == 62 && (aa == 1 || aa == 2) {
        return Err(Exception::InstructionError { address: pc });
    }

    let (limm_val, has_limm) = resolve_limm(b_reg, c_reg, pc, 4, mem)?;
    let base = resolve_operand(b_reg, limm_val);
    let src = resolve_operand(c_reg, limm_val);

    Ok(DecodedInstruction {
        inst: Instruction::Store {
            src,
            base,
            offset: Operand::Imm(offset_val),
            data_size,
            writeback,
            cache_bypass: di,
        },
        size: 4,
        has_limm,
        pc,
    })
}

// ============== Major 0x04: General Operations ==============

fn decode_general_ops(
    word: u32,
    pc: u32,
    mem: &dyn InstructionFetch,
) -> Result<DecodedInstruction, Exception> {
    let p = extract_p_field(word);
    let sub = extract_subopcode_04(word);

    // Check for LD register-register format: bits[21:20]=11, bit[19]=0
    // bits[23:22]=AA (address mode), NOT the P field for this encoding
    if (sub >> 4) == 3 && (sub & 0x08) == 0 {
        // LD register-register 0x04,[0x30-0x37] with any AA mode
        return decode_load_reg_reg(word, pc, mem);
    }

    // Single operand instructions (sub=0x2F) — must be checked before the 0x20+ range
    if sub == 0x2F {
        return decode_single_op(word, pc, p, mem);
    }

    // Special format instructions 0x20-0x3F
    if sub >= 0x20 {
        return decode_special_ops(word, pc, sub, mem);
    }

    // Normal ALU ops (sub 0x00-0x1D)
    let alu_op = AluOp::from_u8(sub)
        .ok_or(Exception::InstructionError { address: pc })?;

    decode_alu_op(word, pc, p, alu_op, mem)
}

fn decode_alu_op(
    word: u32,
    pc: u32,
    p: u8,
    op: AluOp,
    mem: &dyn InstructionFetch,
) -> Result<DecodedInstruction, Exception> {
    let b_reg = extract_b_reg(word);
    let f = extract_f_bit(word);

    match p {
        0b00 => {
            // REG_REG: A = B op C
            let c_reg = extract_c_reg(word);
            let a_reg = extract_a_reg(word);

            // For MOV/test-only, B is destination-only (not source), so B=62 means
            // LP_COUNT register, NOT LIMM. Only check C for LIMM in those cases.
            let (limm_val, has_limm) = if op.is_mov() || op.is_test_only() {
                let needs = c_reg == 62;
                if needs {
                    let limm = mem.fetch_word(pc + 4)?;
                    (Some(limm), true)
                } else {
                    (None, false)
                }
            } else {
                resolve_limm(b_reg, c_reg, pc, 4, mem)?
            };
            let src1 = resolve_operand(b_reg, limm_val);
            let src2 = resolve_operand(c_reg, limm_val);
            let _dst_unused = if op.is_test_only() {
                Operand::None
            } else if op.is_mov() {
                resolve_operand(b_reg, limm_val)
            } else {
                resolve_operand(a_reg, limm_val)
            };

            // For MOV: dst=B, src=C (no src1 from B perspective)
            let (final_dst, final_src1, final_src2) = if op.is_mov() {
                (Operand::Reg(b_reg), Operand::None, resolve_operand(c_reg, limm_val))
            } else if op.is_test_only() {
                (Operand::None, resolve_operand(b_reg, limm_val), resolve_operand(c_reg, limm_val))
            } else {
                (resolve_operand(a_reg, limm_val), src1, src2)
            };

            Ok(DecodedInstruction {
                inst: Instruction::Alu {
                    op,
                    dst: final_dst,
                    src1: final_src1,
                    src2: final_src2,
                    set_flags: f,
                    cc: None,
                },
                size: 4,
                has_limm,
                pc,
            })
        }
        0b01 => {
            // REG_U6IMM: A = B op U6
            let u6 = extract_u6(word);
            let a_reg = extract_a_reg(word);

            // For MOV, B is destination-only — B=62 does NOT mean LIMM.
            let has_limm = if op.is_mov() { false } else { b_reg == 62 };
            let limm_val = if has_limm {
                Some(mem.fetch_word(pc + 4)?)
            } else {
                None
            };

            let (dst, src1, src2) = if op.is_mov() {
                (Operand::Reg(b_reg), Operand::None, Operand::Imm(u6))
            } else if op.is_test_only() {
                (Operand::None, resolve_operand(b_reg, limm_val), Operand::Imm(u6))
            } else {
                (resolve_operand(a_reg, limm_val), resolve_operand(b_reg, limm_val), Operand::Imm(u6))
            };

            Ok(DecodedInstruction {
                inst: Instruction::Alu {
                    op,
                    dst,
                    src1,
                    src2,
                    set_flags: f,
                    cc: None,
                },
                size: 4,
                has_limm,
                pc,
            })
        }
        0b10 => {
            // REG_S12IMM: B = B op S12
            let s12 = extract_s12(word) as u32;

            // For MOV, B is destination-only — B=62 does NOT mean LIMM.
            let has_limm = if op.is_mov() { false } else { b_reg == 62 };
            let limm_val = if has_limm {
                Some(mem.fetch_word(pc + 4)?)
            } else {
                None
            };

            let (dst, src1, src2) = if op.is_mov() {
                (Operand::Reg(b_reg), Operand::None, Operand::Imm(s12))
            } else if op.is_test_only() {
                (Operand::None, resolve_operand(b_reg, limm_val), Operand::Imm(s12))
            } else {
                (Operand::Reg(b_reg), resolve_operand(b_reg, limm_val), Operand::Imm(s12))
            };

            Ok(DecodedInstruction {
                inst: Instruction::Alu {
                    op,
                    dst,
                    src1,
                    src2,
                    set_flags: f,
                    cc: None,
                },
                size: 4,
                has_limm,
                pc,
            })
        }
        0b11 => {
            // COND_REG or COND_U6IMM
            let m = (word >> 5) & 1;
            let q = extract_condition_q(word);
            let cc = ConditionCode::from_u8(q)
                .ok_or(Exception::InstructionError { address: pc })?;

            if m == 0 {
                // COND_REG: B = B op C
                let c_reg = extract_c_reg(word);
                // For MOV/test-only, B is dest-only, B=62 means LP_COUNT not LIMM
                let (limm_val, has_limm) = if op.is_mov() || op.is_test_only() {
                    let needs = c_reg == 62;
                    if needs {
                        let limm = mem.fetch_word(pc + 4)?;
                        (Some(limm), true)
                    } else {
                        (None, false)
                    }
                } else {
                    resolve_limm(b_reg, c_reg, pc, 4, mem)?
                };

                let (dst, src1, src2) = if op.is_mov() {
                    (Operand::Reg(b_reg), Operand::None, resolve_operand(c_reg, limm_val))
                } else if op.is_test_only() {
                    (Operand::None, resolve_operand(b_reg, limm_val), resolve_operand(c_reg, limm_val))
                } else {
                    (Operand::Reg(b_reg), resolve_operand(b_reg, limm_val), resolve_operand(c_reg, limm_val))
                };

                Ok(DecodedInstruction {
                    inst: Instruction::Alu {
                        op,
                        dst,
                        src1,
                        src2,
                        set_flags: f,
                        cc: Some(cc),
                    },
                    size: 4,
                    has_limm,
                    pc,
                })
            } else {
                // COND_U6IMM: B = B op U6
                let u6 = extract_u6(word);

                // For MOV, B is destination-only — B=62 does NOT mean LIMM.
                let has_limm = if op.is_mov() { false } else { b_reg == 62 };
                let limm_val = if has_limm {
                    Some(mem.fetch_word(pc + 4)?)
                } else {
                    None
                };

                let (dst, src1, src2) = if op.is_mov() {
                    (Operand::Reg(b_reg), Operand::None, Operand::Imm(u6))
                } else if op.is_test_only() {
                    (Operand::None, resolve_operand(b_reg, limm_val), Operand::Imm(u6))
                } else {
                    (Operand::Reg(b_reg), resolve_operand(b_reg, limm_val), Operand::Imm(u6))
                };

                Ok(DecodedInstruction {
                    inst: Instruction::Alu {
                        op,
                        dst,
                        src1,
                        src2,
                        set_flags: f,
                        cc: Some(cc),
                    },
                    size: 4,
                    has_limm,
                    pc,
                })
            }
        }
        _ => unreachable!(),
    }
}

fn decode_single_op(
    word: u32,
    pc: u32,
    p: u8,
    mem: &dyn InstructionFetch,
) -> Result<DecodedInstruction, Exception> {
    // P=10 is invalid for single-op
    if p == 0b10 {
        return Err(Exception::InstructionError { address: pc });
    }

    let b_reg = extract_b_reg(word);
    let f = extract_f_bit(word);
    let a_reg = extract_a_reg(word); // sub-opcode2 = A field

    // Check for zero-operand (sub-opcode2=0x3F)
    if a_reg == 0x3F {
        return decode_zero_op(word, pc, b_reg);
    }

    // Map sub-opcode2 per Table 61
    let sop = match a_reg {
        0x00 => SingleOp::Asl,
        0x01 => SingleOp::Asr,
        0x02 => SingleOp::Lsr,
        0x03 => SingleOp::Ror,
        0x04 => SingleOp::Rrc,
        0x05 => SingleOp::Sexb,
        0x06 => SingleOp::Sexw,
        0x07 => SingleOp::Extb,
        0x08 => SingleOp::Extw,
        0x09 => SingleOp::Abs,
        0x0A => SingleOp::Not,
        0x0B => SingleOp::Rlc,
        0x0C => SingleOp::Ex,
        _ => return Err(Exception::InstructionError { address: pc }),
    };

    match p {
        0b00 => {
            // B <- op(C)
            let c_reg = extract_c_reg(word);
            let has_limm = c_reg == 62;
            let limm_val = if has_limm {
                Some(mem.fetch_word(pc + 4)?)
            } else {
                None
            };

            Ok(DecodedInstruction {
                inst: Instruction::SingleOp {
                    op: sop,
                    dst: Operand::Reg(b_reg),
                    src: resolve_operand(c_reg, limm_val),
                    set_flags: f,
                    cc: None,
                },
                size: 4,
                has_limm,
                pc,
            })
        }
        0b01 => {
            // B <- op(U6)
            let u6 = extract_u6(word);
            Ok(DecodedInstruction {
                inst: Instruction::SingleOp {
                    op: sop,
                    dst: Operand::Reg(b_reg),
                    src: Operand::Imm(u6),
                    set_flags: f,
                    cc: None,
                },
                size: 4,
                has_limm: false,
                pc,
            })
        }
        0b11 => {
            // Conditional: B <- op(C) or B <- op(U6)
            let m = (word >> 5) & 1;
            let q = extract_condition_q(word);
            let cc = ConditionCode::from_u8(q)
                .ok_or(Exception::InstructionError { address: pc })?;

            if m == 0 {
                let c_reg = extract_c_reg(word);
                let has_limm = c_reg == 62;
                let limm_val = if has_limm {
                    Some(mem.fetch_word(pc + 4)?)
                } else {
                    None
                };
                Ok(DecodedInstruction {
                    inst: Instruction::SingleOp {
                        op: sop,
                        dst: Operand::Reg(b_reg),
                        src: resolve_operand(c_reg, limm_val),
                        set_flags: f,
                        cc: Some(cc),
                    },
                    size: 4,
                    has_limm,
                    pc,
                })
            } else {
                let u6 = extract_u6(word);
                Ok(DecodedInstruction {
                    inst: Instruction::SingleOp {
                        op: sop,
                        dst: Operand::Reg(b_reg),
                        src: Operand::Imm(u6),
                        set_flags: f,
                        cc: Some(cc),
                    },
                    size: 4,
                    has_limm: false,
                    pc,
                })
            }
        }
        _ => Err(Exception::InstructionError { address: pc }),
    }
}

fn decode_zero_op(
    word: u32,
    pc: u32,
    b_field: u8,
) -> Result<DecodedInstruction, Exception> {
    // Table 62: sub-opcode3 = B field
    let zop = match b_field {
        0x01 => {
            let u6 = extract_u6(word) as u8;
            ZeroOp::Sleep { u6 }
        }
        0x02 => ZeroOp::Swi,
        0x03 => ZeroOp::Sync,
        0x04 => ZeroOp::Rtie,
        0x05 => ZeroOp::Brk,
        _ => return Err(Exception::InstructionError { address: pc }),
    };

    Ok(DecodedInstruction {
        inst: Instruction::ZeroOp(zop),
        size: 4,
        has_limm: false,
        pc,
    })
}

fn decode_special_ops(
    word: u32,
    pc: u32,
    sub: u8,
    mem: &dyn InstructionFetch,
) -> Result<DecodedInstruction, Exception> {
    match sub {
        // J/JL: 0x20-0x23
        0x20 | 0x21 | 0x22 | 0x23 => decode_jump(word, pc, sub, mem),
        // LP: 0x28
        0x28 => decode_loop(word, pc),
        // FLAG: 0x29
        0x29 => decode_flag(word, pc, mem),
        // LR: 0x2A
        0x2A => decode_lr(word, pc, mem),
        // SR: 0x2B
        0x2B => decode_sr(word, pc, mem),
        // Single-op (0x2F) handled before we get here
        // LD reg-reg (0x30-0x37) handled before we get here
        _ => Err(Exception::InstructionError { address: pc }),
    }
}

fn decode_jump(
    word: u32,
    pc: u32,
    sub: u8,
    mem: &dyn InstructionFetch,
) -> Result<DecodedInstruction, Exception> {
    let p = extract_p_field(word);
    let f = extract_f_bit(word);
    let link = sub == 0x22 || sub == 0x23;
    let delay = if sub == 0x21 || sub == 0x23 {
        DelayMode::Delay
    } else {
        DelayMode::NoDelay
    };

    match p {
        0b00 => {
            // REG: target = C
            let c_reg = extract_c_reg(word);
            let has_limm = c_reg == 62;
            let limm_val = if has_limm {
                Some(mem.fetch_word(pc + 4)?)
            } else {
                None
            };

            // Check F bit with ILINK registers
            let flag_restore = if f {
                if c_reg == 29 || c_reg == 30 {
                    true // J.F [ILINK1/2] restores STATUS32
                } else {
                    return Err(Exception::InstructionError { address: pc });
                }
            } else if c_reg == 29 || c_reg == 30 {
                // OBSERVED (silicon): on the BCM55030 ARC700, a bare
                // `j [ilink1/2]` (F=0) is ACCEPTED as a valid interrupt
                // return — silicon bisect 2026-05-18 showed a minimal ISR
                // ending in `j [ilink]` returning cleanly (IRQ serviced, no
                // fault), whereas textbook ARCompact would reject F=0 here.
                // See the design notes.
                //
                // INFERRED: this core's bare-ILINK jump also restores STATUS32
                // from STATUS32_L1/L2 (like the F form). The bisect proved the
                // jump returns cleanly but did not directly measure the
                // STATUS32 restore, and reference never emits F=0 (so it is
                // untestable from reference — the 4 MB scan found 0 plain
                // `j [ilink]`). We set flag_restore=true to keep interrupts
                // correctly re-enabled on return, matching the documented
                // silicon behavior.
                true
            } else {
                false
            };

            Ok(DecodedInstruction {
                inst: Instruction::Jump {
                    target: resolve_operand(c_reg, limm_val),
                    cc: None,
                    delay,
                    link,
                    flag_restore,
                },
                size: 4,
                has_limm,
                pc,
            })
        }
        0b01 => {
            // U6: target = u6
            let u6 = extract_u6(word);
            Ok(DecodedInstruction {
                inst: Instruction::Jump {
                    target: Operand::Imm(u6),
                    cc: None,
                    delay,
                    link,
                    flag_restore: false,
                },
                size: 4,
                has_limm: false,
                pc,
            })
        }
        0b10 => {
            // S12
            let s12 = extract_s12(word) as u32;
            Ok(DecodedInstruction {
                inst: Instruction::Jump {
                    target: Operand::Imm(s12),
                    cc: None,
                    delay,
                    link,
                    flag_restore: false,
                },
                size: 4,
                has_limm: false,
                pc,
            })
        }
        0b11 => {
            // Conditional
            let m = (word >> 5) & 1;
            let q = extract_condition_q(word);
            let cc = ConditionCode::from_u8(q)
                .ok_or(Exception::InstructionError { address: pc })?;

            let (target, has_limm) = if m == 0 {
                let c_reg = extract_c_reg(word);
                let has_limm = c_reg == 62;
                let limm_val = if has_limm {
                    Some(mem.fetch_word(pc + 4)?)
                } else {
                    None
                };
                (resolve_operand(c_reg, limm_val), has_limm)
            } else {
                let u6 = extract_u6(word);
                (Operand::Imm(u6), false)
            };

            // Check F bit with ILINK (only in REG mode, m=0)
            // Using ILINK without F raises InstructionError
            let flag_restore = if f {
                if m == 0 {
                    if let Operand::Reg(r) = target {
                        if r == 29 || r == 30 {
                            true
                        } else {
                            return Err(Exception::InstructionError { address: pc });
                        }
                    } else {
                        return Err(Exception::InstructionError { address: pc });
                    }
                } else {
                    return Err(Exception::InstructionError { address: pc });
                }
            } else {
                // OBSERVED (silicon): bare `j<cc> [ilink1/2]` (F=0, REG form)
                // is ACCEPTED as a valid interrupt return on this core, not an
                // InstructionError. INFERRED: it also restores STATUS32 from
                // STATUS32_L1/L2 (flag_restore=true) — same rationale as the
                // unconditional p==0b00 arm above. See
                // the design notes.
                if m == 0 {
                    if let Operand::Reg(r) = target {
                        r == 29 || r == 30
                    } else {
                        false
                    }
                } else {
                    false
                }
            };

            Ok(DecodedInstruction {
                inst: Instruction::Jump {
                    target,
                    cc: Some(cc),
                    delay,
                    link,
                    flag_restore,
                },
                size: 4,
                has_limm,
                pc,
            })
        }
        _ => unreachable!(),
    }
}

fn decode_loop(word: u32, pc: u32) -> Result<DecodedInstruction, Exception> {
    let p = extract_p_field(word);

    let (offset, cc) = match p {
        0b01 => {
            // U6 immediate offset -> U7 (shifted left by 1 for 16-bit alignment)
            let u6 = extract_u6(word);
            (u6 << 1, None)
        }
        0b10 => {
            // S12 immediate offset -> S13 (shifted left by 1 for 16-bit alignment)
            let s12 = extract_s12(word) as u32;
            (s12 << 1, None)
        }
        0b11 => {
            // Conditional with U6 -> U7 (shifted left by 1 for 16-bit alignment)
            let u6 = extract_u6(word);
            let q = extract_condition_q(word);
            let cc = ConditionCode::from_u8(q)
                .ok_or(Exception::InstructionError { address: pc })?;
            (u6 << 1, Some(cc))
        }
        _ => return Err(Exception::InstructionError { address: pc }),
    };

    Ok(DecodedInstruction {
        inst: Instruction::Loop { offset, cc },
        size: 4,
        has_limm: false,
        pc,
    })
}

fn decode_flag(
    word: u32,
    pc: u32,
    mem: &dyn InstructionFetch,
) -> Result<DecodedInstruction, Exception> {
    let p = extract_p_field(word);

    match p {
        0b00 => {
            let c_reg = extract_c_reg(word);
            let has_limm = c_reg == 62;
            let limm_val = if has_limm {
                Some(mem.fetch_word(pc + 4)?)
            } else {
                None
            };
            Ok(DecodedInstruction {
                inst: Instruction::Flag {
                    src: resolve_operand(c_reg, limm_val),
                    cc: None,
                },
                size: 4,
                has_limm,
                pc,
            })
        }
        0b01 => {
            let u6 = extract_u6(word);
            Ok(DecodedInstruction {
                inst: Instruction::Flag {
                    src: Operand::Imm(u6),
                    cc: None,
                },
                size: 4,
                has_limm: false,
                pc,
            })
        }
        0b11 => {
            let m = (word >> 5) & 1;
            let q = extract_condition_q(word);
            let cc = ConditionCode::from_u8(q)
                .ok_or(Exception::InstructionError { address: pc })?;

            let (src, has_limm) = if m == 0 {
                let c_reg = extract_c_reg(word);
                let has_limm = c_reg == 62;
                let limm_val = if has_limm {
                    Some(mem.fetch_word(pc + 4)?)
                } else {
                    None
                };
                (resolve_operand(c_reg, limm_val), has_limm)
            } else {
                let u6 = extract_u6(word);
                (Operand::Imm(u6), false)
            };

            Ok(DecodedInstruction {
                inst: Instruction::Flag { src, cc: Some(cc) },
                size: 4,
                has_limm,
                pc,
            })
        }
        _ => Err(Exception::InstructionError { address: pc }),
    }
}

fn decode_lr(
    word: u32,
    pc: u32,
    mem: &dyn InstructionFetch,
) -> Result<DecodedInstruction, Exception> {
    let b_reg = extract_b_reg(word);
    let p = extract_p_field(word);

    match p {
        0b00 => {
            let c_reg = extract_c_reg(word);
            let has_limm = c_reg == 62;
            let limm_val = if has_limm {
                Some(mem.fetch_word(pc + 4)?)
            } else {
                None
            };
            Ok(DecodedInstruction {
                inst: Instruction::LoadAux {
                    dst: Operand::Reg(b_reg),
                    addr: resolve_operand(c_reg, limm_val),
                },
                size: 4,
                has_limm,
                pc,
            })
        }
        0b01 => {
            let u6 = extract_u6(word);
            Ok(DecodedInstruction {
                inst: Instruction::LoadAux {
                    dst: Operand::Reg(b_reg),
                    addr: Operand::Imm(u6),
                },
                size: 4,
                has_limm: false,
                pc,
            })
        }
        0b10 => {
            let s12 = extract_s12(word) as u32;
            Ok(DecodedInstruction {
                inst: Instruction::LoadAux {
                    dst: Operand::Reg(b_reg),
                    addr: Operand::Imm(s12),
                },
                size: 4,
                has_limm: false,
                pc,
            })
        }
        _ => Err(Exception::InstructionError { address: pc }),
    }
}

fn decode_sr(
    word: u32,
    pc: u32,
    mem: &dyn InstructionFetch,
) -> Result<DecodedInstruction, Exception> {
    let b_reg = extract_b_reg(word);
    let p = extract_p_field(word);

    let has_limm_b = b_reg == 62;

    match p {
        0b00 => {
            let c_reg = extract_c_reg(word);
            let (limm_val, has_limm) = resolve_limm(b_reg, c_reg, pc, 4, mem)?;
            Ok(DecodedInstruction {
                inst: Instruction::StoreAux {
                    src: resolve_operand(b_reg, limm_val),
                    addr: resolve_operand(c_reg, limm_val),
                },
                size: 4,
                has_limm,
                pc,
            })
        }
        0b01 => {
            let u6 = extract_u6(word);
            let limm_val = if has_limm_b {
                Some(mem.fetch_word(pc + 4)?)
            } else {
                None
            };
            Ok(DecodedInstruction {
                inst: Instruction::StoreAux {
                    src: resolve_operand(b_reg, limm_val),
                    addr: Operand::Imm(u6),
                },
                size: 4,
                has_limm: has_limm_b,
                pc,
            })
        }
        0b10 => {
            let s12 = extract_s12(word) as u32;
            let limm_val = if has_limm_b {
                Some(mem.fetch_word(pc + 4)?)
            } else {
                None
            };
            Ok(DecodedInstruction {
                inst: Instruction::StoreAux {
                    src: resolve_operand(b_reg, limm_val),
                    addr: Operand::Imm(s12),
                },
                size: 4,
                has_limm: has_limm_b,
                pc,
            })
        }
        _ => Err(Exception::InstructionError { address: pc }),
    }
}

fn decode_load_reg_reg(
    word: u32,
    pc: u32,
    mem: &dyn InstructionFetch,
) -> Result<DecodedInstruction, Exception> {
    let b_reg = extract_b_reg(word);
    let c_reg = extract_c_reg(word);
    let a_reg = extract_a_reg(word);

    let aa = extract_bits(word, 23, 22) as u8;
    let zz = extract_bits(word, 18, 17) as u8;
    let x = (word >> 16) & 1 != 0;
    let di = extract_f_bit(word); // bit[15] is Di in this format

    if zz == 3 {
        return Err(Exception::InstructionError { address: pc });
    }

    let data_size = match zz {
        0 => DataSize::Word,
        1 => DataSize::Byte,
        2 => DataSize::HalfWord,
        _ => unreachable!(),
    };

    let writeback = match aa {
        0 => WritebackMode::None,
        1 => WritebackMode::PreWrite,
        2 => WritebackMode::PostWrite,
        3 => WritebackMode::Scaled,
        _ => unreachable!(),
    };

    // Check destination restrictions
    if a_reg == 60 {
        return Err(Exception::InstructionError { address: pc });
    }
    if (a_reg >= 32 && a_reg <= 59) || a_reg == 63 {
        return Err(Exception::InstructionError { address: pc });
    }

    let (limm_val, has_limm) = resolve_limm(b_reg, c_reg, pc, 4, mem)?;
    let base = resolve_operand(b_reg, limm_val);
    let offset = resolve_operand(c_reg, limm_val);

    let dst = if a_reg == 62 {
        Operand::None
    } else {
        Operand::Reg(a_reg)
    };

    Ok(DecodedInstruction {
        inst: Instruction::Load {
            dst,
            base,
            offset,
            data_size,
            sign_extend: x,
            writeback,
            cache_bypass: di,
        },
        size: 4,
        has_limm,
        pc,
    })
}

// ============== Major 0x05: Extension ALU ==============

fn decode_extension_ops(
    word: u32,
    pc: u32,
    mem: &dyn InstructionFetch,
) -> Result<DecodedInstruction, Exception> {
    let sub = extract_subopcode_04(word);
    let p = extract_p_field(word);
    let b_reg = extract_b_reg(word);
    let f = extract_f_bit(word);

    // Single-operand extensions (sub=0x2F)
    if sub == 0x2F {
        let a_field = extract_a_reg(word);
        let ext_sop = match a_field {
            0x00 => ExtArithOp::Swap,
            0x01 => ExtArithOp::Norm,
            0x02 => ExtArithOp::Sat16,
            0x03 => ExtArithOp::Rnd16,
            0x04 => ExtArithOp::Abssw,
            0x05 => ExtArithOp::Abss,
            0x06 => ExtArithOp::Negsw,
            0x07 => ExtArithOp::Negs,
            0x08 => ExtArithOp::Normw,
            _ => return Err(Exception::InstructionError { address: pc }),
        };

        let (src, has_limm) = match p {
            0b00 => {
                let c_reg = extract_c_reg(word);
                let has_limm = c_reg == 62;
                let limm_val = if has_limm {
                    Some(mem.fetch_word(pc + 4)?)
                } else {
                    None
                };
                (resolve_operand(c_reg, limm_val), has_limm)
            }
            0b01 => {
                let u6 = extract_u6(word);
                (Operand::Imm(u6), false)
            }
            _ => return Err(Exception::InstructionError { address: pc }),
        };

        return Ok(DecodedInstruction {
            inst: Instruction::ExtArith {
                op: ext_sop,
                dst: Operand::Reg(b_reg),
                src1: src,
                src2: Operand::None,
                set_flags: f,
                cc: None,
            },
            size: 4,
            has_limm,
            pc,
        });
    }

    // Dual-operand extensions
    let ext_op = match sub {
        0x00 => ExtArithOp::Asl,
        0x01 => ExtArithOp::Lsr,
        0x02 => ExtArithOp::Asr,
        0x03 => ExtArithOp::Ror,
        // 0x04 (MUL64) and 0x05 (MULU64) are not supported on ARC 700
        0x04 | 0x05 => return Err(Exception::InstructionError { address: pc }),
        0x06 => ExtArithOp::Adds,
        0x07 => ExtArithOp::Subs,
        0x08 => ExtArithOp::Divaw,
        0x0A => ExtArithOp::Asls,
        0x0B => ExtArithOp::Asrs,
        0x28 => ExtArithOp::Addsdw,
        0x29 => ExtArithOp::Subsdw,
        _ => return Err(Exception::InstructionError { address: pc }),
    };

    // Use same decoding as ALU for operand formats
    let (dst, src1, src2, has_limm, cc) = match p {
        0b00 => {
            let c_reg = extract_c_reg(word);
            let a_reg = extract_a_reg(word);
            let (limm_val, has_limm) = resolve_limm(b_reg, c_reg, pc, 4, mem)?;
            (
                resolve_operand(a_reg, limm_val),
                resolve_operand(b_reg, limm_val),
                resolve_operand(c_reg, limm_val),
                has_limm,
                None,
            )
        }
        0b01 => {
            let u6 = extract_u6(word);
            let a_reg = extract_a_reg(word);
            let has_limm = b_reg == 62;
            let limm_val = if has_limm {
                Some(mem.fetch_word(pc + 4)?)
            } else {
                None
            };
            (
                resolve_operand(a_reg, limm_val),
                resolve_operand(b_reg, limm_val),
                Operand::Imm(u6),
                has_limm,
                None,
            )
        }
        0b10 => {
            let s12 = extract_s12(word) as u32;
            let has_limm = b_reg == 62;
            let limm_val = if has_limm {
                Some(mem.fetch_word(pc + 4)?)
            } else {
                None
            };
            (
                Operand::Reg(b_reg),
                resolve_operand(b_reg, limm_val),
                Operand::Imm(s12),
                has_limm,
                None,
            )
        }
        0b11 => {
            let m = (word >> 5) & 1;
            let q = extract_condition_q(word);
            let cc = ConditionCode::from_u8(q)
                .ok_or(Exception::InstructionError { address: pc })?;

            if m == 0 {
                let c_reg = extract_c_reg(word);
                let (limm_val, has_limm) = resolve_limm(b_reg, c_reg, pc, 4, mem)?;
                (
                    Operand::Reg(b_reg),
                    resolve_operand(b_reg, limm_val),
                    resolve_operand(c_reg, limm_val),
                    has_limm,
                    Some(cc),
                )
            } else {
                let u6 = extract_u6(word);
                let has_limm = b_reg == 62;
                let limm_val = if has_limm {
                    Some(mem.fetch_word(pc + 4)?)
                } else {
                    None
                };
                (
                    Operand::Reg(b_reg),
                    resolve_operand(b_reg, limm_val),
                    Operand::Imm(u6),
                    has_limm,
                    Some(cc),
                )
            }
        }
        _ => unreachable!(),
    };

    Ok(DecodedInstruction {
        inst: Instruction::ExtArith {
            op: ext_op,
            dst,
            src1,
            src2,
            set_flags: f,
            cc,
        },
        size: 4,
        has_limm,
        pc,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::decoder::ByteSliceFetch;

    #[test]
    fn mov_b62_u6_no_limm() {
        // MOV r62, 0: P=1, sub=0x0A, B=62, U6=0, F=0.
        // B=62 is destination-only for MOV → no LIMM fetch.
        let bytes: [u8; 8] = [0x26, 0x4A, 0x70, 0x00, 0xDE, 0xAD, 0xBE, 0xEF];
        let fetch = ByteSliceFetch::new(&bytes, 0);
        let decoded = decode_32bit(0x264A7000, 0, &fetch).unwrap();
        assert!(!decoded.has_limm, "MOV with B=62 in P=1 must not fetch LIMM");
        assert_eq!(decoded.size, 4);
    }

    #[test]
    fn mov_sp_limm_p00_first_instruction() {
        // Bug emu-mov-limm-first-instruction: MOV SP, 0x10000 au premier
        // mot après cold boot. Encodage 24 0A 3F 80 / 00 01 00 00.
        // P=00, sub=0x0A, B[5:3]=011 + B[2:0]=100 → B=28=SP ; C=62=LIMM.
        let bytes: [u8; 8] = [0x24, 0x0A, 0x3F, 0x80, 0x00, 0x01, 0x00, 0x00];
        let fetch = ByteSliceFetch::new(&bytes, 0);
        let dec = decode_32bit(0x240A3F80, 0, &fetch).unwrap();
        assert!(dec.has_limm, "MOV reg, LIMM en P=00 doit fetch LIMM");
        assert_eq!(dec.total_size(), 8);
        if let Instruction::Alu { op, dst, src2, .. } = dec.inst {
            assert_eq!(op, AluOp::Mov);
            assert_eq!(dst, Operand::Reg(28));
            assert_eq!(src2, Operand::Imm(0x10000));
        } else {
            panic!("attendu Instruction::Alu, obtenu {:?}", dec.inst);
        }
    }

    #[test]
    fn cmp_b62_u6_has_limm() {
        // CMP limm, 0: P=1, sub=0x0C, B=62, U6=0.
        // B=62 is source for CMP → LIMM fetch required.
        let bytes: [u8; 8] = [0x26, 0x4C, 0x70, 0x00, 0x00, 0x00, 0x00, 0x42];
        let fetch = ByteSliceFetch::new(&bytes, 0);
        let decoded = decode_32bit(0x264C7000, 0, &fetch).unwrap();
        assert!(decoded.has_limm, "CMP with B=62 in P=1 must fetch LIMM");
        assert_eq!(decoded.size, 4);
    }

    // BCM55030 silicon-faithful interrupt-return idiom.
    // See the design notes.

    #[test]
    fn rtie_decodes_as_zeroop_rtie() {
        // rtie = 0x246F003F. Decode must still LABEL it rtie (so the
        // disassembler can render it). The EXECUTION fault (vec=2) is tested
        // in the executor unit tests, not here.
        let bytes: [u8; 4] = [0x24, 0x6F, 0x00, 0x3F];
        let fetch = ByteSliceFetch::new(&bytes, 0);
        let dec = decode_32bit(0x246F003F, 0, &fetch).unwrap();
        assert!(
            matches!(dec.inst, Instruction::ZeroOp(ZeroOp::Rtie)),
            "0x246F003F must decode to ZeroOp::Rtie, got {:?}",
            dec.inst
        );
    }

    #[test]
    fn j_ilink1_bare_f0_accepted_with_flag_restore() {
        // Bare `j [ilink1]` = 0x20200740 (sub=0x20, p=00, c_reg=29, F=0).
        // OBSERVED on BCM55030: accepted as a valid IRQ return (NOT an
        // InstructionError). INFERRED: restores STATUS32 → flag_restore=true.
        let bytes: [u8; 4] = [0x20, 0x20, 0x07, 0x40];
        let fetch = ByteSliceFetch::new(&bytes, 0);
        let dec = decode_32bit(0x20200740, 0, &fetch)
            .expect("bare j [ilink1] (F=0) must decode, not error");
        if let Instruction::Jump {
            target,
            cc,
            link,
            flag_restore,
            ..
        } = dec.inst
        {
            assert_eq!(target, Operand::Reg(29), "target must be ILINK1 (r29)");
            assert!(cc.is_none(), "unconditional jump");
            assert!(!link, "j (not jl) must not link");
            assert!(flag_restore, "bare j [ilink1] must restore STATUS32");
        } else {
            panic!("expected Instruction::Jump, got {:?}", dec.inst);
        }
    }

    #[test]
    fn j_ilink2_bare_f0_accepted_with_flag_restore() {
        // Bare `j [ilink2]` = 0x20200780 (c_reg=30, F=0).
        let bytes: [u8; 4] = [0x20, 0x20, 0x07, 0x80];
        let fetch = ByteSliceFetch::new(&bytes, 0);
        let dec = decode_32bit(0x20200780, 0, &fetch)
            .expect("bare j [ilink2] (F=0) must decode, not error");
        if let Instruction::Jump {
            target,
            flag_restore,
            ..
        } = dec.inst
        {
            assert_eq!(target, Operand::Reg(30), "target must be ILINK2 (r30)");
            assert!(flag_restore, "bare j [ilink2] must restore STATUS32");
        } else {
            panic!("expected Instruction::Jump, got {:?}", dec.inst);
        }
    }

    #[test]
    fn j_f_ilink1_still_works() {
        // `j.f [ilink1]` = 0x20208740 (F=1) — the reference idiom, must be
        // unchanged: a Jump to r29 with flag_restore.
        let bytes: [u8; 4] = [0x20, 0x20, 0x87, 0x40];
        let fetch = ByteSliceFetch::new(&bytes, 0);
        let dec = decode_32bit(0x20208740, 0, &fetch)
            .expect("j.f [ilink1] must decode");
        if let Instruction::Jump {
            target,
            flag_restore,
            ..
        } = dec.inst
        {
            assert_eq!(target, Operand::Reg(29));
            assert!(flag_restore, "j.f [ilink1] restores STATUS32");
        } else {
            panic!("expected Instruction::Jump, got {:?}", dec.inst);
        }
    }
}
