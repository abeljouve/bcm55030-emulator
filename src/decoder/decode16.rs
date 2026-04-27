use crate::cpu::condition::ConditionCode;
use crate::cpu::exception::Exception;
use crate::cpu::registers::{REG_BLINK, REG_GP, REG_SP};
use crate::decoder::fields::*;
use crate::decoder::instruction::*;
use crate::decoder::InstructionFetch;

/// Decode a 16-bit instruction
pub fn decode_16bit(
    half: u16,
    pc: u32,
    mem: &dyn InstructionFetch,
) -> Result<DecodedInstruction, Exception> {
    let major = major_opcode(half);
    match major {
        0x0C => decode_0c_ld_add_reg(half, pc),
        0x0D => decode_0d_add_sub_shift_imm(half, pc),
        0x0E => decode_0e_mov_cmp_add_high(half, pc, mem),
        0x0F => decode_0f_general(half, pc),
        0x10 => decode_ld_offset(half, pc, DataSize::Word, false),   // LD_S C,[B,U7]
        0x11 => decode_ld_offset(half, pc, DataSize::Byte, false),   // LDB_S C,[B,U5]
        0x12 => decode_ld_offset(half, pc, DataSize::HalfWord, false), // LDW_S C,[B,U6]
        0x13 => decode_ld_offset(half, pc, DataSize::HalfWord, true),  // LDW_S.X C,[B,U6]
        0x14 => decode_st_offset(half, pc, DataSize::Word),     // ST_S C,[B,U7]
        0x15 => decode_st_offset(half, pc, DataSize::Byte),     // STB_S C,[B,U5]
        0x16 => decode_st_offset(half, pc, DataSize::HalfWord), // STW_S C,[B,U6]
        0x17 => decode_17_shift_sub_bit(half, pc),
        0x18 => decode_18_sp_based(half, pc),
        0x19 => decode_19_gp_relative(half, pc),
        0x1A => decode_1a_pcl_relative(half, pc),
        0x1B => decode_1b_mov_imm(half, pc),
        0x1C => decode_1c_add_cmp_imm(half, pc),
        0x1D => decode_1d_brcc(half, pc),
        0x1E => decode_1e_bcc(half, pc),
        0x1F => decode_1f_bl(half, pc),
        _ => Err(Exception::InstructionError { address: pc }),
    }
}

// ===== 0x0C: LD_S/LDB_S/LDW_S/ADD_S register-register =====

fn decode_0c_ld_add_reg(half: u16, pc: u32) -> Result<DecodedInstruction, Exception> {
    let b = map_16bit_reg(((half >> 8) & 7) as u8);
    let c = map_16bit_reg(((half >> 5) & 7) as u8);
    let i = (half >> 3) & 3;
    let a = map_16bit_reg((half & 7) as u8);

    let inst = match i {
        0x00 => Instruction::Load {
            dst: Operand::Reg(a), base: Operand::Reg(b), offset: Operand::Reg(c),
            data_size: DataSize::Word, sign_extend: false, writeback: WritebackMode::None, cache_bypass: false,
        },
        0x01 => Instruction::Load {
            dst: Operand::Reg(a), base: Operand::Reg(b), offset: Operand::Reg(c),
            data_size: DataSize::Byte, sign_extend: false, writeback: WritebackMode::None, cache_bypass: false,
        },
        0x02 => Instruction::Load {
            dst: Operand::Reg(a), base: Operand::Reg(b), offset: Operand::Reg(c),
            data_size: DataSize::HalfWord, sign_extend: false, writeback: WritebackMode::None, cache_bypass: false,
        },
        0x03 => Instruction::Alu {
            op: AluOp::Add, dst: Operand::Reg(a), src1: Operand::Reg(b),
            src2: Operand::Reg(c), set_flags: false, cc: None,
        },
        _ => unreachable!(),
    };

    Ok(DecodedInstruction { inst, size: 2, has_limm: false, pc })
}

// ===== 0x0D: ADD_S/SUB_S/ASL_S/ASR_S register-immediate =====

fn decode_0d_add_sub_shift_imm(half: u16, pc: u32) -> Result<DecodedInstruction, Exception> {
    let b = map_16bit_reg(((half >> 8) & 7) as u8);
    let c = map_16bit_reg(((half >> 5) & 7) as u8);
    let i = (half >> 3) & 3;
    let u3 = (half & 7) as u32;

    let inst = match i {
        0x00 => Instruction::Alu {
            op: AluOp::Add, dst: Operand::Reg(c), src1: Operand::Reg(b),
            src2: Operand::Imm(u3), set_flags: false, cc: None,
        },
        0x01 => Instruction::Alu {
            op: AluOp::Sub, dst: Operand::Reg(c), src1: Operand::Reg(b),
            src2: Operand::Imm(u3), set_flags: false, cc: None,
        },
        0x02 => Instruction::ExtArith {
            op: ExtArithOp::Asl, dst: Operand::Reg(c), src1: Operand::Reg(b),
            src2: Operand::Imm(u3), set_flags: false, cc: None,
        },
        0x03 => Instruction::ExtArith {
            op: ExtArithOp::Asr, dst: Operand::Reg(c), src1: Operand::Reg(b),
            src2: Operand::Imm(u3), set_flags: false, cc: None,
        },
        _ => unreachable!(),
    };

    Ok(DecodedInstruction { inst, size: 2, has_limm: false, pc })
}

// ===== 0x0E: MOV_S/CMP_S/ADD_S with high register =====

fn decode_0e_mov_cmp_add_high(
    half: u16,
    pc: u32,
    mem: &dyn InstructionFetch,
) -> Result<DecodedInstruction, Exception> {
    let b = map_16bit_reg(((half >> 8) & 7) as u8);
    let h = extract_h_reg_16(half);
    let i = (half >> 3) & 3;

    // Check for LIMM: if h == 62, need to read LIMM
    let (h_operand, has_limm) = if h == 62 {
        let limm = mem.fetch_word(pc + 2)?;
        (Operand::Imm(limm), true)
    } else {
        (Operand::Reg(h), false)
    };

    let inst = match i {
        0x00 => Instruction::Alu {
            op: AluOp::Add, dst: Operand::Reg(b), src1: Operand::Reg(b),
            src2: h_operand, set_flags: false, cc: None,
        },
        0x01 => Instruction::Alu {
            op: AluOp::Mov, dst: Operand::Reg(b), src1: Operand::None,
            src2: h_operand, set_flags: false, cc: None,
        },
        0x02 => Instruction::Alu {
            op: AluOp::Cmp, dst: Operand::None, src1: Operand::Reg(b),
            src2: h_operand, set_flags: true, cc: None,
        },
        0x03 => {
            // MOV_S H, B (destination is high register)
            if h == 63 {
                return Err(Exception::InstructionError { address: pc });
            }
            Instruction::Alu {
                op: AluOp::Mov, dst: Operand::Reg(h), src1: Operand::None,
                src2: Operand::Reg(b), set_flags: false, cc: None,
            }
        }
        _ => unreachable!(),
    };

    Ok(DecodedInstruction { inst, size: 2, has_limm, pc })
}

// ===== 0x0F: General 16-bit ops =====

fn decode_0f_general(half: u16, pc: u32) -> Result<DecodedInstruction, Exception> {
    let b_enc = ((half >> 8) & 7) as u8;
    let c_enc = ((half >> 5) & 7) as u8;
    let i = (half & 0x1F) as u8; // bits[4:0]

    // Zero operand: i=0x00, c=0x07, check b for sub-opcode3
    if i == 0x00 && c_enc == 0x07 {
        return decode_0f_zero_op(b_enc, pc);
    }

    // Single operand / jump: i=0x00
    if i == 0x00 {
        return decode_0f_single_op(b_enc, c_enc, pc);
    }

    let b = map_16bit_reg(b_enc);
    let c = map_16bit_reg(c_enc);

    let inst = match i {
        0x02 => Instruction::Alu {
            op: AluOp::Sub, dst: Operand::Reg(b), src1: Operand::Reg(b),
            src2: Operand::Reg(c), set_flags: false, cc: None,
        },
        0x04 => Instruction::Alu {
            op: AluOp::And, dst: Operand::Reg(b), src1: Operand::Reg(b),
            src2: Operand::Reg(c), set_flags: false, cc: None,
        },
        0x05 => Instruction::Alu {
            op: AluOp::Or, dst: Operand::Reg(b), src1: Operand::Reg(b),
            src2: Operand::Reg(c), set_flags: false, cc: None,
        },
        0x06 => Instruction::Alu {
            op: AluOp::Bic, dst: Operand::Reg(b), src1: Operand::Reg(b),
            src2: Operand::Reg(c), set_flags: false, cc: None,
        },
        0x07 => Instruction::Alu {
            op: AluOp::Xor, dst: Operand::Reg(b), src1: Operand::Reg(b),
            src2: Operand::Reg(c), set_flags: false, cc: None,
        },
        0x0B => Instruction::Alu {
            op: AluOp::Tst, dst: Operand::None, src1: Operand::Reg(b),
            src2: Operand::Reg(c), set_flags: true, cc: None,
        },
        // 0x0C: MUL64_S not supported on ARC 700
        0x0C => return Err(Exception::InstructionError { address: pc }),
        0x0D => Instruction::SingleOp {
            op: SingleOp::Sexb, dst: Operand::Reg(b), src: Operand::Reg(c),
            set_flags: false, cc: None,
        },
        0x0E => Instruction::SingleOp {
            op: SingleOp::Sexw, dst: Operand::Reg(b), src: Operand::Reg(c),
            set_flags: false, cc: None,
        },
        0x0F => Instruction::SingleOp {
            op: SingleOp::Extb, dst: Operand::Reg(b), src: Operand::Reg(c),
            set_flags: false, cc: None,
        },
        0x10 => Instruction::SingleOp {
            op: SingleOp::Extw, dst: Operand::Reg(b), src: Operand::Reg(c),
            set_flags: false, cc: None,
        },
        0x11 => Instruction::SingleOp {
            op: SingleOp::Abs, dst: Operand::Reg(b), src: Operand::Reg(c),
            set_flags: false, cc: None,
        },
        0x12 => Instruction::SingleOp {
            op: SingleOp::Not, dst: Operand::Reg(b), src: Operand::Reg(c),
            set_flags: false, cc: None,
        },
        0x13 => {
            // NEG_S B,C: B = 0 - C. Must use Sub (not Rsub which is src2-src1).
            Instruction::Alu {
                op: AluOp::Sub, dst: Operand::Reg(b), src1: Operand::Imm(0),
                src2: Operand::Reg(c), set_flags: false, cc: None,
            }
        }
        0x14 => Instruction::Alu {
            op: AluOp::Add1, dst: Operand::Reg(b), src1: Operand::Reg(b),
            src2: Operand::Reg(c), set_flags: false, cc: None,
        },
        0x15 => Instruction::Alu {
            op: AluOp::Add2, dst: Operand::Reg(b), src1: Operand::Reg(b),
            src2: Operand::Reg(c), set_flags: false, cc: None,
        },
        0x16 => Instruction::Alu {
            op: AluOp::Add3, dst: Operand::Reg(b), src1: Operand::Reg(b),
            src2: Operand::Reg(c), set_flags: false, cc: None,
        },
        // Multi-shift by register
        0x18 => Instruction::ExtArith {
            op: ExtArithOp::Asl, dst: Operand::Reg(b), src1: Operand::Reg(b),
            src2: Operand::Reg(c), set_flags: false, cc: None,
        },
        0x19 => Instruction::ExtArith {
            op: ExtArithOp::Lsr, dst: Operand::Reg(b), src1: Operand::Reg(b),
            src2: Operand::Reg(c), set_flags: false, cc: None,
        },
        0x1A => Instruction::ExtArith {
            op: ExtArithOp::Asr, dst: Operand::Reg(b), src1: Operand::Reg(b),
            src2: Operand::Reg(c), set_flags: false, cc: None,
        },
        // Single shift by 1
        0x1B => Instruction::SingleOp {
            op: SingleOp::Asl, dst: Operand::Reg(b), src: Operand::Reg(c),
            set_flags: false, cc: None,
        },
        0x1C => Instruction::SingleOp {
            op: SingleOp::Asr, dst: Operand::Reg(b), src: Operand::Reg(c),
            set_flags: false, cc: None,
        },
        0x1D => Instruction::SingleOp {
            op: SingleOp::Lsr, dst: Operand::Reg(b), src: Operand::Reg(c),
            set_flags: false, cc: None,
        },
        0x1E => {
            // TRAP_S u6
            let param = ((half >> 5) & 0x3F) as u8;
            Instruction::ZeroOp(ZeroOp::Trap { param })
        }
        0x1F => Instruction::ZeroOp(ZeroOp::Brk),
        _ => return Err(Exception::InstructionError { address: pc }),
    };

    Ok(DecodedInstruction { inst, size: 2, has_limm: false, pc })
}

fn decode_0f_single_op(b_enc: u8, c_enc: u8, pc: u32) -> Result<DecodedInstruction, Exception> {
    let b = map_16bit_reg(b_enc);
    let inst = match c_enc {
        0x00 => Instruction::Jump {
            target: Operand::Reg(b), cc: None, delay: DelayMode::NoDelay,
            link: false, flag_restore: false,
        },
        0x01 => Instruction::Jump {
            target: Operand::Reg(b), cc: None, delay: DelayMode::Delay,
            link: false, flag_restore: false,
        },
        0x02 => Instruction::Jump {
            target: Operand::Reg(b), cc: None, delay: DelayMode::NoDelay,
            link: true, flag_restore: false,
        },
        0x03 => Instruction::Jump {
            target: Operand::Reg(b), cc: None, delay: DelayMode::Delay,
            link: true, flag_restore: false,
        },
        0x06 => {
            // SUB_S.NE B, B, B (if Z==0, B <- 0)
            Instruction::Alu {
                op: AluOp::Sub, dst: Operand::Reg(b), src1: Operand::Reg(b),
                src2: Operand::Reg(b), set_flags: false, cc: Some(ConditionCode::NE),
            }
        }
        _ => return Err(Exception::InstructionError { address: pc }),
    };
    Ok(DecodedInstruction { inst, size: 2, has_limm: false, pc })
}

fn decode_0f_zero_op(b_enc: u8, pc: u32) -> Result<DecodedInstruction, Exception> {
    let inst = match b_enc {
        0x00 => Instruction::ZeroOp(ZeroOp::Nop),
        0x01 => return Err(Exception::InstructionError { address: pc }), // UNIMP_S
        0x04 => Instruction::Jump {
            target: Operand::Reg(REG_BLINK), cc: Some(ConditionCode::EQ),
            delay: DelayMode::NoDelay, link: false, flag_restore: false,
        },
        0x05 => Instruction::Jump {
            target: Operand::Reg(REG_BLINK), cc: Some(ConditionCode::NE),
            delay: DelayMode::NoDelay, link: false, flag_restore: false,
        },
        0x06 => Instruction::Jump {
            target: Operand::Reg(REG_BLINK), cc: None,
            delay: DelayMode::NoDelay, link: false, flag_restore: false,
        },
        0x07 => Instruction::Jump {
            target: Operand::Reg(REG_BLINK), cc: None,
            delay: DelayMode::Delay, link: false, flag_restore: false,
        },
        _ => return Err(Exception::InstructionError { address: pc }),
    };
    Ok(DecodedInstruction { inst, size: 2, has_limm: false, pc })
}

// ===== 0x10-0x16: Load/Store with offset =====

fn decode_ld_offset(
    half: u16,
    pc: u32,
    data_size: DataSize,
    sign_ext: bool,
) -> Result<DecodedInstruction, Exception> {
    let b = map_16bit_reg(((half >> 8) & 7) as u8);
    let c = map_16bit_reg(((half >> 5) & 7) as u8);
    let u5 = (half & 0x1F) as u32;

    // Scale offset by data size
    let offset = match data_size {
        DataSize::Word => u5 << 2,     // U7
        DataSize::HalfWord => u5 << 1, // U6
        DataSize::Byte => u5,           // U5
    };

    Ok(DecodedInstruction {
        inst: Instruction::Load {
            dst: Operand::Reg(c), base: Operand::Reg(b),
            offset: Operand::Imm(offset), data_size, sign_extend: sign_ext,
            writeback: WritebackMode::None, cache_bypass: false,
        },
        size: 2, has_limm: false, pc,
    })
}

fn decode_st_offset(
    half: u16,
    pc: u32,
    data_size: DataSize,
) -> Result<DecodedInstruction, Exception> {
    let b = map_16bit_reg(((half >> 8) & 7) as u8);
    let c = map_16bit_reg(((half >> 5) & 7) as u8);
    let u5 = (half & 0x1F) as u32;

    let offset = match data_size {
        DataSize::Word => u5 << 2,
        DataSize::HalfWord => u5 << 1,
        DataSize::Byte => u5,
    };

    Ok(DecodedInstruction {
        inst: Instruction::Store {
            src: Operand::Reg(c), base: Operand::Reg(b),
            offset: Operand::Imm(offset), data_size,
            writeback: WritebackMode::None, cache_bypass: false,
        },
        size: 2, has_limm: false, pc,
    })
}

// ===== 0x17: Shift/Subtract/Bit immediate =====

fn decode_17_shift_sub_bit(half: u16, pc: u32) -> Result<DecodedInstruction, Exception> {
    let b = map_16bit_reg(((half >> 8) & 7) as u8);
    let i = ((half >> 5) & 7) as u8;
    let u5 = (half & 0x1F) as u32;

    let inst = match i {
        0x00 => Instruction::ExtArith {
            op: ExtArithOp::Asl, dst: Operand::Reg(b), src1: Operand::Reg(b),
            src2: Operand::Imm(u5), set_flags: false, cc: None,
        },
        0x01 => Instruction::ExtArith {
            op: ExtArithOp::Lsr, dst: Operand::Reg(b), src1: Operand::Reg(b),
            src2: Operand::Imm(u5), set_flags: false, cc: None,
        },
        0x02 => Instruction::ExtArith {
            op: ExtArithOp::Asr, dst: Operand::Reg(b), src1: Operand::Reg(b),
            src2: Operand::Imm(u5), set_flags: false, cc: None,
        },
        0x03 => Instruction::Alu {
            op: AluOp::Sub, dst: Operand::Reg(b), src1: Operand::Reg(b),
            src2: Operand::Imm(u5), set_flags: false, cc: None,
        },
        0x04 => Instruction::Alu {
            op: AluOp::Bset, dst: Operand::Reg(b), src1: Operand::Reg(b),
            src2: Operand::Imm(u5), set_flags: false, cc: None,
        },
        0x05 => Instruction::Alu {
            op: AluOp::Bclr, dst: Operand::Reg(b), src1: Operand::Reg(b),
            src2: Operand::Imm(u5), set_flags: false, cc: None,
        },
        0x06 => Instruction::Alu {
            op: AluOp::Bmsk, dst: Operand::Reg(b), src1: Operand::Reg(b),
            src2: Operand::Imm(u5), set_flags: false, cc: None,
        },
        0x07 => Instruction::Alu {
            op: AluOp::Btst, dst: Operand::None, src1: Operand::Reg(b),
            src2: Operand::Imm(u5), set_flags: true, cc: None,
        },
        _ => return Err(Exception::InstructionError { address: pc }),
    };

    Ok(DecodedInstruction { inst, size: 2, has_limm: false, pc })
}

// ===== 0x18: SP-based instructions =====

fn decode_18_sp_based(half: u16, pc: u32) -> Result<DecodedInstruction, Exception> {
    let b = map_16bit_reg(((half >> 8) & 7) as u8);
    let i = ((half >> 5) & 7) as u8;
    let u5 = (half & 0x1F) as u32;
    let u7 = u5 << 2; // 32-bit aligned offset

    match i {
        0x00 => Ok(DecodedInstruction {
            inst: Instruction::Load {
                dst: Operand::Reg(b), base: Operand::Reg(REG_SP),
                offset: Operand::Imm(u7), data_size: DataSize::Word,
                sign_extend: false, writeback: WritebackMode::None, cache_bypass: false,
            },
            size: 2, has_limm: false, pc,
        }),
        0x01 => Ok(DecodedInstruction {
            inst: Instruction::Load {
                dst: Operand::Reg(b), base: Operand::Reg(REG_SP),
                offset: Operand::Imm(u7), data_size: DataSize::Byte,
                sign_extend: false, writeback: WritebackMode::None, cache_bypass: false,
            },
            size: 2, has_limm: false, pc,
        }),
        0x02 => Ok(DecodedInstruction {
            inst: Instruction::Store {
                src: Operand::Reg(b), base: Operand::Reg(REG_SP),
                offset: Operand::Imm(u7), data_size: DataSize::Word,
                writeback: WritebackMode::None, cache_bypass: false,
            },
            size: 2, has_limm: false, pc,
        }),
        0x03 => Ok(DecodedInstruction {
            inst: Instruction::Store {
                src: Operand::Reg(b), base: Operand::Reg(REG_SP),
                offset: Operand::Imm(u7), data_size: DataSize::Byte,
                writeback: WritebackMode::None, cache_bypass: false,
            },
            size: 2, has_limm: false, pc,
        }),
        0x04 => Ok(DecodedInstruction {
            inst: Instruction::Alu {
                op: AluOp::Add, dst: Operand::Reg(b), src1: Operand::Reg(REG_SP),
                src2: Operand::Imm(u7), set_flags: false, cc: None,
            },
            size: 2, has_limm: false, pc,
        }),
        0x05 => {
            // ADD_S SP,SP,U7 or SUB_S SP,SP,U7
            let b_enc = ((half >> 8) & 7) as u8;
            match b_enc {
                0x00 => Ok(DecodedInstruction {
                    inst: Instruction::Alu {
                        op: AluOp::Add, dst: Operand::Reg(REG_SP),
                        src1: Operand::Reg(REG_SP), src2: Operand::Imm(u7),
                        set_flags: false, cc: None,
                    },
                    size: 2, has_limm: false, pc,
                }),
                0x01 => Ok(DecodedInstruction {
                    inst: Instruction::Alu {
                        op: AluOp::Sub, dst: Operand::Reg(REG_SP),
                        src1: Operand::Reg(REG_SP), src2: Operand::Imm(u7),
                        set_flags: false, cc: None,
                    },
                    size: 2, has_limm: false, pc,
                }),
                _ => Err(Exception::InstructionError { address: pc }),
            }
        }
        0x06 => {
            // POP_S: u[4:0] selects variant
            let u_val = u5 as u8;
            if u_val == 0x01 {
                // POP_S b: load from [SP], SP += 4
                Ok(DecodedInstruction {
                    inst: Instruction::Load {
                        dst: Operand::Reg(b), base: Operand::Reg(REG_SP),
                        offset: Operand::Imm(4), data_size: DataSize::Word,
                        sign_extend: false, writeback: WritebackMode::PostWrite, cache_bypass: false,
                    },
                    size: 2, has_limm: false, pc,
                })
            } else if u_val == 0x11 {
                // POP_S BLINK
                Ok(DecodedInstruction {
                    inst: Instruction::Load {
                        dst: Operand::Reg(REG_BLINK), base: Operand::Reg(REG_SP),
                        offset: Operand::Imm(4), data_size: DataSize::Word,
                        sign_extend: false, writeback: WritebackMode::PostWrite, cache_bypass: false,
                    },
                    size: 2, has_limm: false, pc,
                })
            } else {
                Err(Exception::InstructionError { address: pc })
            }
        }
        0x07 => {
            // PUSH_S: u[4:0] selects variant
            let u_val = u5 as u8;
            if u_val == 0x01 {
                // PUSH_S b: SP -= 4, store b to [SP]
                Ok(DecodedInstruction {
                    inst: Instruction::Store {
                        src: Operand::Reg(b), base: Operand::Reg(REG_SP),
                        offset: Operand::Imm((-4i32) as u32), data_size: DataSize::Word,
                        writeback: WritebackMode::PreWrite, cache_bypass: false,
                    },
                    size: 2, has_limm: false, pc,
                })
            } else if u_val == 0x11 {
                // PUSH_S BLINK
                Ok(DecodedInstruction {
                    inst: Instruction::Store {
                        src: Operand::Reg(REG_BLINK), base: Operand::Reg(REG_SP),
                        offset: Operand::Imm((-4i32) as u32), data_size: DataSize::Word,
                        writeback: WritebackMode::PreWrite, cache_bypass: false,
                    },
                    size: 2, has_limm: false, pc,
                })
            } else {
                Err(Exception::InstructionError { address: pc })
            }
        }
        _ => Err(Exception::InstructionError { address: pc }),
    }
}

// ===== 0x19: GP-relative =====

fn decode_19_gp_relative(half: u16, pc: u32) -> Result<DecodedInstruction, Exception> {
    let i = ((half >> 9) & 3) as u8;
    let s9 = (half & 0x1FF) as u32;

    match i {
        0x00 => {
            let offset = s9 << 2; // S11, 32-bit aligned
            Ok(DecodedInstruction {
                inst: Instruction::Load {
                    dst: Operand::Reg(0), base: Operand::Reg(REG_GP),
                    offset: Operand::Imm(sign_extend_u32(offset, 11)),
                    data_size: DataSize::Word, sign_extend: false,
                    writeback: WritebackMode::None, cache_bypass: false,
                },
                size: 2, has_limm: false, pc,
            })
        }
        0x01 => {
            // S9, byte aligned
            Ok(DecodedInstruction {
                inst: Instruction::Load {
                    dst: Operand::Reg(0), base: Operand::Reg(REG_GP),
                    offset: Operand::Imm(sign_extend_u32(s9, 9)),
                    data_size: DataSize::Byte, sign_extend: false,
                    writeback: WritebackMode::None, cache_bypass: false,
                },
                size: 2, has_limm: false, pc,
            })
        }
        0x02 => {
            let offset = s9 << 1; // S10, 16-bit aligned
            Ok(DecodedInstruction {
                inst: Instruction::Load {
                    dst: Operand::Reg(0), base: Operand::Reg(REG_GP),
                    offset: Operand::Imm(sign_extend_u32(offset, 10)),
                    data_size: DataSize::HalfWord, sign_extend: false,
                    writeback: WritebackMode::None, cache_bypass: false,
                },
                size: 2, has_limm: false, pc,
            })
        }
        0x03 => {
            let offset = s9 << 2; // S11
            Ok(DecodedInstruction {
                inst: Instruction::Alu {
                    op: AluOp::Add, dst: Operand::Reg(0), src1: Operand::Reg(REG_GP),
                    src2: Operand::Imm(sign_extend_u32(offset, 11)),
                    set_flags: false, cc: None,
                },
                size: 2, has_limm: false, pc,
            })
        }
        _ => unreachable!(),
    }
}

// ===== 0x1A: PCL-relative load =====

fn decode_1a_pcl_relative(half: u16, pc: u32) -> Result<DecodedInstruction, Exception> {
    let b = map_16bit_reg(((half >> 8) & 7) as u8);
    let u8_val = (half & 0xFF) as u32;
    let offset = u8_val << 2; // U10, 32-bit aligned

    Ok(DecodedInstruction {
        inst: Instruction::Load {
            dst: Operand::Reg(b), base: Operand::Reg(63), // PCL
            offset: Operand::Imm(offset), data_size: DataSize::Word,
            sign_extend: false, writeback: WritebackMode::None, cache_bypass: false,
        },
        size: 2, has_limm: false, pc,
    })
}

// ===== 0x1B: MOV_S B, U8 =====

fn decode_1b_mov_imm(half: u16, pc: u32) -> Result<DecodedInstruction, Exception> {
    let b = map_16bit_reg(((half >> 8) & 7) as u8);
    let u8_val = (half & 0xFF) as u32;

    Ok(DecodedInstruction {
        inst: Instruction::Alu {
            op: AluOp::Mov, dst: Operand::Reg(b), src1: Operand::None,
            src2: Operand::Imm(u8_val), set_flags: false, cc: None,
        },
        size: 2, has_limm: false, pc,
    })
}

// ===== 0x1C: ADD_S / CMP_S B, U7 =====

fn decode_1c_add_cmp_imm(half: u16, pc: u32) -> Result<DecodedInstruction, Exception> {
    let b = map_16bit_reg(((half >> 8) & 7) as u8);
    let i = (half >> 7) & 1;
    let u7 = (half & 0x7F) as u32;

    let inst = if i == 0 {
        Instruction::Alu {
            op: AluOp::Add, dst: Operand::Reg(b), src1: Operand::Reg(b),
            src2: Operand::Imm(u7), set_flags: false, cc: None,
        }
    } else {
        Instruction::Alu {
            op: AluOp::Cmp, dst: Operand::None, src1: Operand::Reg(b),
            src2: Operand::Imm(u7), set_flags: true, cc: None,
        }
    };

    Ok(DecodedInstruction { inst, size: 2, has_limm: false, pc })
}

// ===== 0x1D: BRcc_S =====

fn decode_1d_brcc(half: u16, pc: u32) -> Result<DecodedInstruction, Exception> {
    let b = map_16bit_reg(((half >> 8) & 7) as u8);
    let i = (half >> 7) & 1; // 0=BREQ, 1=BRNE
    let s7 = (half & 0x7F) as u32;
    let offset = sign_extend(s7 << 1, 8); // 8-bit signed offset (S8), 16-bit aligned

    let kind = if i == 0 { BrCompareKind::Breq } else { BrCompareKind::Brne };

    Ok(DecodedInstruction {
        inst: Instruction::BranchCompare {
            kind, src1: Operand::Reg(b), src2: Operand::Imm(0),
            offset, delay: DelayMode::NoDelay,
        },
        size: 2, has_limm: false, pc,
    })
}

// ===== 0x1E: Bcc_S =====

fn decode_1e_bcc(half: u16, pc: u32) -> Result<DecodedInstruction, Exception> {
    let sub = ((half >> 9) & 3) as u8;

    match sub {
        0x00 => {
            // B_S: unconditional, s10 offset
            let s9 = (half & 0x1FF) as u32;
            let offset = sign_extend(s9 << 1, 10);
            Ok(DecodedInstruction {
                inst: Instruction::Branch {
                    offset, cc: None, delay: DelayMode::NoDelay, link: false,
                },
                size: 2, has_limm: false, pc,
            })
        }
        0x01 => {
            // BEQ_S: s10 offset
            let s9 = (half & 0x1FF) as u32;
            let offset = sign_extend(s9 << 1, 10);
            Ok(DecodedInstruction {
                inst: Instruction::Branch {
                    offset, cc: Some(ConditionCode::EQ),
                    delay: DelayMode::NoDelay, link: false,
                },
                size: 2, has_limm: false, pc,
            })
        }
        0x02 => {
            // BNE_S: s10 offset
            let s9 = (half & 0x1FF) as u32;
            let offset = sign_extend(s9 << 1, 10);
            Ok(DecodedInstruction {
                inst: Instruction::Branch {
                    offset, cc: Some(ConditionCode::NE),
                    delay: DelayMode::NoDelay, link: false,
                },
                size: 2, has_limm: false, pc,
            })
        }
        0x03 => {
            // Bcc_S: GT,GE,LT,LE,HI,HS,LO,LS with s7
            let cc_enc = ((half >> 6) & 7) as u8;
            let s6 = (half & 0x3F) as u32;
            let offset = sign_extend(s6 << 1, 7);

            let cc = match cc_enc {
                0x00 => ConditionCode::GT,
                0x01 => ConditionCode::GE,
                0x02 => ConditionCode::LT,
                0x03 => ConditionCode::LE,
                0x04 => ConditionCode::HI,
                0x05 => ConditionCode::CC, // HS = CC
                0x06 => ConditionCode::CS, // LO = CS
                0x07 => ConditionCode::LS,
                _ => return Err(Exception::InstructionError { address: pc }),
            };

            Ok(DecodedInstruction {
                inst: Instruction::Branch {
                    offset, cc: Some(cc), delay: DelayMode::NoDelay, link: false,
                },
                size: 2, has_limm: false, pc,
            })
        }
        _ => unreachable!(),
    }
}

// ===== 0x1F: BL_S =====

fn decode_1f_bl(half: u16, pc: u32) -> Result<DecodedInstruction, Exception> {
    let s11 = (half & 0x7FF) as u32;
    let offset = sign_extend(s11 << 2, 13); // S13, 32-bit aligned

    Ok(DecodedInstruction {
        inst: Instruction::Branch {
            offset, cc: None, delay: DelayMode::NoDelay, link: true,
        },
        size: 2, has_limm: false, pc,
    })
}
