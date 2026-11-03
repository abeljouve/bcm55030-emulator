#![allow(dead_code, clippy::all)]
use crate::cpu::condition::ConditionCode;

pub fn evaluate(cc: ConditionCode, z: bool, n: bool, c: bool, v: bool) -> bool {
    match cc {
        ConditionCode::AL => true,
        ConditionCode::EQ => z,
        ConditionCode::NE => !z,
        ConditionCode::PL => !n,
        ConditionCode::MI => n,
        ConditionCode::CS => c,
        ConditionCode::CC => !c,
        ConditionCode::VS => v,
        ConditionCode::VC => !v,
        ConditionCode::GT => (n && v && !z) || (!n && !v && !z),
        ConditionCode::GE => (n && v) || (!n && !v),
        ConditionCode::LT => (n && !v) || (!n && v),
        ConditionCode::LE => z || (n && !v) || (!n && v),
        ConditionCode::HI => !c && !z,
        ConditionCode::LS => c || z,
        ConditionCode::PNZ => !n && !z,
    }
}

