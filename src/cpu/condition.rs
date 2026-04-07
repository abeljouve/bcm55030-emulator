/// Condition codes for ARCompact ISA (Table 50)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum ConditionCode {
    AL = 0x00,  // Always
    EQ = 0x01,  // Zero / Equal
    NE = 0x02,  // Non-zero / Not equal
    PL = 0x03,  // Positive
    MI = 0x04,  // Negative
    CS = 0x05,  // Carry set / Lower than (unsigned)
    CC = 0x06,  // Carry clear / Higher or same (unsigned)
    VS = 0x07,  // Overflow set
    VC = 0x08,  // Overflow clear
    GT = 0x09,  // Greater than (signed)
    GE = 0x0A,  // Greater than or equal (signed)
    LT = 0x0B,  // Less than (signed)
    LE = 0x0C,  // Less than or equal (signed)
    HI = 0x0D,  // Higher than (unsigned)
    LS = 0x0E,  // Lower than or same (unsigned)
    PNZ = 0x0F, // Positive non-zero
}

impl ConditionCode {
    pub fn from_u8(val: u8) -> Option<Self> {
        match val {
            0x00 => Some(Self::AL),
            0x01 => Some(Self::EQ),
            0x02 => Some(Self::NE),
            0x03 => Some(Self::PL),
            0x04 => Some(Self::MI),
            0x05 => Some(Self::CS),
            0x06 => Some(Self::CC),
            0x07 => Some(Self::VS),
            0x08 => Some(Self::VC),
            0x09 => Some(Self::GT),
            0x0A => Some(Self::GE),
            0x0B => Some(Self::LT),
            0x0C => Some(Self::LE),
            0x0D => Some(Self::HI),
            0x0E => Some(Self::LS),
            0x0F => Some(Self::PNZ),
            _ => None,
        }
    }

    pub fn evaluate(self, z: bool, n: bool, c: bool, v: bool) -> bool {
        match self {
            Self::AL => true,
            Self::EQ => z,
            Self::NE => !z,
            Self::PL => !n,
            Self::MI => n,
            Self::CS => c,
            Self::CC => !c,
            Self::VS => v,
            Self::VC => !v,
            Self::GT => (n && v && !z) || (!n && !v && !z),
            Self::GE => (n && v) || (!n && !v),
            Self::LT => (n && !v) || (!n && v),
            Self::LE => z || (n && !v) || (!n && v),
            Self::HI => !c && !z,
            Self::LS => c || z,
            Self::PNZ => !n && !z,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_always() {
        assert!(ConditionCode::AL.evaluate(false, false, false, false));
        assert!(ConditionCode::AL.evaluate(true, true, true, true));
    }

    #[test]
    fn test_eq_ne() {
        assert!(ConditionCode::EQ.evaluate(true, false, false, false));
        assert!(!ConditionCode::EQ.evaluate(false, false, false, false));
        assert!(ConditionCode::NE.evaluate(false, false, false, false));
        assert!(!ConditionCode::NE.evaluate(true, false, false, false));
    }

    #[test]
    fn test_signed_comparisons() {
        // GT: (N AND V AND !Z) OR (!N AND !V AND !Z)
        assert!(ConditionCode::GT.evaluate(false, false, false, false)); // !N && !V && !Z
        assert!(ConditionCode::GT.evaluate(false, true, false, true)); // N && V && !Z
        assert!(!ConditionCode::GT.evaluate(true, false, false, false)); // Z set

        // GE: (N AND V) OR (!N AND !V)
        assert!(ConditionCode::GE.evaluate(false, false, false, false));
        assert!(ConditionCode::GE.evaluate(false, true, false, true));
        assert!(!ConditionCode::GE.evaluate(false, true, false, false));

        // LT: (N AND !V) OR (!N AND V)
        assert!(ConditionCode::LT.evaluate(false, true, false, false));
        assert!(ConditionCode::LT.evaluate(false, false, false, true));
        assert!(!ConditionCode::LT.evaluate(false, false, false, false));

        // LE: Z OR (N AND !V) OR (!N AND V)
        assert!(ConditionCode::LE.evaluate(true, false, false, false)); // Z
        assert!(ConditionCode::LE.evaluate(false, true, false, false)); // N && !V
    }

    #[test]
    fn test_unsigned_comparisons() {
        // ARC convention: C = borrow for subtraction
        // After CMP A, B (A - B): C=1 if A < B (borrow), C=0 if A >= B (no borrow)
        // HI: /C AND /Z (no borrow AND not zero = A > B unsigned)
        assert!(ConditionCode::HI.evaluate(false, false, false, false)); // C=0, Z=0 (A > B)
        assert!(!ConditionCode::HI.evaluate(true, false, false, false)); // Z=1 (A == B)
        assert!(!ConditionCode::HI.evaluate(false, false, true, false)); // C=1 (A < B)

        // LS: C OR Z (borrow OR zero = A <= B unsigned)
        assert!(ConditionCode::LS.evaluate(true, false, false, false)); // Z=1 (A == B)
        assert!(ConditionCode::LS.evaluate(false, false, true, false)); // C=1 (A < B)
        assert!(!ConditionCode::LS.evaluate(false, false, false, false)); // C=0, Z=0 (A > B)
    }

    #[test]
    fn test_from_u8() {
        assert_eq!(ConditionCode::from_u8(0x00), Some(ConditionCode::AL));
        assert_eq!(ConditionCode::from_u8(0x0F), Some(ConditionCode::PNZ));
        assert_eq!(ConditionCode::from_u8(0x10), None);
        assert_eq!(ConditionCode::from_u8(0xFF), None);
    }
}
