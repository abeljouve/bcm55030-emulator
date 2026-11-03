//! Auxiliary registers whose presence on this core was measured on silicon.
//!
//! The exception-state auxiliary registers of the generic ARC 700 are not
//! implemented: ERET, ERBTA, ERSTATUS, ECR, EFA (0x400-0x404),
//! ICAUSE1/2 (0x40A/0x40B) and BTA, BTA_L1, BTA_L2 (0x412-0x414). Measured on
//! silicon: each reads back `IDENTITY` before and after a write of two
//! different patterns, and a write never faults. STATUS32_L1/L2 and
//! AUX_IRQ_LV12, which this core does implement, are the controls.

use bcm55030_emulator::cpu::registers::{
    CpuState, AUX_BTA, AUX_BTA_L1, AUX_BTA_L2, AUX_ECR, AUX_EFA, AUX_ERBTA, AUX_ERET,
    AUX_ERSTATUS, AUX_ICAUSE1, AUX_ICAUSE2, AUX_STATUS32_L1, AUX_STATUS32_L2, IDENTITY_VALUE,
};

const ABSENT: [u32; 10] = [
    AUX_ERET, AUX_ERBTA, AUX_ERSTATUS, AUX_ECR, AUX_EFA, AUX_ICAUSE1, AUX_ICAUSE2, AUX_BTA,
    AUX_BTA_L1, AUX_BTA_L2,
];

#[test]
fn absent_exception_regs_read_identity_and_ignore_writes() {
    let mut st = CpuState::new();
    for addr in ABSENT {
        assert_eq!(st.read_aux_reg(addr).unwrap(), IDENTITY_VALUE, "aux {addr:#x}");
        for pattern in [0x1234_5678, 0xA5A5_A5A5] {
            st.write_aux_reg(addr, pattern).unwrap();
            assert_eq!(st.read_aux_reg(addr).unwrap(), IDENTITY_VALUE, "aux {addr:#x}");
        }
    }
}

#[test]
fn absent_exception_regs_are_not_privileged() {
    let mut st = CpuState::new();
    st.flag_u = true;
    for addr in ABSENT {
        assert!(st.write_aux_reg(addr, 1).is_ok(), "aux {addr:#x}");
    }
}

#[test]
fn level_status_registers_still_exist() {
    let mut st = CpuState::new();
    st.write_aux_reg(AUX_STATUS32_L1, 0x0000_0A00).unwrap();
    st.write_aux_reg(AUX_STATUS32_L2, 0x0000_0500).unwrap();
    assert_eq!(st.read_aux_reg(AUX_STATUS32_L1).unwrap(), 0x0000_0A00);
    assert_eq!(st.read_aux_reg(AUX_STATUS32_L2).unwrap(), 0x0000_0500);
}

#[test]
fn mulhi_is_a_plain_register_reading_2_after_reset() {
    use bcm55030_emulator::cpu::registers::AUX_MULHI;
    let mut st = CpuState::new();
    assert_eq!(st.read_aux_reg(AUX_MULHI).unwrap(), 0x2);
    for v in [1, 0, 0xDEAD_BEEF] {
        st.write_aux_reg(AUX_MULHI, v).unwrap();
        assert_eq!(st.read_aux_reg(AUX_MULHI).unwrap(), v);
    }
}
