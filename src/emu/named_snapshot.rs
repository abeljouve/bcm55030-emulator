use std::collections::HashMap;

use crate::cache::{DcacheSaveState, IcacheSaveState};
use crate::cpu::registers::CpuState;
use crate::soc::alarm_events::AlarmEvents;
use crate::soc::bsc_i2c::BscI2c;
use crate::soc::dma::DmaChannelController;
use crate::soc::efuse_udr::EfuseUdr;
use crate::soc::epon_mac::EponMac;
use crate::soc::fatal_filter::FatalFilter;
use crate::soc::macsec::Macsec;
use crate::soc::mpcp::Mpcp;
use crate::soc::nco::Nco;
use crate::soc::olt::Olt;
use crate::soc::pbc::Pbc;
use crate::soc::scenario::ScenarioEngine;
use crate::soc::serdes::SerDes;
use crate::soc::sysreg_shim::SysregShim;
use crate::soc::timer::EponTimer;
use crate::soc::uart::Uart;
use crate::soc::vlan_lue::VlanLue;

#[derive(Clone)]
pub struct PeripheralBankSaveState {
    pub uart: Uart,
    pub pbc: Pbc,
    pub bsc_i2c: BscI2c,
    pub serdes: SerDes,
    pub epon_mac: EponMac,
    pub macsec: Macsec,
    pub dma: DmaChannelController,
    pub alarm_events: AlarmEvents,
    pub timer: EponTimer,
    pub efuse_udr: EfuseUdr,
    pub fatal_filter: FatalFilter,
    pub mpcp: Mpcp,
    pub nco: Nco,
    pub vlan_lue: VlanLue,
    pub olt: Olt,
    pub scenario: ScenarioEngine,
    pub sysreg: SysregShim,
}

pub struct NamedSnapshot {
    pub name: String,
    pub timestamp: String,
    pub instruction_count: u64,
    pub pc: u32,

    pub cpu_state: CpuState,
    pub sram: Vec<u8>,
    pub dcache: Option<DcacheSaveState>,
    pub icache: Option<IcacheSaveState>,
    pub bank_state: Option<PeripheralBankSaveState>,

    pub timer_frac_acc: u32,
    pub bank_tick_accumulator: u64,
    pub shadow_call_stack: Vec<u32>,
    pub function_profile: HashMap<u32, u64>,
    pub profiling_enabled: bool,
}

impl NamedSnapshot {
    pub fn size_bytes(&self) -> usize {
        std::mem::size_of::<CpuState>()
            + self.sram.len()
            + 8 * 1024 // approximate cache + peripheral state
    }
}

#[derive(Clone, Debug)]
pub struct SnapshotRegDiff {
    pub name: String,
    pub a: u32,
    pub b: u32,
}

#[derive(Clone, Debug)]
pub struct SnapshotDiff {
    pub register_diffs: Vec<SnapshotRegDiff>,
    pub pc_a: u32,
    pub pc_b: u32,
    pub insn_a: u64,
    pub insn_b: u64,
    pub sram_changed_bytes: usize,
}

#[derive(Clone, Debug, serde::Serialize)]
pub struct SnapshotInfo {
    pub name: String,
    pub instruction_count: u64,
    pub pc: u32,
    pub timestamp: String,
    pub size_bytes: usize,
}
