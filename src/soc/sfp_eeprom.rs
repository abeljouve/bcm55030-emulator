//! SFP EEPROM model — SFF-8472 Rev 12.3.
//!
//! Two I²C pages of 256 B each:
//!   - A0h = Serial ID (static) — Table 4-1
//!   - A2h = Digital Diagnostics (DDM) — Table 4-2
//!
//! Captured snapshot: Generic GENERIC-BC+ (Device ONU OLT-side SFP+),
//! read from real BCM55030 hardware on 2026-04-10 via `access/read 4 {0,1} …`.
//! No OLT connected, so TX/RX power and bias are at their laser-off floors.
//!
//! All 16-bit fields on the wire are big-endian (MSB at low address) per
//! SFF-8472 §9.1. The struct definitions below store fields as typed scalars
//! and `to_bytes()` serialises them in the correct byte order.

// ── A0h — Serial ID page (SFF-8472 Table 4-1) ──────────────────────────────

/// SFP A0h identification page. Byte offsets as in SFF-8472 Rev 12.3.
#[derive(Clone, Copy)]
pub struct SfpA0 {
    // Base ID fields (0-62)
    pub identifier: u8,              // 0      — Table 5-1 (03h = SFP/SFP+)
    pub ext_identifier: u8,          // 1      — Table 5-2 (04h = two-wire ID)
    pub connector: u8,               // 2      — SFF-8024 connector code
    pub transceiver: [u8; 8],        // 3-10   — Table 5-3 compliance bits
    pub encoding: u8,                // 11     — SFF-8024 encoding code
    pub br_nominal: u8,              // 12     — signalling rate / 100 MBd
    pub rate_identifier: u8,         // 13     — Table 5-6
    pub length_smf_km: u8,           // 14     — SMF reach, km
    pub length_smf_100m: u8,         // 15     — SMF reach, 100 m
    pub length_om2: u8,              // 16     — 50 µm OM2, 10 m
    pub length_om1: u8,              // 17     — 62.5 µm OM1, 10 m
    pub length_om4_copper: u8,       // 18     — OM4 (10 m) / copper (m)
    pub length_om3: u8,              // 19     — 50 µm OM3, 10 m
    pub vendor_name: [u8; 16],       // 20-35  — ASCII, space-padded
    pub ext_compliance: u8,          // 36     — SFF-8024 §4-4 extended
    pub vendor_oui: [u8; 3],         // 37-39  — IEEE company ID
    pub vendor_pn: [u8; 16],         // 40-55  — ASCII, space-padded
    pub vendor_rev: [u8; 4],         // 56-59  — ASCII
    pub wavelength: u16,             // 60-61  — nm (or copper spec bits)
    pub unallocated_62: u8,          // 62
    pub cc_base: u8,                 // 63     — sum(bytes 0..62) & 0xFF
    // Extended ID fields (64-95)
    pub options: u16,                // 64-65  — Table 8-3
    pub br_max: u8,                  // 66     — BR, max (% above nominal)
    pub br_min: u8,                  // 67     — BR, min (% below nominal)
    pub vendor_sn: [u8; 16],         // 68-83  — ASCII, space-padded
    pub date_year: [u8; 2],          // 84-85  — ASCII YY (00 = 2000)
    pub date_month: [u8; 2],         // 86-87  — ASCII MM
    pub date_day: [u8; 2],           // 88-89  — ASCII DD
    pub date_lot: [u8; 2],           // 90-91  — ASCII vendor lot
    pub diag_monitoring_type: u8,    // 92     — Table 8-5
    pub enhanced_options: u8,        // 93     — Table 8-6
    pub sff8472_compliance: u8,      // 94     — Table 8-8
    pub cc_ext: u8,                  // 95     — sum(bytes 64..94) & 0xFF
    // Vendor-specific + reserved (96-255)
    pub vendor_specific: [u8; 32],   // 96-127
    pub reserved_8079: [u8; 128],    // 128-255 — SFF-8079
}

impl SfpA0 {
    pub const fn to_bytes(&self) -> [u8; 256] {
        let mut b = [0u8; 256];
        b[0] = self.identifier;
        b[1] = self.ext_identifier;
        b[2] = self.connector;
        let mut i = 0;
        while i < 8 {
            b[3 + i] = self.transceiver[i];
            i += 1;
        }
        b[11] = self.encoding;
        b[12] = self.br_nominal;
        b[13] = self.rate_identifier;
        b[14] = self.length_smf_km;
        b[15] = self.length_smf_100m;
        b[16] = self.length_om2;
        b[17] = self.length_om1;
        b[18] = self.length_om4_copper;
        b[19] = self.length_om3;
        i = 0;
        while i < 16 {
            b[20 + i] = self.vendor_name[i];
            i += 1;
        }
        b[36] = self.ext_compliance;
        i = 0;
        while i < 3 {
            b[37 + i] = self.vendor_oui[i];
            i += 1;
        }
        i = 0;
        while i < 16 {
            b[40 + i] = self.vendor_pn[i];
            i += 1;
        }
        i = 0;
        while i < 4 {
            b[56 + i] = self.vendor_rev[i];
            i += 1;
        }
        b[60] = (self.wavelength >> 8) as u8;
        b[61] = self.wavelength as u8;
        b[62] = self.unallocated_62;
        b[63] = self.cc_base;
        b[64] = (self.options >> 8) as u8;
        b[65] = self.options as u8;
        b[66] = self.br_max;
        b[67] = self.br_min;
        i = 0;
        while i < 16 {
            b[68 + i] = self.vendor_sn[i];
            i += 1;
        }
        b[84] = self.date_year[0];
        b[85] = self.date_year[1];
        b[86] = self.date_month[0];
        b[87] = self.date_month[1];
        b[88] = self.date_day[0];
        b[89] = self.date_day[1];
        b[90] = self.date_lot[0];
        b[91] = self.date_lot[1];
        b[92] = self.diag_monitoring_type;
        b[93] = self.enhanced_options;
        b[94] = self.sff8472_compliance;
        b[95] = self.cc_ext;
        i = 0;
        while i < 32 {
            b[96 + i] = self.vendor_specific[i];
            i += 1;
        }
        i = 0;
        while i < 128 {
            b[128 + i] = self.reserved_8079[i];
            i += 1;
        }
        b
    }
}

// ── A2h — DDM page (SFF-8472 Table 4-2) ─────────────────────────────────────

/// SFP A2h digital diagnostics page.
///
/// Encoding conventions (SFF-8472 §9.2):
///   - Temperature:          i16, 1/256 °C
///   - Vcc:                  u16, 100 µV
///   - TX bias:              u16, 2 µA
///   - TX/RX power:          u16, 0.1 µW
///   - Slopes (cal):         u16 unsigned fixed-point (upper byte integer)
///   - Offsets (cal):        i16 two's complement
///   - Rx_PWR(n):            IEEE-754 single precision (stored raw: MSB first)
#[derive(Clone, Copy)]
pub struct SfpA2 {
    // Alarm/Warning thresholds (0-39) — Table 9-5
    pub temp_high_alarm: i16,            // 0-1
    pub temp_low_alarm: i16,             // 2-3
    pub temp_high_warning: i16,          // 4-5
    pub temp_low_warning: i16,           // 6-7
    pub vcc_high_alarm: u16,             // 8-9
    pub vcc_low_alarm: u16,              // 10-11
    pub vcc_high_warning: u16,           // 12-13
    pub vcc_low_warning: u16,            // 14-15
    pub tx_bias_high_alarm: u16,         // 16-17
    pub tx_bias_low_alarm: u16,          // 18-19
    pub tx_bias_high_warning: u16,       // 20-21
    pub tx_bias_low_warning: u16,        // 22-23
    pub tx_power_high_alarm: u16,        // 24-25
    pub tx_power_low_alarm: u16,         // 26-27
    pub tx_power_high_warning: u16,      // 28-29
    pub tx_power_low_warning: u16,       // 30-31
    pub rx_power_high_alarm: u16,        // 32-33
    pub rx_power_low_alarm: u16,         // 34-35
    pub rx_power_high_warning: u16,      // 36-37
    pub rx_power_low_warning: u16,       // 38-39
    // Optional Laser Temp + TEC thresholds (40-55) — Table 9-5
    pub opt_laser_temp_high_alarm: i16,  // 40-41
    pub opt_laser_temp_low_alarm: i16,   // 42-43
    pub opt_laser_temp_high_warning: i16,// 44-45
    pub opt_laser_temp_low_warning: i16, // 46-47
    pub opt_tec_current_high_alarm: i16, // 48-49
    pub opt_tec_current_low_alarm: i16,  // 50-51
    pub opt_tec_current_high_warning: i16,// 52-53
    pub opt_tec_current_low_warning: i16,// 54-55
    // External calibration constants (56-91) — Table 9-6
    pub rx_pwr_4: [u8; 4],               // 56-59  IEEE-754 MSB-first
    pub rx_pwr_3: [u8; 4],               // 60-63
    pub rx_pwr_2: [u8; 4],               // 64-67
    pub rx_pwr_1: [u8; 4],               // 68-71
    pub rx_pwr_0: [u8; 4],               // 72-75
    pub tx_i_slope: u16,                 // 76-77
    pub tx_i_offset: i16,                // 78-79
    pub tx_pwr_slope: u16,               // 80-81
    pub tx_pwr_offset: i16,              // 82-83
    pub t_slope: u16,                    // 84-85
    pub t_offset: i16,                   // 86-87
    pub v_slope: u16,                    // 88-89
    pub v_offset: i16,                   // 90-91
    pub unallocated_92_94: [u8; 3],      // 92-94
    pub cc_dmi: u8,                      // 95     sum(0..94) & 0xFF
    // Real-time diagnostics (96-109) — Table 9-11
    pub temperature: i16,                // 96-97
    pub vcc: u16,                        // 98-99
    pub tx_bias: u16,                    // 100-101
    pub tx_power: u16,                   // 102-103
    pub rx_power: u16,                   // 104-105
    pub opt_laser_temp_wavelength: u16,  // 106-107
    pub opt_tec_current: i16,            // 108-109
    // Optional status/control + flags (110-119)
    pub status_control: u8,              // 110    Table 9-11
    pub reserved_111: u8,                // 111    SFF-8079
    pub alarm_flags: u16,                // 112-113 Table 9-12
    pub tx_input_eq: u8,                 // 114    Table 9-13
    pub rx_output_emphasis: u8,          // 115    Table 9-14 (or CDR unlocked)
    pub warning_flags: u16,              // 116-117 Table 9-12
    pub ext_status_control: u16,         // 118-119 Table 10-1
    // General use (120-127)
    pub vendor_specific: [u8; 7],        // 120-126
    pub page_select: u8,                 // 127
    // Page 00/01 payload (128-255)
    pub user_eeprom: [u8; 120],          // 128-247
    pub vendor_control: [u8; 8],         // 248-255
}

impl SfpA2 {
    pub const fn to_bytes(&self) -> [u8; 256] {
        let mut b = [0u8; 256];
        // Helper: write a big-endian u16 at offset `o`.
        macro_rules! be16 {
            ($o:expr, $v:expr) => {{
                let v = $v as u16;
                b[$o] = (v >> 8) as u8;
                b[$o + 1] = v as u8;
            }};
        }
        be16!(0, self.temp_high_alarm);
        be16!(2, self.temp_low_alarm);
        be16!(4, self.temp_high_warning);
        be16!(6, self.temp_low_warning);
        be16!(8, self.vcc_high_alarm);
        be16!(10, self.vcc_low_alarm);
        be16!(12, self.vcc_high_warning);
        be16!(14, self.vcc_low_warning);
        be16!(16, self.tx_bias_high_alarm);
        be16!(18, self.tx_bias_low_alarm);
        be16!(20, self.tx_bias_high_warning);
        be16!(22, self.tx_bias_low_warning);
        be16!(24, self.tx_power_high_alarm);
        be16!(26, self.tx_power_low_alarm);
        be16!(28, self.tx_power_high_warning);
        be16!(30, self.tx_power_low_warning);
        be16!(32, self.rx_power_high_alarm);
        be16!(34, self.rx_power_low_alarm);
        be16!(36, self.rx_power_high_warning);
        be16!(38, self.rx_power_low_warning);
        be16!(40, self.opt_laser_temp_high_alarm);
        be16!(42, self.opt_laser_temp_low_alarm);
        be16!(44, self.opt_laser_temp_high_warning);
        be16!(46, self.opt_laser_temp_low_warning);
        be16!(48, self.opt_tec_current_high_alarm);
        be16!(50, self.opt_tec_current_low_alarm);
        be16!(52, self.opt_tec_current_high_warning);
        be16!(54, self.opt_tec_current_low_warning);
        let mut i = 0;
        while i < 4 { b[56 + i] = self.rx_pwr_4[i]; i += 1; }
        i = 0;
        while i < 4 { b[60 + i] = self.rx_pwr_3[i]; i += 1; }
        i = 0;
        while i < 4 { b[64 + i] = self.rx_pwr_2[i]; i += 1; }
        i = 0;
        while i < 4 { b[68 + i] = self.rx_pwr_1[i]; i += 1; }
        i = 0;
        while i < 4 { b[72 + i] = self.rx_pwr_0[i]; i += 1; }
        be16!(76, self.tx_i_slope);
        be16!(78, self.tx_i_offset);
        be16!(80, self.tx_pwr_slope);
        be16!(82, self.tx_pwr_offset);
        be16!(84, self.t_slope);
        be16!(86, self.t_offset);
        be16!(88, self.v_slope);
        be16!(90, self.v_offset);
        b[92] = self.unallocated_92_94[0];
        b[93] = self.unallocated_92_94[1];
        b[94] = self.unallocated_92_94[2];
        b[95] = self.cc_dmi;
        be16!(96, self.temperature);
        be16!(98, self.vcc);
        be16!(100, self.tx_bias);
        be16!(102, self.tx_power);
        be16!(104, self.rx_power);
        be16!(106, self.opt_laser_temp_wavelength);
        be16!(108, self.opt_tec_current);
        b[110] = self.status_control;
        b[111] = self.reserved_111;
        be16!(112, self.alarm_flags);
        b[114] = self.tx_input_eq;
        b[115] = self.rx_output_emphasis;
        be16!(116, self.warning_flags);
        be16!(118, self.ext_status_control);
        i = 0;
        while i < 7 { b[120 + i] = self.vendor_specific[i]; i += 1; }
        b[127] = self.page_select;
        i = 0;
        while i < 120 { b[128 + i] = self.user_eeprom[i]; i += 1; }
        i = 0;
        while i < 8 { b[248 + i] = self.vendor_control[i]; i += 1; }
        b
    }
}

// ── Generic GENERIC-BC+ captured snapshot ───────────────────────────────────

/// A0h snapshot captured from Device ONU on 2026-04-10.
pub const GENERIC_SFP_A0: SfpA0 = SfpA0 {
    identifier: 0x03,                            // SFP/SFP+
    ext_identifier: 0x04,                        // two-wire ID
    connector: 0x01,                             // SC (SFF-8024)
    transceiver: [0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00],
    encoding: 0x03,                              // NRZ
    br_nominal: 0x0D,                            // 1300 MBd nominal (vendor)
    rate_identifier: 0x00,
    length_smf_km: 0x14,                         // 20 km
    length_smf_100m: 0xC8,                       // 200 × 100m = 20 km
    length_om2: 0x00,
    length_om1: 0x00,
    length_om4_copper: 0x00,
    length_om3: 0x00,
    vendor_name: *b"Generic         ",
    ext_compliance: 0x00,                        // none
    vendor_oui: [0x00, 0x00, 0x00],              // Generic Broadband OUI
    vendor_pn: *b"GENERIC-PN01    ",
    vendor_rev: *b"V1.0",
    wavelength: 0x051E,                          // 1310 nm
    unallocated_62: 0x00,
    cc_base: 0xA2,                               // vendor-supplied
    options: 0x001A,                             // TX_DISABLE + TX_FAULT + RX_LOS
    br_max: 0x14,                                // +20 %
    br_min: 0x14,                                // −20 %
    vendor_sn: *b"SN000000001     ",
    date_year: *b"23",
    date_month: *b"10",
    date_day: *b"21",
    date_lot: *b"  ",
    diag_monitoring_type: 0x68,                  // DDM + internal cal + avg pwr
    enhanced_options: 0xF0,                      // alarms + soft TX_DIS/FAULT/LOS
    sff8472_compliance: 0x02,                    // Rev 9.5
    cc_ext: 0x14,                                // vendor-supplied
    vendor_specific: *b"GENERICSFP;00000-SN000000001-F3;",
    reserved_8079: [0; 128],
};

/// A2h snapshot captured from Device ONU on 2026-04-10.
///
/// No OLT connected → TX/RX power and bias sit at their laser-off floors,
/// and both the RX Power Low and TX Power Low alarms are asserted in
/// `alarm_flags`/`warning_flags`.
pub const GENERIC_SFP_A2: SfpA2 = SfpA2 {
    // Temperature: industrial range (1/256 °C units)
    temp_high_alarm:    0x4B00, //  75.0 °C
    temp_low_alarm:    -0x0A00, // -10.0 °C  (raw 0xF600)
    temp_high_warning:  0x4600, //  70.0 °C
    temp_low_warning:  -0x0500, //  -5.0 °C  (raw 0xFB00)
    // Vcc (100 µV/LSB)
    vcc_high_alarm:    0x8CA0,  // 3.608 V
    vcc_low_alarm:     0x7530,  // 3.000 V
    vcc_high_warning:  0x88B8,  // 3.500 V
    vcc_low_warning:   0x7918,  // 3.100 V
    // TX bias (2 µA/LSB)
    tx_bias_high_alarm:    0xC350, // 100 mA
    tx_bias_low_alarm:     0x0000,
    tx_bias_high_warning:  0x9C40, //  80 mA
    tx_bias_low_warning:   0x0000,
    // TX power (0.1 µW/LSB)
    tx_power_high_alarm:    0xB360, // ≈ 4.586 mW
    tx_power_low_alarm:     0x22FA, // ≈ 0.894 mW
    tx_power_high_warning:  0x8E7B, // ≈ 3.650 mW
    tx_power_low_warning:   0x2D0F, // ≈ 1.153 mW
    // RX power (0.1 µW/LSB)
    rx_power_high_alarm:    0x04EB, // ≈ 125.9 µW (-9 dBm)
    rx_power_low_alarm:     0x000B, // ≈   1.1 µW (-29.6 dBm)
    rx_power_high_warning:  0x03E8, // ≈ 100.0 µW (-10 dBm)
    rx_power_low_warning:   0x000E, // ≈   1.4 µW (-28.5 dBm)
    // Optional DWDM thresholds — not used by this module
    opt_laser_temp_high_alarm:    0,
    opt_laser_temp_low_alarm:     0,
    opt_laser_temp_high_warning:  0,
    opt_laser_temp_low_warning:   0,
    opt_tec_current_high_alarm:   0,
    opt_tec_current_low_alarm:    0,
    opt_tec_current_high_warning: 0,
    opt_tec_current_low_warning:  0,
    // Rx_PWR polynomial: only Rx_PWR(1) = 1.0f → identity mapping
    // (internally-calibrated device per SFF-8472 §9.5)
    rx_pwr_4: [0x00, 0x00, 0x00, 0x00],
    rx_pwr_3: [0x00, 0x00, 0x00, 0x00],
    rx_pwr_2: [0x00, 0x00, 0x00, 0x00],
    rx_pwr_1: [0x3F, 0x80, 0x00, 0x00], // 1.0f
    rx_pwr_0: [0x00, 0x00, 0x00, 0x00],
    // Tx_I / Tx_PWR / T / V: slope=1.0, offset=0 (internally calibrated)
    tx_i_slope:   0x0100,
    tx_i_offset:  0x0000,
    tx_pwr_slope: 0x0100,
    tx_pwr_offset: 0x0000,
    t_slope:   0x0100,
    t_offset:  0x0000,
    v_slope:   0x0100,
    v_offset:  0x0000,
    unallocated_92_94: [0x00, 0x00, 0x00],
    cc_dmi: 0x3D,
    // Live readings at capture time (no OLT, laser off)
    temperature: 0x2A71,                 //  42.44 °C
    vcc:         0x7FAB,                 //  3.2683 V
    tx_bias:     0x0001,                 //  2 µA (floor)
    tx_power:    0x0001,                 //  0.1 µW (floor)
    rx_power:    0x0001,                 //  0.1 µW (floor)
    opt_laser_temp_wavelength: 0xFFFF,
    opt_tec_current:           -1,       // 0xFFFF — not implemented
    status_control: 0x02,                // Rx_LOS asserted
    reserved_111:   0x00,
    // Alarm/Warning flags reflect laser-off state:
    //   byte 112 bit 0 = TX Power Low Alarm
    //   byte 113 bit 6 = RX Power Low Alarm
    alarm_flags:   0x0140,
    tx_input_eq:   0x00,
    rx_output_emphasis: 0x04,
    warning_flags: 0x0140,
    ext_status_control: 0x0000,
    vendor_specific: [0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF],
    page_select: 0x00,
    user_eeprom: [0; 120],
    vendor_control: [0xFF; 8],
};

// ── Flat byte-array views (consumed by the BSC I²C emulation) ───────────────

pub const SFP_A0: [u8; 256] = GENERIC_SFP_A0.to_bytes();
pub const SFP_A2: [u8; 256] = GENERIC_SFP_A2.to_bytes();

/// Returns 4 EEPROM bytes packed little-endian (byte 0 → LSB).
/// `device`: 0=A0h, 1=A2h. `byte_offset` wraps modulo 256.
pub fn read_word(device: u8, byte_offset: u16) -> u32 {
    let page: &[u8; 256] = match device {
        1 => &SFP_A2,
        _ => &SFP_A0,
    };
    let off = (byte_offset as usize) & 0xFF;
    let b0 = page[off] as u32;
    let b1 = page[(off + 1) & 0xFF] as u32;
    let b2 = page[(off + 2) & 0xFF] as u32;
    let b3 = page[(off + 3) & 0xFF] as u32;
    (b3 << 24) | (b2 << 16) | (b1 << 8) | b0
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Raw A0h bytes as captured on real BCM55030 hardware, 2026-04-10.
    /// Source of truth — changes to the struct must keep this golden identical.
    #[rustfmt::skip]
    const GOLDEN_A0: [u8; 256] = [
        0x03, 0x04, 0x01, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x03, 0x0D, 0x00, 0x14, 0xC8,
        0x00, 0x00, 0x00, 0x00, 0x47, 0x65, 0x6E, 0x65, 0x72, 0x69, 0x63, 0x20, 0x20, 0x20, 0x20, 0x20,
        0x20, 0x20, 0x20, 0x20, 0x00, 0x00, 0x00, 0x00, 0x4C, 0x54, 0x46, 0x37, 0x32, 0x31, 0x35, 0x2D,
        0x42, 0x43, 0x2B, 0x31, 0x20, 0x20, 0x20, 0x20, 0x56, 0x31, 0x2E, 0x30, 0x05, 0x1E, 0x00, 0xA2,
        0x00, 0x1A, 0x14, 0x14, 0x4C, 0x33, 0x39, 0x44, 0x41, 0x30, 0x35, 0x38, 0x33, 0x30, 0x32, 0x20,
        0x20, 0x20, 0x20, 0x20, 0x32, 0x33, 0x31, 0x30, 0x32, 0x31, 0x20, 0x20, 0x68, 0xF0, 0x02, 0x14,
        0x46, 0x2D, 0x53, 0x46, 0x50, 0x4F, 0x4E, 0x55, 0x31, 0x41, 0x3B, 0x31, 0x35, 0x34, 0x37, 0x35,
        0x2D, 0x4C, 0x33, 0x39, 0x44, 0x41, 0x30, 0x35, 0x38, 0x33, 0x30, 0x32, 0x2D, 0x46, 0x33, 0x3B,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    ];

    /// Raw A2h bytes as captured on real BCM55030 hardware, 2026-04-10.
    #[rustfmt::skip]
    const GOLDEN_A2: [u8; 256] = [
        0x4B, 0x00, 0xF6, 0x00, 0x46, 0x00, 0xFB, 0x00, 0x8C, 0xA0, 0x75, 0x30, 0x88, 0xB8, 0x79, 0x18,
        0xC3, 0x50, 0x00, 0x00, 0x9C, 0x40, 0x00, 0x00, 0xB3, 0x60, 0x22, 0xFA, 0x8E, 0x7B, 0x2D, 0x0F,
        0x04, 0xEB, 0x00, 0x0B, 0x03, 0xE8, 0x00, 0x0E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x3F, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,
        0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3D,
        0x2A, 0x71, 0x7F, 0xAB, 0x00, 0x01, 0x00, 0x01, 0x00, 0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0x02, 0x00,
        0x01, 0x40, 0x00, 0x04, 0x01, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    ];

    /// Each byte of the serialised struct must match the expected serialisation.
    /// On mismatch, the panic message points to the exact offending offset.
    fn assert_equals_golden(label: &str, got: &[u8; 256], want: &[u8; 256]) {
        for i in 0..256 {
            assert_eq!(
                got[i], want[i],
                "{} mismatch at offset 0x{:02X}: got 0x{:02X}, want 0x{:02X}",
                label, i, got[i], want[i]
            );
        }
    }

    #[test]
    fn a0_matches_expected() {
        assert_equals_golden("A0h", &SFP_A0, &GOLDEN_A0);
    }

    #[test]
    fn a2_matches_expected() {
        assert_equals_golden("A2h", &SFP_A2, &GOLDEN_A2);
    }

    /// CC_BASE = low 8 bits of sum(bytes 0..62), per SFF-8472 §8.2.
    #[test]
    fn a0_cc_base_is_valid() {
        let sum: u32 = SFP_A0[0..=62].iter().map(|&b| b as u32).sum();
        assert_eq!((sum & 0xFF) as u8, SFP_A0[63], "CC_BASE checksum mismatch");
    }

    /// CC_EXT = low 8 bits of sum(bytes 64..94), per SFF-8472 §8.12.
    #[test]
    fn a0_cc_ext_is_valid() {
        let sum: u32 = SFP_A0[64..=94].iter().map(|&b| b as u32).sum();
        assert_eq!((sum & 0xFF) as u8, SFP_A0[95], "CC_EXT checksum mismatch");
    }

    /// CC_DMI = low 8 bits of sum(A2h bytes 0..94), per SFF-8472 §9.6.
    #[test]
    fn a2_cc_dmi_is_valid() {
        let sum: u32 = SFP_A2[0..=94].iter().map(|&b| b as u32).sum();
        assert_eq!((sum & 0xFF) as u8, SFP_A2[95], "CC_DMI checksum mismatch");
    }

    /// Temperature thresholds must be in the industrial range, not near 0 °C.
    /// This guards against the byte-swap bug that previously had
    /// Temp Low Alarm = +0.96 °C instead of -10 °C.
    #[test]
    fn a2_temp_low_thresholds_are_subzero() {
        let low_alarm = i16::from_be_bytes([SFP_A2[2], SFP_A2[3]]);
        let low_warn = i16::from_be_bytes([SFP_A2[6], SFP_A2[7]]);
        assert!(low_alarm < 0, "Temp Low Alarm = {} (raw 0x{:04X}) should be sub-zero",
            low_alarm, low_alarm as u16);
        assert!(low_warn < 0, "Temp Low Warning = {} (raw 0x{:04X}) should be sub-zero",
            low_warn, low_warn as u16);
    }
}
