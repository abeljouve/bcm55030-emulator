//! MMIO block/register name lookup. Generated from hwregs DB — do not hand-edit.

/// Resolved MMIO register entry.
pub struct RegInfo {
    pub block_id: u32,
    pub block_name: &'static str,
    pub reg_name: &'static str,
    pub access: &'static str,
    pub desc: &'static str,
}

/// All known MMIO register addresses with metadata. Total: 192 entries.
/// Sorted by address for binary search.
pub const MMIO_REGISTERS: &[(u32, RegInfo)] = &[
    (0x00FC1014, RegInfo {
        block_id: 47, block_name: "Clock Divider / NCO Config",
        reg_name: "CLK_DIV_COMMAND", access: "W",
        desc: "Commande : 0xa4 = set divider",
    }),
    (0x00FC1015, RegInfo {
        block_id: 47, block_name: "Clock Divider / NCO Config",
        reg_name: "CLK_CTRL_FIELD1", access: "W",
        desc: "0 pendant init",
    }),
    (0x00FC1016, RegInfo {
        block_id: 47, block_name: "Clock Divider / NCO Config",
        reg_name: "CLK_CTRL_FIELD2", access: "W",
        desc: "0 pendant init",
    }),
    (0x00FC1017, RegInfo {
        block_id: 47, block_name: "Clock Divider / NCO Config",
        reg_name: "CLK_READY_FLAG", access: "R",
        desc: "1 = clock stable (`serdes_hw_ready_flag`)",
    }),
    (0x00FC1018, RegInfo {
        block_id: 47, block_name: "Clock Divider / NCO Config",
        reg_name: "CLK_DIV_VALUE_LOW", access: "W",
        desc: "Valeur diviseur (octet bas)",
    }),
    (0x00FC1019, RegInfo {
        block_id: 47, block_name: "Clock Divider / NCO Config",
        reg_name: "CLK_CTRL_FIELD5", access: "W",
        desc: "0 pendant init",
    }),
    (0x00FC101C, RegInfo {
        block_id: 47, block_name: "Clock Divider / NCO Config",
        reg_name: "CLK_DIV_VALUE_HIGH", access: "W",
        desc: "Valeur diviseur (octet haut)",
    }),
    (0x01000000, RegInfo {
        block_id: 49, block_name: "HW State Monitoring Table",
        reg_name: "HW_STATE_REG_N", access: "R",
        desc: "Registre d'etat HW poll periodiquement",
    }),
    (0x01000008, RegInfo {
        block_id: 39, block_name: "SerDes Global Control",
        reg_name: "SERDES_LANE0_1_ENABLE", access: "RMW",
        desc: "OR 0x800000 = enable lanes 0-1 (bus 0)",
    }),
    (0x0100000C, RegInfo {
        block_id: 40, block_name: "SerDes Lane1 MMIO Configuration",
        reg_name: "LANE1_MODE_CONFIG", access: "RMW",
        desc: "Configuration mode lane 1 :",
    }),
    (0x01000014, RegInfo {
        block_id: 39, block_name: "SerDes Global Control",
        reg_name: "MAILBOX_DMA_CLEAR_0", access: "W",
        desc: "Mailbox DMA initialization register 0. Written with 0xFFFFFFFF by mailbox_dma_in",
    }),
    (0x0100001C, RegInfo {
        block_id: 39, block_name: "SerDes Global Control",
        reg_name: "MAILBOX_DMA_CLEAR_1", access: "W",
        desc: "Mailbox DMA initialization register 1. Written with 0xFFFFFFFF by mailbox_dma_in",
    }),
    (0x01000020, RegInfo {
        block_id: 39, block_name: "SerDes Global Control",
        reg_name: "MPCP_SLOT_CONFIG", access: "W",
        desc: "MPCP slot configuration register. Written with 0x00028124 by mpcp_slot_cfg_init_",
    }),
    (0x01000024, RegInfo {
        block_id: 39, block_name: "SerDes Global Control",
        reg_name: "EPON_LINK_ENABLE_LOW", access: "RMW",
        desc: "EPON link enable bitmap for links 0-17 (low LLID range). Each bit enables/disabl",
    }),
    (0x01000038, RegInfo {
        block_id: 39, block_name: "SerDes Global Control",
        reg_name: "EPON_LINK_ENABLE_BITMAP", access: "RMW",
        desc: "EPON link enable bitmap for DPoE links 18-33 (high LLID range). Each bit enables",
    }),
    (0x0100003C, RegInfo {
        block_id: 68, block_name: "EPON Link Enable Controller",
        reg_name: "LLID_ENABLE_CLEAR", access: "W",
        desc: "Clear all LLID enables (ecrit 0)",
    }),
    (0x01000040, RegInfo {
        block_id: 39, block_name: "SerDes Global Control",
        reg_name: "I2C_SERIAL_BUS_CTRL", access: "RMW",
        desc: "I2C/serial bus control register. Used by serial_bus_read_80bytes to bit-bang I2C",
    }),
    (0x01000048, RegInfo {
        block_id: 50, block_name: "I2C / SFP Serial Bus Controller",
        reg_name: "I2C_ACK_CTRL", access: "RMW",
        desc: "ACK/NACK controle (1=NACK end, 0=ACK continue)",
    }),
    (0x0100004C, RegInfo {
        block_id: 50, block_name: "I2C / SFP Serial Bus Controller",
        reg_name: "I2C_CLK_TOGGLE", access: "RMW",
        desc: "Set puis clear = 1 pulse horloge I2C",
    }),
    (0x01000050, RegInfo {
        block_id: 45, block_name: "SerDes Speed Control",
        reg_name: "CHIP_ID_LOW16", access: "R",
        desc: "16-bit chip ID/revision -- `serdes_read_chip_id_low16`",
    }),
    (0x01000054, RegInfo {
        block_id: 45, block_name: "SerDes Speed Control",
        reg_name: "SPEED_CTRL_PARAM", access: "W",
        desc: "Parametre additionnel -- `serdes_set_speed_ctrl_reg_0x54`",
    }),
    (0x01000060, RegInfo {
        block_id: 73, block_name: "MDIO Clause 45 Controller",
        reg_name: "MDIO_COMMAND", access: "W",
        desc: "0x80000 = start read/write cycle",
    }),
    (0x01000064, RegInfo {
        block_id: 73, block_name: "MDIO Clause 45 Controller",
        reg_name: "MDIO_DATA", access: "RW",
        desc: "Read: 16-bit result, Write: data/addr",
    }),
    (0x01000080, RegInfo {
        block_id: 38, block_name: "IRQ Priority Controller",
        reg_name: "IRQ_PRIORITY_N", access: "RMW",
        desc: "Priorite 3-bit par source IRQ (8 sources/mot)",
    }),
    (0x01000084, RegInfo {
        block_id: 38, block_name: "IRQ Priority Controller",
        reg_name: "IRQ_PRIORITY_1", access: "RMW",
        desc: "IRQ priority register for channels 8-15. Each 3-bit field sets priority for one ",
    }),
    (0x01000088, RegInfo {
        block_id: 38, block_name: "IRQ Priority Controller",
        reg_name: "IRQ_PRIORITY_2", access: "RMW",
        desc: "IRQ priority register for channels 16-23. Each 3-bit field sets priority for one",
    }),
    (0x0100008C, RegInfo {
        block_id: 38, block_name: "IRQ Priority Controller",
        reg_name: "IRQ_PRIORITY_3", access: "RMW",
        desc: "IRQ priority register for channels 24-25 (last group, 26 total IRQ sources). Eac",
    }),
    (0x01000090, RegInfo {
        block_id: 90040, block_name: "IRQ Configuration Shadow Copy",
        reg_name: "IRQ_CONFIG_SHADOW", access: "W",
        desc: "IRQ configuration shadow copy. Written in parallel with DAT_2000e9fc for synchro",
    }),
    (0x01000092, RegInfo {
        block_id: 90040, block_name: "IRQ Configuration Shadow Copy",
        reg_name: "WRITE_READY_STATUS", access: "R",
        desc: "Flash write ready status flag (byte)",
    }),
    (0x01000094, RegInfo {
        block_id: 90040, block_name: "IRQ Configuration Shadow Copy",
        reg_name: "IRQ_CONFIG_SHADOW_1", access: "W",
        desc: "IRQ configuration shadow register 1. Written in parallel with DCCM IRQ config by",
    }),
    (0x01000098, RegInfo {
        block_id: 90040, block_name: "IRQ Configuration Shadow Copy",
        reg_name: "IRQ_CONFIG_SHADOW_2", access: "W",
        desc: "IRQ configuration shadow register 2. Written in parallel with DCCM IRQ config by",
    }),
    (0x0100009C, RegInfo {
        block_id: 90040, block_name: "IRQ Configuration Shadow Copy",
        reg_name: "IRQ_CONFIG_SHADOW_3", access: "W",
        desc: "IRQ configuration shadow register 3. Written in parallel with DCCM IRQ config by",
    }),
    (0x010000A4, RegInfo {
        block_id: 90006, block_name: "Lane Sync Status Register",
        reg_name: "LANE0_SYNC", access: "R",
        desc: "Lane 0 sync status bit",
    }),
    (0x01000120, RegInfo {
        block_id: 69, block_name: "MPCP Slot Config Queue-to-Pin",
        reg_name: "QUEUE_PIN_MAP_I", access: "RMW",
        desc: "Champ 5-bit par index de queue",
    }),
    (0x01000140, RegInfo {
        block_id: 90001, block_name: "Lane State Table (DAT_ram_20032cc8)",
        reg_name: "LANE_CMD_CONFIG", access: "RW",
        desc: "Config command (busy-wait pattern)",
    }),
    (0x01000150, RegInfo {
        block_id: 90001, block_name: "Lane State Table (DAT_ram_20032cc8)",
        reg_name: "LANE_CMD_WRITE", access: "RW",
        desc: "Write command, bit 31=busy",
    }),
    (0x01000180, RegInfo {
        block_id: 5, block_name: "SerDes Lane Configuration",
        reg_name: "PON_LANE_INDEX", access: "RMW",
        desc: "Index lane PON (2 bits)",
    }),
    (0x01000194, RegInfo {
        block_id: 44, block_name: "SerDes Link Status MMIO",
        reg_name: "LANE0_LINK_LOCK", access: "R",
        desc: "Bit 1 = lane 0 link lock status",
    }),
    (0x01000198, RegInfo {
        block_id: 5, block_name: "SerDes Lane Configuration",
        reg_name: "PON_LINK_STATUS_TRIGGER", access: "W",
        desc: "Ecrire bit -> attendre 100ms -> lire pour check link",
    }),
    (0x010001A0, RegInfo {
        block_id: 5, block_name: "SerDes Lane Configuration",
        reg_name: "LLID_BROADCAST_PARAM1", access: "W",
        desc: "LLID broadcast configuration parameter 1. Written with 0xFFFFFFFF by mpcp_set_ll",
    }),
    (0x010001A4, RegInfo {
        block_id: 5, block_name: "SerDes Lane Configuration",
        reg_name: "LLID_BROADCAST_PARAM0", access: "W",
        desc: "LLID broadcast configuration parameter 0. Written with 0xFFFFFFFF by mpcp_set_ll",
    }),
    (0x010001A8, RegInfo {
        block_id: 5, block_name: "SerDes Lane Configuration",
        reg_name: "SERDES_LANE1_BYTE_ENABLE", access: "RMW",
        desc: "SerDes lane 1 per-byte enable/status register. Cleared with AND 0xFEFEFEFE (clea",
    }),
    (0x010001AC, RegInfo {
        block_id: 5, block_name: "SerDes Lane Configuration",
        reg_name: "PON_LANE_CONFIG_GRP0", access: "RMW",
        desc: "Config lane PON groupe 0",
    }),
    (0x010001B0, RegInfo {
        block_id: 5, block_name: "SerDes Lane Configuration",
        reg_name: "PON_LANE_CONFIG_GRP1", access: "RMW",
        desc: "PON lane config group 1. Also used as MPCP pending grants register 1 (odd LLID g",
    }),
    (0x010001B4, RegInfo {
        block_id: 6, block_name: "SerDes PHY Status",
        reg_name: "PON_PHY_STATUS", access: "R",
        desc: "Status PHY direction PON",
    }),
    (0x010001C0, RegInfo {
        block_id: 5, block_name: "SerDes Lane Configuration",
        reg_name: "UNI_LANE_INDEX", access: "RMW",
        desc: "UNI lane index (bits 21:20). Also used by serdes_lane2_init_pon_rx which clears ",
    }),
    (0x010001D4, RegInfo {
        block_id: 44, block_name: "SerDes Link Status MMIO",
        reg_name: "LANE2_LINK_LOCK", access: "R",
        desc: "Bit 1 = lane 2 link lock status",
    }),
    (0x010001D8, RegInfo {
        block_id: 5, block_name: "SerDes Lane Configuration",
        reg_name: "UNI_LINK_STATUS_TRIGGER", access: "W",
        desc: "Ecrire `0x1000000 << lane_idx` -> wait 100ms -> read",
    }),
    (0x010001E8, RegInfo {
        block_id: 5, block_name: "SerDes Lane Configuration",
        reg_name: "SERDES_LANE2_BYTE_ENABLE", access: "RMW",
        desc: "SerDes lane 2 per-byte enable/status register. Cleared with AND 0xFEFEFEFE (clea",
    }),
    (0x010001EC, RegInfo {
        block_id: 5, block_name: "SerDes Lane Configuration",
        reg_name: "UNI_LANE_CONFIG_GRP0", access: "RMW",
        desc: "Config lane UNI groupe 0 (meme layout que PON)",
    }),
    (0x010001F0, RegInfo {
        block_id: 5, block_name: "SerDes Lane Configuration",
        reg_name: "UNI_LANE_CONFIG_GRP1", access: "RMW",
        desc: "Config lane UNI groupe 1",
    }),
    (0x010001F4, RegInfo {
        block_id: 6, block_name: "SerDes PHY Status",
        reg_name: "UNI_PHY_STATUS", access: "R",
        desc: "Status PHY direction UNI",
    }),
    (0x01000200, RegInfo {
        block_id: 9, block_name: "Peripheral Bus Controller (SPI + MDIO)",
        reg_name: "SPI_SPEED_MODE", access: "RMW",
        desc: "Mode vitesse SPI (3 bits)",
    }),
    (0x0100020C, RegInfo {
        block_id: 9, block_name: "Peripheral Bus Controller (SPI + MDIO)",
        reg_name: "SPI_BUSY", access: "R",
        desc: "Bit 0 = SPI flash occupee (busy loop)",
    }),
    (0x01000210, RegInfo {
        block_id: 9, block_name: "Peripheral Bus Controller (SPI + MDIO)",
        reg_name: "SPI_DATA_BUFFER", access: "W",
        desc: "Buffer donnees SPI (ecriture byte-par-byte dans 32-bit word)",
    }),
    (0x0100021C, RegInfo {
        block_id: 10, block_name: "SPI Flash FIFO Controller",
        reg_name: "SPI_FIFO_READ_DATA", access: "R",
        desc: "Donnees lues (compare a `DAT_20011cf8` = JEDEC ID attendu)",
    }),
    (0x01000224, RegInfo {
        block_id: 9, block_name: "Peripheral Bus Controller (SPI + MDIO)",
        reg_name: "SPI_CONFIG", access: "W",
        desc: "Registre configuration SPI (ecrit depuis DAT_20011b00)",
    }),
    (0x01000228, RegInfo {
        block_id: 9, block_name: "Peripheral Bus Controller (SPI + MDIO)",
        reg_name: "MDIO_BUSY", access: "R",
        desc: "Bit 0 = MDIO busy (attente dans `hw_mdio_wait_busy`)",
    }),
    (0x0100022C, RegInfo {
        block_id: 9, block_name: "Peripheral Bus Controller (SPI + MDIO)",
        reg_name: "MDIO_DATA_LOW", access: "W",
        desc: "Donnees MDIO (low word) -- ecrit avant commande write",
    }),
    (0x01000230, RegInfo {
        block_id: 9, block_name: "Peripheral Bus Controller (SPI + MDIO)",
        reg_name: "MDIO_DATA_HIGH", access: "W",
        desc: "Donnees MDIO (high word) -- ecrit avant commande write",
    }),
    (0x01000324, RegInfo {
        block_id: 52, block_name: "MPCP TX Rate Configuration",
        reg_name: "MPCP_TX_RATE_VALUE", access: "W",
        desc: "Valeur debit TX configurable",
    }),
    (0x01000400, RegInfo {
        block_id: 3, block_name: "PHY 1G (Gigabit Ethernet PHY)",
        reg_name: "UNI_PHY_CONTROL", access: "RMW",
        desc: "Registre de controle principal PHY 1G",
    }),
    (0x01000404, RegInfo {
        block_id: 3, block_name: "PHY 1G (Gigabit Ethernet PHY)",
        reg_name: "UNI_PHY_CONFIG_A", access: "W",
        desc: "Config A -- valeur 0x206 pendant init 1G",
    }),
    (0x01000408, RegInfo {
        block_id: 3, block_name: "PHY 1G (Gigabit Ethernet PHY)",
        reg_name: "UNI_PHY_CONFIG_C", access: "W",
        desc: "PHY 1G config register C. Written with value 0x07 during serdes_init_auto_rx. Lo",
    }),
    (0x0100040C, RegInfo {
        block_id: 3, block_name: "PHY 1G (Gigabit Ethernet PHY)",
        reg_name: "UNI_PHY_CONFIG_B", access: "RMW",
        desc: "Config B",
    }),
    (0x01000410, RegInfo {
        block_id: 3, block_name: "PHY 1G (Gigabit Ethernet PHY)",
        reg_name: "UNI_1G_LINK_STATUS", access: "RW",
        desc: "Bit 1 = link UP. Write-1-to-clear (W1C).",
    }),
    (0x01000420, RegInfo {
        block_id: 12, block_name: "MACsec Key Engine",
        reg_name: "KEY_ENGINE_BUSY", access: "RMW",
        desc: "Bit 31 = busy (`macsec_hw_wait_key_engine_ready` loop)",
    }),
    (0x01000424, RegInfo {
        block_id: 12, block_name: "MACsec Key Engine",
        reg_name: "KEY_DATA", access: "W",
        desc: "Donnees cle (ecrit avant commande)",
    }),
    (0x0100043C, RegInfo {
        block_id: 24, block_name: "PON TX Config Table",
        reg_name: "LLID_TX_CONFIG", access: "RW",
        desc: "Config TX par LLID (32 entries, stride 4 bytes, 16 bits useful). Read by hw_pon_",
    }),
    (0x010004D0, RegInfo {
        block_id: 26, block_name: "SerDes Auto RX Block",
        reg_name: "RX_AUTO_ADJUST", access: "RMW",
        desc: "Ajouter 0x1a a valeur existante",
    }),
    (0x01000520, RegInfo {
        block_id: 1402, block_name: "MACsec SA PN Overflow -- 1G Variant",
        reg_name: "SA_1G_PN_OVERFLOW", access: "R",
        desc: "Bitmap overflow PN par SA (bit N = SA index N)",
    }),
    (0x010005E0, RegInfo {
        block_id: 4801, block_name: "MACsec SA Key Slot Counter Block -- 10G SA Counter",
        reg_name: "SA_1G_SLOT_COUNTER", access: "R",
        desc: "Compteur par slot SA (slots >= 8)",
    }),
    (0x01000680, RegInfo {
        block_id: 4801, block_name: "MACsec SA Key Slot Counter Block -- 10G SA Counter",
        reg_name: "SA_10G_SLOT_COUNTER", access: "R",
        desc: "Compteur par slot SA (8 slots, slot < 8)",
    }),
    (0x01000C00, RegInfo {
        block_id: 15, block_name: "MACsec SA Programming 10G",
        reg_name: "SA_MODE_CONFIG", access: "RMW",
        desc: "Mode SA bits 0-1 (clear=disable, 3=enable both)",
    }),
    (0x01000C04, RegInfo {
        block_id: 15, block_name: "MACsec SA Programming 10G",
        reg_name: "SA_CONFIG_2", access: "W",
        desc: "Config secondaire (0xffffffff = all enabled)",
    }),
    (0x01000C0C, RegInfo {
        block_id: 13, block_name: "MACsec PN Threshold Engine",
        reg_name: "PN_THRESHOLD_BUSY", access: "R",
        desc: "Bit 31 = busy (wait while set)",
    }),
    (0x01000C10, RegInfo {
        block_id: 13, block_name: "MACsec PN Threshold Engine",
        reg_name: "PN_THRESHOLD_DATA", access: "W",
        desc: "Donnees seuil (ecrit avant commande)",
    }),
    (0x01000C14, RegInfo {
        block_id: 13, block_name: "MACsec PN Threshold Engine",
        reg_name: "PN_THRESHOLD_RESULT", access: "R",
        desc: "Resultat lecture (apres commande read)",
    }),
    (0x01000C18, RegInfo {
        block_id: 15, block_name: "MACsec SA Programming 10G",
        reg_name: "SA_KEY_WORD_1", access: "W",
        desc: "Mot cle SA 1",
    }),
    (0x01000C1C, RegInfo {
        block_id: 15, block_name: "MACsec SA Programming 10G",
        reg_name: "SA_KEY_WORD_2", access: "W",
        desc: "Mot cle SA 2",
    }),
    (0x01000C20, RegInfo {
        block_id: 15, block_name: "MACsec SA Programming 10G",
        reg_name: "SA_KEY_WORD_3", access: "W",
        desc: "Mot cle SA 3 (mode non-compact seulement)",
    }),
    (0x01000C24, RegInfo {
        block_id: 15, block_name: "MACsec SA Programming 10G",
        reg_name: "SA_NONCE_0", access: "W",
        desc: "Nonce/IV word 0",
    }),
    (0x01000C28, RegInfo {
        block_id: 15, block_name: "MACsec SA Programming 10G",
        reg_name: "SA_NONCE_1", access: "W",
        desc: "Nonce/IV word 1",
    }),
    (0x01000C2C, RegInfo {
        block_id: 15, block_name: "MACsec SA Programming 10G",
        reg_name: "SA_TRIGGER", access: "W",
        desc: "Bit 1 = start SA programming",
    }),
    (0x01000C30, RegInfo {
        block_id: 15, block_name: "MACsec SA Programming 10G",
        reg_name: "SA_EXTRA_PARAM", access: "W",
        desc: "Parametre additionnel SA",
    }),
    (0x01000C74, RegInfo {
        block_id: 1401, block_name: "MACsec SA PN Overflow -- 10G Variant",
        reg_name: "SA_10G_PN_OVERFLOW", access: "R",
        desc: "Bitmap overflow PN par SA (bit N = SA index N)",
    }),
    (0x01000D40, RegInfo {
        block_id: 90070, block_name: "PON LLID RX Config",
        reg_name: "LLID_RX_CONFIG_16", access: "W",
        desc: "LLID 16 RX config (cleared to 0 by hw_pon_clear_llid_rx_config). Array: base + (",
    }),
    (0x01000D44, RegInfo {
        block_id: 90070, block_name: "PON LLID RX Config",
        reg_name: "LLID_RX_CONFIG_17", access: "W",
        desc: "LLID 17 RX config (cleared to 0 by hw_pon_clear_llid_rx_config)",
    }),
    (0x01000D48, RegInfo {
        block_id: 90070, block_name: "PON LLID RX Config",
        reg_name: "LLID_RX_CONFIG_18", access: "W",
        desc: "LLID 18 RX config (cleared to 0 by hw_pon_clear_llid_rx_config)",
    }),
    (0x01000D4C, RegInfo {
        block_id: 90070, block_name: "PON LLID RX Config",
        reg_name: "LLID_RX_CONFIG_19", access: "W",
        desc: "LLID 19 RX config (cleared to 0 by hw_pon_clear_llid_rx_config)",
    }),
    (0x01000D50, RegInfo {
        block_id: 90070, block_name: "PON LLID RX Config",
        reg_name: "LLID_RX_CONFIG_20", access: "W",
        desc: "LLID 20 RX config (cleared to 0 by hw_pon_clear_llid_rx_config)",
    }),
    (0x01000D54, RegInfo {
        block_id: 90070, block_name: "PON LLID RX Config",
        reg_name: "LLID_RX_CONFIG_21", access: "W",
        desc: "LLID 21 RX config (cleared to 0 by hw_pon_clear_llid_rx_config)",
    }),
    (0x01000D58, RegInfo {
        block_id: 90070, block_name: "PON LLID RX Config",
        reg_name: "LLID_RX_CONFIG_22", access: "W",
        desc: "LLID 22 RX config (cleared to 0 by hw_pon_clear_llid_rx_config)",
    }),
    (0x01000D5C, RegInfo {
        block_id: 90070, block_name: "PON LLID RX Config",
        reg_name: "LLID_RX_CONFIG_23", access: "W",
        desc: "LLID 23 RX config (cleared to 0 by hw_pon_clear_llid_rx_config)",
    }),
    (0x01000D60, RegInfo {
        block_id: 90070, block_name: "PON LLID RX Config",
        reg_name: "LLID_RX_CONFIG_24", access: "W",
        desc: "LLID 24 RX config (cleared to 0 by hw_pon_clear_llid_rx_config)",
    }),
    (0x01000D64, RegInfo {
        block_id: 90070, block_name: "PON LLID RX Config",
        reg_name: "LLID_RX_CONFIG_25", access: "W",
        desc: "LLID 25 RX config (cleared to 0 by hw_pon_clear_llid_rx_config)",
    }),
    (0x01000D68, RegInfo {
        block_id: 90070, block_name: "PON LLID RX Config",
        reg_name: "LLID_RX_CONFIG_26", access: "W",
        desc: "LLID 26 RX config (cleared to 0 by hw_pon_clear_llid_rx_config)",
    }),
    (0x01000D6C, RegInfo {
        block_id: 90070, block_name: "PON LLID RX Config",
        reg_name: "LLID_RX_CONFIG_27", access: "W",
        desc: "LLID 27 RX config (cleared to 0 by hw_pon_clear_llid_rx_config)",
    }),
    (0x01000D70, RegInfo {
        block_id: 90070, block_name: "PON LLID RX Config",
        reg_name: "LLID_RX_CONFIG_28", access: "W",
        desc: "LLID 28 RX config (cleared to 0 by hw_pon_clear_llid_rx_config)",
    }),
    (0x01000D74, RegInfo {
        block_id: 90070, block_name: "PON LLID RX Config",
        reg_name: "LLID_RX_CONFIG_29", access: "W",
        desc: "LLID 29 RX config (cleared to 0 by hw_pon_clear_llid_rx_config)",
    }),
    (0x01000D78, RegInfo {
        block_id: 90070, block_name: "PON LLID RX Config",
        reg_name: "LLID_RX_CONFIG_30", access: "W",
        desc: "LLID 30 RX config (cleared to 0 by hw_pon_clear_llid_rx_config)",
    }),
    (0x01000D94, RegInfo {
        block_id: 25, block_name: "SerDes 10G TX Block",
        reg_name: "TX_10G_INIT_FLAG", access: "W",
        desc: "Ecrire 3 pendant init 10G",
    }),
    (0x01000D9C, RegInfo {
        block_id: 25, block_name: "SerDes 10G TX Block",
        reg_name: "TX_10G_MODE_FLAG", access: "W",
        desc: "Ecrire 1 = mode 10G TX",
    }),
    (0x01000E00, RegInfo {
        block_id: 15, block_name: "MACsec SA Programming 10G",
        reg_name: "SA_ENABLE_REGISTER", access: "RMW",
        desc: "Save/restore pendant speed change",
    }),
    (0x01000E04, RegInfo {
        block_id: 4, block_name: "PHY 10G / UNI 10G Link Status",
        reg_name: "UNI_10G_LINK_STATUS", access: "RW",
        desc: "Bit 6 set = 10G LINK UP. Write 0x40 to clear.",
    }),
    (0x01000E0C, RegInfo {
        block_id: 7, block_name: "PON Lane Enable / Speed",
        reg_name: "PON_ENABLE_BIT2", access: "RMW",
        desc: "Bit 2 du registre base -- set par `hw_pon_set_enable_bit2`",
    }),
    (0x01000E40, RegInfo {
        block_id: 7, block_name: "PON Lane Enable / Speed",
        reg_name: "PON_SPEED_MODE", access: "RMW",
        desc: "Bit 0 : 0=10G mode, 1=1G mode",
    }),
    (0x01000E44, RegInfo {
        block_id: 7, block_name: "PON Lane Enable / Speed",
        reg_name: "PON_SPEED_PARAM", access: "W",
        desc: "Parametre vitesse : 4=10G, 0x13=1G",
    }),
    (0x01000E5C, RegInfo {
        block_id: 25, block_name: "SerDes 10G TX Block",
        reg_name: "TX_LANE_PARAM", access: "RMW",
        desc: "Bits [24:16] = 0x11b (lane config)",
    }),
    (0x01000EC0, RegInfo {
        block_id: 4803, block_name: "MACsec SA Key Slot Counter Block -- 1G MACsec Coun",
        reg_name: "MACSEC_1G_CTR_I", access: "R",
        desc: "Compteur MACsec 1G (7 entrees, i=0..6)",
    }),
    (0x01000F80, RegInfo {
        block_id: 15, block_name: "MACsec SA Programming 10G",
        reg_name: "MACSEC_CHANNEL_ENABLE", access: "RMW",
        desc: "Toggle enable channel -- `macsec_hw_10g_set_llid_mask`",
    }),
    (0x01000F84, RegInfo {
        block_id: 15, block_name: "MACsec SA Programming 10G",
        reg_name: "MACSEC_CHANNEL_RESET", access: "W",
        desc: "Ecrire 0xffffffff = reset all channels",
    }),
    (0x01000FB0, RegInfo {
        block_id: 25, block_name: "SerDes 10G TX Block",
        reg_name: "TX_TIMING", access: "W",
        desc: "FEC param << 8",
    }),
    (0x01000FB4, RegInfo {
        block_id: 25, block_name: "SerDes 10G TX Block",
        reg_name: "TX_MODE_SELECT", access: "W",
        desc: "Ecrire 3 pendant init 10G",
    }),
    (0x01000FFC, RegInfo {
        block_id: 1, block_name: "EPON MAC -- Bloc principal",
        reg_name: "CHIP_ID", access: "R",
        desc: "Chip ID register -- BCM4701 = 0x47010203. Lu via `reg 0`.",
    }),
    (0x01001000, RegInfo {
        block_id: 1, block_name: "EPON MAC -- Bloc principal",
        reg_name: "CHIP_REV", access: "R",
        desc: "Chip revision / bond options -- 0xB2110816. Lu via `reg 1`.",
    }),
    (0x01001004, RegInfo {
        block_id: 2, block_name: "EPON RX Configuration",
        reg_name: "RX_FILTER_MODE", access: "RMW",
        desc: "Configure via table de lookup selon param_2",
    }),
    (0x01001008, RegInfo {
        block_id: 1, block_name: "EPON MAC -- Bloc principal",
        reg_name: "LLID_CAPTURE_MASK", access: "RMW",
        desc: "Masque capture LLID -- `hw_counter_set/clear_llid_capture_mask`",
    }),
    (0x01001014, RegInfo {
        block_id: 1, block_name: "EPON MAC -- Bloc principal",
        reg_name: "LLID_ACTIVE_BITMAP", access: "RMW",
        desc: "Bitmap des LLIDs actifs -- `hw_counter_clear_active_llid_bit` clear un bit",
    }),
    (0x0100101C, RegInfo {
        block_id: 1, block_name: "EPON MAC -- Bloc principal",
        reg_name: "LLID_MASK_CONTROL", access: "W",
        desc: "Masque LLID control -- `epon_hw_set_llid_mask_control`",
    }),
    (0x01001020, RegInfo {
        block_id: 1, block_name: "EPON MAC -- Bloc principal",
        reg_name: "LLID_COUNTER_MASK", access: "RMW",
        desc: "Masque compteurs LLID -- set/clear par `hw_counter_set/clear_active_llid_mask`",
    }),
    (0x01001024, RegInfo {
        block_id: 1, block_name: "EPON MAC -- Bloc principal",
        reg_name: "TX_GRANT_MASK", access: "RMW",
        desc: "Masque grants TX -- set par `hw_grant_set_tx_mask`, clear par `hw_grant_clear_tx",
    }),
    (0x0100102C, RegInfo {
        block_id: 1, block_name: "EPON MAC -- Bloc principal",
        reg_name: "RX_GRANT_MASK", access: "RMW",
        desc: "Masque grants RX -- set par `hw_grant_set_rx_mask`, clear par `hw_grant_clear_rx",
    }),
    (0x01001030, RegInfo {
        block_id: 1, block_name: "EPON MAC -- Bloc principal",
        reg_name: "IRQ_MASK", access: "RMW",
        desc: "Masque d'interruption EPON -- `epon_hw_set_interrupt_mask_bits` set bits",
    }),
    (0x0100103C, RegInfo {
        block_id: 16, block_name: "EPON Grant Timing",
        reg_name: "GRANT_TIMING_OFFSET", access: "W",
        desc: "Offset timing grant = 0x54",
    }),
    (0x01001040, RegInfo {
        block_id: 1, block_name: "EPON MAC -- Bloc principal",
        reg_name: "EPON_STATUS", access: "RW",
        desc: "**REGISTRE CRITIQUE** -- Statut EPON MAC",
    }),
    (0x01001050, RegInfo {
        block_id: 1, block_name: "EPON MAC -- Bloc principal",
        reg_name: "ACTIVE_FLAGS", access: "RW",
        desc: "Registre flags actifs -- `hw_epon_read_active_flags_reg`",
    }),
    (0x010010C4, RegInfo {
        block_id: 2, block_name: "EPON RX Configuration",
        reg_name: "SPEED_LIMIT", access: "W",
        desc: "Limite vitesse (11 bits) -- param & 0x7ff",
    }),
    (0x010010D0, RegInfo {
        block_id: 16, block_name: "EPON Grant Timing",
        reg_name: "GRANT_INTERVAL_1", access: "W",
        desc: "Intervalle grant 1 = 0x100",
    }),
    (0x010010D4, RegInfo {
        block_id: 16, block_name: "EPON Grant Timing",
        reg_name: "GRANT_INTERVAL_2", access: "W",
        desc: "Intervalle grant 2 = 0x100",
    }),
    (0x010010D8, RegInfo {
        block_id: 16, block_name: "EPON Grant Timing",
        reg_name: "GRANT_WINDOW_SIZE", access: "W",
        desc: "Taille fenetre grant = 0x40",
    }),
    (0x010010E0, RegInfo {
        block_id: 16, block_name: "EPON Grant Timing",
        reg_name: "GRANT_TIMING_LOW", access: "RMW",
        desc: "Timing low 16 bits = 0xee8",
    }),
    (0x010010E4, RegInfo {
        block_id: 16, block_name: "EPON Grant Timing",
        reg_name: "GRANT_RETRY_COUNT", access: "W",
        desc: "Compteur retry = 2",
    }),
    (0x010010E8, RegInfo {
        block_id: 16, block_name: "EPON Grant Timing",
        reg_name: "GRANT_TIMEOUT", access: "W",
        desc: "Timeout grant = 300 (0x12c)",
    }),
    (0x01001180, RegInfo {
        block_id: 19, block_name: "Grant Queue Enable Table",
        reg_name: "QUEUE_ENABLE", access: "RMW",
        desc: "Bit 31 = queue enable (32 queues max)",
    }),
    (0x01001240, RegInfo {
        block_id: 2, block_name: "EPON RX Configuration",
        reg_name: "RX_EXTENDED_CONFIG", access: "W",
        desc: "Configure dans modes non-standard (param_2 dependant)",
    }),
    (0x01001280, RegInfo {
        block_id: 34, block_name: "EPON Counter Engine",
        reg_name: "CTR_DIRECTION_FLAG", access: "W",
        desc: "Bit 4 = direction (0=downstream, 1=upstream)",
    }),
    (0x01001284, RegInfo {
        block_id: 34, block_name: "EPON Counter Engine",
        reg_name: "CTR_REGISTER_INDEX", access: "W",
        desc: "Index du compteur a lire (calcule par `hw_counter_compute_register_index`)",
    }),
    (0x01001288, RegInfo {
        block_id: 34, block_name: "EPON Counter Engine",
        reg_name: "CTR_DATA_HIGH", access: "R",
        desc: "Partie haute du compteur 64-bit",
    }),
    (0x0100128C, RegInfo {
        block_id: 34, block_name: "EPON Counter Engine",
        reg_name: "CTR_DATA_LOW", access: "R",
        desc: "Partie basse du compteur 64-bit",
    }),
    (0x01001290, RegInfo {
        block_id: 34, block_name: "EPON Counter Engine",
        reg_name: "CTR_DIRECT_READ_I", access: "R",
        desc: "Lecture directe compteur i (6 entrees, i=0..5)",
    }),
    (0x010012BC, RegInfo {
        block_id: 2, block_name: "EPON RX Configuration",
        reg_name: "LLID_ENABLE_TRIGGER", access: "W",
        desc: "Ecrire 1 pour declencher enable LLID",
    }),
    (0x010012C0, RegInfo {
        block_id: 21, block_name: "MPCP Direction / Lane State",
        reg_name: "BW_WINDOW_REG", access: "W",
        desc: "Fenetre BW par lane : [end:start] 13+13 bits",
    }),
    (0x010012C4, RegInfo {
        block_id: 21, block_name: "MPCP Direction / Lane State",
        reg_name: "LLID_MAC_CTRL", access: "W",
        desc: "LLID MAC address control word. Written with 0x0D during MAC registration. Paired",
    }),
    (0x010012C8, RegInfo {
        block_id: 21, block_name: "MPCP Direction / Lane State",
        reg_name: "LLID_MAC_DATA_1", access: "W",
        desc: "LLID MAC address data word for channel 1. Same pattern as LLID_MAC_TABLE at 0x58",
    }),
    (0x01001340, RegInfo {
        block_id: 20, block_name: "EPON LLID Enable Table",
        reg_name: "LLID_ENABLE", access: "RMW",
        desc: "Bit 31 = LLID enable",
    }),
    (0x01001348, RegInfo {
        block_id: 20, block_name: "EPON LLID Enable Table",
        reg_name: "LLID_ENABLE_1", access: "RMW",
        desc: "LLID enable register channel 1. Same as LLID_ENABLE at 0x5c but for next channel",
    }),
    (0x01001380, RegInfo {
        block_id: 55, block_name: "EPON Timing Config Block",
        reg_name: "TIMING_PARAM_WORD1", access: "W",
        desc: "Mot timing 1 (depuis structure config)",
    }),
    (0x01001384, RegInfo {
        block_id: 55, block_name: "EPON Timing Config Block",
        reg_name: "TIMING_PARAM_WORD0", access: "W",
        desc: "Mot timing 0 (depuis structure config)",
    }),
    (0x01001400, RegInfo {
        block_id: 1801, block_name: "Channel DMA / IRQ Controller -- Config Channel",
        reg_name: "CHAN_CONFIG", access: "RMW",
        desc: "Config canal : preserve bit 2, set 0x43",
    }),
    (0x01001404, RegInfo {
        block_id: 36, block_name: "DMA Channel Enable Register",
        reg_name: "CHAN_PACKET_ENABLE", access: "RMW",
        desc: "Bit 8 (0x100) = packet processing enable",
    }),
    (0x01001410, RegInfo {
        block_id: 1801, block_name: "Channel DMA / IRQ Controller -- Config Channel",
        reg_name: "CHAN_IRQ_STATUS", access: "RW",
        desc: "Channel IRQ status register (per sub-channel bit). Read/write status tracking fo",
    }),
    (0x01001428, RegInfo {
        block_id: 1802, block_name: "Channel DMA / IRQ Controller -- IRQ Channel",
        reg_name: "CHAN_IRQ_ENABLE", access: "RMW",
        desc: "Bitmap enable IRQ par canal (OR avec mask)",
    }),
    (0x0100142C, RegInfo {
        block_id: 1802, block_name: "Channel DMA / IRQ Controller -- IRQ Channel",
        reg_name: "CHAN_IRQ_PENDING", access: "RMW",
        desc: "Bitmap pending IRQ par canal (OR avec mask)",
    }),
    (0x0100143C, RegInfo {
        block_id: 74, block_name: "DMA Channel Queue Drain Register",
        reg_name: "SUBCHAN_INDEX", access: "RW",
        desc: "Channel queue drain command register. Also read as status: bit 8 indicates drain",
    }),
    (0x01001480, RegInfo {
        block_id: 37, block_name: "DMA Queue Descriptor Table",
        reg_name: "QUEUE_DESCRIPTOR", access: "RW",
        desc: "Queue descriptor per sub-channel. Write 0 to clear, or write compound config (bi",
    }),
    (0x010015C0, RegInfo {
        block_id: 1803, block_name: "Channel DMA / IRQ Controller -- Mailbox",
        reg_name: "MAILBOX_BUSY", access: "R",
        desc: "Bit 31 = mailbox busy",
    }),
    (0x010015C4, RegInfo {
        block_id: 1803, block_name: "Channel DMA / IRQ Controller -- Mailbox",
        reg_name: "MAILBOX_DATA", access: "W",
        desc: "Donnees mailbox (ecriture)",
    }),
    (0x010015D4, RegInfo {
        block_id: 35, block_name: "DMA Channel Counter Block",
        reg_name: "CHAN_CTR_LATCH_COMMAND", access: "W",
        desc: "Counter latch command register. Bits contain channel and sub-channel index plus ",
    }),
    (0x010015D8, RegInfo {
        block_id: 35, block_name: "DMA Channel Counter Block",
        reg_name: "CHAN_CTR_RESULT", access: "R",
        desc: "Resultat lecture compteur",
    }),
    (0x0100216C, RegInfo {
        block_id: 90007, block_name: "DMA Status Registers",
        reg_name: "CHAN_BUSY_FLAGS", access: "R",
        desc: "Per-channel busy status bits",
    }),
    (0x01002400, RegInfo {
        block_id: 57, block_name: "Lane HW Reset Controller",
        reg_name: "LANE_RESET_BITS", access: "RMW",
        desc: "Clear [1:0] puis set [1:0] = reset sequence",
    }),
    (0x01002404, RegInfo {
        block_id: 62, block_name: "SerDes MDIO Event Controller",
        reg_name: "EVENT_STATUS", access: "RW",
        desc: "Bit 5 = event pending. Ecrire 0x20 pour clear",
    }),
    (0x01002410, RegInfo {
        block_id: 5602, block_name: "Fatal Error Status / Mask -- Fatal Error Mask",
        reg_name: "LANE_ID_CONFIG", access: "R",
        desc: "Lane ID / configuration initiale",
    }),
    (0x01002484, RegInfo {
        block_id: 54, block_name: "SerDes Lane HW Enable",
        reg_name: "LANE_SPEED_MODE", access: "R",
        desc: "Mode vitesse lane (2 bits, read only)",
    }),
    (0x01002488, RegInfo {
        block_id: 54, block_name: "SerDes Lane HW Enable",
        reg_name: "SPEED_CTRL_FIELD", access: "RMW",
        desc: "3-bit speed control par demi-lane",
    }),
    (0x010024C0, RegInfo {
        block_id: 54, block_name: "SerDes Lane HW Enable",
        reg_name: "LANE_HW_ENABLE", access: "RMW",
        desc: "1 = enable, 0 = disable",
    }),
    (0x010024C8, RegInfo {
        block_id: 62, block_name: "SerDes MDIO Event Controller",
        reg_name: "EVENT_DATA_0", access: "R",
        desc: "12-bit address + 2-bit type",
    }),
    (0x010024CC, RegInfo {
        block_id: 62, block_name: "SerDes MDIO Event Controller",
        reg_name: "EVENT_DATA_1", access: "R",
        desc: "Donnees evenement (mot 1)",
    }),
    (0x010024D0, RegInfo {
        block_id: 62, block_name: "SerDes MDIO Event Controller",
        reg_name: "EVENT_DATA_2", access: "R",
        desc: "Donnees evenement (mot 2)",
    }),
    (0x010024D4, RegInfo {
        block_id: 62, block_name: "SerDes MDIO Event Controller",
        reg_name: "EVENT_DATA_3", access: "R",
        desc: "Donnees evenement (mot 3)",
    }),
    (0x010024D8, RegInfo {
        block_id: 62, block_name: "SerDes MDIO Event Controller",
        reg_name: "EVENT_DATA_4", access: "R",
        desc: "Donnees evenement (mot 4)",
    }),
    (0x01002500, RegInfo {
        block_id: 90004, block_name: "SerDes Lane Speed Mode HW (DAT_ram_20035b68)",
        reg_name: "SPEED_MODE_HW", access: "W",
        desc: "Speed mode hardware (2-bit)",
    }),
    (0x01002584, RegInfo {
        block_id: 66, block_name: "MDIO RX FIFO Level",
        reg_name: "RX_FIFO_LEVEL", access: "R",
        desc: "Niveau FIFO RX (12-bit, 0-4095)",
    }),
    (0x01002640, RegInfo {
        block_id: 90005, block_name: "Lane Config Write (DAT_ram_20035db4)",
        reg_name: "LANE_CONFIG_REG", access: "W",
        desc: "Configuration register par lane",
    }),
    (0x01002644, RegInfo {
        block_id: 61, block_name: "MPCP Slot HW Command Engine",
        reg_name: "SLOT_CMD_BUSY", access: "RW",
        desc: "1=operation en cours, 0=termine",
    }),
    (0x01002648, RegInfo {
        block_id: 61, block_name: "MPCP Slot HW Command Engine",
        reg_name: "SLOT_DATA_0", access: "RW",
        desc: "Data word 0 (write avant cmd, read apres)",
    }),
    (0x0100264C, RegInfo {
        block_id: 61, block_name: "MPCP Slot HW Command Engine",
        reg_name: "SLOT_DATA_1", access: "RW",
        desc: "Data word 1",
    }),
    (0x01002804, RegInfo {
        block_id: 5601, block_name: "Fatal Error Status / Mask -- Fatal Error Status",
        reg_name: "FATAL_ERROR_STATUS", access: "R",
        desc: "Bits 0x105c : si non-zero = erreur fatale",
    }),
    (0x01002C00, RegInfo {
        block_id: 11, block_name: "MACsec Control",
        reg_name: "MACSEC_CTRL", access: "RMW",
        desc: "Bit 15 : clear par `macsec_hw_clear_ctrl_bit15` (mask 0xffff7fff)",
    }),
    (0x01002D00, RegInfo {
        block_id: 65, block_name: "SerDes Lane Mode Controller",
        reg_name: "MODE_ENABLE", access: "W",
        desc: "Enable bit (toggle 0→1 = reset sequence)",
    }),
    (0x01002D04, RegInfo {
        block_id: 65, block_name: "SerDes Lane Mode Controller",
        reg_name: "CIPHER_PARAMS", access: "W",
        desc: "Parametres cipher MACsec (si non standard)",
    }),
    (0x01002D18, RegInfo {
        block_id: 65, block_name: "SerDes Lane Mode Controller",
        reg_name: "TIMING_CONFIG", access: "W",
        desc: "Valeur timing/clock depuis FDS record (1,0,6)",
    }),
    (0x01003000, RegInfo {
        block_id: 8, block_name: "VLAN / EtherType Configuration",
        reg_name: "VLAN_CTRL", access: "RMW",
        desc: "Bit 2 : 0=standard 802.1Q (0x8100), 1=custom EtherType",
    }),
    (0x0100300C, RegInfo {
        block_id: 8, block_name: "VLAN / EtherType Configuration",
        reg_name: "CUSTOM_VLAN_ETHERTYPE", access: "RMW",
        desc: "Valeur EtherType personnalisee (16 bits low)",
    }),
    (0x01003010, RegInfo {
        block_id: 8, block_name: "VLAN / EtherType Configuration",
        reg_name: "INDIRECT_CMD", access: "RW",
        desc: "Indirect register access command. Bits [7:0] = register index, bit 30 = write (0",
    }),
    (0x01003014, RegInfo {
        block_id: 8, block_name: "VLAN / EtherType Configuration",
        reg_name: "INDIRECT_DATA_2", access: "RW",
        desc: "Indirect access data word 2 (MSB of 96-bit payload). Written before write comman",
    }),
    (0x01003018, RegInfo {
        block_id: 8, block_name: "VLAN / EtherType Configuration",
        reg_name: "INDIRECT_DATA_1", access: "RW",
        desc: "Indirect access data word 1 (middle of 96-bit payload). Written before write com",
    }),
    (0x0100301C, RegInfo {
        block_id: 8, block_name: "VLAN / EtherType Configuration",
        reg_name: "INDIRECT_DATA_0", access: "RW",
        desc: "Indirect access data word 0 (LSB of 96-bit payload). Written before write comman",
    }),
    (0x01003600, RegInfo {
        block_id: 46, block_name: "Filter/Mask Controller",
        reg_name: "FILTER_CHANNEL_MASK", access: "RMW",
        desc: "Clear bits [3:0] puis set a 0xf = enable all",
    }),
    (0x01003604, RegInfo {
        block_id: 46, block_name: "Filter/Mask Controller",
        reg_name: "FILTER_TABLE_PTR", access: "W",
        desc: "Pointeur table de filtrage (DAT_20011980)",
    }),
    (0x01003800, RegInfo {
        block_id: 67, block_name: "Channel Config Register",
        reg_name: "CHAN_ADDR_COUNT", access: "RMW",
        desc: "Adresse ou compteur 23-bit",
    }),
];

/// Look up an MMIO address in the register database.
/// Returns `Some(RegInfo)` if the address is known, `None` otherwise.
pub fn lookup(addr: u32) -> Option<&'static RegInfo> {
    MMIO_REGISTERS.binary_search_by_key(&addr, |(a, _)| *a).ok()
        .map(|i| &MMIO_REGISTERS[i].1)
}
