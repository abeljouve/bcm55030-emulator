/// BCM55030 D-cache: 4 KB, 2-way, 64 sets, 32 B lines, write-back, round-robin.
/// DC_CTRL (aux 0x48): bit 0=DC (0=enabled), bit 6=IM, bit 7=LM. Reset: 0xC2.
const LINE_SIZE: usize = 32;
const NUM_WAYS: usize = 2;
const NUM_SETS: usize = 64;

const OFFSET_BITS: u32 = 5; // log2(32)
const INDEX_BITS: u32 = 6;  // log2(64)

#[derive(Clone, Copy)]
struct CacheLine {
    valid: bool,
    dirty: bool,
    tag: u32,
    data: [u8; LINE_SIZE],
}

impl CacheLine {
    const fn empty() -> Self {
        Self {
            valid: false,
            dirty: false,
            tag: 0,
            data: [0; LINE_SIZE],
        }
    }
}

/// Dirty cache line eviction data: base address + line contents.
pub struct EvictedLine {
    pub addr: u32,
    pub data: [u8; LINE_SIZE],
}

/// Clone-struct describing one physical D-cache line, exposed so the
/// UI can render cache state without touching the private `CacheLine`
/// type. One entry per (set, way) slot — 128 total for the BCM55030
/// 2-way × 64-set geometry.
#[derive(Clone, Debug)]
pub struct DCacheLineInfo {
    pub set: usize,
    pub way: usize,
    pub valid: bool,
    pub dirty: bool,
    pub tag: u32,
    pub base_addr: u32,
    pub data: [u8; LINE_SIZE],
}

/// R/W bit mask for DC_CTRL verified on real BCM55030 hardware via scan7b.
/// = bits 0 (DC), 1 (reserved but writable), 2 (SB), 5 (AT), 6 (IM), 7 (LM)
pub const DC_CTRL_RW_MASK: u32 = 0xE7;

/// BCM55030 DC_CTRL reset value (verified boot read via scan7b).
pub const DC_CTRL_RESET: u32 = 0xC2;

pub struct DCache {
    // 2-way × 64 sets = 4 KB — box to keep the DCache struct off the stack.
    lines: Box<[[CacheLine; NUM_WAYS]; NUM_SETS]>,
    /// Per-set round-robin counter. Replacement is RR (verified scan7c v7).
    next_way: Box<[u8; NUM_SETS]>,
    // DC_CTRL decoded fields (DC_CTRL raw value preserved in ctrl_raw)
    enabled: bool,
    im: bool,
    lm: bool,
    /// Raw DC_CTRL value (0xE7 masked). Software may set any R/W bit
    /// including reserved bits (SB, AT, bit 1) and read them back.
    ctrl_raw: u32,
    /// DC_RAM_ADDR (aux 0x58): address for direct cache probe.
    ram_addr: u32,
}

pub struct DcacheSaveState {
    lines: Box<[[CacheLine; NUM_WAYS]; NUM_SETS]>,
    next_way: Box<[u8; NUM_SETS]>,
    enabled: bool,
    im: bool,
    lm: bool,
    ctrl_raw: u32,
    ram_addr: u32,
}

impl DCache {
    /// Create a new D-cache matching BCM55030 reset state (DC_CTRL = 0xC2).
    pub fn new() -> Self {
        Self {
            lines: Box::new([[CacheLine::empty(); NUM_WAYS]; NUM_SETS]),
            next_way: Box::new([0; NUM_SETS]),
            enabled: true, // DC bit 0 = 0 → enabled
            im: true,      // IM bit 6 = 1
            lm: true,      // LM bit 7 = 1
            ctrl_raw: DC_CTRL_RESET,
            ram_addr: 0,
        }
    }

    pub fn is_enabled(&self) -> bool {
        self.enabled
    }

    /// Snapshot every physical cache line into a flat vec. Used by
    /// the UI memory-viewer "D-cache state" sub-tab. Always returns
    /// `NUM_SETS * NUM_WAYS = 128` entries. Invalid lines are
    /// reported as `valid: false` so the UI can grey them out.
    pub fn snapshot_lines(&self) -> Vec<DCacheLineInfo> {
        let mut out = Vec::with_capacity(NUM_SETS * NUM_WAYS);
        for set in 0..NUM_SETS {
            for way in 0..NUM_WAYS {
                let line = &self.lines[set][way];
                out.push(DCacheLineInfo {
                    set,
                    way,
                    valid: line.valid,
                    dirty: line.dirty,
                    tag: line.tag,
                    base_addr: Self::base_addr(line.tag, set),
                    data: line.data,
                });
            }
        }
        out
    }

    /// Decompose address into (tag, set_index, byte_offset).
    fn decompose(addr: u32) -> (u32, usize, usize) {
        let offset = (addr & ((1 << OFFSET_BITS) - 1)) as usize;
        let index = ((addr >> OFFSET_BITS) & ((1 << INDEX_BITS) - 1)) as usize;
        let tag = addr >> (OFFSET_BITS + INDEX_BITS);
        (tag, index, offset)
    }

    /// Reconstruct base address from tag and set index.
    fn base_addr(tag: u32, set_index: usize) -> u32 {
        (tag << (OFFSET_BITS + INDEX_BITS)) | ((set_index as u32) << OFFSET_BITS)
    }

    /// Find the way that holds the given tag in a set, if any.
    fn find_way(&self, set: usize, tag: u32) -> Option<usize> {
        for way in 0..NUM_WAYS {
            if self.lines[set][way].valid && self.lines[set][way].tag == tag {
                return Some(way);
            }
        }
        None
    }

    /// Select the next victim way in a set using round-robin.
    /// Prefers invalid ways first (so the counter only advances on an
    /// actual eviction of a valid line). Advances the counter afterwards.
    fn rr_victim(&mut self, set: usize) -> usize {
        for way in 0..NUM_WAYS {
            if !self.lines[set][way].valid {
                return way;
            }
        }
        let victim = self.next_way[set] as usize;
        self.next_way[set] = ((victim + 1) % NUM_WAYS) as u8;
        victim
    }

    /// Check if addr is cached without changing cache state.
    pub fn contains(&self, addr: u32) -> bool {
        let (tag, set, _) = Self::decompose(addr);
        self.find_way(set, tag).is_some()
    }

    /// Read a byte from the cache without touching replacement state.
    pub fn peek_byte(&self, addr: u32) -> Option<u8> {
        let (tag, set, offset) = Self::decompose(addr);
        if let Some(way) = self.find_way(set, tag) {
            Some(self.lines[set][way].data[offset])
        } else {
            None
        }
    }

    /// Read a byte from the cache. Returns Some(byte) on hit, None on miss.
    /// RR replacement does not care about reads, so hit state is untouched.
    pub fn read_byte(&mut self, addr: u32) -> Option<u8> {
        let (tag, set, offset) = Self::decompose(addr);
        self.find_way(set, tag)
            .map(|way| self.lines[set][way].data[offset])
    }

    /// Write a byte to the cache. Returns true on hit (line updated + marked dirty),
    /// false on miss (caller must do write-allocate: fill_line then retry).
    pub fn write_byte(&mut self, addr: u32, val: u8) -> bool {
        let (tag, set, offset) = Self::decompose(addr);
        if let Some(way) = self.find_way(set, tag) {
            self.lines[set][way].data[offset] = val;
            self.lines[set][way].dirty = true;
            true
        } else {
            false
        }
    }

    /// Fill a cache line with data from backing store.
    /// If the victim way holds a dirty line, it is returned for writeback.
    /// After fill, the new line is valid and clean. The round-robin counter
    /// advances only when an actual eviction happens.
    pub fn fill_line(&mut self, addr: u32, data: &[u8; LINE_SIZE]) -> Option<EvictedLine> {
        let (tag, set, _offset) = Self::decompose(addr);

        if self.find_way(set, tag).is_some() {
            return None;
        }

        let victim_way = self.rr_victim(set);
        let evicted = if self.lines[set][victim_way].valid && self.lines[set][victim_way].dirty {
            let victim = &self.lines[set][victim_way];
            Some(EvictedLine {
                addr: Self::base_addr(victim.tag, set),
                data: victim.data,
            })
        } else {
            None
        };

        self.lines[set][victim_way] = CacheLine {
            valid: true,
            dirty: false,
            tag,
            data: *data,
        };

        evicted
    }

    /// Invalidate the entire cache.
    /// If `flush_dirty` is true (IM=1), dirty lines are returned for writeback.
    /// Returns all evicted dirty lines.
    pub fn invalidate_all(&mut self) -> Vec<EvictedLine> {
        let flush_dirty = self.im;
        let mut evicted = Vec::new();
        for set in 0..NUM_SETS {
            for way in 0..NUM_WAYS {
                let line = &self.lines[set][way];
                if line.valid && line.dirty && flush_dirty {
                    evicted.push(EvictedLine {
                        addr: Self::base_addr(line.tag, set),
                        data: line.data,
                    });
                }
                self.lines[set][way] = CacheLine::empty();
            }
            self.next_way[set] = 0;
        }
        evicted
    }

    /// Invalidate a single cache line by address.
    /// If the line is dirty and IM=1, returns it for writeback.
    pub fn invalidate_line(&mut self, addr: u32) -> Option<EvictedLine> {
        let flush_dirty = self.im;
        let (tag, set, _) = Self::decompose(addr);
        if let Some(way) = self.find_way(set, tag) {
            let line = &self.lines[set][way];
            let evicted = if line.dirty && flush_dirty {
                Some(EvictedLine {
                    addr: Self::base_addr(line.tag, set),
                    data: line.data,
                })
            } else {
                None
            };
            self.lines[set][way] = CacheLine::empty();
            evicted
        } else {
            None
        }
    }

    /// Coherence action for a DMA-originated SRAM write covering this
    /// line. On real BCM55030 a direct/DMA SRAM write does **not**
    /// disturb a *dirty* cached line — the CPU's pending write shadows
    /// SRAM and subsequent cached reads still return the cached value
    /// (HW-verified, scan7b test 7; same class as a `.di` store). The
    /// emulator previously evicted the line unconditionally, dropping
    /// the firmware's cached saved-return slot when a flash-read
    /// destination shared a 32-byte line with a live stack word
    /// (bug `emu-reboot-halt-and-transfer-model-divergences` D2).
    ///
    /// Behaviour:
    /// - line present and **dirty** → left intact (CPU value wins).
    /// - line present and **clean** → invalidated, so the next cached
    ///   read refills from the freshly DMA-written SRAM.
    ///
    /// Returns `true` when a dirty line was preserved (for tracing).
    pub fn dma_sram_overwrite(&mut self, addr: u32) -> bool {
        let (tag, set, _) = Self::decompose(addr);
        if let Some(way) = self.find_way(set, tag) {
            if self.lines[set][way].dirty {
                return true;
            }
            self.lines[set][way] = CacheLine::empty();
        }
        false
    }

    /// Read DC_CTRL register value (aux 0x48).
    /// Returns the raw stored value (masked by DC_CTRL_RW_MASK on write).
    pub fn read_dc_ctrl(&self) -> u32 {
        self.ctrl_raw
    }

    /// Write DC_CTRL register (aux 0x48). Only R/W bits (mask 0xE7) stick.
    /// Updates enabled/IM/LM flags from the new value.
    pub fn write_dc_ctrl(&mut self, val: u32) {
        self.ctrl_raw = val & DC_CTRL_RW_MASK;
        self.enabled = self.ctrl_raw & (1 << 0) == 0; // DC bit
        self.im = self.ctrl_raw & (1 << 6) != 0;
        self.lm = self.ctrl_raw & (1 << 7) != 0;
    }

    // ========== DC_RAM_ADDR / DC_TAG / DC_DATA (direct cache probe) ==========

    /// Set DC_RAM_ADDR (aux 0x58): address for direct cache probe.
    pub fn set_ram_addr(&mut self, addr: u32) {
        self.ram_addr = addr;
    }

    /// DC_TAG (aux 0x59): bit 0 = valid, bits[31:5] = line base.
    pub fn read_tag(&self) -> u32 {
        let (tag, set, _) = Self::decompose(self.ram_addr);
        if self.find_way(set, tag).is_some() {
            let base = Self::base_addr(tag, set);
            base | 1  // valid bit set
        } else {
            // Not present: return line-aligned address with valid=0
            self.ram_addr & !((LINE_SIZE as u32) - 1)
        }
    }

    /// Read DC_DATA (aux 0x5B) for the probe address.
    /// Returns the 32-bit word at the probe address within the cached line.
    pub fn read_data(&self) -> u32 {
        let (tag, set, offset) = Self::decompose(self.ram_addr);
        if let Some(way) = self.find_way(set, tag) {
            let line = &self.lines[set][way];
            // Word within the line (4-byte aligned by masking offset)
            let word_off = offset & !3;
            ((line.data[word_off] as u32) << 24)
                | ((line.data[word_off + 1] as u32) << 16)
                | ((line.data[word_off + 2] as u32) << 8)
                | (line.data[word_off + 3] as u32)
        } else {
            0
        }
    }

    pub fn save_state(&self) -> DcacheSaveState {
        DcacheSaveState {
            lines: self.lines.clone(),
            next_way: self.next_way.clone(),
            enabled: self.enabled,
            im: self.im,
            lm: self.lm,
            ctrl_raw: self.ctrl_raw,
            ram_addr: self.ram_addr,
        }
    }

    pub fn restore_state(&mut self, state: DcacheSaveState) {
        self.lines = state.lines;
        self.next_way = state.next_way;
        self.enabled = state.enabled;
        self.im = state.im;
        self.lm = state.lm;
        self.ctrl_raw = state.ctrl_raw;
        self.ram_addr = state.ram_addr;
    }
}

// BCM55030 I-cache: 4 KB, 1-way direct, 128 sets, 32 B lines.
// IC_IVIL is a HW no-op — only IC_IVIC flushes. `.di` stores skip I-cache.
pub const IC_LINE_SIZE: usize = 32;
pub const IC_NUM_WAYS: usize = 1;
pub const IC_NUM_SETS: usize = 128;

const IC_OFFSET_BITS: u32 = 5; // log2(32)
const IC_INDEX_BITS: u32 = 7;  // log2(128)

#[derive(Clone, Copy)]
struct ICacheLine {
    valid: bool,
    tag: u32,
    data: [u8; IC_LINE_SIZE],
}

impl ICacheLine {
    const fn empty() -> Self {
        Self {
            valid: false,
            tag: 0,
            data: [0; IC_LINE_SIZE],
        }
    }
}

pub struct ICache {
    // 4 KB: 128 sets × 1 way. Box to keep the struct off the stack.
    lines: Box<[ICacheLine; IC_NUM_SETS]>,
    enabled: bool,
}

pub struct IcacheSaveState {
    lines: Box<[ICacheLine; IC_NUM_SETS]>,
    enabled: bool,
}

impl ICache {
    pub fn new() -> Self {
        Self {
            lines: Box::new([ICacheLine::empty(); IC_NUM_SETS]),
            enabled: true,
        }
    }

    pub fn is_enabled(&self) -> bool {
        self.enabled
    }

    pub fn set_enabled(&mut self, on: bool) {
        self.enabled = on;
    }

    fn decompose(addr: u32) -> (u32, usize, usize) {
        let offset = (addr & ((1 << IC_OFFSET_BITS) - 1)) as usize;
        let index = ((addr >> IC_OFFSET_BITS) & ((1 << IC_INDEX_BITS) - 1)) as usize;
        let tag = addr >> (IC_OFFSET_BITS + IC_INDEX_BITS);
        (tag, index, offset)
    }

    /// Peek a halfword from the I-cache, or None on miss.
    pub fn peek_half(&self, addr: u32) -> Option<u16> {
        let (tag, set, offset) = Self::decompose(addr);
        let line = &self.lines[set];
        if line.valid && line.tag == tag {
            Some(((line.data[offset] as u16) << 8) | (line.data[offset + 1] as u16))
        } else {
            None
        }
    }

    /// Peek a word from the I-cache, or None on miss.
    pub fn peek_word(&self, addr: u32) -> Option<u32> {
        let (tag, set, offset) = Self::decompose(addr);
        let line = &self.lines[set];
        if line.valid && line.tag == tag {
            Some(
                ((line.data[offset] as u32) << 24)
                    | ((line.data[offset + 1] as u32) << 16)
                    | ((line.data[offset + 2] as u32) << 8)
                    | (line.data[offset + 3] as u32),
            )
        } else {
            None
        }
    }

    /// Check if address is cached.
    pub fn contains(&self, addr: u32) -> bool {
        let (tag, set, _) = Self::decompose(addr);
        let line = &self.lines[set];
        line.valid && line.tag == tag
    }

    /// Fill the (single) line for this address. Direct-mapped, so any
    /// existing occupant is unconditionally replaced.
    pub fn fill_line(&mut self, addr: u32, data: &[u8; IC_LINE_SIZE]) {
        let (tag, set, _) = Self::decompose(addr);
        self.lines[set] = ICacheLine {
            valid: true,
            tag,
            data: *data,
        };
    }

    /// Invalidate the entire I-cache (IC_IVIC aux 0x10).
    pub fn invalidate_all(&mut self) {
        for set in 0..IC_NUM_SETS {
            self.lines[set] = ICacheLine::empty();
        }
    }

    /// IC_IVIL (aux 0x19) single-line invalidate: NO-OP on BCM55030.
    /// scan7d sanity test showed the cached line survives an IC_IVIL,
    /// so we model it as a no-op to match hardware.
    pub fn invalidate_line(&mut self, addr: u32) {
        let _ = addr;
    }

    pub fn save_state(&self) -> IcacheSaveState {
        IcacheSaveState {
            lines: self.lines.clone(),
            enabled: self.enabled,
        }
    }

    pub fn restore_state(&mut self, state: IcacheSaveState) {
        self.lines = state.lines;
        self.enabled = state.enabled;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_cache_miss_then_fill_then_hit() {
        let mut cache = DCache::new();
        let addr = 0x1000u32;

        // Miss on empty cache
        assert!(cache.read_byte(addr).is_none());

        // Fill the line
        let mut line_data = [0u8; LINE_SIZE];
        line_data[0] = 0xAB;
        let evicted = cache.fill_line(addr, &line_data);
        assert!(evicted.is_none()); // no eviction on empty cache

        // Hit
        assert_eq!(cache.read_byte(addr), Some(0xAB));
    }

    #[test]
    fn test_write_hit_marks_dirty() {
        let mut cache = DCache::new();
        let addr = 0x2000u32;

        // Fill
        let line_data = [0u8; LINE_SIZE];
        cache.fill_line(addr, &line_data);

        // Write hit
        assert!(cache.write_byte(addr, 0xFF));
        assert_eq!(cache.read_byte(addr), Some(0xFF));

        // Verify dirty on invalidation with flush
        let evicted = cache.invalidate_all();
        assert_eq!(evicted.len(), 1);
        assert_eq!(evicted[0].data[0], 0xFF);
    }

    #[test]
    fn test_write_miss_returns_false() {
        let mut cache = DCache::new();
        assert!(!cache.write_byte(0x3000, 0x42));
    }

    #[test]
    fn test_eviction_on_full_set() {
        // 2-way round-robin: fill both ways of set 1, then a third fill
        // must evict way 0 (the first one filled = counter starts at 0).
        let mut cache = DCache::new();
        let base_set_index = 1usize;

        for way in 0..NUM_WAYS {
            let addr = ((way as u32 + 1) << (OFFSET_BITS + INDEX_BITS))
                | ((base_set_index as u32) << OFFSET_BITS);
            let mut data = [0u8; LINE_SIZE];
            data[0] = way as u8;
            cache.fill_line(addr, &data);
            // Make it dirty so eviction produces a writeback.
            cache.write_byte(addr, way as u8 + 0x10);
        }

        let new_tag = (NUM_WAYS as u32) + 1;
        let new_addr = (new_tag << (OFFSET_BITS + INDEX_BITS))
            | ((base_set_index as u32) << OFFSET_BITS);
        let new_data = [0xEE; LINE_SIZE];
        let evicted = cache.fill_line(new_addr, &new_data).expect("eviction expected");
        // RR evicts way 0 first; its dirty byte is 0x10.
        assert_eq!(evicted.data[0], 0x10);
    }

    #[test]
    fn test_rr_cycles_through_ways() {
        // Sequentially filling N+1 distinct tags in one set evicts way 0,
        // then way 1, then way 0 again.
        let mut cache = DCache::new();
        let set_index: u32 = 3;
        let addr_for = |tag: u32| -> u32 {
            (tag << (OFFSET_BITS + INDEX_BITS)) | (set_index << OFFSET_BITS)
        };

        let mut data = [0u8; LINE_SIZE];
        for tag in 0..(NUM_WAYS as u32) {
            data[0] = tag as u8 + 1;
            cache.fill_line(addr_for(tag), &data);
            cache.write_byte(addr_for(tag), tag as u8 + 1);
        }

        // NUM_WAYS-th insert -> evicts way 0 (tag 0).
        data[0] = 0xAA;
        cache.fill_line(addr_for(NUM_WAYS as u32), &data);
        cache.write_byte(addr_for(NUM_WAYS as u32), 0xAA);
        // (fill evicted tag 0 way 0; check via next eviction)

        // Next insert -> evicts way 1 (tag 1) per RR.
        data[0] = 0xBB;
        cache.fill_line(addr_for(NUM_WAYS as u32 + 1), &data);
        cache.write_byte(addr_for(NUM_WAYS as u32 + 1), 0xBB);

        // Third extra insert -> back to way 0, which now holds the
        // NUM_WAYS-th fill (dirty byte 0xAA).
        data[0] = 0xCC;
        let ev = cache
            .fill_line(addr_for(NUM_WAYS as u32 + 2), &data)
            .expect("third eviction");
        assert_eq!(ev.data[0], 0xAA);
    }

    #[test]
    fn test_write_hit_does_not_protect_line() {
        // scan7c v7 verified: a cached_write hit on L0 does NOT save it
        // from being the next RR victim. Touches are irrelevant.
        let mut cache = DCache::new();
        let set_index: u32 = 5;
        let addr_for = |tag: u32| -> u32 {
            (tag << (OFFSET_BITS + INDEX_BITS)) | (set_index << OFFSET_BITS)
        };

        cache.fill_line(addr_for(0), &[0xA0; LINE_SIZE]); // way 0
        cache.fill_line(addr_for(1), &[0xA1; LINE_SIZE]); // way 1
        // Write-hit to L0 (the future RR victim).
        assert!(cache.write_byte(addr_for(0), 0xFF));
        // Insert a third line -> RR picks way 0 regardless of the touch.
        let ev = cache
            .fill_line(addr_for(2), &[0xA2; LINE_SIZE])
            .expect("eviction expected");
        assert_eq!(ev.addr, addr_for(0));
        assert_eq!(ev.data[0], 0xFF); // dirty byte from the touch
    }

    #[test]
    fn test_invalidate_all_with_im() {
        let mut cache = DCache::new();
        assert!(cache.im); // IM=1 by default (0xC2)

        // Fill and dirty two lines in different sets
        let addr1 = 0x1000u32;
        let addr2 = 0x2000u32;
        cache.fill_line(addr1, &[0x11; LINE_SIZE]);
        cache.fill_line(addr2, &[0x22; LINE_SIZE]);
        cache.write_byte(addr1, 0xAA);
        cache.write_byte(addr2, 0xBB);

        let evicted = cache.invalidate_all();
        assert_eq!(evicted.len(), 2);

        // Cache should be empty now
        assert!(cache.read_byte(addr1).is_none());
        assert!(cache.read_byte(addr2).is_none());
    }

    #[test]
    fn test_invalidate_all_without_im() {
        let mut cache = DCache::new();
        cache.im = false; // IM=0: invalidate only, don't flush dirty

        let addr = 0x1000u32;
        cache.fill_line(addr, &[0x11; LINE_SIZE]);
        cache.write_byte(addr, 0xAA);

        let evicted = cache.invalidate_all();
        assert_eq!(evicted.len(), 0); // dirty lines discarded, not flushed

        assert!(cache.read_byte(addr).is_none());
    }

    #[test]
    fn test_dc_ctrl_read_write() {
        let mut cache = DCache::new();

        // Default: 0xC2 = enabled, IM=1, LM=1, bit1=1
        assert_eq!(cache.read_dc_ctrl(), 0xC2);
        assert!(cache.is_enabled());

        // Write all 1s — only 0xE7 bits stick (RW mask)
        cache.write_dc_ctrl(0xFFFFFFFF);
        assert_eq!(cache.read_dc_ctrl(), 0xE7);
        assert!(!cache.is_enabled()); // DC bit set → disabled

        // Write 0x00: all cleared
        cache.write_dc_ctrl(0x00);
        assert_eq!(cache.read_dc_ctrl(), 0x00);
        assert!(cache.is_enabled()); // DC bit 0 → enabled
        assert!(!cache.im);
        assert!(!cache.lm);

        // Non-writable bits (3, 4, 8-31) ignored
        cache.write_dc_ctrl(0x18); // bits 3+4 (not in 0xE7)
        assert_eq!(cache.read_dc_ctrl(), 0x00);
    }

    #[test]
    fn test_invalidate_single_line() {
        let mut cache = DCache::new();
        // 32-byte lines: use addresses >= 32 apart to ensure different lines.
        let addr1 = 0x1000u32;
        let addr2 = 0x1020u32; // different 32B line

        cache.fill_line(addr1, &[0x11; LINE_SIZE]);
        cache.fill_line(addr2, &[0x22; LINE_SIZE]);
        cache.write_byte(addr1, 0xAA);

        // Invalidate only addr1
        let evicted = cache.invalidate_line(addr1);
        assert!(evicted.is_some());
        assert_eq!(evicted.unwrap().data[0], 0xAA);

        // addr1 is gone, addr2 still there
        assert!(cache.read_byte(addr1).is_none());
        assert_eq!(cache.read_byte(addr2), Some(0x22));
    }

    #[test]
    fn test_dc_tag_data_probe() {
        let mut cache = DCache::new();
        let addr = 0x67000u32;
        let mut data = [0u8; LINE_SIZE];
        data[0] = 0xCA; data[1] = 0xFE; data[2] = 0xBA; data[3] = 0xBE;
        cache.fill_line(addr, &data);

        // Probe cached line
        cache.set_ram_addr(addr);
        let tag = cache.read_tag();
        assert_eq!(tag & 1, 1);                   // valid bit
        assert_eq!(tag & !0x1F, addr);            // line base address

        let cache_data = cache.read_data();
        assert_eq!(cache_data, 0xCAFEBABE);

        // Probe uncached line: valid bit = 0
        cache.set_ram_addr(0xE0000);
        let tag = cache.read_tag();
        assert_eq!(tag & 1, 0);                   // not valid
        assert_eq!(cache.read_data(), 0);
    }

    #[test]
    fn test_icache_basic() {
        let mut ic = ICache::new();
        let addr = 0x1000u32;

        assert!(ic.peek_word(addr).is_none());

        let mut data = [0u8; IC_LINE_SIZE];
        data[0] = 0xAB; data[1] = 0xCD; data[2] = 0xEF; data[3] = 0x12;
        ic.fill_line(addr, &data);

        assert_eq!(ic.peek_word(addr), Some(0xABCDEF12));
        assert_eq!(ic.peek_half(addr), Some(0xABCD));
        assert_eq!(ic.peek_half(addr + 2), Some(0xEF12));

        // IC_IVIL is a no-op per scan7d: the line must still be cached.
        ic.invalidate_line(addr);
        assert_eq!(ic.peek_word(addr), Some(0xABCDEF12));

        // IC_IVIC does the real invalidation.
        ic.invalidate_all();
        assert!(ic.peek_word(addr).is_none());
    }

    #[test]
    fn test_icache_direct_mapped_collision() {
        // Two addresses mapping to the same set under 1-way direct-mapped:
        // the second fill must displace the first.
        let mut ic = ICache::new();
        let stride: u32 = (IC_NUM_SETS as u32) << IC_OFFSET_BITS;
        let addr_a = 0x1000u32;
        let addr_b = 0x1000u32 + stride; // same set, different tag

        ic.fill_line(addr_a, &[0xAA; IC_LINE_SIZE]);
        assert!(ic.contains(addr_a));

        ic.fill_line(addr_b, &[0xBB; IC_LINE_SIZE]);
        assert!(!ic.contains(addr_a));
        assert!(ic.contains(addr_b));
    }

    #[test]
    fn test_icache_invalidate_all() {
        let mut ic = ICache::new();
        let addr1 = 0x1000u32;
        let addr2 = 0x2000u32;
        ic.fill_line(addr1, &[0x11; IC_LINE_SIZE]);
        ic.fill_line(addr2, &[0x22; IC_LINE_SIZE]);

        ic.invalidate_all();
        assert!(ic.peek_word(addr1).is_none());
        assert!(ic.peek_word(addr2).is_none());
    }

    #[test]
    fn test_address_decomposition() {
        // 32B lines, 64 sets:
        //   offset = bits [4:0] (5 bits, 32 values)
        //   set    = bits [10:5] (6 bits, 64 values)
        //   tag    = bits [31:11]
        let addr = 0x1234u32;
        let (tag, set, offset) = DCache::decompose(addr);
        assert_eq!(offset, (addr & 0x1F) as usize);         // 0x14
        assert_eq!(set, ((addr >> 5) & 0x3F) as usize);     // (0x91 & 0x3F) = 0x11
        assert_eq!(tag, addr >> 11);                         // 0x2

        let base = DCache::base_addr(tag, set);
        assert_eq!(base, addr & !0x1F); // line-aligned
    }
}
