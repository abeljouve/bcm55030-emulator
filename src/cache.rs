/// ARC700 D-cache model for BCM55030.
///
/// BCR 0x72 (D_CACHE_BUILD) = 0x00013001:
///   4 KB total, 8-way set-associative, 16-byte cache lines.
///   32 sets x 8 ways = 256 cache lines.
///
/// Address decomposition (16-byte lines, 32 sets):
///   bits [3:0]  = byte offset within line (4 bits)
///   bits [8:4]  = set index (5 bits)
///   bits [31:9] = tag (23 bits)
///
/// Write policy: write-back, write-allocate.
/// Replacement: pseudo-LRU via per-way counters.
///
/// DC_CTRL (aux 0x48) bit layout:
///   bit 0 (DC): 0 = cache enabled, 1 = cache disabled
///   bit 6 (IM): invalidate mode (0 = invalidate only, 1 = invalidate + flush dirty)
///   bit 7 (LM): lock mode (0 = no flush locked, 1 = flush locked)
///   BCM55030 reset value: 0xC2 (enabled, IM=1, LM=1, bit1=1)

// Verified on real HW via scan7b (2026-04-12):
//   Line size: 32 bytes (DC_IVDL refill test)
//   Capacity:  8 KB (4096 no-evict, 8192 evict)
//   Associativity: 4-way (stride=2048 evict at n=5)
//   Sets: 8192 / (4 × 32) = 64
const LINE_SIZE: usize = 32;
const NUM_WAYS: usize = 4;
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

/// R/W bit mask for DC_CTRL verified on real BCM55030 hardware via scan7b.
/// = bits 0 (DC), 1 (reserved but writable), 2 (SB), 5 (AT), 6 (IM), 7 (LM)
pub const DC_CTRL_RW_MASK: u32 = 0xE7;

/// BCM55030 DC_CTRL reset value (verified boot read via scan7b).
pub const DC_CTRL_RESET: u32 = 0xC2;

pub struct DCache {
    // 4-way × 64 sets ≈ 9 KB — box to avoid huge stack frames at construction.
    lines: Box<[[CacheLine; NUM_WAYS]; NUM_SETS]>,
    /// Per-way counter for pseudo-LRU replacement within each set.
    /// Lower value = used more recently. On access, set to 0 and increment others.
    lru: Box<[[u8; NUM_WAYS]; NUM_SETS]>,
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

impl DCache {
    /// Create a new D-cache matching BCM55030 reset state (DC_CTRL = 0xC2).
    pub fn new() -> Self {
        Self {
            lines: Box::new([[CacheLine::empty(); NUM_WAYS]; NUM_SETS]),
            lru: Box::new([[0; NUM_WAYS]; NUM_SETS]),
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

    /// Update LRU counters: mark `way` as most recently used in `set`.
    fn touch_lru(&mut self, set: usize, way: usize) {
        let old = self.lru[set][way];
        for w in 0..NUM_WAYS {
            if self.lru[set][w] < old {
                self.lru[set][w] = self.lru[set][w].saturating_add(1);
            }
        }
        self.lru[set][way] = 0;
    }

    /// Find the LRU victim way in a set (highest counter value).
    fn lru_victim(&self, set: usize) -> usize {
        let mut victim = 0;
        let mut max_val = 0;
        for way in 0..NUM_WAYS {
            // Prefer invalid lines first
            if !self.lines[set][way].valid {
                return way;
            }
            if self.lru[set][way] > max_val {
                max_val = self.lru[set][way];
                victim = way;
            }
        }
        victim
    }

    /// Check if addr is cached without updating LRU state.
    pub fn contains(&self, addr: u32) -> bool {
        let (tag, set, _) = Self::decompose(addr);
        self.find_way(set, tag).is_some()
    }

    /// Read a byte from the cache without updating LRU.
    /// Used by direct SRAM access paths (hooks, DMA) to maintain coherence
    /// with data written through the D-cache.
    pub fn peek_byte(&self, addr: u32) -> Option<u8> {
        let (tag, set, offset) = Self::decompose(addr);
        if let Some(way) = self.find_way(set, tag) {
            Some(self.lines[set][way].data[offset])
        } else {
            None
        }
    }

    /// Read a byte from the cache. Returns Some(byte) on hit, None on miss.
    /// Updates LRU on hit.
    pub fn read_byte(&mut self, addr: u32) -> Option<u8> {
        let (tag, set, offset) = Self::decompose(addr);
        if let Some(way) = self.find_way(set, tag) {
            self.touch_lru(set, way);
            Some(self.lines[set][way].data[offset])
        } else {
            None
        }
    }

    /// Write a byte to the cache. Returns true on hit (line updated + marked dirty),
    /// false on miss (caller must do write-allocate: fill_line then retry).
    pub fn write_byte(&mut self, addr: u32, val: u8) -> bool {
        let (tag, set, offset) = Self::decompose(addr);
        if let Some(way) = self.find_way(set, tag) {
            self.lines[set][way].data[offset] = val;
            self.lines[set][way].dirty = true;
            self.touch_lru(set, way);
            true
        } else {
            false
        }
    }

    /// Fill a cache line with data from backing store.
    /// If the victim way holds a dirty line, it is returned for writeback.
    /// After fill, the new line is valid, clean, and marked MRU.
    pub fn fill_line(&mut self, addr: u32, data: &[u8; LINE_SIZE]) -> Option<EvictedLine> {
        let (tag, set, _offset) = Self::decompose(addr);

        // Don't double-allocate if already present
        if self.find_way(set, tag).is_some() {
            return None;
        }

        let victim_way = self.lru_victim(set);
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
        self.touch_lru(set, victim_way);

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
            self.lru[set] = [0; NUM_WAYS];
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

    /// Flush a single dirty cache line to backing store WITHOUT invalidating.
    /// Returns the dirty line data if the line was dirty; the line stays
    /// valid in the cache with dirty=false after the flush.
    ///
    /// Corresponds to DC_FLSH (aux 0x4B). Per scan7b on real BCM55030,
    /// this opcode showed no observable effect. We implement it for
    /// software compatibility regardless.
    pub fn flush_line(&mut self, addr: u32) -> Option<EvictedLine> {
        let (tag, set, _) = Self::decompose(addr);
        if let Some(way) = self.find_way(set, tag) {
            if self.lines[set][way].dirty {
                let evicted = EvictedLine {
                    addr: Self::base_addr(tag, set),
                    data: self.lines[set][way].data,
                };
                self.lines[set][way].dirty = false;
                return Some(evicted);
            }
        }
        None
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

    /// Read DC_TAG (aux 0x59) for the probe address.
    /// Format verified on real HW (scan7b test 8):
    ///   (line_base_address) | valid_bit
    ///   bit 0 = valid
    ///   bits [31:5] = line-aligned address
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
}

// ========== I-cache ==========
//
// BCR 0x77 (I_CACHE_BUILD) = 0x00023001:
//   ver=1, cfg=0, ll=0 (32 bytes), ways=3 (4-way), cap=2 (16 KB)
// Sets: 16384 / (4 × 32) = 128
// Address decomposition (32B lines, 128 sets):
//   bits [4:0]   = byte offset (5 bits)
//   bits [11:5]  = set index (7 bits)
//   bits [31:12] = tag (20 bits)
//
// Read-only cache. Instruction fetch paths check the I-cache first, then
// fall back to SRAM. DMA writes invalidate covered lines for coherence.

pub const IC_LINE_SIZE: usize = 32;
pub const IC_NUM_WAYS: usize = 4;
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
    // 16KB: 128 sets × 4 ways. Box to avoid huge stack frames.
    lines: Box<[[ICacheLine; IC_NUM_WAYS]; IC_NUM_SETS]>,
    lru: Box<[[u8; IC_NUM_WAYS]; IC_NUM_SETS]>,
    enabled: bool,
}

impl ICache {
    pub fn new() -> Self {
        Self {
            lines: Box::new([[ICacheLine::empty(); IC_NUM_WAYS]; IC_NUM_SETS]),
            lru: Box::new([[0; IC_NUM_WAYS]; IC_NUM_SETS]),
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

    fn find_way(&self, set: usize, tag: u32) -> Option<usize> {
        for way in 0..IC_NUM_WAYS {
            if self.lines[set][way].valid && self.lines[set][way].tag == tag {
                return Some(way);
            }
        }
        None
    }

    fn touch_lru(&mut self, set: usize, way: usize) {
        let old = self.lru[set][way];
        for w in 0..IC_NUM_WAYS {
            if self.lru[set][w] < old {
                self.lru[set][w] = self.lru[set][w].saturating_add(1);
            }
        }
        self.lru[set][way] = 0;
    }

    fn lru_victim(&self, set: usize) -> usize {
        let mut victim = 0;
        let mut max_val = 0;
        for way in 0..IC_NUM_WAYS {
            if !self.lines[set][way].valid {
                return way;
            }
            if self.lru[set][way] > max_val {
                max_val = self.lru[set][way];
                victim = way;
            }
        }
        victim
    }

    /// Peek a halfword without updating LRU (used via peek for &self access).
    pub fn peek_half(&self, addr: u32) -> Option<u16> {
        let (tag, set, offset) = Self::decompose(addr);
        if let Some(way) = self.find_way(set, tag) {
            let line = &self.lines[set][way];
            Some(((line.data[offset] as u16) << 8) | (line.data[offset + 1] as u16))
        } else {
            None
        }
    }

    /// Peek a word without updating LRU.
    pub fn peek_word(&self, addr: u32) -> Option<u32> {
        let (tag, set, offset) = Self::decompose(addr);
        if let Some(way) = self.find_way(set, tag) {
            let line = &self.lines[set][way];
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
        self.find_way(set, tag).is_some()
    }

    /// Fill a line with data from SRAM.
    pub fn fill_line(&mut self, addr: u32, data: &[u8; IC_LINE_SIZE]) {
        let (tag, set, _) = Self::decompose(addr);
        if self.find_way(set, tag).is_some() {
            return; // already cached
        }
        let victim_way = self.lru_victim(set);
        self.lines[set][victim_way] = ICacheLine {
            valid: true,
            tag,
            data: *data,
        };
        self.touch_lru(set, victim_way);
    }

    /// Invalidate the entire I-cache (IC_IVIC aux 0x10).
    pub fn invalidate_all(&mut self) {
        for set in 0..IC_NUM_SETS {
            for way in 0..IC_NUM_WAYS {
                self.lines[set][way] = ICacheLine::empty();
            }
            self.lru[set] = [0; IC_NUM_WAYS];
        }
    }

    /// Invalidate a single line (IC_IVIL aux 0x19).
    pub fn invalidate_line(&mut self, addr: u32) {
        let (tag, set, _) = Self::decompose(addr);
        if let Some(way) = self.find_way(set, tag) {
            self.lines[set][way] = ICacheLine::empty();
        }
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
        let mut cache = DCache::new();
        let base_set_index = 1usize; // target set index 1

        // Fill all 8 ways in set 1
        for way in 0..NUM_WAYS {
            // Addresses that map to set 1 with different tags:
            // addr = tag << 9 | set_index << 4
            let addr = ((way as u32 + 1) << (OFFSET_BITS + INDEX_BITS))
                | ((base_set_index as u32) << OFFSET_BITS);
            let mut data = [0u8; LINE_SIZE];
            data[0] = way as u8;
            cache.fill_line(addr, &data);
            // Write to make dirty
            cache.write_byte(addr, way as u8 + 0x10);
        }

        // 9th access — should evict the LRU way (way 0, the first filled)
        let new_addr = ((9u32) << (OFFSET_BITS + INDEX_BITS))
            | ((base_set_index as u32) << OFFSET_BITS);
        let new_data = [0xEE; LINE_SIZE];
        let evicted = cache.fill_line(new_addr, &new_data);
        assert!(evicted.is_some());
        let ev = evicted.unwrap();
        // The evicted line should have dirty data (0x10 from write_byte to way 0)
        assert_eq!(ev.data[0], 0x10);
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

        // Invalidate line
        ic.invalidate_line(addr);
        assert!(ic.peek_word(addr).is_none());
    }

    #[test]
    fn test_icache_eviction() {
        let mut ic = ICache::new();
        // Fill 5 lines mapping to same set (IC_NUM_SETS=128, so stride=128*32=4096)
        // Use tag differences to map to same set 0.
        for i in 0..(IC_NUM_WAYS + 1) {
            let addr = ((i as u32 + 1) << (IC_OFFSET_BITS + IC_INDEX_BITS));
            let mut data = [0u8; IC_LINE_SIZE];
            data[0] = i as u8;
            ic.fill_line(addr, &data);
        }
        // After 5 fills with 4-way: first entry should be evicted
        let first_addr = 1u32 << (IC_OFFSET_BITS + IC_INDEX_BITS);
        assert!(ic.peek_word(first_addr).is_none());
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
    fn test_flush_line_keeps_valid() {
        let mut cache = DCache::new();
        let addr = 0x1000u32;

        cache.fill_line(addr, &[0x00; LINE_SIZE]);
        cache.write_byte(addr, 0xAB);

        let ev = cache.flush_line(addr);
        assert!(ev.is_some());
        assert_eq!(ev.unwrap().data[0], 0xAB);

        // Line still valid after flush (flush != invalidate)
        assert_eq!(cache.read_byte(addr), Some(0xAB));

        // Subsequent flush returns None (not dirty anymore)
        assert!(cache.flush_line(addr).is_none());
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
