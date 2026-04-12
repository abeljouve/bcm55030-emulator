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

const LINE_SIZE: usize = 16;
const NUM_WAYS: usize = 8;
const NUM_SETS: usize = 32;

const OFFSET_BITS: u32 = 4; // log2(LINE_SIZE)
const INDEX_BITS: u32 = 5;  // log2(NUM_SETS)

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

pub struct DCache {
    lines: [[CacheLine; NUM_WAYS]; NUM_SETS],
    /// Per-way counter for pseudo-LRU replacement within each set.
    /// Lower value = used more recently. On access, set to 0 and increment others.
    lru: [[u8; NUM_WAYS]; NUM_SETS],
    // DC_CTRL fields
    enabled: bool,
    im: bool,
    lm: bool,
}

impl DCache {
    /// Create a new D-cache matching BCM55030 reset state (DC_CTRL = 0xC2).
    pub fn new() -> Self {
        Self {
            lines: [[CacheLine::empty(); NUM_WAYS]; NUM_SETS],
            lru: [[0; NUM_WAYS]; NUM_SETS],
            enabled: true, // DC bit 0 = 0 → enabled
            im: true,      // IM bit 6 = 1
            lm: true,      // LM bit 7 = 1
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

    /// Read DC_CTRL register value (aux 0x48).
    pub fn read_dc_ctrl(&self) -> u32 {
        let mut val: u32 = 0;
        if !self.enabled {
            val |= 1 << 0; // DC bit: 1 = disabled
        }
        val |= 1 << 1; // BCM55030-specific: bit 1 always reads 1 (0xC2 at reset)
        if self.im {
            val |= 1 << 6;
        }
        if self.lm {
            val |= 1 << 7;
        }
        val
    }

    /// Write DC_CTRL register (aux 0x48). Updates enabled/IM/LM state.
    pub fn write_dc_ctrl(&mut self, val: u32) {
        self.enabled = val & (1 << 0) == 0; // DC=0 → enabled
        self.im = val & (1 << 6) != 0;
        self.lm = val & (1 << 7) != 0;
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

        // Disable cache (set bit 0)
        cache.write_dc_ctrl(0xC3);
        assert!(!cache.is_enabled());
        assert_eq!(cache.read_dc_ctrl() & 1, 1);

        // Re-enable, clear IM
        cache.write_dc_ctrl(0x80); // LM=1 only
        assert!(cache.is_enabled());
        assert!(!cache.im);
        assert!(cache.lm);
    }

    #[test]
    fn test_invalidate_single_line() {
        let mut cache = DCache::new();
        let addr1 = 0x1000u32;
        let addr2 = 0x1010u32; // same set, different tag? No, different set.

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
    fn test_address_decomposition() {
        // addr = 0x00001234
        // offset = 0x4 (bits 3:0)
        // set index = 0x03 (bits 8:4 = 0b00011)
        // tag = 0x00001234 >> 9 = 0x91A >> bits
        // 0x1234: offset = 0x4, set = (0x1234 >> 4) & 0x1F = 0x123 & 0x1F = 3
        let (tag, set, offset) = DCache::decompose(0x1234);
        assert_eq!(offset, 0x4);
        assert_eq!(set, ((0x1234u32 >> 4) & 0x1F) as usize); // = 3
        assert_eq!(tag, 0x1234u32 >> 9);

        // Round-trip: base_addr(tag, set) should give line-aligned addr
        let base = DCache::base_addr(tag, set);
        assert_eq!(base, 0x1234 & !0xF); // line-aligned
    }
}
