//! Precomputed pixel-activation density patterns, extracted verbatim from
//! `apps/server/src/terrain/processing/gpu/patterns/{arcmode,scanlinemode}.ts`
//! by `tools/dump_patterns.mjs`.
//!
//! Each value is a product of primes encoding in which density modes the pixel
//! is active: low density = 3, high density = 5, water = 7 (solid = every
//! pixel, no pattern test). A pixel is active for prime `p` iff
//! `value % p == 0` with `value != 0`; 0 means never active.

pub const PATTERN_WIDTH: usize = 768;
/// The arc pattern drives the A32NX ND (max map height 492).
pub const ARC_PATTERN_HEIGHT: usize = 492;
/// The scanline pattern drives the A380X ND (max map height 592).
pub const SCANLINE_PATTERN_HEIGHT: usize = 592;

pub const PRIME_LOW_DENSITY: u8 = 3;
pub const PRIME_HIGH_DENSITY: u8 = 5;
pub const PRIME_WATER: u8 = 7;

pub static ARC_PATTERN: &[u8; PATTERN_WIDTH * ARC_PATTERN_HEIGHT] =
    include_bytes!("../assets/arcmode_pattern.bin");
pub static SCANLINE_PATTERN: &[u8; PATTERN_WIDTH * SCANLINE_PATTERN_HEIGHT] =
    include_bytes!("../assets/scanlinemode_pattern.bin");

/// Pattern value at (x, y); out-of-range reads return 0 (inactive), matching
/// the zero-padded 768x592 GPU texture the TS version sampled.
#[inline]
pub fn pattern_value(pattern: &[u8], x: usize, y: usize) -> u8 {
    let index = y * PATTERN_WIDTH + x;
    if x >= PATTERN_WIDTH || index >= pattern.len() {
        0
    } else {
        pattern[index]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fnv1a(data: &[u8]) -> u64 {
        let mut hash: u64 = 0xcbf29ce484222325;
        for &b in data {
            hash ^= b as u64;
            hash = hash.wrapping_mul(0x100000001b3);
        }
        hash
    }

    #[test]
    fn arc_pattern_matches_ts_source() {
        assert_eq!(ARC_PATTERN.len(), 377_856);
        // first values of createArcModePatternMap()
        assert_eq!(&ARC_PATTERN[..12], &[0, 3, 3, 5, 5, 5, 21, 5, 0, 0, 7, 0]);
        for &v in ARC_PATTERN.iter() {
            assert!(matches!(v, 0 | 3 | 5 | 7 | 15 | 21 | 35 | 105), "unexpected value {v}");
        }
        assert_eq!(fnv1a(&ARC_PATTERN[..]), 0xd930d21f48490815);
    }

    #[test]
    fn scanline_pattern_matches_ts_source() {
        assert_eq!(SCANLINE_PATTERN.len(), 454_656);
        // first values of createScanlineModePatternMap()
        assert_eq!(&SCANLINE_PATTERN[..12], &[0, 0, 5, 5, 35, 21, 21, 3, 5, 5, 5, 0]);
        for &v in SCANLINE_PATTERN.iter() {
            assert!(matches!(v, 0 | 3 | 5 | 7 | 15 | 21 | 35 | 105), "unexpected value {v}");
        }
        assert_eq!(fnv1a(&SCANLINE_PATTERN[..]), 0x13e36acd88a23bff);
    }

    #[test]
    fn out_of_range_reads_are_inactive() {
        assert_eq!(pattern_value(&ARC_PATTERN[..], 0, ARC_PATTERN_HEIGHT), 0);
        assert_eq!(pattern_value(&ARC_PATTERN[..], PATTERN_WIDTH, 0), 0);
    }
}
