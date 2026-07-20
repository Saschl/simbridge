//! Elevation histogram, port of the two-stage GPU kernels in
//! `apps/server/src/terrain/processing/gpu/statistics.ts`.
//!
//! The GPU split the work into 128x128 patches purely for parallelism; integer
//! bin counts are associative, so a single CPU pass produces identical totals.

use crate::fileformat::{ELEV_INVALID, ELEV_UNKNOWN, ELEV_WATER};

pub const HISTOGRAM_BIN_RANGE: f64 = 100.0;
/// Some areas in the world are below water level.
pub const HISTOGRAM_MIN_ELEVATION: f64 = -500.0;
/// Mount Everest.
pub const HISTOGRAM_MAX_ELEVATION: f64 = 29040.0;
/// ceil((29040 + 500 + 1) / 100) = 296
pub const HISTOGRAM_BIN_COUNT: usize = 296;

/// Histogram over all valid elevations. Matching the TS kernel, bin indices
/// are clamped to [0, 296] but the kernel only materialized bins 0..295 — so
/// elevations above 29,000 ft fall into bin 296 and are silently dropped.
pub fn elevation_histogram(elevations: &[i16]) -> [u32; HISTOGRAM_BIN_COUNT] {
    let mut histogram = [0u32; HISTOGRAM_BIN_COUNT];

    for &elevation in elevations {
        if elevation == ELEV_UNKNOWN || elevation == ELEV_INVALID || elevation == ELEV_WATER {
            continue;
        }
        let bin = ((elevation as f64 - HISTOGRAM_MIN_ELEVATION) / HISTOGRAM_BIN_RANGE)
            .ceil()
            .clamp(0.0, HISTOGRAM_BIN_COUNT as f64) as usize;
        if bin < HISTOGRAM_BIN_COUNT {
            histogram[bin] += 1;
        }
    }

    histogram
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bins_follow_the_ts_ceil_formula() {
        // bin b covers elevations in (100*b - 600, 100*b - 500]
        let hist = elevation_histogram(&[-500, -400, -399, 0, 100, 101, 29000]);
        assert_eq!(hist[0], 1); // -500
        assert_eq!(hist[1], 1); // -400
        assert_eq!(hist[2], 1); // -399
        assert_eq!(hist[5], 1); // 0 -> bin 5 = (-100, 0]
        assert_eq!(hist[6], 1); // 100 -> bin 6 = (0, 100]
        assert_eq!(hist[7], 1); // 101 -> bin 7
        assert_eq!(hist[295], 1); // 29000 is the highest binnable elevation
        assert_eq!(hist.iter().sum::<u32>(), 7);
    }

    #[test]
    fn sentinels_and_above_29000_are_dropped() {
        use crate::fileformat::{ELEV_INVALID, ELEV_UNKNOWN, ELEV_WATER};
        let hist = elevation_histogram(&[ELEV_WATER, ELEV_UNKNOWN, ELEV_INVALID, 29001, 29029]);
        assert_eq!(hist.iter().sum::<u32>(), 0);
    }

    #[test]
    fn below_histogram_floor_clamps_into_bin_zero() {
        // Dead Sea shore: -1411 ft clamps to bin 0
        let hist = elevation_histogram(&[-1411]);
        assert_eq!(hist[0], 1);
    }
}
