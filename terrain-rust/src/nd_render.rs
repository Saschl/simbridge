//! Navigation display rendering: statistics evaluation, color band decisions,
//! density pattern masking, threshold metadata and the runway cut-off altitude.
//!
//! Port of `apps/server/src/terrain/processing/gpu/rendering/navigationdisplay.ts`
//! and the CPU-side pieces of `processing/navigationdisplayrenderer.ts`.
//! Instead of the GPU's embedded metadata row, thresholds are computed
//! directly — the math is identical.

use crate::fileformat::{ELEV_INVALID, ELEV_UNKNOWN, ELEV_WATER};
use crate::geodesy::distance_wgs84;
use crate::jsmath::js_round;
use crate::patterns::{PATTERN_WIDTH, PRIME_HIGH_DENSITY, PRIME_LOW_DENSITY, PRIME_WATER};
use crate::state::NdMapGeometry;
use crate::statistics::{HISTOGRAM_BIN_COUNT, HISTOGRAM_BIN_RANGE, HISTOGRAM_MIN_ELEVATION};
use crate::worldmap::WorldMap;

pub const RENDERING_LOWER_PERCENTILE: f64 = 0.85;
pub const RENDERING_UPPER_PERCENTILE: f64 = 0.95;
pub const RENDERING_FLAT_EARTH_THRESHOLD: f64 = 100.0;
pub const RENDERING_MAX_AIRPORT_DISTANCE_NM: f64 = 4.0;
pub const RENDERING_LOW_DENSITY_GREEN_OFFSET: f64 = 2000.0;
pub const RENDERING_HIGH_DENSITY_GREEN_OFFSET: f64 = 1000.0;
pub const RENDERING_HIGH_DENSITY_YELLOW_OFFSET: f64 = 1000.0;
pub const RENDERING_HIGH_DENSITY_RED_OFFSET: f64 = 2000.0;
pub const RENDERING_GEAR_DOWN_OFFSET: f64 = 250.0;
pub const RENDERING_NON_GEAR_DOWN_OFFSET: f64 = 500.0;
pub const RENDERING_CUT_OFF_ALTITUDE_MINIMUM: f64 = 200.0;
pub const RENDERING_CUT_OFF_ALTITUDE_MAXIMUM: f64 = 400.0;
pub const FEET_PER_NAUTICAL_MILE: f64 = 6076.12;
pub const THREE_NAUTICAL_MILES_IN_FEET: f64 = 18228.3;

pub const COLOR_DISABLED: [u8; 4] = [4, 4, 5, 0];
pub const COLOR_BLACK: [u8; 4] = [0, 0, 0, 255];
pub const COLOR_RED: [u8; 4] = [255, 0, 0, 255];
pub const COLOR_YELLOW: [u8; 4] = [255, 255, 50, 255];
pub const COLOR_GREEN: [u8; 4] = [0, 255, 0, 255];
pub const COLOR_WATER: [u8; 4] = [0, 255, 255, 255];
pub const COLOR_UNKNOWN: [u8; 4] = [255, 148, 255, 255];

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TerrainLevelMode {
    PeaksMode = 0,
    Warning = 1,
    Caution = 2,
}

/// Threshold metadata for one rendered cycle (`NavigationDisplayData` minus
/// the transmission bookkeeping added by the orchestrator).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Thresholds {
    pub minimum_elevation: f64,
    pub minimum_elevation_mode: TerrainLevelMode,
    pub maximum_elevation: f64,
    pub maximum_elevation_mode: TerrainLevelMode,
}

/// Frame-wide statistics derived from the histogram — computed once per cycle
/// (the GPU kernel recomputed them per pixel).
#[derive(Debug, Clone, Copy)]
pub struct RenderStats {
    pub reference_altitude: f64,
    pub minimum_elevation: f64,
    pub maximum_elevation: f64,
    pub lower_percentile_elevation: f64,
    pub upper_percentile_elevation: f64,
    pub flat_earth: f64,
    pub half_elevation: f64,
    pub gear_down_altitude_offset: f64,
    pub cut_off_altitude: f64,
    /// normal mode when the terrain reaches up to the reference altitude
    pub normal_mode: bool,
}

pub fn compute_render_stats(
    histogram: &[u32; HISTOGRAM_BIN_COUNT],
    altitude: f64,
    vertical_speed: f64,
    gear_is_down: bool,
    cut_off_altitude: f64,
) -> RenderStats {
    let cut_off_altitude_bin =
        ((cut_off_altitude - HISTOGRAM_MIN_ELEVATION) / HISTOGRAM_BIN_RANGE).floor() as i64;
    // predict 30 seconds -> half of the vertical speed (feet per minute)
    let reference_altitude = altitude
        + if vertical_speed <= -1000.0 {
            vertical_speed * 0.5
        } else {
            0.0
        };
    let gear_down_altitude_offset = if gear_is_down {
        RENDERING_GEAR_DOWN_OFFSET
    } else {
        RENDERING_NON_GEAR_DOWN_OFFSET
    };

    let start_bin = cut_off_altitude_bin.max(0) as usize;
    let total_frequency: u32 = histogram[start_bin.min(HISTOGRAM_BIN_COUNT)..].iter().sum();

    let mut min_elevation_bin: i64 = -1;
    let mut max_elevation_bin: i64 = -1;
    let mut lower_bin: i64 = -1;
    let mut upper_bin: i64 = -1;
    let mut current_percentile = 0.0f64;

    for bin in start_bin..HISTOGRAM_BIN_COUNT {
        if total_frequency > 0 {
            current_percentile += histogram[bin] as f64 / total_frequency as f64;
            if lower_bin == -1 && current_percentile >= RENDERING_LOWER_PERCENTILE {
                lower_bin = bin as i64;
            }
            if upper_bin == -1 && current_percentile >= RENDERING_UPPER_PERCENTILE {
                upper_bin = bin as i64;
            }
        }

        if histogram[bin] > 0 {
            if min_elevation_bin < 0 {
                min_elevation_bin = bin as i64;
            }
            max_elevation_bin = bin as i64;
        }
    }

    // TS quirk kept: an unset lowerBin (-1) yields -600 while an unset
    // upperBin is clamped to the top bin.
    if upper_bin < 0 {
        upper_bin = HISTOGRAM_BIN_COUNT as i64 - 1;
    }
    let lower_percentile_elevation = lower_bin as f64 * HISTOGRAM_BIN_RANGE + HISTOGRAM_MIN_ELEVATION;
    let upper_percentile_elevation = upper_bin as f64 * HISTOGRAM_BIN_RANGE + HISTOGRAM_MIN_ELEVATION;

    let minimum_elevation = if min_elevation_bin >= 0 {
        min_elevation_bin as f64 * HISTOGRAM_BIN_RANGE + HISTOGRAM_MIN_ELEVATION
    } else {
        -1.0
    };
    let maximum_elevation = if max_elevation_bin >= 0 {
        (max_elevation_bin + 1) as f64 * HISTOGRAM_BIN_RANGE + HISTOGRAM_MIN_ELEVATION
    } else {
        0.0
    };

    let flat_earth = RENDERING_FLAT_EARTH_THRESHOLD - (maximum_elevation - minimum_elevation);
    let half_elevation = maximum_elevation * 0.5;
    let normal_mode = maximum_elevation >= reference_altitude - gear_down_altitude_offset;

    RenderStats {
        reference_altitude,
        minimum_elevation,
        maximum_elevation,
        lower_percentile_elevation,
        upper_percentile_elevation,
        flat_earth,
        half_elevation,
        gear_down_altitude_offset,
        cut_off_altitude,
        normal_mode,
    }
}

/// (lowDensityGreen, highDensityGreen)
fn normal_mode_green_thresholds(stats: &RenderStats) -> (f64, f64) {
    let mut low_density_green =
        if stats.reference_altitude - RENDERING_LOW_DENSITY_GREEN_OFFSET <= stats.minimum_elevation {
            stats.minimum_elevation + 200.0
        } else {
            stats.reference_altitude - RENDERING_LOW_DENSITY_GREEN_OFFSET
        };
    let high_density_green =
        if stats.reference_altitude - RENDERING_HIGH_DENSITY_GREEN_OFFSET <= stats.minimum_elevation {
            stats.minimum_elevation + 200.0
        } else {
            stats.reference_altitude - RENDERING_HIGH_DENSITY_GREEN_OFFSET
        };

    if stats.flat_earth >= 0.0 {
        if stats.half_elevation <= stats.lower_percentile_elevation
            && low_density_green > stats.half_elevation
        {
            low_density_green = stats.half_elevation;
        } else if stats.half_elevation > stats.lower_percentile_elevation
            && low_density_green > stats.lower_percentile_elevation
        {
            low_density_green = stats.lower_percentile_elevation;
        }
    }

    (low_density_green, high_density_green)
}

/// (lowDensityYellow, highDensityYellow, highDensityRed)
fn normal_mode_warning_thresholds(stats: &RenderStats) -> (f64, f64, f64) {
    let mut low_density_yellow = stats.reference_altitude - stats.gear_down_altitude_offset;
    let high_density_yellow = stats.reference_altitude + RENDERING_HIGH_DENSITY_YELLOW_OFFSET;
    let high_density_red = stats.reference_altitude + RENDERING_HIGH_DENSITY_RED_OFFSET;

    if low_density_yellow <= stats.minimum_elevation {
        low_density_yellow = stats.minimum_elevation + 200.0;
    }

    (low_density_yellow, high_density_yellow, high_density_red)
}

/// (lowerDensity, higherDensity, solidDensity)
fn peaks_mode_thresholds(stats: &RenderStats) -> (f64, f64, f64) {
    let lower_density = stats.lower_percentile_elevation.min(stats.half_elevation);
    let mut higher_density = stats.upper_percentile_elevation.min(
        (stats.maximum_elevation - stats.minimum_elevation) * 0.65 + stats.minimum_elevation,
    );
    let mut solid_density =
        (stats.maximum_elevation - stats.minimum_elevation) * 0.95 + stats.minimum_elevation;

    if lower_density >= higher_density
        || lower_density >= solid_density
        || higher_density >= solid_density
        || stats.lower_percentile_elevation >= stats.upper_percentile_elevation
        || stats.lower_percentile_elevation >= solid_density
        || stats.upper_percentile_elevation >= solid_density
    {
        higher_density = stats.maximum_elevation + 100.0;
        solid_density = stats.maximum_elevation + 100.0;
    }

    (lower_density, higher_density, solid_density)
}

#[inline]
fn density_pixel(pattern: u8, prime: u8, color: [u8; 4]) -> [u8; 4] {
    if pattern % prime == 0 {
        color
    } else {
        COLOR_DISABLED
    }
}

/// Colour band an aligned 8x8 block falls into. The band depends only on the
/// block's elevation, never on the pattern, so it is decided once per block
/// instead of once per pixel; the density pattern is applied afterwards via
/// [`BandColors`].
///
/// Both display modes draw from this one set: peaks mode never yields the
/// red/yellow bands and normal mode never yields `SolidGreen`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
enum Band {
    Black = 0,
    Water = 1,
    Unknown = 2,
    HighRed = 3,
    HighYellow = 4,
    HighGreen = 5,
    LowYellow = 6,
    LowGreen = 7,
    SolidGreen = 8,
}

const BANDS: [Band; 9] = [
    Band::Black,
    Band::Water,
    Band::Unknown,
    Band::HighRed,
    Band::HighYellow,
    Band::HighGreen,
    Band::LowYellow,
    Band::LowGreen,
    Band::SolidGreen,
];

/// The colour a band assigns to each of the 256 possible pattern values.
///
/// Pattern value 0 means "never active" and resolves to `COLOR_DISABLED` ahead
/// of the density test — 0 is divisible by every prime, so it cannot be left
/// to [`density_pixel`].
type BandColors = [[u8; 4]; 256];

fn band_colors(band: Band) -> BandColors {
    let mut colors = [COLOR_DISABLED; 256];
    for (value, color) in colors.iter_mut().enumerate().skip(1) {
        let value = value as u8;
        *color = match band {
            Band::Black => COLOR_BLACK,
            Band::Water => density_pixel(value, PRIME_WATER, COLOR_WATER),
            Band::Unknown => density_pixel(value, PRIME_HIGH_DENSITY, COLOR_UNKNOWN),
            Band::HighRed => density_pixel(value, PRIME_HIGH_DENSITY, COLOR_RED),
            Band::HighYellow => density_pixel(value, PRIME_HIGH_DENSITY, COLOR_YELLOW),
            Band::HighGreen => density_pixel(value, PRIME_HIGH_DENSITY, COLOR_GREEN),
            Band::LowYellow => density_pixel(value, PRIME_LOW_DENSITY, COLOR_YELLOW),
            Band::LowGreen => density_pixel(value, PRIME_LOW_DENSITY, COLOR_GREEN),
            Band::SolidGreen => COLOR_GREEN,
        };
    }
    colors
}

/// Colour tables for every band, indexed by the band's discriminant.
fn band_table() -> [BandColors; BANDS.len()] {
    let mut table = [[COLOR_DISABLED; 256]; BANDS.len()];
    for (index, &band) in BANDS.iter().enumerate() {
        debug_assert_eq!(index, band as usize, "BANDS must follow the discriminants");
        table[index] = band_colors(band);
    }
    table
}

/// The seed an empty/fully-invalid block keeps, unchanged from the TS kernel.
/// It doubles as the substitute for `ELEV_INVALID` in the maximum: the seed is
/// also the floor, so anything mapped onto it is indistinguishable from an
/// elevation that lost the comparison anyway.
const BLOCK_EMPTY: i16 = -1000;

/// Highest elevation per aligned 8x8 patch, emulating the lower resolution of
/// the real system.
///
/// Only `ELEV_INVALID` is excluded, so Unknown/Water can still win the maximum
/// (matching the kernel). Mapping Invalid onto the `BLOCK_EMPTY` seed turns
/// that guard into a select, which lets the row-wise maximum vectorise; the
/// result is unchanged, since a value mapped to the seed could never have
/// beaten it anyway.
///
/// Reducing whole rows first (rather than 8x8 patch by patch) is what makes
/// the elementwise maximum a straight `i16` vector op. Classifying the maxima
/// into bands is left to the caller on purpose: an opaque call in the loop
/// below it is enough to stop LLVM vectorising this one.
#[inline(never)]
fn block_maxima(
    elevations: &[i16],
    width: usize,
    height: usize,
    blocks_x: usize,
    blocks_y: usize,
) -> Vec<i16> {
    let mut maxima = vec![BLOCK_EMPTY; blocks_x * blocks_y];
    let mut row_max = vec![BLOCK_EMPTY; width];

    for by in 0..blocks_y {
        row_max.fill(BLOCK_EMPTY);
        for y in by * 8..((by + 1) * 8).min(height) {
            let row = &elevations[y * width..(y + 1) * width];
            for (max, &elevation) in row_max.iter_mut().zip(row) {
                let elevation = if elevation == ELEV_INVALID { BLOCK_EMPTY } else { elevation };
                *max = if elevation > *max { elevation } else { *max };
            }
        }

        let block_row = &mut maxima[by * blocks_x..(by + 1) * blocks_x];
        for (slot, block) in block_row.iter_mut().zip(row_max.chunks(8)) {
            let mut max = BLOCK_EMPTY;
            for &elevation in block {
                max = if elevation > max { elevation } else { max };
            }
            *slot = max;
        }
    }

    maxima
}

/// Paints the RGBA frame from the per-block bands: one pattern byte and one
/// table lookup per pixel.
///
/// Kept separate from [`block_maxima`] deliberately — merging the two passes
/// into one function makes LLVM spill the colour tables and costs ~2x, so the
/// split is load-bearing rather than cosmetic.
#[inline(never)]
fn paint_frame(
    bands: &[Band],
    blocks_x: usize,
    pattern: &[u8],
    table: &[BandColors],
    width: usize,
    height: usize,
) -> Vec<u8> {
    let mut frame = vec![0u8; width * height * 4];

    for (y, frame_row) in frame.chunks_exact_mut(width * 4).enumerate() {
        // rows past the end of the zero-padded pattern texture read as 0
        let start = (y * PATTERN_WIDTH).min(pattern.len());
        let active = (pattern.len() - start).min(PATTERN_WIDTH).min(width);
        let pattern_row = &pattern[start..start + active];
        let band_row = &bands[(y / 8) * blocks_x..(y / 8 + 1) * blocks_x];

        for (bx, band) in band_row.iter().enumerate() {
            let from = bx * 8;
            let to = (from + 8).min(active);
            if from >= to {
                continue;
            }
            let colors = &table[*band as usize];
            let pixels = &mut frame_row[from * 4..to * 4];
            for (pixel, &value) in pixels.chunks_exact_mut(4).zip(&pattern_row[from..to]) {
                pixel.copy_from_slice(&colors[value as usize]);
            }
        }

        for pixel in frame_row[active * 4..].chunks_exact_mut(4) {
            pixel.copy_from_slice(&COLOR_DISABLED);
        }
    }

    frame
}

/// Renders the ND terrain map into an RGBA frame (row 0 = top of display).
pub fn render_navigation_display(
    elevations: &[i16],
    pattern: &[u8],
    geometry: &NdMapGeometry,
    stats: &RenderStats,
) -> Vec<u8> {
    let width = geometry.width;
    let height = geometry.height;
    let blocks_x = width.div_ceil(8);
    let blocks_y = height.div_ceil(8);

    let maxima = block_maxima(elevations, width, height, blocks_x, blocks_y);

    // the band depends only on the block's elevation, so the threshold
    // comparisons run once per 8x8 block instead of once per pixel
    let bands: Vec<Band> = if stats.normal_mode {
        let warning = normal_mode_warning_thresholds(stats);
        let green = normal_mode_green_thresholds(stats);
        maxima
            .iter()
            .map(|&max| normal_mode_band(max as i32, stats, &warning, &green))
            .collect()
    } else {
        let peaks = peaks_mode_thresholds(stats);
        maxima.iter().map(|&max| peaks_mode_band(max as i32, &peaks)).collect()
    };

    paint_frame(&bands, blocks_x, pattern, &band_table(), width, height)
}

fn normal_mode_band(
    elevation: i32,
    stats: &RenderStats,
    warning: &(f64, f64, f64),
    green: &(f64, f64),
) -> Band {
    let (low_density_yellow, high_density_yellow, high_density_red) = *warning;
    let (low_density_green, high_density_green) = *green;
    let e = elevation as f64;

    if elevation != ELEV_INVALID as i32
        && elevation != ELEV_UNKNOWN as i32
        && elevation != ELEV_WATER as i32
        && e >= stats.cut_off_altitude
    {
        if e >= high_density_red {
            return Band::HighRed;
        }
        if e >= high_density_yellow {
            return Band::HighYellow;
        }
        if e >= high_density_green && e < low_density_yellow {
            return Band::HighGreen;
        }
        if e >= low_density_yellow && e < high_density_yellow {
            return Band::LowYellow;
        }
        if e >= low_density_green && e < high_density_green {
            return Band::LowGreen;
        }
    } else if elevation == ELEV_WATER as i32 {
        return Band::Water;
    } else if elevation == ELEV_UNKNOWN as i32 {
        return Band::Unknown;
    }

    Band::Black
}

fn peaks_mode_band(elevation: i32, peaks: &(f64, f64, f64)) -> Band {
    let (lower_density, higher_density, solid_density) = *peaks;
    let e = elevation as f64;

    if elevation != ELEV_INVALID as i32
        && elevation != ELEV_UNKNOWN as i32
        && elevation != ELEV_WATER as i32
    {
        if solid_density <= e {
            return Band::SolidGreen;
        }
        if higher_density <= e {
            return Band::HighGreen;
        }
        if lower_density <= e {
            return Band::LowGreen;
        }
    } else if elevation == ELEV_WATER as i32 {
        return Band::Water;
    } else if elevation == ELEV_UNKNOWN as i32 {
        return Band::Unknown;
    }

    Band::Black
}

/// Threshold metadata (`analyzeMetadata`), computed straight from the stats.
/// FIXED vs TS: the max-elevation Caution decision was keyed off the Warning
/// enum value in `terrainworker.ts:586` when packing the HTTP DTO — modes here
/// carry the intended semantics.
pub fn compute_thresholds(stats: &RenderStats) -> Thresholds {
    if stats.normal_mode {
        let (low_density_yellow, _, high_density_red) = normal_mode_warning_thresholds(stats);
        let (low_density_green, high_density_green) = normal_mode_green_thresholds(stats);

        Thresholds {
            minimum_elevation: if stats.cut_off_altitude > low_density_green {
                stats.cut_off_altitude
            } else {
                low_density_green
            },
            minimum_elevation_mode: if low_density_yellow <= high_density_green {
                TerrainLevelMode::Warning
            } else {
                TerrainLevelMode::PeaksMode
            },
            maximum_elevation: stats.maximum_elevation,
            maximum_elevation_mode: if stats.maximum_elevation >= high_density_red {
                TerrainLevelMode::Caution
            } else {
                TerrainLevelMode::Warning
            },
        }
    } else {
        let (lower_density, _, _) = peaks_mode_thresholds(stats);

        let (minimum_elevation, maximum_elevation) = if stats.maximum_elevation < 0.0 {
            (-1.0, 0.0)
        } else {
            (
                if lower_density > stats.minimum_elevation {
                    lower_density
                } else {
                    stats.minimum_elevation
                },
                stats.maximum_elevation,
            )
        };

        Thresholds {
            minimum_elevation,
            minimum_elevation_mode: TerrainLevelMode::PeaksMode,
            maximum_elevation,
            maximum_elevation_mode: TerrainLevelMode::PeaksMode,
        }
    }
}

/// Runway-proximity cut-off altitude (`calculateAbsoluteCutOffAltitude`).
pub fn absolute_cut_off_altitude(
    world: &WorldMap,
    latitude: f64,
    longitude: f64,
    altitude: f64,
    runway_data_valid: bool,
    runway_latitude: f64,
    runway_longitude: f64,
) -> f64 {
    if !runway_data_valid {
        return HISTOGRAM_MIN_ELEVATION;
    }

    let destination_elevation = world.extract_elevation(latitude, runway_latitude, runway_longitude);
    if destination_elevation == ELEV_INVALID {
        return HISTOGRAM_MIN_ELEVATION;
    }

    let mut cut_off_altitude = RENDERING_CUT_OFF_ALTITUDE_MAXIMUM;

    let distance = distance_wgs84(latitude, longitude, runway_latitude, runway_longitude);
    if distance <= RENDERING_MAX_AIRPORT_DISTANCE_NM {
        let distance_feet = distance * FEET_PER_NAUTICAL_MILE;

        // glide slope until touchdown; opposite is in feet
        let opposite = altitude - destination_elevation as f64;
        let glide_radian = if opposite > 0.0 && distance > 0.0 {
            (opposite / distance_feet).atan()
        } else {
            0.0
        };

        // only shrink the cut-off below a 3° glide
        if glide_radian < 0.0523599 {
            if distance <= 1.0 || glide_radian == 0.0 {
                cut_off_altitude = RENDERING_CUT_OFF_ALTITUDE_MINIMUM;
            } else {
                // linear from max to min between 4 nm and 1 nm
                let slope = (RENDERING_CUT_OFF_ALTITUDE_MINIMUM - RENDERING_CUT_OFF_ALTITUDE_MAXIMUM)
                    / THREE_NAUTICAL_MILES_IN_FEET;
                cut_off_altitude =
                    js_round(slope * (distance_feet - FEET_PER_NAUTICAL_MILE) + RENDERING_CUT_OFF_ALTITUDE_MAXIMUM)
                        .clamp(RENDERING_CUT_OFF_ALTITUDE_MINIMUM, RENDERING_CUT_OFF_ALTITUDE_MAXIMUM);
            }
        }
    }

    cut_off_altitude
}
