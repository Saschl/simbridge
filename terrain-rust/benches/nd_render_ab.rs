//! Reproducible A/B benchmark for the `render_navigation_display`
//! optimisation (per-block band hoisting, per-band colour tables and the
//! vectorised 8x8 block maxima).
//!
//! A = the direct port of the TS/GPU kernel as originally written: per-pixel
//!     band decision and density test ([`baseline`] below, kept verbatim).
//! B = the shipped `nd_render::render_navigation_display`.
//!
//! Before timing anything, the bench proves A == B byte-for-byte across a
//! matrix of geometries, patterns, degenerate elevation maps and flight
//! states — the timings are only meaningful while the implementations agree,
//! and the gate doubles as a regression tripwire for both sides.
//!
//! Run: cargo bench --no-default-features --bench nd_render_ab
//!      (append `-- --quick` for a fast smoke run)

use fbw_simbridge_terrain::nd_render::render_navigation_display;
use fbw_simbridge_terrain::patterns::{ARC_PATTERN, SCANLINE_PATTERN};
use fbw_simbridge_terrain::state::nd_map_geometry;

#[path = "common/mod.rs"]
mod common;

/// The pre-optimisation implementation, copied verbatim. The private
/// threshold helpers are duplicated because the library does not export them;
/// if either copy ever drifts from the library's behaviour, the bit-exact
/// gate in `main` fails before any timing is reported.
// the copy stays byte-for-byte as it was shipped, warts included
#[allow(clippy::manual_is_multiple_of)]
mod baseline {
    use fbw_simbridge_terrain::fileformat::{ELEV_INVALID, ELEV_UNKNOWN, ELEV_WATER};
    use fbw_simbridge_terrain::nd_render::{
        RenderStats, COLOR_BLACK, COLOR_DISABLED, COLOR_GREEN, COLOR_RED, COLOR_UNKNOWN,
        COLOR_WATER, COLOR_YELLOW, RENDERING_HIGH_DENSITY_GREEN_OFFSET,
        RENDERING_HIGH_DENSITY_RED_OFFSET, RENDERING_HIGH_DENSITY_YELLOW_OFFSET,
        RENDERING_LOW_DENSITY_GREEN_OFFSET,
    };
    use fbw_simbridge_terrain::patterns::{
        pattern_value, PRIME_HIGH_DENSITY, PRIME_LOW_DENSITY, PRIME_WATER,
    };
    use fbw_simbridge_terrain::state::NdMapGeometry;

    /// (lowDensityGreen, highDensityGreen)
    fn normal_mode_green_thresholds(stats: &RenderStats) -> (f64, f64) {
        let mut low_density_green = if stats.reference_altitude - RENDERING_LOW_DENSITY_GREEN_OFFSET
            <= stats.minimum_elevation
        {
            stats.minimum_elevation + 200.0
        } else {
            stats.reference_altitude - RENDERING_LOW_DENSITY_GREEN_OFFSET
        };
        let high_density_green = if stats.reference_altitude - RENDERING_HIGH_DENSITY_GREEN_OFFSET
            <= stats.minimum_elevation
        {
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

    /// Renders the ND terrain map into an RGBA frame (row 0 = top of display).
    pub fn render_navigation_display(
        elevations: &[i16],
        pattern: &[u8],
        geometry: &NdMapGeometry,
        stats: &RenderStats,
    ) -> Vec<u8> {
        let width = geometry.width;
        let height = geometry.height;

        // highest elevation per aligned 8x8 patch (ignoring Invalid only, so
        // Unknown/Water can win the max — matches the kernel), emulating the
        // lower resolution of the real system
        let blocks_x = width.div_ceil(8);
        let blocks_y = height.div_ceil(8);
        let mut block_max = vec![-1000i32; blocks_x * blocks_y];
        for by in 0..blocks_y {
            for bx in 0..blocks_x {
                let mut pixel_elevation = -1000i32;
                for y in by * 8..((by + 1) * 8).min(height) {
                    for x in bx * 8..((bx + 1) * 8).min(width) {
                        let elevation = elevations[y * width + x] as i32;
                        if elevation > pixel_elevation && elevation != ELEV_INVALID as i32 {
                            pixel_elevation = elevation;
                        }
                    }
                }
                block_max[by * blocks_x + bx] = pixel_elevation;
            }
        }

        let warning = normal_mode_warning_thresholds(stats);
        let green = normal_mode_green_thresholds(stats);
        let peaks = peaks_mode_thresholds(stats);

        let mut frame = vec![0u8; width * height * 4];
        for y in 0..height {
            for x in 0..width {
                let pattern_val = pattern_value(pattern, x, y);
                let rgba = if pattern_val == 0 {
                    COLOR_DISABLED
                } else {
                    let elevation = block_max[(y / 8) * blocks_x + x / 8];
                    if stats.normal_mode {
                        render_normal_mode_pixel(elevation, pattern_val, stats, &warning, &green)
                    } else {
                        render_peaks_mode_pixel(elevation, pattern_val, &peaks)
                    }
                };
                frame[(y * width + x) * 4..(y * width + x) * 4 + 4].copy_from_slice(&rgba);
            }
        }

        frame
    }

    fn render_normal_mode_pixel(
        elevation: i32,
        pattern_val: u8,
        stats: &RenderStats,
        warning: &(f64, f64, f64),
        green: &(f64, f64),
    ) -> [u8; 4] {
        let (low_density_yellow, high_density_yellow, high_density_red) = *warning;
        let (low_density_green, high_density_green) = *green;
        let e = elevation as f64;

        if elevation != ELEV_INVALID as i32
            && elevation != ELEV_UNKNOWN as i32
            && elevation != ELEV_WATER as i32
            && e >= stats.cut_off_altitude
        {
            if e >= high_density_red {
                return density_pixel(pattern_val, PRIME_HIGH_DENSITY, COLOR_RED);
            }
            if e >= high_density_yellow {
                return density_pixel(pattern_val, PRIME_HIGH_DENSITY, COLOR_YELLOW);
            }
            if e >= high_density_green && e < low_density_yellow {
                return density_pixel(pattern_val, PRIME_HIGH_DENSITY, COLOR_GREEN);
            }
            if e >= low_density_yellow && e < high_density_yellow {
                return density_pixel(pattern_val, PRIME_LOW_DENSITY, COLOR_YELLOW);
            }
            if e >= low_density_green && e < high_density_green {
                return density_pixel(pattern_val, PRIME_LOW_DENSITY, COLOR_GREEN);
            }
        } else if elevation == ELEV_WATER as i32 {
            return density_pixel(pattern_val, PRIME_WATER, COLOR_WATER);
        } else if elevation == ELEV_UNKNOWN as i32 {
            return density_pixel(pattern_val, PRIME_HIGH_DENSITY, COLOR_UNKNOWN);
        }

        COLOR_BLACK
    }

    fn render_peaks_mode_pixel(
        elevation: i32,
        pattern_val: u8,
        peaks: &(f64, f64, f64),
    ) -> [u8; 4] {
        let (lower_density, higher_density, solid_density) = *peaks;
        let e = elevation as f64;

        if elevation != ELEV_INVALID as i32
            && elevation != ELEV_UNKNOWN as i32
            && elevation != ELEV_WATER as i32
        {
            if solid_density <= e {
                return COLOR_GREEN;
            }
            if higher_density <= e {
                return density_pixel(pattern_val, PRIME_HIGH_DENSITY, COLOR_GREEN);
            }
            if lower_density <= e {
                return density_pixel(pattern_val, PRIME_LOW_DENSITY, COLOR_GREEN);
            }
        } else if elevation == ELEV_WATER as i32 {
            return density_pixel(pattern_val, PRIME_WATER, COLOR_WATER);
        } else if elevation == ELEV_UNKNOWN as i32 {
            return density_pixel(pattern_val, PRIME_HIGH_DENSITY, COLOR_UNKNOWN);
        }

        COLOR_BLACK
    }
}

fn main() {
    let quick = common::quick();
    let (samples, iters) = if quick { (5, 8) } else { (12, 25) };

    println!(
        "render_navigation_display A/B — arch {}, bench profile (inherits release, lto)\n",
        std::env::consts::ARCH
    );

    // ---- bit-exact gate: the A/B numbers mean nothing unless A == B
    let geometries = [
        ("A32NX arc  756x492", nd_map_geometry(true, false)),
        ("A380X arc  756x592", nd_map_geometry(true, true)),
        ("A32NX rose 678x250", nd_map_geometry(false, false)),
        ("A380X rose 678x592", nd_map_geometry(false, true)),
    ];
    let patterns = [("arc", &ARC_PATTERN[..]), ("scanline", &SCANLINE_PATTERN[..])];

    let mut checked = 0usize;
    for (gname, geometry) in &geometries {
        for (iname, elevations) in common::gate_inputs(geometry) {
            for flight in common::FLIGHTS {
                let stats = common::stats_for(&elevations, flight);
                for &(pname, pattern) in &patterns {
                    let a = baseline::render_navigation_display(&elevations, pattern, geometry, &stats);
                    let b = render_navigation_display(&elevations, pattern, geometry, &stats);
                    if a != b {
                        let first = a.iter().zip(&b).position(|(x, y)| x != y).unwrap();
                        eprintln!(
                            "A/B MISMATCH: {gname} / {pname} pattern / {iname} / {}: \
                             first differing byte at {first} (baseline {} vs current {})",
                            flight.name, a[first], b[first]
                        );
                        std::process::exit(1);
                    }
                    checked += 1;
                }
            }
        }
    }
    println!("bit-exact gate: {checked} frame pairs identical\n");

    // ---- timings. A380X arc + arc pattern is a synthetic combination (the
    // A380X drives the scanline pattern) but exercises the rows past the
    // 492-row arc pattern, i.e. the zero-padded texture tail.
    let configs = [
        ("A32NX arc  756x492 | arc pat ", nd_map_geometry(true, false), &ARC_PATTERN[..]),
        ("A380X arc  756x592 | scanline", nd_map_geometry(true, true), &SCANLINE_PATTERN[..]),
        ("A380X arc  756x592 | arc pat ", nd_map_geometry(true, true), &ARC_PATTERN[..]),
        ("A32NX rose 678x250 | arc pat ", nd_map_geometry(false, false), &ARC_PATTERN[..]),
        ("A380X rose 678x592 | scanline", nd_map_geometry(false, true), &SCANLINE_PATTERN[..]),
    ];

    println!(
        "{:<30} {:<15} {:>17} {:>17} {:>9}",
        "config", "flight", "baseline med(min)", "current med(min)", "speedup"
    );
    let mut ln_sum = 0.0f64;
    let mut rows = 0u32;
    let (mut total_a, mut total_b) = (0.0f64, 0.0f64);

    for (name, geometry, pattern) in configs {
        let elevations = common::ridge_elevations(&geometry, 0x9E37_79B9_7F4A_7C15);
        for flight in common::FLIGHTS {
            let stats = common::stats_for(&elevations, flight);
            let a = common::measure(
                || baseline::render_navigation_display(&elevations, pattern, &geometry, &stats),
                samples,
                iters,
            );
            let b = common::measure(
                || render_navigation_display(&elevations, pattern, &geometry, &stats),
                samples,
                iters,
            );
            let speedup = a.median_ms / b.median_ms;
            println!(
                "{:<30} {:<15} {:>9.3} ({:>5.3}) {:>9.3} ({:>5.3}) {:>8.2}x",
                name, flight.name, a.median_ms, a.min_ms, b.median_ms, b.min_ms, speedup
            );
            ln_sum += speedup.ln();
            rows += 1;
            total_a += a.median_ms;
            total_b += b.median_ms;
        }
    }

    println!(
        "\ntotals (sum of medians): baseline {total_a:.2} ms, current {total_b:.2} ms — \
         {:.2}x overall, {:.2}x geometric mean over {rows} rows",
        total_a / total_b,
        (ln_sum / rows as f64).exp()
    );
}
