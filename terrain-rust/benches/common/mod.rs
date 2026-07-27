//! Shared inputs and timing helpers for the terrain benches.
//!
//! Everything here is deterministic — fixed seeds, no wall-clock dependence in
//! the generated data — so runs are comparable across machines and sessions.
#![allow(dead_code)] // each bench target compiles this file and uses a subset

use fbw_simbridge_terrain::fileformat::{ELEV_INVALID, ELEV_UNKNOWN, ELEV_WATER};
use fbw_simbridge_terrain::nd_render::{compute_render_stats, RenderStats};
use fbw_simbridge_terrain::state::NdMapGeometry;
use fbw_simbridge_terrain::statistics::elevation_histogram;
use std::time::Instant;

/// xorshift64: deterministic and seedable without pulling in a dependency.
pub struct Rng(pub u64);

impl Rng {
    pub fn step(&mut self) -> u64 {
        let mut x = self.0;
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        self.0 = x;
        x
    }
}

/// Synthetic terrain: smooth ridges with water/unknown/invalid speckle, and
/// the corners cut off the way `extract_local_elevation_map` shapes an A32NX
/// arc-mode map. Deliberately mixes every sentinel with plausible relief so
/// both the density patterns and the sentinel branches get exercised.
pub fn ridge_elevations(geometry: &NdMapGeometry, seed: u64) -> Vec<i16> {
    let (width, height) = (geometry.width, geometry.height);
    let mut rng = Rng(seed | 1);
    let mut map = vec![ELEV_INVALID; width * height];
    let center_x = width as f64 / 2.0;

    for y in 0..height {
        let delta_y = height as f64 - y as f64 - geometry.center_offset_y as f64;
        for x in 0..width {
            let delta_x = x as f64 - center_x;
            if geometry.center_offset_y == 0
                && (delta_x * delta_x + delta_y * delta_y).sqrt() > height as f64
            {
                continue; // arc cut-off -> stays ELEV_INVALID
            }
            let fx = x as f64 / 60.0;
            let fy = y as f64 / 60.0;
            let relief =
                fx.sin() * 1800.0 + fy.cos() * 1400.0 + (fx + fy).sin() * 900.0 + 1500.0;
            map[y * width + x] = match rng.step() % 100 {
                0..=6 => ELEV_WATER,
                7..=9 => ELEV_UNKNOWN,
                10 => ELEV_INVALID,
                _ => relief.clamp(-400.0, 12000.0) as i16,
            };
        }
    }
    map
}

/// Elevation maps for the bit-exact gate: the degenerate ones push the
/// histogram/threshold logic into its edge branches (unset percentile bins,
/// the flat-earth path, a negative-free maximum from a single spike).
pub fn gate_inputs(geometry: &NdMapGeometry) -> Vec<(&'static str, Vec<i16>)> {
    let (width, height) = (geometry.width, geometry.height);
    let mut spike = vec![0i16; width * height];
    spike[(height / 2) * width + width / 2] = 9000;

    vec![
        ("all-invalid", vec![ELEV_INVALID; width * height]),
        ("all-water", vec![ELEV_WATER; width * height]),
        ("all-unknown", vec![ELEV_UNKNOWN; width * height]),
        ("flat-1500", vec![1500; width * height]),
        ("spike", spike),
        ("ridge", ridge_elevations(geometry, 0x9E37_79B9_7F4A_7C15)),
    ]
}

/// Aircraft states chosen to hit both display modes plus the vertical-speed
/// prediction and gear-offset branches of `compute_render_stats`.
pub struct Flight {
    pub name: &'static str,
    pub altitude: f64,
    pub vertical_speed: f64,
    pub gear_is_down: bool,
    pub cut_off: f64,
}

pub const FLIGHTS: &[Flight] = &[
    Flight { name: "cruise (peaks)", altitude: 25000.0, vertical_speed: 0.0, gear_is_down: false, cut_off: -500.0 },
    Flight { name: "approach", altitude: 3000.0, vertical_speed: 0.0, gear_is_down: true, cut_off: 200.0 },
    Flight { name: "descent", altitude: 1200.0, vertical_speed: -1500.0, gear_is_down: true, cut_off: 400.0 },
    Flight { name: "sea level", altitude: 0.0, vertical_speed: 0.0, gear_is_down: false, cut_off: -500.0 },
];

pub fn stats_for(elevations: &[i16], flight: &Flight) -> RenderStats {
    let histogram = elevation_histogram(elevations);
    compute_render_stats(
        &histogram,
        flight.altitude,
        flight.vertical_speed,
        flight.gear_is_down,
        flight.cut_off,
    )
}

/// Per-call time over `samples` timing samples of `iters` calls each, after a
/// short warmup. The median is the headline number (robust against one-off
/// scheduler noise); the minimum approximates the uncontended cost.
pub struct Measurement {
    pub median_ms: f64,
    pub min_ms: f64,
}

pub fn measure<T>(mut f: impl FnMut() -> T, samples: usize, iters: usize) -> Measurement {
    for _ in 0..2 {
        std::hint::black_box(f());
    }
    let mut per_call = Vec::with_capacity(samples);
    for _ in 0..samples {
        let start = Instant::now();
        for _ in 0..iters {
            std::hint::black_box(f());
        }
        per_call.push(start.elapsed().as_secs_f64() * 1000.0 / iters as f64);
    }
    per_call.sort_by(f64::total_cmp);
    Measurement {
        median_ms: per_call[per_call.len() / 2],
        min_ms: per_call[0],
    }
}

/// `-- --quick` requests a fast smoke run. Cargo appends its own flags (e.g.
/// `--bench`) to the binary's arguments, so unknown ones are ignored.
pub fn quick() -> bool {
    std::env::args().any(|arg| arg == "--quick")
}
