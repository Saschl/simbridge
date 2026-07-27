//! Reproducible A/B benchmark for the `extract_local_elevation_map`
//! rewrite (warp grid: exact mapping at tile corners, bilinear in between,
//! with centre-probe and polar guards; exact path = closed-form bearing).
//!
//! A = the straight per-pixel port of the TS/GPU kernel, calling the geodesy
//!     helpers for every pixel ([`baseline`] below, kept verbatim).
//! B = the shipped `elevation_map::extract_local_elevation_map`.
//!
//! The warp grid is an approximation by design (accepted 2026-07-26: "close
//! enough" beats exact — the TS original ran f32 on a GPU), so the gate
//! enforces BUDGETS rather than strict equality, sized from the measured
//! behaviour at ship time with headroom. It aborts non-zero if any budget is
//! exceeded:
//!
//! * polar regions must be *identical* (the polar guard forces exact
//!   evaluation there);
//! * per-case divergence <= 0.5% of samples, global <= 0.1% (measured at
//!   ship time: 0.28% worst case, 0.021% global);
//! * elevation delta on any divergent sample <= 100 ft (measured: 10 ft —
//!   divergent samples land on world cells adjacent to the exact one);
//! * downstream, on the rendered ND: threshold metadata identical, frame
//!   byte divergence <= 0.2% (measured: 0.057% worst).
//!
//! Run: cargo bench --no-default-features --bench extract_ab
//!      (append `-- --quick` for a fast smoke run)

use fbw_simbridge_terrain::elevation_map::{extract_local_elevation_map, metres_per_pixel};
use fbw_simbridge_terrain::nd_render::{
    compute_render_stats, compute_thresholds, render_navigation_display,
};
use fbw_simbridge_terrain::patterns::ARC_PATTERN;
use fbw_simbridge_terrain::state::{nd_map_geometry, NdMapGeometry};
use fbw_simbridge_terrain::statistics::elevation_histogram;
use fbw_simbridge_terrain::worldmap::WorldMap;
use std::sync::Arc;

#[path = "common/mod.rs"]
mod common;

const PER_CASE_BUDGET: f64 = 0.005; // fraction of samples
const GLOBAL_BUDGET: f64 = 0.001;
const ELEVATION_DELTA_BUDGET: i32 = 100; // ft
const FRAME_BYTE_BUDGET: f64 = 0.002; // fraction of RGBA bytes

/// The pre-optimisation implementation, copied verbatim. It only uses the
/// library's public geodesy API, so unlike `nd_render_ab` nothing private had
/// to be duplicated.
mod baseline {
    use fbw_simbridge_terrain::fileformat::{ELEV_INVALID, ELEV_UNKNOWN};
    use fbw_simbridge_terrain::geodesy::{
        normalize_heading, project_wgs84, rad2deg, wgs84_to_pixel_coordinate,
    };
    use fbw_simbridge_terrain::state::NdMapGeometry;
    use fbw_simbridge_terrain::worldmap::WorldMap;

    /// Extracts the `width x height` local elevation map, row 0 = top of the
    /// display (farthest ahead).
    pub fn extract_local_elevation_map(
        world: &WorldMap,
        latitude: f64,
        longitude: f64,
        heading: f64,
        geometry: &NdMapGeometry,
        metres_per_pixel: f64,
        arc_mode: bool,
    ) -> Vec<i16> {
        let width = geometry.width;
        let height = geometry.height;
        let center_x = width as f64 / 2.0;
        let center_offset_y = geometry.center_offset_y as f64;

        let mut map = vec![ELEV_INVALID; width * height];

        for y in 0..height {
            let delta_y = height as f64 - y as f64 - center_offset_y;
            let row = &mut map[y * width..(y + 1) * width];

            for (x, out) in row.iter_mut().enumerate() {
                let delta_x = x as f64 - center_x;
                let distance_pixels = (delta_x * delta_x + delta_y * delta_y).sqrt();

                // cut off the arc shape for the A32NX in arc mode
                if center_offset_y == 0.0 && arc_mode && distance_pixels > height as f64 {
                    *out = ELEV_INVALID;
                    continue;
                }

                let distance = distance_pixels * (metres_per_pixel / 2.0);
                // the centre pixel divides 0/0 in the TS kernel; defined as
                // the aircraft's own position instead
                let bearing = if distance_pixels == 0.0 {
                    heading
                } else {
                    let angle = rad2deg((delta_y / distance_pixels).acos());
                    let raw = if x as f64 > center_x { angle } else { 360.0 - angle };
                    normalize_heading(raw + heading)
                };

                let projected = project_wgs84(latitude, longitude, bearing, distance);
                let (px, py) = wgs84_to_pixel_coordinate(
                    latitude,
                    projected.0,
                    projected.1,
                    world.ground_truth_lat,
                    world.ground_truth_lon,
                    world.sw_lat,
                    world.sw_lon,
                    world.ne_lat,
                    world.ne_lon,
                    world.width,
                    world.height,
                    world.ego_x,
                    world.ego_y,
                );

                *out = if px < 0 || py < 0 || px >= world.width as i64 || py >= world.height as i64
                {
                    ELEV_UNKNOWN
                } else {
                    world.elevations[py as usize * world.width + px as usize]
                };
            }
        }

        map
    }
}

/// Deterministic pseudo-relief world with water/unknown speckle.
fn synth_world(
    (sw_lat, sw_lon, ne_lat, ne_lon): (f64, f64, f64, f64),
    (width, height): (usize, usize),
    (gt_lat, gt_lon): (f64, f64),
    seed: u64,
) -> WorldMap {
    let mut rng = common::Rng(seed | 1);
    let mut elevations = Vec::with_capacity(width * height);
    for y in 0..height {
        for x in 0..width {
            let fx = x as f64 / 220.0;
            let fy = y as f64 / 190.0;
            let relief =
                fx.sin() * 1400.0 + fy.cos() * 1100.0 + (fx * 0.5 + fy).sin() * 700.0 + 900.0;
            elevations.push(match rng.step() % 64 {
                0 => -1i16,    // ELEV_WATER
                1 => 32766,    // ELEV_UNKNOWN
                _ => relief.clamp(-450.0, 8800.0) as i16,
            });
        }
    }
    WorldMap {
        sw_lat,
        sw_lon,
        ne_lat,
        ne_lon,
        width,
        height,
        elevations: Arc::new(elevations),
        ground_truth_lat: gt_lat,
        ground_truth_lon: gt_lon,
        ego_x: width as f64 / 2.0,
        ego_y: height as f64 / 2.0,
    }
}

struct Region {
    name: &'static str,
    /// polar-guarded regions must reproduce the baseline exactly
    exact: bool,
    world: WorldMap,
    latitude: f64,
    longitude: f64,
}

/// Alps as the typical case, plus the geodesic edge cases: equator, both
/// poles (the `degrees_per_pixel` special branches and the polar guard) and
/// the antimeridian (the longitude wrap, caught by the centre probe).
fn regions() -> Vec<Region> {
    let specs = [
        ("alps    ", false, (45.0, 9.0, 49.0, 14.0), (4000, 3200), (47.26, 11.35)),
        ("equator ", false, (-2.0, -1.0, 2.0, 3.0), (3600, 3600), (0.05, 0.9)),
        ("north   ", true, (84.0, 10.0, 89.5, 40.0), (2400, 2000), (87.4, 25.0)),
        ("south   ", true, (-89.5, -30.0, -84.0, 10.0), (2400, 2000), (-87.1, -8.0)),
        ("dateline", false, (40.0, 176.0, 44.0, -177.0), (3000, 2600), (42.0, 179.4)),
    ];
    specs
        .into_iter()
        .enumerate()
        .map(|(index, (name, exact, bounds, size, position))| Region {
            name,
            exact,
            world: synth_world(bounds, size, position, 0x9E37_79B9_7F4A_7C15 ^ (index as u64) << 8),
            latitude: position.0,
            longitude: position.1,
        })
        .collect()
}

fn geometries() -> [(&'static str, NdMapGeometry, bool); 4] {
    [
        ("a32nx-arc ", nd_map_geometry(true, false), true),
        ("a32nx-rose", nd_map_geometry(false, false), false),
        ("a380x-arc ", nd_map_geometry(true, true), true),
        ("a380x-rose", nd_map_geometry(false, true), false),
    ]
}

fn fail(message: &str) -> ! {
    eprintln!("BUDGET EXCEEDED: {message}");
    std::process::exit(1);
}

fn main() {
    let quick = common::quick();

    println!(
        "extract_local_elevation_map A/B — arch {}, bench profile (inherits release, lto)\n",
        std::env::consts::ARCH
    );

    // ---- budgeted gate
    let nd_ranges: &[f64] = if quick { &[40.0] } else { &[10.0, 40.0, 160.0] };
    let headings: &[f64] = if quick { &[47.5, 183.25] } else { &[0.0, 47.5, 183.25, 359.9] };

    let regions = regions();
    let mut checked = 0usize;
    let mut total_samples = 0usize;
    let mut total_diverging = 0usize;
    let mut worst_case = (0.0f64, String::new());
    let mut max_elevation_delta = 0i32;
    let mut frames = 0usize;
    let mut worst_frame = 0.0f64;

    for region in &regions {
        for (gname, geometry, arc_mode) in geometries() {
            for &nd_range in nd_ranges {
                for &heading in headings {
                    let mpp = metres_per_pixel(nd_range, &geometry, arc_mode);
                    let a = baseline::extract_local_elevation_map(
                        &region.world, region.latitude, region.longitude,
                        heading, &geometry, mpp, arc_mode,
                    );
                    let b = extract_local_elevation_map(
                        &region.world, region.latitude, region.longitude,
                        heading, &geometry, mpp, arc_mode,
                    );

                    let diverging = a.iter().zip(&b).filter(|(l, r)| l != r).count();
                    let label = format!(
                        "{} / {gname} / range {nd_range} / heading {heading}",
                        region.name
                    );
                    if region.exact && diverging > 0 {
                        fail(&format!("polar case must be identical, {diverging} samples differ — {label}"));
                    }
                    let fraction = diverging as f64 / a.len() as f64;
                    if fraction > PER_CASE_BUDGET {
                        fail(&format!(
                            "{:.3}% of samples differ (budget {:.3}%) — {label}",
                            100.0 * fraction,
                            100.0 * PER_CASE_BUDGET
                        ));
                    }
                    if fraction > worst_case.0 {
                        worst_case = (fraction, label);
                    }
                    for (l, r) in a.iter().zip(&b) {
                        if l != r && *l < 32000 && *r < 32000 && *l != -1 && *r != -1 {
                            let delta = (*l as i32 - *r as i32).abs();
                            max_elevation_delta = max_elevation_delta.max(delta);
                            if delta > ELEVATION_DELTA_BUDGET {
                                fail(&format!("elevation delta {delta} ft on a divergent sample"));
                            }
                        }
                    }
                    total_samples += a.len();
                    total_diverging += diverging;
                    checked += 1;

                    // downstream check on the non-polar arc cases: the frame a
                    // pilot sees and the threshold numbers on the ND
                    if !region.exact && arc_mode {
                        for flight in common::FLIGHTS.iter().take(3) {
                            let sa = compute_render_stats(
                                &elevation_histogram(&a),
                                flight.altitude, flight.vertical_speed,
                                flight.gear_is_down, flight.cut_off,
                            );
                            let sb = compute_render_stats(
                                &elevation_histogram(&b),
                                flight.altitude, flight.vertical_speed,
                                flight.gear_is_down, flight.cut_off,
                            );
                            if compute_thresholds(&sa) != compute_thresholds(&sb) {
                                fail(&format!("ND threshold metadata differs — {}", worst_case.1));
                            }
                            let fa = render_navigation_display(&a, &ARC_PATTERN[..], &geometry, &sa);
                            let fb = render_navigation_display(&b, &ARC_PATTERN[..], &geometry, &sb);
                            let bytes = fa.iter().zip(&fb).filter(|(l, r)| l != r).count();
                            let fraction = bytes as f64 / fa.len() as f64;
                            if fraction > FRAME_BYTE_BUDGET {
                                fail(&format!(
                                    "{:.3}% of rendered frame bytes differ (budget {:.3}%)",
                                    100.0 * fraction,
                                    100.0 * FRAME_BYTE_BUDGET
                                ));
                            }
                            worst_frame = worst_frame.max(fraction);
                            frames += 1;
                        }
                    }
                }
            }
        }
    }

    println!("budgeted gate: {checked} extraction pairs within budget");
    println!(
        "  divergence: {total_diverging}/{total_samples} samples ({:.4}%, budget {:.1}%)",
        100.0 * total_diverging as f64 / total_samples as f64,
        100.0 * GLOBAL_BUDGET
    );
    if 100.0 * (total_diverging as f64 / total_samples as f64) > 100.0 * GLOBAL_BUDGET {
        fail("global sample divergence over budget");
    }
    println!(
        "  worst case: {:.4}% — {}",
        100.0 * worst_case.0,
        if worst_case.1.is_empty() { "(none)" } else { &worst_case.1 }
    );
    println!("  max elevation delta on divergent samples: {max_elevation_delta} ft");
    println!(
        "  downstream: {frames} ND frames rendered, thresholds identical, worst frame {:.4}% bytes\n",
        100.0 * worst_frame
    );

    // ---- timings on the typical region
    let (samples, iters) = if quick { (3, 2) } else { (5, 3) };
    let alps = &regions[0];
    let configs = [
        ("A32NX arc  756x492, 10 nm", nd_map_geometry(true, false), true, 10.0),
        ("A380X arc  756x592, 20 nm", nd_map_geometry(true, true), true, 20.0),
        ("A380X rose 678x592, 40 nm", nd_map_geometry(false, true), false, 40.0),
    ];

    println!(
        "{:<28} {:>17} {:>17} {:>9}",
        "config", "baseline med(min)", "current med(min)", "speedup"
    );
    for (name, geometry, arc_mode, nd_range) in configs {
        let mpp = metres_per_pixel(nd_range, &geometry, arc_mode);
        let a = common::measure(
            || {
                baseline::extract_local_elevation_map(
                    &alps.world, alps.latitude, alps.longitude, 260.0, &geometry, mpp, arc_mode,
                )
            },
            samples,
            iters,
        );
        let b = common::measure(
            || {
                extract_local_elevation_map(
                    &alps.world, alps.latitude, alps.longitude, 260.0, &geometry, mpp, arc_mode,
                )
            },
            samples,
            iters,
        );
        println!(
            "{:<28} {:>10.3} ({:>5.3}) {:>10.3} ({:>5.3}) {:>8.2}x",
            name, a.median_ms, a.min_ms, b.median_ms, b.min_ms,
            a.median_ms / b.median_ms
        );
    }
}
