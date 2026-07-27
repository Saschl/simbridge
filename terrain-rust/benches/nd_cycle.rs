//! Where a full ND cycle's CPU time goes: elevation extraction vs histogram
//! vs rendering, on a synthetic world map.
//!
//! Not an A/B — a profile that keeps optimisation targets honest. As of the
//! render optimisation, `extract_local_elevation_map` dominates the cycle by
//! a wide margin; run this before optimising anything else.
//!
//! Run: cargo bench --no-default-features --bench nd_cycle
//!      (append `-- --quick` for a fast smoke run)

use fbw_simbridge_terrain::elevation_map::{extract_local_elevation_map, metres_per_pixel};
use fbw_simbridge_terrain::nd_render::render_navigation_display;
use fbw_simbridge_terrain::patterns::{ARC_PATTERN, SCANLINE_PATTERN};
use fbw_simbridge_terrain::state::nd_map_geometry;
use fbw_simbridge_terrain::statistics::elevation_histogram;
use fbw_simbridge_terrain::worldmap::WorldMap;
use std::sync::Arc;

#[path = "common/mod.rs"]
mod common;

/// Alps-sized stitched world map with deterministic pseudo-relief. The exact
/// values do not matter for timing — only that the sampled reads are spread
/// across a realistically large (24 MiB) elevation array.
fn synth_world() -> WorldMap {
    let (width, height) = (4000usize, 3200usize);
    WorldMap {
        sw_lat: 45.0,
        sw_lon: 9.0,
        ne_lat: 49.0,
        ne_lon: 14.0,
        width,
        height,
        elevations: Arc::new((0..width * height).map(|i| ((i * 7919) % 3000) as i16).collect()),
        ground_truth_lat: 47.26,
        ground_truth_lon: 11.35,
        ego_x: width as f64 / 2.0,
        ego_y: height as f64 / 2.0,
    }
}

fn main() {
    let quick = common::quick();
    // extraction is ~two orders of magnitude slower than the other stages;
    // give it fewer iterations so the bench stays snappy
    let (ext_samples, ext_iters) = if quick { (3, 2) } else { (5, 3) };
    let (samples, iters) = if quick { (5, 10) } else { (10, 20) };

    println!(
        "ND cycle stage profile — arch {}, bench profile (inherits release, lto)\n",
        std::env::consts::ARCH
    );

    let world = synth_world();
    let configs = [
        ("A32NX arc  756x492, 10 nm", nd_map_geometry(true, false), true, 10.0, &ARC_PATTERN[..]),
        ("A380X arc  756x592, 20 nm", nd_map_geometry(true, true), true, 20.0, &SCANLINE_PATTERN[..]),
    ];

    for (name, geometry, arc_mode, nd_range, pattern) in configs {
        let mpp = metres_per_pixel(nd_range, &geometry, arc_mode);

        let extract = common::measure(
            || extract_local_elevation_map(&world, 47.26, 11.35, 260.0, &geometry, mpp, arc_mode),
            ext_samples,
            ext_iters,
        );

        let elevations =
            extract_local_elevation_map(&world, 47.26, 11.35, 260.0, &geometry, mpp, arc_mode);
        let histogram = common::measure(|| elevation_histogram(&elevations), samples, iters);

        let stats = common::stats_for(&elevations, &common::FLIGHTS[1]);
        let render = common::measure(
            || render_navigation_display(&elevations, pattern, &geometry, &stats),
            samples,
            iters,
        );

        let total = extract.median_ms + histogram.median_ms + render.median_ms;
        println!("{name}");
        for (stage, m) in [
            ("extract_local_elevation_map", &extract),
            ("elevation_histogram", &histogram),
            ("render_navigation_display", &render),
        ] {
            println!(
                "  {stage:<28} {:>8.3} ms  ({:>4.1}% of cycle)",
                m.median_ms,
                100.0 * m.median_ms / total
            );
        }
        println!("  {:<28} {total:>8.3} ms\n", "cycle total");
    }
}
