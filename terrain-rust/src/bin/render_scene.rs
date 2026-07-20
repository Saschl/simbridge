//! Renders ND frames for a fixed aircraft state (milestone M3 deliverable and
//! the golden-frame comparison driver of M4).
//!
//! Usage:
//!   render_scene --scenario <file.json> [--db <terrain.map>] [--out <out.png>] [--side L|R]
//!   render_scene --scenario <file.json> --compare <golden.png>

use std::process::exit;

use fbw_simbridge_terrain::compositor::{
    compose_screen_frame, SCREEN_HEIGHT_WITHOUT_VD, SCREEN_HEIGHT_WITH_VD,
};
use fbw_simbridge_terrain::elevation_map::{extract_local_elevation_map, metres_per_pixel};
use fbw_simbridge_terrain::fileformat::TerrainMap;
use fbw_simbridge_terrain::geodesy::{project_wgs84, NM_TO_METRES};
use fbw_simbridge_terrain::nd_render::{
    absolute_cut_off_altitude, compute_render_stats, compute_thresholds, render_navigation_display,
};
use fbw_simbridge_terrain::patterns::{ARC_PATTERN, SCANLINE_PATTERN};
use fbw_simbridge_terrain::png_out::encode_rgba;
use fbw_simbridge_terrain::state::{nd_map_geometry, AircraftStatus, Side};
use fbw_simbridge_terrain::statistics::elevation_histogram;
use fbw_simbridge_terrain::transition::{Transition, TransitionStyle};
use fbw_simbridge_terrain::vd_render::{
    extract_elevation_profile, render_vertical_display, vd_range_from_nd, ElevationProfileConfig,
    VD_PROFILE_HEIGHT, VD_PROFILE_WIDTH,
};
use fbw_simbridge_terrain::worldmap::WorldMapManager;

const DEFAULT_DB_PROBES: [&str; 3] = [
    "terrain/terrain.map",
    "../terrain/terrain.map",
    "../cache/terrain.map",
];

fn usage() -> ! {
    eprintln!("Usage: render_scene --scenario <file.json> [--db <path>] [--out <out.png>] [--side L|R] [--compare <golden.png>]");
    exit(2);
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let mut scenario_path = None;
    let mut db_path: Option<String> = None;
    let mut out_path: Option<String> = None;
    let mut compare_path: Option<String> = None;
    let mut diff_out_path: Option<String> = None;
    let mut animate_dir: Option<String> = None;
    let mut compare_vd_path: Option<String> = None;
    let mut side = Side::Left;

    let mut i = 0;
    while i < args.len() {
        match args[i].as_str() {
            "--scenario" => {
                scenario_path = args.get(i + 1).cloned();
                i += 2;
            }
            "--db" => {
                db_path = args.get(i + 1).cloned();
                i += 2;
            }
            "--out" => {
                out_path = args.get(i + 1).cloned();
                i += 2;
            }
            "--compare" => {
                compare_path = args.get(i + 1).cloned();
                i += 2;
            }
            "--diff-out" => {
                diff_out_path = args.get(i + 1).cloned();
                i += 2;
            }
            "--animate" => {
                animate_dir = args.get(i + 1).cloned();
                i += 2;
            }
            "--compare-vd" => {
                compare_vd_path = args.get(i + 1).cloned();
                i += 2;
            }
            "--side" => {
                side = match args.get(i + 1).map(String::as_str) {
                    Some("L") => Side::Left,
                    Some("R") => Side::Right,
                    _ => usage(),
                };
                i += 2;
            }
            _ => usage(),
        }
    }

    let Some(scenario_path) = scenario_path else { usage() };
    let scenario = std::fs::read_to_string(&scenario_path).unwrap_or_else(|e| {
        eprintln!("Failed to read {scenario_path}: {e}");
        exit(1);
    });
    let status: AircraftStatus = serde_json::from_str(&scenario).unwrap_or_else(|e| {
        eprintln!("Failed to parse scenario: {e}");
        exit(1);
    });

    let db_path = db_path
        .or_else(|| {
            DEFAULT_DB_PROBES
                .iter()
                .find(|p| std::path::Path::new(p).is_file())
                .map(|p| p.to_string())
        })
        .unwrap_or_else(|| {
            eprintln!("No terrain.map found; pass --db <path>");
            exit(1);
        });

    let terrain = TerrainMap::from_bytes(std::fs::read(&db_path).expect("read terrain.map"))
        .expect("parse terrain.map");
    let mut manager = WorldMapManager::new(terrain);
    let world = manager
        .update_position(status.ground_truth_latitude, status.ground_truth_longitude)
        .expect("world map snapshot");

    let efis = status.efis(side).clone();
    let geometry = nd_map_geometry(efis.arc_mode, status.vertical_display_required());
    let mpp = metres_per_pixel(efis.nd_range, &geometry, efis.arc_mode);

    let elevations = extract_local_elevation_map(
        &world,
        status.latitude,
        status.longitude,
        status.heading as f64,
        &geometry,
        mpp,
        efis.arc_mode,
    );
    let histogram = elevation_histogram(&elevations);
    let cut_off = absolute_cut_off_altitude(
        &world,
        status.latitude,
        status.longitude,
        status.altitude,
        status.runway_data_valid,
        status.runway_latitude,
        status.runway_longitude,
    );
    let stats = compute_render_stats(
        &histogram,
        status.altitude,
        status.vertical_speed,
        status.gear_is_down,
        cut_off,
    );
    let pattern: &[u8] = if status.scanline_mode() {
        &SCANLINE_PATTERN[..]
    } else {
        &ARC_PATTERN[..]
    };
    let frame = render_navigation_display(&elevations, pattern, &geometry, &stats);
    let thresholds = compute_thresholds(&stats);

    println!(
        "rendered {}x{} | mode {} | cutOff {} ft | thresholds: min {} ({:?}), max {} ({:?})",
        geometry.width,
        geometry.height,
        if stats.normal_mode { "NORMAL" } else { "PEAKS" },
        stats.cut_off_altitude,
        thresholds.minimum_elevation,
        thresholds.minimum_elevation_mode,
        thresholds.maximum_elevation,
        thresholds.maximum_elevation_mode,
    );

    if let Some(out_path) = &out_path {
        let bytes = encode_rgba(geometry.width, geometry.height, &frame).expect("encode png");
        std::fs::write(out_path, &bytes).expect("write png");
        println!("wrote {out_path} ({} bytes)", bytes.len());
    }

    if let Some(compare_path) = &compare_path {
        let mut golden = png::Decoder::new(std::fs::File::open(compare_path).expect("open golden"));
        golden.set_transformations(png::Transformations::EXPAND | png::Transformations::STRIP_16);
        let mut reader = golden.read_info().expect("golden header");
        let mut buf = vec![0u8; reader.output_buffer_size()];
        let info = reader.next_frame(&mut buf).expect("golden frame");
        buf.truncate(info.buffer_size());

        // normalize RGB (no alpha channel) to RGBA
        if info.color_type == png::ColorType::Rgb {
            let mut rgba = Vec::with_capacity(buf.len() / 3 * 4);
            for px in buf.chunks_exact(3) {
                rgba.extend_from_slice(px);
                rgba.push(255);
            }
            buf = rgba;
        } else if info.color_type != png::ColorType::Rgba {
            eprintln!("golden has unsupported color type {:?}", info.color_type);
            exit(1);
        }

        // a full composited screen frame (768x768/1024) is cropped to the ND
        // region: the map is blitted at (offset_x, 128) by the compositor
        let golden: Vec<u8> = if info.width as usize == geometry.width
            && info.height as usize == geometry.height
        {
            buf
        } else if info.width as usize >= geometry.width + geometry.offset_x
            && info.height as usize >= geometry.height + 128
        {
            let stride = info.width as usize * 4;
            let mut cropped = Vec::with_capacity(geometry.width * geometry.height * 4);
            for y in 0..geometry.height {
                let start = (y + 128) * stride + geometry.offset_x * 4;
                cropped.extend_from_slice(&buf[start..start + geometry.width * 4]);
            }
            cropped
        } else {
            eprintln!(
                "MISMATCH: golden is {}x{}, rendered {}x{}",
                info.width, info.height, geometry.width, geometry.height
            );
            exit(1);
        };
        let buf = golden;

        let mut differing = 0usize;
        let mut max_channel_delta = 0u8;
        for (a, b) in frame.iter().zip(buf.iter()) {
            let delta = a.abs_diff(*b);
            if delta > 0 {
                max_channel_delta = max_channel_delta.max(delta);
            }
        }
        let mut diff_image = vec![0u8; frame.len()];
        for ((pixel_a, pixel_b), diff) in frame
            .chunks_exact(4)
            .zip(buf.chunks_exact(4))
            .zip(diff_image.chunks_exact_mut(4))
        {
            if pixel_a != pixel_b {
                differing += 1;
                // red = only rendered set, blue = only golden set, magenta = both differ
                diff.copy_from_slice(&[255, 0, 0, 255]);
                if pixel_b[3] > 0 && pixel_b != [4, 4, 5, 0] {
                    diff[2] = 255;
                    if pixel_a[3] == 0 || pixel_a == [4, 4, 5, 0] {
                        diff[0] = 0;
                    }
                }
            } else {
                // matching pixels shown dimmed
                diff.copy_from_slice(&[pixel_a[0] / 4, pixel_a[1] / 4, pixel_a[2] / 4, 255]);
            }
        }
        if let Some(diff_path) = &diff_out_path {
            let bytes = encode_rgba(geometry.width, geometry.height, &diff_image).expect("encode diff png");
            std::fs::write(diff_path, &bytes).expect("write diff png");
            println!("wrote diff image {diff_path}");
        }

        let total = geometry.width * geometry.height;
        println!(
            "compare: {differing}/{total} differing pixels ({:.4}%), max channel delta {max_channel_delta}",
            differing as f64 / total as f64 * 100.0
        );
        if differing > 0 {
            exit(1);
        }
        println!("PIXEL-EXACT MATCH");
    }

    let vd_active = status.vertical_display_required()
        && efis.terr_on_vd
        && (efis.efis_mode == 2 || efis.efis_mode == 3);
    // VD final frame via the manual azimuth path (A380X without FMS path)
    let vd_final = if vd_active {
        let azim_end = project_wgs84(
            status.latitude,
            status.longitude,
            status.manual_azim_degrees,
            160.0 * NM_TO_METRES,
        );
        let profile_config = ElevationProfileConfig {
            path_width: 1.0,
            waypoints: vec![azim_end],
            range: vd_range_from_nd(efis.nd_range, efis.arc_mode),
            track_changes_significantly_at_distance: -1.0,
            fms_path_used: false,
        };
        let profile =
            extract_elevation_profile(&world, status.latitude, status.longitude, &profile_config);
        Some(render_vertical_display(
            &profile,
            efis.vd_range_lower as f64,
            efis.vd_range_upper as f64,
            -1.0,
        ))
    } else {
        None
    };

    if let Some(vd_golden_path) = &compare_vd_path {
        let Some(vd_final) = &vd_final else {
            eprintln!("--compare-vd given but the scenario renders no VD");
            exit(1);
        };
        let mut golden = png::Decoder::new(std::fs::File::open(vd_golden_path).expect("open vd golden"));
        golden.set_transformations(png::Transformations::EXPAND | png::Transformations::STRIP_16);
        let mut reader = golden.read_info().expect("vd golden header");
        let mut buf = vec![0u8; reader.output_buffer_size()];
        let info = reader.next_frame(&mut buf).expect("vd golden frame");
        buf.truncate(info.buffer_size());
        assert_eq!(
            (info.width as usize, info.height as usize),
            (VD_PROFILE_WIDTH, VD_PROFILE_HEIGHT),
            "vd golden dimensions"
        );

        let differing = vd_final
            .chunks_exact(4)
            .zip(buf.chunks_exact(4))
            .filter(|(a, b)| a != b)
            .count();
        println!(
            "vd compare: {differing}/{} differing pixels",
            VD_PROFILE_WIDTH * VD_PROFILE_HEIGHT
        );
        if differing > 0 {
            exit(1);
        }
        println!("VD PIXEL-EXACT MATCH");
    }

    if let Some(dir) = &animate_dir {
        std::fs::create_dir_all(dir).expect("create animate dir");

        let screen_height = if status.vertical_display_required() {
            SCREEN_HEIGHT_WITH_VD
        } else {
            SCREEN_HEIGHT_WITHOUT_VD
        };

        let mut nd_transition = Transition::new(if status.scanline_mode() {
            TransitionStyle::ScanlineNd
        } else {
            TransitionStyle::Arc
        });
        let mut vd_transition = Transition::new(TransitionStyle::VerticalDisplay);

        // two cycles: the first starts mid-phase (650 ms after startup) to
        // exercise the first-activation offset, the second runs a full sweep
        let mut frame_number = 0usize;
        let mut now_ms: u64 = 650;
        for _cycle in 0..2 {
            nd_transition.start_new_cycle(frame.clone(), geometry.width, geometry.height, now_ms, 0);
            if let Some(vd_final) = &vd_final {
                vd_transition.start_new_cycle(
                    vd_final.clone(),
                    VD_PROFILE_WIDTH,
                    VD_PROFILE_HEIGHT,
                    now_ms,
                    0,
                );
            }

            let mut nd_done = false;
            let mut vd_done = !vd_active;
            while !(nd_done && vd_done) {
                if !nd_done {
                    nd_done = nd_transition.render();
                }
                if !vd_done {
                    vd_done = vd_transition.render();
                }

                let screen = compose_screen_frame(
                    screen_height,
                    nd_transition.current_frame.as_deref(),
                    geometry.width,
                    geometry.height,
                    geometry.offset_x,
                    vd_transition.current_frame.as_deref(),
                    VD_PROFILE_WIDTH,
                    VD_PROFILE_HEIGHT,
                );
                let bytes = encode_rgba(768, screen_height, &screen).expect("encode animate png");
                std::fs::write(format!("{dir}/frame_{frame_number:03}.png"), bytes)
                    .expect("write animate png");
                frame_number += 1;
                now_ms += 40;
            }
            now_ms += 1000;
        }
        println!("wrote {frame_number} animation frames to {dir}");
    }
}
