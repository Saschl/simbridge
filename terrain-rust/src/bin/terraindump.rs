//! Inspection tool for the terrain.map database (milestone M1 deliverable).
//!
//! Usage:
//!   terraindump [--db <path>] --info
//!   terraindump [--db <path>] --tile <lat> <lon> [--pgm <out.pgm>]
//!   terraindump [--db <path>] --elevation <lat> <lon>

use std::process::exit;

use fbw_simbridge_terrain::fileformat::{
    world_to_grid_indices, TerrainMap, ELEV_INVALID, ELEV_UNKNOWN, ELEV_WATER,
};

const DEFAULT_DB_PROBES: [&str; 3] = [
    "terrain/terrain.map",
    "../terrain/terrain.map",
    "../cache/terrain.map",
];

fn usage() -> ! {
    eprintln!("Usage:");
    eprintln!("  terraindump [--db <path>] --info");
    eprintln!("  terraindump [--db <path>] --tile <lat> <lon> [--pgm <out.pgm>]");
    eprintln!("  terraindump [--db <path>] --elevation <lat> <lon>");
    exit(2);
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let mut db_path: Option<String> = None;
    let mut mode: Option<&str> = None;
    let mut coords: Option<(f64, f64)> = None;
    let mut pgm_path: Option<String> = None;

    let mut i = 0;
    while i < args.len() {
        match args[i].as_str() {
            "--db" => {
                db_path = Some(args.get(i + 1).cloned().unwrap_or_else(|| usage()));
                i += 2;
            }
            "--pgm" => {
                pgm_path = Some(args.get(i + 1).cloned().unwrap_or_else(|| usage()));
                i += 2;
            }
            "--info" => {
                mode = Some("info");
                i += 1;
            }
            m @ ("--tile" | "--elevation") => {
                mode = Some(&m[2..]);
                let lat = args.get(i + 1).and_then(|v| v.parse::<f64>().ok());
                let lon = args.get(i + 2).and_then(|v| v.parse::<f64>().ok());
                match (lat, lon) {
                    (Some(lat), Some(lon)) => coords = Some((lat, lon)),
                    _ => usage(),
                }
                i += 3;
            }
            _ => usage(),
        }
    }

    let Some(mode) = mode else { usage() };

    let db_path = db_path.or_else(|| {
        DEFAULT_DB_PROBES
            .iter()
            .find(|p| std::path::Path::new(p).is_file())
            .map(|p| p.to_string())
    });
    let Some(db_path) = db_path else {
        eprintln!("No terrain.map found (tried {DEFAULT_DB_PROBES:?}); pass --db <path>");
        exit(1);
    };

    let bytes = match std::fs::read(&db_path) {
        Ok(b) => b,
        Err(e) => {
            eprintln!("Failed to read {db_path}: {e}");
            exit(1);
        }
    };
    let file_len = bytes.len();
    let map = match TerrainMap::from_bytes(bytes) {
        Ok(m) => m,
        Err(e) => {
            eprintln!("Failed to parse {db_path}: {e}");
            exit(1);
        }
    };

    match mode {
        "info" => info(&map, &db_path, file_len),
        "tile" => {
            let (lat, lon) = coords.unwrap();
            dump_tile(&map, lat, lon, pgm_path.as_deref());
        }
        "elevation" => {
            let (lat, lon) = coords.unwrap();
            elevation(&map, lat, lon);
        }
        _ => usage(),
    }
}

fn info(map: &TerrainMap, path: &str, file_len: usize) {
    let h = &map.header;
    println!("terrain.map: {path} ({file_len} bytes)");
    println!(
        "  latitude range:  [{}, {}]  longitude range: [{}, {}]",
        h.lat_min, h.lat_max, h.lon_min, h.lon_max
    );
    println!(
        "  angular steps:   {}° lat x {}° lon per tile",
        h.angular_step_lat, h.angular_step_lon
    );
    println!("  horizontal res:  {:.1} m", h.horizontal_resolution_m);
    println!("  tiles:           {}", map.tiles.len());

    let (mut min_rows, mut max_rows, mut min_cols, mut max_cols) = (u16::MAX, 0u16, u16::MAX, 0u16);
    for t in &map.tiles {
        min_rows = min_rows.min(t.rows);
        max_rows = max_rows.max(t.rows);
        min_cols = min_cols.min(t.columns);
        max_cols = max_cols.max(t.columns);
    }
    let payload_bytes: usize = file_len - 14 - map.tiles.len() * 11;
    println!("  tile grid dims:  rows {min_rows}..{max_rows}, columns {min_cols}..{max_cols}");
    println!("  payload bytes:   {payload_bytes} (compressed)");
}

fn dump_tile(map: &TerrainMap, lat: f64, lon: f64, pgm: Option<&str>) {
    let Some(idx) = map.tile_containing(lat, lon) else {
        println!("No tile covers ({lat}, {lon}) — treated as water by the renderer");
        return;
    };
    let tile = map.tiles[idx].clone();
    println!(
        "Tile #{idx}: SW corner ({}, {}), {} rows x {} columns",
        tile.sw_lat, tile.sw_lon, tile.rows, tile.columns
    );

    let elevations = match map.load_tile_feet(idx) {
        Ok(e) => e,
        Err(e) => {
            eprintln!("Failed to load tile: {e}");
            exit(1);
        }
    };

    let mut min = i16::MAX;
    let mut max = i16::MIN;
    let mut water = 0usize;
    for &e in &elevations {
        if e == ELEV_WATER {
            water += 1;
        } else {
            min = min.min(e);
            max = max.max(e);
        }
    }
    if min == i16::MAX {
        println!("  all {} cells are water", elevations.len());
    } else {
        println!(
            "  elevation range: {min}..{max} ft, water cells: {water}/{}",
            elevations.len()
        );
    }

    if let Some(pgm_path) = pgm {
        write_pgm(pgm_path, tile.columns as usize, tile.rows as usize, &elevations, min, max);
        println!("  wrote {pgm_path}");
    }
}

fn write_pgm(path: &str, width: usize, height: usize, elevations: &[i16], min: i16, max: i16) {
    let span = ((max as i32 - min as i32).max(1)) as f64;
    let mut out = format!("P5\n{width} {height}\n255\n").into_bytes();
    out.reserve(elevations.len());
    for &e in elevations {
        // water -> black; land normalized into 1..=255
        let px = if e == ELEV_WATER {
            0u8
        } else {
            (1.0 + (e as i32 - min as i32) as f64 / span * 254.0) as u8
        };
        out.push(px);
    }
    if let Err(e) = std::fs::write(path, out) {
        eprintln!("Failed to write {path}: {e}");
        exit(1);
    }
}

fn elevation(map: &TerrainMap, lat: f64, lon: f64) {
    let Some(idx) = map.tile_containing(lat, lon) else {
        println!("({lat}, {lon}): water (no tile in database)");
        return;
    };
    let tile = map.tiles[idx].clone();
    let elevations = match map.load_tile_feet(idx) {
        Ok(e) => e,
        Err(e) => {
            eprintln!("Failed to load tile: {e}");
            exit(1);
        }
    };

    let sw_lat = tile.sw_lat as f64;
    let sw_lon = tile.sw_lon as f64;
    let ne_lat = sw_lat + map.header.angular_step_lat as f64;
    let ne_lon = sw_lon + map.header.angular_step_lon as f64;
    let (row, col) =
        world_to_grid_indices(tile.rows, tile.columns, sw_lat, sw_lon, ne_lat, ne_lon, lat, lon);
    let value = elevations[row * tile.columns as usize + col];

    match value {
        ELEV_WATER => println!("({lat}, {lon}): water"),
        ELEV_UNKNOWN => println!("({lat}, {lon}): unknown"),
        ELEV_INVALID => println!("({lat}, {lon}): invalid"),
        ft => println!("({lat}, {lon}): {ft} ft (tile #{idx}, row {row}, col {col})"),
    }
}
