//! Parser for the SimBridge `terrain.map` database.
//!
//! Port of `apps/server/src/terrain/fileformat/{terrainmap,tile}.ts`.
//!
//! Layout (all little-endian):
//! - 14-byte file header: lat_min i16 @0, lat_max i16 @2, lon_min i16 @4,
//!   lon_max i16 @6, angular_step_lat u8 @8, angular_step_lon u8 @9,
//!   horizontal_resolution f32 @10 (nautical miles).
//! - Tiles follow back-to-back from offset 14. Each tile: 11-byte header
//!   (rows u16 @0, columns u16 @2, sw_lat i8 @4, sw_lon i16 @5,
//!   compressed_len u32 @7) then a GZip payload of `rows*columns` i16
//!   elevations in metres, row-major with row 0 at the tile's NORTH edge.

use std::collections::HashMap;
use std::io::Read;

use crate::jsmath::js_round;

pub const ELEV_INVALID: i16 = 32767;
pub const ELEV_UNKNOWN: i16 = 32766;
pub const ELEV_WATER: i16 = -1;

const FILE_HEADER_BYTES: usize = 14;
const TILE_HEADER_BYTES: usize = 11;
const METRES_TO_FEET: f64 = 3.28084;

#[derive(Debug, Clone, PartialEq)]
pub struct TerrainMapHeader {
    pub lat_min: i16,
    pub lat_max: i16,
    pub lon_min: i16,
    pub lon_max: i16,
    pub angular_step_lat: u8,
    pub angular_step_lon: u8,
    /// Metres, converted from the stored nautical-mile f32 (matches TS `readFloatLE(10) * 1852`).
    pub horizontal_resolution_m: f64,
}

#[derive(Debug, Clone, PartialEq)]
pub struct TileIndex {
    pub sw_lat: i8,
    pub sw_lon: i16,
    pub rows: u16,
    pub columns: u16,
    payload_offset: usize,
    compressed_len: u32,
}

#[derive(Debug)]
pub enum ParseError {
    TruncatedHeader,
    TruncatedTile { offset: usize },
    Inflate(std::io::Error),
    PayloadSizeMismatch { expected: usize, actual: usize },
}

impl std::fmt::Display for ParseError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ParseError::TruncatedHeader => write!(f, "terrain.map shorter than the 14-byte header"),
            ParseError::TruncatedTile { offset } => write!(f, "truncated tile at byte offset {offset}"),
            ParseError::Inflate(e) => write!(f, "tile payload inflate failed: {e}"),
            ParseError::PayloadSizeMismatch { expected, actual } => {
                write!(f, "tile payload holds {actual} bytes, header implies {expected}")
            }
        }
    }
}

impl std::error::Error for ParseError {}

pub struct TerrainMap {
    pub header: TerrainMapHeader,
    pub tiles: Vec<TileIndex>,
    /// SW corner (lat, lon) -> index into `tiles`.
    grid: HashMap<(i8, i16), usize>,
    data: Vec<u8>,
}

impl TerrainMap {
    pub fn from_bytes(data: Vec<u8>) -> Result<Self, ParseError> {
        if data.len() < FILE_HEADER_BYTES {
            return Err(ParseError::TruncatedHeader);
        }

        let header = TerrainMapHeader {
            lat_min: i16::from_le_bytes([data[0], data[1]]),
            lat_max: i16::from_le_bytes([data[2], data[3]]),
            lon_min: i16::from_le_bytes([data[4], data[5]]),
            lon_max: i16::from_le_bytes([data[6], data[7]]),
            angular_step_lat: data[8],
            angular_step_lon: data[9],
            horizontal_resolution_m: f32::from_le_bytes([data[10], data[11], data[12], data[13]])
                as f64
                * 1852.0,
        };

        let mut tiles = Vec::new();
        let mut grid = HashMap::new();
        let mut offset = FILE_HEADER_BYTES;
        while offset < data.len() {
            if offset + TILE_HEADER_BYTES > data.len() {
                return Err(ParseError::TruncatedTile { offset });
            }
            let rows = u16::from_le_bytes([data[offset], data[offset + 1]]);
            let columns = u16::from_le_bytes([data[offset + 2], data[offset + 3]]);
            let sw_lat = data[offset + 4] as i8;
            let sw_lon = i16::from_le_bytes([data[offset + 5], data[offset + 6]]);
            let compressed_len = u32::from_le_bytes([
                data[offset + 7],
                data[offset + 8],
                data[offset + 9],
                data[offset + 10],
            ]);
            let payload_offset = offset + TILE_HEADER_BYTES;
            if payload_offset + compressed_len as usize > data.len() {
                return Err(ParseError::TruncatedTile { offset });
            }

            grid.insert((sw_lat, sw_lon), tiles.len());
            tiles.push(TileIndex {
                sw_lat,
                sw_lon,
                rows,
                columns,
                payload_offset,
                compressed_len,
            });
            offset = payload_offset + compressed_len as usize;
        }

        Ok(Self {
            header,
            tiles,
            grid,
            data,
        })
    }

    /// Index of the tile whose SW corner is exactly (`sw_lat`, `sw_lon`).
    pub fn tile_at_southwest(&self, sw_lat: i8, sw_lon: i16) -> Option<usize> {
        self.grid.get(&(sw_lat, sw_lon)).copied()
    }

    /// Index of the tile containing the coordinate, based on the angular steps
    /// (mirrors `Worldmap.worldMapIndices` grid math).
    pub fn tile_containing(&self, latitude: f64, longitude: f64) -> Option<usize> {
        let lat_step = self.header.angular_step_lat as f64;
        let lon_step = self.header.angular_step_lon as f64;
        let sw_lat = ((latitude + 90.0) / lat_step).floor() * lat_step - 90.0;
        let sw_lon = ((longitude + 180.0) / lon_step).floor() * lon_step - 180.0;
        self.tile_at_southwest(sw_lat as i8, sw_lon as i16)
    }

    /// Decompress a tile and convert metres to feet, keeping the water marker
    /// (-1) untouched — exact port of `Tile.loadElevationGrid`.
    pub fn load_tile_feet(&self, tile_index: usize) -> Result<Vec<i16>, ParseError> {
        let tile = &self.tiles[tile_index];
        let payload =
            &self.data[tile.payload_offset..tile.payload_offset + tile.compressed_len as usize];

        let mut decompressed = Vec::new();
        flate2::read::GzDecoder::new(payload)
            .read_to_end(&mut decompressed)
            .map_err(ParseError::Inflate)?;

        let cell_count = tile.rows as usize * tile.columns as usize;
        if decompressed.len() < cell_count * 2 {
            return Err(ParseError::PayloadSizeMismatch {
                expected: cell_count * 2,
                actual: decompressed.len(),
            });
        }

        let mut elevations = Vec::with_capacity(cell_count);
        for cell in 0..cell_count {
            let metres = i16::from_le_bytes([decompressed[cell * 2], decompressed[cell * 2 + 1]]);
            if metres == ELEV_WATER {
                elevations.push(ELEV_WATER);
            } else {
                elevations.push(js_round(metres as f64 * METRES_TO_FEET) as i16);
            }
        }
        Ok(elevations)
    }
}

/// Row/column of a coordinate inside a tile grid; row 0 is the NORTH edge.
/// Exact port of `ElevationGrid.worldToGridIndices`.
pub fn world_to_grid_indices(
    rows: u16,
    columns: u16,
    sw_lat: f64,
    sw_lon: f64,
    ne_lat: f64,
    ne_lon: f64,
    latitude: f64,
    longitude: f64,
) -> (usize, usize) {
    let lat_range = ne_lat - sw_lat;
    let lat_delta = latitude - sw_lat;
    let row = (rows as f64).min(rows as f64 - ((lat_delta / lat_range) * rows as f64).floor()) - 1.0;

    let lon_range = ne_lon - sw_lon;
    let lon_delta = longitude - sw_lon;
    let column = (columns as f64 - 1.0).min(((lon_delta / lon_range) * columns as f64).floor());

    (row.max(0.0) as usize, column.max(0.0) as usize)
}
