//! World elevation map: tile visibility, loading/eviction and stitching into
//! one flat i16 buffer around the aircraft's ground-truth position.
//!
//! Port of `apps/server/src/terrain/mapdata/{worldmap,tilemanager}.ts` and the
//! stitching/ego-position part of `apps/server/src/terrain/processing/maphandler.ts`.
//! The TS version kept the stitched map as a Float32Array and uploaded it as a
//! GPU texture; elevations are integral feet in i16 range, so we keep i16 and
//! halve the memory.

use std::collections::{HashMap, HashSet};
use std::sync::Arc;

use crate::fileformat::{TerrainMap, ELEV_INVALID, ELEV_UNKNOWN, ELEV_WATER};
use crate::geodesy::{degrees_per_pixel, project_wgs84, NM_TO_METRES};

/// Radius around the aircraft kept in memory, nautical miles (`Worldmap.VisibilityRange`).
pub const VISIBILITY_RANGE_NM: f64 = 800.0;
/// Upper bound of the stitched map per dimension (`GpuMaxPixelSize` — kept as a
/// plain buffer cap even though there is no GPU anymore).
pub const MAX_WORLD_PIXELS: usize = 16384;
/// Fallback tile grid dimension (`DefaultTileSize`).
pub const DEFAULT_TILE_SIZE: usize = 300;

pub struct GridLookup {
    pub sw_lat: f64,
    pub sw_lon: f64,
    pub ne_lat: f64,
    pub ne_lon: f64,
    /// Tile-grid cell indices (row, column), sorted north->south, west->east.
    pub grid: Vec<Vec<(usize, usize)>>,
    pub min_width_per_tile: usize,
    pub min_height_per_tile: usize,
}

/// Immutable snapshot handed to the renderers. Cheap to clone.
#[derive(Clone)]
pub struct WorldMap {
    pub sw_lat: f64,
    pub sw_lon: f64,
    pub ne_lat: f64,
    pub ne_lon: f64,
    pub width: usize,
    pub height: usize,
    pub elevations: Arc<Vec<i16>>,
    /// Ground-truth aircraft position and its (fractional) pixel coordinate.
    pub ground_truth_lat: f64,
    pub ground_truth_lon: f64,
    pub ego_x: f64,
    pub ego_y: f64,
}

impl WorldMap {
    #[inline]
    pub fn elevation_at_pixel(&self, x: i64, y: i64) -> i16 {
        if x < 0 || y < 0 || x >= self.width as i64 || y >= self.height as i64 {
            ELEV_UNKNOWN
        } else {
            self.elevations[y as usize * self.width + x as usize]
        }
    }

    /// Point elevation lookup (port of `MapHandler.extractElevation`).
    /// `aircraft_latitude` is the ADIRU latitude used for the pole checks.
    pub fn extract_elevation(&self, aircraft_latitude: f64, latitude: f64, longitude: f64) -> i16 {
        if self.elevations.is_empty() {
            return ELEV_INVALID;
        }

        let (lat_step, lon_step) = degrees_per_pixel(
            self.sw_lat,
            self.sw_lon,
            self.ne_lat,
            self.ne_lon,
            aircraft_latitude,
            self.width,
            self.height,
        );
        let lat_pixel_delta = (self.ground_truth_lat - latitude) / lat_step;
        let lon_pixel_delta = (longitude - self.ground_truth_lon) / lon_step;

        // FIXED vs the TS original, which floored the *combined* flat index
        // ((egoY+latDelta)*width + egoX+lonDelta): the fractional part of the
        // y coordinate leaked width-scaled pixels into x, shifting the lookup
        // thousands of cells sideways. Floor row and column separately.
        self.elevation_at_pixel(
            (self.ego_x + lon_pixel_delta).floor() as i64,
            (self.ego_y + lat_pixel_delta).floor() as i64,
        )
    }
}

pub struct WorldMapManager {
    terrain: TerrainMap,
    /// Tile-grid dimensions (180 x 360 for 1-degree steps).
    rows: usize,
    columns: usize,
    lat_step: f64,
    lon_step: f64,
    /// Loaded tiles in feet, keyed by tile-grid (row, column).
    loaded: HashMap<(usize, usize), Arc<Vec<i16>>>,
    /// Metadata of the last stitched map.
    stitched: Option<StitchedMeta>,
    elevations: Arc<Vec<i16>>,
    cached_tile_count: usize,
    snapshot: Option<WorldMap>,
}

struct StitchedMeta {
    sw_lat: f64,
    sw_lon: f64,
    ne_lat: f64,
    ne_lon: f64,
    width: usize,
    height: usize,
    min_width_per_tile: usize,
    min_height_per_tile: usize,
}

impl WorldMapManager {
    pub fn new(terrain: TerrainMap) -> Self {
        let lat_step = terrain.header.angular_step_lat as f64;
        let lon_step = terrain.header.angular_step_lon as f64;
        // TS TileManager builds the grid with `lat < 90` / `lon < 180` loops.
        let rows = (180.0 / lat_step).ceil() as usize;
        let columns = (360.0 / lon_step).ceil() as usize;
        Self {
            terrain,
            rows,
            columns,
            lat_step,
            lon_step,
            loaded: HashMap::new(),
            stitched: None,
            elevations: Arc::new(Vec::new()),
            cached_tile_count: 0,
            snapshot: None,
        }
    }

    pub fn terrain(&self) -> &TerrainMap {
        &self.terrain
    }

    pub fn snapshot(&self) -> Option<WorldMap> {
        self.snapshot.clone()
    }

    pub fn reset(&mut self) {
        self.loaded.clear();
        self.stitched = None;
        self.elevations = Arc::new(Vec::new());
        self.cached_tile_count = 0;
        self.snapshot = None;
    }

    /// Tile-grid indices for a coordinate (`Worldmap.worldMapIndices`).
    pub fn world_map_indices(&self, latitude: f64, longitude: f64) -> Option<(usize, usize)> {
        let row = ((latitude + 90.0) / self.lat_step).floor();
        let column = ((longitude + 180.0) / self.lon_step).floor();
        if row < 0.0 || row >= self.rows as f64 || column < 0.0 || column >= self.columns as f64 {
            None
        } else {
            Some((row as usize, column as usize))
        }
    }

    fn tile_index_of_cell(&self, row: usize, column: usize) -> Option<usize> {
        let sw_lat = row as f64 * self.lat_step - 90.0;
        let sw_lon = column as f64 * self.lon_step - 180.0;
        self.terrain.tile_at_southwest(sw_lat as i8, sw_lon as i16)
    }

    /// SW corner of the tile containing the coordinate (`getSouthwestCoordinateOfTile`).
    fn southwest_of_tile(&self, latitude: f64, longitude: f64) -> Option<(f64, f64)> {
        let (row, column) = self.world_map_indices(latitude, longitude)?;
        Some((
            row as f64 * self.lat_step - 90.0,
            column as f64 * self.lon_step - 180.0,
        ))
    }

    /// Port of `Worldmap.createGridLookupTable` (clamped instead of crashing
    /// where TS would dereference `undefined` grid indices).
    pub fn create_grid_lookup(&self, latitude: f64, longitude: f64) -> GridLookup {
        let range = VISIBILITY_RANGE_NM * NM_TO_METRES;
        let south = project_wgs84(latitude, longitude, 180.0, range).0;
        let southwest = project_wgs84(latitude, longitude, 225.0, range);
        let west = project_wgs84(latitude, longitude, 270.0, range).1;
        let north = project_wgs84(latitude, longitude, 0.0, range).0;
        let east = project_wgs84(latitude, longitude, 90.0, range).1;
        let northeast = project_wgs84(latitude, longitude, 45.0, range);

        let mut sw_lat = south.min(southwest.0);
        let mut ne_lat = north.max(northeast.0);
        let mut sw_lon = west.min(southwest.1);
        let mut ne_lon = east.min(northeast.1);

        // 180 degree wrap arounds for the western/eastern coordinates
        if west * southwest.1 < 0.0 {
            sw_lon = west.max(southwest.1);
        }
        if east * northeast.1 < 0.0 {
            ne_lon = east.max(northeast.1);
        }

        let clamp = |row: f64, column: f64| -> (usize, usize) {
            (
                (row.max(0.0) as usize).min(self.rows - 1),
                (column.max(0.0) as usize).min(self.columns - 1),
            )
        };
        let sw_grid = clamp(
            ((sw_lat + 90.0) / self.lat_step).floor(),
            ((sw_lon + 180.0) / self.lon_step).floor(),
        );
        let ne_grid = clamp(
            ((ne_lat + 90.0) / self.lat_step).floor(),
            ((ne_lon + 180.0) / self.lon_step).floor(),
        );

        let mut row_count = ne_grid.0 as i64 - sw_grid.0 as i64;
        let mut row_direction: i64 = 1;
        if sw_lat >= latitude {
            // south pole
            row_count = sw_grid.0 as i64 + ne_grid.0 as i64;
            row_direction = -1;
        } else if ne_lat <= latitude {
            // north pole
            row_count = 2 * self.rows as i64 - sw_grid.0 as i64 - ne_grid.0 as i64;
        }
        row_count += 1;

        let mut column_count = ne_grid.1 as i64 - sw_grid.1 as i64;
        if ne_lon < sw_lon {
            // wrap around at 180°
            column_count = self.columns as i64 - sw_grid.1 as i64 + ne_grid.1 as i64;
        }
        column_count += 1;

        // build the lookup grid, sorted north->south / west->east
        let row_count = row_count.max(1) as usize;
        let column_count = column_count.max(1) as usize;
        let mut grid = vec![Vec::new(); row_count];
        for y in 0..row_count {
            let mut row = sw_grid.0 as i64 + row_direction * y as i64;
            if row < 0 {
                row = row.abs();
            }
            if row >= self.rows as i64 {
                row -= self.rows as i64;
            }

            let mut cells = Vec::with_capacity(column_count);
            for x in 0..column_count {
                let column = (sw_grid.1 + x) % self.columns;
                cells.push((row as usize, column));
            }
            grid[row_count - 1 - y] = cells;
        }

        // minimum tile dimensions across the region
        let mut min_width_per_tile = 5000usize;
        let mut min_height_per_tile = 5000usize;
        for row in &grid {
            for &(cell_row, cell_col) in row {
                if let Some(tile_idx) = self.tile_index_of_cell(cell_row, cell_col) {
                    let tile = &self.terrain.tiles[tile_idx];
                    min_width_per_tile = min_width_per_tile.min(tile.columns as usize);
                    min_height_per_tile = min_height_per_tile.min(tile.rows as usize);
                }
            }
        }
        if min_width_per_tile == 5000 {
            min_width_per_tile = DEFAULT_TILE_SIZE;
        }
        if min_height_per_tile == 5000 {
            min_height_per_tile = DEFAULT_TILE_SIZE;
        }

        let map_height = min_height_per_tile * grid.len();
        let map_width = min_width_per_tile * grid[0].len();

        // shared row clipping between top and bottom
        if map_height > MAX_WORLD_PIXELS {
            let clipping_tiles =
                (map_height - MAX_WORLD_PIXELS).div_ceil(min_height_per_tile);
            let top = clipping_tiles.div_ceil(2);
            let bottom = clipping_tiles / 2;

            grid.drain(0..top.min(grid.len()));
            let keep = grid.len().saturating_sub(bottom);
            grid.truncate(keep);

            ne_lat -= self.lat_step * top as f64;
            sw_lat += self.lat_step * bottom as f64;
        }

        // shared column clipping between left and right
        if map_width > MAX_WORLD_PIXELS {
            let clipping_tiles = (map_width - MAX_WORLD_PIXELS).div_ceil(min_width_per_tile);
            let start = clipping_tiles.div_ceil(2);
            let end = clipping_tiles / 2;

            for row in &mut grid {
                row.drain(0..start.min(row.len()));
                let keep = row.len().saturating_sub(end);
                row.truncate(keep);
            }

            sw_lon += self.lon_step * start as f64;
            ne_lon -= self.lon_step * end as f64;

            if sw_lon >= 180.0 {
                sw_lon -= 360.0;
            }
            if ne_lon < -180.0 {
                ne_lon += 360.0;
            }
        }

        GridLookup {
            sw_lat,
            sw_lon,
            ne_lat,
            ne_lon,
            grid,
            min_width_per_tile,
            min_height_per_tile,
        }
    }

    /// Load any missing tiles of the lookup grid; returns whether any tile was
    /// loaded (`Worldmap.updatePosition`).
    fn load_missing_tiles(&mut self, lookup: &GridLookup) -> bool {
        let mut loaded_any = false;
        for row in &lookup.grid {
            for &cell in row {
                if self.loaded.contains_key(&cell) {
                    continue;
                }
                if let Some(tile_idx) = self.tile_index_of_cell(cell.0, cell.1) {
                    if let Ok(feet) = self.terrain.load_tile_feet(tile_idx) {
                        self.loaded.insert(cell, Arc::new(feet));
                        loaded_any = true;
                    }
                }
            }
        }
        loaded_any
    }

    /// Evict tiles that fell out of the visible grid (`cleanupElevationCache`).
    fn cleanup_tiles(&mut self, lookup: &GridLookup) {
        let keep: HashSet<(usize, usize)> = lookup
            .grid
            .iter()
            .flat_map(|row| row.iter().copied())
            .collect();
        self.loaded.retain(|cell, _| keep.contains(cell));
    }

    /// Port of `MapHandler.updateGroundTruthPositionAndCachedTiles`: refresh
    /// tiles, restitch when the tile set changed, and recompute the ego pixel
    /// position every call. Returns the refreshed snapshot.
    pub fn update_position(&mut self, latitude: f64, longitude: f64) -> Option<WorldMap> {
        let lookup = self.create_grid_lookup(latitude, longitude);
        let tiles_loaded = self.load_missing_tiles(&lookup);
        let relevant_tile_count = lookup.grid.len() * lookup.grid[0].len();

        if tiles_loaded || self.cached_tile_count != relevant_tile_count {
            self.stitch(&lookup);
            self.cleanup_tiles(&lookup);
            self.cached_tile_count = relevant_tile_count;
        }

        let meta = self.stitched.as_ref()?;

        // ego pixel position, recomputed on every position update
        let (ego_x, ego_y) = if let Some((tile_sw_lat, tile_sw_lon)) =
            self.southwest_of_tile(latitude, longitude)
        {
            let lat_step = self.lat_step / meta.min_height_per_tile as f64;
            let lon_step = self.lon_step / meta.min_width_per_tile as f64;
            let lat_delta = latitude - tile_sw_lat;
            let lon_delta = longitude - tile_sw_lon;

            let mut x_offset = 0.0;
            let mut y_offset = 0.0;
            if let Some(ego_index) = self.world_map_indices(latitude, longitude) {
                for (row_idx, row) in lookup.grid.iter().enumerate() {
                    if row.first().is_some_and(|cell| cell.0 == ego_index.0) {
                        for (column_idx, cell) in row.iter().enumerate() {
                            if cell.1 == ego_index.1 {
                                y_offset = (row_idx * meta.min_height_per_tile) as f64;
                                x_offset = (column_idx * meta.min_width_per_tile) as f64;
                            }
                        }
                    }
                }
            }

            (
                x_offset + lon_delta / lon_step,
                y_offset + meta.min_height_per_tile as f64 - lat_delta / lat_step,
            )
        } else {
            (meta.width as f64 / 2.0, meta.height as f64 / 2.0)
        };

        let snapshot = WorldMap {
            sw_lat: meta.sw_lat,
            sw_lon: meta.sw_lon,
            ne_lat: meta.ne_lat,
            ne_lon: meta.ne_lon,
            width: meta.width,
            height: meta.height,
            elevations: Arc::clone(&self.elevations),
            ground_truth_lat: latitude,
            ground_truth_lon: longitude,
            ego_x,
            ego_y,
        };
        self.snapshot = Some(snapshot.clone());
        Some(snapshot)
    }

    fn stitch(&mut self, lookup: &GridLookup) {
        let world_width = lookup.min_width_per_tile * lookup.grid[0].len();
        let world_height = lookup.min_height_per_tile * lookup.grid.len();
        let mut data = Vec::with_capacity(world_width * world_height);

        for row in &lookup.grid {
            for y in 0..lookup.min_height_per_tile {
                for &cell in row {
                    let tile_idx = self.tile_index_of_cell(cell.0, cell.1);
                    let elevations = tile_idx.and_then(|_| self.loaded.get(&cell));

                    match (tile_idx, elevations) {
                        (None, _) => {
                            // no tile in the database -> open water
                            data.extend(std::iter::repeat_n(ELEV_WATER, lookup.min_width_per_tile));
                        }
                        (Some(_), None) => {
                            // tile exists but is not loaded -> unknown
                            data.extend(std::iter::repeat_n(ELEV_UNKNOWN, lookup.min_width_per_tile));
                        }
                        (Some(tile_idx), Some(elevations)) => {
                            let tile = &self.terrain.tiles[tile_idx];
                            // share the subsampling crop between all tile sides
                            let row_delta = tile.rows as usize - lookup.min_height_per_tile;
                            let column_delta = tile.columns as usize - lookup.min_width_per_tile;
                            let y_off = row_delta.div_ceil(2);
                            let x_off = column_delta.div_ceil(2);

                            let start = (y + y_off) * tile.columns as usize + x_off;
                            data.extend_from_slice(
                                &elevations[start..start + lookup.min_width_per_tile],
                            );
                        }
                    }
                }
            }
        }

        // metadata corners snapped to the tile grid (maphandler.ts:287-296)
        let clamp = |lat: f64, lon: f64| -> (usize, usize) {
            (
                ((((lat + 90.0) / self.lat_step).floor()).max(0.0) as usize).min(self.rows - 1),
                ((((lon + 180.0) / self.lon_step).floor()).max(0.0) as usize).min(self.columns - 1),
            )
        };
        let sw_grid = clamp(lookup.sw_lat, lookup.sw_lon);
        let ne_grid = clamp(lookup.ne_lat, lookup.ne_lon);

        self.elevations = Arc::new(data);
        self.stitched = Some(StitchedMeta {
            sw_lat: sw_grid.0 as f64 * self.lat_step - 90.0,
            sw_lon: sw_grid.1 as f64 * self.lon_step - 180.0,
            ne_lat: ne_grid.0 as f64 * self.lat_step - 90.0 + self.lat_step,
            ne_lon: ne_grid.1 as f64 * self.lon_step - 180.0 + self.lon_step,
            width: world_width,
            height: world_height,
            min_width_per_tile: lookup.min_width_per_tile,
            min_height_per_tile: lookup.min_height_per_tile,
        });
    }
}
