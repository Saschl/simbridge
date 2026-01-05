//! World map handler for terrain data management

#![allow(dead_code)]
#![allow(unused_imports)]

use crate::types::{
    constants::NAUTICAL_MILES_TO_METRES,
    GridDefinition, GridIndex, GridLookupData, PositionData,
};
use crate::fileformat::{TerrainMap, Tile};
use crate::mapdata::TileManager;

/// Helper functions for WGS84 calculations
pub mod geo {
    use std::f64::consts::PI;

    pub fn deg2rad(degree: f64) -> f64 {
        degree * (PI / 180.0)
    }

    pub fn rad2deg(radian: f64) -> f64 {
        radian * (180.0 / PI)
    }

    /// Calculate distance between two points in nautical miles
    pub fn distance_wgs84(lat0: f64, lon0: f64, lat1: f64, lon1: f64) -> f64 {
        let delta_latitude = deg2rad(lat1 - lat0);
        let delta_longitude = deg2rad(lon1 - lon0);
        let latitude0_radian = deg2rad(lat0);
        let latitude1_radian = deg2rad(lat1);

        let a = 0.5 - delta_latitude.cos() * 0.5
            + latitude0_radian.cos() * latitude1_radian.cos()
            * (1.0 - delta_longitude.cos()) * 0.5;

        let distance_metres = 12742020.0 * a.sqrt().asin();
        distance_metres * 0.000539957
    }

    /// Project a point from a given position along a bearing for a distance
    pub fn project_wgs84(latitude: f64, longitude: f64, bearing: f64, distance: f64) -> (f64, f64) {
        let lat_rad = deg2rad(latitude);
        let long_rad = deg2rad(longitude);
        let bearing_rad = deg2rad(bearing);
        let ratio = distance / 6371010.0;

        let mut lat_dest = (lat_rad.sin() * ratio.cos()
            + lat_rad.cos() * ratio.sin() * bearing_rad.cos()).asin();
        let mut long_dest = long_rad
            + (bearing_rad.sin() * ratio.sin() * lat_rad.cos()).atan2(
                ratio.cos() - lat_rad.sin() * lat_dest.sin()
            );

        // Ensure latitude is between [-90.0, 90.0]
        lat_dest = rad2deg(lat_dest);
        if lat_dest < -90.0 { lat_dest = -180.0 - lat_dest; }
        if lat_dest > 90.0 { lat_dest = 180.0 - lat_dest; }

        // Ensure longitude is between [-180.0, 180.0]
        long_dest = rad2deg(long_dest);
        if long_dest < -180.0 { long_dest = 360.0 + long_dest; }
        if long_dest > 180.0 { long_dest -= 360.0; }

        (lat_dest, long_dest)
    }

    /// Calculate bearing between two points
    pub fn bearing_wgs84(lat0: f64, lon0: f64, lat1: f64, lon1: f64) -> f64 {
        let start_lat = deg2rad(lat0);
        let start_long = deg2rad(lon0);
        let end_lat = deg2rad(lat1);
        let end_long = deg2rad(lon1);

        let y = (end_long - start_long).sin() * end_lat.cos();
        let x = start_lat.cos() * end_lat.sin()
            - start_lat.sin() * end_lat.cos() * (end_long - start_long).cos();
        let bearing = y.atan2(x) + PI;

        (rad2deg(bearing) + 360.0) % 360.0
    }

    /// Normalize heading to [0, 360)
    pub fn normalize_heading(angle: f64) -> f64 {
        angle - (angle / 360.0).floor() * 360.0
    }
}

/// World map containing terrain data
pub struct Worldmap {
    /// Grid definition for the terrain
    pub grid_data: GridDefinition,
    /// Tile manager for handling individual tiles
    pub tile_manager: TileManager,
    /// Reference to terrain data
    terrain_data: TerrainMap,
    /// Visibility range in nautical miles
    pub visibility_range: f64,
}

impl Worldmap {
    /// Create a new world map from terrain data
    pub fn new(terrain_data: TerrainMap) -> Self {
        let tile_manager = TileManager::new(&terrain_data);

        let grid_data = GridDefinition {
            rows: tile_manager.rows(),
            columns: tile_manager.columns(),
            latitude_step: terrain_data.latitude_step(),
            longitude_step: terrain_data.longitude_step(),
        };

        Worldmap {
            grid_data,
            tile_manager,
            terrain_data,
            visibility_range: 800.0,
        }
    }

    /// Reset internal data (clear cached elevation maps)
    pub fn reset_internal_data(&mut self) {
        self.tile_manager.clear_elevation_cache();
    }

    /// Find tile index for given coordinates
    pub fn find_tile_index(tiles: &[Tile], latitude: f64, longitude: f64) -> Option<usize> {
        tiles.iter().position(|t| {
            t.southwest.latitude as i32 == latitude as i32
                && t.southwest.longitude as i32 == longitude as i32
        })
    }

    /// Create a grid lookup table for the given position
    pub fn create_grid_lookup_table(
        &self,
        position: PositionData,
        max_width: usize,
        max_height: usize,
        default_tile_size: usize,
    ) -> GridLookupData {
        let distance_m = self.visibility_range * NAUTICAL_MILES_TO_METRES;

        let south = geo::project_wgs84(position.latitude, position.longitude, 180.0, distance_m).0;
        let southwest = geo::project_wgs84(position.latitude, position.longitude, 225.0, distance_m);
        let west = geo::project_wgs84(position.latitude, position.longitude, 270.0, distance_m).1;
        let north = geo::project_wgs84(position.latitude, position.longitude, 0.0, distance_m).0;
        let east = geo::project_wgs84(position.latitude, position.longitude, 90.0, distance_m).1;
        let northeast = geo::project_wgs84(position.latitude, position.longitude, 45.0, distance_m);

        let mut southwest_lat = south.min(southwest.0);
        let mut northeast_lat = north.max(northeast.0);
        let mut southwest_long = west.min(southwest.1);
        let mut northeast_long = east.min(northeast.1);

        // Handle the 180 degree wrap around for the western coordinate
        if west * southwest.1 < 0.0 {
            southwest_long = west.max(southwest.1);
        }
        // Handle the 180 degree wrap around for the eastern coordinate
        if east * northeast.1 < 0.0 {
            northeast_long = east.max(northeast.1);
        }

        let southwest_grid = self.world_map_indices(southwest_lat, southwest_long);
        let northeast_grid = self.world_map_indices(northeast_lat, northeast_long);

        let (southwest_grid, northeast_grid) = match (southwest_grid, northeast_grid) {
            (Some(sw), Some(ne)) => (sw, ne),
            _ => return GridLookupData::default(),
        };

        let mut row_count;
        let mut row_direction = 1i32;

        if southwest_lat >= position.latitude {
            // We are at the south pole
            row_count = southwest_grid.row + northeast_grid.row;
            row_direction = -1;
        } else if northeast_lat <= position.latitude {
            // We are at the north pole
            row_count = self.tile_manager.rows() - southwest_grid.row
                + self.tile_manager.rows() - northeast_grid.row;
        } else {
            row_count = northeast_grid.row.saturating_sub(southwest_grid.row);
        }
        row_count += 1;

        let mut column_count;
        if northeast_long < southwest_long {
            // Wrap around at 180°
            column_count = self.tile_manager.columns() - southwest_grid.column + northeast_grid.column;
        } else {
            column_count = northeast_grid.column.saturating_sub(southwest_grid.column);
        }
        column_count += 1;

        // Create the lookup table sorted from north->south and west->east
        let mut grid = Vec::with_capacity(row_count);
        for y in 0..row_count {
            let mut row_idx = southwest_grid.row as i32 + row_direction * y as i32;

            // Ensure row index is not outside bounds
            if row_idx < 0 { row_idx = row_idx.abs(); }
            if row_idx >= self.tile_manager.rows() as i32 {
                row_idx -= self.tile_manager.rows() as i32;
            }

            let mut row = Vec::with_capacity(column_count);
            for x in 0..column_count {
                let column = (southwest_grid.column + x) % self.tile_manager.columns();
                row.push(GridIndex { row: row_idx as usize, column });
            }
            grid.insert(0, row); // Insert at front to reverse order (north to south)
        }

        // Find minimum dimensions per tile
        let mut min_width_per_tile = 5000usize;
        let mut min_height_per_tile = 5000usize;

        for row in &grid {
            for cell_idx in row {
                let cell = &self.tile_manager.grid[cell_idx.row][cell_idx.column];
                if cell.tile_index != -1 {
                    let tile = &self.terrain_data.tiles[cell.tile_index as usize];
                    min_width_per_tile = min_width_per_tile.min(tile.columns());
                    min_height_per_tile = min_height_per_tile.min(tile.rows());
                }
            }
        }

        if min_width_per_tile == 5000 { min_width_per_tile = default_tile_size; }
        if min_height_per_tile == 5000 { min_height_per_tile = default_tile_size; }

        let map_height = min_height_per_tile * grid.len();
        let map_width = min_width_per_tile * grid.first().map(|r| r.len()).unwrap_or(0);

        // Clip grid if necessary
        let mut grid = grid;

        // Delete rows if necessary
        if map_height > max_height {
            let clipping_tile_count = (map_height - max_height + min_height_per_tile - 1) / min_height_per_tile;
            let top_clipping_count = (clipping_tile_count + 1) / 2;
            let bottom_clipping_count = clipping_tile_count / 2;

            for _ in 0..top_clipping_count {
                if !grid.is_empty() { grid.remove(0); }
            }
            for _ in 0..bottom_clipping_count {
                if !grid.is_empty() { grid.pop(); }
            }

            northeast_lat -= self.grid_data.latitude_step * top_clipping_count as f64;
            southwest_lat += self.grid_data.latitude_step * bottom_clipping_count as f64;
        }

        // Delete columns if necessary
        if map_width > max_width {
            let clipping_tile_count = (map_width - max_width + min_width_per_tile - 1) / min_width_per_tile;
            let start_tile_clipping = (clipping_tile_count + 1) / 2;
            let end_tile_clipping = clipping_tile_count / 2;

            for row in &mut grid {
                for _ in 0..start_tile_clipping {
                    if !row.is_empty() { row.remove(0); }
                }
                for _ in 0..end_tile_clipping {
                    if !row.is_empty() { row.pop(); }
                }
            }

            southwest_long += self.grid_data.longitude_step * start_tile_clipping as f64;
            northeast_long -= self.grid_data.longitude_step * end_tile_clipping as f64;

            // Ensure correct updates at -180.0, 180.0 degree wrap around
            if southwest_long >= 180.0 { southwest_long -= 360.0; }
            if northeast_long < -180.0 { northeast_long += 360.0; }
        }

        // Get actual southwest/northeast from tile grid corners (like TypeScript)
        let actual_southwest = if !grid.is_empty() && !grid.last().unwrap().is_empty() {
            // Last row (southernmost), first column (westernmost)
            let sw_idx = &grid.last().unwrap()[0];
            let sw_cell = &self.tile_manager.grid[sw_idx.row][sw_idx.column];
            PositionData {
                latitude: sw_cell.southwest.latitude,
                longitude: sw_cell.southwest.longitude,
            }
        } else {
            PositionData { latitude: southwest_lat, longitude: southwest_long }
        };

        let actual_northeast = if !grid.is_empty() && !grid[0].is_empty() {
            // First row (northernmost), last column (easternmost)
            let ne_idx = grid[0].last().unwrap();
            let ne_cell = &self.tile_manager.grid[ne_idx.row][ne_idx.column];
            PositionData {
                latitude: ne_cell.southwest.latitude + self.grid_data.latitude_step,
                longitude: ne_cell.southwest.longitude + self.grid_data.longitude_step,
            }
        } else {
            PositionData { latitude: northeast_lat, longitude: northeast_long }
        };

        GridLookupData {
            southwest: actual_southwest,
            northeast: actual_northeast,
            grid,
            min_width_per_tile,
            min_height_per_tile,
        }
    }

    /// Update position and load relevant tiles
    pub fn update_position(&mut self, relevant_tiles: &[Vec<GridIndex>]) -> bool {
        let mut loaded_tiles = 0;

        for row in relevant_tiles {
            for cell_idx in row {
                let cell = &self.tile_manager.grid[cell_idx.row][cell_idx.column];

                if cell.tile_index != -1 && cell.elevation_map.is_none() {
                    let tile = &self.terrain_data.tiles[cell.tile_index as usize];
                    if let Ok(map) = tile.load_elevation_grid() {
                        self.tile_manager.set_elevation_map(*cell_idx, map);
                        loaded_tiles += 1;
                    }
                }
            }
        }

        loaded_tiles > 0
    }

    /// Get world map indices for a given position
    pub fn world_map_indices(&self, latitude: f64, longitude: f64) -> Option<GridIndex> {
        let row = ((latitude + 90.0) / self.grid_data.latitude_step).floor() as i32;
        let column = ((longitude + 180.0) / self.grid_data.longitude_step).floor() as i32;

        if row < 0 || row >= self.grid_data.rows as i32
            || column < 0 || column >= self.grid_data.columns as i32
        {
            return None;
        }

        Some(GridIndex { row: row as usize, column: column as usize })
    }

    /// Check if a tile is valid
    pub fn valid_tile(&self, index: GridIndex) -> bool {
        index.row < self.tile_manager.rows()
            && index.column < self.tile_manager.columns()
            && self.tile_manager.grid[index.row][index.column].tile_index != -1
    }

    /// Get elevation at a specific position
    pub fn get_elevation(&self, position: PositionData) -> Option<i16> {
        let grid_idx = self.world_map_indices(position.latitude, position.longitude)?;
        let cell = &self.tile_manager.grid[grid_idx.row][grid_idx.column];
        let elevation_map = cell.elevation_map.as_ref()?;

        let (row, column) = elevation_map.world_to_grid_indices(position);
        elevation_map.get_elevation(row, column)
    }
}
