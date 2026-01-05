//! Tile manager for caching and managing terrain tiles

#![allow(dead_code)]

use crate::types::{ElevationGrid, GridIndex, PositionData};
use crate::fileformat::TerrainMap;

/// Represents a cell in the tile grid
pub struct TileCell {
    /// Southwest corner of this cell
    pub southwest: PositionData,
    /// Index into TerrainMap.tiles, or -1 if no tile data exists
    pub tile_index: i32,
    /// Cached elevation map for this cell
    pub elevation_map: Option<ElevationGrid>,
}

/// Manages the grid of terrain tiles
pub struct TileManager {
    /// 2D grid of tile cells indexed by [row][column]
    pub grid: Vec<Vec<TileCell>>,
}

impl TileManager {
    /// Create a new tile manager from terrain map data
    pub fn new(terrain_data: &TerrainMap) -> Self {
        let lat_step = terrain_data.angular_steps.0 as i32;
        let lon_step = terrain_data.angular_steps.1 as i32;

        let mut grid = Vec::new();

        let mut lat = -90i32;
        while lat < 90 {
            let mut row = Vec::new();

            let mut lon = -180i32;
            while lon < 180 {
                // Find tile index for this cell
                let tile_index = Self::find_tile_index(&terrain_data.tiles, lat, lon);

                row.push(TileCell {
                    southwest: PositionData {
                        latitude: lat as f64,
                        longitude: lon as f64,
                    },
                    tile_index,
                    elevation_map: None,
                });

                lon += lon_step;
            }

            grid.push(row);
            lat += lat_step;
        }

        TileManager { grid }
    }

    /// Find the tile index for a given latitude/longitude
    fn find_tile_index(tiles: &[crate::fileformat::Tile], latitude: i32, longitude: i32) -> i32 {
        for (idx, tile) in tiles.iter().enumerate() {
            if tile.southwest.latitude as i32 == latitude
                && tile.southwest.longitude as i32 == longitude
            {
                return idx as i32;
            }
        }
        -1
    }

    /// Set the elevation map for a cell
    pub fn set_elevation_map(&mut self, index: GridIndex, map: ElevationGrid) {
        if index.row < self.grid.len() && index.column < self.grid[index.row].len() {
            self.grid[index.row][index.column].elevation_map = Some(map);
        }
    }

    /// Get the number of rows in the grid
    pub fn rows(&self) -> usize {
        self.grid.len()
    }

    /// Get the number of columns in the grid (assumes uniform)
    pub fn columns(&self) -> usize {
        self.grid.first().map(|r| r.len()).unwrap_or(0)
    }

    /// Clear all cached elevation maps
    pub fn clear_elevation_cache(&mut self) {
        for row in &mut self.grid {
            for cell in row {
                cell.elevation_map = None;
            }
        }
    }

    /// Clean up elevation cache, keeping only tiles in the relevant set
    pub fn cleanup_elevation_cache(&mut self, relevant_tiles: &[Vec<GridIndex>]) {
        let relevant_set: std::collections::HashSet<(usize, usize)> = relevant_tiles
            .iter()
            .flat_map(|row| row.iter().map(|idx| (idx.row, idx.column)))
            .collect();

        for (row_idx, row) in self.grid.iter_mut().enumerate() {
            for (col_idx, cell) in row.iter_mut().enumerate() {
                if !relevant_set.contains(&(row_idx, col_idx)) {
                    cell.elevation_map = None;
                }
            }
        }
    }
}
