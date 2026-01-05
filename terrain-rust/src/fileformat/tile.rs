//! Terrain tile format
//!
//! Each tile contains compressed elevation data for a geographic region.

use anyhow::{Context, Result};
use flate2::read::GzDecoder;
use std::io::Read;

use crate::types::{ElevationGrid, PositionData};

/// A single terrain tile containing elevation data for a geographic region
pub struct Tile {
    /// Southwest corner coordinates
    pub southwest: PositionData,
    /// Grid dimensions (rows, columns)
    pub grid_dimension: (u16, u16),
    /// Offset in the original buffer where tile data starts
    pub buffer_offset: usize,
    /// Size of the compressed data in bytes
    pub buffer_size: usize,
    /// Parent angular steps
    angular_steps: (u8, u8),
    /// Compressed elevation data
    compressed_data: Vec<u8>,
}

impl Tile {
    /// Parse a tile from the buffer at the given offset
    pub fn from_buffer(
        buffer: &[u8],
        offset: usize,
        latitude_step: u8,
        longitude_step: u8,
    ) -> Result<Self> {
        if offset + 11 > buffer.len() {
            anyhow::bail!("Buffer too small for tile header at offset {}", offset);
        }

        // Extract the tile header
        let rows = u16::from_le_bytes([buffer[offset], buffer[offset + 1]]);
        let columns = u16::from_le_bytes([buffer[offset + 2], buffer[offset + 3]]);
        let southwest_lat = buffer[offset + 4] as i8;
        let southwest_lon = i16::from_le_bytes([buffer[offset + 5], buffer[offset + 6]]);
        let buffer_size = u32::from_le_bytes([
            buffer[offset + 7],
            buffer[offset + 8],
            buffer[offset + 9],
            buffer[offset + 10],
        ]) as usize;

        let data_offset = offset + 11;

        if data_offset + buffer_size > buffer.len() {
            anyhow::bail!(
                "Buffer too small for tile data: need {} bytes at offset {}, have {}",
                buffer_size, data_offset, buffer.len() - data_offset
            );
        }

        let compressed_data = buffer[data_offset..data_offset + buffer_size].to_vec();

        Ok(Tile {
            southwest: PositionData {
                latitude: southwest_lat as f64,
                longitude: southwest_lon as f64,
            },
            grid_dimension: (rows, columns),
            buffer_offset: data_offset,
            buffer_size,
            angular_steps: (latitude_step, longitude_step),
            compressed_data,
        })
    }

    /// Load and decompress the elevation grid for this tile
    pub fn load_elevation_grid(&self) -> Result<ElevationGrid> {
        let northeast = PositionData {
            latitude: self.southwest.latitude + self.angular_steps.0 as f64,
            longitude: self.southwest.longitude + self.angular_steps.1 as f64,
        };

        // Keep dimensions as-is (no swap) - rows = latitude count, columns = longitude count
        let tile_rows = self.grid_dimension.0 as usize;
        let tile_columns = self.grid_dimension.1 as usize;
        let mut grid = ElevationGrid::new(self.southwest, northeast, tile_rows, tile_columns);

        // Decompress the data
        let mut decoder = GzDecoder::new(&self.compressed_data[..]);
        let mut decompressed = Vec::new();
        decoder.read_to_end(&mut decompressed)
            .with_context(|| "Failed to decompress tile data")?;

        // Parse elevation values - use tile dimensions for the loop
        let expected_size = tile_rows * tile_columns * 2; // 2 bytes per elevation value
        if decompressed.len() < expected_size {
            anyhow::bail!(
                "Decompressed data too small: expected {} bytes, got {}",
                expected_size, decompressed.len()
            );
        }

        let mut offset = 0;
        // Read data row-by-row (latitude major) - standard row-major order
        for row in 0..tile_rows {
            for col in 0..tile_columns {
                let elevation_meters = i16::from_le_bytes([
                    decompressed[offset],
                    decompressed[offset + 1],
                ]);

                // Convert to feet if not water (-1)
                let elevation_feet = if elevation_meters != -1 {
                    (elevation_meters as f64 * 3.28084).round() as i16
                } else {
                    elevation_meters
                };

                // Store in row-major order: index = row * columns + col
                grid.elevation_map[row * tile_columns + col] = elevation_feet;
                offset += 2;
            }
        }

        Ok(grid)
    }

    /// Get the number of rows in this tile
    pub fn rows(&self) -> usize {
        self.grid_dimension.0 as usize
    }

    /// Get the number of columns in this tile
    pub fn columns(&self) -> usize {
        self.grid_dimension.1 as usize
    }
}
