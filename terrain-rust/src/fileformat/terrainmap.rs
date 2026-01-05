//! Terrain map file format parser
//!
//! The terrain.map file contains a header followed by compressed tile data.

#![allow(dead_code)]

use std::fs::File;
use std::io::{BufReader, Read};
use std::path::Path;
use anyhow::{Context, Result};
use log::info;

use super::Tile;

/// Terrain map containing all elevation tiles
pub struct TerrainMap {
    /// Latitude range (min, max)
    pub latitude_range: (i16, i16),
    /// Longitude range (min, max)
    pub longitude_range: (i16, i16),
    /// Angular steps (latitude step, longitude step)
    pub angular_steps: (u8, u8),
    /// Horizontal resolution in meters
    pub horizontal_resolution: f32,
    /// All tiles in the terrain map
    pub tiles: Vec<Tile>,
}

impl TerrainMap {
    /// Load a terrain map from a file path
    pub fn from_file(path: &Path) -> Result<Self> {
        let file = File::open(path)
            .with_context(|| format!("Failed to open terrain map file: {:?}", path))?;

        let mut reader = BufReader::new(file);
        let mut buffer = Vec::new();
        reader.read_to_end(&mut buffer)
            .with_context(|| "Failed to read terrain map file")?;

        Self::from_buffer(&buffer)
    }

    /// Parse a terrain map from a buffer
    pub fn from_buffer(buffer: &[u8]) -> Result<Self> {
        if buffer.len() < 14 {
            anyhow::bail!("Terrain map buffer too small for header");
        }

        // Extract the file header
        let latitude_min = i16::from_le_bytes([buffer[0], buffer[1]]);
        let latitude_max = i16::from_le_bytes([buffer[2], buffer[3]]);
        let longitude_min = i16::from_le_bytes([buffer[4], buffer[5]]);
        let longitude_max = i16::from_le_bytes([buffer[6], buffer[7]]);
        let latitude_step = buffer[8];
        let longitude_step = buffer[9];

        // Horizontal resolution is stored in nautical miles, convert to meters
        let horizontal_resolution_nm = f32::from_le_bytes([
            buffer[10], buffer[11], buffer[12], buffer[13]
        ]);
        let horizontal_resolution = horizontal_resolution_nm * 1852.0;

        info!(
            "Terrain map: lat [{}, {}], lon [{}, {}], steps [{}, {}], resolution {} m",
            latitude_min, latitude_max, longitude_min, longitude_max,
            latitude_step, longitude_step, horizontal_resolution
        );

        // Parse tiles
        let mut tiles = Vec::new();
        let mut offset = 14usize;

        while offset < buffer.len() {
            let tile = Tile::from_buffer(buffer, offset, latitude_step, longitude_step)?;
            offset = tile.buffer_offset + tile.buffer_size;
            tiles.push(tile);
        }

        info!("Loaded {} terrain tiles", tiles.len());

        Ok(TerrainMap {
            latitude_range: (latitude_min, latitude_max),
            longitude_range: (longitude_min, longitude_max),
            angular_steps: (latitude_step, longitude_step),
            horizontal_resolution,
            tiles,
        })
    }

    /// Get the latitude step size
    pub fn latitude_step(&self) -> f64 {
        self.angular_steps.0 as f64
    }

    /// Get the longitude step size
    pub fn longitude_step(&self) -> f64 {
        self.angular_steps.1 as f64
    }
}
