//! Terrain file format parsing
//!
//! This module handles parsing the terrain.map file format which contains
//! compressed elevation data organized into tiles.

mod terrainmap;
mod tile;

pub use terrainmap::TerrainMap;
pub use tile::Tile;
