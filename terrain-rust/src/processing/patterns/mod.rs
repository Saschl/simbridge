//! Pattern generation for terrain display rendering
//!
//! Uses pre-computed pattern maps from TypeScript:
//! - Arc mode pattern (768x492) for A32NX
//! - Scanline mode pattern (768x592) for A380X

#![allow(dead_code)]

mod pattern_data;
mod scanline_pattern_data;

pub use pattern_data::{ARC_MODE_PATTERN_MAP, ARC_MODE_PATTERN_WIDTH, ARC_MODE_PATTERN_HEIGHT};
pub use scanline_pattern_data::{SCANLINE_MODE_PATTERN_MAP, SCANLINE_MODE_PATTERN_WIDTH, SCANLINE_MODE_PATTERN_HEIGHT};

/// Get pattern value at (x, y) from the arc mode pattern map (A32NX)
/// Returns 0 if out of bounds (transparent)
#[inline]
pub fn get_arc_mode_pattern_value(x: usize, y: usize) -> u8 {
    if y >= ARC_MODE_PATTERN_HEIGHT || x >= ARC_MODE_PATTERN_WIDTH {
        return 0;
    }
    let idx = y * ARC_MODE_PATTERN_WIDTH + x;
    if idx < ARC_MODE_PATTERN_MAP.len() {
        ARC_MODE_PATTERN_MAP[idx]
    } else {
        0
    }
}

/// Get pattern value at (x, y) from the scanline mode pattern map (A380X)
/// Returns 0 if out of bounds (transparent)
#[inline]
pub fn get_scanline_mode_pattern_value(x: usize, y: usize) -> u8 {
    if y >= SCANLINE_MODE_PATTERN_HEIGHT || x >= SCANLINE_MODE_PATTERN_WIDTH {
        return 0;
    }
    let idx = y * SCANLINE_MODE_PATTERN_WIDTH + x;
    if idx < SCANLINE_MODE_PATTERN_MAP.len() {
        SCANLINE_MODE_PATTERN_MAP[idx]
    } else {
        0
    }
}

/// Get pattern value based on rendering mode
/// - use_scanline_mode = true: A380X (592 height pattern)
/// - use_scanline_mode = false: A32NX (492 height pattern)
#[inline]
pub fn get_pattern_value(x: usize, y: usize, use_scanline_mode: bool) -> u8 {
    if use_scanline_mode {
        get_scanline_mode_pattern_value(x, y)
    } else {
        get_arc_mode_pattern_value(x, y)
    }
}

/// Check if a pixel should be drawn based on pattern value and pattern index
/// This is the exact same logic as TypeScript's drawDensityPixel:
/// `if (Math.round(patternValue % patternIndex) === 0)` -> draw, else transparent
#[inline]
pub fn draw_density_pixel(pattern_value: u8, pattern_index: u8) -> bool {
    if pattern_value == 0 {
        return false; // Always transparent when pattern value is 0
    }
    if pattern_index == 0 {
        return false; // Avoid division by zero
    }
    // TypeScript: Math.round(patternValue % patternIndex) === 0
    // Since we're dealing with integers, Math.round is not needed
    (pattern_value % pattern_index) == 0
}

/// Get the pattern index for a terrain color category
/// Based on TypeScript renderNormalMode and renderPeaksMode
#[inline]
pub fn get_pattern_index_for_category(category: TerrainColorCategory) -> u8 {
    match category {
        TerrainColorCategory::RedSolid => 5,      // solid high danger - high density
        TerrainColorCategory::RedHighDensity => 5, // high density red
        TerrainColorCategory::YellowSolid => 5,   // solid yellow - high density
        TerrainColorCategory::YellowHighDensity => 5, // high density yellow
        TerrainColorCategory::YellowLowDensity => 3,  // low density yellow
        TerrainColorCategory::GreenHighDensity => 5,  // high density green
        TerrainColorCategory::GreenLowDensity => 3,   // low density green
        TerrainColorCategory::Water => 7,         // water uses pattern 7
        TerrainColorCategory::Unknown => 5,       // unknown terrain
        TerrainColorCategory::Transparent => 0,   // no pattern - always transparent
    }
}

/// Terrain color categories for pattern rendering
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum TerrainColorCategory {
    RedSolid,
    RedHighDensity,
    YellowSolid,
    YellowHighDensity,
    YellowLowDensity,
    GreenHighDensity,
    GreenLowDensity,
    Water,
    Unknown,
    Transparent,
}
