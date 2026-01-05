//! Type definitions for the terrain system
//!
//! This module contains all the core types used throughout the terrain rendering system.

#![allow(dead_code)]

use serde::{Deserialize, Serialize};

/// Display side enumeration (Left/Right cockpit display)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum DisplaySide {
    #[serde(rename = "L")]
    Left,
    #[serde(rename = "R")]
    Right,
}

impl std::fmt::Display for DisplaySide {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            DisplaySide::Left => write!(f, "L"),
            DisplaySide::Right => write!(f, "R"),
        }
    }
}

impl std::str::FromStr for DisplaySide {
    type Err = String;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        match s.to_uppercase().as_str() {
            "L" | "LEFT" => Ok(DisplaySide::Left),
            "R" | "RIGHT" => Ok(DisplaySide::Right),
            _ => Err(format!("Invalid display side: {}", s)),
        }
    }
}

/// Position data (latitude/longitude)
#[derive(Debug, Clone, Copy, Default, Serialize, Deserialize)]
pub struct PositionData {
    pub latitude: f64,
    pub longitude: f64,
}

/// Terrain rendering mode
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum TerrainRenderingMode {
    ArcMode = 0,
    ScanlineMode = 1,
    VerticalDisplayRequired = 2,
}

impl From<u8> for TerrainRenderingMode {
    fn from(value: u8) -> Self {
        match value {
            0 => TerrainRenderingMode::ArcMode,
            1 => TerrainRenderingMode::ScanlineMode,
            _ => TerrainRenderingMode::VerticalDisplayRequired,
        }
    }
}

/// Terrain level mode for elevation display
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(u8)]
pub enum TerrainLevelMode {
    #[default]
    PeaksMode = 0,
    Warning = 1,
    Caution = 2,
}

/// EFIS (Electronic Flight Instrument System) data
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct EfisData {
    pub nd_range: u16,
    pub arc_mode: bool,
    pub terr_on_nd: bool,
    pub terr_on_vd: bool,
    pub efis_mode: u8,
    pub vd_range_lower: i32,
    pub vd_range_upper: i32,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub map_offset_x: Option<i32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub map_width: Option<u32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub map_height: Option<u32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub center_offset_y: Option<i32>,
}

/// Aircraft status data
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct AircraftStatus {
    pub adiru_data_valid: bool,
    pub taws_inop: bool,
    pub latitude: f64,
    pub longitude: f64,
    pub altitude: i32,
    pub heading: i16,
    pub vertical_speed: i16,
    pub gear_is_down: bool,
    pub runway_data_valid: bool,
    pub runway_latitude: f64,
    pub runway_longitude: f64,
    pub efis_data_capt: EfisData,
    pub efis_data_fo: EfisData,
    pub navigation_display_rendering_mode: u8,
    pub manual_azim_enabled: bool,
    pub manual_azim_degrees: u16,
    pub ground_truth_latitude: f64,
    pub ground_truth_longitude: f64,
}

/// Vertical path waypoint
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct Waypoint {
    pub latitude: f64,
    pub longitude: f64,
}

/// Vertical path data for flight path display
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct VerticalPathData {
    pub path_width: f64,
    pub track_changes_significantly_at_distance: f64,
    pub waypoints: Vec<Waypoint>,
}

/// Vertical display configuration
#[derive(Debug, Clone, Default)]
pub struct VerticalDisplay {
    pub range: f64,
    pub minimum_altitude: i32,
    pub maximum_altitude: i32,
    pub map_width: Option<u32>,
    pub map_height: Option<u32>,
}

/// Navigation display data for SimConnect transmission
#[derive(Debug, Clone, Default)]
pub struct NavigationDisplayData {
    pub minimum_elevation: i16,
    pub minimum_elevation_mode: TerrainLevelMode,
    pub maximum_elevation: i16,
    pub maximum_elevation_mode: TerrainLevelMode,
    pub first_frame: bool,
    pub display_range: f64,
    pub display_mode: u8,
    pub frame_byte_count: u32,
}

/// Navigation display thresholds DTO for web API
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct NavigationDisplayThresholdsDto {
    pub min_elevation: i32,
    pub min_elevation_is_warning: bool,
    pub min_elevation_is_caution: bool,
    pub max_elevation: i32,
    pub max_elevation_is_warning: bool,
    pub max_elevation_is_caution: bool,
}

/// TAWS EFIS data DTO for web API
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct TawsEfisDataDto {
    pub nd_range: u16,
    pub arc_mode: bool,
    pub terr_on_nd: bool,
    pub terr_on_vd: bool,
    pub efis_mode: u8,
    pub vd_range_lower: i32,
    pub vd_range_upper: i32,
}

/// TAWS aircraft status data DTO for web API
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct TawsAircraftStatusDataDto {
    pub adiru_data_valid: bool,
    pub taws_inop: bool,
    pub latitude: f64,
    pub longitude: f64,
    pub altitude: f64,
    pub heading: f64,
    pub vertical_speed: f64,
    pub gear_is_down: bool,
    pub runway_data_valid: bool,
    pub runway_latitude: f64,
    pub runway_longitude: f64,
    pub efis_data_capt: TawsEfisDataDto,
    pub efis_data_fo: TawsEfisDataDto,
    pub navigation_display_rendering_mode: u8,
    pub manual_azim_enabled: bool,
    pub manual_azim_degrees: u16,
    pub ground_truth_latitude: f64,
    pub ground_truth_longitude: f64,
}

impl From<TawsAircraftStatusDataDto> for AircraftStatus {
    fn from(dto: TawsAircraftStatusDataDto) -> Self {
        AircraftStatus {
            adiru_data_valid: dto.adiru_data_valid,
            taws_inop: dto.taws_inop,
            latitude: dto.latitude,
            longitude: dto.longitude,
            altitude: dto.altitude as i32,
            heading: dto.heading as i16,
            vertical_speed: dto.vertical_speed as i16,
            gear_is_down: dto.gear_is_down,
            runway_data_valid: dto.runway_data_valid,
            runway_latitude: dto.runway_latitude,
            runway_longitude: dto.runway_longitude,
            efis_data_capt: EfisData {
                nd_range: dto.efis_data_capt.nd_range,
                arc_mode: dto.efis_data_capt.arc_mode,
                terr_on_nd: dto.efis_data_capt.terr_on_nd,
                terr_on_vd: dto.efis_data_capt.terr_on_vd,
                efis_mode: dto.efis_data_capt.efis_mode,
                vd_range_lower: dto.efis_data_capt.vd_range_lower,
                vd_range_upper: dto.efis_data_capt.vd_range_upper,
                ..Default::default()
            },
            efis_data_fo: EfisData {
                nd_range: dto.efis_data_fo.nd_range,
                arc_mode: dto.efis_data_fo.arc_mode,
                terr_on_nd: dto.efis_data_fo.terr_on_nd,
                terr_on_vd: dto.efis_data_fo.terr_on_vd,
                efis_mode: dto.efis_data_fo.efis_mode,
                vd_range_lower: dto.efis_data_fo.vd_range_lower,
                vd_range_upper: dto.efis_data_fo.vd_range_upper,
                ..Default::default()
            },
            navigation_display_rendering_mode: dto.navigation_display_rendering_mode,
            manual_azim_enabled: dto.manual_azim_enabled,
            manual_azim_degrees: dto.manual_azim_degrees,
            ground_truth_latitude: dto.ground_truth_latitude,
            ground_truth_longitude: dto.ground_truth_longitude,
        }
    }
}

/// Waypoint DTO for web API
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct WaypointDto {
    pub latitude: f64,
    pub longitude: f64,
}

/// Elevation sample path DTO for web API
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct ElevationSamplePathDto {
    pub path_width: f64,
    pub track_changes_significantly_at_distance: f64,
    pub waypoints: Vec<WaypointDto>,
}

impl From<ElevationSamplePathDto> for VerticalPathData {
    fn from(dto: ElevationSamplePathDto) -> Self {
        VerticalPathData {
            path_width: dto.path_width,
            track_changes_significantly_at_distance: dto.track_changes_significantly_at_distance,
            waypoints: dto.waypoints.into_iter().map(|w| Waypoint {
                latitude: w.latitude,
                longitude: w.longitude,
            }).collect(),
        }
    }
}

/// Elevation profile for terrain sampling
#[derive(Debug, Clone, Default)]
pub struct ElevationProfile {
    pub path_width: f64,
    pub waypoints_latitudes: Vec<f64>,
    pub waypoints_longitudes: Vec<f64>,
    pub range: f64,
    pub track_changes_significantly_at_distance: f64,
    pub fms_path_used: bool,
}

/// Grid definition for terrain tiles
#[derive(Debug, Clone, Default)]
pub struct GridDefinition {
    pub rows: usize,
    pub columns: usize,
    pub latitude_step: f64,
    pub longitude_step: f64,
}

/// Grid lookup data for finding relevant tiles
#[derive(Debug, Clone, Default)]
pub struct GridLookupData {
    pub southwest: PositionData,
    pub northeast: PositionData,
    pub grid: Vec<Vec<GridIndex>>,
    pub min_width_per_tile: usize,
    pub min_height_per_tile: usize,
}

/// Index into the tile grid
#[derive(Debug, Clone, Copy, Default)]
pub struct GridIndex {
    pub row: usize,
    pub column: usize,
}

/// Elevation grid containing actual elevation data
#[derive(Debug, Clone)]
pub struct ElevationGrid {
    pub southwest: PositionData,
    pub northeast: PositionData,
    pub rows: usize,
    pub columns: usize,
    pub elevation_map: Vec<i16>,
}

impl ElevationGrid {
    pub fn new(southwest: PositionData, northeast: PositionData, rows: usize, columns: usize) -> Self {
        ElevationGrid {
            southwest,
            northeast,
            rows,
            columns,
            elevation_map: vec![0; rows * columns],
        }
    }

    /// Convert world coordinates to grid indices
    pub fn world_to_grid_indices(&self, coordinate: PositionData) -> (usize, usize) {
        let lat_range = self.northeast.latitude - self.southwest.latitude;
        let lat_delta = coordinate.latitude - self.southwest.latitude;
        let row = (self.rows as f64 - (lat_delta / lat_range * self.rows as f64).floor())
            .min(self.rows as f64) as usize - 1;

        let lon_range = self.northeast.longitude - self.southwest.longitude;
        let lon_delta = coordinate.longitude - self.southwest.longitude;
        let column = ((lon_delta / lon_range * self.columns as f64).floor() as usize)
            .min(self.columns - 1);

        (row, column)
    }

    /// Get elevation at grid position
    pub fn get_elevation(&self, row: usize, column: usize) -> Option<i16> {
        if row < self.rows && column < self.columns {
            Some(self.elevation_map[row * self.columns + column])
        } else {
            None
        }
    }
}

// Constants
pub mod constants {
    // Execution parameters
    pub const GPU_PROCESSING_ACTIVE: bool = false; // CPU only in Rust version

    // Mathematical conversion constants
    pub const FEET_PER_NAUTICAL_MILE: f64 = 6076.12;
    pub const THREE_NAUTICAL_MILES_IN_FEET: f64 = 18228.3;
    pub const NAUTICAL_MILES_TO_METRES: f64 = 1852.0;
    pub const RENDERING_COLOR_CHANNEL_COUNT: usize = 4;

    // Map grid creation
    pub const INVALID_ELEVATION: i16 = 32767;
    pub const UNKNOWN_ELEVATION: i16 = 32766;
    pub const WATER_ELEVATION: i16 = -1;
    pub const DEFAULT_TILE_SIZE: usize = 300;

    // Navigation display parameters
    pub const NAVIGATION_DISPLAY_MAP_START_OFFSET_Y: usize = 128;
    pub const NAVIGATION_DISPLAY_MAX_PIXEL_WIDTH: usize = 768;
    pub const NAVIGATION_DISPLAY_ARC_MODE_PIXEL_HEIGHT_A32NX: usize = 492;
    pub const NAVIGATION_DISPLAY_ROSE_MODE_PIXEL_HEIGHT_A32NX: usize = 250;
    pub const NAVIGATION_DISPLAY_ARC_MODE_PIXEL_HEIGHT_A380X: usize = 592;
    pub const NAVIGATION_DISPLAY_ROSE_MODE_PIXEL_HEIGHT_A380X: usize = 592;

    pub const NAVIGATION_DISPLAY_MAX_PIXEL_HEIGHT: usize = 592; // Max of all heights

    pub const NAVIGATION_DISPLAY_CENTER_OFFSET_Y_A32NX: i32 = 0;
    pub const NAVIGATION_DISPLAY_ARC_MODE_CENTER_OFFSET_Y_A380X: i32 = 100;
    pub const NAVIGATION_DISPLAY_ROSE_MODE_CENTER_OFFSET_Y_A380X: i32 = 342;

    // Vertical display parameters
    pub const VERTICAL_DISPLAY_MAP_START_OFFSET_Y: usize = 800;
    pub const VERTICAL_DISPLAY_MAP_START_OFFSET_X: usize = 150;

    // Rendering parameters
    pub const RENDERING_MAP_TRANSITION_DELTA_TIME: u64 = 40;
    pub const RENDERING_MAP_TRANSITION_DURATION_ARC_MODE: u64 = 1500;
    pub const RENDERING_MAP_UPDATE_TIMEOUT_ARC_MODE: u64 = 1000;
    pub const RENDERING_MAP_TRANSITION_DURATION_SCANLINE_MODE: u64 = 600;
    pub const RENDERING_MAP_UPDATE_TIMEOUT_SCANLINE_MODE: u64 = 500;

    // Display dimensions
    pub const DISPLAY_SCREEN_PIXEL_HEIGHT_WITHOUT_VERTICAL_DISPLAY: usize = 768;
    pub const DISPLAY_SCREEN_PIXEL_HEIGHT_WITH_VERTICAL_DISPLAY: usize = 1024;

    // Histogram parameters
    pub const HISTOGRAM_BIN_RANGE: i32 = 100;
    pub const HISTOGRAM_MINIMUM_ELEVATION: i32 = -500;
    pub const HISTOGRAM_MAXIMUM_ELEVATION: i32 = 29040;

    // Rendering parameters
    pub const RENDERING_ARC_MODE_PIXEL_WIDTH: usize = 756;
    pub const RENDERING_ROSE_MODE_PIXEL_WIDTH: usize = 678;
    pub const RENDERING_CUT_OFF_ALTITUDE_MINIMUM: i32 = 200;
    pub const RENDERING_CUT_OFF_ALTITUDE_MAXIMUM: i32 = 400;
    pub const RENDERING_LOWER_PERCENTILE: f64 = 0.85;
    pub const RENDERING_UPPER_PERCENTILE: f64 = 0.95;
    pub const RENDERING_FLAT_EARTH_THRESHOLD: i32 = 100;
    pub const RENDERING_MAX_AIRPORT_DISTANCE: f64 = 4.0;

    // Vertical display rendering
    pub const RENDERING_ELEVATION_PROFILE_WIDTH: usize = 540;
    pub const RENDERING_ELEVATION_PROFILE_HEIGHT: usize = 200;

    // GPU Max Pixel Size (for limiting cached area)
    pub const GPU_MAX_PIXEL_SIZE: usize = 16384;
}
