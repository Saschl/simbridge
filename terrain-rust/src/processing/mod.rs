//! Terrain processing module
//!
//! This module handles terrain rendering for both navigation and vertical displays.

#![allow(dead_code)]
#![allow(unused_imports)]
#![allow(unused_variables)]

mod renderer;
mod patterns;

use patterns::{get_pattern_value, draw_density_pixel, ARC_MODE_PATTERN_WIDTH, ARC_MODE_PATTERN_HEIGHT, SCANLINE_MODE_PATTERN_WIDTH, SCANLINE_MODE_PATTERN_HEIGHT};

use std::path::Path;
use std::time::Instant;
use std::collections::HashMap;
use anyhow::Result;
use log::{info, error, debug, warn};

use crate::types::{
    constants::*, AircraftStatus, DisplaySide,
    NavigationDisplayThresholdsDto, PositionData,
    TerrainLevelMode, TerrainRenderingMode, VerticalPathData,
};
use crate::fileformat::TerrainMap;
use crate::mapdata::Worldmap;

pub use renderer::{NavigationDisplayRenderer, VerticalDisplayRenderer};

/// Frame data for a display cycle
#[derive(Debug, Clone)]
pub struct FrameCycleData {
    pub timestamp: u64,
    pub thresholds: Option<NavigationDisplayThresholdsDto>,
    pub frames: Vec<Vec<u8>>,
}

impl Default for FrameCycleData {
    fn default() -> Self {
        FrameCycleData {
            timestamp: 0,
            thresholds: None,
            frames: Vec::new(),
        }
    }
}

/// Display rendering state
pub struct DisplayRenderingState {
    pub navigation_display: NavigationDisplayRenderer,
    pub vertical_display: VerticalDisplayRenderer,
    pub cycle_data: FrameCycleData,
    pub last_render: Instant,
}

/// Main terrain processor handling all terrain rendering
pub struct TerrainProcessor {
    /// Flag indicating if processor is initialized
    initialized: bool,
    /// World map with terrain data
    worldmap: Option<Worldmap>,
    /// Current aircraft status
    aircraft_status: Option<AircraftStatus>,
    /// Current position
    current_position: Option<PositionData>,
    /// Display rendering states for left and right displays
    display_rendering: HashMap<DisplaySide, DisplayRenderingState>,
    /// Rendering mode
    rendering_mode: TerrainRenderingMode,
    /// Whether vertical display is required
    vertical_display_required: bool,
    /// Manual azimuth settings
    manual_azim_enabled: bool,
    manual_azim_degrees: f64,
    /// Track changes distance for each side
    track_changes_distance: HashMap<DisplaySide, f64>,
    /// Startup time
    startup_time: Instant,
    /// Cached elevation data
    cached_elevation_data: Option<CachedElevationData>,
    /// World map metadata
    world_map_metadata: WorldMapMetadata,
}

#[derive(Default)]
struct CachedElevationData {
    data: Vec<f32>,
    cached_tiles: usize,
}

#[derive(Default)]
struct WorldMapMetadata {
    southwest: PositionData,
    northeast: PositionData,
    current_grid_position: (usize, usize),
    min_width_per_tile: usize,
    min_height_per_tile: usize,
    width: usize,
    height: usize,
}

/// Pre-calculated thresholds for terrain rendering (calculated once per frame)
/// These values are used for BOTH coloring and metadata output
#[derive(Debug, Clone, Copy)]
struct RenderingThresholds {
    /// Low density green threshold (lowest rendered terrain in normal mode)
    low_density_green: i32,
    /// High density green threshold
    high_density_green: i32,
    /// Low density yellow threshold
    low_density_yellow: i32,
    /// High density yellow threshold
    high_density_yellow: i32,
    /// High density red threshold
    high_density_red: i32,
    /// Cutoff altitude (terrain below this is not rendered)
    cutoff_altitude: i32,
    /// Reference altitude (aircraft altitude with vertical speed prediction)
    reference_altitude: i32,
    /// Whether we're in normal mode (vs peaks mode)
    use_normal_mode: bool,
    /// Raw min elevation from terrain data
    min_elevation: i32,
    /// Raw max elevation from terrain data
    max_elevation: i32,
}

impl TerrainProcessor {
    /// Create a new terrain processor from a terrain map file
    pub fn new(terrain_path: &Path) -> Result<Self> {
        let terrain_map = TerrainMap::from_file(terrain_path)?;
        info!("Loaded terrain map with {} tiles", terrain_map.tiles.len());

        let worldmap = Worldmap::new(terrain_map);
        let startup_time = Instant::now();

        let mut display_rendering = HashMap::new();

        // Initialize left display
        display_rendering.insert(DisplaySide::Left, DisplayRenderingState {
            navigation_display: NavigationDisplayRenderer::new(startup_time),
            vertical_display: VerticalDisplayRenderer::new(startup_time),
            cycle_data: FrameCycleData::default(),
            last_render: startup_time,
        });

        // Initialize right display (offset by 1.5 seconds for more realistic behavior)
        let right_startup = startup_time; // In real implementation, offset this
        display_rendering.insert(DisplaySide::Right, DisplayRenderingState {
            navigation_display: NavigationDisplayRenderer::new(right_startup),
            vertical_display: VerticalDisplayRenderer::new(right_startup),
            cycle_data: FrameCycleData::default(),
            last_render: right_startup,
        });

        let mut track_changes_distance = HashMap::new();
        track_changes_distance.insert(DisplaySide::Left, -1.0);
        track_changes_distance.insert(DisplaySide::Right, -1.0);

        Ok(TerrainProcessor {
            initialized: true,
            worldmap: Some(worldmap),
            aircraft_status: None,
            current_position: None,
            display_rendering,
            rendering_mode: TerrainRenderingMode::ArcMode,
            vertical_display_required: false,
            manual_azim_enabled: false,
            manual_azim_degrees: 0.0,
            track_changes_distance,
            startup_time,
            cached_elevation_data: None,
            world_map_metadata: WorldMapMetadata::default(),
        })
    }

    /// Create an empty terrain processor (when terrain map is not available)
    pub fn empty() -> Self {
        let startup_time = Instant::now();

        let mut display_rendering = HashMap::new();
        display_rendering.insert(DisplaySide::Left, DisplayRenderingState {
            navigation_display: NavigationDisplayRenderer::new(startup_time),
            vertical_display: VerticalDisplayRenderer::new(startup_time),
            cycle_data: FrameCycleData::default(),
            last_render: startup_time,
        });
        display_rendering.insert(DisplaySide::Right, DisplayRenderingState {
            navigation_display: NavigationDisplayRenderer::new(startup_time),
            vertical_display: VerticalDisplayRenderer::new(startup_time),
            cycle_data: FrameCycleData::default(),
            last_render: startup_time,
        });

        let mut track_changes_distance = HashMap::new();
        track_changes_distance.insert(DisplaySide::Left, -1.0);
        track_changes_distance.insert(DisplaySide::Right, -1.0);

        TerrainProcessor {
            initialized: false,
            worldmap: None,
            aircraft_status: None,
            current_position: None,
            display_rendering,
            rendering_mode: TerrainRenderingMode::ArcMode,
            vertical_display_required: false,
            manual_azim_enabled: false,
            manual_azim_degrees: 0.0,
            track_changes_distance,
            startup_time,
            cached_elevation_data: None,
            world_map_metadata: WorldMapMetadata::default(),
        }
    }

    /// Reset the terrain processor
    pub fn reset(&mut self) {
        if !self.initialized { return; }

        if let Some(ref mut worldmap) = self.worldmap {
            worldmap.reset_internal_data();
        }

        for (_, state) in &mut self.display_rendering {
            state.navigation_display.reset();
            state.vertical_display.reset(true);
            state.cycle_data = FrameCycleData::default();
        }

        self.cached_elevation_data = None;
        self.world_map_metadata = WorldMapMetadata::default();
    }

    /// Update position
    pub fn position_update(&mut self, position: PositionData) {
        if !self.initialized { return; }

        self.current_position = Some(position);
        self.update_cached_tiles(position);
    }

    /// Update aircraft status
    pub fn aircraft_status_update(&mut self, status: AircraftStatus) {
        if !self.initialized {
            warn!("aircraft_status_update called but processor not initialized");
            return;
        }

        // Update rendering mode
        self.vertical_display_required =
            (status.navigation_display_rendering_mode & TerrainRenderingMode::VerticalDisplayRequired as u8) != 0;

        self.rendering_mode = if (status.navigation_display_rendering_mode & TerrainRenderingMode::ScanlineMode as u8) != 0 {
            TerrainRenderingMode::ScanlineMode
        } else {
            TerrainRenderingMode::ArcMode
        };

        self.manual_azim_enabled = status.manual_azim_enabled;
        self.manual_azim_degrees = status.manual_azim_degrees as f64;

        // Update position
        let position = PositionData {
            latitude: status.latitude,
            longitude: status.longitude,
        };
        self.position_update(position);

        // Update display renderers
        self.update_rendering(DisplaySide::Left, &status);
        self.update_rendering(DisplaySide::Right, &status);

        self.aircraft_status = Some(status);
    }

    /// Update vertical path data
    pub fn vertical_path_update(&mut self, path: VerticalPathData) {
        if !self.initialized { return; }

        self.update_path_data(DisplaySide::Left, &path);
        self.update_path_data(DisplaySide::Right, &path);
    }

    /// Get frame data for a display side
    pub fn get_frame_data(&self, side: DisplaySide) -> FrameCycleData {
        self.display_rendering
            .get(&side)
            .map(|state| state.cycle_data.clone())
            .unwrap_or_default()
    }

    /// Render and get current frames
    pub fn render_frames(&mut self, side: DisplaySide) -> Option<FrameCycleData> {
        if !self.initialized { return None; }

        // First, check if we need to render and get necessary data
        let (should_render, terr_enabled) = {
            let state = self.display_rendering.get(&side)?;
            let elapsed = state.last_render.elapsed();
            let timeout = if self.rendering_mode == TerrainRenderingMode::ArcMode {
                RENDERING_MAP_UPDATE_TIMEOUT_ARC_MODE
            } else {
                RENDERING_MAP_UPDATE_TIMEOUT_SCANLINE_MODE
            };

            if elapsed.as_millis() < timeout as u128 {
                return Some(state.cycle_data.clone());
            }

            let nd_config = state.navigation_display.display_configuration();
            (true, nd_config.terr_on_nd || nd_config.terr_on_vd)
        };

        if !should_render {
            return self.display_rendering.get(&side).map(|s| s.cycle_data.clone());
        }

        // Render the frame (this doesn't borrow display_rendering)
        let frame = if terr_enabled {
            self.render_navigation_display_frame(side)
        } else {
            None
        };

        // Now update the state with the rendered frame
        let state = self.display_rendering.get_mut(&side)?;
        state.last_render = Instant::now();

        if let Some(frame_data) = frame {
            state.cycle_data.frames = vec![frame_data];
            state.cycle_data.timestamp = state.last_render
                .duration_since(self.startup_time)
                .as_millis() as u64;

            let display_data = state.navigation_display.display_data();
            state.cycle_data.thresholds = Some(NavigationDisplayThresholdsDto {
                min_elevation: display_data.minimum_elevation as i32,
                min_elevation_is_warning: display_data.minimum_elevation_mode == TerrainLevelMode::Warning,
                min_elevation_is_caution: display_data.minimum_elevation_mode == TerrainLevelMode::Caution,
                max_elevation: display_data.maximum_elevation as i32,
                max_elevation_is_warning: display_data.maximum_elevation_mode == TerrainLevelMode::Warning,
                max_elevation_is_caution: display_data.maximum_elevation_mode == TerrainLevelMode::Caution,
            });
        }

        Some(state.cycle_data.clone())
    }

    // Private helper methods

    fn update_rendering(&mut self, side: DisplaySide, status: &AircraftStatus) {
        let config = if side == DisplaySide::Left {
            &status.efis_data_capt
        } else {
            &status.efis_data_fo
        };

        // Check if this is startup (first update)
        let startup = self.aircraft_status.is_none();

        if let Some(state) = self.display_rendering.get_mut(&side) {
            state.navigation_display.aircraft_status_update(status.clone(), side, startup);
            state.vertical_display.aircraft_status_update(status.clone(), side);
        }
    }

    fn update_path_data(&mut self, side: DisplaySide, path: &VerticalPathData) {
        if let Some(state) = self.display_rendering.get_mut(&side) {
            state.vertical_display.path_data_update(path.clone());

            let old_distance = self.track_changes_distance.get(&side).copied().unwrap_or(-1.0);
            if (path.track_changes_significantly_at_distance - old_distance).abs() > 0.1 {
                state.navigation_display.reset();
                state.vertical_display.reset(false);
            }

            self.track_changes_distance.insert(side, path.track_changes_significantly_at_distance);
        }
    }

    fn update_cached_tiles(&mut self, position: PositionData) {
        let worldmap = match &mut self.worldmap {
            Some(wm) => wm,
            None => {
                debug!("update_cached_tiles: no worldmap available");
                return;
            }
        };

     //   debug!("update_cached_tiles: pos=({:.4}, {:.4})", position.latitude, position.longitude);

        let lookup = worldmap.create_grid_lookup_table(
            position,
            GPU_MAX_PIXEL_SIZE,
            GPU_MAX_PIXEL_SIZE,
            DEFAULT_TILE_SIZE,
        );

        let tiles_loaded = worldmap.update_position(&lookup.grid);
        let relevant_tile_count = lookup.grid.len() * lookup.grid.first().map(|r| r.len()).unwrap_or(0);

        let cached_tiles = self.cached_elevation_data.as_ref().map(|c| c.cached_tiles).unwrap_or(0);

        if tiles_loaded || cached_tiles != relevant_tile_count {
            // Update cached elevation data
            let width = lookup.min_width_per_tile * lookup.grid.first().map(|r| r.len()).unwrap_or(0);
            let height = lookup.min_height_per_tile * lookup.grid.len();

            let mut elevation_data = vec![0.0f32; width * height];
            let mut target_index = 0;

            for row in &lookup.grid {
                for y in 0..lookup.min_height_per_tile {
                    for grid_idx in row {
                        let cell = &worldmap.tile_manager.grid[grid_idx.row][grid_idx.column];

                        for x in 0..lookup.min_width_per_tile {
                            let elevation = if cell.tile_index == -1 {
                                WATER_ELEVATION as f32
                            } else if let Some(ref elevation_map) = cell.elevation_map {
                                // Standard row-major order: y * columns + x
                                if y < elevation_map.rows && x < elevation_map.columns {
                                    elevation_map.elevation_map[y * elevation_map.columns + x] as f32
                                } else {
                                    UNKNOWN_ELEVATION as f32
                                }
                            } else {
                                UNKNOWN_ELEVATION as f32
                            };

                            if target_index < elevation_data.len() {
                                elevation_data[target_index] = elevation;
                            }
                            target_index += 1;
                        }
                    }
                }
            }

            self.cached_elevation_data = Some(CachedElevationData {
                data: elevation_data,
                cached_tiles: relevant_tile_count,
            });

            self.world_map_metadata = WorldMapMetadata {
                southwest: lookup.southwest,
                northeast: lookup.northeast,
                current_grid_position: (0, 0),
                min_width_per_tile: lookup.min_width_per_tile,
                min_height_per_tile: lookup.min_height_per_tile,
                width,
                height,
            };
        }
    }

    fn render_navigation_display_frame(&mut self, side: DisplaySide) -> Option<Vec<u8>> {
        self.render_navigation_display_frame_with_stats(side).map(|(frame, _, _, _)| frame)
    }

    /// Render navigation display frame and return (frame_data, min_for_display, max_for_display, is_normal_mode)
    /// Render raw RGBA frame with elevation stats (no PNG encoding)
    /// Returns (raw_rgba_frame, min_elevation, max_elevation, is_normal_mode, width, height)
    fn render_raw_frame_with_stats(&mut self, side: DisplaySide) -> Option<(Vec<u8>, i32, i32, bool, usize, usize)> {
        let state = self.display_rendering.get(&side)?;
        let config = state.navigation_display.display_configuration();

        debug!("render_raw_frame_with_stats({:?}): terr_on_nd={}, terr_on_vd={}, nd_range={}",
            side, config.terr_on_nd, config.terr_on_vd, config.nd_range);

        if !config.terr_on_nd && !config.terr_on_vd {
            debug!("Terrain not enabled for {:?}, skipping render", side);
            return None;
        }

        let status = match self.aircraft_status.as_ref() {
            Some(s) => s,
            None => {
                warn!("No aircraft status available for rendering {:?}", side);
                return None;
            }
        };
        let _cached_data = match self.cached_elevation_data.as_ref() {
            Some(d) => d,
            None => {
                warn!("No cached elevation data available for rendering {:?}", side);
                return None;
            }
        };

        // Determine display dimensions
        let display_width = NAVIGATION_DISPLAY_MAX_PIXEL_WIDTH;
        let display_height = if self.vertical_display_required {
            DISPLAY_SCREEN_PIXEL_HEIGHT_WITH_VERTICAL_DISPLAY
        } else {
            DISPLAY_SCREEN_PIXEL_HEIGHT_WITHOUT_VERTICAL_DISPLAY
        };

        // Create RGBA frame buffer
        let mut frame = vec![0u8; display_width * display_height * RENDERING_COLOR_CHANNEL_COUNT];

        // Fill with background color (dark blue/gray - matches TypeScript RGBA(4, 4, 5, 0))
        // Alpha = 0 for transparency (same as TypeScript's 328708 = 0x00050404)
        for i in 0..(display_width * display_height) {
            frame[i * 4] = 4;     // R
            frame[i * 4 + 1] = 4; // G
            frame[i * 4 + 2] = 5; // B
            frame[i * 4 + 3] = 0; // A = 0 (transparent, same as TypeScript)
        }

        // Render terrain on navigation display area if enabled
        let (min_elev, max_elev, is_normal_mode) = if config.terr_on_nd {
            self.render_terrain_to_frame(
                &mut frame,
                display_width,
                side,
                status,
                NAVIGATION_DISPLAY_MAP_START_OFFSET_Y,
                config.map_width.unwrap_or(NAVIGATION_DISPLAY_MAX_PIXEL_WIDTH as u32) as usize,
                config.map_height.unwrap_or(NAVIGATION_DISPLAY_MAX_PIXEL_HEIGHT as u32) as usize,
            )
        } else {
            (0, 0, false)
        };

        // NOTE: Vertical display is rendered separately with its own transition
        // It will be composited onto the frame after both transitions are applied

        Some((frame, min_elev, max_elev, is_normal_mode, display_width, display_height))
    }

    fn render_navigation_display_frame_with_stats(&mut self, side: DisplaySide) -> Option<(Vec<u8>, i32, i32, bool)> {
        let (frame, min_elev, max_elev, is_normal_mode, display_width, display_height) =
            self.render_raw_frame_with_stats(side)?;

        // Encode to PNG
        match self.encode_frame_to_png(&frame, display_width, display_height) {
            Ok(png_data) => {
                info!("Encoded PNG frame for {:?}: {} bytes, {}x{}", side, png_data.len(), display_width, display_height);

                // Save PNG to disk for debugging - use temp directory for easy access
                let side_str = match side {
                    DisplaySide::Left => "left",
                    DisplaySide::Right => "right",
                };
            /*     let debug_path = std::path::PathBuf::from(r"C:\temp\terrain_debug");
                if let Err(e) = std::fs::create_dir_all(&debug_path) {
                    warn!("Failed to create debug directory {:?}: {}", debug_path, e);
                }
                let filename = debug_path.join(format!("terrain_frame_{}.png", side_str));
                if let Err(e) = std::fs::write(&filename, &png_data) {
                    warn!("Failed to save debug PNG to {:?}: {}", filename, e);
                } else {
                    info!("Saved debug PNG to {:?}", filename);
                }
 */
                Some((png_data, min_elev, max_elev, is_normal_mode))
            }
            Err(e) => {
                error!("Failed to encode frame to PNG: {}", e);
                None
            }
        }
    }

    /// Project a WGS84 coordinate by bearing and distance
    /// Returns (latitude, longitude) in degrees
    fn project_wgs84(latitude: f64, longitude: f64, bearing_deg: f64, distance_m: f64) -> (f64, f64) {
        const EARTH_RADIUS: f64 = 6371010.0;

        let lat_rad = latitude.to_radians();
        let lon_rad = longitude.to_radians();
        let bearing_rad = bearing_deg.to_radians();
        let ratio = distance_m / EARTH_RADIUS;

        let lat_dest = (lat_rad.sin() * ratio.cos() +
                        lat_rad.cos() * ratio.sin() * bearing_rad.cos()).asin();

        let lon_dest = lon_rad + (bearing_rad.sin() * ratio.sin() * lat_rad.cos())
            .atan2(ratio.cos() - lat_rad.sin() * lat_dest.sin());

        let mut lat_deg = lat_dest.to_degrees();
        let mut lon_deg = lon_dest.to_degrees();

        // Normalize latitude to [-90, 90]
        if lat_deg < -90.0 { lat_deg = -180.0 - lat_deg; }
        if lat_deg > 90.0 { lat_deg = 180.0 - lat_deg; }

        // Normalize longitude to [-180, 180]
        if lon_deg < -180.0 { lon_deg += 360.0; }
        if lon_deg > 180.0 { lon_deg -= 360.0; }

        (lat_deg, lon_deg)
    }

    /// Normalize heading to [0, 360)
    fn normalize_heading(angle: f64) -> f64 {
        angle - (angle / 360.0).floor() * 360.0
    }

    /// Render terrain to frame buffer and return (min_for_display, max_for_display, is_normal_mode)
    /// min_for_display is the lowDensityGreen threshold (or cutoff if higher) - the lowest rendered
    /// max_for_display is the actual maximum terrain elevation
    fn render_terrain_to_frame(
        &self,
        frame: &mut [u8],
        frame_width: usize,
        side: DisplaySide,
        status: &AircraftStatus,
        offset_y: usize,
        map_width: usize,
        map_height: usize,
    ) -> (i32, i32, bool) {
        let cached_data = match &self.cached_elevation_data {
            Some(data) => data,
            None => {
                warn!("render_terrain_to_frame: no cached elevation data");
                return (0, 0, false);
            }
        };

        let metadata = &self.world_map_metadata;
        if metadata.width == 0 || metadata.height == 0 {
            warn!("render_terrain_to_frame: invalid world map metadata ({}x{})", metadata.width, metadata.height);
            return (0, 0, false);
        }

        let config = self.display_rendering.get(&side)
            .map(|s| s.navigation_display.display_configuration().clone())
            .unwrap_or_default();

        let offset_x = config.map_offset_x.unwrap_or(0) as usize;
        let center_offset_y = config.center_offset_y.unwrap_or(0) as f64;
        let nd_range = config.nd_range as f64; // in nautical miles
        let arc_mode = config.arc_mode;

        // Calculate meters per pixel based on ND range
        // Range is displayed from center to top of display
        // TypeScript uses Math.round() for this value
        let display_range_pixels = (map_height as f64) - center_offset_y;
        let mut meters_per_pixel = ((nd_range * NAUTICAL_MILES_TO_METRES) / display_range_pixels).round();
        if arc_mode {
            meters_per_pixel *= 2.0; // Arc mode displays half the range
        }

        let aircraft_lat = status.latitude;
        let aircraft_lon = status.longitude;
        let heading = status.heading as f64;

        // Calculate degrees per pixel for the world map
        let lat_step = (metadata.northeast.latitude - metadata.southwest.latitude) / metadata.height as f64;
        let lon_step = (metadata.northeast.longitude - metadata.southwest.longitude) / metadata.width as f64;

        debug!("Rendering terrain: aircraft at ({:.4}, {:.4}), heading={}, range={}nm, meters_per_pixel={:.1}",
            aircraft_lat, aircraft_lon, heading, nd_range, meters_per_pixel);
        debug!("World map: SW=({:.4}, {:.4}), NE=({:.4}, {:.4}), size={}x{}",
            metadata.southwest.latitude, metadata.southwest.longitude,
            metadata.northeast.latitude, metadata.northeast.longitude,
            metadata.width, metadata.height);

        // Debug: sample elevation at aircraft position
      /*   let aircraft_sample_x = ((aircraft_lon - metadata.southwest.longitude) / lon_step) as usize;
        let aircraft_sample_y = ((metadata.northeast.latitude - aircraft_lat) / lat_step) as usize;
        if aircraft_sample_x < metadata.width && aircraft_sample_y < metadata.height {
            let idx = aircraft_sample_y * metadata.width + aircraft_sample_x;
            if idx < cached_data.data.len() {
                debug!("Elevation at aircraft position: {} ft (grid pos: {}, {})",
                    cached_data.data[idx] as i16, aircraft_sample_x, aircraft_sample_y);
            }
        } */

        let center_x = map_width as f64 / 2.0;

        // Color statistics for debugging
        let mut color_stats = std::collections::HashMap::new();
        let mut elevation_samples: Vec<i16> = Vec::new();

        // First pass: calculate min/max elevation in visible area for mode selection
        let mut min_elevation: i32 = i32::MAX;
        let mut max_elevation: i32 = i32::MIN;

        for y in 0..map_height {
            for x in 0..map_width {
                let delta_x = x as f64 - center_x;
                let delta_y = (map_height as f64) - (y as f64) - center_offset_y;

                // Skip pixels behind the aircraft (negative delta_y means behind)
                if delta_y < 0.0 {
                    continue;
                }

                let distance_pixels = (delta_x * delta_x + delta_y * delta_y).sqrt();

                // Arc clipping for A32NX (centerOffsetY == 0)
                if center_offset_y == 0.0 && arc_mode && distance_pixels > map_height as f64 {
                    continue;
                }

                let distance_m = distance_pixels * meters_per_pixel / 2.0;
                let angle = if distance_pixels > 0.0 {
                    (delta_y / distance_pixels).acos().to_degrees()
                } else {
                    0.0
                };
                let bearing = if x as f64 > center_x {
                    Self::normalize_heading(angle + heading)
                } else {
                    Self::normalize_heading(360.0 - angle + heading)
                };

                let (proj_lat, proj_lon) = Self::project_wgs84(aircraft_lat, aircraft_lon, bearing, distance_m);

                if proj_lat >= metadata.southwest.latitude && proj_lat <= metadata.northeast.latitude &&
                   proj_lon >= metadata.southwest.longitude && proj_lon <= metadata.northeast.longitude {
                    let sample_x = ((proj_lon - metadata.southwest.longitude) / lon_step) as usize;
                    let sample_y = ((metadata.northeast.latitude - proj_lat) / lat_step) as usize;

                    if sample_x < metadata.width && sample_y < metadata.height {
                        let elevation_idx = sample_y * metadata.width + sample_x;
                        if elevation_idx < cached_data.data.len() {
                            let elevation = cached_data.data[elevation_idx] as i16;
                            if elevation != WATER_ELEVATION && elevation != UNKNOWN_ELEVATION && elevation != INVALID_ELEVATION {
                                let elev_i32 = elevation as i32;
                                if elev_i32 < min_elevation { min_elevation = elev_i32; }
                                if elev_i32 > max_elevation { max_elevation = elev_i32; }
                                // Store elevation for histogram calculation
                                elevation_samples.push(elevation);
                            }
                        }
                    }
                }
            }
        }

        // Handle case where no valid elevations were found
        if min_elevation == i32::MAX { min_elevation = 0; }
        if max_elevation == i32::MIN { max_elevation = 0; }

        // Build histogram from elevation samples (like TypeScript GPU)
        // Histogram bins from HISTOGRAM_MINIMUM_ELEVATION (-500) to HISTOGRAM_MAXIMUM_ELEVATION (29040)
        // Each bin is HISTOGRAM_BIN_RANGE (100) feet wide
        let bin_count = ((HISTOGRAM_MAXIMUM_ELEVATION - HISTOGRAM_MINIMUM_ELEVATION) / HISTOGRAM_BIN_RANGE) as usize;
        let mut histogram = vec![0u32; bin_count + 1];
        let mut total_samples = 0u32;
        let mut min_bin: i32 = -1;
        let mut max_bin: i32 = -1;

        for &elev in &elevation_samples {
            let elev_i32 = elev as i32;
            let bin = ((elev_i32 - HISTOGRAM_MINIMUM_ELEVATION) / HISTOGRAM_BIN_RANGE) as usize;
            if bin < histogram.len() {
                histogram[bin] += 1;
                total_samples += 1;
                if min_bin < 0 || (bin as i32) < min_bin { min_bin = bin as i32; }
                if max_bin < 0 || (bin as i32) > max_bin { max_bin = bin as i32; }
            }
        }

        // Bin min/max elevations to histogram boundaries (like TypeScript)
        // TypeScript: minElevation = minElevationBin * histogramBinRange + histogramMinElevation
        // TypeScript: maxElevation = (maxElevationBin + 1) * histogramBinRange + histogramMinElevation
        let min_elevation_binned = if min_bin >= 0 {
            min_bin * HISTOGRAM_BIN_RANGE + HISTOGRAM_MINIMUM_ELEVATION
        } else {
            min_elevation
        };
        let max_elevation_binned = if max_bin >= 0 {
            (max_bin + 1) * HISTOGRAM_BIN_RANGE + HISTOGRAM_MINIMUM_ELEVATION
        } else {
            max_elevation
        };

        // Calculate percentile elevations from histogram (like TypeScript GPU)
        // lowerPercentile = 0.85, upperPercentile = 0.95
        let mut lower_percentile_bin: i32 = -1;
        let mut upper_percentile_bin: i32 = -1;
        let mut cumulative_percent = 0.0f64;

        let cutoff_altitude = HISTOGRAM_MINIMUM_ELEVATION; // -500
        let cutoff_bin = 0i32; // First bin (for elevations >= -500)

        for bin in cutoff_bin as usize..histogram.len() {
            if total_samples > 0 {
                cumulative_percent += histogram[bin] as f64 / total_samples as f64;
                if lower_percentile_bin < 0 && cumulative_percent >= RENDERING_LOWER_PERCENTILE {
                    lower_percentile_bin = bin as i32;
                }
                if upper_percentile_bin < 0 && cumulative_percent >= RENDERING_UPPER_PERCENTILE {
                    upper_percentile_bin = bin as i32;
                }
            }
        }

        // Convert percentile bins to elevations
        let lower_percentile_elev = if lower_percentile_bin >= 0 {
            lower_percentile_bin * HISTOGRAM_BIN_RANGE + HISTOGRAM_MINIMUM_ELEVATION
        } else {
            (max_elevation_binned + min_elevation_binned) / 2 // Fallback to half elevation
        };
        let upper_percentile_elev = if upper_percentile_bin >= 0 {
            upper_percentile_bin * HISTOGRAM_BIN_RANGE + HISTOGRAM_MINIMUM_ELEVATION
        } else {
            max_elevation_binned // Fallback to max
        };

        // Use binned values for mode calculations
        let min_elevation = min_elevation_binned;
        let max_elevation = max_elevation_binned;

        // Calculate reference altitude with vertical speed prediction (like TypeScript)
        let reference_altitude = if status.vertical_speed <= -1000 {
            status.altitude + (status.vertical_speed as i32 / 2)
        } else {
            status.altitude
        };

        let gear_offset = if status.gear_is_down { 250 } else { 500 };
        let use_normal_mode = max_elevation >= reference_altitude - gear_offset;

        // Calculate thresholds like TypeScript
        // These are the same values used in elevation_to_color_with_thresholds
        const LOW_DENSITY_GREEN_OFFSET: i32 = 2000;
        const HIGH_DENSITY_GREEN_OFFSET: i32 = 1000;
        const HIGH_DENSITY_YELLOW_OFFSET: i32 = 1000;
        const HIGH_DENSITY_RED_OFFSET: i32 = 2000;
        const FLAT_EARTH_THRESHOLD: i32 = 100;

        // Calculate flatEarth like TypeScript: flatEarthThreshold - (maxElevation - minElevation)
        let flat_earth = FLAT_EARTH_THRESHOLD - (max_elevation - min_elevation);
        let half_elevation = (max_elevation as f64 * 0.5) as i32;

        // Calculate green thresholds (from calculateNormalModeGreenThresholds)
        let mut low_density_green = if reference_altitude - LOW_DENSITY_GREEN_OFFSET <= min_elevation {
            min_elevation + 200
        } else {
            reference_altitude - LOW_DENSITY_GREEN_OFFSET
        };

        let high_density_green = if reference_altitude - HIGH_DENSITY_GREEN_OFFSET <= min_elevation {
            min_elevation + 200
        } else {
            reference_altitude - HIGH_DENSITY_GREEN_OFFSET
        };

        // Apply flatEarth adjustments like TypeScript using actual percentile values
        // TypeScript: lowerPercentile is the elevation at 85th percentile of terrain
        if flat_earth >= 0 {
            if half_elevation <= lower_percentile_elev && low_density_green > half_elevation {
                low_density_green = half_elevation;
            } else if half_elevation > lower_percentile_elev && low_density_green > lower_percentile_elev {
                low_density_green = lower_percentile_elev;
            }
        }

        // Warning thresholds
        let low_density_yellow = if reference_altitude - gear_offset <= min_elevation {
            min_elevation + 200
        } else {
            reference_altitude - gear_offset
        };
        let high_density_yellow = reference_altitude + HIGH_DENSITY_YELLOW_OFFSET;
        let high_density_red = reference_altitude + HIGH_DENSITY_RED_OFFSET;

        // Create thresholds struct for consistent use in coloring and metadata
        let thresholds = RenderingThresholds {
            low_density_green,
            high_density_green,
            low_density_yellow,
            high_density_yellow,
            high_density_red,
            cutoff_altitude,
            reference_altitude,
            use_normal_mode,
            min_elevation,
            max_elevation,
        };
/*
        debug!("Thresholds: low_green={}, high_green={}, low_yellow={}, high_yellow={}, high_red={}",
            thresholds.low_density_green, thresholds.high_density_green, thresholds.low_density_yellow,
            thresholds.high_density_yellow, thresholds.high_density_red);
        debug!("Elevation range: {} to {} ft, aircraft: {} ft (ref: {}), cutoff: {} ft, using {} mode",
            thresholds.min_elevation, thresholds.max_elevation, status.altitude, thresholds.reference_altitude,
            thresholds.cutoff_altitude, if thresholds.use_normal_mode { "NORMAL" } else { "PEAKS" });
 */
        // Second pass: Render each pixel
        for y in 0..map_height {
            for x in 0..map_width {
                let pixel_x = offset_x + x;
                let pixel_y = offset_y + y;

                // Calculate distance and bearing from aircraft to this pixel
                let delta_x = x as f64 - center_x;
                let delta_y = (map_height as f64) - (y as f64) - center_offset_y;

                // Skip pixels behind the aircraft (negative delta_y means behind)
                if delta_y < 0.0 {
                    continue;
                }

                let distance_pixels = (delta_x * delta_x + delta_y * delta_y).sqrt();

                // Arc clipping for A32NX (centerOffsetY == 0)
                if center_offset_y == 0.0 && arc_mode && distance_pixels > map_height as f64 {
                    continue; // Skip - outside arc
                }

                // Calculate distance in meters
                let distance_m = distance_pixels * meters_per_pixel / 2.0;

                // Calculate bearing angle
                let angle = if distance_pixels > 0.0 {
                    (delta_y / distance_pixels).acos().to_degrees()
                } else {
                    0.0
                };

                let bearing = if x as f64 > center_x {
                    Self::normalize_heading(angle + heading)
                } else {
                    Self::normalize_heading(360.0 - angle + heading)
                };

                // Project to world coordinates
                let (proj_lat, proj_lon) = Self::project_wgs84(aircraft_lat, aircraft_lon, bearing, distance_m);

                // Debug: log projection at center-bottom pixel (aircraft position)
                if x == map_width / 2 && y == map_height - 1 {
                    debug!("Center-bottom pixel: delta=({:.1}, {:.1}), dist_px={:.1}, dist_m={:.1}, angle={:.1}, bearing={:.1}, proj=({:.4}, {:.4})",
                        delta_x, delta_y, distance_pixels, distance_m, angle, bearing, proj_lat, proj_lon);
                }

                // Convert world coordinates to cached data indices
                // Check if projected point is within our cached world map
                if proj_lat < metadata.southwest.latitude || proj_lat > metadata.northeast.latitude ||
                   proj_lon < metadata.southwest.longitude || proj_lon > metadata.northeast.longitude {
                    // Outside cached area - show as unknown
                    let frame_idx = (pixel_y * frame_width + pixel_x) * RENDERING_COLOR_CHANNEL_COUNT;
                    if frame_idx + 3 < frame.len() {
                        frame[frame_idx] = 64;     // Gray
                        frame[frame_idx + 1] = 64;
                        frame[frame_idx + 2] = 64;
                        frame[frame_idx + 3] = 255;
                    }
                    continue;
                }

                // Calculate pixel position in cached elevation data
                let sample_x = ((proj_lon - metadata.southwest.longitude) / lon_step) as usize;
                let sample_y = ((metadata.northeast.latitude - proj_lat) / lat_step) as usize;

                if sample_x < metadata.width && sample_y < metadata.height {
                    let elevation_idx = sample_y * metadata.width + sample_x;
                    if elevation_idx < cached_data.data.len() {
                        let elevation = cached_data.data[elevation_idx] as i16;

                        // Sample elevations for debug stats
                        if elevation_samples.len() < 100 {
                            elevation_samples.push(elevation);
                        }

                        // Determine color and pattern based on elevation relative to aircraft
                        let (r, g, b, pattern_idx) = Self::elevation_to_color_with_thresholds(
                            elevation,
                            &thresholds,
                        );

                        // Handle pattern index 254 = black background (always draw black)
                        if pattern_idx == 254 {
                            let frame_idx = (pixel_y * frame_width + pixel_x) * RENDERING_COLOR_CHANNEL_COUNT;
                            if frame_idx + 3 < frame.len() {
                                frame[frame_idx] = 0;
                                frame[frame_idx + 1] = 0;
                                frame[frame_idx + 2] = 0;
                                frame[frame_idx + 3] = 255; // Opaque black like TypeScript
                            }
                            continue;
                        }

                        // Apply pattern - only draw if pattern check passes
                        if self.should_draw_pattern(x, y, pattern_idx) {
                            // Track color stats
                            let color_key = format!("{},{},{}", r, g, b);
                            *color_stats.entry(color_key).or_insert(0) += 1;

                            // Set pixel in frame
                            let frame_idx = (pixel_y * frame_width + pixel_x) * RENDERING_COLOR_CHANNEL_COUNT;
                            if frame_idx + 3 < frame.len() {
                                frame[frame_idx] = r;
                                frame[frame_idx + 1] = g;
                                frame[frame_idx + 2] = b;
                                frame[frame_idx + 3] = 255;
                            }
                        }
                        // If pattern check fails, pixel stays transparent (alpha=0 from initialization)
                    }
                }
            }
        }

        // Log color distribution
      /*   debug!("Color distribution: {:?}", color_stats);
        debug!("Sample elevations (first 100): {:?}", elevation_samples);
        debug!("Aircraft altitude: {} ft, gear down: {}", status.altitude, status.gear_is_down); */

        // Draw aircraft position marker (white cross) at center-bottom
        let aircraft_pixel_x = offset_x + (map_width / 2);
        let aircraft_pixel_y = offset_y + map_height - (center_offset_y as usize).max(1);
        for dx in -5i32..=5 {
            let px = (aircraft_pixel_x as i32 + dx) as usize;
            if px < frame_width {
                let idx = (aircraft_pixel_y * frame_width + px) * RENDERING_COLOR_CHANNEL_COUNT;
                if idx + 3 < frame.len() {
                    frame[idx] = 255;
                    frame[idx + 1] = 255;
                    frame[idx + 2] = 255;
                    frame[idx + 3] = 255;
                }
            }
        }
        for dy in -5i32..=5 {
            let py = (aircraft_pixel_y as i32 + dy) as usize;
            if py < offset_y + map_height {
                let idx = (py * frame_width + aircraft_pixel_x) * RENDERING_COLOR_CHANNEL_COUNT;
                if idx + 3 < frame.len() {
                    frame[idx] = 255;
                    frame[idx + 1] = 255;
                    frame[idx + 2] = 255;
                    frame[idx + 3] = 255;
                }
            }
        }

        debug!("Terrain rendering complete for {:?}", side);

        // Return threshold values for metadata display (like TypeScript):
        // - min_for_display: lowDensityGreen threshold (or cutoff if higher), binned to histogram boundary
        // - max_for_display: max terrain elevation, binned to next histogram boundary
        // - is_normal_mode: whether we're in normal or peaks mode
        //
        // TypeScript uses histogram binning with 100ft bins starting at -500ft:
        // - minElevation = bin * 100 - 500 (floor to bin boundary)
        // - maxElevation = (bin + 1) * 100 - 500 (ceil to next bin boundary)
        let min_raw = if thresholds.cutoff_altitude > thresholds.low_density_green {
            thresholds.cutoff_altitude
        } else {
            thresholds.low_density_green
        };

        // Bin to histogram boundaries like TypeScript
        // min: floor to bin boundary
        let min_bin = (min_raw - HISTOGRAM_MINIMUM_ELEVATION) / HISTOGRAM_BIN_RANGE;
        let min_for_display = min_bin * HISTOGRAM_BIN_RANGE + HISTOGRAM_MINIMUM_ELEVATION;

        // max: ceil to next bin boundary (bin + 1)
        let max_bin = (thresholds.max_elevation - HISTOGRAM_MINIMUM_ELEVATION) / HISTOGRAM_BIN_RANGE;
        let max_for_display = (max_bin + 1) * HISTOGRAM_BIN_RANGE + HISTOGRAM_MINIMUM_ELEVATION;

        (min_for_display, max_for_display, thresholds.use_normal_mode)
    }

    /// Render vertical display terrain to a separate buffer for transition processing
    /// Returns a buffer sized for the vertical display (RENDERING_ELEVATION_PROFILE_WIDTH x RENDERING_ELEVATION_PROFILE_HEIGHT)
    fn render_vertical_display_raw(
        &self,
        side: DisplaySide,
        status: &AircraftStatus,
    ) -> Option<Vec<u8>> {
        let cached_data = match &self.cached_elevation_data {
            Some(data) => data,
            None => {
                debug!("render_vertical_display_raw: no cached elevation data");
                return None;
            }
        };

        let metadata = &self.world_map_metadata;
        if metadata.width == 0 || metadata.height == 0 {
            debug!("render_vertical_display_raw: invalid world map metadata");
            return None;
        }

        // Get vertical display configuration
        let vd_config = match self.display_rendering.get(&side) {
            Some(state) => state.vertical_display.display_configuration().clone(),
            None => {
                debug!("render_vertical_display_raw: no display state for {:?}", side);
                return None;
            }
        };

        // Get navigation display config for heading
        let nd_config = match self.display_rendering.get(&side) {
            Some(state) => state.navigation_display.display_configuration().clone(),
            None => return None,
        };

        let vd_width = RENDERING_ELEVATION_PROFILE_WIDTH;
        let vd_height = RENDERING_ELEVATION_PROFILE_HEIGHT;
        let min_altitude = vd_config.minimum_altitude;
        let max_altitude = vd_config.maximum_altitude;

        // Get elevation profile range (how far ahead to sample, in nm)
        // For arc mode, use nd_range; for rose mode, use nd_range / 2
        let profile_range_nm = if nd_config.arc_mode {
            nd_config.nd_range.max(10).min(160) as f64
        } else {
            (nd_config.nd_range / 2).max(5).min(160) as f64
        };

        // Create elevation profile along heading
        let elevation_profile = self.create_elevation_profile(
            status.latitude,
            status.longitude,
            status.heading as f64,
            profile_range_nm,
            vd_width,
        );

        // Create buffer for VD
        let mut buffer = vec![0u8; vd_width * vd_height * RENDERING_COLOR_CHANNEL_COUNT];

        // Render the vertical display
        let altitude_range = (max_altitude - min_altitude) as f64;
        let altitude_step = altitude_range / vd_height as f64;

        for y in 0..vd_height {
            // Altitude at this row (top = max, bottom = min)
            let altitude = (vd_height - y) as f64 * altitude_step + min_altitude as f64;

            for x in 0..vd_width {
                let elevation = elevation_profile[x];

                // Calculate pixel position in the buffer
                let buf_idx = (y * vd_width + x) * RENDERING_COLOR_CHANNEL_COUNT;

                // Determine color based on elevation vs altitude
                let (r, g, b, a) = if elevation == INVALID_ELEVATION as f32 || elevation == UNKNOWN_ELEVATION as f32 {
                    // Unknown/invalid - magenta
                    (255u8, 148u8, 255u8, 255u8)
                } else if altitude > elevation as f64 {
                    // Above terrain - transparent background
                    (0u8, 0u8, 0u8, 0u8)
                } else if elevation == WATER_ELEVATION as f32 {
                    // Water - cyan if at/below sea level
                    if altitude <= 0.0 {
                        (0u8, 255u8, 255u8, 255u8)
                    } else {
                        (0u8, 0u8, 0u8, 0u8)
                    }
                } else {
                    // Terrain/obstacle - brown color (like TypeScript: 110, 51, 14)
                    (110u8, 51u8, 14u8, 255u8)
                };

                buffer[buf_idx] = r;
                buffer[buf_idx + 1] = g;
                buffer[buf_idx + 2] = b;
                buffer[buf_idx + 3] = a;
            }
        }

        debug!("Vertical display raw rendering complete for {:?}", side);
        Some(buffer)
    }

    /// Render vertical display terrain profile to the frame buffer
    /// The VD shows terrain elevation along the flight path as a cross-section view
    fn render_vertical_display_to_frame(
        &self,
        frame: &mut [u8],
        frame_width: usize,
        side: DisplaySide,
        status: &AircraftStatus,
    ) {
        let cached_data = match &self.cached_elevation_data {
            Some(data) => data,
            None => {
                debug!("render_vertical_display_to_frame: no cached elevation data");
                return;
            }
        };

        let metadata = &self.world_map_metadata;
        if metadata.width == 0 || metadata.height == 0 {
            debug!("render_vertical_display_to_frame: invalid world map metadata");
            return;
        }

        // Get vertical display configuration
        let vd_config = match self.display_rendering.get(&side) {
            Some(state) => state.vertical_display.display_configuration().clone(),
            None => {
                debug!("render_vertical_display_to_frame: no display state for {:?}", side);
                return;
            }
        };

        // Get navigation display config for heading
        let nd_config = match self.display_rendering.get(&side) {
            Some(state) => state.navigation_display.display_configuration().clone(),
            None => return,
        };

        let vd_width = RENDERING_ELEVATION_PROFILE_WIDTH;
        let vd_height = RENDERING_ELEVATION_PROFILE_HEIGHT;
        let min_altitude = vd_config.minimum_altitude;
        let max_altitude = vd_config.maximum_altitude;

        // Get elevation profile range (how far ahead to sample, in nm)
        // For arc mode, use nd_range; for rose mode, use nd_range / 2
        let profile_range_nm = if nd_config.arc_mode {
            nd_config.nd_range.max(10).min(160) as f64
        } else {
            (nd_config.nd_range / 2).max(5).min(160) as f64
        };

        // Create elevation profile along heading
        let elevation_profile = self.create_elevation_profile(
            status.latitude,
            status.longitude,
            status.heading as f64,
            profile_range_nm,
            vd_width,
        );

        // Render the vertical display
        let altitude_range = (max_altitude - min_altitude) as f64;
        let altitude_step = altitude_range / vd_height as f64;

        // Calculate lat/lon steps for the world map
        let lat_step = (metadata.northeast.latitude - metadata.southwest.latitude) / metadata.height as f64;
        let lon_step = (metadata.northeast.longitude - metadata.southwest.longitude) / metadata.width as f64;

        for y in 0..vd_height {
            // Altitude at this row (top = max, bottom = min)
            let altitude = (vd_height - y) as f64 * altitude_step + min_altitude as f64;

            for x in 0..vd_width {
                let elevation = elevation_profile[x];

                // Calculate pixel position in the output frame
                let frame_x = VERTICAL_DISPLAY_MAP_START_OFFSET_X + x;
                let frame_y = VERTICAL_DISPLAY_MAP_START_OFFSET_Y + y;
                let frame_idx = (frame_y * frame_width + frame_x) * RENDERING_COLOR_CHANNEL_COUNT;

                if frame_idx + 3 >= frame.len() {
                    continue;
                }

                // Determine color based on elevation vs altitude
                let (r, g, b, a) = if elevation == INVALID_ELEVATION as f32 || elevation == UNKNOWN_ELEVATION as f32 {
                    // Unknown/invalid - magenta
                    (255u8, 148u8, 255u8, 255u8)
                } else if altitude > elevation as f64 {
                    // Above terrain - transparent background
                    (0u8, 0u8, 0u8, 0u8)
                } else if elevation == WATER_ELEVATION as f32 {
                    // Water - cyan if at/below sea level
                    if altitude <= 0.0 {
                        (0u8, 255u8, 255u8, 255u8)
                    } else {
                        (0u8, 0u8, 0u8, 0u8)
                    }
                } else {
                    // Terrain/obstacle - brown color (like TypeScript: 110, 51, 14)
                    (110u8, 51u8, 14u8, 255u8)
                };

                frame[frame_idx] = r;
                frame[frame_idx + 1] = g;
                frame[frame_idx + 2] = b;
                frame[frame_idx + 3] = a;
            }
        }

        debug!("Vertical display rendering complete for {:?}", side);
    }

    /// Create an elevation profile along the aircraft heading
    /// Returns a vector of elevations (one per pixel width)
    fn create_elevation_profile(
        &self,
        latitude: f64,
        longitude: f64,
        heading: f64,
        range_nm: f64,
        profile_width: usize,
    ) -> Vec<f32> {
        let mut profile = vec![INVALID_ELEVATION as f32; profile_width];

        let cached_data = match &self.cached_elevation_data {
            Some(data) => data,
            None => return profile,
        };

        let metadata = &self.world_map_metadata;
        if metadata.width == 0 || metadata.height == 0 {
            return profile;
        }

        // Calculate distance per pixel
        let total_distance_m = range_nm * NAUTICAL_MILES_TO_METRES;
        let distance_per_pixel = total_distance_m / profile_width as f64;

        // Calculate lat/lon steps for the world map
        let lat_step = (metadata.northeast.latitude - metadata.southwest.latitude) / metadata.height as f64;
        let lon_step = (metadata.northeast.longitude - metadata.southwest.longitude) / metadata.width as f64;

        for x in 0..profile_width {
            let distance_m = x as f64 * distance_per_pixel;

            // Project position at this distance along heading
            let (proj_lat, proj_lon) = Self::project_wgs84(latitude, longitude, heading, distance_m);

            // Check if within cached map bounds
            if proj_lat < metadata.southwest.latitude || proj_lat > metadata.northeast.latitude ||
               proj_lon < metadata.southwest.longitude || proj_lon > metadata.northeast.longitude {
                continue; // Leave as invalid
            }

            // Convert to pixel coordinates in cached data
            let sample_x = ((proj_lon - metadata.southwest.longitude) / lon_step) as usize;
            let sample_y = ((metadata.northeast.latitude - proj_lat) / lat_step) as usize;

            if sample_x < metadata.width && sample_y < metadata.height {
                let elevation_idx = sample_y * metadata.width + sample_x;
                if elevation_idx < cached_data.data.len() {
                    profile[x] = cached_data.data[elevation_idx];
                }
            }
        }

        profile
    }

    /// Determine terrain color based on pre-calculated thresholds
    /// This uses the same thresholds that are returned for metadata display
    /// This implements both Normal Mode and Peaks Mode coloring from the TypeScript version
    /// Returns (r, g, b, pattern_index) where pattern_index controls density:
    /// - 0: transparent (don't draw)
    /// - 3: low density (sparse, ~1 in 3 pixels)
    /// - 5: medium density (~1 in 5 pixels)
    /// - 7: sparse (water, ~1 in 7 pixels)
    /// - 255: solid (always draw)
    fn elevation_to_color_with_thresholds(elevation: i16, thresholds: &RenderingThresholds) -> (u8, u8, u8, u8) {
        // Special elevation values
        // Water - cyan with sparse density (pattern index 7)
        if elevation == WATER_ELEVATION {
            return (0, 255, 255, 7);
        }
        // Unknown - magenta/pink (pattern index 5)
        if elevation == UNKNOWN_ELEVATION {
            return (255, 148, 255, 5);
        }
        // Invalid - black background (opaque like TypeScript)
        if elevation == INVALID_ELEVATION {
            return (0, 0, 0, 254); // 254 = black background marker
        }

        let elevation_ft = elevation as i32;

        // TypeScript: check elevation >= absoluteCutOffAltitude before rendering colors
        if elevation_ft < thresholds.cutoff_altitude {
            return (0, 0, 0, 254);
        }

        if thresholds.use_normal_mode {
            // NORMAL MODE - use pre-calculated thresholds directly
            // TypeScript order (from renderNormalMode):
            // 1. elevation >= warningThresholds[2] (high_density_red) -> red high density
            // 2. elevation >= warningThresholds[1] (high_density_yellow) -> yellow high density
            // 3. elevation >= greenThresholds[1] (high_density_green) AND elevation < warningThresholds[0] (low_density_yellow) -> green high density
            // 4. elevation >= warningThresholds[0] (low_density_yellow) AND elevation < warningThresholds[1] (high_density_yellow) -> yellow low density
            // 5. elevation >= greenThresholds[0] (low_density_green) AND elevation < greenThresholds[1] (high_density_green) -> green low density

            if elevation_ft >= thresholds.high_density_red {
                // High density red - immediate danger (pattern index 5)
                (255, 0, 0, 5)
            } else if elevation_ft >= thresholds.high_density_yellow {
                // High density yellow - caution (pattern index 5)
                (255, 255, 50, 5)
            } else if elevation_ft >= thresholds.high_density_green && elevation_ft < thresholds.low_density_yellow {
                // High density green - terrain close but below (pattern index 5)
                (0, 255, 0, 5)
            } else if elevation_ft >= thresholds.low_density_yellow && elevation_ft < thresholds.high_density_yellow {
                // Low density yellow - approaching caution level (pattern index 3)
                (255, 255, 50, 3)
            } else if elevation_ft >= thresholds.low_density_green && elevation_ft < thresholds.high_density_green {
                // Low density green - safe terrain (pattern index 3)
                (0, 255, 0, 3)
            } else {
                // Below all thresholds - black background
                (0, 0, 0, 254)
            }
        } else {
            // PEAKS MODE - terrain is well below aircraft, show terrain relief
            // Calculate thresholds based on terrain distribution
            let elevation_range = thresholds.max_elevation - thresholds.min_elevation;
            let half_elevation = (thresholds.max_elevation + thresholds.min_elevation) / 2;

            // Calculate density thresholds (from calculatePeaksModeThresholds)
            let lower_density = half_elevation;
            let higher_density = thresholds.min_elevation + (elevation_range as f64 * 0.65) as i32;
            let solid_density = thresholds.min_elevation + (elevation_range as f64 * 0.95) as i32;

            // Determine color based on elevation relative to terrain distribution
            if elevation_ft >= solid_density {
                // Solid green - highest peaks (solid, no pattern)
                (0, 255, 0, 255)
            } else if elevation_ft >= higher_density {
                // High density green (pattern index 5)
                (0, 255, 0, 5)
            } else if elevation_ft >= lower_density {
                // Low density green (pattern index 3)
                (0, 255, 0, 3)
            } else {
                // Below all thresholds - black background
                (0, 0, 0, 254)
            }
        }
    }

    /// Check if a pixel should be drawn based on pattern
    /// Uses the exact same logic as TypeScript's drawDensityPixel:
    /// `if (Math.round(patternValue % patternIndex) === 0)` -> draw, else transparent
    fn should_draw_pattern(&self, x: usize, y: usize, pattern_index: u8) -> bool {
        if pattern_index == 0 {
            return false; // Transparent
        }
        if pattern_index == 255 {
            return true; // Solid
        }

        // Get pattern value based on rendering mode:
        // - ScanlineMode (A380X): use scanline pattern (592 height)
        // - ArcMode (A32NX): use arc pattern (492 height)
        let use_scanline_mode = self.rendering_mode == TerrainRenderingMode::ScanlineMode;
        let pattern_value = get_pattern_value(x, y, use_scanline_mode);

        // Use the exact TypeScript logic: drawDensityPixel
        draw_density_pixel(pattern_value, pattern_index)
    }

    fn encode_frame_to_png(&self, frame: &[u8], width: usize, height: usize) -> Result<Vec<u8>> {
        use png::{BitDepth, ColorType, Encoder, FilterType};

        let mut output = Vec::with_capacity(frame.len() + 1024); // Pre-allocate
        {
            let mut encoder = Encoder::new(&mut output, width as u32, height as u32);
            encoder.set_color(ColorType::Rgba);
            encoder.set_depth(BitDepth::Eight);

            let mut writer = encoder.write_header()?;
            writer.write_image_data(frame)?;
        }

        Ok(output)
    }

    /// Public method to encode a raw RGBA frame to PNG
    /// Used by SimConnect handler to encode transition frames before sending
    pub fn encode_to_png(&self, frame: &[u8], width: usize, height: usize) -> Option<Vec<u8>> {
        self.encode_frame_to_png(frame, width, height).ok()
    }

    /// Get the current display dimensions
    pub fn get_display_dimensions(&self) -> (usize, usize) {
        let display_width = NAVIGATION_DISPLAY_MAX_PIXEL_WIDTH;
        let display_height = if self.vertical_display_required {
            DISPLAY_SCREEN_PIXEL_HEIGHT_WITH_VERTICAL_DISPLAY
        } else {
            DISPLAY_SCREEN_PIXEL_HEIGHT_WITHOUT_VERTICAL_DISPLAY
        };
        (display_width, display_height)
    }

    /// Render a raw RGBA navigation display frame for use in transitions
    /// Returns (metadata, raw_rgba_frame, width, height)
    /// The frame_byte_count in metadata is set to raw frame size; caller should update after PNG encoding
    pub fn render_raw_frame_for_transition(&mut self, side: DisplaySide) -> Option<(crate::types::NavigationDisplayData, Vec<u8>, usize, usize)> {
        if !self.initialized {
            debug!("render_raw_frame_for_transition: not initialized");
            return None;
        }

        // Check if terrain display is enabled
        {
            let state = match self.display_rendering.get(&side) {
                Some(s) => s,
                None => {
                    warn!("No display rendering state for {:?}", side);
                    return None;
                }
            };
            let nd_config = state.navigation_display.display_configuration();

            if !nd_config.terr_on_nd && !nd_config.terr_on_vd {
                //debug!("Terrain display not enabled for {:?}", side);
                return None;
            }
        }

        // Render raw RGBA frame
        let (frame, min_for_display, max_for_display, is_normal_mode, width, height) =
            self.render_raw_frame_with_stats(side)?;

        // Calculate elevation modes
        let (min_mode, max_mode) = {
            let status = self.aircraft_status.as_ref()?;
            let gear_offset = if status.gear_is_down { 250 } else { 500 };

            let reference_altitude = if status.vertical_speed <= -1000 {
                status.altitude + (status.vertical_speed as i32 / 2)
            } else {
                status.altitude
            };

            const HIGH_DENSITY_GREEN_OFFSET: i32 = 1000;
            const HIGH_DENSITY_RED_OFFSET: i32 = 2000;

            if is_normal_mode {
                let high_density_green = reference_altitude - HIGH_DENSITY_GREEN_OFFSET;
                let low_density_yellow = reference_altitude - gear_offset;
                let high_density_red = reference_altitude + HIGH_DENSITY_RED_OFFSET;

                let min_mode = if low_density_yellow <= high_density_green {
                    TerrainLevelMode::Warning
                } else {
                    TerrainLevelMode::PeaksMode
                };

                let max_mode = if max_for_display >= high_density_red {
                    TerrainLevelMode::Caution
                } else {
                    TerrainLevelMode::Warning
                };

                (min_mode, max_mode)
            } else {
                (TerrainLevelMode::PeaksMode, TerrainLevelMode::PeaksMode)
            }
        };

        let nd_range = self.display_rendering.get(&side)
            .map(|s| s.navigation_display.display_configuration().nd_range as f64)
            .unwrap_or(10.0);

        let first_frame = self.display_rendering.get(&side)
            .map(|s| s.navigation_display.first_frame())
            .unwrap_or(true);

        let efis_mode = self.display_rendering.get(&side)
            .map(|s| s.navigation_display.display_configuration().efis_mode)
            .unwrap_or(0);

        // Note: frame_byte_count is set to raw size here; caller updates after PNG encoding
        let metadata = crate::types::NavigationDisplayData {
            minimum_elevation: min_for_display as i16,
            minimum_elevation_mode: min_mode,
            maximum_elevation: max_for_display as i16,
            maximum_elevation_mode: max_mode,
            first_frame,
            display_range: nd_range,
            display_mode: efis_mode,
            frame_byte_count: 0, // Will be set by caller after PNG encoding
        };

        // Mark first frame as false
        if let Some(state) = self.display_rendering.get_mut(&side) {
            state.navigation_display.set_first_frame(false);
        }

        Some((metadata, frame, width, height))
    }

    /// Render a navigation display frame and return both metadata and frame data
    /// This is used by the SimConnect handler to send data back to the simulator
    pub fn render_frame_for_simconnect(&mut self, side: DisplaySide) -> Option<(crate::types::NavigationDisplayData, Vec<u8>)> {
        if !self.initialized {
            debug!("render_frame_for_simconnect: not initialized");
            return None;
        }

        // Check if terrain display is enabled
        {
            let state = match self.display_rendering.get(&side) {
                Some(s) => s,
                None => {
                    warn!("No display rendering state for {:?}", side);
                    return None;
                }
            };
            let nd_config = state.navigation_display.display_configuration();

            debug!("render_frame_for_simconnect({:?}): terr_on_nd={}, terr_on_vd={}",
                side, nd_config.terr_on_nd, nd_config.terr_on_vd);

            if !nd_config.terr_on_nd && !nd_config.terr_on_vd {
                debug!("Terrain display not enabled for {:?}", side);
                return None;
            }
        }

        // Render the frame - returns (frame, min_for_display, max_for_display, is_normal_mode)
        // min_for_display is the lowDensityGreen threshold (what TypeScript displays as MinimumElevation)
        // max_for_display is the actual maximum terrain elevation
        let (frame, min_for_display, max_for_display, is_normal_mode) = self.render_navigation_display_frame_with_stats(side)?;

        // Calculate elevation modes based on thresholds and aircraft altitude
        let (min_mode, max_mode) = {
            let status = self.aircraft_status.as_ref()?;
            let gear_offset = if status.gear_is_down { 250 } else { 500 };

            // Calculate reference altitude with vertical speed prediction (like TypeScript)
            let reference_altitude = if status.vertical_speed <= -1000 {
                status.altitude + (status.vertical_speed as i32 / 2)
            } else {
                status.altitude
            };

            // Thresholds (from TypeScript)
            const HIGH_DENSITY_GREEN_OFFSET: i32 = 1000;
            const HIGH_DENSITY_RED_OFFSET: i32 = 2000;

            if is_normal_mode {
                // Normal mode threshold calculations
                let high_density_green = reference_altitude - HIGH_DENSITY_GREEN_OFFSET;
                let low_density_yellow = reference_altitude - gear_offset;
                let high_density_red = reference_altitude + HIGH_DENSITY_RED_OFFSET;

                // Mode for min elevation (from TypeScript analyzeMetadata)
                let min_mode = if low_density_yellow <= high_density_green {
                    TerrainLevelMode::Warning
                } else {
                    TerrainLevelMode::PeaksMode
                };

                // Mode for max elevation
                let max_mode = if max_for_display >= high_density_red {
                    TerrainLevelMode::Caution
                } else {
                    TerrainLevelMode::Warning
                };

                (min_mode, max_mode)
            } else {
                // Peaks mode - both are PeaksMode
                (TerrainLevelMode::PeaksMode, TerrainLevelMode::PeaksMode)
            }
        };

        // Build metadata with actual elevation values
        let nd_range = self.display_rendering.get(&side)
            .map(|s| s.navigation_display.display_configuration().nd_range as f64)
            .unwrap_or(10.0);

        let first_frame = self.display_rendering.get(&side)
            .map(|s| s.navigation_display.first_frame())
            .unwrap_or(true);

        // Get efisMode from the display configuration - this must match what the aircraft sends
        let efis_mode = self.display_rendering.get(&side)
            .map(|s| s.navigation_display.display_configuration().efis_mode)
            .unwrap_or(0);

        let metadata = crate::types::NavigationDisplayData {
            minimum_elevation: min_for_display as i16,
            minimum_elevation_mode: min_mode,
            maximum_elevation: max_for_display as i16,
            maximum_elevation_mode: max_mode,
            first_frame,
            display_range: nd_range,
            display_mode: efis_mode,  // Must match aircraft's efisMode, not just arc/rose
            frame_byte_count: frame.len() as u32,
        };

        // Mark first frame as false for subsequent frames
        if let Some(state) = self.display_rendering.get_mut(&side) {
            state.navigation_display.set_first_frame(false);
        }

        Some((metadata, frame))
    }

    /// Get the current rendering mode
    pub fn get_rendering_mode(&self) -> TerrainRenderingMode {
        self.rendering_mode
    }

    /// Start a new transition cycle for a display side
    /// Called when it's time to refresh the terrain display
    pub fn start_transition_cycle(&mut self, side: DisplaySide, final_frame: Vec<u8>, width: usize, height: usize) {
        if let Some(state) = self.display_rendering.get_mut(&side) {
            let current_time = Instant::now();
            state.navigation_display.start_new_map_cycle(current_time, width, height);
            state.navigation_display.set_final_frame(final_frame);
            debug!("Started ND transition cycle for {:?} with dimensions {}x{}", side, width, height);
        }
    }

    /// Start a new transition cycle for the vertical display
    pub fn start_vd_transition_cycle(&mut self, side: DisplaySide, final_frame: Vec<u8>) {
        if let Some(state) = self.display_rendering.get_mut(&side) {
            let current_time = Instant::now();
            state.vertical_display.start_new_map_cycle(current_time);
            state.vertical_display.set_final_frame(final_frame);
            debug!("Started VD transition cycle for {:?} with dimensions {}x{}", 
                side, RENDERING_ELEVATION_PROFILE_WIDTH, RENDERING_ELEVATION_PROFILE_HEIGHT);
        }
    }

    /// Tick the transition animation for a display side (ND only)
    /// Returns true when transition is complete
    pub fn tick_transition(&mut self, side: DisplaySide) -> bool {
        if let Some(state) = self.display_rendering.get_mut(&side) {
            state.navigation_display.render()
        } else {
            true // No state = consider complete
        }
    }

    /// Tick the VD transition animation for a display side
    /// Returns true when transition is complete
    pub fn tick_vd_transition(&mut self, side: DisplaySide) -> bool {
        if let Some(state) = self.display_rendering.get_mut(&side) {
            state.vertical_display.render()
        } else {
            true // No state = consider complete
        }
    }

    /// Get the current transition frame for a display side
    pub fn get_current_transition_frame(&self, side: DisplaySide) -> Option<Vec<u8>> {
        self.display_rendering.get(&side)
            .and_then(|state| state.navigation_display.current_frame())
            .cloned()
    }

    /// Get the current VD transition frame for a display side
    pub fn get_current_vd_transition_frame(&self, side: DisplaySide) -> Option<Vec<u8>> {
        self.display_rendering.get(&side)
            .and_then(|state| state.vertical_display.current_frame())
            .cloned()
    }

    /// Composite the VD transition frame onto the ND transition frame
    /// The VD is overlaid at (VERTICAL_DISPLAY_MAP_START_OFFSET_X, VERTICAL_DISPLAY_MAP_START_OFFSET_Y)
    pub fn composite_vd_onto_nd(&self, nd_frame: &mut [u8], nd_width: usize, vd_frame: &[u8]) {
        let vd_width = RENDERING_ELEVATION_PROFILE_WIDTH;
        let vd_height = RENDERING_ELEVATION_PROFILE_HEIGHT;

        for y in 0..vd_height {
            for x in 0..vd_width {
                let nd_x = VERTICAL_DISPLAY_MAP_START_OFFSET_X + x;
                let nd_y = VERTICAL_DISPLAY_MAP_START_OFFSET_Y + y;

                let nd_idx = (nd_y * nd_width + nd_x) * RENDERING_COLOR_CHANNEL_COUNT;
                let vd_idx = (y * vd_width + x) * RENDERING_COLOR_CHANNEL_COUNT;

                if nd_idx + 3 >= nd_frame.len() || vd_idx + 3 >= vd_frame.len() {
                    continue;
                }

                // Alpha blending: VD pixels with alpha > 0 replace ND pixels
                let vd_alpha = vd_frame[vd_idx + 3];
                if vd_alpha > 0 {
                    nd_frame[nd_idx] = vd_frame[vd_idx];
                    nd_frame[nd_idx + 1] = vd_frame[vd_idx + 1];
                    nd_frame[nd_idx + 2] = vd_frame[vd_idx + 2];
                    nd_frame[nd_idx + 3] = vd_frame[vd_idx + 3];
                }
            }
        }
    }

    /// Render the raw VD frame for transition (without compositing onto ND)
    pub fn render_raw_vd_frame(&self, side: DisplaySide) -> Option<Vec<u8>> {
        if !self.initialized {
            return None;
        }

        // Check if VD terrain is enabled
        let config = self.display_rendering.get(&side)?
            .navigation_display.display_configuration().clone();

        if !config.terr_on_vd {
            return None;
        }

        // Need aircraft status for heading
        let status = self.aircraft_status.as_ref()?;

        // Render VD terrain to its own buffer
        self.render_vertical_display_raw(side, status)
    }

    /// Check if VD terrain is enabled for a side
    pub fn is_vd_enabled(&self, side: DisplaySide) -> bool {
        self.display_rendering.get(&side)
            .map(|s| s.navigation_display.display_configuration().terr_on_vd)
            .unwrap_or(false)
    }

    /// Check if VD terrain should be rendered (enabled and required)
    pub fn should_render_vd(&self, side: DisplaySide) -> bool {
        self.vertical_display_required && self.is_vd_enabled(side)
    }

    /// Check if a display side needs a new render cycle immediately
    /// (e.g., after config change when frames were invalidated)
    pub fn needs_new_cycle(&self, side: DisplaySide) -> bool {
        self.display_rendering.get(&side)
            .map(|state| state.navigation_display.needs_new_cycle())
            .unwrap_or(false)
    }
}
