//! Terrain processing module
//!
//! This module handles terrain rendering for both navigation and vertical displays.


mod renderer;
mod patterns;

/// Timeout for SimBridge client data (2 minutes in milliseconds)
const SIMBRIDGE_CLIENT_DATA_TIMEOUT_MS: u128 = 2 * 60 * 1000;

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
    /// Flag indicating if web service is providing aircraft status (vs SimConnect)
    simbridge_client_used: bool,
    /// Timestamp of last web service aircraft status update
    last_web_update: Option<Instant>,
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
    low_density_green: f64,
    /// High density green threshold
    high_density_green: f64,
    /// Low density yellow threshold
    low_density_yellow: f64,
    /// High density yellow threshold
    high_density_yellow: f64,
    /// High density red threshold
    high_density_red: f64,
    /// Cutoff altitude (terrain below this is not rendered)
    cutoff_altitude: f64,
    /// Reference altitude (aircraft altitude with vertical speed prediction)
    reference_altitude: f64,
    /// Whether we're in normal mode (vs peaks mode)
    use_normal_mode: bool,
    /// Raw min elevation from terrain data
    min_elevation: f64,
    /// Raw max elevation from terrain data
    max_elevation: f64,
    /// Lower percentile elevation (85th percentile)
    lower_percentile_elev: f64,
    /// Upper percentile elevation (95th percentile)
    upper_percentile_elev: f64,
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
            simbridge_client_used: false,
            last_web_update: None,
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
            simbridge_client_used: false,
            last_web_update: None,
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

    /// Enable SimBridge client data mode (web service providing aircraft status)
    /// This disables SimConnect aircraft status updates
    pub fn enable_simbridge_client_data(&mut self) {
        if !self.simbridge_client_used {
            info!("SimBridge client data received, ignoring SimConnect aircraftStatusUpdate from now on.");
        }
        self.simbridge_client_used = true;
        self.last_web_update = Some(Instant::now());
    }

    /// Disable SimBridge client data mode (resume SimConnect aircraft status updates)
    pub fn disable_simbridge_client_data(&mut self) {
        if self.simbridge_client_used {
            info!("SimBridge client data stopped (due to timeout), resuming SimConnect aircraftStatusUpdate.");
        }
        self.simbridge_client_used = false;
        self.last_web_update = None;
    }

    /// Check if SimConnect aircraft status updates should be processed
    /// Returns false if web service is providing updates (and not timed out)
    pub fn should_use_simconnect_status(&mut self) -> bool {
        if !self.simbridge_client_used {
            return true;
        }

        // Check for timeout
        if let Some(last_update) = self.last_web_update {
            if last_update.elapsed().as_millis() > SIMBRIDGE_CLIENT_DATA_TIMEOUT_MS {
                self.disable_simbridge_client_data();
                return true;
            }
        }

        false
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
        self.manual_azim_degrees = status.manual_azim_degrees;

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
    fn render_raw_frame_with_stats(&mut self, side: DisplaySide) -> Option<(Vec<u8>, f64, f64, bool, usize, usize)> {
        let state = self.display_rendering.get(&side)?;
        let config = state.navigation_display.display_configuration();

        use std::time::{SystemTime, UNIX_EPOCH};
     //   let now = SystemTime::now().duration_since(UNIX_EPOCH).unwrap().as_millis();

        if !config.terr_on_nd && !config.terr_on_vd && !self.vertical_display_required
        {
        //    debug!("Terrain not enabled for {:?}, skipping render", side);
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
            (0.0, 0.0, false)
        };

        // NOTE: Vertical display is rendered separately with its own transition
        // It will be composited onto the frame after both transitions are applied

        Some((frame, min_elev, max_elev, is_normal_mode, display_width, display_height))
    }

    fn render_navigation_display_frame_with_stats(&mut self, side: DisplaySide) -> Option<(Vec<u8>, f64, f64, bool)> {
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

    /// Calculate distance between two WGS84 coordinates in nautical miles
    fn distance_wgs84(lat1: f64, lon1: f64, lat2: f64, lon2: f64) -> f64 {
        const EARTH_RADIUS_NM: f64 = 3440.065; // Earth radius in nautical miles

        let lat1_rad = lat1.to_radians();
        let lat2_rad = lat2.to_radians();
        let delta_lat = (lat2 - lat1).to_radians();
        let delta_lon = (lon2 - lon1).to_radians();

        let a = (delta_lat / 2.0).sin().powi(2)
            + lat1_rad.cos() * lat2_rad.cos() * (delta_lon / 2.0).sin().powi(2);
        let c = 2.0 * a.sqrt().atan2((1.0 - a).sqrt());

        EARTH_RADIUS_NM * c
    }

    /// Extract elevation at a specific coordinate from cached data
    fn extract_elevation(&self, latitude: f64, longitude: f64) -> Option<f32> {
        let cached_data = self.cached_elevation_data.as_ref()?;
        let metadata = &self.world_map_metadata;

        if metadata.width == 0 || metadata.height == 0 {
            return None;
        }

        // Check if coordinate is within bounds
        if latitude < metadata.southwest.latitude || latitude > metadata.northeast.latitude ||
           longitude < metadata.southwest.longitude || longitude > metadata.northeast.longitude {
            return None;
        }

        let lat_step = (metadata.northeast.latitude - metadata.southwest.latitude) / metadata.height as f64;
        let lon_step = (metadata.northeast.longitude - metadata.southwest.longitude) / metadata.width as f64;

        let sample_x = ((longitude - metadata.southwest.longitude) / lon_step) as usize;
        let sample_y = ((metadata.northeast.latitude - latitude) / lat_step) as usize;

        if sample_x < metadata.width && sample_y < metadata.height {
            let idx = sample_y * metadata.width + sample_x;
            if idx < cached_data.data.len() {
                return Some(cached_data.data[idx]);
            }
        }

        None
    }

    /// Calculate the absolute cutoff altitude based on runway proximity and glide slope
    /// This matches TypeScript's calculateAbsoluteCutOffAltitude()
    fn calculate_absolute_cutoff_altitude(&self, status: &AircraftStatus) -> f64 {
        // If no runway data is valid, return histogram minimum
        if !status.runway_data_valid {
            return HISTOGRAM_MINIMUM_ELEVATION as f64;
        }

        // Get destination (runway) elevation
        let destination_elevation = match self.extract_elevation(status.runway_latitude, status.runway_longitude) {
            Some(elev) if elev != INVALID_ELEVATION as f32 => elev as f64,
            _ => return HISTOGRAM_MINIMUM_ELEVATION as f64,
        };

        // Calculate distance to runway in nautical miles
        let distance = Self::distance_wgs84(
            status.latitude,
            status.longitude,
            status.runway_latitude,
            status.runway_longitude,
        );

        // Only apply cutoff logic within RENDERING_MAX_AIRPORT_DISTANCE (4 NM)
        if distance > RENDERING_MAX_AIRPORT_DISTANCE {
            return HISTOGRAM_MINIMUM_ELEVATION as f64;
        }

        let mut cutoff_altitude = RENDERING_CUT_OFF_ALTITUDE_MAXIMUM as f64;
        let distance_feet = distance * FEET_PER_NAUTICAL_MILE;

        // Calculate the glide until touchdown
        let opposite = status.altitude - destination_elevation;
        let mut glide_radian = 0.0;
        if opposite > 0.0 && distance > 0.0 {
            // Calculate the glide slope: opposite [ft] -> distance needs to be converted to feet
            glide_radian = (opposite / distance_feet).atan();
        }

        // Check if the glide is less than 3° (0.0523599 radians)
        if glide_radian < 0.0523599 {
            if distance <= 1.0 || glide_radian == 0.0 {
                // Use the minimum value close to the airport
                cutoff_altitude = RENDERING_CUT_OFF_ALTITUDE_MINIMUM as f64;
            } else {
                // Use a linear model from max to min for 4 nm to 1 nm
                let slope = (RENDERING_CUT_OFF_ALTITUDE_MINIMUM - RENDERING_CUT_OFF_ALTITUDE_MAXIMUM) as f64
                    / THREE_NAUTICAL_MILES_IN_FEET;
                cutoff_altitude = (slope * (distance_feet - FEET_PER_NAUTICAL_MILE)
                    + RENDERING_CUT_OFF_ALTITUDE_MAXIMUM as f64).round();

                // Ensure we are not below the minimum and not above the maximum
                cutoff_altitude = cutoff_altitude.max(RENDERING_CUT_OFF_ALTITUDE_MINIMUM as f64);
                cutoff_altitude = cutoff_altitude.min(RENDERING_CUT_OFF_ALTITUDE_MAXIMUM as f64);
            }
        }

        cutoff_altitude
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
    ) -> (f64, f64, bool) {
        let cached_data = match &self.cached_elevation_data {
            Some(data) => data,
            None => {
                warn!("render_terrain_to_frame: no cached elevation data");
                return (0.0, 0.0, false);
            }
        };

        let metadata = &self.world_map_metadata;
        if metadata.width == 0 || metadata.height == 0 {
            warn!("render_terrain_to_frame: invalid world map metadata ({}x{})", metadata.width, metadata.height);
            return (0.0, 0.0, false);
        }

        let config = self.display_rendering.get(&side)
            .map(|s| s.navigation_display.display_configuration().clone())
            .unwrap_or_default();

        let offset_x = config.map_offset_x.unwrap_or(0) as usize;
        let center_offset_y = config.center_offset_y.unwrap_or(0) as f64;
        let nd_range = config.nd_range as f64; // in nautical miles
        let arc_mode = config.arc_mode;

        info!("render_terrain_to_frame: offset_y={}, map_width={}, map_height={}, offset_x={}, center_offset_y={}, arc_mode={}, rendering_mode={:?}",
            offset_y, map_width, map_height, offset_x, center_offset_y, arc_mode, self.rendering_mode);

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
        let heading = status.heading;

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

        // Elevation samples for histogram
        let mut elevation_samples: Vec<i16> = Vec::new();

        // First pass: calculate min/max elevation in visible area for mode selection
        let mut min_elevation: i32 = i32::MAX;
        let mut max_elevation: i32 = i32::MIN;

        for y in 0..map_height {
            for x in 0..map_width {
                let delta_x = x as f64 - center_x;
                let delta_y = (map_height as f64) - (y as f64) - center_offset_y;

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

        for &elev in &elevation_samples {
            let elev_i32 = elev as i32;
            // TypeScript uses Math.ceil for binning, so we need to round up
            // ceil((elevation - minElevation) / binRange)
            let adjusted = elev_i32 - HISTOGRAM_MINIMUM_ELEVATION;
            let bin = ((adjusted + HISTOGRAM_BIN_RANGE - 1) / HISTOGRAM_BIN_RANGE) as usize; // Ceiling division
            let bin = bin.min(histogram.len() - 1); // Clamp to valid range
            histogram[bin] += 1;
        }

        // Calculate cutoff altitude based on runway proximity (like TypeScript calculateAbsoluteCutOffAltitude)
        let cutoff_altitude = self.calculate_absolute_cutoff_altitude(status);
        // TypeScript uses Math.floor for cutoff bin (not ceil like histogram binning)
        let cutoff_bin = ((cutoff_altitude - HISTOGRAM_MINIMUM_ELEVATION as f64) / HISTOGRAM_BIN_RANGE as f64).floor().max(0.0) as usize;

        // Calculate total frequency starting from cutoff bin (like TypeScript)
        let mut total_samples = 0u32;
        for bin in cutoff_bin..histogram.len() {
            total_samples += histogram[bin];
        }

        // Find min/max bins and percentiles starting from cutoff bin (like TypeScript)
        let mut min_bin: i32 = -1;
        let mut max_bin: i32 = -1;
        let mut lower_percentile_bin: i32 = -1;
        let mut upper_percentile_bin: i32 = -1;
        let mut cumulative_percent = 0.0f64;

        for bin in cutoff_bin..histogram.len() {
            if total_samples > 0 {
                cumulative_percent += histogram[bin] as f64 / total_samples as f64;
                if lower_percentile_bin < 0 && cumulative_percent >= RENDERING_LOWER_PERCENTILE {
                    lower_percentile_bin = bin as i32;
                }
                if upper_percentile_bin < 0 && cumulative_percent >= RENDERING_UPPER_PERCENTILE {
                    upper_percentile_bin = bin as i32;
                }
            }

            if histogram[bin] > 0 {
                if min_bin < 0 { min_bin = bin as i32; }
                max_bin = bin as i32;
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
        let min_elevation = min_elevation_binned as f64;
        let max_elevation = max_elevation_binned as f64;

        // Calculate reference altitude with vertical speed prediction (like TypeScript)
        let reference_altitude = if status.vertical_speed <= -1000.0 {
            status.altitude + (status.vertical_speed / 2.0)
        } else {
            status.altitude
        };

        let gear_offset = if status.gear_is_down { 250.0 } else { 500.0 };
        let use_normal_mode = max_elevation >= reference_altitude - gear_offset;

        // Calculate thresholds like TypeScript
        // These are the same values used in elevation_to_color_with_thresholds
        const LOW_DENSITY_GREEN_OFFSET: f64 = 2000. ;
        const HIGH_DENSITY_GREEN_OFFSET: f64 = 1000.;
        const HIGH_DENSITY_YELLOW_OFFSET: f64 = 1000.;
        const HIGH_DENSITY_RED_OFFSET: f64 = 2000.;
        const FLAT_EARTH_THRESHOLD: f64 = 100.;
        // Calculate flatEarth like TypeScript: flatEarthThreshold - (maxElevation - minElevation)
        let flat_earth = FLAT_EARTH_THRESHOLD - (max_elevation - min_elevation);
        let half_elevation = (max_elevation as f64 * 0.5);

        // Calculate green thresholds (from calculateNormalModeGreenThresholds)
        let mut low_density_green = if reference_altitude - LOW_DENSITY_GREEN_OFFSET <= min_elevation {
            min_elevation + 200.
        } else {
            reference_altitude - LOW_DENSITY_GREEN_OFFSET
        };

        let high_density_green = if reference_altitude - HIGH_DENSITY_GREEN_OFFSET <= min_elevation {
            min_elevation + 200.
        } else {
            reference_altitude - HIGH_DENSITY_GREEN_OFFSET
        };

        // Apply flatEarth adjustments like TypeScript using actual percentile values
        // TypeScript: lowerPercentile is the elevation at 85th percentile of terrain
        if flat_earth >= 0. {
            if half_elevation <= lower_percentile_elev as f64&& low_density_green > half_elevation {
                low_density_green = half_elevation;
            } else if half_elevation > lower_percentile_elev as f64 && low_density_green > lower_percentile_elev as f64{
                low_density_green = lower_percentile_elev as f64;
            }
        }

        // Warning thresholds
        let low_density_yellow = if reference_altitude - gear_offset <= min_elevation as f64 {
            min_elevation as f64 + 200.
        } else {
            reference_altitude - gear_offset
        };
        let high_density_yellow = reference_altitude + HIGH_DENSITY_YELLOW_OFFSET as f64;
        let high_density_red = reference_altitude + HIGH_DENSITY_RED_OFFSET as f64;

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
            lower_percentile_elev: lower_percentile_elev as f64,
            upper_percentile_elev: upper_percentile_elev as f64,
        };

        debug!("Thresholds: low_green={}, high_green={}, low_yellow={}, high_yellow={}, high_red={}",
            thresholds.low_density_green, thresholds.high_density_green, thresholds.low_density_yellow,
            thresholds.high_density_yellow, thresholds.high_density_red);
        debug!("Elevation range: {} to {} ft, aircraft: {} ft (ref: {}), cutoff: {} ft, using {} mode",
            thresholds.min_elevation, thresholds.max_elevation, status.altitude, thresholds.reference_altitude,
            thresholds.cutoff_altitude, if thresholds.use_normal_mode { "NORMAL" } else { "PEAKS" });

        // Second pass: Render each pixel
        for y in 0..map_height {
            for x in 0..map_width {
                let pixel_x = offset_x + x;
                let pixel_y = offset_y + y;

                // Calculate distance and bearing from aircraft to this pixel
                let delta_x = x as f64 - center_x;
                let delta_y = (map_height as f64) - (y as f64) - center_offset_y;

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
        let min_bin = ((min_raw - HISTOGRAM_MINIMUM_ELEVATION as f64) / HISTOGRAM_BIN_RANGE as f64).floor();
        let min_for_display = min_bin * HISTOGRAM_BIN_RANGE as f64 + HISTOGRAM_MINIMUM_ELEVATION as f64;

        // max: ceil to next bin boundary (bin + 1)
        let max_bin = ((thresholds.max_elevation - HISTOGRAM_MINIMUM_ELEVATION as f64) / HISTOGRAM_BIN_RANGE as f64).floor();
        let max_for_display = (max_bin + 1.) * HISTOGRAM_BIN_RANGE as f64 + HISTOGRAM_MINIMUM_ELEVATION as f64;

        (min_for_display, max_for_display, thresholds.use_normal_mode)
    }

    /// Render vertical display terrain to a separate buffer for transition processing
    /// Returns a buffer sized for the vertical display (RENDERING_ELEVATION_PROFILE_WIDTH x RENDERING_ELEVATION_PROFILE_HEIGHT)
    fn render_vertical_display_raw(
        &self,
        side: DisplaySide,
        status: &AircraftStatus,
    ) -> Option<Vec<u8>> {
        let metadata = &self.world_map_metadata;
        if metadata.width == 0 || metadata.height == 0 {
            debug!("render_vertical_display_raw: invalid world map metadata");
            return None;
        }

        // Get vertical display state
        let vd_state = match self.display_rendering.get(&side) {
            Some(state) => &state.vertical_display,
            None => {
                debug!("render_vertical_display_raw: no display state for {:?}", side);
                return None;
            }
        };

        let vd_config = vd_state.display_configuration().clone();
        let waypoints_lat = vd_state.waypoints_latitudes().to_vec();
        let waypoints_lon = vd_state.waypoints_longitudes().to_vec();
        let path_width = vd_state.path_width();
        let fms_path_used = vd_state.fms_path_used();
        let track_changes_distance = vd_state.track_changes_significantly_at_distance();
        let elevation_range_nm = vd_state.elevation_range();

        // Get navigation display config for arc mode
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
        let profile_range_nm = if elevation_range_nm > 0.0 {
            elevation_range_nm
        } else if nd_config.arc_mode {
            nd_config.nd_range.max(10).min(160) as f64
        } else {
            (nd_config.nd_range / 2).max(5).min(160) as f64
        };

        // Create elevation profile - use waypoints if FMS path is active, otherwise use heading
        let elevation_profile = if fms_path_used && !waypoints_lat.is_empty() {
            self.create_elevation_profile_along_waypoints(
                status.latitude,
                status.longitude,
                &waypoints_lat,
                &waypoints_lon,
                path_width,
                profile_range_nm,
                vd_width,
            )
        } else {
            self.create_elevation_profile(
                status.latitude,
                status.longitude,
                status.heading as f64,
                profile_range_nm,
                vd_width,
            )
        };

        // Calculate grey area starting X position (where track changes significantly)
        let grey_area_starts_at_x: i32 = if track_changes_distance >= 0.0 && fms_path_used {
            // Convert distance to pixel X coordinate
            ((track_changes_distance / profile_range_nm) * vd_width as f64) as i32
        } else {
            -1
        };

        // Create buffer for VD
        let mut buffer = vec![0u8; vd_width * vd_height * RENDERING_COLOR_CHANNEL_COUNT];

        // Render the vertical display
        let altitude_range = (max_altitude - min_altitude) as f64;
        let altitude_step = altitude_range / vd_height as f64;

        for y in 0..vd_height {
            // Altitude at this row (top = max, bottom = min)
            let altitude = (vd_height - y) as f64 * altitude_step + min_altitude;

            for x in 0..vd_width {
                let elevation = elevation_profile[x];

                // Calculate pixel position in the buffer
                let buf_idx = (y * vd_width + x) * RENDERING_COLOR_CHANNEL_COUNT;

                // Determine color based on elevation vs altitude
                let (r, g, b, a) = if elevation == INVALID_ELEVATION as f32 || elevation == UNKNOWN_ELEVATION as f32 {
                    // Unknown/invalid - magenta (like TypeScript: 255, 148, 255)
                    (255u8, 148u8, 255u8, 255u8)
                } else if altitude > elevation as f64 {
                    // Above terrain - check if in grey area or transparent
                    if grey_area_starts_at_x >= 0 && (x as i32) >= grey_area_starts_at_x {
                        // Grey background (like TypeScript: 78, 78, 97)
                        (78u8, 78u8, 97u8, 255u8)
                    } else {
                        // Transparent background
                        (0u8, 0u8, 0u8, 0u8)
                    }
                } else if elevation == WATER_ELEVATION as f32 {
                    // Water - cyan if at/below sea level (like TypeScript: 0, 255, 255)
                    if altitude <= 0.0 {
                        (0u8, 255u8, 255u8, 255u8)
                    } else if grey_area_starts_at_x >= 0 && (x as i32) >= grey_area_starts_at_x {
                        // Grey background above water in grey area
                        (78u8, 78u8, 97u8, 255u8)
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

        debug!("Vertical display raw rendering complete for {:?}, fms_path={}, waypoints={}, grey_area_x={}", 
               side, fms_path_used, waypoints_lat.len(), grey_area_starts_at_x);
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

    /// Create an elevation profile along waypoints (FMS path) or heading (manual azimuth)
    /// Uses a corridor/hose around the path and returns the maximum elevation in the corridor
    /// This matches the TypeScript createElevationProfile GPU kernel behavior
    fn create_elevation_profile_along_waypoints(
        &self,
        latitude: f64,
        longitude: f64,
        waypoints_lat: &[f64],
        waypoints_lon: &[f64],
        path_width_nm: f64,
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

        let waypoint_count = waypoints_lat.len();
        if waypoint_count == 0 {
            return profile;
        }

        // Calculate distance per pixel in nautical miles
        let distance_per_pixel_nm = range_nm / profile_width as f64;
        
        // Path offset in metres (half-width of corridor)
        let offset_meters = (path_width_nm * NAUTICAL_MILES_TO_METRES) / 2.0;

        // Calculate lat/lon steps for the world map
        let lat_step = (metadata.northeast.latitude - metadata.southwest.latitude) / metadata.height as f64;
        let lon_step = (metadata.northeast.longitude - metadata.southwest.longitude) / metadata.width as f64;

        for x in 0..profile_width {
            let distance_for_pixel_nm = distance_per_pixel_nm * x as f64;
            
            // Find the correct waypoint segment for this distance
            let mut route_segment_index = waypoint_count;
            let mut route_start_point_distance_nm = 0.0;
            let mut start_latitude = latitude;
            let mut start_longitude = longitude;

            for i in 0..waypoint_count {
                let current_distance_nm = Self::distance_wgs84_nm(
                    start_latitude, start_longitude,
                    waypoints_lat[i], waypoints_lon[i]
                );
                
                if route_start_point_distance_nm + current_distance_nm >= distance_for_pixel_nm {
                    route_segment_index = i;
                    break;
                }

                route_start_point_distance_nm += current_distance_nm;
                start_latitude = waypoints_lat[i];
                start_longitude = waypoints_lon[i];
            }

            // Check if we exceeded the waypoints
            if route_segment_index >= waypoint_count {
                // Leave as invalid - beyond the route
                continue;
            }

            // Get the required projection of latitude, longitude
            let remaining_distance_m = (distance_for_pixel_nm - route_start_point_distance_nm) * NAUTICAL_MILES_TO_METRES;
            let bearing = Self::bearing_wgs84(
                start_latitude, start_longitude,
                waypoints_lat[route_segment_index], waypoints_lon[route_segment_index]
            );
            
            let (center_lat, center_lon) = Self::project_wgs84(
                start_latitude, start_longitude, bearing, remaining_distance_m
            );

            // Calculate perpendicular bearings for corridor sampling
            let mut bearing_start = bearing - 90.0;
            if bearing_start < 0.0 { bearing_start += 360.0; }
            let mut bearing_end = bearing + 90.0;
            if bearing_end >= 360.0 { bearing_end -= 360.0; }

            // Project to corridor endpoints
            let (start_lat, start_lon) = Self::project_wgs84(center_lat, center_lon, bearing_start, offset_meters);
            let (end_lat, end_lon) = Self::project_wgs84(center_lat, center_lon, bearing_end, offset_meters);

            // Convert to pixel coordinates
            let start_pixel = self.wgs84_to_pixel(start_lat, start_lon, lat_step, lon_step);
            let end_pixel = self.wgs84_to_pixel(end_lat, end_lon, lat_step, lon_step);

            // Use Bresenham line algorithm to find max elevation along corridor
            let max_elevation = self.sample_line_max_elevation(
                cached_data,
                start_pixel,
                end_pixel,
            );

            profile[x] = max_elevation;
        }

        profile
    }

    /// Create an elevation profile along the aircraft heading (fallback for manual azimuth)
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

    /// Convert WGS84 coordinates to pixel coordinates in cached elevation data
    fn wgs84_to_pixel(&self, lat: f64, lon: f64, lat_step: f64, lon_step: f64) -> (i32, i32) {
        let metadata = &self.world_map_metadata;
        let x = ((lon - metadata.southwest.longitude) / lon_step) as i32;
        let y = ((metadata.northeast.latitude - lat) / lat_step) as i32;
        (x, y)
    }

    /// Sample a line using Bresenham algorithm and return max elevation
    fn sample_line_max_elevation(
        &self,
        cached_data: &CachedElevationData,
        start: (i32, i32),
        end: (i32, i32),
    ) -> f32 {
        let metadata = &self.world_map_metadata;
        let mut max_elevation = -1000.0f32;

        let delta_x = (end.0 - start.0).abs();
        let step_x = if start.0 < end.0 { 1 } else { -1 };
        let delta_y = -(end.1 - start.1).abs();
        let step_y = if start.1 < end.1 { 1 } else { -1 };
        let mut error = delta_x + delta_y;

        let mut x = start.0;
        let mut y = start.1;

        loop {
            // Check bounds and sample
            if y >= 0 && (y as usize) < metadata.height && x >= 0 && (x as usize) < metadata.width {
                let idx = (y as usize) * metadata.width + (x as usize);
                if idx < cached_data.data.len() {
                    let elevation = cached_data.data[idx];
                    // Skip invalid and unknown for max calculation
                    if elevation != INVALID_ELEVATION as f32 
                        && elevation != UNKNOWN_ELEVATION as f32 
                        && elevation > max_elevation 
                    {
                        max_elevation = elevation;
                    }
                }
            }

            if x == end.0 && y == end.1 {
                break;
            }

            let error_double = 2 * error;
            if error_double >= delta_y {
                if x == end.0 { break; }
                error += delta_y;
                x += step_x;
            }
            if error_double <= delta_x {
                if y == end.1 { break; }
                error += delta_x;
                y += step_y;
            }
        }

        max_elevation
    }

    /// Calculate distance between two WGS84 points in nautical miles
    fn distance_wgs84_nm(lat1: f64, lon1: f64, lat2: f64, lon2: f64) -> f64 {
        let delta_lat = (lat2 - lat1).to_radians();
        let delta_lon = (lon2 - lon1).to_radians();
        let lat1_rad = lat1.to_radians();
        let lat2_rad = lat2.to_radians();

        let a = 0.5 - delta_lat.cos() * 0.5
            + lat1_rad.cos() * lat2_rad.cos() * (1.0 - delta_lon.cos()) * 0.5;
        
        let distance_metres = 12742020.0 * a.sqrt().asin();
        distance_metres / NAUTICAL_MILES_TO_METRES
    }

    /// Calculate bearing between two WGS84 points in degrees
    fn bearing_wgs84(lat1: f64, lon1: f64, lat2: f64, lon2: f64) -> f64 {
        let start_lat = lat1.to_radians();
        let start_lon = lon1.to_radians();
        let end_lat = lat2.to_radians();
        let end_lon = lon2.to_radians();

        let y = (end_lon - start_lon).sin() * end_lat.cos();
        let x = start_lat.cos() * end_lat.sin()
            - start_lat.sin() * end_lat.cos() * (end_lon - start_lon).cos();
        
        let bearing = y.atan2(x).to_degrees();
        (bearing + 360.0) % 360.0
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

        // Use f64 for all comparisons to avoid rounding issues (like TypeScript)
        let elevation_ft = elevation as f64;

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
                // High density green - terrain close but below warning (pattern index 5)
                (0, 255, 0, 5)
            } else if elevation_ft >= thresholds.low_density_yellow && elevation_ft < thresholds.high_density_yellow {
                // Low density yellow - warning level terrain (pattern index 3)
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
            // Calculate thresholds based on terrain distribution (from calculatePeaksModeThresholds)
            let elevation_range = thresholds.max_elevation - thresholds.min_elevation;
            let half_elevation = (thresholds.max_elevation + thresholds.min_elevation) / 2.0;

            // TypeScript: const lowerDensity = Math.min(lowerPercentile, halfElevation);
            let lower_density = thresholds.lower_percentile_elev.min(half_elevation);
            // TypeScript: let higherDensity = Math.min(upperPercentile, (maximumElevation - minimumElevation) * 0.65 + minimumElevation);
            let mut higher_density = thresholds.upper_percentile_elev.min(thresholds.min_elevation + elevation_range * 0.65);
            // TypeScript: let solidDensity = (maximumElevation - minimumElevation) * 0.95 + minimumElevation;
            let mut solid_density = thresholds.min_elevation + elevation_range * 0.95;

            // TypeScript validation: if thresholds overlap incorrectly, disable higher densities
            if lower_density >= higher_density ||
               lower_density >= solid_density ||
               higher_density >= solid_density ||
               thresholds.lower_percentile_elev >= thresholds.upper_percentile_elev ||
               thresholds.lower_percentile_elev >= solid_density ||
               thresholds.upper_percentile_elev >= solid_density {
                higher_density = thresholds.max_elevation + 100.0;
                solid_density = thresholds.max_elevation + 100.0;
            }

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

        // Check if terrain display is enabled OR if A380X is connected (VD always renders)
        {
            let state = match self.display_rendering.get(&side) {
                Some(s) => s,
                None => {
                    warn!("No display rendering state for {:?}", side);
                    return None;
                }
            };
            let nd_config = state.navigation_display.display_configuration();

            // Continue if ND terrain is on, OR if A380X is connected (VD needs rendering)

            if !nd_config.terr_on_nd && !self.vertical_display_required {
                // FIXME !nd_config.terr_on_vd  check, comes from API http
                debug!("Terrain display not enabled for {:?}", side);
                return None;
            }
        }

        // Render raw RGBA frame
        let (frame, min_for_display, max_for_display, is_normal_mode, width, height) =
            self.render_raw_frame_with_stats(side)?;

        // Calculate elevation modes using adjusted thresholds (like TypeScript analyzeMetadata)
        // min_for_display is already the adjusted lowDensityGreen threshold
        let (min_mode, max_mode) = {
            let status = self.aircraft_status.as_ref()?;
            let gear_offset = if status.gear_is_down { 250.0 } else { 500.0 };

            let reference_altitude = if status.vertical_speed <= -1000.0 {
                status.altitude + (status.vertical_speed / 2.0)
            } else {
                status.altitude
            };

            const HIGH_DENSITY_GREEN_OFFSET: f64 = 1000.0;
            const HIGH_DENSITY_RED_OFFSET: f64 = 2000.0;

            if is_normal_mode {
                // Calculate adjusted thresholds like render_terrain_to_frame does
                // Use min_for_display as the effective minimum elevation (it's the lowDensityGreen)
                let min_elevation = min_for_display;

                let high_density_green = if reference_altitude - HIGH_DENSITY_GREEN_OFFSET <= min_elevation {
                    min_elevation + 200.0
                } else {
                    reference_altitude - HIGH_DENSITY_GREEN_OFFSET
                };

                let low_density_yellow = if reference_altitude - gear_offset <= min_elevation {
                    min_elevation + 200.0
                } else {
                    reference_altitude - gear_offset
                };

                let high_density_red = reference_altitude + HIGH_DENSITY_RED_OFFSET;

                // TypeScript: if (lowDensityYellow <= highDensityGreen) -> Warning (yellow)
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

        use std::time::{SystemTime, UNIX_EPOCH};
        let now = SystemTime::now().duration_since(UNIX_EPOCH).unwrap().as_millis();
        debug!("[VD_RENDER] {}ms: render_raw_vd_frame({:?})", now, side);

        // VD always renders when A380X is connected (vertical_display_required)
        // No need to check terr_on_vd configuration

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

    /// Check if VD terrain should be rendered (A380X connected)
    /// VD always renders when A380X is connected, regardless of terr_on_vd setting
    pub fn should_render_vd(&self, side: DisplaySide) -> bool {
        // FIXME         self.vertical_display_required && self.is_vd_enabled(side)
        self.vertical_display_required
    }

    /// Check if a display side needs a new render cycle immediately
    /// (e.g., after config change when frames were invalidated)
    pub fn needs_new_cycle(&self, side: DisplaySide) -> bool {
        self.display_rendering.get(&side)
            .map(|state| state.navigation_display.needs_new_cycle())
            .unwrap_or(false)
    }
}
