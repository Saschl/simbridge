//! Navigation and Vertical display renderers
//!
//! These renderers implement the transition animation system from the TypeScript version.
//! The key concepts are:
//!
//! - **Arc Mode**: Angular sweep transition from center outward (0° to 90°)
//! - **Scanline Mode**: Vertical sweep transition from top to bottom
//! - **Frame Blending**: During transitions, blend old and new frames based on position
//! - **Frame Validity**: Each rendered frame is valid for a certain duration before refresh

#![allow(dead_code)]
#![allow(unused_variables)]

use std::f64::consts::PI;
use std::time::Instant;

use crate::types::{
    constants::*,
    AircraftStatus, DisplaySide, EfisData, NavigationDisplayData, TerrainLevelMode,
    TerrainRenderingMode, VerticalPathData,
};

/// Background color as u32 (RGBA: 4, 4, 5, 0) - matches TypeScript's 328708
const BACKGROUND_COLOR_U32: u32 = 328708; // (0 << 24) | (5 << 16) | (4 << 8) | 4

/// Navigation display renderer with full transition support
pub struct NavigationDisplayRenderer {
    /// Display configuration
    pub configuration: EfisData,
    /// Rendering data including transition state
    rendering_data: NavigationDisplayRenderingData,
    /// Aircraft status
    aircraft_status: Option<AircraftStatus>,
    /// Current rendering mode (arc or scanline)
    rendering_mode: TerrainRenderingMode,
    /// Startup time
    startup_time: Instant,
}

/// Internal rendering state for navigation display
struct NavigationDisplayRenderingData {
    /// Starting angle/position for transition (degrees for arc, pixels for scanline)
    start_transition_border: i32,
    /// Current transition progress (degrees for arc, pixels for scanline)
    current_transition_border: i32,
    /// Frame counter
    frame_counter: u32,
    /// Display threshold data for SimConnect
    threshold_data: NavigationDisplayData,
    /// Fully rendered new frame (target of transition)
    final_frame: Option<Vec<u8>>,
    /// Previous frame (source of transition)
    last_frame: Option<Vec<u8>>,
    /// Current transitioning frame (blended output)
    current_frame: Option<Vec<u8>>,
    /// How long a frame remains valid (ms)
    frame_validity_duration: u64,
    /// Frame width in pixels (set when final_frame is set)
    frame_width: usize,
    /// Frame height in pixels (set when final_frame is set)
    frame_height: usize,
    /// Flag set when config changes and immediate re-render is needed
    needs_immediate_render: bool,
}

impl NavigationDisplayRenderer {
    /// Create a new navigation display renderer
    pub fn new(startup_time: Instant) -> Self {
        NavigationDisplayRenderer {
            configuration: EfisData::default(),
            rendering_data: NavigationDisplayRenderingData {
                start_transition_border: 0,
                current_transition_border: 0,
                frame_counter: 0,
                threshold_data: NavigationDisplayData::default(),
                final_frame: None,
                last_frame: None,
                current_frame: None,
                frame_validity_duration: RENDERING_MAP_FRAME_VALIDITY_TIME_ARC_MODE,
                frame_width: 0,
                frame_height: 0,
                needs_immediate_render: true, // Start with needing a render
            },
            aircraft_status: None,
            rendering_mode: TerrainRenderingMode::ArcMode,
            startup_time,
        }
    }

    /// Reset all rendering state
    pub fn reset(&mut self) {
        self.rendering_data = NavigationDisplayRenderingData {
            start_transition_border: 0,
            current_transition_border: 0,
            frame_counter: 0,
            threshold_data: NavigationDisplayData {
                minimum_elevation: -1,
                minimum_elevation_mode: TerrainLevelMode::PeaksMode,
                maximum_elevation: -1,
                maximum_elevation_mode: TerrainLevelMode::PeaksMode,
                first_frame: true,
                display_range: 0.0,
                display_mode: 0,
                frame_byte_count: 0,
            },
            final_frame: None,
            last_frame: None,
            current_frame: None,
            frame_validity_duration: self.rendering_data.frame_validity_duration,
            frame_width: 0,
            frame_height: 0,
            needs_immediate_render: false,
        };
    }

    /// Update aircraft status and configure the display
    pub fn aircraft_status_update(
        &mut self,
        status: AircraftStatus,
        side: DisplaySide,
        startup: bool,
    ) {
        // Check if rendering mode changed
        let rendering_mode = if (status.navigation_display_rendering_mode
            & TerrainRenderingMode::ScanlineMode as u8)
            != 0
        {
            TerrainRenderingMode::ScanlineMode
        } else {
            TerrainRenderingMode::ArcMode
        };

        if self.rendering_mode != rendering_mode {
            // Update frame validity duration based on mode
            if rendering_mode == TerrainRenderingMode::ScanlineMode {
                self.rendering_data.frame_validity_duration =
                    RENDERING_MAP_FRAME_VALIDITY_TIME_SCANLINE_MODE;
                if !startup {
                    log::info!("Scanline-mode rendering activated");
                }
            } else {
                self.rendering_data.frame_validity_duration =
                    RENDERING_MAP_FRAME_VALIDITY_TIME_ARC_MODE;
                if !startup {
                    log::info!("ARC-mode rendering activated");
                }
            }
            self.rendering_mode = rendering_mode;
        }

        let config = if side == DisplaySide::Left {
            status.efis_data_capt.clone()
        } else {
            status.efis_data_fo.clone()
        };

        self.configure_navigation_display(config);
        self.aircraft_status = Some(status);
    }

    /// Configure navigation display based on EFIS settings
    fn configure_navigation_display(&mut self, mut config: EfisData) {
        let last_config = &self.configuration;
        let arc_mode_changed = last_config.arc_mode != config.arc_mode;
        let config_changed = last_config.efis_mode != config.efis_mode
            || last_config.nd_range != config.nd_range
            || arc_mode_changed
            || last_config.terr_on_nd != config.terr_on_nd
            || last_config.terr_on_vd != config.terr_on_vd;

        let stop_rendering =
            last_config.terr_on_nd && !config.terr_on_nd || last_config.terr_on_vd && !config.terr_on_vd;
        let start_rendering = config_changed || self.configuration.map_width.is_none();

        // Keep previous map dimensions if they exist
        if self.configuration.map_width.is_some() && !arc_mode_changed {
            config.map_width = self.configuration.map_width;
            config.map_height = self.configuration.map_height;
            config.map_offset_x = self.configuration.map_offset_x;
            config.center_offset_y = self.configuration.center_offset_y;
        } else {
            // Recalculate map dimensions when arc_mode changes or first time
            config.map_width = Some(if config.arc_mode {
                RENDERING_ARC_MODE_PIXEL_WIDTH as u32
            } else {
                RENDERING_ROSE_MODE_PIXEL_WIDTH as u32
            });
            config.map_height = Some(if config.arc_mode {
                NAVIGATION_DISPLAY_ARC_MODE_PIXEL_HEIGHT_A380X as u32
            } else {
                NAVIGATION_DISPLAY_ROSE_MODE_PIXEL_HEIGHT_A380X as u32
            });
            config.map_offset_x = Some(
                ((NAVIGATION_DISPLAY_MAX_PIXEL_WIDTH as i32 - config.map_width.unwrap() as i32)
                    as f64
                    * 0.5)
                    .ceil() as i32,
            );
            config.center_offset_y = Some(if config.arc_mode {
                NAVIGATION_DISPLAY_ARC_MODE_CENTER_OFFSET_Y_A380X
            } else {
                NAVIGATION_DISPLAY_ROSE_MODE_CENTER_OFFSET_Y_A380X
            });

            log::debug!(
                "Map dimensions updated: arc_mode={}, map_size={}x{}, offset_x={}, center_offset_y={}",
                config.arc_mode,
                config.map_width.unwrap(),
                config.map_height.unwrap(),
                config.map_offset_x.unwrap(),
                config.center_offset_y.unwrap()
            );
        }

        if stop_rendering || start_rendering {
            self.rendering_data.threshold_data = NavigationDisplayData {
                minimum_elevation: -1,
                minimum_elevation_mode: TerrainLevelMode::PeaksMode,
                maximum_elevation: -1,
                maximum_elevation_mode: TerrainLevelMode::PeaksMode,
                first_frame: true,
                display_range: 0.0,
                display_mode: 0,
                frame_byte_count: 0,
            };

            // Clear last_frame so the transition shows only the new rendered area
            // (no old data to blend with - like TypeScript behavior on range change)
            self.rendering_data.last_frame = None;
            self.rendering_data.final_frame = None;
            self.rendering_data.current_frame = None;

            // Set explicit flag to trigger immediate new cycle
            self.rendering_data.needs_immediate_render = true;

            log::debug!("Config changed - cleared frames for fresh transition, needs_immediate_render=true");
        }

        self.configuration = config;
    }

    /// Get current display configuration
    pub fn display_configuration(&self) -> &EfisData {
        &self.configuration
    }

    /// Get current display data (thresholds for SimConnect)
    pub fn display_data(&self) -> &NavigationDisplayData {
        &self.rendering_data.threshold_data
    }

    /// Get mutable display data
    pub fn display_data_mut(&mut self) -> &mut NavigationDisplayData {
        &mut self.rendering_data.threshold_data
    }

    /// Check if this is the first frame
    pub fn first_frame(&self) -> bool {
        self.rendering_data.threshold_data.first_frame
    }

    /// Set whether this is the first frame
    pub fn set_first_frame(&mut self, first_frame: bool) {
        self.rendering_data.threshold_data.first_frame = first_frame;
    }

    /// Check if a new render cycle is needed (config changed, frames invalidated)
    pub fn needs_new_cycle(&self) -> bool {
        // Explicit flag set when config changes (range, mode, etc.)
        self.rendering_data.needs_immediate_render
    }

    /// Clear the needs_immediate_render flag (called when cycle starts)
    pub fn clear_needs_immediate_render(&mut self) {
        self.rendering_data.needs_immediate_render = false;
    }

    /// Set the final (fully rendered) frame for transition
    /// Note: frame dimensions should be set via start_new_map_cycle before this
    pub fn set_final_frame(&mut self, frame: Vec<u8>) {
        self.rendering_data.final_frame = Some(frame);
    }

    /// Get the current rendering mode
    pub fn rendering_mode(&self) -> TerrainRenderingMode {
        self.rendering_mode
    }

    /// Get the frame validity duration
    pub fn frame_validity_duration(&self) -> u64 {
        self.rendering_data.frame_validity_duration
    }

    /// Start a new map rendering cycle
    ///
    /// This calculates the starting transition border based on:
    /// - For first frame: Calculate based on time since startup
    /// - For subsequent frames: Start from beginning (0 for arc, mapHeight for scanline)
    pub fn start_new_map_cycle(&mut self, current_time: Instant, frame_width: usize, frame_height: usize) {
        // Store the frame dimensions
        self.rendering_data.frame_width = frame_width;
        self.rendering_data.frame_height = frame_height;

        // Clear the immediate render flag since we're starting a new cycle
        self.rendering_data.needs_immediate_render = false;

        let map_height = frame_height as u32;

        log::debug!(
            "start_new_map_cycle: dims={}x{}, last_frame={}, nd_range={}",
            frame_width, frame_height,
            self.rendering_data.last_frame.as_ref().map(|f| f.len()).unwrap_or(0),
            self.configuration.nd_range
        );

        if self.configuration.nd_range == 0 {
            self.reset();
            return;
        }

        if self.rendering_data.last_frame.is_none() {
            // First frame - calculate starting position based on time since startup
            let time_since_start = current_time.duration_since(self.startup_time).as_millis() as u64;
            let frame_update_count =
                time_since_start as f64 / self.rendering_data.frame_validity_duration as f64;
            let ratio_since_last_frame = frame_update_count - frame_update_count.floor();

            if self.rendering_mode == TerrainRenderingMode::ScanlineMode {
                // Scanline mode: start from height, transition downward
                self.rendering_data.start_transition_border =
                    map_height as i32 - (map_height as f64 * ratio_since_last_frame).floor() as i32;
            } else {
                // Arc mode: start from 0°, transition outward
                self.rendering_data.start_transition_border =
                    (90.0 * ratio_since_last_frame).floor() as i32;
            }
        } else {
            // Subsequent frames - start from beginning
            if self.rendering_mode == TerrainRenderingMode::ScanlineMode {
                self.rendering_data.start_transition_border = map_height as i32;
            } else {
                self.rendering_data.start_transition_border = 0;
            }
        }

        self.rendering_data.current_transition_border = self.rendering_data.start_transition_border;
        self.rendering_data.frame_counter = 0;
    }

    /// Perform one step of the transition animation
    ///
    /// Returns true when transition is complete, false if still in progress
    pub fn render(&mut self) -> bool {
        if self.rendering_mode == TerrainRenderingMode::ScanlineMode {
            self.scanline_mode_transition()
        } else {
            self.arc_mode_transition()
        }
    }

    /// Arc mode transition - angular sweep from center outward
    ///
    /// The transition sweeps from start_transition_border to 90° in angular steps.
    /// Each pixel's angle from center is calculated and compared to the current border.
    fn arc_mode_transition(&mut self) -> bool {
        // Nothing to do if no final frame
        let final_frame = match self.rendering_data.final_frame.as_ref() {
            Some(f) => f,
            None => return true,
        };

        // Use dimensions stored when frame was set (from actual rendered frame)
        let map_width = self.rendering_data.frame_width;
        let map_height = self.rendering_data.frame_height;

        // Debug log last_frame state
       /*  log::debug!(
            "arc_mode_transition: last_frame={}, final_frame={}, dims={}x{}, border={}/{}",
            self.rendering_data.last_frame.as_ref().map(|f| f.len()).unwrap_or(0),
            final_frame.len(),
            map_width, map_height,
            self.rendering_data.start_transition_border,
            self.rendering_data.current_transition_border
        ); */

        // Verify dimensions are valid
        if map_width == 0 || map_height == 0 {
            log::warn!("Frame dimensions not set. Skipping transition.");
            self.rendering_data.current_frame = Some(final_frame.clone());
            self.rendering_data.last_frame = self.rendering_data.current_frame.clone();
            return true;
        }

        // Verify the frame size matches expected dimensions
        let expected_size = map_width * map_height * RENDERING_COLOR_CHANNEL_COUNT;
        if final_frame.len() != expected_size {
            log::warn!(
                "Frame size mismatch: expected {} ({}x{}x4), got {}. Skipping transition.",
                expected_size, map_width, map_height, final_frame.len()
            );
            // Just return the final frame as-is
            self.rendering_data.current_frame = Some(final_frame.clone());
            self.rendering_data.last_frame = self.rendering_data.current_frame.clone();
            return true;
        }

        self.rendering_data.threshold_data.display_range = self.configuration.nd_range as f64;
        self.rendering_data.threshold_data.display_mode = self.configuration.efis_mode;

        // Advance transition by angular step
        self.rendering_data.current_transition_border += RENDERING_MAP_TRANSITION_ANGULAR_STEP;

        // Get the center offset for correct aircraft position (0 for arc mode, 342 for rose mode)
        let center_offset_y = self.configuration.center_offset_y.unwrap_or(0) as usize;

        if self.rendering_data.current_transition_border < 90 {
            // Still transitioning
            self.rendering_data.current_frame = Some(self.arc_mode_transition_frame(
                self.rendering_data.last_frame.as_ref(),
                self.rendering_data.final_frame.as_ref().unwrap(),
                self.rendering_data.start_transition_border,
                self.rendering_data.current_transition_border,
                map_width,
                map_height,
                center_offset_y,
            ));
            return false;
        }

        // Perform the last frame (ensure we reach exactly 90°)
        if self.rendering_data.current_transition_border - RENDERING_MAP_TRANSITION_ANGULAR_STEP < 90
        {
            self.rendering_data.current_frame = Some(self.arc_mode_transition_frame(
                self.rendering_data.last_frame.as_ref(),
                self.rendering_data.final_frame.as_ref().unwrap(),
                self.rendering_data.start_transition_border,
                90,
                map_width,
                map_height,
                center_offset_y,
            ));
        }

        // Save current frame as last frame for next cycle
        self.rendering_data.last_frame = self.rendering_data.current_frame.clone();

        true
    }

    /// Create a transition frame for arc mode
    ///
    /// Blends old and new frames based on the angular position of each pixel.
    /// Pixels within the transition zone (start_angle to end_angle) show the new frame,
    /// pixels outside show the old frame.
    fn arc_mode_transition_frame(
        &self,
        old_frame: Option<&Vec<u8>>,
        new_frame: &[u8],
        start_angle: i32,
        end_angle: i32,
        map_width: usize,
        map_height: usize,
        center_offset_y: usize,
    ) -> Vec<u8> {
        let frame_size = map_width * RENDERING_COLOR_CHANNEL_COUNT * map_height;
        let mut result = vec![0u8; frame_size];

        // Access data as u32 arrays for performance (4 bytes per pixel = 1 u32)
        let result_u32: &mut [u32] = unsafe {
            std::slice::from_raw_parts_mut(result.as_mut_ptr() as *mut u32, frame_size / 4)
        };
        result_u32.fill(BACKGROUND_COLOR_U32);

        let old_source_u32: Option<&[u32]> = old_frame.map(|f| unsafe {
            std::slice::from_raw_parts(f.as_ptr() as *const u32, f.len() / 4)
        });
        let new_source_u32: &[u32] = unsafe {
            std::slice::from_raw_parts(new_frame.as_ptr() as *const u32, new_frame.len() / 4)
        };

        let half_width = map_width as f64 / 2.0;
        // Aircraft center is at map_height - center_offset_y (bottom for arc mode, center for rose mode)
        let aircraft_y = (map_height - center_offset_y) as f64;

        let mut array_index = 0;
        for y in 0..map_height {
            for x in 0..map_width {
                let dx = x as f64 - half_width;
                let dy = aircraft_y - y as f64;
                let distance = (dx * dx + dy * dy).sqrt();

                // Calculate angle in degrees (0° = straight ahead, 90° = sides)
                let angle = if distance > 0.0 {
                    (dy / distance).acos() * (180.0 / PI)
                } else {
                    0.0
                };

                if angle >= start_angle as f64 && angle <= end_angle as f64 {
                    // Within transition zone - show new frame
                    if array_index < new_source_u32.len() {
                        result_u32[array_index] = new_source_u32[array_index];
                    }
                } else if let Some(old_src) = old_source_u32 {
                    // Outside transition zone - show old frame
                    if array_index < old_src.len() {
                        result_u32[array_index] = old_src[array_index];
                    }
                }

                array_index += 1;
            }
        }

        result
    }

    /// Scanline mode transition - vertical sweep from top to bottom
    ///
    /// The transition sweeps from map_height down to 0.
    fn scanline_mode_transition(&mut self) -> bool {
        // Nothing to do if no final frame
        let final_frame = match self.rendering_data.final_frame.as_ref() {
            Some(f) => f,
            None => return true,
        };

        // Use dimensions stored when frame was set (from actual rendered frame)
        let map_width = self.rendering_data.frame_width;
        let map_height = self.rendering_data.frame_height;

        // Verify dimensions are valid
        if map_width == 0 || map_height == 0 {
            log::warn!("Frame dimensions not set for scanline. Skipping transition.");
            self.rendering_data.current_frame = Some(final_frame.clone());
            self.rendering_data.last_frame = self.rendering_data.current_frame.clone();
            return true;
        }

        // Verify the frame size matches expected dimensions
        let expected_size = map_width * map_height * RENDERING_COLOR_CHANNEL_COUNT;
        if final_frame.len() != expected_size {
            log::warn!(
                "Frame size mismatch in scanline: expected {} ({}x{}x4), got {}. Skipping transition.",
                expected_size, map_width, map_height, final_frame.len()
            );
            // Just return the final frame as-is
            self.rendering_data.current_frame = Some(final_frame.clone());
            self.rendering_data.last_frame = self.rendering_data.current_frame.clone();
            return true;
        }

        // Calculate vertical step per tick
        let vertical_step = ((map_height as f64 / RENDERING_MAP_TRANSITION_DURATION_SCANLINE_MODE as f64)
            * RENDERING_MAP_TRANSITION_DELTA_TIME as f64)
            .round() as i32;

        self.rendering_data.threshold_data.display_range = self.configuration.nd_range as f64;
        self.rendering_data.threshold_data.display_mode = self.configuration.efis_mode;

        // Decrease border (sweeping downward)
        self.rendering_data.current_transition_border -= vertical_step;

        if self.rendering_data.current_transition_border > 0 {
            // Still transitioning
            self.rendering_data.current_frame = Some(self.scanline_mode_transition_frame(
                self.rendering_data.last_frame.as_ref(),
                self.rendering_data.final_frame.as_ref().unwrap(),
                map_width,
                map_height,
            ));
            return false;
        }

        // Perform the last frame (ensure we reach y=0)
        if self.rendering_data.current_transition_border + vertical_step >= 0 {
            self.rendering_data.current_frame = Some(self.scanline_mode_transition_frame(
                self.rendering_data.last_frame.as_ref(),
                self.rendering_data.final_frame.as_ref().unwrap(),
                map_width,
                map_height,
            ));
        }

        // Save current frame as last frame for next cycle
        self.rendering_data.last_frame = self.rendering_data.current_frame.clone();

        true
    }

    /// Create a transition frame for scanline mode
    ///
    /// Blends old and new frames based on vertical position.
    /// Pixels in the transition zone (between start_border and current_border) show new frame.
    fn scanline_mode_transition_frame(
        &self,
        old_frame: Option<&Vec<u8>>,
        new_frame: &[u8],
        map_width: usize,
        map_height: usize,
    ) -> Vec<u8> {
        let frame_size = map_width * RENDERING_COLOR_CHANNEL_COUNT * map_height;
        let mut result = vec![0u8; frame_size];

        // Access data as u32 arrays for performance
        let result_u32: &mut [u32] = unsafe {
            std::slice::from_raw_parts_mut(result.as_mut_ptr() as *mut u32, frame_size / 4)
        };
        result_u32.fill(BACKGROUND_COLOR_U32);

        let old_source_u32: Option<&[u32]> = old_frame.map(|f| unsafe {
            std::slice::from_raw_parts(f.as_ptr() as *const u32, f.len() / 4)
        });
        let new_source_u32: &[u32] = unsafe {
            std::slice::from_raw_parts(new_frame.as_ptr() as *const u32, new_frame.len() / 4)
        };

        let start_border = self.rendering_data.start_transition_border as usize;
        let current_border = self.rendering_data.current_transition_border.max(0) as usize;

        let mut array_index = 0;
        for y in 0..map_height {
            for x in 0..map_width {
                // Note: TypeScript uses <= and >= for scanline (sweeping down from top)
                // y <= startBorder means above the start
                // y >= currentBorder means above the current line
                if y <= start_border && y >= current_border {
                    // Within transition zone - show new frame
                    if array_index < new_source_u32.len() {
                        result_u32[array_index] = new_source_u32[array_index];
                    }
                } else if let Some(old_src) = old_source_u32 {
                    // Outside transition zone - show old frame
                    if array_index < old_src.len() {
                        result_u32[array_index] = old_src[array_index];
                    }
                }

                array_index += 1;
            }
        }

        result
    }

    /// Get the current transitioning frame
    pub fn current_frame(&self) -> Option<&Vec<u8>> {
        self.rendering_data.current_frame.as_ref()
    }

    /// Check if a transition is currently in progress
    pub fn is_transitioning(&self) -> bool {
        self.rendering_data.final_frame.is_some() && self.rendering_data.current_frame.is_none()
    }
}

/// Vertical display renderer with scanline transition support
pub struct VerticalDisplayRenderer {
    /// Elevation profile configuration
    elevation_config: ElevationProfileConfig,
    /// Display configuration
    pub display_config: VerticalDisplayConfig,
    /// Rendering data
    rendering_data: VerticalDisplayRenderingData,
    /// Startup time
    startup_time: Instant,
}

struct ElevationProfileConfig {
    path_width: f64,
    waypoints_latitudes: Vec<f64>,
    waypoints_longitudes: Vec<f64>,
    range: f64,
    track_changes_significantly_at_distance: f64,
    fms_path_used: bool,
}

#[derive(Clone)]
pub struct VerticalDisplayConfig {
    pub range: f64,
    pub minimum_altitude: i32,
    pub maximum_altitude: i32,
    pub map_width: u32,
    pub map_height: u32,
}

struct VerticalDisplayRenderingData {
    start_transition_border: i32,
    current_transition_border: i32,
    frame_counter: u32,
    final_frame: Option<Vec<u8>>,
    last_frame: Option<Vec<u8>>,
    current_frame: Option<Vec<u8>>,
}

impl VerticalDisplayRenderer {
    pub fn new(startup_time: Instant) -> Self {
        VerticalDisplayRenderer {
            elevation_config: ElevationProfileConfig {
                path_width: 1.0,
                waypoints_latitudes: Vec::new(),
                waypoints_longitudes: Vec::new(),
                range: 0.0,
                track_changes_significantly_at_distance: -1.0,
                fms_path_used: false,
            },
            display_config: VerticalDisplayConfig {
                range: 0.0,
                minimum_altitude: -500,
                maximum_altitude: 24000,
                map_width: RENDERING_ELEVATION_PROFILE_WIDTH as u32,
                map_height: RENDERING_ELEVATION_PROFILE_HEIGHT as u32,
            },
            rendering_data: VerticalDisplayRenderingData {
                start_transition_border: 0,
                current_transition_border: 0,
                frame_counter: 0,
                final_frame: None,
                last_frame: None,
                current_frame: None,
            },
            startup_time,
        }
    }

    pub fn reset(&mut self, reset_path: bool) {
        self.rendering_data = VerticalDisplayRenderingData {
            start_transition_border: 0,
            current_transition_border: 0,
            frame_counter: 0,
            final_frame: None,
            last_frame: None,
            current_frame: None,
        };

        if reset_path {
            self.elevation_config = ElevationProfileConfig {
                path_width: 1.0,
                waypoints_latitudes: Vec::new(),
                waypoints_longitudes: Vec::new(),
                range: 0.0,
                track_changes_significantly_at_distance: -1.0,
                fms_path_used: false,
            };
        }

        self.display_config = VerticalDisplayConfig {
            range: 0.0,
            minimum_altitude: -500,
            maximum_altitude: 24000,
            map_width: RENDERING_ELEVATION_PROFILE_WIDTH as u32,
            map_height: RENDERING_ELEVATION_PROFILE_HEIGHT as u32,
        };
    }

    pub fn aircraft_status_update(&mut self, status: AircraftStatus, side: DisplaySide) {
        self.elevation_config.fms_path_used =
            !status.manual_azim_enabled && !self.elevation_config.waypoints_latitudes.is_empty();

        let efis = if side == DisplaySide::Left {
            &status.efis_data_capt
        } else {
            &status.efis_data_fo
        };

        let vd_range = if efis.arc_mode {
            efis.nd_range.max(10).min(160) as f64
        } else {
            (efis.nd_range / 2).max(5).min(160) as f64
        };

        self.elevation_config.range = vd_range;
        self.display_config.range = efis.nd_range as f64;
        self.display_config.minimum_altitude = efis.vd_range_lower;
        self.display_config.maximum_altitude = efis.vd_range_upper;
    }

    pub fn path_data_update(&mut self, data: VerticalPathData) {
        self.elevation_config.path_width = data.path_width;
        self.elevation_config.waypoints_latitudes =
            data.waypoints.iter().map(|w| w.latitude).collect();
        self.elevation_config.waypoints_longitudes =
            data.waypoints.iter().map(|w| w.longitude).collect();
        self.elevation_config.track_changes_significantly_at_distance =
            data.track_changes_significantly_at_distance;
    }

    pub fn num_path_elements(&self) -> usize {
        self.elevation_config.waypoints_latitudes.len()
    }

    pub fn display_configuration(&self) -> &VerticalDisplayConfig {
        &self.display_config
    }

    /// Start a new map rendering cycle for vertical display
    pub fn start_new_map_cycle(&mut self, current_time: Instant) {
        let map_width = self.display_config.map_width as usize;

        if self.rendering_data.last_frame.is_none() {
            // First frame - calculate starting position based on time since startup
            let time_since_start = current_time.duration_since(self.startup_time).as_millis() as u64;
            let frame_update_count =
                time_since_start as f64 / RENDERING_MAP_FRAME_VALIDITY_TIME_SCANLINE_MODE as f64;
            let ratio_since_last_frame = frame_update_count - frame_update_count.floor();

            self.rendering_data.start_transition_border =
                (map_width as f64 * ratio_since_last_frame).floor() as i32;
        } else {
            self.rendering_data.start_transition_border = 0;
        }

        self.rendering_data.current_transition_border = self.rendering_data.start_transition_border;
        self.rendering_data.frame_counter = 0;
    }

    /// Set the final frame for transition
    pub fn set_final_frame(&mut self, frame: Vec<u8>) {
        self.rendering_data.final_frame = Some(frame);
    }

    /// Perform one step of the horizontal transition (for vertical display)
    pub fn render(&mut self) -> bool {
        if self.rendering_data.final_frame.is_none() {
            return true;
        }

        let map_width = self.display_config.map_width as usize;
        let map_height = self.display_config.map_height as usize;

        // Calculate horizontal step per tick
        let horizontal_step = ((map_width as f64
            / RENDERING_MAP_TRANSITION_DURATION_SCANLINE_MODE as f64)
            * RENDERING_MAP_TRANSITION_DELTA_TIME as f64)
            .round() as i32;

        self.rendering_data.current_transition_border += horizontal_step;

        if self.rendering_data.current_transition_border < map_width as i32 {
            // Still transitioning
            self.rendering_data.current_frame = Some(self.transition_frame(
                self.rendering_data.last_frame.as_ref(),
                self.rendering_data.final_frame.as_ref().unwrap(),
                map_width,
                map_height,
            ));
            return false;
        }

        // Last frame
        if self.rendering_data.current_transition_border + horizontal_step > map_width as i32 {
            self.rendering_data.current_frame = Some(self.transition_frame(
                self.rendering_data.last_frame.as_ref(),
                self.rendering_data.final_frame.as_ref().unwrap(),
                map_width,
                map_height,
            ));
        }

        self.rendering_data.last_frame = self.rendering_data.current_frame.clone();
        true
    }

    /// Create transition frame for vertical display (horizontal sweep)
    fn transition_frame(
        &self,
        old_frame: Option<&Vec<u8>>,
        new_frame: &[u8],
        map_width: usize,
        map_height: usize,
    ) -> Vec<u8> {
        let frame_size = map_width * RENDERING_COLOR_CHANNEL_COUNT * map_height;
        let mut result = vec![0u8; frame_size];

        let result_u32: &mut [u32] = unsafe {
            std::slice::from_raw_parts_mut(result.as_mut_ptr() as *mut u32, frame_size / 4)
        };
        result_u32.fill(BACKGROUND_COLOR_U32);

        let old_source_u32: Option<&[u32]> = old_frame.map(|f| unsafe {
            std::slice::from_raw_parts(f.as_ptr() as *const u32, f.len() / 4)
        });
        let new_source_u32: &[u32] = unsafe {
            std::slice::from_raw_parts(new_frame.as_ptr() as *const u32, new_frame.len() / 4)
        };

        let start_border = self.rendering_data.start_transition_border.max(0) as usize;
        let current_border = self
            .rendering_data
            .current_transition_border
            .min(map_width as i32) as usize;

        let mut array_index = 0;
        for y in 0..map_height {
            for x in 0..map_width {
                if x >= start_border && x <= current_border {
                    // Within transition zone - show new frame
                    if array_index < new_source_u32.len() {
                        result_u32[array_index] = new_source_u32[array_index];
                    }
                } else if let Some(old_src) = old_source_u32 {
                    // Outside transition zone - show old frame
                    if array_index < old_src.len() {
                        result_u32[array_index] = old_src[array_index];
                    }
                }

                array_index += 1;
            }
        }

        result
    }

    /// Get the current frame
    pub fn current_frame(&self) -> Option<&Vec<u8>> {
        self.rendering_data.current_frame.as_ref()
    }
}
