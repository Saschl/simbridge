//! Navigation and Vertical display renderers

#![allow(dead_code)]
#![allow(unused_variables)]

use std::time::Instant;

use crate::types::{
    constants::*,
    AircraftStatus, DisplaySide, EfisData, NavigationDisplayData,
    TerrainLevelMode, VerticalPathData,
};

/// Navigation display renderer
pub struct NavigationDisplayRenderer {
    /// Display configuration
    configuration: EfisData,
    /// Rendering data
    rendering_data: NavigationDisplayRenderingData,
    /// Aircraft status
    aircraft_status: Option<AircraftStatus>,
    /// Startup time
    startup_time: Instant,
}

struct NavigationDisplayRenderingData {
    start_transition_border: i32,
    current_transition_border: i32,
    frame_counter: u32,
    threshold_data: NavigationDisplayData,
    final_frame: Option<Vec<u8>>,
    last_frame: Option<Vec<u8>>,
    current_frame: Option<Vec<u8>>,
    frame_validity_duration: u64,
}

impl NavigationDisplayRenderer {
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
                frame_validity_duration: RENDERING_MAP_UPDATE_TIMEOUT_ARC_MODE
                    + RENDERING_MAP_TRANSITION_DURATION_ARC_MODE,
            },
            aircraft_status: None,
            startup_time,
        }
    }

    pub fn reset(&mut self) {
        self.rendering_data = NavigationDisplayRenderingData {
            start_transition_border: 0,
            current_transition_border: 0,
            frame_counter: 0,
            threshold_data: NavigationDisplayData::default(),
            final_frame: None,
            last_frame: None,
            current_frame: None,
            frame_validity_duration: self.rendering_data.frame_validity_duration,
        };
    }

    pub fn aircraft_status_update(&mut self, status: AircraftStatus, side: DisplaySide) {
        let config = if side == DisplaySide::Left {
            status.efis_data_capt.clone()
        } else {
            status.efis_data_fo.clone()
        };

        self.configure_navigation_display(config);
        self.aircraft_status = Some(status);
    }

    fn configure_navigation_display(&mut self, mut config: EfisData) {
        let last_config = &self.configuration;
        let arc_mode_changed = last_config.arc_mode != config.arc_mode;
        let config_changed =
            last_config.efis_mode != config.efis_mode ||
            last_config.nd_range != config.nd_range ||
            arc_mode_changed ||
            last_config.terr_on_nd != config.terr_on_nd ||
            last_config.terr_on_vd != config.terr_on_vd;

        // Recalculate map dimensions when arc_mode changes or first time
        if self.configuration.map_width.is_none() || arc_mode_changed {
            // Set dimensions based on mode and aircraft type
            // Using A380X dimensions for now (TODO: detect aircraft type from rendering mode)
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
            config.map_offset_x = Some(((NAVIGATION_DISPLAY_MAX_PIXEL_WIDTH as i32 -
                config.map_width.unwrap() as i32) as f64 * 0.5).ceil() as i32);
            config.center_offset_y = Some(if config.arc_mode {
                NAVIGATION_DISPLAY_ARC_MODE_CENTER_OFFSET_Y_A380X
            } else {
                NAVIGATION_DISPLAY_ROSE_MODE_CENTER_OFFSET_Y_A380X
            });

            log::debug!("Map dimensions updated: arc_mode={}, map_size={}x{}, offset_x={}, center_offset_y={}",
                config.arc_mode, config.map_width.unwrap(), config.map_height.unwrap(),
                config.map_offset_x.unwrap(), config.center_offset_y.unwrap());
        }

        if config_changed {
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
        }

        self.configuration = config;
    }

    pub fn display_configuration(&self) -> EfisData {
        self.configuration.clone()
    }

    pub fn display_data(&self) -> NavigationDisplayData {
        self.rendering_data.threshold_data.clone()
    }

    pub fn first_frame(&self) -> bool {
        self.rendering_data.threshold_data.first_frame
    }

    pub fn set_first_frame(&mut self, first_frame: bool) {
        self.rendering_data.threshold_data.first_frame = first_frame;
    }

    pub fn start_new_map_cycle(&mut self, current_time: Instant) {
        self.rendering_data.frame_counter = 0;
        self.rendering_data.current_frame = None;
        self.rendering_data.last_frame = self.rendering_data.final_frame.take();
    }

    pub fn current_frame(&self) -> Option<&Vec<u8>> {
        self.rendering_data.current_frame.as_ref()
    }
}

/// Vertical display renderer
pub struct VerticalDisplayRenderer {
    /// Elevation profile configuration
    elevation_config: ElevationProfileConfig,
    /// Display configuration
    display_config: VerticalDisplayConfig,
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
        self.elevation_config.waypoints_latitudes = data.waypoints.iter().map(|w| w.latitude).collect();
        self.elevation_config.waypoints_longitudes = data.waypoints.iter().map(|w| w.longitude).collect();
        self.elevation_config.track_changes_significantly_at_distance = data.track_changes_significantly_at_distance;
    }

    pub fn num_path_elements(&self) -> usize {
        self.elevation_config.waypoints_latitudes.len()
    }

    pub fn display_configuration(&self) -> &VerticalDisplayConfig {
        &self.display_config
    }
}
