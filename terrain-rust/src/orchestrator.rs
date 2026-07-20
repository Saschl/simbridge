//! The rendering orchestrator: owns the per-side display state machines and
//! the shared aircraft state, drives the 40 ms tick loop, encodes frames and
//! hands them to the transmission sink (SimConnect in the service).
//!
//! Port of the cycle/lifecycle logic in `processing/terrainworker.ts`.

use std::sync::mpsc::Sender;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use crate::compositor::{compose_screen_frame, SCREEN_HEIGHT_WITHOUT_VD, SCREEN_HEIGHT_WITH_VD};
use crate::elevation_map::{extract_local_elevation_map, metres_per_pixel};
use crate::geodesy::{project_wgs84, NM_TO_METRES};
use crate::nd_render::{
    absolute_cut_off_altitude, compute_render_stats, compute_thresholds, render_navigation_display,
    TerrainLevelMode, Thresholds,
};
use crate::patterns::{ARC_PATTERN, SCANLINE_PATTERN};
use crate::png_out::encode_rgba;
use crate::state::{nd_map_geometry, AircraftStatus, EfisData, Side};
use crate::statistics::elevation_histogram;
use crate::transition::{
    Transition, TransitionStyle, ARC_UPDATE_TIMEOUT_MS, SCANLINE_UPDATE_TIMEOUT_MS,
    TRANSITION_DELTA_TIME_MS,
};
use crate::vd_render::{
    extract_elevation_profile, render_vertical_display, vd_range_from_nd, ElevationProfileConfig,
    VD_PROFILE_HEIGHT, VD_PROFILE_WIDTH,
};
use crate::worldmap::{WorldMap, WorldMapManager};

/// SimBridge-client (HTTP) override of SimConnect data, ms.
pub const SIMBRIDGE_CLIENT_DATA_TIMEOUT_MS: u64 = 2 * 60 * 1000;
/// The right side starts with a -1500 ms phase offset for a more realistic look.
pub const RIGHT_SIDE_STARTUP_OFFSET_MS: u64 = 1500;

#[derive(Debug, Clone, PartialEq)]
pub struct VerticalPathData {
    pub path_width: f64,
    pub track_changes_significantly_at_distance: f64,
    pub waypoints: Vec<(f64, f64)>,
}

/// Everything sent towards MSFS for one frame (packed by the SimConnect sink).
pub struct FrameTransmission {
    pub side: Side,
    pub minimum_elevation: f64,
    pub minimum_elevation_mode: TerrainLevelMode,
    pub maximum_elevation: f64,
    pub maximum_elevation_mode: TerrainLevelMode,
    pub first_frame: bool,
    pub display_range: f64,
    pub display_mode: u8,
    pub png: Arc<Vec<u8>>,
}

/// Cached data of the last completed rendering cycle (served over HTTP).
#[derive(Clone, Default)]
pub struct CycleData {
    /// Epoch ms of the last completed cycle (FIXED: the TS version never set it).
    pub timestamp_ms: Option<u64>,
    pub thresholds: Option<Thresholds>,
    pub frames: Vec<Arc<Vec<u8>>>,
}

#[derive(Clone, PartialEq)]
pub enum StatusSource {
    SimConnect,
    HttpClient,
}

/// Shared mutable state between the HTTP server, SimConnect pump and the
/// orchestrator loop.
pub struct SharedState {
    pub status: AircraftStatus,
    pub status_seq: u64,
    pub http_override_until: Option<Instant>,
    pub sim_paused: bool,
    pub vd_path: Option<VerticalPathData>,
    pub vd_path_seq: u64,
    pub reset_seq: u64,
    pub shutdown: bool,
    pub sim_connected: bool,
    /// Frames actually written to SimConnect (diagnostics).
    pub tx_frame_count: u64,
}

impl SharedState {
    pub fn new(status: AircraftStatus) -> Self {
        Self {
            status,
            status_seq: 0,
            http_override_until: None,
            sim_paused: false,
            vd_path: None,
            vd_path_seq: 0,
            reset_seq: 0,
            shutdown: false,
            sim_connected: false,
            tx_frame_count: 0,
        }
    }

    /// Aircraft status ingest with source arbitration: while an HTTP client
    /// feeds data, SimConnect updates are ignored (2-minute timeout).
    pub fn update_status(&mut self, status: AircraftStatus, source: StatusSource) {
        let now = Instant::now();
        match source {
            StatusSource::HttpClient => {
                self.http_override_until =
                    Some(now + Duration::from_millis(SIMBRIDGE_CLIENT_DATA_TIMEOUT_MS));
            }
            StatusSource::SimConnect => {
                if let Some(until) = self.http_override_until {
                    if now < until {
                        return;
                    }
                    self.http_override_until = None;
                }
            }
        }
        self.status = status;
        self.status_seq += 1;
    }
}

fn epoch_ms() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_millis() as u64
}

/// JS `Math.sign`: 0 for zero, unlike `f64::signum`.
fn js_sign(value: f64) -> f64 {
    if value > 0.0 {
        1.0
    } else if value < 0.0 {
        -1.0
    } else {
        0.0
    }
}

/// Per-side display state machine.
struct SideState {
    side: Side,
    startup_ms: u64,
    nd_transition: Transition,
    vd_transition: Transition,
    /// Last EFIS configuration seen (`displayConfiguration()` equivalent).
    last_efis: Option<EfisData>,
    last_manual_azim_enabled: bool,
    current_track_changes: f64,
    /// Live thresholds/metadata of the current cycle (`displayData()`).
    display_range: f64,
    display_mode: u8,
    thresholds: Option<Thresholds>,
    /// Cycle currently animating?
    cycle_active: bool,
    vd_rendered_this_cycle: bool,
    nd_done: bool,
    vd_done: bool,
    /// Vertical path change detection (`updatePathData` force redraw).
    seen_vd_path_seq: u64,
    last_vd_waypoint_count: usize,
    /// When to start the next cycle after the previous one completed.
    next_cycle_at: Option<Instant>,
    /// Building frames of the running cycle; published on completion.
    building: CycleData,
    published: Arc<Mutex<CycleData>>,
    seen_reset_seq: u64,
}

impl SideState {
    fn new(side: Side, startup_ms: u64, published: Arc<Mutex<CycleData>>) -> Self {
        Self {
            side,
            startup_ms,
            nd_transition: Transition::new(TransitionStyle::Arc),
            vd_transition: Transition::new(TransitionStyle::VerticalDisplay),
            last_efis: None,
            last_manual_azim_enabled: false,
            current_track_changes: -1.0,
            display_range: 0.0,
            display_mode: 0,
            thresholds: None,
            cycle_active: false,
            vd_rendered_this_cycle: false,
            nd_done: true,
            vd_done: true,
            seen_vd_path_seq: 0,
            last_vd_waypoint_count: 0,
            next_cycle_at: None,
            building: CycleData::default(),
            published,
            seen_reset_seq: 0,
        }
    }

    fn reset(&mut self) {
        self.nd_transition.reset();
        self.vd_transition.reset();
        self.thresholds = None;
        self.cycle_active = false;
        self.next_cycle_at = None;
        self.building = CycleData::default();
        *self.published.lock().unwrap() = CycleData::default();
    }
}

pub struct Orchestrator {
    world: Arc<Mutex<WorldMapManager>>,
    shared: Arc<Mutex<SharedState>>,
    sides: [SideState; 2],
    sink: Sender<FrameTransmission>,
    last_world_update: Instant,
}

impl Orchestrator {
    #[allow(clippy::type_complexity)]
    pub fn new(
        world: Arc<Mutex<WorldMapManager>>,
        shared: Arc<Mutex<SharedState>>,
        sink: Sender<FrameTransmission>,
        published: (Arc<Mutex<CycleData>>, Arc<Mutex<CycleData>>),
    ) -> Self {
        let startup = epoch_ms();
        Self {
            world,
            shared,
            sides: [
                SideState::new(Side::Left, startup, published.0),
                SideState::new(
                    Side::Right,
                    startup.saturating_sub(RIGHT_SIDE_STARTUP_OFFSET_MS),
                    published.1,
                ),
            ],
            sink,
            last_world_update: Instant::now() - Duration::from_secs(60),
        }
    }

    /// Main loop: 40 ms ticks until shutdown.
    pub fn run(&mut self) {
        loop {
            let tick_started = Instant::now();

            let (status, paused, vd_path, vd_path_seq, reset_seq, shutdown) = {
                let shared = self.shared.lock().unwrap();
                (
                    shared.status.clone(),
                    shared.sim_paused,
                    shared.vd_path.clone(),
                    shared.vd_path_seq,
                    shared.reset_seq,
                    shared.shutdown,
                )
            };
            if shutdown {
                return;
            }

            // ground-truth world map refresh (~1 Hz). Unlike the TS worker this
            // also follows the HTTP ground truth, so pure SimBridge-client
            // operation keeps the tile cache alive.
            if self.last_world_update.elapsed() >= Duration::from_secs(1) {
                self.world
                    .lock()
                    .unwrap()
                    .update_position(status.ground_truth_latitude, status.ground_truth_longitude);
                self.last_world_update = Instant::now();
            }

            let world = self.world.lock().unwrap().snapshot();
            if let Some(world) = world {
                for index in 0..2 {
                    self.tick_side(index, &status, paused, vd_path.as_ref(), vd_path_seq, reset_seq, &world);
                }
            }

            let elapsed = tick_started.elapsed();
            let budget = Duration::from_millis(TRANSITION_DELTA_TIME_MS);
            if elapsed < budget {
                std::thread::sleep(budget - elapsed);
            }
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn tick_side(
        &mut self,
        index: usize,
        status: &AircraftStatus,
        paused: bool,
        vd_path: Option<&VerticalPathData>,
        vd_path_seq: u64,
        reset_seq: u64,
        world: &WorldMap,
    ) {
        let side = self.sides[index].side;
        let efis = status.efis(side).clone();
        let state = &mut self.sides[index];

        // sim reset (SimConnect stop event / disconnect)
        if state.seen_reset_seq != reset_seq {
            state.seen_reset_seq = reset_seq;
            state.reset();
            state.last_efis = None;
            let _ = self.sink.send(Self::reset_metadata(side));
            if index == 0 {
                // wipe the tile cache once (TS `MapHandler.reset`); the 1 Hz
                // ground-truth update restitches on the next tick
                self.world.lock().unwrap().reset();
                self.last_world_update = Instant::now() - Duration::from_secs(60);
            }
            return;
        }

        // configuration diffing (`updateRendering`)
        let config_changed = state.last_efis.as_ref().is_some_and(|last| {
            last.efis_mode != efis.efis_mode
                || last.nd_range != efis.nd_range
                || last.arc_mode != efis.arc_mode
                || last.terr_on_nd != efis.terr_on_nd
                || last.terr_on_vd != efis.terr_on_vd
        });
        let start_rendering = config_changed
            || state.last_manual_azim_enabled != status.manual_azim_enabled
            || state.last_efis.is_none();
        state.last_manual_azim_enabled = status.manual_azim_enabled;
        state.last_efis = Some(efis.clone());

        if start_rendering {
            state.reset();
            // TS `resetRenderingCycle` pushes the reset threshold metadata to
            // the aircraft so the visualizer clears stale state
            let _ = self.sink.send(Self::reset_metadata(side));
            if efis.nd_range > 0.0 && (efis.terr_on_nd || efis.terr_on_vd) {
                self.start_cycle(index, status, vd_path, world);
            }
            return;
        }

        // vertical path updates can force an immediate redraw (`updatePathData`)
        let state = &mut self.sides[index];
        if state.seen_vd_path_seq != vd_path_seq {
            state.seen_vd_path_seq = vd_path_seq;
            let posted_count = vd_path.map_or(0, |p| p.waypoints.len());
            let posted_track = vd_path.map_or(-1.0, |p| p.track_changes_significantly_at_distance);

            let force_redraw = state.last_vd_waypoint_count != posted_count
                || (posted_track - state.current_track_changes).abs() > 0.1
                || js_sign(posted_track) != js_sign(state.current_track_changes);

            // mirror what the TS renderer stores: manual azimuth replaces the
            // posted path with the single projected endpoint
            state.last_vd_waypoint_count = if status.manual_azim_enabled || posted_count == 0 {
                1
            } else {
                posted_count
            };
            state.current_track_changes = posted_track;

            if force_redraw && efis.nd_range > 0.0 && (efis.terr_on_nd || efis.terr_on_vd) {
                self.start_cycle(index, status, vd_path, world);
                return;
            }
        }

        let state = &mut self.sides[index];
        if state.cycle_active {
            self.advance_cycle(index, status, paused);
        } else if state
            .next_cycle_at
            .is_some_and(|at| Instant::now() >= at)
        {
            let efis = status.efis(side).clone();
            if efis.terr_on_nd || efis.terr_on_vd {
                self.start_cycle(index, status, vd_path, world);
            } else {
                self.sides[index].next_cycle_at = None;
            }
        }
    }

    /// Reset-state threshold metadata without a frame (`resetRenderingCycle`).
    fn reset_metadata(side: Side) -> FrameTransmission {
        FrameTransmission {
            side,
            minimum_elevation: -1.0,
            minimum_elevation_mode: TerrainLevelMode::PeaksMode,
            maximum_elevation: -1.0,
            maximum_elevation_mode: TerrainLevelMode::PeaksMode,
            first_frame: true,
            display_range: 0.0,
            display_mode: 0,
            png: Arc::new(Vec::new()),
        }
    }

    /// `startNavigationDisplayRenderingCycle` + `startNewMapCycle`.
    fn start_cycle(
        &mut self,
        index: usize,
        status: &AircraftStatus,
        vd_path: Option<&VerticalPathData>,
        world: &WorldMap,
    ) {
        let side = self.sides[index].side;
        let efis = status.efis(side).clone();
        let geometry = nd_map_geometry(efis.arc_mode, status.vertical_display_required());

        if efis.nd_range == 0.0 {
            self.sides[index].reset();
            return;
        }

        // ND final frame
        let mpp = metres_per_pixel(efis.nd_range, &geometry, efis.arc_mode);
        let elevations = extract_local_elevation_map(
            world,
            status.latitude,
            status.longitude,
            status.heading,
            &geometry,
            mpp,
            efis.arc_mode,
        );
        let histogram = elevation_histogram(&elevations);
        let cut_off = absolute_cut_off_altitude(
            world,
            status.latitude,
            status.longitude,
            status.altitude,
            status.runway_data_valid,
            status.runway_latitude,
            status.runway_longitude,
        );
        let stats = compute_render_stats(
            &histogram,
            status.altitude,
            status.vertical_speed,
            status.gear_is_down,
            cut_off,
        );
        let pattern: &[u8] = if status.scanline_mode() {
            &SCANLINE_PATTERN[..]
        } else {
            &ARC_PATTERN[..]
        };
        let nd_frame = render_navigation_display(&elevations, pattern, &geometry, &stats);
        let mut thresholds = compute_thresholds(&stats);

        // A380X: metadata must keep flowing with TERR ON ND deselected, but the
        // thresholds are hidden with negative values
        if !efis.terr_on_nd {
            thresholds.minimum_elevation = -1.0;
            thresholds.maximum_elevation = -1.0;
        }

        // VD final frame when rendered on this side
        let vd_rendered = status.vertical_display_required()
            && efis.terr_on_vd
            && (efis.efis_mode == 2 || efis.efis_mode == 3);
        let vd_frame = if vd_rendered {
            let (profile_config, grey_from_x) = self.vd_profile_config(index, status, vd_path, &efis);
            profile_config.map(|config| {
                let profile =
                    extract_elevation_profile(world, status.latitude, status.longitude, &config);
                render_vertical_display(
                    &profile,
                    efis.vd_range_lower as f64,
                    efis.vd_range_upper as f64,
                    grey_from_x,
                )
            })
        } else {
            None
        };

        let now = epoch_ms();
        let state = &mut self.sides[index];
        state.thresholds = Some(thresholds);
        state.display_range = efis.nd_range;
        state.display_mode = efis.efis_mode;

        state.nd_transition.style = if status.scanline_mode() {
            TransitionStyle::ScanlineNd
        } else {
            TransitionStyle::Arc
        };
        state
            .nd_transition
            .start_new_cycle(nd_frame, geometry.width, geometry.height, now, state.startup_ms);
        state.nd_done = false;

        if let Some(vd_frame) = vd_frame {
            state
                .vd_transition
                .start_new_cycle(vd_frame, VD_PROFILE_WIDTH, VD_PROFILE_HEIGHT, now, state.startup_ms);
            state.vd_done = false;
            state.vd_rendered_this_cycle = true;
        } else {
            state.vd_done = true;
            state.vd_rendered_this_cycle = false;
        }

        state.building = CycleData::default();
        state.cycle_active = true;
        state.next_cycle_at = None;
    }

    /// VD elevation profile configuration: manual azimuth or the FMS path
    /// (`updateRendering` / `updatePathData`).
    fn vd_profile_config(
        &mut self,
        _index: usize,
        status: &AircraftStatus,
        vd_path: Option<&VerticalPathData>,
        efis: &EfisData,
    ) -> (Option<ElevationProfileConfig>, f64) {
        let range = vd_range_from_nd(efis.nd_range, efis.arc_mode);

        if status.manual_azim_enabled || vd_path.is_none_or(|p| p.waypoints.is_empty()) {
            let end = project_wgs84(
                status.latitude,
                status.longitude,
                status.manual_azim_degrees,
                160.0 * NM_TO_METRES,
            );
            (
                Some(ElevationProfileConfig {
                    path_width: 1.0,
                    waypoints: vec![end],
                    range,
                    track_changes_significantly_at_distance: -1.0,
                    fms_path_used: false,
                }),
                -1.0,
            )
        } else {
            let path = vd_path.unwrap();
            let grey_from_x = if path.track_changes_significantly_at_distance >= 0.0 {
                path.track_changes_significantly_at_distance / range * VD_PROFILE_WIDTH as f64
            } else {
                -1.0
            };
            (
                Some(ElevationProfileConfig {
                    path_width: path.path_width,
                    waypoints: path.waypoints.clone(),
                    range,
                    track_changes_significantly_at_distance: path.track_changes_significantly_at_distance,
                    fms_path_used: true,
                }),
                grey_from_x,
            )
        }
    }

    /// One 40 ms animation tick of a running cycle.
    fn advance_cycle(&mut self, index: usize, status: &AircraftStatus, paused: bool) {
        let state = &mut self.sides[index];
        let efis = status.efis(state.side).clone();
        let geometry = nd_map_geometry(efis.arc_mode, status.vertical_display_required());

        if !state.nd_done {
            state.nd_done = state.nd_transition.render();
        }
        if !state.vd_done {
            state.vd_done = state.vd_transition.render();
        }

        let screen_height = if status.vertical_display_required() {
            SCREEN_HEIGHT_WITH_VD
        } else {
            SCREEN_HEIGHT_WITHOUT_VD
        };
        let nd_map = if efis.terr_on_nd {
            state.nd_transition.current_frame.as_deref()
        } else {
            None
        };
        let vd_map = if state.vd_rendered_this_cycle {
            state.vd_transition.current_frame.as_deref()
        } else {
            None
        };

        if !paused && (nd_map.is_some() || vd_map.is_some()) {
            let screen = compose_screen_frame(
                screen_height,
                nd_map,
                geometry.width,
                geometry.height,
                geometry.offset_x,
                vd_map,
                VD_PROFILE_WIDTH,
                VD_PROFILE_HEIGHT,
            );
            if let Ok(png) = encode_rgba(768, screen_height, &screen) {
                let png = Arc::new(png);
                let thresholds = state.thresholds.unwrap_or(Thresholds {
                    minimum_elevation: -1.0,
                    minimum_elevation_mode: TerrainLevelMode::PeaksMode,
                    maximum_elevation: -1.0,
                    maximum_elevation_mode: TerrainLevelMode::PeaksMode,
                });
                let _ = self.sink.send(FrameTransmission {
                    side: state.side,
                    minimum_elevation: thresholds.minimum_elevation,
                    minimum_elevation_mode: thresholds.minimum_elevation_mode,
                    maximum_elevation: thresholds.maximum_elevation,
                    maximum_elevation_mode: thresholds.maximum_elevation_mode,
                    first_frame: state.building.frames.is_empty(),
                    display_range: state.display_range,
                    display_mode: state.display_mode,
                    png: Arc::clone(&png),
                });
                state.building.frames.push(png);
            }
        }

        if state.nd_done && state.vd_done {
            state.cycle_active = false;
            state.building.timestamp_ms = Some(epoch_ms());
            state.building.thresholds = state.thresholds;
            *state.published.lock().unwrap() = state.building.clone();

            if efis.terr_on_nd || efis.terr_on_vd {
                let timeout = if status.scanline_mode() {
                    SCANLINE_UPDATE_TIMEOUT_MS
                } else {
                    ARC_UPDATE_TIMEOUT_MS
                };
                state.next_cycle_at = Some(Instant::now() + Duration::from_millis(timeout));
            }
        }
    }
}
