//! SimConnect handler for MSFS communication using msfs-rs
//!
//! Handles connection to MSFS via SimConnect and receives aircraft status data.
//!
//! Uses raw byte buffers with ClientDataDefinition since the aircraft sends
//! packed binary data that won't match Rust's struct alignment.

use std::collections::HashMap;
use std::sync::{Arc, RwLock, mpsc};
use std::thread;
use std::time::{Duration, Instant};
use std::pin::Pin;
use log::{info, warn, error, debug};

use msfs::sim_connect::{
    client_data_definition, SimConnect, SimConnectRecv, ClientDataArea,
};

use crate::types::{
    AircraftStatus, EfisData, DisplaySide, TerrainRenderingMode,
    constants::*,
};
use crate::processing::TerrainProcessor;

/// Client data area names (must match aircraft side)
const CLIENT_DATA_NAME_AIRCRAFT_STATUS: &str = "FBW_SIMBRIDGE_EGPWC_AIRCRAFT_STATUS";
const CLIENT_DATA_NAME_METADATA_LEFT: &str = "FBW_SIMBRIDGE_TERRONND_THRESHOLDS_LEFT";
const CLIENT_DATA_NAME_METADATA_RIGHT: &str = "FBW_SIMBRIDGE_TERRONND_THRESHOLDS_RIGHT";
const CLIENT_DATA_NAME_FRAME_LEFT: &str = "FBW_SIMBRIDGE_TERRONND_FRAME_DATA_LEFT";
const CLIENT_DATA_NAME_FRAME_RIGHT: &str = "FBW_SIMBRIDGE_TERRONND_FRAME_DATA_RIGHT";

const SIMCONNECT_CLIENT_NAME: &str = "FBW_SIMBRIDGE_SIMCONNECT_RUST";

// Request IDs for different data types
const REQUEST_ID_AIRCRAFT_STATUS: u32 = 0;

// Wire format sizes
const AIRCRAFT_STATUS_SIZE: usize = 46;
const NAVIGATION_DISPLAY_METADATA_SIZE: usize = 14;
const FRAME_CHUNK_SIZE: usize = 8192;

/// Raw buffer for aircraft status data (46 bytes)
/// Using a fixed-size byte array as ClientDataDefinition
#[client_data_definition]
#[repr(C)]
#[derive(Clone, Copy)]
pub struct AircraftStatusBuffer {
    pub data: [u8; AIRCRAFT_STATUS_SIZE],
}

impl Default for AircraftStatusBuffer {
    fn default() -> Self {
        Self { data: [0u8; AIRCRAFT_STATUS_SIZE] }
    }
}

/// Raw buffer for navigation display metadata (14 bytes)
#[client_data_definition]
#[repr(C)]
#[derive(Clone, Copy)]
pub struct MetadataBuffer {
    pub data: [u8; NAVIGATION_DISPLAY_METADATA_SIZE],
}

impl Default for MetadataBuffer {
    fn default() -> Self {
        Self { data: [0u8; NAVIGATION_DISPLAY_METADATA_SIZE] }
    }
}

/// Frame data chunk - max 8192 bytes per chunk
#[client_data_definition]
#[repr(C)]
#[derive(Clone, Copy)]
pub struct FrameDataChunk {
    pub data: [u8; FRAME_CHUNK_SIZE],
}

impl Default for FrameDataChunk {
    fn default() -> Self {
        Self { data: [0u8; FRAME_CHUNK_SIZE] }
    }
}

/// Messages from SimConnect callback to handler
enum SimConnectMessage {
    AircraftStatus(AircraftStatusBuffer),
    SimulatorRunning(bool),
    Paused(bool),
    Quit,
    Exception(u32),
}

/// SimConnect handler for MSFS communication
pub struct SimConnectHandler {
    terrain_processor: Arc<RwLock<TerrainProcessor>>,
    /// Rendering state for each display side
    rendering_state: HashMap<DisplaySide, DisplayRenderingState>,
}

/// State for display-side transition rendering
struct DisplayRenderingState {
    /// Whether a transition is currently in progress
    transitioning: bool,
    /// When the last transition cycle completed
    last_cycle_complete: Instant,
    /// Whether we've rendered the last frame of this cycle
    rendered_last_frame: bool,
    /// Frame dimensions for current cycle (width, height)
    frame_dimensions: (usize, usize),
    /// Base metadata for current cycle (frame_byte_count updated per frame after PNG encoding)
    base_metadata: Option<crate::types::NavigationDisplayData>,
}

impl SimConnectHandler {
    /// Create a new SimConnect handler
    pub fn new(terrain_processor: Arc<RwLock<TerrainProcessor>>) -> Self {
        let mut rendering_state = HashMap::new();
        let now = Instant::now();
        rendering_state.insert(DisplaySide::Left, DisplayRenderingState {
            transitioning: false,
            last_cycle_complete: now,
            rendered_last_frame: false,
            frame_dimensions: (0, 0),
            base_metadata: None,
        });
        rendering_state.insert(DisplaySide::Right, DisplayRenderingState {
            transitioning: false,
            last_cycle_complete: now,
            rendered_last_frame: false,
            frame_dimensions: (0, 0),
            base_metadata: None,
        });

        SimConnectHandler {
            terrain_processor,
            rendering_state,
        }
    }

    /// Run the SimConnect handler loop
    pub fn run(&mut self) {
        info!("Starting SimConnect handler with msfs-rs...");

        let mut show_connection_error = true;
        let mut paused = true;

        loop {
            // Try to connect and run
            match self.try_connect_and_run(&mut paused) {
                Ok(()) => {
                    // Normal disconnect, try to reconnect
                    info!("SimConnect session ended, reconnecting...");
                    show_connection_error = true;
                }
                Err(e) => {
                    if show_connection_error {
                        warn!("Failed to connect to SimConnect: {:?} - Retrying every 10 seconds", e);
                        show_connection_error = false;
                    }
                }
            }

            // Reset state
            self.on_reset();

            // Wait before retry
            thread::sleep(Duration::from_secs(10));
        }
    }

    /// Try to connect and run the SimConnect session
    fn try_connect_and_run(&mut self, paused: &mut bool) -> Result<(), Box<dyn std::error::Error>> {
        // Create channel for receiving messages from callback
        let (tx, rx) = mpsc::channel::<SimConnectMessage>();

        // Open SimConnect connection with callback
        let mut sim = SimConnect::open(SIMCONNECT_CLIENT_NAME, move |sim, recv| {
            match recv {
                SimConnectRecv::ClientData(data) => {
                    // Try to parse as AircraftStatusBuffer
                    if let Some(buffer) = data.into::<AircraftStatusBuffer>(sim) {
                        debug!("Received AircraftStatus data from SimConnect");
                        let _ = tx.send(SimConnectMessage::AircraftStatus(*buffer));
                    } else {
                        debug!("Received ClientData but not AircraftStatusBuffer");
                    }
                }
                SimConnectRecv::Event(event) => {
                    let event_id = event.id();
                    let event_data = event.data();

                    // System events use IDs we subscribed to
                    match event_id {
                        0 => {
                            // Simulator state: 0 = stopped, 1 = running
                            let _ = tx.send(SimConnectMessage::SimulatorRunning(event_data == 1));
                        }
                        1 => {
                            // Pause state: 0 = unpaused, non-zero = paused
                            let _ = tx.send(SimConnectMessage::Paused(event_data != 0));
                        }
                        _ => {}
                    }
                }
                SimConnectRecv::Quit(_) => {
                    let _ = tx.send(SimConnectMessage::Quit);
                }
                SimConnectRecv::Exception(ex) => {
                    let exception = ex.dwException;
                    let _ = tx.send(SimConnectMessage::Exception(exception));
                }
                SimConnectRecv::Open(_) => {
                    info!("SimConnect connection opened");
                }
                _ => {}
            }
        })?;

        info!("Connected to MSFS via SimConnect");

        // Set up client data areas
        self.setup_client_data(&mut sim)?;

        // Subscribe to system events
        sim.subscribe_to_system_event("Sim")?;
        sim.subscribe_to_system_event("Pause_EX1")?;

        // Request aircraft status data
        sim.request_client_data::<AircraftStatusBuffer>(
            REQUEST_ID_AIRCRAFT_STATUS,
            CLIENT_DATA_NAME_AIRCRAFT_STATUS,
        )?;

        info!("All SimConnect data areas registered successfully");

        // Get client areas for sending data
        let metadata_left: ClientDataArea<MetadataBuffer> =
            sim.get_client_area(CLIENT_DATA_NAME_METADATA_LEFT)?;
        let metadata_right: ClientDataArea<MetadataBuffer> =
            sim.get_client_area(CLIENT_DATA_NAME_METADATA_RIGHT)?;
        let frame_left: ClientDataArea<FrameDataChunk> =
            sim.get_client_area(CLIENT_DATA_NAME_FRAME_LEFT)?;
        let frame_right: ClientDataArea<FrameDataChunk> =
            sim.get_client_area(CLIENT_DATA_NAME_FRAME_RIGHT)?;

        // Timing for transition loop
        let mut last_transition_tick = Instant::now();
        let transition_interval = Duration::from_millis(RENDERING_MAP_TRANSITION_DELTA_TIME);

        // Main message loop
        loop {
            // Process SimConnect messages
            sim.call_dispatch()?;

            // Process any messages from callback
            while let Ok(msg) = rx.try_recv() {
                match msg {
                    SimConnectMessage::AircraftStatus(buffer) => {
                        let status = self.parse_aircraft_status(&buffer.data);

                        debug!("Aircraft status: lat={:.4}, lon={:.4}, alt={}, hdg={}, terr_capt={}, terr_fo={}, efis_mode_capt={}",
                            status.latitude, status.longitude, status.altitude, status.heading,
                            status.efis_data_capt.terr_on_nd, status.efis_data_fo.terr_on_nd,
                            status.efis_data_capt.efis_mode);

                        if let Ok(mut processor) = self.terrain_processor.write() {
                            processor.aircraft_status_update(status.clone());
                        }
                    }
                    SimConnectMessage::SimulatorRunning(running) => {
                        if !running {
                            self.on_reset();
                        }
                    }
                    SimConnectMessage::Paused(is_paused) => {
                        *paused = is_paused;
                        if is_paused {
                            info!("Simulation paused");
                        } else {
                            info!("Simulation unpaused");
                        }
                    }
                    SimConnectMessage::Quit => {
                        info!("SimConnect connection closed");
                        return Ok(());
                    }
                    SimConnectMessage::Exception(code) => {
                        error!("SimConnect exception: {}", code);
                        return Err(format!("SimConnect exception: {}", code).into());
                    }
                }
            }

            // Run transition tick at 40ms intervals (like TypeScript's setInterval)
            if !*paused && last_transition_tick.elapsed() >= transition_interval {
                last_transition_tick = Instant::now();

                self.tick_transitions(
                    &mut sim,
                    &metadata_left,
                    &metadata_right,
                    &frame_left,
                    &frame_right,
                );
            }

            // Small sleep to prevent busy waiting (target ~40ms tick rate)
            let sleep_time = transition_interval.saturating_sub(last_transition_tick.elapsed());
            if sleep_time > Duration::from_millis(1) {
                thread::sleep(sleep_time.min(Duration::from_millis(10)));
            }
        }
    }

    /// Tick the transition system - called every RENDERING_MAP_TRANSITION_DELTA_TIME (40ms)
    /// This matches TypeScript's setInterval-based transition loop
    fn tick_transitions(
        &mut self,
        sim: &mut Pin<Box<SimConnect>>,
        metadata_left: &ClientDataArea<MetadataBuffer>,
        metadata_right: &ClientDataArea<MetadataBuffer>,
        frame_left: &ClientDataArea<FrameDataChunk>,
        frame_right: &ClientDataArea<FrameDataChunk>,
    ) {
        for side in [DisplaySide::Left, DisplaySide::Right] {
            self.tick_transition_for_side(
                sim,
                side,
                metadata_left,
                metadata_right,
                frame_left,
                frame_right,
            );
        }
    }

    /// Tick the transition for a specific display side
    fn tick_transition_for_side(
        &mut self,
        sim: &mut Pin<Box<SimConnect>>,
        side: DisplaySide,
        metadata_left: &ClientDataArea<MetadataBuffer>,
        metadata_right: &ClientDataArea<MetadataBuffer>,
        frame_left: &ClientDataArea<FrameDataChunk>,
        frame_right: &ClientDataArea<FrameDataChunk>,
    ) {
        // Get current rendering state
        let (is_transitioning, elapsed) = {
            match self.rendering_state.get(&side) {
                Some(s) => (s.transitioning, s.last_cycle_complete.elapsed()),
                None => return,
            }
        };

        // Get current rendering mode and timeout
        let timeout = {
            if let Ok(processor) = self.terrain_processor.read() {
                let mode = processor.get_rendering_mode();
                if mode == TerrainRenderingMode::ArcMode {
                    Duration::from_millis(RENDERING_MAP_UPDATE_TIMEOUT_ARC_MODE)
                } else {
                    Duration::from_millis(RENDERING_MAP_UPDATE_TIMEOUT_SCANLINE_MODE)
                }
            } else {
                return;
            }
        };

        // Check if config changed and we need an immediate new cycle
        let needs_immediate_cycle = {
            if let Ok(processor) = self.terrain_processor.read() {
                processor.needs_new_cycle(side)
            } else {
                false
            }
        };

        // Check if we need to start a new cycle (either timeout or config change)
        if !is_transitioning {
            if elapsed >= timeout || needs_immediate_cycle {
                // Start a new rendering cycle
                if needs_immediate_cycle {
                    debug!("Starting immediate rendering cycle for {:?} (config changed)", side);
                } else {
                    debug!("Starting new rendering cycle for {:?} (timeout)", side);
                }

                // Render raw RGBA frame for transitions
                let frame_result = {
                    if let Ok(mut processor) = self.terrain_processor.write() {
                        processor.render_raw_frame_for_transition(side)
                    } else {
                        None
                    }
                };

                if let Some((nav_data, raw_frame, width, height)) = frame_result {
                    // Start the transition with raw RGBA frame and dimensions
                    if let Ok(mut processor) = self.terrain_processor.write() {
                        processor.start_transition_cycle(side, raw_frame, width, height);
                    }

                    // Store dimensions and base metadata for encoding during transition
                    if let Some(state) = self.rendering_state.get_mut(&side) {
                        state.transitioning = true;
                        state.rendered_last_frame = false;
                        state.frame_dimensions = (width, height);
                        state.base_metadata = Some(nav_data);
                    }
                }
            }
            return;
        }

        // We're transitioning - tick the transition
        let (transition_complete, current_frame) = {
            if let Ok(mut processor) = self.terrain_processor.write() {
                let complete = processor.tick_transition(side);
                let frame = processor.get_current_transition_frame(side);
                (complete, frame)
            } else {
                (false, None)
            }
        };

        // Get stored dimensions and metadata for encoding
        let (dimensions, base_metadata) = {
            match self.rendering_state.get(&side) {
                Some(s) => (s.frame_dimensions, s.base_metadata.clone()),
                None => return,
            }
        };

        // Encode and send the current transition frame if available
        if let Some(raw_frame) = current_frame {
            // Encode raw RGBA to PNG
            let png_frame = {
                if let Ok(processor) = self.terrain_processor.read() {
                    processor.encode_to_png(&raw_frame, dimensions.0, dimensions.1)
                } else {
                    None
                }
            };

            if let Some(png_data) = png_frame {
                // Update metadata with actual PNG size and send
                if let Some(mut nav_data) = base_metadata {
                    nav_data.frame_byte_count = png_data.len() as u32;

                    let metadata = self.pack_metadata(&nav_data);
                    let meta_area = match side {
                        DisplaySide::Left => metadata_left,
                        DisplaySide::Right => metadata_right,
                    };
                    if let Err(e) = sim.set_client_data(meta_area, &metadata) {
                        error!("Failed to send metadata for {:?}: {:?}", side, e);
                    }
                }

                // Send the PNG frame
                let frame_area = match side {
                    DisplaySide::Left => frame_left,
                    DisplaySide::Right => frame_right,
                };
                Self::send_frame_data_static(sim, frame_area, &png_data);
            }
        }

        // Check if transition is complete
        if transition_complete {
            debug!("Transition complete for {:?}", side);
            if let Some(state) = self.rendering_state.get_mut(&side) {
                state.transitioning = false;
                state.last_cycle_complete = Instant::now();
                state.rendered_last_frame = true;
                state.base_metadata = None;
            }
        }
    }

    /// Send frame data in chunks (static version to avoid borrow issues)
    fn send_frame_data_static(
        sim: &mut Pin<Box<SimConnect>>,
        area: &ClientDataArea<FrameDataChunk>,
        frame: &[u8],
    ) {
        let chunks = (frame.len() + FRAME_CHUNK_SIZE - 1) / FRAME_CHUNK_SIZE;
        debug!("Sending {} bytes in {} chunks (frame size: {})", frame.len(), chunks, FRAME_CHUNK_SIZE);

        for i in 0..chunks {
            let start = i * FRAME_CHUNK_SIZE;
            let remaining = frame.len() - start;
            let byte_count = remaining.min(FRAME_CHUNK_SIZE);

            let mut chunk = FrameDataChunk::default();
            chunk.data[..byte_count].copy_from_slice(&frame[start..start + byte_count]);

            if let Err(e) = sim.set_client_data(area, &chunk) {
                error!("Failed to send frame chunk {}: {:?}", i, e);
                break;
            }
        }
    }

    /// Set up client data areas
    fn setup_client_data(&self, sim: &mut Pin<Box<SimConnect>>) -> Result<(), Box<dyn std::error::Error>> {
        // The aircraft side creates and owns the aircraft status area
        // We just need to map it for reading via request_client_data

        // Create the output areas (metadata and frame data)
        // These are created by SimBridge for the aircraft to read
        sim.create_client_data::<MetadataBuffer>(CLIENT_DATA_NAME_METADATA_LEFT)?;
        sim.create_client_data::<MetadataBuffer>(CLIENT_DATA_NAME_METADATA_RIGHT)?;
        sim.create_client_data::<FrameDataChunk>(CLIENT_DATA_NAME_FRAME_LEFT)?;
        sim.create_client_data::<FrameDataChunk>(CLIENT_DATA_NAME_FRAME_RIGHT)?;

        Ok(())
    }

    /// Parse aircraft status from raw bytes
    fn parse_aircraft_status(&self, buffer: &[u8]) -> AircraftStatus {
        if buffer.len() < AIRCRAFT_STATUS_SIZE {
            error!("Aircraft status buffer too small: expected {}, got {}", AIRCRAFT_STATUS_SIZE, buffer.len());
            return AircraftStatus::default();
        }

        let adiru_data_valid = buffer[0] != 0;
        let latitude = f32::from_le_bytes([buffer[1], buffer[2], buffer[3], buffer[4]]) as f64;
        let longitude = f32::from_le_bytes([buffer[5], buffer[6], buffer[7], buffer[8]]) as f64;
        let altitude = i32::from_le_bytes([buffer[9], buffer[10], buffer[11], buffer[12]]);
        let heading = i16::from_le_bytes([buffer[13], buffer[14]]);
        let vertical_speed = i16::from_le_bytes([buffer[15], buffer[16]]);
        let gear_is_down = buffer[17] != 0;
        let runway_data_valid = buffer[18] != 0;
        let runway_latitude = f32::from_le_bytes([buffer[19], buffer[20], buffer[21], buffer[22]]) as f64;
        let runway_longitude = f32::from_le_bytes([buffer[23], buffer[24], buffer[25], buffer[26]]) as f64;

        // Captain EFIS data
        let nd_range_capt = u16::from_le_bytes([buffer[27], buffer[28]]);
        let arc_mode_capt = buffer[29] != 0;
        let terr_enabled_capt = buffer[30] != 0;
        let efis_mode_capt = buffer[31];

        // FO EFIS data
        let nd_range_fo = u16::from_le_bytes([buffer[32], buffer[33]]);
        let arc_mode_fo = buffer[34] != 0;
        let terr_enabled_fo = buffer[35] != 0;
        let efis_mode_fo = buffer[36];

        let rendering_mode = buffer[37];
        let ground_truth_lat = f32::from_le_bytes([buffer[38], buffer[39], buffer[40], buffer[41]]) as f64;
        let ground_truth_lon = f32::from_le_bytes([buffer[42], buffer[43], buffer[44], buffer[45]]) as f64;

        AircraftStatus {
            adiru_data_valid,
            taws_inop: false,
            latitude,
            longitude,
            altitude,
            heading,
            vertical_speed,
            gear_is_down,
            runway_data_valid,
            runway_latitude,
            runway_longitude,
            efis_data_capt: EfisData {
                nd_range: nd_range_capt,
                arc_mode: arc_mode_capt,
                terr_on_nd: terr_enabled_capt,
                terr_on_vd: terr_enabled_capt,
                efis_mode: efis_mode_capt,
                vd_range_lower: -500,
                vd_range_upper: 24000,
                ..Default::default()
            },
            efis_data_fo: EfisData {
                nd_range: nd_range_fo,
                arc_mode: arc_mode_fo,
                terr_on_nd: terr_enabled_fo,
                terr_on_vd: terr_enabled_fo,
                efis_mode: efis_mode_fo,
                vd_range_lower: -500,
                vd_range_upper: 24500,
                ..Default::default()
            },
            navigation_display_rendering_mode: rendering_mode,
            manual_azim_enabled: true,
            manual_azim_degrees: heading as u16,
            ground_truth_latitude: ground_truth_lat,
            ground_truth_longitude: ground_truth_lon,
        }
    }

    /// Pack navigation display metadata for transmission
    fn pack_metadata(&self, nav_data: &crate::types::NavigationDisplayData) -> MetadataBuffer {
        let mut buffer = MetadataBuffer::default();

        // MinimumElevation (i16) - bytes 0-1
        let min_elev_bytes = nav_data.minimum_elevation.to_le_bytes();
        buffer.data[0] = min_elev_bytes[0];
        buffer.data[1] = min_elev_bytes[1];

        // MinimumElevationMode (u8) - byte 2
        buffer.data[2] = nav_data.minimum_elevation_mode as u8;

        // MaximumElevation (i16) - bytes 3-4
        let max_elev_bytes = nav_data.maximum_elevation.to_le_bytes();
        buffer.data[3] = max_elev_bytes[0];
        buffer.data[4] = max_elev_bytes[1];

        // MaximumElevationMode (u8) - byte 5
        buffer.data[5] = nav_data.maximum_elevation_mode as u8;

        // FirstFrame (u8) - byte 6
        buffer.data[6] = if nav_data.first_frame { 1 } else { 0 };

        // DisplayRange (u16) - bytes 7-8
        let display_range = nav_data.display_range.round() as u16;
        let range_bytes = display_range.to_le_bytes();
        buffer.data[7] = range_bytes[0];
        buffer.data[8] = range_bytes[1];

        // DisplayMode (u8) - byte 9
        buffer.data[9] = nav_data.display_mode;

        // FrameByteCount (u32) - bytes 10-13
        let byte_count_bytes = nav_data.frame_byte_count.to_le_bytes();
        buffer.data[10] = byte_count_bytes[0];
        buffer.data[11] = byte_count_bytes[1];
        buffer.data[12] = byte_count_bytes[2];
        buffer.data[13] = byte_count_bytes[3];

        debug!("Packed metadata: min_elev={}, min_mode={:?}, max_elev={}, max_mode={:?}, first={}, range={}, mode={}, byte_count={}",
            nav_data.minimum_elevation, nav_data.minimum_elevation_mode,
            nav_data.maximum_elevation, nav_data.maximum_elevation_mode,
            nav_data.first_frame, display_range, nav_data.display_mode, nav_data.frame_byte_count);
        debug!("Metadata bytes: {:02x?}", &buffer.data);

        buffer
    }

    /// Handle reset (disconnect or sim state change)
    fn on_reset(&mut self) {
        if let Ok(mut processor) = self.terrain_processor.write() {
            processor.reset();
        }
        // Reset rendering state
        let now = Instant::now();
        for state in self.rendering_state.values_mut() {
            state.transitioning = false;
            state.last_cycle_complete = now;
            state.rendered_last_frame = false;
        }
    }
}
