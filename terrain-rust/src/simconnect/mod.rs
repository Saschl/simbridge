//! SimConnect integration: receives the EGPWC aircraft status and system
//! events, transmits threshold metadata and PNG frame chunks.
//!
//! Port of `apps/server/src/terrain/communication/simconnect.ts` on top of
//! FBW's msfs-rs bindings. The runner itself is feature-gated (`simconnect`)
//! because building the bindings requires the MSFS SDK; decode/encode are
//! pure and always available (and unit-tested).

pub mod decode;
pub mod encode;

#[cfg(feature = "simconnect")]
pub use runner::run_simconnect;

#[cfg(feature = "simconnect")]
mod runner {
    use std::sync::atomic::{AtomicBool, Ordering};
    use std::sync::mpsc::Receiver;
    use std::sync::{Arc, Mutex};
    use std::time::{Duration, Instant};

    use msfs::sim_connect::{client_data_definition, SimConnect, SimConnectRecv};

    use super::decode::{decode_aircraft_status, EGPWC_AIRCRAFT_STATUS_BYTES};
    use super::encode::{frame_chunks, pack_thresholds, FRAME_CHUNK_BYTES, THRESHOLD_BYTES};
    use crate::orchestrator::{FrameTransmission, SharedState, StatusSource};
    use crate::state::Side;

    const CONNECTION_NAME: &str = "FBW_SIMBRIDGE_SIMCONNECT";
    const EGPWC_STATUS_AREA: &str = "FBW_SIMBRIDGE_EGPWC_AIRCRAFT_STATUS";
    const THRESHOLDS_LEFT_AREA: &str = "FBW_SIMBRIDGE_TERRONND_THRESHOLDS_LEFT";
    const THRESHOLDS_RIGHT_AREA: &str = "FBW_SIMBRIDGE_TERRONND_THRESHOLDS_RIGHT";
    const FRAME_LEFT_AREA: &str = "FBW_SIMBRIDGE_TERRONND_FRAME_DATA_LEFT";
    const FRAME_RIGHT_AREA: &str = "FBW_SIMBRIDGE_TERRONND_FRAME_DATA_RIGHT";
    const RECONNECT_DELAY: Duration = Duration::from_secs(10);

    #[client_data_definition]
    struct EgpwcAircraftStatus {
        data: [u8; EGPWC_AIRCRAFT_STATUS_BYTES],
    }

    #[client_data_definition]
    struct NdThresholds {
        data: [u8; THRESHOLD_BYTES],
    }

    #[client_data_definition]
    struct FrameData {
        data: [u8; FRAME_CHUNK_BYTES],
    }

    /// Connection loop: (re)connect every 10 s, pump until quit/shutdown.
    pub fn run_simconnect(shared: Arc<Mutex<SharedState>>, receiver: Receiver<FrameTransmission>) {
        let mut first_attempt = true;
        loop {
            if shared.lock().unwrap().shutdown {
                return;
            }

            match pump_session(&shared, &receiver) {
                Ok(()) => {
                    eprintln!("[INFO] SimConnect session ended - reconnecting in 10 s");
                    // the sim went away: reset the display state like the TS worker
                    let mut state = shared.lock().unwrap();
                    state.sim_connected = false;
                    state.reset_seq += 1;
                }
                Err(e) => {
                    if first_attempt {
                        eprintln!("[WARN] connection to MSFS failed ({e:?}) - retry every 10 seconds");
                    }
                    shared.lock().unwrap().sim_connected = false;
                }
            }
            first_attempt = false;

            // drain pending frames while disconnected and honor shutdown
            let waited = Instant::now();
            while waited.elapsed() < RECONNECT_DELAY {
                if shared.lock().unwrap().shutdown {
                    return;
                }
                while receiver.try_recv().is_ok() {}
                std::thread::sleep(Duration::from_millis(100));
            }
        }
    }

    fn pump_session(
        shared: &Arc<Mutex<SharedState>>,
        receiver: &Receiver<FrameTransmission>,
    ) -> Result<(), msfs::sim_connect::HResult> {
        let quit = Arc::new(AtomicBool::new(false));
        // (sim_event_id, pause_event_id), known after subscription
        let event_ids = Arc::new(Mutex::new((u32::MAX, u32::MAX)));

        let cb_shared = Arc::clone(shared);
        let cb_quit = Arc::clone(&quit);
        let cb_event_ids = Arc::clone(&event_ids);

        let mut sim = SimConnect::open(CONNECTION_NAME, move |sim, recv| match recv {
            SimConnectRecv::ClientData(data) => {
                if let Some(block) = data.into::<EgpwcAircraftStatus>(sim) {
                    let status = decode_aircraft_status(&block.data);
                    cb_shared
                        .lock()
                        .unwrap()
                        .update_status(status, StatusSource::SimConnect);
                }
            }
            SimConnectRecv::Event(event) => {
                let (sim_event, pause_event) = *cb_event_ids.lock().unwrap();
                if event.uEventID == sim_event {
                    // sim stopped -> reset the rendering state
                    if event.dwData == 0 {
                        cb_shared.lock().unwrap().reset_seq += 1;
                    }
                } else if event.uEventID == pause_event {
                    cb_shared.lock().unwrap().sim_paused = event.dwData != 0;
                }
            }
            SimConnectRecv::Quit(_) => {
                cb_quit.store(true, Ordering::SeqCst);
            }
            _ => {}
        })?;

        // the aircraft usually owns the status area; creating it first covers
        // the case where SimBridge starts before the aircraft
        let _ = sim.create_client_data::<EgpwcAircraftStatus>(EGPWC_STATUS_AREA);
        sim.request_client_data::<EgpwcAircraftStatus>(0, EGPWC_STATUS_AREA)?;

        let thresholds_left = sim.create_client_data::<NdThresholds>(THRESHOLDS_LEFT_AREA)?;
        let thresholds_right = sim.create_client_data::<NdThresholds>(THRESHOLDS_RIGHT_AREA)?;
        let frame_left = sim.create_client_data::<FrameData>(FRAME_LEFT_AREA)?;
        let frame_right = sim.create_client_data::<FrameData>(FRAME_RIGHT_AREA)?;

        let sim_event = sim.subscribe_to_system_event("Sim")?;
        let pause_event = sim.subscribe_to_system_event("Pause_EX1")?;
        *event_ids.lock().unwrap() = (sim_event, pause_event);

        shared.lock().unwrap().sim_connected = true;
        eprintln!("[INFO] connected to MSFS");

        loop {
            if quit.load(Ordering::SeqCst) || shared.lock().unwrap().shutdown {
                return Ok(());
            }

            sim.call_dispatch()?;

            while let Ok(tx) = receiver.try_recv() {
                let (thresholds_area, frame_area) = match tx.side {
                    Side::Left => (&thresholds_left, &frame_left),
                    Side::Right => (&thresholds_right, &frame_right),
                };

                // metadata first so the visualizer knows the byte count
                sim.set_client_data(thresholds_area, &NdThresholds { data: pack_thresholds(&tx) })?;
                for chunk in frame_chunks(&tx.png) {
                    sim.set_client_data(frame_area, &FrameData { data: chunk })?;
                }
                shared.lock().unwrap().tx_frame_count += 1;
            }

            std::thread::sleep(Duration::from_millis(2));
        }
    }
}
