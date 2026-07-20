//! terrain-service: the SimBridge terrain sidecar.
//!
//! Usage: terrain-service [--port <u16>] --terrain-db <path> [--log-level info|debug]
//!
//! Prints `SIMBRIDGE-TERRAIN READY port=<port> pid=<pid>` on stdout once the
//! HTTP API is listening. Exits when stdin reaches EOF (parent died), on
//! POST /shutdown, or on a fatal startup error.

use std::io::{BufRead, Write};
use std::sync::mpsc::channel;
use std::sync::{Arc, Mutex};

use fbw_simbridge_terrain::fileformat::TerrainMap;
use fbw_simbridge_terrain::http_api::HttpApi;
use fbw_simbridge_terrain::orchestrator::{CycleData, Orchestrator, SharedState};
use fbw_simbridge_terrain::state::startup_status;
use fbw_simbridge_terrain::worldmap::WorldMapManager;

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let mut port: Option<u16> = None;
    let mut terrain_db: Option<String> = None;

    let mut i = 0;
    while i < args.len() {
        match args[i].as_str() {
            "--port" => {
                port = args.get(i + 1).and_then(|v| v.parse().ok());
                i += 2;
            }
            "--terrain-db" => {
                terrain_db = args.get(i + 1).cloned();
                i += 2;
            }
            "--log-level" => {
                i += 2;
            }
            other => {
                eprintln!("[ERROR] unknown argument: {other}");
                std::process::exit(2);
            }
        }
    }

    let Some(terrain_db) = terrain_db else {
        eprintln!("[ERROR] --terrain-db <path> is required");
        std::process::exit(2);
    };

    let bytes = match std::fs::read(&terrain_db) {
        Ok(bytes) => bytes,
        Err(e) => {
            eprintln!("[ERROR] failed to read terrain database {terrain_db}: {e}");
            std::process::exit(1);
        }
    };
    eprintln!(
        "[INFO] read {:.2} MB of terrain database",
        bytes.len() as f64 / (1024.0 * 1024.0)
    );
    let terrain = match TerrainMap::from_bytes(bytes) {
        Ok(map) => map,
        Err(e) => {
            eprintln!("[ERROR] failed to parse terrain database: {e}");
            std::process::exit(1);
        }
    };

    // seed with the startup position (Innsbruck) exactly like the TS worker
    let status = startup_status();
    let mut world = WorldMapManager::new(terrain);
    world.update_position(status.ground_truth_latitude, status.ground_truth_longitude);
    let world = Arc::new(Mutex::new(world));
    eprintln!("[INFO] initialized the map handler");

    let shared = Arc::new(Mutex::new(SharedState::new(status)));
    let published_left = Arc::new(Mutex::new(CycleData::default()));
    let published_right = Arc::new(Mutex::new(CycleData::default()));

    let (sink, sink_receiver) = channel();
    #[cfg(feature = "simconnect")]
    let sink_thread = {
        let shared = Arc::clone(&shared);
        std::thread::spawn(move || {
            fbw_simbridge_terrain::simconnect::run_simconnect(shared, sink_receiver)
        })
    };
    #[cfg(not(feature = "simconnect"))]
    let sink_thread = std::thread::spawn(move || while sink_receiver.recv().is_ok() {});

    let http = match HttpApi::bind(
        port,
        Arc::clone(&shared),
        Arc::clone(&published_left),
        Arc::clone(&published_right),
    ) {
        Ok(http) => http,
        Err(e) => {
            eprintln!("[ERROR] failed to bind HTTP API: {e}");
            std::process::exit(1);
        }
    };

    let mut orchestrator = Orchestrator::new(
        Arc::clone(&world),
        Arc::clone(&shared),
        sink,
        (Arc::clone(&published_left), Arc::clone(&published_right)),
    );
    let orchestrator_thread = std::thread::spawn(move || orchestrator.run());

    println!("SIMBRIDGE-TERRAIN READY port={} pid={}", http.port(), std::process::id());
    std::io::stdout().flush().ok();

    // stdin watchdog: parent closing the pipe (or dying) shuts us down
    let watchdog_shared = Arc::clone(&shared);
    std::thread::spawn(move || {
        let stdin = std::io::stdin();
        let mut line = String::new();
        loop {
            line.clear();
            match stdin.lock().read_line(&mut line) {
                Ok(0) | Err(_) => break,
                Ok(_) => {}
            }
        }
        eprintln!("[INFO] stdin closed - shutting down");
        watchdog_shared.lock().unwrap().shutdown = true;
    });

    http.run();
    shared.lock().unwrap().shutdown = true;
    let _ = orchestrator_thread.join();
    drop(http);
    let _ = sink_thread.join();
    eprintln!("[INFO] terrain service stopped");
}
