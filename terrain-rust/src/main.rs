//! SimBridge Terrain Service - Rust Implementation
//!
//! This is a CPU-based terrain rendering service for MSFS that provides:
//! - Terrain data loading and caching
//! - Navigation display rendering
//! - Vertical display rendering
//! - SimConnect integration for aircraft status
//! - REST API for web clients

mod types;
mod fileformat;
mod mapdata;
mod processing;
mod simconnect_handler;
mod web;

use std::sync::{Arc, RwLock};
use std::path::PathBuf;
use actix_web::{web as actix_web_data, App, HttpServer};
use actix_cors::Cors;
use log::{info, error};

use crate::processing::TerrainProcessor;
use crate::simconnect_handler::SimConnectHandler;
use crate::web::configure_routes;

/// Application state shared across the web server and SimConnect handler
pub struct AppState {
    pub terrain_processor: Arc<RwLock<TerrainProcessor>>,
}

#[actix_web::main]
async fn main() -> std::io::Result<()> {
    // Initialize logging with debug level for our crate
    env_logger::Builder::from_env(
        env_logger::Env::default().default_filter_or("simbridge_terrain=debug,info")
    ).init();

    info!("SimBridge Terrain Service (Rust) starting...");

    // Determine terrain map path
    let terrain_path = std::env::current_exe()
        .ok()
        .and_then(|p| p.parent().map(|p| p.to_path_buf()))
        .unwrap_or_else(|| PathBuf::from("."))
        .join("terrain")
        .join("terrain.map");

    info!("Loading terrain map from: {:?}", terrain_path);

    // Initialize terrain processor
    let terrain_processor = match TerrainProcessor::new(&terrain_path) {
        Ok(processor) => {
            info!("Terrain processor initialized successfully");
            Arc::new(RwLock::new(processor))
        }
        Err(e) => {
            error!("Failed to initialize terrain processor: {}", e);
            // Continue without terrain data - will return empty frames
            Arc::new(RwLock::new(TerrainProcessor::empty()))
        }
    };

    let app_state = actix_web_data::Data::new(AppState {
        terrain_processor: terrain_processor.clone(),
    });

    // Start SimConnect handler in a separate thread
    let simconnect_processor = terrain_processor.clone();
    std::thread::spawn(move || {
        let mut handler = SimConnectHandler::new(simconnect_processor);
        handler.run();
    });

    info!("Starting HTTP server on 0.0.0.0:8380");

    // Start HTTP server
    HttpServer::new(move || {
        let cors = Cors::default()
            .allow_any_origin()
            .allow_any_method()
            .allow_any_header();

        App::new()
            .wrap(cors)
            .app_data(app_state.clone())
            .configure(configure_routes)
    })
    .bind("0.0.0.0:8380")?
    .run()
    .await
}
