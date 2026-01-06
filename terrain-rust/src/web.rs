//! Web server module for terrain REST API
//!
//! Provides HTTP endpoints matching the original TypeScript implementation.

#![allow(unused_imports)]

use actix_web::{web, HttpResponse, Responder};
use base64::{Engine as _, engine::general_purpose::STANDARD as BASE64};
use log::{error, info};
use serde::Deserialize;

use crate::AppState;
use crate::types::{
    DisplaySide, ElevationSamplePathDto, TawsAircraftStatusDataDto,
    VerticalPathData,
};

/// Query parameter for display side
#[derive(Debug, Deserialize)]
pub struct DisplayQuery {
    display: String,
}

/// Configure all routes for the terrain API
pub fn configure_routes(cfg: &mut web::ServiceConfig) {
    cfg.service(
        web::scope("/api/v1/terrain")
            .route("/renderingTimestamp", web::get().to(rendering_timestamp))
            .route("/renderingThresholds", web::get().to(rendering_thresholds))
           // .route("/renderingFrames", web::get().to(rendering_frames))
            .route("/aircraftStatusData", web::post().to(aircraft_status_data))
            .route("/verticalDisplayPath", web::post().to(vertical_display_path))
    )
    .route("/health", web::get().to(health_check));
}

/// Health check endpoint
async fn health_check() -> impl Responder {
    HttpResponse::Ok().json(serde_json::json!({
        "status": "ok",
        "service": "terrain-rust"
    }))
}

/// GET /api/v1/terrain/renderingTimestamp
/// Returns the timestamp of the current rendering data
async fn rendering_timestamp(
    data: web::Data<AppState>,
    query: web::Query<DisplayQuery>,
) -> impl Responder {
    let side = match parse_display_side(&query.display) {
        Ok(s) => s,
        Err(_) => return HttpResponse::BadRequest().json(-1),
    };

    let processor = match data.terrain_processor.read() {
        Ok(p) => p,
        Err(_) => return HttpResponse::InternalServerError().json(-1),
    };

    let frame_data = processor.get_frame_data(side);
    HttpResponse::Ok().json(frame_data.timestamp as i64)
}

/// GET /api/v1/terrain/renderingThresholds
/// Returns the thresholds for the current rendering data
async fn rendering_thresholds(
    data: web::Data<AppState>,
    query: web::Query<DisplayQuery>,
) -> impl Responder {
    let side = match parse_display_side(&query.display) {
        Ok(s) => s,
        Err(_) => return HttpResponse::BadRequest().json(serde_json::Value::Null),
    };

    let processor = match data.terrain_processor.read() {
        Ok(p) => p,
        Err(_) => return HttpResponse::InternalServerError().json(serde_json::Value::Null),
    };

    let frame_data = processor.get_frame_data(side);

    match frame_data.thresholds {
        Some(thresholds) => HttpResponse::Ok().json(thresholds),
        None => HttpResponse::Ok().json(serde_json::Value::Null),
    }
}


/// POST /api/v1/terrain/aircraftStatusData
/// Update aircraft status data
async fn aircraft_status_data(
    data: web::Data<AppState>,
    body: web::Json<TawsAircraftStatusDataDto>,
) -> impl Responder {
    let mut processor = match data.terrain_processor.write() {
        Ok(p) => p,
        Err(e) => {
            error!("Failed to acquire processor lock: {}", e);
            return HttpResponse::InternalServerError().finish();
        }
    };

    // Enable SimBridge client mode and update last web update timestamp
    processor.enable_simbridge_client_data();

    let status = body.into_inner().into();

    processor.aircraft_status_update(status);

    HttpResponse::Ok().finish()
}

/// POST /api/v1/terrain/verticalDisplayPath
/// Update the vertical display path
async fn vertical_display_path(
    data: web::Data<AppState>,
    body: web::Json<ElevationSamplePathDto>,
) -> impl Responder {
    let mut processor = match data.terrain_processor.write() {
        Ok(p) => p,
        Err(e) => {
            error!("Failed to acquire processor lock: {}", e);
            return HttpResponse::InternalServerError().finish();
        }
    };

    let path: VerticalPathData = body.into_inner().into();
    //info!("vertical_display_path: received {} waypoints, path_width={}", path.waypoints.len(), path.path_width);
    processor.vertical_path_update(path);

    HttpResponse::Ok().finish()
}

/// Parse display side from query parameter
fn parse_display_side(display: &str) -> Result<DisplaySide, String> {
    match display.to_uppercase().as_str() {
        "L" | "LEFT" => Ok(DisplaySide::Left),
        "R" | "RIGHT" => Ok(DisplaySide::Right),
        _ => Err(format!("Invalid display side: {}", display)),
    }
}
