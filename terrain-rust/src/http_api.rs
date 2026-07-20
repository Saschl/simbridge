//! Local HTTP API of the terrain sidecar. Paths mirror the public SimBridge
//! API (`terrain.controller.ts`) so the NestJS proxy is a pure pass-through.

use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use base64::Engine;
use tiny_http::{Header, Method, Response, Server};

use crate::nd_render::TerrainLevelMode;
use crate::orchestrator::{CycleData, SharedState, StatusSource, VerticalPathData};
use crate::state::{AircraftStatus, Side};

fn json_header() -> Header {
    Header::from_bytes(&b"Content-Type"[..], &b"application/json"[..]).unwrap()
}

/// Formats an f64 like JavaScript (integral values without a fraction).
fn js_number(value: f64) -> String {
    if value.fract() == 0.0 && value.abs() < 9e15 {
        format!("{}", value as i64)
    } else {
        format!("{value}")
    }
}

fn parse_display_side(url: &str) -> Option<Side> {
    let query = url.split_once('?')?.1;
    for pair in query.split('&') {
        if let Some((key, value)) = pair.split_once('=') {
            if key == "display" {
                return match value {
                    "L" => Some(Side::Left),
                    "R" => Some(Side::Right),
                    _ => None,
                };
            }
        }
    }
    None
}

#[derive(serde::Deserialize)]
#[serde(rename_all = "camelCase")]
struct WaypointDto {
    latitude: f64,
    longitude: f64,
}

#[derive(serde::Deserialize)]
#[serde(rename_all = "camelCase")]
struct ElevationSamplePathDto {
    path_width: f64,
    track_changes_significantly_at_distance: f64,
    waypoints: Vec<WaypointDto>,
}

pub struct HttpApi {
    pub server: Server,
    pub shared: Arc<Mutex<SharedState>>,
    pub published_left: Arc<Mutex<CycleData>>,
    pub published_right: Arc<Mutex<CycleData>>,
    pub started: Instant,
}

impl HttpApi {
    pub fn bind(
        port: Option<u16>,
        shared: Arc<Mutex<SharedState>>,
        published_left: Arc<Mutex<CycleData>>,
        published_right: Arc<Mutex<CycleData>>,
    ) -> std::io::Result<Self> {
        let addr = format!("127.0.0.1:{}", port.unwrap_or(0));
        let server = Server::http(&addr).map_err(std::io::Error::other)?;
        Ok(Self {
            server,
            shared,
            published_left,
            published_right,
            started: Instant::now(),
        })
    }

    pub fn port(&self) -> u16 {
        self.server.server_addr().to_ip().map(|addr| addr.port()).unwrap_or(0)
    }

    fn published(&self, side: Side) -> &Arc<Mutex<CycleData>> {
        match side {
            Side::Left => &self.published_left,
            Side::Right => &self.published_right,
        }
    }

    /// Serve until the shared shutdown flag is set.
    pub fn run(&self) {
        loop {
            if self.shared.lock().unwrap().shutdown {
                return;
            }
            let Ok(Some(mut request)) = self.server.recv_timeout(Duration::from_millis(100)) else {
                continue;
            };

            let url = request.url().to_string();
            let path = url.split('?').next().unwrap_or("").to_string();
            let method = request.method().clone();

            let response: Response<std::io::Cursor<Vec<u8>>> = match (&method, path.as_str()) {
                (Method::Get, "/api/v1/terrain/renderingTimestamp") => {
                    match parse_display_side(&url) {
                        Some(side) => {
                            let data = self.published(side).lock().unwrap();
                            let timestamp = data
                                .timestamp_ms
                                .map(|t| t.to_string())
                                .unwrap_or_else(|| "-1".to_string());
                            Response::from_data(timestamp.into_bytes()).with_header(json_header())
                        }
                        None => Response::from_string("display side missing").with_status_code(400),
                    }
                }
                (Method::Get, "/api/v1/terrain/renderingThresholds") => {
                    match parse_display_side(&url) {
                        Some(side) => {
                            let data = self.published(side).lock().unwrap();
                            match &data.thresholds {
                                Some(t) => {
                                    let body = format!(
                                        "{{\"minElevation\":{},\"minElevationIsWarning\":{},\"minElevationIsCaution\":{},\"maxElevation\":{},\"maxElevationIsWarning\":{},\"maxElevationIsCaution\":{}}}",
                                        js_number(t.minimum_elevation),
                                        t.minimum_elevation_mode == TerrainLevelMode::Warning,
                                        t.minimum_elevation_mode == TerrainLevelMode::Caution,
                                        js_number(t.maximum_elevation),
                                        t.maximum_elevation_mode == TerrainLevelMode::Warning,
                                        t.maximum_elevation_mode == TerrainLevelMode::Caution,
                                    );
                                    Response::from_data(body.into_bytes()).with_header(json_header())
                                }
                                None => Response::from_data(Vec::new()),
                            }
                        }
                        None => Response::from_string("display side missing").with_status_code(400),
                    }
                }
                (Method::Get, "/api/v1/terrain/renderingFrames") => match parse_display_side(&url) {
                    Some(side) => {
                        let data = self.published(side).lock().unwrap();
                        let engine = base64::engine::general_purpose::STANDARD;
                        let mut body = String::from("[");
                        for (i, frame) in data.frames.iter().enumerate() {
                            if i > 0 {
                                body.push(',');
                            }
                            body.push('"');
                            body.push_str(&engine.encode(frame.as_slice()));
                            body.push('"');
                        }
                        body.push(']');
                        Response::from_data(body.into_bytes()).with_header(json_header())
                    }
                    None => Response::from_string("display side missing").with_status_code(400),
                },
                (Method::Post, "/api/v1/terrain/aircraftStatusData") => {
                    let mut body = String::new();
                    let _ = request.as_reader().read_to_string(&mut body);
                    match serde_json::from_str::<AircraftStatus>(&body) {
                        Ok(status) => {
                            let mut shared = self.shared.lock().unwrap();
                            if shared.http_override_until.is_none() {
                                eprintln!(
                                    "[INFO] SimBridge client data received, ignoring SimConnect aircraft status from now on"
                                );
                            }
                            shared.update_status(status, StatusSource::HttpClient);
                            Response::from_data(Vec::new())
                        }
                        Err(e) => {
                            eprintln!("[WARN] rejected aircraftStatusData POST: {e}");
                            Response::from_string(format!("invalid aircraft status: {e}"))
                                .with_status_code(400)
                        }
                    }
                }
                (Method::Post, "/api/v1/terrain/verticalDisplayPath") => {
                    let mut body = String::new();
                    let _ = request.as_reader().read_to_string(&mut body);
                    match serde_json::from_str::<ElevationSamplePathDto>(&body) {
                        Ok(path) => {
                            let mut shared = self.shared.lock().unwrap();
                            shared.vd_path = Some(VerticalPathData {
                                path_width: path.path_width,
                                track_changes_significantly_at_distance: path
                                    .track_changes_significantly_at_distance,
                                waypoints: path
                                    .waypoints
                                    .iter()
                                    .map(|w| (w.latitude, w.longitude))
                                    .collect(),
                            });
                            shared.vd_path_seq += 1;
                            Response::from_data(Vec::new())
                        }
                        Err(e) => {
                            eprintln!("[WARN] rejected verticalDisplayPath POST: {e}");
                            Response::from_string(format!("invalid path: {e}")).with_status_code(400)
                        }
                    }
                }
                (Method::Get, "/debug/status") => {
                    let shared = self.shared.lock().unwrap();
                    let status = serde_json::to_string(&shared.status).unwrap_or_default();
                    let body = format!(
                        "{{\"statusSeq\":{},\"simConnected\":{},\"simPaused\":{},\"httpOverride\":{},\"txFrameCount\":{},\"status\":{status}}}",
                        shared.status_seq,
                        shared.sim_connected,
                        shared.sim_paused,
                        shared.http_override_until.is_some(),
                        shared.tx_frame_count,
                    );
                    Response::from_data(body.into_bytes()).with_header(json_header())
                }
                (Method::Get, "/health") => {
                    let shared = self.shared.lock().unwrap();
                    let body = format!(
                        "{{\"status\":\"ok\",\"simConnected\":{},\"uptimeMs\":{}}}",
                        shared.sim_connected,
                        self.started.elapsed().as_millis()
                    );
                    Response::from_data(body.into_bytes()).with_header(json_header())
                }
                (Method::Post, "/shutdown") => {
                    self.shared.lock().unwrap().shutdown = true;
                    Response::from_data(Vec::new())
                }
                _ => Response::from_string("not found").with_status_code(404),
            };

            let _ = request.respond(response);
        }
    }
}
