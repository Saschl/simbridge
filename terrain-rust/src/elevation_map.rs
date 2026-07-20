//! Local ND elevation map extraction: for every display pixel, back-project
//! through bearing/distance onto the world map and sample it.
//!
//! Port of the `createLocalElevationMap` kernel
//! (`apps/server/src/terrain/processing/gpu/elevationmap.ts`) plus the
//! `metresPerPixel` setup from `MapHandler.createLocalElevationMap`.

use crate::fileformat::{ELEV_INVALID, ELEV_UNKNOWN};
use crate::geodesy::{normalize_heading, project_wgs84, rad2deg, wgs84_to_pixel_coordinate, NM_TO_METRES};
use crate::jsmath::js_round;
use crate::state::NdMapGeometry;
use crate::worldmap::WorldMap;

/// `metresPerPixel` for a cycle (JS `Math.round`, doubled in arc mode; the
/// kernel divides by 2 again so arc mode covers twice the range per pixel).
pub fn metres_per_pixel(nd_range: f64, geometry: &NdMapGeometry, arc_mode: bool) -> f64 {
    let mut mpp = js_round(
        nd_range * NM_TO_METRES / (geometry.height - geometry.center_offset_y) as f64,
    );
    if arc_mode {
        mpp *= 2.0;
    }
    mpp
}

/// Extracts the `width x height` local elevation map, row 0 = top of the
/// display (farthest ahead). Aircraft position is the ADIRU position; the
/// world map carries the ground-truth position/pixel.
pub fn extract_local_elevation_map(
    world: &WorldMap,
    latitude: f64,
    longitude: f64,
    heading: f64,
    geometry: &NdMapGeometry,
    metres_per_pixel: f64,
    arc_mode: bool,
) -> Vec<i16> {
    let width = geometry.width;
    let height = geometry.height;
    let center_x = width as f64 / 2.0;
    let center_offset_y = geometry.center_offset_y as f64;

    let mut map = vec![ELEV_INVALID; width * height];

    for y in 0..height {
        let delta_y = height as f64 - y as f64 - center_offset_y;
        let row = &mut map[y * width..(y + 1) * width];

        for (x, out) in row.iter_mut().enumerate() {
            let delta_x = x as f64 - center_x;
            let distance_pixels = (delta_x * delta_x + delta_y * delta_y).sqrt();

            // cut off the arc shape for the A32NX in arc mode
            if center_offset_y == 0.0 && arc_mode && distance_pixels > height as f64 {
                *out = ELEV_INVALID;
                continue;
            }

            let distance = distance_pixels * (metres_per_pixel / 2.0);
            // The pixel exactly at the projection centre divides 0/0 in the TS
            // kernel (NaN cascade, undefined GPU sampling); define it as the
            // aircraft's own position instead.
            let bearing = if distance_pixels == 0.0 {
                heading
            } else {
                let angle = rad2deg((delta_y / distance_pixels).acos());
                let raw = if x as f64 > center_x { angle } else { 360.0 - angle };
                normalize_heading(raw + heading)
            };

            let projected = project_wgs84(latitude, longitude, bearing, distance);
            let (px, py) = wgs84_to_pixel_coordinate(
                latitude,
                projected.0,
                projected.1,
                world.ground_truth_lat,
                world.ground_truth_lon,
                world.sw_lat,
                world.sw_lon,
                world.ne_lat,
                world.ne_lon,
                world.width,
                world.height,
                world.ego_x,
                world.ego_y,
            );

            *out = if px < 0 || py < 0 || px >= world.width as i64 || py >= world.height as i64 {
                ELEV_UNKNOWN
            } else {
                world.elevations[py as usize * world.width + px as usize]
            };
        }
    }

    map
}
