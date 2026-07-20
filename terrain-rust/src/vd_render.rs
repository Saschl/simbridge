//! Vertical display: elevation profile sampling along the path corridor and
//! the 540x200 VD image.
//!
//! Port of the `createElevationProfile` kernel
//! (`apps/server/src/terrain/processing/gpu/elevationprofile.ts`), the
//! `renderVerticalDisplay` kernel (`.../gpu/rendering/verticaldisplay.ts`)
//! and the CPU pieces of `processing/verticaldisplayrenderer.ts`.

use crate::fileformat::{ELEV_INVALID, ELEV_UNKNOWN, ELEV_WATER};
use crate::geodesy::{bearing_wgs84, distance_wgs84, project_wgs84, wgs84_to_pixel_coordinate};
use crate::worldmap::WorldMap;

pub const VD_PROFILE_WIDTH: usize = 540;
pub const VD_PROFILE_HEIGHT: usize = 200;

pub const VD_COLOR_TRANSPARENT: [u8; 4] = [0, 0, 0, 0];
pub const VD_COLOR_UNKNOWN: [u8; 4] = [255, 148, 255, 255];
pub const VD_COLOR_GREY: [u8; 4] = [78, 78, 97, 255];
pub const VD_COLOR_WATER: [u8; 4] = [0, 255, 255, 255];
pub const VD_COLOR_TERRAIN: [u8; 4] = [110, 51, 14, 255];

#[derive(Debug, Clone, PartialEq)]
pub struct ElevationProfileConfig {
    /// Full corridor ("hose") width in NM.
    pub path_width: f64,
    pub waypoints: Vec<(f64, f64)>,
    /// VD range in NM (derived from the ND range).
    pub range: f64,
    pub track_changes_significantly_at_distance: f64,
    pub fms_path_used: bool,
}

/// VD range derived from the ND range (`VerticalDisplayRenderer.aircraftStatusUpdate`).
pub fn vd_range_from_nd(nd_range: f64, arc_mode: bool) -> f64 {
    if arc_mode {
        nd_range.clamp(10.0, 160.0)
    } else {
        (nd_range / 2.0).clamp(5.0, 160.0)
    }
}

/// Samples the maximum elevation across the corridor for each of the 540
/// profile pixels. Returns `ELEV_INVALID` beyond the last waypoint and -1000
/// where the corridor has no valid data.
pub fn extract_elevation_profile(
    world: &WorldMap,
    latitude: f64,
    longitude: f64,
    config: &ElevationProfileConfig,
) -> Vec<i16> {
    let distance_per_pixel = config.range / VD_PROFILE_WIDTH as f64;
    let mut profile = vec![ELEV_INVALID; VD_PROFILE_WIDTH];

    for (pixel, out) in profile.iter_mut().enumerate() {
        let distance_for_pixel = distance_per_pixel * pixel as f64;

        // find the route segment containing this distance
        let mut segment_index = config.waypoints.len();
        let mut segment_start_distance = 0.0f64;
        let mut start_lat = latitude;
        let mut start_lon = longitude;
        for (i, &(wp_lat, wp_lon)) in config.waypoints.iter().enumerate() {
            let current_distance = distance_wgs84(start_lat, start_lon, wp_lat, wp_lon);
            if segment_start_distance + current_distance >= distance_for_pixel {
                segment_index = i;
                break;
            }
            segment_start_distance += current_distance;
            start_lat = wp_lat;
            start_lon = wp_lon;
        }
        if segment_index >= config.waypoints.len() {
            *out = ELEV_INVALID;
            continue;
        }

        let remaining_distance = (distance_for_pixel - segment_start_distance) * 1852.0;
        // NOTE: bearing_wgs84 carries the TS +180 deg quirk and is used
        // unchanged, exactly like the original kernel.
        let bearing = bearing_wgs84(
            start_lat,
            start_lon,
            config.waypoints[segment_index].0,
            config.waypoints[segment_index].1,
        );
        let center = project_wgs84(start_lat, start_lon, bearing, remaining_distance);

        // orthogonal corridor endpoints left/right of track
        let mut bearing_start = bearing - 90.0;
        if bearing_start < 0.0 {
            bearing_start += 360.0;
        }
        let mut bearing_end = bearing + 90.0;
        if bearing_end >= 360.0 {
            bearing_end -= 360.0;
        }
        let offset_metres = config.path_width * 1852.0 / 2.0;

        let pixel_of = |brg: f64| -> (i64, i64) {
            let projected = project_wgs84(center.0, center.1, brg, offset_metres);
            wgs84_to_pixel_coordinate(
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
            )
        };
        let start_pixel = pixel_of(bearing_start);
        let end_pixel = pixel_of(bearing_end);

        // modified Bresenham along the corridor line, taking the maximum
        let delta_x = (end_pixel.0 - start_pixel.0).abs();
        let step_x: i64 = if start_pixel.0 < end_pixel.0 { 1 } else { -1 };
        let delta_y = -(end_pixel.1 - start_pixel.1).abs();
        let step_y: i64 = if start_pixel.1 < end_pixel.1 { 1 } else { -1 };
        let mut error = delta_x + delta_y;
        let mut max_elevation: i32 = -1000;
        let (mut x, mut y) = start_pixel;

        loop {
            if y >= 0 && y < world.height as i64 && x >= 0 && x < world.width as i64 {
                let elevation = world.elevations[y as usize * world.width + x as usize] as i32;
                if elevation != ELEV_INVALID as i32
                    && elevation != ELEV_UNKNOWN as i32
                    && elevation > max_elevation
                {
                    max_elevation = elevation;
                }
            }

            if x == end_pixel.0 && y == end_pixel.1 {
                break;
            }

            let error_double = 2 * error;
            if error_double >= delta_y {
                if x == end_pixel.0 {
                    break;
                }
                error += delta_y;
                x += step_x;
            }
            if error_double <= delta_x {
                if y == end_pixel.1 {
                    break;
                }
                error += delta_x;
                y += step_y;
            }
        }

        *out = max_elevation as i16;
    }

    profile
}

/// Renders the 540x200 RGBA vertical display image.
pub fn render_vertical_display(
    profile: &[i16],
    minimum_altitude: f64,
    maximum_altitude: f64,
    grey_background_from_x: f64,
) -> Vec<u8> {
    let mut frame = vec![0u8; VD_PROFILE_WIDTH * VD_PROFILE_HEIGHT * 4];
    let step_y = (maximum_altitude - minimum_altitude) / VD_PROFILE_HEIGHT as f64;

    for y in 0..VD_PROFILE_HEIGHT {
        let altitude = (VD_PROFILE_HEIGHT - y) as f64 * step_y + minimum_altitude;
        for (x, &elevation) in profile.iter().enumerate() {
            let rgba = if elevation == ELEV_INVALID || elevation == ELEV_UNKNOWN {
                VD_COLOR_UNKNOWN
            } else if altitude > elevation as f64 {
                if grey_background_from_x >= 0.0 && x as f64 >= grey_background_from_x {
                    VD_COLOR_GREY
                } else {
                    VD_COLOR_TRANSPARENT
                }
            } else if elevation == ELEV_WATER {
                if altitude <= 0.0 {
                    VD_COLOR_WATER
                } else {
                    VD_COLOR_TRANSPARENT
                }
            } else {
                VD_COLOR_TERRAIN
            };
            frame[(y * VD_PROFILE_WIDTH + x) * 4..(y * VD_PROFILE_WIDTH + x) * 4 + 4]
                .copy_from_slice(&rgba);
        }
    }

    frame
}
