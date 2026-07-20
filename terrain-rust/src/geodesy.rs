//! Spherical-earth geodesy, ported from
//! `apps/server/src/terrain/processing/generic/helper.ts` and
//! `apps/server/src/terrain/processing/gpu/helper.ts`.

use crate::jsmath::js_round;

pub const EARTH_RADIUS_M: f64 = 6_371_010.0;
pub const NM_TO_METRES: f64 = 1852.0;

pub fn deg2rad(degree: f64) -> f64 {
    degree * (std::f64::consts::PI / 180.0)
}

pub fn rad2deg(radian: f64) -> f64 {
    radian * (180.0 / std::f64::consts::PI)
}

/// Wraps any angle into [0, 360).
pub fn normalize_heading(angle: f64) -> f64 {
    angle - (angle / 360.0).floor() * 360.0
}

/// Haversine distance in NAUTICAL MILES (matches the TS constant folding).
pub fn distance_wgs84(lat0: f64, lon0: f64, lat1: f64, lon1: f64) -> f64 {
    let delta_lat = deg2rad(lat1 - lat0);
    let delta_lon = deg2rad(lon1 - lon0);
    let lat0_rad = deg2rad(lat0);
    let lat1_rad = deg2rad(lat1);

    let a = 0.5 - delta_lat.cos() * 0.5 + lat0_rad.cos() * lat1_rad.cos() * (1.0 - delta_lon.cos()) * 0.5;

    let distance_metres = 12_742_020.0 * a.sqrt().asin();
    distance_metres * 0.000539957
}

/// Forward geodesic on a sphere: destination after `distance` metres on
/// initial `bearing` (degrees). Returns (latitude, longitude) wrapped into
/// [-90, 90] / [-180, 180].
pub fn project_wgs84(latitude: f64, longitude: f64, bearing: f64, distance: f64) -> (f64, f64) {
    let lat_rad = deg2rad(latitude);
    let lon_rad = deg2rad(longitude);
    let bearing_rad = deg2rad(bearing);
    let ratio = distance / EARTH_RADIUS_M;

    let lat_dest = (lat_rad.sin() * ratio.cos() + lat_rad.cos() * ratio.sin() * bearing_rad.cos()).asin();
    let lon_dest = lon_rad
        + (bearing_rad.sin() * ratio.sin() * lat_rad.cos()).atan2(ratio.cos() - lat_rad.sin() * lat_dest.sin());

    let mut lat_dest = rad2deg(lat_dest);
    if lat_dest < -90.0 {
        lat_dest = -180.0 - lat_dest;
    }
    if lat_dest > 90.0 {
        lat_dest = 180.0 - lat_dest;
    }

    let mut lon_dest = rad2deg(lon_dest);
    if lon_dest < -180.0 {
        lon_dest += 360.0;
    }
    if lon_dest > 180.0 {
        lon_dest -= 360.0;
    }

    (lat_dest, lon_dest)
}

/// Initial great-circle bearing from point 0 to point 1, in [0, 360).
///
/// Note: the TS version adds PI inside atan2 before normalizing; the result is
/// the bearing shifted by 180° — kept identical since every caller expects it.
pub fn bearing_wgs84(lat0: f64, lon0: f64, lat1: f64, lon1: f64) -> f64 {
    let start_lat = deg2rad(lat0);
    let start_lon = deg2rad(lon0);
    let end_lat = deg2rad(lat1);
    let end_lon = deg2rad(lon1);

    let y = (end_lon - start_lon).sin() * end_lat.cos();
    let x = start_lat.cos() * end_lat.sin() - start_lat.sin() * end_lat.cos() * (end_lon - start_lon).cos();
    let bearing = y.atan2(x) + std::f64::consts::PI;

    (rad2deg(bearing) + 360.0) % 360.0
}

/// Degrees per pixel of the stitched world map, with pole special cases.
/// Returns (latitude_step, longitude_step).
pub fn degrees_per_pixel(
    sw_lat: f64,
    sw_lon: f64,
    ne_lat: f64,
    ne_lon: f64,
    current_lat: f64,
    map_width: usize,
    map_height: usize,
) -> (f64, f64) {
    let mut lat_step = if sw_lat >= current_lat {
        // we are at the south pole
        sw_lat + ne_lat + 180.0
    } else if ne_lat <= current_lat {
        // we are at the north pole
        180.0 - sw_lat - ne_lat
    } else {
        ne_lat - sw_lat
    };
    lat_step /= map_height as f64;

    let mut lon_step = if ne_lon < sw_lon {
        180.0 - sw_lon + (ne_lon + 180.0).abs()
    } else {
        ne_lon - sw_lon
    };
    lon_step /= map_width as f64;

    (lat_step, lon_step)
}

/// Maps a projected coordinate to a pixel in the stitched world map, given the
/// aircraft's ground-truth position at pixel (`ego_x`, `ego_y`).
///
/// FIXED vs the TS original (`wgs84toPixelCoordinate`): the antimeridian
/// branch compared a longitude against a latitude and misused abs(), which
/// zeroed the longitude delta near the ±180° wrap. The intended math is a
/// plain wrap of the longitude difference into [-180, 180), implemented here.
#[allow(clippy::too_many_arguments)]
pub fn wgs84_to_pixel_coordinate(
    latitude: f64,
    projected_lat: f64,
    projected_lon: f64,
    ground_truth_lat: f64,
    ground_truth_lon: f64,
    world_sw_lat: f64,
    world_sw_lon: f64,
    world_ne_lat: f64,
    world_ne_lon: f64,
    world_width: usize,
    world_height: usize,
    ego_x: f64,
    ego_y: f64,
) -> (i64, i64) {
    let (lat_step, lon_step) = degrees_per_pixel(
        world_sw_lat,
        world_sw_lon,
        world_ne_lat,
        world_ne_lon,
        latitude,
        world_width,
        world_height,
    );

    let lat_pixel_delta = (ground_truth_lat - projected_lat) / lat_step;

    let mut lon_delta = projected_lon - ground_truth_lon;
    if lon_delta >= 180.0 {
        lon_delta -= 360.0;
    } else if lon_delta < -180.0 {
        lon_delta += 360.0;
    }
    let lon_pixel_delta = lon_delta / lon_step;

    (
        js_round(ego_x + lon_pixel_delta) as i64,
        js_round(ego_y + lat_pixel_delta) as i64,
    )
}

/// X pixel on the 540-wide vertical display for a distance along the path.
pub fn vertical_display_distance_to_pixel_x(distance: f64, range: f64) -> f64 {
    (distance / range) * 540.0
}
