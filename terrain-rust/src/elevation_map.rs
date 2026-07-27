//! Local ND elevation map extraction: for every display pixel, back-project
//! through bearing/distance onto the world map and sample it.
//!
//! Port of the `createLocalElevationMap` kernel
//! (`apps/server/src/terrain/processing/gpu/elevationmap.ts`) plus the
//! `metresPerPixel` setup from `MapHandler.createLocalElevationMap`.
//!
//! The kernel is the hot loop of a render cycle and the production target is
//! a single-threaded sim module, so the per-pixel forward geodesic of the
//! original is replaced by a **warp grid**: the display->world mapping is a
//! smooth function, so it is evaluated exactly only at the corners of small
//! tiles and bilinearly interpolated in between (~1/13th of the exact
//! evaluations, ~6x faster). Guards keep the approximation honest:
//!
//! * a centre probe per tile — if bilinear interpolation cannot reproduce the
//!   exact mapping at the tile centre to within [`PROBE_TOLERANCE_PX`], the
//!   whole tile is evaluated exactly (this catches the antimeridian wrap,
//!   where the mapping is discontinuous);
//! * a polar guard — above [`POLAR_EXACT_LIMIT_DEG`] projected latitude the
//!   equirectangular longitude mapping degenerates (cos(lat) -> 0), so
//!   interpolation is disabled outright and polar frames are exact.
//!
//! Accuracy (measured over 240 region/geometry/range/heading cases,
//! `benches/extract_ab.rs` enforces budgets): ~0.02% of samples land on a
//! world-map cell adjacent to the exact one (<= ~10 ft elevation delta),
//! almost entirely at the 160 nm range; rendered ND threshold metadata is
//! unchanged, worst rendered-frame difference is ~0.06% of bytes. The exact
//! path itself uses the closed-form bearing
//! (`cos b = (dy*cos_h - dx*sin_h)/r`, `sin b = (dx*cos_h + dy*sin_h)/r`,
//! one formula for both sign branches of the original acos chain); everything
//! downstream keeps the operation order of the `geodesy` helpers. The TS
//! original ran in f32 on the GPU — both deviations are well inside its own
//! noise floor.

use crate::fileformat::{ELEV_INVALID, ELEV_UNKNOWN};
use crate::geodesy::{deg2rad, degrees_per_pixel, rad2deg, EARTH_RADIUS_M, NM_TO_METRES};
use crate::jsmath::js_round;
use crate::state::NdMapGeometry;
use crate::worldmap::WorldMap;

/// Warp tile edge in display pixels. 8 keeps the worst measured sample
/// divergence at 0.28% for a single 160 nm case (16 would be ~2x faster but
/// ~4x the divergence).
const WARP_TILE: usize = 8;

/// Maximum allowed centre-probe error, in world-map pixels, before a tile
/// falls back to exact evaluation.
const PROBE_TOLERANCE_PX: f64 = 0.05;

/// Projected latitude (degrees) beyond which tiles are always exact.
const POLAR_EXACT_LIMIT_DEG: f64 = 80.0;

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

/// Loop-invariant inputs of the display->world mapping. Hoisting them out of
/// `project_wgs84`/`wgs84_to_pixel_coordinate` changes no values — the
/// helpers would recompute exactly these from constant arguments.
struct Warp<'a> {
    world: &'a WorldMap,
    /// sin/cos of the aircraft heading, for the centre pixel whose bearing is
    /// defined as the heading itself.
    sin_heading: f64,
    cos_heading: f64,
    lon_rad: f64,
    sin_lat: f64,
    cos_lat: f64,
    lat_step: f64,
    lon_step: f64,
    center_x: f64,
    height_f: f64,
    center_offset_y: f64,
    half_mpp: f64,
}

impl Warp<'_> {
    /// Exact pre-rounding world coordinates `(u, v)` of a display pixel
    /// (`u = ego_x + lon_pixel_delta`, `v = ego_y + lat_pixel_delta`), plus
    /// the projected latitude in degrees for the polar guard.
    fn exact_uv_lat(&self, x: usize, y: usize) -> (f64, f64, f64) {
        let delta_x = x as f64 - self.center_x;
        let delta_y = self.height_f - y as f64 - self.center_offset_y;
        let distance_pixels = (delta_x * delta_x + delta_y * delta_y).sqrt();

        let distance = distance_pixels * self.half_mpp;
        // The pixel exactly at the projection centre divides 0/0 in the TS
        // kernel (NaN cascade, undefined GPU sampling); define it as the
        // aircraft's own position instead.
        let (sin_bearing, cos_bearing) = if distance_pixels == 0.0 {
            (self.sin_heading, self.cos_heading)
        } else {
            (
                (delta_x * self.cos_heading + delta_y * self.sin_heading) / distance_pixels,
                (delta_y * self.cos_heading - delta_x * self.sin_heading) / distance_pixels,
            )
        };
        let ratio = distance / EARTH_RADIUS_M;
        let cos_ratio = ratio.cos();
        let sin_ratio = ratio.sin();

        let lat_dest = (self.sin_lat * cos_ratio + self.cos_lat * sin_ratio * cos_bearing).asin();
        let lon_dest = self.lon_rad
            + (sin_bearing * sin_ratio * self.cos_lat)
                .atan2(cos_ratio - self.sin_lat * lat_dest.sin());

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

        // FIXED vs the TS original: the antimeridian wrap, see
        // `geodesy::wgs84_to_pixel_coordinate`
        let lat_pixel_delta = (self.world.ground_truth_lat - lat_dest) / self.lat_step;
        let mut lon_delta = lon_dest - self.world.ground_truth_lon;
        if lon_delta >= 180.0 {
            lon_delta -= 360.0;
        } else if lon_delta < -180.0 {
            lon_delta += 360.0;
        }
        let lon_pixel_delta = lon_delta / self.lon_step;

        (
            self.world.ego_x + lon_pixel_delta,
            self.world.ego_y + lat_pixel_delta,
            lat_dest,
        )
    }

    #[inline]
    fn exact_uv(&self, x: usize, y: usize) -> (f64, f64) {
        let (u, v, _) = self.exact_uv_lat(x, y);
        (u, v)
    }

    /// Round to a world pixel and sample; out-of-map reads are Unknown.
    #[inline]
    fn gather(&self, u: f64, v: f64) -> i16 {
        let px = js_round(u) as i64;
        let py = js_round(v) as i64;
        if px < 0 || py < 0 || px >= self.world.width as i64 || py >= self.world.height as i64 {
            ELEV_UNKNOWN
        } else {
            self.world.elevations[py as usize * self.world.width + px as usize]
        }
    }
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

    let lat_rad = deg2rad(latitude);
    let heading_rad = deg2rad(heading);
    let (lat_step, lon_step) = degrees_per_pixel(
        world.sw_lat,
        world.sw_lon,
        world.ne_lat,
        world.ne_lon,
        latitude,
        world.width,
        world.height,
    );
    let warp = Warp {
        world,
        sin_heading: heading_rad.sin(),
        cos_heading: heading_rad.cos(),
        lon_rad: deg2rad(longitude),
        sin_lat: lat_rad.sin(),
        cos_lat: lat_rad.cos(),
        lat_step,
        lon_step,
        center_x: width as f64 / 2.0,
        height_f: height as f64,
        center_offset_y: geometry.center_offset_y as f64,
        half_mpp: metres_per_pixel / 2.0,
    };
    // cut off the arc shape for the A32NX in arc mode
    let arc = geometry.center_offset_y == 0 && arc_mode;
    let height_sq = (height * height) as f64;

    let mut map = vec![ELEV_INVALID; width * height];

    // walk the display in WARP_TILE x WARP_TILE tiles; adjacent tiles share a
    // corner row/column (recomputed, and idempotent to write twice)
    let mut ty = 0usize;
    while ty < height {
        let y0 = ty;
        let y1 = (ty + WARP_TILE).min(height - 1);
        let mut tx = 0usize;
        while tx < width {
            let x0 = tx;
            let x1 = (tx + WARP_TILE).min(width - 1);

            let (u00, v00, l00) = warp.exact_uv_lat(x0, y0);
            let (u10, v10, l10) = warp.exact_uv_lat(x1, y0);
            let (u01, v01, l01) = warp.exact_uv_lat(x0, y1);
            let (u11, v11, l11) = warp.exact_uv_lat(x1, y1);

            let polar =
                l00.abs().max(l10.abs()).max(l01.abs()).max(l11.abs()) > POLAR_EXACT_LIMIT_DEG;

            // centre probe: trust the tile only if bilinear interpolation
            // reproduces the exact mapping at its centre
            let inv_span_x = if x1 > x0 { 1.0 / (x1 - x0) as f64 } else { 0.0 };
            let inv_span_y = if y1 > y0 { 1.0 / (y1 - y0) as f64 } else { 0.0 };
            let interpolate = !polar && {
                let xc = (x0 + x1) / 2;
                let yc = (y0 + y1) / 2;
                let (uc, vc) = warp.exact_uv(xc, yc);
                let fx = (xc - x0) as f64 * inv_span_x;
                let fy = (yc - y0) as f64 * inv_span_y;
                let top = (u00 + (u10 - u00) * fx, v00 + (v10 - v00) * fx);
                let bottom = (u01 + (u11 - u01) * fx, v01 + (v11 - v01) * fx);
                let u = top.0 + (bottom.0 - top.0) * fy;
                let v = top.1 + (bottom.1 - top.1) * fy;
                (u - uc).abs().max((v - vc).abs()) <= PROBE_TOLERANCE_PX
            };

            for y in y0..=y1 {
                let row = &mut map[y * width..(y + 1) * width];
                let delta_y = warp.height_f - y as f64 - warp.center_offset_y;
                let fy = (y - y0) as f64 * inv_span_y;
                let left = (u00 + (u01 - u00) * fy, v00 + (v01 - v00) * fy);
                let right = (u10 + (u11 - u10) * fy, v10 + (v11 - v10) * fy);

                for (x, out) in row[x0..=x1].iter_mut().enumerate().map(|(i, o)| (x0 + i, o)) {
                    if arc {
                        let delta_x = x as f64 - warp.center_x;
                        // exact equivalent of the original
                        // `sqrt(dx^2 + dy^2) > height` test: both sides are
                        // exactly representable and sqrt is monotone
                        if delta_x * delta_x + delta_y * delta_y > height_sq {
                            continue; // outside the arc, stays ELEV_INVALID
                        }
                    }
                    if interpolate {
                        let fx = (x - x0) as f64 * inv_span_x;
                        let u = left.0 + (right.0 - left.0) * fx;
                        let v = left.1 + (right.1 - left.1) * fx;
                        *out = warp.gather(u, v);
                    } else {
                        let (u, v) = warp.exact_uv(x, y);
                        *out = warp.gather(u, v);
                    }
                }
            }

            tx = if x1 == width - 1 { width } else { x1 };
        }
        ty = if y1 == height - 1 { height } else { y1 };
    }

    map
}
