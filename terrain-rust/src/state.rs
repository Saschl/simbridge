//! Aircraft / EFIS state types shared across the pipeline, mirroring
//! `apps/server/src/terrain/types/msfstypes.ts`.

use serde::{Deserialize, Deserializer, Serialize};

/// Continuous quantities stay f64 like the TS `number` fields; only genuinely
/// discrete enum-like fields are integers, tolerantly truncated from any JSON
/// number (the NestJS DTOs type them as `number` too).
fn js_int_u8<'de, D: Deserializer<'de>>(deserializer: D) -> Result<u8, D::Error> {
    f64::deserialize(deserializer).map(|value| value as u8)
}

/// `TerrainRenderingMode` bitfield values.
pub const RENDERING_MODE_SCANLINE: u8 = 1;
pub const RENDERING_MODE_VERTICAL_DISPLAY_REQUIRED: u8 = 2;

/// ND geometry constants (`processing/generic/constants.ts`).
pub const ND_MAX_WIDTH: usize = 768;
pub const ND_ARC_MODE_WIDTH: usize = 756;
pub const ND_ROSE_MODE_WIDTH: usize = 678;
pub const ND_ARC_HEIGHT_A32NX: usize = 492;
pub const ND_ROSE_HEIGHT_A32NX: usize = 250;
pub const ND_ARC_HEIGHT_A380X: usize = 592;
pub const ND_ROSE_HEIGHT_A380X: usize = 592;
pub const ND_CENTER_OFFSET_Y_A32NX: usize = 0;
pub const ND_ARC_CENTER_OFFSET_Y_A380X: usize = 100;
pub const ND_ROSE_CENTER_OFFSET_Y_A380X: usize = 342;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum Side {
    #[serde(rename = "L")]
    Left,
    #[serde(rename = "R")]
    Right,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct EfisData {
    pub nd_range: f64,
    pub arc_mode: bool,
    pub terr_on_nd: bool,
    pub terr_on_vd: bool,
    #[serde(deserialize_with = "js_int_u8")]
    pub efis_mode: u8,
    pub vd_range_lower: f64,
    pub vd_range_upper: f64,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct AircraftStatus {
    pub adiru_data_valid: bool,
    pub taws_inop: bool,
    pub latitude: f64,
    pub longitude: f64,
    pub altitude: f64,
    pub heading: f64,
    pub vertical_speed: f64,
    pub gear_is_down: bool,
    pub runway_data_valid: bool,
    pub runway_latitude: f64,
    pub runway_longitude: f64,
    pub efis_data_capt: EfisData,
    #[serde(rename = "efisDataFO")]
    pub efis_data_fo: EfisData,
    #[serde(deserialize_with = "js_int_u8")]
    pub navigation_display_rendering_mode: u8,
    pub manual_azim_enabled: bool,
    pub manual_azim_degrees: f64,
    pub ground_truth_latitude: f64,
    pub ground_truth_longitude: f64,
}

impl AircraftStatus {
    pub fn efis(&self, side: Side) -> &EfisData {
        match side {
            Side::Left => &self.efis_data_capt,
            Side::Right => &self.efis_data_fo,
        }
    }

    pub fn scanline_mode(&self) -> bool {
        self.navigation_display_rendering_mode & RENDERING_MODE_SCANLINE != 0
    }

    pub fn vertical_display_required(&self) -> bool {
        self.navigation_display_rendering_mode & RENDERING_MODE_VERTICAL_DISPLAY_REQUIRED != 0
    }
}

/// The aircraft status the worker seeds itself with before the first real
/// update arrives (Innsbruck, `terrainworker.ts` constructor).
pub fn startup_status() -> AircraftStatus {
    let efis = EfisData {
        nd_range: 10.0,
        arc_mode: true,
        terr_on_nd: false,
        terr_on_vd: false,
        efis_mode: 0,
        vd_range_lower: -500.0,
        vd_range_upper: 24000.0,
    };
    AircraftStatus {
        adiru_data_valid: true,
        taws_inop: false,
        latitude: 47.26081,
        longitude: 11.34966,
        altitude: 1904.0,
        heading: 260.0,
        vertical_speed: 0.0,
        gear_is_down: true,
        runway_data_valid: true,
        runway_latitude: 47.26081,
        runway_longitude: 11.34966,
        efis_data_capt: efis.clone(),
        efis_data_fo: efis,
        navigation_display_rendering_mode: 0,
        manual_azim_enabled: false,
        manual_azim_degrees: 0.0,
        ground_truth_latitude: 47.26081,
        ground_truth_longitude: 11.34966,
    }
}

/// Per-cycle ND map geometry (the `mapWidth/mapHeight/mapOffsetX/centerOffsetY`
/// fields `startNewMapCycle` writes into the EFIS config).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct NdMapGeometry {
    pub width: usize,
    pub height: usize,
    pub center_offset_y: usize,
    /// X offset when blitting into the 768-wide screen frame (centered).
    pub offset_x: usize,
}

#[cfg(test)]
mod tests {
    use super::AircraftStatus;

    #[test]
    fn accepts_js_float_numbers_in_integer_fields() {
        // NestJS forwards JS numbers; integral fields may carry fractions
        let json = r#"{
            "adiruDataValid": true, "tawsInop": false,
            "latitude": 50.03, "longitude": 8.57,
            "altitude": 3487.2, "heading": 204.7, "verticalSpeed": -1200.5,
            "gearIsDown": true, "runwayDataValid": false,
            "runwayLatitude": 0, "runwayLongitude": 0,
            "efisDataCapt": {"ndRange": 20.0, "arcMode": true, "terrOnNd": true, "terrOnVd": false,
                             "efisMode": 3.0, "vdRangeLower": -500.0, "vdRangeUpper": 24000.0},
            "efisDataFO": {"ndRange": 40, "arcMode": true, "terrOnNd": false, "terrOnVd": false,
                           "efisMode": 3, "vdRangeLower": -500, "vdRangeUpper": 24000},
            "navigationDisplayRenderingMode": 3.0,
            "manualAzimEnabled": true, "manualAzimDegrees": 204.7,
            "groundTruthLatitude": 50.03, "groundTruthLongitude": 8.57
        }"#;
        let status: AircraftStatus = serde_json::from_str(json).expect("float-typed DTO parses");
        assert_eq!(status.altitude, 3487.2);
        assert_eq!(status.heading, 204.7);
        assert_eq!(status.vertical_speed, -1200.5);
        assert_eq!(status.efis_data_capt.nd_range, 20.0);
        assert_eq!(status.efis_data_capt.efis_mode, 3);
        assert_eq!(status.navigation_display_rendering_mode, 3);
    }
}

pub fn nd_map_geometry(arc_mode: bool, vertical_display_required: bool) -> NdMapGeometry {
    let width = if arc_mode { ND_ARC_MODE_WIDTH } else { ND_ROSE_MODE_WIDTH };
    let (height, center_offset_y) = if vertical_display_required {
        // only the A380X requires the vertical display
        if arc_mode {
            (ND_ARC_HEIGHT_A380X, ND_ARC_CENTER_OFFSET_Y_A380X)
        } else {
            (ND_ROSE_HEIGHT_A380X, ND_ROSE_CENTER_OFFSET_Y_A380X)
        }
    } else if arc_mode {
        (ND_ARC_HEIGHT_A32NX, ND_CENTER_OFFSET_Y_A32NX)
    } else {
        (ND_ROSE_HEIGHT_A32NX, ND_CENTER_OFFSET_Y_A32NX)
    };
    let offset_x = ((ND_MAX_WIDTH - width) as f64 * 0.5).ceil() as usize;

    NdMapGeometry {
        width,
        height,
        center_offset_y,
        offset_x,
    }
}
