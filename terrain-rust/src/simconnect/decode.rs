//! Decoding of the 46-byte EGPWC aircraft status client data block.
//!
//! Port of `simConnectReceivedClientData` in
//! `apps/server/src/terrain/communication/simconnect.ts`, including the
//! intentional decode quirks. FIXED vs TS: the FO `vdRangeUpper` was typo'd
//! 24500 — both sides use 24000 here.

use crate::state::{AircraftStatus, EfisData};

pub const EGPWC_AIRCRAFT_STATUS_BYTES: usize = 46;

fn f32_at(data: &[u8], offset: usize) -> f64 {
    f32::from_le_bytes(data[offset..offset + 4].try_into().unwrap()) as f64
}

fn i32_at(data: &[u8], offset: usize) -> i32 {
    i32::from_le_bytes(data[offset..offset + 4].try_into().unwrap())
}

fn i16_at(data: &[u8], offset: usize) -> i16 {
    i16::from_le_bytes(data[offset..offset + 2].try_into().unwrap())
}

fn u16_at(data: &[u8], offset: usize) -> u16 {
    u16::from_le_bytes(data[offset..offset + 2].try_into().unwrap())
}

pub fn decode_aircraft_status(data: &[u8; EGPWC_AIRCRAFT_STATUS_BYTES]) -> AircraftStatus {
    let heading = i16_at(data, 13);

    let efis = |base: usize| EfisData {
        nd_range: u16_at(data, base) as f64,
        arc_mode: data[base + 2] != 0,
        terr_on_nd: data[base + 3] != 0,
        // quirk kept from TS: TERR ON VD shares the TERR ON ND byte
        terr_on_vd: data[base + 3] != 0,
        efis_mode: data[base + 4],
        vd_range_lower: -500.0,
        vd_range_upper: 24000.0,
    };

    AircraftStatus {
        adiru_data_valid: data[0] != 0,
        taws_inop: false,
        latitude: f32_at(data, 1),
        longitude: f32_at(data, 5),
        altitude: i32_at(data, 9) as f64,
        heading: heading as f64,
        vertical_speed: i16_at(data, 15) as f64,
        gear_is_down: data[17] != 0,
        runway_data_valid: data[18] != 0,
        runway_latitude: f32_at(data, 19),
        runway_longitude: f32_at(data, 23),
        efis_data_capt: efis(27),
        efis_data_fo: efis(32),
        navigation_display_rendering_mode: data[37],
        // quirks kept from TS: manual azimuth follows the heading
        manual_azim_enabled: true,
        manual_azim_degrees: heading as f64,
        ground_truth_latitude: f32_at(data, 38),
        ground_truth_longitude: f32_at(data, 42),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn decodes_the_46_byte_layout() {
        let mut data = [0u8; EGPWC_AIRCRAFT_STATUS_BYTES];
        data[0] = 1; // adiru valid
        data[1..5].copy_from_slice(&47.26081f32.to_le_bytes());
        data[5..9].copy_from_slice(&11.34966f32.to_le_bytes());
        data[9..13].copy_from_slice(&4500i32.to_le_bytes());
        data[13..15].copy_from_slice(&260i16.to_le_bytes());
        data[15..17].copy_from_slice(&(-1200i16).to_le_bytes());
        data[17] = 1; // gear down
        data[18] = 1; // runway valid
        data[19..23].copy_from_slice(&47.26022f32.to_le_bytes());
        data[23..27].copy_from_slice(&11.34396f32.to_le_bytes());
        // CAPT efis block
        data[27..29].copy_from_slice(&20u16.to_le_bytes());
        data[29] = 1;
        data[30] = 1;
        data[31] = 3;
        // FO efis block
        data[32..34].copy_from_slice(&40u16.to_le_bytes());
        data[34] = 0;
        data[35] = 0;
        data[36] = 2;
        data[37] = 3; // scanline + VD required
        data[38..42].copy_from_slice(&47.26f32.to_le_bytes());
        data[42..46].copy_from_slice(&11.35f32.to_le_bytes());

        let status = decode_aircraft_status(&data);
        assert!(status.adiru_data_valid);
        assert!((status.latitude - 47.26081).abs() < 1e-5);
        assert_eq!(status.altitude, 4500.0);
        assert_eq!(status.heading, 260.0);
        assert_eq!(status.vertical_speed, -1200.0);
        assert!(status.gear_is_down);
        assert!(status.runway_data_valid);
        assert_eq!(status.efis_data_capt.nd_range, 20.0);
        assert!(status.efis_data_capt.arc_mode);
        assert!(status.efis_data_capt.terr_on_nd);
        assert!(status.efis_data_capt.terr_on_vd);
        assert_eq!(status.efis_data_capt.efis_mode, 3);
        assert_eq!(status.efis_data_capt.vd_range_upper, 24000.0);
        assert_eq!(status.efis_data_fo.nd_range, 40.0);
        assert!(!status.efis_data_fo.arc_mode);
        assert_eq!(status.efis_data_fo.vd_range_upper, 24000.0); // FIXED: was 24500 in TS
        assert_eq!(status.navigation_display_rendering_mode, 3);
        assert!(status.manual_azim_enabled);
        assert_eq!(status.manual_azim_degrees, 260.0);
        assert!((status.ground_truth_latitude - 47.26).abs() < 1e-5);
    }
}
