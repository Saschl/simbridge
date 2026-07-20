//! Screen-resolution frame assembly: blits the ND map (and the VD on the
//! A380X) into the full display frame that is PNG-encoded and streamed.
//!
//! Port of `TerrainWorker.createScreenResolutionFrame`.

use crate::transition::ND_BACKGROUND;

pub const SCREEN_WIDTH: usize = 768;
pub const SCREEN_HEIGHT_WITHOUT_VD: usize = 768;
pub const SCREEN_HEIGHT_WITH_VD: usize = 1024;
pub const ND_MAP_START_OFFSET_Y: usize = 128;
pub const VD_MAP_START_OFFSET_X: usize = 150;
pub const VD_MAP_START_OFFSET_Y: usize = 800;

/// Composites the screen frame. `nd` and `vd` are RGBA buffers of the given
/// map dimensions; either may be absent.
#[allow(clippy::too_many_arguments)]
pub fn compose_screen_frame(
    screen_height: usize,
    nd: Option<&[u8]>,
    nd_width: usize,
    nd_height: usize,
    nd_offset_x: usize,
    vd: Option<&[u8]>,
    vd_width: usize,
    vd_height: usize,
) -> Vec<u8> {
    let mut frame = Vec::with_capacity(SCREEN_WIDTH * screen_height * 4);
    for _ in 0..SCREEN_WIDTH * screen_height {
        frame.extend_from_slice(&ND_BACKGROUND);
    }

    if let Some(nd) = nd {
        for y in 0..nd_height {
            let dst = ((ND_MAP_START_OFFSET_Y + y) * SCREEN_WIDTH + nd_offset_x) * 4;
            let src = y * nd_width * 4;
            frame[dst..dst + nd_width * 4].copy_from_slice(&nd[src..src + nd_width * 4]);
        }
    }

    if let Some(vd) = vd {
        for y in 0..vd_height {
            let dst = ((VD_MAP_START_OFFSET_Y + y) * SCREEN_WIDTH + VD_MAP_START_OFFSET_X) * 4;
            let src = y * vd_width * 4;
            frame[dst..dst + vd_width * 4].copy_from_slice(&vd[src..src + vd_width * 4]);
        }
    }

    frame
}
