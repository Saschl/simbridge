//! Encoding of the outbound client data blocks: the 14-byte threshold
//! metadata and the 8 kB PNG frame chunks.
//!
//! Port of `sendNavigationDisplayTerrainMapMetadata` /
//! `sendNavigationDisplayTerrainMapFrame` in
//! `apps/server/src/terrain/communication/simconnect.ts`.

use crate::jsmath::js_round;
use crate::orchestrator::FrameTransmission;

pub const THRESHOLD_BYTES: usize = 14;
pub const FRAME_CHUNK_BYTES: usize = 8192;

pub fn pack_thresholds(tx: &FrameTransmission) -> [u8; THRESHOLD_BYTES] {
    let mut packed = [0u8; THRESHOLD_BYTES];
    // JS Buffer.writeInt16LE applies ToInteger (truncation) to non-integers
    packed[0..2].copy_from_slice(&(tx.minimum_elevation as i16).to_le_bytes());
    packed[2] = tx.minimum_elevation_mode as u8;
    packed[3..5].copy_from_slice(&(tx.maximum_elevation as i16).to_le_bytes());
    packed[5] = tx.maximum_elevation_mode as u8;
    packed[6] = tx.first_frame as u8;
    packed[7..9].copy_from_slice(&(js_round(tx.display_range) as u16).to_le_bytes());
    packed[9] = tx.display_mode;
    packed[10..14].copy_from_slice(&(tx.png.len() as u32).to_le_bytes());
    packed
}

/// Splits the PNG into fixed-size chunks; the last chunk is zero-padded.
pub fn frame_chunks(png: &[u8]) -> impl Iterator<Item = [u8; FRAME_CHUNK_BYTES]> + '_ {
    png.chunks(FRAME_CHUNK_BYTES).map(|chunk| {
        let mut block = [0u8; FRAME_CHUNK_BYTES];
        block[..chunk.len()].copy_from_slice(chunk);
        block
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::nd_render::TerrainLevelMode;
    use std::sync::Arc;

    #[test]
    fn packs_the_14_byte_metadata() {
        let tx = FrameTransmission {
            side: crate::state::Side::Left,
            minimum_elevation: 2537.5,
            minimum_elevation_mode: TerrainLevelMode::Warning,
            maximum_elevation: 10900.0,
            maximum_elevation_mode: TerrainLevelMode::Caution,
            first_frame: true,
            display_range: 20.0,
            display_mode: 3,
            png: Arc::new(vec![0u8; 20000]),
        };
        let packed = pack_thresholds(&tx);
        assert_eq!(i16::from_le_bytes([packed[0], packed[1]]), 2537); // truncated
        assert_eq!(packed[2], 1);
        assert_eq!(i16::from_le_bytes([packed[3], packed[4]]), 10900);
        assert_eq!(packed[5], 2);
        assert_eq!(packed[6], 1);
        assert_eq!(u16::from_le_bytes([packed[7], packed[8]]), 20);
        assert_eq!(packed[9], 3);
        assert_eq!(u32::from_le_bytes([packed[10], packed[11], packed[12], packed[13]]), 20000);
    }

    #[test]
    fn chunks_pngs_into_8k_blocks() {
        let png = vec![7u8; 20000];
        let chunks: Vec<_> = frame_chunks(&png).collect();
        assert_eq!(chunks.len(), 3);
        assert!(chunks[0].iter().all(|&b| b == 7));
        assert!(chunks[2][..20000 - 2 * 8192].iter().all(|&b| b == 7));
        assert!(chunks[2][20000 - 2 * 8192..].iter().all(|&b| b == 0));
    }
}
