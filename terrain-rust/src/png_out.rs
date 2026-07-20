//! PNG encoding of RGBA frames (replaces `sharp` in the TS implementation).

use std::io::Cursor;

/// Encodes an RGBA8 buffer as PNG. Fast compression + Sub filtering keeps the
/// encode well under the 40 ms frame budget while staying small enough for the
/// 8 kB SimConnect chunking.
pub fn encode_rgba(width: usize, height: usize, rgba: &[u8]) -> Result<Vec<u8>, png::EncodingError> {
    debug_assert_eq!(rgba.len(), width * height * 4);

    let mut out = Cursor::new(Vec::new());
    {
        let mut encoder = png::Encoder::new(&mut out, width as u32, height as u32);
        encoder.set_color(png::ColorType::Rgba);
        encoder.set_depth(png::BitDepth::Eight);
        encoder.set_compression(png::Compression::Fast);
        encoder.set_filter(png::FilterType::Sub);
        let mut writer = encoder.write_header()?;
        writer.write_image_data(rgba)?;
    }
    Ok(out.into_inner())
}
