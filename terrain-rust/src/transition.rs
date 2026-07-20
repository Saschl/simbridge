//! Map transition animations: the arc-mode angular sweep and scanline-mode
//! vertical sweep of the ND, and the horizontal sweep of the VD.
//!
//! Port of the transition logic in `processing/navigationdisplayrenderer.ts`
//! and `processing/verticaldisplayrenderer.ts`. All frames are RGBA byte
//! buffers; unpainted pixels get the display background fill.

use crate::jsmath::js_round;
use crate::vd_render::{VD_PROFILE_HEIGHT, VD_PROFILE_WIDTH};

pub const TRANSITION_DELTA_TIME_MS: u64 = 40;
pub const ARC_TRANSITION_DURATION_MS: f64 = 1500.0;
pub const SCANLINE_TRANSITION_DURATION_MS: f64 = 600.0;
pub const ARC_UPDATE_TIMEOUT_MS: u64 = 1000;
pub const SCANLINE_UPDATE_TIMEOUT_MS: u64 = 500;
/// Transition duration + update timeout.
pub const FRAME_VALIDITY_ARC_MS: f64 = 2500.0;
pub const FRAME_VALIDITY_SCANLINE_MS: f64 = 1100.0;
/// `round(90 / 1500 * 40)` degrees per 40 ms tick.
const ARC_ANGULAR_STEP: i64 = 2;

/// ND background fill RGBA (4, 4, 5, 0).
pub const ND_BACKGROUND: [u8; 4] = [4, 4, 5, 0];
/// VD background fill RGBA (4, 4, 4, 0).
pub const VD_BACKGROUND: [u8; 4] = [4, 4, 4, 0];

fn blend<F: Fn(usize, usize) -> bool>(
    old_frame: Option<&[u8]>,
    new_frame: &[u8],
    width: usize,
    height: usize,
    background: [u8; 4],
    use_new: F,
) -> Vec<u8> {
    let mut result = Vec::with_capacity(width * height * 4);
    for y in 0..height {
        for x in 0..width {
            let idx = (y * width + x) * 4;
            if use_new(x, y) {
                result.extend_from_slice(&new_frame[idx..idx + 4]);
            } else if let Some(old) = old_frame {
                result.extend_from_slice(&old[idx..idx + 4]);
            } else {
                result.extend_from_slice(&background);
            }
        }
    }
    result
}

/// Pixels within the angular band [start, end] (degrees off the vertical
/// centerline) show the new frame (`arcModeTransitionFrame`).
pub fn arc_mode_frame(
    old_frame: Option<&[u8]>,
    new_frame: &[u8],
    start_angle: i64,
    end_angle: i64,
    width: usize,
    height: usize,
) -> Vec<u8> {
    blend(old_frame, new_frame, width, height, ND_BACKGROUND, |x, y| {
        let dx = x as f64 - width as f64 / 2.0;
        let dy = height as f64 - y as f64;
        let distance = (dx * dx + dy * dy).sqrt();
        let angle = (dy / distance).acos() * (180.0 / std::f64::consts::PI);
        start_angle as f64 <= angle && angle <= end_angle as f64
    })
}

/// Rows between the current border (moving up) and the start border show the
/// new frame (`scanlineModeTransitionFrame`).
pub fn scanline_mode_frame(
    old_frame: Option<&[u8]>,
    new_frame: &[u8],
    start_border: i64,
    current_border: i64,
    width: usize,
    height: usize,
) -> Vec<u8> {
    blend(old_frame, new_frame, width, height, ND_BACKGROUND, |_, y| {
        y as i64 <= start_border && y as i64 >= current_border
    })
}

/// Columns between the start border and the current border (moving right)
/// show the new frame (`VerticalDisplayRenderer.transitionFrame`).
pub fn vd_frame(
    old_frame: Option<&[u8]>,
    new_frame: &[u8],
    start_border: i64,
    current_border: i64,
) -> Vec<u8> {
    blend(
        old_frame,
        new_frame,
        VD_PROFILE_WIDTH,
        VD_PROFILE_HEIGHT,
        VD_BACKGROUND,
        |x, _| x as i64 >= start_border && x as i64 <= current_border,
    )
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TransitionStyle {
    Arc,
    ScanlineNd,
    VerticalDisplay,
}

/// Per-display transition state (the `renderingData` block of the TS
/// renderers, minus the threshold bookkeeping which lives in the orchestrator).
pub struct Transition {
    pub style: TransitionStyle,
    pub start_border: i64,
    pub current_border: i64,
    pub final_frame: Option<Vec<u8>>,
    pub last_frame: Option<Vec<u8>>,
    pub current_frame: Option<Vec<u8>>,
    pub width: usize,
    pub height: usize,
}

impl Transition {
    pub fn new(style: TransitionStyle) -> Self {
        Self {
            style,
            start_border: 0,
            current_border: 0,
            final_frame: None,
            last_frame: None,
            current_frame: None,
            width: 0,
            height: 0,
        }
    }

    pub fn reset(&mut self) {
        self.start_border = 0;
        self.current_border = 0;
        self.final_frame = None;
        self.last_frame = None;
        self.current_frame = None;
    }

    /// Frame validity for the first-activation phase offset.
    fn frame_validity_ms(&self) -> f64 {
        match self.style {
            TransitionStyle::Arc => FRAME_VALIDITY_ARC_MS,
            TransitionStyle::ScanlineNd | TransitionStyle::VerticalDisplay => FRAME_VALIDITY_SCANLINE_MS,
        }
    }

    /// Begin a new cycle with a freshly rendered final frame (the border init
    /// of `startNewMapCycle`). On the first activation the start border is
    /// derived from the elapsed time so the animation appears mid-cycle.
    pub fn start_new_cycle(
        &mut self,
        final_frame: Vec<u8>,
        width: usize,
        height: usize,
        now_ms: u64,
        startup_ms: u64,
    ) {
        self.width = width;
        self.height = height;
        self.final_frame = Some(final_frame);

        self.start_border = if self.last_frame.is_none() {
            let update_count = now_ms.saturating_sub(startup_ms) as f64 / self.frame_validity_ms();
            let ratio = update_count - update_count.floor();
            match self.style {
                TransitionStyle::Arc => (90.0 * ratio).floor() as i64,
                TransitionStyle::ScanlineNd => height as i64 - (height as f64 * ratio).floor() as i64,
                TransitionStyle::VerticalDisplay => (width as f64 * ratio).floor() as i64,
            }
        } else {
            match self.style {
                TransitionStyle::Arc | TransitionStyle::VerticalDisplay => 0,
                TransitionStyle::ScanlineNd => height as i64,
            }
        };
        self.current_border = self.start_border;
    }

    /// Advance one 40 ms tick; returns true when the transition rendered its
    /// last frame (ports `arcModeTransition` / `scanlineModeTransition` /
    /// `VerticalDisplayRenderer.render`).
    pub fn render(&mut self) -> bool {
        let Some(final_frame) = self.final_frame.as_ref() else {
            return true;
        };

        match self.style {
            TransitionStyle::Arc => {
                self.current_border += ARC_ANGULAR_STEP;
                if self.current_border < 90 {
                    self.current_frame = Some(arc_mode_frame(
                        self.last_frame.as_deref(),
                        final_frame,
                        self.start_border,
                        self.current_border,
                        self.width,
                        self.height,
                    ));
                    return false;
                }
                if self.current_border - ARC_ANGULAR_STEP < 90 {
                    self.current_frame = Some(arc_mode_frame(
                        self.last_frame.as_deref(),
                        final_frame,
                        self.start_border,
                        90,
                        self.width,
                        self.height,
                    ));
                }
            }
            TransitionStyle::ScanlineNd => {
                let step = js_round(
                    self.height as f64 / SCANLINE_TRANSITION_DURATION_MS * TRANSITION_DELTA_TIME_MS as f64,
                ) as i64;
                self.current_border -= step;
                if self.current_border > 0 {
                    self.current_frame = Some(scanline_mode_frame(
                        self.last_frame.as_deref(),
                        final_frame,
                        self.start_border,
                        self.current_border,
                        self.width,
                        self.height,
                    ));
                    return false;
                }
                if self.current_border + step >= 0 {
                    self.current_frame = Some(scanline_mode_frame(
                        self.last_frame.as_deref(),
                        final_frame,
                        self.start_border,
                        self.current_border,
                        self.width,
                        self.height,
                    ));
                }
            }
            TransitionStyle::VerticalDisplay => {
                let step = js_round(
                    VD_PROFILE_WIDTH as f64 / SCANLINE_TRANSITION_DURATION_MS * TRANSITION_DELTA_TIME_MS as f64,
                ) as i64;
                self.current_border += step;
                if self.current_border < VD_PROFILE_WIDTH as i64 {
                    self.current_frame = Some(vd_frame(
                        self.last_frame.as_deref(),
                        final_frame,
                        self.start_border,
                        self.current_border,
                    ));
                    return false;
                }
                if self.current_border + step > VD_PROFILE_WIDTH as i64 {
                    self.current_frame = Some(vd_frame(
                        self.last_frame.as_deref(),
                        final_frame,
                        self.start_border,
                        self.current_border,
                    ));
                }
            }
        }

        // keep the completed blend as the base for the next cycle
        self.last_frame = self.current_frame.clone();
        true
    }
}
