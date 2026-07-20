//! Math helpers matching JavaScript semantics, so the port produces
//! bit-identical results to the original TypeScript implementation.

/// `Math.round` rounds half toward +infinity (e.g. -0.5 -> 0), while Rust's
/// `f64::round` rounds half away from zero (-0.5 -> -1).
pub fn js_round(x: f64) -> f64 {
    (x + 0.5).floor()
}

#[cfg(test)]
mod tests {
    use super::js_round;

    #[test]
    fn matches_js_math_round() {
        assert_eq!(js_round(2.5), 3.0);
        assert_eq!(js_round(-2.5), -2.0);
        assert_eq!(js_round(-0.5), 0.0);
        assert_eq!(js_round(-1410.7612), -1411.0);
        assert_eq!(js_round(1902.8872), 1903.0);
    }
}
