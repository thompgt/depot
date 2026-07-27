//! Polar geometry for protective fields.
//!
//! Fields are tested in the sensor's native polar form rather than by converting every
//! ray to Cartesian. For each bearing we ask a single question — *how far away is the
//! field boundary in this direction?* — and compare the measured range against it. That
//! is one division and two comparisons per ray, no square roots, and no per-ray
//! trigonometry once the direction cosines are cached.

/// Range at which a ray leaves a forward-facing rectangle, or `None` if it never
/// enters one.
///
/// The rectangle spans `x ∈ [0, length]` and `y ∈ [-half_width, half_width]` in the
/// robot frame, with the sensor at the origin. `cos_t` and `sin_t` are the direction
/// cosines of the ray's bearing.
///
/// Rays pointing sideways or backwards (`cos_t <= 0`) cannot enter the rectangle,
/// which is what makes the field forward-facing: an obstacle abreast of the robot is
/// not something the robot is about to drive into.
#[must_use]
pub fn forward_rect_boundary(length: f32, half_width: f32, cos_t: f32, sin_t: f32) -> Option<f32> {
    if !(cos_t > 0.0) || length <= 0.0 || half_width <= 0.0 {
        return None;
    }
    // Range at which the ray crosses the plane x = length.
    let front = length / cos_t;
    // If it is still within the side walls when it gets there, the front face is the
    // exit. Otherwise it left through a side wall first.
    let lateral = libm::fabsf(sin_t) * front;
    if lateral <= half_width {
        Some(front)
    } else {
        let side = half_width / libm::fabsf(sin_t);
        Some(side)
    }
}

/// True when a measured return at `(range, bearing)` lies strictly inside the
/// rectangle described by `length` and `half_width`.
#[must_use]
pub fn inside_forward_rect(
    range: f32,
    cos_t: f32,
    sin_t: f32,
    length: f32,
    half_width: f32,
) -> bool {
    match forward_rect_boundary(length, half_width, cos_t, sin_t) {
        Some(boundary) => range < boundary,
        None => false,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const EPS: f32 = 1e-5;

    #[test]
    fn straight_ahead_boundary_is_the_field_length() {
        let b = forward_rect_boundary(2.0, 0.5, 1.0, 0.0).unwrap();
        assert!((b - 2.0).abs() < EPS, "{b}");
    }

    #[test]
    fn rays_behind_the_robot_never_enter_the_field() {
        assert!(forward_rect_boundary(2.0, 0.5, -1.0, 0.0).is_none());
        assert!(forward_rect_boundary(2.0, 0.5, 0.0, 1.0).is_none());
    }

    #[test]
    fn steep_rays_exit_through_the_side_wall() {
        // 45 degrees: the front face is 2 m away along x, so the ray would need to be
        // 2 m off-axis, far outside a 0.5 m half-width.
        let c = core::f32::consts::FRAC_1_SQRT_2;
        let b = forward_rect_boundary(2.0, 0.5, c, c).unwrap();
        let expected = 0.5 / c;
        assert!((b - expected).abs() < EPS, "{b} vs {expected}");
    }

    #[test]
    fn the_boundary_is_symmetric_in_bearing() {
        let c = 0.8_f32;
        let s = 0.6_f32;
        let left = forward_rect_boundary(1.5, 0.4, c, s).unwrap();
        let right = forward_rect_boundary(1.5, 0.4, c, -s).unwrap();
        assert!((left - right).abs() < EPS);
    }

    #[test]
    fn a_point_on_the_boundary_is_outside_the_field() {
        // Strict inequality matters: a wall exactly at the field edge should not latch
        // a stop, or a robot parked at the limit would chatter.
        assert!(!inside_forward_rect(2.0, 1.0, 0.0, 2.0, 0.5));
        assert!(inside_forward_rect(2.0 - 1e-3, 1.0, 0.0, 2.0, 0.5));
    }

    #[test]
    fn a_longer_field_never_excludes_a_point_a_shorter_one_caught() {
        // Monotonicity in length, relied on by the speed-scaling argument.
        for i in 0..64 {
            let theta = -1.0 + 2.0 * (i as f32) / 63.0;
            let (s, c) = (libm::sinf(theta), libm::cosf(theta));
            for j in 0..32 {
                let range = 0.05 + 0.1 * j as f32;
                let short = inside_forward_rect(range, c, s, 1.0, 0.5);
                let long = inside_forward_rect(range, c, s, 2.0, 0.5);
                assert!(!short || long, "theta={theta} range={range}");
            }
        }
    }
}
