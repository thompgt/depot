//! Shared fixtures for the safety-core test suites.
#![allow(dead_code)]

use depot_safety_core::{Micros, Scan, SafetyConfig};

/// Rays in the synthetic sensor: 270° at 1°, the shape of a typical AMR safety lidar.
pub const RAYS: usize = 271;
/// Bearing of ray zero.
pub const ANGLE_MIN: f32 = -2.356_194_5; // -135°
/// Bearing step.
pub const ANGLE_INC: f32 = 0.017_453_292; // 1°
/// Sensor minimum range.
pub const RANGE_MIN: f32 = 0.05;
/// Sensor maximum range.
pub const RANGE_MAX: f32 = 12.0;
/// Control period: 100 Hz, comfortably inside the 10 ms budget.
pub const PERIOD_US: Micros = 10_000;

/// Bearing of ray `i`.
#[must_use]
pub fn bearing(i: usize) -> f32 {
    ANGLE_MIN + ANGLE_INC * i as f32
}

/// A scan of an empty floor.
#[must_use]
pub fn empty_floor() -> [f32; RAYS] {
    [RANGE_MAX; RAYS]
}

/// A scan of an infinite wall perpendicular to the robot's heading, `distance` ahead.
///
/// Rays that would strike the wall beyond sensor range, or that point away from it,
/// report the maximum range.
#[must_use]
pub fn wall_ahead(distance: f32) -> [f32; RAYS] {
    let mut ranges = [RANGE_MAX; RAYS];
    for (i, r) in ranges.iter_mut().enumerate() {
        let c = libm::cosf(bearing(i));
        if c > 1e-3 {
            let range = distance / c;
            if range < RANGE_MAX {
                *r = range;
            }
        }
    }
    ranges
}

/// A pair of shelf legs at `distance`, offset `half_span` either side of centreline.
///
/// This is the docking case: two narrow returns straddling the robot's path, with
/// clear floor between them.
#[must_use]
pub fn shelf_legs(distance: f32, half_span: f32) -> [f32; RAYS] {
    let mut ranges = [RANGE_MAX; RAYS];
    for (i, r) in ranges.iter_mut().enumerate() {
        let theta = bearing(i);
        let (c, s) = (libm::cosf(theta), libm::sinf(theta));
        if c <= 1e-3 {
            continue;
        }
        // Distance along the ray to the plane x = distance, and its lateral offset.
        let along = distance / c;
        let lateral = libm::fabsf(s * along);
        if libm::fabsf(lateral - half_span) < 0.05 && along < RANGE_MAX {
            *r = along;
        }
    }
    ranges
}

/// Wraps a range buffer in a [`Scan`] stamped at `stamp_us`.
#[must_use]
pub fn scan_at(ranges: &[f32], stamp_us: Micros) -> Scan<'_> {
    Scan {
        stamp_us,
        angle_min: ANGLE_MIN,
        angle_increment: ANGLE_INC,
        range_min: RANGE_MIN,
        range_max: RANGE_MAX,
        ranges,
    }
}

/// The configuration under test: the shipped defaults.
#[must_use]
pub fn config() -> SafetyConfig {
    SafetyConfig::default()
}
