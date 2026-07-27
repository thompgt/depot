//! Fixed-capacity lidar scan evaluation.
//!
//! Direction cosines are cached per ray so the per-cycle cost is arithmetic only. The
//! cache is rebuilt when the sensor's angular geometry changes, which in practice
//! happens once at startup.

use crate::field::FieldExtent;
use crate::geometry::forward_rect_boundary;
use crate::types::Scan;

/// Maximum rays the core will consider in one scan.
///
/// Sized for a 270° sensor at 0.25° resolution. Scans longer than this are evaluated
/// up to this many rays and reported as truncated, which the node binding treats as a
/// configuration error rather than silently ignoring the tail.
pub const MAX_RAYS: usize = 1081;

/// Why a scan could not be used.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ScanError {
    /// The scan contained no rays.
    Empty,
    /// `angle_increment` was zero or non-finite.
    BadIncrement,
    /// `angle_min` was non-finite.
    BadAngleMin,
    /// More rays than [`MAX_RAYS`].
    TooManyRays,
}

/// Cached per-ray direction cosines for one sensor geometry.
///
/// Two fixed arrays, no allocation, ~8.6 KB. This lives inside the [`crate::Arbiter`]
/// and is the only sizeable state in the crate.
#[derive(Clone)]
pub struct ScanGeometry {
    angle_min: f32,
    angle_increment: f32,
    count: usize,
    cos: [f32; MAX_RAYS],
    sin: [f32; MAX_RAYS],
}

impl core::fmt::Debug for ScanGeometry {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("ScanGeometry")
            .field("angle_min", &self.angle_min)
            .field("angle_increment", &self.angle_increment)
            .field("count", &self.count)
            .finish_non_exhaustive()
    }
}

impl Default for ScanGeometry {
    fn default() -> Self {
        Self {
            angle_min: 0.0,
            angle_increment: 0.0,
            count: 0,
            cos: [0.0; MAX_RAYS],
            sin: [0.0; MAX_RAYS],
        }
    }
}

impl ScanGeometry {
    /// Number of rays currently cached.
    #[must_use]
    pub fn len(&self) -> usize {
        self.count
    }

    /// True when no geometry has been configured yet.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.count == 0
    }

    /// True when the cache already matches this scan's angular geometry.
    ///
    /// Compared bitwise rather than with a tolerance: these values come from a device
    /// descriptor and are republished unchanged, so any difference at all is a genuine
    /// reconfiguration and worth paying for a rebuild.
    #[must_use]
    pub fn matches(&self, scan: &Scan<'_>) -> bool {
        self.count == scan.ranges.len()
            && self.angle_min.to_bits() == scan.angle_min.to_bits()
            && self.angle_increment.to_bits() == scan.angle_increment.to_bits()
    }

    /// Rebuilds the cache in place for a new sensor geometry.
    ///
    /// Allocates nothing; cost is one `sinf`/`cosf` pair per ray. Called at startup and
    /// on the rare reconfiguration, never on a steady-state cycle.
    ///
    /// # Errors
    ///
    /// Rejects empty, oversized, or non-finite geometries.
    pub fn configure(&mut self, scan: &Scan<'_>) -> Result<(), ScanError> {
        let count = scan.ranges.len();
        if count == 0 {
            return Err(ScanError::Empty);
        }
        if count > MAX_RAYS {
            return Err(ScanError::TooManyRays);
        }
        if !scan.angle_min.is_finite() {
            return Err(ScanError::BadAngleMin);
        }
        if !scan.angle_increment.is_finite() || scan.angle_increment == 0.0 {
            return Err(ScanError::BadIncrement);
        }

        for i in 0..count {
            let theta = scan.angle_min + scan.angle_increment * i as f32;
            // Indexing is bounded by `count <= MAX_RAYS`, checked above.
            if let (Some(c), Some(s)) = (self.cos.get_mut(i), self.sin.get_mut(i)) {
                *c = libm::cosf(theta);
                *s = libm::sinf(theta);
            }
        }
        self.angle_min = scan.angle_min;
        self.angle_increment = scan.angle_increment;
        self.count = count;
        Ok(())
    }

    /// Ensures the cache matches `scan`, rebuilding only if it does not.
    ///
    /// # Errors
    ///
    /// Propagates [`ScanGeometry::configure`] failures.
    pub fn ensure(&mut self, scan: &Scan<'_>) -> Result<(), ScanError> {
        if self.matches(scan) {
            Ok(())
        } else {
            self.configure(scan)
        }
    }
}

/// What a scan says about the current fields.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ScanVerdict {
    /// A return lies inside the protective field.
    pub protective_breach: bool,
    /// A return lies inside the warning field.
    pub warning_breach: bool,
    /// Returns inside the protective field. Saturates at [`u16::MAX`].
    pub protective_hits: u16,
    /// Returns inside the warning field, including protective ones.
    pub warning_hits: u16,
    /// Closest valid return anywhere in the scan, metres. Infinite if there were none.
    ///
    /// Diagnostic only — it is not used to make the stop decision, because "closest
    /// return" and "closest return *in the field*" are different questions and only the
    /// second one matters.
    pub closest_m: f32,
}

impl ScanVerdict {
    /// A verdict for a robot that cannot see: treated as a protective breach.
    ///
    /// Absence of evidence is not evidence of a clear floor. A blind robot stops.
    #[must_use]
    pub fn blind() -> Self {
        Self {
            protective_breach: true,
            warning_breach: true,
            protective_hits: 0,
            warning_hits: 0,
            closest_m: f32::INFINITY,
        }
    }
}

/// Tests every ray against the protective and warning fields.
///
/// Runs in `O(n)` over the ray count with no branches on data beyond range validity,
/// which is what keeps the worst case close to the average case.
///
/// Returns that are non-finite or outside `[range_min, range_max]` are no-returns.
/// This matters: a NaN compared with `<` is quietly false, so an unfiltered NaN would
/// read as "nothing there" instead of "sensor fault". They are excluded explicitly.
#[must_use]
pub fn evaluate(geom: &ScanGeometry, scan: &Scan<'_>, extent: &FieldExtent) -> ScanVerdict {
    let mut v = ScanVerdict { closest_m: f32::INFINITY, ..ScanVerdict::default() };

    let n = geom.count.min(scan.ranges.len());
    for i in 0..n {
        let (Some(&range), Some(&cos_t), Some(&sin_t)) =
            (scan.ranges.get(i), geom.cos.get(i), geom.sin.get(i))
        else {
            continue;
        };
        if !range.is_finite() || range < scan.range_min || range > scan.range_max {
            continue;
        }
        if range < v.closest_m {
            v.closest_m = range;
        }

        // One boundary computation serves both fields: they share a half-width and
        // differ only in length, and the warning length is the larger of the two.
        let Some(warning_boundary) =
            forward_rect_boundary(extent.warning_len, extent.half_width, cos_t, sin_t)
        else {
            continue;
        };
        if range >= warning_boundary {
            continue;
        }
        v.warning_breach = true;
        v.warning_hits = v.warning_hits.saturating_add(1);

        if let Some(protective_boundary) =
            forward_rect_boundary(extent.protective_len, extent.half_width, cos_t, sin_t)
        {
            if range < protective_boundary {
                v.protective_breach = true;
                v.protective_hits = v.protective_hits.saturating_add(1);
            }
        }
    }
    v
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::Scan;

    fn scan_of(ranges: &[f32]) -> Scan<'_> {
        Scan {
            stamp_us: 0,
            angle_min: -core::f32::consts::FRAC_PI_2,
            angle_increment: core::f32::consts::PI / (ranges.len() as f32 - 1.0),
            range_min: 0.05,
            range_max: 10.0,
            ranges,
        }
    }

    fn extent() -> FieldExtent {
        FieldExtent { protective_len: 1.0, warning_len: 2.0, half_width: 0.5 }
    }

    #[test]
    fn an_empty_floor_is_clear() {
        let ranges = [8.0_f32; 181];
        let scan = scan_of(&ranges);
        let mut geom = ScanGeometry::default();
        geom.configure(&scan).unwrap();
        let v = evaluate(&geom, &scan, &extent());
        assert!(!v.protective_breach);
        assert!(!v.warning_breach);
    }

    #[test]
    fn a_wall_dead_ahead_breaches_the_protective_field() {
        let mut ranges = [8.0_f32; 181];
        ranges[90] = 0.5; // index 90 is straight ahead across a 180-degree sweep
        let scan = scan_of(&ranges);
        let mut geom = ScanGeometry::default();
        geom.configure(&scan).unwrap();
        let v = evaluate(&geom, &scan, &extent());
        assert!(v.protective_breach);
        assert!(v.warning_breach, "a protective breach is always a warning breach");
        assert_eq!(v.protective_hits, 1);
    }

    #[test]
    fn an_obstacle_beside_the_robot_is_ignored() {
        let mut ranges = [8.0_f32; 181];
        ranges[0] = 0.2; // hard left, 90 degrees off axis
        ranges[180] = 0.2; // hard right
        let scan = scan_of(&ranges);
        let mut geom = ScanGeometry::default();
        geom.configure(&scan).unwrap();
        let v = evaluate(&geom, &scan, &extent());
        assert!(!v.protective_breach, "the robot is not about to drive sideways");
    }

    #[test]
    fn nan_returns_are_faults_not_clear_floor() {
        let mut ranges = [f32::NAN; 181];
        ranges[90] = 8.0;
        let scan = scan_of(&ranges);
        let mut geom = ScanGeometry::default();
        geom.configure(&scan).unwrap();
        let v = evaluate(&geom, &scan, &extent());
        assert!(!v.protective_breach);
        assert_eq!(v.closest_m, 8.0, "NaN must not become the closest return");
    }

    #[test]
    fn returns_outside_the_sensor_band_are_discarded() {
        let mut ranges = [8.0_f32; 181];
        ranges[90] = 0.01; // below range_min: a self-reflection, not an obstacle
        let scan = scan_of(&ranges);
        let mut geom = ScanGeometry::default();
        geom.configure(&scan).unwrap();
        let v = evaluate(&geom, &scan, &extent());
        assert!(!v.protective_breach);
    }

    #[test]
    fn geometry_is_cached_and_rebuilt_only_when_it_changes() {
        let a = [8.0_f32; 181];
        let b = [8.0_f32; 91];
        let scan_a = scan_of(&a);
        let scan_b = scan_of(&b);
        let mut geom = ScanGeometry::default();
        geom.ensure(&scan_a).unwrap();
        assert!(geom.matches(&scan_a));
        assert!(!geom.matches(&scan_b));
        geom.ensure(&scan_b).unwrap();
        assert_eq!(geom.len(), 91);
    }

    #[test]
    fn malformed_geometry_is_rejected() {
        let ranges = [1.0_f32; 8];
        let mut geom = ScanGeometry::default();
        let mut s = scan_of(&ranges);
        s.angle_increment = 0.0;
        assert_eq!(geom.configure(&s), Err(ScanError::BadIncrement));
        s.angle_increment = 0.01;
        s.angle_min = f32::NAN;
        assert_eq!(geom.configure(&s), Err(ScanError::BadAngleMin));

        let empty: [f32; 0] = [];
        let s = Scan { ranges: &empty, ..scan_of(&ranges) };
        assert_eq!(geom.configure(&s), Err(ScanError::Empty));

        let huge = [1.0_f32; MAX_RAYS + 1];
        let s = Scan { ranges: &huge, ..scan_of(&ranges) };
        assert_eq!(geom.configure(&s), Err(ScanError::TooManyRays));
    }
}
