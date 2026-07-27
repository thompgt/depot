//! Speed-scaled protective and warning field extents.

use crate::config::SafetyConfig;
use crate::types::Mode;

/// The field dimensions in force for one control cycle.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FieldExtent {
    /// Forward length of the protective field, metres. An obstacle inside it stops
    /// the robot.
    pub protective_len: f32,
    /// Forward length of the warning field, metres. An obstacle inside it clamps
    /// speed. Always at least `protective_len`.
    pub warning_len: f32,
    /// Lateral half-extent shared by both fields, metres.
    pub half_width: f32,
}

impl FieldExtent {
    /// Sizes the fields for a given speed and mode.
    ///
    /// The protective length is the distance the robot covers before it can stop:
    ///
    /// ```text
    /// length = min_protective + v·t_reaction + v² / (2·a_decel) + margin
    /// ```
    ///
    /// The `v²` term is why a fast robot needs a much longer field than a slow one, and
    /// why scaling with speed is not a nicety — a fixed field sized for walking pace is
    /// a collision at full speed, and one sized for full speed makes the robot useless
    /// in a corridor.
    ///
    /// `speed` is taken as an absolute value, so reverse motion is sized as
    /// conservatively as forward motion even though the field itself faces forward.
    #[must_use]
    pub fn compute(cfg: &SafetyConfig, speed: f32, mode: Mode) -> Self {
        // A non-finite speed estimate must not produce a non-finite field. Falling back
        // to the configured ceiling yields the largest, most conservative field.
        let v = if speed.is_finite() { libm::fabsf(speed) } else { cfg.max_linear };
        let v = v.min(cfg.max_linear);

        let stopping = v * cfg.reaction_time_s + (v * v) / (2.0 * cfg.max_decel);
        let protective = cfg.min_protective_m + stopping + cfg.protective_margin_m;

        match mode {
            Mode::Normal => Self {
                protective_len: protective,
                warning_len: protective * cfg.warning_scale,
                half_width: cfg.half_width_m,
            },
            Mode::Docking => {
                // Shortened and narrowed, never zero: the robot still stops for anything
                // closer than the docking field, it just tolerates the shelf ahead.
                let protective = protective.min(cfg.docking.max_protective_m);
                Self {
                    protective_len: protective,
                    warning_len: protective * cfg.warning_scale,
                    half_width: cfg.docking.half_width_m,
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_stationary_robot_still_has_a_field() {
        let cfg = SafetyConfig::default();
        let f = FieldExtent::compute(&cfg, 0.0, Mode::Normal);
        assert!(f.protective_len >= cfg.min_protective_m);
    }

    #[test]
    fn the_field_grows_with_speed() {
        let cfg = SafetyConfig::default();
        let slow = FieldExtent::compute(&cfg, 0.2, Mode::Normal);
        let fast = FieldExtent::compute(&cfg, 1.2, Mode::Normal);
        assert!(fast.protective_len > slow.protective_len);
    }

    #[test]
    fn the_field_covers_the_true_stopping_distance() {
        let cfg = SafetyConfig::default();
        for i in 0..=12 {
            let v = 0.1 * i as f32;
            let f = FieldExtent::compute(&cfg, v, Mode::Normal);
            let physical = v * cfg.reaction_time_s + (v * v) / (2.0 * cfg.max_decel);
            assert!(
                f.protective_len >= physical,
                "v={v}: field {} shorter than stopping distance {physical}",
                f.protective_len
            );
        }
    }

    #[test]
    fn the_warning_field_encloses_the_protective_field() {
        let cfg = SafetyConfig::default();
        for mode in [Mode::Normal, Mode::Docking] {
            for i in 0..=12 {
                let f = FieldExtent::compute(&cfg, 0.1 * i as f32, mode);
                assert!(f.warning_len >= f.protective_len);
            }
        }
    }

    #[test]
    fn docking_narrows_the_field_but_never_removes_it() {
        let cfg = SafetyConfig::default();
        let normal = FieldExtent::compute(&cfg, 0.15, Mode::Normal);
        let docking = FieldExtent::compute(&cfg, 0.15, Mode::Docking);
        assert!(docking.protective_len < normal.protective_len);
        assert!(docking.half_width < normal.half_width);
        assert!(docking.protective_len > 0.0);
        assert!(docking.half_width > 0.0);
    }

    #[test]
    fn a_garbage_speed_estimate_yields_the_largest_field() {
        let cfg = SafetyConfig::default();
        let worst = FieldExtent::compute(&cfg, cfg.max_linear, Mode::Normal);
        for bad in [f32::NAN, f32::INFINITY, -f32::INFINITY, 1e9] {
            let f = FieldExtent::compute(&cfg, bad, Mode::Normal);
            assert!(f.protective_len.is_finite());
            assert!((f.protective_len - worst.protective_len).abs() < 1e-6, "{bad}");
        }
    }
}
