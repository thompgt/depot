//! Static configuration for the safety layer.
//!
//! Configuration is validated once at construction and immutable afterwards. There is
//! no runtime tuning path on purpose: a field that can be shrunk while the robot is
//! moving will eventually be shrunk while the robot is moving.

/// Why a [`SafetyConfig`] was rejected.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ConfigError {
    /// A parameter was NaN or infinite.
    NotFinite(&'static str),
    /// A parameter that must be strictly positive was zero or negative.
    NotPositive(&'static str),
    /// A parameter that must be non-negative was negative.
    Negative(&'static str),
    /// The warning field would not enclose the protective field.
    WarningInsideProtective,
    /// The docking field would be wider or longer than the normal field.
    DockingFieldNotNarrower,
    /// A fraction parameter fell outside `[0, 1]`.
    NotAFraction(&'static str),
}

/// The protective-field profile used during shelf docking.
///
/// Docking deliberately drives *at* a known obstacle, so the normal field would latch
/// a protective stop on the shelf itself. The response is a narrower and shorter
/// field plus a hard speed ceiling — never a disabled field.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DockingConfig {
    /// Lateral half-extent of the docking field, metres. Must fit between shelf legs.
    pub half_width_m: f32,
    /// Hard ceiling on the docking protective-field length, metres.
    pub max_protective_m: f32,
    /// Hard ceiling on forward speed while docking, metres per second.
    pub max_linear: f32,
}

impl Default for DockingConfig {
    /// Tuned for a 0.35 m half-width base approaching a shelf with a 0.9 m leg span.
    fn default() -> Self {
        Self { half_width_m: 0.22, max_protective_m: 0.12, max_linear: 0.15 }
    }
}

/// Everything the arbitration core needs to know about the vehicle and its limits.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SafetyConfig {
    /// Absolute forward-speed ceiling, metres per second.
    pub max_linear: f32,
    /// Absolute yaw-rate ceiling, radians per second.
    pub max_angular: f32,
    /// Comfortable forward acceleration used for output ramping, m/s².
    pub max_accel: f32,
    /// Braking capability used both for ramping and for stopping-distance sizing, m/s².
    ///
    /// Size this from *measured* deceleration on the worst floor surface you expect,
    /// not from the motor datasheet. Every field length depends on it.
    pub max_decel: f32,
    /// Yaw acceleration limit used for output ramping, rad/s².
    pub max_angular_accel: f32,
    /// Sensing-plus-compute-plus-actuation latency folded into field length, seconds.
    pub reaction_time_s: f32,
    /// Constant pad added beyond the computed stopping distance, metres.
    pub protective_margin_m: f32,
    /// Field length floor, applied even at a standstill, metres.
    pub min_protective_m: f32,
    /// Warning-field length as a multiple of the protective length. Must exceed 1.
    pub warning_scale: f32,
    /// Lateral half-extent of the field, metres: body half-width plus clearance.
    pub half_width_m: f32,
    /// Fraction of `max_linear` permitted while an obstacle is in the warning field.
    pub warning_speed_frac: f32,
    /// Command age at which the watchdog begins ramping to zero, microseconds.
    pub watchdog_timeout_us: u64,
    /// Scan age beyond which the robot is considered blind, microseconds.
    pub scan_timeout_us: u64,
    /// How long the protective field must stay clear before the stop releases, µs.
    pub clear_hold_us: u64,
    /// The docking profile.
    pub docking: DockingConfig,
}

impl Default for SafetyConfig {
    /// Defaults for a mid-size warehouse AMR: 0.7 m wide, 1.2 m/s, 1.0 m/s² braking.
    fn default() -> Self {
        Self {
            max_linear: 1.2,
            max_angular: 1.5,
            max_accel: 0.8,
            max_decel: 1.0,
            max_angular_accel: 2.0,
            reaction_time_s: 0.12,
            protective_margin_m: 0.10,
            min_protective_m: 0.25,
            warning_scale: 2.0,
            half_width_m: 0.45,
            warning_speed_frac: 0.35,
            watchdog_timeout_us: 100_000,
            scan_timeout_us: 200_000,
            clear_hold_us: 500_000,
            docking: DockingConfig::default(),
        }
    }
}

impl SafetyConfig {
    /// Checks every invariant the rest of the crate relies on.
    ///
    /// # Errors
    ///
    /// Returns the first violated invariant. Called by [`crate::Arbiter::new`], so a
    /// constructed arbiter always holds a valid configuration.
    pub fn validate(&self) -> Result<(), ConfigError> {
        let positive: [(&'static str, f32); 8] = [
            ("max_linear", self.max_linear),
            ("max_angular", self.max_angular),
            ("max_accel", self.max_accel),
            ("max_decel", self.max_decel),
            ("max_angular_accel", self.max_angular_accel),
            ("min_protective_m", self.min_protective_m),
            ("half_width_m", self.half_width_m),
            ("docking.half_width_m", self.docking.half_width_m),
        ];
        for (name, value) in positive {
            if !value.is_finite() {
                return Err(ConfigError::NotFinite(name));
            }
            if value <= 0.0 {
                return Err(ConfigError::NotPositive(name));
            }
        }

        let non_negative: [(&'static str, f32); 4] = [
            ("reaction_time_s", self.reaction_time_s),
            ("protective_margin_m", self.protective_margin_m),
            ("docking.max_protective_m", self.docking.max_protective_m),
            ("docking.max_linear", self.docking.max_linear),
        ];
        for (name, value) in non_negative {
            if !value.is_finite() {
                return Err(ConfigError::NotFinite(name));
            }
            if value < 0.0 {
                return Err(ConfigError::Negative(name));
            }
        }

        if !self.warning_speed_frac.is_finite() {
            return Err(ConfigError::NotFinite("warning_speed_frac"));
        }
        if !(0.0..=1.0).contains(&self.warning_speed_frac) {
            return Err(ConfigError::NotAFraction("warning_speed_frac"));
        }
        if !self.warning_scale.is_finite() {
            return Err(ConfigError::NotFinite("warning_scale"));
        }
        if self.warning_scale <= 1.0 {
            return Err(ConfigError::WarningInsideProtective);
        }
        if self.docking.half_width_m > self.half_width_m {
            return Err(ConfigError::DockingFieldNotNarrower);
        }
        if self.docking.max_linear > self.max_linear {
            return Err(ConfigError::DockingFieldNotNarrower);
        }
        Ok(())
    }
}
