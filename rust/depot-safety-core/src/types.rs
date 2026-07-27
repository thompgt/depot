//! Value types crossing the safety-layer boundary.

/// Monotonic timestamp in microseconds.
///
/// The core never reads a clock. Every timestamp is supplied by the caller, which is
/// what lets a recorded run be replayed bit-for-bit.
pub type Micros = u64;

/// A planar velocity command for a differential-drive base.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Twist {
    /// Forward velocity, metres per second. Positive is forward.
    pub linear: f32,
    /// Yaw rate, radians per second. Positive is counter-clockwise.
    pub angular: f32,
}

impl Twist {
    /// A stationary command.
    pub const ZERO: Self = Self { linear: 0.0, angular: 0.0 };

    /// Constructs a twist.
    #[must_use]
    pub const fn new(linear: f32, angular: f32) -> Self {
        Self { linear, angular }
    }

    /// True when both components are finite.
    ///
    /// A NaN slipping through to the motors is a runaway, so non-finite commands are
    /// rejected rather than clamped.
    #[must_use]
    pub fn is_finite(&self) -> bool {
        self.linear.is_finite() && self.angular.is_finite()
    }

    /// Clamps both components to the given symmetric limits.
    #[must_use]
    pub fn clamped(self, max_linear: f32, max_angular: f32) -> Self {
        Self {
            linear: clamp_sym(self.linear, max_linear),
            angular: clamp_sym(self.angular, max_angular),
        }
    }

    /// Scales both components by `factor`.
    #[must_use]
    pub fn scaled(self, factor: f32) -> Self {
        Self { linear: self.linear * factor, angular: self.angular * factor }
    }
}

/// Clamps `value` into `[-limit, limit]`. `limit` is assumed non-negative.
#[must_use]
pub fn clamp_sym(value: f32, limit: f32) -> f32 {
    if value > limit {
        limit
    } else if value < -limit {
        -limit
    } else {
        value
    }
}

/// A velocity request from the navigation stack, with the time it was produced.
///
/// The stamp is the *origin* time, not the arrival time, so the watchdog measures the
/// age of the freshest command rather than the health of the transport.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Command {
    /// The requested velocity.
    pub twist: Twist,
    /// When the requesting node produced it.
    pub stamp_us: Micros,
}

impl Command {
    /// Constructs a command.
    #[must_use]
    pub const fn new(twist: Twist, stamp_us: Micros) -> Self {
        Self { twist, stamp_us }
    }
}

/// A single planar lidar revolution, borrowed from the caller's buffer.
///
/// Ranges are metres, indexed from `angle_min` in steps of `angle_increment`.
/// Non-finite entries and entries outside `[range_min, range_max]` are treated as
/// no-return, not as obstacles.
#[derive(Clone, Copy, Debug)]
pub struct Scan<'a> {
    /// When the scan was measured.
    pub stamp_us: Micros,
    /// Bearing of `ranges[0]`, radians, robot frame, zero straight ahead.
    pub angle_min: f32,
    /// Bearing step between consecutive rays, radians.
    pub angle_increment: f32,
    /// Minimum trustworthy range, metres.
    pub range_min: f32,
    /// Maximum trustworthy range, metres.
    pub range_max: f32,
    /// Measured ranges, metres.
    pub ranges: &'a [f32],
}

/// Which protective-field profile is in force.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Mode {
    /// Free navigation. The full speed-scaled field applies.
    #[default]
    Normal,
    /// Final approach beneath a shelf.
    ///
    /// A shelf leg is *supposed* to be directly ahead here, so the field is narrowed
    /// and shortened and the speed ceiling is dropped. It is never disabled: the robot
    /// still stops for anything closer than the docking field, and it still cannot
    /// exceed the docking speed limit.
    Docking,
}

/// Externally imposed speed ceilings, typically from human detection upstream.
///
/// These only ever restrict. A zone limit wider than the configured maximum has no
/// effect, so a misbehaving perception node cannot raise the robot's speed.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ZoneLimits {
    /// Maximum permitted forward speed, metres per second.
    pub max_linear: f32,
    /// Maximum permitted yaw rate, radians per second.
    pub max_angular: f32,
}

impl ZoneLimits {
    /// Limits that impose no restriction of their own.
    pub const UNRESTRICTED: Self = Self { max_linear: f32::INFINITY, max_angular: f32::INFINITY };

    /// Constructs zone limits, mapping non-finite or negative values to zero.
    ///
    /// A garbled zone message therefore stops the robot rather than freeing it.
    #[must_use]
    pub fn new(max_linear: f32, max_angular: f32) -> Self {
        Self { max_linear: sanitize_limit(max_linear), max_angular: sanitize_limit(max_angular) }
    }
}

impl Default for ZoneLimits {
    fn default() -> Self {
        Self::UNRESTRICTED
    }
}

fn sanitize_limit(v: f32) -> f32 {
    if v.is_nan() || v < 0.0 {
        0.0
    } else {
        v
    }
}
