//! Velocity arbitration: the single point where a command becomes motion.
//!
//! Everything upstream — Nav2, the fleet service, a teleop joystick — produces a
//! *request*. [`Arbiter::step`] produces the command that actually reaches the motors.
//! Nothing bypasses it.

use crate::config::{ConfigError, SafetyConfig};
use crate::field::FieldExtent;
use crate::scan::{evaluate, ScanError, ScanGeometry, ScanVerdict};
use crate::state::{FieldState, FieldStateMachine};
use crate::types::{clamp_sym, Command, Micros, Mode, Scan, Twist, ZoneLimits};

/// Largest timestep the ramp limiter will honour, seconds.
///
/// If the loop stalls for longer than this, the ramp behaves as if only this much time
/// passed. A stalled loop must not earn the right to a large velocity step.
const MAX_RAMP_DT_S: f32 = 0.05;

/// Why the arbitrated command differs from the requested one.
///
/// Ordered by authority: a larger discriminant overrides a smaller one, and the
/// reported reason is always the highest-authority rule that bound on this cycle.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord)]
#[repr(u8)]
pub enum VetoReason {
    /// The request passed through unmodified.
    #[default]
    None = 0,
    /// Clamped by externally supplied zone limits, typically human proximity.
    ZoneLimit = 1,
    /// Clamped by the docking speed ceiling.
    DockingLimit = 2,
    /// Clamped because an obstacle is in the warning field.
    WarningClamp = 3,
    /// The request contained a non-finite value and was discarded.
    CommandInvalid = 4,
    /// No command has arrived within the watchdog timeout; ramping to zero.
    Watchdog = 5,
    /// An obstacle is in the protective field.
    ProtectiveStop = 6,
    /// The scan is missing, stale, or malformed. The robot is blind, so it stops.
    ScanStale = 7,
    /// The e-stop is asserted or latched.
    EStop = 8,
}

/// One control cycle's worth of input.
///
/// Constructed fresh each cycle by the node binding. Note that everything is optional
/// or defaulted except the timestamp: a cycle where nothing arrived is a normal,
/// well-defined cycle, and the safe response to it is the point of the whole layer.
#[derive(Clone, Copy, Debug)]
pub struct Tick<'a> {
    /// Monotonic time of this cycle, microseconds.
    pub now_us: Micros,
    /// A velocity request, if one arrived since the last cycle.
    pub cmd: Option<Command>,
    /// A lidar scan, if one arrived since the last cycle.
    pub scan: Option<Scan<'a>>,
    /// Whether the e-stop line is asserted right now.
    pub estop: bool,
    /// An explicit operator acknowledgement, permitting a latched e-stop to clear.
    pub estop_reset: bool,
    /// Which field profile applies.
    pub mode: Mode,
    /// Externally imposed speed ceilings.
    pub zone: ZoneLimits,
}

impl<'a> Tick<'a> {
    /// An otherwise-empty cycle at `now_us`.
    #[must_use]
    pub fn new(now_us: Micros) -> Self {
        Self {
            now_us,
            cmd: None,
            scan: None,
            estop: false,
            estop_reset: false,
            mode: Mode::Normal,
            zone: ZoneLimits::UNRESTRICTED,
        }
    }

    /// Attaches a velocity request.
    #[must_use]
    pub fn with_cmd(mut self, cmd: Command) -> Self {
        self.cmd = Some(cmd);
        self
    }

    /// Attaches a lidar scan.
    #[must_use]
    pub fn with_scan(mut self, scan: Scan<'a>) -> Self {
        self.scan = Some(scan);
        self
    }

    /// Sets the field profile.
    #[must_use]
    pub fn with_mode(mut self, mode: Mode) -> Self {
        self.mode = mode;
        self
    }

    /// Sets externally imposed speed ceilings.
    #[must_use]
    pub fn with_zone(mut self, zone: ZoneLimits) -> Self {
        self.zone = zone;
        self
    }
}

/// What the arbiter decided, and enough context to explain why.
///
/// The explanation is not decoration. When a robot stops in the middle of an aisle,
/// the operator needs to know whether it saw something, lost its scan, or lost its
/// planner, and those demand three different responses.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Decision {
    /// The command to send to the motors.
    pub twist: Twist,
    /// The field state this cycle.
    pub state: FieldState,
    /// The highest-authority rule that modified the request.
    pub veto: VetoReason,
    /// The field dimensions used this cycle.
    pub extent: FieldExtent,
    /// Closest valid lidar return, metres; infinite when unknown.
    pub closest_m: f32,
    /// Whether the e-stop is currently latched.
    pub estop_latched: bool,
    /// Why the most recent scan was rejected, if one was.
    ///
    /// Cleared as soon as a usable scan arrives. A misconfigured lidar and a lidar that
    /// has simply gone quiet both surface as [`VetoReason::ScanStale`], and the operator
    /// needs to tell them apart: one is a wiring job and the other is a parameter file.
    pub scan_error: Option<ScanError>,
}

/// The arbitration core.
///
/// Holds the scan-geometry cache, the field state machine, and the small amount of
/// history the watchdog and ramp limiter need. Roughly 8.7 KB, all inline: constructing
/// one allocates nothing and neither does using it.
#[derive(Clone, Debug)]
pub struct Arbiter {
    cfg: SafetyConfig,
    geom: ScanGeometry,
    field: FieldStateMachine,
    /// Last command accepted as valid, held until the watchdog expires.
    last_cmd: Twist,
    last_cmd_us: Option<Micros>,
    /// Verdict from the most recent usable scan, reused between scans.
    last_verdict: ScanVerdict,
    last_scan_us: Option<Micros>,
    last_tick_us: Option<Micros>,
    last_out: Twist,
    estop_latched: bool,
    /// Why the most recent scan was rejected, cleared by the next usable one.
    last_scan_error: Option<ScanError>,
}

impl Arbiter {
    /// Builds an arbiter from a validated configuration.
    ///
    /// # Errors
    ///
    /// Returns the first configuration invariant violated. Validation happens here, once,
    /// so [`Arbiter::step`] never has to ask whether its own parameters make sense.
    pub fn new(cfg: SafetyConfig) -> Result<Self, ConfigError> {
        cfg.validate()?;
        Ok(Self {
            cfg,
            geom: ScanGeometry::default(),
            field: FieldStateMachine::new(),
            last_cmd: Twist::ZERO,
            last_cmd_us: None,
            // Blind until proven otherwise: a freshly started robot has never seen the
            // floor and must not move on the strength of that.
            last_verdict: ScanVerdict::blind(),
            last_scan_us: None,
            last_tick_us: None,
            last_out: Twist::ZERO,
            estop_latched: false,
            last_scan_error: None,
        })
    }

    /// The configuration in force.
    #[must_use]
    pub fn config(&self) -> &SafetyConfig {
        &self.cfg
    }

    /// The most recently emitted command.
    #[must_use]
    pub fn last_output(&self) -> Twist {
        self.last_out
    }

    /// Arbitrates one control cycle.
    ///
    /// Performs a single bounded pass over the scan and a fixed amount of arithmetic.
    /// No allocation, no clock read, no I/O, no recursion, no unbounded loop, and no
    /// panicking path — the same inputs against the same state always yield the same
    /// decision.
    pub fn step(&mut self, tick: &Tick<'_>) -> Decision {
        let dt = self.timestep(tick.now_us);

        // ---- E-stop: latched, and only an explicit acknowledgement clears it -------
        if tick.estop {
            self.estop_latched = true;
        } else if tick.estop_reset {
            self.estop_latched = false;
        }

        // ---- Intake: the request ---------------------------------------------------
        let mut veto = VetoReason::None;
        if let Some(cmd) = tick.cmd {
            if cmd.twist.is_finite() {
                self.last_cmd = cmd.twist;
                // The watchdog measures the age of the freshest *valid* command, so a
                // stream of NaNs looks like silence rather than like health.
                self.last_cmd_us = Some(cmd.stamp_us.min(tick.now_us));
            } else {
                veto = veto.max(VetoReason::CommandInvalid);
                self.last_cmd = Twist::ZERO;
            }
        }
        let request = self.last_cmd;

        // ---- Field sizing ----------------------------------------------------------
        // Sized for the faster of what we are doing and what we have been asked to do,
        // so the field has already grown by the cycle the robot accelerates into it.
        let speed = libm::fabsf(self.last_out.linear).max(libm::fabsf(request.linear));
        let extent = FieldExtent::compute(&self.cfg, speed, tick.mode);

        // ---- Perception ------------------------------------------------------------
        let mut blind = false;
        if let Some(scan) = tick.scan.as_ref() {
            match self.geom.ensure(scan) {
                Ok(()) => {
                    self.last_verdict = evaluate(&self.geom, scan, &extent);
                    self.last_scan_us = Some(scan.stamp_us.min(tick.now_us));
                    self.last_scan_error = None;
                }
                // The *cause* is kept, not just the fact. Telling the operator why the
                // robot stopped is the point of this layer, and "the sensor published
                // 4096 rays" and "the sensor stopped publishing" are different repairs.
                Err(err) => {
                    self.last_scan_error = Some(err);
                    blind = true;
                }
            }
        }
        // Between scans the previous verdict stands. It was computed against a field
        // sized for the speed at that moment; sizing from the requested speed above
        // keeps that conservative, and `scan_timeout_us` bounds how long a stale
        // verdict can be trusted at all.
        match self.last_scan_us {
            Some(t) if tick.now_us.saturating_sub(t) <= self.cfg.scan_timeout_us => {}
            _ => blind = true,
        }
        if blind {
            self.last_verdict = ScanVerdict::blind();
        }

        // A latched e-stop is a stop like any other, and it goes through the same state
        // machine so `Decision.state` can never contradict `Decision.twist`. Forcing
        // *after* the update discards any release timer the update may have started,
        // so the hold only begins on the cycle the operator acknowledges, and re-arming
        // is then governed by the ordinary `clear_hold_us` rather than resuming full
        // speed on the very next cycle.
        let mut state = self.field.update(&self.last_verdict, tick.now_us, self.cfg.clear_hold_us);
        if self.estop_latched {
            self.field.force_stop();
            state = self.field.state();
        }

        // ---- Limits, weakest authority first ---------------------------------------
        let mut target = request.clamped(self.cfg.max_linear, self.cfg.max_angular);

        let zoned = target.clamped(
            self.cfg.max_linear.min(tick.zone.max_linear),
            self.cfg.max_angular.min(tick.zone.max_angular),
        );
        if zoned != target {
            veto = veto.max(VetoReason::ZoneLimit);
            target = zoned;
        }

        if tick.mode == Mode::Docking {
            let docked = target.clamped(self.cfg.docking.max_linear, self.cfg.docking.max_angular);
            if docked != target {
                veto = veto.max(VetoReason::DockingLimit);
                target = docked;
            }
        }

        if state == FieldState::Warning {
            let limit = self.cfg.max_linear * self.cfg.warning_speed_frac;
            let warned = target.clamped(limit, self.cfg.max_angular);
            if warned != target {
                veto = veto.max(VetoReason::WarningClamp);
                target = warned;
            }
        }

        // ---- Stops, strongest authority last ---------------------------------------
        // A watchdog expiry ramps down: the planner has gone quiet, but the floor is
        // still clear, and slamming to zero at speed is its own hazard. A protective
        // breach or an e-stop commands zero on this cycle with no ramp at all.
        let watchdog_expired = match self.last_cmd_us {
            Some(t) => tick.now_us.saturating_sub(t) > self.cfg.watchdog_timeout_us,
            None => true,
        };
        if watchdog_expired {
            veto = veto.max(VetoReason::Watchdog);
            target = Twist::ZERO;
        }

        let hard_stop = state.is_stopped() || self.estop_latched;
        if state.is_stopped() {
            veto = veto.max(if blind { VetoReason::ScanStale } else { VetoReason::ProtectiveStop });
        }
        if self.estop_latched {
            veto = VetoReason::EStop;
        }

        let out = if hard_stop {
            Twist::ZERO
        } else {
            Twist {
                linear: self.ramp_linear(self.last_out.linear, target.linear, dt),
                angular: ramp(
                    self.last_out.angular,
                    target.angular,
                    self.cfg.max_angular_accel * dt,
                ),
            }
        };

        // Belt and braces. Every path above already respects these bounds; this is the
        // one line that has to be true regardless of any mistake in the lines above it.
        let out = out.clamped(self.cfg.max_linear, self.cfg.max_angular);
        let out = if out.is_finite() { out } else { Twist::ZERO };

        self.last_out = out;
        self.last_tick_us = Some(tick.now_us);

        Decision {
            twist: out,
            state,
            veto,
            extent,
            closest_m: self.last_verdict.closest_m,
            estop_latched: self.estop_latched,
            scan_error: self.last_scan_error,
        }
    }

    /// Elapsed time since the previous cycle, seconds, bounded and non-negative.
    fn timestep(&self, now_us: Micros) -> f32 {
        let elapsed = match self.last_tick_us {
            Some(prev) => now_us.saturating_sub(prev),
            None => 0,
        };
        let dt = elapsed as f32 * 1e-6;
        if dt.is_finite() {
            dt.min(MAX_RAMP_DT_S)
        } else {
            0.0
        }
    }

    /// Rate-limits forward velocity, accelerating gently and braking hard.
    fn ramp_linear(&self, current: f32, target: f32, dt: f32) -> f32 {
        let speeding_up = libm::fabsf(target) > libm::fabsf(current);
        let rate = if speeding_up { self.cfg.max_accel } else { self.cfg.max_decel };
        ramp(current, target, rate * dt)
    }
}

/// Moves `current` toward `target` by at most `max_delta`.
fn ramp(current: f32, target: f32, max_delta: f32) -> f32 {
    current + clamp_sym(target - current, max_delta)
}
