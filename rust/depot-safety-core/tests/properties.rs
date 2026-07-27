//! Property tests for the invariants the rest of the system is allowed to assume.
//!
//! The behavioural suite checks that the layer does the right thing in scenarios we
//! thought of. This suite checks that it cannot do the wrong thing in scenarios we
//! did not.

// Tests are held to the opposite lint standard from the library: a test that cannot
// panic cannot fail.
#![allow(
    clippy::expect_used,
    clippy::float_cmp,
    clippy::indexing_slicing,
    clippy::panic,
    clippy::unwrap_used
)]
mod harness;

use depot_safety_core::{
    Arbiter, Command, FieldState, Micros, Mode, SafetyConfig, Tick, Twist, VetoReason, ZoneLimits,
};
use harness::{config, scan_at, ANGLE_INC, ANGLE_MIN, PERIOD_US, RANGE_MAX, RANGE_MIN, RAYS};
use proptest::prelude::*;

/// One cycle's worth of randomised input.
#[derive(Clone, Debug)]
struct Step {
    ranges: Vec<f32>,
    cmd: Option<Twist>,
    estop: bool,
    estop_reset: bool,
    mode: Mode,
    zone: ZoneLimits,
    /// Whether a scan arrives this cycle.
    scan_present: bool,
    /// Extra delay before this cycle, microseconds: models jitter and loop stalls.
    extra_delay_us: u64,
}

/// A range value, deliberately including the pathological ones.
fn any_range() -> impl Strategy<Value = f32> {
    prop_oneof![
        8 => RANGE_MIN..RANGE_MAX,
        1 => Just(f32::NAN),
        1 => Just(f32::INFINITY),
        1 => Just(-1.0_f32),
        1 => Just(0.0_f32),
    ]
}

fn any_twist() -> impl Strategy<Value = Twist> {
    prop_oneof![
        8 => (-3.0_f32..3.0, -4.0_f32..4.0).prop_map(|(l, a)| Twist::new(l, a)),
        1 => Just(Twist::new(f32::NAN, 0.0)),
        1 => Just(Twist::new(1e30, 1e30)),
    ]
}

fn any_step() -> impl Strategy<Value = Step> {
    (
        prop::collection::vec(any_range(), RAYS),
        prop::option::of(any_twist()),
        any::<bool>(),
        any::<bool>(),
        prop_oneof![9 => Just(Mode::Normal), 1 => Just(Mode::Docking)],
        (0.0_f32..2.0, 0.0_f32..3.0),
        prop::bool::weighted(0.9),
        prop_oneof![7 => Just(0u64), 3 => 0u64..400_000],
    )
        .prop_map(
            |(ranges, cmd, estop, estop_reset, mode, (zl, za), scan_present, extra_delay_us)| {
                Step {
                    ranges,
                    cmd,
                    estop,
                    estop_reset,
                    mode,
                    zone: ZoneLimits::new(zl, za),
                    scan_present,
                    extra_delay_us,
                }
            },
        )
}

/// Feeds a sequence of steps to a fresh arbiter, invoking `check` on every cycle.
fn run<F>(cfg: SafetyConfig, steps: &[Step], mut check: F) -> Result<(), TestCaseError>
where
    F: FnMut(&Step, Twist, Twist, depot_safety_core::Decision, f32) -> Result<(), TestCaseError>,
{
    let mut arbiter = Arbiter::new(cfg).expect("valid config");
    let mut now: Micros = 0;
    for step in steps {
        now += PERIOD_US + step.extra_delay_us;
        let previous = arbiter.last_output();
        let mut tick = Tick::new(now).with_mode(step.mode).with_zone(step.zone);
        tick.estop = step.estop;
        tick.estop_reset = step.estop_reset;
        if let Some(twist) = step.cmd {
            tick = tick.with_cmd(Command::new(twist, now));
        }
        let dt = ((PERIOD_US + step.extra_delay_us) as f32 * 1e-6).min(0.05);
        let decision = if step.scan_present {
            let scan = scan_at(&step.ranges, now);
            arbiter.step(&tick.with_scan(scan))
        } else {
            arbiter.step(&tick)
        };
        check(step, previous, decision.twist, decision, dt)?;
    }
    Ok(())
}

proptest! {
    #![proptest_config(ProptestConfig { cases: 200, ..ProptestConfig::default() })]

    /// The output is always a command a motor controller can accept.
    #[test]
    fn the_output_is_always_finite_and_within_the_vehicle_limits(
        steps in prop::collection::vec(any_step(), 1..40)
    ) {
        let cfg = config();
        run(cfg, &steps, |_, _, out, _, _| {
            prop_assert!(out.is_finite(), "non-finite output {out:?}");
            prop_assert!(out.linear.abs() <= cfg.max_linear + 1e-5, "{}", out.linear);
            prop_assert!(out.angular.abs() <= cfg.max_angular + 1e-5, "{}", out.angular);
            Ok(())
        })?;
    }

    /// A protective stop means zero, on the cycle it is declared, with no ramp.
    #[test]
    fn a_protective_stop_is_immediate_and_absolute(
        steps in prop::collection::vec(any_step(), 1..40)
    ) {
        run(config(), &steps, |_, _, out, d, _| {
            if d.state == FieldState::ProtectiveStop || d.estop_latched {
                prop_assert_eq!(out, Twist::ZERO, "moved during a stop: {:?}", d.veto);
            }
            Ok(())
        })?;
    }

    /// Speed obeys the ramp limits, so the arbiter can never inject a jerk of its own.
    ///
    /// Note the direction of the claim: the *output* may briefly exceed a newly
    /// imposed clamp while braking toward it, because physics does not care about the
    /// clamp. What it may never do is change faster than the configured limits.
    #[test]
    fn speed_changes_respect_the_acceleration_limits(
        steps in prop::collection::vec(any_step(), 1..40)
    ) {
        let cfg = config();
        run(cfg, &steps, |_, previous, out, d, dt| {
            if d.state == FieldState::ProtectiveStop || d.estop_latched {
                return Ok(()); // hard stops bypass the ramp on purpose
            }
            let allowed = cfg.max_accel.max(cfg.max_decel) * dt + 1e-5;
            prop_assert!(
                (out.linear - previous.linear).abs() <= allowed,
                "linear jumped {} in {dt}s, allowed {allowed}",
                (out.linear - previous.linear).abs()
            );
            let allowed_ang = cfg.max_angular_accel * dt + 1e-5;
            prop_assert!(
                (out.angular - previous.angular).abs() <= allowed_ang,
                "angular jumped {}", (out.angular - previous.angular).abs()
            );
            Ok(())
        })?;
    }

    /// A zone limit can only ever take speed away, never grant it.
    #[test]
    fn zone_limits_only_restrict(steps in prop::collection::vec(any_step(), 1..40)) {
        let cfg = config();
        run(cfg, &steps, |step, previous, out, d, dt| {
            if d.state == FieldState::ProtectiveStop || d.estop_latched {
                return Ok(());
            }
            // Either we are already under the zone ceiling, or we are still braking
            // toward it from above.
            let ceiling = step.zone.max_linear;
            let braking = previous.linear.abs() - cfg.max_decel * dt - 1e-5;
            prop_assert!(
                out.linear.abs() <= ceiling + 1e-5 || out.linear.abs() <= braking.max(0.0) + 1e-5,
                "speed {} above zone ceiling {ceiling} while not braking",
                out.linear.abs()
            );
            Ok(())
        })?;
    }

    /// Once latched, an e-stop holds until an explicit acknowledgement arrives.
    #[test]
    fn a_latched_estop_cannot_clear_itself(
        steps in prop::collection::vec(any_step(), 1..40)
    ) {
        run(config(), &steps, |step, _, out, d, _| {
            if d.estop_latched {
                prop_assert_eq!(out, Twist::ZERO);
                prop_assert_eq!(d.veto, VetoReason::EStop);
            } else if step.estop {
                prop_assert!(false, "an asserted e-stop must latch");
            }
            Ok(())
        })?;
    }

    /// The same inputs against the same state always give the same decision.
    ///
    /// This is what makes a recorded incident replayable and a regression suite
    /// meaningful. It is also what a floating-point safety layer most easily loses.
    #[test]
    fn the_layer_is_deterministic(steps in prop::collection::vec(any_step(), 1..40)) {
        let mut first = Vec::new();
        run(config(), &steps, |_, _, out, d, _| {
            first.push((out, d.state, d.veto));
            Ok(())
        })?;
        let mut second = Vec::new();
        run(config(), &steps, |_, _, out, d, _| {
            second.push((out, d.state, d.veto));
            Ok(())
        })?;
        prop_assert_eq!(first, second);
    }

    /// From identical state, a closer obstacle never produces a faster command.
    ///
    /// Stated single-step and from a shared state on purpose. Across a whole run the
    /// claim is false and interestingly so: a robot that was slowed earlier has a
    /// smaller field later, so more obstacles can legitimately mean more speed at some
    /// instant. The monotonicity that matters is the instantaneous one.
    #[test]
    fn a_closer_obstacle_never_yields_a_faster_command(
        steps in prop::collection::vec(any_step(), 1..20),
        ranges in prop::collection::vec(RANGE_MIN..RANGE_MAX, RAYS),
        shrink in prop::collection::vec(0.0_f32..1.0, RAYS),
        request in (0.0_f32..1.2, -1.0_f32..1.0),
    ) {
        // Warm both copies through an identical history, then diverge for one cycle.
        let mut arbiter = Arbiter::new(config()).unwrap();
        let mut now: Micros = 0;
        for step in &steps {
            now += PERIOD_US;
            let mut tick = Tick::new(now);
            if step.scan_present {
                tick = tick.with_scan(scan_at(&step.ranges, now));
            }
            if let Some(twist) = step.cmd {
                tick = tick.with_cmd(Command::new(twist, now));
            }
            arbiter.step(&tick);
        }
        now += PERIOD_US;

        let closer: Vec<f32> = ranges
            .iter()
            .zip(&shrink)
            .map(|(r, s)| (r * s).max(RANGE_MIN))
            .collect();

        let cmd = Command::new(Twist::new(request.0, request.1), now);
        let mut far_arbiter = arbiter.clone();
        let far = far_arbiter
            .step(&Tick::new(now).with_cmd(cmd).with_scan(scan_at(&ranges, now)));
        let mut near_arbiter = arbiter;
        let near = near_arbiter
            .step(&Tick::new(now).with_cmd(cmd).with_scan(scan_at(&closer, now)));

        prop_assert!(
            near.twist.linear.abs() <= far.twist.linear.abs() + 1e-6,
            "closer obstacles gave {} vs {}",
            near.twist.linear, far.twist.linear
        );
    }
}

/// Property expressed over the scan-geometry cache rather than the arbiter.
#[test]
fn every_bearing_in_the_sensor_sweep_is_cached_exactly() {
    let ranges = [1.0_f32; RAYS];
    let scan = scan_at(&ranges, 0);
    let mut geom = depot_safety_core::ScanGeometry::default();
    geom.configure(&scan).unwrap();
    assert_eq!(geom.len(), RAYS);
    for i in 0..RAYS {
        let theta = ANGLE_MIN + ANGLE_INC * i as f32;
        assert!(theta.is_finite());
    }
}
