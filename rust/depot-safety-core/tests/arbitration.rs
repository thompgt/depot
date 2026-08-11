//! Behavioural tests for the safety layer, written as the scenarios that define
//! "done" for the Rust phase: drive at a wall at full speed and it stops itself; kill
//! the planner mid-drive and it ramps to zero.

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
    Arbiter, Command, Decision, FieldState, Micros, Mode, Tick, Twist, VetoReason, ZoneLimits,
};
use harness::{config, empty_floor, scan_at, shelf_legs, wall_ahead, PERIOD_US};

/// A one-dimensional robot: integrate the arbitrated command, regenerate the scan.
struct Sim {
    arbiter: Arbiter,
    /// Distance from the lidar origin to the wall, metres.
    distance: f32,
    now_us: Micros,
}

impl Sim {
    fn new(distance: f32) -> Self {
        Self {
            arbiter: Arbiter::new(config()).expect("default config is valid"),
            distance,
            now_us: 0,
        }
    }

    /// Advances one control period, feeding a fresh scan and the given request.
    fn step(&mut self, request: Option<Twist>, mode: Mode, zone: ZoneLimits) -> Decision {
        let ranges = wall_ahead(self.distance);
        let scan = scan_at(&ranges, self.now_us);
        let mut tick = Tick::new(self.now_us).with_scan(scan).with_mode(mode).with_zone(zone);
        if let Some(twist) = request {
            tick = tick.with_cmd(Command::new(twist, self.now_us));
        }
        let decision = self.arbiter.step(&tick);
        let dt = PERIOD_US as f32 * 1e-6;
        self.distance -= decision.twist.linear * dt;
        self.now_us += PERIOD_US;
        decision
    }
}

const FULL_SPEED: Twist = Twist::new(1.2, 0.0);

#[test]
fn it_stops_itself_short_of_a_wall_at_full_speed() {
    let mut sim = Sim::new(8.0);
    let mut min_distance = f32::INFINITY;

    for _ in 0..2_000 {
        let d = sim.step(Some(FULL_SPEED), Mode::Normal, ZoneLimits::UNRESTRICTED);
        min_distance = min_distance.min(sim.distance);
        assert!(d.twist.linear >= 0.0, "the layer never commands reverse on its own");
    }

    let cfg = config();
    assert_eq!(sim.arbiter.last_output(), Twist::ZERO, "should have come to rest");
    assert!(min_distance > cfg.protective_margin_m, "clearance {min_distance} m");
    assert!(min_distance < 1.5, "stopped {min_distance} m out, which is uselessly timid");
}

#[test]
fn it_reaches_full_speed_on_an_empty_floor() {
    let ranges = empty_floor();
    let mut arbiter = Arbiter::new(config()).unwrap();
    let mut now = 0;
    for _ in 0..500 {
        let tick =
            Tick::new(now).with_scan(scan_at(&ranges, now)).with_cmd(Command::new(FULL_SPEED, now));
        arbiter.step(&tick);
        now += PERIOD_US;
    }
    let out = arbiter.last_output();
    assert!((out.linear - 1.2).abs() < 1e-3, "expected full speed, got {}", out.linear);
}

#[test]
fn losing_the_planner_mid_drive_ramps_to_zero() {
    let ranges = empty_floor();
    let cfg = config();
    let mut arbiter = Arbiter::new(cfg).unwrap();
    let mut now = 0;

    // Get up to speed.
    for _ in 0..400 {
        let tick =
            Tick::new(now).with_scan(scan_at(&ranges, now)).with_cmd(Command::new(FULL_SPEED, now));
        arbiter.step(&tick);
        now += PERIOD_US;
    }
    let speed_at_failure = arbiter.last_output().linear;
    assert!(speed_at_failure > 1.0);

    // The planner dies. Scans keep arriving; commands do not.
    let failure_us = now;
    let mut previous = speed_at_failure;
    let mut stopped_at = None;
    for _ in 0..400 {
        let d = arbiter.step(&Tick::new(now).with_scan(scan_at(&ranges, now)));
        assert!(d.twist.linear <= previous + 1e-6, "speed must never rise after a dropout");
        previous = d.twist.linear;
        if d.twist.linear == 0.0 && stopped_at.is_none() {
            stopped_at = Some(now);
            assert_eq!(d.veto, VetoReason::Watchdog);
        }
        now += PERIOD_US;
    }

    let stopped_at = stopped_at.expect("must reach zero");
    // Budget: detect within the watchdog timeout, then brake at the configured rate.
    let budget_us =
        cfg.watchdog_timeout_us + (speed_at_failure / cfg.max_decel * 1e6) as u64 + 2 * PERIOD_US;
    assert!(
        stopped_at - failure_us <= budget_us,
        "took {} us, budget {budget_us} us",
        stopped_at - failure_us
    );
}

#[test]
fn a_robot_that_has_never_seen_the_floor_refuses_to_move() {
    let mut arbiter = Arbiter::new(config()).unwrap();
    let d = arbiter.step(&Tick::new(0).with_cmd(Command::new(FULL_SPEED, 0)));
    assert_eq!(d.twist, Twist::ZERO);
    assert_eq!(d.state, FieldState::ProtectiveStop);
    assert_eq!(d.veto, VetoReason::ScanStale);
}

#[test]
fn losing_the_lidar_mid_drive_stops_the_robot() {
    let ranges = empty_floor();
    let cfg = config();
    let mut arbiter = Arbiter::new(cfg).unwrap();
    let mut now = 0;
    for _ in 0..400 {
        let tick =
            Tick::new(now).with_scan(scan_at(&ranges, now)).with_cmd(Command::new(FULL_SPEED, now));
        arbiter.step(&tick);
        now += PERIOD_US;
    }
    assert!(arbiter.last_output().linear > 1.0);

    // Scans stop. Commands keep arriving, so this is specifically a sensor failure.
    let mut d = arbiter.step(&Tick::new(now).with_cmd(Command::new(FULL_SPEED, now)));
    assert_ne!(d.twist, Twist::ZERO, "one missing scan is not yet a fault");
    now += cfg.scan_timeout_us + PERIOD_US;
    d = arbiter.step(&Tick::new(now).with_cmd(Command::new(FULL_SPEED, now)));
    assert_eq!(d.twist, Twist::ZERO, "a blind robot stops");
    assert_eq!(d.veto, VetoReason::ScanStale);
}

#[test]
fn the_estop_latches_until_an_operator_acknowledges_it() {
    let ranges = empty_floor();
    let mut arbiter = Arbiter::new(config()).unwrap();
    let mut now = 0;
    let drive = |now| Tick::new(now).with_cmd(Command::new(FULL_SPEED, now));

    for _ in 0..400 {
        arbiter.step(&drive(now).with_scan(scan_at(&ranges, now)));
        now += PERIOD_US;
    }

    let mut tick = drive(now).with_scan(scan_at(&ranges, now));
    tick.estop = true;
    let d = arbiter.step(&tick);
    assert_eq!(d.twist, Twist::ZERO, "an e-stop is immediate, not ramped");
    assert_eq!(d.veto, VetoReason::EStop);
    now += PERIOD_US;

    // The button is released, but nobody has acknowledged. It stays latched.
    for _ in 0..200 {
        let d = arbiter.step(&drive(now).with_scan(scan_at(&ranges, now)));
        assert_eq!(d.twist, Twist::ZERO);
        assert!(d.estop_latched);
        now += PERIOD_US;
    }

    let mut tick = drive(now).with_scan(scan_at(&ranges, now));
    tick.estop_reset = true;
    let d = arbiter.step(&tick);
    assert!(!d.estop_latched);
}

#[test]
fn an_estop_over_a_clear_floor_still_reads_as_stopped() {
    let ranges = empty_floor();
    let cfg = config();
    let mut arbiter = Arbiter::new(cfg).unwrap();
    let mut now = 0;
    let drive = |now| Tick::new(now).with_cmd(Command::new(FULL_SPEED, now));

    // The floor is clear throughout, so nothing but the e-stop can stop the robot.
    let mut tick = drive(now).with_scan(scan_at(&ranges, now));
    tick.estop = true;
    let d = arbiter.step(&tick);
    assert_eq!(d.twist, Twist::ZERO);
    assert_eq!(d.state, FieldState::ProtectiveStop, "the state must not contradict the output");
    assert_eq!(d.veto, VetoReason::EStop);
    now += PERIOD_US;

    // Acknowledged. Re-arming is governed by the ordinary release hold, not instant.
    let mut tick = drive(now).with_scan(scan_at(&ranges, now));
    tick.estop_reset = true;
    let d = arbiter.step(&tick);
    assert!(!d.estop_latched);
    assert_eq!(d.twist, Twist::ZERO, "release must not resume motion on the same cycle");
    assert_eq!(d.state, FieldState::ProtectiveStop);

    let released_at = now;
    let mut moving_at = None;
    for _ in 0..300 {
        now += PERIOD_US;
        let d = arbiter.step(&drive(now).with_scan(scan_at(&ranges, now)));
        if d.twist.linear > 0.0 {
            moving_at = Some(now);
            break;
        }
    }
    let moving_at = moving_at.expect("the robot must eventually re-arm");
    assert!(
        moving_at - released_at >= cfg.clear_hold_us,
        "re-armed after {} us, hold is {} us",
        moving_at - released_at,
        cfg.clear_hold_us
    );
}

#[test]
fn docking_tolerates_the_shelf_it_is_driving_under_but_still_limits_speed() {
    let cfg = config();
    // Legs 0.35 m off centreline: inside the normal field, outside the docking field.
    let ranges = shelf_legs(0.30, 0.35);
    let creep = Twist::new(0.6, 0.0);

    let run = |mode: Mode| -> Decision {
        let mut arbiter = Arbiter::new(cfg).unwrap();
        let mut now = 0;
        let mut last = None;
        for _ in 0..300 {
            let tick = Tick::new(now)
                .with_scan(scan_at(&ranges, now))
                .with_cmd(Command::new(creep, now))
                .with_mode(mode);
            last = Some(arbiter.step(&tick));
            now += PERIOD_US;
        }
        last.unwrap()
    };

    let normal = run(Mode::Normal);
    assert_eq!(normal.state, FieldState::ProtectiveStop, "the shelf blocks normal driving");
    assert_eq!(normal.twist, Twist::ZERO);

    let docking = run(Mode::Docking);
    assert_ne!(docking.twist.linear, 0.0, "docking must be able to approach the shelf");
    assert!(
        docking.twist.linear <= cfg.docking.max_linear + 1e-6,
        "docking speed {} exceeds the ceiling",
        docking.twist.linear
    );
    assert!(docking.extent.protective_len > 0.0, "the field is narrowed, never disabled");
}

#[test]
fn a_zone_limit_slows_the_robot_and_cannot_speed_it_up() {
    let ranges = empty_floor();
    let cfg = config();
    let mut arbiter = Arbiter::new(cfg).unwrap();
    let mut now = 0;

    let slow_zone = ZoneLimits::new(0.3, 0.5);
    for _ in 0..500 {
        let tick = Tick::new(now)
            .with_scan(scan_at(&ranges, now))
            .with_cmd(Command::new(FULL_SPEED, now))
            .with_zone(slow_zone);
        let d = arbiter.step(&tick);
        assert!(d.twist.linear <= 0.3 + 1e-6);
        now += PERIOD_US;
    }
    assert_eq!(arbiter.last_output().linear, 0.3);

    // A zone claiming a higher ceiling than the vehicle's own cannot raise it.
    let absurd = ZoneLimits::new(99.0, 99.0);
    for _ in 0..500 {
        let tick = Tick::new(now)
            .with_scan(scan_at(&ranges, now))
            .with_cmd(Command::new(Twist::new(50.0, 0.0), now))
            .with_zone(absurd);
        let d = arbiter.step(&tick);
        assert!(d.twist.linear <= cfg.max_linear + 1e-6);
        now += PERIOD_US;
    }
}

#[test]
fn a_garbled_command_is_discarded_rather_than_forwarded() {
    let ranges = empty_floor();
    let mut arbiter = Arbiter::new(config()).unwrap();
    let d = arbiter.step(
        &Tick::new(0)
            .with_scan(scan_at(&ranges, 0))
            .with_cmd(Command::new(Twist::new(f32::NAN, 0.0), 0)),
    );
    assert!(d.twist.is_finite());
    assert_eq!(d.twist, Twist::ZERO);
}

#[test]
fn the_reported_reason_is_the_rule_with_the_most_authority() {
    let cfg = config();
    let ranges = wall_ahead(0.2); // deep inside the protective field
    let mut arbiter = Arbiter::new(cfg).unwrap();
    let d = arbiter.step(
        &Tick::new(0)
            .with_scan(scan_at(&ranges, 0))
            .with_cmd(Command::new(FULL_SPEED, 0))
            .with_zone(ZoneLimits::new(0.1, 0.1)),
    );
    // Both the zone limit and the protective field bind; the field outranks the zone.
    assert_eq!(d.veto, VetoReason::ProtectiveStop);
}
