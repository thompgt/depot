//! The 10 ms budget, asserted rather than hoped for.
//!
//! `depot-safety` has a hard 10 ms deadline; missing it means a collision. What is
//! measured here is the *worst* observed cycle at full ray count, not the mean, because
//! the mean is not what hits the wall.
//!
//! This is a floor, not a ceiling: it runs on a loaded developer machine under a
//! non-real-time OS, so it proves the algorithm is nowhere near the budget rather than
//! proving any particular scheduling guarantee. That argument belongs to the ROS 2 node
//! and its thread configuration. What this test catches is the day someone puts
//! something accidentally quadratic in the hot path.
//!
//! Which is why the assertion is on a high percentile rather than the maximum. The
//! observed maximum on Windows is routinely several milliseconds and is entirely
//! scheduler preemption — the same code measured either side of it runs in
//! microseconds. Asserting on the max would test the operating system's mood. The max
//! is printed anyway, because on the real robot that preemption is exactly the thing
//! that has to be engineered away, and pretending not to see it here would be the
//! wrong lesson.

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

use std::time::Instant;

use depot_safety_core::{
    Arbiter, Command, Micros, Mode, SafetyConfig, Scan, Tick, Twist, MAX_RAYS,
};

/// The hard budget from the architecture table.
const BUDGET_US: u128 = 10_000;

/// Ray count of the worst case the core will accept.
fn full_scan() -> [f32; MAX_RAYS] {
    let mut ranges = [0.0_f32; MAX_RAYS];
    for (i, r) in ranges.iter_mut().enumerate() {
        // A mix of near and far returns, so both field tests run for many rays: an
        // all-far scan would exit early on most of them and understate the cost.
        *r = 0.3 + (i % 40) as f32 * 0.1;
    }
    ranges
}

fn scan(ranges: &[f32], stamp_us: Micros) -> Scan<'_> {
    Scan {
        stamp_us,
        angle_min: -2.356_194_5,
        angle_increment: 0.004_363_323, // 270 degrees across 1081 rays
        range_min: 0.05,
        range_max: 25.0,
        ranges,
    }
}

/// Ignored by default, because a debug build is roughly forty times slower and fails
/// the p99 assertion on merit — this file's own header says the number is only
/// meaningful optimised. `cargo test --all-targets` in the ordinary test job would
/// otherwise run it unoptimised on a shared runner and red-line main for no reason.
/// The budgets job runs it explicitly:
///
/// ```text
/// cargo test --release --test latency -- --ignored --nocapture
/// ```
#[test]
#[ignore = "meaningful only against an optimised build; run by the budgets job"]
fn a_full_rate_cycle_fits_inside_the_ten_millisecond_budget() {
    let ranges = full_scan();
    let mut arbiter = Arbiter::new(SafetyConfig::default()).expect("valid config");
    let mut now: Micros = 0;

    // Warm up: pay for the direction-cosine cache build and let the CPU settle before
    // anything is timed.
    for _ in 0..200 {
        now += 10_000;
        let tick = Tick::new(now)
            .with_scan(scan(&ranges, now))
            .with_cmd(Command::new(Twist::new(1.2, 0.3), now));
        arbiter.step(&tick);
    }

    const SAMPLES: usize = 20_000;
    let mut samples_ns = Vec::with_capacity(SAMPLES);

    for i in 0..SAMPLES {
        now += 10_000;
        let mode = if i % 500 == 0 { Mode::Docking } else { Mode::Normal };
        let tick = Tick::new(now)
            .with_scan(scan(&ranges, now))
            .with_cmd(Command::new(Twist::new(1.2, 0.3), now))
            .with_mode(mode);

        let start = Instant::now();
        let decision = arbiter.step(&tick);
        let elapsed = start.elapsed().as_nanos();
        std::hint::black_box(decision);

        samples_ns.push(elapsed);
    }

    samples_ns.sort_unstable();
    let at = |q: f64| -> u128 {
        let idx = ((SAMPLES as f64 - 1.0) * q) as usize;
        samples_ns.get(idx).copied().unwrap_or(0)
    };
    let mean_ns = samples_ns.iter().sum::<u128>() / SAMPLES as u128;
    let p99_us = at(0.99) / 1_000;
    let p999_us = at(0.999) / 1_000;

    let build = if cfg!(debug_assertions) { "debug" } else { "release" };
    println!("{MAX_RAYS} rays, {SAMPLES} cycles, {build} build:");
    println!("  mean   {mean_ns:>9} ns");
    println!("  p50    {:>9} ns", at(0.50));
    println!("  p99    {:>9} ns", at(0.99));
    println!("  p99.9  {:>9} ns", at(0.999));
    println!("  max    {:>9} ns  <- scheduler preemption, not the algorithm", at(1.0));
    println!("  budget {:>9} ns", BUDGET_US * 1_000);

    // Two assertions with two different jobs. p99 is still the algorithm — one
    // preempted cycle in a hundred would be an extraordinarily quiet machine — so it is
    // held to a tenth of the budget: the core should be irrelevant to the deadline, not
    // merely inside it. p99.9 is mostly the operating system, so it only has to fit the
    // budget at all, and it is here to catch a regression bad enough to swamp the noise.
    assert!(
        p99_us < BUDGET_US / 10,
        "p99 cycle {p99_us} us is uncomfortably close to the {BUDGET_US} us budget"
    );
    assert!(p999_us < BUDGET_US, "p99.9 cycle {p999_us} us exceeds the {BUDGET_US} us budget");
}
