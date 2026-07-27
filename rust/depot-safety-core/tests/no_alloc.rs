//! Proof that the safety core never touches the heap.
//!
//! "No allocation after init" is easy to claim and easy to break: a stray `Vec`, a
//! `format!` in an error path, a `Box<dyn Error>` three refactors from now. So it is
//! checked mechanically rather than by inspection. A global allocator counts every
//! allocation made while armed, and the arbiter is driven hard through every branch it
//! has — protective stops, e-stops, watchdog expiry, docking, sensor reconfiguration —
//! with the count required to stay at zero.
//!
//! Allocation matters here for a reason specific to this layer: a global allocator is
//! a shared lock and an unbounded-latency operation. A 10 ms hard deadline cannot
//! contain an arbitrary `malloc`.

// The library crate carries `#![forbid(unsafe_code)]`. Installing a custom allocator
// to prove a property *about* that crate necessarily needs `unsafe`, so this test
// target opts back in. It is the only place in the workspace that does.
#![allow(unsafe_code)]

mod harness;

use std::alloc::{GlobalAlloc, Layout, System};
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};

use depot_safety_core::{Arbiter, Command, Mode, SafetyConfig, Tick, Twist, ZoneLimits};
use harness::{empty_floor, scan_at, shelf_legs, wall_ahead, PERIOD_US};

static ARMED: AtomicBool = AtomicBool::new(false);
static ALLOCATIONS: AtomicUsize = AtomicUsize::new(0);

/// The system allocator, with a counter on the way in.
struct CountingAllocator;

// SAFETY: every method forwards to `System` unchanged. The counter is an atomic
// increment and does not allocate, so no re-entrancy is possible.
unsafe impl GlobalAlloc for CountingAllocator {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        if ARMED.load(Ordering::Relaxed) {
            ALLOCATIONS.fetch_add(1, Ordering::Relaxed);
        }
        unsafe { System.alloc(layout) }
    }

    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        unsafe { System.dealloc(ptr, layout) }
    }

    unsafe fn alloc_zeroed(&self, layout: Layout) -> *mut u8 {
        if ARMED.load(Ordering::Relaxed) {
            ALLOCATIONS.fetch_add(1, Ordering::Relaxed);
        }
        unsafe { System.alloc_zeroed(layout) }
    }

    unsafe fn realloc(&self, ptr: *mut u8, layout: Layout, new_size: usize) -> *mut u8 {
        if ARMED.load(Ordering::Relaxed) {
            ALLOCATIONS.fetch_add(1, Ordering::Relaxed);
        }
        unsafe { System.realloc(ptr, layout, new_size) }
    }
}

#[global_allocator]
static ALLOCATOR: CountingAllocator = CountingAllocator;

/// Single test in this binary on purpose: the counter is process-global, so a second
/// test running concurrently would attribute its allocations to this one.
#[test]
fn the_arbiter_never_allocates() {
    // Everything the measured section will touch is built first, outside the guard.
    let cfg = SafetyConfig::default();
    let mut arbiter = Arbiter::new(cfg).expect("valid config");
    let empty = empty_floor();
    let close_wall = wall_ahead(0.3);
    let far_wall = wall_ahead(2.0);
    let legs = shelf_legs(0.3, 0.35);
    // A second sensor geometry, to force a rebuild of the direction-cosine cache
    // inside the measured section.
    let short_scan = [4.0_f32; 61];

    let mut now = 0u64;

    ARMED.store(true, Ordering::SeqCst);
    ALLOCATIONS.store(0, Ordering::SeqCst);

    for cycle in 0..4_000u64 {
        let phase = cycle % 8;
        let ranges: &[f32] = match phase {
            0 | 1 => &empty,
            2 => &far_wall,
            3 => &close_wall,
            4 => &legs,
            5 => &short_scan, // provokes a geometry rebuild, twice per lap
            _ => &empty,
        };
        let mode = if phase == 4 { Mode::Docking } else { Mode::Normal };

        let mut tick = Tick::new(now).with_mode(mode).with_zone(ZoneLimits::new(0.8, 1.0));
        // Skip the scan occasionally to exercise the stale-scan path.
        if phase != 6 {
            tick = tick.with_scan(scan_at(ranges, now));
        }
        // Skip the command occasionally to exercise the watchdog path.
        if phase != 7 {
            tick = tick.with_cmd(Command::new(Twist::new(1.2, 0.4), now));
        }
        tick.estop = cycle % 997 == 0;
        tick.estop_reset = cycle % 997 == 3;

        let decision = arbiter.step(&tick);
        // Consume the result so nothing is optimised away.
        std::hint::black_box(decision);

        now += PERIOD_US;
    }

    ARMED.store(false, Ordering::SeqCst);

    let count = ALLOCATIONS.load(Ordering::SeqCst);
    assert_eq!(count, 0, "the arbiter allocated {count} times across 4000 cycles");
}
