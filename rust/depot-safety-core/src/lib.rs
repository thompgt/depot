//! Deterministic safety and velocity-arbitration core for the Depot AMR fleet.
//!
//! This crate sits *below* the navigation stack. It consumes a raw lidar scan and a
//! requested velocity and decides what actually reaches the motors. Navigation can be
//! wrong; this layer is what keeps wrong from becoming dangerous.
//!
//! # Guarantees
//!
//! * **No heap allocation.** Nothing here allocates, at init or after. Scan geometry
//!   lookup tables live in fixed-size arrays owned by the caller's [`Arbiter`].
//! * **No I/O and no clock reads.** Time enters through [`Tick::now_us`]. The same
//!   inputs against the same state always produce the same [`Decision`], which is what
//!   makes the layer testable and replayable.
//! * **Bounded work.** Every loop is bounded by the ray count, itself bounded by
//!   [`MAX_RAYS`]. There is no recursion and no unbounded iteration.
//! * **No panics on the hot path.** [`Arbiter::step`] performs no indexing that can
//!   go out of bounds and no arithmetic that can overflow in release.
//! * **`no_std`.** The same logic can run on an embedded MCU. `std` is linked only
//!   for tests.
//!
//! # Order of authority
//!
//! When several rules apply, the most restrictive wins. Reported precedence, highest
//! first, is E-stop, missing or stale scan, protective field, command watchdog,
//! warning field, then externally supplied zone limits.
//!
//! ```
//! use depot_safety_core::{Arbiter, Command, FieldState, SafetyConfig, Tick, Twist};
//!
//! let mut arbiter = Arbiter::new(SafetyConfig::default()).unwrap();
//! // No scan has ever arrived, so the layer refuses to move.
//! let decision = arbiter.step(&Tick::new(0));
//! assert_eq!(decision.twist, Twist::ZERO);
//! assert_eq!(decision.state, FieldState::ProtectiveStop);
//! # let _ = Command::new(Twist::new(1.0, 0.0), 0);
//! ```

#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]
#![deny(missing_docs)]
// The library is held to no-panic, no-indexing, no-float-equality discipline because it
// runs inside a 10 ms deadline on a machine that can hurt someone. Tests are held to
// the opposite standard: a test that cannot panic cannot fail, and exact float equality
// is precisely what the determinism claim requires.
#![cfg_attr(
    test,
    allow(
        clippy::expect_used,
        clippy::float_cmp,
        clippy::indexing_slicing,
        clippy::panic,
        clippy::unwrap_used
    )
)]

pub mod arbiter;
pub mod config;
pub mod field;
pub mod geometry;
pub mod scan;
pub mod state;
pub mod types;

pub use arbiter::{Arbiter, Decision, Tick, VetoReason};
pub use config::{ConfigError, DockingConfig, SafetyConfig};
pub use field::FieldExtent;
pub use scan::{ScanError, ScanGeometry, ScanVerdict, MAX_RAYS};
pub use state::FieldState;
pub use types::{Command, Micros, Mode, Scan, Twist, ZoneLimits};
