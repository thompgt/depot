//! The protective-field state machine.
//!
//! Two properties matter here and they pull in opposite directions. A protective stop
//! must engage on the *first* cycle that sees a breach — no filtering, no averaging,
//! no waiting for confirmation. And it must not release the instant the field looks
//! clear, or a robot nosed up against a pallet will chatter between stopped and
//! creeping as returns flicker in and out. So: instant to engage, deliberate to
//! release.

use crate::scan::ScanVerdict;
use crate::types::Micros;

/// Where the robot stands with respect to its protective fields.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum FieldState {
    /// Nothing in either field. Commands pass through subject to other limits.
    #[default]
    Clear,
    /// Something in the warning field. Speed is clamped.
    Warning,
    /// Something in the protective field, or the robot cannot see. Motion is refused.
    ProtectiveStop,
}

impl FieldState {
    /// True when this state forbids motion outright.
    #[must_use]
    pub fn is_stopped(self) -> bool {
        matches!(self, Self::ProtectiveStop)
    }
}

/// Why a protective stop latched.
///
/// A stop lasts at least `clear_hold_us`, so the cycle that *caused* it is long gone by
/// the time an operator looks. Blaming half a second of stop on an obstacle that was
/// never there — because the robot was actually blind for one cycle — sends someone to
/// look for a pallet instead of a lidar cable.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StopCause {
    /// A return lay inside the protective field.
    Obstacle,
    /// The scan was missing, stale or unusable, so the floor was unknown.
    Blind,
    /// The e-stop was asserted or still latched.
    EStop,
}

/// Latching state machine with a release hold.
#[derive(Clone, Copy, Debug, Default)]
pub struct FieldStateMachine {
    state: FieldState,
    /// What latched the current stop, held for as long as the stop is.
    cause: Option<StopCause>,
    /// When the protective field first became clear during a latched stop.
    clear_since_us: Option<Micros>,
}

impl FieldStateMachine {
    /// A machine in [`FieldState::Clear`].
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// The current state.
    #[must_use]
    pub fn state(&self) -> FieldState {
        self.state
    }

    /// What latched the current stop, or `None` when the machine is not stopped.
    #[must_use]
    pub fn cause(&self) -> Option<StopCause> {
        self.cause
    }

    /// Advances the machine by one observation.
    ///
    /// A protective breach latches immediately, recording `cause` so the reason survives
    /// the whole hold rather than only the cycle that produced it. Release requires the
    /// protective field to have been continuously clear for `clear_hold_us`; a single
    /// breached cycle during the hold restarts the timer from scratch.
    ///
    /// If `now_us` moves backwards — a clock that was stepped, a replay seeking
    /// backwards — the elapsed time saturates to zero and the stop simply holds
    /// longer. Time going backwards must never shorten a safety hold.
    pub fn update(
        &mut self,
        verdict: &ScanVerdict,
        cause: StopCause,
        now_us: Micros,
        clear_hold_us: u64,
    ) -> FieldState {
        if verdict.protective_breach {
            self.state = FieldState::ProtectiveStop;
            self.cause = Some(cause);
            self.clear_since_us = None;
            return self.state;
        }

        let observed = if verdict.warning_breach { FieldState::Warning } else { FieldState::Clear };

        if self.state.is_stopped() {
            let since = *self.clear_since_us.get_or_insert(now_us);
            if now_us.saturating_sub(since) >= clear_hold_us {
                self.state = observed;
                self.cause = None;
                self.clear_since_us = None;
            }
        } else {
            self.state = observed;
            self.cause = None;
        }
        self.state
    }

    /// Forces a latched stop, as when an e-stop is asserted.
    pub fn force_stop(&mut self, cause: StopCause) {
        self.state = FieldState::ProtectiveStop;
        self.cause = Some(cause);
        self.clear_since_us = None;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn clear() -> ScanVerdict {
        ScanVerdict { closest_m: f32::INFINITY, ..ScanVerdict::default() }
    }

    fn warning() -> ScanVerdict {
        ScanVerdict { warning_breach: true, warning_hits: 1, ..clear() }
    }

    fn protective() -> ScanVerdict {
        ScanVerdict {
            protective_breach: true,
            warning_breach: true,
            protective_hits: 1,
            warning_hits: 1,
            ..clear()
        }
    }

    const HOLD: u64 = 500_000;

    #[test]
    fn a_breach_stops_on_the_very_first_cycle() {
        let mut m = FieldStateMachine::new();
        assert_eq!(m.update(&clear(), StopCause::Obstacle, 0, HOLD), FieldState::Clear);
        assert_eq!(
            m.update(&protective(), StopCause::Obstacle, 1_000, HOLD),
            FieldState::ProtectiveStop
        );
    }

    #[test]
    fn the_stop_holds_until_the_field_has_been_clear_long_enough() {
        let mut m = FieldStateMachine::new();
        m.update(&protective(), StopCause::Obstacle, 0, HOLD);
        assert_eq!(
            m.update(&clear(), StopCause::Obstacle, 100_000, HOLD),
            FieldState::ProtectiveStop
        );
        assert_eq!(
            m.update(&clear(), StopCause::Obstacle, 400_000, HOLD),
            FieldState::ProtectiveStop
        );
        assert_eq!(m.update(&clear(), StopCause::Obstacle, 600_000, HOLD), FieldState::Clear);
    }

    #[test]
    fn a_flicker_during_the_hold_restarts_it() {
        let mut m = FieldStateMachine::new();
        m.update(&protective(), StopCause::Obstacle, 0, HOLD);
        m.update(&clear(), StopCause::Obstacle, 400_000, HOLD);
        m.update(&protective(), StopCause::Obstacle, 450_000, HOLD); // one bad cycle
        assert_eq!(
            m.update(&clear(), StopCause::Obstacle, 900_000, HOLD),
            FieldState::ProtectiveStop
        );
        assert_eq!(
            m.update(&clear(), StopCause::Obstacle, 1_000_000, HOLD),
            FieldState::ProtectiveStop
        );
        assert_eq!(
            m.update(&clear(), StopCause::Obstacle, 1_000_000 + HOLD, HOLD),
            FieldState::Clear
        );
    }

    #[test]
    fn releasing_into_a_warning_does_not_skip_to_clear() {
        let mut m = FieldStateMachine::new();
        m.update(&protective(), StopCause::Obstacle, 0, HOLD);
        m.update(&warning(), StopCause::Obstacle, 100_000, HOLD); // hold starts here, at the first clear cycle
        assert_eq!(
            m.update(&warning(), StopCause::Obstacle, 500_000, HOLD),
            FieldState::ProtectiveStop
        );
        assert_eq!(m.update(&warning(), StopCause::Obstacle, 600_001, HOLD), FieldState::Warning);
    }

    #[test]
    fn time_moving_backwards_extends_the_hold_rather_than_ending_it() {
        let mut m = FieldStateMachine::new();
        m.update(&protective(), StopCause::Obstacle, 1_000_000, HOLD);
        m.update(&clear(), StopCause::Obstacle, 1_000_000, HOLD);
        assert_eq!(m.update(&clear(), StopCause::Obstacle, 0, HOLD), FieldState::ProtectiveStop);
    }

    #[test]
    fn warning_and_clear_move_freely_without_a_hold() {
        let mut m = FieldStateMachine::new();
        assert_eq!(m.update(&warning(), StopCause::Obstacle, 0, HOLD), FieldState::Warning);
        assert_eq!(m.update(&clear(), StopCause::Obstacle, 1_000, HOLD), FieldState::Clear);
    }

    #[test]
    fn the_latch_cause_survives_the_whole_hold() {
        let mut m = FieldStateMachine::new();
        m.update(&protective(), StopCause::Blind, 0, HOLD);
        assert_eq!(m.cause(), Some(StopCause::Blind));
        // The floor is visibly clear again, but the stop stands — and so must the reason
        // for it, or half a second of stop gets blamed on an obstacle nobody ever saw.
        m.update(&clear(), StopCause::Obstacle, 100_000, HOLD);
        assert_eq!(m.cause(), Some(StopCause::Blind));
        m.update(&clear(), StopCause::Obstacle, 600_000, HOLD);
        assert_eq!(m.cause(), None);
    }

    #[test]
    fn a_forced_stop_latches_like_a_breach() {
        let mut m = FieldStateMachine::new();
        m.force_stop(StopCause::EStop);
        assert_eq!(
            m.update(&clear(), StopCause::Obstacle, 100_000, HOLD),
            FieldState::ProtectiveStop
        );
        assert_eq!(m.update(&clear(), StopCause::Obstacle, 700_000, HOLD), FieldState::Clear);
    }
}
