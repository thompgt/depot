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

/// Latching state machine with a release hold.
#[derive(Clone, Copy, Debug, Default)]
pub struct FieldStateMachine {
    state: FieldState,
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

    /// Advances the machine by one observation.
    ///
    /// A protective breach latches immediately. Release requires the protective field
    /// to have been continuously clear for `clear_hold_us`; a single breached cycle
    /// during the hold restarts the timer from scratch.
    ///
    /// If `now_us` moves backwards — a clock that was stepped, a replay seeking
    /// backwards — the elapsed time saturates to zero and the stop simply holds
    /// longer. Time going backwards must never shorten a safety hold.
    pub fn update(
        &mut self,
        verdict: &ScanVerdict,
        now_us: Micros,
        clear_hold_us: u64,
    ) -> FieldState {
        if verdict.protective_breach {
            self.state = FieldState::ProtectiveStop;
            self.clear_since_us = None;
            return self.state;
        }

        let observed =
            if verdict.warning_breach { FieldState::Warning } else { FieldState::Clear };

        if self.state.is_stopped() {
            let since = *self.clear_since_us.get_or_insert(now_us);
            if now_us.saturating_sub(since) >= clear_hold_us {
                self.state = observed;
                self.clear_since_us = None;
            }
        } else {
            self.state = observed;
        }
        self.state
    }

    /// Forces a latched stop, as when an e-stop is asserted.
    pub fn force_stop(&mut self) {
        self.state = FieldState::ProtectiveStop;
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
        assert_eq!(m.update(&clear(), 0, HOLD), FieldState::Clear);
        assert_eq!(m.update(&protective(), 1_000, HOLD), FieldState::ProtectiveStop);
    }

    #[test]
    fn the_stop_holds_until_the_field_has_been_clear_long_enough() {
        let mut m = FieldStateMachine::new();
        m.update(&protective(), 0, HOLD);
        assert_eq!(m.update(&clear(), 100_000, HOLD), FieldState::ProtectiveStop);
        assert_eq!(m.update(&clear(), 400_000, HOLD), FieldState::ProtectiveStop);
        assert_eq!(m.update(&clear(), 600_000, HOLD), FieldState::Clear);
    }

    #[test]
    fn a_flicker_during_the_hold_restarts_it() {
        let mut m = FieldStateMachine::new();
        m.update(&protective(), 0, HOLD);
        m.update(&clear(), 400_000, HOLD);
        m.update(&protective(), 450_000, HOLD); // one bad cycle
        assert_eq!(m.update(&clear(), 900_000, HOLD), FieldState::ProtectiveStop);
        assert_eq!(m.update(&clear(), 1_000_000, HOLD), FieldState::ProtectiveStop);
        assert_eq!(m.update(&clear(), 1_000_000 + HOLD, HOLD), FieldState::Clear);
    }

    #[test]
    fn releasing_into_a_warning_does_not_skip_to_clear() {
        let mut m = FieldStateMachine::new();
        m.update(&protective(), 0, HOLD);
        m.update(&warning(), 100_000, HOLD); // hold starts here, at the first clear cycle
        assert_eq!(m.update(&warning(), 500_000, HOLD), FieldState::ProtectiveStop);
        assert_eq!(m.update(&warning(), 600_001, HOLD), FieldState::Warning);
    }

    #[test]
    fn time_moving_backwards_extends_the_hold_rather_than_ending_it() {
        let mut m = FieldStateMachine::new();
        m.update(&protective(), 1_000_000, HOLD);
        m.update(&clear(), 1_000_000, HOLD);
        assert_eq!(m.update(&clear(), 0, HOLD), FieldState::ProtectiveStop);
    }

    #[test]
    fn warning_and_clear_move_freely_without_a_hold() {
        let mut m = FieldStateMachine::new();
        assert_eq!(m.update(&warning(), 0, HOLD), FieldState::Warning);
        assert_eq!(m.update(&clear(), 1_000, HOLD), FieldState::Clear);
    }

    #[test]
    fn a_forced_stop_latches_like_a_breach() {
        let mut m = FieldStateMachine::new();
        m.force_stop();
        assert_eq!(m.update(&clear(), 100_000, HOLD), FieldState::ProtectiveStop);
        assert_eq!(m.update(&clear(), 700_000, HOLD), FieldState::Clear);
    }
}
