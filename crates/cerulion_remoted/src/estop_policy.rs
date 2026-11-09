// SPDX-License-Identifier: AGPL-3.0-only
//! The pure decisions of the robot's e-stop policy. Each function here reads
//! only its arguments and this module's constants; its caller reads the
//! lease or the delivery's counts and acts on the answer.

use std::time::Duration;

use cerud::verbs::restart::RestartStep;

/// What a gated `restart` does at one of its steps.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum RestartCheck {
    /// The step runs.
    Go,
    /// The verb is refused before its stop command: nothing is stopped or
    /// started.
    RefuseStop,
    /// The verb is refused after its stop command and before its start
    /// command: the start command does not run.
    RefuseStart,
}

/// The decision for `step` of a `restart` while the e-stop `holds` (the
/// latch is engaged or an engage is counted).
pub(crate) fn restart_check(step: RestartStep, holds: bool) -> RestartCheck {
    match (step, holds) {
        (_, false) => RestartCheck::Go,
        (RestartStep::Stop, true) => RestartCheck::RefuseStop,
        (RestartStep::Start, true) => RestartCheck::RefuseStart,
    }
}

/// What an ops e-stop delivery does once rcpd has answered one attempt.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum AfterAnswer {
    /// No engage arrived after the answered attempt began: the delivery ends.
    Done,
    /// An engage arrived after the answered attempt began: the delivery sends
    /// the `estop` once more.
    Again,
}

/// The decision once rcpd has answered an attempt that began when the
/// delivery's engage count read `engages_at_start`, now that it reads
/// `engages_now`.
pub(crate) fn after_answer(engages_at_start: u64, engages_now: u64) -> AfterAnswer {
    if engages_now == engages_at_start {
        AfterAnswer::Done
    } else {
        AfterAnswer::Again
    }
}

/// The least time an ops e-stop attempt waits for each response head.
pub(crate) const OPS_ESTOP_HEAD_FLOOR: Duration = Duration::from_secs(1);

/// The head bound of an ops e-stop attempt on a link whose head timeout is
/// `flag`: the larger of `flag` and [`OPS_ESTOP_HEAD_FLOOR`].
pub(crate) fn estop_head_bound(flag: Duration) -> Duration {
    flag.max(OPS_ESTOP_HEAD_FLOOR)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_delivery_ends_once_no_engage_arrived_during_its_answered_attempt() {
        assert_eq!(after_answer(0, 0), AfterAnswer::Done);
        assert_eq!(after_answer(7, 7), AfterAnswer::Done);
    }

    #[test]
    fn a_delivery_sends_once_more_after_an_engage_during_its_answered_attempt() {
        assert_eq!(after_answer(1, 2), AfterAnswer::Again);
        assert_eq!(after_answer(3, 9), AfterAnswer::Again);
    }

    #[test]
    fn the_estop_head_bound_is_at_least_one_second() {
        assert_eq!(
            estop_head_bound(Duration::from_millis(1)),
            Duration::from_secs(1)
        );
        assert_eq!(
            estop_head_bound(Duration::from_secs(1)),
            Duration::from_secs(1)
        );
        assert_eq!(
            estop_head_bound(Duration::from_secs(30)),
            Duration::from_secs(30)
        );
    }

    #[test]
    fn restart_runs_each_step_while_the_estop_does_not_hold() {
        assert_eq!(restart_check(RestartStep::Stop, false), RestartCheck::Go);
        assert_eq!(restart_check(RestartStep::Start, false), RestartCheck::Go);
    }

    #[test]
    fn restart_is_refused_at_the_step_the_estop_holds_at() {
        assert_eq!(
            restart_check(RestartStep::Stop, true),
            RestartCheck::RefuseStop
        );
        assert_eq!(
            restart_check(RestartStep::Start, true),
            RestartCheck::RefuseStart
        );
    }
}
