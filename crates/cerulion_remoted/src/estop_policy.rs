// SPDX-License-Identifier: AGPL-3.0-only
//! The pure decisions of the robot's e-stop policy. Each function here reads
//! only its arguments; its caller reads the lease and acts on the answer.

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

#[cfg(test)]
mod tests {
    use super::*;

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
