// SPDX-License-Identifier: AGPL-3.0-only
//! The bounded wait an open of a reserved marker service gives a marker that
//! iceoryx2 reports as marked for destruction.
//!
//! The policy markers (`egress_scope`) and network mirror markers
//! (`mirror_origin`) are event services that live only while a holder keeps
//! them open. When the last holder drops one, iceoryx2 marks it for
//! destruction, then removes its shared-memory files; an open in between
//! fails with `IsMarkedForDestruction`. Once the files are gone an open sees
//! the marker absent and an open-or-create builds a fresh one, so the state
//! ends on its own.
//!
//! Each caller of an open-only check names its [`TeardownWait`]. A request
//! handled on its own thread or task waits the state out within
//! [`MarkerTeardown::BUDGET`] and refuses, naming the budget, only when the
//! state outlasts it. A check inside a zenoh callback, a demand authorizer, a
//! listing filter, or a thread or loop that serves other work (a node's
//! executor thread) opens once and refuses at once, naming the teardown, so a
//! marker whose holder died mid-removal never holds up work that is not that
//! request's.
//!
//! The budget counts opens and never reads a clock, so the outcome is a
//! function of what the opens return. The opens run where a marker is opened,
//! never on a publish or receive path: a pause changes when such an open
//! answers, never what a frame carries.

use std::num::NonZeroU32;
use std::time::Duration;

/// Whether an open-only marker check (the remote access policy check and the
/// network mirror check) waits out a marker that iceoryx2 reports as being
/// torn down: its last holder is dropping it and its files are not yet
/// removed. Every other open error refuses the same way under both.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TeardownWait {
    /// Open once and refuse at once, naming the teardown. For a check inside a
    /// zenoh callback, a demand authorizer, a listing filter, or a thread or
    /// loop that serves other work (a node's executor thread), where a wait
    /// would hold up work that is not this request's.
    Never,
    /// Open again, up to 20 opens 25 ms apart, and refuse, naming that budget,
    /// only if the marker is still being torn down after the last. For a request
    /// handled on its own thread or task: an act verb, a remote demand or schema
    /// read, a CLI command.
    Bounded,
}

impl TeardownWait {
    /// The opens this wait allows.
    pub(crate) fn budget(self) -> MarkerTeardown {
        match self {
            Self::Never => MarkerTeardown::ONCE,
            Self::Bounded => MarkerTeardown::BUDGET,
        }
    }
}

/// How many opens, and how far apart, a marker open retries a teardown in
/// progress.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct MarkerTeardown {
    /// Opens tried in all, the first included.
    opens: NonZeroU32,
    /// The pause before every open after the first.
    pause: Duration,
}

/// Why [`MarkerTeardown::open`] returned without a marker.
#[derive(Debug, PartialEq, Eq)]
pub(crate) enum MarkerOpenError<E> {
    /// The first open error that was not a teardown in progress, as returned.
    Open(E),
    /// Every open of the budget found the marker still being torn down.
    TornDown(MarkerTeardown),
}

impl MarkerTeardown {
    /// Twenty opens 25 ms apart, 475 ms of pauses in all: iceoryx2 0.10's
    /// default 500 ms creation window counted in its 25 ms IO ticks, the
    /// pause `open_then_create` waits between its opens.
    pub(crate) const BUDGET: Self = Self::new(
        NonZeroU32::new(20).expect("the marker teardown budget opens at least once"),
        super::OPEN_THEN_CREATE_TICK,
    );

    /// One open and no pause: [`TeardownWait::Never`].
    pub(crate) const ONCE: Self = Self::new(NonZeroU32::MIN, Duration::ZERO);

    pub(crate) const fn new(opens: NonZeroU32, pause: Duration) -> Self {
        Self { opens, pause }
    }

    /// Run `open` until it returns anything but an error `torn_down`
    /// accepts, at most `opens` times, sleeping `pause` before each retry and
    /// never after the last open.
    pub(crate) fn open<T, E>(
        self,
        mut open: impl FnMut() -> Result<T, E>,
        torn_down: impl Fn(&E) -> bool,
    ) -> Result<T, MarkerOpenError<E>> {
        for opened in 1..=self.opens.get() {
            match open() {
                Err(e) if torn_down(&e) => {
                    if opened < self.opens.get() {
                        std::thread::sleep(self.pause);
                    }
                }
                result => return result.map_err(MarkerOpenError::Open),
            }
        }
        Err(MarkerOpenError::TornDown(self))
    }

    /// The refusal reason for `marker` still being torn down at the end of
    /// this budget; a single open says that this check does not wait.
    // hot-path-alloc-ok-fn: refusal text on a failed cold admission, never frame delivery.
    pub(crate) fn outlasted(self, marker: &str) -> String {
        if self.opens == NonZeroU32::MIN {
            return format!(
                "{marker} was being torn down, and this check does not wait for it; retry the \
                 request"
            );
        }
        format!(
            "{marker} was still being torn down after {} opens {} ms apart; retry the request",
            self.opens,
            self.pause.as_millis()
        )
    }
}

/// The refusal reason for `marker` still being torn down when the open-or-create
/// that iceoryx2 runs gave up: that loop waits the state out itself, tick by
/// tick, until the node's creation window (`window`) ends.
// hot-path-alloc-ok-fn: refusal text on a failed cold admission, never frame delivery.
pub(crate) fn outlasted_creation_window(marker: &str, window: Duration) -> String {
    format!(
        "{marker} was still being torn down when the {} ms shared-memory creation window \
         ended; retry the request",
        window.as_millis()
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    enum Scripted {
        TornDown,
        Other,
    }

    /// What one scripted open returns.
    type Step = Result<u8, Scripted>;
    /// A run's outcome and its number of opens.
    type Outcome = (Result<u8, MarkerOpenError<Scripted>>, usize);

    /// The outcome and the number of opens of a budget of `opens` over
    /// `script`, one result per open; past its end every open succeeds.
    fn run(opens: u32, script: &[Step]) -> Outcome {
        let budget = MarkerTeardown::new(NonZeroU32::new(opens).unwrap(), Duration::ZERO);
        let mut calls = 0usize;
        let outcome = budget.open(
            || {
                calls += 1;
                script.get(calls - 1).copied().unwrap_or(Ok(0))
            },
            |e| *e == Scripted::TornDown,
        );
        (outcome, calls)
    }

    #[test]
    fn a_marker_that_opens_at_once_is_opened_once() {
        assert_eq!(run(20, &[Ok(7)]), (Ok(7), 1));
    }

    #[test]
    fn a_teardown_that_ends_inside_the_budget_is_waited_out() {
        let torn = Err(Scripted::TornDown);
        assert_eq!(run(20, &[torn, torn, torn, Ok(9)]), (Ok(9), 4));
        // The last open of the budget still counts: 4 teardowns, then the
        // fifth and final open succeeds.
        assert_eq!(run(5, &[torn, torn, torn, torn, Ok(3)]), (Ok(3), 5));
    }

    #[test]
    fn a_teardown_that_outlasts_the_budget_refuses_after_exactly_its_opens() {
        let torn = Err(Scripted::TornDown);
        let budget = MarkerTeardown::new(NonZeroU32::new(5).unwrap(), Duration::ZERO);
        assert_eq!(
            run(5, &[torn; 6]),
            (Err(MarkerOpenError::TornDown(budget)), 5)
        );
        let one = MarkerTeardown::new(NonZeroU32::new(1).unwrap(), Duration::ZERO);
        assert_eq!(run(1, &[torn]), (Err(MarkerOpenError::TornDown(one)), 1));
    }

    #[test]
    fn any_other_error_returns_at_once_and_after_a_teardown() {
        assert_eq!(
            run(20, &[Err(Scripted::Other)]),
            (Err(MarkerOpenError::Open(Scripted::Other)), 1)
        );
        assert_eq!(
            run(20, &[Err(Scripted::TornDown), Err(Scripted::Other)]),
            (Err(MarkerOpenError::Open(Scripted::Other)), 2)
        );
    }

    #[test]
    fn two_runs_over_the_same_open_results_match_the_oracle() {
        let torn = Err(Scripted::TornDown);
        let budget = MarkerTeardown::new(NonZeroU32::new(20).unwrap(), Duration::ZERO);
        let cases: [(Vec<Step>, Outcome); 3] = [
            (vec![torn, Ok(1)], (Ok(1), 2)),
            (vec![torn; 20], (Err(MarkerOpenError::TornDown(budget)), 20)),
            (
                vec![torn, torn, Err(Scripted::Other)],
                (Err(MarkerOpenError::Open(Scripted::Other)), 3),
            ),
        ];
        for (script, oracle) in cases {
            for round in 0..2 {
                assert_eq!(run(20, &script), oracle, "round {round}: {script:?}");
            }
        }
    }

    #[test]
    fn never_waiting_is_one_open_that_refuses_with_its_own_reason() {
        assert_eq!(
            TeardownWait::Never.budget(),
            MarkerTeardown::new(NonZeroU32::new(1).unwrap(), Duration::ZERO)
        );
        assert_eq!(TeardownWait::Bounded.budget(), MarkerTeardown::BUDGET);
        let torn = Err(Scripted::TornDown);
        let mut calls = 0usize;
        let outcome = TeardownWait::Never.budget().open(
            || {
                calls += 1;
                [torn, Ok(1)][calls - 1]
            },
            |e| *e == Scripted::TornDown,
        );
        assert_eq!(
            (outcome, calls),
            (Err(MarkerOpenError::TornDown(MarkerTeardown::ONCE)), 1)
        );
        assert_eq!(
            MarkerTeardown::ONCE.outlasted("its policy marker"),
            "its policy marker was being torn down, and this check does not wait for it; retry \
             the request"
        );
    }

    #[test]
    fn the_shipped_budget_is_twenty_opens_twenty_five_ms_apart() {
        assert_eq!(
            MarkerTeardown::BUDGET,
            MarkerTeardown::new(NonZeroU32::new(20).unwrap(), Duration::from_millis(25))
        );
        assert_eq!(
            MarkerTeardown::BUDGET.outlasted("its policy marker"),
            "its policy marker was still being torn down after 20 opens 25 ms apart; retry the request"
        );
        assert_eq!(
            outlasted_creation_window("its policy marker", Duration::from_millis(500)),
            "its policy marker was still being torn down when the 500 ms shared-memory creation \
             window ended; retry the request"
        );
    }
}
