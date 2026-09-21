// SPDX-License-Identifier: AGPL-3.0-only
//! The FIRST-CONTACT convergence wait — the client-side policy that turns
//! the daemon's explicit "discovery has not converged" marker from a *verdict* into a
//! *reason to keep waiting*.
//!
//! # What the cold-start grace does not cover
//!
//! The daemon has a cold-start grace ([`COLD_START_DISCOVERY_BUDGET`]), and
//! every consumer tells "the LAN was searched and nobody has it"
//! ([`DiscoveryState::Settled`]) from "netd never read anything, so this empty answer
//! proves nothing" ([`DiscoveryState::NotConverged`]). That contract HOLDS — measured
//! live on a Go2, the desk never claims a false absence.
//!
//! What it does not do is make the FIRST command work. The same measurement: a cold
//! desk running `cerulion topic hz /go2/camera/h264` against a healthy, streaming,
//! same-LAN robot gets an explicit UNKNOWN after the daemon's 2.5 s grace, and a retry
//! ~11 s later resolves at 29.7 Hz. Real-LAN convergence lands in ~3–13 s, so the
//! daemon-side grace loses the race and the user is told to do by hand what the tool
//! could do for them: wait a few more seconds and ask again.
//!
//! # Why the wait belongs on the CLIENT, not in the daemon's grace
//!
//! Simply widening [`COLD_START_DISCOVERY_BUDGET`] would pay for it in the wrong
//! currency. That budget is spent INSIDE one control round trip, so it is bounded by
//! the client's `ROUNDTRIP_TIMEOUT` (5 s) — widening it past that turns a slow first
//! contact into an opaque IO timeout, which is strictly worse than its explicit
//! marker. It is also spent by EVERY consumer of that daemon, including
//! `cerulion-vizd`, which holds one `NetdClient` behind a mutex: a longer in-daemon
//! grace stalls the whole Studio sidebar, not just the command that asked.
//!
//! A client-side loop has neither problem. Each round trip stays inside its own
//! timeout, the daemon answers other consumers between polls, and the cost is paid
//! only by the command that is actually waiting for an answer.
//!
//! # The policy, in one sentence
//!
//! Keep re-asking a warm daemon while — and only while — its answer is *empty AND
//! not-converged* AND *this daemon's plane has not already been trying longer than
//! the ceiling*, up to [`FIRST_CONTACT_CONVERGENCE_CEILING`]; anything else answers
//! immediately.
//!
//! That is [`ConvergenceWait::decide`], which is PURE (no clock, no socket, no sleep)
//! and oracle-tested below. The I/O loop that drives it lives in [`crate::client`] and
//! does nothing but run round trips, call the progress sink, and sleep in
//! cancellation-checked slices.
//!
//! # "First contact" is a property of the DAEMON, not of the command
//!
//! The plane-age conjunct is not an optimisation; without it this is not a
//! first-contact wait at all. `ever_settled` latches only on a NON-EMPTY gather, so a
//! netd on a robot-less desk (robot off, other VLAN, still booting) reports
//! `NotConverged` for its ENTIRE lifetime — and a client loop keyed only on its own
//! elapsed would spend the full ceiling on EVERY command, forever. That is precisely
//! the shape the daemon removed one layer down, as a stated correctness fix
//! (`query.rs`'s `grace_spent` latch), and re-creating it here ~4× larger would be a
//! regression wearing a feature's clothes.
//!
//! So the daemon reports how long its plane has been running WITHOUT ever settling
//! (`CatalogGather::unsettled_for`), and this policy refuses to add its own wait on
//! top of a plane that has already out-waited the ceiling.
//!
//! **The scope is the DAEMON, and the window is opened by whichever consumer queries
//! FIRST — not by the command that pays for it.** `note_attempt` fires at the top of
//! every gather on the shared per-computer plane, so `cerulion-vizd` opens it as
//! readily as the CLI does. Two consequences, both intended and neither obvious:
//!
//! * On a desk with Studio running, netd's plane age passes the ceiling within
//!   seconds of vizd starting, so a later `cerulion topic hz` gets ONE round trip and
//!   the explicit UNKNOWN rather than a wait.
//! * On a CLI-only desk the first command opens the window and pays; commands inside
//!   the next ~10 s share it; later ones do not (until netd idle-exits, ~30 s after
//!   its last connection closes, and the next command spawns a fresh plane).
//!
//! That is the SAME scoping the daemon chose for `grace_spent`, for the same reason:
//! the quantity being bounded is how long THIS DAEMON has been failing to reach the
//! network, and a second consumer arriving later does not make the network any less
//! searched. A plane that has spent ten seconds getting nothing is not more likely to
//! succeed because a different process asked.
//!
//! The residual is narrow but real, and it is worth naming because it is the
//! feature's own headline scenario: a robot powered on DURING a command's window, on
//! a desk whose plane is already old, gets one round trip instead of a wait. It is
//! narrow because convergence is not required for a gather to SUCCEED — a robot that
//! is up and announcing answers the very first harvest, which settles the plane and
//! resolves the topic. Only a robot that becomes reachable strictly INSIDE the
//! command's own window is affected.
//!
//! `None` (an older daemon that cannot report an age) is UNKNOWN, never a positive
//! claim — the `connect_endpoints` precedent — so it does not cap the wait.
//!
//! # The residual cost, stated exactly
//!
//! On a robot-less desk the FIRST command against each freshly-spawned daemon still
//! spends the ceiling. netd idle-exits after ~30 s with no connections, so an
//! occasional user re-pays it; a working session does not. That residue is not
//! removable from here: "no robot has ever answered" and "a robot has not answered
//! YET" are the same observation, and its whole point is that the desk must not
//! guess which. It is bounded, visible (a progress line per poll), and escapable
//! (`CERULION_NETWORK=off` skips the remote rung entirely).
//!
//! Against a daemon that predates this gate the plane age is absent, `None` caps nothing,
//! and every command on a robot-less desk pays the full ceiling for as long as that
//! daemon lives — the per-command tax this gate exists to prevent. netd is
//! spawn-once, so the skew is real (a CLI upgraded while a running vizd holds an old
//! daemon alive); the remedy is the same as its stale-daemon warn — restart
//! netd.
//!
//! [`COLD_START_DISCOVERY_BUDGET`]: crate::query::COLD_START_DISCOVERY_BUDGET

use std::time::Duration;

use crate::protocol::DiscoveryState;

/// The CEILING on the client-side first-contact wait — how long a consumer
/// keeps re-asking a daemon that reports [`DiscoveryState::NotConverged`] before it
/// gives up and renders its explicit UNKNOWN.
///
/// # Derivation
///
/// The live measurement behind the number: real-LAN convergence against a
/// healthy robot lands in **~3–13 s** from a cold desk, and the daemon's own 2.5 s
/// grace (plus its worst-case ~3.75 s round trip) sits below the bottom of that
/// range. 10 s covers the bulk of the observed distribution while staying inside the
/// "a few seconds slower, but it just works" bar.
///
/// # It bounds when we STOP RE-ASKING, not the total wall
///
/// The decision is taken after a round trip completes, so the ceiling gates whether
/// another poll may START. One round trip can still be in flight when it expires, and
/// that round trip is bounded only by the client's `ROUNDTRIP_TIMEOUT` — so the
/// reachable worst-case WALL is [`worst_case_wall`], ~15 s with the shipped constants,
/// not 10 s. The sleep is excluded from that sum by construction: [`ConvergenceWait::decide`]
/// refuses to start a poll whose sleep would itself cross the ceiling. Guarded by
/// `the_shipped_constants_bound_the_real_wall`, which asserts the WALL rather than the
/// constant (a bound on the constant alone would miss that the reachable wall
/// is one round trip longer).
///
/// # It must clear the daemon's own grace by a wide margin
///
/// The FIRST round trip of a cold daemon already costs up to ~3.75 s
/// ([`COLD_START_DISCOVERY_BUDGET`] plus one paced retry and a final full attempt).
/// A ceiling that merely exceeded the budget would buy a single extra poll; this one
/// leaves room for several, which is the difference between "waited a bit longer" and
/// "waited through convergence". Guarded by
/// `the_shipped_ceiling_buys_several_polls_past_the_daemons_own_grace`.
///
/// # Cost on a robot-less desk
///
/// The first command against each freshly-spawned daemon spends this whole ceiling
/// before answering; later commands against the same daemon do not (the plane-age
/// cap — see the module docs). `CERULION_NETWORK=off` skips the remote rung entirely.
///
/// [`COLD_START_DISCOVERY_BUDGET`]: crate::query::COLD_START_DISCOVERY_BUDGET
pub const FIRST_CONTACT_CONVERGENCE_CEILING: Duration = Duration::from_secs(10);

/// How long the client sleeps between polls of a not-yet-converged daemon.
///
/// A poll is NOT free even against a warm daemon: a `grace_spent` query plane still
/// runs one FRESH announce harvest per query (that is what lets a robot appearing
/// later settle the plane), so each round trip costs roughly one harvest window plus
/// one gather window — ~750 ms of real work on the daemon's side. Sleeping about that
/// long between polls keeps the duty cycle near 50 %: responsive enough that
/// convergence is noticed within about `poll_interval + one round trip` (~1.5 s with
/// the shipped constants — NOT "within a second"), while leaving the shared daemon
/// time to serve `cerulion-vizd` and any other consumer between our asks.
///
/// It is also the cadence of the progress line, so it doubles as the refresh rate the
/// user sees.
pub const CONVERGENCE_POLL_INTERVAL: Duration = Duration::from_millis(750);

/// The longest a caller can observe [`ConvergenceWait`] blocking, given a
/// per-round-trip timeout.
///
/// `ceiling` gates when a NEW poll may start and the sleep is excluded by the
/// predictive gate, so the overshoot is exactly one in-flight round trip. Stated as a
/// function so the docs, the guard test and any caller-side deadline all read the
/// same arithmetic instead of three hand-copied numbers.
pub const fn worst_case_wall(ceiling: Duration, round_trip_timeout: Duration) -> Duration {
    // `Duration::saturating_add` IS a `const fn` (verified against the shipped
    // toolchain), and it is exact for every input. Hand-rolling
    // this over `as_nanos() as u64` would TRUNCATE at the cast for any
    // `Duration` above ~584 years, on a `pub` function taking arbitrary inputs.
    ceiling.saturating_add(round_trip_timeout)
}

/// What a consumer should do after ONE query round trip. PURE — the whole
/// first-contact policy, oracle-tested, with no clock and no network.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WaitDecision {
    /// Answer NOW with what this round trip returned. Either the daemon vouched for
    /// its discovery ([`DiscoveryState::Settled`] — so an empty answer is a genuine
    /// absence and a non-empty one is the real thing), or the answer is non-empty
    /// (waiting cannot improve on data we already hold).
    Proceed,
    /// The answer was empty AND the daemon cannot vouch for it, and there is budget
    /// left: sleep `next_poll_delay`, then ask again.
    KeepWaiting {
        /// How long to sleep before the next round trip.
        next_poll_delay: Duration,
    },
    /// The wait is over and the daemon still cannot vouch for its discovery. Answer
    /// with the last (empty, not-converged) result, which the consumer renders as
    /// the daemon's explicit UNKNOWN, never as absence.
    GiveUpHonestUnknown,
}

/// How a first-contact wait ENDED. Carried out on [`Converged`] so a consumer
/// never has to re-derive it from proxies.
///
/// It exists because the first attempt did exactly that — it gated the user-facing
/// "gave up" line on `(discovery == NotConverged && waited != 0)`, which is true of a
/// wait that SUCCEEDED against an older daemon (the trust gate downgrades every
/// report), so a resolved topic printed "nothing on the network answered" immediately
/// before streaming its frames.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WaitOutcome {
    /// The loop exited via [`WaitDecision::Proceed`] — this answer is the one to use.
    Answered,
    /// The loop exhausted its budget and is returning the last empty, not-converged
    /// answer for the consumer to render as UNKNOWN.
    GaveUp,
    /// The caller's cancellation flag was cleared (Ctrl-C / SIGTERM) — the answer is
    /// whatever the last round trip returned, and NO give-up claim may be made from it.
    Cancelled,
}

/// The first-contact wait POLICY — a ceiling and a poll cadence, plus the
/// pure [`decide`](Self::decide) that turns one round trip's outcome into a
/// [`WaitDecision`].
///
/// [`Default`] is the SHIPPED policy ([`FIRST_CONTACT_CONVERGENCE_CEILING`] /
/// [`CONVERGENCE_POLL_INTERVAL`]); [`new`](Self::new) exists so tests can drive the
/// real loop at shrunk values instead of sleeping against production constants;
/// [`off`](Self::off) is the no-wait policy for call sites that make no absence claim.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ConvergenceWait {
    ceiling: Duration,
    poll_interval: Duration,
}

impl Default for ConvergenceWait {
    fn default() -> Self {
        Self::new(FIRST_CONTACT_CONVERGENCE_CEILING, CONVERGENCE_POLL_INTERVAL)
    }
}

impl ConvergenceWait {
    /// A policy with an explicit ceiling and poll cadence.
    pub const fn new(ceiling: Duration, poll_interval: Duration) -> Self {
        Self {
            ceiling,
            poll_interval,
        }
    }

    /// The NO-WAIT policy — answer on the first round trip, with
    /// no wait at all.
    ///
    /// This is not a debugging knob, it is a real production posture: a caller that
    /// renders NO absence claim (the `topic echo` / `topic info` local-walker
    /// fallback, which degrades to hex on any failure) must not spend a user's ten
    /// seconds on an answer it will discard. Every such call site names THIS
    /// constructor rather than minting its own zero policy, so the structural guard
    /// can pin that `ConvergenceWait::new` never appears in the CLI.
    pub const fn off() -> Self {
        Self::new(Duration::ZERO, CONVERGENCE_POLL_INTERVAL)
    }

    /// Would this policy ever wait? False for [`off`](Self::off).
    ///
    /// The loop uses it to decide whether to announce the wait BEFORE the first round
    /// trip: a no-wait caller must print nothing at all.
    pub const fn enabled(&self) -> bool {
        // `Duration::is_zero` is const, but comparing nanos keeps this readable
        // alongside `decide`'s own arithmetic.
        self.ceiling.as_nanos() > 0
    }

    /// The budget for STARTING new polls — see [`FIRST_CONTACT_CONVERGENCE_CEILING`].
    pub const fn ceiling(&self) -> Duration {
        self.ceiling
    }

    /// The sleep between polls — see [`CONVERGENCE_POLL_INTERVAL`].
    pub const fn poll_interval(&self) -> Duration {
        self.poll_interval
    }

    /// THE first-contact decision. PURE — oracle-tested.
    ///
    /// `answer_empty` is the consumer's own emptiness test on what this round trip
    /// returned. `plane_unsettled_for` is how long the ANSWERING DAEMON's query plane
    /// has been running without ever settling (`None` = it cannot report, which is
    /// UNKNOWN and never a positive claim).
    ///
    /// Three conjuncts, each load-bearing for a different reason:
    ///
    /// * `discovery == Settled || !answer_empty` ⇒ [`WaitDecision::Proceed`]. The
    ///   first half is the genuine-absence arm that must never loop. The second is the
    ///   TRUST-GATE arm: a daemon older than `DISCOVERY_MIN_DAEMON_VERSION` has
    ///   EVERY report downgraded to `NotConverged`, so without it a permanently-stale
    ///   daemon serving a perfectly good non-empty answer would make every command
    ///   wait the full ceiling and then use the answer it already had at 0 ms.
    /// * `elapsed + poll_interval < ceiling` — the PREDICTIVE gate. Checking
    ///   `elapsed < ceiling` alone would start a sleep that lands past the ceiling, so
    ///   the sleep would count against the user's wall while contributing nothing. The
    ///   remaining overshoot is one in-flight round trip; see [`worst_case_wall`].
    /// * `plane_unsettled_for < ceiling` — the FIRST-CONTACT gate. See the module
    ///   docs: without it a robot-less desk pays the ceiling on every command forever.
    ///
    /// A stale daemon serving an EMPTY answer still waits the ceiling. That is
    /// deliberate and unavoidable: it is wire-indistinguishable from a genuine cold
    /// start, the client already warns once per process with the restart remedy, and
    /// the failure mode is bounded lateness rather than a wrong claim.
    pub fn decide(
        &self,
        discovery: DiscoveryState,
        answer_empty: bool,
        elapsed: Duration,
        plane_unsettled_for: Option<Duration>,
    ) -> WaitDecision {
        if discovery == DiscoveryState::Settled || !answer_empty {
            return WaitDecision::Proceed;
        }
        let budget_left = elapsed.saturating_add(self.poll_interval) < self.ceiling;
        let plane_is_young = plane_unsettled_for.is_none_or(|age| age < self.ceiling);
        if budget_left && plane_is_young {
            WaitDecision::KeepWaiting {
                next_poll_delay: self.poll_interval,
            }
        } else {
            WaitDecision::GiveUpHonestUnknown
        }
    }
}

/// A query answer plus how the first-contact wait that produced it ended.
///
/// `outcome` is the terminal [`WaitDecision`] the loop actually took, and
/// `progress_lines` counts the stderr lines the caller's sink emitted. Both exist so a
/// consumer reports what HAPPENED rather than re-deriving it from a duration and a
/// marker — the mistake that made the "gave up" line fire on successful waits.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Converged<T> {
    /// The last answer the daemon gave.
    pub answer: T,
    /// Total wall time the wait ran before `answer` was produced. Measured after the
    /// round trip, so on the immediate-answer path it is the round trip's own latency
    /// (tens of microseconds over a UDS), NOT zero.
    pub waited: Duration,
    /// How the wait ended.
    pub outcome: WaitOutcome,
    /// How many progress lines the caller's sink was asked to emit. Zero means the
    /// user saw nothing, so there is nothing for a closing line to close.
    pub progress_lines: u32,
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::query::{COLD_START_DISCOVERY_BUDGET, QUERY_GATHER_WINDOW, QUERY_HARVEST_WINDOW};

    const CEILING: Duration = Duration::from_millis(1000);
    const POLL: Duration = Duration::from_millis(100);

    fn policy() -> ConvergenceWait {
        ConvergenceWait::new(CEILING, POLL)
    }

    /// A young plane — the common case, and the one that must not suppress the wait.
    const YOUNG: Option<Duration> = Some(Duration::from_millis(500));

    /// The whole decision table against a HAND-WRITTEN oracle — every branch, and
    /// every input combination that can reach it.
    #[test]
    fn decide_covers_every_branch_against_a_hand_oracle() {
        use DiscoveryState::{NotConverged, Settled};
        use WaitDecision::{GiveUpHonestUnknown, KeepWaiting, Proceed};

        let keep = KeepWaiting {
            next_poll_delay: POLL,
        };
        // (discovery, answer_empty, elapsed_ms, plane_unsettled_ms, expected)
        let oracle = [
            // A daemon that vouches for its discovery is believed IMMEDIATELY,
            // whether or not it found anything — the genuine-absence arm that must
            // never loop, and it outranks even an ancient plane.
            (Settled, true, 0, None, Proceed),
            (Settled, true, 500, Some(99_999), Proceed),
            (Settled, false, 0, None, Proceed),
            // A non-empty answer is used immediately even from a daemon that cannot
            // vouch for it (the stale-daemon trust-gate arm).
            (NotConverged, false, 0, None, Proceed),
            (NotConverged, false, 5000, Some(99_999), Proceed),
            // Empty AND not vouched-for, with budget left and a young plane: wait.
            (NotConverged, true, 0, None, keep),
            (NotConverged, true, 0, Some(0), keep),
            (NotConverged, true, 899, Some(999), keep),
            // ... the PREDICTIVE ceiling ends it one poll interval early.
            (NotConverged, true, 900, Some(0), GiveUpHonestUnknown),
            (NotConverged, true, 5000, None, GiveUpHonestUnknown),
            // ... and so does a plane that has already out-waited the ceiling,
            // however fresh THIS command is (the first-contact gate).
            (NotConverged, true, 0, Some(1000), GiveUpHonestUnknown),
            (NotConverged, true, 0, Some(3_600_000), GiveUpHonestUnknown),
        ];
        for (discovery, answer_empty, elapsed_ms, plane_ms, expected) in oracle {
            let plane = plane_ms.map(Duration::from_millis);
            assert_eq!(
                policy().decide(
                    discovery,
                    answer_empty,
                    Duration::from_millis(elapsed_ms),
                    plane
                ),
                expected,
                "discovery={discovery:?} answer_empty={answer_empty} \
                 elapsed_ms={elapsed_ms} plane_ms={plane_ms:?}"
            );
        }
    }

    /// The PREDICTIVE ceiling is a threshold pinned on BOTH sides at nanosecond
    /// resolution — a `<`/`<=` slip, or a reversion to the naive `elapsed < ceiling`,
    /// is invisible to a millisecond table.
    #[test]
    fn the_ceiling_is_predictive_and_pinned_on_both_sides() {
        let p = policy();
        let just_under = CEILING - POLL - Duration::from_nanos(1);
        assert_eq!(
            p.decide(DiscoveryState::NotConverged, true, just_under, YOUNG),
            WaitDecision::KeepWaiting {
                next_poll_delay: POLL
            },
            "one nanosecond under (ceiling - poll) still has room for a whole poll"
        );
        assert_eq!(
            p.decide(DiscoveryState::NotConverged, true, CEILING - POLL, YOUNG),
            WaitDecision::GiveUpHonestUnknown,
            "at exactly (ceiling - poll) the sleep would land ON the ceiling — refuse \
             to start a poll whose own sleep crosses it"
        );
        // The naive gate would still be waiting here; the predictive one is not.
        assert_eq!(
            p.decide(
                DiscoveryState::NotConverged,
                true,
                CEILING - Duration::from_nanos(1),
                YOUNG
            ),
            WaitDecision::GiveUpHonestUnknown,
            "an `elapsed < ceiling` gate would sleep past the ceiling here"
        );
    }

    /// The plane-age gate is a threshold pinned on BOTH sides, and `None` (an older
    /// daemon) is UNKNOWN — never a positive claim that would suppress the wait.
    #[test]
    fn the_plane_age_gate_is_pinned_on_both_sides_and_none_never_caps() {
        let p = policy();
        for (age, expected) in [
            (
                Some(CEILING - Duration::from_nanos(1)),
                WaitDecision::KeepWaiting {
                    next_poll_delay: POLL,
                },
            ),
            (Some(CEILING), WaitDecision::GiveUpHonestUnknown),
            (
                None,
                WaitDecision::KeepWaiting {
                    next_poll_delay: POLL,
                },
            ),
        ] {
            assert_eq!(
                p.decide(DiscoveryState::NotConverged, true, Duration::ZERO, age),
                expected,
                "plane_unsettled_for={age:?}"
            );
        }
    }

    /// [`ConvergenceWait::off`] answers on the first round trip — exactly one
    /// query, no wait — and reports itself as disabled so the loop stays silent.
    #[test]
    fn the_off_policy_never_waits_and_never_announces() {
        let p = ConvergenceWait::off();
        assert!(!p.enabled(), "an off policy must not announce a wait");
        assert!(ConvergenceWait::default().enabled());
        assert_eq!(
            p.decide(
                DiscoveryState::NotConverged,
                true,
                Duration::ZERO,
                Some(Duration::ZERO)
            ),
            WaitDecision::GiveUpHonestUnknown
        );
        // ... and it still does not break the two immediate-answer arms.
        assert_eq!(
            p.decide(DiscoveryState::Settled, true, Duration::ZERO, None),
            WaitDecision::Proceed
        );
        assert_eq!(
            p.decide(DiscoveryState::NotConverged, false, Duration::ZERO, None),
            WaitDecision::Proceed
        );
    }

    /// The SHIPPED policy has to buy several polls PAST the daemon's own worst-case
    /// first answer, or the client-side wait is theatre: it would run one extra round
    /// trip and give the same verdict the daemon already gave.
    #[test]
    fn the_shipped_ceiling_buys_several_polls_past_the_daemons_own_grace() {
        let shipped = ConvergenceWait::default();
        assert_eq!(shipped.ceiling(), FIRST_CONTACT_CONVERGENCE_CEILING);
        assert_eq!(shipped.poll_interval(), CONVERGENCE_POLL_INTERVAL);

        // The daemon's worst-case FIRST answer, derived from `query.rs`'s own
        // constants rather than a hand-copied literal: the whole cold-start budget,
        // plus one paced retry (`QUERY_HARVEST_WINDOW`), plus one final full attempt
        // (a harvest window + a per-robot GET window). = 3750 ms as shipped.
        let daemon_worst_case_first_answer =
            COLD_START_DISCOVERY_BUDGET + QUERY_HARVEST_WINDOW * 2 + QUERY_GATHER_WINDOW;
        assert_eq!(
            daemon_worst_case_first_answer,
            Duration::from_millis(3750),
            "the derived worst case must still match the arithmetic query.rs documents"
        );
        assert!(
            FIRST_CONTACT_CONVERGENCE_CEILING > daemon_worst_case_first_answer,
            "the ceiling must outlast the daemon's own first round trip \
             ({daemon_worst_case_first_answer:?}), or nothing is ever re-asked"
        );
        let room = FIRST_CONTACT_CONVERGENCE_CEILING - daemon_worst_case_first_answer;
        assert!(
            room >= CONVERGENCE_POLL_INTERVAL * 2 * 3,
            "the ceiling should leave room for at least three further polls; \
             room={room:?} interval={CONVERGENCE_POLL_INTERVAL:?}"
        );
        assert!(
            CONVERGENCE_POLL_INTERVAL < FIRST_CONTACT_CONVERGENCE_CEILING,
            "a poll interval at or above the ceiling would allow zero polls"
        );
    }

    /// The interactive-wait bound is on the reachable WALL, not on the constant.
    ///
    /// Asserting `FIRST_CONTACT_CONVERGENCE_CEILING
    /// <= 15 s` under a "must not look wedged" rationale bounds the wrong number — the quantity that
    /// rationale is about is what the USER experiences, which is the ceiling plus one
    /// in-flight round trip. That wall is 15 s while the constant reads 10 s.
    #[test]
    fn the_shipped_constants_bound_the_real_wall() {
        let wall = worst_case_wall(
            FIRST_CONTACT_CONVERGENCE_CEILING,
            crate::client::ROUNDTRIP_TIMEOUT,
        );
        assert_eq!(
            wall,
            Duration::from_secs(15),
            "ceiling (10 s) + one round-trip timeout (5 s); the sleep is excluded by \
             the predictive gate"
        );
        assert!(
            wall <= Duration::from_secs(16),
            "past ~15 s an explicit UNKNOWN plus a visible retry beats a command that \
             reads as wedged; wall={wall:?}. SCOPE: this bounds the WAIT only — a \
             truly cold desk additionally pays netd's spawn-readiness ceiling before \
             the loop starts, and that half prints no counter."
        );
        // The overshoot is ONE round trip, never a sleep on top of it.
        assert_eq!(
            wall - FIRST_CONTACT_CONVERGENCE_CEILING,
            crate::client::ROUNDTRIP_TIMEOUT
        );
    }
}
