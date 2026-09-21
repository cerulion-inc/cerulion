// SPDX-License-Identifier: AGPL-3.0-only
//! The INGRESS-BUILD hold — the recorder's second, independent reason
//! to keep its channel set open.
//!
//! # The defect this closes
//!
//! A freshly-booted robot's FIRST `graph run attach --single-process --record`
//! captured **4 topics** (the declared set) while all ~98 of the bridge's raw
//! routes were reported `appeared_after_bag_creation`. The identical command two
//! minutes later, against a warm DDS bus, captured 102 topics COMPLETE. Two of
//! three fresh-participant runs lost that race. Nothing was misconfigured — the
//! user did nothing wrong, and got a useless bag.
//!
//! Two individually-correct components composed badly:
//!
//! * a `ros2 attach` bridge opens its raw routes SEQUENTIALLY, and on a COLD DDS
//!   bus each open waits on SPDP/SEDP matching plus the schema ladder —
//!   MEASURED at ~55-60 s end to end on the Go2, in BURSTS separated by
//!   multi-second gaps;
//! * the settle releases after
//!   [`DISCOVERY_SETTLE_QUIET_SCANS`](crate::DISCOVERY_SETTLE_QUIET_SCANS) (2)
//!   quiet 250 ms rescans past a 500 ms floor, so ANY gap over half a second
//!   closes it — and channels are frozen at bag creation, so a route that opens
//!   afterwards can only ever be REPORTED.
//!
//! **Raising the settle cap cannot fix this.** The cap is a CEILING; the quiet
//! rule releases far below it, and it is the quiet rule that fires in a gap.
//!
//! # What this term keys on: EVIDENCE OF RECENT CREATION, not a timer
//!
//! `cerulion_core`'s
//! [`ingress_build`](cerulion_core::transport::ingress_build) channel carries the
//! two facts only the producing process holds — *"I have created N runtime
//! ingress routes so far"* and *"my last one was M milliseconds ago"* — and this
//! module turns a stream of those records into a hold decision:
//!
//! * hold WHILE somebody on the machine created a route within the last
//!   [`INGRESS_BUILD_STALL_WINDOW`]; every fresh creation claim RE-ARMS the
//!   window, so a burst-and-gap build like the Go2's is carried across its gaps
//!   as long as no single gap exceeds it. Because the claim is the WRITER's own
//!   ("I created a route N ms ago"), the very FIRST record heard from a writer
//!   already arms the hold — which is what makes a cold boot, where route 1's
//!   record is all the recorder has, work at all;
//! * release when nothing has advanced for that window (the ordinary exit);
//! * release at [`INGRESS_BUILD_HOLD_CEILING`] no matter what — and because the
//!   stall arm is checked FIRST, reaching the ceiling PROVES routes were still
//!   being created, which is exactly the state worth being LOUD about.
//!
//! The signal carries no denominator (see the `ingress_build` module docs for
//! why the producing side must live in `cerulion_core` rather than in the
//! hand-vendored bridge), so "still building" cannot be distinguished from
//! "finished" — hence a stall window rather than a wait-for-done. What it DOES
//! carry is recency, which is what makes the window arm on evidence rather than
//! on a guess.
//!
//! # What it does NOT change
//!
//! When no progress record is ever observed — an old graph, a plain
//! (non-attach) graph, `cerulion bag record --topic`, a machine whose only
//! producers are ordinary graph publishers — [`decide_ingress_build_hold`]
//! returns [`IngressBuildHold::Inactive`] and the recorder's timing is
//! byte-identical to its earlier behaviour. The term is ADDITIVE: it can
//! only ever hold the channel set open LONGER, never close it earlier.
//!
//! # Why the hold does not simply add to bag-creation latency
//!
//! Creation waits for the LATER of (settle release, schema learning) — and now
//! the ingress-build release too. On an attach graph the schema-wait grace is
//! already [`DEFAULT_SCHEMA_WAIT_MS`](crate::DEFAULT_SCHEMA_WAIT_MS) (5 s) and
//! commonly binds (a declared attach tap that has not spoken rides it to the
//! deadline), so on a WARM bus — where the whole build finishes in a second or
//! two — this term releases inside an envelope the recorder was already paying.
//! It is on a COLD bus, where it is the only thing standing between the operator
//! and a 4-topic bag, that it costs real time.

use std::time::Duration;

/// How long the recorder keeps its channel set open after the LAST
/// observed ingress-build advance. Every advance re-arms this window.
///
/// It must comfortably exceed the gaps WITHIN one bridge's route-creation burst
/// (measured in single-digit seconds on a cold Go2 bus) without being so long
/// that a settled machine pays it for nothing — which it does not, because a
/// settled machine never advances at all (the BASELINE rule in
/// [`IngressBuildProgress`](cerulion_core::transport::ingress_build::IngressBuildProgress)
/// means a first sighting of an already-built plane is not motion).
///
/// Five seconds is also, deliberately, the
/// [`DEFAULT_SCHEMA_WAIT_MS`](crate::DEFAULT_SCHEMA_WAIT_MS) grace: on the
/// attach path that grace commonly binds anyway, so this term costs nothing
/// extra on a warm bus.
pub const INGRESS_BUILD_STALL_WINDOW: Duration = Duration::from_secs(5);

/// The ABSOLUTE ceiling on the ingress-build hold, measured from the
/// drive loop's start.
///
/// A producer that keeps opening routes forever must not be able to hold a
/// recording's channel set open forever — a bag that never gets created is worse
/// than one that is missing late topics. 120 s is ~2× the MEASURED cold-attach
/// build on the Go2 (~55-60 s to the last of 98 routes), so the ceiling is
/// generous enough that reaching it means something is genuinely wrong rather
/// than merely slow. Reaching it is therefore reported LOUDLY, with the routes
/// still-arriving fact named.
pub const INGRESS_BUILD_HOLD_CEILING: Duration = Duration::from_secs(120);

// The stall window must outlast the quiet rule it exists to survive: if it were
// shorter than the settle floor, the ingress term could never extend the
// hold past what the settle already gave and would be dead weight.
const _: () = assert!(
    INGRESS_BUILD_STALL_WINDOW.as_millis() > crate::DISCOVERY_SETTLE_MIN.as_millis(),
    "the ingress-build stall window must outlast the discovery settle floor"
);
// And the ceiling must leave room for at least one full stall window, or the
// ordinary (stalled) release would be unreachable and every hold would end on
// the LOUD ceiling arm.
const _: () = assert!(
    INGRESS_BUILD_HOLD_CEILING.as_millis() > INGRESS_BUILD_STALL_WINDOW.as_millis(),
    "the ingress-build ceiling must leave room for the ordinary stalled release"
);

/// Everything [`decide_ingress_build_hold`] looks at. Pure inputs —
/// no clock is read inside the decision, so it is oracle-testable and its
/// verdict is a function of what the recorder observed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct IngressHoldInput {
    /// Whether live-topic discovery is enabled. The hold is USELESS without it:
    /// keeping the channel set open buys nothing if the recorder is never going
    /// to tap the topics that appear. This is also why the hold needs no new
    /// user-facing switch — `CERULION_RECORD_DISCOVERY=off` turns it off.
    pub discovery_on: bool,
    /// The operator's settle CAP. `Duration::ZERO` means "never hold before
    /// creation" and is honoured here too: zero means zero. (Any NON-zero cap is
    /// deliberately NOT a bound on this term — the cap governs the quiet
    /// rule, and bounding an evidence-backed hold by the default 2 s cap would
    /// reintroduce exactly the defect this closes.)
    pub settle_cap: Duration,
    /// Time since the drive loop started — the same origin
    /// `Recorder::discovery_hold_active` measures against.
    pub elapsed: Duration,
    /// How long ago a process on this machine last CREATED an ingress route, as
    /// last evidenced on the progress channel and aged into the recorder's own
    /// frame — or `None` if no FRESH evidence was ever received.
    ///
    /// `None` is the NO-SIGNAL case and yields [`IngressBuildHold::Inactive`].
    /// It covers both "nothing is building" and "the only evidence heard was
    /// already stale on arrival" — an already-built plane's republished total —
    /// which is why a recorder armed against a settled robot makes no claim at
    /// all rather than reporting an instantly-stalled hold.
    ///
    /// This is a RECENCY claim the writer made about itself, not a count
    /// difference the recorder inferred. That distinction is the cold-boot
    /// fix: on a `graph run --record` cold boot the first record the
    /// recorder ever hears is route 1's, and a count difference does not exist
    /// yet — but "I created a route 0 ms ago" does.
    pub since_last_creation: Option<Duration>,
    /// [`INGRESS_BUILD_STALL_WINDOW`], threaded so tests can drive the decision
    /// at their own scale instead of waiting out the shipped one.
    pub stall_window: Duration,
    /// [`INGRESS_BUILD_HOLD_CEILING`], threaded for the same reason.
    pub ceiling: Duration,
}

/// The verdict — whether the ingress-build term is holding the channel
/// set open, and if not, why not.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IngressBuildHold {
    /// The term never engaged: discovery is off, the operator capped the hold at
    /// zero, or no ingress-build progress was ever observed. The recorder's
    /// timing is byte-identical to earlier — this is the arm every graph
    /// that is not a progressively-built ingress plane takes.
    Inactive,
    /// Routes are still arriving: keep the channel set open.
    Hold,
    /// Released because nothing advanced for a whole stall window — the ordinary
    /// exit, meaning the plane looks finished.
    ReleasedStalled,
    /// Released at the absolute ceiling. Because [`Self::ReleasedStalled`] is
    /// decided FIRST, this arm proves routes were STILL being created when the
    /// channel set closed — so topics are about to be lost, and the recorder
    /// says so loudly.
    ReleasedAtCeiling,
}

impl IngressBuildHold {
    /// Whether this verdict holds the channel set open.
    #[must_use]
    pub fn is_holding(self) -> bool {
        matches!(self, Self::Hold)
    }

    /// Whether the term ever engaged at all (i.e. progress was observed and the
    /// hold was permitted). `false` for [`Self::Inactive`] only.
    #[must_use]
    pub fn engaged(self) -> bool {
        !matches!(self, Self::Inactive)
    }
}

/// Decide whether the ingress-build term holds the channel set open.
/// PURE — the whole policy, with no clock and no transport.
///
/// Order matters and is load-bearing: the STALL arm is tested before the
/// CEILING arm, so a ceiling verdict can only be reached while progress was
/// still advancing. That is what lets the recorder's ceiling warn state the
/// strong claim ("routes were still arriving when we had to close") instead of
/// the weak one ("time ran out").
#[must_use]
pub fn decide_ingress_build_hold(input: IngressHoldInput) -> IngressBuildHold {
    if !input.discovery_on || input.settle_cap.is_zero() {
        return IngressBuildHold::Inactive;
    }
    let Some(since) = input.since_last_creation else {
        // NO SIGNAL. Not "released" — the term never had anything to say, and
        // conflating the two would let a bag's manifest claim a hold that never
        // happened.
        return IngressBuildHold::Inactive;
    };
    if since >= input.stall_window {
        return IngressBuildHold::ReleasedStalled;
    }
    if input.elapsed >= input.ceiling {
        return IngressBuildHold::ReleasedAtCeiling;
    }
    IngressBuildHold::Hold
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A default input that HOLDS, so each test perturbs exactly one axis.
    fn holding() -> IngressHoldInput {
        IngressHoldInput {
            discovery_on: true,
            settle_cap: Duration::from_millis(2000),
            elapsed: Duration::from_secs(10),
            since_last_creation: Some(Duration::from_millis(100)),
            stall_window: Duration::from_secs(5),
            ceiling: Duration::from_secs(120),
        }
    }

    #[test]
    fn the_control_holds_so_every_other_arm_is_a_real_perturbation() {
        // Anti-tautology: without this, a decision that returned Inactive
        // unconditionally would pass every "does not hold" assertion below.
        assert_eq!(decide_ingress_build_hold(holding()), IngressBuildHold::Hold);
        assert!(decide_ingress_build_hold(holding()).is_holding());
    }

    #[test]
    fn no_observed_progress_is_inactive_not_released() {
        // THE byte-identical-behaviour arm: an old graph, a plain graph, or
        // `cerulion bag record --topic` observes nothing, and the recorder must
        // behave exactly as it did before the hold existed. Inactive rather than
        // Released, because a manifest must not claim a hold that never was.
        let verdict = decide_ingress_build_hold(IngressHoldInput {
            since_last_creation: None,
            ..holding()
        });
        assert_eq!(verdict, IngressBuildHold::Inactive);
        assert!(!verdict.is_holding());
        assert!(!verdict.engaged());
    }

    #[test]
    fn the_very_first_creation_claim_holds_with_no_history_to_compare_against() {
        // THE cold-boot arm, at the policy level. On a `graph run --record` cold
        // boot the recorder is armed before the graph reaches step 0, so the
        // FIRST evidence it ever has is route 1's "created 0 ms ago" — there is
        // no earlier count to have advanced past. A policy that needed a
        // DIFFERENCE could not hold here, and the channel set would close in the
        // multi-second gap before route 2.
        assert_eq!(
            decide_ingress_build_hold(IngressHoldInput {
                elapsed: Duration::from_millis(300),
                since_last_creation: Some(Duration::ZERO),
                ..holding()
            }),
            IngressBuildHold::Hold
        );
    }

    #[test]
    fn evidence_that_was_already_stale_is_no_signal_rather_than_a_release() {
        // A settled robot republishes a total whose recency is minutes old. The
        // recorder never records such evidence at all (it ingests only FRESH
        // claims), so this arm pins the policy's half of that contract: `None`
        // means "no claim", which keeps an already-built plane's manifest empty
        // instead of carrying an instantly-stalled hold it never actually paid.
        let verdict = decide_ingress_build_hold(IngressHoldInput {
            since_last_creation: None,
            ..holding()
        });
        assert_eq!(verdict, IngressBuildHold::Inactive);
        assert!(!verdict.engaged());
    }

    #[test]
    fn discovery_off_is_inactive_even_with_progress_streaming_in() {
        // Holding buys nothing when the recorder will never tap what appears —
        // which is also why the hold needs no new user-facing switch.
        assert_eq!(
            decide_ingress_build_hold(IngressHoldInput {
                discovery_on: false,
                ..holding()
            }),
            IngressBuildHold::Inactive
        );
    }

    #[test]
    fn a_zero_settle_cap_is_honoured_as_never_hold() {
        // `CERULION_RECORD_DISCOVERY_SETTLE_MS=0` documents "no pre-creation
        // hold". Zero means zero — this is the escape hatch that keeps discovery
        // REPORTING while refusing to wait for anything.
        assert_eq!(
            decide_ingress_build_hold(IngressHoldInput {
                settle_cap: Duration::ZERO,
                ..holding()
            }),
            IngressBuildHold::Inactive
        );
    }

    #[test]
    fn a_nonzero_settle_cap_never_bounds_this_term() {
        // The whole reason this hold exists: the cap governs the quiet
        // rule, and bounding an evidence-backed hold by the DEFAULT 2 s cap
        // would reintroduce the defect. Elapsed is 10 s, far past the 2 s cap,
        // and the term still holds.
        let input = holding();
        assert!(input.elapsed > input.settle_cap);
        assert_eq!(decide_ingress_build_hold(input), IngressBuildHold::Hold);
    }

    #[test]
    fn the_stall_window_is_a_threshold_pinned_on_both_sides() {
        // One millisecond either side of the boundary, so a `>` / `>=` slip is
        // caught rather than absorbed.
        let just_inside = decide_ingress_build_hold(IngressHoldInput {
            since_last_creation: Some(Duration::from_millis(4_999)),
            ..holding()
        });
        assert_eq!(just_inside, IngressBuildHold::Hold);
        let at_the_boundary = decide_ingress_build_hold(IngressHoldInput {
            since_last_creation: Some(Duration::from_secs(5)),
            ..holding()
        });
        assert_eq!(at_the_boundary, IngressBuildHold::ReleasedStalled);
    }

    #[test]
    fn the_ceiling_is_a_threshold_pinned_on_both_sides() {
        let just_inside = decide_ingress_build_hold(IngressHoldInput {
            elapsed: Duration::from_millis(119_999),
            ..holding()
        });
        assert_eq!(just_inside, IngressBuildHold::Hold);
        let at_the_boundary = decide_ingress_build_hold(IngressHoldInput {
            elapsed: Duration::from_secs(120),
            ..holding()
        });
        assert_eq!(at_the_boundary, IngressBuildHold::ReleasedAtCeiling);
    }

    #[test]
    fn a_stalled_plane_past_the_ceiling_releases_as_stalled_not_as_ceiling() {
        // THE ordering pin. The ceiling arm is the LOUD one, and its warn claims
        // routes were still arriving. If the ceiling were tested first, a plane
        // that finished at t=6 s in a recording that ran past the ceiling would
        // be reported as "still building at the ceiling" — an affirmatively
        // false claim on an ordinary long recording, which is every recording
        // longer than two minutes.
        let verdict = decide_ingress_build_hold(IngressHoldInput {
            elapsed: Duration::from_secs(600),
            since_last_creation: Some(Duration::from_secs(594)),
            ..holding()
        });
        assert_eq!(verdict, IngressBuildHold::ReleasedStalled);
    }

    #[test]
    fn every_advance_re_arms_the_window_so_a_bursty_build_is_carried_across_gaps() {
        // The Go2 shape: bursts separated by multi-second gaps, over ~60 s.
        // Walk a hand-written timeline where each gap is under the window and
        // assert the hold never breaks — this is the behaviour the quiet
        // rule could not provide, since a 500 ms gap released it.
        let timeline = [
            (Duration::from_secs(1), Duration::from_millis(0)),
            (Duration::from_secs(4), Duration::from_secs(3)),
            (Duration::from_secs(12), Duration::from_secs(4)),
            (Duration::from_secs(30), Duration::from_millis(900)),
            (Duration::from_secs(58), Duration::from_secs(4)),
        ];
        for (elapsed, since) in timeline {
            assert_eq!(
                decide_ingress_build_hold(IngressHoldInput {
                    elapsed,
                    since_last_creation: Some(since),
                    ..holding()
                }),
                IngressBuildHold::Hold,
                "a {since:?} gap at t={elapsed:?} must not close the channel set"
            );
        }
        // …and once the build really stops, the very next window closes it.
        assert_eq!(
            decide_ingress_build_hold(IngressHoldInput {
                elapsed: Duration::from_secs(63),
                since_last_creation: Some(Duration::from_secs(5)),
                ..holding()
            }),
            IngressBuildHold::ReleasedStalled
        );
    }

    #[test]
    fn engaged_separates_the_no_claim_arm_from_every_other_verdict() {
        assert!(!IngressBuildHold::Inactive.engaged());
        for verdict in [
            IngressBuildHold::Hold,
            IngressBuildHold::ReleasedStalled,
            IngressBuildHold::ReleasedAtCeiling,
        ] {
            assert!(
                verdict.engaged(),
                "{verdict:?} is a claim about a real hold"
            );
        }
    }

    #[test]
    fn the_shipped_constants_admit_both_release_arms() {
        // Drift guard on the two const-asserts above, stated behaviourally: with
        // the SHIPPED constants a plane can stall before the ceiling (so the
        // ordinary arm is reachable) and can also ride to the ceiling (so the
        // loud arm is reachable). Either being unreachable would make one of the
        // recorder's two release reports dead.
        assert_eq!(
            decide_ingress_build_hold(IngressHoldInput {
                elapsed: INGRESS_BUILD_STALL_WINDOW,
                since_last_creation: Some(INGRESS_BUILD_STALL_WINDOW),
                stall_window: INGRESS_BUILD_STALL_WINDOW,
                ceiling: INGRESS_BUILD_HOLD_CEILING,
                ..holding()
            }),
            IngressBuildHold::ReleasedStalled
        );
        assert_eq!(
            decide_ingress_build_hold(IngressHoldInput {
                elapsed: INGRESS_BUILD_HOLD_CEILING,
                since_last_creation: Some(Duration::ZERO),
                stall_window: INGRESS_BUILD_STALL_WINDOW,
                ceiling: INGRESS_BUILD_HOLD_CEILING,
                ..holding()
            }),
            IngressBuildHold::ReleasedAtCeiling
        );
    }
}
