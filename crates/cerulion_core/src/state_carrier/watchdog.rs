// SPDX-License-Identifier: AGPL-3.0-only
//! The capture child's watchdog — a LIVENESS check, never a duration
//! cap — and the recorder-vs-encoder split, which is what stops a dead recorder being reported
//! as a broken node encoder.
//!
//! # The rule, and why "never a duration cap" is the load-bearing half
//!
//! Take a duration cap that kills the child at `max(5 s, 10x observed)`. A
//! 500 MB serde encode costs **2.5-10 s**, so on a Jetson
//! at the slow end the first child is SIGKILLed mid-encode at the 5 s floor;
//! "observed" is undefined for a child that has never COMPLETED, so the deadline stays
//! at its floor and every subsequent attempt dies at the same point. That node then
//! never produces a completed anchor for the life of the robot — **a node-level,
//! cost-derived refusal**, which is precisely the outcome this watchdog must never produce.
//!
//! So the parent watches the child's progress word
//! ([`method@super::breadcrumb::ChildBreadcrumb::progress`]) and kills only when it
//! has not advanced for [`STATE_STALL_TIMEOUT_NS`]. Total encode time is unbounded; STALL
//! time is bounded; **no quantity derived from state size can end a node's anchor.**
//!
//! # Two conditions, two remedies, two words
//!
//! A dead or wedged `bagd` stops draining the state ring, so the child's next push
//! blocks, so its progress word freezes — indistinguishable, on progress alone, from
//! an encoder that deadlocked. With a one-row vocabulary the
//! operator goes hunting a node bug while the RECORDER is the thing that died, and
//! the escalation ("second consecutive stall for one node escalates immediately")
//! compounds the wrong diagnosis once per cadence.
//!
//! The child therefore stamps [`ChildPhase::RingFull`] BEFORE a push that can block,
//! and this classifier reads it: a stall observed in that phase is
//! [`StallVerdict::Backpressured`], everything else is [`StallVerdict::Stalled`]. One
//! relaxed store on the child side; one comparison here.
//!
//! **Both still end the child.** The split is about the DIAGNOSIS, not about
//! declining to kill: a child left alive holds a full CoW image of the parent
//! and holds its claim slot, so a peer that reads a live claim skips
//! its own anchor as `StillEncoding` for as long as the recorder stays dead. What
//! changes is which half of the system the report points at — and, downstream, that a
//! `Backpressured` verdict must not count toward the per-NODE escalation, because the
//! node did nothing wrong.
//!
//! # Clock-free by construction
//!
//! [`StallWatch::observe`] takes the elapsed time since the caller's previous
//! observation rather than reading a clock. That keeps the whole rule a pure state
//! machine an oracle vector can drive at exact boundaries — the alternative is a test
//! that asserts a WALL, which is unreliable on macOS under background QoS (a nominal
//! 150 ms charged as 1100-1696 ms). The reaper thread owns the
//! clock; the rule owns the decision.
//!
//! NOTE: this module is compiled only on Unix — the `#[cfg(unix)]` gate lives on the
//! `pub mod state_carrier;` declaration in `lib.rs`.

use super::breadcrumb::ChildPhase;

/// How long the progress counter may stand still before the child is killed.
///
/// A fixed constant, deliberately: every "adaptive" formulation ends up derived from
/// something the ENCODE does (its size, its previous duration), and any such quantity
/// re-creates the cost-derived refusal this watchdog exists to avoid. Five seconds is the same
/// order as the pre-existing `BARRIER_BOUNDARY_TIMEOUT`, which is the other place this
/// codebase decides a peer is not coming back.
pub const STATE_STALL_TIMEOUT_NS: u64 = 5_000_000_000;

/// One reading of the child's breadcrumb.
///
/// Deliberately a plain value rather than a borrow of the mapped page: the classifier
/// is pure, and a rule that could re-read the page mid-decision would be deciding
/// against two different observations.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ProgressReading {
    /// The child's liveness counter.
    pub progress: u64,
    /// What the child said it was doing.
    pub phase: ChildPhase,
}

/// What the parent should do about the child, this observation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StallVerdict {
    /// The counter advanced. Nothing to do; the stall accumulator is reset.
    Progressing,
    /// The counter has not advanced, but not yet for [`STATE_STALL_TIMEOUT_NS`].
    Waiting {
        /// Nanoseconds accumulated with no advance.
        stalled_for_ns: u64,
    },
    /// Stalled past the timeout while blocked on a FULL state ring.
    ///
    /// The child is healthy and would resume the instant the ring drained. The remedy
    /// is the RECORDER (`bagd` dead, wedged, or unable to keep up), never the node.
    Backpressured {
        /// Nanoseconds accumulated with no advance.
        stalled_for_ns: u64,
    },
    /// Stalled past the timeout in any other phase — the encoder is not coming back.
    Stalled {
        /// Nanoseconds accumulated with no advance.
        stalled_for_ns: u64,
        /// The phase the child was in, so the report names it.
        phase: ChildPhase,
    },
}

impl StallVerdict {
    /// Whether this verdict ends the child.
    ///
    /// TRUE for both terminal arms — see the module docs: a child left alive holds a
    /// CoW image and a claim slot, so "the recorder is at fault" is a statement about
    /// the REPORT, not a reason to leak a process.
    pub fn should_kill(self) -> bool {
        matches!(self, Self::Backpressured { .. } | Self::Stalled { .. })
    }

    /// Whether this verdict should count toward the per-NODE stall escalation.
    ///
    /// FALSE for [`StallVerdict::Backpressured`]: the rule "second consecutive stall for
    /// one node escalates immediately" exists to surface a node whose encoder cannot
    /// complete, and a dead recorder stalls EVERY node equally. Counting it would
    /// escalate the whole graph on the first cadence after `bagd` died, naming a set
    /// of nodes none of which is broken.
    pub fn blames_the_node(self) -> bool {
        matches!(self, Self::Stalled { .. })
    }
}

/// The parent-side stall accumulator: one per live child.
///
/// Constructed at fork, driven by the reaper thread on its existing cadence, dropped
/// when the child is reaped.
#[derive(Debug, Clone, Copy)]
pub struct StallWatch {
    /// The highest progress value ever observed for this child.
    watermark: u64,
    /// Nanoseconds accumulated since the watermark last moved.
    stalled_for_ns: u64,
}

impl StallWatch {
    /// A watch for a freshly forked child, whose breadcrumb has just been re-armed to
    /// zero.
    pub fn new() -> Self {
        Self {
            watermark: 0,
            stalled_for_ns: 0,
        }
    }

    /// Fold one observation in and decide.
    ///
    /// `elapsed_ns` is the time since the caller's PREVIOUS observation. The caller
    /// owns the clock (see the module docs).
    ///
    /// # A regression is NOT an advance
    ///
    /// Only a strictly greater reading resets the accumulator, and a lower one leaves
    /// the watermark alone. The counter is monotone by construction (one child, one
    /// thread, `fetch_add`), so a lower reading means the page is corrupted or the
    /// mapping is not the one this child writes — and the permissive reading of that
    /// ("something changed, so it is alive") would let a scribbled-on page keep a
    /// wedged child alive forever, defeating the watchdog by a memory ordering rather
    /// than by a missing rule. Treating it as no-advance bounds the damage at one
    /// stall timeout.
    ///
    /// # Equally: repeating the SAME value is not an advance
    ///
    /// which is why the state is a watermark rather than "the previous reading". A
    /// child flapping between two values would reset a previous-reading comparison on
    /// every other observation and never be killed.
    pub fn observe(&mut self, reading: ProgressReading, elapsed_ns: u64) -> StallVerdict {
        if reading.progress > self.watermark {
            self.watermark = reading.progress;
            self.stalled_for_ns = 0;
            return StallVerdict::Progressing;
        }
        self.stalled_for_ns = self.stalled_for_ns.saturating_add(elapsed_ns);
        if self.stalled_for_ns < STATE_STALL_TIMEOUT_NS {
            return StallVerdict::Waiting {
                stalled_for_ns: self.stalled_for_ns,
            };
        }
        // THE RECORDER-VS-ENCODER SPLIT — the whole of it.
        if reading.phase == ChildPhase::RingFull {
            StallVerdict::Backpressured {
                stalled_for_ns: self.stalled_for_ns,
            }
        } else {
            StallVerdict::Stalled {
                stalled_for_ns: self.stalled_for_ns,
                phase: reading.phase,
            }
        }
    }

    /// The highest progress this child has been observed to reach.
    ///
    /// Reported alongside a terminal verdict so the operator can tell a child that
    /// died before doing anything from one that got a long way in.
    pub fn watermark(&self) -> u64 {
        self.watermark
    }
}

impl Default for StallWatch {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The gate in observation-sized steps, so a vector can sit exactly on either side
    /// of it without arithmetic in the assertions.
    const TICK: u64 = STATE_STALL_TIMEOUT_NS / 10;

    fn at(progress: u64, phase: ChildPhase) -> ProgressReading {
        ProgressReading { progress, phase }
    }

    #[test]
    fn a_progressing_child_is_never_killed_however_long_it_takes() {
        // THE headline property, and the one a duration cap breaks: an
        // encode legitimately longer than STATE_STALL_TIMEOUT must complete. This
        // drives 100 x TICK = 10 x the timeout, an order of magnitude past a capped
        // deadline, advancing by ONE unit each time — the slowest possible healthy
        // child.
        let mut w = StallWatch::new();
        for i in 1..=100u64 {
            let v = w.observe(at(i, ChildPhase::Encoding), TICK);
            assert_eq!(v, StallVerdict::Progressing, "observation {i}");
            assert!(!v.should_kill(), "a progressing child must never be killed");
        }
        assert_eq!(w.watermark(), 100);
    }

    #[test]
    fn the_stall_gate_is_a_threshold_pinned_on_both_sides() {
        // Hand oracle: nine ticks accumulate to 0.9 x the timeout and must still be
        // Waiting; the tenth lands exactly ON it and must be terminal. A gate written
        // `>` instead of `>=` passes the first half and fails the second.
        let mut w = StallWatch::new();
        assert_eq!(
            w.observe(at(1, ChildPhase::Encoding), TICK),
            StallVerdict::Progressing
        );
        for i in 1..=9u64 {
            assert_eq!(
                w.observe(at(1, ChildPhase::Encoding), TICK),
                StallVerdict::Waiting {
                    stalled_for_ns: i * TICK
                },
                "tick {i} must still be Waiting"
            );
        }
        assert_eq!(
            w.observe(at(1, ChildPhase::Encoding), TICK),
            StallVerdict::Stalled {
                stalled_for_ns: STATE_STALL_TIMEOUT_NS,
                phase: ChildPhase::Encoding,
            },
            "the observation landing exactly on the timeout is terminal"
        );
    }

    #[test]
    fn amendment_8_a_ring_full_stall_blames_the_recorder_not_the_node() {
        // The two conditions are driven with IDENTICAL progress vectors and differ in
        // exactly one field — the phase — so nothing but the phase comparison can
        // separate them.
        let mut recorder_dead = StallWatch::new();
        let mut encoder_wedged = StallWatch::new();
        assert_eq!(
            recorder_dead.observe(at(50, ChildPhase::RingFull), TICK),
            StallVerdict::Progressing
        );
        assert_eq!(
            encoder_wedged.observe(at(50, ChildPhase::Encoding), TICK),
            StallVerdict::Progressing
        );
        let mut a = StallVerdict::Progressing;
        let mut b = StallVerdict::Progressing;
        for _ in 0..10 {
            a = recorder_dead.observe(at(50, ChildPhase::RingFull), TICK);
            b = encoder_wedged.observe(at(50, ChildPhase::Encoding), TICK);
        }
        assert_eq!(
            a,
            StallVerdict::Backpressured {
                stalled_for_ns: STATE_STALL_TIMEOUT_NS
            },
            "a stall in RingFull is the RECORDER's fault"
        );
        assert_eq!(
            b,
            StallVerdict::Stalled {
                stalled_for_ns: STATE_STALL_TIMEOUT_NS,
                phase: ChildPhase::Encoding
            },
            "a stall anywhere else is the encoder's"
        );

        // Both end the child (it holds a CoW image and a claim slot either way) ...
        assert!(a.should_kill(), "backpressured children are still ended");
        assert!(b.should_kill());
        // ... but only ONE of them counts against the node.
        assert!(
            !a.blames_the_node(),
            "a dead recorder stalls every node equally; escalating names innocents"
        );
        assert!(b.blames_the_node());
    }

    #[test]
    fn a_ring_full_child_that_resumes_is_never_terminal() {
        // The healthy backpressure shape: the ring fills, the recorder drains, the
        // child resumes. It must never reach a terminal verdict, so a classifier that
        // keyed on the PHASE alone (rather than on the phase AND a frozen counter)
        // fails here.
        let mut w = StallWatch::new();
        let mut progress = 0u64;
        for round in 0..20 {
            // Blocked for most of the timeout ...
            for _ in 0..9 {
                let v = w.observe(at(progress, ChildPhase::RingFull), TICK);
                assert!(
                    !v.should_kill(),
                    "round {round} must not kill a resuming child"
                );
            }
            // ... then the recorder drains and the push lands.
            progress += 1;
            assert_eq!(
                w.observe(at(progress, ChildPhase::RingFull), TICK),
                StallVerdict::Progressing
            );
        }
        assert_eq!(w.watermark(), 20);
    }

    #[test]
    fn an_unknown_phase_is_reported_as_a_stall_not_as_backpressure() {
        // A corrupted or future-version phase word must not be able to claim "the
        // recorder is dead" — that would send an operator to the wrong half of the
        // system on evidence the child never produced. The safe direction is the
        // generic verdict, which names the phase it actually read.
        let mut w = StallWatch::new();
        w.observe(at(1, ChildPhase::Unknown), TICK);
        let mut v = StallVerdict::Progressing;
        for _ in 0..10 {
            v = w.observe(at(1, ChildPhase::Unknown), TICK);
        }
        assert_eq!(
            v,
            StallVerdict::Stalled {
                stalled_for_ns: STATE_STALL_TIMEOUT_NS,
                phase: ChildPhase::Unknown,
            }
        );
        assert!(v.blames_the_node());
    }

    #[test]
    fn a_regressing_or_repeating_counter_is_not_an_advance() {
        // Neither shape may reset the accumulator: a page being scribbled on, and a
        // child flapping between two values, would each defeat the watchdog forever
        // under a previous-reading comparison. Both are driven past the gate here.
        let mut regressing = StallWatch::new();
        regressing.observe(at(1000, ChildPhase::Encoding), TICK);
        let mut v = StallVerdict::Progressing;
        for i in 0..10 {
            // Strictly decreasing, so a "changed => alive" rule would call every one
            // of these an advance.
            v = regressing.observe(at(999 - i, ChildPhase::Encoding), TICK);
        }
        assert!(
            v.should_kill(),
            "a regressing counter must not keep a child alive"
        );
        assert_eq!(
            regressing.watermark(),
            1000,
            "the watermark must not follow a regression down"
        );

        let mut flapping = StallWatch::new();
        flapping.observe(at(7, ChildPhase::Encoding), TICK);
        let mut v = StallVerdict::Progressing;
        for i in 0..20 {
            v = flapping.observe(
                at(if i % 2 == 0 { 6 } else { 7 }, ChildPhase::Encoding),
                TICK,
            );
        }
        assert!(
            v.should_kill(),
            "a flapping counter must not keep a child alive"
        );
    }

    #[test]
    fn a_single_long_observation_can_cross_the_gate_on_its_own() {
        // The reaper's cadence is not guaranteed: a preempted thread can produce one
        // observation covering the whole window. Accumulating (rather than counting
        // observations) is what makes that behave.
        let mut w = StallWatch::new();
        w.observe(at(3, ChildPhase::Encoding), 1);
        assert_eq!(
            w.observe(at(3, ChildPhase::Encoding), STATE_STALL_TIMEOUT_NS),
            StallVerdict::Stalled {
                stalled_for_ns: STATE_STALL_TIMEOUT_NS,
                phase: ChildPhase::Encoding,
            }
        );
    }

    #[test]
    fn a_fresh_watch_reports_progress_on_the_childs_first_bump() {
        // The breadcrumb is re-armed to zero per child, so the first bump is a
        // strictly-greater reading against the zero watermark. A watch initialised
        // with a sentinel high watermark would report the first bump as a stall.
        let mut w = StallWatch::new();
        assert_eq!(w.watermark(), 0);
        assert_eq!(
            w.observe(at(1, ChildPhase::Starting), TICK),
            StallVerdict::Progressing
        );
    }

    #[test]
    fn a_child_that_never_bumps_at_all_is_still_bounded() {
        // The pathological shape: fork lands in an inherited lock and the child never
        // reaches its first unit of work. Progress stays at the re-armed zero, which
        // is NOT strictly greater than the zero watermark, so the accumulator runs and
        // the child is killed. (An `observe` written `>=` would call this Progressing
        // forever and hang the anchor.)
        let mut w = StallWatch::new();
        let mut v = StallVerdict::Progressing;
        for _ in 0..10 {
            v = w.observe(at(0, ChildPhase::Starting), TICK);
        }
        assert_eq!(
            v,
            StallVerdict::Stalled {
                stalled_for_ns: STATE_STALL_TIMEOUT_NS,
                phase: ChildPhase::Starting,
            }
        );
        assert_eq!(w.watermark(), 0);
    }
}
