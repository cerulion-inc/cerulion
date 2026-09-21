// SPDX-License-Identifier: AGPL-3.0-only
//! The WINDOW-ONLY recorder's scheduler-trace drain.
//!
//! A `--record` recorder's trace rings are drained by its WRITER THREAD, which
//! splices them into the continuous bag and pushes the same span into the
//! rolling retention on the way past (`WriterCore::write_batch`). A window-only
//! recorder writes no continuous bag, so it has no writer thread — and until
//! this module it therefore had no drain at all: a run's rings could be handed
//! to the always-on recorder and would simply fill up unread, which is the gap
//! `roll_trace_retention`'s doc named as this module's to close.
//!
//! This is that drain: the ONE reader this recorder has on those rings, feeding
//! the SAME retention from the SAME origin, so a capture taken by a window-only
//! recorder carries a trace on exactly the terms a `--record` one does.
//!
//! # A second READER of the ring, not a second reader inside one recorder
//!
//! A trace ring is `OverrunPolicy::FailLoud`: every consumer maps its own view
//! and keeps a LOCAL read cursor, publishes nothing, and the producer never
//! looks (`cerulion_core::shm_ring` module doc contract 7).
//! So this thread reading a run's ring while the run's own `--record` recorder,
//! or a mid-run `cerulion bag record --run`, reads the same one is the ring's
//! documented shape rather than a hazard. What is never fine is one RECORDER
//! reading one ring twice, because both reads land in one artifact — which is
//! why this drain exists only in the window-only mode, where nothing else in
//! this process consumes those rings.
//!
//! # Its OWN thread, and that is a lap-exposure decision
//!
//! The obvious home is the drive loop, beside `harvest_window` and
//! `harvest_anchors`. It is the wrong one. The drive loop also runs live
//! discovery, which walks the iceoryx2 service directory INLINE (measured at
//! 117.5 ms p50 at the Go2's 86 topics; still open), and serialises capture
//! writes (~155 MB onto eMMC). A trace drain sitting there inherits every one of
//! those stalls as time the ring is not being read, and a ring that is not being
//! read is a ring the producer is catching up with: at the 11k records/s of a
//! 10-node 1 kHz graph the whole 2^20-record ring is 95 seconds wide, and the
//! stall budget is what decides whether a lap ever happens. So the drain gets a
//! thread whose only job is to keep the cursor moving, and the retention (which
//! IS shared with the drive loop) is touched only under its own short lock.
//!
//! # Open discipline: from the START first, live only as a fallback
//!
//! `Recorder::setup` opens a declared ring with plain `open` (cursor 0) unless
//! the recorder attached mid-run. That is deliberately the FIRST thing tried
//! here too, because a capture whose trace reaches step 0 is judged by resim's
//! from-start arm and needs no anchor at all — and the always-on recorder is
//! spawned within milliseconds of the run, i.e. inside the first lap by three
//! orders of magnitude. A plain `open` does NOT refuse a ring that has already
//! lapped; it succeeds, and the FIRST `drain_slices` returns `Overrun`
//! (`shm_ring`). That is the fallback's trigger: re-open at the live cursor,
//! arm the partial-head-step gate, and report the arm as
//! [`TraceAttach::AtLive`].
//!
//! # A lap mid-run is a HOLE, never a retirement
//!
//! The state-ring harvest retires a ring that laps under it, and says so: later
//! captures of that run are frames-only. The project's rule forbids exactly that
//! outcome for the trace, and it would be a bad trade anyway — one recorder
//! stall would cost a multi-hour run its whole black box. So an `Overrun` here
//! RE-OPENS at the live cursor, re-arms the gate, counts the lap and records the
//! hole's edges ([`TraceGap`]). Only a capture that carries records on BOTH
//! sides of a hole is refused (`ResimGap::TraceLapped`); a capture whose window
//! begins after the re-attach is resimmable, which is the whole point of
//! re-opening rather than retiring.

use std::collections::{BTreeMap, VecDeque};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use cerulion_core::flashback::resim::TraceGap;
use cerulion_core::shm_ring::ShmRingError;
use cerulion_core::trace_ring::{
    HeadStepGate, TraceRingConsumer, TraceRingError, TraceRingRecord, TRACE_DISCARD_BIT,
};

use crate::flashback_plane::{lock_trace, SharedTraceWindow};

/// How long the drain thread sleeps between passes.
///
/// Short relative to every stall it exists to be immune from (a 117 ms discovery
/// walk, a capture write), and long enough that an idle recorder is not spinning
/// — the ring is a wait-free SHM queue, so a pass over an empty one is a pair of
/// atomic loads. It is a CADENCE, never a budget: nothing here is asserted in
/// units of it.
pub(crate) const TRACE_DRAIN_TICK: Duration = Duration::from_millis(5);

/// How many distinct holes this drain will enumerate before it starts MERGING
/// them.
///
/// See [`TraceDrainState::note_gap`] for why merging rather than forgetting, and
/// why a run that reaches this number has already failed at something else.
pub(crate) const MAX_TRACKED_GAPS: usize = 64;

/// Which cursor this recorder's reader started from — the manifest's
/// `trace_attach`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum TraceAttach {
    /// Cursor 0: every record the ring has ever held is in reach, so a capture
    /// can cover step 0 and be judged by resim's from-start arm.
    FromStart,
    /// The live cursor, because the ring had already lapped when this recorder
    /// first read it. Carries the head-step gate's own accounting, which is what
    /// makes the degrade measurable rather than merely asserted.
    AtLive {
        /// The first COMPLETE step this reader admitted — the gate's
        /// `first_step_recorded`. `None` until a boundary has been seen.
        first_step_recorded: Option<u64>,
        /// Records discarded for want of a leading boundary.
        discarded: u64,
    },
}

impl TraceAttach {
    /// The manifest token.
    pub(crate) fn render(&self) -> String {
        match self {
            Self::FromStart => "from_start: this recorder read this run's trace ring from record \
                                0, so a capture reaching step 0 needs no anchor"
                .to_string(),
            Self::AtLive {
                first_step_recorded,
                discarded,
            } => {
                let first = first_step_recorded
                    .map(|s| format!("step {s}"))
                    .unwrap_or_else(|| "no step boundary yet".to_string());
                format!(
                    "at_live: this run's trace ring had already lapped when this recorder first \
                     read it, so it attached at the LIVE cursor — records committed before that \
                     are not in reach of any capture. Its first complete step was {first} \
                     ({discarded} leading record(s) discarded for want of a boundary)"
                )
            }
        }
    }
}

/// What the drain has done — the recorder's window onto a thread it does not
/// otherwise touch (Principle #3).
#[derive(Debug, Clone, Default)]
pub(crate) struct TraceDrainState {
    /// Records ADMITTED into the retention, lifetime. Not records consumed: a
    /// head-gate discard was really read, and counting it here would advertise a
    /// coverage the retention does not have.
    pub(crate) records_admitted: u64,
    /// Holes observed, lifetime. UNCONDITIONAL — it counts every lap whether or
    /// not the hole is still enumerable below, so a merged list can never make
    /// the run look healthier than it was.
    pub(crate) laps: u64,
    /// The holes still enumerable, KEYED BY RING and oldest-first within each.
    ///
    /// # Why a map and not one list
    ///
    /// A hole belongs to the RING it opened on. It has to, because both of the
    /// rules below are stated over "the most recent hole" and both were written
    /// for one ring: the [`note_gap`](Self::note_gap) collapse ("nothing was
    /// admitted, so this is the same hole") and the [`close_gap`](Self::close_gap)
    /// close ("the first record across it"). Over ONE shared list with several
    /// rings draining into it, both rules reach for another ring's hole.
    ///
    /// MEASURED on the shared-list shape: ring A laps and opens a hole; ring B
    /// laps and `note_gap` COLLAPSES it into A's, because A's is still open; A
    /// then admits and `close_gap` closes the back with A's step; B's later admit
    /// finds the back already closed and does nothing. B's lost interval is now
    /// in no list at all, so `gap_spanning` cannot see it and a capture holding
    /// records on both sides of B's hole is ACCEPTED — the confident-false
    /// direction this whole verdict exists to prevent.
    ///
    /// Keying by ring makes that unrepresentable rather than merely fixed: there
    /// is no expression for "the newest hole" that is not already scoped to one
    /// ring's holes. `BTreeMap` rather than a hash map so the enumeration order
    /// a capture's refusal is chosen from is deterministic.
    ///
    /// UNREACHABLE while a window-only recorder is handed
    /// one ring, and live the moment it is handed one per rank.
    pub(crate) gaps: BTreeMap<String, RingHoles>,
    /// Rings this drain LOST — an open or decode failure that is not a lap, so
    /// there is nothing to re-attach to.
    pub(crate) rings_retired: Vec<(String, String)>,
    /// Per-ring attach arm, in the order the rings were handed over.
    pub(crate) attach: Vec<(String, TraceAttach)>,
}

/// One ring's holes, plus whether there were more than can be enumerated.
///
/// # Why the bit exists — a merged hole is NOT a conservative superset
///
/// The obvious bound is to merge the list into one wide hole. It is wrong, and
/// wrong in the ACCEPTING direction, because widening a hole makes it HARDER to
/// straddle rather than easier: `TraceGap::spans` refuses a capture that holds a
/// record at or before the near edge AND one at or after the far edge, so
/// stretching the far edge out to the newest lap means a capture that really did
/// straddle an OLD narrow hole no longer matches. There is no single hole whose
/// refusal set covers a set of disjoint holes — it would need a near edge at the
/// newest hole's and a far edge at the oldest hole's, i.e. a hole running
/// backwards.
///
/// So past the ceiling the correct answer is not a cleverer hole, it is an
/// admission: this ring's continuity is UNPROVABLE, and every capture carrying
/// its records is refused. The cost is real and stated — a run that reaches the
/// ceiling can take no resimmable capture from that ring again — and so is the
/// reachability: at the shipped ring size a single lap needs the drain starved
/// for ~95 seconds, so [`MAX_TRACKED_GAPS`] of them is a recorder that has been
/// broken for hours. Refusing there is the safe direction (the rule
/// forbids a false claim, not a refusal), and the memory stays bounded.
#[derive(Debug, Clone, Default)]
pub(crate) struct RingHoles {
    /// The holes still enumerable, oldest first.
    pub(crate) holes: VecDeque<TraceGap>,
    /// More holes have been observed than are enumerable above. STICKY: nothing
    /// can un-forget a hole.
    pub(crate) overflowed: bool,
}

impl TraceDrainState {
    /// Record a hole, MERGING rather than forgetting once
    /// [`MAX_TRACKED_GAPS`] are held.
    ///
    /// A capture is judged against the holes that fall inside the records it
    /// CARRIES, so a hole that has aged out of the retention can never matter —
    /// but this thread cannot see the retention's floor, so it cannot prune on
    /// that basis. Dropping the oldest instead would be unsound in exactly the
    /// direction that matters: a capture spanning a forgotten hole would be
    /// stamped `resimmable: true` against a bag with a gap in it, which is the
    /// confident-false class this whole verdict exists to prevent.
    ///
    /// So a full list COLLAPSES to one hole running from the oldest hole's near
    /// edge to the newest's far edge. That is a SUPERSET of every hole it
    /// replaces: it spans everything they spanned plus the intact stretches
    /// between them, so it can refuse a capture that was in fact continuous but
    /// can never accept one that was not. The cost is stated rather than hidden,
    /// and it is only reachable after [`MAX_TRACKED_GAPS`] laps in one run —
    /// a recorder that has been lapped 64 times has a problem the verdict is not
    /// the right place to discover.
    pub(crate) fn note_gap(&mut self, ring: &str, gap: TraceGap) {
        // Lifetime and ACROSS rings — see the field's own doc for why that is
        // the right scope for a count while the holes themselves are per-ring.
        self.laps += 1;
        let entry = self.gaps.entry(ring.to_string()).or_default();
        let holes = &mut entry.holes;
        // TWO LAPS ON ONE RING WITH NOTHING ADMITTED BETWEEN THEM ARE ONE HOLE,
        // and keeping them apart is unsound rather than merely untidy:
        // `close_gap` closes the NEWEST hole, so a second one pushed while the
        // first is still open would leave the first unclosed FOREVER — and an
        // unclosed hole spans nothing (see `TraceGap::first_step_after`), so a
        // capture straddling it would be accepted. Nothing was admitted, so this
        // ring's `last_step` has not moved and the second hole's near edge is the
        // first's: keeping the first IS keeping both.
        //
        // Scoped to ONE RING's holes, because the premise ("nothing was
        // admitted") is a fact about that ring's drain and about no other.
        if holes.back().is_some_and(|g| g.first_step_after.is_none()) {
            return;
        }
        if entry.holes.len() >= MAX_TRACKED_GAPS {
            // PAST THE CEILING. The oldest hole leaves the list, and the ring is
            // marked UNPROVABLE — see `RingHoles` for why a merged hole cannot
            // stand in for the ones it replaces, and why refusing is the correct
            // answer here rather than a cleverer envelope.
            //
            // The recent holes are kept anyway, because they are what a refusal
            // quotes: an operator reads real step numbers rather than a sentinel.
            entry.overflowed = true;
            entry.holes.pop_front();
        }
        entry.holes.push_back(gap);
    }

    /// Close THIS RING's newest hole with the first step admitted after its
    /// re-attach.
    ///
    /// Separate from [`note_gap`](Self::note_gap) because the far edge is not
    /// known when the hole opens — nothing has crossed it yet — and a capture
    /// taken in between must see an explicit `None` rather than a guess. See
    /// [`TraceGap::first_step_after`].
    ///
    /// Scoped to the OWNING ring: the step handed in came off THAT ring's drain,
    /// and using it to close another ring's hole would both fabricate a far edge
    /// for a hole nothing has crossed AND leave the real one open forever.
    pub(crate) fn close_gap(&mut self, ring: &str, first_step_after: u64) {
        let Some(entry) = self.gaps.get_mut(ring) else {
            return;
        };
        if let Some(gap) = entry.holes.back_mut() {
            if gap.first_step_after.is_none() {
                gap.first_step_after = Some(first_step_after);
            }
        }
    }

    /// The hole THIS CAPTURE carries both sides of, if any — judged against the
    /// bounds of every record it carries, over EVERY ring.
    ///
    /// # Why the bounds are CAPTURE-WIDE and not each ring's own
    ///
    /// The alternative — judge each ring's hole only against the records THAT
    /// ring contributed — was proposed on the grounds that another ring's
    /// records should not drag a capture's window over a hole it has no records
    /// either side of. It is the wrong rule, and it is wrong in the ACCEPTING
    /// direction, which is the direction this whole verdict exists to close.
    ///
    /// A capture is ONE TIME WINDOW over every ring of a run, and those rings
    /// step in LOCKSTEP: the cross-rank step boundaries of one
    /// step are the same step, so a step number means the same instant on every
    /// ring. If the capture covers steps 10 to 100 and ring A's records for
    /// those steps were provably DESTROYED, the capture's trace is incomplete
    /// for those steps — whichever ring happened to supply the record at 200
    /// that fixes the upper bound. Judging A's hole against A's own thinner
    /// bounds would UN-REFUSE exactly that capture, i.e. stamp resimmable on a
    /// trace known to be missing in-window records. A false refusal costs an
    /// operator one capture; a false acceptance replays a different run and says
    /// nothing.
    ///
    /// The divergence is also not reachable as a WRONG VERDICT on the shipping
    /// recorder, which is worth stating because it bounds what the rule can cost
    /// today: with ONE ring the capture-wide bounds ARE that ring's bounds, and
    /// with several, `write_capture` passes `required_nodes_known: false` (the
    /// node map is resolvable only from a single ring), so the judge refuses
    /// with `AmbiguousNodeMap` whatever this answers. What the choice decides
    /// today is therefore WHICH refusal an operator reads — and a hole with two
    /// step numbers is the more actionable one.
    ///
    /// The judge re-asks `TraceGap::spans` over the SAME capture-wide bounds
    /// (`ResimFacts::{min,max}_recorded_step`, built the same way from the same
    /// records), so the selection here and the decision there cannot drift.
    pub(crate) fn gap_spanning_carried(&self, records: &[TraceRingRecord]) -> Option<TraceGap> {
        self.gap_spanning(
            records.iter().map(|r| r.step).min(),
            records.iter().map(|r| r.step).max(),
        )
    }

    /// The hole this capture carries both sides of, if any — over EVERY ring.
    ///
    /// A capture's trace is the union of what every ring contributed, so a hole
    /// in ANY of them is a hole in the capture. Walked in (ring, age) order and
    /// answering with the first match, so the refusal names one hole
    /// deterministically rather than an arbitrary one.
    pub(crate) fn gap_spanning(
        &self,
        min_step: Option<u64>,
        max_step: Option<u64>,
    ) -> Option<TraceGap> {
        // An EXACT match first, on every ring, so a refusal quotes the hole
        // the capture really straddles wherever one is still enumerable.
        if let Some(exact) = self
            .gaps
            .values()
            .flat_map(|entry| entry.holes.iter().copied())
            .find(|g| g.spans(min_step, max_step))
        {
            return Some(exact);
        }
        // Then the UNPROVABLE arm: a ring past the ceiling forgot holes, so a
        // capture carrying records cannot be shown to be clear of them. The
        // OLDEST retained hole is quoted — real edges, and the closest thing to
        // the forgotten region there is. A capture carrying nothing is not
        // refused here: it straddles nothing by construction, and
        // `ResimGap::NoTrace` is the arm that owns an empty trace.
        if min_step.is_none() || max_step.is_none() {
            return None;
        }
        self.gaps
            .values()
            .find(|entry| entry.overflowed)
            .and_then(|entry| entry.holes.front().copied())
    }

    /// The recorder's ONE attach verdict over however many rings it holds.
    ///
    /// `AtLive` wins, because the claim a capture makes is about the trace it
    /// carries as a whole: if ANY ring's records begin mid-run, the capture
    /// cannot say its trace reaches step 0. The gate figures are summed for the
    /// same reason.
    pub(crate) fn attach_verdict(&self) -> Option<TraceAttach> {
        if self.attach.is_empty() {
            return None;
        }
        let mut first_step_recorded: Option<u64> = None;
        let mut discarded = 0u64;
        let mut any_live = false;
        for (_, arm) in &self.attach {
            if let TraceAttach::AtLive {
                first_step_recorded: f,
                discarded: d,
            } = arm
            {
                any_live = true;
                discarded += d;
                // The LATEST first-complete-step across the rings: a capture's
                // trace reaches back only as far as its least-covered ring.
                first_step_recorded = match (first_step_recorded, f) {
                    (Some(a), Some(b)) => Some(a.max(*b)),
                    (None, Some(b)) => Some(*b),
                    (a, None) => a,
                };
            }
        }
        Some(if any_live {
            TraceAttach::AtLive {
                first_step_recorded,
                discarded,
            }
        } else {
            TraceAttach::FromStart
        })
    }
}

/// The state the drain thread publishes and the recorder reads.
pub(crate) type SharedTraceDrain = Arc<Mutex<TraceDrainState>>;

/// One ring, with everything the drain needs to re-attach to it.
struct DrainRing {
    consumer: TraceRingConsumer,
    gate: HeadStepGate,
    /// The ring's SHM name, kept apart from the consumer so a re-open can be
    /// attempted after the consumer that knew it has been dropped.
    name: String,
    /// The last step ADMITTED off this ring — the near edge of the next hole.
    last_step: Option<u64>,
    /// A hole is open on this ring and the next admitted record closes it.
    gap_open: bool,
    /// The attach ARM has been settled by this ring's FIRST drain outcome.
    ///
    /// Decided ONCE, and that is what makes [`TraceAttach`] mean what it says.
    /// The arm answers "which cursor did this reader START from" — a fact about
    /// the run's beginning that a later mid-run lap does not change, and must not
    /// be allowed to rewrite: a ring read from record 0 that is lapped an hour in
    /// was still read from record 0, and the hour of records it delivered are
    /// still in reach. What the lap costs is reported as a HOLE, which is a
    /// different fact with a different consequence.
    attach_decided: bool,
}

/// The window-only recorder's trace drain: a thread, its stop flag, and the
/// state it publishes.
///
/// Stopped and JOINED by `Drop`, so every exit path out of the recorder — the
/// clean one, the salvage-on-error one, an early return from a `?` — stops it,
/// rather than each having to remember. The join is bounded by one
/// [`TRACE_DRAIN_TICK`] plus one pass.
#[derive(Debug)]
pub(crate) struct TraceDrainHandle {
    stop: Arc<AtomicBool>,
    handle: Option<JoinHandle<()>>,
    state: SharedTraceDrain,
}

impl TraceDrainHandle {
    /// The state the thread publishes. Cloned rather than borrowed because the
    /// recorder reads it while the thread writes it.
    pub(crate) fn state(&self) -> SharedTraceDrain {
        Arc::clone(&self.state)
    }

    /// Stop the thread and JOIN it, so its FINAL PASS has run before the caller
    /// reads the retention.
    ///
    /// # Why `Drop` is not enough
    ///
    /// The loop runs one more `drain_pass` after it observes the stop flag,
    /// precisely so records committed during shutdown reach the retention. But
    /// if that pass were reachable only through `Drop`, which fires when the
    /// `Recorder` is dropped — AFTER `finalize` has resolved the outstanding
    /// capture and built the summary, the tail records the final pass exists
    /// to collect would land in a retention nobody would read again: the last
    /// capture of a run, which is the one an incident is most likely to be in,
    /// was missing them.
    ///
    /// IDEMPOTENT: the handle is taken, so a later `Drop` is a no-op and calling
    /// this twice is safe.
    pub(crate) fn finish(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        let Some(handle) = self.handle.take() else {
            return;
        };
        if handle.join().is_err() {
            tracing::error!(
                "flashback: the scheduler-trace drain thread PANICKED — this \
                 recorder's captures carry whatever trace reached the retention before it \
                 died, and no more"
            );
        }
    }
}

impl Drop for TraceDrainHandle {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(h) = self.handle.take() {
            // A panicked drain thread is reported, never propagated: it is an
            // observability plane, and unwinding out of a `Drop` while the
            // recorder is finalizing would cost the operator the bag as well as
            // the trace.
            if h.join().is_err() {
                tracing::error!(
                    "flashback: the scheduler-trace drain thread PANICKED — this \
                     recorder's captures carry whatever trace reached the retention before it \
                     died, and no more"
                );
            }
        }
    }
}

// TEST SEAM: force the NEXT [`spawn`] on THIS THREAD to take its failure arm.
//
// A real `Builder::spawn` failure needs thread exhaustion, so without a seam
// the no-reader path can only be asserted by hand-editing the state — which
// tests the assertion and not the code. (Deleting the
// production `attach.clear()` survives a hand-driven arm.)
//
// THREAD-LOCAL, not a global: `spawn` runs on its caller's thread, so the flag
// is already scoped to the test that armed it and the drain suite needs no
// serialization.
//
// Gated on `test` ALONE, not `any(test, feature = "test-helpers")` like the
// crate's other seams: only this file's own unit test drives it, and under the
// feature a NON-test lib build (which workspace feature unification really does
// produce) would compile the fn with no `mod tests` to use it — which
// `dead_code = "deny"` rejects.
#[cfg(test)]
thread_local! {
    static FAIL_NEXT_SPAWN: std::cell::Cell<bool> = const { std::cell::Cell::new(false) };
}

/// Arm the one-shot spawn-failure fault for this thread.
#[cfg(test)]
pub(crate) fn fail_next_drain_spawn_for_test() {
    FAIL_NEXT_SPAWN.with(|f| f.set(true));
}

/// Take the armed fault, if any. Always `false` in production builds.
fn spawn_forced_to_fail() -> bool {
    #[cfg(test)]
    {
        FAIL_NEXT_SPAWN.with(|f| f.replace(false))
    }
    #[cfg(not(test))]
    {
        false
    }
}

/// Where the recorder PLACED the read cursor when it opened its rings.
///
/// A TYPE threaded from the opener rather than a fact re-derived downstream,
/// because the only other witness — the head-step gate — cannot answer for every
/// ring. `ring_head_gate` gives a DEPARTURE ring a PASSTHROUGH gate by design
/// (a departure ring carries no `STEP_BOUNDARY`, so an armed gate over
/// it discards every record forever), *including* on a mid-run attach where that
/// ring was opened at the LIVE cursor. Inferring the arm from `gate.is_open()`
/// therefore reported `from_start` — "this reader read from record 0" — about a
/// reader that provably had not, which is the Principle #3 class this vocabulary
/// exists to prevent.
///
/// One value per recorder, not per ring: `Recorder::setup` opens every ring with
/// the same `attached_mid_run`, and a per-ring vector would invent a distinction
/// the opener does not make.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum RingCursorOrigin {
    /// `TraceRingConsumer::open` — cursor 0; the first record read IS record 0.
    RecordZero,
    /// `TraceRingConsumer::open_at_live` — cursor := write_cursor; everything
    /// committed before the attach is unreadable by this reader.
    Live,
}

/// Spawn the drain over `rings`, feeding `trace` from `origin`.
///
/// `gates` is index-aligned with `rings`, exactly as `Recorder::ring_gates` is
/// with `Recorder::rings` — the pairing chosen at open by `ring_head_gate`, so a
/// departure ring (which carries no boundary at all) keeps its passthrough gate
/// rather than discarding everything forever.
///
/// `cursor_origin` is where those rings were opened; see [`RingCursorOrigin`].
pub(crate) fn spawn(
    rings: Vec<TraceRingConsumer>,
    gates: Vec<HeadStepGate>,
    cursor_origin: RingCursorOrigin,
    trace: SharedTraceWindow,
    origin: Instant,
) -> TraceDrainHandle {
    debug_assert_eq!(rings.len(), gates.len());
    let state: SharedTraceDrain = Arc::new(Mutex::new(TraceDrainState::default()));
    // The arms are seeded here rather than discovered on the first pass, so a
    // recorder that is asked before the thread has run once reports the arm its
    // rings were OPENED on rather than nothing.
    //
    // SEEDED FROM THE CURSOR ORIGIN, not assumed and not inferred from the gate.
    //
    // Seeding every ring `FromStart` (the earlier code) made a late-attached recorder's
    // captures and `/bagd/status` claim they read from record 0 when they
    // demonstrably had not. A later fix reads `gate.is_open()`, which
    // is right for every ring the gate can speak for — and WRONG for a departure
    // ring, which gets a passthrough gate by design even on a mid-run attach and
    // so read back as `FromStart` again. The opener knows the answer for every
    // ring, so it hands it over; see [`RingCursorOrigin`].
    //
    // The GATE still supplies the `AtLive` PAYLOAD — how much of the head it
    // discarded, and the first complete step it recorded — because those are
    // facts about what this reader kept, which only the gate observes. On a
    // departure ring both stay at their real zeroes: nothing is discarded and
    // there is no step boundary to record, ever.
    //
    // The first-drain-outcome rule below still owns the `Overrun` fallback; this
    // only fixes the STARTING claim.
    {
        let mut s = lock_state(&state);
        for (r, gate) in rings.iter().zip(&gates) {
            let arm = match cursor_origin {
                RingCursorOrigin::RecordZero => TraceAttach::FromStart,
                RingCursorOrigin::Live => TraceAttach::AtLive {
                    first_step_recorded: gate.first_step_recorded(),
                    discarded: gate.discarded(),
                },
            };
            s.attach.push((r.name().to_string(), arm));
        }
    }
    let mut drain_rings: Vec<DrainRing> = rings
        .into_iter()
        .zip(gates)
        .map(|(consumer, gate)| DrainRing {
            name: consumer.name().to_string(),
            consumer,
            gate,
            last_step: None,
            gap_open: false,
            attach_decided: false,
        })
        .collect();
    let stop = Arc::new(AtomicBool::new(false));
    let thread_stop = Arc::clone(&stop);
    let thread_state = Arc::clone(&state);
    let handle = if spawn_forced_to_fail() {
        Err(std::io::Error::other(
            "TEST SEAM: forced drain-spawn failure",
        ))
    } else {
        std::thread::Builder::new()
            .name("cer-trace-drain".into())
            .spawn(move || {
                let mut out: Vec<TraceRingRecord> = Vec::new();
                loop {
                    let stopping = thread_stop.load(Ordering::Relaxed);
                    drain_pass(&mut drain_rings, &thread_state, &trace, origin, &mut out);
                    // The pass ABOVE the exit test, so the records a producer
                    // committed while the recorder was shutting down are in the
                    // retention before this thread leaves — a capture written at
                    // finalize is the one that most needs them.
                    if stopping {
                        return;
                    }
                    std::thread::sleep(TRACE_DRAIN_TICK);
                }
            })
    };
    match handle {
        Ok(handle) => TraceDrainHandle {
            stop,
            handle: Some(handle),
            state,
        },
        Err(e) => {
            // A recorder that cannot spawn the drain keeps recording FRAMES.
            // Its captures then say they carry no trace, which is true, and the
            // reason is in the log rather than inferred from a silent absence.
            tracing::error!(
                error = %e,
                "flashback: could not spawn the scheduler-trace drain thread — this \
                 recorder holds this run's trace ring(s) and will read none of them, so its \
                 captures carry frames only"
            );
            // THERE IS NO READER, so there is no attach arm to report. Leaving
            // the seeded arms in place would render a nonexistent reader as
            // `from_start` — a positive claim that this recorder is reading from
            // record 0 — on the one path where it is reading nothing at all.
            // Cleared rather than rewritten to `AtLive`, because that would be a
            // different false claim; `attach_verdict` answers `None` on an empty
            // set and the manifest renders JSON `null`, which is the accurate
            // "nobody to ask".
            lock_state(&state).attach.clear();
            TraceDrainHandle {
                stop,
                handle: None,
                state,
            }
        }
    }
}

/// One pass over every ring.
fn drain_pass(
    rings: &mut Vec<DrainRing>,
    state: &SharedTraceDrain,
    trace: &SharedTraceWindow,
    origin: Instant,
    out: &mut Vec<TraceRingRecord>,
) {
    let mut retired: Vec<(String, String)> = Vec::new();
    for ring in rings.iter_mut() {
        out.clear();
        match ring.consumer.drain_gated(&mut ring.gate, out) {
            Ok(_) => admit(ring, state, trace, origin, out),
            Err(TraceRingError::Ring(ShmRingError::Overrun { records_lost, .. })) => {
                reattach(ring, state, records_lost);
            }
            Err(e) => {
                retired.push((ring.name.clone(), e.to_string()));
            }
        }
    }
    if retired.is_empty() {
        return;
    }
    for (name, reason) in &retired {
        // Not a lap: there is no live cursor to re-attach to, because the ring
        // itself is unreadable. Loud once, and the ring is dropped rather than
        // re-tried every 5 ms.
        tracing::error!(
            ring = %name,
            error = %reason,
            "flashback: a scheduler-trace ring failed under this recorder's drain and \
             was dropped — later captures carry no trace from it"
        );
    }
    rings.retain(|r| !retired.iter().any(|(name, _)| *name == r.name));
    lock_state(state).rings_retired.extend(retired);
}

/// Push one pass's admitted records into the retention and account for them.
fn admit(
    ring: &mut DrainRing,
    state: &SharedTraceDrain,
    trace: &SharedTraceWindow,
    origin: Instant,
    out: &[TraceRingRecord],
) {
    let discarded = ring.gate.discarded();
    let admitted = out.len() as u64;
    let first_step = out.first().map(|r| r.step);
    let last_step = out.last().map(|r| r.step);
    {
        let mut s = lock_state(state);
        s.records_admitted += admitted;
        // A drain that RETURNED settles the arm at its SEEDED value: this reader
        // was not lapped before its first read, whether or not that read yielded
        // anything, so the cursor origin the opener reported still stands.
        ring.attach_decided = true;
        if let Some(step) = first_step {
            if ring.gap_open {
                s.close_gap(&ring.name, step);
                ring.gap_open = false;
            }
        }
        if let Some(arm) = s.attach.iter_mut().find(|(n, _)| *n == ring.name) {
            if let TraceAttach::AtLive {
                first_step_recorded,
                discarded: d,
            } = &mut arm.1
            {
                // FROZEN once the initial gate has found its boundary. The figures
                // describe where THIS READER's trace began, and a later re-attach
                // re-arms a DIFFERENT gate whose zeroes would overwrite that with
                // a number about the hole instead — which the hole already
                // reports, better.
                if first_step_recorded.is_none() {
                    *first_step_recorded = ring.gate.first_step_recorded();
                    *d = discarded;
                }
            }
        }
    }
    if let Some(step) = last_step {
        ring.last_step = Some(step);
    }
    if out.is_empty() {
        return;
    }
    // CO-STAMP the ring's HEADER RANK into every record,
    // exactly as the continuous writer does (`WriterCore::write_batch`:
    // `rank | (on_ring & TRACE_DISCARD_BIT)`).
    //
    // A record's on-ring `reserved` carries the discard bit and NOT the rank —
    // the rank lives in the ring header, and the writer stamps it at drain time
    // so a bag's records are self-describing. Admitting them unstamped left
    // every window-only record claiming rank 0, so a capture off a nonzero-rank
    // ring decoded as `ForeignRank` against its own manifest and could not be
    // resimmed at all. The DISCARD BIT is preserved, because it is on-ring
    // evidence and the rank subfield is the only part the header owns.
    //
    // Read from the consumer each pass rather than cached, so a re-attach that
    // re-read the header cannot leave a stale rank behind.
    let rank = ring.consumer.rank();
    let stamped: Vec<TraceRingRecord> = out
        .iter()
        .map(|r| TraceRingRecord {
            reserved: rank | (r.reserved & TRACE_DISCARD_BIT),
            ..*r
        })
        .collect();
    // The retention lock is taken LAST and held for one push: the drive loop
    // evicts under the same lock on every pass, and this thread exists to keep
    // a cursor moving rather than to hold a mutex.
    lock_trace(trace).push(origin.elapsed().as_nanos() as u64, stamped);
}

/// A lap: re-open at the live cursor, re-arm the gate, record the hole.
fn reattach(ring: &mut DrainRing, state: &SharedTraceDrain, records_lost: u64) {
    match TraceRingConsumer::open_at_live(&ring.name) {
        Ok(consumer) => {
            ring.consumer = consumer;
            // A live cursor lands MID-STEP, so the partial head step has to be
            // discarded whether this is the first read or the fiftieth — the
            // same rule `ring_head_gate` applies to a mid-run attach, and for
            // the same reason.
            //
            // The gate is built through THAT function rather than a bare
            // `armed()`, because the rule has an EXCEPTION that a bare `armed()`
            // ignores. A DEPARTURE ring (`rank == DEPARTURE_RING_RANK`) carries
            // no `STEP_BOUNDARY` record at all, so an armed gate over one
            // discards **everything, forever** — the hazard `HeadStepGate`'s own
            // doc names, and the reason `ring_head_gate` hands that ring a
            // PASSTHROUGH gate even on a mid-run attach. Re-arming
            // unconditionally here meant one lap silently ended a run's
            // fault-evidence stream: the drain kept reading, the records kept
            // being discarded, and nothing said so.
            //
            // `true` for the attach flag because a re-attach IS at the live
            // cursor by construction (`open_at_live` one line up), and the RANK
            // is read off the freshly re-opened consumer rather than cached, so
            // a header re-read cannot leave a stale answer behind.
            ring.gate = crate::ring_head_gate(true, ring.consumer.rank());
            let mut s = lock_state(state);
            // ONLY the FIRST outcome settles the arm — see `attach_decided`. A
            // re-attach an hour into a run does not retroactively make this
            // reader a late one.
            if !ring.attach_decided {
                ring.attach_decided = true;
                if let Some(arm) = s.attach.iter_mut().find(|(n, _)| *n == ring.name) {
                    arm.1 = TraceAttach::AtLive {
                        first_step_recorded: None,
                        discarded: 0,
                    };
                }
            }
            if let Some(last_step_before) = ring.last_step {
                // A hole, because this reader had already admitted records: the
                // stretch between them and whatever comes next is gone.
                s.note_gap(
                    &ring.name,
                    TraceGap {
                        last_step_before,
                        first_step_after: None,
                    },
                );
                ring.gap_open = true;
                drop(s);
                tracing::warn!(
                    ring = %ring.name,
                    records_lost,
                    last_step_before,
                    "flashback: this recorder was LAPPED on a scheduler-trace ring and \
                     re-attached at the live cursor — a capture carrying records on both sides \
                     of the hole is refused, later captures are not"
                );
            } else {
                drop(s);
                // Not a hole: this reader had read nothing, so there is no near
                // side to lose. The ring had simply already lapped before the
                // recorder reached it, which is a late ATTACH and is reported as
                // one.
                tracing::info!(
                    ring = %ring.name,
                    records_lost,
                    "flashback: this run's trace ring had already lapped when the \
                     recorder first read it — attached at the live cursor instead of record 0"
                );
            }
        }
        Err(e) => {
            // The re-open is the only recovery, so a failure ends this ring.
            // Reported through the retire path on the next pass rather than
            // silently, by leaving the consumer in place: the next `drain_gated`
            // returns the same overrun and the re-open is retried, which is
            // right for a transient failure and bounded by the run.
            tracing::warn!(
                ring = %ring.name,
                error = %e,
                "flashback: could not re-attach to a lapped scheduler-trace ring; \
                 retrying"
            );
        }
    }
}

/// A copy of the published state, taken under the lock.
///
/// A FREE function rather than a `Recorder` method, and that is a borrow-checker
/// fact rather than a style one: the capture-close path reads this while it holds
/// `&mut` on the flashback plane, and a `&self` method would borrow the whole
/// recorder. Reading the field directly keeps the two borrows disjoint, which is
/// what lets the verdict and the manifest be built from ONE reading of the drain
/// instead of two that could disagree.
pub(crate) fn snapshot(state: &SharedTraceDrain) -> TraceDrainState {
    lock_state(state).clone()
}

/// Lock the published state, recovering a poisoned mutex.
///
/// A drain thread that panicked mid-update has already been reported by
/// [`TraceDrainHandle::drop`]; refusing to read the counters afterwards would
/// cost the operator the accounting as well as the records, and the state is
/// plain counters with no invariant a partial write can break.
fn lock_state(state: &SharedTraceDrain) -> std::sync::MutexGuard<'_, TraceDrainState> {
    state.lock().unwrap_or_else(|e| e.into_inner())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn gap(before: u64, after: Option<u64>) -> TraceGap {
        TraceGap {
            last_step_before: before,
            first_step_after: after,
        }
    }

    /// The holes recorded for one ring, oldest first.
    fn holes(s: &TraceDrainState, ring: &str) -> Vec<TraceGap> {
        s.gaps
            .get(ring)
            .map(|e| e.holes.iter().copied().collect())
            .unwrap_or_default()
    }

    /// The single ring most arms use — they are about the hole RULES, not about
    /// which ring a hole came from.
    const R: &str = "/cer_rg_a";

    /// **The two-ring oracle.** Two rings that lap before
    /// either resumes keep SEPARATE holes, and a capture straddling EITHER is
    /// refused.
    ///
    /// The shape a shared list gets wrong, step for step: A laps, B laps
    /// (while A's hole is still open), A admits, B admits. On one shared list B's
    /// hole was COLLAPSED into A's at the second `note_gap` — the collapse rule
    /// is sound per ring and reaches across rings on a shared list — and then
    /// never closed, because A's admit closed the back with A's step. B's lost
    /// interval ended up in no list at all, so `gap_spanning` could not see it
    /// and a capture holding records on both sides of it was ACCEPTED.
    ///
    /// Three claims, and the third is the anti-tautology half: without it a
    /// "refuse everything" implementation passes the first two.
    #[test]
    fn two_rings_that_lap_before_either_resumes_keep_separate_holes() {
        const A: &str = "/cer_rg_a";
        const B: &str = "/cer_rg_b";
        let mut s = TraceDrainState::default();

        // A laps at step 10, B laps at step 20 — B's while A's hole is OPEN.
        s.note_gap(A, gap(10, None));
        s.note_gap(B, gap(20, None));
        // Each ring then resumes, far apart, so the two holes cannot be mistaken
        // for one another by their edges alone.
        s.close_gap(A, 100);
        s.close_gap(B, 200);

        // (1) THE VERDICT, asserted FIRST because it is the behaviour that
        // matters and the one the shared list got wrong: a capture straddling
        // B's lost interval is REFUSED, naming B's window. On a shared list B's
        // hole was collapsed into A's and never closed, so the only hole left was
        // A's [10, 100] — which does not span [20, 200] — and this answered
        // `None`, i.e. the capture was ACCEPTED.
        assert_eq!(
            s.gap_spanning(Some(20), Some(200)),
            Some(gap(20, Some(200))),
            "a capture holding records either side of B's hole must be refused \
             naming B's window — `None` here is B's hole having vanished"
        );
        // …and one straddling A's is refused naming A's.
        assert_eq!(
            s.gap_spanning(Some(10), Some(100)),
            Some(gap(10, Some(100))),
            "A's hole is still found too"
        );

        // (2) BOTH holes are enumerable, each with its own edges — the
        // supporting detail behind the verdict above.
        assert_eq!(holes(&s, A), vec![gap(10, Some(100))], "A's hole");
        assert_eq!(holes(&s, B), vec![gap(20, Some(200))], "B's hole");
        assert_eq!(s.laps, 2, "both laps counted");

        // (3) ANTI-TAUTOLOGY: a capture clear of BOTH holes is ACCEPTED — its
        // window begins after every far edge, so it straddles neither.
        assert_eq!(
            s.gap_spanning(Some(201), Some(400)),
            None,
            "a capture whose window begins after every hole carries none of them"
        );
    }

    /// **The bounds oracle.** A hole is
    /// judged against the bounds of every record the capture CARRIES — over all
    /// rings — never against the owning ring's own thinner bounds.
    ///
    /// The scenario, step for step: ring A laps between step 10
    /// and step 100, and the capture carries an A record at step 0 and a B
    /// record at step 200. Ring-locally A's bounds are `[0, 0]`, which straddle
    /// nothing; capture-wide they are `[0, 200]`, which straddle A's hole — and
    /// that is the correct answer, because the capture's window really does cover
    /// steps 10-100 (the rings step in LOCKSTEP, so a step number is the same
    /// instant on every one of them) and ring A's records for those steps were
    /// provably destroyed. Judging ring-locally would stamp `resimmable` on a
    /// trace known to be missing in-window records. See `gap_spanning_carried`.
    ///
    /// Driven through `gap_spanning_carried` with real records rather than
    /// through the bounds pair, so the BOUNDS CHOICE is what is under test: a
    /// per-ring implementation lives inside that one function and this arm is
    /// what fails it.
    ///
    /// Both directions, and the second is the anti-tautology half: a capture
    /// whose records all sit after the re-attach carries only the far side and
    /// must be ACCEPTED, so the arm cannot pass on a "refuse anything with a
    /// hole anywhere" implementation.
    #[test]
    fn a_hole_is_judged_against_the_capture_wide_bounds_not_its_own_rings() {
        // Only A owns a hole; B is here as the ring that supplies the far
        // BOUND, which is the whole point of the scenario, so it needs no key.
        const A: &str = "/cer_rg_a";
        let mut s = TraceDrainState::default();
        // A lapped between step 10 and step 100. B never lapped.
        s.note_gap(A, gap(10, None));
        s.close_gap(A, 100);

        // THE SCENARIO: one A record before the hole, one B record long after
        // it, and NOTHING from A on the far side.
        let carried = [rec_at(0, 0), rec_at(200, 1)];
        assert_eq!(
            s.gap_spanning_carried(&carried),
            Some(gap(10, Some(100))),
            "the capture's window covers steps 10-100 and ring A's records for them were \
             DESTROYED — `None` here is a trace known to be incomplete being stamped resimmable"
        );

        // ANTI-TAUTOLOGY: the same rings, a capture that begins after the
        // re-attach. It holds only the far side, so it straddles nothing.
        let after = [rec_at(150, 0), rec_at(200, 1)];
        assert_eq!(
            s.gap_spanning_carried(&after),
            None,
            "a capture whose window begins after the recorder re-attached is CONTINUOUS — \
             refusing it would condemn a whole run for one stall (project rule)"
        );

        // …and a capture that carries nothing straddles nothing either — the
        // empty-bounds arm, which `NoTrace` owns instead.
        assert_eq!(s.gap_spanning_carried(&[]), None, "no records, no straddle");
    }

    /// A record at `step`, stamped with `rank` the way the drain stamps one —
    /// the ring identity a per-ring bounds implementation would group on.
    fn rec_at(step: u64, rank: u32) -> TraceRingRecord {
        TraceRingRecord {
            reserved: rank,
            ..boundary_rec(step)
        }
    }

    /// **The tracking ceiling.** Past the tracking ceiling a ring becomes
    /// UNPROVABLE, and a capture straddling an OLD hole is STILL refused.
    ///
    /// Driven in the PRODUCTION shape, which is what makes it bite: `note_gap` is
    /// always handed an OPEN hole (`first_step_after: None`) and `close_gap`
    /// closes it later. A ceiling branch that replaced the whole list with
    /// ONE entry carrying the new hole's edges would leave that entry UNCLOSED, so
    /// `spans` would answer false for everything and every one of the 64 closed holes
    /// would become invisible until the next record arrived. A capture straddling any
    /// of them would be ACCEPTED.
    ///
    /// The pin is the OLDEST hole, deliberately: the newest is easy to keep and
    /// was never the one at risk.
    #[test]
    fn past_the_ceiling_a_ring_is_unprovable_and_still_refuses_an_old_hole() {
        let mut s = TraceDrainState::default();
        // 64 holes, each opened and then CLOSED — the shipping sequence.
        for i in 0..MAX_TRACKED_GAPS as u64 {
            s.note_gap(R, gap(i * 10, None));
            s.close_gap(R, i * 10 + 1);
        }
        assert_eq!(holes(&s, R).len(), MAX_TRACKED_GAPS, "the list is full");
        assert!(
            !s.gaps[R].overflowed,
            "precondition: nothing has been forgotten yet"
        );
        // A capture straddling the OLDEST hole [0, 1] is refused today.
        assert_eq!(
            s.gap_spanning(Some(0), Some(5)),
            Some(gap(0, Some(1))),
            "precondition: the oldest hole refuses, by its own edges"
        );

        // THE 65th LAP — open, exactly as the drain reports one.
        s.note_gap(R, gap(9_000, None));

        // THE PIN. The oldest hole has left the list, so it can no longer be
        // quoted exactly — but the ring is marked UNPROVABLE and the capture is
        // STILL REFUSED. A merged, unclosed entry would answer `None` here,
        // because an unclosed hole spans nothing.
        let verdict = s.gap_spanning(Some(0), Some(5));
        assert!(
            verdict.is_some(),
            "a capture straddling an OLD hole must still be refused past the \
             ceiling — `None` here is forgotten evidence being read as continuity"
        );
        assert!(s.gaps[R].overflowed, "the ring is marked unprovable");
        assert_eq!(
            s.laps,
            MAX_TRACKED_GAPS as u64 + 1,
            "the COUNT is unconditional"
        );
        // The quoted hole is a REAL one with real edges, not a sentinel.
        let quoted = verdict.expect("a hole");
        assert!(
            quoted.first_step_after.is_some(),
            "a refusal quotes a closed hole an operator can act on: {quoted:?}"
        );

        // A ring that has NOT overflowed is unaffected — the bit is per ring.
        s.note_gap("/cer_rg_quiet", gap(7, None));
        s.close_gap("/cer_rg_quiet", 8);
        assert!(!s.gaps["/cer_rg_quiet"].overflowed);
    }

    /// The ceiling costs a run its later captures on that ring, and that cost is
    /// STATED rather than discovered.
    ///
    /// The anti-tautology half of the arm above, inverted deliberately: past the
    /// ceiling the refusal really is total for captures carrying records, so this
    /// arm asserts the cost instead of pretending there is none. What it still
    /// pins is that an EMPTY capture is not refused here — it straddles nothing
    /// by construction, and `ResimGap::NoTrace` is the arm that owns an empty
    /// trace.
    #[test]
    fn past_the_ceiling_the_refusal_is_total_for_captures_that_carry_records() {
        let mut s = TraceDrainState::default();
        for i in 0..=MAX_TRACKED_GAPS as u64 {
            s.note_gap(R, gap(i * 10, None));
            s.close_gap(R, i * 10 + 1);
        }
        assert!(s.gaps[R].overflowed);
        // A window nowhere near any hole is refused too — the stated cost.
        assert!(
            s.gap_spanning(Some(90_000), Some(90_001)).is_some(),
            "past the ceiling continuity is unprovable for ANY carried range"
        );
        // …but an empty trace is not refused HERE.
        assert_eq!(
            s.gap_spanning(None, None),
            None,
            "an empty trace straddles nothing"
        );
    }

    /// One ring's resume never closes ANOTHER ring's hole.
    ///
    /// The narrow half of the arm above, isolated: a step off A's drain is not
    /// evidence that anything crossed B's hole. Fabricating a far edge for B here
    /// would make B's hole spannable at a number no record supports, AND would
    /// leave the real crossing with nothing to close.
    #[test]
    fn a_resume_on_one_ring_does_not_close_another_rings_hole() {
        const A: &str = "/cer_rg_a";
        const B: &str = "/cer_rg_b";
        let mut s = TraceDrainState::default();
        s.note_gap(A, gap(10, None));
        s.note_gap(B, gap(20, None));
        s.close_gap(A, 100);
        assert_eq!(holes(&s, A), vec![gap(10, Some(100))], "A closed");
        assert_eq!(
            holes(&s, B),
            vec![gap(20, None)],
            "B's hole is UNTOUCHED — nothing has crossed it"
        );
        // …and an unclosed hole still spans nothing, so B refuses no capture yet.
        assert_eq!(
            s.gap_spanning(Some(0), Some(1_000)),
            Some(gap(10, Some(100))),
            "only A's closed hole can refuse anything"
        );
    }

    /// `close_gap` on a ring with no holes is a NO-OP, and invents no entry.
    ///
    /// The drain calls it only for a ring it holds, so an unknown name is a
    /// caller bug — and answering it by creating an empty hole list would put a
    /// ring into the enumeration that never lapped.
    #[test]
    fn closing_an_unknown_rings_hole_is_a_no_op() {
        let mut s = TraceDrainState::default();
        s.note_gap(R, gap(40, None));
        s.close_gap("/cer_rg_nobody", 99);
        assert_eq!(holes(&s, R), vec![gap(40, None)], "untouched");
        assert!(!s.gaps.contains_key("/cer_rg_nobody"), "no entry invented");
    }

    /// A hole opens with an UNKNOWN far edge and is closed by the first record
    /// that crosses it — and an unclosed one is closed exactly once.
    #[test]
    fn a_hole_opens_unclosed_and_the_first_record_across_it_closes_it() {
        let mut s = TraceDrainState::default();
        s.note_gap(R, gap(40, None));
        assert_eq!(holes(&s, R).last().copied(), Some(gap(40, None)));
        assert_eq!(s.laps, 1);
        s.close_gap(R, 97);
        assert_eq!(holes(&s, R).last().copied(), Some(gap(40, Some(97))));
        // A second crossing does not move the edge: the far side is the FIRST
        // step that arrived, not the latest one.
        s.close_gap(R, 120);
        assert_eq!(holes(&s, R).last().copied(), Some(gap(40, Some(97))));
    }

    /// An UNCLOSED hole spans nothing, however wide the capture is.
    ///
    /// Not a degenerate case: it is the state every hole is in for at least one
    /// drain pass, and a capture taken in that window must not be refused for a
    /// far side no record has yet reached.
    #[test]
    fn an_unclosed_hole_spans_nothing() {
        let g = gap(40, None);
        assert!(!g.spans(Some(0), Some(1_000_000)));
    }

    /// The span test is about the records a capture CARRIES, on both sides.
    #[test]
    fn a_hole_is_spanned_only_by_a_capture_that_carries_both_sides() {
        let g = gap(40, Some(97));
        // Both sides: refused.
        assert!(g.spans(Some(38), Some(99)));
        // Exactly the edges, which are both real records: still both sides.
        assert!(g.spans(Some(40), Some(97)));
        // Only the near side — the capture ends before anything crossed.
        assert!(!g.spans(Some(38), Some(40)));
        // Only the far side — the window begins after the re-attach. THE case
        // the no-retirement rule turns on: one stall must not condemn a whole run.
        assert!(!g.spans(Some(97), Some(220)));
        // Empty trace.
        assert!(!g.spans(None, None));
    }

    /// A second lap with nothing admitted since the first does NOT open a second
    /// hole — it would leave the first permanently unclosed, and an unclosed hole
    /// refuses nothing.
    ///
    /// The bug this pins is quiet in exactly the wrong direction: the list still
    /// grows, `laps` still climbs, and the capture that carries the hole is
    /// ACCEPTED. Reachable on the ordinary burst shape, where the drain is lapped
    /// again before it has read a single record off its re-attach.
    #[test]
    fn a_second_lap_with_nothing_admitted_between_extends_the_open_hole() {
        let mut s = TraceDrainState::default();
        s.note_gap(R, gap(12, None));
        s.note_gap(R, gap(12, None));
        s.note_gap(R, gap(12, None));
        assert_eq!(s.laps, 3, "every lap is counted, unconditionally");
        assert_eq!(holes(&s, R).len(), 1, "…but they are ONE hole");
        // The far side closes the hole that is really there…
        s.close_gap(R, 900);
        assert_eq!(holes(&s, R).last().copied(), Some(gap(12, Some(900))));
        // …and a capture straddling it is refused, which is what a second,
        // permanently-unclosed hole would have prevented.
        assert!(s.gap_spanning(Some(11), Some(901)).is_some());
        // A LATER lap, after records really were admitted, is its own hole.
        s.note_gap(R, gap(950, None));
        assert_eq!(holes(&s, R).len(), 2);
    }

    /// One `AtLive` ring makes the recorder's verdict `AtLive`, and the reported
    /// first complete step is the LEAST-covered ring's.
    #[test]
    fn the_attach_verdict_is_at_live_if_any_ring_is_and_reports_the_least_covered() {
        let mut s = TraceDrainState::default();
        assert_eq!(s.attach_verdict(), None, "no ring, no claim");
        s.attach.push(("a".into(), TraceAttach::FromStart));
        assert_eq!(s.attach_verdict(), Some(TraceAttach::FromStart));
        s.attach.push((
            "b".into(),
            TraceAttach::AtLive {
                first_step_recorded: Some(400),
                discarded: 7,
            },
        ));
        s.attach.push((
            "c".into(),
            TraceAttach::AtLive {
                first_step_recorded: Some(900),
                discarded: 3,
            },
        ));
        assert_eq!(
            s.attach_verdict(),
            Some(TraceAttach::AtLive {
                first_step_recorded: Some(900),
                discarded: 10,
            }),
            "the trace reaches back only as far as its least-covered ring"
        );
    }

    /// A lap is a HOLE, never a retirement: the drain re-opens at the live cursor
    /// and KEEPS READING.
    ///
    /// Over a real POSIX-SHM ring, because the property is about a producer
    /// outrunning a reader and no pure state machine has a producer. The pin is
    /// the LAST wait: a drain that retired the ring on overrun (the
    /// `harvest_anchors` rule this module rejects) satisfies every earlier
    /// assertion here and then reads nothing for the rest of the run — which is
    /// exactly the frames-only outcome the module doc forbids.
    #[test]
    fn a_lapped_drain_re_opens_and_keeps_reading() {
        use cerulion_core::trace_ring::TraceRingOwner;

        let tag = format!(
            "drlap{}{}",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .map(|d| d.as_nanos())
                .unwrap_or(0)
        );
        let mut owner = TraceRingOwner::create(&tag, 16, 0, &["a"]).expect("ring");
        let name = owner.name().to_string();
        let mut producer = owner.producer().expect("the single producer");
        let consumer = TraceRingConsumer::open(&name).expect("open at zero");
        let trace: SharedTraceWindow = Arc::new(Mutex::new(crate::trace_window::TraceWindow::new(
            30_000_000_000,
            64 * 1024 * 1024,
        )));
        let handle = spawn(
            vec![consumer],
            vec![HeadStepGate::passthrough()],
            RingCursorOrigin::RecordZero,
            Arc::clone(&trace),
            Instant::now(),
        );
        let state = handle.state();

        // PHASE 1 — the near side, comfortably inside the ring.
        for step in 11..=12u64 {
            producer.push(&boundary_rec(step));
            producer.push(&fire_rec(step));
        }
        assert!(
            wait_for(&state, |s| s.records_admitted >= 4),
            "the drain must read the near side: {:?}",
            lock_state(&state)
        );

        // PHASE 2 — outrun it. A burst into a 16-record ring, repeated until the
        // drain REPORTS the hole.
        let mut burst = 100u64;
        assert!(
            wait_for(&state, |s| {
                for _ in 0..64 {
                    producer.push(&boundary_rec(burst));
                    burst += 1;
                }
                s.laps >= 1
            }),
            "a burst into a 16-record ring must lap the reader: {:?}",
            lock_state(&state)
        );
        let admitted_at_lap = lock_state(&state).records_admitted;

        // PHASE 3 — THE PIN. Records pushed after the burst must still be read.
        //
        // Pushed IN THE WAIT, one step per poll, rather than once up front: when
        // the burst loop exits there may still be an OBSERVED-BUT-UNHANDLED
        // overrun in flight, and the re-attach that handles it sets the cursor to
        // the write cursor AT THAT INSTANT — so a far side pushed before it is
        // jumped over and the arm waits forever for records the drain will never
        // see. (MEASURED, and it is the harness racing the code rather than the
        // code failing: `records_admitted` froze at the near side's 4 with
        // `laps: 2`, `rings_retired: []`, and the primitive itself re-attaches
        // and reads a tail correctly with no thread in sight.)
        //
        // Two records per 5 ms poll into a 16-record ring cannot lap a drain on
        // the same cadence, so this loop cannot cause the very condition it is
        // recovering from; and a drain that RETIRED its ring admits nothing here,
        // which is exactly what this arm exists to catch.
        let mut far = burst + 1_000;
        assert!(
            wait_for(&state, |s| {
                // A boundary FIRST — a re-attach re-arms the head-step gate.
                producer.push(&boundary_rec(far));
                producer.push(&fire_rec(far));
                far += 1;
                s.records_admitted > admitted_at_lap
            }),
            "a lapped drain RE-OPENS and keeps reading — a retirement would stop here: {:?}",
            lock_state(&state)
        );

        // …and the hole it recorded is CLOSED by the far side, with a near edge
        // it really admitted.
        let s = lock_state(&state);
        let recorded = s
            .gaps
            .get(&name)
            .and_then(|e| e.holes.back().copied())
            .expect("a hole was recorded for this ring");
        assert!(
            recorded.first_step_after.is_some(),
            "the hole is closed: {recorded:?}"
        );
        assert!(
            recorded.last_step_before >= 12,
            "the near edge is a step the drain admitted: {recorded:?}"
        );
        drop(s);
        drop(handle);
    }

    /// **The departure-gate oracle.** A lapped DEPARTURE
    /// ring re-attaches with the gate its RANK earns (PASSTHROUGH), so its
    /// records keep reaching the retention.
    ///
    /// A departure ring carries no `STEP_BOUNDARY` record at all (it is the
    /// supervisor's stream of `rank == u32::MAX` fault records), which is
    /// why `ring_head_gate` hands it a passthrough gate even on a mid-run attach
    /// — an ARMED gate over such a ring discards **everything, forever**, as
    /// `HeadStepGate`'s own doc says. `reattach` re-armed one unconditionally, so
    /// ONE lap silently ended a run's fault evidence: the drain kept reading, the
    /// records kept being discarded, and every later capture carried no
    /// departure at all while the manifest reported a healthy, growing
    /// `trace_ring_records` from whatever else was on the ring.
    ///
    /// Over a real POSIX-SHM ring, because the property is a producer outrunning
    /// a reader. THE PIN is phase 3: with an armed `HeadStepGate` the
    /// admitted count freezes at the near side's and this wait times out.
    #[test]
    fn a_lapped_departure_ring_re_attaches_with_a_passthrough_gate() {
        use cerulion_core::trace_ring::{TraceRingOwner, DEPARTURE_RING_RANK};

        let tag = format!(
            "drdep{}{}",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .map(|d| d.as_nanos())
                .unwrap_or(0)
        );
        // The departure-ring shape: the sentinel rank, and NO node table. A
        // departure record's `node_idx` is the departed process rank, not an
        // index.
        let mut owner = TraceRingOwner::create(&tag, 16, DEPARTURE_RING_RANK, &[]).expect("ring");
        let name = owner.name().to_string();
        let mut producer = owner.producer().expect("the single producer");
        let consumer = TraceRingConsumer::open(&name).expect("open at zero");
        let trace: SharedTraceWindow = Arc::new(Mutex::new(crate::trace_window::TraceWindow::new(
            30_000_000_000,
            64 * 1024 * 1024,
        )));
        let handle = spawn(
            vec![consumer],
            // The gate the OPENER gives this ring — `ring_head_gate(_, sentinel)`
            // — so the arm starts from the shipping pairing rather than a
            // hand-picked one.
            vec![crate::ring_head_gate(false, DEPARTURE_RING_RANK)],
            RingCursorOrigin::RecordZero,
            Arc::clone(&trace),
            Instant::now(),
        );
        let state = handle.state();

        // PHASE 1 — the near side: departures, comfortably inside the ring.
        for rank in 1..=2u32 {
            producer.push(&departure_rec(rank));
        }
        assert!(
            wait_for(&state, |s| s.records_admitted >= 2),
            "the drain must read the near side of a departure ring: {:?}",
            lock_state(&state)
        );

        // PHASE 2 — outrun it, exactly as the sibling lap arm does.
        assert!(
            wait_for(&state, |s| {
                for rank in 0..64u32 {
                    producer.push(&departure_rec(100 + rank));
                }
                s.laps >= 1
            }),
            "a burst into a 16-record ring must lap the reader: {:?}",
            lock_state(&state)
        );
        let admitted_at_lap = lock_state(&state).records_admitted;

        // PHASE 3 — THE PIN. Departures pushed after the lap must STILL be
        // admitted. Pushed inside the wait for the reason the sibling arm gives
        // (an overrun observed but not yet handled jumps the cursor to the write
        // cursor of that instant).
        let mut far = 1_000u32;
        assert!(
            wait_for(&state, |s| {
                producer.push(&departure_rec(far));
                far += 1;
                s.records_admitted > admitted_at_lap
            }),
            "a lapped DEPARTURE ring must keep admitting records — an armed gate \
             over a ring that carries no step boundary discards everything \
             forever: {:?}",
            lock_state(&state)
        );

        // …and what reached the RETENTION really is a departure record, so the
        // arm cannot pass on some other record type slipping through.
        //
        // CONDITION-POLLED rather than read once: `admit` bumps
        // `records_admitted` under the STATE lock and takes the retention lock
        // LAST ("held for one push"), so the wait above can return while the
        // pass's records are still BETWEEN the two locks — and a loaded runner
        // parks the drain thread in that gap for long enough for a one-shot
        // read here to see the counter ahead of the window (the macOS shard-0
        // flake: `held.len() == admitted_at_lap` exactly). Load can only DELAY
        // the second lock, never change what reaches the retention, so the
        // poll is bounded like every other wait in this arm and the verdict
        // below is byte-unchanged — a drain that admits on the counter but
        // never pushes to the retention still exhausts the deadline and fails.
        let deadline = Instant::now() + Duration::from_secs(20);
        let held = loop {
            let held = lock_trace(&trace)
                .records_from(0)
                .copied()
                .collect::<Vec<_>>();
            if held.len() as u64 > admitted_at_lap || Instant::now() >= deadline {
                break held;
            }
            std::thread::sleep(Duration::from_millis(5));
        };
        assert!(
            held.iter()
                .all(|r| r.record_type == cerulion_core::trace_ring::RECORD_TYPE_DEPARTURE)
                && held.len() as u64 > admitted_at_lap,
            "the retention holds the departures the drain admitted: {} record(s) held, \
             admitted_at_lap={admitted_at_lap}",
            held.len()
        );
        drop(handle);
    }

    fn departure_rec(departed_rank: u32) -> TraceRingRecord {
        TraceRingRecord {
            step: 0,
            fire_time_ns: 0,
            duration_ns: 0,
            // A departure record's `node_idx` IS the departed process rank.
            node_idx: departed_rank,
            global_level: 0,
            record_type: cerulion_core::trace_ring::RECORD_TYPE_DEPARTURE,
            reserved: 0,
        }
    }

    fn boundary_rec(step: u64) -> TraceRingRecord {
        TraceRingRecord {
            step,
            fire_time_ns: 1_000_000 + step * 4_000_000,
            duration_ns: 0,
            node_idx: 0,
            global_level: 0,
            record_type: cerulion_core::trace_ring::RECORD_TYPE_STEP_BOUNDARY,
            reserved: 0,
        }
    }

    fn fire_rec(step: u64) -> TraceRingRecord {
        TraceRingRecord {
            step,
            fire_time_ns: 1_000_000 + step * 4_000_000,
            duration_ns: 700,
            node_idx: 0,
            global_level: 0,
            record_type: cerulion_core::trace_ring::RECORD_TYPE_FIRE,
            reserved: 0,
        }
    }

    fn wait_for(state: &SharedTraceDrain, mut cond: impl FnMut(&TraceDrainState) -> bool) -> bool {
        let deadline = Instant::now() + Duration::from_secs(20);
        while Instant::now() < deadline {
            if cond(&lock_state(state)) {
                return true;
            }
            std::thread::sleep(Duration::from_millis(5));
        }
        false
    }

    /// **The attach arm's seed.** The
    /// attach arm is seeded from the CURSOR ORIGIN — not assumed, and not
    /// inferred from the gate.
    ///
    /// Seeding every ring `FromStart` would make a mid-run `--ring`
    /// attach claim it had read from record 0 before its first drain — a
    /// positive, checkable claim that is simply false, and one a capture and
    /// `/bagd/status` both publish. Reading `gate.is_open()` instead
    /// is right for every ring the gate can speak for and WRONG for a DEPARTURE
    /// ring: `ring_head_gate` hands that ring a PASSTHROUGH gate by design
    /// even on a mid-run attach, so gate-inference reported
    /// `from_start` for a reader opened at the LIVE cursor. Threading the
    /// opener's own answer through is right on every cell.
    ///
    /// **The oracle is the 2×2**, because only the full grid separates the two
    /// implementations: gate-inference agrees with the truth on three cells and
    /// differs on exactly one — the mid-run DEPARTURE ring — so a test that
    /// omitted it would pass either way. `first_step_recorded`/`discarded` are
    /// asserted on that cell too: a departure ring has no `STEP_BOUNDARY` by
    /// construction, so `None`/0 is the correct payload, and anything else would
    /// be a number about a step that does not exist.
    ///
    /// Over a REAL ring, because the seed is read off a real `HeadStepGate` and
    /// a hand-built state would not exercise `spawn` at all. All four cells in
    /// one body, so none can pass by accident.
    #[test]
    fn the_attach_arm_is_seeded_from_the_cursor_origin_not_the_gate() {
        use cerulion_core::trace_ring::TraceRingOwner;
        // The crate's OWN sentinel — the one `ring_head_gate` compares against.
        use crate::DEPARTURE_RING_RANK;

        // (ring rank, cursor origin, gate armed, expect AtLive)
        //
        // The gate column is what `ring_head_gate(attached_mid_run, rank)`
        // returns for that cell — armed ONLY for a mid-run WORKER ring — and the
        // body asserts that rather than trusting the table.
        let grid: [(u32, RingCursorOrigin, bool, bool); 4] = [
            // A from-zero attach: record 0 really is the first record read.
            (0, RingCursorOrigin::RecordZero, false, false),
            (
                DEPARTURE_RING_RANK,
                RingCursorOrigin::RecordZero,
                false,
                false,
            ),
            // A mid-run attach: the cursor was placed at the live write cursor.
            (0, RingCursorOrigin::Live, true, true),
            // THE CELL THE GATE CANNOT ANSWER FOR — passthrough deliberately,
            // opened at live all the same.
            (DEPARTURE_RING_RANK, RingCursorOrigin::Live, false, true),
        ];

        for (rank, cursor_origin, armed, want_live) in grid {
            let mid_run = matches!(cursor_origin, RingCursorOrigin::Live);
            let tag = format!(
                "seed{}{}{}{}",
                rank,
                u8::from(armed),
                std::process::id(),
                std::time::SystemTime::now()
                    .duration_since(std::time::UNIX_EPOCH)
                    .map(|d| d.as_nanos())
                    .unwrap_or(0)
            );
            let owner = TraceRingOwner::create(&tag, 16, rank, &["a"]).expect("ring");
            let name = owner.name().to_string();
            let consumer = TraceRingConsumer::open(&name).expect("open");
            // The gate the recorder really would pair with this ring — asserted
            // rather than assumed, so the grid cannot describe a pairing
            // `ring_head_gate` does not make.
            assert_eq!(
                crate::ring_head_gate(mid_run, rank).is_open(),
                !armed,
                "rank={rank} mid_run={mid_run}: the grid must use the gate the \
                 opener would actually pair with this ring"
            );
            let gate = if armed {
                HeadStepGate::armed()
            } else {
                HeadStepGate::passthrough()
            };
            let trace: SharedTraceWindow = Arc::new(Mutex::new(
                crate::trace_window::TraceWindow::new(30_000_000_000, 64 * 1024 * 1024),
            ));
            let mut handle = spawn(
                vec![consumer],
                vec![gate],
                cursor_origin,
                trace,
                Instant::now(),
            );
            // Read the SEED before the thread can decide anything: nothing has
            // been pushed, so no drain outcome can have moved it.
            let arm = lock_state(&handle.state())
                .attach
                .first()
                .map(|(_, a)| *a)
                .expect("one ring, one arm");
            let is_live = matches!(arm, TraceAttach::AtLive { .. });
            assert_eq!(
                is_live, want_live,
                "rank={rank} origin={cursor_origin:?}: the seed must report where \
                 the OPENER placed the cursor. A ring opened at the live cursor is \
                 AtLive whichever gate it was given; only a record-0 open is \
                 FromStart. Got {arm:?}"
            );
            if rank == DEPARTURE_RING_RANK && want_live {
                assert_eq!(
                    arm,
                    TraceAttach::AtLive {
                        first_step_recorded: None,
                        discarded: 0,
                    },
                    "a departure ring carries no STEP_BOUNDARY, so its live seed \
                     names no first step and discards nothing — never a number \
                     about a step it cannot have"
                );
            }
            handle.finish();
        }
    }

    /// A drain that could not SPAWN reports NO attach arm.
    ///
    /// The seeded arms describe a reader. When `Builder::spawn` fails there is
    /// no reader, and leaving them in place would render a nonexistent thread as
    /// `from_start` — the strongest possible claim ("this recorder is reading
    /// from record 0") on the one path where it reads nothing. `None` is the
    /// correct answer, and the manifest already renders it as JSON `null`.
    ///
    /// Driven through the PRODUCTION `spawn` via `fail_next_drain_spawn_for_test`.
    /// An arm that hand-cleared the state instead would not catch the
    /// production `attach.clear()` being deleted — such an arm is testing
    /// its own `Vec::clear`. The seam exists because a genuine `Builder::spawn`
    /// failure needs thread exhaustion.
    #[test]
    fn a_drain_that_could_not_spawn_makes_no_attach_claim() {
        use cerulion_core::trace_ring::TraceRingOwner;

        let tag = format!(
            "nospawn{}{}",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .map(|d| d.as_nanos())
                .unwrap_or(0)
        );
        let owner = TraceRingOwner::create(&tag, 16, 0, &["a"]).expect("ring");
        let consumer = TraceRingConsumer::open(owner.name()).expect("open");
        let trace: SharedTraceWindow = Arc::new(Mutex::new(crate::trace_window::TraceWindow::new(
            30_000_000_000,
            64 * 1024 * 1024,
        )));

        fail_next_drain_spawn_for_test();
        let mut handle = spawn(
            vec![consumer],
            vec![HeadStepGate::passthrough()],
            RingCursorOrigin::RecordZero,
            trace,
            Instant::now(),
        );

        let state = handle.state();
        assert!(
            lock_state(&state).attach.is_empty(),
            "a drain with no thread behind it holds no attach arm: {:?}",
            lock_state(&state).attach
        );
        assert_eq!(
            lock_state(&state).attach_verdict(),
            None,
            "…so the verdict is `None`, which the manifest renders as JSON null \
             — never a `from_start` claim about a reader that does not exist"
        );
        // The seam is one-shot: it must not leak into the next spawn.
        assert!(!spawn_forced_to_fail(), "the fault was consumed");
        handle.finish();
    }

    /// The two arms say DIFFERENT things, and the live one names its own numbers.
    #[test]
    fn the_two_attach_arms_render_distinguishable_sentences() {
        let from_start = TraceAttach::FromStart.render();
        assert!(from_start.starts_with("from_start:"), "{from_start}");
        assert!(from_start.contains("record 0"), "{from_start}");
        let live = TraceAttach::AtLive {
            first_step_recorded: Some(412),
            discarded: 9,
        }
        .render();
        assert!(live.starts_with("at_live:"), "{live}");
        assert!(live.contains("step 412"), "{live}");
        assert!(live.contains("9 leading record(s)"), "{live}");
        assert_ne!(from_start, live);
        // An arm that has seen no boundary says so rather than fabricating a step.
        let blind = TraceAttach::AtLive {
            first_step_recorded: None,
            discarded: 2,
        }
        .render();
        assert!(blind.contains("no step boundary yet"), "{blind}");
    }
}
