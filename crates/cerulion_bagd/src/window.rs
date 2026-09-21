//! The ROLLING FRAME WINDOW — the recorder's own drop-oldest
//! retention, and the thing that makes a Flashback possible at all.
//!
//! # Why this exists
//!
//! bagd's taps have NO back-fill: pre-attach retention is exactly each topic's
//! SHM queue depth (the queue depth IS the loss boundary), which
//! is 0.25–1 s on kHz topics and tens of milliseconds elsewhere. A black box
//! needs 30 s. So the recorder keeps its own copy of the recent past, and a
//! trigger finalizes it into a bag.
//!
//! This copy lives in the recorder's
//! memory, not a second SHM ring. The deciding argument was that a frame enters
//! any retention only by being copied out of the publisher's slot, bagd is the
//! only admissible writer, and a ring is therefore the same design plus an SHM
//! hop written by the process whose crash it nominally insures against.
//!
//! # The rule, and how it differs from `StagedFrames`
//!
//! [`StagedFrames`](crate::StagedFrames) is the DRAIN-side arena: append-only,
//! capped per tap at `RECORDING_TAP_STAGING_MAX_BYTES`, and at that cap the tap
//! **stops draining** — never blocks, never evicts.
//!
//! This is a DIFFERENT object with a DIFFERENT rule: it is finite by PURPOSE, so
//! it drops its oldest content to stay inside its bounds. Those two rules are not
//! in tension, and the distinction below is easy to check:
//!
//! > The never-evict rule applies to a TAP's staging and to an iceoryx2
//! > queue — data that has nowhere else to be. The window's drop-oldest applies
//! > to the recorder's own retention buffer, whose entire purpose is to be finite.
//!
//! Nothing here touches a tap, a queue or a borrow.
//!
//! # Two bounds, and only one of them is a promise
//!
//! The SPAN is the promise ("the last 30 seconds"). The BYTE ceiling is a
//! backstop for a robot whose 30 seconds do not fit in the memory budget as
//! priced — and when it bites during a capture the capture SAYS so
//! ([`EvictionReport::truncated_frames`]) rather than quietly covering less than
//! it claims.
//!
//! # The clock is the RECORDER's, and that is not a detail
//!
//! Every batch is stamped with the recorder's own monotonic reading, never the
//! frame's wire `timestamp_ns`. A wire stamp belongs to the PUBLISHER's clock —
//! on a worker that is a `VirtualClock` starting at zero — so ordering a
//! machine-wide window by it would interleave unrelated number lines and evict
//! whatever happened to carry the smallest integer. This is the same cross-clock
//! rule the liveness observer states for stamp advancement, applied to
//! retention. The wire stamps still reach the bag as each message's publish time;
//! they are simply not what the window is ordered by.

use std::collections::VecDeque;
use std::sync::Arc;

use crate::StagedFrames;

/// One tap's drained batch, held for the window's span.
pub(crate) struct WindowBatch {
    /// Index into the recorder's `taps` — resolved to a topic at capture time.
    ///
    /// An index rather than a `String` because a batch is pushed on every drain
    /// of every tap and the window holds tens of thousands of them; cloning a
    /// topic name per batch is a per-frame allocation the drain path does not
    /// need to pay.
    pub topic_idx: usize,
    /// The RECORDER's monotonic reading when this batch was drained — see the
    /// module docs on why this is not the wire stamp.
    pub taken_at_ns: u64,
    /// The batch itself, MOVED out of the tap's staging (no copy) when nothing
    /// else wanted it, or cloned when a continuous bag is also being written.
    ///
    /// # Why this is an `Arc`
    ///
    /// A capture's close used to hand the writer thread a DEEP CLONE of every
    /// batch at or after its floor — `StagedFrames::clone` copies both the frame
    /// table and the whole payload buffer — so closing a capture memcpy'd the
    /// window. That work sits ON THE DRIVE LOOP, which is the thread that drains
    /// the taps, and each topic's own SHM queue is the loss boundary
    /// (0.25–1 s on a kHz topic): the close therefore cost the robot exactly the
    /// frames the capture exists to preserve, at exactly the moment preservation
    /// matters. Sharing the batch makes the close O(#batches) pointer bumps
    /// instead of O(bytes).
    ///
    /// STATED COST (decision D16(a)): eviction can no longer RECLAIM a
    /// batch an in-flight writer still holds — dropping it from the window drops
    /// one reference, not the allocation. Peak memory is unchanged (the deep copy
    /// held the same bytes twice for the same span); what changes is the
    /// ACCOUNTING, because those bytes stop counting against
    /// [`FrameWindow::max_bytes`] while still being resident. That is what
    /// `writer_held_bytes` on the plane exists to report, rather than leaving it
    /// as an invisible overshoot.
    pub frames: Arc<StagedFrames>,
}

/// What one eviction pass did.
///
/// Returned rather than only counted internally, because the TRUNCATION arm is
/// something a capture must report and a caller cannot infer from the totals.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub(crate) struct EvictionReport {
    /// Frames dropped because they aged past the span. Ordinary; this is the
    /// window working.
    ///
    /// It also carries every [`cap_evicted_frames`](Self::cap_evicted_frames) —
    /// see that field for why the two are a SPLIT rather than two buckets.
    pub aged_frames: u64,
    /// Frames dropped because the BYTE ceiling bit while a capture still wanted
    /// them. Never ordinary: a capture that loses these covers less than it says.
    pub truncated_frames: u64,
    /// Frames the BYTE ceiling took with NO capture active — the standing
    /// cap-bite, counted apart.
    ///
    /// A SUBSET of `aged_frames`, deliberately: those two numbers ship in every
    /// manifest and on the status feed, so re-partitioning them would change what
    /// an existing reader is told about a recording that behaved identically.
    /// This is the number that distinguishes "the window reached its span and is
    /// rolling" (the feature working) from "the byte ceiling is the binding limit
    /// and the promised span is not being held" — which the aged total alone
    /// cannot say, and which is the whole of gap (d).
    pub cap_evicted_frames: u64,
}

impl EvictionReport {
    fn is_empty(&self) -> bool {
        // `cap_evicted_frames` is a subset of `aged_frames`, so it cannot be the
        // only nonzero field — asserted by
        // `a_cap_bite_is_counted_in_both_the_aged_total_and_the_cap_split`.
        self.aged_frames == 0 && self.truncated_frames == 0
    }
}

/// The rolling retention buffer. See the module docs.
pub(crate) struct FrameWindow {
    batches: VecDeque<WindowBatch>,
    bytes: usize,
    frames: u64,
    span_ns: u64,
    max_bytes: usize,
    /// Lifetime totals, never reset (Principle #3).
    aged_frames: u64,
    truncated_frames: u64,
    cap_evicted_frames: u64,
    /// Bytes this window has EVICTED that an in-flight capture
    /// writer still holds a reference to.
    ///
    /// A GAUGE, not a lifetime total — it answers "how much memory is resident
    /// right now that the retention has already stopped counting", which is
    /// exactly the accounting decision D16(a) asks be stated rather
    /// than left invisible. It rises as eviction walks past batches the writer
    /// borrowed and returns to zero when that writer settles.
    ///
    /// EXACT rather than an estimate, and the invariant that makes it exact is
    /// [`FlashbackPlane::writer_busy`](crate::flashback_plane::FlashbackPlane::writer_busy):
    /// at most ONE capture writer runs at a time, and the only clones of a
    /// batch's `Arc` outside this window are the ones `batches_from` hands to
    /// that writer's job. So a popped batch whose reference count is still above
    /// one is held by the writer and by nothing else.
    writer_held_bytes: usize,
}

impl FrameWindow {
    /// A window spanning `span_ns` and holding at most `max_bytes`.
    pub(crate) fn new(span_ns: u64, max_bytes: u64) -> Self {
        Self {
            batches: VecDeque::new(),
            bytes: 0,
            frames: 0,
            span_ns,
            // `usize` on a 32-bit target cannot hold an operator's saturating
            // "no practical ceiling", so the ask is CLAMPED rather than wrapped:
            // a wrap would turn "keep everything" into "keep almost nothing",
            // which is the one direction that silently destroys the feature.
            max_bytes: usize::try_from(max_bytes).unwrap_or(usize::MAX),
            aged_frames: 0,
            truncated_frames: 0,
            cap_evicted_frames: 0,
            writer_held_bytes: 0,
        }
    }

    /// Re-point the byte ceiling.
    ///
    /// The frame window's budget is the REMAINDER after the anchor reserve, and
    /// that reserve is re-derived when the measured generation grows — so this
    /// ceiling genuinely moves during a run, downwards on a robot whose state
    /// turns out to be bigger than its first checkpoint suggested.
    ///
    /// Shrinking it takes effect on the next [`evict`](Self::evict) rather than
    /// here, which is deliberate: eviction is where the window's two rules
    /// already live, and dropping frames from a setter would put a second
    /// eviction path in the file whose whole point is that there is one. The
    /// drive loop evicts every pass, so "next pass" is milliseconds.
    pub(crate) fn set_max_bytes(&mut self, max_bytes: u64) {
        self.max_bytes = usize::try_from(max_bytes).unwrap_or(usize::MAX);
    }

    /// The span in force.
    pub(crate) fn span_ns(&self) -> u64 {
        self.span_ns
    }

    /// Bytes currently held.
    pub(crate) fn bytes(&self) -> usize {
        self.bytes
    }

    /// The byte ceiling in force.
    ///
    /// Reported beside [`bytes`](Self::bytes) on every operator surface, because
    /// "the window holds 300 MB" says nothing on its own: whether that is a
    /// healthy window or one pinned against its ceiling is exactly the pair.
    pub(crate) fn max_bytes(&self) -> usize {
        self.max_bytes
    }

    /// Frames currently held.
    pub(crate) fn frames(&self) -> u64 {
        self.frames
    }

    /// Lifetime frames dropped for age.
    pub(crate) fn aged_frames(&self) -> u64 {
        self.aged_frames
    }

    /// Lifetime frames the BYTE ceiling took with no capture active.
    ///
    /// See [`EvictionReport::cap_evicted_frames`]. LIFETIME and never reset
    /// (Principle #3), so it answers "has the ceiling EVER been the binding
    /// limit on this run" rather than "is it biting right now" — which is what
    /// the standing warn's predicate wants, since a window that healed still had
    /// its span shortened while the load lasted.
    pub(crate) fn cap_evicted_frames(&self) -> u64 {
        self.cap_evicted_frames
    }

    /// Lifetime frames dropped by the byte ceiling while a capture wanted them.
    ///
    /// LIFETIME, and a caller rendering ONE capture's manifest must not print it
    /// unchanged: the counter never resets, so after any capture loses a
    /// frame every LATER capture's manifest would repeat that total and label an
    /// unaffected bag truncated. `FlashbackPlane` snapshots this at
    /// `begin_capture` and reports the DELTA.
    pub(crate) fn truncated_frames(&self) -> u64 {
        self.truncated_frames
    }

    /// The oldest batch's stamp, or `None` when empty.
    pub(crate) fn oldest_ns(&self) -> Option<u64> {
        self.batches.front().map(|b| b.taken_at_ns)
    }

    /// Take a tap's drained batch into the window.
    ///
    /// An EMPTY batch is dropped rather than held: it carries no frames, so
    /// keeping it would only give the window a stamp nothing can be recovered
    /// from and make `oldest_ns` claim coverage the window does not have.
    pub(crate) fn push(&mut self, topic_idx: usize, taken_at_ns: u64, frames: StagedFrames) {
        if frames.is_empty() {
            return;
        }
        self.bytes += frames.byte_len();
        self.frames += frames.len() as u64;
        self.batches.push_back(WindowBatch {
            topic_idx,
            taken_at_ns,
            // Wrapped HERE rather than by the caller: the batch arrives owned
            // (moved out of a window-only tap's staging, or copied out of a
            // `--record` tap's), so the `Arc` costs one allocation per batch and
            // no payload copy at all.
            frames: Arc::new(frames),
        });
    }

    /// Drop what the window no longer has to hold.
    ///
    /// `protect_from_ns` is an ACTIVE CAPTURE's floor: everything at or after it
    /// is exempt from the SPAN rule, because the capture already promised to
    /// carry it. It is deliberately not exempt from the BYTE rule — a byte
    /// ceiling that a capture could suspend is not a ceiling, and an
    /// out-of-memory recorder captures nothing at all.
    pub(crate) fn evict(&mut self, now_ns: u64, protect_from_ns: Option<u64>) -> EvictionReport {
        let mut report = EvictionReport::default();

        // (1) AGE. The horizon is the span, pulled BACK to the capture's floor
        // when one is active — so a capture that started 20 s ago still has its
        // pre-window when it finalizes 15 s later, which a bare rolling horizon
        // would have evicted out from under it.
        let horizon = match protect_from_ns {
            Some(floor) => now_ns.saturating_sub(self.span_ns).min(floor),
            None => now_ns.saturating_sub(self.span_ns),
        };
        while let Some(front) = self.batches.front() {
            if front.taken_at_ns >= horizon {
                break;
            }
            let dropped = self.pop_front_counting();
            report.aged_frames += dropped;
        }

        // (2) BYTES. The backstop. Anything dropped here that a capture wanted
        // is a TRUNCATION and is reported as one.
        while self.bytes > self.max_bytes {
            let wanted_by_capture = self
                .batches
                .front()
                .is_some_and(|b| protect_from_ns.is_some_and(|f| b.taken_at_ns >= f));
            let dropped = self.pop_front_counting();
            if dropped == 0 {
                // The window is empty and still over its ceiling, which means the
                // ceiling is below one batch. Nothing further can be dropped;
                // breaking is what stops this being an infinite loop.
                break;
            }
            if wanted_by_capture {
                report.truncated_frames += dropped;
            } else {
                // BOTH, and the doubling is the point: `aged_frames` keeps the
                // meaning every shipped reader already has, while
                // `cap_evicted_frames` isolates the arm that says the CEILING is
                // binding. Booking this into the cap split ALONE would silently
                // re-partition a number that ships in every capture manifest.
                report.aged_frames += dropped;
                report.cap_evicted_frames += dropped;
            }
        }

        self.aged_frames += report.aged_frames;
        self.truncated_frames += report.truncated_frames;
        self.cap_evicted_frames += report.cap_evicted_frames;
        if !report.is_empty() {
            tracing::trace!(
                aged = report.aged_frames,
                truncated = report.truncated_frames,
                cap_evicted = report.cap_evicted_frames,
                held_bytes = self.bytes,
                held_frames = self.frames,
                "flashback window: evicted"
            );
        }
        report
    }

    /// Drop the oldest batch, returning how many frames went with it.
    fn pop_front_counting(&mut self) -> u64 {
        match self.batches.pop_front() {
            Some(batch) => {
                self.bytes -= batch.frames.byte_len();
                let n = batch.frames.len() as u64;
                self.frames -= n;
                self.note_released(&batch);
                n
            }
            None => 0,
        }
    }

    /// Book a batch leaving the window against the writer-held gauge.
    ///
    /// The reference count is read while this window's own reference is still
    /// alive, so a count above one means somebody ELSE holds it — and by the
    /// one-writer invariant on [`writer_held_bytes`](Self::writer_held_bytes)
    /// that somebody is the in-flight capture job.
    fn note_released(&mut self, batch: &WindowBatch) {
        if Arc::strong_count(&batch.frames) > 1 {
            self.writer_held_bytes = self
                .writer_held_bytes
                .saturating_add(batch.frames.byte_len());
        }
    }

    /// Bytes evicted from this window that an in-flight capture writer still
    /// holds. See the field docs.
    pub(crate) fn writer_held_bytes(&self) -> usize {
        self.writer_held_bytes
    }

    /// The writer settled — its job is dropped, so the bytes it pinned are gone.
    ///
    /// A RESET rather than a decrement, and that is what the one-writer
    /// invariant buys: with at most one capture job alive there is no second
    /// writer whose share would have to survive this. Called from the two places
    /// a writer can settle (`poll_writer` and `join_writer`), so a capture that
    /// finishes, fails or is abandoned all clear the gauge — an abandoned
    /// writer's job is dropped with its plane slot, and leaving the gauge
    /// standing would report memory nothing holds.
    pub(crate) fn note_writer_released(&mut self) {
        self.writer_held_bytes = 0;
    }

    /// Every held batch at or after `from_ns`, oldest first.
    ///
    /// Oldest-first is the order a bag wants: `cerulion_bag` writes messages in
    /// call order and a reader's chunk index is built from it, so handing the
    /// writer the window backwards would produce a technically valid bag that
    /// every consumer renders as time running in reverse.
    pub(crate) fn batches_from(&self, from_ns: u64) -> impl Iterator<Item = &WindowBatch> {
        self.batches
            .iter()
            .filter(move |b| b.taken_at_ns >= from_ns)
    }

    /// The stamp of the OLDEST batch a capture from `from_ns` will carry, or
    /// `None` when it will carry nothing.
    ///
    /// This is a capture's ACHIEVED reach, and it is a different number from the
    /// floor it CLAIMS: the byte pass evicts from the
    /// FRONT, so under load the window's oldest held batch walks forward past the
    /// floor while the floor stays frozen where the trigger put it.
    ///
    /// Read off [`batches_from`](Self::batches_from) rather than a second walk,
    /// so the range this reports and the range a capture WRITES cannot disagree.
    /// `None` is a real outcome, not a corner: the byte pass takes ≥ floor frames too
    /// (as truncations), so a capture on an overloaded recorder really can end up
    /// holding nothing of its own window.
    pub(crate) fn first_stamp_from(&self, from_ns: u64) -> Option<u64> {
        self.batches_from(from_ns).next().map(|b| b.taken_at_ns)
    }

    /// How many of `topic_count` taps contribute NO batch at or after `from_ns`.
    ///
    /// An achieved range `[a, b]` reads as coverage, and
    /// for a topic whose period exceeds `b − a` it is not — that topic is simply
    /// absent from the bag. A global range cannot say so, so the COUNT rides
    /// beside it.
    ///
    /// O(#batches), which is why this is a capture-close question and never a hot
    /// status field: the status feed publishes at ~1 Hz over a window that can
    /// hold tens of thousands of batches.
    pub(crate) fn topics_with_no_frames(&self, from_ns: u64, topic_count: usize) -> usize {
        let mut seen = vec![false; topic_count];
        for b in self.batches_from(from_ns) {
            // A batch whose index is past the tap list is not a topic this
            // recorder can name, so it cannot mark one as covered.
            if let Some(slot) = seen.get_mut(b.topic_idx) {
                *slot = true;
            }
        }
        seen.iter().filter(|covered| !**covered).count()
    }

    /// Drop everything. Used when a capture has been written and the recorder is
    /// shutting down — the window has no reason to outlive the drive loop.
    pub(crate) fn clear(&mut self) {
        // Booked against the writer-held gauge exactly as an eviction is: at
        // teardown a capture may still be writing, and a `clear` that dropped
        // its batches silently would report zero resident bytes for memory the
        // writer is still holding — the one direction this gauge must not err in.
        let released: Vec<WindowBatch> = self.batches.drain(..).collect();
        for batch in &released {
            self.note_released(batch);
        }
        self.bytes = 0;
        self.frames = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const MS: u64 = 1_000_000;

    fn batch(n: usize, payload_len: usize) -> StagedFrames {
        let mut frames = StagedFrames::default();
        for i in 0..n {
            // An UNLABELED (single-writer) frame, which is what
            // these window fixtures model.
            frames.push(
                Some((i as u32, 0)),
                crate::producer_labeling::FrameOrigin::Unlabeled,
                &vec![0xAB; payload_len],
            );
        }
        frames
    }

    /// A window built at the shipped span holds its promise and drops what ages
    /// past it — the whole feature, in one hand oracle.
    #[test]
    fn the_window_holds_its_span_and_drops_what_ages_out_of_it() {
        let mut w = FrameWindow::new(30_000 * MS, 1 << 30);
        // Ten batches, one per second.
        for s in 0..10u64 {
            w.push(0, s * 1000 * MS, batch(1, 100));
        }
        assert_eq!(w.frames(), 10);
        assert_eq!(w.bytes(), 1000);

        // At t=9s nothing is 30 s old yet.
        assert_eq!(w.evict(9_000 * MS, None), EvictionReport::default());
        assert_eq!(w.frames(), 10);

        // At t=35s the batches at 0s..=4s are past the horizon (35 − 30 = 5).
        let report = w.evict(35_000 * MS, None);
        assert_eq!(
            report,
            EvictionReport {
                aged_frames: 5,
                truncated_frames: 0,
                cap_evicted_frames: 0,
            }
        );
        assert_eq!(w.frames(), 5);
        assert_eq!(w.oldest_ns(), Some(5_000 * MS));
        // The lifetime counter is unconditional and never reset.
        assert_eq!(w.aged_frames(), 5);
        // The SPAN rule is not the CEILING: this window never came near its byte
        // cap, so the split must stay at zero. Without this the cap counter is
        // satisfied by an implementation that simply mirrors `aged_frames`.
        assert_eq!(w.cap_evicted_frames(), 0);
    }

    /// THE arm the pre-window exists for: a capture that starts now must still
    /// have its 30 s of history when it finalizes 15 s later.
    ///
    /// Without the floor, the ordinary rolling horizon evicts the oldest half of
    /// the pre-window WHILE the capture is recording, and the bag covers
    /// `[T−15, T+15]` instead of `[T−30, T+15]` — the failure is a shorter bag
    /// rather than an error, which is exactly why it needs a test.
    #[test]
    fn an_active_captures_pre_window_is_not_evicted_out_from_under_it() {
        let mut w = FrameWindow::new(30_000 * MS, 1 << 30);
        for s in 0..=30u64 {
            w.push(0, s * 1000 * MS, batch(1, 10));
        }
        assert_eq!(w.frames(), 31);

        // A trigger at t=30s. Its floor is 30 − 30 = 0s.
        let floor = 0u64;

        // Fifteen seconds later the rolling horizon alone would be 45 − 30 = 15s
        // and would have taken half the pre-window. The floor holds it.
        for s in 31..=45u64 {
            w.push(0, s * 1000 * MS, batch(1, 10));
        }
        let report = w.evict(45_000 * MS, Some(floor));
        assert_eq!(
            report,
            EvictionReport::default(),
            "an active capture's floor must exempt its own pre-window from the span rule"
        );
        assert_eq!(
            w.oldest_ns(),
            Some(0),
            "the oldest pre-window batch is still held"
        );
        assert_eq!(w.batches_from(floor).count(), 46);

        // The CONTROL, in the same body: with no capture active the same instant
        // evicts exactly what the span says. Without this the arm above would
        // pass against a window that never evicts at all.
        let report = w.evict(45_000 * MS, None);
        assert_eq!(report.aged_frames, 15);
        assert_eq!(w.oldest_ns(), Some(15_000 * MS));
    }

    /// The byte ceiling is a CEILING: a capture cannot suspend it, and what it
    /// takes from a capture is reported as a truncation rather than folded into
    /// the ordinary aged count.
    #[test]
    fn the_byte_ceiling_bites_through_a_capture_and_says_that_it_did() {
        // Room for exactly 3 batches of 100 bytes.
        let mut w = FrameWindow::new(30_000 * MS, 300);
        for s in 0..3u64 {
            w.push(0, s * 1000 * MS, batch(1, 100));
        }
        assert_eq!(w.evict(3_000 * MS, Some(0)), EvictionReport::default());

        // A fourth batch puts it over. The oldest goes even though a capture
        // wants it — and the report distinguishes that from ageing.
        w.push(0, 3_000 * MS, batch(1, 100));
        let report = w.evict(3_000 * MS, Some(0));
        assert_eq!(
            report,
            EvictionReport {
                aged_frames: 0,
                truncated_frames: 1,
                cap_evicted_frames: 0,
            },
            "a byte-cap eviction inside a capture's floor is a TRUNCATION, not ageing — a \
             capture that loses frames this way covers less than it claims"
        );
        assert_eq!(w.bytes(), 300);
        assert_eq!(w.truncated_frames(), 1);
        assert_eq!(w.aged_frames(), 0);
        // A TRUNCATION is not a standing cap-bite: the capture already reports it
        // as its own loss, and folding it in here would double-count.
        assert_eq!(w.cap_evicted_frames(), 0);

        // With NO capture active the same overflow is ordinary ageing — the
        // anti-tautology half: the report's two counters must be chosen by the
        // floor, not by which loop dropped the batch.
        w.push(0, 4_000 * MS, batch(1, 100));
        let report = w.evict(4_000 * MS, None);
        assert_eq!(
            report,
            EvictionReport {
                aged_frames: 1,
                truncated_frames: 0,
                cap_evicted_frames: 1,
            }
        );
    }

    /// A standing cap-bite is counted in BOTH totals, and
    /// the two together are what tell it from the span rule working.
    ///
    /// Both halves are load-bearing. `aged_frames` must keep carrying it, because
    /// that number ships in every capture manifest and re-partitioning it would
    /// change what a shipped reader is told about an unchanged recording; and
    /// `cap_evicted_frames` must isolate it, because the aged total alone cannot
    /// distinguish "the window reached its 30 s and is rolling" from "the byte
    /// ceiling is the binding limit and the promised span is not being held".
    #[test]
    fn a_cap_bite_is_counted_in_both_the_aged_total_and_the_cap_split() {
        // Room for exactly 2 batches; the span is enormous, so NOTHING here can
        // age out and every eviction is provably the ceiling.
        let mut w = FrameWindow::new(3_600_000 * MS, 200);
        for s in 0..2u64 {
            w.push(0, s * 1000 * MS, batch(1, 100));
        }
        assert_eq!(w.evict(2_000 * MS, None), EvictionReport::default());

        // Three more batches, three cap evictions.
        for s in 2..5u64 {
            w.push(0, s * 1000 * MS, batch(1, 100));
            let report = w.evict(s * 1000 * MS, None);
            assert_eq!(
                report,
                EvictionReport {
                    aged_frames: 1,
                    truncated_frames: 0,
                    cap_evicted_frames: 1,
                },
                "the span is an hour, so nothing here aged out — every drop is the ceiling"
            );
        }
        assert_eq!(w.aged_frames(), 3, "back-compatible: still the aged total");
        assert_eq!(w.cap_evicted_frames(), 3, "and isolated as a cap-bite");
        assert_eq!(w.truncated_frames(), 0);
    }

    /// A capture's ACHIEVED reach is the oldest batch it will
    /// carry, which under a biting ceiling is NOT the floor it claims.
    ///
    /// The headline hazard the field exists for: the floor is frozen at the
    /// trigger while the byte pass evicts from the FRONT, so the claimed span
    /// keeps its full width while the frames behind it walk away.
    #[test]
    fn the_achieved_reach_is_the_oldest_held_batch_not_the_claimed_floor() {
        let mut w = FrameWindow::new(30_000 * MS, 300);
        for s in 0..10u64 {
            w.push(0, s * 1000 * MS, batch(1, 100));
            w.evict(s * 1000 * MS, None);
        }
        // A capture triggered at t=9s claims back to 0 (9 − 30, saturating).
        let floor = 0u64;
        assert_eq!(
            w.first_stamp_from(floor),
            Some(7_000 * MS),
            "three batches fit, so the capture reaches back to 7s, not to the 0 it claims"
        );
        // It is the SAME range the capture writes — read off one walk, so the two
        // cannot disagree.
        assert_eq!(
            w.batches_from(floor).next().map(|b| b.taken_at_ns),
            w.first_stamp_from(floor)
        );
        // An empty window claims nothing rather than reporting an affirmative 0.
        w.clear();
        assert_eq!(w.first_stamp_from(floor), None);
    }

    /// A topic whose period exceeds the achieved span
    /// contributes ZERO frames, and a global range implies coverage it has not.
    #[test]
    fn topics_that_contribute_no_frames_to_the_range_are_counted() {
        let mut w = FrameWindow::new(30_000 * MS, 1 << 30);
        // Four taps declared; only 0 and 2 publish inside the range.
        w.push(0, 1_000 * MS, batch(1, 8));
        w.push(2, 2_000 * MS, batch(1, 8));
        // Tap 1 published, but BEFORE the range — which is exactly the shape a
        // reader would otherwise read as covered.
        w.push(1, 0, batch(1, 8));
        assert_eq!(w.topics_with_no_frames(1_000 * MS, 4), 2);
        // Widening the range to include tap 1's frame drops the count.
        assert_eq!(w.topics_with_no_frames(0, 4), 1);
        // A batch whose index is past the tap list cannot mark a topic covered.
        w.push(9, 3_000 * MS, batch(1, 8));
        assert_eq!(w.topics_with_no_frames(1_000 * MS, 4), 2);
        // No taps at all: nothing to be missing.
        assert_eq!(w.topics_with_no_frames(0, 0), 0);
    }

    /// A ceiling below a single batch must not spin. The window ends up empty and
    /// over its ceiling, which is a configuration the operator asked for.
    #[test]
    fn a_ceiling_below_one_batch_empties_the_window_rather_than_looping() {
        let mut w = FrameWindow::new(30_000 * MS, 10);
        w.push(0, 0, batch(1, 4096));
        let report = w.evict(0, None);
        assert_eq!(report.aged_frames, 1);
        assert_eq!(w.frames(), 0);
        assert_eq!(w.bytes(), 0);
        // Evicting again over an empty window is a no-op, not a hang.
        assert_eq!(w.evict(0, None), EvictionReport::default());
    }

    /// An empty batch is not held: it carries nothing recoverable, and holding it
    /// would let `oldest_ns` claim coverage the window does not have.
    #[test]
    fn an_empty_batch_is_not_held_and_cannot_fake_coverage() {
        let mut w = FrameWindow::new(30_000 * MS, 1 << 30);
        w.push(0, 5_000 * MS, StagedFrames::default());
        assert_eq!(w.oldest_ns(), None);
        assert_eq!(w.frames(), 0);
        assert_eq!(w.batches_from(0).count(), 0);
    }

    /// `batches_from` selects the capture's range and yields it OLDEST FIRST —
    /// the order a bag is written in.
    #[test]
    fn batches_from_yields_the_captures_range_oldest_first() {
        let mut w = FrameWindow::new(30_000 * MS, 1 << 30);
        for s in 0..6u64 {
            w.push(s as usize, s * 1000 * MS, batch(1, 8));
        }
        let picked: Vec<usize> = w.batches_from(2_000 * MS).map(|b| b.topic_idx).collect();
        assert_eq!(
            picked,
            vec![2, 3, 4, 5],
            "the range is inclusive at the floor and ascending — a bag written backwards \
             renders as time running in reverse"
        );
    }

    /// A window whose span is zero still works — every batch ages out at once —
    /// and, more importantly, does not underflow the horizon arithmetic when
    /// `now_ns` is smaller than the span (the recorder's clock starts near zero).
    #[test]
    fn a_clock_below_the_span_does_not_underflow_into_evicting_everything() {
        let mut w = FrameWindow::new(30_000 * MS, 1 << 30);
        w.push(0, 0, batch(1, 8));
        w.push(0, 100 * MS, batch(1, 8));
        // 100 ms into the run the horizon would be NEGATIVE. Saturating means the
        // horizon is 0 and nothing is evicted — a wrapping subtraction would make
        // it enormous and evict the entire window on the recorder's first pass.
        assert_eq!(w.evict(100 * MS, None), EvictionReport::default());
        assert_eq!(w.frames(), 2);
    }

    /// A capture takes a POINTER to each batch, never a copy of
    /// its payload.
    ///
    /// The oracle is `Arc::ptr_eq` against the batch still sitting in the
    /// window, which is the only thing that can tell a shared batch from a
    /// faithful deep copy — every byte-level assertion in this file passes
    /// either way, which is exactly how the memcpy sat on the drive loop
    /// unnoticed. The ANTI-TAUTOLOGY half is in the same body: a batch the
    /// capture's floor excludes must not be handed over at all, so "ptr_eq holds"
    /// cannot be satisfied by a `batches_from` that yields everything.
    #[test]
    fn a_capture_shares_the_windows_batches_rather_than_copying_them() {
        let mut w = FrameWindow::new(30_000 * MS, 1 << 30);
        w.push(0, 10 * MS, batch(4, 64));
        w.push(1, 20 * MS, batch(4, 64));

        // What the window holds, by identity.
        let held: Vec<Arc<StagedFrames>> =
            w.batches_from(0).map(|b| Arc::clone(&b.frames)).collect();
        assert_eq!(
            held.len(),
            2,
            "precondition: both batches are in the window"
        );

        // What a capture from 20 ms would take.
        let taken: Vec<Arc<StagedFrames>> = w
            .batches_from(20 * MS)
            .map(|b| Arc::clone(&b.frames))
            .collect();
        assert_eq!(
            taken.len(),
            1,
            "a capture takes its own range, not the whole window — without this the \
             identity assertion below could be satisfied by handing over everything"
        );
        assert!(
            Arc::ptr_eq(&taken[0], &held[1]),
            "the capture must hold the SAME allocation the window does; a deep copy \
             passes every byte assertion in this file and costs the drive loop the \
             whole window"
        );
        assert!(
            !Arc::ptr_eq(&taken[0], &held[0]),
            "…and it must be the batch its floor selected, not merely some batch"
        );
    }

    /// D16, the STATED COST: eviction cannot reclaim a batch an
    /// in-flight writer still holds, and the gauge says how much that is.
    ///
    /// Driven through the real `evict`, so the number reported is the one a real
    /// eviction produces. The control is the FIRST batch, evicted while nothing
    /// else holds it: its bytes are reclaimed outright and must NOT be counted —
    /// without that half, a gauge that simply summed every eviction would pass.
    #[test]
    fn eviction_cannot_reclaim_what_a_writer_holds_and_the_gauge_says_so() {
        let mut w = FrameWindow::new(30_000 * MS, 1 << 30);
        w.push(0, 10 * MS, batch(2, 100));
        w.push(1, 20 * MS, batch(2, 100));
        let per_batch = 200;
        assert_eq!(w.bytes(), 2 * per_batch, "precondition: hand-built sizes");
        assert_eq!(w.writer_held_bytes(), 0, "nothing has been handed over yet");

        // A capture takes ONLY the second batch — the first is its control.
        let job: Vec<Arc<StagedFrames>> = w
            .batches_from(20 * MS)
            .map(|b| Arc::clone(&b.frames))
            .collect();
        assert_eq!(job.len(), 1);
        assert_eq!(
            w.writer_held_bytes(),
            0,
            "handing a batch over changes NOTHING while the window still holds it — the \
             gauge is about what eviction could not reclaim, not about what is shared"
        );

        // Age both out from under the writer.
        let report = w.evict(60_000 * MS, None);
        assert_eq!(report.aged_frames, 4, "both batches aged out");
        assert_eq!(w.bytes(), 0, "the window has released both");
        assert_eq!(
            w.writer_held_bytes(),
            per_batch,
            "exactly the writer's batch is still resident; the unheld one was really \
             reclaimed and must not be counted"
        );

        // The writer settles: its job is dropped and the bytes go.
        drop(job);
        w.note_writer_released();
        assert_eq!(w.writer_held_bytes(), 0);
    }

    /// A teardown `clear` books the same way an eviction does.
    ///
    /// Its own arm because `clear` does not go through `pop_front_counting`, so
    /// a gauge wired only into eviction reports ZERO for a window torn down
    /// while a capture is still being written — under-reporting resident memory,
    /// which is the one direction this number must not err in.
    #[test]
    fn a_teardown_clear_still_books_what_a_writer_holds() {
        let mut w = FrameWindow::new(30_000 * MS, 1 << 30);
        w.push(0, 10 * MS, batch(2, 100));
        w.push(1, 20 * MS, batch(2, 100));
        let job: Vec<Arc<StagedFrames>> =
            w.batches_from(0).map(|b| Arc::clone(&b.frames)).collect();
        assert_eq!(job.len(), 2);

        w.clear();
        assert_eq!(w.frames(), 0, "the window is empty");
        assert_eq!(
            w.writer_held_bytes(),
            400,
            "…but the writer's two batches are still resident"
        );
        drop(job);
    }
}
