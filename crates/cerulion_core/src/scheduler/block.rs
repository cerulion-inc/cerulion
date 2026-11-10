// SPDX-License-Identifier: AGPL-3.0-only
//! The `block` deferral decision, as pure functions over one reading of each
//! block consumer's credit slot.
//!
//! A producer with `block` consumers is deferred while ANY of them holds its
//! slot full (`outstanding >= depth`). The deferral is of the whole node: a
//! deferred node does not tick, so none of its outputs publish, including
//! outputs nobody consumes with `block`. Consumers that read the same topic
//! with `drop_oldest` or `sample(N)` hold no slot and never defer anyone.
//!
//! Each slot is judged on its own by [`judge_slot`], and the node defers iff
//! some verdict [`SlotVerdict::defers`]. The live call site keeps a
//! [`StallMark`] per slot between evaluations; the mark is the only state, and
//! it is passed in and handed back, so the same inputs always give the same
//! verdict.
//!
//! # `block_timeout_ms`
//!
//! A slot that stays full while its word's wake epoch does not move (the
//! consumer frees nothing) for `block_timeout_ms` gets [`SlotVerdict::TimedOut`].
//! The caller then forgives that slot's count
//! ([`crate::credit::CreditShared::forgive_outstanding`]), emits the loud
//! release event, and does not defer on it this evaluation. A consumer that
//! drains at any rate moves the epoch and restarts the clock, so a slow
//! consumer never times out; a stopped one does.
//!
//! The producers of a topic share one [`StallCell`] per consumer slot. The
//! caller reads the slot, judges it and forgives it inside
//! [`StallCell::update`], under the cell's lock, so a second producer judges
//! the mark the first one stored and one stall releases the slot once. In a
//! graph only a local slot with one producer can time out: graph load refuses
//! `block_timeout_ms` on a topic with several producers and on an edge whose
//! producer runs in another process or executor unit, and the defer edge of a
//! cross-process word carries no timeout.
//!
//! The release lands at the first evaluation at or after the deadline;
//! [`release_deadline_ns`] gives that deadline to a caller that sizes a wait.
//!
//! # Wait sizing
//!
//! A held producer is due by every other signal the scheduler reads (its
//! period passed, or its trigger frame is queued). [`slot_hold`] and
//! [`node_hold`] tell the live loop's wait sizing when such a producer can
//! next fire: at the earliest timeout release of a full slot, or only after a
//! drain or a release, each of which wakes the loop on its own. A slot the
//! loop may not wait on keeps the node due, so the loop re-evaluates it at
//! its floor as before.
//!
//! # Replay
//!
//! None of this runs in a replay: the runtime's replay bypass skips the block
//! gate entirely, and the fires come from the recording.

use crate::credit::CreditShared;
use std::sync::Mutex;

/// One reading of one block consumer's credit slot.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SlotReading {
    /// Frames published to this consumer and not yet drained.
    pub outstanding: u64,
    /// The defer line: the consumer's declared depth.
    pub depth: u64,
    /// The slot was given up for the rest of the run
    /// ([`CreditShared::release_slot`]).
    pub released: bool,
    /// The word's wake epoch; it moves on every freeing drain.
    pub wake_seq: u32,
}

impl SlotReading {
    /// Read `word` against the defer line `depth`.
    pub fn of(word: &CreditShared, depth: u64) -> Self {
        Self {
            released: word.is_released(),
            outstanding: word.outstanding(),
            depth,
            wake_seq: word.wake_seq_snapshot(),
        }
    }
}

/// When a slot was first seen full with its wake epoch at `seq`. `None` while
/// the slot has room.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct StallMark {
    /// The evaluation time at which the current full-and-unmoved stretch began.
    pub full_since_ns: Option<u64>,
    /// The wake epoch read at that time.
    pub seq: u32,
}

/// The verdict on one slot.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SlotVerdict {
    /// Below depth: no deferral.
    Room,
    /// The slot was released for the rest of the run: no deferral.
    Released,
    /// At or above depth: the producer defers.
    Full,
    /// Full and unmoved for at least `block_timeout_ms`: the caller forgives
    /// this slot and emits the release event; no deferral.
    TimedOut {
        /// How long the slot had been full and unmoved.
        held_ns: u64,
    },
}

impl SlotVerdict {
    /// Does this verdict defer the producer?
    pub fn defers(self) -> bool {
        matches!(self, Self::Full)
    }
}

/// Why a block consumer's slot was released.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ReleaseReason {
    /// The consumer's process died, or the consumer node stopped with a fatal
    /// error. Sticky for the rest of the run.
    ConsumerDied,
    /// The slot stayed full with no drain for `block_timeout_ms`. The slot
    /// stays armed and can defer again.
    Timeout,
}

impl ReleaseReason {
    /// The `reason` field value of the release event.
    pub fn as_str(self) -> &'static str {
        match self {
            Self::ConsumerDied => "consumer_died",
            Self::Timeout => "block_timeout",
        }
    }
}

/// Judge one slot.
///
/// `timeout_ns` is the edge's `block_timeout_ms` in ns, `None` when the edge
/// declares none. Returns the verdict and the mark to keep for the next
/// evaluation.
pub fn judge_slot(
    reading: SlotReading,
    mark: StallMark,
    now_ns: u64,
    timeout_ns: Option<u64>,
) -> (SlotVerdict, StallMark) {
    if reading.released {
        return (SlotVerdict::Released, StallMark::default());
    }
    if reading.outstanding < reading.depth {
        return (SlotVerdict::Room, StallMark::default());
    }
    let since = match mark.full_since_ns {
        Some(since) if mark.seq == reading.wake_seq => since,
        // First full reading, or the consumer freed credit since the last
        // one: a new stretch starts now.
        _ => now_ns,
    };
    let held_ns = now_ns.saturating_sub(since);
    if let Some(timeout) = timeout_ns {
        if held_ns >= timeout && !crate::oracle_mutant::active("block_no_timeout_release") {
            return (SlotVerdict::TimedOut { held_ns }, StallMark::default());
        }
    }
    (
        SlotVerdict::Full,
        StallMark {
            full_since_ns: Some(since),
            seq: reading.wake_seq,
        },
    )
}

/// The instant at which a slot under `mark` times out, or `None` when it has
/// room or the edge declares no timeout.
pub fn release_deadline_ns(mark: StallMark, timeout_ns: Option<u64>) -> Option<u64> {
    Some(mark.full_since_ns?.saturating_add(timeout_ns?))
}

/// Defer a node whose block slots received these verdicts?
pub fn node_defers(verdicts: impl IntoIterator<Item = SlotVerdict>) -> bool {
    verdicts.into_iter().any(SlotVerdict::defers)
}

/// How one slot holds its producer, as the live loop's wait sizing reads it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SlotHold {
    /// The slot has room or was released: it holds nothing.
    Free,
    /// The slot is full and the next evaluation decides something this read
    /// cannot: its stall clock has not started, a drain moved the epoch since
    /// it started, or the loop may not wait on this slot.
    Evaluate,
    /// The slot is full and times out at this instant.
    Until(u64),
    /// The slot is full with no timeout; only a drain or a release frees it,
    /// and each wakes the loop.
    Wake,
}

/// Read one slot for the wait sizing. `mark` is the slot's stall mark as the
/// last [`judge_slot`] left it, `timeout_ns` the edge's `block_timeout_ms` in
/// ns, and `may_wait` whether the live loop may wait for this slot's free
/// instead of re-evaluating it: a drain or a release of the slot must wake the
/// loop (a local slot's consumer drains on the same level loop; a mapped
/// slot's peer drain rings the producer's credit park).
pub fn slot_hold(
    reading: SlotReading,
    mark: StallMark,
    timeout_ns: Option<u64>,
    may_wait: bool,
) -> SlotHold {
    if reading.released || reading.outstanding < reading.depth {
        return SlotHold::Free;
    }
    if !may_wait {
        return SlotHold::Evaluate;
    }
    let started = mark.full_since_ns.is_some() && mark.seq == reading.wake_seq;
    match (timeout_ns, started) {
        (None, _) => SlotHold::Wake,
        (Some(_), false) => SlotHold::Evaluate,
        (Some(t), true) => match release_deadline_ns(mark, Some(t)) {
            Some(deadline) => SlotHold::Until(deadline),
            None => SlotHold::Evaluate,
        },
    }
}

/// When a node with `block` slots can next fire, for the live loop's wait
/// sizing.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BlockHold {
    /// No slot holds the node past what the scheduler already reads, or one
    /// needs an evaluation: the node's own deadline stands.
    Clear,
    /// Held; the earliest timeout release of a full slot lands at this
    /// instant, and the loop evaluates the node then.
    Until(u64),
    /// Held by slots that only a drain or a release frees, each of which wakes
    /// the loop: the node contributes no deadline.
    UntilWake,
}

/// Fold the [`slot_hold`] of every `block` slot of one node. Any slot that
/// needs an evaluation keeps the node due; otherwise the node is held while
/// any slot is full, until the earliest timeout among them, or until a wake
/// when none times out.
pub fn node_hold(holds: impl IntoIterator<Item = SlotHold>) -> BlockHold {
    let mut held = false;
    let mut until: Option<u64> = None;
    for h in holds {
        match h {
            SlotHold::Free => {}
            SlotHold::Evaluate => return BlockHold::Clear,
            SlotHold::Until(t) => {
                held = true;
                until = Some(until.map_or(t, |u| u.min(t)));
            }
            SlotHold::Wake => held = true,
        }
    }
    match (held, until) {
        (false, _) => BlockHold::Clear,
        (true, Some(t)) => BlockHold::Until(t),
        (true, None) => BlockHold::UntilWake,
    }
}

/// A [`StallMark`] held across evaluations by a live call site that may run on
/// more than one thread (every producer of a topic shares one per consumer).
/// One lock, so a reader never sees one field of a mark with the other of the
/// next.
#[derive(Debug, Default)]
pub struct StallCell(Mutex<StallMark>);

impl StallCell {
    /// The mark as last stored.
    pub fn load(&self) -> StallMark {
        *self.0.lock().unwrap_or_else(|e| e.into_inner())
    }

    /// Store `mark` for the next evaluation.
    pub fn store(&self, mark: StallMark) {
        *self.0.lock().unwrap_or_else(|e| e.into_inner()) = mark;
    }

    /// Run `f` on the stored mark under the cell's lock, store the mark `f`
    /// returns, and return `f`'s other result. Every other `load`, `store`
    /// and `update` on the cell waits until `f` returns.
    pub fn update<R>(&self, f: impl FnOnce(StallMark) -> (StallMark, R)) -> R {
        let mut mark = self.0.lock().unwrap_or_else(|e| e.into_inner());
        let (next, out) = f(*mark);
        *mark = next;
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn reading(outstanding: u64, depth: u64, seq: u32) -> SlotReading {
        SlotReading {
            outstanding,
            depth,
            released: false,
            wake_seq: seq,
        }
    }

    #[test]
    fn room_below_depth_and_full_at_depth() {
        let (v, m) = judge_slot(reading(1, 2, 0), StallMark::default(), 10, None);
        assert_eq!((v, m), (SlotVerdict::Room, StallMark::default()));
        let (v, m) = judge_slot(reading(2, 2, 7), StallMark::default(), 10, None);
        assert_eq!(v, SlotVerdict::Full);
        assert_eq!(
            m,
            StallMark {
                full_since_ns: Some(10),
                seq: 7
            }
        );
        assert!(v.defers());
        assert!(!SlotVerdict::Room.defers());
    }

    #[test]
    fn a_released_slot_never_defers_whatever_its_count() {
        let r = SlotReading {
            released: true,
            ..reading(99, 2, 0)
        };
        let (v, _) = judge_slot(r, StallMark::default(), 0, Some(1));
        assert_eq!(v, SlotVerdict::Released);
        assert!(!v.defers());
    }

    #[test]
    fn the_timeout_fires_only_after_an_unmoved_stretch_of_the_full_length() {
        let t = Some(1_000);
        let (v, m) = judge_slot(reading(4, 4, 3), StallMark::default(), 5_000, t);
        assert_eq!(v, SlotVerdict::Full);
        let (v, m) = judge_slot(reading(4, 4, 3), m, 5_999, t);
        assert_eq!(v, SlotVerdict::Full, "999 ns held is short of 1000");
        assert_eq!(release_deadline_ns(m, t), Some(6_000));
        let (v, m) = judge_slot(reading(4, 4, 3), m, 6_000, t);
        assert_eq!(v, SlotVerdict::TimedOut { held_ns: 1_000 });
        assert!(!v.defers());
        assert_eq!(m, StallMark::default());
    }

    #[test]
    fn a_drain_restarts_the_timeout_clock() {
        let t = Some(1_000);
        let (_, m) = judge_slot(reading(4, 4, 3), StallMark::default(), 0, t);
        // The consumer freed one and the producer refilled it: still full, but
        // the epoch moved.
        let (v, m) = judge_slot(reading(4, 4, 4), m, 900, t);
        assert_eq!(v, SlotVerdict::Full);
        assert_eq!(m.full_since_ns, Some(900));
        let (v, _) = judge_slot(reading(4, 4, 4), m, 1_800, t);
        assert_eq!(v, SlotVerdict::Full, "only 900 ns since the drain");
        let (v, _) = judge_slot(reading(4, 4, 4), m, 1_900, t);
        assert!(matches!(v, SlotVerdict::TimedOut { .. }));
    }

    #[test]
    fn no_timeout_declared_means_full_forever() {
        let (_, m) = judge_slot(reading(1, 1, 0), StallMark::default(), 0, None);
        let (v, _) = judge_slot(reading(1, 1, 0), m, u64::MAX, None);
        assert_eq!(v, SlotVerdict::Full);
        assert_eq!(release_deadline_ns(m, None), None);
    }

    #[test]
    fn the_node_defers_iff_any_slot_is_full() {
        use SlotVerdict::*;
        assert!(!node_defers([]));
        assert!(!node_defers([Room, Released, TimedOut { held_ns: 1 }]));
        assert!(node_defers([Room, Full, Released]));
    }

    #[test]
    fn a_full_slot_with_a_started_clock_holds_until_its_deadline() {
        let t = Some(1_000);
        let (_, m) = judge_slot(reading(2, 2, 5), StallMark::default(), 4_000, t);
        assert_eq!(
            slot_hold(reading(2, 2, 5), m, t, true),
            SlotHold::Until(5_000)
        );
        assert_eq!(
            slot_hold(reading(2, 2, 5), m, None, true),
            SlotHold::Wake,
            "no timeout: only a drain or a release frees it"
        );
    }

    #[test]
    fn a_slot_the_next_evaluation_must_judge_keeps_its_node_due() {
        let t = Some(1_000);
        assert_eq!(
            slot_hold(reading(2, 2, 5), StallMark::default(), t, true),
            SlotHold::Evaluate,
            "the stall clock has not started"
        );
        let (_, m) = judge_slot(reading(2, 2, 5), StallMark::default(), 0, t);
        assert_eq!(
            slot_hold(reading(2, 2, 6), m, t, true),
            SlotHold::Evaluate,
            "a drain moved the epoch, so the clock restarts at the next evaluation"
        );
        assert_eq!(
            slot_hold(reading(2, 2, 5), m, t, false),
            SlotHold::Evaluate,
            "a slot the loop may not wait on is found free only by evaluating"
        );
        assert_eq!(
            slot_hold(reading(2, 2, 5), m, None, false),
            SlotHold::Evaluate
        );
    }

    #[test]
    fn a_slot_with_room_or_released_holds_nothing() {
        let (_, m) = judge_slot(reading(2, 2, 0), StallMark::default(), 0, Some(9));
        assert_eq!(
            slot_hold(reading(1, 2, 0), m, Some(9), true),
            SlotHold::Free
        );
        let released = SlotReading {
            released: true,
            ..reading(2, 2, 0)
        };
        assert_eq!(slot_hold(released, m, Some(9), true), SlotHold::Free);
    }

    #[test]
    fn the_node_is_held_until_its_earliest_timeout_or_a_wake() {
        use SlotHold::*;
        assert_eq!(node_hold([]), BlockHold::Clear);
        assert_eq!(node_hold([Free, Free]), BlockHold::Clear);
        assert_eq!(node_hold([Until(9), Wake, Until(7)]), BlockHold::Until(7));
        assert_eq!(node_hold([Free, Wake]), BlockHold::UntilWake);
        assert_eq!(
            node_hold([Until(7), Evaluate, Wake]),
            BlockHold::Clear,
            "one slot to evaluate keeps the node due"
        );
    }

    #[test]
    fn the_stall_cell_round_trips_a_mark() {
        let c = StallCell::default();
        assert_eq!(c.load(), StallMark::default());
        let m = StallMark {
            full_since_ns: Some(42),
            seq: 9,
        };
        c.store(m);
        assert_eq!(c.load(), m);
        c.store(StallMark::default());
        assert_eq!(c.load().full_since_ns, None);
    }

    /// Producers sharing a cell each read, judge and forgive under its lock:
    /// of many threads that find one timed-out slot, exactly one releases it,
    /// and every frame a thread publishes after that release stays counted,
    /// where a second release would have forgiven it.
    #[test]
    fn producers_sharing_a_cell_release_one_stall_once() {
        use crate::credit::CreditWord;
        use std::sync::atomic::{AtomicU64, Ordering};
        use std::sync::{Arc, Barrier};
        const THREADS: usize = 8;
        for _ in 0..200 {
            let word = CreditWord::local(2);
            word.record_published_n(2);
            let cell = Arc::new(StallCell::default());
            cell.store(StallMark {
                full_since_ns: Some(0),
                seq: word.wake_seq_snapshot(),
            });
            let releases = Arc::new(AtomicU64::new(0));
            let publishes = Arc::new(AtomicU64::new(0));
            let start = Arc::new(Barrier::new(THREADS));
            let threads: Vec<_> = (0..THREADS)
                .map(|_| {
                    let (word, cell, releases, publishes, start) = (
                        word.clone(),
                        Arc::clone(&cell),
                        Arc::clone(&releases),
                        Arc::clone(&publishes),
                        Arc::clone(&start),
                    );
                    std::thread::spawn(move || {
                        start.wait();
                        let (verdict, forgiven) = cell.update(|mark| {
                            let reading = SlotReading::of(&word, 2);
                            let (verdict, next) = judge_slot(reading, mark, 1_000, Some(1_000));
                            let forgiven = match verdict {
                                SlotVerdict::TimedOut { .. } => word.forgive_outstanding(),
                                _ => 0,
                            };
                            (next, (verdict, forgiven))
                        });
                        if forgiven != 0 {
                            releases.fetch_add(1, Ordering::AcqRel);
                        }
                        if !verdict.defers() {
                            word.record_published();
                            publishes.fetch_add(1, Ordering::AcqRel);
                        }
                    })
                })
                .collect();
            for t in threads {
                t.join().expect("a producer thread panicked");
            }
            assert_eq!(
                releases.load(Ordering::Acquire),
                1,
                "one stall releases the slot once"
            );
            assert_eq!(
                word.outstanding(),
                publishes.load(Ordering::Acquire),
                "every frame published after the one release is still counted"
            );
        }
    }
}
