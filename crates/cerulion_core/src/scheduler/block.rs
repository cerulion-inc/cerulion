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
//! The release lands at the first evaluation at or after the deadline;
//! [`release_deadline_ns`] gives that deadline to a caller that sizes a wait.
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
}
