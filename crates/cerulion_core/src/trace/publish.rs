// SPDX-License-Identifier: AGPL-3.0-only
//! Publish trace ring buffer — metadata-only.

use std::collections::VecDeque;
use std::sync::Arc;
use std::time::Duration;

use super::bag::BagWriter;

/// One publish event recorded in the trace. Fixed-size (40 bytes on a
/// 64-bit target: the 16-byte `Arc<str>` pointer, two `u64` and a padded
/// `u32`) so trace memory scales with publish rate, not payload size.
///
/// The wire payload itself is NOT captured here. Full-payload recording is
/// the MCAP bag: `cerulion graph run --record` and `cerulion bag record`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PublishTraceEntry {
    /// Topic the publish happened on. `Arc<str>` to share storage
    /// across the trace (one allocation per unique topic name).
    pub topic: Arc<str>,
    /// Publisher's per-topic sequence number.
    pub sequence: u32,
    /// Publish timestamp in nanoseconds (from `real_ns` or
    /// `VirtualClock`).
    pub publish_time_ns: u64,
    /// The message's layout-sensitive schema hash, the same value that
    /// lives in `WireHeader.schema_hash` (the recipe is
    /// `codegen::MessageSchema::schema_hash`; it is not a hash of the name
    /// alone). Allows trace consumers to
    /// detect schema-incompatible replay without payload access.
    pub schema_hash: u64,
}

/// Depth specification for the ring buffer. Bounded by count, by a
/// time window, or both (whichever evicts first).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HistoryDepth {
    /// Keep the last N entries; evict oldest on overflow.
    Count(usize),
    /// Keep all entries within the last `Duration` (measured against
    /// the most recent entry's timestamp).
    Window(Duration),
    /// Apply both rules — whichever evicts first. Useful as a safety
    /// belt: time-based retention plus a hard count cap to bound
    /// worst-case memory under publish-rate spikes.
    Bounded { count: usize, window: Duration },
}

impl Default for HistoryDepth {
    /// Default: 30-second time window. Bounds memory by rate, not by
    /// arbitrary count cap; cheap on slow topics, scales with rate
    /// on fast topics.
    fn default() -> Self {
        HistoryDepth::Window(Duration::from_secs(30))
    }
}

/// In-memory publish trace — a per-publisher ring buffer of
/// `PublishTraceEntry` records.
///
/// Single-threaded by design: the publisher's hot path is the only
/// writer, and consumers (test harnesses, CLI inspectors) read out of
/// band. If concurrent access is needed later, wrap in `Mutex` at the
/// call site.
///
/// Optionally attached: a `BagWriter` that receives each evicted
/// entry as a JSON Lines record on disk. See `bag.rs` for the disk
/// overflow protocol.
pub struct PublishTrace {
    depth: HistoryDepth,
    entries: VecDeque<PublishTraceEntry>,
    /// Optional disk-overflow sink. When attached, every entry
    /// evicted from `entries` is also written to `bag`.
    bag: Option<BagWriter>,
}

impl PublishTrace {
    /// Create a new trace with the given depth spec.
    pub fn new(depth: HistoryDepth) -> Self {
        Self {
            depth,
            entries: VecDeque::new(),
            bag: None,
        }
    }

    /// Attach a `BagWriter` for disk overflow. Every entry evicted
    /// from the in-memory ring buffer will be written to the bag.
    pub fn attach_bag(&mut self, bag: BagWriter) {
        self.bag = Some(bag);
    }

    /// Push a new entry. Evicts older entries per the depth policy,
    /// piping each evicted entry to the attached `BagWriter` (if any).
    pub fn record(&mut self, entry: PublishTraceEntry) {
        self.entries.push_back(entry);
        self.evict();
    }

    /// Apply the eviction policy: drop entries that violate `depth`.
    /// Evicted entries are written to the attached bag (if any).
    fn evict(&mut self) {
        match self.depth {
            HistoryDepth::Count(n) => {
                while self.entries.len() > n {
                    if let Some(evicted) = self.entries.pop_front() {
                        self.forward_to_bag(&evicted);
                    }
                }
            }
            HistoryDepth::Window(window) => {
                self.evict_by_window(window);
            }
            HistoryDepth::Bounded { count, window } => {
                while self.entries.len() > count {
                    if let Some(evicted) = self.entries.pop_front() {
                        self.forward_to_bag(&evicted);
                    }
                }
                self.evict_by_window(window);
            }
        }
    }

    /// Evict entries older than `(latest_time - window)`. Uses the
    /// latest entry's timestamp as the reference clock so the trace
    /// is self-consistent without needing an external clock argument.
    fn evict_by_window(&mut self, window: Duration) {
        let Some(latest) = self.entries.back() else {
            return;
        };
        let latest_time = latest.publish_time_ns;
        let window_ns = window.as_nanos() as u64;
        // Saturating: if latest_time < window_ns (very early run),
        // keep everything.
        let cutoff = latest_time.saturating_sub(window_ns);
        while let Some(front) = self.entries.front() {
            if front.publish_time_ns < cutoff {
                if let Some(evicted) = self.entries.pop_front() {
                    self.forward_to_bag(&evicted);
                }
            } else {
                break;
            }
        }
    }

    /// Best-effort forward an evicted entry to the attached bag.
    /// I/O errors are logged via `tracing::warn!` but never panic —
    /// the trace ring buffer must remain functional even when the
    /// disk is full or unavailable.
    fn forward_to_bag(&mut self, entry: &PublishTraceEntry) {
        if let Some(bag) = &mut self.bag {
            if let Err(e) = bag.write_entry(entry) {
                tracing::warn!(
                    error = %e,
                    topic = %entry.topic,
                    "BagWriter failed to persist evicted PublishTraceEntry"
                );
            }
        }
    }

    /// Read-only view of all entries in chronological order
    /// (oldest first).
    pub fn entries(&self) -> impl Iterator<Item = &PublishTraceEntry> {
        self.entries.iter()
    }

    /// Number of entries currently held.
    pub fn len(&self) -> usize {
        self.entries.len()
    }

    /// True if the trace is empty.
    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }

    /// The configured depth policy.
    pub fn depth(&self) -> HistoryDepth {
        self.depth
    }

    /// Drop all entries. The depth policy is preserved.
    pub fn clear(&mut self) {
        self.entries.clear();
    }
}

// ---------------------------------------------------------------------------
// This module's half of the ABI LAYOUT PIN (see `crate::abi_layout`).
//
// `abi_pin_struct!` expands to an exhaustive destructuring pattern with no `..`
// rest pattern, so adding or removing a field of one of these structs is a
// COMPILE ERROR naming the struct; it also measures size/align/`offset_of!`,
// which `crate::abi_layout` compares against the snapshot table keyed to
// `CERULION_ABI_VERSION`. `abi_pin_enum!` does the same for a variant set.
// ---------------------------------------------------------------------------
#[cfg(test)]
pub(crate) fn abi_layout_pins() -> Vec<crate::abi_layout::MeasuredStruct> {
    use crate::abi_layout::{abi_pin_enum, abi_pin_struct};
    vec![
        abi_pin_struct!(PublishTrace {
            depth,
            entries,
            bag
        }),
        abi_pin_struct!(PublishTraceEntry {
            topic,
            sequence,
            publish_time_ns,
            schema_hash
        }),
        abi_pin_enum!(HistoryDepth {
            HistoryDepth::Count(_),
            HistoryDepth::Window(_),
            HistoryDepth::Bounded { .. }
        }),
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    fn entry(seq: u32, ts_ns: u64) -> PublishTraceEntry {
        PublishTraceEntry {
            topic: Arc::from("test/topic"),
            sequence: seq,
            publish_time_ns: ts_ns,
            schema_hash: 0xDEADBEEF,
        }
    }

    #[test]
    fn count_depth_evicts_oldest() {
        let mut trace = PublishTrace::new(HistoryDepth::Count(3));
        trace.record(entry(1, 1_000));
        trace.record(entry(2, 2_000));
        trace.record(entry(3, 3_000));
        trace.record(entry(4, 4_000));
        assert_eq!(trace.len(), 3, "depth=3 → at most 3 entries");
        let seqs: Vec<u32> = trace.entries().map(|e| e.sequence).collect();
        assert_eq!(seqs, vec![2, 3, 4], "oldest (seq=1) evicted");
    }

    #[test]
    fn window_depth_evicts_outside_window() {
        let mut trace = PublishTrace::new(HistoryDepth::Window(Duration::from_millis(100)));
        trace.record(entry(1, 0));
        trace.record(entry(2, 50_000_000)); // 50 ms
        trace.record(entry(3, 100_000_000)); // 100 ms
        trace.record(entry(4, 200_000_000)); // 200 ms — latest. Window
                                             // = (200ms - 100ms, 200ms] → seq 1 dropped,
                                             // seq 2 dropped (50 ms < 100 ms cutoff),
                                             // seqs 3 + 4 kept.
        let seqs: Vec<u32> = trace.entries().map(|e| e.sequence).collect();
        assert_eq!(
            seqs,
            vec![3, 4],
            "seq 1 + 2 are older than (latest - window) and must be evicted"
        );
    }

    #[test]
    fn window_depth_keeps_everything_in_short_run() {
        // If latest_time < window_ns (early run), saturating_sub gives
        // cutoff = 0, so nothing is evicted.
        let mut trace = PublishTrace::new(HistoryDepth::Window(Duration::from_secs(30)));
        for i in 0..10 {
            trace.record(entry(i, i as u64 * 1_000_000));
        }
        assert_eq!(trace.len(), 10);
    }

    #[test]
    fn bounded_depth_applies_both_rules() {
        let mut trace = PublishTrace::new(HistoryDepth::Bounded {
            count: 5,
            window: Duration::from_millis(1_000),
        });
        // Spam 100 entries close together. Count-cap should keep 5.
        for i in 0..100 {
            trace.record(entry(i, i as u64 * 10_000));
        }
        assert_eq!(trace.len(), 5, "count cap (5) wins on dense publishes");
        let seqs: Vec<u32> = trace.entries().map(|e| e.sequence).collect();
        assert_eq!(seqs, vec![95, 96, 97, 98, 99]);
    }

    #[test]
    fn bounded_depth_window_evicts_old_entries() {
        let mut trace = PublishTrace::new(HistoryDepth::Bounded {
            count: 100,
            window: Duration::from_millis(50),
        });
        trace.record(entry(1, 0));
        trace.record(entry(2, 25_000_000)); // 25 ms
        trace.record(entry(3, 100_000_000)); // 100 ms — latest. Cutoff = 50 ms.
                                             // Seq 1 + 2 are < 50 ms; evicted.
        let seqs: Vec<u32> = trace.entries().map(|e| e.sequence).collect();
        assert_eq!(seqs, vec![3]);
    }

    #[test]
    fn default_depth_is_30_second_window() {
        let trace = PublishTrace::new(HistoryDepth::default());
        match trace.depth() {
            HistoryDepth::Window(d) => assert_eq!(d, Duration::from_secs(30)),
            other => panic!("expected 30-second window default; got {other:?}"),
        }
    }

    #[test]
    fn entry_size_is_bounded() {
        // Pin the entry size so a future field addition surfaces here
        // — trace memory budget assumes ~32 B/entry. The Arc<str>
        // is 16 B on 64-bit (pointer + len); + sequence (4) +
        // padding (4) + publish_time_ns (8) + schema_hash (8) = 40 B.
        // Allow up to 48 B to permit alignment + future minor extension
        // without surprise.
        let size = std::mem::size_of::<PublishTraceEntry>();
        assert!(
            size <= 48,
            "PublishTraceEntry must stay ≤ 48 bytes for the trace's per-event memory budget; got {size}"
        );
    }

    #[test]
    fn clear_drops_entries_but_preserves_depth() {
        let mut trace = PublishTrace::new(HistoryDepth::Count(10));
        trace.record(entry(1, 1_000));
        trace.record(entry(2, 2_000));
        trace.clear();
        assert!(trace.is_empty());
        assert!(matches!(trace.depth(), HistoryDepth::Count(10)));
        trace.record(entry(3, 3_000));
        assert_eq!(trace.len(), 1);
    }
}
