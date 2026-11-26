// SPDX-License-Identifier: AGPL-3.0-only
//! Direct capture stream carrier for executor units that share a host process.
//!
//! A process hosting several executor units cannot safely `fork(2)` while its
//! sibling unit threads run: the child inherits locks those threads hold. After
//! [`super::walk_inline`] finishes the bounded inline attempt, this module
//! streams each eligible deferred node directly into a non-blocking,
//! record-budgeted [`crate::state_ring::TryStateRingSink`] on the unit's own
//! thread.
//!
//! - One shared record budget ([`direct_budget_records`]) reserves one slot per
//!   deferred node plus `headroom_records` so caller-emitted SKIP records always
//!   have room to void any truncated prefix.
//! - No ring push ever waits on a slow or dead recorder.
//! - Each encoder runs inside `catch_unwind`; the caller keeps node mutex guards
//!   outside [`stream_deferred_direct`] so a caught panic cannot poison them.
//! - [`DirectCarrier`] remembers per-node overflow budgets so a node larger than
//!   the available ring budget is not re-encoded on every cadence.
//! - This module pushes no SKIP records itself; it returns failures for the
//!   caller to publish after dropping node locks.

use crate::state::{StateError, StateSink};
use crate::state_carrier::ForkReason;
use crate::state_restore::{AnchorBlob, NodeFrameworkState};
use crate::state_ring::{SkipCause, StateRingProducer};

const DETAIL_NO_STATE_SHAPE: &str =
    "node declares no `state_shape()`, so its anchor cannot be framed";
const DETAIL_INLINE_ENCODER_ERROR: &str =
    "node encoder returned an error during the inline walk; not re-entered on the unit thread";
const DETAIL_INLINE_CAPTURE_UNWOUND: &str =
    "node encoder panicked during the inline walk; not re-entered on the unit thread";
const DETAIL_DIRECT_ENCODER_ERROR: &str =
    "the node's encoder returned an error during direct capture";
const DETAIL_DIRECT_CAPTURE_UNWOUND: &str = "the node's encoder panicked during direct capture";
const DETAIL_RECORDER_BEHIND: &str = "the recorder has not drained enough of the state ring to \
                                      hold this anchor without blocking the node thread";
const DETAIL_STATE_LARGER_THAN_RING: &str =
    "node state is larger than the state ring; raise CERULION_FLASHBACK_STATE_RING_MB to capture it";

/// One deferred node, as the direct capture stream sees it.
///
/// The caller resolves the node's mutex guard before calling
/// [`stream_deferred_direct`] and supplies a borrowed view, keeping
/// `catch_unwind` inside the guard's scope so an encoder panic cannot poison
/// the node's mutex.
pub trait DirectCaptureTarget {
    /// Encode this node's state into `sink`.
    fn capture_direct(&self, sink: &mut dyn StateSink) -> Result<(), StateError>;
}

/// Borrowed descriptor for one node in a boundary's deferred set.
pub struct DirectDeferredNode<'a, T: ?Sized> {
    /// Manifest index of this node on the state ring.
    pub node_idx: u32,
    /// Why [`super::walk_inline`] deferred this node.
    pub reason: ForkReason,
    /// Declared `state_shape()` for the `AnchorBlob` header, if any.
    pub shape: Option<u64>,
    /// Scheduler framework state section for this node (empty uses v1 framing).
    pub framework: &'a NodeFrameworkState,
    /// Node view whose encoder runs under `catch_unwind`.
    pub target: &'a T,
}

/// What [`stream_deferred_direct`] should do with one deferred node.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DirectDisposition {
    /// Frame with `shape` and stream the node's state directly into the ring.
    Stream {
        /// The node's declared `state_shape()`.
        shape: u64,
    },
    /// Do not enter the node's encoder; report a SKIP with `cause` and `detail`.
    SkipWithoutEntry {
        /// Named skip cause for the reader.
        cause: SkipCause,
        /// Fixed human-readable detail for the SKIP record.
        detail: &'static str,
    },
}

/// Decide whether a deferred node may be streamed directly or must be skipped
/// without re-entering its encoder.
pub fn direct_disposition(reason: ForkReason, shape: Option<u64>) -> DirectDisposition {
    match (reason, shape) {
        (ForkReason::NotInlineSafe | ForkReason::ArenaOverflow, Some(shape)) => {
            DirectDisposition::Stream { shape }
        }
        (ForkReason::NotInlineSafe | ForkReason::ArenaOverflow, None) => {
            DirectDisposition::SkipWithoutEntry {
                cause: SkipCause::CaptureFailed,
                detail: DETAIL_NO_STATE_SHAPE,
            }
        }
        (ForkReason::EncoderError, _) => DirectDisposition::SkipWithoutEntry {
            cause: SkipCause::CaptureFailed,
            detail: DETAIL_INLINE_ENCODER_ERROR,
        },
        (ForkReason::CaptureUnwound, _) => DirectDisposition::SkipWithoutEntry {
            cause: SkipCause::CaptureFailed,
            detail: DETAIL_INLINE_CAPTURE_UNWOUND,
        },
    }
}

/// Compute the shared direct-stream record budget for one boundary.
///
/// Subtracts `deferred_nodes` (one reserved SKIP slot per deferred node) and
/// `headroom` from `free`, saturating at `0`. `None` means the ring is not
/// backpressure-controlled, so the budget is unbounded (`u64::MAX`).
pub fn direct_budget_records(free: Option<u64>, deferred_nodes: u64, headroom: u64) -> u64 {
    match free {
        None => u64::MAX,
        Some(free) => free.saturating_sub(deferred_nodes).saturating_sub(headroom),
    }
}

fn overflow_skip_detail(budget_records: u64, whole_ring_budget: u64) -> &'static str {
    if budget_records >= whole_ring_budget {
        DETAIL_STATE_LARGER_THAN_RING
    } else {
        DETAIL_RECORDER_BEHIND
    }
}

/// Observable summary of one boundary's direct capture stream.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct DirectStreamStats {
    /// Deferred nodes captured completely into the state ring.
    pub streamed_nodes: u32,
    /// Deferred nodes refused by the ring budget, a full ring, or the overflow memo.
    pub refused_nodes: u32,
    /// Deferred nodes that failed capture (missing shape, prior inline failure,
    /// direct encoder error, or caught panic).
    pub failed_nodes: u32,
    /// Deferred nodes whose encoder was not entered during this direct stream.
    pub not_entered_nodes: u32,
    /// Total state-ring records pushed by this direct stream.
    pub records: u64,
}

/// Per-runtime state for the direct capture carrier.
#[derive(Debug, Default)]
pub struct DirectCarrier {
    overflow_memo: Vec<(u32, u64)>,
    last: Option<DirectStreamStats>,
}

impl DirectCarrier {
    /// Create a fresh carrier with an empty overflow memo and no recorded stats.
    pub fn new() -> Self {
        Self::default()
    }

    /// Stats recorded by the most recent [`stream_deferred_direct`] call.
    pub fn last_stats(&self) -> Option<DirectStreamStats> {
        self.last
    }

    /// Current `(node_idx, failed_budget_records)` overflow memo entries.
    pub fn overflow_memo(&self) -> &[(u32, u64)] {
        &self.overflow_memo
    }

    fn memo_budget(&self, node_idx: u32) -> Option<u64> {
        self.overflow_memo
            .iter()
            .find_map(|&(idx, budget)| (idx == node_idx).then_some(budget))
    }

    fn record_overflow(&mut self, node_idx: u32, budget_records: u64) {
        if let Some((_, existing)) = self
            .overflow_memo
            .iter_mut()
            .find(|(idx, _)| *idx == node_idx)
        {
            *existing = (*existing).max(budget_records);
        } else {
            self.overflow_memo.push((node_idx, budget_records));
        }
    }

    fn clear_overflow(&mut self, node_idx: u32) {
        self.overflow_memo.retain(|&(idx, _)| idx != node_idx);
    }
}

enum NodeOutcome {
    Streamed,
    Refused { entered: bool },
    Failed { entered: bool, detail: &'static str },
}

/// Stream every eligible node in `deferred` directly into `producer` without
/// blocking, in `deferred` order.
///
/// Returns `(node_idx, cause, detail)` for every node that did not produce a
/// complete anchor; pushes no SKIP records itself so the caller can publish
/// them after dropping node mutex guards.
pub fn stream_deferred_direct<T: DirectCaptureTarget + ?Sized>(
    carrier: &mut DirectCarrier,
    producer: &mut StateRingProducer,
    step: u64,
    headroom_records: u64,
    deferred: &[DirectDeferredNode<'_, T>],
) -> Vec<(u32, SkipCause, &'static str)> {
    let deferred_count = deferred.len() as u64;
    let mut remaining_budget =
        direct_budget_records(producer.free_records(), deferred_count, headroom_records);
    let whole_ring_budget =
        direct_budget_records(Some(producer.capacity()), deferred_count, headroom_records);

    let mut stats = DirectStreamStats::default();
    let mut failures = Vec::new();

    for node in deferred {
        let shape = match direct_disposition(node.reason, node.shape) {
            DirectDisposition::Stream { shape } => shape,
            DirectDisposition::SkipWithoutEntry { cause, detail } => {
                stats.failed_nodes += 1;
                stats.not_entered_nodes += 1;
                failures.push((node.node_idx, cause, detail));
                continue;
            }
        };

        if let Some(recorded) = carrier.memo_budget(node.node_idx) {
            if remaining_budget <= recorded {
                stats.refused_nodes += 1;
                stats.not_entered_nodes += 1;
                failures.push((
                    node.node_idx,
                    SkipCause::RecorderBehind,
                    overflow_skip_detail(recorded, whole_ring_budget),
                ));
                continue;
            }
        }

        if remaining_budget == 0 {
            carrier.record_overflow(node.node_idx, 0);
            stats.refused_nodes += 1;
            stats.not_entered_nodes += 1;
            failures.push((
                node.node_idx,
                SkipCause::RecorderBehind,
                overflow_skip_detail(0, whole_ring_budget),
            ));
            continue;
        }

        let node_budget = remaining_budget;
        let pushed_before = producer.pushed();

        let outcome = {
            let mut sink = producer.try_sink(step, node.node_idx, node_budget);
            let header_result = if node.framework.is_empty() {
                AnchorBlob::write_header(shape, &mut sink)
            } else {
                AnchorBlob::write_header_with_framework(shape, node.framework, &mut sink)
            };
            if let Err(e) = header_result {
                if matches!(e, StateError::SinkFull) {
                    NodeOutcome::Refused { entered: false }
                } else {
                    NodeOutcome::Failed {
                        entered: false,
                        detail: DETAIL_DIRECT_ENCODER_ERROR,
                    }
                }
            } else {
                // Caller holds node mutex guards outside this call, so catching
                // a panic here keeps the guard from unwinding and poisoning.
                let captured = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                    node.target.capture_direct(&mut sink)
                }));
                match captured {
                    Ok(Ok(())) if !sink.refused() => match sink.finish() {
                        Ok(_parts) => NodeOutcome::Streamed,
                        Err(_) => NodeOutcome::Refused { entered: true },
                    },
                    // Refused sink drops without finish so a partial stream is
                    // never published as complete.
                    Ok(Ok(())) => NodeOutcome::Refused { entered: true },
                    Ok(Err(e)) if matches!(e, StateError::SinkFull) || sink.refused() => {
                        NodeOutcome::Refused { entered: true }
                    }
                    Ok(Err(_)) => NodeOutcome::Failed {
                        entered: true,
                        detail: DETAIL_DIRECT_ENCODER_ERROR,
                    },
                    Err(_) => NodeOutcome::Failed {
                        entered: true,
                        detail: DETAIL_DIRECT_CAPTURE_UNWOUND,
                    },
                }
            }
        };

        let emitted = producer.pushed().saturating_sub(pushed_before);
        remaining_budget = remaining_budget.saturating_sub(emitted);
        stats.records += emitted;

        match outcome {
            NodeOutcome::Streamed => {
                stats.streamed_nodes += 1;
                carrier.clear_overflow(node.node_idx);
            }
            NodeOutcome::Refused { entered } => {
                if !entered {
                    stats.not_entered_nodes += 1;
                }
                stats.refused_nodes += 1;
                carrier.record_overflow(node.node_idx, node_budget);
                failures.push((
                    node.node_idx,
                    SkipCause::RecorderBehind,
                    overflow_skip_detail(node_budget, whole_ring_budget),
                ));
            }
            NodeOutcome::Failed { entered, detail } => {
                if !entered {
                    stats.not_entered_nodes += 1;
                }
                stats.failed_nodes += 1;
                failures.push((node.node_idx, SkipCause::CaptureFailed, detail));
            }
        }
    }

    carrier.last = Some(stats);
    failures
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::state_restore::{ANCHOR_BLOB_MAGIC, ANCHOR_BLOB_MAGIC_V2};
    use crate::state_ring::{
        encode_record, encode_skip_record, StateAnchorEvent, StateAssembler, StateRecordHeader,
        StateRingConsumer, StateRingOwner, RECORD_KIND_CHUNK_V2, RECORD_KIND_FINAL_V2,
        STATE_RECORD_FORMAT_VERSION, STATE_RECORD_PAYLOAD, STATE_RECORD_SIZE,
    };
    use std::cell::Cell;
    use std::sync::atomic::{AtomicU64, Ordering};
    use std::sync::Mutex;

    static TAG_SEQ: AtomicU64 = AtomicU64::new(0);

    fn unique_tag(label: &str) -> String {
        let n = TAG_SEQ.fetch_add(1, Ordering::Relaxed);
        format!("cer_direct_{label}_{}_{n}", std::process::id())
    }

    #[derive(Clone, Copy)]
    enum Mode {
        Bytes(usize),
        StreamedBlocks(usize),
        Error,
        Panic,
        SwallowRefusal(usize),
    }

    struct SynthTarget {
        mode: Mode,
        seed: u8,
        entered: Cell<u32>,
    }

    impl SynthTarget {
        fn new(mode: Mode, seed: u8) -> Self {
            Self {
                mode,
                seed,
                entered: Cell::new(0),
            }
        }
    }

    impl DirectCaptureTarget for SynthTarget {
        fn capture_direct(&self, sink: &mut dyn StateSink) -> Result<(), StateError> {
            self.entered.set(self.entered.get() + 1);
            match self.mode {
                Mode::Bytes(len) => {
                    let payload: Vec<u8> = (0..len)
                        .map(|i| self.seed.wrapping_add((i % 251) as u8))
                        .collect();
                    sink.write(&payload)?;
                    Ok(())
                }
                Mode::StreamedBlocks(blocks) => {
                    let block = [self.seed; STATE_RECORD_PAYLOAD];
                    for _ in 0..blocks {
                        sink.write(&block)?;
                    }
                    Ok(())
                }
                Mode::Error => Err(StateError::NonUtf8Path),
                Mode::Panic => panic!("synthetic direct encoder panic"),
                Mode::SwallowRefusal(len) => {
                    let _ = sink.write(&vec![self.seed; len]);
                    Ok(())
                }
            }
        }
    }

    fn deferred_node<'a, T: ?Sized>(
        node_idx: u32,
        reason: ForkReason,
        shape: Option<u64>,
        framework: &'a NodeFrameworkState,
        target: &'a T,
    ) -> DirectDeferredNode<'a, T> {
        DirectDeferredNode {
            node_idx,
            reason,
            shape,
            framework,
            target,
        }
    }

    fn drain_raw(consumer: &mut StateRingConsumer) -> Vec<[u8; STATE_RECORD_SIZE as usize]> {
        const RS: usize = STATE_RECORD_SIZE as usize;
        let (a, b) = consumer.drain_slices().expect("no lap");
        let recs: Vec<[u8; RS]> = a
            .as_chunks::<RS>()
            .0
            .iter()
            .chain(b.as_chunks::<RS>().0)
            .copied()
            .collect();
        consumer.commit(recs.len() as u64).expect("commit");
        recs
    }

    fn drain_events(consumer: &mut StateRingConsumer) -> Vec<StateAnchorEvent> {
        let mut asm = StateAssembler::passthrough();
        let mut events = Vec::new();
        consumer.drain(&mut asm, &mut events).expect("drain");
        events.extend(asm.finish());
        events
    }

    #[test]
    fn direct_disposition_oracle_table_covers_every_arm() {
        let skip = |detail| DirectDisposition::SkipWithoutEntry {
            cause: SkipCause::CaptureFailed,
            detail,
        };
        let cases: &[(ForkReason, Option<u64>, DirectDisposition)] = &[
            (
                ForkReason::NotInlineSafe,
                Some(0x1111),
                DirectDisposition::Stream { shape: 0x1111 },
            ),
            (
                ForkReason::ArenaOverflow,
                Some(0x2222),
                DirectDisposition::Stream { shape: 0x2222 },
            ),
            (ForkReason::NotInlineSafe, None, skip(DETAIL_NO_STATE_SHAPE)),
            (ForkReason::ArenaOverflow, None, skip(DETAIL_NO_STATE_SHAPE)),
            (
                ForkReason::EncoderError,
                Some(0x3333),
                skip(DETAIL_INLINE_ENCODER_ERROR),
            ),
            (
                ForkReason::EncoderError,
                None,
                skip(DETAIL_INLINE_ENCODER_ERROR),
            ),
            (
                ForkReason::CaptureUnwound,
                Some(0x4444),
                skip(DETAIL_INLINE_CAPTURE_UNWOUND),
            ),
            (
                ForkReason::CaptureUnwound,
                None,
                skip(DETAIL_INLINE_CAPTURE_UNWOUND),
            ),
        ];
        for &(reason, shape, expected) in cases {
            assert_eq!(
                direct_disposition(reason, shape),
                expected,
                "reason={reason:?}, shape={shape:?}"
            );
        }
    }

    #[test]
    fn direct_budget_records_oracle_table_covers_none_and_saturation() {
        let cases: &[(Option<u64>, u64, u64, u64)] = &[
            (None, 0, 0, u64::MAX),
            (None, 5, 64, u64::MAX),
            (Some(100), 3, 64, 33),
            (Some(67), 3, 64, 0),
            (Some(66), 3, 64, 0),
            (Some(0), 1, 0, 0),
            (Some(16), 0, 0, 16),
            (Some(10), u64::MAX, u64::MAX, 0),
        ];
        for &(free, deferred, headroom, expected) in cases {
            assert_eq!(
                direct_budget_records(free, deferred, headroom),
                expected,
                "free={free:?}, deferred={deferred}, headroom={headroom}"
            );
        }
    }

    #[test]
    fn stream_deferred_direct_writes_v1_and_v2_anchor_blobs_matching_hand_oracles() {
        let tag = unique_tag("v1v2");
        let mut owner =
            StateRingOwner::create(&tag, 16, 1, 0xA0, &["n0", "n1"]).expect("create ring");
        let name = owner.name().to_string();
        let mut producer = owner.producer().expect("producer");
        let mut carrier = DirectCarrier::new();

        // Empty deferred set records zero stats and emits nothing.
        let empty: [DirectDeferredNode<'_, SynthTarget>; 0] = [];
        assert!(stream_deferred_direct(&mut carrier, &mut producer, 4, 2, &empty).is_empty());
        assert_eq!(carrier.last_stats(), Some(DirectStreamStats::default()));

        let empty_fw = NodeFrameworkState::default();
        let period_fw = NodeFrameworkState {
            next_fire_ns: Some(42_000),
            pending_data_count: 3,
            ..NodeFrameworkState::default()
        };
        let t0 = SynthTarget::new(Mode::Bytes(12), 0x10);
        let t1 = SynthTarget::new(Mode::Bytes(20), 0x80);
        let deferred = [
            deferred_node(
                0,
                ForkReason::NotInlineSafe,
                Some(0xDEAD_BEEF),
                &empty_fw,
                &t0,
            ),
            deferred_node(
                1,
                ForkReason::ArenaOverflow,
                Some(0xCAFE_BABE),
                &period_fw,
                &t1,
            ),
        ];

        let failures = stream_deferred_direct(&mut carrier, &mut producer, 5, 2, &deferred);
        assert!(failures.is_empty());
        assert_eq!(producer.backpressure_waits(), 0);
        assert_eq!((t0.entered.get(), t1.entered.get()), (1, 1));
        assert_eq!(
            carrier.last_stats(),
            Some(DirectStreamStats {
                streamed_nodes: 2,
                refused_nodes: 0,
                failed_nodes: 0,
                not_entered_nodes: 0,
                records: 2,
            })
        );

        let mut oracle_v1 = Vec::new();
        oracle_v1.extend_from_slice(&ANCHOR_BLOB_MAGIC.to_le_bytes());
        oracle_v1.extend_from_slice(&0xDEAD_BEEFu64.to_le_bytes());
        oracle_v1.extend((0..12usize).map(|i| 0x10u8.wrapping_add(i as u8)));

        let mut oracle_v2 = Vec::new();
        oracle_v2.extend_from_slice(&ANCHOR_BLOB_MAGIC_V2.to_le_bytes());
        oracle_v2.extend_from_slice(&0xCAFE_BABEu64.to_le_bytes());
        oracle_v2.extend_from_slice(&25u32.to_le_bytes());
        oracle_v2.extend_from_slice(&1u32.to_le_bytes());
        oracle_v2.push(1u8);
        oracle_v2.extend_from_slice(&42_000u64.to_le_bytes());
        oracle_v2.extend_from_slice(&3u64.to_le_bytes());
        oracle_v2.extend_from_slice(&0u32.to_le_bytes());
        oracle_v2.extend((0..20usize).map(|i| 0x80u8.wrapping_add(i as u8)));

        let mut consumer = StateRingConsumer::open(&name).expect("open");
        assert_eq!(
            drain_events(&mut consumer),
            vec![
                StateAnchorEvent::Complete {
                    run_id: 0xA0,
                    step: 5,
                    node_idx: 0,
                    parts: 1,
                    bytes: oracle_v1,
                },
                StateAnchorEvent::Complete {
                    run_id: 0xA0,
                    step: 5,
                    node_idx: 1,
                    parts: 1,
                    bytes: oracle_v2,
                },
            ]
        );
    }

    #[test]
    fn stream_deferred_direct_never_enters_disqualified_targets_and_catches_panics_and_errors() {
        let tag = unique_tag("errors");
        let mut owner = StateRingOwner::create(
            &tag,
            16,
            0,
            0xB1,
            &["n0", "n1", "n2", "n3", "n4", "n5", "n6"],
        )
        .expect("create ring");
        let name = owner.name().to_string();
        let mut producer = owner.producer().expect("producer");
        let mut carrier = DirectCarrier::new();
        let empty_fw = NodeFrameworkState::default();

        let t_enc_err = SynthTarget::new(Mode::Bytes(8), 1);
        let t_unwound = SynthTarget::new(Mode::Bytes(8), 2);
        let t_shapeless = SynthTarget::new(Mode::Bytes(8), 3);
        let t_direct_err = SynthTarget::new(Mode::Error, 0);
        let t_direct_panic = Mutex::new(SynthTarget::new(Mode::Panic, 0));
        let t_swallow = SynthTarget::new(Mode::SwallowRefusal(STATE_RECORD_PAYLOAD * 20), 0x5A);
        let t_survivor = SynthTarget::new(Mode::Bytes(6), 0x40);

        let failures = {
            let panic_guard = t_direct_panic.lock().expect("lock");
            let deferred = [
                deferred_node(
                    0,
                    ForkReason::EncoderError,
                    Some(1),
                    &empty_fw,
                    &t_enc_err as &dyn DirectCaptureTarget,
                ),
                deferred_node(
                    1,
                    ForkReason::CaptureUnwound,
                    Some(2),
                    &empty_fw,
                    &t_unwound,
                ),
                deferred_node(2, ForkReason::NotInlineSafe, None, &empty_fw, &t_shapeless),
                deferred_node(
                    3,
                    ForkReason::NotInlineSafe,
                    Some(3),
                    &empty_fw,
                    &t_direct_err,
                ),
                deferred_node(
                    4,
                    ForkReason::NotInlineSafe,
                    Some(4),
                    &empty_fw,
                    &*panic_guard,
                ),
                deferred_node(5, ForkReason::ArenaOverflow, Some(5), &empty_fw, &t_swallow),
                deferred_node(
                    6,
                    ForkReason::NotInlineSafe,
                    Some(6),
                    &empty_fw,
                    &t_survivor,
                ),
            ];
            stream_deferred_direct(&mut carrier, &mut producer, 10, 1, &deferred)
        };

        assert!(!t_direct_panic.is_poisoned());
        assert_eq!(
            [
                t_enc_err.entered.get(),
                t_unwound.entered.get(),
                t_shapeless.entered.get(),
                t_direct_err.entered.get(),
                t_direct_panic.lock().unwrap().entered.get(),
                t_swallow.entered.get(),
                t_survivor.entered.get(),
            ],
            [0, 0, 0, 1, 1, 1, 1]
        );
        assert_eq!(producer.backpressure_waits(), 0);
        assert_eq!(
            failures,
            vec![
                (0, SkipCause::CaptureFailed, DETAIL_INLINE_ENCODER_ERROR),
                (1, SkipCause::CaptureFailed, DETAIL_INLINE_CAPTURE_UNWOUND),
                (2, SkipCause::CaptureFailed, DETAIL_NO_STATE_SHAPE),
                (3, SkipCause::CaptureFailed, DETAIL_DIRECT_ENCODER_ERROR),
                (4, SkipCause::CaptureFailed, DETAIL_DIRECT_CAPTURE_UNWOUND),
                (5, SkipCause::RecorderBehind, DETAIL_STATE_LARGER_THAN_RING),
            ]
        );
        assert_eq!(
            carrier.last_stats(),
            Some(DirectStreamStats {
                streamed_nodes: 1,
                refused_nodes: 1,
                failed_nodes: 5,
                not_entered_nodes: 3,
                records: 1,
            })
        );

        let mut consumer = StateRingConsumer::open(&name).expect("open");
        let events = drain_events(&mut consumer);
        assert_eq!(events.len(), 1);
        assert!(matches!(
            events[0],
            StateAnchorEvent::Complete { node_idx: 6, .. }
        ));
    }

    #[test]
    fn overflow_memo_skips_repeat_refusals_handles_zero_budget_and_clears_on_success() {
        let tag = unique_tag("memo");
        let mut owner = StateRingOwner::create(&tag, 8, 0, 0xC2, &["n0"]).expect("create ring");
        let name = owner.name().to_string();
        let mut producer = owner.producer().expect("producer");
        let mut consumer = StateRingConsumer::open(&name).expect("open");
        let mut carrier = DirectCarrier::new();
        let empty_fw = NodeFrameworkState::default();

        // Target needs 3 records (16-byte header + 1000-byte payload = 1016 B).
        let medium = SynthTarget::new(Mode::Bytes(1_000), 0x33);
        let deferred = [deferred_node(
            0,
            ForkReason::ArenaOverflow,
            Some(0x99),
            &empty_fw,
            &medium,
        )];

        // Full ring -> budget 0 -> skipped without entering encoder, memoized at 0.
        let prefill = encode_skip_record(0xC2, 1, 0, 0, SkipCause::Contended, "pad");
        for _ in 0..8 {
            assert!(producer.try_push_record(&prefill));
        }
        let f0 = stream_deferred_direct(&mut carrier, &mut producer, 1, 0, &deferred);
        assert_eq!(medium.entered.get(), 0);
        assert_eq!(
            f0,
            vec![(0, SkipCause::RecorderBehind, DETAIL_RECORDER_BEHIND)]
        );
        assert_eq!(carrier.overflow_memo(), &[(0, 0)]);

        // Drain and refill 5 of 8 slots -> budget 2 (> 0, < 3 needed, < whole-ring 7).
        let _ = drain_events(&mut consumer);
        for _ in 0..5 {
            assert!(producer.try_push_record(&prefill));
        }
        let f1 = stream_deferred_direct(&mut carrier, &mut producer, 2, 0, &deferred);
        assert_eq!(medium.entered.get(), 1);
        assert_eq!(
            f1,
            vec![(0, SkipCause::RecorderBehind, DETAIL_RECORDER_BEHIND)]
        );
        assert_eq!(carrier.overflow_memo(), &[(0, 2)]);

        // Same budget 2 -> memo hit skips without entering encoder.
        let f2 = stream_deferred_direct(&mut carrier, &mut producer, 3, 0, &deferred);
        assert_eq!(medium.entered.get(), 1);
        assert_eq!(
            f2,
            vec![(0, SkipCause::RecorderBehind, DETAIL_RECORDER_BEHIND)]
        );
        assert_eq!(carrier.last_stats().unwrap().not_entered_nodes, 1);

        // Drain -> budget 7 > 2 -> re-enters, succeeds, and clears memo.
        let _ = drain_events(&mut consumer);
        let f3 = stream_deferred_direct(&mut carrier, &mut producer, 4, 0, &deferred);
        assert!(f3.is_empty());
        assert_eq!(medium.entered.get(), 2);
        assert!(carrier.overflow_memo().is_empty());

        // Giant node larger than the whole 8-record ring names the env knob and stays memoized.
        let _ = drain_events(&mut consumer);
        let giant = SynthTarget::new(Mode::Bytes(STATE_RECORD_PAYLOAD * 10), 0x77);
        let giant_deferred = [deferred_node(
            0,
            ForkReason::ArenaOverflow,
            Some(0xAA),
            &empty_fw,
            &giant,
        )];
        let f4 = stream_deferred_direct(&mut carrier, &mut producer, 5, 0, &giant_deferred);
        assert_eq!(giant.entered.get(), 1);
        assert_eq!(
            f4,
            vec![(0, SkipCause::RecorderBehind, DETAIL_STATE_LARGER_THAN_RING)]
        );
        assert!(f4[0].2.contains("CERULION_FLASHBACK_STATE_RING_MB"));
        assert_eq!(carrier.overflow_memo(), &[(0, 7)]);

        let f5 = stream_deferred_direct(&mut carrier, &mut producer, 6, 0, &giant_deferred);
        assert_eq!(giant.entered.get(), 1);
        assert_eq!(
            f5,
            vec![(0, SkipCause::RecorderBehind, DETAIL_STATE_LARGER_THAN_RING)]
        );
        assert_eq!(producer.backpressure_waits(), 0);
    }

    #[test]
    fn stream_deferred_direct_two_runs_emit_bit_identical_records() {
        let run_once = |label: &str| -> Vec<[u8; STATE_RECORD_SIZE as usize]> {
            let tag = unique_tag(label);
            let mut owner =
                StateRingOwner::create(&tag, 16, 2, 0xD3, &["n0", "n1"]).expect("create ring");
            let name = owner.name().to_string();
            let mut producer = owner.producer().expect("producer");
            let mut carrier = DirectCarrier::new();
            let empty_fw = NodeFrameworkState::default();
            let period_fw = NodeFrameworkState {
                next_fire_ns: Some(1_234),
                ..NodeFrameworkState::default()
            };
            let t0 = SynthTarget::new(Mode::Bytes(600), 0x11);
            let t1 = SynthTarget::new(Mode::Bytes(50), 0x22);
            let deferred = [
                deferred_node(0, ForkReason::NotInlineSafe, Some(0x1234), &empty_fw, &t0),
                deferred_node(1, ForkReason::ArenaOverflow, Some(0x5678), &period_fw, &t1),
            ];
            assert!(
                stream_deferred_direct(&mut carrier, &mut producer, 8, 1, &deferred).is_empty()
            );
            assert_eq!(producer.backpressure_waits(), 0);
            let mut consumer = StateRingConsumer::open(&name).expect("open");
            drain_raw(&mut consumer)
        };

        // Hand oracle. Node 0: v1 blob = magic, shape, 600 payload bytes (616 B,
        // two records). Node 1: v2 blob = magic, shape, a 25-byte framework
        // section (version 1, next_fire present = 1234, pending 0, no sync
        // inputs), 50 payload bytes (95 B, one record).
        let mut blob0 = Vec::new();
        blob0.extend_from_slice(&ANCHOR_BLOB_MAGIC.to_le_bytes());
        blob0.extend_from_slice(&0x1234u64.to_le_bytes());
        blob0.extend((0..600usize).map(|i| 0x11u8.wrapping_add((i % 251) as u8)));
        let mut blob1 = Vec::new();
        blob1.extend_from_slice(&ANCHOR_BLOB_MAGIC_V2.to_le_bytes());
        blob1.extend_from_slice(&0x5678u64.to_le_bytes());
        blob1.extend_from_slice(&25u32.to_le_bytes());
        blob1.extend_from_slice(&1u32.to_le_bytes());
        blob1.push(1u8);
        blob1.extend_from_slice(&1_234u64.to_le_bytes());
        blob1.extend_from_slice(&0u64.to_le_bytes());
        blob1.extend_from_slice(&0u32.to_le_bytes());
        blob1.extend((0..50usize).map(|i| 0x22u8.wrapping_add(i as u8)));
        assert_eq!((blob0.len(), blob1.len()), (616, 95));
        let record = |node_idx: u32, part: u32, kind: u32, payload: &[u8]| {
            encode_record(
                &StateRecordHeader {
                    run_id: 0xD3,
                    step: 8,
                    node_idx,
                    part,
                    kind,
                    len: payload.len() as u32,
                    rank: 2,
                    format_version: STATE_RECORD_FORMAT_VERSION,
                },
                payload,
            )
        };
        let oracle = vec![
            record(0, 0, RECORD_KIND_CHUNK_V2, &blob0[..472]),
            record(0, 1, RECORD_KIND_FINAL_V2, &blob0[472..]),
            record(1, 0, RECORD_KIND_FINAL_V2, &blob1),
        ];

        let run_a = run_once("det_a");
        let run_b = run_once("det_b");
        assert_eq!(run_a, oracle);
        assert_eq!(run_b, oracle);
    }

    #[test]
    fn truncated_direct_stream_leaves_room_for_skip_record_that_voids_it() {
        let tag = unique_tag("voidskip");
        let mut owner = StateRingOwner::create(&tag, 8, 0, 0xE4, &["n0"]).expect("create ring");
        let name = owner.name().to_string();
        let mut producer = owner.producer().expect("producer");
        let mut carrier = DirectCarrier::new();
        let empty_fw = NodeFrameworkState::default();

        let target = SynthTarget::new(Mode::StreamedBlocks(9), 0x2A);
        let deferred = [deferred_node(
            0,
            ForkReason::ArenaOverflow,
            Some(0xFACE),
            &empty_fw,
            &target,
        )];

        let failures = stream_deferred_direct(&mut carrier, &mut producer, 12, 0, &deferred);
        assert_eq!(
            failures,
            vec![(0, SkipCause::RecorderBehind, DETAIL_STATE_LARGER_THAN_RING)]
        );
        assert!(carrier.last_stats().unwrap().records > 0);
        assert!(producer.free_records().unwrap() >= 1);

        for (node_idx, cause, detail) in &failures {
            assert!(producer.try_push_skip(12, *node_idx, *cause, detail));
        }
        assert_eq!(producer.backpressure_waits(), 0);

        let mut consumer = StateRingConsumer::open(&name).expect("open");
        assert_eq!(
            drain_events(&mut consumer),
            vec![StateAnchorEvent::Skipped {
                run_id: 0xE4,
                step: 12,
                node_idx: 0,
                cause: SkipCause::RecorderBehind,
                detail: DETAIL_STATE_LARGER_THAN_RING.to_string(),
            }]
        );
    }
}
