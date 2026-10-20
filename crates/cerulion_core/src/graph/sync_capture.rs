// SPDX-License-Identifier: AGPL-3.0-only
//! Graph-build wiring for capture-time (`header.stamp`) Sync matching.
//!
//! The decisions live in [`crate::scheduler::sync_stamp`]; this module is the
//! glue the graph runtime needs around them: refuse a node that opted in but
//! cannot match on capture time, and run each head op of an opted-in node
//! through [`capture_head_op`] while it holds the node lock, counting every
//! discarded frame on the node's [`SyncCounters`].

use std::collections::HashSet;
use std::sync::Mutex;

use indexmap::IndexMap;

use crate::error::{TransportError, TransportResult};
use crate::scheduler::handle::SyncCounters;
use crate::scheduler::sync_stamp::{
    capture_head_op, CaptureIo, HeaderStampLocation, StampDecodeError, SyncFrameSlot,
};
use crate::scheduler::{SyncHeadOp, SyncOpAnswer};
use crate::transport::failure_regime_latch::{FailureRegimeLatch, RegimeDecision};

use super::node::{MacroPolicy, NodeEntry, NodeInfo};

/// What a capture-time Sync node needs at run time: the stamp location of
/// each trigger input (by input name) and the node's window in nanoseconds
/// (`None` for an unbounded node).
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct CaptureSpec {
    pub(crate) by_input: IndexMap<String, HeaderStampLocation>,
    pub(crate) window_ns: Option<u64>,
}

/// Decide, at graph build, whether this node matches on capture time, and
/// refuse it loudly if it opted in but cannot.
///
/// `per_set_sync` is the node's per-set Sync capability (head ops installed);
/// `supports_capture` is [`NodeEntry::supports_sync_capture_stamps`]. Returns
/// `Ok(None)` for a node that did not opt in, which keeps publish-time
/// matching exactly as before.
// hot-path-alloc-ok-fn: cold: graph BUILD, once per node, before the first step
pub(crate) fn validate_sync_capture(
    node_id: &str,
    info: &NodeInfo,
    trigger_names: &HashSet<&str>,
    per_set_sync: bool,
    supports_capture: bool,
) -> TransportResult<Option<CaptureSpec>> {
    let Some(locations) = info.sync_capture_stamps() else {
        return Ok(None);
    };
    let window_ns = match info.policy.as_ref() {
        Some(MacroPolicy::Sync { window_ms }) => Some(window_ms.saturating_mul(1_000_000)),
        Some(MacroPolicy::UnboundedSync) => None,
        _ => {
            return Err(TransportError::GraphError {
                reason: format!(
                    "node '{node_id}' asks to match on header.stamp, but only a sync node \
                     (`sync_window_ms` or `unbounded_sync`) aligns its inputs; remove the \
                     header.stamp opt-in or give the node a sync policy"
                ),
            })
        }
    };
    if !per_set_sync || !supports_capture {
        return Err(TransportError::GraphError {
            reason: format!(
                "node '{node_id}' asks to match on header.stamp, but this node kind cannot \
                 expose its frames to the sync matcher (a raw-FFI or prebuilt library, or \
                 CERULION_DRAIN_DISCIPLINE=separate); rebuild the node crate against this \
                 Cerulion, or match this node on publish time"
            ),
        });
    }
    // Every trigger input needs a location, in the node's declaration order
    // so the error names the first one a reader would look at.
    let mut by_input = IndexMap::new();
    for meta in &info.input_meta {
        if !trigger_names.contains(meta.name.as_str()) {
            continue;
        }
        match locations.get(&meta.name) {
            Some(location) => {
                by_input.insert(meta.name.clone(), *location);
            }
            None => {
                return Err(TransportError::SyncCaptureStamp {
                    node_id: node_id.to_string(),
                    input: meta.name.clone(),
                    reason: "no header.stamp location was declared for it".to_string(),
                })
            }
        }
    }
    Ok(Some(CaptureSpec {
        by_input,
        window_ns,
    }))
}

/// Run-time state of one capture-time trigger input, owned by the node's
/// sync-ops closure and touched only under the node lock.
pub(crate) struct CapturePort {
    location: HeaderStampLocation,
    /// The node's window in nanoseconds, `None` when unbounded.
    window_ns: Option<u64>,
    /// The newest capture stamp this input contributed as a head.
    high_water: Mutex<Option<u64>>,
    /// Flood guard for the decode-failure warning.
    decode_latch: Mutex<FailureRegimeLatch>,
}

impl CapturePort {
    pub(crate) fn new(location: HeaderStampLocation, window_ns: Option<u64>) -> Self {
        Self {
            location,
            window_ns,
            high_water: Mutex::new(None),
            decode_latch: Mutex::new(FailureRegimeLatch::new()),
        }
    }
}

/// [`CaptureIo`] over a locked node entry: ops go through
/// [`NodeEntry::sync_head_op`], stamps through
/// [`NodeEntry::sync_capture_stamp`].
struct EntryIo<'a> {
    entry: &'a mut dyn NodeEntry,
    port: &'a str,
    location: HeaderStampLocation,
}

impl CaptureIo for EntryIo<'_> {
    fn op(&mut self, op: SyncHeadOp) -> SyncOpAnswer {
        self.entry.sync_head_op(self.port, op)
    }

    fn stamp(&mut self, slot: SyncFrameSlot) -> Option<Result<u64, StampDecodeError>> {
        self.entry
            .sync_capture_stamp(self.port, slot, self.location)
    }
}

/// Perform one head op for a capture-time input and count what it discarded.
///
/// `topic` is the resolved source topic (the key the counters use), `port`
/// the node's input name. Allocation-free: the counters are pre-registered
/// per input at `add_node`, and both mutexes are uncontended under the node
/// lock.
pub(crate) fn capture_op(
    node_id: &str,
    topic: &str,
    port: &str,
    state: &CapturePort,
    counters: &SyncCounters,
    entry: &mut dyn NodeEntry,
    op: SyncHeadOp,
) -> SyncOpAnswer {
    let mut io = EntryIo {
        entry,
        port,
        location: state.location,
    };
    let outcome = {
        let mut high_water = match state.high_water.lock() {
            Ok(g) => g,
            Err(poisoned) => poisoned.into_inner(),
        };
        capture_head_op(op, state.window_ns, &mut high_water, &mut io)
    };
    if outcome.backwards_drops > 0 {
        let total = SyncCounters::bump(
            &counters.capture_backwards_drops,
            topic,
            u64::from(outcome.backwards_drops),
        );
        tracing::debug!(
            node_id = %node_id,
            sync_input = %topic,
            dropped = outcome.backwards_drops,
            total_failures = total,
            "sync capture-time matching dropped frames whose header.stamp went backwards"
        );
    }
    let mut latch = match state.decode_latch.lock() {
        Ok(g) => g,
        Err(poisoned) => poisoned.into_inner(),
    };
    if outcome.decode_failures > 0 {
        let total = SyncCounters::bump(
            &counters.capture_decode_failures,
            topic,
            u64::from(outcome.decode_failures),
        );
        match latch.on_failure() {
            RegimeDecision::Loud => tracing::warn!(
                node_id = %node_id,
                sync_input = %topic,
                error = ?outcome.last_decode_error,
                total_failures = total,
                "sync capture-time matching discarded a frame whose header.stamp could not \
                 be decoded; the frame is in no set (counted as \
                 `sync_capture_decode_failures`). Check that the producer writes a \
                 complete header"
            ),
            RegimeDecision::StillFailing { total, suppressed } => tracing::warn!(
                node_id = %node_id,
                sync_input = %topic,
                total_failures = total,
                suppressed,
                "sync capture-time stamp decode failures still recurring"
            ),
            RegimeDecision::Suppressed { suppressed } => tracing::debug!(
                node_id = %node_id,
                sync_input = %topic,
                suppressed,
                "sync capture-time stamp decode failure (suppressed)"
            ),
        }
    } else if matches!(outcome.answer, SyncOpAnswer::Head(_)) {
        if let Some(suppressed) = latch.on_success() {
            tracing::info!(
                node_id = %node_id,
                sync_input = %topic,
                suppressed_failures = suppressed,
                "sync capture-time stamps decode again on this input"
            );
        }
    }
    outcome.answer
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::graph::node::{BackpressurePolicy, InputMeta};

    fn meta(name: &str, trigger: bool) -> InputMeta {
        InputMeta {
            name: name.to_string(),
            schema_hash: 0,
            trigger,
            depth: 4,
            backpressure: BackpressurePolicy::DropOldest,
            expect_within_ms: None,
        }
    }

    const LOC: HeaderStampLocation = HeaderStampLocation::Table {
        fixed_size: 56,
        index: 0,
        stamp_offset: 0,
    };

    fn info(policy: MacroPolicy, locations: &[(&str, HeaderStampLocation)]) -> NodeInfo {
        NodeInfo::with_meta(
            vec![meta("cam", true), meta("lidar", true), meta("odom", false)],
            vec![],
        )
        .with_policy(policy)
        .with_sync_capture_stamps(
            locations
                .iter()
                .map(|(n, l)| ((*n).to_string(), *l))
                .collect(),
        )
    }

    fn triggers() -> HashSet<&'static str> {
        ["cam", "lidar"].into_iter().collect()
    }

    #[test]
    fn a_node_that_did_not_opt_in_keeps_publish_time() {
        let plain = NodeInfo::with_meta(vec![meta("cam", true)], vec![])
            .with_policy(MacroPolicy::Sync { window_ms: 10 });
        assert!(matches!(
            validate_sync_capture("fuse", &plain, &triggers(), false, false),
            Ok(None)
        ));
    }

    #[test]
    fn an_opted_in_sync_node_gets_every_trigger_location_and_its_window() {
        let i = info(
            MacroPolicy::Sync { window_ms: 20 },
            &[("lidar", LOC), ("cam", LOC), ("odom", LOC)],
        );
        let spec = validate_sync_capture("fuse", &i, &triggers(), true, true)
            .expect("valid")
            .expect("opted in");
        assert_eq!(spec.window_ns, Some(20_000_000));
        assert_eq!(
            spec.by_input.keys().map(String::as_str).collect::<Vec<_>>(),
            vec!["cam", "lidar"],
            "trigger inputs only, in declaration order"
        );
    }

    #[test]
    fn an_unbounded_node_has_no_window() {
        let i = info(MacroPolicy::UnboundedSync, &[("cam", LOC), ("lidar", LOC)]);
        let spec = validate_sync_capture("fuse", &i, &triggers(), true, true)
            .expect("valid")
            .expect("opted in");
        assert_eq!(spec.window_ns, None);
    }

    #[test]
    fn a_trigger_input_without_a_location_is_refused_by_name() {
        let i = info(MacroPolicy::Sync { window_ms: 20 }, &[("cam", LOC)]);
        match validate_sync_capture("fuse", &i, &triggers(), true, true) {
            Err(TransportError::SyncCaptureStamp { node_id, input, .. }) => {
                assert_eq!((node_id.as_str(), input.as_str()), ("fuse", "lidar"));
            }
            other => panic!("expected SyncCaptureStamp, got {other:?}"),
        }
    }

    #[test]
    fn a_non_sync_node_cannot_opt_in() {
        let i = info(MacroPolicy::Period { period_ms: 10 }, &[("cam", LOC)]);
        let err =
            validate_sync_capture("fuse", &i, &triggers(), true, true).expect_err("period node");
        assert!(err.to_string().contains("only a sync node"), "{err}");
    }

    #[test]
    fn a_node_without_frame_access_is_refused() {
        let i = info(
            MacroPolicy::Sync { window_ms: 20 },
            &[("cam", LOC), ("lidar", LOC)],
        );
        for (per_set, capture) in [(false, true), (true, false)] {
            let err = validate_sync_capture("fuse", &i, &triggers(), per_set, capture)
                .expect_err("no frame access");
            assert!(err.to_string().contains("node 'fuse'"), "{err}");
            assert!(err.to_string().contains("cannot"), "{err}");
        }
    }
}
