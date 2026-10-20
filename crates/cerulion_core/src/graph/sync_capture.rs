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
/// each trigger input (by input name) and the node's window in nanoseconds.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct CaptureSpec {
    pub(crate) by_input: IndexMap<String, HeaderStampLocation>,
    pub(crate) window_ns: u64,
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
    // Only a windowed node: the window is what tells an out-of-order frame
    // (dropped) apart from a capture clock that restarted (a new epoch). An
    // `unbounded_sync` node has no window, so a restart would read as one
    // long run of backwards frames and starve the node.
    let window_ns = match info.policy.as_ref() {
        Some(MacroPolicy::Sync { window_ms }) => window_ms.saturating_mul(1_000_000),
        _ => {
            return Err(TransportError::GraphError {
                reason: format!(
                    "node '{node_id}' asks to match on header.stamp, but only a \
                     `sync_window_ms` node can match on capture time; remove the \
                     header.stamp opt-in or give the node a `sync_window_ms` policy"
                ),
            })
        }
    };
    if !per_set_sync || !supports_capture {
        return Err(TransportError::GraphError {
            reason: format!(
                "node '{node_id}' asks to match on header.stamp, but this node kind cannot \
                 expose its frames to the sync matcher: capture-time matching is not yet \
                 available to `#[cerulion_node]` nodes (in-process or cdylib), raw-FFI \
                 libraries, or CERULION_DRAIN_DISCIPLINE=separate; \
                 remove the header.stamp opt-in"
            ),
        });
    }
    // A location for an input that is not a trigger input is a typo or a
    // stale entry: refuse it rather than drop it silently.
    for name in locations.keys() {
        if !trigger_names.contains(name.as_str()) {
            return Err(TransportError::SyncCaptureStamp {
                node_id: node_id.to_string(),
                input: name.clone(),
                reason: "it is not a trigger input of this node".to_string(),
            });
        }
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
    /// The node's window in nanoseconds.
    window_ns: u64,
    /// The newest capture stamp this input contributed as a head.
    high_water: Mutex<Option<u64>>,
    /// Flood guard for the decode-failure warning.
    decode_latch: Mutex<FailureRegimeLatch>,
    /// Flood guard for the backwards-drop warning.
    backwards_latch: Mutex<FailureRegimeLatch>,
}

impl CapturePort {
    pub(crate) fn new(location: HeaderStampLocation, window_ns: u64) -> Self {
        Self {
            location,
            window_ns,
            high_water: Mutex::new(None),
            decode_latch: Mutex::new(FailureRegimeLatch::new()),
            backwards_latch: Mutex::new(FailureRegimeLatch::new()),
        }
    }

    /// Seed the high-water from a head restored from a recording.
    ///
    /// A restored head is the newest stamp this input contributed when the
    /// recording was taken, which is exactly what the high-water held then.
    /// Without the seed, a resumed run would accept a frame the live run
    /// dropped as backwards, and the two runs would pick different sets.
    // hot-path-alloc-ok-fn: cold: state RESTORE, once at run start
    pub(crate) fn restore_high_water(&self, ts: u64) {
        *lock(&self.high_water) = Some(ts);
    }
}

/// Lock a mutex that is only touched under the node lock. A poisoned guard
/// still holds valid data (every write is a single assignment), so it is
/// taken rather than turned into a failure.
fn lock<T>(m: &Mutex<T>) -> std::sync::MutexGuard<'_, T> {
    match m.lock() {
        Ok(g) => g,
        Err(poisoned) => poisoned.into_inner(),
    }
}

/// One capture-time node's trigger topics and per-port states, aligned. A
/// `None` state is a port with no stamp location (a wiring desync: its ops
/// answer FAILED).
pub(crate) type NodeCapturePorts = (Vec<String>, std::sync::Arc<[Option<CapturePort>]>);

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
    let outcome = capture_head_op(op, state.window_ns, &mut lock(&state.high_water), &mut io);
    let accepted = matches!(outcome.answer, SyncOpAnswer::Head(_));
    let mut latch = lock(&state.backwards_latch);
    if outcome.backwards_drops > 0 {
        let total = SyncCounters::bump(
            &counters.capture_backwards_drops,
            topic,
            u64::from(outcome.backwards_drops),
        );
        match latch.on_failure() {
            RegimeDecision::Loud => tracing::warn!(
                node_id = %node_id,
                sync_input = %topic,
                dropped = outcome.backwards_drops,
                total_failures = total,
                "sync capture-time matching dropped a frame whose header.stamp is \
                 older than one this input already contributed; the frame is in no set \
                 (counted as `sync_capture_backwards_drops`). The producer publishes \
                 out of capture order"
            ),
            RegimeDecision::StillFailing { total, suppressed } => tracing::warn!(
                node_id = %node_id,
                sync_input = %topic,
                total_failures = total,
                suppressed,
                "sync capture-time backwards drops still recurring"
            ),
            RegimeDecision::Suppressed { suppressed } => tracing::debug!(
                node_id = %node_id,
                sync_input = %topic,
                suppressed,
                "sync capture-time backwards drop (suppressed)"
            ),
        }
    } else if accepted {
        if let Some(suppressed) = latch.on_success() {
            tracing::info!(
                node_id = %node_id,
                sync_input = %topic,
                suppressed_failures = suppressed,
                "sync capture-time stamps are in order again on this input"
            );
        }
    }
    drop(latch);
    let mut latch = lock(&state.decode_latch);
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
    } else if accepted {
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
            &[("lidar", LOC), ("cam", LOC)],
        );
        let spec = validate_sync_capture("fuse", &i, &triggers(), true, true)
            .expect("valid")
            .expect("opted in");
        assert_eq!(spec.window_ns, 20_000_000);
        assert_eq!(
            spec.by_input.keys().map(String::as_str).collect::<Vec<_>>(),
            vec!["cam", "lidar"],
            "in declaration order, not the map's order"
        );
    }

    #[test]
    fn an_unbounded_node_cannot_opt_in() {
        // No window, no way to tell a clock restart from out-of-order frames.
        let i = info(MacroPolicy::UnboundedSync, &[("cam", LOC), ("lidar", LOC)]);
        let err =
            validate_sync_capture("fuse", &i, &triggers(), true, true).expect_err("unbounded node");
        assert!(err.to_string().contains("node 'fuse'"), "{err}");
        assert!(err.to_string().contains("`sync_window_ms`"), "{err}");
    }

    #[test]
    fn a_location_for_a_non_trigger_or_unknown_input_is_refused_by_name() {
        for stray in ["odom", "lidr"] {
            let i = info(
                MacroPolicy::Sync { window_ms: 20 },
                &[("cam", LOC), ("lidar", LOC), (stray, LOC)],
            );
            match validate_sync_capture("fuse", &i, &triggers(), true, true) {
                Err(TransportError::SyncCaptureStamp { node_id, input, .. }) => {
                    assert_eq!((node_id.as_str(), input.as_str()), ("fuse", stray));
                }
                other => panic!("expected SyncCaptureStamp for {stray}, got {other:?}"),
            }
        }
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
        assert!(
            err.to_string().contains("only a `sync_window_ms` node"),
            "{err}"
        );
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

    // -- capture_op: counters, flood latches and lock recovery ------------

    /// A node entry whose one input is a FIFO of frames, each a decoded
    /// capture stamp or a decode error. A fill places the next frame in the
    /// head slot (re-offering a head still held), an advance replaces it, a
    /// void clears it; [`FakeEntry::serve`] is a fire's read.
    struct FakeEntry {
        head: Option<Result<u64, StampDecodeError>>,
        queue: std::collections::VecDeque<Result<u64, StampDecodeError>>,
    }

    impl FakeEntry {
        fn new() -> Self {
            Self {
                head: None,
                queue: std::collections::VecDeque::new(),
            }
        }

        fn serve(&mut self) {
            self.head = None;
        }

        fn answer(&self) -> SyncOpAnswer {
            self.head
                .map_or(SyncOpAnswer::Nothing, |_| SyncOpAnswer::Head(0))
        }
    }

    impl NodeEntry for FakeEntry {
        fn info(&self) -> TransportResult<NodeInfo> {
            Ok(NodeInfo::default())
        }

        fn init(&mut self, _context: super::super::node::NodeContext) -> TransportResult<()> {
            Ok(())
        }

        fn tick(&mut self) -> TransportResult<()> {
            Ok(())
        }

        fn sync_head_op(&mut self, input_name: &str, op: SyncHeadOp) -> SyncOpAnswer {
            assert_eq!(input_name, "cam", "the op names the node's input");
            match op {
                SyncHeadOp::FillBoundary | SyncHeadOp::FillRefill => {
                    if self.head.is_none() {
                        self.head = self.queue.pop_front();
                    }
                    self.answer()
                }
                SyncHeadOp::Advance { .. } => {
                    self.head = self.queue.pop_front();
                    self.answer()
                }
                SyncHeadOp::Void => {
                    self.head = None;
                    SyncOpAnswer::Nothing
                }
                SyncHeadOp::ProbeNext | SyncHeadOp::PeekNext { .. } => SyncOpAnswer::Nothing,
            }
        }

        fn sync_capture_stamp(
            &mut self,
            input_name: &str,
            slot: SyncFrameSlot,
            _location: HeaderStampLocation,
        ) -> Option<Result<u64, StampDecodeError>> {
            assert_eq!(input_name, "cam");
            match slot {
                SyncFrameSlot::Head => self.head,
                SyncFrameSlot::Next => None,
            }
        }

        fn supports_sync_capture_stamps(&self) -> bool {
            true
        }
    }

    const MS: u64 = 1_000_000;
    const TOPIC: &str = "/cap/cam";
    const BAD: StampDecodeError = StampDecodeError::NanosecOutOfRange {
        nanosec: 1_000_000_000,
    };

    /// Push one frame, run one boundary fill through `capture_op`, and serve
    /// the head if it was accepted. Returns the answer.
    fn offer(
        entry: &mut FakeEntry,
        port: &CapturePort,
        counters: &SyncCounters,
        frame: Result<u64, StampDecodeError>,
    ) -> SyncOpAnswer {
        entry.queue.push_back(frame);
        let answer = capture_op(
            "fuse",
            TOPIC,
            "cam",
            port,
            counters,
            entry,
            SyncHeadOp::FillBoundary,
        );
        if matches!(answer, SyncOpAnswer::Head(_)) {
            entry.serve();
        }
        answer
    }

    fn counted(
        map: &std::sync::RwLock<
            IndexMap<std::sync::Arc<str>, std::sync::Arc<std::sync::atomic::AtomicU64>>,
        >,
    ) -> u64 {
        map.read()
            .expect("counter map")
            .get(TOPIC)
            .map_or(0, |c| c.load(std::sync::atomic::Ordering::Acquire))
    }

    fn count_lines(lines: &[&str], needle: &str) -> usize {
        lines.iter().filter(|l| l.contains(needle)).count()
    }

    /// Ten undecodable frames in a row: one loud line, eight suppressed
    /// repeats, a decade line at the tenth, then one recovery line when a
    /// good frame is accepted. Every frame is counted once on the topic key.
    #[test]
    #[tracing_test::traced_test]
    fn a_decode_failure_regime_is_counted_latched_and_recovers_once() {
        let port = CapturePort::new(HeaderStampLocation::Fixed { offset: 0 }, 50 * MS);
        let counters = SyncCounters::default();
        let mut entry = FakeEntry::new();
        for _ in 0..10 {
            assert_eq!(
                offer(&mut entry, &port, &counters, Err(BAD)),
                SyncOpAnswer::Nothing
            );
        }
        assert_eq!(
            offer(&mut entry, &port, &counters, Ok(100 * MS)),
            SyncOpAnswer::Head(100 * MS)
        );
        assert_eq!(counted(&counters.capture_decode_failures), 10);
        assert_eq!(counted(&counters.capture_backwards_drops), 0);
        logs_assert(|lines: &[&str]| {
            let got = (
                count_lines(lines, "could not be decoded"),
                count_lines(lines, "decode failure (suppressed)"),
                count_lines(lines, "decode failures still recurring"),
                count_lines(lines, "decode again on this input"),
            );
            if got == (1, 8, 1, 1) {
                Ok(())
            } else {
                Err(format!("loud/suppressed/still/recovered = {got:?}"))
            }
        });
    }

    /// The same regime shape for frames whose stamp goes backwards within the
    /// window: one loud line, suppressed repeats, a decade line, one recovery
    /// line once an in-order head is accepted. The high-water never moves on a
    /// dropped frame.
    #[test]
    #[tracing_test::traced_test]
    fn a_backwards_regime_is_counted_latched_and_recovers_once() {
        let port = CapturePort::new(HeaderStampLocation::Fixed { offset: 0 }, 50 * MS);
        let counters = SyncCounters::default();
        let mut entry = FakeEntry::new();
        assert_eq!(
            offer(&mut entry, &port, &counters, Ok(100 * MS)),
            SyncOpAnswer::Head(100 * MS)
        );
        for back in 0..10u64 {
            assert_eq!(
                offer(&mut entry, &port, &counters, Ok(90 * MS + back)),
                SyncOpAnswer::Nothing
            );
        }
        assert_eq!(*lock(&port.high_water), Some(100 * MS));
        assert_eq!(
            offer(&mut entry, &port, &counters, Ok(101 * MS)),
            SyncOpAnswer::Head(101 * MS)
        );
        assert_eq!(counted(&counters.capture_backwards_drops), 10);
        assert_eq!(counted(&counters.capture_decode_failures), 0);
        logs_assert(|lines: &[&str]| {
            let got = (
                count_lines(lines, "older than one this input already contributed"),
                count_lines(lines, "backwards drop (suppressed)"),
                count_lines(lines, "backwards drops still recurring"),
                count_lines(lines, "in order again on this input"),
            );
            if got == (1, 8, 1, 1) {
                Ok(())
            } else {
                Err(format!("loud/suppressed/still/recovered = {got:?}"))
            }
        });
    }

    /// A lone failure re-arms silently: no recovery line for a regime that
    /// suppressed nothing, and the next failure is loud again.
    #[test]
    #[tracing_test::traced_test]
    fn a_lone_failure_re_arms_without_a_recovery_line() {
        let port = CapturePort::new(HeaderStampLocation::Fixed { offset: 0 }, 50 * MS);
        let counters = SyncCounters::default();
        let mut entry = FakeEntry::new();
        offer(&mut entry, &port, &counters, Err(BAD));
        offer(&mut entry, &port, &counters, Ok(100 * MS));
        offer(&mut entry, &port, &counters, Err(BAD));
        assert_eq!(counted(&counters.capture_decode_failures), 2);
        logs_assert(|lines: &[&str]| {
            let got = (
                count_lines(lines, "could not be decoded"),
                count_lines(lines, "decode again on this input"),
            );
            if got == (2, 0) {
                Ok(())
            } else {
                Err(format!("loud/recovered = {got:?}"))
            }
        });
    }

    /// A poisoned state mutex (a panic while it was held) does not turn the
    /// op into a failure: the state is a plain value, so it is taken and the
    /// op proceeds on it.
    #[test]
    fn a_poisoned_state_lock_is_recovered_not_failed() {
        let port = CapturePort::new(HeaderStampLocation::Fixed { offset: 0 }, 50 * MS);
        let _ = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            let _hw = port.high_water.lock();
            let _bw = port.backwards_latch.lock();
            let _dl = port.decode_latch.lock();
            panic!("poison the capture state");
        }));
        assert!(port.high_water.is_poisoned());
        let counters = SyncCounters::default();
        let mut entry = FakeEntry::new();
        assert_eq!(
            offer(&mut entry, &port, &counters, Ok(100 * MS)),
            SyncOpAnswer::Head(100 * MS)
        );
        assert_eq!(
            offer(&mut entry, &port, &counters, Ok(90 * MS)),
            SyncOpAnswer::Nothing
        );
        assert_eq!(counted(&counters.capture_backwards_drops), 1);
        port.restore_high_water(5 * MS);
        assert_eq!(*lock(&port.high_water), Some(5 * MS));
    }
}
