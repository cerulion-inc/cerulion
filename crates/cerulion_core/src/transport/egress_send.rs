// SPDX-License-Identifier: AGPL-3.0-only
//! Per-topic send queues between the gateway's drive thread and zenoh.
//!
//! The gateway's drive thread drains every demanded topic's shared-memory tap
//! and hands each frame to that topic's [`EgressLane`]: a bounded queue with
//! its own sender thread. The sender thread is the only place a frame is put
//! on zenoh, so a put that waits on a full zenoh transmit queue holds up its
//! own sender thread and not the drive thread, except while a LAN close waits
//! for that put (see [`run_sender`]). Topics sent to the SAME peer
//! still share zenoh's per-priority transmit queue for that peer: while one
//! reliable put waits there, puts of other topics to that peer at that
//! priority wait too. Topics sent to other peers keep flowing.
//!
//! # Congestion control
//!
//! [`congestion_control_for`] maps the topic's [`EgressReliability`] onto the
//! zenoh put:
//!
//! - `Reliable` puts with `CongestionControl::Block`. zenoh 1.10.1 defaults a
//!   put to `Drop`, which discards a frame at the sender when the transmit
//!   queue stays full for `wait_before_drop` (1 ms) and still returns `Ok`.
//!   `Block` waits for the queue instead, for up to zenoh's
//!   `wait_before_close` (5 s by default). If the queue has not moved by then,
//!   zenoh discards that frame, closes its link to the peer, and the put still
//!   returns `Ok`.
//! - `BestEffort` puts with `CongestionControl::Drop`. zenoh treats every
//!   best-effort message as droppable whatever the congestion control says.
//!
//! # When a lane is full
//!
//! [`enqueue_decision`] is the overflow rule:
//!
//! - a reliable lane REFUSES the new frame. The refusal is counted
//!   ([`EgressTopicStats::refused`]) and logged naming the topic, loud on the
//!   first refusal of a run of them (`FailureRegimeLatch`).
//! - a best-effort lane drops its OLDEST queued frame and queues the new one
//!   (newest wins), counted in [`EgressTopicStats::dropped`].
//!
//! # Counters
//!
//! [`EgressTopicStats`] is one lane's snapshot. `forwarded` counts frames whose
//! put returned `Ok`. zenoh 1.10.1 also returns `Ok` for a reliable frame it
//! dropped after waiting out `wait_before_close`, and for a best-effort frame
//! it discards inside its transmit queue after the put; neither is counted
//! anywhere here.
//!
//! Every frame the drive thread offers is in exactly one of `queued`,
//! `in_flight`, `forwarded`, `dropped`, `refused`, `failed` and `discarded`.

use std::collections::{HashMap, VecDeque};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Condvar, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use serde::{Deserialize, Serialize};

use super::bridge::TopicBridgeManager;
use super::carrier::EgressReliability;
use super::failure_regime_latch::{lock_regime_latch, FailureRegimeLatch, RegimeDecision};
use super::network::NetworkManager;

/// Stack size for a lane's sender thread. The thread runs one loop that pops
/// a frame and calls the zenoh put; the frame itself lives on the heap.
const SENDER_STACK_BYTES: usize = 256 * 1024;

/// The most queue slots a lane reserves up front. A lane declared deeper
/// grows its queue the first time it holds more frames than this.
const PREALLOCATED_SLOTS: usize = 4096;

/// The zenoh congestion control a put of a `reliability` topic uses.
pub(crate) fn congestion_control_for(
    reliability: EgressReliability,
) -> zenoh::qos::CongestionControl {
    match reliability {
        EgressReliability::Reliable => zenoh::qos::CongestionControl::Block,
        EgressReliability::BestEffort => zenoh::qos::CongestionControl::Drop,
    }
}

/// The outcome of offering one frame to a lane. [`enqueue_decision`] picks
/// among the first three; `Closed` comes from a lane that is stopping.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum EnqueueOutcome {
    /// The lane had room; the frame is queued.
    Queued,
    /// A best-effort lane was full; its oldest frame is dropped and the new
    /// frame queued.
    ReplacedOldest,
    /// A reliable lane was full; the new frame is refused.
    Refused,
    /// The lane is closed (the gateway is stopping); the frame is discarded.
    Closed,
}

/// The overflow rule for a lane holding `queued` frames out of `capacity`.
///
/// A lane with room queues the frame whatever its reliability. A full lane
/// refuses the frame when reliable and replaces its oldest frame when best
/// effort. A `capacity` of 0 is treated as 1.
pub(crate) fn enqueue_decision(
    reliability: EgressReliability,
    queued: usize,
    capacity: usize,
) -> EnqueueOutcome {
    if queued < capacity.max(1) {
        return EnqueueOutcome::Queued;
    }
    match reliability {
        EgressReliability::Reliable => EnqueueOutcome::Refused,
        EgressReliability::BestEffort => EnqueueOutcome::ReplacedOldest,
    }
}

/// One egress topic's send-queue counters, as
/// [`EgressStats::snapshot`] reads them. A field missing from a serialized
/// copy decodes as its default (`0`, `reliable`, an empty topic), so a
/// reader built against more counters still reads an older writer's.
#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default)]
pub struct EgressTopicStats {
    /// Canonical topic name.
    pub topic: String,
    /// How the topic's frames are put.
    pub reliability: EgressReliability,
    /// The send queue's bound in frames.
    pub send_queue: u64,
    /// Frames waiting in the send queue when the snapshot was taken.
    pub queued: u64,
    /// Frames the sender thread has taken from the queue and is putting
    /// (0 or 1).
    pub in_flight: u64,
    /// Frames whose zenoh put returned `Ok`.
    pub forwarded: u64,
    /// Best-effort frames dropped because the send queue was full (the oldest
    /// queued frame makes room for the newest).
    pub dropped: u64,
    /// Reliable frames refused because the send queue was full.
    pub refused: u64,
    /// Frames whose zenoh put returned an error.
    pub failed: u64,
    /// Frames not sent because demand for the topic ended, LAN access closed,
    /// or the gateway stopped before their put.
    pub discarded: u64,
}

/// The frames a lane holds, whether it still accepts them, and its counters.
/// The counters live under the same lock as the frames, so a frame moves
/// from one to another in one step and a snapshot sees it in exactly one.
#[derive(Debug)]
struct LaneQueue {
    frames: VecDeque<Vec<u8>>,
    closed: bool,
    in_flight: u64,
    forwarded: u64,
    dropped: u64,
    refused: u64,
    failed: u64,
    discarded: u64,
}

/// One topic's bounded send queue and counters, shared by the drive thread
/// (which queues) and the topic's sender thread (which puts).
#[derive(Debug)]
pub(crate) struct EgressLane {
    topic: String,
    reliability: EgressReliability,
    capacity: usize,
    queue: Mutex<LaneQueue>,
    ready: Condvar,
    /// Refusals of a full reliable lane. Touched by the drive thread only.
    refuse_latch: Mutex<FailureRegimeLatch>,
    /// Replacements in a full best-effort lane. Touched by the drive thread only.
    replace_latch: Mutex<FailureRegimeLatch>,
    /// zenoh put errors. Touched by the sender thread only.
    put_latch: Mutex<FailureRegimeLatch>,
}

impl EgressLane {
    // hot-path-alloc-ok-fn: once per topic, when its first egress tap
    // attaches. The queue is sized here (up to `PREALLOCATED_SLOTS`) so a
    // push into a lane of that bound never grows it.
    fn new(topic: &str, reliability: EgressReliability, capacity: usize) -> Self {
        let capacity = capacity.max(1);
        Self {
            topic: topic.to_string(),
            reliability,
            capacity,
            queue: Mutex::new(LaneQueue {
                frames: VecDeque::with_capacity(capacity.min(PREALLOCATED_SLOTS)),
                closed: false,
                in_flight: 0,
                forwarded: 0,
                dropped: 0,
                refused: 0,
                failed: 0,
                discarded: 0,
            }),
            ready: Condvar::new(),
            refuse_latch: Mutex::new(FailureRegimeLatch::new()),
            replace_latch: Mutex::new(FailureRegimeLatch::new()),
            put_latch: Mutex::new(FailureRegimeLatch::new()),
        }
    }

    fn lock_queue(&self) -> std::sync::MutexGuard<'_, LaneQueue> {
        // Every update under this lock is a plain increment, assignment or
        // queue operation with nothing between them that can panic, so a
        // poisoned lock still holds consistent frames and counters.
        self.queue
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
    }

    /// Offer one frame. Never blocks on the sender thread: the lock is held
    /// only for the queue operation.
    pub(crate) fn enqueue(&self, frame: Vec<u8>) -> EnqueueOutcome {
        let mut queue = self.lock_queue();
        if queue.closed {
            queue.discarded += 1;
            return EnqueueOutcome::Closed;
        }
        let outcome = enqueue_decision(self.reliability, queue.frames.len(), self.capacity);
        match outcome {
            EnqueueOutcome::Queued => queue.frames.push_back(frame),
            EnqueueOutcome::ReplacedOldest => {
                queue.frames.pop_front();
                queue.frames.push_back(frame);
                queue.dropped += 1;
            }
            EnqueueOutcome::Refused => queue.refused += 1,
            EnqueueOutcome::Closed => {}
        }
        drop(queue);
        match outcome {
            EnqueueOutcome::Queued => {
                self.ready.notify_one();
                self.on_accepted();
            }
            EnqueueOutcome::ReplacedOldest => {
                self.ready.notify_one();
                self.on_replaced();
            }
            EnqueueOutcome::Refused => self.on_refused(),
            EnqueueOutcome::Closed => {}
        }
        outcome
    }

    fn on_accepted(&self) {
        if let Some(suppressed) = lock_regime_latch(&self.refuse_latch).on_success() {
            tracing::info!(
                topic = %self.topic,
                suppressed,
                "gateway egress send queue has room again; reliable frames are queued"
            );
        }
        // A best-effort replacement regime ends silently: dropping is that
        // topic's declared behaviour, and `dropped` keeps the total.
        let _ = lock_regime_latch(&self.replace_latch).on_success();
    }

    fn on_refused(&self) {
        let decision = lock_regime_latch(&self.refuse_latch).on_failure();
        let capacity = self.capacity;
        match decision {
            RegimeDecision::Loud => tracing::warn!(
                topic = %self.topic,
                send_queue = capacity,
                "gateway egress send queue is full: a reliable frame was refused and will \
                 not be sent. The network link is slower than the topic. Raise `send_queue` \
                 on the topic's `network: egress:` entry, lower the topic's publish rate, or \
                 mark it `reliability: best_effort` if losing frames is acceptable \
                 (repeats log at debug; the refused counter keeps counting)"
            ),
            RegimeDecision::StillFailing { total, suppressed } => tracing::warn!(
                topic = %self.topic,
                send_queue = capacity,
                total_failures = total,
                suppressed,
                "gateway egress send queue is still full: reliable frames are being refused"
            ),
            RegimeDecision::Suppressed { suppressed } => tracing::debug!(
                topic = %self.topic,
                suppressed,
                "gateway egress send queue full: reliable frame refused (repeat)"
            ),
        }
    }

    fn on_replaced(&self) {
        let decision = lock_regime_latch(&self.replace_latch).on_failure();
        match decision {
            RegimeDecision::Loud | RegimeDecision::StillFailing { .. } => tracing::debug!(
                topic = %self.topic,
                send_queue = self.capacity,
                "gateway egress send queue full: dropped the oldest best-effort frame"
            ),
            RegimeDecision::Suppressed { .. } => {}
        }
    }

    /// Drop every queued frame, counting each as discarded. Called when demand
    /// for the topic ends.
    pub(crate) fn discard_queued(&self) {
        let mut queue = self.lock_queue();
        queue.discarded += queue.frames.len() as u64;
        queue.frames.clear();
    }

    /// Stop accepting frames and wake the sender thread so it exits. Frames
    /// still queued are discarded.
    fn close(&self) {
        {
            let mut queue = self.lock_queue();
            queue.closed = true;
            queue.discarded += queue.frames.len() as u64;
            queue.frames.clear();
        }
        self.ready.notify_all();
    }

    /// The next frame to send, waiting while the lane is empty, and counted
    /// in `in_flight` until [`Self::record_put`] or
    /// [`Self::discard_in_flight`]. `None` once the lane is closed.
    fn next_frame(&self) -> Option<Vec<u8>> {
        let mut queue = self.lock_queue();
        loop {
            if queue.closed {
                return None;
            }
            if let Some(frame) = queue.frames.pop_front() {
                queue.in_flight = 1;
                return Some(frame);
            }
            queue = self
                .ready
                .wait(queue)
                .unwrap_or_else(std::sync::PoisonError::into_inner);
        }
    }

    /// The in-flight frame was not put: demand ended or LAN access closed.
    fn discard_in_flight(&self) {
        let mut queue = self.lock_queue();
        queue.in_flight = 0;
        queue.discarded += 1;
    }

    /// Count the in-flight frame's put result.
    fn record_put(&self, result: Result<(), crate::error::TransportError>) {
        {
            let mut queue = self.lock_queue();
            queue.in_flight = 0;
            match &result {
                Ok(()) => queue.forwarded += 1,
                Err(_) => queue.failed += 1,
            }
        }
        match result {
            Ok(()) => {
                if let Some(suppressed) = lock_regime_latch(&self.put_latch).on_success() {
                    tracing::info!(
                        topic = %self.topic,
                        suppressed,
                        "gateway egress recovered; forwarding again"
                    );
                }
            }
            Err(e) => {
                let decision = lock_regime_latch(&self.put_latch).on_failure();
                match decision {
                    RegimeDecision::Loud => tracing::warn!(
                        topic = %self.topic,
                        error = %e,
                        "gateway egress put failed; this frame was not sent (repeats log at \
                         debug; the failed counter keeps counting)"
                    ),
                    RegimeDecision::StillFailing { total, suppressed } => tracing::warn!(
                        topic = %self.topic,
                        error = %e,
                        total_failures = total,
                        suppressed,
                        "gateway egress puts are still failing"
                    ),
                    RegimeDecision::Suppressed { suppressed } => tracing::debug!(
                        topic = %self.topic,
                        error = %e,
                        suppressed,
                        "gateway egress put failed (repeat)"
                    ),
                }
            }
        }
    }

    // hot-path-alloc-ok-fn: a counter read for `status` and tests, never on
    // the forwarding path.
    fn snapshot(&self) -> EgressTopicStats {
        let queue = self.lock_queue();
        EgressTopicStats {
            topic: self.topic.clone(),
            reliability: self.reliability,
            send_queue: self.capacity as u64,
            queued: queue.frames.len() as u64,
            in_flight: queue.in_flight,
            forwarded: queue.forwarded,
            dropped: queue.dropped,
            refused: queue.refused,
            failed: queue.failed,
            discarded: queue.discarded,
        }
    }
}

/// The sender thread's loop: pop a frame, re-check demand under the
/// egress-pass guard, put it, count the result.
///
/// The guard is the same rendezvous a drive pass holds: closing LAN access
/// stores every egress flag `false` and then takes the guard exclusively, so a
/// put that started before the close finishes first and a frame popped after
/// it sees the flag `false` and is discarded.
fn run_sender(
    lane: Arc<EgressLane>,
    network: Arc<NetworkManager>,
    bridges: Arc<TopicBridgeManager>,
    flag: Arc<AtomicBool>,
) {
    while let Some(frame) = lane.next_frame() {
        let _pass = bridges.egress_pass();
        if !flag.load(Ordering::Relaxed) {
            lane.discard_in_flight();
            continue;
        }
        let result = network.publish_to_network_with(&lane.topic, frame, lane.reliability);
        lane.record_put(result);
    }
}

/// A lane plus the thread that sends from it.
#[derive(Debug)]
struct LaneHandle {
    lane: Arc<EgressLane>,
    thread: Option<JoinHandle<()>>,
}

/// Every egress lane a gateway has opened, readable from any thread. The
/// gateway hands out an `Arc` of it ([`crate::GatewayRuntime::egress_stats`])
/// so a caller can keep reading the counters after the gateway moves onto its
/// drive thread.
#[derive(Debug)]
pub struct EgressStats {
    lanes: Mutex<HashMap<String, Arc<EgressLane>>>,
}

impl EgressStats {
    // hot-path-alloc-ok-fn: once per gateway, at construction.
    pub(crate) fn new() -> Self {
        Self {
            lanes: Mutex::new(HashMap::new()),
        }
    }

    // hot-path-alloc-ok-fn: once per topic, when its lane opens.
    fn insert(&self, lane: &Arc<EgressLane>) {
        self.lanes
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
            .insert(lane.topic.clone(), Arc::clone(lane));
    }

    /// One topic's counters, or `None` when no frame of that topic has been
    /// demanded yet (its lane opens with its first egress tap).
    pub fn topic(&self, topic: &str) -> Option<EgressTopicStats> {
        let canonical = super::network::canonical_topic(topic);
        self.lanes
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
            .get(canonical.as_ref())
            .map(|lane| lane.snapshot())
    }

    /// Log every open lane's totals: WARN for a topic that dropped, refused or
    /// failed any frame, INFO otherwise. The gateway calls it when it stops.
    // hot-path-alloc-ok-fn: once, at gateway teardown.
    pub(crate) fn log_totals(&self) {
        for s in self.snapshot() {
            if s.dropped + s.refused + s.failed > 0 {
                tracing::warn!(
                    topic = %s.topic,
                    forwarded = s.forwarded,
                    dropped = s.dropped,
                    refused = s.refused,
                    failed = s.failed,
                    discarded = s.discarded,
                    "gateway egress totals for this run: frames were lost"
                );
            } else {
                tracing::info!(
                    topic = %s.topic,
                    forwarded = s.forwarded,
                    discarded = s.discarded,
                    "gateway egress totals for this run"
                );
            }
        }
    }

    /// Every open lane's counters, sorted by topic.
    // hot-path-alloc-ok-fn: a counter read for `status` and tests, never on
    // the forwarding path.
    pub fn snapshot(&self) -> Vec<EgressTopicStats> {
        let lanes: Vec<Arc<EgressLane>> = self
            .lanes
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
            .values()
            .cloned()
            .collect();
        let mut stats: Vec<EgressTopicStats> = lanes.iter().map(|l| l.snapshot()).collect();
        stats.sort_by(|a, b| a.topic.cmp(&b.topic));
        stats
    }
}

/// How often [`LossReport::tick`] looks at the counters. A topic whose
/// `refused` or `failed` rose since the last look gets one WARN line per
/// look; one whose only rise is best-effort `dropped` gets an INFO line.
pub(crate) const LOSS_REPORT_INTERVAL: Duration = Duration::from_secs(10);

/// One topic's loss totals: `(dropped, refused, failed)`.
pub(crate) type LossTotals = (u64, u64, u64);

/// The topics whose loss totals in `now` exceed what `reported` last saw,
/// with their new totals, in `now`'s order. A topic `reported` has never
/// seen counts from zero.
// hot-path-alloc-ok-fn: runs from `LossReport::tick`, at most once per
// interval.
pub(crate) fn loss_increases(
    reported: &HashMap<String, LossTotals>,
    now: &[EgressTopicStats],
) -> Vec<(String, LossTotals)> {
    now.iter()
        .filter_map(|s| {
            let totals = (s.dropped, s.refused, s.failed);
            let seen = reported.get(&s.topic).copied().unwrap_or((0, 0, 0));
            (totals != seen).then(|| (s.topic.clone(), totals))
        })
        .collect()
}

/// The gateway's periodic loss line: every [`LOSS_REPORT_INTERVAL`] it logs
/// one line per topic that dropped, refused or failed frames since the last
/// one (WARN for refused or failed, INFO for best-effort drops alone), so a
/// run whose only window is its log (a `graph run` gateway with a `network:`
/// block) still shows them. [`EgressStats::log_totals`] writes the
/// run's totals when the gateway stops.
#[derive(Debug, Default)]
pub(crate) struct LossReport {
    reported: HashMap<String, LossTotals>,
    last_look: Option<Instant>,
}

impl LossReport {
    /// Look at `stats` if [`LOSS_REPORT_INTERVAL`] has passed since the last
    /// look (or there was none).
    // hot-path-alloc-ok-fn: runs at most once per interval, not per frame.
    pub(crate) fn tick(&mut self, now: Instant, stats: &EgressStats) {
        if self
            .last_look
            .is_some_and(|last| now.duration_since(last) < LOSS_REPORT_INTERVAL)
        {
            return;
        }
        self.last_look = Some(now);
        for (topic, (dropped, refused, failed)) in loss_increases(&self.reported, &stats.snapshot())
        {
            let (_, was_refused, was_failed) =
                self.reported.get(&topic).copied().unwrap_or((0, 0, 0));
            if refused > was_refused || failed > was_failed {
                tracing::warn!(
                    topic = %topic,
                    dropped,
                    refused,
                    failed,
                    "gateway egress lost frames on this topic since the last report (totals; \
                     refused = reliable frames the full send queue refused, failed = puts \
                     zenoh refused, dropped = best-effort frames the full send queue replaced)"
                );
            } else {
                tracing::info!(
                    topic = %topic,
                    dropped,
                    "gateway egress dropped best-effort frames on this topic since the last \
                     report (total; the full send queue replaced its oldest frame)"
                );
            }
            self.reported.insert(topic, (dropped, refused, failed));
        }
    }
}

/// The gateway's lanes keyed by canonical topic, owned by the drive thread.
///
/// Dropping it closes every lane and joins every sender thread. A sender in
/// the middle of a `Block` put returns when zenoh's transmit queue moves or
/// when zenoh closes the link after its `wait_before_close` (5 s by default).
#[derive(Debug)]
pub(crate) struct EgressLanes {
    lanes: HashMap<String, LaneHandle>,
    stats: Arc<EgressStats>,
}

impl EgressLanes {
    // hot-path-alloc-ok-fn: once per gateway, at construction.
    pub(crate) fn new() -> Self {
        Self {
            lanes: HashMap::new(),
            stats: Arc::new(EgressStats::new()),
        }
    }

    pub(crate) fn stats(&self) -> Arc<EgressStats> {
        Arc::clone(&self.stats)
    }

    pub(crate) fn stats_ref(&self) -> &EgressStats {
        &self.stats
    }

    /// The topic's lane, if one is open.
    pub(crate) fn get(&self, topic: &str) -> Option<&Arc<EgressLane>> {
        self.lanes.get(topic).map(|h| &h.lane)
    }

    /// Open the topic's lane and start its sender thread, unless one is open.
    ///
    /// # Errors
    ///
    /// `TransportError::Internal` when the sender thread cannot be spawned.
    // hot-path-alloc-ok-fn: once per topic, when its first egress tap
    // attaches.
    pub(crate) fn open(
        &mut self,
        topic: &str,
        reliability: EgressReliability,
        capacity: usize,
        network: Arc<NetworkManager>,
        bridges: Arc<TopicBridgeManager>,
        flag: Arc<AtomicBool>,
    ) -> crate::error::TransportResult<()> {
        if self.lanes.contains_key(topic) {
            return Ok(());
        }
        let lane = Arc::new(EgressLane::new(topic, reliability, capacity));
        let thread_lane = Arc::clone(&lane);
        let thread = std::thread::Builder::new()
            .name("cerulion-egress-send".to_string())
            .stack_size(SENDER_STACK_BYTES)
            .spawn(move || run_sender(thread_lane, network, bridges, flag))
            .map_err(|e| crate::error::TransportError::Internal {
                reason: format!("could not start the egress sender thread for '{topic}': {e}"),
            })?;
        tracing::debug!(
            topic = %topic,
            ?reliability,
            send_queue = lane.capacity,
            "gateway egress send queue opened"
        );
        self.stats.insert(&lane);
        self.lanes.insert(
            topic.to_string(),
            LaneHandle {
                lane,
                thread: Some(thread),
            },
        );
        Ok(())
    }

    /// Close every lane and join every sender thread.
    pub(crate) fn shutdown(&mut self) {
        for handle in self.lanes.values() {
            handle.lane.close();
        }
        for handle in self.lanes.values_mut() {
            if let Some(thread) = handle.thread.take() {
                let _ = thread.join();
            }
        }
    }
}

impl Drop for EgressLanes {
    fn drop(&mut self) {
        self.shutdown();
    }
}

#[cfg(test)]
mod tests {
    use super::{
        congestion_control_for, enqueue_decision, loss_increases, EgressLane, EgressStats,
        EgressTopicStats, EnqueueOutcome, LossReport, LOSS_REPORT_INTERVAL,
    };
    use crate::testing::count_at_exclusively;
    use crate::transport::carrier::EgressReliability::{BestEffort, Reliable};
    use std::collections::HashMap;
    use std::sync::Arc;
    use std::time::Instant;
    use tracing_test::traced_test;
    use zenoh::qos::CongestionControl;

    /// A reliable put waits on a full zenoh transmit queue; a best-effort put
    /// may be discarded there.
    #[test]
    fn reliable_puts_block_and_best_effort_puts_drop() {
        assert_eq!(congestion_control_for(Reliable), CongestionControl::Block);
        assert_eq!(congestion_control_for(BestEffort), CongestionControl::Drop);
    }

    /// The overflow table, written out by hand for both reliabilities at,
    /// below and above the bound, and for a zero bound (treated as one).
    #[test]
    fn a_full_reliable_lane_refuses_and_a_full_best_effort_lane_replaces() {
        let cases = [
            (Reliable, 0, 4, EnqueueOutcome::Queued),
            (Reliable, 3, 4, EnqueueOutcome::Queued),
            (Reliable, 4, 4, EnqueueOutcome::Refused),
            (Reliable, 9, 4, EnqueueOutcome::Refused),
            (BestEffort, 0, 4, EnqueueOutcome::Queued),
            (BestEffort, 3, 4, EnqueueOutcome::Queued),
            (BestEffort, 4, 4, EnqueueOutcome::ReplacedOldest),
            (BestEffort, 9, 4, EnqueueOutcome::ReplacedOldest),
            (Reliable, 0, 0, EnqueueOutcome::Queued),
            (Reliable, 1, 0, EnqueueOutcome::Refused),
            (BestEffort, 1, 0, EnqueueOutcome::ReplacedOldest),
        ];
        for (reliability, queued, capacity, want) in cases {
            assert_eq!(
                enqueue_decision(reliability, queued, capacity),
                want,
                "{reliability:?} queued {queued} of {capacity}"
            );
        }
    }

    fn frame(tag: u8) -> Vec<u8> {
        vec![tag; 48]
    }

    /// A full reliable lane refuses the new frame, keeps the queued ones in
    /// order, counts the refusal, and warns once naming the topic; the next
    /// refusal of the same run is not loud, and room ends the run with one
    /// recovery line.
    #[traced_test]
    #[test]
    fn a_full_reliable_lane_refuses_loudly_and_counts() {
        let lane = EgressLane::new("/r/cam", Reliable, 2);
        assert_eq!(lane.enqueue(frame(1)), EnqueueOutcome::Queued);
        assert_eq!(lane.enqueue(frame(2)), EnqueueOutcome::Queued);
        assert_eq!(lane.enqueue(frame(3)), EnqueueOutcome::Refused);
        assert_eq!(lane.enqueue(frame(4)), EnqueueOutcome::Refused);
        let stats = lane.snapshot();
        assert_eq!(
            (stats.queued, stats.refused, stats.dropped, stats.forwarded),
            (2, 2, 0, 0)
        );
        assert_eq!(stats.send_queue, 2);
        assert_eq!(lane.next_frame(), Some(frame(1)));
        let popped = lane.snapshot();
        assert_eq!(
            (popped.queued, popped.in_flight),
            (1, 1),
            "the popped frame is in flight"
        );
        lane.record_put(Ok(()));
        assert_eq!(lane.snapshot().in_flight, 0);
        assert_eq!(lane.enqueue(frame(5)), EnqueueOutcome::Queued);
        assert_eq!(lane.next_frame(), Some(frame(2)));
        assert_eq!(lane.next_frame(), Some(frame(5)));
        logs_assert(|lines: &[&str]| {
            let loud = count_at_exclusively(
                lines,
                "WARN",
                &["/r/cam", "send queue is full: a reliable frame was refused"],
            )?;
            if loud != 1 {
                return Err(format!("want exactly one loud refusal, got {loud}"));
            }
            let recovered =
                count_at_exclusively(lines, "INFO", &["/r/cam", "send queue has room again"])?;
            if recovered != 1 {
                return Err(format!("want exactly one recovery line, got {recovered}"));
            }
            Ok(())
        });
    }

    /// A full best-effort lane drops its OLDEST frame for the new one and
    /// counts the drop; nothing is refused.
    #[test]
    fn a_full_best_effort_lane_keeps_the_newest_and_counts_the_drop() {
        let lane = EgressLane::new("/r/scan", BestEffort, 2);
        assert_eq!(lane.enqueue(frame(1)), EnqueueOutcome::Queued);
        assert_eq!(lane.enqueue(frame(2)), EnqueueOutcome::Queued);
        assert_eq!(lane.enqueue(frame(3)), EnqueueOutcome::ReplacedOldest);
        assert_eq!(lane.enqueue(frame(4)), EnqueueOutcome::ReplacedOldest);
        let stats = lane.snapshot();
        assert_eq!((stats.queued, stats.dropped, stats.refused), (2, 2, 0));
        assert_eq!(lane.next_frame(), Some(frame(3)));
        assert_eq!(lane.next_frame(), Some(frame(4)));
    }

    /// Ending demand discards what is queued; closing discards the rest,
    /// wakes the sender with `None`, and refuses later frames; every discarded
    /// frame is counted.
    #[test]
    fn discarded_frames_are_counted() {
        let lane = EgressLane::new("/r/odom", Reliable, 4);
        lane.enqueue(frame(1));
        lane.enqueue(frame(2));
        lane.discard_queued();
        assert_eq!(lane.snapshot().discarded, 2);
        assert_eq!(lane.snapshot().queued, 0);
        lane.enqueue(frame(3));
        assert_eq!(lane.next_frame(), Some(frame(3)));
        lane.discard_in_flight();
        assert_eq!(lane.snapshot().in_flight, 0);
        lane.enqueue(frame(4));
        lane.close();
        assert_eq!(lane.next_frame(), None);
        assert_eq!(lane.enqueue(frame(5)), EnqueueOutcome::Closed);
        let stats = lane.snapshot();
        assert_eq!((stats.discarded, stats.refused, stats.queued), (5, 0, 0));
    }

    /// `forwarded` and `failed` move only on the put's result.
    #[test]
    fn forwarded_counts_only_accepted_puts() {
        let lane = EgressLane::new("/r/odom", Reliable, 4);
        lane.enqueue(frame(1));
        lane.enqueue(frame(2));
        assert_eq!(lane.snapshot().forwarded, 0, "queuing is not forwarding");
        lane.next_frame();
        assert_eq!(
            lane.snapshot().forwarded,
            0,
            "taking a frame is not forwarding"
        );
        lane.record_put(Ok(()));
        lane.next_frame();
        lane.record_put(Err(crate::error::TransportError::Publish {
            topic: "/r/odom".to_string(),
            reason: "test".to_string(),
        }));
        let stats = lane.snapshot();
        assert_eq!((stats.forwarded, stats.failed), (1, 1));
    }

    /// Two lanes fed the same script end in the same state: same pops in the
    /// same order, same counters.
    #[test]
    fn a_lane_is_deterministic() {
        let run = || {
            let lane = EgressLane::new("/r/scan", BestEffort, 3);
            let mut pops = Vec::new();
            for tag in 0..10u8 {
                lane.enqueue(frame(tag));
                if tag % 3 == 0 {
                    pops.push(lane.next_frame());
                    lane.record_put(Ok(()));
                }
            }
            (pops, lane.snapshot())
        };
        assert_eq!(run(), run());
    }

    fn stats(topic: &str, dropped: u64, refused: u64, failed: u64) -> EgressTopicStats {
        EgressTopicStats {
            topic: topic.to_string(),
            dropped,
            refused,
            failed,
            ..EgressTopicStats::default()
        }
    }

    /// A topic is reported when any of its three loss totals rose since the
    /// last report; one never reported counts from zero.
    #[test]
    fn loss_increases_lists_only_topics_whose_losses_rose() {
        let mut seen = HashMap::new();
        seen.insert("/a".to_string(), (1, 0, 0));
        seen.insert("/b".to_string(), (0, 2, 0));
        let now = [
            stats("/a", 1, 0, 0),
            stats("/b", 0, 3, 0),
            stats("/c", 0, 0, 0),
            stats("/d", 0, 0, 4),
        ];
        assert_eq!(
            loss_increases(&seen, &now),
            vec![("/b".to_string(), (0, 3, 0)), ("/d".to_string(), (0, 0, 4))]
        );
    }

    /// The report warns once per look for a topic that lost frames, stays
    /// quiet inside the interval and when nothing moved, and warns again
    /// when the totals rise.
    #[traced_test]
    #[test]
    fn the_loss_report_warns_when_losses_rise() {
        let table = EgressStats::new();
        let lane = Arc::new(EgressLane::new("/r/cam", Reliable, 1));
        table.insert(&lane);
        let mut report = LossReport::default();
        let t0 = Instant::now();
        lane.enqueue(frame(1));
        lane.enqueue(frame(2));
        report.tick(t0, &table);
        lane.enqueue(frame(3));
        report.tick(t0, &table);
        report.tick(t0 + LOSS_REPORT_INTERVAL, &table);
        report.tick(t0 + LOSS_REPORT_INTERVAL * 2, &table);
        logs_assert(|lines: &[&str]| {
            let n = count_at_exclusively(
                lines,
                "WARN",
                &["/r/cam", "lost frames on this topic since the last report"],
            )?;
            if n == 2 {
                Ok(())
            } else {
                Err(format!(
                    "want two loss reports (refused 1, then 2), got {n}"
                ))
            }
        });
    }
}
