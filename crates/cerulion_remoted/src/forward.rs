// SPDX-License-Identifier: AGPL-3.0-only
//! The WAN forward path: a bounded, drop-to-live per-topic
//! forward queue between the (sync, zero-copy) SHM tap drain and the (async,
//! QUIC-flow-controlled) per-topic uni-stream writer.
//!
//! # Why a bounded drop-to-live queue (Q4 of the Remote Plane design
//! record)
//!
//! The chain is: SHM (`drop_oldest`, bounded) → the tap drain
//! ([`crate::tap`]) → THIS bounded per-topic queue → the QUIC writer (which
//! alone BLOCKS on flow control). When the WAN is slower than a topic, the
//! writer stalls and the queue fills. The rule (mirroring the zenoh gateway's
//! egress + `VizLogWorker`): **drop the OLDEST frame, count it, and warn
//! once-per-regime — never an unbounded buffer, never block the drain.**
//!
//! Blocking the drain would back-pressure into SHM and perturb graph timing —
//! forbidden, taps are observation-only. So the drain thread copies
//! each frame OUT of the SHM slot (releasing the borrow immediately) and pushes
//! the owned bytes into this queue; only the writer task ever awaits QUIC. The
//! freshest frame always wins.

use std::collections::VecDeque;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Mutex;

use tokio::sync::Notify;

/// The bounded per-topic forward-queue depth — a small "freshest-few" window
/// (crib `VizLogWorker::VIZ_QUEUE_CAP = 8`). When the writer stalls on QUIC flow
/// control the queue fills to this depth, then each further push drops the
/// OLDEST frame (drop-to-live).
pub const WAN_QUEUE_CAP: usize = 8;

/// The log level the caller should use for a WAN drop, decided by
/// [`WanDropLatch::on_drop`] — a two-variant enum (not a bare bool) so call
/// sites read as `WanDropLevel::Warn` / `Debug`, the exact inversion-regression
/// surface a flood latch exists to pin. Cribs `DrainWarnLatch`/`DrainFailureLevel`
/// from [`cerulion_core`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WanDropLevel {
    /// The FIRST drop of a regime: log at `tracing::warn!` (operators always see
    /// the diagnostic).
    Warn,
    /// A repeat drop while the regime is open: log at `tracing::debug!` (a
    /// sustained WAN stall must not flood the journal at the publish rate).
    Debug,
}

/// Pure once-per-regime suppression latch for WAN drops (crib
/// [`cerulion_core`]'s `DrainWarnLatch`). A "regime" is a run of drops:
/// the FIRST drop warns, repeats downgrade to debug, and a push that finds ROOM
/// (the writer caught up) ends the regime and re-arms — so the next stall warns
/// again. Holds no transport state and emits no logs itself; the caller maps the
/// returned decisions onto `tracing`.
#[derive(Debug, Default)]
pub struct WanDropLatch {
    /// True while in a dropping regime (≥1 drop since the last push with room).
    dropping: bool,
}

impl WanDropLatch {
    /// Construct a latch in the not-dropping (healthy) state.
    pub const fn new() -> Self {
        Self { dropping: false }
    }

    /// Record a drop; returns the level to log it at. First drop of a regime →
    /// [`WanDropLevel::Warn`] and the latch sets; repeats → [`WanDropLevel::Debug`].
    pub fn on_drop(&mut self) -> WanDropLevel {
        if self.dropping {
            WanDropLevel::Debug
        } else {
            self.dropping = true;
            WanDropLevel::Warn
        }
    }

    /// Record a push that found ROOM (no drop). Returns `true` exactly when this
    /// ends a dropping regime (the caller logs a recovery `info!` once). A push
    /// with room while already healthy returns `false` (the steady state stays
    /// log-free).
    pub fn on_room(&mut self) -> bool {
        std::mem::replace(&mut self.dropping, false)
    }

    /// Whether the latch is currently in a dropping regime (Principle #3).
    pub fn is_dropping(&self) -> bool {
        self.dropping
    }
}

/// The lock-free per-topic counters exposed on the `status` verb. `forwarded` is
/// bumped by the writer task (frames actually written to QUIC); `dropped` +
/// `frames_seen` + the Hz timestamps are bumped by the drain thread. Hz is
/// derived from the wire `timestamp_ns` deltas already carried in each frame —
/// FREE (no clock syscall on the drain hot path).
#[derive(Debug, Default)]
pub struct TopicCounters {
    forwarded: AtomicU64,
    dropped: AtomicU64,
    frames_seen: AtomicU64,
    /// Frames carrying a NON-ZERO wire timestamp — the count that spans the Hz
    /// window. Distinct from `frames_seen` (which counts EVERY frame, including
    /// ones with an unparseable/zero header): the Hz numerator MUST cover the
    /// same set of frames as the `[first_ts, last_ts]` denominator, else a
    /// zero-timestamp frame inflates the reported rate.
    timestamped_seen: AtomicU64,
    /// The wire `timestamp_ns` of the FIRST timestamped frame seen (0 = unset).
    first_ts: AtomicU64,
    /// The wire `timestamp_ns` of the LAST timestamped frame seen (0 = unset).
    last_ts: AtomicU64,
}

impl TopicCounters {
    /// Record one frame READ off SHM (the drain side): bump `frames_seen` and,
    /// when the frame carries a non-zero wire timestamp, bump `timestamped_seen`
    /// and advance the Hz window (first_ts is set once via compare-exchange;
    /// last_ts always advances).
    pub fn record_frame(&self, wire_ts: u64) {
        self.frames_seen.fetch_add(1, Ordering::Relaxed);
        if wire_ts != 0 {
            // Count + span only TIMESTAMPED frames, so the Hz numerator and the
            // [first_ts, last_ts] denominator cover the same frames.
            self.timestamped_seen.fetch_add(1, Ordering::Relaxed);
            // Set first_ts to the first non-zero timestamp seen, once.
            let _ =
                self.first_ts
                    .compare_exchange(0, wire_ts, Ordering::Relaxed, Ordering::Relaxed);
            self.last_ts.store(wire_ts, Ordering::Relaxed);
        }
    }

    /// Record one frame WRITTEN to QUIC (the writer side).
    pub fn record_forwarded(&self) {
        self.forwarded.fetch_add(1, Ordering::Relaxed);
    }

    /// Record one frame DROPPED at the bounded queue (the drain side).
    pub fn record_dropped(&self) {
        self.dropped.fetch_add(1, Ordering::Relaxed);
    }

    /// Frames written to QUIC.
    pub fn forwarded(&self) -> u64 {
        self.forwarded.load(Ordering::Relaxed)
    }

    /// Frames dropped at the bounded queue (WAN slower than the topic).
    pub fn dropped(&self) -> u64 {
        self.dropped.load(Ordering::Relaxed)
    }

    /// Frames read off SHM (= forwarded + dropped + currently queued).
    pub fn frames_seen(&self) -> u64 {
        self.frames_seen.load(Ordering::Relaxed)
    }

    /// The topic's publish rate in Hz, derived from the wire-timestamp span of
    /// the TIMESTAMPED frames. `0.0` until ≥2 timestamped frames span a positive
    /// interval. FREE — reads only the wire timestamps already carried in the
    /// frames, never a clock syscall. Uses `timestamped_seen` (NOT `frames_seen`)
    /// as the numerator so the count and the `[first_ts, last_ts]` span cover the
    /// SAME frames — a zero-timestamp frame never inflates the rate.
    pub fn hz(&self) -> f64 {
        let seen = self.timestamped_seen.load(Ordering::Relaxed);
        if seen < 2 {
            return 0.0;
        }
        let first = self.first_ts.load(Ordering::Relaxed);
        let last = self.last_ts.load(Ordering::Relaxed);
        if first == 0 || last <= first {
            return 0.0;
        }
        (seen - 1) as f64 * 1_000_000_000.0 / (last - first) as f64
    }
}

/// The bounded drop-to-live forward channel shared between the drain thread (the
/// producer, sync) and the writer task (the consumer, async).
///
/// The deque + latch live under one `std::sync::Mutex`; the counters are
/// lock-free atomics; a `Notify` wakes the writer. On [`Self::push`] the deque
/// never exceeds [`WAN_QUEUE_CAP`] — a push at capacity evicts the OLDEST frame
/// first (drop-to-live) and reports the drop via the latch.
#[derive(Debug)]
pub struct ForwardChannel {
    inner: Mutex<ForwardInner>,
    notify: Notify,
    counters: TopicCounters,
    /// Set by the owner at teardown; the writer + drain loops observe it and
    /// exit (the drain thread dropping its tap releases the SHM subscriber slot).
    stop: AtomicBool,
}

#[derive(Debug, Default)]
struct ForwardInner {
    deque: VecDeque<Vec<u8>>,
    latch: WanDropLatch,
}

/// The outcome of one [`ForwardChannel::push`], returned so the (log-free) queue
/// core stays oracle-testable and the caller owns the `tracing` mapping.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PushOutcome {
    /// The frame was queued with room to spare (no drop). `recovered` is `true`
    /// exactly when this push ended a prior dropping regime (log a recovery
    /// `info!` once).
    Queued { recovered: bool },
    /// The queue was full: the OLDEST frame was evicted to make room for this
    /// one (drop-to-live). `level` is the flood-latch level to log the drop at.
    DroppedOldest { level: WanDropLevel },
}

impl ForwardChannel {
    /// A fresh, empty, not-stopped channel.
    pub fn new() -> Self {
        Self {
            inner: Mutex::new(ForwardInner::default()),
            notify: Notify::new(),
            counters: TopicCounters::default(),
            stop: AtomicBool::new(false),
        }
    }

    /// The per-topic counters (for the `status` verb + tests).
    pub fn counters(&self) -> &TopicCounters {
        &self.counters
    }

    /// Push one owned wire frame (drop-to-live). Updates the Hz counters, then:
    /// if the deque is at [`WAN_QUEUE_CAP`], evicts the OLDEST frame + counts the
    /// drop + returns [`PushOutcome::DroppedOldest`]; else queues with room and
    /// returns [`PushOutcome::Queued`]. Always wakes the writer. Never blocks —
    /// so the drain thread never back-pressures into SHM (Principle: taps are
    /// observation-only).
    ///
    /// `wire_ts` is the frame's `WireHeader::timestamp_ns` (0 if unparseable),
    /// fed to the free Hz derivation.
    pub fn push(&self, frame: Vec<u8>, wire_ts: u64) -> PushOutcome {
        self.counters.record_frame(wire_ts);
        let outcome = {
            // Poisoned only if a holder panicked mid-mutation; recover the guard
            // (the deque/latch are plain data — a poisoned lock is not fatal here).
            let mut inner = self.inner.lock().unwrap_or_else(|e| e.into_inner());
            if inner.deque.len() >= WAN_QUEUE_CAP {
                inner.deque.pop_front(); // drop the OLDEST (drop-to-live)
                let level = inner.latch.on_drop();
                inner.deque.push_back(frame);
                PushOutcome::DroppedOldest { level }
            } else {
                let recovered = inner.latch.on_room();
                inner.deque.push_back(frame);
                PushOutcome::Queued { recovered }
            }
        };
        if matches!(outcome, PushOutcome::DroppedOldest { .. }) {
            self.counters.record_dropped();
        }
        self.notify.notify_one();
        outcome
    }

    /// Move ALL currently-queued frames into `out` (appended, oldest-first),
    /// leaving the deque empty. The writer takes them and `write_frame`s each.
    pub fn take_into(&self, out: &mut Vec<Vec<u8>>) {
        let mut inner = self.inner.lock().unwrap_or_else(|e| e.into_inner());
        out.extend(inner.deque.drain(..));
    }

    /// Whether the deque is currently empty (the writer uses it with `stop` to
    /// decide when it may finish).
    pub fn is_empty(&self) -> bool {
        self.inner
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .deque
            .is_empty()
    }

    /// Signal both loops to stop, and wake the writer so it observes the flag
    /// promptly (no reliance on its periodic backstop).
    pub fn stop(&self) {
        self.stop.store(true, Ordering::Release);
        self.notify.notify_waiters();
    }

    /// Whether teardown has been requested.
    pub fn is_stopped(&self) -> bool {
        self.stop.load(Ordering::Acquire)
    }

    /// Await the writer's wake condition: a push, a teardown, or the backstop
    /// timeout (a belt-and-suspenders wake so a lost `notify_one` never wedges).
    pub async fn wait_writable(&self, backstop: std::time::Duration) {
        tokio::select! {
            _ = self.notify.notified() => {}
            _ = tokio::time::sleep(backstop) => {}
        }
    }
}

impl Default for ForwardChannel {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// `WanDropLatch` full cycle: first drop warns, repeats debug, a push with
    /// room recovers once, the next drop warns again. Hand oracle (crib the
    /// `DrainWarnLatch` cycle pin) — pins both inversions in one body.
    #[test]
    fn wan_drop_latch_warn_debug_recover_warn() {
        let mut latch = WanDropLatch::new();
        assert!(!latch.is_dropping());

        assert_eq!(latch.on_drop(), WanDropLevel::Warn);
        assert!(latch.is_dropping());
        assert_eq!(latch.on_drop(), WanDropLevel::Debug);
        assert_eq!(latch.on_drop(), WanDropLevel::Debug);

        assert!(latch.on_room(), "first room after drops = recovery");
        assert!(!latch.is_dropping());
        // A second room while healthy is silent.
        assert!(!latch.on_room());

        // Re-armed: the next drop WARNS again (kills a never-warn-again inversion).
        assert_eq!(latch.on_drop(), WanDropLevel::Warn);
    }

    /// A push that finds room while healthy reports no recovery (nothing to
    /// recover from) — the steady state stays log-free.
    #[test]
    fn wan_drop_latch_room_while_healthy_is_silent() {
        let mut latch = WanDropLatch::default();
        assert!(!latch.on_room());
        assert!(!latch.on_room());
    }

    /// Drop-to-live oracle: fill the queue to `WAN_QUEUE_CAP`, then push 3 more.
    /// EXACTLY 3 frames drop (the 3 OLDEST), the deque holds the FRESHEST
    /// `WAN_QUEUE_CAP`, and the drop count is exact. The freshest frame is at the
    /// back. Hand oracle — never a self-compare.
    #[test]
    fn push_drops_oldest_keeping_the_freshest_window() {
        let ch = ForwardChannel::new();
        // A hand vector of distinct frames 0..(CAP+3), each a single tagged byte
        // plus a valid-length body so the payload is non-trivial.
        let total = WAN_QUEUE_CAP + 3;
        for i in 0..total {
            let frame = vec![i as u8; 4];
            let outcome = ch.push(frame, (i as u64) + 1);
            if i < WAN_QUEUE_CAP {
                assert!(
                    matches!(outcome, PushOutcome::Queued { .. }),
                    "push {i} had room"
                );
            } else {
                assert!(
                    matches!(outcome, PushOutcome::DroppedOldest { .. }),
                    "push {i} evicts the oldest"
                );
            }
        }
        // Exactly 3 drops; frames_seen counts every push.
        assert_eq!(ch.counters().dropped(), 3);
        assert_eq!(ch.counters().frames_seen(), total as u64);

        // The deque holds the freshest WAN_QUEUE_CAP frames: bytes 3..(CAP+3).
        let mut drained = Vec::new();
        ch.take_into(&mut drained);
        assert_eq!(drained.len(), WAN_QUEUE_CAP, "queue capped");
        let got: Vec<u8> = drained.iter().map(|f| f[0]).collect();
        let oracle: Vec<u8> = (3u8..(WAN_QUEUE_CAP as u8 + 3)).collect();
        assert_eq!(got, oracle, "freshest window survives, oldest dropped");
        // The very freshest frame is at the back.
        assert_eq!(
            *drained.last().unwrap(),
            vec![WAN_QUEUE_CAP as u8 + 2; 4],
            "freshest frame is last"
        );
    }

    /// The push flood-latch level sequence for a sustained stall: the FIRST
    /// overflow is `Warn`, every subsequent overflow is `Debug` (no re-arm while
    /// the queue stays full). Hand oracle for the once-per-regime contract at the
    /// `push` seam.
    #[test]
    fn sustained_overflow_warns_once_then_debug() {
        let ch = ForwardChannel::new();
        for _ in 0..WAN_QUEUE_CAP {
            assert!(matches!(ch.push(vec![0u8], 0), PushOutcome::Queued { .. }));
        }
        // First overflow → Warn.
        assert_eq!(
            ch.push(vec![1u8], 0),
            PushOutcome::DroppedOldest {
                level: WanDropLevel::Warn
            }
        );
        // Every further overflow (queue still full) → Debug.
        for _ in 0..5 {
            assert_eq!(
                ch.push(vec![2u8], 0),
                PushOutcome::DroppedOldest {
                    level: WanDropLevel::Debug
                }
            );
        }
        assert_eq!(ch.counters().dropped(), 6);
    }

    /// A recovering flapper: overflow (Warn), drain to room, push with room
    /// (recovered), overflow again (Warn again). Pins that the `push`-seam
    /// re-arms on a room push — the drop→forward→drop cadence warns per regime,
    /// not per drop.
    #[test]
    fn recovery_rearm_at_push_seam() {
        let ch = ForwardChannel::new();
        for _ in 0..WAN_QUEUE_CAP {
            let _ = ch.push(vec![0u8], 0);
        }
        assert_eq!(
            ch.push(vec![1u8], 0),
            PushOutcome::DroppedOldest {
                level: WanDropLevel::Warn
            }
        );
        // Drain everything → the next push has room and RECOVERS.
        let mut sink = Vec::new();
        ch.take_into(&mut sink);
        assert_eq!(
            ch.push(vec![2u8], 0),
            PushOutcome::Queued { recovered: true }
        );
        // A second room push is silent.
        assert_eq!(
            ch.push(vec![3u8], 0),
            PushOutcome::Queued { recovered: false }
        );

        // Drain again so the re-fill below starts from an empty deque, then
        // re-fill to capacity (all room) + overflow → WARN again (a new regime).
        sink.clear();
        ch.take_into(&mut sink);
        for _ in 0..WAN_QUEUE_CAP {
            assert!(matches!(ch.push(vec![4u8], 0), PushOutcome::Queued { .. }));
        }
        assert_eq!(
            ch.push(vec![5u8], 0),
            PushOutcome::DroppedOldest {
                level: WanDropLevel::Warn
            }
        );
    }

    /// Hz derivation from wire timestamps: 11 frames spanning 100 ms of wire time
    /// (10 ms apart) → ~100 Hz. Hand oracle, no clock reads. `frames_seen < 2`
    /// and a zero/degenerate span both read 0.0.
    #[test]
    fn hz_from_wire_timestamps_is_free_and_exact() {
        let counters = TopicCounters::default();
        assert_eq!(counters.hz(), 0.0, "0 frames → 0 Hz");
        // One frame at t=1ms: still 0 (need ≥2 to span an interval).
        counters.record_frame(1_000_000);
        assert_eq!(counters.hz(), 0.0, "1 frame → 0 Hz");
        // 10 more frames 10ms apart: first_ts=1ms, last_ts=101ms, 11 frames.
        for k in 1..=10u64 {
            counters.record_frame(1_000_000 + k * 10_000_000);
        }
        // (11-1) frames over (101-1)ms = 10 / 0.1s = 100 Hz.
        assert!(
            (counters.hz() - 100.0).abs() < 1e-6,
            "hz={}, expected 100.0",
            counters.hz()
        );
    }

    /// A frame with an unparseable header (wire_ts == 0) never corrupts the Hz
    /// window: first_ts stays the first NON-zero timestamp AND hz() is NOT
    /// inflated by the untimestamped frame (the numerator counts only
    /// timestamped frames, matching the [first_ts,last_ts] span). Hand oracle.
    #[test]
    fn zero_timestamp_frames_do_not_poison_the_hz_window() {
        let counters = TopicCounters::default();
        counters.record_frame(0); // corrupt/unparseable header
        counters.record_frame(5_000_000); // first real ts
        counters.record_frame(15_000_000); // +10ms
        assert_eq!(counters.first_ts.load(Ordering::Relaxed), 5_000_000);
        assert_eq!(counters.last_ts.load(Ordering::Relaxed), 15_000_000);
        // frames_seen counts all 3, but hz uses timestamped_seen == 2 over the
        // 10ms span → 100 Hz. The pre-f6 code used seen==3 → (3-1)/10ms = 200 Hz
        // (2× inflated). Reverting hz()'s numerator to
        // frames_seen fails this exact assertion.
        assert_eq!(counters.frames_seen(), 3);
        assert!(
            (counters.hz() - 100.0).abs() < 1e-6,
            "hz={} must be 100 (2 timestamped frames / 10ms), NOT 200 (inflated by the ts=0 frame)",
            counters.hz()
        );
    }
}
