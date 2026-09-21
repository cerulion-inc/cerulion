// SPDX-License-Identifier: AGPL-3.0-only
//! The never-block viz LOGGING worker — a dedicated thread that owns
//! the Rerun `RecordingStream` and does the (potentially BLOCKING) `rec.log`
//! work OFF the caller's poll thread.
//!
//! (The graph-embedded `rerun_sink` node this was originally
//! lifted out of has since been deleted; the one caller today is the `cerulion-vizd` daemon's render
//! loop, which polls its taps on a fixed cadence. "Tick" below is that poll,
//! not a `NodeEntry::tick`.)
//!
//! # Why (the demo freeze incident)
//!
//! A terminal viz consumer must NEVER block on its output path. The Rerun 0.34
//! SDK's log path is backpressure-ONLY: `rec.log` → the chunk batcher's
//! `re_quota_channel` (byte-bounded, BLOCKING `send`) → the forwarding thread →
//! `GrpcSink::send` → the gRPC client's bounded (100-msg) channel (BLOCKING
//! `send_blocking`). There is NO drop-on-full configuration anywhere in the SDK
//! (`ChunkBatcherConfig::max_bytes_in_flight` only sets WHERE the block happens;
//! the gRPC client channel is a fixed `mpsc(100)` — both confirmed by reading
//! `re_chunk::batcher` / `re_quota_channel::sync` / `re_grpc_client::write`).
//! So when the viewer/server side wedges (a suspended browser tab is enough —
//! the rerun server stops draining the gRPC stream), the whole pipeline fills
//! and `rec.log` blocks INDEFINITELY (the observed `re_quota_channel` "Sender
//! has been blocked for over 5 seconds" warns). In the demo where this happened it froze the
//! sink's `tick()` inside `rec.log`, and the whole viz chain froze with it.
//!
//! # The seam (simple-is-better)
//!
//! Since the SDK cannot be made to drop, the DROP is moved to a bounded queue
//! we own, at the SEMANTIC layer (whole per-tick viz batches), matching the
//! per-tick latest-wins coalescing philosophy (the newest state is the
//! only displayable state). The sink `tick()` drains its subscribers (the
//! cheap, non-blocking data-plane read) and hands a per-tick batch to this
//! worker via a bounded [`std::sync::mpsc::sync_channel`]: `try_send` returns
//! IMMEDIATELY, so the tick is non-blocking BY CONSTRUCTION. A FULL queue (the
//! worker blocked in `rec.log` because the viewer wedged) ⇒ the batch is
//! DROPPED, counted, and logged loud-once (the house [`FieldsWarnLatch`]
//! contract: first drop of a regime `warn!`, repeats `debug!` with a running
//! count, recovery `info!` when the queue drains again). The worker owns the
//! `RecordingStream`, the [`FrameWalker`], and the [`SinkState`] and runs the
//! EXACT existing dispatch ([`dispatch_or_stage`] / [`dispatch_frame`] +
//! statics + blueprint) — the schema→archetype code is unchanged, only
//! RELOCATED off the tick thread.
//!
//! The scene statics / blueprint are generated ON the worker (past the drop
//! point), so a drop never drops a control/static message — it only drops
//! sensor frames. `rec.log` blocking on the worker is by DESIGN: the worker is
//! the dedicated blocking thread; the tick stays free.

use std::collections::BTreeMap;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::mpsc::{sync_channel, Receiver, RecvTimeoutError, SyncSender, TrySendError};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use cerulion_core::codegen::FrameWalker;
use rerun::sink::SinkFlushError;
use rerun::RecordingStream;

use crate::blueprint::{apply_runtime_blueprint, send_blueprint_once, BlueprintPlan};
use crate::pointcloud::{FieldsLogAction, FieldsWarnLatch};
use crate::representation::Representation;
use crate::sink::{dispatch_frame, dispatch_or_stage, RenderProof, SinkState};
use crate::tf::log_viz_statics_once;

/// Bounded viz-batch queue depth (per-tick batches). At the sink's ~60 Hz poll
/// this is ≈130 ms of buffering — enough to ride out normal scheduling jitter,
/// small enough that a genuinely wedged viewer fills it (and the sink starts
/// DROPPING) within a fraction of a second instead of freezing.
const VIZ_QUEUE_CAP: usize = 8;

/// Health-probe cadence: how often the worker checks the sink for a dead gRPC
/// connection AND its idle wake interval (bounds statics-setup retry latency).
/// Doubles as the reconnect backoff — at most one reconnect attempt per probe.
const PROBE_INTERVAL: Duration = Duration::from_secs(5);

/// Timeout for the reconnect health-probe flush. SHORT so a still-`Connecting`
/// sink (a fresh reconnect in progress, or a slow-but-alive viewer) returns
/// `Timeout` (⇒ do NOT reconnect) rather than `Failed`; only a genuinely
/// `Disconnected` sink returns `Failed` fast (⇒ reconnect). See
/// [`should_reconnect`].
const PROBE_FLUSH_TIMEOUT: Duration = Duration::from_secs(1);

/// Bounded shutdown budget for joining the worker on `Drop`. A HEALTHY worker
/// drains its backlog + does its final flush (bounded by the teardown flush
/// timeout below) well within this; a worker WEDGED in `rec.log` (viewer stuck)
/// can NEVER drain, so after this budget it is DETACHED rather than hanging the
/// graph's shutdown forever (the earlier `flush_blocking(MAX)` teardown hung
/// indefinitely in exactly this case). The detached thread dies at process exit.
const SHUTDOWN_JOIN_TIMEOUT: Duration = Duration::from_secs(6);

/// Bounded final flush on a clean shutdown (rendering + delivering the tail).
const TEARDOWN_FLUSH_TIMEOUT: Duration = Duration::from_secs(5);

/// Bounded budget for handing a CONTROL message (a runtime blueprint) to the
/// worker. A control message must not be SILENTLY dropped (unlike a per-tick data
/// batch) but must ALSO not block the controller thread FOREVER if the worker is
/// wedged in `rec.log` (viewer stuck). A short bounded retry over the shared queue
/// absorbs a momentary data-batch backlog; past this budget [`VizControl::set_blueprint`]
/// returns an explicit [`VizControlError::Busy`] the caller surfaces (never a hang,
/// never a silent drop).
const CONTROL_SEND_TIMEOUT: Duration = Duration::from_secs(1);

/// Why a [`VizControl::set_blueprint`] could not reach the worker (surfaced
/// VERBATIM by the vizd `set_blueprint` verb — a precise, actionable error, never
/// a silent no-op).
#[derive(Debug, thiserror::Error)]
pub enum VizControlError {
    /// The bounded `CONTROL_SEND_TIMEOUT` elapsed with the worker's queue still
    /// full — the worker is not draining (viewer wedged). The layout was NOT
    /// applied; retry once the viewer recovers.
    #[error("the viz worker is busy (queue full for over {CONTROL_SEND_TIMEOUT:?}) — layout not applied, retry")]
    Busy,
    /// The worker thread is gone (a panic escaped its own containment, or the
    /// daemon is tearing down). Visualization is off for the rest of the run.
    #[error("the viz worker thread is gone — visualization is off")]
    WorkerGone,
}

/// A cloneable CONTROL handle to the never-block worker, held by the vizd daemon
/// so a controller thread can push a runtime layout (the `set_blueprint` verb) to
/// the worker — the ONE thread that owns the `RecordingStream` and is allowed to
/// block on it. Wraps the worker's queue sender behind a
/// `Mutex<Option<_>>` so the daemon can [`close`](Self::close) it at shutdown,
/// releasing the worker's `Disconnected`-driven clean exit even while other
/// controller threads still hold an `Arc<VizControl>` (the sender inside is then
/// `None`, so it no longer keeps the worker's `recv` alive).
///
/// `Debug` so `RunningDaemon` (which holds an `Arc<VizControl>`) keeps its derive
/// (`SyncSender`'s `Debug` is unconditional, so the inner `VizMsg` need not be).
#[derive(Debug)]
pub struct VizControl {
    tx: Mutex<Option<SyncSender<VizMsg>>>,
}

impl VizControl {
    /// Hand `plan` to the worker to install as the runtime blueprint. Bounded
    /// (never blocks forever) + non-silent: `Ok(())` once the message is enqueued,
    /// else a structured [`VizControlError`]. The apply itself happens on the
    /// worker thread; this only enqueues (FIFO behind any queued data batches, so
    /// the layout applies after at most the shallow queue's worth of frames).
    pub fn set_blueprint(&self, plan: BlueprintPlan) -> Result<(), VizControlError> {
        self.send(VizMsg::SetBlueprint(plan))
    }

    /// Hand `representation` to the worker as `input_key`'s override.
    /// Same bounded, non-silent delivery contract as [`Self::set_blueprint`] —
    /// the vizd `representation` verb surfaces the error VERBATIM rather than
    /// reporting a success the render never saw.
    pub fn set_representation(
        &self,
        input_key: &str,
        representation: Representation,
    ) -> Result<(), VizControlError> {
        self.send(VizMsg::SetRepresentation {
            input_key: input_key.to_string(),
            representation,
        })
    }

    /// Enqueue one control message, bounded and non-silent.
    fn send(&self, msg: VizMsg) -> Result<(), VizControlError> {
        // Clone the sender OUT from under the lock so a slow send never serializes
        // `close()` (or another controller) behind it.
        let tx = {
            let guard = self.tx.lock().unwrap();
            guard.clone()
        };
        let Some(tx) = tx else {
            return Err(VizControlError::WorkerGone);
        };
        // std's `SyncSender` has no `send_timeout`, so bounce off `try_send` with a
        // bounded deadline: on `Full`, `try_send` HANDS THE MESSAGE BACK — reclaim
        // it and retry until CONTROL_SEND_TIMEOUT, then report Busy.
        let deadline = Instant::now() + CONTROL_SEND_TIMEOUT;
        let mut msg = msg;
        loop {
            match tx.try_send(msg) {
                Ok(()) => return Ok(()),
                Err(TrySendError::Full(returned)) => {
                    if Instant::now() >= deadline {
                        return Err(VizControlError::Busy);
                    }
                    msg = returned;
                    std::thread::sleep(Duration::from_millis(5));
                }
                Err(TrySendError::Disconnected(_)) => return Err(VizControlError::WorkerGone),
            }
        }
    }

    /// Release the worker's queue sender this handle holds (idempotent). The
    /// daemon calls this at shutdown START so the worker's `recv` can reach
    /// `Disconnected` — and run its bounded teardown flush — once the poll thread
    /// drops the worker, even though `Arc<VizControl>` clones survive on other
    /// controller threads (the sender inside is now `None`).
    pub fn close(&self) {
        *self.tx.lock().unwrap() = None;
    }
}

/// One poll tick's worth of viz work: the raw wire frames drained from each
/// wired input, grouped by input NAME so the worker can run the exact per-input
/// coalescing loop the sink's tick used to run inline.
pub struct InputFrames {
    /// The wired input name (selects the render route + coalescing scratch).
    pub name: String,
    /// The full wire frames (WireHeader + payload) drained this tick,
    /// accumulate-all in arrival order.
    pub frames: Vec<Vec<u8>>,
}

impl InputFrames {
    fn frame_count(&self) -> u64 {
        self.frames.len() as u64
    }
}

/// Messages down the bounded queue: a per-tick batch, or a test-only barrier
/// (a rendezvous `sync_channel` the worker acks once it has processed every
/// prior batch — lets tests observe the async worker's effect deterministically
/// without sleeps).
enum VizMsg {
    Batch(Vec<InputFrames>),
    /// Atomically replace the worker's [`FrameWalker`] between
    /// batches. The dynamic viz daemon owns the schema universe and, when it
    /// learns a schema the current walker cannot decode (a remote type served
    /// over the `cerulion_q` queryable; this message only adds the plumbing,
    /// no network yet), rebuilds the walker and hands it down this channel. In
    /// FIFO order with `Batch`, so frames enqueued after the swap decode against
    /// the new walker and frames before it against the old — deterministic. The
    /// `FrameWalker` is two `BTreeMap`s (~48 bytes), so the enum stays small.
    SwapWalker(FrameWalker),
    /// Runtime layout: apply a client-driven blueprint (the
    /// `set_blueprint` verb). Handled ON the worker — the ONE thread that owns
    /// the `RecordingStream` and is allowed to block on it — by
    /// [`apply_runtime_blueprint`], which remembers the layout (so a reconnect
    /// re-applies it) and sends it, superseding the active blueprint. FIFO with
    /// `Batch`, so it applies after any queued frames.
    SetBlueprint(BlueprintPlan),
    /// Set (or clear) one input's REPRESENTATION override — the
    /// operator's choice of how that topic renders. Handled on the worker for
    /// the same reason `SetBlueprint` is: the [`SinkState`] the choice lives in
    /// is owned there, and FIFO with `Batch` means the choice applies to every
    /// frame enqueued after it and to none before — deterministic, and what an
    /// operator watching the panel expects.
    SetRepresentation {
        /// The sink's per-input key (the tap's route key), not the bare topic.
        input_key: String,
        /// The choice; [`Representation::Auto`] CLEARS the override.
        representation: Representation,
    },
    Barrier(SyncSender<()>),
    /// Test-only: park the worker (confirm via `reached`, then block on
    /// `release`) so a unit test can deterministically fill the bounded queue
    /// and pin the drop accounting — faithfully reproducing "the worker is
    /// stuck in `rec.log` and not draining", without a real wedged sink.
    #[cfg(test)]
    Block {
        reached: SyncSender<()>,
        release: Receiver<()>,
    },
    /// Test-only: panic while handling this message, to pin that the worker's
    /// `catch_unwind` contains a render-path panic instead of dying.
    #[cfg(test)]
    PanicForTest,
}

/// Observability counters shared between the worker thread (writer) and the
/// OWNER (reader, for teardown surfacing + Principle #3 queryability +
/// tests). All `Relaxed` — these are monotone diagnostics, never a
/// synchronization edge.
#[derive(Debug, Default)]
pub struct VizWorkerCounters {
    /// Per-tick batches dropped because the queue was full (viewer wedged).
    pub dropped_batches: AtomicU64,
    /// Total viz FRAMES inside the dropped batches — the operator-facing "how
    /// many frames did the wedge cost" signal.
    pub dropped_frames: AtomicU64,
    /// Mirror of [`SinkState::coalesced_frames`] (frames coalesced away by the
    /// newest-per-tick rendering), refreshed after each processed batch so the
    /// node can surface it at `shutdown()` without owning the state.
    pub coalesced_frames: AtomicU64,
    /// Live gRPC reconnects performed after a detected sink disconnect.
    pub reconnects: AtomicU64,
    /// Panics CAUGHT in the worker's per-iteration render/probe path (a
    /// malformed frame that panics the walker, an SDK-internal panic, etc.). The
    /// worker CONTAINS the panic and continues — viz is never silently lost — so
    /// this is the operator signal that some frames failed to render.
    pub render_panics: AtomicU64,
    /// Mirror of [`crate::video::VideoDemux::rendition_segments`] —
    /// per-input H.264 rendition segments (`"640x360"`), refreshed after each
    /// processed batch beside [`VizWorkerCounters::coalesced_frames`].
    ///
    /// The LAYOUT plane needs it and cannot reach it otherwise: the demux lives in
    /// the worker's [`SinkState`], while the daemon assembles the
    /// `AttachedRender`s both layout producers consume. Without this mirror an
    /// interleaved topic's renditions share ONE `spatial2d` view and OVERLAY —
    /// each stream decodes correctly and you still cannot see one of them.
    ///
    /// A `Mutex` rather than atomics because the value is a map; it is written
    /// once per batch and read only on the (low-frequency) control path, never on
    /// the render hot path.
    pub video_renditions: Mutex<BTreeMap<String, Vec<String>>>,
    /// Mirror of [`SinkState::render_proofs`] — per-input record of what
    /// each topic's render arm has been observed to do, refreshed after each
    /// processed batch beside [`VizWorkerCounters::video_renditions`].
    ///
    /// Same reason as the rendition mirror, one archetype-family wider: the render
    /// arms run HERE and the layout that gates the dump companion is
    /// assembled on the daemon's control thread, so this is the only path between
    /// them. Without it the companion could only ever be refused for video, which
    /// is the video-only half the render proof generalizes.
    ///
    /// A `Mutex` for the same reason as its neighbour: the value is a map, written
    /// once per batch and read only on the (low-frequency) control path, never on
    /// the render hot path.
    pub render_proofs: Mutex<BTreeMap<String, RenderProof>>,
    /// A monotone counter bumped whenever EITHER layout-signal
    /// mirror above publishes a value that DIFFERS from what it held — the one
    /// thing the daemon's drain loop has to watch to know its default layout may
    /// need re-deriving.
    ///
    /// **It covers BOTH mirrors because the layout signal is per-ARCHETYPE.**
    /// `blueprint::render_is_proven` reads `video_renditions` for
    /// [`crate::sink::ArchetypeKind::VideoStream`] and
    /// [`RenderProof::rendered_without_dumping`] for every other kind, so watching
    /// the proof map alone is blind on exactly the topic class the video refusal was raised
    /// for: a camera attached MID-GOP latches its proof on the first pre-keyframe
    /// access unit (the `BeforeKeyframe` drop is not a degradation, so
    /// `note_render_native` runs) and reaches its TERMINAL proof value a second or
    /// more BEFORE the SPS opens the first rendition — so the map is byte-identical
    /// across the one transition the video layout decision actually turns on.
    /// A rendition set that later grows a SECOND entry (the Go2 ships a 640x360 +
    /// 1280x720 pair) changes the placement again and is watched here
    /// for the same reason.
    ///
    /// **Bumped by the WRITER, on a real change** — never per batch. That is what
    /// keeps the reflow BOUNDED (the doc claim on
    /// `Ctx::poll_layout_signal_reflows`): the proof flags are sticky and rendition
    /// sets only grow, so a topic contributes a bounded number of bumps and a
    /// steady stream contributes NONE. It also keeps the compare on this thread,
    /// where the map is being rebuilt anyway, instead of making the daemon's
    /// frame-drain loop clone and compare the whole map on every pass.
    ///
    /// Ordering is deliberate and one-sided: the bump happens AFTER the mirror
    /// lock is released, so a reader that observes a new generation and then locks
    /// the mirror sees at least that value. A reader that observes the OLD
    /// generation simply reflows one pass later — the safe direction.
    pub layout_signal_generation: AtomicU64,
}

impl VizWorkerCounters {
    /// Publish this batch's LAYOUT SIGNALS and report whether
    /// either changed.
    ///
    /// One seam for both mirrors, because they answer ONE question — "would the
    /// default layout come out differently now?" — and a caller that refreshed one
    /// without the other would re-create the watcher gap this exists to close.
    ///
    /// A poisoned lock is SKIPPED rather than propagated (a stale signal costs a
    /// `text_document` companion that is one batch late to appear or to go, never a dropped
    /// frame).
    ///
    /// **The skip is PER MIRROR, not per call.** The poisoned mirror contributes no
    /// `changed`, so nothing bumps ON ITS ACCOUNT — but a healthy sibling still can,
    /// and that arm is deliberate. The daemon reads the two mirrors through two
    /// INDEPENDENT lookups (`Ctx::video_renditions_for` / `Ctx::render_proof_for`),
    /// each of which degrades a poisoned lock to the SAME default the signal carries
    /// before anything has been observed — so a bump can never serve a reader a
    /// STALE value, only a conservative one (for proofs, the companion
    /// KEPT). Suppressing the bump instead would be strictly worse: `Mutex` poison
    /// is PERMANENT and nothing here clears it, so one poisoned mirror would freeze
    /// the default layout against every future change of the OTHER — the unbounded
    /// residual this watcher exists to close. There is no third option:
    /// consistency across the pair was never available, because the daemon never
    /// reads them under one lock.
    ///
    /// Reachability, stated because the arm reads like a live hazard and is not one:
    /// poisoning either mutex needs a thread to PANIC while holding it, and neither
    /// guard here spans an unwinding operation (a `BTreeMap` compare, a move-assign,
    /// a `bool`) nor does either reader (a map lookup behind
    /// `.lock().ok().and_then(..).unwrap_or_default()`). This is also the ONLY
    /// writer, on the worker thread, so a panic inside it would leave nothing to
    /// bump afterwards.
    pub fn publish_layout_signals(
        &self,
        renditions: BTreeMap<String, Vec<String>>,
        proofs: BTreeMap<String, RenderProof>,
    ) -> bool {
        let mut changed = false;
        if let Ok(mut held) = self.video_renditions.lock() {
            if *held != renditions {
                *held = renditions;
                changed = true;
            }
        }
        if let Ok(mut held) = self.render_proofs.lock() {
            if *held != proofs {
                *held = proofs;
                changed = true;
            }
        }
        if changed {
            self.layout_signal_generation
                .fetch_add(1, Ordering::Relaxed);
        }
        changed
    }
}

/// The pure reconnect DECISION. Reconnect ONLY on a genuine sink
/// connection failure (`Failed`), never on `Timeout` (a slow-but-alive viewer,
/// or a fresh reconnect still `Connecting`) or `Ok` (healthy) — so a merely
/// WEDGED-but-alive viewer is never needlessly reconnected (which would drop its
/// buffered data). Grounded in the 0.34 SDK: a `Disconnected` gRPC sink returns
/// `Failed` fast from a short-timeout flush; a `Connecting`/slow one returns
/// `Timeout`. Oracle-tested; the worker calls it on its probe timer.
pub fn should_reconnect(probe: &Result<(), SinkFlushError>) -> bool {
    matches!(probe, Err(SinkFlushError::Failed { .. }))
}

/// Sink-health probe: reports whether the gRPC sink is alive (production = a
/// short-timeout flush).
type ProbeFn = Box<dyn Fn(&RecordingStream) -> Result<(), SinkFlushError> + Send>;
/// Sink-swap action: install a fresh sink on the stream (production =
/// [`crate::stream::reconnect`]).
type ReconnectFn = Box<dyn Fn(&RecordingStream) -> Result<(), String> + Send>;

/// Injectable reconnect actions so the worker's reconnect ORCHESTRATION is
/// testable without a real gRPC server that bounces: `probe` reports sink health
/// (production = a short-timeout flush), `reconnect` swaps a fresh sink
/// (production = [`crate::stream::reconnect`]). The worker owns the app-level
/// re-arm (statics guards + `/tf_static` dedup) around these hooks.
pub struct ReconnectHooks {
    probe: ProbeFn,
    reconnect: ReconnectFn,
}

impl ReconnectHooks {
    /// Production hooks: a short-timeout flush probe + a live gRPC sink swap.
    pub fn production() -> Self {
        Self {
            probe: Box::new(|rec| rec.flush_with_timeout(PROBE_FLUSH_TIMEOUT)),
            reconnect: Box::new(crate::stream::reconnect),
        }
    }

    /// Inject custom probe + reconnect actions (an embedder seam, and the seam
    /// the reconnect orchestration tests use — e.g. a probe that always reports
    /// `Failed` and a reconnect that records its calls — to pin the worker's
    /// reconnect behavior deterministically without a real bouncing server).
    pub fn with_hooks(
        probe: impl Fn(&RecordingStream) -> Result<(), SinkFlushError> + Send + 'static,
        reconnect: impl Fn(&RecordingStream) -> Result<(), String> + Send + 'static,
    ) -> Self {
        Self {
            probe: Box::new(probe),
            reconnect: Box::new(reconnect),
        }
    }
}

/// The never-block viz logging worker handle. Owned by the render loop that
/// drives it (the `cerulion-vizd` daemon); the heavy
/// state (rec + walker + [`SinkState`]) lives on the spawned thread.
pub struct VizLogWorker {
    /// `Option` so `Drop` can drop the sender BEFORE joining (the worker's
    /// `recv` sees `Disconnected` once the queue drains → clean exit).
    tx: Option<SyncSender<VizMsg>>,
    handle: Option<JoinHandle<()>>,
    counters: Arc<VizWorkerCounters>,
    /// TICK-side loud-once drop latch (the enqueue happens on the tick thread,
    /// so its flood suppression lives here on the handle).
    drop_latch: FieldsWarnLatch,
    /// Set once the worker's receiver is observed GONE while the node is still
    /// ticking — i.e. the worker thread EXITED UNEXPECTEDLY (a panic escaped its
    /// own `catch_unwind`). Guards the one-time loud error so a dead worker is
    /// never a silent viz loss.
    worker_dead: bool,
}

impl VizLogWorker {
    /// Spawn the worker, moving `rec` + `walker` + `state` onto its dedicated
    /// thread. The node keeps only the enqueue handle + shared counters.
    /// Production hooks (short-timeout flush probe + live gRPC reconnect) at the
    /// default 5 s probe interval. Fallible: a thread-spawn failure
    /// (fd/thread exhaustion on a constrained robot) is returned so the sink can
    /// disable viz gracefully instead of aborting the tick.
    pub fn spawn(
        rec: RecordingStream,
        walker: FrameWalker,
        state: SinkState,
    ) -> std::io::Result<Self> {
        Self::spawn_with_hooks(
            rec,
            walker,
            state,
            ReconnectHooks::production(),
            PROBE_INTERVAL,
        )
    }

    /// Spawn with injected reconnect hooks + probe interval (test seam; also the
    /// single implementation [`Self::spawn`] delegates to).
    pub fn spawn_with_hooks(
        rec: RecordingStream,
        walker: FrameWalker,
        state: SinkState,
        hooks: ReconnectHooks,
        probe_interval: Duration,
    ) -> std::io::Result<Self> {
        let (tx, rx) = sync_channel::<VizMsg>(VIZ_QUEUE_CAP);
        let counters = Arc::new(VizWorkerCounters::default());
        let counters_worker = Arc::clone(&counters);
        let handle = std::thread::Builder::new()
            .name("go2-viz-log".to_string())
            .spawn(move || {
                run(
                    rec,
                    walker,
                    state,
                    rx,
                    counters_worker,
                    hooks,
                    probe_interval,
                )
            })?;
        Ok(Self {
            tx: Some(tx),
            handle: Some(handle),
            counters,
            drop_latch: FieldsWarnLatch::new(),
            worker_dead: false,
        })
    }

    /// A cloneable [`VizControl`] handle for pushing runtime blueprints (the
    /// `set_blueprint` verb) to the worker. The vizd daemon takes ONE handle
    /// (BEFORE moving the worker onto its thread) and shares it across its
    /// controller threads. `Arc`-wrapped so
    /// the daemon can `close()` the shared sender at shutdown.
    pub fn control(&self) -> Arc<VizControl> {
        Arc::new(VizControl {
            tx: Mutex::new(self.tx.clone()),
        })
    }

    /// A clone of the shared observability counters. The node holds this so it
    /// can read the FINAL totals at `shutdown()` AFTER dropping (joining) the
    /// worker — the drop drains the backlog, so reading through the handle
    /// (which is gone post-drop) would under-report the tail.
    pub fn counters(&self) -> Arc<VizWorkerCounters> {
        Arc::clone(&self.counters)
    }

    /// Live gRPC reconnects performed after a detected sink disconnect.
    /// Readable by tests / operators (Principle #3).
    pub fn reconnects(&self) -> u64 {
        self.counters.reconnects.load(Ordering::Relaxed)
    }

    /// Panics caught + contained in the worker's render path.
    pub fn render_panics(&self) -> u64 {
        self.counters.render_panics.load(Ordering::Relaxed)
    }

    /// Enqueue one tick's viz batch. NON-BLOCKING by construction: `try_send`
    /// returns immediately. A FULL queue (worker blocked in `rec.log` because
    /// the viewer wedged) DROPS the batch, counts it, and logs loud-once. An
    /// empty batch is a no-op (nothing to enqueue, no drop-latch churn).
    pub fn try_enqueue(&mut self, batch: Vec<InputFrames>) {
        if batch.iter().all(|i| i.frames.is_empty()) {
            return;
        }
        let Some(tx) = self.tx.as_ref() else {
            return;
        };
        let dropped: u64 = batch.iter().map(InputFrames::frame_count).sum();
        match tx.try_send(VizMsg::Batch(batch)) {
            Ok(()) => {
                // The viewer is draining again — heal the drop regime with one
                // recovery info carrying the total suppressed (None = never in a
                // regime, so a healthy stream is silent).
                if let Some(suppressed) = self.drop_latch.on_decoded() {
                    tracing::info!(
                        suppressed_count = suppressed,
                        total_dropped_frames = self.counters.dropped_frames.load(Ordering::Relaxed),
                        "cerulion_viz: viz queue draining again — viewer-wedge drop regime healed"
                    );
                }
            }
            Err(TrySendError::Full(_)) => {
                self.counters
                    .dropped_batches
                    .fetch_add(1, Ordering::Relaxed);
                let total = self
                    .counters
                    .dropped_frames
                    .fetch_add(dropped, Ordering::Relaxed)
                    + dropped;
                match self.drop_latch.on_inferred() {
                    FieldsLogAction::WarnFirst => tracing::warn!(
                        dropped_frames = dropped,
                        total_dropped_frames = total,
                        "cerulion_viz: viz consumer/viewer not draining — DROPPING viz frames to keep \
                         the sink ticking (the viz plane never blocks the sink; repeats log at \
                         debug until the queue drains)"
                    ),
                    FieldsLogAction::DebugSuppressed { suppressed } => tracing::debug!(
                        suppressed,
                        dropped_frames = dropped,
                        total_dropped_frames = total,
                        "cerulion_viz: viz frames still dropping (viewer wedged; warn suppressed)"
                    ),
                }
            }
            Err(TrySendError::Disconnected(_)) => {
                // The worker's receiver is gone. A CLEAN shutdown drops this
                // handle's sender (and `tick()` is not running then), so
                // reaching here from an enqueue means the worker thread EXITED
                // UNEXPECTEDLY — a panic that escaped its own `catch_unwind`.
                // Never silent: count the lost frames + loud-once error.
                self.counters
                    .dropped_batches
                    .fetch_add(1, Ordering::Relaxed);
                self.counters
                    .dropped_frames
                    .fetch_add(dropped, Ordering::Relaxed);
                if !self.worker_dead {
                    self.worker_dead = true;
                    tracing::error!(
                        "cerulion_viz: viz worker thread exited unexpectedly — visualization is off \
                         for the rest of the run; viz frames are now dropped + counted (never a \
                         tick failure)"
                    );
                }
            }
        }
    }

    /// Hand the worker a fresh [`FrameWalker`], which it swaps
    /// in atomically between batches (frames enqueued after this decode against
    /// the new walker, frames before it against the old — FIFO on the shared
    /// channel). The dynamic viz daemon calls this after learning a schema the
    /// current walker could not decode (e.g. a remote type seeded over the
    /// schema queryable) so an unknown-hash topic starts rendering.
    ///
    /// Uses a BLOCKING send (like [`Self::sync`], unlike the non-blocking
    /// [`Self::try_enqueue`]): a walker swap is a rare control message that must
    /// NOT be dropped when the queue is momentarily full — dropping it would
    /// silently keep the old (unknowing) walker and the topic would never render.
    /// A dead worker (receiver gone) makes the send fail; it is ignored (the
    /// enqueue path already surfaces a dead worker loud-once).
    pub fn swap_walker(&self, walker: FrameWalker) {
        let Some(tx) = self.tx.as_ref() else {
            return;
        };
        let _ = tx.send(VizMsg::SwapWalker(walker));
    }

    /// NON-BLOCKING walker swap for a caller that must NEVER block on the worker
    /// — the dynamic viz daemon's POLL THREAD, which also drains every
    /// runtime tap. A wedged viewer keeps this bounded queue FULL (the exact
    /// steady state [`try_enqueue`](Self::try_enqueue)'s drop machinery survives),
    /// so the blocking [`swap_walker`](Self::swap_walker) would park the poll
    /// thread INDEFINITELY → every tap stops draining, the whole viz plane hangs.
    /// This uses `try_send` instead: on success returns `None`; on a FULL queue
    /// returns `Some(walker)` so the caller RETAINS it and retries next iteration
    /// (the swap lands once the queue drains — a walker swap must never be dropped,
    /// unlike a data batch, or the topic would render against the old schema set);
    /// on a dead worker (`Disconnected`) returns `None` (dropped — the enqueue path
    /// already surfaces a dead worker loud-once).
    pub fn try_swap_walker(&self, walker: FrameWalker) -> Option<FrameWalker> {
        let Some(tx) = self.tx.as_ref() else {
            return None; // no worker / already closed — drop.
        };
        match tx.try_send(VizMsg::SwapWalker(walker)) {
            Ok(()) => None,
            Err(TrySendError::Full(VizMsg::SwapWalker(w))) => Some(w), // retain + retry
            // `try_send` only ever fails on the message we just handed it, so a
            // non-`SwapWalker` payload here is impossible; drop defensively.
            Err(TrySendError::Full(_)) => None,
            Err(TrySendError::Disconnected(_)) => None, // dead worker — drop.
        }
    }

    /// The running total of viz frames dropped because the viewer wedged
    /// (Principle #3 queryability; surfaced at the owner's teardown).
    pub fn dropped_frames(&self) -> u64 {
        self.counters.dropped_frames.load(Ordering::Relaxed)
    }

    /// The number of per-tick batches dropped (companion to
    /// [`Self::dropped_frames`]).
    pub fn dropped_batches(&self) -> u64 {
        self.counters.dropped_batches.load(Ordering::Relaxed)
    }

    /// Mirror of the worker's [`SinkState::coalesced_frames`] total (frames
    /// replaced by a newer frame in the same tick's batch), refreshed after
    /// each processed batch.
    pub fn coalesced_frames(&self) -> u64 {
        self.counters.coalesced_frames.load(Ordering::Relaxed)
    }

    /// Block until the worker has processed every batch enqueued so far (a
    /// rendezvous barrier). Test-only synchronization for the async worker —
    /// production never calls it (it is non-`cfg(test)` only because integration
    /// tests in another crate reach it). Uses a BLOCKING send so the barrier is
    /// never dropped even if the queue is momentarily full.
    #[doc(hidden)]
    pub fn sync(&self) {
        let Some(tx) = self.tx.as_ref() else {
            return;
        };
        let (ack_tx, ack_rx) = sync_channel::<()>(0);
        if tx.send(VizMsg::Barrier(ack_tx)).is_ok() {
            let _ = ack_rx.recv();
        }
    }

    /// Test-only: send a message that panics the worker's render path, to pin
    /// that the panic is CONTAINED (worker survives). Blocking send so it is
    /// never dropped.
    #[cfg(test)]
    fn inject_panic_for_test(&self) {
        if let Some(tx) = self.tx.as_ref() {
            tx.send(VizMsg::PanicForTest).expect("inject panic");
        }
    }

    /// Test-only: park the worker until the returned guard drops, so a test can
    /// deterministically fill the bounded queue and observe drops. Returns once
    /// the worker CONFIRMS it is parked (so a subsequent `try_enqueue` burst is
    /// racing nothing).
    #[cfg(test)]
    fn park_worker_for_test(&self) -> BlockGuard {
        let tx = self.tx.as_ref().expect("worker alive").clone();
        let (reached_tx, reached_rx) = sync_channel::<()>(1);
        let (release_tx, release_rx) = sync_channel::<()>(0);
        tx.send(VizMsg::Block {
            reached: reached_tx,
            release: release_rx,
        })
        .expect("send block");
        reached_rx.recv().expect("worker parks");
        BlockGuard {
            release: Some(release_tx),
        }
    }
}

/// RAII guard that keeps the worker parked (test-only); dropping it releases the
/// worker via a rendezvous send.
#[cfg(test)]
struct BlockGuard {
    release: Option<SyncSender<()>>,
}

#[cfg(test)]
impl Drop for BlockGuard {
    fn drop(&mut self) {
        if let Some(release) = self.release.take() {
            let _ = release.send(());
        }
    }
}

impl Drop for VizLogWorker {
    fn drop(&mut self) {
        // Drop the sender first so the worker's `recv` returns `Disconnected`
        // once the queued backlog drains — then join so a clean shutdown's tail
        // frames are rendered + flushed before the process moves on.
        self.tx = None;
        let Some(handle) = self.handle.take() else {
            return;
        };
        // BOUNDED join: a healthy worker finishes fast; a worker WEDGED in
        // `rec.log` (viewer stuck) can never drain, so rather than hang the
        // graph's shutdown forever we detach it after the budget (it dies at
        // process exit). This is the never-block guarantee extended to teardown.
        let deadline = Instant::now() + SHUTDOWN_JOIN_TIMEOUT;
        while !handle.is_finished() && Instant::now() < deadline {
            std::thread::sleep(Duration::from_millis(5));
        }
        if handle.is_finished() {
            let _ = handle.join();
        } else {
            tracing::warn!(
                "cerulion_viz: viz worker did not finish within the shutdown budget (viewer wedged) \
                 — detaching it so teardown never blocks; the last viz frames are dropped"
            );
            // `handle` dropped without join → the stuck thread is detached.
        }
    }
}

/// The worker thread body: own the heavy state, drain the queue, render each
/// batch through the existing dispatch, keep the scene statics + blueprint set
/// up, and reconnect a dead gRPC sink on a health-probe timer.
fn run(
    rec: RecordingStream,
    // Mutable so a `SwapWalker` message can replace it in place
    // between batches (the daemon reseeds it with a remote schema).
    mut walker: FrameWalker,
    mut state: SinkState,
    rx: Receiver<VizMsg>,
    counters: Arc<VizWorkerCounters>,
    hooks: ReconnectHooks,
    probe_interval: Duration,
) {
    // Set the scene up ASAP so the viewer renders correctly even before data
    // (idempotent once-guards; re-armed on reconnect).
    ensure_setup(&rec);
    let mut last_probe = Instant::now();
    // Loud-once reconnect regime latch (first disconnect warn!, repeats debug!,
    // recovery info! when the sink is healthy again).
    let mut reconnect_latch = FieldsWarnLatch::new();
    // Loud-once latch for caught render panics: a panicking frame must
    // NOT silently kill the whole viz worker — it is contained + counted, and
    // the worker keeps going.
    let mut panic_latch = FieldsWarnLatch::new();
    loop {
        let msg = match rx.recv_timeout(probe_interval) {
            Ok(msg) => Some(msg),
            Err(RecvTimeoutError::Timeout) => None,
            Err(RecvTimeoutError::Disconnected) => {
                // Shutdown: the node dropped the sender and the backlog is
                // drained. Best-effort BOUNDED tail flush (the shared stream is
                // a process-local static that is never dropped, so without this
                // the last queued frames can be lost at a clean shutdown).
                let flush = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                    rec.flush_with_timeout(TEARDOWN_FLUSH_TIMEOUT)
                }));
                if let Ok(Err(e)) = flush {
                    tracing::warn!(error = %e, "cerulion_viz: worker teardown flush failed (tail frames may be lost)");
                }
                break;
            }
        };
        // A walker swap is handled HERE (not in `handle_message`)
        // so it can take `walker` by ownership. A plain move — it cannot panic, so
        // it needs no `catch_unwind` — and it carries no batch to render, so
        // `continue` skips the dispatch for this iteration. FIFO on the channel
        // means every batch enqueued after this decodes against the new walker.
        // The `match ... => other` rebind consumes `msg` without partial-moving
        // it, so the non-swap path still owns `msg` for `handle_message`.
        let msg = match msg {
            Some(VizMsg::SwapWalker(new_walker)) => {
                walker = new_walker;
                tracing::debug!("cerulion_viz: viz walker swapped (new schema set installed)");
                continue;
            }
            other => other,
        };
        // CONTAIN any panic in the render / probe / setup path so ONE bad frame
        // (or an SDK-internal panic) never kills the worker — it is caught,
        // counted, logged loud-once, and the loop continues. `AssertUnwindSafe`
        // is sound here: viz is best-effort, so a possibly-inconsistent
        // `SinkState` after a mid-render panic is acceptable (the next frame
        // overwrites it).
        let outcome = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            handle_message(
                &rec,
                &walker,
                &mut state,
                &hooks,
                &counters,
                &mut last_probe,
                &mut reconnect_latch,
                probe_interval,
                msg,
            )
        }));
        if outcome.is_err() {
            counters.render_panics.fetch_add(1, Ordering::Relaxed);
            match panic_latch.on_inferred() {
                FieldsLogAction::WarnFirst => tracing::error!(
                    "cerulion_viz: viz worker CAUGHT a panic in the render/probe path — skipping this \
                     work + continuing (viz is never silently lost; repeats log at debug)"
                ),
                FieldsLogAction::DebugSuppressed { suppressed } => tracing::debug!(
                    suppressed,
                    "cerulion_viz: viz worker render panic sustained (warn suppressed)"
                ),
            }
        }
    }
}

/// Handle one loop iteration's work (a message, or `None` on the idle-probe
/// timeout). Split out so [`run`] can wrap it in `catch_unwind` (a
/// panic here is contained, never fatal to the worker).
#[allow(clippy::too_many_arguments)]
fn handle_message(
    rec: &RecordingStream,
    walker: &FrameWalker,
    state: &mut SinkState,
    hooks: &ReconnectHooks,
    counters: &VizWorkerCounters,
    last_probe: &mut Instant,
    reconnect_latch: &mut FieldsWarnLatch,
    probe_interval: Duration,
    msg: Option<VizMsg>,
) {
    match msg {
        // A pure state edit — no `rec`, no render, no flush.
        Some(VizMsg::SetRepresentation {
            input_key,
            representation,
        }) => {
            tracing::debug!(
                input = %input_key,
                representation = representation.as_wire(),
                "cerulion_viz: topic representation set"
            );
            state.set_representation(&input_key, representation);
        }
        Some(VizMsg::Batch(inputs)) => {
            maybe_probe_reconnect(
                rec,
                state,
                hooks,
                counters,
                last_probe,
                reconnect_latch,
                probe_interval,
            );
            ensure_setup(rec);
            process_batch(rec, walker, state, inputs);
            counters
                .coalesced_frames
                .store(state.coalesced_frames(), Ordering::Relaxed);
            // Publish this batch's LAYOUT SIGNALS — the
            // per-input rendition sets and what each input's render arm did — for
            // the layout plane. Same refresh point and the same reason as
            // `coalesced_frames`: the state lives here, the consumer is elsewhere.
            // ONE seam for both, and it reports whether either CHANGED, which is
            // the whole of what the daemon's drain loop watches (see
            // `VizWorkerCounters::layout_signal_generation` for why watching the
            // proof map alone is blind on video).
            counters
                .publish_layout_signals(state.video().rendition_segments(), state.render_proofs());
        }
        Some(VizMsg::SetBlueprint(plan)) => {
            // Install the client-driven runtime layout ON the worker
            // (the one thread that owns + may block on `rec`). apply remembers it
            // (reconnect re-applies it) + sends it, superseding the active
            // blueprint. A send failure is warned + swallowed inside apply — never
            // a panic (and the worker's `catch_unwind` would contain one anyway).
            apply_runtime_blueprint(rec, plan);
        }
        Some(VizMsg::Barrier(ack)) => {
            // All prior batches are processed (in-order channel) — ack.
            let _ = ack.send(());
        }
        Some(VizMsg::SwapWalker(_)) => {
            // Unreachable by construction: `run` intercepts `SwapWalker` before
            // dispatch (it needs `walker` by ownership) and `continue`s. If a
            // future refactor lets one slip through, the worker's `catch_unwind`
            // contains this panic + counts it (never a silent no-op that would
            // drop the swap).
            unreachable!("SwapWalker is handled in run() before handle_message");
        }
        #[cfg(test)]
        Some(VizMsg::Block { reached, release }) => {
            // Confirm the worker is parked, then block until released — the
            // deterministic "worker not draining" stand-in for a wedged sink.
            let _ = reached.send(());
            let _ = release.recv();
        }
        #[cfg(test)]
        Some(VizMsg::PanicForTest) => panic!("injected render-path panic (test)"),
        None => {
            // Idle-probe timeout: check sink health + keep the scene set up.
            maybe_probe_reconnect(
                rec,
                state,
                hooks,
                counters,
                last_probe,
                reconnect_latch,
                probe_interval,
            );
            ensure_setup(rec);
        }
    }
}

/// Probe the sink for a dead gRPC connection at most once per `probe_interval`;
/// on a genuine disconnect (`Failed`), swap a fresh sink and RE-ARM the scene
/// setup so the bounced (empty) server re-receives the statics + blueprint +
/// skeleton tree + `/tf_static` mounts. Loud-once. A wedged-but-alive viewer
/// (probe `Timeout`) or a healthy one (`Ok`) is left untouched (see
/// [`should_reconnect`]).
///
/// IMPORTANT — the probe's `flush_with_timeout` does NOT bound the SDK's
/// internal step-1 `batcher.flush_blocking(MAX)`, so on a WEDGED-BUT-ALIVE
/// viewer (batcher full behind a stuck-but-connected sink) this probe can BLOCK
/// the worker until the viewer recovers. That is acceptable, by design. The
/// TICK never blocks regardless (it only `try_send`s — a stuck worker just means
/// its queue fills and the tick DROPS, the whole point of the worker). A
/// wedged-but-ALIVE viewer is one we deliberately do NOT reconnect (see
/// [`should_reconnect`]), and the worker self-heals the instant the viewer
/// drains again (the pending flush completes → the loop resumes). A genuinely
/// DEAD/bounced server IS the case we act on, and there the gRPC sink fails its
/// sends fast (the client thread has exited), so the batcher drains, the probe
/// returns `Failed` promptly, and reconnect fires. So the never-block guarantee
/// holds and real bounces are handled; the only cost is that a suspended-tab
/// wedge parks the worker (already not draining) slightly differently. The probe
/// is not truly bounded (`re_sdk`'s connection status could bound it).
/// The reconnect swap likewise only fires on `Failed` (a `Disconnected` sink
/// fails its flush fast in `set_sink`), so it cannot block on a live-but-wedged
/// sink.
#[allow(clippy::too_many_arguments)]
fn maybe_probe_reconnect(
    rec: &RecordingStream,
    state: &mut SinkState,
    hooks: &ReconnectHooks,
    counters: &VizWorkerCounters,
    last_probe: &mut Instant,
    latch: &mut FieldsWarnLatch,
    probe_interval: Duration,
) {
    if last_probe.elapsed() < probe_interval {
        return;
    }
    *last_probe = Instant::now();

    let probe = (hooks.probe)(rec);
    if !should_reconnect(&probe) {
        // Healthy (or slow-but-alive): heal the reconnect regime if one was open.
        if let Some(suppressed) = latch.on_decoded() {
            tracing::info!(
                suppressed_count = suppressed,
                "cerulion_viz: viz gRPC sink healthy again — reconnect regime healed"
            );
        }
        return;
    }

    // Genuine disconnect (bounced server). Loud-once.
    match latch.on_inferred() {
        FieldsLogAction::WarnFirst => tracing::warn!(
            "cerulion_viz: viz gRPC sink disconnected (server bounced?) — reconnecting + re-logging \
             scene statics (repeats log at debug until it reconnects)"
        ),
        FieldsLogAction::DebugSuppressed { suppressed } => tracing::debug!(
            suppressed,
            "cerulion_viz: viz gRPC sink still disconnected (reconnect sustained)"
        ),
    }
    match (hooks.reconnect)(rec) {
        Ok(()) => {
            counters.reconnects.fetch_add(1, Ordering::Relaxed);
            // Re-arm the scene setup so the fresh (empty) server re-receives it:
            // world statics + blueprint + skeleton guards, and the `/tf_static`
            // re-broadcast dedup (else an unchanged mount never re-logs).
            //
            // Video re-arm still matters, but only in the NO-DECODER
            // fallback regime — that path hands access units to the viewer and
            // rerun cannot decode H.264 without the static codec component a
            // bounced server no longer holds. When this desk is decoding, every
            // logged `Image` is self-contained and this costs one breadcrumb.
            crate::stream::rearm_after_reconnect();
            state.clear_rebroadcast_dedup();
            state.rearm_video_after_reconnect();
            // The fresh server holds NO markers, so every believed-live
            // marker key is a claim about a viewer that no longer exists — see
            // `SinkState::reset_marker_state` for the two DELETE-tracking failure
            // modes it causes (draws are unaffected).
            state.reset_marker_state();
        }
        Err(e) => tracing::debug!(
            error = %e,
            "cerulion_viz: viz gRPC reconnect attempt failed (will retry next probe)"
        ),
    }
}

/// Render one poll's batch: the EXACT per-input drain loop the caller
/// used to run inline (staged newest-wins for a replacing kind, render
/// every per-sample frame, fold the coalesced count), now off the tick thread.
fn process_batch(
    rec: &RecordingStream,
    walker: &FrameWalker,
    state: &mut SinkState,
    inputs: Vec<InputFrames>,
) {
    for input in inputs {
        let mut staged: Option<Vec<u8>> = None;
        let mut coalesced: u64 = 0;
        for frame in input.frames {
            dispatch_or_stage(
                rec,
                walker,
                &input.name,
                frame,
                state,
                &mut staged,
                &mut coalesced,
            );
        }
        if let Some(frame) = staged.take() {
            dispatch_frame(rec, walker, &input.name, &frame, state);
        }
        if coalesced > 0 {
            state.record_coalesced(coalesced);
        }
    }
}

/// The scene statics (Z-up world + camera Pinhole) + the default dashboard
/// blueprint, each behind its own once-per-recording guard so this is a cheap
/// no-op after the first fire (and re-fires after a reconnect re-arm).
fn ensure_setup(rec: &RecordingStream) {
    log_viz_statics_once(rec);
    send_blueprint_once(rec);
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A single-input batch of `n` empty (header-less) frames — enough to count
    /// through the enqueue path (the worker never walks them here; these unit
    /// tests exercise the QUEUE + drop accounting, not the dispatch).
    fn batch(name: &str, n: usize) -> Vec<InputFrames> {
        vec![InputFrames {
            name: name.to_string(),
            frames: (0..n).map(|_| vec![0u8; 4]).collect(),
        }]
    }

    /// Poison a mutex the only way one can be poisoned: panic while holding it.
    /// The hook is swapped out so the deliberate panic does not print.
    fn poison<T>(m: &std::sync::Mutex<T>) {
        let prev = std::panic::take_hook();
        std::panic::set_hook(Box::new(|_| {}));
        let _ = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            let _held = m.lock().expect("not yet poisoned");
            panic!("poisoning on purpose");
        }));
        std::panic::set_hook(prev);
        assert!(m.is_poisoned(), "the harness must really poison the mutex");
    }

    /// A poisoned mirror contributes no bump
    /// OF ITS OWN, and does NOT veto its healthy sibling's.
    ///
    /// The suggested alternative — bump only when EVERY mirror published — passes
    /// every other test in the repo, so this is the arm that holds the line. It
    /// would be strictly worse: `Mutex` poison is permanent, so one poisoned map
    /// would freeze the default layout against every future change of the other,
    /// which is the unbounded residual the generation exists to close. And
    /// the "stale value" a bump is alleged to expose is unreachable by construction:
    /// both daemon-side readers degrade a poisoned lock to the SAME default the
    /// signal carries before anything is observed, so the reflow reads the
    /// conservative answer (companion KEPT), never the poisoned map's contents.
    #[test]
    fn a_poisoned_mirror_neither_bumps_nor_vetoes_its_siblings_bump() {
        // (a) A poisoned mirror whose OWN value would have changed: no bump.
        let counters = VizWorkerCounters::default();
        poison(&counters.render_proofs);
        let mut proofs = BTreeMap::new();
        proofs.insert(
            "/a".to_string(),
            RenderProof {
                rendered_without_dumping: true,
                degraded: false,
            },
        );
        assert!(
            !counters.publish_layout_signals(BTreeMap::new(), proofs.clone()),
            "a poisoned mirror must contribute no `changed` of its own"
        );
        assert_eq!(
            counters.layout_signal_generation.load(Ordering::Relaxed),
            0,
            "…and therefore must not bump the generation on its own account"
        );

        // (b) THE ARM: the healthy sibling changed, so the generation MUST bump —
        // the reflow is what carries that change to the viewer.
        let mut renditions = BTreeMap::new();
        renditions.insert("/v".to_string(), vec!["1280x720".to_string()]);
        assert!(
            counters.publish_layout_signals(renditions.clone(), proofs.clone()),
            "a healthy mirror's change must still be reported while its sibling is poisoned"
        );
        assert_eq!(
            counters.layout_signal_generation.load(Ordering::Relaxed),
            1,
            "…and must bump the generation the drain loop watches"
        );

        // (c) The bump is still CHANGE-driven, not poison-driven: republishing the
        // same pair bumps nothing (otherwise (b) would pass on a per-batch bump).
        assert!(!counters.publish_layout_signals(renditions, proofs));
        assert_eq!(counters.layout_signal_generation.load(Ordering::Relaxed), 1);
    }

    #[test]
    fn empty_batch_is_a_noop_no_drop_churn() {
        // This test SPAWNS a worker, whose boot runs `ensure_setup`
        // -> a BLUEPRINT_SENT swap on the PRODUCTION path. Guard taken FIRST
        // so it drops LAST, outliving `VizLogWorker::drop`'s thread join.
        let _statics = crate::test_support::blueprint_statics_guard();
        // A batch whose every input drained nothing must not enqueue, drop, or
        // touch the latch (the wired-but-quiet input path every tick).
        let (rec, _storage) = rerun::RecordingStreamBuilder::new("go2_test")
            .recording_id("viz_worker_empty")
            .memory()
            .expect("memory sink");
        let (walker, _warn) = FrameWalker::new(Vec::new());
        let mut worker = VizLogWorker::spawn(rec, walker, SinkState::new()).expect("spawn worker");
        worker.try_enqueue(vec![InputFrames {
            name: "cloud".to_string(),
            frames: Vec::new(),
        }]);
        worker.sync();
        assert_eq!(worker.dropped_frames(), 0, "an empty batch drops nothing");
        assert_eq!(worker.dropped_batches(), 0);
    }

    #[test]
    fn healthy_worker_that_keeps_up_never_drops() {
        // This test SPAWNS a worker, whose boot runs `ensure_setup`
        // -> a BLUEPRINT_SENT swap on the PRODUCTION path. Guard taken FIRST
        // so it drops LAST, outliving `VizLogWorker::drop`'s thread join.
        let _statics = crate::test_support::blueprint_statics_guard();
        // A worker that drains each batch (memory sink, synced between sends)
        // never fills the queue → never drops. Anti-tautology control for the
        // parked-worker drop test below.
        let (rec, _storage) = rerun::RecordingStreamBuilder::new("go2_test")
            .recording_id("viz_worker_healthy")
            .memory()
            .expect("memory sink");
        let (walker, _warn) = FrameWalker::new(Vec::new());
        let mut worker = VizLogWorker::spawn(rec, walker, SinkState::new()).expect("spawn worker");
        for _ in 0..20 {
            worker.try_enqueue(batch("cloud", 3));
            worker.sync(); // drain each before the next → never full → never drops
        }
        assert_eq!(
            worker.dropped_frames(),
            0,
            "a worker that keeps up drops nothing"
        );
        assert_eq!(worker.dropped_batches(), 0);
    }

    #[test]
    fn parked_worker_drops_overflow_and_enqueue_stays_fast_hand_oracle() {
        // This test SPAWNS a worker, whose boot runs `ensure_setup`
        // -> a BLUEPRINT_SENT swap on the PRODUCTION path. Guard taken FIRST
        // so it drops LAST, outliving `VizLogWorker::drop`'s thread join.
        let _statics = crate::test_support::blueprint_statics_guard();
        // THE never-block pin (deterministic form): park the worker so it is
        // provably NOT draining (the exact "stuck in rec.log" failure mode),
        // then flood the bounded queue. The first VIZ_QUEUE_CAP batches fit; the
        // next K overflow → DROP. Hand oracle: dropped_batches == K,
        // dropped_frames == K*F. And every enqueue stays FAST (non-blocking by
        // construction — try_send never waits on the parked worker).
        use std::time::Instant;
        const F: usize = 5; // frames per batch
        const K: usize = 6; // overflow batches
        let (rec, _storage) = rerun::RecordingStreamBuilder::new("go2_test")
            .recording_id("viz_worker_parked")
            .memory()
            .expect("memory sink");
        let (walker, _warn) = FrameWalker::new(Vec::new());
        let mut worker = VizLogWorker::spawn(rec, walker, SinkState::new()).expect("spawn worker");

        let guard = worker.park_worker_for_test(); // worker now parked
        let mut max_enqueue = Duration::ZERO;
        // Fill the queue exactly to capacity (these are BUFFERED, not dropped).
        for _ in 0..VIZ_QUEUE_CAP {
            let t0 = Instant::now();
            worker.try_enqueue(batch("cloud", F));
            max_enqueue = max_enqueue.max(t0.elapsed());
        }
        assert_eq!(worker.dropped_frames(), 0, "the first CAP batches fit");
        // K more → each overflows and drops.
        for _ in 0..K {
            let t0 = Instant::now();
            worker.try_enqueue(batch("cloud", F));
            max_enqueue = max_enqueue.max(t0.elapsed());
        }
        assert_eq!(
            worker.dropped_batches() as usize,
            K,
            "exactly K overflow batches dropped while the worker was parked"
        );
        assert_eq!(
            worker.dropped_frames() as usize,
            K * F,
            "dropped_frames sums FRAMES across the K dropped batches (K*F)"
        );
        // The enqueue never blocks on the parked worker: even the worst enqueue
        // is far under any tick budget (generous 50 ms ceiling — try_send is
        // O(1), this is really microseconds).
        assert!(
            max_enqueue < Duration::from_millis(50),
            "enqueue must stay non-blocking even with the worker wedged (worst {max_enqueue:?})"
        );
        drop(guard); // release the worker → it drains the buffered CAP batches
        worker.sync();
    }

    #[test]
    fn try_swap_walker_never_blocks_on_a_full_queue_and_lands_after_it_drains() {
        // This test SPAWNS a worker, whose boot runs `ensure_setup`
        // -> a BLUEPRINT_SENT swap on the PRODUCTION path. Guard taken FIRST
        // so it drops LAST, outliving `VizLogWorker::drop`'s thread join.
        let _statics = crate::test_support::blueprint_statics_guard();
        // The poll thread's walker hand-off must NEVER block on the worker.
        // Park the worker (provably NOT draining — the wedged-viewer
        // stand-in), FILL the bounded queue to
        // capacity, then swap a walker: `try_swap_walker` must return it BACK
        // (retain) IMMEDIATELY. A regression to the blocking `swap_walker` would
        // HANG here forever (the parked worker never makes room), so a tight time
        // bound is the kill. Then release + drain → the retained swap lands.
        use std::time::Instant;
        let (rec, _storage) = rerun::RecordingStreamBuilder::new("go2_test")
            .recording_id("viz_worker_try_swap")
            .memory()
            .expect("memory sink");
        let (walker, _warn) = FrameWalker::new(Vec::new());
        let mut worker = VizLogWorker::spawn(rec, walker, SinkState::new()).expect("spawn worker");

        // Idle-worker control: an accepted swap returns None (sent, not retained).
        let (w0, _) = FrameWalker::new(Vec::new());
        assert!(
            worker.try_swap_walker(w0).is_none(),
            "an idle worker ACCEPTS the swap (None = sent)"
        );
        worker.sync();

        // Park + fill the queue to capacity → the next swap can't be sent.
        let guard = worker.park_worker_for_test();
        for _ in 0..VIZ_QUEUE_CAP {
            worker.try_enqueue(batch("cloud", 1));
        }
        let (w1, _) = FrameWalker::new(Vec::new());
        let t0 = Instant::now();
        let returned = worker.try_swap_walker(w1);
        let elapsed = t0.elapsed();
        assert!(
            returned.is_some(),
            "a FULL queue returns the walker BACK to retain (never dropped, never blocked)"
        );
        assert!(
            elapsed < Duration::from_millis(50),
            "the swap hand-off is NON-BLOCKING even against a wedged worker (took {elapsed:?}); \
             a blocking send would hang forever here"
        );

        // Release → the worker drains the buffered batches → a retry of the
        // retained swap now LANDS (None = accepted).
        drop(guard);
        worker.sync();
        let landed = worker.try_swap_walker(returned.expect("walker retained"));
        assert!(
            landed.is_none(),
            "once the queue drains, the retained swap is accepted (it is never lost)"
        );
        worker.sync();
    }

    #[test]
    fn worker_contains_a_render_panic_and_keeps_going() {
        // This test SPAWNS a worker, whose boot runs `ensure_setup`
        // -> a BLUEPRINT_SENT swap on the PRODUCTION path. Guard taken FIRST
        // so it drops LAST, outliving `VizLogWorker::drop`'s thread join.
        let _statics = crate::test_support::blueprint_statics_guard();
        // A panic in the worker's render path must not kill the worker.
        // Inject a panic, then prove the worker is still alive (a later batch is
        // still processed via a sync barrier) and that the panic was COUNTED,
        // not silently swallowed. Hand oracle: render_panics == 1.
        let (rec, _storage) = rerun::RecordingStreamBuilder::new("go2_test")
            .recording_id("viz_worker_panic")
            .memory()
            .expect("memory sink");
        let (walker, _warn) = FrameWalker::new(Vec::new());
        let mut worker = VizLogWorker::spawn(rec, walker, SinkState::new()).expect("spawn worker");
        // The default panic hook prints the injected panic to stderr — expected.
        worker.inject_panic_for_test();
        // The worker survived the panic: this barrier is acked (would hang/return
        // Disconnected if the thread had died — the barrier recv tolerates both,
        // so also assert a healthy enqueue still flows).
        worker.sync();
        assert_eq!(
            worker.render_panics(),
            1,
            "the render panic was contained + counted exactly once"
        );
        // Still fully functional after the contained panic.
        worker.try_enqueue(batch("cloud", 2));
        worker.sync();
        assert_eq!(
            worker.dropped_frames(),
            0,
            "a healthy enqueue after the contained panic still flows (worker alive)"
        );
    }

    #[test]
    fn should_reconnect_only_on_failed_not_timeout_or_ok() {
        // Hand oracle for the reconnect discriminator: reconnect ONLY on a
        // genuine connection Failure. A slow-but-alive / still-connecting sink
        // (Timeout) and a healthy sink (Ok) must NOT reconnect — else a merely
        // wedged-but-alive viewer would be needlessly reconnected.
        assert!(should_reconnect(&Err(SinkFlushError::failed(
            "connection severed"
        ))));
        assert!(!should_reconnect(&Err(SinkFlushError::Timeout)));
        assert!(!should_reconnect(&Ok(())));
    }

    #[test]
    fn drop_regime_heals_when_worker_drains_again() {
        // This test SPAWNS a worker, whose boot runs `ensure_setup`
        // -> a BLUEPRINT_SENT swap on the PRODUCTION path. Guard taken FIRST
        // so it drops LAST, outliving `VizLogWorker::drop`'s thread join.
        let _statics = crate::test_support::blueprint_statics_guard();
        // After a drop regime, a successful enqueue (worker draining again)
        // heals the latch. Pin: dropped_frames stops growing once unparked +
        // synced, and further healthy enqueues add no drops.
        const F: usize = 4;
        let (rec, _storage) = rerun::RecordingStreamBuilder::new("go2_test")
            .recording_id("viz_worker_heal")
            .memory()
            .expect("memory sink");
        let (walker, _warn) = FrameWalker::new(Vec::new());
        let mut worker = VizLogWorker::spawn(rec, walker, SinkState::new()).expect("spawn worker");
        let guard = worker.park_worker_for_test();
        for _ in 0..(VIZ_QUEUE_CAP + 4) {
            worker.try_enqueue(batch("cloud", F));
        }
        let dropped_during_wedge = worker.dropped_frames();
        assert_eq!(dropped_during_wedge as usize, 4 * F, "4 batches overflowed");
        drop(guard);
        worker.sync(); // worker drained the backlog
                       // Healthy again: enqueues no longer drop.
        for _ in 0..5 {
            worker.try_enqueue(batch("cloud", F));
            worker.sync();
        }
        assert_eq!(
            worker.dropped_frames(),
            dropped_during_wedge,
            "no further drops once the worker drains again (regime healed)"
        );
    }

    // ── runtime-layout control handle ───────────────────────────────────────
    // NOTE: these worker-level tests deliberately do NOT drive a SUCCESSFUL
    // set_blueprint enqueue — that path runs `apply_runtime_blueprint`, which
    // writes the process-global RUNTIME_BLUEPRINT static (owned by the
    // `apply_runtime_blueprint_remembers...` test in `blueprint.rs`, the sole
    // toucher in this binary). The happy apply path is proven end-to-end by the
    // `cerulion_vizd` daemon e2e (a SEPARATE process). Here we pin only the
    // control-handle FAILURE paths (Busy + WorkerGone), which never enqueue and
    // so never touch the static.

    #[test]
    fn set_blueprint_reports_worker_gone_after_control_close() {
        // This test SPAWNS a worker, whose boot runs `ensure_setup`
        // -> a BLUEPRINT_SENT swap on the PRODUCTION path. Guard taken FIRST
        // so it drops LAST, outliving `VizLogWorker::drop`'s thread join.
        let _statics = crate::test_support::blueprint_statics_guard();
        // `close()` releases the control handle's sender → a later set_blueprint
        // reports WorkerGone (never a panic, never a silent drop). This is the
        // shutdown seam: the daemon closes the control so the worker can reach
        // Disconnected even while controller threads still hold the handle.
        let (rec, _storage) = rerun::RecordingStreamBuilder::new("go2_test")
            .recording_id("viz_ctl_close")
            .memory()
            .expect("memory sink");
        let (walker, _warn) = FrameWalker::new(Vec::new());
        let worker = VizLogWorker::spawn(rec, walker, SinkState::new()).expect("spawn worker");
        let control = worker.control();
        control.close();
        let err = control
            .set_blueprint(BlueprintPlan::go2_default())
            .expect_err("a closed control reports the worker gone");
        assert!(matches!(err, VizControlError::WorkerGone), "got {err:?}");
        // Idempotent: a second close is a no-op, still WorkerGone.
        control.close();
        assert!(control.set_blueprint(BlueprintPlan::go2_default()).is_err());
    }

    #[test]
    fn set_blueprint_is_bounded_busy_when_worker_is_wedged() {
        // This test SPAWNS a worker, whose boot runs `ensure_setup`
        // -> a BLUEPRINT_SENT swap on the PRODUCTION path. Guard taken FIRST
        // so it drops LAST, outliving `VizLogWorker::drop`'s thread join.
        let _statics = crate::test_support::blueprint_statics_guard();
        // THE never-block pin for the control path: a WEDGED worker (parked, not
        // draining) with a FULL queue makes set_blueprint return Busy within the
        // bounded CONTROL_SEND_TIMEOUT — never a hang, never a silent drop (and
        // the message never enqueues, so no static write).
        const F: usize = 3;
        let (rec, _storage) = rerun::RecordingStreamBuilder::new("go2_test")
            .recording_id("viz_ctl_busy")
            .memory()
            .expect("memory sink");
        let (walker, _warn) = FrameWalker::new(Vec::new());
        let mut worker = VizLogWorker::spawn(rec, walker, SinkState::new()).expect("spawn worker");
        let control = worker.control();

        let guard = worker.park_worker_for_test(); // worker parked, not draining
        for _ in 0..VIZ_QUEUE_CAP {
            worker.try_enqueue(batch("cloud", F)); // fill the queue to capacity
        }
        // The queue is full + the worker is parked → set_blueprint bounces off
        // try_send until the deadline, then reports Busy (bounded).
        let t0 = Instant::now();
        let err = control
            .set_blueprint(BlueprintPlan::go2_default())
            .expect_err("a wedged worker makes set_blueprint Busy");
        let elapsed = t0.elapsed();
        assert!(matches!(err, VizControlError::Busy), "got {err:?}");
        assert!(
            elapsed >= CONTROL_SEND_TIMEOUT
                && elapsed < CONTROL_SEND_TIMEOUT + Duration::from_secs(2),
            "Busy is bounded by CONTROL_SEND_TIMEOUT (took {elapsed:?})"
        );
        drop(guard); // release the worker → drains the buffered batches
        worker.sync();
    }
}
