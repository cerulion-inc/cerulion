// SPDX-License-Identifier: AGPL-3.0-only
//! The DDS pump — the helper-thread state machine behind the node's
//! [`ExternalSource::Blocking`] closure (the camera_jpeg `CaptureLoop`
//! shape: the closure only injects real time, sleeps the returned pace, and
//! forwards the ring; ALL behavior lives here).
//!
//! # Drain path: `async_stream()`, NOT `take()` (empirically decisive)
//!
//! Samples are drained via ros2-client's `Subscription::async_stream()` +
//! `StreamExt::next()` — the pattern ros2-client's own examples
//! use. Synchronous `Subscription::take()` polling
//! EMPIRICALLY NEVER YIELDS under ros2-client 0.10 against a
//! CycloneDDS peer: a take()-polling e2e saw 0 samples in 30 s on BOTH QoS arms while
//! an `async_stream` subscriber against the SAME peer, machine, and
//! `--only-network` received 58 clouds byte-exact — isolating the defect to
//! the take() drain (whatever the internal mechanism: reader-cache readiness
//! signaling or ros2-client 0.10 internals). Do not "simplify" back to
//! take()-polling without re-running that control experiment.
//!
//! # ALL DDS objects are created ON the drain thread
//!
//! The participant, node, topics, subscriptions, and spinner are built on the
//! SAME dedicated thread that drives the streams ([`drain_thread_main`]).
//! This matches the probe shapes that deliver against a live peer (the
//! single-thread shape and the create-then-move shape both delivered
//! 3/3), while create-on-the-pump-thread is the structural
//! delta of the shape that failed — creating on the drain thread removes
//! that delta entirely and simplifies lifetimes (no DDS object ever crosses a
//! thread boundary). Init results surface back to the pump through
//! [`StartupStatus`]; an init failure exits the thread (dropping the
//! participant guard → the one-per-process slot frees) and the pump retries
//! on the 1 s backoff — loud, never silent-dead (the camera failure
//! philosophy). The WHOLE thread body additionally runs under `catch_unwind`:
//! a PANIC anywhere — the rustdds init phase included — marks the
//! terminal `PHASE_DEAD` + one loud `error!` carrying the payload, never a
//! silent thread death that leaves the status PENDING (or UP) forever.
//!
//! # Observability (Principle #3)
//!
//! [`PumpStats`] carries cheap atomic counters over every hop of the
//! DDS→queue path — `init_attempts/failures`, `streams_started/ended`,
//! `samples_pushed_total`, `drain_stream_errors_total` — shared via
//! [`BridgePump::stats`] (the live-DDS e2e renders them in its failure message so
//! the dead hop is visible on sight). The queue→tick hop has its own counter
//! on [`SampleQueue`] (`lifetime_drained_total`, bumped by the tick's
//! `drain_all`).
//!
//! # Lifecycle
//!
//! - LAZY spawn on the first iterate; spawn/init failures are loud and
//!   backoff-retried.
//! - The drain thread runs a `smol::LocalExecutor` hosting the node SPINNER
//!   (discovery/matching housekeeping) plus one long-lived pinned
//!   `async_stream()` drain task per subscription.
//!   Each received sample lands in the shared latest-wins
//!   [`SampleQueue`] and flags the shared `pushed` bit.
//! - GRACEFUL SHUTDOWN (no ghost readers): the executor is
//!   driven by a STOP future (a `oneshot` the node signals), NOT
//!   `future::pending()`. A shutdown request resolves it → the executor
//!   returns → the drain thread's locals drop. Dropping the participant guard
//!   runs rustdds's participant `Drop`, which SENDS the SPDP participant-dispose
//!   plus the per-endpoint SEDP disposes (`DomainParticipantDisc::drop` →
//!   `on_participant_shutting_down`), so remote writers unregister this node's readers
//!   IMMEDIATELY instead of holding them as ghosts until the DDS lease ages out.
//!   The thread then signals `finished` and exits; [`BridgePump::shutdown`]
//!   joins it under a bounded deadline ([`DRAIN_JOIN_DEADLINE`]) and, on
//!   expiry, warns loudly + DETACHES (never blocks teardown forever). The node
//!   reaches this seam from `NodeEntry::shutdown()` through a shared
//!   [`PumpShutdown`] handle, because the pump itself is MOVED into the
//!   `ExternalSource::Blocking` closure (owned by the host's own detached
//!   helper thread) — the shared handle is the ONLY path the node retains to
//!   signal + join the drain thread. It also latches `stopping`, so a helper
//!   iterate that races AFTER shutdown is a graceful no-op (never respawns an
//!   orphaned participant). A thread detached for the whole process would
//!   orphan the participant on EVERY teardown (SIGINT included), leaving a
//!   ghost reader on the robot.
//! - Each helper iterate is BOUNDED (the doorbell/stop contract): it swaps
//!   the `pushed` flag (ring iff samples arrived since the last iterate) and
//!   paces [`POLL_PACE`]. Helper sleeps never touch the drain thread — the
//!   streams run continuously regardless of the ring cadence.
//!
//! # Typed subscriptions ARE the `cerulion_go2_dds` codec path
//!
//! Subscriptions are `Subscription<cerulion_go2_dds::messages::T>` — the
//! field-verified `cerulion_go2_dds` structs, deserialized by the SAME serde-CDR engine
//! the pure `cerulion_go2_dds::cdr` codecs wrap (wire-identical; the pure fns
//! remain the byte-level oracle seam — `registry::decode_sample`).
//!
//! # Raw-generic mappings ride the same thread
//!
//! Config mappings whose `ros_type` is OUTSIDE the four-type registry but
//! schema-resolvable get a RAW-CDR reader (the rustdds drop-down —
//! `generic::raw` module docs carry the reasoning) created ON the drain
//! thread like every other DDS object, drained via the same
//! `as_async_stream()` executor shape ([`drain_raw_from_stream`] mirrors
//! [`drain_from_stream`]'s err-streak/stream-END discipline), and published
//! through [`RawIngressRoute::publish_cdr_body`] — the generic codec →
//! `publish_raw`, OFF the node tick (no port, no queue, no doorbell). Wire
//! timestamps come from the TRANSPORT clock at drain-publish time —
//! consistent with the typed ports' loan-time stamps and deliberately NOT
//! the DDS writer's `source_timestamp` (a remote wall clock; lib.rs docs).
//! The transport is CONTEXT-CARRIED: the graph runtime injects
//! the host's manager into the `NodeContext`, `DdsBridge::init` stores it,
//! and `external_source` hands it to [`BridgePump::with_transport`]. There
//! is NO `TransportManager::get()` fallback — the bridge runs as a CDYLIB
//! with its OWN copy of cerulion_core's statics (see
//! [`resolve_raw_transport`]); a pump built without a transport fails raw
//! init LOUDLY. Raw hop counters (`raw_*` on [`PumpStats`]) localize a dead
//! raw hop exactly like the typed ones.

use std::panic::AssertUnwindSafe;
use std::pin::Pin;
use std::sync::atomic::{AtomicBool, AtomicU64, AtomicU8, Ordering};
use std::sync::mpsc::{Receiver as MpscReceiver, RecvTimeoutError, Sender as MpscSender};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use cerulion_core::clock::Clock;
use cerulion_core::codegen::CdrCodec;
use cerulion_core::transport::TransportManager;
use cerulion_go2_dds::messages;
use cerulion_go2_dds::participant::ParticipantConfig;
use cerulion_go2_dds::ros2_client::rustdds::Subscriber as DdsSubscriber;
use cerulion_go2_dds::ros2_client::{MessageTypeName, Name, Node, Subscription};
use cerulion_go2_dds::{best_effort_qos, reliable_volatile_qos, Go2Participant};
use futures::{Stream, StreamExt};

use crate::config::{BridgeConfig, QosMode, TopicMapping};
use crate::generic::raw::{create_raw_reader, RawReader, RawSample};
use crate::generic::RawIngressRoute;
use crate::latch::{FloodAction, FloodLatch};
use crate::queue::SampleQueue;
use crate::registry::{BridgeSample, RosType};

/// Helper-iterate pace once live: the doorbell-ring cadence (bounded —
/// shutdown responsiveness; the drain itself runs continuously on the drain
/// thread, unaffected by this sleep).
pub const POLL_PACE: Duration = Duration::from_millis(5);
/// Backoff after a failed DDS init before the next attempt.
pub const INIT_BACKOFF: Duration = Duration::from_secs(1);
/// Pace slice while waiting out startup / the init backoff (bounded, never a
/// 1 s sleep — the helper must observe shutdown promptly).
pub const BACKOFF_SLICE: Duration = Duration::from_millis(50);
/// The bounded deadline [`BridgePump::shutdown`] waits for the drain
/// thread to finish (send its SPDP/SEDP disposes, drop the participant, exit)
/// before DETACHING it with a loud warn. Generous enough for rustdds's
/// teardown (which itself joins its discovery + event-loop threads) yet short
/// enough that node teardown never hangs a robot's stop sequence.
pub const DRAIN_JOIN_DEADLINE: Duration = Duration::from_secs(5);

/// One iterate's outcome (the camera `CaptureLoop` contract): ring the
/// doorbell iff samples were pushed since the previous iterate; sleep `pace`
/// before the next iterate.
#[derive(Debug)]
pub struct PumpOutcome {
    pub ring: bool,
    pub pace: Option<Duration>,
}

// ---------------------------------------------------------------------------
// Observability (Principle #3)
// ---------------------------------------------------------------------------

/// Cheap atomic counters over every hop of the DDS→queue path. Never reset;
/// shared between the pump, the drain thread, and diagnostics readers
/// ([`BridgePump::stats`]).
#[derive(Debug, Default)]
pub struct PumpStats {
    /// Drain-thread spawn attempts (== DDS init attempts).
    pub init_attempts_total: AtomicU64,
    /// Failed inits/spawns (thread exited at startup; backoff-retried).
    pub init_failures_total: AtomicU64,
    /// Drain stream tasks entered (== subscriptions whose stream started).
    pub streams_started_total: AtomicU64,
    /// Drain streams that ENDED (should stay 0 — an ended stream is a dead
    /// mapping).
    pub streams_ended_total: AtomicU64,
    /// Samples received off DDS and pushed into the queue (the `Ok` arm of
    /// `drain_stream`).
    pub samples_pushed_total: AtomicU64,
    /// Stream item errors (the `Err` arm of `drain_stream`).
    pub drain_stream_errors_total: AtomicU64,
    /// RAW drain tasks entered (== raw mappings whose stream started).
    pub raw_streams_started_total: AtomicU64,
    /// RAW drain streams that ENDED (should stay 0 — a dead raw mapping).
    pub raw_streams_ended_total: AtomicU64,
    /// Frames transcoded + published by raw routes (the `Ok` arm of
    /// `publish_cdr_body` inside [`drain_raw_from_stream`]).
    pub raw_published_total: AtomicU64,
    /// Raw-route transcode/publish failures (samples dropped, sequence not
    /// burned — the route's own counters break these down per route).
    pub raw_route_failures_total: AtomicU64,
    /// Raw stream item errors (the `Err` arm of a raw drain stream).
    pub raw_stream_errors_total: AtomicU64,
}

impl PumpStats {
    /// One-line render for failure messages / diagnostics (pair it with the
    /// queue's `lifetime_drained_total` for the queue→tick hop). The raw_*
    /// terms localize a dead RAW hop exactly like the typed ones (Principle
    /// #3): `raw_published == 0` with `raw_streams_started > 0` and zero
    /// failures → DDS side quiet; `raw_route_failures > 0` → frames arrive
    /// but do not transcode/publish.
    pub fn render(&self) -> String {
        format!(
            "init_attempts={} init_failures={} streams_started={} streams_ended={} \
             samples_pushed={} stream_errors={} raw_streams_started={} \
             raw_streams_ended={} raw_published={} raw_route_failures={} \
             raw_stream_errors={}",
            self.init_attempts_total.load(Ordering::Relaxed),
            self.init_failures_total.load(Ordering::Relaxed),
            self.streams_started_total.load(Ordering::Relaxed),
            self.streams_ended_total.load(Ordering::Relaxed),
            self.samples_pushed_total.load(Ordering::Relaxed),
            self.drain_stream_errors_total.load(Ordering::Relaxed),
            self.raw_streams_started_total.load(Ordering::Relaxed),
            self.raw_streams_ended_total.load(Ordering::Relaxed),
            self.raw_published_total.load(Ordering::Relaxed),
            self.raw_route_failures_total.load(Ordering::Relaxed),
            self.raw_stream_errors_total.load(Ordering::Relaxed),
        )
    }
}

// ---------------------------------------------------------------------------
// Drain-thread startup reporting
// ---------------------------------------------------------------------------

const PHASE_PENDING: u8 = 0;
const PHASE_UP: u8 = 1;
const PHASE_FAILED: u8 = 2;
/// The drain thread panicked — in the executor (a drain task) or in the
/// DDS init/setup phase (the whole thread body is contained).
/// Terminal — the pump does NOT auto-restart (dead-until-relaunch); it
/// surfaces DEAD loudly on every iterate instead of reporting healthy (or
/// "init still running") forever.
const PHASE_DEAD: u8 = 3;

/// How the drain thread reports its startup outcome back to the pump (DDS
/// init happens ON the drain thread — module docs).
#[derive(Debug)]
struct StartupStatus {
    phase: AtomicU8,
    error: Mutex<Option<String>>,
}

impl StartupStatus {
    fn new() -> Self {
        Self {
            phase: AtomicU8::new(PHASE_PENDING),
            error: Mutex::new(None),
        }
    }
    fn set_up(&self) {
        self.phase.store(PHASE_UP, Ordering::Release);
    }
    fn set_failed(&self, e: String) {
        *self
            .error
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner()) = Some(e);
        self.phase.store(PHASE_FAILED, Ordering::Release);
    }
    /// Mark the drain thread DEAD after a contained executor panic,
    /// stashing the panic payload for the operator.
    fn set_dead(&self, e: String) {
        *self
            .error
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner()) = Some(e);
        self.phase.store(PHASE_DEAD, Ordering::Release);
    }
    fn take_error(&self) -> String {
        self.error
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner())
            .take()
            .unwrap_or_else(|| "unknown drain-thread init failure".to_string())
    }
}

/// The init/re-init gate. `Wait` = a prior failure's backoff deadline is
/// still in the future; `Attempt` = no prior failure or the deadline passed.
/// Extracted (pure) so the backoff contract is testable without spawning a
/// drain thread.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum InitGate {
    Wait,
    Attempt,
}

fn init_gate(next_init_at: Option<Instant>, now: Instant) -> InitGate {
    match next_init_at {
        Some(at) if now < at => InitGate::Wait,
        _ => InitGate::Attempt,
    }
}

/// The under-lock spawn/abort decision at the drain-thread spawn site.
/// Re-reads `stopping` INSIDE `current`'s critical section (after the lock-free
/// top-of-[`BridgePump::iterate`] check) so a `shutdown()` that latched
/// `stopping` + took the handle under this SAME lock cannot have a fresh drain
/// thread slipped past it. Returns `true` iff the spawn may proceed (i.e. NOT
/// stopping). Extracted (pure) so the race guard is deterministically pinned
/// without staging a concurrent shutdown — single-threaded the top-of-`iterate`
/// check short-circuits before this site is ever reached, so the re-check is
/// otherwise untestable (deleting it would pass the suite).
fn should_spawn_under_lock(stopping: bool) -> bool {
    !stopping
}

/// The log LEVEL for a drain-stream error at 1-based consecutive
/// `streak`. Extracted (pure) so the warn→debug latching is testable without a
/// live peer: the first error of a streak warns, repeats log at debug.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum StreamErrLevel {
    Warn,
    Debug,
}

fn stream_err_level(streak: u64) -> StreamErrLevel {
    if streak == 1 {
        StreamErrLevel::Warn
    } else {
        StreamErrLevel::Debug
    }
}

/// The pump's handle onto a starting/running drain thread.
struct DrainHandle {
    pushed: Arc<AtomicBool>,
    status: Arc<StartupStatus>,
}

/// The stop + join handle onto ONE spawned drain thread — created alongside it
/// in [`spawn_drain_thread`], registered into [`PumpShutdown::current`], and
/// consumed by [`DrainThreadStop::request_and_join`] at shutdown.
///
/// Kept in the SHARED [`PumpShutdown`] (not the pump-local [`DrainHandle`]) so
/// the node's `NodeEntry::shutdown()` can reach it even though the pump is
/// owned by the host's detached `ExternalSource::Blocking` helper thread.
struct DrainThreadStop {
    /// Resolving the drain thread's stop future — dropping (or sending on) this
    /// makes `stop_rx.await` complete, so the executor returns and the drain
    /// thread's locals (the participant guard) drop → the dispose is sent.
    stop_tx: futures::channel::oneshot::Sender<()>,
    /// The drain thread signals `()` (or disconnects on drop) at its very end —
    /// AFTER the participant dropped + disposed — so the bounded join can tell
    /// "finished, reap now" from "still tearing down" without blocking forever.
    finished_rx: MpscReceiver<()>,
    /// Reaps the exited OS thread (near-instant once `finished_rx` has fired).
    join: JoinHandle<()>,
}

/// The outcome of a bounded drain-thread join (diagnostics + tests).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ShutdownOutcome {
    /// No drain thread was ever registered (config error / never iterated).
    NothingToStop,
    /// The drain thread finished (disposes sent, participant dropped) and was
    /// reaped within [`DRAIN_JOIN_DEADLINE`].
    Joined,
    /// The deadline elapsed; the thread was DETACHED (ghost readers may persist
    /// until the DDS lease ages them out). Logged loudly at the call site.
    TimedOut,
    /// A prior [`PumpShutdown::shutdown`] already ran (idempotent no-op).
    AlreadyShutDown,
}

impl DrainThreadStop {
    /// Signal stop, then wait (bounded by `deadline`) for the drain thread to
    /// finish and reap it. The signal (dropping `stop_tx`) resolves the drain
    /// future → executor returns → participant drops → SPDP/SEDP disposes go
    /// out → the thread signals `finished`. On deadline expiry the thread is
    /// DETACHED (its `JoinHandle` dropped) rather than blocking teardown; the
    /// caller warns loudly.
    fn request_and_join(self, deadline: Duration) -> ShutdownOutcome {
        let DrainThreadStop {
            stop_tx,
            finished_rx,
            join,
        } = self;
        // Signal stop. Dropping the sender cancels `stop_rx`, which the drain
        // future awaits as its completion (the `let _ = stop_rx.await` arm).
        drop(stop_tx);
        match finished_rx.recv_timeout(deadline) {
            // Finished (explicit signal) or the drain thread already dropped its
            // sender on exit — either way it is done; reap it (near-instant).
            Ok(()) | Err(RecvTimeoutError::Disconnected) => {
                let _ = join.join();
                ShutdownOutcome::Joined
            }
            // Wedged in teardown — detach (drop `join`) so teardown proceeds.
            Err(RecvTimeoutError::Timeout) => ShutdownOutcome::TimedOut,
        }
    }
}

/// The SHARED shutdown coordinator between the node (`DdsBridge::shutdown`) and
/// the pump (moved into the `ExternalSource::Blocking` closure). Held via `Arc`:
/// the pump keeps one clone (to register each spawned drain thread's stop
/// handle + to gate `iterate` post-shutdown), the node keeps one clone (to
/// signal + join at teardown).
///
/// Ownership story: `external_source` builds the pump, clones this handle onto
/// the node BEFORE moving the pump into the closure, so the node retains the
/// ONLY reachable path to the drain thread once the pump is closure-captured.
pub struct PumpShutdown {
    /// Latched true by [`Self::shutdown`]. Read at the top of every
    /// [`BridgePump::iterate`] (lock-free) so a helper iterate racing AFTER
    /// shutdown never (re)spawns an orphaned participant, and re-checked under
    /// `current`'s lock at the spawn site so a shutdown concurrent with a spawn
    /// cannot slip a fresh thread past the take.
    stopping: AtomicBool,
    /// The CURRENT drain thread's stop handle — replaced on each (re)spawn,
    /// `None` before the first spawn / while a FAILED thread's slot is empty.
    /// Guarded because the helper thread stores here (on spawn) while the
    /// shutdown thread takes here (on stop).
    current: Mutex<Option<DrainThreadStop>>,
    /// Idempotency: [`Self::shutdown`] has already run to completion.
    shutdown_done: AtomicBool,
}

impl PumpShutdown {
    fn new() -> Self {
        Self {
            stopping: AtomicBool::new(false),
            current: Mutex::new(None),
            shutdown_done: AtomicBool::new(false),
        }
    }

    /// True once [`Self::shutdown`] has latched stop. The pump reads this at the
    /// top of `iterate` to no-op gracefully post-shutdown.
    fn is_stopping(&self) -> bool {
        self.stopping.load(Ordering::Acquire)
    }

    /// Signal the current drain thread to stop and join it within `deadline`
    /// (default [`DRAIN_JOIN_DEADLINE`]). Synchronous + BOUNDED — reachable from
    /// `NodeEntry::shutdown()` on host teardown. Idempotent: a second call is an
    /// `AlreadyShutDown` no-op (the stop handle was consumed by the first).
    ///
    /// Sets `stopping` UNDER `current`'s lock and takes the handle in the same
    /// critical section, so it composes race-free with the pump's
    /// under-lock-guarded spawn (see [`BridgePump::iterate`]).
    pub fn shutdown(&self, deadline: Duration) -> ShutdownOutcome {
        if self.shutdown_done.swap(true, Ordering::AcqRel) {
            return ShutdownOutcome::AlreadyShutDown;
        }
        let taken = {
            let mut current = self
                .current
                .lock()
                .unwrap_or_else(|poisoned| poisoned.into_inner());
            // Latch stop under the lock so the pump's under-lock spawn sees it.
            self.stopping.store(true, Ordering::Release);
            current.take()
        };
        match taken {
            Some(stop) => {
                let outcome = stop.request_and_join(deadline);
                match outcome {
                    ShutdownOutcome::Joined => tracing::info!(
                        "dds_bridge pump: drain thread stopped + joined — DDS participant \
                         dropped (SPDP/SEDP disposes sent, readers unregistered)"
                    ),
                    ShutdownOutcome::TimedOut => tracing::warn!(
                        deadline_ms = deadline.as_millis(),
                        "dds_bridge pump: drain thread did NOT finish within the shutdown \
                         deadline — DETACHING it. The DDS participant may not have sent its \
                         SPDP/SEDP dispose, leaving GHOST READERS on remote writers until the \
                         DDS lease ages them out"
                    ),
                    // request_and_join only returns Joined | TimedOut.
                    ShutdownOutcome::NothingToStop | ShutdownOutcome::AlreadyShutDown => {}
                }
                outcome
            }
            None => {
                tracing::debug!(
                    "dds_bridge pump: shutdown with no live drain thread (never spawned or \
                     already exited) — nothing to stop"
                );
                ShutdownOutcome::NothingToStop
            }
        }
    }
}

/// The pump state machine. Constructed by `DdsBridge::external_source` (or a
/// live-DDS e2e directly) with the validated config + the queue the node drains.
pub struct BridgePump {
    cfg: BridgeConfig,
    queue: Arc<Mutex<SampleQueue>>,
    stats: Arc<PumpStats>,
    drain: Option<DrainHandle>,
    next_init_at: Option<Instant>,
    /// Flood latch for the terminal DEAD reporting (loud first, then debug
    /// — the DEAD arm never recovers).
    dead_latch: FloodLatch,
    /// The transport the RAW routes publish on: CONTEXT-CARRIED
    /// (the production node hands the manager its `NodeContext` carried;
    /// tests hand their isolated `init_for_test` manager). Both go
    /// through [`Self::with_transport`]. `None` (a [`Self::new`] pump) fails
    /// raw init LOUDLY — there is deliberately NO singleton fallback (see
    /// [`resolve_raw_transport`]: the cdylib static trap). Unused when the
    /// config has no raw mappings. `pub(crate)` so the lib.rs regression
    /// test can `Arc::ptr_eq` the carried manager through `build_pump`.
    pub(crate) transport: Option<Arc<TransportManager>>,
    /// The SHARED shutdown coordinator. Cloned onto the node in
    /// `external_source` (BEFORE the pump is moved into the Blocking closure),
    /// so `DdsBridge::shutdown()` can signal + join the drain thread. Each
    /// (re)spawn registers its stop handle here; `iterate` reads `stopping` to
    /// no-op post-shutdown.
    shutdown_coord: Arc<PumpShutdown>,
}

impl BridgePump {
    pub fn new(cfg: BridgeConfig, queue: Arc<Mutex<SampleQueue>>) -> Self {
        Self {
            cfg,
            queue,
            stats: Arc::new(PumpStats::default()),
            drain: None,
            next_init_at: None,
            dead_latch: FloodLatch::new(),
            transport: None,
            shutdown_coord: Arc::new(PumpShutdown::new()),
        }
    }

    /// A clone of the SHARED shutdown coordinator. `DdsBridge`
    /// clones this onto itself in `external_source` before moving the pump into
    /// the `ExternalSource::Blocking` closure — the ONLY path the node retains
    /// to signal + join the drain thread once the pump is closure-captured.
    pub fn shutdown_handle(&self) -> Arc<PumpShutdown> {
        Arc::clone(&self.shutdown_coord)
    }

    /// Signal the drain thread to stop and join it within `deadline` — the
    /// direct (live-DDS e2e / owned-pump) entry to [`PumpShutdown::shutdown`]. The
    /// production node teardown path goes through the cloned [`Self::shutdown_handle`].
    pub fn shutdown(&self, deadline: Duration) -> ShutdownOutcome {
        self.shutdown_coord.shutdown(deadline)
    }

    /// Construct the pump with an EXPLICIT transport for the raw-generic
    /// routes — THE production path: `DdsBridge::external_source`
    /// hands the manager its `NodeContext` carried (injected by the graph
    /// runtime before `init()`; the same FFI path the ports take). Tests
    /// hand their isolated `init_for_test` manager. A [`Self::new`] pump
    /// (no transport) fails raw init LOUDLY — never a singleton fallback
    /// (see [`resolve_raw_transport`]).
    pub fn with_transport(
        cfg: BridgeConfig,
        queue: Arc<Mutex<SampleQueue>>,
        transport: Arc<TransportManager>,
    ) -> Self {
        let mut pump = Self::new(cfg, queue);
        pump.transport = Some(transport);
        pump
    }

    /// The shared hop counters (Principle #3) — read them in diagnostics and
    /// test failure messages.
    pub fn stats(&self) -> Arc<PumpStats> {
        Arc::clone(&self.stats)
    }

    /// True once the drain thread has PANICKED — in a drain task OR the
    /// DDS init phase (terminal — the pump does NOT auto-restart;
    /// dead-until-relaunch). Diagnostics/tests read it to distinguish a DEAD
    /// pump from a merely-quiet UP (or still-PENDING) one.
    pub fn is_dead(&self) -> bool {
        self.drain
            .as_ref()
            .is_some_and(|d| d.status.phase.load(Ordering::Acquire) == PHASE_DEAD)
    }

    /// The FULL production helper-loop step, as driven by the node's
    /// `ExternalSource::Blocking` closure: one [`Self::iterate`] with real
    /// wall time + the paced sleep, returning the doorbell ring. ALL wall
    /// time lives HERE, in helper-thread code OUTSIDE the deterministic
    /// replay surface (replay/polled paths never query
    /// `external_source`, so this code never runs there) — node code (the
    /// `#[cerulion_node_impl]` block) stays wall-clock-free for the
    /// determinism lint, which is right to demand it.
    pub fn run_helper_iteration(&mut self) -> bool {
        let outcome = self.iterate(Instant::now());
        if let Some(pace) = outcome.pace {
            std::thread::sleep(pace);
        }
        outcome.ring
    }

    /// One helper-loop iteration (see the module docs). Time-injected for
    /// testability (the camera `CaptureLoop::iterate` pattern); production
    /// enters via [`Self::run_helper_iteration`].
    pub fn iterate(&mut self, now: Instant) -> PumpOutcome {
        // Once shutdown has latched stop, NEVER spawn/respawn — the
        // host's detached Blocking helper may keep calling the closure after
        // teardown; a fresh drain thread here would orphan a NEW participant.
        // No-op quietly (bounded pace so the helper still observes exit).
        if self.shutdown_coord.is_stopping() {
            return PumpOutcome {
                ring: false,
                pace: Some(BACKOFF_SLICE),
            };
        }
        if self.drain.is_none() {
            if let InitGate::Wait = init_gate(self.next_init_at, now) {
                return PumpOutcome {
                    ring: false,
                    pace: Some(BACKOFF_SLICE),
                };
            }
            self.stats
                .init_attempts_total
                .fetch_add(1, Ordering::Relaxed);
            // Spawn + register the drain thread's stop handle ATOMICALLY under
            // the shutdown lock, re-checking `stopping` first: shutdown() sets
            // `stopping` + takes `current` under the same lock, so a shutdown
            // concurrent with this spawn either (a) wins the lock first — we see
            // `stopping` and abort, no orphan — or (b) loses it — we register,
            // and shutdown then takes+joins our fresh handle. Either way no
            // drain thread is ever left unreachable.
            let mut current = self
                .shutdown_coord
                .current
                .lock()
                .unwrap_or_else(|poisoned| poisoned.into_inner());
            if !should_spawn_under_lock(self.shutdown_coord.stopping.load(Ordering::Acquire)) {
                return PumpOutcome {
                    ring: false,
                    pace: Some(BACKOFF_SLICE),
                };
            }
            match spawn_drain_thread(
                self.cfg.clone(),
                self.transport.clone(),
                Arc::clone(&self.queue),
                Arc::clone(&self.stats),
            ) {
                Ok((handle, stop)) => {
                    *current = Some(stop);
                    drop(current);
                    tracing::info!(
                        mappings = self.cfg.mappings.len(),
                        domain = self.cfg.domain_id,
                        "dds_bridge pump: drain thread spawned (DDS init runs ON it)"
                    );
                    self.drain = Some(handle);
                    self.next_init_at = None;
                }
                Err(e) => {
                    drop(current);
                    self.stats
                        .init_failures_total
                        .fetch_add(1, Ordering::Relaxed);
                    tracing::error!(
                        error = %e,
                        "dds_bridge pump: drain-thread spawn failed — retrying on backoff"
                    );
                    self.next_init_at = Some(now + INIT_BACKOFF);
                    return PumpOutcome {
                        ring: false,
                        pace: Some(BACKOFF_SLICE),
                    };
                }
            }
        }

        let drain = self.drain.as_ref().expect("drain set above");
        match drain.status.phase.load(Ordering::Acquire) {
            PHASE_PENDING => PumpOutcome {
                // DDS init still running on the drain thread.
                ring: false,
                pace: Some(BACKOFF_SLICE),
            },
            PHASE_FAILED => {
                let e = drain.status.take_error();
                self.stats
                    .init_failures_total
                    .fetch_add(1, Ordering::Relaxed);
                tracing::error!(
                    error = %e,
                    "dds_bridge pump: DDS init failed on the drain thread — retrying on \
                     backoff (the bridge stays up; check domain/only_networks/GO2_IFACE)"
                );
                // The failed thread has exited — its participant guard dropped,
                // so the one-per-process slot is free for the retry.
                self.drain = None;
                self.next_init_at = Some(now + INIT_BACKOFF);
                PumpOutcome {
                    ring: false,
                    pace: Some(BACKOFF_SLICE),
                }
            }
            PHASE_DEAD => {
                // The drain thread panicked (in a drain task, or in the
                // DDS init phase — whole-body containment). Terminal —
                // do NOT reset `drain` (no respawn: dead-until-relaunch).
                // Surface loudly via the flood latch, never ring.
                // (Re-borrow `self.drain` fresh inside the arm — the outer
                // `drain` binding is dead on this path, so the `self.dead_latch`
                // mutation below is unambiguously conflict-free.)
                match self.dead_latch.on_event() {
                    FloodAction::First => {
                        let e = self
                            .drain
                            .as_ref()
                            .map(|d| d.status.take_error())
                            .unwrap_or_else(|| "drain thread panicked".to_string());
                        tracing::error!(
                            error = %e,
                            "dds_bridge pump: drain thread DEAD (it PANICKED — DDS init or a \
                             drain task) — ingress is lost until the graph is relaunched (NO \
                             auto-restart)"
                        );
                    }
                    FloodAction::Suppressed { suppressed } => tracing::debug!(
                        suppressed,
                        "dds_bridge pump: drain thread still DEAD (warn suppressed)"
                    ),
                }
                PumpOutcome {
                    ring: false,
                    pace: Some(BACKOFF_SLICE),
                }
            }
            PHASE_UP => PumpOutcome {
                // Up: ring iff the drain thread pushed since the last iterate.
                ring: drain.pushed.swap(false, Ordering::AcqRel),
                pace: Some(POLL_PACE),
            },
            _ => PumpOutcome {
                // Unknown phase — defensive: treat as not-yet-up (never rings).
                ring: false,
                pace: Some(BACKOFF_SLICE),
            },
        }
    }
}

/// Split a full ROS topic (`/api/sport/request`) into `(namespace, base)` for
/// `ros2_client::Name::new`.
fn split_topic(full: &str) -> (String, String) {
    let full = full.trim();
    match full.rfind('/') {
        Some(0) => ("/".to_string(), full[1..].to_string()),
        Some(idx) => (full[..idx].to_string(), full[idx + 1..].to_string()),
        None => ("/".to_string(), full.to_string()),
    }
}

/// One subscription's drain loop —
/// `async_stream()` pinned, `next().await`, push each sample into the
/// latest-wins queue + flag the doorbell bit + bump the hop counters. Error
/// accounting keeps the streak semantics: warn on the first consecutive
/// stream error, debug on repeats, info on recovery. A stream that ENDS is a
/// loud error — that mapping is dead until relaunch (should not happen; the
/// stream is normally infinite).
// Bound note: NOT `M: ros2_client::Message` — the `cerulion_go2_dds` serde structs do
// not implement that marker (no blanket impl in ros2-client 0.10), yet an
// `async_stream()` receive loop compiles and RUNS on the same plain
// serde structs: `Subscription<M>`'s own method bounds are satisfied
// by `DeserializeOwned` alone. Mirror exactly that.
async fn drain_stream<M: serde::de::DeserializeOwned + 'static>(
    sub: Subscription<M>,
    dds_topic: String,
    tag: fn(M) -> BridgeSample,
    queue: Arc<Mutex<SampleQueue>>,
    pushed: Arc<AtomicBool>,
    stats: Arc<PumpStats>,
) {
    let stream = sub.async_stream();
    futures::pin_mut!(stream);
    drain_from_stream(stream, &dds_topic, tag, &queue, &pushed, &stats).await;
}

/// The drain loop over an ABSTRACT stream — extracted from
/// [`drain_stream`] so it is testable with a hand-built `futures::stream::iter`
/// (no live DDS peer). Production behaviour is byte-identical: the counters,
/// tag routing, warn→debug err latching (via [`stream_err_level`]), recovery
/// `info!`, and the stream-END loud error all happen HERE.
async fn drain_from_stream<M, I, E, S>(
    mut stream: Pin<&mut S>,
    dds_topic: &str,
    tag: fn(M) -> BridgeSample,
    queue: &Arc<Mutex<SampleQueue>>,
    pushed: &AtomicBool,
    stats: &PumpStats,
) where
    S: Stream<Item = Result<(M, I), E>>,
    E: std::fmt::Debug,
{
    stats.streams_started_total.fetch_add(1, Ordering::Relaxed);
    let mut err_streak: u64 = 0;
    while let Some(result) = stream.next().await {
        match result {
            Ok((msg, _info)) => {
                if err_streak > 0 {
                    tracing::info!(
                        dds_topic = %dds_topic,
                        after_errors = err_streak,
                        "dds_bridge pump: stream recovered"
                    );
                    err_streak = 0;
                }
                queue
                    .lock()
                    .unwrap_or_else(|poisoned| poisoned.into_inner())
                    .push(tag(msg));
                stats.samples_pushed_total.fetch_add(1, Ordering::Relaxed);
                pushed.store(true, Ordering::Release);
            }
            Err(e) => {
                err_streak += 1;
                stats
                    .drain_stream_errors_total
                    .fetch_add(1, Ordering::Relaxed);
                match stream_err_level(err_streak) {
                    StreamErrLevel::Warn => tracing::warn!(
                        dds_topic = %dds_topic,
                        error = ?e,
                        "dds_bridge pump: stream error (repeats log at debug)"
                    ),
                    StreamErrLevel::Debug => tracing::debug!(
                        dds_topic = %dds_topic,
                        streak = err_streak,
                        error = ?e,
                        "dds_bridge pump: stream still erroring"
                    ),
                }
            }
        }
    }
    stats.streams_ended_total.fetch_add(1, Ordering::Relaxed);
    tracing::error!(
        dds_topic = %dds_topic,
        "dds_bridge pump: subscription stream ENDED — this mapping is dead until \
         the graph is relaunched"
    );
}

// ---------------------------------------------------------------------------
// Raw-generic drains
// ---------------------------------------------------------------------------

/// Resolve the transport the raw routes publish on: the
/// CONTEXT-CARRIED manager ONLY (extracted so the None arm's contract is
/// hermetically pinned). There is deliberately NO `TransportManager::get()` /
/// `get_or_init()` fallback: the bridge runs as a CDYLIB, which links its OWN
/// copy of cerulion_core — its `INSTANCE` static is NEVER the host's (`get()`
/// fails forever, so raw init would retry on backoff without end),
/// and a `get_or_init()` would mint a SECOND manager on the DEFAULT
/// SHM root, silently losing every raw route into the wrong namespace
/// whenever the host runs a non-default config.
fn resolve_raw_transport(
    transport: Option<Arc<TransportManager>>,
) -> Result<Arc<TransportManager>, String> {
    transport.ok_or_else(|| {
        "raw mappings need the HOST's TransportManager, but none was carried into this \
         pump. The transport travels via NodeContext (the graph runtime injects \
         it before init(); DdsBridge::init stores it; external_source hands it to \
         BridgePump::with_transport) — a cdylib CANNOT resolve TransportManager::get(): \
         it links its OWN copy of cerulion_core, whose INSTANCE static the host never \
         initialized, and get_or_init() would mint a SECOND manager on the default SHM \
         root (raw routes silently lost into the wrong namespace). Hand-rolled hosts: \
         construct the pump via BridgePump::with_transport"
            .to_string()
    })
}

/// Build one raw mapping's DDS reader + Cerulion route (drain-thread init).
/// Errors are strings for the `set_failed` convention (retry on backoff).
fn build_raw_binding(
    node: &Node,
    dds_sub: &DdsSubscriber,
    mgr: &Arc<TransportManager>,
    codec: &Arc<CdrCodec>,
    m: &TopicMapping,
) -> Result<(String, RawReader, RawIngressRoute), String> {
    let (ns, base) = split_topic(&m.dds_topic);
    let name = Name::new(&ns, &base)
        .map_err(|e| format!("invalid DDS topic name {:?}: {e:?}", m.dds_topic))?;
    let (pkg, ty) = m
        .ros_type
        .split_once('/')
        .ok_or_else(|| format!("raw ros_type {:?} is not pkg/Type-shaped", m.ros_type))?;
    let qos = match m.qos {
        QosMode::BestEffort => best_effort_qos(),
        QosMode::Reliable => reliable_volatile_qos(),
    };
    // ros2-client applies the ROS `rt/` + type-name mangling; the reader
    // itself drops to rustdds for the raw-CDR adapter (generic::raw docs).
    let topic = node
        .create_topic(&name, MessageTypeName::new(pkg, ty), &qos)
        .map_err(|e| format!("create_topic {:?} failed: {e:?}", m.dds_topic))?;
    let reader = create_raw_reader(dds_sub, &topic, Some(qos))
        .map_err(|e| format!("raw reader for {:?} failed: {e:?}", m.dds_topic))?;
    // The slice is derived PER ROUTE from the route's own schema (an
    // explicit `max_slice_len:` still wins). It is the slice that sets the
    // route's receive-queue depth, and the depth is the frame-loss boundary —
    // see `BridgeConfig::raw_slice_len_for_route`.
    let route = RawIngressRoute::open(
        mgr,
        codec,
        &m.ros_type,
        &m.cerulion_topic,
        BridgeConfig::raw_slice_len_for_route(m, codec),
    )
    .map_err(|e| format!("raw route init failed: {e}"))?;
    tracing::info!(
        dds_topic = %m.dds_topic,
        ros_type = %m.ros_type,
        cerulion_topic = %m.cerulion_topic,
        qos = ?m.qos,
        "dds_bridge pump: RAW route created (on the drain thread)"
    );
    Ok((m.dds_topic.clone(), reader, route))
}

/// One raw mapping's drain loop: `as_async_stream()` over the raw-CDR reader
/// (the same executor shape as the typed drains), each sample transcoded +
/// published via [`RawIngressRoute::publish_cdr_body`]. Wire timestamps are
/// the TRANSPORT clock at drain-publish time — consistent with how the typed
/// ports stamp (the publisher's loan-time clock) and deliberately NOT the
/// DDS writer's `source_timestamp` (a remote peer's wall clock; see the
/// lib.rs raw-path docs).
async fn drain_raw_stream(
    reader: RawReader,
    dds_topic: String,
    route: RawIngressRoute,
    codec: Arc<CdrCodec>,
    clock: Arc<dyn Clock>,
    stats: Arc<PumpStats>,
) {
    let stream = reader
        .as_async_stream()
        .map(|r| r.map(|dcc| dcc.into_value()));
    futures::pin_mut!(stream);
    let now_ns = || clock.now_ns();
    let _route = drain_raw_from_stream(stream, &dds_topic, route, &codec, &now_ns, &stats).await;
}

/// The raw drain loop over an ABSTRACT stream — the raw sibling of
/// [`drain_from_stream`], testable with a hand-built `futures::stream::iter`
/// (no DDS). Same discipline: stream-item errors keep the warn→debug
/// `err_streak` latching + recovery `info!`; a stream END is a loud error +
/// counter. Route (transcode/publish) failures are flood-latched separately
/// (a hostile peer's frames must not flood the log) and NEVER burn a wire
/// sequence (the route's commit rule). Returns the route so tests
/// can inspect its counters.
async fn drain_raw_from_stream<E, S>(
    mut stream: Pin<&mut S>,
    dds_topic: &str,
    mut route: RawIngressRoute,
    codec: &CdrCodec,
    now_ns: &dyn Fn() -> u64,
    stats: &PumpStats,
) -> RawIngressRoute
where
    S: Stream<Item = Result<RawSample, E>>,
    E: std::fmt::Debug,
{
    stats
        .raw_streams_started_total
        .fetch_add(1, Ordering::Relaxed);
    let mut err_streak: u64 = 0;
    let mut route_latch = FloodLatch::new();
    while let Some(result) = stream.next().await {
        match result {
            Ok(raw) => {
                if err_streak > 0 {
                    tracing::info!(
                        dds_topic = %dds_topic,
                        after_errors = err_streak,
                        "dds_bridge pump: raw stream recovered"
                    );
                    err_streak = 0;
                }
                match route.publish_cdr_body(codec, raw.endianness, &raw.body, now_ns()) {
                    Ok(_recipients) => {
                        stats.raw_published_total.fetch_add(1, Ordering::Relaxed);
                        if let Some(suppressed) = route_latch.on_recovered() {
                            tracing::info!(
                                dds_topic = %dds_topic,
                                suppressed,
                                "dds_bridge pump: raw route recovered"
                            );
                        }
                    }
                    Err(e) => {
                        stats
                            .raw_route_failures_total
                            .fetch_add(1, Ordering::Relaxed);
                        match route_latch.on_event() {
                            FloodAction::First => tracing::warn!(
                                dds_topic = %dds_topic,
                                error = %e,
                                "dds_bridge pump: raw route transcode/publish failed — sample \
                                 dropped, sequence not burned (repeats log at debug)"
                            ),
                            FloodAction::Suppressed { suppressed } => tracing::debug!(
                                dds_topic = %dds_topic,
                                suppressed,
                                error = %e,
                                "dds_bridge pump: raw route still failing"
                            ),
                        }
                    }
                }
            }
            Err(e) => {
                err_streak += 1;
                stats
                    .raw_stream_errors_total
                    .fetch_add(1, Ordering::Relaxed);
                match stream_err_level(err_streak) {
                    StreamErrLevel::Warn => tracing::warn!(
                        dds_topic = %dds_topic,
                        error = ?e,
                        "dds_bridge pump: raw stream error (repeats log at debug)"
                    ),
                    StreamErrLevel::Debug => tracing::debug!(
                        dds_topic = %dds_topic,
                        streak = err_streak,
                        error = ?e,
                        "dds_bridge pump: raw stream still erroring"
                    ),
                }
            }
        }
    }
    stats
        .raw_streams_ended_total
        .fetch_add(1, Ordering::Relaxed);
    tracing::error!(
        dds_topic = %dds_topic,
        "dds_bridge pump: raw stream ENDED — this mapping is dead until the graph \
         is relaunched"
    );
    route
}

/// Spawn the drain thread. ALL DDS objects are created ON that thread (module
/// docs — the all-green probe shape); only config/queue/flag/stats handles
/// cross the boundary. Returns as soon as the thread is spawned; the init
/// outcome arrives via [`StartupStatus`].
///
/// Also mints the STOP + FINISHED channels — `stop_rx` drives the
/// drain executor's completion (see [`drain_thread_body`]); `finished_rx` fires
/// after the thread's body returns (participant dropped + disposed). The
/// returned [`DrainThreadStop`] is registered into [`PumpShutdown::current`] by
/// the caller so `NodeEntry::shutdown()` can signal + join this thread.
fn spawn_drain_thread(
    cfg: BridgeConfig,
    transport: Option<Arc<TransportManager>>,
    queue: Arc<Mutex<SampleQueue>>,
    stats: Arc<PumpStats>,
) -> Result<(DrainHandle, DrainThreadStop), String> {
    let pushed = Arc::new(AtomicBool::new(false));
    let status = Arc::new(StartupStatus::new());
    let (stop_tx, stop_rx) = futures::channel::oneshot::channel::<()>();
    let (finished_tx, finished_rx) = std::sync::mpsc::channel::<()>();
    let handle = DrainHandle {
        pushed: Arc::clone(&pushed),
        status: Arc::clone(&status),
    };
    let join = std::thread::Builder::new()
        .name("dds-bridge-drain".to_string())
        .spawn(move || {
            drain_thread_main(
                cfg,
                transport,
                queue,
                pushed,
                stats,
                status,
                stop_rx,
                finished_tx,
            )
        })
        .map_err(|e| format!("failed to spawn drain thread: {e}"))?;
    Ok((
        handle,
        DrainThreadStop {
            stop_tx,
            finished_rx,
            join,
        },
    ))
}

/// One typed subscription paired with its port tag (drain-thread-local; never
/// crosses a thread boundary).
enum SubKind {
    Cloud(Subscription<messages::PointCloud2>),
    Sport(Subscription<messages::SportModeState>),
    Twist(Subscription<messages::Twist>),
    Request(Subscription<messages::Request>),
}

/// The drain-thread ENTRY POINT: run [`drain_thread_body`] under whole-body
/// panic containment. The DDS init phase (`Go2Participant::new`,
/// `create_node`, `create_topic`, `create_subscription`, `spinner` — all
/// third-party rustdds code, the most panic-prone part: socket/multicast/
/// interface setup) runs BEFORE the executor's own containment
/// ([`run_drain_executor_contained`]); without this outer boundary a panic
/// there kills the thread while [`StartupStatus`] stays `PHASE_PENDING`
/// forever — every iterate reports "init still running", the bridge silently
/// produces no ingress indefinitely, no DEAD/FAILED surfacing, no operator
/// log. With this boundary a panic there marks `PHASE_DEAD` + ONE loud `error!`
/// (the terminal DEAD semantics).
///
/// After the contained body returns — which, on the graceful-stop path,
/// is AFTER the participant guard dropped and its SPDP/SEDP disposes were sent —
/// signal `finished` so [`DrainThreadStop::request_and_join`]'s bounded wait can
/// reap the thread. The send is best-effort: if the receiver was already dropped
/// (e.g. the pump respawned past a FAILED thread), the drop-disconnect is an
/// equally valid "finished" signal.
#[allow(clippy::too_many_arguments)]
fn drain_thread_main(
    cfg: BridgeConfig,
    transport: Option<Arc<TransportManager>>,
    queue: Arc<Mutex<SampleQueue>>,
    pushed: Arc<AtomicBool>,
    stats: Arc<PumpStats>,
    status: Arc<StartupStatus>,
    stop_rx: futures::channel::oneshot::Receiver<()>,
    finished_tx: MpscSender<()>,
) {
    let status_boundary = Arc::clone(&status);
    run_drain_thread_contained(&status_boundary, move || {
        drain_thread_body(cfg, transport, queue, pushed, stats, status, stop_rx)
    });
    // The body has returned: on the stop path the participant guard has dropped
    // (dispose sent). Announce finished so a bounded join can reap us.
    let _ = finished_tx.send(());
}

/// The whole-thread-body panic boundary (see [`drain_thread_main`]).
/// Extracted so the PENDING→DEAD transition is testable with an injected
/// panicking body (no DDS). Executor-phase panics are already contained INSIDE
/// the body by [`run_drain_executor_contained`] (which sets DEAD itself and
/// returns normally), so this boundary only ever fires for panics OUTSIDE it —
/// the DDS init/setup phase.
fn run_drain_thread_contained(status: &StartupStatus, body: impl FnOnce()) {
    let outcome = std::panic::catch_unwind(AssertUnwindSafe(body));
    if let Err(payload) = outcome {
        let msg = panic_payload_string(payload.as_ref());
        status.set_dead(msg.clone());
        tracing::error!(
            panic = %msg,
            "dds_bridge pump: drain thread PANICKED during DDS init/setup — DDS ingress \
             is DEAD until the graph is relaunched (NO auto-restart)"
        );
    }
}

/// Test seam: when armed (self-disarming swap), [`drain_thread_body`]
/// panics at entry — modelling a third-party panic in the DDS init phase,
/// BEFORE any DDS object exists. `cfg(test)`-only; lets the wiring test drive
/// the REAL [`drain_thread_main`] on a real thread with zero DDS.
#[cfg(test)]
static INIT_PANIC_FOR_TEST: AtomicBool = AtomicBool::new(false);

/// The drain thread BODY: build the WHOLE DDS side here, report the outcome,
/// then drive the spinner + every drain stream on one `LocalExecutor` until
/// `stop_rx` resolves (the driver is the stop future, not a
/// `future::pending()` forever-driver, so
/// shutdown can return the executor and drop the participant). Runs under
/// [`run_drain_thread_contained`]'s panic boundary.
fn drain_thread_body(
    cfg: BridgeConfig,
    transport: Option<Arc<TransportManager>>,
    queue: Arc<Mutex<SampleQueue>>,
    pushed: Arc<AtomicBool>,
    stats: Arc<PumpStats>,
    status: Arc<StartupStatus>,
    stop_rx: futures::channel::oneshot::Receiver<()>,
) {
    #[cfg(test)]
    if INIT_PANIC_FOR_TEST.swap(false, Ordering::SeqCst) {
        panic!("injected DDS-init panic (test seam)");
    }
    // ---- DDS init, ON this thread ------------------------------------------
    // The one-per-process guard + only_networks precedence
    // (config value → GO2_IFACE env → loud warn) live in ParticipantConfig.
    let pconf = ParticipantConfig::new(cfg.domain_id, cfg.only_networks.clone());
    let participant = match Go2Participant::new(&pconf) {
        Ok(p) => p,
        Err(e) => return status.set_failed(e.to_string()),
    };
    let mut node = match participant.create_node("dds_bridge_pump") {
        Ok(n) => n,
        Err(e) => return status.set_failed(e.to_string()),
    };

    let mut subs: Vec<(String, SubKind)> = Vec::new();
    for (ros_type, m) in cfg.typed_mappings() {
        let (ns, base) = split_topic(&m.dds_topic);
        let name = match Name::new(&ns, &base) {
            Ok(n) => n,
            Err(e) => {
                return status
                    .set_failed(format!("invalid DDS topic name {:?}: {e:?}", m.dds_topic))
            }
        };
        let (pkg, ty) = ros_type.pkg_and_type();
        let qos = match m.qos {
            QosMode::BestEffort => best_effort_qos(),
            QosMode::Reliable => reliable_volatile_qos(),
        };
        let topic = match node.create_topic(&name, MessageTypeName::new(pkg, ty), &qos) {
            Ok(t) => t,
            Err(e) => {
                return status.set_failed(format!("create_topic {:?} failed: {e:?}", m.dds_topic))
            }
        };
        let kind = match ros_type {
            RosType::PointCloud2 => {
                match node.create_subscription::<messages::PointCloud2>(&topic, Some(qos)) {
                    Ok(s) => SubKind::Cloud(s),
                    Err(e) => {
                        return status.set_failed(format!(
                            "create_subscription {:?} failed: {e:?}",
                            m.dds_topic
                        ))
                    }
                }
            }
            RosType::SportModeState => {
                match node.create_subscription::<messages::SportModeState>(&topic, Some(qos)) {
                    Ok(s) => SubKind::Sport(s),
                    Err(e) => {
                        return status.set_failed(format!(
                            "create_subscription {:?} failed: {e:?}",
                            m.dds_topic
                        ))
                    }
                }
            }
            RosType::Twist => {
                match node.create_subscription::<messages::Twist>(&topic, Some(qos)) {
                    Ok(s) => SubKind::Twist(s),
                    Err(e) => {
                        return status.set_failed(format!(
                            "create_subscription {:?} failed: {e:?}",
                            m.dds_topic
                        ))
                    }
                }
            }
            RosType::Request => {
                match node.create_subscription::<messages::Request>(&topic, Some(qos)) {
                    Ok(s) => SubKind::Request(s),
                    Err(e) => {
                        return status.set_failed(format!(
                            "create_subscription {:?} failed: {e:?}",
                            m.dds_topic
                        ))
                    }
                }
            }
        };
        tracing::info!(
            dds_topic = %m.dds_topic,
            ros_type = ros_type.ros_type_str(),
            port = ros_type.port_name(),
            qos = ?m.qos,
            "dds_bridge pump: subscription created (on the drain thread)"
        );
        subs.push((m.dds_topic.clone(), kind));
    }

    // ---- Raw-generic mappings, also ON this thread --------
    // Each schema-resolvable non-registry mapping gets a raw-CDR reader (the
    // rustdds drop-down documented in `generic::raw`) + a `RawIngressRoute`
    // (codec-transcoded `publish_raw`). Init failures follow the SAME
    // status/backoff convention as the typed side (`set_failed` → retry);
    // panics ride the whole-body containment.
    let raw_mappings = cfg.raw_mappings();
    let mut raws: Vec<(String, RawReader, RawIngressRoute)> = Vec::new();
    let mut raw_codec: Option<Arc<CdrCodec>> = None;
    let mut raw_clock: Option<Arc<dyn Clock>> = None;
    let mut raw_dds_subscriber: Option<DdsSubscriber> = None;
    if !raw_mappings.is_empty() {
        let mgr = match resolve_raw_transport(transport) {
            Ok(m) => m,
            Err(e) => return status.set_failed(e),
        };
        // Seed the run-time codec with the config's workspace
        // `.msg` store dirs, so raw mappings of store-only types transcode. THE
        // SAME `BridgeConfig::runtime_codec` seam that config validation uses to
        // ACCEPT these mappings — so a store-only mapping that validated at load
        // is transcodable here (they share `effective_msg_dirs`).
        let codec = Arc::new(cfg.runtime_codec());
        let dds_sub = match participant
            .context()
            .domain_participant()
            .create_subscriber(&best_effort_qos())
        {
            Ok(s) => s,
            Err(e) => {
                return status
                    .set_failed(format!("rustdds subscriber for raw mappings failed: {e:?}"))
            }
        };
        for m in raw_mappings {
            match build_raw_binding(&node, &dds_sub, &mgr, &codec, m) {
                Ok(binding) => raws.push(binding),
                Err(e) => return status.set_failed(e),
            }
        }
        raw_clock = Some(mgr.clock_arc());
        raw_codec = Some(codec);
        raw_dds_subscriber = Some(dds_sub);
    }

    let spinner = match node.spinner() {
        Ok(s) => s,
        Err(e) => return status.set_failed(format!("spinner: {e:?}")),
    };

    // ---- Init done: report up, then drive until the STOP signal -------------
    // (NOT forever: `stop_rx` completing returns the executor so this
    // fn exits and the DDS guards below drop; see the driver comment ~50 lines
    // down and the participant-guard note just below.)
    status.set_up();
    tracing::info!(
        subscriptions = subs.len(),
        raw_routes = raws.len(),
        "dds_bridge pump: DDS side up on the drain thread (async_stream drains running)"
    );

    // Keep the participant + node alive for the life of the thread — dropping
    // the participant MID-RUN would free the one-per-process slot AND kill DDS.
    // The rustdds subscriber hosting the raw readers stays alive alongside them.
    // But the guard-drop on thread EXIT (the stop-signal path below) is NOT
    // merely destructive — it IS the intended graceful teardown: the participant
    // Drop sends the SPDP participant-dispose + SEDP endpoint-disposes that
    // unregister this node's readers, so remote writers see no ghost readers.
    let _participant_guard = participant;
    let _node_guard = node;
    let _raw_subscriber_guard = raw_dds_subscriber;

    let ex = smol::LocalExecutor::new();
    ex.spawn(async move {
        if let Err(e) = spinner.spin().await {
            tracing::error!(error = ?e, "dds_bridge pump: node spinner exited with error");
        }
    })
    .detach();
    for (dds_topic, kind) in subs {
        let (q, p, st) = (Arc::clone(&queue), Arc::clone(&pushed), Arc::clone(&stats));
        match kind {
            SubKind::Cloud(sub) => ex
                .spawn(drain_stream(sub, dds_topic, BridgeSample::Cloud, q, p, st))
                .detach(),
            SubKind::Sport(sub) => ex
                .spawn(drain_stream(sub, dds_topic, BridgeSample::Sport, q, p, st))
                .detach(),
            SubKind::Twist(sub) => ex
                .spawn(drain_stream(sub, dds_topic, BridgeSample::Twist, q, p, st))
                .detach(),
            SubKind::Request(sub) => ex
                .spawn(drain_stream(
                    sub,
                    dds_topic,
                    BridgeSample::Request,
                    q,
                    p,
                    st,
                ))
                .detach(),
        }
    }
    // Raw-generic drains: same executor, same lifecycle. NOTE: raw
    // publishes go straight to iceoryx2 (`publish_raw`) OFF the node tick —
    // they deliberately do NOT touch the `pushed` doorbell bit (there is
    // nothing for the tick to drain).
    for (dds_topic, reader, route) in raws {
        let codec = Arc::clone(raw_codec.as_ref().expect("codec built with raw mappings"));
        let clock = Arc::clone(
            raw_clock
                .as_ref()
                .expect("clock resolved with raw mappings"),
        );
        let st = Arc::clone(&stats);
        ex.spawn(drain_raw_stream(reader, dds_topic, route, codec, clock, st))
            .detach();
    }
    // Drive the executor until the STOP signal, NOT forever: the
    // detached drain tasks are the work; `stop_rx` completing (on shutdown's
    // `stop_tx` drop) returns the executor so this fn returns and the DDS
    // guards below drop — sending the SPDP/SEDP disposes. Wrapped in panic
    // containment: a panic in any drain task must not kill the thread
    // SILENTLY and leave the pump reporting healthy forever.
    let stop_driver = async move {
        // A cancelled receiver (shutdown dropped `stop_tx`) is the stop signal,
        // exactly like a value send — both complete the driver.
        let _ = stop_rx.await;
    };
    run_drain_executor_contained(ex, stop_driver, &status);
}

/// Drive `driver` on `ex` with panic containment. A panic in ANY drain task
/// unwinds out of `block_on`; catch it, mark the drain thread DEAD in `status`,
/// and log ONE loud error — instead of the thread dying silently. Production
/// passes the STOP future (drive until shutdown); the tasks are detached onto
/// `ex` before this call. This EXECUTOR-phase boundary catches + returns
/// normally, so the outer whole-body boundary ([`run_drain_thread_contained`])
/// never double-handles it — the outer one fires only for panics OUTSIDE this
/// call (the DDS init phase).
fn run_drain_executor_contained<F>(ex: smol::LocalExecutor<'_>, driver: F, status: &StartupStatus)
where
    F: std::future::Future<Output = ()>,
{
    let outcome = std::panic::catch_unwind(AssertUnwindSafe(|| {
        smol::block_on(ex.run(driver));
    }));
    if let Err(payload) = outcome {
        let msg = panic_payload_string(payload.as_ref());
        status.set_dead(msg.clone());
        tracing::error!(
            panic = %msg,
            "dds_bridge pump: drain thread executor PANICKED — DDS ingress is DEAD until \
             the graph is relaunched (no auto-restart; all mappings lost)"
        );
    }
}

/// Best-effort render of a caught panic payload as a string.
fn panic_payload_string(payload: &(dyn std::any::Any + Send)) -> String {
    if let Some(s) = payload.downcast_ref::<&'static str>() {
        (*s).to_string()
    } else if let Some(s) = payload.downcast_ref::<String>() {
        s.clone()
    } else {
        "non-string panic payload".to_string()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn split_topic_matches_the_spike_semantics() {
        assert_eq!(
            split_topic("/api/sport/request"),
            ("/api/sport".to_string(), "request".to_string())
        );
        assert_eq!(
            split_topic("/utlidar/cloud"),
            ("/utlidar".to_string(), "cloud".to_string())
        );
        assert_eq!(
            split_topic("/cmd_vel"),
            ("/".to_string(), "cmd_vel".to_string())
        );
        assert_eq!(split_topic("bare"), ("/".to_string(), "bare".to_string()));
    }

    #[test]
    fn startup_status_phases_and_error_surface() {
        let s = StartupStatus::new();
        assert_eq!(s.phase.load(Ordering::Acquire), PHASE_PENDING);
        s.set_failed("boom".to_string());
        assert_eq!(s.phase.load(Ordering::Acquire), PHASE_FAILED);
        assert_eq!(s.take_error(), "boom");
        // A drained error slot degrades to the fallback text, never a panic.
        assert!(s.take_error().contains("unknown"));
        let up = StartupStatus::new();
        up.set_up();
        assert_eq!(up.phase.load(Ordering::Acquire), PHASE_UP);
    }

    #[test]
    fn drain_seam_pushes_tag_flags_and_counts() {
        // The push half of the drain contract WITHOUT DDS: pushing a tagged
        // sample into the shared queue, flagging `pushed`, and bumping the
        // counter — exactly what each Ok(...) stream item does — pinned at
        // the queue/flag/stats seam (the stream half needs a live peer; that
        // is the live-DDS e2e's job, whose failure message renders these stats).
        let queue: Arc<Mutex<SampleQueue>> = Arc::new(Mutex::new(SampleQueue::default()));
        let pushed = Arc::new(AtomicBool::new(false));
        let stats = Arc::new(PumpStats::default());
        let tag: fn(messages::Twist) -> BridgeSample = BridgeSample::Twist;
        queue
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner())
            .push(tag(messages::Twist::default()));
        stats.samples_pushed_total.fetch_add(1, Ordering::Relaxed);
        pushed.store(true, Ordering::Release);
        // The iterate edge: first swap rings, second is quiet.
        assert!(pushed.swap(false, Ordering::AcqRel));
        assert!(!pushed.swap(false, Ordering::AcqRel));
        assert_eq!(stats.samples_pushed_total.load(Ordering::Relaxed), 1);
        assert!(
            stats.render().contains("samples_pushed=1"),
            "{}",
            stats.render()
        );
        let drained = queue
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner())
            .drain_all();
        assert_eq!(drained.samples.len(), 1);
        assert_eq!(drained.samples[0].ros_type(), RosType::Twist);
    }

    // ---- Shared helpers ---------------------------------------------------

    fn test_config() -> BridgeConfig {
        BridgeConfig::from_yaml(
            "mappings:\n  - dds_topic: /x\n    ros_type: geometry_msgs/Twist\n    \
             cerulion_topic: /y/x\n",
            "pump test",
        )
        .expect("minimal pump test config")
    }

    fn test_pump() -> BridgePump {
        BridgePump::new(test_config(), Arc::new(Mutex::new(SampleQueue::default())))
    }

    fn handle(pushed: bool, status: Arc<StartupStatus>) -> DrainHandle {
        DrainHandle {
            pushed: Arc::new(AtomicBool::new(pushed)),
            status,
        }
    }

    // ---- drain_from_stream over a hand-built stream -----------------------

    /// Drive ONE Ok item through `drain_from_stream` and return the drained
    /// sample — the per-tag routing pin (asserts the counters + queue slot).
    fn run_one_ok<M: 'static>(tag: fn(M) -> BridgeSample, msg: M) -> BridgeSample {
        let queue: Arc<Mutex<SampleQueue>> = Arc::new(Mutex::new(SampleQueue::default()));
        let pushed = Arc::new(AtomicBool::new(false));
        let stats = Arc::new(PumpStats::default());
        let items: Vec<Result<(M, ()), &'static str>> = vec![Ok((msg, ()))];
        let stream = futures::stream::iter(items);
        futures::pin_mut!(stream);
        smol::block_on(drain_from_stream(
            stream, "/x", tag, &queue, &pushed, &stats,
        ));
        assert_eq!(stats.streams_started_total.load(Ordering::Relaxed), 1);
        assert_eq!(stats.samples_pushed_total.load(Ordering::Relaxed), 1);
        assert_eq!(stats.streams_ended_total.load(Ordering::Relaxed), 1);
        assert!(
            pushed.load(Ordering::Relaxed),
            "an Ok item flags the doorbell"
        );
        let mut drained = queue.lock().unwrap_or_else(|p| p.into_inner()).drain_all();
        assert_eq!(drained.samples.len(), 1);
        drained.samples.pop().unwrap()
    }

    #[test]
    fn drain_from_stream_routes_each_tag_to_its_slot() {
        // Each BridgeSample arm is POSITIVELY asserted (not an assertion-free
        // probe): the tag routes the message into ITS fixed slot.
        assert_eq!(
            run_one_ok(BridgeSample::Cloud, messages::PointCloud2::default()).ros_type(),
            RosType::PointCloud2
        );
        assert_eq!(
            run_one_ok(BridgeSample::Sport, messages::SportModeState::default()).ros_type(),
            RosType::SportModeState
        );
        assert_eq!(
            run_one_ok(BridgeSample::Twist, messages::Twist::default()).ros_type(),
            RosType::Twist
        );
        assert_eq!(
            run_one_ok(BridgeSample::Request, messages::Request::default()).ros_type(),
            RosType::Request
        );
    }

    #[test]
    fn drain_from_stream_counts_errors_recovers_and_ends() {
        let queue: Arc<Mutex<SampleQueue>> = Arc::new(Mutex::new(SampleQueue::default()));
        let pushed = Arc::new(AtomicBool::new(false));
        let stats = Arc::new(PumpStats::default());
        let a = messages::Twist {
            linear: messages::Vector3 {
                x: 1.0,
                y: 0.0,
                z: 0.0,
            },
            angular: messages::Vector3::default(),
        };
        let b = messages::Twist {
            linear: messages::Vector3 {
                x: 2.0,
                y: 0.0,
                z: 0.0,
            },
            angular: messages::Vector3::default(),
        };
        // Ok(a), Err, Err, Ok(b), then end: two pushes, two errors (streak
        // recovers on the second Ok), one stream-end.
        let items: Vec<Result<(messages::Twist, ()), &'static str>> =
            vec![Ok((a, ())), Err("boom1"), Err("boom2"), Ok((b.clone(), ()))];
        let stream = futures::stream::iter(items);
        futures::pin_mut!(stream);
        smol::block_on(drain_from_stream(
            stream,
            "/t",
            BridgeSample::Twist,
            &queue,
            &pushed,
            &stats,
        ));
        assert_eq!(stats.samples_pushed_total.load(Ordering::Relaxed), 2);
        assert_eq!(stats.drain_stream_errors_total.load(Ordering::Relaxed), 2);
        assert_eq!(stats.streams_started_total.load(Ordering::Relaxed), 1);
        assert_eq!(
            stats.streams_ended_total.load(Ordering::Relaxed),
            1,
            "the stream end bumps streams_ended_total + logs a loud error"
        );
        // Latest-wins: the queue holds b (the Ok that recovered the streak).
        let drained = queue.lock().unwrap_or_else(|p| p.into_inner()).drain_all();
        assert_eq!(drained.samples, vec![BridgeSample::Twist(b)]);
    }

    // ---- The context-carried transport contract ----------------------------

    #[test]
    fn resolve_raw_transport_none_is_a_loud_context_contract_error() {
        // The None arm: a pump without a carried transport must FAIL raw init
        // with the context-carried contract named — never fall back to
        // cross-linkage statics (the cdylib trap: `get()` reads the cdylib's
        // own never-initialized INSTANCE and fails on every backoff retry;
        // a get_or_init fallback would silently mint a SECOND
        // manager on the default SHM root instead).
        let Err(err) = resolve_raw_transport(None) else {
            panic!("resolve_raw_transport(None) must be a loud Err (the Ok type is not Debug)")
        };
        for needle in [
            "NodeContext",
            "BridgePump::with_transport",
            "OWN copy of cerulion_core",
            "SECOND manager",
            "TransportManager::get()",
        ] {
            assert!(err.contains(needle), "missing {needle:?} in: {err}");
        }
    }

    #[test]
    fn resolve_raw_transport_some_returns_the_same_manager() {
        let mgr = raw_test_transport("resolve_some");
        let resolved = resolve_raw_transport(Some(Arc::clone(&mgr))).expect("Some resolves");
        assert!(
            Arc::ptr_eq(&resolved, &mgr),
            "the carried manager is returned untouched (no re-resolution)"
        );
    }

    // ---- drain_raw_from_stream over a hand-built stream --

    /// Fresh isolated transport for the raw-drain tests (per-test SHM root —
    /// parallel-safe; the raw_route_test.rs `build_mgr` pattern).
    fn raw_test_transport(tag: &str) -> Arc<TransportManager> {
        TransportManager::init_for_test(
            cerulion_core::transport::TransportConfig {
                node_name: format!("pump_raw_{tag}"),
                clock: Arc::new(cerulion_core::clock::VirtualClock::new()),
                subscriber_buffer_size: 16,
                network: None,
            },
            cerulion_core::testing::iceoryx_test_config(),
        )
        .expect("isolated transport")
    }

    /// A LE Vector3 CDR body as a [`RawSample`] (what the raw adapter yields
    /// — encapsulation already stripped).
    fn raw_v3_sample(x: f64, y: f64, z: f64) -> RawSample {
        let mut body = Vec::new();
        body.extend_from_slice(&x.to_le_bytes());
        body.extend_from_slice(&y.to_le_bytes());
        body.extend_from_slice(&z.to_le_bytes());
        RawSample {
            endianness: cerulion_core::codegen::CdrEndianness::Little,
            body,
        }
    }

    /// Hand-built expected wire frame for that Vector3 (the raw_route_test
    /// oracle: generated SCHEMA_HASH + frame-relative table offset).
    fn expected_v3_frame(x: f64, y: f64, z: f64, seq: u32, ts: u64) -> Vec<u8> {
        use cerulion_core::message::ShmMessage;
        use cerulion_core::wire::WireHeader;
        let header = WireHeader {
            schema_hash: <native_ros2_messages::geometry_msgs::Vector3 as ShmMessage>::SCHEMA_HASH,
            total_size: (WireHeader::SIZE + 24) as u32,
            offset_table_offset: (WireHeader::SIZE + 24) as u32,
            offset_table_count: 0,
            sequence: seq,
            timestamp_ns: ts,
        };
        let mut f = vec![0u8; WireHeader::SIZE];
        header.write_to_buf(&mut f);
        f.extend_from_slice(&x.to_le_bytes());
        f.extend_from_slice(&y.to_le_bytes());
        f.extend_from_slice(&z.to_le_bytes());
        f
    }

    /// Drain all frames off a subscriber as full wire-frame bytes.
    fn captured_frames(sub: &cerulion_core::CerulionSubscriber) -> Vec<Vec<u8>> {
        use cerulion_core::wire::WireHeader;
        let mut frames = Vec::new();
        sub.try_receive(|msg| {
            let mut f = vec![0u8; WireHeader::SIZE];
            msg.header().write_to_buf(&mut f);
            f.extend_from_slice(msg.payload());
            frames.push(f);
        })
        .expect("try_receive");
        frames
    }

    /// Run the raw-drain seam over the canonical 4-item script — Ok(good),
    /// stream Err, Ok(hostile body → route Decode failure), Ok(good) — on a
    /// fresh isolated transport; returns the captured frames + final stats.
    fn run_raw_drain_script(tag: &str) -> (Vec<Vec<u8>>, PumpStats, RawIngressRoute) {
        use cerulion_core::wire::MaxSliceLen;
        let mgr = raw_test_transport(tag);
        let codec = crate::generic::bridge_codec();
        let topic = format!("/pump726/{tag}/v3");
        let route = RawIngressRoute::open(
            &mgr,
            &codec,
            "geometry_msgs/Vector3",
            &topic,
            MaxSliceLen::const_new(4096),
        )
        .expect("raw route");
        let sub = mgr.create_subscriber(&topic).expect("observer");
        let stats = PumpStats::default();

        let items: Vec<Result<RawSample, &'static str>> = vec![
            Ok(raw_v3_sample(1.0, 2.0, 3.0)),
            Err("stream boom"),
            Ok(RawSample {
                endianness: cerulion_core::codegen::CdrEndianness::Little,
                body: vec![1, 2, 3], // truncated → route Decode failure
            }),
            Ok(raw_v3_sample(4.0, 5.0, 6.0)),
        ];
        let stream = futures::stream::iter(items);
        futures::pin_mut!(stream);
        // Scripted clock: +100 per publish ATTEMPT (the failed decode
        // consumes a stamp, never a sequence) → stamps 100, 200, 300.
        let t = std::cell::Cell::new(0u64);
        let now = || {
            t.set(t.get() + 100);
            t.get()
        };
        let route = smol::block_on(drain_raw_from_stream(
            stream, "/raw_dds", route, &codec, &now, &stats,
        ));
        (captured_frames(&sub), stats, route)
    }

    #[test]
    fn drain_raw_from_stream_transcodes_publishes_and_counts() {
        let (frames, stats, route) = run_raw_drain_script("counts");

        // The raw hop counters (Principle #3 — these localize a dead raw hop).
        assert_eq!(stats.raw_streams_started_total.load(Ordering::Relaxed), 1);
        assert_eq!(stats.raw_streams_ended_total.load(Ordering::Relaxed), 1);
        assert_eq!(stats.raw_published_total.load(Ordering::Relaxed), 2);
        assert_eq!(stats.raw_route_failures_total.load(Ordering::Relaxed), 1);
        assert_eq!(stats.raw_stream_errors_total.load(Ordering::Relaxed), 1);
        assert!(
            stats.render().contains("raw_published=2"),
            "render carries the raw terms: {}",
            stats.render()
        );
        // The route is returned for inspection (its own per-route counters).
        assert_eq!(route.published_total(), 2);
        assert_eq!(route.decode_failures_total(), 1);
        assert_eq!(route.next_sequence(), 2, "the failure burned NO sequence");

        // Delivered frames == hand oracles: gap-free seq 0,1 across the
        // failure; stamps 100 and 300 (the failed attempt consumed 200).
        assert_eq!(frames.len(), 2);
        assert_eq!(frames[0], expected_v3_frame(1.0, 2.0, 3.0, 0, 100));
        assert_eq!(frames[1], expected_v3_frame(4.0, 5.0, 6.0, 1, 300));
    }

    #[test]
    fn drain_raw_from_stream_is_deterministic_across_isolated_runs() {
        // Principle #7: the same script on two isolated transports yields
        // byte-identical captures, each anchored to the hand oracle above
        // (never a bare self-compare).
        let (a, _, _) = run_raw_drain_script("det_a");
        let (b, _, _) = run_raw_drain_script("det_b");
        assert_eq!(a, b, "two isolated runs are byte-identical");
        assert_eq!(a[0], expected_v3_frame(1.0, 2.0, 3.0, 0, 100));
        assert_eq!(a[1], expected_v3_frame(4.0, 5.0, 6.0, 1, 300));
    }

    /// The NOT-FULLY-CONSUMED refusal must ride the
    /// SHARED per-route failure path — same `RawRouteError::Decode` arm,
    /// same flood latch, same counters — and must not create a per-frame log
    /// flood on a route whose local schema is wrong (which is a SUSTAINED
    /// condition: every single sample of a mis-defined type trips it, at the
    /// topic's full publish rate).
    ///
    /// Scope: the level MAPPING (loud first / debug repeats / recovery
    /// `info!`) is `FloodLatch`, oracle-tested in `crate::latch`; this pins
    /// that this error class REACHES it through the same arm as every
    /// other decode failure — i.e. with no plumbing of its own — plus the
    /// counter/sequence discipline, which is assertable without log capture.
    #[test]
    fn a_trailing_bytes_refusal_rides_the_route_failure_latch_and_burns_no_sequence() {
        use crate::generic::RawRouteError;
        use cerulion_core::codegen::CdrCodecError;
        use cerulion_core::wire::MaxSliceLen;

        let mgr = raw_test_transport("raw");
        let codec = crate::generic::bridge_codec();
        let topic = "/pump904/trailing/v3";
        let mut route = RawIngressRoute::open(
            &mgr,
            &codec,
            "geometry_msgs/Vector3",
            topic,
            MaxSliceLen::const_new(4096),
        )
        .expect("raw route");
        let sub = mgr.create_subscriber(topic).expect("observer");

        /// A well-formed Vector3 body with 4 unaccounted trailing bytes —
        /// what a local definition SHORTER than the writer's produces.
        fn v3_with_trailing(x: f64, y: f64, z: f64) -> RawSample {
            let mut s = raw_v3_sample(x, y, z);
            s.body.extend_from_slice(&[0xAA, 0xBB, 0xCC, 0xDD]);
            s
        }

        // (a) CLASS PIN at the route seam: the refusal surfaces as the SAME
        // `Decode` variant the pump's flood-latched arm already handles,
        // carrying the `TrailingBytes` cause with its exact split.
        let bad = v3_with_trailing(1.0, 2.0, 3.0);
        let err = route
            .publish_cdr_body(&codec, bad.endianness, &bad.body, 10)
            .expect_err("4 unconsumed bytes must be refused, not published");
        match &err {
            RawRouteError::Decode { source, .. } => assert!(
                matches!(
                    **source,
                    CdrCodecError::TrailingBytes {
                        consumed: 24,
                        remaining: 4,
                        ..
                    }
                ),
                "expected TrailingBytes(24, 4), got {source:?}"
            ),
            other => panic!("expected RawRouteError::Decode, got {other:?}"),
        }

        // (b) The pump loop over a SUSTAINED wrong-schema regime with a good
        // sample on each side. Counters are unconditional (one per sample,
        // never suppressed with the log), and the failures burn no sequence.
        let items: Vec<Result<RawSample, &'static str>> = vec![
            Ok(raw_v3_sample(1.0, 2.0, 3.0)),
            Ok(v3_with_trailing(9.0, 9.0, 9.0)),
            Ok(v3_with_trailing(9.0, 9.0, 9.0)),
            Ok(v3_with_trailing(9.0, 9.0, 9.0)),
            Ok(raw_v3_sample(4.0, 5.0, 6.0)),
        ];
        let stream = futures::stream::iter(items);
        futures::pin_mut!(stream);
        let t = std::cell::Cell::new(0u64);
        let now = || {
            t.set(t.get() + 100);
            t.get()
        };
        let stats = PumpStats::default();
        let route = smol::block_on(drain_raw_from_stream(
            stream, "/raw_dds", route, &codec, &now, &stats,
        ));

        assert_eq!(stats.raw_published_total.load(Ordering::Relaxed), 2);
        assert_eq!(stats.raw_route_failures_total.load(Ordering::Relaxed), 3);
        assert_eq!(stats.raw_stream_errors_total.load(Ordering::Relaxed), 0);
        // 3 from the loop + 1 from the direct (a) call above.
        assert_eq!(route.decode_failures_total(), 4);
        assert_eq!(route.published_total(), 2);
        assert_eq!(
            route.next_sequence(),
            2,
            "a refused sample must not burn a wire sequence (the commit-time rule)"
        );

        // Delivery oracle: only the two well-formed samples crossed, gap-free
        // across the refused regime. Stamps 100 and 500 — the three refused
        // attempts consumed 200/300/400.
        let frames = captured_frames(&sub);
        assert_eq!(frames.len(), 2);
        assert_eq!(frames[0], expected_v3_frame(1.0, 2.0, 3.0, 0, 100));
        assert_eq!(frames[1], expected_v3_frame(4.0, 5.0, 6.0, 1, 500));
    }

    #[test]
    fn stream_err_level_latches_warn_then_debug() {
        // The warn→debug half of the err-streak latch, purely (the recovery
        // info! fires when a later Ok resets the streak — pinned above by the
        // 2-push recovery run).
        assert_eq!(stream_err_level(1), StreamErrLevel::Warn);
        for streak in 2..50u64 {
            assert_eq!(stream_err_level(streak), StreamErrLevel::Debug);
        }
    }

    // ---- The init backoff gate --------------------------------------------

    #[test]
    fn init_gate_respects_the_backoff_deadline() {
        let base = Instant::now();
        let at = base + INIT_BACKOFF;
        assert_eq!(
            init_gate(None, base),
            InitGate::Attempt,
            "no prior failure → attempt"
        );
        assert_eq!(
            init_gate(Some(at), base),
            InitGate::Wait,
            "before the deadline → wait"
        );
        assert_eq!(
            init_gate(Some(at), at),
            InitGate::Attempt,
            "at the deadline → attempt"
        );
        assert_eq!(
            init_gate(Some(at), at + Duration::from_millis(1)),
            InitGate::Attempt,
            "after the deadline → attempt"
        );
    }

    #[test]
    fn iterate_does_not_spawn_before_the_backoff_deadline() {
        // Hermetic: `now < next_init_at` returns at the gate BEFORE any spawn.
        let mut pump = test_pump();
        let base = Instant::now();
        pump.next_init_at = Some(base + INIT_BACKOFF);
        let out = pump.iterate(base);
        assert!(
            pump.drain.is_none(),
            "no drain thread spawned before the deadline"
        );
        assert_eq!(
            pump.stats.init_attempts_total.load(Ordering::Relaxed),
            0,
            "no init attempt counted before the deadline"
        );
        assert!(!out.ring);
        assert_eq!(out.pace, Some(BACKOFF_SLICE));
    }

    // ---- Iterate phase-arm transitions ------------------------------------

    #[test]
    fn iterate_up_arm_rings_exactly_on_pushed_swap() {
        let mut pump = test_pump();
        let status = Arc::new(StartupStatus::new());
        status.set_up();
        pump.drain = Some(handle(true, status));
        // pushed == true → rings once, then quiet (swap semantics). A mutation
        // making `ring` unconditionally true fails the second assert.
        let out = pump.iterate(Instant::now());
        assert!(out.ring, "rings when the drain pushed");
        assert_eq!(out.pace, Some(POLL_PACE));
        let out2 = pump.iterate(Instant::now());
        assert!(!out2.ring, "quiet after the swap consumed the push");
    }

    #[test]
    fn iterate_pending_arm_waits_retaining_the_drain() {
        let mut pump = test_pump();
        // Fresh status = PHASE_PENDING.
        pump.drain = Some(handle(false, Arc::new(StartupStatus::new())));
        let out = pump.iterate(Instant::now());
        assert!(
            pump.drain.is_some(),
            "PENDING retains the drain (init still running)"
        );
        assert!(!out.ring);
        assert_eq!(out.pace, Some(BACKOFF_SLICE));
    }

    #[test]
    fn iterate_failed_arm_resets_for_respawn_and_counts() {
        let mut pump = test_pump();
        let status = Arc::new(StartupStatus::new());
        status.set_failed("init boom".to_string());
        pump.drain = Some(handle(false, status));
        let before = pump.stats.init_failures_total.load(Ordering::Relaxed);
        let out = pump.iterate(Instant::now());
        assert!(
            pump.drain.is_none(),
            "FAILED clears the drain so the next iterate respawns"
        );
        assert!(pump.next_init_at.is_some(), "backoff deadline armed");
        assert_eq!(
            pump.stats.init_failures_total.load(Ordering::Relaxed),
            before + 1
        );
        assert!(!out.ring);
        assert_eq!(out.pace, Some(BACKOFF_SLICE));
    }

    // ---- DEAD arm + panic containment -------------------------------------

    #[test]
    fn iterate_dead_arm_is_terminal_loud_and_never_rings() {
        let mut pump = test_pump();
        let status = Arc::new(StartupStatus::new());
        status.set_dead("drain task panicked: boom".to_string());
        // pushed == true is the anti-tautology: a mutation routing DEAD to the
        // UP arm would ring here.
        pump.drain = Some(handle(true, status));
        assert!(pump.is_dead(), "phase DEAD surfaces via is_dead");
        let out = pump.iterate(Instant::now());
        assert!(!out.ring, "a DEAD pump NEVER rings (no fabricated wake)");
        assert!(
            pump.drain.is_some(),
            "DEAD is terminal — the drain is retained, never respawned (dead-until-relaunch)"
        );
        assert!(pump.is_dead());
        // Repeats stay dead (flood-latched), no state churn, no panic.
        let out2 = pump.iterate(Instant::now());
        assert!(!out2.ring);
        assert!(pump.is_dead());
    }

    #[test]
    fn contained_executor_marks_dead_on_driver_panic() {
        // The seam catches ANY panic unwinding out of block_on(ex.run(driver))
        // — here from the driver future (guaranteed to unwind); in production
        // the panic comes from a detached drain task, which async-executor
        // propagates through `ex.run` to the same catch site.
        let status = Arc::new(StartupStatus::new());
        status.set_up();
        let ex = smol::LocalExecutor::new();
        run_drain_executor_contained(
            ex,
            async {
                panic!("injected drain-task panic");
            },
            &status,
        );
        assert_eq!(
            status.phase.load(Ordering::Acquire),
            PHASE_DEAD,
            "a task panic marks the thread DEAD, not silently gone"
        );
        assert!(
            status.take_error().contains("injected drain-task panic"),
            "the panic payload is captured for the operator"
        );
    }

    #[test]
    fn contained_executor_stays_up_without_a_panic() {
        // The anti-tautology control: a driver that completes normally leaves
        // the phase UP (the seam does not spuriously mark DEAD).
        let status = Arc::new(StartupStatus::new());
        status.set_up();
        let ex = smol::LocalExecutor::new();
        run_drain_executor_contained(ex, std::future::ready(()), &status);
        assert_eq!(status.phase.load(Ordering::Acquire), PHASE_UP);
    }

    #[test]
    fn startup_status_set_dead_stores_phase_and_error() {
        let s = StartupStatus::new();
        s.set_dead("kaboom".to_string());
        assert_eq!(s.phase.load(Ordering::Acquire), PHASE_DEAD);
        assert_eq!(s.take_error(), "kaboom");
    }

    // ---- Init-phase panic containment (PENDING → DEAD) --------------------

    #[test]
    fn init_phase_panic_marks_dead_not_stuck_pending() {
        // Without the whole-body boundary, a panic before the executor (the DDS
        // init phase — third-party rustdds socket/multicast/interface setup)
        // kills the drain thread with the status stuck PHASE_PENDING
        // forever (every iterate: "init still running", zero ingress, zero
        // operator signal). The whole-body boundary must transition it to
        // the terminal DEAD instead.
        let status = Arc::new(StartupStatus::new());
        assert_eq!(status.phase.load(Ordering::Acquire), PHASE_PENDING);
        run_drain_thread_contained(&status, || panic!("injected init-phase panic"));
        assert_eq!(
            status.phase.load(Ordering::Acquire),
            PHASE_DEAD,
            "an init-phase panic must surface as DEAD — never stuck PENDING"
        );
        // iterate surfaces it: no ring, terminal (no respawn), is_dead true.
        let mut pump = test_pump();
        pump.drain = Some(handle(true, status));
        assert!(pump.is_dead());
        let out = pump.iterate(Instant::now());
        assert!(!out.ring, "a DEAD-from-init pump never rings");
        assert!(
            pump.drain.is_some(),
            "terminal — never respawned (dead-until-relaunch)"
        );
        assert!(pump.is_dead());
    }

    #[test]
    fn contained_drain_body_control_stays_on_the_normal_path() {
        // Anti-tautology controls: the boundary never fabricates DEAD.
        // (a) A body that reports UP (a successful init) stays UP.
        let up = Arc::new(StartupStatus::new());
        let up_inner = Arc::clone(&up);
        run_drain_thread_contained(&up, move || up_inner.set_up());
        assert_eq!(up.phase.load(Ordering::Acquire), PHASE_UP);
        // (b) A body that reports a non-panic init FAILURE (the Err path)
        // stays FAILED — the boundary never overwrites the retryable outcome
        // with the terminal DEAD.
        let failed = Arc::new(StartupStatus::new());
        let failed_inner = Arc::clone(&failed);
        run_drain_thread_contained(&failed, move || {
            failed_inner.set_failed("init err (retryable)".to_string())
        });
        assert_eq!(failed.phase.load(Ordering::Acquire), PHASE_FAILED);
        assert_eq!(failed.take_error(), "init err (retryable)");
    }

    #[test]
    fn drain_thread_main_contains_an_init_panic_end_to_end() {
        // The WIRING pin: drive the REAL thread entry point (drain_thread_main
        // — exactly what spawn_drain_thread spawns) with the init-panic seam
        // armed (fires at drain_thread_body entry, BEFORE any DDS object
        // exists — zero DDS in this test). Without containment this fails BOTH
        // ways: the uncontained panic unwinds across join() (Err) AND the status
        // stays PENDING; with it the thread exits cleanly with status DEAD +
        // payload captured.
        INIT_PANIC_FOR_TEST.store(true, Ordering::SeqCst);
        let status = Arc::new(StartupStatus::new());
        let queue: Arc<Mutex<SampleQueue>> = Arc::new(Mutex::new(SampleQueue::default()));
        let pushed = Arc::new(AtomicBool::new(false));
        let stats = Arc::new(PumpStats::default());
        let st = Arc::clone(&status);
        // The init panic fires before the executor, so `stop_rx` is
        // never awaited; `finished_tx` still fires after the contained body
        // returns. We keep `finished_rx` so the channel isn't disconnected.
        let (_stop_tx, stop_rx) = futures::channel::oneshot::channel::<()>();
        let (finished_tx, finished_rx) = std::sync::mpsc::channel::<()>();
        let h = std::thread::Builder::new()
            .name("dds-bridge-drain-test".to_string())
            .spawn(move || {
                drain_thread_main(
                    test_config(),
                    None,
                    queue,
                    pushed,
                    stats,
                    st,
                    stop_rx,
                    finished_tx,
                )
            })
            .expect("spawn test drain thread");
        // The thread announces `finished` after the contained init-panic body
        // returns — proving the finished-signal path runs even on the DEAD arm.
        finished_rx
            .recv_timeout(Duration::from_secs(5))
            .expect("drain thread signals finished after the contained init panic");
        h.join().expect(
            "the init panic must be CONTAINED — the thread exits cleanly, never \
             unwinding across join (the silent-death shape)",
        );
        assert_eq!(
            status.phase.load(Ordering::Acquire),
            PHASE_DEAD,
            "the real entry point must mark DEAD on an init panic (not stuck PENDING)"
        );
        assert!(
            status.take_error().contains("injected DDS-init panic"),
            "the panic payload is captured for the operator"
        );
        assert!(
            !INIT_PANIC_FOR_TEST.load(Ordering::SeqCst),
            "the seam self-disarms (swap) — no cross-test leakage"
        );
    }

    // ---- Shutdown coordination (hermetic — no DDS) ----------------

    /// A SYNTHETIC drain-thread stop handle whose thread mimics the drain
    /// thread's stop contract WITHOUT DDS: it blocks on `stop_rx`, does a small
    /// settle (modelling the participant-drop dispose work), then signals
    /// `finished`. Lets the [`PumpShutdown`]/[`DrainThreadStop`] join logic be
    /// pinned deterministically with zero rustdds.
    fn synthetic_stop(settle: Duration) -> DrainThreadStop {
        let (stop_tx, stop_rx) = futures::channel::oneshot::channel::<()>();
        let (finished_tx, finished_rx) = std::sync::mpsc::channel::<()>();
        let join = std::thread::spawn(move || {
            // Resolves when shutdown drops `stop_tx` (the production signal).
            let _ = smol::block_on(stop_rx);
            std::thread::sleep(settle);
            let _ = finished_tx.send(());
        });
        DrainThreadStop {
            stop_tx,
            finished_rx,
            join,
        }
    }

    #[test]
    fn shutdown_signals_and_joins_within_deadline_and_is_idempotent() {
        let ps = PumpShutdown::new();
        *ps.current.lock().unwrap_or_else(|p| p.into_inner()) =
            Some(synthetic_stop(Duration::from_millis(20)));

        let t0 = Instant::now();
        let outcome = ps.shutdown(Duration::from_secs(5));
        let elapsed = t0.elapsed();
        assert_eq!(
            outcome,
            ShutdownOutcome::Joined,
            "the stop signal resolves the thread; it is joined"
        );
        assert!(
            elapsed < Duration::from_secs(5),
            "joined WELL within the deadline (settle ~20ms): {elapsed:?}"
        );
        assert!(ps.is_stopping(), "shutdown latches the stopping flag");
        // Idempotent: the stop handle was consumed by the first call — a second
        // is an AlreadyShutDown no-op, never a double-join panic.
        assert_eq!(
            ps.shutdown(Duration::from_secs(5)),
            ShutdownOutcome::AlreadyShutDown
        );
    }

    #[test]
    fn shutdown_with_no_registered_thread_is_nothing_to_stop() {
        // The config-error / never-iterated path: no drain thread was ever
        // registered, so there is nothing to stop (but stopping still latches).
        let ps = PumpShutdown::new();
        assert_eq!(
            ps.shutdown(Duration::from_secs(5)),
            ShutdownOutcome::NothingToStop
        );
        assert!(ps.is_stopping());
        assert_eq!(
            ps.shutdown(Duration::from_secs(5)),
            ShutdownOutcome::AlreadyShutDown
        );
    }

    #[test]
    fn shutdown_detaches_a_wedged_thread_at_the_deadline() {
        // A wedged drain thread NEVER signals `finished`; shutdown must WAIT the
        // bounded deadline, then DETACH (never block teardown forever). Held
        // `finished_tx` keeps the channel connected so recv_timeout genuinely
        // times out (a disconnect would report Joined).
        let ps = PumpShutdown::new();
        let (stop_tx, _stop_rx_keepalive) = futures::channel::oneshot::channel::<()>();
        let (finished_tx_keepalive, finished_rx) = std::sync::mpsc::channel::<()>();
        let join = std::thread::spawn(|| std::thread::sleep(Duration::from_millis(50)));
        *ps.current.lock().unwrap_or_else(|p| p.into_inner()) = Some(DrainThreadStop {
            stop_tx,
            finished_rx,
            join,
        });

        let deadline = Duration::from_millis(120);
        let t0 = Instant::now();
        let outcome = ps.shutdown(deadline);
        let elapsed = t0.elapsed();
        assert_eq!(
            outcome,
            ShutdownOutcome::TimedOut,
            "a thread that never signals finished is detached, not joined"
        );
        assert!(
            elapsed >= deadline,
            "waited the full deadline before detaching: {elapsed:?}"
        );
        assert!(
            elapsed < deadline + Duration::from_secs(2),
            "did not block far past the deadline: {elapsed:?}"
        );
        // Keep the sender alive until AFTER the timed-out recv (anti-tautology:
        // a dropped sender would have made this Joined, not TimedOut).
        drop(finished_tx_keepalive);
    }

    #[test]
    fn post_shutdown_iterate_is_a_graceful_noop_never_respawns() {
        // The host's detached Blocking helper may keep calling
        // the closure (→ iterate) AFTER shutdown. Every such call must no-op —
        // NEVER spawn a fresh drain thread / participant, never panic.
        let mut pump = test_pump();
        assert_eq!(
            pump.shutdown(Duration::from_secs(5)),
            ShutdownOutcome::NothingToStop,
            "no drain was ever spawned (never iterated), so nothing to stop"
        );
        assert!(pump.shutdown_coord.is_stopping());
        for _ in 0..3 {
            let out = pump.iterate(Instant::now());
            assert!(
                !out.ring,
                "post-shutdown iterate NEVER rings (no fabricated wake)"
            );
            assert_eq!(out.pace, Some(BACKOFF_SLICE));
        }
        assert!(
            pump.drain.is_none(),
            "post-shutdown iterate NEVER spawns a drain thread"
        );
        assert_eq!(
            pump.stats.init_attempts_total.load(Ordering::Relaxed),
            0,
            "no spawn attempt counted after shutdown latched"
        );
    }

    #[test]
    fn should_spawn_under_lock_refuses_when_stopping() {
        // Spawn-race guard: shutdown() latches `stopping` + takes the
        // handle under `current`'s lock, so a spawn that already grabbed the lock
        // must re-read `stopping` and ABORT — otherwise it orphans a fresh
        // participant past teardown. The lock-free top-of-iterate check
        // short-circuits before the site is single-threaded-reachable, so pin the
        // decision directly here (deleting the re-check would otherwise pass the
        // suite). Reverting `should_spawn_under_lock` to always-spawn
        // fails exactly this assertion.
        assert!(
            !should_spawn_under_lock(true),
            "a spawn racing a shutdown that already latched `stopping` must ABORT — no orphan"
        );
        assert!(
            should_spawn_under_lock(false),
            "the normal path (not stopping) proceeds to spawn"
        );
    }
}

// ===========================================================================
// Per-route slice sizing: the production call site.
//
// `build_raw_binding` is where a route's slice is chosen, and it cannot be
// driven from a unit test: it needs a live DDS `Node` + subscriber to create a
// topic and a reader before it ever reaches `RawIngressRoute::open`. The pump's
// other raw tests construct a `RawIngressRoute` directly and so pass straight
// through this decision.
//
// That gap is real. With the call site reverted to
// `BridgeConfig::raw_slice_len(m)` (bridge-wide sizing, i.e. the per-route
// rule wired to nothing), every OTHER test in the dds_bridge suite stays
// green. The six slice-sizing oracles in `config.rs` prove the RULE; only this
// pin proves production USES it.
//
// So the pin is structural, over this file's own comment-stripped source, the
// standard shape for a call site no unit test can reach.
// ===========================================================================
#[cfg(test)]
mod call_site {
    /// Strip `//`-to-end-of-line and `/* */` comments, tracking nesting because
    /// Rust's block comments nest.
    ///
    /// Load-bearing, not hygiene: the comment block directly above this module
    /// names BOTH `raw_slice_len_for_route` and `raw_slice_len(m)` in prose, so
    /// a scan over the raw text would be satisfied by the prose and would pass
    /// even if the call site stopped calling it, which is what this module
    /// exists to catch.
    ///
    /// String literals are deliberately not modelled; this file has no literal
    /// containing a comment marker, and the anti-tautology arm below fails if
    /// the stripper ever eats real code.
    fn code_only(src: &str) -> String {
        let b = src.as_bytes();
        let mut out = String::with_capacity(src.len());
        let mut i = 0usize;
        let mut depth = 0usize;
        while i < b.len() {
            if depth > 0 {
                if b[i] == b'/' && i + 1 < b.len() && b[i + 1] == b'*' {
                    depth += 1;
                    i += 2;
                } else if b[i] == b'*' && i + 1 < b.len() && b[i + 1] == b'/' {
                    depth -= 1;
                    i += 2;
                } else {
                    i += 1;
                }
                continue;
            }
            if b[i] == b'/' && i + 1 < b.len() && b[i + 1] == b'*' {
                depth += 1;
                i += 2;
                continue;
            }
            if b[i] == b'/' && i + 1 < b.len() && b[i + 1] == b'/' {
                while i < b.len() && b[i] != b'\n' {
                    i += 1;
                }
                continue;
            }
            out.push(b[i] as char);
            i += 1;
        }
        out
    }

    /// The body of `fn <name>`, from its opening brace to the matching close.
    fn fn_body(code: &str, name: &str) -> String {
        let at = code
            .find(&format!("fn {name}"))
            .unwrap_or_else(|| panic!("`fn {name}` not found in the stripped source"));
        let open = code[at..]
            .find('{')
            .unwrap_or_else(|| panic!("`fn {name}` has no body"))
            + at;
        let mut depth = 0usize;
        for (off, c) in code[open..].char_indices() {
            match c {
                '{' => depth += 1,
                '}' => {
                    depth -= 1;
                    if depth == 0 {
                        return code[open..open + off + 1].to_string();
                    }
                }
                _ => {}
            }
        }
        panic!("`fn {name}` body is unbalanced");
    }

    fn stripped_source() -> String {
        code_only(include_str!("pump.rs"))
    }

    /// THE pin: the route builder sizes its slice with the SCHEMA-AWARE rule.
    ///
    /// Both halves are needed. Requiring `raw_slice_len_for_route` alone would
    /// pass a builder that computed it and then handed `RawIngressRoute::open`
    /// the un-narrowed value; forbidding the bare call alone would pass one that
    /// sized nothing at all.
    #[test]
    fn the_raw_route_builder_sizes_its_slice_per_route() {
        let body = fn_body(&stripped_source(), "build_raw_binding");
        assert!(
            body.contains("raw_slice_len_for_route("),
            "build_raw_binding must derive its slice with \
             BridgeConfig::raw_slice_len_for_route — the CONFIGURED-only \
             `raw_slice_len` leaves every `ros2 attach` route at the bridge-wide \
             1 MiB default and therefore at depth 64. Body was:\n{body}"
        );
        // The bare call, spelled as the mutation writes it: `raw_slice_len(`
        // NOT followed by the `_for_route` suffix.
        let bare = body
            .match_indices("raw_slice_len(")
            .filter(|(at, _)| {
                !body[..*at].ends_with("raw_slice_len_for_route".trim_end_matches("("))
            })
            .count();
        assert_eq!(
            bare, 0,
            "build_raw_binding must not call the CONFIGURED-only \
             BridgeConfig::raw_slice_len — that is bridge-wide sizing, not per-route. \
             Body was:\n{body}"
        );
    }

    /// Anti-tautology: the stripped view still contains the code it describes.
    ///
    /// Without this, a `code_only` that returned an empty string (or ate real
    /// code) would make the "must not contain" half of the pin above vacuous.
    #[test]
    fn the_stripped_view_still_contains_real_code() {
        let code = stripped_source();
        assert!(
            code.contains("fn build_raw_binding"),
            "the stripper ate the function under test"
        );
        assert!(
            code.contains("RawIngressRoute::open("),
            "the stripper ate the call the slice is handed to"
        );
        // ...and it really does strip. The needle is ASSEMBLED rather than
        // written out, because a verbatim literal would itself appear in this
        // file as CODE and survive stripping, failing this assertion against a
        // working stripper. The phrase lives in the comment block above the module.
        let comment_only = format!("{}{}", "wired to ", "nothing");
        assert!(
            !code.contains(&comment_only),
            "the stripper left comment prose in the view — every 'must not \
             contain' assertion in this module is then satisfied by prose"
        );
    }

    /// The stripper handles both syntaxes, including Rust's NESTING block
    /// comments, and nothing else.
    #[test]
    fn code_only_strips_both_comment_syntaxes() {
        assert_eq!(
            code_only("let a = 1; // tail\nlet b = 2;\n"),
            "let a = 1; \nlet b = 2;\n"
        );
        assert_eq!(code_only("a /* mid */ b"), "a  b");
        assert_eq!(code_only("a /* outer /* inner */ still */ b"), "a  b");
        assert_eq!(code_only("a // /* not a block\nb"), "a \nb");
        // Unterminated block: everything after it is consumed (fails CLOSED —
        // the "must contain" assertions above go red rather than silently
        // passing).
        assert_eq!(code_only("a /* forever"), "a ");
    }
}
