// SPDX-License-Identifier: AGPL-3.0-only
//! The EVENT-DRIVEN catalog-change plane — netd learns the LAN's topic
//! catalog changed the moment it changes, and PUSHES that fact to the desk consumers
//! that asked for it, so nothing anywhere has to poll for a refresh.
//!
//! # Why this exists
//!
//! Without it, when a robot's serve is swapped (a small demo graph → a full
//! `ros2 attach` graph, ~75 topics) the desk sidebar keeps rendering the OLD
//! six-topic view until somebody clicks refresh. The catalog has changed; nothing
//! tells the desk.
//!
//! Nothing NEEDED to poll, either — the events were already on the wire. A robot's
//! produced topics are advertised as zenoh LIVELINESS tokens under `cerulion_ann/`
//! by the serving gateway, and a liveliness SUBSCRIBER is told the instant a token
//! appears or its declaring session dies. netd already owns the machine's ONE zenoh
//! session (Principle #8), so the whole feature is: subscribe on that session, fold
//! the transitions into a view, and push a change notification down the UDS control
//! connections that subscribed.
//!
//! # Shape
//!
//! - [`AnnounceView`] is the incrementally-maintained picture of who is announcing
//!   what. PURE — it takes [`AnnounceEvent`]s and reports [`CatalogDelta`]s.
//! - [`ChangeCoalescer`] batches deltas over [`CHANGE_COALESCE_WINDOW`] so an attach
//!   graph's ~75-topic announce BURST becomes ONE push, not 75.
//! - [`CatalogEventHub`] holds the per-connection push slots and the announce-watch
//!   thread. A push is a MUTEX STORE and nothing else — the watch thread never does
//!   I/O, so a stalled consumer can never stall netd or a healthy sibling.
//! - [`AnnounceWatchPlane`] is the DI seam (production [`GatewayAnnounceWatchPlane`]
//!   over netd's shared session, [`NoopAnnounceWatchPlane`] for a mirror-only daemon,
//!   a spy in the tests) — the same pattern as the mirror / egress / query planes.
//!
//! # What this does NOT cover
//!
//! **Only the REMOTE half is event-driven.** A LOCAL iceoryx2 service appearing or
//! disappearing on the desk carries NO liveliness token — there is no event to
//! subscribe to — so a locally-produced topic's arrival is still discovered by
//! whatever refresh the consumer already does (including the manual one, which stays
//! as the escape hatch for everything). That is not a gap in this design so much as
//! the shape of the two planes: the stale-sidebar case is entirely the remote
//! half (a ROBOT's attach graph coming up), and inventing a local watcher — a polling
//! thread over `list_topics`, which is exactly the polling this plane removes — would
//! cost more than it buys.

use std::collections::{BTreeMap, BTreeSet};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, MutexGuard, PoisonError};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use cerulion_core::transport::discovery::AnnounceEvent;

use crate::protocol::CatalogChanged;
use crate::registry::ConnId;

/// How long announce transitions COALESCE into one push.
///
/// The sizing case is a burst, not a trickle: a `ros2 attach` graph coming up declares
/// one announce token per bridged topic (~75 on a Go2-class robot) back to back on
/// one session, so without batching a single operator action would fan out into ~75
/// pushes and ~75 sidebar refreshes.
///
/// 250 ms is chosen because it is (a) comfortably longer than such a burst — the
/// declares are local session operations landing in single-digit milliseconds — and
/// (b) short enough that the result still reads as instant: the worst-case
/// notification latency is this window plus one control-connection poll interval
/// (netd's 200 ms `CONN_READ_TIMEOUT`), i.e. under half a second, against a manual
/// refresh that might never come at all.
///
/// It also BOUNDS the push rate: the window starts at the FIRST change after a push
/// (not on the LAST change, which a continuous trickle could extend forever), so a
/// pathologically flapping robot costs at most four pushes per second per subscriber
/// and every change still arrives within one window of happening.
pub const CHANGE_COALESCE_WINDOW: Duration = Duration::from_millis(250);

/// How long the announce-watch thread blocks for the next transition before looping.
/// Only bounds SHUTDOWN responsiveness and the coalescer's flush check when nothing
/// is pending — an actual transition wakes the subscriber immediately.
const WATCH_IDLE_TIMEOUT: Duration = Duration::from_millis(500);

/// What changed in the LAN catalog, at the granularity a consumer needs to decide
/// how much to redraw.
///
/// The robot sets are NET over the coalescing window (a robot that appeared and then
/// vanished inside one window cancels out — reporting both would send a consumer
/// chasing a robot that is not there). The topic counts are CHURN, not net: they say
/// how much moved, which is what distinguishes "one topic came up" from "a whole
/// attach graph came up" in a log or a status line. Deliberately no topic NAME lists:
/// the consumer's response to any of this is to refresh, which is where names come
/// from, and keeping the notification small is what keeps the push slot cheap.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct CatalogDelta {
    /// Robots that became present (a first token of any kind).
    pub robots_added: BTreeSet<String>,
    /// Robots that went away entirely (every token they held is gone — the shape a
    /// dying gateway produces, since one session death drops them all at once).
    pub robots_removed: BTreeSet<String>,
    /// How many topic tokens appeared over the window (churn, not net).
    pub topics_added: u64,
    /// How many topic tokens went away over the window (churn, not net).
    pub topics_removed: u64,
}

impl CatalogDelta {
    /// Nothing moved.
    pub fn is_empty(&self) -> bool {
        self.robots_added.is_empty()
            && self.robots_removed.is_empty()
            && self.topics_added == 0
            && self.topics_removed == 0
    }

    /// Fold `other` (a LATER delta) into `self`.
    ///
    /// Robot membership is resolved to the NET effect — a later add cancels an
    /// earlier remove and vice versa — so the merged delta describes where the robot
    /// ENDED UP, never a contradiction. Topic counts SUM (they are churn).
    pub fn merge(&mut self, other: CatalogDelta) {
        for robot in other.robots_added {
            self.robots_removed.remove(&robot);
            self.robots_added.insert(robot);
        }
        for robot in other.robots_removed {
            self.robots_added.remove(&robot);
            self.robots_removed.insert(robot);
        }
        self.topics_added = self.topics_added.saturating_add(other.topics_added);
        self.topics_removed = self.topics_removed.saturating_add(other.topics_removed);
    }
}

/// One announcing robot's tokens: whether its BARE identity token is live, and which
/// canonical topics it currently announces. A robot is PRESENT while it holds either.
#[derive(Debug, Default, Clone, PartialEq, Eq)]
struct RobotEntry {
    /// The bare `cerulion_ann/{robot}` identity token (a gateway declares it at boot,
    /// so an ingress-only robot with zero produced topics is still present).
    identity: bool,
    /// The canonical topics this robot announces.
    topics: BTreeSet<String>,
}

impl RobotEntry {
    fn is_present(&self) -> bool {
        self.identity || !self.topics.is_empty()
    }
}

/// The incrementally-maintained picture of the announce space.
///
/// PURE: it consumes [`AnnounceEvent`]s and reports what changed. Feeding it the same
/// event twice is a NO-OP (liveliness can legitimately re-deliver a token — the
/// history replay at watch start is exactly that), so a re-delivery never manufactures
/// a phantom change and never fires a push.
#[derive(Debug, Default)]
pub struct AnnounceView {
    robots: BTreeMap<String, RobotEntry>,
}

impl AnnounceView {
    /// A fresh, empty view.
    pub fn new() -> Self {
        Self::default()
    }

    /// Fold one announce transition in. Returns the delta it produced, or `None` when
    /// the event told us nothing new.
    pub fn apply(&mut self, event: &AnnounceEvent) -> Option<CatalogDelta> {
        let robot = event.robot().to_string();
        let mut delta = CatalogDelta::default();
        let entry = self.robots.entry(robot.clone()).or_default();
        let was_present = entry.is_present();

        let moved = match event {
            AnnounceEvent::Alive { topic, .. } => match topic {
                None => !std::mem::replace(&mut entry.identity, true),
                Some(topic) => entry.topics.insert(topic.clone()),
            },
            AnnounceEvent::Lost { topic, .. } => match topic {
                None => std::mem::replace(&mut entry.identity, false),
                Some(topic) => entry.topics.remove(topic),
            },
        };
        if !moved {
            // Nothing changed. Do not leave a phantom empty entry behind for a robot
            // we have never actually seen (a `Lost` for an unknown robot inserts one).
            if !was_present && !entry.is_present() {
                self.robots.remove(&robot);
            }
            return None;
        }
        if let AnnounceEvent::Alive { topic: Some(_), .. } = event {
            delta.topics_added += 1;
        }
        if let AnnounceEvent::Lost { topic: Some(_), .. } = event {
            delta.topics_removed += 1;
        }

        let is_present = entry.is_present();
        if !is_present {
            self.robots.remove(&robot);
        }
        match (was_present, is_present) {
            (false, true) => {
                delta.robots_added.insert(robot);
            }
            (true, false) => {
                delta.robots_removed.insert(robot);
            }
            _ => {}
        }
        Some(delta)
    }

    /// The robots currently announcing, sorted (deterministic presentation).
    pub fn robots(&self) -> Vec<String> {
        self.robots.keys().cloned().collect()
    }

    /// How many topics `robot` currently announces (diagnostics / tests).
    pub fn topic_count(&self, robot: &str) -> usize {
        self.robots.get(robot).map_or(0, |e| e.topics.len())
    }
}

/// Batches [`CatalogDelta`]s so a burst of transitions becomes ONE push.
///
/// The window opens at the FIRST change after a flush and is NOT extended by later
/// ones — a quiet-window (reset-on-every-change) debounce would let a continuously
/// churning robot defer its notification indefinitely, which is the one failure mode
/// worse than a slightly chatty one. See [`CHANGE_COALESCE_WINDOW`].
#[derive(Debug)]
pub struct ChangeCoalescer {
    window: Duration,
    pending: Option<PendingBatch>,
}

#[derive(Debug)]
struct PendingBatch {
    /// When the FIRST un-flushed change landed — the window's anchor.
    opened_at: Instant,
    /// How many `note` calls this batch folds (>= 1) — the true "this push stands
    /// for N observed transitions" number the notification carries.
    batches: u64,
    delta: CatalogDelta,
}

impl ChangeCoalescer {
    /// A coalescer over an explicit window (production passes
    /// [`CHANGE_COALESCE_WINDOW`]; tests pass a short one so an arm is not a sleep).
    pub fn new(window: Duration) -> Self {
        Self {
            window,
            pending: None,
        }
    }

    /// Fold a delta in. An EMPTY delta is ignored — it opens no window, so a stream of
    /// no-op events never manufactures a push.
    pub fn note(&mut self, delta: CatalogDelta, now: Instant) {
        if delta.is_empty() {
            return;
        }
        match &mut self.pending {
            Some(batch) => {
                batch.delta.merge(delta);
                batch.batches += 1;
            }
            None => {
                self.pending = Some(PendingBatch {
                    opened_at: now,
                    batches: 1,
                    delta,
                });
            }
        }
    }

    /// When the pending batch becomes due (`None` when nothing is pending).
    pub fn due_at(&self) -> Option<Instant> {
        self.pending.as_ref().map(|b| b.opened_at + self.window)
    }

    /// Take the batch if its window has elapsed. Returns `(delta, batches_folded)`.
    pub fn take_if_due(&mut self, now: Instant) -> Option<(CatalogDelta, u64)> {
        let due = self.due_at()?;
        if now < due {
            return None;
        }
        self.pending.take().map(|b| (b.delta, b.batches))
    }

    /// How long to block before the next flush check: the time remaining on a pending
    /// batch, or `idle` when nothing is pending. Never zero (a zero-timeout receive
    /// would spin).
    pub fn block_for(&self, now: Instant, idle: Duration) -> Duration {
        match self.due_at() {
            None => idle,
            Some(due) => due
                .saturating_duration_since(now)
                .max(Duration::from_millis(1)),
        }
    }
}

/// Why an announce watch could not be declared.
#[derive(Debug)]
pub enum WatchError {
    /// This daemon has no network plane (a mirror-only / `CERULION_NETD_NETWORK=off`
    /// daemon). Not a failure to report loudly — there is simply nothing to watch.
    NoNetwork,
    /// The shared session could not be opened, or the subscription was refused.
    Session(String),
}

impl std::fmt::Display for WatchError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            WatchError::NoNetwork => write!(
                f,
                "cerulion-netd is not network-configured — no announce space to watch"
            ),
            WatchError::Session(e) => write!(f, "announce watch could not be declared: {e}"),
        }
    }
}

impl std::error::Error for WatchError {}

/// A live announce subscription, drained one transition at a time.
///
/// `next_event` blocks up to `timeout` and returns `None` on a quiet window — a
/// TIMEOUT, never an error and never "nothing is out there".
pub trait AnnounceStream: Send {
    /// Block up to `timeout` for the next transition.
    fn next_event(&mut self, timeout: Duration) -> Option<AnnounceEvent>;
}

/// The DI seam for the announce watch (mirroring the mirror / egress / query planes):
/// production watches netd's ONE shared zenoh session; the tests inject a scripted
/// stream so the coalescing + push contracts are pinned without a network.
pub trait AnnounceWatchPlane: Send + Sync {
    /// Declare the watch. Called ONCE, on the first `subscribe_catalog`.
    fn watch(&self) -> Result<Box<dyn AnnounceStream>, WatchError>;
}

/// The mirror-only daemon's plane: there is no session, so there is nothing to watch.
/// A `subscribe_catalog` against it succeeds (the consumer is subscribed) but no push
/// can ever arrive — which is the correct outcome for a daemon with no network, and is
/// reported as such in the subscribe response's `watching` flag.
pub struct NoopAnnounceWatchPlane;

impl AnnounceWatchPlane for NoopAnnounceWatchPlane {
    fn watch(&self) -> Result<Box<dyn AnnounceStream>, WatchError> {
        Err(WatchError::NoNetwork)
    }
}

/// The production plane: an [`cerulion_core::transport::discovery::AnnounceWatch`]
/// over the SAME network-configured `TransportManager` the mirror / egress / query
/// planes own (Principle #8 — one zenoh session for the whole machine).
pub struct GatewayAnnounceWatchPlane {
    manager: Arc<cerulion_core::transport::TransportManager>,
}

impl GatewayAnnounceWatchPlane {
    /// Wrap the shared (network-configured) transport manager.
    pub fn new(manager: Arc<cerulion_core::transport::TransportManager>) -> Self {
        Self { manager }
    }
}

/// The production stream: netd's liveliness subscriber over `cerulion_ann/**`.
struct GatewayAnnounceStream {
    watch: cerulion_core::transport::discovery::AnnounceWatch,
    /// Keeps the manager (and therefore the session the subscriber rides) alive for as
    /// long as the stream is drained.
    _manager: Arc<cerulion_core::transport::TransportManager>,
}

impl AnnounceStream for GatewayAnnounceStream {
    fn next_event(&mut self, timeout: Duration) -> Option<AnnounceEvent> {
        self.watch.next_event(timeout)
    }
}

impl AnnounceWatchPlane for GatewayAnnounceWatchPlane {
    fn watch(&self) -> Result<Box<dyn AnnounceStream>, WatchError> {
        // The session type stays INFERRED (never named), so `zenoh` need not be a
        // direct netd dependency — the same discipline `query.rs` keeps.
        let net = self.manager.network().ok_or(WatchError::NoNetwork)?;
        let session = net
            .session()
            .map_err(|e| WatchError::Session(e.to_string()))?;
        let watch = cerulion_core::transport::discovery::AnnounceWatch::declare(session)
            .map_err(|e| WatchError::Session(e.to_string()))?;
        Ok(Box::new(GatewayAnnounceStream {
            watch,
            _manager: Arc::clone(&self.manager),
        }))
    }
}

/// ONE subscribed control connection's push slot.
///
/// Capacity ONE, with a LOSSLESS merge on overflow: a push that lands while an
/// undelivered one is still sitting here folds into it (same [`CatalogDelta::merge`]
/// the coalescer uses) and takes the newer version. So a slow consumer never wedges
/// anything and never loses information either — it simply receives one notification
/// describing everything it missed, which is exactly the full-resync semantics its
/// response (refresh) needs anyway.
#[derive(Debug, Default)]
struct PushSlot {
    pending: Option<CatalogChanged>,
    /// How many pushes were folded into a still-undelivered one (Principle #3 — an
    /// operator can see a consumer falling behind rather than inferring it).
    coalesced_in_slot: u64,
}

/// The per-connection subscriber registry + the announce-watch thread.
///
/// The watch thread NEVER writes to a socket: it stores into per-connection push slots under a
/// short-lived mutex and returns. Each subscribed connection's OWN handler thread
/// drains its slot on the read loop it is already running (netd's `CONN_READ_TIMEOUT`
/// wakes it every 200 ms), so the one thread that writes a connection's stream is the
/// one that always did — no second writer, no interleaved half-lines — and a consumer
/// that has stopped reading can only ever stall ITSELF (its own write times out and
/// its own connection is dropped, exactly as for any other response).
pub struct CatalogEventHub {
    inner: Mutex<HubState>,
    /// Whether the watch thread is running (it is started by the FIRST subscriber, so
    /// a netd nobody subscribes to opens no session and behaves exactly as before).
    watching: AtomicBool,
    shutdown: Arc<AtomicBool>,
    plane: Arc<dyn AnnounceWatchPlane>,
    /// The coalescing window this hub's watch thread uses (production
    /// [`CHANGE_COALESCE_WINDOW`]; tests shorten it).
    window: Duration,
}

impl std::fmt::Debug for CatalogEventHub {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let state = lock_hub(&self.inner);
        f.debug_struct("CatalogEventHub")
            .field("version", &state.version)
            .field("watching", &self.watching.load(Ordering::Acquire))
            .field("subscribers", &state.subscribers.len())
            .finish_non_exhaustive()
    }
}

/// ONE control connection's push WAKER — the write end of a pipe
/// its handler thread waits on alongside its socket.
///
/// It exists because the push drain sits immediately before the read, so without it a
/// catalog-change push waits up to one `CONN_READ_TIMEOUT` (200 ms) for the read to
/// time out. A condvar cannot serve here: the handler must wait for
/// socket-readable OR push-slot-filled in ONE call, and only an fd can join the
/// socket in a `poll(2)` set. Waiting on the socket alone would make a push wait for
/// the client's next request — i.e. NEVER, on an idle Studio — which would be a
/// silent regression.
///
/// **The write is NON-BLOCKING and its failure is deliberately ignored.** The watch
/// thread pokes this while holding the hub lock, and the whole no-wedge story
/// is that the watch thread never does I/O that a stalled consumer can block. A full
/// pipe means the handler has not yet drained a byte that is ALREADY there, so the
/// wake it would deliver is redundant by construction; a broken pipe means the
/// handler is gone and its slot is about to be removed. Neither is worth blocking a
/// process-wide thread for, and neither loses a push: the slot itself is the
/// durable carrier, and the `CONN_READ_TIMEOUT` fallback still runs.
struct PushWaker {
    writer: std::io::PipeWriter,
}

/// Put ONE end of a push-waker pipe into non-blocking mode.
///
/// BOTH ends go through here, because both would wedge a thread that must never
/// block — the WRITE end is poked by the shared watch thread under the hub lock
/// (see [`PushWaker`]), and the READ end is drained by a connection handler that
/// is also the only thread that could ever close the writer, so a blocking read on
/// an empty pipe is a permanent self-deadlock (`drain_waker`'s doc states the
/// argument in full).
///
/// `std::io::pipe()` returns BOTH ends blocking, and the two ends are separate open
/// file descriptions — arming one says nothing about the other, which is exactly the
/// mistake this helper exists to make hard to repeat.
pub(crate) fn set_fd_nonblocking(fd: std::os::fd::RawFd) -> std::io::Result<()> {
    // SAFETY: the caller owns `fd` for the duration of these calls.
    let flags = unsafe { libc::fcntl(fd, libc::F_GETFL) };
    if flags < 0 {
        return Err(std::io::Error::last_os_error());
    }
    if unsafe { libc::fcntl(fd, libc::F_SETFL, flags | libc::O_NONBLOCK) } < 0 {
        return Err(std::io::Error::last_os_error());
    }
    Ok(())
}

#[cfg(test)]
thread_local! {
    /// Test seam — see [`fail_waker_arming_on_this_thread_for_test`].
    static FAIL_WAKER_ARMING: std::cell::Cell<bool> = const { std::cell::Cell::new(false) };
}

/// Test seam: make every [`PushWaker::new`] on THIS THREAD fail.
///
/// The real trigger — `fcntl(F_GETFL)` or `F_SETFL(O_NONBLOCK)` failing on a freshly
/// created, owned pipe write fd — is unreachable on either platform netd targets,
/// and the obvious stand-in ABORTS the process: an `OwnedFd` over an unallocatable
/// number is closed by its own `Drop`, `close` returns `EBADF`, and std's I/O-safety
/// check aborts. So the arm is driven here instead.
///
/// THREAD-LOCAL, not a global: `PushWaker::new` runs on the CALLER's thread, so a
/// test arms it inside the handler thread it spawns and no sibling lib test in the
/// same binary can observe it. `#[cfg(test)]` — none of this reaches a shipped daemon.
#[cfg(test)]
pub(crate) fn fail_waker_arming_on_this_thread_for_test(fail: bool) {
    FAIL_WAKER_ARMING.with(|f| f.set(fail));
}

impl PushWaker {
    /// Wrap a pipe write end, making it NON-BLOCKING (see the type doc — a blocking
    /// write here could stall the shared watch thread on one wedged consumer).
    fn new(writer: std::io::PipeWriter) -> std::io::Result<Self> {
        use std::os::fd::AsRawFd;
        #[cfg(test)]
        if FAIL_WAKER_ARMING.with(|f| f.get()) {
            return Err(std::io::Error::other(
                "test seam: forced waker-arming failure",
            ));
        }
        set_fd_nonblocking(writer.as_raw_fd())?;
        Ok(Self { writer })
    }

    /// Poke the handler. Never blocks; a full or broken pipe is a no-op (see the
    /// type doc — the push itself is already in the slot either way).
    fn wake(&self) {
        use std::io::Write;
        let _ = (&self.writer).write(&[1u8]);
    }
}

#[derive(Default)]
struct HubState {
    subscribers: BTreeMap<ConnId, PushSlot>,
    /// Per-connection push wakers, registered by each handler at
    /// start and cleared on its cleanup.
    ///
    /// Deliberately a SEPARATE map from `subscribers` rather than a field on
    /// [`PushSlot`]: `subscribe` REPLACES the slot (so the snapshot supersedes
    /// anything undelivered), which would silently drop a waker stored inside it and
    /// leave a re-subscribing connection paced by the timeout forever. The two have
    /// different lifetimes on purpose — a waker lives for the CONNECTION, a slot for
    /// the SUBSCRIPTION.
    wakers: BTreeMap<ConnId, PushWaker>,
    /// The announce-view generation, bumped once per PUSH — so a consumer that missed a
    /// notification (a dropped connection, a re-subscribe) sees a version jump rather
    /// than silently believing it is current.
    ///
    /// It lives INSIDE the lock, beside `robots`, deliberately. As an
    /// `AtomicU64` outside it, `publish` would bump the version BEFORE taking the lock to
    /// write the robot set — so a `subscribe` that won the lock in that gap would read
    /// `(version N, robots R_{N-1})` and report a generation its own robot list did
    /// not belong to. A consumer starts from this snapshot, so the pairing is
    /// load-bearing.
    version: u64,
    /// The last robot set the watch published — carried on every push so a consumer
    /// has the authoritative post-change state without a second-query race.
    robots: Vec<String>,
    watch_thread: Option<JoinHandle<()>>,
}

fn lock_hub(m: &Mutex<HubState>) -> MutexGuard<'_, HubState> {
    m.lock().unwrap_or_else(PoisonError::into_inner)
}

impl CatalogEventHub {
    /// A hub over `plane` with the production coalescing window.
    pub fn new(plane: Arc<dyn AnnounceWatchPlane>, shutdown: Arc<AtomicBool>) -> Self {
        Self::with_window(plane, shutdown, CHANGE_COALESCE_WINDOW)
    }

    /// [`Self::new`] with an explicit coalescing window (the test seam — an arm that
    /// pins the burst-to-one-push contract should not have to sleep 250 ms to do it).
    pub fn with_window(
        plane: Arc<dyn AnnounceWatchPlane>,
        shutdown: Arc<AtomicBool>,
        window: Duration,
    ) -> Self {
        Self {
            inner: Mutex::new(HubState::default()),
            watching: AtomicBool::new(false),
            shutdown,
            plane,
            window,
        }
    }

    /// Subscribe `conn` to catalog-change pushes and start the watch if this is the
    /// first subscriber. Returns `(version, robots, watching)` — the snapshot the
    /// subscribe response carries.
    ///
    /// `version` and `robots` are read under ONE lock acquisition, so they are always
    /// the SAME generation — a consumer can compare the version it was handed against
    /// the version on a later push and trust that its robot list belonged to it.
    ///
    /// The subscribe also CLEARS any push still sitting undelivered in this
    /// connection's slot. That is not a loss: the snapshot carries the same `version`
    /// and the same authoritative `robots` the push would have, and the delta a push
    /// adds on top exists only to say WHAT MOVED — which a consumer that is about to
    /// refresh from the snapshot does not need. Clearing makes the contract exact: the
    /// snapshot IS the state you start from, and every push after it is strictly newer
    /// information.
    ///
    /// `watching` is FALSE when this daemon has no announce space to watch (no network
    /// plane): the subscription is still registered — nothing is wedged, and a later
    /// network-configured daemon is a restart away — but the consumer is told plainly
    /// that no push will arrive, so it keeps its own refresh path rather than waiting
    /// forever on an event that cannot come.
    pub fn subscribe(self: &Arc<Self>, conn: ConnId) -> (u64, Vec<String>, bool) {
        let watching = self.ensure_watching();
        let mut state = lock_hub(&self.inner);
        let version = state.version;
        let robots = state.robots.clone();
        // Re-subscribing replaces the slot, so the snapshot supersedes anything
        // undelivered (see above).
        state.subscribers.insert(conn, PushSlot::default());
        (version, robots, watching)
    }

    /// Drop `conn`'s subscription (connection close — the same crash-safe rule the
    /// demand plane uses). A no-op for a connection that never subscribed.
    ///
    /// This does NOT clear the connection's push waker — a client
    /// may unsubscribe and re-subscribe on one connection, and the waker belongs to
    /// the connection. [`Self::clear_waker`] is the connection-scoped teardown.
    pub fn unsubscribe(&self, conn: ConnId) {
        lock_hub(&self.inner).subscribers.remove(&conn);
    }

    /// Register `conn`'s push waker — the write end of the pipe its
    /// handler thread waits on beside its socket. Called ONCE at handler start (a
    /// connection is not yet subscribed then, which is fine: the waker is keyed by
    /// connection, and a push is only ever delivered to a SUBSCRIBED one).
    ///
    /// **Returns whether the waker was ARMED**, and the caller must act on it. The
    /// writer is taken BY VALUE, so a failed arming DROPS it and closes the write
    /// end; a handler that kept polling the read end would then see a permanent
    /// `POLLHUP`, take the push arm on every poll, read `Ok(0)` and go straight round
    /// again — MEASURED at ~365k iterations/s, i.e. the busy-loop
    /// class, on the one path whose own message promises the timeout
    /// cadence. Reporting the failure is what lets the handler drop its read end and
    /// get the cadence it was promised. (The sibling degradation — `mint_waker_pipe`
    /// failing — needs no such report, because there is no read end to keep.)
    ///
    /// `#[must_use]` is load-bearing rather than decorative: with `dead_code`/clippy
    /// denied at the gate, dropping this answer cannot compile without a deliberate,
    /// reviewable `let _ =`.
    #[must_use]
    pub fn set_waker(&self, conn: ConnId, writer: std::io::PipeWriter) -> bool {
        match PushWaker::new(writer) {
            Ok(waker) => {
                lock_hub(&self.inner).wakers.insert(conn, waker);
                true
            }
            Err(e) => {
                // Degrade LOUDLY to the timeout rather than failing the connection:
                // the push still arrives, just up to one `CONN_READ_TIMEOUT` later,
                // which is exactly the cadence of a connection with no waker.
                tracing::warn!(
                    conn,
                    error = %e,
                    "could not arm this connection's push waker — catalog-change \
                     pushes fall back to the read-timeout cadence for its lifetime"
                );
                false
            }
        }
    }

    /// Drop `conn`'s push waker (connection teardown — every
    /// `cleanup_connection` exit path). Closing the write end also lets a handler
    /// still polling its read end see EOF rather than block, though in practice the
    /// handler is the caller.
    pub fn clear_waker(&self, conn: ConnId) {
        lock_hub(&self.inner).wakers.remove(&conn);
    }

    /// Take `conn`'s pending push, if any. Called by the connection's OWN handler
    /// thread on the read loop it already runs.
    pub fn take_pending(&self, conn: ConnId) -> Option<CatalogChanged> {
        let mut state = lock_hub(&self.inner);
        let slot = state.subscribers.get_mut(&conn)?;
        slot.coalesced_in_slot = 0;
        slot.pending.take()
    }

    /// Observable state (Principle #3): how many connections currently hold a push
    /// waker. This is the LEAK observable: a waker is armed once per connection and
    /// released by `cleanup_connection`, so a value that does not return to zero
    /// after every connection closes means an exit path forgot to release it — and a
    /// daemon that is spawned once and lives for the machine's uptime would grow a
    /// pipe fd pair per dead consumer.
    pub fn waker_count(&self) -> usize {
        lock_hub(&self.inner).wakers.len()
    }

    /// How many connections are currently subscribed (Principle #3 / tests).
    pub fn subscriber_count(&self) -> usize {
        lock_hub(&self.inner).subscribers.len()
    }

    /// The current catalog version (Principle #3 / tests).
    pub fn version(&self) -> u64 {
        lock_hub(&self.inner).version
    }

    /// Whether the announce watch is running.
    pub fn is_watching(&self) -> bool {
        self.watching.load(Ordering::Acquire)
    }

    /// Publish ONE coalesced change to every subscriber. Pure fan-out under the hub
    /// lock — NO I/O, so a stalled consumer cannot stall the watch thread or a healthy
    /// sibling.
    fn publish(&self, delta: CatalogDelta, batches: u64, robots: Vec<String>) {
        self.publish_inner(delta, batches, robots, || {});
    }

    /// [`Self::publish`] with a hook run at the point where a concurrent `subscribe`
    /// could interleave (the generation-consistency test seam).
    ///
    /// The hook fires BEFORE the lock is taken and therefore before the version is
    /// bumped, which is the whole point: a `subscribe` called from the hook observes
    /// the generation that is still current. Were the version bumped by an
    /// `AtomicU64::fetch_add` ABOVE this point, the same hook would see `(N, R_{N-1})` —
    /// a version and a robot set from different generations. The hook's placement is the
    /// same either way; only what has already happened by then differs, which is what
    /// makes the pin fail on that shape rather than merely describe it.
    fn publish_inner(
        &self,
        delta: CatalogDelta,
        batches: u64,
        robots: Vec<String>,
        at_the_interleave_point: impl FnOnce(),
    ) {
        at_the_interleave_point();
        let mut state = lock_hub(&self.inner);
        // The bump and the robot-set write happen under ONE lock acquisition, so no
        // reader can ever see one without the other (D2).
        state.version += 1;
        let version = state.version;
        state.robots = robots.clone();
        let subscriber_count = state.subscribers.len();
        // Collect the connections that got a push so their wakers
        // can be poked below. Reborrowed after the slot loop because `subscribers`
        // and `wakers` are separate fields of the same guard.
        // hot-path-alloc-ok: once per COALESCED catalog change (a >=250 ms window),
        // never per frame.
        let mut poked: Vec<ConnId> = Vec::with_capacity(subscriber_count);
        for (conn, slot) in state.subscribers.iter_mut() {
            poked.push(*conn);
            match &mut slot.pending {
                // The consumer has not drained the previous push yet: fold this one
                // into it (lossless) and bump the version to the newest.
                Some(existing) => {
                    slot.coalesced_in_slot = slot.coalesced_in_slot.saturating_add(1);
                    existing.merge_newer(&delta, batches, version, &robots);
                }
                None => {
                    slot.pending = Some(CatalogChanged::new(version, &delta, batches, &robots));
                }
            }
        }
        // The slot is written FIRST, then the waker is poked — a
        // handler woken before its slot was filled would find nothing and go back to
        // the timeout, which is the lost-wakeup shape in miniature. Both happen under
        // the SAME lock acquisition, so a woken handler's `take_pending` (which takes
        // that lock) cannot observe an empty slot.
        for conn in &poked {
            if let Some(waker) = state.wakers.get(conn) {
                waker.wake();
            }
        }
        tracing::debug!(
            version,
            subscribers = subscriber_count,
            batches,
            robots_added = delta.robots_added.len(),
            robots_removed = delta.robots_removed.len(),
            topics_added = delta.topics_added,
            topics_removed = delta.topics_removed,
            "catalog change published to subscribers"
        );
    }

    /// Test seam: publish with a hook at the interleave point (see
    /// [`Self::publish_inner`]). Lets a test drive a `subscribe` INTO the window a
    /// concurrent publish opens, deterministically and with no threads.
    #[cfg(test)]
    pub(crate) fn publish_with_interleave_for_test(
        &self,
        delta: CatalogDelta,
        batches: u64,
        robots: Vec<String>,
        at_the_interleave_point: impl FnOnce(),
    ) {
        self.publish_inner(delta, batches, robots, at_the_interleave_point);
    }

    /// Start the announce watch thread if it is not already running. Returns whether a
    /// watch is live.
    fn ensure_watching(self: &Arc<Self>) -> bool {
        {
            let state = lock_hub(&self.inner);
            if state.watch_thread.is_some() {
                return self.watching.load(Ordering::Acquire);
            }
        }
        let stream = match self.plane.watch() {
            Ok(stream) => stream,
            Err(WatchError::NoNetwork) => {
                tracing::info!(
                    "no network plane — catalog-change pushes are unavailable on \
                     this daemon (consumers keep their own refresh path)"
                );
                return false;
            }
            Err(e) => {
                tracing::warn!(
                    error = %e,
                    "could not declare the announce watch — catalog-change pushes \
                     are unavailable (consumers keep their own refresh path)"
                );
                return false;
            }
        };
        let hub = Arc::clone(self);
        let shutdown = Arc::clone(&self.shutdown);
        let window = self.window;
        let spawned = std::thread::Builder::new()
            .name("netd-announce".to_string())
            .spawn(move || watch_loop(hub, stream, shutdown, window));
        match spawned {
            Ok(handle) => {
                let mut state = lock_hub(&self.inner);
                // A racing `subscribe` could have started one in the gap; keep the
                // first and let this one see the shutdown flag on its own schedule.
                if state.watch_thread.is_none() {
                    state.watch_thread = Some(handle);
                }
                self.watching.store(true, Ordering::Release);
                tracing::info!("announce watch started — catalog changes now PUSH");
                true
            }
            Err(e) => {
                tracing::warn!(
                    error = %e,
                    "could not spawn the announce-watch thread — catalog-change \
                     pushes are unavailable (consumers keep their own refresh path)"
                );
                false
            }
        }
    }

    /// Join the watch thread (daemon shutdown). The caller has already flipped the
    /// shared shutdown flag.
    pub fn shutdown(&self) {
        let handle = lock_hub(&self.inner).watch_thread.take();
        if let Some(handle) = handle {
            let _ = handle.join();
        }
    }
}

/// The announce-watch loop: fold transitions into the view, coalesce them, and publish
/// one push per window. Never writes a socket.
fn watch_loop(
    hub: Arc<CatalogEventHub>,
    mut stream: Box<dyn AnnounceStream>,
    shutdown: Arc<AtomicBool>,
    window: Duration,
) {
    let mut view = AnnounceView::new();
    let mut coalescer = ChangeCoalescer::new(window);
    while !shutdown.load(Ordering::Acquire) {
        let block = coalescer.block_for(Instant::now(), WATCH_IDLE_TIMEOUT);
        if let Some(event) = stream.next_event(block) {
            if let Some(delta) = view.apply(&event) {
                coalescer.note(delta, Instant::now());
            }
        }
        if let Some((delta, batches)) = coalescer.take_if_due(Instant::now()) {
            hub.publish(delta, batches, view.robots());
        }
    }
    tracing::debug!("announce watch stopped");
}

#[cfg(test)]
mod tests {
    use super::*;

    fn alive(robot: &str, topic: Option<&str>) -> AnnounceEvent {
        AnnounceEvent::Alive {
            robot: robot.to_string(),
            topic: topic.map(str::to_string),
        }
    }

    fn lost(robot: &str, topic: Option<&str>) -> AnnounceEvent {
        AnnounceEvent::Lost {
            robot: robot.to_string(),
            topic: topic.map(str::to_string),
        }
    }

    /// The view's core contract against a HAND-WRITTEN expectation: a robot becomes
    /// present on its FIRST token of any kind, stays present while it holds any, and
    /// leaves only when the last one goes.
    #[test]
    fn a_robot_is_present_from_its_first_token_until_its_last() {
        let mut view = AnnounceView::new();

        // First topic token ⇒ the robot ARRIVES (and one topic is counted).
        let d = view.apply(&alive("go2", Some("/tf"))).expect("a change");
        assert_eq!(d.robots_added, BTreeSet::from(["go2".to_string()]));
        assert_eq!(d.topics_added, 1);
        assert!(d.robots_removed.is_empty());
        assert_eq!(view.robots(), vec!["go2".to_string()]);

        // The bare identity token arriving later is NOT a second arrival.
        let d = view.apply(&alive("go2", None)).expect("a change");
        assert!(d.robots_added.is_empty(), "already present");
        assert_eq!(d.topics_added, 0, "the identity token is not a topic");

        // A second topic: present throughout.
        let d = view.apply(&alive("go2", Some("/odom"))).expect("a change");
        assert!(d.robots_added.is_empty());
        assert_eq!(d.topics_added, 1);
        assert_eq!(view.topic_count("go2"), 2);

        // Losing ONE topic does not lose the robot.
        let d = view.apply(&lost("go2", Some("/tf"))).expect("a change");
        assert_eq!(d.topics_removed, 1);
        assert!(d.robots_removed.is_empty(), "still holds /odom + identity");

        // Nor does losing the identity while a topic remains.
        let d = view.apply(&lost("go2", None)).expect("a change");
        assert!(d.robots_removed.is_empty(), "still holds /odom");

        // The LAST token leaving is the departure.
        let d = view.apply(&lost("go2", Some("/odom"))).expect("a change");
        assert_eq!(d.topics_removed, 1);
        assert_eq!(d.robots_removed, BTreeSet::from(["go2".to_string()]));
        assert!(view.robots().is_empty());
    }

    /// A re-delivered token is a NO-OP. Liveliness legitimately re-delivers (the
    /// history replay at watch start is exactly that), and a view that treated a
    /// re-delivery as a change would push a notification every time netd re-subscribed.
    #[test]
    fn a_re_delivered_token_produces_no_change() {
        let mut view = AnnounceView::new();
        assert!(view.apply(&alive("go2", Some("/tf"))).is_some());
        assert!(
            view.apply(&alive("go2", Some("/tf"))).is_none(),
            "the same token again is not a change"
        );
        assert!(view.apply(&alive("go2", None)).is_some());
        assert!(view.apply(&alive("go2", None)).is_none());
    }

    /// A `Lost` for a robot never seen is a no-op AND leaves no phantom entry — a
    /// robot must never appear in `robots()` because it was reported gone.
    #[test]
    fn a_lost_for_an_unknown_robot_leaves_no_phantom() {
        let mut view = AnnounceView::new();
        assert!(view.apply(&lost("ghost", Some("/tf"))).is_none());
        assert!(view.apply(&lost("ghost", None)).is_none());
        assert!(
            view.robots().is_empty(),
            "a never-seen robot must not be listed, got {:?}",
            view.robots()
        );
    }

    /// A whole gateway dying drops EVERY token it held at once. The view must report
    /// that as ONE robot removal (plus the topic churn), not as a robot that lingers
    /// because its identity token happened to be processed first.
    #[test]
    fn a_dying_gateway_drops_every_token_and_removes_the_robot_once() {
        let mut view = AnnounceView::new();
        view.apply(&alive("go2", None));
        for i in 0..5 {
            view.apply(&alive("go2", Some(&format!("/t{i}"))));
        }
        // Session death: the identity token first, then each topic.
        let mut merged = CatalogDelta::default();
        if let Some(d) = view.apply(&lost("go2", None)) {
            merged.merge(d);
        }
        for i in 0..5 {
            if let Some(d) = view.apply(&lost("go2", Some(&format!("/t{i}")))) {
                merged.merge(d);
            }
        }
        assert_eq!(merged.robots_removed, BTreeSet::from(["go2".to_string()]));
        assert!(merged.robots_added.is_empty());
        assert_eq!(merged.topics_removed, 5);
        assert!(view.robots().is_empty());
    }

    /// Merge resolves robot membership to the NET effect. Reporting a robot as both
    /// added and removed would send a consumer chasing something that is not there.
    #[test]
    fn merge_nets_robot_membership_and_sums_topic_churn() {
        let mut a = CatalogDelta {
            robots_added: BTreeSet::from(["go2".to_string()]),
            topics_added: 3,
            ..Default::default()
        };
        a.merge(CatalogDelta {
            robots_removed: BTreeSet::from(["go2".to_string()]),
            topics_removed: 3,
            ..Default::default()
        });
        assert!(a.robots_added.is_empty(), "the later removal wins");
        assert_eq!(a.robots_removed, BTreeSet::from(["go2".to_string()]));
        assert_eq!(a.topics_added, 3, "churn sums, it does not net");
        assert_eq!(a.topics_removed, 3);

        // And the other direction: a robot that came back is ADDED, not removed.
        let mut b = CatalogDelta {
            robots_removed: BTreeSet::from(["orin".to_string()]),
            ..Default::default()
        };
        b.merge(CatalogDelta {
            robots_added: BTreeSet::from(["orin".to_string()]),
            ..Default::default()
        });
        assert_eq!(b.robots_added, BTreeSet::from(["orin".to_string()]));
        assert!(b.robots_removed.is_empty());
    }

    /// THE burst pin, at the level of the coalescer: 75 topic announces inside one
    /// window produce EXACTLY ONE batch, carrying all 75.
    #[test]
    fn a_seventy_five_topic_burst_coalesces_into_one_batch() {
        let window = Duration::from_millis(250);
        let mut coalescer = ChangeCoalescer::new(window);
        let t0 = Instant::now();
        let mut view = AnnounceView::new();
        for i in 0..75 {
            let e = alive("go2", Some(&format!("/attach/t{i}")));
            let delta = view.apply(&e).expect("each token is new");
            // The whole burst lands well inside the window.
            coalescer.note(delta, t0 + Duration::from_millis(i % 10));
        }
        assert!(
            coalescer
                .take_if_due(t0 + Duration::from_millis(100))
                .is_none(),
            "not due yet — a mid-burst flush would fan out into several pushes"
        );
        let (delta, batches) = coalescer
            .take_if_due(t0 + window + Duration::from_millis(1))
            .expect("due after the window");
        assert_eq!(batches, 75, "all 75 transitions folded into ONE push");
        assert_eq!(delta.topics_added, 75);
        assert_eq!(delta.robots_added, BTreeSet::from(["go2".to_string()]));
        assert!(
            coalescer
                .take_if_due(t0 + Duration::from_secs(10))
                .is_none(),
            "the batch was taken — nothing left to push"
        );
    }

    /// The window anchors on the FIRST change and is NOT extended by later ones, so a
    /// continuous trickle still notifies once per window instead of never. (A
    /// quiet-window debounce would fail this: every 10 ms change would reset it.)
    #[test]
    fn a_continuous_trickle_still_flushes_once_per_window() {
        let window = Duration::from_millis(100);
        let mut coalescer = ChangeCoalescer::new(window);
        let t0 = Instant::now();
        let mut flushes = 0;
        // A change every 10 ms for 1 second.
        for step in 0..100u64 {
            let now = t0 + Duration::from_millis(step * 10);
            coalescer.note(
                CatalogDelta {
                    topics_added: 1,
                    ..Default::default()
                },
                now,
            );
            if coalescer.take_if_due(now).is_some() {
                flushes += 1;
            }
        }
        assert!(
            (9..=11).contains(&flushes),
            "a 1s trickle at a 100ms window must flush ~10 times, got {flushes}"
        );
    }

    /// An EMPTY delta never opens a window — a stream of no-op events (e.g. a
    /// re-subscribe replaying tokens we already know) must produce no push at all.
    #[test]
    fn an_empty_delta_never_opens_a_window() {
        let mut coalescer = ChangeCoalescer::new(Duration::from_millis(10));
        let t0 = Instant::now();
        coalescer.note(CatalogDelta::default(), t0);
        assert!(coalescer.due_at().is_none());
        assert!(coalescer.take_if_due(t0 + Duration::from_secs(1)).is_none());
    }

    /// THE generation-consistency pin: a `subscribe` that interleaves
    /// with an in-flight `publish` must be handed a version and a robot set from the
    /// SAME generation.
    ///
    /// Were the version an `AtomicU64` bumped BEFORE the lock that writes the
    /// robot set, a subscribe winning the lock in that gap would get `(version N, robots
    /// R_{N-1})` — a consumer would then believe it was current at N while holding the
    /// robot list from N-1, and every later push at N+1 would look like the expected
    /// next step, so the stale list would never be corrected. This test drives exactly
    /// that window through the deterministic hook (no threads, no timing).
    #[test]
    fn a_subscribe_interleaved_with_a_publish_reads_one_generation() {
        let hub = Arc::new(CatalogEventHub::with_window(
            Arc::new(NoopAnnounceWatchPlane),
            Arc::new(AtomicBool::new(false)),
            Duration::from_millis(10),
        ));

        // Generation 1: only `go2` is announcing.
        hub.publish(
            CatalogDelta {
                robots_added: BTreeSet::from(["go2".to_string()]),
                ..Default::default()
            },
            1,
            vec!["go2".to_string()],
        );
        assert_eq!(hub.version(), 1);

        // Generation 2 is in flight; a consumer subscribes right inside the window.
        let observed: Mutex<Option<(u64, Vec<String>)>> = Mutex::new(None);
        let hub_for_hook = Arc::clone(&hub);
        hub.publish_with_interleave_for_test(
            CatalogDelta {
                robots_added: BTreeSet::from(["orin".to_string()]),
                ..Default::default()
            },
            1,
            vec!["go2".to_string(), "orin".to_string()],
            || {
                let (version, robots, _watching) = hub_for_hook.subscribe(7);
                *observed.lock().unwrap() = Some((version, robots));
            },
        );

        let (version, robots) = observed.lock().unwrap().clone().expect("subscribed");
        assert_eq!(
            (version, robots.as_slice()),
            (1, ["go2".to_string()].as_slice()),
            "the snapshot must be ONE generation — never version 2 with \
             generation 1's robot list"
        );
        // And the world really did move on, so the arm is not vacuous.
        assert_eq!(hub.version(), 2);
        let (version, robots, _) = hub.subscribe(8);
        assert_eq!(version, 2);
        assert_eq!(robots, vec!["go2".to_string(), "orin".to_string()]);
    }

    /// A second generation-consistency pin: a re-subscribe CLEARS the
    /// connection's undelivered push, so its stream cannot show the snapshot and then
    /// a notification describing the same generation over again. The snapshot IS the
    /// starting state.
    #[test]
    fn a_re_subscribe_clears_the_undelivered_push_it_supersedes() {
        let hub = Arc::new(CatalogEventHub::with_window(
            Arc::new(NoopAnnounceWatchPlane),
            Arc::new(AtomicBool::new(false)),
            Duration::from_millis(10),
        ));
        hub.subscribe(1);
        hub.publish(
            CatalogDelta {
                robots_added: BTreeSet::from(["go2".to_string()]),
                ..Default::default()
            },
            1,
            vec!["go2".to_string()],
        );
        // A push is waiting, undelivered.
        assert_eq!(hub.subscriber_count(), 1);

        let (version, robots, _) = hub.subscribe(1);
        assert_eq!(version, 1);
        assert_eq!(robots, vec!["go2".to_string()]);
        assert!(
            hub.take_pending(1).is_none(),
            "the snapshot carries the same generation, so the superseded push must be \
             cleared rather than delivered on top of it"
        );
        // A LATER change still reaches the re-subscribed connection (the clear is not
        // an unsubscribe).
        hub.publish(
            CatalogDelta {
                robots_added: BTreeSet::from(["orin".to_string()]),
                ..Default::default()
            },
            1,
            vec!["go2".to_string(), "orin".to_string()],
        );
        let push = hub.take_pending(1).expect("later changes still arrive");
        assert_eq!(push.version, 2);
    }

    /// `block_for` returns the idle timeout when nothing is pending, the remaining
    /// window when something is, and NEVER zero (a zero-timeout receive would spin).
    #[test]
    fn block_for_never_returns_zero_and_tracks_the_pending_window() {
        let window = Duration::from_millis(100);
        let idle = Duration::from_millis(500);
        let mut coalescer = ChangeCoalescer::new(window);
        let t0 = Instant::now();
        assert_eq!(coalescer.block_for(t0, idle), idle, "nothing pending");
        coalescer.note(
            CatalogDelta {
                topics_added: 1,
                ..Default::default()
            },
            t0,
        );
        assert_eq!(
            coalescer.block_for(t0 + Duration::from_millis(40), idle),
            Duration::from_millis(60)
        );
        // Overdue: clamped to a positive minimum rather than zero.
        assert_eq!(
            coalescer.block_for(t0 + Duration::from_secs(5), idle),
            Duration::from_millis(1)
        );
    }

    /// A FULL waker pipe must not block `publish` — and therefore
    /// must not wedge the hub lock the poke loop holds.
    ///
    /// The `PushWaker` type doc makes this the whole no-wedge story ("the
    /// watch thread never does I/O that a stalled consumer can block"), and a single
    /// `fcntl` is all that enforces it — a line no other netd test
    /// notices when it is removed. The consequence is worse
    /// than a stalled push plane: `publish_inner` pokes under `lock_hub`, so a blocked
    /// write parks the shared announce-watch thread while holding the mutex that
    /// `take_pending`, `subscribe`, `unsubscribe` and `clear_waker` all take — and
    /// `handle_connection` calls `take_pending` at the top of EVERY pass, so every
    /// connection on the machine blocks behind one unresponsive consumer.
    ///
    /// Both halves are needed and neither implies the other: the publisher finishing
    /// says the write did not block, and a THIRD thread reading `waker_count()` says
    /// the lock was not held across one. The read end is deliberately never drained,
    /// so the pipe genuinely fills (typically 65,536 bytes at one byte per
    /// publish) and the loop keeps going well past it.
    #[test]
    fn a_full_waker_pipe_never_blocks_a_publish_or_the_hub_lock() {
        use std::sync::atomic::AtomicU64;

        // Comfortably past any platform's pipe capacity, so the buffer is FULL for
        // the great majority of the run rather than just at the end.
        const PUBLISHES: u64 = 200_000;
        // Where the probe asks for the lock: past a 64 KiB capacity, so it lands while
        // the writes are being refused (or, if that guarantee regressed, blocked).
        const PROBE_AFTER: u64 = 100_000;
        const DEADLINE: Duration = Duration::from_secs(30);

        let hub = Arc::new(CatalogEventHub::with_window(
            Arc::new(NoopAnnounceWatchPlane),
            Arc::new(AtomicBool::new(false)),
            Duration::from_millis(10),
        ));
        // The read end is HELD and never read — the wedged-consumer shape.
        let (_never_drained, tx) = std::io::pipe().expect("pipe");
        assert!(hub.set_waker(1, tx), "the waker must arm");
        let _ = hub.subscribe(1);

        let done = Arc::new(AtomicU64::new(0));
        let (pub_tx, pub_rx) = std::sync::mpsc::channel();
        let publisher = {
            let hub = Arc::clone(&hub);
            let done = Arc::clone(&done);
            std::thread::spawn(move || {
                for _ in 0..PUBLISHES {
                    hub.publish_with_interleave_for_test(
                        CatalogDelta::default(),
                        1,
                        Vec::new(),
                        || {},
                    );
                    done.fetch_add(1, Ordering::Relaxed);
                }
                let _ = pub_tx.send(());
            })
        };

        // The lock probe waits on a CONDITION (publishes observed), never a sleep, so
        // it asks at the right moment whichever way the run goes: past the capacity
        // with a non-blocking waker, and parked exactly at it with a blocking one.
        let (probe_tx, probe_rx) = std::sync::mpsc::channel();
        {
            let hub = Arc::clone(&hub);
            let done = Arc::clone(&done);
            std::thread::spawn(move || {
                let deadline = Instant::now() + DEADLINE;
                while done.load(Ordering::Relaxed) < PROBE_AFTER && Instant::now() < deadline {
                    std::thread::yield_now();
                }
                let _ = probe_tx.send(hub.waker_count());
            });
        }

        match probe_rx.recv_timeout(DEADLINE) {
            Ok(count) => assert_eq!(count, 1, "the waker is still registered"),
            Err(_) => panic!(
                "a third thread could not take the hub lock within {DEADLINE:?} while \
                 the poke loop was writing to a FULL waker pipe — the write end is \
                 blocking, so the shared watch thread is parked inside `publish_inner` \
                 holding `lock_hub`, and every connection's `take_pending` is stuck \
                 behind it"
            ),
        }
        match pub_rx.recv_timeout(DEADLINE) {
            Ok(()) => {}
            Err(_) => panic!(
                "publish did not complete {PUBLISHES} pokes within {DEADLINE:?} \
                 (reached {}) — a full waker pipe BLOCKED the poke, which is exactly \
                 what the O_NONBLOCK arming on the write end exists to prevent",
                done.load(Ordering::Relaxed)
            ),
        }
        publisher.join().expect("publisher thread");
    }
}
