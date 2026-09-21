// SPDX-License-Identifier: AGPL-3.0-only
//! The desk half of the event-driven sidebar — vizd relays
//! `cerulion-netd`'s catalog-change PUSHES to the controllers that asked for them.
//!
//! # The chain
//!
//! A robot's produced topics are zenoh LIVELINESS tokens. netd subscribes to that
//! space, coalesces the transitions, and pushes a notification down every UDS control
//! connection that sent `subscribe_catalog` (see `cerulion_netd::catalog_events`).
//! This module is the next hop: a background thread holds ONE such subscription and
//! forwards each change to every vizd controller that sent `subscribe_events`. The
//! controller's response is to run the refresh it already has — so a Studio sidebar
//! fills itself in when `ros2 attach` comes up on the robot, with nothing polling
//! anywhere on the path.
//!
//! # Why a SECOND netd connection, and what it costs
//!
//! vizd's demand plane holds one `NetdClient` behind a mutex and every demand /
//! release / query serializes on it. A subscription is a long-lived READ, so putting
//! it on that client would either wedge the demand plane or desync it (an unsolicited
//! push arriving where a response was expected). The subscription therefore gets its
//! own connection: same daemon, same socket, same zenoh session, and it holds no
//! demands, so the refcount plane is untouched.
//!
//! # Lifecycle: the standing connection is intended (by design)
//!
//! Unlike the demand plane's client — which is LAZY, connecting on the first remote
//! demand — the forwarder connects at STARTUP and stays connected for vizd's whole
//! life. netd counts a live connection as busy (`DemandRegistry::is_busy` →
//! `active_connections > 0`), so while vizd runs, netd's idle self-exit does
//! not fire even with nothing demanded.
//!
//! That is deliberate, not an accident: Studio keeps netd alive while
//! it is running, and netd exits when idle once Studio closes. A standing
//! subscription is what keeps discovery WARM — the sidebar is current the moment the
//! user looks, rather than after their first attach — and the idle grace is what
//! reclaims netd once the desk's consumers are gone.
//!
//! The second half of that only works if vizd actually exits when Studio does.
//! Closing Studio ORPHANS the vizd it spawned, which in turn pins netd; the
//! teardown belongs to the Studio shell, not to this daemon.
//!
//! # Opt-in, for the same reason netd's push is
//!
//! Studio's shell REFUSES to drive a vizd whose banner protocol differs from the one
//! it was built against (an exact-equality check), so this feature deliberately does
//! NOT bump [`crate::protocol::PROTOCOL_VERSION`] — and it does not need to. A
//! controller that never sends `subscribe_events` receives no unsolicited line, so it
//! reads exactly the lines an older daemon would have sent it; one that does send the
//! verb gets a structured unknown-method error from an OLD daemon and degrades to its
//! existing refresh path. Capability negotiation by verb, not by version.

use std::collections::{BTreeMap, BTreeSet};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex, MutexGuard, PoisonError};
use std::thread::JoinHandle;
use std::time::Duration;

use cerulion_core::transport::failure_regime_latch::{FailureRegimeLatch, RegimeDecision};

use crate::protocol::CatalogChangedEvent;

/// How long the forwarder waits before re-attempting a netd subscription that failed
/// for a transient reason (netd not up yet, connection dropped). Long enough not to
/// hammer a daemon that is down, short enough that a desk recovers on its own well
/// inside a user's patience.
const RESUBSCRIBE_BACKOFF: Duration = Duration::from_secs(2);

/// One controller connection's push slot — capacity ONE with a LOSSLESS merge, the
/// same policy netd uses. A controller that has not drained the previous notification
/// gets ONE line describing everything since, carrying the newest version; nothing is
/// lost and nothing queues without bound.
#[derive(Debug, Default)]
struct PushSlot {
    pending: Option<CatalogChangedEvent>,
}

/// Registry of controllers subscribed to catalog-change events, plus the
/// connection-id minting.
///
/// The forwarder thread NEVER writes a controller's socket: it stores into these slots
/// under a short-lived mutex. Each controller connection drains its own slot on the
/// read loop it already runs, so exactly one thread ever writes a given stream and a
/// controller that stopped reading can only stall itself.
#[derive(Debug, Default)]
pub struct EventHub {
    inner: Mutex<HubState>,
    next_conn: AtomicU64,
}

#[derive(Debug, Default)]
struct HubState {
    subscribers: BTreeMap<u64, PushSlot>,
    /// The last version relayed — reported to a controller at subscribe time so it can
    /// tell whether it is current without a second query.
    version: u64,
    /// The robots named by the last relayed change (the snapshot half).
    robots: Vec<String>,
    /// Whether a netd subscription is currently live. `false` means no push can
    /// arrive, which a controller is told PLAINLY at subscribe time.
    connected: bool,
    /// How the subscribe-retry latch has classified failures, so the
    /// flood-suppression ladder is observable independently of the log level
    /// (Principle #3 — the counter is exactly what survives filtering, which is the
    /// justification the shared latch's own docs give for downgrading repeats).
    subscribe_failures: SubscribeFailureCounts,
}

/// The subscribe-retry latch's decisions, counted. Unconditional: never reset by a
/// recovery, so an operator can see how bad a regime got after it cleared.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub struct SubscribeFailureCounts {
    /// Failures logged LOUDLY as the head of a regime.
    pub loud: u64,
    /// Failures downgraded to `debug!` while a regime was open.
    pub suppressed: u64,
    /// Loud re-announcements of a still-open regime at a decade boundary.
    pub re_announced: u64,
    /// Recoveries reported (a regime cleared after suppressing something).
    pub recovered: u64,
}

impl SubscribeFailureCounts {
    /// Every failure this hub's forwarder has seen, across all regimes.
    pub fn total(&self) -> u64 {
        self.loud + self.suppressed + self.re_announced
    }
}

fn lock_hub(m: &Mutex<HubState>) -> MutexGuard<'_, HubState> {
    m.lock().unwrap_or_else(PoisonError::into_inner)
}

impl EventHub {
    /// A fresh hub.
    pub fn new() -> Self {
        Self::default()
    }

    /// Mint a connection id (one per accepted controller connection).
    pub fn mint_conn(&self) -> u64 {
        self.next_conn.fetch_add(1, Ordering::Relaxed)
    }

    /// Subscribe `conn`. Returns `(version, robots, connected)` — the snapshot a
    /// controller starts from.
    pub fn subscribe(&self, conn: u64) -> (u64, Vec<String>, bool) {
        let mut state = lock_hub(&self.inner);
        state.subscribers.entry(conn).or_default();
        (state.version, state.robots.clone(), state.connected)
    }

    /// Drop `conn`'s subscription (connection close). A no-op if it never subscribed.
    pub fn unsubscribe(&self, conn: u64) {
        lock_hub(&self.inner).subscribers.remove(&conn);
    }

    /// Take `conn`'s pending notification, if any — called by the connection's own
    /// handler thread.
    pub fn take_pending(&self, conn: u64) -> Option<CatalogChangedEvent> {
        lock_hub(&self.inner)
            .subscribers
            .get_mut(&conn)?
            .pending
            .take()
    }

    /// How many controllers are subscribed (Principle #3 / tests).
    pub fn subscriber_count(&self) -> usize {
        lock_hub(&self.inner).subscribers.len()
    }

    /// The last relayed version (Principle #3 / tests).
    pub fn version(&self) -> u64 {
        lock_hub(&self.inner).version
    }

    /// Whether a netd subscription is currently live (Principle #3 / tests).
    pub fn is_connected(&self) -> bool {
        lock_hub(&self.inner).connected
    }

    /// Record whether the upstream netd subscription is live. A DROP is recorded so a
    /// controller that subscribes during an outage is told `connected: false` rather
    /// than being left to wait on a push that cannot come.
    pub fn set_connected(&self, connected: bool) {
        lock_hub(&self.inner).connected = connected;
    }

    /// The subscribe-retry latch's decision counts (Principle #3).
    pub fn subscribe_failure_counts(&self) -> SubscribeFailureCounts {
        lock_hub(&self.inner).subscribe_failures
    }

    /// Record one subscribe-retry latch decision.
    fn note_subscribe_decision(&self, decision: &RegimeDecision) {
        let counts = &mut lock_hub(&self.inner).subscribe_failures;
        match decision {
            RegimeDecision::Loud => counts.loud += 1,
            RegimeDecision::Suppressed { .. } => counts.suppressed += 1,
            RegimeDecision::StillFailing { .. } => counts.re_announced += 1,
        }
    }

    /// Record that a subscribe-failure regime recovered.
    fn note_subscribe_recovery(&self) {
        lock_hub(&self.inner).subscribe_failures.recovered += 1;
    }

    /// Adopt the upstream's SNAPSHOT as this hub's starting state.
    ///
    /// Called once per successful subscription, BEFORE the poll loop, so a controller
    /// that subscribes to a freshly-restarted vizd is handed the catalog that already
    /// exists rather than an empty one. Without it, a vizd restarted against a live
    /// netd reported `version: 0, robots: []` until the next CHANGE happened — and
    /// netd's watch is already running by then, so nothing replays what it missed: a
    /// desk whose robots were all up and steady would show an empty sidebar
    /// indefinitely.
    ///
    /// `connected` takes the snapshot's own `watching`, so a netd that cannot watch is
    /// reported as such instead of behind an optimistic `true`.
    ///
    /// The version only ever moves FORWARD: a reconnect to a RESTARTED netd (whose
    /// generation begins again at 0) must not walk a controller's version backwards,
    /// since a controller compares versions to decide whether it is current. This
    /// clamp composes with the stamping in [`Self::publish`] — together they are what
    /// make vizd's number monotone across a netd restart; the clamp ALONE only fixed
    /// the seed, leaving the very next push to walk it back.
    ///
    /// The robot set is adopted regardless — it is the authoritative present state
    /// either way. Note the transient this creates on a reconnect to a JUST-restarted
    /// netd: that netd's discovery has not converged yet, so the snapshot's robot list
    /// can be briefly EMPTY and a controller subscribing in that window sees no robots.
    /// It self-corrects within one coalescing window, as soon as the announce tokens
    /// (which are replayed to netd's watch by liveliness history) produce the first
    /// change.
    pub fn seed(&self, snapshot: &CatalogSnapshot) {
        let mut state = lock_hub(&self.inner);
        state.version = state.version.max(snapshot.version);
        state.robots = snapshot.robots.clone();
        state.connected = snapshot.watching;
    }

    /// Relay one change to every subscriber. Pure fan-out under the hub lock — NO I/O.
    pub fn publish(&self, mut event: CatalogChangedEvent) {
        let mut state = lock_hub(&self.inner);
        // STAMP vizd's OWN generation, ignoring whatever number the
        // upstream carried.
        //
        // netd's counter restarts at 1 when netd does. Mirroring it verbatim broke the
        // monotonicity this contract promises: a controller handed version 40 by
        // `seed` would see the very next push carry 1 (measured: snapshot 40, next push
        // 1), and an external consumer that legitimately skips a lower version — which
        // is what "compare it against the version on later events" invites — would then
        // ignore every push until netd's generation climbed back past 40.
        //
        // The two event types are deliberately separate contracts with separate
        // version stories, so this is where they part: vizd's number counts what vizd
        // has RELAYED, and only ever goes up.
        state.version += 1;
        event.version = state.version;
        state.robots = event.robots.clone();
        for slot in state.subscribers.values_mut() {
            match &mut slot.pending {
                Some(existing) => existing.merge_newer(&event),
                None => slot.pending = Some(event.clone()),
            }
        }
    }
}

/// The state an upstream subscription starts from.
///
/// A subscription hands back a SNAPSHOT before its first event, and dropping it is not
/// a cosmetic loss. `watching` is the upstream's own answer to "can a push ever
/// arrive here?" — reporting `connected: true` when netd said `watching: false` (a
/// `--network off` daemon, a failed watch declaration) tells a controller to wait for
/// an event that cannot come. And `version`/`robots` are what make the documented
/// recovery real: a vizd restarted against a live netd learns the CURRENT catalog from
/// them, where before it seeded an empty hub and stayed empty until the next change —
/// netd's watch is already running by then, so no history replay rescues it.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct CatalogSnapshot {
    /// The upstream catalog generation at subscribe time. Monotonic.
    pub version: u64,
    /// The robots currently announcing, sorted.
    pub robots: Vec<String>,
    /// Whether the upstream can actually deliver a change. `false` = no push will
    /// arrive (netd has no announce space to watch), which is reported through to the
    /// controller rather than hidden behind an optimistic `true`.
    pub watching: bool,
}

/// One poll of a catalog-event stream.
#[derive(Debug)]
pub enum CatalogEventPoll {
    /// A change arrived.
    Changed(Box<CatalogChangedEvent>),
    /// The poll window elapsed with nothing to report — NOT an error.
    Idle,
    /// The upstream connection is gone; the caller re-subscribes. Carries the reason.
    Disconnected(String),
}

/// A live catalog-change subscription, drained one poll at a time.
pub trait CatalogEventStream: Send {
    /// Poll for the next change (bounded — see [`CatalogEventPoll::Idle`]).
    fn poll(&mut self) -> CatalogEventPoll;
}

/// The DI seam for the upstream subscription (production: `cerulion-netd`; tests: a
/// scripted stream). Mirrors the `DemandPlane` pattern so the e2e stays hermetic.
pub trait CatalogEventSource: Send + Sync {
    /// Open a subscription, returning the upstream's SNAPSHOT alongside the stream.
    ///
    /// The snapshot is part of the seam rather than swallowed inside the production
    /// impl deliberately: it is what the hub seeds from, and a seam
    /// that discarded it made the two consequences — a fabricated `connected: true`
    /// and an empty-on-restart sidebar — INEXPRESSIBLE in a test, which is why neither
    /// was caught.
    ///
    /// `Err` carries a human-readable reason; the forwarder logs it and retries (a
    /// transient outage) or gives up (an unsupported daemon).
    fn subscribe(&self)
        -> Result<(CatalogSnapshot, Box<dyn CatalogEventStream>), SubscribeFailure>;
}

/// Why a subscription attempt failed.
#[derive(Debug)]
pub enum SubscribeFailure {
    /// Transient — netd is not up yet, or the connection dropped. Retry.
    Transient(String),
    /// PERMANENT for this daemon: the connected netd is too old to serve the verb.
    /// Retrying cannot help (netd is spawn-once and long-lived), so the forwarder
    /// stops and the desk keeps its existing refresh path — LOUDLY, never silently.
    Unsupported(String),
}

impl std::fmt::Display for SubscribeFailure {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            SubscribeFailure::Transient(m) | SubscribeFailure::Unsupported(m) => write!(f, "{m}"),
        }
    }
}

/// The production source: a SECOND `cerulion-netd` connection, dedicated to the
/// catalog-change subscription.
pub struct NetdCatalogEventSource {
    /// `Some` = an explicit socket (the e2e's in-process netd); `None` = the
    /// well-known path, spawning the daemon if it is not running.
    socket: Option<std::path::PathBuf>,
}

impl NetdCatalogEventSource {
    /// The production source (well-known socket, spawn-if-absent).
    pub fn new() -> Self {
        Self { socket: None }
    }

    /// A source pointed at an EXPLICIT socket (the test seam).
    pub fn with_socket(socket: std::path::PathBuf) -> Self {
        Self {
            socket: Some(socket),
        }
    }
}

impl Default for NetdCatalogEventSource {
    fn default() -> Self {
        Self::new()
    }
}

struct NetdCatalogStream {
    client: cerulion_netd::NetdEventClient,
}

impl CatalogEventStream for NetdCatalogStream {
    fn poll(&mut self) -> CatalogEventPoll {
        match self.client.next_event() {
            Ok(cerulion_netd::NextEvent::Changed(change)) => {
                CatalogEventPoll::Changed(Box::new(CatalogChangedEvent::from_netd(&change)))
            }
            Ok(cerulion_netd::NextEvent::Idle) => CatalogEventPoll::Idle,
            Err(e) => CatalogEventPoll::Disconnected(e.to_string()),
        }
    }
}

impl CatalogEventSource for NetdCatalogEventSource {
    fn subscribe(
        &self,
    ) -> Result<(CatalogSnapshot, Box<dyn CatalogEventStream>), SubscribeFailure> {
        let client = match &self.socket {
            Some(sock) => cerulion_netd::NetdClient::connect_or_spawn_at(sock.clone()),
            None => cerulion_netd::NetdClient::connect_or_spawn(),
        }
        .map_err(|e| SubscribeFailure::Transient(format!("cannot reach cerulion-netd: {e}")))?;
        // Classify PERMANENCE from the daemon's announced version, not
        // from the error variant. The per-verb gate refuses a too-old daemon before
        // anything is sent and surfaces a `Protocol` error — but so does an unparseable
        // response line or an unexpected response shape, and those are ordinary
        // transient faults that a reconnect can clear. Only the version is a property
        // of the daemon this desk is stuck with (netd is spawn-once and long-lived), so
        // only the version makes a refusal permanent.
        let needed = cerulion_netd::Request::SubscribeCatalog { id: 0 }.min_daemon_version();
        let daemon = client.daemon_protocol();
        if daemon < needed {
            return Err(SubscribeFailure::Unsupported(format!(
                "the connected cerulion-netd speaks protocol v{daemon} but catalog-change \
                 pushes need v{needed} — restart cerulion-netd (it is spawn-once, so a \
                 running old daemon stays old for this desk's lifetime)"
            )));
        }
        match client.subscribe_catalog() {
            Ok((snapshot, events)) => Ok((
                CatalogSnapshot {
                    version: snapshot.version,
                    robots: snapshot.robots,
                    watching: snapshot.watching,
                },
                Box::new(NetdCatalogStream { client: events }),
            )),
            Err(e) => Err(SubscribeFailure::Transient(e.to_string())),
        }
    }
}

/// A source that never subscribes — the hermetic default for tests that must not
/// touch (or spawn) a real daemon. It reports WHY, so a controller sees
/// `connected: false` rather than an unexplained silence.
pub struct NoCatalogEventSource;

impl CatalogEventSource for NoCatalogEventSource {
    fn subscribe(
        &self,
    ) -> Result<(CatalogSnapshot, Box<dyn CatalogEventStream>), SubscribeFailure> {
        Err(SubscribeFailure::Unsupported(
            "this daemon was started with no catalog-event source".to_string(),
        ))
    }
}

/// Run the forwarder: hold ONE upstream subscription and relay every change into
/// `hub`, re-subscribing across transient outages until `shutdown`.
///
/// Stops permanently on [`SubscribeFailure::Unsupported`] — a too-old netd cannot
/// grow the verb while this desk runs, so the right outcome is one loud line and a
/// desk that keeps its manual refresh, not an infinite retry loop.
pub fn run_forwarder(
    source: Arc<dyn CatalogEventSource>,
    hub: Arc<EventHub>,
    shutdown: Arc<AtomicBool>,
) {
    run_forwarder_with_backoff(source, hub, shutdown, RESUBSCRIBE_BACKOFF)
}

/// [`run_forwarder`] with an explicit retry backoff — the test seam. Production passes
/// the shipped `RESUBSCRIBE_BACKOFF`; a test that drives a failure regime past the
/// latch's first decade boundary passes a short one so it does not sleep for half a
/// minute.
pub fn run_forwarder_with_backoff(
    source: Arc<dyn CatalogEventSource>,
    hub: Arc<EventHub>,
    shutdown: Arc<AtomicBool>,
    backoff: Duration,
) {
    // The retry loop can fail FOREVER (no netd binary on PATH, a
    // socket the daemon cannot bind), and a bare `debug!` every 2 s is invisible at
    // the default level — an operator whose sidebar never updates gets no signal at
    // all unless they happen to raise the filter. The shared flood-suppression latch
    // gives it the repo's standard shape: the first failure of a regime is LOUD, the
    // repeats are `debug!` with a running suppressed count, the regime re-announces
    // itself at each decade so a permanent failure stays visible without flooding,
    // and a recovery says once how much was suppressed.
    let mut subscribe_latch = FailureRegimeLatch::new();
    while !shutdown.load(Ordering::Acquire) {
        let mut stream = match source.subscribe() {
            Ok((snapshot, stream)) => {
                // Adopt the upstream's state BEFORE the first poll — this is what a
                // controller subscribing to a just-restarted vizd is handed.
                hub.seed(&snapshot);
                if let Some(suppressed) = subscribe_latch.on_success() {
                    hub.note_subscribe_recovery();
                    tracing::info!(
                        suppressed,
                        "cerulion-vizd: cerulion-netd reachable again — catalog-change \
                         subscription recovered"
                    );
                }
                if snapshot.watching {
                    tracing::info!(
                        version = snapshot.version,
                        robots = snapshot.robots.len(),
                        "cerulion-vizd: subscribed to cerulion-netd catalog changes — the topic \
                         list now updates itself"
                    );
                } else {
                    // Subscribed, but the upstream has no announce space to watch. Say
                    // so — a controller that is told `connected: false` keeps its own
                    // refresh path instead of waiting on a push that cannot come.
                    tracing::info!(
                        version = snapshot.version,
                        robots = snapshot.robots.len(),
                        "cerulion-vizd: cerulion-netd accepted the catalog subscription but is \
                         NOT watching an announce space (no network plane) — no change will be \
                         pushed; controllers keep their own refresh path"
                    );
                }
                stream
            }
            Err(SubscribeFailure::Unsupported(msg)) => {
                hub.set_connected(false);
                tracing::info!(
                    reason = %msg,
                    "cerulion-vizd: catalog-change pushes are unavailable — controllers keep \
                     their own refresh path"
                );
                return;
            }
            Err(SubscribeFailure::Transient(msg)) => {
                hub.set_connected(false);
                let decision = subscribe_latch.on_failure();
                hub.note_subscribe_decision(&decision);
                match decision {
                    RegimeDecision::Loud => tracing::warn!(
                        reason = %msg,
                        "cerulion-vizd: cannot subscribe to cerulion-netd catalog changes — \
                         the topic list will not update itself until this clears (retrying; \
                         repeats log at debug)"
                    ),
                    RegimeDecision::StillFailing { total, suppressed } => tracing::warn!(
                        reason = %msg,
                        total_failures = total,
                        suppressed,
                        "cerulion-vizd: STILL cannot subscribe to cerulion-netd catalog changes"
                    ),
                    RegimeDecision::Suppressed { suppressed } => tracing::debug!(
                        reason = %msg,
                        suppressed,
                        "cerulion-vizd: catalog-change subscription still unavailable — retrying"
                    ),
                }
                sleep_until_shutdown(&shutdown, backoff);
                continue;
            }
        };

        while !shutdown.load(Ordering::Acquire) {
            match stream.poll() {
                CatalogEventPoll::Changed(event) => {
                    tracing::debug!(
                        version = event.version,
                        robots = event.robots.len(),
                        topics_added = event.topics_added,
                        topics_removed = event.topics_removed,
                        "cerulion-vizd: relaying a catalog change to controllers"
                    );
                    hub.publish(*event);
                }
                CatalogEventPoll::Idle => {}
                CatalogEventPoll::Disconnected(reason) => {
                    hub.set_connected(false);
                    // A drop is not itself a failure regime — the re-subscribe below
                    // decides that. A netd restart drops us and we reconnect at once;
                    // only a re-subscribe that keeps FAILING is worth being loud about,
                    // and that is the latch's job on the next iteration.
                    tracing::debug!(
                        %reason,
                        "cerulion-vizd: catalog-change subscription dropped — re-subscribing"
                    );
                    sleep_until_shutdown(&shutdown, backoff);
                    break;
                }
            }
        }
    }
    hub.set_connected(false);
}

/// Sleep in short slices so a shutdown is noticed promptly rather than after the whole
/// backoff.
fn sleep_until_shutdown(shutdown: &AtomicBool, total: Duration) {
    let slice = Duration::from_millis(50);
    let mut waited = Duration::ZERO;
    while waited < total {
        if shutdown.load(Ordering::Acquire) {
            return;
        }
        std::thread::sleep(slice);
        waited += slice;
    }
}

/// Spawn the forwarder thread. Returns its handle so shutdown can join it.
pub fn spawn_forwarder(
    source: Arc<dyn CatalogEventSource>,
    hub: Arc<EventHub>,
    shutdown: Arc<AtomicBool>,
) -> std::io::Result<JoinHandle<()>> {
    std::thread::Builder::new()
        .name("vizd-catalog-events".to_string())
        .spawn(move || run_forwarder(source, hub, shutdown))
}

/// Union two sorted name lists, deduped (the merge helper the notification uses).
pub(crate) fn union_names(a: &[String], b: &[String]) -> Vec<String> {
    let set: BTreeSet<&String> = a.iter().chain(b.iter()).collect();
    set.into_iter().cloned().collect()
}

/// Remove every name in `remove` from `from` (the net-membership helper).
pub(crate) fn without_names(from: &[String], remove: &[String]) -> Vec<String> {
    let drop: BTreeSet<&String> = remove.iter().collect();
    from.iter().filter(|n| !drop.contains(n)).cloned().collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn change(
        version: u64,
        added: &[&str],
        removed: &[&str],
        robots: &[&str],
    ) -> CatalogChangedEvent {
        CatalogChangedEvent {
            event: crate::protocol::CATALOG_CHANGED_EVENT.to_string(),
            version,
            robots: robots.iter().map(|s| s.to_string()).collect(),
            robots_added: added.iter().map(|s| s.to_string()).collect(),
            robots_removed: removed.iter().map(|s| s.to_string()).collect(),
            topics_added: 0,
            topics_removed: 0,
        }
    }

    /// A change reaches every subscriber and nobody else — the fan-out contract.
    #[test]
    fn a_change_reaches_every_subscriber_and_only_subscribers() {
        let hub = EventHub::new();
        let a = hub.mint_conn();
        let b = hub.mint_conn();
        let unsubscribed = hub.mint_conn();
        hub.subscribe(a);
        hub.subscribe(b);
        assert_eq!(hub.subscriber_count(), 2);

        hub.publish(change(1, &["go2"], &[], &["go2"]));
        assert_eq!(hub.take_pending(a).expect("a gets it").version, 1);
        assert_eq!(hub.take_pending(b).expect("b gets it").version, 1);
        assert!(
            hub.take_pending(unsubscribed).is_none(),
            "an unsubscribed connection receives nothing"
        );
        assert!(hub.take_pending(a).is_none(), "taking it drains the slot");
    }

    /// The bounded slot MERGES rather than dropping: a controller that missed a
    /// notification gets one line describing everything since, at the newest version.
    #[test]
    fn a_slow_controller_receives_one_merged_notification_not_a_queue() {
        let hub = EventHub::new();
        let c = hub.mint_conn();
        hub.subscribe(c);
        hub.publish(change(1, &["go2"], &[], &["go2"]));
        hub.publish(change(2, &["orin"], &[], &["go2", "orin"]));
        hub.publish(change(3, &[], &["go2"], &["orin"]));

        let merged = hub.take_pending(c).expect("one merged notification");
        assert_eq!(merged.version, 3, "the newest version");
        assert_eq!(merged.robots, vec!["orin".to_string()], "the newest state");
        assert_eq!(
            merged.robots_added,
            vec!["orin".to_string()],
            "go2 arrived then left inside the window — reporting it as added would send \
             a controller chasing a robot that is gone"
        );
        assert_eq!(merged.robots_removed, vec!["go2".to_string()]);
        assert!(hub.take_pending(c).is_none(), "exactly ONE notification");
    }

    /// Unsubscribe (a connection close) removes the slot, so a dead controller never
    /// accumulates notifications nobody drains.
    #[test]
    fn unsubscribing_removes_the_slot() {
        let hub = EventHub::new();
        let c = hub.mint_conn();
        hub.subscribe(c);
        hub.publish(change(1, &["go2"], &[], &["go2"]));
        hub.unsubscribe(c);
        assert_eq!(hub.subscriber_count(), 0);
        assert!(hub.take_pending(c).is_none());
        // A later publish does not resurrect it.
        hub.publish(change(2, &["orin"], &[], &["go2", "orin"]));
        assert!(hub.take_pending(c).is_none());
    }

    /// Connection ids are UNIQUE — two controllers must never share a push slot (one
    /// would eat the other's notification).
    #[test]
    fn minted_connection_ids_are_unique() {
        let hub = EventHub::new();
        let ids: BTreeSet<u64> = (0..100).map(|_| hub.mint_conn()).collect();
        assert_eq!(ids.len(), 100);
    }

    /// A controller subscribing during an upstream outage is told `connected: false`
    /// rather than left waiting on a push that cannot come.
    #[test]
    fn subscribe_reports_the_upstream_state_honestly() {
        let hub = EventHub::new();
        let c = hub.mint_conn();
        let (version, robots, connected) = hub.subscribe(c);
        assert_eq!(version, 0);
        assert!(robots.is_empty());
        assert!(!connected, "nothing has connected upstream yet");

        hub.set_connected(true);
        // The upstream's own number (4) is deliberately NOT what a controller is told —
        // this daemon stamps its own generation, which is 1 after one relayed change.
        hub.publish(change(4, &["go2"], &[], &["go2"]));
        let d = hub.mint_conn();
        let (version, robots, connected) = hub.subscribe(d);
        assert_eq!(
            version, 1,
            "a late subscriber is told THIS daemon's current generation, not the \
             upstream's"
        );
        assert_eq!(robots, vec!["go2".to_string()]);
        assert!(connected);

        hub.set_connected(false);
        let e = hub.mint_conn();
        assert!(
            !hub.subscribe(e).2,
            "an outage is reported, not hidden behind a stale `true`"
        );
    }

    /// The name-set helpers, against hand oracles — they are what makes the merge
    /// report NET membership rather than a contradiction.
    #[test]
    fn name_set_helpers_union_and_subtract() {
        let a = vec!["go2".to_string(), "orin".to_string()];
        let b = vec!["orin".to_string(), "zed".to_string()];
        assert_eq!(
            union_names(&a, &b),
            vec!["go2".to_string(), "orin".to_string(), "zed".to_string()]
        );
        assert_eq!(
            without_names(&a, &["orin".to_string()]),
            vec!["go2".to_string()]
        );
        assert_eq!(without_names(&a, &[]), a);
        assert!(without_names(&[], &["x".to_string()]).is_empty());
    }

    /// A one-shot source that hands back a scripted snapshot and then an immediately
    /// exhausted stream, so the forwarder seeds and returns without threads or sleeps.
    ///
    /// Its stream RECORDS `hub.is_connected()` on its first poll — i.e. at the moment
    /// the subscription is live, which is when a controller would ask. Reading the hub
    /// AFTER `run_forwarder` returns would be vacuous: the loop always ends
    /// disconnected, so code reporting `connected: true` while subscribed would
    /// pass unnoticed on a post-return read.
    struct OneShotSource {
        snapshot: CatalogSnapshot,
        attempts: Mutex<usize>,
        hub: Arc<EventHub>,
        /// `is_connected()` observed WHILE the subscription was live.
        connected_while_live: Arc<Mutex<Option<bool>>>,
    }

    impl OneShotSource {
        fn new(snapshot: CatalogSnapshot, hub: &Arc<EventHub>) -> Self {
            Self {
                snapshot,
                attempts: Mutex::new(0),
                hub: Arc::clone(hub),
                connected_while_live: Arc::new(Mutex::new(None)),
            }
        }

        /// What the hub reported while the subscription was live (`None` if it never
        /// got that far).
        fn connected_while_live(&self) -> Option<bool> {
            *self.connected_while_live.lock().unwrap()
        }
    }

    struct ExhaustedStream {
        hub: Arc<EventHub>,
        observed: Arc<Mutex<Option<bool>>>,
    }

    impl CatalogEventStream for ExhaustedStream {
        fn poll(&mut self) -> CatalogEventPoll {
            *self.observed.lock().unwrap() = Some(self.hub.is_connected());
            CatalogEventPoll::Disconnected("one-shot".to_string())
        }
    }

    impl CatalogEventSource for OneShotSource {
        fn subscribe(
            &self,
        ) -> Result<(CatalogSnapshot, Box<dyn CatalogEventStream>), SubscribeFailure> {
            let mut a = self.attempts.lock().unwrap();
            *a += 1;
            if *a == 1 {
                Ok((
                    self.snapshot.clone(),
                    Box::new(ExhaustedStream {
                        hub: Arc::clone(&self.hub),
                        observed: Arc::clone(&self.connected_while_live),
                    }),
                ))
            } else {
                // Second attempt ends the loop without a backoff sleep.
                Err(SubscribeFailure::Unsupported("one-shot done".to_string()))
            }
        }
    }

    /// Snapshot seeding, half 1: THE `watching: false` pin. An upstream that ACCEPTED the
    /// subscription but cannot watch anything (a `CERULION_NETD_NETWORK=off` netd, or
    /// one whose watch declaration failed) must be reported to controllers as
    /// `connected: false`.
    ///
    /// A forwarder that discards the snapshot and calls `set_connected(true)`
    /// unconditionally tells every controller a push is coming when
    /// netd has already said none can — the exact silent failure the `watching` flag
    /// exists to prevent, and one a stream-only seam cannot even EXPRESS (the source
    /// returns only a stream).
    #[test]
    fn an_upstream_that_is_not_watching_is_reported_as_disconnected() {
        let hub = Arc::new(EventHub::new());
        let source = Arc::new(OneShotSource::new(
            CatalogSnapshot {
                version: 4,
                robots: vec!["go2".to_string()],
                watching: false,
            },
            &hub,
        ));
        run_forwarder(
            Arc::clone(&source) as Arc<dyn CatalogEventSource>,
            Arc::clone(&hub),
            Arc::new(AtomicBool::new(false)),
        );
        assert_eq!(
            source.connected_while_live(),
            Some(false),
            "netd said watching:false — a controller asking WHILE we are subscribed \
             must not be told a push is coming"
        );

        // ANTI-TAUTOLOGY: the identical shape with watching:true reports connected at
        // the same observation point, so the assertion above is about the FLAG and not
        // about the observation point happening to be false.
        let hub2 = Arc::new(EventHub::new());
        let watching = Arc::new(OneShotSource::new(
            CatalogSnapshot {
                version: 4,
                robots: vec!["go2".to_string()],
                watching: true,
            },
            &hub2,
        ));
        run_forwarder(
            Arc::clone(&watching) as Arc<dyn CatalogEventSource>,
            Arc::clone(&hub2),
            Arc::new(AtomicBool::new(false)),
        );
        assert_eq!(
            watching.connected_while_live(),
            Some(true),
            "a watching upstream must report connected"
        );
    }

    /// Snapshot seeding, half 2: THE restart pin. A forwarder that subscribes to a live
    /// upstream ADOPTS its catalog, so a controller connecting to a freshly-restarted
    /// vizd is handed the robots that already exist.
    ///
    /// Without adoption the hub starts empty and stays empty until the next CHANGE. netd's
    /// watch is already running by then and replays nothing, so on a desk whose robots
    /// are all up and steady the sidebar would show nothing indefinitely — the
    /// "re-subscribe recovers" behaviour the protocol documents would be inert.
    ///
    /// The existing fan-out test cannot catch this: it pre-populates the hub by
    /// PUSHING a change through it, which is the one path that was never broken.
    #[test]
    fn a_forwarder_seeds_the_hub_from_the_upstream_snapshot() {
        let hub = Arc::new(EventHub::new());
        // A controller that subscribes BEFORE anything is seeded sees the empty start.
        let early = hub.mint_conn();
        assert_eq!(hub.subscribe(early), (0, vec![], false));

        run_forwarder(
            Arc::new(OneShotSource::new(
                CatalogSnapshot {
                    version: 9,
                    robots: vec!["go2".to_string(), "orin".to_string()],
                    watching: true,
                },
                &hub,
            )),
            Arc::clone(&hub),
            Arc::new(AtomicBool::new(false)),
        );

        // The hub now KNOWS the catalog, with no change having been pushed at all.
        assert_eq!(
            hub.version(),
            9,
            "the upstream generation must be adopted — a controller compares against it"
        );
        let late = hub.mint_conn();
        let (version, robots, _connected) = hub.subscribe(late);
        assert_eq!(version, 9);
        assert_eq!(
            robots,
            vec!["go2".to_string(), "orin".to_string()],
            "a controller on a just-restarted vizd is handed the CURRENT robots, not an \
             empty list it would keep until the next change"
        );
    }

    /// THE netd-restart pin: the version a controller is handed must never
    /// go backwards, INCLUDING on the push that follows a reconnect to a restarted netd.
    ///
    /// The clamp in `seed` alone only half-fixed this: `publish` mirrored netd's number
    /// verbatim, so a controller handed version 40 by the snapshot saw the very next
    /// push carry 1 (measured: snapshot 40, next push 1) — contradicting the
    /// `SubscribeEventsResponse::version` contract Studio and any external consumer
    /// implement against. A consumer that skips a version not greater than the last it
    /// handled — which "compare it against the version on later events" invites — would
    /// then ignore every push until netd's generation climbed back past 40.
    #[test]
    fn a_netd_restart_never_walks_the_relayed_version_backwards() {
        let hub = EventHub::new();
        let conn = hub.mint_conn();
        hub.subscribe(conn);

        // A long-running netd: we have relayed up to generation 40.
        hub.seed(&CatalogSnapshot {
            version: 40,
            robots: vec!["go2".to_string()],
            watching: true,
        });
        assert_eq!(hub.version(), 40);

        // netd RESTARTS. Its own generation begins again — the next push it sends
        // carries a LOWER upstream number than the snapshot we already reported.
        hub.publish(change(1, &["orin"], &[], &["go2", "orin"]));
        let relayed = hub.take_pending(conn).expect("the change is relayed");
        assert!(
            relayed.version > 40,
            "the relayed version must exceed the one already reported, got {} — a \
             consumer that skips a non-greater version would ignore this change",
            relayed.version
        );
        assert_eq!(relayed.version, 41, "and it is exactly the next generation");
        assert_eq!(hub.version(), 41);

        // The upstream keeps counting from its own low base; ours keeps climbing.
        hub.publish(change(2, &["orin"], &[], &["orin"]));
        let next = hub.take_pending(conn).expect("relayed");
        assert_eq!(next.version, 42);
        assert!(
            next.version > relayed.version,
            "strictly increasing across the whole sequence"
        );
        assert_eq!(hub.version(), 42);
    }

    /// A reconnect to a RESTARTED upstream (whose generation begins again at 0) must
    /// not walk the version backwards — a controller uses it to decide whether it is
    /// current, and a rewind would make a stale view look fresh. The robot set is
    /// adopted regardless, since it is the authoritative present state either way.
    #[test]
    fn seeding_never_walks_the_version_backwards() {
        let hub = EventHub::new();
        hub.seed(&CatalogSnapshot {
            version: 7,
            robots: vec!["go2".to_string()],
            watching: true,
        });
        assert_eq!(hub.version(), 7);

        hub.seed(&CatalogSnapshot {
            version: 1,
            robots: vec!["orin".to_string()],
            watching: true,
        });
        assert_eq!(hub.version(), 7, "a restarted upstream must not rewind us");
        let conn = hub.mint_conn();
        assert_eq!(
            hub.subscribe(conn).1,
            vec!["orin".to_string()],
            "but the robot set is the new upstream's — it is the present state"
        );
    }

    /// The subscribe-retry ladder: a regime that keeps failing is
    /// LOUD once, then suppressed, then LOUD again at each decade, and its recovery is
    /// reported once.
    ///
    /// A bare `debug!` per retry is invisible at the default
    /// level, so a permanently-failing forwarder (no netd binary on PATH, an
    /// unbindable socket) would give an operator whose sidebar never updates no signal at
    /// all. The counters make the ladder observable independently of the log filter,
    /// which is the same Principle #3 argument the shared latch's own docs make.
    #[test]
    fn a_failing_subscribe_regime_is_loud_then_suppressed_then_loud_at_the_decade() {
        /// Fails transiently `fail_for` times, then succeeds once, then ends the loop.
        struct FlakySource {
            attempts: Mutex<usize>,
            fail_for: usize,
        }
        impl CatalogEventSource for FlakySource {
            fn subscribe(
                &self,
            ) -> Result<(CatalogSnapshot, Box<dyn CatalogEventStream>), SubscribeFailure>
            {
                let mut a = self.attempts.lock().unwrap();
                *a += 1;
                if *a <= self.fail_for {
                    return Err(SubscribeFailure::Transient(format!("attempt {a}")));
                }
                if *a == self.fail_for + 1 {
                    // One success — closes the regime and reports the recovery.
                    return Ok((
                        CatalogSnapshot {
                            watching: true,
                            ..Default::default()
                        },
                        Box::new(ExhaustedStreamNoProbe),
                    ));
                }
                Err(SubscribeFailure::Unsupported("done".to_string()))
            }
        }
        struct ExhaustedStreamNoProbe;
        impl CatalogEventStream for ExhaustedStreamNoProbe {
            fn poll(&mut self) -> CatalogEventPoll {
                CatalogEventPoll::Disconnected("one-shot".to_string())
            }
        }

        let hub = Arc::new(EventHub::new());
        // 12 failures crosses the shared latch's first decade boundary (10), which is
        // the arm that keeps a PERMANENT failure visible without flooding — and the one
        // a shorter drive would leave unreachable.
        run_forwarder_with_backoff(
            Arc::new(FlakySource {
                attempts: Mutex::new(0),
                fail_for: 12,
            }),
            Arc::clone(&hub),
            Arc::new(AtomicBool::new(false)),
            Duration::from_millis(1),
        );

        let counts = hub.subscribe_failure_counts();
        assert_eq!(
            counts.total(),
            12,
            "every failure is counted, unconditionally"
        );
        assert_eq!(counts.loud, 1, "exactly ONE loud head per regime");
        assert_eq!(
            counts.re_announced, 1,
            "the regime re-announces itself when the running total crosses 10 — without \
             it a permanent failure goes silent after the head"
        );
        assert_eq!(
            counts.suppressed, 10,
            "the rest are downgraded (12 = 1 head + 1 decade + 10 suppressed)"
        );
        assert_eq!(
            counts.recovered, 1,
            "and the recovery is reported once, so the operator learns it cleared"
        );
    }

    /// ANTI-TAUTOLOGY for the ladder: a forwarder that never fails counts NOTHING, so
    /// the arm above is about real failures and not about the loop running at all.
    #[test]
    fn a_healthy_forwarder_counts_no_subscribe_failures() {
        let hub = Arc::new(EventHub::new());
        run_forwarder_with_backoff(
            Arc::new(OneShotSource::new(
                CatalogSnapshot {
                    watching: true,
                    ..Default::default()
                },
                &hub,
            )),
            Arc::clone(&hub),
            Arc::new(AtomicBool::new(false)),
            Duration::from_millis(1),
        );
        assert_eq!(
            hub.subscribe_failure_counts(),
            SubscribeFailureCounts::default(),
            "a healthy desk logs and counts nothing"
        );
    }

    /// A forwarder pointed at an UNSUPPORTED source stops immediately (a too-old netd
    /// cannot grow the verb while this desk runs) and reports the outage rather than
    /// spinning forever.
    #[test]
    fn an_unsupported_source_stops_the_forwarder_instead_of_spinning() {
        let hub = Arc::new(EventHub::new());
        let shutdown = Arc::new(AtomicBool::new(false));
        let started = std::time::Instant::now();
        run_forwarder(
            Arc::new(NoCatalogEventSource),
            Arc::clone(&hub),
            Arc::clone(&shutdown),
        );
        assert!(
            started.elapsed() < RESUBSCRIBE_BACKOFF,
            "it must return at once, not after a backoff"
        );
        assert!(!hub.is_connected());
    }

    /// A scripted source drives the whole forwarder loop: changes reach the hub, a
    /// disconnect re-subscribes, and a shutdown ends it.
    #[test]
    fn the_forwarder_relays_changes_and_recovers_from_a_disconnect() {
        struct ScriptedStream {
            steps: Vec<CatalogEventPoll>,
        }
        impl CatalogEventStream for ScriptedStream {
            fn poll(&mut self) -> CatalogEventPoll {
                if self.steps.is_empty() {
                    return CatalogEventPoll::Disconnected("script exhausted".to_string());
                }
                self.steps.remove(0)
            }
        }
        struct ScriptedSource {
            attempts: Mutex<usize>,
        }
        impl CatalogEventSource for ScriptedSource {
            fn subscribe(
                &self,
            ) -> Result<(CatalogSnapshot, Box<dyn CatalogEventStream>), SubscribeFailure>
            {
                let mut a = self.attempts.lock().unwrap();
                *a += 1;
                let watching = CatalogSnapshot {
                    watching: true,
                    ..Default::default()
                };
                match *a {
                    1 => Ok((
                        watching,
                        Box::new(ScriptedStream {
                            steps: vec![
                                CatalogEventPoll::Idle,
                                CatalogEventPoll::Changed(Box::new(change(
                                    1,
                                    &["go2"],
                                    &[],
                                    &["go2"],
                                ))),
                                CatalogEventPoll::Disconnected("peer left".to_string()),
                            ],
                        }),
                    )),
                    2 => Ok((
                        watching,
                        Box::new(ScriptedStream {
                            steps: vec![CatalogEventPoll::Changed(Box::new(change(
                                2,
                                &["orin"],
                                &[],
                                &["go2", "orin"],
                            )))],
                        }),
                    )),
                    _ => Err(SubscribeFailure::Unsupported("done".to_string())),
                }
            }
        }

        let hub = Arc::new(EventHub::new());
        let conn = hub.mint_conn();
        hub.subscribe(conn);
        let shutdown = Arc::new(AtomicBool::new(false));
        let source = Arc::new(ScriptedSource {
            attempts: Mutex::new(0),
        });
        // The third subscribe attempt is Unsupported, so the loop returns on its own.
        run_forwarder(source, Arc::clone(&hub), Arc::clone(&shutdown));

        let merged = hub.take_pending(conn).expect("both changes relayed");
        assert_eq!(merged.version, 2, "the second change arrived too");
        assert_eq!(
            merged.robots_added,
            vec!["go2".to_string(), "orin".to_string()],
            "a re-subscribe after a disconnect does not lose the first change"
        );
        assert!(
            !hub.is_connected(),
            "the terminal Unsupported leaves the hub reporting disconnected"
        );
    }
}
