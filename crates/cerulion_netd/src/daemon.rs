// SPDX-License-Identifier: AGPL-3.0-only
//! The `cerulion-netd` runtime — the UDS NDJSON control server,
//! the per-connection demand/release dispatch, and the idle-watch thread that
//! flips self-exit.
//!
//! The runtime is DEPENDENCY-INJECTED with a [`MirrorPlane`]:
//! `main.rs` injects the production [`GatewayMirrorPlane`](crate::mirror::GatewayMirrorPlane)
//! (the one zenoh session), while the e2e drives an IN-PROCESS daemon over a temp
//! socket with a counting spy plane — no zenoh, parallel-safe.
//!
//! # Threads
//!
//! - **accept**: a nonblocking `accept` polled against the shutdown flag, one
//!   handler thread per consumer connection. Finished handles are reaped each pass.
//! - **connection** (one per consumer): reads NDJSON request lines, dispatches
//!   `demand`/`release`/`status` against the shared [`DemandRegistry`], writes each
//!   response. On EOF / error it disconnects — which RELEASES every demand that
//!   connection held (the crash-safe refcount).
//! - **idle-watch**: every `idle_watch_poll`, checks whether the registry has been
//!   idle (no connections) for the grace; on grace-elapsed it flips `self_exit`
//!   (main.rs then shuts the daemon down and the process exits — the
//!   self-exit lifecycle, which is by design).
//!
//! # The demand→mirror lock discipline
//!
//! The mirror ENSURE (on the first demand) runs UNDER the registry lock; the mirror
//! TEARDOWN (on the last release) runs its DECISION under the lock but its
//! (potentially BLOCKING) plane call OFF the lock:
//!
//! - A `demand` decides the refcount transition AND, on the first demand, calls
//!   `ensure_mirror` under the lock — so the FirstDemand→ensure→rollback sequence
//!   is race-free (a concurrent demand for the same key cannot observe a
//!   half-registered mirror). `ensure_mirror`'s lazy zenoh-session open is a
//!   one-time cost and does not block indefinitely, so holding the lock across it is
//!   acceptable.
//! - A `release`/disconnect teardown does NOT hold the registry lock across
//!   `release_mirror`. `release_mirror` is a REAL teardown — an
//!   `unregister_ingress_topic` that undeclares the zenoh subscriber + demand token
//!   and releases the mirror's iceoryx2 publisher slot — its zenoh undeclare can
//!   BLOCK on the real plane, and holding the registry lock across a blocking
//!   teardown would STARVE the idle-watch's self-exit check + every other connection's
//!   `connect` (the wedge: self-exit never fires, the socket stays bound,
//!   and new connections hang forever). So `process_released_key` re-checks
//!   `is_lingering` under the lock, drops it, tears the bridge down OFF the lock,
//!   then re-acquires to retire (re-checking `is_lingering`). The re-demand race is
//!   kept CLOSED for the common case by the pre-teardown re-check; the narrow
//!   residual (a re-demand landing DURING the off-lock teardown) is documented at
//!   `process_released_key` and is empty in the idle-self-exit path.
//!
//! (A teardown that FAILS returns `Lingering` instead of `Retired`, so the entry is
//! kept reusable rather than half-torn-down — the lingering-reuse path is the
//! failure fallback, not a designed end state.)

use std::io::{self, Read};
use std::os::unix::net::{UnixListener, UnixStream};
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex, MutexGuard, PoisonError};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use crate::catalog_events::{AnnounceWatchPlane, CatalogEventHub, NoopAnnounceWatchPlane};
use crate::egress::{EgressPlane, NoopEgressPlane};
use crate::hygiene::{acquire_socket, SocketGuard};
use crate::mirror::MirrorPlane;
use crate::protocol::{
    parse_request, CatalogQueryResponse, DemandEntry, DemandResponse, EgressReleaseResponse,
    EgressResponse, Hello, ReleaseResponse, Request, Response, RunsQueryResponse,
    SchemaQueryResponse, StatusResponse, SubscribeCatalogResponse, MAX_REQUEST_LINE_BYTES,
};
use crate::query::{NoopQueryPlane, QueryPlane};
use crate::registry::{
    ConnId, DemandOutcome, DemandRegistry, RegisterEgressOutcome, ReleaseOutcome, TopicKey,
};

/// The default idle grace before a consumer-less netd self-exits (~30 s, by
/// design). Overridable via [`resolve_idle_grace`] /
/// [`NetdConfig::idle_grace`].
pub const DEFAULT_IDLE_GRACE: Duration = Duration::from_secs(30);

/// The env var overriding the idle grace, in MILLISECONDS (ops + the e2e's short
/// grace). Unset / invalid falls back to [`DEFAULT_IDLE_GRACE`].
pub const IDLE_GRACE_ENV: &str = "CERULION_NETD_IDLE_GRACE_MS";

/// How often the idle-watch thread re-checks the idle grace.
const DEFAULT_IDLE_WATCH_POLL: Duration = Duration::from_millis(250);

/// How often the accept loop re-checks the shutdown flag between nonblocking
/// `accept`s (bounds shutdown latency without a busy spin).
const ACCEPT_POLL: Duration = Duration::from_millis(50);

/// Per-connection read timeout — bounds how long a connection thread blocks
/// before re-checking the shutdown flag, and is what PACES that loop:
/// an idle connection goes round it once per this interval, never faster. `pub` so
/// the e2e derives its iteration bound from the shipped value rather than a literal.
pub const CONN_READ_TIMEOUT: Duration = Duration::from_millis(200);

/// Per-connection write NO-PROGRESS budget — a consumer that stops reading (its
/// send buffer stays full so the daemon can place NO further bytes) for this long
/// is dropped rather than blocking a handler thread forever. It bounds the
/// *no-forward-progress* window, NOT a single `write` call: a live-but-slow reader
/// that keeps draining resets the budget on every byte placed (see [`write_line`]).
const CONN_WRITE_TIMEOUT: Duration = Duration::from_secs(2);

/// How long a `demand` for a topic that is mid-teardown WAITS (polling,
/// WITHOUT the registry lock) for the teardown to finish before returning a loud
/// busy error. Kept UNDER a client's per-request round-trip timeout (the netd
/// client's `ROUNDTRIP_TIMEOUT` is 5 s) so the daemon always answers before the
/// client gives up. When the teardown finishes the entry retires → the retried
/// demand is a fresh `FirstDemand` that re-creates a REAL mirror.
const DEMAND_TEARDOWN_WAIT: Duration = Duration::from_secs(4);

/// How often a `Retiring` demand re-checks whether the teardown has finished.
const DEMAND_TEARDOWN_POLL: Duration = Duration::from_millis(20);

/// Runtime configuration for [`start`].
#[derive(Debug, Clone)]
pub struct NetdConfig {
    /// Idle grace before self-exit (last consumer left → wait this long → exit).
    pub idle_grace: Duration,
    /// Idle-watch poll interval (kept short in tests).
    pub idle_watch_poll: Duration,
    /// How long announce transitions COALESCE before one catalog-change push
    /// (see [`crate::catalog_events::CHANGE_COALESCE_WINDOW`]). Configurable so a test
    /// can pin the burst-to-one-push contract without sleeping the production window.
    pub change_coalesce_window: Duration,
    /// Observable state (Principle #3): the zenoh connect endpoints this daemon is
    /// CONFIGURED TO DIAL — the explicit [`crate::net::CONNECT_ENV`] locators plus
    /// whatever the boot-time peer-cache fold added ([`crate::discovery_fold`]).
    /// Reported verbatim by `status` so an operator can see WHY a desk is talking
    /// to a particular address, instead of inferring it from logs.
    ///
    /// CONFIGURED, not connected: the set is fixed before `TransportManager::init`
    /// and the zenoh session is LAZY, so this never claims a link is live. Empty
    /// for an in-process/test daemon (nothing was folded), which is the correct
    /// answer for one.
    pub connect_endpoints: Vec<String>,
    /// Whether the idle-watch runs at all (default `true`, the
    /// self-exit lifecycle, by design). `main.rs` sets `false` when the machine is
    /// LISTEN-configured (`crate::net::standing_gateway_requested`): such a
    /// daemon hosts the STANDING egress gateway + mDNS beacon booted at start,
    /// which no refcounted demand holds open — an idle self-exit would withdraw
    /// the machine's whole network presence 30 s after the last local consumer
    /// left, and on a pure-ROS robot NOTHING local ever respawns netd (it is
    /// spawned by local consumers, which such a robot has none of). A standing
    /// daemon therefore runs until SIGINT/SIGTERM. INTENT-based, deliberately:
    /// it reflects the operator's LISTEN configuration, not the boot's success —
    /// a failed gateway boot leaves a loudly-warned daemon a later
    /// `register_egress` can heal, rather than a silent exit-and-never-respawn.
    pub idle_self_exit: bool,
    /// Production network daemons require prior login before a raw local
    /// registration can boot egress. Injected library daemons default to no gate.
    pub egress_login: crate::serving_login::EgressLoginPolicy,
    /// Recheck installed account identity every 250 ms. Production enables this;
    /// injected daemons opt in explicitly. The watch checks local state and
    /// retires stale readers; it never dials or fetches account-service data.
    pub account_identity_watch: bool,
}

impl Default for NetdConfig {
    fn default() -> Self {
        Self {
            idle_grace: DEFAULT_IDLE_GRACE,
            idle_watch_poll: DEFAULT_IDLE_WATCH_POLL,
            change_coalesce_window: crate::catalog_events::CHANGE_COALESCE_WINDOW,
            connect_endpoints: Vec::new(),
            idle_self_exit: true,
            egress_login: crate::serving_login::EgressLoginPolicy::default(),
            account_identity_watch: false,
        }
    }
}

/// Resolve the idle grace from a [`IDLE_GRACE_ENV`] value (already read from the
/// env): a positive integer number of milliseconds, else [`DEFAULT_IDLE_GRACE`].
/// A non-empty non-parseable / zero value warns LOUDLY (never silently uses a
/// surprising grace). Pure — oracle-tested.
pub fn resolve_idle_grace(env_value: Option<&str>) -> Duration {
    match env_value {
        None => DEFAULT_IDLE_GRACE,
        Some(v) if v.trim().is_empty() => DEFAULT_IDLE_GRACE,
        Some(v) => match v.trim().parse::<u64>() {
            Ok(ms) if ms > 0 => Duration::from_millis(ms),
            _ => {
                tracing::warn!(
                    value = v,
                    "cerulion-netd: {IDLE_GRACE_ENV} is set but not a positive integer (ms) — \
                     using the default {}s idle grace",
                    DEFAULT_IDLE_GRACE.as_secs()
                );
                DEFAULT_IDLE_GRACE
            }
        },
    }
}

/// Lock the registry, tolerating a poisoned mutex (a panicked handler thread must
/// never wedge the whole daemon — the state under the lock is plain data).
fn lock_registry(m: &Mutex<DemandRegistry>) -> MutexGuard<'_, DemandRegistry> {
    m.lock().unwrap_or_else(PoisonError::into_inner)
}

/// The shared per-connection context (cheap to clone — all `Arc`s).
#[derive(Clone)]
struct Ctx {
    registry: Arc<Mutex<DemandRegistry>>,
    plane: Arc<dyn MirrorPlane>,
    /// The egress plane — the machine's ONE network egress path (a
    /// shared embedded gateway over the SAME session the mirror plane owns), or the
    /// [`NoopEgressPlane`] for a mirror-only daemon.
    egress_plane: Arc<dyn EgressPlane>,
    egress_login: crate::serving_login::EgressLoginPolicy,
    /// The query plane — the CATALOG/SCHEMA query surface over the SAME
    /// session, or the [`NoopQueryPlane`] for a mirror-only daemon (an explicit
    /// no-network refusal the consumer degrades on).
    query_plane: Arc<dyn QueryPlane>,
    /// The daemon's self-exit flag. A connection handler consults it —
    /// UNDER the registry lock — before registering, so a connection that arrives
    /// AFTER the idle-watch has committed to self-exit is refused rather than
    /// served-then-dropped. See [`Ctx::connect_unless_exiting`].
    self_exit: Arc<AtomicBool>,
    /// The catalog-change PUSH plane — the per-connection push slots + the
    /// announce watch that fills them. The watch is started by the FIRST
    /// `subscribe_catalog`, so a daemon nobody subscribes to opens no extra
    /// subscription and behaves exactly as it did before.
    events: Arc<CatalogEventHub>,
    /// Observable state (Principle #3): how many times ANY connection handler has gone round
    /// its read loop since this daemon started. On an idle connection the loop is
    /// paced by `CONN_READ_TIMEOUT`, so this grows at ~5 Hz per connection; a
    /// regression that leaves the accepted socket nonblocking turns the same loop
    /// into a spin and this counter into millions per second. Read via
    /// [`RunningNetd::conn_read_loop_iterations`].
    conn_loop_iterations: Arc<AtomicU64>,
    /// Observable state (Principle #3): cumulative control-loop waits that ended
    /// because a connection's PUSH WAKER fired rather than the socket or the
    /// timeout. A LOWER bound load can delay but never fake — a timeout cannot
    /// increment it — which makes it the only load-safe evidence that the catalog
    /// push really is wake-driven rather than paced by `CONN_READ_TIMEOUT`.
    conn_push_wakes: Arc<AtomicU64>,
    /// Observable state (Principle #3): the HIGH-WATER MARK of wake bytes any single
    /// [`drain_waker`] call consumed, across every connection this daemon has served.
    ///
    /// It is the wake plane's BACKLOG observable, and the only thing that can see the
    /// difference between draining on every pass and draining only when the push arm
    /// happens to win: both end with an empty pipe, so a total-bytes counter agrees
    /// either way. A healthy daemon sits at 1-2; a drain-on-`Push`-only
    /// shape MEASURED 1396 on one connection under a readable socket, and any value
    /// that reaches a multiple of 64 wedges such a handler outright.
    conn_waker_max_drain: Arc<AtomicU64>,
    /// Observable state (Principle #3): the TOTAL wake bytes every [`drain_waker`] call has
    /// consumed, across every connection this daemon has served — the running sum
    /// beside `conn_waker_max_drain`'s running max.
    ///
    /// The max alone cannot see a drain that never RAN. It is a `fetch_max` over what
    /// a drain consumed, so on a design that drains only when the push arm wins, a
    /// connection whose socket stays readable throughout produces NO drains at all and
    /// the mark stays at its initial 0 — which satisfies any ceiling. The sum sees the
    /// same run from the other side: with `n` pokes written to a connection's pipe,
    /// draining at the loop top consumes all `n`, while draining only on the push arm
    /// consumes exactly the prefix that arm reached. Neither statistic implies the
    /// other — a large max says nothing about how much was left undrained, and a large
    /// sum says nothing about how it was split — which is why both are kept.
    conn_waker_total_drain: Arc<AtomicU64>,
    /// Observable state (Principle #3): the zenoh connect endpoints this daemon's session
    /// was configured with — explicit `CERULION_NETD_CONNECT` locators plus
    /// whatever the boot-time peer-cache fold added. Reported verbatim by
    /// `status`; empty for an in-process/test daemon, which folds nothing.
    connect_endpoints: Vec<String>,
}

impl Ctx {
    /// Register a new connection at `now` UNLESS the daemon has already committed
    /// to idle self-exit. Returns `None` when the exit is committed —
    /// this connection arrived too late (the socket is about to be / has been
    /// unlinked); the caller refuses it cleanly so the client re-classifies (no
    /// socket → respawn) and retries, rather than being served a `Hello` and then
    /// dropped mid-exchange when [`RunningNetd::shutdown`] flips the shutdown flag.
    ///
    /// This is ATOMIC with the idle-watch's commit because both go through the
    /// registry lock (the single serialization point): whichever grabs the lock
    /// first wins. A `connect` that beats the commit makes the registry BUSY, so
    /// the idle-watch's `should_self_exit` (also under the lock) then returns
    /// `false` and does NOT commit — the exit is CANCELLED and this connection is
    /// served. A commit that beats the `connect` flips `self_exit` under the lock,
    /// which this method then observes and refuses.
    fn connect_unless_exiting(&self) -> Option<ConnId> {
        let mut reg = lock_registry(&self.registry);
        if self.self_exit.load(Ordering::Acquire) {
            return None;
        }
        Some(reg.connect(Instant::now()))
    }

    /// Dispatch one request from connection `conn` into its response.
    fn dispatch(&self, conn: ConnId, req: Request) -> Response {
        match req {
            Request::ServingSchemaSnapshot { id } => {
                match self.egress_plane.serving_schema_snapshot() {
                    Ok(serving_schema) => Response::ServingSchemaSnapshot(
                        crate::protocol::ServingSchemaSnapshotResponse { id, serving_schema },
                    ),
                    Err(error) => Response::error(Some(id), error, None, None),
                }
            }
            Request::AccountAccess { id, action } => {
                match action.validate().and_then(|()| match &action {
                    crate::account_access::AccountAccessRequest::Install { snapshot } => self
                        .plane
                        .install_account_snapshot(snapshot, &mut lock_registry(&self.registry)),
                    _ => self.plane.account_access(&action),
                }) {
                    Ok(reply) if action.accepts(&reply) => {
                        Response::AccountAccess(crate::account_access::AccountAccessResponse {
                            id,
                            account_access: reply,
                        })
                    }
                    Ok(_) => Response::error(
                        Some(id),
                        "account controller returned a mismatched operation",
                        None,
                        None,
                    ),
                    Err(error) => Response::error(Some(id), error, None, None),
                }
            }
            Request::Demand {
                id,
                robot,
                topic,
                schema_hash,
            } => self.demand(conn, id, robot, topic, schema_hash),
            Request::Release { id, robot, topic } => self.release(conn, id, robot, topic),
            Request::Status { id } => self.status(id),
            Request::RegisterEgress {
                id,
                plan,
                schema_serving,
                ix_config_json,
            } => self.register_egress(conn, id, plan, schema_serving, ix_config_json),
            Request::ReleaseEgress { id } => self.release_egress(conn, id),
            Request::QueryCatalog { id, robot } => self.query_catalog(id, robot),
            Request::QuerySchema {
                id,
                robot,
                requested,
            } => self.query_schema(id, robot, requested),
            Request::QueryRuns { id, robot } => self.query_runs(id, robot),
            Request::SubscribeCatalog { id } => self.subscribe_catalog(conn, id),
        }
    }

    /// Subscribe this connection to catalog-change pushes and answer with the
    /// current snapshot. Starts the announce watch on the FIRST subscriber.
    ///
    /// Never fails: a daemon with no announce space to watch still registers the
    /// subscription and reports `watching: false`, so the consumer knows plainly that
    /// no push can arrive here and keeps its own refresh path — rather than being
    /// refused (which would be indistinguishable from an old daemon) or being told
    /// `true` and waiting forever.
    fn subscribe_catalog(&self, conn: ConnId, id: u64) -> Response {
        let (version, robots, watching) = self.events.subscribe(conn);
        Response::SubscribeCatalog(SubscribeCatalogResponse {
            id,
            version,
            robots,
            watching,
        })
    }

    /// Handle a `query_catalog` — run the LAN catalog GET over netd's
    /// ONE zenoh session via the query plane and return every decoded reply. STATELESS:
    /// no registry lock, no refcount, no mirror (a query creates nothing). A plane
    /// error (netd is not network-configured / the session could not open) becomes a
    /// structured [`Response::error`] the consumer LOUDLY degrades on; an empty reply
    /// list is a valid answer (netd reached the LAN, nobody answered), NOT an error.
    ///
    /// The plane also reports whether it had completed a DISCOVERY PASS,
    /// which rides out on the response so the consumer can tell "nobody has this"
    /// from "netd cannot answer yet" (the cold-start false not-found).
    fn query_catalog(&self, id: u64, robot: Option<String>) -> Response {
        match robot
            .as_deref()
            .map(crate::account_access::parse_robot_route)
            .transpose()
        {
            Err(error) => return Response::error(Some(id), error, robot, None),
            Ok(Some(Some(robot_id))) => {
                let action = crate::account_access::AccountAccessRequest::Catalog { robot_id };
                return match self.plane.account_access(&action) {
                    Ok(crate::account_access::AccountAccessReply::Catalog {
                        robot_id: returned_id,
                        mut catalog,
                    }) if returned_id == robot_id
                        && catalog.version
                            == cerulion_core::transport::cerulion_q::CATALOG_WIRE_VERSION =>
                    {
                        // Correlate the authenticated target by its stable route, not its
                        // mutable display name. The controller retains the original catalog.
                        catalog.robot = crate::account_access::robot_route(&robot_id);
                        Response::CatalogQuery(CatalogQueryResponse {
                            id,
                            catalogs: vec![catalog],
                            discovery: crate::protocol::DiscoveryState::Settled,
                            plane_unsettled_ms: None,
                        })
                    }
                    Ok(_) => Response::error(
                        Some(id),
                        "account catalog response did not match its request",
                        robot,
                        None,
                    ),
                    Err(error) => Response::error(Some(id), error, robot, None),
                };
            }
            Ok(_) => {}
        }
        match self.query_plane.query_catalog(robot.as_deref()) {
            Ok(gather) => Response::CatalogQuery(CatalogQueryResponse {
                id,
                catalogs: gather.catalogs,
                discovery: gather.discovery,
                // The plane's un-settled age, so the consumer's first-contact
                // wait is scoped to this DAEMON rather than re-paid per command.
                plane_unsettled_ms: gather.unsettled_for.map(|d| d.as_millis() as u64),
            }),
            Err(e) => {
                tracing::debug!(error = %e, "netd: query_catalog could not run");
                Response::error(Some(id), format!("catalog query failed: {e}"), robot, None)
            }
        }
    }

    /// Handle a `query_schema` — run the `schema`-closure GET over netd's
    /// ONE zenoh session via the query plane and return every decoded reply (found /
    /// not-found / refused; the consumer picks the first with docs). STATELESS (no
    /// lock, no refcount). A plane error becomes a structured error; an empty list is a
    /// valid "nobody answered".
    fn query_schema(&self, id: u64, robot: Option<String>, requested: String) -> Response {
        if requested.is_empty() {
            return Response::error(
                Some(id),
                "query_schema requires a non-empty requested type",
                robot,
                None,
            );
        }
        match robot
            .as_deref()
            .map(crate::account_access::parse_robot_route)
            .transpose()
        {
            Err(error) => return Response::error(Some(id), error, robot, None),
            Ok(Some(Some(robot_id))) => {
                return match self.plane.account_schema_by_type(robot_id, &requested) {
                    Ok(mut reply)
                        if reply.requested == requested
                            && reply.version
                                == cerulion_core::transport::cerulion_q::SCHEMA_WIRE_VERSION =>
                    {
                        reply.robot = crate::account_access::robot_route(&robot_id);
                        Response::SchemaQuery(SchemaQueryResponse {
                            id,
                            replies: vec![reply],
                            discovery: crate::protocol::DiscoveryState::Settled,
                            plane_unsettled_ms: None,
                        })
                    }
                    Ok(_) => Response::error(
                        Some(id),
                        "account schema response did not match its request",
                        robot,
                        None,
                    ),
                    Err(error) => Response::error(Some(id), error, robot, None),
                };
            }
            Ok(_) => {}
        }
        match self.query_plane.query_schema(robot.as_deref(), &requested) {
            Ok(gather) => Response::SchemaQuery(SchemaQueryResponse {
                id,
                replies: gather.replies,
                // Carried so a cold-start empty is distinguishable from a
                // settled "nobody serves this type".
                discovery: gather.discovery,
                // See the catalog twin.
                plane_unsettled_ms: gather.unsettled_for.map(|d| d.as_millis() as u64),
            }),
            Err(e) => {
                tracing::debug!(requested = %requested, error = %e, "netd: query_schema could not run");
                Response::error(
                    Some(id),
                    format!("schema query for '{requested}' failed: {e}"),
                    robot,
                    None,
                )
            }
        }
    }

    /// Handle a `query_runs`: run the live-runs GET over netd's ONE
    /// zenoh session via the query plane and return every decoded reply. STATELESS:
    /// no registry lock, no refcount, no mirror. A plane error (netd is not
    /// network-configured / the session could not open) becomes a structured
    /// [`Response::error`]; an empty reply list is a valid answer, NOT an error.
    ///
    /// The empty answer means "no robot ANSWERED", which is weaker than "no robot is
    /// running anything" in two ways the consumer must keep apart: `discovery` says
    /// whether the LAN was searched at all, and a robot that answered
    /// nothing is absent from the list rather than present with an empty run set.
    fn query_runs(&self, id: u64, robot: Option<String>) -> Response {
        match robot
            .as_deref()
            .map(crate::account_access::parse_robot_route)
            .transpose()
        {
            Err(error) => return Response::error(Some(id), error, robot, None),
            Ok(Some(Some(_))) => {
                return Response::error(
                    Some(id),
                    "live run queries are not supported over account robot access",
                    robot,
                    None,
                )
            }
            Ok(_) => {}
        }
        match self.query_plane.query_runs(robot.as_deref()) {
            Ok(gather) => Response::RunsQuery(RunsQueryResponse {
                id,
                run_replies: gather.replies,
                unusable: gather.unusable,
                // The COVERAGE half: which asked robots stayed
                // silent. Without it a consumer cannot tell a complete answer from
                // a partial one and folds either into a settled absence.
                silent: gather.silent,
                discovery: gather.discovery,
                // See the catalog twin.
                plane_unsettled_ms: gather.unsettled_for.map(|d| d.as_millis() as u64),
            }),
            Err(e) => {
                tracing::debug!(error = %e, "netd: query_runs could not run");
                Response::error(Some(id), format!("runs query failed: {e}"), robot, None)
            }
        }
    }

    /// Handle a `demand`. Holds the registry lock across the (fallible)
    /// `ensure_mirror` so the FirstDemand→ensure→rollback is race-free. A
    /// demand for a topic that is mid-teardown (`Retiring`) WAITS (bounded, without
    /// the lock) for the teardown to finish, then re-demands into a FRESH mirror —
    /// so a same-key re-demand landing during the off-lock teardown never acks a
    /// phantom-held mirror on a slot about to be released.
    fn demand(
        &self,
        conn: ConnId,
        id: u64,
        robot: String,
        topic: String,
        schema_hash: u64,
    ) -> Response {
        if let Err(error) = crate::account_access::parse_robot_route(&robot) {
            return Response::error(Some(id), error, Some(robot), Some(topic));
        }
        let key = TopicKey::new(&robot, &topic);
        if key.robot.is_empty() || key.topic.is_empty() {
            return Response::error(
                Some(id),
                "demand requires a non-empty robot and topic",
                Some(robot),
                Some(topic),
            );
        }

        let deadline = Instant::now() + DEMAND_TEARDOWN_WAIT;
        loop {
            let mut reg = lock_registry(&self.registry);
            if let Err(error) = self.plane.prepare_demand(&key, &mut reg) {
                return Response::error(Some(id), error, Some(key.robot), Some(key.topic));
            }
            match reg.demand(conn, key.clone(), schema_hash, Instant::now()) {
                DemandOutcome::FirstDemand { refcount } => {
                    // Register the shared mirror while STILL holding the lock (race-free
                    // rollback + mark). On success, RECORD the bridge as present so the
                    // entry LINGERS (reusable) past its last release rather than being
                    // removed. On failure, roll the just-added demand back so a failed
                    // mirror leaves no phantom refcount + no phantom lingering entry.
                    return match self.plane.ensure_mirror(&key, schema_hash) {
                        Ok(()) => {
                            reg.mark_mirror_present(&key);
                            Response::Demand(DemandResponse {
                                id,
                                robot: key.robot,
                                topic: key.topic,
                                refcount,
                                mirror_created: true,
                            })
                        }
                        Err(e) => {
                            reg.release(conn, &key, Instant::now());
                            tracing::warn!(
                                robot = %key.robot,
                                topic = %key.topic,
                                error = %e,
                                "netd: mirror registration failed — demand refused, refcount rolled back"
                            );
                            Response::error(
                                Some(id),
                                format!("mirror registration failed: {e}"),
                                Some(key.robot),
                                Some(key.topic),
                            )
                        }
                    };
                }
                DemandOutcome::AlreadyMirrored { refcount } => {
                    return Response::Demand(DemandResponse {
                        id,
                        robot: key.robot,
                        topic: key.topic,
                        refcount,
                        mirror_created: false,
                    })
                }
                DemandOutcome::SchemaConflict {
                    existing,
                    requested,
                } => {
                    return Response::error(
                        Some(id),
                        format!(
                            "schema conflict for topic '{}': already mirrored with schema hash \
                             {existing:#018x}, this demand requested {requested:#018x} (one data \
                             source = one topic = one schema)",
                            key.topic
                        ),
                        Some(key.robot),
                        Some(key.topic),
                    )
                }
                DemandOutcome::Retiring => {
                    // The topic's previous mirror is being torn down (off-lock). DROP
                    // the lock and WAIT for the teardown to finish, then retry — the
                    // retired entry re-classifies as a fresh FirstDemand → a REAL new
                    // mirror. If the teardown outlives the bound (a wedged zenoh
                    // undeclare), refuse with a loud busy error rather than hang.
                    drop(reg);
                    if Instant::now() >= deadline {
                        tracing::warn!(
                            robot = %key.robot,
                            topic = %key.topic,
                            wait_secs = DEMAND_TEARDOWN_WAIT.as_secs(),
                            "netd: demand for a topic still mid-teardown after the wait bound — \
                             refused (retry)"
                        );
                        return Response::error(
                            Some(id),
                            format!(
                                "topic '{}' is mid-teardown (its previous mirror has not finished \
                                 releasing within {}s) — retry the demand",
                                key.topic,
                                DEMAND_TEARDOWN_WAIT.as_secs()
                            ),
                            Some(key.robot),
                            Some(key.topic),
                        );
                    }
                    std::thread::sleep(DEMAND_TEARDOWN_POLL);
                }
                DemandOutcome::EgressConflict { topic } => {
                    // Cross-plan loop guard: this topic is produced +
                    // ANNOUNCED for egress by a local graph on the shared session
                    // (its bridge flag persists until netd idle-exits, even after the
                    // producing graph released it), so mirroring it IN would echo-loop
                    // (the ingress loop-exclusion rule). Refuse loudly + EXPLICITLY (rather than passing the
                    // guard and dying late in create_ingress_publisher on the flag).
                    return Response::error(
                        Some(id),
                        format!(
                            "demand refused: topic '{topic}' is produced + ANNOUNCED for egress by \
                             a local graph on this machine's shared session (the announce persists \
                             until netd idle-exits), so mirroring it IN would echo-loop. \
                             It is a LOCAL topic here — read it directly, not as a remote mirror"
                        ),
                        Some(key.robot),
                        Some(topic),
                    );
                }
                DemandOutcome::UnknownConnection => {
                    return Response::error(Some(id), "internal: unknown connection", None, None)
                }
            }
        }
    }

    /// Handle a `release`. Decrements the refcount; on the LAST release drives the
    /// REAL mirror-plane teardown (`release_mirror` → `Retired`, retires the entry;
    /// `Lingering` only on teardown failure → kept for reuse) via
    /// [`process_released_key`], which runs the (potentially BLOCKING) teardown
    /// WITHOUT the registry lock held (a blocking zenoh undeclare must
    /// never stall the idle-watch or a new connection).
    fn release(&self, conn: ConnId, id: u64, robot: String, topic: String) -> Response {
        if let Err(error) = crate::account_access::parse_robot_route(&robot) {
            return Response::error(Some(id), error, Some(robot), Some(topic));
        }
        let key = TopicKey::new(&robot, &topic);
        let mut reg = lock_registry(&self.registry);
        match reg.release(conn, &key, Instant::now()) {
            ReleaseOutcome::LastRelease => {
                // The entry now has refcount 0 (lingering). DROP the registry lock
                // BEFORE the mirror teardown: `release_mirror`'s zenoh undeclare can
                // BLOCK on the real plane, and holding the registry lock across it
                // would starve the idle-watch's self-exit check + every other
                // connection (the wedge). `process_released_key` re-checks
                // is_lingering under the lock, tears down off-lock, then retires.
                drop(reg);
                process_released_key(&self.registry, self.plane.as_ref(), &key);
                Response::Release(ReleaseResponse {
                    id,
                    robot: key.robot,
                    topic: key.topic,
                    refcount: 0,
                    last_release: true,
                })
            }
            ReleaseOutcome::StillDemanded { refcount } => Response::Release(ReleaseResponse {
                id,
                robot: key.robot,
                topic: key.topic,
                refcount,
                last_release: false,
            }),
            ReleaseOutcome::NotHeld => Response::error(
                Some(id),
                "release for a topic this connection did not demand",
                Some(key.robot),
                Some(key.topic),
            ),
            ReleaseOutcome::UnknownConnection => {
                Response::error(Some(id), "internal: unknown connection", None, None)
            }
        }
    }

    /// Handle a `status` — a snapshot of the demand table (Principle #3).
    fn status(&self, id: u64) -> Response {
        let reg = lock_registry(&self.registry);
        let demands = reg
            .snapshot()
            .into_iter()
            .map(|(k, refcount)| DemandEntry {
                robot: k.robot,
                topic: k.topic,
                refcount,
            })
            .collect();
        Response::Status(StatusResponse {
            id,
            demands,
            active_connections: reg.active_connections(),
            idle: reg.is_idle(),
            // Always `Some` from a daemon that HAS the field — an empty
            // vec is the explicit "nothing was folded", distinct from an older
            // daemon's absent field (which decodes to `None`).
            connect_endpoints: Some(self.connect_endpoints.clone()),
        })
    }

    /// Handle a `register_egress`. The registry records the plan's
    /// produced topics for this connection AND enforces the cross-plan loop guard
    /// (refuse a topic mirrored IN) UNDER the lock; then the egress plane boots/feeds
    /// the shared gateway OFF the lock (its first-boot spawns a thread + opens the
    /// session — never held across the registry lock, so a producing graph can never
    /// starve the idle-watch or another connection — the wedge class). On a
    /// plane failure the just-added topics are rolled back precisely.
    fn register_egress(
        &self,
        conn: ConnId,
        id: u64,
        plan: cerulion_core::GatewayPlan,
        serving: cerulion_core::SchemaServing,
        // The producing run's forwarded iceoryx2 namespace (`None` for a
        // monolith that shares netd's namespace; `Some` for a multi-process run whose
        // supervisor minted the shared worker namespace). The plane VERIFIES a `Some`
        // config matches netd's shared session before tapping.
        ix_config_json: Option<String>,
    ) -> Response {
        if plan.announce.is_empty() {
            return Response::error(
                Some(id),
                "register_egress requires a non-empty announce set (the produced topics to \
                 egress)",
                None,
                None,
            );
        }
        // Read local policy before any registry mutation or lazy gateway boot.
        // This is the production dispatch boundary, not a pure plane concern.
        if let Err(reason) = self.egress_login.check() {
            return Response::error(Some(id), reason, None, None);
        }
        // Step 1: record + loop-guard UNDER the lock.
        let (added, total) = {
            let mut reg = lock_registry(&self.registry);
            match reg.register_egress(conn, plan.announce.clone()) {
                RegisterEgressOutcome::Registered { added, total } => (added, total),
                RegisterEgressOutcome::LoopConflict { topic, robot } => {
                    return Response::error(
                        Some(id),
                        format!(
                            "egress registration refused: topic '{topic}' is currently MIRRORED IN \
                             from robot '{robot}' on this machine's shared session, so announcing \
                             it OUT would echo-loop. Stop consuming '{topic}' as a remote \
                             topic, or produce under a different name"
                        ),
                        Some(robot),
                        Some(topic),
                    );
                }
                RegisterEgressOutcome::UnknownConnection => {
                    return Response::error(Some(id), "internal: unknown connection", None, None);
                }
            }
        };
        // Step 2: boot/feed the shared gateway OFF the lock; roll back on failure.
        // The plane verifies the forwarded namespace matches netd's shared
        // session before tapping (a mismatch → refuse → the run uses a per-run child).
        match self
            .egress_plane
            .register_egress(conn, &plan, &serving, ix_config_json.as_deref())
        {
            Ok(gateway_started) => {
                // Step 3 (the mark_mirror_present analogue): record the now-set
                // physical announce so a later release keeps the demand-side loop
                // guard correct (the bridge flag is ADD-ONLY — it lingers until
                // process-exit, so a demand for a topic egressed this session must
                // still be refused).
                lock_registry(&self.registry).mark_egress_announced(&added);
                Response::Egress(EgressResponse {
                    id,
                    registered_topics: total,
                    gateway_started,
                })
            }
            Err(e) => {
                lock_registry(&self.registry).rollback_egress(conn, &added);
                tracing::warn!(
                    error = %e,
                    "netd: egress plan registration failed — refused, egress accounting rolled back"
                );
                Response::error(
                    Some(id),
                    format!("egress plan registration failed: {e}"),
                    None,
                    None,
                )
            }
        }
    }

    /// Handle a `release_egress`. Drops this connection's egress
    /// accounting UNDER the lock, then drives the plane's release OFF the lock. A
    /// connection that held no egress releases 0 (a benign no-op).
    fn release_egress(&self, conn: ConnId, id: u64) -> Response {
        let released = { lock_registry(&self.registry).release_egress(conn) };
        if !released.is_empty() {
            self.egress_plane.release_egress(conn);
        }
        Response::EgressRelease(EgressReleaseResponse {
            id,
            released_topics: released.len(),
        })
    }
}

/// A running `cerulion-netd` daemon: the shutdown + self-exit flags, the accept /
/// idle / connection threads, the shared registry, and the socket cleanup guard.
/// Dropping it (or [`shutdown`](Self::shutdown)) stops every thread and removes the
/// socket + pidfile.
#[derive(Debug)]
pub struct RunningNetd {
    shutdown: Arc<AtomicBool>,
    self_exit: Arc<AtomicBool>,
    accept_handle: Option<JoinHandle<()>>,
    idle_handle: Option<JoinHandle<()>>,
    identity_handle: Option<JoinHandle<()>>,
    conns: Arc<Mutex<Vec<JoinHandle<()>>>>,
    registry: Arc<Mutex<DemandRegistry>>,
    /// The socket-cleanup guard, SHARED with the idle-watch thread: the
    /// idle-watch drops it AT the self-exit commit (unlinking the socket + pidfile +
    /// releasing the flock the instant the daemon decides to exit), so a racing
    /// client finds no socket → `is_not_running` → respawns rather than binding to a
    /// dying daemon. [`RunningNetd::shutdown`] takes whatever is left (idempotent —
    /// `None` if the idle-watch already dropped it on the self-exit path).
    guard: Arc<Mutex<Option<SocketGuard>>>,
    socket_path: PathBuf,
    /// The catalog-change push hub (observability + the watch-thread join at
    /// shutdown).
    events: Arc<CatalogEventHub>,
    /// The connection read-loop iteration counter (Principle #3) — see
    /// [`Ctx::conn_loop_iterations`].
    conn_loop_iterations: Arc<AtomicU64>,
    /// See [`Ctx::conn_push_wakes`].
    conn_push_wakes: Arc<AtomicU64>,
    /// See [`Ctx::conn_waker_max_drain`].
    conn_waker_max_drain: Arc<AtomicU64>,
    /// See [`Ctx::conn_waker_total_drain`].
    conn_waker_total_drain: Arc<AtomicU64>,
}

impl RunningNetd {
    /// Observable state (Principle #3): total connection read-loop iterations across every
    /// handler since this daemon started. An IDLE connection contributes one per
    /// `CONN_READ_TIMEOUT` (~5/s) because its `read` blocks for that long; if the
    /// accepted socket were left nonblocking the same loop would spin and this would
    /// climb by millions per second. The oracle for that difference is an UPPER
    /// bound over a held-idle window — see `daemon_e2e_test`.
    pub fn conn_read_loop_iterations(&self) -> u64 {
        self.conn_loop_iterations.load(Ordering::Relaxed)
    }

    /// Observable state (Principle #3): cumulative control-loop waits ended by a
    /// connection's PUSH WAKER — the no-inert-shipping observable for the
    /// wake-driven catalog push. Zero on a daemon nobody subscribes to, and zero
    /// on a daemon whose pushes are all still paced by `CONN_READ_TIMEOUT`.
    pub fn conn_push_wakes(&self) -> u64 {
        self.conn_push_wakes.load(Ordering::Relaxed)
    }

    /// Observable state (Principle #3): the largest wake-byte backlog any single
    /// waker drain has consumed, across every connection this daemon has served.
    ///
    /// The oracle for "the waker is drained on every pass", which no total-bytes
    /// counter can express: draining at the loop top and draining only when the push
    /// arm wins both end with an empty pipe. A healthy daemon sits at 1-2; a
    /// drain-on-`Push`-only shape MEASURED 1396 on one connection under a
    /// continuously-readable socket, and any value reaching a multiple of the 64-byte
    /// drain sink wedges such a handler outright.
    pub fn conn_waker_max_drain(&self) -> u64 {
        self.conn_waker_max_drain.load(Ordering::Relaxed)
    }

    /// Observable state (Principle #3): the TOTAL wake bytes drained across every connection
    /// this daemon has served — the running SUM beside `conn_waker_max_drain`'s
    /// running max.
    ///
    /// The companion to the high-water mark, and it answers the question the mark
    /// structurally cannot: how much of what was POKED was ever consumed. A drain that
    /// never runs leaves the mark at 0 and passes any ceiling; it leaves this at 0 too,
    /// against a poke count the caller knows.
    pub fn conn_waker_total_drain(&self) -> u64 {
        self.conn_waker_total_drain.load(Ordering::Relaxed)
    }

    /// Observable state (Principle #3): how many connections currently hold a push
    /// waker — see [`CatalogEventHub::waker_count`]. The leak observable for
    /// `cleanup_connection`'s waker release.
    pub fn catalog_waker_count(&self) -> usize {
        self.events.waker_count()
    }

    /// How many connections are subscribed to catalog-change pushes
    /// (Principle #3 / tests).
    pub fn catalog_subscriber_count(&self) -> usize {
        self.events.subscriber_count()
    }

    /// The announce-view generation — bumped once per published change.
    pub fn catalog_version(&self) -> u64 {
        self.events.version()
    }

    /// Whether the announce watch is running (started by the first
    /// `subscribe_catalog`; never started on a daemon with no network plane).
    pub fn is_watching_announces(&self) -> bool {
        self.events.is_watching()
    }

    /// The bound control-socket path (for logging / a consumer to connect to).
    pub fn socket_path(&self) -> &Path {
        &self.socket_path
    }

    /// Whether the idle-watch thread has requested self-exit (idle for the grace).
    /// `main.rs` polls this to drive the self-exit; the e2e asserts on it.
    pub fn self_exit_requested(&self) -> bool {
        self.self_exit.load(Ordering::Acquire)
    }

    /// The number of live consumer connections (Principle #3 / tests).
    pub fn active_connections(&self) -> usize {
        lock_registry(&self.registry).active_connections()
    }

    /// The number of physical mirror bridges — ACTIVE plus LINGERING (Principle #3
    /// / tests). A released mirror whose teardown failed lingers (reused on re-demand) until idle
    /// self-exit, so this can exceed [`Self::active_demand_count`].
    pub fn mirror_count(&self) -> usize {
        lock_registry(&self.registry).mirror_count()
    }

    /// The number of ACTIVELY-demanded mirrors (refcount > 0). Tests / Principle #3.
    pub fn active_demand_count(&self) -> usize {
        lock_registry(&self.registry).active_demand_count()
    }

    /// The number of LINGERING mirrors (refcount 0, bridge kept — the
    /// leaked-until-idle set). Tests / Principle #3.
    pub fn lingering_count(&self) -> usize {
        lock_registry(&self.registry).lingering_count()
    }

    /// Whether netd is currently idle (no live connections) — the idle-grace timer
    /// toward self-exit is running.
    pub fn is_idle(&self) -> bool {
        lock_registry(&self.registry).is_idle()
    }

    /// Stop the daemon: unlink the control socket + pidfile FIRST, flip the
    /// shutdown flag, then join the accept + idle threads and every connection
    /// thread. Idempotent.
    ///
    /// The socket-cleanup [`SocketGuard`] is dropped FIRST (before any
    /// `join`), NOT last. A client that connects during teardown then finds no
    /// socket, classifies `is_not_running`, and respawns a FRESH daemon — it never
    /// binds to a half-torn-down zombie. Were the guard dropped LAST, a
    /// teardown that BLOCKED at a `join` (a wedged real-plane zenoh/mirror teardown
    /// running on a thread this joins) would leave the socket bound FOREVER and every new
    /// client would connect to the wedged process — the idle-exit wedge. On the
    /// idle self-exit path the idle-watch already dropped the guard AT the commit, so
    /// this `take()` is a no-op then; on the signal path this is where it drops. The
    /// production entry (`main.rs`) additionally arms a bounded hard-exit watchdog
    /// around this call, so even a `join` that never returns cannot leave a zombie.
    pub fn shutdown(&mut self) {
        // Unlink the socket + pidfile BEFORE anything that can block, so new
        // clients respawn instead of binding to us mid-teardown (idempotent — the
        // idle-watch may have already dropped the guard at the self-exit commit).
        lock_guard(&self.guard).take();
        self.shutdown.store(true, Ordering::SeqCst);
        if let Some(h) = self.accept_handle.take() {
            let _ = h.join();
        }
        if let Some(h) = self.idle_handle.take() {
            let _ = h.join();
        }
        if let Some(handle) = self.identity_handle.take() {
            let _ = handle.join();
        }
        // The announce-watch thread observes the same shutdown flag; join it
        // here so a stopped daemon leaves no thread holding a zenoh subscription.
        self.events.shutdown();
        let handles = std::mem::take(&mut *lock_conns(&self.conns));
        for h in handles {
            let _ = h.join();
        }
    }
}

impl Drop for RunningNetd {
    fn drop(&mut self) {
        self.shutdown();
    }
}

/// Lock the connection-handle vec (poison-tolerant).
fn lock_conns(m: &Mutex<Vec<JoinHandle<()>>>) -> MutexGuard<'_, Vec<JoinHandle<()>>> {
    m.lock().unwrap_or_else(PoisonError::into_inner)
}

/// Lock the shared socket-cleanup guard slot (poison-tolerant). The idle-watch and
/// [`RunningNetd::shutdown`] both `take()` through this — whoever gets there first
/// drops the [`SocketGuard`] (unlinks the socket + pidfile, releases the flock).
fn lock_guard(m: &Mutex<Option<SocketGuard>>) -> MutexGuard<'_, Option<SocketGuard>> {
    m.lock().unwrap_or_else(PoisonError::into_inner)
}

/// Start the daemon with a mirror plane ONLY (a [`NoopEgressPlane`] + [`NoopQueryPlane`]
/// are injected for the egress + query verbs — an explicit refusal). The mirror-only
/// entry point; [`start_with_planes`] is the production entry that
/// adds the egress + query planes.
///
/// DEPENDENCY-INJECTED: `plane` is the mirror plane (production `GatewayMirrorPlane`
/// or a test spy). A second start on the same socket while a daemon holds the flock
/// is refused with [`io::ErrorKind::AddrInUse`] (the first-consumer-spawns race).
pub fn start(
    socket_path: PathBuf,
    plane: Arc<dyn MirrorPlane>,
    config: NetdConfig,
) -> io::Result<RunningNetd> {
    start_with_planes(
        socket_path,
        plane,
        Arc::new(NoopEgressPlane),
        Arc::new(NoopQueryPlane),
        config,
    )
}

/// Start the daemon with the mirror plane + the egress plane (a
/// [`NoopQueryPlane`] is injected for the query verbs). The entry point for a
/// daemon without a query plane; [`start_with_planes`] is the full production entry that also wires the query
/// plane.
pub fn start_with_egress(
    socket_path: PathBuf,
    plane: Arc<dyn MirrorPlane>,
    egress_plane: Arc<dyn EgressPlane>,
    config: NetdConfig,
) -> io::Result<RunningNetd> {
    start_with_planes(
        socket_path,
        plane,
        egress_plane,
        Arc::new(NoopQueryPlane),
        config,
    )
}

/// Start the daemon with ALL THREE planes — the mirror plane (ingress),
/// the egress plane, AND the query plane (catalog/schema) — the production entry
/// (`main.rs` builds a `GatewayMirrorPlane` + a `GatewayEgressPlane` + a
/// `GatewayQueryPlane` over the SAME shared manager, Principle #8). The mirror-only /
/// mirror+egress entries ([`start`] / [`start_with_egress`]) delegate here with the
/// no-op planes.
///
/// DEPENDENCY-INJECTED: the e2e drives an in-process daemon with counting spy planes
/// (no zenoh, parallel-safe). A second start on the same socket while a daemon holds
/// the flock is refused with [`io::ErrorKind::AddrInUse`] (the first-consumer-spawns
/// race).
pub fn start_with_planes(
    socket_path: PathBuf,
    plane: Arc<dyn MirrorPlane>,
    egress_plane: Arc<dyn EgressPlane>,
    query_plane: Arc<dyn QueryPlane>,
    config: NetdConfig,
) -> io::Result<RunningNetd> {
    start_with_planes_and_events(
        socket_path,
        plane,
        egress_plane,
        query_plane,
        Arc::new(NoopAnnounceWatchPlane),
        config,
    )
}

/// Start the daemon with all four planes — mirror (ingress), egress, query,
/// AND the announce-watch plane behind the catalog-change PUSH seam. The production
/// entry (`main.rs` builds all four over the SAME shared manager, Principle #8).
///
/// [`start_with_planes`] delegates here with a [`NoopAnnounceWatchPlane`], so
/// its callers get no watch: `subscribe_catalog` still succeeds
/// (nothing is refused) but reports `watching: false` and no push can arrive.
///
/// DEPENDENCY-INJECTED for the same reason as the others: the e2e drives an in-process
/// daemon with a SCRIPTED announce stream, so the coalescing + push + no-wedge
/// contracts are pinned without a network.
pub fn start_with_planes_and_events(
    socket_path: PathBuf,
    plane: Arc<dyn MirrorPlane>,
    egress_plane: Arc<dyn EgressPlane>,
    query_plane: Arc<dyn QueryPlane>,
    announce_plane: Arc<dyn AnnounceWatchPlane>,
    config: NetdConfig,
) -> io::Result<RunningNetd> {
    let (listener, guard) = acquire_socket(socket_path.clone())?;
    listener.set_nonblocking(true)?;

    let now = Instant::now();
    let registry = Arc::new(Mutex::new(DemandRegistry::new(now)));
    let shutdown = Arc::new(AtomicBool::new(false));
    let self_exit = Arc::new(AtomicBool::new(false));
    let conns: Arc<Mutex<Vec<JoinHandle<()>>>> = Arc::new(Mutex::new(Vec::new()));
    // Share the socket-cleanup guard so the idle-watch can drop it AT the
    // self-exit commit (unlink the socket the instant the daemon decides to exit).
    let guard: Arc<Mutex<Option<SocketGuard>>> = Arc::new(Mutex::new(Some(guard)));
    // The push hub shares the daemon's shutdown flag, so the announce-watch
    // thread stops with everything else.
    let events = Arc::new(CatalogEventHub::with_window(
        announce_plane,
        Arc::clone(&shutdown),
        config.change_coalesce_window,
    ));
    let conn_loop_iterations = Arc::new(AtomicU64::new(0));
    let conn_push_wakes = Arc::new(AtomicU64::new(0));
    let conn_waker_max_drain = Arc::new(AtomicU64::new(0));
    let conn_waker_total_drain = Arc::new(AtomicU64::new(0));
    let ctx = Ctx {
        registry: Arc::clone(&registry),
        plane,
        egress_plane,
        query_plane,
        self_exit: Arc::clone(&self_exit),
        egress_login: config.egress_login.clone(),
        events: Arc::clone(&events),
        conn_loop_iterations: Arc::clone(&conn_loop_iterations),
        conn_push_wakes: Arc::clone(&conn_push_wakes),
        conn_waker_max_drain: Arc::clone(&conn_waker_max_drain),
        conn_waker_total_drain: Arc::clone(&conn_waker_total_drain),
        connect_endpoints: config.connect_endpoints.clone(),
    };

    // Idle-watch thread: flips self_exit when idle for the grace AND unlinks the
    // socket at that commit (via the shared guard). A STANDING
    // daemon (`idle_self_exit: false` — a LISTEN-configured machine hosting the
    // start-booted gateway + beacon) spawns NO idle-watch at all: nothing ever
    // commits `self_exit`, so it runs until SIGINT/SIGTERM (see the field's doc
    // for why the gateway must hold the daemon open).
    let idle_handle = if config.idle_self_exit {
        let registry = Arc::clone(&registry);
        let shutdown = Arc::clone(&shutdown);
        let self_exit = Arc::clone(&self_exit);
        let guard = Arc::clone(&guard);
        let idle_grace = config.idle_grace;
        let poll = config.idle_watch_poll;
        Some(
            std::thread::Builder::new()
                .name("netd-idle".to_string())
                .spawn(move || {
                    idle_watch(registry, shutdown, self_exit, guard, idle_grace, poll)
                })?,
        )
    } else {
        tracing::info!(
            "cerulion-netd: idle self-exit DISABLED — this daemon hosts a standing gateway (a \
             LISTEN-configured machine) and runs until SIGINT/SIGTERM"
        );
        None
    };

    // Accept loop. Clone `shutdown` into `sd` for the closure so the outer
    // `shutdown` stays owned by this fn (used in the Err arm below to tear the
    // idle thread down on a failed accept spawn).
    let accept_handle = {
        let ctx = ctx.clone();
        let sd = Arc::clone(&shutdown);
        let conns = Arc::clone(&conns);
        match std::thread::Builder::new()
            .name("netd-accept".to_string())
            .spawn(move || accept_loop(listener, ctx, sd, conns))
        {
            Ok(h) => h,
            Err(e) => {
                // The idle thread (if any) is already running — flip shutdown +
                // join it before propagating, so a failed accept spawn tears down
                // cleanly. After the join drops the idle-watch's guard clone, this
                // fn's remaining `guard` Arc drops on the early return → the
                // SocketGuard drops (removing the socket + pidfile).
                shutdown.store(true, Ordering::SeqCst);
                if let Some(h) = idle_handle {
                    let _ = h.join();
                }
                return Err(e);
            }
        }
    };

    let identity_handle = if config.account_identity_watch {
        let watch_ctx = ctx.clone();
        let watch_shutdown = Arc::clone(&shutdown);
        match std::thread::Builder::new()
            .name("netd-account-identity".into())
            .spawn(move || account_identity_watch(watch_ctx, watch_shutdown))
        {
            Ok(handle) => Some(handle),
            Err(error) => {
                shutdown.store(true, Ordering::SeqCst);
                let _ = accept_handle.join();
                if let Some(handle) = idle_handle {
                    let _ = handle.join();
                }
                return Err(error);
            }
        }
    } else {
        None
    };

    tracing::info!(
        socket = %socket_path.display(),
        idle_grace_secs = config.idle_grace.as_secs(),
        "cerulion-netd listening (NDJSON control protocol)"
    );
    Ok(RunningNetd {
        shutdown,
        self_exit,
        accept_handle: Some(accept_handle),
        idle_handle,
        identity_handle,
        conns,
        registry,
        guard,
        socket_path,
        events,
        conn_loop_iterations,
        conn_push_wakes,
        conn_waker_max_drain,
        conn_waker_total_drain,
    })
}

fn account_identity_watch(ctx: Ctx, shutdown: Arc<AtomicBool>) {
    use cerulion_core::transport::failure_regime_latch::{FailureRegimeLatch, RegimeDecision};
    let mut failures = FailureRegimeLatch::new();
    while !shutdown.load(Ordering::SeqCst) && !ctx.self_exit.load(Ordering::SeqCst) {
        let result = match ctx.registry.try_lock() {
            Ok(mut registry) => Some(ctx.plane.refresh_identity(&mut registry)),
            Err(std::sync::TryLockError::WouldBlock) => None,
            Err(std::sync::TryLockError::Poisoned(_)) => {
                Some(Err("daemon registry is poisoned".into()))
            }
        };
        match result {
            Some(Err(error)) => match failures.on_failure() {
                RegimeDecision::Loud => {
                    tracing::warn!(%error, "netd: account identity refresh refused")
                }
                RegimeDecision::StillFailing { total, suppressed } => {
                    tracing::warn!(%error, total, suppressed, "netd: account identity refresh remains unavailable")
                }
                RegimeDecision::Suppressed { suppressed } => {
                    tracing::debug!(%error, suppressed, "netd: account identity refresh still unavailable")
                }
            },
            Some(Ok(())) => {
                if let Some(suppressed) = failures.on_success() {
                    tracing::info!(suppressed, "netd: account identity refresh recovered");
                }
            }
            None => {}
        }
        std::thread::sleep(Duration::from_millis(250));
    }
}

/// The idle-watch loop: on a grace-elapsed idle state, commit `self_exit`, UNLINK
/// the control socket, and exit.
fn idle_watch(
    registry: Arc<Mutex<DemandRegistry>>,
    shutdown: Arc<AtomicBool>,
    self_exit: Arc<AtomicBool>,
    guard: Arc<Mutex<Option<SocketGuard>>>,
    idle_grace: Duration,
    poll: Duration,
) {
    while !shutdown.load(Ordering::Acquire) {
        std::thread::sleep(poll);
        if shutdown.load(Ordering::Acquire) {
            return;
        }
        // Decide, commit self_exit, AND unlink the socket ALL UNDER the
        // registry lock, so the whole commit is atomic with a racing connection's
        // `connect` ([`Ctx::connect_unless_exiting`]). A connection that grabbed the
        // lock first made the registry busy → `should_self_exit` returns false here
        // and we do NOT commit (the connection is served, exit cancelled). If we grab
        // the lock first and commit, we drop the guard (unlink the socket + pidfile,
        // release the flock) so a racing client finds NO socket → `is_not_running` →
        // respawns a fresh daemon, and any already-queued backlog connection is
        // refused by `connect_unless_exiting` (self_exit is set). Never hold the loud
        // `info!` under the lock. main.rs's shutdown then just stops the threads.
        let committed = {
            let reg = lock_registry(&registry);
            if reg.should_self_exit(Instant::now(), idle_grace) {
                self_exit.store(true, Ordering::Release);
                // Drop the guard AT the commit (unlink now, not ~200ms later when
                // main notices self_exit) — closes the commit→unlink race window a
                // client retry could not otherwise bridge.
                lock_guard(&guard).take();
                true
            } else {
                false
            }
        };
        if committed {
            tracing::info!(
                idle_grace_secs = idle_grace.as_secs(),
                "cerulion-netd: idle (no consumers) for the grace period — self-exiting \
                 (control socket unlinked)"
            );
            return;
        }
    }
}

/// A once-per-regime flood latch (the `DrainWarnLatch` pattern class): the FIRST
/// failure of a regime warns loudly, repeats downgrade to `debug!` carrying a
/// running suppressed count, and a success re-arms the latch (so a healed-then-
/// broken regime warns loudly again). The counter is UNCONDITIONAL — it bumps on
/// every failure regardless of log level. Keeps a persistent failure (`EMFILE`
/// fd-exhaustion, ~20/s at [`ACCEPT_POLL`]) from storming stderr.
#[derive(Default)]
struct FloodLatch {
    warned: bool,
    suppressed: u64,
    total: u64,
}

impl FloodLatch {
    /// Record a failure. Returns `true` iff this is the FIRST of the regime (the
    /// caller `warn!`s); otherwise the caller `debug!`s with the suppressed count.
    fn on_failure(&mut self) -> bool {
        self.total += 1;
        if self.warned {
            self.suppressed += 1;
            false
        } else {
            self.warned = true;
            true
        }
    }
    /// Record a success — re-arm the latch (a fresh failure regime warns again).
    fn on_success(&mut self) {
        self.warned = false;
        self.suppressed = 0;
    }
}

/// The accept loop: nonblocking `accept` polled against the shutdown flag, one
/// handler thread per connection. Finished handles are reaped each pass so the
/// tracking vec stays bounded. Accept errors and connection-thread SPAWN failures
/// have SEPARATE flood latches ([`FloodLatch`]) — a spawn latch re-armed only on a
/// spawn SUCCESS, not on every accept — so a persistent spawn failure (fd/thread
/// exhaustion) warns once per regime, then `debug!`s with a running count, instead
/// of once per accepted connection.
fn accept_loop(
    listener: UnixListener,
    ctx: Ctx,
    shutdown: Arc<AtomicBool>,
    conns: Arc<Mutex<Vec<JoinHandle<()>>>>,
) {
    let mut accept_latch = FloodLatch::default();
    let mut spawn_latch = FloodLatch::default();
    while !shutdown.load(Ordering::Acquire) {
        match listener.accept() {
            Ok((stream, _addr)) => {
                accept_latch.on_success(); // a successful accept ends an accept-error regime.
                let ctx = ctx.clone();
                let sd = Arc::clone(&shutdown);
                match std::thread::Builder::new()
                    .name("netd-conn".to_string())
                    .spawn(move || handle_connection(stream, ctx, sd))
                {
                    Ok(h) => {
                        spawn_latch.on_success(); // a successful spawn ends a spawn-failure regime.
                        let mut guard = lock_conns(&conns);
                        guard.retain(|h| !h.is_finished()); // reap finished handlers
                        guard.push(h);
                    }
                    Err(e) => {
                        if spawn_latch.on_failure() {
                            tracing::warn!(error = %e, "netd: failed to spawn a connection thread (repeats log at debug)");
                        } else {
                            tracing::debug!(error = %e, suppressed = spawn_latch.suppressed, "netd: still failing to spawn a connection thread");
                        }
                    }
                }
            }
            Err(e) if e.kind() == io::ErrorKind::WouldBlock => {
                std::thread::sleep(ACCEPT_POLL);
            }
            Err(e) => {
                if accept_latch.on_failure() {
                    tracing::warn!(error = %e, "netd: control-socket accept error (repeats log at debug)");
                } else {
                    tracing::debug!(error = %e, suppressed = accept_latch.suppressed, "netd: control-socket accept still failing");
                }
                std::thread::sleep(ACCEPT_POLL);
            }
        }
    }
}

/// What ended one control-loop wait.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ControlWake {
    /// The socket has bytes (or has closed) — read it.
    Socket,
    /// This connection's push waker fired — a catalog-change push is in its slot.
    Push,
    /// Neither; the loop re-checks its shutdown flag and drains its slot anyway.
    /// This is the read-timeout cadence, kept as the FALLBACK that makes the wait a
    /// strict superset of a plain blocking read.
    Timeout,
}

/// Wait for socket-readable OR push-slot-filled, up to `timeout`.
///
/// `poll(2)` is what makes this ONE wait rather than two. The push drain sits
/// immediately before the read, so with a plain read a push waits up to a full
/// `CONN_READ_TIMEOUT` for the read to time out; waiting on the SOCKET alone would be
/// worse (an idle Studio sends nothing, so the push would wait
/// forever). A condvar cannot be joined to a socket in one
/// syscall, which is why the waker is a pipe.
///
/// The socket keeps its `SO_RCVTIMEO`: this reports readiness, and a spurious
/// readiness (or a peer that closes between the poll and the read) must not be able
/// to block the loop.
///
/// Errors and unexpected revents both map to [`ControlWake::Socket`] — the read is
/// the path that classifies EOF and real errors, and routing an oddity there keeps
/// exactly one place deciding when a connection ends. `EINTR` maps to `Timeout`, i.e.
/// "go round again", which is what a plain read's `Interrupted` arm does.
fn wait_for_control_input(
    stream: &UnixStream,
    waker_rx: Option<&std::io::PipeReader>,
    timeout: Duration,
) -> ControlWake {
    use std::os::fd::AsRawFd;
    let mut fds = [
        libc::pollfd {
            fd: stream.as_raw_fd(),
            events: libc::POLLIN,
            revents: 0,
        },
        libc::pollfd {
            fd: waker_rx.map(|r| r.as_raw_fd()).unwrap_or(-1),
            events: libc::POLLIN,
            revents: 0,
        },
    ];
    // A negative fd is ignored by `poll`, so a connection whose waker could not be
    // armed degrades to the timeout cadence with no special case here.
    let n = fds.len() as libc::nfds_t;
    let millis = timeout.as_millis().min(i32::MAX as u128) as libc::c_int;
    // SAFETY: `fds` is a live, correctly-sized array of `pollfd` owned by this frame.
    let rc = unsafe { libc::poll(fds.as_mut_ptr(), n, millis) };
    if rc < 0 {
        let err = std::io::Error::last_os_error();
        if err.kind() == io::ErrorKind::Interrupted {
            return ControlWake::Timeout;
        }
        // Anything else: let the read classify it (see the fn doc).
        return ControlWake::Socket;
    }
    if rc == 0 {
        return ControlWake::Timeout;
    }
    // The SOCKET wins a tie: a request already on the wire is served this pass, and
    // the push is drained at the top of the next one (the drain runs before every
    // wait, so it is never skipped either way). This is a DELIBERATE preference with
    // NO arm pinning it — a simultaneous-readiness tie is a genuine race, and a test
    // that tried to force it would assert on scheduling rather than on behaviour. It
    // is stated rather than tested because both orders are CORRECT (nothing is lost
    // under either); the choice keeps request/response servicing exactly as prompt as
    // a plain blocking read, so the waker can only ever add promptness.
    if fds[0].revents != 0 {
        return ControlWake::Socket;
    }
    if fds[1].revents != 0 {
        return ControlWake::Push;
    }
    ControlWake::Timeout
}

/// Drain the waker pipe so one poke does not keep the poll hot.
///
/// **Non-blocking because the READ END IS ARMED `O_NONBLOCK` at creation** — not
/// because of where this is called from. The tempting argument ("we only call this
/// when `poll` said the read end is readable, and we stop at the first short read")
/// is false on both halves and such a loop WEDGES a live connection: a FULL read is
/// not a short read, so exactly-64 queued bytes send the loop round again, and the
/// poll-readiness precondition covers only the FIRST read. On an unarmed blocking
/// read end that second read parks forever on an empty pipe whose only writer lives
/// in `HubState::wakers`, released by `clear_waker` — whose one caller is
/// `cleanup_connection`, on the very thread now blocked. Consequences of that
/// self-deadlock: the handler never reads its socket again (the
/// client's next request dies on `client::ROUNDTRIP_TIMEOUT`, 5 s — this is a NETD
/// connection, so a `NetdClient` round trip is what a wedged handler kills; the
/// similarly-sized `viz_client` deadline governs a different socket), never reaches
/// `cleanup_connection` (so its demands are never released and its mirrors never
/// retired — the refcount contract), keeps `active_connections >= 1` (so the
/// idle self-exit never commits), and hangs [`RunningNetd::shutdown`]'s join.
///
/// So the loop leaves on ANY read that is not a full sink: `WouldBlock` (drained),
/// a short read (drained), `Ok(0)` (EOF — reported as DEAD) or any other error. A
/// read error is otherwise ignored — the pipe is a WAKE, never data, so nothing is
/// lost by failing to drain it (the next poll simply fires again, and the slot drain
/// at the loop top is idempotent).
fn drain_waker(waker_rx: &mut std::io::PipeReader) -> WakerDrain {
    let mut sink = [0u8; 64];
    let mut bytes = 0u64;
    loop {
        match waker_rx.read(&mut sink) {
            // EOF: every write end is gone. The caller must drop this reader.
            Ok(0) => {
                return WakerDrain {
                    bytes,
                    alive: false,
                }
            }
            Ok(n) if n == sink.len() => bytes += n as u64, // the sink filled — read on.
            Ok(n) => {
                return WakerDrain {
                    bytes: bytes + n as u64,
                    alive: true,
                }
            }
            // `WouldBlock` is the ordinary end of a full-sink read; anything else is
            // an oddity the next poll will re-raise.
            Err(_) => return WakerDrain { bytes, alive: true },
        }
    }
}

/// What [`drain_waker`] observed.
struct WakerDrain {
    /// Wake bytes consumed by this call, for the Principle-#3 high-water mark
    /// ([`Ctx::conn_waker_max_drain`]). A wake is a SIGNAL, so this means "how far
    /// behind was I", never "how many pushes".
    bytes: u64,
    /// `false` once the pipe reports EOF — every write end is gone, so the read end
    /// is permanently `POLLHUP`-ready and must be dropped or the poll spins.
    alive: bool,
}

/// Mint one connection's push-waker pipe with the READ END ARMED
/// non-blocking.
///
/// Extracted so the arming is ASSERTABLE. Its absence is otherwise catastrophic but
/// nearly silent: the handler drains at the loop top, so an unarmed read end blocks
/// on the FIRST pass of every connection with an empty pipe — the daemon serves its
/// banner, answers nothing else ever, and `RunningNetd::shutdown` hangs joining it.
/// A test suite that hits this condition HANGS rather than going red, which is the
/// camouflaged failure class this repo refuses to rely on. `arms_the_read_end_...`
/// reads the flag back instead, and because this function's only other caller would
/// then be a test, deleting the CALL from `handle_connection` trips `dead_code =
/// deny` at the lint gate rather than shipping.
///
/// The WRITE end is armed by `PushWaker::new` inside `set_waker` — the hub owns that
/// invariant, because it is the hub's own watch thread a blocking write would park
/// (see `PushWaker`'s type doc). Both ends go through the SAME
/// `catalog_events::set_fd_nonblocking`.
fn mint_waker_pipe() -> std::io::Result<(std::io::PipeReader, std::io::PipeWriter)> {
    use std::os::fd::AsRawFd;
    let (rx, tx) = std::io::pipe()?;
    crate::catalog_events::set_fd_nonblocking(rx.as_raw_fd())?;
    Ok((rx, tx))
}

/// One connection handler: register the connection, send the [`Hello`] banner,
/// serve request lines until EOF / error, then disconnect — RELEASING every demand
/// the connection held (the crash-safe refcount) and tearing down each now-unheld
/// mirror.
fn handle_connection(mut stream: UnixStream, ctx: Ctx, shutdown: Arc<AtomicBool>) {
    // Clear the accept-inherited O_NONBLOCK, THEN install the timeouts.
    // The read loop below is PACED by `read` blocking for `CONN_READ_TIMEOUT`; on a
    // nonblocking socket that read returns EAGAIN in microseconds and the
    // `WouldBlock => continue` arm becomes an unbounded tight loop (MEASURED at
    // 198.9 % CPU on a netd with an EMPTY demand table). One shared seam with vizd —
    // see `cerulion_core::prepare_accepted_stream`.
    if cerulion_core::prepare_accepted_stream(&stream, CONN_READ_TIMEOUT, CONN_WRITE_TIMEOUT)
        .is_err()
    {
        return; // an unusable socket — nothing to serve.
    }

    // Register the connection UNLESS the daemon has already committed to
    // idle self-exit (under the registry lock). If it has, this connection arrived
    // too late — refuse it CLEANLY (drop the stream WITHOUT a Hello) so the client
    // re-classifies (the socket is being unlinked → respawn) and retries, instead
    // of being served a Hello and then dropped when the shutdown flag flips.
    // Nothing was registered, so there is no cleanup to do.
    let Some(conn) = ctx.connect_unless_exiting() else {
        return;
    };

    // Arm this connection's push waker — the pipe the hub pokes so a
    // catalog-change push does not wait out a `CONN_READ_TIMEOUT`. A pipe that cannot
    // be created degrades LOUDLY to the read-timeout cadence (`None` below
    // makes `wait_for_control_input` poll the socket alone); it never fails the
    // connection, because the push is an OPTIMISATION and the demand plane this
    // connection carries is not.
    let mut waker_rx = match mint_waker_pipe() {
        // `set_waker` takes the writer BY VALUE and DROPS it when arming the WRITE end
        // fails, which closes the pipe; keeping the read end then makes every poll
        // report POLLHUP and the loop spins. Believing its answer is what makes this
        // degradation message true rather than a promise the code breaks.
        Ok((rx, tx)) => ctx.events.set_waker(conn, tx).then_some(rx),
        Err(e) => {
            tracing::warn!(
                conn,
                error = %e,
                "could not create this connection's push-waker pipe — \
                 catalog-change pushes fall back to the read-timeout cadence"
            );
            None
        }
    };

    // Banner FIRST (the version handshake). A write failure = the consumer already
    // left → clean up and return.
    if !write_line(&mut stream, &Hello::new().to_json_line()) {
        cleanup_connection(&ctx, conn);
        return;
    }

    let mut pending: Vec<u8> = Vec::new();
    let mut tmp = [0u8; 4096];
    loop {
        // One bump per trip round the loop (Principle #3). Cheap (a relaxed
        // fetch_add against a loop whose other arm is a syscall) and it is the ONLY
        // way to tell "paced by the read timeout" from "spinning" without a profiler.
        ctx.conn_loop_iterations.fetch_add(1, Ordering::Relaxed);
        if shutdown.load(Ordering::Acquire) {
            break;
        }
        // Drain the push waker BEFORE the slot, on EVERY pass.
        //
        // This is where `wait_for_control_input`'s tie-order argument becomes TRUE:
        // it says the socket wins a tie and "the push is drained at the top of the
        // next one (the drain runs before every wait, so it is never skipped either
        // way)". It was NOT — the drain lived on the `Push` arm alone, so every poke
        // that landed while the socket was readable, and every poke a `Timeout` pass
        // stepped over, stayed in the pipe. MEASURED against the real daemon: 1396
        // stale bytes on one connection, i.e. an unbounded backlog whose only bound
        // was a `Push` arm eventually firing. Draining here bounds the residue at the
        // pokes landing inside ONE pass, and the tie order is unchanged: the socket
        // still wins, the push still rides the next pass.
        //
        // Ordering is safe in BOTH directions because `publish_inner` fills the slot
        // and pokes under ONE lock acquisition: any byte we consume here belongs to a
        // slot fill that already happened, and `take_pending` below sees it; a byte
        // that lands after this drain merely costs one extra (idempotent) pass.
        //
        // Cost, stated: one extra `read(2)` per pass, returning `EAGAIN` when there is
        // nothing to drain. On an idle connection that is 5/s.
        let mut waker_died = false;
        if let Some(rx) = waker_rx.as_mut() {
            let drain = drain_waker(rx);
            if drain.bytes > 0 {
                ctx.conn_waker_max_drain
                    .fetch_max(drain.bytes, Ordering::Relaxed);
                // The running SUM beside the running MAX. Same guard, same
                // value, one extra relaxed add on a path that just made a syscall.
                ctx.conn_waker_total_drain
                    .fetch_add(drain.bytes, Ordering::Relaxed);
            }
            waker_died = !drain.alive;
        }
        if waker_died {
            // EOF on the read end: every write end is gone, so `poll` reports POLLHUP
            // forever and the push arm would fire on every pass with nothing to
            // consume — the spin class. Drop it and take the timeout cadence.
            tracing::warn!(
                conn,
                "this connection's push waker closed unexpectedly — \
                 catalog-change pushes fall back to the read-timeout cadence"
            );
            waker_rx = None;
        }
        // Drain this connection's catalog-change push, if it has one.
        //
        // The push is written HERE — by the connection's own handler thread, on the
        // read loop it already runs — and not by the announce watch. That is the whole
        // no-wedge story: exactly one thread ever writes a given stream (so no
        // interleaved half-lines are possible), the watch thread does a mutex store and
        // never any I/O (so a consumer that stopped reading cannot stall it or a
        // healthy sibling), and a stalled consumer's own `write_line` times out and
        // drops its OWN connection exactly as it would for any response.
        //
        // The loop reaches here at least every `CONN_READ_TIMEOUT` (200 ms — the read
        // below times out rather than blocking forever), so the extra notification
        // latency this costs is bounded by that, on top of the coalescing window.
        // 200 ms is now the REAL cadence on an idle connection, not just a
        // ceiling. Until the accepted socket's inherited O_NONBLOCK was cleared, the
        // read returned EAGAIN instantly and this drain ran in a microsecond-scale
        // spin — a push latency nobody asked for, bought with a whole CPU core.
        if let Some(event) = ctx.events.take_pending(conn) {
            if !write_line(&mut stream, &event.to_json_line()) {
                cleanup_connection(&ctx, conn);
                return; // consumer gone / wedged — only this connection dies.
            }
        }
        // Wait for socket-readable OR push-slot-filled, instead of
        // blocking in `read` and discovering the push only when that timed out. The
        // drain ABOVE runs before every wait, so a push that landed while we were
        // writing the previous response is delivered without needing a wake at all —
        // the wake only has to cover a push that lands while we are parked here.
        match wait_for_control_input(&stream, waker_rx.as_ref(), CONN_READ_TIMEOUT) {
            ControlWake::Push => {
                ctx.conn_push_wakes.fetch_add(1, Ordering::Relaxed);
                continue; // the loop top drains the waker, then the slot.
            }
            ControlWake::Timeout => continue, // idle: re-check the shutdown flag.
            ControlWake::Socket => {}
        }
        match stream.read(&mut tmp) {
            Ok(0) => break, // EOF: the consumer closed the connection.
            Ok(n) => {
                pending.extend_from_slice(&tmp[..n]);
                while let Some(pos) = pending.iter().position(|&b| b == b'\n') {
                    let line: Vec<u8> = pending.drain(..=pos).collect();
                    let text = String::from_utf8_lossy(&line[..line.len() - 1]);
                    let text = text.trim_end_matches('\r'); // tolerate CRLF
                    if text.trim().is_empty() {
                        continue;
                    }
                    let response = dispatch_line(text, &ctx, conn);
                    if !write_line(&mut stream, &response.to_json_line()) {
                        cleanup_connection(&ctx, conn);
                        return; // consumer gone / wedged mid-exchange.
                    }
                }
                // Whatever remains is an in-progress line. If it grew past the cap
                // without a newline, the consumer is building an unbounded line —
                // refuse it and drop the connection so the buffer can never OOM.
                if pending.len() > MAX_REQUEST_LINE_BYTES {
                    tracing::warn!(
                        pending_bytes = pending.len(),
                        cap = MAX_REQUEST_LINE_BYTES,
                        "netd: request line exceeded the cap without a newline — dropping the connection"
                    );
                    let response = Response::error(
                        None,
                        format!(
                            "request line exceeded {MAX_REQUEST_LINE_BYTES} bytes without a newline \
                             — dropping the connection"
                        ),
                        None,
                        None,
                    );
                    let _ = write_line(&mut stream, &response.to_json_line());
                    break;
                }
            }
            Err(e)
                if matches!(
                    e.kind(),
                    io::ErrorKind::WouldBlock
                        | io::ErrorKind::TimedOut
                        | io::ErrorKind::Interrupted
                ) =>
            {
                continue; // idle: re-check the shutdown flag.
            }
            Err(_) => break, // a real read error ends the connection.
        }
    }
    cleanup_connection(&ctx, conn);
}

/// Disconnect `conn`: release every demand AND egress registration it held (the
/// crash-safe refcount), drive the mirror-plane teardown for each now-unheld mirror,
/// and drive the egress-plane release if it held egress. Called on EVERY
/// connection-handler exit path.
fn cleanup_connection(ctx: &Ctx, conn: ConnId) {
    // The connection close IS the unsubscribe (the same crash-safe rule the
    // demand plane uses) — a dead consumer never accumulates a push slot nobody drains.
    ctx.events.unsubscribe(conn);
    // The waker is CONNECTION-scoped (a client may unsubscribe and
    // re-subscribe on one connection), so it is released here — on every
    // `cleanup_connection` exit path — rather than in `unsubscribe`.
    ctx.events.clear_waker(conn);
    // Collect the last-released demand keys + the released egress under the lock,
    // then process each off the lock (re-check + release_mirror + retire) — see
    // [`process_released_key`] — and the egress plane's release.
    let out = { lock_registry(&ctx.registry).disconnect(conn, Instant::now()) };
    for key in out.mirror_last_released {
        process_released_key(&ctx.registry, ctx.plane.as_ref(), &key);
    }
    if !out.egress_released.is_empty() {
        ctx.egress_plane.release_egress(conn);
    }
}

/// Process ONE last-released `(robot, topic)` — the shared teardown path for BOTH
/// the `release` verb and a connection disconnect. Three steps:
///
/// 1. **CLAIM the teardown UNDER the lock.** [`DemandRegistry::claim_tearing`]
///    atomically marks the lingering entry `tearing` and returns whether THIS caller
///    won the claim. It returns `false` — and we SKIP — when the key was re-claimed
///    by a demand in the gap since the release (refcount > 0, its bridge is live: the
///    re-claim guard) OR another teardown is already in flight (`tearing` — a
///    concurrent same-key teardown; a second `release_mirror` here would
///    double-unregister and fire a spurious loud error, so we no-op). Then release
///    the lock.
/// 2. **Tear down OFF the lock.** `release_mirror`'s zenoh undeclare can
///    BLOCK on the real plane; running it while holding the registry lock would
///    starve the idle-watch's self-exit check and every other connection's `connect`
///    — the exact wedge that left the daemon half-dead forever. While `tearing` is
///    set every demand for the key returns [`DemandOutcome::Retiring`] (the daemon
///    WAITS then re-demands into a fresh mirror), so no demand can reuse the dying
///    bridge — the phantom-held-ack window is CLOSED.
/// 3. **Retire / clear under the lock.** A `Retired` teardown [`DemandRegistry::retire`]s
///    the entry (refcount is still 0 — `tearing` blocked every demand). A `Lingering`
///    teardown (failure) [`DemandRegistry::clear_tearing`]s so the kept bridge is
///    reusable again.
fn process_released_key(registry: &Mutex<DemandRegistry>, plane: &dyn MirrorPlane, key: &TopicKey) {
    // Step 1: CLAIM the teardown under the lock (marks `tearing`), then RELEASE it.
    {
        let mut reg = lock_registry(registry);
        if !reg.claim_tearing(key) {
            // Re-claimed by a demand (live bridge) OR already being torn down by a
            // concurrent caller — either way, leave it alone.
            return;
        }
    }
    // Step 2: the (potentially BLOCKING) teardown runs with NO registry lock held.
    // `tearing` keeps every concurrent demand for the key in Retiring (no reuse).
    let outcome = plane.release_mirror(key);
    // Step 3: retire (Retired) or clear the tearing flag (Lingering) under the lock.
    let mut reg = lock_registry(registry);
    match outcome {
        crate::mirror::MirrorRelease::Retired => {
            if reg.retire(key) {
                plane.mirror_retired(key);
            }
        }
        crate::mirror::MirrorRelease::Lingering => {
            reg.clear_tearing(key);
        }
    }
}

/// Parse + dispatch one request line into its response — a malformed line becomes
/// a structured error carrying the best-effort correlation id (never a panic).
fn dispatch_line(line: &str, ctx: &Ctx, conn: ConnId) -> Response {
    match parse_request(line) {
        Ok(req) => ctx.dispatch(conn, req),
        Err(e) => Response::error(
            e.id,
            format!("malformed request: {}", e.message),
            None,
            None,
        ),
    }
}

/// Write one NDJSON line + newline, returning whether the write SUCCEEDED (so the
/// caller ends the connection on failure). Larger-than-send-buffer lines (e.g. the
/// 75-topic Go2 catalog / demand-table reply) go through the shared progress-based
/// bounded resume loop ([`cerulion_core::write_line_bounded`], shared with
/// `cerulion_vizd`): a full send buffer does not truncate the line — the
/// write RESUMES once the consumer drains, and the connection is dropped only when
/// the consumer makes NO forward progress within [`CONN_WRITE_TIMEOUT`] (the real
/// "stopped reading" signal). Small lines take the exact same path with no extra
/// latency (they fit in one `write`). The shared impl is logging-agnostic; netd maps
/// its no-progress / peer-closed outcomes to a `debug!` (an ordinary disconnect stays
/// silent).
fn write_line(stream: &mut UnixStream, line: &str) -> bool {
    match cerulion_core::write_line_bounded(stream, line, CONN_WRITE_TIMEOUT) {
        cerulion_core::UdsWriteOutcome::Complete => true,
        cerulion_core::UdsWriteOutcome::NoProgressTimeout { written, total } => {
            tracing::debug!(
                written,
                total,
                no_progress_ms = CONN_WRITE_TIMEOUT.as_millis() as u64,
                "netd: consumer not reading — no write progress within the timeout, dropping the connection"
            );
            false
        }
        cerulion_core::UdsWriteOutcome::PeerClosed { written, total } => {
            tracing::debug!(
                written,
                total,
                "netd: consumer closed mid-write — dropping the connection"
            );
            false
        }
        cerulion_core::UdsWriteOutcome::IoError => false, // normal disconnect (BrokenPipe/etc.).
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mirror::{MirrorError, MirrorRelease};

    /// A minimal counting mirror-plane double for the `process_released_key` re-check
    /// tests: records ensure/release calls and returns a configurable
    /// `MirrorRelease` (Lingering = a failed teardown, Retired = a real one). NOT fabricated data — a
    /// dependency-injection test double for the `MirrorPlane` trait.
    #[derive(Default)]
    struct TestSpy {
        ensure_calls: Mutex<usize>,
        release_calls: Mutex<Vec<TopicKey>>,
        /// When true, `release_mirror` returns `Retired` (models a real teardown).
        retired: bool,
    }
    impl TestSpy {
        fn release_count(&self) -> usize {
            self.release_calls.lock().unwrap().len()
        }
    }
    impl MirrorPlane for TestSpy {
        fn ensure_mirror(&self, _key: &TopicKey, _schema_hash: u64) -> Result<(), MirrorError> {
            *self.ensure_calls.lock().unwrap() += 1;
            Ok(())
        }
        fn release_mirror(&self, key: &TopicKey) -> MirrorRelease {
            self.release_calls.lock().unwrap().push(key.clone());
            if self.retired {
                MirrorRelease::Retired
            } else {
                MirrorRelease::Lingering
            }
        }
    }

    /// Demand `key` on `conn`, mark the mirror present, then disconnect `conn` so
    /// `key` is left LINGERING (refcount 0, bridge present) — the exact post-
    /// disconnect state `process_released_key` runs against.
    fn linger_key(registry: &Mutex<DemandRegistry>, conn: ConnId, key: &TopicKey) {
        let mut reg = registry.lock().unwrap();
        reg.demand(conn, key.clone(), 7, Instant::now());
        reg.mark_mirror_present(key);
        assert_eq!(
            reg.disconnect(conn, Instant::now()).mirror_last_released,
            vec![key.clone()]
        );
        assert!(reg.is_lingering(key));
    }

    #[test]
    fn process_released_key_skips_a_key_reclaimed_in_the_gap() {
        // THE re-claim race guard (deterministic, no threads): a demand RE-CLAIMS the
        // lingering key in the gap between disconnect and the per-key teardown; the
        // re-check must SKIP `release_mirror` so the now-live bridge is not torn
        // down. Uses a Retired spy so a missing re-check would DESTROY the
        // re-claimed bridge: removing the `is_lingering` guard
        // makes release_count == 1 here.
        let registry = Mutex::new(DemandRegistry::new(Instant::now()));
        let spy = TestSpy {
            retired: true,
            ..Default::default()
        };
        let key = TopicKey::new("ubuntu", "/tf");
        let a = registry.lock().unwrap().connect(Instant::now());
        let b = registry.lock().unwrap().connect(Instant::now());
        linger_key(&registry, a, &key);

        // GAP: b re-claims the lingering key (refcount 0→1) — models the concurrent
        // demand landing between disconnect and the per-key processing.
        assert_eq!(
            registry
                .lock()
                .unwrap()
                .demand(b, key.clone(), 7, Instant::now()),
            DemandOutcome::AlreadyMirrored { refcount: 1 }
        );
        assert!(!registry.lock().unwrap().is_lingering(&key));

        // The teardown re-checks under the lock → SKIP.
        process_released_key(&registry, &spy, &key);
        assert_eq!(
            spy.release_count(),
            0,
            "release_mirror NOT called on a re-claimed key"
        );
        let reg = registry.lock().unwrap();
        assert_eq!(reg.refcount(&key), 1, "the mirror stays working");
        assert!(reg.has_mirror(&key), "the bridge was NOT torn down");
    }

    #[test]
    fn process_released_key_tears_down_a_genuinely_lingering_key_c1_keeps_it() {
        // Control (Lingering): a still-lingering key IS processed —
        // release_mirror is called, and the Lingering return KEEPS the entry (reused
        // on re-demand until idle self-exit).
        let registry = Mutex::new(DemandRegistry::new(Instant::now()));
        let spy = TestSpy::default(); // Lingering
        let key = TopicKey::new("ubuntu", "/tf");
        let a = registry.lock().unwrap().connect(Instant::now());
        linger_key(&registry, a, &key);

        process_released_key(&registry, &spy, &key);
        assert_eq!(
            spy.release_count(),
            1,
            "release_mirror called on a lingering key"
        );
        let reg = registry.lock().unwrap();
        assert_eq!(reg.lingering_count(), 1, "Lingering keeps the entry");
        assert_eq!(reg.refcount(&key), 0);
    }

    #[test]
    fn process_released_key_retires_the_entry_when_the_plane_reports_retired() {
        // Forward path: a still-lingering key + a Retired teardown removes the
        // registry entry (the retire seam a real teardown exercises).
        let registry = Mutex::new(DemandRegistry::new(Instant::now()));
        let spy = TestSpy {
            retired: true,
            ..Default::default()
        };
        let key = TopicKey::new("ubuntu", "/tf");
        let a = registry.lock().unwrap().connect(Instant::now());
        linger_key(&registry, a, &key);

        process_released_key(&registry, &spy, &key);
        assert_eq!(spy.release_count(), 1);
        assert_eq!(
            registry.lock().unwrap().mirror_count(),
            0,
            "Retired → the entry is removed"
        );
    }

    #[test]
    fn connect_unless_exiting_gates_on_the_self_exit_commit() {
        // Atomicity: `connect_unless_exiting` registers a connection while
        // the daemon has NOT committed to self-exit, and refuses (None) once the
        // commit flag is set — both decided under the ONE registry lock, so the
        // idle-watch's commit and a racing connect can never both "win".
        let ctx = Ctx {
            registry: Arc::new(Mutex::new(DemandRegistry::new(Instant::now()))),
            plane: Arc::new(TestSpy::default()),
            egress_plane: Arc::new(NoopEgressPlane),
            egress_login: Default::default(),
            query_plane: Arc::new(NoopQueryPlane),
            self_exit: Arc::new(AtomicBool::new(false)),
            events: Arc::new(CatalogEventHub::new(
                Arc::new(NoopAnnounceWatchPlane),
                Arc::new(AtomicBool::new(false)),
            )),
            conn_loop_iterations: Arc::new(AtomicU64::new(0)),
            conn_push_wakes: Arc::new(AtomicU64::new(0)),
            conn_waker_max_drain: Arc::new(AtomicU64::new(0)),
            conn_waker_total_drain: Arc::new(AtomicU64::new(0)),
            connect_endpoints: Vec::new(),
        };
        // Not committed → the connection is accepted (a real ConnId, registry busy).
        let first = ctx.connect_unless_exiting();
        assert_eq!(first, Some(0), "a pre-commit connect is served");
        assert_eq!(
            lock_registry(&ctx.registry).active_connections(),
            1,
            "the served connection armed the registry busy"
        );
        // Commit self-exit → every later connect is refused (None), and the
        // registry is NOT armed by the refused attempt.
        ctx.self_exit.store(true, Ordering::Release);
        assert_eq!(
            ctx.connect_unless_exiting(),
            None,
            "a post-commit connect is refused"
        );
        assert_eq!(
            lock_registry(&ctx.registry).active_connections(),
            1,
            "the refused connect never registered (still just the first)"
        );
    }

    #[test]
    fn resolve_idle_grace_parses_ms_and_falls_back() {
        assert_eq!(resolve_idle_grace(None), DEFAULT_IDLE_GRACE);
        assert_eq!(resolve_idle_grace(Some("")), DEFAULT_IDLE_GRACE);
        assert_eq!(resolve_idle_grace(Some("   ")), DEFAULT_IDLE_GRACE);
        assert_eq!(resolve_idle_grace(Some("500")), Duration::from_millis(500));
        assert_eq!(
            resolve_idle_grace(Some("  1000  ")),
            Duration::from_millis(1000)
        );
        // Zero + non-numeric fall back to the default (loud).
        assert_eq!(resolve_idle_grace(Some("0")), DEFAULT_IDLE_GRACE);
        assert_eq!(resolve_idle_grace(Some("nope")), DEFAULT_IDLE_GRACE);
        assert_eq!(resolve_idle_grace(Some("-5")), DEFAULT_IDLE_GRACE);
    }

    #[test]
    fn netd_config_default_is_the_thirty_second_grace() {
        assert_eq!(NetdConfig::default().idle_grace, DEFAULT_IDLE_GRACE);
        assert_eq!(NetdConfig::default().idle_grace, Duration::from_secs(30));
    }

    #[test]
    fn flood_latch_warns_once_per_regime_counts_unconditionally_and_rearms() {
        let mut latch = FloodLatch::default();
        // First of a regime → warn (true); repeats → debug (false) with a running
        // suppressed count; the total counter bumps UNCONDITIONALLY.
        assert!(latch.on_failure(), "first failure warns");
        assert!(!latch.on_failure(), "repeat is suppressed");
        assert!(!latch.on_failure(), "still suppressed");
        assert_eq!(latch.suppressed, 2);
        assert_eq!(latch.total, 3);

        // A success re-arms the latch (a fresh regime warns again) but does NOT
        // reset the unconditional total.
        latch.on_success();
        assert!(
            latch.on_failure(),
            "post-recovery failure warns loudly again"
        );
        assert_eq!(latch.suppressed, 0, "suppressed count reset on re-arm");
        assert_eq!(latch.total, 4, "total is unconditional across regimes");
    }

    // ----------------------------------------------------------------------
    // The push WAKER's drain and arming contract.
    //
    // These drive the PRODUCTION pieces (`CatalogEventHub::set_waker`, the real
    // `publish` poke loop, `drain_waker`, `wait_for_control_input`) rather than a
    // transliteration, because the failure they pin is a disagreement BETWEEN two
    // production sites: `PushWaker::new` arming the WRITE end while `drain_waker`
    // loops over an unarmed READ end.
    // ----------------------------------------------------------------------

    /// A bounded wait for a `drain_waker` that MUST return.
    ///
    /// **EVERY** `drain_waker` call in this module's tests goes through here, not just
    /// the ones drilling the exact-multiple boundary: a drain over an unarmed read end
    /// blocks on an EMPTY pipe too, so a direct call is a hung test binary when the
    /// arming is dropped — the same camouflaged class `mint_waker_pipe` was extracted
    /// to remove. This can neither `join()` (that IS the hang) nor assert on a wall: it
    /// takes the reader, runs the PRODUCTION drain on a helper thread, and either
    /// reports the outcome or panics naming the state it parked on. A wedged helper is
    /// abandoned holding the reader — sound only because the process exits moments later.
    fn drain_or_panic(rx: std::io::PipeReader, state: &str) -> (std::io::PipeReader, WakerDrain) {
        let (done_tx, done_rx) = std::sync::mpsc::channel();
        std::thread::spawn(move || {
            let mut rx = rx;
            let drain = drain_waker(&mut rx);
            let _ = done_tx.send((rx, drain));
        });
        match done_rx.recv_timeout(Duration::from_secs(10)) {
            Ok(out) => out,
            Err(_) => panic!(
                "drain_waker DID NOT RETURN ({state}) — the read end is blocking and \
                 the loop parked on an empty pipe. In production nothing but the \
                 handler's own thread can close that writer, so the connection is \
                 wedged for good: no requests served, no `cleanup_connection`, no \
                 demand release, and `RunningNetd::shutdown` hangs on its join."
            ),
        }
    }

    /// A hub with no announce plane — these arms exercise the WAKER, and a Noop plane
    /// keeps the watch thread out of every oracle.
    fn waker_hub() -> Arc<CatalogEventHub> {
        Arc::new(CatalogEventHub::with_window(
            Arc::new(crate::catalog_events::NoopAnnounceWatchPlane),
            Arc::new(AtomicBool::new(false)),
            Duration::from_millis(10),
        ))
    }

    /// A `Ctx` over inert planes — enough to drive `handle_connection` itself.
    fn waker_ctx() -> Ctx {
        Ctx {
            registry: Arc::new(Mutex::new(DemandRegistry::new(Instant::now()))),
            plane: Arc::new(TestSpy::default()),
            egress_plane: Arc::new(NoopEgressPlane),
            egress_login: Default::default(),
            query_plane: Arc::new(NoopQueryPlane),
            self_exit: Arc::new(AtomicBool::new(false)),
            events: waker_hub(),
            conn_loop_iterations: Arc::new(AtomicU64::new(0)),
            conn_push_wakes: Arc::new(AtomicU64::new(0)),
            conn_waker_max_drain: Arc::new(AtomicU64::new(0)),
            conn_waker_total_drain: Arc::new(AtomicU64::new(0)),
            connect_endpoints: Vec::new(),
        }
    }

    /// Read a descriptor's `O_NONBLOCK` bit back off the kernel.
    fn is_nonblocking(fd: std::os::fd::RawFd) -> bool {
        // SAFETY: `fd` is owned by the caller for the duration of this call.
        let flags = unsafe { libc::fcntl(fd, libc::F_GETFL) };
        assert!(flags >= 0, "fcntl(F_GETFL) failed on fd {fd}");
        flags & libc::O_NONBLOCK != 0
    }

    /// The waker pipe a connection is given has its READ END armed non-blocking.
    ///
    /// The pin is a FLAG READ rather than a behaviour, deliberately: with the
    /// loop-top drain, an unarmed read end blocks on the first pass of EVERY
    /// connection, so the observable failure is a suite that HANGS — indistinguishable
    /// from a slow runner until a CI job times out, and the exact camouflage the repo
    /// records as its worst failure class. Reading the flag back fails in milliseconds
    /// and names the cause.
    ///
    /// The `std::io::pipe()` control in the same body is what makes it non-vacuous:
    /// it establishes that a freshly minted pipe is BLOCKING, so the assertion above
    /// is measuring `mint_waker_pipe`'s work rather than a platform default.
    #[test]
    fn the_waker_pipe_arms_the_read_end_because_the_drain_runs_before_every_wait() {
        use std::os::fd::AsRawFd;

        let (plain_rx, _plain_tx) = std::io::pipe().expect("pipe");
        assert!(
            !is_nonblocking(plain_rx.as_raw_fd()),
            "a freshly minted pipe read end is BLOCKING — if this ever changes, the \
             assertion below stops measuring anything"
        );

        let (rx, _tx) = mint_waker_pipe().expect("mint");
        assert!(
            is_nonblocking(rx.as_raw_fd()),
            "the waker READ end must be non-blocking: `drain_waker` runs at the top of \
             every loop pass, so an unarmed read end parks the handler on an empty \
             pipe on its FIRST pass and the connection never answers again"
        );
    }

    /// THE wedge pin: a waker pipe holding an EXACT MULTIPLE
    /// of the drain sink must not park the connection handler forever.
    ///
    /// Driven through the production path end to end — a real `std::io::pipe()` armed
    /// exactly as `handle_connection` arms it, the write end registered through the
    /// real `set_waker`, the bytes produced by the real `publish` poke loop, and the
    /// production `drain_waker` doing the draining. The 63/65 neighbours are what stop
    /// this from passing on a drain that simply never loops; the exact byte counts pin
    /// the loop's arithmetic (a `continue` that dropped its running total would fail
    /// here while still returning).
    #[test]
    fn a_full_sink_of_wake_bytes_does_not_wedge_the_drain() {
        use std::os::fd::AsRawFd;

        const CONN: ConnId = 7;
        let hub = waker_hub();
        let (rx, tx) = std::io::pipe().expect("pipe");
        crate::catalog_events::set_fd_nonblocking(rx.as_raw_fd()).expect("arm the read end");
        assert!(hub.set_waker(CONN, tx), "the waker must arm");
        let _ = hub.subscribe(CONN);

        let mut rx = rx;
        // 64 and 128 are the wedging shapes (one and two full sinks); 63 and 65 are
        // the neighbours that return even from a drain that wedges on a full sink.
        for queued in [63u64, 64, 65, 128] {
            for _ in 0..queued {
                hub.publish_with_interleave_for_test(
                    crate::catalog_events::CatalogDelta::default(),
                    1,
                    Vec::new(),
                    || {},
                );
            }
            let (returned, drain) =
                drain_or_panic(rx, &format!("{queued} wake bytes queued, writer held"));
            rx = returned;
            assert_eq!(
                drain.bytes, queued,
                "the drain must consume every queued wake byte"
            );
            assert!(drain.alive, "the writer is still held by the hub");
        }
    }

    /// A connection whose waker could not be ARMED is PACED by the read timeout —
    /// the degradation both messages promise — instead of spinning.
    ///
    /// `set_waker` takes the writer BY VALUE and DROPS it on the error arm, closing
    /// the pipe. A handler that kept polling that writer-less read end sees `POLLHUP`
    /// on every `poll`, takes the push arm, drains nothing, and goes straight round
    /// again — MEASURED at ~365k iterations/s, i.e. the busy-loop
    /// class, on the one path whose own warning says pushes "fall back to the
    /// read-timeout cadence". So `set_waker` REPORTS the failure and
    /// the handler drops its reader, and this drives the real `handle_connection`
    /// with the failure forced to prove it.
    ///
    /// The oracle is the busy-loop one: an iteration CEILING over a held-idle window.
    /// Load can only make a paced loop run FEWER passes, so contention delays this
    /// assertion but can never satisfy it by accident, while a spinner misses it by
    /// ~four orders of magnitude. The anti-vacuity floor + the served banner are what
    /// stop a handler that died early from passing the ceiling trivially.
    ///
    /// KILL SCOPE, stated because it is narrower than the name suggests and was
    /// MEASURED rather than assumed: the handler carries TWO guards against a
    /// writer-less read end — it believes `set_waker`'s answer, and it drops a reader
    /// whose drain reports EOF — and on THIS path they are redundant, because a failed
    /// arming DROPS the writer, which is exactly what EOF means. Reverting either
    /// guard alone leaves this arm green (RUN); reverting both fails it. The EOF guard
    /// gets its own isolating arm below, and `set_waker`'s answer its own beside that.
    #[test]
    fn a_connection_whose_waker_cannot_be_armed_is_paced_not_spinning() {
        let delta = drive_idle_connection(|_ctx, _conn| {}, true);
        assert_paced("a connection whose waker could not be ARMED", delta);
    }

    /// The EOF guard in ISOLATION: a waker armed normally and CLOSED afterwards must
    /// also degrade to the timeout cadence.
    ///
    /// This is the shape the arming arm cannot see. `clear_waker` drops the hub's
    /// `PushWaker`, closing the write end under a handler already polling its read
    /// end — so `set_waker` answered `true` and cannot help, and only the drain's EOF
    /// report can stop the poll reporting `Push` forever. The trigger is a real hub
    /// verb rather than a synthetic one: `HubState::wakers` is keyed by `ConnId`, so
    /// any path that clears or replaces an entry under a live handler produces it.
    #[test]
    fn a_waker_closed_under_a_live_handler_is_paced_not_spinning() {
        let delta = drive_idle_connection(
            // The handler is inside its loop with a HEALTHY waker; take the write end
            // away underneath it.
            |ctx, conn| ctx.events.clear_waker(conn),
            false,
        );
        assert_paced("a connection whose waker was CLOSED underneath it", delta);
    }

    /// How long the idle-pacing arms hold a connection open while counting passes.
    const IDLE_HOLD: Duration = Duration::from_millis(1200);

    /// Drive one real `handle_connection` to idle and report its loop-pass delta over
    /// [`IDLE_HOLD`].
    ///
    /// `after_hello` runs once the banner proves the handler reached its loop, so a
    /// disturbance lands on a live connection rather than racing its setup;
    /// `fail_arming` forces `PushWaker::new` to fail INSIDE the handler thread (the
    /// seam is thread-local, so no sibling test in this binary can observe it).
    fn drive_idle_connection(after_hello: impl FnOnce(&Ctx, ConnId), fail_arming: bool) -> u64 {
        let (client, server) = UnixStream::pair().expect("socketpair");
        let ctx = waker_ctx();
        let observed = ctx.clone();
        let iterations = Arc::clone(&ctx.conn_loop_iterations);
        let shutdown = Arc::new(AtomicBool::new(false));
        let handler_shutdown = Arc::clone(&shutdown);
        let handler = spawn_handler(server, ctx, handler_shutdown, fail_arming);

        // The banner proves the handler really reached its loop before the window.
        client
            .set_read_timeout(Some(Duration::from_secs(10)))
            .expect("read timeout");
        let mut reader = std::io::BufReader::new(client.try_clone().expect("clone"));
        let mut hello = String::new();
        std::io::BufRead::read_line(&mut reader, &mut hello).expect("hello line");
        assert!(
            hello.contains("protocol"),
            "the handler must greet before the idle window: {hello:?}"
        );
        // `handle_connection` takes the FIRST ConnId out of a fresh registry.
        after_hello(&observed, 0);

        let before = iterations.load(Ordering::Relaxed);
        std::thread::sleep(IDLE_HOLD);
        let delta = iterations.load(Ordering::Relaxed) - before;

        shutdown.store(true, Ordering::Release);
        drop(reader);
        drop(client);
        handler.join_or_panic();
        delta
    }

    /// A connection handler with a BOUNDED join.
    ///
    /// The handler under test is exactly the thing that can wedge, so a bare
    /// `JoinHandle::join()` at the end of an arm turns a caught defect into a hung
    /// binary — the failure class these tests exist to remove. It recovers today
    /// only because dropping the socket and (for the EOF arm) `clear_waker` closing
    /// the writer both unblock it; neither is a property the handler guarantees.
    struct BoundedHandler {
        handle: std::thread::JoinHandle<()>,
        done: std::sync::mpsc::Receiver<()>,
    }

    impl BoundedHandler {
        /// Wait for the handler to EXIT, then join it (propagating any panic).
        ///
        /// A generous liveness ceiling in seconds against a loop paced at 200 ms —
        /// load can delay this, never invert it. `Disconnected` means the thread is
        /// already gone (its sender dropped), so the join returns at once and carries
        /// the real panic rather than this one.
        fn join_or_panic(self) {
            match self.done.recv_timeout(Duration::from_secs(30)) {
                Ok(()) | Err(std::sync::mpsc::RecvTimeoutError::Disconnected) => self
                    .handle
                    .join()
                    .expect("the handler thread must exit cleanly"),
                Err(std::sync::mpsc::RecvTimeoutError::Timeout) => panic!(
                    "the connection handler did not exit within 30s of the shutdown \
                     flag and an EOF on its socket — it is WEDGED, most likely parked \
                     inside `drain_waker` on a read end that was never armed \
                     non-blocking."
                ),
            }
        }
    }

    fn spawn_handler(
        server: UnixStream,
        ctx: Ctx,
        shutdown: Arc<AtomicBool>,
        fail_arming: bool,
    ) -> BoundedHandler {
        let (done_tx, done) = std::sync::mpsc::channel();
        let handle = std::thread::spawn(move || {
            // Armed INSIDE the handler thread: `PushWaker::new` runs here, and the
            // seam is thread-local so no sibling test in this binary can see it.
            crate::catalog_events::fail_waker_arming_on_this_thread_for_test(fail_arming);
            handle_connection(server, ctx, shutdown);
            let _ = done_tx.send(());
        });
        BoundedHandler { handle, done }
    }

    /// The shared verdict: a paced loop, not a spin, with anti-vacuity.
    fn assert_paced(what: &str, delta: u64) {
        // Hand oracle: one pass per CONN_READ_TIMEOUT ⇒ 1200/200 = 6. The ceiling is
        // 5× that; a spinning handler reaches hundreds of thousands.
        let expected = (IDLE_HOLD.as_nanos() / CONN_READ_TIMEOUT.as_nanos()) as u64;
        let ceiling = expected * 5;
        assert!(
            delta <= ceiling,
            "{what} went round its loop {delta} times in {IDLE_HOLD:?} (expected \
             ~{expected}, ceiling {ceiling}) — it kept a read end with no writer, so \
             every poll reports POLLHUP and the handler is SPINNING rather than \
             falling back to the read-timeout cadence"
        );
        assert!(
            delta >= 2,
            "the loop ran only {delta} times in {IDLE_HOLD:?} — the handler is not \
             alive, so the ceiling above proves nothing"
        );
    }

    /// A waker-less connection still RECEIVES its catalog pushes — the other half of
    /// what both degradation warnings promise.
    ///
    /// They say pushes "fall back to the read-timeout cadence", which is TWO claims:
    /// the loop is paced (its arms above) and the push still ARRIVES. Only the first
    /// was pinned, so moving the `take_pending` drain under `if waker_rx.is_some()`
    /// would leave a degraded connection paced, quiet, and permanently deaf to every
    /// catalog change — with the pacing arms and every e2e arm (which all arm their
    /// wakers successfully) still green.
    ///
    /// Driven through the REAL handler with arming forced to fail: subscribe over the
    /// socket, publish through the hub, and require the event line. The read budget is
    /// a generous liveness ceiling against a 200 ms cadence, not a latency claim.
    #[test]
    fn a_waker_less_connection_still_receives_its_catalog_pushes() {
        use std::io::{BufRead, Write};

        let (client, server) = UnixStream::pair().expect("socketpair");
        let ctx = waker_ctx();
        let hub = Arc::clone(&ctx.events);
        let shutdown = Arc::new(AtomicBool::new(false));
        let handler = spawn_handler(server, ctx, Arc::clone(&shutdown), true);

        client
            .set_read_timeout(Some(Duration::from_secs(10)))
            .expect("read timeout");
        let mut reader = std::io::BufReader::new(client.try_clone().expect("clone"));
        let mut line = String::new();
        reader.read_line(&mut line).expect("hello line");
        assert!(line.contains("protocol"), "banner first: {line:?}");

        // Subscribe over the wire, so the hub state is whatever the REAL verb sets.
        let mut w = client.try_clone().expect("clone");
        writeln!(w, "{}", Request::SubscribeCatalog { id: 1 }.to_json_line()).expect("subscribe");
        w.flush().expect("flush");
        line.clear();
        reader.read_line(&mut line).expect("subscribe response");
        assert!(
            line.contains("\"watching\"") && !line.contains(crate::protocol::CATALOG_CHANGED_EVENT),
            "the subscribe SNAPSHOT must be answered before the publish: {line:?}"
        );
        assert_eq!(hub.subscriber_count(), 1, "the connection is subscribed");
        assert_eq!(
            hub.waker_count(),
            0,
            "and holds NO waker (else this arm is not testing the degraded path)"
        );

        hub.publish_with_interleave_for_test(
            crate::catalog_events::CatalogDelta {
                topics_added: 1,
                ..Default::default()
            },
            1,
            Vec::new(),
            || {},
        );

        line.clear();
        reader
            .read_line(&mut line)
            .expect("a waker-less connection must still be delivered its catalog push");
        assert!(
            line.contains(crate::protocol::CATALOG_CHANGED_EVENT),
            "the pushed line must be the catalog-change event: {line:?}"
        );

        shutdown.store(true, Ordering::Release);
        drop(reader);
        drop(w);
        drop(client);
        handler.join_or_panic();
    }

    /// The other half of that contract, in isolation: `set_waker` answers TRUTHFULLY.
    ///
    /// Without it, "reports the failure" is satisfied by a `set_waker` that returns
    /// `false` unconditionally — which would ship the waker permanently disabled and
    /// still pass the pacing arm above (a never-armed connection is paced too).
    #[test]
    fn set_waker_reports_whether_it_actually_armed() {
        let hub = waker_hub();

        let (_rx, tx) = std::io::pipe().expect("pipe");
        assert!(hub.set_waker(1, tx), "a healthy pipe ARMS");
        assert_eq!(hub.waker_count(), 1, "and is registered");

        crate::catalog_events::fail_waker_arming_on_this_thread_for_test(true);
        let (_rx2, tx2) = std::io::pipe().expect("pipe");
        let armed = hub.set_waker(2, tx2);
        crate::catalog_events::fail_waker_arming_on_this_thread_for_test(false);
        assert!(
            !armed,
            "an unarmable writer is REPORTED — the handler cannot otherwise learn that \
             its read end has no writer left"
        );
        assert_eq!(
            hub.waker_count(),
            1,
            "and nothing was registered for that connection"
        );
    }

    /// A waker whose writers are all gone reports DEAD, and the poll really would
    /// spin on it — which is why the handler drops it.
    ///
    /// The two halves belong together: `wait_for_control_input` returning `Push`
    /// forever is the HAZARD, and `drain_waker` reporting `alive: false` is the only
    /// signal the handler has to act on it. Pinning either alone leaves the pair free
    /// to drift back into a spin.
    #[test]
    fn a_waker_whose_writers_are_gone_reports_dead_rather_than_spinning() {
        let (rx, tx) = mint_waker_pipe().expect("mint");

        // Alive while a writer is held, even with nothing to read. Bounded, because
        // THIS is the call an unarmed read end parks on forever.
        let (rx, drain) = drain_or_panic(rx, "empty pipe, writer held");
        assert_eq!(drain.bytes, 0);
        assert!(drain.alive, "an empty pipe with a live writer is not EOF");

        drop(tx);

        // The HAZARD: an EOF'd read end is permanently ready, so the wait reports a
        // push on every call and the loop would never reach its timeout.
        let (sock, _peer) = UnixStream::pair().expect("socketpair");
        for _ in 0..3 {
            assert!(
                matches!(
                    wait_for_control_input(&sock, Some(&rx), Duration::from_millis(50)),
                    ControlWake::Push
                ),
                "a closed write end leaves the read end POLLHUP-ready forever"
            );
        }

        // The SIGNAL that lets the handler escape it.
        let (_rx, drain) = drain_or_panic(rx, "empty pipe, every writer dropped (EOF)");
        assert_eq!(drain.bytes, 0);
        assert!(
            !drain.alive,
            "EOF must be reported so the handler drops the reader and falls back to \
             the read-timeout cadence its own warning promises"
        );
    }

    // NOTE: the pure write-outcome classifier + its oracle-vector tests now
    // live with the SHARED impl in `cerulion_core::transport::uds_write` (used by both
    // netd and vizd); the netd big-line round-trip regression + dead-consumer pins are
    // in `tests/daemon_e2e_test.rs`.
}
