// SPDX-License-Identifier: AGPL-3.0-only
//! The iroh WAN [`MirrorPlane`] — the WAN half of the
//! dual-plane, one-endpoint model, folding cerulion_connectd's re-inject engine into
//! netd.
//!
//! A robot is ONE identity reachable over two transports (zenoh on the LAN, iroh over
//! the WAN), and netd owns THE one mirror publisher per `(robot, topic)` for BOTH.
//! Where the zenoh [`GatewayMirrorPlane`](crate::mirror::GatewayMirrorPlane) ensures a
//! mirror via `register_ingress_topic` (a zenoh subscriber + the SHM mirror
//! publisher), this plane ensures the SAME kind of mirror over iroh: it dials the
//! robot's `cerulion/wire/1` ALPN, demands the topic over the shared `cerulion_q`
//! control vocabulary, and re-injects each per-topic uni-stream frame into desk SHM
//! through the SHARED validated re-inject primitive
//! [`IngressInjector::reinject_raw`](cerulion_core::transport::network::IngressInjector::reinject_raw)
//! (the `total_size` + `schema_hash` gate — the SAME primitive
//! `cerulion connect`'s `run_topic_reader` drives, so there is ONE validated re-inject
//! implementation, never a fork).
//!
//! # LAN parity: the injector is created SYNCHRONOUSLY (no phantom mirror)
//!
//! The zenoh plane's `register_ingress_topic` fails SYNCHRONOUSLY + loudly when the
//! desk-local single-writer mirror slot is already taken (a local producer / a second
//! mirror). This plane matches that: `ensure_mirror` creates the desk-local
//! [`IngressInjector`] SYNCHRONOUSLY — the injector's existence IS the mirror's
//! readiness — BEFORE spawning the background reader. A slot-taken failure fails the
//! demand with NO mirror state left behind, so the daemon never commits a refcount +
//! presence on a dead mirror (the phantom-mirror class).
//!
//! # Bounded everywhere (no admit-then-stall wedge)
//!
//! netd's daemon calls [`MirrorPlane::ensure_mirror`] / [`MirrorPlane::release_mirror`]
//! WHILE holding the registry lock. iroh is async, so this plane owns a dedicated
//! multi-thread tokio [`Runtime`] and each `MirrorPlane` method `block_on`s a bounded
//! op: EVERY await in the dial/demand path (endpoint bind, dial, open control stream,
//! each control write + read, uni-stream accept, preamble read) is wrapped in a
//! `bounded` timeout so an admit-then-stall peer fails FAST + LOUD rather than pinning
//! the control lock indefinitely. Moving that network I/O off the lock entirely (a
//! per-key background ensure) would be a broader daemon refactor; it is not done here.
//!
//! # Connection health (no orphaned / poisoned connections)
//!
//! A control-stream error/timeout desyncs the (cancel-unsafe) framing, so a
//! per-robot connection whose control round-trip failed is marked `control_poisoned`
//! and DROPPED (the next demand re-dials a fresh connection). A demand that fails after
//! the dial (e.g. a slot-taken injector) also drops a now-readerless connection, so a
//! failed demand never leaves an orphaned connection wedging the robot.
//!
//! # Reader-death observability (mid-stream disconnect)
//!
//! Each per-topic reader runs to completion in a supervising task. On an INTENTIONAL
//! release the task is aborted (the death handler is cancelled before it runs); on a
//! NATURAL exit (the robot dropped the connection / tap) the death handler fires a
//! LOUD `error!`, drops the SHM injector (frees the slot), removes the reader + its
//! provenance, and drops the connection if it was the last — so the plane never keeps
//! a zombie mirror. Clearing the DAEMON's refcount/presence so a re-demand
//! auto-re-creates (rather than short-circuiting to `AlreadyMirrored`) needs a
//! plane→daemon death-notification seam that does not exist yet.
//!
//! # Pairing / trust preserved
//!
//! The dial uses the robot's PINNED [`EndpointId`](cerulion_link::EndpointId) (from
//! the WAN registry) + the desk's device seed — so the QUIC/TLS device-key
//! authentication is unchanged: a mis-keyed robot fails the peer-identity check, and
//! an unpaired desk key is REFUSED by the robot's `PairingAuthorizer` (surfaced here
//! as a loud [`MirrorError::Iroh`] at the catalog gate). netd folds in the re-inject
//! engine; it does NOT touch the robot's accept gate.
//!
//! Whole module `#[cfg(feature = "wan")]` — `wan` is DEFAULT-ON (the shipped netd
//! includes the iroh WAN plane; `--no-default-features` is the lean opt-out). netd is
//! kept out of `default-members` so a plain `cargo build` stays iroh-free.

use std::collections::{HashMap, HashSet};
use std::future::Future;
use std::sync::{Arc, Weak};
use std::time::Duration;

use cerulion_core::transport::demand_authorizer::{
    AllowAllAuthorizer, DemandAuthorizer, DemandDecision, DemandSubject,
};
use cerulion_core::transport::network::{IngressInjector, ReinjectOutcome};
use cerulion_core::wire::MaxSliceLen;
use cerulion_core::TransportManager;
use cerulion_link::{
    accept_uni_frame_stream, alpn, build_endpoint, dial, direct_addr, open_frame_stream,
    read_frame, write_frame, Connection, Endpoint, EndpointAddr, EndpointConfig, RecvStream,
    SendStream, DEFAULT_MAX_FRAME_LEN,
};
use cerulion_pairing::client::DeviceIdentity;
// The SHARED epoch-push substrate — the SAME decisions the `cerulion connect`
// verb makes on ITS dial (one implementation, never a fork).
use cerulion_wireclient::epoch::{
    classify_epoch_reply, note_transport_failure, prepare_epoch_push, EpochPushPlan,
};
pub use cerulion_wireclient::epoch::{EpochPushOutcome, EpochPushSeverity};
use cerulion_wireclient::protocol::{
    decode_first_reply, StreamPreamble, WireRequest, WireResponse,
};
use tokio::runtime::Runtime;
use tokio::sync::Mutex as AsyncMutex;
use tokio::task::JoinHandle;

use crate::mirror::{MirrorError, MirrorPlane, MirrorRelease};
use crate::registry::TopicKey;
use crate::wan::WanRegistry;

/// The default bound on a WAN dial + each control/stream await. Held under the
/// daemon's registry lock (see the module docs), so it is SHORTER than
/// `cerulion connect`'s 30 s default: a slow/unreachable WAN robot must fail fast +
/// loud rather than pin every other control op. Overridable via
/// [`IrohMirrorPlane::with_dial_timeout`] (tests shorten it).
pub const DEFAULT_WAN_DIAL_TIMEOUT: Duration = Duration::from_secs(15);

/// The per-mirror ingress buffer ceiling — DERIVED from cerulion_core's
/// `graph::config::DEFAULT_MAX_SLICE_LEN` (128 MiB), the SAME value netd's zenoh plane
/// (`register_ingress_topic`) and the graph build use, so BOTH planes provision the
/// mirror identically (a topic behaves the same regardless of which plane serves it).
/// A frame larger than the wire read cap ([`DEFAULT_MAX_FRAME_LEN`], 16 MiB) is
/// refused before it reaches the injector, and iceoryx2 Static pools are lazy, so an
/// oversized ceiling costs nothing until used.
const MIRROR_MAX_SLICE_LEN: MaxSliceLen =
    MaxSliceLen::const_new(cerulion_core::graph::config::DEFAULT_MAX_SLICE_LEN as u32);

/// One live iroh connection to a WAN robot: its QUIC connection, the ONE bidi control
/// stream (reused for every demand/undemand), the per-topic reader tasks, and a
/// poison flag set when a control round-trip desyncs the (cancel-unsafe) framing.
struct RobotConn {
    connection: Connection,
    control_send: SendStream,
    control_recv: RecvStream,
    /// topic → the supervising reader task. Aborting it on release cancels BEFORE the
    /// death handler runs; a natural exit (robot disconnect) runs the death handler.
    readers: HashMap<String, JoinHandle<()>>,
    /// Set when a control round-trip errors/times out — the framing is desynced, so
    /// this connection is fatal for future demands and must be re-dialed.
    control_poisoned: bool,
}

/// The plane's mutable async state (behind a tokio [`AsyncMutex`] so it can be held
/// across `.await`s — the daemon serializes calls anyway, so contention is nil): the
/// LAZY desk iroh endpoint (built on the first ensure) + the per-robot connections.
struct IrohState {
    /// The desk's iroh endpoint (wire ALPN only — the desk never accepts). Built
    /// lazily on the first `ensure_mirror`.
    endpoint: Option<Endpoint>,
    /// robot name → its live connection.
    robots: HashMap<String, RobotConn>,
}

/// The iroh WAN mirror plane. Owns the desk's iroh endpoint, the per-robot
/// connections, and a dedicated tokio runtime, and re-injects each demanded topic
/// into `manager`'s desk SHM. `Send + Sync` (shared across the daemon threads behind
/// the trait object).
pub struct IrohMirrorPlane {
    /// The desk's local SHM transport (SHARED with the zenoh plane — the ONE mirror
    /// per topic lives here). `create_ingress_injector` is zenoh-free.
    manager: Arc<TransportManager>,
    /// The WAN robot registry (dial params + the desk seed + relay posture).
    registry: Arc<WanRegistry>,
    /// The dedicated tokio runtime the async iroh world runs on.
    rt: Runtime,
    /// The lazy endpoint + per-robot connections.
    inner: Arc<AsyncMutex<IrohState>>,
    /// The bound on a dial + each control/stream await.
    dial_timeout: Duration,
    /// Netd's OWN desk device key (the public half of the WAN registry's desk
    /// seed) — the DEMANDER identity threaded onto the pre-flight
    /// [`DemandSubject::Wan`]. netd is the party asking the robot for the topic, so its
    /// own desk key (the key the robot TLS-authenticates + access-lists) is the true
    /// demander — NEVER the dial target. Derived once at construction.
    desk_key: [u8; 32],
    /// The WAN half of the ONE demand-authorization seam — consulted
    /// BEFORE dialing so an unauthorized WAN demand never opens a connection. This
    /// COMPOSES WITH (never replaces) the robot's own `PairingAuthorizer` accept at the
    /// `cerulion/wire/1` ALPN (which still refuses an unpaired desk key on the wire); it
    /// is netd's desk-side pre-flight gate. It STAYS the deny-nothing
    /// [`AllowAllAuthorizer`] (byte-identical): the robot is the authoritative
    /// per-demand enforcement point — the `is_allowed`-backed
    /// `PairingAuthorizer` is installed on the ROBOT serving plane, NOT here
    /// (netd holds no robot access list). A desk-side authorizer that ever DID resolve
    /// this subject would resolve netd's OWN desk account (`desk_key`), not the dial
    /// target's — the true client-side identity.
    authorizer: Arc<dyn DemandAuthorizer>,
    /// Robot → the outcome of its LAST revocation-epoch push, so a
    /// non-delivery is observable state rather than log-only (Principle #3). Read via
    /// [`Self::last_push_outcome`].
    epoch_pushes: Arc<AsyncMutex<HashMap<String, EpochPushOutcome>>>,
    /// Robots whose missing epoch cache has already been logged, so the
    /// no-cached-epoch `info!` fires once per (robot, process) instead of on every
    /// reconnect. A plain `std` mutex (no `.await` is ever held across it) so the
    /// shared push substrate can consult it as a synchronous gate.
    missing_cache_noted: Arc<std::sync::Mutex<HashSet<String>>>,
}

impl IrohMirrorPlane {
    /// Build the iroh WAN plane over the desk's `manager` (its SHM mirror target) and
    /// `registry` (the WAN dial params). Builds the dedicated tokio runtime; binds NO
    /// iroh endpoint yet (the endpoint is lazy — bound on the first ensure).
    ///
    /// # Errors
    ///
    /// If the tokio runtime cannot be built.
    pub fn new(manager: Arc<TransportManager>, registry: Arc<WanRegistry>) -> Result<Self, String> {
        let rt = tokio::runtime::Builder::new_multi_thread()
            .worker_threads(2)
            .enable_all()
            .thread_name("netd-iroh")
            .build()
            .map_err(|e| format!("failed to build the cerulion-netd iroh runtime: {e}"))?;
        // Derive netd's OWN desk device key (the demander identity) from the
        // WAN registry's desk seed — the SAME ed25519 public half the endpoint's iroh
        // id would carry, computed here without binding the (lazy) endpoint so the
        // pre-flight gate can run before a dial.
        let desk_key = DeviceIdentity::from_seed(&registry.desk_seed())
            .public_key()
            .0;
        Ok(Self {
            manager,
            registry,
            rt,
            inner: Arc::new(AsyncMutex::new(IrohState {
                endpoint: None,
                robots: HashMap::new(),
            })),
            dial_timeout: DEFAULT_WAN_DIAL_TIMEOUT,
            desk_key,
            authorizer: Arc::new(AllowAllAuthorizer),
            epoch_pushes: Arc::new(AsyncMutex::new(HashMap::new())),
            missing_cache_noted: Arc::new(std::sync::Mutex::new(HashSet::new())),
        })
    }

    /// Override the dial / await timeout (tests shorten it to catch a stalling /
    /// unreachable robot fast).
    pub fn with_dial_timeout(mut self, dial_timeout: Duration) -> Self {
        self.dial_timeout = dial_timeout;
        self
    }

    /// Install the WAN demand-authorization pre-flight gate. In production
    /// this STAYS the deny-nothing [`AllowAllAuthorizer`] — the robot's serving-side
    /// `PairingAuthorizer` (installed on the robot) is the authoritative
    /// per-demand enforcement point; netd's desk-side gate is a pre-dial hook that holds
    /// no robot access list. Tests inject a stub deny-authorizer to prove the WAN
    /// pre-dial refusal short-circuits ahead of any endpoint bind / dial.
    pub fn with_authorizer(mut self, authorizer: Arc<dyn DemandAuthorizer>) -> Self {
        self.authorizer = authorizer;
        self
    }

    /// The number of live WAN robot connections (diagnostics / tests).
    pub fn connection_count(&self) -> usize {
        self.rt
            .block_on(async { self.inner.lock().await.robots.len() })
    }

    /// The number of live per-topic readers across all robots (diagnostics / tests).
    pub fn reader_count(&self) -> usize {
        self.rt.block_on(async {
            self.inner
                .lock()
                .await
                .robots
                .values()
                .map(|r| r.readers.len())
                .sum()
        })
    }

    /// Whether this plane's WAN registry has RESOLVED the desk's logged-in account yet.
    /// A straight delegation to
    /// [`WanRegistry::account_resolved`], i.e. the state of the `OnceLock` that HOLDS
    /// the deferred work (the device-cert read + I1 ed25519 key match).
    ///
    /// It reads the REGISTRY cell deliberately, not a plane-side "we called it" flag: a
    /// flag set inside the plane's own `resolve_dial_account` would stay `false` when the
    /// resolution moved back to boot and `true` when it never happened at all, so it
    /// could not distinguish resolved-at-dial from resolved-at-boot — the exact contract
    /// this observable exists to pin. `false` on a plane whose registry has never been
    /// asked IS the contract: no dial ⇒ no cert read, no ed25519 derivation, no
    /// boot cost.
    pub fn desk_account_resolved(&self) -> bool {
        self.registry.account_resolved()
    }

    /// Resolve (once per process, via the registry's `OnceLock`) the logged-in account
    /// this desk's WAN dials carry, and record this dial's posture.
    ///
    /// This is the PRODUCTION first-use point for [`WanRegistry::account`]. That
    /// resolution was deferred off netd's boot path to "first use", but the only
    /// callers were tests, so "first use" was really "never" and the device-cert
    /// diagnostics inside `resolve_account_from` (notably the WARN for a stale/foreign
    /// cert, or a `CERULION_NETD_DESK_KEY` pointing at a different key than the one that
    /// logged in) were unreachable in production. Those diagnostics matter most at
    /// exactly this moment: a misconfigured cert means the robot refuses this dial at
    /// the catalog gate, and the operator needs the CAUSE named.
    ///
    /// Placed at the TOP of [`Self::dial_robot`], before the DIAL's network I/O, so the
    /// classification lands ABOVE the catalog-gate refusal it explains. (Not before ALL
    /// network I/O: `ensure_async` binds the desk endpoint — UDP sockets, and the relay
    /// handshake unless relays are disabled — before it calls `dial_robot`. An endpoint
    /// bind failure is its own loud, self-explaining error and needs no account
    /// classification above it.) A netd that never dials still pays nothing: the deferral's
    /// whole point is preserved, it is now genuinely "first dial" rather than "never".
    ///
    /// The account is not yet PRESENTED on the wire — a separate seam owns that. What is
    /// live today is the diagnostic and the observable
    /// ([`Self::desk_account_resolved`], which reads the registry cell this call
    /// initializes).
    fn resolve_dial_account(&self, robot: &str) {
        let account = self.registry.account();
        // The classifying info/warn lines fire once per process inside the registry;
        // this per-dial line says which posture THIS dial ran under.
        tracing::debug!(
            robot = %robot,
            account_present = account.is_some(),
            "cerulion-netd: WAN dial account posture resolved"
        );
    }

    /// Wrap an await in the dial/control timeout budget — the single seam that makes
    /// EVERY step of the demand/dial path bounded. `phase` names the step
    /// in the loud timeout error.
    async fn bounded<F: Future>(&self, phase: &str, fut: F) -> Result<F::Output, String> {
        tokio::time::timeout(self.dial_timeout, fut)
            .await
            .map_err(|_| {
                format!(
                    "iroh {phase} timed out after {}ms (an admit-then-stall / unreachable peer)",
                    self.dial_timeout.as_millis()
                )
            })
    }

    /// Build the desk iroh endpoint (wire ALPN only, the registry's desk seed +
    /// relay). Bounded.
    async fn build_desk_endpoint(&self) -> Result<Endpoint, String> {
        self.bounded(
            "endpoint bind",
            build_endpoint(
                EndpointConfig::new(self.registry.desk_seed())
                    .with_alpns(vec![alpn::WIRE.to_vec()])
                    .with_relay(self.registry.relay().clone()),
            ),
        )
        .await?
        .map_err(|e| format!("desk iroh endpoint bind failed: {e}"))
    }

    /// Dial a WAN robot, open its control stream, and gate on the catalog fetch (which
    /// surfaces an unpaired/unclaimed REFUSAL). Every await is bounded. Returns a fresh
    /// [`RobotConn`] with no readers.
    async fn dial_robot(&self, endpoint: &Endpoint, robot: &str) -> Result<RobotConn, String> {
        let wan = self.registry.get(robot).ok_or_else(|| {
            format!(
                "robot '{robot}' has no WAN endpoint configured (not in the WAN registry) — \
                 cannot dial it over iroh"
            )
        })?;
        // The production FIRST-USE point for the desk account, placed
        // before this dial's network I/O, so a device-cert misconfiguration is
        // classified ABOVE the catalog-gate refusal it causes. See
        // `resolve_dial_account`.
        self.resolve_dial_account(robot);
        let peer: EndpointAddr = if wan.direct_addrs.is_empty() {
            EndpointAddr::new(wan.eid)
        } else {
            direct_addr(wan.eid, wan.direct_addrs.iter().copied())
        };
        let connection = self
            .bounded("dial", dial(endpoint, peer, alpn::WIRE))
            .await?
            .map_err(|e| format!("dial robot '{robot}' over iroh: {e}"))?;
        let (mut control_send, mut control_recv) = self
            .bounded("open control stream", open_frame_stream(&connection))
            .await?
            .map_err(|e| format!("open control stream to robot '{robot}': {e}"))?;
        // The catalog fetch is the ADMISSION gate: an unpaired desk key / unclaimed
        // robot is routed to the robot's skeleton path, which answers an AcceptDecision
        // here → a loud refusal (never a silent dead connection).
        self.fetch_catalog(&mut control_send, &mut control_recv, robot)
            .await?;
        // Now that we are ADMITTED, deliver the robot's latest revocation
        // epoch. Ordered AFTER the catalog gate so an unpaired/unclaimed refusal keeps
        // its own precise message; ordered BEFORE the connection is handed out so the
        // robot's ACL is current for everything that follows.
        //
        // Epoch-POLICY failures (no cache, corrupt cache, robot refusal, an older robot
        // that does not know the verb) are logged and IGNORED — "never block the
        // connection on epoch freshness". Only a control-stream TRANSPORT failure
        // propagates, because it leaves the framing desynced and every later demand on
        // this connection would fail anyway (see `push_epoch`'s docs).
        self.push_epoch(&mut control_send, &mut control_recv, robot)
            .await?;
        tracing::info!(
            robot = %robot,
            "cerulion-netd: iroh WAN connection established (admitted)"
        );
        Ok(RobotConn {
            connection,
            control_send,
            control_recv,
            readers: HashMap::new(),
            control_poisoned: false,
        })
    }

    /// One control round-trip on a raw stream pair (write a request, read its reply) —
    /// BOTH awaits bounded. Used at dial time (before a [`RobotConn`] exists).
    async fn control_round_trip(
        &self,
        send: &mut SendStream,
        recv: &mut RecvStream,
        request: &WireRequest,
    ) -> Result<Vec<u8>, String> {
        let bytes =
            serde_json::to_vec(request).map_err(|e| format!("encode control request: {e}"))?;
        self.control_round_trip_encoded(send, recv, &bytes).await
    }

    /// The same round-trip over a PRE-ENCODED request frame — the seam the
    /// epoch push uses, because the shared substrate encodes the frame itself in order
    /// to size-check it against the peer's frame cap BEFORE anything is written.
    async fn control_round_trip_encoded(
        &self,
        send: &mut SendStream,
        recv: &mut RecvStream,
        frame: &[u8],
    ) -> Result<Vec<u8>, String> {
        self.bounded("control write", write_frame(send, frame))
            .await?
            .map_err(|e| format!("control write: {e}"))?;
        self.bounded("control read", read_frame(recv, DEFAULT_MAX_FRAME_LEN))
            .await?
            .map_err(|e| format!("control read: {e}"))
    }

    /// A control round-trip on a [`RobotConn`] — POISONS the connection on any
    /// error/timeout (the cancel-unsafe framing is desynced, so the connection is
    /// fatal for future demands and must be re-dialed).
    async fn robot_round_trip(
        &self,
        rc: &mut RobotConn,
        request: &WireRequest,
    ) -> Result<Vec<u8>, String> {
        let res = self
            .control_round_trip(&mut rc.control_send, &mut rc.control_recv, request)
            .await;
        if res.is_err() {
            rc.control_poisoned = true;
        }
        res
    }

    /// Fetch the catalog — the admission gate. A refusal (`AcceptDecision`) becomes a
    /// loud reason; the catalog itself is discarded (the demand carries the hash).
    async fn fetch_catalog(
        &self,
        send: &mut SendStream,
        recv: &mut RecvStream,
        robot: &str,
    ) -> Result<(), String> {
        let bytes = self
            .control_round_trip(send, recv, &WireRequest::Catalog)
            .await?;
        match decode_first_reply(&bytes) {
            Ok(WireResponse::Catalog(_)) => Ok(()),
            Ok(WireResponse::Error { message, .. }) => Err(message),
            Ok(other) => Err(format!(
                "robot '{robot}' answered the catalog request with an unexpected reply: {other:?}"
            )),
            Err(Some(reason)) => Err(format!(
                "robot '{robot}' refused the wire plane: {reason} (pair the desk first: \
                 `cerulion pair {robot}`)"
            )),
            Err(None) => Err(format!(
                "robot '{robot}' reply decoded as neither a wire response nor an accept decision \
                 (an incompatible robot or a corrupt stream)"
            )),
        }
    }

    /// PUSH this desk's cached revocation epoch to `robot` — the mechanism by
    /// which an owner's revocation reaches the robot at all (by design:
    /// desk-push on connect).
    ///
    /// Two failure classes, deliberately treated differently:
    ///
    /// - **Epoch-POLICY failures never block the connection** (issue point 3): no
    ///   cache, a corrupt cache, an older robot that answers `Error` for the unknown
    ///   verb, or a robot that REJECTS the epoch. Each is LOGGED at a level matching
    ///   how much an operator should care, RECORDED in [`Self::last_push_outcome`], and
    ///   the dial proceeds — a desk that cannot deliver a revocation must still reach
    ///   its robot (never-bricks-offline). The one thing these must never be is SILENT.
    /// - **A control round-trip TRANSPORT failure is fatal to the dial** (`Err`). The
    ///   framing is cancel-unsafe, so a timeout/desync mid-push leaves the control
    ///   stream unusable; handing that connection out would surface later as a
    ///   mysterious demand failure. Failing here is symmetric with `fetch_catalog`
    ///   (which fails the dial on the same class) and the caller re-dials cleanly. It is
    ///   not a NEW failure class — a peer that stalls mid-catalog already fails the dial.
    ///
    /// The push deliberately rides the SHARED control stream rather than a dedicated
    /// one: the robot's `serve_wire_connection` accepts EXACTLY ONE bidi stream, so a
    /// second stream would never be answered and every push would silently time out.
    /// (Giving the epoch its own stream — which would demote a stall to a mere policy
    /// failure — requires a robot-side accept loop, so it is not a
    /// desk-side choice.)
    ///
    /// Back-compat is safe: a robot that predates this verb fails to deserialize the
    /// request and answers `WireResponse::Error` carrying the shared
    /// `UNDECODABLE_REQUEST_NEEDLE` — classified as
    /// [`EpochPushOutcome::NoSink`] ("upgrade the robot"), a POLICY failure, not a
    /// framing break. Pinned by `cerulion_connectd/tests/protocol_parity_test.rs`.
    ///
    /// Every decision here (what to send, the size guard, what the answer meant) is the
    /// SHARED [`cerulion_wireclient::epoch`] substrate — the exact code the
    /// `cerulion connect` verb runs on its own dial, so the two desk paths can never
    /// diverge in what a revocation push does or reports.
    async fn push_epoch(
        &self,
        send: &mut SendStream,
        recv: &mut RecvStream,
        robot: &str,
    ) -> Result<(), String> {
        let result = self.push_epoch_inner(send, recv, robot).await;
        // Record the outcome for BOTH classes — including the fatal transport one, so
        // `last_push_outcome` can never read stale-successful after a failed push
        // (Principle #3: the observable must reflect the LAST attempt).
        let outcome = match &result {
            Ok(o) => *o,
            Err(_) => EpochPushOutcome::TransportFailed,
        };
        self.epoch_pushes
            .lock()
            .await
            .insert(robot.to_string(), outcome);
        result.map(|_| ())
    }

    /// The push body — returns the classified outcome its caller records, or `Err` only
    /// for the one fatal class (a control round-trip transport failure).
    async fn push_epoch_inner(
        &self,
        send: &mut SendStream,
        recv: &mut RecvStream,
        robot: &str,
    ) -> Result<EpochPushOutcome, String> {
        let cache_path = self.registry.epoch_cache_path(robot);
        // netd re-dials, so the "no cached epoch" line is deduped to once per
        // (robot, process). The gate is consulted ONLY on the missing branch, so a
        // cache that DISAPPEARS after a healthy dial still gets its line.
        let frame = match prepare_epoch_push(robot, cache_path.as_deref(), &|| {
            self.note_missing_cache(robot)
        }) {
            EpochPushPlan::Skip(outcome) => return Ok(outcome),
            EpochPushPlan::Send { frame } => frame,
        };
        // A transport/framing failure here leaves the control stream desynced — fatal
        // to the dial (see the fn docs), never a connection we hand out.
        let bytes = match self.control_round_trip_encoded(send, recv, &frame).await {
            Ok(b) => b,
            Err(e) => {
                note_transport_failure(robot, &e);
                return Err(format!("revocation-epoch push to robot '{robot}': {e}"));
            }
        };
        Ok(classify_epoch_reply(robot, &bytes))
    }

    /// Record that `robot` had no cached epoch; `true` the FIRST time per process so the
    /// caller logs once instead of on every reconnect.
    ///
    /// SYNCHRONOUS (a `std::sync::Mutex` over a tiny, uncontended set with no `.await`
    /// inside) so it can be handed to the shared substrate as a plain `Fn() -> bool`
    /// gate that is consulted ONLY when there is genuinely no cached artifact.
    fn note_missing_cache(&self, robot: &str) -> bool {
        match self.missing_cache_noted.lock() {
            Ok(mut set) => set.insert(robot.to_string()),
            Err(poisoned) => {
                // A poisoned lock must never SUPPRESS a delivery-visibility line, and
                // must never be swallowed either: recover the set (the data is a plain
                // string set — a panic mid-insert cannot leave it inconsistent) and say
                // so, since a poisoned mutex means some other thread panicked.
                tracing::warn!(
                    robot = %robot,
                    "cerulion-netd: the revocation-epoch missing-cache dedup lock was POISONED \
                     (a thread panicked while holding it) — recovering it and continuing; the \
                     no-cached-epoch line may repeat"
                );
                poisoned.into_inner().insert(robot.to_string())
            }
        }
    }

    /// Principle #3: the last recorded epoch-push outcome for `robot`, so
    /// "did this desk actually deliver the revocation?" is answerable as STATE, not only
    /// by grepping logs. `None` until a dial has run.
    pub fn last_push_outcome(&self, robot: &str) -> Option<EpochPushOutcome> {
        self.rt
            .block_on(async { self.epoch_push_outcome_async(robot).await })
    }

    /// Demand ONE topic on an existing connection: send `Demand`, await
    /// `DemandAccepted`, accept + correlate the uni data stream, create the desk-local
    /// injector SYNCHRONOUSLY (LAN parity — the mirror's readiness), register C0
    /// provenance, then spawn the supervising re-inject reader. Called ONCE per live
    /// mirror (the refcount plane guarantees single-ensure).
    async fn demand_topic(
        &self,
        robot_conn: &mut RobotConn,
        key: &TopicKey,
        schema_hash: u64,
    ) -> Result<(), String> {
        // 1. Demand (control round-trip — poisons the conn on a control error).
        let bytes = self
            .robot_round_trip(
                robot_conn,
                &WireRequest::Demand {
                    topic: key.topic.clone(),
                },
            )
            .await?;
        let resp: WireResponse =
            serde_json::from_slice(&bytes).map_err(|e| format!("decode demand reply: {e}"))?;
        match resp {
            WireResponse::DemandAccepted { topic } if topic == key.topic => {}
            WireResponse::Error { message, .. } => {
                return Err(format!(
                    "robot rejected the demand for '{}': {message}",
                    key.topic
                ))
            }
            other => {
                return Err(format!(
                    "expected DemandAccepted for '{}', got {other:?}",
                    key.topic
                ))
            }
        }

        // 2. Accept the robot's uni data stream (opened BEFORE the DemandAccepted, so
        //    it is already in flight) + read/verify its preamble. Both bounded.
        let mut ustream = self
            .bounded(
                "uni-stream accept",
                accept_uni_frame_stream(&robot_conn.connection),
            )
            .await?
            .map_err(|e| format!("accept uni stream for '{}': {e}", key.topic))?;
        let preamble_bytes = self
            .bounded(
                "uni-stream preamble",
                read_frame(&mut ustream, DEFAULT_MAX_FRAME_LEN),
            )
            .await?
            .map_err(|e| format!("read preamble for '{}': {e}", key.topic))?;
        let preamble: StreamPreamble = serde_json::from_slice(&preamble_bytes)
            .map_err(|e| format!("decode uni-stream preamble for '{}': {e}", key.topic))?;
        // Refuse a stream whose preamble names a DIFFERENT topic than we demanded.
        if preamble.topic != key.topic {
            return Err(format!(
                "robot opened a uni stream for '{}' but we demanded '{}' — refusing to mirror it",
                preamble.topic, key.topic
            ));
        }

        // 3. Create the desk-local ingress injector SYNCHRONOUSLY (LAN parity): a
        //    slot-taken failure (a local producer / a second mirror owns the
        //    single-writer slot) fails the demand HERE with NO mirror state left — so
        //    the daemon never commits refcount+presence on a phantom mirror.
        //    The injector's existence IS the mirror's readiness.
        let injector = match self.manager.create_ingress_injector(
            &key.topic,
            schema_hash,
            MIRROR_MAX_SLICE_LEN,
        ) {
            Ok(inj) => inj,
            Err(e) => {
                // The robot already opened a tap for this topic (we sent Demand + got
                // DemandAccepted + accepted its uni stream). Cancel it so the robot
                // tears the tap down — CRITICAL when this connection has OTHER live
                // readers and so is NOT dropped by `ensure_async`'s readerless guard,
                // else the robot keeps forwarding a stream nobody will read. Best-effort
                // — the injector error is what we propagate.
                if let Err(ue) = self.undemand_topic(robot_conn, &key.topic).await {
                    tracing::warn!(
                        robot = %key.robot, topic = %key.topic, error = %ue,
                        "cerulion-netd: could not undemand after a failed injector create — the \
                         robot may keep the tap until the connection closes"
                    );
                }
                return Err(format!(
                    "desk ingress publisher for '{}' could not be created: {e} (is the topic \
                     already produced locally, or already mirrored?)",
                    key.topic
                ));
            }
        };

        // 4. Register C0 provenance at the re-injection point (best-effort — a failure
        //    only means `topic list` would show LOCAL instead of REMOTE).
        if let Err(e) = self
            .manager
            .register_mirror_provenance(&key.topic, &key.robot)
        {
            tracing::warn!(
                robot = %key.robot, topic = %key.topic, error = %e,
                "cerulion-netd: could not register iroh mirror provenance — the mirror still \
                 streams, but `topic list` will show it as LOCAL rather than REMOTE"
            );
        }

        // 5. Spawn the supervising re-inject reader (owns the uni stream + injector).
        //    On an intentional release the task is aborted (the death handler is
        //    cancelled before it runs); on a natural exit it fires the death handler.
        let handle = self.rt.spawn(run_reinject_reader(
            ustream,
            injector,
            Arc::downgrade(&self.inner),
            self.manager.clone(),
            key.robot.clone(),
            key.topic.clone(),
        ));
        robot_conn.readers.insert(key.topic.clone(), handle);
        tracing::info!(
            robot = %key.robot,
            topic = %key.topic,
            "cerulion-netd: iroh WAN mirror registered (first demand) — re-injecting into desk SHM"
        );
        Ok(())
    }

    /// Best-effort `Undemand` on the robot's tap (bounded; poisons the conn on error).
    async fn undemand_topic(&self, robot_conn: &mut RobotConn, topic: &str) -> Result<(), String> {
        let bytes = self
            .robot_round_trip(
                robot_conn,
                &WireRequest::Undemand {
                    topic: topic.to_string(),
                },
            )
            .await?;
        let resp: WireResponse =
            serde_json::from_slice(&bytes).map_err(|e| format!("decode undemand reply: {e}"))?;
        match resp {
            WireResponse::Undemanded { .. } => Ok(()),
            WireResponse::Error { message, .. } => Err(message),
            other => Err(format!("expected Undemanded for '{topic}', got {other:?}")),
        }
    }

    /// The async body of `ensure_mirror`: lazily bind the endpoint, get-or-dial the
    /// robot connection (dropping a POISONED cached one first), then demand the topic.
    /// On failure, drop a now-readerless / poisoned connection so the next demand
    /// re-dials — no orphaned/poisoned connection wedges the robot.
    async fn ensure_async(&self, key: &TopicKey, schema_hash: u64) -> Result<(), String> {
        // The WAN demand-authorization pre-flight gate — consulted BEFORE
        // binding the endpoint / dialing, so an unauthorized WAN demand never opens a
        // connection. COMPOSES WITH (never replaces) the robot's own PairingAuthorizer
        // accept at the wire ALPN. The subject carries netd's OWN desk key (the
        // DEMANDER — netd is the party asking the robot for the topic), NOT the dial
        // target's pinned EndpointId, so a desk-side authorizer would resolve the RIGHT
        // (client) identity. This gate STAYS deny-nothing (`AllowAllAuthorizer`): the
        // robot is the authoritative per-demand enforcement point (the robot installs the
        // `is_allowed` predicate on the ROBOT serving plane). A robot ABSENT from the WAN
        // registry falls through to `dial_robot`'s "not configured" error unchanged (it
        // never dials anyway), so the gate is scoped to registered robots.
        if self.registry.is_wan_robot(&key.robot) {
            let subject = DemandSubject::Wan {
                demander_key: self.desk_key,
            };
            if let DemandDecision::Deny { reason } =
                self.authorizer.authorize_demand(&subject, &key.topic)
            {
                tracing::warn!(
                    robot = %key.robot,
                    topic = %key.topic,
                    plane = "iroh-wan",
                    reason = %reason,
                    "WAN demand REFUSED by the demand-authorization gate — this \
                     desk is not authorized for the robot (account/pairing grant); \
                     not dialing"
                );
                return Err(format!(
                    "WAN demand for '{}' on robot '{}' refused by the demand-authorization \
                     gate: {reason}",
                    key.topic, key.robot
                ));
            }
        }

        let mut st = self.inner.lock().await;

        // Drop a cached POISONED connection before reuse (a prior control error
        // desynced its framing — re-dial a fresh one).
        if st
            .robots
            .get(&key.robot)
            .is_some_and(|rc| rc.control_poisoned)
        {
            tracing::warn!(
                robot = %key.robot,
                "cerulion-netd: dropping a poisoned iroh connection before re-dial"
            );
            st.robots.remove(&key.robot);
        }

        // Endpoint is Clone (Arc-backed); clone it out so the borrow of `st.endpoint`
        // does not conflict with the later `st.robots` mutation.
        let endpoint = match &st.endpoint {
            Some(e) => e.clone(),
            None => {
                let e = self.build_desk_endpoint().await?;
                st.endpoint = Some(e.clone());
                e
            }
        };
        if !st.robots.contains_key(&key.robot) {
            let robot_conn = self.dial_robot(&endpoint, &key.robot).await?;
            st.robots.insert(key.robot.clone(), robot_conn);
        }

        let result = {
            let robot_conn = st
                .robots
                .get_mut(&key.robot)
                .expect("robot connection just inserted / already present");
            self.demand_topic(robot_conn, key, schema_hash).await
        };
        if result.is_err() {
            // A failed demand must leave NO orphaned connection: drop it when it is
            // poisoned (control desync) OR now-readerless (nothing to keep alive), so
            // the next demand re-dials cleanly rather than reusing a broken conn.
            if let Some(rc) = st.robots.get(&key.robot) {
                if rc.control_poisoned || rc.readers.is_empty() {
                    st.robots.remove(&key.robot); // drop closes the QUIC connection
                }
            }
        } else {
            // SURFACE the robot's revocation-epoch state on the DEMAND path
            // (Principle #3 — the recorded outcome exists so an operator learns a
            // revocation did not land). Not redundant with the dial's own logging: a
            // demand that reuses an EXISTING connection performs no push, so this is
            // the only place the current carriage state is reported at demand time.
            // (Lock order is always inner → epoch_pushes, the same order `dial_robot`
            // takes, so this can never deadlock.)
            //
            // THREE levels, not two: netd is a long-lived daemon and this
            // line fires on EVERY demand, so a desk that holds NOTHING for the robot
            // (never synced, or a guest desk — the account service's access endpoint is
            // owner-only) must not warn. It has no revocation it failed to carry, and
            // warning per demand forever is a false alarm that trains an operator to
            // ignore the real one. `warn` is reserved for a revocation this desk was (or
            // may have been) holding that did NOT reach the robot.
            if let Some(outcome) = self.epoch_push_outcome_async(&key.robot).await {
                match outcome.severity() {
                    EpochPushSeverity::Current => tracing::info!(
                        robot = %key.robot, topic = %key.topic, outcome = %outcome,
                        "cerulion-netd: revocation epoch"
                    ),
                    EpochPushSeverity::NothingToCarry => tracing::debug!(
                        robot = %key.robot, topic = %key.topic, outcome = %outcome,
                        "cerulion-netd: revocation epoch — nothing to carry"
                    ),
                    EpochPushSeverity::NotCarried => tracing::warn!(
                        robot = %key.robot, topic = %key.topic, outcome = %outcome,
                        "cerulion-netd: mirroring a robot whose revocation epoch is NOT \
                         confirmed current"
                    ),
                }
            }
        }
        result
    }

    /// The async read behind [`Self::last_push_outcome`] — usable from INSIDE the
    /// plane's runtime (the public accessor `block_on`s, which would panic here).
    async fn epoch_push_outcome_async(&self, robot: &str) -> Option<EpochPushOutcome> {
        self.epoch_pushes.lock().await.get(robot).copied()
    }

    /// The async body of `release_mirror`: abort the topic's reader (releasing its SHM
    /// slot), `Undemand` it on the robot, and drop the whole connection when the robot
    /// has no more demanded topics. Best-effort C0 provenance cleanup. Returns
    /// [`MirrorRelease::Retired`] on a genuine teardown, [`MirrorRelease::Lingering`]
    /// only when the connection is unexpectedly missing (nothing to retire cleanly).
    async fn release_async(&self, key: &TopicKey) -> MirrorRelease {
        let mut st = self.inner.lock().await;
        if !st.robots.contains_key(&key.robot) {
            drop(st);
            tracing::error!(
                robot = %key.robot,
                topic = %key.topic,
                "cerulion-netd: iroh release for a robot with no live WAN connection — the mirror \
                 bridge state is inconsistent; LINGERING (reclaimed at idle self-exit)"
            );
            return MirrorRelease::Lingering;
        }

        // 1. Remove the reader from the map FIRST (so the reader task's death handler,
        //    if it races a natural exit, no-ops), then abort + await it so its injector
        //    Drop RUNS before we return (the SHM mirror slot is genuinely freed).
        let handle = st
            .robots
            .get_mut(&key.robot)
            .and_then(|rc| rc.readers.remove(&key.topic));
        if let Some(h) = handle {
            h.abort();
            let _ = h.await; // Err(Cancelled) once the task's injector has dropped.
        }

        // 2. Best-effort `Undemand` (the robot drops its tap) + close the connection
        //    when this robot has no more demanded topics.
        let robot_conn = st
            .robots
            .get_mut(&key.robot)
            .expect("connection present (checked above)");
        if let Err(e) = self.undemand_topic(robot_conn, &key.topic).await {
            tracing::warn!(
                robot = %key.robot, topic = %key.topic, error = %e,
                "cerulion-netd: iroh undemand failed (the robot may drop its tap on connection \
                 close) — continuing teardown"
            );
        }
        if robot_conn.readers.is_empty() {
            let _ = robot_conn.control_send.finish();
            st.robots.remove(&key.robot);
            tracing::info!(
                robot = %key.robot,
                "cerulion-netd: iroh WAN connection closed (no more demanded topics)"
            );
        }
        drop(st);

        // 3. Best-effort C0 provenance cleanup so `topic list` stops folding the
        //    torn-down mirror into REMOTE.
        if let Err(e) = self.manager.unregister_mirror_provenance(&key.topic) {
            tracing::warn!(
                robot = %key.robot, topic = %key.topic, error = %e,
                "cerulion-netd: iroh mirror torn down but could not remove its provenance"
            );
        }
        tracing::info!(
            robot = %key.robot,
            topic = %key.topic,
            "cerulion-netd: iroh WAN mirror torn down (last demand released)"
        );
        MirrorRelease::Retired
    }
}

/// The supervising per-topic reader: drive the SHARED validated re-inject primitive
/// ([`IngressInjector::reinject_raw`]) until the uni stream ends, then — ONLY on a
/// natural exit (a release-abort cancels this future before this point) — fire the
/// reader-death teardown.
async fn run_reinject_reader(
    mut recv: RecvStream,
    injector: IngressInjector,
    inner: Weak<AsyncMutex<IrohState>>,
    manager: Arc<TransportManager>,
    robot: String,
    topic: String,
) {
    loop {
        let frame = match read_frame(&mut recv, DEFAULT_MAX_FRAME_LEN).await {
            Ok(f) => f,
            Err(e) => {
                tracing::debug!(robot = %robot, topic = %topic, error = %e, "cerulion-netd: iroh uni stream ended");
                break;
            }
        };
        match injector.reinject_raw(&frame) {
            ReinjectOutcome::Injected { .. } => {}
            ReinjectOutcome::Rejected(_) => {
                tracing::debug!(robot = %robot, topic = %topic, "cerulion-netd: iroh frame REJECTED by wire-frame validation");
            }
            ReinjectOutcome::ReinjectFailed => {
                tracing::debug!(robot = %robot, topic = %topic, "cerulion-netd: iroh frame validated but the local publish failed");
            }
        }
    }
    // Reaching here means the loop ended NATURALLY (the uni stream died) — a
    // release-abort would have cancelled this future inside the loop above, before
    // this point. So this is an UNEXPECTED reader death. Free the SHM slot first, then
    // tear down the plane's state for this topic.
    drop(injector);
    on_reader_death(inner, manager, robot, topic).await;
}

/// Tear down the plane's state after an UNEXPECTED reader death (robot disconnect /
/// tap dropped): LOUDLY error, remove the reader, drop the connection if it was the
/// last, and remove the mirror's C0 provenance — so a later demand re-creates cleanly.
/// A no-op if the reader was already removed (the release-race guard) or the plane has
/// been dropped.
async fn on_reader_death(
    inner: Weak<AsyncMutex<IrohState>>,
    manager: Arc<TransportManager>,
    robot: String,
    topic: String,
) {
    let Some(inner) = inner.upgrade() else {
        return; // the plane was dropped — nothing to tear down.
    };
    let mut st = inner.lock().await;
    let Some(rc) = st.robots.get_mut(&robot) else {
        return;
    };
    // If release already removed this reader (the intentional-teardown race), no-op.
    if rc.readers.remove(&topic).is_none() {
        return;
    }
    tracing::error!(
        robot = %robot,
        topic = %topic,
        "cerulion-netd: iroh WAN reader exited UNEXPECTEDLY (robot disconnect / tap dropped) — the \
         mirror stopped streaming; tearing down its plane state so a re-demand re-creates it \
         (the daemon registry is not healed automatically)"
    );
    if rc.readers.is_empty() {
        st.robots.remove(&robot); // drop closes the QUIC connection.
    }
    drop(st);
    if let Err(e) = manager.unregister_mirror_provenance(&topic) {
        tracing::warn!(
            robot = %robot, topic = %topic, error = %e,
            "cerulion-netd: could not remove provenance after an iroh reader death"
        );
    }
}

impl MirrorPlane for IrohMirrorPlane {
    fn ensure_mirror(&self, key: &TopicKey, schema_hash: u64) -> Result<(), MirrorError> {
        self.rt
            .block_on(self.ensure_async(key, schema_hash))
            .map_err(|reason| MirrorError::Iroh {
                key: key.clone(),
                reason,
            })
    }

    fn release_mirror(&self, key: &TopicKey) -> MirrorRelease {
        self.rt.block_on(self.release_async(key))
    }
}
