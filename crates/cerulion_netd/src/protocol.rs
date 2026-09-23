// SPDX-License-Identifier: AGPL-3.0-only
//! The sacred `cerulion-netd` control protocol — a
//! Unix-domain-socket NDJSON (one JSON object per line) request/response contract
//! (the `cerulion-vizd` shape, reused).
//!
//! A consumer opens the UDS, reads the [`Hello`] banner (a version handshake), and
//! then exchanges one-line requests/responses. The three verbs are:
//!
//! - [`Request::Demand`] — "I want remote `(robot, topic)`; ensure the shared
//!   mirror exists." netd refcounts the demand and, on the FIRST demand for a
//!   `(robot, topic)`, registers ONE ingress bridge (the single shared mirror).
//! - [`Request::Release`] — "I no longer need it." netd decrements the refcount;
//!   the per-topic TEARDOWN at refcount 0 is C2 (this chunk leaves the seam).
//! - [`Request::Status`] — observability (Principle #3): the live demand table.
//!
//! **The connection close IS an implicit release** (the crash-safe refcount): a
//! consumer that drops its socket releases every demand it held, so netd never
//! leaks a demand behind a dead consumer. That accounting lives in [`crate::registry`];
//! this module is only the wire types + codec.
//!
//! This module is transport-free (serde + `serde_json` only) so a future client
//! (C3's migrated vizd / `topic echo` / connectd) can lift it verbatim.

use cerulion_core::transport::cerulion_q::UnusableRunsAnswer;
use cerulion_core::{CatalogReply, GatewayPlan, RunsReply, SchemaReply, SchemaServing};
use serde::{Deserialize, Serialize};

/// The `cerulion-netd` control-protocol version. Bumped on any
/// backwards-incompatible change to the request/response shapes so a consumer
/// detects skew from the [`Hello`] banner.
///
/// `2`: added the `register_egress` / `release_egress` verbs (the
/// desk egress-convergence seam — a graph pushes its produced-topic egress plan
/// into netd so ONE session serves the machine's whole network plane). A v1
/// consumer never sends the new verbs and a v2 daemon serves v1's `demand` /
/// `release` / `status` unchanged, but the banner still bumps so a consumer that
/// DOES depend on the egress verbs detects an old daemon.
///
/// `3`: added the `query_catalog` / `query_schema` verbs (the desk
/// CATALOG/SCHEMA query plane — a consumer resolves a remote robot's topic catalog
/// or a type's `.msg`/YAML closure over netd's ONE zenoh session instead of opening
/// its own transient discovery session). Both are STATELESS one-shots (never
/// refcounted, they create no mirror). A v<3 consumer never sends them and a v3
/// daemon serves the v1/v2 vocabulary unchanged; the banner bumps so a consumer that
/// depends on the query verbs detects a too-old daemon (per-verb gated at the client
/// send site, so a v3 client still uses demand/release against a pinned v2 daemon).
///
/// `4`: `register_egress` gained the optional `ix_config_json` field —
/// the producing run's resolved iceoryx2 namespace, so netd's embedded gateway can
/// VERIFY it matches netd's shared session before tapping the run's SHM (a
/// multi-process deployment's supervisor mints the namespace; the monolith omits it).
/// A CONFIG-CARRYING `register_egress` needs a v4 daemon
/// ([`Request::min_daemon_version`] returns 4 for it) so a v4 client never sends a
/// config a v<4 daemon would silently ignore (→ tap on the wrong namespace); a
/// config-LESS `register_egress` stays a v2 verb and works against any v2+ daemon
/// unchanged. The banner bumps so the per-verb gate detects a too-old daemon.
///
/// `5`: the query responses gained the [`DiscoveryState`] marker. The FIELD
/// is `#[serde(default)]` so the wire is compatible both ways — but the DEFAULT is
/// [`DiscoveryState::Settled`], which is a POSITIVE assertion ("discovery ran, so an
/// empty answer is real absence") that an older daemon cannot actually make. netd is
/// a long-lived spawn-once daemon, so upgrading the CLI while an old netd is still
/// held alive by a running vizd would silently restore the old false "not found"
/// with no signal anywhere. The banner bump is what makes that detectable: the client
/// treats a `< 5` daemon's report as UNTRUSTWORTHY rather than as `Settled` (see
/// [`crate::client::DISCOVERY_MIN_DAEMON_VERSION`]). The query verbs still report
/// `min_daemon_version() == 3`, so they keep WORKING against a v3/v4 daemon — this is
/// a trust downgrade, not a refusal.
///
/// `6`: added the `subscribe_catalog` verb AND — the first of its kind on
/// this seam — a SERVER-INITIATED message, [`CatalogChanged`], which netd writes
/// UNPROMPTED when the LAN catalog changes so a desk sidebar updates itself instead of
/// waiting for somebody to click refresh.
///
/// An unsolicited line is exactly the thing that can DESYNC a request/response reader
/// (the next `read_line` would return the push where the response was expected), so
/// the push is strictly OPT-IN: netd sends one ONLY down a connection that sent
/// `subscribe_catalog`. That makes the skew story symmetric and wedge-free in BOTH
/// directions, with no trust downgrade needed:
///
/// - **NEW client, OLD (`< 6`) daemon** — the verb's `min_daemon_version()` is 6, so
///   the client's per-verb compatibility gate refuses it BEFORE it is sent and the
///   consumer degrades LOUDLY to its own refresh path. Nothing is wedged and no
///   response is ever mis-read.
/// - **OLD client, NEW daemon** — an old client never sends `subscribe_catalog`, so it
///   is never subscribed and receives NO LINE a v5 daemon would not have sent it. (The
///   `Hello` banner's own `protocol` field does of course read 6 rather than 5 — that
///   is the version handshake doing its job; what is unchanged is every line AFTER it,
///   which is the part a request/response reader correlates.) This is the property
///   that makes the feature safe to ship into a spawn-once daemon that long-lived
///   consumers hold alive: existing consumers cannot be desynced by it.
///
/// `7`: added the `query_runs` verb — the desk's view of which runs are
/// LIVE on a robot right now and what each one's graph is, gathered over netd's ONE
/// zenoh session from each robot's `cerulion_q/{robot}/runs` surface. STATELESS
/// one-shot like its `query_catalog` / `query_schema` siblings (no mirror, no
/// refcount).
///
/// The bump GATES ONLY THE NEW VERB, and that per-verb gate IS the capability
/// negotiation (the `subscribe_catalog` shape, one bump earlier):
///
/// - **NEW client, OLD (`< 7`) daemon** — [`RUNS_MIN_DAEMON_VERSION`] is 7, so
///   the client's per-verb gate refuses `query_runs` BEFORE it is sent and the consumer
///   degrades LOUDLY. Without the bump the request would reach the daemon and come back
///   as the generic unknown-method error, which is indistinguishable from a daemon that
///   knows the verb and failed.
/// - **OLD client, NEW daemon** — an old client never sends `query_runs`, and every
///   pre-existing verb keeps its own `min_daemon_version()` and its own response shape,
///   so its byte stream after the banner is unchanged.
///
/// **No trust downgrade is needed here, by DESIGN rather than by luck.** The v5
/// bump needed one because it added a field whose serde default was a POSITIVE claim
/// an older daemon could not make. This verb's response type is NEW — only a v7 daemon
/// can send one at all — and its own `discovery` field is deliberately MANDATORY on the
/// wire (see [`RunsQueryResponse::discovery`]), so there is no absent-field default to
/// distrust; its one optional field, `plane_unsettled_ms`, means UNKNOWN when absent
/// and suppresses nothing.
///
/// `8`: account snapshots, presence probes and authorized metadata queries use the
/// existing socket. A local serving-schema snapshot also lets remoted use the
/// gateway's accumulated bindings without network discovery. Older daemons must
/// be restarted before these operations.
pub const PROTOCOL_VERSION: u32 = 8;

const _: () = assert!(crate::account_access::MIN_DAEMON_VERSION <= PROTOCOL_VERSION);

/// First protocol version with the local-only serving-schema snapshot.
pub const SERVING_SCHEMA_MIN_DAEMON_VERSION: u32 = 8;
const _: () = assert!(SERVING_SCHEMA_MIN_DAEMON_VERSION <= PROTOCOL_VERSION);

/// Metadata accumulated by this daemon's egress plane, in its exact SHM namespace.
/// Reading this value never performs network discovery or creates a demand.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ServingSchemaSnapshot {
    /// Topic bindings, hash bindings and custom schema documents registered locally.
    pub schema_serving: SchemaServing,
    /// Serialized iceoryx2 configuration for namespace validation by the reader.
    pub ix_config_json: String,
}

/// The required `serving_schema` field separates this reply from all other verbs.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ServingSchemaSnapshotResponse {
    /// Echoed request identifier.
    pub id: u64,
    /// Current local serving metadata, including its namespace.
    pub serving_schema: ServingSchemaSnapshot,
}

/// The minimum daemon [`PROTOCOL_VERSION`] that serves the `query_runs`
/// verb — the version that INTRODUCED it, frozen here rather than spelled inline so
/// the trust-gate reasoning below can be machine-checked.
///
/// Named because two separate const assertions turn on it: it must not exceed what
/// this binary speaks (or the verb would be permanently unusable), and it must be at
/// or above [`crate::client::DISCOVERY_MIN_DAEMON_VERSION`] — which is what makes a
/// `query_runs` answer's [`DiscoveryState`] trustworthy WITHOUT a downgrade, since any
/// daemon old enough to need one is already too old to serve the verb.
pub const RUNS_MIN_DAEMON_VERSION: u32 = 7;

const _: () = assert!(RUNS_MIN_DAEMON_VERSION <= PROTOCOL_VERSION);

/// The fixed `hello` marker string in the [`Hello`] banner — lets a consumer
/// confirm it reached `cerulion-netd` (and not some other UDS server) before
/// trusting the protocol version.
pub const HELLO_MARKER: &str = "cerulion-netd";

/// A cap on a single request line's length (bytes) before the newline. A consumer
/// building an unbounded line (buggy serializer / partial flush / malicious client)
/// is dropped WITH A LOUD structured error (never a silent drop — see the daemon's
/// oversized-line path) rather than allowed to OOM the daemon.
///
/// Sized for the LARGEST realistic FRAMED request — a
/// [`Request::RegisterEgress`] carries a `GatewayPlan` + a `SchemaServing` (the
/// workspace's produced-topic egress plan + its CUSTOM-type schema closure), because
/// netd cannot reach the workspace `.msg` store itself. A big `ros2 attach` bridge
/// (a Go2-class robot: ~90 topics, dozens of custom types whose `.msg` texts run to
/// a few KB each) JSON-encodes to roughly 60–120 KiB — already over the old 64 KiB
/// cap, which would SILENTLY hard-fail a legitimate plan. Even a pathological
/// workspace (hundreds of custom types × a few KB) stays well under ~2 MiB. The cap
/// is set to **8 MiB** — several × headroom over that worst case, and a bounded
/// per-connection buffer a handful of TRUSTED LOCAL UDS consumers can never OOM the
/// daemon with (a malicious 8 MiB line is dropped with the loud error at the cap).
/// A demand/release request stays well under 1 KiB.
pub const MAX_REQUEST_LINE_BYTES: usize = 8 * 1024 * 1024;

/// The one-line banner netd writes IMMEDIATELY on accept, before any request. A
/// consumer reads it to confirm the marker + detect a [`PROTOCOL_VERSION`] skew.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Hello {
    /// Always [`HELLO_MARKER`] (`"cerulion-netd"`).
    pub hello: String,
    /// The [`PROTOCOL_VERSION`] this daemon speaks.
    pub protocol: u32,
}

impl Hello {
    /// The banner for THIS daemon's protocol version.
    pub fn new() -> Self {
        Self {
            hello: HELLO_MARKER.to_string(),
            protocol: PROTOCOL_VERSION,
        }
    }

    /// Serialize to the single NDJSON banner line (no trailing newline — the
    /// writer adds it).
    pub fn to_json_line(&self) -> String {
        serde_json::to_string(self).expect("Hello serializes")
    }
}

impl Default for Hello {
    fn default() -> Self {
        Self::new()
    }
}

/// One control request. Internally tagged by `method` (snake_case) so a line is a
/// flat JSON object, e.g.
/// `{"method":"demand","id":1,"robot":"ubuntu","topic":"/utlidar/robot_odom","schema_hash":123}`.
///
/// Not `Eq`: nothing keys a `Request`, and keeping it `PartialEq` is all the
/// round-trip assertions need.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "method", rename_all = "snake_case")]
pub enum Request {
    /// Read local egress metadata without invoking either network query plane.
    ServingSchemaSnapshot {
        /// Correlation id echoed in the response.
        id: u64,
    },
    /// Account robot access through the daemon's single WAN controller.
    AccountAccess {
        /// Correlation id echoed in the response.
        id: u64,
        /// Bounded public identity operation; no token or private key.
        action: crate::account_access::AccountAccessRequest,
    },
    /// Demand a remote `(robot, topic)`: ensure the shared desk mirror exists and
    /// increment this connection's refcount for it. Idempotent per connection (a
    /// re-demand of a key the connection already holds does not double-count).
    Demand {
        /// Correlation id echoed in the response.
        id: u64,
        /// The remote robot identity (the announce/mirror provenance).
        robot: String,
        /// The absolute canonical topic to mirror (e.g. `/utlidar/robot_odom`).
        topic: String,
        /// The topic's wire `schema_hash` (recipe-3) — netd validates every
        /// inbound frame against it before re-injecting. The consumer resolves the
        /// `pkg/Type` name to this hash before demanding (name→hash resolution is a
        /// consumer concern; C3's migrated consumers reuse the catalog).
        ///
        /// A `u64` (a schema hash can exceed 2^53) — Rust consumers round-trip it
        /// exactly through `serde_json`.
        schema_hash: u64,
    },
    /// Release a previously-demanded `(robot, topic)` for this connection. A no-op
    /// if the connection did not hold it. Dropping the connection releases every
    /// demand it held — this verb is the explicit early release.
    Release {
        /// Correlation id echoed in the response.
        id: u64,
        /// The remote robot identity.
        robot: String,
        /// The absolute canonical topic to release.
        topic: String,
    },
    /// Report the live demand table (Principle #3): every demanded `(robot,
    /// topic)` with its refcount, the active connection count, and whether netd is
    /// currently idle (idle-grace running toward self-exit).
    Status {
        /// Correlation id echoed in the response.
        id: u64,
    },
    /// Register THIS connection's egress plan — the produced topics a
    /// desk graph wants netd to announce + egress-on-demand over the machine's ONE
    /// zenoh session. netd boots the shared embedded gateway on the FIRST egress
    /// registration and pushes each of the plan's announce topics onto it (via
    /// `register_dynamic_egress_topic`). The registration is SCOPED to this
    /// connection: dropping the connection (or [`Self::ReleaseEgress`]) releases it.
    /// Additive per connection — a re-register ADDs the plan's topics to the
    /// connection's set (the reg-channel is add-only).
    ///
    /// Refused LOUDLY if any announce topic is currently MIRRORED IN by an ingress
    /// demand (the cross-plan loop guard: netd must never announce OUT a topic it
    /// re-injects IN — the echo loop).
    RegisterEgress {
        /// Correlation id echoed in the response.
        id: u64,
        /// The produced-topic egress plan (announce set + posture). Only the egress
        /// half is consumed by netd's egress plane; a graph's ingress converges via
        /// the demand plane. Serde-clean (`cerulion_core::GatewayPlan`).
        plan: GatewayPlan,
        /// The catalog/schema payload the desk resolved for this egress graph — netd
        /// cannot reach the workspace `.msg` store itself, so the producing side
        /// hands it across. Applied at the shared gateway's boot (an empty serving
        /// is a bare boot). Serde-clean (`cerulion_core::SchemaServing`).
        #[serde(default)]
        schema_serving: SchemaServing,
        /// The producing run's RESOLVED iceoryx2 namespace, serialized
        /// (`serde_json` of an `iceoryx2::config::Config`). netd's embedded gateway
        /// taps SHM on netd's OWN namespace, so it compares this config's discovery
        /// identity (`root_path` + `prefix`) against netd's shared session before
        /// tapping — a run that resolved a DIFFERENT namespace is refused so it falls
        /// back to a per-run gateway child rather than being silently tapped on the
        /// wrong namespace. `None` for a MONOLITH run (it shares netd's default
        /// namespace by construction — its own `init` resolves the same
        /// `global_config()`); `Some` for a MULTI-PROCESS run (the supervisor mints
        /// the shared worker namespace and hands it across). A config-carrying
        /// registration requires a v4 daemon ([`Request::min_daemon_version`]).
        #[serde(default)]
        ix_config_json: Option<String>,
    },
    /// Release THIS connection's egress registration (the inverse of
    /// [`Self::RegisterEgress`]). A no-op if the connection holds none. Dropping the
    /// connection releases it implicitly — this verb is the explicit early release.
    ReleaseEgress {
        /// Correlation id echoed in the response.
        id: u64,
    },
    /// Query the LAN topic CATALOG over netd's ONE zenoh session — the
    /// desk consumer's replacement for opening its own transient discovery session.
    /// netd harvests the announce space + GETs each robot's `cerulion_q/{robot}/catalog`
    /// (or, with `robot: Some`, just that robot's) and returns every decoded
    /// [`CatalogReply`]. STATELESS one-shot: it creates NO mirror and is NEVER
    /// refcounted; the query connection counts as live only while it is open.
    QueryCatalog {
        /// Correlation id echoed in the response.
        id: u64,
        /// `Some(robot)` scopes the query to ONE robot's catalog (vizd's
        /// single-robot resolve); `None` harvests every announcing robot and GETs
        /// all their catalogs (the `topic echo`/`schema info` LAN gather).
        #[serde(default)]
        robot: Option<String>,
    },
    /// Fetch a remote type's `.msg`/YAML closure over netd's ONE zenoh
    /// session — the desk consumer's replacement for a transient `schema` GET. netd
    /// GETs `cerulion_q/{robot}/schema/{requested}` from ONE robot (`robot: Some`) or
    /// every announcing robot (`robot: None`) and returns every decoded
    /// [`SchemaReply`] (found OR structured not-found/refused — the desk picks the
    /// first with non-empty `docs`). STATELESS one-shot (no mirror, no refcount).
    QuerySchema {
        /// Correlation id echoed in the response.
        id: u64,
        /// `Some(robot)` scopes the fetch to ONE robot; `None` asks every
        /// announcing robot and returns every decodable reply.
        #[serde(default)]
        robot: Option<String>,
        /// The requested qualified `pkg/Type` OR a package-less bare `Name` (fed
        /// verbatim to the `schema` selector — a bare name has no package half).
        requested: String,
    },
    /// Subscribe THIS connection to catalog-change PUSHES — netd writes a
    /// [`CatalogChanged`] line down this connection, unprompted, whenever the LAN's
    /// announce space changes (a robot appearing, a robot dying, an `ros2 attach`
    /// graph's topics coming up). The desk sidebar updates itself; nothing polls.
    ///
    /// Answered with a [`SubscribeCatalogResponse`] carrying the CURRENT snapshot
    /// (version + announcing robots), so a consumer that connects mid-stream starts
    /// from a known state rather than from whatever it last remembered.
    ///
    /// The subscription is SCOPED to this connection: dropping the connection
    /// unsubscribes it (the same crash-safe rule the demand plane uses — there is
    /// deliberately no `unsubscribe` verb, because closing the connection IS one and a
    /// second way to say it would be a second thing to keep correct). Re-subscribing
    /// on a connection that is already subscribed is idempotent and re-reports the
    /// snapshot.
    ///
    /// A connection that sends this MUST be prepared to read an unsolicited line at
    /// any point (see [`CatalogChanged`]); a connection that does NOT send it never
    /// receives one.
    SubscribeCatalog {
        /// Correlation id echoed in the response.
        id: u64,
    },
    /// Ask which runs are LIVE on a robot right now, each carrying its
    /// effective `graph.yaml` + `run.json` VERBATIM. netd GETs
    /// `cerulion_q/{robot}/runs` from ONE robot (`robot: Some`) or every announcing
    /// robot (`robot: None`) over its ONE zenoh session and returns every decoded
    /// [`RunsReply`]. STATELESS one-shot: it creates NO mirror and is NEVER refcounted.
    ///
    /// **Never fold this into a polling loop.** A serve builds a fresh iceoryx2 reader
    /// node per call (the ~620 ms shape measured for `gather_mirror_provenance`)
    /// plus up to one gather window when a live registry writer must be heard — so the
    /// robot-side cost is orders of magnitude above a `catalog` serve. It is affordable
    /// only because a run's DAG is immutable for the life of its `run_id`, so the desk
    /// fetches once per run and refreshes liveness off the catalog poll it already
    /// makes.
    QueryRuns {
        /// Correlation id echoed in the response.
        id: u64,
        /// `Some(robot)` scopes the query to ONE robot (the desk's fetch for a run it
        /// is about to render); `None` harvests every announcing robot and asks each.
        #[serde(default)]
        robot: Option<String>,
    },
}

impl Request {
    /// The correlation `id` this request carries (echoed in its response).
    pub fn id(&self) -> u64 {
        match self {
            Request::ServingSchemaSnapshot { id }
            | Request::AccountAccess { id, .. }
            | Request::Demand { id, .. }
            | Request::Release { id, .. }
            | Request::Status { id }
            | Request::RegisterEgress { id, .. }
            | Request::ReleaseEgress { id }
            | Request::QueryCatalog { id, .. }
            | Request::QuerySchema { id, .. }
            | Request::QueryRuns { id, .. }
            | Request::SubscribeCatalog { id } => *id,
        }
    }

    /// The `method` tag (snake_case) this request serializes with — for diagnostics /
    /// the client's per-verb version-compat message.
    pub fn method_name(&self) -> &'static str {
        match self {
            Request::ServingSchemaSnapshot { .. } => "serving_schema_snapshot",
            Request::AccountAccess { .. } => "account_access",
            Request::Demand { .. } => "demand",
            Request::Release { .. } => "release",
            Request::Status { .. } => "status",
            Request::RegisterEgress { .. } => "register_egress",
            Request::ReleaseEgress { .. } => "release_egress",
            Request::QueryCatalog { .. } => "query_catalog",
            Request::QuerySchema { .. } => "query_schema",
            Request::QueryRuns { .. } => "query_runs",
            Request::SubscribeCatalog { .. } => "subscribe_catalog",
        }
    }

    /// The MINIMUM daemon [`PROTOCOL_VERSION`] a daemon must speak to
    /// serve this verb. `demand` / `release` / `status` are the ORIGINAL v1
    /// vocabulary; `register_egress` / `release_egress` are the v2 egress-convergence
    /// verbs. A daemon at or above a verb's minimum serves it (a newer daemon's
    /// vocabulary is a strict SUPERSET), which is exactly the client's forward/back
    /// compatibility contract: a v2 client keeps working against a pinned v1 daemon
    /// for the v1 verbs, and a v1 client works against a v2 daemon unchanged. The
    /// client refuses a specific verb ONLY when the connected daemon is older than
    /// THAT verb requires (see [`crate::client`]).
    pub fn min_daemon_version(&self) -> u32 {
        match self {
            Request::Demand { robot, .. } | Request::Release { robot, .. }
                if robot
                    .trim()
                    .starts_with(crate::account_access::ACCOUNT_ROUTE_PREFIX) =>
            {
                crate::account_access::MIN_DAEMON_VERSION
            }
            Request::QueryCatalog {
                robot: Some(robot), ..
            }
            | Request::QuerySchema {
                robot: Some(robot), ..
            }
            | Request::QueryRuns {
                robot: Some(robot), ..
            } if robot
                .trim()
                .starts_with(crate::account_access::ACCOUNT_ROUTE_PREFIX) =>
            {
                crate::account_access::MIN_DAEMON_VERSION
            }
            // The catalog-change SUBSCRIPTION needs a v6 daemon. A v<6 daemon
            // has no announce watch and would answer an unknown-method error; gating it
            // here means the consumer gets a precise version message and degrades to
            // its own refresh path, rather than silently waiting forever for a push
            // that can never come.
            Request::ServingSchemaSnapshot { .. } => SERVING_SCHEMA_MIN_DAEMON_VERSION,
            Request::AccountAccess { .. } => crate::account_access::MIN_DAEMON_VERSION,
            Request::SubscribeCatalog { .. } => 6,
            // The `runs` verb needs a v7 daemon. A v<7 daemon has no
            // `runs` handler and would answer the generic unknown-method error, which
            // a consumer cannot tell from a daemon that knows the verb and failed —
            // so the gate here IS the capability negotiation, and it deliberately
            // leaves every OTHER verb's minimum untouched.
            Request::QueryRuns { .. } => RUNS_MIN_DAEMON_VERSION,
            // The catalog/schema query verbs need a v3 daemon.
            Request::QueryCatalog { .. } | Request::QuerySchema { .. } => 3,
            // A CONFIG-CARRYING `register_egress` needs a v4 daemon (only
            // v4+ VERIFIES the forwarded namespace); a v<4 daemon would silently ignore
            // the field and tap on its own namespace. A config-LESS register stays the
            // v2 egress verb (the monolith path — the run shares netd's namespace, so no
            // config to verify) and works against any v2+ daemon unchanged.
            Request::RegisterEgress {
                ix_config_json: Some(_),
                ..
            } => 4,
            // The egress verbs need a v2 daemon.
            Request::RegisterEgress { .. } | Request::ReleaseEgress { .. } => 2,
            // The original v1 vocabulary.
            Request::Demand { .. } | Request::Release { .. } | Request::Status { .. } => 1,
        }
    }

    /// Serialize to the single NDJSON request line (client/test helper).
    pub fn to_json_line(&self) -> String {
        serde_json::to_string(self).expect("Request serializes")
    }
}

/// A failed [`parse_request`]: the line was not a valid request. Carries the
/// BEST-EFFORT correlation `id` (extracted from the raw JSON's `id` field even
/// when `method` was unknown / missing) so the daemon's error response still
/// correlates where possible, plus the parse `message`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RequestError {
    /// Best-effort correlation id (`None` when the line carried no numeric `id`).
    pub id: Option<u64>,
    /// The parse-failure detail (a serde message).
    pub message: String,
}

/// Parse ONE NDJSON request line. On success returns the typed [`Request`]; on
/// ANY failure (not JSON, missing/unknown `method`, missing `id`, wrong types)
/// returns a [`RequestError`] carrying a best-effort `id` for correlation — the
/// caller answers with a structured [`Response::error`], NEVER a panic or a
/// dropped connection.
pub fn parse_request(line: &str) -> Result<Request, RequestError> {
    match serde_json::from_str::<Request>(line) {
        Ok(req) => Ok(req),
        Err(e) => {
            // Best-effort id extraction: re-parse loosely so an unknown-method /
            // malformed-body line still correlates its error to the request id.
            let id = serde_json::from_str::<serde_json::Value>(line)
                .ok()
                .and_then(|v| v.get("id").and_then(serde_json::Value::as_u64));
            Err(RequestError {
                id,
                message: e.to_string(),
            })
        }
    }
}

/// Which transport carried a mirror's frames.
///
/// The two planes are not interchangeable and an operator cannot tell them apart
/// from the frames: both re-inject into the same local shared memory under the
/// same topic name. A robot on the same local network is reachable over BOTH, so
/// "the robot is on the internet plane" is a claim about routing that only the
/// daemon can make.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum ServingPlane {
    /// The local-network plane: the shared gateway session, mirrored by name.
    Zenoh,
    /// The internet plane: dial the robot's endpoint id directly, authenticated by
    /// its device key, and re-inject what that one connection carries.
    Iroh,
}

impl ServingPlane {
    /// The lowercase token this plane serializes as, for a message or a log field.
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Zenoh => "zenoh",
            Self::Iroh => "iroh",
        }
    }
}

/// One demand-table row in a [`StatusResponse`].
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct DemandEntry {
    /// The remote robot identity.
    pub robot: String,
    /// The canonical topic.
    pub topic: String,
    /// How many connections currently demand this `(robot, topic)` (the shared
    /// mirror's refcount).
    pub refcount: usize,
    /// Which plane serves this row, when the daemon can say.
    ///
    /// `Option`, not a bare [`ServingPlane`], for the reason `connect_endpoints`
    /// is optional: a daemon that predates this field omits it, and any default
    /// value would be a POSITIVE claim about routing that such a daemon never
    /// made. `None` means "this daemon does not report the plane"; a reader must
    /// never read it as "the local-network plane". A mirror plane that cannot
    /// attribute a key (a test double, a plane whose route pin is gone) also
    /// answers `None` rather than guessing.
    #[serde(default)]
    pub plane: Option<ServingPlane>,
}

/// The response to [`Request::Demand`].
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct DemandResponse {
    /// Echoed correlation id.
    pub id: u64,
    /// The demanded robot.
    pub robot: String,
    /// The demanded topic.
    pub topic: String,
    /// The refcount for this `(robot, topic)` AFTER this demand.
    pub refcount: usize,
    /// Whether THIS demand created the shared mirror (the first demander); `false`
    /// when it joined an existing mirror.
    pub mirror_created: bool,
}

/// The response to [`Request::Release`].
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ReleaseResponse {
    /// Echoed correlation id.
    pub id: u64,
    /// The released robot.
    pub robot: String,
    /// The released topic.
    pub topic: String,
    /// The refcount for this `(robot, topic)` AFTER this release.
    pub refcount: usize,
    /// Whether this release dropped the LAST demander (refcount hit 0). NB: this
    /// reports the DEMAND accounting, not the physical bridge — in C1 the mirror
    /// bridge LINGERS past the last release (reused on a re-demand; reclaimed at
    /// idle self-exit), and C2 wires the real per-topic teardown. Named for the
    /// demand truth so it never misleads about the bridge lifecycle.
    pub last_release: bool,
}

/// The response to [`Request::Status`] (Principle #3 observability).
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct StatusResponse {
    /// Echoed correlation id.
    pub id: u64,
    /// Every demanded `(robot, topic)` with its refcount, sorted canonically.
    pub demands: Vec<DemandEntry>,
    /// The number of live consumer connections.
    pub active_connections: usize,
    /// Whether netd is currently idle (no connections, no demands) — the
    /// idle-grace timer toward self-exit is running.
    pub idle: bool,
    /// The zenoh connect endpoints this daemon is CONFIGURED TO DIAL —
    /// the explicit [`crate::net::CONNECT_ENV`] locators plus whatever the
    /// boot-time peer-cache fold added (see [`crate::discovery_fold`]). Lets an
    /// operator answer "why is my desk talking to that address?" directly.
    ///
    /// It does NOT answer "is that robot up?": the set is fixed before the
    /// transport initialises and netd's session is lazy, so a fiat-trusted dead
    /// locator appears here exactly like a live one.
    ///
    /// `Option`, not a bare `Vec`, deliberately: an older daemon does not
    /// send the field at all, and `#[serde(default)]` on a `Vec` would decode
    /// that absence as `[]` — a POSITIVE "this daemon dialled nothing" claim it
    /// never made. `None` means "this daemon does not report its connect set";
    /// `Some([])` means it genuinely folded nothing. Same distinction, and the
    /// same reason, as the `DiscoveryState` marker.
    #[serde(default)]
    pub connect_endpoints: Option<Vec<String>>,
}

/// The response to [`Request::RegisterEgress`].
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct EgressResponse {
    /// Echoed correlation id.
    pub id: u64,
    /// The number of DISTINCT canonical egress topics THIS connection now holds
    /// registered (after this call — additive, so it grows with re-registration).
    pub registered_topics: usize,
    /// Whether THIS registration BOOTED the shared embedded egress gateway (the
    /// first egress plan on this daemon); `false` when it joined a running one.
    pub gateway_started: bool,
}

/// The response to [`Request::ReleaseEgress`].
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct EgressReleaseResponse {
    /// Echoed correlation id.
    pub id: u64,
    /// The number of canonical egress topics released for this connection (0 if it
    /// held no egress registration). NB: like the mirror bridge, the physical
    /// announce LINGERS on the shared gateway past release (reclaimed at netd's idle
    /// self-exit); this reports the CONNECTION-SCOPED demand accounting, not the
    /// physical announce lifecycle.
    pub released_topics: usize,
}

/// Whether netd's shared zenoh session had COMPLETED at least one
/// DISCOVERY PASS when it answered a query — the distinction between "discovery
/// ran and the thing genuinely is not out there" and "netd cannot answer yet".
///
/// # Why the protocol has to carry this
///
/// netd's session is LAZY: the FIRST query on a freshly-spawned daemon opens it,
/// and a just-opened scouting session has not yet completed the multicast scout →
/// TCP connect → session establishment → liveliness/queryable exchange that makes a
/// robot's announce tokens (and its `catalog` queryable) visible. Before the discovery marker the
/// resulting empty gather was indistinguishable on the wire from "the LAN was
/// reached and nobody has this topic", so the desk rendered a TERMINAL "not found"
/// for a robot that was streaming perfectly — and it healed on the next invocation
/// (the daemon was warm by then), which is the worst possible shape: a
/// self-contradicting error that also mis-educates the user about a topic that
/// exists. netd now WAITS out that window itself (see
/// `crate::query::COLD_START_DISCOVERY_BUDGET`) and, when the wait expires with
/// nothing seen, says SO here instead of claiming absence.
///
/// # Back-compat: additive on the WIRE, gated by the BANNER
///
/// The field is `#[serde(default)]`, so the wire decodes both ways: a v4
/// daemon's response carries no `discovery` key and still parses, and a v5 daemon's
/// response still parses for anything that ignores the key.
///
/// But that default is [`DiscoveryState::Settled`] — a POSITIVE assertion
/// ("discovery ran, so an empty answer is real absence") that an old daemon never
/// made and cannot make. Believing it would silently restore the exact cold-start
/// false "not found" this issue removes, on a daemon that is spawn-once and
/// long-lived (upgrading the CLI while a running vizd holds an old netd alive hits
/// it on every query until that daemon exits). So [`PROTOCOL_VERSION`] IS bumped —
/// to 5 — and the client DOWNGRADES a `< 5` daemon's reported state to
/// [`DiscoveryState::NotConverged`] rather than trusting the serde default: see
/// [`crate::client::trust_reported_discovery`] and
/// [`crate::client::DISCOVERY_MIN_DAEMON_VERSION`].
///
/// The bump is a TRUST gate, not a refusal: the query verbs still report
/// `min_daemon_version() == 3`, so they keep WORKING against a v3/v4 daemon — its
/// answers simply read "unknown" instead of "not found", and the client warns once
/// naming the one-step remedy (restart the daemon). Rebuild `cerulion-netd`
/// alongside the desk to get precise answers back (the same rule the
/// `CatalogEntry`-field note below states).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DiscoveryState {
    /// netd's session has completed at least one discovery pass — some query on
    /// this daemon has gathered a non-empty answer from the network since boot. An
    /// EMPTY result is therefore AUTHORITATIVE: discovery works here and the thing
    /// asked for is not being served.
    ///
    /// This is the serde DEFAULT, so an older daemon's response (no `discovery`
    /// key) DECODES as `Settled` — but the client does not BELIEVE it: a `< 5` daemon
    /// never made that assertion, so [`crate::client::trust_reported_discovery`]
    /// downgrades it to [`DiscoveryState::NotConverged`]. The default exists so the
    /// wire stays parseable both ways, not so an old daemon inherits an absence claim.
    #[default]
    Settled,
    /// netd's discovery has NOT converged: it has never gathered a non-empty answer
    /// from the network, and the cold-start grace expired. TWO causes produce this
    /// and the wire deliberately does not separate them, because the consumer's
    /// conclusion is the same for both and neither licenses an absence claim:
    ///
    /// 1. No robot answered the announce space at all (none discovered).
    /// 2. A robot WAS announced, but no `catalog`/`schema` GET came back inside its
    ///    window — so nothing was actually read. (Reachable: `query_robot_catalogs`
    ///    drops every robot that does not answer in time, so an announced-but-silent
    ///    robot yields an empty gather.)
    ///
    /// Either way netd never successfully read anything, so an empty result proves
    /// NOTHING about the thing asked for. Where the two causes CAN be separated they
    /// are, in netd's own log rather than on the wire: the cold-start `warn!` carries
    /// an announce discriminator that is `yes` / `no` / `not-observed` — the last for
    /// a query shape that ran no announce harvest (a SINGLE-ROBOT GET) or whose
    /// harvest FAILED, neither of which can say anything either way. That is operator
    /// diagnostics, not a consumer decision.
    NotConverged,
}

/// The response to [`Request::QueryCatalog`]. Carries every decoded
/// [`CatalogReply`] netd gathered over its ONE zenoh session — one per answering
/// robot. The list is authoritative ONLY when `discovery` is
/// [`DiscoveryState::Settled`]: an empty list under
/// [`DiscoveryState::NotConverged`] means netd never completed a discovery pass, NOT
/// that nobody has the topic. Either way the desk does NOT open its own session —
/// only a netd TRANSPORT failure, surfaced as an [`ErrorResponse`], triggers the
/// desk's transient-session fallback. A reply carrying an [`CatalogReply::error`] is
/// a serve-side REFUSAL (the queried robot's authorizer denied it) passed through
/// VERBATIM so the desk surfaces it loudly (never a silent empty).
///
/// The catalog rides through as `cerulion_core`'s OWN [`CatalogReply`] — netd
/// hand-mirrors no catalog struct of its own — so a field added to
/// `CatalogEntry` (`producer_count`, `liveness`) crosses this
/// middlebox with no netd-side change. What DOES matter is the BINARY: netd
/// decodes and re-encodes, so a netd built before such a field existed drops it,
/// and every consumer behind that daemon degrades to UNKNOWN for it. That
/// degradation is accurate (the desk renders an absent `liveness` exactly as it
/// renders an old robot's) but it is a REBUILD, not a protocol negotiation —
/// rebuild `cerulion-netd` alongside the desk. Pinned by
/// `a_liveness_carrying_catalog_survives_the_netd_re_encode`.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct CatalogQueryResponse {
    /// Echoed correlation id.
    pub id: u64,
    /// Every decoded catalog netd gathered (one per answering robot), sorted +
    /// deduped upstream. Empty when no robot answered.
    pub catalogs: Vec<CatalogReply>,
    /// Whether netd had completed a discovery pass when it answered.
    /// Additive with the older default; see [`DiscoveryState`].
    #[serde(default)]
    pub discovery: DiscoveryState,
    /// Milliseconds this daemon's query plane has been running WITHOUT ever
    /// settling, or `None` once it has settled / on a daemon that predates the field.
    ///
    /// The consumer's first-contact wait caps itself on this, which is what makes it
    /// a per-DAEMON wait rather than a per-command tax (see
    /// [`ConvergenceWait`](crate::convergence::ConvergenceWait)). No `PROTOCOL_VERSION`
    /// bump: `None` is plainly UNKNOWN and suppresses nothing, so an old daemon's
    /// absent field degrades to today's behaviour rather than making a claim it
    /// cannot support, unlike the `discovery` marker, whose serde default WAS a
    /// positive claim and did need the bump.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub plane_unsettled_ms: Option<u64>,
}

/// The response to [`Request::QuerySchema`]. Carries every decoded
/// [`SchemaReply`] netd gathered — found, structured not-found, OR refused (the
/// desk picks the first with non-empty `docs`; a docs-empty `error` reply is the
/// robot's explicit "I do not have / will not serve this type", surfaced loudly).
/// An EMPTY list means no robot answered — authoritatively so only when `discovery`
/// is [`DiscoveryState::Settled`]; a netd TRANSPORT failure comes back as
/// an [`ErrorResponse`] and triggers the desk's fallback.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct SchemaQueryResponse {
    /// Echoed correlation id.
    pub id: u64,
    /// Every decoded schema reply netd gathered (one per answering robot). Empty
    /// when no robot answered.
    pub replies: Vec<SchemaReply>,
    /// Whether netd had completed a discovery pass when it answered.
    /// Additive with the older default; see [`DiscoveryState`].
    #[serde(default)]
    pub discovery: DiscoveryState,
    /// Milliseconds this daemon's query plane has been running WITHOUT ever
    /// settling, or `None` once it has settled / on a daemon that predates the field.
    ///
    /// The consumer's first-contact wait caps itself on this, which is what makes it
    /// a per-DAEMON wait rather than a per-command tax (see
    /// [`ConvergenceWait`](crate::convergence::ConvergenceWait)). No `PROTOCOL_VERSION`
    /// bump: `None` is plainly UNKNOWN and suppresses nothing, so an old daemon's
    /// absent field degrades to today's behaviour rather than making a claim it
    /// cannot support, unlike the `discovery` marker, whose serde default WAS a
    /// positive claim and did need the bump.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub plane_unsettled_ms: Option<u64>,
}

/// The response to [`Request::QueryRuns`]. Carries every decoded
/// [`RunsReply`] netd gathered — one per robot that ANSWERED.
///
/// # A robot with no reply is UNKNOWN, never idle
///
/// The list is a possibly-STRICT SUBSET of the robots asked. Two shapes contribute
/// nothing and neither says anything about what that machine is running: a robot
/// whose binary predates the verb, and one running a Strict ingress-only gateway,
/// which declares no query surface at all and is therefore discoverable in
/// the announce space yet unanswerable here. So a consumer must key on reply
/// PRESENCE per robot — a robot with a reply and an empty `runs` is described by that
/// reply's own [`RunsCompleteness`](cerulion_core::RunsCompleteness), while a robot
/// with NO reply is simply unknown.
/// netd never synthesizes one (see `cerulion_core`'s `gather_runs_outcome`).
///
/// The runs ride through as `cerulion_core`'s OWN [`RunsReply`] — netd hand-mirrors
/// no run struct of its own — so a field added upstream crosses this middlebox with
/// no netd-side change. What DOES matter is the BINARY, exactly as for the catalog:
/// netd decodes and re-encodes, so a netd built before a field existed drops it.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct RunsQueryResponse {
    /// Echoed correlation id.
    pub id: u64,
    /// Every decoded runs reply netd gathered (one per ANSWERING robot). Empty when
    /// no robot answered — which is not an absence claim about any of them.
    ///
    /// # Why not `replies`
    ///
    /// [`Response`] is `#[serde(untagged)]`, so a line is matched by its REQUIRED
    /// discriminating field set, and [`SchemaQueryResponse`] already owns `replies`.
    /// An EMPTY list decodes as `Vec<T>` for ANY `T`, so a `replies`-keyed runs
    /// response with nothing gathered — the ordinary answer on a desk with no robots
    /// — would deserialize as a `SchemaQuery` (the earlier variant) and the client's
    /// `round_trip` would report a protocol error on a perfectly good answer. The
    /// distinct key keeps the variant sets non-overlapping, which is the property
    /// that enum's doc promises.
    pub run_replies: Vec<RunsReply>,
    /// Robots that ANSWERED with something netd could not decode, a
    /// wire skew or corruption.
    ///
    /// Reported SEPARATELY from the robots simply missing from
    /// [`Self::run_replies`], because the two absences have OPPOSITE remedies: a
    /// robot that did not answer may answer later, while one listed here will
    /// re-serve the same unusable bytes until it is rebuilt. A consumer renders it
    /// as a redeploy condition, never as "still discovering" — and netd stops
    /// retrying the moment one appears, since a retry re-reads the same bytes at
    /// the robot's full serve cost.
    ///
    /// Additive: a healthy answer OMITS the key.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub unusable: Vec<UnusableRunsAnswer>,
    /// Robots that were ASKED and did not answer at all, the
    /// COVERAGE half of the answer.
    ///
    /// [`Self::run_replies`] alone cannot distinguish a LAN where every robot
    /// answered from one where half stayed silent: both arrive as a list of
    /// replies. A consumer folding either into "these are the runs" then makes a
    /// SETTLED ABSENCE claim about machines nobody heard from — the same
    /// confident-empty defect [`Self::discovery`] closes at the discovery latch,
    /// reached instead through coverage.
    ///
    /// A third remedy, distinct from both siblings: such a robot may answer later
    /// (a slow serve), or never (a binary predating the verb, or a Strict
    /// ingress-only gateway that declares no query surface). So it is neither the
    /// redeploy of `unusable` nor the "keep refreshing" of an unsettled
    /// `discovery` — a consumer names the robot and withholds the claim.
    ///
    /// Additive: a fully-covered answer OMITS the key. A v7 daemon predating this
    /// field decodes to an EMPTY list, which is the pre-B4 behaviour exactly — it
    /// asserted full coverage implicitly, so nothing regresses, and the field is
    /// what lets a newer daemon stop asserting it.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub silent: Vec<String>,
    /// Whether netd had completed a discovery pass when it answered — see
    /// [`DiscoveryState`]. An empty [`Self::run_replies`] is a claim that the LAN was
    /// searched only under [`DiscoveryState::Settled`].
    ///
    /// **MANDATORY on the wire, with no `#[serde(default)]`** — deliberately unlike
    /// its `query_catalog` / `query_schema` siblings, which carry the default for
    /// back-compat with older daemons. The only plausible default is
    /// [`DiscoveryState::Settled`], a POSITIVE claim that discovery ran; here there is
    /// no older peer that could omit the key (this response type is v7-only), so a
    /// document without it is corrupt rather than lean and fails to decode LOUDLY
    /// instead. Same reasoning as `RunsReply::completeness`.
    pub discovery: DiscoveryState,
    /// Milliseconds this daemon's query plane has been running WITHOUT ever
    /// settling, or `None` once it has settled. `None` is plainly UNKNOWN and
    /// suppresses nothing, so it stays optional where `discovery` does not.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub plane_unsettled_ms: Option<u64>,
}

/// A structured error response (never a panic / dropped connection).
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ErrorResponse {
    /// Echoed correlation id, or `None` when the request line carried no numeric id.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub id: Option<u64>,
    /// The error detail.
    pub error: String,
    /// The offending robot, when the error is topic-scoped.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub robot: Option<String>,
    /// The offending topic, when the error is topic-scoped.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub topic: Option<String>,
}

/// The response to [`Request::SubscribeCatalog`] — the snapshot a consumer
/// starts from, so it never has to guess whether the state it already holds is current.
///
/// `version` is netd's announce-view GENERATION, bumped once per push. It is what
/// keeps a missed notification from wedging a stale sidebar forever: a consumer that
/// dropped its connection (or was slow enough that pushes merged) re-subscribes, reads
/// a version it has not seen, and refreshes — the version, not the delta, is the
/// authority on "am I current".
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct SubscribeCatalogResponse {
    /// Echoed correlation id.
    pub id: u64,
    /// The announce-view generation at subscribe time (0 on a daemon that has not yet
    /// published a change).
    pub version: u64,
    /// The robots currently announcing, sorted. The snapshot half — a consumer that
    /// connects mid-stream knows the CURRENT state without racing a second query.
    pub robots: Vec<String>,
    /// Whether an announce watch is actually running on this daemon. `false` means no
    /// push can ever arrive here (a daemon with no network plane — `--network off` /
    /// a mirror-only build), which the consumer is told PLAINLY so it keeps its own
    /// refresh path instead of waiting on an event that cannot come.
    pub watching: bool,
}

/// The SERVER-INITIATED catalog-change notification — the one message on
/// this seam netd writes UNPROMPTED.
///
/// It is deliberately a NOTIFICATION, not a delta stream: the consumer's response is
/// to refresh, so the payload's job is to say *that* something changed, *which robots*
/// it involved, and *how current* the sender is — not to reconstruct the catalog. That
/// keeps the line small (a 75-topic attach graph is one short line, not a 75-name
/// list), keeps the per-connection push slot cheap, and — because `version` is
/// authoritative — makes a MISSED push harmless rather than corrupting.
///
/// Distinguished from a [`Response`] on the wire by its `event` field, which no
/// response carries; every response carries `id`, which this does not. A reader
/// classifies a line by which of the two it sees.
///
/// Only a connection that sent [`Request::SubscribeCatalog`] ever receives one.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct CatalogChanged {
    /// Always [`CATALOG_CHANGED_EVENT`] — the discriminator.
    pub event: String,
    /// The announce-view generation AFTER this change. Monotonic per daemon.
    pub version: u64,
    /// The robots announcing after this change, sorted — the authoritative
    /// post-change robot set (a consumer needs no further query to learn it).
    pub robots: Vec<String>,
    /// Robots that became present. NET over the coalescing window.
    pub robots_added: Vec<String>,
    /// Robots that went away entirely — the whole-robot transition (a dying gateway
    /// drops every token at once, and this reports it as ONE robot removal rather than
    /// as N topic removals). NET over the coalescing window.
    pub robots_removed: Vec<String>,
    /// How many topic announces appeared over the window (CHURN, not net).
    pub topics_added: u64,
    /// How many topic announces went away over the window (CHURN, not net).
    pub topics_removed: u64,
    /// How many observed announce batches this ONE notification stands for. `75` for
    /// an attach graph's burst — the true measure of what was coalesced away, and
    /// the number an operator reads to see a consumer falling behind.
    pub coalesced: u64,
}

/// The `event` discriminator [`CatalogChanged`] always carries.
pub const CATALOG_CHANGED_EVENT: &str = "catalog_changed";

impl CatalogChanged {
    /// Build a notification from a coalesced change.
    pub(crate) fn new(
        version: u64,
        delta: &crate::catalog_events::CatalogDelta,
        coalesced: u64,
        robots: &[String],
    ) -> Self {
        Self {
            event: CATALOG_CHANGED_EVENT.to_string(),
            version,
            robots: robots.to_vec(),
            robots_added: delta.robots_added.iter().cloned().collect(),
            robots_removed: delta.robots_removed.iter().cloned().collect(),
            topics_added: delta.topics_added,
            topics_removed: delta.topics_removed,
            coalesced,
        }
    }

    /// Fold a NEWER change into this still-undelivered notification (the slow-consumer
    /// path — see `crate::catalog_events::CatalogEventHub`). LOSSLESS: robot membership
    /// resolves to the net effect, churn counts sum, and the version + robot set take
    /// the newest values, so the one line the consumer eventually reads describes
    /// everything it missed.
    pub(crate) fn merge_newer(
        &mut self,
        delta: &crate::catalog_events::CatalogDelta,
        coalesced: u64,
        version: u64,
        robots: &[String],
    ) {
        let mut merged = crate::catalog_events::CatalogDelta {
            robots_added: self.robots_added.iter().cloned().collect(),
            robots_removed: self.robots_removed.iter().cloned().collect(),
            topics_added: self.topics_added,
            topics_removed: self.topics_removed,
        };
        merged.merge(delta.clone());
        self.robots_added = merged.robots_added.into_iter().collect();
        self.robots_removed = merged.robots_removed.into_iter().collect();
        self.topics_added = merged.topics_added;
        self.topics_removed = merged.topics_removed;
        self.coalesced = self.coalesced.saturating_add(coalesced);
        self.version = version;
        self.robots = robots.to_vec();
    }

    /// Serialize to the single NDJSON push line (no trailing newline — the writer adds
    /// it).
    pub fn to_json_line(&self) -> String {
        serde_json::to_string(self).expect("CatalogChanged serializes")
    }
}

/// Classify ONE line read from a netd control connection.
///
/// A subscribed connection's stream carries BOTH responses and unsolicited
/// [`CatalogChanged`] pushes, so a reader must be able to tell them apart before
/// parsing. The discriminator is structural, not positional: a push carries `event`
/// and no `id`; every response carries `id` and no `event`. A line that is neither
/// (unparseable, or an object with neither key) is reported as such rather than
/// guessed at.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ControlLine {
    /// An unsolicited catalog-change push.
    Event(CatalogChanged),
    /// A response to a request (correlate on its `id`).
    Response(String),
    /// Neither — the caller decides whether to warn or skip. Carries the raw line.
    Unknown(String),
}

/// Classify one NDJSON control line — see [`ControlLine`].
pub fn classify_control_line(line: &str) -> ControlLine {
    let trimmed = line.trim();
    let Ok(value) = serde_json::from_str::<serde_json::Value>(trimmed) else {
        return ControlLine::Unknown(trimmed.to_string());
    };
    if value.get("event").and_then(serde_json::Value::as_str) == Some(CATALOG_CHANGED_EVENT) {
        return match serde_json::from_str::<CatalogChanged>(trimmed) {
            Ok(event) => ControlLine::Event(event),
            // Tagged as our event but shaped wrong — never silently treated as a
            // response (which is how a desync starts).
            Err(_) => ControlLine::Unknown(trimmed.to_string()),
        };
    }
    if value.get("id").is_some() {
        return ControlLine::Response(trimmed.to_string());
    }
    ControlLine::Unknown(trimmed.to_string())
}

/// One control response. `#[serde(untagged)]` so it is a FLAT object on the wire
/// (the `cerulion-vizd` shape) — the consumer reads `id` for correlation and each
/// variant's REQUIRED discriminating field(s) for the shape: `mirror_created`
/// ([`DemandResponse`]), `last_release` ([`ReleaseResponse`]), `demands`
/// ([`StatusResponse`]), `gateway_started`+`registered_topics` ([`EgressResponse`]),
/// `released_topics` ([`EgressReleaseResponse`]), `catalogs` ([`CatalogQueryResponse`]),
/// `replies` ([`SchemaQueryResponse`]), `run_replies` ([`RunsQueryResponse`]),
/// `watching` ([`SubscribeCatalogResponse`]), and
/// `error` ([`ErrorResponse`]). Those field
/// sets are non-overlapping, so serde deserializes each line unambiguously. netd only
/// ever SERIALIZES a `Response`; the consumer deserializes the concrete struct it
/// expects.
///
/// **That non-overlap is a LIVE constraint, not a description.** An empty `Vec`
/// decodes for any element type, so two variants keyed on the same field name are
/// ambiguous on exactly the empty answer — which is the common one. The
/// runs response is keyed `run_replies` for that reason rather than reusing
/// `replies`; a new variant must pick a key no other variant carries.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(untagged)]
pub enum Response {
    /// Local metadata, discriminated by its required `serving_schema` key.
    ServingSchemaSnapshot(ServingSchemaSnapshotResponse),
    /// Account operation result, discriminated by its required `account_access` key.
    AccountAccess(crate::account_access::AccountAccessResponse),
    /// Demand accepted.
    Demand(DemandResponse),
    /// Release accepted.
    Release(ReleaseResponse),
    /// A status report.
    Status(StatusResponse),
    /// An egress registration accepted.
    Egress(EgressResponse),
    /// An egress release accepted.
    EgressRelease(EgressReleaseResponse),
    /// A LAN catalog query result.
    CatalogQuery(CatalogQueryResponse),
    /// A schema-closure query result.
    SchemaQuery(SchemaQueryResponse),
    /// A live-runs query result. Discriminated by its REQUIRED
    /// `run_replies` field, which no other variant carries.
    RunsQuery(RunsQueryResponse),
    /// A catalog-change subscription accepted. Discriminated by its
    /// REQUIRED `watching` field, which no other variant carries.
    SubscribeCatalog(SubscribeCatalogResponse),
    /// A structured error.
    Error(ErrorResponse),
}

impl Response {
    /// Build a structured error response.
    pub fn error(
        id: Option<u64>,
        error: impl Into<String>,
        robot: Option<String>,
        topic: Option<String>,
    ) -> Self {
        Response::Error(ErrorResponse {
            id,
            error: error.into(),
            robot,
            topic,
        })
    }

    /// Serialize to the single NDJSON response line (no trailing newline — the
    /// writer adds it).
    pub fn to_json_line(&self) -> String {
        serde_json::to_string(self).expect("Response serializes")
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn hello_carries_marker_and_version() {
        let h = Hello::new();
        assert_eq!(h.hello, HELLO_MARKER);
        assert_eq!(h.protocol, PROTOCOL_VERSION);
        // Round-trips as a flat object.
        let line = h.to_json_line();
        assert_eq!(
            serde_json::from_str::<Hello>(&line).expect("Hello round-trips"),
            h
        );
        // And carries the exact wire keys a consumer keys off.
        let v: serde_json::Value = serde_json::from_str(&line).unwrap();
        assert_eq!(v["hello"], "cerulion-netd");
        assert_eq!(v["protocol"], PROTOCOL_VERSION);
    }

    #[test]
    fn demand_request_round_trips_with_all_fields() {
        let req = Request::Demand {
            id: 7,
            robot: "ubuntu".to_string(),
            topic: "/utlidar/robot_odom".to_string(),
            schema_hash: 0x0123_4567_89ab_cdef,
        };
        let line = req.to_json_line();
        assert_eq!(parse_request(&line).expect("parses"), req);
        assert_eq!(req.id(), 7);
        // The `method` tag is on the wire.
        let v: serde_json::Value = serde_json::from_str(&line).unwrap();
        assert_eq!(v["method"], "demand");
        // A large schema_hash (> 2^53) round-trips EXACTLY through serde_json.
        assert_eq!(v["schema_hash"].as_u64(), Some(0x0123_4567_89ab_cdef));
    }

    #[test]
    fn register_and_release_egress_requests_round_trip() {
        use cerulion_core::{GatewayEgressPolicy, GatewayPlan};
        let reg = Request::RegisterEgress {
            id: 11,
            plan: GatewayPlan {
                egress_policy: GatewayEgressPolicy::AllowAll,
                announce: vec!["/robot/cmd_vel".to_string()],
                ingress: vec![],
            },
            schema_serving: SchemaServing::default(),
            ix_config_json: None,
        };
        let line = reg.to_json_line();
        assert_eq!(parse_request(&line).expect("register_egress parses"), reg);
        assert_eq!(reg.id(), 11);
        // The `method` tag + the nested plan are on the wire.
        let v: serde_json::Value = serde_json::from_str(&line).unwrap();
        assert_eq!(v["method"], "register_egress");
        assert_eq!(v["plan"]["announce"][0], "/robot/cmd_vel");

        let rel = Request::ReleaseEgress { id: 12 };
        assert_eq!(
            parse_request(&rel.to_json_line()).expect("release_egress parses"),
            rel
        );
        assert_eq!(rel.id(), 12);
        let v: serde_json::Value = serde_json::from_str(&rel.to_json_line()).unwrap();
        assert_eq!(v["method"], "release_egress");
    }

    #[test]
    fn query_catalog_request_round_trips_with_and_without_robot() {
        // No robot filter → the LAN gather (harvest + all robots).
        let all = Request::QueryCatalog {
            id: 21,
            robot: None,
        };
        let line = all.to_json_line();
        assert_eq!(parse_request(&line).expect("parses"), all);
        assert_eq!(all.id(), 21);
        let v: serde_json::Value = serde_json::from_str(&line).unwrap();
        assert_eq!(v["method"], "query_catalog");
        // `robot: None` omits nothing surprising — a bare-null or absent field both parse.
        let absent = r#"{"method":"query_catalog","id":22}"#;
        assert_eq!(
            parse_request(absent).expect("parses without robot"),
            Request::QueryCatalog {
                id: 22,
                robot: None
            }
        );
        // With a robot filter (vizd's single-robot resolve).
        let one = Request::QueryCatalog {
            id: 23,
            robot: Some("ubuntu".to_string()),
        };
        assert_eq!(parse_request(&one.to_json_line()).expect("parses"), one);
        let v: serde_json::Value = serde_json::from_str(&one.to_json_line()).unwrap();
        assert_eq!(v["robot"], "ubuntu");
    }

    #[test]
    fn query_schema_request_round_trips_with_and_without_robot() {
        let all = Request::QuerySchema {
            id: 31,
            robot: None,
            requested: "acme/Widget".to_string(),
        };
        let line = all.to_json_line();
        assert_eq!(parse_request(&line).expect("parses"), all);
        assert_eq!(all.id(), 31);
        let v: serde_json::Value = serde_json::from_str(&line).unwrap();
        assert_eq!(v["method"], "query_schema");
        assert_eq!(v["requested"], "acme/Widget");
        // A bare-name request (no package half) round-trips verbatim.
        let bare = Request::QuerySchema {
            id: 32,
            robot: Some("go2".to_string()),
            requested: "Widget".to_string(),
        };
        assert_eq!(parse_request(&bare.to_json_line()).expect("parses"), bare);
        // A missing `requested` is a parse error (it is required).
        assert!(parse_request(r#"{"method":"query_schema","id":33}"#).is_err());
    }

    /// The `discovery` marker's WIRE contract — the spellings operators and
    /// other-language consumers see, and that a response from a PRE-916 daemon (no
    /// `discovery` key at all) still PARSES, defaulting to [`DiscoveryState::Settled`].
    ///
    /// SCOPE: this pins the wire/serde layer ONLY. That default is deliberately NOT
    /// the semantics an old daemon's answer carries — [`PROTOCOL_VERSION`] was bumped
    /// to 5 precisely because it is a positive claim a `< 5` daemon never made, and
    /// [`crate::client::trust_reported_discovery`] downgrades it to `NotConverged`
    /// before any consumer sees it (pinned in `client.rs`, both purely and at the call
    /// site). Hand oracles: the literal JSON is written out, never re-derived from the
    /// serializer under test.
    #[test]
    fn discovery_state_wire_spellings_and_the_pre_916_absent_key_default() {
        assert_eq!(
            serde_json::to_string(&DiscoveryState::Settled).unwrap(),
            "\"settled\""
        );
        assert_eq!(
            serde_json::to_string(&DiscoveryState::NotConverged).unwrap(),
            "\"not_converged\""
        );
        assert_eq!(DiscoveryState::default(), DiscoveryState::Settled);

        // A pre-discovery-marker daemon's catalog response (no `discovery` key) — the exact
        // bytes a v4 daemon emits — must still PARSE, taking the serde default. What
        // that default MEANS is decided one layer up: the client's banner-version gate
        // downgrades a `< 5` daemon's report to NotConverged, so this `Settled` never
        // reaches a consumer as an absence licence.
        let legacy = r#"{"id":3,"catalogs":[]}"#;
        let decoded: CatalogQueryResponse = serde_json::from_str(legacy).expect("legacy decodes");
        assert_eq!(decoded.id, 3);
        assert!(decoded.catalogs.is_empty());
        assert_eq!(
            decoded.discovery,
            DiscoveryState::Settled,
            "a keyless response takes the serde default (the WIRE stays compatible); the \
             client's version gate is what refuses to believe it"
        );
        // Same for the schema response.
        let legacy = r#"{"id":4,"replies":[]}"#;
        let decoded: SchemaQueryResponse = serde_json::from_str(legacy).expect("legacy decodes");
        assert_eq!(decoded.discovery, DiscoveryState::Settled);

        // And a daemon's cold-start answer round-trips through the untagged
        // `Response` (the client's real decode path) carrying the marker.
        let cold = Response::CatalogQuery(CatalogQueryResponse {
            id: 5,
            catalogs: vec![],
            discovery: DiscoveryState::NotConverged,
            plane_unsettled_ms: None,
        });
        let line = cold.to_json_line();
        assert!(
            line.contains(r#""discovery":"not_converged""#),
            "the marker rides the wire verbatim: {line}"
        );
        assert_eq!(
            serde_json::from_str::<Response>(&line).expect("untagged decode"),
            cold
        );
    }

    #[test]
    fn query_verbs_report_v3_min_daemon_version() {
        let cat = Request::QueryCatalog { id: 1, robot: None };
        assert_eq!(cat.method_name(), "query_catalog");
        assert_eq!(cat.min_daemon_version(), 3, "the query verbs need v3");
        let sch = Request::QuerySchema {
            id: 1,
            robot: None,
            requested: "p/T".to_string(),
        };
        assert_eq!(sch.method_name(), "query_schema");
        assert_eq!(sch.min_daemon_version(), 3);
    }

    #[test]
    fn query_responses_serialize_flat_and_are_untagged_unambiguous() {
        use cerulion_core::{CatalogEntry, CatalogProvenance, CatalogReply, SchemaReply};
        // A catalog-query response is a FLAT object keyed by `catalogs` — no wrapper
        // key, and it deserializes through the untagged `Response` enum unambiguously
        // (not as Demand/Status/Error, whose required fields it lacks).
        let cat = Response::CatalogQuery(CatalogQueryResponse {
            id: 5,
            catalogs: vec![CatalogReply {
                version: 1,
                robot: "ubuntu".to_string(),
                entries: vec![CatalogEntry {
                    topic: "/tf".to_string(),
                    schema_hash: Some(7),
                    schema_name: Some("tf2_msgs/TFMessage".to_string()),
                    provenance: CatalogProvenance::Runtime,
                    producer_count: None,
                    liveness: None,
                }],
                error: None,
            }],
            discovery: DiscoveryState::Settled,
            plane_unsettled_ms: None,
        });
        let line = cat.to_json_line();
        let v: serde_json::Value = serde_json::from_str(&line).unwrap();
        assert_eq!(v["id"], 5);
        assert_eq!(v["catalogs"][0]["robot"], "ubuntu");
        assert!(v.get("CatalogQuery").is_none(), "flat untagged");
        // Round-trips through the untagged Response (the client's `round_trip` path).
        assert_eq!(
            serde_json::from_str::<Response>(&line).expect("untagged decode"),
            cat
        );
        // And the concrete struct decodes too (the direct-decode path).
        assert_eq!(
            serde_json::from_str::<CatalogQueryResponse>(&line)
                .unwrap()
                .id,
            5
        );

        // A schema-query response is keyed by `replies`.
        let sch = Response::SchemaQuery(SchemaQueryResponse {
            id: 6,
            replies: vec![SchemaReply::not_found("go2", "acme/Nope", "no such type")],
            discovery: DiscoveryState::Settled,
            plane_unsettled_ms: None,
        });
        let line = sch.to_json_line();
        let v: serde_json::Value = serde_json::from_str(&line).unwrap();
        assert_eq!(v["replies"][0]["robot"], "go2");
        assert_eq!(
            serde_json::from_str::<Response>(&line).expect("untagged decode"),
            sch
        );
        // An EMPTY catalog list still round-trips as a CatalogQuery (not an Error /
        // Status), so "netd reached the LAN, nobody answered" is distinguishable from
        // a transport failure.
        let empty = Response::CatalogQuery(CatalogQueryResponse {
            id: 7,
            catalogs: vec![],
            discovery: DiscoveryState::Settled,
            plane_unsettled_ms: None,
        });
        assert_eq!(
            serde_json::from_str::<Response>(&empty.to_json_line()).expect("decode"),
            empty
        );
    }

    /// A robot catalog carrying per-topic DATA-FLOW liveness survives
    /// netd's decode → re-encode → decode middlebox VERBATIM.
    ///
    /// netd sits between the robot's `catalog` serve and every desk consumer
    /// (`topic echo`/`info`/`hz`, `schema info`, vizd's `attach_remote`), and it
    /// does not pass bytes through — it decodes into `cerulion_core`'s
    /// `CatalogReply` and re-encodes onto its UDS. Anything the decode drops is
    /// invisible to the whole desk. This asserts the round trip against a HAND
    /// oracle, and asserts the JSON key is really present (a `liveness: None`
    /// round-trips too, so struct equality alone would pass on a dropped field).
    #[test]
    fn a_liveness_carrying_catalog_survives_the_netd_re_encode() {
        use cerulion_core::{CatalogEntry, CatalogProvenance, CatalogReply, TopicLiveness};
        // A streaming/silent topic pair: identical producer_count, only the observation
        // tells them apart.
        // The observed row also carries the robot's RATE ESTIMATE, which
        // is nested INSIDE `liveness` and therefore rides this same middlebox. It
        // is asserted separately below: netd decodes into a typed `TopicLiveness`,
        // so a field the decode does not know about is dropped silently and the
        // desk sidebar simply never shows a frequency for an unchecked topic —
        // exactly the symptom the rate estimate exists to remove.
        let observed = TopicLiveness {
            last_frame_age_ms: Some(48),
            observed_for_ms: 30_000,
            frames_observed: 600,
            rate_estimate: Some(cerulion_core::TopicRateEstimate {
                millihertz: 500_000,
                is_floor: false,
            }),
        };
        let dead = TopicLiveness {
            last_frame_age_ms: None,
            observed_for_ms: 30_000,
            frames_observed: 0,
            rate_estimate: None,
        };
        let entry = |topic: &str, liveness: Option<TopicLiveness>| CatalogEntry {
            topic: topic.to_string(),
            schema_hash: Some(7),
            schema_name: Some("sensor_msgs/PointCloud2".to_string()),
            provenance: CatalogProvenance::Runtime,
            producer_count: Some(1),
            liveness,
        };
        let resp = Response::CatalogQuery(CatalogQueryResponse {
            id: 11,
            catalogs: vec![CatalogReply {
                version: 1,
                robot: "go2".to_string(),
                entries: vec![
                    entry("/utlidar/cloud_deskewed", Some(observed)),
                    entry("/uslam/cloud_map", Some(dead)),
                    // An old robot's row: UNKNOWN must stay UNKNOWN, not become a value.
                    entry("/legacy", None),
                ],
                error: None,
            }],
            discovery: DiscoveryState::Settled,
            plane_unsettled_ms: None,
        });
        let line = resp.to_json_line();
        // The field really is ON netd's wire (not merely equal after a drop).
        let v: serde_json::Value = serde_json::from_str(&line).unwrap();
        assert_eq!(
            v["catalogs"][0]["entries"][0]["liveness"]["frames_observed"],
            600
        );
        assert_eq!(
            v["catalogs"][0]["entries"][1]["liveness"]["last_frame_age_ms"],
            serde_json::Value::Null
        );
        assert!(
            v["catalogs"][0]["entries"][2].get("liveness").is_none(),
            "an unobserved row carries NO liveness key: {line}"
        );
        // The nested rate estimate survives too, both halves of it.
        assert_eq!(
            v["catalogs"][0]["entries"][0]["liveness"]["rate_estimate"]["millihertz"],
            500_000
        );
        assert_eq!(
            v["catalogs"][0]["entries"][0]["liveness"]["rate_estimate"]["is_floor"],
            false
        );
        assert!(
            v["catalogs"][0]["entries"][1]["liveness"]
                .get("rate_estimate")
                .is_none(),
            "a dead route has no rate, and no rate means NO KEY — never a 0: {line}"
        );
        // And the full value round-trips through the untagged `Response` the
        // client decodes.
        assert_eq!(
            serde_json::from_str::<Response>(&line).expect("untagged decode"),
            resp
        );
    }

    #[test]
    fn a_serve_side_refusal_rides_through_the_catalog_query_verbatim() {
        // A queried robot's authorizer REFUSAL is a decodable
        // CatalogReply with `error: Some` + empty entries — it must pass through the
        // netd query response VERBATIM so the desk surfaces it loudly (never a silent
        // empty). (The desk-side loudness is pinned in the migration tests.)
        use cerulion_core::CatalogReply;
        let refused = CatalogReply::refused("go2", "not authorized (account/pairing)");
        let resp = Response::CatalogQuery(CatalogQueryResponse {
            id: 9,
            catalogs: vec![refused.clone()],
            discovery: DiscoveryState::Settled,
            plane_unsettled_ms: None,
        });
        let decoded = serde_json::from_str::<Response>(&resp.to_json_line()).unwrap();
        match decoded {
            Response::CatalogQuery(c) => {
                assert_eq!(c.catalogs.len(), 1);
                assert_eq!(
                    c.catalogs[0].error.as_deref(),
                    Some(refused.error.as_deref().unwrap())
                );
                assert!(c.catalogs[0].entries.is_empty(), "a refusal serves nothing");
            }
            other => panic!("expected CatalogQuery, got {other:?}"),
        }
    }

    #[test]
    fn verb_method_names_and_min_daemon_versions() {
        // demand/release/status are the v1 vocabulary; the egress verbs are v2.
        let demand = Request::Demand {
            id: 1,
            robot: "r".into(),
            topic: "/t".into(),
            schema_hash: 0,
        };
        assert_eq!(demand.method_name(), "demand");
        assert_eq!(demand.min_daemon_version(), 1);

        let rel = Request::Release {
            id: 1,
            robot: "r".into(),
            topic: "/t".into(),
        };
        assert_eq!(rel.method_name(), "release");
        assert_eq!(rel.min_daemon_version(), 1);

        assert_eq!(Request::Status { id: 1 }.min_daemon_version(), 1);

        let reg = Request::RegisterEgress {
            id: 1,
            plan: cerulion_core::GatewayPlan {
                egress_policy: cerulion_core::GatewayEgressPolicy::AllowAll,
                announce: vec!["/t".into()],
                ingress: vec![],
            },
            schema_serving: SchemaServing::default(),
            ix_config_json: None,
        };
        assert_eq!(reg.method_name(), "register_egress");
        assert_eq!(
            reg.min_daemon_version(),
            2,
            "a config-LESS register (monolith) stays a v2 egress verb"
        );

        // A CONFIG-CARRYING register (multi-process) needs a v4 daemon —
        // only v4+ verifies the forwarded namespace instead of silently ignoring it.
        let reg_cfg = Request::RegisterEgress {
            id: 1,
            plan: cerulion_core::GatewayPlan {
                egress_policy: cerulion_core::GatewayEgressPolicy::AllowAll,
                announce: vec!["/t".into()],
                ingress: vec![],
            },
            schema_serving: SchemaServing::default(),
            ix_config_json: Some(r#"{"global":{}}"#.to_string()),
        };
        assert_eq!(reg_cfg.method_name(), "register_egress");
        assert_eq!(
            reg_cfg.min_daemon_version(),
            4,
            "a config-carrying register needs a v4 (namespace-verifying) daemon"
        );

        let rele = Request::ReleaseEgress { id: 1 };
        assert_eq!(rele.method_name(), "release_egress");
        assert_eq!(rele.min_daemon_version(), 2);
    }

    #[test]
    fn register_egress_omitted_schema_serving_defaults() {
        // A minimal register_egress line WITHOUT a schema_serving OR ix_config_json
        // field boots with the empty serving + `None` config (`#[serde(default)]`) —
        // an older/leaner producer OR the monolith path (no namespace to verify).
        let line = r#"{"method":"register_egress","id":3,"plan":{"egress_policy":"AllowAll","announce":["/t"],"ingress":[]}}"#;
        match parse_request(line).expect("parses without schema_serving/ix_config_json") {
            Request::RegisterEgress {
                id,
                schema_serving,
                ix_config_json,
                ..
            } => {
                assert_eq!(id, 3);
                assert_eq!(schema_serving, SchemaServing::default());
                assert_eq!(
                    ix_config_json, None,
                    "an omitted ix_config_json defaults to None (the monolith / v2-producer shape)"
                );
                // A config-less register is still a v2 verb.
                assert_eq!(
                    parse_request(line).unwrap().min_daemon_version(),
                    2,
                    "the parsed config-less register is a v2 verb"
                );
            }
            other => panic!("expected RegisterEgress, got {other:?}"),
        }
    }

    #[test]
    fn register_egress_carries_ix_config_json_on_the_wire() {
        // A multi-process register carries the forwarded namespace JSON
        // verbatim; it round-trips and shows up under the `ix_config_json` wire key.
        use cerulion_core::{GatewayEgressPolicy, GatewayPlan};
        let cfg_json = r#"{"global":{"prefix":"iox2_"}}"#.to_string();
        let reg = Request::RegisterEgress {
            id: 44,
            plan: GatewayPlan {
                egress_policy: GatewayEgressPolicy::AllowAll,
                announce: vec!["/mp/telemetry".to_string()],
                ingress: vec![],
            },
            schema_serving: SchemaServing::default(),
            ix_config_json: Some(cfg_json.clone()),
        };
        let line = reg.to_json_line();
        assert_eq!(parse_request(&line).expect("config register parses"), reg);
        let v: serde_json::Value = serde_json::from_str(&line).unwrap();
        assert_eq!(v["ix_config_json"].as_str(), Some(cfg_json.as_str()));
    }

    #[test]
    fn egress_responses_serialize_with_discriminating_fields() {
        let e = Response::Egress(EgressResponse {
            id: 1,
            registered_topics: 3,
            gateway_started: true,
        });
        let v: serde_json::Value = serde_json::from_str(&e.to_json_line()).unwrap();
        assert_eq!(v["id"], 1);
        assert_eq!(v["registered_topics"], 3);
        assert_eq!(v["gateway_started"], true);
        assert!(v.get("Egress").is_none(), "flat untagged, no wrapper key");
        // The concrete struct round-trips (the consumer half).
        assert_eq!(
            serde_json::from_str::<EgressResponse>(&e.to_json_line()).unwrap(),
            EgressResponse {
                id: 1,
                registered_topics: 3,
                gateway_started: true,
            }
        );

        let r = Response::EgressRelease(EgressReleaseResponse {
            id: 2,
            released_topics: 2,
        });
        let v: serde_json::Value = serde_json::from_str(&r.to_json_line()).unwrap();
        assert_eq!(v["released_topics"], 2);
        assert_eq!(
            serde_json::from_str::<EgressReleaseResponse>(&r.to_json_line()).unwrap(),
            EgressReleaseResponse {
                id: 2,
                released_topics: 2,
            }
        );
    }

    #[test]
    fn release_and_status_requests_round_trip() {
        let rel = Request::Release {
            id: 8,
            robot: "ubuntu".to_string(),
            topic: "/tf".to_string(),
        };
        assert_eq!(parse_request(&rel.to_json_line()).unwrap(), rel);
        assert_eq!(rel.id(), 8);

        let st = Request::Status { id: 9 };
        assert_eq!(parse_request(&st.to_json_line()).unwrap(), st);
        assert_eq!(st.id(), 9);
    }

    #[test]
    fn parse_request_extracts_best_effort_id_on_unknown_method() {
        // Unknown method → error, but the id is still recovered for correlation.
        let err = parse_request(r#"{"method":"bogus","id":42}"#).expect_err("unknown method");
        assert_eq!(err.id, Some(42));
        assert!(!err.message.is_empty());
    }

    #[test]
    fn parse_request_missing_id_and_non_json_are_errors_without_id() {
        // Missing required `id`.
        let err = parse_request(r#"{"method":"status"}"#).expect_err("missing id");
        assert_eq!(err.id, None);
        // Not JSON at all.
        let err = parse_request("not json at all").expect_err("not json");
        assert_eq!(err.id, None);
        // Empty object.
        let err = parse_request("{}").expect_err("no method");
        assert_eq!(err.id, None);
    }

    #[test]
    fn parse_request_wrong_field_types_error_but_recover_id() {
        // schema_hash as a string is a type error, but the numeric id is recovered.
        let err = parse_request(
            r#"{"method":"demand","id":3,"robot":"r","topic":"/t","schema_hash":"nope"}"#,
        )
        .expect_err("bad schema_hash type");
        assert_eq!(err.id, Some(3));
    }

    #[test]
    fn demand_response_serializes_with_discriminating_field() {
        let resp = Response::Demand(DemandResponse {
            id: 1,
            robot: "ubuntu".to_string(),
            topic: "/utlidar/robot_odom".to_string(),
            refcount: 1,
            mirror_created: true,
        });
        let v: serde_json::Value = serde_json::from_str(&resp.to_json_line()).unwrap();
        assert_eq!(v["id"], 1);
        assert_eq!(v["robot"], "ubuntu");
        assert_eq!(v["refcount"], 1);
        assert_eq!(v["mirror_created"], true);
        // Flat object (untagged) — no wrapper key.
        assert!(v.get("Demand").is_none());
        // The concrete struct round-trips.
        assert_eq!(
            serde_json::from_str::<DemandResponse>(&resp.to_json_line()).unwrap(),
            DemandResponse {
                id: 1,
                robot: "ubuntu".to_string(),
                topic: "/utlidar/robot_odom".to_string(),
                refcount: 1,
                mirror_created: true,
            }
        );
    }

    #[test]
    fn release_and_status_responses_serialize() {
        let rel = Response::Release(ReleaseResponse {
            id: 2,
            robot: "ubuntu".to_string(),
            topic: "/tf".to_string(),
            refcount: 0,
            last_release: true,
        });
        let v: serde_json::Value = serde_json::from_str(&rel.to_json_line()).unwrap();
        assert_eq!(v["last_release"], true);
        assert_eq!(v["refcount"], 0);

        let st = Response::Status(StatusResponse {
            id: 3,
            demands: vec![DemandEntry {
                robot: "ubuntu".to_string(),
                topic: "/tf".to_string(),
                refcount: 2,
                plane: Some(ServingPlane::Iroh),
            }],
            active_connections: 3,
            idle: false,
            connect_endpoints: Some(vec!["tcp/10.0.0.5:7683".to_string()]),
        });
        let v: serde_json::Value = serde_json::from_str(&st.to_json_line()).unwrap();
        assert_eq!(v["active_connections"], 3);
        assert_eq!(v["idle"], false);
        assert_eq!(v["demands"][0]["refcount"], 2);
        assert_eq!(v["demands"][0]["topic"], "/tf");
        // The plane rides the same row, as the lowercase token operators grep for.
        assert_eq!(v["demands"][0]["plane"], "iroh");
        // The folded connect set rides the same response.
        assert_eq!(v["connect_endpoints"][0], "tcp/10.0.0.5:7683");
    }

    /// An older daemon does not send `connect_endpoints` at all, and
    /// that absence must decode to `None` — "this daemon does not report its
    /// connect set" — never to `Some([])`, which would be a positive "it dialled
    /// nothing" claim the old daemon never made. Hand-written wire line (the
    /// exact bytes an older daemon emits), so the pin cannot be satisfied by
    /// round-tripping our own encoder.
    #[test]
    fn a_pre_957_status_line_decodes_its_absent_connect_set_as_unknown() {
        let pre_957 = r#"{"id":3,"demands":[],"active_connections":1,"idle":false}"#;
        let decoded: StatusResponse = serde_json::from_str(pre_957).unwrap();
        assert_eq!(
            decoded.connect_endpoints, None,
            "an absent field is UNKNOWN, never an empty (and therefore positive) claim"
        );
        // The anti-tautology half: a daemon that DOES report an empty set is
        // distinguishable from the one above.
        let empty_reported =
            r#"{"id":3,"demands":[],"active_connections":1,"idle":false,"connect_endpoints":[]}"#;
        let decoded: StatusResponse = serde_json::from_str(empty_reported).unwrap();
        assert_eq!(
            decoded.connect_endpoints,
            Some(Vec::new()),
            "an explicitly empty set means 'nothing folded', which is a real answer"
        );
    }

    /// A daemon that predates the plane field omits it, and that absence must stay
    /// UNKNOWN. Reading it as the local-network plane would be a routing claim the
    /// old daemon never made, and it is the wrong one exactly when it matters: a
    /// robot reachable over both planes. Hand-written wire lines, so the pin cannot
    /// be satisfied by round-tripping our own encoder.
    #[test]
    fn a_status_row_without_a_plane_decodes_as_unknown_never_as_the_local_plane() {
        let older = r#"{"id":1,"demands":[{"robot":"r","topic":"/t","refcount":1}],"active_connections":1,"idle":false}"#;
        let decoded: StatusResponse = serde_json::from_str(older).unwrap();
        assert_eq!(
            decoded.demands[0].plane, None,
            "an absent plane is UNKNOWN, never a claim that the local plane served it"
        );
        // The anti-tautology half: a daemon that DOES report a plane is
        // distinguishable from the one above, on both values.
        for (token, expected) in [("zenoh", ServingPlane::Zenoh), ("iroh", ServingPlane::Iroh)] {
            let line = format!(
                r#"{{"id":1,"demands":[{{"robot":"r","topic":"/t","refcount":1,"plane":"{token}"}}],"active_connections":1,"idle":false}}"#
            );
            let decoded: StatusResponse = serde_json::from_str(&line).unwrap();
            assert_eq!(decoded.demands[0].plane, Some(expected));
            assert_eq!(expected.as_str(), token, "the token and the value agree");
        }
    }

    #[test]
    fn error_response_omits_none_fields_and_keeps_id() {
        let e = Response::error(Some(5), "boom", None, None);
        let v: serde_json::Value = serde_json::from_str(&e.to_json_line()).unwrap();
        assert_eq!(v["id"], 5);
        assert_eq!(v["error"], "boom");
        // None robot/topic are skipped on the wire.
        assert!(v.get("robot").is_none());
        assert!(v.get("topic").is_none());

        // A topic-scoped error carries the offending topic.
        let e = Response::error(
            None,
            "schema conflict",
            Some("ubuntu".to_string()),
            Some("/tf".to_string()),
        );
        let v: serde_json::Value = serde_json::from_str(&e.to_json_line()).unwrap();
        assert!(v.get("id").is_none(), "None id is skipped");
        assert_eq!(v["robot"], "ubuntu");
        assert_eq!(v["topic"], "/tf");
    }

    // ----------------------------------------------------------------------
    // The `runs` verb.
    // ----------------------------------------------------------------------

    fn run_entry(run_id: &str, graph: &str) -> cerulion_core::RunEntry {
        cerulion_core::RunEntry {
            run_id: run_id.to_string(),
            graph_name: graph.to_string(),
            run_started_at_ns: 1_700_000_000_000_000_000,
            state: cerulion_core::RunEntryState::Live,
            graph_yaml: "name: perception\nnodes:\n  - id: cam\n".to_string(),
            run_json: "{\"run_id\":\"0x1\"}\n".to_string(),
        }
    }

    /// A `RunsReply` shaped the way a robot's serve side really mints one.
    fn runs_reply(robot: &str, entries: Vec<cerulion_core::RunEntry>) -> RunsReply {
        RunsReply {
            version: 1,
            robot: robot.to_string(),
            runs: entries,
            completeness: cerulion_core::RunsCompleteness::Settled,
            undescribable: Vec::new(),
            error: None,
        }
    }

    #[test]
    fn the_runs_verb_reports_its_own_minimum_and_leaves_its_siblings_alone() {
        let runs = Request::QueryRuns { id: 1, robot: None };
        assert_eq!(runs.method_name(), "query_runs");
        assert_eq!(runs.id(), 1);
        assert_eq!(
            runs.min_daemon_version(),
            RUNS_MIN_DAEMON_VERSION,
            "the runs verb gates on the version that introduced it"
        );
        assert_eq!(RUNS_MIN_DAEMON_VERSION, 7);
        assert_eq!(
            PROTOCOL_VERSION, 7,
            "the banner bump is what makes the per-verb gate reachable"
        );

        // The SCOPED half: a bump that raised every verb's floor would lock a v6
        // consumer out of the vocabulary it has always been able to use.
        assert_eq!(
            Request::QueryCatalog { id: 2, robot: None }.min_daemon_version(),
            3
        );
        assert_eq!(
            Request::QuerySchema {
                id: 3,
                robot: None,
                requested: "pkg/Type".to_string(),
            }
            .min_daemon_version(),
            3
        );
        assert_eq!(Request::SubscribeCatalog { id: 4 }.min_daemon_version(), 6);
        assert_eq!(Request::Status { id: 5 }.min_daemon_version(), 1);

        // Robot scoping rides the wire; `None` is the fan-out.
        let one = Request::QueryRuns {
            id: 6,
            robot: Some("go2".to_string()),
        };
        let v: serde_json::Value = serde_json::from_str(&one.to_json_line()).unwrap();
        assert_eq!(v["method"], "query_runs");
        assert_eq!(v["robot"], "go2");
        assert_eq!(parse_request(&one.to_json_line()).expect("round trip"), one);
        let all = Request::QueryRuns { id: 7, robot: None };
        assert_eq!(parse_request(&all.to_json_line()).expect("round trip"), all);
    }

    /// **THE untagged-collision pin.** [`Response`] is matched by its REQUIRED
    /// discriminating field set, and an EMPTY `Vec` decodes for ANY element type —
    /// so a runs response keyed `replies` (the design row's spelling) would
    /// deserialize as the earlier `SchemaQuery` variant on exactly the answer a desk
    /// with no robots gets, and the client would report a protocol error on a
    /// perfectly good response.
    ///
    /// Driven on BOTH the empty and the populated shape, because the populated one
    /// passes by ACCIDENT (a `RunsReply` lacks `requested`, so it fails to decode as
    /// a `SchemaReply`) while the empty one is the real hazard.
    #[test]
    fn a_runs_query_response_is_never_decoded_as_a_schema_query() {
        // THE hazard: an EMPTY answer, which is what a desk with no robots gets.
        let empty = Response::RunsQuery(RunsQueryResponse {
            id: 11,
            run_replies: vec![],
            unusable: Vec::new(),
            silent: Vec::new(),
            discovery: DiscoveryState::Settled,
            plane_unsettled_ms: None,
        });
        let line = empty.to_json_line();
        let v: serde_json::Value = serde_json::from_str(&line).unwrap();
        assert!(
            v.get("run_replies").is_some() && v.get("replies").is_none(),
            "the runs response must NOT reuse the schema response's key: {line}"
        );
        assert_eq!(
            serde_json::from_str::<Response>(&line).expect("untagged decode"),
            empty,
            "an empty runs answer must decode as RunsQuery, not SchemaQuery"
        );

        // Populated.
        let full = Response::RunsQuery(RunsQueryResponse {
            id: 12,
            run_replies: vec![runs_reply("go2", vec![run_entry("0x2a", "perception")])],
            unusable: Vec::new(),
            silent: Vec::new(),
            discovery: DiscoveryState::Settled,
            plane_unsettled_ms: None,
        });
        assert_eq!(
            serde_json::from_str::<Response>(&full.to_json_line()).expect("untagged decode"),
            full
        );

        // The MIRROR: an EMPTY schema answer still decodes as SchemaQuery — the
        // anti-tautology half, without which "runs decodes as runs" is satisfied by
        // a build in which the two variants merely swapped places.
        let schema_empty = Response::SchemaQuery(SchemaQueryResponse {
            id: 13,
            replies: vec![],
            discovery: DiscoveryState::Settled,
            plane_unsettled_ms: None,
        });
        assert_eq!(
            serde_json::from_str::<Response>(&schema_empty.to_json_line()).expect("decode"),
            schema_empty
        );

        // The concrete-struct decode path (what a direct consumer uses) agrees.
        assert_eq!(
            serde_json::from_str::<RunsQueryResponse>(&line)
                .expect("concrete decode")
                .id,
            11
        );
    }

    /// `discovery` is MANDATORY on this response, deliberately unlike its siblings:
    /// the only plausible default is `Settled`, a POSITIVE "discovery ran, so an
    /// empty answer is real absence", and there is no older peer that could omit it
    /// (this type is v7-only). A document without it must fail LOUDLY rather than
    /// inherit a claim nobody made.
    #[test]
    fn a_runs_response_without_a_discovery_state_refuses_to_decode() {
        let without = r#"{"id":9,"run_replies":[]}"#;
        assert!(
            serde_json::from_str::<RunsQueryResponse>(without).is_err(),
            "an absent discovery state must not default to a positive absence claim"
        );
        // …and it does not fall through to some OTHER variant of the untagged enum
        // either, which would be a silently mis-typed response rather than an error.
        assert!(
            serde_json::from_str::<Response>(without).is_err(),
            "no variant may absorb a runs response missing its mandatory field"
        );
        // ANTI-TAUTOLOGY: the same document WITH the field decodes, so the refusal
        // above is about that key and not about the document being malformed.
        let with = r#"{"id":9,"run_replies":[],"discovery":"settled"}"#;
        let decoded: RunsQueryResponse = serde_json::from_str(with).expect("decodes with it");
        assert_eq!(decoded.discovery, DiscoveryState::Settled);
        assert_eq!(decoded.plane_unsettled_ms, None, "absent = UNKNOWN");

        // The age stays OPTIONAL — `None` is plainly unknown and caps
        // nothing, so it is skipped on the wire rather than sent as a number.
        let line = Response::RunsQuery(RunsQueryResponse {
            id: 9,
            run_replies: vec![],
            unusable: Vec::new(),
            silent: Vec::new(),
            discovery: DiscoveryState::NotConverged,
            plane_unsettled_ms: None,
        })
        .to_json_line();
        let v: serde_json::Value = serde_json::from_str(&line).unwrap();
        assert!(v.get("plane_unsettled_ms").is_none(), "skipped: {line}");
        assert_eq!(v["discovery"], "not_converged");
    }

    /// A robot's runs reply survives netd's decode → re-encode → decode
    /// middlebox VERBATIM — the [`a_liveness_carrying_catalog_survives_the_netd_re_encode`]
    /// twin.
    ///
    /// netd does not pass bytes through; it decodes into `cerulion_core`'s own
    /// [`RunsReply`] and re-encodes onto its UDS, so anything the decode drops is
    /// invisible to the whole desk. The two artifacts are the point: they are
    /// DOCUMENTS the desk re-parses with the same `GraphConfig` parser the robot
    /// used, so a mangled byte is a graph that renders wrong. Asserted on the JSON as
    /// well as through struct equality, because a dropped field round-trips to itself.
    #[test]
    fn a_runs_reply_survives_the_netd_re_encode() {
        // A verbatim graph.yaml carrying what JSON must escape: newlines, quotes, a
        // tab, and a non-ASCII byte.
        let graph_yaml = "name: \"percep\"\nnodes:\n\t- id: cam\n# ünicode\n";
        let run_json = "{\n  \"run_id\": \"0x0000000000000000000000000000002a\"\n}\n";
        let entry = cerulion_core::RunEntry {
            run_id: "0x0000000000000000000000000000002a".to_string(),
            graph_name: "perception".to_string(),
            run_started_at_ns: 1_700_000_000_123_456_789,
            state: cerulion_core::RunEntryState::Ending,
            graph_yaml: graph_yaml.to_string(),
            run_json: run_json.to_string(),
        };
        let reply = RunsReply {
            version: 1,
            robot: "go2".to_string(),
            runs: vec![entry],
            // An INCOMPLETE verdict with real counts — the unverified-absence carrier,
            // and the half a desk must not lose.
            completeness: cerulion_core::RunsCompleteness::Incomplete {
                live_writers: 3,
                writers_heard: 1,
            },
            undescribable: vec![cerulion_core::UndescribableRun {
                run_id: "0x0000000000000000000000000000002b".to_string(),
                reason: "run directory holds no readable graph.yaml".to_string(),
            }],
            error: None,
        };
        let resp = Response::RunsQuery(RunsQueryResponse {
            id: 21,
            run_replies: vec![reply],
            unusable: Vec::new(),
            silent: Vec::new(),
            discovery: DiscoveryState::Settled,
            plane_unsettled_ms: Some(4200),
        });
        let line = resp.to_json_line();
        let v: serde_json::Value = serde_json::from_str(&line).unwrap();

        // The DOCUMENTS really are on netd's wire, byte for byte.
        assert_eq!(v["run_replies"][0]["runs"][0]["graph_yaml"], graph_yaml);
        assert_eq!(v["run_replies"][0]["runs"][0]["run_json"], run_json);
        // The identity stays TEXT (B1: a JSON number of that magnitude cannot
        // survive a JavaScript consumer).
        assert_eq!(
            v["run_replies"][0]["runs"][0]["run_id"],
            "0x0000000000000000000000000000002a"
        );
        assert_eq!(v["run_replies"][0]["runs"][0]["state"], "ending");
        // The unverified-absence verdict and its counts.
        assert_eq!(v["run_replies"][0]["completeness"]["kind"], "incomplete");
        assert_eq!(v["run_replies"][0]["completeness"]["live_writers"], 3);
        assert_eq!(v["run_replies"][0]["completeness"]["writers_heard"], 1);
        // The withheld run, whose absence would make the short list invisible.
        assert_eq!(
            v["run_replies"][0]["undescribable"][0]["run_id"],
            "0x0000000000000000000000000000002b"
        );
        assert_eq!(v["plane_unsettled_ms"], 4200);

        // …and the whole value round-trips through the untagged `Response` the
        // client decodes.
        assert_eq!(
            serde_json::from_str::<Response>(&line).expect("untagged decode"),
            resp
        );
    }

    /// The unusable-answer list is ADDITIVE and its ABSENCE is the
    /// healthy answer — a robot that answered with bytes this binary cannot use is
    /// reported apart from one that simply did not answer.
    #[test]
    fn the_unusable_answer_list_is_additive_and_absent_when_healthy() {
        // HEALTHY: the key is omitted entirely, so a pre-B3 reader and a healthy
        // B3 answer are byte-identical on this field.
        let healthy = Response::RunsQuery(RunsQueryResponse {
            id: 41,
            run_replies: vec![],
            unusable: Vec::new(),
            silent: Vec::new(),
            discovery: DiscoveryState::Settled,
            plane_unsettled_ms: None,
        });
        let line = healthy.to_json_line();
        let v: serde_json::Value = serde_json::from_str(&line).unwrap();
        assert!(v.get("unusable").is_none(), "omitted when empty: {line}");
        assert_eq!(
            serde_json::from_str::<Response>(&line).expect("decode"),
            healthy
        );

        // SKEWED: the robot AND the reason reach the operator, beside a healthy
        // robot's runs (one robot's skew must not suppress another's answer).
        let skewed = Response::RunsQuery(RunsQueryResponse {
            id: 42,
            run_replies: vec![runs_reply("go2", vec![run_entry("0x2a", "perception")])],
            unusable: vec![cerulion_core::UnusableRunsAnswer {
                robot: "orin".to_string(),
                reason: "unknown runs wire version 2 (this binary supports 1)".to_string(),
            }],
            silent: Vec::new(),
            discovery: DiscoveryState::Settled,
            plane_unsettled_ms: None,
        });
        let line = skewed.to_json_line();
        let v: serde_json::Value = serde_json::from_str(&line).unwrap();
        assert_eq!(v["unusable"][0]["robot"], "orin");
        assert_eq!(
            v["unusable"][0]["reason"],
            "unknown runs wire version 2 (this binary supports 1)"
        );
        assert_eq!(v["run_replies"][0]["robot"], "go2");
        assert_eq!(
            serde_json::from_str::<Response>(&line).expect("untagged decode"),
            skewed
        );

        // A document with NO `unusable` key still decodes (the additive contract in
        // the other direction — a pre-B3-fix daemon's answer stays readable).
        let legacy = r#"{"id":43,"run_replies":[],"discovery":"settled"}"#;
        let decoded: RunsQueryResponse = serde_json::from_str(legacy).expect("legacy decodes");
        assert!(decoded.unusable.is_empty());
    }

    /// A serve-side REFUSAL rides through verbatim, exactly as the catalog's does —
    /// a desk must surface "the robot denied you" loudly, never as an empty list.
    #[test]
    fn a_serve_side_refusal_rides_through_the_runs_query_verbatim() {
        let refused = RunsReply {
            version: 1,
            robot: "go2".to_string(),
            runs: vec![],
            // A refusal can NEVER be settled (B1) — an empty list beside `Settled`
            // would tell an unpaired desk the robot is idle.
            completeness: cerulion_core::RunsCompleteness::not_established(),
            undescribable: Vec::new(),
            error: Some("not authorized (account/pairing)".to_string()),
        };
        let resp = Response::RunsQuery(RunsQueryResponse {
            id: 31,
            run_replies: vec![refused],
            unusable: Vec::new(),
            silent: Vec::new(),
            discovery: DiscoveryState::Settled,
            plane_unsettled_ms: None,
        });
        match serde_json::from_str::<Response>(&resp.to_json_line()).expect("decode") {
            Response::RunsQuery(r) => {
                assert_eq!(r.run_replies.len(), 1);
                assert_eq!(
                    r.run_replies[0].error.as_deref(),
                    Some("not authorized (account/pairing)")
                );
                assert!(r.run_replies[0].runs.is_empty(), "a refusal serves nothing");
                assert!(
                    !r.run_replies[0].completeness.is_settled(),
                    "a refused party learned nothing — the empty list is not an absence claim"
                );
            }
            other => panic!("expected RunsQuery, got {other:?}"),
        }
    }
}
