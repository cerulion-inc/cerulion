// SPDX-License-Identifier: AGPL-3.0-only
//! The WIRE PLANE (`cerulion/wire/1`): a demand-driven SHM tap
//! gateway arm speaking the `cerulion_q` catalog/demand vocabulary over
//! iroh, with per-topic uni data streams.
//!
//! A `cerulion/wire/1` connection (admitted ONLY for a paired `CAP_OBSERVE`
//! account — the accept gate runs FIRST, in [`crate::daemon`], per the
//! `cerulion_q` marker) carries:
//!
//! - **ONE bidi control stream** ([`cerulion_link::write_frame`]-framed JSON):
//!   [`WireRequest`] verbs `catalog` / `demand` / `undemand` / `schema` /
//!   `status`, each answered with a [`WireResponse`]. The vocabulary is the SAME
//!   `cerulion_q` shape used on the LAN ([`cerulion_core::transport::cerulion_q`]
//!   — imported, NOT mirrored): [`CatalogReply`] + [`SchemaReply`].
//! - **ONE uni data stream per demanded topic** (robot → desk): a JSON
//!   [`StreamPreamble`] naming the topic, then raw Cerulion wire frames VERBATIM
//!   (32-byte [`WireHeader`] + payload). Per-topic streams give QUIC per-stream
//!   flow control (a slow topic never stalls a fast one) and make the demand
//!   lifecycle = the stream lifecycle.
//!
//! The tap set + WAN drop-to-live backpressure live in [`crate::tap`] +
//! [`crate::forward`]; this module owns the control protocol + the connection
//! handler + the catalog/schema assembly.

use std::collections::{BTreeMap, BTreeSet};
use std::sync::{Arc, Mutex, OnceLock};
use std::time::{Duration, Instant};

use cerulion_core::transport::cerulion_q::{
    build_catalog_reply, collect_schema_closure, CatalogEntry, CatalogProvenance, CatalogReply,
    SchemaDoc, SchemaReply,
};
use cerulion_core::transport::demand_authorizer::{
    AllowAllAuthorizer, DemandAuthorizer, DemandDecision, DemandSubject,
};
use cerulion_core::{TransportConfig, TransportManager, WireHeader};
use cerulion_link::{
    accept_frame_stream, read_frame, write_frame, Connection, DEFAULT_MAX_FRAME_LEN,
};
use cerulion_pairing::verify::{
    EpochOutcome, EpochSyncWire, NO_EPOCH_SINK_NEEDLE, UNDECODABLE_REQUEST_NEEDLE,
};
use serde::{Deserialize, Serialize};

use crate::tap::{peek_schema_hash, TapManager};
use crate::trust::TrustError;
use crate::{RemotedClock, SharedTrust};

/// How often the INDEPENDENT revocation-sweep task (spawned by
/// [`serve_wire_connection`]) re-checks the demander against the LIVE demand authorizer,
/// so a mid-session owner-revoke / grant-expiry evicts an already-streaming demander
/// within this bound. A control-plane cadence (not the per-frame path); tests shorten it
/// via [`WirePlane::with_revocation_sweep_interval`].
const DEFAULT_REVOCATION_SWEEP_INTERVAL: Duration = Duration::from_secs(2);

/// Everything that can go wrong serving one wire-plane connection or handling one
/// verb. Kept per-verb-recoverable — a failed `demand` (unknown topic, no free
/// introspection slot) is surfaced to the desk over the control stream and the
/// connection stays alive.
#[derive(Debug, thiserror::Error)]
pub enum WireError {
    /// The wire plane needs `remoted`'s own iceoryx2 transport, which could not
    /// be initialized (a rare boot-time failure — e.g. shared-memory permissions).
    #[error("wire transport init: {0}")]
    Transport(String),

    /// A `demand` could not attach its tap / open its stream. Carries the topic +
    /// the actionable reason (topic does not exist, no free introspection slot,
    /// stream open failure), surfaced to the desk.
    #[error("demand '{topic}' failed: {reason}")]
    DemandFailed {
        /// The topic the desk demanded.
        topic: String,
        /// The actionable reason (already names the topic where relevant).
        reason: String,
    },

    /// A control frame or reply failed to (de)serialize.
    #[error("wire control (de)serialize: {0}")]
    Encode(String),
}

/// The first frame of every per-topic uni data stream: names the topic the
/// stream carries, so the desk correlates stream ↔ topic. All subsequent frames
/// on the stream are raw Cerulion wire frames (32-byte [`WireHeader`] + payload).
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct StreamPreamble {
    /// The canonical absolute topic name the following frames belong to.
    pub topic: String,
}

/// A control-stream request (desk → robot), length-prefixed JSON. The `verb` tag
/// dispatches; each verb is answered with a [`WireResponse`].
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(tag = "verb", rename_all = "snake_case")]
pub enum WireRequest {
    /// Return the robot's full topic catalog ([`CatalogReply`]).
    Catalog,
    /// Attach a tap for `topic` + open its robot→desk uni data stream.
    Demand {
        /// The canonical absolute topic to stream.
        topic: String,
    },
    /// Drop the tap for `topic` (its uni stream ends). An un-demanded topic costs
    /// nothing.
    Undemand {
        /// The canonical absolute topic to stop streaming.
        topic: String,
    },
    /// Serve `topic`'s `.msg`/YAML closure ([`SchemaReply`]) so a schema-less desk
    /// can decode its frames.
    Schema {
        /// The canonical absolute topic whose type closure is requested.
        topic: String,
    },
    /// Return the per-topic WAN counters ([`StatusReply`]).
    Status,
    /// PUSH the robot's latest access-list revocation epoch (the desk
    /// carries it from the account service and delivers it on connect — the
    /// desk-push sync, by design). Answered with [`WireResponse::EpochSynced`].
    ///
    /// `epoch_postcard` is `hex(postcard(`[`EpochSyncWire`]`))` — hex because the
    /// control frame is JSON and the cert/epoch types are byte-oriented serde
    /// (the exact convention the `grant_postcard` ops arg uses).
    ///
    /// The desk is a COURIER, not a trust anchor: the robot re-verifies the
    /// intermediate against its own root set, the epoch's signature, its robot id,
    /// and the monotonic epoch floor. Any VERIFICATION failure is a loud
    /// [`WireResponse::Error`] with NO state change. (A PERSIST failure is not a
    /// verification failure — the epoch is applied and enforcing, just not durable —
    /// and is reported as applied plus a robot-local `error!`.)
    SyncEpoch {
        /// `hex(postcard(EpochSyncWire))` — the intermediate + the signed epoch.
        epoch_postcard: String,
    },
}

/// A control-stream response (robot → desk), length-prefixed JSON. The `reply`
/// tag mirrors the request verb.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "reply", rename_all = "snake_case")]
pub enum WireResponse {
    /// The `catalog` reply — the exact `cerulion_q` [`CatalogReply`] shape.
    Catalog(CatalogReply),
    /// A `demand` was accepted: the topic's uni data stream is now open (the desk
    /// accepts it and reads the [`StreamPreamble`] then frames).
    DemandAccepted {
        /// The topic now streaming.
        topic: String,
    },
    /// An `undemand` result: `was_demanded` is `true` if a live tap was torn down.
    Undemanded {
        /// The topic.
        topic: String,
        /// Whether a live tap existed and was dropped.
        was_demanded: bool,
    },
    /// The `schema` reply — the `cerulion_q` [`SchemaReply`] shape (found closure
    /// OR an explicit structured not-found).
    Schema(SchemaReply),
    /// The `status` reply — per-topic WAN counters + Hz.
    Status(StatusReply),
    /// The `sync_epoch` result. `applied` distinguishes the two HEALTHY
    /// outcomes — `true` when this epoch was newer and took effect,
    /// `false` when it was not newer than the robot's current one (a stale push is
    /// a harmless no-op, which is exactly why an out-of-date desk cache can never
    /// roll the robot back). `epoch` is the robot's CURRENT epoch after the call in
    /// BOTH cases. A REJECTED epoch (bad signature, wrong robot, rollback, no sync
    /// sink) is never reported here — it is a loud [`WireResponse::Error`]. An epoch
    /// that applied but could not be PERSISTED reports `applied: true` (it IS
    /// enforcing) and screams robot-side about the lost durability.
    EpochSynced {
        /// The robot's current epoch number after this call.
        epoch: u64,
        /// Whether this push was newer and took effect (`false` = stale no-op).
        applied: bool,
    },
    /// A verb-level error (unknown/silent-topic demand, malformed request). Never
    /// silent — always surfaced here.
    Error {
        /// The topic the error concerns, when applicable.
        topic: Option<String>,
        /// The actionable human-readable message (names the topic where relevant).
        message: String,
    },
}

/// The `status` reply payload: one row per demanded topic.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct StatusReply {
    /// Per-topic WAN counter rows, sorted by topic.
    pub topics: Vec<TopicStatus>,
}

/// One demanded topic's WAN counters + free wire-timestamp Hz + live/dead state.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TopicStatus {
    /// The canonical absolute topic name.
    pub topic: String,
    /// Frames written to QUIC.
    pub wan_forwarded: u64,
    /// Frames dropped at the bounded drop-to-live queue (WAN slower than topic).
    pub wan_dropped: u64,
    /// Frames read off SHM (= forwarded + dropped + queued).
    pub frames_seen: u64,
    /// The topic's publish rate in Hz, from wire-timestamp deltas (0.0 until ≥2
    /// timestamped frames span a positive interval).
    pub hz: f64,
    /// Whether the topic's forward stream is still LIVE. `false` once
    /// its uni-stream writer has died (a QUIC stream reset) — the tap has been
    /// torn down (SHM slot released) and a re-demand will re-attach. Surfaced so
    /// `status` never misreports a dead stream as healthy (Principle #3).
    pub stream_alive: bool,
}

/// How the wire plane gets its `TransportManager` (remoted's OWN iceoryx2 node,
/// `network:None`, tapping the live run's DEFAULT namespace exactly as `bagd`).
enum ManagerCell {
    /// Production: lazily init the process singleton on the FIRST verb that needs
    /// SHM — so a `remoted` that never serves a wire peer never touches iceoryx2
    /// (the at-rest binding/refusal paths stay iroh-only).
    Lazy(OnceLock<Arc<TransportManager>>),
    /// A pre-supplied manager (tests: a per-test `init_for_test` SHM root; also
    /// usable to hand a production manager in explicitly).
    Ready(Arc<TransportManager>),
}

/// The shared wire-plane context: the robot identity + the tap-manager transport
/// plus an optional schema source. One `WirePlane` (behind an `Arc`) is shared
/// across every wire connection; each connection owns its own [`TapManager`].
pub struct WirePlane {
    robot: String,
    manager: ManagerCell,
    /// Optional schema source (EMPTY in the production MVP — `remoted` has no
    /// workspace `.msg` access yet): topic → its root qualified type name.
    topic_types: BTreeMap<String, String>,
    /// The served schema doc set, keyed by qualified name; the `schema` verb
    /// walks [`collect_schema_closure`] over it.
    schema_docs: BTreeMap<String, SchemaDoc>,
    /// The per-demand authorization gate. Each `demand` verb + a periodic
    /// mid-session sweep resolve the DEMANDER (the connection's authenticated remote
    /// peer key) through this. Production installs `PairingAuthorizer` (the SAME live
    /// `SharedTrust` the accept gate reads — so an owner-revoke / grant-expiry is
    /// honored per topic); the default is the deny-nothing [`AllowAllAuthorizer`], so a
    /// plane without an installed authorizer is byte-identical to the pre-gate plane.
    demand_authorizer: Arc<dyn DemandAuthorizer>,
    /// How often [`serve_wire_connection`] sweeps established taps for a
    /// mid-session revocation (see [`DEFAULT_REVOCATION_SWEEP_INTERVAL`]).
    revocation_sweep_interval: Duration,
    /// The revocation-epoch SYNC sink — the LIVE [`SharedTrust`] a pushed
    /// epoch is applied to, paired with the trusted clock `apply_epoch` needs.
    /// Production installs the SAME handle the accept gate + demand authorizer read,
    /// so an applied epoch takes effect on the very next accept AND evicts live
    /// sessions on the next sweep. `None` (the default) ⇒ the plane does not
    /// accept epoch sync and answers `sync_epoch` with a LOUD error — never a
    /// silent success that would let a desk believe a revocation landed.
    epoch_sink: Option<(SharedTrust, RemotedClock)>,
}

impl WirePlane {
    /// The PRODUCTION wire plane: a lazily-initialized transport singleton + no
    /// schema source. `robot` is the robot identity (the hostname, by design).
    pub fn lazy(robot: impl Into<String>) -> Self {
        Self {
            robot: robot.into(),
            manager: ManagerCell::Lazy(OnceLock::new()),
            topic_types: BTreeMap::new(),
            schema_docs: BTreeMap::new(),
            demand_authorizer: Arc::new(AllowAllAuthorizer),
            revocation_sweep_interval: DEFAULT_REVOCATION_SWEEP_INTERVAL,
            epoch_sink: None,
        }
    }

    /// A wire plane over an EXPLICIT transport manager (tests hand a per-test
    /// `init_for_test` manager; production may hand its own).
    pub fn with_manager(robot: impl Into<String>, manager: Arc<TransportManager>) -> Self {
        Self {
            robot: robot.into(),
            manager: ManagerCell::Ready(manager),
            topic_types: BTreeMap::new(),
            schema_docs: BTreeMap::new(),
            demand_authorizer: Arc::new(AllowAllAuthorizer),
            revocation_sweep_interval: DEFAULT_REVOCATION_SWEEP_INTERVAL,
            epoch_sink: None,
        }
    }

    /// Install the per-demand authorization gate (builder style). Production
    /// hands the robot's `PairingAuthorizer` (as `Arc<dyn DemandAuthorizer>`), so the
    /// serving plane gates each `demand` per topic AND evicts a revoked demander's live
    /// stream on the mid-session sweep — reading the SAME live `SharedTrust` the accept
    /// gate reads. Without this the default deny-nothing [`AllowAllAuthorizer`] admits
    /// every demand (the pre-gate behavior).
    pub fn with_demand_authorizer(mut self, authorizer: Arc<dyn DemandAuthorizer>) -> Self {
        self.demand_authorizer = authorizer;
        self
    }

    /// Override the mid-session revocation-sweep interval (builder style; tests
    /// shorten it so an eviction is observed fast).
    pub fn with_revocation_sweep_interval(mut self, interval: Duration) -> Self {
        self.revocation_sweep_interval = interval;
        self
    }

    /// Authorize (or refuse) a `demand` for `topic` from a demander, through
    /// the installed [`DemandAuthorizer`]. Consulted at each `demand` verb + on the
    /// mid-session revocation sweep.
    pub fn authorize_demand(&self, subject: &DemandSubject, topic: &str) -> DemandDecision {
        self.demand_authorizer.authorize_demand(subject, topic)
    }

    /// The mid-session revocation-sweep interval this plane serves under.
    pub fn revocation_sweep_interval(&self) -> Duration {
        self.revocation_sweep_interval
    }

    /// Install the revocation-epoch SYNC sink (builder style) — what makes
    /// the [`WireRequest::SyncEpoch`] verb REAL. Production hands the SAME live
    /// [`SharedTrust`] the accept gate + the demand authorizer read, plus the
    /// robot's trusted clock, so an epoch a desk pushes:
    ///
    /// 1. refuses NEW connections from the revoked party at the very next accept, and
    /// 2. EVICTS its live sessions on the next revocation sweep.
    ///
    /// The two are installed TOGETHER (one argument pair) so the sink can never be
    /// half-wired — a store with no clock could not call `apply_epoch` at all.
    /// Without this call the plane answers `sync_epoch` with a loud error rather than
    /// pretending a push landed.
    pub fn with_epoch_sink(mut self, shared: SharedTrust, clock: RemotedClock) -> Self {
        self.epoch_sink = Some((shared, clock));
        self
    }

    /// Apply a desk-pushed access-list epoch to the live trust store.
    ///
    /// The desk is a COURIER: every check that matters runs HERE, inside
    /// [`SharedTrust::apply_epoch`] → `TrustStore::apply_epoch` — the intermediate is
    /// re-verified against the robot's OWN root set, the epoch signature + issuer +
    /// robot id are checked, and the monotonic floor makes a stale push a no-op. So a
    /// hostile desk can push nothing the robot's own CA did not sign.
    ///
    /// Returns the `(current_epoch, applied)` pair on success (both healthy outcomes)
    /// or a loud, actionable reason on rejection.
    ///
    /// A PERSIST failure counts as applied — the live store was already mutated, so the
    /// epoch is enforcing — and is reported to the operator as a robot-local `error!`
    /// rather than to the desk as a rejection. See the arm's comment for why.
    fn apply_pushed_epoch(&self, blob_hex: &str) -> Result<(u64, bool), String> {
        let Some((shared, clock)) = self.epoch_sink.as_ref() else {
            return Err(format!(
                "this robot {NO_EPOCH_SINK_NEEDLE} (no sync sink installed on its wire plane) — \
                 the epoch was NOT applied"
            ));
        };
        let bytes = hex::decode(blob_hex).map_err(|e| {
            format!("epoch_postcard is not valid hex: {e}; the epoch was NOT applied")
        })?;
        let wire = EpochSyncWire::from_postcard(&bytes).map_err(|e| {
            format!("epoch_postcard did not decode as an epoch-sync artifact (postcard): {e}; the epoch was NOT applied")
        })?;
        let pushed = wire.signed_epoch.epoch_data.epoch;
        match shared.apply_epoch(&wire.signed_epoch, &wire.intermediate, clock.now_ns()) {
            Ok(EpochOutcome::Applied { newly_revoked }) => {
                tracing::info!(
                    epoch = pushed,
                    newly_revoked,
                    revoked_devices = wire.signed_epoch.epoch_data.revoked_devices.len(),
                    "cerulion_remoted wire: APPLIED a desk-pushed access-list epoch — \
                     new connections are refused now; live sessions are evicted on the next \
                     revocation sweep"
                );
                Ok((pushed, true))
            }
            Ok(EpochOutcome::NotNewer { current }) => {
                // A stale push is the EXPECTED steady state (every desk pushes what it
                // cached on every connect) — informational, never a warning, and never
                // a rollback: the monotonic floor discarded it.
                tracing::debug!(
                    pushed_epoch = pushed,
                    current_epoch = current,
                    "cerulion_remoted wire: a desk-pushed access-list epoch was not newer than \
                     the current one — no-op"
                );
                Ok((current, false))
            }
            // A PERSIST failure is NOT a rejection: `SharedTrust::apply_epoch` mutates
            // the live store BEFORE it writes, so the epoch IS applied and IS being
            // enforced right now — it just will not survive a reboot. Reporting this as
            // "REJECTED / not applied" would be false on both clauses and would send an
            // operator hunting a forged epoch instead of a full disk. The DESK is told
            // the truth for its own contract (delivery succeeded, the epoch is live);
            // the durability alarm is the ROBOT's own operational problem, so it screams
            // here at `error!` where the failing disk actually is.
            Err(TrustError::Persist(cause)) => {
                tracing::error!(
                    epoch = pushed,
                    cause = %cause,
                    "cerulion_remoted wire: a desk-pushed access-list epoch APPLIED in memory \
                     but could NOT be PERSISTED — it is enforcing NOW but will be \
                     LOST on reboot, reverting to the last durable epoch. Fix the trust-store \
                     write path (disk full / permissions / secure storage) and re-sync."
                );
                Ok((pushed, true))
            }
            Err(e) => Err(format!(
                "the pushed access-list epoch was REJECTED: {e}; the epoch was NOT applied. \
                 Causes: a forged/tampered epoch, one signed by an intermediate this robot's \
                 root set does not trust, one issued for a DIFFERENT robot, a rolled-back \
                 epoch, or a robot CLOCK skewed behind the issuing time (check the robot's \
                 clock/NTP if the epoch is known-good)"
            )),
        }
    }

    /// Attach a schema source (builder style): `topic_types` maps each topic to
    /// its root qualified type; `schema_docs` is the served doc set the `schema`
    /// verb walks. Used by tests and by any workspace-backed schema wiring.
    pub fn with_schema(
        mut self,
        topic_types: BTreeMap<String, String>,
        schema_docs: BTreeMap<String, SchemaDoc>,
    ) -> Self {
        self.topic_types = topic_types;
        self.schema_docs = schema_docs;
        self
    }

    /// The robot identity this plane self-attributes in every reply.
    pub fn robot(&self) -> &str {
        &self.robot
    }

    /// The transport manager, initializing the production singleton lazily on the
    /// first call. `network:None` default config — remoted taps the run's SHM,
    /// it never owns a zenoh session.
    pub fn manager(&self) -> Result<Arc<TransportManager>, WireError> {
        match &self.manager {
            ManagerCell::Ready(m) => Ok(m.clone()),
            ManagerCell::Lazy(cell) => {
                if let Some(m) = cell.get() {
                    return Ok(m.clone());
                }
                let m = TransportManager::init(TransportConfig::default())
                    .map_err(|e| WireError::Transport(e.to_string()))?;
                // A concurrent connection may win the set; both resolve the SAME
                // process singleton, so the winner's value is correct regardless.
                let _ = cell.set(m.clone());
                Ok(cell.get().cloned().unwrap_or(m))
            }
        }
    }

    /// Serve the `schema` verb for `topic`: resolve topic → root type → the
    /// [`collect_schema_closure`] over the served doc set (reusing the
    /// `cerulion_q` closure walk). An unknown topic / unresolvable root → an
    /// EXPLICIT structured [`SchemaReply::not_found`] (never silence, never an
    /// empty-200).
    pub fn serve_schema(&self, topic: &str) -> SchemaReply {
        match self.topic_types.get(topic) {
            Some(root) => match collect_schema_closure(root, &self.schema_docs) {
                Some(docs) => SchemaReply::found(&self.robot, topic, docs),
                None => SchemaReply::not_found(
                    &self.robot,
                    topic,
                    format!(
                        "root type '{root}' for topic '{topic}' is not in the served schema set"
                    ),
                ),
            },
            None => SchemaReply::not_found(
                &self.robot,
                topic,
                format!(
                    "no schema available for topic '{topic}' on this robot (the wire plane has no \
                     schema source wired)"
                ),
            ),
        }
    }
}

/// Serve ONE admitted `cerulion/wire/1` connection to completion. Accepts the
/// bidi control stream and loops read-request → dispatch → write-response until
/// the desk closes it (or the revocation sweep evicts it), then tears down
/// every demanded topic (releasing SHM slots). A long-lived session — the caller
/// MUST NOT wrap this in the accept handshake deadline.
///
/// Cancel-safety: `read_frame` / `write_frame` are NOT cancel-safe, so we
/// follow `cerulion_link`'s documented pattern — a DEDICATED reader task owns
/// `recv` and runs every `read_frame` to COMPLETION, handing whole request frames
/// over an mpsc channel. The control loop drains that CHANNEL and runs
/// `handle_request` + `write_frame` to completion (never as a `select!` arm), so the
/// control stream is never left mid-frame.
///
/// Mid-session eviction: the revocation sweep runs on its OWN
/// spawned task, DECOUPLED from control-loop progress. A revoked demander that STALLS
/// its control-response stream (so the control loop's `write_frame` blocks on QUIC flow
/// control) is STILL evicted, because the sweep task re-checks the demander on its own
/// timer and CLOSES the connection on de-authorization — which resets every uni data
/// stream (evicting the data), unblocks the control loop's `write_frame`, and drives the
/// tap teardown. Today's account-scoped ACL revokes WHOLE-SUBJECT (a grant confers
/// `CAP_OBSERVE` over all topics or none), so a de-authorization evicts EVERY demanded
/// topic at once; per-topic teardown is not implemented (no per-topic authorizer exists).
pub async fn serve_wire_connection(connection: Connection, plane: Arc<WirePlane>) {
    let (mut send, recv) = match accept_frame_stream(&connection).await {
        Ok(pair) => pair,
        Err(e) => {
            tracing::debug!(error = %e, "cerulion_remoted wire: no control stream; closing");
            return;
        }
    };
    tracing::info!(
        remote_id = %connection.remote_id(),
        "cerulion_remoted wire: control stream open (paired CAP_OBSERVE); serving catalog/demand"
    );

    // The dedicated reader task (framing cancel-safety): owns `recv`, runs each
    // `read_frame` to completion, forwards whole request frames over a bounded
    // channel. A stream error / EOF ends the task and closes the channel, which the
    // control loop observes as `req_rx.recv() == None`.
    let (req_tx, mut req_rx) = tokio::sync::mpsc::channel::<Vec<u8>>(1);
    let reader = tokio::spawn(async move {
        let mut recv = recv;
        loop {
            match read_frame(&mut recv, DEFAULT_MAX_FRAME_LEN).await {
                Ok(frame) => {
                    if req_tx.send(frame).await.is_err() {
                        break; // the control loop dropped the receiver
                    }
                }
                Err(e) => {
                    tracing::debug!(error = %e, "cerulion_remoted wire: control stream ended");
                    break;
                }
            }
        }
    });

    // The set of currently-demanded topics, kept in lockstep with the tap set
    // by the control loop (insert on demand-accept, remove on undemand). The INDEPENDENT
    // revocation-sweep task reads it to re-check the demander's authorization — WITHOUT
    // touching the control loop's `TapManager`, so a stalled control-write never starves
    // the sweep.
    let demanded: Arc<Mutex<BTreeSet<String>>> = Arc::new(Mutex::new(BTreeSet::new()));
    let sweep = tokio::spawn(run_revocation_sweep(
        plane.clone(),
        connection.clone(),
        Arc::clone(&demanded),
        plane.revocation_sweep_interval(),
    ));

    let mut taps = TapManager::new();
    while let Some(request_bytes) = req_rx.recv().await {
        let response = handle_request(&plane, &connection, &mut taps, &request_bytes).await;
        // Keep the shared demanded-set in lockstep with the tap set so the sweep re-checks
        // exactly the live streams.
        match &response {
            WireResponse::DemandAccepted { topic } => {
                lock_demanded(&demanded).insert(topic.clone());
            }
            WireResponse::Undemanded { topic, .. } => {
                lock_demanded(&demanded).remove(topic);
            }
            _ => {}
        }
        let response_bytes = match serde_json::to_vec(&response) {
            Ok(bytes) => bytes,
            Err(e) => {
                // The response types are plain — this cannot happen for a valid
                // response; log + skip rather than tear the connection down.
                tracing::error!(error = %e, "cerulion_remoted wire: response serialize failed");
                continue;
            }
        };
        if let Err(e) = write_frame(&mut send, &response_bytes).await {
            tracing::debug!(error = %e, "cerulion_remoted wire: control write failed; closing");
            break;
        }
    }

    // Connection closing (desk-closed, control error, OR the sweep task closed it on a
    // revoke): abort + AWAIT the sweep task (it may have closed the connection already;
    // awaiting reaps it so nothing leaks) and the reader (drops `recv`), then tear down
    // every demanded topic (drop taps → release SHM subscriber slots).
    sweep.abort();
    let _ = sweep.await;
    reader.abort();
    let _ = reader.await;
    taps.shutdown().await;
    let _ = send.finish();
    tracing::info!(
        remote_id = %connection.remote_id(),
        "cerulion_remoted wire: connection closed; all taps released"
    );
}

/// Lock the shared demanded-set, recovering a poisoned guard (a panic elsewhere must
/// never wedge the sweep/control coordination — the set is plain data).
fn lock_demanded(
    demanded: &Arc<Mutex<BTreeSet<String>>>,
) -> std::sync::MutexGuard<'_, BTreeSet<String>> {
    demanded.lock().unwrap_or_else(|p| p.into_inner())
}

/// The INDEPENDENT mid-session revocation sweep. Runs on its
/// OWN task so it is DECOUPLED from control-loop progress — a demander that stalls its
/// control-response stream is still evicted.
///
/// Every `interval`, re-check the demander (the connection's cryptographically-
/// authenticated remote peer key — never a self-reported field) against the LIVE
/// [`DemandAuthorizer`] over the currently-demanded topics. On ANY denial the demander
/// has lost wire access (today's account-scoped ACL flips WHOLE-SUBJECT — `CAP_OBSERVE`
/// over all topics or none), so evict LOUDLY by CLOSING the connection: that resets every
/// uni data stream (evicting the data), unblocks a stalled control `write_frame`, and
/// makes the control loop's reader error → the loop tears down the taps. Returns (the
/// task ends) once it has closed the connection; the `serve_wire_connection` teardown
/// aborts+awaits it on any other exit. A no-op tick when nothing is demanded.
async fn run_revocation_sweep(
    plane: Arc<WirePlane>,
    connection: Connection,
    demanded: Arc<Mutex<BTreeSet<String>>>,
    interval: Duration,
) {
    let remote = connection.remote_id();
    let subject = DemandSubject::Wan {
        demander_key: *remote.as_bytes(),
    };
    let mut ticker = tokio::time::interval(interval);
    ticker.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);
    ticker.tick().await; // consume the immediate t=0 tick (nothing to revoke at accept)
    loop {
        ticker.tick().await;
        // Snapshot the demanded topics under a brief lock (no `.await` held).
        let topics: Vec<String> = lock_demanded(&demanded).iter().cloned().collect();
        if topics.is_empty() {
            continue;
        }
        let denied: Vec<(String, String)> = topics
            .into_iter()
            .filter_map(|t| match plane.authorize_demand(&subject, &t) {
                DemandDecision::Deny { reason } => Some((t, reason)),
                DemandDecision::Allow => None,
            })
            .collect();
        if let Some((topic, reason)) = denied.first() {
            tracing::warn!(
                remote_id = %remote,
                denied_topics = denied.len(),
                topic = %topic,
                reason = %reason,
                "cerulion_remoted wire: demander DE-AUTHORIZED mid-session (owner-revoke / \
                 grant-expiry) — closing the connection, evicting all its streams"
            );
            // Cross-task connection close (iroh/quinn `Connection` is Arc-backed): resets
            // every stream + unblocks any stalled control write. Idempotent + thread-safe.
            connection.close(0u32.into(), b"access revoked");
            return;
        }
    }
}

/// Parse + dispatch ONE control request, returning the [`WireResponse`] to write.
/// A malformed request or a verb-level failure returns [`WireResponse::Error`]
/// (never silent).
async fn handle_request(
    plane: &Arc<WirePlane>,
    connection: &Connection,
    taps: &mut TapManager,
    request_bytes: &[u8],
) -> WireResponse {
    let request: WireRequest = match serde_json::from_slice(request_bytes) {
        Ok(r) => r,
        Err(e) => {
            // The marker is the SHARED `UNDECODABLE_REQUEST_NEEDLE` const (not a
            // literal): a desk classifies a `sync_epoch` answer carrying it as "this
            // robot predates the verb — upgrade it" rather than "this robot rejected
            // the epoch — investigate it". Those are different operator actions, and
            // a literal here could drift out from under the desk's classifier
            // silently. (The const's VALUE is byte-identical to the prefix this arm
            // has always emitted, so robots built before the const existed
            // — which can never be changed — classify correctly too.)
            return WireResponse::Error {
                topic: None,
                message: format!("{UNDECODABLE_REQUEST_NEEDLE}: {e}"),
            };
        }
    };

    match request {
        WireRequest::Catalog => match build_catalog(plane, taps).await {
            Ok(reply) => WireResponse::Catalog(reply),
            Err(e) => WireResponse::Error {
                topic: None,
                message: e.to_string(),
            },
        },
        WireRequest::Demand { topic } => {
            // Per-topic demand authorization. The accept gate already admitted
            // this connection (paired CAP_OBSERVE); the demand authorizer adds per-topic
            // granularity — a topic the DEMANDER (the connection's authenticated remote
            // peer key, never a self-reported field) is not authorized for is refused
            // LOUDLY here, never streamed.
            let remote = connection.remote_id();
            let subject = DemandSubject::Wan {
                demander_key: *remote.as_bytes(),
            };
            if let DemandDecision::Deny { reason } = plane.authorize_demand(&subject, &topic) {
                tracing::warn!(
                    remote_id = %remote,
                    topic = %topic,
                    reason = %reason,
                    "cerulion_remoted wire: demand REFUSED by the demand-authorization gate"
                );
                return WireResponse::Error {
                    topic: Some(topic.clone()),
                    message: format!("demand for '{topic}' refused: {reason}"),
                };
            }
            let manager = match plane.manager() {
                Ok(m) => m,
                Err(e) => {
                    return WireResponse::Error {
                        topic: Some(topic),
                        message: e.to_string(),
                    }
                }
            };
            match taps.demand(&manager, connection, &topic).await {
                Ok(()) => WireResponse::DemandAccepted { topic },
                Err(e) => WireResponse::Error {
                    topic: Some(topic),
                    message: e.to_string(),
                },
            }
        }
        WireRequest::Undemand { topic } => {
            let was_demanded = taps.undemand(&topic).await;
            WireResponse::Undemanded {
                topic,
                was_demanded,
            }
        }
        WireRequest::Schema { topic } => WireResponse::Schema(plane.serve_schema(&topic)),
        WireRequest::Status => WireResponse::Status(StatusReply {
            topics: taps.status(),
        }),
        // The desk pushed the robot's latest revocation epoch. Applying it
        // may revoke the VERY desk that pushed it — that is correct and expected (a
        // desk carrying its own revocation still delivers it faithfully). The revocation
        // sweep then evicts its DEMANDED STREAMS on its next tick; a connection with
        // no demands is not swept (the sweep skips an empty topic set) and simply
        // lingers harmlessly — every later demand is denied at the gate.
        WireRequest::SyncEpoch { epoch_postcard } => {
            match plane.apply_pushed_epoch(&epoch_postcard) {
                Ok((epoch, applied)) => WireResponse::EpochSynced { epoch, applied },
                Err(message) => {
                    tracing::warn!(
                        remote_id = %connection.remote_id(),
                        reason = %message,
                        "cerulion_remoted wire: REFUSED a desk-pushed access-list epoch"
                    );
                    WireResponse::Error {
                        topic: None,
                        message,
                    }
                }
            }
        }
    }
}

/// The whole-catalog PEEK budget cap: every `catalog` verb spends AT
/// MOST this long total peeking silent topics' schema hashes, regardless of how
/// many silent topics exist — so a Go2-class robot with N idle topics never
/// freezes the single control stream for N×750ms. Live topics peek fast (their
/// next publish lands within a poll or two) and cost far less; only genuinely
/// silent topics hit their per-topic budget. Repeated catalogs progressively fill
/// the per-connection hash cache, so re-peeking never recurs for a known topic.
const CATALOG_TOTAL_PEEK_BUDGET: Duration = Duration::from_secs(1);

/// Assemble the robot's [`CatalogReply`] from `remoted`'s LIVE SHM topic list +
/// per-topic wire `schema_hash`. The hash is a cache hit for ANY topic this
/// connection has already recorded (demanded OR peeked), else a
/// best-effort bounded PEEK ([`peek_schema_hash`]) under a whole-catalog time cap
/// ([`CATALOG_TOTAL_PEEK_BUDGET`]); a silent/unbudgeted topic gets
/// `schema_hash: None` (never fabricated). Runs the blocking iceoryx2
/// enumeration + peeks under `spawn_blocking`.
async fn build_catalog(
    plane: &Arc<WirePlane>,
    taps: &mut TapManager,
) -> Result<CatalogReply, WireError> {
    let manager = plane.manager()?;
    let robot = plane.robot().to_string();

    // Cache hits for EVERY topic this connection has recorded a hash for — a live
    // demand AND a prior catalog peek (a demanded-only set would re-peek
    // every peeked hash on every catalog). A known hash is never
    // re-peeked, so repeat catalogs on a connection converge to zero peek cost.
    let known: BTreeMap<String, u64> = taps.all_cached_schema_hashes();

    // Enumerate topics + peek the unknown ones off the async runtime, under a
    // whole-catalog time cap. `peeked` returns the freshly-observed hashes so we
    // can fold them into the per-connection cache below.
    let peeked: Vec<(String, Option<u64>)> = tokio::task::spawn_blocking(move || {
        let topics = manager.list_topics().map_err(|e| e.to_string())?;
        // Hard stop for the SUM of this catalog's peeks (the DoS bound).
        let catalog_deadline = Instant::now() + CATALOG_TOTAL_PEEK_BUDGET;
        let per_topic = crate::tap::catalog_peek_budget();
        let rows: Vec<(String, Option<u64>)> = topics
            .into_iter()
            .map(|topic| {
                let hash = match known.get(&topic) {
                    Some(h) => Some(*h),
                    None => {
                        let now = Instant::now();
                        if now >= catalog_deadline {
                            // The whole-catalog peek budget is spent; the rest of
                            // the silent topics get None this call (a later catalog
                            // fills them in from the growing cache).
                            None
                        } else {
                            // Cap each peek at the min of its per-topic budget and
                            // the remaining whole-catalog budget.
                            let deadline = (now + per_topic).min(catalog_deadline);
                            peek_schema_hash(&manager, &topic, deadline)
                        }
                    }
                };
                (topic, hash)
            })
            .collect();
        Ok::<Vec<(String, Option<u64>)>, String>(rows)
    })
    .await
    .map_err(|e| WireError::Transport(format!("catalog assembly task join failed: {e}")))?
    .map_err(WireError::Transport)?;

    // Record freshly-peeked hashes so a later catalog on this connection is free.
    // Provenance: `Runtime` — the closest `cerulion_q` variant for a topic
    // OBSERVED live at runtime (NOT a boot-plan `announce`). NOTE: the
    // `cerulion_q::CatalogProvenance::Runtime` doc names the LAN gateway's
    // `ros2 attach` dds_bridge route as its illustrative case; a `remoted` SHM tap
    // is a DIFFERENT runtime source. A dedicated `Shm`/`Tap` variant would be the
    // precise label, but that is an additive change to the SHARED `cerulion_q`
    // enum (a wire-compat concern for older desks decoding an unknown variant) —
    // so it is not added. `Runtime` (live/runtime, not boot) is
    // the closest 2-variant fit.
    let entries: Vec<CatalogEntry> = peeked
        .into_iter()
        .map(|(topic, hash)| {
            if let Some(h) = hash {
                taps.record_schema_hash(&topic, h);
            }
            CatalogEntry {
                topic,
                schema_hash: hash,
                schema_name: None,
                provenance: CatalogProvenance::Runtime,
                // The WAN/iroh remoted plane does not stamp the live
                // producer count (the LAN gateway's `build_catalog_reply_from_bridge`
                // probes it via `topic_publisher_count`). `None` degrades the desk to
                // a rendering with no liveness affordance for a WAN-served
                // catalog, exactly like an older robot. (The same probe could run
                // here off the `taps` SHM handle.)
                producer_count: None,
                // Likewise, the WAN/iroh remoted plane runs no
                // `TopicLivenessObserver` (that lives in the LAN gateway's drive
                // loop). `None` is the UNKNOWN verdict — the desk renders a
                // WAN-served row exactly as it renders an older robot's, never as
                // a dead route. An observer hosted here would
                // need the same LONG-LIVED taps, because a tap attached at catalog
                // time provably sees nothing already published (see the
                // `cerulion_core::transport::liveness` module docs).
                liveness: None,
            }
        })
        .collect();

    Ok(build_catalog_reply(&robot, entries))
}

/// Rebuild the full wire frame from a parsed [`WireHeader`] + its body — the
/// desk-side re-inject helper's inverse of the send path (`write_to_buf` is
/// `read_from_buf`'s total inverse). Exposed for the desk client + tests.
pub fn rebuild_frame(header: &WireHeader, body: &[u8]) -> Vec<u8> {
    let mut frame = vec![0u8; WireHeader::SIZE + body.len()];
    header.write_to_buf(&mut frame[..WireHeader::SIZE]);
    frame[WireHeader::SIZE..].copy_from_slice(body);
    frame
}

#[cfg(test)]
mod tests {
    use super::*;
    use cerulion_core::transport::cerulion_q::SchemaEncoding;

    /// The control protocol round-trips through JSON with the documented `verb` /
    /// `reply` tags (hand oracle — the exact wire vocabulary).
    #[test]
    fn wire_request_response_json_tags() {
        // Requests.
        assert_eq!(
            serde_json::to_string(&WireRequest::Catalog).unwrap(),
            r#"{"verb":"catalog"}"#
        );
        assert_eq!(
            serde_json::to_string(&WireRequest::Demand {
                topic: "/imu".to_string()
            })
            .unwrap(),
            r#"{"verb":"demand","topic":"/imu"}"#
        );
        // The epoch-push verb's exact tag + field name (a desk built against
        // this vocabulary must reach the robot's handler).
        assert_eq!(
            serde_json::to_string(&WireRequest::SyncEpoch {
                epoch_postcard: "ab12".to_string()
            })
            .unwrap(),
            r#"{"verb":"sync_epoch","epoch_postcard":"ab12"}"#
        );
        // Round-trip each request variant.
        for req in [
            WireRequest::Catalog,
            WireRequest::Demand {
                topic: "/a".to_string(),
            },
            WireRequest::Undemand {
                topic: "/a".to_string(),
            },
            WireRequest::Schema {
                topic: "/a".to_string(),
            },
            WireRequest::Status,
            WireRequest::SyncEpoch {
                epoch_postcard: "00ff".to_string(),
            },
        ] {
            let bytes = serde_json::to_vec(&req).unwrap();
            let back: WireRequest = serde_json::from_slice(&bytes).unwrap();
            assert_eq!(back, req);
        }
        // The reply tag + both healthy-outcome fields.
        assert_eq!(
            serde_json::to_string(&WireResponse::EpochSynced {
                epoch: 7,
                applied: true
            })
            .unwrap(),
            r#"{"reply":"epoch_synced","epoch":7,"applied":true}"#
        );
    }

    /// A plane with NO epoch sink installed must REFUSE a pushed epoch
    /// loudly — never report a success the desk would read as "the revocation
    /// landed". The default `WirePlane` has no sink, so this is the default
    /// behavior for any plane that forgot `with_epoch_sink`.
    #[test]
    fn sync_epoch_without_a_sink_is_a_loud_refusal() {
        let plane = WirePlane::lazy("robo");
        let err = plane
            .apply_pushed_epoch("00")
            .expect_err("no sink installed ⇒ refusal");
        assert!(
            err.contains("does not accept access-list epoch sync"),
            "the refusal must name the cause: {err}"
        );
        assert!(
            err.contains("NOT applied"),
            "the refusal must state the epoch did not land: {err}"
        );
    }

    // The malformed-blob (bad hex / bad postcard) and forged/stale/wrong-robot
    // refusal arms need a REAL `SharedTrust` sink to be reachable (the sink check
    // runs first — a robot that accepts no epoch sync says so regardless of the
    // blob). They live in `tests/epoch_push_e2e_test.rs`, which builds the real
    // trust store + genuine signed credentials.

    /// The stream preamble round-trips (the uni-stream topic-correlation frame).
    #[test]
    fn stream_preamble_round_trips() {
        let p = StreamPreamble {
            topic: "/utlidar/cloud".to_string(),
        };
        let bytes = serde_json::to_vec(&p).unwrap();
        assert_eq!(serde_json::from_slice::<StreamPreamble>(&bytes).unwrap(), p);
    }

    /// `serve_schema` reuses `collect_schema_closure`: a topic whose root type
    /// nests a custom dep serves BOTH docs (root first), decoded via the REAL
    /// `SchemaReply`. An unknown topic → an explicit not-found. Hand oracle.
    #[test]
    fn serve_schema_reuses_the_cerulion_q_closure() {
        let mut topic_types = BTreeMap::new();
        topic_types.insert("/state".to_string(), "acme/State".to_string());
        let mut docs = BTreeMap::new();
        docs.insert(
            "acme/State".to_string(),
            SchemaDoc {
                qualified: "acme/State".to_string(),
                encoding: SchemaEncoding::Msg,
                text: "acme/Sub sub\n".to_string(),
                deps: vec!["acme/Sub".to_string()],
            },
        );
        docs.insert(
            "acme/Sub".to_string(),
            SchemaDoc {
                qualified: "acme/Sub".to_string(),
                encoding: SchemaEncoding::Msg,
                text: "float64 x\n".to_string(),
                deps: vec![],
            },
        );
        let plane = WirePlane::lazy("robot1").with_schema(topic_types, docs);

        let found = plane.serve_schema("/state");
        assert_eq!(found.error, None);
        let names: Vec<&str> = found.docs.iter().map(|d| d.qualified.as_str()).collect();
        assert_eq!(names, vec!["acme/State", "acme/Sub"], "root first, closure");
        assert_eq!(found.robot, "robot1");

        // An unknown topic → explicit structured not-found (names the topic).
        let missing = plane.serve_schema("/nope");
        assert!(missing.docs.is_empty());
        let reason = missing.error.expect("not-found carries a reason");
        assert!(reason.contains("/nope"), "reason names the topic: {reason}");
    }

    /// `rebuild_frame` is `write_to_buf`'s inverse: header bytes + body reassemble
    /// to a full frame whose header re-parses to the same fields.
    #[test]
    fn rebuild_frame_round_trips_the_header() {
        let header = WireHeader {
            schema_hash: 0xDEAD_BEEF,
            total_size: (WireHeader::SIZE + 3) as u32,
            offset_table_offset: (WireHeader::SIZE + 3) as u32,
            offset_table_count: 0,
            sequence: 7,
            timestamp_ns: 123_456,
        };
        let frame = rebuild_frame(&header, &[1, 2, 3]);
        assert_eq!(frame.len(), WireHeader::SIZE + 3);
        assert_eq!(&frame[WireHeader::SIZE..], &[1, 2, 3]);
        let parsed = WireHeader::read_from_buf(&frame).expect("header re-parses");
        assert_eq!(parsed.schema_hash, 0xDEAD_BEEF);
        assert_eq!(parsed.sequence, 7);
        assert_eq!(parsed.timestamp_ns, 123_456);
    }
}
