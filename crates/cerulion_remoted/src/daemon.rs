// SPDX-License-Identifier: AGPL-3.0-only
//! The daemon: honor the kill switch, serve the [`crate::RemotePlane`] entry over
//! the configured files, and run the accept loop dispatching on the negotiated ALPN.
//!
//! Every connection is first classified by the [`PairingAuthorizer`] (deny-by-
//! default), then dispatched by plane:
//! - `cerulion/ops/1` → the real `cerud::OpsServer` over a `QuicOpsStream` adapter
//!   ([`crate::ops`]) — deploy/inventory/pair/claim/e-stop;
//! - `cerulion/wire/2` (WireAdmit) → the demand-driven SHM tap plane
//!   ([`crate::wire`]);
//! - `cerulion/rcp/1` (RcpAdmit) → the MCP proxy to the local `cerulion-rcpd`
//!   ([`crate::rcp`]);
//! - everything else (Refuse / unknown ALPN, or a plane not wired) → a
//!   deadline-bounded skeleton control-frame report that surfaces the
//!   [`AcceptDecision`] so the classification stays observable end-to-end.

use std::path::Path;
use std::sync::Arc;
use std::time::Duration;

use cerulion_core::TransportManager;
use cerulion_link::{
    accept_frame_stream, read_frame, write_frame, Accepted, Connection, Endpoint, EndpointId,
    IncomingAddr, LinkError, RelayUrl,
};
use cerulion_pairing::format::PublicKey;
use cerulion_pairing::verify::TrustStore;

use crate::authorizer::{AcceptDecision, PairingAuthorizer};
use crate::clock::RemotedClock;
use crate::config::RemotedConfig;
use crate::device_index::DeviceAccountIndex;
use crate::epoch_pull::EpochPull;
use crate::error::RemotedError;
use crate::host::{OpsHost, PlaneFiles, DAEMON_SERVER_NAME};
use crate::ops::OpsServing;
use crate::pairing_verbs::SharedCodePairSessions;
use crate::rcp::{RcpLink, RcpPlane};
use crate::trust::SharedTrust;
use crate::wire::WirePlane;

/// Cap on the skeleton control-frame handshake (hello + reply are tiny).
const MAX_HELLO_LEN: usize = 4096;

/// Deadline for incoming TLS and, separately, the skeleton control-frame handshake
/// (accept the bidi stream + read the hello + write the decision + drain to
/// EOF). A peer that completes the QUIC handshake then STALLS must not hold the
/// spawned task + connection indefinitely (an unauthenticated resource-hold DoS
/// on the always-on daemon). On timeout the connection is dropped. 10s is
/// generous for a hello+reply even over a relay; the ops and wire planes' protocols
/// carry their own tighter budgets.
///
/// At most 32 TLS handshakes are pending, and arrivals other than paired relay
/// keys fill at most 24 of them. Unvalidated addresses are retried from half
/// capacity. A paired relay key answers one Retry, which travels over the relay
/// to the key's holder, before it takes a slot. Validated IPs and paired relay
/// keys hold at most four slots each, unpaired relay arrivals share eight per
/// URL and custom transports share four. Established connections have their own
/// protocol deadlines.
const HANDSHAKE_DEADLINE: Duration = Duration::from_secs(10);

/// Bound unauthenticated work without serializing one peer ahead of safety ops.
const MAX_PENDING_HANDSHAKES: usize = 32;
/// Slots an arrival that is not a paired relay key may fill. The slots above
/// it, up to [`MAX_PENDING_HANDSHAKES`], are a reserve only paired relay keys
/// can take.
const UNPAIRED_PENDING: usize = 24;
const PER_SOURCE_PENDING: usize = 4;
const PER_UNPAIRED_RELAY_PENDING: usize = 8;

/// The admission cap that refused an arrival. Each cap has its own failure
/// regime ([`AdmissionLatches`]).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Cap {
    Global,
    UnpairedShare,
    Ip,
    UnpairedRelay,
    PairedRelay,
    Custom,
}

impl Cap {
    fn as_str(self) -> &'static str {
        match self {
            Self::Global => "global",
            Self::UnpairedShare => "unpaired_share",
            Self::Ip => "ip",
            Self::UnpairedRelay => "unpaired_relay",
            Self::PairedRelay => "paired_relay",
            Self::Custom => "custom",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Admission {
    Admit,
    /// Address validation under pressure: an unvalidated arrival once half the
    /// slots are pending, below the unpaired share.
    Retry,
    /// An unvalidated arrival under a paired relay key. iroh reads that key from
    /// the source address, which an IP datagram can forge, so the arrival takes
    /// no slot until it returns the Retry token. The Retry goes out over the
    /// relay, so only the key's holder receives it.
    KeyRetry,
    Refuse(Cap),
}

// Retry must engage before the unpaired share fills, and the share must leave
// a reserve below the global cap.
const _: () = assert!(
    MAX_PENDING_HANDSHAKES / 2 < UNPAIRED_PENDING && UNPAIRED_PENDING < MAX_PENDING_HANDSHAKES
);

/// The global cap, the paired key Retry, the unpaired share (paired relay keys
/// exempt), address Retry, then the per-IP cap. `ip_pending` is the count of
/// the arrival's validated IP, zero for every other source.
fn handshake_admission(
    pending: usize,
    ip_pending: usize,
    validated: bool,
    paired: bool,
) -> Admission {
    if pending >= MAX_PENDING_HANDSHAKES {
        Admission::Refuse(Cap::Global)
    } else if paired && !validated {
        Admission::KeyRetry
    } else if !paired && pending >= UNPAIRED_PENDING {
        Admission::Refuse(Cap::UnpairedShare)
    } else if !validated && pending >= MAX_PENDING_HANDSHAKES / 2 {
        Admission::Retry
    } else if validated && ip_pending >= PER_SOURCE_PENDING {
        Admission::Refuse(Cap::Ip)
    } else {
        Admission::Admit
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Hash)]
enum SourceKey {
    Ip(std::net::IpAddr),
    UnpairedRelay(RelayUrl),
    PairedRelay(EndpointId),
    Custom,
}

fn handshake_source(
    address: IncomingAddr,
    validated: bool,
    paired: impl Fn(&EndpointId) -> bool,
) -> Option<SourceKey> {
    match address {
        IncomingAddr::Ip(address) => validated.then_some(SourceKey::Ip(address.ip())),
        IncomingAddr::Relay { url, endpoint_id } => Some(if paired(&endpoint_id) {
            SourceKey::PairedRelay(endpoint_id)
        } else {
            SourceKey::UnpairedRelay(url)
        }),
        IncomingAddr::Custom(_) => Some(SourceKey::Custom),
        _ => Some(SourceKey::Custom),
    }
}

impl SourceKey {
    fn cap(&self) -> Cap {
        match self {
            Self::Ip(_) => Cap::Ip,
            Self::UnpairedRelay(_) => Cap::UnpairedRelay,
            Self::PairedRelay(_) => Cap::PairedRelay,
            Self::Custom => Cap::Custom,
        }
    }

    /// Pending handshakes this source may hold.
    fn limit(&self) -> usize {
        match self {
            Self::UnpairedRelay(_) => PER_UNPAIRED_RELAY_PENDING,
            Self::Ip(_) | Self::PairedRelay(_) | Self::Custom => PER_SOURCE_PENDING,
        }
    }
}

/// Pending handshakes already held by `source`. A paired relay key never reads
/// the count of its relay URL.
fn pending_from(sources: &indexmap::IndexMap<SourceKey, u8>, source: Option<&SourceKey>) -> usize {
    source
        .and_then(|key| sources.get(key))
        .map_or(0, |count| usize::from(*count))
}

fn source_admission(
    pending: usize,
    pending_from_source: usize,
    validated: bool,
    source: Option<&SourceKey>,
) -> Admission {
    let ip_pending = if matches!(source, Some(SourceKey::Ip(_))) {
        pending_from_source
    } else {
        0
    };
    let paired = matches!(source, Some(SourceKey::PairedRelay(_)));
    match handshake_admission(pending, ip_pending, validated, paired) {
        Admission::Admit => match source {
            Some(
                source @ (SourceKey::UnpairedRelay(_)
                | SourceKey::PairedRelay(_)
                | SourceKey::Custom),
            ) if pending_from_source >= source.limit() => Admission::Refuse(source.cap()),
            _ => Admission::Admit,
        },
        admission => admission,
    }
}

/// Whether an admission or completion at `pending` closes the global,
/// unpaired-share, address-retry and handshake-failure regimes: only below half
/// capacity.
fn shared_regimes_close(pending: usize) -> bool {
    pending < MAX_PENDING_HANDSHAKES / 2
}

/// The refusal regimes an admitted arrival closes, given the pending counts
/// before it. The shared regimes close only below half capacity, and the
/// arrival's source-class regime only while every source of that class holds
/// less than half its own limit, so arrivals that refill full buckets, from one
/// source or several, never close the regime those buckets hold open.
fn closed_regimes(
    pending: usize,
    source: Option<&SourceKey>,
    sources: &indexmap::IndexMap<SourceKey, u8>,
) -> [Option<Cap>; 3] {
    let shared = shared_regimes_close(pending);
    let class = source.map(SourceKey::cap).filter(|cap| {
        sources
            .iter()
            .filter(|(key, _)| key.cap() == *cap)
            .all(|(key, count)| usize::from(*count) < key.limit() / 2)
    });
    [
        shared.then_some(Cap::Global),
        shared.then_some(Cap::UnpairedShare),
        class,
    ]
}

/// One failure regime per admission cap, so one cap's refusals never close or
/// swallow another's.
#[derive(Default)]
struct AdmissionLatches {
    global: cerulion_core::transport::failure_regime_latch::FailureRegimeLatch,
    unpaired_share: cerulion_core::transport::failure_regime_latch::FailureRegimeLatch,
    ip: cerulion_core::transport::failure_regime_latch::FailureRegimeLatch,
    unpaired_relay: cerulion_core::transport::failure_regime_latch::FailureRegimeLatch,
    paired_relay: cerulion_core::transport::failure_regime_latch::FailureRegimeLatch,
    custom: cerulion_core::transport::failure_regime_latch::FailureRegimeLatch,
}

impl AdmissionLatches {
    fn latch(
        &mut self,
        cap: Cap,
    ) -> &mut cerulion_core::transport::failure_regime_latch::FailureRegimeLatch {
        match cap {
            Cap::Global => &mut self.global,
            Cap::UnpairedShare => &mut self.unpaired_share,
            Cap::Ip => &mut self.ip,
            Cap::UnpairedRelay => &mut self.unpaired_relay,
            Cap::PairedRelay => &mut self.paired_relay,
            Cap::Custom => &mut self.custom,
        }
    }

    fn refused(
        &mut self,
        cap: Cap,
    ) -> cerulion_core::transport::failure_regime_latch::RegimeDecision {
        self.latch(cap).on_failure()
    }

    fn admitted(&mut self, cap: Cap) -> Option<u64> {
        self.latch(cap).on_success()
    }

    fn total(&mut self, cap: Cap) -> u64 {
        self.latch(cap).total_failures()
    }
}

fn release_handshake_source(
    id: tokio::task::Id,
    task_sources: &mut indexmap::IndexMap<tokio::task::Id, SourceKey>,
    sources: &mut indexmap::IndexMap<SourceKey, u8>,
) {
    if let Some(source) = task_sources.shift_remove(&id) {
        if let Some(count) = sources.get_mut(&source) {
            *count -= 1;
            if *count == 0 {
                sources.shift_remove(&source);
            }
        }
    }
}

/// Build the daemon from config and serve until `shutdown` resolves (or the
/// endpoint closes). `shutdown` is dropped as soon as the accept loop stops,
/// before anything is logged and before the shutdown stage runs, or before the
/// kill-switch line when `run` will not serve.
///
/// Honors the `--network off` / `CERULION_NETWORK=off` kill-switch: if
/// engaged, it logs loudly and exits cleanly WITHOUT binding an endpoint.
/// Otherwise it serves the same library entry a host embeds
/// ([`RemotedConfig::daemon_plane`] with the [`RemotedConfig::key_files`]
/// provider), which keeps `restart`, `engage-estop` and the rcp plane.
pub async fn run(
    config: RemotedConfig,
    shutdown: impl std::future::Future<Output = ()>,
) -> Result<(), RemotedError> {
    if config.network_disabled {
        // Dropped before the line below, as `serve_planes` drops it before it
        // logs: a host that watches it learns serving will not start.
        drop(shutdown);
        tracing::warn!(
            "cerulion_remoted: network kill-switch engaged (--network off / \
             CERULION_NETWORK=off) — refusing to serve the remote plane; exiting \
             cleanly"
        );
        return Ok(());
    }

    // The store↔device binding check, the beacon facts and the endpoint close
    // live in the entry: `bind` refuses a store bound to another device, `serve`
    // closes the endpoint however serving ends.
    config
        .daemon_plane()
        .bind(&config.key_files())
        .await?
        .serve(shutdown)
        .await
}

/// The daemon's serving core over an endpoint a test built and device-bound itself:
/// [`serve_core`] with every ops verb and the daemon's wire plane.
///
/// `access_clock` is the trusted clock every access check reads (production: the robot
/// wall clock); `sweep_interval` overrides the wire plane's mid-session revocation-sweep
/// cadence (`None` = the production default); `manager` overrides the wire plane's SHM
/// transport (`None` = the production lazy singleton reading metadata through the default
/// control socket, `Some` = a per-test root with no metadata source). All three are
/// parameters ONLY so a daemon-level test can inject a fixed, advanceable clock + a short
/// sweep + an isolated SHM root and observe a mid-session revoke EVICT through the SAME
/// `.with_demand_authorizer(classify)` wiring `run` uses. `#[doc(hidden)]`, not a public
/// API. A host embeds [`crate::RemotePlane`] instead.
#[doc(hidden)]
#[allow(clippy::too_many_arguments)]
pub async fn serve_endpoint(
    config: &RemotedConfig,
    trust_store: TrustStore,
    device_index: DeviceAccountIndex,
    mac_key: Vec<u8>,
    endpoint: &Endpoint,
    access_clock: RemotedClock,
    sweep_interval: Option<Duration>,
    manager: Option<Arc<TransportManager>>,
    shutdown: impl std::future::Future<Output = ()>,
) -> Result<(), RemotedError> {
    // Robot identity resolves through `robot_identity_from_env`, the SAME
    // override-aware resolver the LAN gateway uses (`CERULION_ROBOT_IDENTITY` override →
    // else the hostname; by design the hostname is the default robot identity). The
    // transport manager is LAZY: a `remoted` that never serves a wire peer never
    // touches iceoryx2.
    let robot = cerulion_core::graph::robot_identity_from_env();
    let base = match manager {
        Some(m) => WirePlane::with_manager(robot, m),
        None => WirePlane::lazy(robot)
            .with_local_schema_provider(cerulion_netd::hygiene::default_socket_path()),
    };
    serve_core(
        &PlaneFiles::of(config),
        OpsHost::daemon(),
        DAEMON_SERVER_NAME,
        trust_store,
        device_index,
        mac_key,
        endpoint,
        access_clock,
        sweep_interval,
        base,
        Some(config.rcp.clone()),
        shutdown,
    )
    .await
}

/// The serving core every entry shares (its endpoint already built + device-bound):
/// build the LIVE shared trust state (accept gate reads it, pairing verbs mutate it, so a
/// claim/pair takes effect at the next accept with no restart), the ops plane with the
/// `ops_host` verb set announcing `server_name`, the authorizer-wired wire plane over
/// `base`, and, when `rcp` names the robot's rcpd, the rcp plane, then serve until
/// `shutdown`. On the daemon an armed pairing code keeps it serving until the
/// code's end is written, and an exhausted code to its window plus a margin (a
/// serving-hold source, bounded by that margin). The wire plane's `status`
/// reports the armed code's state and the refused requests on every host. When
/// [`serve_planes`] returns, after its shutdown stage, it ends the rcpd e-stop
/// delivery in flight and stops the epoch poller, then returns what
/// `serve_planes` returned.
#[allow(clippy::too_many_arguments)]
pub(crate) async fn serve_core(
    files: &PlaneFiles,
    ops_host: OpsHost,
    server_name: &str,
    trust_store: TrustStore,
    device_index: DeviceAccountIndex,
    mac_key: Vec<u8>,
    endpoint: &Endpoint,
    access_clock: RemotedClock,
    sweep_interval: Option<Duration>,
    base: WirePlane,
    rcp: Option<RcpLink>,
    shutdown: impl std::future::Future<Output = ()>,
) -> Result<(), RemotedError> {
    let shared =
        SharedTrust::new_with_clock(trust_store, device_index, mac_key, access_clock.clone());
    let classify = Arc::new(PairingAuthorizer::from_shared(shared.clone()));
    let shared_for_pull = shared.clone();
    let pull_clock = access_clock.clone();
    // The wire plane's epoch-sync sink gets the SAME live handle + the SAME
    // trusted clock the accept gate and the access list run on, so a desk-pushed
    // epoch is authoritative everywhere at once.
    let epoch_sink = shared.clone();

    // The ops plane over `cerulion/ops/1`: cerud's mechanical verbs + the pairing
    // bootstrap/safety verbs, gated by the same live trust state and receipted by
    // cerud's hash-chained audit log. Built once; reused (serialized) per session.
    let sessions = SharedCodePairSessions::new();
    // The wire plane's `status` reports the armed code of the SAME store the
    // code-pair verbs read.
    let status_sessions = sessions.clone();
    // The daemon's e-stop lease, for the rcp plane below: the same instance the
    // `engage-estop` and act verbs use, so its latch and its engage count are
    // shared. A library host has none, and serves no rcp plane.
    let estop_lease = ops_host.estop_lease();
    // On the daemon, `engage-estop` also sends an `estop` to the rcpd the rcp
    // plane forwards to, over the same link, until rcpd answers; the delivery
    // in flight ends when the planes stop.
    let ops_host = ops_host.with_rcpd_estop(rcp.as_ref(), &shared);
    let rcpd_estop = ops_host.rcpd_estop();
    let serving_holds = if server_name == DAEMON_SERVER_NAME {
        let root = files
            .beacon_facts_file
            .parent()
            .and_then(std::path::Path::parent)
            .ok_or_else(|| RemotedError::Config("remote state has no parent directory".into()))?;
        let path = crate::serving_hold::fence_path(root);
        let holds = crate::serving_hold::ServingHolds::open(&path)
            .map_err(|error| RemotedError::Ops(format!("opening remote serving hold: {error}")))?;
        {
            let lease = estop_lease.clone().ok_or_else(|| {
                RemotedError::Config("owned daemon is missing its e-stop idle hold source".into())
            })?;
            holds
                .register_source(Arc::new(move || lease.holds()))
                .map_err(|error| RemotedError::Ops(format!("registering e-stop hold: {error}")))?;
        }
        if let Some(rcpd) = rcpd_estop.clone() {
            holds
                .register_source(Arc::new(move || rcpd.is_owing()))
                .map_err(|error| {
                    RemotedError::Ops(format!("registering owed delivery hold: {error}"))
                })?;
        }
        // An armed pairing code keeps the daemon serving until its end is
        // written, an exhausted one to its window plus a margin, and never
        // past that. The source reads an atomic and the clock and takes no
        // lock: it runs under the holds' own lock.
        let codes = sessions.clone();
        holds
            .register_source(Arc::new(move || codes.holds_serving()))
            .map_err(|error| {
                RemotedError::Ops(format!("registering pairing code hold: {error}"))
            })?;
        Some(holds)
    } else {
        None
    };
    let ops = Arc::new(OpsServing::for_host(
        ops_host,
        shared,
        sessions,
        access_clock.clone(),
        &files.receipt_file,
        files.log_root.clone(),
        server_name,
        serving_holds,
    )?);

    // Install the SAME `PairingAuthorizer` as the wire plane's per-demand
    // authorization gate (this line is the wiring a daemon-level test pins). The serving
    // plane then (a) authorizes each `demand` per topic and (b) sweeps established taps
    // mid-session, so an owner-revoke / grant-expiry that lands AFTER the accept-time
    // admission EVICTS the already-streaming demander — both read the LIVE `SharedTrust`
    // the accept gate reads (no re-plumbing). `classify` is an `Arc<PairingAuthorizer>`; it
    // unsize-coerces to `Arc<dyn DemandAuthorizer>` here.
    //
    // Install the epoch-sync sink on the SAME plane (this line is the wiring
    // a daemon-level test pins). A desk that carries a newer revocation epoch delivers
    // it on connect; applying it here refuses the revoked party's NEXT accept and lets
    // the sweep above evict its LIVE session — the two halves of "revocation
    // reaches the robot" close on one shared handle.
    let mut wire_plane = base
        .with_demand_authorizer(classify.clone())
        .with_epoch_sink(epoch_sink, access_clock)
        .with_pair_code_status(status_sessions);
    if let Some(interval) = sweep_interval {
        wire_plane = wire_plane.with_revocation_sweep_interval(interval);
    }
    let wire = Arc::new(wire_plane);
    // The rcp plane authorizes each MCP message against the SAME live handle,
    // under the SAME e-stop lease as the ops verbs.
    let rcpd = rcp.as_ref().map(|link| link.addr);
    let rcp = match (rcp, estop_lease) {
        (Some(link), Some(lease)) => Some(Arc::new(RcpPlane::new(
            link,
            classify.shared().clone(),
            lease,
        ))),
        _ => None,
    };
    tracing::info!(
        endpoint_id = %endpoint.id(),
        robot = %wire.robot(),
        rcpd = ?rcpd,
        "cerulion_remoted: listening (ONE endpoint; ALPNs wire+ops, plus rcp on the daemon; \
         always-on)"
    );
    #[cfg(feature = "test-seam")]
    crate::log_writer::shrink_queue_for_test();
    #[cfg(feature = "test-seam")]
    crate::shutdown::close_endpoint_for_test(endpoint);

    // The robot's own revocation fetch: the supervisor's sync worker leaves
    // fetched epochs in the state directory, applied here on a poll. Started
    // only after every fallible plane is built, and owned by a guard that
    // aborts it however this future ends (an error, a cancelled future, a
    // normal return).
    let pull = files.store_file.parent().map(|dir| {
        let recheck = crate::epoch_pull::key_recheck_from_env();
        Arc::new(
            EpochPull::new(dir, shared_for_pull.clone(), pull_clock.clone()).with_recheck(recheck),
        )
    });
    let poller = AbortOnDrop(pull.clone().map(|pull| tokio::spawn(pull.run())));
    let served = serve_planes(
        endpoint,
        classify,
        Some(Arc::clone(&ops)),
        Some(wire),
        pull,
        rcp,
        shutdown,
    )
    .await;
    // `serve_planes` ran the ops plane's shutdown stage before it returned,
    // failed or not; the teardown below runs either way.
    if let Some(rcpd_estop) = rcpd_estop {
        rcpd_estop.shut_down().await;
    }
    poller.stop().await;
    tracing::info!("cerulion_remoted: the rcpd e-stop sender and the epoch poller are stopped");
    // The ops plane and its receipt sink stay alive until the teardown ends.
    drop(ops);
    served
}

/// A spawned task aborted when this guard drops.
struct AbortOnDrop(Option<tokio::task::JoinHandle<()>>);

impl AbortOnDrop {
    /// Abort the task and wait for the aborted future to end. An apply already
    /// handed to the blocking pool is not awaited: it runs to completion on its
    /// own. That apply replaces the trust store by a rename in the same
    /// directory, so the store file holds the old store or the new one, never
    /// a torn one.
    async fn stop(mut self) {
        if let Some(task) = self.0.take() {
            task.abort();
            let _ = task.await;
        }
    }
}

impl Drop for AbortOnDrop {
    fn drop(&mut self) {
        if let Some(task) = self.0.take() {
            task.abort();
        }
    }
}

/// Assert the loaded trust store belongs to THIS device: the endpoint id (the
/// device key's public half) must equal the store's `robot_transport_key`.
/// A mismatch is a LOUD [`RemotedError::ProvisioningMismatch`] (a transplanted
/// store governing the wrong robot). Pure so it is oracle-testable.
pub(crate) fn check_store_device_binding(
    endpoint_key: &[u8; 32],
    store_key: &PublicKey,
) -> Result<(), RemotedError> {
    if endpoint_key == &store_key.0 {
        Ok(())
    } else {
        Err(RemotedError::ProvisioningMismatch(format!(
            "the loaded trust store's robot_transport_key ({}) does not match this device's \
             endpoint id ({}) — the store belongs to a DIFFERENT robot (transplanted, restored \
             from another unit's backup, or fleet-misprovisioned). Refusing to serve this robot \
             under another robot's owner + access list; re-provision the store for this device",
            hex::encode(store_key.0),
            hex::encode(endpoint_key),
        )))
    }
}

/// The accept loop with NO wire plane (the ops-only path): a wire connection
/// reports its `AcceptDecision` over the skeleton control frame rather than serving
/// the SHM tap plane. Thin delegate to [`serve_with_wire`] with `wire = None` —
/// kept as the stable signature the accept-classification tests drive directly.
///
/// `classify` is the accept-time plane/pairing gate; `ops` is the ops-plane
/// serving context (the real `cerud` ops server over `QuicOpsStream`) — `None`
/// leaves the ops arm reporting the skeleton decision frame (used only by the
/// accept-classification tests; the daemon always passes `Some`).
pub async fn serve(
    endpoint: &Endpoint,
    classify: Arc<PairingAuthorizer>,
    ops: Option<Arc<OpsServing>>,
    shutdown: impl std::future::Future<Output = ()>,
) -> Result<(), RemotedError> {
    serve_with_wire(endpoint, classify, ops, None, shutdown).await
}

/// The accept loop: accept one connection at a time, spawning a per-connection
/// handler, until `shutdown` resolves or the endpoint closes.
///
/// Dispatch by plane: an ops connection → the real `cerud` ops server via `ops`;
/// when `wire` is `Some`, an admitted `cerulion/wire/2` connection is served the
/// demand-driven SHM tap plane ([`crate::wire`]); every other ALPN/decision (and a
/// `None` plane) gets the deadline-bounded skeleton report.
pub async fn serve_with_wire(
    endpoint: &Endpoint,
    classify: Arc<PairingAuthorizer>,
    ops: Option<Arc<OpsServing>>,
    wire: Option<Arc<WirePlane>>,
    shutdown: impl std::future::Future<Output = ()>,
) -> Result<(), RemotedError> {
    serve_with_epoch_pull(endpoint, classify, ops, wire, None, shutdown).await
}

/// [`serve_with_wire`] that also tells `pull` about every accepted key, so a
/// key not seen recently requests an early revocation fetch
/// ([`EpochPull::note_accept`]).
pub async fn serve_with_epoch_pull(
    endpoint: &Endpoint,
    classify: Arc<PairingAuthorizer>,
    ops: Option<Arc<OpsServing>>,
    wire: Option<Arc<WirePlane>>,
    pull: Option<Arc<EpochPull>>,
    shutdown: impl std::future::Future<Output = ()>,
) -> Result<(), RemotedError> {
    serve_planes(endpoint, classify, ops, wire, pull, None, shutdown).await
}

/// [`serve_with_epoch_pull`], plus the rcp plane: when `rcp` is `Some`, an
/// admitted `cerulion/rcp/1` connection is served by
/// [`crate::rcp::serve_rcp_connection`].
///
/// Once the accept loop stops, it drops `shutdown` before it logs anything,
/// then runs the
/// [`ShutdownStage`](crate::shutdown::ShutdownStage) of `ops` while the ops
/// receipt sink is alive, and returns [`RemotedError::Shutdown`] when a
/// participant did not complete. The endpoint and its connections are alive
/// during the stage after a stop request, and already gone when the endpoint
/// closed on its own.
pub async fn serve_planes(
    endpoint: &Endpoint,
    classify: Arc<PairingAuthorizer>,
    ops: Option<Arc<OpsServing>>,
    wire: Option<Arc<WirePlane>>,
    pull: Option<Arc<EpochPull>>,
    rcp: Option<Arc<RcpPlane>>,
    shutdown: impl std::future::Future<Output = ()>,
) -> Result<(), RemotedError> {
    let mut shutdown = Box::pin(shutdown);
    let holds = ops.as_ref().and_then(|ops| ops.serving_holds()).cloned();
    let fence = ops.as_ref().and_then(|ops| ops.admission_fence()).cloned();
    let mut refresh = tokio::time::interval(Duration::from_millis(100));
    let mut refresh_failures =
        cerulion_core::transport::failure_regime_latch::FailureRegimeLatch::default();
    let mut handshakes = tokio::task::JoinSet::<Result<Accepted, LinkError>>::new();
    let mut handshake_admission_failures = AdmissionLatches::default();
    let mut handshake_failures =
        cerulion_core::transport::failure_regime_latch::FailureRegimeLatch::default();
    let mut retry_unavailable =
        cerulion_core::transport::failure_regime_latch::FailureRegimeLatch::default();
    let mut handshake_retry_failures =
        cerulion_core::transport::failure_regime_latch::FailureRegimeLatch::default();
    let mut sources = indexmap::IndexMap::<SourceKey, u8>::new();
    let mut task_sources = indexmap::IndexMap::<tokio::task::Id, SourceKey>::new();
    let closed_on_its_own = loop {
        tokio::select! {
            biased;
            _ = &mut shutdown => break false,
            _ = refresh.tick() => {
                if let Some(holds) = &holds {
                    match holds.refresh() {
                        Ok(()) => if let Some(suppressed) = refresh_failures.on_success() { tracing::info!(suppressed, "remote serving hold refresh recovered"); },
                        Err(error) => {
                            use cerulion_core::transport::failure_regime_latch::RegimeDecision;
                            match refresh_failures.on_failure() {
                                RegimeDecision::Loud | RegimeDecision::StillFailing { .. } => tracing::error!(error = %error, total_failures = refresh_failures.total_failures(), "remote serving hold refresh failed"),
                                RegimeDecision::Suppressed { .. } => tracing::debug!(error = %error, total_failures = refresh_failures.total_failures(), "remote serving hold refresh failed"),
                            }
                        }
                    }
                }
            },
            // join_next is cancellation-safe; a tick cannot lose a completed
            // handshake. Deliver it before consuming another new arrival.
            completed = handshakes.join_next_with_id(), if !handshakes.is_empty() => {
                let accepted = match completed {
                    Some(Ok((id, accepted))) => {
                        release_handshake_source(id, &mut task_sources, &mut sources);
                        accepted
                    }
                    Some(Err(error)) => {
                        release_handshake_source(error.id(), &mut task_sources, &mut sources);
                        tracing::error!(error = %error, "remote handshake worker failed; continuing to serve");
                        continue;
                    }
                    None => continue,
                };
                match accepted {
                    Ok(a) => {
                        // Below half capacity only, so completions interleaved
                        // with a flood of failures cannot re-arm its loud line.
                        if shared_regimes_close(handshakes.len()) {
                            if let Some(suppressed) = handshake_failures.on_success() {
                                tracing::info!(suppressed, "remote TLS handshake completion recovered");
                            }
                        }
                        let classify = classify.clone();
                        let ops = ops.clone();
                        let wire = wire.clone();
                        let rcp = rcp.clone();
                        let fence = fence.clone();
                        let pull = pull.clone();
                        tokio::spawn(async move {
                            serve_accepted(a, classify, ops, wire, rcp, fence, pull).await;
                        });
                    }
                    // A SINGLE inbound connection failed its handshake (a scanner
                    // sending a bogus ALPN, a TLS failure, a peer aborting
                    // mid-handshake). This is per-connection, NOT the endpoint
                    // closing — an unauthenticated peer must NEVER be able to
                    // terminate the robot's always-on remote plane. Log through the
                    // handshake failure latch and keep serving. Only the endpoint
                    // accept stream ending below terminates serving; one failed
                    // handshake cannot do that.
                    Err(e) => {
                        use cerulion_core::transport::failure_regime_latch::RegimeDecision;
                        match handshake_failures.on_failure() {
                            RegimeDecision::Loud | RegimeDecision::StillFailing { .. } => tracing::warn!(error = %e, total_failures = handshake_failures.total_failures(), "remote TLS handshake failed; continuing to serve"),
                            RegimeDecision::Suppressed { .. } => tracing::debug!(error = %e, total_failures = handshake_failures.total_failures(), "remote TLS handshake failed; continuing to serve"),
                        }
                        continue;
                    }
                }
            }
            // Endpoint::accept has not consumed an Incoming until it resolves;
            // cancelling this wait on a refresh tick is safe. TLS completes in
            // bounded independent workers so a stalled peer cannot block ops.
            incoming = endpoint.accept() => {
                let Some(incoming) = incoming else { break true; };
                let validated = incoming.remote_addr_validated();
                let source = handshake_source(incoming.remote_addr(), validated, |id| {
                    classify.is_paired_and_allowed(id.as_bytes())
                });
                let pending_from_source = pending_from(&sources, source.as_ref());
                use cerulion_core::transport::failure_regime_latch::RegimeDecision;
                let admission = source_admission(handshakes.len(), pending_from_source, validated, source.as_ref());
                match admission {
                    Admission::Refuse(cap) => {
                        match handshake_admission_failures.refused(cap) {
                            RegimeDecision::Loud | RegimeDecision::StillFailing { .. } => tracing::warn!(pending = handshakes.len(), cap = cap.as_str(), total_failures = handshake_admission_failures.total(cap), "remote handshake refused at its admission cap"),
                            RegimeDecision::Suppressed { .. } => tracing::debug!(pending = handshakes.len(), cap = cap.as_str(), total_failures = handshake_admission_failures.total(cap), "remote handshake refused at its admission cap"),
                        }
                        incoming.refuse();
                        continue;
                    }
                    Admission::Retry | Admission::KeyRetry => {
                        // A paired key's Retry is its normal path, not pressure,
                        // so only address retry feeds the retry latch.
                        if admission == Admission::Retry {
                            match handshake_retry_failures.on_failure() {
                                RegimeDecision::Loud | RegimeDecision::StillFailing { .. } => tracing::debug!(pending = handshakes.len(), total_failures = handshake_retry_failures.total_failures(), "remote unvalidated handshake requires address retry"),
                                RegimeDecision::Suppressed { .. } => tracing::trace!(pending = handshakes.len(), total_failures = handshake_retry_failures.total_failures(), "remote unvalidated handshake requires address retry"),
                            }
                        }
                        match incoming.retry() {
                            Ok(()) => {
                                if let Some(suppressed) = retry_unavailable.on_success() {
                                    tracing::info!(suppressed, "remote handshake address retry became available");
                                }
                            }
                            Err(error) => {
                                match retry_unavailable.on_failure() {
                                    RegimeDecision::Loud | RegimeDecision::StillFailing { .. } => tracing::warn!(error = %error, pending = handshakes.len(), total_failures = retry_unavailable.total_failures(), "remote handshake retry unavailable; refusing incoming connection"),
                                    RegimeDecision::Suppressed { .. } => tracing::debug!(error = %error, pending = handshakes.len(), total_failures = retry_unavailable.total_failures(), "remote handshake retry unavailable; refusing incoming connection"),
                                }
                                error.into_incoming().refuse();
                            }
                        }
                        continue;
                    }
                    Admission::Admit => {}
                }
                for cap in closed_regimes(handshakes.len(), source.as_ref(), &sources).into_iter().flatten() {
                    if let Some(suppressed) = handshake_admission_failures.admitted(cap) {
                        tracing::info!(suppressed, cap = cap.as_str(), "remote handshake admission recovered");
                    }
                }
                if shared_regimes_close(handshakes.len()) {
                    if let Some(suppressed) = handshake_retry_failures.on_success() {
                        tracing::debug!(suppressed, "remote handshake address retry recovered");
                    }
                }
                let task = handshakes.spawn(async move {
                    let connection = tokio::time::timeout(HANDSHAKE_DEADLINE, async { incoming.await })
                        .await
                        .map_err(|error| LinkError::Accept(Box::new(error)))?
                        .map_err(|error| LinkError::Accept(Box::new(error)))?;
                    Ok(Accepted {
                        remote_id: connection.remote_id(),
                        alpn: connection.alpn().to_vec(),
                        connection,
                    })
                });
                if let Some(source) = source {
                    *sources.entry(source.clone()).or_insert(0) += 1;
                    task_sources.insert(task.id(), source);
                }
            }
        }
    };
    // Dropped before anything is logged or awaited, so a host that watches it
    // learns the accept loop stopped even when a log write blocks; the binary
    // arms its hard-exit deadline then.
    drop(shutdown);
    handshakes.shutdown().await;
    if closed_on_its_own {
        tracing::info!("cerulion_remoted: endpoint closed; stopping accept loop");
    } else {
        tracing::info!("cerulion_remoted: shutdown signalled; stopping accept loop");
    }
    // The ops receipt sink is alive here; the endpoint and its connections are
    // too, unless the endpoint closed on its own.
    if let Some(ops) = &ops {
        ops.shutdown_stage()
            .run()
            .await
            .map_err(RemotedError::Shutdown)?;
    }
    Ok(())
}

/// Classify one accepted connection, then run the skeleton per-plane handshake
/// stub with NO wire plane. Thin delegate to [`handle_accepted_with_wire`] with
/// `wire = None` — the stable signature the accept-classification tests drive
/// directly (they still exercise the ops arm via `ops`).
pub async fn handle_accepted(
    accepted: Accepted,
    classify: Arc<PairingAuthorizer>,
    ops: Option<Arc<OpsServing>>,
) {
    handle_accepted_with_wire(accepted, classify, ops, None).await
}

/// Classify one accepted connection through the authorizer, then dispatch on the
/// negotiated plane.
///
/// - **Ops plane** (`cerulion/ops/1`, classified `OpsAdmit` / `OpsBootstrapOnly`):
///   hand the connection to the real [`OpsServing`] — `cerud`'s `serve_connection`
///   over a [`cerulion_link::QuicOpsStream`]. The accept-time class is advisory
///   (logged); the ops server's per-verb [`PairingAuthorizer`] is the enforcement
///   point (an `OpsBootstrapOnly` key reaches ONLY the self-gating bootstrap
///   verbs; a paired key its capabilities). When `ops` is `None` the ops arm falls
///   back to the skeleton decision frame.
/// - **Wire plane** (`cerulion/wire/2`, `WireAdmit`, the accept gate already
///   required a paired `CAP_OBSERVE` account): handed to
///   [`crate::wire::serve_wire_connection`] as a LONG-LIVED session (NOT bounded by
///   the handshake deadline, which exists only for the tiny skeleton hello) when
///   `wire` is `Some`.
/// - **Everything else** (`Refuse` / `UnknownAlpn`, and a `None` plane): the
///   deadline-bounded skeleton control-frame report.
pub async fn handle_accepted_with_wire(
    accepted: Accepted,
    classify: Arc<PairingAuthorizer>,
    ops: Option<Arc<OpsServing>>,
    wire: Option<Arc<WirePlane>>,
) {
    handle_accepted_planes(accepted, classify, ops, wire, None).await
}

/// [`handle_accepted_with_wire`], plus the rcp arm: an `RcpAdmit` connection
/// is handed to [`crate::rcp::serve_rcp_connection`] as a long-lived session
/// when `rcp` is `Some`; with `None` it falls through to the skeleton report.
/// It takes no idle-teardown guard and notes no key for an epoch fetch: the
/// accept loop's own handler does both.
pub async fn handle_accepted_planes(
    accepted: Accepted,
    classify: Arc<PairingAuthorizer>,
    ops: Option<Arc<OpsServing>>,
    wire: Option<Arc<WirePlane>>,
    rcp: Option<Arc<RcpPlane>>,
) {
    serve_accepted(accepted, classify, ops, wire, rcp, None, None).await
}

/// Whether a connection with `decision` holds the serving fence that keeps
/// idle teardown off: only a session admitted with paired authority, on the
/// ops or rcp plane. A bootstrap-only, wire, refused or unknown-ALPN
/// connection holds none, so a peer that never pairs cannot keep the server
/// running.
fn fences_idle_teardown(decision: &AcceptDecision) -> bool {
    match decision {
        AcceptDecision::OpsAdmit | AcceptDecision::RcpAdmit => true,
        AcceptDecision::OpsBootstrapOnly
        | AcceptDecision::WireAdmit
        | AcceptDecision::Refuse { .. }
        | AcceptDecision::UnknownAlpn { .. } => false,
    }
}

/// Classify one accepted connection, then take its idle-teardown guard from
/// `fence` when [`fences_idle_teardown`] says the decision needs one, note the
/// key with `pull` and dispatch on the negotiated plane. When `fence` refuses
/// the guard (an idle stop is committed, or the lock call fails), a
/// connection that needs one is closed with the reason `remote server is
/// stopping`, before its key is noted. The guard is held until the
/// connection's serving ends.
async fn serve_accepted(
    accepted: Accepted,
    classify: Arc<PairingAuthorizer>,
    ops: Option<Arc<OpsServing>>,
    wire: Option<Arc<WirePlane>>,
    rcp: Option<Arc<RcpPlane>>,
    fence: Option<crate::serving_hold::AdmissionFence>,
    pull: Option<Arc<EpochPull>>,
) {
    let (decision, unpaired) = classify
        .classify_accept_with_enrollment_hint(&accepted.alpn, accepted.remote_id.as_bytes());
    tracing::info!(
        remote_id = %accepted.remote_id,
        alpn = %String::from_utf8_lossy(&accepted.alpn),
        ?decision,
        "cerulion_remoted: accept classified"
    );
    let _hold = match &fence {
        Some(fence) if fences_idle_teardown(&decision) => match fence.admit() {
            Some(hold) => Some(hold),
            None => {
                accepted
                    .connection
                    .close(0u32.into(), b"remote server is stopping");
                return;
            }
        },
        _ => None,
    };
    if let Some(pull) = &pull {
        pull.note_accept(accepted.remote_id.as_bytes());
    }

    // ── The ops-ALPN dispatch arm ───────────────────────────
    // Ops plane → the real cerud ops server (per-verb authz is the enforcement).
    if matches!(
        decision,
        AcceptDecision::OpsAdmit | AcceptDecision::OpsBootstrapOnly
    ) {
        if let Some(ops) = ops {
            ops.serve_session(accepted).await;
            return;
        }
        // No ops-serving context wired (accept-classification test harness) → fall
        // through to the skeleton decision frame so the classification stays
        // observable.
    }

    // ── The wire-ALPN dispatch arm ──────────────────────────
    if matches!(decision, AcceptDecision::WireAdmit) {
        if let Some(plane) = wire {
            crate::wire::serve_wire_connection(accepted.connection, plane, classify).await;
            return;
        }
        // WireAdmit but no wire plane wired (ops-only builds + the
        // accept-classification tests): fall through to the skeleton report.
    }

    // ── The rcp-ALPN dispatch arm ───────────────────────────
    if matches!(decision, AcceptDecision::RcpAdmit) {
        if let Some(plane) = rcp {
            let key = *accepted.remote_id.as_bytes();
            crate::rcp::serve_rcp_connection(accepted.connection, key, plane).await;
            return;
        }
    }

    // A refused key on the ops ALPN (a cloud row while the allow-from-cloud
    // switch is OFF): answer in cerud's own framing so an ops client reads the
    // reason as a handshake `Reject`.
    if accepted.alpn == cerulion_link::alpn::OPS {
        if let AcceptDecision::Refuse { reason } = &decision {
            match tokio::time::timeout(
                HANDSHAKE_DEADLINE,
                report_ops_refusal(&accepted.connection, reason),
            )
            .await
            {
                Ok(Ok(())) => {}
                Ok(Err(e)) => {
                    tracing::debug!(error = %e, "cerulion_remoted: ops refusal ended early");
                }
                Err(_elapsed) => {
                    tracing::warn!(
                        remote_id = %accepted.remote_id,
                        deadline_secs = HANDSHAKE_DEADLINE.as_secs(),
                        "cerulion_remoted: ops refusal exceeded its deadline; dropping the connection"
                    );
                }
            }
            return;
        }
    }

    // Everything else (Refuse / UnknownAlpn, and the no-plane fallbacks): the
    // skeleton control-frame stub. Bound the whole handshake: a peer that completed
    // the QUIC handshake then stalls must not hold this task + connection forever.
    match tokio::time::timeout(
        HANDSHAKE_DEADLINE,
        report_decision(&accepted.connection, &decision, unpaired),
    )
    .await
    {
        Ok(Ok(())) => {}
        Ok(Err(e)) => {
            tracing::debug!(error = %e, "cerulion_remoted: accept handshake ended early");
        }
        Err(_elapsed) => {
            // On timeout the future is dropped and `accepted` (owning the
            // connection) drops at the end of this fn → the connection closes.
            tracing::warn!(
                remote_id = %accepted.remote_id,
                deadline_secs = HANDSHAKE_DEADLINE.as_secs(),
                "cerulion_remoted: control-frame handshake exceeded its deadline; dropping the \
                 connection (a peer that completes the QUIC handshake then stalls must not hold \
                 the task/connection indefinitely)"
            );
        }
    }
}

/// The skeleton control-frame handshake: the peer opens a control stream and
/// sends a hello; the daemon replies with the [`AcceptDecision`] JSON, then
/// drains the peer's stream to EOF so the reply is delivered before the
/// connection drops. (The ops and wire ALPNs speak their own protocols instead;
/// a wire `Refuse` there closes the connection serving nothing.)
async fn report_decision(
    connection: &Connection,
    decision: &AcceptDecision,
    unpaired: bool,
) -> Result<(), LinkError> {
    let (mut send, mut recv) = accept_frame_stream(connection).await?;
    // Drain the peer's hello (its presence is what fired accept_bi).
    let _ = read_frame(&mut recv, MAX_HELLO_LEN).await?;
    // The decision is small structured JSON; serialization cannot fail for it.
    let bytes = admission_payload(decision, unpaired);
    write_frame(&mut send, &bytes).await?;
    let _ = send.finish();
    // Hold the connection until the peer finishes (EOF) so the reply is delivered.
    let _ = read_frame(&mut recv, MAX_HELLO_LEN).await;
    Ok(())
}

/// The ops-plane refusal: cerud's framing (a `u32` big-endian length, then
/// JSON). Reads the client's opening frame (bounded by [`MAX_HELLO_LEN`]),
/// writes one `HandshakeReply::Reject` carrying `reason`, then holds the
/// connection until the peer closes its stream so the reply is delivered. The
/// caller bounds the whole exchange with the handshake deadline.
async fn report_ops_refusal(connection: &Connection, reason: &str) -> Result<(), LinkError> {
    let stream = |e| LinkError::Stream(Box::new(e));
    let (mut send, mut recv) = accept_frame_stream(connection).await?;
    let mut len_buf = [0u8; 4];
    recv.read_exact(&mut len_buf)
        .await
        .map_err(|e| LinkError::Stream(Box::new(e)))?;
    let len = u32::from_be_bytes(len_buf) as usize;
    if len > MAX_HELLO_LEN {
        return Err(LinkError::FrameExceedsMaxLen {
            len,
            max: MAX_HELLO_LEN,
        });
    }
    let mut hello = vec![0u8; len];
    recv.read_exact(&mut hello)
        .await
        .map_err(|e| LinkError::Stream(Box::new(e)))?;
    let reply = cerud::protocol::HandshakeReply::Reject {
        server_min: cerud::protocol::MIN_PROTOCOL_VERSION,
        server_max: cerud::protocol::PROTOCOL_VERSION,
        message: reason.to_string(),
    };
    let payload =
        serde_json::to_vec(&reply).expect("a handshake reply contains only JSON-compatible fields");
    let payload_len = u32::try_from(payload.len()).expect("a refusal reply is far below 4 GiB");
    send.write_all(&payload_len.to_be_bytes())
        .await
        .map_err(stream)?;
    send.write_all(&payload).await.map_err(stream)?;
    let _ = send.finish();
    let mut sink = [0u8; 256];
    while let Ok(Some(_)) = recv.read(&mut sink).await {}
    Ok(())
}

fn admission_payload(decision: &AcceptDecision, unpaired: bool) -> Vec<u8> {
    #[derive(serde::Serialize)]
    struct Reply<'a> {
        #[serde(flatten)]
        decision: &'a AcceptDecision,
        #[serde(skip_serializing_if = "Option::is_none")]
        admission_code: Option<&'static str>,
    }
    let admission_code =
        (unpaired && matches!(decision, AcceptDecision::Refuse { .. })).then_some("unpaired");
    serde_json::to_vec(&Reply {
        decision,
        admission_code,
    })
    .expect("admission reply contains only JSON-compatible fields")
}

#[cfg(test)]
mod admission_reply_tests {
    use super::*;

    #[test]
    fn only_an_explicit_unpaired_refusal_carries_the_retry_code() {
        let refused = AcceptDecision::Refuse {
            reason: "access refused".into(),
        };
        assert_eq!(
            admission_payload(&refused, true),
            br#"{"decision":"refuse","reason":"access refused","admission_code":"unpaired"}"#,
        );
        for decision in [
            refused,
            AcceptDecision::WireAdmit,
            AcceptDecision::OpsBootstrapOnly,
        ] {
            let reply = admission_payload(&decision, false);
            let value: serde_json::Value = serde_json::from_slice(&reply).unwrap();
            assert!(value.get("admission_code").is_none());
        }
        let admitted = admission_payload(&AcceptDecision::WireAdmit, true);
        assert_eq!(admitted, br#"{"decision":"wire_admit"}"#);
    }
}

/// Load the 32-byte ed25519 device secret (raw bytes). An absent / wrong-size
/// key is a provisioning gap, never fabricated. On Unix, when `path` itself is a
/// regular file with a single link, its group and other permission bits are
/// cleared before it is read; a symbolic link at `path` is read through
/// unchanged.
pub(crate) fn load_device_key(path: &Path) -> Result<[u8; 32], RemotedError> {
    let bytes = crate::private_file::read_private(path, "device key").map_err(|e| {
        RemotedError::DeviceKey(format!(
            "{}: {e} (provision the robot's 32-byte device key)",
            path.display()
        ))
    })?;
    let len = bytes.len();
    bytes.try_into().map_err(|_| {
        RemotedError::DeviceKey(format!(
            "{} must be exactly 32 raw bytes, got {len}",
            path.display()
        ))
    })
}

/// Load the trust-store MAC key. **Skeleton seam** — production reads this from
/// firmware secure storage (`cerulion_pairing` holds no key-management policy).
///
/// `expected` is `true` when a trust store or MAC'd device index already exists on
/// disk, so the key is REQUIRED to verify them. Classification:
///
/// - **`expected` + any read error** (missing OR unreadable) → a LOUD
///   provisioning-class [`RemotedError::MacKey`]: the secret that authenticates
///   this robot's existing trust state is gone (a provisioning gap / potential
///   tamper), never a generic `Config` error.
/// - **not `expected` + genuinely absent (`NotFound`)** → benign. Nothing on disk
///   needs the key; an empty placeholder is returned. It is NEVER used to verify
///   anything — the absent trust store fails its own read before decode (surfacing
///   a `Store` provisioning gap), and any TOCTOU appearance of a store/index fails
///   the MAC closed. This is the genuinely-fresh-robot path.
/// - **not `expected` + present-but-unreadable** → still a LOUD
///   [`RemotedError::MacKey`]: a broken secure-storage secret is never silently
///   swallowed, even when no artifact currently requires it.
///
/// A present but EMPTY file loads as an empty key when not `expected` (a fresh
/// install's placeholder). When `expected` it is refused here with
/// [`RemotedError::MacKey`] naming the file: an empty key verifies nothing and
/// cannot persist a mutation. [`crate::RemotePlane::bind`] repeats the refusal
/// for a [`crate::KeyProvider`] that has no file to name.
///
/// On Unix, when `path` itself is a regular file with a single link, its group
/// and other permission bits are cleared before it is read; a symbolic link at
/// `path` is read through unchanged.
pub(crate) fn load_mac_key(path: &Path, expected: bool) -> Result<Vec<u8>, RemotedError> {
    match crate::private_file::read_private(path, "trust-store MAC key") {
        Ok(bytes) if bytes.is_empty() && expected => Err(RemotedError::MacKey(format!(
            "empty trust-store MAC key file {}: a trust store or device index exists and \
             must be verified with it. An empty key file is only a fresh install's \
             placeholder; restore this device's own MAC key into it (mode 0600)",
            path.display()
        ))),
        Ok(bytes) => Ok(bytes),
        // Genuinely-fresh robot: nothing on disk needs this key and it is simply
        // absent. Defer to the absent store's own `Store` provisioning gap.
        Err(e) if !expected && e.kind() == std::io::ErrorKind::NotFound => Ok(Vec::new()),
        // Required by a present store/index but missing or unreadable — the
        // headline tamper/provisioning case.
        Err(e) if expected => Err(RemotedError::MacKey(format!(
            "trust-store MAC key {}: {e} — a trust store and/or MAC'd device index is \
             present but the key that authenticates them is missing or unreadable. The \
             secure-storage secret that verifies this robot's trust state is gone \
             (provisioning gap / potential tamper); refusing to serve. Re-provision the \
             MAC key for THIS device (skeleton seam — production reads it from firmware \
             secure storage)",
            path.display()
        ))),
        // Not required by any on-disk artifact, but present-and-unreadable (e.g.
        // permission denied). Never silently swallowed — still provisioning-class.
        Err(e) => Err(RemotedError::MacKey(format!(
            "trust-store MAC key {}: {e} — the key file is present but could not be read \
             (a broken secure-storage secret; skeleton seam — production reads it from \
             firmware secure storage)",
            path.display()
        ))),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Only an ops or rcp session admitted with paired authority fences idle
    /// teardown; every other decision, the bootstrap surface included, holds
    /// nothing. Each row is (decision, fences).
    #[test]
    fn only_paired_ops_and_rcp_admissions_fence_idle_teardown() {
        let rows = [
            (AcceptDecision::OpsAdmit, true),
            (AcceptDecision::RcpAdmit, true),
            (AcceptDecision::OpsBootstrapOnly, false),
            (AcceptDecision::WireAdmit, false),
            (
                AcceptDecision::Refuse {
                    reason: "rcp plane refused: unpaired device key".into(),
                },
                false,
            ),
            (
                AcceptDecision::UnknownAlpn {
                    alpn: b"cerulion/unknown/1".to_vec(),
                },
                false,
            ),
        ];
        for (decision, fences) in rows {
            assert_eq!(fences_idle_teardown(&decision), fences, "{decision:?}");
        }
    }

    /// `cerulion_netd`'s `automatic_serving_test.rs` copies this deadline by
    /// hand as `REMOTED_HANDSHAKE_DEADLINE` and asserts the owned server is
    /// reaped before it passes. That shows a refused rcp connection does not
    /// fence idle teardown only while remoted keeps such a connection this
    /// long, so the two values change together.
    #[test]
    fn the_handshake_deadline_is_ten_seconds() {
        assert_eq!(HANDSHAKE_DEADLINE, Duration::from_secs(10));
    }

    #[test]
    fn handshake_admission_reserves_capacity_at_every_boundary() {
        for (pending, from_source, validated, paired, expected) in [
            (0, 0, false, false, Admission::Admit),
            (15, 0, false, false, Admission::Admit),
            (16, 0, false, false, Admission::Retry),
            (17, 0, false, false, Admission::Retry),
            (23, 4, false, false, Admission::Retry),
            (24, 0, false, false, Admission::Refuse(Cap::UnpairedShare)),
            (31, 4, false, false, Admission::Refuse(Cap::UnpairedShare)),
            (32, 0, false, false, Admission::Refuse(Cap::Global)),
            (33, 0, false, false, Admission::Refuse(Cap::Global)),
            (15, 4, false, false, Admission::Admit),
            (0, 3, true, false, Admission::Admit),
            (0, 4, true, false, Admission::Refuse(Cap::Ip)),
            (0, 5, true, false, Admission::Refuse(Cap::Ip)),
            (16, 3, true, false, Admission::Admit),
            (23, 3, true, false, Admission::Admit),
            (24, 0, true, false, Admission::Refuse(Cap::UnpairedShare)),
            (31, 3, true, false, Admission::Refuse(Cap::UnpairedShare)),
            (32, 0, true, false, Admission::Refuse(Cap::Global)),
            (33, 0, true, false, Admission::Refuse(Cap::Global)),
            (32, 4, true, false, Admission::Refuse(Cap::Global)),
            (0, 0, false, true, Admission::KeyRetry),
            (15, 0, false, true, Admission::KeyRetry),
            (16, 0, false, true, Admission::KeyRetry),
            (24, 0, false, true, Admission::KeyRetry),
            (31, 0, false, true, Admission::KeyRetry),
            (0, 0, true, true, Admission::Admit),
            (15, 0, true, true, Admission::Admit),
            (23, 0, true, true, Admission::Admit),
            (24, 0, true, true, Admission::Admit),
            (31, 0, true, true, Admission::Admit),
            (32, 0, true, true, Admission::Refuse(Cap::Global)),
            (32, 0, false, true, Admission::Refuse(Cap::Global)),
            (33, 0, true, true, Admission::Refuse(Cap::Global)),
        ] {
            assert_eq!(
                handshake_admission(pending, from_source, validated, paired),
                expected,
                "pending {pending}, from source {from_source}, validated {validated}, paired {paired}"
            );
        }
    }

    #[test]
    fn validated_sources_cannot_claim_another_ips_bucket() {
        let address = "127.0.0.1:1234".parse().unwrap();
        assert_eq!(
            handshake_source(IncomingAddr::Ip(address), false, |_| false),
            None
        );
        let ip = handshake_source(IncomingAddr::Ip(address), true, |_| false).unwrap();
        assert_eq!(ip, SourceKey::Ip("127.0.0.1".parse().unwrap()));
        let relay = SourceKey::UnpairedRelay("https://relay.example.invalid".parse().unwrap());
        for (source, bound, cap) in [
            (&relay, 8, Cap::UnpairedRelay),
            (&SourceKey::Custom, 4, Cap::Custom),
            (&ip, 4, Cap::Ip),
        ] {
            assert_eq!(
                source_admission(15, bound - 1, true, Some(source)),
                Admission::Admit
            );
            assert_eq!(
                source_admission(15, bound, true, Some(source)),
                Admission::Refuse(cap)
            );
            assert_eq!(
                source_admission(15, bound + 1, true, Some(source)),
                Admission::Refuse(cap)
            );
        }
        assert_eq!(source_admission(15, 100, false, None), Admission::Admit);
        assert_eq!(source_admission(16, 0, false, None), Admission::Retry);
        // The shared caps and Retry precede every per-source bucket.
        for (pending, from_source, validated, source, expected) in [
            (16, 8, false, &relay, Admission::Retry),
            (24, 8, true, &relay, Admission::Refuse(Cap::UnpairedShare)),
            (16, 4, true, &ip, Admission::Refuse(Cap::Ip)),
            (23, 4, true, &ip, Admission::Refuse(Cap::Ip)),
            (24, 4, true, &ip, Admission::Refuse(Cap::UnpairedShare)),
            (32, 4, true, &ip, Admission::Refuse(Cap::Global)),
            (
                24,
                4,
                true,
                &SourceKey::Custom,
                Admission::Refuse(Cap::UnpairedShare),
            ),
        ] {
            assert_eq!(
                source_admission(pending, from_source, validated, Some(source)),
                expected,
                "pending {pending}, from source {from_source}, validated {validated}, {source:?}"
            );
        }
    }

    #[test]
    fn paired_relay_keys_are_independent_of_the_unpaired_url_bucket() {
        let url: RelayUrl = "http://127.0.0.1:1234".parse().unwrap();
        let key = iroh::SecretKey::from_bytes(&[7; 32]).public();
        let address = IncomingAddr::Relay {
            url: url.clone(),
            endpoint_id: key,
        };
        let paired = handshake_source(address.clone(), false, |id| *id == key).unwrap();
        assert_eq!(paired, SourceKey::PairedRelay(key));
        let unpaired = handshake_source(address, false, |_| false).unwrap();
        assert_eq!(unpaired, SourceKey::UnpairedRelay(url));
        // Eight stranger handshakes fill the URL bucket: the next stranger is
        // refused, and the paired key on the same relay still reads zero.
        let mut sources = indexmap::IndexMap::new();
        sources.insert(unpaired.clone(), 8u8);
        assert_eq!(pending_from(&sources, Some(&unpaired)), 8);
        assert_eq!(pending_from(&sources, Some(&paired)), 0);
        assert_eq!(pending_from(&sources, None), 0);
        assert_eq!(
            source_admission(
                8,
                pending_from(&sources, Some(&unpaired)),
                false,
                Some(&unpaired)
            ),
            Admission::Refuse(Cap::UnpairedRelay)
        );
        assert_eq!(
            source_admission(
                8,
                pending_from(&sources, Some(&paired)),
                true,
                Some(&paired)
            ),
            Admission::Admit
        );
        // The reserve: at 24 pending every other source is refused, while a
        // paired key is admitted until the global cap.
        let ip = SourceKey::Ip("127.0.0.1".parse().unwrap());
        for source in [&unpaired, &ip, &SourceKey::Custom] {
            assert_eq!(
                source_admission(24, 0, true, Some(source)),
                Admission::Refuse(Cap::UnpairedShare)
            );
            assert_eq!(
                source_admission(32, 0, true, Some(source)),
                Admission::Refuse(Cap::Global)
            );
        }
        assert_eq!(
            source_admission(24, 0, false, None),
            Admission::Refuse(Cap::UnpairedShare)
        );
        assert_eq!(
            source_admission(24, 0, true, Some(&paired)),
            Admission::Admit
        );
        assert_eq!(
            source_admission(31, 3, true, Some(&paired)),
            Admission::Admit
        );
        assert_eq!(
            source_admission(31, 4, true, Some(&paired)),
            Admission::Refuse(Cap::PairedRelay)
        );
        for (source, count, validated, expected) in [
            (&unpaired, 7, false, Admission::Admit),
            (&unpaired, 8, false, Admission::Refuse(Cap::UnpairedRelay)),
            (&unpaired, 9, false, Admission::Refuse(Cap::UnpairedRelay)),
            (&paired, 0, true, Admission::Admit),
            (&paired, 3, true, Admission::Admit),
            (&paired, 4, true, Admission::Refuse(Cap::PairedRelay)),
            (&paired, 5, true, Admission::Refuse(Cap::PairedRelay)),
            (&paired, 0, false, Admission::KeyRetry),
            (&paired, 4, false, Admission::KeyRetry),
        ] {
            assert_eq!(
                source_admission(8, count, validated, Some(source)),
                expected,
                "{source:?} at {count}, validated {validated}"
            );
        }
        assert_eq!(
            source_admission(16, 0, false, Some(&paired)),
            Admission::KeyRetry
        );
        assert_eq!(
            source_admission(16, 4, false, Some(&paired)),
            Admission::KeyRetry
        );
        assert_eq!(
            source_admission(32, 4, true, Some(&paired)),
            Admission::Refuse(Cap::Global)
        );
        assert_eq!(
            source_admission(16, 0, true, Some(&paired)),
            Admission::Admit
        );
        assert_eq!(
            source_admission(32, 0, true, Some(&paired)),
            Admission::Refuse(Cap::Global)
        );
    }

    /// An IP datagram whose source is a relay-mapped address reads as an
    /// unvalidated relay arrival under the key that address maps to. However many such
    /// arrivals come, at any pending count and any bucket count, none takes a
    /// slot: each gets a Retry that only the key's holder receives.
    #[test]
    fn a_forged_relay_source_never_takes_a_paired_key_slot() {
        let key = iroh::SecretKey::from_bytes(&[7; 32]).public();
        let forged = IncomingAddr::Relay {
            url: "https://relay.example.invalid".parse().unwrap(),
            endpoint_id: key,
        };
        let source = handshake_source(forged, false, |id| *id == key).unwrap();
        assert_eq!(source, SourceKey::PairedRelay(key));
        for pending in 0..=33 {
            for from_source in 0..=5 {
                let expected = if pending >= 32 {
                    Admission::Refuse(Cap::Global)
                } else {
                    Admission::KeyRetry
                };
                assert_eq!(
                    source_admission(pending, from_source, false, Some(&source)),
                    expected,
                    "pending {pending}, from source {from_source}"
                );
            }
        }
        // The same key, validated by the returned token, takes its bucket.
        assert_eq!(
            source_admission(0, 0, true, Some(&source)),
            Admission::Admit
        );
        assert_eq!(
            source_admission(0, 3, true, Some(&source)),
            Admission::Admit
        );
        assert_eq!(
            source_admission(0, 4, true, Some(&source)),
            Admission::Refuse(Cap::PairedRelay)
        );
    }

    #[test]
    fn an_admission_closes_only_the_regimes_it_ends() {
        let relay = SourceKey::UnpairedRelay("https://relay.example.invalid".parse().unwrap());
        let other_relay =
            SourceKey::UnpairedRelay("https://other.example.invalid".parse().unwrap());
        let paired = SourceKey::PairedRelay(iroh::SecretKey::from_bytes(&[7; 32]).public());
        let other_paired = SourceKey::PairedRelay(iroh::SecretKey::from_bytes(&[8; 32]).public());
        let ip = SourceKey::Ip("127.0.0.1".parse().unwrap());
        let other_ip = SourceKey::Ip("127.0.0.2".parse().unwrap());
        let both = [Some(Cap::Global), Some(Cap::UnpairedShare)];
        type Held<'a> = &'a [(&'a SourceKey, u8)];
        type Row<'a> = (usize, Option<&'a SourceKey>, Held<'a>, [Option<Cap>; 3]);
        let none: Held = &[];
        let rows: &[Row] = &[
            (0, None, none, [both[0], both[1], None]),
            (15, None, none, [both[0], both[1], None]),
            (16, None, none, [None, None, None]),
            (
                15,
                Some(&relay),
                &[(&relay, 3)],
                [both[0], both[1], Some(Cap::UnpairedRelay)],
            ),
            (15, Some(&relay), &[(&relay, 4)], [both[0], both[1], None]),
            (20, Some(&relay), &[(&relay, 7)], [None, None, None]),
            (
                20,
                Some(&relay),
                none,
                [None, None, Some(Cap::UnpairedRelay)],
            ),
            (20, Some(&relay), &[(&other_relay, 4)], [None, None, None]),
            (
                20,
                Some(&relay),
                &[(&other_relay, 3)],
                [None, None, Some(Cap::UnpairedRelay)],
            ),
            (
                20,
                Some(&paired),
                &[(&paired, 1)],
                [None, None, Some(Cap::PairedRelay)],
            ),
            (20, Some(&paired), &[(&paired, 2)], [None, None, None]),
            (20, Some(&paired), &[(&other_paired, 2)], [None, None, None]),
            (
                20,
                Some(&paired),
                &[(&other_paired, 1)],
                [None, None, Some(Cap::PairedRelay)],
            ),
            (0, Some(&ip), &[(&ip, 1)], [both[0], both[1], Some(Cap::Ip)]),
            (0, Some(&ip), &[(&ip, 2)], [both[0], both[1], None]),
            (0, Some(&ip), &[(&ip, 3)], [both[0], both[1], None]),
            // Another IP still holding a full bucket keeps the IP regime open.
            (0, Some(&ip), &[(&other_ip, 4)], [both[0], both[1], None]),
            (
                0,
                Some(&ip),
                &[(&other_ip, 1)],
                [both[0], both[1], Some(Cap::Ip)],
            ),
            // Other classes never hold a class regime open.
            (
                0,
                Some(&ip),
                &[(&relay, 8), (&paired, 4)],
                [both[0], both[1], Some(Cap::Ip)],
            ),
            (
                0,
                Some(&SourceKey::Custom),
                &[(&SourceKey::Custom, 1)],
                [both[0], both[1], Some(Cap::Custom)],
            ),
            (
                0,
                Some(&SourceKey::Custom),
                &[(&SourceKey::Custom, 2)],
                [both[0], both[1], None],
            ),
        ];
        for (pending, source, held, expected) in rows {
            let sources: indexmap::IndexMap<SourceKey, u8> = held
                .iter()
                .map(|(key, count)| ((*key).clone(), *count))
                .collect();
            assert_eq!(
                closed_regimes(*pending, *source, &sources),
                *expected,
                "pending {pending}, source {source:?}, held {held:?}"
            );
        }
        assert!(shared_regimes_close(15));
        assert!(!shared_regimes_close(16));
    }

    #[test]
    fn an_admitted_source_only_recovers_its_own_cap_regime() {
        use cerulion_core::transport::failure_regime_latch::RegimeDecision;
        let mut latches = AdmissionLatches::default();
        // An admission with no open regime recovers nothing.
        assert_eq!(latches.admitted(Cap::Global), None);
        assert_eq!(latches.admitted(Cap::UnpairedShare), None);
        for (cap, name) in [
            (Cap::Global, "global"),
            (Cap::UnpairedShare, "unpaired_share"),
            (Cap::Ip, "ip"),
            (Cap::UnpairedRelay, "unpaired_relay"),
            (Cap::PairedRelay, "paired_relay"),
            (Cap::Custom, "custom"),
        ] {
            assert_eq!(cap.as_str(), name);
            assert_eq!(latches.refused(cap), RegimeDecision::Loud);
            assert_eq!(
                latches.refused(cap),
                RegimeDecision::Suppressed { suppressed: 1 }
            );
        }
        assert_eq!(latches.admitted(Cap::PairedRelay), Some(1));
        assert_eq!(
            latches.refused(Cap::UnpairedRelay),
            RegimeDecision::Suppressed { suppressed: 2 }
        );
        assert_eq!(
            latches.refused(Cap::Ip),
            RegimeDecision::Suppressed { suppressed: 2 }
        );
        assert_eq!(
            latches.refused(Cap::Global),
            RegimeDecision::Suppressed { suppressed: 2 }
        );
        assert_eq!(
            latches.refused(Cap::UnpairedShare),
            RegimeDecision::Suppressed { suppressed: 2 }
        );
        assert_eq!(latches.admitted(Cap::UnpairedRelay), Some(2));
        assert_eq!(latches.refused(Cap::UnpairedRelay), RegimeDecision::Loud);
        assert_eq!(latches.refused(Cap::PairedRelay), RegimeDecision::Loud);
        assert_eq!(latches.admitted(Cap::Global), Some(2));
        assert_eq!(latches.admitted(Cap::Custom), Some(1));
        for (cap, total) in [
            (Cap::Global, 3),
            (Cap::UnpairedShare, 3),
            (Cap::Ip, 3),
            (Cap::UnpairedRelay, 4),
            (Cap::PairedRelay, 3),
            (Cap::Custom, 2),
        ] {
            assert_eq!(latches.total(cap), total, "{cap:?}");
        }
    }

    #[tokio::test]
    async fn completed_and_failed_handshakes_release_source_counts() {
        let source = SourceKey::Custom;
        let mut tasks = tokio::task::JoinSet::new();
        let mut ids = indexmap::IndexMap::new();
        let mut counts = indexmap::IndexMap::new();
        let success = tasks.spawn(async { 1 });
        let failure = tasks.spawn(async { panic!("handshake task failed") });
        let pending = tasks.spawn(std::future::pending::<i32>());
        for task in [&success, &failure, &pending] {
            ids.insert(task.id(), source.clone());
        }
        counts.insert(source.clone(), 3);
        pending.abort();
        let mut successes = 0;
        let mut failures = 0;
        while let Some(result) = tasks.join_next_with_id().await {
            let id = match result {
                Ok((id, 1)) => {
                    successes += 1;
                    id
                }
                Ok(_) => panic!("unexpected handshake value"),
                Err(error) => {
                    failures += 1;
                    error.id()
                }
            };
            release_handshake_source(id, &mut ids, &mut counts);
            assert_eq!(ids.len(), tasks.len());
            assert_eq!(
                counts.values().map(|n| usize::from(*n)).sum::<usize>(),
                tasks.len()
            );
        }
        assert_eq!((successes, failures), (1, 2));
        assert!(ids.is_empty() && counts.is_empty());
    }

    #[test]
    fn matching_store_device_key_is_accepted() {
        // The endpoint id equals the store's robot_transport_key → the store
        // belongs to this device.
        let key = [42u8; 32];
        assert!(check_store_device_binding(&key, &PublicKey(key)).is_ok());
    }

    #[test]
    fn mismatched_store_device_key_is_refused_loudly() {
        // A store recorded against robot B (transport key [7;32]) loaded on robot
        // A (endpoint id [42;32]) → refuse, naming both keys + the fix.
        let endpoint_key = [42u8; 32];
        let store_key = PublicKey([7u8; 32]);
        let err = check_store_device_binding(&endpoint_key, &store_key).unwrap_err();
        assert!(matches!(err, RemotedError::ProvisioningMismatch(_)));
        let msg = err.to_string();
        assert!(msg.contains("does not match"), "msg: {msg}");
        assert!(
            msg.contains(&hex::encode(endpoint_key)),
            "msg names endpoint id: {msg}"
        );
        assert!(
            msg.contains(&hex::encode(store_key.0)),
            "msg names store key: {msg}"
        );
        assert!(msg.contains("re-provision"), "msg states the fix: {msg}");
    }

    #[test]
    fn a_single_byte_difference_is_caught() {
        // Anti-tautology: an all-equal-but-one-byte key is still refused (not a
        // prefix/length false-accept).
        let mut store = [3u8; 32];
        let endpoint = store;
        store[31] ^= 0x01;
        assert!(check_store_device_binding(&endpoint, &PublicKey(store)).is_err());
    }

    #[test]
    fn required_missing_mac_key_is_provisioning_class_not_config() {
        // A store/index EXISTS (expected == true) but the MAC key file is absent →
        // a LOUD provisioning-class MacKey error, NOT a generic Config.
        // Reverting the classification to RemotedError::Config
        // fails this exact assertion.
        let dir = tempfile::tempdir().unwrap();
        let missing = dir.path().join("trust_store.mac_key"); // never created
        let err = load_mac_key(&missing, /*expected=*/ true).unwrap_err();
        assert!(
            matches!(err, RemotedError::MacKey(_)),
            "expected MacKey (provisioning-class), got {err:?}"
        );
        assert!(
            !matches!(err, RemotedError::Config(_)),
            "a required-but-missing MAC key must NOT be a config error: {err:?}"
        );
        let msg = err.to_string();
        assert!(msg.contains("provisioning gap"), "msg: {msg}");
        assert!(msg.contains("tamper"), "msg names the tamper case: {msg}");
    }

    #[test]
    fn unrequired_absent_mac_key_is_a_benign_empty_placeholder() {
        // Genuinely-fresh robot: no store, no index (expected == false) and the
        // MAC key is simply absent → Ok(empty), deferring to the absent store's
        // own Store provisioning gap downstream. NOT an error at this seam.
        let dir = tempfile::tempdir().unwrap();
        let missing = dir.path().join("trust_store.mac_key"); // never created
        let key = load_mac_key(&missing, /*expected=*/ false)
            .expect("an unrequired absent MAC key is benign, not an error");
        assert!(key.is_empty(), "the placeholder must be empty, got {key:?}");
    }

    #[test]
    fn empty_mac_key_file_is_refused_by_name_only_when_required() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("trust_store.mac_key");
        std::fs::write(&path, b"").unwrap();
        let err = load_mac_key(&path, /*expected=*/ true).unwrap_err();
        assert!(matches!(err, RemotedError::MacKey(_)), "got {err:?}");
        assert!(
            err.to_string().contains(&path.display().to_string()),
            "the error names the key file: {err}"
        );
        // A fresh install's empty placeholder stays benign.
        let key = load_mac_key(&path, /*expected=*/ false).unwrap();
        assert!(key.is_empty());
    }

    #[test]
    fn a_present_mac_key_loads_its_bytes_regardless_of_requirement() {
        // Anti-tautology control: a present, readable key loads its exact bytes on
        // BOTH the required and not-required paths (the apparatus moves; the error
        // arms above are not vacuous).
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("trust_store.mac_key");
        std::fs::write(&path, b"present-mac-key").unwrap();
        for expected in [true, false] {
            let key = load_mac_key(&path, expected).unwrap();
            assert_eq!(key, b"present-mac-key", "expected={expected}");
        }
    }

    #[cfg(unix)]
    fn mode_of(path: &Path) -> u32 {
        use std::os::unix::fs::PermissionsExt as _;
        std::fs::metadata(path).unwrap().permissions().mode() & 0o7777
    }

    #[cfg(unix)]
    #[test]
    fn a_0644_device_key_is_loaded_and_narrowed_to_0600() {
        use std::os::unix::fs::PermissionsExt as _;
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("device_key");
        std::fs::write(&path, [7u8; 32]).unwrap();
        std::fs::set_permissions(&path, std::fs::Permissions::from_mode(0o644)).unwrap();
        assert_eq!(load_device_key(&path).unwrap(), [7u8; 32]);
        assert_eq!(mode_of(&path), 0o600);
    }

    #[cfg(unix)]
    #[test]
    fn a_symlinked_device_key_is_loaded_without_changing_its_target() {
        use std::os::unix::fs::PermissionsExt as _;
        let dir = tempfile::tempdir().unwrap();
        let target = dir.path().join("elsewhere");
        std::fs::write(&target, [9u8; 32]).unwrap();
        std::fs::set_permissions(&target, std::fs::Permissions::from_mode(0o644)).unwrap();
        let link = dir.path().join("device_key");
        std::os::unix::fs::symlink(&target, &link).unwrap();
        assert_eq!(load_device_key(&link).unwrap(), [9u8; 32]);
        assert_eq!(mode_of(&target), 0o644, "the link's target keeps its mode");
    }

    #[cfg(unix)]
    #[test]
    fn a_0644_mac_key_is_loaded_and_narrowed_to_0600() {
        use std::os::unix::fs::PermissionsExt as _;
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("trust_store.mac_key");
        std::fs::write(&path, b"a-mac-key").unwrap();
        std::fs::set_permissions(&path, std::fs::Permissions::from_mode(0o644)).unwrap();
        assert_eq!(load_mac_key(&path, true).unwrap(), b"a-mac-key");
        assert_eq!(mode_of(&path), 0o600);
    }

    #[cfg(unix)]
    #[test]
    fn a_symlinked_mac_key_is_loaded_without_changing_its_target() {
        use std::os::unix::fs::PermissionsExt as _;
        let dir = tempfile::tempdir().unwrap();
        let target = dir.path().join("elsewhere");
        std::fs::write(&target, b"linked-mac-key").unwrap();
        std::fs::set_permissions(&target, std::fs::Permissions::from_mode(0o644)).unwrap();
        let link = dir.path().join("trust_store.mac_key");
        std::os::unix::fs::symlink(&target, &link).unwrap();
        assert_eq!(load_mac_key(&link, true).unwrap(), b"linked-mac-key");
        assert_eq!(mode_of(&target), 0o644, "the link's target keeps its mode");
    }
}
