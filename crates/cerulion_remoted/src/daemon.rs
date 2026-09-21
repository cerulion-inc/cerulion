// SPDX-License-Identifier: AGPL-3.0-only
//! The daemon: load the device key + trust store + side-map, build the ONE iroh
//! endpoint, and run the accept loop dispatching on the negotiated ALPN.
//!
//! Every connection is first classified by the [`PairingAuthorizer`] (deny-by-
//! default), then dispatched by plane:
//! - `cerulion/ops/1` → the real `cerud::OpsServer` over a `QuicOpsStream` adapter
//!   ([`crate::ops`]) — deploy/inventory/pair/claim/e-stop;
//! - `cerulion/wire/1` (WireAdmit) → the demand-driven SHM tap plane
//!   ([`crate::wire`]);
//! - everything else (Refuse / unknown ALPN, or a plane not wired) → a
//!   deadline-bounded skeleton control-frame report that surfaces the
//!   [`AcceptDecision`] so the classification stays observable end-to-end.

use std::path::Path;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use cerud::lease::ControlLease;
use cerulion_core::TransportManager;
use cerulion_link::{
    accept_frame_stream, accept_one, build_endpoint, read_frame, write_frame, Accepted, Connection,
    Endpoint, EndpointConfig, LinkError,
};
use cerulion_pairing::format::PublicKey;
use cerulion_pairing::verify::TrustStore;

use crate::authorizer::{AcceptDecision, PairingAuthorizer};
use crate::clock::RemotedClock;
use crate::config::RemotedConfig;
use crate::device_index::DeviceAccountIndex;
use crate::error::RemotedError;
use crate::ops::OpsServing;
use crate::pairing_verbs::SharedCodePairSessions;
use crate::trust::SharedTrust;
use crate::wire::WirePlane;

/// Cap on the skeleton control-frame handshake (hello + reply are tiny).
const MAX_HELLO_LEN: usize = 4096;

/// Per-connection deadline for the whole skeleton control-frame handshake
/// (accept the bidi stream + read the hello + write the decision + drain to
/// EOF). A peer that completes the QUIC handshake then STALLS must not hold the
/// spawned task + connection indefinitely (an unauthenticated resource-hold DoS
/// on the always-on daemon). On timeout the connection is dropped. 10s is
/// generous for a hello+reply even over a relay; the ops and wire planes' protocols
/// carry their own tighter budgets.
///
/// NOTE: this bounds each connection's HANDSHAKE, not the
/// GLOBAL number of concurrent connections. A global concurrency cap (a bounded
/// semaphore acquired before `spawn`, released on the handler's completion) is a
/// defensible additional guard and is not implemented.
const HANDSHAKE_DEADLINE: Duration = Duration::from_secs(10);

/// The loaded robot state the daemon serves from.
struct RemotedState {
    device_key: [u8; 32],
    trust_store: TrustStore,
    device_index: DeviceAccountIndex,
    /// The firmware secure-storage MAC key that authenticates the trust store AND
    /// the side-map — retained so the pairing verbs can PERSIST a mutation
    /// (claim/pair/code-pair write both artifacts back to disk).
    mac_key: Vec<u8>,
}

/// Build the daemon from config and serve until `shutdown` resolves (or the
/// endpoint closes).
///
/// Honors the `--network off` / `CERULION_NETWORK=off` kill-switch: if
/// engaged, it logs loudly and exits cleanly WITHOUT binding an endpoint.
pub async fn run(
    config: RemotedConfig,
    shutdown: impl std::future::Future<Output = ()>,
) -> Result<(), RemotedError> {
    if config.network_disabled {
        tracing::warn!(
            "cerulion_remoted: network kill-switch engaged (--network off / \
             CERULION_NETWORK=off) — refusing to serve the remote plane; exiting \
             cleanly"
        );
        return Ok(());
    }

    let state = load_state(&config)?;
    let endpoint =
        build_endpoint(EndpointConfig::new(state.device_key).with_relay(config.relay.clone()))
            .await?;

    // Bind the trust store to THIS device: the endpoint id (== the device key's
    // public half, the identity peers TLS-authenticate against) MUST equal the
    // store's recorded `robot_transport_key`. A transplanted / restored / fleet-
    // misprovisioned store would otherwise boot cleanly and serve THIS robot
    // under a DIFFERENT robot's owner + access list. LOUD refusal, never a silent
    // wrong-robot boot.
    if let Err(e) = check_store_device_binding(
        endpoint.id().as_bytes(),
        &state.trust_store.robot_transport_key(),
    ) {
        tracing::error!(error = %e, "cerulion_remoted: refusing to serve — trust store is not bound to this device");
        endpoint.close().await;
        return Err(e);
    }

    // Publish the PUBLIC beacon facts (eid / iroh_port / claimable) for the
    // gateway's mDNS TXT enrichment. Best-effort + additive — a write
    // failure warns, never fails the daemon. Read ownership BEFORE the store moves
    // into the shared trust state; the claim / pair handlers refresh it after a
    // successful claim.
    crate::beacon_facts::write_at_startup(
        &config.beacon_facts_file,
        &endpoint,
        state.trust_store.ownership(),
    );

    // Build the live trust state + the ops/wire planes and serve until shutdown. The
    // access-check clock is the robot's own WALL clock and the wire plane uses its default
    // revocation-sweep cadence; the `serve_endpoint` seam parameterizes both so a
    // daemon-level test can drive a mid-session revoke deterministically through the SAME
    // wiring.
    let result = serve_endpoint(
        &config,
        state.trust_store,
        state.device_index,
        state.mac_key,
        &endpoint,
        RemotedClock::wall(),
        None,
        None,
        shutdown,
    )
    .await;
    endpoint.close().await;
    result
}

/// The serving core `run` delegates to (its endpoint already built + device-bound): build
/// the LIVE shared trust state (accept gate reads it, pairing verbs mutate it — a
/// claim/pair takes effect at the next accept with no restart), the ops plane, and the
/// authorizer-wired wire plane, then serve until `shutdown`.
///
/// `access_clock` is the trusted clock every access check reads (production: the robot
/// wall clock); `sweep_interval` overrides the wire plane's mid-session revocation-sweep
/// cadence (`None` = the production default); `manager` overrides the wire plane's SHM
/// transport (`None` = the production lazy singleton, `Some` = a per-test root). All three
/// are parameters ONLY so a daemon-level test can inject a fixed, advanceable clock + a
/// short sweep + an isolated SHM root and observe a mid-session revoke EVICT through the
/// SAME `.with_demand_authorizer(classify)` wiring `run` uses — `#[doc(hidden)]`, not a
/// public API.
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
    let shared =
        SharedTrust::new_with_clock(trust_store, device_index, mac_key, access_clock.clone());
    let classify = Arc::new(PairingAuthorizer::from_shared(shared.clone()));
    // The wire plane's epoch-sync sink gets the SAME live handle + the SAME
    // trusted clock the accept gate and the access list run on, so a desk-pushed
    // epoch is authoritative everywhere at once.
    let epoch_sink = shared.clone();

    // The ops plane over `cerulion/ops/1`: cerud's mechanical verbs + the pairing
    // bootstrap/safety verbs, gated by the same live trust state and receipted by
    // cerud's hash-chained audit log. Built once; reused (serialized) per session.
    let lease = Arc::new(Mutex::new(ControlLease::with_default_window()));
    let sessions = SharedCodePairSessions::new();
    let ops = Arc::new(OpsServing::new(
        shared,
        lease,
        sessions,
        RemotedClock::wall(),
        &config.receipt_file,
        config.log_root.clone(),
        "cerulion-remoted",
    )?);

    // The production wire plane. Robot identity resolves through
    // `robot_identity_from_env` — the SAME override-aware resolver the LAN gateway uses
    // (`CERULION_ROBOT_IDENTITY` override → else the hostname; by design the hostname is the
    // default robot identity). The transport manager is LAZY — a `remoted` that never
    // serves a wire peer never touches iceoryx2.
    //
    // Install the SAME `PairingAuthorizer` as the wire plane's per-demand
    // authorization gate (this line is the wiring a daemon-level test pins). The serving
    // plane then (a) authorizes each `demand` per topic and (b) sweeps established taps
    // mid-session, so an owner-revoke / grant-expiry that lands AFTER the accept-time
    // admission EVICTS the already-streaming demander — both read the LIVE `SharedTrust`
    // the accept gate reads (no re-plumbing). `classify` is an `Arc<PairingAuthorizer>`; it
    // unsize-coerces to `Arc<dyn DemandAuthorizer>` here.
    let robot = cerulion_core::graph::robot_identity_from_env();
    let base = match manager {
        Some(m) => WirePlane::with_manager(robot, m),
        None => WirePlane::lazy(robot),
    };
    //
    // Install the epoch-sync sink on the SAME plane (this line is the wiring
    // a daemon-level test pins). A desk that carries a newer revocation epoch delivers
    // it on connect; applying it here refuses the revoked party's NEXT accept and lets
    // the sweep above evict its LIVE session — the two halves of "revocation
    // reaches the robot" close on one shared handle.
    let mut wire_plane = base
        .with_demand_authorizer(classify.clone())
        .with_epoch_sink(epoch_sink, access_clock);
    if let Some(interval) = sweep_interval {
        wire_plane = wire_plane.with_revocation_sweep_interval(interval);
    }
    let wire = Arc::new(wire_plane);
    tracing::info!(
        endpoint_id = %endpoint.id(),
        robot = %wire.robot(),
        "cerulion_remoted: listening (ONE endpoint; ALPNs wire+ops; always-on)"
    );

    serve_with_wire(endpoint, classify, Some(ops), Some(wire), shutdown).await
}

/// Assert the loaded trust store belongs to THIS device: the endpoint id (the
/// device key's public half) must equal the store's `robot_transport_key`.
/// A mismatch is a LOUD [`RemotedError::ProvisioningMismatch`] (a transplanted
/// store governing the wrong robot). Pure so it is oracle-testable.
fn check_store_device_binding(
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
/// when `wire` is `Some`, an admitted `cerulion/wire/1` connection is served the
/// demand-driven SHM tap plane ([`crate::wire`]); every other ALPN/decision (and a
/// `None` plane) gets the deadline-bounded skeleton report.
pub async fn serve_with_wire(
    endpoint: &Endpoint,
    classify: Arc<PairingAuthorizer>,
    ops: Option<Arc<OpsServing>>,
    wire: Option<Arc<WirePlane>>,
    shutdown: impl std::future::Future<Output = ()>,
) -> Result<(), RemotedError> {
    tokio::pin!(shutdown);
    loop {
        tokio::select! {
            _ = &mut shutdown => {
                tracing::info!("cerulion_remoted: shutdown signalled; stopping accept loop");
                break;
            }
            accepted = accept_one(endpoint) => {
                match accepted {
                    // The endpoint's accept stream ended → the daemon is closing.
                    // This is the ONLY per-accept signal that terminates the loop.
                    Ok(None) => {
                        tracing::info!("cerulion_remoted: endpoint closed; stopping accept loop");
                        break;
                    }
                    Ok(Some(a)) => {
                        let classify = classify.clone();
                        let ops = ops.clone();
                        let wire = wire.clone();
                        tokio::spawn(async move {
                            handle_accepted_with_wire(a, classify, ops, wire).await;
                        });
                    }
                    // A SINGLE inbound connection failed its handshake (a scanner
                    // sending a bogus ALPN, a TLS failure, a peer aborting
                    // mid-handshake). This is per-connection, NOT the endpoint
                    // closing — an unauthenticated peer must NEVER be able to
                    // terminate the robot's always-on remote plane. Log loudly and
                    // keep serving. `accept_one` returns `Ok(None)` (handled above)
                    // when the endpoint itself is gone, so this arm never hot-loops.
                    Err(e) => {
                        tracing::warn!(
                            error = %e,
                            "cerulion_remoted: an inbound connection failed to complete its \
                             handshake; dropping it and continuing to serve (the accept loop is \
                             NOT terminated by one bad/unauthenticated peer)"
                        );
                        continue;
                    }
                }
            }
        }
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
/// - **Wire plane** (`cerulion/wire/1`, `WireAdmit` — the accept gate already
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
    let decision = classify.classify_accept(&accepted.alpn, accepted.remote_id.as_bytes());
    tracing::info!(
        remote_id = %accepted.remote_id,
        alpn = %String::from_utf8_lossy(&accepted.alpn),
        ?decision,
        "cerulion_remoted: accept classified"
    );

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
            crate::wire::serve_wire_connection(accepted.connection, plane).await;
            return;
        }
        // WireAdmit but no wire plane wired (ops-only builds + the
        // accept-classification tests): fall through to the skeleton report.
    }

    // Everything else (Refuse / UnknownAlpn, and the no-plane fallbacks): the
    // skeleton control-frame stub. Bound the whole handshake: a peer that completed
    // the QUIC handshake then stalls must not hold this task + connection forever.
    match tokio::time::timeout(
        HANDSHAKE_DEADLINE,
        report_decision(&accepted.connection, &decision),
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
) -> Result<(), LinkError> {
    let (mut send, mut recv) = accept_frame_stream(connection).await?;
    // Drain the peer's hello (its presence is what fired accept_bi).
    let _ = read_frame(&mut recv, MAX_HELLO_LEN).await?;
    // The decision is small structured JSON; serialization cannot fail for it.
    let bytes = serde_json::to_vec(decision).unwrap_or_default();
    write_frame(&mut send, &bytes).await?;
    let _ = send.finish();
    // Hold the connection until the peer finishes (EOF) so the reply is delivered.
    let _ = read_frame(&mut recv, MAX_HELLO_LEN).await;
    Ok(())
}

/// Load the device key, trust store, and device→account side-map from disk.
fn load_state(config: &RemotedConfig) -> Result<RemotedState, RemotedError> {
    let device_key = load_device_key(&config.key_file)?;
    // The trust-store MAC key authenticates BOTH the trust store AND the
    // device→account side-map. It is REQUIRED whenever either MAC'd artifact is
    // already on disk: a present store/index whose verification key is gone or
    // unreadable is a PROVISIONING-class / potential-tamper failure (the secret
    // that authenticates this robot's trust state is missing), NOT a config slip.
    // On a genuinely-fresh robot (neither artifact present) the key is required by
    // nothing — its absence is benign, and the absent trust store surfaces its OWN
    // provisioning-gap error (`RemotedError::Store`) downstream (never fabricated
    // into an empty store — see the `RemotedError::Store` docs).
    let mac_required = config.store_file.exists() || config.index_file.exists();
    let mac_key = load_mac_key(&config.store_mac_key_file, mac_required)?;
    let trust_store =
        TrustStore::load(&config.store_file, &mac_key).map_err(RemotedError::Store)?;
    // The device→account side-map is MAC-authenticated with the SAME secure-
    // storage key as the trust store: it is the sole device_key→
    // account binding the accept gate trusts.
    let device_index = DeviceAccountIndex::load(&config.index_file, &mac_key)?;
    tracing::info!(
        claimed = trust_store.is_claimed(),
        bound_devices = device_index.len(),
        "cerulion_remoted: state loaded"
    );
    Ok(RemotedState {
        device_key,
        trust_store,
        device_index,
        mac_key,
    })
}

/// Load the 32-byte ed25519 device secret (raw bytes). An absent / wrong-size
/// key is a provisioning gap, never fabricated.
fn load_device_key(path: &Path) -> Result<[u8; 32], RemotedError> {
    let bytes = std::fs::read(path).map_err(|e| {
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
fn load_mac_key(path: &Path, expected: bool) -> Result<Vec<u8>, RemotedError> {
    match std::fs::read(path) {
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
}
