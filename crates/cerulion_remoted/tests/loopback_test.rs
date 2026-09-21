// SPDX-License-Identifier: AGPL-3.0-only
//! Two-endpoint loopback accept-gate test (RelayConfig::Disabled, direct dial).
//!
//! Over REAL iroh endpoints the daemon's [`handle_accepted`] classifies the
//! dialer's TLS-authenticated `remote_id` and reports the [`AcceptDecision`] on
//! the skeleton control frame; the dialer reads it back and we assert it. The
//! authorizer is seeded with the dialer's ACTUAL endpoint id (read from its own
//! endpoint), so the classification runs against the real authenticated key — no
//! oracle guessing. Every await is bounded so CI can never hang.

use std::future::Future;
use std::net::{Ipv4Addr, Ipv6Addr, SocketAddr};
use std::sync::Arc;
use std::time::Duration;

use cerulion_link::{
    accept_one, alpn, build_endpoint, dial, direct_addr, open_frame_stream, read_frame,
    write_frame, Endpoint, EndpointAddr, EndpointConfig, EndpointId, RelayConfig,
};
use cerulion_pairing::format::{AccountId, PrincipalKind, PublicKey, RobotId, RootSet};
use cerulion_pairing::verify::TrustStore;

use cerulion_remoted::{
    handle_accepted, run, serve, AcceptDecision, DeviceAccountIndex, PairingAuthorizer,
    RemotedConfig, RemotedError,
};

const STEP_TIMEOUT: Duration = Duration::from_secs(15);
const T_NOW: u64 = 1_000_000_000_000;
const CHASSIS: &[u8] = b"remoted-loopback-chassis-secret-not-serial-derived";
const OWNER: AccountId = AccountId([10; 32]);

async fn bounded<F: Future>(what: &str, fut: F) -> F::Output {
    match tokio::time::timeout(STEP_TIMEOUT, fut).await {
        Ok(v) => v,
        Err(_) => panic!("remoted loopback: '{what}' timed out after {STEP_TIMEOUT:?}"),
    }
}

async fn disabled_endpoint(secret: [u8; 32]) -> Endpoint {
    bounded(
        "build_endpoint",
        build_endpoint(EndpointConfig::new(secret).with_relay(RelayConfig::Disabled)),
    )
    .await
    .expect("endpoint should bind")
}

/// Rewrite an endpoint's unspecified (`0.0.0.0` / `[::]`) bound sockets to their
/// loopback form so they are dialable.
fn loopback_addr(id: EndpointId, bound: &[SocketAddr]) -> EndpointAddr {
    let dialable = bound.iter().map(|s| match s {
        SocketAddr::V4(v4) if v4.ip().is_unspecified() => {
            SocketAddr::from((Ipv4Addr::LOCALHOST, v4.port()))
        }
        SocketAddr::V6(v6) if v6.ip().is_unspecified() => {
            SocketAddr::from((Ipv6Addr::LOCALHOST, v6.port()))
        }
        other => *other,
    });
    direct_addr(id, dialable)
}

/// A provisioned + CLAIMED trust store (owner = [`OWNER`], OWNER_FULL scope).
fn claimed_store() -> TrustStore {
    let root_set = RootSet::new(vec![PublicKey([1; 32])], 1).unwrap();
    let mut store = TrustStore::provision(
        RobotId([5; 32]),
        PublicKey([6; 32]),
        root_set,
        CHASSIS,
        T_NOW,
    )
    .unwrap();
    store
        .claim(OWNER, CHASSIS, PrincipalKind::Human, T_NOW)
        .unwrap();
    store
}

/// A provisioned-but-UNCLAIMED trust store.
fn unclaimed_store() -> TrustStore {
    let root_set = RootSet::new(vec![PublicKey([1; 32])], 1).unwrap();
    TrustStore::provision(
        RobotId([5; 32]),
        PublicKey([6; 32]),
        root_set,
        CHASSIS,
        T_NOW,
    )
    .unwrap()
}

/// The MAC key the store↔device binding wiring tests write to disk.
const WIRING_MAC_KEY: &[u8] = b"loopback-wiring-mac-key";

/// Write a full daemon state root (`<root>/remoted/{device_key, trust_store.mac_key,
/// trust_store}`), provisioning + claiming the store against `robot_transport_key`.
/// The device index is left absent (a fresh empty map). Returns the resolved
/// config (network ENABLED, relays disabled). Used by the binding wiring tests.
fn write_state(
    root: &std::path::Path,
    secret: [u8; 32],
    robot_transport_key: [u8; 32],
) -> RemotedConfig {
    let remoted = root.join("remoted");
    std::fs::create_dir_all(&remoted).unwrap();
    std::fs::write(remoted.join("device_key"), secret).unwrap();
    std::fs::write(remoted.join("trust_store.mac_key"), WIRING_MAC_KEY).unwrap();

    let root_set = RootSet::new(vec![PublicKey([1; 32])], 1).unwrap();
    let mut store = TrustStore::provision(
        RobotId([5; 32]),
        PublicKey(robot_transport_key),
        root_set,
        CHASSIS,
        T_NOW,
    )
    .unwrap()
    .with_path(remoted.join("trust_store"));
    store
        .claim(OWNER, CHASSIS, PrincipalKind::Human, T_NOW)
        .unwrap();
    store.save(WIRING_MAC_KEY).unwrap();

    RemotedConfig::from_state_root(root, RelayConfig::Disabled, false)
}

/// Claimed robot; the dialer's key is bound to the OWNER (paired, OWNER_FULL).
fn claimed_paired(client_key: [u8; 32]) -> PairingAuthorizer {
    let mut index = DeviceAccountIndex::new();
    index.bind(PublicKey(client_key), OWNER);
    PairingAuthorizer::new(claimed_store(), index)
}

/// Claimed robot; the dialer's key is NOT bound (unpaired).
fn claimed_unpaired(_client_key: [u8; 32]) -> PairingAuthorizer {
    PairingAuthorizer::new(claimed_store(), DeviceAccountIndex::new())
}

/// Unclaimed robot; empty side-map.
fn unclaimed(_client_key: [u8; 32]) -> PairingAuthorizer {
    PairingAuthorizer::new(unclaimed_store(), DeviceAccountIndex::new())
}

/// Drive one accept over loopback: the daemon (secret [200;…]) classifies the
/// dialer (built from `client_secret`) on `alpn`; the dialer reads the reported
/// [`AcceptDecision`]. `make_authz` is handed the dialer's REAL endpoint id.
async fn accept_decision(
    daemon_secret: [u8; 32],
    client_secret: [u8; 32],
    alpn: &[u8],
    make_authz: fn([u8; 32]) -> PairingAuthorizer,
) -> AcceptDecision {
    let daemon = disabled_endpoint(daemon_secret).await;
    let client = disabled_endpoint(client_secret).await;
    let client_key = *client.id().as_bytes();
    let authz = Arc::new(make_authz(client_key));
    let daemon_addr = loopback_addr(daemon.id(), &daemon.bound_sockets());

    let daemon_ref = &daemon;
    let client_ref = &client;

    let server_fut = async move {
        let accepted = bounded("accept_one", accept_one(daemon_ref))
            .await
            .expect("accept ok")
            .expect("an incoming connection");
        // `ops = None`: this harness exercises the accept-time CLASSIFY over a real
        // iroh connection via the skeleton decision frame (the real cerud ops path,
        // with `ops = Some`, is `ops_loopback_test.rs`). For an ops decision that
        // means `handle_accepted` falls through to `report_decision`, so the dialer
        // reads the classified AcceptDecision back.
        handle_accepted(accepted, authz, None).await;
    };
    let client_fut = async move {
        let conn = bounded("dial", dial(client_ref, daemon_addr, alpn))
            .await
            .expect("dial ok");
        let (mut send, mut recv) = bounded("open_bi", open_frame_stream(&conn))
            .await
            .expect("open_bi ok");
        bounded("hello", write_frame(&mut send, b"hello"))
            .await
            .expect("hello write ok");
        let frame = bounded("read decision", read_frame(&mut recv, 65536))
            .await
            .expect("read decision ok");
        let _ = send.finish();
        serde_json::from_slice::<AcceptDecision>(&frame).expect("decision json")
        // conn/send/recv drop on return → the daemon's read-to-EOF completes.
    };

    let (_, decision) = tokio::join!(server_fut, client_fut);
    decision
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn ops_paired_key_is_admitted() {
    let d = accept_decision([200; 32], [9; 32], alpn::OPS, claimed_paired).await;
    assert_eq!(d, AcceptDecision::OpsAdmit);
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn ops_unpaired_key_reaches_bootstrap_only() {
    let d = accept_decision([201; 32], [10; 32], alpn::OPS, claimed_unpaired).await;
    assert_eq!(d, AcceptDecision::OpsBootstrapOnly);
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn wire_paired_observe_key_is_admitted() {
    let d = accept_decision([202; 32], [11; 32], alpn::WIRE, claimed_paired).await;
    assert_eq!(d, AcceptDecision::WireAdmit);
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn wire_unpaired_key_is_refused() {
    let d = accept_decision([203; 32], [12; 32], alpn::WIRE, claimed_unpaired).await;
    match d {
        AcceptDecision::Refuse { reason } => {
            assert!(reason.contains("unpaired"), "reason: {reason}");
        }
        other => panic!("expected Refuse, got {other:?}"),
    }
}

/// The accept loop must SURVIVE one bad inbound handshake
/// (a scanner sending a bogus ALPN / a peer aborting mid-handshake). Drives
/// `serve` (not `handle_accepted`) with a BAD connection first, then proves a
/// subsequent VALID paired peer is still served — so an unauthenticated remote
/// cannot kill the robot's always-on remote plane.
///
/// Reverting `serve`'s per-accept `Err` arm to `accepted?`
/// makes `serve` return on the bad handshake, the good peer's read never
/// completes, and the bounded await panics.
#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn serve_survives_a_bad_inbound_handshake_and_serves_the_next() {
    use tokio::sync::oneshot;

    let daemon = disabled_endpoint([210; 32]).await;
    let good_client = disabled_endpoint([44; 32]).await;
    let bad_client = disabled_endpoint([45; 32]).await;
    let good_key = *good_client.id().as_bytes();
    let authz = Arc::new(claimed_paired(good_key));
    let daemon_addr = loopback_addr(daemon.id(), &daemon.bound_sockets());

    let (shutdown_tx, shutdown_rx) = oneshot::channel::<()>();
    let daemon_ref = &daemon;

    let server_fut = async move {
        serve(daemon_ref, authz, None, async {
            let _ = shutdown_rx.await;
        })
        .await
    };

    let client_fut = async {
        // 1) A BAD inbound connection: a bogus ALPN the daemon does not accept.
        //    Its handshake fails on the daemon's `incoming.await` → `accept_one`
        //    returns Err. The loop must log + continue, NOT terminate.
        let bad = bounded(
            "bad dial",
            dial(&bad_client, daemon_addr.clone(), b"cerulion/bogus/9"),
        )
        .await;
        assert!(bad.is_err(), "a bogus-ALPN dial should fail its handshake");
        // Let the daemon observe + skip the failed accept before the good dial.
        tokio::time::sleep(Duration::from_millis(300)).await;

        // 2) A GOOD ops dial from a paired key: it MUST be served (proving the
        //    accept loop survived the bad handshake above).
        let conn = bounded(
            "good dial",
            dial(&good_client, daemon_addr.clone(), alpn::OPS),
        )
        .await
        .expect("good dial ok");
        let (mut send, mut recv) = bounded("open_bi", open_frame_stream(&conn))
            .await
            .expect("open_bi ok");
        bounded("hello", write_frame(&mut send, b"hello"))
            .await
            .expect("hello ok");
        let frame = bounded("read decision", read_frame(&mut recv, 65536))
            .await
            .expect("read decision ok");
        let _ = send.finish();
        let decision: AcceptDecision = serde_json::from_slice(&frame).expect("decision json");
        let _ = shutdown_tx.send(());
        decision
    };

    let (server_result, decision) = tokio::join!(server_fut, client_fut);
    assert!(
        server_result.is_ok(),
        "serve must return Ok after a bad handshake, got {server_result:?}"
    );
    assert_eq!(
        decision,
        AcceptDecision::OpsAdmit,
        "the good paired peer must be served AFTER the bad handshake"
    );
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn unclaimed_robot_wire_refused_ops_bootstrap_only() {
    // Wire plane on an unclaimed robot → refused outright.
    let wire = accept_decision([204; 32], [13; 32], alpn::WIRE, unclaimed).await;
    match wire {
        AcceptDecision::Refuse { reason } => {
            assert!(reason.contains("UNCLAIMED"), "reason: {reason}");
        }
        other => panic!("expected Refuse, got {other:?}"),
    }
    // Ops plane on an unclaimed robot → bootstrap surface only (only claim is
    // admissible there; enforced per-verb by the authorizer downstream).
    let ops = accept_decision([205; 32], [14; 32], alpn::OPS, unclaimed).await;
    assert_eq!(ops, AcceptDecision::OpsBootstrapOnly);
}

/// WIRING: `run` REFUSES to serve when the loaded store's
/// `robot_transport_key` does not match this device's endpoint id (a
/// transplanted / restored / fleet-misprovisioned store). Proves the binding
/// check is wired into `run`, not merely a dead pure fn.
#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn run_refuses_a_store_not_bound_to_this_device() {
    let dir = tempfile::tempdir().unwrap();
    let secret = [220u8; 32];
    // Learn this device's REAL endpoint id, then provision the store against a
    // one-byte-different key → guaranteed mismatch (no assumption about the id).
    let ep = disabled_endpoint(secret).await;
    let mut wrong = *ep.id().as_bytes();
    drop(ep);
    wrong[0] ^= 0xff;

    let cfg = write_state(dir.path(), secret, wrong);
    let err = bounded("run(mismatch)", run(cfg, async {}))
        .await
        .expect_err("a mismatched store must refuse to serve");
    assert!(
        matches!(err, RemotedError::ProvisioningMismatch(_)),
        "expected ProvisioningMismatch, got {err:?}"
    );
    assert!(err.to_string().contains("does not match"), "err: {err}");
    assert!(
        err.to_string().contains("re-provision"),
        "err states the fix: {err}"
    );
}

/// The control: `run` ACCEPTS a store correctly bound to this device — it
/// binds, passes the check, and exits cleanly on an immediate shutdown.
#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn run_accepts_a_store_bound_to_this_device() {
    let dir = tempfile::tempdir().unwrap();
    let secret = [221u8; 32];
    // Provision the store against this device's ACTUAL endpoint id.
    let ep = disabled_endpoint(secret).await;
    let real_id = *ep.id().as_bytes();
    drop(ep);

    let cfg = write_state(dir.path(), secret, real_id);
    // An immediately-ready shutdown: `run` builds the endpoint, passes the
    // binding check, then the accept loop breaks at once → Ok(()).
    let result = bounded("run(match)", run(cfg, async {})).await;
    assert!(
        result.is_ok(),
        "a matching store must bind + serve then exit cleanly, got {result:?}"
    );
}
