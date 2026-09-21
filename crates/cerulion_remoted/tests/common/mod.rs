// SPDX-License-Identifier: AGPL-3.0-only
//! Shared loopback ops-plane harness for the ops-plane e2e tests (the
//! e-stop-starvation safety pin, the lease-reconnect e2e, and the offline-connect
//! e2e). Cribbed from `ops_loopback_test.rs`: two REAL iroh endpoints on loopback
//! (direct dial, no relay unless a test injects one), the robot hosting the REAL
//! `cerud::OpsServer` via `OpsServing`, a `cerud::OpsClient` on the desk side.
//!
//! Every key is derived from a fixed seed (Principle #13: deterministic fixtures,
//! never fake data) and every oracle is HAND-BUILT (a real cert chain, real
//! CPace) — the trust state is verified against a subsequent verb / a disk reload,
//! never a self-compare.

#![allow(dead_code)] // each test file uses a subset of these helpers.

use std::future::Future;
use std::net::{Ipv4Addr, Ipv6Addr, SocketAddr};
use std::path::Path;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use cerulion_link::{
    alpn, build_endpoint, dial, direct_addr, open_frame_stream, Endpoint, EndpointAddr,
    EndpointConfig, EndpointId, QuicOpsStream, RelayConfig,
};

use cerud::client::OpsClient;
use cerud::lease::ControlLease;

use cerulion_pairing::format::{
    AccountId, DeviceCert, Grant, IntermediateCert, PrincipalKind, PublicKey, RobotId, Role,
    RootSet, Scope, Validity, FORMAT_VERSION,
};
use cerulion_pairing::verify::TrustStore;
use ed25519_dalek::SigningKey;

use cerulion_remoted::pairing_verbs::{PresentationWire, SharedCodePairSessions};
use cerulion_remoted::{OpsServing, PairingAuthorizer, RemotedClock, SharedTrust};

// ── constants ─────────────────────────────────────────────────────────────────

pub const STEP_TIMEOUT: Duration = Duration::from_secs(20);
pub const T_NOW: u64 = 1_000_000_000_000;
pub const ISSUED: u64 = 500_000_000_000;
pub const CHASSIS: &[u8] = b"chunk8-harness-chassis-secret-not-serial-derived";
pub const MAC_KEY: &[u8] = b"chunk8-harness-firmware-mac-key";
pub const OWNER: AccountId = AccountId([10; 32]);
pub const ROBOT_ID: RobotId = RobotId([5; 32]);

// ── tiny helpers ────────────────────────────────────────────────────────────────

pub async fn bounded<F: Future>(what: &str, fut: F) -> F::Output {
    match tokio::time::timeout(STEP_TIMEOUT, fut).await {
        Ok(v) => v,
        Err(_) => panic!("chunk8 harness: '{what}' timed out after {STEP_TIMEOUT:?}"),
    }
}

/// Build a real iroh endpoint with a chosen relay posture (the offline test
/// injects an unreachable `Custom` relay; every other test uses `Disabled`).
pub async fn endpoint_with_relay(secret: [u8; 32], relay: RelayConfig) -> Endpoint {
    bounded(
        "build_endpoint",
        build_endpoint(EndpointConfig::new(secret).with_relay(relay)),
    )
    .await
    .expect("endpoint should bind")
}

/// Build a real iroh endpoint with NO relay (direct/LAN dial).
pub async fn disabled_endpoint(secret: [u8; 32]) -> Endpoint {
    endpoint_with_relay(secret, RelayConfig::Disabled).await
}

/// Rewrite an endpoint's unspecified (`0.0.0.0` / `[::]`) bound sockets to their
/// loopback form so they are dialable.
pub fn loopback_addr(id: EndpointId, bound: &[SocketAddr]) -> EndpointAddr {
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

pub fn sk(seed: u8) -> SigningKey {
    SigningKey::from_bytes(&[seed; 32])
}
pub fn pk(k: &SigningKey) -> PublicKey {
    PublicKey(k.verifying_key().to_bytes())
}
pub fn wide() -> Validity {
    Validity {
        not_before_ns: 0,
        not_after_ns: 100_000_000_000_000,
    }
}

// ── robot harness ────────────────────────────────────────────────────────────

/// A robot's live trust state + the shared handles a test asserts on / arms.
pub struct Robot {
    pub shared: SharedTrust,
    pub lease: Arc<Mutex<ControlLease>>,
    pub sessions: SharedCodePairSessions,
    pub clock: RemotedClock,
    pub classify: Arc<PairingAuthorizer>,
    pub ops: Arc<OpsServing>,
    pub store_path: std::path::PathBuf,
    pub index_path: std::path::PathBuf,
    pub receipt_path: std::path::PathBuf,
    pub dir: std::path::PathBuf,
}

/// Build a robot over a provisioned (optionally CLAIMED) store whose
/// `robot_transport_key` equals `robot_transport_key` (the robot's iroh endpoint
/// id). `seed_bindings` pre-binds `device_key → account` rows in the side-map (a
/// paired caller without running the whole ceremony — an ESTABLISHED pairing).
/// Persistence is wired to `dir` under a fixed MAC key; a Fixed clock at `T_NOW`.
pub fn build_robot(
    dir: &Path,
    robot_transport_key: [u8; 32],
    claim_owner: bool,
    seed_bindings: &[([u8; 32], AccountId)],
) -> Robot {
    let store_path = dir.join("trust_store");
    let index_path = dir.join("device_index.json");
    let receipt_path = dir.join("receipts.log");

    let root_set = RootSet::new(vec![pk(&sk(1))], 1).unwrap();
    let mut store = TrustStore::provision(
        ROBOT_ID,
        PublicKey(robot_transport_key),
        root_set,
        CHASSIS,
        T_NOW,
    )
    .unwrap()
    .with_path(&store_path);
    if claim_owner {
        store
            .claim(OWNER, CHASSIS, PrincipalKind::Human, T_NOW)
            .unwrap();
        store.save(MAC_KEY).unwrap();
    }
    let mut index = cerulion_remoted::DeviceAccountIndex::new().with_path(&index_path);
    for (key, account) in seed_bindings {
        index.bind(PublicKey(*key), *account);
    }
    if !seed_bindings.is_empty() {
        index.save(MAC_KEY).unwrap();
    }

    let shared = SharedTrust::new(store, index, MAC_KEY.to_vec());
    let lease = Arc::new(Mutex::new(ControlLease::with_default_window()));
    let sessions = SharedCodePairSessions::new();
    let clock = RemotedClock::fixed(T_NOW);
    let classify = Arc::new(PairingAuthorizer::from_shared(shared.clone()));
    let ops = Arc::new(
        OpsServing::new(
            shared.clone(),
            lease.clone(),
            sessions.clone(),
            clock.clone(),
            &receipt_path,
            dir.to_path_buf(),
            "cerulion-remoted-test",
        )
        .expect("ops serving builds"),
    );
    Robot {
        shared,
        lease,
        sessions,
        clock,
        classify,
        ops,
        store_path,
        index_path,
        receipt_path,
        dir: dir.to_path_buf(),
    }
}

/// Build the `pair` verb's `presentation_postcard` argument: a REAL offline cert
/// chain (root-signed intermediate → device cert for `device_key` → grant) with a
/// chosen `scope`, postcard-encoded + hex.
pub fn build_presentation_hex(account: AccountId, device_key: [u8; 32], scope: Scope) -> String {
    let root = sk(1);
    let intermediate = sk(2);
    let int_pk = pk(&intermediate);
    let int_cert = IntermediateCert {
        version: FORMAT_VERSION,
        intermediate_key: int_pk,
        validity: wide(),
        issued_at_ns: ISSUED,
        max_scope: Scope::OWNER_FULL,
    }
    .sign_by_roots(&[&root]);
    let device_cert = DeviceCert {
        version: FORMAT_VERSION,
        device_key: PublicKey(device_key),
        account,
        principal_kind: PrincipalKind::Human,
        scope,
        validity: wide(),
        issued_at_ns: ISSUED,
        issuer_key: int_pk,
    }
    .sign(&intermediate);
    let grant = Grant {
        version: FORMAT_VERSION,
        subject: account,
        robot: ROBOT_ID,
        scope,
        principal_kind: PrincipalKind::Human,
        delegation_depth: 0,
        validity: wide(),
        issued_at_ns: ISSUED,
        issuer: AccountId([2; 32]),
        issuer_key: int_pk,
    }
    .sign(&intermediate);
    PresentationWire {
        intermediate: int_cert,
        device_cert,
        grant,
        delegation: None,
    }
    .to_postcard_hex()
    .expect("presentation encodes")
}

/// A VIEWER + CAP_OBSERVE presentation (the least-privileged paired role).
pub fn viewer_presentation_hex(account: AccountId, device_key: [u8; 32]) -> String {
    build_presentation_hex(
        account,
        device_key,
        Scope {
            role: Role::VIEWER,
            caps: Scope::CAP_OBSERVE,
        },
    )
}

/// Run `f` (a synchronous cerud client session) against the robot over ONE ops
/// connection, dialing `robot_addr` from `client`. Bridges async QUIC → cerud's
/// sync `OpsClient` on a blocking task.
pub async fn run_client<F, R>(client: &Endpoint, robot_addr: EndpointAddr, f: F) -> R
where
    F: FnOnce(&mut OpsClient<QuicOpsStream>) -> R + Send + 'static,
    R: Send + 'static,
{
    let conn = bounded("dial", dial(client, robot_addr, alpn::OPS))
        .await
        .expect("dial ok");
    let (send, recv) = bounded("open_bi", open_frame_stream(&conn))
        .await
        .expect("open_bi ok");
    let ops = QuicOpsStream::new(send, recv);
    let handle = tokio::task::spawn_blocking(move || {
        let mut client = OpsClient::connect_current(ops).expect("cerud handshake");
        f(&mut client)
        // `client` (owning the QuicOpsStream) drops → clean EOF to the robot.
    });
    let r = bounded("client task", handle)
        .await
        .expect("client task join");
    drop(conn);
    r
}

/// Drive `robot.ops` serving on `endpoint` concurrently with `client`, signalling
/// shutdown after the scenario so `serve` returns cleanly.
pub async fn with_serving<C, R>(robot: &Robot, endpoint: &Endpoint, client: C) -> R
where
    C: Future<Output = R>,
{
    let (tx, rx) = tokio::sync::oneshot::channel::<()>();
    let classify = robot.classify.clone();
    let ops = robot.ops.clone();
    let server_fut = async move {
        let _ = cerulion_remoted::serve(endpoint, classify, Some(ops), async {
            let _ = rx.await;
        })
        .await;
    };
    let client_fut = async {
        let out = client.await;
        let _ = tx.send(());
        out
    };
    let (_, out) = tokio::join!(server_fut, client_fut);
    out
}
