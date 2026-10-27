// SPDX-License-Identifier: AGPL-3.0-only
//! The act verbs through the production serving core `daemon::serve_endpoint`
//! (the one `daemon::run` delegates to): a paired operator's ops session sends
//! each of `topic-publish`, `service-call`, `topic-snapshot` and
//! `topic-echo-once` with empty arguments and gets that handler's own usage
//! error back, so a verb missing from the daemon's registration fails here as
//! `unknown_verb`. Nothing is published, called or opened.
//!
//! Real iroh loopback and a per-test iceoryx2 SHM root; parallel-safe.

use std::future::Future;
use std::net::{Ipv4Addr, Ipv6Addr, SocketAddr};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Duration;

use cerud::client::OpsClient;
use cerud::error::CerudError;
use cerulion_core::{TransportConfig, TransportManager};
use cerulion_link::{
    alpn, build_endpoint, dial, direct_addr, open_frame_stream, EndpointAddr, EndpointConfig,
    EndpointId, QuicOpsStream, RelayConfig,
};
use cerulion_pairing::format::{
    AccessGrant, AccountId, DeviceCert, IntermediateCert, PrincipalKind, PublicKey, RobotId, Role,
    RootSet, Scope, Validity, FORMAT_VERSION,
};
use cerulion_pairing::verify::TrustStore;
use ed25519_dalek::SigningKey;
use tokio::sync::oneshot;

use cerulion_remoted::pairing_verbs::OwnerGrantWire;
use cerulion_remoted::{serve_endpoint, DeviceAccountIndex, RemotedClock, RemotedConfig};

const T_NOW: u64 = 1_000_000_000_000;
const ISSUED: u64 = 500_000_000_000;
const EXPIRY: u64 = 2_000_000_000_000; // > T_NOW (1e12), < grant_validity (1e14)
const CHASSIS: &[u8] = b"daemon-gate-chassis-secret-not-serial-derived";
const OWNER: AccountId = AccountId([10; 32]);
const SUBJECT: AccountId = AccountId([0x5C; 32]); // != OWNER ([10;32]) — not a self-grant
const ROBOT: RobotId = RobotId([0x0B; 32]);

fn unique_id() -> String {
    static N: AtomicU64 = AtomicU64::new(0);
    format!(
        "{}_{}",
        std::process::id(),
        N.fetch_add(1, Ordering::Relaxed)
    )
}

async fn bounded<F: Future>(what: &str, fut: F) -> F::Output {
    match tokio::time::timeout(Duration::from_secs(20), fut).await {
        Ok(v) => v,
        Err(_) => panic!("act-verbs daemon e2e: '{what}' timed out"),
    }
}

async fn disabled_endpoint(secret: [u8; 32]) -> cerulion_link::Endpoint {
    build_endpoint(EndpointConfig::new(secret).with_relay(RelayConfig::Disabled))
        .await
        .expect("endpoint binds")
}

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

fn test_manager(tag: &str) -> Arc<TransportManager> {
    let ix = cerulion_core::testing::iceoryx_test_config();
    TransportManager::init_for_test(
        TransportConfig {
            node_name: format!("act_daemon_{tag}_{}", unique_id()),
            ..Default::default()
        },
        ix,
    )
    .expect("init_for_test")
}

// ── owner-grant fixtures (crib owner_grant_present_test) ─────────────────────

fn sk(seed: u8) -> SigningKey {
    SigningKey::from_bytes(&[seed; 32])
}
fn pk(k: &SigningKey) -> PublicKey {
    PublicKey(k.verifying_key().to_bytes())
}
fn grant_validity() -> Validity {
    Validity {
        not_before_ns: 0,
        not_after_ns: 100_000_000_000_000,
    }
}
fn op_scope() -> Scope {
    Scope {
        role: Role::OPERATOR,
        caps: Scope::CAP_TELEOP | Scope::CAP_OBSERVE,
    }
}

/// A CLAIMED store (owner = OWNER) with the REAL root pubkey so the grant chain verifies.
fn grant_root_store() -> TrustStore {
    let root_set = RootSet::new(vec![pk(&sk(1))], 1).unwrap();
    let mut store = TrustStore::provision(ROBOT, pk(&sk(0x77)), root_set, CHASSIS, 0).unwrap();
    store
        .claim(OWNER, CHASSIS, PrincipalKind::Human, T_NOW)
        .unwrap();
    store
}

/// A real owner-signed grant for `subject_device_key` (the desk's iroh key), expiring at
/// `EXPIRY` (valid at T_NOW).
fn expiring_owner_grant_for(subject_device_key: [u8; 32]) -> OwnerGrantWire {
    OwnerGrantWire {
        intermediate: IntermediateCert {
            version: FORMAT_VERSION,
            intermediate_key: pk(&sk(10)),
            validity: grant_validity(),
            issued_at_ns: ISSUED,
            max_scope: Scope::OWNER_FULL,
        }
        .sign_by_roots(&[&sk(1)]),
        owner_cert: DeviceCert {
            version: FORMAT_VERSION,
            device_key: pk(&sk(30)),
            account: OWNER,
            principal_kind: PrincipalKind::Human,
            scope: Scope::OWNER_FULL,
            validity: grant_validity(),
            issued_at_ns: ISSUED,
            issuer_key: pk(&sk(10)),
        }
        .sign(&sk(10)),
        subject_cert: DeviceCert {
            version: FORMAT_VERSION,
            device_key: PublicKey(subject_device_key),
            account: SUBJECT,
            principal_kind: PrincipalKind::Human,
            scope: op_scope(),
            validity: grant_validity(),
            issued_at_ns: ISSUED,
            issuer_key: pk(&sk(10)),
        }
        .sign(&sk(10)),
        access_grant: AccessGrant {
            version: FORMAT_VERSION,
            subject: SUBJECT,
            robot: ROBOT,
            scope: op_scope(),
            principal_kind: PrincipalKind::Human,
            validity: Validity {
                not_before_ns: 0,
                not_after_ns: EXPIRY,
            },
            issued_at_ns: ISSUED,
            owner: OWNER,
            owner_device_key: pk(&sk(30)),
        }
        .sign(&sk(30)),
    }
}

/// Run `calls` on one ops session to `addr` from `desk`, on a blocking task
/// (the sync `OpsClient` reads and writes the QUIC stream synchronously).
async fn ops_session(
    desk: &cerulion_link::Endpoint,
    addr: EndpointAddr,
    verbs: Vec<&'static str>,
) -> Vec<Result<serde_json::Value, CerudError>> {
    let conn = bounded("dial ops", dial(desk, addr, alpn::OPS))
        .await
        .expect("dial ops");
    let (send, recv) = bounded("open ops stream", open_frame_stream(&conn))
        .await
        .expect("open ops stream");
    let results = bounded(
        "ops calls",
        tokio::task::spawn_blocking(move || {
            let mut client =
                OpsClient::connect_current(QuicOpsStream::new(send, recv)).expect("handshake");
            verbs
                .into_iter()
                .map(|verb| client.call(verb, serde_json::json!({})))
                .collect::<Vec<_>>()
        }),
    )
    .await
    .expect("ops task");
    drop(conn);
    results
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn the_production_daemon_registers_every_act_verb() {
    let robot_ep = disabled_endpoint([0x33; 32]).await;
    let robot_addr = loopback_addr(robot_ep.id(), &robot_ep.bound_sockets());
    let desk_ep = disabled_endpoint([0x44; 32]).await;
    let desk_key = *desk_ep.id().as_bytes();

    let mut store = grant_root_store();
    store
        .establish_by_owner_grant(
            &expiring_owner_grant_for(desk_key).into_presentation(),
            &PublicKey(desk_key),
            T_NOW,
            Some("desk".into()),
        )
        .expect("establish the desk's owner grant");
    let mut index = DeviceAccountIndex::new();
    index.bind(PublicKey(desk_key), SUBJECT);

    let manager = test_manager("act");
    let dir = tempfile::tempdir().unwrap();
    std::fs::create_dir_all(dir.path().join("remoted")).unwrap();
    let config = RemotedConfig::from_state_root(dir.path(), RelayConfig::Disabled, false);
    std::fs::create_dir_all(&config.log_root).unwrap();

    let (shutdown_tx, shutdown_rx) = oneshot::channel::<()>();
    let serve = tokio::spawn(async move {
        let _ = serve_endpoint(
            &config,
            store,
            index,
            Vec::new(),
            &robot_ep,
            RemotedClock::fixed(T_NOW),
            None,
            Some(manager),
            async {
                let _ = shutdown_rx.await;
            },
        )
        .await;
    });

    // Each verb, called with `{}`, fails on its first argument check, before
    // any schema, shared-memory or service work: the exact usage error proves
    // the daemon dispatched to the registered handler (an unregistered verb is
    // `unknown_verb`, a refused one `denied`).
    let verbs = vec![
        "topic-publish",
        "service-call",
        "topic-snapshot",
        "topic-echo-once",
    ];
    let results = ops_session(&desk_ep, robot_addr, verbs.clone()).await;
    let want = [
        "verb error: usage: missing string argument `topic`",
        "verb error: usage: missing string argument `service`",
        "verb error: usage: missing string argument `topic`",
        "verb error: usage: missing string argument `topic`",
    ];
    for ((verb, result), message) in verbs.iter().zip(results).zip(want) {
        match result {
            Err(CerudError::Remote { kind, message: got }) => {
                assert_eq!(
                    (kind.as_str(), got.as_str()),
                    ("verb_error", message),
                    "{verb}"
                )
            }
            other => panic!("{verb}: expected the handler's usage error, got {other:?}"),
        }
    }

    let _ = shutdown_tx.send(());
    let _ = bounded("serve join", serve).await;
}
