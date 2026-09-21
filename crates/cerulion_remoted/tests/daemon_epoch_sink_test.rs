// SPDX-License-Identifier: AGPL-3.0-only
//! The inert-wiring pin: a DAEMON-LEVEL e2e through the production
//! `daemon::serve_endpoint` seam — the exact serving core `daemon::run` delegates to —
//! proving the `.with_epoch_sink(epoch_sink, access_clock)` wiring is LIVE.
//!
//! Without this, reverting that one line keeps every other epoch-push test green
//! (`epoch_push_e2e_test.rs` builds the `WirePlane` directly and installs the sink
//! itself). Here the sink is wired by `serve_endpoint`; a real desk pushes a real
//! CA-signed epoch over the real `cerulion/wire/1` plane and the robot must both REPORT
//! it applied AND actually refuse the revoked party afterwards. Reverting the wiring
//! turns the reply into the sink-less refusal and this test fails.
//!
//! It also pins the seam's CLOCK half: `serve_endpoint` hands the sink the SAME
//! `access_clock` the access list runs on, so `apply_epoch`'s anti-rollback floor and
//! the access checks agree. A fixed clock makes that deterministic.
//!
//! Real iroh loopback + a per-test iceoryx2 SHM root (injected into `serve_endpoint`) ⇒
//! parallel-safe without `#[serial]`; sole test in its own binary.

use std::future::Future;
use std::net::{Ipv4Addr, Ipv6Addr, SocketAddr};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Duration;

use cerulion_core::{TransportConfig, TransportManager};
use cerulion_link::{
    alpn, build_endpoint, dial, direct_addr, open_frame_stream, read_frame, write_frame,
    EndpointAddr, EndpointConfig, EndpointId, RecvStream, RelayConfig, SendStream,
    DEFAULT_MAX_FRAME_LEN,
};
use cerulion_pairing::format::{
    AccessListEpoch, AccountId, DeviceCert, Grant, IntermediateCert, PrincipalKind, PublicKey,
    RobotId, Role, RootSet, Scope, SignedEpoch, SignedIntermediateCert, Validity, FORMAT_VERSION,
};
use cerulion_pairing::verify::{EpochSyncWire, PairingPresentation, TrustStore};
use ed25519_dalek::SigningKey;
use tokio::sync::oneshot;

use cerulion_remoted::{
    serve_endpoint, DeviceAccountIndex, RemotedClock, RemotedConfig, WireRequest, WireResponse,
};

const T_NOW: u64 = 1_000_000_000_000;
const ISSUED: u64 = 500_000_000_000;
const CHASSIS: &[u8] = b"daemon-sink-chassis-secret-not-serial-derived";
const OWNER: AccountId = AccountId([10; 32]);
const SUBJECT: AccountId = AccountId([0x5C; 32]);
/// The account the pushed epoch revokes — a PROBE that never dials, so its post-push
/// refusal at the accept gate is a clean observable.
const PROBE: AccountId = AccountId([0x77; 32]);
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
        Err(_) => panic!("daemon epoch-sink e2e: '{what}' timed out"),
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
            node_name: format!("daemon_sink_{tag}_{}", unique_id()),
            ..Default::default()
        },
        ix,
    )
    .expect("init_for_test")
}

fn sk(seed: u8) -> SigningKey {
    SigningKey::from_bytes(&[seed; 32])
}
fn pk(k: &SigningKey) -> PublicKey {
    PublicKey(k.verifying_key().to_bytes())
}
fn wide() -> Validity {
    Validity {
        not_before_ns: 0,
        not_after_ns: 100_000_000_000_000,
    }
}
fn viewer_scope() -> Scope {
    Scope {
        role: Role::VIEWER,
        caps: Scope::CAP_OBSERVE,
    }
}

fn intermediate_cert() -> SignedIntermediateCert {
    IntermediateCert {
        version: FORMAT_VERSION,
        intermediate_key: pk(&sk(10)),
        validity: wide(),
        issued_at_ns: ISSUED,
        max_scope: Scope::OWNER_FULL,
    }
    .sign_by_roots(&[&sk(1)])
}

/// A REAL strong-chain presentation binding `device_key` to `account` (observer scope).
fn presentation_for(device_key: [u8; 32], account: AccountId) -> PairingPresentation {
    PairingPresentation {
        intermediate: intermediate_cert(),
        device_cert: DeviceCert {
            version: FORMAT_VERSION,
            device_key: PublicKey(device_key),
            account,
            principal_kind: PrincipalKind::Human,
            scope: viewer_scope(),
            validity: wide(),
            issued_at_ns: ISSUED,
            issuer_key: pk(&sk(10)),
        }
        .sign(&sk(10)),
        grant: Grant {
            version: FORMAT_VERSION,
            subject: account,
            robot: ROBOT,
            scope: viewer_scope(),
            principal_kind: PrincipalKind::Human,
            delegation_depth: 0,
            validity: wide(),
            issued_at_ns: ISSUED,
            issuer: AccountId([2; 32]),
            issuer_key: pk(&sk(10)),
        }
        .sign(&sk(10)),
        delegation: None,
    }
}

/// A genuine CA-signed epoch revoking `accounts`, chained to the trusted intermediate.
fn epoch(n: u64, accounts: Vec<AccountId>) -> SignedEpoch {
    AccessListEpoch {
        version: FORMAT_VERSION,
        robot: ROBOT,
        epoch: n,
        revoked_accounts: accounts,
        revoked_devices: vec![],
        issued_at_ns: ISSUED,
        issuer_key: pk(&sk(10)),
    }
    .sign(&sk(10))
}

async fn request(send: &mut SendStream, recv: &mut RecvStream, req: &WireRequest) -> WireResponse {
    let bytes = serde_json::to_vec(req).unwrap();
    bounded("write request", write_frame(send, &bytes))
        .await
        .expect("write request");
    let resp = bounded("read response", read_frame(recv, DEFAULT_MAX_FRAME_LEN))
        .await
        .expect("read response");
    serde_json::from_slice(&resp).expect("decode WireResponse")
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn daemon_serve_endpoint_applies_a_pushed_epoch_through_the_production_wiring() {
    // The robot endpoint is built EXTERNALLY (so the test knows its loopback addr) — the
    // one thing `serve_endpoint` does not build (`run` builds it internally + passes in).
    let robot_ep = disabled_endpoint([0x31; 32]).await;
    let robot_addr = loopback_addr(robot_ep.id(), &robot_ep.bound_sockets());
    let desk_ep = disabled_endpoint([0x42; 32]).await;
    let desk_key = *desk_ep.id().as_bytes();
    // The PROBE desk pairs but never dials until AFTER the push.
    let probe_ep = disabled_endpoint([0x44; 32]).await;
    let probe_key = *probe_ep.id().as_bytes();

    // In-memory trust: a claimed store with BOTH desks strong-paired. Empty MAC key ⇒
    // in-memory only, no disk.
    let root_set = RootSet::new(vec![pk(&sk(1))], 1).unwrap();
    let mut store = TrustStore::provision(ROBOT, pk(&sk(0x77)), root_set, CHASSIS, 0).unwrap();
    store
        .claim(OWNER, CHASSIS, PrincipalKind::Human, T_NOW)
        .unwrap();
    let mut index = DeviceAccountIndex::new();
    for (key, account) in [(desk_key, SUBJECT), (probe_key, PROBE)] {
        let verified = store
            .verify_new_pairing(&presentation_for(key, account), &PublicKey(key), T_NOW)
            .expect("the presentation verifies");
        store
            .establish_pairing(
                &verified,
                cerulion_pairing::verify::PairingSource::StrongChain,
                Some("desk".into()),
                T_NOW,
                None,
            )
            .expect("establish the pairing");
        index.bind(PublicKey(key), account);
    }

    let manager = test_manager("sink");
    let dir = tempfile::tempdir().unwrap();
    std::fs::create_dir_all(dir.path().join("remoted")).unwrap();
    let config = RemotedConfig::from_state_root(dir.path(), RelayConfig::Disabled, false);
    std::fs::create_dir_all(&config.log_root).unwrap();

    // Drive the PRODUCTION serving core. `run` delegates to this exact seam; the
    // `.with_epoch_sink(..)` wiring line under test lives inside it.
    let clock = RemotedClock::fixed(T_NOW);
    let (shutdown_tx, shutdown_rx) = oneshot::channel::<()>();
    let serve_clock = clock.clone();
    let serve_manager = manager.clone();
    let serve = tokio::spawn(async move {
        let _ = serve_endpoint(
            &config,
            store,
            index,
            Vec::new(),
            &robot_ep,
            serve_clock,
            Some(Duration::from_millis(200)),
            Some(serve_manager),
            async {
                let _ = shutdown_rx.await;
            },
        )
        .await;
    });

    // PRE-CONDITION (anti-tautology): the PROBE key is admitted to the wire plane
    // BEFORE the push — so the post-push refusal below is caused by the epoch, not by a
    // pairing that never worked.
    {
        let conn = bounded(
            "probe pre-dial",
            dial(&probe_ep, robot_addr.clone(), alpn::WIRE),
        )
        .await
        .expect("probe dials");
        let (mut s, mut r) = bounded("probe control", open_frame_stream(&conn))
            .await
            .expect("probe control stream");
        let bytes = serde_json::to_vec(&WireRequest::Catalog).unwrap();
        bounded("probe write", write_frame(&mut s, &bytes))
            .await
            .expect("probe write");
        let raw = bounded("probe read", read_frame(&mut r, DEFAULT_MAX_FRAME_LEN))
            .await
            .expect("probe read");
        let v: serde_json::Value = serde_json::from_slice(&raw).unwrap_or_default();
        assert_eq!(
            v.get("reply").and_then(|x| x.as_str()),
            Some("catalog"),
            "pre-condition: the PROBE key is admitted to the wire plane before the push \
             (got {v})"
        );
    }

    // The DESK dials and pushes a genuine epoch revoking PROBE.
    let conn = bounded("desk dial", dial(&desk_ep, robot_addr.clone(), alpn::WIRE))
        .await
        .expect("desk dials wire");
    let (mut send, mut recv) = bounded("desk control", open_frame_stream(&conn))
        .await
        .expect("desk control stream");
    let blob = hex::encode(
        EpochSyncWire::new(intermediate_cert(), epoch(6, vec![PROBE]))
            .to_postcard()
            .unwrap(),
    );
    let resp = request(
        &mut send,
        &mut recv,
        &WireRequest::SyncEpoch {
            epoch_postcard: blob,
        },
    )
    .await;

    // THE WIRING PIN: with `.with_epoch_sink(..)` present this is the applied reply;
    // revert that line and it becomes the sink-less `Error` refusal.
    assert_eq!(
        resp,
        WireResponse::EpochSynced {
            epoch: 6,
            applied: true
        },
        "serve_endpoint must install the epoch sink so a pushed epoch APPLIES \
         (a sink-less plane answers a loud Error instead)"
    );

    // ...and it is REAL enforcement, not just a reply: the PROBE key is now REFUSED at
    // the wire accept gate (the same live SharedTrust the accept gate reads).
    let refused = bounded("probe re-dial", dial(&probe_ep, robot_addr, alpn::WIRE)).await;
    if let Ok(conn) = refused {
        if let Ok((mut s, mut r)) = open_frame_stream(&conn).await {
            let bytes = serde_json::to_vec(&WireRequest::Catalog).unwrap();
            let _ = bounded("refused write", write_frame(&mut s, &bytes)).await;
            if let Ok(raw) =
                bounded("refused read", read_frame(&mut r, DEFAULT_MAX_FRAME_LEN)).await
            {
                let v: serde_json::Value = serde_json::from_slice(&raw).unwrap_or_default();
                // A refused wire peer gets an `AcceptDecision` (`decision: "refuse"`),
                // never a `WireResponse` (`reply: "catalog"`).
                assert_ne!(
                    v.get("reply").and_then(|x| x.as_str()),
                    Some("catalog"),
                    "the epoch the desk pushed must REFUSE the revoked account at the \
                     accept gate (got {v})"
                );
            }
        }
    }

    let _ = shutdown_tx.send(());
    let _ = bounded("serve shutdown", serve).await;
}
