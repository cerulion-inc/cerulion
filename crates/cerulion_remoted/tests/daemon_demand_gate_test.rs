// SPDX-License-Identifier: AGPL-3.0-only
//! Inert-wiring pin: a daemon-level e2e through the
//! production `daemon::serve_endpoint` seam — the exact serving core `daemon::run`
//! delegates to — proving the `.with_demand_authorizer(classify)` wiring is LIVE.
//!
//! Without this, reverting that one line to the deny-nothing default keeps every other
//! test green (they build the `WirePlane` directly with an injected authorizer). Here the
//! authorizer is wired by `serve_endpoint` itself; a mid-session grant-expiry EVICTS the
//! demander → reverting the wiring line makes the eviction never happen and this test
//! fails at the deadline.
//!
//! Deterministic: the access-check clock is a fixed, advanceable `RemotedClock` and the
//! desk's access rides a REAL owner-signed grant with a finite expiry. Real iroh loopback
//! plus a per-test iceoryx2 SHM root (injected into `serve_endpoint`) make it
//! parallel-safe without `#[serial]`; it is the sole test in its own binary.

use std::future::Future;
use std::net::{Ipv4Addr, Ipv6Addr, SocketAddr};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use cerulion_core::wire::{MaxSliceLen, WireHeader};
use cerulion_core::{TransportConfig, TransportManager};
use cerulion_link::{
    accept_uni_frame_stream, alpn, build_endpoint, dial, direct_addr, open_frame_stream,
    read_frame, write_frame, EndpointAddr, EndpointConfig, EndpointId, RecvStream, RelayConfig,
    SendStream, DEFAULT_MAX_FRAME_LEN,
};
use cerulion_pairing::format::{
    AccessGrant, AccountId, DeviceCert, IntermediateCert, PrincipalKind, PublicKey, RobotId, Role,
    RootSet, Scope, Validity, FORMAT_VERSION,
};
use cerulion_pairing::verify::TrustStore;
use ed25519_dalek::SigningKey;
use tokio::sync::oneshot;

use cerulion_remoted::pairing_verbs::OwnerGrantWire;
use cerulion_remoted::{
    serve_endpoint, DeviceAccountIndex, RemotedClock, RemotedConfig, StreamPreamble, WireRequest,
    WireResponse,
};

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
        Err(_) => panic!("daemon-gate e2e: '{what}' timed out"),
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
            node_name: format!("daemon_gate_{tag}_{}", unique_id()),
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

async fn open_control(conn: &cerulion_link::Connection) -> (SendStream, RecvStream) {
    bounded("open control", open_frame_stream(conn))
        .await
        .expect("open_bi control stream")
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

/// Read `ustream` until it RESETS (an error) or the bound elapses; `true` if reset.
async fn read_until_reset(ustream: &mut RecvStream) -> bool {
    let deadline = Instant::now() + Duration::from_secs(12);
    while Instant::now() < deadline {
        match tokio::time::timeout(
            Duration::from_millis(500),
            read_frame(ustream, DEFAULT_MAX_FRAME_LEN),
        )
        .await
        {
            Ok(Ok(_)) => continue,
            Ok(Err(_)) => return true,
            Err(_) => continue,
        }
    }
    false
}

fn make_frame(seq: u32) -> Vec<u8> {
    let payload = b"daemon-frame";
    let header = WireHeader {
        schema_hash: 0xCE87_0DEA_D000_F00D,
        total_size: (WireHeader::SIZE + payload.len()) as u32,
        offset_table_offset: (WireHeader::SIZE + payload.len()) as u32,
        offset_table_count: 0,
        sequence: seq,
        timestamp_ns: 1_000_000 + seq as u64 * 10_000,
    };
    let mut frame = vec![0u8; WireHeader::SIZE + payload.len()];
    header.write_to_buf(&mut frame[..WireHeader::SIZE]);
    frame[WireHeader::SIZE..].copy_from_slice(payload);
    frame
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn daemon_serve_endpoint_evicts_a_revoked_demander_through_the_production_wiring() {
    // The robot endpoint is built EXTERNALLY (so the test knows its loopback addr) — the
    // one thing `serve_endpoint` does not build (`run` builds it internally + passes it in).
    let robot_ep = disabled_endpoint([0x31; 32]).await;
    let robot_addr = loopback_addr(robot_ep.id(), &robot_ep.bound_sockets());
    let desk_ep = disabled_endpoint([0x42; 32]).await;
    let desk_key = *desk_ep.id().as_bytes();

    // In-memory trust: claimed store + the desk's EXPIRING owner grant established (the
    // store method + a direct index bind — the SharedTrust::commit shape for a real
    // pairing). Empty MAC key ⇒ in-memory only, no disk.
    let mut store = grant_root_store();
    store
        .establish_by_owner_grant(
            &expiring_owner_grant_for(desk_key).into_presentation(),
            &PublicKey(desk_key),
            T_NOW,
            Some("desk".into()),
        )
        .expect("establish the desk's expiring owner grant");
    let mut index = DeviceAccountIndex::new();
    index.bind(PublicKey(desk_key), SUBJECT);

    // A per-test SHM root + a producer on the demanded topic — injected into
    // `serve_endpoint`, so the wire plane's tap sees it (parallel-safe, no default root).
    let manager = test_manager("evict");
    let topic = format!("/wire/daemon/{}", unique_id());
    let stop = Arc::new(AtomicBool::new(false));
    let producer = {
        let manager = manager.clone();
        let topic = topic.clone();
        let stop = stop.clone();
        std::thread::spawn(move || {
            let mut publisher = manager
                .create_publisher_simple(&topic, MaxSliceLen::const_new(256))
                .expect("producer");
            let mut seq = 0u32;
            while !stop.load(Ordering::Relaxed) {
                let _ = publisher.publish_raw(&make_frame(seq));
                seq = seq.wrapping_add(1);
                std::thread::sleep(Duration::from_millis(5));
            }
        })
    };

    // A tempdir config for the ops plane (receipt/log paths).
    let dir = tempfile::tempdir().unwrap();
    std::fs::create_dir_all(dir.path().join("remoted")).unwrap();
    let config = RemotedConfig::from_state_root(dir.path(), RelayConfig::Disabled, false);
    std::fs::create_dir_all(&config.log_root).unwrap();

    // Drive the PRODUCTION serving core with a fixed, advanceable access clock + a short
    // sweep + the per-test manager. `run` delegates to this exact seam with (wall, None,
    // None); the `.with_demand_authorizer(classify)` wiring line under test is inside it.
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

    // The desk dials the wire ALPN (admitted at accept — grant valid at T_NOW), demands
    // the topic, and reads its uni stream.
    let conn = bounded("dial", dial(&desk_ep, robot_addr, alpn::WIRE))
        .await
        .expect("dial wire");
    let (mut send, mut recv) = open_control(&conn).await;
    let resp = request(
        &mut send,
        &mut recv,
        &WireRequest::Demand {
            topic: topic.clone(),
        },
    )
    .await;
    assert_eq!(
        resp,
        WireResponse::DemandAccepted {
            topic: topic.clone()
        },
        "the paired + allowed demand is accepted through serve_endpoint's wiring"
    );
    let mut ustream = bounded("accept_uni", accept_uni_frame_stream(&conn))
        .await
        .expect("uni");
    let preamble: StreamPreamble = serde_json::from_slice(
        &bounded("preamble", read_frame(&mut ustream, DEFAULT_MAX_FRAME_LEN))
            .await
            .expect("preamble"),
    )
    .unwrap();
    assert_eq!(preamble.topic, topic);
    // Healthy pre-revoke.
    bounded("pre frame", read_frame(&mut ustream, DEFAULT_MAX_FRAME_LEN))
        .await
        .expect("stream healthy pre-revoke");

    // REVOKE mid-session: advance the robot's access clock past the grant's expiry. The
    // wire plane's sweep (wired via serve_endpoint) re-checks the demander → Deny → evicts.
    clock.set(EXPIRY);
    assert!(
        read_until_reset(&mut ustream).await,
        "a mid-session grant-expiry must EVICT the demander through serve_endpoint's wiring; \
         reverting the .with_demand_authorizer wiring would never evict"
    );

    // Clean teardown.
    drop(conn);
    let _ = shutdown_tx.send(());
    let _ = bounded("serve join", serve).await;
    stop.store(true, Ordering::Relaxed);
    let _ = producer.join();
}
