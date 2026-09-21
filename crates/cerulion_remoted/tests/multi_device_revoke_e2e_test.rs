// SPDX-License-Identifier: AGPL-3.0-only
//! A6 — THE multi-device selective-revoke e2e, driven through the PRODUCTION
//! sync (`SharedTrust::apply_epoch`) + authorizer (`PairingAuthorizer`) + wire-plane
//! sweep paths — no inert shipping.
//!
//! Two desks (A + B) of the SAME account pair to a robot and each demand + stream a
//! topic over the real `cerulion/wire/1` plane (loopback iroh + a per-test iceoryx2
//! SHM root). Mid-session the robot applies a CA-signed epoch revoking ONLY desk A's
//! DEVICE key. Desk A is EVICTED (its uni stream resets) while desk B stays
//! connected — the surgical, account-preserving device revocation.
//!
//! Every credential is REAL cryptography (deterministic fixed-seed keys — Principle
//! #13); the epoch is a genuine `SignedEpoch`. The eviction is driven by the same
//! `apply_epoch` seam the account-service sync feeds, and the same sweep the
//! grant-expiry e2e (`daemon_demand_gate_test`) exercises — so this is the production
//! path, not a test-only shortcut.

use std::future::Future;
use std::net::{Ipv4Addr, Ipv6Addr, SocketAddr};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use cerulion_core::wire::{MaxSliceLen, WireHeader};
use cerulion_core::{TransportConfig, TransportManager};
use cerulion_link::{
    accept_one, accept_uni_frame_stream, alpn, build_endpoint, dial, direct_addr,
    open_frame_stream, read_frame, write_frame, Connection, EndpointAddr, EndpointConfig,
    EndpointId, RecvStream, RelayConfig, SendStream, DEFAULT_MAX_FRAME_LEN,
};
use cerulion_pairing::format::{
    AccessListEpoch, AccountId, DeviceCert, Grant, IntermediateCert, PrincipalKind, PublicKey,
    RobotId, Role, RootSet, Scope, SignedEpoch, SignedIntermediateCert, Validity, FORMAT_VERSION,
};
use cerulion_pairing::verify::{PairingPresentation, TrustStore};
use ed25519_dalek::SigningKey;

use cerulion_remoted::{
    handle_accepted_with_wire, DeviceAccountIndex, PairingAuthorizer, SharedTrust, StreamPreamble,
    WirePlane, WireRequest, WireResponse,
};

const T_NOW: u64 = 1_000_000_000_000;
const ISSUED: u64 = 500_000_000_000;
const CHASSIS: &[u8] = b"a6-multi-device-chassis-secret-not-serial-derived";
const OWNER: AccountId = AccountId([10; 32]);
const SUBJECT: AccountId = AccountId([0x5C; 32]); // both desks belong to THIS account
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
        Err(_) => panic!("multi-device revoke e2e: '{what}' timed out"),
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
            node_name: format!("a6_revoke_{tag}_{}", unique_id()),
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
/// The observer scope both desks pair with (CAP_OBSERVE is the wire-plane requirement).
fn viewer_scope() -> Scope {
    Scope {
        role: Role::VIEWER,
        caps: Scope::CAP_OBSERVE,
    }
}

/// The root-signed intermediate cert (seed 10) the pairings + the epoch chain to.
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

/// A REAL strong-chain presentation binding `device_key` to SUBJECT at the observer
/// scope (root sk(1) → intermediate sk(10) → device cert → grant).
fn subject_presentation(device_key: [u8; 32]) -> PairingPresentation {
    let device_cert = DeviceCert {
        version: FORMAT_VERSION,
        device_key: PublicKey(device_key),
        account: SUBJECT,
        principal_kind: PrincipalKind::Human,
        scope: viewer_scope(),
        validity: wide(),
        issued_at_ns: ISSUED,
        issuer_key: pk(&sk(10)),
    }
    .sign(&sk(10));
    let grant = Grant {
        version: FORMAT_VERSION,
        subject: SUBJECT,
        robot: ROBOT,
        scope: viewer_scope(),
        principal_kind: PrincipalKind::Human,
        delegation_depth: 0,
        validity: wide(),
        issued_at_ns: ISSUED,
        issuer: AccountId([2; 32]),
        issuer_key: pk(&sk(10)),
    }
    .sign(&sk(10));
    PairingPresentation {
        intermediate: intermediate_cert(),
        device_cert,
        grant,
        delegation: None,
    }
}

/// A CA-signed epoch (number `n`) revoking `devices`, signed by the intermediate.
fn device_revoke_epoch(n: u64, devices: Vec<PublicKey>) -> SignedEpoch {
    AccessListEpoch {
        version: FORMAT_VERSION,
        robot: ROBOT,
        epoch: n,
        revoked_accounts: vec![],
        revoked_devices: devices,
        issued_at_ns: ISSUED,
        issuer_key: pk(&sk(10)),
    }
    .sign(&sk(10))
}

async fn open_control(conn: &Connection) -> (SendStream, RecvStream) {
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

/// Read `ustream` and require it to deliver `>= min_frames` FRESH, strictly
/// sequence-ADVANCING frames (parsed from the wire header) across a window spanning AT
/// LEAST `min_window` — proving the stream keeps flowing THROUGH the whole window (not
/// merely a single frame that arrived before an eviction). Returns `false` on any
/// stream RESET (a wrongly-evicted survivor) or if the advancing-frame bar is not met.
/// Bounded.
async fn stream_streams_through(
    ustream: &mut RecvStream,
    min_frames: usize,
    min_window: Duration,
) -> bool {
    let start = Instant::now();
    let deadline = start + Duration::from_secs(10);
    let mut seqs: Vec<u32> = Vec::new();
    let met = |seqs: &[u32]| {
        seqs.len() >= min_frames
            && start.elapsed() >= min_window
            && seqs.windows(2).all(|w| w[1] > w[0]) // strictly advancing (no wrap in-test)
    };
    while Instant::now() < deadline {
        match tokio::time::timeout(
            Duration::from_millis(250),
            read_frame(ustream, DEFAULT_MAX_FRAME_LEN),
        )
        .await
        {
            Ok(Ok(bytes)) => {
                if bytes.len() >= WireHeader::SIZE {
                    if let Some(h) = WireHeader::read_from_buf(&bytes[..WireHeader::SIZE]) {
                        seqs.push(h.sequence);
                    }
                }
                if met(&seqs) {
                    return true;
                }
            }
            Ok(Err(_)) => return false, // stream RESET → the survivor was wrongly evicted
            Err(_) => continue,         // no frame this poll; keep going until the window is met
        }
    }
    met(&seqs)
}

fn make_frame(seq: u32) -> Vec<u8> {
    let payload = b"a6-frame";
    let header = WireHeader {
        schema_hash: 0xCE84_4A06_D000_F00D,
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

/// A desk's live wire session: control streams + the demanded uni stream.
struct Desk {
    _conn: Connection,
    _send: SendStream,
    _recv: RecvStream,
    ustream: RecvStream,
}

/// Dial the wire plane as `desk_ep`, demand `topic`, and read ONE healthy pre-revoke
/// frame — returning the live session so the caller can watch the uni stream.
async fn connect_and_demand(
    desk_ep: &cerulion_link::Endpoint,
    robot_addr: EndpointAddr,
    topic: &str,
    who: &str,
) -> Desk {
    let conn = bounded("dial", dial(desk_ep, robot_addr, alpn::WIRE))
        .await
        .unwrap_or_else(|e| panic!("{who} dial wire: {e}"));
    let (mut send, mut recv) = open_control(&conn).await;
    let resp = request(
        &mut send,
        &mut recv,
        &WireRequest::Demand {
            topic: topic.to_string(),
        },
    )
    .await;
    assert_eq!(
        resp,
        WireResponse::DemandAccepted {
            topic: topic.to_string()
        },
        "{who}: the paired CAP_OBSERVE demand is accepted"
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
    bounded("pre frame", read_frame(&mut ustream, DEFAULT_MAX_FRAME_LEN))
        .await
        .unwrap_or_else(|e| panic!("{who} stream healthy pre-revoke: {e}"));
    Desk {
        _conn: conn,
        _send: send,
        _recv: recv,
        ustream,
    }
}

#[tokio::test(flavor = "multi_thread", worker_threads = 6)]
async fn revoking_one_desks_device_evicts_it_and_leaves_the_accounts_other_desk_connected() {
    // ── the robot's LIVE trust: claimed + BOTH desks (same account) established ──
    let robot_ep = disabled_endpoint([0x31; 32]).await;
    let robot_addr = loopback_addr(robot_ep.id(), &robot_ep.bound_sockets());
    let desk_a_ep = disabled_endpoint([0x42; 32]).await;
    let desk_b_ep = disabled_endpoint([0x43; 32]).await;
    let k_a = *desk_a_ep.id().as_bytes();
    let k_b = *desk_b_ep.id().as_bytes();

    let root_set = RootSet::new(vec![pk(&sk(1))], 1).unwrap();
    let mut store = TrustStore::provision(ROBOT, pk(&sk(0x77)), root_set, CHASSIS, 0).unwrap();
    store
        .claim(OWNER, CHASSIS, PrincipalKind::Human, T_NOW)
        .unwrap();
    // Empty MAC key ⇒ in-memory only (no disk); the shared handle is the accept gate,
    // the demand authorizer, AND the apply_epoch seam — all reading ONE live state.
    let shared = SharedTrust::new(store, DeviceAccountIndex::new(), Vec::new());
    shared
        .verify_and_establish(
            &subject_presentation(k_a),
            &k_a,
            Some("desk-A".into()),
            T_NOW,
        )
        .expect("desk A pairs");
    shared
        .verify_and_establish(
            &subject_presentation(k_b),
            &k_b,
            Some("desk-B".into()),
            T_NOW,
        )
        .expect("desk B pairs");

    let authz = Arc::new(PairingAuthorizer::from_shared(shared.clone()));

    // ── the robot's producer + wire plane (short sweep so eviction is prompt) ──
    let manager = test_manager("evict");
    let topic = format!("/wire/a6/{}", unique_id());
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

    let plane = Arc::new(
        WirePlane::with_manager("robot-a6", manager.clone())
            .with_demand_authorizer(authz.clone())
            .with_revocation_sweep_interval(Duration::from_millis(200)),
    );

    // ── the robot's accept loop: one served connection per desk ──
    let accept_task = {
        let authz = authz.clone();
        let plane = plane.clone();
        tokio::spawn(async move {
            while let Ok(Some(accepted)) = accept_one(&robot_ep).await {
                let a = authz.clone();
                let p = plane.clone();
                tokio::spawn(async move {
                    handle_accepted_with_wire(accepted, a, None, Some(p)).await;
                });
            }
        })
    };

    // ── both desks connect + stream (healthy pre-revoke) ──
    let mut desk_a = connect_and_demand(&desk_a_ep, robot_addr.clone(), &topic, "desk-A").await;
    let mut desk_b = connect_and_demand(&desk_b_ep, robot_addr.clone(), &topic, "desk-B").await;

    // ── REVOKE desk A's device via the PRODUCTION sync seam (a CA-signed epoch) ──
    // The account is NOT revoked; only desk A's DEVICE key is.
    let outcome = shared
        .apply_epoch(
            &device_revoke_epoch(2, vec![PublicKey(k_a)]),
            &intermediate_cert(),
            T_NOW,
        )
        .expect("the device-revocation epoch applies");
    assert!(matches!(
        outcome,
        cerulion_pairing::verify::EpochOutcome::Applied { .. }
    ));

    // ── desk A is EVICTED; desk B streams THROUGH the whole window ──
    // Watch both concurrently: A's uni stream must RESET (the sweep re-checks
    // authorize_demand → DeviceRevoked → closes A's connection), while B must keep
    // delivering >= 5 strictly sequence-ADVANCING frames across a window spanning
    // several sweep intervals (the sweep runs every 200ms). Requiring B to stream
    // THROUGH >= 3 sweep cycles catches an OVER-BROAD eviction that would kill B a
    // moment later (on a subsequent sweep tick) — a first-frame-only check would miss it.
    const SWEEP: Duration = Duration::from_millis(200);
    let (a_evicted, b_through) = tokio::join!(
        read_until_reset(&mut desk_a.ustream),
        stream_streams_through(&mut desk_b.ustream, 5, SWEEP * 3)
    );
    assert!(
        a_evicted,
        "desk A's device was revoked → its stream MUST be evicted (the revocation sweep over \
         the A6 device-revocation epoch)"
    );
    assert!(
        b_through,
        "desk B (the SAME account, a DIFFERENT device) MUST keep streaming ADVANCING \
         frames THROUGH >= 3 sweep cycles after the epoch — the revocation is surgical, \
         not a global account lockout, and not an eviction that lands a beat later"
    );

    // Anti-tautology on the ACCEPT side: a fresh DIAL from desk A's revoked key is now
    // REFUSED at the wire accept gate — the plane serves the skeleton `AcceptDecision`
    // (Refuse), NEVER the wire protocol, so a demand can never be re-admitted. (The
    // control-stream reply is the accept-decision JSON, not a `WireResponse`.)
    let a_redial = dial(&desk_a_ep, robot_addr.clone(), alpn::WIRE).await;
    if let Ok(conn) = a_redial {
        if let Ok((mut s, mut r)) = open_frame_stream(&conn).await {
            let req = serde_json::to_vec(&WireRequest::Demand {
                topic: topic.clone(),
            })
            .unwrap();
            let _ = bounded("redial write", write_frame(&mut s, &req)).await;
            if let Ok(bytes) =
                bounded("redial read", read_frame(&mut r, DEFAULT_MAX_FRAME_LEN)).await
            {
                let v: serde_json::Value = serde_json::from_slice(&bytes).unwrap_or_default();
                // A refused wire peer gets an AcceptDecision (`decision: "refuse"`),
                // never a WireResponse (`reply: "demand_accepted"`).
                assert_ne!(
                    v.get("reply").and_then(|r| r.as_str()),
                    Some("demand_accepted"),
                    "a revoked device must never be re-admitted to demand (got {v})"
                );
            }
        }
    }

    // ── teardown ──
    drop(desk_a);
    drop(desk_b);
    accept_task.abort();
    let _ = accept_task.await;
    stop.store(true, Ordering::Relaxed);
    let _ = producer.join();
}
