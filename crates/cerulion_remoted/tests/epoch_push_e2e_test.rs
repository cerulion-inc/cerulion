// SPDX-License-Identifier: AGPL-3.0-only
//! THE desk-push revocation-epoch e2e, driven through the PRODUCTION wire
//! plane (`cerulion/wire/1`), the PRODUCTION sync seam (`SharedTrust::apply_epoch`),
//! the PRODUCTION accept gate (`PairingAuthorizer`) and the eviction sweep.
//! No inert shipping: every assertion goes through the real `sync_epoch` verb over a
//! real loopback iroh connection.
//!
//! By design: a robot learns about a revocation because the desks that talk to
//! it PUSH the latest signed epoch on connect. These tests pin that the push actually
//! lands, that a stale one cannot roll the robot back, that a desk carrying its OWN
//! revocation still delivers it faithfully (and is then evicted), and that NOTHING about
//! the push can block a connection.
//!
//! Every credential is REAL cryptography (deterministic fixed-seed keys — Principle
//! #13); every epoch is a genuine `SignedEpoch`. Oracles are hand-written: the expected
//! `(epoch, applied)` pair and the expected `KeyAccess` for a probe key are stated
//! independently of what the code returns, so no assertion is a self-compare.
//!
//! Per-test loopback endpoints + a per-test iceoryx2 SHM root ⇒ parallel-safe (no
//! `#[serial]`).

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
use cerulion_pairing::verify::{EpochSyncWire, PairingPresentation, TrustStore};
use ed25519_dalek::SigningKey;

use cerulion_remoted::{
    handle_accepted_with_wire, DeviceAccountIndex, KeyAccess, PairingAuthorizer, RemotedClock,
    SharedTrust, StreamPreamble, WirePlane, WireRequest, WireResponse,
};

const T_NOW: u64 = 1_000_000_000_000;
const ISSUED: u64 = 500_000_000_000;
const CHASSIS: &[u8] = b"epoch-push-chassis-secret-not-serial-derived";
const OWNER: AccountId = AccountId([10; 32]);
/// The account both desks belong to (so a DEVICE revocation is provably surgical).
const SUBJECT: AccountId = AccountId([0x5C; 32]);
/// A third account used purely as a REVOCATION PROBE: never dials, so its
/// `KeyAccess` transition is a clean observable of "the pushed epoch was applied".
const PROBE: AccountId = AccountId([0x77; 32]);
const ROBOT: RobotId = RobotId([0x0B; 32]);
/// A DIFFERENT robot id — for the wrong-robot refusal arm.
const OTHER_ROBOT: RobotId = RobotId([0xB0; 32]);

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
        Err(_) => panic!("epoch-push e2e: '{what}' timed out"),
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
            node_name: format!("{tag}_{}", unique_id()),
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

/// The root-signed intermediate the pairings + every epoch chain to.
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

/// An intermediate the robot's root set does NOT trust (root sk(9), not sk(1)) — the
/// forged-anchor arm.
fn untrusted_intermediate() -> SignedIntermediateCert {
    IntermediateCert {
        version: FORMAT_VERSION,
        intermediate_key: pk(&sk(11)),
        validity: wide(),
        issued_at_ns: ISSUED,
        max_scope: Scope::OWNER_FULL,
    }
    .sign_by_roots(&[&sk(9)])
}

/// A REAL strong-chain presentation binding `device_key` to `account` at the observer
/// scope (root sk(1) → intermediate sk(10) → device cert → grant).
fn presentation_for(device_key: [u8; 32], account: AccountId) -> PairingPresentation {
    let device_cert = DeviceCert {
        version: FORMAT_VERSION,
        device_key: PublicKey(device_key),
        account,
        principal_kind: PrincipalKind::Human,
        scope: viewer_scope(),
        validity: wide(),
        issued_at_ns: ISSUED,
        issuer_key: pk(&sk(10)),
    }
    .sign(&sk(10));
    let grant = Grant {
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
    .sign(&sk(10));
    PairingPresentation {
        intermediate: intermediate_cert(),
        device_cert,
        grant,
        delegation: None,
    }
}

/// Build a genuine `SignedEpoch` for `robot`, signed by `signer`.
fn epoch_signed_by(
    robot: RobotId,
    n: u64,
    accounts: Vec<AccountId>,
    devices: Vec<PublicKey>,
    signer: &SigningKey,
) -> SignedEpoch {
    AccessListEpoch {
        version: FORMAT_VERSION,
        robot,
        epoch: n,
        revoked_accounts: accounts,
        revoked_devices: devices,
        issued_at_ns: ISSUED,
        issuer_key: pk(signer),
    }
    .sign(signer)
}

/// The healthy epoch shape: for THIS robot, signed by the trusted intermediate sk(10).
fn epoch(n: u64, accounts: Vec<AccountId>, devices: Vec<PublicKey>) -> SignedEpoch {
    epoch_signed_by(ROBOT, n, accounts, devices, &sk(10))
}

/// The `epoch_postcard` hex blob a desk hands the `sync_epoch` verb — built through the
/// SHARED wire shape, exactly as `cerulion_wireclient::resolve_epoch_sync` does.
fn push_blob(signed_epoch: SignedEpoch, intermediate: SignedIntermediateCert) -> String {
    hex::encode(
        EpochSyncWire::new(intermediate, signed_epoch)
            .to_postcard()
            .expect("encode the epoch artifact"),
    )
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

/// The bounded NEGATIVE twin of `read_until_reset`: drain `ustream` for the whole
/// `window` and report `(frames_read, reset)`. A stream that SURVIVED the window ends
/// with `reset == false` AND a nonzero frame count — the frame count is what stops a
/// silently-dead-but-un-reset stream from passing as healthy.
async fn read_for(ustream: &mut RecvStream, window: Duration) -> (usize, bool) {
    let deadline = Instant::now() + window;
    let mut frames = 0usize;
    while Instant::now() < deadline {
        match tokio::time::timeout(
            Duration::from_millis(100),
            read_frame(ustream, DEFAULT_MAX_FRAME_LEN),
        )
        .await
        {
            Ok(Ok(_)) => frames += 1,
            Ok(Err(_)) => return (frames, true),
            Err(_) => continue,
        }
    }
    (frames, false)
}

fn make_frame(seq: u32) -> Vec<u8> {
    let payload = b"frame";
    let header = WireHeader {
        schema_hash: 0xCE88_0000_D000_F00D,
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

/// One robot under test: its live trust handle, its served wire plane, and the address
/// desks dial. `epoch_sink` controls whether `with_epoch_sink` is installed (the
/// no-sink arm needs it OFF).
struct Robot {
    shared: SharedTrust,
    addr: EndpointAddr,
    topic: String,
    stop: Arc<AtomicBool>,
    producer: Option<std::thread::JoinHandle<()>>,
    accept: tokio::task::JoinHandle<()>,
}

impl Robot {
    /// Provision a claimed robot, pair each `(device_key, account)`, start a real SHM
    /// producer + the served wire plane, and begin accepting.
    async fn start(
        tag: &str,
        robot_secret: [u8; 32],
        pairings: &[([u8; 32], AccountId, &str)],
        epoch_sink: bool,
        sweep: Duration,
    ) -> Robot {
        let robot_ep = disabled_endpoint(robot_secret).await;
        let addr = loopback_addr(robot_ep.id(), &robot_ep.bound_sockets());

        let root_set = RootSet::new(vec![pk(&sk(1))], 1).unwrap();
        let mut store = TrustStore::provision(ROBOT, pk(&sk(0x77)), root_set, CHASSIS, 0).unwrap();
        store
            .claim(OWNER, CHASSIS, PrincipalKind::Human, T_NOW)
            .unwrap();
        // Empty MAC key ⇒ in-memory only (no disk). The ONE handle is the accept gate,
        // the demand authorizer, AND the epoch sink — the production arrangement.
        let shared = SharedTrust::new(store, DeviceAccountIndex::new(), Vec::new());
        for (key, account, name) in pairings {
            shared
                .verify_and_establish(
                    &presentation_for(*key, *account),
                    key,
                    Some((*name).to_string()),
                    T_NOW,
                )
                .unwrap_or_else(|e| panic!("{name} pairs: {e}"));
        }

        let authz = Arc::new(PairingAuthorizer::from_shared(shared.clone()));
        let manager = test_manager(tag);
        let topic = format!("/wire/epoch/{}", unique_id());
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

        let mut plane = WirePlane::with_manager(format!("robot-{tag}"), manager.clone())
            .with_demand_authorizer(authz.clone())
            .with_revocation_sweep_interval(sweep);
        if epoch_sink {
            // The production wiring: the SAME live handle the accept gate reads.
            plane = plane.with_epoch_sink(shared.clone(), RemotedClock::fixed(T_NOW));
        }
        let plane = Arc::new(plane);

        let accept = {
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

        Robot {
            shared,
            addr,
            topic,
            stop,
            producer: Some(producer),
            accept,
        }
    }

    /// The PROBE account's live access state — the observable that says whether a
    /// pushed epoch actually took effect in the robot's trust store.
    fn probe_access(&self, probe_key: &[u8; 32]) -> KeyAccess {
        self.shared.snapshot_for_key(probe_key).1
    }

    fn shutdown(mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(p) = self.producer.take() {
            let _ = p.join();
        }
        self.accept.abort();
    }
}

/// A desk's live wire session.
struct Desk {
    conn: Connection,
    send: SendStream,
    recv: RecvStream,
    ustream: Option<RecvStream>,
}

/// Dial the wire plane and open its control stream (no demand yet).
async fn connect(desk_ep: &cerulion_link::Endpoint, addr: EndpointAddr, who: &str) -> Desk {
    let conn = bounded("dial", dial(desk_ep, addr, alpn::WIRE))
        .await
        .unwrap_or_else(|e| panic!("{who} dial wire: {e}"));
    let (send, recv) = open_control(&conn).await;
    Desk {
        conn,
        send,
        recv,
        ustream: None,
    }
}

impl Desk {
    async fn push(&mut self, blob: String) -> WireResponse {
        request(
            &mut self.send,
            &mut self.recv,
            &WireRequest::SyncEpoch {
                epoch_postcard: blob,
            },
        )
        .await
    }

    /// Demand `topic` + accept its uni stream, asserting the healthy path.
    async fn demand(&mut self, topic: &str, who: &str) {
        let resp = request(
            &mut self.send,
            &mut self.recv,
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
        let mut ustream = bounded("accept_uni", accept_uni_frame_stream(&self.conn))
            .await
            .expect("uni");
        let preamble: StreamPreamble = serde_json::from_slice(
            &bounded("preamble", read_frame(&mut ustream, DEFAULT_MAX_FRAME_LEN))
                .await
                .expect("preamble"),
        )
        .unwrap();
        assert_eq!(preamble.topic, topic);
        bounded(
            "first frame",
            read_frame(&mut ustream, DEFAULT_MAX_FRAME_LEN),
        )
        .await
        .unwrap_or_else(|e| panic!("{who} stream healthy: {e}"));
        self.ustream = Some(ustream);
    }
}

/// Assert a response is a loud verb-level error whose message contains `needle` AND
/// says the epoch did not apply. Never a silent success.
fn assert_refused(resp: &WireResponse, needle: &str) {
    match resp {
        WireResponse::Error { topic, message } => {
            assert_eq!(*topic, None, "an epoch refusal is not topic-scoped");
            assert!(
                message.contains(needle),
                "the refusal must name the cause '{needle}': {message}"
            );
            assert!(
                message.contains("NOT applied"),
                "the refusal must state the epoch did not land: {message}"
            );
        }
        other => panic!("expected a loud Error refusal, got {other:?}"),
    }
}

// ───────────────────────────────────────────────────────────────────────────────
// 1. THE HEADLINE: an epoch travels desk → robot on connect and takes effect.
// ───────────────────────────────────────────────────────────────────────────────

#[tokio::test(flavor = "multi_thread", worker_threads = 6)]
async fn a_desk_pushes_an_epoch_on_connect_and_the_robot_applies_it() {
    let desk_ep = disabled_endpoint([0x42; 32]).await;
    let k_desk = *desk_ep.id().as_bytes();
    // The PROBE desk pairs but never dials: its access transition is the clean
    // observable of "the pushed epoch reached the robot's trust store".
    let probe_ep = disabled_endpoint([0x44; 32]).await;
    let k_probe = *probe_ep.id().as_bytes();

    let robot = Robot::start(
        "headline",
        [0x31; 32],
        &[(k_desk, SUBJECT, "desk"), (k_probe, PROBE, "probe")],
        true,
        Duration::from_millis(200),
    )
    .await;

    // HAND ORACLE, pre-push: the probe account is ALLOWED at the observer scope.
    assert_eq!(
        robot.probe_access(&k_probe),
        KeyAccess::Allowed(viewer_scope()),
        "pre-condition: the probe account is on the access list"
    );

    // The desk connects and PUSHES epoch 5, which revokes the PROBE account.
    let mut desk = connect(&desk_ep, robot.addr.clone(), "desk").await;
    let resp = desk
        .push(push_blob(
            epoch(5, vec![PROBE], vec![]),
            intermediate_cert(),
        ))
        .await;

    // HAND ORACLE: the robot reports epoch 5, applied.
    assert_eq!(
        resp,
        WireResponse::EpochSynced {
            epoch: 5,
            applied: true
        },
        "a newer epoch pushed over the wire plane must be APPLIED"
    );

    // ...and it REALLY took effect in the live trust the accept gate reads — the
    // revocation is observable, not just reported.
    assert_eq!(
        robot.probe_access(&k_probe),
        KeyAccess::NotAllowed,
        "the pushed epoch must flip the PROBE account's live access state"
    );
    // The pusher itself is untouched (the epoch named only PROBE).
    assert_eq!(
        robot.probe_access(&k_desk),
        KeyAccess::Allowed(viewer_scope()),
        "the epoch revoked only PROBE — the pushing desk keeps its access"
    );

    robot.shutdown();
}

// ───────────────────────────────────────────────────────────────────────────────
// 2. A STALE push is a harmless no-op and can NEVER roll the robot back.
// ───────────────────────────────────────────────────────────────────────────────

#[tokio::test(flavor = "multi_thread", worker_threads = 6)]
async fn a_stale_epoch_push_is_a_noop_and_never_un_revokes() {
    let desk_ep = disabled_endpoint([0x42; 32]).await;
    let k_desk = *desk_ep.id().as_bytes();
    let probe_ep = disabled_endpoint([0x44; 32]).await;
    let k_probe = *probe_ep.id().as_bytes();

    let robot = Robot::start(
        "stale",
        [0x32; 32],
        &[(k_desk, SUBJECT, "desk"), (k_probe, PROBE, "probe")],
        true,
        Duration::from_millis(200),
    )
    .await;

    let mut desk = connect(&desk_ep, robot.addr.clone(), "desk").await;
    // Land epoch 7 revoking PROBE.
    assert_eq!(
        desk.push(push_blob(
            epoch(7, vec![PROBE], vec![]),
            intermediate_cert()
        ))
        .await,
        WireResponse::EpochSynced {
            epoch: 7,
            applied: true
        }
    );
    assert_eq!(robot.probe_access(&k_probe), KeyAccess::NotAllowed);

    // A LOWER epoch whose set is EMPTY — the rollback attempt a stale desk cache
    // would make. It must be a reported no-op AND leave the revocation standing.
    assert_eq!(
        desk.push(push_blob(epoch(3, vec![], vec![]), intermediate_cert()))
            .await,
        WireResponse::EpochSynced {
            epoch: 7,
            applied: false
        },
        "a lower epoch is NOT applied and the CURRENT epoch is reported"
    );
    assert_eq!(
        robot.probe_access(&k_probe),
        KeyAccess::NotAllowed,
        "a stale push must NEVER un-revoke (the monotonic floor holds)"
    );

    // The SAME epoch again (the steady state — every desk pushes its cache on every
    // connect) is likewise a no-op, not an error.
    assert_eq!(
        desk.push(push_blob(
            epoch(7, vec![PROBE], vec![]),
            intermediate_cert()
        ))
        .await,
        WireResponse::EpochSynced {
            epoch: 7,
            applied: false
        },
        "re-pushing the current epoch is an idempotent no-op"
    );
    assert_eq!(robot.probe_access(&k_probe), KeyAccess::NotAllowed);

    robot.shutdown();
}

// ───────────────────────────────────────────────────────────────────────────────
// 3. THE ISSUE'S HEADLINE CORRECTNESS CASE: a revoked desk pushing the very epoch
//    that cuts it off is FINE — it delivers faithfully, then the sweep evicts it.
// ───────────────────────────────────────────────────────────────────────────────

#[tokio::test(flavor = "multi_thread", worker_threads = 6)]
async fn a_desk_pushing_its_own_revocation_delivers_it_then_is_evicted() {
    let desk_ep = disabled_endpoint([0x42; 32]).await;
    let k_desk = *desk_ep.id().as_bytes();
    // A SIBLING desk on the SAME account: it must NOT be collateral damage (the
    // revocation is per-DEVICE), which also proves the eviction is surgical.
    let sib_ep = disabled_endpoint([0x43; 32]).await;
    let k_sib = *sib_ep.id().as_bytes();

    const SWEEP: Duration = Duration::from_millis(200);
    let robot = Robot::start(
        "selfrevoke",
        [0x33; 32],
        &[(k_desk, SUBJECT, "desk-A"), (k_sib, SUBJECT, "desk-B")],
        true,
        SWEEP,
    )
    .await;

    // Both desks connect + stream healthily.
    let mut desk_a = connect(&desk_ep, robot.addr.clone(), "desk-A").await;
    desk_a.demand(&robot.topic, "desk-A").await;
    let mut desk_b = connect(&sib_ep, robot.addr.clone(), "desk-B").await;
    desk_b.demand(&robot.topic, "desk-B").await;

    // Desk A pushes the epoch revoking ITS OWN device key. The robot has not synced
    // yet, so A is admitted; delivering the epoch that cuts it off is correct.
    let resp = desk_a
        .push(push_blob(
            epoch(4, vec![], vec![PublicKey(k_desk)]),
            intermediate_cert(),
        ))
        .await;
    assert_eq!(
        resp,
        WireResponse::EpochSynced {
            epoch: 4,
            applied: true
        },
        "a desk carrying its OWN revocation still delivers it — and is told it applied"
    );

    // Its own access state has flipped...
    assert_eq!(
        robot.probe_access(&k_desk),
        KeyAccess::DeviceRevoked,
        "the pusher's DEVICE key is now epoch-revoked (its account is untouched)"
    );
    // ...the sibling on the SAME account is NOT cut (surgical, per-device).
    assert_eq!(
        robot.probe_access(&k_sib),
        KeyAccess::Allowed(viewer_scope()),
        "the SAME account's other device keeps access — a device revocation is surgical"
    );

    // ...and the sweep EVICTS desk A's live stream.
    let mut a_stream = desk_a.ustream.take().expect("desk A demanded");
    assert!(
        read_until_reset(&mut a_stream).await,
        "the self-pushed device revocation MUST evict desk A's live stream (the revocation sweep)"
    );

    // The SURGICAL claim is about the DATA PLANE, and the access-state assert above
    // cannot carry it: a buggy sweep that tore down BOTH desks' streams while leaving
    // the access tables correct would pass every assert so far. So read desk B's LIVE
    // stream AFTER desk A is provably evicted, across 3 full sweep periods (so the
    // sweep runs repeatedly inside the window with the revocation already in force):
    // a reset lands as `reset == true`, and the frame count refuses a stream that
    // merely went quiet.
    let mut b_stream = desk_b.ustream.take().expect("desk B demanded");
    let (b_frames, b_reset) = read_for(&mut b_stream, SWEEP * 3).await;
    assert!(
        !b_reset && b_frames > 0,
        "the SIBLING device's LIVE stream must SURVIVE desk A's device revocation — the \
         sweep eviction is per-DEVICE, not a plane-wide cut (desk B read {b_frames} \
         frame(s) over 3 sweep periods, reset={b_reset})"
    );

    robot.shutdown();
}

// ───────────────────────────────────────────────────────────────────────────────
// 4. NOTHING about the push can block a connection.
// ───────────────────────────────────────────────────────────────────────────────

#[tokio::test(flavor = "multi_thread", worker_threads = 6)]
async fn a_desk_that_pushes_no_epoch_connects_and_streams_normally() {
    // The no-cache path (a never-synced desk, and EVERY guest desk — the account
    // service's access endpoint is owner-only) plus the accountd-unreachable path:
    // both resolve to "push nothing" desk-side, so on the wire they are the SAME
    // behavior — a connection with no `sync_epoch` frame at all.
    let desk_ep = disabled_endpoint([0x42; 32]).await;
    let k_desk = *desk_ep.id().as_bytes();
    let robot = Robot::start(
        "nopush",
        [0x34; 32],
        &[(k_desk, SUBJECT, "desk")],
        true,
        Duration::from_millis(200),
    )
    .await;

    let mut desk = connect(&desk_ep, robot.addr.clone(), "desk").await;
    desk.demand(&robot.topic, "desk").await;
    // Access is untouched and the stream is live — the epoch plane is inert when the
    // desk carries nothing.
    assert_eq!(
        robot.probe_access(&k_desk),
        KeyAccess::Allowed(viewer_scope()),
        "a desk that pushes nothing changes nothing"
    );
    assert!(desk.ustream.is_some(), "and it streams normally");

    robot.shutdown();
}

#[tokio::test(flavor = "multi_thread", worker_threads = 6)]
async fn a_refused_push_leaves_the_connection_fully_usable() {
    // The "never block the connection on epoch freshness" contract at the wire level:
    // after a REFUSED push the SAME control stream must still serve a demand. This is
    // what lets netd log-and-continue instead of failing the dial.
    let desk_ep = disabled_endpoint([0x42; 32]).await;
    let k_desk = *desk_ep.id().as_bytes();
    let robot = Robot::start(
        "usable",
        [0x35; 32],
        &[(k_desk, SUBJECT, "desk")],
        true,
        Duration::from_millis(200),
    )
    .await;

    let mut desk = connect(&desk_ep, robot.addr.clone(), "desk").await;
    // A forged epoch (signed by an untrusted key) → refused.
    let resp = desk
        .push(push_blob(
            epoch_signed_by(ROBOT, 9, vec![PROBE], vec![], &sk(66)),
            intermediate_cert(),
        ))
        .await;
    assert_refused(&resp, "REJECTED");
    // The connection still works: a demand on the SAME stream succeeds.
    desk.demand(&robot.topic, "desk").await;
    assert!(
        desk.ustream.is_some(),
        "a refused epoch push must NOT break the connection"
    );

    robot.shutdown();
}

// ───────────────────────────────────────────────────────────────────────────────
// 5. Every REJECTION path is loud, and none of them mutates state.
// ───────────────────────────────────────────────────────────────────────────────

#[tokio::test(flavor = "multi_thread", worker_threads = 6)]
async fn forged_wrong_robot_and_malformed_pushes_are_refused_loudly_with_no_state_change() {
    let desk_ep = disabled_endpoint([0x42; 32]).await;
    let k_desk = *desk_ep.id().as_bytes();
    let probe_ep = disabled_endpoint([0x44; 32]).await;
    let k_probe = *probe_ep.id().as_bytes();

    let robot = Robot::start(
        "refuse",
        [0x36; 32],
        &[(k_desk, SUBJECT, "desk"), (k_probe, PROBE, "probe")],
        true,
        Duration::from_millis(200),
    )
    .await;
    let mut desk = connect(&desk_ep, robot.addr.clone(), "desk").await;

    // (a) A BAD SIGNATURE: the epoch claims the trusted issuer key but is signed by a
    //     different key. This is the arm that proves the desk cannot forge policy.
    let mut forged = epoch(9, vec![PROBE], vec![]);
    forged.signature = epoch_signed_by(ROBOT, 9, vec![PROBE], vec![], &sk(66)).signature;
    assert_refused(
        &desk.push(push_blob(forged, intermediate_cert())).await,
        "REJECTED",
    );

    // (b) An epoch for a DIFFERENT robot must never apply here.
    assert_refused(
        &desk
            .push(push_blob(
                epoch_signed_by(OTHER_ROBOT, 9, vec![PROBE], vec![], &sk(10)),
                intermediate_cert(),
            ))
            .await,
        "DIFFERENT robot",
    );

    // (c) An UNTRUSTED intermediate (chained to a root the robot does not know) — the
    //     desk is a courier, not a trust anchor.
    assert_refused(
        &desk
            .push(push_blob(
                epoch_signed_by(ROBOT, 9, vec![PROBE], vec![], &sk(11)),
                untrusted_intermediate(),
            ))
            .await,
        "root set does not trust",
    );

    // (d) Not hex.
    assert_refused(
        &desk.push("zzzz-not-hex".to_string()).await,
        "not valid hex",
    );

    // (e) Valid hex whose bytes are not an artifact.
    assert_refused(
        &desk.push(hex::encode(b"not an artifact")).await,
        "did not decode",
    );

    // (f) An EMPTY blob (the degenerate case a bug could produce) is refused, not
    //     treated as a default artifact.
    assert_refused(&desk.push(String::new()).await, "did not decode");

    // NO state change across every rejection: the probe is still allowed and nothing
    // was applied.
    assert_eq!(
        robot.probe_access(&k_probe),
        KeyAccess::Allowed(viewer_scope()),
        "not one refused push may mutate the access list"
    );
    // ...and a subsequent VALID push still works (the refusals left no poison).
    assert_eq!(
        desk.push(push_blob(
            epoch(9, vec![PROBE], vec![]),
            intermediate_cert()
        ))
        .await,
        WireResponse::EpochSynced {
            epoch: 9,
            applied: true
        },
        "after the refusals a genuine epoch still applies"
    );
    assert_eq!(robot.probe_access(&k_probe), KeyAccess::NotAllowed);

    robot.shutdown();
}

// ───────────────────────────────────────────────────────────────────────────────
// 6. A plane with NO sink installed refuses LOUDLY (never a phantom success).
// ───────────────────────────────────────────────────────────────────────────────

#[tokio::test(flavor = "multi_thread", worker_threads = 6)]
async fn a_plane_without_an_epoch_sink_refuses_the_push_loudly() {
    let desk_ep = disabled_endpoint([0x42; 32]).await;
    let k_desk = *desk_ep.id().as_bytes();
    let probe_ep = disabled_endpoint([0x44; 32]).await;
    let k_probe = *probe_ep.id().as_bytes();

    // epoch_sink = FALSE: the anti-tautology control for every test above. A desk must
    // be TOLD the robot cannot sync, never be allowed to believe a revocation landed.
    let robot = Robot::start(
        "nosink",
        [0x37; 32],
        &[(k_desk, SUBJECT, "desk"), (k_probe, PROBE, "probe")],
        false,
        Duration::from_millis(200),
    )
    .await;

    let mut desk = connect(&desk_ep, robot.addr.clone(), "desk").await;
    let resp = desk
        .push(push_blob(
            epoch(5, vec![PROBE], vec![]),
            intermediate_cert(),
        ))
        .await;
    assert_refused(&resp, "does not accept access-list epoch sync");
    // A perfectly VALID epoch changed nothing — proving the sink (not the epoch) is
    // what makes the verb real, and that the tests above are not passing by accident.
    assert_eq!(
        robot.probe_access(&k_probe),
        KeyAccess::Allowed(viewer_scope()),
        "with no sink installed a valid epoch must NOT be applied"
    );
    // The connection is still usable (a sink-less robot is not a broken robot).
    desk.demand(&robot.topic, "desk").await;
    assert!(desk.ustream.is_some());

    robot.shutdown();
}

// ───────────────────────────────────────────────────────────────────────────────
// 7. Determinism: the same push sequence yields the same outcomes (Principle #7).
// ───────────────────────────────────────────────────────────────────────────────

#[tokio::test(flavor = "multi_thread", worker_threads = 6)]
async fn the_push_outcome_sequence_is_deterministic_across_two_robots() {
    // Two independently provisioned robots, the SAME push sequence, byte-identical
    // outcome vectors — AND both equal a HAND oracle, so this is not a self-compare.
    let oracle = vec![
        WireResponse::EpochSynced {
            epoch: 2,
            applied: true,
        },
        WireResponse::EpochSynced {
            epoch: 2,
            applied: false,
        },
        WireResponse::EpochSynced {
            epoch: 6,
            applied: true,
        },
        WireResponse::EpochSynced {
            epoch: 6,
            applied: false,
        },
    ];
    let mut runs = Vec::new();
    for (i, secret) in [[0x38u8; 32], [0x39u8; 32]].into_iter().enumerate() {
        let desk_ep = disabled_endpoint([0x42 + i as u8; 32]).await;
        let k_desk = *desk_ep.id().as_bytes();
        let robot = Robot::start(
            &format!("determ{i}"),
            secret,
            &[(k_desk, SUBJECT, "desk")],
            true,
            Duration::from_millis(200),
        )
        .await;
        let mut desk = connect(&desk_ep, robot.addr.clone(), "desk").await;
        let mut out = Vec::new();
        for (n, accounts) in [
            (2u64, vec![PROBE]),
            (1, vec![]),
            (6, vec![PROBE]),
            (6, vec![PROBE]),
        ] {
            out.push(
                desk.push(push_blob(epoch(n, accounts, vec![]), intermediate_cert()))
                    .await,
            );
        }
        runs.push(out);
        robot.shutdown();
    }
    assert_eq!(runs[0], oracle, "run 0 must equal the hand oracle");
    assert_eq!(runs[1], oracle, "run 1 must equal the hand oracle");
}

// ───────────────────────────────────────────────────────────────────────────────
// THE PERSIST-REPORTING ARM + the older-peer marker, over the REAL wire.
// ───────────────────────────────────────────────────────────────────────────────

/// Send RAW control bytes (not a typed `WireRequest`) and read the reply — the only
/// way to make the REAL robot see a verb it does not know.
async fn request_raw(send: &mut SendStream, recv: &mut RecvStream, bytes: &[u8]) -> WireResponse {
    bounded("write raw request", write_frame(send, bytes))
        .await
        .expect("write raw request");
    let resp = bounded("read response", read_frame(recv, DEFAULT_MAX_FRAME_LEN))
        .await
        .expect("read response");
    serde_json::from_slice(&resp).expect("decode WireResponse")
}

/// A robot whose trust store PERSISTS to `store_path` (a non-empty MAC key), so a
/// later push actually reaches `SharedTrust::apply_epoch`'s save. Returns the live
/// handle + the address desks dial. Deliberately slim (no producer/topic): the
/// persist arm needs only the control plane.
async fn persisting_robot(
    secret: [u8; 32],
    store_path: &std::path::Path,
    index_path: &std::path::Path,
    pairings: &[([u8; 32], AccountId)],
) -> (SharedTrust, EndpointAddr, tokio::task::JoinHandle<()>) {
    let robot_ep = disabled_endpoint(secret).await;
    let addr = loopback_addr(robot_ep.id(), &robot_ep.bound_sockets());

    let root_set = RootSet::new(vec![pk(&sk(1))], 1).unwrap();
    let mut store = TrustStore::provision(ROBOT, pk(&sk(0x77)), root_set, CHASSIS, 0)
        .unwrap()
        .with_path(store_path);
    store
        .claim(OWNER, CHASSIS, PrincipalKind::Human, T_NOW)
        .unwrap();
    // A NON-EMPTY MAC key ⇒ mutations persist (the production shape). The pairings
    // below therefore write to `store_path`, which must exist NOW.
    let shared = SharedTrust::new(
        store,
        DeviceAccountIndex::new().with_path(index_path),
        b"mac-key".to_vec(),
    );
    for (key, account) in pairings {
        shared
            .verify_and_establish(&presentation_for(*key, *account), key, None, T_NOW)
            .expect("pairs");
    }

    let authz = Arc::new(PairingAuthorizer::from_shared(shared.clone()));
    let plane = Arc::new(
        WirePlane::with_manager("persist-robot", test_manager("persist"))
            .with_demand_authorizer(authz.clone())
            .with_epoch_sink(shared.clone(), RemotedClock::fixed(T_NOW)),
    );
    let accept = tokio::spawn(async move {
        while let Ok(Some(accepted)) = accept_one(&robot_ep).await {
            let a = authz.clone();
            let p = plane.clone();
            tokio::spawn(async move {
                handle_accepted_with_wire(accepted, a, None, Some(p)).await;
            });
        }
    });
    (shared, addr, accept)
}

/// A push that APPLIES in memory but cannot be PERSISTED is reported to the desk as
/// `applied: true` — because it IS applied and IS enforcing right now; it just will
/// not survive a reboot.
///
/// Reporting it as a rejection would be false on both clauses and would send an
/// operator hunting a forged epoch instead of a failing disk. The durability alarm is
/// the ROBOT's own problem and screams robot-side (an `error!`) where the disk is.
///
/// This is the arm every other test short-circuits: they all use an empty MAC key, so
/// `apply_epoch` returns before the save. Here the store persists for real, and its
/// directory is REMOVED after pairing so the save fails with NotFound (portable — no
/// permission games, and never root-defeatable).
#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn a_persist_failure_is_reported_as_applied_and_is_really_enforcing() {
    let desk_ep = disabled_endpoint([0x5A; 32]).await;
    let k_desk = *desk_ep.id().as_bytes();
    let probe_ep = disabled_endpoint([0x5B; 32]).await;
    let k_probe = *probe_ep.id().as_bytes();

    let dir = tempfile::tempdir().expect("tempdir");
    let store_dir = dir.path().join("store");
    std::fs::create_dir_all(&store_dir).expect("create the store dir");
    let store_path = store_dir.join("trust_store");

    let (shared, addr, accept) = persisting_robot(
        [0x5C; 32],
        &store_path,
        &store_dir.join("device_index"),
        &[(k_desk, SUBJECT), (k_probe, PROBE)],
    )
    .await;

    // Precondition: the pairings persisted (so the store path really was live) and
    // the probe is allowed.
    assert!(
        store_path.exists(),
        "precondition: persistence is genuinely wired (the pairings wrote the store)"
    );
    assert_eq!(
        shared.snapshot_for_key(&k_probe).1,
        KeyAccess::Allowed(viewer_scope())
    );

    // DOOM the persistence: remove the directory the store writes into. Every later
    // save fails with NotFound — a real, portable I/O failure.
    std::fs::remove_dir_all(&store_dir).expect("remove the store dir");

    let mut desk = connect(&desk_ep, addr, "desk").await;
    let resp = desk
        .push(push_blob(
            epoch(4, vec![PROBE], vec![]),
            intermediate_cert(),
        ))
        .await;

    // (1) The DESK is told the truth for its own contract: delivery succeeded and the
    //     epoch is live. NOT a rejection.
    assert_eq!(
        resp,
        WireResponse::EpochSynced {
            epoch: 4,
            applied: true
        },
        "a persist failure is NOT a verification failure — the epoch applied"
    );
    // (2) And it really IS enforcing: the probe's access flipped on the live store.
    assert_eq!(
        shared.snapshot_for_key(&k_probe).1,
        KeyAccess::NotAllowed,
        "the epoch must be in force in memory even though it could not be saved"
    );
    // (3) The durability really did fail (the store was never re-created) — so the
    //     arm under test is the Persist arm, not a healthy save.
    assert!(
        !store_path.exists(),
        "the save must genuinely have failed — otherwise this test proves nothing"
    );

    accept.abort();
}

/// The REAL robot's answer to a verb it cannot decode carries the SHARED
/// `UNDECODABLE_REQUEST_NEEDLE` — the marker a desk classifies as
/// `EpochPushOutcome::NoSink` ("upgrade the robot").
///
/// This closes the review-HIGH chain at the production end: the desk-side
/// classification is only correct if the robot really emits this marker. (The
/// older robot's serde text that carries it is reproduced in
/// `cerulion_connectd/tests/protocol_parity_test.rs`; the classification itself is
/// pinned in `cerulion_wireclient::epoch`.)
#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn an_undecodable_verb_is_answered_with_the_shared_older_peer_marker() {
    let desk_ep = disabled_endpoint([0x5D; 32]).await;
    let k_desk = *desk_ep.id().as_bytes();

    let robot = Robot::start(
        "undecodable",
        [0x5E; 32],
        &[(k_desk, SUBJECT, "desk")],
        true,
        Duration::from_secs(3600),
    )
    .await;
    let mut desk = connect(&desk_ep, robot.addr.clone(), "desk").await;

    // A verb THIS robot does not know: exactly the position an older robot is
    // in when a newer desk pushes `sync_epoch`.
    let resp = request_raw(
        &mut desk.send,
        &mut desk.recv,
        br#"{"verb":"a_verb_from_the_future","payload":"x"}"#,
    )
    .await;

    match &resp {
        WireResponse::Error { message, .. } => {
            assert!(
                message.contains(cerulion_pairing::verify::UNDECODABLE_REQUEST_NEEDLE),
                "the robot's undecodable-request answer must carry the SHARED marker a \
                 desk classifies on: {message}"
            );
            assert!(
                !message.contains(cerulion_pairing::verify::NO_EPOCH_SINK_NEEDLE),
                "and it must NOT carry the no-sink needle (only a sink-aware build emits \
                 that): {message}"
            );
        }
        other => panic!("expected a loud Error for an unknown verb, got {other:?}"),
    }

    // The control stream survives the unknown verb (a policy answer, not a framing
    // break) — the next real verb still works.
    let ok = desk
        .push(push_blob(
            epoch(3, vec![PROBE], vec![]),
            intermediate_cert(),
        ))
        .await;
    assert_eq!(
        ok,
        WireResponse::EpochSynced {
            epoch: 3,
            applied: true
        }
    );

    robot.shutdown();
}
