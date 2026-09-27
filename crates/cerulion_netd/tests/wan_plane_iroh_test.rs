// SPDX-License-Identifier: AGPL-3.0-only
//! The iroh WAN [`IrohMirrorPlane`] / [`DualMirrorPlane`] end-to-end
//! over REAL loopback iroh endpoints + REAL iceoryx2 (DISTINCT per-test SHM roots for
//! the robot and the desk — parallel-safe, no `#[serial]`).
//!
//! The headline: an in-process ROBOT (`cerulion_remoted::WirePlane` +
//! `PairingAuthorizer` over SHM root A, a real producer publishing hand-oracle
//! frames) ↔ netd's iroh WAN plane (on DISTINCT SHM root B) over real loopback iroh.
//! netd DEMANDS the topic via the plane's `MirrorPlane::ensure_mirror` (which runs the
//! folded connectd dial/demand/re-inject engine), the robot forwards, and a desk-local
//! subscriber on root B reads frames BYTE-IDENTICAL to a HAND oracle (each frame
//! recomputed from its own wire `sequence` — never a self-compare, Principle #13).
//!
//! Cribs `cerulion_connectd/tests/reinject_e2e_test.rs` (the loopback robot harness),
//! but drives netd's plane (which owns its OWN tokio runtime + block_on) from a plain
//! `#[test]` — so `ensure_mirror`/`release_mirror` are the exact synchronous seam the
//! daemon calls under the registry lock.
//!
//! Whole file `#![cfg(feature = "wan")]`. `wan` is DEFAULT-ON, so `cargo test -p
//! cerulion_netd` already compiles this file; `--no-default-features` compiles it to
//! nothing. Run it explicitly with
//! `cargo test -p cerulion_netd --features wan --test wan_plane_iroh_test`.
#![cfg(feature = "wan")]

use std::collections::HashMap;
use std::net::{Ipv4Addr, Ipv6Addr, SocketAddr};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use cerulion_core::transport::cerulion_q::{
    CatalogEntry, CatalogProvenance, CatalogReply, SchemaReply, CATALOG_WIRE_VERSION,
};
use cerulion_core::transport::demand_authorizer::{
    DemandAuthorizer, DemandDecision, DemandSubject,
};
use cerulion_core::transport::subscriber::OwnedInboundSample;
use cerulion_core::wire::{MaxSliceLen, WireHeader};
use cerulion_core::{TransportConfig, TransportManager};
use cerulion_link::{
    accept_frame_stream, accept_one, build_endpoint, open_uni_frame_stream, read_frame,
    write_frame, Endpoint, EndpointConfig, EndpointId, RelayConfig, DEFAULT_MAX_FRAME_LEN,
};
use cerulion_netd::mirror::{GatewayMirrorPlane, MirrorError, MirrorPlane, MirrorRelease};
use cerulion_netd::registry::TopicKey;
use cerulion_netd::{DualMirrorPlane, IrohMirrorPlane, Plane, WanRegistry, WanRobot};
use cerulion_pairing::client::DeviceIdentity;
use cerulion_pairing::format::{AccountId, PrincipalKind, PublicKey, RobotId, RootSet};
use cerulion_pairing::verify::TrustStore;
use cerulion_remoted::{
    handle_accepted_with_wire, DeviceAccountIndex, PairingAuthorizer, SharedTrust, WirePlane,
};
use cerulion_wireclient::protocol::{StreamPreamble, WireRequest, WireResponse};

const T_NOW: u64 = 1_000_000_000_000;
const CHASSIS: &[u8] = b"netd-test-chassis-secret-not-serial-derived";
const OWNER: AccountId = AccountId([10; 32]);
/// The hand-chosen wire `schema_hash` the oracle frames carry.
const ORACLE_SCHEMA_HASH: u64 = 0xCE83_7C04_D00D_F00D_u64;
/// A generous dial bound for the loopback path (never hit; a stall would surface as a
/// bounded iroh error, not a hang).
const DIAL_TIMEOUT: Duration = Duration::from_secs(10);

// ---------------------------------------------------------------------------
// Test-unique identity + SHM roots (parallel-safe).
// ---------------------------------------------------------------------------

fn unique_id() -> String {
    static N: AtomicU64 = AtomicU64::new(0);
    format!(
        "{}_{}",
        std::process::id(),
        N.fetch_add(1, Ordering::Relaxed)
    )
}

/// A per-test-unique 32-byte ed25519 seed (any 32 bytes is a valid seed), so parallel
/// tests never share an iroh node identity.
fn unique_secret() -> [u8; 32] {
    let mut s = [0u8; 32];
    s[..8].copy_from_slice(&(std::process::id() as u64).to_le_bytes());
    let n = {
        static N: AtomicU64 = AtomicU64::new(1);
        N.fetch_add(1, Ordering::Relaxed)
    };
    s[8..16].copy_from_slice(&n.to_le_bytes());
    s[16] = 0xAB; // non-zero filler
    s
}

/// The public half (== iroh endpoint id bytes) of a device seed.
fn public_of(seed: &[u8; 32]) -> [u8; 32] {
    DeviceIdentity::from_seed(seed).public_key().0
}

/// A fresh per-test transport manager on its own SHM root (parallel-safe).
fn test_manager(tag: &str) -> Arc<TransportManager> {
    let ix = cerulion_core::testing::iceoryx_test_config();
    TransportManager::init_for_test(
        TransportConfig {
            node_name: format!("netd_wan_{tag}_{}", unique_id()),
            ..Default::default()
        },
        ix,
    )
    .expect("init_for_test")
}

// ---------------------------------------------------------------------------
// Robot side: a real cerulion_remoted WirePlane over loopback iroh.
// ---------------------------------------------------------------------------

async fn disabled_endpoint(secret: [u8; 32]) -> Endpoint {
    build_endpoint(EndpointConfig::new(secret).with_relay(RelayConfig::Disabled))
        .await
        .expect("endpoint should bind")
}

/// Rewrite unspecified bound sockets to loopback so they are dialable.
fn loopback_sockets(bound: &[SocketAddr]) -> Vec<SocketAddr> {
    bound
        .iter()
        .map(|s| match s {
            SocketAddr::V4(v4) if v4.ip().is_unspecified() => {
                SocketAddr::from((Ipv4Addr::LOCALHOST, v4.port()))
            }
            SocketAddr::V6(v6) if v6.ip().is_unspecified() => {
                SocketAddr::from((Ipv6Addr::LOCALHOST, v6.port()))
            }
            other => *other,
        })
        .collect()
}

/// A provisioned + CLAIMED trust store (owner has OWNER_FULL scope ⇒ CAP_OBSERVE).
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

/// Claimed robot; the desk's key is bound to the OWNER (paired, CAP_OBSERVE).
fn claimed_paired(desk_key: [u8; 32]) -> PairingAuthorizer {
    let mut index = DeviceAccountIndex::new();
    index.bind(PublicKey(desk_key), OWNER);
    PairingAuthorizer::new(claimed_store(), index)
}

/// Claimed robot; NO desk key bound (unpaired → wire refused).
fn claimed_unpaired() -> PairingAuthorizer {
    PairingAuthorizer::new(claimed_store(), DeviceAccountIndex::new())
}

/// Serve wire connections in a LOOP until aborted — so a demand→release (which drops
/// the connection) can be followed by a re-demand (a fresh dial the loop accepts).
async fn robot_serve_loop(daemon: Endpoint, authz: Arc<PairingAuthorizer>, plane: Arc<WirePlane>) {
    // A non-`Ok(Some(..))` (Err / Ok(None)) ends the loop — the desk went away.
    while let Ok(Some(accepted)) = accept_one(&daemon).await {
        handle_accepted_with_wire(accepted, authz.clone(), None, Some(plane.clone())).await;
    }
}

/// A live in-process robot: its own tokio runtime + iroh endpoint + a WirePlane over
/// its SHM manager. Producers publish to `manager`; netd dials `eid`/`addrs`.
struct RobotFixture {
    rt: tokio::runtime::Runtime,
    /// A clone of the serve endpoint, so the test can CLOSE it (`kill`) to simulate a
    /// mid-stream robot disconnect (iroh `Endpoint` is Arc-backed — closing any clone
    /// closes the endpoint the serve loop reads).
    endpoint: Endpoint,
    eid: EndpointId,
    addrs: Vec<SocketAddr>,
    manager: Arc<TransportManager>,
    _serve: tokio::task::JoinHandle<()>,
}

impl RobotFixture {
    fn start(tag: &str, authz: Arc<PairingAuthorizer>) -> Self {
        let rt = tokio::runtime::Runtime::new().expect("robot runtime");
        let manager = test_manager(&format!("robot_{tag}"));
        let plane = Arc::new(WirePlane::with_manager(
            format!("robot-{tag}"),
            manager.clone(),
        ));
        let daemon = rt.block_on(disabled_endpoint(unique_secret()));
        let eid = daemon.id();
        let addrs = loopback_sockets(&daemon.bound_sockets());
        let endpoint = daemon.clone();
        let serve = rt.spawn(robot_serve_loop(daemon, authz, plane));
        Self {
            rt,
            endpoint,
            eid,
            addrs,
            manager,
            _serve: serve,
        }
    }

    /// Close the robot's iroh endpoint — the desk's live uni stream dies, simulating a
    /// mid-stream robot disconnect (the reader-death path).
    fn kill(&self) {
        self.rt.block_on(self.endpoint.close());
    }
}

/// A cached desk device cert blob — `base64url(postcard(SignedDeviceCert))`, the exact
/// on-disk shape `cerulion login` writes — binding `device_key` → `account`.
///
/// The signature is zero: `resolve_desk_account` performs the I1 KEY MATCH only (it
/// deliberately does not verify the chain), so a zero signature exercises the real code
/// path. Mirrors `wan.rs`'s in-crate `make_cert_b64`; duplicated here because that one
/// is `#[cfg(test)]`-private to the lib.
fn make_device_cert_b64(device_key: [u8; 32], account: [u8; 32]) -> String {
    use base64::engine::general_purpose::URL_SAFE_NO_PAD;
    use base64::Engine;
    use cerulion_pairing::format::{
        DeviceCert, Scope, Signature, SignedDeviceCert, Validity, FORMAT_VERSION,
    };
    let signed = SignedDeviceCert {
        cert: DeviceCert {
            version: FORMAT_VERSION,
            device_key: PublicKey(device_key),
            account: AccountId(account),
            principal_kind: PrincipalKind::Human,
            scope: Scope::OWNER_FULL,
            validity: Validity {
                not_before_ns: 0,
                not_after_ns: u64::MAX,
            },
            issued_at_ns: 0,
            issuer_key: PublicKey([0xAB; 32]),
        },
        signature: Signature([0u8; 64]),
    };
    URL_SAFE_NO_PAD.encode(postcard::to_stdvec(&signed).unwrap())
}

/// A single-robot WAN registry naming `fx` as `name`, keyed to `desk_seed`.
fn one_robot_registry(name: &str, fx: &RobotFixture, desk_seed: [u8; 32]) -> WanRegistry {
    let mut robots = HashMap::new();
    robots.insert(
        name.to_string(),
        WanRobot {
            eid: fx.eid,
            direct_addrs: fx.addrs.clone(),
        },
    );
    WanRegistry::new(robots, desk_seed, RelayConfig::Disabled)
}

// ---------------------------------------------------------------------------
// The hand oracle: each frame is a deterministic function of its sequence.
// ---------------------------------------------------------------------------

fn oracle_ts(seq: u32) -> u64 {
    1_000_000 + seq as u64 * 10_000
}

fn oracle_payload(seq: u32) -> Vec<u8> {
    let mut p = b"wan".to_vec();
    p.extend_from_slice(&seq.to_le_bytes());
    p
}

fn make_wire_frame(schema_hash: u64, seq: u32, ts: u64, payload: &[u8]) -> Vec<u8> {
    let header = WireHeader {
        schema_hash,
        total_size: (WireHeader::SIZE + payload.len()) as u32,
        offset_table_offset: (WireHeader::SIZE + payload.len()) as u32,
        offset_table_count: 0,
        sequence: seq,
        timestamp_ns: ts,
    };
    let mut frame = vec![0u8; WireHeader::SIZE + payload.len()];
    header.write_to_buf(&mut frame[..WireHeader::SIZE]);
    frame[WireHeader::SIZE..].copy_from_slice(payload);
    frame
}

/// The oracle frame for a given sequence (the ONLY thing a received frame is ever
/// compared against — never a self-compare).
fn oracle_frame(seq: u32) -> Vec<u8> {
    make_wire_frame(
        ORACLE_SCHEMA_HASH,
        seq,
        oracle_ts(seq),
        &oracle_payload(seq),
    )
}

/// Spawn a producer publishing `oracle_frame(seq)` for seq = 0,1,2,… until `stop`.
fn spawn_producer(
    manager: Arc<TransportManager>,
    topic: String,
    stop: Arc<AtomicBool>,
) -> std::thread::JoinHandle<()> {
    std::thread::spawn(move || {
        let mut publisher = manager
            .create_publisher_simple(&topic, MaxSliceLen::const_new(256))
            .expect("producer");
        let mut seq = 0u32;
        while !stop.load(Ordering::Relaxed) {
            let _ = publisher.publish_raw(&oracle_frame(seq));
            seq = seq.wrapping_add(1);
            std::thread::sleep(Duration::from_millis(5));
        }
    })
}

/// Attach a desk-local data-only tap on `topic` (poll-retry until the ingress service
/// exists — netd creates it on the first re-injected frame), then collect `n` full
/// frames + their sequences.
fn collect_frames(manager: &TransportManager, topic: &str, n: usize) -> Vec<(u32, Vec<u8>)> {
    let deadline = Instant::now() + Duration::from_secs(20);
    let mut tap = loop {
        match manager.create_data_only_subscriber(topic) {
            Ok(t) => break t,
            Err(_) if Instant::now() < deadline => std::thread::sleep(Duration::from_millis(20)),
            Err(e) => panic!("desk ingress service never appeared for {topic}: {e}"),
        }
    };
    let budget = tap.max_borrowed_samples().max(1);
    let mut batch: Vec<OwnedInboundSample> = Vec::with_capacity(budget);
    let mut collected: Vec<(u32, Vec<u8>)> = Vec::new();
    let deadline = Instant::now() + Duration::from_secs(20);
    while collected.len() < n && Instant::now() < deadline {
        batch.clear();
        match tap.drain_owned(budget, &mut batch) {
            Ok(0) => std::thread::sleep(Duration::from_millis(3)),
            Ok(_) => {
                for sample in batch.drain(..) {
                    let seq = sample.wire_header().map(|h| h.sequence);
                    let frame = sample.payload().to_vec();
                    drop(sample);
                    if let Some(s) = seq {
                        collected.push((s, frame));
                    }
                }
            }
            Err(_) => std::thread::sleep(Duration::from_millis(3)),
        }
    }
    assert!(
        collected.len() >= n,
        "collected only {} of {n} frames for {topic}",
        collected.len()
    );
    collected
}

/// Assert the collected frames are BYTE-IDENTICAL to the hand oracle (by their own
/// sequence) and GAP-FREE from the first received.
fn assert_oracle_and_gap_free(collected: &[(u32, Vec<u8>)]) {
    let first = collected[0].0;
    for (i, (seq, frame)) in collected.iter().enumerate() {
        assert_eq!(
            *seq,
            first + i as u32,
            "sequences gap-free from first-received"
        );
        assert_eq!(
            frame,
            &oracle_frame(*seq),
            "frame {seq} must be BYTE-IDENTICAL to the hand oracle"
        );
    }
}

/// Poll `gather_mirror_provenance` until `topic` appears (or the deadline), returning
/// its attributed origin robot.
fn gather_origin(manager: &TransportManager, topic: &str) -> Option<String> {
    let deadline = Instant::now() + Duration::from_secs(5);
    while Instant::now() < deadline {
        let snapshot = manager
            .gather_mirror_provenance(Duration::from_millis(300))
            .expect("gather provenance");
        if let Some(rec) = snapshot.iter().find(|r| r.topic == topic) {
            return Some(rec.origin_robot.clone());
        }
    }
    None
}

/// Whether `topic`'s provenance is ABSENT (polled until absent or the deadline).
fn gather_absent(manager: &TransportManager, topic: &str) -> bool {
    let deadline = Instant::now() + Duration::from_secs(5);
    while Instant::now() < deadline {
        let snapshot = manager
            .gather_mirror_provenance(Duration::from_millis(300))
            .expect("gather provenance");
        if !snapshot.iter().any(|r| r.topic == topic) {
            return true;
        }
        std::thread::sleep(Duration::from_millis(30));
    }
    false
}

// ---------------------------------------------------------------------------
// (1) HEADLINE: demand over iroh → BYTE-IDENTICAL frames re-injected into desk SHM.
// ---------------------------------------------------------------------------

/// Run the full ensure→collect flow once and return the desk-collected pairs, plus
/// the plane (kept alive for release assertions) + the teardown handles.
fn run_headline(tag: &str, n: usize) -> Vec<(u32, Vec<u8>)> {
    let desk_seed = unique_secret();
    let authz = Arc::new(claimed_paired(public_of(&desk_seed)));
    let fx = RobotFixture::start(tag, authz);
    let topic = format!("/wan/{tag}/{}", unique_id());

    let stop = Arc::new(AtomicBool::new(false));
    let producer = spawn_producer(fx.manager.clone(), topic.clone(), stop.clone());

    let manager_b = test_manager(&format!("{tag}_desk"));
    let registry = Arc::new(one_robot_registry("ubuntu", &fx, desk_seed));
    let plane = IrohMirrorPlane::new(manager_b.clone(), registry)
        .expect("iroh plane")
        .with_dial_timeout(DIAL_TIMEOUT);
    let key = TopicKey::new("ubuntu", &topic);

    plane
        .ensure_mirror(&key, ORACLE_SCHEMA_HASH)
        .expect("ensure_mirror over iroh WAN");
    assert_eq!(
        plane.connection_count(),
        1,
        "one WAN connection to the robot"
    );
    assert_eq!(plane.reader_count(), 1, "one live re-inject reader");

    let collected = collect_frames(&manager_b, &topic, n);

    stop.store(true, Ordering::Relaxed);
    let _ = producer.join();
    drop(plane); // drops the iroh runtime → aborts the reader + closes the connection
    collected
}

#[test]
fn iroh_wan_plane_reinjects_byte_identical_frames() {
    let collected = run_headline("hdl", 6);
    assert_oracle_and_gap_free(&collected);
}

/// Determinism (Principle #7): two full runs each match the hand oracle (each frame
/// equals `oracle_frame(its_seq)` — NOT a self-compare of the two runs).
#[test]
fn iroh_wan_plane_reinject_is_deterministic() {
    let a = run_headline("det_a", 5);
    let b = run_headline("det_b", 5);
    assert_oracle_and_gap_free(&a);
    assert_oracle_and_gap_free(&b);
}

// ---------------------------------------------------------------------------
// (2) RELEASE tears the mirror down (Retired) + frees the single-writer SHM slot.
// ---------------------------------------------------------------------------

#[test]
fn iroh_wan_plane_release_retires_and_frees_the_slot() {
    let desk_seed = unique_secret();
    let authz = Arc::new(claimed_paired(public_of(&desk_seed)));
    let fx = RobotFixture::start("rel", authz);
    let topic = format!("/wan/rel/{}", unique_id());

    let stop = Arc::new(AtomicBool::new(false));
    let producer = spawn_producer(fx.manager.clone(), topic.clone(), stop.clone());

    let manager_b = test_manager("rel_desk");
    let registry = Arc::new(one_robot_registry("ubuntu", &fx, desk_seed));
    let plane = IrohMirrorPlane::new(manager_b.clone(), registry)
        .expect("iroh plane")
        .with_dial_timeout(DIAL_TIMEOUT);
    let key = TopicKey::new("ubuntu", &topic);

    plane
        .ensure_mirror(&key, ORACLE_SCHEMA_HASH)
        .expect("ensure");
    let _ = collect_frames(&manager_b, &topic, 2); // let frames flow

    // Release: the LAST-release path tears down the reader + drops the connection.
    assert_eq!(
        plane.release_mirror(&key),
        MirrorRelease::Retired,
        "iroh last-release RETIRES the mirror"
    );
    assert_eq!(plane.reader_count(), 0, "reader aborted");
    assert_eq!(
        plane.connection_count(),
        0,
        "connection dropped (no more topics)"
    );

    // The single-writer SHM mirror slot is freed → a fresh injector attaches (the
    // mutation-kill: a teardown that forgot to abort+await the reader leaves the slot
    // owned and this create errs).
    manager_b
        .create_ingress_injector(&topic, ORACLE_SCHEMA_HASH, MaxSliceLen::const_new(256))
        .expect("a fresh ingress injector attaches after the iroh release (slot freed)");

    stop.store(true, Ordering::Relaxed);
    let _ = producer.join();
}

// ---------------------------------------------------------------------------
// (3) PROVENANCE: the iroh re-inject registers mirror provenance (→ REMOTE fold)
//     attributed to the DEMAND's robot, and release removes it.
// ---------------------------------------------------------------------------

#[test]
fn iroh_wan_plane_registers_and_removes_mirror_provenance() {
    let desk_seed = unique_secret();
    let authz = Arc::new(claimed_paired(public_of(&desk_seed)));
    let fx = RobotFixture::start("prov", authz);
    let topic = format!("/wan/prov/{}", unique_id());

    let stop = Arc::new(AtomicBool::new(false));
    let producer = spawn_producer(fx.manager.clone(), topic.clone(), stop.clone());

    let manager_b = test_manager("prov_desk");
    let registry = Arc::new(one_robot_registry("go2-ubuntu", &fx, desk_seed));
    let plane = IrohMirrorPlane::new(manager_b.clone(), registry)
        .expect("iroh plane")
        .with_dial_timeout(DIAL_TIMEOUT);
    let key = TopicKey::new("go2-ubuntu", &topic);

    plane
        .ensure_mirror(&key, ORACLE_SCHEMA_HASH)
        .expect("ensure");
    let _ = collect_frames(&manager_b, &topic, 2);

    // Provenance registered, attributed to the DEMAND's robot (not the robot's
    // self-declared catalog identity) — so `topic list` folds it into REMOTE.
    assert_eq!(
        gather_origin(&manager_b, &topic).as_deref(),
        Some("go2-ubuntu"),
        "the iroh re-inject registers mirror provenance attributed to the demand's robot"
    );

    // Release removes the provenance (the mirror stops folding into REMOTE).
    assert_eq!(plane.release_mirror(&key), MirrorRelease::Retired);
    assert!(
        gather_absent(&manager_b, &topic),
        "release removes the mirror provenance"
    );

    stop.store(true, Ordering::Relaxed);
    let _ = producer.join();
}

// ---------------------------------------------------------------------------
// (4) UNPAIRED REFUSAL: an unpaired desk key is refused with a LOUD MirrorError::Iroh
//     (the pairing gate holds through the fold); no mirror is created.
// ---------------------------------------------------------------------------

#[test]
fn unpaired_desk_is_refused_with_loud_iroh_error() {
    let desk_seed = unique_secret();
    // The robot's access list does NOT bind the desk key → wire refused.
    let fx = RobotFixture::start("unp", Arc::new(claimed_unpaired()));
    let manager_b = test_manager("unp_desk");
    let registry = Arc::new(one_robot_registry("ubuntu", &fx, desk_seed));
    let plane = IrohMirrorPlane::new(manager_b, registry)
        .expect("iroh plane")
        .with_dial_timeout(DIAL_TIMEOUT);
    let key = TopicKey::new("ubuntu", "/wan/unp/topic");

    let err = plane
        .ensure_mirror(&key, ORACLE_SCHEMA_HASH)
        .expect_err("an unpaired desk must be refused");
    match err {
        MirrorError::Iroh { reason, key: k } => {
            assert!(
                reason.contains("refused") || reason.contains("unpaired"),
                "the robot's refusal reason is surfaced verbatim: {reason}"
            );
            assert_eq!(k.robot, "ubuntu");
        }
        other => panic!("expected MirrorError::Iroh, got {other:?}"),
    }
    // The refusal happens at the catalog gate BEFORE any connection is tracked / any
    // reader is spawned — no phantom mirror.
    assert_eq!(
        plane.connection_count(),
        0,
        "no connection tracked on refusal"
    );
    assert_eq!(plane.reader_count(), 0, "no reader spawned on refusal");
}

// ---------------------------------------------------------------------------
// (5) DUAL PLANE: a WAN-registered robot routes to iroh + delivers; an unregistered
//     robot routes to the zenoh LAN plane (the picker decision).
// ---------------------------------------------------------------------------

#[test]
fn dual_plane_routes_wan_robot_to_iroh_and_delivers() {
    let desk_seed = unique_secret();
    let authz = Arc::new(claimed_paired(public_of(&desk_seed)));
    let fx = RobotFixture::start("dual", authz);
    let topic = format!("/wan/dual/{}", unique_id());

    let stop = Arc::new(AtomicBool::new(false));
    let producer = spawn_producer(fx.manager.clone(), topic.clone(), stop.clone());

    let manager_b = test_manager("dual_desk");
    let registry = Arc::new(one_robot_registry("ubuntu", &fx, desk_seed));
    // Both planes wrap the SAME desk manager (the one SHM mirror target). The zenoh
    // plane is inert here (manager_b is network-less) — the picker never routes the
    // WAN robot to it, which is exactly the property under test.
    let iroh = IrohMirrorPlane::new(manager_b.clone(), Arc::clone(&registry))
        .expect("iroh plane")
        .with_dial_timeout(DIAL_TIMEOUT);
    let zenoh = GatewayMirrorPlane::new(manager_b.clone());
    let dual = DualMirrorPlane::new(zenoh, iroh, Arc::clone(&registry));

    // The routing decision (the picker through the DualMirrorPlane surface).
    assert_eq!(
        dual.plane_for(&TopicKey::new("ubuntu", &topic)).unwrap(),
        Plane::Iroh,
        "a WAN-registered robot routes to iroh"
    );
    assert_eq!(
        dual.plane_for(&TopicKey::new("lan-bot", "/tf")).unwrap(),
        Plane::Zenoh,
        "an unregistered robot routes to the zenoh LAN default"
    );

    // Demand the WAN robot THROUGH the dual plane → routes to iroh → delivers.
    let key = TopicKey::new("ubuntu", &topic);
    dual.ensure_mirror(&key, ORACLE_SCHEMA_HASH)
        .expect("dual ensure routes the WAN robot to iroh");
    let collected = collect_frames(&manager_b, &topic, 5);
    assert_oracle_and_gap_free(&collected);

    // Release routes back to iroh (re-pick) and retires.
    assert_eq!(dual.release_mirror(&key), MirrorRelease::Retired);

    stop.store(true, Ordering::Relaxed);
    let _ = producer.join();
}

// ---------------------------------------------------------------------------
// (6) DEFENSIVE: a direct iroh ensure for a robot NOT in the WAN registry is refused
//     loudly WITHOUT tracking a connection (the DualMirrorPlane never reaches here —
//     the picker routes an unregistered robot to zenoh — but the plane guards itself).
// ---------------------------------------------------------------------------

#[test]
fn iroh_ensure_for_unknown_robot_is_refused() {
    let manager_b = test_manager("unk_desk");
    let registry = Arc::new(WanRegistry::new(
        HashMap::new(),
        unique_secret(),
        RelayConfig::Disabled,
    ));
    let plane = IrohMirrorPlane::new(manager_b, registry)
        .expect("iroh plane")
        .with_dial_timeout(DIAL_TIMEOUT);
    let key = TopicKey::new("ghost", "/x");

    let err = plane
        .ensure_mirror(&key, ORACLE_SCHEMA_HASH)
        .expect_err("a robot with no WAN endpoint must be refused");
    match err {
        MirrorError::Iroh { reason, .. } => assert!(
            reason.contains("no WAN endpoint configured"),
            "reason: {reason}"
        ),
        other => panic!("expected MirrorError::Iroh, got {other:?}"),
    }
    assert_eq!(plane.connection_count(), 0);
}

// ---------------------------------------------------------------------------
// (7) UNREACHABLE ROBOT: a WAN-registered robot with a valid but
//     unreachable EndpointId → the demand fails LOUDLY within the dial bound (never
//     hangs). The timeout is shrunk via the test seam so the test stays fast.
// ---------------------------------------------------------------------------

#[test]
fn unreachable_wan_robot_demand_fails_loudly_within_bound() {
    // A valid ed25519 endpoint id that NO robot serves + NO direct addr + relay
    // disabled ⇒ iroh has no path to the peer ⇒ the dial fails/times out within the
    // (shrunk) bound rather than hanging.
    let ghost_eid = EndpointId::from_bytes(&public_of(&unique_secret())).expect("valid eid");
    let mut robots = HashMap::new();
    robots.insert(
        "ubuntu".to_string(),
        WanRobot {
            eid: ghost_eid,
            direct_addrs: vec![],
        },
    );
    let registry = Arc::new(WanRegistry::new(
        robots,
        unique_secret(),
        RelayConfig::Disabled,
    ));
    let dial_timeout = Duration::from_millis(800);
    let plane = IrohMirrorPlane::new(test_manager("unreach_desk"), registry)
        .expect("iroh plane")
        .with_dial_timeout(dial_timeout);
    let key = TopicKey::new("ubuntu", "/wan/unreach");

    let start = Instant::now();
    let err = plane
        .ensure_mirror(&key, ORACLE_SCHEMA_HASH)
        .expect_err("an unreachable robot's demand must fail, not hang");
    let elapsed = start.elapsed();

    match err {
        MirrorError::Iroh { reason, key: k } => {
            assert_eq!(k.robot, "ubuntu", "the error names the robot");
            assert!(
                reason.contains("dial") || reason.contains("timed out"),
                "the error names the dial phase: {reason}"
            );
        }
        other => panic!("expected MirrorError::Iroh, got {other:?}"),
    }
    // Loud AND bounded — comfortably under 2× the dial timeout (never an unbounded hang).
    assert!(
        elapsed < dial_timeout * 3,
        "the demand must fail within the bound, took {elapsed:?}"
    );
    assert_eq!(
        plane.connection_count(),
        0,
        "no connection tracked on a failed dial"
    );
}

// ---------------------------------------------------------------------------
// (8) The desk-local single-writer slot is TAKEN (a local producer owns the
//     topic) → the demand FAILS LOUDLY and synchronously with NO phantom mirror (the
//     injector is created BEFORE the reader spawns), fix 4 drops the orphaned
//     connection, and — once the slot is freed — a RE-DEMAND re-dials + delivers.
// ---------------------------------------------------------------------------

#[test]
fn injector_slot_taken_fails_demand_no_phantom_then_re_demand_succeeds() {
    let desk_seed = unique_secret();
    let authz = Arc::new(claimed_paired(public_of(&desk_seed)));
    let fx = RobotFixture::start("slot", authz);
    let topic = format!("/wan/slot/{}", unique_id());

    // The robot IS producing the topic (so the mirror WOULD stream if the slot were free).
    let stop = Arc::new(AtomicBool::new(false));
    let producer = spawn_producer(fx.manager.clone(), topic.clone(), stop.clone());

    let manager_b = test_manager("slot_desk");
    // Occupy BOTH of the desk-local ingress service's 2 publisher slots (at the SAME
    // slice-len the mirror uses, so the collision is on SLOTS, not slice-len config) —
    // so the mirror's injector create (a 3rd publisher) is REJECTED. This is the
    // slot-taken failure this test guards against; dropping the occupiers frees the slots.
    let mirror_slice =
        MaxSliceLen::const_new(cerulion_core::graph::config::DEFAULT_MAX_SLICE_LEN as u32);
    let occ1 = manager_b
        .create_ingress_injector(&topic, ORACLE_SCHEMA_HASH, mirror_slice)
        .expect("occupier injector 1");
    let occ2 = manager_b
        .create_ingress_injector(&topic, ORACLE_SCHEMA_HASH, mirror_slice)
        .expect("occupier injector 2");

    let registry = Arc::new(one_robot_registry("ubuntu", &fx, desk_seed));
    let plane = IrohMirrorPlane::new(manager_b.clone(), registry)
        .expect("iroh plane")
        .with_dial_timeout(DIAL_TIMEOUT);
    let key = TopicKey::new("ubuntu", &topic);

    // First demand: the dial + demand + uni stream all succeed, but the SYNCHRONOUS
    // injector create fails (slot taken) → a loud MirrorError::Iroh, NO phantom mirror.
    let err = plane
        .ensure_mirror(&key, ORACLE_SCHEMA_HASH)
        .expect_err("a taken single-writer slot must fail the demand loudly");
    match err {
        MirrorError::Iroh { reason, .. } => assert!(
            reason.contains("could not be created") || reason.contains("already produced"),
            "reason names the slot-taken cause: {reason}"
        ),
        other => panic!("expected MirrorError::Iroh, got {other:?}"),
    }
    assert_eq!(
        plane.reader_count(),
        0,
        "NO phantom reader on a failed injector create"
    );
    assert_eq!(
        plane.connection_count(),
        0,
        "fix 4: the orphaned readerless connection is dropped so a re-demand re-dials"
    );

    // Free the slots, then RE-DEMAND — the plane re-dials a fresh connection (the robot
    // loop accepts it), creates the injector (slots now free), and delivers.
    drop(occ1);
    drop(occ2);
    plane
        .ensure_mirror(&key, ORACLE_SCHEMA_HASH)
        .expect("re-demand succeeds after the slot is freed (fix 4 re-dial)");
    assert_eq!(plane.reader_count(), 1);
    let collected = collect_frames(&manager_b, &topic, 4);
    assert_oracle_and_gap_free(&collected);

    stop.store(true, Ordering::Relaxed);
    let _ = producer.join();
}

// ---------------------------------------------------------------------------
// (9) A mid-stream robot disconnect kills the reader → the supervising task
//     LOUDLY tears the plane's mirror state down (reader removed, connection dropped,
//     provenance removed) so the plane never keeps a zombie mirror.
// ---------------------------------------------------------------------------

// NB: the teardown's loudness (`on_reader_death`'s unconditional `error!`) is
// code-verified, not asserted here — it runs on the plane's tokio WORKER thread, which
// `#[traced_test]`'s thread-local subscriber cannot capture. The FUNCTIONAL teardown
// (reader removed + connection dropped + provenance removed) IS the fix, and
// is asserted on state below.
#[test]
fn reader_death_on_robot_disconnect_tears_down() {
    let desk_seed = unique_secret();
    let authz = Arc::new(claimed_paired(public_of(&desk_seed)));
    let fx = RobotFixture::start("death", authz);
    let topic = format!("/wan/death/{}", unique_id());

    let stop = Arc::new(AtomicBool::new(false));
    let producer = spawn_producer(fx.manager.clone(), topic.clone(), stop.clone());

    let manager_b = test_manager("death_desk");
    let registry = Arc::new(one_robot_registry("ubuntu", &fx, desk_seed));
    let plane = IrohMirrorPlane::new(manager_b.clone(), registry)
        .expect("iroh plane")
        .with_dial_timeout(DIAL_TIMEOUT);
    let key = TopicKey::new("ubuntu", &topic);

    plane
        .ensure_mirror(&key, ORACLE_SCHEMA_HASH)
        .expect("ensure");
    let _ = collect_frames(&manager_b, &topic, 2); // let the mirror stream
    assert_eq!(plane.reader_count(), 1);
    // Provenance is registered while streaming.
    assert_eq!(gather_origin(&manager_b, &topic).as_deref(), Some("ubuntu"));

    // The robot goes away MID-STREAM: close its endpoint → the desk's uni stream dies.
    fx.kill();

    // The supervising reader task detects the death and tears the mirror state down.
    let deadline = Instant::now() + Duration::from_secs(15);
    while plane.reader_count() > 0 && Instant::now() < deadline {
        std::thread::sleep(Duration::from_millis(30));
    }
    assert_eq!(plane.reader_count(), 0, "the dead reader is torn down");
    assert_eq!(
        plane.connection_count(),
        0,
        "the connection is dropped (its last reader died)"
    );
    assert!(
        gather_absent(&manager_b, &topic),
        "the mirror's provenance is removed on reader death"
    );

    stop.store(true, Ordering::Relaxed);
    let _ = producer.join();
}

// ---------------------------------------------------------------------------
// (11) When the injector fails on a connection that has
//      OTHER live readers (so the connection is NOT dropped), the plane must send an
//      Undemand so the robot tears down the tap it opened — else the robot keeps
//      forwarding a stream nobody reads. Uses a hand-rolled RECORDING robot (the real
//      WirePlane's TapManager is per-connection/internal, not observable), cribbing
//      cerulion_connectd's spoofing_robot, so the test can assert the robot SAW the
//      Undemand message.
// ---------------------------------------------------------------------------

/// A hand-rolled robot: serves Catalog + Demand (opens a uni stream + preamble, kept
/// alive so the desk reader stays live) and RECORDS every Undemand's topic. No
/// pairing gate (the desk's catalog fetch is admitted by any Catalog reply).
async fn recording_robot(
    daemon: Endpoint,
    catalog_topics: Vec<(String, u64)>,
    undemands: Arc<std::sync::Mutex<Vec<String>>>,
) {
    recording_robot_recording_epochs(
        daemon,
        catalog_topics,
        undemands,
        Arc::default(),
        EpochReply::NoSink,
    )
    .await
}

/// How the stub robot answers a `sync_epoch` push — ONE arm per
/// [`EpochPushOutcome`] a desk can observe, so every classification is exercised
/// against the real desk-side decoder rather than only the two happy shapes.
#[derive(Debug, Clone, Copy)]
enum EpochReply {
    /// A robot WITH a sink that APPLIED the epoch.
    Applied,
    /// A robot already at/ahead of the pushed epoch (the steady state).
    AlreadyCurrent,
    /// An epoch-sync-aware robot with no sync sink wired (its own needle).
    NoSink,
    /// A robot PREDATING the verb: it cannot decode the request at all and answers
    /// its malformed-request marker — the case that must not be
    /// misreported as a REJECTED epoch.
    OlderRobot,
    /// A robot that verified the epoch and REJECTED it (forged / wrong robot / skew).
    Rejected,
    /// A robot answering something else entirely.
    UnexpectedReply,
    /// A robot that ACCEPTS the push and then never answers, holding the connection
    /// open — the desk's bounded control read must time out. The one FATAL class:
    /// the control framing is desynced, so the dial fails and `TransportFailed` is
    /// recorded (the production-written outcome that
    /// no test asserted).
    StallForever,
}

impl EpochReply {
    fn response(self) -> WireResponse {
        match self {
            EpochReply::Applied => WireResponse::EpochSynced {
                epoch: 7,
                applied: true,
            },
            EpochReply::AlreadyCurrent => WireResponse::EpochSynced {
                epoch: 12,
                applied: false,
            },
            EpochReply::NoSink => WireResponse::Error {
                topic: None,
                message: format!(
                    "this stub robot {} (no sync sink)",
                    cerulion_pairing::verify::NO_EPOCH_SINK_NEEDLE
                ),
            },
            // The EXACT shape an older robot produces: its malformed-request marker
            // plus serde's unknown-variant text. (The REAL serde error is reproduced
            // in `cerulion_connectd/tests/protocol_parity_test.rs`.)
            EpochReply::OlderRobot => WireResponse::Error {
                topic: None,
                message: format!(
                    "{}: unknown variant `sync_epoch`, expected one of `catalog`, `demand`, \
                     `undemand`, `schema`, `status` at line 1 column 20",
                    cerulion_pairing::verify::UNDECODABLE_REQUEST_NEEDLE
                ),
            },
            EpochReply::Rejected => WireResponse::Error {
                topic: None,
                message: "the pushed access-list epoch was REJECTED: invalid signature on epoch; \
                          the epoch was NOT applied"
                    .to_string(),
            },
            EpochReply::UnexpectedReply => WireResponse::Undemanded {
                topic: "/not/an/epoch/reply".to_string(),
                was_demanded: false,
            },
            // Never rendered: the stub returns BEFORE building a response.
            EpochReply::StallForever => unreachable!("the stalling arm never answers"),
        }
    }
}

/// The same hand-rolled robot, additionally RECORDING every `sync_epoch` blob it
/// receives and choosing how to answer it via [`EpochReply`].
///
/// The push arrives on the ONE bidi control stream this robot accepts — the SAME
/// discipline the real robot's `serve_wire_connection` uses (see the comment in the
/// body). A stub that accepted a second bidi stream would make the desk-side tests
/// pass against behavior production does not have.
async fn recording_robot_recording_epochs(
    daemon: Endpoint,
    catalog_topics: Vec<(String, u64)>,
    undemands: Arc<std::sync::Mutex<Vec<String>>>,
    epochs: Arc<std::sync::Mutex<Vec<String>>>,
    accept_epoch: EpochReply,
) {
    let Ok(Some(accepted)) = accept_one(&daemon).await else {
        return;
    };
    let conn = accepted.connection;
    // ONE bidi control stream — the SAME discipline the real robot's
    // `serve_wire_connection` uses (it calls `accept_frame_stream` exactly once). The
    // epoch push therefore arrives on THIS stream, and a stub that accepted a second
    // stream would make the desk-side tests pass against behavior production does not
    // have.
    let Ok((mut send, mut recv)) = accept_frame_stream(&conn).await else {
        return;
    };
    let mut open_streams = Vec::new(); // keep the per-topic uni streams alive
    loop {
        let req_bytes = match read_frame(&mut recv, DEFAULT_MAX_FRAME_LEN).await {
            Ok(b) => b,
            Err(_) => break,
        };
        let req: WireRequest = match serde_json::from_slice(&req_bytes) {
            Ok(r) => r,
            Err(_) => break,
        };
        let resp = match req {
            WireRequest::Catalog => WireResponse::Catalog(CatalogReply {
                version: CATALOG_WIRE_VERSION,
                robot: "rec-robot".to_string(),
                entries: catalog_topics
                    .iter()
                    .map(|(t, h)| CatalogEntry {
                        topic: t.clone(),
                        schema_hash: Some(*h),
                        schema_name: None,
                        provenance: CatalogProvenance::Runtime,
                        producer_count: None,
                        liveness: None,
                    })
                    .collect(),
                error: None,
            }),
            WireRequest::Demand { topic } => {
                if let Ok(mut ustream) = open_uni_frame_stream(&conn).await {
                    let preamble = serde_json::to_vec(&StreamPreamble {
                        topic: topic.clone(),
                    })
                    .unwrap();
                    let _ = write_frame(&mut ustream, &preamble).await;
                    open_streams.push(ustream);
                }
                WireResponse::DemandAccepted { topic }
            }
            WireRequest::Undemand { topic } => {
                undemands.lock().unwrap().push(topic.clone());
                WireResponse::Undemanded {
                    topic,
                    was_demanded: true,
                }
            }
            WireRequest::Schema { topic } => {
                WireResponse::Schema(SchemaReply::not_found("rec-robot", &topic, "no schema"))
            }
            WireRequest::Status => WireResponse::Error {
                topic: None,
                message: "n/a".to_string(),
            },
            // Record the pushed blob, then answer as the arm under test
            // dictates. The demand assertions in each test run AFTER this reply on the
            // SAME stream, so they double as the "the push left the control stream
            // usable" pin.
            WireRequest::SyncEpoch { epoch_postcard } => {
                epochs.lock().unwrap().push(epoch_postcard);
                if matches!(accept_epoch, EpochReply::StallForever) {
                    // Received it, never answer, and HOLD the connection open so the
                    // desk sees a timeout (not a stream error).
                    tokio::time::sleep(Duration::from_secs(30)).await;
                    return;
                }
                accept_epoch.response()
            }
        };
        if write_frame(&mut send, &serde_json::to_vec(&resp).unwrap())
            .await
            .is_err()
        {
            break;
        }
    }
}

#[test]
fn injector_fail_on_a_connection_with_readers_undemands_the_robot() {
    let rt = tokio::runtime::Runtime::new().expect("robot runtime");
    let manager_b = test_manager("undemand_desk");
    let topic_a = format!("/wan/undemand/a/{}", unique_id());
    let topic_b = format!("/wan/undemand/b/{}", unique_id());
    let undemands = Arc::new(std::sync::Mutex::new(Vec::<String>::new()));

    // The recording robot advertises + serves both topics.
    let daemon = rt.block_on(disabled_endpoint(unique_secret()));
    let eid = daemon.id();
    let addrs = loopback_sockets(&daemon.bound_sockets());
    let catalog = vec![
        (topic_a.clone(), ORACLE_SCHEMA_HASH),
        (topic_b.clone(), ORACLE_SCHEMA_HASH),
    ];
    let _serve = rt.spawn(recording_robot(daemon, catalog, undemands.clone()));

    let mut robots = HashMap::new();
    robots.insert(
        "ubuntu".to_string(),
        WanRobot {
            eid,
            direct_addrs: addrs,
        },
    );
    let registry = Arc::new(WanRegistry::new(
        robots,
        unique_secret(),
        RelayConfig::Disabled,
    ));
    let plane = IrohMirrorPlane::new(manager_b.clone(), registry)
        .expect("iroh plane")
        .with_dial_timeout(DIAL_TIMEOUT);

    // Occupy topic B's desk-local ingress slots so ITS injector create fails (topic A
    // stays free so A's demand succeeds and keeps a live reader on the connection).
    let mirror_slice =
        MaxSliceLen::const_new(cerulion_core::graph::config::DEFAULT_MAX_SLICE_LEN as u32);
    let occ1 = manager_b
        .create_ingress_injector(&topic_b, ORACLE_SCHEMA_HASH, mirror_slice)
        .expect("occupier 1");
    let occ2 = manager_b
        .create_ingress_injector(&topic_b, ORACLE_SCHEMA_HASH, mirror_slice)
        .expect("occupier 2");

    // Demand A → succeeds; a live reader now keeps the ONE connection alive.
    plane
        .ensure_mirror(&TopicKey::new("ubuntu", &topic_a), ORACLE_SCHEMA_HASH)
        .expect("demand A succeeds");
    assert_eq!(plane.reader_count(), 1);
    assert_eq!(plane.connection_count(), 1);

    // Demand B → its injector fails (slots taken). The demand errs, but the connection
    // is NOT dropped (reader A is alive) — so the ONLY tap-teardown for B is the
    // explicit Undemand this test pins.
    let err = plane
        .ensure_mirror(&TopicKey::new("ubuntu", &topic_b), ORACLE_SCHEMA_HASH)
        .expect_err("demand B fails at the injector");
    assert!(
        matches!(err, MirrorError::Iroh { .. }),
        "the slot-taken failure is a loud MirrorError::Iroh"
    );
    assert_eq!(
        plane.reader_count(),
        1,
        "reader A survives (no phantom B reader)"
    );
    assert_eq!(
        plane.connection_count(),
        1,
        "the connection is KEPT (reader A is alive — this is the has-other-readers case)"
    );

    // The robot received the Undemand for B (the round-trip completes inside
    // ensure_mirror, but poll to absorb any serve-loop scheduling lag).
    let deadline = Instant::now() + Duration::from_secs(5);
    let saw_b_undemand = loop {
        if undemands.lock().unwrap().iter().any(|t| t == &topic_b) {
            break true;
        }
        if Instant::now() >= deadline {
            break false;
        }
        std::thread::sleep(Duration::from_millis(20));
    };
    assert!(
        saw_b_undemand,
        "the plane MUST Undemand topic B on the robot after the injector fail (HIGH #1)"
    );
    // A (still streaming) was NOT undemanded.
    assert!(
        !undemands.lock().unwrap().iter().any(|t| t == &topic_a),
        "topic A (streaming) must NOT be undemanded"
    );

    drop(occ1);
    drop(occ2);
    drop(plane);
}

// ---------------------------------------------------------------------------
// The WAN (iroh) half of the ONE demand-authorization seam. The plane
// consults a DemandAuthorizer BEFORE dialing, so an unauthorized demand never opens a
// connection. It COMPOSES WITH (never replaces) the robot's own PairingAuthorizer
// accept: the deny test uses a PAIRED robot (its pairing gate WOULD admit), so a
// refusal is provably the demand-authorization gate's doing, refused BEFORE the robot is even dialed.
// The AllowAll default admits + delivers byte-identically (anti-regression).
// ---------------------------------------------------------------------------

/// A hand-oracle authorizer that DENIES exactly one WAN topic (and asserts the subject
/// IS the WAN variant carrying a device key — the real demander plumbing) — the stand-in
/// for the account authorizer.
struct DenyWanTopic(String);
impl DemandAuthorizer for DenyWanTopic {
    fn authorize_demand(&self, subject: &DemandSubject, topic: &str) -> DemandDecision {
        // The WAN plane threads its subject as the paired robot's pinned key.
        assert!(
            matches!(subject, DemandSubject::Wan { .. }),
            "the WAN demand-grant must pass a Wan subject, got {subject:?}"
        );
        if topic == self.0 {
            DemandDecision::deny(format!("test: {topic} not authorized"))
        } else {
            DemandDecision::Allow
        }
    }
}

/// A SPY authorizer that ADMITS everything (AllowAll semantics) but COUNTS every
/// consultation and records whether it was passed the WAN subject — the anti-inert-
/// shipping oracle proving the installed authorizer is genuinely CONSULTED on the
/// grant path (not a dropped no-op).
struct SpyAuthorizer {
    calls: Arc<AtomicU64>,
    saw_wan: Arc<AtomicBool>,
}
impl DemandAuthorizer for SpyAuthorizer {
    fn authorize_demand(&self, subject: &DemandSubject, _topic: &str) -> DemandDecision {
        self.calls.fetch_add(1, Ordering::Relaxed);
        if matches!(subject, DemandSubject::Wan { .. }) {
            self.saw_wan.store(true, Ordering::Relaxed);
        }
        DemandDecision::Allow
    }
}

/// HEADLINE: with a deny-authorizer installed, a WAN demand for the denied topic is
/// refused with a LOUD `MirrorError::Iroh` naming the demand-authorization gate —
/// BEFORE any dial (`connection_count == 0`, `reader_count == 0`). The robot is PAIRED
/// (its own PairingAuthorizer WOULD admit), so the refusal is provably the demand-authorization gate's,
/// not the pairing gate's — and it fires without ever touching the network.
#[test]
fn c5c_wan_demand_authorizer_refuses_before_dial() {
    let desk_seed = unique_secret();
    // A PAIRED robot — the pairing gate would admit; only the demand-authorization gate refuses.
    let fx = RobotFixture::start("authdeny", Arc::new(claimed_paired(public_of(&desk_seed))));
    let manager_b = test_manager("authdeny_desk");
    let registry = Arc::new(one_robot_registry("ubuntu", &fx, desk_seed));
    let topic = format!("/wan/authdeny/{}", unique_id());
    let plane = IrohMirrorPlane::new(manager_b, registry)
        .expect("iroh plane")
        .with_dial_timeout(DIAL_TIMEOUT)
        .with_authorizer(Arc::new(DenyWanTopic(topic.clone())));
    let key = TopicKey::new("ubuntu", &topic);

    let err = plane
        .ensure_mirror(&key, ORACLE_SCHEMA_HASH)
        .expect_err("a demand the authorizer denies must be refused");
    match err {
        MirrorError::Iroh { reason, key: k } => {
            assert!(
                reason.contains("demand-authorization gate"),
                "the refusal names the demand-authorization gate: {reason}"
            );
            assert_eq!(k.robot, "ubuntu");
        }
        other => panic!("expected MirrorError::Iroh, got {other:?}"),
    }
    // Refused BEFORE any dial — no phantom connection / reader (the gate short-circuits
    // ahead of endpoint bind).
    assert_eq!(
        plane.connection_count(),
        0,
        "a gate refusal opens NO connection"
    );
    assert_eq!(
        plane.reader_count(),
        0,
        "no reader spawned on a gate refusal"
    );
    drop(plane);
}

/// SELECTIVITY + ANTI-REGRESSION: under a deny-authorizer that targets a DIFFERENT
/// topic, the demanded topic is admitted + delivers byte-identical frames (the gate is
/// selective, not a blanket deny — the deny-nothing plumbing withholds nothing on the
/// WAN plane). The "an explicitly-installed authorizer is genuinely CONSULTED on the
/// grant path" pin lives in `c5c_wan_authorizer_is_consulted_on_the_grant_path` (a spy
/// authorizer with an asserted call count — the anti-inert-shipping oracle).
#[test]
fn c5c_wan_allow_all_and_untargeted_deny_both_admit_and_deliver() {
    let desk_seed = unique_secret();
    let fx = RobotFixture::start("authallow", Arc::new(claimed_paired(public_of(&desk_seed))));
    let topic = format!("/wan/authallow/{}", unique_id());

    let stop = Arc::new(AtomicBool::new(false));
    let producer = spawn_producer(fx.manager.clone(), topic.clone(), stop.clone());

    let manager_b = test_manager("authallow_desk");
    let registry = Arc::new(one_robot_registry("ubuntu", &fx, desk_seed));
    // A deny-authorizer that targets a DIFFERENT topic → the demanded one is admitted.
    let plane = IrohMirrorPlane::new(manager_b.clone(), registry)
        .expect("iroh plane")
        .with_dial_timeout(DIAL_TIMEOUT)
        .with_authorizer(Arc::new(DenyWanTopic(
            "/wan/authallow/some-other".to_string(),
        )));
    let key = TopicKey::new("ubuntu", &topic);

    plane
        .ensure_mirror(&key, ORACLE_SCHEMA_HASH)
        .expect("an untargeted topic is admitted through the WAN gate");
    assert_eq!(plane.connection_count(), 1, "the admitted demand dialed");
    assert_eq!(plane.reader_count(), 1, "one live re-inject reader");

    // Delivery is byte-identical to the hand oracle (the gate did not perturb the wire).
    let collected = collect_frames(&manager_b, &topic, 5);
    assert_oracle_and_gap_free(&collected);

    stop.store(true, Ordering::Relaxed);
    let _ = producer.join();
    drop(plane);
}

/// Anti-inert-shipping: the WAN plane's authorizer is
/// genuinely CONSULTED on the GRANT path — not built-and-dropped. A SPY authorizer
/// that ADMITS everything (AllowAll semantics) but COUNTS its consultations is
/// installed via `with_authorizer`; the demand admits + delivers byte-identical
/// frames (proving the spy is functionally AllowAll — the wire is unperturbed) AND
/// the spy's call count is `>= 1` with the WAN subject observed (proving the
/// installed authorizer was actually reached on the grant path). Replaces the
/// former vacuous `let _explicit = Arc::new(AllowAllAuthorizer)` tail (which built +
/// dropped an authorizer without ever exercising it).
#[test]
fn c5c_wan_authorizer_is_consulted_on_the_grant_path() {
    let desk_seed = unique_secret();
    let fx = RobotFixture::start("authspy", Arc::new(claimed_paired(public_of(&desk_seed))));
    let topic = format!("/wan/authspy/{}", unique_id());

    let stop = Arc::new(AtomicBool::new(false));
    let producer = spawn_producer(fx.manager.clone(), topic.clone(), stop.clone());

    let manager_b = test_manager("authspy_desk");
    let registry = Arc::new(one_robot_registry("ubuntu", &fx, desk_seed));
    let calls = Arc::new(AtomicU64::new(0));
    let saw_wan = Arc::new(AtomicBool::new(false));
    let plane = IrohMirrorPlane::new(manager_b.clone(), registry)
        .expect("iroh plane")
        .with_dial_timeout(DIAL_TIMEOUT)
        .with_authorizer(Arc::new(SpyAuthorizer {
            calls: Arc::clone(&calls),
            saw_wan: Arc::clone(&saw_wan),
        }));
    let key = TopicKey::new("ubuntu", &topic);

    plane
        .ensure_mirror(&key, ORACLE_SCHEMA_HASH)
        .expect("the spy authorizer admits (AllowAll semantics)");
    assert_eq!(plane.connection_count(), 1, "the admitted demand dialed");
    assert_eq!(plane.reader_count(), 1, "one live re-inject reader");

    // The anti-inert-shipping oracle: the installed authorizer was genuinely reached
    // on the grant path (a dropped/no-op authorizer would leave the count at 0), and
    // it was handed the WAN subject.
    assert!(
        calls.load(Ordering::Relaxed) >= 1,
        "the explicitly-installed authorizer must be CONSULTED on the grant path (was it dropped?)"
    );
    assert!(
        saw_wan.load(Ordering::Relaxed),
        "the WAN grant path must pass a Wan subject to the authorizer"
    );

    // AllowAll semantics = byte-identical delivery (the spy did not perturb the wire).
    let collected = collect_frames(&manager_b, &topic, 5);
    assert_oracle_and_gap_free(&collected);

    stop.store(true, Ordering::Relaxed);
    let _ = producer.join();
    drop(plane);
}

// ---------------------------------------------------------------------------
// The DESK-SIDE epoch push. Without these, deleting the `push_epoch` call
// site in `dial_robot` leaves the whole suite green (every other test has no epoch
// dir, so the push short-circuits before it does anything).
// ---------------------------------------------------------------------------

/// One driven dial: the blobs the robot received, the plane (for its recorded
/// outcome), and the robot's runtime (kept alive for the assertions' lifetime).
struct DrivenDial {
    epochs: Arc<std::sync::Mutex<Vec<String>>>,
    plane: IrohMirrorPlane,
    _rt: tokio::runtime::Runtime,
}

impl DrivenDial {
    /// The blobs the stub robot received on its epoch stream.
    fn received(&self) -> Vec<String> {
        self.epochs.lock().unwrap().clone()
    }
    /// The plane's recorded outcome for the dialed robot.
    fn outcome(&self) -> Option<cerulion_netd::EpochPushOutcome> {
        self.plane.last_push_outcome("ubuntu")
    }
}

/// Stand up a recording robot + a plane whose registry has `epoch_dir`, then demand
/// one topic — which forces the dial, hence the push. Reaching the return at all means
/// the MIRROR CAME UP, so every caller doubles as a "the push did not block the
/// connection" pin.
fn drive_dial_with_epoch_dir(
    tag: &str,
    epoch_dir: Option<std::path::PathBuf>,
    accept_epoch: EpochReply,
) -> DrivenDial {
    let (driven, result) = try_drive_dial_with_epoch_dir(tag, epoch_dir, accept_epoch);
    result.expect("demand succeeds — the epoch push must never block the connection");
    driven
}

/// The same driver WITHOUT the success expectation — for the one fatal class (a peer
/// that stalls mid-push desyncs the control framing, so the dial itself fails).
fn try_drive_dial_with_epoch_dir(
    tag: &str,
    epoch_dir: Option<std::path::PathBuf>,
    accept_epoch: EpochReply,
) -> (DrivenDial, Result<(), MirrorError>) {
    let rt = tokio::runtime::Runtime::new().expect("robot runtime");
    let manager_b = test_manager(tag);
    let topic = format!("/wan/epoch/{}/{}", tag, unique_id());
    let epochs: Arc<std::sync::Mutex<Vec<String>>> = Arc::default();

    let daemon = rt.block_on(disabled_endpoint(unique_secret()));
    let eid = daemon.id();
    let addrs = rt.block_on(async { daemon.bound_sockets() });
    let addrs = loopback_sockets(&addrs);
    let _serve = rt.spawn(recording_robot_recording_epochs(
        daemon,
        vec![(topic.clone(), ORACLE_SCHEMA_HASH)],
        Arc::default(),
        epochs.clone(),
        accept_epoch,
    ));

    let mut robots = HashMap::new();
    robots.insert(
        "ubuntu".to_string(),
        WanRobot {
            eid,
            direct_addrs: addrs,
        },
    );
    let registry = Arc::new(
        WanRegistry::new(robots, unique_secret(), RelayConfig::Disabled).with_epoch_dir(epoch_dir),
    );
    let plane = IrohMirrorPlane::new(manager_b, registry)
        .expect("iroh plane")
        .with_dial_timeout(DIAL_TIMEOUT);
    // The demand forces the dial (and therefore the push).
    let result = plane.ensure_mirror(&TopicKey::new("ubuntu", &topic), ORACLE_SCHEMA_HASH);
    (
        DrivenDial {
            epochs,
            plane,
            _rt: rt,
        },
        result,
    )
}

/// Write a real cached epoch artifact for `robot` into a fresh temp dir.
fn epoch_dir_with_artifact(robot: &str) -> (tempfile::TempDir, String) {
    use cerulion_pairing::format::*;
    let dir = tempfile::tempdir().expect("tempdir");
    let inter_sk = ed25519_dalek::SigningKey::from_bytes(&[10u8; 32]);
    let root_sk = ed25519_dalek::SigningKey::from_bytes(&[1u8; 32]);
    let inter_pk = PublicKey(inter_sk.verifying_key().to_bytes());
    let validity = Validity {
        not_before_ns: 0,
        not_after_ns: 100_000_000_000_000,
    };
    let wire = cerulion_pairing::verify::EpochSyncWire::new(
        IntermediateCert {
            version: FORMAT_VERSION,
            intermediate_key: inter_pk,
            validity,
            issued_at_ns: 5,
            max_scope: Scope::OWNER_FULL,
        }
        .sign_by_roots(&[&root_sk]),
        AccessListEpoch {
            version: FORMAT_VERSION,
            robot: RobotId([0x0B; 32]),
            epoch: 5,
            revoked_accounts: vec![AccountId([0x0C; 32])],
            revoked_devices: vec![],
            issued_at_ns: 6,
            issuer_key: inter_pk,
        }
        .sign(&inter_sk),
    );
    let artifact = cerulion_wireclient::config::encode_epoch_cache(&wire).expect("encode");
    std::fs::write(
        dir.path()
            .join(cerulion_wireclient::config::epoch_cache_file_name(robot)),
        &artifact,
    )
    .expect("write the cache");
    (dir, artifact)
}

#[test]
fn a_dial_pushes_the_cached_epoch_and_records_the_applied_outcome() {
    // THE headline desk-side pin: a cached artifact is actually SENT on the dial, the
    // robot receives it BYTE-IDENTICAL to what the cache holds (a hand oracle — the
    // artifact this test wrote), and the plane records `Applied`.
    let (dir, artifact) = epoch_dir_with_artifact("ubuntu");
    let d = drive_dial_with_epoch_dir(
        "push_ok",
        Some(dir.path().to_path_buf()),
        EpochReply::Applied,
    );

    let received = d.received();
    assert_eq!(
        received.len(),
        1,
        "exactly ONE epoch push per dial (got {received:?})"
    );
    // The blob the robot got must decode to the SAME artifact the cache holds — the
    // desk is a courier and must not mutate it in transit.
    let sent = hex::decode(&received[0]).expect("the push is hex");
    let cached = {
        use base64::engine::general_purpose::URL_SAFE_NO_PAD;
        use base64::Engine;
        URL_SAFE_NO_PAD.decode(artifact.as_bytes()).unwrap()
    };
    assert_eq!(sent, cached, "the pushed bytes must be the cached artifact");
    let wire = cerulion_pairing::verify::EpochSyncWire::from_postcard(&sent).expect("decodes");
    assert_eq!(
        wire.signed_epoch.epoch_data.epoch, 5,
        "the hand-written epoch"
    );

    assert_eq!(
        d.outcome(),
        Some(cerulion_netd::EpochPushOutcome::Applied { epoch: 7 }),
        "an accepted push must be recorded as Applied (the robot reported epoch 7)"
    );
}

#[test]
fn a_refused_push_is_recorded_and_the_mirror_still_comes_up() {
    // The never-block contract on the REAL desk path: the robot refuses the epoch (a
    // sink-less / older robot) and the demand STILL succeeds — `drive_dial_with_epoch_dir`
    // asserts the mirror came up, so reaching this line is the pin.
    let (dir, _) = epoch_dir_with_artifact("ubuntu");
    let d = drive_dial_with_epoch_dir(
        "push_refused",
        Some(dir.path().to_path_buf()),
        EpochReply::NoSink,
    );
    assert_eq!(d.received().len(), 1, "it was still SENT");
    assert_eq!(
        d.outcome(),
        Some(cerulion_netd::EpochPushOutcome::NoSink),
        "a sink-less robot must be recorded as NoSink (upgrade the robot), NOT as a \
         Rejected epoch (investigate the epoch) — different operator actions"
    );
}

/// A robot PREDATING the `sync_epoch` verb answers its
/// malformed-request marker — NOT the no-sink needle, which only an epoch-sync-aware
/// build can emit. It must be recorded as `NoSink` ("upgrade the robot"), never as
/// `Rejected` ("investigate the epoch — forged, wrong robot, skewed clock").
///
/// Landing in the generic error arm would record `Rejected`, falsifying
/// both the `NoSink` doc ("including a robot predating the verb") and the operator
/// guidance in the warn. The REAL serde error text an older robot produces is
/// reproduced in `cerulion_connectd/tests/protocol_parity_test.rs`.
#[test]
fn an_older_robot_is_recorded_as_no_sink_not_as_a_rejected_epoch() {
    let (dir, _) = epoch_dir_with_artifact("ubuntu");
    let d = drive_dial_with_epoch_dir(
        "push_older",
        Some(dir.path().to_path_buf()),
        EpochReply::OlderRobot,
    );
    assert_eq!(d.received().len(), 1, "it was still SENT");
    assert_eq!(
        d.outcome(),
        Some(cerulion_netd::EpochPushOutcome::NoSink),
        "a robot that cannot DECODE the verb must be recorded as NoSink (upgrade it), \
         never as a Rejected epoch (investigate it) — different operator actions"
    );
}

/// The remaining desk-observable outcomes, each driven over the REAL dial path:
/// `AlreadyCurrent` (the steady state), `Rejected` (a genuine epoch refusal), and
/// `NotDelivered` (an unexpected reply shape). Previously only `Applied` / `NoSink`
/// were ever asserted, so three of the enum's arms shipped unexercised.
#[test]
fn the_remaining_push_outcomes_are_each_recorded_and_never_block_the_mirror() {
    for (tag, reply, expected) in [
        (
            "push_current",
            EpochReply::AlreadyCurrent,
            cerulion_netd::EpochPushOutcome::AlreadyCurrent { epoch: 12 },
        ),
        (
            "push_rejected",
            EpochReply::Rejected,
            cerulion_netd::EpochPushOutcome::Rejected,
        ),
        (
            "push_unexpected",
            EpochReply::UnexpectedReply,
            cerulion_netd::EpochPushOutcome::NotDelivered,
        ),
    ] {
        let (dir, _) = epoch_dir_with_artifact("ubuntu");
        // Reaching past `drive_dial_with_epoch_dir` at all is the never-block pin: it
        // asserts the mirror came up.
        let d = drive_dial_with_epoch_dir(tag, Some(dir.path().to_path_buf()), reply);
        assert_eq!(d.received().len(), 1, "{tag}: it was still SENT");
        assert_eq!(d.outcome(), Some(expected), "{tag}: recorded outcome");
    }
}

/// An OVERSIZED cached artifact is a POLICY failure, not a permanent dial failure.
///
/// Nothing bounded the cached artifact's size: a blob whose control frame exceeds the
/// peer's 16 MiB frame cap was written fine by the desk, REFUSED by the robot's
/// reader, and desynced its control stream — so the push Err'd, `dial_robot`
/// propagated it, and the mirror was NEVER created, on every dial, permanently. The
/// size guard turns that into `CacheTooLarge` + a loud warn, with the mirror still
/// coming up (the never-block-on-epoch-freshness invariant).
#[test]
fn an_oversized_cache_is_a_policy_failure_and_the_mirror_still_comes_up() {
    use cerulion_pairing::format::*;

    let (dir, artifact) = epoch_dir_with_artifact("ubuntu");
    // Re-write the SAME artifact padded past the frame cap (it still decodes as a
    // real artifact — the guard must bite on SIZE, not on malformedness).
    let mut wire = cerulion_wireclient::config::decode_epoch_cache(&artifact).expect("decode");
    wire.signed_epoch.epoch_data.revoked_accounts =
        (0..600_000u32).map(|i| AccountId([i as u8; 32])).collect();
    std::fs::write(
        dir.path()
            .join(cerulion_wireclient::config::epoch_cache_file_name("ubuntu")),
        cerulion_wireclient::config::encode_epoch_cache(&wire).expect("encode"),
    )
    .expect("write the oversized cache");

    let d = drive_dial_with_epoch_dir(
        "push_huge",
        Some(dir.path().to_path_buf()),
        EpochReply::Applied,
    );
    assert!(
        d.received().is_empty(),
        "an oversized artifact must never be SENT (it would desync the robot's control \
         stream and fail this dial and every later one)"
    );
    match d.outcome() {
        Some(cerulion_netd::EpochPushOutcome::CacheTooLarge { bytes }) => assert!(
            bytes > cerulion_link::DEFAULT_MAX_FRAME_LEN,
            "the recorded size must be the over-cap frame size, got {bytes}"
        ),
        other => panic!("expected a CacheTooLarge outcome, got {other:?}"),
    }
}

#[test]
fn a_corrupt_cache_pushes_nothing_and_the_mirror_still_comes_up() {
    // A corrupt cache must NOT block the connection (the loud-but-continue arm).
    let dir = tempfile::tempdir().expect("tempdir");
    std::fs::write(
        dir.path()
            .join(cerulion_wireclient::config::epoch_cache_file_name("ubuntu")),
        "this is not a valid epoch artifact",
    )
    .expect("write junk");
    let d = drive_dial_with_epoch_dir(
        "push_corrupt",
        Some(dir.path().to_path_buf()),
        EpochReply::Applied,
    );
    assert!(
        d.received().is_empty(),
        "a corrupt cache must push NOTHING (never a malformed blob)"
    );
    assert_eq!(
        d.outcome(),
        Some(cerulion_netd::EpochPushOutcome::CacheUnreadable),
        "the desk must record that it is NOT carrying revocations for this robot"
    );
}

#[test]
fn an_empty_cache_dir_and_no_cache_dir_both_push_nothing_and_are_distinguished() {
    // The two benign no-push states must be told APART in the recorded outcome — an
    // empty dir (possibly a filename mismatch, worth investigating) vs no dir at all.
    let dir = tempfile::tempdir().expect("tempdir");
    let d = drive_dial_with_epoch_dir(
        "push_empty",
        Some(dir.path().to_path_buf()),
        EpochReply::Applied,
    );
    assert!(d.received().is_empty());
    assert_eq!(
        d.outcome(),
        Some(cerulion_netd::EpochPushOutcome::NoCachedEpoch)
    );

    let d2 = drive_dial_with_epoch_dir("push_nodir", None, EpochReply::Applied);
    assert!(d2.received().is_empty());
    assert_eq!(
        d2.outcome(),
        Some(cerulion_netd::EpochPushOutcome::NoCacheDir),
        "no cache DIRECTORY is a different (and less alarming) state than an empty one"
    );
}

/// The ONE FATAL class: a peer that RECEIVES the push
/// and then never answers. `TransportFailed` was written by production and asserted by
/// no test, so nothing pinned that the fatal path is reached, that it is recorded, or
/// that it fails the dial instead of silently handing out a desynced connection.
///
/// The stub robot answers the catalog (so the dial is admitted and reaches the push),
/// swallows the `sync_epoch` blob, and then holds the connection open in silence. The
/// desk's bounded control read must time out, the dial must FAIL (the cancel-unsafe
/// framing is desynced — every later demand on that stream would fail anyway), and the
/// recorded outcome must read `TransportFailed` — never stale-successful (Principle #3).
#[test]
fn a_stalled_push_fails_the_dial_and_is_recorded_as_transport_failed() {
    let (dir, _) = epoch_dir_with_artifact("ubuntu");
    let (d, result) = try_drive_dial_with_epoch_dir(
        "push_stall",
        Some(dir.path().to_path_buf()),
        EpochReply::StallForever,
    );

    let err = result.expect_err("a stalled push desyncs the control stream — the dial must FAIL");
    let msg = err.to_string();
    assert!(
        msg.contains("revocation-epoch push"),
        "the dial failure must name the push as its cause: {msg}"
    );

    // The robot really did RECEIVE the artifact (so this is a stalled ANSWER, not a
    // push that never went out) — the anti-tautology half.
    assert_eq!(
        d.received().len(),
        1,
        "the stub must have received exactly the one pushed blob"
    );
    assert_eq!(
        d.outcome(),
        Some(cerulion_netd::EpochPushOutcome::TransportFailed),
        "the fatal class must be RECORDED, so the observable never reads \
         stale-successful after a failed push"
    );
}

// ===========================================================================
// The REAL-ROBOT push arm.
//
// The four arms above drive the hand-rolled stub robot, which proves the DESK half
// (what is sent, what each answer is classified as). This one drives the REAL
// `cerulion_remoted::WirePlane` with a REAL `SharedTrust` epoch sink over the same
// loopback iroh — so "the epoch LANDS robot-side" is proven behaviorally, on the
// robot's own live trust store, rather than inferred from a stub's reply.
// ===========================================================================

/// The root signing key the epoch fixture chains to.
fn live_root_sk() -> ed25519_dalek::SigningKey {
    ed25519_dalek::SigningKey::from_bytes(&[1u8; 32])
}

fn live_inter_sk() -> ed25519_dalek::SigningKey {
    ed25519_dalek::SigningKey::from_bytes(&[10u8; 32])
}

fn live_pk(k: &ed25519_dalek::SigningKey) -> PublicKey {
    PublicKey(k.verifying_key().to_bytes())
}

/// The PROBE device key the pushed epoch revokes — a third party, so revoking it
/// never disturbs netd's own session. Unbound ⇒ `Unpaired` before the push,
/// `DeviceRevoked` after (`is_device_revoked` is checked before any account row).
const LIVE_PROBE_DEVICE_KEY: [u8; 32] = [0xB7; 32];

/// A CLAIMED store whose root set is the REAL `live_root_sk` public half, so a
/// genuinely signed epoch verifies (the file's other `claimed_store` uses a
/// placeholder root and would refuse every real epoch).
fn claimed_store_for_live_epochs() -> TrustStore {
    let root_set = RootSet::new(vec![live_pk(&live_root_sk())], 1).unwrap();
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

/// A REAL signed artifact for robot `RobotId([5;32])` revoking [`LIVE_PROBE_DEVICE_KEY`].
fn live_epoch_artifact(n: u64) -> String {
    use cerulion_pairing::format::*;
    let inter = live_inter_sk();
    let inter_pk = live_pk(&inter);
    let wire = cerulion_pairing::verify::EpochSyncWire::new(
        IntermediateCert {
            version: FORMAT_VERSION,
            intermediate_key: inter_pk,
            validity: Validity {
                not_before_ns: 0,
                not_after_ns: T_NOW * 10,
            },
            issued_at_ns: 5,
            max_scope: Scope::OWNER_FULL,
        }
        .sign_by_roots(&[&live_root_sk()]),
        AccessListEpoch {
            version: FORMAT_VERSION,
            robot: RobotId([5; 32]),
            epoch: n,
            revoked_accounts: vec![],
            revoked_devices: vec![PublicKey(LIVE_PROBE_DEVICE_KEY)],
            issued_at_ns: 6,
            issuer_key: inter_pk,
        }
        .sign(&inter),
    );
    cerulion_wireclient::config::encode_epoch_cache(&wire).expect("encode")
}

/// THE ROBOT-SIDE PIN: netd's dial pushes the cached epoch to a REAL robot wire
/// plane, and the robot APPLIES it — observed on the robot's OWN live trust store
/// (the probe device flips `Unpaired` → `DeviceRevoked`), not on a stub's reply.
///
/// The ANTI-TAUTOLOGY control is the BEFORE assertion in the same body: the probe is
/// asserted `Unpaired` on the same live store immediately before the dial, so the
/// flip is the push's doing and not a fixture that starts out revoked.
#[test]
fn a_real_robot_applies_the_pushed_epoch_from_a_netd_dial() {
    let desk_seed = unique_secret();
    let desk_key = public_of(&desk_seed);

    // A REAL robot: its ONE SharedTrust handle is the accept gate, the demand
    // authorizer AND the epoch sink (the production arrangement).
    let mut index = DeviceAccountIndex::new();
    index.bind(PublicKey(desk_key), OWNER);
    let shared = SharedTrust::new(claimed_store_for_live_epochs(), index, Vec::new());
    let authz = Arc::new(PairingAuthorizer::from_shared(shared.clone()));

    let rt = tokio::runtime::Runtime::new().expect("robot runtime");
    let robot_manager = test_manager("live_epoch_robot");
    let plane = Arc::new(
        WirePlane::with_manager("live-epoch-robot", robot_manager.clone())
            .with_demand_authorizer(authz.clone())
            .with_epoch_sink(shared.clone(), cerulion_remoted::RemotedClock::fixed(T_NOW)),
    );
    let daemon = rt.block_on(disabled_endpoint(unique_secret()));
    let eid = daemon.id();
    let addrs = loopback_sockets(&daemon.bound_sockets());
    let _serve = rt.spawn(robot_serve_loop(daemon, authz, plane));

    // The robot publishes a topic netd can demand (the demand is what forces the dial).
    let topic = format!("/wan/epoch/live/{}", unique_id());
    let stop = Arc::new(AtomicBool::new(false));
    let producer = spawn_producer(robot_manager.clone(), topic.clone(), stop.clone());

    assert_eq!(
        shared.snapshot_for_key(&LIVE_PROBE_DEVICE_KEY).1,
        cerulion_remoted::KeyAccess::Unpaired,
        "precondition: the probe device is NOT revoked before the push"
    );

    // The desk caches a REAL signed epoch for this robot, keyed by the name netd
    // dials it by.
    let dir = tempfile::tempdir().expect("tempdir");
    std::fs::write(
        dir.path()
            .join(cerulion_wireclient::config::epoch_cache_file_name("ubuntu")),
        live_epoch_artifact(9),
    )
    .expect("write the cache");

    let mut robots = HashMap::new();
    robots.insert(
        "ubuntu".to_string(),
        WanRobot {
            eid,
            direct_addrs: addrs,
        },
    );
    let registry = Arc::new(
        WanRegistry::new(robots, desk_seed, RelayConfig::Disabled)
            .with_epoch_dir(Some(dir.path().to_path_buf())),
    );
    let desk_plane = IrohMirrorPlane::new(test_manager("live_epoch_desk"), registry)
        .expect("iroh plane")
        .with_dial_timeout(DIAL_TIMEOUT);

    desk_plane
        .ensure_mirror(&TopicKey::new("ubuntu", &topic), ORACLE_SCHEMA_HASH)
        .expect("the demand (and therefore the dial + push) succeeds");

    assert_eq!(
        desk_plane.last_push_outcome("ubuntu"),
        Some(cerulion_netd::EpochPushOutcome::Applied { epoch: 9 }),
        "the REAL robot must report the epoch APPLIED"
    );
    assert_eq!(
        shared.snapshot_for_key(&LIVE_PROBE_DEVICE_KEY).1,
        cerulion_remoted::KeyAccess::DeviceRevoked,
        "the epoch must be IN FORCE on the robot's live trust store — the whole point \
         of the push (a stub reply alone proves nothing landed)"
    );

    stop.store(true, Ordering::Relaxed);
    let _ = producer.join();
    drop(desk_plane);
}

// ---------------------------------------------------------------------------
// (11) The desk ACCOUNT is resolved AT THE DIAL, the
//      production first-use point the account deferral assumed but did not have.
// ---------------------------------------------------------------------------

/// `WanRegistry::account()` (the cached device-cert read + I1 ed25519
/// key match) was deferred off netd's boot path to "first use". But nothing in production called it,
/// so "first use" was really NEVER — and the classifying diagnostics inside
/// `resolve_account_from`, including the WARN for a stale/foreign cert or a
/// `CERULION_NETD_DESK_KEY` pointing at a different key than the one that logged in,
/// were unreachable in a shipped daemon.
///
/// `IrohMirrorPlane::dial_robot` resolves it before that dial's network I/O, so a
/// misconfiguration is classified ABOVE the catalog-gate refusal it causes.
///
/// The registry here carries REAL desk paths (`with_desk_paths` — the DI twin of what
/// `from_env` captures) pointing at a REAL cached device cert bound to this desk's seed,
/// so `resolve_account_from` — the function that owns those diagnostics — genuinely
/// runs. A `WanRegistry::new` fixture would have `desk_paths: None`, short-circuit
/// before that function, and pin nothing about it.
///
/// The observable is `WanRegistry::account_resolved` (the state of the `OnceLock` that
/// HOLDS the deferred work), reached through the plane's delegating
/// `desk_account_resolved`. A plane-side "we called it" flag could not tell
/// resolved-at-dial from resolved-at-boot; the cell can.
///
/// Arms, over a REAL loopback dial:
///
/// - (a) constructing the registry resolves nothing;
/// - (b) constructing the PLANE over it resolves nothing either (fails if a
///   `registry.account()` call is moved into `IrohMirrorPlane::new` / `from_env`);
/// - (c) after one successful dial it IS resolved (fails if `resolve_dial_account` is
///   dropped from `dial_robot` — that would make the diagnostics unreachable);
/// - (d) and it resolved to the cert's ACTUAL account, so `resolve_account_from` really
///   read the file rather than short-circuiting to `None`.
///
/// (The BOOT half — that netd's real `from_env` constructor defers this too — is pinned
/// by `wan.rs`'s `from_env_leaves_the_account_cell_unresolved_until_first_use`, which
/// needs the process env and so cannot live in this parallel-safe file.)
#[test]
fn the_desk_account_is_resolved_at_the_dial_not_before() {
    let desk_seed = unique_secret();
    let authz = Arc::new(claimed_paired(public_of(&desk_seed)));
    let fx = RobotFixture::start("acct", authz);
    let topic = format!("/wan/acct/{}", unique_id());

    let stop = Arc::new(AtomicBool::new(false));
    let producer = spawn_producer(fx.manager.clone(), topic.clone(), stop.clone());

    // A REAL cached device cert bound to THIS desk seed, at the sibling path the desk
    // key file implies — exactly the production shape `cerulion login` writes.
    let certdir = tempfile::tempdir().expect("tempdir");
    let account = [0x7Cu8; 32];
    std::fs::write(
        certdir.path().join("device.cert"),
        make_device_cert_b64(public_of(&desk_seed), account),
    )
    .expect("write device.cert");

    let manager_b = test_manager("acct_desk");
    let registry = Arc::new(
        one_robot_registry("ubuntu", &fx, desk_seed).with_desk_paths(
            cerulion_netd::DeskPathInputs {
                key_file: Some(certdir.path().join("desk.key")),
                ..Default::default()
            },
        ),
    );

    // (a) Building the registry reads no cert.
    assert!(
        !registry.account_resolved(),
        "building the WAN registry must not resolve the desk account"
    );

    let plane = IrohMirrorPlane::new(manager_b.clone(), registry.clone())
        .expect("iroh plane")
        .with_dial_timeout(DIAL_TIMEOUT);

    // (b) Building the PLANE over it reads no cert either.
    assert!(
        !plane.desk_account_resolved(),
        "a netd that has not dialed must not have read the device cert"
    );

    plane
        .ensure_mirror(&TopicKey::new("ubuntu", &topic), ORACLE_SCHEMA_HASH)
        .expect("the demand (and therefore the dial) succeeds");

    // (c) The dial resolved it — so `resolve_account_from`'s device-cert diagnostics
    //     are reachable on the production path.
    assert!(
        plane.desk_account_resolved(),
        "the dial must resolve the desk account, or the device-cert \
         diagnostics are unreachable in a shipped daemon"
    );

    // (d) …and it resolved the cert we wrote — the resolver READ the file (a
    //     short-circuit to `None`, or a resolution that never happened, fails here).
    //     Safe to read now: (c) already proved the cell was initialized by the dial, so
    //     this call cannot be what resolves it.
    assert_eq!(
        registry.account(),
        Some(account),
        "the dial resolved the account bound to this desk's cached device cert"
    );

    stop.store(true, Ordering::Relaxed);
    let _ = producer.join();
    drop(plane);
}

#[path = "wan_plane_iroh_test/owner_pair.rs"]
mod owner_pair;

#[path = "wan_plane_iroh_test/poison_retirement.rs"]
mod poison_retirement;

#[path = "wan_plane_iroh_test/serving_plane.rs"]
mod serving_plane;
