// SPDX-License-Identifier: AGPL-3.0-only
//! The desk-side RE-INJECT client end-to-end over REAL iroh
//! loopback endpoints + REAL iceoryx2 (DISTINCT per-test SHM roots for the robot
//! and the desk — parallel-safe, no `#[serial]`).
//!
//! The headline: an in-process ROBOT (`cerulion_remoted::WirePlane` +
//! `PairingAuthorizer` over SHM root A, a real producer publishing hand-oracle
//! frames) ↔ the in-process DESK client ([`cerulion_connectd::run_connect`] on
//! DISTINCT SHM root B) over real loopback iroh. The desk DEMANDS the topic, the
//! robot forwards, the desk RE-INJECTS into B, and a desk-local subscriber reads
//! frames BYTE-IDENTICAL to a HAND oracle (each frame recomputed from its own wire
//! `sequence` — never a self-compare, Principle #13).
//!
//! Crib `cerulion_remoted/tests/wire_plane_test.rs` (the robot serve harness) +
//! `cerulion_core/tests/network_ingress_e2e_test.rs` (the cross-manager re-inject
//! shape), but iroh instead of zenoh.

use std::future::Future;
use std::net::{Ipv4Addr, Ipv6Addr, SocketAddr};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use cerulion_connectd::config::{ConnectConfig, DemandSet};
use cerulion_connectd::protocol::{StreamPreamble, WireRequest, WireResponse};
use cerulion_connectd::worker::{run_connect, run_topic_reader, TopicReinjectCounters};
use cerulion_connectd::ConnectError;
use cerulion_core::transport::cerulion_q::{
    CatalogEntry, CatalogProvenance, CatalogReply, SchemaDoc, SchemaEncoding, SchemaReply,
    CATALOG_WIRE_VERSION,
};
use cerulion_core::transport::subscriber::OwnedInboundSample;
use cerulion_core::wire::{MaxSliceLen, WireHeader};
use cerulion_core::{TransportConfig, TransportManager};
use cerulion_link::{
    accept_frame_stream, accept_one, accept_uni_frame_stream, alpn, build_endpoint, dial,
    open_uni_frame_stream, read_frame, write_frame, Endpoint, EndpointAddr, EndpointConfig,
    RelayConfig, DEFAULT_MAX_FRAME_LEN,
};
use cerulion_pairing::client::DeviceIdentity;
use cerulion_pairing::format::{AccountId, PrincipalKind, PublicKey, RobotId, RootSet};
use cerulion_pairing::verify::TrustStore;
use cerulion_remoted::{
    handle_accepted_with_wire, DeviceAccountIndex, PairingAuthorizer, RemotedClock, SharedTrust,
    WirePlane,
};

const STEP_TIMEOUT: Duration = Duration::from_secs(25);
const T_NOW: u64 = 1_000_000_000_000;
const CHASSIS: &[u8] = b"connect-chassis-secret-not-serial-derived";
const OWNER: AccountId = AccountId([10; 32]);
/// The desk's device seed — its public half is bound onto the robot's access list
/// (paired), so the wire plane admits it.
const DESK_SEED: [u8; 32] = [77; 32];
/// The hand-chosen wire `schema_hash` the oracle frames carry.
const ORACLE_SCHEMA_HASH: u64 = 0xCE82_2406_D00D_F00D_u64;
/// A DIFFERENT hash for the rejection arm (a schema-mismatched frame).
const MISMATCH_SCHEMA_HASH: u64 = 0x0BAD_0BAD_0BAD_0BAD_u64;

/// A no-op catalog sink for `run_connect` (tests don't print the catalog). A
/// named fn item (implements `FnMut(&CatalogReply)`) sidesteps closure inference.
fn ignore_catalog(_: &cerulion_core::transport::cerulion_q::CatalogReply) {}

async fn bounded<F: Future>(what: &str, fut: F) -> F::Output {
    match tokio::time::timeout(STEP_TIMEOUT, fut).await {
        Ok(v) => v,
        Err(_) => panic!("connect e2e: '{what}' timed out after {STEP_TIMEOUT:?}"),
    }
}

fn unique_id() -> String {
    static N: AtomicU64 = AtomicU64::new(0);
    format!(
        "{}_{}",
        std::process::id(),
        N.fetch_add(1, Ordering::Relaxed)
    )
}

async fn disabled_endpoint(secret: [u8; 32]) -> Endpoint {
    bounded(
        "build_endpoint",
        build_endpoint(EndpointConfig::new(secret).with_relay(RelayConfig::Disabled)),
    )
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

/// The desk's public device key (== its iroh EndpointId bytes) from DESK_SEED.
fn desk_public_key() -> [u8; 32] {
    DeviceIdentity::from_seed(&DESK_SEED).public_key().0
}

/// A fresh per-test transport manager on its own SHM root (parallel-safe).
fn test_manager(tag: &str) -> Arc<TransportManager> {
    let ix = cerulion_core::testing::iceoryx_test_config();
    TransportManager::init_for_test(
        TransportConfig {
            node_name: format!("connect_{tag}_{}", unique_id()),
            ..Default::default()
        },
        ix,
    )
    .expect("init_for_test")
}

// ---- The hand oracle: each frame is a deterministic function of its sequence. ----

fn oracle_ts(seq: u32) -> u64 {
    1_000_000 + seq as u64 * 10_000
}

fn oracle_payload(seq: u32) -> Vec<u8> {
    let mut p = b"reinject".to_vec();
    p.extend_from_slice(&seq.to_le_bytes());
    p
}

/// Build a raw wire frame (32-byte LE header + payload). `publish_raw` publishes
/// these bytes VERBATIM, so the published bytes ARE the oracle bytes.
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

/// A topic-TAGGED oracle payload — so topic A's frames are DISTINGUISHABLE from
/// topic B's. A cross-wiring bug (one topic's frames re-injected under another's
/// name) makes the received bytes miss the tagged oracle → CAUGHT.
fn oracle_payload_tagged(tag: u8, seq: u32) -> Vec<u8> {
    let mut p = b"reinject".to_vec();
    p.push(tag);
    p.extend_from_slice(&seq.to_le_bytes());
    p
}

fn oracle_frame_tagged(tag: u8, seq: u32) -> Vec<u8> {
    make_wire_frame(
        ORACLE_SCHEMA_HASH,
        seq,
        oracle_ts(seq),
        &oracle_payload_tagged(tag, seq),
    )
}

/// Spawn a continuous producer publishing `oracle_frame(seq)` for seq = 0,1,2,…
/// on `topic` until `stop` is set. Returns the join handle.
fn spawn_producer(
    manager: Arc<TransportManager>,
    topic: String,
    stop: Arc<AtomicBool>,
) -> std::thread::JoinHandle<()> {
    spawn_producer_tagged(manager, topic, None, stop)
}

/// Spawn a continuous producer. `tag: Some(t)` publishes `oracle_frame_tagged(t,
/// seq)` (topic-distinguishable — for the multi-topic cross-wiring proof); `None`
/// publishes the plain `oracle_frame(seq)`.
fn spawn_producer_tagged(
    manager: Arc<TransportManager>,
    topic: String,
    tag: Option<u8>,
    stop: Arc<AtomicBool>,
) -> std::thread::JoinHandle<()> {
    std::thread::spawn(move || {
        let mut publisher = manager
            .create_publisher_simple(&topic, MaxSliceLen::const_new(256))
            .expect("producer");
        let mut seq = 0u32;
        while !stop.load(Ordering::Relaxed) {
            let frame = match tag {
                Some(t) => oracle_frame_tagged(t, seq),
                None => oracle_frame(seq),
            };
            let _ = publisher.publish_raw(&frame);
            seq = seq.wrapping_add(1);
            std::thread::sleep(Duration::from_millis(5));
        }
    })
}

/// Attach a desk-local data-only tap on `topic` (poll-retry until the ingress
/// service exists — connectd creates it on the first re-injected frame), then
/// collect `n` full frames + their sequences. Blocking — runs under
/// `spawn_blocking`.
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

/// The desk `ConnectConfig` dialing a robot at `daemon`, demanding `topics`.
/// `epoch_dir: None` ⇒ no revocation-epoch cache on this desk, so the push is
/// a no-op (the arms that DO push pass a real directory).
fn desk_config(
    daemon: &Endpoint,
    demand: DemandSet,
    schemas_dir: Option<std::path::PathBuf>,
) -> ConnectConfig {
    desk_config_with_epochs(daemon, demand, schemas_dir, None, None)
}

/// The same config with an explicit epoch-cache directory and the DESK's own
/// name for the robot (`robots.toml`), which is what keys the cache lookup — never the
/// name the robot reports about itself.
fn desk_config_with_epochs(
    daemon: &Endpoint,
    demand: DemandSet,
    schemas_dir: Option<std::path::PathBuf>,
    epoch_dir: Option<std::path::PathBuf>,
    desk_robot_name: Option<&str>,
) -> ConnectConfig {
    ConnectConfig {
        robot_eid: daemon.id(),
        direct_addrs: loopback_sockets(&daemon.bound_sockets()),
        demand,
        desk_seed: DESK_SEED,
        relay: RelayConfig::Disabled,
        schemas_dir,
        // A generous robot-response bound — the loopback path never hits it.
        robot_timeout: Duration::from_secs(30),
        epoch_dir,
        robot_name: desk_robot_name.map(str::to_string),
    }
}

/// Spawn the robot: accept ONE connection and serve it the wire plane.
fn spawn_robot(
    daemon: Endpoint,
    authz: Arc<PairingAuthorizer>,
    plane: Arc<WirePlane>,
) -> tokio::task::JoinHandle<()> {
    tokio::spawn(async move {
        if let Ok(Some(accepted)) = accept_one(&daemon).await {
            handle_accepted_with_wire(accepted, authz, None, Some(plane)).await;
        }
    })
}

// ---------------------------------------------------------------------------
// (1) HEADLINE: demand a topic → BYTE-IDENTICAL frames re-injected into desk SHM.
// ---------------------------------------------------------------------------

/// Run the full flow once and return the desk-collected (seq, frame) pairs.
async fn run_headline(tag: &str, n: usize) -> Vec<(u32, Vec<u8>)> {
    let manager_a = test_manager(&format!("{tag}A"));
    let manager_b = test_manager(&format!("{tag}B"));
    let topic = format!("/reinj/{tag}/{}", unique_id());

    let stop = Arc::new(AtomicBool::new(false));
    let producer = spawn_producer(manager_a.clone(), topic.clone(), stop.clone());

    let daemon = disabled_endpoint([200; 32]).await;
    let authz = Arc::new(claimed_paired(desk_public_key()));
    let plane = Arc::new(WirePlane::with_manager("robot-hdl", manager_a.clone()));
    let config = desk_config(&daemon, DemandSet::Named(vec![topic.clone()]), None);
    let robot = spawn_robot(daemon, authz, plane);

    let (shutdown_tx, shutdown_rx) = tokio::sync::oneshot::channel::<()>();
    let desk = tokio::spawn(run_connect(
        config,
        manager_b.clone(),
        ignore_catalog,
        async move {
            let _ = shutdown_rx.await;
        },
    ));

    // Collect the re-injected frames off the desk-local SHM (bounded).
    let reader_mgr = manager_b.clone();
    let reader_topic = topic.clone();
    let collected = bounded(
        "collect",
        tokio::task::spawn_blocking(move || collect_frames(&reader_mgr, &reader_topic, n)),
    )
    .await
    .expect("reader join");

    // Tear down: shutdown the desk, stop the producer, reap the robot.
    let _ = shutdown_tx.send(());
    let summary = bounded("desk join", desk)
        .await
        .expect("desk join")
        .expect("run_connect ok");
    assert!(
        summary
            .per_topic
            .iter()
            .any(|s| s.topic == topic && s.reinjected >= n as u64),
        "summary must report the re-injected topic: {:?}",
        summary.per_topic
    );
    stop.store(true, Ordering::Relaxed);
    let _ = producer.join();
    let _ = bounded("robot join", robot).await;

    collected
}

/// Assert the collected frames are BYTE-IDENTICAL to the hand oracle (by their own
/// sequence) and GAP-FREE from the first received (contiguous +1).
fn assert_oracle_and_gap_free(collected: &[(u32, Vec<u8>)]) {
    let first = collected[0].0;
    for (i, (seq, frame)) in collected.iter().enumerate() {
        assert_eq!(
            *seq,
            first + i as u32,
            "sequences must be gap-free from first-received"
        );
        assert_eq!(
            frame,
            &oracle_frame(*seq),
            "frame {seq} must be BYTE-IDENTICAL to the hand oracle"
        );
    }
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn reinject_delivers_byte_identical_frames() {
    let collected = run_headline("headline", 6).await;
    assert_oracle_and_gap_free(&collected);
}

/// Determinism (Principle #7): two full runs each match the hand oracle (each
/// received frame equals `oracle_frame(its_seq)` — NOT a self-compare of the two
/// runs, which could differ in start-seq under startup drops).
#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn reinject_is_deterministic() {
    let a = run_headline("det_a", 5).await;
    let b = run_headline("det_b", 5).await;
    assert_oracle_and_gap_free(&a);
    assert_oracle_and_gap_free(&b);
}

// ---------------------------------------------------------------------------
// (2) SCHEMA MATERIALIZATION: the demanded topic's .msg closure lands on disk.
// ---------------------------------------------------------------------------

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn schema_closure_is_materialized_into_the_desk_store() {
    use std::collections::BTreeMap;

    let manager_a = test_manager("schemaA");
    let manager_b = test_manager("schemaB");
    let topic = format!("/reinj/schema/{}", unique_id());

    let stop = Arc::new(AtomicBool::new(false));
    let producer = spawn_producer(manager_a.clone(), topic.clone(), stop.clone());

    // The robot serves a two-doc closure for the topic.
    let root_text = "acme/Sub sub\nuint32 seq\n";
    let sub_text = "float64 x\n";
    let mut topic_types = BTreeMap::new();
    topic_types.insert(topic.clone(), "acme/State".to_string());
    let mut docs = BTreeMap::new();
    docs.insert(
        "acme/State".to_string(),
        SchemaDoc {
            qualified: "acme/State".to_string(),
            encoding: SchemaEncoding::Msg,
            text: root_text.to_string(),
            deps: vec!["acme/Sub".to_string()],
        },
    );
    docs.insert(
        "acme/Sub".to_string(),
        SchemaDoc {
            qualified: "acme/Sub".to_string(),
            encoding: SchemaEncoding::Msg,
            text: sub_text.to_string(),
            deps: vec![],
        },
    );

    let schemas_dir = tempfile::tempdir().unwrap();
    let daemon = disabled_endpoint([201; 32]).await;
    let authz = Arc::new(claimed_paired(desk_public_key()));
    let plane = Arc::new(
        WirePlane::with_manager("robot-sch", manager_a.clone()).with_schema(topic_types, docs),
    );
    let config = desk_config(
        &daemon,
        DemandSet::Named(vec![topic.clone()]),
        Some(schemas_dir.path().to_path_buf()),
    );
    let robot = spawn_robot(daemon, authz, plane);

    let (shutdown_tx, shutdown_rx) = tokio::sync::oneshot::channel::<()>();
    let desk = tokio::spawn(run_connect(
        config,
        manager_b.clone(),
        ignore_catalog,
        async move {
            let _ = shutdown_rx.await;
        },
    ));

    // Poll for the materialized store files (bounded); assert byte-exact content.
    let state_msg = schemas_dir
        .path()
        .join("acme")
        .join("msg")
        .join("State.msg");
    let sub_msg = schemas_dir.path().join("acme").join("msg").join("Sub.msg");
    let deadline = Instant::now() + Duration::from_secs(20);
    loop {
        if state_msg.exists() && sub_msg.exists() {
            break;
        }
        assert!(Instant::now() < deadline, "schema files never materialized");
        tokio::time::sleep(Duration::from_millis(30)).await;
    }
    assert_eq!(std::fs::read_to_string(&state_msg).unwrap(), root_text);
    assert_eq!(std::fs::read_to_string(&sub_msg).unwrap(), sub_text);

    let _ = shutdown_tx.send(());
    let _ = bounded("desk join", desk).await;
    stop.store(true, Ordering::Relaxed);
    let _ = producer.join();
    let _ = bounded("robot join", robot).await;
}

// ---------------------------------------------------------------------------
// (3) REJECTION + reader contract via a HAND-FED uni stream (deterministic).
// ---------------------------------------------------------------------------

/// Feed `run_topic_reader` a controlled uni stream: 1 GOOD frame (establishes the
/// expected hash) then `bad` schema-mismatched frames, and assert the exact
/// counters. A direct uni stream (no wire plane) so the counts are DETERMINISTIC
/// (no drop-to-live). Hand oracle.
async fn run_reader_over_uni(tag: &str, good: usize, bad: usize) -> (u64, u64) {
    let manager = test_manager(tag);
    let topic = format!("/reader/{tag}/{}", unique_id());

    // sender (S, plays the robot) accepts; reader (R, plays the desk) dials.
    let sender = disabled_endpoint([210; 32]).await;
    let reader = disabled_endpoint([9; 32]).await;
    let mut sender_addr = EndpointAddr::new(sender.id());
    for s in loopback_sockets(&sender.bound_sockets()) {
        sender_addr = sender_addr.with_ip_addr(s);
    }

    let counters = Arc::new(TopicReinjectCounters::default());

    // Spawn the READER FIRST (it dials + accept_uni + drains until EOF). Feeding
    // the accepted uni stream directly to run_topic_reader (expected_hash=None ⇒ it
    // derives the hash from the first frame; the worker reads the preamble before
    // spawning the reader, so the reader sees only frames).
    let reader_topic = topic.clone();
    let reader_manager = manager.clone();
    let reader_counters = counters.clone();
    let reader_task = tokio::spawn(async move {
        let conn = dial(&reader, sender_addr, alpn::WIRE).await.expect("dial");
        let recv = accept_uni_frame_stream(&conn).await.expect("accept_uni");
        run_topic_reader(
            recv,
            reader_topic,
            "test-robot".to_string(),
            reader_manager,
            reader_counters,
            None,
        )
        .await;
    });

    // The sender accepts the reader's dial, opens a uni stream, and writes 1 GOOD
    // frame (establishes the expected hash) then `bad` schema-mismatched frames.
    let accepted = bounded("accept", accept_one(&sender))
        .await
        .expect("accept ok")
        .expect("incoming");
    let mut send = bounded("open_uni", open_uni_frame_stream(&accepted.connection))
        .await
        .expect("open_uni");
    for i in 0..good {
        bounded(
            "write good",
            write_frame(&mut send, &oracle_frame(i as u32)),
        )
        .await
        .expect("write good");
    }
    for i in 0..bad {
        let bad_frame = make_wire_frame(
            MISMATCH_SCHEMA_HASH,
            (good + i) as u32,
            oracle_ts((good + i) as u32),
            &oracle_payload((good + i) as u32),
        );
        bounded("write bad", write_frame(&mut send, &bad_frame))
            .await
            .expect("write bad");
    }
    let _ = send.finish();

    // Await the reader (it returns on the finish()-induced EOF), THEN drop the
    // sender connection — so the connection stays up through delivery.
    bounded("reader done", reader_task)
        .await
        .expect("reader join");
    drop(accepted);
    (counters.reinjected(), counters.rejected())
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn schema_mismatched_frames_are_rejected_and_counted() {
    // 1 good frame establishes the expected hash; 4 mismatched frames are refused.
    let (reinjected, rejected) = run_reader_over_uni("reject", 1, 4).await;
    assert_eq!(reinjected, 1, "the good frame is re-injected");
    assert_eq!(
        rejected, 4,
        "every schema-mismatched frame is rejected + counted"
    );
}

/// Anti-tautology control: all-good frames re-inject, zero rejected (the
/// apparatus moves — the rejection above is not vacuous).
#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn all_good_frames_reinject_none_rejected() {
    let (reinjected, rejected) = run_reader_over_uni("allgood", 5, 0).await;
    assert_eq!(reinjected, 5, "all good frames re-injected");
    assert_eq!(rejected, 0, "no rejections");
}

/// The connectd half: the PRODUCTION callsite
/// (`run_topic_reader` registering mirror PROVENANCE) is asserted e2e. After a
/// real re-inject of a topic attributed to robot `go2-ubuntu`, the desk manager's
/// `/__cerulion/mirrors` registry must carry `(topic → go2-ubuntu)` so
/// `topic list` folds the mirror into REMOTE. Keeps the manager alive (unlike the
/// shared `run_reader_over_uni` helper) so it can gather its own provenance.
#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn reinject_registers_mirror_provenance() {
    let manager = test_manager("provtag");
    let topic = format!("/reader/prov/{}", unique_id());
    let robot = "go2-ubuntu";

    let sender = disabled_endpoint([211; 32]).await;
    let reader = disabled_endpoint([12; 32]).await;
    let mut sender_addr = EndpointAddr::new(sender.id());
    for s in loopback_sockets(&sender.bound_sockets()) {
        sender_addr = sender_addr.with_ip_addr(s);
    }

    let counters = Arc::new(TopicReinjectCounters::default());
    let reader_topic = topic.clone();
    let reader_manager = manager.clone();
    let reader_counters = counters.clone();
    let reader_task = tokio::spawn(async move {
        let conn = dial(&reader, sender_addr, alpn::WIRE).await.expect("dial");
        let recv = accept_uni_frame_stream(&conn).await.expect("accept_uni");
        run_topic_reader(
            recv,
            reader_topic,
            robot.to_string(),
            reader_manager,
            reader_counters,
            None,
        )
        .await;
    });

    let accepted = bounded("accept", accept_one(&sender))
        .await
        .expect("accept ok")
        .expect("incoming");
    let mut send = bounded("open_uni", open_uni_frame_stream(&accepted.connection))
        .await
        .expect("open_uni");
    for i in 0..3 {
        bounded(
            "write good",
            write_frame(&mut send, &oracle_frame(i as u32)),
        )
        .await
        .expect("write good");
    }
    let _ = send.finish();
    bounded("reader done", reader_task)
        .await
        .expect("reader join");

    assert!(counters.reinjected() >= 1, "at least one frame re-injected");

    // The production callsite registered provenance: gather the desk manager's own
    // registry (bounded retry for the background republish + in-process warm-up).
    let deadline = Instant::now() + Duration::from_secs(5);
    let mut found = None;
    while Instant::now() < deadline {
        let snapshot = manager
            .gather_mirror_provenance(Duration::from_millis(300))
            .expect("gather provenance");
        if let Some(rec) = snapshot.iter().find(|r| r.topic == topic) {
            found = Some(rec.origin_robot.clone());
            break;
        }
    }
    assert_eq!(
        found.as_deref(),
        Some(robot),
        "run_topic_reader must register the mirror's provenance attributed to {robot}"
    );

    drop(accepted);
}

// ---------------------------------------------------------------------------
// (4) TEARDOWN: the robot disconnects mid-stream → the desk exits cleanly.
// ---------------------------------------------------------------------------

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn robot_disconnect_exits_cleanly_and_releases_ingress() {
    let manager_a = test_manager("teardownA");
    let manager_b = test_manager("teardownB");
    let topic = format!("/reinj/teardown/{}", unique_id());

    let stop = Arc::new(AtomicBool::new(false));
    let producer = spawn_producer(manager_a.clone(), topic.clone(), stop.clone());

    let daemon = disabled_endpoint([202; 32]).await;
    let authz = Arc::new(claimed_paired(desk_public_key()));
    let plane = Arc::new(WirePlane::with_manager("robot-td", manager_a.clone()));
    let config = desk_config(&daemon, DemandSet::Named(vec![topic.clone()]), None);

    // Own the daemon so we can CLOSE it to simulate the robot going away.
    let daemon = Arc::new(daemon);
    let daemon_serve = daemon.clone();
    let robot = tokio::spawn(async move {
        if let Ok(Some(accepted)) = accept_one(&daemon_serve).await {
            handle_accepted_with_wire(accepted, authz, None, Some(plane)).await;
        }
    });

    let desk = tokio::spawn(run_connect(
        config,
        manager_b.clone(),
        ignore_catalog,
        async {
            // Never self-shutdown — only the robot-drop tears this down.
            std::future::pending::<()>().await;
        },
    ));

    // Let some frames flow (the ingress service comes up on the first frame).
    let reader_mgr = manager_b.clone();
    let reader_topic = topic.clone();
    let _flowed = bounded(
        "collect a few",
        tokio::task::spawn_blocking(move || collect_frames(&reader_mgr, &reader_topic, 2)),
    )
    .await
    .expect("reader join");

    // The robot goes away: abort its serve task + CLOSE the endpoint.
    robot.abort();
    daemon.close().await;

    // The desk must exit CLEANLY (bounded — no hang) once the connection drops.
    let summary = bounded("desk exits on robot drop", desk)
        .await
        .expect("desk join")
        .expect("run_connect returns Ok on a clean disconnect");
    assert!(
        summary.per_topic.iter().any(|s| s.topic == topic),
        "summary carries the streamed topic"
    );

    // A fresh ingress injector on the same topic succeeds (the desk released it).
    manager_b
        .create_ingress_injector(&topic, ORACLE_SCHEMA_HASH, MaxSliceLen::const_new(256))
        .expect("a fresh ingress injector attaches after teardown");

    stop.store(true, Ordering::Relaxed);
    let _ = producer.join();
}

// ---------------------------------------------------------------------------
// (5) UNPAIRED REFUSAL: an unpaired desk key is refused with its real reason, exit non-zero.
// ---------------------------------------------------------------------------

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn unpaired_desk_is_refused_honestly() {
    let manager_b = test_manager("unpairedB");
    // The robot's access list does NOT bind the desk key → wire refused.
    let daemon = disabled_endpoint([203; 32]).await;
    let authz = Arc::new(claimed_unpaired());
    // A wire plane IS present (proves the gate holds even WITH the plane wired).
    let plane = Arc::new(WirePlane::with_manager(
        "robot-unp",
        test_manager("unpairedRobot"),
    ));
    let config = desk_config(&daemon, DemandSet::CatalogOnly, None);
    let robot = spawn_robot(daemon, authz, plane);

    let result = bounded(
        "desk refused",
        run_connect(config, manager_b, ignore_catalog, async {
            std::future::pending::<()>().await;
        }),
    )
    .await;

    match result {
        Err(ConnectError::Refused(reason)) => {
            assert!(
                reason.contains("unpaired") || reason.contains("refused"),
                "the refusal reason is surfaced verbatim: {reason}"
            );
        }
        other => panic!("expected ConnectError::Refused, got {other:?}"),
    }
    let _ = bounded("robot join", robot).await;
}

// ---------------------------------------------------------------------------
// (6) MULTI-TOPIC INTERLEAVED: two demanded topics, frames interleaved across
//     their two uni streams → both re-inject BYTE-IDENTICAL, gap-free per topic,
//     with NO cross-wiring (distinct payload tags). The core connect-a-real-robot
//     path (also exercises the f2 membership check across multiple streams).
// ---------------------------------------------------------------------------

/// Assert the collected frames byte-match the TAGGED hand oracle (by their own
/// sequence) + are gap-free from first-received. Cross-wiring (a frame from the
/// OTHER topic) fails the byte compare (wrong tag).
fn assert_tagged_oracle_and_gap_free(collected: &[(u32, Vec<u8>)], tag: u8) {
    let first = collected[0].0;
    for (i, (seq, frame)) in collected.iter().enumerate() {
        assert_eq!(
            *seq,
            first + i as u32,
            "topic {tag:#x}: gap-free from first-received"
        );
        assert_eq!(
            frame,
            &oracle_frame_tagged(tag, *seq),
            "topic {tag:#x} frame {seq} must be BYTE-IDENTICAL to its tagged oracle (no cross-wiring)"
        );
    }
}

async fn run_multi_topic(tag: &str, n: usize) -> (Vec<(u32, Vec<u8>)>, Vec<(u32, Vec<u8>)>) {
    let manager_a = test_manager(&format!("{tag}A"));
    let manager_b = test_manager(&format!("{tag}B"));
    let topic_a = format!("/reinj/{tag}/A/{}", unique_id());
    let topic_b = format!("/reinj/{tag}/B/{}", unique_id());

    let stop = Arc::new(AtomicBool::new(false));
    let prod_a =
        spawn_producer_tagged(manager_a.clone(), topic_a.clone(), Some(0xAA), stop.clone());
    let prod_b =
        spawn_producer_tagged(manager_a.clone(), topic_b.clone(), Some(0xBB), stop.clone());

    let daemon = disabled_endpoint([211; 32]).await;
    let authz = Arc::new(claimed_paired(desk_public_key()));
    let plane = Arc::new(WirePlane::with_manager("robot-mt", manager_a.clone()));
    let config = desk_config(
        &daemon,
        DemandSet::Named(vec![topic_a.clone(), topic_b.clone()]),
        None,
    );
    let robot = spawn_robot(daemon, authz, plane);

    let (shutdown_tx, shutdown_rx) = tokio::sync::oneshot::channel::<()>();
    let desk = tokio::spawn(run_connect(
        config,
        manager_b.clone(),
        ignore_catalog,
        async move {
            let _ = shutdown_rx.await;
        },
    ));

    // Collect BOTH topics off desk-local SHM concurrently (bounded).
    let (mgr_a, ta) = (manager_b.clone(), topic_a.clone());
    let (mgr_b, tb) = (manager_b.clone(), topic_b.clone());
    let got_a = bounded(
        "collect A",
        tokio::task::spawn_blocking(move || collect_frames(&mgr_a, &ta, n)),
    )
    .await
    .expect("collect A join");
    let got_b = bounded(
        "collect B",
        tokio::task::spawn_blocking(move || collect_frames(&mgr_b, &tb, n)),
    )
    .await
    .expect("collect B join");

    let _ = shutdown_tx.send(());
    let summary = bounded("desk join", desk)
        .await
        .expect("desk join")
        .expect("run_connect ok");
    assert_eq!(
        summary.per_topic.len(),
        2,
        "both topics streamed: {:?}",
        summary.per_topic
    );
    stop.store(true, Ordering::Relaxed);
    let _ = prod_a.join();
    let _ = prod_b.join();
    let _ = bounded("robot join", robot).await;
    (got_a, got_b)
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn multi_topic_interleaved_reinject_byte_identical_per_topic() {
    let (a, b) = run_multi_topic("mt", 5).await;
    assert_tagged_oracle_and_gap_free(&a, 0xAA);
    assert_tagged_oracle_and_gap_free(&b, 0xBB);
}

// ---------------------------------------------------------------------------
// (7) f2 — a robot preamble naming an UNDEMANDED topic is REFUSED: no desk-local
//     publisher is created for it (a hand-rolled MISBEHAVING robot).
// ---------------------------------------------------------------------------

/// A single-topic catalog (the misbehaving robot lists the topic it will ACK).
fn single_topic_catalog(robot: &str, topic: &str) -> CatalogReply {
    CatalogReply {
        version: CATALOG_WIRE_VERSION,
        robot: robot.to_string(),
        entries: vec![CatalogEntry {
            topic: topic.to_string(),
            schema_hash: Some(ORACLE_SCHEMA_HASH),
            schema_name: None,
            provenance: CatalogProvenance::Runtime,
            producer_count: None,
            liveness: None,
        }],
        error: None,
    }
}

/// A hand-rolled MISBEHAVING robot: admits the wire connection + lists/ACKs
/// `list_topic`, but opens the uni stream with a preamble naming `spoof_topic` (a
/// DIFFERENT, undemanded topic) + writes a frame — the f2 attack. The desk must
/// REFUSE the spoofed stream (never create a `spoof_topic` publisher).
async fn spoofing_robot(daemon: Endpoint, list_topic: String, spoof_topic: String) {
    let Ok(Some(accepted)) = accept_one(&daemon).await else {
        return;
    };
    let conn = accepted.connection;
    let Ok((mut send, mut recv)) = accept_frame_stream(&conn).await else {
        return;
    };
    // Keep the spoofed uni streams alive for the connection's lifetime.
    let mut open_streams = Vec::new();
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
            WireRequest::Catalog => {
                WireResponse::Catalog(single_topic_catalog("robot-spoof", &list_topic))
            }
            WireRequest::Demand { topic } => {
                if let Ok(mut ustream) = open_uni_frame_stream(&conn).await {
                    let preamble = serde_json::to_vec(&StreamPreamble {
                        topic: spoof_topic.clone(),
                    })
                    .unwrap();
                    let _ = write_frame(&mut ustream, &preamble).await;
                    let _ = write_frame(&mut ustream, &oracle_frame(0)).await;
                    open_streams.push(ustream);
                }
                WireResponse::DemandAccepted { topic }
            }
            WireRequest::Schema { topic } => {
                WireResponse::Schema(SchemaReply::not_found("robot-spoof", &topic, "no schema"))
            }
            WireRequest::Undemand { topic } => WireResponse::Undemanded {
                topic,
                was_demanded: false,
            },
            WireRequest::Status => WireResponse::Error {
                topic: None,
                message: "n/a".to_string(),
            },
            // This stub has no trust store, so it answers the epoch push the
            // way a sink-less robot does — a loud Error. The demand assertions that
            // follow on the SAME stream double as the connection-stays-usable pin.
            WireRequest::SyncEpoch { .. } => WireResponse::Error {
                topic: None,
                message: "this stub robot accepts no epoch sync".to_string(),
            },
        };
        if write_frame(&mut send, &serde_json::to_vec(&resp).unwrap())
            .await
            .is_err()
        {
            break;
        }
    }
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn undemanded_preamble_topic_is_refused_no_desk_publisher() {
    let manager_b = test_manager("spoofB");
    let demanded = format!("/reinj/spoof/imu/{}", unique_id());
    let spoof = format!("/reinj/spoof/evil/{}", unique_id());

    let daemon = disabled_endpoint([212; 32]).await;
    let config = desk_config(&daemon, DemandSet::Named(vec![demanded.clone()]), None);
    let robot = tokio::spawn(spoofing_robot(daemon, demanded.clone(), spoof.clone()));

    let (shutdown_tx, shutdown_rx) = tokio::sync::oneshot::channel::<()>();
    let desk = tokio::spawn(run_connect(
        config,
        manager_b.clone(),
        ignore_catalog,
        async move {
            let _ = shutdown_rx.await;
        },
    ));

    // Poll: the SPOOFED topic must NEVER get a desk-local service. Reverting
    // the membership check makes the desk create a `spoof` publisher within a
    // moment → this probe succeeds → the test fails (mutation-meaningful).
    let mgr = manager_b.clone();
    let spoof_probe = spoof.clone();
    let created = bounded(
        "spoof probe",
        tokio::task::spawn_blocking(move || {
            let deadline = Instant::now() + Duration::from_secs(3);
            while Instant::now() < deadline {
                if mgr.create_data_only_subscriber(&spoof_probe).is_ok() {
                    return true; // the desk created a publisher for the undemanded topic
                }
                std::thread::sleep(Duration::from_millis(50));
            }
            false
        }),
    )
    .await
    .expect("probe join");

    assert!(
        !created,
        "f2: the desk must NOT create a desk-local publisher for an UNDEMANDED (spoofed) topic"
    );

    let _ = shutdown_tx.send(());
    let _ = bounded("desk join", desk).await;
    robot.abort();
}

// ---------------------------------------------------------------------------
// (8) f1 — an admit-then-stall robot must make the desk ERROR within the bound,
//     not hang (a hand-rolled robot that accepts the control stream then withholds
//     every reply). Uses a SHORT robot_timeout.
// ---------------------------------------------------------------------------

/// A hand-rolled STALLING robot: admits the connection + accepts the control
/// stream, reads the desk's first control request, then NEVER replies (holding the
/// connection alive so the desk's read TIMES OUT rather than getting a stream
/// error).
async fn stalling_robot(daemon: Endpoint) {
    let Ok(Some(accepted)) = accept_one(&daemon).await else {
        return;
    };
    let conn = accepted.connection;
    let Ok((_send, mut recv)) = accept_frame_stream(&conn).await else {
        return;
    };
    // Read the desk's first request but send NO reply.
    let _ = read_frame(&mut recv, DEFAULT_MAX_FRAME_LEN).await;
    // Hold the connection alive well past the desk's (short) timeout.
    tokio::time::sleep(Duration::from_secs(8)).await;
    drop(conn);
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn admit_then_stall_robot_errors_within_bound() {
    let manager_b = test_manager("stallB");
    let daemon = disabled_endpoint([213; 32]).await;
    let mut config = desk_config(&daemon, DemandSet::Named(vec!["/imu".to_string()]), None);
    // A SHORT robot-response bound so the stall is caught fast.
    config.robot_timeout = Duration::from_millis(500);
    let robot = tokio::spawn(stalling_robot(daemon));

    // run_connect MUST return (an Err) within a tight bound — an admit-then-stall
    // robot must not hang. An unbounded read hangs → the 5s timeout fires
    // → this `.expect` panics.
    let result = tokio::time::timeout(
        Duration::from_secs(5),
        run_connect(config, manager_b, ignore_catalog, async {
            std::future::pending::<()>().await;
        }),
    )
    .await
    .expect("run_connect must return within 5s — an admit-then-stall robot must not hang");

    match result {
        Err(ConnectError::Control(msg)) => {
            assert!(
                msg.contains("did not answer"),
                "expected a timeout message: {msg}"
            );
        }
        other => panic!("expected a Control-timeout error, got {other:?}"),
    }
    robot.abort();
}

/// A hand-rolled robot that ANSWERS the catalog (so the desk proceeds to the
/// epoch push) and then STOPS READING its control stream entirely — the peer shape that
/// stalls a desk's WRITE rather than its read.
async fn catalog_then_stop_reading_robot(daemon: Endpoint, robot: &str) {
    let Ok(Some(accepted)) = accept_one(&daemon).await else {
        return;
    };
    let conn = accepted.connection;
    let Ok((mut send, mut recv)) = accept_frame_stream(&conn).await else {
        return;
    };
    // Answer exactly ONE request (the catalog), then never read again.
    let _ = read_frame(&mut recv, DEFAULT_MAX_FRAME_LEN).await;
    let reply = WireResponse::Catalog(CatalogReply {
        version: CATALOG_WIRE_VERSION,
        robot: robot.to_string(),
        entries: vec![],
        error: None,
    });
    let _ = write_frame(&mut send, &serde_json::to_vec(&reply).unwrap()).await;
    // Hold the connection open (so the desk sees flow-control back-pressure, not a
    // stream error) well past the desk's short bound.
    tokio::time::sleep(Duration::from_secs(20)).await;
    drop(conn);
}

/// The WRITE half is bounded too.
///
/// netd bounds both halves of its control round-trip; bounding only the READ here
/// would leave the WRITE free to block forever. That matters precisely because of the
/// epoch push: it is the one control frame that can approach the 16 MiB cap, and QUIC
/// stream flow control stalls a large write as soon as the peer stops reading. A desk
/// dialing a robot that admits the connection and then goes quiet would hang the verb
/// indefinitely. The requirement "the push must NEVER block or delay the connection"
/// is broken by the first control write that can stall.
///
/// The robot here answers the catalog (so the session reaches the push) and then stops
/// reading; the desk holds a MULTI-MEGABYTE cached epoch, far past any QUIC stream
/// window. An unbounded write never returns; `run_connect` must error inside the bound.
#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn a_stalled_peer_cannot_hang_the_verb_on_the_epoch_write() {
    let manager_b = test_manager("writestallB");
    // A cached epoch large enough to exhaust any QUIC stream/connection window, yet
    // under the frame cap so it is genuinely SENT (not refused as CacheTooLarge).
    let dir = epochs_dir_with("epoch-writestall", &big_epoch_cache_artifact(9, 120_000));
    let daemon = disabled_endpoint([214; 32]).await;
    let mut config = desk_config_with_epochs(
        &daemon,
        DemandSet::CatalogOnly,
        None,
        Some(dir.path().to_path_buf()),
        Some("epoch-writestall"),
    );
    // A SHORT bound so the stall is caught fast.
    config.robot_timeout = Duration::from_millis(500);
    let robot = tokio::spawn(catalog_then_stop_reading_robot(daemon, "epoch-writestall"));

    // MUST return within a tight bound. With an unbounded write this hangs and the
    // outer timeout fires, panicking here.
    let result = tokio::time::timeout(
        Duration::from_secs(10),
        run_connect(config, manager_b, ignore_catalog, async {
            std::future::pending::<()>().await;
        }),
    )
    .await
    .expect(
        "run_connect must return within 10s — a peer that stops READING must not hang the \
         verb on the epoch push's write (kernel D)",
    );

    match result {
        Err(ConnectError::Control(msg)) => assert!(
            msg.contains("did not read a control request"),
            "the failure must name the WRITE stall (not the read): {msg}"
        ),
        other => panic!("expected a Control write-timeout error, got {other:?}"),
    }
    robot.abort();
}

// ===========================================================================
// THE EPOCH PUSH — `cerulion connect` carries the revocation epoch on
// EVERY dial, UNCONDITIONALLY (by design).
//
// Revocation propagation is not something a user opts into: there is no flag, no
// env gate and no config knob, because any of those would let someone dial a robot
// while silently withholding a revocation they are holding. These arms drive the
// REAL `run_connect` session against the REAL robot wire plane (a GENUINE signed
// intermediate + epoch, re-verified by the robot's own trust store — a hand-faked
// blob would be refused and would prove nothing) and pin:
//
//   (a) the epoch LANDS robot-side — the robot's live access state flips;
//   (b) no cached epoch is a silent no-op that still connects;
//   (c) a push FAILURE never blocks the session, and is classified by its real cause.
// ===========================================================================

/// The root signing key the epoch fixture chains to (its public half is the root set
/// in `claimed_store_for_epochs`).
fn epoch_root_sk() -> ed25519_dalek::SigningKey {
    ed25519_dalek::SigningKey::from_bytes(&[1u8; 32])
}

fn epoch_inter_sk() -> ed25519_dalek::SigningKey {
    ed25519_dalek::SigningKey::from_bytes(&[10u8; 32])
}

fn epoch_pk(k: &ed25519_dalek::SigningKey) -> PublicKey {
    PublicKey(k.verifying_key().to_bytes())
}

/// The PROBE device key the epoch revokes — a third-party desk, so revoking it never
/// disturbs the session under test. Unbound ⇒ `Unpaired` before, `DeviceRevoked`
/// after (`is_device_revoked` is checked before any account row).
const PROBE_DEVICE_KEY: [u8; 32] = [0xB4; 32];

/// A CLAIMED store whose root set is the REAL `epoch_root_sk` public half, so a
/// genuinely signed epoch verifies against it (the file's other `claimed_store` uses
/// a placeholder root and would refuse every real epoch).
fn claimed_store_for_epochs() -> TrustStore {
    let root_set = RootSet::new(vec![epoch_pk(&epoch_root_sk())], 1).unwrap();
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

/// The `base64url(postcard(EpochSyncWire))` artifact a desk caches: a REAL
/// root-signed intermediate + a REAL intermediate-signed epoch `n` for robot
/// `RobotId([5;32])` revoking [`PROBE_DEVICE_KEY`].
fn epoch_cache_artifact(n: u64) -> String {
    use cerulion_pairing::format::{
        AccessListEpoch, IntermediateCert, Scope, Validity, FORMAT_VERSION,
    };
    let inter = epoch_inter_sk();
    let inter_pk = epoch_pk(&inter);
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
        .sign_by_roots(&[&epoch_root_sk()]),
        AccessListEpoch {
            version: FORMAT_VERSION,
            robot: RobotId([5; 32]),
            epoch: n,
            revoked_accounts: vec![],
            revoked_devices: vec![PublicKey(PROBE_DEVICE_KEY)],
            issued_at_ns: 6,
            issuer_key: inter_pk,
        }
        .sign(&inter),
    );
    cerulion_wireclient::config::encode_epoch_cache(&wire).expect("encode the artifact")
}

/// The SAME artifact stamped with a FUTURE envelope version — what a desk running a
/// newer build would hand a robot that predates the shape. The robot's real
/// `EpochSyncWire::from_postcard` refuses it BY NAME (kernel C).
fn future_version_epoch_cache_artifact(n: u64) -> String {
    let b64 = epoch_cache_artifact(n);
    let mut wire = cerulion_wireclient::config::decode_epoch_cache(&b64).expect("decode");
    wire.version = cerulion_pairing::verify::EPOCH_SYNC_WIRE_VERSION + 1;
    cerulion_wireclient::config::encode_epoch_cache(&wire).expect("re-encode")
}

/// The same artifact PADDED with `extra_revoked` throwaway revoked accounts, so its
/// control frame is multiple megabytes — big enough that a peer which stops reading
/// stalls the desk's WRITE on QUIC stream flow control (the kernel-D pin), yet
/// still under the 16 MiB frame cap so it is genuinely sent rather than refused.
fn big_epoch_cache_artifact(n: u64, extra_revoked: usize) -> String {
    use cerulion_pairing::format::{
        AccessListEpoch, AccountId, IntermediateCert, Scope, Validity, FORMAT_VERSION,
    };
    let inter = epoch_inter_sk();
    let inter_pk = epoch_pk(&inter);
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
        .sign_by_roots(&[&epoch_root_sk()]),
        AccessListEpoch {
            version: FORMAT_VERSION,
            robot: RobotId([5; 32]),
            epoch: n,
            revoked_accounts: (0..extra_revoked)
                .map(|i| AccountId([(i % 251) as u8; 32]))
                .collect(),
            revoked_devices: vec![PublicKey(PROBE_DEVICE_KEY)],
            issued_at_ns: 6,
            issuer_key: inter_pk,
        }
        .sign(&inter),
    );
    cerulion_wireclient::config::encode_epoch_cache(&wire).expect("encode the artifact")
}

/// Write `artifact` into a fresh epochs dir under `<name>.epoch` — the shared
/// convention every reader resolves.
fn epochs_dir_with(name: &str, artifact: &str) -> tempfile::TempDir {
    let dir = tempfile::tempdir().expect("tempdir");
    std::fs::write(
        dir.path()
            .join(cerulion_wireclient::config::epoch_cache_file_name(name)),
        artifact,
    )
    .expect("write the epoch cache");
    dir
}

/// A per-test-unique robot endpoint seed (parallel-safe iroh identities), kept away
/// from the file's fixed seeds.
fn unique_secret_epoch() -> [u8; 32] {
    static N: AtomicU64 = AtomicU64::new(0);
    let mut seed = [0xC8u8; 32];
    seed[..8].copy_from_slice(&N.fetch_add(1, Ordering::Relaxed).to_le_bytes());
    seed[8..16].copy_from_slice(&(std::process::id() as u64).to_le_bytes());
    seed
}

/// Drive ONE catalog-only `cerulion connect` session against a robot whose wire plane
/// has (or has not) an epoch sink, returning the session summary + the robot's LIVE
/// trust handle so the caller can observe whether the epoch actually landed.
async fn run_epoch_session(
    tag: &str,
    robot_name: &str,
    epoch_dir: Option<std::path::PathBuf>,
    epoch_sink: bool,
) -> (cerulion_connectd::ConnectSummary, SharedTrust) {
    // The ordinary case: the desk knows this robot by the same name the robot reports.
    run_epoch_session_named(tag, robot_name, Some(robot_name), epoch_dir, epoch_sink).await
}

/// The same session with the DESK's name for the robot given SEPARATELY from the name
/// the robot reports — the kernel-B split (the cache is keyed by the desk's
/// name; `None` means this desk has no verified name and must push nothing).
async fn run_epoch_session_named(
    tag: &str,
    robot_name: &str,
    desk_robot_name: Option<&str>,
    epoch_dir: Option<std::path::PathBuf>,
    epoch_sink: bool,
) -> (cerulion_connectd::ConnectSummary, SharedTrust) {
    let manager_a = test_manager(&format!("{tag}A"));
    let manager_b = test_manager(&format!("{tag}B"));

    let mut index = DeviceAccountIndex::new();
    index.bind(PublicKey(desk_public_key()), OWNER);
    // The ONE handle is the accept gate, the demand authorizer AND the epoch sink —
    // the production arrangement. An empty MAC key keeps it in-memory (no disk).
    let shared = SharedTrust::new(claimed_store_for_epochs(), index, Vec::new());
    let authz = Arc::new(PairingAuthorizer::from_shared(shared.clone()));

    let mut plane = WirePlane::with_manager(robot_name, manager_a.clone());
    if epoch_sink {
        plane = plane.with_epoch_sink(shared.clone(), RemotedClock::fixed(T_NOW));
    }
    let plane = Arc::new(plane);

    let daemon = disabled_endpoint(unique_secret_epoch()).await;
    let config = desk_config_with_epochs(
        &daemon,
        DemandSet::CatalogOnly,
        None,
        epoch_dir,
        desk_robot_name,
    );
    let robot = spawn_robot(daemon, authz, plane);

    let summary = bounded(
        "connect",
        run_connect(config, manager_b, ignore_catalog, async {}),
    )
    .await
    .expect("run_connect must succeed — the epoch push never blocks the connection");
    let _ = bounded("robot join", robot).await;
    (summary, shared)
}

/// The probe device's live access state on the robot — the observable that says
/// whether a pushed epoch actually took effect.
fn probe_access(shared: &SharedTrust) -> cerulion_remoted::KeyAccess {
    shared.snapshot_for_key(&PROBE_DEVICE_KEY).1
}

/// (a) HEADLINE: a `cerulion connect` dial DELIVERS the cached epoch and the robot
/// APPLIES it — proven on the robot's own live trust store against a hand oracle
/// (the probe device reads `Unpaired` before and `DeviceRevoked` after), with the
/// session reporting `Applied { epoch: 9 }`.
#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn connect_pushes_the_cached_epoch_and_the_robot_applies_it() {
    let dir = epochs_dir_with("epoch-robot", &epoch_cache_artifact(9));

    let (summary, shared) = run_epoch_session(
        "epochpush",
        "epoch-robot",
        Some(dir.path().to_path_buf()),
        true,
    )
    .await;

    assert_eq!(
        summary.epoch_push,
        cerulion_connectd::EpochPushOutcome::Applied { epoch: 9 },
        "the connect verb must DELIVER the cached epoch on every dial (no flag, no opt-in)"
    );
    assert_eq!(
        probe_access(&shared),
        cerulion_remoted::KeyAccess::DeviceRevoked,
        "the pushed epoch must be IN FORCE on the robot, not merely acknowledged"
    );
}

/// ANTI-TAUTOLOGY control for (a): the SAME robot + the SAME probe, with NO desk
/// cache — the probe stays `Unpaired`, so the flip above is the push's doing and not
/// a fixture that starts out revoked.
#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn connect_with_no_cached_epoch_is_a_silent_no_op() {
    // An epochs dir that exists but holds nothing for this robot.
    let empty = tempfile::tempdir().expect("tempdir");
    let (summary, shared) = run_epoch_session(
        "epochnone",
        "epoch-empty",
        Some(empty.path().to_path_buf()),
        true,
    )
    .await;
    assert_eq!(
        summary.epoch_push,
        cerulion_connectd::EpochPushOutcome::NoCachedEpoch
    );
    assert_eq!(
        probe_access(&shared),
        cerulion_remoted::KeyAccess::Unpaired,
        "nothing cached ⇒ nothing pushed ⇒ the robot's access list is untouched"
    );

    // And with NO epochs directory at all (an ephemeral desk key) — a distinct, less
    // alarming state that must also connect normally.
    let (summary, shared) = run_epoch_session("epochnodir", "epoch-nodir", None, true).await;
    assert_eq!(
        summary.epoch_push,
        cerulion_connectd::EpochPushOutcome::NoCacheDir
    );
    assert_eq!(probe_access(&shared), cerulion_remoted::KeyAccess::Unpaired);
}

/// (c) A push FAILURE never blocks the session: the robot has NO epoch sink (an
/// older / unconfigured build), so it refuses the push — the connect session still
/// completes, and the refusal is reported accurately as `NoSink` ("upgrade the robot"),
/// never as a `Rejected` epoch ("investigate the epoch").
#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn a_refused_push_never_blocks_the_connect_session() {
    let dir = epochs_dir_with("epoch-sinkless", &epoch_cache_artifact(9));

    let (summary, shared) = run_epoch_session(
        "epochrefuse",
        "epoch-sinkless",
        Some(dir.path().to_path_buf()),
        // NO epoch sink installed ⇒ the robot answers a loud refusal.
        false,
    )
    .await;

    // Reaching here at all is the never-block pin: `run_connect` returned Ok.
    assert_eq!(
        summary.epoch_push,
        cerulion_connectd::EpochPushOutcome::NoSink,
        "a sink-less robot is a NoSink (upgrade it), never a Rejected epoch"
    );
    assert_eq!(
        summary.catalog.robot, "epoch-sinkless",
        "the catalog still came back — the session was not blocked by the refusal"
    );
    assert_eq!(
        probe_access(&shared),
        cerulion_remoted::KeyAccess::Unpaired,
        "a refused push must change NOTHING on the robot"
    );
}

/// A cache filed under a DIFFERENT name than the DESK knows the robot by is not found
/// — the silent-non-delivery class the INFO log (naming the exact path searched)
/// exists for. The session still connects.
#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn a_cache_filed_under_another_name_is_not_pushed_and_still_connects() {
    // Filed under "some-other-robot" while the desk knows this robot as "epoch-named".
    let dir = epochs_dir_with("some-other-robot", &epoch_cache_artifact(9));

    let (summary, shared) = run_epoch_session(
        "epochname",
        "epoch-named",
        Some(dir.path().to_path_buf()),
        true,
    )
    .await;
    assert_eq!(
        summary.epoch_push,
        cerulion_connectd::EpochPushOutcome::NoCachedEpoch,
        "the desk looks for <the name IT knows the robot by>.epoch — a mis-filed cache \
         reads as nothing cached (and is logged with the exact path it looked for)"
    );
    assert_eq!(probe_access(&shared), cerulion_remoted::KeyAccess::Unpaired);
}

/// THE security pin: the cache lookup is keyed by the
/// name the DESK knows the robot by, NEVER by the identity the robot reports about
/// itself. A robot that renames itself to another robot's cache key must NOT be handed
/// that robot's signed epoch.
///
/// The robot reports `"victim"` (the name of a cache this desk really holds) while the
/// desk dialed it as `"impostor"`. A lookup keyed by the catalog name would hand the
/// artifact over; the desk looks under its own name instead, finds nothing,
/// and the session still connects.
#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn a_robot_cannot_name_itself_into_another_robots_cached_epoch() {
    // The desk holds a real cached epoch for "victim" — and none for "impostor".
    let dir = epochs_dir_with("victim", &epoch_cache_artifact(9));

    let (summary, shared) = run_epoch_session_named(
        "epochimp",
        // What the ROBOT reports about itself: the victim's cache key.
        "victim",
        // What the DESK dialed / knows it as.
        Some("impostor"),
        Some(dir.path().to_path_buf()),
        true,
    )
    .await;
    assert_eq!(
        summary.epoch_push,
        cerulion_connectd::EpochPushOutcome::NoCachedEpoch,
        "the peer's self-reported name must NOT select a cache file — the desk looked \
         for impostor.epoch, which does not exist"
    );
    assert_eq!(
        summary.catalog.robot, "victim",
        "the robot really did claim the victim's name (the attack was attempted)"
    );
    assert_eq!(
        probe_access(&shared),
        cerulion_remoted::KeyAccess::Unpaired,
        "nothing was delivered to the impostor"
    );

    // ANTI-TAUTOLOGY: the SAME cache IS delivered when the DESK's own name for the
    // robot is `victim` — so the refusal above is the keying rule, not a broken fixture.
    let dir = epochs_dir_with("victim", &epoch_cache_artifact(9));
    let (summary, shared) = run_epoch_session_named(
        "epochvic",
        "victim",
        Some("victim"),
        Some(dir.path().to_path_buf()),
        true,
    )
    .await;
    assert_eq!(
        summary.epoch_push,
        cerulion_connectd::EpochPushOutcome::Applied { epoch: 9 }
    );
    assert_eq!(
        probe_access(&shared),
        cerulion_remoted::KeyAccess::DeviceRevoked
    );
}

/// An epoch-sync ENVELOPE VERSION this build cannot
/// read is never mistaken for a delivered epoch, and never blocks the session.
///
/// SCOPE. There are two version-skew directions, and only ONE is reachable from
/// a single-process test:
///
/// - **Writer ahead of the DESK** (this test): the cached artifact carries a future
///   version, so the desk's own reader refuses it — a `CacheUnreadable` skip whose
///   remediation is "UPGRADE this desk", pinned by
///   `cerulion_wireclient::epoch::an_unreadable_cache_says_upgrade_for_a_version_skew_and_resync_for_corruption`.
///   The push is a no-op and the session still connects.
/// - **Desk ahead of the ROBOT**: the desk sends fine and the ROBOT refuses by version.
///   That is a genuinely CROSS-BUILD scenario — desk and robot here share one
///   `EPOCH_SYNC_WIRE_VERSION` const in one process, so it cannot be staged e2e. Its
///   classification (upgrade-shaped `NoSink`, never a `Rejected` epoch) is pinned in
///   `cerulion_wireclient::epoch::classify_maps_every_reply_shape`, against a refusal
///   string produced by the REAL `EpochSyncWire::from_postcard` — the exact text the
///   robot's `apply_pushed_epoch` wraps and returns.
#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn a_future_envelope_version_never_reads_as_a_delivered_epoch() {
    let dir = epochs_dir_with("epoch-vskew", &future_version_epoch_cache_artifact(9));

    let (summary, shared) = run_epoch_session(
        "epochvskew",
        "epoch-vskew",
        Some(dir.path().to_path_buf()),
        // A robot WITH a working epoch sink: nothing about the ROBOT is the problem.
        true,
    )
    .await;

    assert_eq!(
        summary.epoch_push,
        cerulion_connectd::EpochPushOutcome::CacheUnreadable,
        "an artifact this build cannot read is a skip — never a silent success"
    );
    assert_eq!(
        summary.catalog.robot, "epoch-vskew",
        "the session still completed (freshness never blocks the connection)"
    );
    assert_eq!(
        probe_access(&shared),
        cerulion_remoted::KeyAccess::Unpaired,
        "an unreadable envelope changes NOTHING on the robot"
    );

    // ANTI-TAUTOLOGY: the SAME robot, SAME sink, SAME epoch at the CURRENT version IS
    // applied — so the skip above is the version stamp, not a broken fixture.
    let dir = epochs_dir_with("epoch-vok", &epoch_cache_artifact(9));
    let (summary, shared) = run_epoch_session(
        "epochvok",
        "epoch-vok",
        Some(dir.path().to_path_buf()),
        true,
    )
    .await;
    assert_eq!(
        summary.epoch_push,
        cerulion_connectd::EpochPushOutcome::Applied { epoch: 9 }
    );
    assert_eq!(
        probe_access(&shared),
        cerulion_remoted::KeyAccess::DeviceRevoked
    );
}

/// A desk with NO verified name for the robot it dialed
/// (a raw `--eid` dial of an unpinned robot) pushes NOTHING and says so — it never falls
/// back to the peer's word to pick a file. The session still connects normally.
#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn no_desk_verified_name_pushes_nothing_and_still_connects() {
    // A cache the peer's self-reported name WOULD have matched.
    let dir = epochs_dir_with("epoch-unpinned", &epoch_cache_artifact(9));

    let (summary, shared) = run_epoch_session_named(
        "epochunpin",
        "epoch-unpinned",
        None, // the desk has no robots.toml entry for this eid
        Some(dir.path().to_path_buf()),
        true,
    )
    .await;
    assert_eq!(
        summary.epoch_push,
        cerulion_connectd::EpochPushOutcome::UnverifiedRobotIdentity,
        "with no desk-verified name the desk must refuse to guess — never key the \
         lookup off the peer's self-report"
    );
    assert_eq!(
        summary.catalog.robot, "epoch-unpinned",
        "the session still completed (the push is a policy result, never a blocker)"
    );
    assert_eq!(probe_access(&shared), cerulion_remoted::KeyAccess::Unpaired);
}

/// When a desk has NEITHER a cache directory NOR a pinned name, the
/// reported reason is the one that matches its actual situation — `NoCacheDir` ("this desk
/// holds nothing"), NOT `UnverifiedRobotIdentity` (whose remediation is "pair the robot").
///
/// The original ordering checked the identity first, so an ephemeral-key desk with no
/// epochs directory at all was told to go pair a robot — a fix for a problem it does not
/// have, while the real one (no cache to push from) went unnamed. The name only matters
/// once there is a directory to look in, which the sibling test above still covers.
#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn no_cache_dir_wins_over_no_desk_name_in_the_reported_reason() {
    let (summary, shared) = run_epoch_session_named(
        "epochbothx",
        "epoch-both",
        None, // no desk-verified name…
        None, // …AND no epochs directory at all
        true,
    )
    .await;
    assert_eq!(
        summary.epoch_push,
        cerulion_connectd::EpochPushOutcome::NoCacheDir,
        "with no cache directory the correct reason is NoCacheDir — reporting \
         UnverifiedRobotIdentity would prescribe 'pair the robot' to a desk that holds nothing"
    );
    // And it is the QUIET class: this desk is not failing to carry anything.
    assert_eq!(
        summary.epoch_push.severity(),
        cerulion_connectd::EpochPushSeverity::NothingToCarry
    );
    assert_eq!(
        summary.catalog.robot, "epoch-both",
        "the session still completed"
    );
    assert_eq!(probe_access(&shared), cerulion_remoted::KeyAccess::Unpaired);
}
