// SPDX-License-Identifier: AGPL-3.0-only
//! The WIRE PLANE end-to-end over REAL iroh loopback endpoints
//! plus REAL iceoryx2 (per-test SHM roots via `init_for_test`, so parallel-safe:
//! no `#[serial]`, no shared singleton).
//!
//! A paired `CAP_OBSERVE` wire client dials `cerulion/wire/1`, speaks the
//! `cerulion_q` catalog/demand/schema/status vocabulary over the bidi control
//! stream, and reads per-topic uni data streams — the frames flowing VERBATIM
//! from a real SHM producer through `remoted`'s demand-driven tap, compared
//! BYTE-FOR-BYTE against a HAND oracle (never a self-compare — Principle #13).
//!
//! # Producer determinism
//!
//! Frames are hand-built ([`make_wire_frame`]) and published via `publish_raw`,
//! which loans a slot of exactly the frame length and copies every byte VERBATIM
//! — the published bytes ARE the oracle bytes (no VirtualClock / codegen type
//! needed). Sequences/timestamps/schema_hash are hand-chosen, so the desk-side
//! byte compare is exact.
//!
//! # WAN drop-to-live is pinned in-crate, not here
//!
//! A slow-consumer `wan_dropped` e2e over real QUIC is non-deterministic (QUIC's
//! stream flow window absorbs many small frames before `write_frame` blocks), so
//! the drop-to-live SEMANTICS (drop-oldest, exact counts, freshest survives) +
//! the once-per-regime warn EMISSION are pinned deterministically by the
//! `forward.rs` oracle vectors + the `tap.rs` `#[traced_test]` (see those files).

use std::future::Future;
use std::net::{Ipv4Addr, Ipv6Addr, SocketAddr};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use std::collections::BTreeMap;

use cerulion_core::transport::cerulion_q::{
    CatalogReply, SchemaDoc, SchemaEncoding, CATALOG_WIRE_VERSION,
};
use cerulion_core::transport::demand_authorizer::{
    DemandAuthorizer, DemandDecision, DemandSubject,
};
use cerulion_core::wire::{MaxSliceLen, WireHeader};
use cerulion_core::{TransportConfig, TransportManager};
use cerulion_link::{
    accept_one, accept_uni_frame_stream, alpn, build_endpoint, dial, direct_addr,
    open_frame_stream, read_frame, write_frame, Connection, Endpoint, EndpointAddr, EndpointConfig,
    EndpointId, RecvStream, RelayConfig, SendStream, DEFAULT_MAX_FRAME_LEN,
};
use cerulion_pairing::format::{
    AccessGrant, AccountId, DeviceCert, IntermediateCert, PrincipalKind, PublicKey, RobotId, Role,
    RootSet, Scope, Validity, FORMAT_VERSION,
};
use cerulion_pairing::verify::TrustStore;
use cerulion_remoted::pairing_verbs::OwnerGrantWire;
use cerulion_remoted::{
    handle_accepted_with_wire, AcceptDecision, DeviceAccountIndex, KeyAccess, PairingAuthorizer,
    RemotedClock, SharedTrust, StreamPreamble, WirePlane, WireRequest, WireResponse,
};
use ed25519_dalek::SigningKey;

const STEP_TIMEOUT: Duration = Duration::from_secs(20);
const T_NOW: u64 = 1_000_000_000_000;
const CHASSIS: &[u8] = b"wire-plane-chassis-secret-not-serial-derived";
const OWNER: AccountId = AccountId([10; 32]);
/// The hand-chosen schema hash the oracle frames carry (a distinct constant, not
/// any real type's hash — the test asserts the exact value round-trips).
const ORACLE_SCHEMA_HASH: u64 = 0xCE82_2405_F00D_BEEF_u64;

async fn bounded<F: Future>(what: &str, fut: F) -> F::Output {
    match tokio::time::timeout(STEP_TIMEOUT, fut).await {
        Ok(v) => v,
        Err(_) => panic!("wire-plane e2e: '{what}' timed out after {STEP_TIMEOUT:?}"),
    }
}

fn unique_id() -> String {
    use std::sync::atomic::AtomicU64;
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

/// Rewrite unspecified (`0.0.0.0` / `[::]`) bound sockets to loopback so they are
/// dialable.
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

/// A provisioned + CLAIMED trust store (owner = [`OWNER`], OWNER_FULL scope, so
/// it carries `CAP_OBSERVE` — a wire-admittable account).
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

/// Claimed robot; the dialer's key is bound to the OWNER (paired, CAP_OBSERVE).
fn claimed_paired(client_key: [u8; 32]) -> PairingAuthorizer {
    let mut index = DeviceAccountIndex::new();
    index.bind(PublicKey(client_key), OWNER);
    PairingAuthorizer::new(claimed_store(), index)
}

/// Claimed robot; the dialer's key is NOT bound (unpaired → wire refused).
fn claimed_unpaired() -> PairingAuthorizer {
    PairingAuthorizer::new(claimed_store(), DeviceAccountIndex::new())
}

/// A fresh per-test transport manager on its own SHM root (parallel-safe).
fn test_manager(tag: &str) -> Arc<TransportManager> {
    let ix = cerulion_core::testing::iceoryx_test_config();
    TransportManager::init_for_test(
        TransportConfig {
            node_name: format!("wire_{tag}_{}", unique_id()),
            ..Default::default()
        },
        ix,
    )
    .expect("init_for_test")
}

/// Hand-build a raw wire frame (32-byte little-endian header + payload) — the
/// oracle. `publish_raw` publishes these bytes VERBATIM.
fn make_wire_frame(seq: u32, ts: u64, payload: &[u8]) -> Vec<u8> {
    let header = WireHeader {
        schema_hash: ORACLE_SCHEMA_HASH,
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

/// The 3-frame hand oracle: distinct payloads, sequences 0,1,2, ascending ts.
fn oracle_frames() -> Vec<Vec<u8>> {
    vec![
        make_wire_frame(0, 1_000_000, b"wire-frame-zero\x00\x01"),
        make_wire_frame(1, 2_000_000, b"wire-frame-one\xDE\xAD"),
        make_wire_frame(2, 3_000_000, b"wire-frame-two\xBE\xEF\xCA"),
    ]
}

/// Open the bidi control stream on a dialed wire connection.
async fn open_control(conn: &Connection) -> (SendStream, RecvStream) {
    bounded("open control", open_frame_stream(conn))
        .await
        .expect("open_bi control stream")
}

/// Send a control request and read the [`WireResponse`].
async fn request(send: &mut SendStream, recv: &mut RecvStream, req: &WireRequest) -> WireResponse {
    let bytes = serde_json::to_vec(req).expect("encode request");
    bounded("write request", write_frame(send, &bytes))
        .await
        .expect("write request");
    let resp = bounded("read response", read_frame(recv, DEFAULT_MAX_FRAME_LEN))
        .await
        .expect("read response");
    serde_json::from_slice(&resp).expect("decode WireResponse")
}

/// Read the uni data stream's preamble + exactly `n` wire frames (bounded).
async fn read_uni_frames(conn: &Connection, n: usize) -> (String, Vec<Vec<u8>>) {
    let mut recv = bounded("accept_uni", accept_uni_frame_stream(conn))
        .await
        .expect("accept_uni");
    let preamble_bytes = bounded(
        "read preamble",
        read_frame(&mut recv, DEFAULT_MAX_FRAME_LEN),
    )
    .await
    .expect("read preamble");
    let preamble: StreamPreamble =
        serde_json::from_slice(&preamble_bytes).expect("decode preamble");
    let mut frames = Vec::with_capacity(n);
    for _ in 0..n {
        let f = bounded(
            "read data frame",
            read_frame(&mut recv, DEFAULT_MAX_FRAME_LEN),
        )
        .await
        .expect("read data frame");
        frames.push(f);
    }
    (preamble.topic, frames)
}

// ---------------------------------------------------------------------------
// (1) The headline: demand a live topic → BYTE-IDENTICAL frames (hand oracle).
// ---------------------------------------------------------------------------

async fn run_demand_e2e(tag: &str) -> (String, Vec<Vec<u8>>) {
    let manager = test_manager(tag);
    let topic = format!("/wire/{tag}/{}", unique_id());
    // Create the producer FIRST so the topic's data service exists for the tap.
    let mut publisher = manager
        .create_publisher_simple(&topic, MaxSliceLen::const_new(256))
        .expect("producer");

    let daemon = disabled_endpoint([200; 32]).await;
    let client = disabled_endpoint([9; 32]).await;
    let client_key = *client.id().as_bytes();
    let authz = Arc::new(claimed_paired(client_key));
    let plane = Arc::new(WirePlane::with_manager("robot-e2e", manager.clone()));
    let daemon_addr = loopback_addr(daemon.id(), &daemon.bound_sockets());

    let daemon_ref = &daemon;
    let client_ref = &client;
    let topic_ref = &topic;

    let daemon_fut = async move {
        let accepted = bounded("accept_one", accept_one(daemon_ref))
            .await
            .expect("accept ok")
            .expect("incoming");
        handle_accepted_with_wire(accepted, authz, None, Some(plane)).await;
    };

    let client_fut = async move {
        let conn = bounded("dial", dial(client_ref, daemon_addr, alpn::WIRE))
            .await
            .expect("dial wire");
        let (mut send, mut recv) = open_control(&conn).await;

        // Demand → DemandAccepted (the tap is attached by the robot BEFORE it
        // replies, so publishing now is seen).
        let resp = request(
            &mut send,
            &mut recv,
            &WireRequest::Demand {
                topic: topic_ref.clone(),
            },
        )
        .await;
        assert_eq!(
            resp,
            WireResponse::DemandAccepted {
                topic: topic_ref.clone()
            },
            "demand accepted"
        );

        // Publish the 3 oracle frames VERBATIM (publish_raw copies bytes as-is).
        for frame in oracle_frames() {
            publisher.publish_raw(&frame).expect("publish_raw");
        }

        // Read the uni stream: preamble names the topic; frames are byte-identical.
        let (stream_topic, frames) = read_uni_frames(&conn, 3).await;
        assert_eq!(
            &stream_topic, topic_ref,
            "preamble names the demanded topic"
        );

        // Close the control stream so the daemon tears down + returns.
        let _ = send.finish();
        drop(conn);
        (stream_topic, frames)
    };

    let (_, out) = tokio::join!(daemon_fut, client_fut);
    out
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn wire_demand_streams_byte_identical_frames() {
    let (topic, frames) = run_demand_e2e("headline").await;
    assert!(topic.starts_with("/wire/headline/"));
    assert_eq!(
        frames,
        oracle_frames(),
        "frames must arrive VERBATIM, byte-for-byte == the hand oracle"
    );
}

/// Determinism: two full runs each equal the hand oracle (not merely each other).
#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn wire_demand_is_deterministic() {
    let (_, a) = run_demand_e2e("det_a").await;
    let (_, b) = run_demand_e2e("det_b").await;
    let oracle = oracle_frames();
    assert_eq!(a, oracle, "run A == hand oracle");
    assert_eq!(b, oracle, "run B == hand oracle");
}

// ---------------------------------------------------------------------------
// (2) catalog lists the topic + its schema_hash; the reply IS a real
//     cerulion_q::CatalogReply (shape parity pin).
// ---------------------------------------------------------------------------

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn catalog_lists_topic_with_schema_hash_and_parses_as_catalog_reply() {
    let manager = test_manager("catalog");
    let topic = format!("/wire/catalog/{}", unique_id());

    // A CONTINUOUS producer so the catalog PEEK reliably reads a frame (a fresh
    // tap has no history — it sees only frames published after it attaches).
    let mut publisher = manager
        .create_publisher_simple(&topic, MaxSliceLen::const_new(256))
        .expect("producer");
    let stop = Arc::new(AtomicBool::new(false));
    let producer_stop = stop.clone();
    let producer = std::thread::spawn(move || {
        let mut seq = 0u32;
        while !producer_stop.load(Ordering::Relaxed) {
            let frame = make_wire_frame(seq, 1_000_000 + seq as u64 * 10_000, b"catalog-probe");
            let _ = publisher.publish_raw(&frame);
            seq = seq.wrapping_add(1);
            std::thread::sleep(Duration::from_millis(3));
        }
    });

    let daemon = disabled_endpoint([201; 32]).await;
    let client = disabled_endpoint([11; 32]).await;
    let client_key = *client.id().as_bytes();
    let authz = Arc::new(claimed_paired(client_key));
    let plane = Arc::new(WirePlane::with_manager("robot-cat", manager.clone()));
    let daemon_addr = loopback_addr(daemon.id(), &daemon.bound_sockets());

    let daemon_ref = &daemon;
    let client_ref = &client;
    let topic_ref = &topic;

    let daemon_fut = async move {
        let accepted = bounded("accept_one", accept_one(daemon_ref))
            .await
            .expect("accept ok")
            .expect("incoming");
        handle_accepted_with_wire(accepted, authz, None, Some(plane)).await;
    };

    let client_fut = async move {
        let conn = bounded("dial", dial(client_ref, daemon_addr, alpn::WIRE))
            .await
            .expect("dial wire");
        let (mut send, mut recv) = open_control(&conn).await;
        let resp = request(&mut send, &mut recv, &WireRequest::Catalog).await;
        let _ = send.finish();
        drop(conn);
        resp
    };

    let (_, resp) = tokio::join!(daemon_fut, client_fut);
    stop.store(true, Ordering::Relaxed);
    let _ = producer.join();

    let reply: CatalogReply = match resp {
        WireResponse::Catalog(r) => r,
        other => panic!("expected a Catalog reply, got {other:?}"),
    };
    // Shape parity: it IS a real cerulion_q CatalogReply (correct wire version +
    // self-attributed robot).
    assert_eq!(
        reply.version, CATALOG_WIRE_VERSION,
        "real CatalogReply version"
    );
    assert_eq!(reply.robot, "robot-cat", "self-attributed robot identity");
    // The topic is listed with the EXACT hand-chosen schema hash.
    let entry = reply
        .entries
        .iter()
        .find(|e| &e.topic == topic_ref)
        .unwrap_or_else(|| panic!("catalog must list {topic_ref}: {:?}", reply.entries));
    assert_eq!(
        entry.schema_hash,
        Some(ORACLE_SCHEMA_HASH),
        "catalog carries the topic's live wire schema_hash"
    );
    // Re-encoding + decoding through the STANDALONE real struct also succeeds
    // (belt-and-suspenders shape parity).
    let bytes = serde_json::to_vec(&reply).unwrap();
    let round: CatalogReply = serde_json::from_slice(&bytes).unwrap();
    assert_eq!(round, reply);
}

// ---------------------------------------------------------------------------
// (3) undemand drops the tap and RELEASES the SHM subscriber slot.
// ---------------------------------------------------------------------------

/// Release pin: demand+undemand the SAME topic MANY more
/// times than the topic's subscriber ceiling. A leak would exhaust the slots and
/// a later demand would fail with `ExceedsMaxSupportedSubscribers`; every cycle
/// succeeding proves each undemand freed the slot.
///
/// The REAL ceiling here is iceoryx2's DEFAULT `max_subscribers` = 8:
/// `create_publisher_simple` provisions the data service as an `External` writer
/// (`max_subscribers: None` → the iceoryx2 default 8, pinned at 0.9.1 by
/// `iceoryx2_version_lockstep_test`). It is NOT the graph `SingleWriter` path's
/// `INTROSPECTION_SUBSCRIBER_HEADROOM = 5` (four spare tooling slots plus the
/// liveness observer's) — that headroom governs only GRAPH-provisioned
/// topics, not this externally-provisioned service. `CYCLES = 12`
/// exceeds the real ceiling (8), so a leak would fail by the 9th concurrent tap.
#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn undemand_releases_the_slot() {
    let manager = test_manager("undemand");
    let topic = format!("/wire/undemand/{}", unique_id());
    let _publisher = manager
        .create_publisher_simple(&topic, MaxSliceLen::const_new(256))
        .expect("producer");

    let daemon = disabled_endpoint([202; 32]).await;
    let client = disabled_endpoint([12; 32]).await;
    let client_key = *client.id().as_bytes();
    let authz = Arc::new(claimed_paired(client_key));
    let plane = Arc::new(WirePlane::with_manager("robot-und", manager.clone()));
    let daemon_addr = loopback_addr(daemon.id(), &daemon.bound_sockets());

    let daemon_ref = &daemon;
    let client_ref = &client;
    let topic_ref = &topic;

    // Well above the real ceiling (iceoryx2 default max_subscribers = 8)
    // — a leak would exhaust the slots (fail by the 9th tap) long before the
    // last cycle.
    const CYCLES: usize = 12;

    let daemon_fut = async move {
        let accepted = bounded("accept_one", accept_one(daemon_ref))
            .await
            .expect("accept ok")
            .expect("incoming");
        handle_accepted_with_wire(accepted, authz, None, Some(plane)).await;
    };

    let client_fut = async move {
        let conn = bounded("dial", dial(client_ref, daemon_addr, alpn::WIRE))
            .await
            .expect("dial wire");
        let (mut send, mut recv) = open_control(&conn).await;

        for cycle in 0..CYCLES {
            // Demand → each attaches a fresh tap (a leak would have already
            // exhausted the slots by now).
            let d = request(
                &mut send,
                &mut recv,
                &WireRequest::Demand {
                    topic: topic_ref.clone(),
                },
            )
            .await;
            assert_eq!(
                d,
                WireResponse::DemandAccepted {
                    topic: topic_ref.clone()
                },
                "cycle {cycle}: demand must succeed (undemand released the prior slot)"
            );
            // Accept + drop the uni stream this demand opened (so the next demand
            // does not queue an unbounded pile of streams).
            let _recv = bounded("accept_uni", accept_uni_frame_stream(&conn))
                .await
                .expect("uni stream opened");

            // Undemand → releases the slot (awaits the drain-thread join).
            let u = request(
                &mut send,
                &mut recv,
                &WireRequest::Undemand {
                    topic: topic_ref.clone(),
                },
            )
            .await;
            assert_eq!(
                u,
                WireResponse::Undemanded {
                    topic: topic_ref.clone(),
                    was_demanded: true,
                },
                "cycle {cycle}: undemand tears down the live tap"
            );
        }

        // After the last undemand, a fresh direct tap on the manager attaches —
        // the slot is genuinely free.
        manager
            .create_data_only_subscriber(topic_ref)
            .expect("a fresh tap attaches: undemand released the slot");
        // Undemanding a NOT-demanded topic reports was_demanded == false.
        let u = request(
            &mut send,
            &mut recv,
            &WireRequest::Undemand {
                topic: topic_ref.clone(),
            },
        )
        .await;
        assert_eq!(
            u,
            WireResponse::Undemanded {
                topic: topic_ref.clone(),
                was_demanded: false,
            },
            "undemanding a non-demanded topic is a no-op (was_demanded == false)"
        );

        let _ = send.finish();
        drop(conn);
    };

    tokio::join!(daemon_fut, client_fut);
}

// ---------------------------------------------------------------------------
// (4) The pairing gate: an UNPAIRED peer's wire connection is REFUSED; catalog
//     / demand are never reachable.
// ---------------------------------------------------------------------------

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn unpaired_peer_wire_refused_catalog_unreachable() {
    // The wire plane is fully wired (Some(plane)), so this proves the gate holds
    // even WITH the plane present — an unpaired key never reaches the tap plane.
    let manager = test_manager("unpaired");
    let daemon = disabled_endpoint([203; 32]).await;
    let client = disabled_endpoint([13; 32]).await;
    let authz = Arc::new(claimed_unpaired());
    let plane = Arc::new(WirePlane::with_manager("robot-unp", manager));
    let daemon_addr = loopback_addr(daemon.id(), &daemon.bound_sockets());

    let daemon_ref = &daemon;
    let client_ref = &client;

    let daemon_fut = async move {
        let accepted = bounded("accept_one", accept_one(daemon_ref))
            .await
            .expect("accept ok")
            .expect("incoming");
        handle_accepted_with_wire(accepted, authz, None, Some(plane)).await;
    };

    let client_fut = async move {
        let conn = bounded("dial", dial(client_ref, daemon_addr, alpn::WIRE))
            .await
            .expect("dial wire");
        let (mut send, mut recv) = open_control(&conn).await;
        // Try to catalog: the daemon's skeleton-report path answers with an
        // AcceptDecision (Refuse) instead — the wire vocabulary is NEVER served.
        let cat = serde_json::to_vec(&WireRequest::Catalog).unwrap();
        bounded("write catalog", write_frame(&mut send, &cat))
            .await
            .expect("write catalog");
        let bytes = bounded("read reply", read_frame(&mut recv, DEFAULT_MAX_FRAME_LEN))
            .await
            .expect("read reply");
        let _ = send.finish();
        drop(conn);
        bytes
    };

    let (_, bytes) = tokio::join!(daemon_fut, client_fut);
    // It is NOT a WireResponse (the plane was never served) — it is a Refuse
    // AcceptDecision naming the unpaired reason.
    assert!(
        serde_json::from_slice::<WireResponse>(&bytes).is_err(),
        "an unpaired peer must NOT receive a wire response"
    );
    let decision: AcceptDecision =
        serde_json::from_slice(&bytes).expect("unpaired peer gets an AcceptDecision");
    match decision {
        AcceptDecision::Refuse { reason } => {
            assert!(
                reason.contains("unpaired"),
                "refusal names the cause: {reason}"
            );
        }
        other => panic!("expected Refuse, got {other:?}"),
    }
}

// ---------------------------------------------------------------------------
// (5) A demand for a topic that never existed → an ACTIONABLE error reply that
//     names the topic (never silent).
// ---------------------------------------------------------------------------

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn demand_nonexistent_topic_returns_actionable_error() {
    let manager = test_manager("nonexistent");
    // NO producer: the topic's SHM data service does not exist.
    let missing = format!("/wire/nonexistent/{}", unique_id());

    let daemon = disabled_endpoint([204; 32]).await;
    let client = disabled_endpoint([14; 32]).await;
    let client_key = *client.id().as_bytes();
    let authz = Arc::new(claimed_paired(client_key));
    let plane = Arc::new(WirePlane::with_manager("robot-miss", manager));
    let daemon_addr = loopback_addr(daemon.id(), &daemon.bound_sockets());

    let daemon_ref = &daemon;
    let client_ref = &client;
    let missing_ref = &missing;

    let daemon_fut = async move {
        let accepted = bounded("accept_one", accept_one(daemon_ref))
            .await
            .expect("accept ok")
            .expect("incoming");
        handle_accepted_with_wire(accepted, authz, None, Some(plane)).await;
    };

    let client_fut = async move {
        let conn = bounded("dial", dial(client_ref, daemon_addr, alpn::WIRE))
            .await
            .expect("dial wire");
        let (mut send, mut recv) = open_control(&conn).await;
        let resp = request(
            &mut send,
            &mut recv,
            &WireRequest::Demand {
                topic: missing_ref.clone(),
            },
        )
        .await;
        let _ = send.finish();
        drop(conn);
        resp
    };

    let (_, resp) = tokio::join!(daemon_fut, client_fut);
    match resp {
        WireResponse::Error { topic, message } => {
            assert_eq!(
                topic.as_deref(),
                Some(missing.as_str()),
                "error names the topic"
            );
            assert!(
                message.contains(&missing),
                "message names the topic: {message}"
            );
            assert!(
                message.contains("does not exist"),
                "message is actionable (topic does not exist): {message}"
            );
        }
        other => panic!("a nonexistent-topic demand must be a loud Error, got {other:?}"),
    }
}

// ---------------------------------------------------------------------------
// (6) status counters match a hand oracle after a known publish sequence.
// ---------------------------------------------------------------------------

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn status_counters_match_oracle() {
    const N: usize = 5;
    let manager = test_manager("status");
    let topic = format!("/wire/status/{}", unique_id());
    let mut publisher = manager
        .create_publisher_simple(&topic, MaxSliceLen::const_new(256))
        .expect("producer");

    let daemon = disabled_endpoint([205; 32]).await;
    let client = disabled_endpoint([15; 32]).await;
    let client_key = *client.id().as_bytes();
    let authz = Arc::new(claimed_paired(client_key));
    let plane = Arc::new(WirePlane::with_manager("robot-stat", manager.clone()));
    let daemon_addr = loopback_addr(daemon.id(), &daemon.bound_sockets());

    let daemon_ref = &daemon;
    let client_ref = &client;
    let topic_ref = &topic;

    let daemon_fut = async move {
        let accepted = bounded("accept_one", accept_one(daemon_ref))
            .await
            .expect("accept ok")
            .expect("incoming");
        handle_accepted_with_wire(accepted, authz, None, Some(plane)).await;
    };

    let client_fut = async move {
        let conn = bounded("dial", dial(client_ref, daemon_addr, alpn::WIRE))
            .await
            .expect("dial wire");
        let (mut send, mut recv) = open_control(&conn).await;
        let d = request(
            &mut send,
            &mut recv,
            &WireRequest::Demand {
                topic: topic_ref.clone(),
            },
        )
        .await;
        assert_eq!(
            d,
            WireResponse::DemandAccepted {
                topic: topic_ref.clone()
            }
        );

        // Publish N frames with 10ms wire-timestamp spacing (a known Hz window).
        for i in 0..N {
            let frame = make_wire_frame(i as u32, 1_000_000 + i as u64 * 10_000_000, b"status");
            publisher.publish_raw(&frame).expect("publish_raw");
        }
        // Read all N off the uni stream (a frame can't be read before it is
        // written → the writer forwarded ≥ N).
        let (_topic, frames) = read_uni_frames(&conn, N).await;
        assert_eq!(frames.len(), N, "read all N frames");

        // Poll status until the counters converge (bounded) — the writer bumps
        // wan_forwarded on a separate task, so allow it to catch up.
        let deadline = Instant::now() + Duration::from_secs(10);
        loop {
            let resp = request(&mut send, &mut recv, &WireRequest::Status).await;
            let status = match resp {
                WireResponse::Status(s) => s,
                other => panic!("expected Status, got {other:?}"),
            };
            let row = status
                .topics
                .iter()
                .find(|t| &t.topic == topic_ref)
                .expect("status lists the demanded topic")
                .clone();
            if row.wan_forwarded == N as u64 && row.frames_seen == N as u64 {
                // Converged: assert the full hand oracle.
                assert_eq!(row.wan_dropped, 0, "no stall → zero drops");
                assert_eq!(row.frames_seen, N as u64);
                assert_eq!(row.wan_forwarded, N as u64);
                assert!(row.stream_alive, "a healthy demanded stream is alive");
                // Hz from wire timestamps: (N-1) frames over (N-1)*10ms = 100 Hz.
                assert!(
                    (row.hz - 100.0).abs() < 1.0,
                    "hz ~= 100 from wire timestamps, got {}",
                    row.hz
                );
                break;
            }
            assert!(
                Instant::now() < deadline,
                "status counters did not converge: {row:?}"
            );
            tokio::time::sleep(Duration::from_millis(20)).await;
        }

        let _ = send.finish();
        drop(conn);
    };

    tokio::join!(daemon_fut, client_fut);
}

// ---------------------------------------------------------------------------
// (7) schema verb: serves the closure over the wire when a source is wired;
//     an explicit not-found otherwise.
// ---------------------------------------------------------------------------

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn schema_verb_serves_closure_and_honest_not_found() {
    use cerulion_core::transport::cerulion_q::{SchemaDoc, SchemaEncoding};
    use std::collections::BTreeMap;

    let manager = test_manager("schema");
    let served_topic = "/wire/schema/state".to_string();
    let mut topic_types = BTreeMap::new();
    topic_types.insert(served_topic.clone(), "acme/State".to_string());
    let mut docs = BTreeMap::new();
    docs.insert(
        "acme/State".to_string(),
        SchemaDoc {
            qualified: "acme/State".to_string(),
            encoding: SchemaEncoding::Msg,
            text: "acme/Sub sub\n".to_string(),
            deps: vec!["acme/Sub".to_string()],
        },
    );
    docs.insert(
        "acme/Sub".to_string(),
        SchemaDoc {
            qualified: "acme/Sub".to_string(),
            encoding: SchemaEncoding::Msg,
            text: "float64 x\n".to_string(),
            deps: vec![],
        },
    );
    let plane =
        Arc::new(WirePlane::with_manager("robot-sch", manager).with_schema(topic_types, docs));

    let daemon = disabled_endpoint([206; 32]).await;
    let client = disabled_endpoint([16; 32]).await;
    let client_key = *client.id().as_bytes();
    let authz = Arc::new(claimed_paired(client_key));
    let daemon_addr = loopback_addr(daemon.id(), &daemon.bound_sockets());

    let daemon_ref = &daemon;
    let client_ref = &client;

    let daemon_fut = async move {
        let accepted = bounded("accept_one", accept_one(daemon_ref))
            .await
            .expect("accept ok")
            .expect("incoming");
        handle_accepted_with_wire(accepted, authz, None, Some(plane)).await;
    };

    let client_fut = async move {
        let conn = bounded("dial", dial(client_ref, daemon_addr, alpn::WIRE))
            .await
            .expect("dial wire");
        let (mut send, mut recv) = open_control(&conn).await;
        let found = request(
            &mut send,
            &mut recv,
            &WireRequest::Schema {
                topic: "/wire/schema/state".to_string(),
            },
        )
        .await;
        let missing = request(
            &mut send,
            &mut recv,
            &WireRequest::Schema {
                topic: "/wire/schema/unknown".to_string(),
            },
        )
        .await;
        let _ = send.finish();
        drop(conn);
        (found, missing)
    };

    let (_, (found, missing)) = tokio::join!(daemon_fut, client_fut);

    // Found: the closure (root first, then its custom dep) via the real SchemaReply.
    match found {
        WireResponse::Schema(reply) => {
            assert_eq!(reply.error, None);
            let names: Vec<&str> = reply.docs.iter().map(|d| d.qualified.as_str()).collect();
            assert_eq!(
                names,
                vec!["acme/State", "acme/Sub"],
                "root first, closure walked"
            );
        }
        other => panic!("expected a Schema reply, got {other:?}"),
    }
    // Not-found: an explicit structured error naming the topic (never silence).
    match missing {
        WireResponse::Schema(reply) => {
            assert!(reply.docs.is_empty());
            let reason = reply.error.expect("not-found carries a reason");
            assert!(
                reason.contains("/wire/schema/unknown"),
                "not-found names the topic: {reason}"
            );
        }
        other => panic!("expected a Schema reply, got {other:?}"),
    }
}

// ---------------------------------------------------------------------------
// (8) a flow-control-STALLED writer must not wedge undemand. The desk
//     demands a topic then never reads its uni stream; a burst of large frames
//     fills the QUIC window + stalls the writer; `undemand` on the still-healthy
//     control stream must COMPLETE within a bound (not hang forever).
// ---------------------------------------------------------------------------

/// A `DemandedTopic::stop` that does a bare
/// unbounded `self.writer.await` fails this test: a flow-control-stalled `write_frame` never
/// completes, so `undemand` hangs forever. With the writer aborted after a
/// short grace, `undemand` returns promptly. The desk deliberately NEVER reads
/// the uni stream, so a burst of ~1 MiB frames fills the QUIC stream flow window
/// (default ~1.25 MB) and parks the writer inside `write_all`.
#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn stalled_writer_undemand_completes_within_a_bound() {
    let manager = test_manager("stall");
    let topic = format!("/wire/stall/{}", unique_id());
    // Large frames so a handful fills the QUIC flow window and stalls the writer.
    const FRAME_BYTES: usize = 1024 * 1024;
    let mut publisher = manager
        .create_publisher_simple(&topic, MaxSliceLen::const_new(2 * 1024 * 1024))
        .expect("producer");

    let daemon = disabled_endpoint([207; 32]).await;
    let client = disabled_endpoint([17; 32]).await;
    let client_key = *client.id().as_bytes();
    let authz = Arc::new(claimed_paired(client_key));
    let plane = Arc::new(WirePlane::with_manager("robot-stall", manager.clone()));
    let daemon_addr = loopback_addr(daemon.id(), &daemon.bound_sockets());

    let daemon_ref = &daemon;
    let client_ref = &client;
    let topic_ref = &topic;

    let daemon_fut = async move {
        let accepted = bounded("accept_one", accept_one(daemon_ref))
            .await
            .expect("accept ok")
            .expect("incoming");
        handle_accepted_with_wire(accepted, authz, None, Some(plane)).await;
    };

    let client_fut = async move {
        let conn = bounded("dial", dial(client_ref, daemon_addr, alpn::WIRE))
            .await
            .expect("dial wire");
        let (mut send, mut recv) = open_control(&conn).await;
        let d = request(
            &mut send,
            &mut recv,
            &WireRequest::Demand {
                topic: topic_ref.clone(),
            },
        )
        .await;
        assert_eq!(
            d,
            WireResponse::DemandAccepted {
                topic: topic_ref.clone()
            }
        );

        // Publish a burst of large frames. The desk NEVER accepts/reads the uni
        // stream, so the QUIC window fills and the robot's writer stalls in
        // write_all. The drop-to-live queue keeps only the freshest 8.
        let big = vec![0xABu8; FRAME_BYTES];
        for i in 0..24u32 {
            let frame = make_wire_frame(i, 1_000_000 + i as u64, &big);
            publisher.publish_raw(&frame).expect("publish_raw");
        }
        // Give the writer time to fill the window + park.
        tokio::time::sleep(Duration::from_millis(400)).await;

        // Undemand on the STILL-HEALTHY control stream. Without the abort this hangs
        // (the stalled writer's `self.writer.await` never returns); with it the
        // writer is aborted after WRITER_STOP_GRACE, so it returns in well under
        // this 8s bound. This dedicated timeout IS the discriminator.
        let undemand_req = serde_json::to_vec(&WireRequest::Undemand {
            topic: topic_ref.clone(),
        })
        .unwrap();
        write_frame(&mut send, &undemand_req)
            .await
            .expect("write undemand");
        let reply = tokio::time::timeout(
            Duration::from_secs(8),
            read_frame(&mut recv, DEFAULT_MAX_FRAME_LEN),
        )
        .await
        .expect("undemand must NOT hang on a stalled writer")
        .expect("read undemand reply");
        let _ = send.finish();
        drop(conn);
        serde_json::from_slice::<WireResponse>(&reply).expect("decode reply")
    };

    let (_, reply) = tokio::join!(daemon_fut, client_fut);
    assert_eq!(
        reply,
        WireResponse::Undemanded {
            topic: topic.clone(),
            was_demanded: true,
        },
        "undemand of a stalled topic tears it down cleanly"
    );
}

// ---------------------------------------------------------------------------
// (9) a uni-stream writer death (QUIC STOP_SENDING) while the control
//     connection lives must tear the tap down (SHM slot released), surface
//     stream_alive=false in status, and let a re-demand RE-ATTACH a fresh stream.
// ---------------------------------------------------------------------------

/// A writer death that just `return`s without
/// `channel.stop()` fails this test: the drain thread keeps the SHM tap slot pinned + spinning,
/// the topic stays `demanded`, and a re-demand idempotently short-circuits to
/// DemandAccepted WITH NO STREAM (dead + unrecoverable). Here the writer sets
/// stop (drain exits, slot released), status reports stream_alive=false, and a
/// re-demand reaps + re-attaches a fresh, delivering stream.
#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn writer_death_releases_slot_status_dead_and_redemand_reattaches() {
    let manager = test_manager("death");
    let topic = format!("/wire/death/{}", unique_id());
    let mut publisher = manager
        .create_publisher_simple(&topic, MaxSliceLen::const_new(256))
        .expect("producer");

    let daemon = disabled_endpoint([208; 32]).await;
    let client = disabled_endpoint([18; 32]).await;
    let client_key = *client.id().as_bytes();
    let authz = Arc::new(claimed_paired(client_key));
    let plane = Arc::new(WirePlane::with_manager("robot-death", manager.clone()));
    let daemon_addr = loopback_addr(daemon.id(), &daemon.bound_sockets());

    let daemon_ref = &daemon;
    let client_ref = &client;
    let topic_ref = &topic;
    let manager_ref = &manager;

    let daemon_fut = async move {
        let accepted = bounded("accept_one", accept_one(daemon_ref))
            .await
            .expect("accept ok")
            .expect("incoming");
        handle_accepted_with_wire(accepted, authz, None, Some(plane)).await;
    };

    let client_fut = async move {
        let conn = bounded("dial", dial(client_ref, daemon_addr, alpn::WIRE))
            .await
            .expect("dial wire");
        let (mut send, mut recv) = open_control(&conn).await;

        // 1) Demand + confirm the stream is live (preamble + one frame).
        let d = request(
            &mut send,
            &mut recv,
            &WireRequest::Demand {
                topic: topic_ref.clone(),
            },
        )
        .await;
        assert_eq!(
            d,
            WireResponse::DemandAccepted {
                topic: topic_ref.clone()
            }
        );
        let mut uni = bounded("accept_uni", accept_uni_frame_stream(&conn))
            .await
            .expect("accept_uni");
        let preamble = bounded("preamble", read_frame(&mut uni, DEFAULT_MAX_FRAME_LEN))
            .await
            .expect("preamble");
        let sp: StreamPreamble = serde_json::from_slice(&preamble).unwrap();
        assert_eq!(&sp.topic, topic_ref);
        let first = make_wire_frame(0, 1_000_000, b"alive-1");
        publisher.publish_raw(&first).expect("publish first");
        let got = bounded("first frame", read_frame(&mut uni, DEFAULT_MAX_FRAME_LEN))
            .await
            .expect("read first frame");
        assert_eq!(got, first, "stream is live before the kill");

        // 2) KILL the uni stream via QUIC STOP_SENDING (control stream stays up).
        uni.stop(iroh::endpoint::VarInt::from_u32(7))
            .expect("stop the uni recv");
        drop(uni);

        // 3) Keep publishing so the robot's writer attempts a write + sees the
        //    reset → dies → channel.stop() → drain releases the slot.
        for i in 1..40u32 {
            let f = make_wire_frame(i, 1_000_000 + i as u64, b"post-kill");
            let _ = publisher.publish_raw(&f);
            tokio::time::sleep(Duration::from_millis(10)).await;
        }

        // 4) status must report the topic stream_alive == false (surfaced dead),
        //    bounded poll.
        let deadline = Instant::now() + Duration::from_secs(10);
        loop {
            let resp = request(&mut send, &mut recv, &WireRequest::Status).await;
            let status = match resp {
                WireResponse::Status(s) => s,
                other => panic!("expected Status, got {other:?}"),
            };
            let row = status.topics.iter().find(|t| &t.topic == topic_ref);
            if let Some(row) = row {
                if !row.stream_alive {
                    break; // surfaced dead — the status pin
                }
            }
            assert!(
                Instant::now() < deadline,
                "status never surfaced the dead stream: {status:?}"
            );
            tokio::time::sleep(Duration::from_millis(50)).await;
        }

        // 5) The SHM slot was released: a fresh direct tap on the manager attaches.
        manager_ref
            .create_data_only_subscriber(topic_ref)
            .expect("slot released after writer death");

        // 6) RE-DEMAND reaps the dead topic + re-attaches a fresh, DELIVERING
        //    stream (an idempotent short-circuit → DemandAccepted with no new
        //    stream would make the accept_uni below time out).
        let d2 = request(
            &mut send,
            &mut recv,
            &WireRequest::Demand {
                topic: topic_ref.clone(),
            },
        )
        .await;
        assert_eq!(
            d2,
            WireResponse::DemandAccepted {
                topic: topic_ref.clone()
            }
        );
        let mut uni2 = bounded("accept_uni2", accept_uni_frame_stream(&conn))
            .await
            .expect("re-demand opened a FRESH uni stream (re-attach)");
        let pre2 = bounded("preamble2", read_frame(&mut uni2, DEFAULT_MAX_FRAME_LEN))
            .await
            .expect("fresh preamble");
        let sp2: StreamPreamble = serde_json::from_slice(&pre2).unwrap();
        assert_eq!(&sp2.topic, topic_ref);
        let reborn = make_wire_frame(100, 5_000_000, b"reattached");
        publisher.publish_raw(&reborn).expect("publish reborn");
        let got2 = bounded("reborn frame", read_frame(&mut uni2, DEFAULT_MAX_FRAME_LEN))
            .await
            .expect("re-attached stream delivers");
        assert_eq!(got2, reborn, "the re-attached stream delivers fresh frames");

        let _ = send.finish();
        drop(conn);
    };

    tokio::join!(daemon_fut, client_fut);
}

// ---------------------------------------------------------------------------
// (10) a peeked schema_hash is CACHED per connection, so a second
//      catalog reuses it (never re-peeks). Proven functionally: the producer is
//      stopped between the two catalogs — a re-peek of the now-silent topic would
//      yield None, but the cache still carries the hash.
// ---------------------------------------------------------------------------

/// A `build_catalog` that sources its known set
/// from the DEMANDED set alone (rather than the live
/// source, `all_cached_schema_hashes`) fails this test: a peeked-but-not-demanded topic's
/// hash is NEVER cached — every catalog re-peeks it. This test SILENCES the
/// producer (keeping its publisher ALIVE so the topic stays in `list_topics`, but
/// emitting no frames) between two catalogs: demanded-only, the 2nd catalog re-peeks the
/// now-silent topic → schema_hash None; the cache carries the hash →
/// still present.
#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn catalog_caches_peeked_hash_across_calls() {
    let manager = test_manager("catcache");
    let topic = format!("/wire/catcache/{}", unique_id());
    let mut publisher = manager
        .create_publisher_simple(&topic, MaxSliceLen::const_new(256))
        .expect("producer");
    // Two-phase control: `silent` stops PUBLISHING (topic goes quiet but the
    // publisher — and thus the topic's SHM service — stays ALIVE, so it remains in
    // `list_topics`); `terminate` exits the thread + drops the publisher. Dropping
    // the publisher would REMOVE the topic entirely, which is NOT what we want to
    // test (we want a live-but-silent topic that a re-peek would miss).
    let silent = Arc::new(AtomicBool::new(false));
    let terminate = Arc::new(AtomicBool::new(false));
    let producer_silent = silent.clone();
    let producer_terminate = terminate.clone();
    let producer = std::thread::spawn(move || {
        let mut seq = 0u32;
        while !producer_terminate.load(Ordering::Relaxed) {
            if !producer_silent.load(Ordering::Relaxed) {
                let frame = make_wire_frame(seq, 1_000_000 + seq as u64 * 10_000, b"cache-probe");
                let _ = publisher.publish_raw(&frame);
                seq = seq.wrapping_add(1);
            }
            std::thread::sleep(Duration::from_millis(3));
        }
        // Publisher drops HERE (after terminate) — the topic is removed only now.
    });

    let daemon = disabled_endpoint([209; 32]).await;
    let client = disabled_endpoint([19; 32]).await;
    let client_key = *client.id().as_bytes();
    let authz = Arc::new(claimed_paired(client_key));
    let plane = Arc::new(WirePlane::with_manager("robot-cc", manager.clone()));
    let daemon_addr = loopback_addr(daemon.id(), &daemon.bound_sockets());

    let daemon_ref = &daemon;
    let client_ref = &client;
    let topic_ref = &topic;
    let silent_ref = &silent;

    let daemon_fut = async move {
        let accepted = bounded("accept_one", accept_one(daemon_ref))
            .await
            .expect("accept ok")
            .expect("incoming");
        handle_accepted_with_wire(accepted, authz, None, Some(plane)).await;
    };

    let client_fut = async move {
        let conn = bounded("dial", dial(client_ref, daemon_addr, alpn::WIRE))
            .await
            .expect("dial wire");
        let (mut send, mut recv) = open_control(&conn).await;

        // 1) First catalog: the peek observes + caches the hash.
        let first = request(&mut send, &mut recv, &WireRequest::Catalog).await;
        let hash1 = catalog_hash_for(&first, topic_ref);
        assert_eq!(
            hash1,
            Some(ORACLE_SCHEMA_HASH),
            "first catalog peeks the live topic's hash"
        );

        // 2) SILENCE the producer (publisher stays ALIVE → topic stays in
        //    list_topics, but no more frames). A re-peek would now time out → None.
        silent_ref.store(true, Ordering::Relaxed);
        // Let the producer thread observe the silence + the SHM queue drain, so a
        // fresh peek tap (no history) sees nothing.
        tokio::time::sleep(Duration::from_millis(250)).await;

        // 3) Second catalog: MUST reuse the cached hash. A
        //    demanded-only cache misses → a re-peek of the silent topic → None.
        let second = request(&mut send, &mut recv, &WireRequest::Catalog).await;
        let hash2 = catalog_hash_for(&second, topic_ref);
        assert_eq!(
            hash2,
            Some(ORACLE_SCHEMA_HASH),
            "second catalog reuses the CACHED hash — a re-peek of the now-silent \
             topic would return None"
        );

        let _ = send.finish();
        drop(conn);
    };

    let (_, ()) = tokio::join!(daemon_fut, client_fut);
    terminate.store(true, Ordering::Relaxed);
    let _ = producer.join();
}

/// Pull a topic's catalog `schema_hash` out of a `Catalog` reply.
fn catalog_hash_for(resp: &WireResponse, topic: &str) -> Option<u64> {
    match resp {
        WireResponse::Catalog(reply) => reply
            .entries
            .iter()
            .find(|e| e.topic == topic)
            .and_then(|e| e.schema_hash),
        other => panic!("expected a Catalog reply, got {other:?}"),
    }
}

// ---------------------------------------------------------------------------
// (11) a malformed control frame is answered with a structured
//      WireResponse::Error (never silent), and the connection SURVIVES — a
//      following valid verb still succeeds.
// ---------------------------------------------------------------------------

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn malformed_control_frame_is_a_loud_error_and_connection_survives() {
    let manager = test_manager("malformed");
    let daemon = disabled_endpoint([210; 32]).await;
    let client = disabled_endpoint([20; 32]).await;
    let client_key = *client.id().as_bytes();
    let authz = Arc::new(claimed_paired(client_key));
    let plane = Arc::new(WirePlane::with_manager("robot-mal", manager));
    let daemon_addr = loopback_addr(daemon.id(), &daemon.bound_sockets());

    let daemon_ref = &daemon;
    let client_ref = &client;

    let daemon_fut = async move {
        let accepted = bounded("accept_one", accept_one(daemon_ref))
            .await
            .expect("accept ok")
            .expect("incoming");
        handle_accepted_with_wire(accepted, authz, None, Some(plane)).await;
    };

    let client_fut = async move {
        let conn = bounded("dial", dial(client_ref, daemon_addr, alpn::WIRE))
            .await
            .expect("dial wire");
        let (mut send, mut recv) = open_control(&conn).await;

        // 1) Garbage (not a WireRequest) → a structured Error (never silent, never
        //    a panic/hang).
        bounded("write garbage", write_frame(&mut send, b"{not valid json"))
            .await
            .expect("write garbage");
        let err_bytes = bounded("read error", read_frame(&mut recv, DEFAULT_MAX_FRAME_LEN))
            .await
            .expect("read error reply");
        let err: WireResponse = serde_json::from_slice(&err_bytes).expect("decode error");
        match err {
            WireResponse::Error { topic, message } => {
                assert_eq!(topic, None, "a malformed frame has no topic");
                assert!(
                    message.contains("malformed"),
                    "error names the malformation: {message}"
                );
            }
            other => panic!("expected a malformed Error, got {other:?}"),
        }

        // 2) The connection SURVIVES: a following valid verb still succeeds.
        let status = request(&mut send, &mut recv, &WireRequest::Status).await;
        match status {
            WireResponse::Status(s) => assert!(
                s.topics.is_empty(),
                "no demands → empty status (connection alive after the bad frame)"
            ),
            other => panic!("connection did not survive the bad frame: {other:?}"),
        }

        let _ = send.finish();
        drop(conn);
    };

    tokio::join!(daemon_fut, client_fut);
}

// ===========================================================================
// Robot serving-plane per-demand authorization + mid-session eviction.
//
// The accept gate admits a wire connection ONCE (paired CAP_OBSERVE). The daemon
// installs the PairingAuthorizer's DemandAuthorizer on the SERVING plane, keyed by
// the DEMANDER's cryptographically-authenticated key (`connection.remote_id`), so a
// demand is gated PER TOPIC and a mid-session owner-revoke / grant-expiry DROPS an
// already-established stream (a periodic sweep). Every fixture is a REAL cert chain /
// a hand-oracle authorizer — never a self-compare (Principle #13).
// ===========================================================================

// Distinct from the file's `OWNER` (`[10; 32]` == `[0x0A; 32]`), else the grant is a
// self-grant (`OwnerGrantForOwner`).
const SUBJECT: AccountId = AccountId([0x5C; 32]);
const ROBOT: RobotId = RobotId([0x0B; 32]);
const ISSUED: u64 = 500_000_000_000;

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

/// A CLAIMED store (owner = [`OWNER`]) provisioned with the REAL root pubkey `pk(sk(1))`
/// so an owner-grant chain built with `sk` verifies. Seeds: 1 = root, 10 = intermediate,
/// 30 = owner device.
fn grant_root_store() -> TrustStore {
    let root_set = RootSet::new(vec![pk(&sk(1))], 1).unwrap();
    let mut store = TrustStore::provision(ROBOT, pk(&sk(0x77)), root_set, CHASSIS, 0).unwrap();
    store
        .claim(OWNER, CHASSIS, PrincipalKind::Human, T_NOW)
        .unwrap();
    store
}

/// A real owner-signed grant for `subject_device_key` (the desk's iroh key), granting
/// [`SUBJECT`] the operator scope on [`ROBOT`] with a FINITE expiry — so advancing the
/// robot's clock to `expiry` denies it (grant-expiry, the revocation model).
fn expiring_owner_grant_for(subject_device_key: [u8; 32], expiry: u64) -> OwnerGrantWire {
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
                not_after_ns: expiry,
            },
            issued_at_ns: ISSUED,
            owner: OWNER,
            owner_device_key: pk(&sk(30)),
        }
        .sign(&sk(30)),
    }
}

/// A LIVE `SharedTrust` whose access-read clock is `clock` (RETAINED by the caller so it
/// can be driven past `expiry` mid-session), with `desk_key` established via an expiring
/// owner grant → Allowed at `T_NOW`, NotAllowed at `expiry`.
fn revocable_shared(desk_key: [u8; 32], clock: RemotedClock, expiry: u64) -> SharedTrust {
    let shared = SharedTrust::new_with_clock(
        grant_root_store(),
        DeviceAccountIndex::new(),
        Vec::new(),
        clock,
    );
    let pres = expiring_owner_grant_for(desk_key, expiry).into_presentation();
    shared
        .establish_owner_grant(&pres, &desk_key, None, T_NOW)
        .expect("establish the desk's expiring owner grant");
    shared
}

/// A hand-oracle [`DemandAuthorizer`] that DENIES exactly one topic (any demander) — the
/// stand-in for a per-topic-scoped authorizer. The assert pins that the robot serving
/// plane threads a `Wan` subject (the demander key), not a self-reported field.
struct DenyTopic(String);
impl DemandAuthorizer for DenyTopic {
    fn authorize_demand(&self, subject: &DemandSubject, topic: &str) -> DemandDecision {
        assert!(
            matches!(subject, DemandSubject::Wan { .. }),
            "the robot serving plane must pass a Wan subject, got {subject:?}"
        );
        if topic == self.0 {
            DemandDecision::deny(format!("test: {topic} not authorized"))
        } else {
            DemandDecision::Allow
        }
    }
}

// ---------------------------------------------------------------------------
// (A) per-topic DEMAND gate: a denied topic is refused LOUDLY; a different topic
//     is admitted + streams byte-identically (selectivity / anti-tautology).
// ---------------------------------------------------------------------------

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn demand_gate_refuses_a_denied_topic_and_admits_others() {
    let manager = test_manager("gate");
    let allowed_topic = format!("/wire/gate/allowed/{}", unique_id());
    let denied_topic = format!("/wire/gate/denied/{}", unique_id());
    // The allowed topic has a real producer (its demand streams); the denied one is
    // refused BEFORE any tap, so it needs none.
    let mut publisher = manager
        .create_publisher_simple(&allowed_topic, MaxSliceLen::const_new(256))
        .expect("producer");

    let daemon = disabled_endpoint([211; 32]).await;
    let client = disabled_endpoint([12; 32]).await;
    let client_key = *client.id().as_bytes();
    // Accept-admitted (paired CAP_OBSERVE); the per-topic DEMAND gate is a DenyTopic stub.
    let authz = Arc::new(claimed_paired(client_key));
    let plane = Arc::new(
        WirePlane::with_manager("robot-gate", manager.clone())
            .with_demand_authorizer(Arc::new(DenyTopic(denied_topic.clone()))),
    );
    let daemon_addr = loopback_addr(daemon.id(), &daemon.bound_sockets());

    let daemon_ref = &daemon;
    let client_ref = &client;
    let allowed_ref = &allowed_topic;
    let denied_ref = &denied_topic;

    let daemon_fut = async move {
        let accepted = bounded("accept_one", accept_one(daemon_ref))
            .await
            .expect("accept ok")
            .expect("incoming");
        handle_accepted_with_wire(accepted, authz, None, Some(plane)).await;
    };

    let client_fut = async move {
        let conn = bounded("dial", dial(client_ref, daemon_addr, alpn::WIRE))
            .await
            .expect("dial wire");
        let (mut send, mut recv) = open_control(&conn).await;

        // (a) the DENIED topic → a loud Error naming it (never a stream).
        let denied_resp = request(
            &mut send,
            &mut recv,
            &WireRequest::Demand {
                topic: denied_ref.clone(),
            },
        )
        .await;
        match denied_resp {
            WireResponse::Error { topic, message } => {
                assert_eq!(
                    topic.as_deref(),
                    Some(denied_ref.as_str()),
                    "the error names the denied topic"
                );
                assert!(
                    message.contains("refused"),
                    "the error is the gate refusal: {message}"
                );
            }
            other => panic!("a denied topic must be refused, got {other:?}"),
        }

        // (b) SELECTIVITY / anti-tautology: a DIFFERENT topic is admitted + streams.
        let allowed_resp = request(
            &mut send,
            &mut recv,
            &WireRequest::Demand {
                topic: allowed_ref.clone(),
            },
        )
        .await;
        assert_eq!(
            allowed_resp,
            WireResponse::DemandAccepted {
                topic: allowed_ref.clone()
            },
            "an untargeted topic is admitted through the gate"
        );
        for frame in oracle_frames() {
            publisher.publish_raw(&frame).expect("publish_raw");
        }
        let (stream_topic, frames) = read_uni_frames(&conn, 3).await;
        assert_eq!(&stream_topic, allowed_ref, "the admitted topic streams");
        assert_eq!(
            frames,
            oracle_frames(),
            "the admitted stream is byte-identical to the hand oracle"
        );

        let _ = send.finish();
        drop(conn);
    };

    tokio::join!(daemon_fut, client_fut);
}

// ---------------------------------------------------------------------------
// (B) HEADLINE: a mid-session owner-grant EXPIRY (the grant-expiry revocation model)
//     DROPS the already-established stream within the sweep bound — the REAL
//     PairingAuthorizer wired as BOTH the accept gate AND the demand gate (no inert
//     shipping). The desk was admitted + streaming; advancing the robot's own clock
//     past the grant's expiry makes the sweep re-check the demander → Deny → evict.
// ---------------------------------------------------------------------------

/// A continuous producer thread on `topic` (created on the caller's thread first so the
/// data service exists before the demand, then moved in). Publishes every 5 ms until
/// `stop`.
fn spawn_continuous_producer(
    manager: &Arc<TransportManager>,
    topic: &str,
    stop: Arc<AtomicBool>,
) -> std::thread::JoinHandle<()> {
    let publisher = manager
        .create_publisher_simple(topic, MaxSliceLen::const_new(256))
        .expect("producer");
    std::thread::spawn(move || {
        let mut publisher = publisher;
        let mut seq = 0u32;
        while !stop.load(Ordering::Relaxed) {
            let frame = make_wire_frame(seq, 1_000_000 + seq as u64 * 10_000, b"probe");
            let _ = publisher.publish_raw(&frame);
            seq = seq.wrapping_add(1);
            std::thread::sleep(Duration::from_millis(5));
        }
    })
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn mid_session_revoke_evicts_the_established_stream() {
    const EXPIRY: u64 = 2_000_000_000_000; // > T_NOW (1e12), < grant_validity() (1e14)
    let manager = test_manager("evict");
    let topic = format!("/wire/evict/{}", unique_id());
    let stop = Arc::new(AtomicBool::new(false));
    let producer = spawn_continuous_producer(&manager, &topic, stop.clone());

    let daemon = disabled_endpoint([212; 32]).await;
    let client = disabled_endpoint([13; 32]).await;
    let client_key = *client.id().as_bytes();

    // The REAL PairingAuthorizer over a live SharedTrust with a RETAINED, advanceable
    // clock; the desk's iroh key is established via an EXPIRING owner grant.
    let clock = RemotedClock::fixed(T_NOW);
    let shared = revocable_shared(client_key, clock.clone(), EXPIRY);
    assert!(
        matches!(
            shared.snapshot_for_key(&client_key).1,
            KeyAccess::Allowed(_)
        ),
        "the desk is admitted at T_NOW (before expiry)"
    );
    let authz = Arc::new(PairingAuthorizer::from_shared(shared));
    let plane = Arc::new(
        WirePlane::with_manager("robot-evict", manager.clone())
            // SAME authorizer as the accept gate → an owner-revoke / expiry is honored
            // per topic AND evicts the live stream (both read the one live SharedTrust).
            .with_demand_authorizer(authz.clone())
            .with_revocation_sweep_interval(Duration::from_millis(200)),
    );
    let daemon_addr = loopback_addr(daemon.id(), &daemon.bound_sockets());

    let daemon_ref = &daemon;
    let client_ref = &client;
    let topic_ref = &topic;
    let clock_for_client = clock.clone();

    let daemon_fut = async move {
        let accepted = bounded("accept_one", accept_one(daemon_ref))
            .await
            .expect("accept ok")
            .expect("incoming");
        handle_accepted_with_wire(accepted, authz, None, Some(plane)).await;
    };

    let client_fut = async move {
        let conn = bounded("dial", dial(client_ref, daemon_addr, alpn::WIRE))
            .await
            .expect("dial wire");
        let (mut send, mut recv) = open_control(&conn).await;
        let resp = request(
            &mut send,
            &mut recv,
            &WireRequest::Demand {
                topic: topic_ref.clone(),
            },
        )
        .await;
        assert_eq!(
            resp,
            WireResponse::DemandAccepted {
                topic: topic_ref.clone()
            },
            "the paired + allowed demand is accepted at T_NOW"
        );

        // Accept the uni stream, confirm frames FLOW pre-revoke (a healthy stream).
        let mut ustream = bounded("accept_uni", accept_uni_frame_stream(&conn))
            .await
            .expect("accept_uni");
        let preamble = bounded("preamble", read_frame(&mut ustream, DEFAULT_MAX_FRAME_LEN))
            .await
            .expect("preamble");
        let p: StreamPreamble = serde_json::from_slice(&preamble).expect("decode preamble");
        assert_eq!(&p.topic, topic_ref, "preamble names the demanded topic");
        for _ in 0..3 {
            bounded(
                "pre-revoke frame",
                read_frame(&mut ustream, DEFAULT_MAX_FRAME_LEN),
            )
            .await
            .expect("the stream is healthy pre-revoke");
        }

        // REVOKE mid-session: advance the robot's own trusted clock past the grant's
        // expiry. The periodic sweep re-checks the demander through the SAME live
        // SharedTrust → Deny → it evicts the tap (and, on a full revoke, closes the
        // connection), so the desk's uni-stream read ERRORS within the sweep bound.
        clock_for_client.set(EXPIRY);
        let deadline = Instant::now() + Duration::from_secs(10);
        let mut evicted = false;
        while Instant::now() < deadline {
            match tokio::time::timeout(
                Duration::from_millis(500),
                read_frame(&mut ustream, DEFAULT_MAX_FRAME_LEN),
            )
            .await
            {
                Ok(Ok(_frame)) => continue, // still draining pre-eviction frames
                Ok(Err(_)) => {
                    evicted = true; // stream reset = the tap was evicted
                    break;
                }
                Err(_) => continue, // no frame within 500ms; keep waiting for the reset
            }
        }
        assert!(
            evicted,
            "the revoked demander's established stream must be EVICTED within the sweep bound"
        );
        drop(conn);
    };

    tokio::join!(daemon_fut, client_fut);
    stop.store(true, Ordering::Relaxed);
    let _ = producer.join();
}

// ---------------------------------------------------------------------------
// (C) CONTROL / anti-tautology: the sweep does NOT evict an AUTHORIZED stream — with
//     no revoke, frames keep flowing across several sweep intervals. Proves the
//     eviction in (B) is attributable to the revoke, not the sweep firing at all.
// ---------------------------------------------------------------------------

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn sweep_does_not_evict_an_authorized_stream() {
    const EXPIRY: u64 = 2_000_000_000_000;
    let manager = test_manager("noevict");
    let topic = format!("/wire/noevict/{}", unique_id());
    let stop = Arc::new(AtomicBool::new(false));
    let producer = spawn_continuous_producer(&manager, &topic, stop.clone());

    let daemon = disabled_endpoint([213; 32]).await;
    let client = disabled_endpoint([14; 32]).await;
    let client_key = *client.id().as_bytes();

    // The SAME real-authorizer setup as (B) — but the clock is NEVER advanced, so the
    // demander stays Allowed and the sweep is a no-op.
    let clock = RemotedClock::fixed(T_NOW);
    let shared = revocable_shared(client_key, clock, EXPIRY);
    let authz = Arc::new(PairingAuthorizer::from_shared(shared));
    let plane = Arc::new(
        WirePlane::with_manager("robot-noevict", manager.clone())
            .with_demand_authorizer(authz.clone())
            .with_revocation_sweep_interval(Duration::from_millis(200)),
    );
    let daemon_addr = loopback_addr(daemon.id(), &daemon.bound_sockets());

    let daemon_ref = &daemon;
    let client_ref = &client;
    let topic_ref = &topic;

    let daemon_fut = async move {
        let accepted = bounded("accept_one", accept_one(daemon_ref))
            .await
            .expect("accept ok")
            .expect("incoming");
        handle_accepted_with_wire(accepted, authz, None, Some(plane)).await;
    };

    let client_fut = async move {
        let conn = bounded("dial", dial(client_ref, daemon_addr, alpn::WIRE))
            .await
            .expect("dial wire");
        let (mut send, mut recv) = open_control(&conn).await;
        let resp = request(
            &mut send,
            &mut recv,
            &WireRequest::Demand {
                topic: topic_ref.clone(),
            },
        )
        .await;
        assert_eq!(
            resp,
            WireResponse::DemandAccepted {
                topic: topic_ref.clone()
            }
        );

        let mut ustream = bounded("accept_uni", accept_uni_frame_stream(&conn))
            .await
            .expect("accept_uni");
        let _preamble = bounded("preamble", read_frame(&mut ustream, DEFAULT_MAX_FRAME_LEN))
            .await
            .expect("preamble");

        // Across >= 5 sweep intervals (@200ms) with NO revoke, the sweep re-checks an
        // ALLOWED demander and does NOT evict — frames keep flowing.
        let mut got = 0usize;
        let deadline = Instant::now() + Duration::from_millis(1300);
        while Instant::now() < deadline {
            match tokio::time::timeout(
                Duration::from_millis(500),
                read_frame(&mut ustream, DEFAULT_MAX_FRAME_LEN),
            )
            .await
            {
                Ok(Ok(_)) => got += 1,
                Ok(Err(e)) => {
                    panic!("an authorized stream must NOT be evicted by the sweep: {e}")
                }
                Err(_) => {} // slow producer window; keep waiting
            }
        }
        assert!(
            got >= 5,
            "an authorized stream keeps flowing across the sweeps (got {got})"
        );

        let _ = send.finish();
        drop(conn);
    };

    tokio::join!(daemon_fut, client_fut);
    stop.store(true, Ordering::Relaxed);
    let _ = producer.join();
}

// ---------------------------------------------------------------------------
// (D) FINDING-1 (eviction-bypass) REGRESSION PIN: a revoked demander that STALLS its
//     control-response stream (so the robot's control loop blocks in write_frame) is
//     STILL evicted within the sweep bound — because the sweep runs on its OWN task,
//     decoupled from control-loop progress. Reverting to a control-loop-coupled sweep
//     starves the sweep on the stalled write and this test times out (fails).
// ---------------------------------------------------------------------------

/// A controllable [`DemandAuthorizer`]: admits everything until `revoked` flips true,
/// then denies every topic (models a WHOLE-SUBJECT owner-revoke). Deterministic — the
/// verdict is driven by the flag the test sets, never a self-compare.
struct FlipAuthorizer(Arc<AtomicBool>);
impl DemandAuthorizer for FlipAuthorizer {
    fn authorize_demand(&self, subject: &DemandSubject, topic: &str) -> DemandDecision {
        assert!(
            matches!(subject, DemandSubject::Wan { .. }),
            "the serving plane must pass a Wan subject, got {subject:?}"
        );
        if self.0.load(Ordering::Relaxed) {
            DemandDecision::deny(format!("test: subject revoked; {topic} denied"))
        } else {
            DemandDecision::Allow
        }
    }
}

/// Read `ustream` until it RESETS (an error) or the bound elapses; `true` if reset.
async fn read_until_reset(ustream: &mut RecvStream) -> bool {
    let deadline = Instant::now() + Duration::from_secs(10);
    while Instant::now() < deadline {
        match tokio::time::timeout(
            Duration::from_millis(500),
            read_frame(ustream, DEFAULT_MAX_FRAME_LEN),
        )
        .await
        {
            Ok(Ok(_)) => continue,     // a frame flowed; keep reading
            Ok(Err(_)) => return true, // the stream reset = evicted
            Err(_) => continue,        // no frame this window; keep waiting
        }
    }
    false
}

/// A wire plane serving a topic whose SCHEMA is huge (`~256 KiB`), so a modest flood of
/// `Schema` requests the desk never reads reliably fills the desk's QUIC recv window →
/// the robot's control loop BLOCKS in `write_frame`. This is the deterministic way to put
/// the control loop into the stuck state the independent sweep exists for (a small-response flood can't
/// fill iroh's large auto-tuned window in a bounded test).
fn huge_schema_plane(
    manager: Arc<TransportManager>,
    authz: Arc<dyn DemandAuthorizer>,
) -> WirePlane {
    const BIG: &str = "/big/schema";
    let mut topic_types = BTreeMap::new();
    topic_types.insert(BIG.to_string(), "big/Type".to_string());
    let mut docs = BTreeMap::new();
    docs.insert(
        "big/Type".to_string(),
        SchemaDoc {
            qualified: "big/Type".to_string(),
            encoding: SchemaEncoding::Msg,
            // ~256 KiB of comment lines — a valid-enough .msg body for serve_schema (it
            // returns the text VERBATIM in the reply, which is all we need here).
            text: "# huge schema padding\n".repeat(9000),
            deps: vec![],
        },
    );
    WirePlane::with_manager("robot-stall", manager)
        .with_schema(topic_types, docs)
        .with_demand_authorizer(authz)
        .with_revocation_sweep_interval(Duration::from_millis(200))
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn stalled_control_stream_demander_is_still_evicted() {
    let manager = test_manager("stall");
    let topic = format!("/wire/stall/{}", unique_id());
    let stop = Arc::new(AtomicBool::new(false));
    let producer = spawn_continuous_producer(&manager, &topic, stop.clone());

    let daemon = disabled_endpoint([214; 32]).await;
    let client = disabled_endpoint([15; 32]).await;
    let client_key = *client.id().as_bytes();
    let revoked = Arc::new(AtomicBool::new(false));
    // Accept-admitted (paired); the DEMAND gate is a FlipAuthorizer the test revokes.
    let authz = Arc::new(claimed_paired(client_key));
    let plane = Arc::new(huge_schema_plane(
        manager.clone(),
        Arc::new(FlipAuthorizer(revoked.clone())),
    ));
    let daemon_addr = loopback_addr(daemon.id(), &daemon.bound_sockets());

    let daemon_ref = &daemon;
    let client_ref = &client;
    let topic_ref = &topic;
    let revoked_for_client = revoked.clone();

    let daemon_fut = async move {
        let accepted = bounded("accept_one", accept_one(daemon_ref))
            .await
            .expect("accept ok")
            .expect("incoming");
        handle_accepted_with_wire(accepted, authz, None, Some(plane)).await;
    };

    let client_fut = async move {
        let conn = bounded("dial", dial(client_ref, daemon_addr, alpn::WIRE))
            .await
            .expect("dial wire");
        let (mut send, mut recv) = open_control(&conn).await;
        let resp = request(
            &mut send,
            &mut recv,
            &WireRequest::Demand {
                topic: topic_ref.clone(),
            },
        )
        .await;
        assert_eq!(
            resp,
            WireResponse::DemandAccepted {
                topic: topic_ref.clone()
            }
        );

        // A uni reader on a connection clone detects the eviction (the uni stream reset)
        // — the MAIN task is busy stalling the control stream, so eviction detection runs
        // concurrently on its own (separate) QUIC stream.
        let conn_uni = conn.clone();
        let evicted = Arc::new(AtomicBool::new(false));
        let ev = evicted.clone();
        let uni_reader = tokio::spawn(async move {
            let mut ustream = match accept_uni_frame_stream(&conn_uni).await {
                Ok(s) => s,
                Err(_) => {
                    ev.store(true, Ordering::Relaxed);
                    return;
                }
            };
            let _ = read_frame(&mut ustream, DEFAULT_MAX_FRAME_LEN).await; // preamble
            if read_until_reset(&mut ustream).await {
                ev.store(true, Ordering::Relaxed);
            }
        });

        // STALL the control loop: flood huge-response `Schema` requests WITHOUT reading
        // the responses. The requests are tiny (all sends succeed), but each ~256 KiB
        // reply piles up unread in our recv window → after a few the robot's control loop
        // BLOCKS in `write_frame`, stuck. We don't need to detect the exact moment — the
        // volume GUARANTEES the block (a coupled sweep would then be starved).
        let schema_req = serde_json::to_vec(&WireRequest::Schema {
            topic: "/big/schema".to_string(),
        })
        .unwrap();
        for _ in 0..200 {
            // Bounded per-send: once our send window also fills (the robot stopped
            // reading), the send blocks — that is the stall, so stop flooding.
            if tokio::time::timeout(
                Duration::from_millis(300),
                write_frame(&mut send, &schema_req),
            )
            .await
            .map(|r| r.is_ok())
                != Ok(true)
            {
                break;
            }
        }
        // Give the robot a beat to reach the blocked write, then REVOKE. The INDEPENDENT
        // sweep must evict us DESPITE the stalled control loop.
        tokio::time::sleep(Duration::from_millis(300)).await;
        revoked_for_client.store(true, Ordering::Relaxed);

        let deadline = Instant::now() + Duration::from_secs(12);
        while !evicted.load(Ordering::Relaxed) && Instant::now() < deadline {
            tokio::time::sleep(Duration::from_millis(50)).await;
        }
        assert!(
            evicted.load(Ordering::Relaxed),
            "a revoked demander that STALLS its control stream must STILL be evicted by the \
             INDEPENDENT sweep; a control-loop-coupled sweep would starve here"
        );
        uni_reader.abort();
        let _ = uni_reader.await;
        drop(conn);
    };

    tokio::join!(daemon_fut, client_fut);
    stop.store(true, Ordering::Relaxed);
    let _ = producer.join();
}

// ---------------------------------------------------------------------------
// (E) FINDING-3 (whole-subject reality): today's ACL revokes WHOLE-SUBJECT (a grant
//     confers CAP_OBSERVE over all topics or none — there is NO per-topic scoping), so a
//     mid-session revoke evicts EVERY demanded topic + closes the connection. Two topics
//     on ONE connection; a real grant-expiry revoke drops BOTH streams.
// ---------------------------------------------------------------------------

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn full_revoke_evicts_both_streams_and_closes_the_connection() {
    const EXPIRY: u64 = 2_000_000_000_000;
    let manager = test_manager("two");
    let topic_a = format!("/wire/two/a/{}", unique_id());
    let topic_b = format!("/wire/two/b/{}", unique_id());
    let stop = Arc::new(AtomicBool::new(false));
    let prod_a = spawn_continuous_producer(&manager, &topic_a, stop.clone());
    let prod_b = spawn_continuous_producer(&manager, &topic_b, stop.clone());

    let daemon = disabled_endpoint([215; 32]).await;
    let client = disabled_endpoint([16; 32]).await;
    let client_key = *client.id().as_bytes();
    let clock = RemotedClock::fixed(T_NOW);
    let shared = revocable_shared(client_key, clock.clone(), EXPIRY);
    let authz = Arc::new(PairingAuthorizer::from_shared(shared));
    let plane = Arc::new(
        WirePlane::with_manager("robot-two", manager.clone())
            .with_demand_authorizer(authz.clone())
            .with_revocation_sweep_interval(Duration::from_millis(200)),
    );
    let daemon_addr = loopback_addr(daemon.id(), &daemon.bound_sockets());

    let daemon_ref = &daemon;
    let client_ref = &client;
    let a_ref = &topic_a;
    let b_ref = &topic_b;
    let clock_for_client = clock.clone();

    let daemon_fut = async move {
        let accepted = bounded("accept_one", accept_one(daemon_ref))
            .await
            .expect("accept ok")
            .expect("incoming");
        handle_accepted_with_wire(accepted, authz, None, Some(plane)).await;
    };

    let client_fut = async move {
        let conn = bounded("dial", dial(client_ref, daemon_addr, alpn::WIRE))
            .await
            .expect("dial wire");
        let (mut send, mut recv) = open_control(&conn).await;
        // Demand BOTH topics; the robot opens one uni stream per demand (in order).
        assert_eq!(
            request(
                &mut send,
                &mut recv,
                &WireRequest::Demand {
                    topic: a_ref.clone()
                }
            )
            .await,
            WireResponse::DemandAccepted {
                topic: a_ref.clone()
            }
        );
        let mut ustream_a = bounded("accept_uni a", accept_uni_frame_stream(&conn))
            .await
            .expect("uni a");
        assert_eq!(
            request(
                &mut send,
                &mut recv,
                &WireRequest::Demand {
                    topic: b_ref.clone()
                }
            )
            .await,
            WireResponse::DemandAccepted {
                topic: b_ref.clone()
            }
        );
        let mut ustream_b = bounded("accept_uni b", accept_uni_frame_stream(&conn))
            .await
            .expect("uni b");
        // Correlate each stream to its topic via the preamble (accept order = demand order).
        let pa: StreamPreamble = serde_json::from_slice(
            &bounded("pa", read_frame(&mut ustream_a, DEFAULT_MAX_FRAME_LEN))
                .await
                .expect("pa"),
        )
        .unwrap();
        let pb: StreamPreamble = serde_json::from_slice(
            &bounded("pb", read_frame(&mut ustream_b, DEFAULT_MAX_FRAME_LEN))
                .await
                .expect("pb"),
        )
        .unwrap();
        assert_eq!(&pa.topic, a_ref);
        assert_eq!(&pb.topic, b_ref);
        // Both streams healthy pre-revoke.
        bounded("a frame", read_frame(&mut ustream_a, DEFAULT_MAX_FRAME_LEN))
            .await
            .expect("a healthy pre-revoke");
        bounded("b frame", read_frame(&mut ustream_b, DEFAULT_MAX_FRAME_LEN))
            .await
            .expect("b healthy pre-revoke");

        // WHOLE-SUBJECT revoke (grant expiry): BOTH streams die + the connection closes.
        clock_for_client.set(EXPIRY);
        assert!(
            read_until_reset(&mut ustream_a).await,
            "topic A stream must be evicted (whole-subject revoke)"
        );
        assert!(
            read_until_reset(&mut ustream_b).await,
            "topic B stream must be evicted (whole-subject revoke) — NOT just A"
        );
        drop(conn);
    };

    tokio::join!(daemon_fut, client_fut);
    stop.store(true, Ordering::Relaxed);
    let _ = prod_a.join();
    let _ = prod_b.join();
}
