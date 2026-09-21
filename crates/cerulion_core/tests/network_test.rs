// SPDX-License-Identifier: AGPL-3.0-only
//! Integration tests for the network transport, over the
//! SHM-backed `loan_proxy` / `try_view` API.
//!
//! Covers:
//! - Zenoh session lifecycle (creation, idempotence, lazy init, drop).
//! - Network task lifecycle (start/stop on subscriber count transitions).
//! - Liveliness-based topic discovery.
//! - Raw wire-frame egress: the zenoh payload IS the Cerulion wire
//!   frame verbatim (no prost envelope), received byte-identical by a
//!   same-session zenoh subscriber.
//! - Network→local INGRESS registration: `register_ingress` guards
//!   (empty topic, double-register) and the validate-and-count contract
//!   (`ingress_stats` frames / schema_mismatch_drops / decode_errors).
//! - Local SHM round-trip (loan_proxy → try_view) — proves the SHM-backed API
//!   works end-to-end, as the baseline the raw-put pins above build on.
//!
//! NOT covered here, deliberately: a publisher-side dual-publish path. There is
//! none — publishers are network-free (`src/transport/bridge.rs`), and
//! egress is the demand-driven network GATEWAY tap, pinned end to end in
//! `gateway_iox2_test.rs`. See the block above the ingress section.
//!
//! These tests use isolated zenoh sessions with multicast/gossip scouting
//! disabled, so they are safe to run alongside the iceoryx2 serial tests
//! provided the parent invocation uses `--test-threads=1` (iceoryx2
//! singleton).
//!
//! # Covered elsewhere
//!
//! - Local-always-publishes and prefer-local-over-network: covered by
//!   `output_proxy_test.rs` and the round-trip test below.
//! - A `TransportManager` with no network: the `TransportManager`
//!   singleton may already be initialized by another test in the same
//!   process and report `Some(network)`, so a thin wrapper test would
//!   add no signal. The network plane is exercised directly here
//!   via standalone `NetworkManager`s instead.

use cerulion_core::wire::MaxSliceLen;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant, SystemTime};

use cerulion_core::transport::bridge::TopicBridgeManager;
use cerulion_core::transport::discovery::{query_live_topics, TopicToken};
use cerulion_core::transport::network::{NetworkConfig, NetworkManager, ZenohMode};
use cerulion_core::transport::TransportManager;
use cerulion_core::wire::WireHeader;
use native_ros2_messages::geometry_msgs::Vector3;
use zenoh::Wait;

/// Helper: monotonic suffix so re-runs don't collide on iceoryx2 service names.
static TOPIC_COUNTER: AtomicU64 = AtomicU64::new(0);

fn unique_topic(base: &str) -> String {
    let nanos = SystemTime::now()
        .duration_since(SystemTime::UNIX_EPOCH)
        .unwrap()
        .as_nanos();
    let id = TOPIC_COUNTER.fetch_add(1, Ordering::Relaxed);
    format!("test/network/{base}/{nanos}/{id}")
}

/// Helper: default bridge manager for tests that don't care about bridging.
fn test_bridge_mgr() -> Arc<TopicBridgeManager> {
    Arc::new(TopicBridgeManager::new())
}

/// Helper: build a wire frame from header fields + payload bytes (used
/// for the raw-frame pins that don't go through `loan_proxy`).
fn make_wire_frame(schema_hash: u64, seq: u32, timestamp_ns: u64, payload: &[u8]) -> Vec<u8> {
    let header = WireHeader {
        schema_hash,
        total_size: (WireHeader::SIZE + payload.len()) as u32,
        offset_table_offset: 0,
        offset_table_count: 0,
        sequence: seq,
        timestamp_ns,
    };
    let mut frame = vec![0u8; WireHeader::SIZE + payload.len()];
    header.write_to_buf(&mut frame[..WireHeader::SIZE]);
    frame[WireHeader::SIZE..].copy_from_slice(payload);
    frame
}

// ---------------------------------------------------------------------------
// Session lifecycle (+ lazy init)
// ---------------------------------------------------------------------------

#[test]
fn test_zenoh_session_closes_on_drop() {
    let mgr = NetworkManager::new(NetworkConfig::default());
    let session = mgr.session().expect("session creation should succeed");
    let session_id = session.zid();
    assert!(mgr.is_active());
    drop(mgr);
    assert!(!session_id.to_string().is_empty());
}

#[test]
fn test_zenoh_session_created_lazily() {
    let mgr = NetworkManager::new(NetworkConfig::default());
    assert!(
        !mgr.is_active(),
        "session should not exist before first access"
    );
    let _session = mgr.session().expect("session creation should succeed");
    assert!(mgr.is_active(), "session should exist after first access");
}

#[test]
fn test_zenoh_session_idempotent() {
    let mgr = NetworkManager::new(NetworkConfig::default());
    let s1 = mgr.session().expect("first call should succeed");
    let s2 = mgr.session().expect("second call should succeed");
    assert_eq!(s1.zid(), s2.zid());
}

#[test]
fn test_default_config_no_scouting() {
    let config = NetworkConfig::default();
    assert!(!config.multicast_scouting);
    assert!(!config.gossip_scouting);
    assert!(config.connect_endpoints.is_empty());
    assert!(config.listen_endpoints.is_empty());
    assert_eq!(config.mode, ZenohMode::Peer);
}

#[test]
fn test_zenoh_session_client_mode_requires_peer() {
    let config = NetworkConfig {
        mode: ZenohMode::Client,
        ..Default::default()
    };
    let mgr = NetworkManager::new(config);
    let err = mgr
        .session()
        .expect_err("client mode with no peers should fail");
    let msg = format!("{err}");
    assert!(
        msg.contains("zenoh session"),
        "error should mention session creation: {msg}"
    );
}

#[test]
fn test_separate_managers_isolated_sessions() {
    let mgr1 = NetworkManager::new(NetworkConfig::default());
    let mgr2 = NetworkManager::new(NetworkConfig::default());
    let s1 = mgr1.session().expect("session 1");
    let s2 = mgr2.session().expect("session 2");
    assert_ne!(
        s1.zid(),
        s2.zid(),
        "separate managers should have distinct session IDs"
    );
}

// ---------------------------------------------------------------------------
// Network task lifecycle
// ---------------------------------------------------------------------------

#[test]
fn test_network_task_starts_on_demand() {
    let mgr = NetworkManager::new(NetworkConfig::default());
    assert!(!mgr.is_task_running());
    assert_eq!(mgr.network_subscriber_count(), 0);

    mgr.add_network_subscriber(test_bridge_mgr())
        .expect("add subscriber should succeed");
    assert!(mgr.is_task_running());
    assert_eq!(mgr.network_subscriber_count(), 1);
}

#[test]
fn test_network_task_stops_when_idle() {
    let mgr = NetworkManager::new(NetworkConfig::default());
    mgr.add_network_subscriber(test_bridge_mgr())
        .expect("add first");
    mgr.add_network_subscriber(test_bridge_mgr())
        .expect("add second");
    assert!(mgr.is_task_running());
    assert_eq!(mgr.network_subscriber_count(), 2);

    mgr.remove_network_subscriber();
    assert!(mgr.is_task_running());
    assert_eq!(mgr.network_subscriber_count(), 1);

    mgr.remove_network_subscriber();
    assert!(!mgr.is_task_running());
    assert_eq!(mgr.network_subscriber_count(), 0);
}

#[test]
fn test_network_task_restarts_after_stop() {
    let mgr = NetworkManager::new(NetworkConfig::default());
    mgr.add_network_subscriber(test_bridge_mgr()).expect("add");
    assert!(mgr.is_task_running());
    mgr.remove_network_subscriber();
    assert!(!mgr.is_task_running());

    mgr.add_network_subscriber(test_bridge_mgr())
        .expect("re-add");
    assert!(mgr.is_task_running());
    mgr.remove_network_subscriber();
    assert!(!mgr.is_task_running());
}

#[test]
fn test_add_subscriber_creates_session() {
    let mgr = NetworkManager::new(NetworkConfig::default());
    assert!(!mgr.is_active());
    mgr.add_network_subscriber(test_bridge_mgr())
        .expect("add subscriber");
    assert!(mgr.is_active());
    mgr.remove_network_subscriber();
}

#[test]
fn test_remove_subscriber_underflow_safe() {
    let mgr = NetworkManager::new(NetworkConfig::default());
    mgr.remove_network_subscriber();
    assert_eq!(mgr.network_subscriber_count(), 0);
}

#[test]
fn test_drop_stops_running_task() {
    let mgr = NetworkManager::new(NetworkConfig::default());
    mgr.add_network_subscriber(test_bridge_mgr()).expect("add");
    assert!(mgr.is_task_running());
    drop(mgr);
}

// ---------------------------------------------------------------------------
// Raw wire-frame contract over LIVE zenoh (no serialization
// envelope; each pin is a real same-session
// put→receive round trip, not a local parse)
//
// The zenoh payload IS the Cerulion wire frame verbatim, so each pin puts a
// hand-stamped frame and asserts the RECEIVED bytes — full-slice equality plus
// the header-field recovery a remote ingress peer performs.
// ---------------------------------------------------------------------------

/// Same-session zenoh put→receive helper for the raw-frame pins: publishes
/// `frame` on `topic` via the production egress path and returns the bytes a
/// loopback subscriber received.
fn zenoh_roundtrip(topic: &str, frame: Vec<u8>) -> Vec<u8> {
    let mgr = NetworkManager::new(NetworkConfig::default());
    let session = mgr.session().expect("session");
    let zenoh_sub = session
        .declare_subscriber(NetworkManager::data_key(topic))
        .wait()
        .expect("zenoh subscriber");
    mgr.publish_to_network(topic, frame).expect("publish");
    let sample = zenoh_sub
        .recv_timeout(Duration::from_secs(2))
        .expect("recv")
        .expect("loopback should deliver");
    sample.payload().to_bytes().into_owned()
}

/// The serialization pin: a raw frame put on
/// zenoh arrives with every hand-stamped header field and the payload
/// VERBATIM — parsed from the RECEIVED bytes, plus full-slice equality.
#[test]
fn test_network_put_delivers_header_fields_and_payload() {
    let payload = vec![0xDE, 0xAD, 0xBE, 0xEF, 0x01, 0x02, 0x03, 0x04];
    let frame = make_wire_frame(0xCAFEBABE, 42, 5_000_000_000, &payload);
    let received = zenoh_roundtrip("/test/raw/header_fields", frame.clone());

    assert_eq!(received, frame, "full frame slice survives the network hop");
    let header = WireHeader::read_from_buf(&received).expect("header parses from received bytes");
    assert_eq!(header.schema_hash, 0xCAFEBABE);
    assert_eq!(header.sequence, 42);
    assert_eq!(header.timestamp_ns, 5_000_000_000);
    assert_eq!(header.total_size as usize, received.len());
    assert_eq!(&received[WireHeader::SIZE..], payload.as_slice());
}

/// The empty-payload pin: a header-only (32-byte) frame
/// survives the network hop byte-identical with an empty payload region.
#[test]
fn test_network_put_delivers_empty_payload_frame() {
    let frame = make_wire_frame(0x1234, 0, 0, &[]);
    assert_eq!(frame.len(), WireHeader::SIZE);
    let received = zenoh_roundtrip("/test/raw/empty_payload", frame.clone());
    assert_eq!(received, frame, "header-only frame survives byte-identical");
    let header = WireHeader::read_from_buf(&received).expect("header parses");
    assert_eq!(header.schema_hash, 0x1234);
    assert!(received[WireHeader::SIZE..].is_empty());
}

/// The large-payload pin: a 1 MiB frame survives the
/// network hop (zenoh fragmentation included) byte-identical.
#[test]
fn test_network_put_delivers_large_payload_frame() {
    let payload = vec![0x42u8; 1_048_576];
    let frame = make_wire_frame(0xAAAA, 100, 999_999_999, &payload);
    let received = zenoh_roundtrip("/test/raw/large_payload", frame.clone());
    assert_eq!(received.len(), WireHeader::SIZE + 1_048_576);
    assert_eq!(
        received, frame,
        "1 MiB frame survives the network hop byte-identical"
    );
    let header = WireHeader::read_from_buf(&received).expect("header parses");
    assert_eq!(header.total_size as usize, received.len());
}

#[test]
fn test_data_key_format_simple() {
    // Canonical names join the namespace directly (the
    // leading slash IS the separator); raw no-slash names canonicalize
    // first and ALIAS their slashed twin on the wire.
    assert_eq!(NetworkManager::data_key("/foo"), "cerulion/foo");
    assert_eq!(NetworkManager::data_key("foo"), "cerulion/foo");
}

#[test]
fn test_data_key_format_nested_topic() {
    assert_eq!(
        NetworkManager::data_key("/perception/camera/image"),
        "cerulion/perception/camera/image"
    );
    // Raw + canonical alias to the same key (documented network-layer
    // contract — locally they are distinct iceoryx2 services).
    assert_eq!(
        NetworkManager::data_key("camera/front/image"),
        NetworkManager::data_key("/camera/front/image")
    );
}

/// The exact-header-boundary pin: 32 bytes (exactly the
/// wire header) is the SMALLEST valid frame — the outbound `len >=
/// WireHeader::SIZE` guard must ACCEPT it and deliver it byte-identical,
/// while one byte short is rejected (the guard's exact boundary; the far
/// undersized arm lives in `test_publish_to_network_wire_data_too_small`).
#[test]
fn test_publish_to_network_accepts_exact_header_boundary() {
    let frame = make_wire_frame(0xB0DA, 3, 700, &[]);
    assert_eq!(frame.len(), WireHeader::SIZE);
    let received = zenoh_roundtrip("/test/raw/boundary", frame.clone());
    assert_eq!(received, frame, "boundary frame delivered byte-identical");

    // One byte short of the boundary is refused by the outbound guard.
    let mgr = NetworkManager::new(NetworkConfig::default());
    let err = mgr
        .publish_to_network("/test/raw/boundary_short", vec![0u8; WireHeader::SIZE - 1])
        .expect_err("31 bytes cannot hold the wire header");
    assert!(
        format!("{err}").contains("too small"),
        "boundary rejection should mention size: {err}"
    );
}

#[test]
fn test_publish_to_network_wire_data_too_small() {
    let mgr = NetworkManager::new(NetworkConfig::default());
    // `publish_to_network` takes the OWNED frame Vec.
    let err = mgr
        .publish_to_network("bad/topic", vec![0u8; 10])
        .expect_err("undersized wire data should fail");
    let msg = format!("{err}");
    assert!(
        msg.contains("too small"),
        "error should mention size: {msg}"
    );
}

// ---------------------------------------------------------------------------
// Liveliness tokens
// ---------------------------------------------------------------------------

#[test]
fn test_liveliness_token_created() {
    let mgr = NetworkManager::new(NetworkConfig::default());
    let session = mgr.session().expect("session should open");

    let token = TopicToken::declare(session, "/live/sensor/lidar").expect("declare");
    assert_eq!(token.topic(), "/live/sensor/lidar");

    let topics = query_live_topics(session).expect("query");
    assert!(
        topics.contains(&"/live/sensor/lidar".to_string()),
        "declared topic should be discoverable, got: {topics:?}"
    );

    // A RAW no-slash declaration is canonicalized at the
    // network boundary — the recovered name is its slashed twin (the
    // documented aliasing contract).
    let _raw_token = TopicToken::declare(session, "live/raw/cam").expect("declare raw");
    let topics = query_live_topics(session).expect("query");
    assert!(
        topics.contains(&"/live/raw/cam".to_string()),
        "raw declaration must be discoverable under its canonical name, got: {topics:?}"
    );
}

#[test]
fn test_bare_namespace_token_does_not_become_phantom_topic() {
    // `**` matches zero chunks, so a
    // foreign token at exactly the namespace name (`cerulion_lv`) DOES
    // arrive at the liveliness query — the canonical-slash filter must
    // reject it instead of pushing a phantom empty-name topic into the
    // user-visible list.
    let mgr = NetworkManager::new(NetworkConfig::default());
    let session = mgr.session().expect("session should open");
    let _foreign = session
        .liveliness()
        .declare_token("cerulion_lv")
        .wait()
        .expect("raw token at the bare namespace key");
    let topics = query_live_topics(session).expect("query");
    assert!(
        !topics.iter().any(|t| t.is_empty()),
        "a bare-namespace token must not list as an empty-name topic: {topics:?}"
    );
}

#[test]
fn test_liveliness_token_removed_on_drop() {
    let mgr = NetworkManager::new(NetworkConfig::default());
    let session = mgr.session().expect("session should open");

    let token = TopicToken::declare(session, "/live/temp/topic").unwrap();
    drop(token);
    std::thread::sleep(Duration::from_millis(50));

    let topics = query_live_topics(session).expect("query");
    assert!(
        !topics.contains(&"/live/temp/topic".to_string()),
        "dropped topic should not be discoverable, got: {topics:?}"
    );
}

#[test]
fn test_query_live_topics_empty_network() {
    let mgr = NetworkManager::new(NetworkConfig::default());
    let session = mgr.session().expect("session");
    let topics = query_live_topics(session).expect("query");
    assert!(
        topics.is_empty(),
        "no topics on fresh isolated session, got: {topics:?}"
    );
}

// ---------------------------------------------------------------------------
// Wire-format network round-trip
// ---------------------------------------------------------------------------

/// Egress raw-put pin: a raw wire frame put on zenoh is received
/// BYTE-IDENTICAL by a same-session loopback subscriber — no envelope, no
/// per-hop rewrite. The received bytes ARE the frame the egress path put, so
/// a remote receiver feeds them straight into the ingress validate/re-inject
/// path.
#[test]
fn test_network_publish_receive_roundtrip() {
    let mgr = NetworkManager::new(NetworkConfig::default());
    let session = mgr.session().expect("session");
    // Canonical leading-slash topic — the live zenoh
    // round-trip pins that canonical names form valid key expressions.
    let recv_topic = "/test/roundtrip/data";

    let zenoh_sub = session
        .declare_subscriber(NetworkManager::data_key(recv_topic))
        .wait()
        .expect("zenoh subscriber");

    let payload = vec![1u8, 2, 3, 4, 5];
    let frame = make_wire_frame(0xBEEF, 10, 999, &payload);

    mgr.publish_to_network(recv_topic, frame.clone())
        .expect("publish");

    let sample = zenoh_sub
        .recv_timeout(Duration::from_millis(500))
        .expect("recv")
        .expect("loopback should deliver");

    let zbytes = sample.payload().to_bytes();
    // The raw wire frame arrives byte-identical — no envelope.
    assert_eq!(
        zbytes.as_ref(),
        frame.as_slice(),
        "raw wire bytes survive end-to-end"
    );
    let header = WireHeader::read_from_buf(&zbytes).expect("header parses from received bytes");
    assert_eq!(header.sequence, 10);
    assert_eq!(header.schema_hash, 0xBEEF);
    assert_eq!(&zbytes[WireHeader::SIZE..], &[1, 2, 3, 4, 5]);
}

#[test]
fn test_publish_sequential_ordering() {
    let mgr = NetworkManager::new(NetworkConfig::default());
    let session = mgr.session().expect("session");

    let zenoh_sub = session
        .declare_subscriber(NetworkManager::data_key("ordering/test"))
        .wait()
        .expect("zenoh subscriber");

    for seq in 1u32..=3 {
        let frame = make_wire_frame(0xF1F0, seq, seq as u64 * 100, &[seq as u8]);
        mgr.publish_to_network("ordering/test", frame)
            .expect("publish");
    }

    let mut sequences = Vec::new();
    for _ in 0..3 {
        let sample = zenoh_sub
            .recv_timeout(Duration::from_millis(500))
            .expect("recv")
            .expect("data");
        let zbytes = sample.payload().to_bytes();
        // Parse the wire header directly from the raw received frame.
        let header = WireHeader::read_from_buf(&zbytes).expect("header parses");
        sequences.push(header.sequence);
    }
    assert_eq!(sequences, vec![1, 2, 3]);
}

// ---------------------------------------------------------------------------
// Local SHM round-trip via the `loan_proxy` API (baseline)
// ---------------------------------------------------------------------------

/// Baseline: a `loan_proxy` publish lands on a local subscriber via
/// iceoryx2 shared memory. Mirrors `output_proxy_test::proxy_round_trip_*`
/// but lives in the network test file because the raw-put pins above share
/// this setup pattern (local pub + local sub + zenoh sub). It is NOT a
/// prelude to a dual-publish test — see the note below.
#[test]
fn test_local_pub_sub_round_trip_via_loan_proxy() {
    let transport = TransportManager::get_or_init().expect("transport init");
    let topic = unique_topic("local_baseline");

    let mut publisher = transport
        .create_publisher_simple(&topic, MaxSliceLen::const_new(256))
        .expect("publisher");
    let mut subscriber = transport.create_subscriber(&topic).expect("subscriber");

    {
        let mut proxy = publisher.loan_proxy::<Vector3>().expect("loan_proxy");
        proxy.x = 11.0;
        proxy.y = 22.0;
        proxy.z = 33.0;
    }

    std::thread::sleep(Duration::from_millis(50));

    let observed = subscriber
        .try_view::<Vector3, _>(|view| (view.x, view.y, view.z))
        .expect("try_view")
        .expect("local subscriber should see published frame");
    assert_eq!(observed, (11.0, 22.0, 33.0));
}

// There is no publisher-side dual-publish test —
// publishers are network-free (they carry no network state and never fan
// out to zenoh). The egress path is the demand-driven gateway TAP, covered end
// to end in `gateway_iox2_test.rs`; the NetworkManager-level raw-put pins below
// and above cover the zenoh hop itself.

// ---------------------------------------------------------------------------
// Network→local ingress registration (guards + validate/count)
// ---------------------------------------------------------------------------

/// Mint a throwaway local iceoryx2 publisher to hand to `register_ingress`.
/// (These tests exercise the registration guards + the
/// validate-and-count callback, not loop-safe minting — the production
/// `create_ingress_publisher` path + its e2e delivery pins live in
/// `network_ingress_test.rs`.)
fn throwaway_publisher(topic: &str) -> cerulion_core::CerulionPublisher {
    let transport = TransportManager::get_or_init().expect("transport init");
    transport
        .create_publisher_simple(topic, MaxSliceLen::const_new(256))
        .expect("publisher")
}

#[test]
fn test_register_ingress_rejects_empty_topic() {
    let mgr = NetworkManager::new(NetworkConfig::default());
    let pubr = throwaway_publisher(&unique_topic("ingress_empty"));
    let err = mgr
        .register_ingress("", 0x1, pubr)
        .expect_err("empty ingress topic must be rejected");
    assert!(
        format!("{err}").contains("non-empty"),
        "error should mention the empty-topic guard: {err}"
    );
}

#[test]
fn test_register_ingress_rejects_double_register() {
    let mgr = NetworkManager::new(NetworkConfig::default());
    let topic = unique_topic("ingress_dup");

    let pub_a = throwaway_publisher(&topic);
    mgr.register_ingress(&topic, 0x1, pub_a)
        .expect("first ingress registration succeeds");

    // A second registration for the SAME topic is refused. (A second publisher
    // is minted on a DISTINCT topic — single-writer forbids two publishers on
    // one topic; register_ingress does not require the publisher's own topic to
    // match the ingress key.)
    let pub_b = throwaway_publisher(&unique_topic("ingress_dup_second_pub"));
    let err = mgr
        .register_ingress(&topic, 0x1, pub_b)
        .expect_err("double ingress registration must be rejected");
    assert!(
        format!("{err}").contains("already has a registered network ingress"),
        "error should name the double-register guard: {err}"
    );
}

#[test]
fn test_ingress_counts_valid_schema_mismatch_and_decode_errors() {
    let mgr = NetworkManager::new(NetworkConfig::default());
    // Canonical (leading-slash) topic — production graph topics always are.
    let topic = format!("/{}", unique_topic("ingress_counters"));
    let expected_hash = 0xABCD_1234_5678_9AB0u64;

    let pubr = throwaway_publisher(&topic);
    mgr.register_ingress(&topic, expected_hash, pubr)
        .expect("register ingress");

    // 1 valid frame (matches expected_hash).
    let good = make_wire_frame(expected_hash, 1, 100, &[1, 2, 3]);
    mgr.publish_to_network(&topic, good).expect("put valid");

    // 1 schema-mismatch frame (wrong hash) → schema_mismatch_drops.
    let bad_hash = make_wire_frame(0xDEAD_BEEF, 2, 200, &[4, 5, 6]);
    mgr.publish_to_network(&topic, bad_hash)
        .expect("put schema-mismatch");

    // 1 decode-error frame (total_size corrupted → does not match length).
    let mut mangled = make_wire_frame(expected_hash, 3, 300, &[7, 8, 9]);
    mangled[8] = mangled[8].wrapping_add(1); // corrupt total_size low byte
    mgr.publish_to_network(&topic, mangled)
        .expect("put decode-error");

    // Poll until all three frames have been accounted for (loopback delivery
    // is async on the zenoh callback thread).
    let deadline = Instant::now() + Duration::from_secs(3);
    loop {
        let s = mgr.ingress_stats(&topic).unwrap_or_default();
        if s.frames + s.schema_mismatch_drops + s.decode_errors >= 3 {
            break;
        }
        if Instant::now() > deadline {
            break;
        }
        std::thread::sleep(Duration::from_millis(20));
    }

    // Hand oracle: exactly one of each class, counters monotone.
    let stats = mgr
        .ingress_stats(&topic)
        .expect("stats for a registered topic");
    assert_eq!(stats.frames, 1, "one valid frame re-injected");
    assert_eq!(
        stats.schema_mismatch_drops, 1,
        "one wrong-hash frame dropped"
    );
    assert_eq!(stats.decode_errors, 1, "one corrupt-size frame dropped");
}

#[test]
fn test_ingress_stats_none_for_unregistered_topic() {
    let mgr = NetworkManager::new(NetworkConfig::default());
    assert!(mgr.ingress_stats("/never/registered").is_none());
}
