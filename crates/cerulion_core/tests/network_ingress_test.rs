// SPDX-License-Identifier: AGPL-3.0-only
//! End-to-end network→local INGRESS re-injection.
//!
//! A raw Cerulion wire frame put on zenoh (same-session loopback) is validated
//! by the topic's `register_ingress` callback and re-injected VERBATIM into the
//! topic's LOCAL iceoryx2 publisher, where a local subscriber reads it. The
//! preservation pins are hand oracles (the KNOWN header fields + payload we
//! stamped), never two-run self-compares.
//!
//! Also pins the loop-safety of `create_ingress_publisher`:
//! - it SUPPRESSES the egress bridge (never appears in the bridge map — the
//!   structural loop exclusion), while a NORMAL publisher registers one;
//! - it REFUSES a topic this process already bridges OUT (ingress+egress loop);
//! - it is REJECTED on a topic already owned by an in-graph single-writer
//!   producer (the single publisher slot is taken).
//!
//! This file also pins the `TransportManager` production wiring:
//! `register_ingress_topic` (no-network loud Err naming the
//! `network:` fix; lazy session; the wired-not-just-registered same-session
//! e2e), `start_network_bridge_watch` (latched-idempotent — the subscriber
//! count never stacks, register shares the latch), and the teardown pin
//! (drop the manager → no hang, ingress services released).
//!
//! Also covered: full-frame-slice byte compares, multi-topic
//! concurrent ingress isolation, and the REAL `publish_raw`-Err arm —
//! counted `reinject_failure_drops` + warn-once latch + recovery info
//! (via the fire-once `fault_inject_publish_raw_after` seam).
//!
//! Real iceoryx2 via PER-TEST SHM roots (`init_for_test` +
//! `iceoryx_test_config`) + isolated scouting-off zenoh sessions, so the whole
//! file is parallel-safe (no `#[serial]`, no `--test-threads=1`).

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant, SystemTime};

use cerulion_core::transport::network::{NetworkConfig, NetworkManager};
use cerulion_core::transport::{
    PublisherProvisioning, TopicServiceConfig, TransportConfig, TransportManager,
};
use cerulion_core::wire::{MaxSliceLen, WireHeader};
use cerulion_core::CerulionSubscriber;

static COUNTER: AtomicU64 = AtomicU64::new(0);

/// Unique suffix so per-test iceoryx2 service names + node names never collide.
fn unique_id() -> String {
    let nanos = SystemTime::now()
        .duration_since(SystemTime::UNIX_EPOCH)
        .unwrap()
        .as_nanos();
    let id = COUNTER.fetch_add(1, Ordering::Relaxed);
    format!("{nanos}_{id}")
}

/// A fresh per-test `TransportManager` (isolated SHM root, network=None) plus a
/// unique canonical topic.
fn setup(base: &str) -> (Arc<TransportManager>, String) {
    let ix = cerulion_core::testing::iceoryx_test_config();
    let id = unique_id();
    let transport = TransportManager::init_for_test(
        TransportConfig {
            node_name: format!("ingress_{base}_{id}"),
            ..Default::default()
        },
        ix,
    )
    .expect("init_for_test");
    (transport, format!("/ingress/{base}/{id}"))
}

/// A fresh per-test `TransportManager` WITH a (lazy) network manager, so a
/// normal publisher registers an egress bridge flag — the control for the
/// loop-exclusion pin.
fn setup_with_network(base: &str) -> (Arc<TransportManager>, String) {
    let ix = cerulion_core::testing::iceoryx_test_config();
    let id = unique_id();
    let transport = TransportManager::init_for_test(
        TransportConfig {
            node_name: format!("ingress_net_{base}_{id}"),
            network: Some(NetworkConfig::default()),
            ..Default::default()
        },
        ix,
    )
    .expect("init_for_test with network");
    (transport, format!("/ingress/{base}/{id}"))
}

/// Hand-build a raw wire frame (32-byte little-endian header + payload).
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

/// Rebuild the FULL received wire frame from a `ReceivedMessage`'s two
/// surfaces: `header()` is an owned parse of ALL 32
/// header bytes (`read_from_buf`/`write_to_buf` are total inverses — the six
/// fields cover every header byte), and `payload()` is the zero-copy slice of
/// EVERYTHING after the header (fixed section + offset table + variable
/// data). Their concatenation therefore IS the full received frame slice —
/// comparable byte-for-byte against the sent frame, offset-table bytes
/// included.
fn rebuild_frame(header: &WireHeader, payload: &[u8]) -> Vec<u8> {
    let mut frame = vec![0u8; WireHeader::SIZE + payload.len()];
    header.write_to_buf(&mut frame[..WireHeader::SIZE]);
    frame[WireHeader::SIZE..].copy_from_slice(payload);
    frame
}

/// Poll the local subscriber until ONE frame is delivered (or timeout).
/// Returns the received (header, payload).
fn poll_one(sub: &CerulionSubscriber, timeout: Duration) -> Option<(WireHeader, Vec<u8>)> {
    let deadline = Instant::now() + timeout;
    loop {
        let mut got: Option<(WireHeader, Vec<u8>)> = None;
        sub.try_receive(|msg| {
            // `header()` returns `&WireHeader` (Copy) — deref-copy it out.
            got = Some((*msg.header(), msg.payload().to_vec()));
        })
        .expect("try_receive");
        if got.is_some() {
            return got;
        }
        if Instant::now() > deadline {
            return None;
        }
        std::thread::sleep(Duration::from_millis(20));
    }
}

/// The headline preservation pin: a raw frame put on zenoh is re-injected
/// VERBATIM into local iceoryx2 — the local subscriber reads back the EXACT
/// header (schema_hash, sequence, timestamp_ns) + payload we stamped
/// (hand oracle, not a self-compare — Principle #7).
#[test]
fn test_ingress_reinjects_frame_byte_identical() {
    let (transport, topic) = setup("byte_identical");
    let expected_hash = 0xABCD_EF01_2345_6789u64;

    // Local subscriber connected BEFORE the ingress publisher so it receives
    // the re-injected send.
    let subscriber = transport.create_subscriber(&topic).expect("subscriber");
    let ingress_pub = transport
        .create_ingress_publisher(&topic, MaxSliceLen::const_new(256))
        .expect("ingress publisher");

    let mgr = NetworkManager::new(NetworkConfig::default());
    mgr.register_ingress(&topic, expected_hash, ingress_pub)
        .expect("register ingress");

    // Hand oracle: KNOWN header fields + payload.
    let seq = 7u32;
    let ts = 123_456_789u64;
    let payload = vec![0xDE, 0xAD, 0xBE, 0xEF];
    let frame = make_wire_frame(expected_hash, seq, ts, &payload);
    mgr.publish_to_network(&topic, frame.clone())
        .expect("egress put");

    let (header, got) =
        poll_one(&subscriber, Duration::from_secs(3)).expect("re-injected frame delivered locally");

    // Field-level diagnostics first (readable failure messages), then the
    // full-slice pin: the entire received frame must equal the
    // entire sent frame — offset-table bytes included.
    assert_eq!(header.schema_hash, expected_hash);
    assert_eq!(header.sequence, seq, "wire sequence preserved verbatim");
    assert_eq!(
        header.timestamp_ns, ts,
        "wire timestamp_ns preserved verbatim"
    );
    assert_eq!(header.total_size as usize, frame.len());
    assert_eq!(got, payload, "payload preserved verbatim");
    assert_eq!(
        rebuild_frame(&header, &got),
        frame,
        "FULL frame slice must be byte-identical (offset-table bytes included)"
    );

    assert_eq!(
        mgr.ingress_stats(&topic).expect("stats").frames,
        1,
        "exactly one frame re-injected"
    );
}

/// A schema-mismatch frame is dropped at ingress — NEVER re-injected — and
/// counted. The local subscriber sees nothing.
#[test]
fn test_ingress_schema_mismatch_not_delivered() {
    let (transport, topic) = setup("schema_mismatch");
    let expected_hash = 0x1111_2222_3333_4444u64;

    let subscriber = transport.create_subscriber(&topic).expect("subscriber");
    let ingress_pub = transport
        .create_ingress_publisher(&topic, MaxSliceLen::const_new(256))
        .expect("ingress publisher");

    let mgr = NetworkManager::new(NetworkConfig::default());
    mgr.register_ingress(&topic, expected_hash, ingress_pub)
        .expect("register ingress");

    // Wrong-hash frame → dropped, never re-injected.
    let bad = make_wire_frame(0x9999_9999_9999_9999, 5, 500, &[1, 2, 3]);
    mgr.publish_to_network(&topic, bad).expect("egress put");

    // Wait for the drop to be accounted for.
    let deadline = Instant::now() + Duration::from_secs(3);
    while Instant::now() < deadline {
        if mgr
            .ingress_stats(&topic)
            .unwrap_or_default()
            .schema_mismatch_drops
            >= 1
        {
            break;
        }
        std::thread::sleep(Duration::from_millis(20));
    }

    // Nothing reached the local subscriber.
    assert!(
        poll_one(&subscriber, Duration::from_millis(200)).is_none(),
        "a schema-mismatch frame must NOT be delivered to the local subscriber"
    );
    let stats = mgr.ingress_stats(&topic).expect("stats");
    assert_eq!(stats.schema_mismatch_drops, 1);
    assert_eq!(stats.frames, 0);
}

/// Structural loop pin (control + treatment): a normal publisher is
/// NETWORK-FREE and registers NO bridge flag; only the gateway registers a flag
/// for an ANNOUNCED (egress) topic ([`register_topic`]). An INGRESS publisher
/// never registers a flag either — it never appears in the bridge map, so it can
/// never be tapped back out.
#[test]
fn test_ingress_publisher_suppresses_egress_bridge() {
    let (transport, base_topic) = setup_with_network("loop_exclusion");
    let normal_topic = format!("{base_topic}/normal");
    let announced_topic = format!("{base_topic}/announced");
    let ingress_topic = format!("{base_topic}/inject");

    // A normal publisher is network-free — it registers NO flag.
    let _normal = transport
        .create_publisher_simple(&normal_topic, MaxSliceLen::const_new(256))
        .expect("normal publisher");
    assert!(
        !transport
            .bridge_manager()
            .is_registered(&normal_topic)
            .unwrap(),
        "a publisher is network-free and registers no egress bridge flag"
    );

    // Control: the GATEWAY registering an announce topic DOES mark it.
    transport
        .bridge_manager()
        .register_topic(&announced_topic)
        .expect("gateway announces the topic");
    assert!(
        transport
            .bridge_manager()
            .is_registered(&announced_topic)
            .unwrap(),
        "an announced (egress) topic is registered by the gateway"
    );

    // Treatment: the ingress publisher does NOT register a flag (loop exclusion).
    let _ingress = transport
        .create_ingress_publisher(&ingress_topic, MaxSliceLen::const_new(256))
        .expect("ingress publisher");
    assert!(
        !transport
            .bridge_manager()
            .is_registered(&ingress_topic)
            .unwrap(),
        "an ingress publisher must NOT register an egress bridge flag — it never taps out"
    );
}

/// Ingress + egress on the SAME topic is refused loudly (the loop):
/// once the gateway has ANNOUNCED a topic (registered its flag), an ingress
/// injector for that topic is rejected before any iceoryx2 port is created.
#[test]
fn test_ingress_refused_when_topic_already_bridged_out() {
    let (transport, topic) = setup_with_network("egress_conflict");

    // The gateway announces `topic` for egress (registers its bridge flag).
    transport
        .bridge_manager()
        .register_topic(&topic)
        .expect("gateway announces the topic");
    assert!(transport.bridge_manager().is_registered(&topic).unwrap());

    // Ingress on the SAME topic is refused (loop prevention).
    // (`let Err(..) = .. else` because `CerulionPublisher` is not Debug, so
    // `expect_err` cannot format the Ok arm.)
    let Err(err) = transport.create_ingress_publisher(&topic, MaxSliceLen::const_new(256)) else {
        panic!("ingress + egress on one topic must be refused");
    };
    let msg = format!("{err}");
    assert!(
        msg.contains("already announced the topic for egress"),
        "error must name the ingress+egress loop: {msg}"
    );
}

/// Ingress is rejected on a topic already owned by an in-graph single-writer
/// producer — the one publisher slot is taken, so you cannot inject remote
/// data into a topic this graph already produces locally.
#[test]
fn test_ingress_refused_when_live_single_writer_producer_exists() {
    let (transport, topic) = setup("single_writer_conflict");

    // A SingleWriter producer creates the service with max_publishers = 1.
    let sw_cfg = TopicServiceConfig::for_topology(
        transport.default_topic_config(),
        16, // max_consumer_depth
        1,  // in_graph_subscribers
        PublisherProvisioning::SingleWriter,
        0, // history_size
        0, // extra_event_listeners
    );
    let _producer = transport
        .create_publisher_with_topic_config(&topic, MaxSliceLen::const_new(256), 0, sw_cfg)
        .expect("single-writer producer");

    // Ingress on the same topic must fail — the single publisher slot is taken.
    // (`let Err(..) = .. else` because `CerulionPublisher` is not Debug, so
    // `expect_err` cannot format the Ok arm.)
    let Err(err) = transport.create_ingress_publisher(&topic, MaxSliceLen::const_new(256)) else {
        panic!("ingress must be refused when a live producer owns the topic");
    };
    let msg = format!("{err}").to_lowercase();
    assert!(
        msg.contains("publisher") || msg.contains("slot") || msg.contains("single"),
        "error should indicate the single-writer publisher-slot conflict: {msg}"
    );
}

/// Determinism (Principle #7): two IDENTICAL injection sequences produce
/// BYTE-IDENTICAL delivered frames, and both equal a HAND oracle (the injected
/// seq/ts/payload) — not merely a self-compare.
#[test]
fn test_ingress_delivery_is_deterministic() {
    let hash = 0xCAFE_D00D_0000_0001u64;
    let injections: [(u32, u64, Vec<u8>); 3] = [
        (1, 10, vec![1, 2, 3]),
        (2, 20, vec![4, 5, 6]),
        (3, 30, vec![7, 8, 9]),
    ];

    let run = |base: &str| -> Vec<(u32, u64, Vec<u8>)> {
        let (transport, topic) = setup(base);
        let subscriber = transport.create_subscriber(&topic).expect("subscriber");
        let ingress_pub = transport
            .create_ingress_publisher(&topic, MaxSliceLen::const_new(256))
            .expect("ingress publisher");
        let mgr = NetworkManager::new(NetworkConfig::default());
        mgr.register_ingress(&topic, hash, ingress_pub)
            .expect("register ingress");

        for (seq, ts, payload) in &injections {
            mgr.publish_to_network(&topic, make_wire_frame(hash, *seq, *ts, payload))
                .expect("egress put");
        }

        // Collect all delivered frames (FIFO per single publisher).
        let mut got: Vec<(u32, u64, Vec<u8>)> = Vec::new();
        let deadline = Instant::now() + Duration::from_secs(3);
        while got.len() < injections.len() && Instant::now() < deadline {
            subscriber
                .try_receive(|msg| {
                    let h = msg.header();
                    got.push((h.sequence, h.timestamp_ns, msg.payload().to_vec()));
                })
                .expect("try_receive");
            if got.len() < injections.len() {
                std::thread::sleep(Duration::from_millis(10));
            }
        }
        got
    };

    let a = run("determinism_a");
    let b = run("determinism_b");

    // Hand oracle: exactly the injected (seq, ts, payload), in order.
    let oracle: Vec<(u32, u64, Vec<u8>)> = injections
        .iter()
        .map(|(s, t, p)| (*s, *t, p.clone()))
        .collect();
    assert_eq!(
        a, oracle,
        "run A delivers the injected frames verbatim, in order"
    );
    assert_eq!(
        b, oracle,
        "run B delivers the injected frames verbatim, in order"
    );
    assert_eq!(
        a, b,
        "two identical injection sequences deliver byte-identically"
    );
}

// ---------------------------------------------------------------------------
// TransportManager wiring (register_ingress_topic +
// start_network_bridge_watch) — production one-call ingress + egress watch
// ---------------------------------------------------------------------------

/// register_ingress_topic on a manager with NO network transport errs LOUDLY,
/// naming the fix (the `network:` block).
#[test]
fn test_register_ingress_topic_without_network_errs_loudly() {
    let (transport, topic) = setup("no_network_ingress");
    assert!(transport.network().is_none(), "setup() has no network");

    let err = transport
        .register_ingress_topic(&topic, 0x1, MaxSliceLen::const_new(256))
        .expect_err("ingress without a configured network must be refused");
    let msg = format!("{err}");
    assert!(
        msg.contains("`network:` block"),
        "error must name the `network:` block fix: {msg}"
    );
    assert!(
        msg.contains(&topic),
        "error must name the offending topic: {msg}"
    );
}

/// start_network_bridge_watch on a manager with NO network transport errs
/// loudly with the same fix guidance.
#[test]
fn test_start_network_bridge_watch_without_network_errs() {
    let (transport, _topic) = setup("no_network_watch");
    let err = transport
        .start_network_bridge_watch()
        .expect_err("bridge watch without a configured network must be refused");
    let msg = format!("{err}");
    assert!(
        msg.contains("`network:` block"),
        "error must name the `network:` block fix: {msg}"
    );
}

/// The zenoh session is LAZY: configured-but-unused network stays closed
/// (`is_active` false) until the first register_ingress_topic opens it.
#[test]
fn test_session_lazy_until_first_register_ingress_topic() {
    let (transport, topic) = setup_with_network("lazy_session");
    let net = transport.network().expect("network configured");
    assert!(
        !net.is_active(),
        "session must NOT exist before the first ingress registration"
    );
    assert!(!net.is_task_running(), "watch task must not run yet");

    transport
        .register_ingress_topic(&topic, 0xBEEF, MaxSliceLen::const_new(256))
        .expect("register ingress topic");

    assert!(
        net.is_active(),
        "first register_ingress_topic must open the session"
    );
    assert!(
        net.is_task_running(),
        "first register_ingress_topic must start the liveliness watch task"
    );
}

/// The watch start is LATCHED-idempotent: repeated start_network_bridge_watch
/// calls and register_ingress_topic's internal ensure all share ONE
/// network-subscriber ref (graph build may call the hook once per egress
/// topic — the count must not stack).
#[test]
fn test_watch_start_is_latched_idempotent() {
    let (transport, topic) = setup_with_network("watch_latch");
    let net = transport.network().expect("network configured");
    assert!(!net.is_task_running());

    transport
        .start_network_bridge_watch()
        .expect("first watch start");
    assert!(net.is_task_running(), "watch task runs after first start");
    assert_eq!(net.network_subscriber_count(), 1);

    // Repeat calls latch — no count stacking.
    transport
        .start_network_bridge_watch()
        .expect("second watch start (latched no-op)");
    transport
        .start_network_bridge_watch()
        .expect("third watch start (latched no-op)");
    assert_eq!(
        net.network_subscriber_count(),
        1,
        "repeated watch starts must not stack the subscriber count"
    );

    // register_ingress_topic shares the SAME latch — still 1.
    transport
        .register_ingress_topic(&topic, 0xF00D, MaxSliceLen::const_new(256))
        .expect("register ingress topic");
    assert_eq!(
        net.network_subscriber_count(),
        1,
        "register_ingress_topic must reuse the latched watch, not stack it"
    );
    assert!(net.is_task_running());
}

/// Teardown pin: register an ingress bridge, then drop the manager — no hang,
/// no panic, and the topic's iceoryx2 services are RELEASED (a fresh manager
/// on the SAME SHM root sees no leftover service). Pins the Drop chain:
/// NetworkManager (declared before `node`) tears the ingress map down —
/// zenoh subscriber + token undeclared, injection publisher's ports dropped —
/// then the watch task joins, then the session closes, then the node drops.
#[test]
fn test_teardown_releases_services_without_hang() {
    let ix = cerulion_core::testing::iceoryx_test_config();
    let id = unique_id();
    let topic = format!("/ingress/teardown/{id}");

    let transport = TransportManager::init_for_test(
        TransportConfig {
            node_name: format!("ingress_teardown_{id}"),
            network: Some(NetworkConfig::default()),
            ..Default::default()
        },
        ix.clone(),
    )
    .expect("init_for_test with network");

    transport
        .register_ingress_topic(&topic, 0xABCD, MaxSliceLen::const_new(256))
        .expect("register ingress topic");
    assert!(transport.network().expect("network").is_task_running());

    // Drop the ONLY Arc — the full teardown chain runs (watch thread joined,
    // ingress cleared, session closed, node dropped). Completing without a
    // hang/panic is the liveness half of the pin.
    drop(transport);

    // Services-released half: a fresh manager on the SAME SHM root finds no
    // leftover service for the topic (open-only refuses to create).
    let probe = TransportManager::init_for_test(
        TransportConfig {
            node_name: format!("ingress_teardown_probe_{id}"),
            ..Default::default()
        },
        ix,
    )
    .expect("probe manager on the same SHM root");
    assert!(
        probe.create_subscriber_open_only(&topic).is_err(),
        "the ingress topic's iceoryx2 service must be gone after manager drop"
    );
}

/// The wired-not-just-registered pin: after ONE register_ingress_topic call,
/// a raw frame put on the transport's OWN zenoh session (same-session
/// loopback) lands on a local iceoryx2 subscriber with the hand-stamped
/// header + payload intact — the whole production wiring resolves end-to-end.
#[test]
fn test_register_ingress_topic_wired_end_to_end_same_session() {
    let (transport, topic) = setup_with_network("wired_e2e");
    let expected_hash = 0x5EED_5EED_5EED_5EEDu64;

    let subscriber = transport.create_subscriber(&topic).expect("subscriber");
    transport
        .register_ingress_topic(&topic, expected_hash, MaxSliceLen::const_new(256))
        .expect("register ingress topic");

    let net = transport.network().expect("network");
    let frame = make_wire_frame(expected_hash, 11, 2_222, &[0xAA, 0xBB]);
    net.publish_to_network(&topic, frame.clone())
        .expect("same-session put");

    let (header, payload) =
        poll_one(&subscriber, Duration::from_secs(3)).expect("wired ingress delivers locally");
    assert_eq!(header.schema_hash, expected_hash);
    assert_eq!(header.sequence, 11);
    assert_eq!(header.timestamp_ns, 2_222);
    assert_eq!(header.total_size as usize, frame.len());
    assert_eq!(payload, vec![0xAA, 0xBB]);
    // The full received frame slice equals the sent frame.
    assert_eq!(
        rebuild_frame(&header, &payload),
        frame,
        "FULL frame slice must be byte-identical (offset-table bytes included)"
    );

    let stats = net.ingress_stats(&topic).expect("stats");
    assert_eq!(stats.frames, 1);
    assert_eq!(stats.schema_mismatch_drops, 0);
    assert_eq!(stats.decode_errors, 0);
    assert_eq!(stats.reinject_failure_drops, 0);
}

// ---------------------------------------------------------------------------
// Multi-topic isolation + the
// re-injection-failure arm (counted, latched, recovers)
// ---------------------------------------------------------------------------

/// TWO concurrent ingress bridges on ONE `NetworkManager`
/// — distinct topics, distinct schema hashes, interleaved frames. Each frame
/// delivers ONLY to its own local topic, the per-topic counters stay
/// independent, and a cross-schema probe (topic B's hash put on topic A's
/// key) is dropped by A's gate without touching B's counters.
#[test]
fn test_multi_topic_concurrent_ingress_isolation() {
    let (transport, base) = setup("multi_topic");
    let topic_a = format!("{base}/a");
    let topic_b = format!("{base}/b");
    let hash_a = 0xAAAA_0000_0000_000Au64;
    let hash_b = 0xBBBB_0000_0000_000Bu64;

    let sub_a = transport.create_subscriber(&topic_a).expect("sub A");
    let sub_b = transport.create_subscriber(&topic_b).expect("sub B");
    let pub_a = transport
        .create_ingress_publisher(&topic_a, MaxSliceLen::const_new(256))
        .expect("ingress pub A");
    let pub_b = transport
        .create_ingress_publisher(&topic_b, MaxSliceLen::const_new(256))
        .expect("ingress pub B");

    let mgr = NetworkManager::new(NetworkConfig::default());
    mgr.register_ingress(&topic_a, hash_a, pub_a)
        .expect("register A");
    mgr.register_ingress(&topic_b, hash_b, pub_b)
        .expect("register B");

    // Interleave the two topics' frames, plus a cross-schema probe (topic
    // B's hash on topic A's key) that A's gate must drop.
    mgr.publish_to_network(&topic_a, make_wire_frame(hash_a, 1, 100, &[0xA1]))
        .expect("a1");
    mgr.publish_to_network(&topic_b, make_wire_frame(hash_b, 10, 1000, &[0xB1]))
        .expect("b1");
    mgr.publish_to_network(&topic_a, make_wire_frame(hash_b, 2, 200, &[0xEE]))
        .expect("cross-schema probe");
    mgr.publish_to_network(&topic_a, make_wire_frame(hash_a, 3, 300, &[0xA2]))
        .expect("a2");
    mgr.publish_to_network(&topic_b, make_wire_frame(hash_b, 11, 1100, &[0xB2]))
        .expect("b2");

    // Collect both sides (bounded). One publisher per topic → FIFO order.
    let collect = |sub: &CerulionSubscriber, want: usize| -> Vec<(u32, Vec<u8>)> {
        let mut got: Vec<(u32, Vec<u8>)> = Vec::new();
        let deadline = Instant::now() + Duration::from_secs(3);
        while got.len() < want && Instant::now() < deadline {
            sub.try_receive(|msg| {
                got.push((msg.header().sequence, msg.payload().to_vec()));
            })
            .expect("try_receive");
            if got.len() < want {
                std::thread::sleep(Duration::from_millis(10));
            }
        }
        got
    };
    let got_a = collect(&sub_a, 2);
    let got_b = collect(&sub_b, 2);

    // Hand oracles: each topic sees ONLY its own frames, in order — the
    // cross-schema probe reached neither subscriber.
    assert_eq!(
        got_a,
        vec![(1, vec![0xA1]), (3, vec![0xA2])],
        "topic A delivers only A's frames"
    );
    assert_eq!(
        got_b,
        vec![(10, vec![0xB1]), (11, vec![0xB2])],
        "topic B delivers only B's frames"
    );

    // Counters independent: A took the schema-mismatch hit; B is clean.
    // (Poll for the probe's drop to be accounted — the callback is async.)
    let deadline = Instant::now() + Duration::from_secs(3);
    while Instant::now() < deadline
        && mgr
            .ingress_stats(&topic_a)
            .unwrap_or_default()
            .schema_mismatch_drops
            < 1
    {
        std::thread::sleep(Duration::from_millis(10));
    }
    let stats_a = mgr.ingress_stats(&topic_a).expect("stats A");
    let stats_b = mgr.ingress_stats(&topic_b).expect("stats B");
    assert_eq!(stats_a.frames, 2);
    assert_eq!(
        stats_a.schema_mismatch_drops, 1,
        "the cross-schema probe lands in A's bucket"
    );
    assert_eq!(stats_a.decode_errors, 0);
    assert_eq!(stats_a.reinject_failure_drops, 0);
    assert_eq!(stats_b.frames, 2);
    assert_eq!(
        stats_b.schema_mismatch_drops, 0,
        "B's counters are untouched by A's probe"
    );
    assert_eq!(stats_b.decode_errors, 0);
    assert_eq!(stats_b.reinject_failure_drops, 0);
}

/// Drive the `publish_raw`-Err arm of the ingress
/// callback for REAL — the existing fire-once `fault_inject_publish_raw_after`
/// seam (publisher.rs precedent) makes the FIRST re-injection fail with
/// `LoanCapacity` (the genuine SHM-pressure error class). The frame is
/// dropped COUNTED (`reinject_failure_drops`), the latch fires the loud warn
/// EXACTLY once, and the next frame recovers (delivered + success-counted +
/// exactly one recovery info). The repeat-downgrade half of the discipline is
/// unit-pinned at the latch level in `network.rs`
/// (`record_reinject_failure_counts_and_latches_once`) — the fire-once seam
/// produces exactly one failure per arming, so it cannot exercise repeats.
#[test]
#[tracing_test::traced_test]
fn test_reinject_failure_is_counted_and_latch_warns_once() {
    let (transport, topic) = setup("reinject_fail");
    let hash = 0xFA11_FA11_FA11_FA11u64;

    let subscriber = transport.create_subscriber(&topic).expect("subscriber");
    let mut ingress_pub = transport
        .create_ingress_publisher(&topic, MaxSliceLen::const_new(256))
        .expect("ingress publisher");
    // Fire-once fault: the FIRST publish_raw call fails, then the seam
    // self-clears — frame 2 exercises the recovery path.
    ingress_pub.fault_inject_publish_raw_after(0);

    let mgr = NetworkManager::new(NetworkConfig::default());
    mgr.register_ingress(&topic, hash, ingress_pub)
        .expect("register ingress");

    // Frame 1: validates, re-injection FAILS → counted drop, no delivery.
    mgr.publish_to_network(&topic, make_wire_frame(hash, 1, 100, &[0x01]))
        .expect("put frame 1");
    let deadline = Instant::now() + Duration::from_secs(3);
    while Instant::now() < deadline
        && mgr
            .ingress_stats(&topic)
            .unwrap_or_default()
            .reinject_failure_drops
            < 1
    {
        std::thread::sleep(Duration::from_millis(10));
    }
    let stats = mgr.ingress_stats(&topic).expect("stats");
    assert_eq!(
        stats.reinject_failure_drops, 1,
        "the failed re-injection is COUNTED"
    );
    assert_eq!(stats.frames, 0, "a failed re-injection is not a success");
    assert!(
        poll_one(&subscriber, Duration::from_millis(200)).is_none(),
        "the failed frame must not be delivered"
    );

    // Frame 2: the fault cleared — delivered, success-counted, recovery info.
    mgr.publish_to_network(&topic, make_wire_frame(hash, 2, 200, &[0x02]))
        .expect("put frame 2");
    let (header, payload) =
        poll_one(&subscriber, Duration::from_secs(3)).expect("frame 2 delivers after recovery");
    assert_eq!(header.sequence, 2);
    assert_eq!(payload, vec![0x02]);
    let stats = mgr.ingress_stats(&topic).expect("stats");
    assert_eq!(stats.frames, 1);
    assert_eq!(stats.reinject_failure_drops, 1);

    // Latch discipline (the unique topic name filters out any parallel
    // test's lines): the loud warn fired EXACTLY once, and the recovery info
    // fired EXACTLY once.
    logs_assert(|lines: &[&str]| {
        let warns = lines
            .iter()
            .filter(|l| {
                l.contains(topic.as_str()) && l.contains("re-injection into local iceoryx2 failed")
            })
            .count();
        let recoveries = lines
            .iter()
            .filter(|l| l.contains(topic.as_str()) && l.contains("recovered — frames validating"))
            .count();
        if warns == 1 && recoveries == 1 {
            Ok(())
        } else {
            Err(format!(
                "expected exactly 1 reinject warn + 1 recovery info, got {warns} warn(s) / \
                 {recoveries} recovery(ies)"
            ))
        }
    });
}

// ---------------------------------------------------------------------------
// The liveliness watch must NOT warn-refuse a demand token for a topic
// THIS manager INGRESSES (its own re-injection echo — the wrong-audience noise
// a `topic hz`/`echo` desk raises against an attached robot), while STILL warning for a
// FOREIGN demand it does not ingress (the load-bearing robot-side genuine
// refusal path). One manager, one isolated same-session watch drives both arms.
// ---------------------------------------------------------------------------

/// Self-ingress behavioral e2e. The desk's `register_ingress_topic` starts the watch
/// AND declares its OWN demand token for the ingressed topic; the same-session
/// watch hears that Put and, absent the self-ingress check, would run
/// `enable_bridge` (deny-all posture) → the "bridge-enable refused … egress
/// allow-list …" WARN about the machine's own demand. With the check the watch
/// recognizes the topic as self-ingress and SKIPS
/// it before `enable_bridge`. A DISTINCT foreign demand token (a stand-in for a
/// remote subscriber asking for a topic this desk does NOT ingress) still routes
/// to `enable_bridge` and IS refused — the anti-tautology control proving the
/// refusal path is alive.
///
/// The observable is the durable per-topic refusal latch (`was_egress_refused`),
/// NOT a log line: the WARN fires on the dedicated watch thread, which
/// `tracing_test` cannot capture (it scopes to the in-span test thread). The
/// foreign refusal is the DETERMINISTIC wait signal — the watch hears the mine
/// token (declared during `register_ingress_topic`) BEFORE the foreign token
/// (declared after), so once `was_egress_refused(foreign)` flips, the mine token
/// has definitely been processed; a mine refusal, had the skip regressed, would
/// already be latched. Hand oracle (foreign refused / mine NOT refused), unique
/// per-test topic names, per-test SHM root + isolated scouting-off zenoh session
/// (parallel-safe). Removing the watch's self-ingress skip flips
/// `was_egress_refused(mine)` to `true` and fails.
#[test]
fn self_ingress_demand_is_not_refused_but_foreign_is() {
    use cerulion_core::transport::discovery::TopicToken;

    let (transport, mine) = setup_with_network("mine");
    // A separately-minted unique topic so `mine` and `foreign` are unrelated.
    let (_t2, foreign) = setup_with_network("foreign");

    // Register the desk's ingress for `mine`: starts the watch AND declares our
    // OWN demand token (the Put the watch hears about itself).
    transport
        .register_ingress_topic(&mine, 0xABCD_0000_0000_0001, MaxSliceLen::const_new(256))
        .expect("register ingress for the desk's own topic");

    // Declare a FOREIGN demand token on the SAME session — a stand-in for a
    // remote subscriber demanding a topic this desk does NOT ingress. The watch
    // hears it, it is not in `self_ingress`, so it routes to enable_bridge and is
    // refused (deny-all posture) — the control.
    let net = transport.network().expect("network configured");
    let session = net
        .session()
        .expect("session opened by register_ingress_topic");
    let _foreign_token =
        TopicToken::declare(session, &foreign).expect("declare a foreign demand token");

    let bridge = transport.bridge_manager();
    // Bounded wait until the foreign refusal is latched — the deterministic
    // signal the watch processed the (earlier-declared) mine token too.
    let deadline = Instant::now() + Duration::from_secs(5);
    while Instant::now() < deadline && !bridge.was_egress_refused(&foreign).unwrap_or(false) {
        std::thread::sleep(Duration::from_millis(20));
    }

    assert!(
        bridge.was_egress_refused(&foreign).expect("gate readable"),
        "control: a FOREIGN demand for a topic this desk does not ingress must still \
         be egress-refused (the load-bearing genuine-refusal path stays alive)"
    );
    assert!(
        !bridge.was_egress_refused(&mine).expect("gate readable"),
        "the desk's OWN ingress demand echo must NOT be egress-refused — the watch \
         skips enable_bridge for a self-ingress topic (no wrong-audience WARN)"
    );
}
