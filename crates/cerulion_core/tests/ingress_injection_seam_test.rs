// SPDX-License-Identifier: AGPL-3.0-only
//! Remote plane: the ZENOH-FREE local-SHM ingress injection
//! seam (`TransportManager::create_ingress_injector` → `IngressInjector`).
//!
//! This is `register_ingress_topic`'s Step 2 + the validate/count logic
//! exposed as a standalone primitive that works on a `network: None`
//! `TransportManager` — the exact primitive the iroh-sourced desk re-inject
//! client (`cerulion connect`) and `cerulion_remoted` use to turn a frame read
//! off the iroh tunnel into a LOCAL SHM topic ("remote = local"), with NO zenoh
//! session opened.
//!
//! Cribs `network_ingress_test.rs`'s hand-oracle discipline (KNOWN header
//! fields and payload we stamped; full-frame-slice byte compares via
//! `write_to_buf` + the payload slice), MINUS the zenoh half — here the
//! re-injection is a direct synchronous `IngressInjector::reinject_raw`, so
//! there is no async settle window.
//!
//! Real iceoryx2 via PER-TEST SHM roots (`init_for_test` +
//! `iceoryx_test_config`), so the whole file is parallel-safe (no `#[serial]`,
//! no `--test-threads=1`).

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant, SystemTime};

use cerulion_core::transport::{
    PublisherProvisioning, TopicServiceConfig, TransportConfig, TransportManager,
};
use cerulion_core::wire::{MaxSliceLen, WireHeader};
use cerulion_core::{CerulionSubscriber, IngressRejection, ReinjectOutcome};

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

/// A fresh per-test `TransportManager` (isolated SHM root, `network: None`) plus
/// a unique canonical topic. The manager has NO `NetworkManager` — the seam must
/// work without one (and open no session).
fn setup(base: &str) -> (Arc<TransportManager>, String) {
    let ix = cerulion_core::testing::iceoryx_test_config();
    let id = unique_id();
    let transport = TransportManager::init_for_test(
        TransportConfig {
            node_name: format!("inj_{base}_{id}"),
            ..Default::default()
        },
        ix,
    )
    .expect("init_for_test");
    (transport, format!("/inj/{base}/{id}"))
}

/// Hand-build a raw wire frame (32-byte little-endian header + payload) with a
/// CORRECT `total_size`.
fn make_wire_frame(schema_hash: u64, seq: u32, timestamp_ns: u64, payload: &[u8]) -> Vec<u8> {
    make_wire_frame_with_total_size(
        schema_hash,
        seq,
        timestamp_ns,
        payload,
        (WireHeader::SIZE + payload.len()) as u32,
    )
}

/// Hand-build a raw wire frame whose header carries `total_size_override` (which
/// may DISAGREE with the real byte length — used to forge a `total_size` lie).
fn make_wire_frame_with_total_size(
    schema_hash: u64,
    seq: u32,
    timestamp_ns: u64,
    payload: &[u8],
    total_size_override: u32,
) -> Vec<u8> {
    let header = WireHeader {
        schema_hash,
        total_size: total_size_override,
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

/// Rebuild the FULL received wire frame from a `ReceivedMessage`'s two surfaces
/// (crib `network_ingress_test.rs`): the owned header re-serialized via
/// `write_to_buf` (the exact inverse of `read_from_buf`) + the zero-copy payload
/// slice. Their concatenation IS the full received frame slice — comparable
/// byte-for-byte against the sent frame, offset-table bytes included.
fn rebuild_frame(header: &WireHeader, payload: &[u8]) -> Vec<u8> {
    let mut frame = vec![0u8; WireHeader::SIZE + payload.len()];
    header.write_to_buf(&mut frame[..WireHeader::SIZE]);
    frame[WireHeader::SIZE..].copy_from_slice(payload);
    frame
}

/// Poll the local subscriber until ONE frame is delivered (or timeout). Returns
/// the received (header, payload).
fn poll_one(sub: &CerulionSubscriber, timeout: Duration) -> Option<(WireHeader, Vec<u8>)> {
    let deadline = Instant::now() + timeout;
    loop {
        let mut got: Option<(WireHeader, Vec<u8>)> = None;
        sub.try_receive(|msg| {
            got = Some((*msg.header(), msg.payload().to_vec()));
        })
        .expect("try_receive");
        if got.is_some() {
            return got;
        }
        if Instant::now() > deadline {
            return None;
        }
        std::thread::sleep(Duration::from_millis(10));
    }
}

/// THE headline seam pin: on a `network: None` manager, a hand-oracle frame
/// re-injected through the zenoh-free `IngressInjector::reinject_raw` lands on a
/// local iceoryx2 subscriber BYTE-IDENTICAL (schema_hash + sequence +
/// timestamp_ns + payload we stamped — a hand oracle, not a self-compare), and
/// NO zenoh session is ever opened.
#[test]
fn seam_reinjects_frame_byte_identical_without_zenoh() {
    let (transport, topic) = setup("byte_identical");
    let expected_hash = 0xABCD_EF01_2345_6789u64;

    // The pin's first half: the manager has NO network — the seam must work
    // without one (a desk / ingress-only remoted is `network: None`).
    assert!(
        transport.network().is_none(),
        "the injection seam must work on a `network: None` manager (no zenoh session)"
    );

    // Local subscriber connected BEFORE the injector's publisher so it receives
    // the re-injected send.
    let subscriber = transport.create_subscriber(&topic).expect("subscriber");
    let injector = transport
        .create_ingress_injector(&topic, expected_hash, MaxSliceLen::const_new(256))
        .expect("ingress injector");

    assert_eq!(injector.expected_schema_hash(), expected_hash);
    assert_eq!(injector.topic(), topic.as_str());

    // Hand oracle: KNOWN header fields + payload.
    let seq = 7u32;
    let ts = 123_456_789u64;
    let payload = vec![0xDE, 0xAD, 0xBE, 0xEF];
    let frame = make_wire_frame(expected_hash, seq, ts, &payload);

    let outcome = injector.reinject_raw(&frame);
    let ReinjectOutcome::Injected { recipients } = outcome else {
        panic!("expected Injected, got {outcome:?}");
    };
    assert!(
        recipients >= 1,
        "the re-injected frame must reach the local subscriber (recipients={recipients})"
    );

    let (header, got) =
        poll_one(&subscriber, Duration::from_secs(3)).expect("re-injected frame delivered locally");

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

    let stats = injector.stats();
    assert_eq!(stats.frames, 1, "exactly one frame re-injected");
    assert_eq!(stats.schema_mismatch_drops, 0);
    assert_eq!(stats.decode_errors, 0);
    assert_eq!(stats.reinject_failure_drops, 0);

    // The seam never opened a network.
    assert!(
        transport.network().is_none(),
        "the seam must NOT create a network manager / zenoh session"
    );
}

/// A schema-mismatch frame is REFUSED + COUNTED (`schema_mismatch_drops`), never
/// re-injected — the local subscriber sees nothing. The outcome names the class.
#[test]
fn seam_refuses_and_counts_schema_mismatch() {
    let (transport, topic) = setup("schema_mismatch");
    let expected_hash = 0x1111_2222_3333_4444u64;

    let subscriber = transport.create_subscriber(&topic).expect("subscriber");
    let injector = transport
        .create_ingress_injector(&topic, expected_hash, MaxSliceLen::const_new(256))
        .expect("ingress injector");

    let bad = make_wire_frame(0x9999_9999_9999_9999, 5, 500, &[1, 2, 3]);
    let outcome = injector.reinject_raw(&bad);
    assert_eq!(
        outcome,
        ReinjectOutcome::Rejected(IngressRejection::SchemaMismatch),
        "a wrong-hash frame is a schema-mismatch rejection"
    );

    let stats = injector.stats();
    assert_eq!(stats.schema_mismatch_drops, 1);
    assert_eq!(stats.frames, 0);
    assert_eq!(stats.decode_errors, 0);
    assert_eq!(stats.reinject_failure_drops, 0);

    assert!(
        poll_one(&subscriber, Duration::from_millis(200)).is_none(),
        "a schema-mismatch frame must NOT be delivered to the local subscriber"
    );
}

/// A `total_size`-lie frame (header's `total_size` disagrees with the actual
/// byte length) is REFUSED + COUNTED as a structural `decode_errors` — never
/// re-injected. This is the total_size check applied at the zenoh-free
/// seam.
#[test]
fn seam_refuses_and_counts_total_size_lie() {
    let (transport, topic) = setup("total_size_lie");
    let expected_hash = 0x5A5A_5A5A_5A5A_5A5Au64;

    let subscriber = transport.create_subscriber(&topic).expect("subscriber");
    let injector = transport
        .create_ingress_injector(&topic, expected_hash, MaxSliceLen::const_new(256))
        .expect("ingress injector");

    // Correct schema_hash, but a header total_size that lies about the length:
    // real frame is SIZE + 3 bytes, header claims SIZE + 99.
    let payload = vec![0xAA, 0xBB, 0xCC];
    let lie = make_wire_frame_with_total_size(
        expected_hash,
        9,
        900,
        &payload,
        (WireHeader::SIZE + 99) as u32,
    );
    let outcome = injector.reinject_raw(&lie);
    assert_eq!(
        outcome,
        ReinjectOutcome::Rejected(IngressRejection::SizeMismatch),
        "a total_size-lie frame is a size-mismatch rejection"
    );

    let stats = injector.stats();
    assert_eq!(
        stats.decode_errors, 1,
        "the total_size lie is a decode error"
    );
    assert_eq!(stats.frames, 0);
    assert_eq!(stats.schema_mismatch_drops, 0);
    assert_eq!(stats.reinject_failure_drops, 0);

    // A frame SHORTER than the 32-byte header is the `TooSmall` decode class.
    let too_small = vec![0u8; WireHeader::SIZE - 1];
    let outcome = injector.reinject_raw(&too_small);
    assert_eq!(
        outcome,
        ReinjectOutcome::Rejected(IngressRejection::TooSmall),
        "a sub-header frame is a too-small rejection"
    );
    assert_eq!(injector.stats().decode_errors, 2);

    assert!(
        poll_one(&subscriber, Duration::from_millis(200)).is_none(),
        "no malformed frame reaches the local subscriber"
    );
}

/// Ingress injection is REFUSED (loud `Err` naming the conflict) on a topic
/// already owned by a live in-graph single-writer producer — the one publisher
/// slot is taken. The LOCAL single-writer invariant is preserved at the
/// zenoh-free seam (it is zenoh-independent).
#[test]
fn seam_refused_when_live_single_writer_producer_exists() {
    let (transport, topic) = setup("single_writer_conflict");

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

    // `IngressInjector` is not `Debug`, so `let Err(..) = .. else` (not
    // `expect_err`, which would format the Ok arm).
    let Err(err) = transport.create_ingress_injector(&topic, 0xABCD, MaxSliceLen::const_new(256))
    else {
        panic!("ingress injection must be refused when a live producer owns the topic");
    };
    // Pin the DISTINCTIVE single-writer slot-conflict discriminator, not a
    // disjunction of generic words: EVERY `TransportError::PublisherCreation`
    // Display carries "Failed to create publisher on topic ..." (so a bare
    // `contains("publisher")` matches ANY creation failure — loop refusal, bad
    // name, pool exhaustion). The actual refusal here is the `finish_publisher`
    // ExceedsMaxSupportedPublishers hint: the ingress publisher opens the live
    // SingleWriter producer's `max_publishers = 1` service and is rejected at
    // port creation with " — a publisher already holds the topic's only slot
    // (graph topics are provisioned single-writer: ...)". Assert those distinct
    // phrases CONJUNCTIVELY so a generic creation failure can no longer satisfy
    // the oracle.
    let msg = format!("{err}").to_lowercase();
    assert!(
        msg.contains("holds the topic's only slot") && msg.contains("single-writer"),
        "error must name the single-writer slot-taken conflict (the \
         ExceedsMaxSupportedPublishers hint), got: {msg}"
    );
}

/// Determinism (Principle #7): two IDENTICAL injection sequences through the
/// seam deliver BYTE-IDENTICAL frames, and both equal a HAND oracle (the
/// injected seq/ts/payload) — not merely a self-compare.
#[test]
fn seam_delivery_is_deterministic() {
    let hash = 0xCAFE_D00D_0000_0001u64;
    let injections: [(u32, u64, Vec<u8>); 3] = [
        (1, 10, vec![1, 2, 3]),
        (2, 20, vec![4, 5, 6]),
        (3, 30, vec![7, 8, 9]),
    ];

    let run = |base: &str| -> Vec<(u32, u64, Vec<u8>)> {
        let (transport, topic) = setup(base);
        let subscriber = transport.create_subscriber(&topic).expect("subscriber");
        let injector = transport
            .create_ingress_injector(&topic, hash, MaxSliceLen::const_new(256))
            .expect("ingress injector");

        for (seq, ts, payload) in &injections {
            let outcome = injector.reinject_raw(&make_wire_frame(hash, *seq, *ts, payload));
            assert!(
                matches!(outcome, ReinjectOutcome::Injected { .. }),
                "each in-sequence frame injects: {outcome:?}"
            );
        }

        // Collect all delivered frames (FIFO — one publisher per topic).
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

/// The `ReinjectFailed` path is COUNTED + latch-warned, never silent — driven
/// for REAL via the fire-once `publish_raw` fault seam (the same seam
/// `network_ingress_test.rs` uses, here on the zenoh-free injector). Frame 1's
/// re-injection FAILS (counted `reinject_failure_drops`, one loud warn, no
/// delivery); frame 2 recovers (delivered, success-counted, one recovery info).
#[test]
#[tracing_test::traced_test]
fn seam_reinject_failure_is_counted_and_latch_warns_once() {
    let (transport, topic) = setup("reinject_fail");
    let hash = 0xFA11_FA11_FA11_FA11u64;

    let subscriber = transport.create_subscriber(&topic).expect("subscriber");
    let injector = transport
        .create_ingress_injector(&topic, hash, MaxSliceLen::const_new(256))
        .expect("ingress injector");
    // Fire-once fault: the FIRST publish_raw fails, then the seam self-clears.
    injector.fault_inject_publish_raw_after(0);

    // Frame 1: validates, re-injection FAILS → counted drop, no delivery.
    let outcome = injector.reinject_raw(&make_wire_frame(hash, 1, 100, &[0x01]));
    assert_eq!(
        outcome,
        ReinjectOutcome::ReinjectFailed,
        "a validated-but-unpublishable frame is a ReinjectFailed outcome"
    );
    let stats = injector.stats();
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
    let outcome = injector.reinject_raw(&make_wire_frame(hash, 2, 200, &[0x02]));
    assert!(
        matches!(outcome, ReinjectOutcome::Injected { .. }),
        "frame 2 recovers: {outcome:?}"
    );
    let (header, payload) =
        poll_one(&subscriber, Duration::from_secs(3)).expect("frame 2 delivers after recovery");
    assert_eq!(header.sequence, 2);
    assert_eq!(payload, vec![0x02]);
    let stats = injector.stats();
    assert_eq!(stats.frames, 1);
    assert_eq!(stats.reinject_failure_drops, 1);

    // Latch discipline (the unique topic name filters any parallel test's
    // lines): the loud warn fired EXACTLY once and the recovery info EXACTLY
    // once.
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

/// `recipients` is `Injected`'s ENTIRE payload — pin its two unpinned regimes:
/// the `recipients == 0` steady state (the desk viewer closed while `cerulion
/// connect` keeps streaming) and injector survivability across a subscriber
/// disconnect mid-stream. Hand oracles at every phase (never a self-compare);
/// per-test SHM root (the file's `setup` pattern).
#[test]
fn seam_survives_zero_subscribers_and_subscriber_drop_midstream() {
    let (transport, topic) = setup("zero_sub_survival");
    let hash = 0xB0B0_C0DE_0000_0001u64;

    let injector = transport
        .create_ingress_injector(&topic, hash, MaxSliceLen::const_new(256))
        .expect("ingress injector");

    // (a) NO subscriber attached: a valid frame still injects, reaching ZERO
    // recipients (deterministic — no subscriber can be connected), and `frames`
    // is counted (the injector never gates on a consumer being present — the
    // `cerulion connect` stream keeps flowing with the desk viewer closed).
    let outcome = injector.reinject_raw(&make_wire_frame(hash, 1, 100, &[0x11]));
    assert_eq!(
        outcome,
        ReinjectOutcome::Injected { recipients: 0 },
        "with no local subscriber the frame injects to ZERO recipients"
    );
    assert_eq!(
        injector.stats().frames,
        1,
        "a zero-recipient re-injection is still a counted success"
    );

    // (b) Attach a subscriber AFTER the injector's publisher already exists;
    // poll until the publisher establishes the connection (`recipients >= 1`).
    // The FIRST frame with `recipients >= 1` is the first one this late
    // subscriber receives — there is no late-joiner history, so earlier
    // zero-recipient frames are gone.
    let sub_a = transport.create_subscriber(&topic).expect("subscriber A");
    let deadline = Instant::now() + Duration::from_secs(3);
    let mut seq = 2u32;
    let delivered_seq = loop {
        let outcome =
            injector.reinject_raw(&make_wire_frame(hash, seq, seq as u64 * 100, &[seq as u8]));
        let ReinjectOutcome::Injected { recipients } = outcome else {
            panic!("expected Injected while sub A attaches, got {outcome:?}");
        };
        if recipients >= 1 {
            break seq;
        }
        assert!(
            Instant::now() < deadline,
            "subscriber A never became a recipient of the live injector"
        );
        seq += 1;
        std::thread::sleep(Duration::from_millis(10));
    };
    let (h, p) = poll_one(&sub_a, Duration::from_secs(3)).expect("frame delivered to sub A");
    assert_eq!(
        h.sequence, delivered_seq,
        "sub A receives the recipient frame"
    );
    assert_eq!(p, vec![delivered_seq as u8]);

    // ... then DROP the sole subscriber mid-stream. The next re-injections must
    // NOT Err / wedge — `recipients` falls back to 0 (iceoryx2 reclaims the
    // dropped subscriber's connection asynchronously; poll it back to 0). Every
    // re-injection across the disconnect stays an `Injected` success.
    drop(sub_a);
    let deadline = Instant::now() + Duration::from_secs(3);
    seq += 1;
    let recipients_after_drop = loop {
        let outcome =
            injector.reinject_raw(&make_wire_frame(hash, seq, seq as u64 * 100, &[seq as u8]));
        let ReinjectOutcome::Injected { recipients } = outcome else {
            panic!("re-injection after subscriber drop must still Inject, got {outcome:?}");
        };
        if recipients == 0 {
            break recipients;
        }
        assert!(
            Instant::now() < deadline,
            "recipient count never fell back to 0 after the sole subscriber dropped"
        );
        seq += 1;
        std::thread::sleep(Duration::from_millis(10));
    };
    assert_eq!(
        recipients_after_drop, 0,
        "after the sole subscriber drops, re-injection reaches ZERO recipients (injector survived)"
    );

    // (c) A FRESH subscriber attaching AFTER the disconnect receives SUBSEQUENT
    // frames — the injector (and its publisher) is still live. No late-joiner
    // history, so it sees only frames injected after IT attaches.
    let sub_b = transport.create_subscriber(&topic).expect("subscriber B");
    let deadline = Instant::now() + Duration::from_secs(3);
    seq += 1;
    let fresh_seq = loop {
        let outcome =
            injector.reinject_raw(&make_wire_frame(hash, seq, seq as u64 * 100, &[seq as u8]));
        let ReinjectOutcome::Injected { recipients } = outcome else {
            panic!("expected Injected while sub B attaches, got {outcome:?}");
        };
        if recipients >= 1 {
            break seq;
        }
        assert!(
            Instant::now() < deadline,
            "fresh subscriber B never became a recipient (injector did not survive)"
        );
        seq += 1;
        std::thread::sleep(Duration::from_millis(10));
    };
    let (h, p) =
        poll_one(&sub_b, Duration::from_secs(3)).expect("subsequent frame delivered to sub B");
    assert_eq!(
        h.sequence, fresh_seq,
        "the fresh subscriber receives a subsequent frame (proving survival)"
    );
    assert_eq!(p, vec![fresh_seq as u8]);

    // No re-injection ever failed across the whole disconnect/reconnect cycle.
    let stats = injector.stats();
    assert_eq!(stats.reinject_failure_drops, 0, "no re-injection failed");
    assert_eq!(stats.schema_mismatch_drops, 0);
    assert_eq!(stats.decode_errors, 0);
}
