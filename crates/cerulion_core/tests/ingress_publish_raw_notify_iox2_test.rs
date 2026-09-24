// SPDX-License-Identifier: AGPL-3.0-only
//! A raw-INGRESS publisher (`TransportManager::create_ingress_publisher`)
//! wakes its FOREIGN consumers from `publish_raw` itself — the structural fix for
//! a raw-ingress publisher that never notified its listeners.
//!
//! # The bug
//!
//! The DDS-bridge's `RawIngressRoute` (the `dds_bridge` node — the SINGLE
//! checked-in bridge every `cerulion ros2 attach` runs) republishes each raw
//! DDS frame via `CerulionPublisher::publish_raw`, which — by contract — does NOT
//! notify. Every OTHER raw-ingress caller (`IngressInjector::reinject_raw`, the
//! rmw/service bridges) calls `notify_sent_sample` right after `publish_raw`;
//! `RawIngressRoute` FORGOT it. So an event-driven `topic hz`/`echo` on a
//! raw-routed topic (e.g. `/lowstate` at 500 Hz — the majority of an attach
//! graph's ~90 topics) blocked on the topic's event listener FOREVER: the frames
//! were in the SHM queue (iceoryx2's `send` auto-updates its subscriber
//! connections) but no `SentSample` wake ever fired.
//!
//! # The fix (structural, at the GENERAL cerulion_core seam)
//!
//! `create_ingress_publisher` arms the publisher (`arm_publish_raw_notify`) so
//! `publish_raw` fires the wake ITSELF — routed through `notify_sent_sample` so
//! the elision gate still applies (an unarmed ingress publisher always
//! notifies, correct: its consumers are cross-process, never in-graph
//! level-gated). The wake is now part of the publish for EVERY ingress caller —
//! no "remember to notify after publish_raw" convention (the convention that
//! CAUSED this bug). Nothing robot-specific: any attach bridge on any robot
//! heals. The now-redundant explicit `notify_sent_sample` in
//! `IngressInjector::reinject_raw` was removed (folded into `publish_raw`) so the
//! network re-inject path does not double-notify.
//!
//! # SCOPE: the LISTENER-driven ingress class only
//!
//! This wakes a LISTENER-driven consumer of a raw-injected topic (`topic hz`/
//! `echo`, vizd, an rmw listener). It does NOT touch the poll-paced GATEWAY /
//! `remoted` EGRESS tap, which is a listener-less `DataOnlySubscriber` drained in
//! a bare poll loop — structurally notify-immune, so this fix cannot (and does
//! not claim to) heal a stall on the egress tap. That
//! path has a different root cause.
//!
//! # What this file pins
//!
//! - **(A) headline, hand oracle:** a REAL `create_ingress_publisher` +
//!   `publish_raw` wakes a blocked foreign LISTENER-full subscriber (the
//!   `topic hz` shape, on a SEPARATE manager over the same SHM root) within a
//!   BOUNDED window well under its own long wait timeout, delivering the exact
//!   stamped frame. With the fix reverted the subscriber wakes only on its own
//!   2 s timeout — the negative control.
//! - **(B) deterministic core + scope control:** an ingress publisher's
//!   `publish_raw` fires a `SentSample` a foreign raw `Listener` receives, while
//!   a NON-ingress `create_publisher`'s `publish_raw` fires NOTHING (the flag
//!   scopes the auto-notify to ingress publishers, so the rmw/service callers —
//!   which `notify_sent_sample` on their own schedule — never double-notify).
//!
//! `#[serial]` (second manager over one SHM root + a blocking-wait timing
//! assertion).
//!
//! # Running
//!
//! ```bash
//! cargo test -p cerulion_core --test ingress_publish_raw_notify_iox2_test -- --test-threads=1
//! ```

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use cerulion_core::clock::RealClock;
use cerulion_core::message::ShmMessage;
use cerulion_core::transport::{TransportConfig, TransportManager};
use cerulion_core::wire::{MaxSliceLen, WireHeader};
use cerulion_core::ReinjectOutcome;
use native_ros2_messages::geometry_msgs::Vector3;
use serial_test::serial;

/// Drain a raw event listener, returning one entry per NOTIFY.
///
/// iceoryx2 0.10 calls the drain callback once per DISTINCT event id, carrying
/// how many times that id was activated since the last drain; 0.9.1's socket
/// queue held one datagram per notify and was popped one at a time. Expanding
/// by `count` keeps these oracles counting notifies, which is what "did the
/// publish wake the listener" means here.
fn drain_notifies<S: iceoryx2::service::Service>(
    listener: &iceoryx2::port::listener::Listener<S>,
) -> Vec<iceoryx2::prelude::EventId> {
    let mut seen = Vec::new();
    let _ = listener.try_wait(|activation| {
        for _ in 0..activation.count {
            seen.push(activation.id);
        }
    });
    seen
}

const WAIT_TIMEOUT: Duration = Duration::from_secs(2);
const BOUNDED_WINDOW: Duration = Duration::from_millis(1000);
const SETTLE: Duration = Duration::from_millis(150);

/// Header-only, all-fixed Vector3 raw frame carrying a stamped `timestamp_ns`
/// (the hand oracle read back via `msg.header().timestamp_ns`).
fn build_raw_frame(sequence: u32, timestamp_ns: u64) -> Vec<u8> {
    let mut header = WireHeader::new(Vector3::SCHEMA_HASH, sequence, timestamp_ns);
    header.total_size = WireHeader::SIZE as u32;
    let mut buf = vec![0u8; WireHeader::SIZE];
    header.write_to_buf(&mut buf);
    buf
}

fn max_slice() -> MaxSliceLen {
    MaxSliceLen::try_new(1024).expect("1024 >= WireHeader::SIZE")
}

// ===========================================================================
// (A) headline: publish_raw on an ingress publisher wakes a blocked foreign
//     subscriber within a bounded window (hand oracle).
// ===========================================================================

#[test]
#[serial]
fn ingress_publish_raw_wakes_a_blocked_foreign_subscriber_within_a_bounded_window() {
    let mgr = TransportManager::init_for_test(
        TransportConfig {
            node_name: "ingress_pub_raw".into(),
            clock: Arc::new(RealClock),
            subscriber_buffer_size: 8,
            network: None,
        },
        cerulion_core::testing::iceoryx_test_config(),
    )
    .expect("per-test SHM transport");

    let topic = "/ingress/lowstate";
    let mut publisher = mgr
        .create_ingress_publisher(topic, max_slice())
        .expect("ingress publisher (the RawIngressRoute path)");

    // A SECOND manager on the SAME SHM root — the `topic hz` separate-process
    // shape (listener-full open-only subscriber).
    let foreign_mgr = TransportManager::init_for_test(
        TransportConfig {
            node_name: "ingress_topic_hz".into(),
            clock: Arc::new(RealClock),
            subscriber_buffer_size: 8,
            network: None,
        },
        mgr.iox_config(),
    )
    .expect("second manager on the same SHM root");

    let entered_wait = Arc::new(AtomicBool::new(false));
    let entered_t = Arc::clone(&entered_wait);
    let topic_t = topic.to_string();

    let handle = std::thread::spawn(move || {
        let sub = foreign_mgr
            .create_subscriber_open_only(&topic_t)
            .expect("foreign topic-hz subscriber attaches");
        // Drain any startup frames so the measured wait begins on an EMPTY queue.
        let _ = sub.wait_for_message(Duration::from_millis(1), |_m| {});
        entered_t.store(true, Ordering::Release);
        let start = Instant::now();
        let mut received_ts: Option<u64> = None;
        let _ = sub.wait_for_message(WAIT_TIMEOUT, |m| {
            received_ts = Some(m.header().timestamp_ns);
        });
        (received_ts, start.elapsed())
    });

    // Wait for the subscriber to block on an empty queue, then SETTLE so the
    // frame arrives DURING the blocking wait — the only way to get it is a WAKE.
    let deadline = Instant::now() + Duration::from_secs(5);
    while !entered_wait.load(Ordering::Acquire) {
        assert!(Instant::now() < deadline, "foreign subscriber never armed");
        std::thread::sleep(Duration::from_millis(2));
    }
    std::thread::sleep(SETTLE);

    const ORACLE_TS: u64 = 0x00CE_1845;
    let frame = build_raw_frame(0, ORACLE_TS);
    // publish_raw ALONE — no explicit notify anywhere (exactly RawIngressRoute).
    let recipients = publisher.publish_raw(&frame).expect("publish_raw");
    assert!(
        recipients >= 1,
        "publish_raw delivered to the connected foreign subscriber"
    );

    let (received_ts, elapsed) = handle.join().expect("foreign subscriber thread panicked");
    assert_eq!(
        received_ts,
        Some(ORACLE_TS),
        "the foreign subscriber must receive the raw-published frame (hand oracle \
         on stamped timestamp_ns)"
    );
    assert!(
        elapsed < BOUNDED_WINDOW,
        "the foreign subscriber must wake via publish_raw's own SentSample wake \
         within the bounded window ({:?}), not its own {:?} timeout — got {:?}. \
         With the fix reverted (publish_raw does not notify) this climbs to \
         ~{:?}.",
        BOUNDED_WINDOW,
        WAIT_TIMEOUT,
        elapsed,
        WAIT_TIMEOUT
    );
}

// ===========================================================================
// (B) deterministic core + scope control: ingress publish_raw fires a
//     SentSample; a NON-ingress publisher's publish_raw fires nothing.
// ===========================================================================

#[test]
#[serial]
fn ingress_publish_raw_notifies_but_a_plain_publisher_publish_raw_does_not() {
    let mgr = TransportManager::init_for_test(
        TransportConfig {
            node_name: "ingress_scope".into(),
            clock: Arc::new(RealClock),
            subscriber_buffer_size: 8,
            network: None,
        },
        cerulion_core::testing::iceoryx_test_config(),
    )
    .expect("per-test SHM transport");

    // --- INGRESS publisher: publish_raw MUST notify ---
    let ingress_topic = "/ingress/armed";
    let mut ingress = mgr
        .create_ingress_publisher(ingress_topic, max_slice())
        .expect("ingress publisher");
    let ingress_listener = mgr
        .create_trigger_listener_for_test(ingress_topic, mgr.default_topic_config())
        .expect("foreign listener on the ingress topic");
    drain_notifies(&ingress_listener);

    ingress
        .publish_raw(&build_raw_frame(0, 1))
        .expect("ingress publish_raw");
    assert_eq!(
        drain_notifies(&ingress_listener),
        vec![cerulion_core::transport::events::PubSubEvent::SentSample.into()],
        "ingress publish_raw MUST wake the foreign listener exactly once, with a \
         SentSample event"
    );

    // --- PLAIN publisher (create_publisher — the rmw/service/graph path):
    //     publish_raw must NOT auto-notify (scope control; no double-notify) ---
    let plain_topic = "/plain/unarmed";
    let mut plain = mgr
        .create_publisher(plain_topic, max_slice(), 0)
        .expect("plain publisher");
    let plain_listener = mgr
        .create_trigger_listener_for_test(plain_topic, mgr.default_topic_config())
        .expect("foreign listener on the plain topic");
    drain_notifies(&plain_listener);

    plain
        .publish_raw(&build_raw_frame(0, 2))
        .expect("plain publish_raw");
    assert!(
        drain_notifies(&plain_listener).is_empty(),
        "a NON-ingress publisher's publish_raw must NOT auto-notify — the flag \
         scopes the wake to create_ingress_publisher, so the rmw/service callers \
         (which notify_sent_sample on their own schedule) never double-notify"
    );

    drop(ingress_listener);
    drop(plain_listener);
}

// ===========================================================================
// (C) the wake through the REAL re-inject path. This change
//     REMOVED the explicit `notify_sent_sample` from `IngressInjector::
//     reinject_raw` (folded into the armed `publish_raw`). Every ingress/netd
//     delivery test receives via POLLED `try_receive`/`drain_owned`/`try_view`,
//     which read the SHM queue regardless of any notify — so a zero-wake
//     regression (a future reinject publisher minted via `create_publisher`
//     instead of `create_ingress_publisher`, or an early-return re-added to
//     `publish_raw`) would stay green everywhere. This pins the wake ON the
//     `reinject_raw` path with an EVENT-DRIVEN subscriber. Mutation-check:
//     disabling the `publish_raw` arm flag makes the subscriber wake only on its
//     own 2 s timeout (the network re-inject path is then silently notify-less).
// ===========================================================================

#[test]
#[serial]
fn reinject_raw_wakes_a_blocked_foreign_subscriber_within_a_bounded_window() {
    let mgr = TransportManager::init_for_test(
        TransportConfig {
            node_name: "reinject_raw_wake".into(),
            clock: Arc::new(RealClock),
            subscriber_buffer_size: 8,
            network: None,
        },
        cerulion_core::testing::iceoryx_test_config(),
    )
    .expect("per-test SHM transport");

    let topic = "/ingress/reinject";
    let schema_hash = Vector3::SCHEMA_HASH;
    // The REAL network re-inject seam (also the netd LAN/WAN mirror path). Works
    // on a `network: None` manager (no zenoh session).
    let injector = mgr
        .create_ingress_injector(topic, schema_hash, max_slice())
        .expect("ingress injector");

    let foreign_mgr = TransportManager::init_for_test(
        TransportConfig {
            node_name: "reinject_topic_hz".into(),
            clock: Arc::new(RealClock),
            subscriber_buffer_size: 8,
            network: None,
        },
        mgr.iox_config(),
    )
    .expect("second manager on the same SHM root");

    let entered_wait = Arc::new(AtomicBool::new(false));
    let entered_t = Arc::clone(&entered_wait);
    let topic_t = topic.to_string();

    let handle = std::thread::spawn(move || {
        let sub = foreign_mgr
            .create_subscriber_open_only(&topic_t)
            .expect("foreign subscriber attaches");
        let _ = sub.wait_for_message(Duration::from_millis(1), |_m| {});
        entered_t.store(true, Ordering::Release);
        let start = Instant::now();
        let mut received_ts: Option<u64> = None;
        let _ = sub.wait_for_message(WAIT_TIMEOUT, |m| {
            received_ts = Some(m.header().timestamp_ns);
        });
        (received_ts, start.elapsed())
    });

    let deadline = Instant::now() + Duration::from_secs(5);
    while !entered_wait.load(Ordering::Acquire) {
        assert!(Instant::now() < deadline, "foreign subscriber never armed");
        std::thread::sleep(Duration::from_millis(2));
    }
    std::thread::sleep(SETTLE);

    const ORACLE_TS: u64 = 0x00CE_1846;
    // reinject_raw validates the frame against `schema_hash`, then publish_raw's —
    // publish_raw fires the wake (the removed explicit notify is now folded in).
    let frame = build_raw_frame(0, ORACLE_TS);
    let outcome = injector.reinject_raw(&frame);
    let ReinjectOutcome::Injected { recipients } = outcome else {
        panic!("expected Injected, got {outcome:?}");
    };
    assert!(
        recipients >= 1,
        "reinject_raw delivered to the foreign subscriber"
    );

    let (received_ts, elapsed) = handle.join().expect("foreign subscriber thread panicked");
    assert_eq!(
        received_ts,
        Some(ORACLE_TS),
        "the foreign subscriber must receive the re-injected frame (hand oracle)"
    );
    assert!(
        elapsed < BOUNDED_WINDOW,
        "the foreign subscriber must wake via reinject_raw's publish_raw notify \
         within the bounded window ({:?}), not its own {:?} timeout — got {:?}. \
         With the arm flag disabled this climbs to ~{:?} (the network re-inject \
         path silently stops waking listener-driven consumers).",
        BOUNDED_WINDOW,
        WAIT_TIMEOUT,
        elapsed,
        WAIT_TIMEOUT
    );
}
