// SPDX-License-Identifier: AGPL-3.0-only
//! Integration tests for the iceoryx2 transport (rewrite).
//!
//! Exercises the SHM-backed `loan_proxy` / `try_view` round-trip on real
//! iceoryx2 ports. These tests share the global iceoryx2 singleton, so the
//! parent `cargo test` invocation must run with `--test-threads=1`.
//!
//! # Test isolation
//!
//! Each test uses a unique topic name (timestamped + counter) so parallel
//! test runs and the global singleton don't interfere.
//!
//! # Dropped tests vs. the legacy file
//!
//! The legacy file ran 21 tests against the deleted heap-pub/sub API
//! (`publish<M>` / `wait_for_message` / `try_receive` / `wait_for_typed_message`).
//! Coverage that survives semantically is folded into the SHM-backed shape
//! below (round-trip, fan-out, isolation, recreate-publisher, schema mismatch).
//! Specifically dropped:
//!
//! - `test_message_stamping`, `test_publish_receive_large_message`,
//!   `test_receive_zero_allocations_*`, `test_payload_byte_correctness`,
//!   `test_event_based_receive`, `test_total_size_field_validation` — the
//!   notion of "user inspects WireHeader fields off the receive path" is
//!   gone with the SHM API. The wire header is an internal contract again
//!   (verified in `wire_test.rs`); user-facing reads are typed accessors.
//! - `test_drain_multiple_messages` — `try_view` is latest-wins;
//!   bulk drain is no longer a transport-level concern.
//! - `test_subscriber_buffer_overflow`, `test_buffer_exhaustion`,
//!   `test_backpressure_no_panic` — these poked at the `try_receive` /
//!   sequence-gap behaviour of the deleted heap-pub/sub API. Buffer sizing
//!   is now a publisher concern (`max_slice_len`); we only assert the
//!   `loan_proxy` error variants in `output_proxy_test.rs`.
//! - `test_real_clock_integration`, `test_wire_header_alignment_safe_roundtrip`
//!   — these belong to clock_test / wire_test respectively, not transport.
//! - `test_publisher_accessors` — `topic()` and `sequence()` are exercised
//!   incidentally by the round-trip tests below.
//! - `test_schema_validation_accepts_match` — duplicates the round-trip
//!   tests below.

use cerulion_core::wire::{MaxSliceLen, WireHeader};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{Duration, SystemTime};

use cerulion_core::error::TransportError;
use cerulion_core::transport::publisher::CerulionPublisher;
use cerulion_core::transport::subscriber::CerulionSubscriber;
use cerulion_core::transport::TransportManager;
use native_ros2_messages::geometry_msgs::{Point, Vector3};

static TOPIC_COUNTER: AtomicU64 = AtomicU64::new(0);

fn unique_topic(base: &str) -> String {
    let nanos = SystemTime::now()
        .duration_since(SystemTime::UNIX_EPOCH)
        .unwrap()
        .as_nanos();
    let id = TOPIC_COUNTER.fetch_add(1, Ordering::Relaxed);
    format!("test/transport/{base}/{nanos}/{id}")
}

/// Create a publisher + subscriber pair on a unique topic with a 256-byte
/// buffer (room for the 40-byte WireHeader plus a fixed 24-byte payload).
fn setup_pubsub(base: &str) -> (CerulionPublisher, CerulionSubscriber) {
    let mgr = TransportManager::get_or_init().expect("init");
    let topic = unique_topic(base);
    let publisher = mgr
        .create_publisher_simple(&topic, MaxSliceLen::const_new(256))
        .expect("create publisher");
    let subscriber = mgr.create_subscriber(&topic).expect("create subscriber");
    (publisher, subscriber)
}

// ============================================================
// TransportManager singleton
// ============================================================

#[test]
fn test_transport_manager_singleton() {
    let mgr1 = TransportManager::get_or_init().expect("init should succeed");
    let mgr2 = TransportManager::get().expect("get should succeed after init");
    assert!(
        Arc::ptr_eq(&mgr1, &mgr2),
        "singleton should return same Arc",
    );
}

// ============================================================
// Round-trip via real iceoryx2 (loan_proxy + Drop publish + try_view)
// ============================================================

#[test]
fn test_round_trip_fixed_schema() {
    let (mut publisher, mut subscriber) = setup_pubsub("round_trip_fixed");

    {
        let mut proxy = publisher.loan_proxy::<Vector3>().expect("loan_proxy");
        proxy.x = 1.5;
        proxy.y = 2.5;
        proxy.z = -3.5;
    }

    let observed = subscriber
        .try_view::<Vector3, _>(|view| (view.x, view.y, view.z))
        .expect("try_view")
        .expect("subscriber should see the published frame");
    assert!((observed.0 - 1.5).abs() < f64::EPSILON);
    assert!((observed.1 - 2.5).abs() < f64::EPSILON);
    assert!((observed.2 - -3.5).abs() < f64::EPSILON);
}

// ============================================================
// Multiple subscribers on the same topic (fan-out)
// ============================================================

#[test]
fn test_multiple_subscribers_fan_out() {
    let mgr = TransportManager::get_or_init().expect("init");
    let topic = unique_topic("fan_out");

    let mut publisher = mgr
        .create_publisher_simple(&topic, MaxSliceLen::const_new(256))
        .expect("create publisher");
    let mut sub_a = mgr.create_subscriber(&topic).expect("create sub_a");
    let mut sub_b = mgr.create_subscriber(&topic).expect("create sub_b");

    {
        let mut proxy = publisher.loan_proxy::<Vector3>().expect("loan_proxy");
        proxy.x = 42.0;
        proxy.y = 7.0;
        proxy.z = -1.0;
    }

    std::thread::sleep(Duration::from_millis(50));

    let observed_a = sub_a
        .try_view::<Vector3, _>(|view| (view.x, view.y, view.z))
        .expect("sub_a try_view")
        .expect("sub_a should see the frame");
    let observed_b = sub_b
        .try_view::<Vector3, _>(|view| (view.x, view.y, view.z))
        .expect("sub_b try_view")
        .expect("sub_b should see the frame");

    assert_eq!(observed_a, (42.0, 7.0, -1.0));
    assert_eq!(observed_b, (42.0, 7.0, -1.0));
}

// ============================================================
// Subscriber not yet attached: publish does not block / fail
// ============================================================

/// A subscriber that connects after the publisher already loaned + dropped
/// a proxy must not panic. With history disabled the late joiner sees no
/// past frames; a fresh publish after attach is observable.
#[test]
fn test_subscriber_not_yet_attached() {
    let mgr = TransportManager::get_or_init().expect("init");
    let topic = unique_topic("late_attach");

    let mut publisher = mgr
        .create_publisher_simple(&topic, MaxSliceLen::const_new(256))
        .expect("create publisher");

    // Publish before any subscriber exists.
    {
        let mut proxy = publisher.loan_proxy::<Vector3>().expect("loan");
        proxy.x = 1.0;
    }

    // Now attach.
    let mut subscriber = mgr.create_subscriber(&topic).expect("create subscriber");

    // try_view immediately: subscriber missed the first publish (no history),
    // so try_view returns Ok(None). Crucially, no panic.
    let pre_attach = subscriber
        .try_view::<Vector3, _>(|_view| panic!("no frame should be available yet"))
        .expect("try_view");
    assert!(
        pre_attach.is_none(),
        "subscriber attached after publish must not see the historical frame",
    );

    // A fresh publish should be observable.
    {
        let mut proxy = publisher.loan_proxy::<Vector3>().expect("loan");
        proxy.x = 7.5;
    }
    std::thread::sleep(Duration::from_millis(50));

    let observed = subscriber
        .try_view::<Vector3, _>(|view| view.x)
        .expect("try_view")
        .expect("subscriber should see the post-attach publish");
    assert_eq!(observed, 7.5);
}

// ============================================================
// Topic isolation: different topics don't cross-talk
// ============================================================

#[test]
fn test_topic_isolation() {
    let mgr = TransportManager::get_or_init().expect("init");
    let topic_a = unique_topic("iso_a");
    let topic_b = unique_topic("iso_b");

    let mut pub_a = mgr
        .create_publisher_simple(&topic_a, MaxSliceLen::const_new(256))
        .expect("create pub_a");
    let mut sub_a = mgr.create_subscriber(&topic_a).expect("create sub_a");
    let mut sub_b = mgr.create_subscriber(&topic_b).expect("create sub_b");

    {
        let mut proxy = pub_a.loan_proxy::<Vector3>().expect("loan");
        proxy.x = 99.0;
    }
    std::thread::sleep(Duration::from_millis(50));

    let observed_a = sub_a
        .try_view::<Vector3, _>(|view| view.x)
        .expect("sub_a try_view")
        .expect("sub_a should receive its own topic's frame");
    assert_eq!(observed_a, 99.0);

    let observed_b = sub_b
        .try_view::<Vector3, _>(|_view| panic!("sub_b must not see topic_a's frame"))
        .expect("sub_b try_view");
    assert!(
        observed_b.is_none(),
        "sub_b on a different topic must not receive cross-topic frames",
    );
}

// ============================================================
// Singleton manager survives multiple create_publisher calls
// ============================================================

/// Repeatedly create publishers through the singleton. Each must succeed
/// and operate independently — no shared-state corruption across creations.
#[test]
fn test_singleton_manager_handles_multiple_create_publisher() {
    let mgr = TransportManager::get_or_init().expect("init");

    // Three distinct topics, each gets its own pub/sub pair through the
    // singleton manager.
    let mut pubs_subs: Vec<(CerulionPublisher, CerulionSubscriber, f64)> = Vec::new();
    for (i, base) in ["multi_pub_0", "multi_pub_1", "multi_pub_2"]
        .iter()
        .enumerate()
    {
        let topic = unique_topic(base);
        let publisher = mgr
            .create_publisher_simple(&topic, MaxSliceLen::const_new(256))
            .expect("create publisher via singleton");
        let subscriber = mgr.create_subscriber(&topic).expect("create subscriber");
        pubs_subs.push((publisher, subscriber, (i + 1) as f64));
    }

    // Publish a unique value on each topic.
    for (publisher, _, value) in pubs_subs.iter_mut() {
        let mut proxy = publisher.loan_proxy::<Vector3>().expect("loan");
        proxy.x = *value;
    }
    std::thread::sleep(Duration::from_millis(50));

    // Each subscriber should see its own publisher's value (no crosstalk).
    for (_, subscriber, expected) in pubs_subs.iter_mut() {
        let observed = subscriber
            .try_view::<Vector3, _>(|view| view.x)
            .expect("try_view")
            .expect("subscriber should see its publisher's frame");
        assert_eq!(
            observed, *expected,
            "singleton-vended pub/sub pair must isolate per topic",
        );
    }
}

// ============================================================
// Schema mismatch is detected on the receive path
// ============================================================

/// Publish a `Vector3` (24-byte fixed schema) and read as `Point`. Both are
/// 24 bytes wide but have different `SCHEMA_HASH` constants — the
/// subscriber must reject the wire frame with `TransportError::SchemaMismatch`.
#[test]
fn test_schema_mismatch_rejected_on_try_view() {
    let mgr = TransportManager::get_or_init().expect("init");
    let topic = unique_topic("schema_mismatch");

    let mut publisher = mgr
        .create_publisher_simple(&topic, MaxSliceLen::const_new(256))
        .expect("create publisher");
    let mut subscriber = mgr.create_subscriber(&topic).expect("create subscriber");

    {
        let mut proxy = publisher.loan_proxy::<Vector3>().expect("loan");
        proxy.x = 1.0;
        proxy.y = 2.0;
        proxy.z = 3.0;
    }
    std::thread::sleep(Duration::from_millis(50));

    match subscriber.try_view::<Point, _>(|_view| ()) {
        Err(TransportError::SchemaMismatch { .. }) => {}
        other => panic!("expected SchemaMismatch, got {other:?}"),
    }
}

// ============================================================
// Owned one-frame receive (the rmw loaned-take surface)
// ============================================================

/// `try_receive_one_owned` — round trip, FIFO order, HOLD-across-publish
/// byte stability, and the LOUD bounded borrow-budget exhaustion. The
/// service is created loan-ready via the create-leg borrow floor
/// (`create_borrow_floor = Some(4)`), so the hand oracle for the budget is
/// exact on BOTH sides: 4 concurrent holds succeed, the 5th receive fails
/// with iceoryx2's `ExceedsMaxBorrows`, and dropping one held sample
/// recovers the next receive.
#[test]
fn test_try_receive_one_owned_hold_and_budget() {
    use cerulion_core::wire::WireHeader;

    let mgr = TransportManager::get_or_init().expect("init");
    let topic = unique_topic("owned_take");

    // Publisher-first create with the floor: the service is born at
    // subscriber_max_borrowed_samples = 4.
    let mut cfg = mgr.default_topic_config();
    cfg.create_borrow_floor = Some(4);
    let mut publisher = mgr
        .create_publisher_with_topic_config(&topic, MaxSliceLen::const_new(256), 0, cfg)
        .expect("create publisher with borrow floor");
    let subscriber = mgr.create_subscriber(&topic).expect("create subscriber");

    // Publish 5 frames, x = 1.0..=5.0 (the FIFO oracle).
    for i in 1..=5u32 {
        let mut proxy = publisher.loan_proxy::<Vector3>().expect("loan_proxy");
        proxy.x = f64::from(i);
        proxy.y = 0.0;
        proxy.z = -f64::from(i);
    }
    std::thread::sleep(Duration::from_millis(50));

    // First owned take: frame 1, held. Full-frame shape: 40-byte header +
    // 24-byte Vector3 fixed section, seq 0.
    let held = subscriber
        .try_receive_one_owned()
        .expect("owned receive")
        .expect("frame 1 must be queued");
    let header = held.wire_header().expect("held frame header");
    assert_eq!(header.sequence, 0, "FIFO: first published frame first");
    assert_eq!(held.payload().len(), WireHeader::SIZE + 24);
    let x = f64::from_le_bytes(
        held.payload()[WireHeader::SIZE..WireHeader::SIZE + 8]
            .try_into()
            .unwrap(),
    );
    assert_eq!(x, 1.0);

    // HOLD across further publishes: the held bytes must be UNCHANGED —
    // iceoryx2 borrow accounting pins the SHM slot while the sample lives,
    // so no later publish can reclaim/overwrite it.
    let snapshot: Vec<u8> = held.payload().to_vec();
    for i in 6..=7u32 {
        let mut proxy = publisher.loan_proxy::<Vector3>().expect("loan_proxy");
        proxy.x = f64::from(i);
        proxy.y = 0.0;
        proxy.z = -f64::from(i);
    }
    std::thread::sleep(Duration::from_millis(50));
    assert_eq!(
        held.payload(),
        snapshot.as_slice(),
        "held sample bytes must not change while later frames publish"
    );

    // Hold three more (4 total = the provisioned budget), values 2..=4.
    let mut more = Vec::new();
    for expect_x in 2..=4u32 {
        let s = subscriber
            .try_receive_one_owned()
            .expect("owned receive")
            .expect("queued frame");
        let x = f64::from_le_bytes(
            s.payload()[WireHeader::SIZE..WireHeader::SIZE + 8]
                .try_into()
                .unwrap(),
        );
        assert_eq!(x, f64::from(expect_x), "FIFO order");
        more.push(s);
    }

    // 5th concurrent borrow: LOUD bounded failure, never silent loss and
    // never a hang — the queue still holds frames 5..=7.
    let err = match subscriber.try_receive_one_owned() {
        Err(e) => e,
        Ok(_) => panic!("a receive past the borrow budget must fail"),
    };
    match &err {
        TransportError::Receive { reason, .. } => assert!(
            reason.contains("ExceedsMaxBorrows"),
            "the failure must name the borrow budget: {reason}"
        ),
        other => panic!("expected TransportError::Receive, got {other:?}"),
    }

    // Returning ONE loan recovers the next take, still in FIFO order.
    drop(more.pop().expect("held sample"));
    let next = subscriber
        .try_receive_one_owned()
        .expect("recovered receive")
        .expect("frame 5 must still be queued");
    let x = f64::from_le_bytes(
        next.payload()[WireHeader::SIZE..WireHeader::SIZE + 8]
            .try_into()
            .unwrap(),
    );
    assert_eq!(x, 5.0, "no frame was lost to the refused receive");

    // Drain the rest + empty.
    drop(next);
    drop(held);
    drop(more);
    for expect_x in 6..=7u32 {
        let s = subscriber
            .try_receive_one_owned()
            .expect("owned receive")
            .expect("queued frame");
        let x = f64::from_le_bytes(
            s.payload()[WireHeader::SIZE..WireHeader::SIZE + 8]
                .try_into()
                .unwrap(),
        );
        assert_eq!(x, f64::from(expect_x));
    }
    assert!(
        subscriber
            .try_receive_one_owned()
            .expect("owned receive")
            .is_none(),
        "drained queue must serve None"
    );
}

/// The create-leg port floors size the service a floored opener CREATES,
/// data and event halves: with floors of 6 publishers and 14 subscribers,
/// exactly 6 publishers and 14 subscribers attach (default openers included)
/// and the next of each is refused. 6 + 14 = 20 data ports is past
/// iceoryx2's event create default of 16 notifiers/listeners, so the 20th
/// attach also proves the event service was created for every data port.
#[test]
fn test_create_port_floors_size_the_created_service() {
    let mgr = TransportManager::get_or_init().expect("init");
    let topic = unique_topic("port_floors");
    let mut cfg = mgr.default_topic_config();
    cfg.create_publisher_floor = Some(6);
    cfg.create_subscriber_floor = Some(14);
    let msl = MaxSliceLen::const_new(256);

    let mut publishers = vec![mgr
        .create_publisher_with_topic_config(&topic, msl, 0, cfg)
        .expect("floored publisher creates the service")];
    for i in 1..6 {
        publishers.push(
            mgr.create_publisher(&topic, msl, 0)
                .unwrap_or_else(|e| panic!("publisher {i} of 6 must attach: {e}")),
        );
    }
    let mut subscribers = Vec::new();
    for i in 0..14 {
        subscribers.push(
            mgr.create_subscriber(&topic)
                .unwrap_or_else(|e| panic!("subscriber {i} of 14 must attach: {e}")),
        );
    }
    let refused = match mgr.create_publisher(&topic, msl, 0) {
        Err(e) => e.to_string(),
        Ok(_) => panic!("a 7th publisher must be refused: the service has 6 slots"),
    };
    assert!(
        refused.contains("ExceedsMaxSupportedPublishers"),
        "refusal names the publisher cap: {refused}"
    );
    let refused = match mgr.create_subscriber(&topic) {
        Err(e) => e.to_string(),
        Ok(_) => panic!("a 15th subscriber must be refused: the service has 14 slots"),
    };
    assert!(
        refused.contains("ExceedsMaxSupportedSubscribers"),
        "refusal names the subscriber cap: {refused}"
    );

    // One frame from the last publisher reaches every subscriber.
    {
        let mut proxy = publishers[5].loan_proxy::<Vector3>().expect("loan_proxy");
        proxy.x = 7.0;
        proxy.y = 0.0;
        proxy.z = 0.0;
    }
    std::thread::sleep(Duration::from_millis(50));
    for (i, sub) in subscribers.iter().enumerate() {
        let sample = sub
            .try_receive_one_owned()
            .expect("owned receive")
            .unwrap_or_else(|| panic!("subscriber {i} must receive the frame"));
        let x = f64::from_le_bytes(
            sample.payload()[WireHeader::SIZE..WireHeader::SIZE + 8]
                .try_into()
                .unwrap(),
        );
        assert_eq!(x, 7.0, "subscriber {i} payload");
    }
}

/// The port floors arm no open requirement: a floored opener ATTACHES to a
/// service a default opener created first (iceoryx2's 2 publishers), and
/// that service's caps then bind (2 publishers attach, the 3rd is refused).
/// If a floor armed the open leg, the floored create here would fail with
/// `DoesNotSupportRequestedAmountOfPublishers`.
#[test]
fn test_create_port_floors_open_leg_tolerates_smaller_service() {
    let mgr = TransportManager::get_or_init().expect("init");
    let topic = unique_topic("port_floors_open");
    let _subscriber = mgr
        .create_subscriber(&topic)
        .expect("default opener creates");
    let mut cfg = mgr.default_topic_config();
    cfg.create_publisher_floor = Some(32);
    cfg.create_subscriber_floor = Some(32);
    let msl = MaxSliceLen::const_new(256);
    let defaults = &iceoryx2::config::Config::global_config()
        .defaults
        .publish_subscribe;
    let mut publishers = Vec::new();
    for i in 0..defaults.max_publishers {
        publishers.push(
            mgr.create_publisher_with_topic_config(&topic, msl, 0, cfg)
                .unwrap_or_else(|e| {
                    panic!("floored publisher {i} must attach to the smaller service: {e}")
                }),
        );
    }
    let refused = match mgr.create_publisher_with_topic_config(&topic, msl, 0, cfg) {
        Err(e) => e.to_string(),
        Ok(_) => panic!("the pre-existing service's publisher cap binds"),
    };
    assert!(
        refused.contains("ExceedsMaxSupportedPublishers"),
        "refusal names the publisher cap: {refused}"
    );
}

/// A service an rmw-style floored opener created (32 publishers, 32
/// subscribers) must still open for a graph that OWNS the topic. That graph
/// arms `publishers + subscribers + extra_event_listeners` on the event
/// service from the live data caps; with a unified data-trigger input
/// (`extra_event_listeners > 0`) that is past `32 + 32`, so an event service
/// created at exactly one port per data slot would refuse the graph.
#[test]
fn test_create_port_floors_leave_room_for_an_owning_graph() {
    use cerulion_core::transport::{PublisherProvisioning, TopicServiceConfig};
    let mgr = TransportManager::get_or_init().expect("init");
    let topic = unique_topic("port_floors_owner");
    let mut floored = mgr.default_topic_config();
    floored.create_publisher_floor = Some(32);
    floored.create_subscriber_floor = Some(32);
    let rmw_side = mgr
        .create_subscriber_with_buffers(&topic, floored, floored.subscriber_max_buffer_size)
        .expect("floored subscriber creates the service");

    let in_graph_subscribers = 3;
    let unified_inputs = 3;
    let owned = TopicServiceConfig::for_topology(
        mgr.default_topic_config(),
        0,
        in_graph_subscribers,
        PublisherProvisioning::SingleWriter,
        0,
        unified_inputs,
    );
    let graph_side = mgr
        .create_subscriber_with_buffers(&topic, owned, owned.subscriber_max_buffer_size)
        .expect("an owning graph's open must pass the event requirement on a floored service");
    drop(graph_side);
    drop(rmw_side);
}

/// The create floors are checked at the choke point like the port caps: zero,
/// a value past `MAX_REASONABLE_PORTS`, and a pair whose event service
/// (`publishers + 2 x subscribers`) would pass it are refused, naming the
/// field.
#[test]
fn test_create_port_floors_out_of_range_are_refused() {
    use cerulion_core::transport::MAX_REASONABLE_PORTS;
    let mgr = TransportManager::get_or_init().expect("init");
    let msl = MaxSliceLen::const_new(256);
    let cases: [(Option<usize>, Option<usize>, &str); 4] = [
        (Some(0), None, "create_publisher_floor"),
        (
            None,
            Some(MAX_REASONABLE_PORTS + 1),
            "create_subscriber_floor",
        ),
        (
            Some(MAX_REASONABLE_PORTS / 2),
            Some(MAX_REASONABLE_PORTS / 2),
            "2 x create_subscriber_floor",
        ),
        (None, Some(0), "create_subscriber_floor"),
    ];
    for (i, (pubs, subs, named)) in cases.into_iter().enumerate() {
        let mut cfg = mgr.default_topic_config();
        cfg.create_publisher_floor = pubs;
        cfg.create_subscriber_floor = subs;
        let topic = unique_topic(&format!("port_floors_bad_{i}"));
        let err = match mgr.create_publisher_with_topic_config(&topic, msl, 0, cfg) {
            Err(e) => e.to_string(),
            Ok(_) => panic!("case {i}: floors {pubs:?}/{subs:?} must be refused"),
        };
        assert!(
            err.contains(named),
            "case {i}: the refusal names {named}: {err}"
        );
    }
}

/// The create-leg borrow floor NEVER arms an open
/// requirement — a floored publisher create must ATTACH to a pre-existing
/// service born at the iceoryx2 default (2), and the effective loan budget
/// is then that smaller created value (2 holds OK, the 3rd refused, drop
/// one recovers). The mutation kill: if the floor armed the open leg, the
/// publisher create here errors with
/// `DoesNotSupportRequestedMinSubscriberBorrowedSamples`.
#[test]
fn test_borrow_floor_open_leg_tolerates_smaller_service() {
    let mgr = TransportManager::get_or_init().expect("init");
    let topic = unique_topic("owned_take_degraded");

    // Subscriber-first: a default opener CREATES the service at the
    // iceoryx2 default borrow (2).
    let subscriber = mgr.create_subscriber(&topic).expect("create subscriber");

    // Floored publisher create must still attach (open leg tolerant).
    let mut cfg = mgr.default_topic_config();
    cfg.create_borrow_floor = Some(4);
    let mut publisher = mgr
        .create_publisher_with_topic_config(&topic, MaxSliceLen::const_new(256), 0, cfg)
        .expect("floored publisher must open a pre-existing borrow-2 service");

    for i in 1..=3u32 {
        let mut proxy = publisher.loan_proxy::<Vector3>().expect("loan_proxy");
        proxy.x = f64::from(i);
        proxy.y = 0.0;
        proxy.z = 0.0;
    }
    std::thread::sleep(Duration::from_millis(50));

    // The budget really is the pre-existing 2: two holds, then refusal.
    let a = subscriber
        .try_receive_one_owned()
        .expect("owned receive")
        .expect("frame 1");
    let b = subscriber
        .try_receive_one_owned()
        .expect("owned receive")
        .expect("frame 2");
    let err = match subscriber.try_receive_one_owned() {
        Err(e) => e,
        Ok(_) => panic!("third concurrent borrow must exceed the created budget of 2"),
    };
    match &err {
        TransportError::Receive { reason, .. } => assert!(
            reason.contains("ExceedsMaxBorrows"),
            "must name the borrow budget: {reason}"
        ),
        other => panic!("expected TransportError::Receive, got {other:?}"),
    }
    drop(a);
    let c = subscriber
        .try_receive_one_owned()
        .expect("recovered receive")
        .expect("frame 3");
    let x = f64::from_le_bytes(
        c.payload()[WireHeader::SIZE..WireHeader::SIZE + 8]
            .try_into()
            .unwrap(),
    );
    assert_eq!(x, 3.0);
    drop(b);
    drop(c);
}
