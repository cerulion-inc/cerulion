// SPDX-License-Identifier: AGPL-3.0-only
//! iceoryx2-direct integration tests for
//! adaptive `loan_proxy` sizing.
//!
//! The bulk of the adaptive-sizing coverage lives in `adaptive_sizing_test.rs`,
//! which runs the SAME iceoryx2 publishers over isolated per-test
//! `TestTransport` SHM roots (parallel-safe). This file mirrors a handful
//! of load-bearing scenarios on the process-global singleton
//! `TransportManager` instead, to pin the `record_payload_size` callback
//! wiring on the `OutputProxy::Drop` branch and to verify
//! `CerulionPublisher::sizer` warms + shrinks the same way there.
//!
//! # Running
//!
//! ```bash
//! cargo test -p cerulion_core --test adaptive_sizing_iceoryx2_test -- --test-threads=1
//! ```
//!
//! `--test-threads=1` is required because iceoryx2 is a singleton
//! per process (Principle #8).

use std::sync::atomic::{AtomicU64, Ordering};
use std::time::SystemTime;

use cerulion_core::transport::adaptive_sizer::WINDOW_SIZE;
use cerulion_core::transport::TransportManager;
use cerulion_core::wire::MaxSliceLen;
use native_ros2_messages::geometry_msgs::Vector3;

static TOPIC_COUNTER: AtomicU64 = AtomicU64::new(0);

fn unique_topic(base: &str) -> String {
    let nanos = SystemTime::now()
        .duration_since(SystemTime::UNIX_EPOCH)
        .unwrap()
        .as_nanos();
    let id = TOPIC_COUNTER.fetch_add(1, Ordering::Relaxed);
    format!("test/adaptive_iox2/{base}/{nanos}/{id}")
}

#[test]
fn iox2_first_tick_uses_max_slice_len() {
    let mgr = TransportManager::get_or_init().expect("init");
    let topic = unique_topic("first_tick");
    let publisher = mgr
        .create_publisher_simple(&topic, MaxSliceLen::const_new(16 * 1024 * 1024))
        .expect("create publisher");

    // Cold publisher: adaptive loan size == max_slice_len.
    assert!(!publisher.sizer_warm());
    assert_eq!(
        publisher.adaptive_loan_size_for_min_required(40),
        16 * 1024 * 1024
    );
}

#[test]
fn iox2_warms_after_window_size_publishes() {
    let mgr = TransportManager::get_or_init().expect("init");
    let topic = unique_topic("warms");
    let mut publisher = mgr
        .create_publisher_simple(&topic, MaxSliceLen::const_new(16 * 1024 * 1024))
        .expect("create publisher");

    // WINDOW_SIZE successful Vector3 publishes — each Drop calls
    // record_payload_size on the iceoryx2 publisher branch. After
    // WINDOW_SIZE records the publisher is warm.
    for _ in 0..WINDOW_SIZE {
        let mut proxy = publisher.loan_proxy::<Vector3>().expect("loan_proxy");
        proxy.x = 1.0;
        proxy.y = 2.0;
        proxy.z = 3.0;
    }
    assert!(
        publisher.sizer_warm(),
        "iceoryx2 publisher must warm after WINDOW_SIZE drops — proves \
         OutputProxy::Drop's iceoryx2 branch calls record_payload_size"
    );
    // Vector3 wire frame = 32 (header) + 24 (3× f64) = 56. Warm loan
    // = 56 × 1.5 = 84.
    let warm_loan = publisher.adaptive_loan_size_for_min_required(40);
    assert!(
        warm_loan < 16 * 1024,
        "warm Vector3 iceoryx2 loan should be ≪ 16 KiB; got {warm_loan}"
    );
}

#[test]
fn iox2_max_slice_len_accessor_unchanged_post_warmup() {
    // Mirror of the in-process version: iceoryx2 publisher's
    // configured max_slice_len ceiling must not be rewritten by the
    // adaptive logic.
    let mgr = TransportManager::get_or_init().expect("init");
    let topic = unique_topic("max_unchanged");
    let mut publisher = mgr
        .create_publisher_simple(&topic, MaxSliceLen::const_new(16 * 1024 * 1024))
        .expect("create publisher");
    assert_eq!(publisher.max_slice_len().get(), 16 * 1024 * 1024);
    for _ in 0..WINDOW_SIZE {
        let mut proxy = publisher.loan_proxy::<Vector3>().expect("loan_proxy");
        proxy.x = 1.0;
        proxy.y = 0.0;
        proxy.z = 0.0;
    }
    assert!(publisher.sizer_warm());
    assert_eq!(
        publisher.max_slice_len().get(),
        16 * 1024 * 1024,
        "max_slice_len() accessor must remain the configured ceiling, \
         not the adaptive loan size"
    );
}

#[test]
fn iox2_adaptive_payload_round_trips_byte_for_byte() {
    // After warmup the adaptive loan is much smaller than
    // max_slice_len — verify subscribers still receive the published
    // payload byte-for-byte. This is the determinism contract: the
    // smaller adaptive loan does NOT corrupt wire frames.
    let mgr = TransportManager::get_or_init().expect("init");
    let topic = unique_topic("round_trip_warm");
    let mut publisher = mgr
        .create_publisher_simple(&topic, MaxSliceLen::const_new(16 * 1024 * 1024))
        .expect("create publisher");
    let mut subscriber = mgr.create_subscriber(&topic).expect("create subscriber");

    // Warm the publisher with WINDOW_SIZE Vector3 publishes.
    for _ in 0..WINDOW_SIZE {
        let mut proxy = publisher.loan_proxy::<Vector3>().expect("loan_proxy");
        proxy.x = 0.0;
        proxy.y = 0.0;
        proxy.z = 0.0;
    }
    assert!(publisher.sizer_warm());

    // Drain subscriber to clear warmup frames.
    let _ = subscriber.try_view::<Vector3, _>(|_| ()).expect("try_view");

    // Publish canary values through the (now-adaptive-sized) loan path.
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
