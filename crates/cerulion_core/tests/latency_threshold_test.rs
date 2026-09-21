// SPDX-License-Identifier: AGPL-3.0-only
//! Release-mode latency threshold tests (SHM-backed API rewrite).
//!
//! Catches catastrophic regressions (e.g., accidental memcpy) with generous
//! thresholds that accommodate CI VM noise. Uses `Instant::now()` for timing.
//!
//! These tests target the SHM-backed `loan_proxy` / `try_view` round trip
//! that replaced the legacy `publish<M>` / `wait_for_message` flow.
//!
//! # Running
//!
//! ```bash
//! cargo test -p cerulion_core --test latency_threshold_test --release \
//!   -- --nocapture --test-threads=1
//! ```
//!
//! Must run with `--release` (debug builds are 10-50x slower) and
//! `--test-threads=1` (iceoryx2 singleton + shared memory requires serial
//! access). The tests are gated behind `cfg(not(debug_assertions))` so a
//! plain `cargo test` (no --release) is a no-op for this file rather
//! than a noisy failure on debug-build latencies.

#![cfg(not(debug_assertions))]

use cerulion_core::wire::MaxSliceLen;
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::{Duration, Instant, SystemTime};

use cerulion_core::transport::TransportManager;
use cerulion_core::wire::WireHeader;
use native_ros2_messages::builtin_interfaces::Time;
use native_ros2_messages::geometry_msgs::Point;
use native_ros2_messages::sensor_msgs::Image;

/// Monotonic counter guaranteeing unique topics even within the same nanosecond.
static TOPIC_COUNTER: AtomicU64 = AtomicU64::new(0);

/// Generate a unique topic name for test isolation.
fn unique_topic(base: &str) -> String {
    let nanos = SystemTime::now()
        .duration_since(SystemTime::UNIX_EPOCH)
        .unwrap()
        .as_nanos();
    let id = TOPIC_COUNTER.fetch_add(1, Ordering::Relaxed);
    format!("test/lat/{}/{}/{}", base, nanos, id)
}

const WARMUP_ITERS: usize = 100;
const MEASURE_ITERS: usize = 500;

/// Compute median of a sorted slice.
fn median(sorted: &[f64]) -> f64 {
    let n = sorted.len();
    if n.is_multiple_of(2) {
        (sorted[n / 2 - 1] + sorted[n / 2]) / 2.0
    } else {
        sorted[n / 2]
    }
}

/// Spin until `try_view` yields a sample, then return.
///
/// Hot loop: `try_view` is non-blocking (drain-keep-latest). On `Ok(None)`
/// we spin and try again, bounded by `timeout`.
fn spin_recv_one<T, F>(
    sub: &mut cerulion_core::transport::subscriber::CerulionSubscriber,
    timeout: Duration,
    mut callback: F,
) where
    T: cerulion_core::message::ShmMessage,
    F: FnMut(),
{
    let deadline = Instant::now() + timeout;
    loop {
        match sub.try_view::<T, _>(|_view| ()) {
            Ok(Some(())) => {
                callback();
                return;
            }
            Ok(None) => {
                if Instant::now() > deadline {
                    panic!("spin_recv_one: timed out waiting for sample");
                }
                std::hint::spin_loop();
            }
            Err(e) => panic!("spin_recv_one error: {e:?}"),
        }
    }
}

// ============================================================
// Test 1: Small message (24B Point) median latency < 500µs
// ============================================================

#[test]
fn test_small_message_latency() {
    let mgr = TransportManager::get_or_init().expect("init");
    let topic = unique_topic("small_latency");

    let mut publisher = mgr
        .create_publisher_simple(&topic, MaxSliceLen::const_new(1024))
        .expect("create pub");
    let mut subscriber = mgr.create_subscriber(&topic).expect("create sub");

    // Warmup
    for _ in 0..WARMUP_ITERS {
        {
            let mut proxy = publisher.loan_proxy::<Point>().expect("warm loan");
            proxy.x = 1.0;
            proxy.y = 2.0;
            proxy.z = 3.0;
        }
        spin_recv_one::<Point, _>(&mut subscriber, Duration::from_secs(2), || {});
    }

    // Measure
    let mut latencies_us = Vec::with_capacity(MEASURE_ITERS);
    for i in 0..MEASURE_ITERS {
        let start = Instant::now();
        {
            let mut proxy = publisher.loan_proxy::<Point>().expect("hot loan");
            proxy.x = i as f64;
            proxy.y = (i + 1) as f64;
            proxy.z = (i + 2) as f64;
        }
        spin_recv_one::<Point, _>(&mut subscriber, Duration::from_secs(2), || {});
        let elapsed = start.elapsed();
        latencies_us.push(elapsed.as_secs_f64() * 1_000_000.0);
    }

    latencies_us.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let med = median(&latencies_us);
    println!(
        "small message (24B Point): floor={:.1}µs, median={:.1}µs, p99={:.1}µs, max={:.1}µs",
        latencies_us[0],
        med,
        latencies_us[(latencies_us.len() as f64 * 0.99) as usize],
        latencies_us[latencies_us.len() - 1]
    );

    assert!(
        med < 500.0,
        "small message median latency {:.1}µs exceeds 500µs threshold — \
         possible gross overhead regression",
        med
    );
}

// ============================================================
// Test 2: Large message (1MB Image) median latency < 2ms
// ============================================================

#[test]
fn test_large_message_latency() {
    let data_len = 1_000_000; // 1MB
    let max_slice = WireHeader::SIZE + 4096 + data_len;

    let mgr = TransportManager::get_or_init().expect("init");
    let topic = unique_topic("large_latency");

    let mut publisher = mgr
        .create_publisher_simple(&topic, MaxSliceLen::const_new(max_slice as u32))
        .expect("create pub");
    let mut subscriber = mgr.create_subscriber(&topic).expect("create sub");

    let publish_image = |publisher: &mut cerulion_core::transport::publisher::CerulionPublisher| {
        let mut proxy = publisher.loan_proxy::<Image>().expect("loan image");
        proxy.height = 1000;
        proxy.width = 1000;
        proxy.step = 1000;
        proxy.is_bigendian = 0;
        proxy.set_header_bytes(&[]).expect("set header");
        proxy.set_encoding("mono8").expect("set encoding");
        let dst: &mut [u8] = proxy.loan_data(data_len).expect("loan_data");
        // Fill with 0x80 — same byte everywhere matches the legacy test.
        for b in dst.iter_mut() {
            *b = 0x80;
        }
    };

    // Warmup
    for _ in 0..WARMUP_ITERS {
        publish_image(&mut publisher);
        spin_recv_one::<Image, _>(&mut subscriber, Duration::from_secs(2), || {});
    }

    // Measure
    let mut latencies_us = Vec::with_capacity(MEASURE_ITERS);
    for _ in 0..MEASURE_ITERS {
        let start = Instant::now();
        publish_image(&mut publisher);
        spin_recv_one::<Image, _>(&mut subscriber, Duration::from_secs(2), || {});
        let elapsed = start.elapsed();
        latencies_us.push(elapsed.as_secs_f64() * 1_000_000.0);
    }

    latencies_us.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let med = median(&latencies_us);
    println!(
        "large message (1MB Image): floor={:.1}µs, median={:.1}µs, p99={:.1}µs, max={:.1}µs",
        latencies_us[0],
        med,
        latencies_us[(latencies_us.len() as f64 * 0.99) as usize],
        latencies_us[latencies_us.len() - 1]
    );

    assert!(
        med < 2000.0,
        "large message median latency {:.1}µs exceeds 2000µs (2ms) threshold — \
         possible memcpy regression",
        med
    );
}

// ============================================================
// Test 3: Large/small latency ratio < 20x
// ============================================================

#[test]
fn test_latency_ratio_bounded() {
    let mgr = TransportManager::get_or_init().expect("init");

    // Small: 8B Time
    let small_med = {
        let topic = unique_topic("ratio_small");
        let mut publisher = mgr
            .create_publisher_simple(&topic, MaxSliceLen::const_new(1024))
            .expect("create pub");
        let mut subscriber = mgr.create_subscriber(&topic).expect("create sub");

        for _ in 0..WARMUP_ITERS {
            {
                let mut proxy = publisher.loan_proxy::<Time>().expect("warm");
                proxy.sec = 1;
                proxy.nanosec = 0;
            }
            spin_recv_one::<Time, _>(&mut subscriber, Duration::from_secs(2), || {});
        }
        let mut latencies = Vec::with_capacity(MEASURE_ITERS);
        for _ in 0..MEASURE_ITERS {
            let start = Instant::now();
            {
                let mut proxy = publisher.loan_proxy::<Time>().expect("loan");
                proxy.sec = 1;
                proxy.nanosec = 0;
            }
            spin_recv_one::<Time, _>(&mut subscriber, Duration::from_secs(2), || {});
            latencies.push(start.elapsed().as_secs_f64() * 1_000_000.0);
        }
        latencies.sort_by(|a, b| a.partial_cmp(b).unwrap());
        median(&latencies)
    };

    // Large: 1MB Image
    let data_len = 1_000_000;
    let max_slice = WireHeader::SIZE + 4096 + data_len;
    let large_med = {
        let topic = unique_topic("ratio_large");
        let mut publisher = mgr
            .create_publisher_simple(&topic, MaxSliceLen::const_new(max_slice as u32))
            .expect("create pub");
        let mut subscriber = mgr.create_subscriber(&topic).expect("create sub");

        let publish = |p: &mut cerulion_core::transport::publisher::CerulionPublisher| {
            let mut proxy = p.loan_proxy::<Image>().expect("loan");
            proxy.height = 1000;
            proxy.width = 1000;
            proxy.step = 1000;
            proxy.is_bigendian = 0;
            proxy.set_header_bytes(&[]).expect("hdr");
            proxy.set_encoding("mono8").expect("enc");
            let dst = proxy.loan_data(data_len).expect("loan_data");
            for b in dst.iter_mut() {
                *b = 0x80;
            }
        };

        for _ in 0..WARMUP_ITERS {
            publish(&mut publisher);
            spin_recv_one::<Image, _>(&mut subscriber, Duration::from_secs(2), || {});
        }
        let mut latencies = Vec::with_capacity(MEASURE_ITERS);
        for _ in 0..MEASURE_ITERS {
            let start = Instant::now();
            publish(&mut publisher);
            spin_recv_one::<Image, _>(&mut subscriber, Duration::from_secs(2), || {});
            latencies.push(start.elapsed().as_secs_f64() * 1_000_000.0);
        }
        latencies.sort_by(|a, b| a.partial_cmp(b).unwrap());
        median(&latencies)
    };

    let ratio = large_med / small_med;

    println!(
        "latency ratio: small={:.1}µs, large={:.1}µs, ratio={:.1}x",
        small_med, large_med, ratio
    );

    assert!(
        ratio < 20.0,
        "large/small latency ratio {:.1}x exceeds 20x — \
         true zero-copy should have bounded ratio; memcpy would show 100x+",
        ratio
    );
}

// ============================================================
// Test 4: Receive overhead is flat across payload sizes
// ============================================================

#[test]
fn test_receive_overhead_flat() {
    let mgr = TransportManager::get_or_init().expect("init");

    // 24B fixed: Point
    let small_med = {
        let topic = unique_topic("flat_small");
        let mut publisher = mgr
            .create_publisher_simple(&topic, MaxSliceLen::const_new(1024))
            .expect("create pub");
        let mut subscriber = mgr.create_subscriber(&topic).expect("create sub");
        for _ in 0..WARMUP_ITERS {
            {
                let mut p = publisher.loan_proxy::<Point>().expect("warm");
                p.x = 1.0;
                p.y = 2.0;
                p.z = 3.0;
            }
            spin_recv_one::<Point, _>(&mut subscriber, Duration::from_secs(2), || {});
        }
        let mut latencies = Vec::with_capacity(MEASURE_ITERS);
        for _ in 0..MEASURE_ITERS {
            let start = Instant::now();
            {
                let mut p = publisher.loan_proxy::<Point>().expect("loan");
                p.x = 1.0;
                p.y = 2.0;
                p.z = 3.0;
            }
            spin_recv_one::<Point, _>(&mut subscriber, Duration::from_secs(2), || {});
            latencies.push(start.elapsed().as_secs_f64() * 1_000_000.0);
        }
        latencies.sort_by(|a, b| a.partial_cmp(b).unwrap());
        median(&latencies)
    };

    // 64KB Image
    let medium_med = {
        let data_len = 65_536;
        let max_slice = WireHeader::SIZE + 4096 + data_len;
        let topic = unique_topic("flat_medium");
        let mut publisher = mgr
            .create_publisher_simple(&topic, MaxSliceLen::const_new(max_slice as u32))
            .expect("create pub");
        let mut subscriber = mgr.create_subscriber(&topic).expect("create sub");

        let publish = |p: &mut cerulion_core::transport::publisher::CerulionPublisher| {
            let mut proxy = p.loan_proxy::<Image>().expect("loan");
            proxy.height = 256;
            proxy.width = 256;
            proxy.step = 256;
            proxy.is_bigendian = 0;
            proxy.set_header_bytes(&[]).expect("hdr");
            proxy.set_encoding("mono8").expect("enc");
            let dst = proxy.loan_data(data_len).expect("loan_data");
            for b in dst.iter_mut() {
                *b = 0;
            }
        };

        for _ in 0..WARMUP_ITERS {
            publish(&mut publisher);
            spin_recv_one::<Image, _>(&mut subscriber, Duration::from_secs(2), || {});
        }
        let mut latencies = Vec::with_capacity(MEASURE_ITERS);
        for _ in 0..MEASURE_ITERS {
            let start = Instant::now();
            publish(&mut publisher);
            spin_recv_one::<Image, _>(&mut subscriber, Duration::from_secs(2), || {});
            latencies.push(start.elapsed().as_secs_f64() * 1_000_000.0);
        }
        latencies.sort_by(|a, b| a.partial_cmp(b).unwrap());
        median(&latencies)
    };

    // 1MB Image
    let large_med = {
        let data_len = 1_000_000;
        let max_slice = WireHeader::SIZE + 4096 + data_len;
        let topic = unique_topic("flat_large");
        let mut publisher = mgr
            .create_publisher_simple(&topic, MaxSliceLen::const_new(max_slice as u32))
            .expect("create pub");
        let mut subscriber = mgr.create_subscriber(&topic).expect("create sub");

        let publish = |p: &mut cerulion_core::transport::publisher::CerulionPublisher| {
            let mut proxy = p.loan_proxy::<Image>().expect("loan");
            proxy.height = 1000;
            proxy.width = 1000;
            proxy.step = 1000;
            proxy.is_bigendian = 0;
            proxy.set_header_bytes(&[]).expect("hdr");
            proxy.set_encoding("mono8").expect("enc");
            let dst = proxy.loan_data(data_len).expect("loan_data");
            for b in dst.iter_mut() {
                *b = 0x80;
            }
        };

        for _ in 0..WARMUP_ITERS {
            publish(&mut publisher);
            spin_recv_one::<Image, _>(&mut subscriber, Duration::from_secs(2), || {});
        }
        let mut latencies = Vec::with_capacity(MEASURE_ITERS);
        for _ in 0..MEASURE_ITERS {
            let start = Instant::now();
            publish(&mut publisher);
            spin_recv_one::<Image, _>(&mut subscriber, Duration::from_secs(2), || {});
            latencies.push(start.elapsed().as_secs_f64() * 1_000_000.0);
        }
        latencies.sort_by(|a, b| a.partial_cmp(b).unwrap());
        median(&latencies)
    };

    let max_med = small_med.max(medium_med).max(large_med);
    let min_med = small_med.min(medium_med).min(large_med);
    let flatness_ratio = max_med / min_med;

    println!(
        "flatness: small={:.1}µs, medium={:.1}µs, large={:.1}µs, ratio={:.1}x",
        small_med, medium_med, large_med, flatness_ratio
    );

    assert!(
        flatness_ratio < 25.0,
        "max/min median ratio {:.1}x exceeds 25x — \
         variable-size publish serialization adds O(n) cost (~12x typical), \
         but a memcpy regression would show 100x+",
        flatness_ratio
    );
}
