// SPDX-License-Identifier: AGPL-3.0-only
//! Release-mode latency threshold tests (SHM-backed API rewrite).
//!
//! Catches a payload path whose cost per hop is a large multiple of handing the
//! payload over, with thresholds loose enough for a shared runner's noise: absolute
//! ceilings in microseconds, and no ratio asserted anywhere in the file.
//!
//! Four payloads are timed here and FIVE assertions cover three of them: 500 on the
//! 24 B median, 100 on the 1 MB median in each of the two arms that time it, 2000 on
//! the 1 MB median of the third, and 50 on the 64 KB median. The 8 B leg is timed and
//! PRINTED and bounded by nothing; it exists as one arm's ratio denominator.
//!
//! A SINGLE added copy is a doubling of the affected leg, and these ceilings
//! deliberately do not catch it; a size ratio cannot catch it either without failing
//! on a cheaper small read, which is why the two ratios here are printed and not
//! asserted. `flat_latency_test` is the arm for that class: it takes each size's
//! FASTEST sample, drops the slowest of those as an outlier, and holds the next
//! slowest within 1.5x of the quickest.
//!
//! Uses `Instant::now()` for timing.
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
        "large message median latency {:.1}µs exceeds the 2000µs backstop. This arm \
         keeps the loose bound; the 100µs ceiling the other two arms assert on the \
         same payload is the one that moves first, and neither catches a single added \
         copy, which is a doubling",
        med
    );
}

// ============================================================
// Test 3: the 1 MB hop's median < 100 µs
// ============================================================

/// The 1 MB leg's ceiling, in microseconds, asserted by two of the three arms that
/// time that payload. The third keeps the looser 2000 backstop below.
///
/// Derived from measurement, not chosen, and stated as a RANGE over the readings on
/// record. Across ten runs on two machines, a shared hosted Linux runner and an Apple
/// Silicon workstation, every 1 MB median ON RECORD for these three arms, twenty six
/// readings, falls between 9.1 and 38.6, both ends from the workstation and the top of
/// it under ambient load. The runner's own recorded readings span 15.4 to 20.1, inside
/// that. The single samples on record for the payload, which only the third arm prints,
/// top out at 73.9.
///
/// So 100 is 2.6 times the worst median and clear of every single sample recorded. It
/// is twenty times tighter than that 2000 backstop, which against this range admits a
/// 52 to 220 fold inflation of the leg and so cannot fire on anything short of a
/// catastrophe.
///
/// What it refuses: a per-message cost orders of magnitude over handing the payload
/// over. These legs read in the tens of microseconds, while `docs/PERFORMANCE.md`
/// records a 1 MiB payload through a ROS 2 default stack in MILLISECONDS. No ratio is
/// taken across those two, because that document states its campaign measures a
/// different thing (a paced round trip across processes with the payload fill excluded)
/// and that dividing one of its figures by an in-tree test's gives a meaningless
/// number; its own rows come from a third machine besides these two. The ceiling is
/// therefore derived from the readings above and from nothing else. It is NOT a lower
/// bound on serialising a megabyte: a cheap serialising path that adds one pass over
/// the payload lands at the doubling below.
///
/// What it does NOT catch: ONE added copy inside an otherwise handover path costs about
/// one more pass over the payload, so it roughly doubles the leg, and twice the worst
/// median is 77.2, still under this ceiling. A size ratio cannot catch it without
/// failing on a healthy run, because the healthy runs already sit at the cap: a
/// doubling of the 1 MB leg crosses a 20x or 25x cap in nine of these ten runs, and
/// those same ten runs print 21.5x against a 20x cap and 49.2x against a 25x one
/// with every leg in range. Raising this ceiling to clear the doubling would chase one
/// workstation's loaded reading and weaken every other machine's margin.
///
/// On the leg's composition: a megabyte over the leg's own duration gives 26 GB/s at
/// the 38.6 reading and 110 GB/s at the 9.1 one, a LOWER bound on the publisher's fill
/// rate, since the leg also carries the loan, the header write, the publish, the notify
/// and the subscriber's spin. The 4.2x swing across that range on one machine is the
/// state of that machine, not the code.
const LARGE_1MB_CEILING_US: f64 = 100.0;

/// The 64 KB leg's ceiling, in microseconds. Only one arm times this payload, and
/// this is its only bound; the 8 B leg in the hop arm is the one payload here that no
/// assertion covers at all.
///
/// Across the same ten runs on the same two machines, every median printed for 64 KB,
/// ten readings, falls between 0.9 and 4.0, the top of that range on the workstation
/// under ambient load. 50 is therefore 12.5 times the worst, and twice that worst
/// reading, 8.0, is still more than six times under it.
///
/// The class it refuses is the megabyte ceiling's, scaled down: `docs/PERFORMANCE.md`
/// records 64 KiB through a ROS 2 default stack in hundreds of microseconds against
/// this leg's single digits, and again no ratio is taken across them, for the reason
/// that document gives. The margin here is thinner than the megabyte's by the nature
/// of the size: the payload pass is small, so a serialising stack's cost is mostly
/// fixed per message rather than proportional to the bytes.
const MEDIUM_64KB_CEILING_US: f64 = 50.0;

#[test]
fn test_large_hop_median_under_ceiling() {
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
        large_med < LARGE_1MB_CEILING_US,
        "the 1 MB hop's median is {large_med:.1}µs, over the \
         {LARGE_1MB_CEILING_US:.0}µs ceiling. A megabyte that is handed over costs one \
         pass over it, and every median ON RECORD for the payload in this file, on \
         two machines over ten runs, falls between 9.1 and 38.6µs. docs/PERFORMANCE.md \
         records a megabyte through a ROS 2 default stack in milliseconds, which is \
         the shape this refuses"
    );

    // The RATIO is reported above and deliberately not asserted. Its denominator is
    // the cost of an 8 byte read, so a cheaper small read raises it with no megabyte
    // copied anywhere: across the ten runs behind the ceiling above this arm has
    // printed between 3.8x and 49.2x while every 1 MB median stayed inside the range
    // the ceiling is derived over, and five of those ratios came off ONE machine. A cap
    // loose enough not to punish a cheaper small read is also unreachable: 200x against
    // the smallest small median this arm prints lands on the ceiling asserted above, so
    // the ceiling reds first. A cap tight enough to bite reds on an improvement.
}

// ============================================================
// Test 4: the 64 KB median < 50 µs and the 1 MB median < the shared ceiling
// ============================================================

#[test]
fn test_per_size_medians_under_ceilings() {
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
        medium_med < MEDIUM_64KB_CEILING_US,
        "the 64 KB hop's median is {medium_med:.1}µs, over the \
         {MEDIUM_64KB_CEILING_US:.0}µs ceiling. That leg has read 0.9 to 4.0µs over ten \
         runs on the two machines this bound comes from, so this is a twelvefold \
         inflation of it, not a machine's noise"
    );

    assert!(
        large_med < LARGE_1MB_CEILING_US,
        "the 1 MB hop's median is {large_med:.1}µs here, over the \
         {LARGE_1MB_CEILING_US:.0}µs ceiling the two asserting arms share, against a \
         9.1 to 38.6µs range on record over ten runs"
    );

    // The max-over-min FLATNESS RATIO is printed above and deliberately not asserted,
    // for the reason the other arm records: its denominator is the smallest of the
    // three medians, which is the 24 byte read in all ten runs on record, so a cheaper
    // small read raises it with nothing copied anywhere. Across those ten runs it has
    // printed between 3.8x and 21.5x, and the runner printed both 1.1 and
    // 0.7 for its 24 byte leg on ONE revision across two attempts, which is run to run
    // spread on that machine rather than a property of the code. A 25x cap on this
    // ratio therefore sits 1.16x from a red at the 21.5x these runs print, and the
    // claim such a cap carries, that a copy regression shows 100x and up, is about
    // three times out on this very ratio: one added pass over the payload doubles a
    // leg, which takes a printed 14.9x to 21.5x to roughly 30x to 43x. One run of the
    // ten printed 3.8x, which doubles to 7.6x and would clear any such cap. Per-size
    // FLATNESS is pinned, on floors rather than medians, by `flat_latency_test`.
}
