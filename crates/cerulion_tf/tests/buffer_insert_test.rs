// SPDX-License-Identifier: AGPL-3.0-only
//! Feeding the tf buffer through its public surface: what an insert
//! records, how the retention window expires samples, how a full ring
//! overflows, writers, epochs, interned frame ids, and every refusal, each
//! checked to leave the buffer exactly as it was (a refusal that
//! half-wrote is a corruption defect wearing an error message).
//!
//! Expected values are written by hand from the rules in the crate docs,
//! never read off the buffer.

use cerulion_tf::{
    EdgeInfo, EdgePolicy, FrameId, FrameLabel, TfBuffer, TfBufferConfig, TfError, Transform,
    UnitQuaternion, WriterId,
};
use std::num::{NonZeroU32, NonZeroUsize};
use std::time::Duration;

/// The writer of every edge unless a test says otherwise.
const W: WriterId = WriterId::new(1);
/// A second publisher.
const W2: WriterId = WriterId::new(2);

fn tf(x: f64) -> Transform {
    Transform::new([x, 0.0, 0.0], UnitQuaternion::IDENTITY).expect("a valid hand-written transform")
}

fn nz(n: usize) -> NonZeroUsize {
    NonZeroUsize::new(n).expect("a nonzero test capacity")
}

/// A buffer with room for `frames` frames, `samples` samples per ring, the
/// default 10 s window and 64 name bytes per frame.
fn small(frames: usize, samples: usize) -> TfBuffer {
    let rate = NonZeroU32::new(100).expect("a nonzero rate");
    TfBuffer::with_config(TfBufferConfig {
        ring_capacity: nz(samples),
        ..TfBufferConfig::new(nz(frames), Duration::from_secs(10), rate)
    })
}

/// A buffer of 8 frames and rings of 8 with a `retention_ns` window.
fn windowed(retention_ns: u64) -> TfBuffer {
    TfBuffer::with_config(TfBufferConfig {
        frames: nz(8),
        retention: Duration::from_nanos(retention_ns),
        ring_capacity: nz(8),
        ..TfBufferConfig::default()
    })
}

fn edge<'a>(b: &'a TfBuffer, child: &str) -> EdgeInfo<'a> {
    b.edge(child).expect("the edge exists")
}

fn id(b: &TfBuffer, frame: &str) -> FrameId {
    b.frame_id(frame).expect("a registered frame")
}

// ------------------------------------------------------------------- happy

#[test]
fn a_fresh_buffer_knows_no_frame_and_has_counted_nothing() {
    let b = TfBuffer::new();
    assert_eq!(b.frame_count(), 0);
    assert_eq!(b.frame_capacity(), 64);
    // 10 s at 100 Hz is 1000 samples, plus the one at the window's start.
    assert_eq!(b.ring_capacity(), 1_001);
    assert_eq!(b.retention(), Duration::from_secs(10));
    assert_eq!(b.name_budget(), 4_096);
    assert_eq!(cerulion_tf::DEFAULT_NAME_BYTES, 4_096);
    assert_eq!((b.expired(), b.replacements(), b.epoch()), (0, 0, 0));
    assert!(!b.contains_frame("odom"));
    assert_eq!(b.edge("odom"), None);
    assert_eq!(b.frame_id("odom"), None);
}

#[test]
fn a_config_from_a_rate_bound_sizes_the_ring_for_the_whole_window() {
    let rate = |hz| NonZeroU32::new(hz).unwrap();
    // 10 s at 1 kHz: 10 000 samples plus one.
    let c = TfBufferConfig::new(nz(3), Duration::from_secs(10), rate(1_000));
    assert_eq!(c.ring_capacity, nz(10_001));
    assert_eq!(c.name_bytes, 3 * 64);
    // 2.5 s at 3 Hz is 7.5 samples, rounded down to 7, plus one.
    let c = TfBufferConfig::new(nz(1), Duration::from_millis(2_500), rate(3));
    assert_eq!(c.ring_capacity, nz(8));
    // A zero window keeps only the newest sample.
    let c = TfBufferConfig::new(nz(1), Duration::ZERO, rate(1_000));
    assert_eq!(c.ring_capacity, nz(1));
    assert_eq!(TfBufferConfig::default(), {
        TfBufferConfig::new(nz(64), Duration::from_secs(10), rate(100))
    });
}

#[test]
fn one_dynamic_insert_registers_both_frames_and_one_sample() {
    let mut b = TfBuffer::new();
    b.insert(W, "odom", "base", 1_000, tf(1.0)).unwrap();
    assert_eq!(b.frame_count(), 2);
    assert!(b.contains_frame("odom") && b.contains_frame("base"));
    assert_eq!(
        edge(&b, "base"),
        EdgeInfo {
            parent: "odom",
            is_static: false,
            policy: EdgePolicy::Interpolate,
            writer: Some(W),
            samples: 1,
            oldest_ns: Some(1_000),
            newest_ns: Some(1_000),
            overflows: 0,
            static_refusals: 0,
            second_writer_refusals: 0,
        }
    );
    // A root has no edge of its own.
    assert_eq!(b.edge("odom"), None);
}

#[test]
fn a_static_insert_records_the_edge_without_a_ring_sample() {
    let mut b = TfBuffer::new();
    b.insert_static(W, "base", "lidar", tf(0.17)).unwrap();
    assert_eq!(
        edge(&b, "lidar"),
        EdgeInfo {
            parent: "base",
            is_static: true,
            policy: EdgePolicy::Interpolate,
            writer: Some(W),
            samples: 0,
            oldest_ns: None,
            newest_ns: None,
            overflows: 0,
            static_refusals: 0,
            second_writer_refusals: 0,
        }
    );
}

#[test]
fn a_rebroadcast_static_with_the_same_value_changes_nothing() {
    let mut b = TfBuffer::new();
    b.insert_static(W, "base", "lidar", tf(0.17)).unwrap();
    b.insert_static(W, "base", "lidar", tf(0.17)).unwrap();
    assert_eq!(b.frame_count(), 2);
    assert_eq!(b.replacements(), 0);
    // A new value replaces the old and is counted.
    b.insert_static(W, "base", "lidar", tf(0.25)).unwrap();
    assert_eq!(b.replacements(), 1);
    assert_eq!(edge(&b, "lidar").samples, 0);
    assert_eq!(b.lookup_latest("base", "lidar").unwrap(), tf(0.25));
}

#[test]
fn the_same_stamp_and_value_twice_is_a_no_op() {
    let mut b = TfBuffer::new();
    b.insert(W, "odom", "base", 1_000, tf(1.0)).unwrap();
    b.insert(W, "odom", "base", 1_000, tf(1.0)).unwrap();
    assert_eq!(edge(&b, "base").samples, 1);
    assert_eq!(b.replacements(), 0);
}

#[test]
fn the_same_stamp_with_a_different_value_replaces_it_and_counts() {
    let mut b = TfBuffer::new();
    b.insert(W, "odom", "base", 1_000, tf(1.0)).unwrap();
    b.insert(W, "odom", "base", 1_000, tf(2.0)).unwrap();
    assert_eq!(edge(&b, "base").samples, 1);
    assert_eq!(b.replacements(), 1);
    assert_eq!(b.lookup("odom", "base", 1_000).unwrap(), tf(2.0));
}

#[test]
fn out_of_order_stamps_are_kept_sorted() {
    let mut b = TfBuffer::new();
    b.insert(W, "odom", "base", 3_000, tf(3.0)).unwrap();
    b.insert(W, "odom", "base", 1_000, tf(1.0)).unwrap();
    b.insert(W, "odom", "base", 2_000, tf(2.0)).unwrap();
    let e = edge(&b, "base");
    assert_eq!(
        (e.samples, e.oldest_ns, e.newest_ns),
        (3, Some(1_000), Some(3_000))
    );
    // The ring is sorted, so the midpoint between 1000 and 2000 is 1.5.
    assert_eq!(b.lookup("odom", "base", 1_500).unwrap(), tf(1.5));
}

#[test]
fn several_edges_can_hang_off_one_parent() {
    let mut b = TfBuffer::new();
    b.insert(W, "base", "left", 10, tf(1.0)).unwrap();
    b.insert(W, "base", "right", 10, tf(-1.0)).unwrap();
    b.insert_static(W, "base", "lidar", tf(0.17)).unwrap();
    assert_eq!(b.frame_count(), 4);
    assert_eq!(edge(&b, "left").parent, "base");
    assert_eq!(edge(&b, "right").parent, "base");
    assert_eq!(edge(&b, "lidar").parent, "base");
}

#[test]
fn different_writers_may_feed_different_edges() {
    let mut b = TfBuffer::new();
    b.insert(W, "odom", "base", 10, tf(1.0)).unwrap();
    b.insert(W2, "base", "arm", 10, tf(2.0)).unwrap();
    b.insert_static(W2, "base", "lidar", tf(0.17)).unwrap();
    assert_eq!(edge(&b, "base").writer, Some(W));
    assert_eq!(edge(&b, "arm").writer, Some(W2));
    assert_eq!(edge(&b, "arm").writer.map(WriterId::get), Some(2));
    assert_eq!(WriterId::new(u64::MAX).get(), u64::MAX);
    assert_eq!(b.lookup("odom", "arm", 10).unwrap(), tf(3.0));
}

// ------------------------------------------------------------ the retention

#[test]
fn an_insert_expires_every_sample_older_than_the_window() {
    let mut b = windowed(1_000);
    for stamp in [0u64, 500, 1_000] {
        b.insert(W, "odom", "base", stamp, tf(stamp as f64))
            .unwrap();
    }
    assert_eq!(b.expired(), 0, "0 is exactly 1000 before 1000: kept");
    b.insert(W, "odom", "base", 1_500, tf(1_500.0)).unwrap();
    // The window is now 500..=1500: 0 left it.
    let e = edge(&b, "base");
    assert_eq!(
        (e.samples, e.oldest_ns, e.newest_ns, e.overflows),
        (3, Some(500), Some(1_500), 0)
    );
    assert_eq!(b.expired(), 1);
    // A jump of two windows expires everything but the new sample.
    b.insert(W, "odom", "base", 5_000, tf(5_000.0)).unwrap();
    let e = edge(&b, "base");
    assert_eq!((e.samples, e.oldest_ns), (1, Some(5_000)));
    assert_eq!(b.expired(), 4);
}

#[test]
fn a_stamp_inside_the_window_lands_in_order_and_expires_nothing() {
    let mut b = windowed(1_000);
    b.insert(W, "odom", "base", 500, tf(0.0)).unwrap();
    b.insert(W, "odom", "base", 1_500, tf(10.0)).unwrap();
    // 500 is the window's first stamp, and 600 is inside it.
    b.insert(W, "odom", "base", 600, tf(1.0)).unwrap();
    assert_eq!(edge(&b, "base").samples, 3);
    assert_eq!(b.expired(), 0);
    assert_eq!(b.lookup("odom", "base", 550).unwrap(), tf(0.5));
}

#[test]
fn a_ring_full_before_its_window_overflows_its_oldest_sample_and_counts_it() {
    let mut b = small(8, 3);
    for (k, stamp) in [100u64, 200, 300].into_iter().enumerate() {
        b.insert(W, "odom", "base", stamp, tf(k as f64)).unwrap();
    }
    assert_eq!(edge(&b, "base").overflows, 0);
    b.insert(W, "odom", "base", 400, tf(3.0)).unwrap();
    let e = edge(&b, "base");
    assert_eq!(
        (e.samples, e.oldest_ns, e.newest_ns, e.overflows),
        (3, Some(200), Some(400), 1)
    );
    // An overflow is not a normal expiry.
    assert_eq!(b.expired(), 0);
    assert!(matches!(
        b.lookup("odom", "base", 100),
        Err(TfError::OutOfRange {
            oldest_ns: 200,
            newest_ns: 400,
            ..
        })
    ));
}

#[test]
fn a_sample_older_than_a_full_ring_is_the_one_that_overflows() {
    let mut b = small(8, 2);
    b.insert(W, "odom", "base", 200, tf(2.0)).unwrap();
    b.insert(W, "odom", "base", 300, tf(3.0)).unwrap();
    b.insert(W, "odom", "base", 100, tf(1.0)).unwrap();
    let e = edge(&b, "base");
    assert_eq!(
        (e.samples, e.oldest_ns, e.newest_ns, e.overflows),
        (2, Some(200), Some(300), 1)
    );
}

#[test]
fn a_full_ring_keeps_an_out_of_order_sample_that_is_newer_than_its_oldest() {
    let mut b = small(8, 3);
    b.insert(W, "odom", "base", 100, tf(1.0)).unwrap();
    b.insert(W, "odom", "base", 300, tf(3.0)).unwrap();
    b.insert(W, "odom", "base", 400, tf(4.0)).unwrap();
    // 200 lands between 100 and 300; 100 makes room.
    b.insert(W, "odom", "base", 200, tf(2.0)).unwrap();
    let e = edge(&b, "base");
    assert_eq!(
        (e.samples, e.oldest_ns, e.newest_ns, e.overflows),
        (3, Some(200), Some(400), 1)
    );
    assert_eq!(b.lookup("odom", "base", 250).unwrap(), tf(2.5));
}

#[test]
fn overflows_are_counted_per_edge() {
    let mut b = small(8, 1);
    b.insert(W, "odom", "base", 1, tf(1.0)).unwrap();
    b.insert(W, "odom", "base", 2, tf(2.0)).unwrap();
    b.insert(W, "base", "arm", 1, tf(1.0)).unwrap();
    b.insert(W, "base", "arm", 2, tf(2.0)).unwrap();
    b.insert(W, "base", "arm", 3, tf(3.0)).unwrap();
    assert_eq!(edge(&b, "base").overflows, 1);
    assert_eq!(edge(&b, "arm").overflows, 2);
}

#[test]
fn a_static_insert_on_a_dynamic_edge_clears_its_history_and_wins() {
    let mut b = TfBuffer::new();
    b.insert(W, "base", "lidar", 100, tf(1.0)).unwrap();
    b.insert(W, "base", "lidar", 200, tf(2.0)).unwrap();
    b.insert_static(W, "base", "lidar", tf(0.17)).unwrap();
    let e = edge(&b, "lidar");
    assert_eq!(
        (e.is_static, e.samples, e.oldest_ns, e.newest_ns),
        (true, 0, None, None)
    );
    assert_eq!(b.expired(), 2, "the two cleared samples are expired");
    // At any time, including one the ring never held.
    assert_eq!(b.lookup("base", "lidar", 7).unwrap(), tf(0.17));
    assert_eq!(b.lookup("base", "lidar", 10_000).unwrap(), tf(0.17));
    // A re-broadcast static has nothing left to clear.
    b.insert_static(W, "base", "lidar", tf(0.17)).unwrap();
    assert_eq!(b.expired(), 2);
}

// ---------------------------------------------------------------- epochs

#[test]
fn a_backward_jump_on_a_full_window_is_refused_not_dropped() {
    // A bag loop: the stamps restart near zero after running to 2000.
    let mut b = windowed(1_000);
    for stamp in [1_000u64, 1_500, 2_000] {
        b.insert(W, "odom", "base", stamp, tf(1.0)).unwrap();
    }
    b.insert_static(W, "base", "lidar", tf(0.17)).unwrap();
    let base = id(&b, "base");
    let odom = id(&b, "odom");
    assert_eq!(
        b.insert(W, "odom", "base", 10, tf(9.0)),
        Err(TfError::StampBeforeWindow {
            parent: odom,
            child: base,
            stamp_ns: 10,
            newest_ns: 2_000,
        })
    );
    // 999 is one nanosecond before the window start of 1000.
    assert!(matches!(
        b.insert(W, "odom", "base", 999, tf(9.0)),
        Err(TfError::StampBeforeWindow { stamp_ns: 999, .. })
    ));
    assert_eq!(edge(&b, "base").samples, 3, "the refusal kept the ring");
}

#[test]
fn begin_epoch_clears_the_rings_and_keeps_the_tree_and_the_statics() {
    let mut b = windowed(1_000);
    for stamp in [1_000u64, 1_500, 2_000] {
        b.insert(W, "odom", "base", stamp, tf(1.0)).unwrap();
    }
    b.insert_static(W, "base", "lidar", tf(0.17)).unwrap();
    b.set_edge_policy("map", "odom", EdgePolicy::HoldLast { max_age_ns: 50 })
        .unwrap();
    b.begin_epoch();
    assert_eq!(b.epoch(), 1);
    assert_eq!(b.expired(), 3);
    assert_eq!(b.frame_count(), 4);
    let e = edge(&b, "base");
    assert_eq!((e.samples, e.writer, e.parent), (0, Some(W), "odom"));
    assert_eq!(
        edge(&b, "odom").policy,
        EdgePolicy::HoldLast { max_age_ns: 50 }
    );
    // The empty edge says so; the static still resolves.
    assert_eq!(
        b.lookup("odom", "base", 1_500),
        Err(TfError::EdgeEmpty {
            parent: id(&b, "odom"),
            child: id(&b, "base"),
        })
    );
    assert_eq!(b.lookup("base", "lidar", 7).unwrap(), tf(0.17));
    // The new timeline is accepted from its first stamp.
    b.insert(W, "odom", "base", 10, tf(2.0)).unwrap();
    b.insert(W, "odom", "base", 20, tf(4.0)).unwrap();
    assert_eq!(b.lookup("odom", "lidar", 15).unwrap(), tf(3.17));
    b.begin_epoch();
    assert_eq!(b.epoch(), 2);
    assert_eq!(b.expired(), 5);
}

// ---------------------------------------------------------------- writers

#[test]
fn a_second_writer_is_refused_once_then_counted() {
    let mut b = TfBuffer::new();
    b.insert(W, "odom", "base", 100, tf(1.0)).unwrap();
    let (odom, base) = (id(&b, "odom"), id(&b, "base"));
    assert_eq!(
        b.insert(W2, "odom", "base", 200, tf(2.0)),
        Err(TfError::SecondWriter {
            parent: odom,
            child: base,
            writer: W2,
            existing_writer: W,
        })
    );
    // The repeats are counted on the edge, not returned again.
    assert_eq!(b.insert(W2, "odom", "base", 300, tf(3.0)), Ok(()));
    assert_eq!(b.insert(W2, "odom", "base", 400, tf(4.0)), Ok(()));
    let e = edge(&b, "base");
    assert_eq!(
        (e.samples, e.newest_ns, e.writer, e.second_writer_refusals),
        (1, Some(100), Some(W), 3)
    );
    // The first writer keeps feeding the edge.
    b.insert(W, "odom", "base", 200, tf(2.0)).unwrap();
    assert_eq!(edge(&b, "base").samples, 2);
}

#[test]
fn a_second_writer_at_a_held_stamp_is_refused_not_replaced() {
    let mut b = TfBuffer::new();
    b.insert(W, "odom", "base", 100, tf(1.0)).unwrap();
    assert!(matches!(
        b.insert(W2, "odom", "base", 100, tf(9.0)),
        Err(TfError::SecondWriter { .. })
    ));
    assert_eq!(b.replacements(), 0);
    assert_eq!(b.lookup("odom", "base", 100).unwrap(), tf(1.0));
}

#[test]
fn a_second_static_writer_is_refused() {
    let mut b = TfBuffer::new();
    b.insert_static(W, "base", "lidar", tf(0.17)).unwrap();
    assert!(matches!(
        b.insert_static(W2, "base", "lidar", tf(0.25)),
        Err(TfError::SecondWriter { writer, existing_writer, .. })
            if writer == W2 && existing_writer == W
    ));
    assert_eq!(b.lookup("base", "lidar", 0).unwrap(), tf(0.17));
    assert_eq!(edge(&b, "lidar").second_writer_refusals, 1);
}

#[test]
fn a_dynamic_transform_on_a_static_edge_is_refused_once_then_counted() {
    let mut b = TfBuffer::new();
    b.insert_static(W, "base", "lidar", tf(0.17)).unwrap();
    let (base, lidar) = (id(&b, "base"), id(&b, "lidar"));
    assert_eq!(
        b.insert(W, "base", "lidar", 1, tf(1.0)),
        Err(TfError::EdgeIsStatic {
            parent: base,
            child: lidar
        })
    );
    for stamp in 2..=100u64 {
        assert_eq!(b.insert(W, "base", "lidar", stamp, tf(1.0)), Ok(()));
    }
    let e = edge(&b, "lidar");
    assert_eq!((e.samples, e.static_refusals), (0, 100));
    assert_eq!(b.lookup("base", "lidar", 50).unwrap(), tf(0.17));
}

// ---------------------------------------------------------- edge policies

#[test]
fn a_policy_set_before_any_sample_declares_an_empty_edge() {
    let mut b = TfBuffer::new();
    b.set_edge_policy("map", "odom", EdgePolicy::HoldLast { max_age_ns: 7 })
        .unwrap();
    assert_eq!(b.frame_count(), 2);
    let e = edge(&b, "odom");
    assert_eq!(
        (e.policy, e.writer, e.samples),
        (EdgePolicy::HoldLast { max_age_ns: 7 }, None, 0)
    );
    assert_eq!(
        b.lookup_latest("map", "odom"),
        Err(TfError::EdgeEmpty {
            parent: id(&b, "map"),
            child: id(&b, "odom"),
        })
    );
    // The first insert takes the writer; the policy stays.
    b.insert(W2, "map", "odom", 100, tf(1.0)).unwrap();
    let e = edge(&b, "odom");
    assert_eq!((e.writer, e.samples), (Some(W2), 1));
    assert_eq!(b.lookup("map", "odom", 107).unwrap(), tf(1.0));
    // Setting it again replaces it.
    b.set_edge_policy("map", "odom", EdgePolicy::Interpolate)
        .unwrap();
    assert!(matches!(
        b.lookup("map", "odom", 101),
        Err(TfError::NotYetAvailable { .. })
    ));
}

#[test]
fn a_policy_on_a_static_edge_is_refused_every_time() {
    let mut b = TfBuffer::new();
    b.insert_static(W, "base", "lidar", tf(0.17)).unwrap();
    for _ in 0..2 {
        assert!(matches!(
            b.set_edge_policy("base", "lidar", EdgePolicy::HoldLast { max_age_ns: 1 }),
            Err(TfError::EdgeIsStatic { .. })
        ));
    }
    let e = edge(&b, "lidar");
    assert_eq!((e.policy, e.static_refusals), (EdgePolicy::Interpolate, 0));
}

// --------------------------------------------------------------- frame ids

#[test]
fn a_long_namespaced_frame_id_is_accepted_whole() {
    let mut b = TfBuffer::new();
    let realsense = "robot_0/camera_aligned_depth_to_color_frame";
    assert_eq!(realsense.len(), 43);
    let long = "z".repeat(300);
    b.insert_static(W, "robot_0/camera_link", realsense, tf(0.01))
        .unwrap();
    b.insert_static(W, realsense, &long, tf(0.02)).unwrap();
    assert_eq!(edge(&b, &long).parent, realsense);
    let rs = id(&b, realsense);
    assert_eq!(b.frame_name(rs), Some(realsense));
    assert_eq!(b.lookup("robot_0/camera_link", &long, 0).unwrap(), tf(0.03));
}

#[test]
fn frame_ids_are_handles_that_round_trip_through_their_names() {
    let mut b = TfBuffer::new();
    b.insert(W, "odom", "base", 1, tf(1.0)).unwrap();
    let odom = id(&b, "odom");
    let base = id(&b, "/base");
    assert_ne!(odom, base);
    assert_eq!(b.frame_name(odom), Some("odom"));
    assert_eq!(b.frame_name(base), Some("base"));
    // Handles print as `#n` on their own.
    assert_eq!(odom.to_string(), "#0");
    assert_eq!(base.to_string(), "#1");
    // A handle this buffer never issued has no name.
    let mut bigger = TfBuffer::new();
    bigger.insert(W, "a", "b", 1, tf(1.0)).unwrap();
    bigger.insert(W, "b", "c", 1, tf(1.0)).unwrap();
    let c = bigger.frame_id("c").unwrap();
    assert_eq!(b.frame_name(c), None);
}

#[test]
fn every_frame_of_a_full_table_is_found_by_name() {
    // 64 names in an index of 128 slots: collisions are certain, and every
    // name still finds its own handle.
    let mut b = TfBuffer::new();
    let names: Vec<String> = (0..64).map(|i| format!("frame_{i}")).collect();
    for pair in names.windows(2) {
        b.insert_static(W, &pair[0], &pair[1], tf(1.0)).unwrap();
    }
    assert_eq!(b.frame_count(), 64);
    for (i, name) in names.iter().enumerate() {
        let got = b.frame_id(name).expect("a registered frame");
        assert_eq!(b.frame_name(got), Some(name.as_str()), "frame {i}");
    }
    assert_eq!(b.frame_id("frame_64"), None);
    // The chain of 63 one-metre edges.
    assert_eq!(b.lookup("frame_0", "frame_63", 0).unwrap(), tf(63.0));
}

#[test]
fn leading_slashes_are_removed_so_slash_odom_and_odom_name_one_frame() {
    let mut b = TfBuffer::new();
    b.insert(W, "/odom", "/base_link", 1_000, tf(1.0)).unwrap();
    assert_eq!(b.frame_count(), 2);
    assert!(b.contains_frame("odom") && b.contains_frame("/odom"));
    assert_eq!(edge(&b, "base_link").parent, "odom");
    assert_eq!(edge(&b, "/base_link"), edge(&b, "base_link"));
    // The bare id lands on the same edge: one ring, two samples.
    b.insert(W, "odom", "base_link", 2_000, tf(2.0)).unwrap();
    assert_eq!(b.frame_count(), 2);
    assert_eq!(edge(&b, "base_link").samples, 2);
    assert_eq!(b.lookup("/odom", "/base_link", 1_500).unwrap(), tf(1.5));
    // Every leading slash goes: `//odom` is the frame `odom`, so a stored
    // name passed back names the frame it came from.
    b.insert(W, "//odom", "lidar", 1_000, tf(0.5)).unwrap();
    assert_eq!(b.frame_count(), 3, "`//odom` is the existing `odom`");
    let parent = edge(&b, "lidar").parent.to_string();
    assert_eq!(parent, "odom");
    assert_eq!(b.lookup(&parent, "lidar", 1_000).unwrap(), tf(0.5));
}

#[test]
fn the_buffer_fills_to_exactly_its_frame_capacity() {
    let mut b = small(3, 4);
    b.insert(W, "a", "b", 1, tf(1.0)).unwrap();
    b.insert(W, "b", "c", 1, tf(1.0)).unwrap();
    assert_eq!(b.frame_count(), 3);
    assert_eq!(
        b.insert(W, "c", "d", 1, tf(1.0)),
        Err(TfError::FrameTableFull {
            frame: FrameLabel::new("d"),
            capacity: 3
        })
    );
}

#[test]
fn the_name_budget_fills_to_exactly_its_bytes() {
    // Ten bytes: `odom` (4) and `base` (4) leave 2.
    let mut b = TfBuffer::with_config(TfBufferConfig {
        name_bytes: 10,
        ..TfBufferConfig::default()
    });
    b.insert(W, "odom", "base", 1, tf(1.0)).unwrap();
    b.insert(W, "base", "ab", 1, tf(1.0)).unwrap();
    assert_eq!(
        b.insert(W, "base", "c", 1, tf(1.0)),
        Err(TfError::NameBudgetFull {
            frame: FrameLabel::new("c"),
            len: 1,
            budget: 10,
        })
    );
    assert_eq!(b.frame_count(), 3);
    // Both frames new: the parent is interned first. With 2 bytes left,
    // `x` (1) fits and `yy` (2) does not fit after it, so `yy` is named;
    // `xyz` (3) does not fit on its own, so it is named.
    let mut b = TfBuffer::with_config(TfBufferConfig {
        name_bytes: 10,
        ..TfBufferConfig::default()
    });
    b.insert(W, "odom", "base", 1, tf(1.0)).unwrap();
    assert_eq!(
        b.insert(W, "x", "yy", 1, tf(1.0)),
        Err(TfError::NameBudgetFull {
            frame: FrameLabel::new("yy"),
            len: 2,
            budget: 10,
        })
    );
    assert_eq!(
        b.insert(W, "xyz", "q", 1, tf(1.0)),
        Err(TfError::NameBudgetFull {
            frame: FrameLabel::new("xyz"),
            len: 3,
            budget: 10,
        })
    );
    assert_eq!(b.frame_count(), 2);
    b.insert(W, "x", "y", 1, tf(1.0)).unwrap();
    assert_eq!(b.frame_count(), 4);
}

#[test]
fn an_edge_from_a_known_frame_to_another_trees_root_joins_the_trees() {
    // `map -> odom` and `base -> lidar` are two trees; `odom -> base` hangs
    // the second under the first. `base` is a root with a child, so the
    // cycle check walks up from `odom` and must find no `base` there.
    let mut b = TfBuffer::new();
    b.insert(W, "map", "odom", 1, tf(1.0)).unwrap();
    b.insert(W, "base", "lidar", 1, tf(1.0)).unwrap();
    b.insert(W, "odom", "base", 1, tf(1.0)).unwrap();
    assert_eq!(edge(&b, "base").parent, "odom");
    assert_eq!(b.frame_count(), 4);
    // Joined, the whole chain resolves: 1 m per edge, three edges.
    assert_eq!(b.lookup("map", "lidar", 1).unwrap(), tf(3.0));
}

// ------------------------------------------------------------- adversarial

#[test]
fn a_nan_quaternion_never_reaches_the_buffer() {
    // The feed shape: build the transform, then insert; `?` carries the math
    // error unchanged: the crate has one error type.
    fn feed(b: &mut TfBuffer, q: [f64; 4]) -> Result<(), TfError> {
        let rotation = UnitQuaternion::from_array(q)?;
        let transform = Transform::new([0.0, 0.0, 0.0], rotation)?;
        b.insert(W, "odom", "base", 1, transform)
    }
    let mut b = TfBuffer::new();
    assert_eq!(
        feed(&mut b, [f64::NAN, 0.0, 0.0, 1.0]),
        Err(TfError::NonFiniteQuaternion)
    );
    assert_eq!(
        feed(&mut b, [0.0, 0.0, 0.0, 0.0]),
        Err(TfError::DegenerateQuaternion { norm: 0.0 })
    );
    assert_eq!(b.frame_count(), 0, "a refused transform registers no frame");
}

#[test]
fn a_non_finite_translation_never_reaches_the_buffer() {
    let mut b = TfBuffer::new();
    let bad = Transform::new([f64::INFINITY, 0.0, 0.0], UnitQuaternion::IDENTITY);
    assert_eq!(bad, Err(TfError::NonFiniteTranslation));
    assert_eq!(b.frame_count(), 0);
    // The buffer is still usable.
    b.insert(W, "odom", "base", 1, tf(1.0)).unwrap();
    assert_eq!(b.frame_count(), 2);
}

// --------------------------------------------------------- every error arm

/// Runs `refused` against a buffer holding `odom -> base` and `base -> lidar`
/// (static) and checks the refusal leaves both edges and the counters as
/// they were.
fn refusal_leaves_the_buffer_untouched(
    refused: impl FnOnce(&mut TfBuffer) -> Result<(), TfError>,
    want: impl FnOnce(&TfBuffer) -> TfError,
) {
    let mut b = small(4, 4);
    b.insert(W, "odom", "base", 100, tf(1.0)).unwrap();
    b.insert(W, "odom", "base", 200, tf(2.0)).unwrap();
    b.insert_static(W, "base", "lidar", tf(0.17)).unwrap();
    let before = snapshot(&b);
    let want = want(&b);
    assert_eq!(refused(&mut b), Err(want));
    assert_eq!(before, snapshot(&b), "the refusal changed the buffer");
}

/// Everything observable about the fixture buffer, owned so it can be
/// compared across a mutable borrow. The refusal counters are left out:
/// counting a refusal is its job.
type Snapshot = (
    usize,
    u64,
    u64,
    OwnedEdge,
    OwnedEdge,
    [Transform; 3],
    Transform,
);
type OwnedEdge = (
    String,
    bool,
    Option<WriterId>,
    usize,
    Option<u64>,
    Option<u64>,
);

fn snapshot(b: &TfBuffer) -> Snapshot {
    let own = |e: EdgeInfo<'_>| {
        (
            e.parent.to_string(),
            e.is_static,
            e.writer,
            e.samples,
            e.oldest_ns,
            e.newest_ns,
        )
    };
    (
        b.frame_count(),
        b.expired(),
        b.replacements(),
        own(edge(b, "base")),
        own(edge(b, "lidar")),
        // Each stored sample at its exact stamp, and the midpoint between
        // them, so a refusal that rewrites the samples but keeps their mean
        // still fails.
        [100, 150, 200].map(|t| b.lookup("odom", "base", t).unwrap()),
        // The static edge's value, so a refusal that overwrites it fails.
        b.lookup("base", "lidar", 150).unwrap(),
    )
}

#[test]
fn an_empty_frame_id_is_refused() {
    refusal_leaves_the_buffer_untouched(
        |b| b.insert(W, "", "base", 1, tf(1.0)),
        |_| TfError::EmptyFrameId,
    );
    refusal_leaves_the_buffer_untouched(
        |b| b.insert(W, "odom", "/", 1, tf(1.0)),
        |_| TfError::EmptyFrameId,
    );
    refusal_leaves_the_buffer_untouched(
        |b| b.insert_static(W, "", "", tf(1.0)),
        |_| TfError::EmptyFrameId,
    );
    refusal_leaves_the_buffer_untouched(
        |b| b.set_edge_policy("///", "x", EdgePolicy::Interpolate),
        |_| TfError::EmptyFrameId,
    );
}

#[test]
fn a_self_edge_is_refused() {
    refusal_leaves_the_buffer_untouched(
        |b| b.insert(W, "base", "base", 1, tf(1.0)),
        |_| TfError::SelfEdge {
            frame: FrameLabel::new("base"),
        },
    );
    refusal_leaves_the_buffer_untouched(
        |b| b.insert(W, "/base", "base", 1, tf(1.0)),
        |_| TfError::SelfEdge {
            frame: FrameLabel::new("base"),
        },
    );
    refusal_leaves_the_buffer_untouched(
        |b| b.insert_static(W, "new", "new", tf(1.0)),
        |_| TfError::SelfEdge {
            frame: FrameLabel::new("new"),
        },
    );
}

#[test]
fn a_second_parent_for_a_child_is_refused() {
    refusal_leaves_the_buffer_untouched(
        |b| b.insert(W, "map", "base", 1, tf(1.0)),
        |b| TfError::SecondParent {
            child: id(b, "base"),
            parent: FrameLabel::new("map"),
            existing_parent: id(b, "odom"),
        },
    );
    // A static edge is protected the same way.
    refusal_leaves_the_buffer_untouched(
        |b| b.insert_static(W, "odom", "lidar", tf(1.0)),
        |b| TfError::SecondParent {
            child: id(b, "lidar"),
            parent: FrameLabel::new("odom"),
            existing_parent: id(b, "base"),
        },
    );
}

#[test]
fn an_edge_that_would_close_a_cycle_is_refused() {
    // lidar -> odom would make odom descend from itself.
    refusal_leaves_the_buffer_untouched(
        |b| b.insert(W, "lidar", "odom", 1, tf(1.0)),
        |b| TfError::Cycle {
            parent: id(b, "lidar"),
            child: id(b, "odom"),
        },
    );
    refusal_leaves_the_buffer_untouched(
        |b| b.set_edge_policy("base", "odom", EdgePolicy::Interpolate),
        |b| TfError::Cycle {
            parent: id(b, "base"),
            child: id(b, "odom"),
        },
    );
}

#[test]
fn a_write_conflicting_with_the_edge_is_refused_without_changing_it() {
    refusal_leaves_the_buffer_untouched(
        |b| b.insert(W, "base", "lidar", 1, tf(1.0)),
        |b| TfError::EdgeIsStatic {
            parent: id(b, "base"),
            child: id(b, "lidar"),
        },
    );
    refusal_leaves_the_buffer_untouched(
        |b| b.insert(W2, "odom", "base", 150, tf(1.0)),
        |b| TfError::SecondWriter {
            parent: id(b, "odom"),
            child: id(b, "base"),
            writer: W2,
            existing_writer: W,
        },
    );
    // The fixture's newest stamp is 200 ns, inside the first 10 s window,
    // so no stamp is before it; a stamp more than 10 s before 20 s is.
    let mut b = small(4, 4);
    b.insert(W, "odom", "base", 20_000_000_000, tf(1.0))
        .unwrap();
    let span = |b: &TfBuffer| {
        let e = edge(b, "base");
        (e.samples, e.oldest_ns, e.newest_ns)
    };
    let before = span(&b);
    assert!(matches!(
        b.insert(W, "odom", "base", 9_999_999_999, tf(1.0)),
        Err(TfError::StampBeforeWindow {
            stamp_ns: 9_999_999_999,
            newest_ns: 20_000_000_000,
            ..
        })
    ));
    assert_eq!(span(&b), before);
    b.insert(W, "odom", "base", 10_000_000_000, tf(1.0))
        .unwrap();
}

#[test]
fn a_frame_beyond_the_capacity_is_refused_naming_the_frame_that_does_not_fit() {
    // The fixture holds 3 of 4 frames: one more fits, two do not.
    refusal_leaves_the_buffer_untouched(
        |b| b.insert(W, "x", "y", 1, tf(1.0)),
        |_| TfError::FrameTableFull {
            frame: FrameLabel::new("x"),
            capacity: 4,
        },
    );
    // One new frame fits; the refusal names the child when the parent is known.
    let mut b = small(4, 4);
    b.insert(W, "odom", "base", 1, tf(1.0)).unwrap();
    b.insert(W, "base", "lidar", 1, tf(1.0)).unwrap();
    b.insert(W, "base", "cam", 1, tf(1.0)).unwrap();
    assert_eq!(
        b.insert(W, "base", "imu", 1, tf(1.0)),
        Err(TfError::FrameTableFull {
            frame: FrameLabel::new("imu"),
            capacity: 4
        })
    );
    assert_eq!(b.frame_count(), 4);
}

#[test]
fn a_frame_beyond_the_name_budget_is_refused_naming_it() {
    // 4 frames of 64 bytes each: 256 bytes, of which the fixture's `odom`,
    // `base` and `lidar` use 13.
    let long = "q".repeat(244);
    refusal_leaves_the_buffer_untouched(
        |b| b.insert(W, "base", &long, 1, tf(1.0)),
        |_| TfError::NameBudgetFull {
            frame: FrameLabel::new(&"q".repeat(244)),
            len: 244,
            budget: 256,
        },
    );
    // 243 bytes fit exactly.
    let mut b = small(4, 4);
    b.insert(W, "odom", "base", 100, tf(1.0)).unwrap();
    b.insert_static(W, "base", "lidar", tf(0.17)).unwrap();
    b.insert(W, "base", &"q".repeat(243), 1, tf(1.0)).unwrap();
}

// ----------------------------------------------------- the error text contract

#[test]
fn every_refusal_names_the_frames_involved() {
    let mut b = TfBuffer::new();
    b.insert(W, "odom", "base", 300, tf(1.0)).unwrap();
    b.insert_static(W, "base", "lidar", tf(1.0)).unwrap();
    b.insert(W, "map", "x", 0, tf(1.0)).unwrap();
    let [odom, base, lidar, map] = ["odom", "base", "lidar", "map"].map(|f| id(&b, f));
    let label = FrameLabel::new;
    // (error, its plain text, its text described by the buffer)
    let cases: [(TfError, &str, &str); 14] = [
        (
            TfError::SelfEdge {
                frame: label("base"),
            },
            "transform `base` to `base` is a self edge: a frame cannot be its own parent",
            "",
        ),
        (
            TfError::SecondParent {
                child: base,
                parent: label("map"),
                existing_parent: odom,
            },
            "frame `#1` already has the parent `#0`; a transform from `map` would give it a \
             second parent",
            "frame `base` already has the parent `odom`; a transform from `map` would give it a \
             second parent",
        ),
        (
            TfError::Cycle {
                parent: lidar,
                child: odom,
            },
            "transform `#2` to `#0` would close a cycle: `#2` descends from `#0`",
            "transform `lidar` to `odom` would close a cycle: `lidar` descends from `odom`",
        ),
        (
            TfError::EdgeIsStatic {
                parent: base,
                child: lidar,
            },
            "",
            "transform `base` to `lidar` is static; a dynamic transform or an edge policy on a \
             static edge is refused",
        ),
        (
            TfError::SecondWriter {
                parent: odom,
                child: base,
                writer: W2,
                existing_writer: W,
            },
            "",
            "transform `odom` to `base` is fed by writer 1; a transform from writer 2 is \
             refused, since two writers of one edge would interleave",
        ),
        (
            TfError::StampBeforeWindow {
                parent: odom,
                child: base,
                stamp_ns: 5,
                newest_ns: 20_000_000_000,
            },
            "",
            "transform `odom` to `base` at 5 ns is older than the retention window before the \
             newest stamp 20000000000 ns; after a clock reset or a bag loop, begin a new epoch",
        ),
        (
            TfError::FrameTableFull {
                frame: label("imu"),
                capacity: 64,
            },
            "frame `imu` does not fit: the buffer holds its capacity of 64 frames",
            "",
        ),
        (
            TfError::NameBudgetFull {
                frame: label(&"r".repeat(50)),
                len: 50,
                budget: 4_096,
            },
            "frame `rrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrr...` does not fit: its 50-byte \
             id exceeds what is left of the name budget of 4096 bytes",
            "",
        ),
        (
            TfError::UnknownFrame {
                frame: label("/imu"),
            },
            "frame `imu` is unknown: no transform naming it has been received",
            "",
        ),
        (
            TfError::NoPath {
                parent: map,
                child: lidar,
            },
            "no path between frames `#3` and `#2`: they are in separate trees",
            "no path between frames `map` and `lidar`: they are in separate trees",
        ),
        (
            TfError::EdgeEmpty {
                parent: odom,
                child: base,
            },
            "",
            "transform `odom` to `base` has no sample yet in this epoch",
        ),
        (
            TfError::NotYetAvailable {
                parent: odom,
                child: base,
                requested_ns: 301,
                newest_ns: 300,
            },
            "",
            "transform `odom` to `base` at 301 ns is not yet available: the newest buffered \
             stamp is 300 ns, and the edge's policy does not reach that far",
        ),
        (
            TfError::OutOfRange {
                parent: odom,
                child: base,
                requested_ns: 99,
                oldest_ns: 100,
                newest_ns: 300,
            },
            "",
            "transform `odom` to `base` at 99 ns is out of range: the buffered stamps run from \
             100 ns to 300 ns",
        ),
        (
            TfError::ChainMath {
                parent: map,
                child: lidar,
            },
            "",
            "transform `map` to `lidar` could not be composed along its chain: translation has \
             a non-finite component (NaN or infinity)",
        ),
    ];
    for (err, plain, described) in cases {
        // An empty plain text means "the described text with handles";
        // an empty described text means "the same as the plain text".
        if !plain.is_empty() {
            assert_eq!(err.to_string(), plain);
        }
        let described = if described.is_empty() {
            plain
        } else {
            described
        };
        assert_eq!(b.describe(&err).to_string(), described);
    }
    assert_eq!(TfError::EmptyFrameId.to_string(), "frame id is empty");
    // A handle the buffer never issued prints as a handle either way.
    let empty = TfBuffer::new();
    assert_eq!(
        empty
            .describe(&TfError::UnknownFrameId { frame: map })
            .to_string(),
        "frame handle #3 was not issued by this buffer"
    );
}

#[test]
fn a_frame_label_keeps_a_prefix_cut_at_a_character_boundary() {
    let short = FrameLabel::new("//base_link");
    assert_eq!(short.as_str(), "base_link");
    assert!(!short.is_truncated());
    assert_eq!(short, "base_link");
    assert_ne!(short, "base");
    let unsized_eq = <FrameLabel as PartialEq<str>>::eq;
    assert!(unsized_eq(&short, "base_link"));
    assert!(!unsized_eq(&short, "base"));
    assert_eq!(format!("{short:?}"), "\"base_link\"");
    // 47 bytes of `a` and a three-byte euro sign are 50 bytes; byte 48 is
    // inside the sign, so the label keeps the 47 `a`s.
    let straddles = format!("{}\u{20ac}", "a".repeat(47));
    let cut = FrameLabel::new(&straddles);
    assert_eq!(cut.as_str(), "a".repeat(47));
    assert!(cut.is_truncated());
    // A cut label never equals a str, not even its own prefix.
    assert_ne!(cut, "a".repeat(47).as_str());
    assert_eq!(cut.to_string(), format!("{}...", "a".repeat(47)));
    // Exactly 48 bytes is whole.
    let whole = FrameLabel::new(&"b".repeat(48));
    assert!(!whole.is_truncated());
}

#[test]
fn the_error_stays_small_enough_to_return_by_value() {
    // Handles and inline labels keep a failed lookup free of allocation;
    // clippy's `result_large_err` lint fires at 128 bytes, in this crate and
    // in every node crate that returns the error.
    assert!(
        std::mem::size_of::<TfError>() < 128,
        "{}",
        std::mem::size_of::<TfError>()
    );
    assert!(std::mem::size_of::<Result<Transform, TfError>>() < 128);
}
