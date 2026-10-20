// SPDX-License-Identifier: AGPL-3.0-only
//! Feeding the tf buffer through its public surface: what an insert
//! records, how a full ring evicts, and every refusal, each checked to
//! leave the buffer exactly as it was (a refusal that half-wrote is a
//! corruption defect wearing an error message).
//!
//! Expected values are written by hand from the rules in the crate docs,
//! never read off the buffer.

use cerulion_tf::{EdgeInfo, TfBuffer, TfError, Transform, UnitQuaternion, MAX_FRAME_ID_LEN};
use std::num::NonZeroUsize;

fn tf(x: f64) -> Transform {
    Transform::new([x, 0.0, 0.0], UnitQuaternion::IDENTITY).expect("a valid hand-written transform")
}

fn nz(n: usize) -> NonZeroUsize {
    NonZeroUsize::new(n).expect("a nonzero test capacity")
}

/// A buffer with room for `frames` frames and `samples` samples per edge.
fn small(frames: usize, samples: usize) -> TfBuffer {
    TfBuffer::with_capacity(nz(frames), nz(samples))
}

fn edge<'a>(b: &'a TfBuffer, child: &str) -> EdgeInfo<'a> {
    b.edge(child).expect("the edge exists")
}

// ------------------------------------------------------------------- happy

#[test]
fn a_fresh_buffer_knows_no_frame_and_has_evicted_nothing() {
    let b = TfBuffer::new();
    assert_eq!(b.frame_count(), 0);
    assert_eq!(b.frame_capacity(), cerulion_tf::DEFAULT_FRAME_CAPACITY);
    assert_eq!(b.samples_per_edge(), cerulion_tf::DEFAULT_SAMPLES_PER_EDGE);
    assert_eq!(b.evictions(), 0);
    assert!(!b.contains_frame("odom"));
    assert_eq!(b.edge("odom"), None);
}

#[test]
fn one_dynamic_insert_registers_both_frames_and_one_sample() {
    let mut b = TfBuffer::new();
    b.insert("odom", "base", 1_000, tf(1.0)).unwrap();
    assert_eq!(b.frame_count(), 2);
    assert!(b.contains_frame("odom") && b.contains_frame("base"));
    assert_eq!(
        edge(&b, "base"),
        EdgeInfo {
            parent: "odom",
            is_static: false,
            samples: 1,
            oldest_ns: Some(1_000),
            newest_ns: Some(1_000),
        }
    );
    // A root has no edge of its own.
    assert_eq!(b.edge("odom"), None);
}

#[test]
fn a_static_insert_records_the_edge_without_a_ring_sample() {
    let mut b = TfBuffer::new();
    b.insert_static("base", "lidar", tf(0.17)).unwrap();
    assert_eq!(
        edge(&b, "lidar"),
        EdgeInfo {
            parent: "base",
            is_static: true,
            samples: 0,
            oldest_ns: None,
            newest_ns: None,
        }
    );
}

#[test]
fn a_rebroadcast_static_replaces_the_value_and_grows_nothing() {
    let mut b = TfBuffer::new();
    b.insert_static("base", "lidar", tf(0.17)).unwrap();
    b.insert_static("base", "lidar", tf(0.25)).unwrap();
    assert_eq!(b.frame_count(), 2);
    assert_eq!(edge(&b, "lidar").samples, 0);
    assert_eq!(b.lookup_latest("base", "lidar").unwrap(), tf(0.25));
}

#[test]
fn the_same_stamp_twice_keeps_one_sample_with_the_later_value() {
    let mut b = TfBuffer::new();
    b.insert("odom", "base", 1_000, tf(1.0)).unwrap();
    b.insert("odom", "base", 1_000, tf(2.0)).unwrap();
    assert_eq!(edge(&b, "base").samples, 1);
    assert_eq!(b.lookup("odom", "base", 1_000).unwrap(), tf(2.0));
}

#[test]
fn out_of_order_stamps_are_kept_sorted() {
    let mut b = TfBuffer::new();
    b.insert("odom", "base", 3_000, tf(3.0)).unwrap();
    b.insert("odom", "base", 1_000, tf(1.0)).unwrap();
    b.insert("odom", "base", 2_000, tf(2.0)).unwrap();
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
    b.insert("base", "left", 10, tf(1.0)).unwrap();
    b.insert("base", "right", 10, tf(-1.0)).unwrap();
    b.insert_static("base", "lidar", tf(0.17)).unwrap();
    assert_eq!(b.frame_count(), 4);
    assert_eq!(edge(&b, "left").parent, "base");
    assert_eq!(edge(&b, "right").parent, "base");
    assert_eq!(edge(&b, "lidar").parent, "base");
}

// -------------------------------------------------------------------- edge

#[test]
fn a_full_ring_evicts_its_oldest_sample_and_counts_it() {
    let mut b = small(8, 3);
    for (k, stamp) in [100u64, 200, 300].into_iter().enumerate() {
        b.insert("odom", "base", stamp, tf(k as f64)).unwrap();
    }
    assert_eq!(b.evictions(), 0);
    b.insert("odom", "base", 400, tf(3.0)).unwrap();
    let e = edge(&b, "base");
    assert_eq!(
        (e.samples, e.oldest_ns, e.newest_ns),
        (3, Some(200), Some(400))
    );
    assert_eq!(b.evictions(), 1);
    // The ring keeps 200, 300 and 400; 100 is gone.
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
fn a_sample_older_than_a_full_ring_is_dropped_and_counted() {
    let mut b = small(8, 2);
    b.insert("odom", "base", 200, tf(2.0)).unwrap();
    b.insert("odom", "base", 300, tf(3.0)).unwrap();
    b.insert("odom", "base", 100, tf(1.0)).unwrap();
    let e = edge(&b, "base");
    assert_eq!(
        (e.samples, e.oldest_ns, e.newest_ns),
        (2, Some(200), Some(300))
    );
    assert_eq!(b.evictions(), 1);
}

#[test]
fn a_full_ring_keeps_an_out_of_order_sample_that_is_newer_than_its_oldest() {
    let mut b = small(8, 3);
    b.insert("odom", "base", 100, tf(1.0)).unwrap();
    b.insert("odom", "base", 300, tf(3.0)).unwrap();
    b.insert("odom", "base", 400, tf(4.0)).unwrap();
    // 200 lands between 100 and 300; 100 is evicted to make room.
    b.insert("odom", "base", 200, tf(2.0)).unwrap();
    let e = edge(&b, "base");
    assert_eq!(
        (e.samples, e.oldest_ns, e.newest_ns),
        (3, Some(200), Some(400))
    );
    assert_eq!(b.evictions(), 1);
    assert_eq!(b.lookup("odom", "base", 250).unwrap(), tf(2.5));
}

#[test]
fn eviction_counts_every_dropped_sample_across_edges() {
    let mut b = small(8, 1);
    b.insert("odom", "base", 1, tf(1.0)).unwrap();
    b.insert("odom", "base", 2, tf(2.0)).unwrap();
    b.insert("base", "arm", 1, tf(1.0)).unwrap();
    b.insert("base", "arm", 2, tf(2.0)).unwrap();
    b.insert("base", "arm", 3, tf(3.0)).unwrap();
    assert_eq!(b.evictions(), 3);
}

#[test]
fn a_static_insert_on_a_dynamic_edge_clears_its_history_and_wins() {
    let mut b = TfBuffer::new();
    b.insert("base", "lidar", 100, tf(1.0)).unwrap();
    b.insert("base", "lidar", 200, tf(2.0)).unwrap();
    b.insert_static("base", "lidar", tf(0.17)).unwrap();
    assert_eq!(
        edge(&b, "lidar"),
        EdgeInfo {
            parent: "base",
            is_static: true,
            samples: 0,
            oldest_ns: None,
            newest_ns: None,
        }
    );
    assert_eq!(b.evictions(), 2, "the two cleared samples are evictions");
    // At any time, including one the ring never held.
    assert_eq!(b.lookup("base", "lidar", 7).unwrap(), tf(0.17));
    assert_eq!(b.lookup("base", "lidar", 10_000).unwrap(), tf(0.17));
}

#[test]
fn a_static_insert_counts_every_dynamic_sample_it_clears() {
    let mut b = TfBuffer::new();
    for stamp in [100u64, 200, 300] {
        b.insert("base", "lidar", stamp, tf(1.0)).unwrap();
    }
    assert_eq!(b.evictions(), 0);
    b.insert_static("base", "lidar", tf(0.17)).unwrap();
    assert_eq!(b.evictions(), 3, "three cleared samples, three evictions");
    // A re-broadcast static has nothing left to clear.
    b.insert_static("base", "lidar", tf(0.17)).unwrap();
    assert_eq!(b.evictions(), 3);
    // A static edge that never had history clears nothing.
    b.insert_static("base", "camera", tf(0.27)).unwrap();
    assert_eq!(b.evictions(), 3);
}

#[test]
fn frame_ids_at_the_limit_are_accepted_and_counted_in_bytes() {
    let mut b = TfBuffer::new();
    let longest = "x".repeat(MAX_FRAME_ID_LEN);
    b.insert("odom", &longest, 1, tf(1.0)).unwrap();
    assert!(b.contains_frame(&longest));
    // Ten three-byte characters are 30 bytes, not 10.
    let thirty_bytes = "\u{20ac}".repeat(10);
    assert_eq!(thirty_bytes.len(), 30);
    b.insert("odom", &thirty_bytes, 1, tf(1.0)).unwrap();
    assert!(b.contains_frame(&thirty_bytes));
}

#[test]
fn leading_slashes_are_removed_so_slash_odom_and_odom_name_one_frame() {
    let mut b = TfBuffer::new();
    b.insert("/odom", "/base_link", 1_000, tf(1.0)).unwrap();
    assert_eq!(b.frame_count(), 2);
    assert!(b.contains_frame("odom") && b.contains_frame("/odom"));
    assert_eq!(edge(&b, "base_link").parent, "odom");
    assert_eq!(edge(&b, "/base_link"), edge(&b, "base_link"));
    // The bare id lands on the same edge: one ring, two samples.
    b.insert("odom", "base_link", 2_000, tf(2.0)).unwrap();
    assert_eq!(b.frame_count(), 2);
    assert_eq!(edge(&b, "base_link").samples, 2);
    assert_eq!(
        b.lookup("/odom", "/base_link", 1_500).unwrap(),
        b.lookup("odom", "base_link", 1_500).unwrap()
    );
    assert_eq!(b.lookup("odom", "base_link", 1_500).unwrap(), tf(1.5));
    // Every leading slash goes: `//odom` is the frame `odom`, so a stored
    // name parses back to the frame it came from.
    let n: cerulion_tf::FrameName = "//odom".parse().unwrap();
    assert_eq!(n.as_str(), "odom");
    b.insert("//odom", "lidar", 1_000, tf(0.5)).unwrap();
    assert_eq!(b.frame_count(), 3, "`//odom` is the existing `odom`");
    let parent = edge(&b, "lidar").parent.to_string();
    assert_eq!(parent, "odom");
    assert_eq!(b.lookup(&parent, "lidar", 1_000).unwrap(), tf(0.5));
    // The limit counts the id after the slash.
    let longest = format!("/{}", "x".repeat(MAX_FRAME_ID_LEN));
    b.insert("odom", &longest, 1, tf(1.0)).unwrap();
    assert!(b.contains_frame(&longest[1..]));
}

#[test]
fn a_slash_prefixed_self_edge_and_a_bare_slash_are_refused() {
    refusal_leaves_the_buffer_untouched(
        |b| b.insert("/base", "base", 1, tf(1.0)),
        TfError::SelfEdge {
            frame: "base".parse().unwrap(),
        },
    );
    refusal_leaves_the_buffer_untouched(
        |b| b.insert("odom", "/", 1, tf(1.0)),
        TfError::EmptyFrameId,
    );
}

#[test]
fn the_buffer_fills_to_exactly_its_frame_capacity() {
    let mut b = small(3, 4);
    b.insert("a", "b", 1, tf(1.0)).unwrap();
    b.insert("b", "c", 1, tf(1.0)).unwrap();
    assert_eq!(b.frame_count(), 3);
    assert_eq!(
        b.insert("c", "d", 1, tf(1.0)),
        Err(TfError::FrameTableFull {
            frame: "d".parse().unwrap(),
            capacity: 3
        })
    );
}

#[test]
fn an_edge_from_a_known_frame_to_another_trees_root_joins_the_trees() {
    // `map -> odom` and `base -> lidar` are two trees; `odom -> base` hangs
    // the second under the first. `base` is a root with a child, so the
    // cycle check walks up from `odom` and must find no `base` there.
    let mut b = TfBuffer::new();
    b.insert("map", "odom", 1, tf(1.0)).unwrap();
    b.insert("base", "lidar", 1, tf(1.0)).unwrap();
    b.insert("odom", "base", 1, tf(1.0)).unwrap();
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
        b.insert("odom", "base", 1, transform)
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
    b.insert("odom", "base", 1, tf(1.0)).unwrap();
    assert_eq!(b.frame_count(), 2);
}

// --------------------------------------------------------- every error arm

/// Runs `refused` against a buffer holding `odom -> base` and `base -> lidar`
/// (static) and checks the refusal leaves both edges and the counters as
/// they were.
fn refusal_leaves_the_buffer_untouched(
    refused: impl FnOnce(&mut TfBuffer) -> Result<(), TfError>,
    want: TfError,
) {
    let mut b = small(4, 4);
    b.insert("odom", "base", 100, tf(1.0)).unwrap();
    b.insert("odom", "base", 200, tf(2.0)).unwrap();
    b.insert_static("base", "lidar", tf(0.17)).unwrap();
    let before = snapshot(&b);
    assert_eq!(refused(&mut b), Err(want));
    assert_eq!(before, snapshot(&b), "the refusal changed the buffer");
}

/// Everything observable about the fixture buffer, owned so it can be
/// compared across a mutable borrow.
type Snapshot = (usize, u64, OwnedEdge, OwnedEdge, Transform, Transform);
type OwnedEdge = (String, bool, usize, Option<u64>, Option<u64>);

fn snapshot(b: &TfBuffer) -> Snapshot {
    let own = |e: EdgeInfo<'_>| {
        (
            e.parent.to_string(),
            e.is_static,
            e.samples,
            e.oldest_ns,
            e.newest_ns,
        )
    };
    (
        b.frame_count(),
        b.evictions(),
        own(edge(b, "base")),
        own(edge(b, "lidar")),
        b.lookup("odom", "base", 150).unwrap(),
        // The static edge's value, so a refusal that overwrites it fails.
        b.lookup("base", "lidar", 150).unwrap(),
    )
}

#[test]
fn an_empty_frame_id_is_refused() {
    refusal_leaves_the_buffer_untouched(
        |b| b.insert("", "base", 1, tf(1.0)),
        TfError::EmptyFrameId,
    );
    refusal_leaves_the_buffer_untouched(
        |b| b.insert("odom", "", 1, tf(1.0)),
        TfError::EmptyFrameId,
    );
    refusal_leaves_the_buffer_untouched(
        |b| b.insert_static("", "", tf(1.0)),
        TfError::EmptyFrameId,
    );
}

#[test]
fn a_frame_id_over_the_limit_is_refused() {
    let long = "y".repeat(MAX_FRAME_ID_LEN + 1);
    refusal_leaves_the_buffer_untouched(
        |b| b.insert("odom", &long, 1, tf(1.0)),
        TfError::FrameIdTooLong {
            prefix: "y".repeat(MAX_FRAME_ID_LEN).parse().unwrap(),
            len: MAX_FRAME_ID_LEN + 1,
        },
    );
    let huge = "z".repeat(300);
    refusal_leaves_the_buffer_untouched(
        |b| b.insert_static(&huge, "base", tf(1.0)),
        TfError::FrameIdTooLong {
            prefix: "z".repeat(MAX_FRAME_ID_LEN).parse().unwrap(),
            len: 300,
        },
    );
    // The prefix is cut at a character boundary: 35 bytes of `a` and a
    // three-byte euro sign are 38 bytes, and byte 36 is inside the sign.
    let straddles = format!("{}\u{20ac}", "a".repeat(35));
    assert_eq!(straddles.len(), 38);
    refusal_leaves_the_buffer_untouched(
        |b| b.insert("odom", &straddles, 1, tf(1.0)),
        TfError::FrameIdTooLong {
            prefix: "a".repeat(35).parse().unwrap(),
            len: 38,
        },
    );
}

#[test]
fn a_self_edge_is_refused() {
    refusal_leaves_the_buffer_untouched(
        |b| b.insert("base", "base", 1, tf(1.0)),
        TfError::SelfEdge {
            frame: "base".parse().unwrap(),
        },
    );
    refusal_leaves_the_buffer_untouched(
        |b| b.insert_static("new", "new", tf(1.0)),
        TfError::SelfEdge {
            frame: "new".parse().unwrap(),
        },
    );
}

#[test]
fn a_second_parent_for_a_child_is_refused() {
    refusal_leaves_the_buffer_untouched(
        |b| b.insert("map", "base", 1, tf(1.0)),
        TfError::SecondParent {
            child: "base".parse().unwrap(),
            parent: "map".parse().unwrap(),
            existing_parent: "odom".parse().unwrap(),
        },
    );
    // A static edge is protected the same way.
    refusal_leaves_the_buffer_untouched(
        |b| b.insert_static("odom", "lidar", tf(1.0)),
        TfError::SecondParent {
            child: "lidar".parse().unwrap(),
            parent: "odom".parse().unwrap(),
            existing_parent: "base".parse().unwrap(),
        },
    );
}

#[test]
fn an_edge_that_would_close_a_cycle_is_refused() {
    // lidar -> odom would make odom descend from itself.
    refusal_leaves_the_buffer_untouched(
        |b| b.insert("lidar", "odom", 1, tf(1.0)),
        TfError::Cycle {
            parent: "lidar".parse().unwrap(),
            child: "odom".parse().unwrap(),
        },
    );
    refusal_leaves_the_buffer_untouched(
        |b| b.insert_static("base", "odom", tf(1.0)),
        TfError::Cycle {
            parent: "base".parse().unwrap(),
            child: "odom".parse().unwrap(),
        },
    );
}

#[test]
fn a_dynamic_transform_on_a_static_edge_is_refused() {
    refusal_leaves_the_buffer_untouched(
        |b| b.insert("base", "lidar", 1, tf(1.0)),
        TfError::EdgeIsStatic {
            parent: "base".parse().unwrap(),
            child: "lidar".parse().unwrap(),
        },
    );
}

#[test]
fn a_frame_beyond_the_capacity_is_refused_naming_the_frame_that_does_not_fit() {
    // The fixture holds 3 of 4 frames: one more fits, two do not.
    refusal_leaves_the_buffer_untouched(
        |b| b.insert("x", "y", 1, tf(1.0)),
        TfError::FrameTableFull {
            frame: "x".parse().unwrap(),
            capacity: 4,
        },
    );
    // One new frame fits; the refusal names the child when the parent is known.
    let mut b = small(4, 4);
    b.insert("odom", "base", 1, tf(1.0)).unwrap();
    b.insert("base", "lidar", 1, tf(1.0)).unwrap();
    b.insert("base", "cam", 1, tf(1.0)).unwrap();
    assert_eq!(
        b.insert("base", "imu", 1, tf(1.0)),
        Err(TfError::FrameTableFull {
            frame: "imu".parse().unwrap(),
            capacity: 4
        })
    );
    assert_eq!(b.frame_count(), 4);
}

// ----------------------------------------------------- the error text contract

#[test]
fn every_refusal_names_the_frames_involved() {
    let name = |s: &str| s.parse::<cerulion_tf::FrameName>().unwrap();
    assert_eq!(MAX_FRAME_ID_LEN, 36, "the documented frame id limit");
    let cases: [(TfError, &str); 12] = [
        (TfError::EmptyFrameId, "frame id is empty"),
        (
            TfError::FrameIdTooLong {
                prefix: name("robot_0/camera_aligned_depth_to_colo"),
                len: 43,
            },
            "frame id `robot_0/camera_aligned_depth_to_colo...` is 43 bytes long; the limit is \
             36 bytes",
        ),
        (
            TfError::SelfEdge {
                frame: name("base"),
            },
            "transform `base` to `base` is a self edge: a frame cannot be its own parent",
        ),
        (
            TfError::SecondParent {
                child: name("base"),
                parent: name("map"),
                existing_parent: name("odom"),
            },
            "frame `base` already has the parent `odom`; a transform from `map` would give it a \
             second parent",
        ),
        (
            TfError::Cycle {
                parent: name("lidar"),
                child: name("odom"),
            },
            "transform `lidar` to `odom` would close a cycle: `lidar` descends from `odom`",
        ),
        (
            TfError::EdgeIsStatic {
                parent: name("base"),
                child: name("lidar"),
            },
            "transform `base` to `lidar` is static; a dynamic transform on a static edge is \
             refused",
        ),
        (
            TfError::FrameTableFull {
                frame: name("imu"),
                capacity: 64,
            },
            "frame `imu` does not fit: the buffer holds its capacity of 64 frames",
        ),
        (
            TfError::UnknownFrame { frame: name("map") },
            "frame `map` is unknown: no transform naming it has been received",
        ),
        (
            TfError::NoPath {
                parent: name("map"),
                child: name("lidar"),
            },
            "no path between frames `map` and `lidar`: they are in separate trees",
        ),
        (
            TfError::NotYetAvailable {
                parent: name("odom"),
                child: name("base"),
                requested_ns: 301,
                newest_ns: 300,
            },
            "transform `odom` to `base` at 301 ns is not yet available: the newest buffered \
             stamp is 300 ns, and lookups never extrapolate",
        ),
        (
            TfError::OutOfRange {
                parent: name("odom"),
                child: name("base"),
                requested_ns: 99,
                oldest_ns: 100,
                newest_ns: 300,
            },
            "transform `odom` to `base` at 99 ns is out of range: the buffered stamps run from \
             100 ns to 300 ns, and lookups never extrapolate",
        ),
        (
            TfError::ChainMath {
                parent: name("map"),
                child: name("lidar"),
            },
            "transform `map` to `lidar` could not be composed along its chain: translation has \
             a non-finite component (NaN or infinity)",
        ),
    ];
    for (err, text) in cases {
        assert_eq!(err.to_string(), text);
    }
}

#[test]
fn a_frame_name_round_trips_and_compares_with_str() {
    let n: cerulion_tf::FrameName = "base_link".parse().unwrap();
    assert_eq!(n.as_str(), "base_link");
    assert_eq!(n, "base_link");
    assert_ne!(n, "base");
    // The unsized `str` comparison is its own impl: both answers pinned.
    let unsized_eq = <cerulion_tf::FrameName as PartialEq<str>>::eq;
    assert!(unsized_eq(&n, "base_link"));
    assert!(!unsized_eq(&n, "base"));
    let as_ref: &str = n.as_ref();
    assert_eq!(as_ref, "base_link");
    assert_eq!(n.to_string(), "base_link");
    assert_eq!(format!("{n:?}"), "\"base_link\"");
    assert_eq!(
        "".parse::<cerulion_tf::FrameName>(),
        Err(TfError::EmptyFrameId)
    );
    assert_eq!(
        "w".repeat(MAX_FRAME_ID_LEN + 1)
            .parse::<cerulion_tf::FrameName>(),
        Err(TfError::FrameIdTooLong {
            prefix: "w".repeat(MAX_FRAME_ID_LEN).parse().unwrap(),
            len: MAX_FRAME_ID_LEN + 1
        })
    );
    // Leading slashes are removed; the name then compares as its own str.
    let slashed: cerulion_tf::FrameName = "/base_link".parse().unwrap();
    assert_eq!(slashed, n);
    assert_eq!(slashed, "base_link");
    assert_eq!(
        "/".parse::<cerulion_tf::FrameName>(),
        Err(TfError::EmptyFrameId)
    );
    assert_eq!(
        "///".parse::<cerulion_tf::FrameName>(),
        Err(TfError::EmptyFrameId)
    );
}

#[test]
fn the_error_stays_small_enough_to_return_by_value() {
    // The inline frame names are what keep a failed lookup free of
    // allocation; the price is paid in bytes, and clippy's
    // `result_large_err` lint fires at 128, in this crate and in every
    // node crate that returns the error.
    assert!(
        std::mem::size_of::<TfError>() < 128,
        "{}",
        std::mem::size_of::<TfError>()
    );
    assert!(std::mem::size_of::<Result<Transform, TfError>>() < 128);
}
