// SPDX-License-Identifier: AGPL-3.0-only
//! `lookup(parent, child, time)` and `lookup_latest` through the public
//! surface: exact hits, interpolation, chains through a common ancestor,
//! the inverse direction, static edges, and every refusal.
//!
//! Every expected value is written by hand from the geometry (a 90 degree
//! turn about z sends +x to +y; a half-angle sine for an interpolated
//! rotation) or from an exact arithmetic identity, never read off the
//! buffer. Stamps are nanoseconds.

use cerulion_tf::{
    EdgePolicy, FrameId, FrameLabel, LatestTransform, TfBuffer, TfBufferConfig, TfError, Transform,
    UnitQuaternion,
};
use std::f64::consts::FRAC_1_SQRT_2;
use std::num::{NonZeroU32, NonZeroUsize};
use std::time::Duration;

const TOL: f64 = 1e-15;

/// sin and cos of 22.5 degrees (a 45 degree turn).
const SIN_22_5: f64 = 0.382_683_432_365_089_8;
const COS_22_5: f64 = 0.923_879_532_511_286_7;
/// sin and cos of 15 degrees (a 30 degree turn).
const SIN_15: f64 = 0.258_819_045_102_520_76;
const COS_15: f64 = 0.965_925_826_289_068_3;

fn q(x: f64, y: f64, z: f64, w: f64) -> UnitQuaternion {
    UnitQuaternion::new(x, y, z, w).expect("a valid hand-written quaternion")
}

fn tf(t: [f64; 3], r: UnitQuaternion) -> Transform {
    Transform::new(t, r).expect("a valid hand-written transform")
}

fn shift(x: f64) -> Transform {
    tf([x, 0.0, 0.0], UnitQuaternion::IDENTITY)
}

/// 90 degrees about +z.
fn z90() -> UnitQuaternion {
    q(0.0, 0.0, FRAC_1_SQRT_2, FRAC_1_SQRT_2)
}

/// Minus 90 degrees about +z.
fn z_minus_90() -> UnitQuaternion {
    q(0.0, 0.0, -FRAC_1_SQRT_2, FRAC_1_SQRT_2)
}

fn id(b: &TfBuffer, frame: &str) -> FrameId {
    b.frame_id(frame).expect("a registered frame")
}

fn assert_close3(got: [f64; 3], want: [f64; 3], tol: f64) {
    for i in 0..3 {
        assert!(
            (got[i] - want[i]).abs() <= tol,
            "component {i}: got {got:?}, want {want:?} (tol {tol:e})"
        );
    }
}

fn assert_close4(got: [f64; 4], want: [f64; 4], tol: f64) {
    for i in 0..4 {
        assert!(
            (got[i] - want[i]).abs() <= tol,
            "component {i}: got {got:?}, want {want:?} (tol {tol:e})"
        );
    }
}

fn assert_close(got: Transform, want: Transform, tol: f64) {
    assert_close3(got.translation(), want.translation(), tol);
    assert_close4(got.rotation().to_array(), want.rotation().to_array(), tol);
}

/// `odom -> base` turning from identity at 0 ns to 90 about z at 1000 ns
/// while moving 2 m along +x.
fn turning_edge() -> TfBuffer {
    let mut b = TfBuffer::new();
    b.insert(
        "odom",
        "base",
        0,
        tf([0.0, 0.0, 0.0], UnitQuaternion::IDENTITY),
    )
    .unwrap();
    b.insert("odom", "base", 1_000, tf([2.0, 0.0, 0.0], z90()))
        .unwrap();
    b
}

// ------------------------------------------------------------------- happy

#[test]
fn an_exact_stamp_hit_returns_the_sample_bit_for_bit() {
    let mut b = TfBuffer::new();
    let sample = tf([1.5, -2.25, 3.0], q(0.1, 0.2, 0.3, 0.9));
    b.insert("odom", "base", 100, shift(0.0)).unwrap();
    b.insert("odom", "base", 200, sample).unwrap();
    b.insert("odom", "base", 300, shift(9.0)).unwrap();
    let got = b.lookup("odom", "base", 200).unwrap();
    assert_eq!(got.translation(), sample.translation());
    assert_eq!(
        got.rotation().to_array().map(f64::to_bits),
        sample.rotation().to_array().map(f64::to_bits)
    );
    // The ends of the ring are exact hits too.
    assert_eq!(b.lookup("odom", "base", 100).unwrap(), shift(0.0));
    assert_eq!(b.lookup("odom", "base", 300).unwrap(), shift(9.0));
}

#[test]
fn the_midpoint_is_a_linear_translation_and_a_slerp_rotation() {
    let b = turning_edge();
    let got = b.lookup("odom", "base", 500).unwrap();
    // Half of 2 m is 1 m; half of 90 about z is 45 about z.
    assert_eq!(got.translation(), [1.0, 0.0, 0.0]);
    assert_close4(
        got.rotation().to_array(),
        [0.0, 0.0, SIN_22_5, COS_22_5],
        TOL,
    );
}

#[test]
fn a_third_of_the_way_is_a_third_of_the_turn() {
    let mut b = TfBuffer::new();
    b.insert(
        "odom",
        "base",
        0,
        tf([0.0, 0.0, 0.0], UnitQuaternion::IDENTITY),
    )
    .unwrap();
    b.insert("odom", "base", 3_000, tf([3.0, 0.0, 0.0], z90()))
        .unwrap();
    let got = b.lookup("odom", "base", 1_000).unwrap();
    assert_close3(got.translation(), [1.0, 0.0, 0.0], TOL);
    assert_close4(got.rotation().to_array(), [0.0, 0.0, SIN_15, COS_15], TOL);
}

#[test]
fn interpolation_uses_the_bracketing_pair_not_the_ring_ends() {
    let mut b = TfBuffer::new();
    // The 200 sample is off the line through the ring ends, so a blend of
    // the ends (0 at 0, 30 at 300) gives a different answer at every
    // interior stamp: 12.5 at 125 and 27.5 at 275.
    for (stamp, x) in [(0u64, 0.0), (100, 10.0), (200, 40.0), (300, 30.0)] {
        b.insert("odom", "base", stamp, shift(x)).unwrap();
    }
    // 125 is a quarter of the way from the 100 sample to the 200 sample:
    // 10 + 0.25 * (40 - 10) = 17.5.
    assert_eq!(b.lookup("odom", "base", 125).unwrap(), shift(17.5));
    // 275 is three quarters of the way from 200 to 300:
    // 40 + 0.75 * (30 - 40) = 32.5.
    assert_eq!(b.lookup("odom", "base", 275).unwrap(), shift(32.5));
}

#[test]
fn a_chain_through_the_root_composes_every_edge() {
    // map -> odom: 10 m along +x and a 90 turn about z.
    // odom -> base: 1 m along +x (dynamic, constant over the ring).
    // base -> lidar: the static mount, 0.17 m forward and 0.11 m up.
    let mut b = TfBuffer::new();
    b.insert("map", "odom", 0, tf([10.0, 0.0, 0.0], z90()))
        .unwrap();
    b.insert("map", "odom", 1_000, tf([10.0, 0.0, 0.0], z90()))
        .unwrap();
    b.insert("odom", "base", 0, shift(1.0)).unwrap();
    b.insert("odom", "base", 1_000, shift(1.0)).unwrap();
    b.insert_static(
        "base",
        "lidar",
        tf([0.17, 0.0, 0.11], UnitQuaternion::IDENTITY),
    )
    .unwrap();

    let got = b.lookup("map", "lidar", 500).unwrap();
    // The lidar origin in odom is (1.17, 0, 0.11); the 90 turn sends +x to
    // +y, then the 10 m offset: (10, 1.17, 0.11).
    assert_close3(got.translation(), [10.0, 1.17, 0.11], 1e-14);
    assert_close4(got.rotation().to_array(), z90().to_array(), TOL);
    // The same chain, one hop shorter.
    let got = b.lookup("map", "base", 500).unwrap();
    assert_close3(got.translation(), [10.0, 1.0, 0.0], 1e-14);
}

#[test]
fn two_siblings_resolve_through_their_common_parent() {
    // base -> lidar is turned 90 about z; base -> camera is not.
    let mut b = TfBuffer::new();
    b.insert_static("base", "lidar", tf([0.17, 0.0, 0.11], z90()))
        .unwrap();
    b.insert_static(
        "base",
        "camera",
        tf([0.27, 0.0, 0.05], UnitQuaternion::IDENTITY),
    )
    .unwrap();
    let got = b.lookup("lidar", "camera", 42).unwrap();
    // The camera sits (0.10, 0, -0.06) from the lidar in base axes; in the
    // lidar's axes (turned 90) that is (0, -0.10, -0.06), and the camera is
    // turned -90 relative to the lidar.
    assert_close3(got.translation(), [0.0, -0.10, -0.06], 1e-15);
    assert_close4(got.rotation().to_array(), z_minus_90().to_array(), TOL);
}

#[test]
fn the_inverse_direction_is_the_inverse_transform() {
    // odom -> base: 1 m along +x, 2 m along +y, turned 90 about z.
    let mut b = TfBuffer::new();
    b.insert("odom", "base", 7, tf([1.0, 2.0, 0.0], z90()))
        .unwrap();
    let got = b.lookup("base", "odom", 7).unwrap();
    // The odom origin seen from base: undo the turn (-90, sending (1, 2)
    // to (2, -1)) and negate: (-2, 1, 0).
    assert_close3(got.translation(), [-2.0, 1.0, 0.0], 1e-15);
    assert_close4(got.rotation().to_array(), z_minus_90().to_array(), TOL);
    // And it is exactly what the math crate's inverse gives.
    assert_eq!(got, tf([1.0, 2.0, 0.0], z90()).inverse().unwrap());
}

#[test]
fn a_frame_to_itself_is_the_identity() {
    let b = turning_edge();
    assert_eq!(b.lookup("odom", "odom", 500).unwrap(), Transform::IDENTITY);
    assert_eq!(b.lookup("base", "base", 500).unwrap(), Transform::IDENTITY);
    // A frame to itself crosses no edge, so the chain has no time.
    assert_eq!(
        b.lookup_latest("base", "base").unwrap(),
        LatestTransform {
            transform: Transform::IDENTITY,
            stamp_ns: None,
        }
    );
}

#[test]
fn a_static_edge_answers_at_every_time() {
    let mut b = turning_edge();
    b.insert_static("base", "lidar", shift(0.17)).unwrap();
    for stamp in [0u64, 500, 1_000, u64::MAX] {
        assert_eq!(b.lookup("base", "lidar", stamp).unwrap(), shift(0.17));
    }
    // Through a dynamic edge, the dynamic edge still rules the time.
    let got = b.lookup("odom", "lidar", 500).unwrap();
    // base at 500 ns: 1 m along +x, turned 45 about z. The lidar is 0.17 m
    // along base's +x, which at 45 degrees is (0.17 cos 45, 0.17 sin 45).
    assert_close3(
        got.translation(),
        [1.0 + 0.17 * FRAC_1_SQRT_2, 0.17 * FRAC_1_SQRT_2, 0.0],
        1e-15,
    );
    assert!(matches!(
        b.lookup("odom", "lidar", 1_001),
        Err(TfError::NotYetAvailable { .. })
    ));
}

#[test]
fn stamps_near_the_epoch_and_a_billion_seconds_later_interpolate_alike() {
    // 1.7e18 ns is a 2023 wall-clock stamp: too large for an f64 to hold
    // exactly, so the fraction must come from the differences.
    let late: u64 = 1_700_000_000_000_000_000;
    let mut early = TfBuffer::new();
    let mut later = TfBuffer::new();
    for (stamp, x) in [(0u64, 0.0), (1_000, 4.0)] {
        early.insert("odom", "base", stamp, shift(x)).unwrap();
        later
            .insert("odom", "base", late + stamp, shift(x))
            .unwrap();
    }
    let a = early.lookup("odom", "base", 250).unwrap();
    let b = later.lookup("odom", "base", late + 250).unwrap();
    assert_eq!(a, shift(1.0));
    assert_eq!(a, b);
    // One nanosecond past the newest sample is still refused at that scale.
    assert!(matches!(
        later.lookup("odom", "base", late + 1_001),
        Err(TfError::NotYetAvailable { requested_ns, newest_ns, .. })
            if requested_ns == late + 1_001 && newest_ns == late + 1_000
    ));
}

#[test]
fn a_gap_wider_than_2_to_the_53_ns_rounds_the_fraction_the_same_way_every_time() {
    // Two samples 2^54 ns apart (about 208 days): a difference this wide no
    // longer converts to f64 exactly, so the fraction rounds. The rounding
    // is IEEE-754 round-to-nearest-even, worked out by hand below, and it
    // is the same on every run.
    let span: u64 = 1 << 54;
    // The default 10 s window would expire the first sample; this buffer
    // keeps the whole span.
    let mut b = TfBuffer::with_config(TfBufferConfig {
        retention: Duration::from_nanos(span),
        ..TfBufferConfig::default()
    });
    b.insert("odom", "base", 0, shift(0.0)).unwrap();
    b.insert("odom", "base", span, shift(2.0)).unwrap();
    // 2^53 is exact: the fraction is 0.5 and the translation 1 m.
    assert_eq!(b.lookup("odom", "base", 1 << 53).unwrap(), shift(1.0));
    // 2^53 + 1 is halfway between the doubles 2^53 and 2^53 + 2; the tie
    // goes to the even one, 2^53, so the result is the same 1 m.
    assert_eq!(b.lookup("odom", "base", (1 << 53) + 1).unwrap(), shift(1.0));
    // 2^53 + 3 is halfway between 2^53 + 2 and 2^53 + 4; the even one is
    // 2^53 + 4, so the fraction is 0.5 + 2^-52 and the translation
    // 1 + 2^-51 m (f64::EPSILON is 2^-52).
    assert_eq!(
        b.lookup("odom", "base", (1 << 53) + 3).unwrap(),
        shift(1.0 + 2.0 * f64::EPSILON)
    );
}

#[test]
fn lookup_latest_resolves_at_the_newest_time_the_whole_chain_has() {
    // map -> odom has samples to 900 ns; odom -> base to 1000 ns. The
    // latest time both hold is 900 ns.
    let mut b = TfBuffer::new();
    b.insert("map", "odom", 0, shift(0.0)).unwrap();
    b.insert("map", "odom", 900, shift(9.0)).unwrap();
    b.insert("odom", "base", 0, shift(0.0)).unwrap();
    b.insert("odom", "base", 1_000, shift(10.0)).unwrap();
    let latest = b.lookup_latest("map", "base").unwrap();
    // The oldest of the newest stamps is map -> odom's 900 ns, and the
    // transform is the one `lookup` gives there.
    assert_eq!(latest.stamp_ns, Some(900));
    assert_eq!(latest.transform, b.lookup("map", "base", 900).unwrap());
    // At 900 ns: map -> odom is 9 m; odom -> base is 9 m (0.9 of 10 m).
    assert_close3(latest.transform.translation(), [18.0, 0.0, 0.0], 1e-14);
    // The inverse direction picks the same time.
    assert_eq!(
        b.lookup_latest("base", "map").unwrap(),
        LatestTransform {
            transform: b.lookup("base", "map", 900).unwrap(),
            stamp_ns: Some(900),
        }
    );
}

#[test]
fn lookup_latest_on_static_edges_only_needs_no_time() {
    let mut b = TfBuffer::new();
    b.insert_static("base", "lidar", shift(0.17)).unwrap();
    b.insert_static("base", "camera", shift(0.27)).unwrap();
    let got = b.lookup_latest("lidar", "camera").unwrap();
    assert_eq!(got.stamp_ns, None, "static edges hold at every time");
    assert_close3(got.transform.translation(), [0.10, 0.0, 0.0], 1e-15);
}

#[test]
fn lookup_latest_uses_the_single_dynamic_edges_newest_sample() {
    let b = turning_edge();
    assert_eq!(
        b.lookup_latest("odom", "base").unwrap(),
        LatestTransform {
            transform: tf([2.0, 0.0, 0.0], z90()),
            stamp_ns: Some(1_000),
        }
    );
}

// ------------------------------------------------------- the mislabelled frame

#[test]
fn swapping_parent_and_child_gives_the_inverse_never_the_same_value() {
    let b = turning_edge();
    let forward = b.lookup("odom", "base", 500).unwrap();
    let backward = b.lookup("base", "odom", 500).unwrap();
    assert_ne!(forward, backward);
    assert_close(backward, forward.inverse().unwrap(), TOL);
    // Composing the two is the identity, within rounding.
    assert_close(
        forward.compose(&backward).unwrap(),
        Transform::IDENTITY,
        1e-15,
    );
}

#[test]
fn a_misspelt_frame_is_refused_by_name() {
    let b = turning_edge();
    assert_eq!(
        b.lookup("odom", "base_link", 500),
        Err(TfError::UnknownFrame {
            frame: FrameLabel::new("base_link")
        })
    );
    assert_eq!(
        b.lookup("Odom", "base", 500),
        Err(TfError::UnknownFrame {
            frame: FrameLabel::new("Odom")
        })
    );
}

// ------------------------------------------------------------ every error arm

#[test]
fn an_unknown_frame_is_refused_naming_the_first_unknown_one() {
    let b = turning_edge();
    assert_eq!(
        b.lookup("map", "base", 500),
        Err(TfError::UnknownFrame {
            frame: FrameLabel::new("map")
        })
    );
    assert_eq!(
        b.lookup("odom", "lidar", 500),
        Err(TfError::UnknownFrame {
            frame: FrameLabel::new("lidar")
        })
    );
    // Both unknown: the parent is named.
    assert_eq!(
        b.lookup_latest("map", "lidar"),
        Err(TfError::UnknownFrame {
            frame: FrameLabel::new("map")
        })
    );
    // The empty id is refused as such; a long unknown id is named by its
    // first 48 bytes.
    assert_eq!(b.lookup("", "base", 500), Err(TfError::EmptyFrameId));
    assert_eq!(b.lookup("odom", "/", 500), Err(TfError::EmptyFrameId));
    let long = "y".repeat(60);
    let Err(TfError::UnknownFrame { frame }) = b.lookup("odom", &long, 500) else {
        panic!("a long unknown id is an unknown frame");
    };
    assert_eq!(frame.as_str(), "y".repeat(48));
    assert!(frame.is_truncated());
    assert_eq!(frame.to_string(), format!("{}...", "y".repeat(48)));
}

#[test]
fn an_empty_buffer_knows_no_frame() {
    let b = TfBuffer::new();
    assert_eq!(
        b.lookup("odom", "base", 0),
        Err(TfError::UnknownFrame {
            frame: FrameLabel::new("odom")
        })
    );
}

#[test]
fn two_trees_have_no_path() {
    let mut b = TfBuffer::new();
    b.insert("map", "odom", 0, shift(1.0)).unwrap();
    b.insert("base", "lidar", 0, shift(1.0)).unwrap();
    assert_eq!(
        b.lookup("map", "lidar", 0),
        Err(TfError::NoPath {
            parent: id(&b, "map"),
            child: id(&b, "lidar")
        })
    );
    assert_eq!(
        b.lookup_latest("odom", "base"),
        Err(TfError::NoPath {
            parent: id(&b, "odom"),
            child: id(&b, "base")
        })
    );
}

#[test]
fn one_nanosecond_before_the_oldest_sample_is_out_of_range() {
    let mut b = TfBuffer::new();
    b.insert("odom", "base", 100, shift(1.0)).unwrap();
    b.insert("odom", "base", 300, shift(3.0)).unwrap();
    assert_eq!(
        b.lookup("odom", "base", 99),
        Err(TfError::OutOfRange {
            parent: id(&b, "odom"),
            child: id(&b, "base"),
            requested_ns: 99,
            oldest_ns: 100,
            newest_ns: 300,
        })
    );
    assert!(matches!(
        b.lookup("odom", "base", 0),
        Err(TfError::OutOfRange {
            requested_ns: 0,
            ..
        })
    ));
}

#[test]
fn one_nanosecond_after_the_newest_sample_is_not_yet_available() {
    let mut b = TfBuffer::new();
    b.insert("odom", "base", 100, shift(1.0)).unwrap();
    b.insert("odom", "base", 300, shift(3.0)).unwrap();
    assert_eq!(
        b.lookup("odom", "base", 301),
        Err(TfError::NotYetAvailable {
            parent: id(&b, "odom"),
            child: id(&b, "base"),
            requested_ns: 301,
            newest_ns: 300,
        })
    );
    // The inverse direction reports the edge as stored, parent first.
    assert_eq!(
        b.lookup("base", "odom", 301),
        Err(TfError::NotYetAvailable {
            parent: id(&b, "odom"),
            child: id(&b, "base"),
            requested_ns: 301,
            newest_ns: 300,
        })
    );
}

#[test]
fn a_single_sample_answers_at_its_stamp_only() {
    let mut b = TfBuffer::new();
    b.insert("odom", "base", 500, shift(5.0)).unwrap();
    assert_eq!(b.lookup("odom", "base", 500).unwrap(), shift(5.0));
    assert_eq!(
        b.lookup("odom", "base", 499),
        Err(TfError::OutOfRange {
            parent: id(&b, "odom"),
            child: id(&b, "base"),
            requested_ns: 499,
            oldest_ns: 500,
            newest_ns: 500,
        })
    );
    assert_eq!(
        b.lookup("odom", "base", 501),
        Err(TfError::NotYetAvailable {
            parent: id(&b, "odom"),
            child: id(&b, "base"),
            requested_ns: 501,
            newest_ns: 500,
        })
    );
}

#[test]
fn a_refusal_on_a_chain_names_the_failing_edge() {
    // map -> odom covers 0..=1000 ns; odom -> base covers only 500..=600.
    let mut b = TfBuffer::new();
    b.insert("map", "odom", 0, shift(0.0)).unwrap();
    b.insert("map", "odom", 1_000, shift(1.0)).unwrap();
    b.insert("odom", "base", 500, shift(0.0)).unwrap();
    b.insert("odom", "base", 600, shift(1.0)).unwrap();
    assert_eq!(
        b.lookup("map", "base", 700),
        Err(TfError::NotYetAvailable {
            parent: id(&b, "odom"),
            child: id(&b, "base"),
            requested_ns: 700,
            newest_ns: 600,
        })
    );
    assert_eq!(
        b.lookup("map", "base", 400),
        Err(TfError::OutOfRange {
            parent: id(&b, "odom"),
            child: id(&b, "base"),
            requested_ns: 400,
            oldest_ns: 500,
            newest_ns: 600,
        })
    );
    // Inside both windows the chain resolves.
    assert!(b.lookup("map", "base", 550).is_ok());
}

#[test]
fn lookup_latest_with_windows_that_do_not_overlap_is_out_of_range() {
    // odom -> base holds 0..=100; map -> odom holds 500..=600. The newest
    // time both could have is 100, which map -> odom does not reach.
    let mut b = TfBuffer::new();
    b.insert("odom", "base", 0, shift(0.0)).unwrap();
    b.insert("odom", "base", 100, shift(1.0)).unwrap();
    b.insert("map", "odom", 500, shift(0.0)).unwrap();
    b.insert("map", "odom", 600, shift(1.0)).unwrap();
    assert_eq!(
        b.lookup_latest("map", "base"),
        Err(TfError::OutOfRange {
            parent: id(&b, "map"),
            child: id(&b, "odom"),
            requested_ns: 100,
            oldest_ns: 500,
            newest_ns: 600,
        })
    );
}

#[test]
fn a_chain_whose_translations_overflow_is_refused_naming_the_lookup_frames() {
    let mut b = TfBuffer::new();
    b.insert("a", "b", 0, shift(1e308)).unwrap();
    b.insert("b", "c", 0, shift(1e308)).unwrap();
    b.insert("a", "d", 0, shift(-1e308)).unwrap();
    assert_eq!(
        b.lookup("a", "c", 0),
        Err(TfError::ChainMath {
            parent: id(&b, "a"),
            child: id(&b, "c"),
        })
    );
    // The frames are the lookup's own, in the order it named them.
    assert_eq!(
        b.lookup("c", "a", 0),
        Err(TfError::ChainMath {
            parent: id(&b, "c"),
            child: id(&b, "a"),
        })
    );
    // Siblings through `a`: the chain `a -> c` overflows on its own.
    assert_eq!(
        b.lookup("d", "c", 0),
        Err(TfError::ChainMath {
            parent: id(&b, "d"),
            child: id(&b, "c"),
        })
    );
    assert!(matches!(
        b.lookup_latest("a", "c"),
        Err(TfError::ChainMath { .. })
    ));
    // Each edge on its own is fine.
    assert_eq!(b.lookup("a", "b", 0).unwrap(), shift(1e308));
    assert_eq!(b.lookup("a", "d", 0).unwrap(), shift(-1e308));
}

#[test]
fn lookups_never_change_the_buffer() {
    let mut b = turning_edge();
    b.insert_static("base", "lidar", shift(0.17)).unwrap();
    let before = (
        b.frame_count(),
        b.expired(),
        b.edge("base").map(|e| e.samples),
    );
    let _ = b.lookup("odom", "lidar", 500);
    let _ = b.lookup("odom", "lidar", 5_000);
    let _ = b.lookup("map", "lidar", 500);
    let _ = b.lookup_latest("lidar", "odom");
    assert_eq!(
        before,
        (
            b.frame_count(),
            b.expired(),
            b.edge("base").map(|e| e.samples)
        )
    );
}

// ------------------------------------------------- the common navigation chain

/// `map -> odom` at 1 Hz (y = the second, k) and `odom -> base` every
/// `step_ms` (x = the sample index n) for 20 s, from 0 to the last full
/// step before 20 s. Pure translations, so every composed value is exact.
fn nav_chain(b: &mut TfBuffer, step_ms: u64) {
    for k in 0..20u64 {
        b.insert(
            "map",
            "odom",
            k * 1_000_000_000,
            tf([0.0, k as f64, 0.0], UnitQuaternion::IDENTITY),
        )
        .unwrap();
    }
    for n in 0..(20_000 / step_ms) {
        b.insert("odom", "base", n * step_ms * 1_000_000, shift(n as f64))
            .unwrap();
    }
}

#[test]
fn a_1_hz_map_edge_and_a_100_hz_odom_edge_resolve_at_the_slow_edges_newest_stamp() {
    let mut b = TfBuffer::new();
    nav_chain(&mut b, 10);
    // The slow edge's newest is 19 s, inside the fast edge's 10 s window
    // (9.99 s to 19.99 s). At 19 s: y = 19, and n = 1900.
    let latest = b.lookup_latest("map", "base").unwrap();
    assert_eq!(latest.stamp_ns, Some(19_000_000_000));
    assert_eq!(latest.transform.translation(), [1_900.0, 19.0, 0.0]);
    let e = b.edge("base").unwrap();
    assert_eq!(
        (e.samples, e.oldest_ns, e.newest_ns, e.overflows),
        (1_001, Some(9_990_000_000), Some(19_990_000_000), 0)
    );
}

#[test]
fn a_1_hz_map_edge_and_a_1_khz_odom_edge_resolve_in_a_buffer_sized_for_1_khz() {
    let rate = NonZeroU32::new(1_000).unwrap();
    let mut b = TfBuffer::with_config(TfBufferConfig::new(
        NonZeroUsize::new(3).unwrap(),
        Duration::from_secs(10),
        rate,
    ));
    nav_chain(&mut b, 1);
    // At 19 s: y = 19, and n = 19 000.
    let latest = b.lookup_latest("map", "base").unwrap();
    assert_eq!(latest.stamp_ns, Some(19_000_000_000));
    assert_eq!(latest.transform.translation(), [19_000.0, 19.0, 0.0]);
    let e = b.edge("base").unwrap();
    assert_eq!(
        (e.samples, e.oldest_ns, e.overflows),
        (10_001, Some(9_999_000_000), 0)
    );
}

#[test]
fn a_1_khz_edge_in_a_buffer_sized_for_100_hz_counts_every_overflow() {
    let mut b = TfBuffer::new();
    nav_chain(&mut b, 1);
    // The ring of 1001 fills at n = 1000, a window of 1 s; nothing expires
    // inside 10 s, so each of the remaining 18 999 samples pushes one out.
    let e = b.edge("base").unwrap();
    assert_eq!(
        (e.samples, e.oldest_ns, e.newest_ns, e.overflows),
        (1_001, Some(18_999_000_000), Some(19_999_000_000), 18_999)
    );
    assert_eq!(
        b.expired(),
        9,
        "only the slow edge's 0 s to 8 s samples expire"
    );
    // The short window still covers 19 s.
    let latest = b.lookup_latest("map", "base").unwrap();
    assert_eq!(latest.stamp_ns, Some(19_000_000_000));
    assert_eq!(latest.transform.translation(), [19_000.0, 19.0, 0.0]);
}

#[test]
fn hold_last_on_the_slow_edge_lets_the_chain_resolve_at_the_fast_edges_newest_stamp() {
    let mut b = TfBuffer::new();
    b.set_edge_policy(
        "map",
        "odom",
        EdgePolicy::HoldLast {
            max_age_ns: 1_000_000_000,
        },
    )
    .unwrap();
    nav_chain(&mut b, 10);
    // At 19.99 s: map -> odom holds y = 19, and n = 1999. The stamp is
    // the fast edge's newest, 990 ms into the slow edge's hold.
    let latest = b.lookup_latest("map", "base").unwrap();
    assert_eq!(latest.stamp_ns, Some(19_990_000_000));
    assert_eq!(latest.transform.translation(), [1_999.0, 19.0, 0.0]);
    assert_eq!(
        latest.transform,
        b.lookup("map", "base", 19_990_000_000).unwrap()
    );
    // The hold reaches exactly 1 s past 19 s, and not one nanosecond more.
    assert_eq!(
        b.lookup("map", "odom", 20_000_000_000)
            .unwrap()
            .translation(),
        [0.0, 19.0, 0.0]
    );
    assert_eq!(
        b.lookup("map", "odom", 20_000_000_001),
        Err(TfError::NotYetAvailable {
            parent: id(&b, "map"),
            child: id(&b, "odom"),
            requested_ns: 20_000_000_001,
            newest_ns: 19_000_000_000,
        })
    );
    // A slow edge that stopped holds the chain back to what its hold
    // reaches: newest 14 s plus 1 s is 15 s, where n = 1500.
    let mut stale = TfBuffer::new();
    stale
        .set_edge_policy(
            "map",
            "odom",
            EdgePolicy::HoldLast {
                max_age_ns: 1_000_000_000,
            },
        )
        .unwrap();
    for k in 0..15u64 {
        let y = tf([0.0, k as f64, 0.0], UnitQuaternion::IDENTITY);
        stale.insert("map", "odom", k * 1_000_000_000, y).unwrap();
    }
    for n in 0..2_000u64 {
        stale
            .insert("odom", "base", n * 10_000_000, shift(n as f64))
            .unwrap();
    }
    let held = stale.lookup_latest("map", "base").unwrap();
    assert_eq!(held.stamp_ns, Some(15_000_000_000));
    assert_eq!(held.transform.translation(), [1_500.0, 14.0, 0.0]);
    // The fast edge keeps the default: nothing past its newest.
    assert!(matches!(
        b.lookup("map", "base", 19_990_000_001),
        Err(TfError::NotYetAvailable {
            newest_ns: 19_990_000_000,
            ..
        })
    ));
}

#[test]
fn lookup_latest_on_held_edges_only_resolves_at_their_newest_stamp() {
    let mut b = TfBuffer::new();
    let hold = EdgePolicy::HoldLast { max_age_ns: 1_000 };
    b.set_edge_policy("map", "odom", hold).unwrap();
    b.set_edge_policy("odom", "base", hold).unwrap();
    b.insert("map", "odom", 100, shift(1.0)).unwrap();
    b.insert("odom", "base", 0, shift(0.0)).unwrap();
    b.insert("odom", "base", 400, shift(4.0)).unwrap();
    // The newest stamp among them is 400; map -> odom holds 1 m there.
    assert_eq!(
        b.lookup_latest("map", "base").unwrap(),
        LatestTransform {
            transform: shift(5.0),
            stamp_ns: Some(400),
        }
    );
}

// ---------------------------------------------------------- extrapolation

#[test]
fn extrapolate_continues_the_last_motion_up_to_its_bound() {
    let mut b = TfBuffer::new();
    b.set_edge_policy("odom", "base", EdgePolicy::Extrapolate { max_ns: 5_000 })
        .unwrap();
    b.insert("odom", "base", 0, shift(0.0)).unwrap();
    b.insert("odom", "base", 1_000, shift(2.0)).unwrap();
    // 2 m per 1000 ns: half a step past the newest is 3 m, two steps 6 m,
    // two and a half 7 m.
    assert_eq!(b.lookup("odom", "base", 1_500).unwrap(), shift(3.0));
    assert_eq!(b.lookup("odom", "base", 3_000).unwrap(), shift(6.0));
    assert_eq!(b.lookup("odom", "base", 3_500).unwrap(), shift(7.0));
    // The bound: 5000 past the newest is answered, 5001 is not.
    assert_eq!(b.lookup("odom", "base", 6_000).unwrap(), shift(12.0));
    assert_eq!(
        b.lookup("odom", "base", 6_001),
        Err(TfError::NotYetAvailable {
            parent: id(&b, "odom"),
            child: id(&b, "base"),
            requested_ns: 6_001,
            newest_ns: 1_000,
        })
    );
    // Inside the ring it still interpolates.
    assert_eq!(b.lookup("odom", "base", 500).unwrap(), shift(1.0));
}

#[test]
fn extrapolate_continues_a_turn_in_the_frame_of_the_newest_sample() {
    let mut b = TfBuffer::new();
    b.set_edge_policy("odom", "base", EdgePolicy::Extrapolate { max_ns: 1_000 })
        .unwrap();
    // From the origin facing +x to (2, 0) facing +y over 1000 ns. One more
    // step repeats the motion from there, in the turned frame: 2 m along
    // +y to (2, 2), facing -x (180 about z).
    b.insert("odom", "base", 0, Transform::IDENTITY).unwrap();
    b.insert("odom", "base", 1_000, tf([2.0, 0.0, 0.0], z90()))
        .unwrap();
    let got = b.lookup("odom", "base", 2_000).unwrap();
    assert_close3(got.translation(), [2.0, 2.0, 0.0], 1e-15);
    assert_close4(got.rotation().to_array(), [0.0, 0.0, 1.0, 0.0], TOL);
}

#[test]
fn extrapolate_with_one_sample_has_no_motion_to_continue() {
    let mut b = TfBuffer::new();
    b.set_edge_policy("odom", "base", EdgePolicy::Extrapolate { max_ns: 1_000 })
        .unwrap();
    b.insert("odom", "base", 100, shift(1.0)).unwrap();
    assert_eq!(b.lookup("odom", "base", 100).unwrap(), shift(1.0));
    assert!(matches!(
        b.lookup("odom", "base", 101),
        Err(TfError::NotYetAvailable { newest_ns: 100, .. })
    ));
}

#[test]
fn an_extrapolation_that_overflows_is_chain_math_naming_the_lookup_frames() {
    let mut b = TfBuffer::new();
    b.set_edge_policy("odom", "base", EdgePolicy::Extrapolate { max_ns: u64::MAX })
        .unwrap();
    b.insert("odom", "base", 0, shift(0.0)).unwrap();
    b.insert("odom", "base", 1, shift(1e300)).unwrap();
    assert_eq!(
        b.lookup("odom", "base", 1_000_000_000),
        Err(TfError::ChainMath {
            parent: id(&b, "odom"),
            child: id(&b, "base"),
        })
    );
}

// --------------------------------------------------------- lookup by handle

#[test]
fn lookup_by_handle_is_lookup_by_name() {
    let mut b = turning_edge();
    b.insert_static("base", "lidar", shift(0.17)).unwrap();
    let (odom, lidar) = (id(&b, "odom"), id(&b, "lidar"));
    assert_eq!(
        b.lookup_ids(odom, lidar, 500),
        b.lookup("odom", "lidar", 500)
    );
    assert_eq!(
        b.lookup_latest_ids(lidar, odom),
        b.lookup_latest("lidar", "odom")
    );
    assert_eq!(
        b.lookup_ids(odom, lidar, 1_001),
        b.lookup("odom", "lidar", 1_001)
    );
}

#[test]
fn a_handle_from_another_buffer_is_refused() {
    let b = turning_edge();
    let mut other = turning_edge();
    other.insert_static("base", "lidar", shift(0.17)).unwrap();
    let foreign = other.frame_id("lidar").unwrap();
    assert_eq!(
        b.lookup_ids(id(&b, "odom"), foreign, 500),
        Err(TfError::UnknownFrameId { frame: foreign })
    );
    assert_eq!(
        b.lookup_latest_ids(foreign, id(&b, "odom")),
        Err(TfError::UnknownFrameId { frame: foreign })
    );
}

#[test]
fn a_handle_from_another_buffer_in_a_used_slot_is_refused() {
    // Slot 1 is `base` here and `lidar` there: the foreign handle must not
    // look up `base`.
    let b = turning_edge();
    let mut other = TfBuffer::new();
    other.insert_static("map", "lidar", shift(0.17)).unwrap();
    let (map, lidar) = (id(&other, "map"), id(&other, "lidar"));
    let odom = id(&b, "odom");
    assert_eq!(
        b.lookup_ids(odom, lidar, 500),
        Err(TfError::UnknownFrameId { frame: lidar })
    );
    assert_eq!(
        b.lookup_latest_ids(map, odom),
        Err(TfError::UnknownFrameId { frame: map })
    );
    // The issuing buffer still resolves its own handles.
    assert_eq!(other.lookup_ids(map, lidar, 0).unwrap(), shift(0.17));
}

#[test]
fn latest_counts_a_one_sample_extrapolating_edge_only_to_its_sample() {
    // map -> odom has one sample at 100 ns and may extrapolate 1000 ns,
    // but one sample gives no motion; odom -> base covers 0 to 200 ns.
    // Both answer at 100 ns: map -> odom 1 m, odom -> base halfway, 1 m.
    let mut b = TfBuffer::new();
    b.set_edge_policy("map", "odom", EdgePolicy::Extrapolate { max_ns: 1_000 })
        .unwrap();
    b.insert("map", "odom", 100, shift(1.0)).unwrap();
    b.insert("odom", "base", 0, shift(0.0)).unwrap();
    b.insert("odom", "base", 200, shift(2.0)).unwrap();
    assert_eq!(
        b.lookup_latest("map", "base").unwrap(),
        LatestTransform {
            transform: shift(2.0),
            stamp_ns: Some(100),
        }
    );
    // With a second sample the edge has motion and reaches past it: the
    // chain resolves at 200 ns, map -> odom one 50 ns step past 150 ns.
    b.insert("map", "odom", 150, shift(1.5)).unwrap();
    assert_eq!(
        b.lookup_latest("map", "base").unwrap(),
        LatestTransform {
            transform: shift(4.0),
            stamp_ns: Some(200),
        }
    );
}

#[test]
fn extrapolate_steps_by_the_span_between_the_two_newest_samples() {
    // Samples at 1000 and 3000 ns: the span is 2000, not their sum.
    let mut b = TfBuffer::new();
    b.set_edge_policy("odom", "base", EdgePolicy::Extrapolate { max_ns: 4_000 })
        .unwrap();
    b.insert("odom", "base", 1_000, shift(1.0)).unwrap();
    b.insert("odom", "base", 3_000, shift(3.0)).unwrap();
    // 2 m per 2000 ns: 1000 ns past the newest is 4 m, 4000 ns is 7 m.
    assert_eq!(b.lookup("odom", "base", 4_000).unwrap(), shift(4.0));
    assert_eq!(b.lookup("odom", "base", 7_000).unwrap(), shift(7.0));
}

#[test]
fn one_whole_extrapolation_step_composes_the_step_once_and_no_more() {
    // The step is 1.7e308 m: one step from 0 is 1.7e308, which is finite,
    // and two steps would overflow. A lookup one span past the newest
    // composes the step exactly once.
    let mut b = TfBuffer::new();
    b.set_edge_policy("odom", "base", EdgePolicy::Extrapolate { max_ns: 1_000 })
        .unwrap();
    b.insert("odom", "base", 0, shift(-1.7e308)).unwrap();
    b.insert("odom", "base", 1_000, shift(0.0)).unwrap();
    assert_eq!(b.lookup("odom", "base", 2_000).unwrap(), shift(1.7e308));
}
