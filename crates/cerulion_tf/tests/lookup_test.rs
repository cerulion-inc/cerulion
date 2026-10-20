// SPDX-License-Identifier: AGPL-3.0-only
//! `lookup(parent, child, time)` and `lookup_latest` through the public
//! surface: exact hits, interpolation, chains through a common ancestor,
//! the inverse direction, static edges, and every refusal.
//!
//! Every expected value is written by hand from the geometry (a 90 degree
//! turn about z sends +x to +y; a half-angle sine for an interpolated
//! rotation) or from an exact arithmetic identity, never read off the
//! buffer. Stamps are nanoseconds.

use cerulion_tf::{TfBuffer, TfError, TfMathError, Transform, UnitQuaternion};
use std::f64::consts::FRAC_1_SQRT_2;

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

fn name(s: &str) -> cerulion_tf::FrameName {
    s.parse().expect("a valid frame name")
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
    for (stamp, x) in [(0u64, 0.0), (100, 10.0), (200, 20.0), (300, 30.0)] {
        b.insert("odom", "base", stamp, shift(x)).unwrap();
    }
    // 150 lies between the 100 and 200 samples: 15, not a blend of 0 and 30
    // (which would also be 15) so the oracle must separate them: 125.
    assert_eq!(b.lookup("odom", "base", 125).unwrap(), shift(12.5));
    assert_eq!(b.lookup("odom", "base", 275).unwrap(), shift(27.5));
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
    assert_eq!(
        b.lookup_latest("base", "base").unwrap(),
        Transform::IDENTITY
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
    let mut b = TfBuffer::new();
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
    assert_eq!(latest, b.lookup("map", "base", 900).unwrap());
    // At 900 ns: map -> odom is 9 m; odom -> base is 9 m (0.9 of 10 m).
    assert_close3(latest.translation(), [18.0, 0.0, 0.0], 1e-14);
    // The inverse direction picks the same time.
    assert_eq!(
        b.lookup_latest("base", "map").unwrap(),
        b.lookup("base", "map", 900).unwrap()
    );
}

#[test]
fn lookup_latest_on_static_edges_only_needs_no_time() {
    let mut b = TfBuffer::new();
    b.insert_static("base", "lidar", shift(0.17)).unwrap();
    b.insert_static("base", "camera", shift(0.27)).unwrap();
    let got = b.lookup_latest("lidar", "camera").unwrap();
    assert_close3(got.translation(), [0.10, 0.0, 0.0], 1e-15);
}

#[test]
fn lookup_latest_uses_the_single_dynamic_edges_newest_sample() {
    let b = turning_edge();
    assert_eq!(
        b.lookup_latest("odom", "base").unwrap(),
        tf([2.0, 0.0, 0.0], z90())
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
            frame: name("base_link")
        })
    );
    assert_eq!(
        b.lookup("Odom", "base", 500),
        Err(TfError::UnknownFrame {
            frame: name("Odom")
        })
    );
}

// ------------------------------------------------------------ every error arm

#[test]
fn an_unknown_frame_is_refused_naming_the_first_unknown_one() {
    let b = turning_edge();
    assert_eq!(
        b.lookup("map", "base", 500),
        Err(TfError::UnknownFrame { frame: name("map") })
    );
    assert_eq!(
        b.lookup("odom", "lidar", 500),
        Err(TfError::UnknownFrame {
            frame: name("lidar")
        })
    );
    // Both unknown: the parent is named.
    assert_eq!(
        b.lookup_latest("map", "lidar"),
        Err(TfError::UnknownFrame { frame: name("map") })
    );
    // A frame id that could never be inserted gets the id's own refusal.
    assert_eq!(b.lookup("", "base", 500), Err(TfError::EmptyFrameId));
    let long = "y".repeat(cerulion_tf::MAX_FRAME_ID_LEN + 1);
    assert_eq!(
        b.lookup("odom", &long, 500),
        Err(TfError::FrameIdTooLong {
            prefix: name(&"y".repeat(cerulion_tf::MAX_FRAME_ID_LEN)),
            len: cerulion_tf::MAX_FRAME_ID_LEN + 1
        })
    );
}

#[test]
fn an_empty_buffer_knows_no_frame() {
    let b = TfBuffer::new();
    assert_eq!(
        b.lookup("odom", "base", 0),
        Err(TfError::UnknownFrame {
            frame: name("odom")
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
            parent: name("map"),
            child: name("lidar")
        })
    );
    assert_eq!(
        b.lookup_latest("odom", "base"),
        Err(TfError::NoPath {
            parent: name("odom"),
            child: name("base")
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
            parent: name("odom"),
            child: name("base"),
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
            parent: name("odom"),
            child: name("base"),
            requested_ns: 301,
            newest_ns: 300,
        })
    );
    // The inverse direction reports the edge as stored, parent first.
    assert_eq!(
        b.lookup("base", "odom", 301),
        Err(TfError::NotYetAvailable {
            parent: name("odom"),
            child: name("base"),
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
            parent: name("odom"),
            child: name("base"),
            requested_ns: 499,
            oldest_ns: 500,
            newest_ns: 500,
        })
    );
    assert_eq!(
        b.lookup("odom", "base", 501),
        Err(TfError::NotYetAvailable {
            parent: name("odom"),
            child: name("base"),
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
            parent: name("odom"),
            child: name("base"),
            requested_ns: 700,
            newest_ns: 600,
        })
    );
    assert_eq!(
        b.lookup("map", "base", 400),
        Err(TfError::OutOfRange {
            parent: name("odom"),
            child: name("base"),
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
            parent: name("map"),
            child: name("odom"),
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
            parent: name("a"),
            child: name("c"),
            source: TfMathError::NonFiniteTranslation,
        })
    );
    // The frames are the lookup's own, in the order it named them.
    assert_eq!(
        b.lookup("c", "a", 0),
        Err(TfError::ChainMath {
            parent: name("c"),
            child: name("a"),
            source: TfMathError::NonFiniteTranslation,
        })
    );
    // Siblings through `a`: the chain `a -> c` overflows on its own.
    assert_eq!(
        b.lookup("d", "c", 0),
        Err(TfError::ChainMath {
            parent: name("d"),
            child: name("c"),
            source: TfMathError::NonFiniteTranslation,
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
        b.evictions(),
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
            b.evictions(),
            b.edge("base").map(|e| e.samples)
        )
    );
}
