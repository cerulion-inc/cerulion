// SPDX-License-Identifier: AGPL-3.0-only
//! Interpolation through its public surface: SLERP on the shortest arc for
//! rotation, linear for translation, exact endpoints, and refusal of any
//! fraction outside `[0, 1]`.
//!
//! The expected quaternions are half-angle sines and cosines of round angles
//! written out by hand (a turn of `a` about +z is `(0, 0, sin a/2, cos a/2)`),
//! never computed by the code under test.

use cerulion_tf::{TfMathError, Transform, UnitQuaternion};
use std::f64::consts::FRAC_1_SQRT_2;

const TOL: f64 = 1e-15;

/// sin and cos of 22.5 degrees (a 45 degree turn).
const SIN_22_5: f64 = 0.382_683_432_365_089_8;
const COS_22_5: f64 = 0.923_879_532_511_286_7;
/// sin and cos of 15 degrees (a 30 degree turn).
const SIN_15: f64 = 0.258_819_045_102_520_76;
const COS_15: f64 = 0.965_925_826_289_068_3;
/// cos of 30 degrees (a 60 degree turn; sin 30 is 0.5).
const COS_30: f64 = 0.866_025_403_784_438_6;

fn q(x: f64, y: f64, z: f64, w: f64) -> UnitQuaternion {
    UnitQuaternion::new(x, y, z, w).expect("a valid hand-written quaternion")
}

fn tf(t: [f64; 3], r: UnitQuaternion) -> Transform {
    Transform::new(t, r).expect("a valid hand-written transform")
}

fn rot_z90() -> UnitQuaternion {
    q(0.0, 0.0, FRAC_1_SQRT_2, FRAC_1_SQRT_2)
}

fn assert_close4(got: [f64; 4], want: [f64; 4], tol: f64) {
    for i in 0..4 {
        assert!(
            (got[i] - want[i]).abs() <= tol,
            "component {i}: got {got:?}, want {want:?} (tol {tol:e})"
        );
    }
}

fn slerp(a: UnitQuaternion, b: UnitQuaternion, t: f64) -> [f64; 4] {
    a.slerp(&b, t).expect("a valid fraction").to_array()
}

// ------------------------------------------------------------------- happy

#[test]
fn halfway_from_identity_to_90_about_z_is_45_about_z() {
    let got = slerp(UnitQuaternion::IDENTITY, rot_z90(), 0.5);
    assert_close4(got, [0.0, 0.0, SIN_22_5, COS_22_5], TOL);
}

#[test]
fn a_third_of_90_about_z_is_30_and_two_thirds_is_60() {
    let got = slerp(UnitQuaternion::IDENTITY, rot_z90(), 1.0 / 3.0);
    assert_close4(got, [0.0, 0.0, SIN_15, COS_15], TOL);
    let got = slerp(UnitQuaternion::IDENTITY, rot_z90(), 2.0 / 3.0);
    assert_close4(got, [0.0, 0.0, 0.5, COS_30], TOL);
}

#[test]
fn slerp_between_two_non_identity_rotations() {
    // From 30 about z to 90 about z, halfway: 60 about z.
    let a = q(0.0, 0.0, SIN_15, COS_15);
    let got = slerp(a, rot_z90(), 0.5);
    assert_close4(got, [0.0, 0.0, 0.5, COS_30], TOL);
}

/// A turn of `deg` degrees about the diagonal axis (1, 1, 1) / sqrt 3,
/// from hand-written half-angle values: each of x, y, z is
/// sin(deg / 2) / sqrt 3.
fn about_diagonal(deg: u32) -> UnitQuaternion {
    let (axis_part, w) = match deg {
        // sin 30 / sqrt 3, cos 30.
        60 => (0.288_675_134_594_812_9, COS_30),
        // sin 45 / sqrt 3, cos 45.
        90 => (0.408_248_290_463_863, FRAC_1_SQRT_2),
        // sin 60 / sqrt 3 = 1/2, cos 60 = 1/2.
        120 => (0.5, 0.5),
        // sin 90 / sqrt 3, cos 90.
        180 => (0.577_350_269_189_625_8, 0.0),
        _ => unreachable!("no hand value for {deg}"),
    };
    q(axis_part, axis_part, axis_part, w)
}

#[test]
fn slerp_about_a_skew_axis_uses_every_component() {
    // 60 to 180 about the diagonal: halfway is 120, a quarter is 90.
    let (a, b) = (about_diagonal(60), about_diagonal(180));
    assert_close4(slerp(a, b, 0.5), about_diagonal(120).to_array(), TOL);
    assert_close4(slerp(a, b, 0.25), about_diagonal(90).to_array(), TOL);
    // 60 to 120: every component of both ends is nonzero.
    let b = about_diagonal(120);
    assert_close4(slerp(a, b, 0.5), about_diagonal(90).to_array(), TOL);
}

#[test]
fn slerp_about_a_skew_axis_takes_the_short_arc_from_a_negated_target() {
    let a = about_diagonal(60);
    let [x, y, z, w] = about_diagonal(180).to_array();
    let b = q(-x, -y, -z, -w);
    assert_close4(slerp(a, b, 0.5), about_diagonal(120).to_array(), TOL);
    let [x, y, z, w] = about_diagonal(120).to_array();
    let b = q(-x, -y, -z, -w);
    assert_close4(slerp(a, b, 0.5), about_diagonal(90).to_array(), TOL);
}

#[test]
fn halfway_between_90_about_x_and_90_about_y_through_a_negated_target() {
    // a and b are 60 degrees apart in 4-D, so the midpoint is
    // (a + b) / (2 cos 30): (1/sqrt 6, 1/sqrt 6, 0, sqrt(2/3)).
    let inv_sqrt6 = 0.408_248_290_463_863;
    let sqrt_two_thirds = 0.816_496_580_927_726;
    let x90 = q(FRAC_1_SQRT_2, 0.0, 0.0, FRAC_1_SQRT_2);
    let y90 = q(0.0, FRAC_1_SQRT_2, 0.0, FRAC_1_SQRT_2);
    let neg_x90 = q(-FRAC_1_SQRT_2, 0.0, 0.0, -FRAC_1_SQRT_2);
    let neg_y90 = q(0.0, -FRAC_1_SQRT_2, 0.0, -FRAC_1_SQRT_2);
    let want = [inv_sqrt6, inv_sqrt6, 0.0, sqrt_two_thirds];
    assert_close4(slerp(x90, y90, 0.5), want, TOL);
    assert_close4(slerp(x90, neg_y90, 0.5), want, TOL);
    assert_close4(slerp(y90, neg_x90, 0.5), want, TOL);
}

#[test]
fn slerp_result_is_unit_length() {
    let a = q(0.1, 0.2, 0.3, 0.9);
    let b = q(-0.4, 0.1, 0.0, 0.8);
    let [x, y, z, w] = slerp(a, b, 0.37);
    let norm = (x * x + y * y + z * z + w * w).sqrt();
    assert!((norm - 1.0).abs() <= TOL, "norm {norm}");
}

#[test]
fn translation_interpolates_linearly() {
    let a = tf([0.0, 0.0, 0.0], UnitQuaternion::IDENTITY);
    let b = tf([2.0, -4.0, 8.0], UnitQuaternion::IDENTITY);
    let got = a.interpolate(&b, 0.25).unwrap();
    assert_eq!(got.translation(), [0.5, -1.0, 2.0]);
}

#[test]
fn transform_interpolation_blends_both_parts() {
    let a = tf([1.0, 1.0, 1.0], UnitQuaternion::IDENTITY);
    let b = tf([3.0, 1.0, -1.0], rot_z90());
    let got = a.interpolate(&b, 0.5).unwrap();
    assert_eq!(got.translation(), [2.0, 1.0, 0.0]);
    assert_close4(
        got.rotation().to_array(),
        [0.0, 0.0, SIN_22_5, COS_22_5],
        TOL,
    );
}

// -------------------------------------------------------------------- edge

#[test]
fn endpoints_return_the_inputs_bit_for_bit() {
    let a = tf([0.1, -0.2, 0.3], q(0.1, 0.2, 0.3, 0.9));
    // b is given with a negative w on purpose: t = 1 returns it as given,
    // with no sign flip, even though the interior takes the short arc.
    let b = tf(
        [5.0, 6.0, -7.0],
        q(0.0, 0.0, -FRAC_1_SQRT_2, -FRAC_1_SQRT_2),
    );
    assert_eq!(a.interpolate(&b, 0.0).unwrap(), a);
    assert_eq!(a.interpolate(&b, 1.0).unwrap(), b);
    assert_eq!(
        a.rotation().slerp(&b.rotation(), 1.0).unwrap(),
        b.rotation()
    );
    // Negative zero is zero.
    assert_eq!(a.interpolate(&b, -0.0).unwrap(), a);
}

#[test]
fn negated_target_takes_the_short_arc() {
    // -(90 about z) is the same rotation as 90 about z. Without the sign
    // flip the halfway point would be a 135 degree turn the long way round.
    let b = q(0.0, 0.0, -FRAC_1_SQRT_2, -FRAC_1_SQRT_2);
    let got = slerp(UnitQuaternion::IDENTITY, b, 0.5);
    assert_close4(got, [0.0, 0.0, SIN_22_5, COS_22_5], TOL);
    let got = slerp(UnitQuaternion::IDENTITY, b, 1.0 / 3.0);
    assert_close4(got, [0.0, 0.0, SIN_15, COS_15], TOL);
}

#[test]
fn negated_source_takes_the_short_arc() {
    let a = q(0.0, 0.0, 0.0, -1.0);
    let got = slerp(a, rot_z90(), 0.5);
    // The blend is built from a and the flipped b, so it comes out as
    // -(45 about z); the rotation is 45 about z either way.
    assert_close4(got, [0.0, 0.0, -SIN_22_5, -COS_22_5], TOL);
}

#[test]
fn rotation_of_180_degrees_interpolates_through_90() {
    // Identity to 180 about z: dot is exactly zero, both arcs are equally
    // short, and the target is kept as given.
    let b = q(0.0, 0.0, 1.0, 0.0);
    assert_close4(
        slerp(UnitQuaternion::IDENTITY, b, 0.5),
        [0.0, 0.0, FRAC_1_SQRT_2, FRAC_1_SQRT_2],
        TOL,
    );
    assert_close4(
        slerp(UnitQuaternion::IDENTITY, b, 0.25),
        [0.0, 0.0, SIN_22_5, COS_22_5],
        TOL,
    );
    // The other sign of the same 180 degree turn goes the other way round.
    let b_neg = q(0.0, 0.0, -1.0, 0.0);
    assert_close4(
        slerp(UnitQuaternion::IDENTITY, b_neg, 0.5),
        [0.0, 0.0, -FRAC_1_SQRT_2, FRAC_1_SQRT_2],
        TOL,
    );
}

#[test]
fn equal_quaternions_interpolate_to_themselves() {
    // A zero angle: the SLERP weights would divide zero by zero, so the
    // normalized-lerp fallback must take this case.
    let a = q(0.5, 0.5, 0.5, 0.5);
    for t in [0.1, 0.5, 0.9] {
        assert_close4(slerp(a, a, t), [0.5, 0.5, 0.5, 0.5], TOL);
    }
}

#[test]
fn near_parallel_quaternions_fall_back_to_normalized_lerp() {
    // A 1e-9 rad turn about x: (sin 5e-10, 0, 0, cos 5e-10). cos 5e-10 is
    // 1 - 1.25e-19, which rounds to 1.0. Its 4-D angle to the identity is
    // 5e-10, below the 1e-6 threshold.
    assert_eq!(UnitQuaternion::SLERP_LERP_THRESHOLD, 1e-6);
    let b = q(5e-10, 0.0, 0.0, 1.0);
    let [x, y, z, w] = slerp(UnitQuaternion::IDENTITY, b, 0.5);
    assert!((x - 2.5e-10).abs() <= 1e-25, "x {x:e}");
    assert_eq!([y, z, w], [0.0, 0.0, 1.0]);
}

#[test]
fn just_above_the_threshold_still_slerps_accurately() {
    // A 4e-6 rad turn about z: 4-D angle 2e-6, above the threshold.
    // Halfway is a 2e-6 rad turn: (0, 0, sin 1e-6, cos 1e-6).
    // sin 1e-6 = 1e-6 - 1.6667e-19; cos 1e-6 = 1 - 5e-13.
    let b = q(0.0, 0.0, 1.999_999_999_998_666_7e-6, 0.999_999_999_998);
    let [x, y, z, w] = slerp(UnitQuaternion::IDENTITY, b, 0.5);
    assert_eq!([x, y], [0.0, 0.0]);
    assert!((z - 9.999_999_999_998_333e-7).abs() <= 1e-21, "z {z:e}");
    assert!((w - 0.999_999_999_999_5).abs() <= TOL, "w {w}");
}

#[test]
fn a_non_normalized_input_interpolates_as_its_unit_form() {
    // (0, 0, 7, 7) normalizes to 90 about z.
    let b = q(0.0, 0.0, 7.0, 7.0);
    assert_close4(
        slerp(UnitQuaternion::IDENTITY, b, 0.5),
        [0.0, 0.0, SIN_22_5, COS_22_5],
        TOL,
    );
}

// ------------------------------------------------------- adversarial, errors

#[test]
fn a_nan_fraction_is_refused() {
    let a = Transform::IDENTITY;
    assert_eq!(
        a.interpolate(&a, f64::NAN),
        Err(TfMathError::FractionNotANumber)
    );
    assert_eq!(
        UnitQuaternion::IDENTITY.slerp(&UnitQuaternion::IDENTITY, f64::NAN),
        Err(TfMathError::FractionNotANumber)
    );
}

#[test]
fn a_fraction_below_zero_is_refused_never_extrapolated() {
    let a = Transform::IDENTITY;
    for t in [-1e-300, -0.5, -1.0, f64::NEG_INFINITY] {
        assert_eq!(
            a.interpolate(&a, t),
            Err(TfMathError::FractionOutOfRange { t }),
            "t = {t}"
        );
        assert_eq!(
            UnitQuaternion::IDENTITY.slerp(&UnitQuaternion::IDENTITY, t),
            Err(TfMathError::FractionOutOfRange { t }),
            "t = {t}"
        );
    }
}

#[test]
fn a_fraction_above_one_is_refused_never_extrapolated() {
    let a = Transform::IDENTITY;
    // 1.0000000000000002 is the next f64 above 1.
    for t in [1.000_000_000_000_000_2, 1.5, 2.0, f64::INFINITY] {
        assert_eq!(
            a.interpolate(&a, t),
            Err(TfMathError::FractionOutOfRange { t }),
            "t = {t}"
        );
        assert_eq!(
            UnitQuaternion::IDENTITY.slerp(&UnitQuaternion::IDENTITY, t),
            Err(TfMathError::FractionOutOfRange { t }),
            "t = {t}"
        );
    }
}

#[test]
fn the_largest_fraction_below_one_is_accepted() {
    // 0.9999999999999999 is the next f64 below 1: an interior point.
    let t = 0.999_999_999_999_999_9;
    let got = slerp(UnitQuaternion::IDENTITY, rot_z90(), t);
    assert_close4(got, rot_z90().to_array(), TOL);
}

#[test]
fn translation_near_the_f64_limit_interpolates_without_overflow() {
    // (1 - t) a + t b stays between a and b, so endpoints far apart near
    // the limit still blend: a difference b - a would overflow here.
    let a = tf([-1.5e308, 0.0, 0.0], UnitQuaternion::IDENTITY);
    let b = tf([1.5e308, 0.0, 0.0], UnitQuaternion::IDENTITY);
    assert_eq!(
        a.interpolate(&b, 0.5).unwrap().translation(),
        [0.0, 0.0, 0.0]
    );
}
