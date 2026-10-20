// SPDX-License-Identifier: AGPL-3.0-only
//! The SE(3) math through its public surface: construction rules,
//! compose, invert and apply.
//!
//! Every expected value below is written by hand from the geometry (a
//! 90 degree turn about z sends +x to +y, and so on) or from an exact
//! arithmetic identity, never computed by the code under test.

use cerulion_tf::{TfMathError, Transform, UnitQuaternion};
use std::f64::consts::FRAC_1_SQRT_2;

/// Absolute tolerance for unit-magnitude results: a few ulps of 1.0.
const TOL: f64 = 1e-15;

fn q(x: f64, y: f64, z: f64, w: f64) -> UnitQuaternion {
    UnitQuaternion::new(x, y, z, w).expect("a valid hand-written quaternion")
}

fn tf(t: [f64; 3], r: UnitQuaternion) -> Transform {
    Transform::new(t, r).expect("a valid hand-written transform")
}

/// 90 degrees about +z: (0, 0, sin 45, cos 45).
fn rot_z90() -> UnitQuaternion {
    q(0.0, 0.0, FRAC_1_SQRT_2, FRAC_1_SQRT_2)
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

// ---------------------------------------------------------------- quaternion

#[test]
fn identity_quaternion_is_x_y_z_w_zero_zero_zero_one() {
    assert_eq!(UnitQuaternion::IDENTITY.to_array(), [0.0, 0.0, 0.0, 1.0]);
    assert_eq!(UnitQuaternion::IDENTITY.w(), 1.0);
}

#[test]
fn accessors_return_components_in_ros_order() {
    let r = q(0.5, -0.5, 0.5, 0.5);
    assert_eq!((r.x(), r.y(), r.z(), r.w()), (0.5, -0.5, 0.5, 0.5));
}

#[test]
fn construction_normalizes_a_scaled_quaternion() {
    // (0, 0, 0, 2) has norm 2; the unit form is the identity, exactly.
    assert_eq!(q(0.0, 0.0, 0.0, 2.0).to_array(), [0.0, 0.0, 0.0, 1.0]);
    // (1, 1, 1, 1) has norm 2; every component halves, exactly.
    assert_eq!(q(1.0, 1.0, 1.0, 1.0).to_array(), [0.5, 0.5, 0.5, 0.5]);
    // (0, 0, 3, 4) has norm 5: the 3-4-5 triangle.
    assert_eq!(q(0.0, 0.0, 3.0, 4.0).to_array(), [0.0, 0.0, 0.6, 0.8]);
}

#[test]
fn construction_keeps_the_sign_it_was_given() {
    // q and -q name the same rotation; construction never flips the sign.
    assert_eq!(q(0.0, 0.0, 0.0, -1.0).to_array(), [0.0, 0.0, 0.0, -1.0]);
    assert_eq!(q(0.0, 0.0, -3.0, -4.0).to_array(), [0.0, 0.0, -0.6, -0.8]);
}

#[test]
fn construction_survives_components_whose_squares_overflow() {
    // 1e300 squared overflows f64; the unit form is still the identity.
    // Powers of two keep the 3-4-5 ratio exact.
    let big = 2f64.powi(1000);
    assert_eq!(q(0.0, 0.0, 0.0, 1e300).to_array(), [0.0, 0.0, 0.0, 1.0]);
    assert_eq!(
        q(0.0, 0.0, 3.0 * big, 4.0 * big).to_array(),
        [0.0, 0.0, 0.6, 0.8]
    );
}

#[test]
fn construction_accepts_a_small_but_not_degenerate_norm() {
    // Norm 5 * 2^-30 (about 4.7e-9) is above the 1e-12 floor and keeps its
    // direction. Powers of two keep the 3-4-5 ratio exact.
    let small = 2f64.powi(-30);
    assert_eq!(
        q(0.0, 0.0, 3.0 * small, 4.0 * small).to_array(),
        [0.0, 0.0, 0.6, 0.8]
    );
}

#[test]
fn construction_rejects_the_zero_quaternion() {
    assert_eq!(
        UnitQuaternion::new(0.0, 0.0, 0.0, 0.0),
        Err(TfMathError::DegenerateQuaternion { norm: 0.0 })
    );
}

#[test]
fn construction_rejects_a_norm_below_the_floor() {
    // (0, 0, 3e-13, 4e-13) has norm 5e-13, below UnitQuaternion::MIN_NORM (1e-12).
    match UnitQuaternion::new(0.0, 0.0, 3e-13, 4e-13) {
        Err(TfMathError::DegenerateQuaternion { norm }) => {
            assert!((norm - 5e-13).abs() <= 1e-27, "norm {norm:e}");
        }
        other => panic!("expected DegenerateQuaternion, got {other:?}"),
    }
    assert_eq!(UnitQuaternion::MIN_NORM, 1e-12);
}

#[test]
fn degenerate_quaternion_message_names_the_min_norm() {
    // The message hard-codes the floor; this pins it to MIN_NORM so a change
    // to either one fails here.
    assert_eq!(UnitQuaternion::MIN_NORM, 1e-12);
    assert_eq!(
        TfMathError::DegenerateQuaternion { norm: 0.0 }.to_string(),
        "quaternion norm 0e0 is below the minimum of 1e-12, so it names no rotation"
    );
}

#[test]
fn construction_accepts_a_norm_exactly_at_the_floor() {
    // The floor itself is accepted: only a norm strictly below it is refused.
    let at_floor = UnitQuaternion::new(0.0, 0.0, 0.0, 1e-12).unwrap();
    assert_eq!(at_floor.to_array(), [0.0, 0.0, 0.0, 1.0]);
}

#[test]
fn construction_rejects_nan_in_every_component() {
    for i in 0..4 {
        let mut c = [0.0, 0.0, 0.0, 1.0];
        c[i] = f64::NAN;
        assert_eq!(
            UnitQuaternion::new(c[0], c[1], c[2], c[3]),
            Err(TfMathError::NonFiniteQuaternion),
            "NaN in component {i}"
        );
    }
}

#[test]
fn construction_rejects_infinity_in_every_component() {
    for i in 0..4 {
        for inf in [f64::INFINITY, f64::NEG_INFINITY] {
            let mut c = [0.0, 0.0, 0.0, 1.0];
            c[i] = inf;
            assert_eq!(
                UnitQuaternion::new(c[0], c[1], c[2], c[3]),
                Err(TfMathError::NonFiniteQuaternion),
                "{inf} in component {i}"
            );
        }
    }
}

#[test]
fn from_array_reads_x_y_z_w() {
    let r = UnitQuaternion::from_array([0.0, 0.0, 3.0, 4.0]).unwrap();
    assert_eq!(r.to_array(), [0.0, 0.0, 0.6, 0.8]);
}

#[test]
fn inverse_quaternion_is_the_conjugate() {
    let r = q(0.5, -0.5, 0.5, 0.5).inverse();
    assert_eq!(r.to_array(), [-0.5, 0.5, -0.5, 0.5]);
}

// ------------------------------------------------------------------- rotate

#[test]
fn rotating_x_by_90_about_z_gives_y() {
    let v = rot_z90().rotate([1.0, 0.0, 0.0]);
    assert_close3(v, [0.0, 1.0, 0.0], TOL);
}

#[test]
fn rotating_y_by_90_about_z_gives_minus_x() {
    let v = rot_z90().rotate([0.0, 1.0, 0.0]);
    assert_close3(v, [-1.0, 0.0, 0.0], TOL);
}

#[test]
fn rotating_by_180_about_x_flips_y_and_z_exactly() {
    // (1, 0, 0, 0): 180 degrees about +x. Every product is exact here.
    let r = q(1.0, 0.0, 0.0, 0.0);
    assert_eq!(r.rotate([1.0, 2.0, 3.0]), [1.0, -2.0, -3.0]);
}

#[test]
fn rotating_by_120_about_the_diagonal_cycles_the_axes() {
    // (0.5, 0.5, 0.5, 0.5): 120 degrees about (1, 1, 1) maps x->y, y->z, z->x.
    let r = q(0.5, 0.5, 0.5, 0.5);
    assert_eq!(r.rotate([1.0, 0.0, 0.0]), [0.0, 1.0, 0.0]);
    assert_eq!(r.rotate([0.0, 1.0, 0.0]), [0.0, 0.0, 1.0]);
    assert_eq!(r.rotate([0.0, 0.0, 1.0]), [1.0, 0.0, 0.0]);
}

#[test]
fn quaternion_compose_applies_the_right_factor_first() {
    // a = 90 about z, b = 90 about x. (a * b) v = a (b v).
    // b sends +y to +z; a leaves +z alone, so (a * b) y = z.
    // b sends +z to -y; a sends -y to +x, so (a * b) z = x.
    let a = rot_z90();
    let b = q(FRAC_1_SQRT_2, 0.0, 0.0, FRAC_1_SQRT_2);
    let ab = a.compose(&b);
    assert_close3(ab.rotate([0.0, 1.0, 0.0]), [0.0, 0.0, 1.0], TOL);
    assert_close3(ab.rotate([0.0, 0.0, 1.0]), [1.0, 0.0, 0.0], TOL);
    // The Hamilton product of the two, by hand: (0.5, 0.5, 0.5, 0.5).
    assert_close4(ab.to_array(), [0.5, 0.5, 0.5, 0.5], TOL);
}

#[test]
fn quaternion_compose_renormalizes() {
    // Twelve 30 degree steps; the product stays unit length.
    let step = q(0.0, 0.0, 0.258_819_045_102_520_74, 0.965_925_826_289_068_3);
    let mut acc = UnitQuaternion::IDENTITY;
    for _ in 0..12 {
        acc = acc.compose(&step);
    }
    let [x, y, z, w] = acc.to_array();
    let norm = (x * x + y * y + z * z + w * w).sqrt();
    assert!((norm - 1.0).abs() <= TOL, "norm {norm}");
    // 360 degrees is -identity as a quaternion.
    assert_close4(acc.to_array(), [0.0, 0.0, 0.0, -1.0], 1e-14);
}

// ---------------------------------------------------------------- transform

#[test]
fn transform_identity_is_zero_translation_and_identity_rotation() {
    let i = Transform::IDENTITY;
    assert_eq!(i.translation(), [0.0, 0.0, 0.0]);
    assert_eq!(i.rotation(), UnitQuaternion::IDENTITY);
}

#[test]
fn transform_rejects_a_non_finite_translation_in_every_component() {
    for i in 0..3 {
        for bad in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            let mut t = [1.0, 2.0, 3.0];
            t[i] = bad;
            assert_eq!(
                Transform::new(t, UnitQuaternion::IDENTITY),
                Err(TfMathError::NonFiniteTranslation),
                "{bad} in component {i}"
            );
        }
    }
}

#[test]
fn apply_point_rotates_then_translates() {
    // Child frame turned 90 about z and moved to (1, 2, 3) in the parent.
    // The child's +x unit point sits at (1, 3, 3) in the parent.
    let t = tf([1.0, 2.0, 3.0], rot_z90());
    assert_close3(t.apply_point([1.0, 0.0, 0.0]), [1.0, 3.0, 3.0], TOL);
}

#[test]
fn apply_vector_rotates_and_ignores_translation() {
    let t = tf([1.0, 2.0, 3.0], rot_z90());
    assert_close3(t.apply_vector([1.0, 0.0, 0.0]), [0.0, 1.0, 0.0], TOL);
}

#[test]
fn identity_applies_as_the_identity_exactly() {
    let p = [1.25, -2.5, 1e-300];
    assert_eq!(Transform::IDENTITY.apply_point(p), p);
    assert_eq!(Transform::IDENTITY.apply_vector(p), p);
}

#[test]
fn compose_chains_parent_to_middle_to_child() {
    // T_a_b: b sits at (1, 0, 0) in a, turned 90 about z.
    // T_b_c: c sits at (1, 0, 0) in b, not turned.
    // c's origin is b's +x unit point, which is a's (1, 1, 0).
    let t_a_b = tf([1.0, 0.0, 0.0], rot_z90());
    let t_b_c = tf([1.0, 0.0, 0.0], UnitQuaternion::IDENTITY);
    let t_a_c = t_a_b.compose(&t_b_c).unwrap();
    assert_close3(t_a_c.translation(), [1.0, 1.0, 0.0], TOL);
    assert_close4(
        t_a_c.rotation().to_array(),
        [0.0, 0.0, FRAC_1_SQRT_2, FRAC_1_SQRT_2],
        TOL,
    );
}

#[test]
fn compose_is_not_commutative() {
    // The other order: rotation of the identity leaves (1, 0, 0) alone.
    let t_a_b = tf([1.0, 0.0, 0.0], rot_z90());
    let t_b_c = tf([1.0, 0.0, 0.0], UnitQuaternion::IDENTITY);
    let swapped = t_b_c.compose(&t_a_b).unwrap();
    assert_close3(swapped.translation(), [2.0, 0.0, 0.0], TOL);
}

#[test]
fn compose_then_apply_equals_apply_twice() {
    let t_a_b = tf([0.5, -1.0, 2.0], q(0.1, 0.2, 0.3, 0.9));
    let t_b_c = tf([-3.0, 0.25, 1.0], q(-0.4, 0.1, 0.0, 0.8));
    let p = [0.7, -0.2, 1.9];
    let once = t_a_b.compose(&t_b_c).unwrap().apply_point(p);
    let twice = t_a_b.apply_point(t_b_c.apply_point(p));
    assert_close3(once, twice, 1e-14);
}

#[test]
fn identity_is_a_two_sided_unit_for_compose() {
    let t = tf([1.5, -2.25, 3.0], q(0.1, 0.2, 0.3, 0.9));
    for c in [
        Transform::IDENTITY.compose(&t).unwrap(),
        t.compose(&Transform::IDENTITY).unwrap(),
    ] {
        assert_close3(c.translation(), t.translation(), TOL * 4.0);
        assert_close4(c.rotation().to_array(), t.rotation().to_array(), TOL);
    }
}

#[test]
fn inverse_of_a_hand_transform() {
    // b sits at (1, 0, 0) in a, turned 90 about z. a's origin is one unit
    // along a's -x from b, and a's -x is b's +y: a's origin is b's (0, 1, 0).
    let t = tf([1.0, 0.0, 0.0], rot_z90());
    let inv = t.inverse().unwrap();
    assert_close3(inv.translation(), [0.0, 1.0, 0.0], TOL);
    assert_close4(
        inv.rotation().to_array(),
        [0.0, 0.0, -FRAC_1_SQRT_2, FRAC_1_SQRT_2],
        TOL,
    );
}

#[test]
fn compose_with_inverse_is_identity_within_tolerance() {
    let t = tf([1.5, -2.25, 3.0], q(0.1, 0.2, 0.3, 0.9));
    let inv = t.inverse().unwrap();
    for c in [t.compose(&inv).unwrap(), inv.compose(&t).unwrap()] {
        // Translation magnitude ~4: allow a few ulps of 4.
        assert_close3(c.translation(), [0.0, 0.0, 0.0], 4e-15);
        let [x, y, z, w] = c.rotation().to_array();
        assert_close4([x, y, z, w.abs()], [0.0, 0.0, 0.0, 1.0], TOL);
    }
}

#[test]
fn rotation_by_180_composed_twice_is_the_identity_motion() {
    // 180 about x, twice: the quaternion is -identity, the motion is identity.
    let half = tf([0.0, 0.0, 0.0], q(1.0, 0.0, 0.0, 0.0));
    let full = half.compose(&half).unwrap();
    assert_eq!(full.rotation().to_array(), [0.0, 0.0, 0.0, -1.0]);
    assert_eq!(full.apply_point([1.0, 2.0, 3.0]), [1.0, 2.0, 3.0]);
}

#[test]
fn compose_rejects_a_translation_that_overflows() {
    let far = tf([1e308, 0.0, 0.0], UnitQuaternion::IDENTITY);
    assert_eq!(far.compose(&far), Err(TfMathError::NonFiniteTranslation));
}

#[test]
fn inverse_rejects_a_translation_that_overflows() {
    // Turned 45 about z, both x and y near f64::MAX: the rotated x is
    // (x + y) / sqrt 2, about 1.41 * 1.7e308, past f64::MAX.
    let s = 0.382_683_432_365_089_8; // sin 22.5
    let c = 0.923_879_532_511_286_7; // cos 22.5
    let t = tf([1.7e308, 1.7e308, 0.0], q(0.0, 0.0, s, c));
    assert_eq!(t.inverse(), Err(TfMathError::NonFiniteTranslation));
}

#[test]
fn rotating_a_vector_near_the_f64_limit_does_not_overflow() {
    // 180 about z maps (1e308, 0, 0) to (-1e308, 0, 0). Unscaled, the term
    // t = 2 (u x v) would be 2e308, past f64::MAX, and inf * 0 a NaN.
    let half_turn_z = q(0.0, 0.0, 1.0, 0.0);
    assert_eq!(half_turn_z.rotate([1e308, 0.0, 0.0]), [-1e308, 0.0, 0.0]);
    // 90 about z maps (0, f64::MAX, 0) to (-f64::MAX, 0, 0).
    let got = rot_z90().rotate([0.0, f64::MAX, 0.0]);
    // 1e293 is about 5 ulps of f64::MAX.
    assert_close3(got, [-f64::MAX, 0.0, 0.0], 1e293);
    assert!(got.iter().all(|c| c.is_finite()), "got {got:?}");
}

#[test]
fn rotating_three_large_components_keeps_each_one() {
    // 120 about the diagonal cycles the axes, (x, y, z) to (z, x, y), so
    // every component of the input and of the output is near the limit.
    let cycle = q(0.5, 0.5, 0.5, 0.5);
    let got = cycle.rotate([1e308, -5e307, 2e307]);
    // 1e293 is a few ulps at this magnitude.
    assert_close3(got, [2e307, 1e308, -5e307], 1e293);
}

#[test]
fn transform_inverse_and_compose_accept_a_translation_near_the_f64_limit() {
    let far = tf([1e308, 0.0, 0.0], q(0.0, 0.0, 1.0, 0.0));
    // The inverse of (t, R) is (-(R^-1 t), R^-1): -(-1e308) = 1e308.
    let inv = far.inverse().unwrap();
    assert_eq!(inv.translation(), [1e308, 0.0, 0.0]);
    assert_eq!(inv.rotation().to_array(), [0.0, 0.0, -1.0, 0.0]);
    // R t + t = -1e308 + 1e308 = 0.
    let back = far.compose(&tf([1e308, 0.0, 0.0], UnitQuaternion::IDENTITY));
    assert_eq!(back.unwrap().translation(), [0.0, 0.0, 0.0]);
    assert_eq!(far.apply_vector([1e308, 0.0, 0.0]), [-1e308, 0.0, 0.0]);
}

#[test]
fn rotation_at_the_scaling_threshold_is_exact_for_the_identity() {
    // At the threshold itself no scaling applies, so the identity returns
    // even a subnormal component bit for bit (a 1/8 scaling would round
    // 3 * 2^-1074 to zero).
    assert_eq!(UnitQuaternion::ROTATE_SCALE_ABOVE, f64::MAX / 8.0);
    let tiny = 3.0 * f64::from_bits(1);
    let v = [UnitQuaternion::ROTATE_SCALE_ABOVE, tiny, 0.0];
    let got = UnitQuaternion::IDENTITY.rotate(v);
    assert_eq!(got.map(f64::to_bits), v.map(f64::to_bits));
}
