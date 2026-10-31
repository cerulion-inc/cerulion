// SPDX-License-Identifier: AGPL-3.0-only
//! Deterministic sine and arctangent on the small domains SLERP needs.
//!
//! The platform math library is not used: `f64::sin` and `f64::atan` call
//! into libm, whose last bit differs between libraries and targets. These
//! kernels are fixed polynomials evaluated by Horner's rule with plain
//! `+ - * / sqrt`, so every target gives the same bits.

/// Taylor coefficients of `sin(x) / x` in powers of `x²`: `(-1)^k / (2k+1)!`
/// for `k = 0..=10`, the last term `x^21 / 21!`. On `[0, pi/2]` the first
/// dropped term, `x^23 / 23!`, is below `1.3e-18`, under a hundredth of an
/// ulp of the result.
const SIN_COEFFS: [f64; 11] = [
    1.0,
    -1.0 / 6.0,
    1.0 / 120.0,
    -1.0 / 5_040.0,
    1.0 / 362_880.0,
    -1.0 / 39_916_800.0,
    1.0 / 6_227_020_800.0,
    -1.0 / 1_307_674_368_000.0,
    1.0 / 355_687_428_096_000.0,
    -1.0 / 121_645_100_408_832_000.0,
    1.0 / 51_090_942_171_709_440_000.0,
];

/// Taylor coefficients of `atan(x) / x` in powers of `x²`: `(-1)^k / (2k+1)`
/// for `k = 0..=10`, the last term `x^21 / 21`. On `|x| <= tan(pi/16)`
/// (about 0.199) the first dropped term, `x^23 / 23`, is below `3.3e-18`,
/// under a tenth of an ulp of the result.
const ATAN_COEFFS: [f64; 11] = [
    1.0,
    -1.0 / 3.0,
    1.0 / 5.0,
    -1.0 / 7.0,
    1.0 / 9.0,
    -1.0 / 11.0,
    1.0 / 13.0,
    -1.0 / 15.0,
    1.0 / 17.0,
    -1.0 / 19.0,
    1.0 / 21.0,
];

/// `x * (c[0] + x² (c[1] + x² (c[2] + ...)))`, Horner from the last term.
fn odd_series(x: f64, coeffs: &[f64]) -> f64 {
    let x2 = x * x;
    let mut acc = 0.0;
    for c in coeffs.iter().rev() {
        acc = acc * x2 + c;
    }
    x * acc
}

/// `sin(x)` for `x` in `[0, pi/2]`, accurate to a few ulps.
pub(crate) fn sin_quarter_turn(x: f64) -> f64 {
    odd_series(x, &SIN_COEFFS)
}

/// `atan(r)` for `r` in `[0, 1]`, accurate to a few ulps. The half-angle
/// steps keep that accuracy a few ulps above 1, where a SLERP ratio can
/// round near a half turn.
///
/// Two half-angle steps, `atan(x) = 2 atan(x / (1 + sqrt(1 + x²)))`, bring
/// the argument down to `[0, tan(pi/16)]`, where the series converges fast.
pub(crate) fn atan_unit(r: f64) -> f64 {
    let h1 = r / (1.0 + (1.0 + r * r).sqrt());
    let h2 = h1 / (1.0 + (1.0 + h1 * h1).sqrt());
    4.0 * odd_series(h2, &ATAN_COEFFS)
}

#[cfg(test)]
mod tests {
    use super::{atan_unit, sin_quarter_turn};
    use std::f64::consts::{FRAC_1_SQRT_2, FRAC_PI_2, FRAC_PI_4, FRAC_PI_6};

    /// A few ulps of a value near 1 (one ulp of 1.0 is 2.2e-16).
    const ULPS_NEAR_ONE: f64 = 4.5e-16;

    /// Hand values: sines and arctangents of round angles, written out.
    fn close(got: f64, want: f64, tol: f64) {
        assert!((got - want).abs() <= tol, "got {got:e}, want {want:e}");
    }

    #[test]
    fn sine_of_round_angles() {
        assert_eq!(sin_quarter_turn(0.0), 0.0);
        close(sin_quarter_turn(FRAC_PI_6), 0.5, ULPS_NEAR_ONE);
        close(sin_quarter_turn(FRAC_PI_4), FRAC_1_SQRT_2, ULPS_NEAR_ONE);
        // sin 60 degrees = sqrt(3) / 2.
        close(
            sin_quarter_turn(std::f64::consts::FRAC_PI_3),
            0.866_025_403_784_438_6,
            ULPS_NEAR_ONE,
        );
        close(sin_quarter_turn(FRAC_PI_2), 1.0, ULPS_NEAR_ONE);
        // sin 15 degrees.
        close(
            sin_quarter_turn(FRAC_PI_6 / 2.0),
            0.258_819_045_102_520_76,
            1e-16,
        );
    }

    #[test]
    fn sine_of_a_tiny_angle_is_the_angle() {
        // sin x = x - x³/6: for x = 1e-8 the cubic term is 1.7e-25.
        close(sin_quarter_turn(1e-8), 1e-8, 1e-24);
    }

    #[test]
    fn arctangent_of_round_ratios() {
        assert_eq!(atan_unit(0.0), 0.0);
        close(atan_unit(1.0), FRAC_PI_4, ULPS_NEAR_ONE);
        // atan(1 / sqrt 3) = pi / 6.
        close(atan_unit(0.577_350_269_189_625_8), FRAC_PI_6, ULPS_NEAR_ONE);
        // atan(2 - sqrt 3) = pi / 12.
        close(
            atan_unit(0.267_949_192_431_122_7),
            0.261_799_387_799_149_4,
            1e-16,
        );
        // atan(sqrt 2 - 1) = pi / 8.
        close(
            atan_unit(0.414_213_562_373_095_03),
            std::f64::consts::FRAC_PI_8,
            1e-16,
        );
    }

    #[test]
    fn arctangent_of_a_tiny_ratio_is_the_ratio() {
        close(atan_unit(1e-10), 1e-10, 1e-26);
    }
}
