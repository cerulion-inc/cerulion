// SPDX-License-Identifier: AGPL-3.0-only
//! The exact sign of a 4-D dot product of unit quaternions.
//!
//! SLERP picks its arc from the sign of `a . b`. Near a half turn the dot
//! product is close to zero and the four rounded products cancel, so the
//! rounded sum can have the wrong sign: the arc taken is then longer by a
//! rounding error, and the midpoint turns half a turn away from the
//! shortest-arc one. This module gets the sign of the exact real-number dot
//! product of the stored components instead.
//!
//! Each product is split into its rounded value and its exact rounding
//! error (Dekker's product, with no fused multiply-add), and the parts are
//! summed into a nonoverlapping expansion (Shewchuk's grow-expansion of
//! Knuth two-sums). The sign of the most significant nonzero part is the
//! sign of the exact sum. Every step is a plain `+ - *` or a scaling by a
//! power of two, so the answer is the same on every target.
//!
//! A product below [`LARGE_PRODUCT`] can underflow, and its rounding error
//! with it, so it is not summed at its own scale. When the large products
//! leave a sum below [`LARGE_SUM`], where the small ones could decide the
//! sign, the sum is rescaled by `2^1100` and the small products are
//! recomputed from inputs scaled by `2^550`, where nothing underflows. The
//! sign is therefore exact for every pair of finite components of magnitude
//! at most 1, which covers every unit quaternion.

use std::cmp::Ordering;

/// `2^27 + 1`: multiplying by it splits a binary64 value into two halves of
/// at most 26 significant bits each, whose pairwise products are exact.
const SPLITTER: f64 = 134_217_729.0;

/// `2^-960`. In a product at least this large, of factors at most 1, the
/// last bits of the two factors weigh at least `2^-1065` together, so its
/// rounding error and every partial product of the split are exact
/// multiples of `2^-1065`: representable, never lost to underflow.
const LARGE_PRODUCT: f64 = f64::from_bits(63 << 52);

/// `2^-900`. A sum of large products at least this big outweighs the four
/// small products together (each below `2^-960`), so it alone has the sign.
const LARGE_SUM: f64 = f64::from_bits(123 << 52);

/// `2^550`. Small products are recomputed from factors scaled by this,
/// which lifts their bits clear of the subnormal range; a large sum below
/// [`LARGE_SUM`] is scaled by its square to match, and stays below `2^200`.
const SMALL_SCALE: f64 = f64::from_bits(1573 << 52);

/// The sign of the exact dot product `a . b` of components of magnitude at
/// most 1, as an ordering against zero.
pub(crate) fn dot4_sign(a: [f64; 4], b: [f64; 4]) -> Ordering {
    let mut parts = [0.0_f64; 16];
    let mut len = 0;
    let mut small = [false; 4];
    for i in 0..4 {
        let (p, e) = two_product(a[i], b[i]);
        if p.abs() >= LARGE_PRODUCT {
            grow(&mut parts, &mut len, e);
            grow(&mut parts, &mut len, p);
        } else {
            small[i] = true;
        }
    }
    let top = most_significant(&parts[..len]);
    if top.abs() >= LARGE_SUM {
        return top.total_cmp(&0.0);
    }
    for part in parts[..len].iter_mut() {
        *part = (*part * SMALL_SCALE) * SMALL_SCALE;
    }
    for i in 0..4 {
        if small[i] {
            let (p, e) = two_product(a[i] * SMALL_SCALE, b[i] * SMALL_SCALE);
            grow(&mut parts, &mut len, e);
            grow(&mut parts, &mut len, p);
        }
    }
    most_significant(&parts[..len]).total_cmp(&0.0)
}

/// The largest nonzero part of a nonoverlapping expansion (it is the last
/// one and carries the sign of the sum), or `+0.0` for a zero sum, so that
/// comparing it with zero gives the sign of the sum.
fn most_significant(parts: &[f64]) -> f64 {
    parts
        .iter()
        .rev()
        .copied()
        .find(|c| *c != 0.0)
        .unwrap_or(0.0)
}

/// Adds `v` to the nonoverlapping expansion `parts[..len]` (smallest part
/// first) and keeps it nonoverlapping: the expansion grows by one part.
fn grow(parts: &mut [f64; 16], len: &mut usize, v: f64) {
    let mut carry = v;
    for part in parts[..*len].iter_mut() {
        let (s, e) = two_sum(carry, *part);
        *part = e;
        carry = s;
    }
    parts[*len] = carry;
    *len += 1;
}

/// `(s, e)` with `s = fl(a + b)` and `s + e = a + b` exactly (Knuth).
fn two_sum(a: f64, b: f64) -> (f64, f64) {
    let s = a + b;
    let bb = s - a;
    let e = (a - (s - bb)) + (b - bb);
    (s, e)
}

/// `(hi, lo)` with `hi + lo = a` exactly and each half at most 26 bits.
fn split(a: f64) -> (f64, f64) {
    let c = SPLITTER * a;
    let hi = c - (c - a);
    (hi, a - hi)
}

/// `(p, e)` with `p = fl(a * b)` and `p + e = a * b` exactly (Dekker).
fn two_product(a: f64, b: f64) -> (f64, f64) {
    let p = a * b;
    let (ah, al) = split(a);
    let (bh, bl) = split(b);
    let e = al * bl - (((p - ah * bh) - al * bh) - ah * bl);
    (p, e)
}

#[cfg(test)]
mod tests {
    use super::{dot4_sign, split, two_product, two_sum};
    use std::cmp::Ordering;

    /// `2^-k`, written out so no test leans on the code under test.
    fn pow2_neg(k: i32) -> f64 {
        f64::powi(2.0, -k)
    }

    /// `2^-k` from its bit pattern, down to the smallest subnormal `2^-1074`.
    fn bits_pow2_neg(k: u64) -> f64 {
        if k <= 1022 {
            f64::from_bits((1023 - k) << 52)
        } else {
            f64::from_bits(1 << (1074 - k))
        }
    }

    #[test]
    fn the_scale_constants_are_the_powers_of_two_they_name() {
        assert_eq!(super::LARGE_PRODUCT, pow2_neg(960));
        assert_eq!(super::LARGE_SUM, pow2_neg(900));
        assert_eq!(super::SMALL_SCALE, f64::powi(2.0, 550));
        assert_eq!(bits_pow2_neg(1074), 5e-324);
        assert_eq!(bits_pow2_neg(1022), f64::MIN_POSITIVE);
    }

    #[test]
    fn two_sum_returns_the_bits_the_rounded_sum_drops() {
        // 1 + 2^-60 rounds to 1; the dropped 2^-60 comes back as the error,
        // in either argument order.
        assert_eq!(two_sum(1.0, pow2_neg(60)), (1.0, pow2_neg(60)));
        assert_eq!(two_sum(pow2_neg(60), 1.0), (1.0, pow2_neg(60)));
        assert_eq!(two_sum(-1.0, -pow2_neg(60)), (-1.0, -pow2_neg(60)));
        // An exact sum has no error.
        assert_eq!(two_sum(0.5, 0.25), (0.75, 0.0));
    }

    #[test]
    fn split_gives_two_halves_that_sum_back_exactly() {
        // 1 + 2^-30 + 2^-52: the top 26 bits are 1, the rest is the low half.
        let a = 1.0 + pow2_neg(30) + pow2_neg(52);
        assert_eq!(split(a), (1.0, pow2_neg(30) + pow2_neg(52)));
        // 1 + 2^-20 fits in the high half entirely.
        assert_eq!(split(1.0 + pow2_neg(20)), (1.0 + pow2_neg(20), 0.0));
        // Rounding the high half up leaves a negative low half:
        // 1 - 2^-53 splits as 1 and -2^-53.
        assert_eq!(split(1.0 - pow2_neg(53)), (1.0, -pow2_neg(53)));
        // The split scales with the value: 2^-20 + 2^-30 + 2^-60 keeps its
        // top 26 bits, 2^-20 + 2^-30, in the high half.
        let small = pow2_neg(20) + pow2_neg(30) + pow2_neg(60);
        assert_eq!(split(small), (pow2_neg(20) + pow2_neg(30), pow2_neg(60)));
    }

    #[test]
    fn two_product_returns_the_bits_the_rounded_product_drops() {
        // (1 + 2^-30)(1 + 2^-31) = 1 + 3 * 2^-31 + 2^-61; the last term is
        // below the ulp of 1 and is the error.
        let (p, e) = two_product(1.0 + pow2_neg(30), 1.0 + pow2_neg(31));
        assert_eq!(p, 1.0 + 3.0 * pow2_neg(31));
        assert_eq!(e, pow2_neg(61));
        // (1 - 2^-53)^2 = 1 - 2^-52 + 2^-106: rounds to 1 - 2^-52.
        let (p, e) = two_product(1.0 - pow2_neg(53), 1.0 - pow2_neg(53));
        assert_eq!(p, 1.0 - pow2_neg(52));
        assert_eq!(e, pow2_neg(106));
        // Opposite signs: (1 + 2^-30)(-(1 - 2^-30)) = -1 + 2^-60, which
        // rounds to -1 (the ulp below 1 is 2^-53).
        let (p, e) = two_product(1.0 + pow2_neg(30), -(1.0 - pow2_neg(30)));
        assert_eq!((p, e), (-1.0, pow2_neg(60)));
        // An exact product has no error.
        assert_eq!(two_product(0.5, -0.25), (-0.125, 0.0));
    }

    #[test]
    fn the_sign_of_an_ordinary_dot_product() {
        assert_eq!(
            dot4_sign([0.0, 0.8, 0.0, 0.6], [0.0, 0.8, 0.0, -0.6]),
            Ordering::Greater
        );
        assert_eq!(
            dot4_sign([0.0, 0.6, 0.0, 0.8], [0.0, 0.6, 0.0, -0.8]),
            Ordering::Less
        );
        assert_eq!(
            dot4_sign([0.0, 0.0, 0.0, 1.0], [0.0, 0.0, 1.0, 0.0]),
            Ordering::Equal
        );
        assert_eq!(dot4_sign([0.0; 4], [0.0; 4]), Ordering::Equal);
    }

    #[test]
    fn a_dot_product_the_rounded_sum_gets_wrong() {
        // Exact terms: (1 + 2^-30)(1 - 2^-30) = 1 - 2^-60, then -1 * 1 = -1,
        // then 2^-70 * 1. Exact sum: -2^-60 + 2^-70 < 0. The rounded first
        // product is 1 (2^-60 is below its half ulp), so the rounded sum is
        // 1 - 1 + 2^-70 = 2^-70 > 0: the wrong sign.
        let a = [1.0 + pow2_neg(30), -1.0, pow2_neg(70), 0.0];
        let b = [1.0 - pow2_neg(30), 1.0, 1.0, 0.0];
        let rounded = ((a[0] * b[0] + a[1] * b[1]) + a[2] * b[2]) + a[3] * b[3];
        assert!(
            rounded > 0.0,
            "the case must fool the rounded sum: {rounded:e}"
        );
        assert_eq!(dot4_sign(a, b), Ordering::Less);
        // Its mirror image is positive.
        let neg_a = [-a[0], -a[1], -a[2], -a[3]];
        assert_eq!(dot4_sign(neg_a, b), Ordering::Greater);
    }

    #[test]
    fn the_sign_comes_from_every_term() {
        // A dot product that is exactly zero until the last term decides it,
        // in each of the four slots: any dropped term gives Equal.
        for slot in 0..4 {
            let mut a = [0.5, -0.5, 0.5, -0.5];
            let b = [0.5, 0.5, 0.5, 0.5];
            assert_eq!(dot4_sign(a, b), Ordering::Equal);
            a[slot] += pow2_neg(40);
            assert_eq!(dot4_sign(a, b), Ordering::Greater, "slot {slot}");
            a[slot] -= 2.0 * pow2_neg(40);
            assert_eq!(dot4_sign(a, b), Ordering::Less, "slot {slot}");
        }
    }

    #[test]
    fn a_product_that_underflows_still_decides_the_sign() {
        // [1, 2^-700, 0, 0] . [0, -2^-700, 1, 0] = -2^-1400, below the
        // smallest subnormal: the rounded product is -0.
        let t = bits_pow2_neg(700);
        assert_eq!(
            dot4_sign([1.0, t, 0.0, 0.0], [0.0, -t, 1.0, 0.0]),
            Ordering::Less
        );
        assert_eq!(
            dot4_sign([1.0, t, 0.0, 0.0], [0.0, t, 1.0, 0.0]),
            Ordering::Greater
        );
        // Two large products cancel exactly and leave the underflowing one:
        // 0.25 - 0.25 - 2^-1400.
        let a = [0.5, 0.5, t, 0.0];
        assert_eq!(dot4_sign(a, [0.5, -0.5, -t, 0.0]), Ordering::Less);
        assert_eq!(dot4_sign(a, [0.5, -0.5, t, 0.0]), Ordering::Greater);
        // Subnormal factors: 2^-1074 * -1 and 2^-1074 * -2^-1074.
        let s = bits_pow2_neg(1074);
        assert_eq!(
            dot4_sign([s, 0.0, 0.0, 0.0], [-1.0, 0.0, 0.0, 0.0]),
            Ordering::Less
        );
        assert_eq!(
            dot4_sign([0.0, 0.0, s, 0.0], [0.0, 0.0, -s, 0.0]),
            Ordering::Less
        );
        assert_eq!(
            dot4_sign([0.0, 0.0, 0.0, s], [0.0, 0.0, 0.0, s]),
            Ordering::Greater
        );
    }

    #[test]
    fn a_small_remainder_of_large_products_is_weighed_against_the_small_products() {
        // (1 + 2^-52) * 2^-900 (1 + 2^-52) - 2^-900 (1 + 2^-51) = 2^-1004:
        // two large products that cancel down to their rounding error.
        let a = [1.0 + pow2_neg(52), 1.0, 0.0, 0.0];
        let b0 = bits_pow2_neg(900) * (1.0 + pow2_neg(52));
        let b1 = -(bits_pow2_neg(900) * (1.0 + pow2_neg(51)));
        // A small product of -2^-961 outweighs the remainder ...
        let (a2, b2) = (bits_pow2_neg(480), bits_pow2_neg(481));
        assert_eq!(
            dot4_sign([a[0], a[1], a2, 0.0], [b0, b1, -b2, 0.0]),
            Ordering::Less
        );
        assert_eq!(
            dot4_sign([a[0], a[1], a2, 0.0], [b0, b1, b2, 0.0]),
            Ordering::Greater
        );
        // ... and one of -2^-1010 does not.
        let c = bits_pow2_neg(505);
        assert_eq!(
            dot4_sign([a[0], a[1], 0.0, c], [b0, b1, 0.0, -c]),
            Ordering::Greater
        );
        // The remainder alone, with no small product, keeps its sign.
        assert_eq!(dot4_sign(a, [b0, b1, 0.0, 0.0]), Ordering::Greater);
        assert_eq!(dot4_sign(a, [-b0, -b1, 0.0, 0.0]), Ordering::Less);
    }

    #[test]
    fn a_large_sum_outweighs_every_small_product() {
        // 2^-900 - 2^-961 and its negation: the large product decides.
        let (a1, b1) = (bits_pow2_neg(480), bits_pow2_neg(481));
        let big = bits_pow2_neg(900);
        assert_eq!(
            dot4_sign([1.0, a1, 0.0, 0.0], [big, -b1, 0.0, 0.0]),
            Ordering::Greater
        );
        assert_eq!(
            dot4_sign([1.0, a1, 0.0, 0.0], [-big, b1, 0.0, 0.0]),
            Ordering::Less
        );
        // A sum of order 1 with a small product beside it.
        assert_eq!(
            dot4_sign([1.0, a1, 0.0, 0.0], [-0.5, b1, 0.0, 0.0]),
            Ordering::Less
        );
    }
}
