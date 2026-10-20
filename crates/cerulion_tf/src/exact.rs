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
//! error (Dekker's product, with no fused multiply-add), and the eight parts
//! are summed into a nonoverlapping expansion (Shewchuk's grow-expansion of
//! Knuth two-sums). The sign of the most significant nonzero part is the
//! sign of the exact sum. Every step is a plain `+ - *`, so the answer is
//! the same on every target.
//!
//! The split is exact for components of magnitude at most 1 whose products
//! do not underflow (each nonzero product above about `1e-292`). A smaller
//! product can lose bits, which can change the sign only when the whole dot
//! product is within about that much of zero.

use std::cmp::Ordering;

/// `2^27 + 1`: multiplying by it splits a binary64 value into two halves of
/// at most 26 significant bits each, whose pairwise products are exact.
const SPLITTER: f64 = 134_217_729.0;

/// The sign of the exact dot product `a . b`, as an ordering against zero.
pub(crate) fn dot4_sign(a: [f64; 4], b: [f64; 4]) -> Ordering {
    let mut parts = [0.0_f64; 8];
    let mut len = 0;
    for i in 0..4 {
        let (p, e) = two_product(a[i], b[i]);
        grow(&mut parts, &mut len, e);
        grow(&mut parts, &mut len, p);
    }
    // The largest nonzero part is last and carries the sign of the sum.
    parts[..len]
        .iter()
        .rev()
        .find(|c| **c != 0.0)
        .map_or(Ordering::Equal, |c| c.total_cmp(&0.0))
}

/// Adds `v` to the nonoverlapping expansion `parts[..len]` (smallest part
/// first) and keeps it nonoverlapping: the expansion grows by one part.
fn grow(parts: &mut [f64; 8], len: &mut usize, v: f64) {
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
}
