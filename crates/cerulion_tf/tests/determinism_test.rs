// SPDX-License-Identifier: AGPL-3.0-only
//! Bit-identical results across runs and across machines.
//!
//! A fixed program of compose, invert, apply and interpolate runs twice and
//! its `f64` output is compared bit for bit, run against run and against the
//! bits committed below. CI runs this on Linux x86_64 and macOS arm64, so a
//! platform that rounds differently, fuses a multiply-add or reaches a
//! platform math library fails here.
//!
//! The committed bits are a regression pin, not the correctness oracle: the
//! hand-written vectors in `se3_test.rs`, `interpolate_test.rs` and
//! `lookup_test.rs` are. The tying tests check that the pinned SLERP values
//! decode to the hand-written half-angle sines, and that the buffer's
//! interpolated lookup lands on the same bits as the direct interpolation
//! it is built from.
//!
//! The second program feeds a tf buffer and reads it back through
//! `lookup` and `lookup_latest`, so the walk order over the tree, the
//! fraction formed from the stamps and the composition along the chain are
//! all under the same bit pin.

use cerulion_tf::{TfBuffer, Transform, UnitQuaternion};
use std::f64::consts::FRAC_1_SQRT_2;

const N: usize = 47;

/// The fixed program. Every value it returns lands in the output, in order.
fn program() -> [f64; N] {
    let q = |x, y, z, w| UnitQuaternion::new(x, y, z, w).unwrap();
    let a = Transform::new([1.5, -2.25, 3.0], q(0.1, 0.2, 0.3, 0.9)).unwrap();
    let b = Transform::new([-3.0, 0.25, 1.0], q(-0.4, 0.1, 0.0, 0.8)).unwrap();
    let z90 = q(0.0, 0.0, FRAC_1_SQRT_2, FRAC_1_SQRT_2);

    let c = a.compose(&b).unwrap();
    let i = a.inverse().unwrap();
    let s = a.interpolate(&b, 0.37).unwrap();
    let p = c.apply_point([0.7, -0.2, 1.9]);
    let v = i.apply_vector([0.7, -0.2, 1.9]);
    let third = UnitQuaternion::IDENTITY.slerp(&z90, 1.0 / 3.0).unwrap();
    let short = UnitQuaternion::IDENTITY
        .slerp(&q(0.0, 0.0, -FRAC_1_SQRT_2, -FRAC_1_SQRT_2), 0.5)
        .unwrap();

    let mut out = [0.0; N];
    let mut k = 0;
    let mut push = |x: f64| {
        out[k] = x;
        k += 1;
    };
    for t in [&c, &i, &s] {
        t.translation().into_iter().for_each(&mut push);
        t.rotation().to_array().into_iter().for_each(&mut push);
    }
    p.into_iter().for_each(&mut push);
    v.into_iter().for_each(&mut push);
    third.to_array().into_iter().for_each(&mut push);
    short.to_array().into_iter().for_each(&mut push);
    // A slerp from an interpolated value: errors would compound here.
    let chain = third.slerp(&b.rotation(), 0.61).unwrap();
    chain.to_array().into_iter().for_each(&mut push);
    // The normalized-lerp branch: a 4-D angle of 5e-10, below the threshold.
    let lerp = UnitQuaternion::IDENTITY
        .slerp(&q(5e-10, 0.0, 0.0, 1.0), 0.5)
        .unwrap();
    lerp.to_array().into_iter().for_each(&mut push);
    // The 180 degree equal-norm tie (|b - a| == |b + a|): b is kept as given.
    let tie = UnitQuaternion::IDENTITY
        .slerp(&q(0.0, 0.0, 1.0, 0.0), 0.25)
        .unwrap();
    tie.to_array().into_iter().for_each(&mut push);
    out
}

fn bits(v: [f64; N]) -> [u64; N] {
    v.map(f64::to_bits)
}

/// The committed bits of [`program`].
const EXPECTED: [u64; N] = [
    0xbfd79435e50d7940,
    0xc00f9435e50d7944,
    0x4013af286bca1af4,
    0xbfd69dfbc23dcbda,
    0x3fc2f81d799f2f1a,
    0x3fd81387df4a0a8c,
    0x3feafea0196287f3,
    0x3ff3b48c20563b4a,
    0x3ff9e50d79435e50,
    0xc00bea712dcf7ea7,
    0xbfba43da0adc689a,
    0xbfca43da0adc689a,
    0xbfd3b2e388254e73,
    0x3fed8c554c37f5ad,
    0xbfc51eb851eb8518,
    0xbff5333333333333,
    0x4002147ae147ae15,
    0xbfbb7b225b1cd9af,
    0x3fc6ddd3523f2fc8,
    0x3fc9e8940924c617,
    0x3fee9ea59f28ec02,
    0x3fcc384a4145c390,
    0xc00299e64013299e,
    0x4017fecd6619bfee,
    0xbfcb5a0113f0e8de,
    0x3fb2dcf7ea712dd4,
    0x40002d458632a93a,
    0x0000000000000000,
    0x0000000000000000,
    0x3fd0907dc1930690,
    0x3feee8dd4748bf16,
    0x0000000000000000,
    0x0000000000000000,
    0x3fd87de2a6aea963,
    0x3fed906bcf328d46,
    0xbfd1e4809b106e86,
    0x3fb1e4809b106e86,
    0x3fbaeed574796727,
    0x3fee74fc72c8eaec,
    0x3df12e0be826d695,
    0x0000000000000000,
    0x0000000000000000,
    0x3ff0000000000000,
    0x0000000000000000,
    0x0000000000000000,
    0x3fd87de2a6aea963,
    0x3fed906bcf328d46,
];

/// Five lookups of seven values each.
const M: usize = 35;

/// The buffer program: a four-frame chain with two dynamic edges and two
/// static mounts, read at a midpoint, at the fraction 0.37, along the whole
/// chain, between two siblings, and at the latest common time in the
/// inverse direction.
fn buffer_program() -> [f64; M] {
    let q = |x, y, z, w| UnitQuaternion::new(x, y, z, w).unwrap();
    let a = Transform::new([1.5, -2.25, 3.0], q(0.1, 0.2, 0.3, 0.9)).unwrap();
    let b = Transform::new([-3.0, 0.25, 1.0], q(-0.4, 0.1, 0.0, 0.8)).unwrap();
    let z90 = q(0.0, 0.0, FRAC_1_SQRT_2, FRAC_1_SQRT_2);

    let mut buf = TfBuffer::new();
    buf.insert("map", "odom", 0, a).unwrap();
    buf.insert("map", "odom", 1_000, b).unwrap();
    buf.insert("odom", "base", 0, Transform::IDENTITY).unwrap();
    buf.insert(
        "odom",
        "base",
        1_000,
        Transform::new([2.0, 0.0, 0.0], z90).unwrap(),
    )
    .unwrap();
    buf.insert_static(
        "base",
        "lidar",
        Transform::new([0.17, 0.0, 0.11], z90).unwrap(),
    )
    .unwrap();
    buf.insert_static(
        "base",
        "camera",
        Transform::new([0.27, 0.0, 0.05], UnitQuaternion::IDENTITY).unwrap(),
    )
    .unwrap();

    let reads = [
        buf.lookup("odom", "base", 500).unwrap(),
        buf.lookup("map", "odom", 370).unwrap(),
        buf.lookup("map", "lidar", 370).unwrap(),
        buf.lookup("camera", "lidar", 123).unwrap(),
        buf.lookup_latest("lidar", "map").unwrap(),
    ];
    let mut out = [0.0; M];
    let mut k = 0;
    for t in &reads {
        for x in t.translation().into_iter().chain(t.rotation().to_array()) {
            out[k] = x;
            k += 1;
        }
    }
    out
}

fn buffer_bits(xs: [f64; M]) -> [u64; M] {
    xs.map(f64::to_bits)
}

#[test]
fn two_runs_give_identical_bits() {
    assert_eq!(bits(program()), bits(program()));
}

#[test]
fn two_buffer_runs_give_identical_bits() {
    assert_eq!(buffer_bits(buffer_program()), buffer_bits(buffer_program()));
}

/// The bits `buffer_program` produced, committed as the regression pin.
const EXPECTED_BUFFER: [u64; M] = [
    0x3ff0000000000000,
    0x0000000000000000,
    0x0000000000000000,
    0x0000000000000000,
    0x0000000000000000,
    0x3fd87de2a6aea963,
    0x3fed906bcf328d46,
    0xbfc51eb851eb8518,
    0xbff5333333333333,
    0x4002147ae147ae15,
    0xbfbb7b225b1cd9af,
    0x3fc6ddd3523f2fc8,
    0x3fc9e8940924c617,
    0x3fee9ea59f28ec02,
    0x3fe29c7882690f4f,
    0xbfece5dcbb7d0661,
    0x400010a1c148e2bd,
    0x3fbb32cba47724a7,
    0x3fc6f3687ba3d528,
    0x3fee05e2d3874fd0,
    0x3fd1adeb39378cde,
    0xbfb999999999999a,
    0x0000000000000000,
    0x3faeb851eb851eb8,
    0x0000000000000000,
    0x0000000000000000,
    0x3fe6a09e667f3bcc,
    0x3fe6a09e667f3bcc,
    0xbff25ed097b425ee,
    0xbfc617839a5bc7e0,
    0xbfd2e450672894ab,
    0xbfbc71c71c71c71c,
    0xbfdc71c71c71c71c,
    0xbfec71c71c71c71c,
    0x0000000000000000,
];

#[test]
fn the_buffer_output_matches_the_committed_bits() {
    let got = buffer_bits(buffer_program());
    for (k, (g, w)) in got.iter().zip(EXPECTED_BUFFER.iter()).enumerate() {
        assert_eq!(
            g,
            w,
            "buffer output {k}: got {g:#018x} ({}), committed {w:#018x} ({})",
            f64::from_bits(*g),
            f64::from_bits(*w)
        );
    }
}

#[test]
fn the_pinned_midpoint_lookup_is_the_hand_value() {
    // Outputs 0..7: halfway from the identity to (2 m, 90 about z) is
    // (1 m, 45 about z): translation (1, 0, 0), rotation
    // (0, 0, sin 22.5, cos 22.5).
    let want = [
        1.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.382_683_432_365_089_8,
        0.923_879_532_511_286_7,
    ];
    for (k, w) in want.iter().enumerate() {
        let got = f64::from_bits(EXPECTED_BUFFER[k]);
        assert!((got - w).abs() <= 1e-15, "buffer output {k}: {got} vs {w}");
    }
}

#[test]
fn the_pinned_fraction_lookup_is_the_direct_interpolation_bit_for_bit() {
    // Outputs 7..14: `map -> odom` at 370 of 1000 ns is `a.interpolate(&b,
    // 0.37)`, the value the SE(3) program pins at outputs 14..21. The
    // fraction 370 / 1000 rounds to the same double as the literal 0.37, so
    // the buffer's walk adds nothing to the math.
    assert_eq!(&EXPECTED_BUFFER[7..14], &EXPECTED[14..21]);
}

#[test]
fn the_pinned_sibling_lookup_is_the_hand_value() {
    // Outputs 21..28: the lidar seen from the camera. Both are mounts on
    // `base`, so the offset is the difference of the mounts, (0.17 - 0.27,
    // 0, 0.11 - 0.05), and the rotation is the lidar's 90 about z.
    let want = [-0.1, 0.0, 0.06, 0.0, 0.0, FRAC_1_SQRT_2, FRAC_1_SQRT_2];
    for (k, w) in want.iter().enumerate() {
        let got = f64::from_bits(EXPECTED_BUFFER[21 + k]);
        assert!(
            (got - w).abs() <= 1e-15,
            "buffer output {}: {got} vs {w}",
            21 + k
        );
    }
}

#[test]
fn the_output_matches_the_committed_bits() {
    let got = bits(program());
    for (k, (g, w)) in got.iter().zip(EXPECTED.iter()).enumerate() {
        assert_eq!(
            g,
            w,
            "output {k}: got {g:#018x} ({}), committed {w:#018x} ({})",
            f64::from_bits(*g),
            f64::from_bits(*w)
        );
    }
}

#[test]
fn the_pinned_slerp_values_are_the_hand_half_angle_sines() {
    // Outputs 27..31: a third of 90 about z is 30 about z, (0, 0, sin 15,
    // cos 15). Outputs 31..35: halfway on the short arc to -(90 about z) is
    // 45 about z, (0, 0, sin 22.5, cos 22.5).
    let want = [
        0.0,
        0.0,
        0.258_819_045_102_520_76,
        0.965_925_826_289_068_3,
        0.0,
        0.0,
        0.382_683_432_365_089_8,
        0.923_879_532_511_286_7,
    ];
    for (k, w) in want.iter().enumerate() {
        let got = f64::from_bits(EXPECTED[27 + k]);
        assert!((got - w).abs() <= 1e-15, "output {}: {got} vs {w}", 27 + k);
    }
}

#[test]
fn the_pinned_lerp_and_tie_values_are_the_hand_values() {
    // Outputs 39..43: halfway to a 1e-9 rad turn about x on the lerp
    // branch is (2.5e-10, 0, 0, 1). Outputs 43..47: a quarter of the way
    // to 180 about z, with the sign kept as given, is 45 about z,
    // (0, 0, sin 22.5, cos 22.5).
    let want = [
        2.5e-10,
        0.0,
        0.0,
        1.0,
        0.0,
        0.0,
        0.382_683_432_365_089_8,
        0.923_879_532_511_286_7,
    ];
    for (k, w) in want.iter().enumerate() {
        let got = f64::from_bits(EXPECTED[39 + k]);
        assert!((got - w).abs() <= 1e-15, "output {}: {got} vs {w}", 39 + k);
    }
}
