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
//! hand-written vectors in `se3_test.rs` and `interpolate_test.rs` are. The
//! last test ties the two together by checking that the pinned SLERP
//! values decode to the hand-written half-angle sines.

use cerulion_tf::{Transform, UnitQuaternion};
use std::f64::consts::FRAC_1_SQRT_2;

const N: usize = 39;

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
];

#[test]
fn two_runs_give_identical_bits() {
    assert_eq!(bits(program()), bits(program()));
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
