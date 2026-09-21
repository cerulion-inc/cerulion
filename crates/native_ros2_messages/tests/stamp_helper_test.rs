// SPDX-License-Identifier: AGPL-3.0-only
//! Behavioral coverage for the built-in `builtin_interfaces/Time::from_ns`
//! stamp helper.
//!
//! The helper is emitted by codegen onto the `Time` marker (the codegen SHAPE
//! is pinned in `cerulion_core/tests/codegen_test.rs`); here we exercise the
//! REAL generated type against HAND-COMPUTED oracle vectors — never a
//! self-compare. `from_ns` returns the `TimeShm` overlay (a marker cannot
//! carry `sec`/`nanosec`), which exposes `pub sec: i32` / `pub nanosec: u32`.
//!
//! Boundary behavior (loudest-cheap): a second count past `i32::MAX` (the ROS
//! 2038 `i32` horizon) fires a `debug_assert!` LOUDLY in debug/test builds and
//! SATURATES `sec` to `i32::MAX` in release — never a two's-complement wrap to
//! a negative time. Both build-mode arms are pinned below, cfg-gated so exactly
//! one is active per build.

use native_ros2_messages::builtin_interfaces::Time;

/// One nanosecond value per equivalence class → its hand-computed
/// `(sec, nanosec)` split. Zero, sub-second, exact-second, a realistic
/// unix-epoch stamp, and the `sec == i32::MAX` boundary (which must NOT trip
/// the overflow guard).
#[test]
fn from_ns_hand_oracle() {
    // (ns, want_sec, want_nanosec)
    const NS_PER_SEC: u64 = 1_000_000_000;
    let i32_max_secs = i32::MAX as u64; // 2_147_483_647

    let cases: &[(u64, i32, u32)] = &[
        (0, 0, 0),                                               // zero
        (123_456_789, 0, 123_456_789),                           // sub-second
        (2_000_000_000, 2, 0),                                   // exact second
        (1_704_067_200_500_000_000, 1_704_067_200, 500_000_000), // 2024-01-01T00:00:00.5Z
        // i32-second boundary: sec == i32::MAX, max nanosec — must not panic.
        (
            i32_max_secs * NS_PER_SEC + 999_999_999,
            i32::MAX,
            999_999_999,
        ),
    ];

    for &(ns, want_sec, want_nsec) in cases {
        let t = Time::from_ns(ns);
        assert_eq!(t.sec, want_sec, "from_ns({ns}).sec");
        assert_eq!(t.nanosec, want_nsec, "from_ns({ns}).nanosec");
    }
}

/// `nanosec` is always a valid ROS sub-second field (`< 1e9`) across the whole
/// in-bounds range, including at the `i32::MAX`-second boundary.
#[test]
fn from_ns_nanosec_always_in_range() {
    const NS_PER_SEC: u64 = 1_000_000_000;
    let boundary = i32::MAX as u64 * NS_PER_SEC + 999_999_999;
    // Every value here has `secs <= i32::MAX`, so none trips the debug guard.
    for ns in [
        0u64,
        1,
        999_999_999,
        1_000_000_000,
        123_456_789_012,
        boundary,
    ] {
        assert!(
            Time::from_ns(ns).nanosec < 1_000_000_000,
            "nanosec must be a valid ROS sub-second field for ns={ns}"
        );
    }
}

/// One second PAST `i32::MAX` — in debug/test builds the `debug_assert!` fires
/// LOUDLY (the developer catches the 2038 overflow immediately). Gated to
/// debug so `cargo test --release` runs the saturation twin below instead.
#[test]
#[cfg(debug_assertions)]
#[should_panic(expected = "exceeds i32::MAX seconds")]
fn from_ns_second_past_i32_max_panics_loudly_in_debug() {
    let ns = (i32::MAX as u64 + 1) * 1_000_000_000;
    let _ = Time::from_ns(ns);
}

/// One second PAST `i32::MAX` — release builds SATURATE `sec` to `i32::MAX` (a
/// defined, deterministic, non-negative value), never wrapping to `i32::MIN`.
#[test]
#[cfg(not(debug_assertions))]
fn from_ns_second_past_i32_max_saturates_in_release() {
    let ns = (i32::MAX as u64 + 1) * 1_000_000_000;
    let t = Time::from_ns(ns);
    assert_eq!(
        t.sec,
        i32::MAX,
        "release must saturate sec to i32::MAX, never two's-complement wrap"
    );
    assert_eq!(t.nanosec, 0);
}
