// SPDX-License-Identifier: AGPL-3.0-only
//! Direct unit tests for `MaxSliceLen` newtype.
//!
//! The newtype's contract — `try_new(n) -> Some` iff
//! `WireHeader::SIZE <= n <= u32::MAX`, `try_new(n) -> None` otherwise —
//! is load-bearing for the entire compile-time-prevention story
//! introduced by `b10246b`. Pre-this-file, the contract was tested
//! indirectly via downstream resolver / publisher tests. A future
//! refactor that loosens `<` to `<=` or `>=` to `>` would silently
//! slip past those indirect checks. This file pins the contract
//! directly so a single-line bound flip fails loud.

use cerulion_core::wire::{MaxSliceLen, WireHeader};

#[test]
fn try_new_rejects_zero() {
    assert!(MaxSliceLen::try_new(0).is_none());
}

#[test]
fn try_new_rejects_one() {
    assert!(MaxSliceLen::try_new(1).is_none());
}

#[test]
fn try_new_rejects_just_below_wire_header_size() {
    // 31 = WireHeader::SIZE - 1 = floor of the rejection band.
    assert_eq!(WireHeader::SIZE, 32);
    assert!(MaxSliceLen::try_new(31).is_none());
}

#[test]
fn try_new_accepts_exactly_wire_header_size() {
    // 32 = WireHeader::SIZE = smallest legal MaxSliceLen.
    // Header-only payloads (e.g. std_msgs::Empty, total = 32) are
    // valid; rejecting 32 would break empty-message schemas.
    let v = MaxSliceLen::try_new(32).expect("32 must be accepted");
    assert_eq!(v.get(), 32);
}

#[test]
fn try_new_accepts_one_above_wire_header_size() {
    let v = MaxSliceLen::try_new(33).expect("33 must be accepted");
    assert_eq!(v.get(), 33);
}

#[test]
fn try_new_accepts_typical_value() {
    let v = MaxSliceLen::try_new(64 * 1024).expect("64 KiB must be accepted");
    assert_eq!(v.get(), 64 * 1024);
}

#[test]
fn try_new_accepts_u32_max_at_ceiling() {
    // u32::MAX is the wire-format ceiling (`WireHeader::total_size:
    // u32` → 4 GiB max).
    let v = MaxSliceLen::try_new(u32::MAX).expect("u32::MAX must be accepted");
    assert_eq!(v.get(), u32::MAX);
}

#[test]
fn const_new_accepts_at_floor() {
    // `const_new` is the const-eval-aware constructor used by codegen.
    // The const itself proves the floor (32) is accepted at const-eval.
    const FLOOR: MaxSliceLen = MaxSliceLen::const_new(32);
    assert_eq!(FLOOR.get(), 32);
}

#[test]
fn const_new_accepts_at_ceiling() {
    const CEILING: MaxSliceLen = MaxSliceLen::const_new(u32::MAX);
    assert_eq!(CEILING.get(), u32::MAX);
}

#[test]
#[should_panic(expected = "MaxSliceLen::const_new requires n >= WireHeader::SIZE")]
fn const_new_panics_on_zero_at_runtime() {
    // `const_new` in non-const context is still a runtime panic on
    // invalid input. The compile-fail path is exercised by the codegen
    // tests (which generate code containing `const_new(<N>)` and
    // compile it).
    let _ = MaxSliceLen::const_new(0);
}

#[test]
#[should_panic(expected = "MaxSliceLen::const_new requires n >= WireHeader::SIZE")]
fn const_new_panics_on_just_below_floor() {
    let _ = MaxSliceLen::const_new(31);
}

#[test]
fn get_round_trips_construction() {
    for n in [32u32, 33, 64, 256, 4096, 65536, 1 << 24, u32::MAX] {
        let v = MaxSliceLen::try_new(n).unwrap_or_else(|| panic!("{} must be accepted", n));
        assert_eq!(v.get(), n, "round-trip failed for {}", n);
    }
}

#[test]
fn repr_transparent_is_zero_overhead() {
    // `MaxSliceLen` is `#[repr(transparent)]` over `NonZeroU32`, so it
    // must occupy exactly 4 bytes (same as u32) and the niche
    // optimization should let `Option<MaxSliceLen>` also fit in 4
    // bytes. This is the proof that the compile-time-prevention work
    // introduces zero runtime overhead on the hot path.
    assert_eq!(std::mem::size_of::<MaxSliceLen>(), 4);
    assert_eq!(std::mem::size_of::<Option<MaxSliceLen>>(), 4);
}
