// SPDX-License-Identifier: AGPL-3.0-only
//! Per-schema slot sizing for `TransportManager::create_publisher_typed`.
//!
//! Asserts that fixed-schema publishers created via `create_publisher_typed`
//! ignore the user-supplied `max_slice_len` and use exactly
//! `WireHeader::SIZE + T::WIRE_FIXED_SIZE`. Variable-schema publishers
//! continue to honour `max_slice_len` (required, errors if `None`).
//!
//! Touches the iceoryx2 singleton, so this test file must run with
//! `--test-threads=1` (matches the parent serial-test harness).

use cerulion_core::wire::MaxSliceLen;
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::SystemTime;

use cerulion_core::message::ShmMessage;
use cerulion_core::transport::TransportManager;
use cerulion_core::wire::WireHeader;
use cerulion_core::TransportError;
use native_ros2_messages::geometry_msgs::Vector3;
use native_ros2_messages::std_msgs::String as RosString;

static TOPIC_COUNTER: AtomicU64 = AtomicU64::new(0);

fn unique_topic(base: &str) -> String {
    let nanos = SystemTime::now()
        .duration_since(SystemTime::UNIX_EPOCH)
        .unwrap()
        .as_nanos();
    let id = TOPIC_COUNTER.fetch_add(1, Ordering::Relaxed);
    format!("test/pool_size/{base}/{nanos}/{id}")
}

#[test]
fn typed_fixed_schema_ignores_max_slice_len_argument() {
    let mgr = TransportManager::get_or_init().expect("init");
    let topic = unique_topic("fixed_ignore_msl");

    // User passes a wildly oversized hint — typed creation must ignore it
    // for fixed schemas and use the schema-derived slot.
    let publisher = mgr
        .create_publisher_typed::<Vector3>(&topic, Some(MaxSliceLen::const_new(1 << 20)))
        .expect("typed create_publisher should succeed for fixed schema");

    let expected: u32 = (WireHeader::SIZE + <Vector3 as ShmMessage>::WIRE_FIXED_SIZE) as u32;
    assert_eq!(
        publisher.max_slice_len().get(),
        expected,
        "fixed-schema typed publisher must size its iceoryx2 slot at \
         WireHeader::SIZE + WIRE_FIXED_SIZE = {expected}, ignoring the \
         user-supplied max_slice_len hint"
    );
}

#[test]
fn typed_fixed_schema_works_when_hint_is_none() {
    let mgr = TransportManager::get_or_init().expect("init");
    let topic = unique_topic("fixed_no_hint");

    let publisher = mgr
        .create_publisher_typed::<Vector3>(&topic, None)
        .expect("typed create_publisher should succeed for fixed schema with None hint");

    let expected: u32 = (WireHeader::SIZE + <Vector3 as ShmMessage>::WIRE_FIXED_SIZE) as u32;
    assert_eq!(publisher.max_slice_len().get(), expected);
}

#[test]
fn typed_variable_schema_requires_max_slice_len() {
    let mgr = TransportManager::get_or_init().expect("init");
    let topic = unique_topic("var_requires_msl");

    match mgr.create_publisher_typed::<RosString>(&topic, None) {
        Err(TransportError::MaxSliceLenRequired { topic: t }) => {
            assert_eq!(t, topic, "MaxSliceLenRequired must carry the topic");
        }
        Err(other) => panic!("expected MaxSliceLenRequired, got {other:?}"),
        Ok(_) => panic!("variable-schema typed publisher should reject None max_slice_len"),
    }
}

#[test]
fn typed_variable_schema_uses_supplied_max_slice_len() {
    let mgr = TransportManager::get_or_init().expect("init");
    let topic = unique_topic("var_uses_msl");

    let publisher = mgr
        .create_publisher_typed::<RosString>(&topic, Some(MaxSliceLen::const_new(2048)))
        .expect("typed create_publisher should succeed for variable schema");

    assert_eq!(
        publisher.max_slice_len().get(),
        2048,
        "variable-schema typed publisher must use the user-supplied max_slice_len"
    );
}

#[test]
fn typed_fixed_schema_publishes_through_smaller_slot_than_legacy_call() {
    // Side-by-side proof that the typed path produces a strictly smaller
    // pool slot for fixed schemas than the legacy `create_publisher_simple`
    // path called with a default-style hint.
    let mgr = TransportManager::get_or_init().expect("init");

    let typed_topic = unique_topic("fixed_typed_small");
    let legacy_topic = unique_topic("fixed_legacy_big");

    let typed = mgr
        .create_publisher_typed::<Vector3>(&typed_topic, Some(MaxSliceLen::const_new(65_536)))
        .expect("typed");
    let legacy = mgr
        .create_publisher_simple(&legacy_topic, MaxSliceLen::const_new(65_536))
        .expect("legacy");

    let expected_fixed: u32 = (WireHeader::SIZE + <Vector3 as ShmMessage>::WIRE_FIXED_SIZE) as u32;
    assert_eq!(typed.max_slice_len().get(), expected_fixed);
    assert_eq!(legacy.max_slice_len().get(), 65_536);
    assert!(
        typed.max_slice_len().get() < legacy.max_slice_len().get(),
        "typed fixed-schema slot ({}) must be smaller than legacy hinted slot ({})",
        typed.max_slice_len(),
        legacy.max_slice_len()
    );
    // The footprint reduction is significant: ~1100x for Vector3 vs the
    // 64KB default hint. Spot-check a lower bound to fail loudly if the
    // dispatch ever regresses.
    assert!(
        legacy.max_slice_len().get() / typed.max_slice_len().get() > 100,
        "expected at least 100x footprint reduction for fixed schema"
    );
}

/// Direct coverage for `create_publisher_typed_with_history`
/// (the `_typed` wrapper above forwards to this with `history_size=0`,
/// so this test exercises the with-history branch independently). Also
/// pins that the `u32` signature on this entry point
/// accepts `Option<u32>` and produces the expected fixed-schema slot.
#[test]
fn typed_with_history_fixed_schema_sizes_correctly() {
    let mgr = TransportManager::get_or_init().expect("init");
    let topic = unique_topic("typed_with_hist_fixed");

    let publisher = mgr
        .create_publisher_typed_with_history::<Vector3>(
            &topic,
            Some(MaxSliceLen::const_new(1 << 20)),
            4,
        )
        .expect("typed_with_history must succeed for fixed schema");

    let expected: u32 = (WireHeader::SIZE + <Vector3 as ShmMessage>::WIRE_FIXED_SIZE) as u32;
    assert_eq!(
        publisher.max_slice_len().get(),
        expected,
        "fixed-schema typed_with_history must use the schema-derived slot \
         (history_size unchanged)",
    );
    // History is iceoryx2-native (the service is created
    // with history_size — no Cerulion `has_history()` accessor). The
    // behavioral late-joiner-delivery contract for native history is pinned
    // in history_test.rs / deliver_history_failure*_test.rs; here we only
    // assert that requesting history does not disturb the fixed-schema slot
    // sizing (the assertion above).
}

/// Variable-schema `create_publisher_typed_with_history` with
/// `max_slice_len: None` returns `MaxSliceLenRequired` — same contract
/// as the non-history wrapper.
#[test]
fn typed_with_history_variable_schema_requires_max_slice_len() {
    let mgr = TransportManager::get_or_init().expect("init");
    let topic = unique_topic("typed_with_hist_var_req");

    match mgr.create_publisher_typed_with_history::<RosString>(&topic, None, 4) {
        Err(TransportError::MaxSliceLenRequired { topic: t }) => {
            assert_eq!(t, topic);
        }
        Err(other) => panic!("expected MaxSliceLenRequired, got {other:?}"),
        Ok(_) => panic!("variable-schema typed_with_history must reject None"),
    }
}
