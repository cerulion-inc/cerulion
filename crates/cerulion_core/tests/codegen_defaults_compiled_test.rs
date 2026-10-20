// SPDX-License-Identifier: AGPL-3.0-only
//! Compiled pins for `.msg` constants and string, array and all-zero
//! defaults.
//!
//! No vendored message carries a string or array default, so these paths
//! are generated here from the `codegen_fixture_msgs` fixture crate (its
//! `build.rs` runs the real parse, resolve and generate steps). The crate
//! building at all, under the workspace clippy gate, proves the generated
//! code compiles and lints clean.
//! The tests below check the values the generated `Default` impls hold
//! against hand-written oracles, and that the default `string[]` payload
//! publishes and decodes with the `FrameWalker` as the strings it names.

use std::sync::atomic::{AtomicU64, Ordering};

use cerulion_core::codegen::{parse_rosmsg, FrameValueKind, FrameWalker};
use cerulion_core::testing::TestTransport;
use cerulion_core::wire::{MaxSliceLen, WireHeader};

use codegen_fixture_msgs::codegen_fixture::{
    DefaultsFixedShm, DefaultsFixedSnapshot, DefaultsVar, DefaultsVarSnapshot, ZeroDefaultsSnapshot,
};

const DEFAULTS_VAR_MSG: &str =
    include_str!("../../test_fixtures/codegen_fixture_msgs/msg/DefaultsVar.msg");

static TOPIC_COUNTER: AtomicU64 = AtomicU64::new(0);

#[test]
fn constants_are_associated_consts_on_the_marker() {
    assert_eq!(DefaultsVar::MODE_A, 1u8);
    assert_eq!(DefaultsVar::NAME, "fixture");
}

#[test]
fn a_variable_snapshot_default_holds_every_declared_default() {
    let d = DefaultsVarSnapshot::default();
    assert_eq!(d.s, "hi");
    assert_eq!(d.bounded, "bound");
    // `string[] names ["ab", ""]`: u32 count 2, then "ab" (len 2) and ""
    // (len 0), all little-endian.
    assert_eq!(
        d.names,
        vec![2, 0, 0, 0, 2, 0, 0, 0, b'a', b'b', 0, 0, 0, 0]
    );
    assert_eq!(d.ids, vec![1, -2]);
    assert_eq!(d.gains, [0.5, -1.0]);
    assert_eq!(d.mask, [true, false]);
    // A field without a default keeps the zero default.
    assert_eq!(d.plain, 0);
}

#[test]
fn a_fixed_snapshot_and_shm_default_hold_every_declared_default() {
    let d = DefaultsFixedSnapshot::default();
    assert_eq!(d.a, [1, 0, -3]);
    assert_eq!(d.gains, [0.5, -1.0]);
    assert_eq!(d.mask, [true, false]);
    assert_eq!(d.k, 7);

    // The SHM overlay stores `bool` as `u8`.
    let shm = DefaultsFixedShm::default();
    assert_eq!(shm.a, [1, 0, -3]);
    assert_eq!(shm.gains, [0.5, -1.0]);
    assert_eq!(shm.mask, [1u8, 0u8]);
    assert_eq!(shm.k, 7);
}

#[test]
fn all_zero_defaults_equal_the_derived_default() {
    let d = ZeroDefaultsSnapshot::default();
    assert_eq!(d.a, [0, 0, 0]);
    assert_eq!(d.gains, [0.0, 0.0]);
    assert_eq!(d.mask, [false, false]);
    assert_eq!(d.s, "");
    assert!(d.ids.is_empty());
}

#[test]
fn two_default_snapshots_are_identical() {
    assert_eq!(
        DefaultsVarSnapshot::default(),
        DefaultsVarSnapshot::default()
    );
    assert_eq!(
        DefaultsFixedSnapshot::default(),
        DefaultsFixedSnapshot::default()
    );
}

/// The default snapshot publishes through the generated writer, and the
/// frame decodes with the `FrameWalker`: the canonical `string[]` default
/// bytes read back as the two strings the `.msg` names.
#[test]
fn the_default_string_array_payload_decodes_through_the_frame_walker() {
    let id = TOPIC_COUNTER.fetch_add(1, Ordering::Relaxed);
    let topic = format!("test/codegen/defaults_var/{id}");
    let tt = TestTransport::with_buffer_size(4);
    let mut publisher = tt.publisher(&topic, MaxSliceLen::const_new(64 * 1024), 0);
    let subscriber = tt.subscriber(&topic);
    {
        let mut proxy = publisher.loan_proxy::<DefaultsVar>().expect("loan_proxy");
        proxy
            .write_from_snapshot(&DefaultsVarSnapshot::default())
            .expect("write the default snapshot");
    }
    let mut frames: Vec<Vec<u8>> = Vec::new();
    subscriber
        .try_receive(|msg| {
            let mut frame = vec![0u8; WireHeader::SIZE];
            msg.header().write_to_buf(&mut frame);
            frame.extend_from_slice(msg.payload());
            frames.push(frame);
        })
        .expect("drain subscriber");
    let frame = frames.pop().expect("one delivered frame");

    let schema = parse_rosmsg(DEFAULTS_VAR_MSG, "DefaultsVar", Some("codegen_fixture"))
        .expect("fixture parses");
    let (walker, warnings) = FrameWalker::new(vec![schema]);
    assert!(warnings.is_empty(), "{warnings:?}");
    let fv = walker
        .walk("codegen_fixture/DefaultsVar", &frame)
        .expect("walk the DefaultsVar frame");
    match fv.field("names") {
        Some(FrameValueKind::NestedArray { elements, .. }) => assert_eq!(
            elements.as_slice(),
            &[FrameValueKind::Str("ab"), FrameValueKind::Str("")]
        ),
        other => panic!("expected a NestedArray for names, got {other:?}"),
    }
    assert_eq!(fv.field("s"), Some(&FrameValueKind::Str("hi")));
    assert_eq!(fv.field("bounded"), Some(&FrameValueKind::Str("bound")));
    assert_eq!(fv.field("plain"), Some(&FrameValueKind::U16(0)));
}
