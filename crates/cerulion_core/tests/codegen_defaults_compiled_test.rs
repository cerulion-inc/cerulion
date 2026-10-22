// SPDX-License-Identifier: AGPL-3.0-only
//! Compiled pins for `.msg` constants and string, array and all-zero
//! defaults, and for a `.srv` service whose request and response keep the
//! ROS 2 names (`Probe_Request`, `Probe_Response`).
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

use cerulion_core::codegen::{parse_rosmsg, parse_rossrv, FrameValueKind, FrameWalker};
use cerulion_core::testing::TestTransport;
use cerulion_core::transport::publisher::CerulionPublisher;
use cerulion_core::wire::{MaxSliceLen, WireHeader};

use codegen_fixture_msgs::codegen_fixture::{
    DefaultsFixedShm, DefaultsFixedSnapshot, DefaultsVar, DefaultsVarSnapshot, Probe_RequestShm,
    Probe_RequestSnapshot, Probe_Response, Probe_ResponseSnapshot, ZeroDefaultsSnapshot,
};

const DEFAULTS_VAR_MSG: &str =
    include_str!("../../test_fixtures/codegen_fixture_msgs/msg/DefaultsVar.msg");
const PROBE_SRV: &str = include_str!("../../test_fixtures/codegen_fixture_msgs/srv/Probe.srv");

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
    // `-0.0` compares equal to `0.0`, so read the sign bit.
    assert_eq!(d.neg.map(f64::to_bits), [NEG_ZERO_F64; 2]);
    assert_eq!(d.nz.to_bits(), NEG_ZERO_F32);

    // The SHM overlay stores `bool` as `u8`.
    let shm = DefaultsFixedShm::default();
    assert_eq!(shm.a, [1, 0, -3]);
    assert_eq!(shm.gains, [0.5, -1.0]);
    assert_eq!(shm.mask, [1u8, 0u8]);
    assert_eq!(shm.k, 7);
    assert_eq!({ shm.neg }.map(f64::to_bits), [NEG_ZERO_F64; 2]);
    assert_eq!({ shm.nz }.to_bits(), NEG_ZERO_F32);
}

/// The IEEE 754 bit patterns of `-0.0`: only the sign bit set.
const NEG_ZERO_F64: u64 = 0x8000_0000_0000_0000;
const NEG_ZERO_F32: u32 = 0x8000_0000;

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
    let frame = publish_default_frame(|publisher| {
        let mut proxy = publisher.loan_proxy::<DefaultsVar>().expect("loan_proxy");
        proxy
            .write_from_snapshot(&DefaultsVarSnapshot::default())
            .expect("write the default snapshot");
    });
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

/// The `.srv` request and response compile under their ROS 2 names and hold
/// the defaults `Probe.srv` declares: `bool data true` in the fixed request,
/// `string message "ready"` in the variable response.
#[test]
fn a_service_request_and_response_compile_with_their_defaults() {
    assert!(Probe_RequestSnapshot::default().data);
    // The SHM overlay stores `bool` as `u8`.
    assert_eq!(Probe_RequestShm::default().data, 1u8);
    let response = Probe_ResponseSnapshot::default();
    assert!(!response.success);
    assert_eq!(response.message, "ready");
}

/// The default response publishes through the generated writer, and the
/// frame decodes with the `FrameWalker` built from the `.srv` response.
#[test]
fn the_default_service_response_decodes_through_the_frame_walker() {
    let frame = publish_default_frame(|publisher| {
        let mut proxy = publisher
            .loan_proxy::<Probe_Response>()
            .expect("loan_proxy");
        proxy
            .write_from_snapshot(&Probe_ResponseSnapshot::default())
            .expect("write the default snapshot");
    });
    let srv = parse_rossrv(PROBE_SRV, "Probe", Some("codegen_fixture"), "Probe.srv")
        .expect("fixture parses");
    let (walker, warnings) = FrameWalker::new(vec![srv.response]);
    assert!(warnings.is_empty(), "{warnings:?}");
    let fv = walker
        .walk("codegen_fixture/Probe_Response", &frame)
        .expect("walk the Probe_Response frame");
    assert_eq!(fv.field("success"), Some(&FrameValueKind::Bool(false)));
    assert_eq!(fv.field("message"), Some(&FrameValueKind::Str("ready")));
}

/// Publish once through `write` on a fresh topic and return the one
/// delivered frame (header then payload).
fn publish_default_frame(write: impl FnOnce(&mut CerulionPublisher)) -> Vec<u8> {
    let id = TOPIC_COUNTER.fetch_add(1, Ordering::Relaxed);
    let topic = format!("test/codegen/defaults/{id}");
    let tt = TestTransport::with_buffer_size(4);
    let mut publisher = tt.publisher(&topic, MaxSliceLen::const_new(64 * 1024), 0);
    let subscriber = tt.subscriber(&topic);
    write(&mut publisher);
    let mut frames: Vec<Vec<u8>> = Vec::new();
    subscriber
        .try_receive(|msg| {
            let mut frame = vec![0u8; WireHeader::SIZE];
            msg.header().write_to_buf(&mut frame);
            frame.extend_from_slice(msg.payload());
            frames.push(frame);
        })
        .expect("drain subscriber");
    assert_eq!(frames.len(), 1, "exactly one frame is delivered");
    frames.pop().expect("one delivered frame")
}
