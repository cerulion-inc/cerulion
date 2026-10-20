// SPDX-License-Identifier: AGPL-3.0-only
//! Acceptance pins for the vendored service types: `std_srvs` and the
//! `rcl_interfaces` parameter services, against the REAL generated bindings.
//!
//! A `.srv` file generates a service marker plus two ordinary messages
//! (`<Name>Request`, `<Name>Response`). These tests check the marker names
//! the right pair, that the parts carry the declared constants and defaults,
//! that a part publishes and decodes like any message (a real loan over an
//! isolated `TestTransport`, then the `FrameWalker` over the embedded
//! registry), and that the parts' snapshots go through serde.

use std::sync::atomic::{AtomicU64, Ordering};

use cerulion_core::codegen::{parse_rosmsg, FrameValueKind, FrameWalker, MessageSchema};
use cerulion_core::message::{ServiceType, ShmMessage};
use cerulion_core::testing::TestTransport;
use cerulion_core::wire::{MaxSliceLen, WireHeader};
use cerulion_core::CerulionSubscriber;

use native_ros2_messages::rcl_interfaces::{
    DescribeParameters, GetParameterTypes, GetParameters, ListParameters, ListParametersRequest,
    ParameterDescriptorSnapshot, ParameterType, SetParameters, SetParametersAtomically,
};
use native_ros2_messages::std_srvs::{
    Empty, EmptyRequest, EmptyResponse, SetBool, SetBoolRequest, SetBoolResponse,
    SetBoolResponseSnapshot, Trigger, TriggerRequest, TriggerResponse,
};

static TOPIC_COUNTER: AtomicU64 = AtomicU64::new(0);

fn hash_of<T: ShmMessage>() -> u64 {
    T::SCHEMA_HASH
}

/// The marker's `NAME`, request hash and response hash.
fn pair<S: ServiceType>() -> (&'static str, u64, u64) {
    (S::NAME, hash_of::<S::Request>(), hash_of::<S::Response>())
}

#[test]
fn each_std_srvs_marker_names_its_own_request_and_response() {
    assert_eq!(
        pair::<Trigger>(),
        (
            "std_srvs/Trigger",
            hash_of::<TriggerRequest>(),
            hash_of::<TriggerResponse>()
        )
    );
    assert_eq!(
        pair::<SetBool>(),
        (
            "std_srvs/SetBool",
            hash_of::<SetBoolRequest>(),
            hash_of::<SetBoolResponse>()
        )
    );
    assert_eq!(
        pair::<Empty>(),
        (
            "std_srvs/Empty",
            hash_of::<EmptyRequest>(),
            hash_of::<EmptyResponse>()
        )
    );
}

#[test]
fn the_rcl_interfaces_parameter_services_are_named_by_their_qualified_names() {
    let names = [
        DescribeParameters::NAME,
        GetParameterTypes::NAME,
        GetParameters::NAME,
        ListParameters::NAME,
        SetParameters::NAME,
        SetParametersAtomically::NAME,
    ];
    assert_eq!(
        names,
        [
            "rcl_interfaces/DescribeParameters",
            "rcl_interfaces/GetParameterTypes",
            "rcl_interfaces/GetParameters",
            "rcl_interfaces/ListParameters",
            "rcl_interfaces/SetParameters",
            "rcl_interfaces/SetParametersAtomically",
        ]
    );
}

/// Two empty parts and an empty message share a layout but not a qualified
/// name, so every one of them has its own wire hash: a request can never be
/// read as a response.
#[test]
fn service_part_hashes_are_pairwise_distinct() {
    let hashes = [
        hash_of::<EmptyRequest>(),
        hash_of::<EmptyResponse>(),
        hash_of::<native_ros2_messages::std_msgs::Empty>(),
        hash_of::<TriggerRequest>(),
        hash_of::<TriggerResponse>(),
        hash_of::<SetBoolRequest>(),
        hash_of::<SetBoolResponse>(),
    ];
    for (i, a) in hashes.iter().enumerate() {
        for b in &hashes[i + 1..] {
            assert_ne!(a, b, "two service parts share a schema hash");
        }
    }
}

/// The wire hash of each `std_srvs` part, written here by hand. A bag records
/// these numbers, so a change to how a `.srv` splits into parts or to how a
/// part is named (for example `Trigger_Request` in place of `TriggerRequest`)
/// would stop old recordings matching on replay. Such a change must fail here
/// first. A failure is a wire-compatibility break: do not copy the new value in.
#[test]
fn service_part_hashes_are_pinned() {
    let pinned: [(&str, u64, u64); 6] = [
        (
            "std_srvs/EmptyRequest",
            hash_of::<EmptyRequest>(),
            0x78cb_d57f_cdcd_38c7,
        ),
        (
            "std_srvs/EmptyResponse",
            hash_of::<EmptyResponse>(),
            0xde13_77f6_19f3_0b7e,
        ),
        (
            "std_srvs/TriggerRequest",
            hash_of::<TriggerRequest>(),
            0xd3f4_21a1_f52e_85fa,
        ),
        (
            "std_srvs/TriggerResponse",
            hash_of::<TriggerResponse>(),
            0x7f18_953d_08e1_9c8b,
        ),
        (
            "std_srvs/SetBoolRequest",
            hash_of::<SetBoolRequest>(),
            0x6772_e6b3_1a6b_064b,
        ),
        (
            "std_srvs/SetBoolResponse",
            hash_of::<SetBoolResponse>(),
            0xc42b_1bc3_5b20_eb6f,
        ),
    ];
    let drifted: Vec<String> = pinned
        .iter()
        .filter(|(_, ours, pin)| ours != pin)
        .map(|(name, ours, pin)| format!("{name}: generated 0x{ours:016x}, pinned 0x{pin:016x}"))
        .collect();
    assert!(
        drifted.is_empty(),
        "service part hashes changed:\n{}",
        drifted.join("\n")
    );
}

/// `ParameterType` is constants only; `ListParameters`' request declares
/// `DEPTH_RECURSIVE`. Values from the upstream files.
#[test]
fn parameter_constants_are_on_the_generated_types() {
    let string_type: u8 = ParameterType::PARAMETER_STRING;
    assert_eq!(string_type, 4);
    assert_eq!(ParameterType::PARAMETER_NOT_SET, 0);
    assert_eq!(ParameterType::PARAMETER_STRING_ARRAY, 9);
    let depth: u64 = ListParametersRequest::DEPTH_RECURSIVE;
    assert_eq!(depth, 0);
}

/// The snapshot default of `ParameterDescriptor` is all zero and empty. Its
/// declared `false` defaults equal `bool::default()`, so this cannot tell a
/// parsed default literal from a plain derive; `generated_bindings_test`
/// pins default-literal parsing with a non-zero default.
#[test]
fn a_default_parameter_descriptor_is_zero_and_empty() {
    let d = ParameterDescriptorSnapshot::default();
    assert!(!d.read_only);
    assert!(!d.dynamic_typing);
    assert_eq!(d.name, "");
    assert!(d.floating_point_range.is_empty());
}

fn builtin_walker() -> FrameWalker {
    let mut schemas: Vec<MessageSchema> = Vec::new();
    for (pkg, name, text) in native_ros2_messages::BUILTIN_MSGS {
        schemas.push(
            parse_rosmsg(text, name, Some(pkg))
                .unwrap_or_else(|e| panic!("built-in {pkg}/{name} must parse: {e}")),
        );
    }
    let (walker, warnings) = FrameWalker::new(schemas);
    assert!(warnings.is_empty(), "built-ins must resolve: {warnings:?}");
    walker
}

fn last_frame(sub: &CerulionSubscriber) -> Vec<u8> {
    let mut frames: Vec<Vec<u8>> = Vec::new();
    sub.try_receive(|msg| {
        let mut frame = vec![0u8; WireHeader::SIZE];
        msg.header().write_to_buf(&mut frame);
        frame.extend_from_slice(msg.payload());
        frames.push(frame);
    })
    .expect("drain subscriber");
    frames.pop().expect("one delivered frame")
}

/// A `Trigger` response publishes through the real generated writer and the
/// frame decodes, by its own schema hash, with the `FrameWalker` over the
/// embedded registry: the service part is in the registry under its
/// qualified name.
#[test]
fn a_trigger_response_frame_decodes_through_the_frame_walker() {
    let id = TOPIC_COUNTER.fetch_add(1, Ordering::Relaxed);
    let topic = format!("test/srv/trigger_response/{id}");
    let tt = TestTransport::with_buffer_size(4);
    let mut publisher = tt.publisher(&topic, MaxSliceLen::const_new(64 * 1024), 0);
    let subscriber = tt.subscriber(&topic);
    {
        let mut proxy = publisher
            .loan_proxy::<TriggerResponse>()
            .expect("loan_proxy");
        proxy.success = 1;
        proxy.set_message("calibrated").expect("set_message");
    }
    let frame = last_frame(&subscriber);
    let fv = builtin_walker()
        .walk("std_srvs/TriggerResponse", &frame)
        .expect("walk the TriggerResponse frame");
    assert_eq!(fv.field("success"), Some(&FrameValueKind::Bool(true)));
    assert_eq!(
        fv.field("message"),
        Some(&FrameValueKind::Str("calibrated"))
    );
}

/// The response snapshot serializes and deserializes back to the same
/// hand-built value.
#[test]
fn a_set_bool_response_snapshot_round_trips_through_serde() {
    let value = SetBoolResponseSnapshot {
        success: true,
        message: "motor enabled".to_string(),
    };
    let json = serde_json::to_string(&value).expect("serialize");
    assert_eq!(json, r#"{"success":true,"message":"motor enabled"}"#);
    let back: SetBoolResponseSnapshot = serde_json::from_str(&json).expect("deserialize");
    assert_eq!(back, value);
}
