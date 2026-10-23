// SPDX-License-Identifier: AGPL-3.0-only
//! Acceptance pins for the vendored service types: `std_srvs` and the
//! `rcl_interfaces` parameter services, against the REAL generated bindings.
//!
//! A `.srv` file generates a service marker plus two ordinary messages
//! (`<Name>_Request`, `<Name>_Response`, the names ROS 2 gives them). These
//! tests check the marker names
//! the right pair, that the parts carry the declared constants and defaults,
//! that a part publishes and decodes like any message (a real loan over an
//! isolated `TestTransport`, then the `FrameWalker` over the embedded
//! registry), that the parts' snapshots go through serde, and that each
//! marker's service hash is pinned and follows its parts.
//!
//! The markers need the `unstable-services` feature, which the crate's
//! dev-dependency on `cerulion_core` turns on for these tests.

use std::sync::atomic::{AtomicU64, Ordering};

use cerulion_core::codegen::{
    parse_rosmsg, parse_rossrv, service_part_replaced, FrameValueKind, FrameWalker, MessageSchema,
};
use cerulion_core::message::{ServiceType, ShmMessage};
use cerulion_core::testing::TestTransport;
use cerulion_core::wire::{MaxSliceLen, WireHeader};
use cerulion_core::CerulionSubscriber;

use native_ros2_messages::rcl_interfaces::{
    DescribeParameters, GetParameterTypes, GetParameters, ListParameters, ListParameters_Request,
    ParameterDescriptorSnapshot, ParameterType, SetParameters, SetParametersAtomically,
};
use native_ros2_messages::std_srvs::{
    Empty, Empty_Request, Empty_Response, SetBool, SetBool_Request, SetBool_Response,
    SetBool_ResponseSnapshot, Trigger, Trigger_Request, Trigger_Response,
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
            hash_of::<Trigger_Request>(),
            hash_of::<Trigger_Response>()
        )
    );
    assert_eq!(
        pair::<SetBool>(),
        (
            "std_srvs/SetBool",
            hash_of::<SetBool_Request>(),
            hash_of::<SetBool_Response>()
        )
    );
    assert_eq!(
        pair::<Empty>(),
        (
            "std_srvs/Empty",
            hash_of::<Empty_Request>(),
            hash_of::<Empty_Response>()
        )
    );
}

/// The service hash of `std_srvs/Trigger` and `std_srvs/SetBool`, written
/// here by hand: FNV-1a 64 over the name length (`u64` little-endian), the
/// name bytes, then the request and the response schema hashes (each `u64`
/// little-endian), computed outside this crate from the part hashes that
/// `service_part_hashes_are_pinned` pins. A failure is a service identity
/// break: do not copy the new value in.
#[test]
fn service_hashes_are_pinned() {
    assert_eq!(Trigger::SERVICE_HASH, 0xf307_a411_94e5_db20);
    assert_eq!(SetBool::SERVICE_HASH, 0xa1ba_4277_988b_13b5);
}

/// A service with Trigger's name but SetBool's request.
struct OtherRequest;
impl ServiceType for OtherRequest {
    const NAME: &'static str = "std_srvs/Trigger";
    type Request = SetBool_Request;
    type Response = Trigger_Response;
}

/// A service with Trigger's name but SetBool's response.
struct OtherResponse;
impl ServiceType for OtherResponse {
    const NAME: &'static str = "std_srvs/Trigger";
    type Request = Trigger_Request;
    type Response = SetBool_Response;
}

/// A service with Trigger's parts in swapped order.
struct Swapped;
impl ServiceType for Swapped {
    const NAME: &'static str = "std_srvs/Trigger";
    type Request = Trigger_Response;
    type Response = Trigger_Request;
}

/// Same name, other request or other response: the service hash changes,
/// so a peer whose parts differ never takes the service for the same one.
#[test]
fn the_service_hash_changes_with_either_part_schema() {
    let base = Trigger::SERVICE_HASH;
    assert_ne!(OtherRequest::SERVICE_HASH, base, "request schema differs");
    assert_ne!(OtherResponse::SERVICE_HASH, base, "response schema differs");
    assert_ne!(Swapped::SERVICE_HASH, base, "parts swapped");
}

/// Every vendored service has its own service hash.
#[test]
fn service_hashes_are_pairwise_distinct() {
    let hashes = [
        Empty::SERVICE_HASH,
        SetBool::SERVICE_HASH,
        Trigger::SERVICE_HASH,
        DescribeParameters::SERVICE_HASH,
        GetParameterTypes::SERVICE_HASH,
        GetParameters::SERVICE_HASH,
        ListParameters::SERVICE_HASH,
        SetParameters::SERVICE_HASH,
        SetParametersAtomically::SERVICE_HASH,
    ];
    for (i, a) in hashes.iter().enumerate() {
        for b in &hashes[i + 1..] {
            assert_ne!(a, b, "two services share a service hash");
        }
    }
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
        hash_of::<Empty_Request>(),
        hash_of::<Empty_Response>(),
        hash_of::<native_ros2_messages::std_msgs::Empty>(),
        hash_of::<Trigger_Request>(),
        hash_of::<Trigger_Response>(),
        hash_of::<SetBool_Request>(),
        hash_of::<SetBool_Response>(),
    ];
    for (i, a) in hashes.iter().enumerate() {
        for b in &hashes[i + 1..] {
            assert_ne!(a, b, "two service parts share a schema hash");
        }
    }
}

/// The wire hash of each `std_srvs` part, written here by hand. A bag records
/// these numbers, so a change to how a `.srv` splits into parts or to how a
/// part is named (ROS 2 names them `Trigger_Request` and `Trigger_Response`)
/// would stop old recordings matching on replay. Such a change must fail here
/// first. A failure is a wire-compatibility break: do not copy the new value in.
#[test]
fn service_part_hashes_are_pinned() {
    let pinned: [(&str, u64, u64); 6] = [
        (
            "std_srvs/Empty_Request",
            hash_of::<Empty_Request>(),
            0x3ee8_ab5e_1858_0aa3,
        ),
        (
            "std_srvs/Empty_Response",
            hash_of::<Empty_Response>(),
            0xc145_3c0b_3907_74a6,
        ),
        (
            "std_srvs/Trigger_Request",
            hash_of::<Trigger_Request>(),
            0x8c6c_bf82_ecbc_302c,
        ),
        (
            "std_srvs/Trigger_Response",
            hash_of::<Trigger_Response>(),
            0x4816_15eb_ca66_da0d,
        ),
        (
            "std_srvs/SetBool_Request",
            hash_of::<SetBool_Request>(),
            0x532a_9017_f928_93c9,
        ),
        (
            "std_srvs/SetBool_Response",
            hash_of::<SetBool_Response>(),
            0x2019_f962_535a_2ee1,
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

/// The schema `rmw_cerulion` builds for a service part on a ROS 2 peer: the
/// part's own lines, named `<pkg>/<Name>_Request` or `<pkg>/<Name>_Response`
/// from the rosidl introspection (namespace `std_srvs__srv` gives package
/// `std_srvs`). `body` is one side of the vendored `.srv`, split at its `---`.
fn rmw_peer_hash(body: &str, rosidl_message_name: &str) -> u64 {
    parse_rosmsg(body, rosidl_message_name, Some("std_srvs"))
        .unwrap_or_else(|e| panic!("{rosidl_message_name} must parse: {e}"))
        .schema_hash()
}

/// Split a vendored `.srv` at its one `---` line into request and response.
fn srv_sides(text: &str) -> (String, String) {
    let lines: Vec<&str> = text.lines().collect();
    let sep = lines
        .iter()
        .position(|l| l.trim() == "---")
        .expect("a .srv has a --- line");
    let join = |part: &[&str]| part.iter().map(|l| format!("{l}\n")).collect::<String>();
    (join(&lines[..sep]), join(&lines[sep + 1..]))
}

/// Each native `std_srvs` part hashes like the part a ROS 2 peer on
/// `rmw_cerulion` announces for the same service, so the two can match on the
/// wire.
#[test]
fn each_std_srvs_part_hashes_like_the_rmw_cerulion_peer_part() {
    let cases: [(&str, &str, u64, u64); 3] = [
        (
            "Empty",
            include_str!("../srv/std_srvs/Empty.srv"),
            hash_of::<Empty_Request>(),
            hash_of::<Empty_Response>(),
        ),
        (
            "SetBool",
            include_str!("../srv/std_srvs/SetBool.srv"),
            hash_of::<SetBool_Request>(),
            hash_of::<SetBool_Response>(),
        ),
        (
            "Trigger",
            include_str!("../srv/std_srvs/Trigger.srv"),
            hash_of::<Trigger_Request>(),
            hash_of::<Trigger_Response>(),
        ),
    ];
    for (name, text, native_request, native_response) in cases {
        let (request, response) = srv_sides(text);
        assert_eq!(
            native_request,
            rmw_peer_hash(&request, &format!("{name}_Request")),
            "std_srvs/{name}_Request"
        );
        assert_eq!(
            native_response,
            rmw_peer_hash(&response, &format!("{name}_Response")),
            "std_srvs/{name}_Response"
        );
    }
}

/// `ParameterType` is constants only; `ListParameters`' request declares
/// `DEPTH_RECURSIVE`. Values from the upstream files.
#[test]
fn parameter_constants_are_on_the_generated_types() {
    let string_type: u8 = ParameterType::PARAMETER_STRING;
    assert_eq!(string_type, 4);
    assert_eq!(ParameterType::PARAMETER_NOT_SET, 0);
    assert_eq!(ParameterType::PARAMETER_STRING_ARRAY, 9);
    let depth: u64 = ListParameters_Request::DEPTH_RECURSIVE;
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
            .loan_proxy::<Trigger_Response>()
            .expect("loan_proxy");
        proxy.success = 1;
        proxy.set_message("calibrated").expect("set_message");
    }
    let frame = last_frame(&subscriber);
    let fv = builtin_walker()
        .walk_by_hash(&frame)
        .expect("walk the Trigger_Response frame by its header hash");
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
    let value = SetBool_ResponseSnapshot {
        success: true,
        message: "motor enabled".to_string(),
    };
    let json = serde_json::to_string(&value).expect("serialize");
    assert_eq!(json, r#"{"success":true,"message":"motor enabled"}"#);
    let back: SetBool_ResponseSnapshot = serde_json::from_str(&json).expect("deserialize");
    assert_eq!(back, value);
}

/// Every vendored `.srv`, as `(package, name, text)`.
const VENDORED_SRVS: &[(&str, &str, &str)] = &[
    (
        "rcl_interfaces",
        "DescribeParameters",
        include_str!("../srv/rcl_interfaces/DescribeParameters.srv"),
    ),
    (
        "rcl_interfaces",
        "GetParameterTypes",
        include_str!("../srv/rcl_interfaces/GetParameterTypes.srv"),
    ),
    (
        "rcl_interfaces",
        "GetParameters",
        include_str!("../srv/rcl_interfaces/GetParameters.srv"),
    ),
    (
        "rcl_interfaces",
        "ListParameters",
        include_str!("../srv/rcl_interfaces/ListParameters.srv"),
    ),
    (
        "rcl_interfaces",
        "SetParameters",
        include_str!("../srv/rcl_interfaces/SetParameters.srv"),
    ),
    (
        "rcl_interfaces",
        "SetParametersAtomically",
        include_str!("../srv/rcl_interfaces/SetParametersAtomically.srv"),
    ),
    (
        "std_srvs",
        "Empty",
        include_str!("../srv/std_srvs/Empty.srv"),
    ),
    (
        "std_srvs",
        "SetBool",
        include_str!("../srv/std_srvs/SetBool.srv"),
    ),
    (
        "std_srvs",
        "Trigger",
        include_str!("../srv/std_srvs/Trigger.srv"),
    ),
];

/// The services whose marker a workspace override of `pkg/name` drops,
/// over the real corpus: the check `build.rs` runs before it emits markers.
fn services_dropped_by(pkg: &str, name: &str, override_text: &str) -> Vec<String> {
    let vendored: Vec<MessageSchema> = native_ros2_messages::BUILTIN_MSGS
        .iter()
        .map(|&(p, n, text)| parse_rosmsg(text, n, Some(p)).expect("vendored text parses"))
        .collect();
    let held: Vec<MessageSchema> = vendored
        .iter()
        .map(|s| {
            if s.package.as_deref() == Some(pkg) && s.name == name {
                parse_rosmsg(override_text, name, Some(pkg)).expect("override parses")
            } else {
                s.clone()
            }
        })
        .collect();
    let find = |set: &[MessageSchema], p: &str, n: &str| -> Option<usize> {
        set.iter()
            .position(|s| s.package.as_deref() == Some(p) && s.name == n)
    };
    VENDORED_SRVS
        .iter()
        .filter_map(|&(p, n, text)| {
            let srv = parse_rossrv(text, n, Some(p), &format!("{p}/{n}.srv")).expect("srv parses");
            service_part_replaced(
                &srv,
                |p, n| find(&vendored, p, n).map(|i| &vendored[i]),
                |p, n| find(&held, p, n).map(|i| &held[i]),
            )
            .map(|_| format!("{p}/{n}"))
        })
        .collect()
}

/// A workspace that redefines `rcl_interfaces/ParameterValue` changes the
/// wire type of every parameter service that carries a value, two levels
/// deep for `SetParameters` (request `Parameter[]`, each `Parameter` a
/// `ParameterValue`), although no part's own field list changes. Those
/// markers are left out; the services that never carry a value keep theirs.
#[test]
fn a_parameter_value_override_drops_every_marker_that_embeds_it() {
    let dropped = services_dropped_by(
        "rcl_interfaces",
        "ParameterValue",
        "uint8 type\nint64 integer_value\n",
    );
    assert_eq!(
        dropped,
        [
            "rcl_interfaces/GetParameters",
            "rcl_interfaces/SetParameters",
            "rcl_interfaces/SetParametersAtomically",
        ]
    );
}

/// An override that keeps the vendored fields (only comments differ) drops
/// no marker.
#[test]
fn an_override_with_the_vendored_fields_drops_no_marker() {
    let (_, _, text) = native_ros2_messages::BUILTIN_MSGS
        .iter()
        .find(|&&(p, n, _)| p == "rcl_interfaces" && n == "ParameterValue")
        .expect("ParameterValue is vendored");
    let commented = format!("# a workspace copy\n{text}");
    assert!(services_dropped_by("rcl_interfaces", "ParameterValue", &commented).is_empty());
}
