use super::*;
// Test-only isolated transport; node code uses the public prelude.
use cerulion_core::testing::TestTransport;

// Hand-encoded Header: zero timestamp and an empty frame_id at offset 16.
const HEADER: [u8; 16] = [0, 0, 0, 0, 0, 0, 0, 0, 16, 0, 0, 0, 0, 0, 0, 0];

#[test]
fn controller_publishes_stop_or_cruise_for_close_clear_empty_and_nan_scans() {
    let transport = TestTransport::with_buffer_size(4);
    let mut input = transport.publisher("scan", MaxSliceLen::const_new(4096), 0);
    let input_sub = transport.subscriber("scan");
    let output = transport.publisher("velocity", MaxSliceLen::const_new(4096), 0);
    let mut output_sub = transport.subscriber("velocity");
    let mut entry = SafetyControllerNodeEntry::new();
    entry
        .init(NodeContext::for_tests(
            [("linear_velocity".into(), AnyPublisher::Ipc(output))]
                .into_iter()
                .collect(),
            [("scan".into(), AnySubscriber::Ipc(input_sub))]
                .into_iter()
                .collect(),
        ))
        .unwrap();
    // Literal velocity oracles pin the safety behavior at the published port.
    let cases: &[(&[f32], f64)] = &[
        (&[5.0], 0.3),
        (&[0.3, 5.0], 0.0),
        (&[5.0, 0.3], 0.0),
        (&[], 0.0),
        (&[5.0, f32::NAN], 0.0),
        (&[0.5, 5.0], 0.3),
        (&[5.0], 0.3),
    ];
    for &(ranges, expected) in cases {
        {
            let mut scan = input.loan_proxy::<LaserScan>().unwrap();
            scan.set_header_bytes(&HEADER).unwrap();
            scan.loan_ranges(ranges.len())
                .unwrap()
                .copy_from_slice(ranges);
            scan.loan_intensities(0).unwrap();
        }
        entry.tick().unwrap();
        assert_eq!(
            output_sub
                .try_view::<Vector3, _>(|velocity| (velocity.x, velocity.y, velocity.z))
                .unwrap(),
            Some((expected, 0.0, 0.0)),
            "ranges: {ranges:?}"
        );
    }
    entry.shutdown().unwrap();
}

// The built-in corpus ships the standard ROS 2 service types as message pairs,
// so a workspace names `std_srvs/SetBool` or `std_srvs/Trigger` without
// defining them. A node cannot serve or call a service yet; this pins that the
// types resolve from a workspace, that each marker names its own pair, and
// that the parts build with their declared fields.
#[test]
fn std_srvs_service_types_resolve_from_the_workspace() {
    // `ServiceType` is not in the prelude: it is experimental until nodes can
    // serve and call services.
    use cerulion_core::message::{ServiceType, ShmMessage};
    use native_ros2_messages::std_srvs::{
        SetBool, SetBoolRequest, SetBoolRequestSnapshot, Trigger, TriggerResponse,
        TriggerResponseSnapshot,
    };

    assert_eq!(<Trigger as ServiceType>::NAME, "std_srvs/Trigger");
    assert_eq!(<SetBool as ServiceType>::NAME, "std_srvs/SetBool");
    assert_eq!(
        <<SetBool as ServiceType>::Request as ShmMessage>::SCHEMA_HASH,
        SetBoolRequest::SCHEMA_HASH
    );
    assert_eq!(
        <<Trigger as ServiceType>::Response as ShmMessage>::SCHEMA_HASH,
        TriggerResponse::SCHEMA_HASH
    );
    // Construction is the check: a missing or renamed field fails to compile.
    let _enable = SetBoolRequestSnapshot { data: true };
    let _done = TriggerResponseSnapshot {
        success: true,
        message: "stopped".to_string(),
    };
}
