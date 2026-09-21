// SPDX-License-Identifier: AGPL-3.0-only
//! The raw-generic ingress ROUTE half over REAL iceoryx2 —
//! `RawIngressRoute` (dynamic `create_ingress_publisher` + codec-decoded
//! `publish_raw`), driven with hand-built DDS payloads (no DDS stack; the
//! DDS-side adapter half is unit-pinned in `generic::raw`, and the live
//! CycloneDDS pump wiring is pinned by `cyclone_e2e_test`).
//!
//! Pinned (every assert against a HAND oracle, never a self-compare):
//! - A type outside the hand registry (`geometry_msgs/Vector3`, fixed) and
//!   a variable one (`std_msgs/String`) flow DDS-payload → wire frame →
//!   local subscriber with ZERO per-type code; the captured frames are
//!   byte-equal to hand-built expected frames whose `schema_hash` is the
//!   GENERATED type's `SCHEMA_HASH` (the cross-stack pin: codec hash ==
//!   codegen hash on the delivered wire).
//! - Sequence discipline: decode failures drop the sample WITHOUT
//!   burning a sequence — the delivered stream stays gap-free (0,1,2 …).
//! - `publish_cdr_body` (the pump-facing entry — encapsulation already
//!   stripped, the `raw::RawSample` shape) is byte-identical to
//!   `publish_dds_payload` (the full-payload convenience).
//! - An unknown `ros_type` is refused at `open` BEFORE any publisher/service
//!   exists ("no schema", never "no codec").
//! - Two routes on one transport are independent (delivery + counters).
//! - Determinism: the same script on two isolated transports is
//!   byte-identical, anchored to the hand oracle (Principle #7).
//!
//! Isolated per-test SHM root (`init_for_test`) — parallel-safe, no
//! `#[serial]`.

use std::sync::Arc;

use cerulion_core::clock::VirtualClock;
use cerulion_core::codegen::CdrEndianness;
use cerulion_core::message::ShmMessage;
use cerulion_core::transport::{TransportConfig, TransportManager};
use cerulion_core::wire::{MaxSliceLen, WireHeader};
use cerulion_core::TransportError;

use dds_bridge::generic::{bridge_codec, RawIngressRoute, RawRouteError};

// ---------------------------------------------------------------------------
// Rig + hand-oracle builders
// ---------------------------------------------------------------------------

fn build_mgr(tag: &str) -> Arc<TransportManager> {
    let clock = Arc::new(VirtualClock::new());
    TransportManager::init_for_test(
        TransportConfig {
            node_name: format!("raw_route_{tag}"),
            clock,
            subscriber_buffer_size: 16,
            network: None,
        },
        cerulion_core::testing::iceoryx_test_config(),
    )
    .expect("init isolated transport")
}

const CDR_LE_ENCAPSULATION: [u8; 4] = [0x00, 0x01, 0x00, 0x00];

/// Hand-build the full DDS payload for a `geometry_msgs/Vector3` (CDR_LE):
/// encapsulation header + three LE f64s (CDR body == the 24-byte fixed
/// section byte-for-byte, which is what makes the frame oracle fully
/// hand-derivable).
fn vector3_dds_payload(x: f64, y: f64, z: f64) -> Vec<u8> {
    let mut p = CDR_LE_ENCAPSULATION.to_vec();
    p.extend_from_slice(&x.to_le_bytes());
    p.extend_from_slice(&y.to_le_bytes());
    p.extend_from_slice(&z.to_le_bytes());
    p
}

/// Hand-build the EXPECTED Cerulion wire frame for that Vector3: 32-byte
/// header (schema_hash = the GENERATED type's `SCHEMA_HASH`) + the same 24
/// fixed-section bytes. Table offset is FRAME-relative (header + fixed = 56)
/// — the native-publisher convention, count 0.
fn vector3_expected_frame(x: f64, y: f64, z: f64, sequence: u32, timestamp_ns: u64) -> Vec<u8> {
    let header = WireHeader {
        schema_hash: <native_ros2_messages::geometry_msgs::Vector3 as ShmMessage>::SCHEMA_HASH,
        total_size: (WireHeader::SIZE + 24) as u32,
        offset_table_offset: (WireHeader::SIZE + 24) as u32,
        offset_table_count: 0,
        sequence,
        timestamp_ns,
    };
    let mut frame = vec![0u8; WireHeader::SIZE];
    header.write_to_buf(&mut frame);
    frame.extend_from_slice(&x.to_le_bytes());
    frame.extend_from_slice(&y.to_le_bytes());
    frame.extend_from_slice(&z.to_le_bytes());
    frame
}

/// Hand-build the full DDS payload for a `std_msgs/String` (CDR_LE):
/// encapsulation + `u32(len+1) | utf8 | NUL`.
fn string_dds_payload(content: &str) -> Vec<u8> {
    let mut p = CDR_LE_ENCAPSULATION.to_vec();
    p.extend_from_slice(&(content.len() as u32 + 1).to_le_bytes());
    p.extend_from_slice(content.as_bytes());
    p.push(0);
    p
}

/// Hand-build the EXPECTED Cerulion wire frame for that String: fixed size 0,
/// ONE offset-table entry `(offset=8, len)` then the UTF-8 content (no NUL).
/// The header's table offset is FRAME-relative: String's fixed section is
/// empty, so the table starts right after the 32-byte header.
fn string_expected_frame(content: &str, sequence: u32, timestamp_ns: u64) -> Vec<u8> {
    let header = WireHeader {
        schema_hash: <native_ros2_messages::std_msgs::String as ShmMessage>::SCHEMA_HASH,
        total_size: (WireHeader::SIZE + 8 + content.len()) as u32,
        offset_table_offset: WireHeader::SIZE as u32,
        offset_table_count: 1,
        sequence,
        timestamp_ns,
    };
    let mut frame = vec![0u8; WireHeader::SIZE];
    header.write_to_buf(&mut frame);
    frame.extend_from_slice(&8u32.to_le_bytes()); // entry offset
    frame.extend_from_slice(&(content.len() as u32).to_le_bytes()); // entry len
    frame.extend_from_slice(content.as_bytes());
    frame
}

/// Drain every frame currently queued on `sub`, rebuilt as full wire frames
/// (header re-serialized via `write_to_buf` — `read_from_buf`'s total
/// inverse — + the payload slice; the `network_ingress_test` pattern).
fn drain_frames(sub: &cerulion_core::CerulionSubscriber) -> Vec<Vec<u8>> {
    let mut frames = Vec::new();
    sub.try_receive(|msg| {
        let mut frame = vec![0u8; WireHeader::SIZE];
        msg.header().write_to_buf(&mut frame);
        frame.extend_from_slice(msg.payload());
        frames.push(frame);
    })
    .expect("try_receive");
    frames
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[test]
fn test_vector3_flows_end_to_end_with_zero_per_type_code() {
    let mgr = build_mgr("v3");
    let codec = bridge_codec();
    let topic = "/rawroute/v3/flow";

    let mut route = RawIngressRoute::open(
        &mgr,
        &codec,
        "geometry_msgs/Vector3",
        topic,
        MaxSliceLen::const_new(4096),
    )
    .expect("open route for a schema-known type");
    assert_eq!(route.ros_type(), "geometry_msgs/Vector3");
    assert_eq!(route.cerulion_topic(), topic);

    let sub = mgr.create_subscriber(topic).expect("observer subscriber");

    let script = [(1.0, 2.0, 3.0), (4.0, 5.0, 6.0), (-7.5, 0.0, 9.25)];
    let stamps = [100u64, 200, 300];
    for (i, &(x, y, z)) in script.iter().enumerate() {
        let recipients = route
            .publish_dds_payload(&codec, &vector3_dds_payload(x, y, z), stamps[i])
            .expect("publish");
        assert!(recipients >= 1, "the attached subscriber must be served");
    }

    let frames = drain_frames(&sub);
    assert_eq!(frames.len(), 3, "all three frames delivered");
    for (i, &(x, y, z)) in script.iter().enumerate() {
        assert_eq!(
            frames[i],
            vector3_expected_frame(x, y, z, i as u32, stamps[i]),
            "frame {i} must byte-match the hand oracle"
        );
    }
    assert_eq!(route.published_total(), 3);
    assert_eq!(route.decode_failures_total(), 0);
    assert_eq!(route.next_sequence(), 3);
}

/// Opening a raw route pushes its dynamic-egress registration onto
/// the `__cerulion/gateway_topics` control channel (canonical topic + the
/// codec's schema hash) so the separate gateway process can announce + serve
/// remote demand for it — the wiring that makes a `ros2 attach` robot's raw
/// routes remotely demandable. Oracle = the test seam reading
/// the writer-side authoritative record; a refused open registers NOTHING.
#[test]
fn test_open_registers_dynamic_egress_with_codec_schema_hash() {
    let mgr = build_mgr("dynreg");
    let codec = bridge_codec();
    let topic = "/rawdyn/dynreg/cloud";

    let expected = codec.schema_hash("geometry_msgs/Vector3");
    assert!(
        expected.is_some(),
        "precondition: the codec must know the type's hash (guards a vacuous \
         None == None pass below)"
    );

    let _route = RawIngressRoute::open(
        &mgr,
        &codec,
        "geometry_msgs/Vector3",
        topic,
        MaxSliceLen::const_new(4096),
    )
    .expect("open route for a schema-known type");
    assert_eq!(
        mgr.dynamic_egress_record_hash_for_test(topic),
        expected,
        "the route's registration record must carry the codec's schema hash"
    );

    // A REFUSED open (unknown type — fails before the publisher exists) must
    // register nothing: no phantom demandable topic for a route that never
    // opened.
    let missing = "/rawdyn/dynreg/never";
    assert!(
        RawIngressRoute::open(
            &mgr,
            &codec,
            "not_a_pkg/NotAType",
            missing,
            MaxSliceLen::const_new(64),
        )
        .is_err(),
        "unknown type must refuse the open"
    );
    assert_eq!(
        mgr.dynamic_egress_record_hash_for_test(missing),
        None,
        "a refused open must not push a registration"
    );
}

#[test]
fn test_decode_failure_drops_sample_and_burns_no_sequence() {
    let mgr = build_mgr("seq");
    let codec = bridge_codec();
    let topic = "/rawroute/seq/gapfree";

    let mut route = RawIngressRoute::open(
        &mgr,
        &codec,
        "std_msgs/String",
        topic,
        MaxSliceLen::const_new(4096),
    )
    .expect("open route");
    let sub = mgr.create_subscriber(topic).expect("observer");

    // Good publish A (seq 0).
    route
        .publish_dds_payload(&codec, &string_dds_payload("alpha"), 11)
        .expect("A publishes");

    // Truncated CDR (length prefix claims more than the body holds) →
    // Decode Err, counted, sequence NOT burned.
    let mut truncated = CDR_LE_ENCAPSULATION.to_vec();
    truncated.extend_from_slice(&100u32.to_le_bytes()); // hostile/truncated
    truncated.extend_from_slice(b"xy");
    let err = route
        .publish_dds_payload(&codec, &truncated, 12)
        .unwrap_err();
    assert!(
        matches!(err, RawRouteError::Decode { .. }),
        "expected Decode, got: {err}"
    );
    assert!(
        err.to_string().contains("sequence not burned"),
        "the drop semantics are named in the error: {err}"
    );

    // A parameter-list encapsulation is also a Decode-class failure (the
    // codec's split_encapsulation rejects PL_CDR) — never a publish.
    let mut pl_cdr = vec![0x00, 0x02, 0x00, 0x00];
    pl_cdr.extend_from_slice(&[0u8; 8]);
    let err = route.publish_dds_payload(&codec, &pl_cdr, 13).unwrap_err();
    assert!(matches!(err, RawRouteError::Decode { .. }), "got: {err}");

    // Good publish B — must carry seq 1 (gap-free), not 3.
    route
        .publish_dds_payload(&codec, &string_dds_payload("bravo"), 14)
        .expect("B publishes");

    let frames = drain_frames(&sub);
    assert_eq!(frames.len(), 2, "only the two good samples delivered");
    assert_eq!(frames[0], string_expected_frame("alpha", 0, 11));
    assert_eq!(frames[1], string_expected_frame("bravo", 1, 14));
    assert_eq!(route.published_total(), 2);
    assert_eq!(route.decode_failures_total(), 2);
    assert_eq!(route.next_sequence(), 2);
}

#[test]
fn test_unknown_type_refused_at_open_before_any_service_exists() {
    let mgr = build_mgr("unknown");
    let codec = bridge_codec();
    let topic = "/rawroute/unknown/bogus";

    // (let-else, not unwrap_err: the Ok type `RawIngressRoute` is
    // deliberately not Debug — nothing useful to print about a live route.)
    let Err(err) = RawIngressRoute::open(
        &mgr,
        &codec,
        "totally/Bogus",
        topic,
        MaxSliceLen::const_new(4096),
    ) else {
        panic!("a schema-less ros_type must be refused at open");
    };
    assert!(
        matches!(err, RawRouteError::UnknownType { .. }),
        "got: {err}"
    );
    let msg = err.to_string();
    assert!(msg.contains("no MessageSchema"), "{msg}");
    assert!(msg.contains("never \"no codec\""), "{msg}");
    assert!(msg.contains("totally/Bogus"), "names the offender: {msg}");

    // The refusal happened BEFORE publisher creation: the topic has no
    // services (an open-only subscriber cannot attach).
    let Err(open_err) = mgr.create_subscriber_open_only(topic) else {
        panic!("no service may exist for a refused route — open_only attached to one");
    };
    assert!(
        matches!(open_err, TransportError::SubscriberCreation { .. }),
        "expected SubscriberCreation (no services), got: {open_err}"
    );
}

#[test]
fn test_publish_cdr_body_is_byte_identical_to_publish_dds_payload() {
    let mgr = build_mgr("parity");
    let codec = bridge_codec();
    let topic_full = "/rawroute/parity/full";
    let topic_body = "/rawroute/parity/body";

    let mut via_payload = RawIngressRoute::open(
        &mgr,
        &codec,
        "std_msgs/String",
        topic_full,
        MaxSliceLen::const_new(4096),
    )
    .expect("open full-payload route");
    let mut via_body = RawIngressRoute::open(
        &mgr,
        &codec,
        "std_msgs/String",
        topic_body,
        MaxSliceLen::const_new(4096),
    )
    .expect("open body route");
    let sub_full = mgr.create_subscriber(topic_full).expect("sub full");
    let sub_body = mgr.create_subscriber(topic_body).expect("sub body");

    let payload = string_dds_payload("probe");
    via_payload
        .publish_dds_payload(&codec, &payload, 42)
        .expect("full-payload publish");
    // The pump-facing shape: encapsulation already stripped (raw::RawSample).
    via_body
        .publish_cdr_body(&codec, CdrEndianness::Little, &payload[4..], 42)
        .expect("body publish");

    let f_full = drain_frames(&sub_full);
    let f_body = drain_frames(&sub_body);
    assert_eq!(f_full.len(), 1);
    assert_eq!(f_body.len(), 1);
    let expected = string_expected_frame("probe", 0, 42);
    assert_eq!(f_full[0], expected, "full-payload frame == hand oracle");
    assert_eq!(f_body[0], expected, "body frame == hand oracle");
    assert_eq!(f_full[0], f_body[0], "the two entry points are one path");
}

#[test]
fn test_decoded_frame_header_matches_a_natively_published_equivalent() {
    // Parity pin: for the same logical message, the
    // codec-decoded frame's header layout (schema_hash, total_size,
    // offset_table_offset, offset_table_count) must equal a NATIVELY
    // published frame's — sequence/timestamp aside (publisher counters /
    // clock). A payload-relative offset_table_offset (0 for String) would
    // disagree with the native publisher's frame-relative stamp (32).
    let mgr = build_mgr("parity_native");
    let topic_native = "/rawroute/parity/native";
    let topic_raw = "/rawroute/parity/decoded";

    // Native half: loan_proxy + set_data — the production writer path.
    let mut publisher = mgr
        .create_publisher_simple(topic_native, MaxSliceLen::const_new(4096))
        .expect("native publisher");
    let sub_native = mgr.create_subscriber(topic_native).expect("sub native");
    publisher
        .loan_proxy::<native_ros2_messages::std_msgs::String>()
        .expect("loan")
        .set_data("probe")
        .expect("set_data");
    // (The OutputProxy publishes on drop, at the end of that statement.)

    // Codec half, through the raw route.
    let codec = bridge_codec();
    let mut route = RawIngressRoute::open(
        &mgr,
        &codec,
        "std_msgs/String",
        topic_raw,
        MaxSliceLen::const_new(4096),
    )
    .expect("raw route");
    let sub_raw = mgr.create_subscriber(topic_raw).expect("sub raw");
    route
        .publish_dds_payload(&codec, &string_dds_payload("probe"), 42)
        .expect("raw publish");

    let native = drain_frames(&sub_native);
    let raw = drain_frames(&sub_raw);
    assert_eq!(native.len(), 1, "native frame delivered");
    assert_eq!(raw.len(), 1, "raw-route frame delivered");
    let nh = WireHeader::read_from_buf(&native[0]).expect("native header");
    let rh = WireHeader::read_from_buf(&raw[0]).expect("raw header");
    assert_eq!(rh.schema_hash, nh.schema_hash, "schema hash parity");
    assert_eq!(rh.total_size, nh.total_size, "total size parity");
    assert_eq!(
        rh.offset_table_offset, nh.offset_table_offset,
        "FRAME-relative table offset parity (payload-relative would read 0 vs 32)"
    );
    assert_eq!(rh.offset_table_count, nh.offset_table_count, "count parity");
    // Payload parity too: the same logical message, byte-for-byte.
    assert_eq!(
        native[0][WireHeader::SIZE..],
        raw[0][WireHeader::SIZE..],
        "payload bytes must agree between the native writer and the codec"
    );
}

#[test]
fn test_two_routes_on_one_transport_are_independent() {
    let mgr = build_mgr("multi");
    let codec = bridge_codec();
    let topic_v3 = "/rawroute/multi/v3";
    let topic_s = "/rawroute/multi/s";

    let mut r_v3 = RawIngressRoute::open(
        &mgr,
        &codec,
        "geometry_msgs/Vector3",
        topic_v3,
        MaxSliceLen::const_new(4096),
    )
    .expect("v3 route");
    let mut r_s = RawIngressRoute::open(
        &mgr,
        &codec,
        "std_msgs/String",
        topic_s,
        MaxSliceLen::const_new(4096),
    )
    .expect("string route");
    let sub_v3 = mgr.create_subscriber(topic_v3).expect("sub v3");
    let sub_s = mgr.create_subscriber(topic_s).expect("sub s");

    // Interleave: v3 good, string good, string DECODE FAILURE, v3 good.
    r_v3.publish_dds_payload(&codec, &vector3_dds_payload(1.0, 0.0, 0.0), 1)
        .expect("v3 #1");
    r_s.publish_dds_payload(&codec, &string_dds_payload("ok"), 2)
        .expect("s #1");
    let mut bad = CDR_LE_ENCAPSULATION.to_vec();
    bad.extend_from_slice(&999u32.to_le_bytes());
    assert!(matches!(
        r_s.publish_dds_payload(&codec, &bad, 3).unwrap_err(),
        RawRouteError::Decode { .. }
    ));
    r_v3.publish_dds_payload(&codec, &vector3_dds_payload(2.0, 0.0, 0.0), 4)
        .expect("v3 #2");

    // Per-topic delivery, each against its hand oracle; the string route's
    // failure never touches the v3 route's stream or counters.
    let f_v3 = drain_frames(&sub_v3);
    assert_eq!(f_v3.len(), 2);
    assert_eq!(f_v3[0], vector3_expected_frame(1.0, 0.0, 0.0, 0, 1));
    assert_eq!(f_v3[1], vector3_expected_frame(2.0, 0.0, 0.0, 1, 4));
    let f_s = drain_frames(&sub_s);
    assert_eq!(f_s.len(), 1);
    assert_eq!(f_s[0], string_expected_frame("ok", 0, 2));

    assert_eq!(r_v3.published_total(), 2);
    assert_eq!(r_v3.decode_failures_total(), 0);
    assert_eq!(r_s.published_total(), 1);
    assert_eq!(r_s.decode_failures_total(), 1);
}

#[test]
fn test_determinism_same_script_on_two_isolated_transports_is_byte_identical() {
    // Principle #7: the route is a pure function of (payload, seq, ts) — the
    // same script on two isolated transports yields byte-identical captures,
    // each anchored to the hand oracle (never a bare self-compare).
    let run = |tag: &str| -> Vec<Vec<u8>> {
        let mgr = build_mgr(tag);
        let codec = bridge_codec();
        let topic = "/rawroute/det/stream";
        let mut route = RawIngressRoute::open(
            &mgr,
            &codec,
            "geometry_msgs/Vector3",
            topic,
            MaxSliceLen::const_new(4096),
        )
        .expect("route");
        let sub = mgr.create_subscriber(topic).expect("sub");
        route
            .publish_dds_payload(&codec, &vector3_dds_payload(0.5, -0.5, 8.0), 7)
            .expect("p1");
        // A failure mid-script must not perturb the byte stream.
        let mut bad = CDR_LE_ENCAPSULATION.to_vec();
        bad.extend_from_slice(&[1, 2, 3]);
        assert!(route.publish_dds_payload(&codec, &bad, 8).is_err());
        route
            .publish_dds_payload(&codec, &vector3_dds_payload(1.5, 2.5, -3.5), 9)
            .expect("p2");
        drain_frames(&sub)
    };

    let a = run("det_a");
    let b = run("det_b");
    assert_eq!(a, b, "two isolated runs are byte-identical");
    assert_eq!(a.len(), 2);
    assert_eq!(a[0], vector3_expected_frame(0.5, -0.5, 8.0, 0, 7));
    assert_eq!(a[1], vector3_expected_frame(1.5, 2.5, -3.5, 1, 9));
}

#[test]
#[ignore] // Run by hand (the cyclone_e2e convention): constructs a REAL DDS
          // participant (UDP sockets + the one-per-process slot) — run
          // explicitly with `-- --ignored`. No peer needed: this is
          // the construction-only smoke for the rustdds drop-down
          // (Context::domain_participant → create_subscriber →
          // create_simple_datareader_no_key with the raw adapter).
fn test_box_raw_reader_constructs_via_the_rustdds_drop_down() {
    use cerulion_go2_dds::participant::ParticipantConfig;
    use cerulion_go2_dds::ros2_client::{MessageTypeName, Name};
    use cerulion_go2_dds::{best_effort_qos, Go2Participant};
    use dds_bridge::generic::raw::create_raw_reader;

    let participant = Go2Participant::new(&ParticipantConfig::new(0, Vec::new()))
        .expect("participant (one per process — run this test alone)");
    let node = participant.create_node("rawroute_smoke").expect("node");
    let name = Name::new("/rawroute", "smoke").expect("name");
    let qos = best_effort_qos();
    let topic = node
        .create_topic(&name, MessageTypeName::new("sensor_msgs", "Imu"), &qos)
        .expect("topic (ROS rt/ + type mangling applied by ros2-client)");
    let dp = participant.context().domain_participant();
    let subscriber = dp.create_subscriber(&qos).expect("rustdds subscriber");
    let reader = create_raw_reader(&subscriber, &topic, Some(qos)).expect("raw reader");
    // Construction IS the assertion: the drop-down path documented in
    // `generic::raw` compiles and runs against real rustdds. The stream half
    // needs a live CycloneDDS peer — `cyclone_e2e_test`'s job.
    let _ = reader.guid();
}
