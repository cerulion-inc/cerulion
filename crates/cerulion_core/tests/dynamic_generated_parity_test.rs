// SPDX-License-Identifier: AGPL-3.0-only
//! `cerulion_core::dynamic::FrameEncoder` byte parity against the generated
//! `ShmMessage` / `OutputProxy` publish path.
//!
//! Each test publishes a message through the real generated writer, captures
//! the committed frame, then encodes the SAME values through `FrameEncoder`
//! for the layout `SchemaSet` resolved from the `.msg` text the generated
//! types were compiled against (`BUILTIN_MSGS`). The two byte strings must be
//! identical. Hand-written oracles pin the header fields and offset table so
//! the comparison can never pass by both sides being wrong the same way.

use std::sync::Arc;

use cerulion_core::dynamic::{
    parse_rosmsg, FieldType, FrameEncoder, FrameView, MessageSchema, SchemaSet, WireHeader,
};
use cerulion_core::message::ShmMessage;
use cerulion_core::testing::TestTransport;
use cerulion_core::wire::{MaxPayloadCapacity, MaxSliceLen};
use cerulion_core::CerulionSubscriber;
use native_ros2_messages::sensor_msgs::{ChannelFloat32, ChannelFloat32Shm, Image};

fn builtin_set() -> SchemaSet {
    let mut schemas: Vec<MessageSchema> = Vec::new();
    for (pkg, name, text) in native_ros2_messages::BUILTIN_MSGS {
        if let Ok(s) = parse_rosmsg(text, name, Some(pkg)) {
            schemas.push(s);
        }
    }
    let (set, warnings) = SchemaSet::from_schemas(schemas).expect("schemas preflight");
    assert!(
        warnings.is_empty(),
        "built-in schema set must resolve cleanly: {warnings:?}"
    );
    set
}

fn capture_last_frame(sub: &CerulionSubscriber) -> Vec<u8> {
    let mut frames: Vec<Vec<u8>> = Vec::new();
    sub.try_receive(|msg| {
        let mut frame = vec![0u8; WireHeader::SIZE];
        msg.header().write_to_buf(&mut frame);
        frame.extend_from_slice(msg.payload());
        frames.push(frame);
    })
    .expect("drain subscriber");
    match frames.pop() {
        Some(last) => last,
        None => panic!("expected at least one delivered frame"),
    }
}

/// `ChannelFloat32 { name: "rgb", values: [0.5, -1.0, 2.0] }`.
///
/// Wire oracle (payload-relative): fixed 0 B, table [0,16), floor 16,
/// `name` @16 len 3, pad to 20, `values` @20 len 12 → payload 32, total 64.
#[test]
fn channel_float32_generated_publish_equals_frame_encoder_bytes() {
    let set = builtin_set();
    let layout = set
        .layout("sensor_msgs/ChannelFloat32")
        .expect("built-in ChannelFloat32");
    assert_eq!(layout.schema_hash, ChannelFloat32::SCHEMA_HASH);
    assert_eq!(layout.fixed_size, 0);
    assert_eq!(layout.data_floor(), 16);

    let tt = TestTransport::new();
    let mut publisher = tt.publisher("dyn/chan", MaxSliceLen::const_new(256), 0);
    let sub = tt.subscriber("dyn/chan");
    {
        let mut proxy = publisher
            .loan_proxy::<ChannelFloat32>()
            .expect("loan ChannelFloat32");
        proxy.set_name("rgb").expect("name");
        proxy.set_values(&[0.5, -1.0, 2.0]).expect("values");
    }
    let generated = capture_last_frame(&sub);
    assert_eq!(generated.len(), 64);
    let gen_view = FrameView::new(set.walker(), &generated).expect("generated frame is valid");

    let enc = FrameEncoder::new(layout).expect("valid layout");
    assert_eq!(enc.required_len(&[3, 12]), Ok(64));
    let mut ours = vec![0xA5u8; 64];
    let mut cur = enc
        .begin(&mut ours, &[3, 12], gen_view.timestamp_ns())
        .expect("begin");
    cur.set_sequence(gen_view.sequence());
    cur.variable_field_mut("name")
        .expect("name")
        .copy_from_slice(b"rgb");
    let values = cur.variable_field_mut("values").expect("values");
    values[0..4].copy_from_slice(&0.5f32.to_le_bytes());
    values[4..8].copy_from_slice(&(-1.0f32).to_le_bytes());
    values[8..12].copy_from_slice(&2.0f32.to_le_bytes());
    assert_eq!(cur.finish(), 64);

    assert_eq!(
        ours, generated,
        "FrameEncoder must be byte-identical to OutputProxy"
    );

    // Hand-written oracle for everything the publisher does not choose at
    // runtime (sequence/timestamp live at [20..32)).
    let mut oracle = ChannelFloat32::SCHEMA_HASH.to_le_bytes().to_vec();
    oracle.extend_from_slice(&[64, 0, 0, 0, 32, 0, 0, 0, 2, 0, 0, 0]);
    assert_eq!(&generated[..20], &oracle[..]);
    let mut payload = vec![16, 0, 0, 0, 3, 0, 0, 0, 20, 0, 0, 0, 12, 0, 0, 0];
    payload.extend_from_slice(b"rgb\0");
    payload.extend_from_slice(&[0, 0, 0, 0x3F, 0, 0, 0x80, 0xBF, 0, 0, 0, 0x40]);
    assert_eq!(&generated[32..], &payload[..]);
}

/// `Image` exercises fixed fields with interior padding, an empty nested
/// `header` blob, a string and a byte array - the shapes the Python binding
/// hits first.
#[test]
fn image_generated_publish_equals_frame_encoder_bytes() {
    let set = builtin_set();
    let layout = set.layout("sensor_msgs/Image").expect("built-in Image");
    assert_eq!(layout.schema_hash, Image::SCHEMA_HASH);
    assert_eq!(layout.fixed_size, Image::WIRE_FIXED_SIZE);
    assert_eq!(layout.variable_fields.len(), Image::VARIABLE_FIELD_COUNT);

    let data: Vec<u8> = (0u8..64).collect();
    let tt = TestTransport::new();
    let mut publisher = tt.publisher("dyn/image", MaxSliceLen::const_new(64 * 1024), 0);
    let sub = tt.subscriber("dyn/image");
    {
        let mut proxy = publisher.loan_proxy::<Image>().expect("loan Image");
        proxy.height = 480;
        proxy.width = 640;
        proxy.is_bigendian = 0;
        proxy.step = 1920;
        proxy.set_header_bytes(&[]).expect("empty header idiom");
        proxy.set_encoding("rgb8").expect("set encoding");
        proxy.set_data(&data).expect("set data");
    }
    let generated = capture_last_frame(&sub);
    let gen_view = FrameView::new(set.walker(), &generated).expect("generated frame is valid");

    let var_lens: Vec<usize> = layout
        .variable_fields
        .iter()
        .map(|v| match v.name.as_str() {
            "header" => 0,
            "encoding" => 4,
            "data" => data.len(),
            other => panic!("unexpected variable field {other}"),
        })
        .collect();
    let enc = FrameEncoder::new(layout).expect("valid layout");
    let total = enc.required_len(&var_lens).expect("fits");
    assert_eq!(total, generated.len());
    let mut ours = vec![0x5Au8; total];
    let mut cur = enc
        .begin(&mut ours, &var_lens, gen_view.timestamp_ns())
        .expect("begin");
    cur.set_sequence(gen_view.sequence());
    cur.fixed_field_mut("height")
        .expect("height")
        .copy_from_slice(&480u32.to_le_bytes());
    cur.fixed_field_mut("width")
        .expect("width")
        .copy_from_slice(&640u32.to_le_bytes());
    cur.fixed_field_mut("is_bigendian").expect("is_bigendian")[0] = 0;
    cur.fixed_field_mut("step")
        .expect("step")
        .copy_from_slice(&1920u32.to_le_bytes());
    cur.variable_field_mut("encoding")
        .expect("encoding")
        .copy_from_slice(b"rgb8");
    cur.variable_field_mut("data")
        .expect("data")
        .copy_from_slice(&data);
    assert_eq!(cur.finish(), total);

    assert_eq!(
        ours, generated,
        "FrameEncoder must be byte-identical to OutputProxy"
    );
    assert_eq!(
        FrameView::new(set.walker(), &ours)
            .expect("ours valid")
            .str_field("encoding"),
        Ok("rgb8")
    );
}

/// The writer-level path (no transport): `ChannelFloat32Shm::from_bytes_mut`
/// on a caller buffer, exactly what a binding gets from `loan_raw_uninit`.
#[test]
fn channel_float32_writer_payload_equals_frame_encoder_payload() {
    let set = builtin_set();
    let layout = set
        .layout("sensor_msgs/ChannelFloat32")
        .expect("built-in ChannelFloat32");

    let mut writer_buf = [0xC3u8; 64];
    let payload_len = {
        let mut w = ChannelFloat32Shm::from_bytes_mut(
            &mut writer_buf,
            MaxPayloadCapacity::const_new(64),
            Arc::from("dyn/chan"),
        );
        w.set_name("xy").expect("name");
        w.set_values(&[1.0, 2.0]).expect("values");
        ChannelFloat32::payload_wire_size(&w)
    };
    // floor 16 + "xy" 2 → 18, align 4 → 20, + 8 B of f32 = 28.
    assert_eq!(payload_len, 28);

    let enc = FrameEncoder::new(layout).expect("valid layout");
    let mut ours = vec![0u8; WireHeader::SIZE + payload_len];
    let mut cur = enc.begin(&mut ours, &[2, 8], 0).expect("begin");
    cur.variable_field_mut("name")
        .expect("name")
        .copy_from_slice(b"xy");
    let values = cur.variable_field_mut("values").expect("values");
    values[0..4].copy_from_slice(&1.0f32.to_le_bytes());
    values[4..8].copy_from_slice(&2.0f32.to_le_bytes());
    assert_eq!(cur.finish(), WireHeader::SIZE + payload_len);

    assert_eq!(&ours[WireHeader::SIZE..], &writer_buf[..payload_len]);
    let mut oracle = vec![16, 0, 0, 0, 2, 0, 0, 0, 20, 0, 0, 0, 8, 0, 0, 0];
    oracle.extend_from_slice(b"xy\0\0");
    oracle.extend_from_slice(&[0, 0, 0x80, 0x3F, 0, 0, 0, 0x40]);
    assert_eq!(&writer_buf[..payload_len], &oracle[..]);
}

/// Everything about a schema that reaches the wire: qualified name, field
/// names and types in declaration order, and the recipe-3 hash.
type IrShape = (String, Vec<(String, FieldType)>, u64);

fn ir_shape(schemas: &[MessageSchema]) -> Vec<IrShape> {
    schemas
        .iter()
        .map(|s| {
            (
                s.qualified_name(),
                s.fields
                    .iter()
                    .map(|f| (f.name.clone(), f.field_type.clone()))
                    .collect(),
                s.checked_schema_hash().expect("sizable"),
            )
        })
        .collect()
}

/// `SchemaSet::add_yaml_str` must build the SAME IR the CLI's workspace
/// schema parser builds (`cerulion_cli_engine::schema_cmd::parse_message_schemas`),
/// so a binding and `cerulion schema`/`graph run` agree on every hash. The
/// example workspaces' real schema files are the corpus; the assertion is
/// against the CLI parse, plus a hand-written pin for one of them.
#[test]
fn yaml_schema_set_matches_cli_parser_on_example_workspaces() {
    let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../examples");
    let files = [("perception", "detections.yaml"), ("go2", "mux_state.yaml")];
    for (ws, file) in files {
        let rel = format!("{ws}/schemas/{file}");
        let text = std::fs::read_to_string(root.join(&rel)).expect(&rel);
        let from_cli =
            cerulion_cli_engine::schema_cmd::parse_message_schemas(&text).expect("cli parses");
        let expected = match ws {
            "perception" => vec![(
                "DetectionArray".to_string(),
                vec![
                    (
                        "boxes".to_string(),
                        FieldType::DynamicArray {
                            element_type: Box::new(FieldType::F64),
                        },
                    ),
                    (
                        "scores".to_string(),
                        FieldType::DynamicArray {
                            element_type: Box::new(FieldType::F64),
                        },
                    ),
                    (
                        "class_ids".to_string(),
                        FieldType::DynamicArray {
                            element_type: Box::new(FieldType::F64),
                        },
                    ),
                ],
                0xb327_476e_e92d_d913,
            )],
            "go2" => vec![(
                "MuxState".to_string(),
                vec![
                    ("active".to_string(), FieldType::U8),
                    ("joy_stale".to_string(), FieldType::Bool),
                    ("key_stale".to_string(), FieldType::Bool),
                ],
                0x592f_b17a_26d5_c0eb,
            )],
            _ => unreachable!(),
        };
        assert_eq!(ir_shape(&from_cli), expected, "{rel} independent oracle");
        if ws == "go2" {
            assert_eq!(
                from_cli[0].description.as_deref(),
                Some("Teleop mux arbitration state: active source + per-source staleness")
            );
        }
        let (mut set, _) = SchemaSet::from_schemas(Vec::new()).unwrap();
        set.add_yaml_str(&text).expect("dynamic parses");
        assert_eq!(ir_shape(set.schemas()), expected, "{rel} dynamic oracle");
        assert_eq!(ir_shape(set.schemas()), ir_shape(&from_cli), "{rel}");
        let (from_ws, warnings) = SchemaSet::from_workspace_dir(&root.join(ws)).expect("workspace");
        assert!(warnings.is_empty(), "{warnings:?}");
        // The workspace loader yields the YAML schemas first, then the
        // `schemas/<pkg>/msg/*.msg` store (go2 ships one such file).
        let ws_shape = ir_shape(from_ws.schemas());
        assert_eq!(
            &ws_shape[..expected.len()],
            &expected[..],
            "{rel} workspace oracle"
        );
        assert_eq!(
            &ws_shape[..from_cli.len()],
            &ir_shape(&from_cli)[..],
            "{rel} via workspace dir"
        );
        let msg_store: Vec<&str> = ws_shape[from_cli.len()..]
            .iter()
            .map(|(name, _, _)| name.as_str())
            .collect();
        let expected_store: &[&str] = if ws == "go2" {
            &["unitree_go/Go2FrontVideoData"]
        } else {
            &[]
        };
        assert_eq!(msg_store, expected_store, "{ws} .msg store");
    }
    let (set, _) = SchemaSet::from_workspace_dir(&root.join("perception")).expect("ws");
    let det = set.layout("DetectionArray").expect("DetectionArray");
    assert_eq!(det.fixed_size, 0);
    assert_eq!(
        det.variable_fields
            .iter()
            .map(|v| v.name.as_str())
            .collect::<Vec<_>>(),
        ["boxes", "scores", "class_ids"]
    );
    assert_eq!(det.data_floor(), 24);
}
