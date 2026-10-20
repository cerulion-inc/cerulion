// SPDX-License-Identifier: AGPL-3.0-only
//! Hand-written vectors for the message YAML codec.
//!
//! Frames come from an independent producer: hand-written bytes, or a
//! hand-written CDR body turned into a frame by [`CdrCodec`] (the ROS 2
//! ingress codec, which shares no code with this module). The YAML each
//! test expects is written out by hand.

use super::decode::f32_from_f64;
use super::encode::{format_f32, format_f64, key, message_to_yaml_capped, quote};
use super::scan::find_anchor_or_alias;
use super::*;
use crate::codegen::{
    parse_rosmsg, CdrCodec, CdrEndianness, FieldDef, FieldType, FrameWalker, MessageSchema,
};
use crate::dynamic::DynamicError;
use crate::dynamic::SchemaSet;

const TS: u64 = 0x1122_3344_5566_7788;

const PROBE_YAML: &str = "\
schemas:
  Probe:
    fields:
      uint32 id: {}
      uint8 flag: {}
      string name: {}
      float64[] samples: {}
";

/// `Shapes`: fixed arrays, bool arrays, a bounded string, a fixed array of
/// fixed messages and a byte blob.
///
/// ```text
/// fixed: v float32[3] @0..12, flags bool[2] @12..14, tag string_fixed[4]
///        @14..18, 2 B pad, corners Vec2[2] @20..36   (fixed_size 36)
/// table: signed #0, more #1, blob #2 at [36, 60)      (data floor 60)
/// ```
const SHAPES_YAML: &str = "\
schemas:
  Vec2:
    fields:
      float32 x: {}
      float32 y: {}
  Shapes:
    fields:
      float32[3] v: {}
      bool[2] flags: {}
      int8[] signed: {}
      bool[] more: {}
      string_fixed[4] tag: {}
      Vec2[2] corners: {}
      bytes blob: {}
";

const SHAPES_TEXT: &str = "\
v: [1.0, -2.0, 0.5]
flags: [true, false]
signed: [-1, 2]
more: [false, true, true]
tag: \"ab\"
corners:
  - x: 1.0
    y: 2.0
  - x: 3.0
    y: 4.0
blob: [7]
";

const PROBE_TEXT: &str = "id: 16909060\nflag: 7\nname: \"ab\"\nsamples: [1.0, 2.0]\n";

fn set_from_yaml(yaml: &str) -> SchemaSet {
    let (mut set, _) = SchemaSet::from_schemas(Vec::new()).unwrap();
    let warnings = set.add_yaml_str(yaml).expect("fixture YAML parses");
    assert!(warnings.is_empty(), "unexpected warnings: {warnings:?}");
    set
}

fn one_field_schema(name: &str, field: &str, ft: FieldType) -> MessageSchema {
    let mut schema = MessageSchema::new(name);
    schema.fields.push(FieldDef::new(field, ft));
    schema
}

fn builtin_schemas() -> Vec<MessageSchema> {
    native_ros2_messages::BUILTIN_MSGS
        .iter()
        .map(|(pkg, name, text)| {
            parse_rosmsg(text, name, Some(pkg))
                .unwrap_or_else(|e| panic!("built-in {pkg}/{name} must parse: {e}"))
        })
        .collect()
}

fn builtin_walker() -> FrameWalker {
    FrameWalker::new(builtin_schemas()).0
}

fn header(hash: u64, total: u32, table_offset: u32, count: u32) -> Vec<u8> {
    let mut f = hash.to_le_bytes().to_vec();
    for v in [total, table_offset, count, 0] {
        f.extend_from_slice(&v.to_le_bytes());
    }
    f.extend_from_slice(&TS.to_le_bytes());
    f
}

/// The `Probe` frame for id=0x01020304, flag=7, name="ab", samples=[1.0,
/// 2.0]: the worked byte example of the dynamic dossier, by hand.
fn probe_frame(hash: u64) -> Vec<u8> {
    let mut f = header(hash, 80, 40, 2);
    f.extend_from_slice(&[0x04, 0x03, 0x02, 0x01, 0x07, 0, 0, 0]);
    f.extend_from_slice(&[0x18, 0, 0, 0, 0x02, 0, 0, 0, 0x20, 0, 0, 0, 0x10, 0, 0, 0]);
    f.extend_from_slice(&[0x61, 0x62, 0, 0, 0, 0, 0, 0]);
    f.extend_from_slice(&[0, 0, 0, 0, 0, 0, 0xF0, 0x3F, 0, 0, 0, 0, 0, 0, 0, 0x40]);
    f
}

/// The `Shapes` frame of [`SHAPES_TEXT`], by hand.
fn shapes_frame(hash: u64, more: [u8; 3]) -> Vec<u8> {
    let mut f = header(hash, 98, 68, 3);
    f.extend_from_slice(&[0, 0, 0x80, 0x3F, 0, 0, 0, 0xC0, 0, 0, 0, 0x3F]); // v
    f.extend_from_slice(&[1, 0]); // flags
    f.extend_from_slice(&[0x61, 0x62, 0, 0]); // tag
    f.extend_from_slice(&[0, 0]); // padding to 20
    f.extend_from_slice(&[
        0, 0, 0x80, 0x3F, 0, 0, 0, 0x40, 0, 0, 0x40, 0x40, 0, 0, 0x80, 0x40,
    ]);
    f.extend_from_slice(&[
        60, 0, 0, 0, 2, 0, 0, 0, 62, 0, 0, 0, 3, 0, 0, 0, 65, 0, 0, 0, 1, 0, 0, 0,
    ]);
    f.extend_from_slice(&[0xFF, 0x02]);
    f.extend_from_slice(&more);
    f.push(7);
    assert_eq!(f.len(), 98);
    f
}

/// A little-endian CDR body writer for hand-written test messages.
#[derive(Default)]
struct Cdr(Vec<u8>);

impl Cdr {
    fn align(&mut self, n: usize) -> &mut Self {
        while !self.0.len().is_multiple_of(n) {
            self.0.push(0);
        }
        self
    }
    fn u8(&mut self, v: u8) -> &mut Self {
        self.0.push(v);
        self
    }
    fn u32(&mut self, v: u32) -> &mut Self {
        self.align(4);
        self.0.extend_from_slice(&v.to_le_bytes());
        self
    }
    fn i32(&mut self, v: i32) -> &mut Self {
        self.u32(v as u32)
    }
    fn f32(&mut self, v: f32) -> &mut Self {
        self.u32(v.to_bits())
    }
    fn f64(&mut self, v: f64) -> &mut Self {
        self.align(8);
        self.0.extend_from_slice(&v.to_le_bytes());
        self
    }
    fn string(&mut self, s: &str) -> &mut Self {
        self.u32(s.len() as u32 + 1);
        self.0.extend_from_slice(s.as_bytes());
        self.0.push(0);
        self
    }
}

fn cdr_frame(qname: &str, body: &[u8]) -> Vec<u8> {
    let (codec, _) = CdrCodec::new(builtin_schemas());
    codec
        .decode(qname, CdrEndianness::Little, body, 0, TS)
        .unwrap_or_else(|e| panic!("CdrCodec decode of {qname}: {e}"))
}

/// Encode `frame`, compare with `text`, decode `text`, compare with `frame`.
fn assert_both_ways(walker: &FrameWalker, qname: &str, frame: &[u8], text: &str) {
    assert_eq!(
        message_to_yaml(walker, frame).unwrap(),
        text,
        "encode of {qname}"
    );
    let back = yaml_to_message(walker, qname, text.as_bytes(), TS).unwrap();
    assert_eq!(back, frame, "decode of {qname} must give the same frame");
}

fn decode_err(walker: &FrameWalker, qname: &str, text: &str) -> String {
    match yaml_to_message(walker, qname, text.as_bytes(), TS) {
        Ok(frame) => panic!("{text:?} must be refused, got a {} byte frame", frame.len()),
        Err(e) => e.to_string(),
    }
}

// ------------------------------------------------------------- happy paths

#[test]
fn probe_frame_encodes_to_hand_written_yaml_in_declaration_order() {
    let set = set_from_yaml(PROBE_YAML);
    let frame = probe_frame(set.schema_hash("Probe").unwrap());
    assert_both_ways(set.walker(), "Probe", &frame, PROBE_TEXT);
}

#[test]
fn nested_messages_and_time_follow_the_msg_order() {
    let mut body = Cdr::default();
    body.i32(1).u32(2).string("map");
    body.f64(1.0).f64(2.5).f64(-0.0);
    body.f64(0.0).f64(0.0).f64(0.0).f64(1.0);
    let frame = cdr_frame("geometry_msgs/PoseStamped", &body.0);
    let text = "\
header:
  stamp:
    sec: 1
    nanosec: 2
  frame_id: \"map\"
pose:
  position:
    x: 1.0
    y: 2.5
    z: -0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0
";
    assert_both_ways(&builtin_walker(), "geometry_msgs/PoseStamped", &frame, text);
}

#[test]
fn duration_takes_a_negative_second_count() {
    let mut body = Cdr::default();
    body.i32(-1).u32(999_999_999);
    let frame = cdr_frame("builtin_interfaces/Duration", &body.0);
    let text = "sec: -1\nnanosec: 999999999\n";
    assert_both_ways(
        &builtin_walker(),
        "builtin_interfaces/Duration",
        &frame,
        text,
    );
}

#[test]
fn unbounded_arrays_of_strings_and_floats_are_flow_sequences() {
    let mut body = Cdr::default();
    body.i32(0).u32(0).string("");
    body.u32(2).string("j1").string("j\"2");
    body.u32(2).f64(0.5).f64(1e-7);
    body.u32(0);
    body.u32(1).f64(1.5);
    let frame = cdr_frame("sensor_msgs/JointState", &body.0);
    let text = "\
header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: \"\"
name: [\"j1\", \"j\\\"2\"]
position: [0.5, 1e-7]
velocity: []
effort: [1.5]
";
    assert_both_ways(&builtin_walker(), "sensor_msgs/JointState", &frame, text);
}

#[test]
fn array_of_fixed_messages_is_a_block_sequence() {
    let mut body = Cdr::default();
    body.u32(2).f32(1.0).f32(2.0).f32(0.1);
    body.f32(-1.5).f32(0.0).f32(f32::MAX);
    let frame = cdr_frame("geometry_msgs/Polygon", &body.0);
    let text = "\
points:
  - x: 1.0
    y: 2.0
    z: 0.1
  - x: -1.5
    y: 0.0
    z: 3.4028235e38
";
    assert_both_ways(&builtin_walker(), "geometry_msgs/Polygon", &frame, text);
}

/// `level` takes the constants `OK`..`STALE`; the YAML carries the number.
#[test]
fn array_of_variable_messages_and_a_constant_valued_field() {
    let mut body = Cdr::default();
    body.u8(1).string("motor").string("hot").string("");
    body.u32(2)
        .string("temp")
        .string("71")
        .string("unit")
        .string("C");
    let frame = cdr_frame("diagnostic_msgs/DiagnosticStatus", &body.0);
    let text = "\
level: 1
name: \"motor\"
message: \"hot\"
hardware_id: \"\"
values:
  - key: \"temp\"
    value: \"71\"
  - key: \"unit\"
    value: \"C\"
";
    let walker = builtin_walker();
    assert_both_ways(&walker, "diagnostic_msgs/DiagnosticStatus", &frame, text);
    assert_eq!(
        decode_err(
            &walker,
            "diagnostic_msgs/DiagnosticStatus",
            &text.replace("level: 1", "level: WARN")
        ),
        "level: expected uint8, got string"
    );
}

#[test]
fn fixed_arrays_bool_arrays_bounded_string_and_bytes() {
    let set = set_from_yaml(SHAPES_YAML);
    let frame = shapes_frame(set.schema_hash("Shapes").unwrap(), [0, 1, 1]);
    assert_both_ways(set.walker(), "Shapes", &frame, SHAPES_TEXT);
}

#[test]
fn empty_message_is_an_empty_mapping_and_empty_input_reads_as_one() {
    let walker = builtin_walker();
    let frame = cdr_frame("std_msgs/Empty", &[0]);
    assert_eq!(message_to_yaml(&walker, &frame).unwrap(), "{}\n");
    for text in ["{}", "", "# nothing\n", "null"] {
        let back = yaml_to_message(&walker, "std_msgs/Empty", text.as_bytes(), TS).unwrap();
        assert_eq!(back, frame, "{text:?}");
    }
}

/// An empty variable nested entry (how a writer leaves a nested field
/// unset) prints the target's zero values, so the YAML is complete.
#[test]
fn unset_variable_nested_field_prints_zero_values() {
    let set = set_from_yaml(
        "schemas:\n  Inner:\n    fields:\n      string s: {}\n      uint16 n: {}\n      float32[2] f: {}\n      int8[] d: {}\n  Wrap:\n    fields:\n      Inner inner: {}\n      uint8 k: {}\n",
    );
    // fixed: k @0 (size 1, align 1); table: inner #0 at [1, 9).
    let mut frame = header(set.schema_hash("Wrap").unwrap(), 41, 33, 1);
    frame.push(3);
    frame.extend_from_slice(&[9, 0, 0, 0, 0, 0, 0, 0]);
    assert_eq!(
        message_to_yaml(set.walker(), &frame).unwrap(),
        "inner:\n  s: \"\"\n  n: 0\n  f: [0.0, 0.0]\n  d: []\nk: 3\n"
    );
}

/// A zero-default message (an unset variable nested field) that holds an
/// array type with no YAML form is refused, not printed as an empty array.
#[test]
fn zero_default_of_an_unsupported_array_is_refused() {
    let nested_array = |name: &str, ft: FieldType| {
        let mut inner = MessageSchema::new(format!("{name}Inner"));
        inner.fields.push(FieldDef::new("s", FieldType::String));
        inner.fields.push(FieldDef::new("m", ft));
        let outer = one_field_schema(
            name,
            "inner",
            FieldType::Nested {
                schema_name: format!("{name}Inner"),
                package: None,
                fixed: None,
            },
        );
        let (set, _) = SchemaSet::from_schemas(vec![inner, outer]).unwrap();
        // fixed 0; table: inner #0 at [0, 8), empty.
        let mut frame = header(set.schema_hash(name).unwrap(), 40, 32, 1);
        frame.extend_from_slice(&[8, 0, 0, 0, 0, 0, 0, 0]);
        match message_to_yaml(set.walker(), &frame) {
            Err(e) => e.to_string(),
            Ok(t) => panic!("must be refused, got {t}"),
        }
    };
    let byte_list = || {
        Box::new(FieldType::DynamicArray {
            element_type: Box::new(FieldType::U8),
        })
    };
    assert_eq!(
        nested_array(
            "Rows",
            FieldType::DynamicArray {
                element_type: byte_list()
            }
        ),
        "inner.m: field type uint8[][] has no YAML form"
    );
    assert_eq!(
        nested_array(
            "Square",
            FieldType::FixedArray {
                element_type: byte_list(),
                length: 2
            }
        ),
        "inner.m: field type uint8[][2] has no YAML form"
    );
}

/// Schemas whose unset nested field expands to far more YAML than the
/// frame holds: 2^31 zero-default leaves (two nested fixed arrays, a 2 GiB
/// fixed section, under the 4 GiB wire bound) and 2^20 zero bytes, each
/// behind one empty table entry.
const HUGE_ZERO_YAML: &str = "schemas:\n  Leaf:\n    fields:\n      uint8 v: {}\n  Row:\n    fields:\n      Leaf[65536] leaves: {}\n  Inner:\n    fields:\n      string s: {}\n      Row[32768] rows: {}\n  Raw:\n    fields:\n      string s: {}\n      uint8[1048576] raw: {}\n  Wrap:\n    fields:\n      Inner inner: {}\n  WrapRaw:\n    fields:\n      Raw inner: {}\n";

/// A frame that leaves its one variable nested field unset.
fn unset_nested_frame(set: &SchemaSet, name: &str) -> Vec<u8> {
    // fixed 0; table: inner #0 at [0, 8), empty.
    let mut frame = header(set.schema_hash(name).unwrap(), 40, 32, 1);
    frame.extend_from_slice(&[8, 0, 0, 0, 0, 0, 0, 0]);
    frame
}

/// A schema-sized zero default is shown lazily: truncation keeps the
/// output small without first building a list of every element.
#[test]
fn huge_zero_default_arrays_truncate_without_materialising() {
    let set = set_from_yaml(HUGE_ZERO_YAML);
    let options = YamlEncodeOptions {
        truncate_arrays: Some(2),
        ..Default::default()
    };
    assert_eq!(
        message_to_yaml_with(set.walker(), &unset_nested_frame(&set, "Wrap"), options).unwrap(),
        "inner:\n  s: \"\"\n  rows:\n    - leaves:\n        - v: 0\n        - v: 0\n        - !truncated 65534\n    - leaves:\n        - v: 0\n        - v: 0\n        - !truncated 65534\n    - !truncated 32766\n"
    );
    assert_eq!(
        message_to_yaml_with(set.walker(), &unset_nested_frame(&set, "WrapRaw"), options).unwrap(),
        "inner:\n  s: \"\"\n  raw: [0, 0, !truncated 1048574]\n"
    );
}

/// A schema whose fixed section alone passes the 1 GiB cap is refused
/// before the reader allocates it: `Inner`'s fixed section is 2 GiB of
/// leaves, and the YAML that names it is a few bytes.
#[test]
fn a_fixed_section_past_the_cap_is_refused_before_it_is_allocated() {
    let set = set_from_yaml(HUGE_ZERO_YAML);
    let w = set.walker();
    assert_eq!(
        decode_err(w, "Inner", "s: \"\"\nrows: []\n"),
        "(message): the message is larger than the 1 GiB frame cap"
    );
    // Behind a variable nested field the refusal names that field.
    assert_eq!(
        decode_err(w, "Wrap", "inner:\n  s: \"\"\n  rows: []\n"),
        "inner: the message is larger than the 1 GiB frame cap"
    );
    // A fixed section under the cap still reaches the value checks.
    assert_eq!(
        decode_err(w, "Raw", "s: \"\"\nraw: []\n"),
        "raw: expected exactly 1048576 elements, got 0"
    );
}

/// `Path`: an array of fixed messages and a `string[]`, three elements each.
///
/// ```text
/// fixed: none                        (fixed_size 0)
/// table: pts #0 at [16, 28), names #1 at [28, 47)
/// pts:   x=1.0, x=2.0, x=3.0         (stride 4, no count)
/// names: count 3, then (len, bytes) for "a", "b", "c"
/// ```
const PATH_YAML: &str = "schemas:\n  Pt:\n    fields:\n      float32 x: {}\n  Path:\n    fields:\n      Pt[] pts: {}\n      string[] names: {}\n";

fn path_frame(hash: u64, third_name_len: u8) -> Vec<u8> {
    let mut f = header(hash, 79, 32, 2);
    f.extend_from_slice(&[16, 0, 0, 0, 12, 0, 0, 0, 28, 0, 0, 0, 19, 0, 0, 0]);
    f.extend_from_slice(&[0, 0, 0x80, 0x3F, 0, 0, 0, 0x40, 0, 0, 0x40, 0x40]);
    f.extend_from_slice(&[3, 0, 0, 0, 1, 0, 0, 0, b'a', 1, 0, 0, 0, b'b']);
    f.extend_from_slice(&[third_name_len, 0, 0, 0, b'c']);
    assert_eq!(f.len(), 79);
    f
}

/// A truncated echo decodes the frame holding only the elements it shows,
/// and still checks every element: a bad element past the cut refuses the
/// array exactly as a full echo does.
#[test]
fn truncation_holds_only_the_elements_shown_and_checks_the_rest() {
    let set = set_from_yaml(PATH_YAML);
    let w = set.walker();
    let hash = set.schema_hash("Path").unwrap();
    let good = path_frame(hash, 1);
    assert_both_ways(
        w,
        "Path",
        &good,
        "pts:\n  - x: 1.0\n  - x: 2.0\n  - x: 3.0\nnames: [\"a\", \"b\", \"c\"]\n",
    );
    let two = YamlEncodeOptions {
        truncate_arrays: Some(2),
        ..Default::default()
    };
    assert_eq!(
        message_to_yaml_with(w, &good, two).unwrap(),
        "pts:\n  - x: 1.0\n  - x: 2.0\n  - !truncated 1\nnames: [\"a\", \"b\", !truncated 1]\n"
    );
    // The kept tree holds two elements of each array; `raw` is all three.
    let kept = w.walk_by_hash_keeping(&good, 2).unwrap();
    for (name, raw_len) in [("pts", 12), ("names", 19)] {
        match kept.field(name) {
            Some(crate::codegen::FrameValueKind::NestedArray { elements, raw }) => {
                assert_eq!((elements.len(), raw.len()), (2, raw_len), "{name}");
            }
            other => panic!("{name}: expected a NestedArray, got {other:?}"),
        }
    }
    // The third name claims 200 bytes: the array is refused cut or not.
    let bad = path_frame(hash, 200);
    let refused =
        "names: array bytes are not in the canonical element framing; the frame has no YAML form";
    assert_eq!(message_to_yaml(w, &bad).unwrap_err().to_string(), refused);
    assert_eq!(
        message_to_yaml_with(w, &bad, two).unwrap_err().to_string(),
        refused
    );
}

/// A string whose escaped form passes the cap is refused while it is being
/// escaped: forty control bytes are 160 bytes of `\xNN`.
#[test]
fn a_string_is_refused_at_the_cap_before_its_escapes_are_built() {
    let set = set_from_yaml(PROBE_YAML);
    let w = set.walker();
    let name = "\u{1}".repeat(40);
    let text = format!(
        "id: 1\nflag: 2\nname: \"{}\"\nsamples: []\n",
        "\\x01".repeat(40)
    );
    let frame = yaml_to_message(w, "Probe", text.as_bytes(), TS).unwrap();
    assert_eq!(message_to_yaml(w, &frame).unwrap(), text);
    let none = YamlEncodeOptions::default();
    // "id: 1\nflag: 2\n" is 14 bytes, then "name:"; 100 bytes cannot hold
    // the 162-byte quoted name.
    assert_eq!(
        message_to_yaml_capped(w, &frame, none, 100)
            .unwrap_err()
            .to_string(),
        "name: the YAML form passes the 100-byte output cap"
    );
    assert!(super::encode::quote_within(&name, 100).is_none());
    assert_eq!(
        super::encode::quote_within(&name, 162).as_deref(),
        Some(text["id: 1\nflag: 2\nname: ".len()..text.len() - "\nsamples: []\n".len()].as_ref())
    );
}

/// Output past the cap is refused at the field being written: block lines
/// are counted as written, flow items as they are rendered.
#[test]
fn output_past_the_cap_is_refused_at_the_field_being_written() {
    let set = set_from_yaml(HUGE_ZERO_YAML);
    let none = YamlEncodeOptions::default();
    // "inner:\n" (7) + "  s: \"\"\n" (9) + "  rows:\n" (8) +
    // "    - leaves:\n" (14) = 38, then 15 bytes per "        - v: 0\n":
    // leaves[0] ends at 53, leaves[1] at 68.
    let err = message_to_yaml_capped(set.walker(), &unset_nested_frame(&set, "Wrap"), none, 60)
        .unwrap_err();
    assert_eq!(
        err.to_string(),
        "inner.rows[0].leaves[1].v: the YAML form passes the 60-byte output cap"
    );
    assert!(matches!(
        err,
        DynamicError::YamlOutputTooLarge { max_bytes: 60, .. }
    ));
    // At exactly 53 bytes leaves[0] still fits.
    let err = message_to_yaml_capped(set.walker(), &unset_nested_frame(&set, "Wrap"), none, 53)
        .unwrap_err();
    assert_eq!(
        err.to_string(),
        "inner.rows[0].leaves[1].v: the YAML form passes the 53-byte output cap"
    );
    let err = message_to_yaml_capped(set.walker(), &unset_nested_frame(&set, "WrapRaw"), none, 64)
        .unwrap_err();
    assert_eq!(
        err.to_string(),
        "inner.raw: the YAML form passes the 64-byte output cap"
    );
    // A message that fits is unchanged by the cap.
    let probe = set_from_yaml(PROBE_YAML);
    let frame = probe_frame(probe.schema_hash("Probe").unwrap());
    assert_eq!(
        message_to_yaml_capped(probe.walker(), &frame, none, PROBE_TEXT.len()).unwrap(),
        PROBE_TEXT
    );
    assert!(message_to_yaml_capped(probe.walker(), &frame, none, PROBE_TEXT.len() - 1).is_err());
}

/// `HUGE_ZERO_YAML` with leaves that hold one empty message: every output
/// line is a key and ` {}`, no inline scalar.
const HUGE_EMPTY_LEAF_YAML: &str = "schemas:\n  Empty:\n    fields: {}\n  Leaf:\n    fields:\n      Empty e: {}\n  Row:\n    fields:\n      Leaf[65536] leaves: {}\n  Inner:\n    fields:\n      string s: {}\n      Row[32768] rows: {}\n  Wrap:\n    fields:\n      Inner inner: {}\n";

/// Key lines, ` {}` and ` []` count toward the cap like inline values: a
/// schema whose every line is `- e: {}` is still refused.
#[test]
fn key_lines_and_empty_messages_count_toward_the_cap() {
    let set = set_from_yaml(HUGE_EMPTY_LEAF_YAML);
    let none = YamlEncodeOptions::default();
    // 38 bytes up to "    - leaves:\n", then 16 per "        - e: {}\n":
    // leaves[0] ends at 54; leaves[1]'s key line would end at 66.
    let err = message_to_yaml_capped(set.walker(), &unset_nested_frame(&set, "Wrap"), none, 60)
        .unwrap_err();
    assert_eq!(
        err.to_string(),
        "inner.rows[0].leaves[1].e: the YAML form passes the 60-byte output cap"
    );
    // The ` {}\n` after the key is counted too.
    let err = message_to_yaml_capped(set.walker(), &unset_nested_frame(&set, "Wrap"), none, 67)
        .unwrap_err();
    assert_eq!(
        err.to_string(),
        "inner.rows[0].leaves[1].e: the YAML form passes the 67-byte output cap"
    );
}

/// The real cap stops a 2^31-line form after 256 MiB, not 32 GiB.
#[test]
fn the_real_cap_stops_an_expanding_frame() {
    let set = set_from_yaml(HUGE_EMPTY_LEAF_YAML);
    let err = message_to_yaml(set.walker(), &unset_nested_frame(&set, "Wrap")).unwrap_err();
    // 24 bytes up to "  rows:\n", then 14 + 65536 * 16 = 1048590 per
    // row: 255 rows end at 267390474, and 65310 more 16-byte leaves at
    // 268435434, so leaves[65310]'s 12-byte key line passes the cap.
    assert_eq!(
        err.to_string(),
        "inner.rows[255].leaves[65310].e: the YAML form passes the 268435456-byte output cap"
    );
}

/// The `!truncated <n>` marker of a cut array counts toward the cap.
#[test]
fn the_truncation_marker_counts_toward_the_cap() {
    let set = set_from_yaml(PROBE_YAML);
    let frame = probe_frame(set.schema_hash("Probe").unwrap());
    let opts = YamlEncodeOptions {
        truncate_arrays: Some(1),
        ..Default::default()
    };
    let text = "id: 16909060\nflag: 7\nname: \"ab\"\nsamples: [1.0, !truncated 1]\n";
    assert_eq!(
        message_to_yaml_capped(set.walker(), &frame, opts, text.len()).unwrap(),
        text
    );
    assert_eq!(
        message_to_yaml_capped(set.walker(), &frame, opts, text.len() - 1)
            .unwrap_err()
            .to_string(),
        format!(
            "samples: the YAML form passes the {}-byte output cap",
            text.len() - 1
        )
    );
    // A cut block sequence: the `- !truncated <n>` line is counted.
    let set = set_from_yaml(HUGE_ZERO_YAML);
    let frame = unset_nested_frame(&set, "Wrap");
    let opts = YamlEncodeOptions {
        truncate_arrays: Some(1),
        ..Default::default()
    };
    let text = "inner:\n  s: \"\"\n  rows:\n    - leaves:\n        - v: 0\n        - !truncated 65535\n    - !truncated 32767\n";
    assert_eq!(
        message_to_yaml_capped(set.walker(), &frame, opts, text.len()).unwrap(),
        text
    );
    assert_eq!(
        message_to_yaml_capped(set.walker(), &frame, opts, text.len() - 1)
            .unwrap_err()
            .to_string(),
        format!(
            "inner.rows: the YAML form passes the {}-byte output cap",
            text.len() - 1
        )
    );
}

/// `max_lines` stops the output after that many lines and ends it with a
/// column-0 `!truncated` line; the work stops with it.
#[test]
fn max_lines_stops_the_output_at_a_whole_line() {
    let set = set_from_yaml(HUGE_ZERO_YAML);
    let frame = unset_nested_frame(&set, "Wrap");
    let lines = |n| YamlEncodeOptions {
        max_lines: Some(n),
        ..Default::default()
    };
    let cut = message_to_yaml_with(set.walker(), &frame, lines(5)).unwrap();
    assert_eq!(
        cut,
        "inner:\n  s: \"\"\n  rows:\n    - leaves:\n        - v: 0\n!truncated\n"
    );
    // Two runs give the same bytes.
    assert_eq!(
        message_to_yaml_with(set.walker(), &frame, lines(5)).unwrap(),
        cut
    );
    // Cut output never reads back as a shorter message.
    assert!(matches!(
        yaml_to_message(set.walker(), "Wrap", cut.as_bytes(), TS),
        Err(DynamicError::YamlSyntax(_))
    ));
    // Zero lines: only the marker.
    assert_eq!(
        message_to_yaml_with(set.walker(), &frame, lines(0)).unwrap(),
        "!truncated\n"
    );
    // A message of exactly `n` lines is not cut.
    let probe = set_from_yaml(PROBE_YAML);
    let frame = probe_frame(probe.schema_hash("Probe").unwrap());
    assert_eq!(
        message_to_yaml_with(probe.walker(), &frame, lines(4)).unwrap(),
        PROBE_TEXT
    );
    assert_eq!(
        message_to_yaml_with(probe.walker(), &frame, lines(3)).unwrap(),
        "id: 16909060\nflag: 7\nname: \"ab\"\n!truncated\n"
    );
    // The marker counts toward the byte cap.
    let three = "id: 16909060\nflag: 7\nname: \"ab\"\n!truncated\n";
    assert_eq!(
        message_to_yaml_capped(probe.walker(), &frame, lines(3), three.len()).unwrap(),
        three
    );
    assert!(matches!(
        message_to_yaml_capped(probe.walker(), &frame, lines(3), three.len() - 1),
        Err(DynamicError::YamlOutputTooLarge { .. })
    ));
}

#[test]
fn the_output_cap_is_256_mib() {
    assert_eq!(MAX_YAML_BYTES, 268_435_456);
}

#[test]
fn integers_are_accepted_for_float_fields() {
    let set = set_from_yaml(PROBE_YAML);
    let text = "id: 16909060\nflag: 7\nname: \"ab\"\nsamples: [1, 2]\n";
    let frame = yaml_to_message(set.walker(), "Probe", text.as_bytes(), TS).unwrap();
    assert_eq!(frame, probe_frame(set.schema_hash("Probe").unwrap()));
}

#[test]
fn hand_typed_flow_and_plain_forms_decode_the_same() {
    let set = set_from_yaml(PROBE_YAML);
    let text = "{flag: 7, id: 16909060, name: ab, samples: [1.0, 2.0]}";
    let frame = yaml_to_message(set.walker(), "Probe", text.as_bytes(), TS).unwrap();
    assert_eq!(frame, probe_frame(set.schema_hash("Probe").unwrap()));
}

// ------------------------------------------------------------- edge cases

#[test]
fn strings_are_quoted_with_yaml_escapes() {
    assert_eq!(
        quote("a\"b\\c\nd\te\r\0\u{1}\u{1f} ~\u{7f}\u{85}\u{a0}\u{2028}\u{feff}é😀"),
        "\"a\\\"b\\\\c\\nd\\te\\r\\0\\x01\\x1F ~\\x7F\\x85\u{a0}\\u2028\\uFEFFé😀\""
    );
    let set = set_from_yaml(PROBE_YAML);
    let hostile = "a\"b\\c\nd\te\r\0\u{1}\u{7f}\u{85}\u{2028}\u{feff}é😀 true";
    let text = format!("id: 1\nflag: 0\nname: {}\nsamples: []\n", quote(hostile));
    let frame = yaml_to_message(set.walker(), "Probe", text.as_bytes(), TS).unwrap();
    assert_eq!(message_to_yaml(set.walker(), &frame).unwrap(), text);
    let view = crate::dynamic::FrameView::new(set.walker(), &frame).unwrap();
    assert_eq!(view.str_field("name").unwrap(), hostile);
}

#[test]
fn floats_use_the_shortest_form_and_yaml_spellings() {
    let cases = [
        (1.0, "1.0"),
        (0.1, "0.1"),
        (1e-7, "1e-7"),
        (1e21, "1e21"),
        (-0.0, "-0.0"),
        (f64::NAN, ".nan"),
        (f64::INFINITY, ".inf"),
        (f64::NEG_INFINITY, "-.inf"),
        (f64::MAX, "1.7976931348623157e308"),
        (5e-324, "5e-324"),
    ];
    for (v, want) in cases {
        assert_eq!(format_f64(v), want);
    }
    assert_eq!(format_f32(0.1), "0.1");
    assert_eq!(format_f32(-0.0), "-0.0");
    assert_eq!(format_f32(f32::NAN), ".nan");
    assert_eq!(format_f32(f32::INFINITY), ".inf");
    assert_eq!(format_f32(f32::NEG_INFINITY), "-.inf");
    assert_eq!(format_f32(f32::MIN_POSITIVE), "1.1754944e-38");
}

/// A NaN's sign and payload bits are not part of the YAML form: any NaN
/// writes `.nan` and reads back as the quiet NaN, the documented exception
/// to the byte-exact round trip.
#[test]
fn every_nan_writes_dot_nan_and_reads_back_as_the_quiet_nan() {
    let set =
        set_from_yaml("schemas:\n  F:\n    fields:\n      float64 d: {}\n      float32 f: {}\n");
    let w = set.walker();
    // fixed: d @0..8, f @8..12, 4 B pad (fixed_size 16), no table.
    assert_eq!(w.layout_for_name("F").unwrap().fixed_size, 16);
    let frame = |d: u64, f: u32| {
        let mut out = header(set.schema_hash("F").unwrap(), 48, 48, 0);
        out.extend_from_slice(&d.to_le_bytes());
        out.extend_from_slice(&f.to_le_bytes());
        out.extend_from_slice(&[0; 4]);
        out
    };
    // A negative NaN with a payload, in both widths.
    let odd = frame(0xFFF0_0000_0000_0001, 0xFF80_0001);
    assert_eq!(message_to_yaml(w, &odd).unwrap(), "d: .nan\nf: .nan\n");
    let back = yaml_to_message(w, "F", b"d: .nan\nf: .nan\n", TS).unwrap();
    assert_eq!(back, frame(0x7FF8_0000_0000_0000, 0x7FC0_0000));
    assert_ne!(back, odd);
}

/// For bit pattern 363742205 a plain `as f32` of the parsed `f64` lands one
/// step off; the codec still reads back the exact `f32`.
#[test]
fn float32_reads_back_bit_exact_where_a_plain_cast_rounds_twice() {
    let x = f32::from_bits(363_742_205);
    let text = format_f32(x);
    assert_eq!(text, "7.038531e-26");
    let parsed: f64 = text.parse().unwrap();
    assert_ne!(
        (parsed as f32).to_bits(),
        363_742_205,
        "the double rounding this guards"
    );
    assert_eq!(f32_from_f64(parsed).to_bits(), 363_742_205);
    // The negative mirror is the other value a plain cast misses.
    let neg = 363_742_205 | 0x8000_0000;
    let parsed: f64 = format_f32(f32::from_bits(neg)).parse().unwrap();
    assert_ne!((parsed as f32).to_bits(), neg);
    assert_eq!(f32_from_f64(parsed).to_bits(), neg);
    // Hand-typed decimals still land on the nearest f32.
    assert_eq!(f32_from_f64(0.1), 0.1f32);
    // Values too small for an f32 become a signed zero, never a panic.
    assert_eq!(f32_from_f64(1e-50).to_bits(), 0);
    assert_eq!(f32_from_f64(-1e-50).to_bits(), 0x8000_0000);
    assert_eq!(f32_from_f64(f64::INFINITY), f32::INFINITY);
}

#[test]
fn keys_are_plain_only_for_identifiers_yaml_reads_as_strings() {
    let cases = [
        ("x", "x"),
        ("_private", "_private"),
        ("frame_id2", "frame_id2"),
        ("1", "\"1\""),
        ("2d", "\"2d\""),
        ("a-b", "\"a-b\""),
        ("a b", "\"a b\""),
        ("", "\"\""),
        ("null", "\"null\""),
        ("False", "\"False\""),
    ];
    for (name, want) in cases {
        assert_eq!(key(name), want, "{name:?}");
    }
}

#[test]
fn the_scanner_finds_anchors_at_node_starts_only() {
    let cases: [(&str, Option<(usize, usize)>); 24] = [
        ("a: &x 1", Some((1, 4))),
        ("a: *x", Some((1, 4))),
        ("- *x", Some((1, 3))),
        ("[1, *x]", Some((1, 5))),
        ("{a: *x}", Some((1, 5))),
        ("a: !t &x 1", Some((1, 7))),
        ("--- &x\na: 1", Some((1, 5))),
        ("... *x", Some((1, 5))),
        ("a: |\n  *text\nb: *x", Some((3, 4))),
        // A block scalar after a dash belongs to the key, not the dash: a
        // sibling key at the key's column is scanned again.
        ("- body: |\n    text\n  other: &a 1", Some((3, 10))),
        ("- body: |\n    text\n  other: *a", Some((3, 10))),
        ("- - k: >\n      t\n    j: &a 1", Some((3, 8))),
        ("- body: |\n    *text\n  b: 1", None),
        ("- |\n  *text\n- *x", Some((3, 3))),
        // A `:` right after a quoted key is a value indicator with no blank.
        ("{\"a\":&v hello,\"b\":*v}", Some((1, 6))),
        ("{'a':*v}", Some((1, 6))),
        ("\"a\" :*v", Some((1, 6))),
        ("{a:*v}", None),
        ("{a :*v}", Some((1, 5))),
        ("{[a]:*v}", Some((1, 6))),
        // A node after `---` is at column 0 for its block scalar.
        ("--- body: |\n  *text\nb: *x", Some((3, 4))),
        ("a: a*b &c", None),
        ("a: \"*x\" # &y", None),
        ("a: 'it''s *x'", None),
    ];
    for (text, want) in cases {
        assert_eq!(find_anchor_or_alias(text), want, "{text:?}");
    }
}

#[test]
fn keys_that_yaml_would_retype_are_quoted() {
    let set =
        set_from_yaml("schemas:\n  Odd:\n    fields:\n      bool true: {}\n      uint8 y: {}\n");
    let frame = yaml_to_message(set.walker(), "Odd", b"\"true\": true\ny: 2\n", TS).unwrap();
    assert_eq!(
        message_to_yaml(set.walker(), &frame).unwrap(),
        "\"true\": true\ny: 2\n"
    );
}

#[test]
fn truncated_display_marks_the_cut_and_is_refused_back() {
    let set = set_from_yaml(PROBE_YAML);
    let frame = probe_frame(set.schema_hash("Probe").unwrap());
    let opts = YamlEncodeOptions {
        truncate_arrays: Some(1),
        ..Default::default()
    };
    let text = message_to_yaml_with(set.walker(), &frame, opts).unwrap();
    assert_eq!(
        text,
        "id: 16909060\nflag: 7\nname: \"ab\"\nsamples: [1.0, !truncated 1]\n"
    );
    assert_eq!(
        decode_err(set.walker(), "Probe", &text),
        "samples[1]: this array was cut short for display (elements left out: 1); print the \
         message again with a larger --truncate-length"
    );
    // A cut longer than the array changes nothing.
    let wide = YamlEncodeOptions {
        truncate_arrays: Some(2),
        ..Default::default()
    };
    assert_eq!(
        message_to_yaml_with(set.walker(), &frame, wide).unwrap(),
        PROBE_TEXT
    );

    let mut body = Cdr::default();
    body.u32(2)
        .f32(1.0)
        .f32(2.0)
        .f32(0.1)
        .f32(-1.5)
        .f32(0.0)
        .f32(1.0);
    let polygon = cdr_frame("geometry_msgs/Polygon", &body.0);
    let walker = builtin_walker();
    let block = message_to_yaml_with(&walker, &polygon, opts).unwrap();
    assert_eq!(
        block,
        "points:\n  - x: 1.0\n    y: 2.0\n    z: 0.1\n  - !truncated 1\n"
    );
    assert_eq!(
        decode_err(&walker, "geometry_msgs/Polygon", &block),
        "points[1]: this array was cut short for display (elements left out: 1); print the \
         message again with a larger --truncate-length"
    );
}

fn bounded_set() -> SchemaSet {
    let (mut set, _) = SchemaSet::from_schemas(Vec::new()).unwrap();
    set.add_rosmsg_str(
        "string<=2 name\nint32[<=2] ids\nstring<=2[<=2] tags\n",
        "Bounded",
        Some("demo"),
    )
    .unwrap();
    set
}

/// ROS `string<=N` and `T[<=N]` values at their bound read and print back.
#[test]
fn ros_upper_bounds_accept_values_at_the_bound() {
    let set = bounded_set();
    let text = "name: \"ab\"\nids: [1, 2]\ntags: [\"x\", \"yz\"]\n";
    let frame = yaml_to_message(set.walker(), "demo/Bounded", text.as_bytes(), TS).unwrap();
    assert_eq!(message_to_yaml(set.walker(), &frame).unwrap(), text);
    let empty = "name: \"\"\nids: []\ntags: []\n";
    let frame = yaml_to_message(set.walker(), "demo/Bounded", empty.as_bytes(), TS).unwrap();
    assert_eq!(message_to_yaml(set.walker(), &frame).unwrap(), empty);
}

/// A value past a ROS upper bound is refused with the path and the bound.
#[test]
fn ros_upper_bounds_refuse_values_past_the_bound() {
    let set = bounded_set();
    let w = set.walker();
    let err = |text: &str| decode_err(w, "demo/Bounded", text);
    assert_eq!(
        err("name: \"abc\"\nids: []\ntags: []\n"),
        "name: string is 3 bytes, over the bound of 2 bytes"
    );
    assert_eq!(
        err("name: \"\"\nids: [1, 2, 3]\ntags: []\n"),
        "ids: expected at most 2 elements, got 3"
    );
    assert_eq!(
        err("name: \"\"\nids: []\ntags: [\"x\", \"xyz\"]\n"),
        "tags[1]: string is 3 bytes, over the bound of 2 bytes"
    );
    assert_eq!(
        err("name: \"\"\nids: []\ntags: [\"\", \"\", \"\"]\n"),
        "tags: expected at most 2 elements, got 3"
    );
    // A cut array says it was cut, not that it is too long.
    assert_eq!(
        err("name: \"\"\nids: [1, 2, !truncated 5]\ntags: []\n"),
        "ids[2]: this array was cut short for display (elements left out: 5); print the \
         message again with a larger --truncate-length"
    );
    // A value of the wrong kind gets the type error.
    assert_eq!(
        err("name: 1\nids: []\ntags: []\n"),
        "name: expected string, got integer"
    );
    assert_eq!(
        err("name: \"\"\nids: \"x\"\ntags: []\n"),
        "ids: expected int32[], got string"
    );
    // A sequence for the bounded string, and a long string for the bounded
    // string array, are type errors, not bound errors.
    assert_eq!(
        err("name: [\"abc\"]\nids: []\ntags: []\n"),
        "name: expected string, got sequence"
    );
    assert_eq!(
        err("name: \"\"\nids: []\ntags: \"abc\"\n"),
        "tags: expected string[], got string"
    );
    assert!(matches!(
        yaml_to_message(
            w,
            "demo/Bounded",
            b"name: \"\"\nids: [1, 2, 3]\ntags: []\n",
            TS
        ),
        Err(DynamicError::YamlTooManyElements {
            bound: 2,
            got: 3,
            ..
        })
    ));
}

/// The vendored corpus carries bounds: `shape_msgs/SolidPrimitive` has
/// `float64[<=3] dimensions`.
#[test]
fn corpus_bounded_array_is_enforced() {
    let walker = builtin_walker();
    let ok = "type: 1\ndimensions: [1.0, 2.0, 0.5]\npolygon:\n  points: []\n";
    let frame = yaml_to_message(&walker, "shape_msgs/SolidPrimitive", ok.as_bytes(), TS).unwrap();
    assert_eq!(message_to_yaml(&walker, &frame).unwrap(), ok);
    assert_eq!(
        decode_err(
            &walker,
            "shape_msgs/SolidPrimitive",
            "type: 1\ndimensions: [1.0, 2.0, 0.5, 4.0]\npolygon:\n  points: []\n"
        ),
        "dimensions: expected at most 3 elements, got 4"
    );
}

#[test]
fn fixed_array_of_fixed_strings_from_the_ir() {
    let schema = one_field_schema(
        "Codes",
        "codes",
        FieldType::FixedArray {
            element_type: Box::new(FieldType::StringFixed(3)),
            length: 2,
        },
    );
    let (set, _) = SchemaSet::from_schemas(vec![schema]).unwrap();
    let text = "codes: [\"ab\", \"xyz\"]\n";
    let frame = yaml_to_message(set.walker(), "Codes", text.as_bytes(), TS).unwrap();
    assert_eq!(&frame[32..], b"ab\0xyz");
    assert_eq!(message_to_yaml(set.walker(), &frame).unwrap(), text);
}

// ------------------------------------------------------------- adversarial

#[test]
fn anchors_and_aliases_are_refused_where_quotes_and_comments_are_not() {
    let set = set_from_yaml(PROBE_YAML);
    let w = set.walker();
    assert_eq!(
        decode_err(w, "Probe", "id: &a 1\nflag: *a\nname: \"\"\nsamples: []\n"),
        "message YAML uses an anchor or alias at line 1, column 5; write every value out in full"
    );
    assert_eq!(
        decode_err(
            w,
            "Probe",
            "id: 1\nflag: 2\nname: \"\"\nsamples: [1.0, *x]\n"
        ),
        "message YAML uses an anchor or alias at line 4, column 16; write every value out in full"
    );
    assert_eq!(
        decode_err(
            w,
            "Probe",
            "{\"id\":&a 1,\"flag\":*a,\"name\":\"\",\"samples\":[]}"
        ),
        "message YAML uses an anchor or alias at line 1, column 7; write every value out in full"
    );
    assert_eq!(
        decode_err(w, "Probe", "&top\nid: 1\n"),
        "message YAML uses an anchor or alias at line 1, column 1; write every value out in full"
    );
    // `*` and `&` inside a quoted string, a plain string, a comment and a
    // block scalar are text, not anchors.
    for name in [
        "\"*a &b\"",
        "'it''s *a'",
        "a *b &c",
        "x # &comment",
        "|\n  *star",
    ] {
        let text = format!("id: 1\nflag: 2\nname: {name}\nsamples: []\n");
        yaml_to_message(w, "Probe", text.as_bytes(), TS)
            .unwrap_or_else(|e| panic!("{name:?} must decode: {e}"));
    }
}

/// An array of fieldless messages has no count on the wire, so only the
/// empty one has a frame: `[{}, {}]` is refused, never read back as `[]`.
#[test]
fn a_nonempty_array_of_fieldless_messages_is_refused() {
    let set = set_from_yaml(
        "schemas:\n  Nothing:\n    fields: {}\n  Holder:\n    fields:\n      Nothing[] items: {}\n",
    );
    let w = set.walker();
    assert_eq!(
        decode_err(w, "Holder", "items: [{}, {}]\n"),
        "items: field type Nothing[] has no YAML form"
    );
    let frame = yaml_to_message(w, "Holder", b"items: []\n", TS).unwrap();
    assert_eq!(message_to_yaml(w, &frame).unwrap(), "items: []\n");
}

/// An anchor on a sibling key after `- body: |` is still found: the block
/// scalar's content is what is more indented than `body`, not than the dash.
#[test]
fn an_anchor_after_a_block_scalar_in_a_sequence_element_is_refused() {
    let set = set_from_yaml(
        "schemas:\n  Item:\n    fields:\n      string body: {}\n      string other: {}\n  List:\n    fields:\n      Item[] items: {}\n",
    );
    let w = set.walker();
    let text =
        "items:\n  - body: |\n      hi\n    other: &a \"x\"\n  - body: \"y\"\n    other: *a\n";
    assert_eq!(
        decode_err(w, "List", text),
        "message YAML uses an anchor or alias at line 4, column 12; write every value out in full"
    );
    // The same text without the anchor and alias decodes, block scalar and
    // all, so the refusal above is the anchor and nothing else.
    let plain =
        "items:\n  - body: |\n      hi\n    other: \"x\"\n  - body: \"y\"\n    other: \"x\"\n";
    yaml_to_message(w, "List", plain.as_bytes(), TS).expect("plain text decodes");
}

#[test]
fn invalid_utf8_duplicate_keys_and_two_documents_are_refused() {
    let set = set_from_yaml(PROBE_YAML);
    let w = set.walker();
    match yaml_to_message(w, "Probe", b"id: \xff\n", TS) {
        Err(e) => assert_eq!(
            e.to_string(),
            "message YAML is not valid UTF-8 (invalid byte at offset 4)"
        ),
        Ok(_) => panic!("non-UTF-8 must be refused"),
    }
    assert_syntax(w, "id: 1\nid: 2\n", Some(" at line 1 column 1"));
    assert_syntax(w, "id: 1\n---\nid: 2\n", None);
    assert_syntax(w, "id: [1, 2\n", Some(" at line 2 column 1"));
}

/// A `YamlSyntax` refusal: the stable prefix, plus the location when the
/// parser gives one. The text in between is the parser's own and may
/// change with its version, so it is not pinned.
fn assert_syntax(w: &FrameWalker, text: &str, location: Option<&str>) {
    match yaml_to_message(w, "Probe", text.as_bytes(), TS) {
        Err(e @ DynamicError::YamlSyntax(_)) => {
            let shown = e.to_string();
            assert!(
                shown.starts_with("message YAML does not parse: "),
                "{shown}"
            );
            if let Some(at) = location {
                assert!(shown.contains(at), "{shown} names {at}");
            }
        }
        other => panic!("{text:?} must be a syntax error, got {other:?}"),
    }
}

#[test]
fn deep_yaml_nesting_is_refused_by_the_parser_limit() {
    let set = set_from_yaml(PROBE_YAML);
    let text = format!("id: {}{}", "[".repeat(200), "]".repeat(200));
    assert_syntax(set.walker(), &text, Some(" at line 1 column 132"));
}

#[test]
fn huge_array_past_its_fixed_length_is_refused() {
    let set = set_from_yaml(SHAPES_YAML);
    let v = vec!["1.0"; 10_000].join(", ");
    let text = SHAPES_TEXT.replace("v: [1.0, -2.0, 0.5]", &format!("v: [{v}]"));
    assert_eq!(
        decode_err(set.walker(), "Shapes", &text),
        "v: expected exactly 3 elements, got 10000"
    );
}

/// `L0 -> L1 -> ... -> L69`, each a fixed message holding the next.
fn chain_set() -> SchemaSet {
    let mut yaml = String::from("schemas:\n");
    for i in 0..70 {
        yaml.push_str(&format!(
            "  L{i}:\n    fields:\n      L{} next: {{}}\n",
            i + 1
        ));
    }
    yaml.push_str("  L70:\n    fields:\n      uint8 v: {}\n");
    set_from_yaml(&yaml)
}

#[test]
fn nesting_past_the_depth_bound_is_refused_both_ways() {
    let set = chain_set();
    let deep_path = vec!["next"; 65].join(".");
    let mut text = String::new();
    for i in 0..70 {
        text.push_str(&format!("{:w$}next:\n", "", w = 2 * i));
    }
    text.push_str(&format!("{:w$}v: 1\n", "", w = 140));
    assert_eq!(
        decode_err(set.walker(), "L0", &text),
        format!("{deep_path}: messages nest deeper than 64 levels")
    );
    let mut frame = header(set.schema_hash("L0").unwrap(), 33, 33, 0);
    frame.push(1);
    match message_to_yaml(set.walker(), &frame) {
        Err(e) => assert_eq!(
            e.to_string(),
            format!("{deep_path}: messages nest deeper than 64 levels")
        ),
        Ok(t) => panic!("must be refused, got {t}"),
    }
}

// ------------------------------------------------------------- error arms

#[test]
fn every_value_error_names_the_full_path() {
    let walker = builtin_walker();
    let pose = "\
header:
  stamp:
    sec: 1
    nanosec: 2
  frame_id: \"map\"
pose:
  position:
    x: 1.0
    y: 2.5
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0
";
    let q = "geometry_msgs/PoseStamped";
    let cases = [
        (
            pose.replace("x: 1.0", "x: \"abc\""),
            "pose.position.x: expected float64, got string",
        ),
        (
            pose.replace("sec: 1", "sec: 2147483648"),
            "header.stamp.sec: 2147483648 is out of range for int32 (-2147483648 to 2147483647)",
        ),
        (
            pose.replace("nanosec: 2", "nanosec: -1"),
            "header.stamp.nanosec: -1 is out of range for uint32 (0 to 4294967295)",
        ),
        (
            pose.replace("nanosec: 2", "nanosec: 2.5"),
            "header.stamp.nanosec: expected uint32, got float",
        ),
        (
            pose.replace("  position:", "  positon:"),
            "pose.positon: unknown field; geometry_msgs/Pose has fields: position, orientation",
        ),
        (
            pose.replace("    w: 1.0\n", ""),
            "pose.orientation.w: missing field required by geometry_msgs/Quaternion",
        ),
        (
            pose.replace("frame_id: \"map\"", "frame_id: [1]"),
            "header.frame_id: expected string, got sequence",
        ),
        (
            pose.replace("  stamp:\n    sec: 1\n    nanosec: 2\n", "  stamp: 5\n"),
            "header.stamp: expected builtin_interfaces/Time, got integer",
        ),
        (
            pose.replace("x: 1.0", "x: !meters 1.0"),
            "pose.position.x: YAML tag !meters is not supported",
        ),
        (
            "[1]".to_string(),
            "(message): expected geometry_msgs/PoseStamped, got sequence",
        ),
        (
            "1: 2".to_string(),
            "(message): expected a field name, got integer",
        ),
    ];
    for (text, want) in cases {
        assert_eq!(decode_err(&walker, q, &text), want, "{text}");
    }
}

#[test]
fn every_bound_error_names_the_bound() {
    let set = set_from_yaml(SHAPES_YAML);
    let w = set.walker();
    let cases = [
        (SHAPES_TEXT.replace("tag: \"ab\"", "tag: \"abcde\""), "tag: string is 5 bytes, over the bound of 4 bytes"),
        (SHAPES_TEXT.replace("tag: \"ab\"", "tag: \"é€\""), "tag: string is 5 bytes, over the bound of 4 bytes"),
        (SHAPES_TEXT.replace("tag: \"ab\"", "tag: \"a\\0\""), "tag: a string_fixed value cannot contain a NUL character"),
        (SHAPES_TEXT.replace("flags: [true, false]", "flags: [true]"), "flags: expected exactly 2 elements, got 1"),
        (SHAPES_TEXT.replace("signed: [-1, 2]", "signed: [-129]"), "signed[0]: -129 is out of range for int8 (-128 to 127)"),
        (SHAPES_TEXT.replace("blob: [7]", "blob: [256]"), "blob[0]: 256 is out of range for uint8 (0 to 255)"),
        (SHAPES_TEXT.replace("more: [false, true, true]", "more: [1]"), "more[0]: expected bool, got integer"),
        (SHAPES_TEXT.replace("    y: 4.0", "    y: 1e39"), "corners[1].y: 1e39 is out of range for float32 (finite values up to 3.4028235e38 in magnitude)"),
        (SHAPES_TEXT.replace("  - x: 3.0\n    y: 4.0\n", ""), "corners: expected exactly 2 elements, got 1"),
    ];
    for (text, want) in cases {
        assert_eq!(decode_err(w, "Shapes", &text), want, "{text}");
    }
}

#[test]
fn unknown_schema_unresolved_nested_and_unsupported_types_are_named() {
    let set = set_from_yaml(PROBE_YAML);
    match yaml_to_message(set.walker(), "Nope", b"{}", TS) {
        Err(e) => assert_eq!(e, DynamicError::UnknownSchema("Nope".to_string())),
        Ok(_) => panic!("unknown schema must be refused"),
    }

    let (mut ghost, _) = SchemaSet::from_schemas(Vec::new()).unwrap();
    ghost
        .add_yaml_str("schemas:\n  Holder:\n    fields:\n      Ghost g: {}\n")
        .unwrap();
    assert_eq!(
        decode_err(ghost.walker(), "Holder", "g: {}"),
        "g: nested type Ghost is not in the schema set"
    );

    let schema = one_field_schema(
        "Grid",
        "rows",
        FieldType::DynamicArray {
            element_type: Box::new(FieldType::DynamicArray {
                element_type: Box::new(FieldType::U8),
            }),
        },
    );
    let (grid, _) = SchemaSet::from_schemas(vec![schema]).unwrap();
    assert_eq!(
        decode_err(grid.walker(), "Grid", "rows: []"),
        "rows: field type uint8[][] has no YAML form"
    );
    let square = one_field_schema(
        "Square",
        "m",
        FieldType::FixedArray {
            element_type: Box::new(FieldType::FixedArray {
                element_type: Box::new(FieldType::U8),
                length: 2,
            }),
            length: 2,
        },
    );
    let (square, _) = SchemaSet::from_schemas(vec![square]).unwrap();
    assert_eq!(
        decode_err(square.walker(), "Square", "m: [[1, 2], [3, 4]]"),
        "m: field type uint8[2][2] has no YAML form"
    );
    let mut fixed2d = header(square.schema_hash("Square").unwrap(), 36, 36, 0);
    fixed2d.extend_from_slice(&[1, 2, 3, 4]);
    match message_to_yaml(square.walker(), &fixed2d) {
        Err(e) => assert_eq!(e.to_string(), "m: field type uint8[2][2] has no YAML form"),
        Ok(t) => panic!("must be refused, got {t}"),
    }
    let mut frame = header(grid.schema_hash("Grid").unwrap(), 40, 32, 1);
    frame.extend_from_slice(&[8, 0, 0, 0, 0, 0, 0, 0]);
    match message_to_yaml(grid.walker(), &frame) {
        Err(e) => assert_eq!(e.to_string(), "rows: field type uint8[][] has no YAML form"),
        Ok(t) => panic!("must be refused, got {t}"),
    }
}

#[test]
fn frames_without_a_yaml_form_are_refused_by_field() {
    let set = set_from_yaml(PROBE_YAML);
    let mut bad_name = probe_frame(set.schema_hash("Probe").unwrap());
    bad_name[56] = 0xFF;
    let err = |set: &SchemaSet, frame: &[u8]| match message_to_yaml(set.walker(), frame) {
        Err(e) => e.to_string(),
        Ok(t) => panic!("must be refused, got {t}"),
    };
    assert_eq!(
        err(&set, &bad_name),
        "name: string field is not valid UTF-8; the frame has no YAML form"
    );

    let shapes = set_from_yaml(SHAPES_YAML);
    let bad_bool = shapes_frame(shapes.schema_hash("Shapes").unwrap(), [0, 2, 1]);
    assert_eq!(
        err(&shapes, &bad_bool),
        "more: bool byte 2 is neither 0 nor 1; the frame has no YAML form"
    );

    let names = set_from_yaml("schemas:\n  Names:\n    fields:\n      string[] n: {}\n");
    let mut opaque = header(names.schema_hash("Names").unwrap(), 44, 32, 1);
    opaque.extend_from_slice(&[8, 0, 0, 0, 4, 0, 0, 0, 1, 0, 0, 0]);
    assert_eq!(
        err(&names, &opaque),
        "n: array bytes are not in the canonical element framing; the frame has no YAML form"
    );

    match message_to_yaml(set.walker(), &[0u8; 8]) {
        Err(DynamicError::Walk(_)) => {}
        other => panic!("a short frame is a walk error, got {other:?}"),
    }
}

#[test]
fn length_prefix_past_u32_is_the_size_cap_arm() {
    match super::decode::len_prefix(u32::MAX as usize + 1, "data") {
        Err(e) => assert_eq!(
            e.to_string(),
            "data: the message is larger than the 1 GiB frame cap"
        ),
        Ok(_) => panic!("must be refused"),
    }
}

// ------------------------------------------------------------- corpus

/// One message of any schema, written twice from one running counter: as a
/// CDR body (for the independent [`CdrCodec`] to turn into a frame) and as
/// the YAML the module docs specify, spelled out here rule by rule with no
/// call into the codec. Values are chosen so their text is known without
/// formatting a float: an `f32` is `n + 0.5` (`"{n}.5"`), an `f64` is
/// `n + 0.25` (`"{n}.25"`), a string is `"s{n}"`.
struct Sample<'a> {
    walker: &'a FrameWalker,
    cdr: Cdr,
    yaml: String,
    n: u64,
}

impl Sample<'_> {
    fn pad(&mut self, width: usize) {
        self.yaml.extend(std::iter::repeat_n(' ', width));
    }

    /// The fields of `qname` as block-mapping lines at `indent`; `dash`
    /// starts the first line with the sequence `- ` (its fields then sit at
    /// `indent`, the dash two columns left).
    fn message(&mut self, qname: &str, indent: usize, dash: bool) {
        let schema = self.walker.schema_for_name(qname).unwrap().clone();
        assert!(!schema.fields.is_empty(), "callers write an empty {qname}");
        for (i, f) in schema.fields.iter().enumerate() {
            if dash && i == 0 {
                self.pad(indent - 2);
                self.yaml.push_str("- ");
            } else {
                self.pad(indent);
            }
            self.yaml.push_str(&f.name);
            self.yaml.push(':');
            self.value(&f.field_type, qname, indent);
        }
    }

    fn target(&self, ft: &FieldType, parent: &str) -> Option<(String, bool)> {
        let FieldType::Nested {
            schema_name,
            package,
            ..
        } = ft
        else {
            return None;
        };
        let q = self
            .walker
            .nested_layout(schema_name, package, parent)
            .unwrap()
            .qualified_name
            .clone();
        let empty = self.walker.schema_for_name(&q).unwrap().fields.is_empty();
        Some((q, empty))
    }

    /// One field value after its `key:`, to the end of its lines.
    fn value(&mut self, ft: &FieldType, parent: &str, indent: usize) {
        if let Some((q, empty)) = self.target(ft, parent) {
            if empty {
                self.cdr.u8(0);
                self.yaml.push_str(" {}\n");
            } else {
                self.yaml.push('\n');
                self.message(&q, indent + 2, false);
            }
            return;
        }
        let (element_type, count) = match ft {
            FieldType::DynamicArray { element_type } => {
                self.cdr.u32(2);
                (element_type.as_ref(), 2)
            }
            FieldType::FixedArray {
                element_type,
                length,
            } => (element_type.as_ref(), *length),
            scalar => {
                let text = self.scalar(scalar);
                self.yaml.push(' ');
                self.yaml.push_str(&text);
                self.yaml.push('\n');
                return;
            }
        };
        match self.target(element_type, parent) {
            Some(_) if count == 0 => self.yaml.push_str(" []\n"),
            Some((q, empty)) => {
                self.yaml.push('\n');
                for _ in 0..count {
                    if empty {
                        self.cdr.u8(0);
                        self.pad(indent + 2);
                        self.yaml.push_str("- {}\n");
                    } else {
                        self.message(&q, indent + 4, true);
                    }
                }
            }
            None => {
                let items: Vec<String> = (0..count).map(|_| self.scalar(element_type)).collect();
                self.yaml.push_str(&format!(" [{}]\n", items.join(", ")));
            }
        }
    }

    /// One primitive or string: its CDR bytes, and its YAML text returned.
    fn scalar(&mut self, ft: &FieldType) -> String {
        self.n += 1;
        let n = self.n;
        let c = &mut self.cdr;
        match ft {
            FieldType::Bool => {
                c.u8((n % 2) as u8);
                if n % 2 == 1 { "true" } else { "false" }.to_string()
            }
            FieldType::I8 | FieldType::U8 => {
                let v = (n % 251) as u8;
                c.u8(v);
                if matches!(ft, FieldType::I8) {
                    (v as i8).to_string()
                } else {
                    v.to_string()
                }
            }
            FieldType::I16 | FieldType::U16 => {
                let v = (n * 37) as u16;
                c.align(2);
                c.0.extend_from_slice(&v.to_le_bytes());
                if matches!(ft, FieldType::I16) {
                    (v as i16).to_string()
                } else {
                    v.to_string()
                }
            }
            FieldType::I32 | FieldType::U32 => {
                let v = (n * 7919) as u32;
                c.u32(v);
                if matches!(ft, FieldType::I32) {
                    (v as i32).to_string()
                } else {
                    v.to_string()
                }
            }
            FieldType::I64 | FieldType::U64 => {
                let v = n * 104_729;
                c.align(8);
                c.0.extend_from_slice(&v.to_le_bytes());
                v.to_string()
            }
            FieldType::F32 => {
                c.f32(n as f32 + 0.5);
                format!("{n}.5")
            }
            FieldType::F64 => {
                c.f64(n as f64 + 0.25);
                format!("{n}.25")
            }
            FieldType::String => {
                c.string(&format!("s{n}"));
                format!("\"s{n}\"")
            }
            other => panic!("no ROS corpus type uses {other:?}"),
        }
    }
}

/// `(qualified name, frame, expected YAML)` for one message of every type
/// in the vendored corpus.
fn corpus(walker: &FrameWalker, codec: &CdrCodec) -> Vec<(String, Vec<u8>, String)> {
    let mut out = Vec::new();
    for (pkg, name, _) in native_ros2_messages::BUILTIN_MSGS {
        let q = format!("{pkg}/{name}");
        if walker.layout_for_name(&q).is_none() {
            continue;
        }
        let mut s = Sample {
            walker,
            cdr: Cdr::default(),
            yaml: String::new(),
            n: 0,
        };
        if walker.schema_for_name(&q).unwrap().fields.is_empty() {
            s.cdr.u8(0);
            s.yaml.push_str("{}\n");
        } else {
            s.message(&q, 0, false);
        }
        let frame = codec
            .decode(&q, CdrEndianness::Little, &s.cdr.0, 0, TS)
            .unwrap_or_else(|e| panic!("{q}: CdrCodec refused the sample: {e}"));
        out.push((q, frame, s.yaml));
    }
    out
}

/// Every type in the vendored corpus, against an oracle on both sides: the
/// frame comes from [`CdrCodec`], the YAML from [`Sample`]'s own rules. The
/// encoder must print that YAML and the reader must build that frame, so a
/// wrong mapping the two directions shared would fail here, not cancel out.
#[test]
fn every_corpus_type_matches_the_oracle_both_ways() {
    let walker = builtin_walker();
    let (codec, _) = CdrCodec::new(builtin_schemas());
    let corpus = corpus(&walker, &codec);
    assert_eq!(
        corpus.len(),
        native_ros2_messages::BUILTIN_MSGS.len(),
        "every built-in type must be checked"
    );
    for (q, frame, yaml) in &corpus {
        assert_eq!(
            &message_to_yaml(&walker, frame).unwrap_or_else(|e| panic!("{q}: {e}")),
            yaml,
            "{q}: the encoder must print the oracle YAML"
        );
        let back = yaml_to_message(&walker, q, yaml.as_bytes(), TS)
            .unwrap_or_else(|e| panic!("{q}: the oracle YAML must decode: {e}\n{yaml}"));
        assert_eq!(
            &back, frame,
            "{q}: the reader must build the CdrCodec frame"
        );
    }
}

/// Two runs in one process, and a second process, give the same bytes.
#[test]
fn encoding_is_deterministic_across_runs_and_processes() {
    let walker = builtin_walker();
    let (codec, _) = CdrCodec::new(builtin_schemas());
    let frames = corpus(&walker, &codec);
    let digest = |walker: &FrameWalker| {
        frames
            .iter()
            .map(|(q, frame, _)| format!("== {q}\n{}", message_to_yaml(walker, frame).unwrap()))
            .collect::<String>()
    };
    let first = digest(&walker);
    assert_eq!(first, digest(&builtin_walker()));

    if std::env::var_os("CERULION_YAML_CODEC_CHILD").is_some() {
        print!("<<<{first}>>>");
        return;
    }
    let out = std::process::Command::new(std::env::current_exe().unwrap())
        .args([
            "--exact",
            "dynamic::message_yaml::tests::encoding_is_deterministic_across_runs_and_processes",
            "--nocapture",
            "--test-threads=1",
        ])
        .env("CERULION_YAML_CODEC_CHILD", "1")
        .output()
        .expect("spawn the test binary");
    let stdout = String::from_utf8(out.stdout).unwrap();
    let start = stdout.find("<<<").expect("child printed its digest") + 3;
    let end = stdout.rfind(">>>").unwrap();
    assert_eq!(
        &stdout[start..end],
        first,
        "a second process must print the same YAML"
    );
}
