// SPDX-License-Identifier: AGPL-3.0-only
//! Encoder for the JSON publisher input surface.
//!
//! Publish reads JSON with the message YAML reader of
//! [`cerulion_core::dynamic`] (JSON is YAML), the reader `cerulion` uses for
//! a message typed as text, so the app and the CLI share one set of encode
//! rules, nested messages and arrays included. The one difference is that a
//! field the JSON leaves out is written as its zero value. Like that reader,
//! publish allocates per message: it is the app's convenience path, never
//! the zero-copy path a node publishes on.
use std::borrow::Cow;
use std::fmt::Write as _;

use cerulion_core::codegen::FieldType;
use cerulion_core::dynamic::{yaml_to_message_with, FrameWalker, MissingFields, YamlDecodeOptions};
use cerulion_core::wire::WireHeader;

use crate::error::EmbedError;

const JSON_BYTES_PER_PAYLOAD_BYTE: usize = 8;
const JSON_FIXED_OVERHEAD_BYTES: usize = 65_536;

/// The whitespace JSON allows around a value.
const JSON_WHITESPACE: [char; 4] = [' ', '\t', '\n', '\r'];

/// Byte offset of `sequence` in the wire header (see [`WireHeader`]).
const SEQUENCE_OFFSET: usize = 20;

/// Encode a JSON object as one message of `schema` (a qualified name
/// `walker` knows) into a wire frame stamped `timestamp_ns` and numbered
/// `sequence`.
///
/// The JSON text is refused before the message reader runs when it is
/// longer than `max_payload_bytes * 8 + 65536` bytes, is not an object or
/// is not strict JSON (`invalid publish JSON: ...`, a number no `float64`
/// holds included). The reader refuses as soon as the body it builds holds
/// more than `max_payload_bytes`, and the finished payload, alignment
/// padding included, is checked against it again.
pub fn encode_json(
    walker: &FrameWalker,
    schema: &str,
    json: &str,
    max_payload_bytes: usize,
    timestamp_ns: u64,
    sequence: u32,
) -> Result<Vec<u8>, EmbedError> {
    // Bound escaped payload values plus field names and whitespace before parsing.
    let json_limit = max_publish_json_len(max_payload_bytes);
    if json.len() > json_limit {
        return Err(EmbedError::Encode(format!(
            "publish JSON length {} exceeds the limit {} for max_payload_bytes {}",
            json.len(),
            json_limit,
            max_payload_bytes
        )));
    }
    // The YAML reader takes empty text, `null` and block YAML as a message
    // too; publish takes a JSON object only. The whitespace around it is
    // dropped, since YAML refuses a tab that starts a line outside braces.
    let object = json.trim_matches(JSON_WHITESPACE);
    if !object.starts_with('{') {
        return Err(EmbedError::Encode(
            "publish JSON must be an object".to_string(),
        ));
    }
    // The reader takes YAML-only forms too (`.nan`, comments, unquoted
    // text); publish takes JSON only.
    serde_json::from_str::<StrictJson>(object)
        .map_err(|error| EmbedError::Encode(format!("invalid publish JSON: {error}")))?;
    let object = yaml_safe_json(object);
    let options = YamlDecodeOptions {
        missing_fields: MissingFields::Zero,
        max_body_bytes: Some(max_payload_bytes),
    };
    let mut frame = yaml_to_message_with(walker, schema, object.as_bytes(), timestamp_ns, options)
        .map_err(|error| EmbedError::Encode(error.to_string()))?;
    let payload_len = frame.len().saturating_sub(WireHeader::SIZE);
    if payload_len > max_payload_bytes {
        return Err(EmbedError::Encode(format!(
            "encoded payload length {payload_len} exceeds max_payload_bytes {max_payload_bytes}"
        )));
    }
    // The reader writes sequence 0; the app's publisher numbers its frames.
    frame[SEQUENCE_OFFSET..SEQUENCE_OFFSET + 4].copy_from_slice(&sequence.to_le_bytes());
    Ok(frame)
}

/// A JSON value that is read and dropped, like [`serde::de::IgnoredAny`],
/// except that every number is parsed: `serde_json` then refuses a number
/// no `float64` holds (`1e999`), which YAML would read as a string. It
/// keeps nothing, so a check allocates no value tree.
struct StrictJson;

impl<'de> serde::Deserialize<'de> for StrictJson {
    fn deserialize<D: serde::Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        deserializer.deserialize_any(StrictJsonVisitor)
    }
}

struct StrictJsonVisitor;

impl<'de> serde::de::Visitor<'de> for StrictJsonVisitor {
    type Value = StrictJson;

    fn expecting(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        formatter.write_str("a JSON value")
    }

    fn visit_bool<E>(self, _: bool) -> Result<StrictJson, E> {
        Ok(StrictJson)
    }

    fn visit_i64<E>(self, _: i64) -> Result<StrictJson, E> {
        Ok(StrictJson)
    }

    fn visit_u64<E>(self, _: u64) -> Result<StrictJson, E> {
        Ok(StrictJson)
    }

    fn visit_f64<E>(self, _: f64) -> Result<StrictJson, E> {
        Ok(StrictJson)
    }

    fn visit_str<E>(self, _: &str) -> Result<StrictJson, E> {
        Ok(StrictJson)
    }

    fn visit_unit<E>(self) -> Result<StrictJson, E> {
        Ok(StrictJson)
    }

    fn visit_seq<A: serde::de::SeqAccess<'de>>(self, mut seq: A) -> Result<StrictJson, A::Error> {
        while seq.next_element::<StrictJson>()?.is_some() {}
        Ok(StrictJson)
    }

    fn visit_map<A: serde::de::MapAccess<'de>>(self, mut map: A) -> Result<StrictJson, A::Error> {
        while map
            .next_entry::<serde::de::IgnoredAny, StrictJson>()?
            .is_some()
        {}
        Ok(StrictJson)
    }
}

/// `object` (valid JSON) rewritten so the YAML reader reads every string
/// as the JSON does. YAML reads NEL, LS and PS as line breaks and folds them
/// into a space, and it refuses DEL, the C1 controls, a byte order mark,
/// U+FFFE and U+FFFF, which JSON allows raw in a string: each is written as
/// its `\uXXXX` escape. YAML also refuses a `\uXXXX` escape of a surrogate,
/// so an escaped surrogate pair is written as the character it spells. JSON
/// allows none of these outside a string, and escapes there are well formed,
/// so the rewrite never changes a value.
fn yaml_safe_json(object: &str) -> Cow<'_, str> {
    let not_kept_raw = |c: char| {
        matches!(
            c,
            '\u{7f}'..='\u{9f}' | '\u{2028}' | '\u{2029}' | '\u{feff}' | '\u{fffe}' | '\u{ffff}'
        )
    };
    if !object.contains(not_kept_raw) && !object.contains("\\u") {
        return Cow::Borrowed(object);
    }
    let mut out = String::with_capacity(object.len());
    let mut in_string = false;
    let mut rest = object;
    while let Some(c) = rest.chars().next() {
        let mut used = c.len_utf8();
        if not_kept_raw(c) {
            // Writing to a String cannot fail.
            let _ = write!(out, "\\u{:04X}", u32::from(c));
        } else if in_string && c == '\\' {
            if let Some(pair) = surrogate_pair(rest) {
                out.push(pair);
                used = SURROGATE_PAIR_ESCAPE_LEN;
            } else {
                // The backslash and the ASCII character it escapes, so an
                // escaped quote does not end the string.
                used = 2;
                out.push_str(&rest[..used]);
            }
        } else {
            if c == '"' {
                in_string = !in_string;
            }
            out.push(c);
        }
        rest = &rest[used..];
    }
    Cow::Owned(out)
}

/// The length of an escaped surrogate pair: two six-byte `\uXXXX` escapes.
const SURROGATE_PAIR_ESCAPE_LEN: usize = 12;

/// The character an escaped surrogate pair at the start of `escape` spells,
/// or `None` when `escape` does not start with one.
fn surrogate_pair(escape: &str) -> Option<char> {
    let unit = |at: usize| {
        let hex = escape.get(at..at + 6)?.strip_prefix("\\u")?;
        u16::from_str_radix(hex, 16).ok().map(u32::from)
    };
    let high = unit(0).filter(|unit| (0xD800..0xDC00).contains(unit))?;
    let low = unit(6).filter(|unit| (0xDC00..0xE000).contains(unit))?;
    char::from_u32(0x10000 + ((high - 0xD800) << 10) + (low - 0xDC00))
}

pub(crate) fn max_publish_json_len(max_payload_bytes: usize) -> usize {
    max_payload_bytes
        .saturating_mul(JSON_BYTES_PER_PAYLOAD_BYTE)
        .saturating_add(JSON_FIXED_OVERHEAD_BYTES)
}

/// The field types poll reads: the primitives, `String`, `StringFixed(n)`
/// and byte arrays.
pub(crate) fn is_flat(field_type: &FieldType) -> bool {
    matches!(
        field_type,
        FieldType::Bool
            | FieldType::I8
            | FieldType::U8
            | FieldType::I16
            | FieldType::U16
            | FieldType::I32
            | FieldType::U32
            | FieldType::I64
            | FieldType::U64
            | FieldType::F32
            | FieldType::F64
            | FieldType::String
            | FieldType::StringFixed(_)
    ) || is_byte_array(field_type)
}

pub(crate) fn is_byte_array(field_type: &FieldType) -> bool {
    matches!(field_type, FieldType::Bytes)
        || matches!(
            field_type,
            FieldType::DynamicArray { element_type }
                if matches!(element_type.as_ref(), FieldType::U8)
        )
}

#[cfg(test)]
mod tests {
    use super::*;
    use cerulion_cli_engine::schema_cmd::parse_builtin_schemas;
    use cerulion_core::codegen::parse_rosmsg;
    use cerulion_core::dynamic::SchemaSet;

    /// A schema set holding one `.msg` schema, `package/name`.
    fn layout(text: &str, name: &str, package: &str) -> SchemaSet {
        let schema = parse_rosmsg(text, name, Some(package)).unwrap();
        SchemaSet::from_schemas(vec![schema]).unwrap().0
    }

    fn builtins() -> SchemaSet {
        SchemaSet::from_schemas(parse_builtin_schemas()).unwrap().0
    }

    fn fixed_name() -> SchemaSet {
        let (mut set, _) = SchemaSet::from_schemas(Vec::new()).unwrap();
        set.add_yaml_str("schemas:\n  FixedName:\n    fields:\n      string_fixed[4] name: {}\n")
            .unwrap();
        set
    }

    #[test]
    fn encodes_string_to_hand_written_frame() {
        let layout = layout("string data\n", "String", "std_msgs");
        let frame = encode_json(
            layout.walker(),
            "std_msgs/String",
            r#"{"data":"hello"}"#,
            64,
            7,
            42,
        )
        .unwrap();
        assert_eq!(frame.len(), WireHeader::SIZE + 13);
        assert_eq!(&frame[20..24], &[42, 0, 0, 0]);
        assert_eq!(
            &frame[WireHeader::SIZE..],
            &[8, 0, 0, 0, 5, 0, 0, 0, b'h', b'e', b'l', b'l', b'o']
        );
    }

    #[test]
    fn rejects_unknown_fields_and_bad_json_values() {
        let layout = layout("string data\n", "String", "std_msgs");
        let encode = |json| encode_json(layout.walker(), "std_msgs/String", json, 64, 0, 0);
        assert!(encode(r#"{"unknown":"x"}"#).is_err());
        assert!(encode(r#"[1]"#).is_err());
        assert!(encode(r#"{"data":1}"#).is_err());
    }

    #[test]
    fn encodes_fixed_vector_with_oracle_bytes() {
        let layout = layout(
            "float64 x\nfloat64 y\nfloat64 z\n",
            "Vector3",
            "geometry_msgs",
        );
        let frame = encode_json(
            layout.walker(),
            "geometry_msgs/Vector3",
            r#"{"x":1.5,"y":-2.0}"#,
            64,
            0,
            0,
        )
        .unwrap();
        let expected = [
            0, 0, 0, 0, 0, 0, 248, 63, 0, 0, 0, 0, 0, 0, 0, 192, 0, 0, 0, 0, 0, 0, 0, 0,
        ];
        assert_eq!(&frame[WireHeader::SIZE..], &expected);
    }

    #[test]
    fn rejects_payload_limit_and_unsupported_type() {
        let string = layout("string data\n", "String", "std_msgs");
        assert!(encode_json(
            string.walker(),
            "std_msgs/String",
            r#"{"data":"large"}"#,
            1,
            0,
            0
        )
        .is_err());
        // A nested type missing from the schema set is refused at its field.
        let nested = layout("geometry_msgs/Point point\n", "Pose", "geometry_msgs");
        let error = encode_json(nested.walker(), "geometry_msgs/Pose", r#"{}"#, 64, 0, 0)
            .expect_err("Point is not in the set");
        assert_eq!(
            error.to_string(),
            "point: nested type geometry_msgs/Point is not in the schema set"
        );
    }

    #[test]
    fn over_limit_string_fails_at_budget_boundary() {
        let string = layout("string data\n", "String", "std_msgs");
        // The frame carries an 8-byte offset/length table before the
        // payload, so a budget of 8 + N admits exactly N string bytes.
        let over = format!(r#"{{"data":"{}"}}"#, "x".repeat(4097));
        let error = encode_json(string.walker(), "std_msgs/String", &over, 8 + 4096, 0, 0)
            .expect_err("over the limit");
        assert_eq!(error.code(), crate::error::codes::ENCODE);
        assert_eq!(
            error.to_string(),
            "data: the message body is larger than the 4104-byte limit"
        );
        let exact = format!(r#"{{"data":"{}"}}"#, "x".repeat(4096));
        let frame = encode_json(string.walker(), "std_msgs/String", &exact, 8 + 4096, 0, 0)
            .expect("at the limit");
        assert_eq!(frame.len(), WireHeader::SIZE + 8 + 4096);
    }

    /// The reader counts no alignment padding, so a payload it lets through
    /// can still pass the limit; the finished length is checked again.
    /// `string s` + `float64[] v`: table 16 B, `s` @16 (1 B), `v` aligned up
    /// to @24 (8 B), 32 B in all, of which the reader counts 25.
    #[test]
    fn a_payload_past_the_limit_by_its_padding_is_refused() {
        let schema = layout("string s\nfloat64[] v\n", "Padded", "test_msgs");
        let json = r#"{"s":"a","v":[1.0]}"#;
        let encode = |max| encode_json(schema.walker(), "test_msgs/Padded", json, max, 0, 0);
        assert_eq!(encode(32).unwrap().len(), WireHeader::SIZE + 32);
        assert_eq!(
            encode(31).unwrap_err().to_string(),
            "encoded payload length 32 exceeds max_payload_bytes 31"
        );
    }

    /// A few bytes of JSON can name many left-out messages: the reader stops
    /// at the first one past the payload limit instead of building them all.
    /// `PoseArray`: table 16 B, header body 16 B, then 56 B per pose, so
    /// 4096 B hold 72 poses (4064 B) and the 73rd is refused.
    #[test]
    fn many_left_out_messages_are_refused_at_the_payload_limit() {
        let set = builtins();
        let poses = |n: usize| format!(r#"{{"poses":[{}]}}"#, vec!["{}"; n].join(","));
        let encode = |n| {
            encode_json(
                set.walker(),
                "geometry_msgs/PoseArray",
                &poses(n),
                4096,
                0,
                0,
            )
        };
        assert_eq!(encode(72).unwrap().len(), WireHeader::SIZE + 4064);
        assert_eq!(
            encode(30_000).unwrap_err().to_string(),
            "poses[72]: the message body is larger than the 4096-byte limit"
        );
    }

    #[test]
    fn rejects_json_over_budget_before_parsing_and_encodes_maximum_string() {
        let string = layout("string data\n", "String", "std_msgs");
        let max_payload_bytes = 4096;
        let input_limit = max_publish_json_len(max_payload_bytes);
        let oversized_invalid_json = "x".repeat(input_limit + 1);
        let error = encode_json(
            string.walker(),
            "std_msgs/String",
            &oversized_invalid_json,
            max_payload_bytes,
            0,
            0,
        )
        .expect_err("oversized JSON must be refused before parsing");
        assert_eq!(
            error.to_string(),
            format!(
                "publish JSON length {} exceeds the limit {} for max_payload_bytes {}",
                input_limit + 1,
                input_limit,
                max_payload_bytes
            )
        );

        let max_string = "x".repeat(max_payload_bytes - 8);
        let json = format!(r#"{{"data":"{max_string}"}}"#);
        assert!(json.len() <= input_limit);
        let frame = encode_json(
            string.walker(),
            "std_msgs/String",
            &json,
            max_payload_bytes,
            0,
            0,
        )
        .expect("maximum-size String fits the JSON input budget");
        assert_eq!(frame.len(), WireHeader::SIZE + max_payload_bytes);
    }

    #[test]
    fn rejects_integer_float_and_bounded_string_errors() {
        let integer = layout("int8 value\n", "Int8", "test_msgs");
        let int8 = |json| encode_json(integer.walker(), "test_msgs/Int8", json, 64, 0, 0);
        assert!(int8(r#"{"value":1.5}"#).is_err());
        assert!(int8(r#"{"value":128}"#).is_err());
        let float = layout("float32 value\n", "Float32", "test_msgs");
        assert!(encode_json(
            float.walker(),
            "test_msgs/Float32",
            r#"{"value":1e300}"#,
            64,
            0,
            0
        )
        .is_err());
        let fixed = fixed_name();
        let name = |json| encode_json(fixed.walker(), "FixedName", json, 64, 0, 0);
        let error = name(r#"{"name":"a\u0000b"}"#)
            .expect_err("a string_fixed value cannot carry an embedded NUL");
        assert_eq!(
            error.to_string(),
            "name: a string_fixed value cannot contain a NUL character"
        );
        assert!(name(r#"{"name":"four"}"#).is_ok());
        assert!(name(r#"{"name":"longer"}"#).is_err());
    }

    #[test]
    fn encodes_bytes_and_rejects_invalid_bytes_and_booleans() {
        let bytes = layout("uint8[] data\n", "Bytes", "test_msgs");
        let encode = |json| encode_json(bytes.walker(), "test_msgs/Bytes", json, 64, 0, 0);
        let frame = encode(r#"{"data":[1,2,255]}"#).unwrap();
        assert_eq!(
            &frame[WireHeader::SIZE..],
            &[8, 0, 0, 0, 3, 0, 0, 0, 1, 2, 255]
        );
        assert!(encode(r#"{"data":[256]}"#).is_err());
        let boolean = layout("bool value\n", "Boolean", "test_msgs");
        assert!(encode_json(
            boolean.walker(),
            "test_msgs/Boolean",
            r#"{"value":"true"}"#,
            64,
            0,
            0
        )
        .is_err());
    }

    #[test]
    fn output_is_deterministic_except_timestamp() {
        let layout = layout("string data\n", "String", "std_msgs");
        let encode = |timestamp| {
            encode_json(
                layout.walker(),
                "std_msgs/String",
                r#"{"data":"same"}"#,
                64,
                timestamp,
                17,
            )
            .unwrap()
        };
        let first = encode(123);
        let second = encode(456);
        assert_eq!(&first[..24], &second[..24]);
        assert_eq!(&first[32..], &second[32..]);
    }

    /// The header of a frame `total` bytes long with a fixed section of
    /// `fixed` bytes and `count` variable fields, numbered `sequence` and
    /// stamped `stamp`.
    fn header(hash: u64, total: u32, fixed: u32, count: u32, sequence: u32, stamp: u64) -> Vec<u8> {
        let mut bytes = hash.to_le_bytes().to_vec();
        for value in [total, 40 + fixed, count, sequence] {
            bytes.extend_from_slice(&value.to_le_bytes());
        }
        bytes.extend_from_slice(&stamp.to_le_bytes());
        bytes.extend_from_slice(&[0; 8]);
        bytes
    }

    /// `Twist` is two fixed `Vector3`s, six `float64`s back to back with no
    /// offset table; the fields the JSON leaves out are zero.
    #[test]
    fn encodes_a_nested_twist_to_hand_written_bytes() {
        let set = builtins();
        let frame = encode_json(
            set.walker(),
            "geometry_msgs/Twist",
            r#"{"linear":{"x":1.0,"y":-2.5},"angular":{"z":0.5}}"#,
            64,
            9,
            3,
        )
        .unwrap();
        let hash = set.schema_hash("geometry_msgs/Twist").unwrap();
        let mut want = header(hash, 88, 48, 0, 3, 9);
        want.extend_from_slice(&[0, 0, 0, 0, 0, 0, 0xF0, 0x3F]); // linear.x 1.0
        want.extend_from_slice(&[0, 0, 0, 0, 0, 0, 0x04, 0xC0]); // linear.y -2.5
        want.extend_from_slice(&[0; 8]); // linear.z
        want.extend_from_slice(&[0; 16]); // angular.x, angular.y
        want.extend_from_slice(&[0, 0, 0, 0, 0, 0, 0xE0, 0x3F]); // angular.z 0.5
        assert_eq!(frame, want);
    }

    /// `PoseStamped`: the fixed `pose` (seven `float64`s) is the fixed
    /// section, and `header` is its one variable field, a headerless body of
    /// its own: `stamp` (two 32-bit integers), an offset table row for
    /// `frame_id`, then the text.
    #[test]
    fn encodes_a_pose_stamped_with_a_header_to_hand_written_bytes() {
        let set = builtins();
        let json = r#"{"header":{"stamp":{"sec":5,"nanosec":7},"frame_id":"map"},"pose":{"position":{"x":1.5},"orientation":{"w":1.0}}}"#;
        let frame =
            encode_json(set.walker(), "geometry_msgs/PoseStamped", json, 128, 11, 2).unwrap();
        let hash = set.schema_hash("geometry_msgs/PoseStamped").unwrap();
        let mut want = header(hash, 123, 56, 1, 2, 11);
        want.extend_from_slice(&[0, 0, 0, 0, 0, 0, 0xF8, 0x3F]); // position.x 1.5
        want.extend_from_slice(&[0; 40]); // position.y, z, orientation.x, y, z
        want.extend_from_slice(&[0, 0, 0, 0, 0, 0, 0xF0, 0x3F]); // orientation.w 1.0
        want.extend_from_slice(&[64, 0, 0, 0, 19, 0, 0, 0]); // header @64, 19 B
        want.extend_from_slice(&[5, 0, 0, 0, 7, 0, 0, 0]); // stamp
        want.extend_from_slice(&[16, 0, 0, 0, 3, 0, 0, 0]); // frame_id @16, 3 B
        want.extend_from_slice(b"map");
        assert_eq!(frame, want);
    }

    #[test]
    fn a_bad_nested_field_names_its_path() {
        let set = builtins();
        let encode = |json| encode_json(set.walker(), "geometry_msgs/PoseStamped", json, 128, 0, 0);
        let error = encode(r#"{"pose":{"position":{"x":"a"}}}"#).unwrap_err();
        assert_eq!(error.code(), crate::error::codes::ENCODE);
        assert_eq!(
            error.to_string(),
            "pose.position.x: expected float64, got string"
        );
        assert_eq!(
            encode(r#"{"header":{"stamp":{"nanosec":-1}}}"#)
                .unwrap_err()
                .to_string(),
            "header.stamp.nanosec: -1 is out of range for uint32 (0 to 4294967295)"
        );
        assert_eq!(
            encode(r#"{"header":{"frame":"map"}}"#)
                .unwrap_err()
                .to_string(),
            "header.frame: unknown field; std_msgs/Header has fields: stamp, frame_id"
        );
    }

    /// The reader takes empty text, `null` and block YAML as a message;
    /// publish refuses anything but a JSON object before the reader runs,
    /// and text after the object as invalid JSON.
    #[test]
    fn refuses_a_value_that_is_not_an_object_before_parsing() {
        let set = builtins();
        for json in ["", "  ", "null", "[1]", "data: x", "\"x\""] {
            let error =
                encode_json(set.walker(), "std_msgs/String", json, 64, 0, 0).expect_err(json);
            assert_eq!(error.code(), crate::error::codes::ENCODE, "{json:?}");
            assert_eq!(
                error.to_string(),
                "publish JSON must be an object",
                "{json:?}"
            );
        }
        for json in [
            "{\"data\":\"a\"} trailing",
            "{\"data\":\"a\",\"data\":\"b\"}",
        ] {
            let error =
                encode_json(set.walker(), "std_msgs/String", json, 64, 0, 0).expect_err(json);
            assert_eq!(error.code(), crate::error::codes::ENCODE, "{json:?}");
            assert!(
                !error.to_string().contains("must be an object"),
                "{json:?}: {error}"
            );
        }
        assert!(encode_json(
            set.walker(),
            "std_msgs/String",
            "{\"data\":\"a\"} trailing",
            64,
            0,
            0
        )
        .unwrap_err()
        .to_string()
        .starts_with("invalid publish JSON: "));
        let frame = encode_json(
            set.walker(),
            "std_msgs/String",
            " \n\t{\"data\":\"a\"}\r\n",
            64,
            0,
            0,
        )
        .expect("JSON whitespace around the object");
        assert_eq!(&frame[WireHeader::SIZE..], &[8, 0, 0, 0, 1, 0, 0, 0, b'a']);
        let pretty = "\t{\n\t\t\"data\": \"a\"\n\t}\n\t";
        let frame = encode_json(set.walker(), "std_msgs/String", pretty, 64, 0, 0)
            .expect("JSON indented with tabs");
        assert_eq!(&frame[WireHeader::SIZE..], &[8, 0, 0, 0, 1, 0, 0, 0, b'a']);
    }

    #[test]
    fn a_nested_message_encodes_the_same_bytes_every_time() {
        let set = builtins();
        let json = r#"{"header":{"frame_id":"odom"},"pose":{"position":{"y":-1.25}}}"#;
        let encode =
            || encode_json(set.walker(), "geometry_msgs/PoseStamped", json, 128, 5, 1).unwrap();
        assert_eq!(encode(), encode());
    }

    /// Publish takes JSON, not the wider YAML the reader knows: a YAML-only
    /// form is refused as invalid JSON, so a `.nan` never reaches a topic.
    #[test]
    fn refuses_yaml_forms_that_are_not_json() {
        let set = builtins();
        for json in [
            r#"{"x": .nan}"#,
            r#"{"x": .inf}"#,
            r#"{"x": -.inf}"#,
            r#"{"x": 0x10}"#,
            r#"{x: 1.0}"#,
            r#"{'x': 1.0}"#,
            "{\"x\": 1.0} # note",
            "{\"x\": 1.0}\n---",
        ] {
            let error =
                encode_json(set.walker(), "geometry_msgs/Vector3", json, 64, 0, 0).expect_err(json);
            assert_eq!(error.code(), crate::error::codes::ENCODE, "{json:?}");
            assert!(
                error.to_string().starts_with("invalid publish JSON: "),
                "{json:?}: {error}"
            );
        }
        // A number no float64 holds would read as a string in YAML, so a
        // string field would take its digits: it is refused as JSON.
        let huge_integer = format!(r#"{{"data": 1{}}}"#, "0".repeat(400));
        for (schema, json) in [
            ("geometry_msgs/Vector3", r#"{"x": 1e999}"#),
            ("std_msgs/String", r#"{"data": 1e999}"#),
            ("std_msgs/String", r#"{"data": -1e999}"#),
            ("std_msgs/String", huge_integer.as_str()),
        ] {
            let error = encode_json(set.walker(), schema, json, 1024, 0, 0).expect_err(json);
            assert!(
                error.to_string().starts_with("invalid publish JSON: "),
                "{json}: {error}"
            );
        }
    }

    /// JSON lets a string hold these characters raw, but YAML reads NEL, LS
    /// and PS as line breaks and refuses the rest: each reaches the frame
    /// as the same UTF-8 bytes the JSON held.
    #[test]
    fn a_string_keeps_characters_yaml_reads_as_breaks_or_refuses() {
        let set = builtins();
        for c in [
            '\u{7f}', '\u{80}', '\u{85}', '\u{9f}', '\u{2028}', '\u{2029}', '\u{feff}', '\u{fffe}',
            '\u{ffff}',
        ] {
            let text = format!("a{c}b");
            let json = format!(r#"{{"data":"{text}"}}"#);
            let frame = encode_json(set.walker(), "std_msgs/String", &json, 64, 0, 0)
                .unwrap_or_else(|error| panic!("U+{:04X}: {error}", c as u32));
            let mut want = vec![8, 0, 0, 0, text.len() as u8, 0, 0, 0];
            want.extend_from_slice(text.as_bytes());
            assert_eq!(&frame[WireHeader::SIZE..], &want[..], "U+{:04X}", c as u32);
        }
        // An escape that spells such a character keeps it too.
        let frame = encode_json(
            set.walker(),
            "std_msgs/String",
            r#"{"data":"\u0085\\u2028"}"#,
            64,
            0,
            0,
        )
        .unwrap();
        let mut want = vec![8, 0, 0, 0, 8, 0, 0, 0, 0xC2, 0x85];
        want.extend_from_slice(b"\\u2028");
        assert_eq!(&frame[WireHeader::SIZE..], &want[..]);
    }

    /// A character past U+FFFF, sent as UTF-8 or as an escaped surrogate
    /// pair, reaches the frame as its UTF-8 bytes; a lone escaped surrogate
    /// spells no character and is refused.
    #[test]
    fn a_character_past_the_basic_plane_publishes_raw_or_escaped() {
        let set = builtins();
        let encode = |json| encode_json(set.walker(), "std_msgs/String", json, 64, 0, 0);
        for json in ["{\"data\":\"\u{1F600}\"}", "{\"data\":\"\\ud83d\\ude00\"}"] {
            let frame = encode(json).unwrap_or_else(|error| panic!("{json}: {error}"));
            assert_eq!(
                &frame[WireHeader::SIZE..],
                &[8, 0, 0, 0, 4, 0, 0, 0, 0xF0, 0x9F, 0x98, 0x80],
                "{json}"
            );
        }
        let error = encode("{\"data\":\"\\ud83d\"}").unwrap_err();
        assert_eq!(error.code(), crate::error::codes::ENCODE);
        assert!(
            error.to_string().starts_with("invalid publish JSON: "),
            "{error}"
        );
        // An escaped backslash before `ud83d` is text, not an escape.
        let frame = encode(r#"{"data":"\\ud83d\\ude00"}"#).unwrap();
        let mut want = vec![8, 0, 0, 0, 12, 0, 0, 0];
        want.extend_from_slice(br"\ud83d\ude00");
        assert_eq!(&frame[WireHeader::SIZE..], &want[..]);
        // An escaped quote or backslash does not end the string or start an
        // escape, so the pair after it is still read as one character.
        for (json, first) in [
            (r#"{"data":"\"\ud83d\ude00"}"#, b'"'),
            (r#"{"data":"\\\ud83d\ude00"}"#, b'\\'),
        ] {
            let frame = encode(json).unwrap_or_else(|error| panic!("{json}: {error}"));
            assert_eq!(
                &frame[WireHeader::SIZE..],
                &[8, 0, 0, 0, 5, 0, 0, 0, first, 0xF0, 0x9F, 0x98, 0x80],
                "{json}"
            );
        }
        // A surrogate escape that is not half of a pair spells no character.
        for json in [
            r#"{"data":"\ud83dA"}"#,
            r#"{"data":"\ud83d\u0041"}"#,
            r#"{"data":"\ud83d\ud83d"}"#,
            r#"{"data":"\ude00"}"#,
        ] {
            let error = encode(json).unwrap_err();
            assert_eq!(error.code(), crate::error::codes::ENCODE, "{json}");
            assert!(
                error.to_string().starts_with("invalid publish JSON: "),
                "{json}: {error}"
            );
        }
    }

    /// The YAML reader reads a bare `-0` as the integer 0, so a float field
    /// gets positive zero; `-0.0` keeps the sign.
    #[test]
    fn a_bare_minus_zero_is_positive_zero_and_minus_zero_point_zero_keeps_its_sign() {
        let set = builtins();
        let x = |json| {
            let frame = encode_json(set.walker(), "geometry_msgs/Vector3", json, 64, 0, 0).unwrap();
            frame[WireHeader::SIZE..WireHeader::SIZE + 8].to_vec()
        };
        assert_eq!(x(r#"{"x":-0}"#), [0; 8]);
        assert_eq!(x(r#"{"x":-0.0}"#), [0, 0, 0, 0, 0, 0, 0, 0x80]);
    }

    /// Text of exactly the JSON length limit is read; one byte more is not.
    #[test]
    fn the_json_length_limit_admits_its_own_length() {
        let set = builtins();
        let max_payload_bytes = 16;
        let limit = max_publish_json_len(max_payload_bytes);
        let object = r#"{"data":"a"}"#;
        let at_limit = format!("{object}{}", " ".repeat(limit - object.len()));
        assert_eq!(at_limit.len(), limit);
        assert!(encode_json(
            set.walker(),
            "std_msgs/String",
            &at_limit,
            max_payload_bytes,
            0,
            0
        )
        .is_ok());
        let past = format!("{at_limit} ");
        let error = encode_json(
            set.walker(),
            "std_msgs/String",
            &past,
            max_payload_bytes,
            0,
            0,
        )
        .unwrap_err();
        assert_eq!(
            error.to_string(),
            format!(
                "publish JSON length {} exceeds the limit {limit} for max_payload_bytes 16",
                limit + 1
            )
        );
    }
}
