// SPDX-License-Identifier: AGPL-3.0-only
//! Decoder for the JSON poll output surface, the counterpart of
//! [`crate::encode`]. It reads the same flat field types publish accepts.
use cerulion_core::codegen::FieldType;
use cerulion_core::dynamic::{FrameView, WireLayout};

use crate::error::EmbedError;

/// Decode one wire frame (header first) into JSON object text in `out`.
///
/// `out` is cleared and reused, so a per-subscription buffer keeps its
/// capacity across polls.
pub fn decode_json_into(
    layout: &WireLayout,
    frame: &[u8],
    out: &mut Vec<u8>,
) -> Result<(), EmbedError> {
    let view = FrameView::with_layout(layout, frame)
        .map_err(|error| EmbedError::Decode(error.to_string()))?;
    out.clear();
    out.push(b'{');
    let mut first = true;
    let mut key = |out: &mut Vec<u8>, name: &str| -> Result<(), EmbedError> {
        if !std::mem::take(&mut first) {
            out.push(b',');
        }
        write_json(out, name)?;
        out.push(b':');
        Ok(())
    };
    for (index, field) in layout.fixed_fields.iter().enumerate() {
        key(out, &field.name)?;
        let bytes = view
            .fixed_field_at(index)
            .map_err(|error| EmbedError::Decode(error.to_string()))?;
        write_fixed(out, bytes, &field.name, &field.field_type)?;
    }
    for (index, field) in layout.variable_fields.iter().enumerate() {
        key(out, &field.name)?;
        let bytes = view
            .variable_field_at(index)
            .map_err(|error| EmbedError::Decode(error.to_string()))?;
        match &field.field_type {
            FieldType::String => write_json(out, utf8(bytes, &field.name)?)?,
            ty if crate::encode::is_byte_array(ty) => write_json(out, bytes)?,
            other => return Err(unsupported(&field.name, other)),
        }
    }
    out.push(b'}');
    Ok(())
}

fn write_fixed(
    out: &mut Vec<u8>,
    bytes: &[u8],
    field_name: &str,
    field_type: &FieldType,
) -> Result<(), EmbedError> {
    // `FrameView` checked each fixed field's span against the layout, and
    // the layout's width per type, so each `try_into` below is exact.
    let width = |n: usize| -> Result<(), EmbedError> {
        if bytes.len() == n {
            Ok(())
        } else {
            Err(EmbedError::Decode(format!(
                "field '{field_name}' layout width {} does not match {field_type:?} width {n}",
                bytes.len()
            )))
        }
    };
    macro_rules! number {
        ($ty:ty, $n:expr) => {{
            width($n)?;
            let mut raw = [0u8; $n];
            raw.copy_from_slice(bytes);
            write_json(out, <$ty>::from_le_bytes(raw))
        }};
    }
    match field_type {
        FieldType::Bool => {
            width(1)?;
            match bytes[0] {
                0 => write_json(out, false),
                1 => write_json(out, true),
                other => Err(EmbedError::Decode(format!(
                    "field '{field_name}' holds {other}, which is not a Bool"
                ))),
            }
        }
        FieldType::I8 => number!(i8, 1),
        FieldType::U8 => number!(u8, 1),
        FieldType::I16 => number!(i16, 2),
        FieldType::U16 => number!(u16, 2),
        FieldType::I32 => number!(i32, 4),
        FieldType::U32 => number!(u32, 4),
        FieldType::I64 => number!(i64, 8),
        FieldType::U64 => number!(u64, 8),
        // serde_json writes a NaN or an infinity as `null`.
        FieldType::F32 => number!(f32, 4),
        FieldType::F64 => number!(f64, 8),
        FieldType::StringFixed(capacity) => {
            width(*capacity)?;
            let end = bytes
                .iter()
                .position(|byte| *byte == 0)
                .unwrap_or(bytes.len());
            write_json(out, utf8(&bytes[..end], field_name)?)
        }
        other => Err(unsupported(field_name, other)),
    }
}

fn unsupported(field_name: &str, field_type: &FieldType) -> EmbedError {
    EmbedError::Decode(format!(
        "field {field_name} has type {field_type:?}, which poll does not support"
    ))
}

fn utf8<'a>(bytes: &'a [u8], field_name: &str) -> Result<&'a str, EmbedError> {
    std::str::from_utf8(bytes).map_err(|error| {
        EmbedError::Decode(format!(
            "field '{field_name}' is not valid UTF-8 (valid up to byte {})",
            error.valid_up_to()
        ))
    })
}

fn write_json(out: &mut Vec<u8>, value: impl serde::Serialize) -> Result<(), EmbedError> {
    serde_json::to_writer(out, &value).map_err(|error| EmbedError::Decode(error.to_string()))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::encode::encode_json;
    use cerulion_core::codegen::parse_rosmsg;
    use cerulion_core::wire::WireHeader;

    fn layout(text: &str, name: &str, package: &str) -> WireLayout {
        let schema = parse_rosmsg(text, name, Some(package)).unwrap();
        let (schemas, _) = cerulion_core::dynamic::SchemaSet::from_schemas(vec![schema]).unwrap();
        schemas
            .layout(&format!("{package}/{name}"))
            .unwrap()
            .clone()
    }

    fn decode(layout: &WireLayout, frame: &[u8]) -> Result<String, EmbedError> {
        let mut out = Vec::new();
        decode_json_into(layout, frame, &mut out)?;
        Ok(String::from_utf8(out).expect("decoded JSON is UTF-8"))
    }

    #[test]
    fn decodes_a_hand_written_string_frame() {
        let layout = layout("string data\n", "String", "std_msgs");
        // The encoder's oracle frame for {"data":"hello"}, pinned byte for
        // byte in `encode::tests::encodes_string_to_hand_written_frame`.
        let frame = encode_json(&layout, r#"{"data":"hello"}"#, 64, 7, 42).unwrap();
        assert_eq!(
            &frame[WireHeader::SIZE..],
            &[8, 0, 0, 0, 5, 0, 0, 0, b'h', b'e', b'l', b'l', b'o']
        );
        assert_eq!(decode(&layout, &frame).unwrap(), r#"{"data":"hello"}"#);
    }

    /// Every type publish accepts comes back as the JSON publish took, field
    /// for field: integers at their extremes, floats, escapes, bytes.
    #[test]
    fn every_publish_type_round_trips_to_hand_written_json() {
        let layout = layout(
            "bool flag\nint8 a\nuint8 b\nint16 c\nuint16 d\nint32 e\nuint32 f\nint64 g\nuint64 h\nfloat32 x\nfloat64 y\nstring name\nuint8[] blob\n",
            "Everything",
            "test_msgs",
        );
        let json = r#"{"flag":true,"a":-128,"b":255,"c":-32768,"d":65535,"e":-2147483648,"f":4294967295,"g":-9223372036854775808,"h":18446744073709551615,"x":1.5,"y":-0.25,"name":"tab\t\"q\" é","blob":[0,1,255]}"#;
        let frame = encode_json(&layout, json, 256, 0, 0).unwrap();
        assert_eq!(decode(&layout, &frame).unwrap(), json);
    }

    #[test]
    fn missing_publish_fields_read_back_as_zero_and_empty() {
        let vector = layout(
            "float64 x\nfloat64 y\nfloat64 z\n",
            "Vector3",
            "geometry_msgs",
        );
        let frame = encode_json(&vector, r#"{"x":10.0}"#, 64, 0, 0).unwrap();
        assert_eq!(
            decode(&vector, &frame).unwrap(),
            r#"{"x":10.0,"y":0.0,"z":0.0}"#
        );
        let bytes = layout("uint8[] data\nstring note\n", "Bytes", "test_msgs");
        let frame = encode_json(&bytes, "{}", 64, 0, 0).unwrap();
        assert_eq!(decode(&bytes, &frame).unwrap(), r#"{"data":[],"note":""}"#);
    }

    fn fixed_name_layout() -> WireLayout {
        WireLayout {
            qualified_name: "test_msgs/FixedName".into(),
            schema_hash: 0,
            fixed_size: 4,
            fixed_align: 1,
            fixed_fields: vec![cerulion_core::dynamic::FieldLayout {
                name: "name".into(),
                offset: 0,
                size: 4,
                align: 1,
                field_type: FieldType::StringFixed(4),
            }],
            variable_fields: vec![],
        }
    }

    #[test]
    fn string_fixed_reads_up_to_its_first_nul() {
        let fixed = fixed_name_layout();
        for (json, want) in [
            (r#"{"name":"ab"}"#, r#"{"name":"ab"}"#),
            (r#"{"name":"four"}"#, r#"{"name":"four"}"#),
            ("{}", r#"{"name":""}"#),
        ] {
            let frame = encode_json(&fixed, json, 64, 0, 0).unwrap();
            assert_eq!(decode(&fixed, &frame).unwrap(), want, "{json}");
        }
    }

    /// Publish refuses a non-finite float, but a node can send one: JSON
    /// has no NaN, so it reads as `null` and the rest of the message stays.
    #[test]
    fn a_non_finite_float_reads_as_null() {
        let layout = layout("float32 a\nfloat64 b\nfloat64 c\n", "Floats", "test_msgs");
        let mut frame = encode_json(&layout, r#"{"c":2.0}"#, 64, 0, 0).unwrap();
        let body = WireHeader::SIZE;
        frame[body..body + 4].copy_from_slice(&f32::NAN.to_le_bytes());
        frame[body + 8..body + 16].copy_from_slice(&f64::NEG_INFINITY.to_le_bytes());
        assert_eq!(
            decode(&layout, &frame).unwrap(),
            r#"{"a":null,"b":null,"c":2.0}"#
        );
    }

    fn decode_error(layout: &WireLayout, frame: &[u8]) -> String {
        let error = decode(layout, frame).expect_err("the frame must not decode");
        assert_eq!(error.code(), crate::error::codes::DECODE);
        error.to_string()
    }

    #[test]
    fn refuses_a_frame_of_another_schema() {
        let string = layout("string data\n", "String", "std_msgs");
        let vector = layout(
            "float64 x\nfloat64 y\nfloat64 z\n",
            "Vector3",
            "geometry_msgs",
        );
        let frame = encode_json(&vector, r#"{"x":1.0}"#, 64, 0, 0).unwrap();
        let message = decode_error(&string, &frame);
        assert!(message.contains("does not match layout"), "{message}");
    }

    #[test]
    fn refuses_a_truncated_frame() {
        let string = layout("string data\n", "String", "std_msgs");
        let frame = encode_json(&string, r#"{"data":"hello"}"#, 64, 0, 0).unwrap();
        decode_error(&string, &frame[..frame.len() - 1]);
        decode_error(&string, &frame[..WireHeader::SIZE - 1]);
    }

    #[test]
    fn refuses_invalid_utf8_and_a_bool_byte_other_than_0_or_1() {
        let string = layout("string data\n", "String", "std_msgs");
        let mut frame = encode_json(&string, r#"{"data":"hi"}"#, 64, 0, 0).unwrap();
        let last = frame.len() - 1;
        frame[last] = 0xFF;
        assert_eq!(
            decode_error(&string, &frame),
            "field 'data' is not valid UTF-8 (valid up to byte 1)"
        );

        let mut fixed = encode_json(&fixed_name_layout(), r#"{"name":"ab"}"#, 64, 0, 0).unwrap();
        fixed[WireHeader::SIZE + 1] = 0xFF;
        assert_eq!(
            decode_error(&fixed_name_layout(), &fixed),
            "field 'name' is not valid UTF-8 (valid up to byte 1)"
        );

        let boolean = layout("bool value\n", "Boolean", "test_msgs");
        let mut frame = encode_json(&boolean, r#"{"value":true}"#, 64, 0, 0).unwrap();
        frame[WireHeader::SIZE] = 2;
        assert_eq!(
            decode_error(&boolean, &frame),
            "field 'value' holds 2, which is not a Bool"
        );
    }

    #[test]
    fn decoding_is_deterministic_and_reuses_the_buffer() {
        let layout = layout("string data\n", "String", "std_msgs");
        let long = encode_json(&layout, r#"{"data":"a longer message"}"#, 64, 1, 1).unwrap();
        let short = encode_json(&layout, r#"{"data":"x"}"#, 64, 2, 2).unwrap();
        let mut out = Vec::new();
        decode_json_into(&layout, &long, &mut out).unwrap();
        let first = out.clone();
        decode_json_into(&layout, &short, &mut out).unwrap();
        assert_eq!(out, br#"{"data":"x"}"#);
        decode_json_into(&layout, &long, &mut out).unwrap();
        assert_eq!(out, first);
    }
}
