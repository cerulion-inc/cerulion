// SPDX-License-Identifier: AGPL-3.0-only
//! Encoder for the JSON publisher input surface.
use cerulion_core::codegen::FieldType;
use cerulion_core::dynamic::{FieldLayout, FrameEncoder, WireLayout};
use cerulion_core::wire::WireHeader;
use serde_json::Value;

use crate::error::EmbedError;

/// Encode a JSON object into a timestamped wire frame using a compiled layout.
pub fn encode_json(
    layout: &WireLayout,
    json: &str,
    max_payload_bytes: usize,
    timestamp_ns: u64,
) -> Result<Vec<u8>, EmbedError> {
    let value: Value = serde_json::from_str(json)
        .map_err(|error| EmbedError::Encode(format!("invalid publish JSON: {error}")))?;
    let object = value
        .as_object()
        .ok_or_else(|| EmbedError::Encode("publish JSON must be an object".to_string()))?;
    let mut known = std::collections::BTreeSet::new();
    known.extend(layout.fixed_fields.iter().map(|field| field.name.as_str()));
    known.extend(
        layout
            .variable_fields
            .iter()
            .map(|field| field.name.as_str()),
    );
    if let Some(key) = object.keys().find(|key| !known.contains(key.as_str())) {
        return Err(EmbedError::Encode(format!("unknown field '{key}'")));
    }

    for field in &layout.fixed_fields {
        ensure_supported(&field.field_type, &field.name)?;
    }
    for field in &layout.variable_fields {
        ensure_supported(&field.field_type, &field.name)?;
    }

    let mut variable_values = Vec::with_capacity(layout.variable_fields.len());
    let mut variable_lengths = Vec::with_capacity(layout.variable_fields.len());
    for field in &layout.variable_fields {
        let value = object.get(&field.name);
        let bytes = encode_variable(value, &field.name, &field.field_type)?;
        variable_lengths.push(bytes.len());
        variable_values.push(bytes);
    }

    let encoder =
        FrameEncoder::new(layout).map_err(|error| EmbedError::Encode(error.to_string()))?;
    let total_len = encoder
        .required_len(&variable_lengths)
        .map_err(|error| EmbedError::Encode(error.to_string()))?;
    let payload_len = total_len.saturating_sub(WireHeader::SIZE);
    if payload_len > max_payload_bytes {
        return Err(EmbedError::Encode(format!(
            "encoded payload length {payload_len} exceeds max_payload_bytes {max_payload_bytes}"
        )));
    }
    let mut frame = vec![0; total_len];
    {
        let mut cursor = encoder
            .begin(&mut frame, &variable_lengths, timestamp_ns)
            .map_err(|error| EmbedError::Encode(error.to_string()))?;
        for field in &layout.fixed_fields {
            encode_fixed(
                cursor
                    .fixed_field_mut(&field.name)
                    .map_err(|error| EmbedError::Encode(error.to_string()))?,
                object.get(&field.name),
                field,
            )?;
        }
        for (index, field) in layout.variable_fields.iter().enumerate() {
            cursor
                .variable_field_mut(&field.name)
                .map_err(|error| EmbedError::Encode(error.to_string()))?
                .copy_from_slice(&variable_values[index]);
        }
        cursor.finish();
    }
    Ok(frame)
}

fn ensure_supported(field_type: &FieldType, name: &str) -> Result<(), EmbedError> {
    if matches!(
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
    {
        Ok(())
    } else {
        Err(EmbedError::Encode(format!(
            "field {name} has type {field_type:?}, which publish does not support"
        )))
    }
}

fn encode_fixed(
    bytes: &mut [u8],
    value: Option<&Value>,
    field: &FieldLayout,
) -> Result<(), EmbedError> {
    let invalid = || {
        EmbedError::Encode(format!(
            "invalid value for field '{}', expected {:?}",
            field.name, field.field_type
        ))
    };
    match &field.field_type {
        FieldType::Bool => {
            bytes[0] = match value {
                None => 0,
                Some(Value::Bool(false)) => 0,
                Some(Value::Bool(true)) => 1,
                _ => return Err(invalid()),
            };
        }
        FieldType::I8 => put_integer::<i8>(bytes, value, &field.name, |v| v.to_le_bytes())?,
        FieldType::U8 => put_integer::<u8>(bytes, value, &field.name, |v| v.to_le_bytes())?,
        FieldType::I16 => put_integer::<i16>(bytes, value, &field.name, |v| v.to_le_bytes())?,
        FieldType::U16 => put_integer::<u16>(bytes, value, &field.name, |v| v.to_le_bytes())?,
        FieldType::I32 => put_integer::<i32>(bytes, value, &field.name, |v| v.to_le_bytes())?,
        FieldType::U32 => put_integer::<u32>(bytes, value, &field.name, |v| v.to_le_bytes())?,
        FieldType::I64 => put_integer::<i64>(bytes, value, &field.name, |v| v.to_le_bytes())?,
        FieldType::U64 => put_integer::<u64>(bytes, value, &field.name, |v| v.to_le_bytes())?,
        FieldType::F32 => {
            let number = match value {
                None => 0.0,
                Some(Value::Number(n)) => {
                    n.as_f64().filter(|v| v.is_finite()).ok_or_else(invalid)?
                }
                _ => return Err(invalid()),
            };
            let converted = number as f32;
            if !converted.is_finite() {
                return Err(invalid());
            }
            bytes.copy_from_slice(&converted.to_le_bytes());
        }
        FieldType::F64 => {
            let number = match value {
                None => 0.0,
                Some(Value::Number(n)) => {
                    n.as_f64().filter(|v| v.is_finite()).ok_or_else(invalid)?
                }
                _ => return Err(invalid()),
            };
            bytes.copy_from_slice(&number.to_le_bytes());
        }
        FieldType::StringFixed(capacity) => {
            if let Some(value) = value {
                let text = value.as_str().ok_or_else(invalid)?;
                let encoded = text.as_bytes();
                if encoded.len() > *capacity {
                    return Err(EmbedError::Encode(format!(
                        "field '{}' exceeds StringFixed({capacity})",
                        field.name
                    )));
                }
                bytes[..encoded.len()].copy_from_slice(encoded);
            }
        }
        _ => return Err(invalid()),
    }
    Ok(())
}

fn put_integer<T: IntegerValue>(
    bytes: &mut [u8],
    value: Option<&Value>,
    field_name: &str,
    to_bytes: impl FnOnce(T) -> T::Bytes,
) -> Result<(), EmbedError> {
    let value = match value {
        None => T::zero(),
        Some(value) => T::from_json(value).ok_or_else(|| {
            EmbedError::Encode(format!("invalid integer value for field '{field_name}'"))
        })?,
    };
    bytes.copy_from_slice(to_bytes(value).as_ref());
    Ok(())
}

trait IntegerValue: Sized {
    type Bytes: AsRef<[u8]>;
    fn zero() -> Self;
    fn from_json(value: &Value) -> Option<Self>;
}

macro_rules! integer_value {
    ($ty:ty, $getter:ident, $size:expr) => {
        impl IntegerValue for $ty {
            type Bytes = [u8; $size];
            fn zero() -> Self {
                0
            }
            fn from_json(value: &Value) -> Option<Self> {
                value.as_number()?.$getter()?.try_into().ok()
            }
        }
    };
}

integer_value!(i8, as_i64, 1);
integer_value!(u8, as_u64, 1);
integer_value!(i16, as_i64, 2);
integer_value!(u16, as_u64, 2);
integer_value!(i32, as_i64, 4);
integer_value!(u32, as_u64, 4);
integer_value!(i64, as_i64, 8);
integer_value!(u64, as_u64, 8);

fn encode_variable(
    value: Option<&Value>,
    field_name: &str,
    field_type: &FieldType,
) -> Result<Vec<u8>, EmbedError> {
    match field_type {
        FieldType::String => match value {
            None => Ok(Vec::new()),
            Some(Value::String(text)) => Ok(text.as_bytes().to_vec()),
            _ => Err(EmbedError::Encode(format!(
                "invalid string value for field '{field_name}'"
            ))),
        },
        ty if is_byte_array(ty) => encode_bytes(value, field_name),
        _ => Err(EmbedError::Encode(format!(
            "field {field_name} has type {field_type:?}, which publish does not support"
        ))),
    }
}

fn is_byte_array(field_type: &FieldType) -> bool {
    matches!(field_type, FieldType::Bytes)
        || matches!(
            field_type,
            FieldType::DynamicArray { element_type }
                if matches!(element_type.as_ref(), FieldType::U8)
        )
}

fn encode_bytes(value: Option<&Value>, field_name: &str) -> Result<Vec<u8>, EmbedError> {
    match value {
        None => Ok(Vec::new()),
        Some(Value::Array(items)) => items
            .iter()
            .map(|item| {
                item.as_u64()
                    .and_then(|number| u8::try_from(number).ok())
                    .ok_or_else(|| {
                        EmbedError::Encode(format!(
                            "field '{field_name}' bytes must be 0..=255 integers"
                        ))
                    })
            })
            .collect(),
        _ => Err(EmbedError::Encode(format!(
            "invalid bytes value for field '{field_name}'"
        ))),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use cerulion_core::codegen::parse_rosmsg;

    fn layout(text: &str, name: &str, package: &str) -> WireLayout {
        let schema = parse_rosmsg(text, name, Some(package)).unwrap();
        let (schemas, _) = cerulion_core::dynamic::SchemaSet::from_schemas(vec![schema]).unwrap();
        schemas
            .layout(&format!("{package}/{name}"))
            .unwrap()
            .clone()
    }

    #[test]
    fn encodes_string_to_hand_written_frame() {
        let layout = layout("string data\n", "String", "std_msgs");
        let frame = encode_json(&layout, r#"{"data":"hello"}"#, 64, 7).unwrap();
        assert_eq!(frame.len(), WireHeader::SIZE + 13);
        assert_eq!(
            &frame[WireHeader::SIZE..],
            &[8, 0, 0, 0, 5, 0, 0, 0, b'h', b'e', b'l', b'l', b'o']
        );
    }

    #[test]
    fn rejects_unknown_fields_and_bad_json_values() {
        let layout = layout("string data\n", "String", "std_msgs");
        assert!(encode_json(&layout, r#"{"unknown":"x"}"#, 64, 0).is_err());
        assert!(encode_json(&layout, r#"[1]"#, 64, 0).is_err());
        assert!(encode_json(&layout, r#"{"data":1}"#, 64, 0).is_err());
    }

    #[test]
    fn encodes_fixed_vector_with_oracle_bytes() {
        let layout = layout(
            "float64 x\nfloat64 y\nfloat64 z\n",
            "Vector3",
            "geometry_msgs",
        );
        let frame = encode_json(&layout, r#"{"x":1.5,"y":-2.0}"#, 64, 0).unwrap();
        let expected = [
            0, 0, 0, 0, 0, 0, 248, 63, 0, 0, 0, 0, 0, 0, 0, 192, 0, 0, 0, 0, 0, 0, 0, 0,
        ];
        assert_eq!(&frame[WireHeader::SIZE..], &expected);
    }

    #[test]
    fn rejects_payload_limit_and_unsupported_type() {
        let string = layout("string data\n", "String", "std_msgs");
        assert!(encode_json(&string, r#"{"data":"large"}"#, 1, 0).is_err());
        let nested = layout("geometry_msgs/Point point\n", "Pose", "geometry_msgs");
        assert!(encode_json(&nested, r#"{}"#, 64, 0).is_err());
    }

    #[test]
    fn rejects_integer_float_and_bounded_string_errors() {
        let integer = layout("int8 value\n", "Int8", "test_msgs");
        assert!(encode_json(&integer, r#"{"value":1.5}"#, 64, 0).is_err());
        assert!(encode_json(&integer, r#"{"value":128}"#, 64, 0).is_err());
        let float = layout("float32 value\n", "Float32", "test_msgs");
        assert!(encode_json(&float, r#"{"value":1e300}"#, 64, 0).is_err());
        let fixed = WireLayout {
            qualified_name: "test_msgs/FixedName".into(),
            schema_hash: 0,
            fixed_size: 4,
            fixed_align: 1,
            fixed_fields: vec![FieldLayout {
                name: "name".into(),
                offset: 0,
                size: 4,
                align: 1,
                field_type: FieldType::StringFixed(4),
            }],
            variable_fields: vec![],
        };
        assert!(encode_json(&fixed, r#"{"name":"longer"}"#, 64, 0).is_err());
    }

    #[test]
    fn encodes_bytes_and_rejects_invalid_bytes_and_booleans() {
        let bytes = layout("uint8[] data\n", "Bytes", "test_msgs");
        let frame = encode_json(&bytes, r#"{"data":[1,2,255]}"#, 64, 0).unwrap();
        assert_eq!(
            &frame[WireHeader::SIZE..],
            &[8, 0, 0, 0, 3, 0, 0, 0, 1, 2, 255]
        );
        assert!(encode_json(&bytes, r#"{"data":[256]}"#, 64, 0).is_err());
        let boolean = layout("bool value\n", "Boolean", "test_msgs");
        assert!(encode_json(&boolean, r#"{"value":"true"}"#, 64, 0).is_err());
    }

    #[test]
    fn output_is_deterministic_except_timestamp() {
        let layout = layout("string data\n", "String", "std_msgs");
        let first = encode_json(&layout, r#"{"data":"same"}"#, 64, 123).unwrap();
        let second = encode_json(&layout, r#"{"data":"same"}"#, 64, 456).unwrap();
        assert_eq!(&first[..24], &second[..24]);
        assert_eq!(&first[32..], &second[32..]);
    }
}
