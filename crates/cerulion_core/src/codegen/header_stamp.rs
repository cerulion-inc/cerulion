// SPDX-License-Identifier: AGPL-3.0-only
//! Resolve where a schema's `header.stamp` lives in a wire frame.
//!
//! Capture-time Sync matching (see [`crate::scheduler::sync_stamp`]) reads
//! each trigger input's `header.stamp` on every frame, so the location is
//! resolved ONCE per input from the schema, at load, and the per-frame read is
//! a few fixed-offset slice reads. A schema with no usable `header.stamp` is
//! refused here, before any frame flows.
//!
//! The location comes from the same [`LayoutResolver`] the generated writers
//! and the frame walker use, so the offsets agree with what producers write.

use super::layout::LayoutResolver;
use super::schema::FieldType;
use crate::scheduler::sync_stamp::HeaderStampLocation;

/// Why a schema has no usable `header.stamp`.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum HeaderStampSchemaError {
    /// The schema is not in the schema set.
    #[error("schema `{schema}` is not known")]
    UnknownSchema {
        /// The schema asked for.
        schema: String,
    },
    /// The schema has no field named `header`.
    #[error("schema `{schema}` has no `header` field")]
    NoHeaderField {
        /// The schema asked for.
        schema: String,
    },
    /// `header` is not a nested message.
    #[error(
        "schema `{schema}` has a `header` field of type `{found}`, not a message with a `stamp`"
    )]
    HeaderNotAMessage {
        /// The schema asked for.
        schema: String,
        /// The `header` field's type.
        found: String,
    },
    /// `header`'s message type is not in the schema set.
    #[error("schema `{schema}`: the `header` type `{header_schema}` is not known")]
    UnknownHeaderSchema {
        /// The schema asked for.
        schema: String,
        /// The `header` field's message type.
        header_schema: String,
    },
    /// `header`'s message type has no `stamp` field (or keeps it out of its
    /// fixed section).
    #[error("schema `{schema}`: `header` (`{header_schema}`) has no fixed `stamp` field")]
    NoStampField {
        /// The schema asked for.
        schema: String,
        /// The `header` field's message type.
        header_schema: String,
    },
    /// `header.stamp` is not a `builtin_interfaces/Time` shape
    /// (`int32 sec` then `uint32 nanosec`).
    #[error("schema `{schema}`: `header.stamp` is `{found}`, not `int32 sec` + `uint32 nanosec`")]
    StampNotTime {
        /// The schema asked for.
        schema: String,
        /// The `stamp` field's type.
        found: String,
    },
}

/// The qualified name a nested reference points at. A bare reference from a
/// packaged schema means a same-package type (ROS `.msg` semantics), so the
/// parent's package is tried when the bare name is not known.
fn nested_target(
    resolver: &mut LayoutResolver,
    field_type: &FieldType,
    parent: &str,
) -> Option<String> {
    let FieldType::Nested {
        schema_name,
        package,
        ..
    } = field_type
    else {
        return None;
    };
    if package.is_some() || resolver.layout_of(schema_name).is_some() {
        return Some(field_type.canonical_str());
    }
    match parent.rsplit_once('/') {
        Some((pkg, _)) => Some(format!("{pkg}/{schema_name}")),
        None => Some(schema_name.clone()),
    }
}

/// Resolve the location of `header.stamp` in frames of `schema`.
///
/// Accepts a `header` field of any message type (fixed or variable) whose
/// fixed section holds a `stamp` field shaped like `builtin_interfaces/Time`:
/// `int32 sec` immediately followed by `uint32 nanosec`.
pub fn header_stamp_location(
    resolver: &mut LayoutResolver,
    schema: &str,
) -> Result<HeaderStampLocation, HeaderStampSchemaError> {
    let layout =
        resolver
            .layout_of(schema)
            .ok_or_else(|| HeaderStampSchemaError::UnknownSchema {
                schema: schema.to_string(),
            })?;

    // Where `header` sits in the parent: inline in the fixed section, or
    // behind an offset-table entry.
    let fixed = layout.fixed_fields.iter().find(|f| f.name == "header");
    let variable = layout
        .variable_fields
        .iter()
        .position(|f| f.name == "header");
    let header_type = match (fixed, variable) {
        (Some(f), _) => f.field_type.clone(),
        (None, Some(i)) => layout.variable_fields[i].field_type.clone(),
        (None, None) => {
            return Err(HeaderStampSchemaError::NoHeaderField {
                schema: schema.to_string(),
            })
        }
    };
    let header_schema = nested_target(resolver, &header_type, schema).ok_or_else(|| {
        HeaderStampSchemaError::HeaderNotAMessage {
            schema: schema.to_string(),
            found: header_type.canonical_str(),
        }
    })?;
    let header_layout = resolver.layout_of(&header_schema).ok_or_else(|| {
        HeaderStampSchemaError::UnknownHeaderSchema {
            schema: schema.to_string(),
            header_schema: header_schema.clone(),
        }
    })?;
    let stamp = header_layout
        .fixed_fields
        .iter()
        .find(|f| f.name == "stamp")
        .ok_or_else(|| HeaderStampSchemaError::NoStampField {
            schema: schema.to_string(),
            header_schema: header_schema.clone(),
        })?;

    let not_time = || HeaderStampSchemaError::StampNotTime {
        schema: schema.to_string(),
        found: stamp.field_type.canonical_str(),
    };
    let time_schema =
        nested_target(resolver, &stamp.field_type, &header_schema).ok_or_else(not_time)?;
    let time_layout = resolver.layout_of(&time_schema).ok_or_else(not_time)?;
    let sec = time_layout.fixed_fields.iter().find(|f| f.name == "sec");
    let nanosec = time_layout
        .fixed_fields
        .iter()
        .find(|f| f.name == "nanosec");
    let sec_offset = match (sec, nanosec) {
        (Some(sec), Some(nanosec))
            if sec.field_type == FieldType::I32
                && nanosec.field_type == FieldType::U32
                && nanosec.offset == sec.offset + 4 =>
        {
            sec.offset
        }
        _ => return Err(not_time()),
    };
    let stamp_offset = stamp.offset + sec_offset;

    Ok(match (fixed, variable) {
        (Some(header), _) => HeaderStampLocation::Fixed {
            offset: header.offset + stamp_offset,
        },
        (None, Some(index)) => HeaderStampLocation::Table {
            fixed_size: layout.fixed_size,
            index,
            stamp_offset,
        },
        (None, None) => unreachable!("the `header` lookup above returned on this arm"),
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::codegen::{parse_rosmsg, MessageSchema};

    /// The built-in ROS 2 corpus plus a small hand-written `demo` package.
    fn resolver(extra: &[(&str, &str)]) -> LayoutResolver {
        let mut schemas: Vec<MessageSchema> = Vec::new();
        for (pkg, name, text) in native_ros2_messages::BUILTIN_MSGS {
            if let Ok(s) = parse_rosmsg(text, name, Some(pkg)) {
                schemas.push(s);
            }
        }
        for (name, text) in extra {
            schemas.push(parse_rosmsg(text, name, Some("demo")).expect("demo schema parses"));
        }
        LayoutResolver::new(schemas).0
    }

    #[test]
    fn pose_stamped_reaches_its_stamp_through_the_offset_table() {
        // `pose` is 56 fixed bytes; `header` is the only variable field.
        assert_eq!(
            header_stamp_location(&mut resolver(&[]), "geometry_msgs/PoseStamped"),
            Ok(HeaderStampLocation::Table {
                fixed_size: 56,
                index: 0,
                stamp_offset: 0
            })
        );
    }

    #[test]
    fn image_header_is_table_entry_zero_after_sixteen_fixed_bytes() {
        // height, width, is_bigendian (padded), step: 16 fixed bytes; the
        // variable fields are header, encoding, data in that order.
        assert_eq!(
            header_stamp_location(&mut resolver(&[]), "sensor_msgs/Image"),
            Ok(HeaderStampLocation::Table {
                fixed_size: 16,
                index: 0,
                stamp_offset: 0
            })
        );
    }

    #[test]
    fn a_fixed_header_is_read_inline_at_its_fixed_offset() {
        let extra = [
            ("FixedHeader", "uint32 seq\nbuiltin_interfaces/Time stamp\n"),
            ("Reading", "uint64 id\nFixedHeader header\n"),
        ];
        // Reading: id at 0, header at 8; FixedHeader: seq at 0, stamp at 4.
        assert_eq!(
            header_stamp_location(&mut resolver(&extra), "demo/Reading"),
            Ok(HeaderStampLocation::Fixed { offset: 12 })
        );
    }

    #[test]
    fn a_schema_with_no_header_is_refused() {
        assert_eq!(
            header_stamp_location(&mut resolver(&[]), "geometry_msgs/Vector3"),
            Err(HeaderStampSchemaError::NoHeaderField {
                schema: "geometry_msgs/Vector3".into()
            })
        );
    }

    #[test]
    fn an_unknown_schema_is_refused() {
        assert_eq!(
            header_stamp_location(&mut resolver(&[]), "demo/Nope"),
            Err(HeaderStampSchemaError::UnknownSchema {
                schema: "demo/Nope".into()
            })
        );
    }

    #[test]
    fn a_primitive_header_is_refused() {
        let extra = [("Bad", "uint64 header\n")];
        assert_eq!(
            header_stamp_location(&mut resolver(&extra), "demo/Bad"),
            Err(HeaderStampSchemaError::HeaderNotAMessage {
                schema: "demo/Bad".into(),
                found: "uint64".into()
            })
        );
    }

    #[test]
    fn a_header_without_a_stamp_is_refused() {
        let extra = [("SeqOnly", "uint32 seq\n"), ("Bad", "SeqOnly header\n")];
        assert_eq!(
            header_stamp_location(&mut resolver(&extra), "demo/Bad"),
            Err(HeaderStampSchemaError::NoStampField {
                schema: "demo/Bad".into(),
                header_schema: "demo/SeqOnly".into()
            })
        );
    }

    #[test]
    fn a_stamp_that_is_not_a_time_is_refused() {
        let extra = [("NsHeader", "uint64 stamp\n"), ("Bad", "NsHeader header\n")];
        assert_eq!(
            header_stamp_location(&mut resolver(&extra), "demo/Bad"),
            Err(HeaderStampSchemaError::StampNotTime {
                schema: "demo/Bad".into(),
                found: "uint64".into()
            })
        );
    }

    #[test]
    fn an_unsigned_seconds_field_is_not_a_time() {
        let extra = [
            ("UTime", "uint32 sec\nuint32 nanosec\n"),
            ("H", "UTime stamp\n"),
            ("Bad", "H header\n"),
        ];
        assert!(matches!(
            header_stamp_location(&mut resolver(&extra), "demo/Bad"),
            Err(HeaderStampSchemaError::StampNotTime { .. })
        ));
    }

    #[test]
    fn a_time_shape_with_swapped_fields_is_refused() {
        let extra = [
            ("Swapped", "uint32 nanosec\nint32 sec\n"),
            ("H", "Swapped stamp\n"),
            ("Bad", "H header\n"),
        ];
        assert!(matches!(
            header_stamp_location(&mut resolver(&extra), "demo/Bad"),
            Err(HeaderStampSchemaError::StampNotTime { .. })
        ));
    }
}
