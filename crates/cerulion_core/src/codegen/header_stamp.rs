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

/// The index of the schema a nested reference in the `parent_idx`-th schema
/// binds to, by the SAME identity binding the layouts use (qualified, then
/// same-package, then the bare `Header` to `std_msgs/Header` rule, then an
/// unambiguous bare name), so the type this module reads is the type the
/// writer embedded, even when a second schema shares its qualified text.
/// `None` for a non-message field or a reference that resolves to nothing.
fn nested_target(
    resolver: &LayoutResolver,
    field_type: &FieldType,
    parent_idx: usize,
) -> Option<usize> {
    let FieldType::Nested {
        schema_name,
        package,
        ..
    } = field_type
    else {
        return None;
    };
    resolver.nested_target_idx(schema_name, package, parent_idx)
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
    // The top-level schema is named by text; every hop below it keeps the
    // bound index and never goes back through qualified text.
    let unknown = || HeaderStampSchemaError::UnknownSchema {
        schema: schema.to_string(),
    };
    let schema_idx = resolver.index_of(schema).ok_or_else(unknown)?;
    let layout = resolver.layout_at(schema_idx).map_err(|_| unknown())?;

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
    if !matches!(header_type, FieldType::Nested { .. }) {
        return Err(HeaderStampSchemaError::HeaderNotAMessage {
            schema: schema.to_string(),
            found: header_type.canonical_str(),
        });
    }
    let header_idx = nested_target(resolver, &header_type, schema_idx).ok_or_else(|| {
        HeaderStampSchemaError::UnknownHeaderSchema {
            schema: schema.to_string(),
            header_schema: header_type.canonical_str(),
        }
    })?;
    // The qualified text is for messages only.
    let header_schema = resolver.schemas()[header_idx].qualified_name();
    let header_layout = resolver.layout_at(header_idx).map_err(|_| {
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
    let time_idx = nested_target(resolver, &stamp.field_type, header_idx).ok_or_else(not_time)?;
    let time_layout = resolver.layout_at(time_idx).map_err(|_| not_time())?;
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
            data_floor: layout.data_floor(),
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
        resolver_with_bare(extra, &[])
    }

    /// [`resolver`] plus package-less schemas, for the name-precedence arms.
    fn resolver_with_bare(extra: &[(&str, &str)], bare: &[(&str, &str)]) -> LayoutResolver {
        let mut schemas: Vec<MessageSchema> = Vec::new();
        for (name, text) in bare {
            schemas.push(parse_rosmsg(text, name, None).expect("bare schema parses"));
        }
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
                stamp_offset: 0,
                data_floor: 64
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
                stamp_offset: 0,
                data_floor: 40
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
    fn a_bare_header_in_a_package_without_its_own_resolves_to_std_msgs_header() {
        // `demo` has no `Header`, so the bare reference is `std_msgs/Header`
        // (variable: its `frame_id`), the type the writer embeds. `value` is
        // the 8-byte fixed section; `header` is table entry 0.
        let extra = [("Stamped", "Header header\nfloat64 value\n")];
        assert_eq!(
            header_stamp_location(&mut resolver(&extra), "demo/Stamped"),
            Ok(HeaderStampLocation::Table {
                fixed_size: 8,
                index: 0,
                stamp_offset: 0,
                data_floor: 16
            })
        );
    }

    #[test]
    fn a_same_package_header_wins_over_a_package_less_one_of_the_same_name() {
        // Two fixed `Header`s with different prefixes: `demo/Header` puts
        // `stamp` at 4 (after `seq`), the package-less one at 0. The writer
        // of `demo/Reading` embeds `demo/Header` at offset 8 (after `id`), so
        // the stamp is at 12; reading the package-less one would say 8.
        let extra = [
            ("Header", "uint32 seq\nbuiltin_interfaces/Time stamp\n"),
            ("Reading", "uint64 id\nHeader header\n"),
        ];
        let bare = [("Header", "builtin_interfaces/Time stamp\n")];
        assert_eq!(
            header_stamp_location(&mut resolver_with_bare(&extra, &bare), "demo/Reading"),
            Ok(HeaderStampLocation::Fixed { offset: 12 })
        );
    }

    #[test]
    fn a_header_type_that_resolves_to_nothing_is_refused_by_its_written_name() {
        let extra = [("Bad", "nope_msgs/Missing header\n")];
        assert_eq!(
            header_stamp_location(&mut resolver(&extra), "demo/Bad"),
            Err(HeaderStampSchemaError::UnknownHeaderSchema {
                schema: "demo/Bad".into(),
                header_schema: "nope_msgs/Missing".into()
            })
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

    #[test]
    fn a_later_package_less_twin_of_std_msgs_header_never_moves_the_stamp() {
        // A package-less schema NAMED `std_msgs/Header`, listed after the
        // corpus so it owns the qualified text, puts `stamp` at 4 (after
        // `seq`). `demo/Cam { std_msgs/Header header; float64 value }` binds
        // the corpus `std_msgs/Header` (variable: its `frame_id`), so
        // `header` is table entry 0 and the stamp is at 0 inside it; the
        // fixed section is `value` alone (8 bytes, data floor 8 + 8).
        let mut schemas: Vec<MessageSchema> = Vec::new();
        for (pkg, name, text) in native_ros2_messages::BUILTIN_MSGS {
            if let Ok(s) = parse_rosmsg(text, name, Some(pkg)) {
                schemas.push(s);
            }
        }
        schemas.push(
            parse_rosmsg(
                "std_msgs/Header header\nfloat64 value\n",
                "Cam",
                Some("demo"),
            )
            .expect("demo/Cam parses"),
        );
        schemas.push(
            parse_rosmsg(
                "uint32 seq\nbuiltin_interfaces/Time stamp\n",
                "std_msgs/Header",
                None,
            )
            .expect("twin parses"),
        );
        let mut resolver = LayoutResolver::new(schemas).0;
        // The twin owns the text: a lookup by text sees `seq` first.
        assert_eq!(
            resolver
                .layout_of("std_msgs/Header")
                .expect("twin")
                .fixed_fields[0]
                .name,
            "seq"
        );
        assert_eq!(
            header_stamp_location(&mut resolver, "demo/Cam"),
            Ok(HeaderStampLocation::Table {
                fixed_size: 8,
                index: 0,
                stamp_offset: 0,
                data_floor: 16
            })
        );
    }
}
