// SPDX-License-Identifier: AGPL-3.0-only
//! ROS2 .msg file parser.
//!
//! Parses ROS2 message definitions and converts them to our schema IR.
//!
//! # ROS2 .msg format
//!
//! ```text
//! # Comment
//! fieldtype1 fieldname1
//! fieldtype2 fieldname2 defaultvalue
//! uint8 CONSTANT=42
//! ```
//!
//! Supported types: bool, int8, uint8, int16, uint16, int32, uint32,
//! int64, uint64, float32, float64, string, pkg/MsgType, T[], T[N]

use super::schema::{DefaultLiteral, FieldDef, FieldType, MessageSchema};

/// Parse a ROS2 .msg file content into a MessageSchema.
///
/// # Arguments
/// * `msg_content` - The content of the .msg file
/// * `msg_name` - The message name (e.g., "Image")
/// * `package` - Package name (e.g., "sensor_msgs"). When `Some`, the
///   schema hash is computed over the qualified name `"pkg/Name"` so
///   identically named messages in different packages never collide on
///   the wire. `None` keeps the legacy bare-name hash.
pub fn parse_rosmsg(
    msg_content: &str,
    msg_name: &str,
    package: Option<&str>,
) -> Result<MessageSchema, ParseError> {
    let mut schema = match package {
        Some(pkg) => MessageSchema::new_in_package(msg_name, pkg),
        None => MessageSchema::new(msg_name),
    };

    for (line_num, line) in msg_content.lines().enumerate() {
        let line = line.trim();

        // Skip empty lines and comments
        if line.is_empty() || line.starts_with('#') {
            continue;
        }

        // Skip constant definitions (e.g., "uint8 CONSTANT=42",
        // `string NAME="value"`). The '=' belongs to the NAME token —
        // checking the whole line would also swallow bounded-type FIELDS
        // like `string<=64 name` or `int32[<=8] ids`, whose '=' lives in
        // the TYPE token.
        let mut tokens = line.split_whitespace();
        let _type_token = tokens.next();
        if let Some(name_token) = tokens.next() {
            if name_token.contains('=') {
                continue;
            }
        }
        // `uint8 FOO = 1` (spaces around '='): name token is bare, the
        // NEXT token starts with '='.
        if let Some(third) = tokens.next() {
            if third.starts_with('=') {
                continue;
            }
        }

        // Parse field: "type name" or "type name default"
        let field = parse_field_line(line, line_num + 1)?;
        schema.add_field(field);
    }

    Ok(schema)
}

fn parse_field_line(line: &str, line_num: usize) -> Result<FieldDef, ParseError> {
    // Split into parts: type, name, optional default
    let parts: Vec<&str> = line.split_whitespace().collect();
    if parts.len() < 2 {
        return Err(ParseError::InvalidLine {
            line_num,
            message: format!("expected 'type name', got: {line}"),
        });
    }

    let ros_type = parts[0];
    let field_name = parts[1];

    let field_type = parse_ros_type(ros_type).map_err(|e| ParseError::InvalidType {
        line_num,
        message: e,
    })?;

    let mut field = FieldDef::new(field_name, field_type);

    // Capture a scalar default value (`float64 w 1`) for
    // primitive numeric/bool fields. A third token starting with '#' is a
    // trailing comment, not a default. Defaults on strings/arrays/nested
    // types are not supported and are dropped (rosidl supports them; the
    // wire format gains nothing from them since variable fields are
    // always explicitly written).
    if let Some(token) = parts.get(2) {
        if !token.starts_with('#') {
            if let Some(default) = parse_default_literal(&field.field_type, token, line_num)? {
                field = field.with_default(default);
            }
        }
    }

    Ok(field)
}

/// Parse + validate a scalar default token for a primitive field type.
/// Returns `Ok(None)` for field types that don't support defaults
/// (strings, arrays, nested) — those are dropped, not errors.
fn parse_default_literal(
    ft: &FieldType,
    token: &str,
    line_num: usize,
) -> Result<Option<DefaultLiteral>, ParseError> {
    let invalid = |msg: String| ParseError::InvalidLine {
        line_num,
        message: msg,
    };
    match ft {
        // rosidl's value parser is case-insensitive for bool literals
        // ("True"/"False" Python-style appear in upstream .msg files).
        FieldType::Bool => match token.to_ascii_lowercase().as_str() {
            "true" | "1" => Ok(Some(DefaultLiteral::Bool(true))),
            "false" | "0" => Ok(Some(DefaultLiteral::Bool(false))),
            _ => Err(invalid(format!("invalid bool default: {token}"))),
        },
        FieldType::I8
        | FieldType::U8
        | FieldType::I16
        | FieldType::U16
        | FieldType::I32
        | FieldType::U32
        | FieldType::I64
        | FieldType::U64 => token
            .parse::<i128>()
            .map(|v| Some(DefaultLiteral::Int(v)))
            .map_err(|_| invalid(format!("invalid integer default: {token}"))),
        FieldType::F32 | FieldType::F64 => {
            let v = token
                .parse::<f64>()
                .map_err(|_| invalid(format!("invalid float default: {token}")))?;
            // `f64::from_str` accepts "inf"/"NaN" (and saturates 1e999 to
            // inf) — but `{:?}` renders those as `inf`/`NaN`, which are
            // NOT valid Rust literals: the generated code would fail to
            // compile far from the .msg line. Reject here with the line
            // number instead.
            if !v.is_finite() {
                return Err(invalid(format!(
                    "non-finite float default: {token} (inf/NaN defaults are not supported)"
                )));
            }
            Ok(Some(DefaultLiteral::Float(v)))
        }
        // Strings / arrays / nested: defaults unsupported, dropped.
        _ => Ok(None),
    }
}

fn parse_ros_type(ros_type: &str) -> Result<FieldType, String> {
    // Check for array types: T[], T[N], or bounded T[<=N]
    if let Some(bracket_pos) = ros_type.find('[') {
        if !ros_type.ends_with(']') {
            return Err(format!("invalid array syntax: {ros_type}"));
        }

        let base_type = &ros_type[..bracket_pos];
        let length_str = &ros_type[bracket_pos + 1..ros_type.len() - 1];

        let element_type = parse_ros_type(base_type)?;

        if length_str.is_empty() {
            // Dynamic array: T[]
            return Ok(FieldType::DynamicArray {
                element_type: Box::new(element_type),
            });
        } else if let Some(bound_str) = length_str.strip_prefix("<=") {
            // Bounded array: T[<=N] (ROS2 IDL). The bound is an upper
            // limit, not a fixed length — wire-wise it is a dynamic array.
            bound_str
                .parse::<usize>()
                .map_err(|_| format!("invalid array bound: {length_str}"))?;
            return Ok(FieldType::DynamicArray {
                element_type: Box::new(element_type),
            });
        } else {
            // Fixed array: T[N]
            let length: usize = length_str
                .parse()
                .map_err(|_| format!("invalid array length: {length_str}"))?;
            return Ok(FieldType::FixedArray {
                element_type: Box::new(element_type),
                length,
            });
        }
    }

    // Bounded string: string<=N (ROS2 IDL). The bound is an upper limit —
    // wire-wise it is an ordinary variable-length string.
    if let Some(bound_str) = ros_type.strip_prefix("string<=") {
        bound_str
            .parse::<usize>()
            .map_err(|_| format!("invalid string bound: {ros_type}"))?;
        return Ok(FieldType::String);
    }
    if let Some(bound_str) = ros_type.strip_prefix("wstring<=") {
        bound_str
            .parse::<usize>()
            .map_err(|_| format!("invalid string bound: {ros_type}"))?;
        return Ok(FieldType::String);
    }

    // Check for package/MsgType format
    if ros_type.contains('/') {
        let parts: Vec<&str> = ros_type.split('/').collect();
        if parts.len() != 2 || parts[0].is_empty() || parts[1].is_empty() {
            return Err(format!("invalid message type: {ros_type}"));
        }
        // Keep the package on the reference so cross-package
        // resolution never guesses when two packages share a bare name.
        return Ok(FieldType::Nested {
            schema_name: parts[1].to_string(),
            package: Some(parts[0].to_string()),
            fixed: None,
        });
    }

    // Primitive types
    match ros_type {
        "bool" => Ok(FieldType::Bool),
        "int8" => Ok(FieldType::I8),
        // ROS2 Jazzy defines `byte` as octet/uint8 (the old `byte` → int8
        // mapping was the ROS1 convention). Matches FieldType::parse.
        "uint8" | "char" | "byte" => Ok(FieldType::U8),
        "int16" => Ok(FieldType::I16),
        "uint16" => Ok(FieldType::U16),
        "int32" => Ok(FieldType::I32),
        "uint32" => Ok(FieldType::U32),
        "int64" => Ok(FieldType::I64),
        "uint64" => Ok(FieldType::U64),
        "float32" => Ok(FieldType::F32),
        "float64" => Ok(FieldType::F64),
        "string" | "wstring" => Ok(FieldType::String),
        // ROS1-style builtins still present in some upstream .msg files
        // (e.g. moveit_msgs/CartesianTrajectoryPoint uses `duration`).
        // rosidl's adapter maps them to builtin_interfaces; mirror that.
        "time" => Ok(FieldType::Nested {
            schema_name: "Time".to_string(),
            package: Some("builtin_interfaces".to_string()),
            fixed: None,
        }),
        "duration" => Ok(FieldType::Nested {
            schema_name: "Duration".to_string(),
            package: Some("builtin_interfaces".to_string()),
            fixed: None,
        }),
        // Assume anything else is a message type from the same package.
        //
        // `package: None` means "same package as the parent schema". Note:
        // NOTHING EVER FILLS IT IN. `resolve_fixed_nested`'s rewrite phase
        // mutates only the `fixed` flag; `resolve.rs::lookup` resolves a bare
        // name to a schema INDEX for fixedness/layout purposes but never
        // writes the package back onto the `FieldType`. So `None` survives
        // into `canonical_str()` (which renders the BARE name) and therefore
        // into `MessageSchema::schema_hash`.
        //
        // Consequence, stated because it is a live wire-skew mechanism: a
        // bare `Vector3` and a qualified `geometry_msgs/Vector3` hash
        // DIFFERENTLY even after full resolution. `rmw_cerulion` always
        // produces `Some(pkg)` (it reads the rosidl introspection
        // namespace), so the qualified form is the one that interoperates,
        // while upstream ROS `.msg` text writes intra-package references
        // bare.
        //
        // STATUS — `native_ros2_messages` is now UNIFORMLY QUALIFIED. It
        // used to carry 80 bare declarations across 48 of its 254 files, so
        // an rmw publisher of any of those 48 types produced a hash the
        // `FrameWalker` index did not contain and the topic rendered
        // nothing; /3 qualified all 80 after a live rmw capture
        // confirmed, on 5 sampled types, that the qualified hash is exactly
        // what a stock `rclpy` publisher under `RMW_IMPLEMENTATION=
        // rmw_cerulion` puts on the wire (all 64 bits, with two no-bare-ref
        // controls unchanged).
        //
        // What HOLDS that line is `upstream_drift_test.rs::
        // the_vendored_corpus_declares_no_bare_nested_refs`, a
        // zero-tolerance gate: ANY bare ref re-entering the corpus fails it
        // loudly. The upstream-drift gate itself still cannot see this
        // class — its normalizer collapses bare and qualified on BOTH sides
        // so upstream's bare intra-package convention does not swamp the
        // field-list class it exists to catch (see
        // `upstream_drift_test.rs::normalize_type`) — which is precisely
        // why the zero-tolerance gate is a SEPARATE test.
        //
        // Note this is a property of the VENDORED corpus only. A `.msg`
        // acquired at runtime (its wire rung, a workspace `.msg`
        // store) can still carry bare refs, and for those the skew above is
        // live: nothing normalizes them before `schema_hash`.
        other => Ok(FieldType::Nested {
            schema_name: other.to_string(),
            package: None,
            fixed: None,
        }),
    }
}

/// Parse errors for .msg files.
#[derive(Debug)]
pub enum ParseError {
    /// Invalid line format
    InvalidLine { line_num: usize, message: String },

    /// Invalid type specification
    InvalidType { line_num: usize, message: String },
}

impl std::fmt::Display for ParseError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::InvalidLine { line_num, message } => {
                write!(f, "line {line_num}: {message}")
            }
            Self::InvalidType { line_num, message } => {
                write!(f, "line {line_num}: invalid type: {message}")
            }
        }
    }
}

impl std::error::Error for ParseError {}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parse_time_msg() {
        let msg = r#"
# Time represents a specific point in time
int32 sec
uint32 nanosec
"#;
        let schema = parse_rosmsg(msg, "Time", None).unwrap();
        assert_eq!(schema.name, "Time");
        assert_eq!(schema.fields.len(), 2);
        assert_eq!(schema.fields[0].name, "sec");
        assert_eq!(schema.fields[0].field_type, FieldType::I32);
        assert_eq!(schema.fields[1].name, "nanosec");
        assert_eq!(schema.fields[1].field_type, FieldType::U32);
    }

    #[test]
    fn test_parse_header_msg() {
        let msg = r#"
# Standard metadata for higher-level stamped data types.
builtin_interfaces/Time stamp
string frame_id
"#;
        let schema = parse_rosmsg(msg, "Header", None).unwrap();
        assert_eq!(schema.fields.len(), 2);

        // Check nested type
        match &schema.fields[0].field_type {
            FieldType::Nested { schema_name, .. } => {
                assert_eq!(schema_name, "Time");
            }
            _ => panic!("expected Nested type"),
        }

        assert_eq!(schema.fields[1].field_type, FieldType::String);
    }

    #[test]
    fn test_parse_image_msg() {
        let msg = r#"
# This message contains an uncompressed image
std_msgs/Header header
uint32 height
uint32 width
string encoding
uint8 is_bigendian
uint32 step
uint8[] data
"#;
        let schema = parse_rosmsg(msg, "Image", None).unwrap();
        assert_eq!(schema.fields.len(), 7);

        // Check header is nested
        assert!(matches!(
            &schema.fields[0].field_type,
            FieldType::Nested { schema_name, .. } if schema_name == "Header"
        ));

        // Check data is dynamic array
        assert!(matches!(
            &schema.fields[6].field_type,
            FieldType::DynamicArray { element_type } if **element_type == FieldType::U8
        ));
    }

    #[test]
    fn test_parse_fixed_array() {
        let msg = r#"
float64[9] covariance
"#;
        let schema = parse_rosmsg(msg, "Test", None).unwrap();
        assert!(matches!(
            &schema.fields[0].field_type,
            FieldType::FixedArray { element_type, length: 9 } if **element_type == FieldType::F64
        ));
    }

    #[test]
    fn test_skip_constants() {
        let msg = r#"
uint8 INT8=1
uint8 UINT8=2
string name
"#;
        let schema = parse_rosmsg(msg, "PointField", None).unwrap();
        // Constants should be skipped, only 'name' field remains
        assert_eq!(schema.fields.len(), 1);
        assert_eq!(schema.fields[0].name, "name");
    }

    /// The schema hash is computed over the
    /// QUALIFIED name (recipe 3 feeds `qualified_name()`), so identically
    /// named messages in different packages never collide on the wire.
    #[test]
    fn test_fqn_schema_hash_distinguishes_packages() {
        use crate::wire::fnv1a_hash;

        let mesh = "geometry_msgs/Point[] vertices\n";

        let shape_mesh = parse_rosmsg(mesh, "Mesh", Some("shape_msgs")).unwrap();
        let moveit_mesh = parse_rosmsg(mesh, "Mesh", Some("moveit_msgs")).unwrap();

        // Same bare name + same content — but the qualified name feeds the
        // recipe-3 hash, so the wire hashes MUST differ across packages.
        assert_ne!(shape_mesh.schema_hash(), moveit_mesh.schema_hash());
        assert_eq!(shape_mesh.qualified_name(), "shape_msgs/Mesh");
        assert_eq!(shape_mesh.package.as_deref(), Some("shape_msgs"));

        // Recipe 3 is layout-sensitive: the hash folds in wire_fixed_size +
        // per-field canonical_str, so it is NOT a name-only FNV over the
        // qualified name (the superseded recipe-1/#55 form).
        assert_ne!(shape_mesh.schema_hash(), fnv1a_hash(b"shape_msgs/Mesh"));

        // Package-less parses hash over the bare name (workspace YAML
        // schemas live in a flat namespace) — also layout-sensitive, hence
        // distinct from a bare-name FNV and from the packaged hashes.
        let bare = parse_rosmsg(mesh, "Mesh", None).unwrap();
        assert_eq!(bare.qualified_name(), "Mesh");
        assert_eq!(bare.package, None);
        assert_ne!(bare.schema_hash(), fnv1a_hash(b"Mesh"));
        assert_ne!(bare.schema_hash(), shape_mesh.schema_hash());
    }

    /// Qualified nested references keep their package; bare
    /// references carry `None` (= same package as parent).
    #[test]
    fn test_nested_reference_package_preserved() {
        let msg = r#"
geometry_msgs/Pose pose
Time stamp
"#;
        let schema = parse_rosmsg(msg, "Test", Some("test_msgs")).unwrap();

        assert!(matches!(
            &schema.fields[0].field_type,
            FieldType::Nested { schema_name, package, .. }
                if schema_name == "Pose" && package.as_deref() == Some("geometry_msgs")
        ));
        assert!(matches!(
            &schema.fields[1].field_type,
            FieldType::Nested { schema_name, package, .. }
                if schema_name == "Time" && package.is_none()
        ));
    }

    /// ROS2 IDL bounded types: `string<=N` and `T[<=N]` are upper-bounded,
    /// wire-wise plain variable-length — needed for moveit_msgs.
    #[test]
    fn test_parse_bounded_types() {
        let msg = r#"
string<=64 name
int32[<=8] ids
wstring<=10 wname
string<=4[<=3] tags
"#;
        let schema = parse_rosmsg(msg, "Bounded", None).unwrap();
        assert_eq!(schema.fields.len(), 4);
        assert_eq!(schema.fields[0].field_type, FieldType::String);
        assert!(matches!(
            &schema.fields[1].field_type,
            FieldType::DynamicArray { element_type } if **element_type == FieldType::I32
        ));
        assert_eq!(schema.fields[2].field_type, FieldType::String);
        assert!(matches!(
            &schema.fields[3].field_type,
            FieldType::DynamicArray { element_type } if **element_type == FieldType::String
        ));
    }

    /// Malformed qualified references are rejected, not silently mangled.
    #[test]
    fn test_invalid_qualified_reference_rejected() {
        assert!(parse_rosmsg("a/b/c field\n", "Bad", None).is_err());
        assert!(parse_rosmsg("/Pose field\n", "Bad", None).is_err());
        assert!(parse_rosmsg("geometry_msgs/ field\n", "Bad", None).is_err());
    }

    /// The constant-skip logic must skip every
    /// constant SHAPE (tight `=`, spaced `=`, quoted values with spaces)
    /// while keeping default-bearing FIELDS. A regression here is silent —
    /// constants would become phantom fields without any test going red.
    #[test]
    fn test_constant_shapes_skipped_fields_kept() {
        let msg = r#"
uint8 SPACED = 1
uint8 TIGHT=2
string QUOTED="a b c"
string QUOTED_SPACED = "a b"
int32 real_field 5
float64 plain
"#;
        let schema = parse_rosmsg(msg, "C", None).unwrap();
        assert_eq!(schema.fields.len(), 2, "constants must be skipped");
        // The default-bearing line is a FIELD (with its default captured).
        assert_eq!(schema.fields[0].name, "real_field");
        assert_eq!(
            schema.fields[0].default_literal,
            Some(DefaultLiteral::Int(5))
        );
        assert_eq!(schema.fields[1].name, "plain");
        assert_eq!(schema.fields[1].default_literal, None);
    }

    /// ROS1-style lowercase `time`/`duration`
    /// builtins map to builtin_interfaces (rosidl adapter parity). The
    /// `duration` arm is exercised by vendored moveit_msgs; `time` has no
    /// vendored instance, so pin both here.
    #[test]
    fn test_time_duration_builtins_map_to_builtin_interfaces() {
        let schema = parse_rosmsg(
            "time stamp
duration timeout
",
            "T",
            Some("p"),
        )
        .unwrap();
        assert!(matches!(
            &schema.fields[0].field_type,
            FieldType::Nested { schema_name, package, .. }
                if schema_name == "Time" && package.as_deref() == Some("builtin_interfaces")
        ));
        assert!(matches!(
            &schema.fields[1].field_type,
            FieldType::Nested { schema_name, package, .. }
                if schema_name == "Duration" && package.as_deref() == Some("builtin_interfaces")
        ));
    }

    /// Scalar defaults are captured + validated; trailing
    /// comments are NOT defaults; invalid defaults are loud errors.
    #[test]
    fn test_scalar_defaults_captured_and_validated() {
        let msg = r#"
float64 w 1
float64 x 0
int8 kind -1
bool flag true
int32 plain
uint32 commented # speed = 3
"#;
        let schema = parse_rosmsg(msg, "D", None).unwrap();
        assert_eq!(
            schema.fields[0].default_literal,
            Some(DefaultLiteral::Float(1.0))
        );
        assert_eq!(
            schema.fields[1].default_literal,
            Some(DefaultLiteral::Float(0.0))
        );
        assert!(schema.fields[1].default_literal.unwrap().is_zero());
        assert_eq!(
            schema.fields[2].default_literal,
            Some(DefaultLiteral::Int(-1))
        );
        assert_eq!(
            schema.fields[3].default_literal,
            Some(DefaultLiteral::Bool(true))
        );
        assert_eq!(schema.fields[4].default_literal, None);
        // A trailing comment token is not a default.
        assert_eq!(schema.fields[5].default_literal, None);

        // Invalid defaults are errors, not silently dropped.
        assert!(parse_rosmsg(
            "bool b maybe
",
            "Bad",
            None
        )
        .is_err());
        assert!(parse_rosmsg(
            "int32 x fast
",
            "Bad",
            None
        )
        .is_err());
        assert!(parse_rosmsg(
            "float64 y abc
",
            "Bad",
            None
        )
        .is_err());

        // Non-finite floats parse via f64::from_str but `{:?}` renders
        // them as invalid Rust literals — reject at parse time with the
        // .msg line number.
        assert!(parse_rosmsg("float64 y inf\n", "Bad", None).is_err());
        assert!(parse_rosmsg("float64 y -inf\n", "Bad", None).is_err());
        assert!(parse_rosmsg("float64 y nan\n", "Bad", None).is_err());
        // 1e999 saturates to inf in f64::from_str.
        assert!(parse_rosmsg("float64 y 1e999\n", "Bad", None).is_err());

        // rosidl bool literals are case-insensitive (Python-style
        // True/False appear in upstream .msg files).
        let s = parse_rosmsg("bool a True\nbool b False\n", "B", None).unwrap();
        assert_eq!(
            s.fields[0].default_literal,
            Some(DefaultLiteral::Bool(true))
        );
        assert_eq!(
            s.fields[1].default_literal,
            Some(DefaultLiteral::Bool(false))
        );

        // String/array defaults are unsupported and dropped (not errors).
        let s = parse_rosmsg(
            "string name hello
",
            "S",
            None,
        )
        .unwrap();
        assert_eq!(s.fields[0].default_literal, None);
    }

    #[test]
    fn test_byte_maps_to_u8_ros2_jazzy_semantics() {
        // ROS2 Jazzy: `byte` is octet/uint8. The old `byte` -> int8
        // mapping was the ROS1 convention (a hardening fix).
        let msg = "byte data\n";
        let schema = parse_rosmsg(msg, "ByteProbe", None).unwrap();
        assert_eq!(schema.fields[0].field_type, FieldType::U8);
        // And it must agree with FieldType::parse's mapping.
        assert_eq!(FieldType::parse("byte").unwrap(), FieldType::U8);
    }

    #[test]
    fn test_parse_pose_array() {
        let msg = r#"
std_msgs/Header header
geometry_msgs/Pose[] poses
"#;
        let schema = parse_rosmsg(msg, "PoseArray", None).unwrap();
        assert_eq!(schema.fields.len(), 2);

        // Check poses is dynamic array of nested Pose
        match &schema.fields[1].field_type {
            FieldType::DynamicArray { element_type } => match element_type.as_ref() {
                FieldType::Nested { schema_name, .. } => {
                    assert_eq!(schema_name, "Pose");
                }
                _ => panic!("expected Nested element type"),
            },
            _ => panic!("expected DynamicArray"),
        }
    }
}
