// SPDX-License-Identifier: AGPL-3.0-only
//! Messages as text for the verbs that send or show one message:
//! `topic pub` and `service call` read a message from JSON or YAML,
//! `service call` prints a response as one JSON document, and `topic pub`,
//! `service call` and `topic snapshot` share one exit-code contract
//! ([`VerbError::exit_code`]).
//!
//! The text form is the [message YAML](cerulion_core::dynamic) form with one
//! difference: a field the text leaves out takes the value the `.msg` file
//! declares for it, or its type's zero value (`0`, `false`, `""`, an empty
//! array, a fixed array of zeros, a nested message of defaults). JSON is
//! read as YAML, so `{"data": 1}` and `data: 1` are the same message.

use cerulion_core::codegen::{
    DefaultLiteral, FieldType, FrameWalker, MessageSchema, ScalarLiteral,
};
use cerulion_core::dynamic::{message_to_yaml, yaml_to_message, DynamicError, MAX_YAML_DEPTH};
use cerulion_core::wire::{StepTag, WireHeader};
use serde_yaml::{Mapping, Number, Value};

/// Exit code for a value that does not fit its field, or a frame that cannot
/// be encoded, decoded or converted.
pub const EXIT_ENCODE: u8 = 1;
/// Exit code for a usage error: a malformed or unknown type, text that is
/// not JSON or YAML, a bad flag value. Clap's own usage errors exit 2 too.
pub const EXIT_USAGE: u8 = 2;
/// Exit code for nothing answering in time: no service server, no response,
/// no frame.
pub const EXIT_UNAVAILABLE: u8 = 3;

/// A failure of `topic pub`, `service call` or `topic snapshot`, carrying
/// its exit code class.
#[derive(Debug, thiserror::Error)]
pub enum VerbError {
    /// A usage error ([`EXIT_USAGE`]).
    #[error("{0}")]
    Usage(String),
    /// An encode, decode or conversion error ([`EXIT_ENCODE`]).
    #[error("{0}")]
    Encode(String),
    /// Nothing answered within the wait ([`EXIT_UNAVAILABLE`]).
    #[error("{0}")]
    Unavailable(String),
    /// Any other failure: transport, I/O (exit 1).
    #[error("{0}")]
    Internal(String),
}

impl VerbError {
    /// The process exit code for this error.
    pub fn exit_code(&self) -> u8 {
        match self {
            VerbError::Usage(_) => EXIT_USAGE,
            VerbError::Encode(_) => EXIT_ENCODE,
            VerbError::Unavailable(_) => EXIT_UNAVAILABLE,
            VerbError::Internal(_) => 1,
        }
    }
}

impl VerbError {
    /// The class token a remote verb error starts with (`usage`, `encode`,
    /// `unavailable`, `internal`), which [`VerbError::from_classed`] reads
    /// back.
    pub fn class(&self) -> &'static str {
        match self {
            VerbError::Usage(_) => "usage",
            VerbError::Encode(_) => "encode",
            VerbError::Unavailable(_) => "unavailable",
            VerbError::Internal(_) => "internal",
        }
    }

    /// `"<class>: <message>"`, the text a remote verb error carries.
    pub fn to_classed(&self) -> String {
        format!("{}: {self}", self.class())
    }

    /// The error a `"<class>: <message>"` text names; text with no known
    /// class token is [`VerbError::Internal`] holding the whole text.
    pub fn from_classed(text: &str) -> Self {
        let (class, message) = text.split_once(": ").unwrap_or(("", text));
        let message = message.to_string();
        match class {
            "usage" => VerbError::Usage(message),
            "encode" => VerbError::Encode(message),
            "unavailable" => VerbError::Unavailable(message),
            "internal" => VerbError::Internal(message),
            _ => VerbError::Internal(text.to_string()),
        }
    }
}

impl From<cerulion_core::TransportError> for VerbError {
    fn from(e: cerulion_core::TransportError) -> Self {
        VerbError::Internal(e.to_string())
    }
}

impl From<std::io::Error> for VerbError {
    fn from(e: std::io::Error) -> Self {
        VerbError::Internal(format!("I/O error: {e}"))
    }
}

/// Which interface kind a type name names: the middle segment of the
/// three-part ROS 2 spelling (`pkg/msg/Type`, `pkg/srv/Type`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InterfaceKind {
    /// A message type (`pkg/Type`, `pkg/msg/Type` or a bare workspace `Type`).
    Message,
    /// A service type (`pkg/Type` or `pkg/srv/Type`).
    Service,
}

impl InterfaceKind {
    fn segment(self) -> &'static str {
        match self {
            InterfaceKind::Message => "msg",
            InterfaceKind::Service => "srv",
        }
    }
}

/// The Cerulion name of a type as typed: `pkg/Type` for `pkg/Type` and the
/// ROS 2 form `pkg/msg/Type` (`pkg/srv/Type` for a service), and a bare
/// `Type` as written. Any other shape, an empty part or a control character
/// is a [`VerbError::Usage`] naming the input.
pub fn qualified_type_name(input: &str, kind: InterfaceKind) -> Result<String, VerbError> {
    let parts: Vec<&str> = input.split('/').collect();
    let well_formed = parts
        .iter()
        .all(|p| !p.is_empty() && !p.chars().any(char::is_control));
    let name = match (well_formed, parts.as_slice()) {
        (true, [ty]) => Some((*ty).to_string()),
        (true, [pkg, ty]) => Some(format!("{pkg}/{ty}")),
        (true, [pkg, segment, ty]) if *segment == kind.segment() => Some(format!("{pkg}/{ty}")),
        _ => None,
    };
    name.ok_or_else(|| {
        VerbError::Usage(format!(
            "malformed type name '{}': expected <package>/<Type> or <package>/{}/<Type>, \
             for example {}",
            input.escape_debug(),
            kind.segment(),
            match kind {
                InterfaceKind::Message => "geometry_msgs/msg/Twist",
                InterfaceKind::Service => "std_srvs/srv/Trigger",
            }
        ))
    })
}

/// Encode the JSON or YAML mapping `text` as one wire frame of the type
/// `schema` (a name `walker` knows), header included, with `timestamp_ns` in
/// the header and sequence 0. Omitted fields take their declared default or
/// zero value (see the module docs).
///
/// Errors: text that is not one JSON or YAML document is a
/// [`VerbError::Usage`]; a value that does not fit its field, an unknown
/// field name, a wrong array length and every other codec refusal is a
/// [`VerbError::Encode`] naming the field path.
pub fn message_frame(
    walker: &FrameWalker,
    schema: &str,
    text: &str,
    timestamp_ns: u64,
) -> Result<Vec<u8>, VerbError> {
    let target = walker
        .schema_for_name(schema)
        .ok_or_else(|| VerbError::Usage(format!("unknown message type '{schema}'")))?;
    let mut value = parse_message_text(text)?;
    fill_omitted_fields(walker, target, &mut value, 0);
    let filled = serde_yaml::to_string(&value)
        .map_err(|e| VerbError::Encode(format!("could not re-serialize the message: {e}")))?;
    yaml_to_message(walker, schema, filled.as_bytes(), timestamp_ns).map_err(codec_error)
}

/// One JSON or YAML document as a value tree; empty text or `null` is the
/// empty mapping.
fn parse_message_text(text: &str) -> Result<Value, VerbError> {
    let value: Value = serde_yaml::from_str(text).map_err(|e| {
        VerbError::Usage(format!("the message is not one JSON or YAML document: {e}"))
    })?;
    Ok(match value {
        Value::Null => Value::Mapping(Mapping::new()),
        other => other,
    })
}

/// A codec refusal as a verb error: the input-shape refusals are usage
/// errors, everything else is an encode error.
fn codec_error(e: DynamicError) -> VerbError {
    match e {
        DynamicError::UnknownSchema(_)
        | DynamicError::YamlSyntax(_)
        | DynamicError::YamlInputNotUtf8 { .. }
        | DynamicError::YamlInputTooLarge { .. }
        | DynamicError::YamlAnchorOrAlias { .. } => VerbError::Usage(e.to_string()),
        other => VerbError::Encode(other.to_string()),
    }
}

/// Insert every field of `schema` that the mapping `value` leaves out, with
/// its default, and recurse into the nested messages it does give. A value
/// that is not a mapping is left as it is, for the codec to refuse with its
/// field path. Past [`MAX_YAML_DEPTH`] nothing is filled, so a schema that
/// refers to itself stops here and the codec reports the depth.
fn fill_omitted_fields(
    walker: &FrameWalker,
    schema: &MessageSchema,
    value: &mut Value,
    depth: usize,
) {
    if depth > MAX_YAML_DEPTH {
        return;
    }
    let Value::Mapping(map) = value else {
        return;
    };
    let parent = schema.package.as_deref();
    for field in &schema.fields {
        let key = Value::String(field.name.clone());
        match map.get_mut(&key) {
            Some(given) => fill_given(walker, &field.field_type, given, parent, depth),
            None => {
                let default = match &field.default_literal {
                    Some(literal) => literal_value(literal),
                    None => zero_value(walker, &field.field_type, parent, depth),
                };
                map.insert(key, default);
            }
        }
    }
}

/// Recurse into a given value of type `ft` that holds messages: a nested
/// message, or an array of them.
fn fill_given(
    walker: &FrameWalker,
    ft: &FieldType,
    value: &mut Value,
    parent: Option<&str>,
    depth: usize,
) {
    match ft {
        FieldType::Nested {
            schema_name,
            package,
            ..
        } => {
            if let Some((_, nested)) = walker.nested_target(schema_name, package, parent) {
                fill_omitted_fields(walker, nested, value, depth + 1);
            }
        }
        FieldType::FixedArray { element_type, .. } | FieldType::DynamicArray { element_type } => {
            if let Value::Sequence(items) = value {
                for item in items {
                    fill_given(walker, element_type, item, parent, depth + 1);
                }
            }
        }
        _ => {}
    }
}

/// The zero value of a field of type `ft`: `false`, `0`, `0.0`, `""`, `[]`,
/// a fixed array of zero elements, or a nested message of defaults.
fn zero_value(walker: &FrameWalker, ft: &FieldType, parent: Option<&str>, depth: usize) -> Value {
    match ft {
        FieldType::Bool => Value::Bool(false),
        FieldType::I8
        | FieldType::U8
        | FieldType::I16
        | FieldType::U16
        | FieldType::I32
        | FieldType::U32
        | FieldType::I64
        | FieldType::U64 => Value::Number(Number::from(0u64)),
        FieldType::F32 | FieldType::F64 => Value::Number(Number::from(0.0f64)),
        FieldType::String | FieldType::StringFixed(_) => Value::String(String::new()),
        FieldType::Bytes | FieldType::DynamicArray { .. } => Value::Sequence(Vec::new()),
        FieldType::FixedArray {
            element_type,
            length,
        } => {
            let element = zero_value(walker, element_type, parent, depth + 1);
            Value::Sequence(vec![element; *length])
        }
        FieldType::Nested { .. } => {
            let mut message = Value::Mapping(Mapping::new());
            fill_given(walker, ft, &mut message, parent, depth);
            message
        }
    }
}

/// A `.msg` default as a YAML value.
fn literal_value(literal: &DefaultLiteral) -> Value {
    match literal {
        DefaultLiteral::Scalar(s) => scalar_value(s),
        DefaultLiteral::Array(items) => Value::Sequence(items.iter().map(scalar_value).collect()),
    }
}

fn scalar_value(s: &ScalarLiteral) -> Value {
    match s {
        ScalarLiteral::Bool(b) => Value::Bool(*b),
        // The `.msg` parser checks an integer default fits its field, so
        // every one fits `i64` or `u64`.
        ScalarLiteral::Int(i) => match (i64::try_from(*i), u64::try_from(*i)) {
            (Ok(v), _) => Value::Number(Number::from(v)),
            (_, Ok(v)) => Value::Number(Number::from(v)),
            _ => Value::String(i.to_string()),
        },
        ScalarLiteral::Float(f) => Value::Number(Number::from(*f)),
        ScalarLiteral::Str(text) => Value::String(text.clone()),
    }
}

/// Render a message body (`[fixed][offset table][variable]`, no wire
/// header: what a service frame carries after its envelope) of the type
/// whose hash is `schema_hash` as one line of JSON, fields in declaration
/// order. Integers keep every bit; a NaN or infinite float is the string
/// `"NaN"`, `"inf"` or `"-inf"`.
pub fn body_to_json(
    walker: &FrameWalker,
    schema_hash: u64,
    body: &[u8],
) -> Result<String, VerbError> {
    let layout = walker.layout_for_hash(schema_hash).ok_or_else(|| {
        VerbError::Encode(format!(
            "no known type has the schema hash 0x{schema_hash:016x}"
        ))
    })?;
    let total = WireHeader::SIZE + body.len();
    let total_size = u32::try_from(total).map_err(|_| {
        VerbError::Encode(format!("a {total}-byte message passes the u32 size field"))
    })?;
    let mut frame = vec![0u8; WireHeader::SIZE];
    WireHeader {
        schema_hash,
        total_size,
        offset_table_offset: (WireHeader::SIZE + layout.fixed_size) as u32,
        offset_table_count: layout.variable_fields.len() as u32,
        sequence: 0,
        timestamp_ns: 0,
        step_tag: StepTag::UNTAGGED,
    }
    .write_to_buf(&mut frame);
    frame.extend_from_slice(body);
    frame_to_json(walker, &frame)
}

/// Render a whole wire frame (header included) as one line of JSON, the
/// type picked by its header's schema hash, like [`body_to_json`].
pub fn frame_to_json(walker: &FrameWalker, frame: &[u8]) -> Result<String, VerbError> {
    let yaml = message_to_yaml(walker, frame).map_err(|e| VerbError::Encode(e.to_string()))?;
    yaml_to_json(&yaml)
}

/// One YAML document (the codec's output) as one line of JSON.
fn yaml_to_json(yaml: &str) -> Result<String, VerbError> {
    let value: Value = serde_yaml::from_str(yaml)
        .map_err(|e| VerbError::Encode(format!("could not read the decoded message back: {e}")))?;
    let mut out = String::new();
    write_json(&value, &mut out);
    Ok(out)
}

/// Write `value` as compact JSON, mapping keys in their YAML order.
fn write_json(value: &Value, out: &mut String) {
    match value {
        Value::Null => out.push_str("null"),
        Value::Bool(b) => out.push_str(if *b { "true" } else { "false" }),
        Value::Number(n) => out.push_str(&json_number(n)),
        Value::String(s) => out.push_str(&json_string(s)),
        Value::Sequence(items) => {
            out.push('[');
            for (i, item) in items.iter().enumerate() {
                if i > 0 {
                    out.push(',');
                }
                write_json(item, out);
            }
            out.push(']');
        }
        Value::Mapping(map) => {
            out.push('{');
            for (i, (k, v)) in map.iter().enumerate() {
                if i > 0 {
                    out.push(',');
                }
                let key = match k {
                    Value::String(s) => s.clone(),
                    other => serde_yaml::to_string(other)
                        .unwrap_or_default()
                        .trim_end()
                        .to_string(),
                };
                out.push_str(&json_string(&key));
                out.push(':');
                write_json(v, out);
            }
            out.push('}');
        }
        Value::Tagged(t) => write_json(&t.value, out),
    }
}

fn json_number(n: &Number) -> String {
    if let Some(i) = n.as_i64() {
        return i.to_string();
    }
    if let Some(u) = n.as_u64() {
        return u.to_string();
    }
    let f = n.as_f64().unwrap_or(f64::NAN);
    match serde_json::Number::from_f64(f) {
        Some(num) => num.to_string(),
        None if f.is_nan() => json_string("NaN"),
        None if f.is_sign_negative() => json_string("-inf"),
        None => json_string("inf"),
    }
}

fn json_string(s: &str) -> String {
    serde_json::Value::String(s.to_string()).to_string()
}

#[cfg(test)]
mod tests {
    use super::*;
    use cerulion_core::codegen::parse_rosmsg;

    fn walker(defs: &[(&str, &str, &str)]) -> FrameWalker {
        let mut schemas = crate::builtin_schemas();
        for (pkg, name, text) in defs {
            schemas.push(parse_rosmsg(text, name, Some(pkg)).expect("fixture parses"));
        }
        FrameWalker::new(schemas).0
    }

    fn body(frame: &[u8]) -> &[u8] {
        &frame[WireHeader::SIZE..]
    }

    #[test]
    fn type_names_accept_the_cerulion_and_ros_forms_only() {
        let m = InterfaceKind::Message;
        let s = InterfaceKind::Service;
        assert_eq!(
            qualified_type_name("geometry_msgs/Twist", m).unwrap(),
            "geometry_msgs/Twist"
        );
        assert_eq!(
            qualified_type_name("geometry_msgs/msg/Twist", m).unwrap(),
            "geometry_msgs/Twist"
        );
        assert_eq!(qualified_type_name("Scan", m).unwrap(), "Scan");
        assert_eq!(
            qualified_type_name("std_srvs/srv/Trigger", s).unwrap(),
            "std_srvs/Trigger"
        );
        for bad in ["std_srvs/srv/Trigger", "a//b", "", "a/b/c/d", "a/\u{7}"] {
            assert!(
                matches!(qualified_type_name(bad, m), Err(VerbError::Usage(_))),
                "{bad:?} is refused as a message type"
            );
        }
        assert!(matches!(
            qualified_type_name("geometry_msgs/msg/Twist", s),
            Err(VerbError::Usage(_))
        ));
    }

    #[test]
    fn a_partial_twist_encodes_the_given_values_and_zero_for_the_rest() {
        let w = walker(&[]);
        let frame =
            message_frame(&w, "geometry_msgs/Twist", r#"{"linear": {"x": 0.5}}"#, 7).unwrap();
        let header = WireHeader::read_from_buf(&frame).unwrap();
        assert_eq!(header.timestamp_ns, 7);
        assert_eq!(
            header.schema_hash,
            w.schema_hash_for("geometry_msgs/Twist").unwrap()
        );
        // Twist is six float64 in declaration order: linear x y z, angular x y z.
        let mut want = Vec::new();
        for v in [0.5f64, 0.0, 0.0, 0.0, 0.0, 0.0] {
            want.extend_from_slice(&v.to_le_bytes());
        }
        assert_eq!(body(&frame), &want[..]);
    }

    #[test]
    fn json_and_yaml_spell_the_same_message() {
        let w = walker(&[]);
        let json = message_frame(&w, "std_msgs/String", r#"{"data": "hi"}"#, 0).unwrap();
        let yaml = message_frame(&w, "std_msgs/String", "data: \"hi\"\n", 0).unwrap();
        assert_eq!(json, yaml);
        let empty = message_frame(&w, "std_msgs/String", "", 0).unwrap();
        let null = message_frame(&w, "std_msgs/String", "null", 0).unwrap();
        let braces = message_frame(&w, "std_msgs/String", "{}", 0).unwrap();
        assert_eq!(empty, null);
        assert_eq!(empty, braces);
    }

    #[test]
    fn a_declared_default_fills_an_omitted_field() {
        let w = walker(&[(
            "demo_msgs",
            "Gains",
            "float64 kp 2.5\nint32 n -3\nstring tag \"x\"\nbool on true\n",
        )]);
        let frame = message_frame(&w, "demo_msgs/Gains", "{}", 0).unwrap();
        let back = body_to_json(
            &w,
            w.schema_hash_for("demo_msgs/Gains").unwrap(),
            body(&frame),
        )
        .unwrap();
        assert_eq!(back, r#"{"kp":2.5,"n":-3,"tag":"x","on":true}"#);
    }

    #[test]
    fn nested_arrays_of_messages_fill_each_element() {
        let w = walker(&[
            ("demo_msgs", "Point2", "float64 x\nfloat64 y 1.0\n"),
            (
                "demo_msgs",
                "Path2",
                "Point2[] points\nPoint2[2] ends\nstring name\n",
            ),
        ]);
        let frame = message_frame(
            &w,
            "demo_msgs/Path2",
            r#"{"points": [{"x": 3}, {}], "name": "p"}"#,
            0,
        )
        .unwrap();
        let back = body_to_json(
            &w,
            w.schema_hash_for("demo_msgs/Path2").unwrap(),
            body(&frame),
        )
        .unwrap();
        assert_eq!(
            back,
            r#"{"points":[{"x":3.0,"y":1.0},{"x":0.0,"y":1.0}],"ends":[{"x":0.0,"y":1.0},{"x":0.0,"y":1.0}],"name":"p"}"#
        );
    }

    #[test]
    fn an_unknown_field_is_an_encode_error_naming_it() {
        let w = walker(&[]);
        let err =
            message_frame(&w, "geometry_msgs/Twist", r#"{"linear": {"q": 1}}"#, 0).unwrap_err();
        assert!(matches!(err, VerbError::Encode(_)), "{err:?}");
        assert_eq!(err.exit_code(), EXIT_ENCODE);
        assert!(err.to_string().contains("linear.q"), "{err}");
    }

    #[test]
    fn a_wrong_value_type_is_an_encode_error_naming_the_path() {
        let w = walker(&[]);
        let err = message_frame(
            &w,
            "geometry_msgs/Twist",
            r#"{"angular": {"z": "fast"}}"#,
            0,
        )
        .unwrap_err();
        assert_eq!(err.exit_code(), EXIT_ENCODE);
        assert!(err.to_string().contains("angular.z"), "{err}");
        let err = message_frame(&w, "std_msgs/UInt8", "data: 300", 0).unwrap_err();
        assert_eq!(err.exit_code(), EXIT_ENCODE);
    }

    #[test]
    fn text_that_is_not_a_document_and_an_unknown_type_are_usage_errors() {
        let w = walker(&[]);
        let err = message_frame(&w, "std_msgs/String", "{\"data\": ", 0).unwrap_err();
        assert_eq!(err.exit_code(), EXIT_USAGE, "{err}");
        let err = message_frame(&w, "nope_msgs/Nope", "{}", 0).unwrap_err();
        assert_eq!(err.exit_code(), EXIT_USAGE, "{err}");
        // A scalar is a document, but not a message: the codec refuses it.
        let err = message_frame(&w, "std_msgs/String", "5", 0).unwrap_err();
        assert_eq!(err.exit_code(), EXIT_ENCODE, "{err}");
    }

    #[test]
    fn json_output_keeps_wide_integers_and_names_non_finite_floats() {
        let w = walker(&[(
            "demo_msgs",
            "Wide",
            "uint64 big\nint64 small\nfloat64 f\nfloat32 g\nuint8[] raw\n",
        )]);
        let frame = message_frame(
            &w,
            "demo_msgs/Wide",
            "big: 18446744073709551615\nsmall: -9223372036854775808\nf: .nan\ng: -.inf\nraw: [1, 255]\n",
            0,
        )
        .unwrap();
        let back = body_to_json(
            &w,
            w.schema_hash_for("demo_msgs/Wide").unwrap(),
            body(&frame),
        )
        .unwrap();
        assert_eq!(
            back,
            r#"{"big":18446744073709551615,"small":-9223372036854775808,"f":"NaN","g":"-inf","raw":[1,255]}"#
        );
        let parsed: serde_json::Value = serde_json::from_str(&back).expect("valid JSON");
        assert_eq!(parsed["raw"][1], 255);
    }

    #[test]
    fn json_strings_are_escaped() {
        let w = walker(&[]);
        let frame = message_frame(&w, "std_msgs/String", r#"{"data": "a\"b\nc"}"#, 0).unwrap();
        let back = body_to_json(
            &w,
            w.schema_hash_for("std_msgs/String").unwrap(),
            body(&frame),
        )
        .unwrap();
        assert_eq!(back, r#"{"data":"a\"b\nc"}"#);
    }

    #[test]
    fn encoding_is_deterministic() {
        let w = walker(&[]);
        let text = r#"{"header": {"frame_id": "map"}, "pose": {"position": {"x": 1}}}"#;
        let a = message_frame(&w, "geometry_msgs/PoseStamped", text, 5).unwrap();
        let b = message_frame(&w, "geometry_msgs/PoseStamped", text, 5).unwrap();
        assert_eq!(a, b);
    }

    #[test]
    fn a_classed_error_text_reads_back_as_its_class() {
        for e in [
            VerbError::Usage("u: x".into()),
            VerbError::Encode("e".into()),
            VerbError::Unavailable("n".into()),
            VerbError::Internal("i".into()),
        ] {
            let back = VerbError::from_classed(&e.to_classed());
            assert_eq!(
                (back.exit_code(), back.to_string()),
                (e.exit_code(), e.to_string())
            );
        }
        let other = VerbError::from_classed("denied by policy");
        assert_eq!(other.exit_code(), 1);
        assert_eq!(other.to_string(), "denied by policy");
    }

    #[test]
    fn a_whole_frame_renders_as_json() {
        let w = walker(&[]);
        let frame = message_frame(&w, "std_msgs/String", r#"{"data": "x"}"#, 0).unwrap();
        assert_eq!(frame_to_json(&w, &frame).unwrap(), r#"{"data":"x"}"#);
    }

    #[test]
    fn an_unknown_hash_is_an_encode_error() {
        let w = walker(&[]);
        let err = body_to_json(&w, 0x1234, &[]).unwrap_err();
        assert_eq!(err.exit_code(), EXIT_ENCODE);
    }
}
