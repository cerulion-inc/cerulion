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

use super::schema::{
    ConstantDef, DefaultLiteral, FieldBounds, FieldDef, FieldType, MessageSchema, ScalarLiteral,
};

/// Parse a ROS2 .msg file content into a MessageSchema.
///
/// Fields keep their default values (scalars, strings and arrays), and
/// constant lines (`uint8 FOO=1`, `string NAME="x"`) land in
/// [`MessageSchema::constants`]. Neither reaches the wire or the schema
/// hash.
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

    for (idx, line) in msg_content.lines().enumerate() {
        let line_num = idx + 1;
        let line = strip_comment(line);

        // Skip empty lines and comment-only lines
        if line.is_empty() {
            continue;
        }

        match split_constant(line) {
            Some((type_token, name, value)) => {
                let constant = parse_constant(type_token, name, value)
                    .map_err(|message| ParseError::InvalidLine { line_num, message })?;
                if schema.constants.iter().any(|c| c.name == constant.name) {
                    return Err(ParseError::InvalidLine {
                        line_num,
                        message: format!("duplicate constant {}", constant.name),
                    });
                }
                schema.constants.push(constant);
            }
            None => schema.add_field(parse_field_line(line, line_num)?),
        }
    }

    Ok(schema)
}

/// Split the first whitespace-separated token off `line`.
fn split_first_token(line: &str) -> (&str, &str) {
    match line.find(char::is_whitespace) {
        Some(i) => (&line[..i], line[i..].trim_start()),
        None => (line, ""),
    }
}

/// `Some((type, name, value))` when `line` declares a constant: the name
/// token is followed (after optional spaces) by `=`. The `=` of a bounded
/// type (`string<=64 name`) lives in the TYPE token, so it never counts.
fn split_constant(line: &str) -> Option<(&str, &str, &str)> {
    let (type_token, rest) = split_first_token(line);
    let name_end = rest
        .find(|c: char| c.is_whitespace() || c == '=')
        .unwrap_or(rest.len());
    let value = rest[name_end..].trim_start().strip_prefix('=')?;
    Some((type_token, &rest[..name_end], value))
}

/// Tracks quoted strings while a value is scanned left to right.
///
/// A quote opens a quoted string only where a value or an array item
/// begins: at the start of the text, or after `[` or `,` and any spaces.
/// An apostrophe inside bare text (`it's`) is plain text. Inside a quoted
/// string, `\\` escapes the next character.
struct QuoteScan {
    quote: Option<char>,
    escaped: bool,
    at_item_start: bool,
}

impl QuoteScan {
    fn new() -> Self {
        Self {
            quote: None,
            escaped: false,
            at_item_start: true,
        }
    }

    /// Feed one character. Returns true when `c` is outside every quoted
    /// string, so the caller may treat it as syntax (the `,` between items).
    fn outside(&mut self, c: char) -> bool {
        if let Some(q) = self.quote {
            if self.escaped {
                self.escaped = false;
            } else if c == '\\' {
                self.escaped = true;
            } else if c == q {
                self.quote = None;
            }
            return false;
        }
        if self.at_item_start && (c == '"' || c == '\'') {
            self.quote = Some(c);
            self.at_item_start = false;
            return false;
        }
        self.at_item_start = match c {
            '[' | ',' => true,
            c if c.is_whitespace() => self.at_item_start,
            _ => false,
        };
        true
    }
}

/// Cut `line` at its first `#` and trim the result.
///
/// rosidl treats the first `#` anywhere on a line as the start of a
/// comment, inside a quoted string or bare text alike, before it reads
/// any value. So `string s a#b` has the default `a`, and
/// `string S="a#b"` is an unterminated string. Reading it the same way
/// keeps a `.msg` file meaning the same thing here as in a ROS 2 build.
fn strip_comment(line: &str) -> &str {
    line.split_once('#').map_or(line, |(code, _)| code).trim()
}

/// Which kind of value a literal belongs to, for the error text.
#[derive(Clone, Copy)]
enum ValueKind {
    Default,
    Constant,
}

impl ValueKind {
    fn word(self) -> &'static str {
        match self {
            Self::Default => "default",
            Self::Constant => "constant",
        }
    }
}

/// True when `name` is a valid rosidl constant name: an upper-case
/// letter, then upper-case letters, digits and single `_` separators, with
/// no trailing `_` (rosidl `[A-Z]([A-Z0-9_]?[A-Z0-9]+)*`). The generator
/// emits the name as a Rust `const` as is, so a lower-case name
/// (`uint8 type=1`) would fail to compile far from the .msg line.
fn is_valid_constant_name(name: &str) -> bool {
    let bytes = name.as_bytes();
    bytes.first().is_some_and(u8::is_ascii_uppercase)
        && bytes
            .iter()
            .all(|b| b.is_ascii_uppercase() || b.is_ascii_digit() || *b == b'_')
        && !name.contains("__")
        && !name.ends_with('_')
}

fn parse_constant(type_token: &str, name: &str, value: &str) -> Result<ConstantDef, String> {
    if !is_valid_constant_name(name) {
        return Err(format!("invalid constant name: {name}"));
    }
    // A constant keeps no bounds: `string_bound` checks its value below.
    let field_type = parse_ros_type(type_token, &mut FieldBounds::default())?;
    if !is_scalar_or_string(&field_type) {
        return Err(format!(
            "constant {name}: type {type_token} is not a primitive type or a string"
        ));
    }
    let value = value.trim();
    if value.is_empty() {
        return Err(format!("constant {name} has no value"));
    }
    let value = parse_scalar(
        &field_type,
        type_token,
        string_bound(type_token),
        value,
        ValueKind::Constant,
    )?;
    Ok(ConstantDef {
        name: name.to_string(),
        field_type,
        value,
    })
}

fn is_scalar_or_string(ft: &FieldType) -> bool {
    matches!(
        ft,
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
    )
}

fn parse_field_line(line: &str, line_num: usize) -> Result<FieldDef, ParseError> {
    let (ros_type, rest) = split_first_token(line);
    let (field_name, default_text) = split_first_token(rest);
    if field_name.is_empty() {
        return Err(ParseError::InvalidLine {
            line_num,
            message: format!("expected 'type name', got: {line}"),
        });
    }

    let mut bounds = FieldBounds::default();
    let field_type =
        parse_ros_type(ros_type, &mut bounds).map_err(|e| ParseError::InvalidType {
            line_num,
            message: e,
        })?;

    let mut field = FieldDef::new(field_name, field_type).with_bounds(bounds);

    // Everything after the name is the default value (`float64 w 1`,
    // `string s "a b"`, `int32[] v [1, 2]`); the comment is already cut.
    if !default_text.is_empty() {
        let default = parse_default(&field.field_type, ros_type, default_text)
            .and_then(|d| refuse_wide_string_default(ros_type, d))
            .map_err(|message| ParseError::InvalidLine { line_num, message })?;
        field = field.with_default(default);
    }

    Ok(field)
}

/// Refuse a non-empty default on a `wstring` field. A `wstring` is carried as
/// a [`FieldType::String`], which the codec writes as UTF-8, not the UTF-16 a
/// ROS 2 peer reads, so a pre-filled value would go on the wire in the wrong
/// encoding. An empty default (`""`, `[]`, `["", ""]`) is the zero value and
/// is kept. A `wstring` constant is still accepted: constants never reach the
/// wire.
fn refuse_wide_string_default(
    type_token: &str,
    default: DefaultLiteral,
) -> Result<DefaultLiteral, String> {
    if !type_token.starts_with("wstring") {
        return Ok(default);
    }
    let non_empty = |v: &ScalarLiteral| matches!(v, ScalarLiteral::Str(s) if !s.is_empty());
    let has_text = match &default {
        DefaultLiteral::Scalar(v) => non_empty(v),
        DefaultLiteral::Array(items) => items.iter().any(non_empty),
    };
    if has_text {
        return Err(format!(
            "a non-empty default on {type_token} is not supported: \
             a wstring is written as UTF-8, not UTF-16"
        ));
    }
    Ok(default)
}

/// Parse + validate a field default against its type. `type_token` is the
/// declared type text, read again for the `<=N` bounds the [`FieldType`]
/// does not keep.
fn parse_default(ft: &FieldType, type_token: &str, text: &str) -> Result<DefaultLiteral, String> {
    let element_type = match ft {
        FieldType::FixedArray { element_type, .. } | FieldType::DynamicArray { element_type } => {
            element_type.as_ref()
        }
        _ => {
            return parse_scalar(
                ft,
                type_token,
                string_bound(type_token),
                text,
                ValueKind::Default,
            )
            .map(DefaultLiteral::Scalar)
        }
    };
    if matches!(element_type, FieldType::Nested { .. }) {
        return Err("default values are not supported on message-type fields".to_string());
    }
    let inner = text
        .strip_prefix('[')
        .and_then(|t| t.strip_suffix(']'))
        .ok_or_else(|| format!("array default must be written [a, b, ...]: {text}"))?;
    let element_token = &type_token[..type_token.find('[').unwrap_or(type_token.len())];
    let bound = string_bound(element_token);
    let items = split_array_items(inner)
        .into_iter()
        .map(|item| parse_scalar(element_type, element_token, bound, item, ValueKind::Default))
        .collect::<Result<Vec<_>, _>>()?;
    match ft {
        FieldType::FixedArray { length, .. } if items.len() != *length => Err(format!(
            "array default has {} elements, the type has {length}",
            items.len()
        )),
        _ => match array_bound(type_token) {
            Some(max) if items.len() > max => Err(format!(
                "array default has {} elements, the bound is {max}",
                items.len()
            )),
            _ => Ok(DefaultLiteral::Array(items)),
        },
    }
}

/// Split the inside of `[...]` at the commas outside quoted items (see
/// [`QuoteScan`]). An empty (or
/// all-space) inside is zero items.
fn split_array_items(inner: &str) -> Vec<&str> {
    if inner.trim().is_empty() {
        return Vec::new();
    }
    let mut items = Vec::new();
    let mut scan = QuoteScan::new();
    let mut start = 0;
    for (i, c) in inner.char_indices() {
        if scan.outside(c) && c == ',' {
            items.push(inner[start..i].trim());
            start = i + 1;
        }
    }
    items.push(inner[start..].trim());
    items
}

/// `N` of a bounded array type `T[<=N]`.
fn array_bound(type_token: &str) -> Option<usize> {
    let open = type_token.find("[<=")?;
    type_token[open + 3..].strip_suffix(']')?.parse().ok()
}

/// The `<=N` bound of a bounded string type, in the unit ROS 2 counts it in.
#[derive(Clone, Copy)]
enum StringBound {
    /// `string<=N`: at most `N` UTF-8 bytes.
    Utf8Bytes(usize),
    /// `wstring<=N`: at most `N` UTF-16 code units.
    Utf16Units(usize),
}

impl StringBound {
    /// `Some(error)` when `s` is longer than the bound.
    fn check(self, s: &str, what: &str) -> Option<String> {
        let (len, max, unit) = match self {
            StringBound::Utf8Bytes(max) => (s.len(), max, "bytes"),
            StringBound::Utf16Units(max) => (s.encode_utf16().count(), max, "UTF-16 units"),
        };
        (len > max).then(|| format!("string {what} is longer than its bound of {max} {unit}"))
    }
}

/// The bound of a bounded string type `string<=N` / `wstring<=N` (no array
/// part).
fn string_bound(type_token: &str) -> Option<StringBound> {
    let base = &type_token[..type_token.find('[').unwrap_or(type_token.len())];
    if let Some(n) = base.strip_prefix("string<=") {
        return n.parse().ok().map(StringBound::Utf8Bytes);
    }
    base.strip_prefix("wstring<=")?
        .parse()
        .ok()
        .map(StringBound::Utf16Units)
}

/// Inclusive value range of an integer field type, and its ROS name.
fn int_range(ft: &FieldType) -> Option<(i128, i128, &'static str)> {
    Some(match ft {
        FieldType::I8 => (i8::MIN.into(), i8::MAX.into(), "int8"),
        FieldType::U8 => (0, u8::MAX.into(), "uint8"),
        FieldType::I16 => (i16::MIN.into(), i16::MAX.into(), "int16"),
        FieldType::U16 => (0, u16::MAX.into(), "uint16"),
        FieldType::I32 => (i32::MIN.into(), i32::MAX.into(), "int32"),
        FieldType::U32 => (0, u32::MAX.into(), "uint32"),
        FieldType::I64 => (i64::MIN.into(), i64::MAX.into(), "int64"),
        FieldType::U64 => (0, u64::MAX.into(), "uint64"),
        _ => return None,
    })
}

/// Parse + validate one scalar or string value for a field type.
fn parse_scalar(
    ft: &FieldType,
    type_token: &str,
    bound: Option<StringBound>,
    token: &str,
    kind: ValueKind,
) -> Result<ScalarLiteral, String> {
    let what = kind.word();
    if let Some((min, max, ros)) = int_range(ft) {
        let out_of_range = || format!("value {token} is out of range for {ros}");
        let v = match parse_integer(token) {
            IntLiteral::Value(v) => v,
            IntLiteral::TooLarge => return Err(out_of_range()),
            IntLiteral::Invalid => return Err(format!("invalid integer {what}: {token}")),
        };
        if v < min || v > max {
            return Err(out_of_range());
        }
        return Ok(ScalarLiteral::Int(v));
    }
    match ft {
        // rosidl's value parser is case-insensitive for bool literals
        // ("True"/"False" Python-style appear in upstream .msg files).
        FieldType::Bool => match token.to_ascii_lowercase().as_str() {
            "true" | "1" => Ok(ScalarLiteral::Bool(true)),
            "false" | "0" => Ok(ScalarLiteral::Bool(false)),
            _ => Err(format!("invalid bool {what}: {token}")),
        },
        FieldType::F32 | FieldType::F64 => {
            let v = token
                .parse::<f64>()
                .map_err(|_| format!("invalid float {what}: {token}"))?;
            // `f64::from_str` accepts "inf"/"NaN" (and saturates 1e999 to
            // inf) — but `{:?}` renders those as `inf`/`NaN`, which are
            // NOT valid Rust literals: the generated code would fail to
            // compile far from the .msg line. Reject here with the line
            // number instead.
            if !v.is_finite() {
                return Err(format!(
                    "non-finite float {what}: {token} (inf/NaN are not supported)"
                ));
            }
            // A float32 literal that rounds to infinity is an
            // `overflowing_literals` compile error in the generated code;
            // refuse it here instead. A literal just past `f32::MAX` that
            // still rounds to it (`3.4028235e38`, how `f32::MAX` prints)
            // is accepted, exactly as rustc accepts it.
            if matches!(ft, FieldType::F32) && !(v as f32).is_finite() {
                return Err(format!("value {token} is out of range for float32"));
            }
            Ok(ScalarLiteral::Float(v))
        }
        FieldType::String => {
            let s = parse_string_literal(token, kind)?;
            // ROS 2 bounds a `string<=N` by its UTF-8 byte length and a
            // `wstring<=N` by its UTF-16 code units, not by characters
            // (`"é"` is two bytes but one UTF-16 unit).
            if let Some(e) = bound.and_then(|b| b.check(&s, what)) {
                return Err(e);
            }
            Ok(ScalarLiteral::Str(s))
        }
        FieldType::Nested { .. } => {
            Err("default values are not supported on message-type fields".to_string())
        }
        _ => Err(format!("type {type_token} takes no {what} value")),
    }
}

/// The result of reading an integer literal.
enum IntLiteral {
    Value(i128),
    /// Well formed, but past every ROS integer type.
    TooLarge,
    Invalid,
}

/// Read an integer the way rosidl does: Python `int(text)`, then
/// `int(text, 0)`. That is an optional sign, then decimal digits (leading
/// zeros allowed) or a `0x`, `0o` or `0b` prefix (any case) and digits of
/// that base, with single `_` separators between digits (`1_000`,
/// `0x_ff`).
fn parse_integer(token: &str) -> IntLiteral {
    let (negative, body) = match token.as_bytes().first() {
        Some(b'-') => (true, &token[1..]),
        Some(b'+') => (false, &token[1..]),
        _ => (false, token),
    };
    let prefix = body.get(..2).map(str::to_ascii_lowercase);
    let (radix, digits) = match prefix.as_deref() {
        Some("0x") => (16, &body[2..]),
        Some("0o") => (8, &body[2..]),
        Some("0b") => (2, &body[2..]),
        _ => (10, body),
    };
    // After a base prefix one `_` may come before the first digit.
    let digits = match digits.strip_prefix('_') {
        Some(rest) if radix != 10 => rest,
        _ => digits,
    };
    if digits.is_empty()
        || digits.starts_with('_')
        || digits.ends_with('_')
        || digits.contains("__")
        || !digits.chars().all(|c| c == '_' || c.is_digit(radix))
    {
        return IntLiteral::Invalid;
    }
    let clean: String = digits.chars().filter(|c| *c != '_').collect();
    match u128::from_str_radix(&clean, radix) {
        Ok(v) => match i128::try_from(v) {
            Ok(v) => IntLiteral::Value(if negative { -v } else { v }),
            Err(_) => IntLiteral::TooLarge,
        },
        Err(_) => IntLiteral::TooLarge,
    }
}

/// A string value: `"..."` or `'...'` (with `\<quote>` and `\\`
/// unescaped), or the bare text as written. A quoted value must end at
/// its first unescaped matching quote: an unescaped quote inside it
/// (`"a"b"`) is an error, as in rosidl, and so is a value whose last
/// quote is escaped (`"abc\"`), which never closes.
fn parse_string_literal(token: &str, kind: ValueKind) -> Result<String, String> {
    let Some(quote) = token.chars().next().filter(|c| *c == '"' || *c == '\'') else {
        return Ok(token.to_string());
    };
    let mut out = String::with_capacity(token.len());
    let mut chars = token.char_indices().skip(1);
    while let Some((i, c)) = chars.next() {
        if c == '\\' {
            match chars.next() {
                Some((_, n)) if n == quote || n == '\\' => out.push(n),
                Some((_, n)) => {
                    out.push(c);
                    out.push(n);
                }
                None => break,
            }
        } else if c == quote {
            if i + c.len_utf8() == token.len() {
                return Ok(out);
            }
            return Err(format!(
                "unescaped quote inside string {}: {token}",
                kind.word()
            ));
        } else {
            out.push(c);
        }
    }
    Err(format!("unterminated string {}: {token}", kind.word()))
}

/// Parse one `.msg` type token. ROS 2 upper bounds (`T[<=N]`,
/// `string<=N`) land in `bounds`; they do not change the wire type.
fn parse_ros_type(ros_type: &str, bounds: &mut FieldBounds) -> Result<FieldType, String> {
    // Check for array types: T[], T[N], or bounded T[<=N]
    if let Some(bracket_pos) = ros_type.find('[') {
        if !ros_type.ends_with(']') {
            return Err(format!("invalid array syntax: {ros_type}"));
        }

        let base_type = &ros_type[..bracket_pos];
        let length_str = &ros_type[bracket_pos + 1..ros_type.len() - 1];

        let element_type = parse_ros_type(base_type, bounds)?;

        if length_str.is_empty() {
            // Dynamic array: T[]
            return Ok(FieldType::DynamicArray {
                element_type: Box::new(element_type),
            });
        } else if let Some(bound_str) = length_str.strip_prefix("<=") {
            // Bounded array: T[<=N] (ROS2 IDL). The bound is an upper
            // limit, not a fixed length — wire-wise it is a dynamic array.
            let bound = bound_str
                .parse::<usize>()
                .map_err(|_| format!("invalid array bound: {length_str}"))?;
            bounds.max_elements = Some(bound);
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
        let bound = bound_str
            .parse::<usize>()
            .map_err(|_| format!("invalid string bound: {ros_type}"))?;
        bounds.max_string_bytes = Some(bound);
        return Ok(FieldType::String);
    }
    // A `wstring` bound counts UTF-16 code units, not the UTF-8 bytes the
    // wire carries, so it is checked but not kept.
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
#[derive(Debug, thiserror::Error)]
pub enum ParseError {
    /// Invalid line format
    #[error("line {line_num}: {message}")]
    InvalidLine { line_num: usize, message: String },

    /// Invalid type specification
    #[error("line {line_num}: invalid type: {message}")]
    InvalidType { line_num: usize, message: String },
}

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
        // The bounds are kept beside the wire type (wstring's is not).
        let b = |max_elements, max_string_bytes| FieldBounds {
            max_elements,
            max_string_bytes,
        };
        let kept: Vec<FieldBounds> = schema.fields.iter().map(|f| f.bounds).collect();
        assert_eq!(
            kept,
            [
                b(None, Some(64)),
                b(Some(8), None),
                b(None, None),
                b(Some(3), Some(4))
            ]
        );
        // The hash is the unbounded twin's: bounds are not wire shape.
        let twin = parse_rosmsg(
            "string name\nint32[] ids\nwstring wname\nstring[] tags\n",
            "Bounded",
            None,
        )
        .unwrap();
        assert_eq!(schema.schema_hash(), twin.schema_hash());
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
            Some(DefaultLiteral::Scalar(ScalarLiteral::Int(5)))
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
            Some(DefaultLiteral::Scalar(ScalarLiteral::Float(1.0)))
        );
        assert_eq!(
            schema.fields[1].default_literal,
            Some(DefaultLiteral::Scalar(ScalarLiteral::Float(0.0)))
        );
        assert!(schema.fields[1]
            .default_literal
            .as_ref()
            .is_some_and(|d| d.is_zero_for(&FieldType::F64)));
        assert_eq!(
            schema.fields[2].default_literal,
            Some(DefaultLiteral::Scalar(ScalarLiteral::Int(-1)))
        );
        assert_eq!(
            schema.fields[3].default_literal,
            Some(DefaultLiteral::Scalar(ScalarLiteral::Bool(true)))
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
            Some(DefaultLiteral::Scalar(ScalarLiteral::Bool(true)))
        );
        assert_eq!(
            s.fields[1].default_literal,
            Some(DefaultLiteral::Scalar(ScalarLiteral::Bool(false)))
        );

        // An unquoted string default is kept as written (rosidl reads it
        // the same way).
        let s = parse_rosmsg("string name hello\n", "S", None).unwrap();
        assert_eq!(
            s.fields[0].default_literal,
            Some(DefaultLiteral::Scalar(ScalarLiteral::Str("hello".into())))
        );
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

#[cfg(test)]
mod constant_and_default_tests {
    use super::*;
    use crate::codegen::schema::ConstantDef;

    fn one(text: &str) -> MessageSchema {
        parse_rosmsg(text, "M", Some("p")).unwrap()
    }

    fn err(text: &str) -> String {
        parse_rosmsg(text, "M", Some("p")).unwrap_err().to_string()
    }

    fn constant(name: &str, field_type: FieldType, value: ScalarLiteral) -> ConstantDef {
        ConstantDef {
            name: name.into(),
            field_type,
            value,
        }
    }

    #[test]
    fn constants_are_captured_in_declaration_order_and_are_not_fields() {
        let s = one("uint8 PARAMETER_NOT_SET=0\n\
             int32 SPACED = -7   # a comment\n\
             float64 RATE=2.5\n\
             bool ON=True\n\
             byte RAW=255\n\
             string MODE = \"AUTO\"  # a comment\n\
             string BARE=plain text\n\
             uint64 BIG=18446744073709551615\n\
             string name\n");
        assert_eq!(s.fields.len(), 1);
        assert_eq!(s.fields[0].name, "name");
        assert_eq!(
            s.constants,
            vec![
                constant("PARAMETER_NOT_SET", FieldType::U8, ScalarLiteral::Int(0)),
                constant("SPACED", FieldType::I32, ScalarLiteral::Int(-7)),
                constant("RATE", FieldType::F64, ScalarLiteral::Float(2.5)),
                constant("ON", FieldType::Bool, ScalarLiteral::Bool(true)),
                constant("RAW", FieldType::U8, ScalarLiteral::Int(255)),
                constant("MODE", FieldType::String, ScalarLiteral::Str("AUTO".into())),
                constant(
                    "BARE",
                    FieldType::String,
                    ScalarLiteral::Str("plain text".into())
                ),
                constant(
                    "BIG",
                    FieldType::U64,
                    ScalarLiteral::Int(18446744073709551615)
                ),
            ]
        );
    }

    #[test]
    fn constants_do_not_change_the_schema_hash() {
        let with = one("uint8 A=1\nstring S=\"x\"\nint32 v\n");
        let without = one("int32 v\n");
        assert_eq!(with.schema_hash(), without.schema_hash());
    }

    #[test]
    fn quoted_string_constants_unescape_their_quote_and_backslash() {
        let s = one("string A='it\\'s'\nstring B=\"a\\\\b\"\nstring C=\"\"\n");
        assert_eq!(s.constants[0].value, ScalarLiteral::Str("it's".into()));
        assert_eq!(s.constants[1].value, ScalarLiteral::Str("a\\b".into()));
        assert_eq!(s.constants[2].value, ScalarLiteral::Str(String::new()));
        // Any other backslash pair is kept as written, both characters.
        let other = one("string D=\"a\\nb\\'c\"\n");
        assert_eq!(
            other.constants[0].value,
            ScalarLiteral::Str("a\\nb\\'c".into())
        );
    }

    #[test]
    fn bad_constants_are_errors_that_name_the_line() {
        assert_eq!(
            err("int32 a\nuint8 X=256\n"),
            "line 2: value 256 is out of range for uint8"
        );
        assert_eq!(
            err("int8 X=-129\n"),
            "line 1: value -129 is out of range for int8"
        );
        assert_eq!(
            err("uint8 X=-1\n"),
            "line 1: value -1 is out of range for uint8"
        );
        assert_eq!(
            err("uint8 X=one\n"),
            "line 1: invalid integer constant: one"
        );
        assert_eq!(err("bool X=yes\n"), "line 1: invalid bool constant: yes");
        assert_eq!(
            err("float32 X=nan\n"),
            "line 1: non-finite float constant: nan (inf/NaN are not supported)"
        );
        assert_eq!(err("uint8 X=\n"), "line 1: constant X has no value");
        assert_eq!(
            err("uint8 X=1\nuint8 X=2\n"),
            "line 2: duplicate constant X"
        );
        assert_eq!(err("uint8 9X=1\n"), "line 1: invalid constant name: 9X");
        // rosidl constant names are upper case; the generator emits them as
        // Rust consts as is, so a lower-case name is refused at parse time.
        for bad in ["type", "lower", "Mixed", "_X", "A__B", "A_", "X-Y"] {
            assert_eq!(
                err(&format!("int32 a\nuint8 {bad}=1\n")),
                format!("line 2: invalid constant name: {bad}")
            );
        }
        // rosidl reads any `=` after the name as a constant separator, so a
        // bare string default that starts with `=` is a constant named `s`,
        // and that name is refused (rosidl raises the same error).
        assert_eq!(err("string s =foo\n"), "line 1: invalid constant name: s");
        let ok = one("uint8 A=1\nuint8 A_B2=2\nuint8 X9_Y_Z=3\nuint8 B1=4\n");
        assert_eq!(
            ok.constants
                .iter()
                .map(|c| c.name.as_str())
                .collect::<Vec<_>>(),
            ["A", "A_B2", "X9_Y_Z", "B1"]
        );
        assert_eq!(
            err("uint8[] X=[1]\n"),
            "line 1: constant X: type uint8[] is not a primitive type or a string"
        );
        assert_eq!(
            err("geometry_msgs/Point X=1\n"),
            "line 1: constant X: type geometry_msgs/Point is not a primitive type or a string"
        );
        assert_eq!(
            err("string X=\"open\n"),
            "line 1: unterminated string constant: \"open"
        );
        assert_eq!(
            err("float32 X=1e39\n"),
            "line 1: value 1e39 is out of range for float32"
        );
        assert_eq!(
            err("float32 x -1e39\n"),
            "line 1: value -1e39 is out of range for float32"
        );
        // rustc rounds a float32 literal to the nearest f32 and refuses it
        // only when that is infinity: `3.4028235e38` (how `f32::MAX`
        // prints) rounds to `f32::MAX` and is accepted; `3.5e38` is not.
        assert_eq!(
            one("float32 X=3.4028235e38\n").constants[0].value,
            ScalarLiteral::Float(3.402_823_5e38)
        );
        assert_eq!(
            err("float32 X=3.5e38\n"),
            "line 1: value 3.5e38 is out of range for float32"
        );
        assert_eq!(
            one("float32 X=3.4e38\n").constants[0].value,
            ScalarLiteral::Float(3.4e38)
        );
        // Exactly the largest finite float32 is still in range, at both signs.
        assert_eq!(
            one("float32 X=3.4028234663852886e38\nfloat32 Y=-3.4028234663852886e38\n")
                .constants
                .iter()
                .map(|c| c.value.clone())
                .collect::<Vec<_>>(),
            vec![
                ScalarLiteral::Float(3.402_823_466_385_288_6e38),
                ScalarLiteral::Float(-3.402_823_466_385_288_6e38),
            ]
        );
    }

    #[test]
    fn every_integer_type_accepts_its_own_range_and_refuses_one_past_each_end() {
        // Hand-written bounds, one row per ROS integer type.
        let rows: [(&str, &str, &str, &str, &str); 8] = [
            ("int8", "-128", "127", "-129", "128"),
            ("uint8", "0", "255", "-1", "256"),
            ("int16", "-32768", "32767", "-32769", "32768"),
            ("uint16", "0", "65535", "-1", "65536"),
            (
                "int32",
                "-2147483648",
                "2147483647",
                "-2147483649",
                "2147483648",
            ),
            ("uint32", "0", "4294967295", "-1", "4294967296"),
            (
                "int64",
                "-9223372036854775808",
                "9223372036854775807",
                "-9223372036854775809",
                "9223372036854775808",
            ),
            (
                "uint64",
                "0",
                "18446744073709551615",
                "-1",
                "18446744073709551616",
            ),
        ];
        for (ros, min, max, below, above) in rows {
            let ok = one(&format!("{ros} LO={min}\n{ros} HI={max}\n"));
            assert_eq!(
                ok.constants
                    .iter()
                    .map(|c| c.value.clone())
                    .collect::<Vec<_>>(),
                vec![
                    ScalarLiteral::Int(min.parse().unwrap()),
                    ScalarLiteral::Int(max.parse().unwrap()),
                ],
                "{ros}"
            );
            for bad in [below, above] {
                assert_eq!(
                    err(&format!("{ros} X={bad}\n")),
                    format!("line 1: value {bad} is out of range for {ros}")
                );
            }
        }
    }

    #[test]
    fn string_defaults_are_kept_quoted_or_not() {
        let s = one("string a \"hello world\"  # comment\n\
             string b 'single'\n\
             string<=5 c \"abcde\"\n\
             string d \"\"\n");
        let d: Vec<_> = s.fields.iter().map(|f| f.default_literal.clone()).collect();
        assert_eq!(
            d,
            vec![
                Some(DefaultLiteral::Scalar(ScalarLiteral::Str(
                    "hello world".into()
                ))),
                Some(DefaultLiteral::Scalar(ScalarLiteral::Str("single".into()))),
                Some(DefaultLiteral::Scalar(ScalarLiteral::Str("abcde".into()))),
                Some(DefaultLiteral::Scalar(ScalarLiteral::Str(String::new()))),
            ]
        );
    }

    #[test]
    fn array_defaults_are_kept_element_by_element() {
        let s = one("int32[] a [1, -2, 3]\n\
             float64[3] b [1, 2.5, 0]\n\
             bool[<=2] c [true, False]\n\
             string[] d [\"x, y\", 'z', plain]\n\
             uint8[] e []\n");
        use ScalarLiteral::*;
        let d: Vec<_> = s.fields.iter().map(|f| f.default_literal.clone()).collect();
        assert_eq!(
            d,
            vec![
                Some(DefaultLiteral::Array(vec![Int(1), Int(-2), Int(3)])),
                Some(DefaultLiteral::Array(vec![
                    Float(1.0),
                    Float(2.5),
                    Float(0.0)
                ])),
                Some(DefaultLiteral::Array(vec![Bool(true), Bool(false)])),
                Some(DefaultLiteral::Array(vec![
                    Str("x, y".into()),
                    Str("z".into()),
                    Str("plain".into())
                ])),
                Some(DefaultLiteral::Array(vec![])),
            ]
        );
    }

    #[test]
    fn defaults_do_not_change_the_schema_hash() {
        let with = one("string a \"x\"\nint32[] b [1]\nfloat64 c 2\n");
        let without = one("string a\nint32[] b\nfloat64 c\n");
        assert_eq!(with.schema_hash(), without.schema_hash());
    }

    #[test]
    fn a_zero_default_is_the_type_default_and_any_other_is_not() {
        use ScalarLiteral::*;
        let fixed = |t: FieldType| FieldType::FixedArray {
            element_type: Box::new(t),
            length: 2,
        };
        let dynamic = |t: FieldType| FieldType::DynamicArray {
            element_type: Box::new(t),
        };
        let zero = |d: DefaultLiteral, ft: &FieldType| d.is_zero_for(ft);
        assert!(zero(Str(String::new()).into(), &FieldType::String));
        assert!(!zero(Str("a".into()).into(), &FieldType::String));
        assert!(zero(Bool(false).into(), &FieldType::Bool));
        assert!(!zero(Int(1).into(), &FieldType::I32));
        // A dynamic array derives an empty `Vec`: only `[]` is zero.
        assert!(zero(
            DefaultLiteral::Array(vec![]),
            &dynamic(FieldType::I32)
        ));
        assert!(!zero(
            DefaultLiteral::Array(vec![Int(0)]),
            &dynamic(FieldType::I32)
        ));
        // A fixed array derives all-zero elements: any all-zero list is zero.
        assert!(zero(
            DefaultLiteral::Array(vec![Int(0), Int(0)]),
            &fixed(FieldType::I32)
        ));
        assert!(zero(
            DefaultLiteral::Array(vec![Float(0.0), Float(0.0)]),
            &fixed(FieldType::F64)
        ));
        assert!(zero(
            DefaultLiteral::Array(vec![Str(String::new()), Str(String::new())]),
            &fixed(FieldType::String)
        ));
        assert!(!zero(
            DefaultLiteral::Array(vec![Int(0), Int(1)]),
            &fixed(FieldType::I32)
        ));
        assert!(!zero(
            DefaultLiteral::Array(vec![Bool(false), Bool(true)]),
            &fixed(FieldType::Bool)
        ));
    }

    #[test]
    fn bad_defaults_are_errors_that_name_the_line() {
        assert_eq!(
            err("int32[3] a [1, 2]\n"),
            "line 1: array default has 2 elements, the type has 3"
        );
        assert_eq!(
            err("int32[<=1] a [1, 2]\n"),
            "line 1: array default has 2 elements, the bound is 1"
        );
        assert_eq!(
            err("string<=2 a \"abc\"\n"),
            "line 1: string default is longer than its bound of 2 bytes"
        );
        assert_eq!(
            err("string<=2[] a [\"abc\"]\n"),
            "line 1: string default is longer than its bound of 2 bytes"
        );
        // The bound counts UTF-8 bytes, as ROS 2 does: `é` is two bytes.
        assert_eq!(
            err("string<=1 a \"\u{e9}\"\n"),
            "line 1: string default is longer than its bound of 1 bytes"
        );
        assert_eq!(
            err("string<=1[] a [\"\u{e9}\"]\n"),
            "line 1: string default is longer than its bound of 1 bytes"
        );
        assert!(parse_rosmsg("string<=2 a \"\u{e9}\"\n", "Fits", None).is_ok());
        // A `wstring<=N` bound counts UTF-16 units: `é` is one unit, so it
        // fits a bound of 1; U+1F600 is a surrogate pair, two units. Only a
        // constant can hold wide text (a non-empty wstring default is
        // refused, see below), so the bound is pinned on constants.
        assert!(parse_rosmsg("wstring<=1 A=\"\u{e9}\"\n", "Fits", None).is_ok());
        assert!(parse_rosmsg("wstring<=2 A=\"\u{1f600}\"\n", "Fits", None).is_ok());
        assert_eq!(
            err("wstring<=1 A=\"\u{1f600}\"\n"),
            "line 1: string constant is longer than its bound of 1 UTF-16 units"
        );
        assert_eq!(
            err("wstring<=2 A=\"abc\"\n"),
            "line 1: string constant is longer than its bound of 2 UTF-16 units"
        );
        // The bound is checked before the wstring refusal, so a default that
        // is both too long and wide names the bound.
        assert_eq!(
            err("wstring<=1[] a [\"ab\"]\n"),
            "line 1: string default is longer than its bound of 1 UTF-16 units"
        );
        // A wstring is written as UTF-8, so a non-empty wstring default
        // would reach a ROS 2 peer in the wrong encoding: it is refused,
        // scalar or array, bounded or not, naming the declared type.
        assert_eq!(
            err("wstring a \"hi\"\n"),
            "line 1: a non-empty default on wstring is not supported: \
             a wstring is written as UTF-8, not UTF-16"
        );
        assert_eq!(
            err("wstring<=4 a \"\u{e9}\"\n"),
            "line 1: a non-empty default on wstring<=4 is not supported: \
             a wstring is written as UTF-8, not UTF-16"
        );
        assert_eq!(
            err("wstring[] a [\"\", \"x\"]\n"),
            "line 1: a non-empty default on wstring[] is not supported: \
             a wstring is written as UTF-8, not UTF-16"
        );
        assert_eq!(
            err("wstring[2] a [\"x\", \"\"]\n"),
            "line 1: a non-empty default on wstring[2] is not supported: \
             a wstring is written as UTF-8, not UTF-16"
        );
        // An empty wstring default is the zero value and is kept; a plain
        // `string` default and a wstring constant are untouched.
        let empty = parse_rosmsg("wstring a \"\"\nwstring[] b [\"\", \"\"]\n", "E", None)
            .expect("an empty wstring default parses");
        assert_eq!(
            empty.fields[0].default_literal,
            Some(DefaultLiteral::Scalar(ScalarLiteral::Str(String::new())))
        );
        assert_eq!(
            empty.fields[1].default_literal,
            Some(DefaultLiteral::Array(vec![
                ScalarLiteral::Str(String::new()),
                ScalarLiteral::Str(String::new()),
            ]))
        );
        let plain = parse_rosmsg("string a \"hi\"\n", "P", None).expect("a string default parses");
        assert_eq!(
            plain.fields[0].default_literal,
            Some(DefaultLiteral::Scalar(ScalarLiteral::Str("hi".to_string())))
        );
        let constant =
            parse_rosmsg("wstring A=\"hi\"\n", "C", None).expect("a wstring constant parses");
        assert_eq!(
            constant.constants[0].value,
            ScalarLiteral::Str("hi".to_string())
        );
        assert_eq!(
            err("int32[] a 1, 2\n"),
            "line 1: array default must be written [a, b, ...]: 1, 2"
        );
        assert_eq!(
            err("int32[] a [1, x]\n"),
            "line 1: invalid integer default: x"
        );
        assert_eq!(err("int32[] a [1,]\n"), "line 1: invalid integer default: ");
        assert_eq!(
            err("uint8 a 300\n"),
            "line 1: value 300 is out of range for uint8"
        );
        assert_eq!(
            err("string a 'open\n"),
            "line 1: unterminated string default: 'open"
        );
        assert_eq!(
            err("p/Other a 1\n"),
            "line 1: default values are not supported on message-type fields"
        );
        assert_eq!(
            err("p/Other[] a []\n"),
            "line 1: default values are not supported on message-type fields"
        );
    }

    #[test]
    fn two_parses_of_the_same_text_are_identical() {
        let text = "uint8 A=1\nstring S=\"s\"\nint32[] v [1, 2]\nstring n \"x\"\n";
        let a = one(text);
        let b = one(text);
        assert_eq!(format!("{a:?}"), format!("{b:?}"));
    }
}

/// The literal syntax rosidl accepts and this parser must match: integer
/// bases, comment cuts around bare and quoted strings, and malformed
/// quoting. Every oracle is the value rosidl's `parse_primitive_value_string`
/// gives for the same text, written out by hand.
#[cfg(test)]
mod literal_syntax_tests {
    use super::*;

    fn one(text: &str) -> MessageSchema {
        parse_rosmsg(text, "M", Some("p")).unwrap()
    }

    fn err(text: &str) -> String {
        parse_rosmsg(text, "M", Some("p")).unwrap_err().to_string()
    }

    fn constant_values(text: &str) -> Vec<ScalarLiteral> {
        one(text).constants.into_iter().map(|c| c.value).collect()
    }

    fn default_of(text: &str) -> Option<DefaultLiteral> {
        one(text).fields.remove(0).default_literal
    }

    #[test]
    fn integers_take_every_base_prefix_rosidl_accepts() {
        use ScalarLiteral::Int;
        assert_eq!(
            constant_values(
                "uint8 A=0x10\nuint8 B=0X1f\nint16 C=-0x8000\nuint8 D=0o17\nuint8 E=0b101\n\
                 uint8 F=0B1\nint32 G=+7\nint32 H=010\nint32 I=1_000\nuint16 J=0x_ff\n\
                 uint64 K=0xFFFFFFFFFFFFFFFF\n"
            ),
            vec![
                Int(16),
                Int(31),
                Int(-32768),
                Int(15),
                Int(5),
                Int(1),
                Int(7),
                Int(10),
                Int(1000),
                Int(255),
                Int(18_446_744_073_709_551_615),
            ]
        );
        // The same reader serves field defaults and array elements.
        assert_eq!(
            default_of("uint8 flags 0x80\n"),
            Some(DefaultLiteral::Scalar(Int(128)))
        );
        assert_eq!(
            default_of("int32[] v [0x10, -0b11, 4]\n"),
            Some(DefaultLiteral::Array(vec![Int(16), Int(-3), Int(4)]))
        );
    }

    #[test]
    fn a_base_prefixed_integer_keeps_the_range_check() {
        assert_eq!(
            err("uint8 X=0x100\n"),
            "line 1: value 0x100 is out of range for uint8"
        );
        assert_eq!(
            err("int8 X=-0x81\n"),
            "line 1: value -0x81 is out of range for int8"
        );
        assert_eq!(
            err("uint64 X=0x1_0000_0000_0000_0000\n"),
            "line 1: value 0x1_0000_0000_0000_0000 is out of range for uint64"
        );
        // Past even a 128-bit value: still out of range, not "invalid".
        let huge = format!("0x{}", "f".repeat(40));
        assert_eq!(
            err(&format!("uint64 X={huge}\n")),
            format!("line 1: value {huge} is out of range for uint64")
        );
    }

    #[test]
    fn malformed_integers_are_refused() {
        for bad in [
            "0x", "0xg", "0b2", "0o8", "1__0", "_1", "1_", "0x__1", "--1", "+", "1.0", "0x1_",
        ] {
            assert_eq!(
                err(&format!("int32 X={bad}\n")),
                format!("line 1: invalid integer constant: {bad}"),
                "{bad}"
            );
        }
    }

    #[test]
    fn an_apostrophe_inside_bare_text_is_not_a_quote() {
        use ScalarLiteral::Str;
        assert_eq!(
            constant_values("string LABEL=it's # note\n"),
            vec![Str("it's".into())]
        );
        assert_eq!(
            default_of("string s don't # note\n"),
            Some(DefaultLiteral::Scalar(Str("don't".into())))
        );
        assert_eq!(
            default_of("string[] v [it's, \"a, b\"] # note\n"),
            Some(DefaultLiteral::Array(vec![
                Str("it's".into()),
                Str("a, b".into())
            ]))
        );
    }

    #[test]
    fn the_first_hash_on_a_line_starts_a_comment_as_rosidl_reads_it() {
        use ScalarLiteral::{Int, Str};
        // rosidl cuts every line at its first `#`, so a bare string value
        // ends there, with or without a space before it.
        assert_eq!(
            constant_values("string TAG=a#b\nstring T2=a #b\n"),
            vec![Str("a".into()), Str("a".into())]
        );
        assert_eq!(
            default_of("string s a#b\n"),
            Some(DefaultLiteral::Scalar(Str("a".into())))
        );
        // A `#` as the value's first character leaves no value.
        assert_eq!(err("string T3=#x\n"), "line 1: constant T3 has no value");
        assert_eq!(default_of("string s #x\n"), None);
        // A `#` inside quotes is cut too, so the quote never closes.
        assert_eq!(
            err("string S=\"a#b\"\n"),
            "line 1: unterminated string constant: \"a"
        );
        assert_eq!(
            err("string s 'a#b'\n"),
            "line 1: unterminated string default: 'a"
        );
        // Right after a closing quote.
        assert_eq!(
            constant_values("string S=\"x\"#note\n"),
            vec![Str("x".into())]
        );
        // In a string array the cut can land inside an item, which then
        // leaves the list unclosed, or right after the closing `]`.
        assert_eq!(
            err("string[] v [a#b, c]\n"),
            "line 1: array default must be written [a, b, ...]: [a"
        );
        assert_eq!(
            default_of("string[] v [\"x\"]# note\n"),
            Some(DefaultLiteral::Array(vec![Str("x".into())]))
        );
        assert_eq!(
            default_of("string[2] v [a, c]#note\n"),
            Some(DefaultLiteral::Array(vec![
                Str("a".into()),
                Str("c".into())
            ]))
        );
        // A scalar string: `]` is plain text, the `#` after it still cuts.
        assert_eq!(constant_values("string S=a]#b\n"), vec![Str("a]".into())]);
        // A non-string value is cut the same way.
        assert_eq!(constant_values("uint8 X=1#c\n"), vec![Int(1)]);
    }

    #[test]
    fn a_quoted_value_must_close_on_its_last_character() {
        // The final quote is escaped, so the string never closes.
        assert_eq!(
            err("string LABEL=\"abc\\\"\n"),
            "line 1: unterminated string constant: \"abc\\\""
        );
        assert_eq!(
            err("string s 'abc\\'\n"),
            "line 1: unterminated string default: 'abc\\'"
        );
        // An unescaped quote inside the value.
        assert_eq!(
            err("string s \"a\"b\"\n"),
            "line 1: unescaped quote inside string default: \"a\"b\""
        );
        assert_eq!(
            err("string S=\"a\" b\n"),
            "line 1: unescaped quote inside string constant: \"a\" b"
        );
        assert_eq!(
            err("string[] v [\"a\"b\"]\n"),
            "line 1: unescaped quote inside string default: \"a\"b\""
        );
        // An escaped backslash before the closing quote still closes it.
        assert_eq!(
            constant_values("string S=\"a\\\\\"\n"),
            vec![ScalarLiteral::Str("a\\".into())]
        );
        // The other quote kind inside is plain text.
        assert_eq!(
            constant_values("string S=\"it's\"\n"),
            vec![ScalarLiteral::Str("it's".into())]
        );
    }

    #[test]
    fn negative_zero_survives_the_parse_and_is_not_the_zero_default() {
        let ft = FieldType::FixedArray {
            element_type: Box::new(FieldType::F64),
            length: 2,
        };
        let d = default_of("float64[2] v [-0.0, -0.0]\n").unwrap();
        let DefaultLiteral::Array(items) = &d else {
            panic!("expected an array default, got {d:?}")
        };
        for item in items {
            let ScalarLiteral::Float(f) = item else {
                panic!("expected a float, got {item:?}")
            };
            assert_eq!(f.to_bits(), (-0.0f64).to_bits());
        }
        assert!(!d.is_zero_for(&ft));
        assert!(!ScalarLiteral::Float(-0.0).is_zero());
        assert!(ScalarLiteral::Float(0.0).is_zero());
        assert!(DefaultLiteral::Array(vec![ScalarLiteral::Float(0.0); 2]).is_zero_for(&ft));
    }
}
