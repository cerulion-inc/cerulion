// SPDX-License-Identifier: AGPL-3.0-only
//! YAML -> frame.

use serde_yaml::{Mapping, Value};

use super::encode::format_f32;
use super::parse::{parse, wide_integer};
use super::scan::find_anchor_or_alias;
use super::{
    push_field, push_index, shown, type_name, MAX_YAML_ARRAY_DEPTH, MAX_YAML_DEPTH,
    MAX_YAML_INPUT_BYTES, YAML_TRUNCATED_TAG,
};
use crate::codegen::element_codec::{CanonicalBodyBuilder, MAX_BODY_BYTES};
use crate::codegen::layout::WireLayout;
use crate::codegen::{FieldBounds, FieldDef, FieldType, FrameWalker, MessageSchema};
use crate::dynamic::DynamicError;
use crate::wire::WireHeader;

/// Parse `yaml` as one message of the schema `schema` (a qualified name
/// the walker knows) and build its wire frame, header included.
///
/// The header carries the schema's hash, `sequence` 0 (the transport stamps
/// it at publish) and `timestamp_ns`. The frame is byte-identical to what
/// every canonical producer writes for the same values: padding is zero and
/// variable payloads follow the shared placement rule.
///
/// Empty input (or a lone `null`) is an empty mapping, so a message with no
/// fields needs no text. One final `---` line, the separator `topic echo`
/// prints after each frame, is ignored.
///
/// # Errors
///
/// - [`DynamicError::UnknownSchema`]: `schema` is not in the walker.
/// - [`DynamicError::YamlInputTooLarge`]: `yaml` is longer than
///   [`super::MAX_YAML_INPUT_BYTES`] (checked before it is parsed).
/// - [`DynamicError::YamlInputNotUtf8`], [`DynamicError::YamlSyntax`]
///   (includes duplicate keys, more than one document, nesting past the
///   parser's limit), [`DynamicError::YamlAnchorOrAlias`].
/// - Value errors, each naming the full field path:
///   [`DynamicError::YamlWrongType`], [`DynamicError::YamlOutOfRange`],
///   [`DynamicError::YamlUnknownField`], [`DynamicError::YamlMissingField`],
///   [`DynamicError::YamlWrongLength`], [`DynamicError::YamlTooManyElements`],
///   [`DynamicError::YamlStringTooLong`],
///   [`DynamicError::YamlWideStringTooLong`],
///   [`DynamicError::YamlNulInFixedString`], [`DynamicError::YamlTruncated`],
///   [`DynamicError::YamlTag`].
/// - Schema shape errors: [`DynamicError::YamlUnsupportedField`],
///   [`DynamicError::YamlUnresolvedNested`], [`DynamicError::YamlTooDeep`].
/// - [`DynamicError::YamlMessageTooLarge`]: the frame would pass the 1 GiB
///   cap.
pub fn yaml_to_message(
    walker: &FrameWalker,
    schema: &str,
    yaml: &[u8],
    timestamp_ns: u64,
) -> Result<Vec<u8>, DynamicError> {
    yaml_to_message_capped(walker, schema, yaml, timestamp_ns, MAX_YAML_INPUT_BYTES)
}

/// [`yaml_to_message`] with the input cap as a parameter, so tests can
/// reach it with small inputs.
pub(super) fn yaml_to_message_capped(
    walker: &FrameWalker,
    schema: &str,
    yaml: &[u8],
    timestamp_ns: u64,
    max_bytes: usize,
) -> Result<Vec<u8>, DynamicError> {
    // One slot gives both halves (see `FrameWalker::schema_for_name`).
    let (layout, resolved) = walker
        .layout_for_name(schema)
        .zip(walker.schema_for_name(schema))
        .ok_or_else(|| DynamicError::UnknownSchema(schema.to_string()))?;
    // The value tree costs a bounded multiple of the text, so the text is
    // bounded before it is parsed.
    if yaml.len() > max_bytes {
        return Err(DynamicError::YamlInputTooLarge {
            len: yaml.len(),
            max_bytes,
        });
    }
    let text = std::str::from_utf8(yaml).map_err(|e| DynamicError::YamlInputNotUtf8 {
        valid_up_to: e.valid_up_to(),
    })?;
    let text = without_trailing_separator(text);
    if let Some((line, column)) = find_anchor_or_alias(text) {
        return Err(DynamicError::YamlAnchorOrAlias { line, column });
    }
    let value = parse(text).map_err(syntax_error)?;
    let value = match value {
        Value::Null => Value::Mapping(Mapping::new()),
        other => other,
    };
    let mut builder = Builder {
        walker,
        path: String::new(),
        array_depth: 0,
        held: 0,
    };
    let payload = builder.body((layout, resolved), &value, 0)?;
    let total = WireHeader::SIZE + payload.len();
    let total_size =
        u32::try_from(total).map_err(|_| DynamicError::YamlMessageTooLarge { path: shown("") })?;
    let mut frame = vec![0u8; WireHeader::SIZE];
    WireHeader {
        schema_hash: layout.schema_hash,
        total_size,
        offset_table_offset: (WireHeader::SIZE + layout.fixed_size) as u32,
        offset_table_count: layout.variable_fields.len() as u32,
        sequence: 0,
        timestamp_ns,
    }
    .write_to_buf(&mut frame);
    frame.extend_from_slice(&payload);
    Ok(frame)
}

/// `text` without one final `---` line: `topic echo` ends every frame with
/// one, and a frame copied with it is still one message, not that message
/// and an empty second document.
fn without_trailing_separator(text: &str) -> &str {
    let body = text.trim_end_matches(['\n', '\r', ' ', '\t']);
    let (head, last) = match body.rfind('\n') {
        Some(at) => (&text[..at + 1], &body[at + 1..]),
        None => ("", body),
    };
    if last.trim_end() == "---" {
        head
    } else {
        text
    }
}

/// The parser's message, with its location appended when the message
/// lacks one (a duplicate key names no line on its own).
fn syntax_error(e: serde_yaml::Error) -> DynamicError {
    let mut text = e.to_string();
    if let Some(at) = e.location() {
        if !text.contains(" at line ") {
            text.push_str(&format!(" at line {} column {}", at.line(), at.column()));
        }
    }
    DynamicError::YamlSyntax(text)
}

/// The YAML kind of a value, as error text names it.
fn kind(v: &Value) -> &'static str {
    if wide_integer(v).is_some() {
        return "integer";
    }
    match v {
        Value::Null => "null",
        Value::Bool(_) => "boolean",
        Value::Number(n) if n.is_f64() => "float",
        Value::Number(_) => "integer",
        Value::String(_) => "string",
        Value::Sequence(_) => "sequence",
        Value::Mapping(_) => "mapping",
        Value::Tagged(_) => "tagged value",
    }
}

/// A message type the builder writes: its layout and its resolved schema,
/// both from one walker slot.
type Target<'w> = (&'w WireLayout, &'w MessageSchema);

struct Builder<'w> {
    walker: &'w FrameWalker,
    path: String,
    /// How many elements of variable-message arrays enclose the value
    /// being built.
    array_depth: usize,
    /// Bytes the unfinished bodies and arrays on the build stack already
    /// hold. Every one of them ends up in the frame, so the sum is kept
    /// under [`MAX_BODY_BYTES`] as it grows: a few bytes of YAML (unset
    /// `string_fixed` fields, say) cannot make the reader hold more than
    /// one frame's worth before it refuses.
    held: usize,
}

impl<'w> Builder<'w> {
    fn here(&self) -> String {
        shown(&self.path)
    }

    /// Refuse when `n` more bytes would take the frame past the cap.
    fn room(&self, n: usize) -> Result<(), DynamicError> {
        if self.held.checked_add(n).is_none_or(|t| t > MAX_BODY_BYTES) {
            return Err(DynamicError::YamlMessageTooLarge { path: self.here() });
        }
        Ok(())
    }

    /// Count `n` more bytes as held, refusing past the cap. A function that
    /// returns a buffer gives its holds back first (`held` is reset to its
    /// value on entry), and the caller holds the returned length.
    fn hold(&mut self, n: usize) -> Result<(), DynamicError> {
        self.room(n)?;
        self.held += n;
        Ok(())
    }

    fn wrong(&self, expected: &FieldType, got: &Value) -> DynamicError {
        DynamicError::YamlWrongType {
            path: self.here(),
            expected: type_name(expected),
            got: kind(got).to_string(),
        }
    }

    /// Refuse a tagged value: `!truncated` with its own message, any other
    /// tag as unsupported. Every value passes through here first.
    fn untagged<'v>(&self, v: &'v Value) -> Result<&'v Value, DynamicError> {
        match v {
            Value::Tagged(t) if t.tag == YAML_TRUNCATED_TAG => Err(DynamicError::YamlTruncated {
                path: self.here(),
                omitted: match &t.value {
                    Value::Number(n) => n.to_string(),
                    other => {
                        wide_integer(other).map_or_else(|| kind(other).to_string(), str::to_string)
                    }
                },
            }),
            // An integer past 64 bits is a plain integer to every field.
            Value::Tagged(_) if wide_integer(v).is_some() => Ok(v),
            Value::Tagged(t) => Err(DynamicError::YamlTag {
                path: self.here(),
                tag: t.tag.to_string(),
            }),
            other => Ok(other),
        }
    }

    /// At [`MAX_YAML_ARRAY_DEPTH`] the walker decodes only an empty blob
    /// (zero elements): `Some(empty)` for an empty array, which is then
    /// written as that blob, and a refusal for any other.
    fn past_array_depth(&self, items: &[Value]) -> Result<Option<Vec<u8>>, DynamicError> {
        if self.array_depth < MAX_YAML_ARRAY_DEPTH {
            return Ok(None);
        }
        if items.is_empty() {
            return Ok(Some(Vec::new()));
        }
        Err(DynamicError::YamlArraysTooDeep {
            path: self.here(),
            max: MAX_YAML_ARRAY_DEPTH,
        })
    }

    fn too_deep(&self, depth: usize) -> Result<(), DynamicError> {
        if depth > MAX_YAML_DEPTH {
            return Err(DynamicError::YamlTooDeep {
                path: self.here(),
                max: MAX_YAML_DEPTH,
            });
        }
        Ok(())
    }

    /// The layout and schema a nested reference, written inside a schema
    /// of package `parent`, resolves to.
    fn nested(
        &self,
        schema_name: &str,
        package: &Option<String>,
        parent: Option<&str>,
    ) -> Result<Target<'w>, DynamicError> {
        self.walker
            .nested_target(schema_name, package, parent)
            .ok_or_else(|| DynamicError::YamlUnresolvedNested {
                path: self.here(),
                target: match package {
                    Some(p) => format!("{p}/{schema_name}"),
                    None => schema_name.to_string(),
                },
            })
    }

    /// The values of `schema`'s fields from the mapping `v`, in declaration
    /// order. Refuses a non-mapping, a key that is not a field (first in
    /// document order) and a missing field (first in declaration order).
    fn fields<'v>(
        &self,
        schema: &MessageSchema,
        v: &'v Value,
    ) -> Result<Vec<&'v Value>, DynamicError> {
        let qname = schema.qualified_name();
        let map = match self.untagged(v)? {
            Value::Mapping(m) => m,
            other => {
                return Err(DynamicError::YamlWrongType {
                    path: self.here(),
                    expected: qname,
                    got: kind(other).to_string(),
                })
            }
        };
        for k in map.keys() {
            let Value::String(name) = k else {
                return Err(DynamicError::YamlWrongType {
                    path: self.here(),
                    expected: "a field name".to_string(),
                    got: kind(k).to_string(),
                });
            };
            if !schema.fields.iter().any(|f| &f.name == name) {
                let mut path = self.path.clone();
                push_field(&mut path, name);
                return Err(DynamicError::YamlUnknownField {
                    path,
                    schema: qname,
                    known: schema
                        .fields
                        .iter()
                        .map(|f| f.name.as_str())
                        .collect::<Vec<_>>()
                        .join(", "),
                });
            }
        }
        schema
            .fields
            .iter()
            .map(|f| {
                map.get(f.name.as_str()).ok_or_else(|| {
                    let mut path = self.path.clone();
                    push_field(&mut path, &f.name);
                    DynamicError::YamlMissingField {
                        path,
                        schema: qname.clone(),
                    }
                })
            })
            .collect()
    }

    /// A headerless body `[fixed][offset table][variable]` for `target`.
    fn body(
        &mut self,
        (layout, schema): Target<'w>,
        v: &Value,
        depth: usize,
    ) -> Result<Vec<u8>, DynamicError> {
        self.too_deep(depth)?;
        let parent = schema.package.as_deref();
        let values = self.fields(schema, v)?;
        // The builder allocates the whole fixed section up front, so a
        // fixed section that does not fit what is left of the cap is
        // refused before that allocation, not after it: a few bytes of YAML
        // must not make the reader ask for gigabytes.
        let base = self.held;
        let floor = layout
            .fixed_size
            .checked_add(layout.offset_table_bytes())
            .ok_or_else(|| DynamicError::YamlMessageTooLarge { path: self.here() })?;
        self.hold(floor)?;
        let mut builder = CanonicalBodyBuilder::new(layout);
        let mut fixed_idx = 0;
        for (field, value) in schema.fields.iter().zip(values) {
            let mark = self.path.len();
            push_field(&mut self.path, &field.name);
            self.within_bounds(field, value)?;
            if field.field_type.is_variable() {
                let bytes = self.variable(&field.field_type, value, parent, depth)?;
                self.hold(bytes.len())?;
                builder
                    .push_variable(bytes)
                    .map_err(|_| DynamicError::YamlMessageTooLarge { path: self.here() })?;
            } else {
                let fl = &layout.fixed_fields[fixed_idx];
                fixed_idx += 1;
                let range = fl.offset..fl.offset + fl.size;
                let dst = &mut builder.fixed_mut()[range];
                self.fixed(&field.field_type, value, dst, parent, depth)?;
            }
            self.path.truncate(mark);
        }
        self.held = base;
        builder
            .finish()
            .map_err(|_| DynamicError::YamlMessageTooLarge { path: self.here() })
    }

    /// Refuse a value past the field's ROS 2 upper bounds (`T[<=N]`,
    /// `string<=N`, both on `string<=M[<=N]`). A value of the wrong kind
    /// passes here and is refused by the type check that follows.
    fn within_bounds(&self, field: &FieldDef, v: &Value) -> Result<(), DynamicError> {
        let FieldBounds {
            max_elements,
            max_string_bytes,
            max_wstring_units,
        } = field.bounds;
        let too_long = |path: &str, s: &str| {
            if let Some(bound) = max_string_bytes.filter(|&b| s.len() > b) {
                return Err(DynamicError::YamlStringTooLong {
                    path: shown(path),
                    bound,
                    got: s.len(),
                });
            }
            if let Some(bound) = max_wstring_units {
                // No arithmetic on the bound: any `usize` the parser takes,
                // `usize::MAX` included, is a valid one.
                let got = s.encode_utf16().count();
                if got > bound {
                    return Err(DynamicError::YamlWideStringTooLong {
                        path: shown(path),
                        bound,
                        got,
                    });
                }
            }
            Ok(())
        };
        let strings_bounded = max_string_bytes.is_some() || max_wstring_units.is_some();
        let is_array = matches!(
            field.field_type,
            FieldType::DynamicArray { .. } | FieldType::FixedArray { .. }
        );
        match v {
            Value::String(s) if !is_array => too_long(&self.path, s),
            Value::Sequence(_) if is_array && (max_elements.is_some() || strings_bounded) => {
                // A `!truncated` element is refused first, with its own
                // message, as for any array.
                let items = self.sequence(&field.field_type, v, None)?;
                if let Some(bound) = max_elements {
                    if items.len() > bound {
                        return Err(DynamicError::YamlTooManyElements {
                            path: self.here(),
                            bound,
                            got: items.len(),
                        });
                    }
                }
                if strings_bounded {
                    for (i, item) in items.iter().enumerate() {
                        if let Value::String(s) = item {
                            let mut path = self.path.clone();
                            push_index(&mut path, i);
                            too_long(&path, s)?;
                        }
                    }
                }
                Ok(())
            }
            _ => Ok(()),
        }
    }

    /// Write one fixed-section value into `dst` (exactly its bytes).
    fn fixed(
        &mut self,
        ft: &FieldType,
        v: &Value,
        dst: &mut [u8],
        parent: Option<&str>,
        depth: usize,
    ) -> Result<(), DynamicError> {
        let v = self.untagged(v)?;
        match ft {
            FieldType::StringFixed(n) => {
                let Value::String(s) = v else {
                    return Err(self.wrong(ft, v));
                };
                if s.len() > *n {
                    return Err(DynamicError::YamlStringTooLong {
                        path: self.here(),
                        bound: *n,
                        got: s.len(),
                    });
                }
                if s.contains('\0') {
                    return Err(DynamicError::YamlNulInFixedString { path: self.here() });
                }
                dst[..s.len()].copy_from_slice(s.as_bytes());
                Ok(())
            }
            FieldType::FixedArray {
                element_type,
                length,
            } => {
                // An array of arrays has no YAML form (the encoder refuses it
                // too), so neither direction accepts what the other cannot.
                if matches!(element_type.as_ref(), FieldType::FixedArray { .. }) {
                    return Err(DynamicError::YamlUnsupportedField {
                        path: self.here(),
                        field_type: type_name(ft),
                    });
                }
                let items = self.sequence(ft, v, Some(*length))?;
                let stride = element_type.fixed_size().unwrap_or(0);
                for (i, item) in items.iter().enumerate() {
                    let mark = self.path.len();
                    push_index(&mut self.path, i);
                    let start = i * stride;
                    // The sequence is a level (see `MAX_YAML_DEPTH`).
                    self.fixed(
                        element_type,
                        item,
                        &mut dst[start..start + stride],
                        parent,
                        depth + 1,
                    )?;
                    self.path.truncate(mark);
                }
                Ok(())
            }
            FieldType::Nested {
                schema_name,
                package,
                ..
            } => {
                self.too_deep(depth + 1)?;
                let (target, schema) = self.nested(schema_name, package, parent)?;
                let values = self.fields(schema, v)?;
                for (field, value) in schema.fields.iter().zip(values) {
                    let fl = target
                        .fixed_fields
                        .iter()
                        .find(|f| f.name == field.name)
                        .ok_or_else(|| DynamicError::YamlUnsupportedField {
                            path: self.here(),
                            field_type: type_name(ft),
                        })?;
                    let mark = self.path.len();
                    push_field(&mut self.path, &field.name);
                    let range = fl.offset..fl.offset + fl.size;
                    self.fixed(
                        &field.field_type,
                        value,
                        &mut dst[range],
                        schema.package.as_deref(),
                        depth + 1,
                    )?;
                    self.path.truncate(mark);
                }
                Ok(())
            }
            _ => {
                let bytes = self.scalar(ft, v)?;
                dst.copy_from_slice(&bytes);
                Ok(())
            }
        }
    }

    /// The little-endian bytes of one primitive value.
    fn scalar(&self, ft: &FieldType, v: &Value) -> Result<Vec<u8>, DynamicError> {
        let v = self.untagged(v)?;
        macro_rules! int {
            ($t:ty) => {{
                // An integer past 64 bits fits no field type.
                let (parsed, written) = match (wide_integer(v), v) {
                    (Some(digits), _) => (None, digits.to_string()),
                    (None, Value::Number(n)) if !n.is_f64() => (
                        match n.as_i64() {
                            Some(i) => <$t>::try_from(i).ok(),
                            None => n.as_u64().and_then(|u| <$t>::try_from(u).ok()),
                        },
                        n.to_string(),
                    ),
                    _ => return Err(self.wrong(ft, v)),
                };
                match parsed {
                    Some(x) => x.to_le_bytes().to_vec(),
                    None => {
                        return Err(DynamicError::YamlOutOfRange {
                            path: self.here(),
                            value: written,
                            expected: format!("{} ({} to {})", type_name(ft), <$t>::MIN, <$t>::MAX),
                        })
                    }
                }
            }};
        }
        Ok(match ft {
            FieldType::Bool => match v {
                Value::Bool(b) => vec![u8::from(*b)],
                other => return Err(self.wrong(ft, other)),
            },
            FieldType::I8 => int!(i8),
            FieldType::U8 => int!(u8),
            FieldType::I16 => int!(i16),
            FieldType::U16 => int!(u16),
            FieldType::I32 => int!(i32),
            FieldType::U32 => int!(u32),
            FieldType::I64 => int!(i64),
            FieldType::U64 => int!(u64),
            FieldType::F64 => self.float(ft, v)?.to_le_bytes().to_vec(),
            FieldType::F32 => {
                let x = self.float(ft, v)?;
                let f = f32_from_f64(x);
                if x.is_finite() && !f.is_finite() {
                    return Err(DynamicError::YamlOutOfRange {
                        path: self.here(),
                        value: super::encode::format_f64(x),
                        expected: "float32 (finite values up to 3.4028235e38 in magnitude)"
                            .to_string(),
                    });
                }
                f.to_le_bytes().to_vec()
            }
            other => {
                return Err(DynamicError::YamlUnsupportedField {
                    path: self.here(),
                    field_type: type_name(other),
                })
            }
        })
    }

    fn float(&self, ft: &FieldType, v: &Value) -> Result<f64, DynamicError> {
        // An integer is accepted for a float field, however wide.
        if let Some(digits) = wide_integer(v) {
            return digits.parse::<f64>().map_err(|_| self.wrong(ft, v));
        }
        match v {
            Value::Number(n) => n.as_f64().ok_or_else(|| self.wrong(ft, v)),
            other => Err(self.wrong(ft, other)),
        }
    }

    /// The elements of a YAML sequence, with the fixed-array length check.
    fn sequence<'v>(
        &self,
        ft: &FieldType,
        v: &'v Value,
        exact: Option<usize>,
    ) -> Result<&'v [Value], DynamicError> {
        let Value::Sequence(items) = self.untagged(v)? else {
            return Err(self.wrong(ft, v));
        };
        // A `!truncated` element anywhere is refused with its own message
        // before the length check, so a cut echo says what happened.
        for (i, item) in items.iter().enumerate() {
            if matches!(item, Value::Tagged(_)) {
                let mut path = self.path.clone();
                push_index(&mut path, i);
                let at = Builder {
                    walker: self.walker,
                    path,
                    array_depth: self.array_depth,
                    held: self.held,
                };
                at.untagged(item)?;
            }
        }
        if let Some(n) = exact {
            if items.len() != n {
                return Err(DynamicError::YamlWrongLength {
                    path: self.here(),
                    expected: n,
                    got: items.len(),
                });
            }
        }
        Ok(items)
    }

    /// The variable-payload bytes of one variable field.
    fn variable(
        &mut self,
        ft: &FieldType,
        v: &Value,
        parent: Option<&str>,
        depth: usize,
    ) -> Result<Vec<u8>, DynamicError> {
        let v = self.untagged(v)?;
        match ft {
            FieldType::String => match v {
                Value::String(s) => {
                    self.room(s.len())?;
                    Ok(s.as_bytes().to_vec())
                }
                other => Err(self.wrong(ft, other)),
            },
            FieldType::Bytes => self.byte_list(ft, &FieldType::U8, v, None),
            FieldType::Nested {
                schema_name,
                package,
                ..
            } => {
                let target = self.nested(schema_name, package, parent)?;
                self.body(target, v, depth + 1)
            }
            FieldType::DynamicArray { element_type } => {
                self.array(ft, element_type, v, None, parent, depth)
            }
            FieldType::FixedArray {
                element_type,
                length,
            } => self.array(ft, element_type, v, Some(*length), parent, depth),
            other => Err(DynamicError::YamlUnsupportedField {
                path: self.here(),
                field_type: type_name(other),
            }),
        }
    }

    /// `uint8[]` / `int8[]` / `bool[]` / `bytes`: one byte per element.
    fn byte_list(
        &mut self,
        ft: &FieldType,
        elem: &FieldType,
        v: &Value,
        exact: Option<usize>,
    ) -> Result<Vec<u8>, DynamicError> {
        let items = self.sequence(ft, v, exact)?;
        // The payload size is known before any of it is built: refuse one
        // past the cap here, not after building it.
        let size = elem.fixed_size().unwrap_or(1);
        let n = items
            .len()
            .checked_mul(size)
            .ok_or_else(|| DynamicError::YamlMessageTooLarge { path: self.here() })?;
        self.room(n)?;
        let mut out = Vec::with_capacity(items.len() * size);
        for (i, item) in items.iter().enumerate() {
            let mark = self.path.len();
            push_index(&mut self.path, i);
            out.extend(self.scalar(elem, item)?);
            self.path.truncate(mark);
        }
        Ok(out)
    }

    fn array(
        &mut self,
        ft: &FieldType,
        elem: &FieldType,
        v: &Value,
        exact: Option<usize>,
        parent: Option<&str>,
        depth: usize,
    ) -> Result<Vec<u8>, DynamicError> {
        match elem {
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
            | FieldType::F64 => self.byte_list(ft, elem, v, exact),
            FieldType::String => {
                let items = self.sequence(ft, v, exact)?;
                if let Some(blob) = self.past_array_depth(items)? {
                    return Ok(blob);
                }
                let base = self.held;
                let mut out = count_prefix(items.len(), &self.path)?;
                self.hold(out.len())?;
                for (i, item) in items.iter().enumerate() {
                    let mark = self.path.len();
                    push_index(&mut self.path, i);
                    let bytes = self.variable(elem, item, parent, depth)?;
                    self.hold(4 + bytes.len())?;
                    out.extend(len_prefix(bytes.len(), &self.path)?);
                    out.extend(bytes);
                    self.path.truncate(mark);
                }
                self.held = base;
                Ok(out)
            }
            FieldType::Nested {
                schema_name,
                package,
                ..
            } => {
                let target = self.nested(schema_name, package, parent)?;
                let (layout, _) = target;
                let items = self.sequence(ft, v, exact)?;
                // Fixed elements are back to back with no count; variable
                // ones carry a count and a length per element.
                let counted = !layout.is_fixed();
                if let Some(blob) = self.past_array_depth(items)? {
                    return Ok(blob);
                }
                // A fieldless fixed element is zero bytes with no count, so
                // the wire cannot say how many there were: only the empty
                // array has a frame, and any other is refused rather than
                // read back as `[]`.
                if !counted && layout.fixed_size == 0 && !items.is_empty() {
                    return Err(DynamicError::YamlUnsupportedField {
                        path: self.here(),
                        field_type: type_name(ft),
                    });
                }
                let base = self.held;
                let mut out = if counted {
                    count_prefix(items.len(), &self.path)?
                } else {
                    Vec::new()
                };
                self.hold(out.len())?;
                for (i, item) in items.iter().enumerate() {
                    let mark = self.path.len();
                    push_index(&mut self.path, i);
                    self.array_depth += usize::from(counted);
                    // The sequence and the element mapping are a level each.
                    let body = self.body(target, item, depth + 2);
                    self.array_depth -= usize::from(counted);
                    let body = body?;
                    self.hold(if counted { 4 } else { 0 } + body.len())?;
                    if counted {
                        out.extend(len_prefix(body.len(), &self.path)?);
                    }
                    out.extend(body);
                    self.path.truncate(mark);
                }
                self.held = base;
                Ok(out)
            }
            _ => Err(DynamicError::YamlUnsupportedField {
                path: self.here(),
                field_type: type_name(ft),
            }),
        }
    }
}

fn count_prefix(n: usize, path: &str) -> Result<Vec<u8>, DynamicError> {
    len_prefix(n, path)
}

pub(super) fn len_prefix(n: usize, path: &str) -> Result<Vec<u8>, DynamicError> {
    u32::try_from(n)
        .map(|n| n.to_le_bytes().to_vec())
        .map_err(|_| DynamicError::YamlMessageTooLarge { path: shown(path) })
}

/// The `f32` a YAML float names. The parser hands over the nearest `f64`;
/// a plain `as f32` of that rounds twice and misses for exactly two `f32`
/// values, bit pattern `363742205` and its negative (an exhaustive check
/// over every finite `f32`), each landing one step away from zero. The
/// shortest form the encoder writes for an `f32` reads back to exactly one
/// `f64`, so when the cast does not read back, the neighbour toward zero
/// that does is the one that was written. A hand-typed decimal keeps the
/// cast.
pub(super) fn f32_from_f64(x: f64) -> f32 {
    let near = x as f32;
    // Zero has no neighbour toward zero (and an f64 too small for an f32
    // casts to zero); a non-finite cast is already exact.
    if !near.is_finite() || near == 0.0 {
        return near;
    }
    let reads_back = |c: f32| format_f32(c).parse::<f64>().ok() == Some(x);
    let toward_zero = f32::from_bits(near.to_bits() - 1);
    if !reads_back(near) && reads_back(toward_zero) {
        toward_zero
    } else {
        near
    }
}
