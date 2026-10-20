// SPDX-License-Identifier: AGPL-3.0-only
//! Frame -> YAML.

use std::fmt::Write as _;

use super::{
    push_field, push_index, shown, type_name, MAX_YAML_BYTES, MAX_YAML_DEPTH, YAML_TRUNCATED_TAG,
};
use crate::codegen::{FieldType, FrameValue, FrameValueKind, FrameWalker, MessageSchema, PrimType};
use crate::dynamic::DynamicError;

/// Display options for [`message_to_yaml_with`].
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct YamlEncodeOptions {
    /// Show at most this many elements of each array; the rest are counted
    /// in a closing `!truncated <n>` element. `None` (the default) shows
    /// every element, which is the only form [`super::yaml_to_message`]
    /// accepts back.
    pub truncate_arrays: Option<usize>,
    /// Stop after this many lines: the output is the first `n` lines and
    /// then a column-0 [`YAML_TRUNCATED_TAG`] line, which no YAML reader
    /// accepts, so cut output never reads back as a shorter message. The
    /// encoder stops writing at the cut, so the work is bounded by `n`,
    /// not by the size of the message. `None` (the default) writes every
    /// line.
    pub max_lines: Option<usize>,
}

/// Render the wire `frame` (header included) as YAML, every element shown.
///
/// The frame's `schema_hash` picks the schema from `walker`; the output is
/// one block mapping in declaration order, ending in a newline (see the
/// module docs for the form). The same frame always gives the same bytes.
///
/// # Errors
///
/// - [`DynamicError::Walk`]: the walker refused the frame (unknown hash,
///   truncated or malformed bytes).
/// - [`DynamicError::YamlUnencodable`]: a value has no YAML form (a string
///   field that is not UTF-8, an array that is not in the canonical element
///   framing, a `bool[]` byte that is neither 0 nor 1).
/// - [`DynamicError::YamlUnsupportedField`]: the schema has a field type the
///   YAML form does not cover (an array of arrays, an array of
///   `string_fixed[n]` outside a fixed array).
/// - [`DynamicError::YamlTooDeep`]: the schema nests deeper than
///   [`super::MAX_YAML_DEPTH`].
/// - [`DynamicError::YamlOutputTooLarge`]: the YAML would be longer than
///   [`super::MAX_YAML_BYTES`].
pub fn message_to_yaml(walker: &FrameWalker, frame: &[u8]) -> Result<String, DynamicError> {
    message_to_yaml_with(walker, frame, YamlEncodeOptions::default())
}

/// [`message_to_yaml`] with display options (array truncation for a
/// terminal). Truncated output is for reading only: decoding it back fails
/// loudly at the first `!truncated` element.
///
/// # Errors
///
/// As [`message_to_yaml`].
pub fn message_to_yaml_with(
    walker: &FrameWalker,
    frame: &[u8],
    options: YamlEncodeOptions,
) -> Result<String, DynamicError> {
    message_to_yaml_capped(walker, frame, options, MAX_YAML_BYTES)
}

/// [`message_to_yaml_with`] with the output cap as a parameter, so tests
/// can reach it with small messages.
pub(super) fn message_to_yaml_capped(
    walker: &FrameWalker,
    frame: &[u8],
    options: YamlEncodeOptions,
    max_bytes: usize,
) -> Result<String, DynamicError> {
    let value = walker.walk_by_hash(frame)?;
    let schema = schema_of(walker, &value.schema_name, "")?;
    let mut emitter = Emitter {
        walker,
        options,
        max_bytes,
        out: String::new(),
        lines: 0,
        cut: false,
        path: String::new(),
    };
    let written = if schema.fields.is_empty() {
        emitter.put("{}\n")
    } else {
        emitter.mapping(schema, Some(&value), 0, 0, false)
    };
    match written {
        Ok(()) => Ok(emitter.out),
        // The line budget ran out: the output so far ends at a whole line.
        Err(_) if emitter.cut => {
            emitter.fits(YAML_TRUNCATED_TAG.len() + 1)?;
            emitter.out.push_str(YAML_TRUNCATED_TAG);
            emitter.out.push('\n');
            Ok(emitter.out)
        }
        Err(e) => Err(e),
    }
}

fn schema_of<'w>(
    walker: &'w FrameWalker,
    qname: &str,
    path: &str,
) -> Result<&'w MessageSchema, DynamicError> {
    walker
        .schema_for_name(qname)
        .ok_or_else(|| DynamicError::YamlUnresolvedNested {
            path: shown(path),
            target: qname.to_string(),
        })
}

struct Emitter<'w> {
    walker: &'w FrameWalker,
    options: YamlEncodeOptions,
    /// The longest output allowed, in bytes.
    max_bytes: usize,
    out: String,
    /// The number of whole lines in `out`.
    lines: usize,
    /// Set when [`YamlEncodeOptions::max_lines`] stopped the output; the
    /// error that unwinds the emitter is then the stop, not a failure.
    cut: bool,
    /// The field path of the value being emitted (for error text).
    path: String,
}

impl<'w> Emitter<'w> {
    fn unencodable(&self, detail: impl Into<String>) -> DynamicError {
        DynamicError::YamlUnencodable {
            path: shown(&self.path),
            detail: detail.into(),
        }
    }

    fn too_large(&self) -> DynamicError {
        DynamicError::YamlOutputTooLarge {
            path: shown(&self.path),
            max_bytes: self.max_bytes,
        }
    }

    /// Refuse once `extra` more bytes would take the output past the cap.
    fn fits(&self, extra: usize) -> Result<(), DynamicError> {
        if self.out.len().saturating_add(extra) > self.max_bytes {
            return Err(self.too_large());
        }
        Ok(())
    }

    /// The ONE way text reaches the output: every write is counted against
    /// the byte cap and the line budget first. `s` holds at most one
    /// newline, at its end.
    fn put(&mut self, s: &str) -> Result<(), DynamicError> {
        // The budget is spent once a write would start or end one more
        // line, so nothing past it is rendered.
        let at_line_start = self.out.is_empty() || self.out.ends_with('\n');
        if self.options.max_lines == Some(self.lines) && (at_line_start || s.ends_with('\n')) {
            // Drop any unfinished line, so the output ends at a whole line.
            let end = self.out.rfind('\n').map_or(0, |i| i + 1);
            self.out.truncate(end);
            self.cut = true;
            return Err(self.too_large());
        }
        self.fits(s.len())?;
        self.out.push_str(s);
        if s.ends_with('\n') {
            self.lines += 1;
        }
        Ok(())
    }

    fn unsupported(&self, ft: &FieldType) -> DynamicError {
        DynamicError::YamlUnsupportedField {
            path: shown(&self.path),
            field_type: type_name(ft),
        }
    }

    /// Emit `schema`'s fields as block-mapping lines at `indent` spaces.
    /// `value` is `None` for a zero-default message: an empty variable
    /// nested field (the producer idiom for "not set") renders as the
    /// target's zero values, so the output stays complete YAML. `dash`
    /// starts the first line with a sequence-element `- ` in its last two
    /// columns of indent.
    fn mapping(
        &mut self,
        schema: &MessageSchema,
        value: Option<&FrameValue<'_>>,
        indent: usize,
        depth: usize,
        dash: bool,
    ) -> Result<(), DynamicError> {
        if depth > MAX_YAML_DEPTH {
            return Err(DynamicError::YamlTooDeep {
                path: shown(&self.path),
                max: MAX_YAML_DEPTH,
            });
        }
        // An empty variable nested blob decodes to a value with no fields:
        // treat it as the zero default.
        let value = value.filter(|v| !v.fields.is_empty() || schema.fields.is_empty());
        let qname = schema.qualified_name();
        for (i, field) in schema.fields.iter().enumerate() {
            let kind = match value {
                Some(v) => {
                    Some(
                        v.field(&field.name)
                            .ok_or_else(|| DynamicError::YamlUnencodable {
                                path: shown(&self.path),
                                detail: format!("decoded frame has no field {}", field.name),
                            })?,
                    )
                }
                None => None,
            };
            let mark = self.path.len();
            push_field(&mut self.path, &field.name);
            let lead = if dash && i == 0 {
                format!("{:w$}- ", "", w = indent - 2)
            } else {
                format!("{:indent$}", "")
            };
            self.put(&format!("{lead}{}:", key(&field.name)))?;
            self.value(&field.field_type, kind, &qname, indent, depth)?;
            self.path.truncate(mark);
        }
        Ok(())
    }

    /// Emit one field value after its `key:`; writes the rest of the line
    /// (` <inline>\n`) or a newline plus a block.
    fn value(
        &mut self,
        ft: &FieldType,
        kind: Option<&FrameValueKind<'_>>,
        parent: &str,
        indent: usize,
        depth: usize,
    ) -> Result<(), DynamicError> {
        match ft {
            FieldType::Nested {
                schema_name,
                package,
                ..
            } => {
                let target = self.nested_schema(schema_name, package, parent)?;
                let inner = match kind {
                    None => None,
                    Some(FrameValueKind::Nested(v)) => Some(v.as_ref()),
                    Some(_) => return Err(self.unencodable("decoded value is not a message")),
                };
                if target.fields.is_empty() {
                    return self.put(" {}\n");
                }
                self.put("\n")?;
                self.mapping(target, inner, indent + 2, depth + 1, false)
            }
            FieldType::DynamicArray { element_type }
            | FieldType::FixedArray { element_type, .. }
                if matches!(element_type.as_ref(), FieldType::Nested { .. }) =>
            {
                self.message_array(ft, element_type, kind, parent, indent, depth)
            }
            _ => {
                let inline = self.inline(ft, kind)?;
                self.put(&format!(" {inline}\n"))
            }
        }
    }

    fn nested_schema(
        &self,
        schema_name: &str,
        package: &Option<String>,
        parent: &str,
    ) -> Result<&'w MessageSchema, DynamicError> {
        let target = self
            .walker
            .nested_layout(schema_name, package, parent)
            .ok_or_else(|| DynamicError::YamlUnresolvedNested {
                path: shown(&self.path),
                target: match package {
                    Some(p) => format!("{p}/{schema_name}"),
                    None => schema_name.to_string(),
                },
            })?;
        schema_of(self.walker, &target.qualified_name, &self.path)
    }

    /// An array of messages: a block sequence (`[]` when empty).
    fn message_array(
        &mut self,
        ft: &FieldType,
        element_type: &FieldType,
        kind: Option<&FrameValueKind<'_>>,
        parent: &str,
        indent: usize,
        depth: usize,
    ) -> Result<(), DynamicError> {
        let FieldType::Nested {
            schema_name,
            package,
            ..
        } = element_type
        else {
            return Err(self.unsupported(ft));
        };
        let target = self.nested_schema(schema_name, package, parent)?;
        // A zero-default fixed array is counted, never built: its length
        // comes from the schema, not from the frame.
        let (len, elements): (usize, Vec<&FrameValue<'_>>) = match (kind, ft) {
            (None, FieldType::FixedArray { length, .. }) => (*length, Vec::new()),
            (None, _) => (0, Vec::new()),
            (Some(FrameValueKind::Array(elems)), _)
            | (
                Some(FrameValueKind::NestedArray {
                    elements: elems, ..
                }),
                _,
            ) => {
                let values = elems
                    .iter()
                    .map(|e| match e {
                        FrameValueKind::Nested(v) => Ok(v.as_ref()),
                        _ => Err(self.unencodable("array element is not a message")),
                    })
                    .collect::<Result<Vec<_>, _>>()?;
                (values.len(), values)
            }
            (Some(FrameValueKind::NestedArrayOpaque(_)), _) => {
                return Err(self.unencodable("array bytes are not in the canonical element framing"))
            }
            (Some(_), _) => return Err(self.unencodable("decoded value is not an array")),
        };
        if len == 0 {
            return self.put(" []\n");
        }
        self.put("\n")?;
        let shown_count = self.shown_count(len);
        for i in 0..shown_count {
            let elem = elements.get(i).copied();
            let mark = self.path.len();
            push_index(&mut self.path, i);
            if target.fields.is_empty() {
                self.put(&format!("{:w$}- {{}}\n", "", w = indent + 2))?;
            } else {
                // The element's fields sit at indent + 4; its first line
                // carries the `- ` at indent + 2.
                self.mapping(target, elem, indent + 4, depth + 1, true)?;
            }
            self.path.truncate(mark);
        }
        if shown_count < len {
            self.put(&format!(
                "{:w$}- {YAML_TRUNCATED_TAG} {}\n",
                "",
                len - shown_count,
                w = indent + 2
            ))?;
        }
        Ok(())
    }

    fn shown_count(&self, len: usize) -> usize {
        self.options.truncate_arrays.map_or(len, |n| n.min(len))
    }

    /// A flow sequence of already-rendered scalars, honouring truncation.
    fn flow(
        &self,
        items: impl ExactSizeIterator<Item = Result<String, DynamicError>>,
    ) -> Result<String, DynamicError> {
        let len = items.len();
        let shown_count = self.shown_count(len);
        // `[`, `]`, each `, ` and the truncation marker count toward the
        // cap with the items, so a long array stops before it is built.
        let mut parts = Vec::new();
        let mut used = 2;
        let mut add = |part: String, parts: &mut Vec<String>| {
            used += part.len() + if parts.is_empty() { 0 } else { 2 };
            self.fits(used)?;
            parts.push(part);
            Ok::<(), DynamicError>(())
        };
        for item in items.take(shown_count) {
            add(item?, &mut parts)?;
        }
        if shown_count < len {
            add(
                format!("{YAML_TRUNCATED_TAG} {}", len - shown_count),
                &mut parts,
            )?;
        }
        Ok(format!("[{}]", parts.join(", ")))
    }

    /// The inline (single-line) form of a non-message value.
    fn inline(
        &self,
        ft: &FieldType,
        kind: Option<&FrameValueKind<'_>>,
    ) -> Result<String, DynamicError> {
        let Some(kind) = kind else {
            return self.zero_inline(ft);
        };
        type K<'a> = FrameValueKind<'a>;
        Ok(match (ft, kind) {
            (FieldType::Bool, K::Bool(v)) => v.to_string(),
            (FieldType::I8, K::I8(v)) => v.to_string(),
            (FieldType::U8, K::U8(v)) => v.to_string(),
            (FieldType::I16, K::I16(v)) => v.to_string(),
            (FieldType::U16, K::U16(v)) => v.to_string(),
            (FieldType::I32, K::I32(v)) => v.to_string(),
            (FieldType::U32, K::U32(v)) => v.to_string(),
            (FieldType::I64, K::I64(v)) => v.to_string(),
            (FieldType::U64, K::U64(v)) => v.to_string(),
            (FieldType::F32, K::F32(v)) => format_f32(*v),
            (FieldType::F64, K::F64(v)) => format_f64(*v),
            (FieldType::String | FieldType::StringFixed(_), K::Str(s)) => quote(s),
            (FieldType::String | FieldType::StringFixed(_), K::Bytes(_)) => {
                return Err(self.unencodable("string field is not valid UTF-8"))
            }
            (FieldType::Bytes, K::Bytes(b)) => self.flow(b.iter().map(|b| Ok(b.to_string())))?,
            (
                FieldType::DynamicArray { element_type }
                | FieldType::FixedArray { element_type, .. },
                _,
            ) => self.scalar_array(ft, element_type, kind)?,
            _ => return Err(self.unencodable("decoded value does not match the schema type")),
        })
    }

    fn scalar_array(
        &self,
        ft: &FieldType,
        element_type: &FieldType,
        kind: &FrameValueKind<'_>,
    ) -> Result<String, DynamicError> {
        type K<'a> = FrameValueKind<'a>;
        match (element_type, kind) {
            (FieldType::U8, K::Bytes(b)) => self.flow(b.iter().map(|b| Ok(b.to_string()))),
            (FieldType::I8, K::Bytes(b)) => self.flow(b.iter().map(|b| Ok((*b as i8).to_string()))),
            (FieldType::Bool, K::Bytes(b) | K::NestedArrayOpaque(b)) => {
                self.flow(b.iter().map(|b| match b {
                    0 => Ok("false".to_string()),
                    1 => Ok("true".to_string()),
                    other => Err(self.unencodable(format!("bool byte {other} is neither 0 nor 1"))),
                }))
            }
            (_, K::PrimArray(a)) if prim_of(element_type) == Some(a.elem) => {
                let size = a.elem.size();
                self.flow(
                    (0..a.count)
                        .map(|i| Ok(format_prim(a.elem, &a.bytes[i * size..(i + 1) * size]))),
                )
            }
            (FieldType::String, K::NestedArray { elements, .. }) => {
                self.flow(elements.iter().map(|e| match e {
                    K::Str(s) => Ok(quote(s)),
                    _ => Err(self.unencodable("string element is not valid UTF-8")),
                }))
            }
            (FieldType::StringFixed(n), K::NestedArrayOpaque(raw))
                if matches!(ft, FieldType::FixedArray { .. }) && *n > 0 =>
            {
                self.flow(raw.chunks(*n).map(|chunk| {
                    let end = chunk.iter().position(|&b| b == 0).unwrap_or(chunk.len());
                    std::str::from_utf8(&chunk[..end])
                        .map(quote)
                        .map_err(|_| self.unencodable("string element is not valid UTF-8"))
                }))
            }
            (FieldType::String, K::NestedArrayOpaque(_)) => {
                Err(self.unencodable("array bytes are not in the canonical element framing"))
            }
            // Every supported shape matched above (the walker derives each
            // value from the same schema), so what is left has no YAML form.
            _ => Err(self.unsupported(ft)),
        }
    }

    /// The zero value of a non-message field (for a zero-default message).
    fn zero_inline(&self, ft: &FieldType) -> Result<String, DynamicError> {
        Ok(match ft {
            FieldType::Bool => "false".to_string(),
            FieldType::I8
            | FieldType::U8
            | FieldType::I16
            | FieldType::U16
            | FieldType::I32
            | FieldType::U32
            | FieldType::I64
            | FieldType::U64 => "0".to_string(),
            FieldType::F32 | FieldType::F64 => "0.0".to_string(),
            FieldType::String | FieldType::StringFixed(_) => "\"\"".to_string(),
            FieldType::Bytes => "[]".to_string(),
            FieldType::DynamicArray { .. } if scalar_array_supported(ft) => "[]".to_string(),
            FieldType::FixedArray {
                element_type,
                length,
            } if scalar_array_supported(ft) => {
                let elem = self.zero_inline(element_type)?;
                self.flow((0..*length).map(|_| Ok(elem.clone())))?
            }
            _ => return Err(self.unsupported(ft)),
        })
    }
}

/// True for the array shapes the flow form covers.
fn scalar_array_supported(ft: &FieldType) -> bool {
    match ft {
        FieldType::DynamicArray { element_type } => matches!(
            element_type.as_ref(),
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
        ),
        FieldType::FixedArray { element_type, .. } => matches!(
            element_type.as_ref(),
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
        ),
        _ => false,
    }
}

fn prim_of(ft: &FieldType) -> Option<PrimType> {
    Some(match ft {
        FieldType::I16 => PrimType::I16,
        FieldType::U16 => PrimType::U16,
        FieldType::I32 => PrimType::I32,
        FieldType::U32 => PrimType::U32,
        FieldType::I64 => PrimType::I64,
        FieldType::U64 => PrimType::U64,
        FieldType::F32 => PrimType::F32,
        FieldType::F64 => PrimType::F64,
        _ => return None,
    })
}

/// One little-endian element of a primitive array, in its own type.
fn format_prim(elem: PrimType, b: &[u8]) -> String {
    let a2 = || [b[0], b[1]];
    let a4 = || [b[0], b[1], b[2], b[3]];
    let a8 = || [b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7]];
    match elem {
        PrimType::I16 => i16::from_le_bytes(a2()).to_string(),
        PrimType::U16 => u16::from_le_bytes(a2()).to_string(),
        PrimType::I32 => i32::from_le_bytes(a4()).to_string(),
        PrimType::U32 => u32::from_le_bytes(a4()).to_string(),
        PrimType::I64 => i64::from_le_bytes(a8()).to_string(),
        PrimType::U64 => u64::from_le_bytes(a8()).to_string(),
        PrimType::F32 => format_f32(f32::from_le_bytes(a4())),
        PrimType::F64 => format_f64(f64::from_le_bytes(a8())),
    }
}

/// Shortest decimal that reads back to `v` (Rust's `{:?}`), YAML spellings
/// for the non-finite values.
pub(super) fn format_f64(v: f64) -> String {
    if v.is_nan() {
        ".nan".to_string()
    } else if v.is_infinite() {
        if v.is_sign_positive() {
            ".inf"
        } else {
            "-.inf"
        }
        .to_string()
    } else {
        format!("{v:?}")
    }
}

/// As [`format_f64`], with the shortest decimal for the `f32` itself.
pub(super) fn format_f32(v: f32) -> String {
    if v.is_finite() {
        format!("{v:?}")
    } else {
        format_f64(f64::from(v))
    }
}

/// A mapping key: plain when it is an identifier YAML reads as a string,
/// double quoted otherwise (`"null"`, `"true"`, names with odd characters).
pub(super) fn key(name: &str) -> String {
    let identifier = name
        .chars()
        .next()
        .is_some_and(|c| c.is_ascii_alphabetic() || c == '_')
        && name.chars().all(|c| c.is_ascii_alphanumeric() || c == '_');
    let reserved = matches!(
        name,
        "null" | "Null" | "NULL" | "true" | "True" | "TRUE" | "false" | "False" | "FALSE"
    );
    if identifier && !reserved {
        name.to_string()
    } else {
        quote(name)
    }
}

/// A YAML double-quoted scalar. Escapes `"` and `\`, every character YAML
/// does not allow raw (C0 and C1 controls, DEL, U+FEFF, U+FFFE, U+FFFF)
/// and the characters a YAML reader folds as line breaks (U+0085,
/// U+2028, U+2029).
pub(super) fn quote(s: &str) -> String {
    let mut out = String::with_capacity(s.len() + 2);
    out.push('"');
    for c in s.chars() {
        match c {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\n"),
            '\t' => out.push_str("\\t"),
            '\r' => out.push_str("\\r"),
            '\0' => out.push_str("\\0"),
            c if (c as u32) < 0x20 || ('\u{7F}'..='\u{9F}').contains(&c) => {
                let _ = write!(out, "\\x{:02X}", c as u32);
            }
            '\u{2028}' | '\u{2029}' | '\u{FEFF}' | '\u{FFFE}' | '\u{FFFF}' => {
                let _ = write!(out, "\\u{:04X}", c as u32);
            }
            c => out.push(c),
        }
    }
    out.push('"');
    out
}
