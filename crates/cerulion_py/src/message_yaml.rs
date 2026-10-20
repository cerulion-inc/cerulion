// SPDX-License-Identifier: AGPL-3.0-only
//! Message <-> dict / YAML for Python, as a thin layer over the core codec
//! (`cerulion_core::dynamic::{message_to_yaml, yaml_to_message}`).
//!
//! The codec is the only place that knows field names, types and value
//! forms. A dict is written out as flow YAML text and handed to
//! `yaml_to_message`, so a bad dict fails with the same field-named error
//! the same YAML would; a frame is rendered by `message_to_yaml` and the
//! text is read back into Python objects, under bounds scaled to the frame
//! (see [`dict_value_budget`]). Nothing here is a publish path:
//! every call allocates and runs once per explicit user call.

use crate::align::aligned_for_validation;
use crate::errors::{kinded, map_dynamic_err, DecodeError, EncodeError};
use crate::typed::PySchemaSet;
use cerulion_core::dynamic::{
    message_to_yaml_capped, yaml_to_message, YamlEncodeOptions, MAX_YAML_BYTES,
};
use pyo3::buffer::PyBuffer;
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use pyo3::types::{
    PyBool, PyByteArray, PyBytes, PyDict, PyFloat, PyInt, PyList, PyString, PyTuple,
};
use pyo3::IntoPyObjectExt;
use serde_yaml::Value;
use std::fmt::Write as _;

/// How many nested collections (mappings and sequences) the codec's YAML
/// reader takes before it refuses the text (`serde_yaml`'s recursion
/// limit); a deeper dict can never encode.
const YAML_READER_DEPTH: usize = 128;

/// How deep a dict may nest before it is refused. The root dict is level 0,
/// so the reader's collections fill levels 0 to 127 and their scalars sit
/// at 128; a NumPy value adds one level for its `tolist()`. So every value
/// the codec can take fits, and a dict that contains itself stops here.
const MAX_DICT_DEPTH: usize = YAML_READER_DEPTH + 1;

/// How much YAML a dict may write before it is refused: the codec's own
/// output cap. A shared list can name far more text than it holds
/// (`[a, a]` nested twenty times is a million copies of `a`), so the cap
/// turns that into an error before the text fills memory.
const MAX_DICT_YAML_BYTES: usize = MAX_YAML_BYTES;

/// The YAML text a frame may render to on its way to a dict: this much
/// per frame byte, plus [`DICT_TEXT_BASE`], and never past
/// [`MAX_YAML_BYTES`]. An unset nested field renders as its schema's zero
/// values, so a few frame bytes can name far more text than they hold;
/// the YAML reader and the Python values it feeds grow with the text, so
/// the dict path bounds it by the frame, where `to_yaml` alone keeps the
/// codec's cap.
const DICT_TEXT_PER_FRAME_BYTE: usize = 64;

/// The text every frame may render to on its way to a dict (16 MiB).
const DICT_TEXT_BASE: usize = 16 << 20;

/// The Python values (each dict, list, key and scalar) a frame may become:
/// this many per frame byte, plus [`DICT_VALUES_BASE`]. A frame value
/// takes at least one byte except where the schema alone gives it (an
/// unset nested field, an empty message), and a key adds one value per
/// field, so real frames sit well inside the bound.
const DICT_VALUES_PER_FRAME_BYTE: usize = 4;

/// The values every frame may become (1 Mi).
const DICT_VALUES_BASE: usize = 1 << 20;

/// The most YAML text and the most Python values the dict of a
/// `frame_len`-byte frame may take.
fn dict_value_budget(frame_len: usize) -> (usize, usize) {
    let text = frame_len
        .saturating_mul(DICT_TEXT_PER_FRAME_BYTE)
        .saturating_add(DICT_TEXT_BASE)
        .min(MAX_YAML_BYTES);
    let values = frame_len
        .saturating_mul(DICT_VALUES_PER_FRAME_BYTE)
        .saturating_add(DICT_VALUES_BASE);
    (text, values)
}

/// The YAML text of the wire `frame` (header included); its `schema_hash`
/// picks the schema.
#[pyfunction]
pub(crate) fn message_to_yaml_text(
    py: Python<'_>,
    schemas: PyRef<'_, PySchemaSet>,
    frame: PyBuffer<u8>,
) -> PyResult<String> {
    frame_yaml(py, &schemas, frame, |_| MAX_YAML_BYTES).map(|(text, _)| text)
}

/// The YAML text of the wire `frame` under the cap `max_text` picks from
/// the frame length, and that length.
fn frame_yaml(
    py: Python<'_>,
    schemas: &PySchemaSet,
    frame: PyBuffer<u8>,
    max_text: impl FnOnce(usize) -> usize,
) -> PyResult<(String, usize)> {
    let bytes = frame.to_vec(py)?;
    let mut scratch = Vec::new();
    let aligned = aligned_for_validation(&bytes, &mut scratch);
    let text = message_to_yaml_capped(
        schemas.inner.walker(),
        aligned,
        YamlEncodeOptions::default(),
        max_text(bytes.len()),
    )
    .map_err(|e| map_dynamic_err(py, e))?;
    Ok((text, bytes.len()))
}

/// The wire frame (header included) of the YAML `text` as one `name`
/// message.
#[pyfunction]
pub(crate) fn yaml_to_message_bytes<'py>(
    py: Python<'py>,
    schemas: PyRef<'_, PySchemaSet>,
    name: &str,
    text: &[u8],
    timestamp_ns: u64,
) -> PyResult<Bound<'py, PyBytes>> {
    let frame = yaml_to_message(schemas.inner.walker(), name, text, timestamp_ns)
        .map_err(|e| map_dynamic_err(py, e))?;
    Ok(PyBytes::new(py, &frame))
}

/// The plain Python value of the wire `frame`: dicts in field declaration
/// order, lists, `str`, `int`, `float` and `bool`.
#[pyfunction]
pub(crate) fn message_to_dict(
    py: Python<'_>,
    schemas: PyRef<'_, PySchemaSet>,
    frame: PyBuffer<u8>,
) -> PyResult<Py<PyAny>> {
    let (text, frame_len) = frame_yaml(py, &schemas, frame, |len| dict_value_budget(len).0)?;
    let max_values = dict_value_budget(frame_len).1;
    let value: Value = serde_yaml::from_str(&text).map_err(|e| read_back_error(py, &e))?;
    let mut budget = Budget {
        left: max_values,
        max: max_values,
        frame_len,
    };
    to_python(py, &value, &mut budget).map(Bound::unbind)
}

/// The text of the YAML reader's error for input nested past
/// [`YAML_READER_DEPTH`]. The codec prints messages nested up to its own
/// message bound, and each level of a message array is two collections,
/// so a frame the codec prints can still be deeper than the reader takes.
const YAML_READER_DEPTH_MESSAGE: &str = "recursion limit exceeded";

/// The error for codec output the YAML reader refuses: `DecodeError`
/// `DictTooDeep` for output nested past the reader's limit (no dict of
/// that depth could encode either), else a plain `ValueError`, since the
/// codec writes no other text the reader refuses.
fn read_back_error(py: Python<'_>, e: &serde_yaml::Error) -> PyErr {
    let reason = e.to_string();
    if reason.contains(YAML_READER_DEPTH_MESSAGE) {
        return kinded::<DecodeError>(
            py,
            "DictTooDeep",
            format!(
                "the dict of this frame nests deeper than the {YAML_READER_DEPTH} collections \
                 a dict may hold; read it with to_yaml()"
            ),
        );
    }
    PyValueError::new_err(format!("codec output does not read back as YAML: {reason}"))
}

/// The Python values [`to_python`] may still build for one frame.
struct Budget {
    left: usize,
    max: usize,
    frame_len: usize,
}

impl Budget {
    /// Take one value, or refuse with `DictTooLarge` once the budget is
    /// spent: nothing past it is built.
    fn take(&mut self, py: Python<'_>) -> PyResult<()> {
        if self.left == 0 {
            return Err(kinded::<DecodeError>(
                py,
                "DictTooLarge",
                format!(
                    "the dict of this {}-byte frame passes its {}-value budget; \
                     read it with to_yaml()",
                    self.frame_len, self.max
                ),
            ));
        }
        self.left -= 1;
        Ok(())
    }
}

/// The wire frame (header included) of the Python `value` as one `name`
/// message: `value` is written out as flow YAML and read by the codec.
#[pyfunction]
pub(crate) fn dict_to_message_bytes<'py>(
    py: Python<'py>,
    schemas: PyRef<'_, PySchemaSet>,
    name: &str,
    value: &Bound<'py, PyAny>,
    timestamp_ns: u64,
) -> PyResult<Bound<'py, PyBytes>> {
    let mut text = String::new();
    emit(value, &mut text, &mut String::new(), 0)?;
    yaml_to_message_bytes(py, schemas, name, text.as_bytes(), timestamp_ns)
}

fn to_python<'py>(
    py: Python<'py>,
    value: &Value,
    budget: &mut Budget,
) -> PyResult<Bound<'py, PyAny>> {
    budget.take(py)?;
    match value {
        Value::Null => Ok(py.None().into_bound(py)),
        Value::Bool(b) => b.into_bound_py_any(py),
        Value::Number(n) => {
            if let Some(u) = n.as_u64() {
                u.into_bound_py_any(py)
            } else if let Some(i) = n.as_i64() {
                i.into_bound_py_any(py)
            } else {
                n.as_f64().unwrap_or(f64::NAN).into_bound_py_any(py)
            }
        }
        Value::String(s) => s.into_bound_py_any(py),
        Value::Sequence(items) => {
            let list = PyList::empty(py);
            for item in items {
                list.append(to_python(py, item, budget)?)?;
            }
            Ok(list.into_any())
        }
        Value::Mapping(map) => {
            let dict = PyDict::new(py);
            for (key, item) in map {
                dict.set_item(to_python(py, key, budget)?, to_python(py, item, budget)?)?;
            }
            Ok(dict.into_any())
        }
        Value::Tagged(_) => Err(PyValueError::new_err(
            "codec output carries a YAML tag, which no Python value represents",
        )),
    }
}

/// Write `value` as flow YAML. Every scalar is written the way a person
/// would type it, so the codec reports a wrong value exactly as it would
/// for typed YAML: a string stays a quoted string, an integer stays its
/// full decimal text. What has no YAML text at all (a value of no message
/// type, a dict that contains itself, YAML past [`MAX_DICT_YAML_BYTES`],
/// an integer too long for decimal text, a `str` with a lone surrogate)
/// is refused here, naming the field path, with a `Dict*` kind (or the
/// codec's `YamlOutOfRange` for the integer).
fn emit(
    value: &Bound<'_, PyAny>,
    out: &mut String,
    path: &mut String,
    depth: usize,
) -> PyResult<()> {
    let py = value.py();
    if depth > MAX_DICT_DEPTH {
        return Err(kinded::<EncodeError>(
            py,
            "DictTooDeep",
            format!(
                "{}: the value nests deeper than {MAX_DICT_DEPTH} levels",
                abridged(path)
            ),
        ));
    }
    if value.is_none() {
        out.push_str("null");
    } else if let Ok(b) = value.cast::<PyBool>() {
        out.push_str(if b.is_true() { "true" } else { "false" });
    } else if value.is_instance_of::<PyInt>() {
        // The int's value, not its `repr` (an `IntEnum` member's is its
        // name). An int past 64 bits is written as its decimal digits too,
        // so the codec decides, as it does for the same YAML text: an
        // integer field refuses it and a float field takes it.
        if let Ok(i) = value.extract::<i64>() {
            let _ = write!(out, "{i}");
        } else if let Ok(u) = value.extract::<u64>() {
            let _ = write!(out, "{u}");
        } else {
            out.push_str(&wide_int(value, path)?);
        }
    } else if let Ok(f) = value.cast::<PyFloat>() {
        push_float(out, f.value());
    } else if let Ok(s) = value.cast::<PyString>() {
        push_quoted(out, text_of(s, path)?);
    } else if let Ok(dict) = value.cast::<PyDict>() {
        out.push('{');
        for (index, (key, item)) in dict.iter().enumerate() {
            if index > 0 {
                out.push_str(", ");
            }
            emit(&key, out, path, depth + 1)?;
            out.push_str(": ");
            let len = path.len();
            if let Ok(name) = key.cast::<PyString>() {
                let name = text_of(name, path)?;
                if !path.is_empty() {
                    path.push('.');
                }
                path.push_str(name);
            }
            emit(&item, out, path, depth + 1)?;
            path.truncate(len);
        }
        out.push('}');
    } else if value.is_instance_of::<PyList>() || value.is_instance_of::<PyTuple>() {
        out.push('[');
        for (index, item) in value.try_iter()?.enumerate() {
            if index > 0 {
                out.push_str(", ");
            }
            let len = path.len();
            let _ = write!(path, "[{index}]");
            emit(&item?, out, path, depth + 1)?;
            path.truncate(len);
        }
        out.push(']');
    } else if value.is_instance_of::<PyBytes>() || value.is_instance_of::<PyByteArray>() {
        // A byte array (`uint8[]`, `bytes`) is a sequence of integers in
        // the YAML form.
        // Each byte writes at least one character, so a value that cannot
        // fit is refused before it is copied out of Python.
        if out.len().saturating_add(value.len()?) > MAX_DICT_YAML_BYTES {
            return Err(too_large(py, path));
        }
        let bytes: Vec<u8> = value.extract()?;
        out.push('[');
        for (index, byte) in bytes.iter().enumerate() {
            if index > 0 {
                out.push_str(", ");
            }
            let _ = write!(out, "{byte}");
            within_budget(py, out, path)?;
        }
        out.push(']');
    } else if let Some(tolist) = callable_tolist(value)? {
        // A NumPy array or scalar: its plain Python value.
        emit(&tolist.call0()?, out, path, depth + 1)?;
    } else {
        return Err(kinded::<EncodeError>(
            py,
            "DictUnsupportedValue",
            format!(
                "{}: a {} value has no message form; use dict, list, str, int, float, bool \
                 or bytes",
                shown(path),
                value.get_type().name()?
            ),
        ));
    }
    within_budget(py, out, path)
}

/// `Ok(())` while `out` is within [`MAX_DICT_YAML_BYTES`], else the
/// `DictTooLarge` error naming `path`.
fn within_budget(py: Python<'_>, out: &str, path: &str) -> PyResult<()> {
    if out.len() <= MAX_DICT_YAML_BYTES {
        return Ok(());
    }
    Err(too_large(py, path))
}

/// The `DictTooLarge` error naming `path`.
fn too_large(py: Python<'_>, path: &str) -> PyErr {
    kinded::<EncodeError>(
        py,
        "DictTooLarge",
        format!(
            "{}: the YAML form passes the {MAX_DICT_YAML_BYTES}-byte cap",
            shown(path)
        ),
    )
}

/// The bound `tolist` method of `value` when it has a callable one (a
/// NumPy array or scalar); `None` for a value with no such attribute or a
/// non-callable one, which then has no message form.
fn callable_tolist<'py>(value: &Bound<'py, PyAny>) -> PyResult<Option<Bound<'py, PyAny>>> {
    match value.getattr("tolist") {
        Ok(tolist) if tolist.is_callable() => Ok(Some(tolist)),
        Ok(_) => Ok(None),
        Err(e) if e.is_instance_of::<pyo3::exceptions::PyAttributeError>(value.py()) => Ok(None),
        Err(e) => Err(e),
    }
}

/// The decimal digits of an int past 64 bits, or the `YamlOutOfRange`
/// error naming `path` when Python has no decimal text for it (past its
/// `int_max_str_digits`, 4300 digits by default): no field type takes a
/// value that wide.
fn wide_int(value: &Bound<'_, PyAny>, path: &str) -> PyResult<String> {
    let py = value.py();
    let int = py.get_type::<PyInt>();
    // `int.__repr__` gives the digits of any int subclass.
    match int.call_method1("__repr__", (value,)) {
        Ok(digits) => digits.extract(),
        Err(e) if e.is_instance_of::<PyValueError>(py) => {
            let bits: u64 = int.call_method1("bit_length", (value,))?.extract()?;
            Err(kinded::<EncodeError>(
                py,
                "YamlOutOfRange",
                format!(
                    "{}: a {bits}-bit integer is out of range for every field type",
                    shown(path)
                ),
            ))
        }
        Err(e) => Err(e),
    }
}

/// The UTF-8 text of `s`, or an `EncodeError` naming `path` when `s`
/// holds a lone surrogate (UTF-8 has no form for one).
fn text_of<'a>(s: &'a Bound<'_, PyString>, path: &str) -> PyResult<&'a str> {
    s.to_str().map_err(|_| {
        kinded::<EncodeError>(
            s.py(),
            "DictNotUtf8",
            format!(
                "{}: the str holds a lone surrogate, which UTF-8 cannot encode",
                shown(path)
            ),
        )
    })
}

fn shown(path: &str) -> &str {
    if path.is_empty() {
        "(message)"
    } else {
        path
    }
}

/// [`shown`], cut to its first [`ABRIDGED_CHARS`] characters plus `...`:
/// the path of a value nested past the depth bound is too long to print.
fn abridged(path: &str) -> String {
    match path.char_indices().nth(ABRIDGED_CHARS) {
        Some((at, _)) => format!("{}...", &path[..at]),
        None => shown(path).to_string(),
    }
}

/// How much of a too-deep path the depth error prints.
const ABRIDGED_CHARS: usize = 60;

/// A float in a form YAML reads back to the same `f64`: Rust's shortest
/// round-trip digits (`1.0`, `1e-7`), and `.nan`, `.inf`, `-.inf`.
fn push_float(out: &mut String, v: f64) {
    if v.is_nan() {
        out.push_str(".nan");
    } else if v.is_infinite() {
        out.push_str(if v > 0.0 { ".inf" } else { "-.inf" });
    } else {
        let _ = write!(out, "{v:?}");
    }
}

/// A YAML double-quoted string, escaped exactly as the codec's encoder
/// quotes a string (`message_yaml/encode.rs`, `quote_within`): libyaml
/// refuses C0 and C1 controls, line separators, the BOM and the two
/// noncharacters raw, so each is escaped and the text reads back
/// unchanged; other characters are written as they are.
///
/// It stops writing once `out` passes [`MAX_DICT_YAML_BYTES`]; the caller
/// then refuses the text, so a long `str` never grows it much further.
fn push_quoted(out: &mut String, s: &str) {
    out.push('"');
    for c in s.chars() {
        if out.len() > MAX_DICT_YAML_BYTES {
            return;
        }
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
}
