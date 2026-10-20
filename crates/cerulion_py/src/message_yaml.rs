// SPDX-License-Identifier: AGPL-3.0-only
//! Message <-> dict / YAML for Python, as a thin layer over the core codec
//! (`cerulion_core::dynamic::{message_to_yaml, yaml_to_message}`).
//!
//! The codec is the only place that knows field names, types and value
//! forms. A dict is written out as flow YAML text and handed to
//! `yaml_to_message`, so a bad dict fails with the same field-named error
//! the same YAML would; a frame is rendered by `message_to_yaml` and the
//! text is read back into Python objects. Nothing here is a publish path:
//! every call allocates and runs once per explicit user call.

use crate::align::aligned_for_validation;
use crate::errors::{kinded, map_dynamic_err, EncodeError};
use crate::typed::PySchemaSet;
use cerulion_core::dynamic::{message_to_yaml, yaml_to_message, MAX_YAML_DEPTH};
use pyo3::buffer::PyBuffer;
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use pyo3::types::{
    PyBool, PyByteArray, PyBytes, PyDict, PyFloat, PyInt, PyList, PyString, PyTuple,
};
use pyo3::IntoPyObjectExt;
use serde_yaml::Value;
use std::fmt::Write as _;

/// How deep a dict may nest before it is refused. Messages nest at most
/// [`MAX_YAML_DEPTH`] levels, and each level of an array of messages adds
/// one list level, so twice that bounds every real message and stops a
/// dict that contains itself.
const MAX_DICT_DEPTH: usize = 2 * MAX_YAML_DEPTH;

/// The YAML text of the wire `frame` (header included); its `schema_hash`
/// picks the schema.
#[pyfunction]
pub(crate) fn message_to_yaml_text(
    py: Python<'_>,
    schemas: PyRef<'_, PySchemaSet>,
    frame: PyBuffer<u8>,
) -> PyResult<String> {
    let bytes = frame.to_vec(py)?;
    let mut scratch = Vec::new();
    let aligned = aligned_for_validation(&bytes, &mut scratch);
    message_to_yaml(schemas.inner.walker(), aligned).map_err(|e| map_dynamic_err(py, e))
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
    let text = message_to_yaml_text(py, schemas, frame)?;
    let value: Value = serde_yaml::from_str(&text).map_err(|e| {
        PyValueError::new_err(format!("codec output does not read back as YAML: {e}"))
    })?;
    to_python(py, &value).map(Bound::unbind)
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

fn to_python<'py>(py: Python<'py>, value: &Value) -> PyResult<Bound<'py, PyAny>> {
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
                list.append(to_python(py, item)?)?;
            }
            Ok(list.into_any())
        }
        Value::Mapping(map) => {
            let dict = PyDict::new(py);
            for (key, item) in map {
                dict.set_item(to_python(py, key)?, to_python(py, item)?)?;
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
/// type, a dict that contains itself, an integer past 64 bits, a `str`
/// with a lone surrogate) is refused here, naming the field path, with a
/// `Dict*` kind (or the codec's `YamlOutOfRange` for the integer).
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
        // `int.__repr__` gives the decimal digits of any int subclass
        // (an `IntEnum` member's own `repr` is its name).
        let repr = py.get_type::<PyInt>().call_method1("__repr__", (value,))?;
        let digits = repr.cast::<PyString>()?.to_str()?;
        // The YAML reader holds integers in 64 bits; a longer one would
        // fail as a parse error at a place in YAML the caller never wrote.
        if digits.parse::<i64>().is_err() && digits.parse::<u64>().is_err() {
            return Err(kinded::<EncodeError>(
                py,
                "YamlOutOfRange",
                format!(
                    "{}: {digits} is out of range for every integer type (64 bits at most)",
                    shown(path)
                ),
            ));
        }
        out.push_str(digits);
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
        let bytes: Vec<u8> = value.extract()?;
        out.push('[');
        for (index, byte) in bytes.iter().enumerate() {
            if index > 0 {
                out.push_str(", ");
            }
            let _ = write!(out, "{byte}");
        }
        out.push(']');
    } else if value.hasattr("tolist")? {
        // A NumPy array or scalar: its plain Python value.
        emit(&value.call_method0("tolist")?, out, path, depth + 1)?;
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
    Ok(())
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
fn push_quoted(out: &mut String, s: &str) {
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
}
