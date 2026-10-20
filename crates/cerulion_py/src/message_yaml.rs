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
use crate::errors::{map_dynamic_err, EncodeError};
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
/// full decimal text (an out-of-range one is the codec's to refuse).
fn emit(
    value: &Bound<'_, PyAny>,
    out: &mut String,
    path: &mut String,
    depth: usize,
) -> PyResult<()> {
    if depth > MAX_DICT_DEPTH {
        return Err(EncodeError::new_err(format!(
            "{}: the value nests deeper than {MAX_DICT_DEPTH} levels",
            shown(path)
        )));
    }
    if value.is_none() {
        out.push_str("null");
    } else if let Ok(b) = value.cast::<PyBool>() {
        out.push_str(if b.is_true() { "true" } else { "false" });
    } else if value.is_instance_of::<PyInt>() {
        // `int.__repr__` gives the decimal digits of any int subclass
        // (an `IntEnum` member's own `repr` is its name).
        let digits = value
            .py()
            .get_type::<PyInt>()
            .call_method1("__repr__", (value,))?;
        out.push_str(digits.cast::<PyString>()?.to_str()?);
    } else if let Ok(f) = value.cast::<PyFloat>() {
        push_float(out, f.value());
    } else if let Ok(s) = value.cast::<PyString>() {
        push_quoted(out, s.to_str()?);
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
                if !path.is_empty() {
                    path.push('.');
                }
                path.push_str(name.to_str()?);
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
        return Err(EncodeError::new_err(format!(
            "{}: a {} value has no message form; use dict, list, str, int, float, bool \
             or bytes",
            shown(path),
            value.get_type().name()?
        )));
    }
    Ok(())
}

fn shown(path: &str) -> &str {
    if path.is_empty() {
        "(message)"
    } else {
        path
    }
}

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

/// A YAML double-quoted string. Control characters and every character
/// YAML treats as a line break are escaped, so the text reads back
/// unchanged; other non-ASCII characters are written as they are.
fn push_quoted(out: &mut String, s: &str) {
    out.push('"');
    for c in s.chars() {
        match c {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\n"),
            '\t' => out.push_str("\\t"),
            '\r' => out.push_str("\\r"),
            c if c.is_control() || matches!(c, '\u{2028}' | '\u{2029}' | '\u{feff}') => {
                let _ = write!(out, "\\u{:04x}", c as u32);
            }
            c => out.push(c),
        }
    }
    out.push('"');
}
