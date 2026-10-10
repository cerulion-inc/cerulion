// SPDX-License-Identifier: AGPL-3.0-only
//! Schema-aware native helpers for the Python typed facade.

use crate::align::{aligned_for_validation, MAX_FIELD_ALIGN};
use crate::errors::{map_dynamic_err, EncodeError};
use crate::frame::Frame;
use cerulion_core::dynamic::{
    DynamicError, FrameValue, FrameValueKind, FrameView, PrimArray, PrimType, SchemaSet,
};
use cerulion_core::wire::WireHeader;
use pyo3::buffer::PyBuffer;
use pyo3::exceptions::{PyBufferError, PyValueError};
use pyo3::prelude::*;
use pyo3::types::{PyDict, PyList, PyTuple};
use pyo3::IntoPyObjectExt;
use std::collections::hash_map::Entry;
use std::collections::HashMap;
use std::path::PathBuf;

#[pyclass(name = "SchemaSet")]
pub struct PySchemaSet {
    pub(crate) inner: SchemaSet,
    warnings: Vec<String>,
}

#[pymethods]
impl PySchemaSet {
    #[new]
    fn new() -> PyResult<Self> {
        let (inner, warnings) = SchemaSet::from_schemas(Vec::new())
            .map_err(|e| PyValueError::new_err(format!("cannot create empty SchemaSet: {e}")))?;
        Ok(Self { inner, warnings })
    }

    #[staticmethod]
    fn from_workspace(path: PathBuf) -> PyResult<Self> {
        let (inner, warnings) = SchemaSet::from_workspace_dir(&path)
            .map_err(|e| Python::attach(|py| map_dynamic_err(py, e)))?;
        for warning in &warnings {
            tracing::warn!(warning = %warning, "schema workspace warning");
        }
        Ok(Self { inner, warnings })
    }

    fn add_yaml(&mut self, text: &str) -> PyResult<Vec<String>> {
        let warnings = self
            .inner
            .add_yaml_str(text)
            .map_err(|e| Python::attach(|py| map_dynamic_err(py, e)))?;
        self.warnings.extend(warnings.iter().cloned());
        Ok(warnings)
    }

    #[pyo3(signature = (text, name, package=None))]
    fn add_rosmsg(
        &mut self,
        text: &str,
        name: &str,
        package: Option<&str>,
    ) -> PyResult<Vec<String>> {
        let (package, name) = match (package, name.split_once('/')) {
            (Some(package), _) => (Some(package), name),
            (None, Some((package, name))) => (Some(package), name),
            (None, None) => (None, name),
        };
        let warnings = self
            .inner
            .add_rosmsg_str(text, name, package)
            .map_err(|e| Python::attach(|py| map_dynamic_err(py, e)))?;
        self.warnings.extend(warnings.iter().cloned());
        Ok(warnings)
    }

    fn names(&self) -> Vec<String> {
        self.inner
            .schemas()
            .iter()
            .map(|schema| schema.qualified_name())
            .collect()
    }

    /// `(qualified_name, package, name)` for every distinct `(package,
    /// name)` key, later definition winning, in first-definition order: the
    /// facade's nested-name resolution mirrors the core resolver, which
    /// keys on `(package, name)` and keeps the later of two duplicates. A
    /// package-less YAML schema named `pkg/Leaf` is NOT a `Leaf` in package
    /// `pkg` (its bare name is `pkg/Leaf`).
    fn schema_keys(&self) -> Vec<(String, Option<String>, String)> {
        let mut slots: HashMap<(Option<&str>, &str), usize> = HashMap::new();
        let mut keys: Vec<(String, Option<String>, String)> = Vec::new();
        for schema in self.inner.schemas() {
            let entry = (
                schema.qualified_name(),
                schema.package.clone(),
                schema.name.clone(),
            );
            match slots.entry((schema.package.as_deref(), schema.name.as_str())) {
                Entry::Occupied(slot) => keys[*slot.get()] = entry,
                Entry::Vacant(slot) => {
                    slot.insert(keys.len());
                    keys.push(entry);
                }
            }
        }
        keys
    }

    fn layout_json(&self, name: &str) -> PyResult<String> {
        let layout = self
            .inner
            .layout(name)
            .ok_or_else(|| DynamicError::UnknownSchema(name.to_string()))
            .map_err(|e| Python::attach(|py| map_dynamic_err(py, e)))?;
        layout
            .to_json()
            .map_err(|e| PyValueError::new_err(e.to_string()))
    }

    fn schema_hash(&self, name: &str) -> PyResult<u64> {
        self.inner.schema_hash(name).ok_or_else(|| {
            Python::attach(|py| map_dynamic_err(py, DynamicError::UnknownSchema(name.to_string())))
        })
    }

    fn schema_name_for_hash(&self, hash: u64) -> Option<String> {
        self.inner.schema_name_for_hash(hash).map(ToOwned::to_owned)
    }

    #[getter]
    fn warnings(&self) -> Vec<String> {
        self.warnings.clone()
    }

    fn resolve_frame(
        &self,
        py: Python<'_>,
        frame: PyRef<'_, Frame>,
        name: Option<&str>,
    ) -> PyResult<Py<PyAny>> {
        let mut scratch = Vec::new();
        let bytes = aligned_for_validation(frame.wire_bytes()?, &mut scratch);
        let view = match name {
            Some(name) => {
                let layout = self
                    .inner
                    .layout(name)
                    .ok_or_else(|| DynamicError::UnknownSchema(name.to_string()))
                    .map_err(|e| map_dynamic_err(py, e))?;
                FrameView::with_layout(layout, bytes).map_err(|e| map_dynamic_err(py, e))?
            }
            None => {
                FrameView::new(self.inner.walker(), bytes).map_err(|e| map_dynamic_err(py, e))?
            }
        };
        let schema_name = view.layout().qualified_name.clone();
        let output = PyDict::new(py);
        output.set_item("schema", &schema_name)?;
        let variables = PyDict::new(py);
        let payload = view.payload();
        let mut decoded = None;
        for variable in &view.layout().variable_fields {
            let value = view
                .variable_field(&variable.name)
                .map_err(|e| map_dynamic_err(py, e))?;
            let Some(offset) = (value.as_ptr() as usize).checked_sub(payload.as_ptr() as usize)
            else {
                return Err(PyValueError::new_err(
                    "variable field slice lies outside the frame payload",
                ));
            };
            let entry = match &variable.field_type {
                cerulion_core::dynamic::FieldType::String => {
                    view.str_field(&variable.name)
                        .map_err(|e| map_dynamic_err(py, e))?;
                    raw_descriptor(py, offset, value.len())?
                }
                // bool[] is a byte array on the wire (one byte per
                // element), so it takes the raw path like i8[]/u8[] -
                // `prim_array_field` deliberately rejects it.
                cerulion_core::dynamic::FieldType::DynamicArray { element_type }
                    if matches!(
                        element_type.as_ref(),
                        cerulion_core::dynamic::FieldType::I8
                            | cerulion_core::dynamic::FieldType::U8
                            | cerulion_core::dynamic::FieldType::Bool
                    ) =>
                {
                    raw_descriptor(py, offset, value.len())?
                }
                cerulion_core::dynamic::FieldType::DynamicArray { element_type }
                    if matches!(
                        element_type.as_ref(),
                        cerulion_core::dynamic::FieldType::I16
                            | cerulion_core::dynamic::FieldType::U16
                            | cerulion_core::dynamic::FieldType::I32
                            | cerulion_core::dynamic::FieldType::U32
                            | cerulion_core::dynamic::FieldType::I64
                            | cerulion_core::dynamic::FieldType::U64
                            | cerulion_core::dynamic::FieldType::F32
                            | cerulion_core::dynamic::FieldType::F64
                    ) =>
                {
                    let prim = view
                        .prim_array_field(&variable.name)
                        .map_err(|e| map_dynamic_err(py, e))?;
                    let dtype = prim_dtype(prim.elem);
                    let items = vec![
                        "prim".into_bound_py_any(py)?,
                        dtype.into_bound_py_any(py)?,
                        offset.into_bound_py_any(py)?,
                        prim.count.into_bound_py_any(py)?,
                    ];
                    PyTuple::new(py, items)?.into_any()
                }
                _ => {
                    if decoded.is_none() {
                        decoded = Some(
                            view.decode(self.inner.walker())
                                .map_err(|e| map_dynamic_err(py, e))?,
                        );
                    }
                    match decoded.as_ref() {
                        Some(decoded) => {
                            value_descriptor(py, decoded.field(&variable.name), payload)?
                        }
                        None => {
                            return Err(PyValueError::new_err(
                                "decoded frame value unexpectedly missing",
                            ))
                        }
                    }
                }
            };
            variables.set_item(&variable.name, entry)?;
        }
        output.set_item("variables", variables)?;
        Ok(output.into_any().unbind())
    }

    fn begin_frame(
        &self,
        name: &str,
        var_lens: Vec<usize>,
        timestamp_ns: u64,
    ) -> PyResult<Vec<u8>> {
        let layout = self
            .inner
            .layout(name)
            .ok_or_else(|| DynamicError::UnknownSchema(name.to_string()))
            .map_err(|e| Python::attach(|py| map_dynamic_err(py, e)))?;
        let encoder = cerulion_core::dynamic::FrameEncoder::new(layout)
            .map_err(|e| Python::attach(|py| map_dynamic_err(py, e)))?;
        let total = encoder
            .required_len(&var_lens)
            .map_err(|e| Python::attach(|py| map_dynamic_err(py, e)))?;
        // `begin` refuses a nonempty primitive array at a misaligned
        // address, and a `Vec<u8>` carries no alignment promise: encode
        // at the first `MAX_FIELD_ALIGN`-aligned byte of a padded buffer,
        // then shift the frame to the front (offsets are payload-relative).
        let mut frame = vec![0; total + MAX_FIELD_ALIGN];
        let start = frame.as_ptr().align_offset(MAX_FIELD_ALIGN);
        encoder
            .begin(&mut frame[start..start + total], &var_lens, timestamp_ns)
            .map_err(|e| Python::attach(|py| map_dynamic_err(py, e)))?;
        frame.drain(..start);
        frame.truncate(total);
        Ok(frame)
    }

    /// Check that `frame` is a complete, well-formed `name` frame before a
    /// typed publisher forwards it: the header, the offset table (bounds,
    /// overlap, alignment) and then the same walker decode a subscriber's
    /// `view()` performs, so whatever that reader would refuse (a bad offset
    /// inside a variable nested body, a truncated nested table) is refused
    /// here as `EncodeError` instead of being delivered as an unreadable
    /// frame. The contract is exactly the reader's: a nested array whose
    /// bytes are not canonically framed is not an error on either side (the
    /// core walker surfaces it as `NestedArrayOpaque` by design, so a
    /// producer-defined convention survives, and the subscriber reads it as
    /// raw bytes), and such a frame is forwarded verbatim. A schema-hash
    /// disagreement raises `SchemaMismatch`;
    /// every other structural fault raises `EncodeError`, since the frame
    /// is about to be sent, not read. Only the frame's own `total_size`
    /// bytes are copied for validation, and only after they are known to
    /// fit `max_payload_len`: trailing buffer capacity is never touched.
    fn validate_frame(
        &self,
        py: Python<'_>,
        name: &str,
        frame: PyBuffer<u8>,
        max_payload_len: usize,
    ) -> PyResult<()> {
        let layout = self
            .inner
            .layout(name)
            .ok_or_else(|| DynamicError::UnknownSchema(name.to_string()))
            .map_err(|e| map_dynamic_err(py, e))?;
        let src = frame
            .as_slice(py)
            .ok_or_else(|| PyBufferError::new_err("frame must be a contiguous buffer"))?;
        if src.len() < WireHeader::SIZE {
            return Err(EncodeError::new_err(
                "frame is shorter than the wire header",
            ));
        }
        let header_bytes: [u8; WireHeader::SIZE] = std::array::from_fn(|index| src[index].get());
        let header = WireHeader::read_from_buf(&header_bytes)
            .ok_or_else(|| EncodeError::new_err("frame is shorter than the wire header"))?;
        // Hash first, as `publisher.rs` does: a foreign frame is a
        // `SchemaMismatch` whatever its size.
        if header.schema_hash != layout.schema_hash {
            return Err(crate::errors::schema_mismatch(
                py,
                format!(
                    "frame schema hash {:#x} does not match the publisher's {:#x}",
                    header.schema_hash, layout.schema_hash
                ),
            ));
        }
        let total = header.total_size as usize;
        if total < WireHeader::SIZE || total > src.len() {
            return Err(EncodeError::new_err(
                "frame total_size is outside its buffer",
            ));
        }
        let payload_len = total - WireHeader::SIZE;
        if payload_len > max_payload_len {
            return Err(EncodeError::new_err(format!(
                "frame payload {payload_len} exceeds max_payload_len {max_payload_len}"
            )));
        }
        // Validation needs an aligned view; the caller's buffer may be any
        // bytes-like object, so copy the frame (bounded above) once into
        // aligned scratch.
        let mut scratch = vec![0u8; total + MAX_FIELD_ALIGN];
        let start = scratch.as_ptr().align_offset(MAX_FIELD_ALIGN);
        for (dst, value) in scratch[start..start + total].iter_mut().zip(src) {
            *dst = value.get();
        }
        FrameView::with_layout(layout, &scratch[start..start + total])
            .and_then(|view| view.decode(self.inner.walker()).map(|_| ()))
            .map_err(|e| match e {
                DynamicError::SchemaHashMismatch { .. } | DynamicError::UnknownSchemaHash(_) => {
                    map_dynamic_err(py, e)
                }
                other => {
                    EncodeError::new_err(format!("frame is not a valid {name} frame: {other}"))
                }
            })
    }
}

/// `slice` must lie inside `payload` (both borrow the same frame); a
/// subtraction that underflows means the core handed back a foreign
/// slice - an internal error, never a wrap.
fn slice_offset(slice: &[u8], payload: &[u8]) -> PyResult<usize> {
    (slice.as_ptr() as usize)
        .checked_sub(payload.as_ptr() as usize)
        .ok_or_else(|| PyValueError::new_err("field slice lies outside the frame payload"))
}

fn prim_dtype(kind: PrimType) -> &'static str {
    match kind {
        PrimType::I16 => "<i2",
        PrimType::U16 => "<u2",
        PrimType::I32 => "<i4",
        PrimType::U32 => "<u4",
        PrimType::I64 => "<i8",
        PrimType::U64 => "<u8",
        PrimType::F32 => "<f4",
        PrimType::F64 => "<f8",
    }
}

fn raw_descriptor<'py>(py: Python<'py>, offset: usize, len: usize) -> PyResult<Bound<'py, PyAny>> {
    let items = vec![
        "raw".into_bound_py_any(py)?,
        offset.into_bound_py_any(py)?,
        len.into_bound_py_any(py)?,
    ];
    Ok(PyTuple::new(py, items)?.into_any())
}

fn value_descriptor<'py, 'a>(
    py: Python<'py>,
    value: Option<&FrameValueKind<'a>>,
    payload: &'a [u8],
) -> PyResult<Bound<'py, PyAny>> {
    let Some(value) = value else {
        return Ok(py.None().into_bound(py));
    };
    match value {
        FrameValueKind::Bool(v) => Ok(v.into_bound_py_any(py)?),
        FrameValueKind::I8(v) => Ok(v.into_bound_py_any(py)?),
        FrameValueKind::U8(v) => Ok(v.into_bound_py_any(py)?),
        FrameValueKind::I16(v) => Ok(v.into_bound_py_any(py)?),
        FrameValueKind::U16(v) => Ok(v.into_bound_py_any(py)?),
        FrameValueKind::I32(v) => Ok(v.into_bound_py_any(py)?),
        FrameValueKind::U32(v) => Ok(v.into_bound_py_any(py)?),
        FrameValueKind::I64(v) => Ok(v.into_bound_py_any(py)?),
        FrameValueKind::U64(v) => Ok(v.into_bound_py_any(py)?),
        FrameValueKind::F32(v) => Ok(v.into_bound_py_any(py)?),
        FrameValueKind::F64(v) => Ok(v.into_bound_py_any(py)?),
        FrameValueKind::Str(v) => Ok(v.into_bound_py_any(py)?),
        FrameValueKind::Bytes(v) | FrameValueKind::NestedArrayOpaque(v) => {
            let offset = slice_offset(v, payload)?;
            Ok(raw_descriptor(py, offset, v.len())?)
        }
        FrameValueKind::PrimArray(PrimArray { elem, count, bytes }) => {
            let offset = slice_offset(bytes, payload)?;
            let items = vec![
                "prim".into_bound_py_any(py)?,
                prim_dtype(*elem).into_bound_py_any(py)?,
                offset.into_bound_py_any(py)?,
                count.into_bound_py_any(py)?,
            ];
            Ok(PyTuple::new(py, items)?.into_any())
        }
        FrameValueKind::Nested(value) => {
            let dict = PyDict::new(py);
            dict.set_item("nested", frame_value_descriptor(py, value, payload)?)?;
            Ok(dict.into_any())
        }
        FrameValueKind::Array(values)
        | FrameValueKind::NestedArray {
            elements: values, ..
        } => {
            let list = PyList::empty(py);
            for value in values {
                list.append(value_descriptor(py, Some(value), payload)?)?;
            }
            Ok(list.into_any())
        }
    }
}

fn frame_value_descriptor<'py, 'a>(
    py: Python<'py>,
    value: &FrameValue<'a>,
    payload: &'a [u8],
) -> PyResult<Bound<'py, PyAny>> {
    let dict = PyDict::new(py);
    dict.set_item("schema", &value.schema_name)?;
    let fields = PyDict::new(py);
    for field in &value.fields {
        fields.set_item(
            &field.name,
            value_descriptor(py, Some(&field.value), payload)?,
        )?;
    }
    dict.set_item("fields", fields)?;
    Ok(dict.into_any())
}
