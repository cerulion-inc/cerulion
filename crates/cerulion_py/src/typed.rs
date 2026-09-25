// SPDX-License-Identifier: AGPL-3.0-only
//! Schema-aware native helpers for the Python typed facade.

use crate::errors::map_dynamic_err;
use crate::frame::Frame;
use cerulion_core::dynamic::{
    DynamicError, FrameValue, FrameValueKind, FrameView, PrimArray, PrimType, SchemaSet,
};
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use pyo3::types::{PyDict, PyList, PyTuple};
use pyo3::IntoPyObjectExt;
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
        let mut frame = vec![0; total];
        encoder
            .begin(&mut frame, &var_lens, timestamp_ns)
            .map_err(|e| Python::attach(|py| map_dynamic_err(py, e)))?;
        Ok(frame)
    }
}

/// Widest primitive alignment a typed frame field can require (`f64`/`u64`).
const MAX_FIELD_ALIGN: usize = 8;

/// `bytes` itself when it is `MAX_FIELD_ALIGN`-aligned, else a copy of it in
/// an aligned region of `scratch`. SHM slots are not guaranteed 8-byte
/// aligned, and `FrameView` refuses a primitive array at a misaligned
/// address. Resolution only reads offsets relative to the payload start, so
/// they index the original frame unchanged and the NumPy views stay
/// zero-copy over the slot (NumPy reads unaligned arrays).
fn aligned_for_validation<'a>(bytes: &'a [u8], scratch: &'a mut Vec<u8>) -> &'a [u8] {
    if (bytes.as_ptr() as usize).is_multiple_of(MAX_FIELD_ALIGN) {
        return bytes;
    }
    scratch.clear();
    scratch.resize(bytes.len() + MAX_FIELD_ALIGN, 0);
    let start = scratch.as_ptr().align_offset(MAX_FIELD_ALIGN);
    scratch[start..start + bytes.len()].copy_from_slice(bytes);
    &scratch[start..start + bytes.len()]
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

#[cfg(test)]
mod tests {
    use super::{aligned_for_validation, MAX_FIELD_ALIGN};

    #[repr(align(8))]
    struct Backing([u8; 32]);

    #[test]
    fn a_misaligned_frame_is_validated_from_an_aligned_copy() {
        let backing = Backing(core::array::from_fn(|i| i as u8));
        let misaligned = &backing.0[1..17];
        let mut scratch = Vec::new();
        let aligned = aligned_for_validation(misaligned, &mut scratch);
        assert_eq!(
            aligned,
            &[1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16]
        );
        assert!((aligned.as_ptr() as usize).is_multiple_of(MAX_FIELD_ALIGN));
    }

    #[test]
    fn an_aligned_frame_is_used_in_place() {
        let backing = Backing([7; 32]);
        let mut scratch = Vec::new();
        let used = aligned_for_validation(&backing.0[8..24], &mut scratch);
        assert_eq!(used.as_ptr(), backing.0[8..24].as_ptr());
        assert!(scratch.is_empty());
    }
}
