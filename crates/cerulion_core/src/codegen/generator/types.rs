// SPDX-License-Identifier: AGPL-3.0-only
//! Type mapping utilities for code generation.
//!
//! Maps `FieldType` variants to Rust type strings and provides
//! primitive type introspection helpers.

use crate::codegen::schema::FieldType;

/// Convert FieldType to Rust type string (for fixed structs/views).
///
/// Uses fully qualified paths for stdlib types to avoid name conflicts
/// with user-defined message types (e.g., `std_msgs::String`).
pub(super) fn field_type_to_rust(ft: &FieldType) -> String {
    match ft {
        FieldType::Bool => "bool".to_string(),
        FieldType::I8 => "i8".to_string(),
        FieldType::U8 => "u8".to_string(),
        FieldType::I16 => "i16".to_string(),
        FieldType::U16 => "u16".to_string(),
        FieldType::I32 => "i32".to_string(),
        FieldType::U32 => "u32".to_string(),
        FieldType::I64 => "i64".to_string(),
        FieldType::U64 => "u64".to_string(),
        FieldType::F32 => "f32".to_string(),
        FieldType::F64 => "f64".to_string(),
        // Use fully qualified path to avoid conflict with std_msgs::String
        FieldType::String => "::std::string::String".to_string(),
        FieldType::Bytes => "::std::vec::Vec<u8>".to_string(),
        FieldType::StringFixed(n) => format!("[u8; {n}]"),
        FieldType::FixedArray {
            element_type,
            length,
        } => {
            let elem = field_type_to_rust(element_type);
            format!("[{elem}; {length}]")
        }
        FieldType::DynamicArray { element_type } => {
            let elem = field_type_to_rust(element_type);
            format!("::std::vec::Vec<{elem}>")
        }
        FieldType::Nested { schema_name, .. } => schema_name.clone(),
    }
}

/// Convert FieldType to a bytemuck-friendly Rust type string for the
/// SHM-backed `<Name>Shm` struct.
///
/// Differs from `field_type_to_rust` in two places:
/// - `Bool` maps to `u8`. Bool is not safely transmutable from arbitrary
///   bytes — only `0` and `1` are valid bit patterns. SHM-backed structs
///   store the wire byte directly as `u8`; the `<Name>Snapshot` companion
///   uses real `bool` and the round-trip helpers handle conversion.
/// - `Nested { schema_name }` maps to `<schema_name>Shm` so SHM composition
///   goes through the SHM-backed accessor (the marker `<schema_name>` is
///   unit and zero-sized, so embedding it directly would break wire layout).
pub(super) fn field_type_to_rust_shm(ft: &FieldType) -> String {
    match ft {
        FieldType::Bool => "u8".to_string(),
        FieldType::FixedArray {
            element_type,
            length,
        } => {
            let elem = field_type_to_rust_shm(element_type);
            format!("[{elem}; {length}]")
        }
        FieldType::Nested { schema_name, .. } => format!("{schema_name}Shm"),
        // Callers (the fixed-only `<Name>Shm` emitter and the
        // `<Name>FixedSection` emitter for variable schemas) gate emission on the
        // field actually living in the fixed section — `is_definitely_fixed()`
        // for fixed-only schemas, `is_fixed_section_field()` for variable
        // schemas. Either way, the remaining variants (String / Bytes /
        // DynamicArray) cannot reach this `_` arm. Delegate to the standard
        // mapper for primitives and StringFixed.
        _ => field_type_to_rust(ft),
    }
}
