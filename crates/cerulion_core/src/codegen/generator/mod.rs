// SPDX-License-Identifier: AGPL-3.0-only
//! Rust code generator for message schemas.
//!
//! Per schema, codegen emits exactly three things:
//!
//! 1. `pub struct <Name>;` — a unit marker carrying `impl ShmMessage`. User
//!    code references this marker (e.g. `OutputProxy<'_, Twist>`); the proxy
//!    derefs to `<<Name> as ShmMessage>::Writer<'a>` (= `<Name>Shm[<'a>]`).
//! 2. `<Name>Shm[<'a>]` — the SHM-backed reader/writer accessor. Fixed
//!    schemas get `#[repr(C)] struct <Name>Shm` overlaid via
//!    `bytemuck::cast_slice_mut`; variable schemas get an opaque
//!    `<Name>Shm<'a>` wrapping `(payload, WriterState)`.
//! 3. `<Name>Snapshot` — plain-data companion (`Default + Clone + Debug +
//!    PartialEq + serde`) for tests, fixtures, and replay.
//!
//! # Module Structure
//!
//! - `types` — FieldType → Rust type string mapping and primitive introspection
//! - `structs` — `<Name>Shm[<'a>]` and `<Name>Snapshot` emission
//! - `wire_impl` — `impl ShmMessage for <Name>` emission (the marker type)
//!
//! The legacy heap `<Name>` struct, `<Name>View<'a>`, `<Name>Build`/Measurer/
//! Writer codegen, and `impl Message for <Name>` were deleted in an earlier
//! change alongside the `Message` trait itself.

mod structs;
mod types;
mod wire_impl;

// The per-schema variable budget is no longer codegen-private. A
// `ros2 attach` bridge route is created at RUN time from a schema NAME (no
// generated type exists for it), so it cannot read `T::MAX_SLICE_LEN` — but it
// should get the same per-schema budget a graph-declared output of that schema
// gets. `codegen::route_budget` is the one consumer; see its module docs.
pub use wire_impl::{variable_schema_max_slice_len, IN_REPO_PACKAGES};

// The split faces of the tier table for `codegen::slice_ceiling`
// (the RUNTIME lookup): the pure table MATCH, the shared in-repo guard, and
// the catch-all byte count — crate-internal, so the public surface stays the
// two functions above plus `slice_ceiling`'s own.
pub(crate) use wire_impl::{
    in_repo_package, variable_schema_listed_tier, UNLISTED_SCHEMA_FALLBACK_BYTES,
};

use super::schema::{FieldType, MessageSchema};

/// Rust reserved keywords that need to be escaped with r# prefix.
const RUST_KEYWORDS: &[&str] = &[
    "as", "async", "await", "break", "const", "continue", "crate", "dyn", "else", "enum", "extern",
    "false", "fn", "for", "if", "impl", "in", "let", "loop", "match", "mod", "move", "mut", "pub",
    "ref", "return", "self", "Self", "static", "struct", "super", "trait", "true", "type",
    "unsafe", "use", "where", "while",
];

/// Escape a field name if it's a Rust keyword.
fn escape_keyword(name: &str) -> String {
    if RUST_KEYWORDS.contains(&name) {
        format!("r#{}", name)
    } else {
        name.to_string()
    }
}

/// Check if a schema (TRANSITIVELY) contains a large fixed array
/// (>32 elements) — directly in its own fields or inside a resolved-fixed
/// nested type it embeds.
///
/// Gates the `Debug` derive and the serde derives on snapshots. serde's
/// array impls genuinely stop at 32 elements; `Debug` has been all-`N`
/// since Rust 1.47, but the generated types deliberately skip it for
/// large-array schemas (narrow surface, matches the historical heap
/// struct) — and once one type skips a derive, every embedding parent is
/// poisoned transitively.
fn has_large_array(schema: &MessageSchema) -> bool {
    schema
        .fields
        .iter()
        .any(|f| field_has_large_array(&f.field_type))
}

fn field_has_large_array(ft: &FieldType) -> bool {
    match ft {
        FieldType::FixedArray {
            length,
            element_type,
        } => *length > 32 || field_has_large_array(element_type),
        // A resolved-fixed nested field embeds `<Target>Shm`
        // inline; if the target (transitively) contains a large array,
        // the embedded type skipped `Debug` (and its snapshot skipped
        // serde) and poisons the parent's derive set exactly like a
        // direct large array. NOTE: `Default` is NOT poisoned
        // transitively — see [`has_direct_large_array`].
        FieldType::Nested {
            fixed: Some(info), ..
        } => info.has_large_array,
        _ => false,
    }
}

/// Check if a schema DIRECTLY contains a large fixed array (>32 elements)
/// among its own fields — NOT through a resolved-fixed nested type.
///
/// Gates the `Default` strategy: arrays past 32 elements lack the stdlib
/// `Default` blanket, so a direct large array forces a manual `Default`
/// impl. A nested type with a (transitive) large array carries its OWN
/// manual `Default` impl, so the parent can still `#[derive(Default)]` —
/// deriving is preferred (clippy `derivable_impls` rejects a manual impl
/// that derive could produce).
fn has_direct_large_array(schema: &MessageSchema) -> bool {
    schema
        .fields
        .iter()
        .any(|f| field_has_direct_large_array(&f.field_type))
}

fn field_has_direct_large_array(ft: &FieldType) -> bool {
    match ft {
        FieldType::FixedArray {
            length,
            element_type,
        } => *length > 32 || field_has_direct_large_array(element_type),
        _ => false,
    }
}

/// True when any field carries a non-zero scalar default —
/// the generated `Default` impls must then be manual (derive would
/// produce zeros). Zero-valued defaults (`float64 x 0`) keep the derive:
/// a manual impl identical to the derive trips clippy `derivable_impls`.
fn has_nonzero_defaults(schema: &MessageSchema) -> bool {
    schema
        .fields
        .iter()
        .any(|f| f.default_literal.is_some_and(|d| !d.is_zero()))
}

/// True when the generated `Default` for this schema's data types must be
/// a MANUAL impl: a direct `[T; N>32]` (no stdlib `Default` blanket past
/// 32) or a non-zero scalar field default (derive would produce zeros).
fn needs_manual_default(schema: &MessageSchema) -> bool {
    has_direct_large_array(schema) || has_nonzero_defaults(schema)
}

/// Generate Rust code for a message schema.
///
/// Per schema, emits:
/// 1. Schema constants (`<NAME>_SCHEMA_HASH`, `<NAME>_WIRE_SIZE` for fixed).
/// 2. `pub struct <Name>;` — unit marker carrying `impl ShmMessage`.
/// 3. `<Name>Shm[<'a>]` — the SHM-backed accessor type.
/// 4. `<Name>Snapshot` — plain-data companion.
/// 5. `impl ShmMessage for <Name>` — wires the marker to `<Name>Shm`.
///
/// Note: This generates just the type definitions, not module-level items
/// like `use` statements. The caller should add those to the file header.
pub fn generate_schema(schema: &MessageSchema) -> String {
    let mut out = String::new();

    // Schema-blind macro surgery: reject any user field name using the
    // reserved `__cer` prefix BEFORE emitting any types. Codegen owns every
    // `__cer_assign_<field>` / `__cer_fill_from_<field>` shim +
    // `__cer_wp_*`/`__cer_wg_*` proxy name, so a user field starting with
    // `__cer` could shadow generated plumbing. Emits a `compile_error!` (no-op when clean) and continues —
    // the compile_error fires first; any cascading rustc noise is harmless.
    structs::emit_reserved_prefix_compile_error(&mut out, schema);

    // Recipe 3: `schema_hash` is a computed METHOD (layout + qualified name
    // + recursive nested fold), not a stored field, so there is nothing to
    // drift — `generate_schema_constants` calls `schema.schema_hash()`
    // directly. (The #55 stored-field tripwire was dropped on the recipe
    // unification — a layout-sensitive hash cannot be a name-only `const fn`.)
    generate_schema_constants(&mut out, schema);

    // Emit the unit marker `pub struct <Name>;` that carries `impl ShmMessage`.
    // User code references this name in `OutputProxy<'_, Twist>` etc.; the
    // SHM-backed accessor type is `<Twist as ShmMessage>::Writer<'a>` =
    // `TwistShm[<'a>]`.
    structs::generate_marker_struct(&mut out, schema);

    if schema.is_definitely_fixed() {
        // Fixed schemas: emit `<Name>Shm` (`#[repr(C)]` overlay) + Snapshot.
        structs::generate_shm_struct(&mut out, schema);
        structs::generate_snapshot_struct(&mut out, schema);
        wire_impl::generate_shm_message_trait_impl(&mut out, schema);
    } else {
        // Variable schemas: emit offset-table-backed `<Name>Shm<'a>` with
        // typed accessors plus the Snapshot companion.
        structs::generate_variable_shm_struct(&mut out, schema);
        structs::generate_variable_snapshot_struct(&mut out, schema);
        wire_impl::generate_variable_shm_message_trait_impl(&mut out, schema);
    }

    // The built-in `builtin_interfaces/Time` stamp helper
    // (`Time::from_ns`). Schema-identity gated (package + name), so it is a
    // no-op for every other schema — call it unconditionally, after the marker
    // + `<Name>Shm` are emitted above (it references both).
    structs::generate_builtin_time_helper(&mut out, schema);

    out
}

fn generate_schema_constants(out: &mut String, schema: &MessageSchema) {
    use std::fmt::Write;

    writeln!(out, "/// Schema hash for {}", schema.name).unwrap();
    writeln!(
        out,
        "pub const {}_SCHEMA_HASH: u64 = 0x{:016X};",
        schema.name.to_uppercase(),
        schema.schema_hash()
    )
    .unwrap();

    // For fixed-size messages, add WIRE_SIZE constant.
    // This used to read `size_of::<{Name}>` —
    // but `{Name}` is the ZERO-SIZED unit marker, so the
    // constant was 0 for every fixed schema. The SHM overlay `{Name}Shm`
    // carries the actual wire layout.
    if schema.is_definitely_fixed() {
        writeln!(out).unwrap();
        writeln!(
            out,
            "/// Wire size in bytes for {} (fixed-size message).",
            schema.name
        )
        .unwrap();
        writeln!(
            out,
            "pub const {}_WIRE_SIZE: usize = size_of::<{}Shm>();",
            schema.name.to_uppercase(),
            schema.name
        )
        .unwrap();
    }
    writeln!(out).unwrap();
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::codegen::schema::{FieldDef, FieldType};

    /// Every schema emits a unit marker `pub struct <Name>;`
    /// plus `impl ShmMessage for <Name>` wiring the GATs to `<Name>Shm[<'a>]`.
    /// Verify the marker / SHM / Snapshot triad for a fixed schema.
    #[test]
    fn test_generate_fixed_marker_shm_snapshot() {
        let mut schema = MessageSchema::new("Point");
        schema.add_field(FieldDef::new("x", FieldType::F32));
        schema.add_field(FieldDef::new("y", FieldType::F32));
        schema.add_field(FieldDef::new("z", FieldType::F32));

        let code = generate_schema(&schema);

        // Unit marker (no fields, no methods of its own).
        assert!(code.contains("pub struct Point;"));
        // SHM-backed accessor with the actual fields.
        assert!(code.contains("pub struct PointShm {"));
        // Plain-data snapshot companion.
        assert!(code.contains("pub struct PointSnapshot {"));
        // ShmMessage impl is on the marker, GATs point at `PointShm`.
        assert!(code.contains("impl cerulion_core::message::ShmMessage for Point {"));
        assert!(code.contains("type Reader<'a> = &'a PointShm;"));
        assert!(code.contains("type Writer<'a> = &'a mut PointShm;"));
        assert!(code.contains("POINT_SCHEMA_HASH"));
        // Legacy types are gone.
        assert!(!code.contains("pub type PointView"));
        assert!(!code.contains("PointBuilder"));
        assert!(!code.contains("PointMeasurer"));
        assert!(!code.contains("PointWriter"));
        assert!(!code.contains("impl cerulion_core::message::Message for"));
        assert!(!code.contains("impl cerulion_core::message::FixedMessage for"));
    }

    /// Same triad for a variable schema: marker + `<Name>Shm<'a>` + Snapshot.
    #[test]
    fn test_generate_variable_marker_shm_snapshot() {
        let mut schema = MessageSchema::new("Image");
        schema.add_field(FieldDef::new("width", FieldType::U32));
        schema.add_field(FieldDef::new("height", FieldType::U32));
        schema.add_field(FieldDef::new("encoding", FieldType::String));
        schema.add_field(FieldDef::new("data", FieldType::Bytes));

        let code = generate_schema(&schema);

        // Marker: zero-size, no fields.
        assert!(code.contains("pub struct Image;"));
        // SHM-backed accessor carries a lifetime for variable schemas.
        assert!(code.contains("pub struct ImageShm<'a> {"));
        // Plain-data snapshot companion.
        assert!(code.contains("pub struct ImageSnapshot {"));
        // ShmMessage impl is on the marker, GATs point at `ImageShm<'a>`.
        assert!(code.contains("impl cerulion_core::message::ShmMessage for Image {"));
        assert!(code.contains("type Reader<'a> = ImageShm<'a>;"));
        assert!(code.contains("type Writer<'a> = ImageShm<'a>;"));
        // Legacy types are gone.
        assert!(!code.contains("pub struct ImageView"));
        assert!(!code.contains("ImageBuilder"));
        assert!(!code.contains("ImageMeasurer"));
        assert!(!code.contains("ImageWriter"));
        assert!(!code.contains("impl cerulion_core::message::Message for"));
    }
}
