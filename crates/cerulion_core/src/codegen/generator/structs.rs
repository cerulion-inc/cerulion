// SPDX-License-Identifier: AGPL-3.0-only
//! Struct generation for message schemas.
//!
//! Per schema this module emits:
//!
//! - `pub struct <Name>;` — unit marker carrying `impl ShmMessage` (in
//!   `wire_impl.rs`). User code references this name.
//! - `<Name>Shm[<'a>]` — the SHM-backed accessor type. `#[repr(C)]` overlay
//!   for fixed schemas, opaque offset-table-backed wrapper for variable
//!   schemas.
//! - `<Name>Snapshot` — plain-data companion (`Default + Clone + Debug +
//!   PartialEq + serde`) used by tests, fixtures, and replay.
//!
//! The legacy heap-owned `<Name>` struct + `<Name>View<'a>` + Build trait /
//! Measurer / Writer + `impl Message for <Name>` codegen was deleted
//! alongside the `Message` trait itself.

use super::types::{field_type_to_rust, field_type_to_rust_shm};
use super::{escape_keyword, has_large_array, needs_manual_default};
use crate::codegen::schema::{FieldDef, FieldType, MessageSchema};
use std::fmt::Write;

/// Emit the unit marker `pub struct <Name>;` that carries `impl ShmMessage`.
///
/// The marker is zero-sized and has no inherent methods. User code references
/// it (e.g. `OutputProxy<'_, Twist>`); the actual SHM-backed accessor type is
/// `<<Name> as ShmMessage>::Writer<'a>` (= `<Name>Shm[<'a>]`). The proxy
/// derefs to that accessor via the GAT, so `proxy.field = value` reaches the
/// SHM overlay without exposing the marker to user code beyond the type-level.
pub(super) fn generate_marker_struct(out: &mut String, schema: &MessageSchema) {
    if let Some(desc) = &schema.description {
        writeln!(out, "/// {desc}").unwrap();
    }
    writeln!(out, "///").unwrap();
    writeln!(
        out,
        "/// Schema marker for `{}`. Zero-sized. Carries the",
        schema.name
    )
    .unwrap();
    writeln!(
        out,
        "/// `cerulion_core::message::ShmMessage` impl whose GATs point at"
    )
    .unwrap();
    writeln!(
        out,
        "/// [`{}Shm`] (the SHM-backed accessor) and `{}Snapshot`.",
        schema.name, schema.name
    )
    .unwrap();

    // Emit a fields summary so users reading `<Name>`'s rustdoc see the
    // schema's fixed and variable fields directly, without chasing the Deref
    // chain (`<Name>Shm` → `<Name>FixedSection`) by hand.
    //
    // For variable fields, the summary uses the EXACT accessor return type
    // (e.g. `Result<&str, WireError>` for `String`) and the EXACT typed slice
    // (`&[bool]` for `DynamicArray<Bool>`, since `primitive_size_and_type`
    // maps `Bool → "bool"` in the typed-array codegen). Keyword field names
    // are rendered with the same `r#`-prefix the actual emitted struct field
    // / accessor uses, so doc text matches user-typed code.
    let mut fixed_lines: Vec<String> = Vec::new();
    let mut variable_lines: Vec<String> = Vec::new();
    let mut has_bool_in_fixed_section = false;
    for field in &schema.fields {
        let fname = escape_keyword(&field.name);
        if is_fixed_section_field(&field.field_type) {
            // Direct-access via Deref to <Name>FixedSection. `bool` fields
            // are stored as `u8` in the overlay (the overlay contract).
            if matches!(&field.field_type, FieldType::Bool)
                || matches!(
                    &field.field_type,
                    FieldType::FixedArray { element_type, .. } if matches!(element_type.as_ref(), FieldType::Bool)
                )
            {
                has_bool_in_fixed_section = true;
            }
            let ty = field_type_to_rust_shm(&field.field_type);
            fixed_lines.push(format!("///   - `{fname}: {ty}` (direct field access)"));
        } else if is_complex_variable(&field.field_type) {
            // Raw-bytes accessors only (Nested + FixedArray<Nested> +
            // DynamicArray<non-primitive>).
            variable_lines.push(format!(
                "///   - `{fname}` — raw bytes: `{fname}_bytes() -> &[u8]` / `set_{fname}_bytes(&[u8]) -> Result<(), TransportError>` / `loan_{fname}_bytes(n) -> Result<&mut [u8], TransportError>` / `fill_from_{fname}_bytes(impl FillFrom) -> Result<(), TransportError>`",
            ));
        } else {
            // Typed variable accessors. The reader signature varies by type
            // (`String` returns `Result<&str, WireError>`; everything else
            // returns the typed slice directly).
            match &field.field_type {
                FieldType::String => {
                    variable_lines.push(format!(
                        "///   - `{fname}: String` — `{fname}() -> Result<&str, WireError>` / `set_{fname}(&str) -> Result<(), TransportError>` / `loan_{fname}(n) -> Result<&mut [u8], TransportError>` / `fill_from_{fname}(impl FillFrom) -> Result<(), TransportError>` (producer responsible for UTF-8)",
                    ));
                }
                FieldType::Bytes | FieldType::DynamicArray { .. } => {
                    let elem_ty = match &field.field_type {
                        FieldType::Bytes => "u8".to_string(),
                        FieldType::DynamicArray { element_type } => match element_type.as_ref() {
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
                            // Non-primitive element types are routed through
                            // is_complex_variable above; this arm is unreachable.
                            _ => "/*element*/".to_string(),
                        },
                        _ => unreachable!(),
                    };
                    variable_lines.push(format!(
                        "///   - `{fname}: &[{elem_ty}]` — `{fname}() -> &[{elem_ty}]` / `set_{fname}(&[{elem_ty}]) -> Result<(), TransportError>` / `loan_{fname}(n) -> Result<&mut [{elem_ty}], TransportError>` / `push_{fname}({elem_ty}) -> Result<(), TransportError>` / `fill_from_{fname}(impl FillFrom<{elem_ty}>) -> Result<(), TransportError>`",
                    ));
                }
                _ => {
                    // is_complex_variable above caught Nested + FixedArray-of-
                    // non-primitive. Remaining shapes (StringFixed,
                    // FixedArray<primitive>) are fixed-section, not variable —
                    // unreachable here.
                    variable_lines.push(format!(
                        "///   - `{fname}` — variable (see `<Name>Shm` accessors)",
                    ));
                }
            }
        }
    }
    if !fixed_lines.is_empty() || !variable_lines.is_empty() {
        writeln!(out, "///").unwrap();
        writeln!(out, "/// # Fields").unwrap();
        writeln!(out, "///").unwrap();
        if !fixed_lines.is_empty() {
            writeln!(out, "/// **Fixed (direct field access via `Deref`):**").unwrap();
            for line in &fixed_lines {
                writeln!(out, "{line}").unwrap();
            }
            if has_bool_in_fixed_section {
                writeln!(out, "///").unwrap();
                writeln!(
                    out,
                    "/// `bool` fields are stored as `u8` in the SHM overlay (canonical values"
                )
                .unwrap();
                writeln!(
                    out,
                    "/// `0` / `1`); the snapshot companion exposes real `bool`. Writes of"
                )
                .unwrap();
                writeln!(
                    out,
                    "/// non-canonical `u8` values do NOT round-trip bit-identically through the"
                )
                .unwrap();
                // Where the contract lives in rustdoc depends on whether
                // this schema is fixed-only or variable. For variable
                // schemas the FixedSection overlay carries the doc; for
                // fixed-only schemas it lives on `<Name>Shm` itself.
                // Reference whichever exists so rustdoc's intra-doc-link
                // resolves (broken-intra-doc-links is an error in CI).
                if schema.has_variable_fields() {
                    writeln!(
                        out,
                        "/// snapshot — see [`{}FixedSection`] for the full contract.",
                        schema.name
                    )
                    .unwrap();
                } else {
                    writeln!(
                        out,
                        "/// snapshot — see [`{}Shm`] for the full contract.",
                        schema.name
                    )
                    .unwrap();
                }
            }
            if !variable_lines.is_empty() {
                writeln!(out, "///").unwrap();
            }
        }
        if !variable_lines.is_empty() {
            writeln!(out, "/// **Variable (accessor methods):**").unwrap();
            for line in &variable_lines {
                writeln!(out, "{line}").unwrap();
            }
        }
    }

    // `Default` lets users put the marker into a `#[derive(Default)] struct`
    // (the canonical pattern for `#[cerulion_node(... zero_copy)]` user
    // structs that hold port markers as fields).
    writeln!(out, "#[derive(Debug, Clone, Copy, Default)]").unwrap();
    writeln!(out, "pub struct {};", schema.name).unwrap();
    writeln!(out).unwrap();
}

/// Get a default value expression for a field type.
///
/// Used by `<Name>Snapshot` codegen below. Mirrors the heap struct's
/// historical defaults so snapshot deserialisation stays compatible across
/// the deletion of that struct.
pub(super) fn field_type_default_value(ft: &FieldType) -> String {
    match ft {
        FieldType::Bool => "false".to_string(),
        FieldType::I8 | FieldType::I16 | FieldType::I32 | FieldType::I64 => "0".to_string(),
        FieldType::U8 | FieldType::U16 | FieldType::U32 | FieldType::U64 => "0".to_string(),
        FieldType::F32 => "0.0".to_string(),
        FieldType::F64 => "0.0".to_string(),
        FieldType::String => "::std::string::String::new()".to_string(),
        FieldType::Bytes => "::std::vec::Vec::new()".to_string(),
        FieldType::StringFixed(n) => format!("[0u8; {}]", n),
        FieldType::FixedArray {
            element_type,
            length,
        } => {
            // Array-repeat needs `Copy`; Snapshot element types are
            // Clone-only — route those through `array::from_fn`.
            if let Some(expr) = snapshot_array_element_default(element_type, *length) {
                return expr;
            }
            let elem_default = field_type_default_value(element_type);
            format!("[{}; {}]", elem_default, length)
        }
        FieldType::DynamicArray { .. } => "::std::vec::Vec::new()".to_string(),
        FieldType::Nested { schema_name, .. } => {
            // Nested schemas now compose via `<NameSnapshot>::default()` (the
            // marker `<Name>` is unit and has no Default — that's intentional,
            // since users are not meant to construct a marker). Snapshot
            // composition keeps test fixtures buildable on the heap.
            format!("{}Snapshot::default()", schema_name)
        }
    }
}

/// Like [`field_type_default_value`] but safe inside array-repeat
/// position: `[expr; N]` requires `Copy` (or a const), and Snapshot types
/// are `Clone`-not-`Copy` — so `FixedArray<Nested>` must construct via
/// `array::from_fn` instead.
fn snapshot_array_element_default(ft: &FieldType, length: usize) -> Option<String> {
    if let FieldType::Nested { schema_name, .. } = ft {
        return Some(format!(
            "::std::array::from_fn::<{schema_name}Snapshot, {length}, _>(|_| {schema_name}Snapshot::default())"
        ));
    }
    None
}

// =============================================================================
// SHM-backed type emission
// =============================================================================
//
// For every fixed schema (`is_definitely_fixed() == true`) we emit two
// additional types alongside the existing heap struct:
//
//   1. `<Name>Shm` — `#[repr(C)]` struct with the same field order as
//      the heap struct. Bool fields become `u8` so the type is safely
//      transmutable from arbitrary bytes (bool's bit pattern is restricted
//      to `0` and `1`; reading any other byte as bool is UB). Constructed
//      via `<Name>Shm::from_bytes(&[u8])` / `from_bytes_mut(&mut [u8])`,
//      both of which return a borrow into the SHM slot.
//
//   2. `<Name>Snapshot` — plain-data companion with `Default, Clone, Debug,
//      PartialEq, serde::Serialize, serde::Deserialize`. Bool fields stay
//      as `bool`. Used by tests and replay infrastructure that need to
//      snapshot a SHM frame to the heap and back.
//
// `<Name>Shm::snapshot(&self) -> <Name>Snapshot` and
// `<Name>Shm::write_from_snapshot(&mut self, &<Name>Snapshot)` round-trip
// between the two types, doing the bool ↔ u8 conversion field-by-field.
//
// A compile-time layout assertion ensures
// `mem::size_of::<<Name>Shm>() == size_of::<<Name>>()` to catch field-order
// or padding drift between the legacy heap struct and the new SHM struct.
//
// Variable schemas get their counterparts from
// `generate_variable_shm_struct` below. `<Name>` itself is the unit marker:
// the `impl ShmMessage` (in `wire_impl.rs`) is implemented on it, so user
// code writes `OutputProxy<'_, Twist>` while the SHM-backed accessor type
// stays `<Name>Shm`.

/// Generate the SHM-backed `<Name>Shm` struct for a fixed schema.
///
/// Caller must verify `schema.is_definitely_fixed() == true`. For variable
/// schemas this function does nothing useful (`generate_variable_shm_struct`
/// handles those).
pub(super) fn generate_shm_struct(out: &mut String, schema: &MessageSchema) {
    let name = &schema.name;
    let shm_name = format!("{}Shm", name);

    writeln!(out, "/// SHM-backed view of [`{name}`].").unwrap();
    writeln!(out, "///").unwrap();
    writeln!(
        out,
        "/// `#[repr(C)]` mirror of the heap-owned [`{name}`] struct, intended to be"
    )
    .unwrap();
    writeln!(
        out,
        "/// constructed directly over a borrowed iceoryx2 sample payload via"
    )
    .unwrap();
    writeln!(
        out,
        "/// [`from_bytes`](Self::from_bytes) / [`from_bytes_mut`](Self::from_bytes_mut)."
    )
    .unwrap();
    writeln!(out, "///").unwrap();
    writeln!(
        out,
        "/// `bool` fields are stored as `u8` because `bool`'s valid bit patterns are"
    )
    .unwrap();
    writeln!(
        out,
        "/// restricted to `0` and `1` — interpreting an arbitrary byte as `bool` is"
    )
    .unwrap();
    writeln!(
        out,
        "/// undefined behavior. The companion [`{name}Snapshot`] uses real `bool`"
    )
    .unwrap();
    writeln!(out, "/// fields and the [`snapshot`](Self::snapshot) /").unwrap();
    writeln!(
        out,
        "/// [`write_from_snapshot`](Self::write_from_snapshot) helpers handle the conversion."
    )
    .unwrap();
    writeln!(out, "#[repr(C)]").unwrap();

    // Derive strategy. Manual `Default` is forced by a DIRECT
    // large array (>32 elements lacks the stdlib `Default` blanket) or a
    // non-zero scalar field default (`float64 w 1` — derive would produce
    // zeros). `Debug` is poisoned by a large array anywhere (including
    // inside an embedded resolved-fixed nested type, which skipped its own
    // `Debug`). When neither forces a manual impl, deriving is REQUIRED
    // (clippy `derivable_impls`).
    match (needs_manual_default(schema), has_large_array(schema)) {
        (true, true) => writeln!(out, "#[derive(Clone, Copy, PartialEq)]").unwrap(),
        (true, false) => writeln!(out, "#[derive(Debug, Clone, Copy, PartialEq)]").unwrap(),
        (false, true) => writeln!(out, "#[derive(Clone, Copy, PartialEq, Default)]").unwrap(),
        (false, false) => {
            writeln!(out, "#[derive(Debug, Clone, Copy, PartialEq, Default)]").unwrap()
        }
    }
    writeln!(out, "pub struct {shm_name} {{").unwrap();

    for field in &schema.fields {
        if let Some(desc) = &field.description {
            writeln!(out, "    /// {desc}").unwrap();
        }
        let rust_type = field_type_to_rust_shm(&field.field_type);
        let field_name = escape_keyword(&field.name);
        writeln!(out, "    pub {field_name}: {rust_type},").unwrap();
    }

    writeln!(out, "}}").unwrap();
    writeln!(out).unwrap();

    // Manual Default impl when forced (direct large array or non-zero
    // scalar defaults). Transitive-only large arrays derive Default above.
    if needs_manual_default(schema) {
        generate_shm_default_impl(out, schema, &shm_name);
    }

    // Inherent methods: from_bytes, from_bytes_mut, snapshot, write_from_snapshot.
    writeln!(out, "impl {shm_name} {{").unwrap();
    generate_shm_from_bytes(out, &shm_name);
    generate_shm_from_bytes_mut(out, &shm_name);
    generate_shm_snapshot_method(out, schema, &shm_name);
    generate_shm_write_from_snapshot_method(out, schema, &shm_name);
    // Uniform per-field `__cer_assign_<f>` write shims. Every field of
    // a fully-fixed schema lives inline in this `#[repr(C)]` overlay and is
    // written today by direct assignment (`self.<f> = value`), so the shim
    // just re-expresses that assignment behind a name the schema-blind macro
    // rewriter can call uniformly for fixed AND variable
    // fields without knowing which is which.
    for field in &schema.fields {
        emit_fixed_assign_shim(out, field);
    }
    // Leaf-composing accessors for inline fixed-nested
    // substructs (`self.<port>.<nested>.<leaf> = …` sugar + `with_<f>`).
    for field in &schema.fields {
        if matches!(&field.field_type, FieldType::Nested { fixed: Some(_), .. }) {
            emit_fixed_nested_accessors(out, field);
        }
    }
    // The explicit `emit()` publish gesture for a ZERO-FIELD
    // output schema (e.g. `std_msgs/Empty`). Gated on `schema.fields.is_empty()`
    // so it exists ONLY for fieldless schemas — calling `emit()` on any output
    // that HAS a field is a compile error (no such method). See
    // `emit_zero_field_publish_gesture` for why a fieldless output needs it.
    if schema.fields.is_empty() {
        emit_zero_field_publish_gesture(out);
    }
    writeln!(out, "}}").unwrap();
    writeln!(out).unwrap();

    // The legacy heap-owned `<Name>` struct is gone — the
    // size-equality assertion that used to live here can't be expressed
    // anymore (the marker `<Name>` is zero-sized). Layout invariants for
    // `{shm_name}` are enforced by the `#[repr(C)]` annotation, the bool→u8
    // substitution in `field_type_to_rust_shm`, and the snapshot ↔ shm
    // round-trip in tests.
    let _ = name;
}

/// Emit the built-in `builtin_interfaces/Time::from_ns` stamp helper.
///
/// Stamping a `std_msgs/Header` is the single most common thing a robotics
/// node does, and every hand-rolled `ns → (sec, nanosec)` split is a fresh
/// chance to get the `nanosec` remainder or the `i32`-second 2038 horizon
/// wrong. This emits ONE canonical converter as an inherent associated
/// function on the `Time` marker, gated on the EXACT `builtin_interfaces/Time`
/// schema identity (package + name) so no other package's message named
/// `Time`, `builtin_interfaces/Duration`, or a package-less workspace `Time`
/// schema picks it up.
///
/// The helper returns the SHM overlay value (`<Name>Shm`), NOT the
/// zero-sized `Time` marker — a marker cannot carry `sec`/`nanosec`. Returning
/// the overlay is what makes the headline one-liner type-check: the generated
/// whole-field write shim (`__cer_assign_stamp(&TimeShm)`, reached through the
/// staged-`Header` `DerefMut` chain) consumes exactly `&TimeShm`, so
///
/// ```text
/// self.out.header.stamp = Time::from_ns(self.now_ns());
/// ```
///
/// (a fragment of a `tick` body; the emitted doc on `Time::from_ns` carries
/// the whole node, which compiles and runs as a doctest of
/// `native_ros2_messages`)
///
/// rewrites to that shim with no intermediate struct. The overlay also exposes
/// `pub sec: i32` / `pub nanosec: u32`, so a caller that needs the split (e.g.
/// feeding a codec) reads `Time::from_ns(ns).sec` directly.
///
/// Must be called AFTER the marker + `<Name>Shm` are emitted (it references
/// both); it is a no-op for every non-`Time` schema, so `generate_schema` can
/// call it unconditionally.
pub(super) fn generate_builtin_time_helper(out: &mut String, schema: &MessageSchema) {
    // Schema-identity gate: ONLY the vendored `builtin_interfaces/Time`.
    if schema.package.as_deref() != Some("builtin_interfaces") || schema.name != "Time" {
        return;
    }

    let name = &schema.name; // "Time"
    let shm_name = format!("{name}Shm");

    writeln!(out, "impl {name} {{").unwrap();
    writeln!(
        out,
        "    /// Split a nanosecond timestamp into a ROS `builtin_interfaces/Time`"
    )
    .unwrap();
    writeln!(
        out,
        "    /// (`sec`, `nanosec`) — the one canonical stamp converter."
    )
    .unwrap();
    writeln!(out, "    ///").unwrap();
    writeln!(
        out,
        "    /// The common case is one line in a node `tick` (the returned"
    )
    .unwrap();
    writeln!(
        out,
        "    /// [`{shm_name}`] overlay is what the generated `header.stamp` write"
    )
    .unwrap();
    writeln!(
        out,
        "    /// consumes, so no intermediate struct is needed):"
    )
    .unwrap();
    writeln!(out, "    ///").unwrap();
    for line in [
        "```",
        "use cerulion_core::prelude::*;",
        "use native_ros2_messages::builtin_interfaces::Time;",
        "use native_ros2_messages::geometry_msgs::PoseStamped;",
        "",
        "#[cerulion_node(period_ms = 100)]",
        "#[derive(Default)]",
        "struct StampedPoseNode {",
        "    #[output]",
        "    pose: PoseStamped,",
        "}",
        "",
        "#[cerulion_node_impl]",
        "impl StampedPoseNode {",
        "    fn tick(&mut self) -> Result<(), NodeError> {",
        "        self.pose.header.stamp = Time::from_ns(self.now_ns());",
        "        self.pose.header.frame_id = \"map\";",
        "        self.pose.pose.position.x = 1.0;",
        "        Ok(())",
        "    }",
        "}",
        "# fn main() {}",
        "```",
    ] {
        if line.is_empty() {
            writeln!(out, "    ///").unwrap();
        } else {
            writeln!(out, "    /// {line}").unwrap();
        }
    }
    writeln!(out, "    ///").unwrap();
    writeln!(out, "    /// # Determinism").unwrap();
    writeln!(out, "    ///").unwrap();
    writeln!(
        out,
        "    /// Feed it the NODE clock (`self.now_ns()`), NEVER a wall clock: the node"
    )
    .unwrap();
    writeln!(
        out,
        "    /// clock is deterministic, so a recorded frame replays the exact stamp"
    )
    .unwrap();
    writeln!(
        out,
        "    /// (Principle #7). The determinism lint guards the read side (it bans"
    )
    .unwrap();
    writeln!(
        out,
        "    /// `SystemTime::now()` / `Instant::now()` inside a `tick`)."
    )
    .unwrap();
    writeln!(out, "    ///").unwrap();
    writeln!(out, "    /// # 2038 horizon").unwrap();
    writeln!(out, "    ///").unwrap();
    writeln!(
        out,
        "    /// `sec` is `i32` per the ROS schema (~68 years of ns measured from 0). A"
    )
    .unwrap();
    writeln!(
        out,
        "    /// `debug_assert!` fires LOUDLY in debug/test builds if the second count"
    )
    .unwrap();
    writeln!(
        out,
        "    /// exceeds `i32::MAX`; release builds SATURATE `sec` to `i32::MAX` (a"
    )
    .unwrap();
    writeln!(
        out,
        "    /// defined, deterministic value — never a two's-complement wrap to a"
    )
    .unwrap();
    writeln!(out, "    /// negative time).").unwrap();
    writeln!(out, "    #[inline]").unwrap();
    writeln!(out, "    #[must_use]").unwrap();
    writeln!(out, "    pub fn from_ns(ns: u64) -> {shm_name} {{").unwrap();
    writeln!(out, "        const NS_PER_SEC: u64 = 1_000_000_000;").unwrap();
    writeln!(out, "        let secs = ns / NS_PER_SEC;").unwrap();
    writeln!(out, "        debug_assert!(").unwrap();
    writeln!(out, "            secs <= i32::MAX as u64,").unwrap();
    writeln!(
        out,
        "            \"builtin_interfaces/Time::from_ns: {{ns}} ns exceeds i32::MAX seconds (the ROS 2038 i32 horizon)\""
    )
    .unwrap();
    writeln!(out, "        );").unwrap();
    writeln!(out, "        {shm_name} {{").unwrap();
    // Saturating cast: `secs` is clamped to `i32::MAX` (fits `i32` losslessly),
    // so the `as i32` can never two's-complement-wrap to a negative second.
    writeln!(out, "            sec: secs.min(i32::MAX as u64) as i32,").unwrap();
    writeln!(out, "            nanosec: (ns % NS_PER_SEC) as u32,").unwrap();
    writeln!(out, "        }}").unwrap();
    writeln!(out, "    }}").unwrap();
    writeln!(out, "}}").unwrap();
    writeln!(out).unwrap();
}

/// Manual `Default` for SHM-backed structs that cannot derive it — a
/// direct `[T; N>32]` (no stdlib blanket) or a non-zero scalar field
/// default. Generates one `<field>: <expr>` line per field
/// where `<expr>` is the bool-aware, default-literal-aware value.
fn generate_shm_default_impl(out: &mut String, schema: &MessageSchema, shm_name: &str) {
    writeln!(out, "impl Default for {shm_name} {{").unwrap();
    writeln!(out, "    fn default() -> Self {{").unwrap();
    writeln!(out, "        Self {{").unwrap();
    for field in &schema.fields {
        let field_name = escape_keyword(&field.name);
        let default_value = shm_field_default_expr(field);
        writeln!(out, "            {field_name}: {default_value},").unwrap();
    }
    writeln!(out, "        }}").unwrap();
    writeln!(out, "    }}").unwrap();
    writeln!(out, "}}").unwrap();
    writeln!(out).unwrap();
}

/// Default-value expression for an SHM-backed field, honoring a non-zero
/// scalar .msg default when present (`float64 w 1` →
/// `QuaternionShm::default().w == 1.0`).
fn shm_field_default_expr(field: &FieldDef) -> String {
    if let Some(d) = &field.default_literal {
        if !d.is_zero() {
            return d.shm_expr();
        }
    }
    shm_field_default_value(&field.field_type)
}

/// Snapshot default-value expression for a FIXED schema's field, honoring
/// a non-zero scalar .msg default when present.
fn fixed_snapshot_field_default_expr(field: &FieldDef) -> String {
    if let Some(d) = &field.default_literal {
        if !d.is_zero() {
            return d.snapshot_expr();
        }
    }
    field_type_default_value(&field.field_type)
}

/// Snapshot default-value expression for a VARIABLE schema's field,
/// honoring a non-zero scalar .msg default when present.
fn var_snapshot_field_default_expr(field: &FieldDef) -> String {
    if let Some(d) = &field.default_literal {
        if !d.is_zero() {
            return d.snapshot_expr();
        }
    }
    snapshot_field_default_value(&field.field_type)
}

/// Default-value expression for an SHM-backed field. Like
/// [`field_type_default_value`] but bool defaults to `0u8` so the value
/// matches the SHM type rather than the snapshot type, and Nested defaults
/// to `<NestedName>Shm::default()` so the SHM struct composes through SHM
/// (not through the marker, which is unit and zero-sized).
fn shm_field_default_value(ft: &FieldType) -> String {
    match ft {
        FieldType::Bool => "0u8".to_string(),
        FieldType::Nested { schema_name, .. } => format!("{schema_name}Shm::default()"),
        FieldType::FixedArray {
            element_type,
            length,
        } => {
            let elem_default = shm_field_default_value(element_type);
            format!("[{elem_default}; {length}]")
        }
        // Other primitives, StringFixed → identical to heap defaults.
        // Variable variants cannot reach here (gated on is_definitely_fixed).
        _ => field_type_default_value(ft),
    }
}

/// Emit `pub fn from_bytes(bytes: &[u8]) -> &Self`.
///
/// Borrows the SHM slot as `&Self`. Panics on an undersized buffer or a
/// misaligned pointer (analogous to `bytemuck::from_bytes`). Bytes beyond
/// `size_of::<Self>()` are ignored — the caller is responsible for providing
/// a properly bounded slice (the proxy/view types pass exactly the payload
/// region of the iceoryx2 sample).
fn generate_shm_from_bytes(out: &mut String, shm_name: &str) {
    writeln!(
        out,
        "    /// Borrow `bytes` as `&{shm_name}` for zero-copy field reads."
    )
    .unwrap();
    writeln!(out, "    ///").unwrap();
    writeln!(out, "    /// # Panics").unwrap();
    writeln!(out, "    ///").unwrap();
    writeln!(
        out,
        "    /// - If `bytes.len() < ::std::mem::size_of::<Self>()`."
    )
    .unwrap();
    writeln!(
        out,
        "    /// - If `bytes.as_ptr()` is not aligned to `::std::mem::align_of::<Self>()`."
    )
    .unwrap();
    writeln!(out, "    #[inline]").unwrap();
    writeln!(out, "    pub fn from_bytes(bytes: &[u8]) -> &Self {{").unwrap();
    writeln!(out, "        let size = ::std::mem::size_of::<Self>();").unwrap();
    writeln!(out, "        let align = ::std::mem::align_of::<Self>();").unwrap();
    writeln!(
        out,
        "        assert!(bytes.len() >= size, \"{shm_name}::from_bytes: buffer too small ({{}} < {{}})\", bytes.len(), size);"
    )
    .unwrap();
    writeln!(
        out,
        "        assert!(bytes.as_ptr().align_offset(align) == 0, \"{shm_name}::from_bytes: misaligned source pointer (need align {{}})\", align);"
    )
    .unwrap();
    writeln!(
        out,
        "        // SAFETY: the asserts above guarantee `bytes` covers `size_of::<Self>()`"
    )
    .unwrap();
    writeln!(
        out,
        "        // bytes at the required alignment, and `Self` is `#[repr(C)]` with no"
    )
    .unwrap();
    writeln!(
        out,
        "        // niche-bearing fields (bool stored as u8). The borrow lifetime is"
    )
    .unwrap();
    writeln!(out, "        // inherited from `bytes`.").unwrap();
    writeln!(
        out,
        "        unsafe {{ &*(bytes.as_ptr() as *const Self) }}"
    )
    .unwrap();
    writeln!(out, "    }}").unwrap();
    writeln!(out).unwrap();
}

/// Emit `pub fn from_bytes_mut(bytes: &mut [u8]) -> &mut Self`.
fn generate_shm_from_bytes_mut(out: &mut String, shm_name: &str) {
    writeln!(
        out,
        "    /// Borrow `bytes` as `&mut {shm_name}` for direct in-place writes."
    )
    .unwrap();
    writeln!(out, "    ///").unwrap();
    writeln!(out, "    /// # Panics").unwrap();
    writeln!(out, "    ///").unwrap();
    writeln!(
        out,
        "    /// - If `bytes.len() < ::std::mem::size_of::<Self>()`."
    )
    .unwrap();
    writeln!(
        out,
        "    /// - If `bytes.as_mut_ptr()` is not aligned to `::std::mem::align_of::<Self>()`."
    )
    .unwrap();
    writeln!(out, "    #[inline]").unwrap();
    writeln!(
        out,
        "    pub fn from_bytes_mut(bytes: &mut [u8]) -> &mut Self {{"
    )
    .unwrap();
    writeln!(out, "        let size = ::std::mem::size_of::<Self>();").unwrap();
    writeln!(out, "        let align = ::std::mem::align_of::<Self>();").unwrap();
    writeln!(
        out,
        "        assert!(bytes.len() >= size, \"{shm_name}::from_bytes_mut: buffer too small ({{}} < {{}})\", bytes.len(), size);"
    )
    .unwrap();
    writeln!(
        out,
        "        assert!(bytes.as_mut_ptr().align_offset(align) == 0, \"{shm_name}::from_bytes_mut: misaligned source pointer (need align {{}})\", align);"
    )
    .unwrap();
    writeln!(
        out,
        "        // SAFETY: see from_bytes; mutable borrow follows the same invariants."
    )
    .unwrap();
    writeln!(
        out,
        "        unsafe {{ &mut *(bytes.as_mut_ptr() as *mut Self) }}"
    )
    .unwrap();
    writeln!(out, "    }}").unwrap();
    writeln!(out).unwrap();
}

/// Emit `pub fn snapshot(&self) -> <Name>Snapshot`.
fn generate_shm_snapshot_method(out: &mut String, schema: &MessageSchema, shm_name: &str) {
    let snapshot_name = format!("{}Snapshot", schema.name);
    writeln!(
        out,
        "    /// Copy this SHM-backed message into a heap-owned [`{snapshot_name}`]."
    )
    .unwrap();
    writeln!(out, "    ///").unwrap();
    writeln!(
        out,
        "    /// Used by tests, replay, and any code path that needs to outlive the iceoryx2"
    )
    .unwrap();
    writeln!(
        out,
        "    /// sample. Copies field-by-field; `bool` fields convert from `u8` (`!= 0`)."
    )
    .unwrap();
    writeln!(out, "    pub fn snapshot(&self) -> {snapshot_name} {{").unwrap();
    writeln!(out, "        {snapshot_name} {{").unwrap();
    for field in &schema.fields {
        let field_name = escape_keyword(&field.name);
        let expr = shm_to_snapshot_expr(&field.field_type, &format!("self.{field_name}"));
        writeln!(out, "            {field_name}: {expr},").unwrap();
    }
    writeln!(out, "        }}").unwrap();
    writeln!(out, "    }}").unwrap();
    writeln!(out).unwrap();
    let _ = shm_name; // silence compiler — shm_name reserved for future doc use
}

/// Emit `pub fn write_from_snapshot(&mut self, snap: &<Name>Snapshot)`.
fn generate_shm_write_from_snapshot_method(
    out: &mut String,
    schema: &MessageSchema,
    shm_name: &str,
) {
    let snapshot_name = format!("{}Snapshot", schema.name);
    writeln!(
        out,
        "    /// Write a heap-owned [`{snapshot_name}`] into this SHM-backed message."
    )
    .unwrap();
    writeln!(out, "    ///").unwrap();
    writeln!(
        out,
        "    /// Inverse of [`snapshot`](Self::snapshot). Copies field-by-field; `bool` fields"
    )
    .unwrap();
    writeln!(
        out,
        "    /// convert to `u8` (`true` -> `1`, `false` -> `0`)."
    )
    .unwrap();
    writeln!(
        out,
        "    pub fn write_from_snapshot(&mut self, snap: &{snapshot_name}) {{"
    )
    .unwrap();
    if schema.fields.is_empty() {
        writeln!(out, "        let _ = snap;").unwrap();
    } else {
        for field in &schema.fields {
            let field_name = escape_keyword(&field.name);
            let expr = snapshot_to_shm_expr(&field.field_type, &format!("snap.{field_name}"));
            writeln!(out, "        self.{field_name} = {expr};").unwrap();
        }
    }
    writeln!(out, "    }}").unwrap();
    writeln!(out).unwrap();
    let _ = shm_name; // silence compiler — shm_name reserved for future doc use
}

/// Convert an SHM-typed field expression to a snapshot-typed value.
///
/// Bool fields convert `u8 != 0` → `bool`. Fixed arrays of bool need
/// element-wise conversion. Nested SHM fields delegate to their own
/// `snapshot()` method. Fixed arrays of nested also delegate per-element.
fn shm_to_snapshot_expr(ft: &FieldType, value: &str) -> String {
    match ft {
        FieldType::Bool => format!("{value} != 0"),
        FieldType::Nested { .. } => format!("{value}.snapshot()"),
        FieldType::FixedArray {
            element_type,
            length,
        } if matches!(element_type.as_ref(), FieldType::Bool) => {
            // Element-wise convert: `[u8; N]` -> `[bool; N]`. Closure-style
            // `core::array::from_fn` keeps it `Copy`-friendly without an
            // intermediate `Vec`.
            format!("::std::array::from_fn::<bool, {length}, _>(|i| {value}[i] != 0)")
        }
        FieldType::FixedArray {
            element_type,
            length,
        } if matches!(element_type.as_ref(), FieldType::Nested { .. }) => {
            // Per-element snapshot of a fixed array of nested types.
            let elem = field_type_to_rust_snapshot(element_type);
            format!("::std::array::from_fn::<{elem}, {length}, _>(|i| {value}[i].snapshot())")
        }
        // Primitive fields and fixed arrays of primitives are bit-identical
        // between SHM and snapshot, so a direct copy is correct.
        _ => value.to_string(),
    }
}

/// Convert a snapshot-typed field expression to an SHM-typed value.
fn snapshot_to_shm_expr(ft: &FieldType, value: &str) -> String {
    match ft {
        FieldType::Bool => format!("{value} as u8"),
        FieldType::Nested { schema_name, .. } => {
            // Construct a default SHM-of-nested then write the snapshot into it.
            // SAFETY-wise this is fine because the SHM struct of a fixed
            // schema is just a `#[repr(C)]` plain-data struct.
            format!(
                "{{ let mut __shm = {schema_name}Shm::default(); __shm.write_from_snapshot(&{value}); __shm }}"
            )
        }
        FieldType::FixedArray {
            element_type,
            length,
        } if matches!(element_type.as_ref(), FieldType::Bool) => {
            format!("::std::array::from_fn::<u8, {length}, _>(|i| {value}[i] as u8)")
        }
        FieldType::FixedArray {
            element_type,
            length,
        } if matches!(element_type.as_ref(), FieldType::Nested { .. }) => {
            let FieldType::Nested { schema_name, .. } = element_type.as_ref() else {
                unreachable!()
            };
            let elem = field_type_to_rust_shm(element_type);
            format!(
                "::std::array::from_fn::<{elem}, {length}, _>(|i| {{ let mut __shm = {schema_name}Shm::default(); __shm.write_from_snapshot(&{value}[i]); __shm }})"
            )
        }
        _ => value.to_string(),
    }
}

/// Path to the `serde` re-export at `cerulion_core`'s crate root.
///
/// Generated code is compiled inside a crate whose `[dependencies]` this
/// generator does not control — a scaffolded node crate lists exactly
/// `cerulion_core` and
/// `native_ros2_messages` — so `::serde` may not resolve there. Routing every
/// generated reference through `cerulion_core`'s own re-export keeps the
/// user's `Cargo.toml` byte-unchanged.
const SERDE_PATH: &str = "::cerulion_core::serde";

/// Path to the >32-element fixed-array serde helper.
const BIG_ARRAY_PATH: &str = "::cerulion_core::codegen::big_array";

/// Path to the `CerulionState` derive.
///
/// Same routing rule as [`SERDE_PATH`]: named through `cerulion_core`'s own
/// re-export so a scaffolded node crate — whose `[dependencies]` is exactly
/// `cerulion_core` and `native_ros2_messages` — resolves it without adding a
/// line to its `Cargo.toml`.
const STATE_PATH: &str = "::cerulion_core::state::CerulionState";

/// Emit the `#[derive(...)]` (and its `#[serde(crate = ...)]` companion) for a
/// `<Name>Snapshot` struct. Shared by the fixed and variable emitters so the
/// two can never disagree about what a snapshot derives.
///
/// Serde is derived for EVERY schema. It used to be skipped
/// for any schema (transitively) carrying a fixed array longer than 32
/// elements, because serde's array impls stop there — three vendored ROS 2
/// covariance messages tripped it and, because nested resolution is
/// transitive, poisoned nine snapshot types in total including
/// `nav_msgs/Odometry`. Those fields now carry a per-field
/// `#[serde(with = ...)]` (see [`emit_snapshot_field_serde_attr`]) instead.
///
/// `Debug` is still skipped for large-array schemas — that is a deliberate,
/// unrelated narrowing of the generated surface and is unchanged here.
///
/// `PartialEq` on f32/f64 gives a clippy warning under some lint configs; the
/// snapshot is only used for snapshot equality in tests/replay where
/// bit-equality is acceptable, so we keep it.
///
/// Manual `Default` is forced by a DIRECT large array (no stdlib blanket past
/// 32) OR a non-zero scalar field default (`float64 w 1` — derive would
/// produce zeros); transitive-only cases MUST derive (clippy
/// `derivable_impls`).
///
/// `CerulionState` is part of this derive list, beside serde ("derive
/// `CerulionState` alongside serde"): the two derives ship together, never
/// one without the other.
///
/// It is load-bearing rather than tidy: a node that holds a ROS message as
/// PLAIN STATE holds the `<Name>Snapshot` (the marker type is a ZST port
/// declaration and carries no data), so without this derive every such node
/// FAILS TO COMPILE the moment `#[cerulion_node]` starts emitting
/// `impl CerulionState` — which is why the derive must ship together with
/// that emission rather than after it.
///
/// Routed through `::cerulion_core::state::CerulionState` for exactly the
/// reason `SERDE_PATH` exists: a scaffolded node crate's `[dependencies]` is
/// two lines, so a bare `cerulion_macros` path would not resolve there. The
/// derive is re-exported beside the trait, as `serde` does for `Serialize`.
///
/// No `#[cerulion(...)]` companion attribute is emitted: every snapshot field
/// is plain data (scalars, `bool`, `String`, `Vec<T>`, `[T; N]`, nested
/// snapshots), all of which are in the closed inventory, so the corpus is
/// zero-tag by construction. A large fixed array is covered by the inventory's
/// const-generic `[T; N]` impl and needs no `serde`-style per-field helper.
fn emit_snapshot_derives(out: &mut String, schema: &MessageSchema) {
    let debug = if has_large_array(schema) {
        ""
    } else {
        "Debug, "
    };
    let default = if needs_manual_default(schema) {
        ""
    } else {
        "Default, "
    };
    writeln!(
        out,
        "#[derive({debug}Clone, {default}PartialEq, {SERDE_PATH}::Serialize, {SERDE_PATH}::Deserialize, {STATE_PATH})]"
    )
    .unwrap();
    // REQUIRED, not cosmetic: without it serde's derive emits
    // `extern crate serde as _serde;`, which needs `serde` in the extern
    // prelude of whatever crate the generated code lands in.
    writeln!(out, "#[serde(crate = \"{SERDE_PATH}\")]").unwrap();
}

/// Emit the per-field `#[serde(with = ...)]` attribute for a snapshot field
/// whose Rust type is a fixed array longer than serde's 32-element ceiling.
///
/// Writes nothing for every other field, so a schema with no large array emits
/// byte-identical field lines to earlier codegen.
fn emit_snapshot_field_serde_attr(out: &mut String, ft: &FieldType) {
    if field_needs_big_array_serde(ft) {
        writeln!(out, "    #[serde(with = \"{BIG_ARRAY_PATH}\")]").unwrap();
    }
}

/// True when a snapshot field's own type is a fixed array past serde's
/// blanket-impl ceiling, and therefore needs the [`big_array`] helper.
///
/// [`big_array`]: crate::codegen::big_array
///
/// **This predicate MIRRORS [`field_type_to_rust_snapshot`], and must keep
/// doing so** — the helper's signature is `&[T; N]`, so the attribute is
/// correct exactly when that function emits an ARRAY. Its first act is an
/// `is_complex_variable` short-circuit to `Vec<u8>`, so this one leads with the
/// same test. (Pinned by `field_needs_big_array_serde_agrees_with_the_emitted_
/// snapshot_type`, which drives BOTH functions over one field-type corpus
/// rather than trusting the two to be edited together.)
///
/// Deliberately NOT the transitive [`has_large_array`] predicate. That one
/// answers "does this schema embed a large array anywhere", which is what
/// gates the `Debug` derive; this one answers "is THIS field's Rust type one
/// serde cannot handle", which is a strictly narrower question:
///
/// - `float64[36]` → yes (`[f64; 36]`).
/// - `Header[36]` → **no**, and this is the case the leading conjunct exists
///   for. `Header` never resolves fixed (it carries a `string`), so the
///   snapshot field is `Vec<u8>` — serde handles it unaided, and attaching the
///   helper emits code that does not COMPILE. Ordinary `.msg` syntax, so this
///   is reachable from a user schema (a build failure, not a nit).
/// - `Vec3[36]` → yes. A resolved-FIXED nested element keeps the field
///   array-backed (`[Vec3Snapshot; 36]`), and `is_complex_variable` recurses
///   into the element type to say so.
/// - `PoseWithCovariance pose` → no. The field's type is
///   `PoseWithCovarianceSnapshot`, which derives serde in its own right;
///   the large array is that type's problem, already solved there.
/// - `PoseWithCovariance[4] poses` → no. `[PoseWithCovarianceSnapshot; 4]` is
///   within serde's ceiling and its element is `Serialize`.
///
/// A fixed array OF large fixed arrays would need this and cannot be served
/// (see [`big_array`]'s module docs); no schema front end can express one.
fn field_needs_big_array_serde(ft: &FieldType) -> bool {
    // `Vec<u8>`-backed fields are handled by serde unaided — and CANNOT take a
    // `[T; N]` helper. Mirrors `field_type_to_rust_snapshot`'s own first line.
    !is_complex_variable(ft)
        && matches!(ft, FieldType::FixedArray { length, .. } if *length > crate::codegen::big_array::SERDE_ARRAY_IMPL_CEILING)
}

/// Generate the heap-owned `<Name>Snapshot` struct for a fixed schema.
///
/// Plain-data companion to `<Name>Shm`. Bool stays as `bool`. Used by tests
/// and replay paths that need to escape the iceoryx2 sample's lifetime.
pub(super) fn generate_snapshot_struct(out: &mut String, schema: &MessageSchema) {
    let name = &schema.name;
    let snapshot_name = format!("{name}Snapshot");

    writeln!(out, "/// Heap-owned snapshot of a [`{name}`] message.").unwrap();
    writeln!(out, "///").unwrap();
    writeln!(
        out,
        "/// Companion to [`{name}Shm`] for code that needs to escape the iceoryx2"
    )
    .unwrap();
    writeln!(
        out,
        "/// sample lifetime — tests, replay buffers, network bridges. Round-trips"
    )
    .unwrap();
    writeln!(out, "/// to/from SHM via [`{name}Shm::snapshot`] and").unwrap();
    writeln!(out, "/// [`{name}Shm::write_from_snapshot`].").unwrap();

    emit_snapshot_derives(out, schema);
    writeln!(out, "pub struct {snapshot_name} {{").unwrap();

    for field in &schema.fields {
        if let Some(desc) = &field.description {
            writeln!(out, "    /// {desc}").unwrap();
        }
        // Snapshot fields use plain Rust types (`bool` stays as `bool`).
        // For nested fields the field type is `<NestedName>Snapshot` — the
        // marker `<NestedName>` is unit and can't carry data on the heap, so
        // snapshot composition goes through the snapshot type instead.
        let rust_type = field_type_to_rust_snapshot(&field.field_type);
        let field_name = escape_keyword(&field.name);
        emit_snapshot_field_serde_attr(out, &field.field_type);
        writeln!(out, "    pub {field_name}: {rust_type},").unwrap();
    }

    writeln!(out, "}}").unwrap();
    writeln!(out).unwrap();

    // Manual Default impl when forced (direct large array or non-zero
    // scalar defaults).
    if needs_manual_default(schema) {
        generate_snapshot_default_impl(out, schema, &snapshot_name);
        // Skip Debug for large-array schemas — large arrays of f64 have
        // stdlib Debug, but suppressing it keeps the surface narrow.
    }
}

/// Manual `Default` for `<Name>Snapshot` when derive is impossible (direct
/// large array) or wrong (non-zero scalar field defaults).
fn generate_snapshot_default_impl(out: &mut String, schema: &MessageSchema, snapshot_name: &str) {
    writeln!(out, "impl Default for {snapshot_name} {{").unwrap();
    writeln!(out, "    fn default() -> Self {{").unwrap();
    writeln!(out, "        Self {{").unwrap();
    for field in &schema.fields {
        let field_name = escape_keyword(&field.name);
        let default_value = fixed_snapshot_field_default_expr(field);
        writeln!(out, "            {field_name}: {default_value},").unwrap();
    }
    writeln!(out, "        }}").unwrap();
    writeln!(out, "    }}").unwrap();
    writeln!(out, "}}").unwrap();
    writeln!(out).unwrap();
}

// =============================================================================
// SHM-backed type emission for VARIABLE schemas
// =============================================================================
//
// For every variable schema (`has_variable_fields() == true`) we emit:
//
//   1. `<Name>FixedSection` — `#[repr(C)]` plain-data overlay holding the
//      schema's fixed primitive/StringFixed/FixedArray<primitive> fields as
//      `pub <field>: <shm_type>`. Bool is stored as `u8`.
//   2. `<Name>Shm<'a>` — opaque struct holding `*mut u8 + len + WriterState<N>`.
//      Per-tick state lives inside; it has no `pub` fields. Constructed via
//      `from_bytes(&'a [u8])` (read) or `from_bytes_mut(&'a mut [u8])` (write).
//      `Deref` / `DerefMut` to `<Name>FixedSection` so user code writes
//      `proxy.height = 1080` directly into the loaned SHM slot.
//   3. `<Name>Snapshot` — plain-data companion (reuses `generate_snapshot_struct`
//      with a snapshot-aware type mapper that maps Nested → `<Name>Snapshot`).
//   4. `impl ShmMessage for <Name>` (in `wire_impl.rs`) with VARIABLE_FIELD_COUNT
//      and `WIRE_FIXED_SIZE = size_of::<<Name>FixedSection>()`.
//
// Wire layout (matches the wire spec):
//
//   [WireHeader (32 bytes)][fixed_section][offset_table][variable_payload]
//
// `<Name>Shm<'a>` borrows the bytes after the WireHeader as `payload: &'a mut [u8]`.
// The fixed section starts at `payload[0]` and is laid out as `<Name>FixedSection`
// (`#[repr(C)]`, native alignment + trailing padding to satisfy the
// struct's alignment; it was originally packed). The offset table starts at
// `payload[WIRE_FIXED_SIZE]` and has `VARIABLE_FIELD_COUNT` entries × 8 bytes
// (offset: u32, length: u32). Variable payload follows.
//
// SCOPE LIMITATIONS:
// - Fixed-section fields support: primitives, FixedArray<primitive>, StringFixed.
// - Variable fields:
//     - String: typed `&str` reader / setter.
//     - Bytes: typed `&[u8]` reader / setter, `push_<field>(u8)`.
//     - DynamicArray<T> where T is primitive: typed `&[T]` reader / setter,
//       `push_<field>(T)`. Cursor aligns to align_of::<T>() before reserving.
//     - DynamicArray<Nested/String/Bytes>, Nested, FixedArray<Nested>: emit
//       raw `&[u8]` accessors (`<field>_bytes()`, `set_<field>_bytes(&[u8])`,
//       `loan_<field>_bytes(n)`). Typed wrapping is left to the macro
//       layer.
//
// Snapshot field types follow the same rule: typed for the supported variable
// kinds, raw `Vec<u8>` (bytes) for nested/array-of-nested. Snapshot derives
// Default + serde.

/// Returns `true` if this field's variable side is "complex" — i.e., it's a
/// Nested type, an array of Nested, or an array of any non-primitive type.
/// Complex fields get raw `&[u8]` accessors instead of typed ones.
///
/// "Simple" (typed) cases handled directly: String, Bytes, DynamicArray<T>
/// where T is a primitive type (Bool, integer, float).
pub(super) fn is_complex_variable(ft: &FieldType) -> bool {
    match ft {
        // A resolved-fixed nested reference is NOT a variable
        // field at all (it lives in the fixed section); only unresolved
        // nested references take the complex raw-bytes path.
        FieldType::Nested { fixed, .. } => fixed.is_none(),
        FieldType::DynamicArray { element_type } => {
            // Only DynArray<primitive> is typed-friendly. Anything else
            // (Nested, String, Bytes, DynArray-of-DynArray, FixedArray-of-X)
            // is complex.
            !matches!(
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
            )
        }
        FieldType::FixedArray { element_type, .. } => is_complex_variable(element_type),
        _ => false,
    }
}

/// Map a [`FieldType`] to the Snapshot field type string.
///
/// Differs from `field_type_to_rust` only in two places:
/// - `Nested { schema_name }` → `<schema_name>Snapshot` (so cross-snapshot
///   composition works without leaking the legacy heap struct's missing
///   serde derives).
/// - "Complex variable" fields (see [`is_complex_variable`]) become
///   `::std::vec::Vec<u8>` because codegen only stores them as raw bytes.
pub(super) fn field_type_to_rust_snapshot(ft: &FieldType) -> String {
    if is_complex_variable(ft) {
        return "::std::vec::Vec<u8>".to_string();
    }
    match ft {
        FieldType::Nested { schema_name, .. } => format!("{schema_name}Snapshot"),
        FieldType::FixedArray {
            element_type,
            length,
        } => {
            let elem = field_type_to_rust_snapshot(element_type);
            format!("[{elem}; {length}]")
        }
        FieldType::DynamicArray { element_type } => {
            let elem = field_type_to_rust_snapshot(element_type);
            format!("::std::vec::Vec<{elem}>")
        }
        // Primitives, String, Bytes, StringFixed → identical to legacy mapper.
        _ => field_type_to_rust(ft),
    }
}

/// Default-value expression for a Snapshot field.
///
/// Identical to [`field_type_default_value`] except `Nested` becomes
/// `<schema_name>Snapshot::default()`.
fn snapshot_field_default_value(ft: &FieldType) -> String {
    if is_complex_variable(ft) {
        return "::std::vec::Vec::new()".to_string();
    }
    match ft {
        FieldType::Nested { schema_name, .. } => format!("{schema_name}Snapshot::default()"),
        FieldType::FixedArray {
            element_type,
            length,
        } => {
            // Array-repeat needs `Copy`; Snapshot element types are
            // Clone-only — route those through `array::from_fn`.
            if let Some(expr) = snapshot_array_element_default(element_type, *length) {
                return expr;
            }
            let elem_default = snapshot_field_default_value(element_type);
            format!("[{elem_default}; {length}]")
        }
        _ => field_type_default_value(ft),
    }
}

/// Returns `true` if `ft` is a "fixed-section" field type — primitive,
/// `StringFixed`, `FixedArray<fixed>`, or a `Nested` reference
/// whose target schema resolved as recursively fixed. Fields with this
/// property live in `<Name>FixedSection` (a `#[repr(C)]` overlay struct);
/// everything else (`String`, `Bytes`, `DynamicArray<*>`, UNRESOLVED
/// `Nested`) lives in the offset table + variable payload region.
fn is_fixed_section_field(ft: &FieldType) -> bool {
    match ft {
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
        | FieldType::StringFixed(_) => true,
        FieldType::FixedArray { element_type, .. } => is_fixed_section_field(element_type),
        // A nested reference whose target schema resolved as
        // recursively fixed lives inline in the fixed section as
        // `<Target>Shm` — direct field access, zero-copy.
        FieldType::Nested { fixed, .. } => fixed.is_some(),
        // String, Bytes, DynamicArray, unresolved Nested → variable.
        _ => false,
    }
}

/// Iterate over the fixed-section fields in declaration order.
///
/// The fixed section is emitted as `#[repr(C)] pub struct <Name>FixedSection`
/// — Rust assigns natural alignment + trailing padding. Each field becomes
/// a `pub <name>: <shm_type>` member of that overlay struct.
fn fixed_section_fields(schema: &MessageSchema) -> impl Iterator<Item = &FieldDef> {
    schema
        .fields
        .iter()
        .filter(|f| is_fixed_section_field(&f.field_type))
}

/// Iterate over variable fields with their variable-field index (in
/// declaration order, restricted to variable fields).
fn variable_field_layout(schema: &MessageSchema) -> Vec<(&FieldDef, usize)> {
    let mut out = Vec::new();
    let mut idx = 0usize;
    for field in &schema.fields {
        if !is_fixed_section_field(&field.field_type) {
            out.push((field, idx));
            idx += 1;
        }
    }
    out
}

/// Generate `<Name>Shm<'a>` for a variable schema.
///
/// Caller must verify `schema.has_variable_fields() == true`. For fixed
/// schemas, `generate_shm_struct` is used instead.
pub(super) fn generate_variable_shm_struct(out: &mut String, schema: &MessageSchema) {
    let name = &schema.name;
    let shm_name = format!("{name}Shm");
    let fixed_section_name = format!("{name}FixedSection");
    let snapshot_name = format!("{name}Snapshot");
    let n_var = schema.variable_field_count();

    // Collision check. Compute once and emit one consolidated
    // `compile_error!` if any fixed-section field name shadows a reserved
    // accessor, private field, or constant on `<Name>Shm`. Colliding fields
    // are skipped in the FixedSection emission below so rustc doesn't pile
    // up cascading "field cannot be reached through Deref" errors.
    let collisions = collect_reserved_collisions(schema);

    // Emit the `#[repr(C)] <Name>FixedSection` overlay first — `<Name>Shm`
    // refers to it via `WIRE_FIXED_SIZE` and Deref.
    generate_variable_fixed_section_struct(out, schema, &fixed_section_name, &collisions);

    // Emit the consolidated reserved-name `compile_error!` (no-op when the
    // collision set is empty) before the `<Name>Shm` definition so the
    // diagnostic anchors at the top of the schema's generated section.
    emit_reserved_collision_compile_error(out, schema, &collisions);

    writeln!(out, "/// SHM-backed view of [`{name}`].").unwrap();
    writeln!(out, "///").unwrap();
    writeln!(
        out,
        "/// Variable-length schema. Holds a borrow into iceoryx2-loaned shared memory plus"
    )
    .unwrap();
    writeln!(
        out,
        "/// per-tick writer state ([`WriterState`](::cerulion_core::shm_runtime::WriterState))."
    )
    .unwrap();
    writeln!(out, "/// Wire layout of `payload`:").unwrap();
    writeln!(out, "///").unwrap();
    writeln!(out, "/// ```text").unwrap();
    writeln!(
        out,
        "/// [<{fixed_section_name}>][offset_table ({n_var} × 8 bytes)][variable_payload ...]"
    )
    .unwrap();
    writeln!(out, "/// ```").unwrap();
    writeln!(out, "///").unwrap();
    writeln!(
        out,
        "/// Reader and writer paths share this type. Variable-field setters are gated on"
    )
    .unwrap();
    writeln!(
        out,
        "/// `&mut self`; readers take `&self`. Construct via [`from_bytes`](Self::from_bytes)"
    )
    .unwrap();
    writeln!(
        out,
        "/// for read or [`from_bytes_mut`](Self::from_bytes_mut) for write."
    )
    .unwrap();
    writeln!(out, "///").unwrap();
    writeln!(
        out,
        "/// Fixed-section fields are exposed via `Deref<Target = {fixed_section_name}>` so"
    )
    .unwrap();
    writeln!(
        out,
        "/// `proxy.<field> = value` writes directly into the loaned SHM slot."
    )
    .unwrap();
    writeln!(out, "pub struct {shm_name}<'a> {{").unwrap();
    writeln!(
        out,
        "    /// Raw pointer to the start of the active payload region."
    )
    .unwrap();
    writeln!(out, "    ///").unwrap();
    writeln!(
        out,
        "    /// Lifetime is tied to `'a` via the phantom field below. Under"
    )
    .unwrap();
    writeln!(
        out,
        "    /// the overflow-redirect path, `payload_mut()` may"
    )
    .unwrap();
    writeln!(
        out,
        "    /// repoint this to `overflow.as_mut().unwrap().as_mut_ptr()` —"
    )
    .unwrap();
    writeln!(
        out,
        "    /// after the spill, all cursor / setter math operates against"
    )
    .unwrap();
    writeln!(
        out,
        "    /// the heap buffer instead of the loaned SHM slice."
    )
    .unwrap();
    writeln!(out, "    ptr: *mut u8,").unwrap();
    writeln!(out, "    /// Length of the active payload region in bytes.").unwrap();
    writeln!(out, "    ///").unwrap();
    writeln!(
        out,
        "    /// Equals the loan size minus `WireHeader::SIZE` under the"
    )
    .unwrap();
    writeln!(
        out,
        "    /// steady-state path; equals `max_capacity as usize` after a"
    )
    .unwrap();
    writeln!(
        out,
        "    /// spill to the heap fallback. The existing `cursor + bytes_needed >"
    )
    .unwrap();
    writeln!(
        out,
        "    /// self.len` checks in every setter naturally guard against"
    )
    .unwrap();
    writeln!(
        out,
        "    /// the absolute ceiling without needing a second compare site."
    )
    .unwrap();
    writeln!(out, "    len: usize,").unwrap();
    writeln!(
        out,
        "    /// Per-tick writer state (cursor, written bitset, per-field starts)."
    )
    .unwrap();
    writeln!(
        out,
        "    state: ::cerulion_core::shm_runtime::WriterState<{n_var}>,"
    )
    .unwrap();
    writeln!(out, "    /// Lazily-allocated heap fallback buffer.").unwrap();
    writeln!(out, "    ///").unwrap();
    writeln!(
        out,
        "    /// `None` in the steady-state hot path (every loan large"
    )
    .unwrap();
    writeln!(
        out,
        "    /// enough). Allocated lazily via `Vec::try_reserve_exact("
    )
    .unwrap();
    writeln!(
        out,
        "    /// max_capacity.div_ceil(8))` the first time a setter would"
    )
    .unwrap();
    writeln!(out, "    /// otherwise overflow the loan; OOM surfaces as").unwrap();
    writeln!(
        out,
        "    /// `AllocationFailed`. Once set, `payload_mut()` returns a"
    )
    .unwrap();
    writeln!(
        out,
        "    /// slice over this buffer instead of the SHM slot — all"
    )
    .unwrap();
    writeln!(
        out,
        "    /// subsequent writes for THIS proxy go to the heap."
    )
    .unwrap();
    writeln!(out, "    /// `OutputProxy::Drop` reads the spill bytes via").unwrap();
    writeln!(
        out,
        "    /// `T::overflow_view_bytes` (BORROWING; production never calls"
    )
    .unwrap();
    writeln!(
        out,
        "    /// `take_overflow`), re-loans a fresh sample sized to fit,"
    )
    .unwrap();
    writeln!(
        out,
        "    /// memcpies header + heap bytes in, and sends — paying one"
    )
    .unwrap();
    writeln!(
        out,
        "    /// memcpy per overflow tick (back to zero-copy on the next)."
    )
    .unwrap();
    writeln!(
        out,
        "    overflow: ::std::option::Option<::std::boxed::Box<[u64]>>,"
    )
    .unwrap();
    writeln!(
        out,
        "    /// Hard ceiling on the payload (max_slice_len minus"
    )
    .unwrap();
    writeln!(
        out,
        "    /// the 32-byte WireHeader). Read by the spill helper to decide"
    )
    .unwrap();
    writeln!(
        out,
        "    /// between `PayloadTooLarge` (no rescue possible) and lazy"
    )
    .unwrap();
    writeln!(
        out,
        "    /// allocation. Set at `from_bytes_mut` time from the publisher's"
    )
    .unwrap();
    writeln!(
        out,
        "    /// `MaxSliceLen::get() - WireHeader::SIZE as u32`."
    )
    .unwrap();
    writeln!(
        out,
        "    max_capacity: ::cerulion_core::wire::MaxPayloadCapacity,"
    )
    .unwrap();
    writeln!(out, "    /// Topic name as `Arc<str>`. Used to").unwrap();
    writeln!(
        out,
        "    /// attribute `PayloadTooLarge` / `AllocationFailed` errors to"
    )
    .unwrap();
    writeln!(
        out,
        "    /// the originating topic — codegen doesn't know the topic at"
    )
    .unwrap();
    writeln!(
        out,
        "    /// expansion time, so the publisher passes it via `build_writer`."
    )
    .unwrap();
    writeln!(out, "    ///").unwrap();
    writeln!(
        out,
        "    /// `Arc<str>` preserves the zero-alloc invariant: each"
    )
    .unwrap();
    writeln!(
        out,
        "    /// loan is one `Arc::clone` (atomic refcount bump), no allocation."
    )
    .unwrap();
    writeln!(
        out,
        "    /// Publisher allocates the Arc once at construction. The"
    )
    .unwrap();
    writeln!(
        out,
        "    /// Arc-not-borrow choice eliminates the raw-pointer reborrow"
    )
    .unwrap();
    writeln!(
        out,
        "    /// `unsafe` block that the pre-deferral-fix `&'a str` variant"
    )
    .unwrap();
    writeln!(
        out,
        "    /// required in `loan_proxy`. Only the error path allocates"
    )
    .unwrap();
    writeln!(out, "    /// (`self.topic.to_string()` per error).").unwrap();
    writeln!(out, "    topic: ::std::sync::Arc<str>,").unwrap();
    writeln!(
        out,
        "    /// Lifetime carrier. Written-side construction goes through"
    )
    .unwrap();
    writeln!(
        out,
        "    /// `&'a mut [u8]`; read-side through `&'a [u8]`. The struct itself"
    )
    .unwrap();
    writeln!(
        out,
        "    /// is `!Send + !Sync` because the raw pointer is non-null but not"
    )
    .unwrap();
    writeln!(
        out,
        "    /// thread-safe — matches the iceoryx2 sample's `!Send` guarantee."
    )
    .unwrap();
    writeln!(
        out,
        "    _phantom: ::std::marker::PhantomData<&'a mut [u8]>,"
    )
    .unwrap();
    // Write-only diagnostic proxy members, one per VARIABLE field,
    // named exactly after the field. `<Name>Shm<'a>` is a plain (NON-repr(C))
    // wrapper, so adding zero-sized fields changes neither its SHM layout nor
    // any derive. Plain field resolution finds these proxies BEFORE the
    // `DerefMut` to `<Name>FixedSection`, so an un-rewritten variable-field
    // access (`self.<port>.<f>[i]`, `+= …`) lands on the proxy and produces a
    // codegen-authored diagnostic instead of a bare `no field <f>` error. See
    // `generate_variable_field_proxies`.
    emit_proxy_field_decls(out, schema);
    // Per complex-nested variable field, the lazily-allocated
    // staging slot backing the `self.<port>.<f>.<leaf> = …` / `with_<f>`
    // nested-writer sugar. Private; `None` until first touch.
    emit_staged_field_decls(out, schema);
    writeln!(out, "}}").unwrap();
    writeln!(out).unwrap();

    // Inherent impl block.
    writeln!(out, "impl<'a> {shm_name}<'a> {{").unwrap();

    // WIRE_FIXED_SIZE associated constant (mirrors the trait's value, used by
    // codegen-emitted bodies to avoid stringly typed offsets). It is
    // sourced from `size_of::<<Name>FixedSection>()` so the wire layout
    // tracks the `#[repr(C)]` overlay (matches fixed-only schemas).
    writeln!(out, "    /// Size of the fixed section in bytes.").unwrap();
    writeln!(
        out,
        "    pub const WIRE_FIXED_SIZE: usize = ::std::mem::size_of::<{fixed_section_name}>();"
    )
    .unwrap();
    writeln!(out, "    /// Number of variable-length fields.").unwrap();
    writeln!(out, "    pub const VARIABLE_FIELD_COUNT: usize = {n_var};").unwrap();
    writeln!(
        out,
        "    /// Byte offset of the offset table from `payload[0]`."
    )
    .unwrap();
    writeln!(
        out,
        "    pub const OFFSET_TABLE_OFFSET: usize = Self::WIRE_FIXED_SIZE;"
    )
    .unwrap();
    writeln!(
        out,
        "    /// Byte length of the offset table (8 bytes per variable field)."
    )
    .unwrap();
    writeln!(
        out,
        "    pub const OFFSET_TABLE_BYTES: usize = 8 * Self::VARIABLE_FIELD_COUNT;"
    )
    .unwrap();
    writeln!(out).unwrap();

    // Constructors.
    generate_var_shm_from_bytes(out, schema, &shm_name, &fixed_section_name);
    generate_var_shm_from_bytes_mut(out, schema, &shm_name, &fixed_section_name);

    // Internal payload accessors (private helpers).
    generate_var_shm_payload_helpers(out);

    // Variable-field accessors. (There are no per-fixed-field setters
    // and getters — fixed fields are reached via `Deref<Target =
    // <Name>FixedSection>` instead.)
    for (field, var_idx) in variable_field_layout(schema) {
        emit_variable_field_accessors(out, field, var_idx);
    }

    // Staging surface. Export/take are emitted for EVERY
    // variable schema (any variable schema can be the TARGET of another
    // schema's complex-nested field); the with_/flush accessors only for
    // schemas that themselves EMBED complex-nested fields.
    generate_var_shm_staging_export_take(out, &shm_name);
    // Children take/restore + the inherent `__cer_flush_staged`
    // are emitted for EVERY variable schema (not just embedders): a parent's
    // flush recurses via `__cer_view.__cer_flush_staged()` and persists
    // grandchildren via take/restore on the TARGET's view type, and the
    // parent codegen cannot know (registry gap) whether the target embeds
    // complex-nested fields itself. Targets without any get the trivial
    // forms (empty take, warn-only restore, no-op flush). The
    // `ShmMessage::flush_staged_nested` trait override stays gated on
    // schema_has_complex_variable — Drop only needs it when slots exist.
    generate_var_shm_staging_children(out, schema, &shm_name);
    // The staged accessor needs the field's variable-section index for
    // guard B (reject staging a field already written wholesale), so
    // iterate the variable layout (which carries `var_idx`) rather than
    // the raw field list.
    for (field, var_idx) in variable_field_layout(schema) {
        if is_complex_variable(&field.field_type) {
            emit_with_nested_complex(out, field, var_idx);
        }
    }
    emit_flush_staged(out, schema);

    // all_variables_written() helper.
    writeln!(
        out,
        "    /// Returns true iff every declared variable field has been written."
    )
    .unwrap();
    writeln!(
        out,
        "    /// Used by `OutputProxy::Drop` to gate the publish."
    )
    .unwrap();
    writeln!(out, "    #[inline]").unwrap();
    writeln!(out, "    pub fn all_variables_written(&self) -> bool {{").unwrap();
    writeln!(out, "        self.state.all_written()").unwrap();
    writeln!(out, "    }}").unwrap();
    writeln!(out).unwrap();

    // Name the FIRST unwritten variable field (declaration
    // order), or `None` if all are written. Used by a PARENT schema's
    // `__cer_flush_staged` to name the unwritten child field in the
    // `NestedChildIncomplete` discard error. In the `__cer` namespace
    // (user fields starting `__cer` are already rejected), so no reserved-
    // list entry is needed. `&'static str` — the names are compile-time
    // schema literals.
    writeln!(
        out,
        "    /// The first unwritten variable field's name (or `None`)."
    )
    .unwrap();
    writeln!(out, "    #[doc(hidden)]").unwrap();
    writeln!(
        out,
        "    pub fn __cer_first_unwritten_variable(&self) -> ::std::option::Option<&'static str> {{"
    )
    .unwrap();
    for (field, var_idx) in variable_field_layout(schema) {
        let raw = &field.name;
        writeln!(out, "        if !self.state.is_written({var_idx}) {{").unwrap();
        writeln!(
            out,
            "            return ::std::option::Option::Some(\"{raw}\");"
        )
        .unwrap();
        writeln!(out, "        }}").unwrap();
    }
    writeln!(out, "        ::std::option::Option::None").unwrap();
    writeln!(out, "    }}").unwrap();
    writeln!(out).unwrap();

    // Cursor accessor (used by OutputProxy::Drop to compute total_size).
    writeln!(
        out,
        "    /// Current write cursor (in bytes from `payload[0]`)."
    )
    .unwrap();
    writeln!(out, "    #[inline]").unwrap();
    writeln!(out, "    pub fn cursor(&self) -> u32 {{").unwrap();
    writeln!(out, "        self.state.cursor").unwrap();
    writeln!(out, "    }}").unwrap();
    writeln!(out).unwrap();

    // snapshot() and write_from_snapshot() methods.
    generate_var_shm_snapshot_method(out, schema, &snapshot_name);
    generate_var_shm_write_from_snapshot_method(out, schema, &snapshot_name);

    writeln!(out, "}}").unwrap();
    writeln!(out).unwrap();

    // Emit Deref / DerefMut to <Name>FixedSection so user code can
    // write `proxy.<fixed_field> = value` as a direct in-place SHM write.
    generate_variable_shm_deref_impls(out, &shm_name, &fixed_section_name);

    // Emit the write-only diagnostic proxy types + never-implemented
    // marker traits + `Index`/`IndexMut`/compound-assign impls that back the
    // proxy fields declared on `<Name>Shm<'a>` above.
    generate_variable_field_proxies(out, schema);
}

/// Emit `#[repr(C)] pub struct <Name>FixedSection` containing the
/// schema's fixed-section fields as `pub <name>: <shm_type>`. Bool fields
/// are stored as `u8` (matches the fixed-only schema codegen) so the type
/// can be safely transmuted from arbitrary bytes.
///
/// The generated struct is `Default`-derivable when there are no
/// `FixedArray<*, N>` with `N > 32` (stdlib `Default` doesn't blanket-impl
/// for those); when one exists we emit a manual `Default` impl so callers
/// can still construct one for snapshot round-trips and tests.
///
/// For schemas with NO fixed-section fields (i.e. every field lives in the
/// offset table + variable payload), the struct is emitted with a single
/// `_marker: [u8; 0]` field so it remains a valid `#[repr(C)]` overlay with
/// `size_of() == 0` and `align_of() == 1`.
fn generate_variable_fixed_section_struct(
    out: &mut String,
    schema: &MessageSchema,
    fixed_section_name: &str,
    _collisions: &[String],
) {
    let name = &schema.name;
    let has_fixed = fixed_section_fields(schema).next().is_some();

    writeln!(out, "/// Fixed-section overlay for [`{name}Shm`].").unwrap();
    writeln!(out, "///").unwrap();
    writeln!(
        out,
        "/// `#[repr(C)]` plain-data struct holding the fixed primitive / `StringFixed` /"
    )
    .unwrap();
    writeln!(
        out,
        "/// `FixedArray<primitive>` fields of [`{name}`]. Reached from [`{name}Shm`] via"
    )
    .unwrap();
    writeln!(
        out,
        "/// `Deref` / `DerefMut`, so user code writes `proxy.<field> = value` directly"
    )
    .unwrap();
    writeln!(out, "/// into the loaned SHM slot.").unwrap();
    writeln!(out, "///").unwrap();
    writeln!(out, "/// # Bool storage and snapshot round-trip").unwrap();
    writeln!(out, "///").unwrap();
    writeln!(
        out,
        "/// `bool` fields are stored as `u8` (`bool`'s valid bit patterns are restricted to"
    )
    .unwrap();
    writeln!(
        out,
        "/// `0` and `1` — interpreting an arbitrary byte as `bool` would be UB). The snapshot"
    )
    .unwrap();
    writeln!(
        out,
        "/// companion exposes the real `bool`. The conversion is `b != 0` on read and"
    )
    .unwrap();
    writeln!(out, "/// `value as u8` on write.").unwrap();
    writeln!(out, "///").unwrap();
    writeln!(
        out,
        "/// CONTRACT: bool fields must be written as canonical `0` or `1`. Direct field"
    )
    .unwrap();
    writeln!(
        out,
        "/// assignment (`proxy.<bool_field> = u8`) accepts any 0-255 value because the"
    )
    .unwrap();
    writeln!(
        out,
        "/// storage type is `u8`. The snapshot read collapses any non-zero byte to `true`,"
    )
    .unwrap();
    writeln!(
        out,
        "/// and `write_from_snapshot` writes `1` back — so the snapshot round-trip is NOT"
    )
    .unwrap();
    writeln!(
        out,
        "/// bit-identical for non-canonical bool inputs (e.g. `5` → snapshot `true` →"
    )
    .unwrap();
    writeln!(
        out,
        "/// SHM byte `1`). This violates [Replay = Live] (CLAUDE.md Principle #7) only when"
    )
    .unwrap();
    writeln!(
        out,
        "/// the original wire byte is non-canonical — write only `0` or `1` to preserve"
    )
    .unwrap();
    writeln!(out, "/// the determinism guarantee.").unwrap();
    writeln!(out, "///").unwrap();
    writeln!(out, "/// # Derives").unwrap();
    writeln!(out, "///").unwrap();
    writeln!(
        out,
        "/// `Clone + Copy + PartialEq` always. `Debug` unless the schema (transitively)"
    )
    .unwrap();
    writeln!(
        out,
        "/// contains a `FixedArray<*, N>32>`. `Default` is DERIVED unless a direct large"
    )
    .unwrap();
    writeln!(
        out,
        "/// array (stdlib `Default` lacks blanket impls past `N=32`) or a non-zero scalar"
    )
    .unwrap();
    writeln!(
        out,
        "/// field default (e.g. `float64 w 1`) forces a manual impl below."
    )
    .unwrap();
    writeln!(
        out,
        "/// `Eq` / `Hash` are NOT derived because schemas may include `f32` / `f64` fields."
    )
    .unwrap();
    writeln!(out, "#[repr(C)]").unwrap();
    // Derive strategy mirrors `generate_shm_struct`: manual
    // `Default` is forced by a direct large array or non-zero scalar
    // defaults; a transitive large array (inside an embedded resolved-
    // fixed nested type) only poisons `Debug` and MUST derive `Default`
    // (clippy `derivable_impls`). Eq/Hash always skipped — float fields
    // disallow them.
    match (needs_manual_default(schema), has_large_array(schema)) {
        (true, true) => writeln!(out, "#[derive(Clone, Copy, PartialEq)]").unwrap(),
        (true, false) => writeln!(out, "#[derive(Debug, Clone, Copy, PartialEq)]").unwrap(),
        (false, true) => writeln!(out, "#[derive(Clone, Copy, PartialEq, Default)]").unwrap(),
        (false, false) => {
            writeln!(out, "#[derive(Debug, Clone, Copy, PartialEq, Default)]").unwrap()
        }
    }
    writeln!(out, "pub struct {fixed_section_name} {{").unwrap();

    if has_fixed {
        for field in fixed_section_fields(schema) {
            // Note: a field whose name collides with a reserved accessor /
            // private field is still emitted here; the consolidated
            // `compile_error!` (emitted before the impl block) halts the
            // build and gives the schema author the actionable diagnostic.
            // Cascading rustc errors that follow from the collision are
            // noise but harmless — compile_error fires first.
            if let Some(desc) = &field.description {
                writeln!(out, "    /// {desc}").unwrap();
            }
            let rust_type = field_type_to_rust_shm(&field.field_type);
            let field_name = escape_keyword(&field.name);
            writeln!(out, "    pub {field_name}: {rust_type},").unwrap();
        }
    } else {
        // Zero-sized overlay for schemas with no fixed-section fields. The
        // `_marker` field is private (no `pub`) so users can't accidentally
        // depend on its name; the struct is still constructible via
        // `<Name>FixedSection::default()`.
        writeln!(out, "    _marker: [u8; 0],").unwrap();
    }

    writeln!(out, "}}").unwrap();
    writeln!(out).unwrap();

    // Manual Default when forced (direct large array or non-zero scalar
    // defaults); transitive-only cases derive it (clippy `derivable_impls`).
    if needs_manual_default(schema) {
        generate_fixed_section_default_impl(out, schema, fixed_section_name);
    }

    // Uniform per-field `__cer_assign_<f>` write shims for the
    // fixed-section fields. `<Name>Shm<'a>` `DerefMut`s to this overlay, so a
    // `self.<port>.__cer_assign_<f>(&v)` call from the schema-blind macro
    // rewriter resolves through the deref chain onto these
    // methods for fixed fields (variable-field shims live on `<Name>Shm<'a>`
    // itself). Skip entirely for the empty (`_marker`) overlay — nothing to
    // assign — so no empty `impl` block is emitted.
    if has_fixed {
        writeln!(out, "impl {fixed_section_name} {{").unwrap();
        for field in fixed_section_fields(schema) {
            emit_fixed_assign_shim(out, field);
        }
        // Fixed-nested accessors (reached from `<Name>Shm`
        // through the DerefMut chain, same as the assign shims above).
        for field in fixed_section_fields(schema) {
            if matches!(&field.field_type, FieldType::Nested { fixed: Some(_), .. }) {
                emit_fixed_nested_accessors(out, field);
            }
        }
        writeln!(out, "}}").unwrap();
        writeln!(out).unwrap();
    }
}

/// Manual `Default` for `<Name>FixedSection` when derive is impossible
/// (direct large array) or wrong (non-zero scalar field defaults) —
/// mirrors the same path on `<Name>Shm` for fixed-only schemas.
fn generate_fixed_section_default_impl(
    out: &mut String,
    schema: &MessageSchema,
    fixed_section_name: &str,
) {
    writeln!(out, "impl Default for {fixed_section_name} {{").unwrap();
    writeln!(out, "    fn default() -> Self {{").unwrap();
    writeln!(out, "        Self {{").unwrap();
    for field in fixed_section_fields(schema) {
        let field_name = escape_keyword(&field.name);
        let default_value = shm_field_default_expr(field);
        writeln!(out, "            {field_name}: {default_value},").unwrap();
    }
    writeln!(out, "        }}").unwrap();
    writeln!(out, "    }}").unwrap();
    writeln!(out, "}}").unwrap();
    writeln!(out).unwrap();
}

/// Emit `Deref` / `DerefMut` for `<Name>Shm<'a>` targeting
/// `<Name>FixedSection`, plus a static assertion that the FixedSection's
/// alignment is `≤ 8` (so the post-WireHeader payload pointer — 8-aligned
/// on iceoryx2 0.9.1 via the 40 B @ align 8 per-sample header + 32-byte
/// WireHeader — is always sufficient).
fn generate_variable_shm_deref_impls(out: &mut String, shm_name: &str, fixed_section_name: &str) {
    // Static assertion: align(FixedSection) must be ≤ 8. The post-header
    // payload pointer (where `<Name>FixedSection` is overlaid) is 8-aligned
    // on iceoryx2 0.9.1: the per-sample header is 40 B @ align 8
    // (`IOX2_SAMPLE_HEADER_BYTES`), so a `[u8]` payload starts at chunk+40
    // of an 8-aligned chunk, and `WireHeader::SIZE == 32` (multiple of 8)
    // keeps `payload_base + 32` 8-aligned. That is DE FACTO (measured), not
    // a declared iceoryx2 contract — the rmw loan paths assert it
    // fail-closed before handing out a typed pointer.
    // If a future schema adds `u128`, vector intrinsics, or any other type
    // with `align_of > 8`, this assertion fails at compile-time and the
    // schema author must either reduce the alignment requirement, change the
    // SHM region alignment guarantee, or grow the WireHeader prefix.
    writeln!(
        out,
        "// Static check — fixed-section align ≤ 8 (matches post-WireHeader payload alignment)."
    )
    .unwrap();
    writeln!(
        out,
        "// If this fires for `{fixed_section_name}`, the schema introduced a field whose"
    )
    .unwrap();
    writeln!(
        out,
        "// alignment exceeds the 8-byte post-WireHeader guarantee. Likely cause: a `u128`,"
    )
    .unwrap();
    writeln!(
        out,
        "// SIMD vector, or `#[repr(align(N>8))]` member. Reduce alignment, change the"
    )
    .unwrap();
    writeln!(
        out,
        "// SHM region's alignment promise, or grow `WireHeader` past its current 32 bytes."
    )
    .unwrap();
    writeln!(out, "const _: () = assert!(").unwrap();
    writeln!(
        out,
        "    ::std::mem::align_of::<{fixed_section_name}>() <= 8,"
    )
    .unwrap();
    writeln!(
        out,
        "    \"align_of::<{fixed_section_name}>() exceeds 8; the post-WireHeader payload pointer is only 8-aligned. See codegen-side comment for remediation.\""
    )
    .unwrap();
    writeln!(out, ");").unwrap();
    writeln!(out).unwrap();

    writeln!(
        out,
        "/// Read fixed-section fields directly: `let h = view.height;`."
    )
    .unwrap();
    writeln!(out, "impl<'a> ::std::ops::Deref for {shm_name}<'a> {{").unwrap();
    writeln!(out, "    type Target = {fixed_section_name};").unwrap();
    writeln!(out, "    #[inline]").unwrap();
    writeln!(out, "    fn deref(&self) -> &{fixed_section_name} {{").unwrap();
    writeln!(
        out,
        "        // SAFETY: `self.ptr` was captured by `from_bytes`/`from_bytes_mut` from a"
    )
    .unwrap();
    writeln!(
        out,
        "        // borrow living for `'a`. Both constructors assert that the pointer is"
    )
    .unwrap();
    writeln!(
        out,
        "        // aligned to `align_of::<{fixed_section_name}>()` and that the buffer"
    )
    .unwrap();
    writeln!(
        out,
        "        // covers at least `OFFSET_TABLE_OFFSET + OFFSET_TABLE_BYTES` bytes (which"
    )
    .unwrap();
    writeln!(
        out,
        "        // includes `size_of::<{fixed_section_name}>()`), so the pointer cast yields"
    )
    .unwrap();
    writeln!(
        out,
        "        // a valid `&{fixed_section_name}`. The returned reference's lifetime is"
    )
    .unwrap();
    writeln!(
        out,
        "        // bounded by `&self` per the `Deref` trait signature; since `self: &Self<'a>`,"
    )
    .unwrap();
    writeln!(
        out,
        "        // that lifetime is at most `'a`. Aliasing is sound because `Reader<'a> =="
    )
    .unwrap();
    writeln!(
        out,
        "        // Writer<'a> = Self<'a>` and the borrow checker prevents two simultaneous"
    )
    .unwrap();
    writeln!(out, "        // `&mut Self` from coexisting.").unwrap();
    writeln!(
        out,
        "        unsafe {{ &*(self.ptr as *const {fixed_section_name}) }}"
    )
    .unwrap();
    writeln!(out, "    }}").unwrap();
    writeln!(out, "}}").unwrap();
    writeln!(out).unwrap();

    writeln!(
        out,
        "/// Write fixed-section fields directly: `proxy.height = 1080;`."
    )
    .unwrap();
    writeln!(out, "impl<'a> ::std::ops::DerefMut for {shm_name}<'a> {{").unwrap();
    writeln!(out, "    #[inline]").unwrap();
    writeln!(
        out,
        "    fn deref_mut(&mut self) -> &mut {fixed_section_name} {{"
    )
    .unwrap();
    writeln!(
        out,
        "        // SAFETY: see `deref` for size + alignment. Mutability is sound iff the"
    )
    .unwrap();
    writeln!(
        out,
        "        // underlying buffer is in fact mutable — guaranteed when the receiver was"
    )
    .unwrap();
    writeln!(
        out,
        "        // constructed via `from_bytes_mut(&'a mut [u8])`."
    )
    .unwrap();
    writeln!(out, "        //").unwrap();
    writeln!(
        out,
        "        // CONTRACT (NOT type-system enforced): a receiver"
    )
    .unwrap();
    writeln!(
        out,
        "        // constructed via `from_bytes(&'a [u8])` is read-only. Calling `deref_mut`"
    )
    .unwrap();
    writeln!(
        out,
        "        // on such a receiver writes through a `*mut u8` derived from a `&[u8]` —"
    )
    .unwrap();
    writeln!(
        out,
        "        // UB if the underlying memory is read-only (mmap RDONLY etc.). The transport"
    )
    .unwrap();
    writeln!(
        out,
        "        // layer enforces this by handing out only `&Self` through `InputView::deref`"
    )
    .unwrap();
    writeln!(
        out,
        "        // (no `DerefMut` on `InputView`); direct callers of `from_bytes` must"
    )
    .unwrap();
    writeln!(
        out,
        "        // observe the same discipline. Until `<Name>Shm` is split into"
    )
    .unwrap();
    writeln!(
        out,
        "        // separate Reader/Writer types, this contract lives in API documentation"
    )
    .unwrap();
    writeln!(out, "        // rather than the type system.").unwrap();
    writeln!(
        out,
        "        unsafe {{ &mut *(self.ptr as *mut {fixed_section_name}) }}"
    )
    .unwrap();
    writeln!(out, "    }}").unwrap();
    writeln!(out, "}}").unwrap();
    writeln!(out).unwrap();
}

/// Inherent-method + private-field + constant names that the
/// FixedSection's `pub <name>: <type>` field cannot collide with.
///
/// Why each entry is here:
/// - `from_bytes`, `from_bytes_mut`, `payload`, `payload_mut`, `cursor`,
///   `snapshot`, `write_from_snapshot`, `all_variables_written` — inherent
///   methods on `<Name>Shm<'a>` (in the impl block emitted alongside the
///   FixedSection). Rust resolves inherent methods before deref targets, so
///   `view.cursor` would call the method instead of reaching the FixedSection
///   field via Deref.
/// - `ptr`, `len`, `state`, `_phantom` — private fields of `<Name>Shm<'a>`.
///   `write_from_snapshot` codegen emits `self.<field> = snap.<field>` for
///   fixed-section fields. `self` here is `&mut <Name>Shm`, and Rust resolves
///   field access on the receiver type BEFORE the deref target. A user field
///   named `state` would route the LHS to `<Name>Shm.state: WriterState<N>`
///   and the RHS to `snap.state` (the user's typed snapshot field). Most
///   schema field types (e.g. `u32`, `bool`) produce a confusing
///   type-mismatch error pointing at codegen-internal lines instead of the
///   schema definition; in the rare case the user's field type happens to
///   coerce to `WriterState<N>` it would silently overwrite the writer's
///   cursor + bitset and corrupt the variable-write protocol. Either way,
///   the collision check turns it into a clear `compile_error!` naming the
///   offending schema field.
/// - `_marker` — private field on the empty-fixed-section variant of
///   `<Name>FixedSection` itself (collides at the FixedSection field-emission
///   level, not the Shm level).
/// - `WIRE_FIXED_SIZE`, `VARIABLE_FIELD_COUNT`, `OFFSET_TABLE_OFFSET`,
///   `OFFSET_TABLE_BYTES` — defensive: ROS2 / Cerulion field names are
///   snake_case lowercase and constants are SCREAMING_SNAKE, so collisions
///   are unlikely in practice. Listed as belt-and-suspenders.
///
/// MAINTENANCE (single source of truth): the reserved-name
/// checks are composed from THREE canonical lists below —
/// [`SHM_INHERENT_METHODS`] (methods emitted on `<Name>Shm<'a>`),
/// [`SHM_PRIVATE_FIELDS`] (private struct fields), and
/// [`SHM_ASSOC_CONSTS`] (associated constants). Fixed-section fields are
/// checked against ALL three via [`is_reserved_for_fixed_field`];
/// variable fields are checked against [`SHM_INHERENT_METHODS`] only
/// (their EMITTED accessor names must not duplicate an inherent method —
/// raw field names live in a different namespace than methods).
/// Future edits that add or rename inherent methods / private fields
/// must update exactly one list.
///
/// Private fields of `<Name>Shm<'a>` + the `_marker` field of the
/// empty-FixedSection variant. A fixed-section field with one of these
/// names breaks `write_from_snapshot` codegen: it emits `self.<field> =
/// snap.<field>`, and Rust resolves field access on the receiver type
/// BEFORE the deref target — the LHS would route to the Shm's private
/// field (e.g. `state: WriterState<N>`), producing either a confusing
/// type-mismatch in generated code or, if the types coincide, silent
/// writer-state corruption.
const SHM_PRIVATE_FIELDS: &[&str] = &[
    "ptr",
    "len",
    "state",
    "_phantom",
    // Private fields added for overflow-redirect.
    "overflow",
    "max_capacity",
    "topic",
    // Private field on the empty-fixed-section variant of <Name>FixedSection.
    "_marker",
];

/// Associated constants on `<Name>Shm<'a>`. Defensive: ROS2 / Cerulion
/// field names are snake_case and constants are SCREAMING_SNAKE, so
/// collisions are unlikely in practice.
const SHM_ASSOC_CONSTS: &[&str] = &[
    "WIRE_FIXED_SIZE",
    "VARIABLE_FIELD_COUNT",
    "OFFSET_TABLE_OFFSET",
    "OFFSET_TABLE_BYTES",
];

/// True iff a FIXED-SECTION field with this name would collide with the
/// generated `<Name>Shm<'a>` surface. Fixed-section fields are reached
/// via `Deref<Target = FixedSection>`, so they must avoid: inherent
/// METHODS (Rust resolves inherent members before Deref targets, so the
/// field would be unreachable / shadowed in emitted bodies), private
/// FIELDS (the `write_from_snapshot` LHS `self.<name> = …` resolves
/// receiver fields before the deref target — see
/// [`SHM_PRIVATE_FIELDS`]), and associated constants.
fn is_reserved_for_fixed_field(name: &str) -> bool {
    SHM_INHERENT_METHODS.contains(&name)
        || SHM_PRIVATE_FIELDS.contains(&name)
        || SHM_ASSOC_CONSTS.contains(&name)
}

/// Compute the set of schema field names that collide with reserved
/// accessors on `<Name>Shm<'a>`.
///
/// Two collision shapes are detected:
///
/// 1. **Fixed-section collisions** — a fixed-section field whose name shadows
///    an inherent method (e.g. `cursor`), private struct field (e.g. `state`),
///    constant, or per-variable-field accessor. Rust resolves inherent
///    members before `Deref` targets, so the FixedSection field would be
///    unreachable through `Deref<Target = FixedSection>` and would produce a
///    confusing type-mismatch or shadowing error in codegen-emitted bodies
///    (`write_from_snapshot`, `snapshot()`).
///
/// 2. **Variable-side collisions** — a *variable* field whose name shadows a
///    `<Name>Shm` inherent method or constant (e.g. a variable field named
///    `cursor` would emit BOTH the inherent `pub fn cursor(&self) -> u32`
///    AND the per-variable-field reader `pub fn cursor(&self) -> &str`,
///    producing a duplicate-method rustc error). A check that covers only
///    the fixed side misses this.
///
/// The single top-level `compile_error!` lists every offending name; the
/// schema-emitted code still contains the colliding member (cascading rustc
/// errors are noise but don't mask the actionable diagnostic).
fn collect_reserved_collisions(schema: &MessageSchema) -> Vec<String> {
    // Per-variable-field accessor names that may shadow OTHER (fixed) field
    // names. Used for the fixed-side check below.
    let mut variable_reserved: Vec<String> = Vec::new();
    for field in &schema.fields {
        if is_fixed_section_field(&field.field_type) {
            continue;
        }
        let raw = &field.name;
        if is_complex_variable(&field.field_type) {
            variable_reserved.push(format!("{raw}_bytes"));
            variable_reserved.push(format!("loan_{raw}_bytes"));
            variable_reserved.push(format!("set_{raw}_bytes"));
            // Complex variable also emits `fill_from_<f>_bytes`.
            variable_reserved.push(format!("fill_from_{raw}_bytes"));
        } else {
            // String / Bytes / DynamicArray<primitive>.
            variable_reserved.push(raw.clone());
            variable_reserved.push(format!("set_{raw}"));
            variable_reserved.push(format!("loan_{raw}"));
            variable_reserved.push(format!("push_{raw}"));
            // Simple variable also emits `fill_from_<f>`.
            variable_reserved.push(format!("fill_from_{raw}"));
        }
    }

    // Register the uniform `__cer_assign_<f>` write shim emitted for
    // EVERY field (fixed and variable) so a fixed-section field whose name
    // shadows another field's shim is caught here too. This is defense in
    // depth: the `__cer` prefix is independently rejected at codegen entry
    // (`emit_reserved_prefix_compile_error`), so no legitimate field name can
    // reach a `__cer_assign_*` string — but registering keeps the reserved
    // surface complete and the collision message accurate.
    for field in &schema.fields {
        variable_reserved.push(format!("__cer_assign_{}", field.name));
        if !is_fixed_section_field(&field.field_type) {
            // Variable fields additionally emit the uniform
            // `__cer_fill_from_<f>` shim.
            variable_reserved.push(format!("__cer_fill_from_{}", field.name));
        }
        // Fields with nested-writer sugar additionally emit
        // `with_<f>` (PUBLIC — the one non-`__cer` accessor, so a sibling
        // field named `with_<f>` must be caught here) plus the doc-hidden
        // `__cer_nested_<f>` / `__cer_with_nested_<f>` (and, for complex
        // targets, the private `__cer_staged_<f>` slot) — the `__cer`
        // prefix guard already blocks user fields on those, listed for
        // completeness.
        let has_nested_sugar =
            matches!(&field.field_type, FieldType::Nested { fixed: Some(_), .. })
                || (field.field_type.is_variable()
                    && complex_nested_target(&field.field_type).is_some());
        if has_nested_sugar {
            variable_reserved.push(format!("with_{}", field.name));
            variable_reserved.push(format!("__cer_nested_{}", field.name));
            variable_reserved.push(format!("__cer_with_nested_{}", field.name));
            variable_reserved.push(format!("__cer_staged_{}", field.name));
        }
    }

    let mut collisions = Vec::new();

    // Fixed-side: every fixed-section field name must NOT match the
    // reserved Shm surface (methods + private fields + constants) or any
    // variable-side accessor name.
    for field in fixed_section_fields(schema) {
        let name = &field.name;
        if is_reserved_for_fixed_field(name) || variable_reserved.contains(name) {
            collisions.push(name.clone());
        }
    }

    // Variable-side: a variable
    // field collides iff one of its EMITTED ACCESSOR METHOD names matches a
    // `<Name>Shm` inherent method name — that produces a duplicate-method
    // rustc error. The raw field name matching a PRIVATE FIELD (`state`,
    // `len`, `topic`, …) is NOT a collision for variable fields: variable
    // fields never become struct fields on `<Name>Shm` (only accessor
    // methods), and Rust resolves fields and methods in separate
    // namespaces. (moveit_msgs/DisplayRobotState has a field named `state`
    // — rejecting real upstream ROS2 field names would make the vendored
    // packages unbuildable.)
    for field in &schema.fields {
        if is_fixed_section_field(&field.field_type) {
            continue;
        }
        let raw = &field.name;
        let emitted: Vec<String> = if is_complex_variable(&field.field_type) {
            vec![
                format!("{raw}_bytes"),
                format!("set_{raw}_bytes"),
                format!("loan_{raw}_bytes"),
                format!("fill_from_{raw}_bytes"),
            ]
        } else {
            // String / Bytes / DynamicArray<primitive>. The bare reader is
            // the raw name; push_ is only emitted for array shapes but is
            // included unconditionally (false-positive-free: `push_<f>`
            // never matches an inherent method unless `<f>` is empty).
            vec![
                raw.clone(),
                format!("set_{raw}"),
                format!("loan_{raw}"),
                format!("push_{raw}"),
                format!("fill_from_{raw}"),
            ]
        };
        if emitted
            .iter()
            .any(|n| SHM_INHERENT_METHODS.contains(&n.as_str()))
            && !collisions.contains(raw)
        {
            collisions.push(raw.clone());
        }
    }

    collisions
}

/// Inherent method names emitted on `<Name>Shm<'a>` for variable schemas.
/// A variable field whose EMITTED accessor name matches one of these
/// produces a duplicate-method rustc error.
///
/// MAINTENANCE: must mirror the `pub fn` / private `fn` set emitted by
/// `generate_variable_shm_struct` and its helpers.
const SHM_INHERENT_METHODS: &[&str] = &[
    "from_bytes",
    "from_bytes_mut",
    "payload",
    "payload_mut",
    "cursor",
    "snapshot",
    "write_from_snapshot",
    "all_variables_written",
    "has_overflow",
    "overflow_view_bytes",
    "take_overflow",
    "ensure_capacity_for",
    "spill_to_overflow",
    // Staging surface (emitted on every variable Shm; the
    // `__cer` prefix guard blocks user fields independently — listed as
    // belt-and-suspenders like the constants).
    "__cer_staging_resume",
    "__cer_staging_export",
    "__cer_staging_take_overflow",
    "__cer_flush_staged",
    // Recursive staging persistence + the child publish gate
    // (same belt-and-suspenders rationale).
    "__cer_staging_take_children",
    "__cer_staging_restore_children",
    "__cer_first_unwritten_variable",
];

/// Emit ONE top-level `compile_error!` listing every colliding
/// fixed-section field name, plus the specific accessors each conflicts
/// with. Called from `generate_variable_shm_struct` AFTER the FixedSection
/// struct is emitted (with colliding fields already skipped) and BEFORE the
/// `<Name>Shm` impl block so the error appears at the top of the schema's
/// generated section.
///
/// Note on `format!` braces: the inner `{{` / `}}` pairs render as literal
/// `{` / `}` in the emitted Rust source's `compile_error!("...")` string —
/// they are NOT runtime formatter placeholders.
fn emit_reserved_collision_compile_error(
    out: &mut String,
    schema: &MessageSchema,
    names: &[String],
) {
    if names.is_empty() {
        return;
    }
    let names_list = names
        .iter()
        .map(|n| format!("`{n}`"))
        .collect::<Vec<_>>()
        .join(", ");
    writeln!(
        out,
        "// Reserved-name collision detected — see compile_error! below."
    )
    .unwrap();
    writeln!(
        out,
        "compile_error!(\"In schema `{schema_name}`, field(s) {names_list} collide with reserved accessor names on the generated SHM type. Rename the offending schema field(s). FIXED-SECTION fields cannot use inherent method names (`from_bytes`, `from_bytes_mut`, `payload`, `payload_mut`, `cursor`, `snapshot`, `write_from_snapshot`, `all_variables_written`, `has_overflow`, `overflow_view_bytes`, `take_overflow`, `ensure_capacity_for`, `spill_to_overflow`), private struct fields (`ptr`, `len`, `state`, `overflow`, `max_capacity`, `topic`, `_phantom`, `_marker`), associated constants (`WIRE_FIXED_SIZE`, `VARIABLE_FIELD_COUNT`, `OFFSET_TABLE_OFFSET`, `OFFSET_TABLE_BYTES`), or another field's accessor names (`<v>`, `set_<v>`, `loan_<v>`, `push_<v>`, `fill_from_<v>`, `__cer_assign_<v>`, `__cer_fill_from_<v>`, `with_<v>`, `__cer_nested_<v>`, `__cer_with_nested_<v>`, `<v>_bytes`, `set_<v>_bytes`, `loan_<v>_bytes`, `fill_from_<v>_bytes`). VARIABLE fields collide only when an emitted accessor name matches an inherent method (e.g. a String field named `cursor` or `snapshot`).\");",
        schema_name = schema.name,
        names_list = names_list,
    )
    .unwrap();
    writeln!(out).unwrap();
}

/// Emit `pub fn from_bytes(bytes: &'a [u8]) -> Self`.
fn generate_var_shm_from_bytes(
    out: &mut String,
    schema: &MessageSchema,
    shm_name: &str,
    fixed_section_name: &str,
) {
    writeln!(
        out,
        "    /// Borrow `bytes` (read-only) and construct a reader-side `{shm_name}`."
    )
    .unwrap();
    writeln!(out, "    ///").unwrap();
    writeln!(
        out,
        "    /// `bytes` must be the SHM payload *after* the 32-byte WireHeader."
    )
    .unwrap();
    writeln!(
        out,
        "    /// Mutating methods on the returned value are unsound when called via"
    )
    .unwrap();
    writeln!(
        out,
        "    /// this constructor — only call `&self` accessors. The transport layer"
    )
    .unwrap();
    writeln!(
        out,
        "    /// enforces this by handing out only `&Self` through `InputView::deref`."
    )
    .unwrap();
    writeln!(out, "    ///").unwrap();
    writeln!(out, "    /// # Panics").unwrap();
    writeln!(out, "    ///").unwrap();
    writeln!(
        out,
        "    /// - If `bytes.len() < Self::OFFSET_TABLE_OFFSET + Self::OFFSET_TABLE_BYTES`"
    )
    .unwrap();
    writeln!(
        out,
        "    ///   (i.e. the buffer cannot fit the fixed section + offset table)."
    )
    .unwrap();
    writeln!(
        out,
        "    /// - If `bytes.as_ptr()` is not aligned to `::std::mem::align_of::<{fixed_section_name}>()`."
    )
    .unwrap();
    writeln!(out, "    #[inline]").unwrap();
    writeln!(out, "    pub fn from_bytes(bytes: &'a [u8]) -> Self {{").unwrap();
    writeln!(
        out,
        "        let align = ::std::mem::align_of::<{fixed_section_name}>();"
    )
    .unwrap();
    writeln!(
        out,
        "        // Require the full prefix (fixed section + offset table)."
    )
    .unwrap();
    writeln!(
        out,
        "        // Variable-field readers index `payload[OFFSET_TABLE_OFFSET..table_end]`"
    )
    .unwrap();
    writeln!(
        out,
        "        // to consult the offset table; an undersized buffer would OOB on the"
    )
    .unwrap();
    writeln!(
        out,
        "        // first variable read with a confusing slice-index panic."
    )
    .unwrap();
    writeln!(
        out,
        "        let table_end = Self::OFFSET_TABLE_OFFSET + Self::OFFSET_TABLE_BYTES;"
    )
    .unwrap();
    writeln!(
        out,
        "        assert!(bytes.len() >= table_end, \"{shm_name}::from_bytes: buffer too small ({{}} < {{}}; need fixed section + offset table)\", bytes.len(), table_end);"
    )
    .unwrap();
    writeln!(
        out,
        "        assert!(bytes.as_ptr().align_offset(align) == 0, \"{shm_name}::from_bytes: misaligned source pointer (need align {{}})\", align);"
    )
    .unwrap();
    // The proxy fields below carry `#[deprecated]`; the
    // constructor's OWN struct-literal inits must stay warning-free.
    // `#[allow(deprecated)]` on the let statement is the tightest stable
    // scope covering the literal.
    writeln!(out, "        #[allow(deprecated)]").unwrap();
    writeln!(out, "        let __cer_shm = Self {{").unwrap();
    writeln!(
        out,
        "            // SAFETY: read-side construction; the resulting raw pointer is"
    )
    .unwrap();
    writeln!(
        out,
        "            // only ever read through `&self` accessors. Mutability is a"
    )
    .unwrap();
    writeln!(
        out,
        "            // public-API contract, not a type-system guarantee, because"
    )
    .unwrap();
    writeln!(
        out,
        "            // `Reader<'a> == Writer<'a>` is the same Rust type."
    )
    .unwrap();
    writeln!(out, "            ptr: bytes.as_ptr() as *mut u8,").unwrap();
    writeln!(out, "            len: bytes.len(),").unwrap();
    writeln!(out, "            state: ::cerulion_core::shm_runtime::WriterState::new(Self::WIRE_FIXED_SIZE as u32),").unwrap();
    // Reader-side never writes, so the overflow / max_capacity /
    // topic fields hold sentinel defaults — the writer-side accessors that
    // would consume them are gated behind `&mut self` and are unreachable
    // through `from_bytes` (which returns `Self` whose `&mut` method
    // surface is documented unsound per the panic doc above).
    writeln!(out, "            overflow: ::std::option::Option::None,").unwrap();
    // Setting `max_capacity: u32::MAX, topic: "<read-only>"` for misuse-error
    // attribution would let any reader-side misuse `&mut self` setter fall
    // through to `spill_to_overflow`, which attempts
    // `Vec<u64>::try_reserve_exact(u32::MAX/8) ≈ 4 GiB` —
    // a multi-second hang or OOM-kill. `max_capacity: 0` instead makes
    // misuse return immediately via `required_end > 0` → PayloadTooLarge
    // with no allocation attempt.
    //
    // The topic is an `Arc<str>`. Reader-side uses the
    // cached `read_only_topic()` sentinel — one `Arc::clone` per build_reader
    // (refcount bump, no allocation), preserving subscriber hot-path
    // zero-alloc invariants.
    writeln!(
        out,
        "            max_capacity: ::cerulion_core::wire::MaxPayloadCapacity::ZERO,"
    )
    .unwrap();
    writeln!(
        out,
        "            topic: ::cerulion_core::message::read_only_topic(),"
    )
    .unwrap();
    writeln!(out, "            _phantom: ::std::marker::PhantomData,").unwrap();
    // Zero-sized diagnostic proxies (one per variable field). Each is a
    // unit struct, so the field initializer is the type name itself.
    emit_proxy_field_inits(out, schema);
    // Staging slots start empty.
    emit_staged_field_inits(out, schema);
    writeln!(out, "        }};").unwrap();
    writeln!(out, "        __cer_shm").unwrap();
    writeln!(out, "    }}").unwrap();
    writeln!(out).unwrap();
}

/// Emit `pub fn from_bytes_mut(bytes: &'a mut [u8], max_capacity: MaxPayloadCapacity, topic: Arc<str>) -> Self`.
fn generate_var_shm_from_bytes_mut(
    out: &mut String,
    schema: &MessageSchema,
    shm_name: &str,
    fixed_section_name: &str,
) {
    writeln!(
        out,
        "    /// Borrow `bytes` mutably for direct in-place writes."
    )
    .unwrap();
    writeln!(out, "    ///").unwrap();
    writeln!(
        out,
        "    /// Initializes the offset table to zeros so reading an unwritten"
    )
    .unwrap();
    writeln!(
        out,
        "    /// variable field returns an empty slice rather than garbage."
    )
    .unwrap();
    writeln!(out, "    /// Cursor starts past the offset table.").unwrap();
    writeln!(out, "    ///").unwrap();
    writeln!(out, "    /// # Panics").unwrap();
    writeln!(out, "    ///").unwrap();
    writeln!(
        out,
        "    /// - If `bytes.len() < Self::OFFSET_TABLE_OFFSET + Self::OFFSET_TABLE_BYTES`"
    )
    .unwrap();
    writeln!(
        out,
        "    ///   (i.e. the buffer cannot fit the fixed section + offset table)."
    )
    .unwrap();
    writeln!(
        out,
        "    /// - If `bytes.as_mut_ptr()` is not aligned to `::std::mem::align_of::<{fixed_section_name}>()`."
    )
    .unwrap();
    writeln!(out, "    /// # Overflow-spill parameters").unwrap();
    writeln!(out, "    ///").unwrap();
    writeln!(
        out,
        "    /// - `max_capacity`: the publisher's configured `max_slice_len`"
    )
    .unwrap();
    writeln!(
        out,
        "    ///   minus `WireHeader::SIZE` (i.e. the absolute ceiling on the"
    )
    .unwrap();
    writeln!(
        out,
        "    ///   post-header payload). Setters use it to distinguish"
    )
    .unwrap();
    writeln!(out, "    ///   recoverable overflow (spill to heap) from").unwrap();
    writeln!(out, "    ///   `PayloadTooLarge` (no rescue possible).").unwrap();
    writeln!(
        out,
        "    /// - `topic`: the publisher's topic name, borrowed for the"
    )
    .unwrap();
    writeln!(
        out,
        "    ///   writer's lifetime. Used to attribute overflow errors."
    )
    .unwrap();
    writeln!(out, "    #[inline]").unwrap();
    writeln!(
        out,
        "    pub fn from_bytes_mut(bytes: &'a mut [u8], max_capacity: ::cerulion_core::wire::MaxPayloadCapacity, topic: ::std::sync::Arc<str>) -> Self {{"
    )
    .unwrap();
    writeln!(
        out,
        "        let align = ::std::mem::align_of::<{fixed_section_name}>();"
    )
    .unwrap();
    writeln!(
        out,
        "        // The buffer must hold the fixed section AND the offset table."
    )
    .unwrap();
    writeln!(
        out,
        "        // An earlier cut asserted only the fixed-section size,"
    )
    .unwrap();
    writeln!(
        out,
        "        // and the offset-table zeroing was conditional. A buffer between"
    )
    .unwrap();
    writeln!(
        out,
        "        // `WIRE_FIXED_SIZE` and `WIRE_FIXED_SIZE + 8 * VARIABLE_FIELD_COUNT` would"
    )
    .unwrap();
    writeln!(
        out,
        "        // pass the assert, then `WriterState::new(WIRE_FIXED_SIZE)` would set the"
    )
    .unwrap();
    writeln!(
        out,
        "        // cursor at the *end* of the buffer; the first variable-field write"
    )
    .unwrap();
    writeln!(
        out,
        "        // (`loan_<f>` → `write_offset_entry`) would OOB-panic on the offset-table"
    )
    .unwrap();
    writeln!(
        out,
        "        // slice index instead of returning a clean `ProxyBufferTooSmall`, so"
    )
    .unwrap();
    writeln!(out, "        // this check requires the full prefix").unwrap();
    writeln!(out, "        // (fixed section + offset table) up front.").unwrap();
    writeln!(
        out,
        "        let table_end = Self::OFFSET_TABLE_OFFSET + Self::OFFSET_TABLE_BYTES;"
    )
    .unwrap();
    writeln!(
        out,
        "        assert!(bytes.len() >= table_end, \"{shm_name}::from_bytes_mut: buffer too small ({{}} < {{}}; need fixed section + offset table)\", bytes.len(), table_end);"
    )
    .unwrap();
    writeln!(
        out,
        "        assert!(bytes.as_mut_ptr().align_offset(align) == 0, \"{shm_name}::from_bytes_mut: misaligned source pointer (need align {{}})\", align);"
    )
    .unwrap();
    writeln!(
        out,
        "        // Zero the offset-table region so partial-write reads are well-defined."
    )
    .unwrap();
    writeln!(
        out,
        "        // The size assert above guarantees the slice covers `table_end` bytes."
    )
    .unwrap();
    writeln!(
        out,
        "        for b in &mut bytes[Self::OFFSET_TABLE_OFFSET..table_end] {{"
    )
    .unwrap();
    writeln!(out, "            *b = 0;").unwrap();
    writeln!(out, "        }}").unwrap();
    writeln!(out, "        let len = bytes.len();").unwrap();
    writeln!(out, "        let ptr = bytes.as_mut_ptr();").unwrap();
    writeln!(
        out,
        "        // Suppress unused-variable warning on the original borrow — we"
    )
    .unwrap();
    writeln!(
        out,
        "        // intentionally drop it to take the raw pointer."
    )
    .unwrap();
    writeln!(out, "        let _ = bytes;").unwrap();
    writeln!(
        out,
        "        let _ = stringify!({shm_name}); // doc anchor only"
    )
    .unwrap();
    // See from_bytes — allow the deprecated proxy-field inits at
    // the let-statement scope.
    writeln!(out, "        #[allow(deprecated)]").unwrap();
    writeln!(out, "        let __cer_shm = Self {{").unwrap();
    writeln!(out, "            ptr,").unwrap();
    writeln!(out, "            len,").unwrap();
    writeln!(out, "            state: ::cerulion_core::shm_runtime::WriterState::new(Self::WIRE_FIXED_SIZE as u32),").unwrap();
    writeln!(out, "            overflow: ::std::option::Option::None,").unwrap();
    writeln!(out, "            max_capacity,").unwrap();
    writeln!(out, "            topic,").unwrap();
    writeln!(out, "            _phantom: ::std::marker::PhantomData,").unwrap();
    // Zero-sized diagnostic proxies (one per variable field).
    emit_proxy_field_inits(out, schema);
    // Staging slots start empty.
    emit_staged_field_inits(out, schema);
    writeln!(out, "        }};").unwrap();
    writeln!(out, "        __cer_shm").unwrap();
    writeln!(out, "    }}").unwrap();
    writeln!(out).unwrap();

    // The resume-state constructor the STAGING accessors of an
    // EMBEDDING schema use to rebuild this type's view over persisted scratch
    // WITHOUT resetting WriterState (from_bytes_mut always resets — its
    // semantics are untouched; this doc-hidden sibling exists instead).
    generate_var_shm_staging_resume(out, schema, shm_name, fixed_section_name);
}

/// Emit private `payload` / `payload_mut` helpers.
fn generate_var_shm_payload_helpers(out: &mut String) {
    writeln!(out, "    /// Internal: borrow the SHM payload immutably.").unwrap();
    writeln!(out, "    ///").unwrap();
    writeln!(
        out,
        "    /// SAFETY: `ptr + len` came from a `&[u8]` or `&mut [u8]` valid for `'a`."
    )
    .unwrap();
    writeln!(out, "    /// After a spill, `ptr` is repointed to the heap").unwrap();
    writeln!(
        out,
        "    /// `overflow` buffer and `len` is the spill capacity; existing"
    )
    .unwrap();
    writeln!(
        out,
        "    /// `cursor + bytes_needed > self.len` checks naturally guard against"
    )
    .unwrap();
    writeln!(
        out,
        "    /// the absolute ceiling without needing a second branch site."
    )
    .unwrap();
    writeln!(out, "    #[inline]").unwrap();
    writeln!(out, "    fn payload(&self) -> &[u8] {{").unwrap();
    writeln!(
        out,
        "        unsafe {{ ::std::slice::from_raw_parts(self.ptr, self.len) }}"
    )
    .unwrap();
    writeln!(out, "    }}").unwrap();
    writeln!(out).unwrap();
    writeln!(out, "    /// Internal: borrow the SHM payload mutably.").unwrap();
    writeln!(out, "    ///").unwrap();
    writeln!(
        out,
        "    /// SAFETY: caller must ensure this `Self` was constructed via"
    )
    .unwrap();
    writeln!(
        out,
        "    /// `from_bytes_mut` (i.e., the underlying slice was originally `&mut [u8]`)."
    )
    .unwrap();
    writeln!(out, "    #[inline]").unwrap();
    writeln!(out, "    fn payload_mut(&mut self) -> &mut [u8] {{").unwrap();
    writeln!(
        out,
        "        unsafe {{ ::std::slice::from_raw_parts_mut(self.ptr, self.len) }}"
    )
    .unwrap();
    writeln!(out, "    }}").unwrap();
    writeln!(out).unwrap();

    // `ensure_capacity_for`: the spill-or-fail helper. Called from
    // every setter BEFORE state mutation. Returns Ok with no side effect
    // when the current loan fits; spills to a heap `Box<[u64]>` (8-byte
    // aligned, fixed-length) when overflow is recoverable; returns
    // PayloadTooLarge or AllocationFailed when the ceiling is hit.
    writeln!(
        out,
        "    /// Ensure the loan (current `self.len`) can absorb"
    )
    .unwrap();
    writeln!(
        out,
        "    /// `bytes_needed` bytes starting at `cursor_required`."
    )
    .unwrap();
    writeln!(out, "    ///").unwrap();
    writeln!(
        out,
        "    /// Fast path (steady state): `cursor_required + bytes_needed <="
    )
    .unwrap();
    writeln!(out, "    /// self.len` — no-op, returns `Ok(())`.").unwrap();
    writeln!(out, "    ///").unwrap();
    writeln!(
        out,
        "    /// Overflow path: spill to a heap `Box<[u64]>` (8-byte aligned,"
    )
    .unwrap();
    writeln!(
        out,
        "    /// for sound multi-byte typed loans), copy `payload[0..cursor]`"
    )
    .unwrap();
    writeln!(
        out,
        "    /// into it (preserves fixed section + offset table + any prior"
    )
    .unwrap();
    writeln!(
        out,
        "    /// variable writes), repoint `self.ptr` / `self.len` so every"
    )
    .unwrap();
    writeln!(
        out,
        "    /// existing setter site naturally writes into the heap from"
    )
    .unwrap();
    writeln!(
        out,
        "    /// this point onwards. `OutputProxy::Drop` consults"
    )
    .unwrap();
    writeln!(
        out,
        "    /// `has_overflow()` to re-loan a fresh SHM sample sized to fit"
    )
    .unwrap();
    writeln!(out, "    /// and memcpy the bytes in.").unwrap();
    writeln!(out, "    ///").unwrap();
    writeln!(
        out,
        "    /// Error paths: `PayloadTooLarge` if the required end exceeds"
    )
    .unwrap();
    writeln!(
        out,
        "    /// `max_capacity` (no rescue possible); `AllocationFailed` if"
    )
    .unwrap();
    writeln!(out, "    /// `Vec::try_reserve_exact` fails.").unwrap();
    writeln!(out, "    ///").unwrap();
    writeln!(out, "    /// # Panic surface").unwrap();
    writeln!(out, "    ///").unwrap();
    writeln!(
        out,
        "    /// `Vec::into_boxed_slice()` (called after the fallible"
    )
    .unwrap();
    writeln!(
        out,
        "    /// `try_reserve_exact + resize` chain) uses the INFALLIBLE"
    )
    .unwrap();
    writeln!(
        out,
        "    /// allocator path internally. If the allocator over-provisioned"
    )
    .unwrap();
    writeln!(
        out,
        "    /// the initial reservation (e.g. glibc rounding to size-class)"
    )
    .unwrap();
    writeln!(
        out,
        "    /// AND the subsequent shrink-realloc fails, the process aborts"
    )
    .unwrap();
    writeln!(
        out,
        "    /// via `handle_alloc_error`. This is the SAME class of risk"
    )
    .unwrap();
    writeln!(out, "    /// protected against on the publish path, but").unwrap();
    writeln!(
        out,
        "    /// stable Rust lacks a fallible `Box<[T]>` constructor."
    )
    .unwrap();
    writeln!(out, "    ///").unwrap();
    writeln!(
        out,
        "    /// Bounded by: the path is `#[cold]`, runs in the setter call"
    )
    .unwrap();
    writeln!(out, "    /// (NOT Drop — so unwind-safe), and requires").unwrap();
    writeln!(
        out,
        "    /// the system to already be in OOM for the shrink-realloc"
    )
    .unwrap();
    writeln!(
        out,
        "    /// to fail. See the in-line construction comment for the full"
    )
    .unwrap();
    writeln!(out, "    /// rationale + remediation options.").unwrap();
    writeln!(out, "    #[cold]").unwrap();
    writeln!(out, "    #[inline(never)]").unwrap();
    writeln!(
        out,
        "    fn spill_to_overflow(&mut self, required_end: usize) -> ::std::result::Result<(), ::cerulion_core::TransportError> {{"
    )
    .unwrap();
    // In-tick allocation-failure fault-injection.
    // Module path is `::cerulion_core::spill_fault_injection`
    // (always-on — kept out of the feature-gated `testing` module so
    // codegen run from crates that don't enable cerulion_core's
    // `test-helpers` feature still finds the symbol; this was the
    // CI canary that caught the original gated-path bug). The CALL
    // SITE here is still cfg-gated behind `any(test, feature =
    // "test-helpers")` so non-test builds of the calling crate omit
    // the check entirely.
    writeln!(out, "        #[cfg(any(test, feature = \"test-helpers\"))]").unwrap();
    writeln!(
        out,
        "        if ::cerulion_core::spill_fault_injection::armed_and_consume() {{"
    )
    .unwrap();
    writeln!(out, "            return ::std::result::Result::Err(::cerulion_core::TransportError::AllocationFailed {{").unwrap();
    writeln!(
        out,
        "                topic: ::std::string::String::from(::std::convert::AsRef::<str>::as_ref(&self.topic)),"
    )
    .unwrap();
    writeln!(
        out,
        "                requested: self.max_capacity.get() as usize,"
    )
    .unwrap();
    writeln!(out, "            }});").unwrap();
    writeln!(out, "        }}").unwrap();
    writeln!(
        out,
        "        // Allocate the 8-byte-aligned spill buffer via `Vec<u64>`,"
    )
    .unwrap();
    writeln!(
        out,
        "        // then narrow to `Box<[u64]>` (drops the `usize` capacity field"
    )
    .unwrap();
    writeln!(
        out,
        "        // since the buffer is fixed-length; alignment of u64 is the strictest"
    )
    .unwrap();
    writeln!(
        out,
        "        // any primitive setter (f64/u64/i64) requires). `try_reserve_exact`"
    )
    .unwrap();
    writeln!(
        out,
        "        // is fallible so OOM surfaces as `AllocationFailed` instead of aborting."
    )
    .unwrap();
    writeln!(
        out,
        "        let max_cap = self.max_capacity.get() as usize;"
    )
    .unwrap();
    // Allocate the full `max_cap` for the spill buffer so
    // subsequent setters in the same tick can write up to the ceiling
    // (e.g. multi-field-spill cases where each field fits alone but
    // together approach max_cap). An earlier design considered
    // `required_end`-only allocation to save heap (250x reduction for
    // small spikes), but it forces multi-field-spill cases into
    // PayloadTooLarge even when total payload fits in max_cap.
    //
    // Trade-off acknowledged: a single small overflow on a 128 MiB
    // ceiling allocates 128 MiB even if the spike was 64 KiB (this is a
    // real HEAP spill on the cold overflow path, not the lazy SHM pool —
    // the tier bump raises this cold-path ceiling too). The
    // alternative is multi-spill within a tick, which the
    // `debug_assert!(self.overflow.is_none())` invariant forbids by
    // design (single-spill keeps the writer state simpler).
    //
    // The `required_end` parameter is pinned by the
    // `debug_assert!(required_end <= max_cap)` below; it does not size
    // the allocation, which is always the full `max_cap`.
    writeln!(out, "        let u64_count = max_cap.div_ceil(8);").unwrap();
    // Build the spill into a `Vec<u64>` with `try_reserve_exact` (fallible
    // — surfaces `AllocationFailed` rather than process-aborting under
    // OOM) and then convert to `Box<[u64]>` via `.into_boxed_slice()`.
    // The Box form drops the unused `usize` capacity field, narrows the
    // type to "this slice has fixed length", and prevents accidental
    // mutation of the buffer (Vec exposed `push`/`resize`/etc.).
    //
    // INFALLIBLE-ALLOC HAZARD:
    // `Vec::into_boxed_slice()` internally calls `Vec::shrink_to_fit()`
    // which uses the INFALLIBLE allocator path (`alloc::realloc` →
    // `handle_alloc_error` on Err). If the allocator over-provisioned the
    // initial `try_reserve_exact(u64_count)` (e.g., glibc rounding up to
    // a size-class), the shrink-realloc on the conversion may fail and
    // process-abort. The publish path is already protected against this
    // same allocator behaviour, but this path cannot replace
    // `into_boxed_slice` with a fallible equivalent on stable Rust.
    //
    // Risk envelope:
    // 1. This is the `#[cold] #[inline(never)]` overflow path — not the
    //    steady-state hot path.
    // 2. The path runs during the user's setter call, NOT during Drop —
    //    a panic here unwinds normally (Drop-no-panic invariant
    //    preserved).
    // 3. The shrink-realloc only happens if the allocator over-provisioned
    //    the initial reservation. `try_reserve_exact` *requests* exact
    //    capacity, but the std docs explicitly permit over-provisioning
    //    ("capacity will be greater than or equal to self.len() +
    //    additional"). In practice, jemalloc/mimalloc respect exact
    //    requests at the std::Vec layer; glibc may round up.
    // 4. A failed shrink-realloc requires the system to be already in OOM
    //    — extremely degraded state. Aborting from a setter call is a
    //    reasonable failure mode in that envelope; the alternative would
    //    be a complex `Box::try_new_uninit_slice` dance (nightly-only) or
    //    storing `Vec<u64>` directly (loses the type-narrowing of #4).
    //
    // If this becomes a practical concern (e.g., observed under
    // memory-pressure soak), revisit with one of:
    // - Replace `into_boxed_slice()` with a custom `Box<[T]>` construction
    //   via the std::alloc API + `Box::from_raw` (unsafe but fallible).
    // - Revert to `Vec<u64>` and document the type-narrowing loss.
    // - Wait for `Box::try_new_uninit_slice` to stabilize.
    writeln!(
        out,
        "        let mut spill: ::std::vec::Vec<u64> = ::std::vec::Vec::new();"
    )
    .unwrap();
    writeln!(
        out,
        "        if spill.try_reserve_exact(u64_count).is_err() {{"
    )
    .unwrap();
    writeln!(out, "            return ::std::result::Result::Err(::cerulion_core::TransportError::AllocationFailed {{").unwrap();
    writeln!(
        out,
        "                topic: ::std::string::String::from(::std::convert::AsRef::<str>::as_ref(&self.topic)),"
    )
    .unwrap();
    writeln!(out, "                requested: max_cap,").unwrap();
    writeln!(out, "            }});").unwrap();
    writeln!(out, "        }}").unwrap();
    writeln!(out, "        spill.resize(u64_count, 0u64);").unwrap();
    writeln!(
        out,
        "        let mut spill: ::std::boxed::Box<[u64]> = spill.into_boxed_slice();"
    )
    .unwrap();
    writeln!(
        out,
        "        // The spill buffer is 8-byte aligned; reinterpret as &mut [u8] of"
    )
    .unwrap();
    writeln!(
        out,
        "        // length `max_cap` (drops trailing padding from u64-round-up)."
    )
    .unwrap();
    writeln!(
        out,
        "        let spill_ptr = spill.as_mut_ptr() as *mut u8;"
    )
    .unwrap();
    writeln!(
        out,
        "        // Copy bytes already written (fixed section + offset table +"
    )
    .unwrap();
    writeln!(
        out,
        "        // any variable payload up to the current cursor) from the loan"
    )
    .unwrap();
    writeln!(
        out,
        "        // into the spill so existing offset-table entries + fixed"
    )
    .unwrap();
    writeln!(
        out,
        "        // fields remain at the same cursor positions."
    )
    .unwrap();
    writeln!(
        out,
        "        let written_so_far = self.state.cursor as usize;"
    )
    .unwrap();
    // Use required_end to pin the spill-fits-current-write
    // invariant. required_end <= max_cap is guaranteed by the caller
    // (ensure_capacity_for checks first).
    writeln!(out, "        debug_assert!(required_end <= max_cap,").unwrap();
    writeln!(
        out,
        "            \"spill_to_overflow: required_end {{}} > max_capacity {{}} — caller invariant violated\","
    )
    .unwrap();
    writeln!(out, "            required_end, max_cap);").unwrap();
    writeln!(out, "        debug_assert!(written_so_far <= self.len,").unwrap();
    writeln!(out, "            \"spill_to_overflow: cursor {{}} exceeds current len {{}} (invariant violated)\",").unwrap();
    writeln!(out, "            written_so_far, self.len);").unwrap();
    writeln!(
        out,
        "        // SAFETY: source `self.ptr..self.ptr + written_so_far` is the"
    )
    .unwrap();
    writeln!(
        out,
        "        // initialised prefix (header + offset table + variable writes)."
    )
    .unwrap();
    writeln!(
        out,
        "        // Destination `spill_ptr..spill_ptr + written_so_far` is"
    )
    .unwrap();
    writeln!(
        out,
        "        // freshly-allocated zeroed bytes; alignment is u64 (>= u8)."
    )
    .unwrap();
    writeln!(
        out,
        "        // Regions are disjoint heaps (loan vs spill)."
    )
    .unwrap();
    writeln!(out, "        unsafe {{").unwrap();
    writeln!(
        out,
        "            ::std::ptr::copy_nonoverlapping(self.ptr, spill_ptr, written_so_far);"
    )
    .unwrap();
    writeln!(out, "        }}").unwrap();
    writeln!(
        out,
        "        // Move the spill into `self.overflow`. `Box<[u64]>`'s heap ptr"
    )
    .unwrap();
    writeln!(
        out,
        "        // is stable across moves; `spill_ptr` (captured BEFORE the move)"
    )
    .unwrap();
    writeln!(out, "        // remains valid.").unwrap();
    writeln!(
        out,
        "        self.overflow = ::std::option::Option::Some(spill);"
    )
    .unwrap();
    writeln!(out, "        self.ptr = spill_ptr;").unwrap();
    writeln!(out, "        self.len = max_cap;").unwrap();
    writeln!(out, "        ::std::result::Result::Ok(())").unwrap();
    writeln!(out, "    }}").unwrap();
    writeln!(out).unwrap();
    writeln!(out, "    /// Capacity check + spill orchestration.").unwrap();
    writeln!(out, "    #[inline]").unwrap();
    writeln!(
        out,
        "    fn ensure_capacity_for(&mut self, bytes_needed: usize, cursor_required: usize) -> ::std::result::Result<(), ::cerulion_core::TransportError> {{"
    )
    .unwrap();
    writeln!(
        out,
        "        let required_end = cursor_required.saturating_add(bytes_needed);"
    )
    .unwrap();
    writeln!(out, "        if required_end <= self.len {{").unwrap();
    writeln!(out, "            return ::std::result::Result::Ok(());").unwrap();
    writeln!(out, "        }}").unwrap();
    writeln!(
        out,
        "        let max_cap = self.max_capacity.get() as usize;"
    )
    .unwrap();
    writeln!(out, "        if required_end > max_cap {{").unwrap();
    writeln!(out, "            return ::std::result::Result::Err(::cerulion_core::TransportError::PayloadTooLarge {{").unwrap();
    writeln!(
        out,
        "                topic: ::std::string::String::from(::std::convert::AsRef::<str>::as_ref(&self.topic)),"
    )
    .unwrap();
    writeln!(out, "                requested: required_end,").unwrap();
    writeln!(out, "                max: max_cap,").unwrap();
    writeln!(out, "            }});").unwrap();
    writeln!(out, "        }}").unwrap();
    writeln!(
        out,
        "        // If we already spilled, `self.len == max_cap`; the check above"
    )
    .unwrap();
    writeln!(
        out,
        "        // would have either returned Ok (fits in spill) or PayloadTooLarge."
    )
    .unwrap();
    writeln!(
        out,
        "        // So reaching this point means we are still on the original loan."
    )
    .unwrap();
    writeln!(out, "        debug_assert!(self.overflow.is_none(),").unwrap();
    writeln!(
        out,
        "            \"ensure_capacity_for: reached spill path with overflow already active\");"
    )
    .unwrap();
    writeln!(out, "        self.spill_to_overflow(required_end)").unwrap();
    writeln!(out, "    }}").unwrap();
    writeln!(out).unwrap();

    // Public accessors for OutputProxy::Drop.
    writeln!(
        out,
        "    /// True iff this writer spilled to a heap fallback"
    )
    .unwrap();
    writeln!(out, "    /// buffer this tick.").unwrap();
    writeln!(out, "    #[inline]").unwrap();
    writeln!(out, "    pub fn has_overflow(&self) -> bool {{").unwrap();
    writeln!(out, "        self.overflow.is_some()").unwrap();
    writeln!(out, "    }}").unwrap();
    writeln!(out).unwrap();
    writeln!(out, "    /// View of the spill bytes `[0..cursor]` for").unwrap();
    writeln!(out, "    /// `OutputProxy::Drop`'s re-loan + memcpy path.").unwrap();
    writeln!(
        out,
        "    /// Returns `None` if no spill occurred this tick."
    )
    .unwrap();
    writeln!(out, "    #[inline]").unwrap();
    writeln!(
        out,
        "    pub fn overflow_view_bytes(&self) -> ::std::option::Option<&[u8]> {{"
    )
    .unwrap();
    // Clippy `question_mark` lint expects the early-return pattern to use
    // `?` — `self.overflow.as_ref()?;` returns None directly when the
    // option is None, otherwise continues with the cursor read below.
    writeln!(out, "        self.overflow.as_ref()?;").unwrap();
    writeln!(out, "        let cursor = self.state.cursor as usize;").unwrap();
    writeln!(
        out,
        "        ::std::option::Option::Some(&self.payload()[..cursor])"
    )
    .unwrap();
    writeln!(out, "    }}").unwrap();
    writeln!(out).unwrap();
    writeln!(
        out,
        "    /// **TEST-ONLY.** Take the heap spill buffer out of the writer."
    )
    .unwrap();
    writeln!(out, "    ///").unwrap();
    writeln!(
        out,
        "    /// `OutputProxy::Drop` does NOT call this — it uses"
    )
    .unwrap();
    writeln!(
        out,
        "    /// `T::overflow_view_bytes` (borrowing). The Vec is dropped"
    )
    .unwrap();
    writeln!(
        out,
        "    /// with the writer at the end of Drop's body. This accessor"
    )
    .unwrap();
    writeln!(
        out,
        "    /// exists only for integration-test introspection (verifying"
    )
    .unwrap();
    writeln!(
        out,
        "    /// the Vec's length + alignment). Calling it invalidates"
    )
    .unwrap();
    writeln!(
        out,
        "    /// `self.ptr` / `self.len`; any subsequent `payload()` /"
    )
    .unwrap();
    writeln!(out, "    /// `payload_mut()` is UB.").unwrap();
    // Previously `pub`, exposing a documented UB hatch (after take,
    // self.ptr is dangling). Production code uses `overflow_view_bytes`
    // (which borrows). Gated to test/`test-helpers` feature so the UB
    // surface isn't exposed to downstream users.
    writeln!(out, "    #[cfg(any(test, feature = \"test-helpers\"))]").unwrap();
    writeln!(out, "    #[inline]").unwrap();
    writeln!(
        out,
        "    pub fn take_overflow(&mut self) -> ::std::option::Option<::std::boxed::Box<[u64]>> {{"
    )
    .unwrap();
    writeln!(out, "        self.overflow.take()").unwrap();
    writeln!(out, "    }}").unwrap();
    writeln!(out).unwrap();
}

/// Helper: return `(byte_size, rust_type_name)` for a primitive field type.
fn primitive_size_and_type(ft: &FieldType) -> (usize, &'static str) {
    match ft {
        FieldType::Bool => (1, "bool"),
        FieldType::I8 => (1, "i8"),
        FieldType::U8 => (1, "u8"),
        FieldType::I16 => (2, "i16"),
        FieldType::U16 => (2, "u16"),
        FieldType::I32 => (4, "i32"),
        FieldType::U32 => (4, "u32"),
        FieldType::I64 => (8, "i64"),
        FieldType::U64 => (8, "u64"),
        FieldType::F32 => (4, "f32"),
        FieldType::F64 => (8, "f64"),
        _ => panic!("primitive_size_and_type called on non-primitive"),
    }
}

/// Emit reader + setter + loan + push accessors for a variable field.
///
/// Naming convention:
/// - The reader uses the keyword-escaped name (so `image.r#type()` works
///   when the field is named `type`).
/// - The setter, loan, and push functions use the raw name (`set_type`,
///   `loan_type`, `push_type` — these are all valid identifiers because
///   the `set_/loan_/push_` prefix neutralises the keyword).
fn emit_variable_field_accessors(out: &mut String, field: &FieldDef, var_idx: usize) {
    let field_name = escape_keyword(&field.name);
    let raw_name = &field.name;
    let ft = &field.field_type;

    // The uniform `__cer_assign_<f>` write shim for this variable field.
    // Delegates to the existing typed `set_<f>` / `set_<f>_bytes` accessor so
    // the schema-blind macro rewriter can emit one call shape for fixed AND
    // variable fields. Emitted first; method order within an impl is
    // irrelevant, and the `set_*` targets are defined below in the same block.
    emit_variable_assign_shim(out, field);
    // The uniform `__cer_fill_from_<f>` shim — same reasoning
    // for `self.<port>.<f>.fill_from(src)`: the schema-blind rewriter cannot
    // pick `fill_from_<f>` vs `fill_from_<f>_bytes`, so codegen resolves the
    // simple-vs-complex target here.
    emit_variable_fill_from_shim(out, field);

    if is_complex_variable(ft) {
        // A complex-NESTED field (`Nested { fixed: None }`)
        // carries a `__cer_staged_<f>` staging slot for the leaf-write sugar.
        // A whole-field write while that staging is in flight is the
        // "staged then whole" conflict (guard A) — reject it loudly. Other
        // complex-variable shapes (e.g. `DynArray<Nested>`) have no staging
        // slot, so `has_staged` is false and no guard (or `self.__cer_staged_*`
        // reference) is emitted for them.
        let has_staged = complex_nested_target(ft).is_some();
        // Raw-bytes API only — typed wrapping deferred to macro chunk.
        writeln!(out, "    /// Raw-bytes reader for variable field `{raw_name}` (complex type — current limitation).").unwrap();
        writeln!(out, "    ///").unwrap();
        writeln!(
            out,
            "    /// Returns the serialized bytes of the field as written by the publisher."
        )
        .unwrap();
        writeln!(
            out,
            "    /// The macro layer wraps this in a typed accessor."
        )
        .unwrap();
        writeln!(out, "    #[inline]").unwrap();
        writeln!(out, "    pub fn {raw_name}_bytes(&self) -> &[u8] {{").unwrap();
        writeln!(out, "        let payload = self.payload();").unwrap();
        writeln!(out, "        let (off, len) = ::cerulion_core::shm_runtime::read_offset_entry(payload, Self::WIRE_FIXED_SIZE, {var_idx});").unwrap();
        writeln!(out, "        let off = off as usize;").unwrap();
        writeln!(out, "        let len = len as usize;").unwrap();
        writeln!(
            out,
            "        if off.saturating_add(len) > payload.len() {{ return &[]; }}"
        )
        .unwrap();
        writeln!(out, "        &payload[off..off+len]").unwrap();
        writeln!(out, "    }}").unwrap();
        writeln!(out).unwrap();

        writeln!(
            out,
            "    /// Reserve `n` bytes for variable field `{raw_name}` (complex type)."
        )
        .unwrap();
        writeln!(out, "    pub fn loan_{raw_name}_bytes(&mut self, n: usize) -> ::std::result::Result<&mut [u8], ::cerulion_core::TransportError> {{").unwrap();
        // Guard A: reject a whole-field write while staged leaf
        // writes are pending. The Drop-flush is IMMUNE by construction — it
        // `.take()`s the staging slot BEFORE calling `set_<f>_bytes` (which
        // routes here), so `__cer_staged_<f>` is already `None` at flush.
        // `set_<f>_bytes` (above) inherits this guard transitively.
        if has_staged {
            emit_staged_then_whole_conflict_guard(out, raw_name);
        }
        emit_loan_body(out, var_idx, "n");
        writeln!(out, "    }}").unwrap();
        writeln!(out).unwrap();

        writeln!(
            out,
            "    /// Write raw bytes into variable field `{raw_name}` (complex type)."
        )
        .unwrap();
        writeln!(out, "    pub fn set_{raw_name}_bytes(&mut self, value: &[u8]) -> ::std::result::Result<(), ::cerulion_core::TransportError> {{").unwrap();
        writeln!(
            out,
            "        let dst = self.loan_{raw_name}_bytes(value.len())?;"
        )
        .unwrap();
        writeln!(out, "        dst.copy_from_slice(value);").unwrap();
        writeln!(out, "        Ok(())").unwrap();
        writeln!(out, "    }}").unwrap();
        writeln!(out).unwrap();

        // `fill_from_<f>_bytes` for the complex variable (raw bytes).
        // `fill_from_<f>_bytes` does NOT route through
        // `loan_<f>_bytes` (it has its own cursor body), so guard A is
        // mirrored inside it via `has_staged`.
        emit_fill_from_method(out, raw_name, var_idx, "u8", 1, "_bytes", has_staged);
        return;
    }

    // Typed API for String / Bytes / DynamicArray<primitive>.
    match ft {
        FieldType::String => {
            writeln!(
                out,
                "    /// Read variable field `{raw_name}` as `&str` borrowed from SHM."
            )
            .unwrap();
            writeln!(out, "    ///").unwrap();
            writeln!(
                out,
                "    /// Returns `Err(WireError::InvalidUtf8)` if the slot's bytes are not"
            )
            .unwrap();
            writeln!(
                out,
                "    /// valid UTF-8 and `Err(WireError::InvalidOffset)` if the offset table"
            )
            .unwrap();
            writeln!(
                out,
                "    /// points past the payload (replaces the prior silent"
            )
            .unwrap();
            writeln!(out, "    /// fallback to `\"\"`).").unwrap();
            writeln!(out, "    #[inline]").unwrap();
            writeln!(out, "    pub fn {field_name}(&self) -> ::std::result::Result<&str, ::cerulion_core::wire::WireError> {{").unwrap();
            writeln!(out, "        let payload = self.payload();").unwrap();
            writeln!(out, "        let (off, len) = ::cerulion_core::shm_runtime::read_offset_entry(payload, Self::WIRE_FIXED_SIZE, {var_idx});").unwrap();
            writeln!(
                out,
                "        let off_u = off as usize; let len_u = len as usize;"
            )
            .unwrap();
            writeln!(
                out,
                "        if off_u.saturating_add(len_u) > payload.len() {{"
            )
            .unwrap();
            writeln!(
                out,
                "            return Err(::cerulion_core::wire::WireError::InvalidOffset {{"
            )
            .unwrap();
            writeln!(out, "                offset: off,").unwrap();
            writeln!(out, "                max: payload.len() as u32,").unwrap();
            writeln!(out, "            }});").unwrap();
            writeln!(out, "        }}").unwrap();
            writeln!(
                out,
                "        ::std::str::from_utf8(&payload[off_u..off_u+len_u])"
            )
            .unwrap();
            writeln!(
                out,
                "            .map_err(|_| ::cerulion_core::wire::WireError::InvalidUtf8 {{"
            )
            .unwrap();
            writeln!(out, "                offset: off,").unwrap();
            writeln!(out, "                length: len,").unwrap();
            writeln!(out, "            }})").unwrap();
            writeln!(out, "    }}").unwrap();
            writeln!(out).unwrap();

            writeln!(
                out,
                "    /// Reserve `n` bytes for variable field `{raw_name}` (String)."
            )
            .unwrap();
            writeln!(out, "    pub fn loan_{raw_name}(&mut self, n: usize) -> ::std::result::Result<&mut [u8], ::cerulion_core::TransportError> {{").unwrap();
            emit_loan_body(out, var_idx, "n");
            writeln!(out, "    }}").unwrap();
            writeln!(out).unwrap();

            writeln!(
                out,
                "    /// Copy `value` into variable field `{raw_name}` (String)."
            )
            .unwrap();
            writeln!(out, "    pub fn set_{raw_name}(&mut self, value: &str) -> ::std::result::Result<(), ::cerulion_core::TransportError> {{").unwrap();
            writeln!(out, "        let bytes = value.as_bytes();").unwrap();
            writeln!(out, "        let dst = self.loan_{raw_name}(bytes.len())?;").unwrap();
            writeln!(out, "        dst.copy_from_slice(bytes);").unwrap();
            writeln!(out, "        Ok(())").unwrap();
            writeln!(out, "    }}").unwrap();
            writeln!(out).unwrap();

            // `fill_from_<f>` hands the producer the raw &mut [u8]
            // dst of the loaned region. The producer is responsible for
            // writing valid UTF-8 — readers see the existing
            // WireError::InvalidUtf8 at read time on misuse.
            // (String is not complex-nested → no staging slot → `has_staged`
            // is false, so no staged-write conflict guard is emitted.)
            emit_fill_from_method(out, raw_name, var_idx, "u8", 1, "", false);
        }
        FieldType::Bytes => {
            emit_typed_array_accessors(out, &field_name, raw_name, var_idx, "u8", 1);
        }
        FieldType::DynamicArray { element_type } => {
            // Must be primitive (complex case handled above).
            let (sz, ty) = primitive_size_and_type(element_type);
            emit_typed_array_accessors(out, &field_name, raw_name, var_idx, ty, sz);
        }
        _ => {
            // Should not reach here — variable_field_layout includes only
            // variable fields and is_complex_variable filters complex ones.
            writeln!(out, "    // unreachable: unsupported variable field shape").unwrap();
        }
    }
}

/// Emit typed `&[T]` reader, `loan_<f>(n) -> &mut [T]`, `set_<f>(&[T])`,
/// `push_<f>(T)` for a primitive element type.
///
/// `field_name` is the keyword-escaped name (used for the bare reader).
/// `raw_name` is the unescaped name (used for set_/loan_/push_ identifiers
/// where the prefix ensures they aren't bare keywords).
fn emit_typed_array_accessors(
    out: &mut String,
    field_name: &str,
    raw_name: &str,
    var_idx: usize,
    elem_ty: &str,
    elem_size: usize,
) {
    writeln!(
        out,
        "    /// Read variable field `{raw_name}` as `&[{elem_ty}]` borrowed from SHM."
    )
    .unwrap();
    writeln!(out, "    #[inline]").unwrap();
    writeln!(out, "    pub fn {field_name}(&self) -> &[{elem_ty}] {{").unwrap();
    writeln!(out, "        let payload = self.payload();").unwrap();
    writeln!(out, "        let (off, byte_len) = ::cerulion_core::shm_runtime::read_offset_entry(payload, Self::WIRE_FIXED_SIZE, {var_idx});").unwrap();
    writeln!(
        out,
        "        let off = off as usize; let byte_len = byte_len as usize;"
    )
    .unwrap();
    writeln!(
        out,
        "        if off.saturating_add(byte_len) > payload.len() {{ return &[]; }}"
    )
    .unwrap();
    if elem_ty == "u8" {
        // u8: payload slice IS &[u8], return the subslice directly.
        writeln!(out, "        &payload[off..off+byte_len]").unwrap();
    } else if elem_ty == "bool" {
        // `bool` is the only primitive with a validity
        // constraint (every byte must be 0 or 1). Creating `&[bool]` over
        // payload bytes that are NOT strictly {0,1} is UB at
        // REFERENCE-CREATION time, even if the caller never reads an
        // element. iceoryx2 does not zero-init recycled SHM slots, so a
        // `bool[]` field overlaying a slot a prior occupant wrote with
        // non-canonical bytes (e.g. a `u8[]` field at the same offset) is
        // reachable. The writer path (`loan_<f>`) zero-fills before its
        // cast; this reader must symmetrically refuse to fabricate an
        // invalid `&[bool]`. Validate first; on any non-canonical byte
        // return `&[]` (same fail-safe as the bounds check above) rather
        // than minting an unsound reference.
        writeln!(out, "        let slice = &payload[off..off+byte_len];").unwrap();
        writeln!(
            out,
            "        // Refuse to mint an unsound `&[bool]` over non-{{0,1}} bytes."
        )
        .unwrap();
        writeln!(
            out,
            "        if !slice.iter().all(|&b| b <= 1) {{ return &[]; }}"
        )
        .unwrap();
        writeln!(
            out,
            "        // SAFETY: every byte verified to be 0 or 1 above (valid `bool`); 1-byte alignment trivially satisfied."
        )
        .unwrap();
        writeln!(out, "        unsafe {{ ::std::slice::from_raw_parts(slice.as_ptr() as *const bool, byte_len) }}").unwrap();
    } else if elem_size == 1 {
        // Other 1-byte types (i8): any byte pattern is a valid value, no
        // validity check needed; type cast still required.
        writeln!(out, "        let slice = &payload[off..off+byte_len];").unwrap();
        writeln!(
            out,
            "        // SAFETY: 1-byte element (i8); any byte pattern is valid; alignment trivially satisfied."
        )
        .unwrap();
        writeln!(out, "        unsafe {{ ::std::slice::from_raw_parts(slice.as_ptr() as *const {elem_ty}, byte_len) }}").unwrap();
    } else {
        // Non-u8 multi-byte: alignment was enforced by loan_<field>.
        writeln!(out, "        let slice = &payload[off..off+byte_len];").unwrap();
        writeln!(
            out,
            "        if !(slice.as_ptr() as usize).is_multiple_of({elem_size}) {{ return &[]; }}"
        )
        .unwrap();
        writeln!(
            out,
            "        // SAFETY: alignment was enforced at loan time; bounds verified above."
        )
        .unwrap();
        writeln!(out, "        unsafe {{ ::std::slice::from_raw_parts(slice.as_ptr() as *const {elem_ty}, byte_len / {elem_size}) }}").unwrap();
    }
    writeln!(out, "    }}").unwrap();
    writeln!(out).unwrap();

    writeln!(out, "    /// Reserve `n` `{elem_ty}` slots in variable field `{raw_name}` and return a writable view.").unwrap();
    writeln!(out, "    pub fn loan_{raw_name}(&mut self, n: usize) -> ::std::result::Result<&mut [{elem_ty}], ::cerulion_core::TransportError> {{").unwrap();
    writeln!(
        out,
        "        let bytes_needed = n.saturating_mul({elem_size});"
    )
    .unwrap();
    // Compute alignment WITHOUT advancing
    // `self.state.cursor` before `ensure_capacity_for`. Were the
    // codegen to commit `self.state.cursor = cursor_aligned` BEFORE the
    // capacity check, then if `cursor_aligned > self.len` (alignment bump
    // pushes past the loan boundary) and the spill helper triggered,
    // `spill_to_overflow` would read `cursor_aligned` bytes from a loan
    // of only `self.len` bytes — OOB read of `cursor_aligned - self.len`
    // bytes of unloaned memory into the spill buffer. Deferring the
    // cursor advancement until AFTER the spill ensures `written_so_far`
    // never exceeds the original loan length.
    // Compute alignment WITHOUT advancing
    // `self.state.cursor` before `ensure_capacity_for`. Were the
    // codegen to commit `self.state.cursor = cursor_aligned` BEFORE the
    // capacity check, then if `cursor_aligned > self.len` (alignment bump
    // pushes past the loan boundary) and the spill helper triggered,
    // `spill_to_overflow` would read `cursor_aligned` bytes from a loan
    // of only `self.len` bytes — OOB read of `cursor_aligned - self.len`
    // bytes of unloaned memory into the spill buffer. Deferring the
    // cursor advancement until AFTER the spill ensures `written_so_far`
    // never exceeds the original loan length. Regression test:
    // `alignment_bump_past_self_len_does_not_oob_read_during_spill`.
    if elem_size > 1 {
        writeln!(out, "        // Align cursor up to alignof::<{elem_ty}>() so the resulting `&mut [{elem_ty}]` is sound.").unwrap();
        writeln!(out, "        let cursor = ((self.state.cursor as usize) + {elem_size} - 1) & !({elem_size} - 1);").unwrap();
    } else {
        writeln!(out, "        let cursor = self.state.cursor as usize;").unwrap();
    }
    // `ensure_capacity_for` replaces inline ProxyBufferTooSmall.
    // Same semantics as `emit_loan_body` — spills on recoverable
    // overflow, returns PayloadTooLarge/AllocationFailed at the ceiling.
    writeln!(
        out,
        "        self.ensure_capacity_for(bytes_needed, cursor)?;"
    )
    .unwrap();
    // Alignment gaps are inside the published payload, so recycled slot bytes
    // must not survive there. Clear only after the capacity check/spill succeeds.
    if elem_size > 1 {
        writeln!(
            out,
            "        let padding_start = self.state.cursor as usize;"
        )
        .unwrap();
        writeln!(
            out,
            "        self.payload_mut()[padding_start..cursor].fill(0);"
        )
        .unwrap();
    }
    writeln!(
        out,
        "        let new_cursor = cursor.saturating_add(bytes_needed);"
    )
    .unwrap();
    // Update state BEFORE the payload_mut borrow so the borrow checker
    // doesn't conflate the returned slice with `&mut self`.
    writeln!(
        out,
        "        self.state.field_starts[{var_idx}] = cursor as u32;"
    )
    .unwrap();
    writeln!(out, "        self.state.cursor = new_cursor as u32;").unwrap();
    writeln!(out, "        self.state.mark_written({var_idx});").unwrap();
    writeln!(out, "        let payload = self.payload_mut();").unwrap();
    writeln!(out, "        ::cerulion_core::shm_runtime::write_offset_entry(payload, Self::WIRE_FIXED_SIZE, {var_idx}, cursor as u32, bytes_needed as u32);").unwrap();
    writeln!(
        out,
        "        let dst_bytes = &mut payload[cursor..new_cursor];"
    )
    .unwrap();
    if elem_ty == "u8" {
        // u8: dst_bytes IS &mut [u8], return directly.
        writeln!(out, "        Ok(dst_bytes)").unwrap();
    } else if elem_ty == "bool" {
        // `bool` is the only primitive with a
        // validity constraint (must be 0 or 1). Creating `&mut [bool]`
        // from raw bytes that were left non-{0,1} by a previous loan
        // of the same SHM slot is UB at REFERENCE-CREATION time, even
        // if the producer never reads. iceoryx2 does not zero-init
        // recycled slots, so this is reachable. Zero-fill before the
        // cast: all 0 bytes are valid `bool::false`.
        writeln!(
            out,
            "        // Zero-fill before bool cast — `from_raw_parts_mut` over"
        )
        .unwrap();
        writeln!(
            out,
            "        // non-{{0,1}} bytes is UB at reference creation; iceoryx2 doesn't"
        )
        .unwrap();
        writeln!(out, "        // zero-init recycled slots.").unwrap();
        writeln!(out, "        dst_bytes.fill(0u8);").unwrap();
        writeln!(out, "        // SAFETY: bytes zero-filled above (all 0 is valid `bool::false`); 1-byte alignment trivially satisfied.").unwrap();
        writeln!(out, "        Ok(unsafe {{ ::std::slice::from_raw_parts_mut(dst_bytes.as_mut_ptr() as *mut bool, n) }})").unwrap();
    } else if elem_size == 1 {
        // i8: any byte pattern is a valid i8 value, no init needed.
        writeln!(
            out,
            "        // SAFETY: 1-byte element (i8); any byte pattern is a valid i8 value; alignment trivially satisfied."
        )
        .unwrap();
        writeln!(out, "        Ok(unsafe {{ ::std::slice::from_raw_parts_mut(dst_bytes.as_mut_ptr() as *mut {elem_ty}, n) }})").unwrap();
    } else {
        // Multi-byte primitives (f32/f64/i16-i64/u16-u64) — no validity
        // constraints; alignment was enforced above.
        writeln!(out, "        // SAFETY: cursor was aligned to {elem_size} bytes above; the slice is contiguous and within bounds; any byte pattern is a valid {elem_ty} value.").unwrap();
        writeln!(out, "        Ok(unsafe {{ ::std::slice::from_raw_parts_mut(dst_bytes.as_mut_ptr() as *mut {elem_ty}, n) }})").unwrap();
    }
    writeln!(out, "    }}").unwrap();
    writeln!(out).unwrap();

    writeln!(
        out,
        "    /// Copy `value` into variable field `{raw_name}` (single boundary memcpy)."
    )
    .unwrap();
    writeln!(out, "    pub fn set_{raw_name}(&mut self, value: &[{elem_ty}]) -> ::std::result::Result<(), ::cerulion_core::TransportError> {{").unwrap();
    writeln!(out, "        let dst = self.loan_{raw_name}(value.len())?;").unwrap();
    writeln!(out, "        dst.copy_from_slice(value);").unwrap();
    writeln!(out, "        Ok(())").unwrap();
    writeln!(out, "    }}").unwrap();
    writeln!(out).unwrap();

    // `fill_from_<f>` for the typed-array variable field. Generic
    // over a `FillFrom<elem_ty>` producer; reserves the remaining
    // payload region, invokes the producer, then truncates cursor +
    // offset entry to the actual elements written.
    // (Typed primitive arrays are not complex-nested → no staging slot →
    // `has_staged` is false, so no staged-write conflict guard is emitted.)
    emit_fill_from_method(out, raw_name, var_idx, elem_ty, elem_size, "", false);

    // push_<field>(item) — appends if field is at the cursor tail, else errors.
    writeln!(
        out,
        "    /// Append one `{elem_ty}` to variable field `{raw_name}`."
    )
    .unwrap();
    writeln!(out, "    ///").unwrap();
    writeln!(
        out,
        "    /// Push semantics: appends iff the field is currently at the cursor tail"
    )
    .unwrap();
    writeln!(
        out,
        "    /// (i.e., the previous write to this field was the most recent variable write)."
    )
    .unwrap();
    writeln!(
        out,
        "    /// Calling push after a write to a *different* variable field returns"
    )
    .unwrap();
    writeln!(
        out,
        "    /// `Err(PushAfterNonTail)` — interleaving pushes is unsupported. Use"
    )
    .unwrap();
    writeln!(
        out,
        "    /// `set_<field>` or `loan_<field>` for non-tail writes."
    )
    .unwrap();
    writeln!(out, "    pub fn push_{raw_name}(&mut self, item: {elem_ty}) -> ::std::result::Result<(), ::cerulion_core::TransportError> {{").unwrap();
    writeln!(
        out,
        "        let was_written = self.state.is_written({var_idx});"
    )
    .unwrap();
    writeln!(
        out,
        "        let (existing_off, existing_len) = if was_written {{"
    )
    .unwrap();
    writeln!(out, "            ::cerulion_core::shm_runtime::read_offset_entry(self.payload(), Self::WIRE_FIXED_SIZE, {var_idx})").unwrap();
    writeln!(out, "        }} else {{ (0u32, 0u32) }};").unwrap();
    writeln!(out, "        if was_written && (existing_off as usize) + (existing_len as usize) != self.state.cursor as usize {{").unwrap();
    writeln!(
        out,
        "            return Err(::cerulion_core::TransportError::PushAfterNonTail {{"
    )
    .unwrap();
    writeln!(out, "                field: \"{raw_name}\",").unwrap();
    writeln!(out, "            }});").unwrap();
    writeln!(out, "        }}").unwrap();
    // Mirror of `loan_<f>` typed — do
    // NOT commit `self.state.cursor = cursor_aligned` before the
    // capacity check. Compute alignment in a local; commit only AFTER
    // `ensure_capacity_for` succeeds. Advancing the cursor
    // before the check lets `spill_to_overflow` OOB-read
    // past the original loan boundary.
    if elem_size > 1 {
        writeln!(out, "        // For first push, align cursor to alignof::<{elem_ty}>(); subsequent pushes preserve alignment.").unwrap();
        writeln!(out, "        let cursor = if !was_written {{").unwrap();
        writeln!(
            out,
            "            ((self.state.cursor as usize) + {elem_size} - 1) & !({elem_size} - 1)"
        )
        .unwrap();
        writeln!(out, "        }} else {{").unwrap();
        writeln!(out, "            self.state.cursor as usize").unwrap();
        writeln!(out, "        }};").unwrap();
    } else {
        writeln!(out, "        let cursor = self.state.cursor as usize;").unwrap();
    }
    // `ensure_capacity_for` replaces inline ProxyBufferTooSmall.
    // Same semantics as the other setter sites. Note: PushAfterNonTail
    // (above) is preserved as a distinct error — non-tail interleave is
    // a misuse pattern, not a capacity issue, so it does NOT funnel
    // through the spill path.
    writeln!(
        out,
        "        self.ensure_capacity_for({elem_size}, cursor)?;"
    )
    .unwrap();
    // A first push has the same published alignment gap as a typed loan.
    // The capacity check above also covers that gap, including after a spill.
    if elem_size > 1 {
        writeln!(out, "        if !was_written {{").unwrap();
        writeln!(
            out,
            "            let padding_start = self.state.cursor as usize;"
        )
        .unwrap();
        writeln!(
            out,
            "            self.payload_mut()[padding_start..cursor].fill(0);"
        )
        .unwrap();
        writeln!(out, "        }}").unwrap();
    }
    writeln!(
        out,
        "        let new_cursor = cursor.saturating_add({elem_size});"
    )
    .unwrap();
    writeln!(out, "        let payload = self.payload_mut();").unwrap();
    // 1-byte element types get a direct byte store. `bool` has no
    // `to_le_bytes()` (the unconditional form did not compile for
    // `DynamicArray<Bool>`); u8/i8 compile either way but a direct store
    // avoids a pointless 1-byte array round-trip. Mirrors the
    // "Other 1-byte types (i8, bool): no alignment concern" arms in the
    // reader/loan paths above.
    if elem_ty == "bool" {
        writeln!(
            out,
            "        payload[cursor] = if item {{ 1 }} else {{ 0 }};"
        )
        .unwrap();
    } else if elem_ty == "u8" {
        writeln!(out, "        payload[cursor] = item;").unwrap();
    } else if elem_ty == "i8" {
        writeln!(out, "        payload[cursor] = item as u8;").unwrap();
    } else {
        writeln!(out, "        let bytes = item.to_le_bytes();").unwrap();
        writeln!(
            out,
            "        payload[cursor..new_cursor].copy_from_slice(&bytes);"
        )
        .unwrap();
    }
    writeln!(out, "        if was_written {{").unwrap();
    writeln!(
        out,
        "            // Extend the existing field by one element."
    )
    .unwrap();
    writeln!(out, "            ::cerulion_core::shm_runtime::write_offset_entry(payload, Self::WIRE_FIXED_SIZE, {var_idx}, existing_off, existing_len + {elem_size}u32);").unwrap();
    writeln!(out, "        }} else {{").unwrap();
    writeln!(out, "            ::cerulion_core::shm_runtime::write_offset_entry(payload, Self::WIRE_FIXED_SIZE, {var_idx}, cursor as u32, {elem_size}u32);").unwrap();
    writeln!(
        out,
        "            self.state.field_starts[{var_idx}] = cursor as u32;"
    )
    .unwrap();
    writeln!(out, "            self.state.mark_written({var_idx});").unwrap();
    writeln!(out, "        }}").unwrap();
    writeln!(out, "        self.state.cursor = new_cursor as u32;").unwrap();
    writeln!(out, "        Ok(())").unwrap();
    writeln!(out, "    }}").unwrap();
    writeln!(out).unwrap();
}

/// Emit `fill_from_<raw>{suffix}` for a variable field.
///
/// The emitted method reserves the remaining payload region for this
/// field, hands the producer a `&mut [<elem_ty>]` slice into the SHM,
/// and truncates the cursor + offset-table entry to the actual number
/// of elements the producer wrote. Replaces the user-level `loan_<f>(n)
/// + write + manual truncate` dance with one call.
///
/// Failure-safety contract: on Err or
/// panic from the producer, the writer's
/// observable state — `field_starts[var_idx]`, `state.cursor`,
/// `mark_written[var_idx]`, AND the SHM offset-table entry for
/// `var_idx` — is **unchanged from before the call**. This is
/// load-bearing when a prior successful write had already set
/// `mark_written[var_idx] = true`: without it, the publish gate
/// would let through a frame whose offset entry points at the
/// failed loan's address instead of the previous successful write's.
///
/// The implementation achieves this by deferring ALL state and
/// offset-entry mutations to the success arm — the producer call
/// is wrapped in a scope that only borrows `payload` for the dst
/// slice; after the call, the borrow is released and the success
/// arm re-borrows to commit. Err / panic propagate before any
/// commit. No `catch_unwind` needed.
///
/// Failure modes:
/// - Producer returns `Err`: NO state mutation happened. Caller sees
///   the writer state as if the call never occurred — if a prior
///   write to the same field succeeded, its offset entry is intact.
///   If the field was never written, `mark_written` stays false and
///   `OutputProxy::Drop` refuses to publish (no partial frame).
/// - Producer panics: same end state by construction (the Err arm
///   IS the panic recovery — no separate path needed; Rust's natural
///   unwind takes care of it).
/// - Producer returns `written > dst.len()` (lying impl): defensively
///   clamped via `min(written, n)` before the cursor / offset commit.
/// - Buffer exhaustion before the loan: when `cursor_aligned > self.len`,
///   the method calls `ensure_capacity_for` which either SPILLS to the
///   heap fallback (if `required_end <= max_capacity`) or
///   returns `Err(TransportError::PayloadTooLarge)` / `AllocationFailed`
///   — never panics on raw slice index. Previously this site
///   returned `ProxyBufferTooSmall` inline.
///
/// Parameters:
/// - `raw_name`: the schema field name (used for the method ident).
/// - `var_idx`: the field's index in the variable section.
/// - `elem_ty`: Rust element type (e.g. `"u8"`, `"f32"`, `"i32"`).
/// - `elem_size`: `size_of::<elem_ty>()`. Alignment of the loaned slice
///   is enforced by rounding the cursor up to `elem_size` BEFORE the
///   reservation (matches `loan_<f>` behavior).
/// - `suffix`: `""` for typed variants (`fill_from_<f>`) or `"_bytes"`
///   for complex variable fields (`fill_from_<f>_bytes`). The suffix
///   only affects the method name — the body is identical (complex
///   variable always uses `elem_ty = "u8"`, `elem_size = 1`).
/// - `has_staged`: `true` only for a complex-NESTED field carrying a
///   `__cer_staged_<f>` slot. When set, guard A is emitted at
///   the TOP of the method body — a whole-field `fill_from` while staged
///   leaf writes are pending is the "staged then whole" conflict. `fill_from`
///   has its own cursor body (does NOT route through `loan_<f>_bytes`), so
///   the guard must be mirrored here.
fn emit_fill_from_method(
    out: &mut String,
    raw_name: &str,
    var_idx: usize,
    elem_ty: &str,
    elem_size: usize,
    suffix: &str,
    has_staged: bool,
) {
    writeln!(
        out,
        "    /// Fill variable field `{raw_name}` from a [`FillFrom`](::cerulion_core::transport::fill_from::FillFrom) producer."
    )
    .unwrap();
    writeln!(out, "    ///").unwrap();
    writeln!(
        out,
        "    /// Reserves the remaining payload capacity for this field, hands the producer a"
    )
    .unwrap();
    writeln!(
        out,
        "    /// `&mut [{elem_ty}]` slice into the SHM-loaned region; on success the cursor"
    )
    .unwrap();
    writeln!(
        out,
        "    /// and offset-table entry are then committed to the actual element count the producer wrote."
    )
    .unwrap();
    writeln!(out, "    ///").unwrap();
    writeln!(
        out,
        "    /// On `Err` from the producer, **the writer state is bit-for-bit unchanged**"
    )
    .unwrap();
    writeln!(
        out,
        "    /// from before the call: no cursor advance, no `field_starts` write, no"
    )
    .unwrap();
    writeln!(
        out,
        "    /// `mark_written` bit set, no offset-table entry written. If a PRIOR successful"
    )
    .unwrap();
    writeln!(
        out,
        "    /// write to this same field already set `mark_written` and an offset entry,"
    )
    .unwrap();
    writeln!(
        out,
        "    /// those are preserved and `OutputProxy::Drop` publishes the original data —"
    )
    .unwrap();
    writeln!(
        out,
        "    /// `fill_from` failures CANNOT corrupt a previously-good write."
    )
    .unwrap();
    writeln!(
        out,
        "    /// Producer panics get the same treatment by construction (no `catch_unwind` needed)."
    )
    .unwrap();
    writeln!(out, "    ///").unwrap();
    writeln!(
        out,
        "    /// Adversarial producers returning `written > dst.len()` are defensively"
    )
    .unwrap();
    writeln!(out, "    /// clamped before the commit.").unwrap();
    writeln!(
        out,
        "    pub fn fill_from_{raw_name}{suffix}<__CerS>(&mut self, mut src: __CerS) -> ::std::result::Result<(), ::cerulion_core::TransportError>"
    )
    .unwrap();
    writeln!(out, "    where").unwrap();
    writeln!(
        out,
        "        __CerS: ::cerulion_core::transport::fill_from::FillFrom<{elem_ty}>,"
    )
    .unwrap();
    writeln!(out, "    {{").unwrap();

    // Guard A (mirror of `loan_<f>_bytes`): reject a whole-field
    // `fill_from` while staged leaf writes are pending. Only emitted for a
    // complex-nested field with a staging slot; the Drop-flush never trips it
    // (it `.take()`s the slot first).
    if has_staged {
        emit_staged_then_whole_conflict_guard(out, raw_name);
    }

    if elem_size > 1 {
        // Align cursor up to alignof::<T>() so the resulting `&mut [T]` slice is sound.
        writeln!(out, "        // Align cursor up to alignof::<{elem_ty}>() so the resulting `&mut [{elem_ty}]` slice is sound.").unwrap();
        writeln!(out, "        let cursor_aligned = ((self.state.cursor as usize) + {elem_size} - 1) & !({elem_size} - 1);").unwrap();
    } else {
        writeln!(
            out,
            "        let cursor_aligned = self.state.cursor as usize;"
        )
        .unwrap();
    }

    // Bounds-check cursor_aligned against the
    // loaned buffer length BEFORE any slice indexing or state mutation.
    // STRICTLY > triggers a spill (recoverable overflow
    // within max_capacity) or PayloadTooLarge (above ceiling). The exact
    // boundary `cursor_aligned == self.len` is preserved as Ok-with-zero-
    // remaining-bytes (a contract pinned by
    // `fill_from_typed_at_exact_capacity_boundary_succeeds_with_zero_elements`).
    //
    // Pass `bytes_needed=0` (not 1) so the edge case `cursor_aligned ==
    // max_cap` correctly SPILLS-then-Ok-with-0-elements instead of
    // spuriously returning `PayloadTooLarge`. With `1`: `required_end =
    // max_cap + 1 > max_cap` → `PayloadTooLarge`. With `0`: `required_end
    // = max_cap` ≤ max_cap → spill cleanly, producer gets an empty dst
    // slice (mirrors the `cursor_aligned == self.len` zero-elements
    // contract). The producer's actual write capacity is enforced
    // post-spill by `remaining_bytes = self.len - cursor_aligned`, not
    // by this pre-flight capacity check — passing 0 is correct.
    writeln!(out, "        if cursor_aligned > self.len {{").unwrap();
    writeln!(
        out,
        "            self.ensure_capacity_for(0, cursor_aligned)?;"
    )
    .unwrap();
    writeln!(out, "        }}").unwrap();

    writeln!(
        out,
        "        let remaining_bytes = self.len.saturating_sub(cursor_aligned);"
    )
    .unwrap();
    if elem_size == 1 {
        // Skip the /1 *1 dance — clippy's `identity_op` lint fires on
        // the trivial form and codegen-emitted code must satisfy
        // `-D warnings` like every other crate.
        writeln!(out, "        let n_elements = remaining_bytes;").unwrap();
        writeln!(out, "        let bytes_needed = remaining_bytes;").unwrap();
    } else {
        writeln!(
            out,
            "        let n_elements = remaining_bytes / {elem_size};"
        )
        .unwrap();
        writeln!(out, "        let bytes_needed = n_elements * {elem_size};").unwrap();
    }

    // ALL state and offset-entry mutations happen ONLY in the Ok arm. The
    // producer call is wrapped in a scope that takes a transient
    // mutable borrow of payload for the dst slice; on producer Err /
    // panic the borrow is released and the writer state is BIT-FOR-BIT
    // unchanged from before the call. This preserves prior-write
    // correctness when fill_from(Err) follows a successful set_/loan_
    // on the same field.
    writeln!(out, "        let result = {{").unwrap();
    writeln!(out, "            let payload = self.payload_mut();").unwrap();
    writeln!(
        out,
        "            let dst_bytes = &mut payload[cursor_aligned..cursor_aligned + bytes_needed];"
    )
    .unwrap();

    if elem_ty == "u8" {
        // u8: dst_bytes IS &mut [u8], pass directly.
        writeln!(out, "            let dst: &mut [u8] = dst_bytes;").unwrap();
    } else if elem_ty == "bool" {
        // Zero-fill before bool cast — same
        // reasoning as the loan_<f> emission. Without this, recycled
        // SHM slots with non-{0,1} bytes cause UB at &mut [bool]
        // creation, even if the producer never reads.
        writeln!(out, "            // Zero-fill before bool cast (`from_raw_parts_mut::<bool>` over non-{{0,1}} bytes is UB).").unwrap();
        writeln!(out, "            dst_bytes.fill(0u8);").unwrap();
        writeln!(
            out,
            "            // SAFETY: bytes zero-filled above (all 0 is valid `bool::false`)."
        )
        .unwrap();
        writeln!(out, "            let dst: &mut [bool] = unsafe {{ ::std::slice::from_raw_parts_mut(dst_bytes.as_mut_ptr() as *mut bool, n_elements) }};").unwrap();
    } else if elem_size == 1 {
        // i8: any byte pattern is a valid i8 value, no init needed.
        writeln!(
            out,
            "            // SAFETY: 1-byte element (i8); any byte pattern is a valid i8 value."
        )
        .unwrap();
        writeln!(out, "            let dst: &mut [{elem_ty}] = unsafe {{ ::std::slice::from_raw_parts_mut(dst_bytes.as_mut_ptr() as *mut {elem_ty}, n_elements) }};").unwrap();
    } else {
        // Multi-byte primitives (f32/f64/i16-i64/u16-u64) — no validity
        // constraints; alignment was enforced above.
        writeln!(out, "            // SAFETY: cursor was aligned to {elem_size} bytes above; the slice is contiguous and within bounds; any byte pattern is a valid {elem_ty} value.").unwrap();
        writeln!(out, "            let dst: &mut [{elem_ty}] = unsafe {{ ::std::slice::from_raw_parts_mut(dst_bytes.as_mut_ptr() as *mut {elem_ty}, n_elements) }};").unwrap();
    }

    writeln!(out, "            src.fill_from(dst)").unwrap();
    writeln!(out, "        }};").unwrap();
    writeln!(out, "        match result {{").unwrap();

    // Success arm: clamp; commit cursor + field_starts + offset entry + mark_written.
    writeln!(out, "            ::std::result::Result::Ok(written) => {{").unwrap();
    writeln!(
        out,
        "                // Defensive clamp: a lying impl returning written > n_elements"
    )
    .unwrap();
    writeln!(
        out,
        "                // would otherwise commit an offset-table length past the loan."
    )
    .unwrap();
    writeln!(
        out,
        "                // Warn loudly when we clamp — the producer is buggy"
    )
    .unwrap();
    writeln!(
        out,
        "                // and the data the user thinks they wrote is wrong, but we cannot"
    )
    .unwrap();
    writeln!(
        out,
        "                // refuse the call (the trait API has no way to surface clamp-as-Err)."
    )
    .unwrap();
    writeln!(out, "                if written > n_elements {{").unwrap();
    writeln!(out, "                    ::cerulion_core::tracing::warn!(").unwrap();
    writeln!(out, "                        field = \"{raw_name}\",").unwrap();
    writeln!(out, "                        reported_written = written,").unwrap();
    writeln!(out, "                        loan_capacity = n_elements,").unwrap();
    writeln!(out, "                        \"FillFrom producer returned written > dst.len() — clamping; producer impl is buggy\"").unwrap();
    writeln!(out, "                    );").unwrap();
    writeln!(out, "                }}").unwrap();
    writeln!(
        out,
        "                let written = written.min(n_elements);"
    )
    .unwrap();
    if elem_size == 1 {
        writeln!(out, "                let actual_bytes = written;").unwrap();
    } else {
        writeln!(
            out,
            "                let actual_bytes = written * {elem_size};"
        )
        .unwrap();
    }
    // Pin invariants after the clamp + cast.
    writeln!(
        out,
        "                debug_assert!(actual_bytes <= bytes_needed, \"fill_from clamp invariant violated: actual_bytes > bytes_needed\");"
    )
    .unwrap();
    writeln!(
        out,
        "                debug_assert!(cursor_aligned + actual_bytes <= u32::MAX as usize, \"fill_from cursor overflow: cursor_aligned + actual_bytes exceeds u32::MAX\");"
    )
    .unwrap();
    // Clear published alignment padding only on success. An Err or panic keeps
    // both the previous committed frame and its cursor unchanged.
    if elem_size > 1 {
        writeln!(
            out,
            "                let padding_start = self.state.cursor as usize;"
        )
        .unwrap();
        writeln!(
            out,
            "                self.payload_mut()[padding_start..cursor_aligned].fill(0);"
        )
        .unwrap();
    }
    // Commit state ONLY on success. Order
    // mirrors emit_loan_body — state field assignments first (each is
    // a transient borrow), then payload_mut for the offset entry, then
    // mark_written (transient borrow again).
    writeln!(
        out,
        "                self.state.field_starts[{var_idx}] = cursor_aligned as u32;"
    )
    .unwrap();
    writeln!(
        out,
        "                self.state.cursor = (cursor_aligned + actual_bytes) as u32;"
    )
    .unwrap();
    writeln!(out, "                self.state.mark_written({var_idx});").unwrap();
    writeln!(out, "                let payload = self.payload_mut();").unwrap();
    writeln!(
        out,
        "                ::cerulion_core::shm_runtime::write_offset_entry(payload, Self::WIRE_FIXED_SIZE, {var_idx}, cursor_aligned as u32, actual_bytes as u32);"
    )
    .unwrap();
    writeln!(out, "                ::std::result::Result::Ok(())").unwrap();
    writeln!(out, "            }}").unwrap();

    // Err arm: NO state mutation. The producer's borrow on payload
    // was released when the scope ended; no commit happens. Prior
    // writes' offset entries + mark_written bits are intact.
    writeln!(out, "            ::std::result::Result::Err(err) => {{").unwrap();
    writeln!(
        out,
        "                // NO state mutation. The producer's transient borrow on"
    )
    .unwrap();
    writeln!(
        out,
        "                // payload was released at the end of the result-scope;"
    )
    .unwrap();
    writeln!(
        out,
        "                // we never wrote the offset entry or touched"
    )
    .unwrap();
    writeln!(
        out,
        "                // field_starts/cursor/mark_written. If a PRIOR successful"
    )
    .unwrap();
    writeln!(
        out,
        "                // write set mark_written for this field, its offset entry"
    )
    .unwrap();
    writeln!(
        out,
        "                // is intact and OutputProxy::Drop publishes the"
    )
    .unwrap();
    writeln!(
        out,
        "                // ORIGINAL data, not corrupt loan-region bytes."
    )
    .unwrap();
    writeln!(out, "                ::std::result::Result::Err(err)").unwrap();
    writeln!(out, "            }}").unwrap();
    writeln!(out, "        }}").unwrap();
    writeln!(out, "    }}").unwrap();
    writeln!(out).unwrap();
}

/// Emit the body of a generic `loan_<field>_bytes(n)` returning `&mut [u8]`.
fn emit_loan_body(out: &mut String, var_idx: usize, n_expr: &str) {
    writeln!(out, "        let cursor = self.state.cursor as usize;").unwrap();
    // `ensure_capacity_for` replaces the old inline
    // `cursor + n > self.len` ProxyBufferTooSmall return. When the
    // current loan can't fit but `max_capacity` can, it spills to a
    // heap `Vec<u64>` and repoints `self.ptr` / `self.len` so the
    // payload_mut() borrow below naturally writes to the spill.
    // When even `max_capacity` is too small, returns `PayloadTooLarge`.
    // When the spill allocation fails, returns `AllocationFailed`.
    writeln!(out, "        self.ensure_capacity_for({n_expr}, cursor)?;").unwrap();
    writeln!(
        out,
        "        let new_cursor = cursor.saturating_add({n_expr});"
    )
    .unwrap();
    // Update state BEFORE acquiring the mutable payload borrow so the borrow
    // checker doesn't conflate the returned `&mut [u8]` with `&mut self`.
    writeln!(
        out,
        "        self.state.field_starts[{var_idx}] = cursor as u32;"
    )
    .unwrap();
    writeln!(out, "        self.state.cursor = new_cursor as u32;").unwrap();
    writeln!(out, "        self.state.mark_written({var_idx});").unwrap();
    writeln!(out, "        let payload = self.payload_mut();").unwrap();
    writeln!(out, "        ::cerulion_core::shm_runtime::write_offset_entry(payload, Self::WIRE_FIXED_SIZE, {var_idx}, cursor as u32, ({n_expr}) as u32);").unwrap();
    writeln!(out, "        Ok(&mut payload[cursor..new_cursor])").unwrap();
}

/// Emit `pub fn snapshot(&self) -> <Name>Snapshot` for a variable schema.
fn generate_var_shm_snapshot_method(out: &mut String, schema: &MessageSchema, snapshot_name: &str) {
    writeln!(
        out,
        "    /// Copy this SHM-backed message into a heap-owned [`{snapshot_name}`]."
    )
    .unwrap();
    writeln!(out, "    ///").unwrap();
    writeln!(
        out,
        "    /// Allocates owned `String`/`Vec` for variable fields. Complex variable"
    )
    .unwrap();
    writeln!(
        out,
        "    /// fields (nested types, arrays of nested) are stored as raw `Vec<u8>`"
    )
    .unwrap();
    writeln!(
        out,
        "    /// in the snapshot — typed wrapping is not yet provided."
    )
    .unwrap();
    writeln!(out, "    pub fn snapshot(&self) -> {snapshot_name} {{").unwrap();
    writeln!(out, "        {snapshot_name} {{").unwrap();
    for field in &schema.fields {
        let field_name = escape_keyword(&field.name);
        let raw_name = &field.name;
        let expr = snapshot_read_expr(&field.field_type, &field_name, raw_name);
        // Struct-init shorthand uses the keyword-escaped form.
        writeln!(out, "            {field_name}: {expr},").unwrap();
    }
    writeln!(out, "        }}").unwrap();
    writeln!(out, "    }}").unwrap();
    writeln!(out).unwrap();
}

/// Generate the read expression for one field of a Snapshot constructor.
///
/// `field_name` is the keyword-escaped name (used for direct field access on
/// the FixedSection deref target and for variable-field reader calls). For
/// fixed-section fields the read goes through `Deref<Target =
/// <Name>FixedSection>` — `self.<field>` instead of `self.<field>()`.
///
/// `raw_name` is the unescaped name (used for `_bytes` suffix to address
/// keyword-named complex fields without producing `r#type_bytes`).
///
/// String accessors return `Result<&str, WireError>`;
/// snapshot construction unwraps to the empty string on UTF-8 decode
/// failure rather than allocating a `String::from_utf8_lossy` — the
/// snapshot is intended for tests/replay where a corrupt wire frame is
/// already a fatal anomaly.
fn snapshot_read_expr(ft: &FieldType, field_name: &str, raw_name: &str) -> String {
    if is_complex_variable(ft) {
        return format!("self.{raw_name}_bytes().to_vec()");
    }
    match ft {
        // Variable fields still go through accessor methods.
        FieldType::String => format!("self.{field_name}().unwrap_or(\"\").to_string()"),
        FieldType::Bytes => format!("self.{field_name}().to_vec()"),
        FieldType::DynamicArray { .. } => format!("self.{field_name}().to_vec()"),
        // Fixed-section fields: read via Deref → <Name>FixedSection.
        // `shm_to_snapshot_expr` handles every fixed-section shape: bool
        // (u8 → bool), FixedArray<bool>, resolved-fixed Nested
        // (`.snapshot()`), FixedArray<Nested>, and plain
        // primitives / StringFixed / FixedArray<primitive> (verbatim copy).
        _ => shm_to_snapshot_expr(ft, &format!("self.{field_name}")),
    }
}

/// Emit `pub fn write_from_snapshot(&mut self, snap: &<Name>Snapshot) -> Result<(), TransportError>`.
fn generate_var_shm_write_from_snapshot_method(
    out: &mut String,
    schema: &MessageSchema,
    snapshot_name: &str,
) {
    writeln!(out, "    /// Write `snap` into this SHM-backed message.").unwrap();
    writeln!(out, "    ///").unwrap();
    writeln!(
        out,
        "    /// Variable fields go through the typed `set_<field>` (or `set_<field>_bytes`"
    )
    .unwrap();
    writeln!(
        out,
        "    /// for complex types) wrappers, which set the per-field `written` bit."
    )
    .unwrap();
    writeln!(out, "    pub fn write_from_snapshot(&mut self, snap: &{snapshot_name}) -> ::std::result::Result<(), ::cerulion_core::TransportError> {{").unwrap();
    if schema.fields.is_empty() {
        writeln!(out, "        let _ = snap;").unwrap();
    } else {
        for field in &schema.fields {
            let stmt = snapshot_write_stmt(&field.field_type, &field.name);
            writeln!(out, "        {stmt}").unwrap();
        }
    }
    writeln!(out, "        Ok(())").unwrap();
    writeln!(out, "    }}").unwrap();
    writeln!(out).unwrap();
}

/// Generate one statement that writes a Snapshot field into the SHM-backed type.
///
/// Variable fields go through their typed `set_<name>` (or `set_<name>_bytes`
/// for complex variants) accessors. Fixed-section fields are
/// written via `Deref<Target = <Name>FixedSection>` direct field assignment
/// (`self.<name> = ...`), with bool→u8 coercion to mirror the storage type.
///
/// Uses the raw field name for `set_<name>` callables and the keyword-escaped
/// name for direct field access on `&mut FixedSection`.
fn snapshot_write_stmt(ft: &FieldType, raw_name: &str) -> String {
    let escaped = escape_keyword(raw_name);
    if is_complex_variable(ft) {
        return format!("self.set_{raw_name}_bytes(&snap.{escaped})?;");
    }
    match ft {
        // Variable accessors still gate on `set_<name>(...)`.
        FieldType::String => format!("self.set_{raw_name}(&snap.{escaped})?;"),
        FieldType::Bytes | FieldType::DynamicArray { .. } => {
            format!("self.set_{raw_name}(&snap.{escaped})?;")
        }
        // Fixed-section direct-field assignment via Deref.
        // `snapshot_to_shm_expr` handles every fixed-section shape: bool
        // (bool → u8), FixedArray<bool>, resolved-fixed Nested
        // (`write_from_snapshot` into a default Shm),
        // FixedArray<Nested>, and plain primitives (verbatim copy).
        _ => {
            let expr = snapshot_to_shm_expr(ft, &format!("snap.{escaped}"));
            format!("self.{escaped} = {expr};")
        }
    }
}

/// Generate `<Name>Snapshot` for a variable schema.
///
/// Differs from [`generate_snapshot_struct`] only in the field type mapping —
/// we use [`field_type_to_rust_snapshot`] so Nested becomes `<schema_name>Snapshot`
/// (and complex types become `Vec<u8>`). Same derives.
pub(super) fn generate_variable_snapshot_struct(out: &mut String, schema: &MessageSchema) {
    let name = &schema.name;
    let snapshot_name = format!("{name}Snapshot");

    writeln!(out, "/// Heap-owned snapshot of a [`{name}`] message.").unwrap();
    writeln!(out, "///").unwrap();
    writeln!(
        out,
        "/// Companion to [`{name}Shm`] for code that needs to escape the iceoryx2"
    )
    .unwrap();
    writeln!(
        out,
        "/// sample lifetime. Variable fields are owned (`Vec`/`String`); nested fields"
    )
    .unwrap();
    writeln!(
        out,
        "/// reference the corresponding `<Nested>Snapshot` types. Complex variable"
    )
    .unwrap();
    writeln!(
        out,
        "/// fields (Nested, arrays of Nested) are stored as raw `Vec<u8>`."
    )
    .unwrap();

    emit_snapshot_derives(out, schema);
    writeln!(out, "pub struct {snapshot_name} {{").unwrap();

    for field in &schema.fields {
        if let Some(desc) = &field.description {
            writeln!(out, "    /// {desc}").unwrap();
        }
        let rust_type = field_type_to_rust_snapshot(&field.field_type);
        let field_name = escape_keyword(&field.name);
        emit_snapshot_field_serde_attr(out, &field.field_type);
        writeln!(out, "    pub {field_name}: {rust_type},").unwrap();
    }

    writeln!(out, "}}").unwrap();
    writeln!(out).unwrap();

    // Manual Default when forced (direct large array or non-zero scalar
    // defaults); transitive-only cases derive it (clippy `derivable_impls`).
    if needs_manual_default(schema) {
        generate_variable_snapshot_default_impl(out, schema, &snapshot_name);
    }
}

/// Manual `Default` for a variable schema's `<Name>Snapshot` when derive
/// is impossible (direct large array) or wrong (non-zero scalar field
/// defaults).
fn generate_variable_snapshot_default_impl(
    out: &mut String,
    schema: &MessageSchema,
    snapshot_name: &str,
) {
    writeln!(out, "impl Default for {snapshot_name} {{").unwrap();
    writeln!(out, "    fn default() -> Self {{").unwrap();
    writeln!(out, "        Self {{").unwrap();
    for field in &schema.fields {
        let field_name = escape_keyword(&field.name);
        let default_value = var_snapshot_field_default_expr(field);
        writeln!(out, "            {field_name}: {default_value},").unwrap();
    }
    writeln!(out, "        }}").unwrap();
    writeln!(out, "    }}").unwrap();
    writeln!(out, "}}").unwrap();
    writeln!(out).unwrap();
}

// =============================================================================
// Schema-blind macro plumbing — uniform per-field write shims + the
// write-only diagnostic proxy for variable fields.
// =============================================================================
//
// The `#[cerulion_node]` macro cannot see schema internals (proc-macro
// registry metadata gap), so it cannot know which `self.<port>.<field>
// = expr` assignments target a variable field. That
// fixed-vs-variable knowledge lives in codegen (which IS schema-aware): every
// field gets a uniform `__cer_assign_<f>(&value) -> Result<(), TransportError>`
// shim, so the rewriter is schema-blind and calls one shape for all
// fields. Fixed shims write through direct assignment (`self.<f> = *v`);
// variable shims delegate to the existing `set_<f>` / `set_<f>_bytes`.
//
// The macro rewriter calls these. Pub methods/types on pub generated types
// trip no `dead_code` lint.

/// Emit ONE `compile_error!` if any field name uses the reserved `__cer`
/// prefix. Codegen owns every generated `__cer_*` identifier
/// (`__cer_assign_<f>` write shims, `__cer_wp_*` proxy types, `__cer_wg_*`
/// marker traits), so a user field starting with `__cer` could shadow that
/// plumbing. Runs at codegen entry for BOTH fixed and variable schemas —
/// the universal choke point (the reserved-accessor collision check in
/// `collect_reserved_collisions` only runs for variable schemas). No-op when
/// no field offends. ROS2 `.msg` field names never start with `__cer`, so
/// this only fires on hand-authored workspace YAML schemas.
pub(super) fn emit_reserved_prefix_compile_error(out: &mut String, schema: &MessageSchema) {
    let offenders: Vec<String> = schema
        .fields
        .iter()
        .filter(|f| f.name.starts_with("__cer"))
        .map(|f| format!("`{}`", f.name))
        .collect();
    if offenders.is_empty() {
        return;
    }
    let list = offenders.join(", ");
    writeln!(
        out,
        "// Reserved `__cer` field-name prefix — see compile_error! below."
    )
    .unwrap();
    writeln!(
        out,
        "compile_error!(\"schema `{schema_name}`: field name(s) {list} use the reserved `__cer` prefix. Cerulion reserves any field name beginning with `__cer` for generated internal plumbing (the `__cer_assign_<field>` / `__cer_fill_from_<field>` write shims and the variable-field diagnostic proxies). Rename the offending field(s) — a schema field name must not begin with `__cer`.\");",
        schema_name = schema.name,
        list = list,
    )
    .unwrap();
    writeln!(out).unwrap();
}

/// Emit the explicit `emit()` publish gesture for a ZERO-FIELD
/// output schema (e.g. `std_msgs/Empty`), on the fixed `<Name>Shm`.
///
/// The macro lazy-loan loans an `#[output]` port's proxy on its FIRST
/// write, so an output the tick never writes never loans, never publishes — a
/// zero-traffic non-event. A FIELDLESS schema has no field to write, so its
/// write shims never fire and it has NO way to express "publish me". This
/// gesture is that way: user code writes `self.<port>.emit()?` in `tick`; the
/// macro rewrites the receiver to the lazy get-or-loan `__cer_<port>.__cer_loan()?`
/// (loaning the proxy — the publish intent), the tick tail's `__cer_arm_if_loaned`
/// arms it on a fully-Ok tick, and the loaned proxy publishes on Drop. The
/// method body is a trivial `Ok(())`: the loan IS the intent, so `emit()` only
/// needs to EXIST and resolve (through `OutputProxy::DerefMut`) to give the
/// receiver a call to hang off. A fieldless proxy is vacuously complete — its
/// `VARIABLE_FIELD_COUNT` is 0, so the `OutputProxy::Drop` all-variables discard
/// gate cannot trip — so an armed `emit()`ted proxy always publishes cleanly.
///
/// The caller gates this on `schema.fields.is_empty()`, so `emit()` exists ONLY
/// for zero-field schemas — calling it on any output that HAS a field is a
/// compile error (no such method); those ports publish by WRITING a field, not
/// by an ambiguous bare gesture. (A zero-field schema has no field name, so
/// `emit` can never collide with a generated field accessor.)
fn emit_zero_field_publish_gesture(out: &mut String) {
    writeln!(
        out,
        "    /// Publish this fieldless message — a heartbeat / pulse."
    )
    .unwrap();
    writeln!(out, "    ///").unwrap();
    writeln!(
        out,
        "    /// A zero-field schema (e.g. `std_msgs/Empty`) has no field to write, so under"
    )
    .unwrap();
    writeln!(
        out,
        "    /// the macro lazy-loan it needs an explicit gesture to publish: call"
    )
    .unwrap();
    writeln!(
        out,
        "    /// `self.<output>.emit()?` in your `tick`. Only outputs you `emit()` are"
    )
    .unwrap();
    writeln!(
        out,
        "    /// published — an un-emitted fieldless output is a zero-traffic non-event,"
    )
    .unwrap();
    writeln!(out, "    /// exactly like an unwritten fielded output.").unwrap();
    writeln!(out, "    #[inline]").unwrap();
    writeln!(
        out,
        "    pub fn emit(&mut self) -> ::std::result::Result<(), ::cerulion_core::TransportError> {{"
    )
    .unwrap();
    writeln!(out, "        ::std::result::Result::Ok(())").unwrap();
    writeln!(out, "    }}").unwrap();
    writeln!(out).unwrap();
}

/// Emit the uniform `__cer_assign_<f>` write shim for a FIXED-section field
/// (on the fully-fixed `<Name>Shm` overlay, or on `<Name>FixedSection` for a
/// variable schema — both expose the field as a `pub` member reached by
/// direct assignment). The parameter is the exact generated overlay type
/// (`field_type_to_rust_shm`, e.g. `u8` for a `bool` field, `[u8; N]` for a
/// `StringFixed`/`FixedArray`), taken by reference; the body dereferences it
/// with `*v` (every fixed-section overlay type is `Copy` — the overlay struct
/// derives `Copy`), preserving the invariant that any RHS compiling as
/// `self.<f> = rhs` keeps compiling as `__cer_assign_<f>(&rhs)`.
fn emit_fixed_assign_shim(out: &mut String, field: &FieldDef) {
    let escaped = escape_keyword(&field.name);
    let raw = &field.name;
    let ty = field_type_to_rust_shm(&field.field_type);
    writeln!(
        out,
        "    /// Write shim for fixed field `{raw}` (schema-blind macro plumbing)."
    )
    .unwrap();
    writeln!(out, "    #[doc(hidden)]").unwrap();
    writeln!(out, "    #[inline]").unwrap();
    writeln!(
        out,
        "    pub fn __cer_assign_{raw}(&mut self, v: &{ty}) -> ::std::result::Result<(), ::cerulion_core::TransportError> {{"
    )
    .unwrap();
    writeln!(out, "        self.{escaped} = *v;").unwrap();
    writeln!(out, "        ::std::result::Result::Ok(())").unwrap();
    writeln!(out, "    }}").unwrap();
    writeln!(out).unwrap();
}

/// Emit the uniform `__cer_assign_<f>` write shim for a VARIABLE field (on
/// `<Name>Shm<'a>`). Delegates to the field's existing typed setter — the
/// same target the current macro rewrites `self.<port>.<f> = expr` to — so
/// the shim inherits its capacity / overflow / error semantics unchanged.
/// The parameter type mirrors that setter's parameter exactly (`&str` for
/// `String`, `&[T]` for `Bytes` / `DynamicArray<T>`, `&[u8]` for complex
/// variable fields delegating to `set_<f>_bytes`).
fn emit_variable_assign_shim(out: &mut String, field: &FieldDef) {
    let raw = &field.name;
    let ft = &field.field_type;
    let (param_ty, delegate) = if is_complex_variable(ft) {
        ("&[u8]".to_string(), format!("self.set_{raw}_bytes(v)"))
    } else {
        match ft {
            FieldType::String => ("&str".to_string(), format!("self.set_{raw}(v)")),
            FieldType::Bytes => ("&[u8]".to_string(), format!("self.set_{raw}(v)")),
            FieldType::DynamicArray { element_type } => {
                let (_sz, ty) = primitive_size_and_type(element_type);
                (format!("&[{ty}]"), format!("self.set_{raw}(v)"))
            }
            // Fixed-section field types never reach here (caller iterates
            // variable fields only); nothing to emit if one somehow did.
            _ => return,
        }
    };
    writeln!(
        out,
        "    /// Write shim for variable field `{raw}` (schema-blind macro plumbing)."
    )
    .unwrap();
    writeln!(out, "    #[doc(hidden)]").unwrap();
    writeln!(out, "    #[inline]").unwrap();
    writeln!(
        out,
        "    pub fn __cer_assign_{raw}(&mut self, v: {param_ty}) -> ::std::result::Result<(), ::cerulion_core::TransportError> {{"
    )
    .unwrap();
    writeln!(out, "        {delegate}").unwrap();
    writeln!(out, "    }}").unwrap();
    writeln!(out).unwrap();
}

/// Emit the uniform `__cer_fill_from_<f>` shim for a VARIABLE field (on
/// `<Name>Shm<'a>`). The schema-blind macro rewriter rewrites
/// `self.<port>.<f>.fill_from(src)` to `__cer_<port>.__cer_fill_from_<f>(src)`
/// without knowing whether the field is simple or complex; this shim resolves
/// that here, delegating to the existing `fill_from_<f>` (simple) or
/// `fill_from_<f>_bytes` (complex) with semantics unchanged. Generic over the
/// same `FillFrom<elem>` bound as the delegate.
fn emit_variable_fill_from_shim(out: &mut String, field: &FieldDef) {
    let raw = &field.name;
    let ft = &field.field_type;
    let (elem_ty, delegate) = if is_complex_variable(ft) {
        ("u8".to_string(), format!("self.fill_from_{raw}_bytes(src)"))
    } else {
        match ft {
            FieldType::String | FieldType::Bytes => {
                ("u8".to_string(), format!("self.fill_from_{raw}(src)"))
            }
            FieldType::DynamicArray { element_type } => {
                let (_sz, ty) = primitive_size_and_type(element_type);
                (ty.to_string(), format!("self.fill_from_{raw}(src)"))
            }
            // Fixed-section field types never reach here (caller iterates
            // variable fields only); nothing to emit if one somehow did.
            _ => return,
        }
    };
    writeln!(
        out,
        "    /// `fill_from` shim for variable field `{raw}` (schema-blind macro plumbing)."
    )
    .unwrap();
    writeln!(out, "    #[doc(hidden)]").unwrap();
    writeln!(out, "    #[inline]").unwrap();
    writeln!(
        out,
        "    pub fn __cer_fill_from_{raw}<__CerS>(&mut self, src: __CerS) -> ::std::result::Result<(), ::cerulion_core::TransportError>"
    )
    .unwrap();
    writeln!(out, "    where").unwrap();
    writeln!(
        out,
        "        __CerS: ::cerulion_core::transport::fill_from::FillFrom<{elem_ty}>,"
    )
    .unwrap();
    writeln!(out, "    {{").unwrap();
    writeln!(out, "        {delegate}").unwrap();
    writeln!(out, "    }}").unwrap();
    writeln!(out).unwrap();
}

/// Emit the `pub <f>: <ProxyTy>,` field declarations (one per VARIABLE field)
/// inside the `<Name>Shm<'a>` struct body. `pub` is required so user code in a
/// downstream node crate can reach the field for the diagnostic; `#[doc(hidden)]`
/// keeps it out of rustdoc. `<Name>Shm<'a>` retains private fields, so this
/// does not make the struct externally constructible.
fn emit_proxy_field_decls(out: &mut String, schema: &MessageSchema) {
    for field in &schema.fields {
        if !var_field_gets_proxy(field) {
            continue;
        }
        let escaped = escape_keyword(&field.name);
        let raw = &field.name;
        let proxy = var_field_proxy_type_name(&schema.name, raw);
        // Decision: READING the proxy field must WARN. The
        // parens-typo on a variable-field read (`self.<port>.<f>` instead
        // of `self.<port>.<f>()`) used to be a loud E0609; with the proxy
        // it silently binds a useless ZST — `#[deprecated]` turns that
        // into a deprecation warning pointing at the right accessor. The
        // reader named in the note is kind-correct: `<f>()` for simple
        // variable fields, `<f>_bytes()` for complex ones — and KEYWORD-
        // ESCAPED for simple readers (a field named `type` has reader
        // `r#type()`; `<f>_bytes` idents are suffixed, never keywords).
        // The write mention uses the escaped form too: user source writes
        // `self.<port>.r#type = expr`.
        let reader = if is_complex_variable(&field.field_type) {
            format!("{raw}_bytes()")
        } else {
            format!("{escaped}()")
        };
        writeln!(
            out,
            "    // Write-only diagnostic proxy for variable field `{raw}`."
        )
        .unwrap();
        writeln!(out, "    #[doc(hidden)]").unwrap();
        writeln!(
            out,
            "    #[deprecated(note = \"write-only proxy for variable field `{raw}` — reading it yields a useless marker. Read with the accessor method `{reader}`; write with `self.<port>.{escaped} = expr` or `.fill_from(...)`.\")]"
        )
        .unwrap();
        writeln!(out, "    pub {escaped}: {proxy},").unwrap();
    }
}

/// Emit the `<f>: <ProxyTy>,` initializers (one per VARIABLE field) inside a
/// `<Name>Shm<'a>` constructor's `Self { .. }` literal. The proxy is a unit
/// struct, so the initializer value is the type name itself.
fn emit_proxy_field_inits(out: &mut String, schema: &MessageSchema) {
    for field in &schema.fields {
        if !var_field_gets_proxy(field) {
            continue;
        }
        let escaped = escape_keyword(&field.name);
        let proxy = var_field_proxy_type_name(&schema.name, &field.name);
        writeln!(out, "            {escaped}: {proxy},").unwrap();
    }
}

/// True iff this variable field receives a write-only diagnostic proxy FIELD
/// on `<Name>Shm<'a>`. Fixed-section fields never do (they resolve through the
/// `DerefMut` to `<Name>FixedSection`). A variable field whose name shadows a
/// PRIVATE `<Name>Shm<'a>` field (`ptr` / `len` / `state` / `overflow` /
/// `max_capacity` / `topic` / `_phantom`) also gets NO proxy — Rust forbids
/// two struct fields of the same name regardless of visibility, and such
/// field names are legal (`moveit_msgs/DisplayRobotState` has a `state`
/// field). Those fields still get the `__cer_assign_<f>` write shim (a method,
/// not a field, so no clash); they simply forgo the `[i]`/`+=` diagnostic.
fn var_field_gets_proxy(field: &FieldDef) -> bool {
    !is_fixed_section_field(&field.field_type) && !SHM_PRIVATE_FIELDS.contains(&field.name.as_str())
}

/// The generated proxy-type identifier for variable field `<field>` of schema
/// `<name>`. Uses the RAW (un-escaped) field name — a keyword like `type`
/// embeds fine as an identifier SUBSTRING (`__cer_wp_Image_type` is valid),
/// while the struct FIELD named after it still needs `escape_keyword`.
fn var_field_proxy_type_name(schema_name: &str, field_name: &str) -> String {
    format!("__cer_wp_{schema_name}_{field_name}")
}

/// The generated never-implemented marker-trait identifier that gates the
/// proxy's operator impls (carries the `#[diagnostic::on_unimplemented]`
/// message).
fn var_field_proxy_guard_name(schema_name: &str, field_name: &str) -> String {
    format!("__cer_wg_{schema_name}_{field_name}")
}

/// Emit, per VARIABLE field, the write-only diagnostic proxy machinery:
///
/// - a never-implemented marker trait carrying
///   `#[diagnostic::on_unimplemented(message = "...")]`,
/// - the zero-sized proxy struct (the type of the same-named field on
///   `<Name>Shm<'a>`), and
/// - `Index<usize>` / `IndexMut<usize>` + compound-assign (`+=`/`-=`/`*=`/`/=`)
///   impls, each gated on the unsatisfiable marker bound so `proxy[i]` and
///   `proxy += x` surface E0277 rendered with the codegen-authored message
///   instead of a bare "no field / cannot index" error.
///
/// The proxy field is write-only by construction: there is no operator to
/// intercept a plain read (`let x = self.<port>.<f>;`), which yields the
/// (useless, `Copy`) ZST value — acceptable by design. Blocking the
/// common mutation mistakes (`[i]`, `+=`) is the goal; the trybuild pins for
/// the exact rendered message live in `tests/ui/type_error/`.
fn generate_variable_field_proxies(out: &mut String, schema: &MessageSchema) {
    for field in &schema.fields {
        if !var_field_gets_proxy(field) {
            continue;
        }
        let raw = &field.name;
        let proxy = var_field_proxy_type_name(&schema.name, raw);
        let guard = var_field_proxy_guard_name(&schema.name, raw);
        // The diagnostic message (real field name substituted; `<port>` stays
        // generic — codegen cannot know the port). `self.<port>.<f> = expr`
        // (macro-rewritten to the `__cer_assign_<f>` shim) and
        // `self.<port>.<f>.fill_from(...)` are the two supported write forms.
        let msg = format!(
            "self.<port>.{raw}[i] / += are unsupported on variable-length fields. Replace the whole value with self.<port>.{raw} = expr, or write incrementally with self.<port>.{raw}.fill_from(|buf| …)."
        );

        // Never-implemented marker trait carrying the diagnostic.
        writeln!(
            out,
            "/// Never-implemented guard for the write-only proxy of variable field `{raw}`."
        )
        .unwrap();
        writeln!(out, "#[doc(hidden)]").unwrap();
        writeln!(out, "#[allow(non_camel_case_types)]").unwrap();
        writeln!(out, "#[diagnostic::on_unimplemented(message = \"{msg}\")]").unwrap();
        writeln!(out, "pub trait {guard} {{}}").unwrap();

        // Zero-sized proxy struct.
        writeln!(
            out,
            "/// Write-only diagnostic proxy for variable field `{raw}` (zero-sized)."
        )
        .unwrap();
        writeln!(out, "#[doc(hidden)]").unwrap();
        writeln!(out, "#[allow(non_camel_case_types)]").unwrap();
        writeln!(out, "#[derive(Clone, Copy)]").unwrap();
        writeln!(out, "pub struct {proxy};").unwrap();

        // Index / IndexMut — the marker bound sits on the impl's GENERIC index
        // type (`__CerIdx: <guard>`), never on the concrete proxy (a bound on a
        // concrete type is a `trivial_bounds` case, unstable on stable Rust).
        // `<guard>` is never implemented, so `proxy[i]` — which needs
        // `<idx-type>: <guard>` — fails with the on_unimplemented diagnostic.
        // Bodies are unreachable (the bound can never hold) but must type-check.
        writeln!(
            out,
            "impl<__CerIdx> ::std::ops::Index<__CerIdx> for {proxy} where __CerIdx: {guard} {{"
        )
        .unwrap();
        writeln!(out, "    type Output = ();").unwrap();
        writeln!(
            out,
            "    #[inline] fn index(&self, _i: __CerIdx) -> &Self::Output {{ unreachable!() }}"
        )
        .unwrap();
        writeln!(out, "}}").unwrap();
        writeln!(
            out,
            "impl<__CerIdx> ::std::ops::IndexMut<__CerIdx> for {proxy} where __CerIdx: {guard} {{"
        )
        .unwrap();
        writeln!(
            out,
            "    #[inline] fn index_mut(&mut self, _i: __CerIdx) -> &mut Self::Output {{ unreachable!() }}"
        )
        .unwrap();
        writeln!(out, "}}").unwrap();

        // Compound-assign ops — the marker bound sits on the generic RHS type
        // (`__CerRhs: <guard>`), so `proxy += x` fails with the diagnostic.
        for (op_trait, method) in [
            ("AddAssign", "add_assign"),
            ("SubAssign", "sub_assign"),
            ("MulAssign", "mul_assign"),
            ("DivAssign", "div_assign"),
        ] {
            writeln!(
                out,
                "impl<__CerRhs> ::std::ops::{op_trait}<__CerRhs> for {proxy} where __CerRhs: {guard} {{"
            )
            .unwrap();
            writeln!(
                out,
                "    #[inline] fn {method}(&mut self, _rhs: __CerRhs) {{}}"
            )
            .unwrap();
            writeln!(out, "}}").unwrap();
        }
        writeln!(out).unwrap();
    }
}

// =============================================================================
// Nested-writer sugar — staged scratch for complex-nested
// fields + leaf-composing accessors for fixed-nested substructs.
// =============================================================================
//
// `self.image.header.frame_id = "cam0";` (the macro rewrites the 3-segment
// path into the closure form below) and the public `with_<f>(|h| …)` both
// need a WRITABLE nested view. Two shapes:
//
// - FIXED nested (`Imu.orientation`): the field IS the target's own fixed
//   overlay type (`QuaternionShm`, which already carries the
//   `__cer_assign_<leaf>` shims), inline in the FixedSection — the accessor
//   is a plain `&mut` projection and the leaf chain composes for free,
//   recursively (a fixed-nested inside a fixed-nested gets its own
//   accessors on ITS overlay type).
//
// - COMPLEX-VARIABLE nested (`Image.header`): the target lives in the
//   variable payload as raw bytes; writes are STAGED in a per-field heap
//   scratch (`shm_runtime::StagedNested`) with the nested writer's state
//   persisted across accessor calls (fresh view per call via
//   `__cer_staging_resume` — never a stored self-referential view), then
//   FLUSHED through the existing `set_<f>_bytes` at `OutputProxy::Drop`
//   (before the all-variables gate). Scratch growth rides the target's
//   spill: the nested view's `max_capacity` is the PARENT's cap,
//   so an over-scratch write spills inside the nested writer and the
//   accessor harvests the spill buffer as the new scratch.

/// True iff the schema declares ≥1 complex VARIABLE field. The
/// per-schema staging/flush machinery is emitted UNCONDITIONALLY on
/// variable schemas (recursion needs it on every possible TARGET); this gate
/// remains only for the `ShmMessage::flush_staged_nested` trait override
/// (`wire_impl.rs`) — Drop needs the override only when slots can exist.
pub(super) fn schema_has_complex_variable(schema: &MessageSchema) -> bool {
    schema
        .fields
        .iter()
        .any(|f| f.field_type.is_variable() && is_complex_variable(&f.field_type))
}

/// The nested TARGET's generated writer type name for a complex-nested
/// field (`Header` → `HeaderShm`). Mirrors `field_type_to_rust_shm`'s
/// Nested arm; complex-variable fields are `Nested { fixed: None }` by
/// construction (see `is_complex_variable`) — arrays-of-nested and other
/// complex shapes get NO nested-writer sugar (raw `set_<f>_bytes` only).
/// Doc emission only: is this field's nested schema exactly
/// `std_msgs/Header`? Gates the worked example in the emitted `with_<field>`
/// doc, whose leaves (`frame_id`, `stamp`) exist on no other schema. Requires
/// the QUALIFIED reference, so a workspace schema that happens to be named
/// `Header` never inherits an example written for the ROS one.
fn nested_is_std_msgs_header(ft: &FieldType) -> bool {
    matches!(
        ft,
        FieldType::Nested { schema_name, package: Some(package), .. }
            if schema_name == "Header" && package == "std_msgs"
    )
}

fn complex_nested_target(ft: &FieldType) -> Option<String> {
    match ft {
        FieldType::Nested {
            schema_name,
            fixed: None,
            ..
        } => Some(format!("{schema_name}Shm")),
        _ => None,
    }
}

/// Guard A: emit the "staged then whole" conflict check for a
/// whole-field writer (`loan_<f>_bytes` / `fill_from_<f>_bytes`) of a
/// complex-nested field. `raw` is the field's raw (unescaped) name — it
/// appears both in the (prefix-guarded, never-a-keyword) `__cer_staged_<raw>`
/// ident and as a plain string literal in the error. Body-level indentation
/// (8 spaces) — emitted at the top of the writer's method body.
fn emit_staged_then_whole_conflict_guard(out: &mut String, raw: &str) {
    writeln!(out, "        if self.__cer_staged_{raw}.is_some() {{").unwrap();
    writeln!(
        out,
        "            return ::std::result::Result::Err(::cerulion_core::TransportError::NestedWriteConflict {{"
    )
    .unwrap();
    writeln!(out, "                field: \"{raw}\",").unwrap();
    writeln!(
        out,
        "                detected: \"a staged nested-field write, then a whole-field write\","
    )
    .unwrap();
    writeln!(out, "            }});").unwrap();
    writeln!(out, "        }}").unwrap();
}

/// Guard B: emit the "whole then staged" conflict check for the
/// staged accessor `__cer_with_nested_<f>` of a complex-nested field. Errors
/// if the PARENT already wrote the field's `var_idx` via a whole-field write.
/// Body-level indentation (8 spaces) — emitted at the top of the accessor.
fn emit_whole_then_staged_conflict_guard(out: &mut String, raw: &str, var_idx: usize) {
    writeln!(out, "        if self.state.is_written({var_idx}) {{").unwrap();
    writeln!(
        out,
        "            return ::std::result::Result::Err(::cerulion_core::TransportError::NestedWriteConflict {{"
    )
    .unwrap();
    writeln!(out, "                field: \"{raw}\",").unwrap();
    writeln!(
        out,
        "                detected: \"a whole-field write, then a staged nested-field write\","
    )
    .unwrap();
    writeln!(out, "            }});").unwrap();
    writeln!(out, "        }}").unwrap();
}

/// Emit the private staging-slot field declarations on `<Name>Shm<'a>` —
/// one per complex-nested variable field with a resolvable target type.
fn emit_staged_field_decls(out: &mut String, schema: &MessageSchema) {
    for field in &schema.fields {
        if !field.field_type.is_variable() || !is_complex_variable(&field.field_type) {
            continue;
        }
        if complex_nested_target(&field.field_type).is_none() {
            continue;
        }
        let raw = &field.name;
        writeln!(
            out,
            "    /// Staging slot for nested-writer sugar on `{raw}`."
        )
        .unwrap();
        writeln!(
            out,
            "    __cer_staged_{raw}: ::std::option::Option<::std::boxed::Box<::cerulion_core::shm_runtime::StagedNested>>,"
        )
        .unwrap();
    }
}

/// Emit the `__cer_staged_<f>: None,` initializers for a `<Name>Shm<'a>`
/// constructor's `Self { .. }` literal.
fn emit_staged_field_inits(out: &mut String, schema: &MessageSchema) {
    for field in &schema.fields {
        if !field.field_type.is_variable() || !is_complex_variable(&field.field_type) {
            continue;
        }
        if complex_nested_target(&field.field_type).is_none() {
            continue;
        }
        let raw = &field.name;
        writeln!(
            out,
            "            __cer_staged_{raw}: ::std::option::Option::None,"
        )
        .unwrap();
    }
}

/// Emit `__cer_staging_resume` — the doc-hidden resume-state sibling of
/// `from_bytes_mut`. Rebuilds a writer over persisted staging scratch
/// WITHOUT zeroing the offset table and WITH `WriterState` restored from
/// the exported `(cursor, written, field_starts)` triple. `from_bytes_mut`
/// keeps its reset semantics untouched.
fn generate_var_shm_staging_resume(
    out: &mut String,
    schema: &MessageSchema,
    shm_name: &str,
    fixed_section_name: &str,
) {
    // NOTE: emitted INSIDE the open `impl<'a> <Name>Shm<'a>` block (called
    // from `generate_var_shm_from_bytes_mut`) — method only, no impl header.
    writeln!(
        out,
        "    /// Resume-state constructor for the STAGING path."
    )
    .unwrap();
    writeln!(out, "    ///").unwrap();
    writeln!(
        out,
        "    /// Like [`from_bytes_mut`](Self::from_bytes_mut) but does NOT zero the"
    )
    .unwrap();
    writeln!(
        out,
        "    /// offset table (the scratch persists across accessor calls) and"
    )
    .unwrap();
    writeln!(out, "    /// restores the writer state exported by a prior").unwrap();
    writeln!(
        out,
        "    /// [`__cer_staging_export`](Self::__cer_staging_export). Called only by"
    )
    .unwrap();
    writeln!(
        out,
        "    /// generated `__cer_with_nested_<f>` accessors of an EMBEDDING schema."
    )
    .unwrap();
    writeln!(out, "    #[doc(hidden)]").unwrap();
    writeln!(out, "    #[inline]").unwrap();
    writeln!(
        out,
        "    pub fn __cer_staging_resume(bytes: &'a mut [u8], max_capacity: ::cerulion_core::wire::MaxPayloadCapacity, topic: ::std::sync::Arc<str>, cursor: u32, written: u64, field_starts: &[u32]) -> Self {{"
    )
    .unwrap();
    writeln!(
        out,
        "        let align = ::std::mem::align_of::<{fixed_section_name}>();"
    )
    .unwrap();
    writeln!(
        out,
        "        let table_end = Self::OFFSET_TABLE_OFFSET + Self::OFFSET_TABLE_BYTES;"
    )
    .unwrap();
    writeln!(
        out,
        "        assert!(bytes.len() >= table_end, \"{shm_name}::__cer_staging_resume: scratch too small ({{}} < {{}})\", bytes.len(), table_end);"
    )
    .unwrap();
    writeln!(
        out,
        "        assert!(bytes.as_mut_ptr().align_offset(align) == 0, \"{shm_name}::__cer_staging_resume: misaligned scratch (need align {{}})\", align);"
    )
    .unwrap();
    writeln!(
        out,
        "        debug_assert_eq!(field_starts.len(), Self::VARIABLE_FIELD_COUNT, \"staging state field_starts length mismatch\");"
    )
    .unwrap();
    writeln!(
        out,
        "        let mut state = ::cerulion_core::shm_runtime::WriterState::new(Self::WIRE_FIXED_SIZE as u32);"
    )
    .unwrap();
    writeln!(out, "        state.cursor = cursor;").unwrap();
    writeln!(out, "        state.written = written;").unwrap();
    writeln!(
        out,
        "        let n = field_starts.len().min(Self::VARIABLE_FIELD_COUNT);"
    )
    .unwrap();
    writeln!(
        out,
        "        state.field_starts[..n].copy_from_slice(&field_starts[..n]);"
    )
    .unwrap();
    writeln!(out, "        let len = bytes.len();").unwrap();
    writeln!(out, "        let ptr = bytes.as_mut_ptr();").unwrap();
    writeln!(out, "        let _ = bytes;").unwrap();
    writeln!(out, "        #[allow(deprecated)]").unwrap();
    writeln!(out, "        let __cer_shm = Self {{").unwrap();
    writeln!(out, "            ptr,").unwrap();
    writeln!(out, "            len,").unwrap();
    writeln!(out, "            state,").unwrap();
    writeln!(out, "            overflow: ::std::option::Option::None,").unwrap();
    writeln!(out, "            max_capacity,").unwrap();
    writeln!(out, "            topic,").unwrap();
    writeln!(out, "            _phantom: ::std::marker::PhantomData,").unwrap();
    emit_proxy_field_inits(out, schema);
    emit_staged_field_inits(out, schema);
    writeln!(out, "        }};").unwrap();
    writeln!(out, "        __cer_shm").unwrap();
    writeln!(out, "    }}").unwrap();
    writeln!(out).unwrap();
}

/// Emit `__cer_staging_export` + `__cer_staging_take_overflow` — the
/// harvest half of the staging round-trip, emitted for EVERY variable
/// schema (any variable schema can be another schema's nested target).
fn generate_var_shm_staging_export_take(out: &mut String, shm_name: &str) {
    writeln!(
        out,
        "    /// Export the writer state for staging persistence."
    )
    .unwrap();
    writeln!(out, "    ///").unwrap();
    writeln!(
        out,
        "    /// Copies `field_starts` into the caller's buffer (no allocation) and"
    )
    .unwrap();
    writeln!(out, "    /// returns `(cursor, written)`. Inverse of").unwrap();
    writeln!(
        out,
        "    /// [`__cer_staging_resume`](Self::__cer_staging_resume)."
    )
    .unwrap();
    writeln!(out, "    #[doc(hidden)]").unwrap();
    writeln!(out, "    #[inline]").unwrap();
    writeln!(
        out,
        "    pub fn __cer_staging_export(&self, field_starts_out: &mut [u32]) -> (u32, u64) {{"
    )
    .unwrap();
    writeln!(
        out,
        "        let n = field_starts_out.len().min(Self::VARIABLE_FIELD_COUNT);"
    )
    .unwrap();
    writeln!(
        out,
        "        field_starts_out[..n].copy_from_slice(&self.state.field_starts[..n]);"
    )
    .unwrap();
    writeln!(out, "        (self.state.cursor, self.state.written)").unwrap();
    writeln!(out, "    }}").unwrap();
    writeln!(out).unwrap();
    writeln!(
        out,
        "    /// Take the overflow-spill buffer for staging growth."
    )
    .unwrap();
    writeln!(out, "    ///").unwrap();
    writeln!(
        out,
        "    /// The staging accessor adopts a spilled buffer as the NEW scratch"
    )
    .unwrap();
    writeln!(
        out,
        "    /// (growth path). Invalidates `ptr`/`len` — the caller exports state"
    )
    .unwrap();
    writeln!(
        out,
        "    /// FIRST and drops the view immediately after, never touching the"
    )
    .unwrap();
    writeln!(
        out,
        "    /// payload again (unlike the test-only `take_overflow`, this is the"
    )
    .unwrap();
    writeln!(
        out,
        "    /// always-on production twin with a single disciplined call site)."
    )
    .unwrap();
    writeln!(out, "    #[doc(hidden)]").unwrap();
    writeln!(out, "    #[inline]").unwrap();
    writeln!(
        out,
        "    pub fn __cer_staging_take_overflow(&mut self) -> ::std::option::Option<::std::boxed::Box<[u64]>> {{"
    )
    .unwrap();
    writeln!(out, "        self.overflow.take()").unwrap();
    writeln!(out, "    }}").unwrap();
    writeln!(out).unwrap();
    let _ = shm_name; // reserved for future doc use
}

/// Recursive staging persistence: emit
/// `__cer_staging_take_children` / `__cer_staging_restore_children` on EVERY
/// variable schema — like export/take_overflow, any variable schema can be
/// the TARGET of another schema's complex-nested field, and the embedding
/// accessor/flush cannot know (registry gap) whether the target itself
/// embeds complex-nested fields, so the pair must exist unconditionally.
/// For a target with no complex-nested fields, take returns an empty vec and
/// restore warns per entry (nothing legal to restore into).
fn generate_var_shm_staging_children(out: &mut String, schema: &MessageSchema, shm_name: &str) {
    let staged_fields: Vec<&str> = schema
        .fields
        .iter()
        .filter(|f| {
            f.field_type.is_variable()
                && is_complex_variable(&f.field_type)
                && complex_nested_target(&f.field_type).is_some()
        })
        .map(|f| f.name.as_str())
        .collect();

    writeln!(
        out,
        "    /// Drain this writer's staged complex-nested children"
    )
    .unwrap();
    writeln!(
        out,
        "    /// (keyed by raw field name) so an EMBEDDING accessor can persist them"
    )
    .unwrap();
    writeln!(
        out,
        "    /// across transient views. Inverse of `__cer_staging_restore_children`."
    )
    .unwrap();
    writeln!(out, "    #[doc(hidden)]").unwrap();
    writeln!(
        out,
        "    pub fn __cer_staging_take_children(&mut self) -> ::std::vec::Vec<(::std::string::String, ::cerulion_core::shm_runtime::StagedNested)> {{"
    )
    .unwrap();
    if staged_fields.is_empty() {
        writeln!(out, "        ::std::vec::Vec::new()").unwrap();
    } else {
        writeln!(out, "        let mut __cer_out = ::std::vec::Vec::new();").unwrap();
        for raw in &staged_fields {
            writeln!(
                out,
                "        if let ::std::option::Option::Some(staged) = self.__cer_staged_{raw}.take() {{"
            )
            .unwrap();
            writeln!(
                out,
                "            __cer_out.push((::std::string::String::from(\"{raw}\"), *staged));"
            )
            .unwrap();
            writeln!(out, "        }}").unwrap();
        }
        writeln!(out, "        __cer_out").unwrap();
    }
    writeln!(out, "    }}").unwrap();
    writeln!(out).unwrap();

    writeln!(
        out,
        "    /// Restore previously-taken staged children into this"
    )
    .unwrap();
    writeln!(
        out,
        "    /// (fresh) view's staging slots. An unknown name is a LOUD `Err`"
    )
    .unwrap();
    writeln!(
        out,
        "    /// (`TransportError::Internal`) — it can only come from a codegen bug"
    )
    .unwrap();
    writeln!(
        out,
        "    /// or a take/restore emitter drift, never from user code. The Err"
    )
    .unwrap();
    writeln!(
        out,
        "    /// routes to the loud discard at flush and to the user's tick at the"
    )
    .unwrap();
    writeln!(
        out,
        "    /// accessor — NEVER a warn-only silent drop of staged data, and NEVER"
    )
    .unwrap();
    writeln!(
        out,
        "    /// a `debug_assert` panic reachable inside `OutputProxy::Drop`"
    )
    .unwrap();
    writeln!(out, "    /// (Drop must not panic).").unwrap();
    writeln!(out, "    #[doc(hidden)]").unwrap();
    writeln!(
        out,
        "    pub fn __cer_staging_restore_children(&mut self, children: ::std::vec::Vec<(::std::string::String, ::cerulion_core::shm_runtime::StagedNested)>) -> ::std::result::Result<(), ::cerulion_core::TransportError> {{"
    )
    .unwrap();
    if staged_fields.is_empty() {
        // No slots: every entry is unknown — error on the first one. The
        // payload is bound to `_` so the generated code carries no unused
        // variable under deny lints.
        writeln!(
            out,
            "        if let ::std::option::Option::Some((__cer_name, _)) = children.into_iter().next() {{"
        )
        .unwrap();
        writeln!(
            out,
            "            return ::std::result::Result::Err(::cerulion_core::TransportError::Internal {{"
        )
        .unwrap();
        writeln!(
            out,
            "                reason: ::std::format!(\"unknown staged child field '{{}}' on {shm_name} — take/restore emitter drift\", __cer_name),"
        )
        .unwrap();
        writeln!(out, "            }});").unwrap();
        writeln!(out, "        }}").unwrap();
    } else {
        writeln!(out, "        for (__cer_name, __cer_staged) in children {{").unwrap();
        for (i, raw) in staged_fields.iter().enumerate() {
            let kw = if i == 0 { "if" } else { "} else if" };
            writeln!(out, "            {kw} __cer_name == \"{raw}\" {{").unwrap();
            writeln!(
                out,
                "                self.__cer_staged_{raw} = ::std::option::Option::Some(::std::boxed::Box::new(__cer_staged));"
            )
            .unwrap();
        }
        writeln!(out, "            }} else {{").unwrap();
        writeln!(
            out,
            "                return ::std::result::Result::Err(::cerulion_core::TransportError::Internal {{"
        )
        .unwrap();
        writeln!(
            out,
            "                    reason: ::std::format!(\"unknown staged child field '{{}}' on {shm_name} — take/restore emitter drift\", __cer_name),"
        )
        .unwrap();
        writeln!(out, "                }});").unwrap();
        writeln!(out, "            }}").unwrap();
        writeln!(out, "        }}").unwrap();
    }
    writeln!(out, "        ::std::result::Result::Ok(())").unwrap();
    writeln!(out, "    }}").unwrap();
    writeln!(out).unwrap();
}

/// Emit `__cer_with_nested_<f>` (doc-hidden, the macro rewriter's target)
/// plus the PUBLIC `with_<f>` closure API for one complex-nested variable
/// field.
fn emit_with_nested_complex(out: &mut String, field: &FieldDef, var_idx: usize) {
    let raw = &field.name;
    let Some(target) = complex_nested_target(&field.field_type) else {
        return;
    };
    // Doc-hidden primitive.
    writeln!(
        out,
        "    /// Primitive behind `with_{raw}` (schema-blind macro plumbing)."
    )
    .unwrap();
    writeln!(out, "    #[doc(hidden)]").unwrap();
    writeln!(
        out,
        "    pub fn __cer_with_nested_{raw}<__CerF>(&mut self, f: __CerF) -> ::std::result::Result<(), ::cerulion_core::TransportError>"
    )
    .unwrap();
    writeln!(out, "    where").unwrap();
    writeln!(
        out,
        "        __CerF: ::std::ops::FnOnce(&mut {target}<'_>) -> ::std::result::Result<(), ::cerulion_core::TransportError>,"
    )
    .unwrap();
    writeln!(out, "    {{").unwrap();
    // Guard B: reject staging a field the PARENT already wrote
    // wholesale (the "whole then staged" conflict). Runs BEFORE any staging
    // allocation so a rejected call leaves the scratch untouched.
    emit_whole_then_staged_conflict_guard(out, raw, var_idx);
    writeln!(out, "        let max_capacity = self.max_capacity;").unwrap();
    writeln!(
        out,
        "        let topic = ::std::sync::Arc::clone(&self.topic);"
    )
    .unwrap();
    writeln!(out, "        if self.__cer_staged_{raw}.is_none() {{").unwrap();
    writeln!(
        out,
        "            // Lazy first-touch allocation (cold path: once per loan per"
    )
    .unwrap();
    writeln!(
        out,
        "            // staged field; the flush + all later calls reuse it)."
    )
    .unwrap();
    writeln!(
        out,
        "            let floor = {target}::OFFSET_TABLE_OFFSET + {target}::OFFSET_TABLE_BYTES;"
    )
    .unwrap();
    writeln!(
        out,
        "            let staged = ::cerulion_core::shm_runtime::StagedNested::try_new(floor, {target}::VARIABLE_FIELD_COUNT, floor as u32)"
    )
    .unwrap();
    writeln!(
        out,
        "                .ok_or_else(|| ::cerulion_core::TransportError::AllocationFailed {{"
    )
    .unwrap();
    writeln!(
        out,
        "                    topic: ::std::string::String::from(::std::convert::AsRef::<str>::as_ref(&topic)),"
    )
    .unwrap();
    writeln!(
        out,
        "                    requested: floor.max(::cerulion_core::shm_runtime::STAGED_SCRATCH_FLOOR_BYTES),"
    )
    .unwrap();
    writeln!(out, "                }})?;").unwrap();
    writeln!(
        out,
        "            self.__cer_staged_{raw} = ::std::option::Option::Some(::std::boxed::Box::new(staged));"
    )
    .unwrap();
    writeln!(out, "        }}").unwrap();
    writeln!(
        out,
        "        let staged = self.__cer_staged_{raw}.as_mut().expect(\"staging slot initialized above\");"
    )
    .unwrap();
    writeln!(
        out,
        "        // Fresh view per call over (scratch, resumed state) — never a"
    )
    .unwrap();
    writeln!(
        out,
        "        // stored self-referential view. `max_capacity` is the PARENT's"
    )
    .unwrap();
    writeln!(
        out,
        "        // cap, so an over-scratch write spills INSIDE the nested writer"
    )
    .unwrap();
    writeln!(
        out,
        "        // instead of erroring; the spill is harvested below."
    )
    .unwrap();
    writeln!(
        out,
        "        let mut __cer_view = {target}::__cer_staging_resume("
    )
    .unwrap();
    writeln!(
        out,
        "            ::cerulion_core::shm_runtime::u64s_as_bytes_mut(&mut staged.scratch),"
    )
    .unwrap();
    writeln!(out, "            max_capacity,").unwrap();
    writeln!(out, "            topic,").unwrap();
    writeln!(out, "            staged.cursor,").unwrap();
    writeln!(out, "            staged.written,").unwrap();
    writeln!(out, "            &staged.field_starts,").unwrap();
    writeln!(out, "        );").unwrap();
    writeln!(
        out,
        "        // Recursive staging persistence: the fresh view starts"
    )
    .unwrap();
    writeln!(
        out,
        "        // with empty staging slots — restore the GRANDCHILD staging a"
    )
    .unwrap();
    writeln!(
        out,
        "        // prior call persisted (else a second `self.<port>.{raw}.<g>.…`"
    )
    .unwrap();
    writeln!(
        out,
        "        // chain would silently reset the first's staged writes)."
    )
    .unwrap();
    writeln!(
        out,
        "        __cer_view.__cer_staging_restore_children(::std::mem::take(&mut staged.children))?;"
    )
    .unwrap();
    writeln!(out, "        let __cer_result = f(&mut __cer_view);").unwrap();
    writeln!(
        out,
        "        // Harvest state ALWAYS (each nested setter commits-or-rewinds"
    )
    .unwrap();
    writeln!(
        out,
        "        // atomically, so post-Err state is the last good state — multi-"
    )
    .unwrap();
    writeln!(
        out,
        "        // statement closures keep their successful earlier writes)."
    )
    .unwrap();
    writeln!(
        out,
        "        let (__cer_c, __cer_w) = __cer_view.__cer_staging_export(&mut staged.field_starts);"
    )
    .unwrap();
    writeln!(out, "        staged.cursor = __cer_c;").unwrap();
    writeln!(out, "        staged.written = __cer_w;").unwrap();
    writeln!(
        out,
        "        // Persist the view's OWN staged children (grandchild"
    )
    .unwrap();
    writeln!(
        out,
        "        // sugar staged into the view's slots during `f`) — they would"
    )
    .unwrap();
    writeln!(out, "        // otherwise die with the transient view.").unwrap();
    writeln!(
        out,
        "        staged.children = __cer_view.__cer_staging_take_children();"
    )
    .unwrap();
    writeln!(
        out,
        "        let __cer_spill = __cer_view.__cer_staging_take_overflow();"
    )
    .unwrap();
    writeln!(out, "        drop(__cer_view);").unwrap();
    writeln!(
        out,
        "        if let ::std::option::Option::Some(spill) = __cer_spill {{"
    )
    .unwrap();
    writeln!(
        out,
        "            // Growth: adopt the spill (already carries every staged byte"
    )
    .unwrap();
    writeln!(
        out,
        "            // up to the cursor) as the new, parent-cap-sized scratch."
    )
    .unwrap();
    writeln!(out, "            staged.scratch = spill.into_vec();").unwrap();
    writeln!(out, "        }}").unwrap();
    writeln!(out, "        __cer_result").unwrap();
    writeln!(out, "    }}").unwrap();
    writeln!(out).unwrap();

    // Public closure API.
    writeln!(
        out,
        "    /// Write into the nested `{raw}` sub-message through a scoped writer."
    )
    .unwrap();
    writeln!(out, "    ///").unwrap();
    writeln!(
        out,
        "    /// The closure receives the nested message's own writer, [`{target}`]:"
    )
    .unwrap();
    writeln!(
        out,
        "    /// fixed leaves are written by assignment, variable-length leaves through"
    )
    .unwrap();
    writeln!(
        out,
        "    /// that type's `set_<leaf>` methods. Writes are STAGED and flushed into the"
    )
    .unwrap();
    writeln!(
        out,
        "    /// frame when the output publishes; repeated calls RESUME the staged state"
    )
    .unwrap();
    writeln!(
        out,
        "    /// (they accumulate: the second call sees the first's writes)."
    )
    .unwrap();
    writeln!(out, "    ///").unwrap();
    writeln!(
        out,
        "    /// In a node's `tick` the receiver is the output port, so the call reads"
    )
    .unwrap();
    writeln!(
        out,
        "    /// `self.<port>.with_{raw}(|w| {{ ...; Ok(()) }})?`. To write one leaf, dotted"
    )
    .unwrap();
    writeln!(
        out,
        "    /// assignment does the same with no closure: `self.<port>.{raw}.<leaf> = value;`."
    )
    .unwrap();
    // The worked example names `std_msgs/Header` leaves, so it is emitted ONLY
    // on a field whose nested schema IS `std_msgs/Header`. On any other nested
    // type (`Polygon`, `RobotState`, ...) those leaves do not exist and the
    // example would not compile.
    if nested_is_std_msgs_header(&field.field_type) {
        writeln!(out, "    ///").unwrap();
        writeln!(
            out,
            "    /// `{raw}` is a `std_msgs/Header`, so inside `tick` (`<port>` is your output"
        )
        .unwrap();
        writeln!(out, "    /// field's name):").unwrap();
        writeln!(out, "    ///").unwrap();
        writeln!(out, "    /// ```text").unwrap();
        writeln!(out, "    /// self.<port>.with_{raw}(|h| {{").unwrap();
        writeln!(out, "    ///     h.set_frame_id(\"cam0\")?;").unwrap();
        writeln!(out, "    ///     h.stamp.sec = 5;").unwrap();
        writeln!(out, "    ///     Ok(())").unwrap();
        writeln!(out, "    /// }})?;").unwrap();
        writeln!(out, "    /// ```").unwrap();
        writeln!(out, "    ///").unwrap();
        writeln!(
            out,
            "    /// (A whole node doing this compiles and runs as a doctest in the"
        )
        .unwrap();
        writeln!(out, "    /// `native_ros2_messages` crate docs.)").unwrap();
    }
    writeln!(
        out,
        "    pub fn with_{raw}<__CerF>(&mut self, f: __CerF) -> ::std::result::Result<(), ::cerulion_core::TransportError>"
    )
    .unwrap();
    writeln!(out, "    where").unwrap();
    writeln!(
        out,
        "        __CerF: ::std::ops::FnOnce(&mut {target}<'_>) -> ::std::result::Result<(), ::cerulion_core::TransportError>,"
    )
    .unwrap();
    writeln!(out, "    {{").unwrap();
    writeln!(out, "        self.__cer_with_nested_{raw}(f)").unwrap();
    writeln!(out, "    }}").unwrap();
    writeln!(out).unwrap();
}

/// Emit `__cer_flush_staged` — flushes every TOUCHED staging slot through
/// the existing `set_<f>_bytes` (marking the field written). Untouched
/// slots (`None`) flush nothing, so the write-all-variable-fields discard
/// rule is unchanged for them. Called by `ShmMessage::flush_staged_nested`
/// from `OutputProxy::Drop` BEFORE the all-variables gate — and
/// RECURSIVELY by an embedding schema's own `__cer_flush_staged` (one level
/// per schema): each staged slot's flush first restores the slot's persisted
/// grandchildren into a resumed view and runs the TARGET's `__cer_flush_staged`
/// (a target without complex fields has the no-op form — clean termination),
/// so arbitrarily deep staged chains land bottom-up. Emitted on EVERY
/// variable schema (the parent cannot know the target's shape — registry gap).
fn emit_flush_staged(out: &mut String, schema: &MessageSchema) {
    writeln!(
        out,
        "    /// Flush staged nested writes (see `ShmMessage::flush_staged_nested`)."
    )
    .unwrap();
    writeln!(out, "    #[doc(hidden)]").unwrap();
    writeln!(
        out,
        "    pub fn __cer_flush_staged(&mut self) -> ::std::result::Result<(), ::cerulion_core::TransportError> {{"
    )
    .unwrap();
    for field in &schema.fields {
        if !field.field_type.is_variable() || !is_complex_variable(&field.field_type) {
            continue;
        }
        let Some(target) = complex_nested_target(&field.field_type) else {
            continue;
        };
        let raw = &field.name;
        writeln!(
            out,
            "        if let ::std::option::Option::Some(mut staged) = self.__cer_staged_{raw}.take() {{"
        )
        .unwrap();
        // Recursion: resume the child view, restore its
        // persisted grandchildren, run the TARGET's own flush (grandchildren
        // land into the child's scratch bottom-up), THEN gate: the staged
        // child obeys the SAME every-variable-field publish rule as the top
        // level — an unwritten child variable field is a loud discard naming
        // the dotted `<parent>.<child>` location (fixed child leaves stay
        // ungated, scratch-zero default — the top level's fixed/variable
        // asymmetry). Guard interactions (verified, do not change the
        // guards): guard A cannot fire during this flush — every slot is
        // `.take()`n BEFORE its bytes route through `set_<g>_bytes`/`
        // loan_<g>_bytes` (both here and one level down), so the staging
        // slots those guards check are already `None`; guard B cannot fire —
        // no `__cer_with_nested_*` accessor runs during a flush. A user
        // mixing grandchild sugar with `with_{raw}(|m| m.set_<g>_bytes(..))`
        // correctly trips guard A on the RESTORED staging inside the
        // accessor's view — that is the one-mechanism-per-field
        // conflict rule composing, by design.
        writeln!(
            out,
            "            let mut __cer_view = {target}::__cer_staging_resume("
        )
        .unwrap();
        writeln!(
            out,
            "                ::cerulion_core::shm_runtime::u64s_as_bytes_mut(&mut staged.scratch),"
        )
        .unwrap();
        writeln!(out, "                self.max_capacity,").unwrap();
        writeln!(out, "                ::std::sync::Arc::clone(&self.topic),").unwrap();
        writeln!(out, "                staged.cursor,").unwrap();
        writeln!(out, "                staged.written,").unwrap();
        writeln!(out, "                &staged.field_starts,").unwrap();
        writeln!(out, "            );").unwrap();
        writeln!(
            out,
            "            __cer_view.__cer_staging_restore_children(::std::mem::take(&mut staged.children))?;"
        )
        .unwrap();
        // The recursive flush propagates a deeper NestedChildIncomplete with
        // THIS level's field name PREPENDED to the dotted child path, so the
        // top-level error names the full user-writable path
        // (`self.<port>.joint_trajectory.header.frame_id`), not the bare
        // deepest pair (validate-pass fix: the deepest pair suggested a
        // remediation path that does not compile). All other errors pass
        // through unchanged.
        writeln!(
            out,
            "            if let ::std::result::Result::Err(__cer_e) = __cer_view.__cer_flush_staged() {{"
        )
        .unwrap();
        writeln!(
            out,
            "                return ::std::result::Result::Err(match __cer_e {{"
        )
        .unwrap();
        writeln!(
            out,
            "                    ::cerulion_core::TransportError::NestedChildIncomplete {{ parent_field: __cer_p, child_field: __cer_c }} =>"
        )
        .unwrap();
        writeln!(
            out,
            "                        ::cerulion_core::TransportError::NestedChildIncomplete {{"
        )
        .unwrap();
        writeln!(out, "                            parent_field: \"{raw}\",").unwrap();
        writeln!(
            out,
            "                            child_field: ::std::format!(\"{{__cer_p}}.{{__cer_c}}\"),"
        )
        .unwrap();
        writeln!(out, "                        }},").unwrap();
        writeln!(out, "                    __cer_other => __cer_other,").unwrap();
        writeln!(out, "                }});").unwrap();
        writeln!(out, "            }}").unwrap();
        writeln!(
            out,
            "            if let ::std::option::Option::Some(__cer_missing) = __cer_view.__cer_first_unwritten_variable() {{"
        )
        .unwrap();
        writeln!(
            out,
            "                return ::std::result::Result::Err(::cerulion_core::TransportError::NestedChildIncomplete {{"
        )
        .unwrap();
        writeln!(out, "                    parent_field: \"{raw}\",").unwrap();
        writeln!(
            out,
            "                    child_field: ::std::string::String::from(__cer_missing),"
        )
        .unwrap();
        writeln!(out, "                }});").unwrap();
        writeln!(out, "            }}").unwrap();
        // RE-export: the recursive flush wrote grandchildren into the child's
        // scratch via its `set_<g>_bytes`, moving the cursor and possibly
        // SPILLING — harvest exactly like the accessor exit, then
        // compute `end` from the NEW cursor.
        writeln!(
            out,
            "            let (__cer_c, __cer_w) = __cer_view.__cer_staging_export(&mut staged.field_starts);"
        )
        .unwrap();
        writeln!(out, "            staged.cursor = __cer_c;").unwrap();
        writeln!(out, "            staged.written = __cer_w;").unwrap();
        writeln!(
            out,
            "            let __cer_spill = __cer_view.__cer_staging_take_overflow();"
        )
        .unwrap();
        writeln!(out, "            drop(__cer_view);").unwrap();
        writeln!(
            out,
            "            if let ::std::option::Option::Some(spill) = __cer_spill {{"
        )
        .unwrap();
        writeln!(out, "                staged.scratch = spill.into_vec();").unwrap();
        writeln!(out, "            }}").unwrap();
        // LOUD invariant check, not a defensive clamp (validate-pass fix):
        // The spill path guarantees any over-scratch write spilled and the spill
        // (sized >= cursor) was adopted above, so cursor <= scratch ALWAYS
        // holds here. A `.min()` clamp would silently TRUNCATE the nested
        // payload into a structurally-valid-but-corrupt frame on a future
        // bookkeeping desync; erroring routes to the loud discard instead.
        writeln!(out, "            let end = staged.cursor as usize;").unwrap();
        writeln!(out, "            if end > staged.len_bytes() {{").unwrap();
        writeln!(
            out,
            "                return ::std::result::Result::Err(::cerulion_core::TransportError::Internal {{"
        )
        .unwrap();
        writeln!(
            out,
            "                    reason: ::std::format!(\"staged cursor {{}} exceeds scratch {{}} bytes for nested field '{raw}' — spill bookkeeping desync\", end, staged.len_bytes()),"
        )
        .unwrap();
        writeln!(out, "                }});").unwrap();
        writeln!(out, "            }}").unwrap();
        writeln!(
            out,
            "            self.set_{raw}_bytes(&::cerulion_core::shm_runtime::u64s_as_bytes(&staged.scratch)[..end])?;"
        )
        .unwrap();
        writeln!(out, "        }}").unwrap();
    }
    writeln!(out, "        ::std::result::Result::Ok(())").unwrap();
    writeln!(out, "    }}").unwrap();
    writeln!(out).unwrap();
}

/// Emit the fixed-nested accessors for one DIRECT `Nested { fixed: Some }`
/// field: `__cer_nested_<f>() -> Result<&mut <Target>Shm, _>` (the plain
/// projection — the target's own `__cer_assign_<leaf>` shims compose
/// recursively), the doc-hidden `__cer_with_nested_<f>` closure primitive
/// (uniform macro-rewriter target across fixed AND complex nested), and
/// the PUBLIC `with_<f>`.
fn emit_fixed_nested_accessors(out: &mut String, field: &FieldDef) {
    let raw = &field.name;
    let escaped = escape_keyword(raw);
    let FieldType::Nested {
        schema_name,
        fixed: Some(_),
        ..
    } = &field.field_type
    else {
        return;
    };
    let target = format!("{schema_name}Shm");

    writeln!(
        out,
        "    /// Nested projection for fixed substruct `{raw}`."
    )
    .unwrap();
    writeln!(out, "    #[doc(hidden)]").unwrap();
    writeln!(out, "    #[inline]").unwrap();
    writeln!(
        out,
        "    pub fn __cer_nested_{raw}(&mut self) -> ::std::result::Result<&mut {target}, ::cerulion_core::TransportError> {{"
    )
    .unwrap();
    writeln!(
        out,
        "        ::std::result::Result::Ok(&mut self.{escaped})"
    )
    .unwrap();
    writeln!(out, "    }}").unwrap();
    writeln!(out).unwrap();

    writeln!(
        out,
        "    /// Primitive behind `with_{raw}` (schema-blind macro plumbing)."
    )
    .unwrap();
    writeln!(out, "    #[doc(hidden)]").unwrap();
    writeln!(out, "    #[inline]").unwrap();
    writeln!(
        out,
        "    pub fn __cer_with_nested_{raw}<__CerF>(&mut self, f: __CerF) -> ::std::result::Result<(), ::cerulion_core::TransportError>"
    )
    .unwrap();
    writeln!(out, "    where").unwrap();
    writeln!(
        out,
        "        __CerF: ::std::ops::FnOnce(&mut {target}) -> ::std::result::Result<(), ::cerulion_core::TransportError>,"
    )
    .unwrap();
    writeln!(out, "    {{").unwrap();
    writeln!(out, "        f(&mut self.{escaped})").unwrap();
    writeln!(out, "    }}").unwrap();
    writeln!(out).unwrap();

    writeln!(
        out,
        "    /// Write into the inline fixed sub-message `{raw}` through a scoped"
    )
    .unwrap();
    writeln!(out, "    /// writer.").unwrap();
    writeln!(out, "    ///").unwrap();
    writeln!(
        out,
        "    /// The closure receives `&mut {target}`, the same `#[repr(C)]`"
    )
    .unwrap();
    writeln!(
        out,
        "    /// overlay living inline in this message's fixed section, so writes"
    )
    .unwrap();
    writeln!(
        out,
        "    /// land directly in shared memory (no staging, no flush needed). In a"
    )
    .unwrap();
    writeln!(
        out,
        "    /// node's `tick` the receiver is the output port: `self.<port>.with_{raw}(..)`,"
    )
    .unwrap();
    writeln!(
        out,
        "    /// or write one leaf directly with `self.<port>.{raw}.<leaf> = value;`."
    )
    .unwrap();
    writeln!(
        out,
        "    pub fn with_{raw}<__CerF>(&mut self, f: __CerF) -> ::std::result::Result<(), ::cerulion_core::TransportError>"
    )
    .unwrap();
    writeln!(out, "    where").unwrap();
    writeln!(
        out,
        "        __CerF: ::std::ops::FnOnce(&mut {target}) -> ::std::result::Result<(), ::cerulion_core::TransportError>,"
    )
    .unwrap();
    writeln!(out, "    {{").unwrap();
    writeln!(out, "        f(&mut self.{escaped})").unwrap();
    writeln!(out, "    }}").unwrap();
    writeln!(out).unwrap();
}
#[cfg(test)]
mod big_array_attr_tests {
    use super::*;

    fn nested(name: &str, fixed: bool) -> FieldType {
        FieldType::Nested {
            schema_name: name.to_string(),
            package: None,
            fixed: fixed.then_some(crate::codegen::NestedFixedInfo {
                has_large_array: false,
                fixed_size: 24,
                alignment: 8,
                target_hash: 0,
            }),
        }
    }

    fn array(element: FieldType, length: usize) -> FieldType {
        FieldType::FixedArray {
            element_type: Box::new(element),
            length,
        }
    }

    /// The AGREEMENT the generated code's compilability rests on: the helper is
    /// attached to a field iff that field's emitted snapshot type is `[T; N]`
    /// with `N` past serde's ceiling.
    ///
    /// Driving BOTH functions over one corpus is what makes this a coupling
    /// test rather than a restatement — a future edit to either alone fails it.
    /// The corpus deliberately includes the shape the leading conjunct exists
    /// for (`Header[36]` → `Vec<u8>`) and its array-backed twin (`Vec3[36]`).
    #[test]
    fn field_needs_big_array_serde_agrees_with_the_emitted_snapshot_type() {
        let corpus = vec![
            FieldType::F64,
            FieldType::String,
            FieldType::Bytes,
            array(FieldType::F64, 32),
            array(FieldType::F64, 33),
            array(FieldType::F64, 36),
            array(FieldType::U8, 4096),
            array(FieldType::StringFixed(8), 36),
            // Nested elements, both resolutions, on both sides of the ceiling.
            array(nested("Vec3", true), 4),
            array(nested("Vec3", true), 36),
            array(nested("Header", false), 4),
            array(nested("Header", false), 36),
            nested("Vec3", true),
            nested("Header", false),
            FieldType::DynamicArray {
                element_type: Box::new(FieldType::F64),
            },
            FieldType::DynamicArray {
                element_type: Box::new(nested("Header", false)),
            },
        ];

        for ft in &corpus {
            let emitted = field_type_to_rust_snapshot(ft);
            // The emitted type is a big array iff it renders as `[…; N]` with
            // N past the ceiling — read off the STRING the generator produces,
            // so the oracle is the artifact and not a second copy of the rule.
            let emitted_is_big_array = emitted
                .strip_prefix('[')
                .and_then(|s| s.strip_suffix(']'))
                .and_then(|s| s.rsplit_once("; "))
                .and_then(|(_, n)| n.parse::<usize>().ok())
                .is_some_and(|n| n > crate::codegen::big_array::SERDE_ARRAY_IMPL_CEILING);

            assert_eq!(
                field_needs_big_array_serde(ft),
                emitted_is_big_array,
                "the helper attribute must track the emitted type; \
                 field {ft:?} emits `{emitted}`"
            );
        }
    }

    /// Anti-tautology: the corpus really does exercise BOTH verdicts, so the
    /// agreement above cannot be satisfied by a predicate that is always false.
    #[test]
    fn the_agreement_corpus_covers_both_verdicts() {
        assert!(field_needs_big_array_serde(&array(FieldType::F64, 36)));
        assert!(field_needs_big_array_serde(&array(
            nested("Vec3", true),
            36
        )));
        assert!(!field_needs_big_array_serde(&array(
            nested("Header", false),
            36
        )));
        assert!(!field_needs_big_array_serde(&array(FieldType::F64, 32)));
    }
}
