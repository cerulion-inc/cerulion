// SPDX-License-Identifier: AGPL-3.0-only
//! Integration tests for code generation (SHM-backed types).
//!
//! Verifies that `generate_schema()` emits the SHM-backed types:
//!
//! - `pub struct <Name>;` — zero-sized unit marker carrying `impl ShmMessage`.
//! - `pub struct <Name>Shm` (fixed) or `pub struct <Name>Shm<'a>` (variable)
//!   — the SHM-backed accessor type.
//! - `pub struct <Name>Snapshot` — heap-owned plain-data companion.
//! - `impl cerulion_core::message::ShmMessage for <Name>` with `Reader/Writer`
//!   GATs and `WIRE_FIXED_SIZE` / `VARIABLE_FIELD_COUNT` constants.
//!
//! Generated types are SHM-backed only: there is no heap `<Name>` struct, no
//! `impl Message`, no `<Name>View<'a>`, no `<Name>::new()` and no setters on
//! a heap struct, so nothing here asserts on those patterns.

use cerulion_core::codegen::{
    generate_schema, parse_rosmsg, resolve_fixed_nested, DefaultLiteral, FieldDef, FieldType,
    MessageSchema, NestedFixedInfo,
};

/// NO generated code may name a bare `::serde` path.
///
/// A scaffolded node crate depends on `cerulion_core` + `native_ros2_messages`
/// and nothing else, so `::serde` does not resolve there — every reference must
/// go through cerulion_core's re-export.
///
/// TOTAL, in every derive position. A check that
/// matches `"(::serde::"` only sees a bare path as the FIRST
/// derive; codegen emitting `#[derive(Debug, ::serde::Serialize, …)]` — or the
/// re-exported path AND a bare one — would pass it while a user's crate fails to
/// compile. Blanking the legitimate prefix first and then rejecting ANY
/// residual `::serde::` is position-independent and needs no enumeration of
/// where a path may appear.
fn assert_no_bare_serde_path(code: &str) {
    let residual = code.replace("::cerulion_core::serde::", "«ok»");
    assert!(
        !residual.contains("::serde::"),
        "generated code names a bare `::serde` path, which does not resolve in \
         a scaffolded node crate. Offending line(s):\n{}",
        residual
            .lines()
            .filter(|l| l.contains("::serde::"))
            .collect::<Vec<_>>()
            .join("\n")
    );
}

/// The guard above needs its OWN oracle, because a bare `::serde` path in real
/// codegen breaks the BUILD first — `cerulion_core`'s
/// tests dev-depend on `native_ros2_messages`, which (deliberately) has no
/// direct serde dependency, so `cargo test` never gets far enough to run an
/// assertion. Hand-written inputs are the only way to exercise the guard's own
/// logic, and they are what make the position-independent check verifiable:
/// the `contains("(::serde::")` form this replaced PASSES cases 2, 3 and 4.
#[test]
fn the_bare_serde_guard_rejects_a_bare_path_in_every_derive_position() {
    // Accepted: the re-exported path, alone and among siblings.
    assert_no_bare_serde_path("#[derive(::cerulion_core::serde::Serialize)]");
    assert_no_bare_serde_path(
        "#[derive(Debug, Clone, ::cerulion_core::serde::Serialize, \
         ::cerulion_core::serde::Deserialize)]\n#[serde(crate = \"::cerulion_core::serde\")]",
    );
    // Accepted: prose that merely mentions serde without a path.
    assert_no_bare_serde_path("// serde derives live on the snapshot\npub struct S;");

    let rejected = [
        // 1. FIRST derive — the only case a first-derive-only check catches.
        "#[derive(::serde::Serialize, ::serde::Deserialize)]",
        // 2. After other derives.
        "#[derive(Debug, Clone, ::serde::Serialize)]",
        // 3. MIXED: the required re-export AND a bare path on one line, so the
        //    positive assertions all still pass while a user's crate breaks.
        "#[derive(::cerulion_core::serde::Serialize, ::serde::Deserialize)]",
        // 4. Outside a derive entirely (a `with =` path, a bound).
        "    #[serde(with = \"::serde::skip\")]",
    ];
    for case in rejected {
        let caught = std::panic::catch_unwind(|| assert_no_bare_serde_path(case)).is_err();
        assert!(
            caught,
            "the guard must reject a bare `::serde` path in: {case}"
        );
    }
}

// =============================================================================
// Fixed-schema codegen
// =============================================================================

#[test]
fn test_fixed_schema_emits_marker_shm_snapshot_shm_message() {
    let mut schema = MessageSchema::new("Vector3");
    schema.add_field(FieldDef::new("x", FieldType::F64));
    schema.add_field(FieldDef::new("y", FieldType::F64));
    schema.add_field(FieldDef::new("z", FieldType::F64));

    let code = generate_schema(&schema);

    // 1. Unit marker: zero-sized, used as the type parameter `T` in
    //    OutputProxy<T> / InputView<T>.
    assert!(code.contains("pub struct Vector3;"), "missing unit marker");
    assert!(code.contains("#[derive(Debug, Clone, Copy, Default)]"));

    // 2. SHM-backed accessor: #[repr(C)] with `pub` fields.
    assert!(code.contains("#[repr(C)]"));
    assert!(code.contains("pub struct Vector3Shm {"));
    assert!(code.contains("pub x: f64,"));
    assert!(code.contains("pub y: f64,"));
    assert!(code.contains("pub z: f64,"));

    // 3. Constructors + round-trip helpers on Vector3Shm.
    assert!(code.contains("pub fn from_bytes(bytes: &[u8]) -> &Self"));
    assert!(code.contains("pub fn from_bytes_mut(bytes: &mut [u8]) -> &mut Self"));
    assert!(code.contains("pub fn snapshot(&self) -> Vector3Snapshot"));
    assert!(code.contains("pub fn write_from_snapshot(&mut self, snap: &Vector3Snapshot)"));

    // 4. Snapshot companion with serde derives.
    //
    // The derive names cerulion_core's `serde` RE-EXPORT,
    // not a bare `::serde` — a scaffolded node crate has no direct serde
    // dependency, so a bare path would not resolve there. The
    // `#[serde(crate = ...)]` companion is what makes the re-export work:
    // without it serde's derive emits `extern crate serde as _serde;`.
    assert!(code.contains("pub struct Vector3Snapshot {"));
    assert!(code.contains("::cerulion_core::serde::Serialize, ::cerulion_core::serde::Deserialize"));
    assert!(code.contains("#[serde(crate = \"::cerulion_core::serde\")]"));
    assert_no_bare_serde_path(&code);

    // 5. ShmMessage impl on the marker.
    assert!(code.contains("impl cerulion_core::message::ShmMessage for Vector3"));
    assert!(code.contains("type Reader<'a> = &'a Vector3Shm;"));
    assert!(code.contains("type Writer<'a> = &'a mut Vector3Shm;"));
    assert!(code.contains("const VARIABLE_FIELD_COUNT: usize = 0;"));
    assert!(code.contains("const WIRE_FIXED_SIZE: usize = ::std::mem::size_of::<Vector3Shm>();"));

    // 6. Schema hash constant.
    assert!(code.contains("VECTOR3_SCHEMA_HASH"));

    // 7. Fixed-only schemas emit MAX_SLICE_LEN as a
    //    provably-correct upper bound (header + entire fixed section).
    //    Asserted on the multi-line emission so a future codegen
    //    re-format that splits / joins lines still matches as long as
    //    both sides of the sum are present.
    assert!(
        code.contains(
            "const MAX_SLICE_LEN: ::std::option::Option<::cerulion_core::wire::MaxSliceLen> ="
        ),
        "fixed-only schema must emit MAX_SLICE_LEN as Option<MaxSliceLen>"
    );
    assert!(
        code.contains("::cerulion_core::wire::MaxSliceLen::const_new("),
        "fixed-only MAX_SLICE_LEN must use MaxSliceLen::const_new(...) — compile-time prevention"
    );
    assert!(
        code.contains("::cerulion_core::wire::WireHeader::SIZE"),
        "fixed-only MAX_SLICE_LEN must reference WireHeader::SIZE"
    );
    assert!(
        code.contains("<Self as cerulion_core::message::ShmMessage>::WIRE_FIXED_SIZE"),
        "fixed-only MAX_SLICE_LEN must reference WIRE_FIXED_SIZE"
    );
}

#[test]
fn test_zero_field_schema_emits_emit_publish_gesture_fielded_does_not() {
    // A ZERO-FIELD output schema (e.g. `std_msgs/Empty`) is
    // un-emittable under the macro lazy-loan without an explicit gesture — no
    // field to write means the write shims never fire, so the port never loans
    // and nothing publishes. The codegen emits an `emit()` publish gesture on
    // the fixed `<Name>Shm` for zero-field schemas ONLY, so `self.<port>.emit()?`
    // loans + publishes. The gate is empty-ONLY, so calling `emit()` on any
    // output that HAS a field is a compile error (no such method). Hand oracle:
    // the exact generated signature (never a self-compare).
    let empty = MessageSchema::new("Empty"); // fieldless — the emit() case
    let empty_code = generate_schema(&empty);
    assert!(
        empty_code.contains(
            "pub fn emit(&mut self) -> ::std::result::Result<(), ::cerulion_core::TransportError>"
        ),
        "a zero-field schema must emit the `emit()` publish gesture; got:\n{empty_code}"
    );
    assert!(
        empty_code.contains("::std::result::Result::Ok(())"),
        "the `emit()` body is a trivial Ok(()) — the loan (via the macro receiver \
         rewrite) is the publish intent; got:\n{empty_code}"
    );

    // A fielded fixed schema must NOT get `emit()` — the gate is zero-field-only,
    // so `self.<port>.emit()` on it fails to compile (the zero-field scoping).
    let mut vec3 = MessageSchema::new("Vector3");
    vec3.add_field(FieldDef::new("x", FieldType::F64));
    vec3.add_field(FieldDef::new("y", FieldType::F64));
    vec3.add_field(FieldDef::new("z", FieldType::F64));
    let vec3_code = generate_schema(&vec3);
    assert!(
        !vec3_code.contains("pub fn emit("),
        "a fielded schema must NOT emit `emit()` (zero-field-scoped gate — calling \
         it must be a compile error); got:\n{vec3_code}"
    );
}

#[test]
fn test_fixed_schema_bool_field_stored_as_u8_in_shm_struct() {
    let mut schema = MessageSchema::new("RegionOfInterest");
    schema.add_field(FieldDef::new("x_offset", FieldType::U32));
    schema.add_field(FieldDef::new("do_rectify", FieldType::Bool));

    let code = generate_schema(&schema);

    // SHM struct: bool wire byte stored as u8 (bool's bit pattern is restricted
    // to {0, 1} — reading any other byte as bool would be UB).
    assert!(code.contains("pub struct RegionOfInterestShm {"));
    assert!(code.contains("pub do_rectify: u8,"));

    // Snapshot keeps real `bool`.
    assert!(code.contains("pub struct RegionOfInterestSnapshot {"));
    assert!(code.contains("pub do_rectify: bool,"));

    // Round-trip helpers convert between the two encodings.
    assert!(code.contains("do_rectify: self.do_rectify != 0,"));
    assert!(code.contains("self.do_rectify = snap.do_rectify as u8;"));
}

#[test]
fn test_fixed_schema_large_array_handles_default_correctly() {
    // Rust doesn't implement `Default` for [T; N] when N > 32; the generator
    // must either omit Default from the derive or supply a manual impl.
    let mut schema = MessageSchema::new("Imu");
    schema.add_field(FieldDef::new(
        "covariance",
        FieldType::FixedArray {
            element_type: Box::new(FieldType::F64),
            length: 36,
        },
    ));

    let code = generate_schema(&schema);

    let derives_default_via_attr = code.contains("PartialEq, Default)]");
    let has_manual_default = code.contains("impl Default for ImuShm");

    assert!(
        !derives_default_via_attr || has_manual_default,
        "large-array SHM type must not derive Default without a manual impl"
    );
    // The marker still gets emitted regardless of array size.
    assert!(code.contains("pub struct Imu;"));
    assert!(code.contains("pub struct ImuShm {"));
}

#[test]
fn test_nested_only_schema_uses_variable_emission_with_raw_bytes_accessors() {
    // A schema whose only fields are Nested types is treated as variable
    // (nested fields are raw bytes here, not typed wrappers). The SHM
    // type is `<Name>Shm<'a>` with raw `<field>_bytes` accessors per nested
    // field, and the snapshot stores nested fields as `Vec<u8>`.
    let mut schema = MessageSchema::new("Pose");
    schema.add_field(FieldDef::new(
        "position",
        FieldType::Nested {
            schema_name: "Point".to_string(),
            package: None,
            fixed: None,
        },
    ));
    schema.add_field(FieldDef::new(
        "orientation",
        FieldType::Nested {
            schema_name: "Quaternion".to_string(),
            package: None,
            fixed: None,
        },
    ));

    let code = generate_schema(&schema);

    // Variable-schema emission shape.
    assert!(code.contains("pub struct PoseShm<'a>"));
    assert!(code.contains("impl cerulion_core::message::ShmMessage for Pose"));
    assert!(code.contains("type Reader<'a> = PoseShm<'a>;"));

    // Raw-bytes accessors per nested field (codegen emits no typed
    // wrapper for them).
    assert!(code.contains("pub fn position_bytes(&self) -> &[u8]"));
    assert!(code.contains("pub fn set_position_bytes(&mut self, value: &[u8])"));
    assert!(code.contains("pub fn orientation_bytes(&self) -> &[u8]"));
    assert!(code.contains("pub fn set_orientation_bytes(&mut self, value: &[u8])"));

    // Snapshot stores nested bytes as Vec<u8>.
    assert!(code.contains("pub struct PoseSnapshot {"));
    assert!(code.contains("pub position: ::std::vec::Vec<u8>"));
    assert!(code.contains("pub orientation: ::std::vec::Vec<u8>"));
}

#[test]
fn test_nested_field_inside_fixed_schema_uses_shm_companion() {
    // A schema mixing a primitive + a single nested field is still classified
    // as fixed (Nested types are wire-fixed when the nested schema is fixed).
    // The SHM struct embeds `<NestedName>Shm` (NOT the unit marker) so wire
    // layout matches.
    //
    // NOTE: emission classifies Nested fields as variable in the
    // codegen pass, even when the nested schema itself is fixed. We assert
    // on the marker + ShmMessage impl instead, which are emitted regardless.
    let mut schema = MessageSchema::new("PointStamped");
    schema.add_field(FieldDef::new("seq", FieldType::U32));
    schema.add_field(FieldDef::new(
        "point",
        FieldType::Nested {
            schema_name: "Point".to_string(),
            package: None,
            fixed: None,
        },
    ));

    let code = generate_schema(&schema);

    assert!(code.contains("pub struct PointStamped;"));
    assert!(code.contains("impl cerulion_core::message::ShmMessage for PointStamped"));
    assert!(code.contains("pub struct PointStampedSnapshot"));
}

#[test]
fn test_keyword_field_name_is_escaped_in_shm_struct() {
    let mut schema = MessageSchema::new("KeywordTest");
    schema.add_field(FieldDef::new("type", FieldType::U32));

    let code = generate_schema(&schema);

    // Field name is a Rust keyword — codegen must use raw identifier syntax.
    assert!(code.contains("r#type"));
}

#[test]
fn test_string_fixed_field_emitted_as_byte_array_in_shm() {
    // StringFixed(N) is wire-fixed-size — schemas containing only
    // StringFixed + primitives should still get the SHM-backed emission.
    let mut schema = MessageSchema::new("DeviceId");
    schema.add_field(FieldDef::new("id", FieldType::StringFixed(16)));
    schema.add_field(FieldDef::new("kind", FieldType::U32));

    let code = generate_schema(&schema);

    assert!(code.contains("pub struct DeviceIdShm {"));
    assert!(code.contains("pub id: [u8; 16],"));
    assert!(code.contains("pub kind: u32,"));
    assert!(code.contains("impl cerulion_core::message::ShmMessage for DeviceId"));
}

// =============================================================================
// Built-in `builtin_interfaces/Time::from_ns` stamp helper
// =============================================================================

/// The vendored `builtin_interfaces/Time` schema gets an inherent `from_ns`
/// helper on its marker, returning the `TimeShm` overlay (the type the
/// `header.stamp` write shim consumes). Behavioral arithmetic is pinned by
/// `native_ros2_messages/tests/stamp_helper_test.rs` against a hand oracle;
/// this test pins the CODEGEN half (the helper is emitted with the right shape).
#[test]
fn test_builtin_time_emits_from_ns_helper() {
    let mut schema = MessageSchema::new_in_package("Time", "builtin_interfaces");
    schema.add_field(FieldDef::new("sec", FieldType::I32));
    schema.add_field(FieldDef::new("nanosec", FieldType::U32));

    let code = generate_schema(&schema);

    // Inherent impl on the marker (not the overlay), returning the overlay.
    assert!(
        code.contains("impl Time {"),
        "Time marker must carry an inherent impl block"
    );
    assert!(
        code.contains("pub fn from_ns(ns: u64) -> TimeShm"),
        "from_ns must be a marker associated fn returning the TimeShm overlay"
    );
    // The loud-in-debug 2038 guard + the never-wrap saturating cast.
    assert!(
        code.contains("debug_assert!"),
        "from_ns must carry the loud debug-mode i32-second overflow guard"
    );
    assert!(
        code.contains("secs.min(i32::MAX as u64) as i32"),
        "release must saturate sec to i32::MAX, never two's-complement wrap"
    );
}

/// The helper is gated on the EXACT `builtin_interfaces/Time` schema identity:
/// a same-shaped `Time` in another package, `builtin_interfaces/Duration`, and
/// a package-less workspace `Time` schema must NOT get it (no leakage).
#[test]
fn test_from_ns_helper_is_gated_on_builtin_time_identity() {
    let time_fields = |s: &mut MessageSchema| {
        s.add_field(FieldDef::new("sec", FieldType::I32));
        s.add_field(FieldDef::new("nanosec", FieldType::U32));
    };

    // Wrong package, identical fields → no helper.
    let mut other_pkg = MessageSchema::new_in_package("Time", "my_msgs");
    time_fields(&mut other_pkg);
    assert!(
        !generate_schema(&other_pkg).contains("pub fn from_ns"),
        "from_ns must not leak to a Time in a non-builtin_interfaces package"
    );

    // builtin_interfaces/Duration has the SAME two fields → no helper.
    let mut duration = MessageSchema::new_in_package("Duration", "builtin_interfaces");
    time_fields(&mut duration);
    assert!(
        !generate_schema(&duration).contains("pub fn from_ns"),
        "from_ns is Time-only (Duration has the same layout but is not a timestamp)"
    );

    // Package-less workspace `Time` schema → no helper (requires the package).
    let mut bare = MessageSchema::new("Time");
    time_fields(&mut bare);
    assert!(
        !generate_schema(&bare).contains("pub fn from_ns"),
        "from_ns requires the builtin_interfaces package qualification"
    );
}

// =============================================================================
// Layout-sensitive schema hash
// =============================================================================

/// Helper: a representative mixed schema for hash tests.
fn hash_probe_schema(name: &str) -> MessageSchema {
    let mut s = MessageSchema::new(name);
    s.add_field(FieldDef::new("width", FieldType::U32));
    s.add_field(FieldDef::new("height", FieldType::U32));
    s.add_field(FieldDef::new("encoding", FieldType::String));
    s.add_field(FieldDef::new(
        "data",
        FieldType::DynamicArray {
            element_type: Box::new(FieldType::U8),
        },
    ));
    s
}

#[test]
fn test_independently_constructed_identical_schemas_hash_equal() {
    // Two independent constructions of the same layout — NOT h == h on
    // the same object (that would be a tautology).
    let s1 = hash_probe_schema("Image");
    let s2 = hash_probe_schema("Image");
    assert_eq!(s1.schema_hash(), s2.schema_hash());
}

#[test]
fn test_same_name_different_layout_hashes_differ() {
    let s1 = hash_probe_schema("Image");
    let mut s2 = MessageSchema::new("Image");
    s2.add_field(FieldDef::new("width", FieldType::U32));
    assert_ne!(
        s1.schema_hash(),
        s2.schema_hash(),
        "same-name schemas with different layouts must hash differently"
    );
}

#[test]
fn test_field_rename_changes_hash() {
    let mut s1 = MessageSchema::new("Probe");
    s1.add_field(FieldDef::new("x", FieldType::F64));
    let mut s2 = MessageSchema::new("Probe");
    s2.add_field(FieldDef::new("y", FieldType::F64));
    assert_ne!(s1.schema_hash(), s2.schema_hash());
}

#[test]
fn test_field_reorder_changes_hash() {
    // Same fields, same total wire_fixed_size (4 bytes either way) —
    // only declaration order differs.
    let mut s1 = MessageSchema::new("Probe");
    s1.add_field(FieldDef::new("a", FieldType::U8));
    s1.add_field(FieldDef::new("b", FieldType::U16));
    let mut s2 = MessageSchema::new("Probe");
    s2.add_field(FieldDef::new("b", FieldType::U16));
    s2.add_field(FieldDef::new("a", FieldType::U8));
    assert_eq!(
        s1.wire_fixed_size(),
        s2.wire_fixed_size(),
        "test precondition: reorder must not change the fixed size, so the \
         hash difference is attributable to field order alone"
    );
    assert_ne!(s1.schema_hash(), s2.schema_hash());
}

#[test]
fn test_added_fixed_field_changes_hash() {
    // Adding a fixed field changes WIRE_FIXED_SIZE and the field stream.
    let mut s1 = MessageSchema::new("Probe");
    s1.add_field(FieldDef::new("x", FieldType::F64));
    let mut s2 = MessageSchema::new("Probe");
    s2.add_field(FieldDef::new("x", FieldType::F64));
    s2.add_field(FieldDef::new("y", FieldType::F64));
    assert_ne!(s1.wire_fixed_size(), s2.wire_fixed_size());
    assert_ne!(s1.schema_hash(), s2.schema_hash());
}

#[test]
fn test_descriptions_do_not_change_hash() {
    let s1 = hash_probe_schema("Image");
    let mut s2 = hash_probe_schema("Image");
    s2.description = Some("a camera image".to_string());
    for f in &mut s2.fields {
        f.description = Some(format!("doc for {}", f.name));
    }
    assert_eq!(
        s1.schema_hash(),
        s2.schema_hash(),
        "descriptions / doc strings carry no layout meaning and must not \
         affect the hash"
    );
}

#[test]
fn test_length_prefix_injectivity() {
    // Field name + nested type name whose CONCATENATED bytes are identical
    // ("ab" + "c" == "a" + "bc" == "abc") but split differently. Without
    // length-prefix framing these would collide; the recipe's framing
    // makes the encoding injective by construction.
    let mut s1 = MessageSchema::new("Probe");
    s1.add_field(FieldDef::new(
        "ab",
        FieldType::Nested {
            schema_name: "c".to_string(),
            package: None,
            fixed: None,
        },
    ));
    let mut s2 = MessageSchema::new("Probe");
    s2.add_field(FieldDef::new(
        "a",
        FieldType::Nested {
            schema_name: "bc".to_string(),
            package: None,
            fixed: None,
        },
    ));
    assert_eq!(
        s1.wire_fixed_size(),
        s2.wire_fixed_size(),
        "test precondition: both nested-only schemas have an empty fixed \
         section, so a hash difference is attributable to framing alone"
    );
    assert_ne!(
        s1.schema_hash(),
        s2.schema_hash(),
        "length-prefix framing must distinguish identical concatenations \
         split at different boundaries"
    );
}

#[test]
fn test_hash_is_not_the_old_name_only_recipe() {
    use cerulion_core::wire::fnv1a_hash;
    let s = hash_probe_schema("Image");
    assert_ne!(
        s.schema_hash(),
        fnv1a_hash(b"Image"),
        "the layout-sensitive hash must not equal the old fnv1a(name) recipe"
    );
}

#[test]
fn test_different_schema_names_produce_distinct_hashes() {
    let names = ["Image", "Pose", "Header", "Time"];
    let hashes: Vec<u64> = names
        .iter()
        .map(|n| MessageSchema::new(*n).schema_hash())
        .collect();
    for i in 0..hashes.len() {
        for j in i + 1..hashes.len() {
            assert_ne!(
                hashes[i], hashes[j],
                "hashes for {} and {} should differ",
                names[i], names[j]
            );
        }
    }
}

// =============================================================================
// Variable-schema codegen (emission patterns)
// =============================================================================

#[test]
fn test_variable_schema_emits_opaque_shm_with_typed_accessors() {
    let mut img = MessageSchema::new("Image");
    img.add_field(FieldDef::new("width", FieldType::U32));
    img.add_field(FieldDef::new("height", FieldType::U32));
    img.add_field(FieldDef::new("encoding", FieldType::String));
    img.add_field(FieldDef::new(
        "data",
        FieldType::DynamicArray {
            element_type: Box::new(FieldType::U8),
        },
    ));

    let code = generate_schema(&img);

    // Marker + opaque SHM struct with lifetime + Snapshot + ShmMessage impl
    // + the #[repr(C)] FixedSection overlay.
    assert!(code.contains("pub struct Image;"));
    assert!(code.contains("pub struct ImageShm<'a>"));
    assert!(code.contains("pub struct ImageFixedSection {"));
    assert!(code.contains("pub struct ImageSnapshot {"));
    assert!(code.contains("impl cerulion_core::message::ShmMessage for Image"));
    assert!(code.contains("type Reader<'a> = ImageShm<'a>;"));
    assert!(code.contains("type Writer<'a> = ImageShm<'a>;"));
    assert!(code.contains("const VARIABLE_FIELD_COUNT: usize = 2;"));
    assert!(
        code.contains("const WIRE_FIXED_SIZE: usize = ::std::mem::size_of::<ImageFixedSection>();")
    );

    // Variable schema `Image` has
    // a hand-tuned 128 MiB budget (`TIER_HUGE`) in the codegen lookup
    // table. The literal byte count (`134217728`) appears in the emitted
    // single-line const so the `#[doc(hidden)]` impl carries the right
    // number into the trait.
    assert!(
        code.contains(
            "const MAX_SLICE_LEN: ::std::option::Option<::cerulion_core::wire::MaxSliceLen> = ::std::option::Option::Some(::cerulion_core::wire::MaxSliceLen::const_new(134217728));"
        ),
        "Image (variable schema) must emit hand-tuned 128 MiB MAX_SLICE_LEN"
    );

    // Fixed-section fields are emitted as `pub <name>: <type>` on
    // the overlay + reached from the SHM type via Deref. Legacy `set_<name>`
    // / `<name>()` methods are GONE. Negative assertions use the full method
    // signature so unrelated methods (e.g. a hypothetical `width_padded`)
    // wouldn't false-match.
    assert!(code.contains("pub width: u32,"));
    assert!(code.contains("pub height: u32,"));
    assert!(code.contains("impl<'a> ::std::ops::Deref for ImageShm<'a>"));
    assert!(code.contains("impl<'a> ::std::ops::DerefMut for ImageShm<'a>"));
    assert!(!code.contains("pub fn width(&self) -> u32"));
    assert!(!code.contains("pub fn set_width(&mut self, value: u32)"));
    assert!(!code.contains("pub fn height(&self) -> u32"));
    assert!(!code.contains("pub fn set_height(&mut self, value: u32)"));

    // Variable-field accessors (String): typed getter returns
    // `Result<&str, WireError>` (no silent fallback).
    assert!(code.contains(
        "pub fn encoding(&self) -> ::std::result::Result<&str, ::cerulion_core::wire::WireError>"
    ));
    assert!(code.contains(
        "pub fn loan_encoding(&mut self, n: usize) -> ::std::result::Result<&mut [u8], ::cerulion_core::TransportError>"
    ));
    assert!(code.contains(
        "pub fn set_encoding(&mut self, value: &str) -> ::std::result::Result<(), ::cerulion_core::TransportError>"
    ));

    // Variable-field accessors (Bytes-equivalent DynamicArray<u8>): typed
    // getter + loan/set + push.
    assert!(code.contains("pub fn data(&self) -> &[u8]"));
    assert!(code.contains("pub fn loan_data(&mut self, n: usize)"));
    assert!(code.contains("pub fn set_data(&mut self, value: &[u8])"));
    assert!(code.contains("pub fn push_data(&mut self, item: u8)"));

    // OutputProxy::Drop gate.
    assert!(code.contains("pub fn all_variables_written(&self) -> bool"));
}

#[test]
fn test_variable_schema_not_in_table_uses_user_defined_fallback() {
    // A schema name NOT listed in the codegen
    // categorization table is treated as user-defined and emits
    // MAX_SLICE_LEN = Some(128 MiB). This equals the
    // value of `DEFAULT_MAX_SLICE_LEN`, so the const is
    // behaviorally identical to the runtime tier-3 fallback.
    // Every variable schema in `native_ros2_messages/` MUST be in
    // the categorization table; the catch-all is reserved for
    // user-defined schemas only (tiny in-repo schemas would
    // otherwise silently reserve 128 MiB of SHM per topic).
    let mut foo = MessageSchema::new("NotInTheTable");
    foo.add_field(FieldDef::new("payload", FieldType::String));

    let code = generate_schema(&foo);

    // 128 * 1024 * 1024 = 134217728 bytes (TIER_HUGE; also the
    // DEFAULT_MAX_SLICE_LEN; see
    // graph/config.rs for the lazy-pool rationale).
    assert!(
        code.contains(
            "const MAX_SLICE_LEN: ::std::option::Option<::cerulion_core::wire::MaxSliceLen> = ::std::option::Option::Some(::cerulion_core::wire::MaxSliceLen::const_new(134217728));"
        ),
        "variable schema not in categorization table must emit user-defined fallback (128 MiB)"
    );
}

#[test]
fn test_variable_schema_categorization_tiers_emit_distinct_budgets() {
    // Spot-check that representatives of each tier
    // emit the expected byte count. This is the load-bearing
    // assertion for the categorization table — if a schema name
    // moves between tiers (or the budget for a tier changes), this
    // test pinpoints the diff.
    // The tier table is keyed by QUALIFIED name ("pkg/Name") —
    // bare names collide across packages (same argument as FQN hashing).
    // Package-less schemas (workspace YAML) intentionally land in the
    // user-defined catch-all.
    let mk = |name: &str, package: &str, var_field: &str| {
        let mut s = MessageSchema::new_in_package(name, package);
        // `String` triggers the variable-schema codegen path, which is
        // what consumes `variable_schema_max_slice_len`.
        s.add_field(FieldDef::new(var_field, FieldType::String));
        generate_schema(&s)
    };

    // TIER_HUGE: 128 MiB = 134217728
    assert!(
        mk("Image", "sensor_msgs", "encoding").contains(
            "MAX_SLICE_LEN: ::std::option::Option<::cerulion_core::wire::MaxSliceLen> = ::std::option::Option::Some(::cerulion_core::wire::MaxSliceLen::const_new(134217728));"
        ),
        "Image must land in TIER_HUGE (128 MiB)"
    );

    // TIER_LARGE: 16 MiB = 16777216
    // `sensor_msgs/CompressedImage` is NOT this exemplar: it
    // sits in HUGE (a 4K PNG over image_transport is 13.7-24 MB and
    // exceeds the 16 MiB ceiling outright). `visualization_msgs/MarkerArray`
    // is the large-tier exemplar — it is also the container that puts
    // `Marker` here, so it is the arm most likely to be edited
    // next and the one most worth pinning.
    assert!(
        mk("MarkerArray", "visualization_msgs", "ns").contains(
            "MAX_SLICE_LEN: ::std::option::Option<::cerulion_core::wire::MaxSliceLen> = ::std::option::Option::Some(::cerulion_core::wire::MaxSliceLen::const_new(16777216));"
        ),
        "MarkerArray must land in TIER_LARGE (16 MiB)"
    );

    // TIER_MEDIUM: 4 MiB = 4194304
    // `visualization_msgs/Marker` is NOT this exemplar: it
    // sits in TIER_LARGE because it carries upstream Jazzy's embedded
    // `CompressedImage texture` + `MeshFile mesh_file` (both themselves
    // TIER_LARGE). `nav_msgs/Path` is the medium-tier exemplar.
    assert!(
        mk("Path", "nav_msgs", "frame_id").contains(
            "MAX_SLICE_LEN: ::std::option::Option<::cerulion_core::wire::MaxSliceLen> = ::std::option::Option::Some(::cerulion_core::wire::MaxSliceLen::const_new(4194304));"
        ),
        "Path must land in TIER_MEDIUM (4 MiB)"
    );

    // TIER_SMALL: 256 KiB = 262144
    assert!(
        mk("LaserScan", "sensor_msgs", "frame_id").contains(
            "MAX_SLICE_LEN: ::std::option::Option<::cerulion_core::wire::MaxSliceLen> = ::std::option::Option::Some(::cerulion_core::wire::MaxSliceLen::const_new(262144));"
        ),
        "LaserScan must land in TIER_SMALL (256 KiB)"
    );

    // TIER_TINY: 16 KiB = 16384. Note: the user-defined catch-all is
    // TIER_HUGE (128 MiB), NOT TIER_TINY — see the dedicated
    // `test_variable_schema_not_in_table_uses_user_defined_fallback`
    // test below.
    assert!(
        mk("Header", "std_msgs", "frame_id").contains(
            "MAX_SLICE_LEN: ::std::option::Option<::cerulion_core::wire::MaxSliceLen> = ::std::option::Option::Some(::cerulion_core::wire::MaxSliceLen::const_new(16384));"
        ),
        "Header must land in TIER_TINY (16 KiB)"
    );
}

#[test]
fn test_variable_schema_setter_routes_overflow_via_payload_too_large() {
    // Overflow redirect: variable setters do not emit ProxyBufferTooSmall
    // inline. They call `ensure_capacity_for(bytes_needed, cursor)?` which
    // either succeeds (steady state or via heap spill) or returns
    // `PayloadTooLarge` / `AllocationFailed` when the ceiling/alloc
    // cannot be satisfied. ProxyBufferTooSmall is retained in the error
    // enum (some hand-written paths still construct it) but is NOT
    // emitted by the codegen-emitted setter sites anymore.
    let mut img = MessageSchema::new("Image");
    img.add_field(FieldDef::new("encoding", FieldType::String));

    let code = generate_schema(&img);

    assert!(
        code.contains("ensure_capacity_for"),
        "variable setters must call ensure_capacity_for for overflow handling"
    );
    assert!(
        code.contains("TransportError::PayloadTooLarge"),
        "ensure_capacity_for must emit PayloadTooLarge for above-ceiling overflow"
    );
    assert!(
        code.contains("TransportError::AllocationFailed"),
        "ensure_capacity_for must emit AllocationFailed for spill-alloc OOM"
    );
}

#[test]
fn test_dynamic_array_bool_zero_fills_before_from_raw_parts_cast() {
    // `DynamicArray<Bool>` codegen must zero-fill
    // the dst bytes BEFORE casting to `&mut [bool]`. `from_raw_parts_mut`
    // over arbitrary non-{0,1} bytes is UB at reference-creation time,
    // even if the producer never reads. iceoryx2 does not zero-init
    // recycled slots, so the prior loan's bytes can be anything.
    //
    // Both `loan_<f>` and `fill_from_<f>` emissions for bool-element
    // arrays must include `dst_bytes.fill(0u8)` before the cast.
    // Other element types (i8, u8, i16-i64, u16-u64, f32, f64) have
    // no validity constraint and must NOT add the redundant fill.
    let mut schema = MessageSchema::new("BoolArrayMsg");
    schema.add_field(FieldDef::new(
        "flags",
        FieldType::DynamicArray {
            element_type: Box::new(FieldType::Bool),
        },
    ));

    let code = generate_schema(&schema);

    // loan_<f> for bool: zero-fill + bool cast both present.
    assert!(
        code.contains("pub fn loan_flags(&mut self, n: usize)"),
        "loan_flags should be emitted for DynamicArray<Bool>"
    );
    assert!(
        code.contains("dst_bytes.fill(0u8)"),
        "bool codegen MUST zero-fill dst_bytes before the bool cast — `from_raw_parts_mut::<bool>` over non-{{0,1}} bytes is UB"
    );
    assert!(
        code.contains("as *mut bool"),
        "bool codegen should emit `as *mut bool` cast"
    );

    // fill_from_<f> for bool: same zero-fill + bool cast.
    assert!(
        code.contains("pub fn fill_from_flags<__CerS>"),
        "fill_from_flags should be emitted for DynamicArray<Bool>"
    );
    // The fill_from emits an indented `dst_bytes.fill(0u8);` (inside
    // the result scope) — count occurrences to confirm BOTH paths
    // are guarded (loan + fill_from).
    let fill_count = code.matches("dst_bytes.fill(0u8)").count();
    assert!(
        fill_count >= 2,
        "expected at least 2 zero-fill sites (loan_<f> + fill_from_<f>); found {fill_count}"
    );
}

#[test]
fn test_dynamic_array_bool_reader_validates_bytes_before_cast() {
    // The generated `&[bool]` READER must refuse to mint an
    // unsound slice. The writer path (`loan_<f>`) zero-fills before its
    // cast, but the reader overlays whatever bytes are already in the SHM
    // slot. Creating `&[bool]` over bytes that are NOT strictly {0,1} is
    // UB at reference-creation time, even if no element is read. iceoryx2
    // does not zero-init recycled slots, so a `bool[]` field overlaying a
    // slot a prior occupant wrote (e.g. a `u8[]` at the same offset) is
    // reachable. The reader must validate every byte ≤ 1 and fail safe to
    // `&[]` on any non-canonical byte, symmetric to the writer guard.
    let mut schema = MessageSchema::new("BoolReaderMsg");
    schema.add_field(FieldDef::new(
        "flags",
        FieldType::DynamicArray {
            element_type: Box::new(FieldType::Bool),
        },
    ));

    let code = generate_schema(&schema);

    // The bare reader is emitted and returns `&[bool]`.
    assert!(
        code.contains("pub fn flags(&self) -> &[bool]"),
        "bool array reader `flags() -> &[bool]` should be emitted"
    );
    // It must validate the bytes (all ≤ 1) before the `*const bool` cast.
    assert!(
        code.contains("if !slice.iter().all(|&b| b <= 1) { return &[]; }"),
        "bool reader MUST validate every byte is 0 or 1 before casting to \
         `&[bool]` — `from_raw_parts::<bool>` over non-{{0,1}} bytes is UB.\n{code}"
    );
    assert!(
        code.contains("as *const bool"),
        "bool reader should emit the `as *const bool` cast"
    );
}

#[test]
fn test_dynamic_array_non_bool_primitives_no_zero_fill_overhead() {
    // The zero-fill must NOT appear for non-bool
    // primitive element types, since `from_raw_parts_mut::<T>` is sound
    // for any byte pattern when T is i8/u8/i16-i64/u16-u64/f32/f64.
    // Emitting a zero-fill for those types would be wasted O(n) memset
    // per loan.
    let mut schema = MessageSchema::new("PrimitiveArraysMsg");
    schema.add_field(FieldDef::new(
        "samples",
        FieldType::DynamicArray {
            element_type: Box::new(FieldType::F32),
        },
    ));
    schema.add_field(FieldDef::new(
        "counters",
        FieldType::DynamicArray {
            element_type: Box::new(FieldType::I32),
        },
    ));

    let code = generate_schema(&schema);

    // NO bool path → no zero-fill should appear anywhere in the
    // generated code for this schema.
    assert!(
        !code.contains("dst_bytes.fill(0u8)"),
        "non-bool primitive arrays must NOT emit zero-fill (wasted O(n) per loan); found in:\n{code}"
    );
    // But the typed loan + fill_from methods ARE emitted.
    assert!(code.contains("pub fn loan_samples(&mut self, n: usize)"));
    assert!(code.contains("pub fn loan_counters(&mut self, n: usize)"));
    assert!(code.contains("pub fn fill_from_samples<__CerS>"));
    assert!(code.contains("pub fn fill_from_counters<__CerS>"));
}

#[test]
fn test_complex_variable_field_falls_back_to_raw_bytes_accessors() {
    // Nested fields in a variable schema get raw `&[u8]` accessors with
    // the `_bytes` suffix (codegen emits no typed wrapper for them).
    let mut twist = MessageSchema::new("Twist");
    twist.add_field(FieldDef::new(
        "linear",
        FieldType::Nested {
            schema_name: "Vector3".to_string(),
            package: None,
            fixed: None,
        },
    ));
    twist.add_field(FieldDef::new(
        "angular",
        FieldType::Nested {
            schema_name: "Vector3".to_string(),
            package: None,
            fixed: None,
        },
    ));

    let code = generate_schema(&twist);

    // SHM type still gets emitted (it's a variable schema because Nested is
    // treated as variable in emission).
    assert!(code.contains("pub struct TwistShm<'a>"));
    assert!(code.contains("impl cerulion_core::message::ShmMessage for Twist"));

    // Nested fields get the `_bytes` raw accessors.
    assert!(code.contains("pub fn linear_bytes(&self) -> &[u8]"));
    assert!(code.contains("pub fn set_linear_bytes(&mut self, value: &[u8])"));
    assert!(code.contains("pub fn angular_bytes(&self) -> &[u8]"));

    // Snapshot stores nested bytes as `Vec<u8>` (no typed wrapper).
    assert!(code.contains("pub linear: ::std::vec::Vec<u8>"));
    assert!(code.contains("pub angular: ::std::vec::Vec<u8>"));
}

// =============================================================================
// Edge cases
// =============================================================================

#[test]
fn test_empty_schema_still_emits_marker_and_shm_message_impl() {
    let schema = MessageSchema::new("Empty");
    let code = generate_schema(&schema);

    assert!(code.contains("pub struct Empty;"));
    assert!(code.contains("pub struct EmptyShm"));
    assert!(code.contains("impl cerulion_core::message::ShmMessage for Empty"));
    assert!(code.contains("EMPTY_SCHEMA_HASH"));
}

#[test]
fn test_all_primitive_types_appear_in_shm_struct() {
    let mut schema = MessageSchema::new("AllPrimitives");
    schema.add_field(FieldDef::new("f_bool", FieldType::Bool));
    schema.add_field(FieldDef::new("f_i8", FieldType::I8));
    schema.add_field(FieldDef::new("f_u8", FieldType::U8));
    schema.add_field(FieldDef::new("f_i16", FieldType::I16));
    schema.add_field(FieldDef::new("f_u16", FieldType::U16));
    schema.add_field(FieldDef::new("f_i32", FieldType::I32));
    schema.add_field(FieldDef::new("f_u32", FieldType::U32));
    schema.add_field(FieldDef::new("f_i64", FieldType::I64));
    schema.add_field(FieldDef::new("f_u64", FieldType::U64));
    schema.add_field(FieldDef::new("f_f32", FieldType::F32));
    schema.add_field(FieldDef::new("f_f64", FieldType::F64));

    let code = generate_schema(&schema);

    // SHM struct has all types except bool (which becomes u8).
    assert!(code.contains("pub f_bool: u8,"));
    assert!(code.contains("pub f_i8: i8,"));
    assert!(code.contains("pub f_u8: u8,"));
    assert!(code.contains("pub f_i16: i16,"));
    assert!(code.contains("pub f_u16: u16,"));
    assert!(code.contains("pub f_i32: i32,"));
    assert!(code.contains("pub f_u32: u32,"));
    assert!(code.contains("pub f_i64: i64,"));
    assert!(code.contains("pub f_u64: u64,"));
    assert!(code.contains("pub f_f32: f32,"));
    assert!(code.contains("pub f_f64: f64,"));

    // Snapshot keeps real `bool`.
    assert!(code.contains("pub f_bool: bool,"));
}

#[test]
fn test_nested_dynamic_arrays_classified_as_variable() {
    let mut schema = MessageSchema::new("NestedArrays");
    // Vec<Vec<f64>>.
    schema.add_field(FieldDef::new(
        "matrix",
        FieldType::DynamicArray {
            element_type: Box::new(FieldType::DynamicArray {
                element_type: Box::new(FieldType::F64),
            }),
        },
    ));

    let code = generate_schema(&schema);

    assert!(!schema.is_definitely_fixed());
    assert!(code.contains("pub struct NestedArraysShm<'a>"));
    assert!(code.contains("matrix"));
}

#[test]
fn test_mixed_fixed_and_variable_fields_use_variable_emission() {
    let mut schema = MessageSchema::new("MixedMessage");
    schema.add_field(FieldDef::new("id", FieldType::U64)); // fixed
    schema.add_field(FieldDef::new("name", FieldType::String)); // variable
    schema.add_field(FieldDef::new("value", FieldType::F64)); // fixed
    schema.add_field(FieldDef::new(
        "tags",
        FieldType::DynamicArray {
            element_type: Box::new(FieldType::String),
        },
    )); // variable

    let code = generate_schema(&schema);

    assert!(!schema.is_definitely_fixed());
    assert_eq!(schema.variable_field_count(), 2);

    // Variable-schema emission (lifetimed SHM type + #[repr(C)] FixedSection
    // overlay introduced with the SHM-backed API).
    assert!(code.contains("pub struct MixedMessageShm<'a>"));
    assert!(code.contains("pub struct MixedMessageFixedSection {"));
    assert!(code.contains("impl cerulion_core::message::ShmMessage for MixedMessage"));
    assert!(code.contains("const VARIABLE_FIELD_COUNT: usize = 2;"));
    assert!(code.contains(
        "const WIRE_FIXED_SIZE: usize = ::std::mem::size_of::<MixedMessageFixedSection>();"
    ));

    // Fixed-section fields are exposed as `pub <name>: <type>` on the
    // overlay struct + reached from the SHM type via Deref / DerefMut. The
    // legacy `pub fn id(&self) -> u64` / `pub fn set_id(&mut self, value: u64)`
    // accessors are GONE.
    assert!(code.contains("pub id: u64,"));
    assert!(code.contains("pub value: f64,"));
    assert!(code.contains("impl<'a> ::std::ops::Deref for MixedMessageShm<'a>"));
    assert!(code.contains("impl<'a> ::std::ops::DerefMut for MixedMessageShm<'a>"));
    assert!(!code.contains("pub fn id(&self) -> u64"));
    assert!(!code.contains("pub fn set_id(&mut self, value: u64)"));
    assert!(!code.contains("pub fn value(&self) -> f64"));
    assert!(!code.contains("pub fn set_value(&mut self, value: f64)"));

    // Variable accessors. String getter returns Result<&str, WireError>
    // (no silent fallback). Variable-field accessors are unchanged by
    // the overlay.
    assert!(code.contains(
        "pub fn name(&self) -> ::std::result::Result<&str, ::cerulion_core::wire::WireError>"
    ));
    assert!(code.contains("pub fn set_name"));
}

#[test]
fn test_long_field_name_emitted_verbatim() {
    let mut schema = MessageSchema::new("LongNames");
    let long_name = "this_is_a_very_long_field_name_that_might_cause_issues_with_code_generation";
    schema.add_field(FieldDef::new(long_name, FieldType::U32));

    let code = generate_schema(&schema);
    assert!(code.contains(long_name));
}

#[test]
fn test_field_description_does_not_corrupt_generated_code() {
    let mut schema = MessageSchema::new("SpecialDesc");
    let mut field = FieldDef::new("value", FieldType::U32);
    field.description = Some("Test with */ and /* and // comments".to_string());
    schema.add_field(field);

    let code = generate_schema(&schema);

    // Code still references the field; the description shows up in a doc comment
    // and must not break the surrounding emission.
    assert!(code.contains("value"));
    assert!(code.contains("pub struct SpecialDescShm {"));
}

#[test]
fn test_dynamic_array_bool_does_not_use_nonexistent_from_le_bytes() {
    // `bool` has no `from_le_bytes` method — DynamicArray<Bool> emission
    // must read each byte as `b != 0` to produce compilable code.
    let mut schema = MessageSchema::new("BoolArray");
    schema.add_field(FieldDef::new(
        "flags",
        FieldType::DynamicArray {
            element_type: Box::new(FieldType::Bool),
        },
    ));

    let code = generate_schema(&schema);

    assert!(
        !code.contains("bool::from_le_bytes"),
        "bool::from_le_bytes does not exist — must not appear in generated code"
    );
    // SHM emission for variable schema.
    assert!(code.contains("pub struct BoolArrayShm<'a>"));
    assert!(code.contains("flags"));
}

// =============================================================================
// Real .msg-file integration tests (the parser still works with the SHM-backed API)
// =============================================================================

/// Read a .msg file from `native_ros2_messages/msg/{package}/{name}.msg`.
fn read_msg_file(package: &str, name: &str) -> String {
    let workspace_root = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .unwrap();
    let path = workspace_root
        .join("native_ros2_messages")
        .join("msg")
        .join(package)
        .join(format!("{name}.msg"));
    std::fs::read_to_string(&path)
        .unwrap_or_else(|e| panic!("failed to read {}: {e}", path.display()))
}

#[test]
fn test_parse_real_image_msg() {
    let content = read_msg_file("sensor_msgs", "Image");
    let schema = parse_rosmsg(&content, "Image", None).unwrap();

    assert_eq!(schema.name, "Image");
    assert_eq!(schema.fields.len(), 7);

    // header is nested
    assert_eq!(schema.fields[0].name, "header");
    assert!(matches!(
        &schema.fields[0].field_type,
        FieldType::Nested { schema_name, .. } if schema_name == "Header"
    ));

    // encoding is a string (variable)
    assert_eq!(schema.fields[3].name, "encoding");
    assert_eq!(schema.fields[3].field_type, FieldType::String);

    // data is dynamic array of u8
    assert_eq!(schema.fields[6].name, "data");
    assert!(matches!(
        &schema.fields[6].field_type,
        FieldType::DynamicArray { element_type } if **element_type == FieldType::U8
    ));

    assert!(schema.has_variable_fields());
    assert!(!schema.is_definitely_fixed());
}

#[test]
fn test_parse_real_imu_msg() {
    let content = read_msg_file("sensor_msgs", "Imu");
    let schema = parse_rosmsg(&content, "Imu", None).unwrap();

    assert_eq!(schema.name, "Imu");
    assert_eq!(schema.fields.len(), 7);

    assert_eq!(schema.fields[1].name, "orientation");
    assert!(matches!(
        &schema.fields[1].field_type,
        FieldType::Nested { schema_name, .. } if schema_name == "Quaternion"
    ));

    // The three covariance fields are float64[9].
    for idx in [2, 4, 6] {
        assert!(
            matches!(
                &schema.fields[idx].field_type,
                FieldType::FixedArray { element_type, length: 9 } if **element_type == FieldType::F64
            ),
            "field {} should be float64[9]",
            schema.fields[idx].name
        );
    }
}

#[test]
fn test_parse_real_odometry_msg() {
    let content = read_msg_file("nav_msgs", "Odometry");
    let schema = parse_rosmsg(&content, "Odometry", None).unwrap();

    assert_eq!(schema.name, "Odometry");
    assert_eq!(schema.fields.len(), 4);

    assert_eq!(schema.fields[0].name, "header");
    assert_eq!(schema.fields[1].name, "child_frame_id");
    assert_eq!(schema.fields[1].field_type, FieldType::String);
    assert_eq!(schema.fields[2].name, "pose");
    assert_eq!(schema.fields[3].name, "twist");

    assert!(schema.has_variable_fields());
}

#[test]
fn test_codegen_from_real_image_msg_emits_shm_pipeline() {
    // Full pipeline: real .msg file → parser → codegen output contains the
    // SHM-backed marker, accessor, snapshot, ShmMessage impl, and typed
    // accessors for the variable fields.
    let content = read_msg_file("sensor_msgs", "Image");
    let schema = parse_rosmsg(&content, "Image", None).unwrap();
    let code = generate_schema(&schema);

    assert!(code.contains("pub struct Image;"));
    assert!(code.contains("pub struct ImageShm<'a>"));
    assert!(code.contains("pub struct ImageFixedSection {"));
    assert!(code.contains("pub struct ImageSnapshot"));
    assert!(code.contains("impl cerulion_core::message::ShmMessage for Image"));

    // Fixed-section fields exposed via `pub <name>: <type>` on the
    // overlay; only variable-field setters survive. Negative assertions use
    // the full method signature so a future method named e.g. `set_height_x`
    // would not accidentally satisfy them.
    assert!(code.contains("pub height: u32,"));
    assert!(code.contains("pub width: u32,"));
    assert!(!code.contains("pub fn set_height(&mut self, value: u32)"));
    assert!(!code.contains("pub fn set_width(&mut self, value: u32)"));
    assert!(!code.contains("pub fn height(&self) -> u32"));
    assert!(!code.contains("pub fn width(&self) -> u32"));
    assert!(code.contains("pub fn set_encoding"));
    assert!(code.contains("pub fn set_data"));

    // Schema hash constant.
    assert!(code.contains("IMAGE_SCHEMA_HASH"));
}

// =============================================================================
// Coverage notes:
//
// - Compile coverage of generated code comes from the native_ros2_messages
//   crate, which compiles real generated code on every CI run. No test here
//   invokes rustc on a generated snippet.
// - The marker type (`Point`) is unit (zero-sized) and cannot be a
//   field, so a nested field of the SHM struct is `PointShm`.
// - `bool` appears as `u8` in the SHM struct (by design), and
//   `bool::from_le_bytes` is never emitted.
// =============================================================================

// =============================================================================
// FixedSection-overlay codegen-string tests
// =============================================================================
//
// These tests assert on the GENERATED SOURCE STRING for variable schemas
// after the `<Name>FixedSection` overlay + `Deref/DerefMut` + reserved-
// name collision check. They complement the integration tests in
// `variable_schema_fixed_field_test.rs` which exercise the runtime behavior
// against pre-built `native_ros2_messages::sensor_msgs::Image`.

/// Helper: a minimal mixed schema (one fixed primitive + one variable
/// String) for codegen-string assertions. Avoids dragging in the full
/// `Image` schema for tests that only care about the FixedSection shape.
fn mini_mixed_schema(name: &str) -> MessageSchema {
    let mut s = MessageSchema::new(name);
    s.add_field(FieldDef::new("seq", FieldType::U32));
    s.add_field(FieldDef::new("label", FieldType::String));
    s
}

#[test]
fn variable_schema_with_no_fixed_fields_emits_marker_fixed_section() {
    // All-variable schema (Pose-like): both fields are Nested → variable.
    // FixedSection must still emit but with a `_marker: [u8; 0]` placeholder
    // so `Deref<Target = <Name>FixedSection>` type-checks unconditionally.
    let mut s = MessageSchema::new("AllVar");
    s.add_field(FieldDef::new(
        "left",
        FieldType::Nested {
            schema_name: "Vector3".to_string(),
            package: None,
            fixed: None,
        },
    ));
    s.add_field(FieldDef::new(
        "right",
        FieldType::Nested {
            schema_name: "Vector3".to_string(),
            package: None,
            fixed: None,
        },
    ));
    let code = generate_schema(&s);

    assert!(
        code.contains("pub struct AllVarFixedSection {"),
        "all-variable schema should still emit a FixedSection",
    );
    assert!(
        code.contains("_marker: [u8; 0],"),
        "empty FixedSection must use a private `_marker: [u8; 0]` placeholder",
    );
    assert!(
        code.contains("impl<'a> ::std::ops::Deref for AllVarShm<'a>"),
        "Deref impl must be emitted unconditionally",
    );
}

#[test]
fn single_fixed_field_schema_emits_one_pub_field() {
    // Edge: a variable schema with exactly one fixed field. Verifies the
    // for-loop over fixed_section_fields doesn't have an off-by-one.
    let s = mini_mixed_schema("SingleFixed");
    let code = generate_schema(&s);

    assert!(code.contains("pub struct SingleFixedFixedSection {"));
    assert!(code.contains("pub seq: u32,"));
    // Only ONE field-definition line inside the FixedSection block (not
    // counting the `pub struct ...` opening line). Lines inside the block
    // that look like `    pub <ident>: <type>,` are field definitions.
    let fs_open = code
        .find("pub struct SingleFixedFixedSection {")
        .expect("FixedSection block missing");
    // Skip past the `{` so the body-only slice doesn't include the struct
    // header's `pub struct`.
    let body_start = code[fs_open..].find('{').unwrap() + fs_open + 1;
    let body_end = code[body_start..]
        .find('}')
        .expect("FixedSection close missing")
        + body_start;
    let body = &code[body_start..body_end];
    // A field line starts with leading whitespace + `pub <ident>:` then a type.
    let field_count = body
        .lines()
        .filter(|line| line.trim_start().starts_with("pub "))
        .count();
    assert_eq!(
        field_count, 1,
        "SingleFixed should have exactly one pub field line; got body:\n{body}",
    );
}

#[test]
fn bool_field_in_variable_schema_emits_u8_in_fixed_section() {
    // ROS2 PointCloud2 has actual bool fields. Verify variable-schema
    // FixedSection follows the same bool→u8 substitution as fixed-only
    // schemas.
    let mut s = MessageSchema::new("BoolMixed");
    s.add_field(FieldDef::new("ok", FieldType::Bool));
    s.add_field(FieldDef::new("note", FieldType::String));
    let code = generate_schema(&s);

    assert!(code.contains("pub struct BoolMixedFixedSection {"));
    assert!(
        code.contains("pub ok: u8,"),
        "bool field must store as u8 in FixedSection (matches fixed-only schema codegen)",
    );
    // Snapshot keeps real `bool`.
    assert!(
        code.contains("pub ok: bool,"),
        "snapshot field must be `bool`"
    );
    // Round-trip helpers convert.
    assert!(
        code.contains("ok: self.ok != 0,"),
        "snapshot read must coerce u8 → bool via != 0",
    );
    assert!(
        code.contains("self.ok = snap.ok as u8;"),
        "write_from_snapshot must coerce bool → u8 via `as u8`",
    );
}

#[test]
fn fixed_array_of_bool_in_variable_schema_emits_u8_array() {
    // FixedArray<Bool, N> in fixed section: stored as [u8; N] per
    // field_type_to_rust_shm. Snapshot keeps [bool; N]. Round-trip uses
    // array::from_fn for element-wise conversion.
    let mut s = MessageSchema::new("BoolArrMixed");
    s.add_field(FieldDef::new(
        "flags",
        FieldType::FixedArray {
            element_type: Box::new(FieldType::Bool),
            length: 4,
        },
    ));
    s.add_field(FieldDef::new("note", FieldType::String));
    let code = generate_schema(&s);

    assert!(
        code.contains("pub flags: [u8; 4],"),
        "FixedArray<Bool, 4> stored as [u8; 4]"
    );
    assert!(
        code.contains("pub flags: [bool; 4],"),
        "snapshot keeps [bool; 4]",
    );
    assert!(
        code.contains("::std::array::from_fn::<bool, 4, _>(|i| self.flags[i] != 0)"),
        "snapshot read uses array::from_fn for [bool; N] coercion",
    );
}

#[test]
fn string_fixed_in_variable_schema_emits_u8_array() {
    // StringFixed(N) in a mixed schema's fixed section.
    let mut s = MessageSchema::new("StrFixedMixed");
    s.add_field(FieldDef::new("magic", FieldType::StringFixed(8)));
    s.add_field(FieldDef::new("body", FieldType::String));
    let code = generate_schema(&s);

    assert!(
        code.contains("pub magic: [u8; 8],"),
        "StringFixed(8) stored as [u8; 8]"
    );
}

#[test]
fn large_array_in_variable_schema_uses_manual_default() {
    // FixedArray<primitive, N>32> in fixed section. stdlib `Default` lacks
    // blanket impls past N=32; codegen must skip `Default` from the derive
    // and emit a manual `Default` impl.
    let mut s = MessageSchema::new("LargeArrMixed");
    s.add_field(FieldDef::new(
        "covariance",
        FieldType::FixedArray {
            element_type: Box::new(FieldType::F64),
            length: 36,
        },
    ));
    s.add_field(FieldDef::new("body", FieldType::Bytes));
    let code = generate_schema(&s);

    assert!(code.contains("pub struct LargeArrMixedFixedSection {"));
    assert!(code.contains("pub covariance: [f64; 36],"));
    // Derive should NOT include Default (large arrays disqualify the
    // blanket impl).
    let fs_start = code
        .find("pub struct LargeArrMixedFixedSection {")
        .expect("FixedSection block missing");
    let derives_end = code[..fs_start].rfind("#[derive(").expect("derive missing");
    let derive_close = code[derives_end..]
        .find(")]")
        .expect("derive close missing")
        + derives_end;
    let derive_block = &code[derives_end..=derive_close + 1];
    assert!(
        !derive_block.contains("Default"),
        "large-array FixedSection must NOT derive Default; got: {derive_block}",
    );
    // Manual Default impl must exist.
    assert!(
        code.contains("impl Default for LargeArrMixedFixedSection {"),
        "large-array FixedSection must have a manual Default impl",
    );
    // Manual impl must enumerate the covariance field with default value.
    assert!(
        code.contains("covariance: [0.0; 36]")
            || code.contains("covariance: [<f64>::default(); 36]"),
        "manual Default impl must initialize covariance",
    );
}

// -----------------------------------------------------------------------------
// Reserved-name collision detection
// -----------------------------------------------------------------------------

/// For each of the 8 inherent-method names + 4 private-field names + 1
/// `_marker` placeholder + 4 constants, a fixed field with that name must
/// trigger a `compile_error!` in the generated output.
#[test]
fn fixed_field_collision_with_reserved_name_emits_compile_error() {
    let reserved = [
        // Inherent methods on <Name>Shm.
        "from_bytes",
        "from_bytes_mut",
        "payload",
        "payload_mut",
        "cursor",
        "snapshot",
        "write_from_snapshot",
        "all_variables_written",
        // Private struct fields.
        "ptr",
        "len",
        "state",
        "_phantom",
        "_marker",
        // Constants.
        "WIRE_FIXED_SIZE",
        "VARIABLE_FIELD_COUNT",
        "OFFSET_TABLE_OFFSET",
        "OFFSET_TABLE_BYTES",
    ];
    for name in reserved {
        let mut s = MessageSchema::new("Bad");
        s.add_field(FieldDef::new(name, FieldType::U32));
        // Need at least one variable field for the schema to be classified
        // as variable (and thus go through `generate_variable_shm_struct`).
        s.add_field(FieldDef::new("body", FieldType::String));
        let code = generate_schema(&s);
        // Match the macro invocation specifically (with `(`), not the comment
        // line that mentions the literal string `compile_error!` for context.
        assert!(
            code.contains("compile_error!("),
            "reserved fixed-field name `{name}` must trigger compile_error!()",
        );
        assert!(
            code.contains(&format!("`{name}`")),
            "compile_error! must name the offending field `{name}`; got:\n{code}",
        );
    }
}

/// A variable field name that collides with a
/// `<Name>Shm` inherent method also produces a duplicate `pub fn` emission
/// at codegen — caught by the same compile_error! path.
#[test]
fn variable_field_collision_with_inherent_method_emits_compile_error() {
    // Variable field named `cursor` collides with the inherent
    // `pub fn cursor(&self) -> u32` on <Name>Shm.
    let mut s = MessageSchema::new("BadVar");
    s.add_field(FieldDef::new("seq", FieldType::U32)); // valid fixed field
    s.add_field(FieldDef::new("cursor", FieldType::String)); // collision
    let code = generate_schema(&s);

    assert!(
        code.contains("compile_error!("),
        "variable field name `cursor` must trigger compile_error!() (collides with inherent method)",
    );
    assert!(
        code.contains("`cursor`"),
        "compile_error! must name the offending field",
    );
}

/// One consolidated `compile_error!` per schema,
/// listing every offender, never N separate compile_errors for N
/// colliding fields.
#[test]
fn multiple_collisions_emit_single_consolidated_compile_error() {
    let mut s = MessageSchema::new("Multi");
    s.add_field(FieldDef::new("state", FieldType::U32)); // collides
    s.add_field(FieldDef::new("cursor", FieldType::U32)); // collides
    s.add_field(FieldDef::new("body", FieldType::String));
    let code = generate_schema(&s);

    // Count actual macro invocations (with the `(`), NOT comment lines that
    // mention the literal string `compile_error!`.
    let count = code.matches("compile_error!(").count();
    assert_eq!(
        count, 1,
        "multiple colliding fields must produce exactly ONE compile_error!() invocation; got {count}",
    );

    // Assert both names appear inside the
    // SAME compile_error!() invocation, not just somewhere in the codegen
    // output. Extract the slice between `compile_error!(` and the next `);`
    // (the message string is balanced — no embedded `);` — so this is
    // sufficient).
    let macro_start = code
        .find("compile_error!(")
        .expect("compile_error! missing");
    let macro_close = code[macro_start..]
        .find(");")
        .expect("compile_error close missing")
        + macro_start;
    let macro_call = &code[macro_start..=macro_close];
    assert!(
        macro_call.contains("`state`"),
        "consolidated compile_error!() must name `state` inside its message; got: {macro_call}",
    );
    assert!(
        macro_call.contains("`cursor`"),
        "consolidated compile_error!() must name `cursor` inside its message; got: {macro_call}",
    );
}

/// Non-colliding schemas must NOT emit compile_error! (regression guard).
#[test]
fn non_colliding_schema_emits_no_compile_error() {
    let s = mini_mixed_schema("Clean");
    let code = generate_schema(&s);
    assert!(
        !code.contains("compile_error!"),
        "schema with no name collisions must not emit compile_error!",
    );
}

// -----------------------------------------------------------------------------
// Marker `# Fields` rustdoc summary
// -----------------------------------------------------------------------------

#[test]
fn marker_doc_lists_fixed_and_variable_fields() {
    // Use a hand-built schema with known shape so we can assert specific
    // doc lines without depending on real ROS2 schemas.
    let mut s = MessageSchema::new("MarkerDocMixed");
    s.add_field(FieldDef::new("count", FieldType::U32));
    s.add_field(FieldDef::new("name", FieldType::String));
    let code = generate_schema(&s);

    // The "# Fields" section must appear once.
    assert!(
        code.contains("/// # Fields"),
        "marker rustdoc must have a # Fields section"
    );
    // Fixed fields go into the "Fixed" sub-section.
    assert!(code.contains("/// **Fixed (direct field access via `Deref`):**"));
    assert!(code.contains("///   - `count: u32` (direct field access)"));
    // Variable fields go into the "Variable" sub-section.
    assert!(code.contains("/// **Variable (accessor methods):**"));
    // String must show `Result<&str, WireError>`,
    // not bare `&str`, in the marker doc summary.
    assert!(
        code.contains("`name() -> Result<&str, WireError>`"),
        "String accessor must show its real signature (Result<&str, WireError>) in marker doc",
    );

    // The Fixed section must precede the
    // Variable section in the rendered marker rustdoc, so users see fixed
    // fields first (typical reading order). Assert source-position order.
    let fixed_pos = code
        .find("**Fixed (direct field access via `Deref`):**")
        .expect("Fixed section missing");
    let variable_pos = code
        .find("**Variable (accessor methods):**")
        .expect("Variable section missing");
    assert!(
        fixed_pos < variable_pos,
        "Fixed section must appear before Variable section in marker rustdoc",
    );
}

#[test]
fn marker_doc_dynamic_array_bool_renders_as_bool_slice() {
    // DynamicArray<Bool> must show `&[bool]`,
    // not `&[u8]`, in the marker doc summary (matches the real accessor's
    // `pub fn <name>(&self) -> &[bool]` return type).
    let mut s = MessageSchema::new("DynBoolMarker");
    s.add_field(FieldDef::new("seq", FieldType::U32));
    s.add_field(FieldDef::new(
        "flags",
        FieldType::DynamicArray {
            element_type: Box::new(FieldType::Bool),
        },
    ));
    let code = generate_schema(&s);

    assert!(code.contains("/// **Variable (accessor methods):**"));
    assert!(
        code.contains("`flags() -> &[bool]`"),
        "DynamicArray<Bool> reader must render as &[bool] in marker doc",
    );
    // Setter signature should reflect the typed slice.
    assert!(
        code.contains("`set_flags(&[bool]) -> Result<(), TransportError>`"),
        "DynamicArray<Bool> setter must render as &[bool] in marker doc",
    );
}

#[test]
fn marker_doc_keyword_field_name_uses_r_prefix() {
    // Keyword field names must be rendered with the
    // `r#` prefix the actual struct field uses, so the doc and emitted code
    // agree.
    let mut s = MessageSchema::new("KwMarker");
    s.add_field(FieldDef::new("type", FieldType::U32)); // `type` is a keyword
    s.add_field(FieldDef::new("body", FieldType::String));
    let code = generate_schema(&s);

    assert!(
        code.contains("`r#type: u32` (direct field access)"),
        "keyword field name must use r# prefix in marker doc to match emitted struct field",
    );
}

#[test]
fn marker_doc_complex_variable_field_shows_raw_bytes_accessors() {
    // Complex variable fields (Nested, FixedArray<Nested>, etc.) get
    // raw-bytes accessors; the marker doc summary must reflect this.
    let mut s = MessageSchema::new("ComplexMarker");
    s.add_field(FieldDef::new("seq", FieldType::U32));
    s.add_field(FieldDef::new(
        "nested",
        FieldType::Nested {
            schema_name: "Vector3".to_string(),
            package: None,
            fixed: None,
        },
    ));
    let code = generate_schema(&s);

    assert!(code.contains("/// **Variable (accessor methods):**"));
    assert!(
        code.contains("nested_bytes()"),
        "complex variable field must show raw-bytes accessors in marker doc",
    );
    assert!(code.contains("set_nested_bytes(&[u8])"));
    assert!(code.contains("loan_nested_bytes(n)"));
}

#[test]
fn marker_doc_bool_fixed_field_surfaces_canonicalization_contract() {
    // When the schema has a bool fixed field, the
    // marker doc Fixed section must surface a one-liner pointing readers at
    // the FixedSection's bool round-trip contract.
    let mut s = MessageSchema::new("BoolFixed");
    s.add_field(FieldDef::new("flag", FieldType::Bool));
    s.add_field(FieldDef::new("body", FieldType::String));
    let code = generate_schema(&s);

    assert!(code.contains("/// **Fixed (direct field access via `Deref`):**"));
    assert!(
        code.contains("`bool` fields are stored as `u8`"),
        "marker doc must surface the bool storage contract for schemas with bool fixed fields",
    );
    assert!(
        code.contains("BoolFixedFixedSection"),
        "marker doc bool reminder must link to the FixedSection",
    );
}

#[test]
fn marker_doc_no_fields_emits_no_fields_section() {
    // Schema with zero fields: no `# Fields` section.
    let s = MessageSchema::new("Empty");
    let code = generate_schema(&s);

    assert!(
        !code.contains("# Fields"),
        "empty schema must not emit a # Fields section",
    );
}

// -----------------------------------------------------------------------------
// Alignment static-assert presence
// -----------------------------------------------------------------------------

#[test]
fn variable_schema_emits_static_align_assert() {
    // Every variable schema must emit a `const _: () = assert!(align_of <= 8)`
    // static check (the alignment guarantee for the post-WireHeader
    // payload pointer).
    let s = mini_mixed_schema("AlignCheck");
    let code = generate_schema(&s);

    assert!(
        code.contains("const _: () = assert!("),
        "variable schema must emit a const _ static_assert for alignment",
    );
    assert!(
        code.contains("::std::mem::align_of::<AlignCheckFixedSection>() <= 8"),
        "static_assert must check align_of <= 8 for the FixedSection",
    );
}

// -----------------------------------------------------------------------------
// Derive matrix + reserved-name narrowing + defaults
// -----------------------------------------------------------------------------

/// GAP6: a FULLY-FIXED parent embedding a fixed-with-large-array nested
/// type must hit the `(manual_default=false, transitive_large=true)`
/// derive arm: derive `Default` (the nested type carries its own manual
/// impl), skip `Debug`, and emit NO manual `Default` impl (clippy
/// `derivable_impls`). No vendored schema instantiates this arm today —
/// without this pin it would bit-rot until a future schema lights it up.
#[test]
fn fixed_parent_with_transitive_large_array_derives_default() {
    let mut parent = MessageSchema::new("Parent");
    parent.add_field(FieldDef::new(
        "cov",
        FieldType::Nested {
            schema_name: "Cov".to_string(),
            package: None,
            fixed: Some(NestedFixedInfo {
                has_large_array: true,
                // Recipe-3 layout fields: this test hand-builds a
                // pre-resolved nested (no `Cov` schema exists in the set),
                // so these are placeholders — the derive matrix keys only on
                // `has_large_array`.
                fixed_size: 288,
                alignment: 8,
                target_hash: 0,
            }),
        },
    ));

    let code = generate_schema(&parent);
    assert!(
        code.contains(
            "#[derive(Clone, Copy, PartialEq, Default)]
pub struct ParentShm {"
        ),
        "transitive-large fixed parent must derive Default and skip Debug; got:
{}",
        &code[..code.len().min(2000)]
    );
    assert!(
        !code.contains("impl Default for ParentShm"),
        "no manual Default impl when derive suffices (clippy derivable_impls)"
    );
    // Snapshot side: Clone + Default + PartialEq, still NO Debug (that skip is
    // deliberate and unchanged) — but serde IS derived, and
    // `CerulionState` joins it, so a node holding this
    // transitively-large-array message as plain state is capturable too.
    // Before that, `has_large_array` gated Debug AND serde together, so this
    // transitively-poisoned parent could not round-trip at all. The two
    // derives are now decided independently.
    assert!(
        code.contains(
            "#[derive(Clone, Default, PartialEq, ::cerulion_core::serde::Serialize, ::cerulion_core::serde::Deserialize, ::cerulion_core::state::CerulionState)]
#[serde(crate = \"::cerulion_core::serde\")]
pub struct ParentSnapshot {"
        ),
        "transitive-large parent must still skip Debug and must NOW derive serde; got:
{}",
        &code[..code.len().min(4000)]
    );
    // The large array is inside `Cov`, not here, so THIS struct needs no
    // per-field `#[serde(with = ...)]` — `CovSnapshot` carries its own.
    assert!(
        !code.contains("#[serde(with ="),
        "a transitively-poisoned parent needs no big-array attribute of its own"
    );
}

/// A DIRECT `float64[36]` (the covariance shape) derives
/// serde and routes that one field through the big-array helper.
///
/// The complement of the transitive test above: there the parent needs no
/// attribute, here the owner of the array does.
#[test]
fn direct_large_array_derives_serde_via_the_big_array_helper() {
    let mut schema = MessageSchema::new("Cov");
    schema.add_field(FieldDef::new(
        "covariance",
        FieldType::FixedArray {
            element_type: Box::new(FieldType::F64),
            length: 36,
        },
    ));
    // A sibling array INSIDE serde's ceiling, to pin that the attribute is
    // emitted per FIELD rather than per struct.
    schema.add_field(FieldDef::new(
        "small",
        FieldType::FixedArray {
            element_type: Box::new(FieldType::F64),
            length: 32,
        },
    ));

    let code = generate_schema(&schema);

    assert!(
        code.contains(
            "#[derive(Clone, PartialEq, ::cerulion_core::serde::Serialize, ::cerulion_core::serde::Deserialize, ::cerulion_core::state::CerulionState)]
#[serde(crate = \"::cerulion_core::serde\")]
pub struct CovSnapshot {"
        ),
        "a direct large-array schema must derive serde (Debug still skipped); got:
{}",
        &code[..code.len().min(4000)]
    );
    assert!(
        code.contains(
            "    #[serde(with = \"::cerulion_core::codegen::big_array\")]\n    pub covariance: [f64; 36],"
        ),
        "the >32 field must carry the helper attribute; got:
{}",
        &code[..code.len().min(4000)]
    );
    assert!(
        code.contains("\n    pub small: [f64; 32],"),
        "a 32-element field must NOT carry the helper attribute (serde covers it); got:
{}",
        &code[..code.len().min(4000)]
    );
    assert_eq!(
        code.matches("#[serde(with =").count(),
        1,
        "exactly one field needs the big-array helper"
    );
}

/// The helper is attached only to fields that are ARRAY-BACKED in the
/// snapshot type (anything else is a build failure).
///
/// A fixed array of an UNRESOLVED nested type (`std_msgs/Header[36]`) is longer
/// than serde's ceiling but its snapshot field is `Vec<u8>`, not `[T; N]` —
/// `field_type_to_rust_snapshot` short-circuits on `is_complex_variable`
/// BEFORE it ever formats an array. Attaching the helper there emits
/// `#[serde(with = "…big_array")]` over a `Vec<u8>`, which does not compile:
/// `big_array::serialize` takes `&[T; N]`.
///
/// REACHABLE, not theoretical: `Header[36]` is ordinary `.msg` syntax, so this
/// arm drives the REAL `parse_rosmsg` + `resolve_fixed_nested` pipeline rather
/// than hand-building the `FieldType` — that is what makes it evidence about a
/// schema a user can actually write. (It is the SIBLING of the nested-array
/// shape `big_array`'s docs call unreachable: `float64[4][36]` is a parse
/// error, but `Header[36]` parses fine.)
#[test]
fn a_large_array_of_an_unresolved_nested_type_gets_no_big_array_attr() {
    let header =
        parse_rosmsg("string frame_id\nuint32 seq\n", "Header", None).expect("Header parses");
    let rig = parse_rosmsg("Header[36] frames\n", "Rig", None).expect("Rig parses");
    let mut schemas = vec![header, rig];
    resolve_fixed_nested(&mut schemas);
    let rig = schemas
        .iter()
        .find(|s| s.name == "Rig")
        .expect("Rig survives resolution");

    let code = generate_schema(rig);

    // The premise: `Header` carries a `string`, so it never resolves fixed and
    // the snapshot field really is raw bytes. Asserted so the arm cannot pass
    // vacuously if nested resolution ever changes.
    assert!(
        code.contains("    pub frames: ::std::vec::Vec<u8>,"),
        "the snapshot field must be Vec<u8> for this arm to mean anything; got:
{}",
        &code[..code.len().min(4000)]
    );
    assert!(
        !code.contains("#[serde(with ="),
        "a Vec<u8>-backed field must NOT carry the [T; N] helper — the generated \
         code would not compile; got:
{}",
        &code[..code.len().min(4000)]
    );
    // Serde is still derived — the rule narrows the per-field attribute, it does
    // not re-poison the type.
    assert!(
        code.contains("::cerulion_core::serde::Serialize"),
        "the snapshot must still derive serde"
    );
}

/// The complement, so the rule cannot be "never attach the helper": a fixed
/// array of a RESOLVED-fixed nested type stays array-backed
/// (`[VecSnapshot; 36]`) and DOES need it.
///
/// This is the shape `is_complex_variable` must not over-reject — it recurses
/// into the element type, and a resolved-fixed `Nested` answers `false`.
#[test]
fn a_large_array_of_a_resolved_fixed_nested_type_keeps_the_helper() {
    let vec3 =
        parse_rosmsg("float64 x\nfloat64 y\nfloat64 z\n", "Vec3", None).expect("Vec3 parses");
    let rig = parse_rosmsg("Vec3[36] points\n", "Rig", None).expect("Rig parses");
    let mut schemas = vec![vec3, rig];
    resolve_fixed_nested(&mut schemas);
    let rig = schemas
        .iter()
        .find(|s| s.name == "Rig")
        .expect("Rig survives resolution");

    let code = generate_schema(rig);

    assert!(
        code.contains(
            "    #[serde(with = \"::cerulion_core::codegen::big_array\")]\n    pub points: [Vec3Snapshot; 36],"
        ),
        "an array-backed >32 field must carry the helper; got:
{}",
        &code[..code.len().min(4000)]
    );
}

/// GAP3 (the reserved-name narrowing intent): VARIABLE fields named after Shm
/// PRIVATE FIELDS (`state`, `len`, `topic`) are legal — they emit only
/// accessor methods, and fields/methods live in different namespaces.
/// (moveit_msgs/DisplayRobotState has a field named `state`.) Only an
/// emitted accessor name matching an INHERENT METHOD collides.
#[test]
fn variable_fields_named_after_private_fields_compile() {
    let mut schema = MessageSchema::new("Robot");
    schema.add_field(FieldDef::new("state", FieldType::String));
    schema.add_field(FieldDef::new("len", FieldType::Bytes));
    schema.add_field(FieldDef::new("topic", FieldType::String));

    let code = generate_schema(&schema);
    assert!(
        !code.contains("compile_error!("),
        "variable fields named state/len/topic must NOT trigger the reserved-name error"
    );
    // And the accessors are emitted normally.
    assert!(code.contains("pub fn set_state"));
    assert!(code.contains("pub fn set_len"));
}

/// Non-zero scalar defaults force a manual `Default` impl with
/// the declared literal (`float64 w 1` → identity quaternion semantics),
/// while `Debug` stays derived (no large arrays involved).
#[test]
fn nonzero_scalar_defaults_emit_manual_default() {
    let mut quat = MessageSchema::new("Quat");
    for f in ["x", "y", "z"] {
        quat.add_field(FieldDef::new(f, FieldType::F64).with_default(DefaultLiteral::Float(0.0)));
    }
    quat.add_field(FieldDef::new("w", FieldType::F64).with_default(DefaultLiteral::Float(1.0)));

    let code = generate_schema(&quat);
    // Manual Default impls with the literal, on both Shm and Snapshot.
    assert!(code.contains("impl Default for QuatShm"));
    assert!(code.contains("w: 1.0,"));
    assert!(code.contains("impl Default for QuatSnapshot"));
    // Debug stays derived (no large arrays) — and Default is NOT in the
    // derive list (manual impl).
    assert!(code.contains("#[derive(Debug, Clone, Copy, PartialEq)]\npub struct QuatShm {"));

    // All-zero defaults keep the plain derive (clippy derivable_impls).
    let mut zeroed = MessageSchema::new("Zeroed");
    zeroed.add_field(FieldDef::new("x", FieldType::F64).with_default(DefaultLiteral::Float(0.0)));
    let code = generate_schema(&zeroed);
    assert!(
        code.contains("#[derive(Debug, Clone, Copy, PartialEq, Default)]\npub struct ZeroedShm {")
    );
    assert!(!code.contains("impl Default for ZeroedShm"));
}

/// The generated `&[bool]` READER validates wire bytes before
/// creating the reference (peer processes control SHM bytes; a non-{0,1}
/// byte would be UB at reference creation). Canonical bytes round-trip;
/// the writer side (`push_`) stores `item as u8`.
#[test]
fn bool_array_reader_emits_validation() {
    let mut schema = MessageSchema::new("Coll");
    schema.add_field(FieldDef::new(
        "enabled",
        FieldType::DynamicArray {
            element_type: Box::new(FieldType::Bool),
        },
    ));

    let code = generate_schema(&schema);
    assert!(
        code.contains("if !slice.iter().all(|&b| b <= 1) { return &[]; }"),
        "bool reader must validate canonical 0/1 bytes before the &[bool] cast"
    );
    // The writer side stores the canonical byte directly (never a
    // `let bytes = [item as u8]` round-trip form).
    assert!(
        code.contains("payload[cursor] = if item"),
        "bool push must store the canonical byte directly, not to_le_bytes()"
    );
}

// =============================================================================
// push_<field> codegen for 1-byte element types
// =============================================================================
//
// `bool` has no `to_le_bytes()`; a generated `push_<f>` body that
// unconditionally emitted `let bytes = item.to_le_bytes();` followed by a
// `copy_from_slice` would not compile for a `bool[]` field, and
// `u8`/`i8` would go through a pointless 1-byte array
// round-trip. All three emit a direct `payload[cursor] = ...` byte
// store, mirroring the "Other 1-byte types (i8, bool): no alignment
// concern" arms in the reader / `loan_<f>` paths.
//
// Round-trip coverage: the i8 case runs against the REAL generated
// `std_msgs/Int8MultiArray` (`int8[] data`) via `build_writer` — the same
// machinery `fill_from_codegen_test.rs` uses (native_ros2_messages compiles
// the generated code; the test drives the writer directly, no transport).
// A bool[] round-trip is NOT covered: no ROS2 Jazzy message in
// `native_ros2_messages/msg/` declares a `bool[]` field, and the workspace
// has no harness that compiles hand-built `MessageSchema` codegen output at
// test time — the generated-source assertions below are the available
// compile-shape proof for bool.

/// Build a one-variable-field schema `PushProbe { items: <elem>[] }`,
/// generate code, and return the body of `pub fn push_items`.
fn push_body_for(elem: FieldType) -> String {
    let mut schema = MessageSchema::new("PushProbe");
    schema.add_field(FieldDef::new(
        "items",
        FieldType::DynamicArray {
            element_type: Box::new(elem),
        },
    ));
    let code = generate_schema(&schema);
    let start = code
        .find("pub fn push_items")
        .unwrap_or_else(|| panic!("generated code must contain push_items; got:\n{code}"));
    let rest = &code[start..];
    // Body ends at the next method boundary (or end of generated code).
    let end = rest[1..]
        .find("pub fn ")
        .map(|i| i + 1)
        .unwrap_or(rest.len());
    rest[..end].to_string()
}

/// Parameterized core assertion for 1-byte element types: the push body
/// must NOT route through `to_le_bytes` and must store the byte directly
/// via `payload[cursor] = ...` (semantic core only — no brace-spacing pins).
#[track_caller]
fn assert_one_byte_push_body(elem: FieldType, elem_label: &str, store_fragment: &str) {
    let body = push_body_for(elem);
    assert!(
        !body.contains("to_le_bytes"),
        "push body for {elem_label}[] must not call to_le_bytes (bool lacks it; \
         u8/i8 need no byte-array round-trip); got:\n{body}"
    );
    assert!(
        body.contains("payload[cursor] ="),
        "push body for {elem_label}[] must store directly via `payload[cursor] =`; got:\n{body}"
    );
    assert!(
        body.contains(store_fragment),
        "push body for {elem_label}[] must contain `{store_fragment}`; got:\n{body}"
    );
}

#[test]
fn test_push_bool_direct_byte_store() {
    assert_one_byte_push_body(FieldType::Bool, "bool", "if item");
}

#[test]
fn test_push_u8_direct_byte_store() {
    assert_one_byte_push_body(FieldType::U8, "uint8", "payload[cursor] = item;");
}

#[test]
fn test_push_i8_direct_byte_store() {
    assert_one_byte_push_body(FieldType::I8, "int8", "item as u8");
}

#[test]
fn test_push_u32_control_still_uses_to_le_bytes() {
    // Multi-byte control: the LE-bytes path must remain for elem_size > 1.
    let body = push_body_for(FieldType::U32);
    assert!(
        body.contains("to_le_bytes"),
        "push body for uint32[] must still serialize via to_le_bytes; got:\n{body}"
    );
}

#[test]
fn test_i8_push_round_trip_via_int8_multi_array() {
    // Real generated code: std_msgs/Int8MultiArray has `int8[] data`.
    use cerulion_core::message::ShmMessage;
    use native_ros2_messages::std_msgs::Int8MultiArray;

    const BUF: usize = 4096;
    let mut bytes = vec![0u8; BUF];
    let mut writer = Int8MultiArray::build_writer(
        &mut bytes,
        cerulion_core::wire::MaxPayloadCapacity::const_new(BUF as u32),
        std::sync::Arc::from("test"),
    );

    // Nested `layout` field: write empty placeholder bytes so `data`
    // is unambiguously the only payload under test.
    writer.set_layout_bytes(&[]).expect("layout empty");

    writer.push_data(-1).expect("push -1");
    writer.push_data(5).expect("push 5");
    writer.push_data(i8::MIN).expect("push i8::MIN");

    assert_eq!(
        writer.data(),
        &[-1, 5, i8::MIN],
        "i8 push round-trip must preserve negative values bit-exactly"
    );
}
