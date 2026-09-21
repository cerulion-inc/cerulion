// SPDX-License-Identifier: AGPL-3.0-only
//! Runtime wire-layout computation.
//!
//! The generated `<Name>Shm` / `<Name>FixedSection` structs get their
//! layout from rustc's `#[repr(C)]` algorithm at compile time. The rmw
//! bridge (`rmw_cerulion`) has no generated structs — it discovers
//! message types at RUNTIME through rosidl's introspection typesupport
//! and must flatten C messages into the EXACT same byte layout, or
//! native Cerulion nodes and ROS 2 nodes on the same topic would
//! misread each other under a matching schema hash.
//!
//! [`LayoutResolver`] re-implements the `#[repr(C)]` layout algorithm
//! over the schema IR:
//!
//! - field offset = `align_up(cursor, align_of(field))`
//! - struct align = max field align (min 1)
//! - struct size  = `align_up(end_of_last_field, struct_align)`
//! - `bool` is stored as `u8` (size 1, align 1) — the SHM storage type
//! - `StringFixed(n)` is `[u8; n]` (align 1)
//! - `FixedArray<T, N>` is `N * size(T)` with `align(T)`
//! - resolved-fixed `Nested` fields embed the target's computed layout
//!   (size INCLUDES the target's own trailing padding, exactly like a
//!   `#[repr(C)]` struct field of that type)
//!
//! The equivalence `computed == rustc` is pinned by tests in
//! `crates/native_ros2_messages/tests/layout_equivalence_test.rs` using
//! `mem::offset_of!` / `size_of` against the real generated structs.
//!
//! # Determinism
//!
//! Pure function of the schema set — same input, same layout, every
//! run, both sides of the FFI boundary (Principle #7).

use super::resolve::resolve_fixed_nested;
use super::schema::{FieldType, MessageSchema};
use std::collections::BTreeMap;

/// Layout of one fixed-section field.
#[derive(Debug, Clone, PartialEq)]
pub struct FieldLayout {
    /// Field name (schema order is preserved in [`WireLayout::fixed_fields`]).
    pub name: String,
    /// Byte offset from the start of the fixed section.
    pub offset: usize,
    /// Size in bytes (for nested: the target's full padded size).
    pub size: usize,
    /// Alignment requirement.
    pub align: usize,
    /// The schema field type (resolution flags included).
    pub field_type: FieldType,
}

/// Complete wire layout for one schema:
///
/// ```text
/// [WireHeader (32 B)]
/// [fixed section  (fixed_size bytes, fields at fixed_fields[i].offset)]
/// [offset table   (8 × variable_fields.len() bytes)]
/// [variable payload …]
/// ```
#[derive(Debug, Clone, PartialEq)]
pub struct WireLayout {
    /// Qualified schema name (`"pkg/Name"` or bare).
    pub qualified_name: String,
    /// The wire `schema_hash`: the recipe-3 layout-sensitive hash from
    /// [`MessageSchema::schema_hash`](crate::codegen::MessageSchema::schema_hash)
    /// (FNV-1a over qualified name + `wire_fixed_size` + each field's
    /// name/canonical type + recursively-folded fixed-nested target hashes).
    /// NOT a name-only hash — the layout delegates the canonical method so
    /// this value matches the generated type's `SCHEMA_HASH`.
    pub schema_hash: u64,
    /// Size of the fixed section in bytes (== `WIRE_FIXED_SIZE` of the
    /// generated type), including trailing padding. 0 for schemas whose
    /// fields are all variable.
    pub fixed_size: usize,
    /// Alignment of the fixed section (≤ 8 — the post-WireHeader payload
    /// pointer guarantee; see the static assert in codegen).
    pub fixed_align: usize,
    /// Fixed-section fields in declaration order with computed offsets.
    pub fixed_fields: Vec<FieldLayout>,
    /// Variable fields in declaration order — index i is offset table
    /// entry i. Carries the `FieldType` because the variable-payload
    /// ENCODING is element-type-dependent (cursor aligned to the
    /// element's ALIGNMENT before reserving, byte length = count ×
    /// element size) — names alone could not drive the rmw bridge's
    /// flattening.
    pub variable_fields: Vec<VariableFieldLayout>,
}

/// Layout info for one variable (offset-table) field.
#[derive(Debug, Clone, PartialEq)]
pub struct VariableFieldLayout {
    /// Field name (matches the rosidl introspection member name).
    pub name: String,
    /// The schema field type — drives the payload encoding (element
    /// size/alignment for typed arrays, UTF-8 for strings, embedded
    /// encoding for unresolved nested).
    pub field_type: FieldType,
}

impl WireLayout {
    /// True when every field lives in the fixed section — the schema is
    /// a fixed (loanable, fully zero-copy) schema.
    pub fn is_fixed(&self) -> bool {
        self.variable_fields.is_empty()
    }

    /// Byte offset of the offset table from the start of the payload
    /// region (== fixed_size).
    pub fn offset_table_offset(&self) -> usize {
        self.fixed_size
    }

    /// Byte length of the offset table — one [`OffsetEntry`](crate::wire::OffsetEntry)
    /// per variable field.
    pub fn offset_table_bytes(&self) -> usize {
        crate::wire::OffsetEntry::SIZE * self.variable_fields.len()
    }

    /// True when no frame of this layout fits the wire: its prefix — the
    /// 32-byte header, the fixed section and the offset table, i.e.
    /// `WireHeader::SIZE + data_floor()` — exceeds the `u32` `total_size`
    /// ([`crate::wire::frame_prefix_exceeds_wire`], read off this layout's
    /// own fixed size and offset-table length so the two cannot be paired
    /// wrongly).
    pub fn frame_prefix_exceeds_wire(&self) -> bool {
        crate::wire::frame_prefix_exceeds_wire(self.fixed_size, self.variable_fields.len())
    }

    /// The DATA FLOOR: the payload-relative offset where variable-field
    /// bytes may begin — the first byte past the offset table. A variable
    /// entry whose offset lies below it aliases the fixed section or the
    /// table itself, so `off >= data_floor()` is the placement rule EVERY
    /// reader of the offset table enforces before trusting an entry: the
    /// `FrameWalker`'s frame audit and `rmw_cerulion`'s forged loaned take
    /// (which would otherwise hand a C++ `std::vector` header/table bytes as
    /// its elements). ONE definition, so the two gates cannot drift.
    pub fn data_floor(&self) -> usize {
        self.offset_table_offset() + self.offset_table_bytes()
    }
}

/// Computes [`WireLayout`]s for a resolved schema set.
pub struct LayoutResolver {
    schemas: Vec<MessageSchema>,
    /// Qualified name → index into `schemas`.
    by_qualified: BTreeMap<String, usize>,
    /// Memoized per-schema `(size, align)` of the fixed REPRESENTATION
    /// (for fixed schemas: the whole struct; only valid when the schema
    /// is recursively fixed).
    size_align_memo: BTreeMap<usize, (usize, usize)>,
}

impl LayoutResolver {
    /// Build a resolver over `schemas`. Runs [`resolve_fixed_nested`]
    /// internally (idempotent if the caller already resolved) and
    /// returns its warnings alongside the resolver — callers must
    /// surface them loudly.
    pub fn new(mut schemas: Vec<MessageSchema>) -> (Self, Vec<String>) {
        let warnings = resolve_fixed_nested(&mut schemas);
        // Loudness is structural, not caller-discipline: emit every
        // warning here so dropping the returned Vec cannot silence the
        // inference (the loud-inference rule: a caller that drops
        // the Vec must not be able to silence it).
        for warning in &warnings {
            tracing::warn!(warning = %warning, "schema layout resolution warning");
        }
        let mut by_qualified = BTreeMap::new();
        for (idx, schema) in schemas.iter().enumerate() {
            by_qualified.insert(schema.qualified_name(), idx);
        }
        (
            Self {
                schemas,
                by_qualified,
                size_align_memo: BTreeMap::new(),
            },
            warnings,
        )
    }

    /// Compute the wire layout for the schema with this qualified name.
    /// Returns `None` for unknown schemas.
    pub fn layout_of(&mut self, qualified_name: &str) -> Option<WireLayout> {
        let idx = *self.by_qualified.get(qualified_name)?;
        Some(self.layout_of_idx(idx))
    }

    fn layout_of_idx(&mut self, idx: usize) -> WireLayout {
        let schema = self.schemas[idx].clone();
        let mut cursor = 0usize;
        let mut struct_align = 1usize;
        let mut fixed_fields = Vec::new();
        let mut variable_fields = Vec::new();

        for field in &schema.fields {
            if field.field_type.is_variable() {
                variable_fields.push(VariableFieldLayout {
                    name: field.name.clone(),
                    field_type: field.field_type.clone(),
                });
                continue;
            }
            let (size, align) = self.size_align_of(&field.field_type, idx);
            let offset = align_up(cursor, align);
            cursor = offset + size;
            struct_align = struct_align.max(align);
            fixed_fields.push(FieldLayout {
                name: field.name.clone(),
                offset,
                size,
                align,
                field_type: field.field_type.clone(),
            });
        }

        let fixed_size = if fixed_fields.is_empty() {
            // Matches the generated `_marker: [u8; 0]` ZST FixedSection.
            0
        } else {
            align_up(cursor, struct_align)
        };
        let fixed_align = if fixed_fields.is_empty() {
            1
        } else {
            struct_align
        };

        WireLayout {
            qualified_name: schema.qualified_name(),
            schema_hash: schema.schema_hash(),
            fixed_size,
            fixed_align,
            fixed_fields,
            variable_fields,
        }
    }

    /// `(size, align)` of one FIXED field type, mirroring the SHM storage
    /// types the generator emits.
    ///
    /// `parent_idx` provides the package context for resolving
    /// unqualified nested references (same lookup the resolver used —
    /// by construction a `fixed: Some(_)` reference resolves).
    fn size_align_of(&mut self, ft: &FieldType, parent_idx: usize) -> (usize, usize) {
        match ft {
            FieldType::Bool | FieldType::I8 | FieldType::U8 => (1, 1),
            FieldType::I16 | FieldType::U16 => (2, 2),
            FieldType::I32 | FieldType::U32 | FieldType::F32 => (4, 4),
            FieldType::I64 | FieldType::U64 | FieldType::F64 => (8, 8),
            FieldType::StringFixed(n) => (*n, 1),
            FieldType::FixedArray {
                element_type,
                length,
            } => {
                let (elem_size, elem_align) = self.size_align_of(element_type, parent_idx);
                // Element size includes the element's trailing padding, so
                // N × size matches Rust's array stride exactly. Checked:
                // a pathological length must fail loudly, not wrap into a
                // silently wrong layout.
                let size = elem_size.checked_mul(*length).unwrap_or_else(|| {
                    panic!(
                        "fixed-array size overflow ({elem_size} × {length}) in schema '{}'",
                        self.schemas[parent_idx].qualified_name()
                    )
                });
                (size, elem_align)
            }
            FieldType::Nested {
                schema_name,
                package,
                fixed,
            } => {
                debug_assert!(
                    fixed.is_some(),
                    "size_align_of called on an unresolved nested reference — \
                     fixed-section emission gates on resolution"
                );
                let target_idx = self.resolve_nested_idx(schema_name, package, parent_idx);
                self.fixed_struct_size_align(target_idx)
            }
            // Variable types never reach here: layout_of_idx routes them
            // to the offset table before calling size_align_of.
            FieldType::String | FieldType::Bytes | FieldType::DynamicArray { .. } => {
                unreachable!("variable field types have no fixed-section layout")
            }
        }
    }

    /// `(size, align)` of a recursively-fixed schema's full `#[repr(C)]`
    /// struct, INCLUDING trailing padding — memoized.
    fn fixed_struct_size_align(&mut self, idx: usize) -> (usize, usize) {
        if let Some(&cached) = self.size_align_memo.get(&idx) {
            return cached;
        }
        let layout = self.layout_of_idx(idx);
        let result = (layout.fixed_size, layout.fixed_align);
        self.size_align_memo.insert(idx, result);
        result
    }

    /// Resolve a nested reference to its schema index using the same
    /// precedence as `resolve_fixed_nested`'s lookup: qualified →
    /// same-package → bare `Header` → unambiguous bare.
    fn resolve_nested_idx(
        &self,
        schema_name: &str,
        package: &Option<String>,
        parent_idx: usize,
    ) -> usize {
        if let Some(pkg) = package {
            if let Some(&idx) = self.by_qualified.get(&format!("{pkg}/{schema_name}")) {
                return idx;
            }
        } else {
            // Same package as the parent.
            if let Some(parent_pkg) = &self.schemas[parent_idx].package {
                if let Some(&idx) = self
                    .by_qualified
                    .get(&format!("{parent_pkg}/{schema_name}"))
                {
                    return idx;
                }
            } else if let Some(&idx) = self.by_qualified.get(schema_name) {
                // Package-less parent: bare key.
                return idx;
            }
            // Bare Header → std_msgs (rosidl legacy special case).
            if schema_name == "Header" {
                if let Some(&idx) = self.by_qualified.get("std_msgs/Header") {
                    return idx;
                }
            }
            // Unambiguous bare fallback.
            let mut matches = self
                .by_qualified
                .iter()
                .filter(|(k, _)| k.rsplit('/').next() == Some(schema_name));
            if let (Some((_, &idx)), None) = (matches.next(), matches.next()) {
                return idx;
            }
        }
        // A `fixed: Some(_)` reference is minted (or CLEARED — the
        // rewriter resets stale flags on lookup failure / variable
        // targets) by the `resolve_fixed_nested` run inside
        // `LayoutResolver::new`, using this same lookup precedence over
        // this same owned schema set. Reaching here would require the
        // set to change between resolution and layout, which ownership
        // prevents.
        unreachable!("resolved nested reference '{schema_name}' not found in schema set")
    }
}

#[inline]
fn align_up(value: usize, align: usize) -> usize {
    debug_assert!(align.is_power_of_two(), "alignment must be a power of two");
    (value + align - 1) & !(align - 1)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::codegen::schema::FieldDef;

    fn f64_field(name: &str) -> FieldDef {
        FieldDef::new(name, FieldType::F64)
    }

    /// Recipe-3 hash of the schema named `qname` after resolving
    /// `schemas` (mirrors `LayoutResolver::new`'s internal
    /// [`resolve_fixed_nested`] run). A [`WireLayout`] delegates this
    /// hash, so the layout's `schema_hash` equals it. Pinned
    /// recipe-agnostically: recipe 3 is layout + qualified name + nested
    /// fold, NOT a name-only `fnv1a` literal.
    fn resolved_hash(mut schemas: Vec<MessageSchema>, qname: &str) -> u64 {
        crate::codegen::resolve_fixed_nested(&mut schemas);
        schemas
            .iter()
            .find(|m| m.qualified_name() == qname)
            .expect("schema present in set")
            .schema_hash()
    }

    fn schema_set() -> Vec<MessageSchema> {
        let mut point = MessageSchema::new_in_package("Point", "geometry_msgs");
        for f in ["x", "y", "z"] {
            point.add_field(f64_field(f));
        }
        let mut quat = MessageSchema::new_in_package("Quaternion", "geometry_msgs");
        for f in ["x", "y", "z", "w"] {
            quat.add_field(f64_field(f));
        }
        let mut pose = MessageSchema::new_in_package("Pose", "geometry_msgs");
        pose.add_field(FieldDef::new(
            "position",
            FieldType::Nested {
                schema_name: "Point".into(),
                package: None,
                fixed: None,
            },
        ));
        pose.add_field(FieldDef::new(
            "orientation",
            FieldType::Nested {
                schema_name: "Quaternion".into(),
                package: None,
                fixed: None,
            },
        ));
        vec![point, quat, pose]
    }

    #[test]
    fn pose_layout_matches_repr_c() {
        let (mut resolver, warnings) = LayoutResolver::new(schema_set());
        assert!(warnings.is_empty(), "{warnings:?}");

        let layout = resolver.layout_of("geometry_msgs/Pose").expect("pose");
        assert!(layout.is_fixed());
        assert_eq!(layout.fixed_size, 56); // 24 (Point) + 32 (Quaternion)
        assert_eq!(layout.fixed_align, 8);
        assert_eq!(layout.fixed_fields.len(), 2);
        assert_eq!(layout.fixed_fields[0].offset, 0);
        assert_eq!(layout.fixed_fields[0].size, 24);
        assert_eq!(layout.fixed_fields[1].offset, 24);
        assert_eq!(layout.fixed_fields[1].size, 32);
        assert_eq!(
            layout.schema_hash,
            resolved_hash(schema_set(), "geometry_msgs/Pose")
        );
    }

    #[test]
    fn padding_between_mixed_alignment_fields() {
        // { u8 a; u32 b; u8 c; } → a@0, pad 3, b@4, c@8, size 12 (align 4).
        let mut s = MessageSchema::new_in_package("Mixed", "test_msgs");
        s.add_field(FieldDef::new("a", FieldType::U8));
        s.add_field(FieldDef::new("b", FieldType::U32));
        s.add_field(FieldDef::new("c", FieldType::U8));

        let (mut resolver, _) = LayoutResolver::new(vec![s]);
        let layout = resolver.layout_of("test_msgs/Mixed").expect("mixed");
        assert_eq!(
            layout
                .fixed_fields
                .iter()
                .map(|f| f.offset)
                .collect::<Vec<_>>(),
            vec![0, 4, 8]
        );
        assert_eq!(layout.fixed_size, 12);
        assert_eq!(layout.fixed_align, 4);
    }

    #[test]
    fn variable_fields_skip_fixed_section_in_declaration_order() {
        // Image-like: { u32 height; u32 width; string encoding; u8
        // is_bigendian; u32 step; bytes data } → fixed {height@0,
        // width@4, is_bigendian@8, step@12} size 16; variable
        // [encoding, data].
        let mut s = MessageSchema::new_in_package("Img", "test_msgs");
        s.add_field(FieldDef::new("height", FieldType::U32));
        s.add_field(FieldDef::new("width", FieldType::U32));
        s.add_field(FieldDef::new("encoding", FieldType::String));
        s.add_field(FieldDef::new("is_bigendian", FieldType::U8));
        s.add_field(FieldDef::new("step", FieldType::U32));
        s.add_field(FieldDef::new("data", FieldType::Bytes));

        let (mut resolver, _) = LayoutResolver::new(vec![s]);
        let layout = resolver.layout_of("test_msgs/Img").expect("img");
        assert!(!layout.is_fixed());
        assert_eq!(
            layout
                .variable_fields
                .iter()
                .map(|f| f.name.as_str())
                .collect::<Vec<_>>(),
            vec!["encoding", "data"]
        );
        // The element type rides along — the rmw bridge needs it to
        // encode the variable payload.
        assert_eq!(layout.variable_fields[1].field_type, FieldType::Bytes);
        assert_eq!(
            layout
                .fixed_fields
                .iter()
                .map(|f| (f.name.as_str(), f.offset))
                .collect::<Vec<_>>(),
            vec![
                ("height", 0),
                ("width", 4),
                ("is_bigendian", 8),
                ("step", 12)
            ]
        );
        assert_eq!(layout.fixed_size, 16);
        assert_eq!(layout.offset_table_offset(), 16);
        assert_eq!(layout.offset_table_bytes(), 16);
    }

    #[test]
    fn all_variable_schema_has_zst_fixed_section() {
        let mut s = MessageSchema::new_in_package("AllVar", "test_msgs");
        s.add_field(FieldDef::new("a", FieldType::String));
        s.add_field(FieldDef::new("b", FieldType::Bytes));

        let (mut resolver, _) = LayoutResolver::new(vec![s]);
        let layout = resolver.layout_of("test_msgs/AllVar").expect("allvar");
        assert_eq!(layout.fixed_size, 0);
        assert_eq!(layout.fixed_align, 1);
        assert!(layout.fixed_fields.is_empty());
    }

    #[test]
    fn bool_and_fixed_arrays_use_shm_storage_sizes() {
        // { bool flag; u8[16] id; f64[4] vals } → flag@0 (u8), id@1,
        // pad to 24? id ends at 17; vals align 8 → @24; size 56.
        let mut s = MessageSchema::new_in_package("Arr", "test_msgs");
        s.add_field(FieldDef::new("flag", FieldType::Bool));
        s.add_field(FieldDef::new(
            "id",
            FieldType::FixedArray {
                element_type: Box::new(FieldType::U8),
                length: 16,
            },
        ));
        s.add_field(FieldDef::new(
            "vals",
            FieldType::FixedArray {
                element_type: Box::new(FieldType::F64),
                length: 4,
            },
        ));

        let (mut resolver, _) = LayoutResolver::new(vec![s]);
        let layout = resolver.layout_of("test_msgs/Arr").expect("arr");
        assert_eq!(layout.fixed_fields[0].offset, 0);
        assert_eq!(layout.fixed_fields[0].size, 1); // bool as u8
        assert_eq!(layout.fixed_fields[1].offset, 1);
        assert_eq!(layout.fixed_fields[1].size, 16);
        assert_eq!(layout.fixed_fields[2].offset, 24); // aligned to 8
        assert_eq!(layout.fixed_fields[2].size, 32);
        assert_eq!(layout.fixed_size, 56);
    }

    /// Array-of-fixed-nested stride: `[Point; 4]` = 4 × 24 (the element
    /// size INCLUDES its trailing padding, matching Rust array stride).
    #[test]
    fn fixed_array_of_nested_uses_padded_stride() {
        let mut s = MessageSchema::new_in_package("Corners", "test_msgs");
        s.add_field(FieldDef::new(
            "corners",
            FieldType::FixedArray {
                element_type: Box::new(FieldType::Nested {
                    schema_name: "Point".into(),
                    package: Some("geometry_msgs".into()),
                    fixed: None,
                }),
                length: 4,
            },
        ));
        s.add_field(FieldDef::new("tag", FieldType::U8));

        let mut set = schema_set();
        set.push(s);
        let (mut resolver, warnings) = LayoutResolver::new(set);
        assert!(warnings.is_empty(), "{warnings:?}");

        let layout = resolver.layout_of("test_msgs/Corners").expect("corners");
        assert!(layout.is_fixed());
        assert_eq!(layout.fixed_fields[0].offset, 0);
        assert_eq!(layout.fixed_fields[0].size, 96); // 4 × 24
        assert_eq!(layout.fixed_fields[0].align, 8);
        assert_eq!(layout.fixed_fields[1].offset, 96); // tag right after
        assert_eq!(layout.fixed_size, 104); // padded to align 8
    }

    /// Package-less (workspace YAML) schemas: bare qualified names, bare
    /// nested resolution, bare-name hashing.
    #[test]
    fn bare_schemas_lay_out_with_bare_names() {
        let mut inner = MessageSchema::new("Inner");
        inner.add_field(FieldDef::new("v", FieldType::F64));
        let mut outer = MessageSchema::new("Outer");
        outer.add_field(FieldDef::new(
            "inner",
            FieldType::Nested {
                schema_name: "Inner".into(),
                package: None,
                fixed: None,
            },
        ));
        outer.add_field(FieldDef::new("flag", FieldType::Bool));

        let schemas = vec![inner, outer];
        let (mut resolver, warnings) = LayoutResolver::new(schemas.clone());
        assert!(warnings.is_empty(), "{warnings:?}");

        let layout = resolver.layout_of("Outer").expect("outer");
        assert_eq!(layout.qualified_name, "Outer");
        assert_eq!(layout.schema_hash, resolved_hash(schemas, "Outer"));
        assert!(layout.is_fixed());
        assert_eq!(layout.fixed_fields[0].offset, 0);
        assert_eq!(layout.fixed_fields[0].size, 8);
        assert_eq!(layout.fixed_fields[1].offset, 8); // bool-as-u8
        assert_eq!(layout.fixed_size, 16); // padded to align 8
    }

    #[test]
    fn unknown_schema_returns_none() {
        let (mut resolver, _) = LayoutResolver::new(schema_set());
        assert!(resolver.layout_of("nope/Nope").is_none());
    }
}
