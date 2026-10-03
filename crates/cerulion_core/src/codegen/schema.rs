// SPDX-License-Identifier: AGPL-3.0-only
//! Schema intermediate representation.
//!
//! Parsed schemas are converted to this IR before code generation.

use crate::wire::Fnv1aHasher;

/// Parsed message schema ready for code generation.
#[derive(Debug, Clone)]
pub struct MessageSchema {
    /// Schema name (e.g., "Image", "JointState")
    pub name: String,

    /// Package the schema belongs to (e.g., "sensor_msgs" for ROS2 .msg
    /// schemas). `None` for package-less schemas (workspace YAML schemas).
    ///
    /// The package participates in `schema_hash` so that two
    /// packages defining the same bare message name (e.g. `shape_msgs/Mesh`
    /// vs a future `moveit_msgs/Mesh`) produce DIFFERENT wire hashes.
    /// `WireHeader::validate_schema` would otherwise silently accept
    /// wrong-type frames across packages.
    pub package: Option<String>,

    /// Optional description
    pub description: Option<String>,

    /// Fields in declaration order
    pub fields: Vec<FieldDef>,
}

impl MessageSchema {
    /// Create a new package-less schema with the given name.
    ///
    /// The hash input is the bare name — used for workspace YAML schemas,
    /// which live in a single flat namespace.
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            package: None,
            description: None,
            fields: Vec::new(),
        }
    }

    /// Create a new schema scoped to a package (ROS2 .msg pipeline).
    ///
    /// The package becomes part of the qualified name (`"pkg/Name"`), which
    /// feeds the layout-sensitive `schema_hash`, so identically named
    /// messages in different packages never collide on the wire.
    pub fn new_in_package(name: impl Into<String>, package: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            package: Some(package.into()),
            description: None,
            fields: Vec::new(),
        }
    }

    /// Qualified name used as the hash input: `"pkg/Name"` or bare `"Name"`.
    pub fn qualified_name(&self) -> String {
        match &self.package {
            Some(pkg) => format!("{}/{}", pkg, self.name),
            None => self.name.clone(),
        }
    }

    /// Layout-sensitive, fully-qualified schema hash — **wire-compat
    /// contract** (layout sensitivity + package qualification
    /// + recursive nested layout).
    ///
    /// This is the canonical (and only) implementation of the schema hash
    /// that lands in `WireHeader.schema_hash` and the generated
    /// `<NAME>_SCHEMA_HASH` / `ShmMessage::SCHEMA_HASH` constants. Any
    /// change to this recipe is a wire-format break: publishers and
    /// subscribers built from different recipes will reject each other's
    /// frames as `SchemaMismatch`.
    ///
    /// # Recipe (recipe id 3; recipe 2 was layout + bare name; recipe 1 was
    /// the earlier name-only `fnv1a(name)`) — see `trace/bag.rs::HASH_RECIPE`.
    ///
    /// FNV-1a 64 (offset `0xcbf29ce484222325`, prime `0x100000001b3`) over
    /// a byte stream built as:
    ///
    /// 1. `(qualified_name().len() as u64).to_le_bytes()` then the
    ///    **qualified** name bytes (`"pkg/Name"`, or bare `"Name"` for
    ///    package-less workspace schemas)
    /// 2. `(wire_fixed_size() as u64).to_le_bytes()` — fixed 8 bytes, no
    ///    length prefix needed
    /// 3. For each field **in declaration order**:
    ///    - `(field_name.len() as u64).to_le_bytes()` then the name bytes
    ///    - `(canonical_type.len() as u64).to_le_bytes()` then the bytes
    ///      of [`FieldType::canonical_str`] (`"pkg/Name"` for packaged
    ///      nested refs)
    ///    - **for a fixed-resolved nested field only**, the target schema's
    ///      own 8-byte `schema_hash` (LE), carried in
    ///      [`NestedFixedInfo::target_hash`] by `resolve_fixed_nested`.
    ///
    /// Length-prefix framing makes the encoding injective by construction:
    /// two schemas whose concatenated name/type bytes are identical but
    /// split differently hash differently (deliberate review-driven
    /// design).
    ///
    /// Descriptions / doc strings are **not** hashed — they carry no
    /// layout meaning.
    ///
    /// # Nested coverage
    ///
    /// A **fixed-resolved** nested field is inlined as `<Target>Shm` in the
    /// parent's fixed section, so a layout change *inside* the target shifts
    /// the parent's bytes. Recipe 3 folds the target's full `schema_hash`
    /// (step 3, third item) into the parent, so ANY change to a fixed nested
    /// target (size, reorder, retype, rename, add/remove) bumps the parent
    /// hash — no silent corruption. A **variable** nested field lives in the
    /// offset table (self-describing, length-prefixed on the wire); it
    /// contributes only its package-qualified name via `canonical_str()`,
    /// matching its self-describing wire shape.
    pub fn schema_hash(&self) -> u64 {
        self.hash_with_fixed_size(self.wire_fixed_size())
    }

    /// Fallible twin of [`schema_hash`](Self::schema_hash): the SAME recipe
    /// (one private body, `hash_with_fixed_size`) over
    /// [`checked_wire_fixed_size`](Self::checked_wire_fixed_size), returning
    /// that method's field-naming error instead of panicking on a hostile
    /// `FixedArray` length. For consumers that must DISPLAY or INDEX a
    /// definition they could not resolve (a `schema info` / `schema list`
    /// fallback, a recorder's served-schema corpus) — a schema whose fixed
    /// section cannot be sized has no wire hash, and saying so loudly beats
    /// a panic or a fabricated number.
    pub fn checked_schema_hash(&self) -> Result<u64, String> {
        Ok(self.hash_with_fixed_size(self.checked_wire_fixed_size()?))
    }

    /// The recipe-3 fold over an already-computed fixed-section size — the
    /// ONE body behind both hash entry points, so the panicking and the
    /// fallible twin can never drift.
    fn hash_with_fixed_size(&self, wire_fixed_size: usize) -> u64 {
        // FNV-1a streaming accumulator. Length prefixes (`u64` LE) then the
        // corresponding bytes, in order, incrementally with zero heap
        // allocation. The value is byte-for-byte identical to
        // `fnv1a_hash(&assembled_bytes)` because FNV-1a is a pure left-fold
        // over the byte sequence; only the order matters, not the chunking.
        // Pinned by `native_ros2_messages/tests/schema_hash_pin_test.rs`.
        let qname = self.qualified_name();
        let mut hash = Fnv1aHasher::new();
        hash.feed(&(qname.len() as u64).to_le_bytes());
        hash.feed(qname.as_bytes());
        hash.feed(&(wire_fixed_size as u64).to_le_bytes());
        for field in &self.fields {
            hash.feed(&(field.name.len() as u64).to_le_bytes());
            hash.feed(field.name.as_bytes());
            let ty = field.field_type.canonical_str();
            hash.feed(&(ty.len() as u64).to_le_bytes());
            hash.feed(ty.as_bytes());
            // Recipe 3: a fixed-resolved nested field is inlined in
            // the fixed section, so the parent hash must depend on the
            // target's full layout. `resolve_fixed_nested` stamps the
            // target's own `schema_hash` into `NestedFixedInfo::target_hash`
            // (bottom-up, so the target is fully resolved first). This must
            // also reach a fixed nested embedded inside a `FixedArray`
            // (e.g. `Point[4]`), which is inlined too — `canonical_str`
            // only carries the element NAME, not its layout, so without
            // this fold a same-size reorder inside the array element would
            // not bump the parent hash (silent wire skew).
            fold_fixed_nested_target_hash(&field.field_type, &mut hash);
        }
        hash.finish()
    }

    /// Size in bytes of the wire fixed section for this schema, computed
    /// from the IR with the same `#[repr(C)]` layout rules rustc applies
    /// to the generated `<Name>Shm` / `<Name>FixedSection` struct:
    ///
    /// - Only definitely-fixed fields participate (primitives,
    ///   `StringFixed`, `FixedArray` of those, and **fixed-resolved
    ///   `Nested`** fields inlined as `<Target>Shm`). Variable
    ///   `String` / `Bytes` / `DynamicArray` / unresolved-`Nested` fields
    ///   live in the offset table + variable payload and contribute nothing.
    /// - Each field is placed at the next offset aligned to its alignment
    ///   (`bool` is stored as `u8`: size 1, align 1; a fixed nested uses its
    ///   target's `align_of::<<Target>Shm>()`, carried in `NestedFixedInfo`).
    /// - The total is rounded up to the largest field alignment
    ///   (trailing padding), matching `size_of::<FixedSection>()`.
    /// - No fixed fields → 0 (matches the `_marker: [u8; 0]` overlay).
    ///
    /// # Panics
    ///
    /// Panics at codegen time (with a message naming the offending field)
    /// if a hostile `FixedArray` length overflows the size arithmetic —
    /// all arithmetic here is checked, never silently wrapping. CLI
    /// surfaces pre-validate `FixedArray` lengths so user input cannot
    /// reach this panic.
    pub fn wire_fixed_size(&self) -> usize {
        match self.checked_wire_fixed_size() {
            Ok(size) => size,
            Err(e) => panic!("{e}"),
        }
    }

    /// Fallible twin of [`wire_fixed_size`](Self::wire_fixed_size): the same
    /// checked `#[repr(C)]` layout arithmetic, returning the field-naming
    /// error message instead of panicking on a hostile `FixedArray` length.
    ///
    /// Exists for consumers that fold UNTRUSTED schema sets before
    /// materializing sizes/hashes — the workspace `.msg` store arrives over
    /// the wire (`ros attach`'s acquisition ladder) and workspace
    /// YAML is hand-edited, so a pre-flight `Err` lets those callers SKIP the
    /// hostile schema loudly instead of crashing the verb (a crash class
    /// closed earlier for the existence probe and here for the
    /// size/hash materializers). [`schema_hash`](Self::schema_hash) feeds
    /// `wire_fixed_size`, so a schema this returns `Ok` for hashes without
    /// panicking too.
    ///
    /// NOTE: this checks the schema AS DECLARED (plus any already-stamped
    /// fixed-nested resolution). A set whose members are individually sizable
    /// can still compose an overflow through fixed-nested inlining
    /// (`resolve_fixed_nested` inlines targets into parents), so callers
    /// folding a set re-check AFTER resolution as well.
    pub fn checked_wire_fixed_size(&self) -> Result<usize, String> {
        let mut offset = 0usize;
        let mut max_align = 1usize;
        for field in &self.fields {
            if !field.field_type.is_definitely_fixed() {
                continue;
            }
            let align = field.field_type.alignment();
            let size = checked_fixed_field_size(&self.name, &field.name, &field.field_type)?;
            offset = align_up_checked(offset, align, &self.name, &field.name)?;
            offset = offset.checked_add(size).ok_or_else(|| {
                format!(
                    "schema `{}` field `{}`: fixed-section size overflows usize \
                     (offset {} + field size {})",
                    self.name, field.name, offset, size
                )
            })?;
            max_align = max_align.max(align);
        }
        align_up_checked(offset, max_align, &self.name, "<trailing padding>")
    }

    /// The variable fields a frame of this schema carries under EVERY
    /// resolution ([`FieldType::is_definitely_variable`]) — the lower bound
    /// on [`variable_field_count`](Self::variable_field_count) a
    /// DECLARED-moment wire-size check may rely on: a refusal computed from
    /// it is never overturned by resolution (inlining a nested reference can
    /// only add fixed bytes and remove entries this count never held), while
    /// counting a not-yet-resolved reference could refuse a schema whose
    /// resolved frame fits. On a resolved schema the two counts differ only
    /// by references that resolved VARIABLE (or not at all).
    pub fn variable_field_count_floor(&self) -> usize {
        self.fields
            .iter()
            .filter(|f| f.field_type.is_definitely_variable())
            .count()
    }

    /// Maximum field alignment of this schema's fixed section, i.e.
    /// `align_of::<<Name>FixedSection>()` under `#[repr(C)]`. Mirrors the
    /// `max_align` fold in [`wire_fixed_size`](Self::wire_fixed_size) so a
    /// fixed-resolved nested reference embedding `<Name>Shm` can report the
    /// correct alignment without rustc. `1` when there are no fixed fields
    /// (matches the empty `_marker: [u8; 0]` overlay).
    ///
    /// `resolve_fixed_nested` calls this on a fully-resolved target schema
    /// (bottom-up), so nested-of-nested alignments are already populated.
    pub fn wire_alignment(&self) -> usize {
        let mut max_align = 1usize;
        for field in &self.fields {
            if field.field_type.is_definitely_fixed() {
                max_align = max_align.max(field.field_type.alignment());
            }
        }
        max_align
    }

    /// Add a field to the schema.
    pub fn add_field(&mut self, field: FieldDef) {
        self.fields.push(field);
    }

    /// Returns true if this schema has any variable-length fields.
    ///
    /// Note: Nested types are conservatively considered variable since they
    /// might contain strings/dynamic arrays.
    pub fn has_variable_fields(&self) -> bool {
        self.fields.iter().any(|f| f.field_type.is_variable())
    }

    /// Returns true if all fields are definitely fixed-size (primitives only).
    pub fn is_definitely_fixed(&self) -> bool {
        self.fields
            .iter()
            .all(|f| f.field_type.is_definitely_fixed())
    }

    /// Count of variable-length fields — one per field the schema classifies
    /// VARIABLE right now ([`FieldType::is_variable`]: `String` / `Bytes` /
    /// `DynamicArray`, a `FixedArray` of those, and a `Nested` reference not
    /// (yet) resolved fixed). This is the frame's offset-table LENGTH: what
    /// the generator stamps into `offset_table_count`, what the layout walker
    /// materializes as `variable_fields`, and what the wire ceiling counts
    /// (`frame_prefix_exceeds_wire`) on a RESOLVED schema — on a DECLARED one
    /// it is an upper bound, because resolution may still inline a nested
    /// reference (see [`variable_field_count_floor`](Self::variable_field_count_floor)).
    pub fn variable_field_count(&self) -> usize {
        self.fields
            .iter()
            .filter(|f| f.field_type.is_variable())
            .count()
    }
}

/// Fold the recursive `target_hash` of every fixed-resolved nested type
/// reachable through a field type into the schema hash (recipe 3).
///
/// A fixed nested is inlined into the parent's fixed section — directly
/// (`Nested`) or as array elements (`FixedArray` of `Nested`, possibly
/// nested arrays). `canonical_str` carries only the type NAME, so a
/// layout change inside the inlined target that preserves its name and
/// size (e.g. a same-type field reorder) would otherwise leave the parent
/// hash unchanged — silent wire skew. Folding the target's own
/// `schema_hash` closes that gap for both shapes. `DynamicArray` is NOT
/// folded: it lives in the offset table (variable, `fixed: None`) and is
/// self-describing on the wire.
fn fold_fixed_nested_target_hash(ft: &FieldType, hash: &mut Fnv1aHasher) {
    match ft {
        FieldType::Nested {
            fixed: Some(info), ..
        } => hash.feed(&info.target_hash.to_le_bytes()),
        FieldType::FixedArray { element_type, .. } => {
            fold_fixed_nested_target_hash(element_type, hash)
        }
        _ => {}
    }
}

/// Checked size of a definitely-fixed field. Returns the field-naming
/// error message on overflow instead of silently wrapping (hardening:
/// hostile `FixedArray` lengths must not corrupt
/// `wire_fixed_size()`, which panics with this message; the fallible
/// `checked_wire_fixed_size` surfaces it as an `Err` instead).
fn checked_fixed_field_size(
    schema_name: &str,
    field_name: &str,
    ft: &FieldType,
) -> Result<usize, String> {
    match ft {
        FieldType::FixedArray {
            element_type,
            length,
        } => {
            let elem = checked_fixed_field_size(schema_name, field_name, element_type)?;
            elem.checked_mul(*length).ok_or_else(|| {
                format!(
                    "schema `{schema_name}` field `{field_name}`: FixedArray size overflows \
                     usize (element size {elem} × length {length})"
                )
            })
        }
        // Callers gate on is_definitely_fixed(), so fixed_size() is Some
        // for every remaining variant (primitives, StringFixed).
        _ => ft.fixed_size().ok_or_else(|| {
            format!(
                "schema `{schema_name}` field `{field_name}`: expected a definitely-fixed \
                 field type, got {ft:?}"
            )
        }),
    }
}

/// Round `offset` up to the next multiple of `align` with checked
/// arithmetic, returning a field-naming error message on overflow (the
/// panicking `wire_fixed_size` wrapper turns it into the historical panic).
fn align_up_checked(
    offset: usize,
    align: usize,
    schema_name: &str,
    field_name: &str,
) -> Result<usize, String> {
    debug_assert!(align.is_power_of_two(), "alignments are powers of two");
    offset
        .checked_add(align - 1)
        .map(|v| v & !(align - 1))
        .ok_or_else(|| {
            format!(
                "schema `{schema_name}` field `{field_name}`: fixed-section alignment \
                 padding overflows usize (offset {offset}, align {align})"
            )
        })
}

/// Field definition within a schema.
#[derive(Debug, Clone)]
pub struct FieldDef {
    /// Field name
    pub name: String,

    /// Field type
    pub field_type: FieldType,

    /// Optional description
    pub description: Option<String>,

    /// Scalar default value from the .msg declaration (`float64 w 1`,
    /// ROS2 IDL field defaults). Honored in the generated
    /// `Default` impls so e.g. `QuaternionShm::default()` is the identity
    /// quaternion (w=1), matching rosidl semantics. Only captured for
    /// primitive numeric/bool field types — string/array defaults are
    /// not supported and are dropped at parse time.
    ///
    /// NOTE: defaults affect `Default` construction (snapshots,
    /// `<Name>Shm::default()` composition) only. `loan_proxy` still
    /// zero-initializes the fixed section — the SHM hot path stays a
    /// straight memset.
    pub default_literal: Option<DefaultLiteral>,
}

/// A validated scalar default value.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DefaultLiteral {
    Bool(bool),
    /// All integer field types; range errors surface as loud compile
    /// errors in the generated code (literal out of range for the typed
    /// field).
    Int(i128),
    Float(f64),
}

impl DefaultLiteral {
    /// True when the default equals the type's zero-default — emitting a
    /// manual `Default` impl for it would be exactly what `derive`
    /// produces (clippy `derivable_impls`).
    pub fn is_zero(&self) -> bool {
        match self {
            Self::Bool(b) => !*b,
            Self::Int(i) => *i == 0,
            Self::Float(f) => *f == 0.0,
        }
    }

    /// Rust literal for a snapshot-typed field (`bool` stays `bool`).
    pub fn snapshot_expr(&self) -> String {
        match self {
            Self::Bool(b) => b.to_string(),
            Self::Int(i) => i.to_string(),
            // `{:?}` keeps a decimal point (`1.0`, not `1`) so the
            // literal is float-typed in the struct-init position.
            Self::Float(f) => format!("{:?}", f),
        }
    }

    /// Rust literal for an SHM-typed field (`bool` stored as `u8`).
    pub fn shm_expr(&self) -> String {
        match self {
            Self::Bool(b) => if *b { "1u8" } else { "0u8" }.to_string(),
            Self::Int(i) => i.to_string(),
            Self::Float(f) => format!("{:?}", f),
        }
    }
}

impl FieldDef {
    /// Create a new field definition.
    pub fn new(name: impl Into<String>, field_type: FieldType) -> Self {
        Self {
            name: name.into(),
            field_type,
            description: None,
            default_literal: None,
        }
    }

    /// Set a scalar default value.
    pub fn with_default(mut self, default: DefaultLiteral) -> Self {
        self.default_literal = Some(default);
        self
    }
}

/// Field types supported in schemas.
///
/// Serializes (serde) externally tagged (`"U32"`, `{"StringFixed":16}`,
/// `{"DynamicArray":{"element_type":"F64"}}`, ...) — the shape
/// [`WireLayout::to_json`](crate::codegen::layout::WireLayout::to_json)
/// emits for language bindings.
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum FieldType {
    // Primitives
    Bool,
    I8,
    U8,
    I16,
    U16,
    I32,
    U32,
    I64,
    U64,
    F32,
    F64,

    // Variable-length types
    String,
    Bytes,

    // Fixed-size string (stored inline)
    StringFixed(usize),

    // Arrays
    FixedArray {
        element_type: Box<FieldType>,
        length: usize,
    },
    DynamicArray {
        element_type: Box<FieldType>,
    },

    // Nested message type
    Nested {
        schema_name: String,
        /// Source package when the reference was qualified
        /// (`geometry_msgs/Pose` → `Some("geometry_msgs")`). `None` for
        /// unqualified references (same-package in ROS2 .msg semantics, or
        /// flat-namespace YAML schemas). Preserved so cross-package
        /// resolution never guesses on bare-name collisions.
        package: Option<String>,
        /// Resolution result from
        /// [`resolve_fixed_nested`](crate::codegen::resolve_fixed_nested).
        /// `Some(info)` when the target schema is RECURSIVELY fixed-size —
        /// the field then lives inline in the parent's fixed section as
        /// `<Target>Shm` and is zero-copy.
        /// `None` (the parser default) keeps the conservative
        /// classification: variable, opaque-bytes accessors.
        fixed: Option<NestedFixedInfo>,
    },
}

/// Resolution metadata for a [`FieldType::Nested`] reference whose target
/// schema is recursively fixed-size. Stamped by
/// [`resolve_fixed_nested`](crate::codegen::resolve_fixed_nested), which
/// resolves targets **bottom-up** so every value here reflects the target's
/// fully-resolved layout (nested-of-nested already populated).
#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct NestedFixedInfo {
    /// True when the target schema (transitively) contains a
    /// `FixedArray<_, N>` with `N > 32`. Parents must know this because
    /// the generated types skip the `Debug` derive (and snapshots skip
    /// serde) for large-array schemas, and a field of such a type poisons
    /// the embedding parent's `Debug`/serde derives transitively. NOTE:
    /// `Default` is NOT poisoned transitively — the nested type carries
    /// its own manual `Default` impl, so the parent can still derive it
    /// (the whole point of the direct-vs-transitive split in the
    /// generator's derive matrix).
    pub has_large_array: bool,

    /// Size of the inlined `<Target>Shm` in the parent's fixed section,
    /// i.e. the target's [`MessageSchema::wire_fixed_size`]
    /// (== `size_of::<<Target>Shm>()` under `#[repr(C)]`). Lets
    /// [`FieldType::fixed_size`] report a real size for the field without a
    /// schema registry, so the parent's `wire_fixed_size()` stays equal to
    /// `size_of::<<Parent>FixedSection>()` (recipe 3).
    pub fixed_size: usize,

    /// Alignment of the inlined `<Target>Shm`, i.e. the target's
    /// [`MessageSchema::wire_alignment`] (== `align_of::<<Target>Shm>()`,
    /// ≤ 8 by the generator's static assert). Feeds the parent's
    /// align-up arithmetic in `wire_fixed_size()`.
    pub alignment: usize,

    /// The target schema's own [`MessageSchema::schema_hash`] (recipe 3).
    /// Folded into the parent's hash so ANY layout change inside the
    /// inlined target (size, reorder, retype, rename, add/remove) bumps the
    /// parent's hash — closing the silent-corruption gap that an inline
    /// fixed nested would otherwise open (a variable nested is
    /// offset-table-self-describing and needs no such fold).
    pub target_hash: u64,
}

impl FieldType {
    /// Canonical type-string rendering for the schema-hash recipe.
    ///
    /// Explicit match-based rendering (NEVER `Debug` formatting — `Debug`
    /// output is not a stability contract). The strings deliberately match
    /// the [`FieldType::parse`] input syntax so the rendering is stable,
    /// human-readable, and round-trippable:
    ///
    /// - primitives: `bool`, `int8`, `uint8`, ..., `float64`
    /// - `String` → `string`, `Bytes` → `bytes`
    /// - `StringFixed(n)` → `string_fixed[n]`
    /// - `FixedArray { e, n }` → `<e>[n]` (recursive)
    /// - `DynamicArray { e }` → `<e>[]` (recursive)
    /// - `Nested { name, package }` → `"pkg/Name"` when packaged, bare
    ///   `"Name"` otherwise (package-qualified so two packages'
    ///   same-named nested refs render — and hash — distinctly)
    ///
    /// Changing any rendering here changes every schema hash that uses the
    /// affected variant — treat as a wire-format break.
    pub fn canonical_str(&self) -> String {
        match self {
            Self::Bool => "bool".to_string(),
            Self::I8 => "int8".to_string(),
            Self::U8 => "uint8".to_string(),
            Self::I16 => "int16".to_string(),
            Self::U16 => "uint16".to_string(),
            Self::I32 => "int32".to_string(),
            Self::U32 => "uint32".to_string(),
            Self::I64 => "int64".to_string(),
            Self::U64 => "uint64".to_string(),
            Self::F32 => "float32".to_string(),
            Self::F64 => "float64".to_string(),
            Self::String => "string".to_string(),
            Self::Bytes => "bytes".to_string(),
            Self::StringFixed(n) => format!("string_fixed[{n}]"),
            Self::FixedArray {
                element_type,
                length,
            } => format!("{}[{}]", element_type.canonical_str(), length),
            Self::DynamicArray { element_type } => {
                format!("{}[]", element_type.canonical_str())
            }
            Self::Nested {
                schema_name,
                package,
                ..
            } => match package {
                Some(pkg) => format!("{pkg}/{schema_name}"),
                None => schema_name.clone(),
            },
        }
    }

    /// Returns true if this type is variable-length.
    ///
    /// Note: Nested types are conservatively marked as variable since we don't
    /// have access to their schema definition here. The code generator resolves
    /// this by looking up the nested schema.
    pub fn is_variable(&self) -> bool {
        match self {
            Self::String | Self::Bytes => true,
            Self::DynamicArray { .. } => true,
            Self::FixedArray { element_type, .. } => element_type.is_variable(),
            // Unresolved nested types are conservatively variable (e.g.
            // Header has a string frame_id). After `resolve_fixed_nested`
            // marks a reference as recursively fixed, it classifies as
            // fixed and lives inline in the fixed section.
            Self::Nested { fixed, .. } => fixed.is_none(),
            _ => false,
        }
    }

    /// Returns true if this type is definitely fixed-size (primitives and fixed arrays of primitives).
    pub fn is_definitely_fixed(&self) -> bool {
        match self {
            Self::Bool
            | Self::I8
            | Self::U8
            | Self::I16
            | Self::U16
            | Self::I32
            | Self::U32
            | Self::I64
            | Self::U64
            | Self::F32
            | Self::F64 => true,
            Self::StringFixed(_) => true,
            Self::FixedArray { element_type, .. } => element_type.is_definitely_fixed(),
            // Resolved-fixed nested references are definitely fixed —
            // `resolve_fixed_nested` proved the whole target schema is
            // recursively fixed-size.
            Self::Nested { fixed, .. } => fixed.is_some(),
            // These are definitely variable.
            Self::String | Self::Bytes | Self::DynamicArray { .. } => false,
        }
    }

    /// Returns true if this type is variable-length under EVERY resolution
    /// — the complement of [`is_definitely_fixed`](Self::is_definitely_fixed)
    /// minus the undecided middle: `String` / `Bytes` / `DynamicArray` and a
    /// `FixedArray` of those always carry an offset-table entry, whereas a
    /// `Nested` reference not yet resolved is only CONSERVATIVELY variable
    /// ([`is_variable`](Self::is_variable)) and may still inline.
    pub fn is_definitely_variable(&self) -> bool {
        match self {
            Self::String | Self::Bytes | Self::DynamicArray { .. } => true,
            Self::FixedArray { element_type, .. } => element_type.is_definitely_variable(),
            // Spelled out rather than folded into the wildcard: this is the
            // undecided middle the method exists to exclude.
            Self::Nested { .. } => false,
            _ => false,
        }
    }

    /// Size of fixed-size types. Returns None for variable types and for
    /// unresolved nested references; a **fixed-resolved** nested reference
    /// reports its inlined `<Target>Shm` size from `NestedFixedInfo`.
    pub fn fixed_size(&self) -> Option<usize> {
        match self {
            Self::Bool | Self::I8 | Self::U8 => Some(1),
            Self::I16 | Self::U16 => Some(2),
            Self::I32 | Self::U32 | Self::F32 => Some(4),
            Self::I64 | Self::U64 | Self::F64 => Some(8),
            Self::StringFixed(n) => Some(*n),
            Self::FixedArray {
                element_type,
                length,
            } => element_type.fixed_size().map(|s| s * length),
            Self::String | Self::Bytes | Self::DynamicArray { .. } => None,
            // A resolved-fixed nested is inlined as `<Target>Shm`;
            // its size is the target's `wire_fixed_size()`, stamped by
            // `resolve_fixed_nested`. Unresolved nested stays None.
            Self::Nested { fixed, .. } => fixed.map(|i| i.fixed_size),
        }
    }

    /// Alignment requirement for this type.
    pub fn alignment(&self) -> usize {
        match self {
            Self::Bool | Self::I8 | Self::U8 => 1,
            Self::I16 | Self::U16 => 2,
            Self::I32 | Self::U32 | Self::F32 => 4,
            Self::I64 | Self::U64 | Self::F64 => 8,
            Self::StringFixed(_) => 1,
            Self::String | Self::Bytes => 8, // Offset entry alignment
            Self::FixedArray { element_type, .. } => element_type.alignment(),
            Self::DynamicArray { .. } => 8, // Offset entry alignment
            // A resolved-fixed nested uses its target's real
            // alignment (`align_of::<<Target>Shm>()`, ≤ 8). An unresolved
            // nested is variable (skipped by `wire_fixed_size`); its 8 here
            // is the conservative offset-entry alignment.
            Self::Nested { fixed, .. } => fixed.map(|i| i.alignment).unwrap_or(8),
        }
    }

    /// Parse a type string like "float", "uint32", "string", `float[4]`, etc.
    pub fn parse(type_str: &str) -> Result<Self, String> {
        let type_str = type_str.trim();

        // Check for fixed array: "type[N]"
        if let Some(bracket_pos) = type_str.find('[') {
            if !type_str.ends_with(']') {
                return Err(format!("invalid array syntax: {type_str}"));
            }
            let base_type = &type_str[..bracket_pos];
            let length_str = &type_str[bracket_pos + 1..type_str.len() - 1];

            // Dynamic array: "type[]"
            if length_str.is_empty() {
                let element_type = Self::parse(base_type)?;
                return Ok(Self::DynamicArray {
                    element_type: Box::new(element_type),
                });
            }

            // Fixed string: "string_fixed[N]"
            if base_type == "string_fixed" {
                let length: usize = length_str
                    .parse()
                    .map_err(|_| format!("invalid string length: {length_str}"))?;
                return Ok(Self::StringFixed(length));
            }

            // Fixed array: "type[N]"
            let length: usize = length_str
                .parse()
                .map_err(|_| format!("invalid array length: {length_str}"))?;
            let element_type = Self::parse(base_type)?;
            return Ok(Self::FixedArray {
                element_type: Box::new(element_type),
                length,
            });
        }

        // Primitive types
        match type_str {
            "bool" => Ok(Self::Bool),
            "int8" | "i8" => Ok(Self::I8),
            "uint8" | "u8" | "byte" => Ok(Self::U8),
            "int16" | "i16" => Ok(Self::I16),
            "uint16" | "u16" => Ok(Self::U16),
            "int32" | "i32" | "int" => Ok(Self::I32),
            "uint32" | "u32" => Ok(Self::U32),
            "int64" | "i64" => Ok(Self::I64),
            "uint64" | "u64" => Ok(Self::U64),
            "float32" | "f32" | "float" => Ok(Self::F32),
            "float64" | "f64" | "double" => Ok(Self::F64),
            "string" => Ok(Self::String),
            "bytes" => Ok(Self::Bytes),
            // Assume anything else is a nested type. A qualified reference
            // ("pkg/Msg") keeps its package; bare names resolve later.
            other if !other.is_empty() => {
                if let Some((pkg, msg)) = other.split_once('/') {
                    if pkg.is_empty() || msg.is_empty() || msg.contains('/') {
                        return Err(format!("invalid message type: {other}"));
                    }
                    Ok(Self::Nested {
                        schema_name: msg.to_string(),
                        package: Some(pkg.to_string()),
                        fixed: None,
                    })
                } else {
                    // LOUD-REJECT a bare case-variant of a known primitive
                    // (e.g. `String`, `Bool`, `Int32`). Such a name would
                    // otherwise fall through to `Nested { schema_name:
                    // "String" }`, which downstream either errors with a
                    // confusing "unknown schema 'String'" OR silently binds
                    // to a registered `std_msgs/String` message wrapper —
                    // so `type: String` quietly means something OTHER than
                    // the `string` primitive (the schema-marker collision).
                    //
                    // The canonical lowercase spellings ARE the single
                    // source of truth (the primitive match arms above): we
                    // lowercase the bare name and re-parse. If the
                    // lowercase form parses to a non-`Nested` (i.e. a true
                    // primitive) AND differs from the original spelling,
                    // the user typed a case-variant of a primitive — reject
                    // with the canonical lowercase hint plus the
                    // package-qualified-message alternative. (We only reach
                    // this for bare names — a package-qualified
                    // `std_msgs/String` took the `split_once('/')` branch
                    // above and resolves as a message, unaffected.)
                    let lower = other.to_lowercase();
                    if lower != other {
                        if let Ok(prim) = Self::parse(&lower) {
                            if !matches!(prim, Self::Nested { .. }) {
                                return Err(format!(
                                    "field type `{other}` is not recognized — did you mean \
                                     the primitive `{lower}` (lowercase), or the message \
                                     `std_msgs/{other}` (package-qualified)?"
                                ));
                            }
                        }
                    }
                    Ok(Self::Nested {
                        schema_name: other.to_string(),
                        package: None,
                        fixed: None,
                    })
                }
            }
            _ => Err(format!("unknown type: {type_str}")),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parse_primitives() {
        assert_eq!(FieldType::parse("bool").unwrap(), FieldType::Bool);
        assert_eq!(FieldType::parse("int32").unwrap(), FieldType::I32);
        assert_eq!(FieldType::parse("uint64").unwrap(), FieldType::U64);
        assert_eq!(FieldType::parse("float").unwrap(), FieldType::F32);
        assert_eq!(FieldType::parse("double").unwrap(), FieldType::F64);
        assert_eq!(FieldType::parse("string").unwrap(), FieldType::String);
    }

    #[test]
    fn test_parse_fixed_array() {
        let ft = FieldType::parse("float[4]").unwrap();
        assert!(matches!(ft, FieldType::FixedArray { length: 4, .. }));
    }

    #[test]
    fn test_parse_dynamic_array() {
        let ft = FieldType::parse("uint8[]").unwrap();
        assert!(matches!(ft, FieldType::DynamicArray { .. }));
    }

    #[test]
    fn test_parse_fixed_string() {
        let ft = FieldType::parse("string_fixed[32]").unwrap();
        assert_eq!(ft, FieldType::StringFixed(32));
    }

    #[test]
    fn test_parse_nested() {
        let ft = FieldType::parse("Position3D").unwrap();
        assert!(matches!(ft, FieldType::Nested { schema_name, .. } if schema_name == "Position3D"));
    }

    // Contract pin: a BARE (unqualified) case-variant of a known primitive
    // is LOUD-REJECTED at parse with a hint pointing at the canonical
    // lowercase primitive AND the package-qualified-message alternative —
    // it must NOT silently become `Nested { "String" }` (the schema-marker
    // collision). Canonical lowercase spellings + package-qualified message
    // refs must keep working unchanged.
    #[test]
    fn test_parse_bare_capitalized_primitive_rejected_with_hint() {
        // `String` → reject, hint the `string` primitive + std_msgs/String.
        let err = FieldType::parse("String").unwrap_err();
        assert!(
            err.contains("string"),
            "error should hint the lowercase primitive `string`: {err}"
        );
        assert!(
            err.contains("std_msgs/String"),
            "error should mention the package-qualified message alternative: {err}"
        );

        // `Bool` → reject, hint `bool`.
        let err = FieldType::parse("Bool").unwrap_err();
        assert!(
            err.contains("bool") && err.contains("std_msgs/Bool"),
            "Bool should hint `bool` + std_msgs/Bool: {err}"
        );

        // `Int32` → reject, hint `int32`.
        let err = FieldType::parse("Int32").unwrap_err();
        assert!(
            err.contains("int32") && err.contains("std_msgs/Int32"),
            "Int32 should hint `int32` + std_msgs/Int32: {err}"
        );

        // A mixed-case alias spelling also rejects (lowercasing `FLOAT64`
        // hits the `float64` arm → primitive, so it is a case-variant).
        assert!(FieldType::parse("FLOAT64").is_err());
    }

    #[test]
    fn test_parse_canonical_lowercase_primitives_still_parse() {
        // The lowercase spellings keep parsing to the right primitive.
        assert_eq!(FieldType::parse("string").unwrap(), FieldType::String);
        assert_eq!(FieldType::parse("bool").unwrap(), FieldType::Bool);
        assert_eq!(FieldType::parse("int32").unwrap(), FieldType::I32);
    }

    #[test]
    fn test_parse_package_qualified_message_still_resolves() {
        // A package-qualified `std_msgs/String` is a legitimate MESSAGE
        // type — it must resolve as a Nested ref, NOT be rejected.
        let ft = FieldType::parse("std_msgs/String").unwrap();
        assert!(matches!(
            ft,
            FieldType::Nested { ref schema_name, package: Some(ref pkg), .. }
                if schema_name == "String" && pkg == "std_msgs"
        ));

        // A genuinely-capitalized bare MESSAGE name (not a primitive
        // case-variant) is unaffected — still a bare Nested ref.
        let ft = FieldType::parse("Position3D").unwrap();
        assert!(matches!(
            ft,
            FieldType::Nested { ref schema_name, package: None, .. }
                if schema_name == "Position3D"
        ));
    }

    #[test]
    fn test_is_variable() {
        assert!(!FieldType::Bool.is_variable());
        assert!(!FieldType::F32.is_variable());
        assert!(FieldType::String.is_variable());
        assert!(FieldType::Bytes.is_variable());
        assert!(FieldType::DynamicArray {
            element_type: Box::new(FieldType::U8)
        }
        .is_variable());
        // Nested types are conservatively marked variable
        assert!(FieldType::Nested {
            schema_name: "Header".to_string(),
            package: None,
            fixed: None,
        }
        .is_variable());
    }

    #[test]
    fn test_is_definitely_fixed() {
        assert!(FieldType::Bool.is_definitely_fixed());
        assert!(FieldType::F64.is_definitely_fixed());
        assert!(FieldType::StringFixed(32).is_definitely_fixed());
        assert!(!FieldType::String.is_definitely_fixed());
        assert!(!FieldType::Nested {
            schema_name: "Header".to_string(),
            package: None,
            fixed: None,
        }
        .is_definitely_fixed());
    }

    #[test]
    fn test_fixed_size() {
        assert_eq!(FieldType::Bool.fixed_size(), Some(1));
        assert_eq!(FieldType::I32.fixed_size(), Some(4));
        assert_eq!(FieldType::F64.fixed_size(), Some(8));
        assert_eq!(FieldType::StringFixed(32).fixed_size(), Some(32));
        assert_eq!(FieldType::String.fixed_size(), None);
    }

    #[test]
    fn test_wire_fixed_size_repr_c_padding() {
        // u8 at 0, u32 needs 4-align → offset 4..8, u16 at 8..10, trailing
        // padding to max align 4 → 12. Mirrors rustc's #[repr(C)] layout.
        let mut s = MessageSchema::new("Padded");
        s.add_field(FieldDef::new("a", FieldType::U8));
        s.add_field(FieldDef::new("b", FieldType::U32));
        s.add_field(FieldDef::new("c", FieldType::U16));
        assert_eq!(s.wire_fixed_size(), 12);
    }

    #[test]
    fn test_wire_fixed_size_skips_variable_fields() {
        let mut s = MessageSchema::new("Mixed");
        s.add_field(FieldDef::new("a", FieldType::U32));
        s.add_field(FieldDef::new("name", FieldType::String));
        s.add_field(FieldDef::new(
            "data",
            FieldType::DynamicArray {
                element_type: Box::new(FieldType::U8),
            },
        ));
        assert_eq!(s.wire_fixed_size(), 4, "only the u32 is fixed-section");
    }

    #[test]
    fn test_wire_fixed_size_empty_fixed_section_is_zero() {
        let mut s = MessageSchema::new("VarOnly");
        s.add_field(FieldDef::new("name", FieldType::String));
        assert_eq!(s.wire_fixed_size(), 0);
    }

    /// The offset-table counts a wire-size ceiling relies on: the RESOLVED
    /// count (`variable_field_count`) is exactly the layout walker's
    /// `variable_fields` (an unresolved nested reference included), the FLOOR
    /// excludes the reference because resolution may still inline it — and
    /// stamping the reference fixed (what `resolve_fixed_nested` does) closes
    /// the gap from above. A fixed-length string and a fixed array of an
    /// unresolved reference are NOT definitely variable (the first is inline,
    /// the second may still inline) — counting either as variable would inflate
    /// the floor and refuse a representable schema.
    #[test]
    fn offset_table_entry_counts_split_only_on_unresolved_nested_references() {
        let mut s = MessageSchema::new("Probe");
        s.add_field(FieldDef::new("a", FieldType::U32));
        s.add_field(FieldDef::new("s", FieldType::String));
        s.add_field(FieldDef::new("b", FieldType::Bytes));
        s.add_field(FieldDef::new(
            "d",
            FieldType::DynamicArray {
                element_type: Box::new(FieldType::F64),
            },
        ));
        s.add_field(FieldDef::new(
            "names",
            FieldType::FixedArray {
                element_type: Box::new(FieldType::String),
                length: 3,
            },
        ));
        s.add_field(FieldDef::new(
            "n",
            FieldType::Nested {
                schema_name: "Other".to_string(),
                package: None,
                fixed: None,
            },
        ));
        assert_eq!(
            s.variable_field_count(),
            5,
            "s, b, d, names[] and the unresolved n"
        );
        assert_eq!(
            s.variable_field_count_floor(),
            4,
            "the reference may still inline"
        );
        assert!(FieldType::String.is_definitely_variable());
        assert!(!FieldType::U32.is_definitely_variable());
        assert!(!FieldType::StringFixed(16).is_definitely_variable());
        let unresolved = || FieldType::Nested {
            schema_name: "Other".to_string(),
            package: None,
            fixed: None,
        };
        assert!(!unresolved().is_definitely_variable());
        assert!(!FieldType::FixedArray {
            element_type: Box::new(unresolved()),
            length: 2,
        }
        .is_definitely_variable());
        // Resolve the reference fixed: the counts meet.
        if let FieldType::Nested { fixed, .. } = &mut s.fields[5].field_type {
            *fixed = Some(NestedFixedInfo {
                has_large_array: false,
                fixed_size: 8,
                alignment: 8,
                target_hash: 0,
            });
        }
        assert_eq!(s.variable_field_count(), 4);
        assert_eq!(s.variable_field_count_floor(), 4);
    }

    #[test]
    fn test_wire_fixed_size_bool_stored_as_one_byte() {
        let mut s = MessageSchema::new("Flags");
        s.add_field(FieldDef::new("a", FieldType::Bool));
        s.add_field(FieldDef::new("b", FieldType::Bool));
        assert_eq!(s.wire_fixed_size(), 2);
    }

    #[test]
    #[should_panic(expected = "FixedArray size overflows")]
    fn test_wire_fixed_size_hostile_fixed_array_panics_loudly() {
        // Hostile length: u64 elements × (usize::MAX / 4) overflows usize.
        // Must panic with a field-naming message — never silently wrap.
        let mut s = MessageSchema::new("Hostile");
        s.add_field(FieldDef::new(
            "evil",
            FieldType::FixedArray {
                element_type: Box::new(FieldType::U64),
                length: usize::MAX / 4,
            },
        ));
        let _ = s.wire_fixed_size();
    }

    /// The fallible twin: same arithmetic, `Err` instead of a panic — the
    /// pre-flight probe combined-set folders (workspace `.msg` store + YAML)
    /// use to SKIP a hostile schema loudly instead of crashing the verb.
    #[test]
    fn test_checked_wire_fixed_size_errs_on_hostile_array_and_matches_on_sane() {
        // Hostile: identical shape to the should_panic twin above; the error
        // message is the SAME string the panicking wrapper raises, naming
        // schema + field.
        let mut hostile = MessageSchema::new("Hostile");
        hostile.add_field(FieldDef::new(
            "evil",
            FieldType::FixedArray {
                element_type: Box::new(FieldType::U64),
                length: usize::MAX / 4,
            },
        ));
        let err = hostile
            .checked_wire_fixed_size()
            .expect_err("a hostile FixedArray length must be an Err, not a wrap");
        assert!(
            err.contains("Hostile") && err.contains("evil") && err.contains("overflows"),
            "the error names schema + field + the overflow: {err}"
        );

        // Sane: Ok, and byte-identical to the panicking accessor (hand
        // oracle: u32 at 0, f64 aligned to 8 → 8..16 → total 16).
        let mut sane = MessageSchema::new("Sane");
        sane.add_field(FieldDef::new("a", FieldType::U32));
        sane.add_field(FieldDef::new("b", FieldType::F64));
        assert_eq!(sane.checked_wire_fixed_size(), Ok(16));
        assert_eq!(sane.wire_fixed_size(), 16);
    }

    #[test]
    fn test_canonical_str_covers_all_variants() {
        assert_eq!(FieldType::Bool.canonical_str(), "bool");
        assert_eq!(FieldType::I8.canonical_str(), "int8");
        assert_eq!(FieldType::U8.canonical_str(), "uint8");
        assert_eq!(FieldType::I16.canonical_str(), "int16");
        assert_eq!(FieldType::U16.canonical_str(), "uint16");
        assert_eq!(FieldType::I32.canonical_str(), "int32");
        assert_eq!(FieldType::U32.canonical_str(), "uint32");
        assert_eq!(FieldType::I64.canonical_str(), "int64");
        assert_eq!(FieldType::U64.canonical_str(), "uint64");
        assert_eq!(FieldType::F32.canonical_str(), "float32");
        assert_eq!(FieldType::F64.canonical_str(), "float64");
        assert_eq!(FieldType::String.canonical_str(), "string");
        assert_eq!(FieldType::Bytes.canonical_str(), "bytes");
        assert_eq!(
            FieldType::StringFixed(16).canonical_str(),
            "string_fixed[16]"
        );
        assert_eq!(
            FieldType::FixedArray {
                element_type: Box::new(FieldType::F64),
                length: 9,
            }
            .canonical_str(),
            "float64[9]"
        );
        assert_eq!(
            FieldType::DynamicArray {
                element_type: Box::new(FieldType::U8),
            }
            .canonical_str(),
            "uint8[]"
        );
        // Bare (package-less) nested → schema name verbatim.
        assert_eq!(
            FieldType::Nested {
                schema_name: "Header".to_string(),
                package: None,
                fixed: None,
            }
            .canonical_str(),
            "Header"
        );
        // Recipe 3: a packaged nested renders "pkg/Name" so two
        // packages' same-named nested refs hash distinctly.
        assert_eq!(
            FieldType::Nested {
                schema_name: "Header".to_string(),
                package: Some("std_msgs".to_string()),
                fixed: None,
            }
            .canonical_str(),
            "std_msgs/Header"
        );
    }

    #[test]
    fn test_schema_has_variable_fields() {
        let mut schema = MessageSchema::new("Test");
        schema.add_field(FieldDef::new("x", FieldType::F32));
        assert!(!schema.has_variable_fields());

        schema.add_field(FieldDef::new("name", FieldType::String));
        assert!(schema.has_variable_fields());
    }

    #[test]
    fn test_schema_variable_field_count() {
        let mut schema = MessageSchema::new("Test");
        schema.add_field(FieldDef::new("x", FieldType::F32));
        schema.add_field(FieldDef::new("name", FieldType::String));
        schema.add_field(FieldDef::new("data", FieldType::Bytes));
        assert_eq!(schema.variable_field_count(), 2);
    }

    /// Independent oracle for the recipe-3 hash recipe: hand-assemble
    /// the documented byte stream and one-shot FNV-1a it, then compare to the
    /// streaming `schema_hash()`. Every OTHER hash test runs the same recipe
    /// function on both sides (so they cannot catch a bug INSIDE the recipe);
    /// this one re-derives the stream by hand, so a transposed step, a missing
    /// length prefix, a dropped `wire_fixed_size`, or a bare-vs-qualified name
    /// mistake would diverge here.
    ///
    /// Covers the no-nested portion (steps 1–3 a/b). The nested `target_hash`
    /// fold (step 3c) is proven by the `resolve.rs` reorder/array/3-level
    /// tests, which would fail if the fold were wrong — re-deriving it here
    /// would not be independent (it would reuse the same recipe to hash the
    /// target).
    #[test]
    fn schema_hash_matches_hand_assembled_recipe3_stream() {
        use crate::wire::fnv1a_hash;
        fn oracle(schema: &MessageSchema) -> u64 {
            let mut bytes = Vec::new();
            let q = schema.qualified_name();
            bytes.extend_from_slice(&(q.len() as u64).to_le_bytes());
            bytes.extend_from_slice(q.as_bytes());
            bytes.extend_from_slice(&(schema.wire_fixed_size() as u64).to_le_bytes());
            for f in &schema.fields {
                bytes.extend_from_slice(&(f.name.len() as u64).to_le_bytes());
                bytes.extend_from_slice(f.name.as_bytes());
                let cs = f.field_type.canonical_str();
                bytes.extend_from_slice(&(cs.len() as u64).to_le_bytes());
                bytes.extend_from_slice(cs.as_bytes());
            }
            fnv1a_hash(&bytes)
        }

        // Fixed, packaged schema → qualified name + non-zero wire_fixed_size.
        let mut point = MessageSchema::new_in_package("Point", "geometry_msgs");
        for f in ["x", "y", "z"] {
            point.add_field(FieldDef::new(f, FieldType::F64));
        }
        assert_eq!(point.wire_fixed_size(), 24);
        assert_eq!(point.schema_hash(), oracle(&point));

        // Variable, package-less schema → bare name + zero fixed section.
        let mut blob = MessageSchema::new("Blob");
        blob.add_field(FieldDef::new("name", FieldType::String));
        blob.add_field(FieldDef::new(
            "data",
            FieldType::DynamicArray {
                element_type: Box::new(FieldType::U8),
            },
        ));
        assert_eq!(blob.wire_fixed_size(), 0);
        assert_eq!(blob.schema_hash(), oracle(&blob));

        // The packaged and bare hashes must differ even with overlapping
        // structure — qualification is in the stream.
        assert_ne!(point.schema_hash(), blob.schema_hash());
    }
}
