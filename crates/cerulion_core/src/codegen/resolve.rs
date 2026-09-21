// SPDX-License-Identifier: AGPL-3.0-only
//! Nested-schema fixedness resolution.
//!
//! The parser conservatively classifies every [`FieldType::Nested`]
//! reference as variable — without the target schema's definition it cannot
//! know whether `geometry_msgs/Pose` is fixed-size. That conservatism
//! defeats zero-copy loaning for almost every real ROS2 message: `Pose`
//! (two all-`f64` nested structs) was emitted as TWO opaque-bytes variable
//! fields.
//!
//! [`resolve_fixed_nested`] is the second phase between parsing and code
//! generation. Given every schema in the build, it:
//!
//! 1. Computes recursive fixedness per schema (a schema is fixed iff every
//!    field is fixed; a `Nested` field is fixed iff its target schema is).
//! 2. Rewrites each `Nested` reference whose target is recursively fixed,
//!    setting [`NestedFixedInfo`] so the classification methods
//!    (`is_variable` / `is_definitely_fixed`) flip to fixed and the
//!    generator inlines the field into the parent's fixed section as
//!    `<Target>Shm` — zero-copy, direct field access. The stamp also
//!    carries the target's resolved `(wire_fixed_size, wire_alignment,
//!    schema_hash)`, computed **bottom-up** so the parent's own
//!    `wire_fixed_size()` and recipe-3 `schema_hash()` see the true inlined
//!    layout and fold in the target's full hash (recipe 3 — a
//!    layout change inside an inlined target bumps every embedder's hash).
//!
//! Reference resolution follows rosidl semantics: qualified (`pkg/Msg`)
//! → exact; unqualified → same package first, then the single legacy
//! special case bare `Header` → `std_msgs/Header` (silent), then the
//! global bare namespace iff unambiguous — that last step is something
//! rosidl would REJECT, so it always emits a loud warning.
//!
//! Warnings are produced for: unresolvable references (unknown target,
//! ambiguous bare name), reference cycles, duplicate schema definitions,
//! cross-package bare bindings, and typo'd `DynamicArray` element types.
//! Unresolvable references are left untouched — they keep the
//! conservative variable classification. Missing schemas are NOT an
//! error here: workspace YAML schemas and partial vendoring must keep
//! working (the `native_ros2_messages` build script independently
//! escalates ANY warning to a build failure for the vendored set).
//!
//! # Determinism
//!
//! Resolution is a pure function of the input schema set: lookups go
//! through `BTreeMap`s, iteration order is declaration order, and the
//! fixedness DFS memoizes per target. Same input → same rewrites → same
//! generated code (Principle #7).

use super::schema::{FieldType, MessageSchema, NestedFixedInfo};
use std::collections::BTreeMap;

/// Key for schema lookup: `(package, name)`. Package-less schemas key on
/// `(None, name)`.
type SchemaKey = (Option<String>, String);

/// Computed fixedness for one schema.
#[derive(Debug, Clone, Copy, PartialEq)]
enum Fixedness {
    /// Recursively fixed-size; payload is `has_large_array` (transitive).
    Fixed { has_large_array: bool },
    /// Contains at least one variable field (or an unresolvable /
    /// cyclic nested reference, which we conservatively treat as variable).
    Variable,
}

/// Fully-resolved layout of a recursively-fixed schema (recipe 3),
/// computed bottom-up and memoized. Every value is taken from a clone of the
/// schema whose nested refs have ALL been resolved first, so the `#[repr(C)]`
/// methods ([`MessageSchema::wire_fixed_size`] / `wire_alignment` /
/// `schema_hash`) reflect the true inlined layout — no recipe duplication.
#[derive(Debug, Clone, Copy, PartialEq)]
struct ResolvedFixed {
    /// `size_of::<<Target>Shm>()` — the inlined fixed-section size.
    wire_fixed_size: usize,
    /// `align_of::<<Target>Shm>()` (≤ 8).
    wire_alignment: usize,
    /// The target's own recipe-3 `schema_hash()` (folded into parents so a
    /// change anywhere inside the target bumps every embedder's hash).
    schema_hash: u64,
    /// Transitive large-array flag (mirrors [`Fixedness::Fixed`]).
    has_large_array: bool,
}

impl From<ResolvedFixed> for NestedFixedInfo {
    /// Single chokepoint that maps a bottom-up-resolved target's layout
    /// into the IR stamp carried on a parent's `FieldType::Nested`. Keeping
    /// the field mapping in one place (vs. hand-copying at each stamp site)
    /// removes the chance of a transposed-field bug on a wire-format value.
    fn from(r: ResolvedFixed) -> Self {
        debug_assert!(
            r.wire_alignment.is_power_of_two() && r.wire_alignment <= 8,
            "fixed-nested alignment must be a power of two <= 8, got {}",
            r.wire_alignment
        );
        NestedFixedInfo {
            has_large_array: r.has_large_array,
            fixed_size: r.wire_fixed_size,
            alignment: r.wire_alignment,
            target_hash: r.schema_hash,
        }
    }
}

/// Resolve nested-reference fixedness across `schemas`, in place.
///
/// Returns human-readable warnings (see the module docs for the full
/// list of warning sources — unresolved/ambiguous/cyclic references,
/// duplicates, cross-package bare bindings, array-element typos).
/// Callers in build scripts should surface them via `cargo:warning=` or
/// escalate to a build failure; the runtime CLI path should
/// `tracing::warn!` them — the inference must be loud, never silent.
pub fn resolve_fixed_nested(schemas: &mut [MessageSchema]) -> Vec<String> {
    let mut resolver = Resolver::new(schemas);

    // Force-compute fixedness for every schema (fully populates the memo).
    for idx in 0..resolver.snapshot.len() {
        resolver.fixedness_of(idx);
    }

    // Rewrite phase: mark resolved-fixed Nested references on the live
    // schemas using the memoized fixedness of the pre-rewrite snapshot.
    for schema in schemas.iter_mut() {
        let parent_package = schema.package.clone();
        let parent_name = schema.name.clone();
        for field in &mut schema.fields {
            resolver.rewrite_field(
                &mut field.field_type,
                &field.name,
                &parent_name,
                &parent_package,
            );
        }
    }

    resolver.warnings
}

/// Internal state for one resolution run: the immutable pre-rewrite
/// snapshot, lookup registries, the fixedness memo, and accumulated
/// warnings.
struct Resolver {
    /// Pre-rewrite copy of the schemas, so fixedness computation does not
    /// depend on rewrite order.
    snapshot: Vec<MessageSchema>,
    /// Qualified `(package, name)` → snapshot index.
    by_key: BTreeMap<SchemaKey, usize>,
    /// Bare name → all snapshot indices carrying it (collision-aware).
    by_bare: BTreeMap<String, Vec<usize>>,
    /// Memoized fixedness per snapshot index.
    memo: Vec<Option<Fixedness>>,
    /// Memoized fully-resolved layout per snapshot index (recipe 3). Only
    /// populated for recursively-fixed schemas (the only ones that get
    /// inlined into a parent's fixed section). Computed bottom-up.
    resolved_info: Vec<Option<ResolvedFixed>>,
    /// In-progress DFS markers for cycle detection.
    visiting: Vec<bool>,
    warnings: Vec<String>,
}

impl Resolver {
    fn new(schemas: &[MessageSchema]) -> Self {
        let mut warnings = Vec::new();
        let mut by_key: BTreeMap<SchemaKey, usize> = BTreeMap::new();
        let mut by_bare: BTreeMap<String, Vec<usize>> = BTreeMap::new();
        for (idx, schema) in schemas.iter().enumerate() {
            let key = (schema.package.clone(), schema.name.clone());
            if let Some(prev) = by_key.insert(key, idx) {
                warnings.push(format!(
                    "duplicate schema definition for '{}' — later definition wins in resolution",
                    schema.qualified_name()
                ));
                // Keep `by_bare` CONSISTENT with `by_key` ("later wins" on
                // BOTH paths): replace the earlier index instead of
                // double-registering. Double-counting would make an
                // unqualified reference report "ambiguous (defined in 2
                // packages)" — wrong count, wrong diagnosis; keeping the
                // earlier index would make bare lookups bind a different
                // definition than qualified ones.
                if let Some(v) = by_bare.get_mut(&schema.name) {
                    for slot in v.iter_mut() {
                        if *slot == prev {
                            *slot = idx;
                        }
                    }
                }
                continue;
            }
            by_bare.entry(schema.name.clone()).or_default().push(idx);
        }
        let len = schemas.len();
        Self {
            snapshot: schemas.to_vec(),
            by_key,
            by_bare,
            memo: vec![None; len],
            resolved_info: vec![None; len],
            visiting: vec![false; len],
            warnings,
        }
    }

    /// Resolve a Nested reference to a snapshot index. `parent_package` is
    /// the package of the schema CONTAINING the reference (ROS2 semantics:
    /// an unqualified reference means "same package").
    fn lookup(
        &self,
        reference_pkg: &Option<String>,
        name: &str,
        parent_package: &Option<String>,
    ) -> Result<Found, String> {
        if let Some(pkg) = reference_pkg {
            // Qualified reference: exact lookup only.
            return self
                .by_key
                .get(&(Some(pkg.clone()), name.to_string()))
                .map(|&idx| Found {
                    idx,
                    cross_package_bare: false,
                })
                .ok_or_else(|| format!("unknown schema '{pkg}/{name}'"));
        }
        // Unqualified: same package first (ROS2 semantics) …
        if let Some(&idx) = self.by_key.get(&(parent_package.clone(), name.to_string())) {
            return Ok(Found {
                idx,
                cross_package_bare: false,
            });
        }
        // … then rosidl's single legacy special case: bare `Header` means
        // `std_msgs/Header` (used unqualified by moveit_msgs and friends).
        // This is the only cross-package bare resolution rosidl accepts,
        // so it resolves silently.
        if name == "Header" {
            if let Some(&idx) = self
                .by_key
                .get(&(Some("std_msgs".to_string()), "Header".to_string()))
            {
                return Ok(Found {
                    idx,
                    cross_package_bare: false,
                });
            }
        }
        // … then the global bare-name namespace, iff unambiguous. rosidl
        // would REJECT this (unqualified means same-package, Header
        // excepted) — we resolve it for pragmatism but flag it so the
        // rewrite phase emits a loud warning: silent cross-package binding
        // can flip when a second package later defines the same bare
        // name.
        match self.by_bare.get(name).map(|v| v.as_slice()) {
            Some([idx]) => Ok(Found {
                idx: *idx,
                cross_package_bare: true,
            }),
            Some(multi) if multi.len() > 1 => Err(format!(
                "ambiguous unqualified reference '{name}' (defined in {} packages) — qualify as pkg/{name}",
                multi.len()
            )),
            _ => Err(format!("unknown schema '{name}'")),
        }
    }

    /// Memoized recursive fixedness of the schema at `idx` (DFS with
    /// cycle detection over the pre-rewrite snapshot).
    fn fixedness_of(&mut self, idx: usize) -> Fixedness {
        if let Some(f) = self.memo[idx] {
            return f;
        }
        if self.visiting[idx] {
            // Reference cycle. ROS2 .msg definitions cannot be cyclic, but
            // hand-written schema sets can — terminate, warn loudly, and
            // classify conservatively as variable.
            self.warnings.push(format!(
                "schema reference cycle detected at '{}' — treating as variable",
                self.snapshot[idx].qualified_name()
            ));
            return Fixedness::Variable;
        }
        self.visiting[idx] = true;

        let parent_package = self.snapshot[idx].package.clone();
        let fields: Vec<FieldType> = self.snapshot[idx]
            .fields
            .iter()
            .map(|f| f.field_type.clone())
            .collect();

        let mut acc = FixednessAcc {
            fixed: true,
            has_large_array: false,
        };
        for ft in &fields {
            self.walk(ft, &parent_package, &mut acc);
        }

        self.visiting[idx] = false;
        let result = if acc.fixed {
            Fixedness::Fixed {
                has_large_array: acc.has_large_array,
            }
        } else {
            Fixedness::Variable
        };
        self.memo[idx] = Some(result);
        result
    }

    /// Field-type walk for [`fixedness_of`](Self::fixedness_of); recurses
    /// into `FixedArray` elements and nested targets.
    fn walk(&mut self, ft: &FieldType, parent_package: &Option<String>, acc: &mut FixednessAcc) {
        match ft {
            FieldType::String | FieldType::Bytes | FieldType::DynamicArray { .. } => {
                acc.fixed = false;
            }
            FieldType::FixedArray {
                element_type,
                length,
            } => {
                if *length > 32 {
                    acc.has_large_array = true;
                }
                self.walk(element_type, parent_package, acc);
            }
            FieldType::Nested {
                schema_name,
                package,
                ..
            } => match self.lookup(package, schema_name, parent_package) {
                Ok(found) => match self.fixedness_of(found.idx) {
                    Fixedness::Fixed { has_large_array } => {
                        if has_large_array {
                            acc.has_large_array = true;
                        }
                    }
                    Fixedness::Variable => acc.fixed = false,
                },
                Err(_) => {
                    // Unresolvable → conservative variable. The warning is
                    // emitted once per REFERENCE during the rewrite phase
                    // (richer context there); here it only affects the
                    // parent's classification.
                    acc.fixed = false;
                }
            },
            // Primitives + StringFixed: fixed, no large-array impact.
            _ => {}
        }
    }

    /// Bottom-up, memoized full layout of the recursively-fixed schema at
    /// `idx` (recipe 3). Builds a clone of the snapshot schema with
    /// EVERY nested ref resolved first (recursively), then reads
    /// `wire_fixed_size()` / `wire_alignment()` / `schema_hash()` off that
    /// fully-resolved clone — so the values reflect the true inlined
    /// `#[repr(C)]` layout and the recursive hash, with the recipe living in
    /// exactly one place (the `MessageSchema` methods).
    ///
    /// Precondition: `self.memo[idx]` is `Fixed` (only fixed schemas are
    /// inlined). Fixed schemas are acyclic — a reference cycle is downgraded
    /// to `Variable` by [`fixedness_of`](Self::fixedness_of) — so the
    /// recursion terminates; the memo also prevents recomputation.
    fn resolved_fixed_of(&mut self, idx: usize) -> ResolvedFixed {
        if let Some(r) = self.resolved_info[idx] {
            return r;
        }
        debug_assert!(
            matches!(self.memo[idx], Some(Fixedness::Fixed { .. })),
            "resolved_fixed_of called on a non-fixed schema"
        );
        let has_large_array = matches!(
            self.memo[idx],
            Some(Fixedness::Fixed {
                has_large_array: true
            })
        );

        // Work on a clone so the snapshot is never mutated (determinism +
        // order-independence). Resolve each field's nested refs first.
        let mut resolved = self.snapshot[idx].clone();
        let parent_package = resolved.package.clone();
        for fi in 0..resolved.fields.len() {
            let ft = resolved.fields[fi].field_type.clone();
            resolved.fields[fi].field_type = self.resolve_ft_for_layout(&ft, &parent_package);
        }

        let info = ResolvedFixed {
            wire_fixed_size: resolved.wire_fixed_size(),
            wire_alignment: resolved.wire_alignment(),
            schema_hash: resolved.schema_hash(),
            has_large_array,
        };
        self.resolved_info[idx] = Some(info);
        info
    }

    /// Return a copy of `ft` with every recursively-fixed nested reference
    /// stamped with its [`NestedFixedInfo`] (resolved bottom-up via
    /// [`resolved_fixed_of`](Self::resolved_fixed_of)). Recurses into
    /// `FixedArray` elements. Variable / unresolvable nested refs are
    /// returned unchanged (they stay `fixed: None`).
    fn resolve_ft_for_layout(
        &mut self,
        ft: &FieldType,
        parent_package: &Option<String>,
    ) -> FieldType {
        match ft {
            FieldType::Nested {
                schema_name,
                package,
                ..
            } => {
                if let Ok(found) = self.lookup(package, schema_name, parent_package) {
                    if matches!(self.memo[found.idx], Some(Fixedness::Fixed { .. })) {
                        let child = self.resolved_fixed_of(found.idx);
                        return FieldType::Nested {
                            schema_name: schema_name.clone(),
                            package: package.clone(),
                            fixed: Some(child.into()),
                        };
                    }
                }
                // Fallthrough (lookup Err or target Variable): leave the ref
                // unchanged (`fixed: None`). Under this method's contract —
                // it only runs for the fields of an already-`Fixed` schema,
                // whose every nested ref resolved Ok to a Fixed target — the
                // Err branch is effectively unreachable; the Variable branch
                // cannot occur either (a Variable field would have made the
                // parent Variable). Kept defensive. Any genuine lookup Err is
                // separately reported by `rewrite_field`, so it is not
                // silently dropped here.
                ft.clone()
            }
            FieldType::FixedArray {
                element_type,
                length,
            } => FieldType::FixedArray {
                element_type: Box::new(self.resolve_ft_for_layout(element_type, parent_package)),
                length: *length,
            },
            other => other.clone(),
        }
    }

    /// Rewrite one field type in place (recursing into FixedArray
    /// elements): set `fixed: Some(info)` on Nested references whose
    /// target is recursively fixed; warn on unresolvable references.
    fn rewrite_field(
        &mut self,
        ft: &mut FieldType,
        field_name: &str,
        parent_name: &str,
        parent_package: &Option<String>,
    ) {
        match ft {
            FieldType::Nested {
                schema_name,
                package,
                fixed,
            } => match self.lookup(package, schema_name, parent_package) {
                Ok(found) => {
                    if found.cross_package_bare {
                        // rosidl would reject this reference (unqualified
                        // means same-package, `Header` excepted). We
                        // resolve it, but never silently — the binding can
                        // flip when a second package later defines the
                        // same bare name.
                        let target = self.snapshot[found.idx].qualified_name();
                        self.warnings.push(format!(
                            "{parent_name}.{field_name}: unqualified reference '{schema_name}' \
                             resolved CROSS-PACKAGE to '{target}' — rosidl would reject this; \
                             qualify the reference in the .msg file"
                        ));
                    }
                    // Re-resolution is authoritative:
                    // stamp the resolved layout for a fixed target and
                    // CLEAR a stale flag for a variable one. A stale `Some`
                    // from a prior resolution against a different (fuller)
                    // schema set must not survive — it would route the field
                    // into the fixed section with a layout the current set
                    // cannot produce.
                    if matches!(self.memo[found.idx], Some(Fixedness::Fixed { .. })) {
                        // Recipe 3: stamp the target's full resolved
                        // layout (size / alignment / recursive hash), computed
                        // bottom-up so nested-of-nested is already populated.
                        let child = self.resolved_fixed_of(found.idx);
                        *fixed = Some(child.into());
                    } else {
                        // Variable (or unresolved-to-variable) target: clear
                        // any stale flag. Genuinely-variable is a fact, but a
                        // leftover `Some` from a fuller set must not survive.
                        *fixed = None;
                    }
                }
                Err(reason) => {
                    if fixed.is_some() {
                        // Clear a stale flag from a prior resolution run
                        // against a different schema set — conservative
                        // variable is the only safe classification for a
                        // reference that no longer resolves.
                        *fixed = None;
                    }
                    self.warnings.push(format!(
                        "{parent_name}.{field_name}: {reason} — field stays conservatively \
                         variable (opaque-bytes accessors, not zero-copy)"
                    ));
                }
            },
            FieldType::FixedArray { element_type, .. } => {
                self.rewrite_field(element_type, field_name, parent_name, parent_package);
            }
            FieldType::DynamicArray { element_type } => {
                // The element reference is intentionally NOT rewritten — a
                // dynamic array of fixed nested types still lives in the
                // variable payload (the ARRAY is variable; typed-slice
                // access is a separate, additive feature). But a TYPO'd
                // element type would otherwise get zero diagnostics
                // anywhere — look it up purely for the
                // unknown-reference warning.
                if let FieldType::Nested {
                    schema_name,
                    package,
                    ..
                } = element_type.as_ref()
                {
                    if let Err(reason) = self.lookup(package, schema_name, parent_package) {
                        self.warnings.push(format!(
                            "{parent_name}.{field_name}: array element: {reason} — array stays \
                             opaque-bytes (layout unaffected)"
                        ));
                    }
                }
            }
            _ => {}
        }
    }
}

/// Successful lookup result: the snapshot index plus whether the binding
/// crossed package boundaries through the bare-name fallback (which rosidl
/// semantics would reject — the rewrite phase warns loudly on it).
struct Found {
    idx: usize,
    cross_package_bare: bool,
}

/// Accumulator for one schema's fixedness walk.
struct FixednessAcc {
    fixed: bool,
    has_large_array: bool,
}

/// The COMPOSED-overflow preflight probe: the indices (ascending) of
/// schemas that classify recursively FIXED over `schemas` (the resolver's
/// classification — sizes play no part in fixedness) but whose composed
/// fixed-section arithmetic overflows — the exact set
/// [`resolve_fixed_nested`] would PANIC materializing as nested targets
/// (`resolved_fixed_of` sizes each fixed target through the panicking
/// `wire_fixed_size`). A caller folding UNTRUSTED declarations (a
/// workspace `.msg` store acquired over the wire, hand-edited YAML, a
/// peer-served schema doc) runs this to a fixpoint BEFORE resolution —
/// drop the reported definitions, re-probe (a drop can flip a bare
/// reference's resolution), repeat until empty — so a hostile
/// composition degrades loudly instead of crashing the verb.
///
/// Lives beside the resolver it mirrors (hoisted from the CLI engine's
/// `retain_composed_sizable_schemas`, which stays the removal POLICY over
/// it): a second copy of the lookup/fixedness rules is how the first
/// consumer without one — the recorder's served-schema corpus — shipped
/// with a remotely reachable panic.
///
/// Only ACTIVE definitions — the later-wins `by_key` values, exactly the
/// set the resolver's `lookup` can ever bind — are classified:
/// a `by_key`-shadowed duplicate is invisible to
/// resolution, so it can neither panic the resolver nor deserve removal;
/// it is left alone precisely as core's later-wins registries leave it
/// (which also warn it as a duplicate downstream). Observationally this
/// filter is a semantic mirror, not a load-bearing guard — the removal
/// rule alone would drop an unreachable loser harmlessly — but skipping
/// it keeps the preflight from ever mutating what resolution cannot see.
pub fn composed_overflow_indices(schemas: &[MessageSchema]) -> Vec<usize> {
    // Mirror the resolver's registries: qualified `(package, name)` key
    // with later-wins duplicate handling, the bare index kept consistent
    // (a double-registered bare name would misreport ambiguity).
    let mut by_key: BTreeMap<(Option<String>, String), usize> = BTreeMap::new();
    let mut by_bare: BTreeMap<String, Vec<usize>> = BTreeMap::new();
    for (idx, schema) in schemas.iter().enumerate() {
        let key = (schema.package.clone(), schema.name.clone());
        if let Some(prev) = by_key.insert(key, idx) {
            if let Some(slots) = by_bare.get_mut(&schema.name) {
                for slot in slots.iter_mut() {
                    if *slot == prev {
                        *slot = idx;
                    }
                }
            }
            continue;
        }
        by_bare.entry(schema.name.clone()).or_default().push(idx);
    }
    let active: std::collections::HashSet<usize> = by_key.values().copied().collect();
    let mut probe = ComposedSizeProbe {
        schemas,
        by_key,
        by_bare,
        memo: vec![None; schemas.len()],
        visiting: vec![false; schemas.len()],
    };
    (0..schemas.len())
        .filter(|idx| active.contains(idx))
        .filter(|&idx| matches!(probe.classify(idx), ComposedClass::Overflow))
        .collect()
}

/// A schema's composed-layout classification within one
/// [`composed_overflow_indices`] pass.
#[derive(Debug, Clone, Copy)]
enum ComposedClass {
    /// The resolver classifies it VARIABLE (a variable-typed field, an
    /// unresolvable/ambiguous nested ref, a cycle, or — this pass only —
    /// a ref to an [`Overflow`](Self::Overflow) target that the fixpoint
    /// is about to drop), so it is never inlined into a parent and never
    /// sized by the resolver.
    NotInlinable,
    /// Recursively fixed with a computable composed layout
    /// (`size`/`align` = the target's inlined `<Target>Shm` layout).
    Fixed { size: usize, align: usize },
    /// Recursively fixed per the resolver's classification, but the
    /// composed size arithmetic overflows — the panic shape; dropped.
    Overflow,
}

/// The memoized bottom-up composed-size walk behind
/// [`composed_overflow_indices`] — the caller-side twin of the resolver's
/// `fixedness_of` + `resolved_fixed_of`, with core's checked arithmetic
/// substituted for the panicking size call.
struct ComposedSizeProbe<'a> {
    schemas: &'a [MessageSchema],
    by_key: BTreeMap<(Option<String>, String), usize>,
    by_bare: BTreeMap<String, Vec<usize>>,
    memo: Vec<Option<ComposedClass>>,
    visiting: Vec<bool>,
}

impl ComposedSizeProbe<'_> {
    /// Mirror of the resolver's nested-reference lookup: qualified exact →
    /// same package → bare `Header` → `std_msgs/Header` → unambiguous
    /// global bare; `None` for unknown AND ambiguous (both classify the
    /// referencing field variable, exactly as the resolver's `Err` arm).
    fn lookup(
        &self,
        reference_pkg: &Option<String>,
        name: &str,
        parent_package: &Option<String>,
    ) -> Option<usize> {
        if let Some(pkg) = reference_pkg {
            return self
                .by_key
                .get(&(Some(pkg.clone()), name.to_string()))
                .copied();
        }
        if let Some(&idx) = self.by_key.get(&(parent_package.clone(), name.to_string())) {
            return Some(idx);
        }
        if name == "Header" {
            if let Some(&idx) = self
                .by_key
                .get(&(Some("std_msgs".to_string()), "Header".to_string()))
            {
                return Some(idx);
            }
        }
        match self.by_bare.get(name).map(|v| v.as_slice()) {
            Some([idx]) => Some(*idx),
            _ => None,
        }
    }

    /// Memoized recursive classification of the schema at `idx` (DFS with
    /// cycle detection, matching the resolver's `fixedness_of` shape; the
    /// cycle response is deliberately NOT memoized, exactly as there).
    fn classify(&mut self, idx: usize) -> ComposedClass {
        if let Some(class) = self.memo[idx] {
            return class;
        }
        if self.visiting[idx] {
            // Reference cycle — the resolver downgrades to variable.
            return ComposedClass::NotInlinable;
        }
        self.visiting[idx] = true;
        let parent_package = self.schemas[idx].package.clone();
        // Stamp a clone bottom-up so core's own checked arithmetic
        // computes the composed layout: every nested ref whose target
        // classifies Fixed carries the target's composed size/alignment
        // (`target_hash` is irrelevant to sizing and left 0 — the clone
        // is never hashed).
        let mut stamped = self.schemas[idx].clone();
        let mut fixed = true;
        for field in &mut stamped.fields {
            let ft = field.field_type.clone();
            field.field_type = self.probe_field_type(&ft, &parent_package, &mut fixed);
        }
        self.visiting[idx] = false;
        let class = if !fixed {
            ComposedClass::NotInlinable
        } else {
            match stamped.checked_wire_fixed_size() {
                Ok(size) => ComposedClass::Fixed {
                    size,
                    align: stamped.wire_alignment(),
                },
                Err(_) => ComposedClass::Overflow,
            }
        };
        self.memo[idx] = Some(class);
        class
    }

    /// The field-type walk for [`classify`](Self::classify): returns `ft`
    /// with fixed targets stamped, clearing `fixed` on anything the
    /// resolver classifies variable. A ref to an `Overflow` target also
    /// clears it — conservative for THIS pass only: the fixpoint drops
    /// the target and re-probes, whereupon the ref is unresolvable and
    /// the parent classifies exactly as the resolver will see it.
    fn probe_field_type(
        &mut self,
        ft: &FieldType,
        parent_package: &Option<String>,
        fixed: &mut bool,
    ) -> FieldType {
        match ft {
            FieldType::String | FieldType::Bytes | FieldType::DynamicArray { .. } => {
                *fixed = false;
                ft.clone()
            }
            FieldType::FixedArray {
                element_type,
                length,
            } => {
                let element = self.probe_field_type(element_type, parent_package, fixed);
                FieldType::FixedArray {
                    element_type: Box::new(element),
                    length: *length,
                }
            }
            FieldType::Nested {
                schema_name,
                package,
                ..
            } => match self.lookup(package, schema_name, parent_package) {
                Some(target_idx) => match self.classify(target_idx) {
                    ComposedClass::Fixed { size, align } => FieldType::Nested {
                        schema_name: schema_name.clone(),
                        package: package.clone(),
                        fixed: Some(NestedFixedInfo {
                            has_large_array: false,
                            fixed_size: size,
                            alignment: align,
                            target_hash: 0,
                        }),
                    },
                    ComposedClass::NotInlinable | ComposedClass::Overflow => {
                        *fixed = false;
                        ft.clone()
                    }
                },
                None => {
                    *fixed = false;
                    ft.clone()
                }
            },
            // Primitives + StringFixed: fixed, contribute their own size.
            _ => ft.clone(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::codegen::schema::{FieldDef, MessageSchema};

    fn nested(name: &str, package: Option<&str>) -> FieldType {
        FieldType::Nested {
            schema_name: name.to_string(),
            package: package.map(|s| s.to_string()),
            fixed: None,
        }
    }

    fn point_schema(pkg: &str) -> MessageSchema {
        let mut s = MessageSchema::new_in_package("Point", pkg);
        s.add_field(FieldDef::new("x", FieldType::F64));
        s.add_field(FieldDef::new("y", FieldType::F64));
        s.add_field(FieldDef::new("z", FieldType::F64));
        s
    }

    #[test]
    fn all_fixed_nested_chain_resolves() {
        // Pose { position: Point, orientation: Quaternion } — all fixed.
        let mut quat = MessageSchema::new_in_package("Quaternion", "geometry_msgs");
        for f in ["x", "y", "z", "w"] {
            quat.add_field(FieldDef::new(f, FieldType::F64));
        }
        let mut pose = MessageSchema::new_in_package("Pose", "geometry_msgs");
        pose.add_field(FieldDef::new("position", nested("Point", None)));
        pose.add_field(FieldDef::new(
            "orientation",
            nested("Quaternion", Some("geometry_msgs")),
        ));

        let mut schemas = vec![point_schema("geometry_msgs"), quat, pose];
        let warnings = resolve_fixed_nested(&mut schemas);
        assert!(warnings.is_empty(), "unexpected warnings: {warnings:?}");

        let pose = &schemas[2];
        assert!(pose.is_definitely_fixed(), "Pose must resolve to fixed");
        assert!(!pose.has_variable_fields());
        assert_eq!(pose.variable_field_count(), 0);
    }

    #[test]
    fn variable_contamination_propagates() {
        // Header { stamp: Time, frame_id: string } is variable;
        // PoseStamped { header: Header, pose: Pose } stays variable but
        // pose resolves fixed.
        let mut time = MessageSchema::new_in_package("Time", "builtin_interfaces");
        time.add_field(FieldDef::new("sec", FieldType::I32));
        time.add_field(FieldDef::new("nanosec", FieldType::U32));

        let mut header = MessageSchema::new_in_package("Header", "std_msgs");
        header.add_field(FieldDef::new(
            "stamp",
            nested("Time", Some("builtin_interfaces")),
        ));
        header.add_field(FieldDef::new("frame_id", FieldType::String));

        let mut pose = MessageSchema::new_in_package("Pose", "geometry_msgs");
        pose.add_field(FieldDef::new(
            "position",
            nested("Point", Some("geometry_msgs")),
        ));

        let mut pose_stamped = MessageSchema::new_in_package("PoseStamped", "geometry_msgs");
        pose_stamped.add_field(FieldDef::new("header", nested("Header", Some("std_msgs"))));
        pose_stamped.add_field(FieldDef::new("pose", nested("Pose", Some("geometry_msgs"))));

        let mut schemas = vec![
            time,
            header,
            point_schema("geometry_msgs"),
            pose,
            pose_stamped,
        ];
        let warnings = resolve_fixed_nested(&mut schemas);
        assert!(warnings.is_empty(), "unexpected warnings: {warnings:?}");

        // Header: stamp resolved fixed, but frame_id keeps it variable.
        let header = &schemas[1];
        assert!(!header.is_definitely_fixed());
        assert!(matches!(
            &header.fields[0].field_type,
            FieldType::Nested { fixed: Some(_), .. }
        ));
        assert_eq!(
            header.variable_field_count(),
            1,
            "only frame_id stays variable once stamp resolves fixed"
        );

        // PoseStamped: header variable, pose fixed → 1 variable field.
        let ps = &schemas[4];
        assert!(!ps.is_definitely_fixed());
        assert!(matches!(
            &ps.fields[0].field_type,
            FieldType::Nested { fixed: None, .. }
        ));
        assert!(matches!(
            &ps.fields[1].field_type,
            FieldType::Nested { fixed: Some(_), .. }
        ));
        assert_eq!(ps.variable_field_count(), 1);
    }

    #[test]
    fn unknown_target_warns_and_stays_variable() {
        let mut s = MessageSchema::new_in_package("Foo", "test_msgs");
        s.add_field(FieldDef::new("mystery", nested("DoesNotExist", None)));

        let mut schemas = vec![s];
        let warnings = resolve_fixed_nested(&mut schemas);
        assert_eq!(warnings.len(), 1);
        assert!(warnings[0].contains("Foo.mystery"));
        assert!(warnings[0].contains("DoesNotExist"));
        assert!(matches!(
            &schemas[0].fields[0].field_type,
            FieldType::Nested { fixed: None, .. }
        ));
        assert!(!schemas[0].is_definitely_fixed());
    }

    #[test]
    fn ambiguous_bare_reference_warns() {
        // Two packages define `Mesh`; an UNQUALIFIED reference from a third
        // package is ambiguous and must not silently pick one.
        let mut mesh_a = MessageSchema::new_in_package("Mesh", "shape_msgs");
        mesh_a.add_field(FieldDef::new("x", FieldType::F64));
        let mut mesh_b = MessageSchema::new_in_package("Mesh", "other_msgs");
        mesh_b.add_field(FieldDef::new("y", FieldType::F64));

        let mut user = MessageSchema::new_in_package("User", "third_msgs");
        user.add_field(FieldDef::new("mesh", nested("Mesh", None)));
        // A QUALIFIED reference resolves fine.
        user.add_field(FieldDef::new(
            "shape_mesh",
            nested("Mesh", Some("shape_msgs")),
        ));

        let mut schemas = vec![mesh_a, mesh_b, user];
        let warnings = resolve_fixed_nested(&mut schemas);
        assert_eq!(warnings.len(), 1, "warnings: {warnings:?}");
        assert!(warnings[0].contains("ambiguous"));
        assert!(matches!(
            &schemas[2].fields[0].field_type,
            FieldType::Nested { fixed: None, .. }
        ));
        assert!(matches!(
            &schemas[2].fields[1].field_type,
            FieldType::Nested { fixed: Some(_), .. }
        ));
    }

    #[test]
    fn cycle_detected_and_conservative() {
        // A → B → A. Cannot occur in real .msg files; the resolver must
        // terminate, warn, and classify both as variable.
        let mut a = MessageSchema::new_in_package("A", "test_msgs");
        a.add_field(FieldDef::new("b", nested("B", Some("test_msgs"))));
        let mut b = MessageSchema::new_in_package("B", "test_msgs");
        b.add_field(FieldDef::new("a", nested("A", Some("test_msgs"))));

        let mut schemas = vec![a, b];
        let warnings = resolve_fixed_nested(&mut schemas);
        assert!(
            warnings.iter().any(|w| w.contains("cycle")),
            "expected a cycle warning, got: {warnings:?}"
        );
        assert!(!schemas[0].is_definitely_fixed());
        assert!(!schemas[1].is_definitely_fixed());
    }

    #[test]
    fn fixed_array_of_fixed_nested_resolves() {
        let mut s = MessageSchema::new_in_package("Corners", "test_msgs");
        s.add_field(FieldDef::new(
            "corners",
            FieldType::FixedArray {
                element_type: Box::new(nested("Point", Some("geometry_msgs"))),
                length: 4,
            },
        ));

        let mut schemas = vec![point_schema("geometry_msgs"), s];
        let warnings = resolve_fixed_nested(&mut schemas);
        assert!(warnings.is_empty(), "unexpected warnings: {warnings:?}");
        assert!(schemas[1].is_definitely_fixed());
    }

    #[test]
    fn dynamic_array_of_fixed_nested_stays_variable() {
        // Pose[] poses — the ARRAY is variable even though Pose is fixed.
        let mut s = MessageSchema::new_in_package("PoseList", "test_msgs");
        s.add_field(FieldDef::new(
            "poses",
            FieldType::DynamicArray {
                element_type: Box::new(nested("Point", Some("geometry_msgs"))),
            },
        ));

        let mut schemas = vec![point_schema("geometry_msgs"), s];
        let warnings = resolve_fixed_nested(&mut schemas);
        assert!(warnings.is_empty(), "unexpected warnings: {warnings:?}");
        assert!(!schemas[1].is_definitely_fixed());
        assert_eq!(schemas[1].variable_field_count(), 1);
        // Element reference NOT rewritten (array stays opaque).
        if let FieldType::DynamicArray { element_type } = &schemas[1].fields[0].field_type {
            assert!(matches!(
                element_type.as_ref(),
                FieldType::Nested { fixed: None, .. }
            ));
        } else {
            panic!("expected DynamicArray");
        }
    }

    #[test]
    fn transitive_large_array_propagates() {
        // PoseWithCovariance { pose: Pose, covariance: f64[36] } is fixed
        // WITH a large array; a parent embedding it must inherit
        // has_large_array so derives are computed correctly.
        let mut pwc = MessageSchema::new_in_package("PoseWithCovariance", "geometry_msgs");
        pwc.add_field(FieldDef::new(
            "pose",
            nested("Point", Some("geometry_msgs")),
        ));
        pwc.add_field(FieldDef::new(
            "covariance",
            FieldType::FixedArray {
                element_type: Box::new(FieldType::F64),
                length: 36,
            },
        ));

        let mut parent = MessageSchema::new_in_package("Parent", "test_msgs");
        parent.add_field(FieldDef::new(
            "pwc",
            nested("PoseWithCovariance", Some("geometry_msgs")),
        ));

        let mut schemas = vec![point_schema("geometry_msgs"), pwc, parent];
        let warnings = resolve_fixed_nested(&mut schemas);
        assert!(warnings.is_empty(), "unexpected warnings: {warnings:?}");

        assert!(schemas[2].is_definitely_fixed());
        assert!(matches!(
            &schemas[2].fields[0].field_type,
            FieldType::Nested {
                fixed: Some(NestedFixedInfo {
                    has_large_array: true,
                    ..
                }),
                ..
            }
        ));
    }

    /// An unambiguous cross-package bare binding resolves
    /// (pragmatism) but ALWAYS warns — rosidl would reject it, and the
    /// binding silently flips if a second package later defines the name.
    #[test]
    fn cross_package_bare_binding_resolves_with_loud_warning() {
        let mut user = MessageSchema::new_in_package("User", "third_msgs");
        user.add_field(FieldDef::new("p", nested("Point", None)));

        let mut schemas = vec![point_schema("geometry_msgs"), user];
        let warnings = resolve_fixed_nested(&mut schemas);
        assert_eq!(warnings.len(), 1, "warnings: {warnings:?}");
        assert!(warnings[0].contains("CROSS-PACKAGE"));
        assert!(warnings[0].contains("geometry_msgs/Point"));
        // Resolution still proceeds — the field resolves fixed.
        assert!(matches!(
            &schemas[1].fields[0].field_type,
            FieldType::Nested { fixed: Some(_), .. }
        ));
    }

    /// Bare `Header` means `std_msgs/Header` (the single
    /// rosidl legacy special case) and resolves SILENTLY — used
    /// unqualified by moveit_msgs and friends. A package's own Header
    /// still wins (same-package lookup runs first).
    #[test]
    fn bare_header_resolves_to_std_msgs_silently() {
        let mut std_header = MessageSchema::new_in_package("Header", "std_msgs");
        std_header.add_field(FieldDef::new("frame_id", FieldType::String));

        let mut user = MessageSchema::new_in_package("DisplayThing", "moveit_msgs");
        user.add_field(FieldDef::new("header", nested("Header", None)));

        let mut schemas = vec![std_header, user];
        let warnings = resolve_fixed_nested(&mut schemas);
        assert!(
            warnings.is_empty(),
            "bare Header → std_msgs must be silent: {warnings:?}"
        );

        // Local Header wins over the special case.
        let mut local_header = MessageSchema::new_in_package("Header", "my_msgs");
        local_header.add_field(FieldDef::new("seq", FieldType::U32)); // FIXED
        let mut std_header = MessageSchema::new_in_package("Header", "std_msgs");
        std_header.add_field(FieldDef::new("frame_id", FieldType::String)); // variable

        let mut user = MessageSchema::new_in_package("Local", "my_msgs");
        user.add_field(FieldDef::new("header", nested("Header", None)));

        let mut schemas = vec![local_header, std_header, user];
        let warnings = resolve_fixed_nested(&mut schemas);
        assert!(warnings.is_empty(), "{warnings:?}");
        // Resolved against the LOCAL (fixed) Header, not std_msgs'
        // variable one — proof: the reference resolved fixed.
        assert!(matches!(
            &schemas[2].fields[0].field_type,
            FieldType::Nested { fixed: Some(_), .. }
        ));
    }

    /// Duplicate `(pkg, name)` definitions warn once and
    /// bind "later wins" on BOTH the qualified and bare paths — a bare
    /// reference must NOT report "ambiguous (defined in 2 packages)".
    #[test]
    fn duplicate_definitions_bind_later_on_both_paths() {
        let mut v1 = MessageSchema::new_in_package("X", "p");
        v1.add_field(FieldDef::new("s", FieldType::String)); // variable
        let mut v2 = MessageSchema::new_in_package("X", "p");
        v2.add_field(FieldDef::new("n", FieldType::F64)); // FIXED — later wins

        let mut user = MessageSchema::new_in_package("User", "other");
        user.add_field(FieldDef::new("bare", nested("X", None)));
        user.add_field(FieldDef::new("qual", nested("X", Some("p"))));

        let mut schemas = vec![v1, v2, user];
        let warnings = resolve_fixed_nested(&mut schemas);
        assert!(
            warnings.iter().any(|w| w.contains("duplicate")),
            "{warnings:?}"
        );
        assert!(
            !warnings.iter().any(|w| w.contains("ambiguous")),
            "same-key duplicate must not be diagnosed as cross-package ambiguity: {warnings:?}"
        );
        // BOTH paths bound the LATER (fixed) definition.
        assert!(matches!(
            &schemas[2].fields[0].field_type,
            FieldType::Nested { fixed: Some(_), .. }
        ));
        assert!(matches!(
            &schemas[2].fields[1].field_type,
            FieldType::Nested { fixed: Some(_), .. }
        ));
    }

    /// A typo'd `DynamicArray` element type warns (it
    /// would otherwise get zero diagnostics anywhere); layout is unaffected
    /// (the array is variable regardless).
    #[test]
    fn dynamic_array_element_typo_warns() {
        let mut s = MessageSchema::new_in_package("List", "test_msgs");
        s.add_field(FieldDef::new(
            "items",
            FieldType::DynamicArray {
                element_type: Box::new(nested("Typo", None)),
            },
        ));

        let mut schemas = vec![s];
        let warnings = resolve_fixed_nested(&mut schemas);
        assert_eq!(warnings.len(), 1, "{warnings:?}");
        assert!(warnings[0].contains("array element"));
        assert!(warnings[0].contains("Typo"));
    }

    /// A stale `fixed: Some(_)` flag from a prior resolution
    /// against a fuller schema set must be CLEARED when re-resolution
    /// cannot find (or no longer classifies as fixed) the target —
    /// otherwise the layout path would route the field into the fixed
    /// section with a layout the current set cannot produce.
    #[test]
    fn stale_fixed_flags_cleared_on_reresolution() {
        // Resolve against the full set: Pose.position resolves fixed.
        let mut full = vec![point_schema("geometry_msgs"), {
            let mut pose = MessageSchema::new_in_package("Pose", "geometry_msgs");
            pose.add_field(FieldDef::new(
                "position",
                nested("Point", Some("geometry_msgs")),
            ));
            pose
        }];
        let w = resolve_fixed_nested(&mut full);
        assert!(w.is_empty(), "{w:?}");
        assert!(matches!(
            &full[1].fields[0].field_type,
            FieldType::Nested { fixed: Some(_), .. }
        ));

        // Re-resolve the Pose ALONE (Point missing): the stale flag must
        // be cleared, with a loud warning.
        let mut subset = vec![full[1].clone()];
        let w = resolve_fixed_nested(&mut subset);
        assert_eq!(w.len(), 1, "{w:?}");
        assert!(w[0].contains("unknown schema"));
        assert!(
            matches!(
                &subset[0].fields[0].field_type,
                FieldType::Nested { fixed: None, .. }
            ),
            "stale fixed flag must be cleared on failed re-resolution"
        );
        assert!(!subset[0].is_definitely_fixed());
    }

    /// The companion case: the **Ok + Variable** branch (target still
    /// resolves but its fixedness flipped Fixed → Variable) must ALSO clear
    /// a stale `fixed: Some(_)` — and, unlike the Err path, WITHOUT an
    /// "unknown schema" warning. Guards the clearing `else` arm; a
    /// refactor dropping it would leak a stale `Some` onto a now-variable
    /// field (latent layout corruption), and the Err-path test above would
    /// still pass.
    #[test]
    fn stale_fixed_flag_cleared_when_target_flips_to_variable() {
        // Full set: Point is fixed (3×f64) → Pose.position resolves fixed.
        let mut full = vec![point_schema("geometry_msgs"), {
            let mut pose = MessageSchema::new_in_package("Pose", "geometry_msgs");
            pose.add_field(FieldDef::new(
                "position",
                nested("Point", Some("geometry_msgs")),
            ));
            pose
        }];
        let w = resolve_fixed_nested(&mut full);
        assert!(w.is_empty(), "{w:?}");
        assert!(matches!(
            &full[1].fields[0].field_type,
            FieldType::Nested { fixed: Some(_), .. }
        ));

        // Re-resolve with Point STILL PRESENT but now VARIABLE (a String
        // field makes it variable). The stale `Some` on the cloned Pose must
        // be cleared via the Ok+else branch, with NO "unknown schema" warning
        // (Point resolves — it just is not fixed anymore).
        let mut variable_point = MessageSchema::new_in_package("Point", "geometry_msgs");
        variable_point.add_field(FieldDef::new("label", FieldType::String));
        let mut subset = vec![variable_point, full[1].clone()];
        let w = resolve_fixed_nested(&mut subset);
        assert!(
            w.iter().all(|m| !m.contains("unknown schema")),
            "Ok+Variable path must NOT emit an unknown-schema warning: {w:?}"
        );
        assert!(
            matches!(
                &subset[1].fields[0].field_type,
                FieldType::Nested { fixed: None, .. }
            ),
            "stale fixed flag must be cleared when the target flips to variable"
        );
        assert!(!subset[1].is_definitely_fixed());
    }

    #[test]
    fn resolution_is_deterministic() {
        // Same input twice → identical rewrites + identical warnings.
        let build = || {
            let mut header = MessageSchema::new_in_package("Header", "std_msgs");
            header.add_field(FieldDef::new("frame_id", FieldType::String));
            let mut pose = MessageSchema::new_in_package("Pose", "geometry_msgs");
            pose.add_field(FieldDef::new(
                "position",
                nested("Point", Some("geometry_msgs")),
            ));
            pose.add_field(FieldDef::new("missing", nested("Nope", None)));
            vec![point_schema("geometry_msgs"), header, pose]
        };
        let mut a = build();
        let mut b = build();
        let wa = resolve_fixed_nested(&mut a);
        let wb = resolve_fixed_nested(&mut b);
        assert_eq!(wa, wb);
        for (sa, sb) in a.iter().zip(b.iter()) {
            assert_eq!(sa.fields.len(), sb.fields.len());
            for (fa, fb) in sa.fields.iter().zip(sb.fields.iter()) {
                assert_eq!(fa.field_type, fb.field_type);
            }
        }
    }

    // ---- Recipe 3: recursive nested layout fold (Option D) ----

    /// Build `[Point, Quaternion(quat_fields), Pose{position, orientation}]`,
    /// resolve, and return Pose's recipe-3 `schema_hash()` plus the
    /// [`NestedFixedInfo`] stamped on its `orientation` (fixed Quaternion).
    fn pose_hash_with_quaternion(quat_fields: &[&str]) -> (u64, NestedFixedInfo) {
        let mut quat = MessageSchema::new_in_package("Quaternion", "geometry_msgs");
        for f in quat_fields {
            quat.add_field(FieldDef::new(*f, FieldType::F64));
        }
        let mut pose = MessageSchema::new_in_package("Pose", "geometry_msgs");
        pose.add_field(FieldDef::new(
            "position",
            nested("Point", Some("geometry_msgs")),
        ));
        pose.add_field(FieldDef::new(
            "orientation",
            nested("Quaternion", Some("geometry_msgs")),
        ));
        let mut schemas = vec![point_schema("geometry_msgs"), quat, pose];
        let warnings = resolve_fixed_nested(&mut schemas);
        assert!(warnings.is_empty(), "unexpected warnings: {warnings:?}");
        let pose = &schemas[2];
        let info = match &pose.fields[1].field_type {
            FieldType::Nested { fixed: Some(i), .. } => *i,
            other => panic!("orientation must resolve fixed, got {other:?}"),
        };
        (pose.schema_hash(), info)
    }

    #[test]
    fn fixed_nested_stamps_target_size_alignment_and_hash() {
        let (_, info) = pose_hash_with_quaternion(&["x", "y", "z", "w"]);
        // QuaternionShm = 4 × f64 = 32 bytes, 8-aligned.
        assert_eq!(info.fixed_size, 32, "QuaternionShm is 4×f64 = 32 bytes");
        assert_eq!(info.alignment, 8);
        assert!(!info.has_large_array);
        // The stamped target_hash must equal the standalone target's own
        // recipe-3 hash (single recipe, no drift).
        let mut quat = MessageSchema::new_in_package("Quaternion", "geometry_msgs");
        for f in ["x", "y", "z", "w"] {
            quat.add_field(FieldDef::new(f, FieldType::F64));
        }
        assert_eq!(info.target_hash, quat.schema_hash());
    }

    /// Sanity floor: a nested target gaining a field bumps the parent hash.
    /// NOTE: this property is also satisfied by a size-only scheme (the
    /// extra field grows `wire_fixed_size()`, hashed independently in step 2),
    /// so this test alone does NOT prove the recursive fold — see
    /// `recursive_fold_nested_reorder_bumps_parent_hash` for the Option-D
    /// proof (a SAME-SIZE change the fold catches but a size-only scheme misses).
    #[test]
    fn recursive_fold_nested_size_change_bumps_parent_hash() {
        let (h4, _) = pose_hash_with_quaternion(&["x", "y", "z", "w"]);
        let (h5, _) = pose_hash_with_quaternion(&["x", "y", "z", "w", "extra"]);
        assert_ne!(
            h4, h5,
            "parent hash must change when an inlined nested target gains a field"
        );
    }

    /// A SAME-SIZE field reorder inside an inlined nested target must STILL
    /// bump the parent hash. This is precisely what distinguishes recipe 3
    /// (recursive hash fold, Option D) from a size-only scheme (Option B):
    /// a same-size reorder is invisible to a size-only parent hash but is
    /// caught here — no silent wire corruption.
    #[test]
    fn recursive_fold_nested_reorder_bumps_parent_hash() {
        let (canonical, info_c) = pose_hash_with_quaternion(&["x", "y", "z", "w"]);
        let (reordered, info_r) = pose_hash_with_quaternion(&["w", "x", "y", "z"]);
        // Identical inlined size + alignment (4×f64 either way) …
        assert_eq!(info_c.fixed_size, info_r.fixed_size);
        assert_eq!(info_c.alignment, info_r.alignment);
        // … but the recursive target hash — and therefore the parent hash —
        // differ, because the nested field ORDER changed.
        assert_ne!(info_c.target_hash, info_r.target_hash);
        assert_ne!(
            canonical, reordered,
            "recipe 3 must catch a same-size reorder inside an inlined nested target"
        );
    }

    /// Build `[Point2(fields), Corners { Point2[4] corners }]`, resolve, and
    /// return `Corners`'s recipe-3 hash. Exercises a fixed nested embedded in
    /// a `FixedArray` (inlined as `[Point2Shm; 4]`).
    fn corners_hash_with_point2(point_fields: &[&str]) -> u64 {
        let mut p2 = MessageSchema::new_in_package("Point2", "geometry_msgs");
        for f in point_fields {
            p2.add_field(FieldDef::new(*f, FieldType::F64));
        }
        let mut corners = MessageSchema::new_in_package("Corners", "geometry_msgs");
        corners.add_field(FieldDef::new(
            "corners",
            FieldType::FixedArray {
                element_type: Box::new(nested("Point2", Some("geometry_msgs"))),
                length: 4,
            },
        ));
        let mut schemas = vec![p2, corners];
        let warnings = resolve_fixed_nested(&mut schemas);
        assert!(warnings.is_empty(), "unexpected warnings: {warnings:?}");
        let corners = &schemas[1];
        assert!(
            corners.fields[0].field_type.is_definitely_fixed(),
            "FixedArray of fixed Point2 must be definitely-fixed"
        );
        corners.schema_hash()
    }

    /// A SAME-SIZE reorder inside a fixed nested element of a `FixedArray`
    /// must bump the parent hash. `canonical_str` only carries the element
    /// NAME (`geometry_msgs/Point2[4]`) and `wire_fixed_size` is unchanged
    /// (4×24 bytes either way), so this passes ONLY because the fold recurses
    /// through `FixedArray`. Without it, the array path silently
    /// drops the element layout from the parent hash.
    #[test]
    fn recursive_fold_through_fixed_array_reorder_bumps_parent_hash() {
        let canonical = corners_hash_with_point2(&["x", "y", "z"]);
        let reordered = corners_hash_with_point2(&["z", "x", "y"]);
        assert_ne!(
            canonical, reordered,
            "recipe 3 must catch a same-size reorder inside a FixedArray-of-fixed-nested element"
        );
    }

    /// Proves the bottom-up fold reaches across MORE THAN TWO levels:
    /// `Outer { mid: Mid }`, `Mid { leaf: Leaf }`, `Leaf { f64×3 }`. A
    /// same-size reorder in the level-3 `Leaf` must bump the level-1 `Outer`
    /// hash, through `Mid`'s stamped `target_hash` (which itself folds in
    /// `Leaf`'s hash).
    #[test]
    fn recursive_fold_three_level_chain_reorder_bumps_root_hash() {
        let build = |leaf_fields: &[&str]| -> u64 {
            let mut leaf = MessageSchema::new_in_package("Leaf", "test_msgs");
            for f in leaf_fields {
                leaf.add_field(FieldDef::new(*f, FieldType::F64));
            }
            let mut mid = MessageSchema::new_in_package("Mid", "test_msgs");
            mid.add_field(FieldDef::new("leaf", nested("Leaf", Some("test_msgs"))));
            let mut outer = MessageSchema::new_in_package("Outer", "test_msgs");
            outer.add_field(FieldDef::new("mid", nested("Mid", Some("test_msgs"))));
            let mut schemas = vec![leaf, mid, outer];
            let warnings = resolve_fixed_nested(&mut schemas);
            assert!(warnings.is_empty(), "unexpected warnings: {warnings:?}");
            assert!(schemas[2].fields[0].field_type.is_definitely_fixed());
            schemas[2].schema_hash()
        };
        assert_ne!(
            build(&["a", "b", "c"]),
            build(&["c", "a", "b"]),
            "a same-size reorder in a level-3 leaf must propagate to the level-1 root hash"
        );
    }
}
