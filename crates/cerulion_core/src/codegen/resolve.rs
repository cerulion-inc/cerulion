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
//! 0. Writes the package into each bare reference of a packaged schema that
//!    the own-package or `Header` step of the ladder below binds: a bare
//!    `Type` in package `pkg` becomes `pkg/Type` when the set holds
//!    `pkg/Type`, and a bare `Header` becomes `std_msgs/Header` when the set
//!    holds that and not `pkg/Header`. The reference's spelling is hashed
//!    ([`MessageSchema::schema_hash`]), and `rmw_cerulion` names every nested
//!    member with the package of its rosidl introspection namespace, so a
//!    `.msg` that spells same-package types bare (as upstream ROS files do)
//!    hashes as the ROS 2 type does. This runs before the snapshot the
//!    fixedness and target hashes are computed from, so an inlined target's
//!    folded hash is its qualified one. A reference bound only through the
//!    global bare namespace, or not at all, keeps its spelling.
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
//! 3. Stamps every other bound reference (a variable target, or any
//!    element under a `DynamicArray`) with the target's own resolved
//!    `schema_hash` in `FieldType::Nested::target_hash`, also bottom-up.
//!    The two stamps are exclusive: an inlined target's hash lives in
//!    [`NestedFixedInfo::target_hash`]. A reference inside a reference
//!    cycle (its target reaches back to the parent, at any array depth) is
//!    not stamped: its target's hash would depend on the parent's own. This
//!    rule reads only the reference graph, never the order of the input,
//!    so a schema outside the cycle gets the same stamps in every order.
//!
//! Reference resolution follows rosidl semantics: qualified (`pkg/Msg`)
//! → exact; unqualified → same package first, then the single legacy
//! special case bare `Header` → `std_msgs/Header` (silent), then the
//! global bare namespace iff unambiguous — that last step is something
//! rosidl would REJECT, so it always emits a loud warning.
//!
//! Warnings are produced for: unresolvable references (unknown target,
//! ambiguous bare name), reference cycles (one per reference that is not
//! stamped, a schema's reference to itself excepted), targets whose fixed
//! section cannot be sized, duplicate schema definitions, cross-package
//! bare bindings, and typo'd `DynamicArray` element types.
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

use super::nested_index::{Bound, Miss, NestedIndex};
use super::schema::{FieldType, MessageSchema, NestedFixedInfo};

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
    qualify_bare_references(schemas);
    let mut resolver = Resolver::new(schemas);

    // Force-compute fixedness for every schema (fully populates the memo).
    for idx in 0..resolver.snapshot.len() {
        resolver.fixedness_of(idx);
    }

    // Rewrite phase: mark resolved-fixed Nested references on the live
    // schemas using the memoized fixedness of the pre-rewrite snapshot.
    for (parent_idx, schema) in schemas.iter_mut().enumerate() {
        for field in &mut schema.fields {
            resolver.rewrite_field(&mut field.field_type, &field.name, parent_idx, false);
        }
    }

    resolver.warnings
}

/// Step 0 of [`resolve_fixed_nested`] (see the module docs): for each bare
/// reference of a packaged schema, at any array depth, that
/// [`NestedIndex::bind`] binds without the global bare namespace, write the
/// bound schema's package into the reference. Package-less schemas and
/// package-less targets are left as they are.
fn qualify_bare_references(schemas: &mut [MessageSchema]) {
    let index = NestedIndex::from_schemas(schemas);
    let mut writes = Vec::new();
    for (schema_idx, schema) in schemas.iter().enumerate() {
        let Some(parent_package) = schema.package.as_deref() else {
            continue;
        };
        for (field_idx, field) in schema.fields.iter().enumerate() {
            let FieldType::Nested {
                schema_name,
                package: None,
                ..
            } = element_type_of(&field.field_type)
            else {
                continue;
            };
            if let Ok(Bound {
                idx,
                cross_package_bare: false,
            }) = index.bind(None, schema_name, Some(parent_package))
            {
                if let Some(target_package) = &schemas[idx].package {
                    writes.push((schema_idx, field_idx, target_package.clone()));
                }
            }
        }
    }
    for (schema_idx, field_idx, target_package) in writes {
        if let FieldType::Nested { package, .. } =
            element_type_of_mut(&mut schemas[schema_idx].fields[field_idx].field_type)
        {
            *package = Some(target_package);
        }
    }
}

/// The element type under every array layer of `field_type`.
fn element_type_of(field_type: &FieldType) -> &FieldType {
    match field_type {
        FieldType::FixedArray { element_type, .. } | FieldType::DynamicArray { element_type } => {
            element_type_of(element_type)
        }
        other => other,
    }
}

/// [`element_type_of`], mutable.
fn element_type_of_mut(field_type: &mut FieldType) -> &mut FieldType {
    match field_type {
        FieldType::FixedArray { element_type, .. } | FieldType::DynamicArray { element_type } => {
            element_type_of_mut(element_type)
        }
        other => other,
    }
}

/// Internal state for one resolution run: the immutable pre-rewrite
/// snapshot, lookup registries, the fixedness and hash memos, and
/// accumulated warnings.
struct Resolver {
    /// Pre-rewrite copy of the schemas, so fixedness computation does not
    /// depend on rewrite order.
    snapshot: Vec<MessageSchema>,
    /// Identity registry the references bind through.
    index: NestedIndex,
    /// Memoized fixedness per snapshot index.
    memo: Vec<Option<Fixedness>>,
    /// Memoized fully-resolved layout per snapshot index. Only computed
    /// for recursively-fixed schemas (the only ones that get inlined into a
    /// parent's fixed section). Computed bottom-up; `Err` names the field
    /// whose composed size overflows.
    resolved_info: Vec<Option<Result<ResolvedFixed, String>>>,
    /// Memoized resolved `schema_hash` per snapshot index, fixed or
    /// variable: the value stamped as `target_hash` on a reference that is
    /// not inlined. `Err` when the target's fixed section cannot be sized.
    resolved_hash: Vec<Option<Result<u64, String>>>,
    /// Strongly connected component of each snapshot index in the
    /// reference graph (every nested reference, at any array depth). Two
    /// schemas share a component iff each reaches the other, so a
    /// reference inside one component is a reference cycle.
    component: Vec<usize>,
    /// In-progress DFS markers for cycle detection.
    visiting: Vec<bool>,
    warnings: Vec<String>,
}

/// How one nested reference binds within the set being resolved, at its
/// position in the parent (inside a `DynamicArray` or not).
enum Binding {
    /// The reference names no schema of the set, or an ambiguous one.
    Unbound(String),
    /// A recursively fixed target at an inlinable position (not under a
    /// `DynamicArray`): the field lives in the parent's fixed section.
    Inline(Bound),
    /// A target the parent references but does not inline (a variable
    /// target, or any target under a `DynamicArray`), outside the parent's
    /// reference cycle: its resolved hash is stamped as `target_hash`.
    Referenced(Bound),
    /// A target in the parent's own reference cycle. Its hash would depend
    /// on the parent's, so no `target_hash` is stamped.
    Cyclic(Bound),
}

impl Resolver {
    fn new(schemas: &[MessageSchema]) -> Self {
        let mut warnings = Vec::new();
        let mut index = NestedIndex::default();
        for (idx, schema) in schemas.iter().enumerate() {
            // "Later wins" on every ladder step, the bare namespace included:
            // a double-registered bare name would make an unqualified
            // reference report the wrong ambiguity, and keeping the earlier
            // index would bind bare lookups to a different definition than
            // qualified ones.
            if index.insert(schema, idx).is_some() {
                warnings.push(format!(
                    "duplicate schema definition for '{}' — later definition wins in resolution",
                    schema.qualified_name()
                ));
            }
        }
        let len = schemas.len();
        let component = reference_components(schemas, &index);
        Self {
            snapshot: schemas.to_vec(),
            index,
            memo: vec![None; len],
            resolved_info: vec![None; len],
            resolved_hash: vec![None; len],
            component,
            visiting: vec![false; len],
            warnings,
        }
    }

    /// Resolve a Nested reference to a snapshot index through the shared
    /// identity ladder ([`NestedIndex::bind`]). `parent_package` is the
    /// package of the schema CONTAINING the reference (ROS2 semantics: an
    /// unqualified reference means "same package").
    fn lookup(
        &self,
        reference_pkg: &Option<String>,
        name: &str,
        parent_package: &Option<String>,
    ) -> Result<Bound, String> {
        self.index
            .bind(reference_pkg.as_deref(), name, parent_package.as_deref())
            .map_err(|miss| match (miss, reference_pkg) {
                (Miss::Ambiguous { candidates }, _) => format!(
                    "ambiguous unqualified reference '{name}' (defined in {candidates} packages); qualify as pkg/{name}"
                ),
                (Miss::Unknown, Some(pkg)) => format!("unknown schema '{pkg}/{name}'"),
                (Miss::Unknown, None) => format!("unknown schema '{name}'"),
            })
    }

    /// Bind the reference `package`/`schema_name` held by the snapshot
    /// schema at `parent_idx`. `under_dynamic` is true for a reference
    /// inside a `DynamicArray` (at any depth), which is never inlined.
    ///
    /// Precondition: the fixedness memo is fully populated.
    fn bind(
        &self,
        package: &Option<String>,
        schema_name: &str,
        parent_idx: usize,
        under_dynamic: bool,
    ) -> Binding {
        let found = match self.lookup(package, schema_name, &self.snapshot[parent_idx].package) {
            Ok(found) => found,
            Err(reason) => return Binding::Unbound(reason),
        };
        if !under_dynamic && matches!(self.memo[found.idx], Some(Fixedness::Fixed { .. })) {
            Binding::Inline(found)
        } else if self.component[found.idx] == self.component[parent_idx] {
            Binding::Cyclic(found)
        } else {
            Binding::Referenced(found)
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

    /// The snapshot schema at `idx` with every nested reference stamped
    /// as the rewrite phase stamps it ([`resolve_ft_for_layout`](Self::resolve_ft_for_layout)),
    /// so the `MessageSchema` methods read the true inlined layout and the
    /// recipe lives in exactly one place. Works on a clone: the snapshot
    /// is never mutated (determinism and order-independence).
    fn resolved_clone_of(&mut self, idx: usize) -> Result<MessageSchema, String> {
        let mut resolved = self.snapshot[idx].clone();
        for field in &mut resolved.fields {
            field.field_type = self.resolve_ft_for_layout(&field.field_type, idx, false)?;
        }
        Ok(resolved)
    }

    /// Bottom-up, memoized full layout of the recursively-fixed schema at
    /// `idx`, read off [`resolved_clone_of`](Self::resolved_clone_of)
    /// through the checked size arithmetic. `Err` names the field whose
    /// composed size overflows; the inlining site in
    /// [`rewrite_field`](Self::rewrite_field) panics with it, a hash-only
    /// site degrades to no stamp.
    ///
    /// Precondition: `self.memo[idx]` is `Fixed` (only fixed schemas are
    /// inlined). Fixed schemas are acyclic — a reference cycle is downgraded
    /// to `Variable` by [`fixedness_of`](Self::fixedness_of) — so the
    /// recursion terminates; the memo also prevents recomputation.
    fn resolved_fixed_of(&mut self, idx: usize) -> Result<ResolvedFixed, String> {
        if let Some(r) = &self.resolved_info[idx] {
            return r.clone();
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
        let info = self.resolved_clone_of(idx).and_then(|resolved| {
            Ok(ResolvedFixed {
                wire_fixed_size: resolved.checked_wire_fixed_size()?,
                wire_alignment: resolved.wire_alignment(),
                schema_hash: resolved.checked_schema_hash()?,
                has_large_array,
            })
        });
        self.resolved_info[idx] = Some(info.clone());
        info
    }

    /// Memoized resolved `schema_hash` of the schema at `idx`, fixed or
    /// variable: the value a reference that does not inline the schema
    /// carries as `target_hash`. Uses the checked size arithmetic, so a
    /// hostile definition (a bag's schema doc, a peer-served closure) is an
    /// `Err`, never a panic.
    ///
    /// Terminates: every reference it follows leaves the schema's reference
    /// cycle ([`Binding::Cyclic`] is never followed), so the calls walk the
    /// acyclic graph of components.
    fn resolved_hash_of(&mut self, idx: usize) -> Result<u64, String> {
        if let Some(r) = &self.resolved_hash[idx] {
            return r.clone();
        }
        let hash = if matches!(self.memo[idx], Some(Fixedness::Fixed { .. })) {
            self.resolved_fixed_of(idx).map(|info| info.schema_hash)
        } else {
            self.resolved_clone_of(idx)
                .and_then(|resolved| resolved.checked_schema_hash())
        };
        self.resolved_hash[idx] = Some(hash.clone());
        hash
    }

    /// Return `ft` with every nested reference stamped as
    /// [`rewrite_field`](Self::rewrite_field) stamps it, recursing into
    /// array elements: an inlined target gets `fixed`, a referenced one
    /// `target_hash`, anything else neither. Every arm rebuilds the
    /// reference, so a stamp left by an earlier resolution never survives.
    /// `Err` only when an inlined target's composed size overflows (the
    /// parent then has no fixed section); a hash that cannot be computed
    /// leaves `target_hash` empty, and the rewrite phase warns per reference.
    fn resolve_ft_for_layout(
        &mut self,
        ft: &FieldType,
        parent_idx: usize,
        under_dynamic: bool,
    ) -> Result<FieldType, String> {
        Ok(match ft {
            FieldType::Nested {
                schema_name,
                package,
                ..
            } => {
                let (fixed, target_hash) =
                    match self.bind(package, schema_name, parent_idx, under_dynamic) {
                        Binding::Inline(found) => {
                            (Some(self.resolved_fixed_of(found.idx)?.into()), None)
                        }
                        Binding::Referenced(found) => (None, self.resolved_hash_of(found.idx).ok()),
                        Binding::Unbound(_) | Binding::Cyclic(_) => (None, None),
                    };
                FieldType::Nested {
                    schema_name: schema_name.clone(),
                    package: package.clone(),
                    fixed,
                    target_hash,
                }
            }
            FieldType::FixedArray {
                element_type,
                length,
            } => FieldType::FixedArray {
                element_type: Box::new(self.resolve_ft_for_layout(
                    element_type,
                    parent_idx,
                    under_dynamic,
                )?),
                length: *length,
            },
            FieldType::DynamicArray { element_type } => FieldType::DynamicArray {
                element_type: Box::new(self.resolve_ft_for_layout(
                    element_type,
                    parent_idx,
                    true,
                )?),
            },
            other => other.clone(),
        })
    }

    /// Rewrite one field type of the schema at `parent_idx` in place
    /// (recursing into array elements): set `fixed: Some(info)` on a
    /// reference that inlines a recursively fixed target, `target_hash` on
    /// one that references a target without inlining it, and clear both
    /// everywhere else. Warns on unresolvable references, on references
    /// inside a reference cycle, and on targets that have no hash.
    fn rewrite_field(
        &mut self,
        ft: &mut FieldType,
        field_name: &str,
        parent_idx: usize,
        under_dynamic: bool,
    ) {
        match ft {
            FieldType::Nested {
                schema_name,
                package,
                fixed,
                target_hash,
            } => {
                // Re-resolution is authoritative: every arm writes both
                // stamps, so a stamp from a prior resolution against a
                // different (fuller) schema set never survives. A stale
                // `fixed` would route the field into the fixed section with
                // a layout the current set cannot produce.
                *fixed = None;
                *target_hash = None;
                let parent_name = self.snapshot[parent_idx].name.clone();
                // A DynamicArray element keeps the array's own warning text:
                // its layout is unaffected either way.
                let (prefix, consequence) = if under_dynamic {
                    (
                        "array element: ",
                        "array stays opaque-bytes (layout unaffected)",
                    )
                } else {
                    (
                        "",
                        "field stays conservatively variable (opaque-bytes accessors, not zero-copy)",
                    )
                };
                match self.bind(package, schema_name, parent_idx, under_dynamic) {
                    Binding::Unbound(reason) => {
                        self.warnings.push(format!(
                            "{parent_name}.{field_name}: {prefix}{reason} — {consequence}" // dash-ok
                        ));
                    }
                    Binding::Inline(found) => {
                        self.warn_cross_package_bare(&found, schema_name, field_name, &parent_name);
                        // Bottom-up: nested-of-nested is already populated.
                        // A composed overflow here is the set
                        // `composed_overflow_indices` preflights.
                        match self.resolved_fixed_of(found.idx) {
                            Ok(child) => *fixed = Some(child.into()),
                            Err(e) => panic!("{e}"),
                        }
                    }
                    Binding::Referenced(found) => {
                        if !under_dynamic {
                            self.warn_cross_package_bare(
                                &found,
                                schema_name,
                                field_name,
                                &parent_name,
                            );
                        }
                        match self.resolved_hash_of(found.idx) {
                            Ok(hash) => *target_hash = Some(hash),
                            Err(reason) => {
                                let target = self.snapshot[found.idx].qualified_name();
                                self.warnings.push(format!(
                                    "{parent_name}.{field_name}: {prefix}'{target}' has no wire \
                                     hash ({reason}); the reference carries no target hash"
                                ));
                            }
                        }
                    }
                    Binding::Cyclic(found) => {
                        if !under_dynamic {
                            self.warn_cross_package_bare(
                                &found,
                                schema_name,
                                field_name,
                                &parent_name,
                            );
                        }
                        // A self reference (`Node[] children` on `Node`)
                        // loses nothing: the parent's own layout is already
                        // in its own hash.
                        if found.idx != parent_idx {
                            let parent = self.snapshot[parent_idx].qualified_name();
                            let target = self.snapshot[found.idx].qualified_name();
                            self.warnings.push(format!(
                                "{parent_name}.{field_name}: {prefix}'{target}' is in a reference \
                                 cycle with '{parent}'; the reference carries no target hash, so \
                                 a layout change in '{target}' does not change the hash of \
                                 '{parent}'"
                            ));
                        }
                    }
                }
            }
            FieldType::FixedArray { element_type, .. } => {
                self.rewrite_field(element_type, field_name, parent_idx, under_dynamic);
            }
            FieldType::DynamicArray { element_type } => {
                // The array lives in the variable payload whatever its
                // element is, so the element is never inlined; it is
                // stamped with the target's hash like any other reference.
                self.rewrite_field(element_type, field_name, parent_idx, true);
            }
            _ => {}
        }
    }

    /// rosidl would reject a cross-package bare reference (unqualified
    /// means same-package, `Header` excepted). It resolves, but never
    /// silently: the binding can flip when a second package later defines
    /// the same bare name.
    fn warn_cross_package_bare(
        &mut self,
        found: &Bound,
        schema_name: &str,
        field_name: &str,
        parent_name: &str,
    ) {
        if found.cross_package_bare {
            let target = self.snapshot[found.idx].qualified_name();
            self.warnings.push(format!(
                "{parent_name}.{field_name}: unqualified reference '{schema_name}' resolved CROSS-PACKAGE to '{target}' — rosidl would reject this; qualify the reference in the .msg file" // dash-ok
            ));
        }
    }
}

/// The strongly connected component of every schema in the reference
/// graph: an edge `S -> T` for each nested reference of `S`, at any array
/// depth, that binds to `T` through `index`. Iterative Tarjan, so a deep
/// chain cannot overflow the stack; components are numbered in the order
/// the walk closes them, which depends only on the input order, and only
/// equality of two numbers is ever read.
fn reference_components(schemas: &[MessageSchema], index: &NestedIndex) -> Vec<usize> {
    let edges: Vec<Vec<usize>> = schemas
        .iter()
        .map(|schema| {
            schema
                .fields
                .iter()
                .filter_map(|field| match element_type_of(&field.field_type) {
                    FieldType::Nested {
                        schema_name,
                        package,
                        ..
                    } => index
                        .bind(package.as_deref(), schema_name, schema.package.as_deref())
                        .ok()
                        .map(|bound| bound.idx),
                    _ => None,
                })
                .collect()
        })
        .collect();

    let len = schemas.len();
    let mut order: Vec<Option<usize>> = vec![None; len];
    let mut low = vec![0usize; len];
    let mut on_stack = vec![false; len];
    let mut stack = Vec::new();
    let mut component = vec![0usize; len];
    let mut next_order = 0usize;
    let mut next_component = 0usize;
    for root in 0..len {
        if order[root].is_some() {
            continue;
        }
        // (node, index of the next outgoing edge to visit)
        let mut calls = vec![(root, 0usize)];
        order[root] = Some(next_order);
        low[root] = next_order;
        next_order += 1;
        stack.push(root);
        on_stack[root] = true;
        while let Some(&(node, edge)) = calls.last() {
            if let Some(&target) = edges[node].get(edge) {
                if let Some(call) = calls.last_mut() {
                    call.1 += 1;
                }
                match order[target] {
                    None => {
                        order[target] = Some(next_order);
                        low[target] = next_order;
                        next_order += 1;
                        stack.push(target);
                        on_stack[target] = true;
                        calls.push((target, 0));
                    }
                    Some(target_order) if on_stack[target] => {
                        low[node] = low[node].min(target_order);
                    }
                    Some(_) => {}
                }
                continue;
            }
            calls.pop();
            if let Some(&(caller, _)) = calls.last() {
                low[caller] = low[caller].min(low[node]);
            }
            if Some(low[node]) == order[node] {
                while let Some(member) = stack.pop() {
                    on_stack[member] = false;
                    component[member] = next_component;
                    if member == node {
                        break;
                    }
                }
                next_component += 1;
            }
        }
    }
    component
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
    // The resolver's own registry, so the probe binds exactly what it binds.
    let index = NestedIndex::from_schemas(schemas);
    let active: Vec<bool> = schemas
        .iter()
        .enumerate()
        .map(|(idx, schema)| index.is_active(schema, idx))
        .collect();
    let mut probe = ComposedSizeProbe {
        schemas,
        index,
        memo: vec![None; schemas.len()],
        visiting: vec![false; schemas.len()],
    };
    (0..schemas.len())
        .filter(|&idx| active[idx])
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
    index: NestedIndex,
    memo: Vec<Option<ComposedClass>>,
    visiting: Vec<bool>,
}

impl ComposedSizeProbe<'_> {
    /// The resolver's nested-reference binding ([`NestedIndex::bind`]);
    /// `None` for unknown AND ambiguous (both classify the referencing field
    /// variable, exactly as the resolver's `Err` arm).
    fn lookup(
        &self,
        reference_pkg: &Option<String>,
        name: &str,
        parent_package: &Option<String>,
    ) -> Option<usize> {
        self.index
            .bind(reference_pkg.as_deref(), name, parent_package.as_deref())
            .ok()
            .map(|bound| bound.idx)
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
                        target_hash: None,
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
            target_hash: None,
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
    fn resolution_writes_the_package_the_own_package_or_header_step_binds() {
        let mut header = MessageSchema::new_in_package("Header", "std_msgs");
        header.add_field(FieldDef::new("frame_id", FieldType::String));
        let mut own_header = MessageSchema::new_in_package("Header", "own");
        own_header.add_field(FieldDef::new("seq", FieldType::U32));
        let mut other = MessageSchema::new_in_package("Other", "elsewhere");
        other.add_field(FieldDef::new("n", FieldType::U8));
        let mut reading = MessageSchema::new_in_package("Reading", "demo");
        reading.add_field(FieldDef::new("header", nested("Header", None)));
        reading.add_field(FieldDef::new("point", nested("Point", None)));
        reading.add_field(FieldDef::new(
            "points",
            FieldType::FixedArray {
                element_type: Box::new(nested("Point", None)),
                length: 2,
            },
        ));
        reading.add_field(FieldDef::new("other", nested("Other", None)));
        reading.add_field(FieldDef::new("missing", nested("Missing", None)));
        let mut tagged = MessageSchema::new_in_package("Tagged", "own");
        tagged.add_field(FieldDef::new("header", nested("Header", None)));
        let mut loose = MessageSchema::new("Loose");
        loose.add_field(FieldDef::new("point", nested("Point", None)));
        loose.add_field(FieldDef::new("header", nested("Header", None)));

        let mut schemas = vec![
            header,
            own_header,
            other,
            point_schema("demo"),
            reading,
            tagged,
            loose,
        ];
        let warnings = resolve_fixed_nested(&mut schemas);
        let package_of = |schema: &MessageSchema, field: &str| {
            let field = schema.fields.iter().find(|f| f.name == field).unwrap();
            let mut field_type = &field.field_type;
            while let FieldType::FixedArray { element_type, .. }
            | FieldType::DynamicArray { element_type } = field_type
            {
                field_type = element_type;
            }
            match field_type {
                FieldType::Nested { package, .. } => package.clone(),
                other => panic!("{other:?}"),
            }
        };
        let reading = &schemas[4];
        assert_eq!(package_of(reading, "header").as_deref(), Some("std_msgs"));
        assert_eq!(package_of(reading, "point").as_deref(), Some("demo"));
        assert_eq!(package_of(reading, "points").as_deref(), Some("demo"));
        assert_eq!(
            package_of(reading, "other"),
            None,
            "a cross-package bare binding keeps its spelling"
        );
        assert_eq!(package_of(reading, "missing"), None);
        assert_eq!(
            package_of(&schemas[5], "header").as_deref(),
            Some("own"),
            "a package's own Header comes first"
        );
        assert_eq!(
            package_of(&schemas[6], "point"),
            None,
            "a package-less schema's reference keeps its spelling"
        );
        assert_eq!(
            package_of(&schemas[6], "header"),
            None,
            "a package-less schema's bare Header keeps its spelling"
        );
        assert!(
            warnings
                .iter()
                .any(|w| w.contains("Reading.other") && w.contains("CROSS-PACKAGE")),
            "{warnings:?}"
        );
    }

    /// `Window { Span span }`, `Span { Level lo, Level hi }` and
    /// `Level { uint8 value }`, spelled bare in package `demo`, hashed by ONE
    /// resolution pass (a `LayoutResolver` over the parsed text). `Span` is
    /// fixed, so its hash is computed from the resolver's snapshot and folded
    /// into `Window`'s; it equals the hash of the same types with every nested
    /// member named `demo/...`, as `rmw_cerulion` builds them, only when the
    /// snapshot already holds the qualified `Span`.
    #[test]
    fn one_resolution_pass_folds_a_fixed_target_with_its_references_qualified() {
        use crate::codegen::layout::LayoutResolver;
        use crate::codegen::parse_rosmsg;
        let window_hash = |schemas: Vec<MessageSchema>| {
            let (mut resolver, warnings) = LayoutResolver::new(schemas);
            assert!(warnings.is_empty(), "{warnings:?}");
            resolver.layout_of("demo/Window").unwrap().schema_hash
        };
        let parsed = vec![
            parse_rosmsg("uint8 value\n", "Level", Some("demo")).unwrap(),
            parse_rosmsg("Level lo\nLevel hi\n", "Span", Some("demo")).unwrap(),
            parse_rosmsg("Span span\n", "Window", Some("demo")).unwrap(),
        ];
        let mut level = MessageSchema::new_in_package("Level", "demo");
        level.add_field(FieldDef::new("value", FieldType::U8));
        let mut span = MessageSchema::new_in_package("Span", "demo");
        span.add_field(FieldDef::new("lo", nested("Level", Some("demo"))));
        span.add_field(FieldDef::new("hi", nested("Level", Some("demo"))));
        let mut window = MessageSchema::new_in_package("Window", "demo");
        window.add_field(FieldDef::new("span", nested("Span", Some("demo"))));
        assert_eq!(window_hash(parsed), window_hash(vec![level, span, window]));
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
        // The element is never inlined (the array stays opaque); it carries
        // the target's hash instead.
        let point = recipe3_hash(
            "geometry_msgs/Point",
            24,
            &[
                ("x", "float64", None),
                ("y", "float64", None),
                ("z", "float64", None),
            ],
        );
        assert_eq!(
            schemas[1].fields[0].field_type,
            FieldType::DynamicArray {
                element_type: Box::new(stamped("Point", "geometry_msgs", Some(point))),
            }
        );
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

    // ---- Target hash stamps on references that are not inlined ----

    /// The documented recipe-3 byte stream (`MessageSchema::schema_hash`),
    /// assembled by hand from literals: the length-prefixed qualified name,
    /// the fixed-section size, then per field its length-prefixed name and
    /// canonical type and, for an inlined target only, that target's hash.
    /// Never calls into `MessageSchema`.
    fn recipe3_hash(qname: &str, fixed_size: u64, fields: &[(&str, &str, Option<u64>)]) -> u64 {
        fn prefixed(bytes: &mut Vec<u8>, text: &str) {
            bytes.extend_from_slice(&(text.len() as u64).to_le_bytes());
            bytes.extend_from_slice(text.as_bytes());
        }
        let mut bytes = Vec::new();
        prefixed(&mut bytes, qname);
        bytes.extend_from_slice(&fixed_size.to_le_bytes());
        for (name, ty, inlined) in fields {
            prefixed(&mut bytes, name);
            prefixed(&mut bytes, ty);
            if let Some(hash) = inlined {
                bytes.extend_from_slice(&hash.to_le_bytes());
            }
        }
        crate::wire::fnv1a_hash(&bytes)
    }

    /// A qualified reference as the rewrite leaves a non-inlined one.
    fn stamped(name: &str, package: &str, target_hash: Option<u64>) -> FieldType {
        FieldType::Nested {
            schema_name: name.to_string(),
            package: Some(package.to_string()),
            fixed: None,
            target_hash,
        }
    }

    fn dynamic(element: FieldType) -> FieldType {
        FieldType::DynamicArray {
            element_type: Box::new(element),
        }
    }

    /// `demo/Inner { string label; <id_type> id }`.
    fn inner(id_type: FieldType) -> MessageSchema {
        let mut s = MessageSchema::new_in_package("Inner", "demo");
        s.add_field(FieldDef::new("label", FieldType::String));
        s.add_field(FieldDef::new("id", id_type));
        s
    }

    /// `demo/Outer { demo/Inner inner; uint8 tag }`.
    fn outer() -> MessageSchema {
        let mut s = MessageSchema::new_in_package("Outer", "demo");
        s.add_field(FieldDef::new("inner", nested("Inner", Some("demo"))));
        s.add_field(FieldDef::new("tag", FieldType::U8));
        s
    }

    fn inner_hash(id_type: &str) -> u64 {
        recipe3_hash(
            "demo/Inner",
            4,
            &[("label", "string", None), ("id", id_type, None)],
        )
    }

    /// `demo/Outer`'s recipe-3 hash, from an implementation outside this
    /// repository: both `Inner` definitions give it.
    const OUTER_RECIPE3_HASH: u64 = 0x2ab5685f302463b2;

    #[test]
    fn a_variable_target_reference_is_stamped_with_the_target_hash_and_moves_no_hash() {
        assert_eq!(
            recipe3_hash(
                "demo/Outer",
                1,
                &[("inner", "demo/Inner", None), ("tag", "uint8", None)]
            ),
            OUTER_RECIPE3_HASH,
            "the hand-assembled stream must reproduce the outside value"
        );
        assert_ne!(inner_hash("uint32"), inner_hash("float32"));
        for (id_type, id_name) in [(FieldType::U32, "uint32"), (FieldType::F32, "float32")] {
            let mut schemas = vec![inner(id_type), outer()];
            let warnings = resolve_fixed_nested(&mut schemas);
            assert!(warnings.is_empty(), "{warnings:?}");
            assert_eq!(
                schemas[1].fields[0].field_type,
                stamped("Inner", "demo", Some(inner_hash(id_name))),
                "Inner {{ {id_name} id }}"
            );
            assert_eq!(schemas[1].schema_hash(), OUTER_RECIPE3_HASH);
        }
    }

    /// `builtin_interfaces/Time`, `std_msgs/Header`, `geometry_msgs/Point`,
    /// `Quaternion`, `Pose` (fields in `pose_fields` order), `PoseArray`
    /// and `PoseStamped`, spelled as the vendored `.msg` files spell them.
    fn pose_family(pose_fields: [(&str, &str); 2]) -> Vec<MessageSchema> {
        let mut time = MessageSchema::new_in_package("Time", "builtin_interfaces");
        time.add_field(FieldDef::new("sec", FieldType::I32));
        time.add_field(FieldDef::new("nanosec", FieldType::U32));
        let mut header = MessageSchema::new_in_package("Header", "std_msgs");
        header.add_field(FieldDef::new(
            "stamp",
            nested("Time", Some("builtin_interfaces")),
        ));
        header.add_field(FieldDef::new("frame_id", FieldType::String));
        let mut quat = MessageSchema::new_in_package("Quaternion", "geometry_msgs");
        for f in ["x", "y", "z", "w"] {
            quat.add_field(FieldDef::new(f, FieldType::F64));
        }
        let mut pose = MessageSchema::new_in_package("Pose", "geometry_msgs");
        for (name, ty) in pose_fields {
            pose.add_field(FieldDef::new(name, nested(ty, Some("geometry_msgs"))));
        }
        let mut pose_array = MessageSchema::new_in_package("PoseArray", "geometry_msgs");
        pose_array.add_field(FieldDef::new("header", nested("Header", Some("std_msgs"))));
        pose_array.add_field(FieldDef::new(
            "poses",
            dynamic(nested("Pose", Some("geometry_msgs"))),
        ));
        let mut pose_stamped = MessageSchema::new_in_package("PoseStamped", "geometry_msgs");
        pose_stamped.add_field(FieldDef::new("header", nested("Header", Some("std_msgs"))));
        pose_stamped.add_field(FieldDef::new("pose", nested("Pose", Some("geometry_msgs"))));
        vec![
            time,
            header,
            point_schema("geometry_msgs"),
            quat,
            pose,
            pose_array,
            pose_stamped,
        ]
    }

    const POSITION_FIRST: [(&str, &str); 2] =
        [("position", "Point"), ("orientation", "Quaternion")];
    const ORIENTATION_FIRST: [(&str, &str); 2] =
        [("orientation", "Quaternion"), ("position", "Point")];
    /// Read off the wire from a stock ROS 2 Jazzy publisher.
    const POSE_WIRE_HASH: u64 = 0x8db061f3f9baff7d;
    /// Read off the wire from a stock ROS 2 Jazzy publisher.
    const POSE_STAMPED_WIRE_HASH: u64 = 0x7bae8da2598b0993;
    /// `geometry_msgs/PoseArray`'s recipe-3 hash, from an implementation
    /// outside this repository: both `Pose` field orders give it.
    const POSE_ARRAY_RECIPE3_HASH: u64 = 0x26b786e1cb2b91bc;

    fn header_hash() -> u64 {
        let time = recipe3_hash(
            "builtin_interfaces/Time",
            8,
            &[("sec", "int32", None), ("nanosec", "uint32", None)],
        );
        recipe3_hash(
            "std_msgs/Header",
            8,
            &[
                ("stamp", "builtin_interfaces/Time", Some(time)),
                ("frame_id", "string", None),
            ],
        )
    }

    fn pose_hash(pose_fields: [(&str, &str); 2]) -> u64 {
        let point = recipe3_hash(
            "geometry_msgs/Point",
            24,
            &[
                ("x", "float64", None),
                ("y", "float64", None),
                ("z", "float64", None),
            ],
        );
        let quat = recipe3_hash(
            "geometry_msgs/Quaternion",
            32,
            &[
                ("x", "float64", None),
                ("y", "float64", None),
                ("z", "float64", None),
                ("w", "float64", None),
            ],
        );
        let fields: Vec<(&str, String, Option<u64>)> = pose_fields
            .iter()
            .map(|(name, ty)| {
                let hash = if *ty == "Point" { point } else { quat };
                (*name, format!("geometry_msgs/{ty}"), Some(hash))
            })
            .collect();
        let fields: Vec<(&str, &str, Option<u64>)> = fields
            .iter()
            .map(|(name, ty, hash)| (*name, ty.as_str(), *hash))
            .collect();
        recipe3_hash("geometry_msgs/Pose", 56, &fields)
    }

    #[test]
    fn a_dynamic_array_element_is_stamped_with_the_target_hash_and_never_inlined() {
        assert_eq!(pose_hash(POSITION_FIRST), POSE_WIRE_HASH);
        assert_ne!(pose_hash(ORIENTATION_FIRST), POSE_WIRE_HASH);
        for pose_fields in [POSITION_FIRST, ORIENTATION_FIRST] {
            let mut schemas = pose_family(pose_fields);
            let warnings = resolve_fixed_nested(&mut schemas);
            assert!(warnings.is_empty(), "{warnings:?}");
            let pose_array = &schemas[5];
            assert_eq!(
                pose_array.fields[0].field_type,
                stamped("Header", "std_msgs", Some(header_hash()))
            );
            assert_eq!(
                pose_array.fields[1].field_type,
                dynamic(stamped(
                    "Pose",
                    "geometry_msgs",
                    Some(pose_hash(pose_fields))
                ))
            );
            assert_eq!(pose_array.schema_hash(), POSE_ARRAY_RECIPE3_HASH);
        }
    }

    #[test]
    fn an_inlined_target_keeps_its_hash_in_the_fixed_stamp_only() {
        let mut schemas = pose_family(POSITION_FIRST);
        let warnings = resolve_fixed_nested(&mut schemas);
        assert!(warnings.is_empty(), "{warnings:?}");
        let pose_stamped = &schemas[6];
        assert_eq!(
            pose_stamped.fields[0].field_type,
            stamped("Header", "std_msgs", Some(header_hash()))
        );
        match &pose_stamped.fields[1].field_type {
            FieldType::Nested {
                fixed: Some(info),
                target_hash: None,
                ..
            } => assert_eq!(info.target_hash, POSE_WIRE_HASH),
            other => panic!("pose must inline with no target_hash, got {other:?}"),
        }
        assert_eq!(pose_stamped.schema_hash(), POSE_STAMPED_WIRE_HASH);
    }

    /// `demo/A { demo/B b; string s }`, `demo/B { demo/A a; string s }` and
    /// `demo/T { demo/A a }`, in the order `names` gives.
    fn two_cycle_with_referrer(names: [&str; 3]) -> Vec<MessageSchema> {
        names
            .iter()
            .map(|name| {
                let mut s = MessageSchema::new_in_package(*name, "demo");
                match *name {
                    "A" => {
                        s.add_field(FieldDef::new("b", nested("B", Some("demo"))));
                        s.add_field(FieldDef::new("s", FieldType::String));
                    }
                    "B" => {
                        s.add_field(FieldDef::new("a", nested("A", Some("demo"))));
                        s.add_field(FieldDef::new("s", FieldType::String));
                    }
                    _ => s.add_field(FieldDef::new("a", nested("A", Some("demo")))),
                }
                s
            })
            .collect()
    }

    #[test]
    fn a_reference_cycle_is_not_stamped_and_an_outside_referrer_is_stamped_in_every_order() {
        let a_hash = recipe3_hash("demo/A", 0, &[("b", "demo/B", None), ("s", "string", None)]);
        let t_hash = recipe3_hash("demo/T", 0, &[("a", "demo/A", None)]);
        let mut expected_warnings = vec![
            "A.b: 'demo/B' is in a reference cycle with 'demo/A'; the reference carries no \
             target hash, so a layout change in 'demo/B' does not change the hash of 'demo/A'"
                .to_string(),
            "B.a: 'demo/A' is in a reference cycle with 'demo/B'; the reference carries no \
             target hash, so a layout change in 'demo/A' does not change the hash of 'demo/B'"
                .to_string(),
        ];
        expected_warnings.sort();
        for names in [["A", "B", "T"], ["T", "B", "A"], ["B", "T", "A"]] {
            let mut schemas = two_cycle_with_referrer(names);
            let mut cycle_warnings: Vec<String> = resolve_fixed_nested(&mut schemas)
                .into_iter()
                .filter(|w| w.contains("is in a reference cycle"))
                .collect();
            cycle_warnings.sort();
            assert_eq!(cycle_warnings, expected_warnings, "order {names:?}");
            for schema in &schemas {
                let expected = match schema.name.as_str() {
                    "A" => stamped("B", "demo", None),
                    "B" => stamped("A", "demo", None),
                    _ => stamped("A", "demo", Some(a_hash)),
                };
                assert_eq!(schema.fields[0].field_type, expected, "order {names:?}");
            }
            let t = schemas.iter().find(|s| s.name == "T").unwrap();
            assert_eq!(t.schema_hash(), t_hash, "order {names:?}");
        }
    }

    #[test]
    fn a_self_reference_through_a_dynamic_array_is_silent_and_unstamped() {
        let mut node = MessageSchema::new_in_package("Node", "demo");
        node.add_field(FieldDef::new("name", FieldType::String));
        node.add_field(FieldDef::new(
            "children",
            dynamic(nested("Node", Some("demo"))),
        ));
        let mut schemas = vec![node];
        let warnings = resolve_fixed_nested(&mut schemas);
        assert!(warnings.is_empty(), "{warnings:?}");
        assert_eq!(
            schemas[0].fields[1].field_type,
            dynamic(stamped("Node", "demo", None))
        );
    }

    #[test]
    fn a_cycle_through_a_dynamic_array_warns_once_per_reference() {
        let mut tree = MessageSchema::new_in_package("Tree", "demo");
        tree.add_field(FieldDef::new(
            "leaves",
            dynamic(nested("Leaf", Some("demo"))),
        ));
        let mut leaf = MessageSchema::new_in_package("Leaf", "demo");
        leaf.add_field(FieldDef::new("parent", nested("Tree", Some("demo"))));
        let mut schemas = vec![tree, leaf];
        let warnings = resolve_fixed_nested(&mut schemas);
        assert_eq!(
            warnings,
            vec![
                "Tree.leaves: array element: 'demo/Leaf' is in a reference cycle with \
                 'demo/Tree'; the reference carries no target hash, so a layout change in \
                 'demo/Leaf' does not change the hash of 'demo/Tree'"
                    .to_string(),
                "Leaf.parent: 'demo/Tree' is in a reference cycle with 'demo/Leaf'; the \
                 reference carries no target hash, so a layout change in 'demo/Tree' does \
                 not change the hash of 'demo/Leaf'"
                    .to_string(),
            ]
        );
        assert_eq!(
            schemas[0].fields[0].field_type,
            dynamic(stamped("Leaf", "demo", None))
        );
        assert_eq!(
            schemas[1].fields[0].field_type,
            stamped("Tree", "demo", None)
        );
    }

    /// `uint64[N]` with `N * 8` just inside `usize`, so two of them in one
    /// fixed section overflow the size arithmetic.
    fn huge_u64_array() -> FieldType {
        FieldType::FixedArray {
            element_type: Box::new(FieldType::U64),
            length: usize::MAX / 8,
        }
    }

    #[test]
    fn a_target_whose_fixed_section_cannot_be_sized_is_warned_never_a_panic() {
        // A variable schema and a fixed one whose composed fixed sections
        // overflow, referenced only where they are not inlined: directly
        // (the variable one) and under a DynamicArray (both).
        let mut hostile_variable = MessageSchema::new_in_package("V", "demo");
        hostile_variable.add_field(FieldDef::new("s", FieldType::String));
        hostile_variable.add_field(FieldDef::new("a", huge_u64_array()));
        hostile_variable.add_field(FieldDef::new("b", huge_u64_array()));
        let mut hostile_fixed = MessageSchema::new_in_package("F", "demo");
        hostile_fixed.add_field(FieldDef::new("a", huge_u64_array()));
        hostile_fixed.add_field(FieldDef::new("b", huge_u64_array()));
        let mut direct = MessageSchema::new_in_package("P", "demo");
        direct.add_field(FieldDef::new("v", nested("V", Some("demo"))));
        let mut variable_array = MessageSchema::new_in_package("Q", "demo");
        variable_array.add_field(FieldDef::new("vs", dynamic(nested("V", Some("demo")))));
        let mut fixed_array = MessageSchema::new_in_package("R", "demo");
        fixed_array.add_field(FieldDef::new("fs", dynamic(nested("F", Some("demo")))));
        let mut schemas = vec![
            hostile_variable,
            hostile_fixed,
            direct,
            variable_array,
            fixed_array,
        ];

        let warnings = resolve_fixed_nested(&mut schemas);

        let prefixes = [
            "P.v: 'demo/V' has no wire hash (schema `V` field `b`: fixed-section size overflows",
            "Q.vs: array element: 'demo/V' has no wire hash (schema `V` field `b`: fixed-section \
             size overflows",
            "R.fs: array element: 'demo/F' has no wire hash (schema `F` field `b`: fixed-section \
             size overflows",
        ];
        assert_eq!(warnings.len(), prefixes.len(), "{warnings:?}");
        for (warning, prefix) in warnings.iter().zip(prefixes) {
            assert!(warning.starts_with(prefix), "{warning}");
            assert!(
                warning.ends_with("; the reference carries no target hash"),
                "{warning}"
            );
        }
        assert_eq!(schemas[2].fields[0].field_type, stamped("V", "demo", None));
        assert_eq!(
            schemas[3].fields[0].field_type,
            dynamic(stamped("V", "demo", None))
        );
        assert_eq!(
            schemas[4].fields[0].field_type,
            dynamic(stamped("F", "demo", None))
        );
    }

    #[test]
    fn an_unbound_dynamic_array_element_under_a_fixed_array_warns_and_stays_unstamped() {
        let mut s = MessageSchema::new_in_package("Grid", "demo");
        s.add_field(FieldDef::new(
            "rows",
            dynamic(FieldType::FixedArray {
                element_type: Box::new(nested("Missing", Some("demo"))),
                length: 3,
            }),
        ));
        let mut schemas = vec![s];
        let warnings = resolve_fixed_nested(&mut schemas);
        assert_eq!(warnings.len(), 1, "{warnings:?}");
        assert!(
            warnings[0].starts_with("Grid.rows: array element: unknown schema 'demo/Missing'"),
            "{warnings:?}"
        );
        assert!(
            warnings[0].ends_with("array stays opaque-bytes (layout unaffected)"),
            "{warnings:?}"
        );
        assert_eq!(
            schemas[0].fields[0].field_type,
            dynamic(FieldType::FixedArray {
                element_type: Box::new(stamped("Missing", "demo", None)),
                length: 3,
            })
        );
    }

    #[test]
    fn stale_target_hash_cleared_on_reresolution() {
        let mut full = vec![inner(FieldType::U32), outer()];
        assert!(resolve_fixed_nested(&mut full).is_empty());
        let resolved_outer = full[1].clone();
        assert_eq!(
            resolved_outer.fields[0].field_type,
            stamped("Inner", "demo", Some(inner_hash("uint32")))
        );

        // The target is gone: the stamp goes with it.
        let mut alone = vec![resolved_outer.clone()];
        let warnings = resolve_fixed_nested(&mut alone);
        assert_eq!(warnings.len(), 1, "{warnings:?}");
        assert!(
            warnings[0].contains("unknown schema 'demo/Inner'"),
            "{warnings:?}"
        );
        assert_eq!(
            alone[0].fields[0].field_type,
            stamped("Inner", "demo", None)
        );

        // The target changed: the stamp follows the current definition.
        let mut changed = vec![inner(FieldType::F32), resolved_outer.clone()];
        assert!(resolve_fixed_nested(&mut changed).is_empty());
        assert_eq!(
            changed[1].fields[0].field_type,
            stamped("Inner", "demo", Some(inner_hash("float32")))
        );

        // The target turned fixed: the hash moves into the fixed stamp.
        let mut fixed_inner = MessageSchema::new_in_package("Inner", "demo");
        fixed_inner.add_field(FieldDef::new("id", FieldType::U32));
        let mut now_fixed = vec![fixed_inner, resolved_outer];
        assert!(resolve_fixed_nested(&mut now_fixed).is_empty());
        let fixed_hash = recipe3_hash("demo/Inner", 4, &[("id", "uint32", None)]);
        match &now_fixed[1].fields[0].field_type {
            FieldType::Nested {
                fixed: Some(info),
                target_hash: None,
                ..
            } => assert_eq!(info.target_hash, fixed_hash),
            other => panic!("expected an inlined Inner with no target_hash, got {other:?}"),
        }

        // And back to variable: the fixed stamp is cleared, the hash returns.
        let mut variable_again = vec![inner(FieldType::U32), now_fixed[1].clone()];
        assert!(resolve_fixed_nested(&mut variable_again).is_empty());
        assert_eq!(
            variable_again[1].fields[0].field_type,
            stamped("Inner", "demo", Some(inner_hash("uint32")))
        );
    }

    #[test]
    fn stale_stamps_on_a_dynamic_array_element_are_replaced() {
        let mut list = MessageSchema::new_in_package("PoseList", "test_msgs");
        list.add_field(FieldDef::new(
            "points",
            dynamic(FieldType::Nested {
                schema_name: "Point".to_string(),
                package: Some("geometry_msgs".to_string()),
                fixed: Some(NestedFixedInfo {
                    has_large_array: false,
                    fixed_size: 24,
                    alignment: 8,
                    target_hash: 0xdead,
                }),
                target_hash: Some(0xbeef),
            }),
        ));
        let mut schemas = vec![point_schema("geometry_msgs"), list];
        assert!(resolve_fixed_nested(&mut schemas).is_empty());
        let point = recipe3_hash(
            "geometry_msgs/Point",
            24,
            &[
                ("x", "float64", None),
                ("y", "float64", None),
                ("z", "float64", None),
            ],
        );
        assert_eq!(
            schemas[1].fields[0].field_type,
            dynamic(stamped("Point", "geometry_msgs", Some(point)))
        );
    }

    #[test]
    fn resolving_a_resolved_set_again_changes_nothing() {
        let mut once = pose_family(POSITION_FIRST);
        once.extend(two_cycle_with_referrer(["A", "B", "T"]));
        let first_warnings = resolve_fixed_nested(&mut once);
        let mut twice = once.clone();
        let second_warnings = resolve_fixed_nested(&mut twice);
        assert_eq!(first_warnings, second_warnings);
        for (a, b) in once.iter().zip(&twice) {
            let types = |s: &MessageSchema| -> Vec<FieldType> {
                s.fields.iter().map(|f| f.field_type.clone()).collect()
            };
            assert_eq!(types(a), types(b), "{}", a.qualified_name());
            assert_eq!(a.schema_hash(), b.schema_hash(), "{}", a.qualified_name());
        }
    }
}
