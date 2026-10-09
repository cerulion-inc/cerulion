// SPDX-License-Identifier: AGPL-3.0-only
//! [`SchemaSet`]: an owned, resolvable collection of message schemas.

use std::collections::BTreeSet;
use std::path::Path;

use super::schema_yaml::{parse_yaml_schemas, validate_fixed_lengths};
use super::DynamicError;
use crate::codegen::layout::WireLayout;
use crate::codegen::parse_rosmsg as parse_rosmsg_raw;
use crate::codegen::{composed_overflow_indices, FieldType, FrameWalker, MessageSchema};

/// An owned set of [`MessageSchema`]s plus the [`FrameWalker`] resolved over
/// them.
///
/// Every mutation (`add_*`) re-resolves the WHOLE set (nested references bind
/// across schemas, so a later addition can flip an earlier schema from
/// "variable" to "fixed" and change its hash) and returns the resolver's
/// warnings. Each warning is also logged at `warn` - an unresolved nested
/// reference silently degrades a schema to opaque bytes, which a binding
/// author must see.
///
/// Building is a cold path (allocates freely). Lookups (`layout`,
/// `layout_for_hash`, `schema_hash`) are `O(log n)` map lookups with no
/// allocation.
#[derive(Clone)]
pub struct SchemaSet {
    schemas: Vec<MessageSchema>,
    walker: FrameWalker,
}

impl std::fmt::Debug for SchemaSet {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("SchemaSet")
            .field("schemas", &self.schemas.len())
            .finish_non_exhaustive()
    }
}

impl SchemaSet {
    /// Build a set from already-parsed schemas (e.g. the caller's own IR or
    /// `native_ros2_messages::BUILTIN_MSGS` parsed with [`parse_rosmsg`]).
    ///
    /// Returns the set and the resolution warnings (also logged). The IR is
    /// preflighted the same way the parse paths are: declared fixed sizes,
    /// composed fixed-nested sizes, and the resolved frame prefix must all fit
    /// the wire.
    pub fn from_schemas(schemas: Vec<MessageSchema>) -> Result<(Self, Vec<String>), DynamicError> {
        let (walker, warnings) = Self::build(&schemas)?;
        log_warnings(&warnings);
        Ok((Self { schemas, walker }, warnings))
    }

    /// Load a workspace the way the `cerulion` CLI does: every top-level
    /// `<workspace>/schemas/*.yaml` (sorted by path) as package-less YAML
    /// schemas, plus every `<workspace>/schemas/<pkg>/msg/*.msg` store
    /// file as a `pkg/Type` ROS 2 schema.
    ///
    /// A missing `schemas/` directory yields an empty set. An unreadable or
    /// unparseable FILE is skipped with a warning (returned and logged) so
    /// one bad file never sinks the rest - the CLI's contract. Returns
    /// `Err` when `<workspace>/schemas` exists but cannot be stat'ed or
    /// listed, or when a package `msg/` directory's metadata cannot be
    /// read (an `msg` path that is missing or a plain file is simply
    /// skipped).
    pub fn from_workspace_dir(workspace: &Path) -> Result<(Self, Vec<String>), DynamicError> {
        let mut yaml = Vec::new();
        let mut store = Vec::new();
        let mut file_warnings = Vec::new();
        let schemas_dir = workspace.join("schemas");
        match std::fs::metadata(&schemas_dir) {
            Ok(_) => {}
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => {
                tracing::debug!(dir = %schemas_dir.display(), "no schemas/ directory");
                return Self::from_schemas(yaml);
            }
            Err(e) => return Err(io_err(&schemas_dir, &e)),
        }

        for path in sorted_entries(&schemas_dir)? {
            if path.extension().and_then(|e| e.to_str()) == Some("yaml") && path.is_file() {
                match std::fs::read_to_string(&path).map_err(|e| io_err(&path, &e)) {
                    Ok(text) => match parse_yaml_checked(&text) {
                        Ok(parsed) => yaml.extend(parsed),
                        Err(e) => file_warnings.push(skip_warning(&path, &e)),
                    },
                    Err(e) => file_warnings.push(skip_warning(&path, &e)),
                }
            } else if path.is_dir() {
                let Some(pkg) = path.file_name().and_then(|s| s.to_str()) else {
                    continue;
                };
                let msg_dir = path.join("msg");
                match std::fs::metadata(&msg_dir) {
                    Ok(meta) if meta.is_dir() => {}
                    Ok(_) => continue,
                    Err(e) if e.kind() == std::io::ErrorKind::NotFound => continue,
                    Err(e) => return Err(io_err(&msg_dir, &e)),
                }
                for msg_path in sorted_entries(&msg_dir)? {
                    if msg_path.extension().and_then(|e| e.to_str()) != Some("msg") {
                        continue;
                    }
                    let Some(stem) = msg_path.file_stem().and_then(|s| s.to_str()) else {
                        continue;
                    };
                    match std::fs::read_to_string(&msg_path).map_err(|e| io_err(&msg_path, &e)) {
                        Ok(text) => match parse_rosmsg(&text, stem, Some(pkg)) {
                            Ok(schema) => store.push(schema),
                            Err(e) => file_warnings.push(skip_warning(&msg_path, &e)),
                        },
                        Err(e) => file_warnings.push(skip_warning(&msg_path, &e)),
                    }
                }
            }
        }

        // Workspace YAML outranks the `.msg` store on a name collision, as the
        // CLI resolves it. A store parent bound the shadowed definition under
        // the resolver's `(package, name)` key; the YAML twin is package-less
        // (its name carries the slash), so that key never binds to it and the
        // parent would re-resolve the field as opaque bytes with a changed
        // layout and hash. Every schema, YAML or store, that bound a
        // shadowed definition goes with it, transitively, bound by registry
        // key over the full pre-shadow set exactly as the resolver bound it.
        let yaml_names: BTreeSet<String> = yaml.iter().map(MessageSchema::qualified_name).collect();
        let (shadowed, store): (Vec<MessageSchema>, Vec<MessageSchema>) = store
            .into_iter()
            .partition(|s| yaml_names.contains(&s.qualified_name()));
        for schema in &shadowed {
            file_warnings.push(format!(
                "workspace YAML schema '{}' shadows the .msg store definition of the same name",
                schema.qualified_name()
            ));
        }
        let mut schemas = yaml;
        schemas.extend(store);
        if !shadowed.is_empty() {
            drop_dependents(&mut schemas, shadowed, "shadowed", &mut file_warnings);
        }

        // Each pass removes only the ACTIVE definition of an offending name
        // (the resolver's later-wins twin, which is the one the verdict was
        // about) and propagates the rejection to dependents only when no
        // same-name twin survives: a survivor becomes the resolver's winner
        // on the next pass and the parents still bind to it.
        loop {
            let bad = composed_overflow_indices(&schemas);
            if !bad.is_empty() {
                let mut removed = Vec::new();
                for &i in bad.iter().rev() {
                    let schema = schemas.remove(i);
                    file_warnings.push(format!(
                        "skipped workspace schema '{}': composed fixed section (after fixed-nested inlining) overflows usize (the rest still load)",
                        schema.qualified_name()
                    ));
                    removed.push(schema);
                }
                let rejected = fully_removed(&schemas, removed);
                drop_dependents(&mut schemas, rejected, "skipped", &mut file_warnings);
                continue;
            }
            let (walker, warnings) = FrameWalker::new(schemas.clone());
            let over: BTreeSet<String> = schemas
                .iter()
                .map(MessageSchema::qualified_name)
                .filter(|q| {
                    walker
                        .layout_for_name(q)
                        .is_some_and(|layout| layout.frame_prefix_exceeds_wire())
                })
                .collect();
            if over.is_empty() {
                log_warnings(&file_warnings);
                log_warnings(&warnings);
                file_warnings.extend(warnings);
                return Ok((Self { schemas, walker }, file_warnings));
            }
            let mut removed = Vec::new();
            for q in &over {
                if let Some(i) = schemas.iter().rposition(|s| s.qualified_name() == *q) {
                    removed.push(schemas.remove(i));
                }
                file_warnings.push(format!(
                    "skipped workspace schema '{q}': frame prefix exceeds the u32 wire total_size (the rest still load)"
                ));
            }
            let rejected = fully_removed(&schemas, removed);
            drop_dependents(&mut schemas, rejected, "skipped", &mut file_warnings);
        }
    }

    /// Parse one workspace schema-YAML document (a `schemas:` mapping of
    /// `<Name>: { description: ..., fields: { "<type> <name>": ... } }`)
    /// and add every schema in it, re-resolving the set.
    ///
    /// Returns the resolution warnings (also logged). On `Err` the set is
    /// unchanged.
    pub fn add_yaml_str(&mut self, yaml: &str) -> Result<Vec<String>, DynamicError> {
        let parsed = parse_yaml_checked(yaml)?;
        let mut candidate = self.schemas.clone();
        candidate.extend(parsed);
        let (walker, warnings) = Self::build(&candidate)?;
        self.schemas = candidate;
        self.walker = walker;
        log_warnings(&warnings);
        Ok(warnings)
    }

    /// Parse one ROS 2 `.msg` text as `package/name` (or bare `name` when
    /// `package` is `None`) and add it, re-resolving the set.
    ///
    /// Returns the resolution warnings (also logged). On `Err` the set is
    /// unchanged.
    pub fn add_rosmsg_str(
        &mut self,
        text: &str,
        name: &str,
        package: Option<&str>,
    ) -> Result<Vec<String>, DynamicError> {
        let schema = parse_rosmsg(text, name, package)?;
        let mut candidate = self.schemas.clone();
        candidate.push(schema);
        let (walker, warnings) = Self::build(&candidate)?;
        self.schemas = candidate;
        self.walker = walker;
        log_warnings(&warnings);
        Ok(warnings)
    }

    /// The schemas in insertion order (unresolved IR, exactly as added).
    pub fn schemas(&self) -> &[MessageSchema] {
        &self.schemas
    }

    /// The walker resolved over the current set.
    pub fn walker(&self) -> &FrameWalker {
        &self.walker
    }

    /// The layout of the schema with this qualified name (`pkg/Type` or
    /// bare `Type`), if present.
    pub fn layout(&self, qualified_name: &str) -> Option<&WireLayout> {
        self.walker.layout_for_name(qualified_name)
    }

    /// The layout of the schema whose wire `schema_hash` is `hash`, if any.
    pub fn layout_for_hash(&self, hash: u64) -> Option<&WireLayout> {
        self.walker.layout_for_hash(hash)
    }

    /// The wire `schema_hash` of the schema with this qualified name, if
    /// present.
    pub fn schema_hash(&self, qualified_name: &str) -> Option<u64> {
        self.walker.schema_hash_for(qualified_name)
    }

    /// Return the metadata emitted for this schema by generated wire types.
    ///
    /// The final value is the wire-header-plus-fixed-body size for fixed
    /// schemas, and otherwise carries the same slice ceiling selected by
    /// codegen for the qualified name.
    pub fn output_meta(&self, qualified_name: &str) -> Option<(u64, usize, Option<u32>)> {
        let layout = self.layout(qualified_name)?;
        let max_slice_len_default = if layout.variable_fields.is_empty() {
            Some((crate::wire::WireHeader::SIZE + layout.fixed_size) as u32)
        } else {
            Some(crate::codegen::slice_ceiling_for_type(qualified_name).get())
        };
        Some((layout.schema_hash, layout.fixed_size, max_slice_len_default))
    }

    /// The qualified name the walker decodes `hash` as, if any.
    pub fn schema_name_for_hash(&self, hash: u64) -> Option<&str> {
        self.walker.schema_name_for_hash(hash)
    }

    fn build(schemas: &[MessageSchema]) -> Result<(FrameWalker, Vec<String>), DynamicError> {
        for schema in schemas {
            if let Err(detail) = schema.checked_wire_fixed_size() {
                return Err(DynamicError::SchemaNotWireRepresentable {
                    schema: schema.qualified_name(),
                    detail,
                });
            }
        }
        let bad = composed_overflow_indices(schemas);
        if let Some(&i) = bad.first() {
            return Err(DynamicError::SchemaNotWireRepresentable {
                schema: schemas[i].qualified_name(),
                detail: "composed fixed section (after fixed-nested inlining) overflows usize"
                    .into(),
            });
        }
        let (walker, warnings) = FrameWalker::new(schemas.to_vec());
        for schema in schemas {
            let qname = schema.qualified_name();
            if let Some(layout) = walker.layout_for_name(&qname) {
                if layout.frame_prefix_exceeds_wire() {
                    return Err(DynamicError::SchemaNotWireRepresentable {
                        schema: qname,
                        detail: format!(
                            "frame prefix (32-byte header + {}-byte fixed section + {}-entry offset table) exceeds the u32 wire total_size",
                            layout.fixed_size,
                            layout.variable_fields.len()
                        ),
                    });
                }
            }
        }
        Ok((walker, warnings))
    }
}

fn log_warnings(warnings: &[String]) {
    for w in warnings {
        tracing::warn!(warning = %w, "dynamic schema set: resolution warning");
    }
}

fn io_err(path: &Path, e: &std::io::Error) -> DynamicError {
    DynamicError::Io {
        path: path.display().to_string(),
        detail: e.to_string(),
    }
}

fn skip_warning(path: &Path, e: &DynamicError) -> String {
    format!(
        "skipped workspace schema file '{}': {e} (the rest still load)",
        path.display()
    )
}

/// Directory entries sorted by path - byte-deterministic enumeration.
fn sorted_entries(dir: &Path) -> Result<Vec<std::path::PathBuf>, DynamicError> {
    let mut paths: Vec<std::path::PathBuf> = std::fs::read_dir(dir)
        .map_err(|e| io_err(dir, &e))?
        .map(|entry| entry.map(|d| d.path()).map_err(|e| io_err(dir, &e)))
        .collect::<Result<_, _>>()?;
    paths.sort();
    Ok(paths)
}

fn parse_yaml_checked(yaml: &str) -> Result<Vec<MessageSchema>, DynamicError> {
    parse_yaml_schemas(yaml)?
        .into_iter()
        .map(check_representable)
        .collect()
}

/// Parse one ROS 2 `.msg` using the same grammar as
/// [`crate::codegen::parse_rosmsg`], plus `validate_fixed_lengths` and the
/// declared-size gate. Composed overflow across a set is caught by
/// [`SchemaSet`] construction, not here.
pub fn parse_rosmsg(
    text: &str,
    name: &str,
    package: Option<&str>,
) -> Result<MessageSchema, DynamicError> {
    let schema = parse_rosmsg_raw(text, name, package).map_err(|e| DynamicError::Rosmsg {
        name: name.to_string(),
        detail: e.to_string(),
    })?;
    for field in &schema.fields {
        validate_fixed_lengths(name, &field.name, &field.field_type)?;
    }
    check_representable(schema)
}

/// Refuse a schema whose fixed section cannot be sized - it has no wire
/// hash, and the panicking size recipe must never see it.
fn check_representable(schema: MessageSchema) -> Result<MessageSchema, DynamicError> {
    match schema.checked_wire_fixed_size() {
        Ok(_) => Ok(schema),
        Err(detail) => Err(DynamicError::SchemaNotWireRepresentable {
            schema: schema.qualified_name(),
            detail,
        }),
    }
}

/// The resolver's registry key of a definition: `(package, name)`. A
/// package-less YAML schema named `pkg/Point` keys as `(None, "pkg/Point")`,
/// which is why it prints like a store `(Some("pkg"), "Point")` but never
/// binds in its place.
type SchemaKey = (Option<String>, String);

fn key_of(schema: &MessageSchema) -> SchemaKey {
    (schema.package.clone(), schema.name.clone())
}

fn qualified(key: &SchemaKey) -> String {
    match &key.0 {
        Some(pkg) => format!("{pkg}/{}", key.1),
        None => key.1.clone(),
    }
}

/// The definitions among `removed` whose key no surviving definition still
/// bears. Only those propagate to dependents: a surviving same-key twin is
/// the resolver's next winner, so the parents that referenced the name still
/// bind.
fn fully_removed(schemas: &[MessageSchema], removed: Vec<MessageSchema>) -> Vec<MessageSchema> {
    let surviving: BTreeSet<SchemaKey> = schemas.iter().map(key_of).collect();
    removed
        .into_iter()
        .filter(|r| !surviving.contains(&key_of(r)))
        .collect()
}

/// Remove every schema that references a rejected one, transitively. Left in
/// place, such a parent would re-resolve the missing target as opaque bytes
/// and silently load with a different layout and hash than it declares.
/// `cause` names why a FIRST-level target went (`skipped`, `shadowed`) in its
/// warning; a schema dropped transitively references a `skipped` one.
pub(super) fn drop_dependents(
    schemas: &mut Vec<MessageSchema>,
    rejected: Vec<MessageSchema>,
    mut cause: &str,
    warnings: &mut Vec<String>,
) {
    let mut rejected: BTreeSet<SchemaKey> = rejected.iter().map(key_of).collect();
    // Bound as the resolver bound them BEFORE the rejection: every surviving
    // definition plus the rejected ones, by registry key.
    let known: BTreeSet<SchemaKey> = schemas
        .iter()
        .map(key_of)
        .chain(rejected.iter().cloned())
        .collect();
    loop {
        let before = schemas.len();
        let mut newly = BTreeSet::new();
        schemas.retain(|schema| {
            let Some(target) = schema
                .fields
                .iter()
                .find_map(|f| rejected_reference(&f.field_type, schema, &known, &rejected))
            else {
                return true;
            };
            let q = schema.qualified_name();
            let target = qualified(&target);
            warnings.push(format!(
                "skipped workspace schema '{q}': it references {cause} schema '{target}' (the rest still load)"
            ));
            newly.insert(key_of(schema));
            false
        });
        if schemas.len() == before {
            return;
        }
        rejected.extend(newly);
        // A schema dropped here was skipped, whatever the cause of the first
        // pass, so the transitive passes say so.
        cause = "skipped";
    }
}

/// The rejected schema `ty` references, if any: the reference is bound among
/// `known` exactly as `codegen::resolve_fixed_nested` binds it, and only that
/// binding counts.
fn rejected_reference(
    ty: &FieldType,
    owner: &MessageSchema,
    known: &BTreeSet<SchemaKey>,
    rejected: &BTreeSet<SchemaKey>,
) -> Option<SchemaKey> {
    match ty {
        FieldType::Nested {
            schema_name,
            package,
            ..
        } => bind_reference(
            schema_name,
            package.as_deref(),
            owner.package.as_deref(),
            known,
        )
        .filter(|target| rejected.contains(target)),
        FieldType::FixedArray { element_type, .. } | FieldType::DynamicArray { element_type } => {
            rejected_reference(element_type, owner, known, rejected)
        }
        _ => None,
    }
}

/// The registry key a nested reference binds to, by the resolver's
/// precedence: an explicit package only; else the parent's package (`None`
/// for a package-less parent); else bare `Header` as `std_msgs/Header`; else
/// the one definition whose NAME is the bare name, whatever its package (a
/// package-less YAML `pkg/Point` has the name `pkg/Point`, so it is never a
/// bare `Point` candidate).
fn bind_reference(
    name: &str,
    package: Option<&str>,
    parent_package: Option<&str>,
    known: &BTreeSet<SchemaKey>,
) -> Option<SchemaKey> {
    let keyed = |pkg: Option<&str>| (pkg.map(str::to_string), name.to_string());
    if package.is_some() {
        let exact = keyed(package);
        return known.contains(&exact).then_some(exact);
    }
    let local = keyed(parent_package);
    if known.contains(&local) {
        return Some(local);
    }
    if name == "Header" {
        let header = (Some("std_msgs".to_string()), "Header".to_string());
        if known.contains(&header) {
            return Some(header);
        }
    }
    let mut matches = known.iter().filter(|k| k.1 == name);
    match (matches.next(), matches.next()) {
        (Some(only), None) => Some(only.clone()),
        _ => None,
    }
}
