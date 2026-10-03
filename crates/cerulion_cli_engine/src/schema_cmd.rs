// SPDX-License-Identifier: AGPL-3.0-only
//! Schema commands: create, delete, and unified info/list over THREE
//! sources — workspace YAML (`schemas/*.yaml`), the workspace `.msg` store
//! (`schemas/<pkg>/msg/<Type>.msg`), and the embedded built-in
//! ROS 2 registry. Precedence: workspace YAML → `.msg` store →
//! built-in registry.

use std::collections::{BTreeMap, BTreeSet};
use std::fmt;
use std::path::Path;

use cerulion_core::codegen::layout::LayoutResolver;
use cerulion_core::codegen::{parse_rosmsg, FieldDef, FieldType, MessageSchema};

use crate::error::{CliError, CliResult};
use crate::schema_store::{builtin_has_qualified, SchemaStore, StoredSchema};
use crate::utils::to_pascal_case;
use crate::workspace_lock::WorkspaceLock;

/// Inline fixed-length cap, shared with `cerulion_core::dynamic` (the ONE
/// YAML schema parser) so CLI and bindings refuse the same declarations.
pub use cerulion_core::dynamic::MAX_FIXED_ARRAY_LEN;

/// `schema info`-context remedy for [`CliError::SchemaNotFound`]:
/// names BOTH lookup sources and the
/// discovery command.
const SCHEMA_INFO_REMEDY: &str = "Workspace schemas are looked up by schemas/*.yaml file \
    stem or schema name; built-in ROS 2 messages use qualified 'pkg/Type' names (e.g. \
    sensor_msgs/Image). Run `cerulion schema list` to see every available schema.";

/// `schema delete`-context remedy: deletion is workspace-only, so
/// built-in lookup guidance would be off-context here (a built-in ROS 2
/// message cannot be deleted).
const SCHEMA_DELETE_REMEDY: &str = "Deletion targets the file schemas/<name>.yaml and only \
    workspace schemas can be deleted (built-in ROS 2 messages cannot). Run \
    `cerulion schema list` to see workspace schemas.";

/// `node modify -o/-i`-context remedy for [`CliError::SchemaNotFound`]:
/// the port-schema arg [`resolve_port_schema`] returns this
/// when a BARE name matched no unique built-in AND no workspace schema.
/// Names BOTH resolution sources, the discovery command, and the
/// explicit-qualification escape hatch — so the user never has to guess
/// which surface a bare name resolves against.
const PORT_SCHEMA_REMEDY: &str = "Bare names resolve against built-in ROS 2 messages \
    (unique short name) or workspace schemas (schemas/*.yaml). Run 'cerulion schema list' \
    to see every available schema, or qualify as 'pkg/Type' (e.g. geometry_msgs/Vector3).";

/// Normalize a schema name from Rust-style `::` separators to path-style `/`.
///
/// Example: `sensor_msgs::LaserScan` -> `sensor_msgs/LaserScan`
pub fn normalize_schema(s: &str) -> String {
    s.replace("::", "/")
}

/// Convert a schema name from path-style `/` to Rust-style `::` for display.
///
/// Example: `sensor_msgs/LaserScan` -> `sensor_msgs::LaserScan`
pub fn display_schema(s: &str) -> String {
    s.replace('/', "::")
}

/// Create a new schema YAML file.
pub fn schema_create(schemas_dir: &Path, name: &str) -> CliResult<()> {
    let root = schemas_dir.parent().unwrap_or(schemas_dir);
    let _lock = WorkspaceLock::acquire_and_track_gitignore(root)?;
    let schemas_dir = schemas_dir.canonicalize()?;
    let schema_path = schemas_dir.join(format!("{}.yaml", name));
    if schema_path.exists() {
        return Err(CliError::Validation(format!(
            "schema '{}' already exists",
            name
        )));
    }

    let pascal_name = to_pascal_case(name);
    let content = format!(
        r#"schemas:
  {pascal_name}:
    description: ""
    fields:
      # Add fields: type name
"#
    );
    std::fs::write(&schema_path, content)?;

    tracing::info!(schema = %name, "schema created");
    Ok(())
}

/// Delete a schema YAML file.
pub fn schema_delete(schemas_dir: &Path, name: &str) -> CliResult<()> {
    let root = schemas_dir.parent().unwrap_or(schemas_dir);
    let _lock = WorkspaceLock::acquire_and_track_gitignore(root)?;
    let schemas_dir = schemas_dir.canonicalize()?;
    let schema_path = schemas_dir.join(format!("{}.yaml", name));
    if !schema_path.exists() {
        return Err(CliError::SchemaNotFound {
            name: name.to_string(),
            remedy: SCHEMA_DELETE_REMEDY,
        });
    }
    std::fs::remove_file(&schema_path)?;

    tracing::info!(schema = %name, "schema deleted");
    Ok(())
}

/// Display info about a schema (looked up by `schemas/<name>.yaml` file
/// stem; [`schema_info_unified`] adds entry-name and built-in tiers).
pub fn schema_info(schemas_dir: &Path, name: &str) -> CliResult<SchemaInfoResult> {
    schema_info_entries(schemas_dir, name, None)
}

/// The FILE VIEW (`only == None`: every entry of `schemas/<name>.yaml`) or
/// the ENTRY VIEW (`only == Some(entry)`: that entry alone) of one workspace
/// YAML file. The size preflight is judged on the entries the view
/// RENDERS — the file view refuses a file holding any over-ceiling entry
/// ("fix the file"), while the entry view — the view `schema info <entry>`,
/// the resolver's qualified arm and the port surfaces bind through — sizes
/// only the entry asked for, so a hostile sibling never refuses a sane
/// entry of the same file on one surface and accepts it on another.
fn schema_info_entries(
    schemas_dir: &Path,
    name: &str,
    only: Option<&str>,
) -> CliResult<SchemaInfoResult> {
    let schema_path = schemas_dir.join(format!("{}.yaml", name));
    if !schema_path.exists() {
        return Err(CliError::SchemaNotFound {
            name: name.to_string(),
            remedy: SCHEMA_INFO_REMEDY,
        });
    }

    let content = std::fs::read_to_string(&schema_path)?;
    // No validation bypass: the file must pass the
    // ONE parse gate every other consumer routes through
    // (`parse_message_schemas` — non-string keys, EMPTY keys, the field-key
    // rules) BEFORE anything is displayed. `schema_info_unified` reaches this
    // function DIRECTLY for a present `<name>.yaml`, and the display loop
    // below coerced a non-string key to `"unknown"` and accepted an empty
    // key outright — so `schema info Foo` rendered a blank schema from a
    // file validation, listing and serving all refuse. The gate is not
    // duplicated here: the parse is simply run first (a cold one-shot verb;
    // the IR it builds is discarded — the loop below needs the raw
    // `schema_def` for the description and the rendered field rows).
    parse_message_schemas(&content)?;
    let doc: serde_yaml::Value = serde_yaml::from_str(&content)?;

    let schemas = doc
        .get("schemas")
        .and_then(|s| s.as_mapping())
        .ok_or_else(|| {
            CliError::Validation("invalid schema format: missing 'schemas' key".to_string())
        })?;

    let mut results = vec![];

    for (schema_name, schema_def) in schemas {
        // Every key is a non-empty string — the gate above refused the rest.
        // The entry keeps its DECLARED spelling (the hash identity); the
        // `only` filter matches it by either spelling.
        let name_str = schema_name.as_str().unwrap_or("unknown").to_string();
        if let Some(only) = only {
            if name_str != only && normalize_schema(&name_str) != normalize_schema(only) {
                continue;
            }
        }

        let description = schema_def
            .get("description")
            .and_then(|d| d.as_str())
            .unwrap_or("")
            .to_string();

        // Parse field keys (`<type> <name>`) in declaration order, build
        // the schema IR, and compute the layout-sensitive hash.
        //
        // Note: `canonical_str()` is called here per field for the display
        // string and again inside `schema.schema_hash()` (the wire-contract
        // hash recipe). The duplicate allocation is intentional and left
        // uncached: `schema info` is a cold one-shot CLI invocation, and
        // threading precomputed canonical strings into `schema_hash()`
        // would complicate the wire-format hash function's signature for no
        // measurable gain. Cold path — readability beats the micro-opt.
        let mut schema = MessageSchema::new(name_str.clone());
        let field_entries = parse_schema_fields_into(&name_str, schema_def, &mut schema)?;

        // The ONE size preflight runs BEFORE any hash or size of the
        // entry is materialized. The panicking recipe is SAFE here:
        // `parse_schema_fields_into` refuses any `FixedArray` past
        // `MAX_FIXED_ARRAY_LEN` (one dimension), nested references contribute
        // nothing before resolution, so the parse-time size is bounded by
        // fields × 2^20 × 8 bytes and no usize overflow is declarable — but
        // 512 such fields DO declare a fixed section past the u32 wire
        // ceiling (the frame prefix: 32-byte header + fixed section + offset
        // table), and a hash/size of that entry is a contract no frame can
        // carry. Such an entry is REFUSED with the same message the store
        // tier gives a rejected definition, never rendered with its
        // parse-time numbers.
        if !wire_representable_or_warn(&schema, "schema info workspace entry", SizeMoment::Declared)
        {
            return Err(preflight_rejection_error(
                &name_str,
                &format!("schemas/{name}.yaml"),
                &schema,
            ));
        }
        let hash = schema.schema_hash();
        // Parse-time layout (unresolved nested = variable, contributing
        // nothing) — the same IR the hash above consumed. Overwritten
        // below with canonically resolved values where resolution
        // succeeds; the hash is NOT recomputed.
        let wire_fixed_size = schema.wire_fixed_size();

        results.push(SchemaEntry {
            name: name_str,
            description,
            field_count: field_entries.len(),
            fields: field_entries,
            schema_hash: Some(hash),
            wire_fixed_size,
            // Flipped by `enrich_entries_with_resolved_layout` below on
            // successful resolution; stays false on the skip arms so
            // the renderer marks the parse-time value loudly.
            wire_fixed_size_resolved: false,
        });
    }

    // Resolve the layout columns against the
    // combined built-in + workspace set so the SAME type never reports
    // a different fixed/variable split here than via the built-in path.
    enrich_entries_with_resolved_layout(schemas_dir, &mut results);

    Ok(SchemaInfoResult { entries: results })
}

/// Parse one workspace schema-YAML document (the `schemas:` mapping) into
/// `MessageSchema` IR, in declaration order.
///
/// The graph-run path uses this to compute each workspace
/// schema's recipe-3 `schema_hash` for the YAML-`schema:`-vs-macro-output
/// divergence warn. It shares the IR-building loop with [`schema_info`]
/// (both call the core parser — same strict field-key parsing +
/// `FixedArray` length guards) but skips the per-field display
/// rendering `schema info` needs — this caller only wants the hashable IR.
///
/// The returned schemas are UNRESOLVED (every `Nested` reference is
/// `fixed: None`). The caller
/// ([`build_workspace_schema_hashes`](crate::graph_cmd)) collects the
/// schemas from EVERY workspace `schemas/*.yaml`, runs
/// [`cerulion_core::codegen::resolve_fixed_nested`] over the FULL set, and
/// only then hashes — matching codegen, which resolves fixed-nested fields
/// before computing `ShmMessage::SCHEMA_HASH`. Hashing a single file's
/// schemas in isolation would diverge from codegen for any schema carrying
/// a fixed-nested field.
///
/// Workspace schemas are package-less, so each returned schema's
/// [`MessageSchema::qualified_name`] is its bare name.
pub fn parse_message_schemas(content: &str) -> CliResult<Vec<MessageSchema>> {
    // Parse twice on purpose (this is a cold path): the raw parse preserves
    // `CliError::Yaml` for malformed documents, while the core parse remains
    // the ONE schema-rule implementation shared with bindings.
    serde_yaml::from_str::<serde_yaml::Value>(content)?;
    cerulion_core::dynamic::parse_yaml_schemas(content)
        .map_err(|e| CliError::Validation(e.to_string()))
}

/// Drop every schema whose fixed-section size arithmetic overflows
/// (`MessageSchema::checked_wire_fixed_size` errs), warning per drop.
///
/// The combined-set folds (`graph_cmd::build_workspace_schema_hashes`, its
/// wire-size sibling, `replay_field_registry`) MATERIALIZE every collected
/// schema's size/hash, and `wire_fixed_size` PANICS on a hostile
/// `FixedArray` length — so one hostile-but-parseable declaration anywhere
/// in `schemas/` (a hand-edited YAML, or a `.msg` acquired over the wire by
/// `ros2 attach`'s acquisition ladder) crashed `graph run`/`--record`/replay
/// outright (the same crash class already closed for `port_schema_exists`).
/// Callers run this BEFORE `resolve_fixed_nested` (so a hostile schema is
/// never inlined into a parent as a fixed-nested target) AND AFTER (a set
/// of individually-sizable schemas can still compose an overflow through
/// fixed-nested inlining). What NEITHER run covers: a composed-overflow
/// schema used as a nested TARGET panics inside `resolve_fixed_nested`
/// ITSELF, before any post-resolve retain runs — that is
/// [`retain_composed_sizable_schemas`]'s job (the replay field
/// registry runs it as a preflight).
///
/// Returns the dropped schemas' qualified names, so a caller holding
/// SIBLING structures keyed on the same set (the replay field registry's
/// schema vec + bare index) can scrub them CONSISTENTLY — a schema dropped
/// from one map but still reachable through another is exactly the panic
/// path this helper exists to close.
pub(crate) fn retain_sizable_schemas(
    schemas: &mut Vec<MessageSchema>,
    context: &'static str,
) -> Vec<String> {
    let mut dropped = Vec::new();
    schemas.retain(|s| {
        if sizable_or_warn(s, context) {
            true
        } else {
            dropped.push(s.qualified_name());
            false
        }
    });
    dropped
}

/// The per-definition verdict [`retain_sizable_schemas`] retains by, as a
/// predicate: `true` keeps; `false` has WARNED (the one message site) and
/// drops. Exposed for a consumer that must keep a PARALLEL structure aligned
/// with the survivors — the two `graph_cmd` folds carry each definition's
/// declaring FILE beside it so a file-STEM claim can be looked up through
/// its owner, and a
/// `Vec::retain` over the bare schema vec would desynchronize that.
pub(crate) fn sizable_or_warn(schema: &MessageSchema, context: &'static str) -> bool {
    match schema.checked_wire_fixed_size() {
        Ok(_) => true,
        Err(e) => {
            tracing::warn!(
                schema = %schema.qualified_name(),
                error = %e,
                context = %context,
                "schema's fixed-section size arithmetic overflows — skipped \
                 (hostile or corrupt declaration; the rest still load)"
            );
            false
        }
    }
}

/// The moment a wire-size ceiling is consulted — it decides how a nested
/// reference the schema has not (yet) resolved counts toward the offset
/// table (see [`offset_table_entries`]).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum SizeMoment {
    /// The schema AS PARSED, before `resolve_fixed_nested` ran over its set
    /// (the `layout_of` panic guard at every resolution-set seam): a nested
    /// reference may still inline, so only the fields variable under EVERY
    /// resolution count — a LOWER bound on the resolved offset table, which
    /// is what makes a refusal here final (resolution cannot shrink the
    /// prefix below it) and never a false one.
    Declared,
    /// The schema AFTER `resolve_fixed_nested`: every field it still
    /// classifies variable carries an offset entry in the frame — the exact
    /// count the layout walker materializes.
    Resolved,
}

/// Offset-table entries a frame of `schema` carries at `moment` — the exact
/// `variable_field_count` (the generator's own offset-table length) on a
/// resolved schema, its floor on a declared one. Nothing checks that
/// `moment` matches the schema: a wrong `Resolved` on a declared schema
/// under-counts by one entry per reference that later resolves VARIABLE and
/// admits a descriptor up to 8 bytes per such reference past the wire.
pub(crate) fn offset_table_entries(schema: &MessageSchema, moment: SizeMoment) -> usize {
    match moment {
        SizeMoment::Declared => schema.variable_field_count_floor(),
        SizeMoment::Resolved => schema.variable_field_count(),
    }
}

/// The wire-representability CEILING — the ONE rule every size-fold
/// surface applies, stated by `cerulion_core::wire` and consulted here:
/// a schema is representable iff its FRAME PREFIX — the 32-byte
/// `WireHeader`, the fixed section and one 8-byte `OffsetEntry` per
/// variable field, i.e. its smallest possible frame — fits the wire's
/// `u32` `total_size` (which is also the type of every recorded
/// `wire_fixed_size` descriptor). A ceiling on the fixed section alone
/// (`fixed_size <= u32::MAX`, the earlier rule) accepted a schema
/// whose emptiest frame is `u32::MAX + 32` bytes: it passed every preflight
/// and reached the recording map as a descriptor no publisher can emit.
/// Consulted at TWO moments: on the DECLARED size of ONE entry where a
/// surface refuses that entry outright (the `schema info` workspace entry,
/// port validation — with the declared offset-table FLOOR), and on
/// RESOLVED sizes by every consumer that reads one ([`representable_layout`]
/// on the schema-info / `schema list` `LayoutResolver` readers, the
/// recording fold's resolved retain, the replay registry's post-resolve
/// leg, the serving bindings, the recorder's own corpus — with the exact
/// resolved count) — because a set of REPRESENTABLE members can still
/// compose past the ceiling through fixed-nested inlining (`Outer =
/// Inner[2]` of a ~4 GiB `Inner`, or a variable schema whose fixed section
/// inlines two of them), and the composed probe deliberately judges only
/// the ARITHMETIC-overflow (panic) shape. NEVER on a SET before it
/// composes: dropping an over-ceiling target there leaves
/// its parents resolved with a variable reference and served as
/// representable.
pub(crate) fn exceeds_wire_ceiling(fixed_size: usize, offset_table_entries: usize) -> bool {
    cerulion_core::wire::frame_prefix_exceeds_wire(fixed_size, offset_table_entries)
}

/// The RESOLVED-size half of the ceiling for a `LayoutResolver` reader: a
/// materialized layout whose fixed section exceeds the ceiling is
/// reported as UNRESOLVED (loudly) — `schema info` then falls back to the
/// parse-time classification with the unresolved marker, exactly the
/// disposition the recording fold gives the same schema (omitted, recorded
/// 0). Without this, a composed-past-u32 schema RESOLVED on
/// `schema info` (its `fixed_size` is a `usize`) while the fold could not
/// carry it — the same declaration, two verdicts.
pub(crate) fn representable_layout(
    layout: Option<cerulion_core::codegen::layout::WireLayout>,
    context: &'static str,
) -> Option<cerulion_core::codegen::layout::WireLayout> {
    let layout = layout?;
    // The resolved layout knows its offset table exactly, and pairs its own
    // two numbers.
    if layout.frame_prefix_exceeds_wire() {
        let entries = layout.variable_fields.len();
        tracing::warn!(
            schema = %layout.qualified_name,
            fixed_size = layout.fixed_size,
            offset_table_entries = entries,
            max_fixed_size = cerulion_core::wire::max_wire_fixed_size(entries),
            context = %context,
            "schema's RESOLVED frame prefix (the 32-byte header, the fixed section after \
             fixed-nested inlining, and the offset table) exceeds the u32 wire \
             total_size — unrepresentable on the wire, reported as unresolved (hostile \
             or corrupt declaration; the rest still resolve)"
        );
        return None;
    }
    Some(layout)
}

/// The per-definition wire-ceiling verdict, as a predicate (the twin of
/// [`sizable_or_warn`]): `true` keeps; `false` has warned and drops. The
/// per-ENTRY refusal sites (`schema info`'s workspace entry, port
/// validation) consult it at the DECLARED moment; the composing folds'
/// post-resolution retains consult it at the RESOLVED one. `moment` names
/// how the schema's offset table is counted ([`offset_table_entries`]). SELF-
/// SUFFICIENT: a declaration whose size arithmetic overflows takes
/// [`sizable_or_warn`]'s verdict (its one message) rather than passing, so a
/// caller that runs only this predicate is still a complete `layout_of`
/// guard; a caller that already ran `sizable_or_warn` short-circuits before
/// reaching it, so nothing warns twice.
pub(crate) fn wire_representable_or_warn(
    schema: &MessageSchema,
    context: &'static str,
    moment: SizeMoment,
) -> bool {
    let Ok(size) = schema.checked_wire_fixed_size() else {
        return sizable_or_warn(schema, context);
    };
    let entries = offset_table_entries(schema, moment);
    if exceeds_wire_ceiling(size, entries) {
        tracing::warn!(
            schema = %schema.qualified_name(),
            fixed_size = size,
            offset_table_entries = entries,
            max_fixed_size = cerulion_core::wire::max_wire_fixed_size(entries),
            moment = ?moment,
            context = %context,
            "schema's frame prefix (the 32-byte header, the fixed section and the offset \
             table) exceeds the u32 wire total_size — unrepresentable on the wire, \
             skipped (hostile or corrupt declaration; the rest still load)"
        );
        return false;
    }
    true
}

/// FAULT INJECTION for the scrub fixpoints' NO-PROGRESS fallback:
/// `cargo test --lib` ONLY.
///
/// All three fixpoints (`graph_cmd`'s `scrub_composed_overflows`,
/// `schema_serve`'s hash-binding loop and `replay_field_registry`'s tier
/// loop) fall back to [`RemovedDefinitions::removes_identity`] when a pass
/// makes NO progress, because termination cannot rest on "every dropped
/// definition is matched by `removes`" — the identity-state bug
/// proved that assumption can fail, and the loop then rebuilt an identical
/// set forever.
///
/// [`RemovedDefinitions::removes_identity`] is unit-pinned, and the healthy
/// path is pinned NOT to reach the fallback
/// (`r41_a_resolved_moment_removal_scrubs_the_tiers_instead_of_spinning_the_fixpoint`
/// asserts the warn is ABSENT). What nothing pinned is the fallback
/// FIRING: that the loop terminates, that the offender leaves, and that
/// the degraded result is usable rather than a panic. The bug that
/// produced the state is fixed, so reaching it needs a deliberate seam.
///
/// The seam reproduces that identity-state shape exactly: a report whose recorded
/// identities are intact while [`RemovedDefinitions::removes`] matches
/// NOTHING. That is the one discriminating difference between the two
/// predicates — `removes` conjoins identity AND hash, `removes_identity`
/// drops the hash — so the fixture clears the two pure-identity arms
/// (`fully_removed_names` and `removed_keys`, which BOTH predicates read)
/// and leaves `removed_definitions` bearing a hash no schema of that
/// identity can carry. `!h` is provably not `h` for any `u64` and `Some(_)`
/// is provably not `None`, so the degrade cannot accidentally still match —
/// and every arm asserts that premise rather than trusting it.
///
/// STRUCTURALLY EXCLUDED FROM PRODUCTION: the module and its single call
/// site are `#[cfg(test)]`, so the seam is not compiled into the library
/// `cerulion_cli` and the `tests/*.rs` integration binaries link against —
/// it cannot be armed from outside this crate's own unit tests even by
/// accident. Pinned by `the_force_scrub_fault_seam_is_confined_to_cfg_test`.
#[cfg(test)]
pub(crate) mod force_scrub_fault {
    use super::RemovedDefinitions;
    use std::cell::Cell;

    thread_local! {
        static ARMED: Cell<bool> = const { Cell::new(false) };
    }

    /// Run `f` with the fault armed for the current THREAD.
    ///
    /// The ONLY way to arm it: `arm`/`Guard` are module-private, so a caller
    /// cannot write `let _ = arm();`, which would arm and disarm in the same
    /// statement and leave the body exercising the HEALTHY path while
    /// claiming to pin the fallback. Making the scope a closure makes that
    /// misuse unspellable rather than merely discouraged.
    ///
    /// Thread-local, so libtest's per-test threads isolate it with no
    /// `#[serial]`. `force_scrub_fault` is thread-scoped in the strict
    /// sense: a body that spawns a thread must call `with_armed` INSIDE that
    /// thread, or the work runs unarmed.
    pub(crate) fn with_armed<R>(f: impl FnOnce() -> R) -> R {
        let _guard = arm();
        f()
    }

    fn arm() -> Guard {
        // SAVE the previous value rather than assuming `false`: two live
        // guards would otherwise have the inner drop disarm the outer, and
        // the outer's remaining scope would silently exercise the healthy
        // path. Nesting is latent today (the arms are sequential) and one
        // refactor away from real.
        Guard(ARMED.with(|a| a.replace(true)))
    }

    /// RAII restore — a panicking body cannot leave the fault set for the
    /// next test that happens to reuse the thread.
    struct Guard(bool);

    impl Drop for Guard {
        fn drop(&mut self) {
            let prev = self.0;
            ARMED.with(|a| a.set(prev));
        }
    }

    /// Degrade `report` to the identity-state shape when armed; a no-op
    /// otherwise, and a no-op on an EMPTY report either way — which is
    /// exactly what lets an armed fixpoint still reach its own exit.
    pub(crate) fn degrade_if_armed(report: &mut RemovedDefinitions) {
        if !ARMED.with(|a| a.get()) {
            return;
        }
        report.fully_removed_names.clear();
        report.removed_keys.clear();
        for (_, _, hash) in &mut report.removed_definitions {
            // `!h` is provably not `h`. The `None` arm maps to a CONCRETE
            // hash, which a definition hashing to exactly `u64::MAX` would
            // match (1 in 2^64) — so the arms assert the premise
            // (`!report.removes(..)`) rather than trusting it, and such a
            // collision fails loudly instead of silently pinning nothing.
            *hash = Some(match *hash {
                Some(h) => !h,
                None => u64::MAX,
            });
        }
    }
}

/// Drop every schema whose COMPOSED fixed section — its size after
/// `resolve_fixed_nested` inlines the recursively-fixed nested targets —
/// overflows the checked `#[repr(C)]` arithmetic, warning per drop.
///
/// The hole this closes: a composed-overflow
/// schema used as a nested TARGET panics inside `resolve_fixed_nested`
/// ITSELF — `resolved_fixed_of` materializes each fixed target via the
/// PANICKING `wire_fixed_size` — so [`retain_sizable_schemas`]'s
/// post-resolve pass never gets to run. E.g. `Inner =
/// float64[2305843009213693951]` (individually sizable: 8·(2⁶¹−1) fits
/// usize), `Outer = Inner[4]` (composed overflow), `Wrapper` nesting
/// `Outer`: resolving `Wrapper` sizes `Outer` and panics. Core's API
/// stays untouched, so this closes it caller-side, before
/// `resolve_fixed_nested` ever sees the set.
///
/// Faithfulness: the probe mirrors the resolver's semantics exactly —
/// lookup precedence (qualified exact → same package → bare `Header` →
/// `std_msgs/Header` → unambiguous global bare; duplicate definitions
/// later-wins with the bare index kept consistent; cycles conservatively
/// variable; String/Bytes/DynamicArray/unresolvable refs variable) — and
/// computes each recursively-FIXED schema's composed layout bottom-up
/// with core's OWN checked arithmetic
/// ([`MessageSchema::checked_wire_fixed_size`] over a clone whose nested
/// refs are stamped with the target's composed
/// [`NestedFixedInfo`](cerulion_core::codegen::NestedFixedInfo)), so a
/// schema this keeps is one the resolver sizes without overflow.
///
/// Runs to a FIXPOINT internally: dropping a schema can flip a bare
/// reference from ambiguous to resolved (or resolved to unresolvable),
/// changing survivors' composed layouts — each pass re-probes the reduced
/// set until nothing more drops (a dropping pass strictly shrinks the
/// set, so it terminates).
///
/// VARIABLE schemas whose own fixed section overflows are deliberately
/// NOT dropped here: `resolve_fixed_nested` never sizes a variable schema
/// (no panic risk inside resolution), and the post-resolve
/// [`retain_sizable_schemas`] pass catches them.
///
/// **Removal is index-precise, never name-keyed.** One qualified
/// name can be borne by several definitions — a
/// composed-overflowing `.msg` store `pkg/Type` beside a valid slash-named
/// workspace YAML `pkg/Type` entry (distinct resolver keys,
/// `(Some(pkg), Type)` vs `(None, "pkg/Type")`, so BOTH are live lookup
/// targets) — and the earlier name-keyed retain threw the valid winner
/// away with the overflowing loser, silently stripping it from the graph
/// hash and recorded wire-size maps. Now only the offending DEFINITION is
/// removed... unless the overflowing definition IS the name's winning one
/// (the LAST bearer — the definition every later-wins consumer serves), in
/// which case EVERY bearer of the name goes: a hostile winner must yield
/// NO entry, never silently resurrect a lower-precedence same-name
/// definition in its place.
///
/// Returns a [`RemovedDefinitions`] (the names-only
/// return could not express a PARTIAL removal, see the struct doc):
/// callers holding sibling copies of the same set scrub them via
/// [`RemovedDefinitions::removes`], never by name alone.
pub(crate) fn retain_composed_sizable_schemas(
    schemas: &mut Vec<MessageSchema>,
    context: &'static str,
) -> RemovedDefinitions {
    let mut all_dropped = RemovedDefinitions::default();
    loop {
        let overflow = cerulion_core::codegen::composed_overflow_indices(schemas);
        if overflow.is_empty() {
            // The ONLY call site of the fault-injection seam, and it
            // exists only in a `cargo test --lib` build: nothing an
            // integration test or the shipped binary links against can
            // reach it (see `force_scrub_fault`).
            #[cfg(test)]
            force_scrub_fault::degrade_if_armed(&mut all_dropped);
            return all_dropped;
        }
        for &idx in &overflow {
            tracing::warn!(
                schema = %schemas[idx].qualified_name(),
                context = %context,
                "schema's COMPOSED fixed section (after fixed-nested inlining) \
                 overflows — skipped before resolution (hostile or corrupt \
                 declaration; the rest still load)"
            );
        }
        let failing: BTreeSet<usize> = overflow.into_iter().collect();
        // `None`: the removal's identity state IS `schemas`. This loop
        // runs to a fixpoint, and cloning the full
        // combined set per pass to hand back an identical slice was pure
        // cost.
        all_dropped.merge(remove_failing_definitions(schemas, None, &failing, context));
    }
}

/// What a removal pass (or a fixpoint of passes) took out of a schema set.
///
/// Under the identity model a
/// qualified-name STRING can be borne by DISTINCT resolver identities — a
/// store `(Some(pkg), "Type")` beside a slash-named workspace YAML
/// `(None, "pkg/Type")` — so a FAILING definition whose name legitimately
/// SURVIVES via a different-identity valid winner is a **partial removal**
/// the old names-only return could not express. The registry's tier scrub
/// then never removed the rejected store definition, `FieldRegistry.schemas`
/// kept it, and a QUALIFIED nested ref — which binds the STORE identity
/// — made `LayoutResolver::new` panic materializing it.
///
/// The earlier invariant generalizes rather than breaks: a NAME is fully
/// present or fully gone *per what its string-keyed winners serve*, while
/// a DEFINITION that failed must be gone from every copy of the set —
/// so the contract carries BOTH: `fully_removed_names` (every bearer
/// gone — safe for name-keyed scrubs) and `removed_keys` (the resolver
/// identity of every removed definition — the identity-keyed scrub that
/// removes a partially-removed definition without evicting the name's
/// surviving winner). One `removes` predicate applies both, so callers
/// cannot consume half the contract.
#[derive(Debug, Default)]
pub(crate) struct RemovedDefinitions {
    /// Qualified names with NO surviving bearer.
    pub fully_removed_names: Vec<String>,
    /// The `(package, name)` resolver key of every removed definition
    /// (full removals included — redundant there, harmless: a same-key
    /// shadowed loser swept by the key is one resolution could rebind on
    /// the next pass, and a rejected identity must not resurrect).
    pub removed_keys: Vec<(Option<String>, String)>,
    /// EVERY removed definition itself — resolver key plus the
    /// definition's own hash, `None` when the definition cannot BE hashed
    /// (its size arithmetic overflows, which is one of the reasons a
    /// definition is removed at all — `schema_hash()` PANICS there, and
    /// this list is built from exactly the definitions most likely to hit
    /// it, so it goes through `checked_schema_hash`). The case it covers:
    /// a failing
    /// `by_key` WINNER that is not its string's winner (a store copy
    /// shadowing a built-in by key while a slash-named YAML twin bears the
    /// string) has a same-key survivor AND a surviving string bearer, so
    /// neither list above records it and the report would come back EMPTY —
    /// both consumers would read that as "nothing to scrub" and keep the hostile
    /// definition for `LayoutResolver::new` to panic on. This list is
    /// filled UNCONDITIONALLY for every removed index, so a non-empty
    /// removal is never an empty report; the hash keeps it precise (the
    /// shadowed built-in shares the key but not the fields). Two
    /// unhashable definitions of one identity are indistinguishable here
    /// and both scrub — which is right: both are hostile.
    ///
    /// Precise IN THE `identity_source` STATE, and only there. Recorded
    /// UNRESOLVED (the state every caller's predicate sees), recipe 3 folds
    /// only a nested target's NAME, so two same-identity definitions
    /// differing solely INSIDE a fixed-nested target hash alike and scrub
    /// together. They must already share `(package, name)` and their whole
    /// declared field list to collide, so the over-scrub is bounded to
    /// textually identical declarations.
    pub removed_definitions: Vec<(Option<String>, String, Option<u64>)>,
}

impl RemovedDefinitions {
    pub(crate) fn is_empty(&self) -> bool {
        self.fully_removed_names.is_empty()
            && self.removed_keys.is_empty()
            && self.removed_definitions.is_empty()
    }

    pub(crate) fn merge(&mut self, other: RemovedDefinitions) {
        for qname in other.fully_removed_names {
            if !self.fully_removed_names.contains(&qname) {
                self.fully_removed_names.push(qname);
            }
        }
        for key in other.removed_keys {
            if !self.removed_keys.contains(&key) {
                self.removed_keys.push(key);
            }
        }
        for def in other.removed_definitions {
            if !self.removed_definitions.contains(&def) {
                self.removed_definitions.push(def);
            }
        }
    }

    /// The IDENTITY-only half of [`Self::removes`]: fully-removed name, or a
    /// removed `(package, name)` from EITHER list — the hash conjunct
    /// dropped.
    ///
    /// This is the guaranteed-monotone fallback the scrub fixpoints use when
    /// a pass makes no progress. Every recorded identity was taken from a
    /// clone of the very set being scrubbed, so this predicate always matches
    /// at least one member: a pass that applies it STRICTLY shrinks the set,
    /// which is what makes those loops terminate unconditionally rather than
    /// on "every arm matched". The cost of the dropped conjunct is that a
    /// healthy same-identity sibling can go with the offender — a loud
    /// schema-unavailable degrade, never a definition left behind for
    /// `resolve_fixed_nested` to panic materializing.
    pub(crate) fn removes_identity(&self, schema: &MessageSchema) -> bool {
        let qname = schema.qualified_name();
        self.fully_removed_names.contains(&qname)
            || self
                .removed_keys
                .iter()
                .any(|(pkg, name)| pkg == &schema.package && name == &schema.name)
            || self
                .removed_definitions
                .iter()
                .any(|(pkg, name, _)| pkg == &schema.package && name == &schema.name)
    }

    /// True when `schema` is covered by this removal — by fully-removed
    /// NAME or by removed IDENTITY. The one predicate sibling-set scrubs
    /// go through.
    pub(crate) fn removes(&self, schema: &MessageSchema) -> bool {
        let qname = schema.qualified_name();
        if self.fully_removed_names.contains(&qname) {
            return true;
        }
        if self
            .removed_keys
            .iter()
            .any(|(pkg, name)| pkg == &schema.package && name == &schema.name)
        {
            return true;
        }
        // The exact definition: same identity AND same fields.
        // CHECKED, for the same reason the list is: this predicate is asked
        // about candidate sets that still HOLD the hostile definition, so a
        // panicking hash here would defeat the scrub that removes it. An
        // unhashable candidate matches an unhashable record of the same
        // identity — both are hostile, and both must go.
        let hash = schema.checked_schema_hash().ok();
        self.removed_definitions
            .iter()
            .any(|(pkg, name, h)| pkg == &schema.package && name == &schema.name && *h == hash)
    }
}

/// The ONE removal policy for a set of FAILING (unsizable) definition
/// indices over a schema set — shared by the composed preflight above and
/// the post-resolve leg in `replay_field_registry::compute_schema_hashes`
/// (two legs implementing this rule
/// independently is exactly how one of them would lose the
/// winner sweep).
///
/// The invariant it enforces (generalized later for the identity
/// model): **no consumer's binding — string-keyed later-wins
/// (`LayoutResolver`'s `by_qualified`, the hash-map collect) OR
/// identity-keyed (`resolve_fixed_nested`'s `by_key`) — may ever pick a
/// definition sizing rejected**; a NAME is fully present or fully gone
/// per what its string winners serve, while a rejected DEFINITION is gone
/// from every copy of the set. Concretely:
///
/// - a failing definition is removed INDEX-precisely (an earlier
///   name-keyed retain evicted a valid same-name winner along with its
///   overflowing shadowed loser);
/// - when the failing definition IS the name's winning (last) bearer,
///   EVERY bearer goes — a hostile winner must yield NO entry, never
///   silently hand the name to a lower-precedence copy, and never stay
///   behind for a later `layout_of` to materialize and PANIC on
///   (a post-resolve leg that dropped a failing winner from its
///   local clone while a surviving earlier bearer masked the name from the
///   returned scrub list would leave the registry's own schema set holding the bad
///   winner);
/// - warns each swept non-failing companion (callers warn the FAILING
///   indices themselves, with their leg-specific diagnostic).
///
/// Returns a [`RemovedDefinitions`]: the names FULLY removed (no
/// surviving bearer — under the winner-sweep rule, exactly the names
/// whose WINNER failed; safe for name-keyed sibling scrubs) PLUS the
/// resolver `(package, name)` key of every removed definition — because
/// a failing definition whose flattened name survives via a
/// DIFFERENT-identity valid winner (a store def beside a slash-named YAML
/// twin) is a PARTIAL removal that must still propagate to sibling copies
/// of the set, identity-keyed, or the rejected definition stays bindable
/// there (see the struct doc for the panic this caused).
pub(crate) fn remove_failing_definitions(
    schemas: &mut Vec<MessageSchema>,
    identity_source: Option<&[MessageSchema]>,
    failing: &BTreeSet<usize>,
    context: &'static str,
) -> RemovedDefinitions {
    // `identity_source` is the STATE the caller's `removes` predicate will be
    // applied to, index-aligned with `schemas`. `None` means "`schemas`
    // itself", which is every caller but one: the replay registry's
    // post-resolve leg removes from a RESOLVED clone while its tiers are
    // UNRESOLVED, and recording the resolved hash there made `removes`'
    // definition arm compare two different states of one schema — see the
    // field doc on `removed_definitions`.
    //
    // With a bare-slice parameter the
    // same-state caller — `retain_composed_sizable_schemas`, which runs a
    // FIXPOINT — would clone the whole combined set (the 254-message built-in
    // corpus plus the `.msg` store plus every workspace YAML entry) on EVERY
    // pass purely to hand back a slice identical to `schemas`. `None` says
    // "same state" without materializing that copy at all; the borrow
    // checker is the reason it is an `Option` rather than a defaulted
    // reference (`schemas` is already borrowed mutably here, so the caller
    // cannot pass `&*schemas`).
    if let Some(src) = identity_source {
        assert_eq!(
            schemas.len(),
            src.len(),
            "remove_failing_definitions: the identity source must be index-aligned \
             with the set being removed from (length is a PROXY for alignment \
             — two same-length unrelated sets pass this check)"
        );
    }
    let mut remove: BTreeSet<usize> = failing.clone();
    for &idx in failing {
        let qname = schemas[idx].qualified_name();
        let winner = (0..schemas.len())
            .rev()
            .find(|&j| schemas[j].qualified_name() == qname)
            .expect("idx itself bears the name");
        if idx == winner {
            for (j, s) in schemas.iter().enumerate() {
                if s.qualified_name() == qname {
                    remove.insert(j);
                }
            }
        }
    }
    for &idx in &remove {
        if !failing.contains(&idx) {
            tracing::warn!(
                schema = %schemas[idx].qualified_name(),
                context = %context,
                "definition removed alongside a same-qualified-name hostile \
                 winner — a dropped winning definition must yield NO entry, \
                 never silently hand its name to a lower-precedence copy"
            );
        }
    }
    // Record every removed definition's NAME (for the fully-removed set)
    // and resolver KEY (for the identity-keyed partial-removal scrub).
    let removed_qnames: Vec<String> = remove
        .iter()
        .map(|&idx| schemas[idx].qualified_name())
        .collect();
    let candidate_keys: Vec<(Option<String>, String)> = remove
        .iter()
        .map(|&idx| (schemas[idx].package.clone(), schemas[idx].name.clone()))
        .collect();
    // Every removed definition, by identity + hash — recorded
    // UNCONDITIONALLY, so a removal can never yield an empty report.
    let removed_definitions: Vec<(Option<String>, String, Option<u64>)> = remove
        .iter()
        .map(|&idx| {
            let s = &schemas[idx];
            // CHECKED: a definition removed for overflowing its own size
            // arithmetic cannot be hashed — `schema_hash()` panics on it,
            // and those are precisely the definitions this list is built
            // from. `None` still identifies it (an unhashable definition of
            // that identity), which is all the scrub needs.
            //
            // The HASH comes from `identity_source`, never from `schemas`:
            // the caller compares it against schemas in THAT state, and a
            // definition with a fixed-nested reference hashes differently
            // before and after `resolve_fixed_nested`. (Identity — package
            // and name — is state-invariant, so it is read from either.)
            let ident = match identity_source {
                Some(src) => &src[idx],
                None => s,
            };
            (
                s.package.clone(),
                s.name.clone(),
                ident.checked_schema_hash().ok(),
            )
        })
        .collect();
    for &idx in remove.iter().rev() {
        schemas.remove(idx);
    }
    let mut fully_removed: Vec<String> = Vec::new();
    for qname in removed_qnames {
        if !schemas.iter().any(|s| s.qualified_name() == qname) && !fully_removed.contains(&qname) {
            fully_removed.push(qname);
        }
    }
    // A removed key propagates ONLY when no SURVIVING definition bears the
    // same key: a same-key survivor means the removed one was a
    // `by_key`-shadowed loser resolution never binds (the suppress rule —
    // propagating its key would evict the valid same-identity winner);
    // no survivor means the IDENTITY itself was rejected and sibling
    // copies must drop it too (r10's cross-identity partial, and — as a
    // harmless redundancy — every full removal).
    let mut removed_keys: Vec<(Option<String>, String)> = Vec::new();
    for key in candidate_keys {
        let survives = schemas
            .iter()
            .any(|s| s.package == key.0 && s.name == key.1);
        if !survives && !removed_keys.contains(&key) {
            removed_keys.push(key);
        }
    }
    RemovedDefinitions {
        fully_removed_names: fully_removed,
        removed_keys,
        removed_definitions,
    }
}

/// Build a [`LayoutResolver`] over EVERY built-in ROS 2 message, paired
/// with the list of their qualified names (`"pkg/Name"`).
///
/// The resolver is constructed from
/// [`native_ros2_messages::BUILTIN_MSGS`] — the exact vendored `.msg`
/// text frozen at build time, the SAME text the generated `ShmMessage`
/// types compiled against — so CLI schema introspection resolves wire
/// layouts that can never skew from the linked-in types.
///
/// The full schema set is parsed and handed to [`LayoutResolver::new`]
/// BEFORE any [`LayoutResolver::layout_of`] call, so nested type
/// references (e.g. `geometry_msgs/Pose` → `Point` + `Quaternion`)
/// resolve against the whole set. The returned `Vec<String>` (in
/// registry order) lets a caller ENUMERATE every built-in;
/// `resolver.layout_of(name)` looks ONE up. Minimal by design — that
/// pair is exactly what the introspection command needs.
///
/// A `parse_rosmsg` failure is structurally impossible here: the text
/// already parsed at build time to emit the types this crate links
/// against. It panics loudly rather than silently degrade the registry
/// if that codegen invariant is ever broken.
pub fn builtin_layout_resolver() -> (LayoutResolver, Vec<String>) {
    let schemas = parse_builtin_schemas();
    // Qualified names are stable across resolution (resolve_fixed_nested
    // only sets `fixed` flags on nested fields; it never renames or
    // reorders), so collecting here — before the move into `new` —
    // yields the same keys `layout_of` accepts, in registry order.
    let names: Vec<String> = schemas.iter().map(|s| s.qualified_name()).collect();
    // `LayoutResolver::new` runs `resolve_fixed_nested` internally and
    // emits every resolution warning via `tracing::warn!`, so dropping
    // the returned Vec cannot silence a problem. For the vendored set it
    // is empty (the build script fails on any resolution warning).
    let (resolver, _warnings) = LayoutResolver::new(schemas);
    (resolver, names)
}

/// Parse every embedded registry entry into schema IR, in registry
/// order. Shared by [`builtin_layout_resolver`] and the
/// workspace-entry layout enrichment, which seeds a
/// resolver with built-ins + workspace schemas combined.
///
/// A `parse_rosmsg` failure is structurally impossible here: the text
/// already parsed at build time to emit the types this crate links
/// against. It panics loudly rather than silently degrade the registry
/// if that codegen invariant is ever broken.
pub(crate) fn parse_builtin_schemas() -> Vec<MessageSchema> {
    let entries = native_ros2_messages::BUILTIN_MSGS;
    let mut schemas = Vec::with_capacity(entries.len());
    for &(package, name, text) in entries {
        let schema = parse_rosmsg(text, name, Some(package))
            .unwrap_or_else(|e| panic!("built-in registry: failed to parse {package}/{name}: {e}"));
        schemas.push(schema);
    }
    schemas
}

/// Enumerate `schemas_dir`'s `*.yaml` files in sorted order (`read_dir`
/// order is OS-dependent — sorting keeps every consumer deterministic,
/// Principle #7). EVERY skip is logged: a missing directory is quiet
/// (`debug!` — legitimately absent), an unreadable directory, and an
/// unreadable directory ENTRY (never swallowed by
/// a `.flatten()`) are `warn!`-logged. Non-`.yaml` files are a filter,
/// not a skip.
/// **TOP-LEVEL ONLY, deliberately.** `schemas/*.yaml`,
/// sorted; a nested `schemas/<pkg>/<Type>.yaml` is NOT walked.
///
/// The SPELLING surfaces do reach a nested file — `workspace_lookup`'s stem
/// tier probes `schemas/<pkg>/<Type>.yaml` by exact path — so the two can
/// disagree, and where they could disagree HARMFULLY (a nested stem file
/// beside a `.msg` store twin) the spelling surfaces REFUSE the name on both
/// spellings, which makes the divergence unreachable through a validated run
/// (`msg_store_parity_test::r26_a_nested_stem_yaml_twin_is_refused_on_both_spellings`).
/// What remains is a nested file with NO twin: it binds as a port schema and
/// is absent from every map built on this walk — the recording fold, the
/// graph-run hash map, gateway serving, the replay registry index, AND
/// `cerulion schema list`'s workspace rows (a user-facing surface, not just
/// an advisory map). Those maps are ADVISORY, so absent means "no recorded
/// hash", never a wrong one,
/// and the shape is pinned by
/// `a_nested_yaml_binds_as_a_port_schema_but_is_absent_from_the_advisory_map`.
///
/// Widening this walk is therefore a deliberate decision about what every
/// consumer of it indexes (six call sites across three modules), not a bug fix — make it here, and
/// re-decide that pin with it.
pub(crate) fn workspace_yaml_files(schemas_dir: &Path) -> Vec<std::path::PathBuf> {
    let entries = match std::fs::read_dir(schemas_dir) {
        Ok(entries) => entries,
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => {
            tracing::debug!(dir = %schemas_dir.display(), "no schemas/ directory");
            return Vec::new();
        }
        Err(e) => {
            tracing::warn!(
                dir = %schemas_dir.display(),
                error = %e,
                "schemas/ directory present but unreadable — skipping workspace schemas"
            );
            return Vec::new();
        }
    };
    let mut files = Vec::new();
    for dirent in entries {
        let path = match dirent {
            Ok(entry) => entry.path(),
            Err(e) => {
                tracing::warn!(
                    dir = %schemas_dir.display(),
                    error = %e,
                    "unreadable directory entry — skipped"
                );
                continue;
            }
        };
        if path.extension().and_then(|e| e.to_str()) == Some("yaml") {
            files.push(path);
        }
    }
    files.sort();
    files
}

/// Overwrite the parse-time `wire_fixed_size`
/// and per-field `is_variable` on `entries` with canonically RESOLVED
/// values, so the workspace source reports the same layout the built-in
/// source (and real codegen) reports for the same type. The resolver is
/// seeded with the ONE shared resolution set ([`ResolutionTiers`] —
/// built-ins, then the `.msg` store, then workspace YAML, ascending
/// precedence so a workspace name-collision resolves workspace-first,
/// preflighted by [`preflight_resolution_set`]) — mirroring codegen's
/// resolve-over-the-full-set phase. A seed of
/// built-ins + YAML ONLY would make a YAML entry nesting a `.msg` store type
/// report a `wire fixed size` WITHOUT the store type's bytes while the
/// recording fold and replay both inline it — the same divergence class at another
/// size-fold site. A nested reference that stays unresolvable even against the
/// combined set keeps its parse-time classification (variable); the
/// unified renderer then flags it loudly on its field line as
/// `(… unknown type — no schema found)`.
///
/// The displayed HASH is deliberately NOT recomputed: it stays the
/// single-file unresolved-parse value, where a nested field contributes
/// only its schema name (see [`MessageSchema::schema_hash`] — the
/// fixed-nested `target_hash` fold and the `wire_fixed_size` input only
/// change for RESOLVED IR, which this function never hashes). So a
/// workspace entry with a fixed-nested field intentionally renders a
/// pair of DIFFERENT provenance: the RESOLVED wire fixed size (the true
/// inlined SHM layout, equal to what the built-in path and a generated
/// `WIRE_FIXED_SIZE` would report) next to the UNRESOLVED recipe hash
/// (the wire-contract value). Both are individually correct —
/// do not "fix" either to match the other's provenance.
///
/// An entry NAME declared in MORE THAN ONE workspace file
/// is a workspace inconsistency — which definition "wins" would be
/// arbitrary, so enrichment is SKIPPED for that name (the parse-time
/// classification and the loud nested marker stand) and a `warn!` names
/// the duplicate and its declaring files.
fn enrich_entries_with_resolved_layout(schemas_dir: &Path, entries: &mut [SchemaEntry]) {
    let tiers = ResolutionTiers::assemble(Some(schemas_dir));
    // Entry name → declaring workspace files, for duplicate detection
    // (the per-FILE grouping is exactly what the shared assembly keeps).
    let mut declared_in: BTreeMap<String, Vec<String>> = BTreeMap::new();
    for (path, parsed) in &tiers.yaml_by_file {
        let file = path
            .file_name()
            .map(|n| n.to_string_lossy().into_owned())
            .unwrap_or_default();
        for schema in parsed {
            declared_in
                .entry(normalize_schema(&schema.name))
                .or_default()
                .push(file.clone());
        }
    }

    for (name, files) in &declared_in {
        if files.len() > 1 {
            tracing::warn!(
                schema = %name,
                files = %files.join(", "),
                "duplicate workspace schema name — layout enrichment skipped for it \
                 (parse-time classification kept); rename one of the definitions"
            );
        }
    }

    let mut schemas = tiers.combined();
    preflight_resolution_set(&mut schemas, "schema info layout enrichment");
    let (mut resolver, _warnings) = LayoutResolver::new(schemas);
    for entry in entries.iter_mut() {
        // A cross-file duplicate keeps its parse-time
        // classification — resolving through whichever definition the
        // resolver's last-wins name map kept would be arbitrary. The
        // warn above already named the inconsistency.
        if declared_in
            .get(&normalize_schema(&entry.name))
            .is_some_and(|files| files.len() > 1)
        {
            continue;
        }
        // Workspace schemas are package-less: the entry name IS the
        // qualified lookup key. `None` is a defensive arm only — the
        // entry was parsed from this same directory moments ago, so
        // only a mid-call file change (TOCTOU) can miss; the
        // parse-time classification stands.
        let layout = resolver.layout_of(&entry.name);
        // Decision: a RESOLVED frame past the ceiling has no
        // hash a producer can stamp — the parse-time hash is withdrawn and
        // the entry renders the same unhashable marker `schema list` gives
        // its row (`representable_layout` below leaves the layout
        // unresolved, so the `(unresolved)` marker rides beside it).
        // A root the preflight
        // REMOVED (a fixed-nested YAML root whose composed layout overflows)
        // resolves to NO layout at all — that is unhashable exactly like a
        // resolved frame past the ceiling, and `schema list` already says
        // so for its row; `None` here is never a "keep the parse-time hash"
        // arm.
        if layout
            .as_ref()
            .is_none_or(|l| l.frame_prefix_exceeds_wire())
        {
            entry.schema_hash = None;
        }
        if let Some(layout) = representable_layout(layout, "schema info layout enrichment") {
            entry.wire_fixed_size = layout.fixed_size;
            entry.wire_fixed_size_resolved = true;
            for field in entry.fields.iter_mut() {
                field.is_variable = layout.variable_fields.iter().any(|v| v.name == field.name);
            }
        }
    }
}

/// Which source satisfied a unified `schema info` lookup.
///
/// Resolution precedence: `Workspace` (YAML) → `MsgStore` (`.msg` store) →
/// `Builtin` — workspace YAML wins over the store wins over the built-in
/// registry (the workspace-wins shadow semantics extended to the store).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SchemaSource {
    /// A workspace-local `schemas/<name>.yaml` file.
    Workspace,
    /// A workspace `.msg` store file `schemas/<pkg>/msg/<Type>.msg`
    /// (the folder the acquisition ladder materializes
    /// into).
    MsgStore,
    /// The embedded built-in ROS 2 registry
    /// ([`native_ros2_messages::BUILTIN_MSGS`]).
    Builtin,
}

/// Unified `schema info` result: the entries plus WHERE they came from.
#[derive(Debug)]
pub struct UnifiedSchemaInfo {
    /// Which source satisfied the lookup (workspace wins over built-in).
    pub source: SchemaSource,
    /// Qualified built-in name(s) (comma-separated when several) that the
    /// winning source shadows: a workspace YAML schema whose name equals a
    /// built-in's bare or qualified name — or a `.msg` store entry
    /// whose qualified name equals a built-in's — takes precedence in
    /// lookups, so the collision fact is carried here for the caller to
    /// surface (the CLI emits the loud warn). `None` for built-in results
    /// and for non-colliding workspace/store schemas.
    pub shadowed_builtin: Option<String>,
    /// The entries, in the source's declaration order.
    pub result: SchemaInfoResult,
    /// Where the winning source's payload lives, for the `source:` line —
    /// the workspace YAML relative path (`schemas/foo.yaml`) or the `.msg`
    /// store relative path (`schemas/<pkg>/msg/<Type>.msg`). `None` for
    /// built-ins (the `source:` line then reads just `built-in (ROS 2)`).
    pub source_detail: Option<String>,
    /// The schema SET the renderer resolves nested references against for
    /// recursive inline expansion: the built-in ROS 2 registry plus (when a
    /// workspace was present) every workspace YAML schema and every `.msg`
    /// store schema. Carried on the result because [`fmt::Display`] has no
    /// access to `schemas_dir`. Built once at lookup time; the renderer
    /// builds a [`LayoutResolver`] + a name→schema map from it per render.
    /// ORDER IS PRECEDENCE: least-priority first (built-ins, `.msg` store,
    /// workspace YAML); `build_resolution_context` folds it LAST-wins.
    pub schema_set: Vec<MessageSchema>,
}

impl UnifiedSchemaInfo {
    /// The one-line provenance string for the `source:` line — consistent
    /// across every source (built-in / workspace / store); the recursive
    /// tree below it is identical in shape regardless of which one won.
    fn source_line(&self) -> String {
        match (self.source, &self.source_detail) {
            (SchemaSource::Builtin, _) => "built-in (ROS 2)".to_string(),
            (SchemaSource::Workspace, Some(detail)) => format!("workspace ({detail})"),
            (SchemaSource::Workspace, None) => "workspace (schemas/*.yaml)".to_string(),
            (SchemaSource::MsgStore, Some(detail)) => format!("msg store ({detail})"),
            (SchemaSource::MsgStore, None) => "msg store (schemas/<pkg>/msg)".to_string(),
        }
    }
}

impl fmt::Display for UnifiedSchemaInfo {
    /// ONE renderer for EVERY source — built-in, workspace YAML, `.msg`
    /// store (and, via [`render_remote_schema`], network-resolved). Each
    /// entry prints in the identical shape: a header block (`schema:` /
    /// `source:` / optional `description:` / `wire fixed size:` / `hash:` /
    /// `fields:`) followed by the recursive field tree — every non-primitive
    /// field's schema is expanded inline, indented directly beneath it (see
    /// `render_schema_entry`).
    ///
    /// Provenance note: for a workspace entry with a
    /// fixed-nested field, the top-level wire fixed size is the canonically
    /// RESOLVED inlined layout while the hash is the recipe value
    /// over the UNRESOLVED single-file IR (nested fields contribute only
    /// their names). Both are correct and intentionally differ in
    /// provenance — see `enrich_entries_with_resolved_layout`. When
    /// enrichment did NOT run for an entry (duplicate-name skip /
    /// unresolvable arms), the size line carries a loud ` (unresolved)`
    /// suffix — a parse-time size must never print identically to a
    /// resolved one.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        // Build the resolution context ONCE per render from `schema_set`
        // (built-ins + workspace + store when a workspace was present): a
        // name→schema map for declaration-ordered field walking, and a
        // LayoutResolver for the fixed/variable classification of nested
        // fields. Both derive from the SAME winner map so the field list and
        // the classification can never disagree on a shadowed type.
        let (schemas, mut resolver) = build_resolution_context(&self.schema_set);
        let source = self.source_line();
        for entry in &self.result.entries {
            // A fresh `seen` set per entry: each top-level schema in a
            // multi-schema file is an independent document to the reader, so
            // a type expanded under entry 1 still expands under entry 2.
            let mut seen: BTreeSet<String> = BTreeSet::new();
            render_schema_entry(f, entry, &source, &schemas, &mut resolver, &mut seen)?;
        }
        Ok(())
    }
}

/// Build the ONE resolution context both the field walk and the layout
/// classification read from: a name→schema winner map + a [`LayoutResolver`]
/// built from THAT SAME winner set.
///
/// The single-winner-map discipline is load-bearing: the field LIST comes
/// from the map (`schemas.get`) while the fixed/variable CLASSIFICATION comes
/// from `resolver.layout_of`. If the two indexes disagreed on which schema
/// wins a shadowed qualified key (e.g. a `.msg`-store `sensor_msgs/PointField`
/// shadowing the built-in), a nested expansion would render one schema's field
/// NAMES with the other's classification. Deriving the resolver from the
/// winner map's values makes that divergence structurally impossible.
///
/// The winner is LAST-wins over `set`, so the CALLER's order is its
/// precedence: `resolution_schema_set` passes least-priority first —
/// built-ins, then the `.msg` store, then workspace YAML — so the shadow wins
/// (a workspace definition over a store twin of the same qualified name,
/// either over a built-in), matching what `schema info` itself selects for a
/// spelling and `LayoutResolver`'s own last-wins `by_qualified` map;
/// [`store_resolution_set`] passes the same three tiers store-LAST (a store
/// definition must be the string winner for its OWN qualified name);
/// `render_remote_schema` passes built-ins then the served docs, and the
/// workspace-less arm built-ins alone.
fn build_resolution_context(
    set: &[MessageSchema],
) -> (BTreeMap<String, MessageSchema>, LayoutResolver) {
    let mut winners: BTreeMap<String, MessageSchema> = BTreeMap::new();
    for schema in set {
        // Last-wins: a later (higher-priority) schema overwrites the slot.
        winners.insert(schema.qualified_name(), schema.clone());
    }
    let (resolver, _warnings) = LayoutResolver::new(winners.values().cloned().collect());
    (winners, resolver)
}

/// One indent level in the schema-info tree = two spaces.
const TREE_INDENT: &str = "  ";

/// Render ONE schema entry: the header block, then its recursive field tree.
///
/// The header (`schema:` / `source:` / optional `description:` / `wire fixed
/// size:` / `hash:` / `fields:`) carries the ENTRY's own precomputed
/// top-level layout (unchanged per source — see [`SchemaEntry`]); `hash` and
/// `wire fixed size` are TOP-LEVEL ONLY (the wire identity of the message you
/// asked about; nested types' own hashes/sizes are implementation detail
/// already folded into the parent and would bury the structure). Every field
/// line at every depth keeps its `(fixed|variable)` classification — the
/// useful, scannable per-field signal.
///
/// `seen` tracks every qualified type already expanded in THIS entry;
/// first-occurrence expands fully, later occurrences of the same type print
/// `(…, expanded above)` and do NOT recurse (which also breaks any cycle in a
/// hand-authored workspace schema — ROS `.msg` types are acyclic).
fn render_schema_entry(
    out: &mut impl fmt::Write,
    entry: &SchemaEntry,
    source: &str,
    schemas: &BTreeMap<String, MessageSchema>,
    resolver: &mut LayoutResolver,
    seen: &mut BTreeSet<String>,
) -> fmt::Result {
    writeln!(out, "schema: {}", entry.name)?;
    writeln!(out, "source: {source}")?;
    if !entry.description.is_empty() {
        writeln!(out, "description: {}", entry.description)?;
    }
    let unresolved = if entry.wire_fixed_size_resolved {
        ""
    } else {
        " (unresolved)"
    };
    writeln!(
        out,
        "wire fixed size: {} bytes{}",
        entry.wire_fixed_size, unresolved
    )?;
    writeln!(out, "hash: {}", render_listing_hash(entry.schema_hash))?;
    writeln!(out, "fields: {}", entry.field_count)?;

    // Seed `seen` with the root's own type so a self-referential field
    // renders `(expanded above)` instead of recursing forever.
    seen.insert(entry.name.clone());
    let parent_package = entry.name.rsplit_once('/').map(|(pkg, _)| pkg);
    for field in &entry.fields {
        render_field_tree(
            out,
            &field.name,
            &field.field_type,
            field.is_variable,
            parent_package,
            1,
            schemas,
            resolver,
            seen,
        )?;
    }
    Ok(())
}

/// Render one field line and, for a not-yet-expanded resolvable nested type,
/// its inline sub-tree beneath it. Primitives never expand. Arrays keep their
/// `Type[N]` / `Type[]` syntax on the field line and expand the ELEMENT type
/// once beneath. `depth` ≥ 1 (top-level fields are depth 1).
#[allow(clippy::too_many_arguments)]
fn render_field_tree(
    out: &mut impl fmt::Write,
    name: &str,
    field_type: &FieldType,
    is_variable: bool,
    parent_package: Option<&str>,
    depth: usize,
    schemas: &BTreeMap<String, MessageSchema>,
    resolver: &mut LayoutResolver,
    seen: &mut BTreeSet<String>,
) -> fmt::Result {
    let indent = TREE_INDENT.repeat(depth);
    let ty = field_type.canonical_str();
    let class = if is_variable { "variable" } else { "fixed" };

    // Only a nested reference (or an array whose element is nested) expands.
    if nested_ref(field_type).is_none() {
        return writeln!(out, "{indent}{name}: {ty} ({class})");
    }
    match resolve_nested_key(field_type, parent_package, schemas) {
        // Not a primitive, and no schema for it anywhere — a typo or a type
        // the resolver has never seen. Flag it loudly (the loud-over-silent
        // rule the old `(treated as nested schema reference)` marker served).
        None => writeln!(
            out,
            "{indent}{name}: {ty} ({class}, unknown type — no schema found)"
        ),
        Some(key) if seen.contains(&key) => {
            writeln!(out, "{indent}{name}: {ty} ({class}, expanded above)")
        }
        Some(key) => {
            writeln!(out, "{indent}{name}: {ty} ({class})")?;
            seen.insert(key.clone());
            expand_nested_type(out, &key, depth + 1, schemas, resolver, seen)
        }
    }
}

/// Expand a nested type's fields inline (declaration order from the raw
/// schema; fixed/variable from the resolved wire layout — a recursively-fixed
/// nested type inlines into its parent's fixed section, which parse-time
/// classification cannot know).
fn expand_nested_type(
    out: &mut impl fmt::Write,
    qualified: &str,
    depth: usize,
    schemas: &BTreeMap<String, MessageSchema>,
    resolver: &mut LayoutResolver,
    seen: &mut BTreeSet<String>,
) -> fmt::Result {
    // Absent from the set: the caller already flagged the field line; nothing
    // to expand. (Defensive — `resolve_nested_key` only returns present keys.)
    let Some(schema) = schemas.get(qualified) else {
        return Ok(());
    };
    let variable: BTreeSet<String> = resolver
        .layout_of(qualified)
        .map(|layout| layout.variable_fields.into_iter().map(|v| v.name).collect())
        .unwrap_or_default();
    let parent_package = qualified.rsplit_once('/').map(|(pkg, _)| pkg);
    // Clone the field list so the resolver (needed mutably in the recursion)
    // isn't aliased by a borrow of `schemas` — cold one-shot introspection.
    let fields = schema.fields.clone();
    for field in &fields {
        let is_variable = variable.contains(&field.name);
        render_field_tree(
            out,
            &field.name,
            &field.field_type,
            is_variable,
            parent_package,
            depth,
            schemas,
            resolver,
            seen,
        )?;
    }
    Ok(())
}

/// The innermost nested reference `(schema_name, package)` of a field type,
/// unwrapping any array nesting. `None` for primitives / strings / bytes.
fn nested_ref(field_type: &FieldType) -> Option<(&str, Option<&str>)> {
    match field_type {
        FieldType::Nested {
            schema_name,
            package,
            ..
        } => Some((schema_name.as_str(), package.as_deref())),
        FieldType::FixedArray { element_type, .. } | FieldType::DynamicArray { element_type } => {
            nested_ref(element_type)
        }
        _ => None,
    }
}

/// Resolve a field's nested reference to the qualified KEY present in
/// `schemas`, applying rosidl same-package semantics (the display analogue of
/// `resolve_fixed_nested`'s lookup):
///
/// 1. a QUALIFIED reference (`pkg/Name`) resolves EXACTLY;
/// 2. an UNQUALIFIED reference tries the parent's package first
///    (`parent_pkg/Name` — ROS2 "same package" default), then
/// 3. the BARE name (flat-namespace workspace schemas), then
/// 4. a UNIQUE `*/Name` suffix match (the one legacy cross-package bare
///    binding rosidl accepts, e.g. `Header` → `std_msgs/Header`); an
///    AMBIGUOUS suffix is left unresolved rather than guessed.
///
/// `None` for a primitive, or a reference that matches nothing in the set.
fn resolve_nested_key(
    field_type: &FieldType,
    parent_package: Option<&str>,
    schemas: &BTreeMap<String, MessageSchema>,
) -> Option<String> {
    let (schema_name, package) = nested_ref(field_type)?;

    // 1. Qualified: exact only.
    if let Some(pkg) = package {
        let qualified = format!("{pkg}/{schema_name}");
        return schemas.contains_key(&qualified).then_some(qualified);
    }
    // 2. Unqualified: parent package first.
    if let Some(pp) = parent_package {
        let qualified = format!("{pp}/{schema_name}");
        if schemas.contains_key(&qualified) {
            return Some(qualified);
        }
    }
    // 3. Bare (flat-namespace workspace schemas).
    if schemas.contains_key(schema_name) {
        return Some(schema_name.to_string());
    }
    // 4. Unique `*/Name` suffix (legacy cross-package bare, e.g. Header).
    let mut suffix = schemas
        .keys()
        .filter(|key| key.rsplit_once('/').map(|(_, n)| n) == Some(schema_name));
    let first = suffix.next()?;
    match suffix.next() {
        None => Some(first.clone()),
        Some(_) => None, // ambiguous — do not guess (mirrors resolve.rs)
    }
}

/// Look up ONE built-in ROS 2 message by qualified name.
///
/// Accepts both `sensor_msgs/Image` and `sensor_msgs::Image`. The entry
/// resolves against the embedded registry via
/// [`builtin_layout_resolver`], so the reported hash, fixed size, and
/// per-field fixed-vs-variable split equal the generated type's
/// `SCHEMA_HASH` / `WIRE_FIXED_SIZE` / offset-table layout.
///
/// Errors: a name that is not `package/Type`-shaped →
/// [`CliError::Validation`]; a well-formed name matching no built-in →
/// [`CliError::SchemaNotFound`].
pub fn builtin_schema_info(name: &str) -> CliResult<SchemaEntry> {
    let normalized = normalize_schema(name);
    let Some((pkg, msg)) = normalized.split_once('/') else {
        return Err(CliError::Validation(format!(
            "'{}' is not a qualified built-in schema name — use 'package/Type' or \
             'package::Type' (e.g. sensor_msgs/Image); workspace schemas are looked \
             up by file name in schemas/",
            name
        )));
    };
    if pkg.is_empty() || msg.is_empty() || msg.contains('/') {
        return Err(CliError::Validation(format!(
            "'{}' is not a valid 'package/Type' schema name (empty or extra segments)",
            name
        )));
    }

    let found = native_ros2_messages::BUILTIN_MSGS
        .iter()
        .find(|&&(p, n, _)| p == pkg && n == msg);
    let Some(&(_, _, text)) = found else {
        return Err(CliError::SchemaNotFound {
            name: name.to_string(),
            remedy: SCHEMA_INFO_REMEDY,
        });
    };

    // Field list in declaration order — the layout's fixed/variable
    // lists are each ordered but lose the interleaving, so the display
    // order comes from re-parsing the frozen registry text.
    let schema = parse_rosmsg(text, msg, Some(pkg))
        .unwrap_or_else(|e| panic!("built-in registry: failed to parse {pkg}/{msg}: {e}"));

    let (mut resolver, _names) = builtin_layout_resolver();
    let layout = resolver.layout_of(&normalized).unwrap_or_else(|| {
        panic!("built-in registry desync: '{normalized}' in BUILTIN_MSGS but not resolvable")
    });

    let fields: Vec<SchemaFieldEntry> = schema
        .fields
        .iter()
        .map(|field| SchemaFieldEntry {
            name: field.name.clone(),
            type_display: field.field_type.canonical_str(),
            is_nested: field_type_is_nested(&field.field_type),
            // The resolved layout is the truth for fixed-vs-variable: a
            // recursively-fixed nested reference inlines into the fixed
            // section, which parse-time classification cannot know.
            is_variable: layout.variable_fields.iter().any(|v| v.name == field.name),
            field_type: field.field_type.clone(),
        })
        .collect();

    Ok(SchemaEntry {
        name: normalized,
        // `.msg` sources carry no parsed description.
        description: String::new(),
        field_count: fields.len(),
        fields,
        schema_hash: Some(layout.schema_hash),
        wire_fixed_size: layout.fixed_size,
        // Straight off the resolver — built-ins are always resolved.
        wire_fixed_size_resolved: true,
    })
}

/// Decide whether a name that resolved NEITHER workspace NOR
/// built-in is a candidate for a REMOTE `schema` fetch, returning the fetch target
/// (the requested type name fed to the `schema` verb).
/// The desk can fetch a QUALIFIED `pkg/Type` (two chunks) OR a package-less bare
/// `Name` (one chunk — a `cerulion schema create <Name>` workspace type the robot
/// serves under its bare key), so BOTH are candidates; the `::` form is
/// normalized to `/`. Returns `None` only for an empty name, a 3+-segment name
/// (`pkg/msg/Type` — not the 2-segment wire form), or any empty chunk. PURE —
/// oracle-tested.
pub fn remote_fetch_target(name: &str) -> Option<String> {
    let normalized = normalize_schema(name);
    if normalized.is_empty() {
        return None;
    }
    // Accept exactly one OR two non-empty `/`-separated chunks (`Name` or
    // `pkg/Type`); reject 3+ segments and any empty chunk.
    let chunks: Vec<&str> = normalized.split('/').collect();
    if chunks.len() > 2 || chunks.iter().any(|c| c.is_empty()) {
        return None;
    }
    Some(normalized)
}

/// The unified renderer: render a REMOTE `schema` reply for
/// `schema info` in the SAME tree shape as a local built-in / workspace /
/// store result — a header block (`schema:` / `source:` / `wire fixed size:` /
/// `hash:` / `fields:`) followed by the recursive field tree.
///
/// The served docs are parsed into schema IR and seeded ALONGSIDE the built-in
/// registry, so nested references to built-in types (e.g. `std_msgs/Header`)
/// expand exactly like custom ones. The `source:` line names the serving robot
/// and states plainly the type is NOT compiled locally, so a user is never
/// misled into thinking it is a local type.
///
/// Robustness (never lose the served text): if the requested root doc cannot
/// be parsed/resolved into IR, this falls back to the header + the verbatim
/// served closure. Any served CUSTOM doc not reached by walking the root's
/// field references (a defensively over-declared closure — should not happen
/// with a well-formed reply) is appended as its own tree block. PURE.
pub fn render_remote_schema(reply: &cerulion_core::SchemaReply) -> String {
    use std::fmt::Write as _;

    // Parse every served doc into schema IR; seed with the built-in registry
    // so nested built-in references resolve + expand. A doc that fails to
    // parse is recorded (never silently dropped) — it just won't expand.
    let mut schema_set = parse_builtin_schemas();
    for doc in &reply.docs {
        if let Some(schema) = parse_remote_doc(doc) {
            schema_set.push(schema);
        } else {
            tracing::warn!(
                schema = %doc.qualified,
                "served schema doc could not be parsed — it will not expand in the tree"
            );
        }
    }
    // Built-ins are seeded FIRST, served docs after — so a served doc that
    // shadows a built-in wins (last-wins), and the field list + classification
    // both derive from the same winner map.
    //
    // A served closure is untrusted input, and
    // `build_resolution_context` builds a `LayoutResolver` — whose
    // `resolve_fixed_nested` PANICS materializing a composed-overflow doc
    // referenced as a fixed nested target, and whose `layout_of` overflows on
    // a declared-huge one — so without a preflight the verbatim fallback below would be unreachable
    // for exactly the hostile reply it exists for. The ONE shared preflight
    // thins the set first; a rejected root simply misses the winner map and
    // takes the verbatim fallback, a rejected dependency leaves its
    // referencing field variable (the `(… unknown type)` marker).
    preflight_resolution_set(&mut schema_set, "remote schema render");
    let (schemas, mut resolver) = build_resolution_context(&schema_set);

    let source = format!(
        "robot '{}' over the network (not compiled locally; {} doc(s) in its closure)",
        reply.robot,
        reply.docs.len()
    );

    let mut out = String::new();
    let Some(root) = build_remote_entry(&reply.requested, &schemas, &mut resolver) else {
        // Root unparseable/unresolvable — keep the same provenance header but
        // fall back to the verbatim served text (never lose the user's data).
        let _ = writeln!(out, "schema: {}", reply.requested);
        let _ = writeln!(out, "source: {source}");
        let _ = writeln!(
            out,
            "note: could not parse the served schema text — showing it verbatim below"
        );
        for doc in &reply.docs {
            let enc = match doc.encoding {
                cerulion_core::SchemaEncoding::Msg => "msg",
                cerulion_core::SchemaEncoding::Yaml => "yaml",
            };
            let _ = writeln!(out, "\n--- {} (encoding: {}) ---", doc.qualified, enc);
            for line in doc.text.lines() {
                let _ = writeln!(out, "  {line}");
            }
        }
        return out;
    };

    // ONE shared `seen` across the root tree + any appended closure member, so
    // a type shown once is never re-expanded.
    let mut seen: BTreeSet<String> = BTreeSet::new();
    let _ = render_schema_entry(&mut out, &root, &source, &schemas, &mut resolver, &mut seen);

    // Defensive: append any served CUSTOM doc the field walk never reached.
    // With an accurate `deps` closure every doc is reachable and this is inert.
    for doc in &reply.docs {
        if seen.contains(&doc.qualified) {
            continue;
        }
        if let Some(entry) = build_remote_entry(&doc.qualified, &schemas, &mut resolver) {
            let note = format!(
                "robot '{}' over the network (closure member, not referenced by {})",
                reply.robot, reply.requested
            );
            let _ = writeln!(out);
            let _ =
                render_schema_entry(&mut out, &entry, &note, &schemas, &mut resolver, &mut seen);
        }
    }
    out
}

/// Parse ONE served [`cerulion_core::SchemaDoc`] into schema IR by its
/// encoding (`.msg` → `parse_rosmsg`, YAML → the workspace `schemas:` parser).
/// `None` on a parse failure (the caller warns + skips) — a served doc is
/// never coerced into a phantom schema.
fn parse_remote_doc(doc: &cerulion_core::SchemaDoc) -> Option<MessageSchema> {
    match doc.encoding {
        cerulion_core::SchemaEncoding::Msg => {
            let (package, name) = match doc.qualified.split_once('/') {
                Some((pkg, msg)) => (Some(pkg), msg),
                None => (None, doc.qualified.as_str()),
            };
            parse_rosmsg(&doc.text, name, package).ok()
        }
        cerulion_core::SchemaEncoding::Yaml => {
            let parsed = parse_message_schemas(&doc.text).ok()?;
            // A served YAML doc defines its one type; prefer the entry whose
            // qualified name matches the doc key, else the first (defensive).
            parsed
                .iter()
                .find(|s| s.qualified_name() == doc.qualified)
                .or_else(|| parsed.first())
                .cloned()
        }
    }
}

/// Build a top-level [`SchemaEntry`] for a resolver-resident type — the
/// network + defensive-append paths' analogue of the local per-source entry
/// builders. `None` when the type isn't in the set or has no wire layout.
fn build_remote_entry(
    qualified: &str,
    schemas: &BTreeMap<String, MessageSchema>,
    resolver: &mut LayoutResolver,
) -> Option<SchemaEntry> {
    let schema = schemas.get(qualified)?;
    // The RESOLVED-moment ceiling on this reader too: a served
    // closure that crosses the wire only through fixed-nested inlining must
    // take the verbatim fallback, never render a size no frame can carry.
    let layout = representable_layout(resolver.layout_of(qualified), "remote schema render")?;
    let variable: BTreeSet<&str> = layout
        .variable_fields
        .iter()
        .map(|v| v.name.as_str())
        .collect();
    let fields: Vec<SchemaFieldEntry> = schema
        .fields
        .iter()
        .map(|field| SchemaFieldEntry {
            name: field.name.clone(),
            type_display: field.field_type.canonical_str(),
            is_nested: field_type_is_nested(&field.field_type),
            is_variable: variable.contains(field.name.as_str()),
            field_type: field.field_type.clone(),
        })
        .collect();
    Some(SchemaEntry {
        name: qualified.to_string(),
        description: schema.description.clone().unwrap_or_default(),
        field_count: fields.len(),
        fields,
        schema_hash: Some(layout.schema_hash),
        wire_fixed_size: layout.fixed_size,
        // Resolved straight off the resolver (built-ins + the served closure).
        wire_fixed_size_resolved: true,
    })
}

/// Unified `schema info`: workspace YAML FIRST, then the `.msg` store,
/// then the embedded built-in ROS 2 registry.
///
/// Lookup order (the ONE workspace lookup — two tiers, entry then file
/// stem — then the store, then built-ins):
/// 1. WORKSPACE tiers (`workspace_lookup`, the order every ladder
///    applies): the ENTRY tier first — a top-level `schemas/*.yaml` whose
///    `schemas:` mapping declares the spelling, the name `schema list`
///    shows and shadow marking keys on, so every listed name is
///    introspectable; the result is that entry ALONE, sized alone
///    — exactly what the port surfaces bind and size, so a
///    hostile sibling entry never refuses a sane entry here and accepts it
///    there. Then the FILE-STEM tier — `schemas/<name>.yaml`, parsed
///    strictly, `source == Workspace` with ALL of the file's entries (a
///    file holding an over-ceiling entry is refused as a whole — "fix the
///    file"). `shadowed_builtin` carries the qualified name(s) of any
///    built-in the rendered entries shadow. A spelling that names more than
///    one DEFINITION is REFUSED by the lookup itself, naming every source
///    (ambiguity-refusal rule) — `schema info` shows the refusal like
///    every other surface, never "the first file"; a stem file that cannot
///    be read or parsed is [`CliError::SchemaUnchecked`], never "absent".
/// 2. STORE tier: the `.msg` store — a qualified `pkg/Type`
///    (or `pkg::Type`) hits a store key; a unique bare stem resolves,
///    an ambiguous one errors (`source == MsgStore`, `shadowed_builtin`
///    set when the store entry shadows a same-named built-in). The
///    nested-YAML/store twin of a bare spelling is refused first
///    (`refuse_ambiguous_bare`), and the selected definition's own layout
///    and tree resolve over the store-LAST `store_resolution_set`.
/// 3. Otherwise the name resolves as a built-in qualified name
///    (`pkg/Type` or `pkg::Type`) via [`builtin_schema_info`].
///
/// Errors: an ambiguous workspace spelling or bare store name →
/// [`CliError::Validation`]; an unreadable workspace stem file →
/// [`CliError::SchemaUnchecked`]; a non-qualified name matching nothing →
/// [`CliError::Validation`]; a qualified name matching no built-in →
/// [`CliError::SchemaNotFound`].
pub fn schema_info_unified(schemas_dir: &Path, name: &str) -> CliResult<UnifiedSchemaInfo> {
    // The set the unified renderer resolves nested references against, for
    // recursive inline expansion — built-ins + every workspace YAML schema +
    // every `.msg` store schema (computed once; only ONE return branch runs).
    let schema_set = resolution_schema_set(Some(schemas_dir));

    // NORMALIZE before probing the file stem. `pkg::Type` is a documented
    // spelling of the qualified name, and joining it RAW looked for a file
    // literally called `pkg::Type.yaml` — so `schema info acme::Widget` missed
    // `schemas/acme/Widget.yaml` while `port_schema_exists` (which normalizes)
    // found it. The graph gate accepted what this verb rejected: the same
    // ladder-disagreement as the other tiers, sign flipped.
    let normalized = normalize_schema(name);
    // The two workspace tiers, in the ONE order every ladder applies them
    // (`workspace_lookup`): the ENTRY a file declares first — a schema's
    // name is its entry — then the FILE of that stem. Stem-first made
    // `Foo` mean `Foo.yaml`'s sole entry `Bar` while another file declared
    // the entry `Foo`, so `schema info Foo` showed the wrong schema and
    // `graph validate` compared a valid alias against it.
    // An ambiguous spelling is REFUSED by the lookup itself (a loud
    // `Validation` error naming every source) — `schema info` shows it
    // like every other surface, never "the first file".
    let workspace_file = match workspace_lookup(schemas_dir, &normalized)? {
        Some(WorkspaceLookup::Entry { file }) => Some((file, true)),
        Some(WorkspaceLookup::Stem { file, .. }) => Some((file, false)),
        None => None,
    };
    if let Some((file, by_entry)) = workspace_file {
        // The stem `schema_info` joins back under `schemas/`, built from the
        // path's real COMPONENTS: on Unix a `\\` is a filename character, not
        // a separator, and rewriting it made `schema_info` open a different
        // file and report the schema missing.
        let relative = file
            .strip_prefix(schemas_dir)
            .unwrap_or(&file)
            .with_extension("");
        let stem = relative
            .components()
            .map(|component| component.as_os_str().to_string_lossy().into_owned())
            .collect::<Vec<_>>()
            .join("/");
        // Route through the same reader a file-stem lookup uses so the
        // matched entry carries the exact same hash + resolved layout
        // columns. The ENTRY view sizes that entry ALONE: a
        // hostile sibling of the same file refuses the FILE view
        // (`schema info <stem>`), never a port named after a sane entry
        // beside it — the file view and the entry view must not disagree
        // about the entry both render. The match is by the ONE canonical
        // identity (an entry keeps its DECLARED spelling, so a
        // `pkg::Type` entry answers to `pkg/Type`).
        let result = if by_entry {
            schema_info_entries(schemas_dir, &stem, Some(&normalized))?
        } else {
            schema_info(schemas_dir, &stem)?
        };
        let shadowed_builtin = shadowed_builtin_names(&result);
        return Ok(UnifiedSchemaInfo {
            source: SchemaSource::Workspace,
            shadowed_builtin,
            result,
            source_detail: Some(format!("schemas/{stem}.yaml")),
            schema_set,
        });
    }

    // The `.msg` store, between workspace YAML and the built-in
    // registry. A store entry whose qualified name equals a built-in shadows
    // it (store wins) — carried in `shadowed_builtin` exactly like a
    // workspace-YAML shadow.
    let store = SchemaStore::load(schemas_dir);
    // Every surface refuses the nested-YAML/store twin of a bare name.
    refuse_ambiguous_bare(schemas_dir, &normalized)?;
    // The selected store definition's own layout resolves over the
    // store-last set (`store_resolution_set`), never the canonical YAML-last
    // `schema_set`. Its nested tree renders
    // over that SAME set — the renderer is string-addressed (its winner map
    // and `layout_of`), and under the canonical set a slash-named workspace
    // YAML twin of a store child (`pkg/Child`) would be the string winner, so a
    // store root's header would carry the store child's layout (core binds the
    // ref by identity) while the tree printed the YAML twin's fields. Over
    // the store-last set the string winners coincide with identity binding
    // for every reference a store root can make.
    let store_set = store_resolution_set(schemas_dir);
    // The bare arm is port resolution's ONE verdict
    // (`bare_store_port_binding`, which asks `workspace_lookup` who owns the
    // qualified string), so `schema info` refuses exactly the bare names
    // `resolve_port_schema` and `port_schema_exists` refuse — a sole bearer
    // whose qualified string a workspace definition claims (the
    // nested-file and same-spelling twins are `workspace_lookup`'s and
    // `refuse_ambiguous_bare`'s verdict above). The docs name `schema info` among
    // the refusing surfaces; the contract is the refusal, on all three.
    if let Some(entry) = store_schema_lookup(schemas_dir, &store, name, &store_set)? {
        let shadowed_builtin = if builtin_has_qualified(&entry.name) {
            Some(entry.name.clone())
        } else {
            None
        };
        let source_detail = store
            .get(&entry.name)
            .map(|stored| format!("schemas/{}", stored.relative_path));
        return Ok(UnifiedSchemaInfo {
            source: SchemaSource::MsgStore,
            shadowed_builtin,
            result: SchemaInfoResult {
                entries: vec![entry],
            },
            source_detail,
            schema_set: store_set,
        });
    }

    let entry = builtin_schema_info(name)?;
    Ok(UnifiedSchemaInfo {
        source: SchemaSource::Builtin,
        shadowed_builtin: None,
        result: SchemaInfoResult {
            entries: vec![entry],
        },
        source_detail: None,
        schema_set,
    })
}

/// Port resolution's BARE-name claim over the `.msg` store, as ONE rule
/// that reads: a bare name maps to a store definition iff EXACTLY ONE
/// parsed store schema bears it — `SchemaStore::matches_bare_name`'s
/// verdict, which `resolve_port_schema` / `port_schema_exists` bind
/// (unique) or REFUSE (ambiguous — `ambiguous_store_name`, so no graph can
/// declare an ambiguous bare name and no bag can carry one) — AND
/// that bearer's qualified spelling is not one the workspace-YAML tier
/// declares as an ENTRY (`yaml_claims`): a slash-named YAML `pkg/Type`
/// owns that string on every string-addressed surface, so validation
/// refuses the bare name ([`bare_store_port_binding`]) and no consumer may
/// alias it to the store twin. Every surface that keys a store definition
/// by a bare name — the recording fold's aliases, the graph-hash
/// map's aliases, gateway serving, the replay registry — derives
/// the claim from HERE, never from its own count. Judged over the PARSED
/// store and the PARSED YAML tier: a bearer a sizing retain later drops
/// still exists on disk and still makes the name ambiguous at validation
/// time; each consumer then intersects the claim with its own
/// survivors.
pub(crate) fn unique_bare_store_names(
    store: &[MessageSchema],
    yaml_claims: &BTreeMap<String, YamlClaim>,
) -> BTreeMap<String, String> {
    let mut by_bare: BTreeMap<String, Vec<String>> = BTreeMap::new();
    for schema in store.iter().filter(|s| s.package.is_some()) {
        by_bare
            .entry(schema.name.clone())
            .or_default()
            .push(schema.qualified_name());
    }
    by_bare
        .into_iter()
        .filter_map(|(bare, bearers)| match bearers.as_slice() {
            [one] if !yaml_tier_claims_string(yaml_claims, one) => Some((bare, one.clone())),
            _ => None,
        })
        .collect()
}

/// Does the workspace-YAML tier own the qualified STRING `qualified` (a
/// slash-named entry `pkg/Type` in a top-level workspace YAML file)? The ONE
/// spelling of that question: the port surfaces refuse a bare
/// store name on it ([`bare_store_port_binding`]), the store claim excludes
/// the bearer on it ([`unique_bare_store_names`]), and the string maps
/// withhold the store's qualified key on it — so the four cannot drift. A
/// file STEM can never contain `/`, so only an ENTRY claim can match a
/// qualified string (a REFUSED one — declared by several files — claims it
/// too, ambiguously); a nested `schemas/<pkg>/<Type>.yaml` file is NOT in the
/// claims (the tier walk is top-level only) and is therefore invisible to
/// every surface alike — the qualified-spelling drift that file creates is
/// pre-existing and separate.
pub(crate) fn yaml_tier_claims_string(
    claims: &BTreeMap<String, YamlClaim>,
    qualified: &str,
) -> bool {
    matches!(
        claims.get(qualified),
        Some(YamlClaim::Entry { .. }) | Some(YamlClaim::Refused(_))
    )
}

/// What the workspace-YAML tier CLAIMS for ONE spelling — a bare name or a
/// slash-named entry (`pkg/Type`) — and the DOCUMENT the claim binds, as ONE
/// verdict every surface consults:
/// validation (`resolve_port_schema`, `port_schema_exists`), `schema info` /
/// `schema list`, gateway serving, the graph-hash map, the recording fold
/// and the replay registry. The tier has two claims: the ENTRY tier (a
/// top-level `schemas/*.yaml` file whose `schemas:` mapping declares the
/// spelling) and the FILE-STEM tier (`schemas/<name>.yaml`, reachable
/// through its own entries — the like-named entry, else the file's sole
/// entry).
///
/// **Every AMBIGUOUS claim is REFUSED — on
/// every surface, naming every source — never resolved by a precedence.**
/// The four ambiguities ([`AmbiguityKind`]): an entry name declared by
/// several files; a stem file beside another file's entry of that name; a
/// stem file declaring several entries and none named like itself; a stem
/// file whose sole entry's own name is ambiguous (declared by another file
/// too, or the stem of another file) — the last one to a FIXPOINT, since
/// refusing a name refuses every stem bound to it. Refusal makes the
/// entry-vs-stem precedence question moot — every claim that survives has
/// exactly ONE source (a sibling lookup, `WorkspaceLookup`, orders entry
/// over stem and agrees on every surviving case). An unreadable or
/// unparseable file claims nothing, loudly: the stem tier ERRORS on it at
/// validation, and no graph can carry it; a file declaring no entries
/// claims nothing, silently.
#[derive(Debug, Clone)]
pub(crate) enum YamlClaim {
    /// Exactly one top-level file declares this spelling as an ENTRY, and no
    /// stem file of that name competes — the claim binds itself.
    Entry { file: std::path::PathBuf },
    /// `schemas/<name>.yaml` binds one entry OF THAT FILE — the like-named
    /// entry, else the file's sole entry — and that entry name has no other
    /// declarer. `file` is the walk path of the file.
    Stem {
        file: std::path::PathBuf,
        entry: String,
    },
    /// Claimed ambiguously — bound by nothing, anywhere; the sources say why.
    Refused(AmbiguousClaim),
}

/// Why a workspace spelling is refused ([`YamlClaim::Refused`]) in the
/// FOLD tiers (the recording fold, the replay registry, gateway serving):
/// each kind means the spelling binds nothing, anywhere, and the sources
/// say why. The RESOLVING surfaces (`schema info`, the two port surfaces)
/// take their verdict from `workspace_lookup` instead — one rule, one
/// message shape.
#[derive(Debug, Clone)]
pub(crate) enum AmbiguityKind {
    /// The entry name is declared by more than one workspace YAML file.
    DuplicateEntry,
    /// A file of that stem exists beside another file's entry of that name.
    StemVsEntry,
    /// The stem file declares several entries and none named like itself.
    MultiEntryStem,
    /// The stem file binds its sole entry, whose own NAME is ambiguous —
    /// declared by another file too, or itself the stem of another file —
    /// so the document the stem would bind is one no surface can address.
    StemEntryAmbiguous { entry: String },
}

/// One document a refused spelling could have meant: a file and its entry.
#[derive(Debug, Clone)]
pub(crate) struct ClaimSource {
    pub(crate) file: std::path::PathBuf,
    pub(crate) entry: String,
}

/// A refused workspace spelling: its name, why, and EVERY source — the one
/// refusal a FOLD reports ([`AmbiguousClaim::message`] for the text,
/// [`AmbiguousClaim::warn`] for the log line). The RESOLVING surfaces take
/// their verdict from `workspace_lookup` instead — this type carries no
/// `error()`, and the two kinds those surfaces also decide render through
/// the same [`ambiguous_workspace_spelling`] the lookup raises, so the two
/// halves cannot print different text for one condition.
#[derive(Debug, Clone)]
pub(crate) struct AmbiguousClaim {
    pub(crate) name: String,
    pub(crate) kind: AmbiguityKind,
    pub(crate) sources: Vec<ClaimSource>,
}

/// The ONE log line every fold emits for a refused workspace spelling (the
/// tracing-discipline const capture; the name, kind and sources ride
/// structured fields).
pub(crate) const AMBIGUOUS_WORKSPACE_NAME_MSG: &str = "the workspace YAML tier claims this name \
    ambiguously — it binds nothing on any surface (validation, schema info/list, serving, \
    recording, replay); declare it once";

fn workspace_file_name(path: &Path) -> String {
    path.file_name()
        .map(|n| n.to_string_lossy().into_owned())
        .unwrap_or_else(|| path.display().to_string())
}

impl AmbiguousClaim {
    /// `a.yaml:Goal, b.yaml:Goal` — every source, in the sorted walk order.
    pub(crate) fn sources_display(&self) -> String {
        self.sources
            .iter()
            .map(|s| format!("{}:{}", workspace_file_name(&s.file), s.entry))
            .collect::<Vec<_>>()
            .join(", ")
    }

    /// The refusal text.
    ///
    /// The two kinds the SPELLING surfaces also decide (`DuplicateEntry`,
    /// `StemVsEntry`) render through the ONE refusal those surfaces raise
    /// ([`ambiguous_workspace_spelling`]), so a user reads one shape whether
    /// the refusal reached them from `graph validate`, `schema info` or a
    /// `schema list` row. The claims index survives because the FOLDS index
    /// every name once and cannot pay a per-spelling lookup — it is a second
    /// INDEX, never a second message. (Its source list is the workspace-YAML
    /// tier's own: where a `.msg` store twin is also a source, the spelling
    /// surfaces' copy names it and this one does not — the index does not
    /// read the store.)
    ///
    /// The other two kinds are the folds' ALONE: a multi-entry stem and a
    /// stem whose sole entry is ambiguous both name exactly ONE file, so the
    /// spelling surfaces resolve them and only a MAP has to say which entry
    /// it could not pick. They keep the reason-carrying text.
    pub(crate) fn message(&self) -> String {
        let name = &self.name;
        let sources = self.sources_display();
        let (reason, remedy) = match &self.kind {
            AmbiguityKind::DuplicateEntry | AmbiguityKind::StemVsEntry => {
                // The claims walk is TOP-LEVEL only, so `schemas/<file name>`
                // is the workspace-relative path `source_label` would build.
                let labels: Vec<String> = self
                    .sources
                    .iter()
                    .map(|source| {
                        let contributes = if source.entry == *name {
                            format!("entry {name}")
                        } else {
                            format!("file stem; entries: {}", source.entry)
                        };
                        format!(
                            "schemas/{} ({contributes})",
                            workspace_file_name(&source.file)
                        )
                    })
                    .collect();
                return ambiguous_workspace_spelling(name, &labels).to_string();
            }
            AmbiguityKind::MultiEntryStem => (
                format!(
                    "the file '{name}.yaml' declares {} entries and none named '{name}'",
                    self.sources.len()
                ),
                "name the port after one entry".to_string(),
            ),
            AmbiguityKind::StemEntryAmbiguous { entry } => (
                format!(
                    "the file '{name}.yaml' binds its sole entry '{entry}', whose own name is \
                     ambiguous in the workspace schemas"
                ),
                format!("declare '{entry}' once, under a name no other file claims"),
            ),
        };
        format!(
            "'{name}' is ambiguous in the workspace schemas — {reason}: {sources}. It binds \
             nothing on any surface (validation, schema info/list, serving, recording, replay); \
             {remedy}."
        )
    }

    /// The fold-side report (the resolving surfaces raise
    /// [`ambiguous_workspace_spelling`] from `workspace_lookup` instead).
    pub(crate) fn warn(&self, context: &'static str) {
        tracing::warn!(
            schema = %self.name,
            kind = ?self.kind,
            sources = %self.sources_display(),
            refusal = %self.message(),
            context = %context,
            "{AMBIGUOUS_WORKSPACE_NAME_MSG}"
        );
    }
}

/// The claims of a set of PARSED workspace YAML files (in the sorted walk
/// order — every consumer's own `yaml_by_file`, so the file a claim names
/// is the exact file that consumer holds the definitions of). See
/// [`YamlClaim`] for the verdicts; every surface that must keep a
/// YAML-claimed name away from the store tier — the graph-hash map's and the
/// recording fold's store aliases, the replay registry's bare index, the
/// gateway's topic bindings, both port surfaces and `schema info` — derives
/// the claim from HERE.
pub(crate) fn workspace_yaml_bare_claims_of(
    files: &[(std::path::PathBuf, Vec<MessageSchema>)],
) -> BTreeMap<String, YamlClaim> {
    // Entry tier: every entry name → its declaring files (walk order).
    // Keyed by the ONE canonical identity: an entry
    // spelled `pkg::Type` claims `pkg/Type`, so two files spelling one name
    // both ways are a DUPLICATE entry, refused naming both — where raw keys
    // let validation accept a pair serving then keyed as one document. The
    // IR name itself stays as declared (it is the hash identity).
    let mut declarers: BTreeMap<String, Vec<&Path>> = BTreeMap::new();
    for (path, parsed) in files {
        for schema in parsed {
            declarers
                .entry(normalize_schema(&schema.name))
                .or_default()
                .push(path.as_path());
        }
    }
    let source = |file: &Path, entry: &str| ClaimSource {
        file: file.to_path_buf(),
        entry: entry.to_string(),
    };
    let mut claims: BTreeMap<String, YamlClaim> = BTreeMap::new();
    for (name, paths) in &declarers {
        let claim = match paths.as_slice() {
            [one] => YamlClaim::Entry {
                file: one.to_path_buf(),
            },
            many => YamlClaim::Refused(AmbiguousClaim {
                name: name.to_string(),
                kind: AmbiguityKind::DuplicateEntry,
                sources: many.iter().map(|p| source(p, name)).collect(),
            }),
        };
        claims.insert(name.to_string(), claim);
    }
    // Stem tier: `<stem>.yaml` binds one entry OF THAT FILE — or refuses.
    for (path, parsed) in files {
        let Some(stem) = path.file_stem().and_then(|s| s.to_str()) else {
            continue;
        };
        // Canonical identities for CLAIM keys; the DECLARED spelling rides
        // beside each one because a bound stem's `entry` is the name every
        // fold and the replay registry look the parsed definition up by
        // (a `Goal.yaml` holding only `pkg::Other`
        // would vanish from the hash map and the registry if the binding
        // carried the canonical `pkg/Other` instead).
        let entries: Vec<(String, &str)> = parsed
            .iter()
            .map(|s| (normalize_schema(&s.name), s.name.as_str()))
            .collect();
        if entries.is_empty() {
            // A file declaring nothing claims nothing — there is no source to
            // name, and no document its stem could bind; the stem falls
            // through to the lower tiers.
            continue;
        }
        let own_sources = || {
            entries
                .iter()
                .map(|(_, declared)| source(path, declared))
                .collect::<Vec<_>>()
        };
        let verdict = if entries.iter().any(|(canonical, _)| canonical == stem) {
            match claims.get(stem) {
                // The file declares its own name, uniquely: it binds itself.
                Some(YamlClaim::Entry { file }) if file == path => YamlClaim::Stem {
                    file: path.clone(),
                    entry: stem.to_string(),
                },
                // A duplicate entry (this file is among the declarers): the
                // refusal already names every source.
                _ => continue,
            }
        } else {
            match claims.get(stem) {
                Some(YamlClaim::Entry { file }) => YamlClaim::Refused(AmbiguousClaim {
                    name: stem.to_string(),
                    kind: AmbiguityKind::StemVsEntry,
                    sources: std::iter::once(source(file, stem))
                        .chain(own_sources())
                        .collect(),
                }),
                Some(YamlClaim::Refused(dup)) => YamlClaim::Refused(AmbiguousClaim {
                    name: stem.to_string(),
                    kind: AmbiguityKind::StemVsEntry,
                    sources: dup.sources.iter().cloned().chain(own_sources()).collect(),
                }),
                // Only `<stem>.yaml` itself can carry a stem claim of this
                // name, and the walk yields each file once.
                Some(YamlClaim::Stem { .. }) => continue,
                None => match entries.as_slice() {
                    // The sole entry binds — by its DECLARED spelling —
                    // provided its own NAME is unambiguous, which the pass
                    // below decides once every claim is known.
                    [(_, declared)] => YamlClaim::Stem {
                        file: path.clone(),
                        entry: (*declared).to_string(),
                    },
                    _ => YamlClaim::Refused(AmbiguousClaim {
                        name: stem.to_string(),
                        kind: AmbiguityKind::MultiEntryStem,
                        sources: own_sources(),
                    }),
                },
            }
        };
        claims.insert(stem.to_string(), verdict);
    }
    // A stem binds an entry only if that entry's own NAME is unambiguous:
    // every string-keyed surface addresses the bound document by the entry
    // name, so a stem bound to a refused name would validate a port that
    // serving, the folds and replay can never reach. The entry may be
    // refused as a duplicate (set by the entry pass) or as a stem-vs-entry
    // collision (set by the pass above, possibly AFTER the stem that binds
    // it was visited) — and refusing a stem can refuse a stem bound to IT,
    // so this runs to a fixpoint (each step refuses one more stem; it
    // terminates).
    loop {
        let refused_stem = claims.iter().find_map(|(stem, claim)| match claim {
            YamlClaim::Stem { entry, .. } => match claims.get(&normalize_schema(entry)) {
                Some(YamlClaim::Refused(inner)) => Some((
                    stem.clone(),
                    AmbiguousClaim {
                        name: stem.clone(),
                        kind: AmbiguityKind::StemEntryAmbiguous {
                            entry: entry.clone(),
                        },
                        sources: inner.sources.clone(),
                    },
                )),
                _ => None,
            },
            _ => None,
        });
        match refused_stem {
            Some((stem, ambiguous)) => {
                claims.insert(stem, YamlClaim::Refused(ambiguous));
            }
            None => break,
        }
    }
    claims
}

/// Every workspace YAML file that READS and PARSES, with its schemas, in the
/// sorted walk order — the parse half of [`ResolutionTiers::assemble`]. A
/// file that cannot be read or parsed CLAIMS NOTHING and is said loudly
/// here: the port surfaces derive their whole workspace verdict
/// from this walk, so a silent skip would turn "this sibling file is malformed"
/// into a bare "no such schema". Per call, so a `graph validate` over N ports repeats it N times
/// — a broken workspace file is a fix-now defect, and the repetition is
/// acceptable. The gateway's `build_schema_docs` walks the
/// directory again for the document TEXT and logs its own skip at `debug!`,
/// since this walk — run first there — has already said it. The `(path,
/// schemas)` pairs are what [`workspace_yaml_bare_claims_of`] keys a stem's
/// OWNER by.
pub(crate) fn parsed_workspace_yaml(
    schemas_dir: &Path,
) -> Vec<(std::path::PathBuf, Vec<MessageSchema>)> {
    let mut files = Vec::new();
    for path in workspace_yaml_files(schemas_dir) {
        let content = match std::fs::read_to_string(&path) {
            Ok(content) => content,
            Err(e) => {
                tracing::warn!(
                    file = %path.display(),
                    error = %e,
                    "could not read schema file for the workspace claim scan"
                );
                continue;
            }
        };
        let parsed = match parse_message_schemas(&content) {
            Ok(parsed) => parsed,
            Err(e) => {
                tracing::warn!(
                    file = %path.display(),
                    error = %e,
                    "could not parse schema file for the workspace claim scan"
                );
                continue;
            }
        };
        files.push((path, parsed));
    }
    files
}

/// One file-STEM claim of a [`workspace_yaml_bare_claims_of`] map, as the
/// consumers iterate it: the bare `stem`, the OWNING `file`, and the entry
/// OF THAT FILE the stem binds. A consumer resolves the binding through the
/// owner's own surviving definition — never through the entry
/// name's string winner (which, since every surviving claim has one source,
/// is the owner anyway) — and applies it AFTER its entry-name inserts.
#[derive(Debug, Clone, Copy)]
pub(crate) struct StemBinding<'a> {
    pub(crate) stem: &'a str,
    pub(crate) file: &'a Path,
    pub(crate) entry: &'a str,
}

/// Every BOUND file-STEM claim of `claims` (see [`StemBinding`]); refused
/// stems are reported by [`refused_claims`] instead.
pub(crate) fn stem_bindings(
    claims: &BTreeMap<String, YamlClaim>,
) -> impl Iterator<Item = StemBinding<'_>> {
    claims.iter().filter_map(|(stem, claim)| match claim {
        YamlClaim::Stem { file, entry } => Some(StemBinding {
            stem: stem.as_str(),
            file: file.as_path(),
            entry: entry.as_str(),
        }),
        YamlClaim::Entry { .. } | YamlClaim::Refused(_) => None,
    })
}

/// Every REFUSED claim of `claims` — what each fold warns about, once per
/// name, and what every resolving surface refuses with.
pub(crate) fn refused_claims(
    claims: &BTreeMap<String, YamlClaim>,
) -> impl Iterator<Item = &AmbiguousClaim> {
    claims.values().filter_map(|claim| match claim {
        YamlClaim::Refused(ambiguous) => Some(ambiguous),
        YamlClaim::Entry { .. } | YamlClaim::Stem { .. } => None,
    })
}

/// SIZE the definition `workspace_lookup` bound for a port spelling, on
/// both port surfaces: `Ok(())` when the bound document passes
/// the wire ceiling, `Err` when it does not ([`preflight_rejection_error`],
/// the same text `schema info` gives it).
///
/// The CLAIM is not re-decided here — `workspace_lookup` already refused an
/// ambiguous spelling and reported an unreadable stem file; this is the
/// half main's lookup has no equivalent for. Sizes the BOUND entry only —
/// a hostile sibling in the same file refuses the FILE view
/// (`schema info <stem>`), never a port named after a sane entry of that
/// file, by either spelling. A stem file declaring SEVERAL entries binds no
/// single definition, so there is nothing to size: the graph identity layer
/// is where such a spelling is judged (`graph_cmd`'s
/// `WorkspaceSchemaClaim::AmbiguousStem`). A stem file declaring NONE is
/// ACCEPTED here. The decision and BOTH its halves are stated in
/// `msg_store_parity_test::r27_a_stem_bound_to_an_ambiguous_entry_is_refused_to_a_fixpoint`'s
/// `Vacant.yaml` arm — the probe answers with the file, and the fold binds
/// nothing under the name. `workspace_schema_claim` maps that case to `None`
/// rather than `AmbiguousStem`, so no layer judges it: a documented gap, not
/// a refusal.
///
/// Deliberately the DECLARED moment: a layout that crosses the ceiling only
/// through fixed-nested inlining is caught by the folds' RESOLVED-moment
/// preflight (loudly — omitted from the hash map, recorded 0,
/// schema-unavailable at replay), not here, where a whole resolver per port
/// would defeat the cheap gate.
///
/// **A binding this cannot CHECK is REFUSED, never accepted.** A
/// fallback that `return Ok(())`s (ACCEPTING the port
/// binding) when the file `workspace_lookup` had just bound cannot be
/// re-read, or when the entry is not found in it, is a LOUD accept of
/// an UNVERIFIED definition, not a "never a silent accept of an unsized
/// definition": the gate's whole job is to answer
/// "present AND sized", and a warn on stderr does not stop the name from
/// being accepted by `graph validate` and then refused by every FOLD
/// downstream. All three conditions ride [`CliError::SchemaUnchecked`] —
/// main's shape for exactly "could not check", which each consumer frames
/// itself, exactly as `workspace_lookup`'s own unchecked refusals are framed
/// — with the file NAMED. They are kept APART because their remedies
/// differ: a file that no longer PARSES is fixed by fixing the
/// YAML, while a file that no longer HOLDS the entry is fixed by restoring
/// the declaration (or re-pointing the port). Folding them into one message
/// (`parse_message_schemas(&content).ok()` swallowing the parse error into
/// the missing-entry arm) told a user with a syntax error to go looking for
/// a deleted entry.
///
/// All three refusals are unreachable through a single-threaded
/// `workspace_lookup` call — it read the very same file microseconds
/// earlier — so this is a TOCTOU refusal, not a routine path: the file must
/// change between two reads.
fn workspace_binding_preflight(
    schemas_dir: &Path,
    lookup: &WorkspaceLookup,
    spelling: &str,
) -> CliResult<()> {
    let (file, entry) = match lookup {
        WorkspaceLookup::Entry { file } => (file, spelling.to_string()),
        // A stem file binds a definition only when it declares exactly one.
        WorkspaceLookup::Stem { file, entries } => match entries.as_slice() {
            [one] => (file, one.clone()),
            // NEITHER zero nor two-plus entries binds a single definition,
            // so there is nothing for THIS gate to size.
            //
            // A ZERO-entry stem still answers the existence probe with the
            // file — a back-compat decision ("exactly as it did before
            // any store existed"), pinned by
            // `msg_store_parity_test::r27_a_stem_bound_to_an_ambiguous_entry_is_refused_to_a_fixpoint`'s
            // `Vacant.yaml` arm, which also pins the acknowledged
            // consequence (the fold binds nothing under the name). Refusing
            // it here was tried and reverted: it is a decision, not an
            // oversight, and this gate is not the place to re-litigate it.
            _ => return Ok(()),
        },
    };
    // The ONE "could not check this binding" framing, shared by the three
    // arms below so a consumer renders one sentence shape whichever fired.
    //
    // It does NOT lead with the port spelling. `CliError::SchemaUnchecked`'s
    // contract is that the CONSUMER frames the name once
    // (`'<spelling>' could not be checked against this workspace (…)`), and
    // `workspace_lookup`'s own unchecked refusals honour it — so self-naming
    // here would print the spelling twice wherever both meet, which is what
    // `the_run_refuses_an_ambiguous_spelling_even_with_no_validate`
    // caught when it was tried.
    let unchecked = |reason: String| {
        CliError::SchemaUnchecked(format!(
            "could not verify this schema's size — {}: {reason}",
            source_label(schemas_dir, file, "bound by the port spelling")
        ))
    };
    let content = std::fs::read_to_string(file).map_err(|e| {
        unchecked(format!(
            "the file the workspace lookup bound could not be re-read ({e})"
        ))
    })?;
    let parsed = parse_message_schemas(&content)
        .map_err(|e| unchecked(format!("the file no longer parses ({e})")))?;
    // An ENTRY claim's key is canonical, a bound STEM's entry is the
    // declared spelling — match by the ONE identity either way.
    //
    // EVERY bearer, and ALL-OR-NOTHING — not the first, not the last, and not
    // "any one of them is fine". `pkg/Type` and `pkg::Type` are ONE identity
    // here but two distinct YAML keys, so a single file can declare two
    // bearers of the spelling, and picking either END of that list made this
    // gate disagree with its sibling surfaces in one declaration order or the
    // other.
    //
    // The rule is set by what those surfaces do with that shape:
    //
    // - the FOLD never serves it. `workspace_yaml_bare_claims_of` keys
    //   `declarers` by normalized identity, so two bearers in one file push
    //   the SAME path twice, take the `many` arm and yield
    //   `YamlClaim::Refused(AmbiguityKind::DuplicateEntry)`;
    //   `build_workspace_schema_hashes` then `continue`s past a refused key,
    //   so NEITHER bearer gets a map entry — healthy or hostile.
    // - `schema info` refuses it: `schema_info_entries` judges every entry the
    //   spelling matches and errors on the hostile one.
    //
    // So a gate that accepted because SOME bearer is representable would
    // accept exactly what `schema info` refuses — breaking the parity this
    // function's own rustdoc promises ("the same text `schema info` gives
    // it"). It refuses if ANY bearer is hostile.
    let bearers: Vec<MessageSchema> = parsed
        .into_iter()
        .filter(|s| normalize_schema(&s.name) == normalize_schema(&entry))
        .collect();
    if bearers.is_empty() {
        return Err(unchecked(format!("it no longer declares '{entry}'")));
    }
    if let Some(hostile) = bearers
        .iter()
        .find(|s| !wire_representable_or_warn(s, "port schema validation", SizeMoment::Declared))
    {
        // `workspace_relative_label`, NOT `workspace_file_name`: the latter
        // is basename-only, so a nested `schemas/pkg/Nested.yaml` rendered as
        // `schemas/Nested.yaml` — a path that does not exist — and item 4 of
        // this change pins nested files as a bindable shape. Bare, because
        // `preflight_rejection_error` parenthesises its location itself.
        return Err(preflight_rejection_error(
            &entry,
            &workspace_relative_label(schemas_dir, file),
            hostile,
        ));
    }
    Ok(())
}

/// The three tiers of the workspace schema RESOLUTION SET — the ONE
/// assembly every surface that resolves nested references against the
/// workspace goes through (the
/// one-builder rule applied to set assembly): the built-in ROS 2 registry
/// ALWAYS, plus (when a workspace is present) every `.msg` store schema
/// and every workspace YAML schema, enumerated through the shared seams
/// ([`SchemaStore::load`] and the sorted [`workspace_yaml_files`] walk),
/// so no two consumers can disagree on membership or on which duplicate
/// wins. Consumers: [`resolution_schema_set`] (schema info/list), the
/// YAML-entry layout enrichment, `graph_cmd`'s recorded wire-size fold,
/// `replay_field_registry::FieldRegistry::from_graph`, and — through
/// [`resolution_schema_set`] — `topic echo`'s local walker. The pre-r14
/// recording fold assembled store + YAML WITHOUT the built-ins, so a
/// store schema whose fixed-nested ref targets a built-in recorded a
/// `wire_fixed_size` missing the built-in's bytes while replay's combined
/// set inlined them — the descriptor disagreed with the layout replay
/// used.
///
/// Tiers are kept SEPARATE (never pre-concatenated) because two consumers
/// need per-tier facts: the recording fold judges bare-name ambiguity over
/// the PARSED store and serves workspace definitions
/// only; the replay registry scrubs tiers by [`RemovedDefinitions`]
/// across its fixpoint. [`Self::combined`] is the ascending-precedence
/// concatenation every last-wins string map expects.
///
/// A malformed/unreadable workspace file is warned + skipped (never
/// fatal) — ONE message site for every surface; the `.msg` store's own
/// per-file robustness contract lives in [`SchemaStore::load`].
pub(crate) struct ResolutionTiers {
    /// The built-in ROS 2 registry, in registry order.
    pub(crate) builtin: Vec<MessageSchema>,
    /// Every `.msg` store schema, in sorted qualified-name order.
    pub(crate) store: Vec<MessageSchema>,
    /// Workspace YAML schemas grouped by DECLARING FILE, in the sorted
    /// walk order (the layout enrichment reports cross-file duplicates by
    /// file name).
    pub(crate) yaml_by_file: Vec<(std::path::PathBuf, Vec<MessageSchema>)>,
}

impl ResolutionTiers {
    /// Assemble the tiers. `None` = no workspace: built-ins only (the
    /// no-workspace `schema info` path never reads a stray
    /// `schemas/` from the cwd).
    pub(crate) fn assemble(schemas_dir: Option<&Path>) -> Self {
        let builtin = parse_builtin_schemas();
        let Some(dir) = schemas_dir else {
            return Self {
                builtin,
                store: Vec::new(),
                yaml_by_file: Vec::new(),
            };
        };
        let store = SchemaStore::load(dir).schemas();
        let mut yaml_by_file = Vec::new();
        for path in workspace_yaml_files(dir) {
            let content = match std::fs::read_to_string(&path) {
                Ok(content) => content,
                Err(e) => {
                    tracing::warn!(
                        file = %path.display(),
                        error = %e,
                        "could not READ this workspace schema file — its schemas are \
                         skipped on every resolution surface (the rest still load)"
                    );
                    continue;
                }
            };
            match parse_message_schemas(&content) {
                Ok(parsed) => yaml_by_file.push((path, parsed)),
                Err(e) => tracing::warn!(
                    file = %path.display(),
                    error = %e,
                    "could not PARSE this workspace schema file — its schemas are \
                     skipped on every resolution surface (the rest still load)"
                ),
            }
        }
        Self {
            builtin,
            store,
            yaml_by_file,
        }
    }

    /// The workspace YAML tier flattened, in walk order.
    pub(crate) fn yaml(&self) -> impl Iterator<Item = &MessageSchema> {
        self.yaml_by_file
            .iter()
            .flat_map(|(_, parsed)| parsed.iter())
    }

    /// Every tier in ONE vec, ASCENDING precedence — built-ins, then the
    /// store, then workspace YAML — so every last-wins string map built
    /// over it (`build_resolution_context`'s dedup, `LayoutResolver`'s
    /// `by_qualified`, the recording fold's winner map) expresses the
    /// documented workspace-YAML → store → built-in ladder.
    pub(crate) fn combined(&self) -> Vec<MessageSchema> {
        self.builtin
            .iter()
            .chain(self.store.iter())
            .chain(self.yaml())
            .cloned()
            .collect()
    }

    /// The same three tiers with the `.msg` store LAST — built-ins, then
    /// workspace YAML, then the store — for the ONE consumer that must
    /// report a STORE DEFINITION's own identity under its qualified name:
    /// `schema list`'s store rows. Under the
    /// canonical order a slash-named workspace YAML entry `pkg/Type` is the
    /// resolver's last-wins STRING winner over a store file of the same
    /// qualified name, so `layout_of("pkg/Type")` would hand the YAML layout to
    /// a row that names the STORE PATH — the YAML hash printed beside a
    /// file that does not define it. Nested-reference BINDING is untouched
    /// by this reordering: core binds by resolver identity `(package,
    /// name)`, and the store `(Some(pkg), Type)` and the YAML
    /// `(None, "pkg/Type")` are distinct identities whichever comes last —
    /// only the string-keyed winner map moves.
    pub(crate) fn combined_store_last(&self) -> Vec<MessageSchema> {
        self.builtin
            .iter()
            .chain(self.yaml())
            .chain(self.store.iter())
            .cloned()
            .collect()
    }
}

/// PREFLIGHT a combined resolution set before ANY consumer hands it to
/// `LayoutResolver::new` / `resolve_fixed_nested` (the preflight class
/// at the resolution-set seam, the one seam every consumer calls): the constructor runs
/// `resolve_fixed_nested`, which PANICS materializing a composed-overflow
/// schema referenced as a fixed nested target (individually-valid
/// declarations whose fixed-nested COMPOSITION overflows), and
/// `layout_of` panics hashing an individually-hostile one. In order: the
/// declared-size overflow retain, the walker-arithmetic guard (a declared
/// frame prefix that overflows `usize` — not a declared
/// u32 ceiling retain, see the body), then the
/// composed probe. The u32 CEILING is judged by every consumer on the
/// RESOLVED layout it reads ([`representable_layout`] on this seam's
/// `LayoutResolver` readers): a set of `usize`-safe members can still
/// compose past the ceiling through fixed-nested inlining, and an
/// over-ceiling member must stay in the set so its parents compose over
/// and are refused WITH it. A schema this seam drops degrades to an
/// unresolved reference (the parse-time classification + ` (unresolved)`
/// suffix, a recorded `wire_fixed_size` of 0 with its loud warn), never a
/// panic and never a saturated number.
pub(crate) fn preflight_resolution_set(schemas: &mut Vec<MessageSchema>, context: &'static str) {
    retain_sizable_schemas(schemas, context);
    // The u32 wire CEILING is no longer judged
    // here. Dropping an over-ceiling member at the DECLARED moment removed a
    // nested TARGET before composition, so its parent resolved with that
    // reference left variable — a small, "representable" layout — and every
    // reader of this set (`schema list` store rows, `schema info`, the
    // remote render) served a hash and size for a frame no producer can
    // emit (the fold defect, at the resolver seam). What this seam
    // must exclude is ONLY what would PANIC the layout walker: a declared
    // section so close to `usize::MAX` that adding the 32-byte header and
    // the offset table overflows (`float64[2^61 − 1]` = 2^64 − 8 bytes,
    // measured). A member past the u32 ceiling but within `usize` stays,
    // composes, and is refused — with every parent that inlines it — by
    // `representable_layout` on the RESOLVED layout, the ONE place the
    // ceiling is judged for this set.
    schemas.retain(|schema| {
        let entries = offset_table_entries(schema, SizeMoment::Declared);
        if cerulion_core::wire::frame_prefix_size(schema.wire_fixed_size(), entries).is_some() {
            return true;
        }
        tracing::warn!(
            schema = %schema.qualified_name(),
            fixed_size = schema.wire_fixed_size(),
            offset_table_entries = entries,
            context = %context,
            "schema's declared frame prefix overflows usize — dropped before layout \
             resolution (hostile or corrupt declaration; the rest still resolve)"
        );
        false
    });
    retain_composed_sizable_schemas(schemas, context);
}

/// The resolution set for a STORE DEFINITION'S OWN layout — the same three
/// tiers as [`resolution_schema_set`], preflighted, ordered store-LAST
/// ([`ResolutionTiers::combined_store_last`]) so the store definition is the
/// string winner for its own qualified name. The
/// ordering serves `schema list`'s store rows AND the `schema info` BARE
/// tier; resolving the selected store
/// definition over the canonical YAML-last set would fail: with a bare `Type`
/// selecting store `pkg/Type` beside a slash-named workspace YAML entry
/// `pkg/Type`, `layout_of("pkg/Type")` hands back the YAML twin's layout —
/// store FIELDS rendered beside the YAML hash and size. ONE builder
/// serves both consumers; nested BINDING is by resolver identity and
/// therefore order-independent, so every dependency still resolves exactly
/// as on the other surfaces.
pub(crate) fn store_resolution_set(schemas_dir: &Path) -> Vec<MessageSchema> {
    let mut schemas = ResolutionTiers::assemble(Some(schemas_dir)).combined_store_last();
    preflight_resolution_set(&mut schemas, "store definition resolution set");
    schemas
}

/// The schema SET nested-reference resolution + recursive inline expansion
/// resolve against: the built-in ROS 2 registry ALWAYS, plus (when a
/// workspace is present) every `.msg` store schema and every TOP-LEVEL
/// workspace YAML schema (`workspace_yaml_files` — a nested
/// `schemas/<pkg>/<Type>.yaml` is reachable by SPELLING through the stem
/// tier, never by the tree's qualified-name fold, so a nested reference to
/// a name such a file and the store both spell renders the store's). This
/// set DIVERGES from the enrichment set (`enrich_entries_with_resolved_layout`:
/// built-ins + top-level YAML, no store) by exactly the store: a store-only
/// qualified name expands in the tree and reads unresolved in the layout
/// columns, and nothing in the enrichment set can overwrite a YAML
/// definition; every WORKSPACE-YAML name both sets carry resolves to the
/// same target, while a qualified built-in the store also copies resolves
/// to the built-in in the enrichment set and to the store's copy here
/// (`schema_store_test` pins that shadow). The SERVED catalog
/// (`schema_serve::build_schema_docs`, what a gateway answers and a bag
/// records) keeps the STORE first for a qualified-name collision,
/// deliberately: it also carries the `schema_hash -> name` binding a
/// runtime-registered `ros2 attach` route and `bag record` are named by, and
/// the bridge encodes with the store's definitions — so for a nested twin
/// this tree shows the workspace's definition while the desk decodes with
/// the store's, a stated divergence; a twin can never be an OUTPUT's
/// spelling (the run refuses it).
///
/// ORDER IS PRECEDENCE. The context built over this set is LAST-wins per
/// qualified name ([`build_resolution_context`]), so the tiers are appended
/// least-priority first: built-ins, then the store, then workspace YAML
/// ([`ResolutionTiers::combined`], the ONE assembly every consumer shares).
/// Store-LAST (the earlier order) let a `.msg` definition of a
/// qualified name a workspace YAML also declares overwrite the YAML
/// definition `schema info` had selected, and every field beneath that
/// entry rendered from the store's definition. A spelling NAMING such a
/// twin is refused outright (`workspace_lookup`) — the
/// precedence here decides only a NESTED reference nobody spelled, and it
/// decides it the way the verb selects: the workspace's own definition.
/// The store-LAST sibling ([`store_resolution_set`]) exists for exactly one
/// consumer: a STORE definition's own layout, where the store must win its
/// own qualified name.
///
/// A malformed/unreadable workspace file is warned + skipped (never fatal),
/// exactly as the enrichment and listing paths handle it.
pub(crate) fn resolution_schema_set(schemas_dir: Option<&Path>) -> Vec<MessageSchema> {
    let mut schemas = ResolutionTiers::assemble(schemas_dir).combined();
    // One preflight at the ONE shared builder covers every consumer — the
    // unified renderer, `store_schema_entry`, `schema list`, and `topic
    // echo`'s local walker — the same way the recording fold and the
    // replay registry preflight the same tiers.
    preflight_resolution_set(&mut schemas, "schema info/list resolution set");
    schemas
}

/// `schema info` that works WITHOUT a workspace.
///
/// The required user story is a brand-new laptop resolving a robot's types with zero
/// setup, so `schema info` must not hard-fail when there is no workspace
/// anywhere up the tree (live-proven gap, Go2: it errored "Workspace
/// not found... Create one with: cerulion workspace create"). `Some(dir)`
/// behaves EXACTLY like [`schema_info_unified`] (the full workspace-YAML →
/// `.msg`-store → built-in ladder, then the caller's remote tier). `None` means
/// no workspace exists: the workspace + store tiers are SKIPPED entirely (never
/// read a stray non-workspace `schemas/` from the cwd) and the name resolves
/// against the built-in registry — a `SchemaNotFound` from there is exactly the
/// boundary where the CLI's remote-fetch tier takes over, so the automagic bar
/// (builtin → remote) holds with zero local setup.
/// What a `schema:` spelling claims inside the WORKSPACE, entry-aware —
/// the identity `graph validate` compares two spellings by. The two
/// workspace tiers of `workspace_lookup`, in ITS order:
///
/// 1. The ENTRY tier: the `schemas:` key a top-level `schemas/*.yaml`
///    declares — a schema's own name — parsed and nothing more, and
///    UNIQUENESS-AWARE: exactly one file declaring the entry binds it;
///    several (by file identity, so a symlink to one file is not "two")
///    are REFUSED by the lookup (a loud error naming every source), as is
///    an entry beside a FILE of the same stem that is not that very
///    declaration, and an entry the `.msg` store also spells (the twin).
/// 2. The FILE-STEM tier, for both spellings: `schemas/<spelling>.yaml`,
///    joined verbatim (`my_pkg/Foo` names `schemas/my_pkg/Foo.yaml`),
///    parsed, and matched by the file's NAME exactly — never by
///    `Path::exists`, which on a case-insensitive filesystem accepts
///    `Goal.yaml` for an on-disk `goal.yaml` and made a bare NAME read as a
///    stem claim on one host and as an entry lookup on another. A stem
///    file declaring exactly one entry is that entry; declaring several it
///    is [`WorkspaceSchemaClaim::AmbiguousStem`]; declaring none it claims
///    nothing.
///
/// A FILE is not a schema; an ENTRY is. An entry claim maps to itself; a
/// file-stem claim maps to the file's SOLE entry (returning every entry
/// let `foo` "agree" with `Bar` when `foo.yaml` declared `Foo` and `Bar`,
/// and validation accepted a mismatched wire type). The serving path
/// applies the same rule: a stem-claimed port binds the file's sole entry,
/// multi-entry is a loud residual.
///
/// `Ok(None)` = the spelling claims nothing at the top level; the caller
/// falls back to the `.msg` store, then the resolver — except for ONE
/// pairing the caller handles itself, through
/// [`nested_sole_entry_claim`]: an entry claim that came from a NESTED
/// stem file (`schemas/my_pkg/Foo.yaml`) against the bare name that file's
/// codegen module publishes (`Foo`). That is the perception-example shape
/// this identity exists for, and it is deliberately NOT a general tier —
/// a bare name is never bound to a nested file on its own, so `Image` next
/// to a `schemas/robot/Image.yaml` still means what `port_schema_exists`
/// and `cerulion schema info` say it means (the built-in).
///
/// Where this identity DIVERGES from the existence probe, on purpose: a
/// stem file declaring NO entries exists for `port_schema_exists` but
/// claims nothing here (no schema in it to be identical to); a bare name
/// two top-level files declare EXISTS for the probe (it takes the first)
/// but is ambiguous here (an identity needs THE file); and the nested
/// pairing above binds a bare name the probe does not resolve, so a node
/// declaring such a name is told the QUALIFIED spelling to use rather than
/// that its schema is absent (`declared_resolvability` in `graph_cmd`).
#[derive(Debug, Clone)]
pub enum WorkspaceSchemaClaim {
    /// ONE entry of ONE file — the identity two spellings are compared by.
    Entry {
        file: std::path::PathBuf,
        entry: String,
    },
    /// A file-stem claim on a file declaring MORE than one entry: the
    /// spelling names a file, not a schema, and agrees with nothing.
    AmbiguousStem {
        file: std::path::PathBuf,
        entries: Vec<String>,
    },
    /// A name the stem tier does not claim that MORE than one workspace
    /// file declares — two top-level files each declaring an entry `Foo`,
    /// or (through [`nested_sole_entry_claim`]) two nested files
    /// (`schemas/a/Foo.yaml`, `schemas/b/Foo.yaml`) each declaring it as
    /// their sole entry: the name could be either, so it names no single
    /// schema and agrees with nothing. An entry tier that is not
    /// uniqueness-aware lets such a name agree with the first declaring
    /// file, or with BOTH qualified stems — which is why every tier collects
    /// its declarers before deciding.
    AmbiguousBare {
        name: String,
        files: Vec<std::path::PathBuf>,
    },
}

/// The `.msg` STORE's qualified name for a spelling — the store tier of the
/// existence contract as an identity: a qualified `pkg/Type` present in the
/// store is itself; a bare name matching exactly one store schema is that
/// schema's `pkg/Type`; an ambiguous bare name is the same loud `Err` the
/// existence probe raises; `Ok(None)` when the store has nothing by that
/// name. The SAME rule the port surfaces bind with, so a
/// store alias the existence contract accepts on the YAML agrees with the
/// qualified name the node declares. It differs from
/// `bare_store_port_binding` in ONE way, deliberately: this identity does
/// not consult the workspace-YAML claims, because its caller has already
/// established that the workspace tier claims nothing for the spelling.
pub fn store_schema_qualified(schemas_dir: &Path, raw: &str) -> CliResult<Option<String>> {
    let normalized = normalize_schema(raw);
    let store = SchemaStore::load(schemas_dir);
    if normalized.contains('/') {
        return Ok(store.get(&normalized).map(|s| s.schema.qualified_name()));
    }
    match store.matches_bare_name(&normalized).as_slice() {
        [] => Ok(None),
        [one] => Ok(Some(one.schema.qualified_name())),
        many => Err(ambiguous_store_name(raw, many)),
    }
}

impl WorkspaceSchemaClaim {
    /// Do two claims name the SAME schema? Two entry claims of the same
    /// file (by identity, not by path string) and the same entry. An
    /// ambiguous claim of either kind names no schema and agrees with
    /// nothing — not even itself. Deliberately NOT a `PartialEq`: a derived equality would
    /// compare the paths as strings, which is the case-insensitive-
    /// filesystem bug `same_workspace_file` exists to close.
    pub fn names_same_schema_as(&self, other: &Self) -> bool {
        match (self, other) {
            (
                Self::Entry {
                    file: fa,
                    entry: ea,
                },
                Self::Entry {
                    file: fb,
                    entry: eb,
                },
            ) => ea == eb && same_workspace_file(fa, fb),
            _ => false,
        }
    }
}

/// Are two workspace-claim paths the SAME file? By identity, not by string:
/// on a case-insensitive filesystem the stem tier accepts `schemas/Goal.yaml`
/// for an on-disk `goal.yaml`, so two spellings of one schema arrive as two
/// path spellings of one file, and a string compare made the identity
/// depend on the host's filesystem (green on Linux CI, red on a macOS
/// desk). Unix compares `(dev, ino)`; elsewhere the canonicalized paths.
/// One path spelling IS one file, readable or not; otherwise an unreadable
/// side is never the same file as anything, and each unreadable side is
/// said.
fn same_workspace_file(a: &Path, b: &Path) -> bool {
    if a == b {
        return true;
    }
    let unreadable = |which: &Path, error: &std::io::Error| {
        tracing::warn!(
            file = %which.display(),
            error = %error,
            "could not read a workspace schema file's identity — treating it as a different \
             file from every other"
        );
        false
    };
    #[cfg(unix)]
    {
        use std::os::unix::fs::MetadataExt;
        match (std::fs::metadata(a), std::fs::metadata(b)) {
            (Ok(ma), Ok(mb)) => ma.dev() == mb.dev() && ma.ino() == mb.ino(),
            // Both said: `unreadable` is always false, so `&&` would never
            // reach the second call.
            (Err(ea), Err(eb)) => {
                let first = unreadable(a, &ea);
                let second = unreadable(b, &eb);
                first && second
            }
            (Err(e), _) => unreadable(a, &e),
            (_, Err(e)) => unreadable(b, &e),
        }
    }
    #[cfg(not(unix))]
    {
        match (std::fs::canonicalize(a), std::fs::canonicalize(b)) {
            (Ok(x), Ok(y)) => x == y,
            (Err(ea), Err(eb)) => {
                let first = unreadable(a, &ea);
                let second = unreadable(b, &eb);
                first && second
            }
            (Err(e), _) => unreadable(a, &e),
            (_, Err(e)) => unreadable(b, &e),
        }
    }
}

/// See [`WorkspaceSchemaClaim`].
pub fn workspace_schema_claim(
    schemas_dir: &Path,
    raw: &str,
) -> CliResult<Option<WorkspaceSchemaClaim>> {
    let normalized = normalize_schema(raw);
    Ok(match workspace_lookup(schemas_dir, &normalized)? {
        None => None,
        Some(WorkspaceLookup::Entry { file }) => Some(WorkspaceSchemaClaim::Entry {
            file,
            entry: normalized,
        }),
        Some(WorkspaceLookup::Stem { file, mut entries }) => match entries.len() {
            0 => None,
            1 => Some(WorkspaceSchemaClaim::Entry {
                file,
                entry: entries.remove(0),
            }),
            _ => Some(WorkspaceSchemaClaim::AmbiguousStem { file, entries }),
        },
    })
}

/// What the workspace YAML holds for ONE spelling — the two tiers every
/// ladder applies (`schema info`, the existence probe, the resolver's
/// workspace arm, the identity), in ONE order, so they cannot disagree:
///
/// 1. The ENTRY tier: the top-level `schemas/*.yaml` files whose `schemas:`
///    mapping declares the spelling as an entry — a schema's own name.
///    Parsed leniently (a file that does not read or parse is warned and
///    skipped, so an unrelated mid-edit file never fails a lookup); the
///    declarers are deduped by file identity.
/// 2. The FILE-STEM tier: `schemas/<spelling>.yaml`, matched by the exact
///    file name (`exact_yaml_file`), parsed STRICTLY — a present-but-broken
///    file is a loud `Err`, never "absent".
///
/// A spelling that names MORE THAN ONE definition is REFUSED — a loud
/// [`CliError::Validation`] listing every source, from this one lookup, so
/// `schema info`, the existence probe, the resolver's workspace arm and the
/// validator's identity all refuse it the same way. The three shapes: an entry name declared by SEVERAL
/// top-level files; a FILE-STEM/ENTRY collision (an entry `Foo` declared in
/// one file AND a `Foo.yaml` that is not that very declaration — stem-first
/// made `Foo` mean `Foo.yaml`'s `Bar`, entry-first would silently pick the
/// entry; neither is the user's intent, so neither is chosen); and a
/// YAML/`.msg`-STORE TWIN — a YAML definition at EITHER tier (an entry, or
/// the stem file that alone binds the spelling) the store also spells: every
/// store type of a BARE name, the one store key of a QUALIFIED one, so
/// `schemas/pkg/Foo.yaml` beside `schemas/pkg/msg/Foo.msg` is refused for
/// `pkg/Foo` exactly as `a.yaml`'s `Foo` beside `nav/msg/Foo.msg` is for
/// `Foo` (the same twin the store-parity rule refuses; a
/// store consulted for bare spellings only would accept the qualified
/// twin as one Entry). The message is
/// [`ambiguous_workspace_spelling`]'s, one format for all three.
pub(crate) enum WorkspaceLookup {
    /// Exactly one top-level file declares the spelling as an entry, and
    /// nothing else spells it.
    Entry { file: std::path::PathBuf },
    /// No entry of that name; `schemas/<spelling>.yaml` exists and parses,
    /// declaring these entries — and, when it declares any, the `.msg`
    /// store does not spell the spelling (that pair is refused as the twin).
    /// A stem file declaring NO entries defines nothing, so it is returned
    /// as `Stem { entries: [] }` whatever the store holds.
    Stem {
        file: std::path::PathBuf,
        entries: Vec<String>,
    },
}

/// The ONE refusal for a spelling that names more than one definition, in
/// the format the store-parity lane's refusals use — `'<spelling>' is
/// ambiguous … — defined by: <source>, <source>.` — so a user reads one
/// shape whichever surface refuses. Each source is a workspace-relative
/// path with what it contributes: `schemas/a.yaml (entry Foo)`,
/// `schemas/Foo.yaml (file stem; entries: Bar)`,
/// `schemas/nav/msg/Foo.msg (store)`.
pub fn ambiguous_workspace_spelling(spelling: &str, sources: &[String]) -> CliError {
    CliError::Validation(format!(
        "'{spelling}' is ambiguous in this workspace — defined by: {}. Declare it once, or \
         name the one you mean: a nested workspace file by '<pkg>/<Type>', a .msg store \
         definition by '<pkg>/<Type>'.",
        sources.join(", ")
    ))
}

/// A source line for [`ambiguous_workspace_spelling`]: the path relative to
/// the workspace (`schemas/…`) and what it contributes.
pub(crate) fn source_label(schemas_dir: &Path, file: &Path, contributes: &str) -> String {
    format!(
        "{} ({contributes})",
        workspace_relative_label(schemas_dir, file)
    )
}

/// `schemas/pkg/Nested.yaml` — the workspace-relative path of `file`, with NO
/// parenthetical. The half of [`source_label`] that names the FILE.
///
/// Split out because [`preflight_rejection_error`]'s template already wraps
/// its `{location}` in parentheses, so handing it a `source_label` renders
/// `(schemas/X.yaml (bound by …))`. Its callers need the path to be correct
/// for a NESTED file (`workspace_file_name` is basename-only and rendered
/// `schemas/pkg/Nested.yaml` as `schemas/Nested.yaml`, a path that does not
/// exist, on a shape the advisory-map test pins as bindable) without inheriting a
/// label the surrounding sentence already supplies.
pub(crate) fn workspace_relative_label(schemas_dir: &Path, file: &Path) -> String {
    let relative = file
        .strip_prefix(schemas_dir)
        .map(|rel| {
            rel.components()
                .map(|component| component.as_os_str().to_string_lossy().into_owned())
                .collect::<Vec<_>>()
                .join("/")
        })
        .unwrap_or_else(|_| file.display().to_string());
    format!("schemas/{relative}")
}

/// See [`WorkspaceLookup`]. `Ok(None)` means no WORKSPACE YAML tier claims
/// the spelling — the `.msg` store may still define it, and that is the
/// CALLER's tier (`port_schema_exists` and `schema_info_unified` ask the
/// store after this returns `None`; the resolver's bare arm goes to the
/// built-in registry instead, its documented gap); the store takes part
/// HERE only as the twin of a YAML definition this lookup found, and only
/// as far as the loaders read it: a `.msg` file that does not parse is
/// skipped with a warn by `SchemaStore::load`, and an ENTRY file that does
/// not parse is warned and skipped by the entry scan (`entry_names_lenient`)
/// — so an unparseable `a.yaml` beside a healthy `b.yaml` declaring the same
/// entry is ONE source, `b.yaml`'s, until `a.yaml` is fixed (the warn says
/// so; only a stem file is a hard error, because it is the one file the
/// spelling names outright).
pub(crate) fn workspace_lookup(
    schemas_dir: &Path,
    spelling: &str,
) -> CliResult<Option<WorkspaceLookup>> {
    let normalized = normalize_schema(spelling);
    // Tier 1 — the entry tier, a cheap scan: parsed and nothing more (a
    // match is not routed through `schema_info`, which resolves every
    // layout against the built-in corpus — a cost a yes/no question asked
    // once per port must not pay). Two PATHS to one file (a symlink beside
    // its target) are one declarer.
    let mut declaring: Vec<std::path::PathBuf> = Vec::new();
    for path in workspace_yaml_files(schemas_dir) {
        let Some(names) = entry_names_lenient(&path) else {
            continue;
        };
        // ONE identity per entry: `pkg::Type` and `pkg/Type`
        // are the same schema declared two ways, so the spelling a file uses
        // cannot decide whether it declares the name. The entry keeps its
        // DECLARED spelling everywhere it is SHOWN (`schema list`'s row, the
        // `file stem; entries:` label) — only the MATCH is canonical.
        if !names.iter().any(|n| normalize_schema(n) == normalized) {
            continue;
        }
        match declaring.iter().position(|d| same_workspace_file(d, &path)) {
            // Two paths to one file are one declarer; the refusal names the
            // FILE, so a symlink recorded first yields to its target.
            Some(seen) => {
                let is_symlink = |p: &Path| {
                    std::fs::symlink_metadata(p)
                        .map(|m| m.file_type().is_symlink())
                        .unwrap_or(false)
                };
                if is_symlink(&declaring[seen]) && !is_symlink(&path) {
                    declaring[seen] = path;
                }
            }
            None => declaring.push(path),
        }
    }
    declaring.sort();
    let (dir, stem) = match normalized.rsplit_once('/') {
        Some((pkg, stem)) => (schemas_dir.join(pkg), stem),
        None => (schemas_dir.to_path_buf(), normalized.as_str()),
    };
    let stem_file = exact_yaml_file(&dir, stem);
    // The `.msg` store's definition(s) of THIS spelling, as the refusal names
    // them — consulted only once a YAML tier has answered, so a store schema
    // alone still claims nothing in the YAML tiers. A qualified spelling is
    // exactly one store key (`pkg/Type`); a bare one is every store type of
    // that name. Either beside a YAML definition of the same spelling is the
    // TWIN, at the ENTRY tier or the STEM tier alike.
    let store_twins = || -> Vec<String> {
        let store = SchemaStore::load(schemas_dir);
        let label = |stored: &StoredSchema| {
            source_label(
                schemas_dir,
                &schemas_dir.join(&stored.relative_path),
                "store",
            )
        };
        if normalized.contains('/') {
            store.get(&normalized).map(label).into_iter().collect()
        } else {
            store
                .matches_bare_name(&normalized)
                .into_iter()
                .map(label)
                .collect()
        }
    };
    // A NESTED sole-entry file of the bare spelling is NOT a twin of a
    // top-level definition: the cfg-decided fixtures model the top-level file
    // and the nested file as the two BUILDS' schemas, addressed by the bare
    // and the qualified spelling — a bare spelling a top-level tier claims
    // names that definition, and a nested file is named by `pkg/Name`; it
    // binds the bare name only when nothing top-level claims it (the
    // identity's `nested_sole_entry_claim`).
    if !declaring.is_empty() {
        // Every definition the spelling names, as the refusal lists them:
        // the declaring entry files; a stem file that is not that very
        // declaration; the `.msg` store's definition(s) of the spelling.
        let mut sources: Vec<String> = declaring
            .iter()
            .map(|file| source_label(schemas_dir, file, &format!("entry {normalized}")))
            .collect();
        if let Some(stem_file) = &stem_file {
            // A stem file that IS one of the declarers contributes once, as
            // that declaration. Keying this on `declaring.len() == 1` named
            // the same file twice whenever the entry was ALSO duplicated
            // elsewhere (`schemas/Goal.yaml (entry Goal), schemas/other.yaml
            // (entry Goal), schemas/Goal.yaml (file stem; entries: Goal)`) —
            // three sources for two files, in the message a user reads to
            // find them.
            let is_the_declarer = declaring
                .iter()
                .any(|declarer| same_workspace_file(declarer, stem_file));
            if !is_the_declarer {
                // A stem file declaring NO entries defines nothing and is no
                // source (the stem tier says the same); one that cannot be
                // read may define anything, so it is named as unreadable.
                let contributes = match entry_names_lenient(stem_file) {
                    Some(names) if names.is_empty() => None,
                    Some(names) => Some(format!("file stem; entries: {}", names.join(", "))),
                    None => Some("file stem; unreadable".to_string()),
                };
                if let Some(contributes) = contributes {
                    sources.push(source_label(schemas_dir, stem_file, &contributes));
                }
            }
        }
        sources.extend(store_twins());
        if sources.len() > 1 {
            return Err(ambiguous_workspace_spelling(&normalized, &sources));
        }
        return Ok(Some(WorkspaceLookup::Entry {
            file: declaring.remove(0),
        }));
    }
    // Tier 2 — the file-stem tier, strict.
    let Some(file) = stem_file else {
        return Ok(None);
    };
    // A present-but-broken stem file is a loud error that NAMES the file: a
    // stray unreadable `schemas/Image.yaml` blocking a run must say which
    // file to fix. It rides `CliError::SchemaUnchecked`, not `Validation`:
    // `Validation` is the REFUSAL channel, rendered verbatim everywhere,
    // while an unchecked spelling is framed by its consumer — the identity's
    // verdict and the report's resolvability say "'<spelling>' could not be
    // checked against this workspace (<this message>); resolve that first",
    // the run's gate says "which this workspace could not check: <this
    // message>" — each exactly once, with no variant prefix leaking in.
    let unchecked = |what: &str, e: &dyn std::fmt::Display| {
        CliError::SchemaUnchecked(format!(
            "{} could not be {what} ({e})",
            source_label(schemas_dir, &file, "file stem")
        ))
    };
    let content = std::fs::read_to_string(&file).map_err(|e| unchecked("read", &e))?;
    let entries: Vec<String> = parse_message_schemas(&content)
        .map_err(|e| unchecked("parsed", &e))?
        .into_iter()
        .map(|s| s.name)
        .collect();
    // A stem file the store also spells is the twin at this tier: a nested
    // `schemas/pkg/Foo.yaml` beside `schemas/pkg/msg/Foo.msg` for `pkg/Foo`,
    // a top-level `Foo.yaml` beside any `<pkg>/msg/Foo.msg` for `Foo`. A
    // stem file declaring NO entries defines nothing, so it is no twin
    // either: the claim built on this result treats it as no claim (and the
    // entry tier does not count it as a source), while the existence probe
    // and `schema info` answer with the empty stem file exactly as they did
    // before any store existed — unchanged here.
    if !entries.is_empty() {
        let twins = store_twins();
        if !twins.is_empty() {
            let mut sources = vec![source_label(
                schemas_dir,
                &file,
                &format!("file stem; entries: {}", entries.join(", ")),
            )];
            sources.extend(twins);
            return Err(ambiguous_workspace_spelling(&normalized, &sources));
        }
    }
    Ok(Some(WorkspaceLookup::Stem { file, entries }))
}

/// The ONE refusal for a bare spelling that more than one NESTED sole-entry
/// file and/or the `.msg` store define ([`WorkspaceSchemaClaim::AmbiguousBare`]
/// from [`nested_sole_entry_claim`]), in the `defined by:` format every other
/// refusal uses — each `.msg` file `(store)`, each YAML file `(entry <name>)`.
/// Rendered here, once, because every surface must refuse it:
/// the existence probe, the resolver, `schema info`, and the
/// validator's identity (a probe that fell through to the store's
/// "present" first would let a nested YAML + store twin run).
pub fn ambiguous_bare_refusal(
    schemas_dir: &Path,
    name: &str,
    files: &[std::path::PathBuf],
) -> CliError {
    let sources = files
        .iter()
        .map(|file| {
            let contributes = if file.extension().and_then(|e| e.to_str()) == Some("msg") {
                "store".to_string()
            } else {
                format!("entry {name}")
            };
            source_label(schemas_dir, file, &contributes)
        })
        .collect::<Vec<_>>();
    ambiguous_workspace_spelling(name, &sources)
}

/// `Err` for a bare spelling whose nested sole-entry claim is ambiguous, or
/// could not be checked (an unreadable nested stem file beside another
/// declarer — propagated as [`CliError::SchemaUnchecked`]); `Ok(())`
/// otherwise — the check every bare-name fallback runs BEFORE the store or
/// the built-in registry may answer.
fn refuse_ambiguous_bare(schemas_dir: &Path, normalized: &str) -> CliResult<()> {
    match nested_sole_entry_claim(schemas_dir, normalized)? {
        Some(WorkspaceSchemaClaim::AmbiguousBare { name, files }) => {
            Err(ambiguous_bare_refusal(schemas_dir, &name, &files))
        }
        _ => Ok(()),
    }
}

/// The ONE way a bare name reaches a NESTED stem file: the file whose SOLE
/// entry it is. `schemas/<pkg>/<Type>.yaml` is addressable by the qualified
/// stem `<pkg>/<Type>` and, from the node's own codegen module, by the bare
/// `<Type>` it declares — so `graph validate` asks this when a nested entry
/// claim meets exactly that bare name (see `schema_spellings_agree`), and
/// when it needs the qualified spelling to recommend for a bare declared
/// name the existence probe does not resolve. Exactly one such file (by
/// identity) binds the name; several are [`WorkspaceSchemaClaim::AmbiguousBare`]
/// — and so is one nested file beside a `.msg` STORE schema of the same
/// bare name (`schemas/nav/msg/Foo.msg`), which is the other workspace
/// declaration a node's codegen module could be compiled from: the parser
/// reports the bare name for both shapes, so the two are indistinguishable
/// from the node's side and the name must be refused, never guessed. A
/// qualified spelling, or a bare name no nested file declares alone, is
/// `None` (a store alias on its own is the store tier's, as the existence
/// probe has it). Deliberately not a tier of [`workspace_schema_claim`]: a
/// bare name on its own means what the existence probe says it means.
/// `Err` when a nested file whose stem is the bare name cannot
/// be read beside another declarer — [`CliError::SchemaUnchecked`], naming
/// the file; every caller (the existence probe through `refuse_ambiguous_bare`,
/// the report's resolvability, the identity) renders it as "could not be
/// checked", never as absence.
pub fn nested_sole_entry_claim(
    schemas_dir: &Path,
    bare: &str,
) -> CliResult<Option<WorkspaceSchemaClaim>> {
    let normalized = normalize_schema(bare);
    if normalized.contains('/') {
        return Ok(None);
    }
    let mut sole: Vec<std::path::PathBuf> = Vec::new();
    // A nested file the scan cannot read, whose STEM is this bare name, may
    // well be its sole declarer. Alone it is skipped (the scan's
    // warn names it; a readable nested file binds nothing at the top-level
    // tiers either), but beside ANOTHER declarer — the store, or a second
    // nested file — skipping it silently would let the twin answer "present":
    // it is instead the loud unchecked class, naming the file and
    // the cause. A nested file whose sole ENTRY is the name but whose stem
    // is not cannot be recognised while unreadable — a stated residual.
    let mut unreadable: Option<std::path::PathBuf> = None;
    for path in nested_stem_yaml_files(schemas_dir) {
        let Some(names) = entry_names_lenient(&path) else {
            if path.file_stem().and_then(|s| s.to_str()) == Some(normalized.as_str()) {
                unreadable.get_or_insert(path);
            }
            continue;
        };
        if names.len() == 1
            && normalize_schema(&names[0]) == normalized
            && !sole.iter().any(|s| same_workspace_file(s, &path))
        {
            sole.push(path);
        }
    }
    let store = SchemaStore::load(schemas_dir);
    let store_twins: Vec<std::path::PathBuf> = store
        .matches_bare_name(&normalized)
        .iter()
        .map(|s| schemas_dir.join(&s.relative_path))
        .collect();
    if let Some(path) = unreadable {
        if !sole.is_empty() || !store_twins.is_empty() {
            let cause = match std::fs::read_to_string(&path) {
                Err(e) => format!("could not be read ({e})"),
                Ok(content) => match parse_message_schemas(&content) {
                    Err(e) => format!("could not be parsed ({e})"),
                    Ok(_) => "could not be read on the first pass".to_string(),
                },
            };
            return Err(CliError::SchemaUnchecked(format!(
                "{} {cause}",
                source_label(schemas_dir, &path, "file stem")
            )));
        }
    }
    if sole.is_empty() {
        return Ok(None);
    }
    sole.sort();
    let mut declarers = sole;
    declarers.extend(store_twins);
    Ok(Some(match declarers.len() {
        1 => WorkspaceSchemaClaim::Entry {
            file: declarers.remove(0),
            entry: normalized,
        },
        _ => WorkspaceSchemaClaim::AmbiguousBare {
            name: normalized,
            files: declarers,
        },
    }))
}

/// `<dir>/<stem>.yaml` when a directory entry of EXACTLY that name exists —
/// the stem tier's lookup, immune to a case-insensitive filesystem's
/// `Path::exists`. An absent directory is quiet; an unreadable one, or an
/// unreadable entry in it, warns and counts as no match.
fn exact_yaml_file(dir: &Path, stem: &str) -> Option<std::path::PathBuf> {
    let wanted = format!("{stem}.yaml");
    let entries = match std::fs::read_dir(dir) {
        Ok(entries) => entries,
        Err(e) => {
            if e.kind() != std::io::ErrorKind::NotFound {
                tracing::warn!(
                    dir = %dir.display(),
                    error = %e,
                    "schema directory unreadable — no file-stem claim can be read from it"
                );
            }
            return None;
        }
    };
    for dirent in entries {
        match dirent {
            Ok(entry) if entry.file_name().to_str() == Some(wanted.as_str()) => {
                return Some(entry.path());
            }
            Ok(_) => {}
            Err(e) => tracing::warn!(
                dir = %dir.display(),
                error = %e,
                "unreadable directory entry — skipped in the file-stem lookup"
            ),
        }
    }
    None
}

/// The entry names a workspace YAML declares, or `None` — warned, skipped —
/// when it does not read or parse (an unrelated mid-edit file must not fail
/// a lookup; the file-stem tier already reports it loudly for its own name).
fn entry_names_lenient(path: &Path) -> Option<Vec<String>> {
    let content = match std::fs::read_to_string(path) {
        Ok(content) => content,
        Err(e) => {
            tracing::warn!(
                file = %path.display(),
                error = %e,
                "could not read schema file for the workspace claim scan"
            );
            return None;
        }
    };
    match parse_message_schemas(&content) {
        Ok(parsed) => Some(parsed.into_iter().map(|s| s.name).collect()),
        Err(e) => {
            tracing::warn!(
                file = %path.display(),
                error = %e,
                "could not parse schema file for the workspace claim scan"
            );
            None
        }
    }
}

/// The nested stem-tier files — `schemas/<pkg>/<Type>.yaml`, exactly one
/// level down, which is the only depth a qualified `schema:` can address
/// (`validate_schema_value` refuses a second separator) — sorted. One
/// level, listed rather than walked: a directory symlink is not followed
/// (a `schemas/<pkg>/loop -> ..` must not recurse), a `schemas/<pkg>/msg/`
/// store directory is not a schema, and a deeper file no spelling can name
/// must not bind a bare one. An absent `schemas/` is quiet; an unreadable
/// directory or entry warns and is skipped.
fn nested_stem_yaml_files(schemas_dir: &Path) -> Vec<std::path::PathBuf> {
    let mut out = Vec::new();
    let top = match std::fs::read_dir(schemas_dir) {
        Ok(top) => top,
        Err(e) => {
            if e.kind() != std::io::ErrorKind::NotFound {
                tracing::warn!(
                    dir = %schemas_dir.display(),
                    error = %e,
                    "schemas/ directory present but unreadable — skipped in the nested claim scan"
                );
            }
            return out;
        }
    };
    for dirent in top {
        let dirent = match dirent {
            Ok(dirent) => dirent,
            Err(e) => {
                tracing::warn!(
                    dir = %schemas_dir.display(),
                    error = %e,
                    "unreadable directory entry — skipped in the nested claim scan"
                );
                continue;
            }
        };
        // `file_type` does not follow a symlink, so a linked directory is
        // not listed — the one place a cycle could enter.
        let is_dir = match dirent.file_type() {
            Ok(kind) => kind.is_dir(),
            Err(e) => {
                tracing::warn!(
                    path = %dirent.path().display(),
                    error = %e,
                    "could not read a directory entry's type — skipped in the nested claim scan"
                );
                false
            }
        };
        if !is_dir {
            continue;
        }
        let pkg_dir = dirent.path();
        let files = match std::fs::read_dir(&pkg_dir) {
            Ok(files) => files,
            Err(e) => {
                tracing::warn!(
                    dir = %pkg_dir.display(),
                    error = %e,
                    "schema directory unreadable — skipped in the nested claim scan"
                );
                continue;
            }
        };
        for file in files {
            let path = match file {
                Ok(file) => file.path(),
                Err(e) => {
                    tracing::warn!(
                        dir = %pkg_dir.display(),
                        error = %e,
                        "unreadable directory entry — skipped in the nested claim scan"
                    );
                    continue;
                }
            };
            if path.extension().and_then(|e| e.to_str()) == Some("yaml") && path.is_file() {
                out.push(path);
            }
        }
    }
    out.sort();
    out
}

/// Does this `schema:` value name a schema that EXISTS?
///
/// [`resolve_port_schema`] cannot answer it. Its tier 2 validates a
/// qualified value against the `.msg` store (msg-store parity) but passes
/// every OTHER value containing a `/` straight through as `Qualified` —
/// that is deliberate (it is the resolver's IDEMPOTENCE anchor, so feeding
/// a prior output back in returns it unchanged), but it means `made/up`
/// resolves happily, and a graph-validation check built on it accepts every
/// non-store typo that happens to carry a slash while catching only bare
/// names. That is the whole of the class this exists to close.
///
/// CHEAP, because `graph validate` asks it once per output and a graph in
/// this tree carries hundreds: the qualified arm is a `SchemaStore` load plus
/// a linear scan of the 254-entry `BUILTIN_MSGS` slice, never
/// [`schema_info_unified`], which PARSES the schema and seeds a resolution
/// set for rendering a tree nobody asked for here. The bare arm runs the ONE
/// workspace lookup (`workspace_lookup` — the claim) plus the size
/// preflight of the entry it binds (`workspace_binding_preflight`; no
/// layout is materialized) and the store tier's refusal — the
/// same reads the resolver already pays.
///
/// Resolution mirrors `schema_info_unified`'s precedence — the workspace
/// tier (the ONE claim verdict: a stem binds its like-named or sole entry,
/// an entry binds itself, an ambiguous spelling is refused), then the `.msg`
/// store, then the built-in registry — so a value this accepts is one
/// `cerulion schema info` can show BY THAT SPELLING: an entry name is judged
/// alone on both surfaces; a file STEM is judged here on the one
/// entry it binds, while `schema info <stem>` is the FILE VIEW and refuses a
/// file holding any over-ceiling entry — the one deliberate asymmetry, since
/// a port binds one document and the file view renders them all.
///
/// **The file-stem tier applies to a QUALIFIED name too, and leaving it out
/// is a real defect.** `schema_info_unified`'s FIRST tier is
/// `schemas/<name>.yaml`, and `<name>` reaches `Path::join` verbatim — so a
/// workspace holding `schemas/my_pkg/Foo.yaml` resolves `my_pkg/Foo` there.
/// Skipping straight to the entry-name tier for
/// anything containing a `/` leaves such a schema accepted by `schema info`
/// and REFUSED by `graph validate` unless the YAML's own `name:` happened to
/// match its path — the two surfaces disagreeing about the same workspace,
/// which is exactly what the "mirrors it exactly" claim above exists to rule
/// out. A `..` segment reaches the probe as it does in `schema_info_unified`:
/// the stem tier reads and parses that ONE file (`schemas/<seg>/<stem>.yaml`)
/// and touches nothing else, and `validate_schema_value` has already refused
/// anything with more than one separator or an empty segment.
pub fn port_schema_exists(schemas_dir: &Path, raw: &str) -> CliResult<bool> {
    let normalized = normalize_schema(raw);
    // The workspace tiers — entry, then file stem — apply to BOTH spellings
    // through the ONE lookup every ladder shares (`workspace_lookup`).
    // Doing it here (rather than only inside the qualified arm) is also what
    // keeps a present-but-BROKEN stem file reportable: the bare arm's
    // fallback collapses the resolver's error with `.is_ok()`, which would
    // turn "this file is malformed" into "no such schema".
    if let Some(lookup) = workspace_lookup(schemas_dir, &normalized)? {
        // The bound definition is SIZED before the name is accepted
        // here: a gate that answered "present" for an entry the wire
        // ceiling refuses would accept a name `resolve_port_schema` — which
        // sizes it — rejects.
        workspace_binding_preflight(schemas_dir, &lookup, &normalized)?;
        return Ok(true);
    }

    if normalized.contains('/') {
        if SchemaStore::load(schemas_dir).get(&normalized).is_some() {
            return Ok(true);
        }
        return Ok(builtin_has_qualified(&normalized));
    }
    // A bare spelling one NESTED sole-entry file binds is checked for its
    // twins BEFORE the store may answer "present": nested YAML + store, or two
    // nested files, is the decided refusal (the store's answer used
    // to come first, so graph validation never saw the twin).
    refuse_ambiguous_bare(schemas_dir, &normalized)?;
    // The `.msg` STORE. Historically `resolve_port_schema` did not consult
    // it at all, so a uniquely-named store schema
    // (`schemas/<pkg>/msg/<Type>.msg`, acquired by `ros2 attach`'s acquisition
    // ladder) was shown by `schema info` and REFUSED by `graph validate`:
    // the third instance of this function disagreeing with the ladder it
    // claims to mirror, after the qualified and bare file-stem tiers.
    // `bare_store_port_binding` is the SAME verdict `resolve_port_schema`
    // binds, so an AMBIGUOUS bare name — or one whose sole bearer's
    // qualified string a top-level workspace YAML entry claims —
    // errors identically here rather than silently reading as absent.
    if bare_store_port_binding(schemas_dir, &SchemaStore::load(schemas_dir), &normalized)?.is_some()
    {
        return Ok(true);
    }
    // A BARE name that is in none of the workspace tiers: the resolver's
    // remaining tier resolves against the built-in registry by unique short
    // name, erroring on both ambiguous and unknown. Collapsing that to a bool
    // is right here — every remaining way to fail means "no schema by that
    // name", which is exactly what `false` says.
    Ok(resolve_port_schema(schemas_dir, &normalized).is_ok())
}

pub fn schema_info_unified_opt(
    schemas_dir: Option<&Path>,
    name: &str,
) -> CliResult<UnifiedSchemaInfo> {
    match schemas_dir {
        Some(dir) => schema_info_unified(dir, name),
        None => {
            let entry = builtin_schema_info(name)?;
            Ok(UnifiedSchemaInfo {
                source: SchemaSource::Builtin,
                shadowed_builtin: None,
                result: SchemaInfoResult {
                    entries: vec![entry],
                },
                source_detail: None,
                // No workspace: nested references resolve against the
                // built-in registry alone (built-in nested types still
                // expand; a custom robot type would be network-resolved).
                schema_set: resolution_schema_set(None),
            })
        }
    }
}

/// Qualified built-in names shadowed by `result`'s workspace entries.
///
/// A workspace schema whose name equals a built-in's BARE message name
/// (`Image` vs `sensor_msgs/Image`) or its full qualified name shadows
/// that built-in in `schema info` lookups. Multiple matches (several
/// entries, or one bare name defined in several packages — e.g. `Pose2D`
/// lives in both geometry_msgs and vision_msgs) join comma-separated.
fn shadowed_builtin_names(result: &SchemaInfoResult) -> Option<String> {
    let mut shadowed: Vec<String> = Vec::new();
    for entry in &result.entries {
        for &(pkg, msg, _) in native_ros2_messages::BUILTIN_MSGS {
            let qualified = format!("{}/{}", pkg, msg);
            let declared = normalize_schema(&entry.name);
            if (declared == msg || declared == qualified) && !shadowed.contains(&qualified) {
                shadowed.push(qualified);
            }
        }
    }
    if shadowed.is_empty() {
        None
    } else {
        Some(shadowed.join(", "))
    }
}

/// Port resolution's BARE-name binding over the `.msg` store — the ONE
/// verdict `resolve_port_schema` binds, `port_schema_exists` probes and
/// `schema info`'s store tier selects by, so the three resolving
/// surfaces can never drift. `bare` is already `::`-normalized and carries
/// no `/`. EXISTENCE + IDENTITY ONLY — deliberately not
/// [`store_schema_lookup`]: that one builds a full [`SchemaEntry`], which
/// RESOLVES the schema's layout, and layout resolution PANICS on fixed-array
/// size overflow (`cerulion_core`'s `layout.rs`: `fixed-array size overflow
/// (8 x 18446744073709551615)`). Calling it from `port_schema_exists` —
/// which only ever asks a yes/no question — meant a single
/// hostile-but-parseable `.msg` anywhere in `schemas/` CRASHED `cerulion
/// graph validate` and `graph run`. `float64[18446744073709551615]
/// a` in a store `.msg`, referenced by a graph output, panics the verb; with
/// this probe it is a clean refusal.
///
/// - `Ok(Some(qualified))`: exactly one store package bears `bare` AND its
///   qualified spelling is the store's to bind — the ONE workspace lookup
///   (`workspace_lookup`, the same call every spelling surface makes)
///   claims nothing for it.
/// - `Ok(None)`: no store package bears it — the built-in tier is next.
/// - `Err`: the three loud refusals, and nothing else. Several bearers — the
///   shared [`ambiguous_store_name`] (`schema info`'s verdict too). Or an
///   AMBIGUOUS qualified spelling, whose refusal is `workspace_lookup`'s own
///   (one message for one condition, whichever spelling reached it). Or
///   a sole bearer whose qualified
///   string a workspace YAML entry also declares: that entry wins the string
///   on every surface that addresses a schema by name (the gateway's served
///   documents, the recording descriptor, the replay registry's expected
///   hash), so binding the bare name to the STORE twin made replay refuse a
///   valid bag as schema drift (exit 2) and the gateway serve the YAML
///   document for frames stamped with the store hash (the desk's hash gate
///   refuses; nothing decodes) — never wrong bytes, the hash gates run
///   first, but a valid bag unreplayable and a valid topic unviewable, and
///   both "working" only while the twins happen to share a layout. The
///   store is string-addressed, not identity-addressed, so the collision cannot be
///   declared or introspected by EITHER spelling — the qualified one is
///   `workspace_lookup`'s twin refusal, the bare one propagates it — and
///   every consumer's bare claim excludes it ([`unique_bare_store_names`],
///   which asks the fold-side claims index the same question: the folds
///   index every name once and cannot pay a lookup per spelling).
///
/// (The panic itself is `schema info`'s problem too, where materializing the
/// layout is the point; that is `cerulion_core` and out of this scope.)
fn bare_store_port_binding(
    schemas_dir: &Path,
    store: &SchemaStore,
    bare: &str,
) -> CliResult<Option<String>> {
    debug_assert!(
        !bare.contains('/'),
        "bare_store_port_binding takes a BARE name; got {bare}"
    );
    match store.matches_bare_name(bare).as_slice() {
        [] => Ok(None),
        [one] => {
            let qualified = one.schema.qualified_name();
            // WHO OWNS THE QUALIFIED STRING is asked of the ONE workspace
            // lookup, never of a second index — and it is the SAME question
            // that lookup already answers as the YAML/`.msg`-store TWIN: a
            // workspace definition of `pkg/Type` beside the store file this
            // bare name resolves to IS that twin, so the lookup refuses it
            // and the bare spelling propagates that refusal verbatim (one
            // condition, one message, whichever spelling reached it). What
            // is left for the bare name to bind is exactly a store type no
            // workspace definition claims.
            workspace_lookup(schemas_dir, &qualified)?;
            Ok(Some(qualified))
        }
        many => Err(ambiguous_store_name(bare, many)),
    }
}

/// The ambiguous-bare-name error, shared by [`store_schema_lookup`] and
/// [`bare_store_port_binding`] so an existence probe and a resolution report
/// the same thing.
fn ambiguous_store_name(name: &str, many: &[&StoredSchema]) -> CliError {
    let candidates = many
        .iter()
        .map(|s| s.schema.qualified_name())
        .collect::<Vec<_>>()
        .join(", ");
    CliError::Validation(format!(
        "'{name}' is ambiguous in the .msg store — defined by: {candidates}. \
         Qualify it as 'pkg/Type'."
    ))
}

/// Resolve `name` against the `.msg` store for `schema info` — a
/// qualified `pkg/Type` (or `pkg::Type`) hits the store key directly; a BARE
/// name takes port resolution's ONE verdict ([`bare_store_port_binding`],
/// the shared rule): bound iff exactly one package bears it AND no workspace YAML
/// entry claims that package's qualified string, refused loudly otherwise
/// (the shared [`ambiguous_store_name`] for several bearers, or
/// `workspace_lookup`'s own twin refusal propagated verbatim for a claimed
/// string — so `schema info`, `resolve_port_schema` and `port_schema_exists`
/// cannot disagree about a bare name). `Ok(None)` = no store match (fall
/// through to the built-in tier). The ONLY caller is `schema_info_unified`'s
/// store tier.
fn store_schema_lookup(
    schemas_dir: &Path,
    store: &SchemaStore,
    name: &str,
    resolution_set: &[MessageSchema],
) -> CliResult<Option<SchemaEntry>> {
    let normalized = normalize_schema(name);
    if normalized.contains('/') {
        return store
            .get(&normalized)
            .map(|s| store_schema_entry(s, resolution_set))
            .transpose();
    }
    match bare_store_port_binding(schemas_dir, store, &normalized)? {
        None => Ok(None),
        Some(qualified) => store
            .get(&qualified)
            .map(|s| store_schema_entry(s, resolution_set))
            .transpose(),
    }
}

/// Build a display [`SchemaEntry`] for one `.msg` store schema.
///
/// Layout columns (fixed-vs-variable split, wire fixed size, hash) resolve
/// against `resolution_set` — the ONE preflighted three-tier set, ordered
/// store-last ([`store_resolution_set`]) so this store definition
/// is the string winner for its own qualified name; nested references bind
/// by identity exactly as on the renderer and graph-hash surfaces (a
/// site using built-ins + store only would let a store
/// field referencing a workspace YAML type report DIFFERENT
/// fixedness/size/hash here than `graph run`'s divergence map and the
/// rendered tree computed for the same schema). The resolved hash is the
/// wire-contract value that matches a compile-time-generated twin's
/// `SCHEMA_HASH` (`frame_walker_production_test` pins the equivalence). A
/// nested reference absent from the set leaves that field VARIABLE in the
/// resolved layout (the renderer's unknown-type marker); a definition the
/// preflight REJECTED is refused outright (see
/// [`preflight_rejection_error`]), never served a hash of its raw
/// declaration.
fn store_schema_entry(
    stored: &StoredSchema,
    resolution_set: &[MessageSchema],
) -> CliResult<SchemaEntry> {
    let schema = &stored.schema;
    let qualified = schema.qualified_name();

    let (mut resolver, _warnings) = LayoutResolver::new(resolution_set.to_vec());
    // `layout_of` is `None` for a
    // store definition ONLY when the shared preflight REMOVED it — every
    // definition left in the set resolves; an absent nested target merely
    // leaves its field variable — or when its RESOLVED layout exceeds the u32
    // wire ceiling (`representable_layout`). Either way the definition is
    // rejected, and there is no meaningful number to print: the checked
    // `schema_hash()` twin avoids the PANIC, but a checked hash
    // of the RAW declaration is still a hash nothing on the wire can match (a
    // composed-overflow `Outer = Inner[2^33]` hashes "successfully" with a
    // 0-byte fixed section), so `schema info` would advertise an identity no
    // producer could ever stamp. The arm is a refusal that names WHY.
    let Some(layout) =
        representable_layout(resolver.layout_of(&qualified), "schema info store entry")
    else {
        return Err(preflight_rejection_error(
            &qualified,
            &stored.relative_path,
            schema,
        ));
    };

    let fields: Vec<SchemaFieldEntry> = schema
        .fields
        .iter()
        .map(|field| SchemaFieldEntry {
            name: field.name.clone(),
            type_display: field.field_type.canonical_str(),
            is_nested: field_type_is_nested(&field.field_type),
            // The resolved layout is the truth for fixed-vs-variable (a
            // recursively-fixed nested reference inlines into the fixed
            // section; an unresolvable one stays variable).
            is_variable: layout.variable_fields.iter().any(|v| v.name == field.name),
            field_type: field.field_type.clone(),
        })
        .collect();

    let (schema_hash, wire_fixed_size, wire_fixed_size_resolved) =
        (Some(layout.schema_hash), layout.fixed_size, true);

    Ok(SchemaEntry {
        name: qualified,
        // `.msg` sources carry no parsed description.
        description: String::new(),
        field_count: fields.len(),
        fields,
        schema_hash,
        wire_fixed_size,
        wire_fixed_size_resolved,
    })
}

/// The loud refusal for a store definition — or, now also, a workspace
/// YAML entry — the resolution preflight REJECTED (or whose resolved layout
/// exceeds the u32 wire ceiling): it has
/// no wire layout and no wire hash, so `schema info` reports WHY — the
/// declared-size overflow with its offending field, the declared u32
/// overshoot, or a composition past either through fixed-nested inlining —
/// instead of a panic or a hash of the raw declaration
/// (an identity no producer could stamp), both earlier behaviours. `schema list` keeps
/// such a row but prints it unhashed ([`StoreSchemaEntry::schema_hash`] /
/// [`WorkspaceSchemaEntry::schema_hash`] is `None`). The workspace-YAML
/// parser caps every `FixedArray` length up front, so the ARITHMETIC arm is
/// reachable from the `.msg` store only; the wire-ceiling arm is reachable
/// from both (4096 capped `uint8` fields sit past `u32::MAX − 32`).
fn preflight_rejection_error(qualified: &str, location: &str, schema: &MessageSchema) -> CliError {
    let entries = offset_table_entries(schema, SizeMoment::Declared);
    let reason = match schema.checked_wire_fixed_size() {
        Err(e) => format!("its declared fixed section overflows the size arithmetic — {e}"),
        Ok(size) if exceeds_wire_ceiling(size, entries) => {
            let max = cerulion_core::wire::max_wire_fixed_size(entries);
            format!(
                "its declared frame prefix — the 32-byte header, the fixed section ({size} \
                 bytes) and the offset table ({entries} entries) — exceeds the u32 wire \
                 ceiling (the largest fixed section that frame can carry is {max} bytes)"
            )
        }
        Ok(_) => "its resolved frame prefix — the fixed section composed through fixed-nested \
                  inlining plus the resolved offset table — overflows the size arithmetic or \
                  exceeds the u32 wire ceiling"
            .to_string(),
    };
    CliError::Validation(format!(
        "schema '{qualified}' ({location}) has no wire layout — the resolution preflight \
         rejected it: {reason}. The declaration is hostile or corrupt (a FixedArray length \
         no frame could carry); fix the file to introspect it."
    ))
}

/// HOW a bare `node modify -o/-i` schema argument resolved.
///
/// A bare name (`Vector3`, `Foo`) is ambiguous by construction — it could
/// mean a built-in ROS 2 message, a workspace schema, or nothing. This
/// enum records which surface won so the caller (the
/// `node_cmd.rs` scaffolding + `cerulion_cli`'s `main.rs`) can both emit
/// the right `use`/port declaration AND surface the resolution loudly
/// (shadowing a built-in with a workspace schema is a fact the user
/// should see, not a silent hijack).
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum PortSchemaProvenance {
    /// The (normalized) argument already contained a `/` — a fully
    /// qualified `pkg/Type` that is NOT a store-only claim: a built-in, a
    /// slash-named workspace entry (file stem OR entry key — and the
    /// workspace tiers outrank the store, so a name BOTH define lands
    /// here, never on [`QualifiedStore`](Self::QualifiedStore)),
    /// or an unknown name. Passed through
    /// unchanged; this is the arm a prior BUILT-IN resolver output lands
    /// on when fed back in (idempotence). Fully-unknown qualified names
    /// deliberately keep this pass-through (existence gating is
    /// [`port_schema_exists`]'s job — see its doc).
    Qualified,
    /// The (normalized) argument contained a `/` AND names a workspace
    /// `.msg` store schema that NO workspace YAML tier claims (msg-store
    /// parity; workspace YAML outranks the store on qualified names too)
    /// — VALIDATED against the store rather than passed through blind.
    /// `schema` is the normalized string unchanged; the arm a prior STORE
    /// resolver output lands on when fed back in (idempotence). Scaffold
    /// call sites REFUSE this provenance via
    /// [`refuse_store_port_scaffold`] — a store type has no generated
    /// Rust type to import.
    QualifiedStore,
    /// A bare name resolved to exactly ONE built-in ROS 2 message.
    /// `qualified` is its canonical `pkg/Name` form.
    ResolvedBuiltin { qualified: String },
    /// A bare name matched a workspace schema, which WINS over any
    /// same-named built-in. `shadowed` carries the qualified name(s) of
    /// built-in(s) whose short name equals the bare name (comma-joined
    /// when several) so the caller can warn about the shadow; `None` when
    /// the bare name collides with no built-in.
    Workspace { shadowed: Option<String> },
    /// A bare name resolved to exactly ONE workspace `.msg` store schema
    /// (msg-store parity). The store tier sits BETWEEN workspace YAML and
    /// built-ins in the resolution ladder: it loses to a same-named
    /// workspace YAML schema and WINS over same-short-named built-ins.
    /// `qualified` is the store's `pkg/Type`; `shadowed` carries the
    /// qualified built-in(s) the store hit outranks (comma-joined), `None`
    /// when the bare name collides with no built-in. Scaffold call sites
    /// REFUSE this provenance via [`refuse_store_port_scaffold`] — a store
    /// type has no generated Rust type to import — so the outranked
    /// built-in is never silently scaffolded in its place either.
    ResolvedStore {
        qualified: String,
        shadowed: Option<String>,
    },
}

/// The result of resolving a bare-or-qualified port-schema argument:
/// the schema string to WRITE into the node source, plus HOW
/// it was resolved.
///
/// `schema` is what the caller should emit verbatim:
/// - `Qualified` / `ResolvedBuiltin` → a `pkg/Name` string;
/// - `Workspace` → the bare name UNCHANGED (workspace names stay bare —
///   the resolver never rewrites a workspace name to a built-in path).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ResolvedPortSchema {
    /// The resolved schema string to scaffold (`pkg/Name` for built-ins;
    /// the bare name unchanged for workspace schemas).
    pub schema: String,
    /// How `schema` was arrived at — see [`PortSchemaProvenance`].
    pub provenance: PortSchemaProvenance,
}

/// Refuse a port-schema resolution the NODE SCAFFOLDER cannot honor.
///
/// A `.msg` store type has no generated Rust type: `templates::
/// schema_to_import` maps every qualified name to
/// `native_ros2_messages::pkg::Type`, and that crate holds ONLY the
/// built-in corpus — so a port scaffolded from a store type mints a node
/// crate whose `use` can NEVER compile (unlike a bare workspace-YAML name,
/// which the author can satisfy in-crate). Until the store codegen front
/// door exists, the scaffold-consuming call sites (`node create` /
/// `node modify`, engine AND CLI) call this to refuse LOUDLY with the
/// remedy; store resolutions stay fully valid on every surface that needs
/// only the name/hash (`port_schema_exists`, `graph validate`'s identity
/// comparison, the combined-set hash/wire-size/replay folds).
pub fn refuse_store_port_scaffold(resolved: &ResolvedPortSchema, raw: &str) -> CliResult<()> {
    let qualified = match &resolved.provenance {
        PortSchemaProvenance::ResolvedStore { qualified, .. } => qualified.as_str(),
        PortSchemaProvenance::QualifiedStore => resolved.schema.as_str(),
        PortSchemaProvenance::Qualified
        | PortSchemaProvenance::ResolvedBuiltin { .. }
        | PortSchemaProvenance::Workspace { .. } => return Ok(()),
    };
    Err(CliError::Validation(format!(
        "schema '{raw}' names the workspace .msg store type '{qualified}', which has no \
         generated Rust type — the scaffolded `use native_ros2_messages::...;` import \
         could never compile, so no node port is created. For node ports use a built-in \
         ROS 2 type or a workspace YAML schema (`cerulion schema create`); `.msg` store \
         types remain first-class for `graph validate`, recording and replay."
    )))
}

/// Resolve a `node modify -o/-i` schema argument.
///
/// Without this resolver a BARE built-in name like `Vector3` would pass
/// through the scaffolder unchanged and emit `use Vector3;` → an E0432
/// unresolved-import compile error. This resolver maps a bare name to the
/// surface it actually names, loudly, before any code is scaffolded.
///
/// Resolution order (workspace WINS — a user's own `schemas/` content is
/// authoritative in their workspace, so a bare workspace name is never
/// silently hijacked to a same-named built-in; within the workspace, YAML
/// outranks the `.msg` store):
/// 1. [`normalize_schema`] first (`::` → `/`).
/// 2. Contains `/` → the ONE workspace lookup first (`workspace_lookup`,
///    which refuses a qualified twin and reports an unreadable stem file),
///    then the `.msg` store: a workspace-owned qualified name is
///    [`PortSchemaProvenance::Qualified`] (the documented
///    workspace-YAML-first ladder: checking
///    the store first would classify a collision `QualifiedStore` and the
///    scaffold gate would refuse a port the workspace legitimately owns, while
///    [`schema_info_unified`] and [`port_schema_exists`] resolve the SAME
///    name to YAML); a non-workspace `pkg/Type` naming a store entry is
///    [`PortSchemaProvenance::QualifiedStore`]; anything else is
///    [`PortSchemaProvenance::Qualified`]. `schema` = the normalized
///    string in every arm. This is the IDEMPOTENCE anchor: feeding a
///    prior resolver output (a `pkg/Name`) back in returns the string
///    unchanged. Fully-unknown qualified names keep the blind
///    pass-through DELIBERATELY — the callers scaffold node source, where
///    an unknown qualified name fails at compile time exactly as before,
///    and existence gating is [`port_schema_exists`]'s job.
/// 3. Bare + claimed by the workspace (`workspace_lookup`'s two tiers,
///    entry then file stem, in [`schema_info_unified`]'s order; the bound
///    definition is SIZED, so a port named after an entry the wire ceiling
///    refuses is refused here too) →
///    [`PortSchemaProvenance::Workspace`], `schema` = the bare name
///    unchanged, `shadowed` = the qualified built-in(s) it shadows.
/// 4. Bare + matching the `.msg` store
///    ([`SchemaStore::matches_bare_name`]):
///    - exactly 1 store package → [`PortSchemaProvenance::ResolvedStore`],
///      `schema` = the store's qualified `pkg/Type`, `shadowed` = the
///      qualified built-in(s) the store hit outranks;
///    - more than 1 → [`CliError::Validation`] via `ambiguous_store_name`
///      (loud — never fall through to built-ins on an ambiguous claim);
///    - exactly 1 whose qualified string a workspace YAML file ALSO declares
///      as an entry (`pkg/Type` in both) → [`CliError::Validation`] via
///      `workspace_lookup`'s twin refusal (the workspace definition
///      owns that string on every string-addressed surface, so the bare name
///      binds nothing). The QUALIFIED spelling is the same collision named
///      from the other side and is refused identically, so the only remedy
///      today is to remove one of the two definitions; naming one of them is
///      not supported.
///
///    NOTE: resolution succeeding is not scaffolding consent — the
///    scaffold-consuming call sites (`node create` / `node modify`) refuse
///    BOTH store provenances via [`refuse_store_port_scaffold`], because a
///    store type has no generated Rust type to import.
/// 5. Bare + in neither workspace tier → resolved against the built-in
///    registry by unique short name:
///    - exactly 1 built-in → [`PortSchemaProvenance::ResolvedBuiltin`];
///    - more than 1 → [`CliError::Validation`] (ambiguous — the user must
///      qualify), candidates listed package-ascending for stable text;
///    - none → [`CliError::SchemaNotFound`] carrying `PORT_SCHEMA_REMEDY`.
///
/// A missing/absent `schemas_dir` is NOT an error — it is treated as an
/// empty workspace (`workspace_lookup` reads it as empty). Empty, whitespace-only,
/// and garbage inputs (e.g. `""`, `"!!!"`) are bare with no `/`, so they
/// fall into the 0-candidate arm (`SchemaNotFound`).
pub fn resolve_port_schema(schemas_dir: &Path, raw: &str) -> CliResult<ResolvedPortSchema> {
    let normalized = normalize_schema(raw);

    // Already qualified (or made qualified by `::` → `/`) — the string
    // passes through untouched in every arm; the probes only decide the
    // provenance. The ONE workspace lookup runs first: it refuses a
    // qualified spelling the workspace spells TWICE (a nested
    // `schemas/pkg/Name.yaml` beside `schemas/pkg/msg/Name.msg`), which the
    // authoring verbs (`node create -o` / `-i`, `node modify`) must not
    // accept when the run and the report refuse it; a workspace
    // definition of the name then classifies with workspace provenance, so
    // the scaffold gate does not refuse a port the workspace owns
    // (store-first here would contradict the
    // workspace-YAML-first ladder the sibling surfaces honor). Then the
    // store probe (validated vs blind). Idempotence rests here: any
    // `pkg/Name` (incl. a prior resolver output) contains `/` and returns
    // unchanged. Store probe is existence only — never
    // `store_schema_lookup`, which materializes a layout (the
    // hostile-`.msg` panic class).
    if normalized.contains('/') {
        let provenance = if let Some(lookup) = workspace_lookup(schemas_dir, &normalized)? {
            workspace_binding_preflight(schemas_dir, &lookup, &normalized)?;
            PortSchemaProvenance::Qualified
        } else if SchemaStore::load(schemas_dir).resolves(&normalized) {
            PortSchemaProvenance::QualifiedStore
        } else {
            PortSchemaProvenance::Qualified
        };
        return Ok(ResolvedPortSchema {
            schema: normalized,
            provenance,
        });
    }

    // Bare name — the workspace tiers win over the store and built-ins, from
    // the ONE lookup (an ambiguous spelling was refused by it, an unreadable
    // stem file reported as unchecked); the bound document is then SIZED
    // (`workspace_binding_preflight` — the wire ceiling's refusal, the same
    // text `schema info` gives it).
    if let Some(lookup) = workspace_lookup(schemas_dir, &normalized)? {
        workspace_binding_preflight(schemas_dir, &lookup, &normalized)?;
        let shadowed_builtins = builtins_named(&normalized);
        let shadowed = if shadowed_builtins.is_empty() {
            None
        } else {
            Some(shadowed_builtins.join(", "))
        };
        return Ok(ResolvedPortSchema {
            // Workspace names stay bare — never rewritten to a built-in path.
            schema: normalized,
            provenance: PortSchemaProvenance::Workspace { shadowed },
        });
    }

    // A bare spelling one NESTED sole-entry file binds is checked for its
    // twins BEFORE the store may bind it: nested YAML + store, or two nested
    // files, is the decided refusal, and it is applied at the SAME
    // point on all three surfaces (`schema info`, this resolver, the
    // existence gate) so none can accept what another refuses.
    refuse_ambiguous_bare(schemas_dir, &normalized)?;

    // `.msg` store tier (msg-store parity): a bare name matching exactly
    // one store package resolves to its qualified form — provided that
    // qualified spelling is the store's to bind (a workspace YAML
    // entry declaring the same `pkg/Type` string owns it everywhere, so the
    // bare name is REFUSED rather than bound to a definition the served,
    // recorded and replayed string never resolves to); an ambiguous one
    // errors loudly (the same shared message `schema info` and
    // `port_schema_exists` raise) rather than falling through to built-ins.
    // ONE verdict for both port surfaces: `bare_store_port_binding`, judged
    // over the SAME parsed YAML claims every consumer derives its store
    // claim from. The nested-YAML/store twin of a bare spelling is
    // `refuse_ambiguous_bare`'s verdict, below.
    if let Some(qualified) =
        bare_store_port_binding(schemas_dir, &SchemaStore::load(schemas_dir), &normalized)?
    {
        let shadowed_builtins = builtins_named(&normalized);
        let shadowed = if shadowed_builtins.is_empty() {
            None
        } else {
            Some(shadowed_builtins.join(", "))
        };
        return Ok(ResolvedPortSchema {
            schema: qualified.clone(),
            provenance: PortSchemaProvenance::ResolvedStore {
                qualified,
                shadowed,
            },
        });
    }

    // In neither workspace nor store tier — resolve against the built-in
    // registry by unique short (message) name.
    let candidates = builtins_named(&normalized);
    match candidates.as_slice() {
        [qualified] => Ok(ResolvedPortSchema {
            schema: qualified.clone(),
            provenance: PortSchemaProvenance::ResolvedBuiltin {
                qualified: qualified.clone(),
            },
        }),
        [] => Err(CliError::SchemaNotFound {
            name: normalized,
            remedy: PORT_SCHEMA_REMEDY,
        }),
        _ => Err(CliError::Validation(format!(
            "schema '{}' is ambiguous — it exists in multiple built-in packages: {}. \
             Qualify it, e.g. '{}'.",
            normalized,
            candidates.join(", "),
            candidates[0]
        ))),
    }
}

/// Qualified built-in names (`"pkg/Name"`) whose short (message) name
/// equals `bare`. [`native_ros2_messages::BUILTIN_MSGS`] is ascending by
/// `(package, name)`, so the returned names are already package-ascending
/// — the deterministic order both the ambiguity error text and the
/// [`PortSchemaProvenance::Workspace`] shadow marker rely on.
fn builtins_named(bare: &str) -> Vec<String> {
    native_ros2_messages::BUILTIN_MSGS
        .iter()
        .filter(|&&(_, msg, _)| msg == bare)
        .map(|&(pkg, msg, _)| format!("{}/{}", pkg, msg))
        .collect()
}

/// One workspace-local schema in a [`SchemaListing`].
/// A store row's hash column: the `0x…` wire hash, or the loud marker for
/// a declaration that cannot be sized (never a fabricated `0x0`).
fn render_listing_hash(hash: Option<u64>) -> String {
    match hash {
        Some(h) => format!("0x{h:016x}"),
        None => {
            "(unhashable: rejected by the size preflight — hostile or over-ceiling declaration)"
                .to_string()
        }
    }
}

#[derive(Debug)]
pub struct WorkspaceSchemaEntry {
    /// Schema name (the YAML `schemas:` mapping key, not the file stem).
    pub name: String,
    /// Recipe-3 hash of the single-file parse — the same value
    /// [`schema_info`] displays for this entry; `None` when the entry's
    /// frame prefix fails the wire ceiling — as DECLARED (`schema
    /// info` REFUSES such an entry) or as RESOLVED through fixed-nested
    /// inlining (the same verdict a store row gets from its
    /// resolved layout) — in which case the row lists with the unhashable
    /// marker, exactly like a rejected store row.
    pub schema_hash: Option<u64>,
    /// File the schema is declared in (`<stem>.yaml`; the stem is the
    /// `schema info` lookup key).
    pub file: String,
    /// The refusal the workspace tier gives this entry's NAME when it is
    /// ambiguous (ambiguity-refusal rule — declared by several files, or
    /// colliding with a stem file): the row prints it, every source named,
    /// instead of a hash, and `schema_hash` is `None`.
    pub refusal: Option<String>,
}

/// One built-in type inside a [`SchemaListing`] package group.
#[derive(Debug)]
pub struct BuiltinTypeEntry {
    /// Bare message name (`Image`).
    pub name: String,
    /// Name of the workspace YAML schema that shadows this built-in in
    /// `schema info` lookups, if any — the same bare-or-qualified
    /// collision semantics as [`UnifiedSchemaInfo::shadowed_builtin`].
    /// Workspace YAML wins over the store, so this takes render precedence
    /// over [`shadowed_by_store`](Self::shadowed_by_store).
    pub shadowed_by: Option<String>,
    /// Qualified name of the `.msg` store entry that shadows this built-in,
    /// if any; the store wins over the built-in but LOSES to a
    /// workspace YAML shadow, so this is rendered only when `shadowed_by`
    /// is `None`.
    pub shadowed_by_store: Option<String>,
}

/// One built-in package group in a [`SchemaListing`].
#[derive(Debug)]
pub struct BuiltinPackageGroup {
    pub package: String,
    /// Types in registry order (ascending by name within the package).
    pub types: Vec<BuiltinTypeEntry>,
}

/// One `.msg` store schema in a [`SchemaListing`].
#[derive(Debug)]
pub struct StoreSchemaEntry {
    /// Qualified name `pkg/Type`.
    pub name: String,
    /// The `<pkg>` path segment — the grouping key in the listing.
    pub package: String,
    /// Layout-resolved recipe-3 hash (over built-ins + the full store) so a
    /// nested store type reports the wire-contract hash that matches a
    /// compile-time twin; falls back to the single-file hash if a nested
    /// reference is absent from the set.
    /// The resolved wire-contract hash of THIS store definition (it is
    /// resolved over a store-LAST set, so a slash-named workspace YAML twin
    /// can never supply the number printed beside the store path); `None`
    /// when the resolution preflight REJECTED the definition or its resolved
    /// layout exceeds the u32 wire ceiling (the row still LISTS,
    /// rendered unhashed — never a hash of the raw declaration).
    pub schema_hash: Option<u64>,
    /// Store-relative provenance path `pkg/msg/Type.msg`.
    pub relative_path: String,
    /// True when this qualified name also exists in the built-in registry —
    /// the store copy WINS at resolution, so the shadow is marked inline.
    pub shadows_builtin: bool,
}

/// `schema list` result: workspace-local YAML schemas,
/// the `.msg` store, and the embedded built-in registry grouped by package.
#[derive(Debug)]
pub struct SchemaListing {
    /// Workspace YAML schemas, sorted by file name then declaration order.
    pub workspace: Vec<WorkspaceSchemaEntry>,
    /// `.msg` store schemas, sorted by qualified name (so entries
    /// group by package).
    pub store: Vec<StoreSchemaEntry>,
    /// Built-in packages in registry order (ascending package name).
    pub builtin_packages: Vec<BuiltinPackageGroup>,
    /// Total built-in type count (`BUILTIN_MSGS.len()`).
    pub builtin_total: usize,
}

impl fmt::Display for SchemaListing {
    /// Compact, scannable listing: the workspace group one line per
    /// schema (name, hash, declaring file), then per-package built-in
    /// headers with wrapped comma-separated type names — never one line
    /// per built-in type. Shadowed built-ins are marked inline.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        writeln!(
            f,
            "Workspace schemas (schemas/*.yaml): {}",
            self.workspace.len()
        )?;
        for entry in &self.workspace {
            let verdict = match &entry.refusal {
                Some(refusal) => format!("(refused: {refusal})"),
                None => render_listing_hash(entry.schema_hash),
            };
            writeln!(f, "  {}  {}  ({})", entry.name, verdict, entry.file)?;
        }
        writeln!(f)?;

        // The `.msg` store, grouped by package. Present even when
        // empty (a `0` count is accurate — the store simply has no schemas).
        // `store` is sorted by qualified name, so same-package entries are
        // adjacent — count distinct packages by adjacent transitions.
        let mut store_packages = 0usize;
        let mut seen_pkg: Option<&str> = None;
        for entry in &self.store {
            if seen_pkg != Some(entry.package.as_str()) {
                store_packages += 1;
                seen_pkg = Some(entry.package.as_str());
            }
        }
        writeln!(
            f,
            "Schemas from the .msg store (schemas/<pkg>/msg): {} in {} packages",
            self.store.len(),
            store_packages
        )?;
        let mut current_pkg: Option<&str> = None;
        for entry in &self.store {
            if current_pkg != Some(entry.package.as_str()) {
                writeln!(f, "  {} (msg store):", entry.package)?;
                current_pkg = Some(entry.package.as_str());
            }
            let shadow = if entry.shadows_builtin {
                " (shadows built-in)"
            } else {
                ""
            };
            writeln!(
                f,
                "    {}  {}  ({}){}",
                entry.name,
                render_listing_hash(entry.schema_hash),
                entry.relative_path,
                shadow
            )?;
        }
        writeln!(f)?;

        writeln!(
            f,
            "Built-in ROS 2 messages: {} in {} packages",
            self.builtin_total,
            self.builtin_packages.len()
        )?;
        for group in &self.builtin_packages {
            writeln!(f, "  {} ({}):", group.package, group.types.len())?;
            // Wrap the comma-separated names at a fixed width so wide
            // packages (moveit_msgs: 46 types) stay readable. The wrap
            // point is content-deterministic — same listing, same lines.
            let mut line = String::new();
            for entry in &group.types {
                // Workspace YAML wins over the store wins over the built-in,
                // so a YAML shadow takes render precedence over a store
                // shadow; the built-in is bare only when neither shadows it.
                let rendered = match (&entry.shadowed_by, &entry.shadowed_by_store) {
                    (Some(ws), _) => format!("{} (shadowed by workspace '{}')", entry.name, ws),
                    (None, Some(store)) => {
                        format!("{} (shadowed by msg store '{}')", entry.name, store)
                    }
                    (None, None) => entry.name.clone(),
                };
                if line.is_empty() {
                    line.push_str(&rendered);
                } else if line.len() + 2 + rendered.len() > 72 {
                    writeln!(f, "    {}", line)?;
                    line.clear();
                    line.push_str(&rendered);
                } else {
                    line.push_str(", ");
                    line.push_str(&rendered);
                }
            }
            if !line.is_empty() {
                writeln!(f, "    {}", line)?;
            }
        }
        Ok(())
    }
}

/// List ALL known schemas: workspace-local
/// `schemas/*.yaml` entries, the `.msg` store
/// (`schemas/<pkg>/msg/<Type>.msg`), and the embedded built-in ROS 2
/// registry grouped by package.
///
/// Workspace enumeration mirrors
/// [`build_workspace_schema_hashes`](crate::graph_cmd::build_workspace_schema_hashes)'
/// best-effort filtering semantics: a missing `schemas/` directory is
/// quiet (`debug!`), while an unreadable directory, an unreadable
/// directory ENTRY, an unreadable file, and a malformed
/// YAML are ALL `warn!`-logged and SKIPPED — a listing must not fail
/// because one file is mid-edit, and no skip is ever silent. Files are
/// processed in sorted order (`read_dir` order is OS-dependent) so the
/// listing renders byte-identically across runs (Principle #7).
///
/// Infallible by design (mirrors the anchor's non-fatal contract): the
/// built-in registry half always lists.
pub fn schema_list(schemas_dir: &Path) -> SchemaListing {
    let mut workspace: Vec<WorkspaceSchemaEntry> = Vec::new();

    // The workspace tier's ONE verdict per name (ambiguity-refusal rule):
    // a row whose NAME is ambiguous prints the refusal, every source named.
    let yaml_claims = workspace_yaml_bare_claims_of(&parsed_workspace_yaml(schemas_dir));
    // Sorted enumeration with every skip logged (incl. unreadable
    // dirents) lives in `workspace_yaml_files`. This walk's
    // own read/parse skips log at `debug!`: the claims walk just
    // above WARNED once per broken file.
    // The rows' hashes are gated on the RESOLVED
    // layout through the ONE preflighted resolution set — the store rows'
    // discipline — so a composition that crosses the ceiling only after
    // fixed-nested inlining lists UNHASHED instead of advertising a hash no
    // frame can carry. The gate is BOUND to the
    // row's own definition. The set is YAML-last with last-wins dedup, so
    // `layout_of(name)` is the YAML definition's layout ONLY while that
    // definition is in the set; a definition the preflight dropped (over the
    // ceiling as declared, or a composed overflow) leaves the name to a
    // lower-tier twin — a slash-named `geometry_msgs/Vector3` YAML entry
    // fell through to the BUILT-IN and listed a hash for an unrepresentable
    // declaration. The survivors are keyed by their parse-time hash (the
    // YAML cap bounds it; every other member passed the sizable retain).
    let resolution_set = resolution_schema_set(Some(schemas_dir));
    let preflight_survivors: std::collections::HashSet<u64> =
        resolution_set.iter().map(|s| s.schema_hash()).collect();
    let (mut resolver, _warnings) = LayoutResolver::new(resolution_set);
    for path in workspace_yaml_files(schemas_dir) {
        let content = match std::fs::read_to_string(&path) {
            Ok(content) => content,
            Err(e) => {
                tracing::debug!(
                    file = %path.display(),
                    error = %e,
                    "could not read schema file for listing"
                );
                continue;
            }
        };
        let file_name = path
            .file_name()
            .map(|n| n.to_string_lossy().into_owned())
            .unwrap_or_default();
        match parse_message_schemas(&content) {
            Ok(schemas) => {
                for schema in schemas {
                    // The ONE wire ceiling, before the hash is
                    // materialized — a declared fixed section past the frame
                    // prefix lists UNHASHED (the store row's verdict), never
                    // with a hash no frame can carry. The parser's array cap
                    // still makes the recipe panic-free here (see
                    // `schema_info`).
                    let refusal = match yaml_claims.get(&normalize_schema(&schema.name)) {
                        Some(YamlClaim::Refused(ambiguous)) => Some(ambiguous.message()),
                        _ => None,
                    };
                    // Judged on the RESOLVED layout of THIS
                    // definition — one the preflight dropped is unhashed before
                    // the resolver is asked (its name may now resolve to a
                    // lower tier's twin), a composed-over one is refused by
                    // `representable_layout`. The hash itself stays the
                    // parse-time value `schema info` displays.
                    let schema_hash = (refusal.is_none()
                        && preflight_survivors.contains(&schema.schema_hash())
                        && representable_layout(
                            resolver.layout_of(&schema.name),
                            "schema list workspace rows",
                        )
                        .is_some())
                    .then(|| schema.schema_hash());
                    workspace.push(WorkspaceSchemaEntry {
                        name: schema.name.clone(),
                        schema_hash,
                        file: file_name.clone(),
                        refusal,
                    });
                }
            }
            Err(e) => {
                tracing::debug!(
                    file = %path.display(),
                    error = %e,
                    "could not parse schema file for listing"
                );
            }
        }
    }

    // The `.msg` store. Hashes resolve over built-ins + the FULL
    // store (nested references), so a nested store type lists the same
    // wire-contract hash `schema info` reports; a store type the preflight
    // rejected lists unhashed.
    let store = SchemaStore::load(schemas_dir);
    let mut store_entries: Vec<StoreSchemaEntry> = Vec::new();
    if !store.is_empty() {
        // The shared tiers + preflight (built-ins + store only
        // would diverge from the renderer + graph-hash surfaces for store fields
        // referencing workspace YAML types), ordered store-last:
        // a row names a store file, so its hash must be
        // that definition's — under the canonical YAML-last order a
        // slash-named YAML twin `pkg/Type` would be the string winner and its
        // hash would print beside the store path. Identity binding of nested
        // refs is order-independent, so every nested dependency still
        // resolves exactly as on the other surfaces (see
        // `ResolutionTiers::combined_store_last`).
        let (mut resolver, _warnings) = LayoutResolver::new(store_resolution_set(schemas_dir));
        for (qualified, stored) in store.iter() {
            // A definition the preflight rejected — or whose resolved
            // layout exceeds the u32 wire ceiling — has NO hash to list (the
            // earlier checked single-file fallback still minted a hash of the RAW
            // declaration that nothing on the wire could match); the row lists
            // with the unhashable marker instead — the same verdict `schema
            // info` gives it.
            let schema_hash =
                representable_layout(resolver.layout_of(qualified), "schema list store rows")
                    .map(|l| l.schema_hash);
            let package = qualified
                .split_once('/')
                .map(|(p, _)| p.to_string())
                .unwrap_or_default();
            store_entries.push(StoreSchemaEntry {
                name: qualified.to_string(),
                package,
                schema_hash,
                relative_path: stored.relative_path.clone(),
                shadows_builtin: builtin_has_qualified(qualified),
            });
        }
    }

    // Group the registry by package, marking any shadows from the workspace
    // YAML + `.msg` store (extracted so the no-workspace path reuses the
    // exact grouping with no shadows).
    let builtin_packages = builtin_package_groups(&workspace, &store);

    SchemaListing {
        workspace,
        store: store_entries,
        builtin_packages,
        builtin_total: native_ros2_messages::BUILTIN_MSGS.len(),
    }
}

/// Group [`native_ros2_messages::BUILTIN_MSGS`] by package (registry order —
/// ascending by `(package, name)`, so one linear pass groups deterministically),
/// marking each type shadowed by a workspace YAML schema (`shadowed_by`) or a
/// `.msg` store entry (`shadowed_by_store`). Shared by [`schema_list`] and the
/// no-workspace [`schema_list_opt`] (which passes an empty workspace + store, so
/// nothing shadows).
fn builtin_package_groups(
    workspace: &[WorkspaceSchemaEntry],
    store: &SchemaStore,
) -> Vec<BuiltinPackageGroup> {
    let mut builtin_packages: Vec<BuiltinPackageGroup> = Vec::new();
    for &(pkg, msg, _) in native_ros2_messages::BUILTIN_MSGS {
        let qualified = format!("{}/{}", pkg, msg);
        // A refused workspace name binds nothing, so it shadows nothing.
        // Matched on the ONE canonical identity, like `shadowed_builtin_names`
        // (the `schema info` half of this same question) and like
        // `workspace_lookup`: an entry declared `std_msgs::String` shadows the
        // built-in exactly as `std_msgs/String` does, and a raw comparison
        // left `schema list` saying the built-in was reachable while
        // `schema info` resolved the workspace copy. The row still RENDERS
        // the declared spelling.
        let shadowed_by = workspace
            .iter()
            .find(|w| {
                w.refusal.is_none() && {
                    let declared = normalize_schema(&w.name);
                    declared == msg || declared == qualified
                }
            })
            .map(|w| w.name.clone());
        // A store shadow is by QUALIFIED name only (store keys are always
        // `pkg/Type`); workspace YAML shadows take render precedence.
        let shadowed_by_store = if store.resolves(&qualified) {
            Some(qualified.clone())
        } else {
            None
        };
        let entry = BuiltinTypeEntry {
            name: msg.to_string(),
            shadowed_by,
            shadowed_by_store,
        };
        if builtin_packages.last().is_none_or(|g| g.package != pkg) {
            builtin_packages.push(BuiltinPackageGroup {
                package: pkg.to_string(),
                types: Vec::new(),
            });
        }
        builtin_packages
            .last_mut()
            .expect("a group for this package was just ensured above")
            .types
            .push(entry);
    }
    builtin_packages
}

/// `schema list` that works WITHOUT a workspace.
///
/// `Some(dir)` behaves EXACTLY like [`schema_list`]. `None` means no workspace
/// anywhere up the tree: the workspace-YAML and `.msg`-store halves are SKIPPED
/// entirely (never enumerate a stray non-workspace `schemas/` from the cwd) and
/// the listing is the embedded built-in registry alone — so `schema list` on a
/// brand-new machine still shows every resolvable built-in type (the usual shape,
/// minus the local halves).
pub fn schema_list_opt(schemas_dir: Option<&Path>) -> SchemaListing {
    match schemas_dir {
        Some(dir) => schema_list(dir),
        None => SchemaListing {
            workspace: Vec::new(),
            store: Vec::new(),
            builtin_packages: builtin_package_groups(&[], &SchemaStore::empty()),
            builtin_total: native_ros2_messages::BUILTIN_MSGS.len(),
        },
    }
}

/// Field-parse loop for [`schema_info`]'s display rows.
///
/// Walks the `fields:` mapping of one schema document in declaration order,
/// rejecting non-string field keys loudly, parsing each `<type> <name>` key
/// through the core parser ([`cerulion_core::dynamic::parse_field_key`] —
/// the same strict 2-token form + `FixedArray`/`StringFixed` length guards
/// [`parse_message_schemas`] applies), and pushing each parsed field onto
/// `schema`. Returns the per-field display entries `schema info` needs.
fn parse_schema_fields_into(
    schema_name: &str,
    schema_def: &serde_yaml::Value,
    schema: &mut MessageSchema,
) -> CliResult<Vec<SchemaFieldEntry>> {
    let schema_def = schema_def.as_mapping().ok_or_else(|| {
        CliError::Validation(format!(
            "schema '{}': definition must be a mapping (`description:` / `fields:`), got a scalar, sequence or null",
            schema_name
        ))
    })?;
    let mut field_entries = vec![];
    let fields = schema_def
        .get(serde_yaml::Value::from("fields"))
        .filter(|value| !value.is_null());
    if let Some(fields) = fields {
        let fields = fields.as_mapping().ok_or_else(|| {
            CliError::Validation(format!(
                "schema '{}': `fields:` must be a mapping of '<type> <name>' keys",
                schema_name
            ))
        })?;
        for (field_key, _value) in fields {
            let key_str = field_key.as_str().ok_or_else(|| {
                CliError::Validation(format!(
                    "schema '{}': field key {:?} is not a string; expected \
                     '<type> <name>' (e.g. 'uint32 height')",
                    schema_name, field_key
                ))
            })?;
            let (field_type, field_name) =
                cerulion_core::dynamic::parse_field_key(schema_name, key_str)
                    .map_err(|e| CliError::Validation(e.to_string()))?;
            let is_nested = field_type_is_nested(&field_type);
            field_entries.push(SchemaFieldEntry {
                name: field_name.clone(),
                // Clean canonical string — a typo'd primitive (e.g.
                // `unit32`) or an unverified nested reference is surfaced
                // loudly by the renderer's `unknown type — no schema found`
                // annotation when it resolves against nothing, not by a
                // marker baked into this string.
                type_display: field_type.canonical_str(),
                is_nested,
                // Parse-time classification: unresolved nested references
                // are conservatively variable — the same rule
                // `schema_hash()` / `wire_fixed_size()` apply to them.
                is_variable: field_type.is_variable(),
                field_type: field_type.clone(),
            });
            schema.add_field(FieldDef::new(field_name, field_type));
        }
    }
    Ok(field_entries)
}

/// True if `field_type` is a nested schema reference, OR an array
/// (`FixedArray` / `DynamicArray`, possibly nested) whose innermost
/// element is a nested reference.
///
/// `FieldType::parse` is recursive, so `Header[] readings` parses as
/// `DynamicArray { element_type: Nested { "Header" } }` and `Header[3]` as
/// `FixedArray { element_type: Nested { "Header" }, .. }`. A bare
/// `matches!(.., Nested { .. })` test only fires for the top-level
/// `Nested` variant, so an array-of-nested field (or a typo like
/// `Heaer[] readings`, where `Heaer` parses as an unknown nested ref)
/// would silently be treated as a leaf — the unified renderer would neither
/// expand it nor flag it. Recursing through array element types keeps the
/// `is_nested` classification (and thus the renderer's expand/flag decision)
/// correct.
fn field_type_is_nested(field_type: &FieldType) -> bool {
    match field_type {
        FieldType::Nested { .. } => true,
        FieldType::FixedArray { element_type, .. } | FieldType::DynamicArray { element_type } => {
            field_type_is_nested(element_type)
        }
        _ => false,
    }
}

/// Result of schema info lookup.
#[derive(Debug)]
pub struct SchemaInfoResult {
    pub entries: Vec<SchemaEntry>,
}

/// A single schema entry (workspace YAML or built-in).
#[derive(Debug)]
pub struct SchemaEntry {
    pub name: String,
    pub description: String,
    pub field_count: usize,
    /// Parsed fields in declaration order. Each carries a CLEAN canonical
    /// type string plus its structured [`FieldType`]; the unified renderer
    /// resolves every non-primitive field's schema and expands it inline,
    /// flagging an unresolvable / typo'd reference loudly on its field line
    /// (`… unknown type — no schema found`) rather than via a marker baked
    /// into the string (see [`SchemaFieldEntry::type_display`]).
    pub fields: Vec<SchemaFieldEntry>,
    /// The entry's wire hash — `None` when NO frame can carry the entry
    /// because its RESOLVED frame prefix crosses the u32 wire ceiling
    /// through fixed-nested inlining, so the parse-time hash would name a
    /// contract no producer can stamp; `schema info` then renders the same
    /// unhashable marker `schema list` gives the row, and `schema list` and
    /// `schema info` agree.
    pub schema_hash: Option<u64>,
    /// Wire fixed-section size in bytes. Workspace entries
    /// compute it from the parsed IR ([`MessageSchema::wire_fixed_size`],
    /// where unresolved nested references classify as variable — matching
    /// the hash recipe's input); built-in entries carry the resolver's
    /// `WireLayout::fixed_size`, which equals the generated type's
    /// `WIRE_FIXED_SIZE`.
    pub wire_fixed_size: usize,
    /// True when `wire_fixed_size` is the canonically RESOLVED layout
    /// (built-in entries always; workspace entries once enrichment
    /// succeeds). False = the parse-time value stands (duplicate-name
    /// skip / unresolvable arms) — the renderer marks it loudly with a
    /// ` (unresolved)` suffix (an unresolved
    /// size must never print identically to a resolved one).
    pub wire_fixed_size_resolved: bool,
}

/// One parsed field of a schema, ready for display.
#[derive(Debug)]
pub struct SchemaFieldEntry {
    pub name: String,
    /// Clean canonical type string (`uint32`, `float64[36]`,
    /// `geometry_msgs/Vector3`, `MotorState[20]`). The unified renderer
    /// (CER — schema-info tree) resolves each non-primitive field's schema
    /// and expands it inline, so an unresolvable / typo'd reference is
    /// flagged loudly on the field line as `(… unknown type — no schema
    /// found)` rather than carrying a marker in this string.
    pub type_display: String,
    pub is_nested: bool,
    /// True when the field rides the offset table (variable payload)
    /// rather than the fixed section. Workspace entries use the
    /// parse-time classification (unresolved nested = variable); built-in
    /// entries use the resolved `WireLayout`'s fixed/variable split.
    pub is_variable: bool,
    /// The structured field type — drives the unified renderer's recursive
    /// inline expansion (which nested type to expand, and the array element
    /// type for `Type[N]` / `Type[]`). Kept alongside [`Self::type_display`]
    /// (which is just its `canonical_str()`) so the renderer never has to
    /// re-parse the string.
    pub field_type: FieldType,
}

#[cfg(test)]
mod tests {
    use super::*;

    // ======================================================================
    // The fault seam's CONFINEMENT, enforced.
    // ======================================================================

    /// `force_scrub_fault` MUTATES the return value of
    /// `retain_composed_sizable_schemas` — a function on the `graph run`,
    /// `schema serve` and replay-registry paths. Its whole safety argument is
    /// that the module AND its single call site are `#[cfg(test)]`, so
    /// neither exists in the library `cerulion_cli` and the integration-test
    /// binaries link. That is true today, and until this test nothing
    /// enforced it: someone re-gating the seam on a Cargo feature, or
    /// dropping the attribute while chasing an integration-test repro, would
    /// arm a fault injector in shipped code with no gate firing.
    ///
    /// Deliberately a SOURCE walk, not a `cfg` assertion: the compiler cannot
    /// tell you about a gate that is no longer there.
    ///
    /// LINE-based, and NOT comment-stripped — which is the opposite of the
    /// usual guard in this repo, for a reason worth recording. The standard
    /// `code_only` stripper does not model string literals, and this file is
    /// full of the literal `schemas/` + `*` + `.yaml`: every one of those
    /// opens a block comment the stripper never closes, so a stripped view of
    /// THIS file swallows the seam entirely (measured: 13 unbalanced opens,
    /// both needles absent). The usual escape — have the guard file exclude
    /// itself — is unavailable when the guard and the seam must live in the
    /// same module. So the walk skips comment LINES instead, which needs no
    /// literal model, and the needles are assembled with `concat!` so the
    /// test's own source does not match them.
    #[test]
    fn the_force_scrub_fault_seam_is_confined_to_cfg_test() {
        let src =
            std::fs::read_to_string(concat!(env!("CARGO_MANIFEST_DIR"), "/src/schema_cmd.rs"))
                .expect("this module's own source");

        // Assembled, so these exact strings do not appear in this file and
        // the guard cannot match itself.
        let decl = concat!("pub(crate) mod ", "force_scrub_fault {");
        // Anchored on the SYMBOL, not the argument list: a second, ungated
        // call spelled with different arguments — precisely the "dropped the
        // attribute while chasing a repro" case this guards against — must
        // not hide behind a needle that names `all_dropped`.
        let call = concat!("force_scrub_fault::", "degrade_if_armed(");

        // Code lines only: a comment LINE is not a call site, and the seam's
        // own doc block names every symbol below in order to explain it.
        // `*`-leading lines are block-comment continuations — EXCEPT a deref
        // assignment (`*hash = …`), which is real code; swallowing those made
        // "immediately preceded by" mean "preceded by, ignoring derefs".
        let is_comment = |l: &str| {
            l.starts_with("//") || l.starts_with("/*") || (l.starts_with('*') && !l.contains('='))
        };
        let code: Vec<(usize, &str)> = src
            .lines()
            .enumerate()
            .map(|(i, l)| (i, l.trim()))
            .filter(|(_, l)| !l.is_empty() && !is_comment(l))
            .collect();

        for needle in [decl, call] {
            let hits: Vec<usize> = code
                .iter()
                .enumerate()
                .filter(|(_, (_, l))| l.starts_with(needle))
                .map(|(pos, _)| pos)
                .collect();
            // ANTI-TAUTOLOGY: a rename or a broken walk would leave this
            // vacuous, so the premise is asserted rather than assumed.
            assert_eq!(
                hits.len(),
                1,
                "premise: exactly ONE code line must start with `{needle}` — \
                 got {}. If the seam moved or was renamed, this guard is \
                 vacuous, not satisfied.",
                hits.len()
            );
            let pos = hits[0];
            let prev = pos
                .checked_sub(1)
                .map(|p| code[p].1)
                .unwrap_or("<start of file>");
            assert_eq!(
                prev,
                "#[cfg(test)]",
                "`{needle}` (line {}) must be immediately preceded by \
                 `#[cfg(test)]`, got `{prev}`. Arming a fault injector in a \
                 shipped build is exactly what that attribute prevents, and it \
                 is the ONLY thing that does.",
                code[pos].0 + 1
            );
        }

        // A Cargo feature is NOT an acceptable substitute: features unify
        // across a workspace build, so one dependant enabling it would arm
        // the seam for every crate in the graph.
        assert!(
            !src.contains(concat!("feature = ", "\"fault")),
            "the seam must never be re-gated on a Cargo feature"
        );
    }

    // ======================================================================
    // `workspace_binding_preflight` never fails OPEN.
    //
    // A fallback that `return Ok(())`s — ACCEPTING the port
    // binding — when the file `workspace_lookup` had just bound cannot
    // be re-read, or when the entry is not found in it, is wrong: a warn on stderr
    // does not stop `graph validate` accepting a name every FOLD then
    // refuses, so an unverifiable binding is REFUSED
    // (`CliError::SchemaUnchecked` — main's shape for exactly "could not
    // check"), with the two conditions kept apart because their remedies
    // differ.
    //
    // Driven at the seam directly: the TOCTOU is by definition "the lookup
    // bound X, the file now says Y", and a hand-built `WorkspaceLookup` IS
    // that state — a single-threaded call through `port_schema_exists`
    // cannot reach it (the lookup read the very same file microseconds
    // earlier), which is exactly why the arms were unreachable before.
    // ======================================================================

    /// Every unverifiable binding is `SchemaUnchecked` (never `Ok`, never
    /// `Validation`), NAMES the file, and carries the condition's OWN
    /// reason. Hand-written expected substrings — never derived from the
    /// code under test.
    #[test]
    fn an_unverifiable_workspace_binding_is_refused_not_accepted() {
        let tmp = tempfile::tempdir().unwrap();
        let dir = tmp.path().join("schemas");
        std::fs::create_dir_all(&dir).unwrap();

        // (a) the bound file is GONE between the lookup and the preflight.
        let vanished = dir.join("Gone.yaml");
        let err = workspace_binding_preflight(
            &dir,
            &WorkspaceLookup::Entry {
                file: vanished.clone(),
            },
            "Gone",
        )
        .expect_err("a binding that cannot be re-read must be REFUSED");
        assert!(
            matches!(err, CliError::SchemaUnchecked(_)),
            "the 'could not check' channel, not a refusal or a build error: {err:?}"
        );
        let msg = err.to_string();
        assert!(
            msg.contains("schemas/Gone.yaml (bound by the port spelling)")
                && msg.contains("could not verify this schema's size")
                && msg.contains("could not be re-read"),
            "must name the FILE and say the read failed: {msg}"
        );

        // (b) the bound file no longer PARSES — a DIFFERENT remedy (fix the
        // YAML), so a different reason. A `parse_message_schemas(..).ok()`
        // would fold this into (c)'s "nothing to size", telling a user with a
        // syntax error to go looking for a deleted entry.
        let broken = dir.join("Broken.yaml");
        std::fs::write(&broken, "schemas:\n  Broken:\n    fields:\n   - : : [\n").unwrap();
        let err = workspace_binding_preflight(
            &dir,
            &WorkspaceLookup::Entry {
                file: broken.clone(),
            },
            "Broken",
        )
        .expect_err("a binding whose file no longer parses must be REFUSED");
        let parse_msg = err.to_string();
        assert!(
            matches!(err, CliError::SchemaUnchecked(_)),
            "the 'could not check' channel: {err:?}"
        );
        assert!(
            parse_msg.contains("schemas/Broken.yaml (bound by the port spelling)")
                && parse_msg.contains("the file no longer parses"),
            "must name the file and the PARSE condition: {parse_msg}"
        );

        // (c) the bound file parses but no longer DECLARES the entry — the
        // third remedy (restore the declaration, or re-point the port).
        let moved = dir.join("Moved.yaml");
        std::fs::write(
            &moved,
            "schemas:\n  SomethingElse:\n    fields:\n      uint32 a:\n",
        )
        .unwrap();
        let err =
            workspace_binding_preflight(&dir, &WorkspaceLookup::Entry { file: moved }, "Moved")
                .expect_err("a binding whose entry is gone must be REFUSED");
        let missing_msg = err.to_string();
        assert!(
            matches!(err, CliError::SchemaUnchecked(_)),
            "the 'could not check' channel: {err:?}"
        );
        assert!(
            missing_msg.contains("schemas/Moved.yaml (bound by the port spelling)")
                && missing_msg.contains("it no longer declares 'Moved'"),
            "must name the file and the MISSING ENTRY condition: {missing_msg}"
        );

        // (d) a STEM binding names the DECLARED entry, not the spelling the
        // port used. An ENTRY claim's key is canonical while a
        // bound STEM's entry is what the FILE declares, and that string now
        // feeds a user-facing refusal — so a user told "it no longer declares
        // 'X'" must be given the name to go and look for. Without this arm,
        // rewriting the bind as `[one] => (file, spelling.to_string())`
        // passes every other assertion in this file.
        let stem_gone = dir.join("StemGone.yaml");
        let err = workspace_binding_preflight(
            &dir,
            &WorkspaceLookup::Stem {
                file: stem_gone,
                entries: vec!["Declared".to_string()],
            },
            "stemgone",
        )
        .expect_err("a stem binding that cannot be re-read must be REFUSED");
        let stem_msg = err.to_string();
        assert!(
            stem_msg.contains("schemas/StemGone.yaml"),
            "the refusal names the FILE (the consumer frames the spelling): {stem_msg}"
        );

        // THE SPLIT, asserted as a difference rather than by text alone: the
        // parse failure and the missing entry are two conditions with two
        // remedies, so they must not render the same sentence.
        assert_ne!(
            parse_msg, missing_msg,
            "a file that does not parse and a file missing the entry are \
             different problems and must not print one message"
        );
        assert!(
            !missing_msg.contains("no longer parses"),
            "the missing-entry arm must not blame the syntax: {missing_msg}"
        );
    }

    /// ANTI-TAUTOLOGY: a binding the preflight CAN verify still passes, and
    /// an over-ceiling one still gets its own `Validation` refusal (not the
    /// new `SchemaUnchecked` channel) — the refusal must not turn the
    /// gate into a blanket refusal, and must not re-route the
    /// over-ceiling rejection.
    #[test]
    fn a_verifiable_binding_still_passes_and_a_hostile_one_still_refuses() {
        let tmp = tempfile::tempdir().unwrap();
        let dir = tmp.path().join("schemas");
        std::fs::create_dir_all(&dir).unwrap();

        let healthy = dir.join("Ok.yaml");
        std::fs::write(&healthy, "schemas:\n  Ok:\n    fields:\n      uint32 a:\n").unwrap();
        assert!(
            workspace_binding_preflight(&dir, &WorkspaceLookup::Entry { file: healthy }, "Ok")
                .is_ok(),
            "a verifiable, wire-representable binding passes"
        );

        // A stem file declaring several entries binds no single definition,
        // so there is nothing to size — still `Ok`, unchanged by this fix.
        let ambiguous = dir.join("Two.yaml");
        std::fs::write(
            &ambiguous,
            "schemas:\n  A:\n    fields:\n      uint32 a:\n  B:\n    fields:\n      uint32 b:\n",
        )
        .unwrap();
        assert!(
            workspace_binding_preflight(
                &dir,
                &WorkspaceLookup::Stem {
                    file: ambiguous,
                    entries: vec!["A".to_string(), "B".to_string()],
                },
                "Two",
            )
            .is_ok(),
            "a multi-entry stem binds nothing to SIZE — not this gate's call"
        );
        // Scope: this pins that the SIZING gate is silent on a
        // stem binding NO single definition (a different class from the three
        // arms above — they had a binding and could not verify it), NOT that
        // such a spelling is fine. A ZERO-entry stem answers the existence
        // probe deliberately (pinned by
        // `msg_store_parity_test::r27_*`'s `Vacant.yaml` arm); a MULTI-entry
        // stem is refused by `graph_cmd`'s `spelling_verdict` as
        // `AmbiguousStem`, but only on a VALIDATED run. This gate leaves
        // that as it is;
        // fixing it here would mean teaching `port_schema_exists` a new
        // refusal, which is a different subsystem's call.

        // TWO BEARERS OF ONE SPELLING, one hostile — REFUSED in BOTH
        // declaration orders, because every sibling surface binds NOTHING for
        // this shape.
        //
        // `pkg/Type` and `pkg::Type` are one identity to this gate and two
        // distinct YAML keys, so a file can declare both. The FOLD does not
        // "drop the hostile bearer and serve the survivor": two bearers in one
        // file push the same path twice into `workspace_yaml_bare_claims_of`'s
        // `declarers`, which takes the `many` arm and yields
        // `Refused(DuplicateEntry)`, and `build_workspace_schema_hashes`
        // `continue`s past a refused key — so NEITHER bearer is served.
        // `schema_info_entries` refuses too, judging every matching entry.
        //
        // Both orders are driven because picking either END of the bearer list
        // passes exactly one of them: a FIRST-bearer rule accepts the
        // healthy-first file, a LAST-bearer rule accepts the hostile-first one.
        let hostile_fields = {
            let mut y = String::new();
            for i in 0..4096 {
                y.push_str(&format!("      uint8[1048576] f{i}:\n"));
            }
            y
        };
        let healthy_first = format!(
            "schemas:\n  pkg/Type:\n    fields:\n      uint32 ok:\n  pkg::Type:\n    fields:\n{hostile_fields}"
        );
        let hostile_first = format!(
            "schemas:\n  pkg::Type:\n    fields:\n{hostile_fields}  pkg/Type:\n    fields:\n      uint32 ok:\n"
        );
        for (idx, (label, body)) in [
            ("healthy declared first", healthy_first),
            ("hostile declared first", hostile_first),
        ]
        .into_iter()
        .enumerate()
        {
            let pair = dir.join(format!("Pair{idx}.yaml"));
            std::fs::write(&pair, &body).unwrap();
            let err = match workspace_binding_preflight(
                &dir,
                &WorkspaceLookup::Entry { file: pair },
                "pkg/Type",
            ) {
                Err(e) => e.to_string(),
                Ok(()) => panic!(
                    "{label}: the binding was ACCEPTED, but the fold binds nothing \
                     for a duplicate identity and `schema info` refuses it"
                ),
            };
            assert!(
                err.contains("has no wire layout"),
                "{label}: a hostile bearer must refuse the spelling in EITHER order \
                 — every sibling surface binds nothing for it: {err}"
            );
        }

        // Over the u32 wire ceiling: the PRE-EXISTING rejection, on the
        // `Validation` channel. `uint8[1048576]` × 4096 fields sits past
        // `u32::MAX - 32`; one capped field is not enough, so the file
        // declares the maximum the YAML parser accepts, repeated.
        let hostile = dir.join("Huge.yaml");
        let mut yaml = String::from("schemas:\n  Huge:\n    fields:\n");
        for i in 0..4096 {
            yaml.push_str(&format!("      uint8[1048576] f{i}:\n"));
        }
        std::fs::write(&hostile, yaml).unwrap();
        let err =
            workspace_binding_preflight(&dir, &WorkspaceLookup::Entry { file: hostile }, "Huge")
                .expect_err("an over-ceiling binding is refused");
        assert!(
            matches!(err, CliError::Validation(_)),
            "the ceiling refusal keeps its own channel — a consumer renders a \
             REFUSAL verbatim and frames an UNCHECKED spelling itself: {err:?}"
        );
        assert!(
            err.to_string().contains("has no wire layout"),
            "the pre-existing text is unchanged: {err}"
        );
    }

    // ======================================================================
    // The advisory maps' TOP-LEVEL walk, pinned as
    // deliberate.
    //
    // `workspace_yaml_files` walks `schemas/*.yaml` only, while the
    // SPELLING surfaces reach a nested `schemas/<pkg>/<Type>.yaml` through
    // `workspace_lookup`'s stem tier (which probes that exact path). The
    // twin shape — a nested stem file beside a `.msg` store definition — is
    // REFUSED on both spellings and pinned by
    // `msg_store_parity_test::r26_a_nested_stem_yaml_twin_is_refused_on_both_spellings`,
    // which also pins the map's aliasing there.
    //
    // What that leaves, and what this pins, is the NON-twin half: a nested
    // file with no store twin RESOLVES as a port schema and is invisible to
    // the advisory maps. That is a DEGRADE, not a wrong answer (the maps
    // are advisory: a missing entry means "no advisory hash recorded"), and
    // widening the walk would change what six production call sites across
    // three modules index — a subsystem change, not a small patch. Pinned so a
    // nested-aware walk has to come back HERE and re-decide deliberately.
    // ======================================================================

    /// A nested `schemas/<pkg>/<Type>.yaml` with no store twin: the port
    /// surfaces BIND it, the advisory map does not INDEX it. Hand oracle on
    /// both halves — the point is the disagreement, so both are asserted.
    #[test]
    fn a_nested_yaml_binds_as_a_port_schema_but_is_absent_from_the_advisory_map() {
        let tmp = tempfile::tempdir().unwrap();
        let nested = tmp.path().join("schemas").join("pkg");
        std::fs::create_dir_all(&nested).unwrap();
        std::fs::write(
            nested.join("Nested.yaml"),
            "schemas:\n  pkg/Nested:\n    fields:\n      uint32 a:\n",
        )
        .unwrap();
        // A TOP-LEVEL file, to prove the map is populated at all — without
        // it an empty map would satisfy the absence assertion vacuously.
        std::fs::write(
            tmp.path().join("schemas").join("top.yaml"),
            "schemas:\n  TopLevel:\n    fields:\n      uint32 b:\n",
        )
        .unwrap();
        let schemas_dir = tmp.path().join("schemas");

        // The SPELLING surfaces reach it (the stem tier probes the path).
        assert!(
            port_schema_exists(&schemas_dir, "pkg/Nested").unwrap(),
            "the nested file binds the qualified spelling"
        );

        // The mechanism, pinned directly: the walk this map is built on sees
        // ONE file. Without this the absence below is only inferred.
        assert_eq!(
            workspace_yaml_files(&schemas_dir).len(),
            1,
            "the top-level walk sees `top.yaml` and NOT the nested file — that \
             walk is the mechanism the absence below rests on"
        );

        // The ADVISORY map does not index it — the top-level walk.
        let map = crate::graph_cmd::build_workspace_schema_hashes(tmp.path());
        assert!(
            map.contains_key("TopLevel"),
            "premise: the map IS populated, so the absence below is real"
        );
        assert_eq!(
            map.get("pkg/Nested"),
            None,
            "the advisory maps walk top-level `schemas/*.yaml` only — a \
             nested file is invisible to them. Deliberate: they are ADVISORY \
             (absent = no recorded hash, never a wrong one), and widening the \
             walk changes what every consumer of `workspace_yaml_files` \
             indexes. If you make them nested-aware, this assertion is the \
             one that must change."
        );
    }

    // ---------------- Remote schema-info tier ----------------

    /// `remote_fetch_target` — a QUALIFIED `pkg/Type` AND
    /// a package-less bare `Name` are BOTH remote-fetch candidates; a
    /// 3-segment name / empty name / empty chunk are not. Hand oracle.
    #[test]
    fn remote_fetch_target_accepts_qualified_and_bare() {
        assert_eq!(
            remote_fetch_target("vendor_msgs/Widget").as_deref(),
            Some("vendor_msgs/Widget")
        );
        // `::` form normalizes to `/`.
        assert_eq!(
            remote_fetch_target("vendor_msgs::Widget").as_deref(),
            Some("vendor_msgs/Widget")
        );
        // A package-less bare `Name` IS a candidate now
        // (a `cerulion schema create <Name>` workspace type served under its bare
        // key). The old-robot/new-desk skew degrades safely: an old robot's
        // queryable rejects the 1-chunk key → the bounded gather returns empty →
        // the caller re-raises the local SchemaNotFound (verified by the desk-tier
        // + live tests).
        assert_eq!(remote_fetch_target("Widget").as_deref(), Some("Widget"));
        assert_eq!(remote_fetch_target("MyState").as_deref(), Some("MyState"));
        // A 3-segment name is not the wire form the verb serves.
        assert_eq!(remote_fetch_target("pkg/msg/Type"), None);
        // Empty name / empty chunk → no candidate.
        assert_eq!(remote_fetch_target(""), None);
        assert_eq!(remote_fetch_target("/Type"), None);
        assert_eq!(remote_fetch_target("pkg/"), None);
    }

    /// `render_remote_schema` (unified tree): a network-resolved reply renders
    /// in the SAME shape as a local result — a header block naming the serving
    /// robot (`source: robot '...' over the network (not compiled locally; ...)`
    /// so the user is never misled into thinking the type is local) followed by
    /// the recursive field tree. A CUSTOM nested dep (`acme/Gadget`) AND a
    /// built-in nested reference (`std_msgs/Header`) BOTH expand inline (the
    /// built-in resolves against the local registry the renderer seeds). No
    /// `--- pkg/Type (encoding: msg) ---` sibling blocks (the old flat format).
    /// A served closure carrying the hostile
    /// composed shape (`Inner` ≈ 4 GiB — representable; `Outer = Inner[2^33]`
    /// — overflows only after inlining; `Wrapper` nests `Outer` as a FIXED
    /// target) would PANIC an unpreflighted renderer inside `build_resolution_context`
    /// (`resolve_fixed_nested` materializes `Outer` for `Wrapper`), leaving the
    /// verbatim fallback unreachable. Instead: `Wrapper` renders with its ref
    /// degraded (the loud unknown-type marker), a rejected ROOT takes the
    /// verbatim fallback, and the sane sibling still renders resolved.
    #[test]
    fn render_remote_schema_degrades_a_hostile_served_closure_instead_of_panicking() {
        let docs = || {
            vec![
                cerulion_core::SchemaDoc {
                    qualified: "hostile/Inner".to_string(),
                    encoding: cerulion_core::SchemaEncoding::Msg,
                    text: "float64[536870907] v\n".to_string(),
                    deps: vec![],
                },
                cerulion_core::SchemaDoc {
                    qualified: "hostile/Outer".to_string(),
                    encoding: cerulion_core::SchemaEncoding::Msg,
                    text: "Inner[8589934592] arr\n".to_string(),
                    deps: vec!["hostile/Inner".to_string()],
                },
                cerulion_core::SchemaDoc {
                    qualified: "hostile/Wrapper".to_string(),
                    encoding: cerulion_core::SchemaEncoding::Msg,
                    text: "Outer o\nfloat64 w\n".to_string(),
                    deps: vec!["hostile/Outer".to_string()],
                },
                cerulion_core::SchemaDoc {
                    qualified: "hostile/Sane".to_string(),
                    encoding: cerulion_core::SchemaEncoding::Msg,
                    text: "float64 x\nfloat64 y\n".to_string(),
                    deps: vec![],
                },
            ]
        };
        // Without the preflight: PANICS (schema.rs: FixedArray size overflows usize).
        let out = render_remote_schema(&cerulion_core::SchemaReply::found(
            "go2",
            "hostile/Wrapper",
            docs(),
        ));
        assert!(
            out.contains("schema: hostile/Wrapper"),
            "renders the root; got:\n{out}"
        );
        assert!(
            out.contains("unknown type"),
            "the ref to the preflight-dropped Outer degrades loudly; got:\n{out}"
        );
        // A REJECTED root is absent from the winner map → the verbatim fallback.
        let out = render_remote_schema(&cerulion_core::SchemaReply::found(
            "go2",
            "hostile/Outer",
            docs(),
        ));
        assert!(
            out.contains("showing it verbatim below") && out.contains("Inner[8589934592] arr"),
            "a rejected root falls back to the served text; got:\n{out}"
        );
        // The sane sibling resolves fully (16 bytes, two f64s).
        let out = render_remote_schema(&cerulion_core::SchemaReply::found(
            "go2",
            "hostile/Sane",
            docs(),
        ));
        assert!(
            out.contains("wire fixed size: 16 bytes"),
            "sane doc resolves; got:\n{out}"
        );
    }

    /// A remote reader
    /// (`build_remote_entry`) reading `layout_of` directly would let a served closure
    /// that crosses the wire ceiling only through fixed-nested inlining —
    /// the declared preflight's floor cannot see it — render a
    /// `wire fixed size` no frame can carry. It reads through
    /// `representable_layout`: the root takes the verbatim fallback, exactly
    /// as a local `schema info` on the same bytes refuses it. `WrapAt` (one
    /// byte under, via the same inlining) still renders its exact size.
    /// The removal report must never be EMPTY when a definition
    /// was removed. The shape: `B` (a built-in-like `(Some(pkg), Type)`),
    /// `F` (a store copy with the SAME key, later ⇒ the `by_key` winner,
    /// and the FAILING one), `Y` (a slash-named YAML twin `(None,
    /// "pkg/Type")`, later still ⇒ the STRING winner). `F` is neither the
    /// string winner (no winner-sweep) nor key-unique (a same-key
    /// survivor suppresses `removed_keys`), so a name-and-key report would be
    /// empty and both consumers would skip their scrub. The report names
    /// the removed DEFINITION itself: it removes `F`, and only `F`.
    #[test]
    fn r39_removal_report_names_a_removed_definition_even_when_its_name_and_key_survive() {
        let mut b = MessageSchema::new_in_package("Type", "pkg");
        b.add_field(FieldDef::new("x", FieldType::F64));
        let mut f = MessageSchema::new_in_package("Type", "pkg");
        f.add_field(FieldDef::new(
            "arr",
            FieldType::FixedArray {
                element_type: Box::new(FieldType::F64),
                length: 8,
            },
        ));
        let mut y = MessageSchema::new("pkg/Type");
        y.add_field(FieldDef::new("a", FieldType::U32));
        let (b0, f0, y0) = (b.clone(), f.clone(), y.clone());
        let mut schemas = vec![b, f, y];
        let failing: BTreeSet<usize> = [1usize].into_iter().collect();

        let identity = schemas.clone();
        let removed =
            remove_failing_definitions(&mut schemas, Some(&identity), &failing, "r39 unit");

        // Stated once and directly: `None` means "the
        // identity state IS `schemas`", so it must produce the report a
        // same-state `Some(&clone)` produces — without materializing the
        // clone a fixpoint would otherwise pay for on EVERY pass. This arm is the
        // only place the two encodings are compared head to head; the
        // `Some(&identity)` call above cannot distinguish them by itself,
        // because its clone is of the same state.
        {
            let mut via_none = vec![b0.clone(), f0.clone(), y0.clone()];
            let none_report =
                remove_failing_definitions(&mut via_none, None, &failing, "r39 unit (None)");
            assert_eq!(
                format!("{none_report:?}"),
                format!("{removed:?}"),
                "`None` and a same-state `Some(&clone)` must report identically"
            );
            assert_eq!(
                via_none.len(),
                2,
                "and remove identically — F is gone from the probe set either way"
            );
        }

        assert_eq!(schemas.len(), 2, "F was removed from the probe set");
        assert!(
            !removed.is_empty(),
            "a removal must be reported even when the name AND the key survive: {removed:?}"
        );
        assert!(removed.removes(&f0), "the removed definition is covered");
        assert!(
            !removed.removes(&b0),
            "the same-key built-in survivor is NOT swept"
        );
        assert!(
            !removed.removes(&y0),
            "the same-string YAML winner is NOT swept"
        );
    }

    /// `removes_identity` is the guaranteed-monotone fallback the
    /// three scrub fixpoints fall back on when a pass makes no progress, so
    /// the property it must have is that it matches on IDENTITY ALONE — the
    /// hash conjunct `removes` carries is exactly the part that failed (two
    /// states of one schema hashing differently), and a fallback that
    /// inherited it would guarantee nothing.
    ///
    /// Hand oracle, three definitions of one `(package, name)`: the removed
    /// one, a same-identity SIBLING whose fields differ (so its hash
    /// differs), and an unrelated schema. `removes` must take only the exact
    /// definition; `removes_identity` must take the sibling too — that
    /// over-scrub IS the documented cost, and asserting it is what stops a
    /// future "precise" fallback from silently reintroducing the hang.
    #[test]
    fn r41_removes_identity_matches_on_identity_alone_so_a_fallback_scrub_always_progresses() {
        let mut removed_def = MessageSchema::new_in_package("Type", "pkg");
        removed_def.add_field(FieldDef::new("x", FieldType::F64));
        let mut sibling = MessageSchema::new_in_package("Type", "pkg");
        sibling.add_field(FieldDef::new("y", FieldType::U32));
        let mut unrelated = MessageSchema::new_in_package("Other", "pkg");
        unrelated.add_field(FieldDef::new("z", FieldType::U32));
        assert_ne!(
            removed_def.schema_hash(),
            sibling.schema_hash(),
            "premise: same identity, different fields — so the hash arm can \
             tell them apart and the identity arm cannot"
        );

        let report = RemovedDefinitions {
            fully_removed_names: Vec::new(),
            removed_keys: Vec::new(),
            removed_definitions: vec![(
                removed_def.package.clone(),
                removed_def.name.clone(),
                removed_def.checked_schema_hash().ok(),
            )],
        };

        assert!(report.removes(&removed_def), "the exact definition");
        assert!(
            !report.removes(&sibling),
            "`removes` is hash-precise: a same-identity sibling is NOT the \
             definition that failed"
        );
        assert!(
            report.removes_identity(&removed_def),
            "the fallback takes the offender"
        );
        assert!(
            report.removes_identity(&sibling),
            "and its same-identity sibling — the documented cost that buys \
             guaranteed progress"
        );
        assert!(
            !report.removes_identity(&unrelated),
            "but nothing else: the fallback is coarser, never unbounded"
        );
    }

    #[test]
    fn render_remote_schema_refuses_a_root_composed_past_the_wire_ceiling() {
        const AT: usize = u32::MAX as usize - 32;
        let docs = || {
            vec![
                cerulion_core::SchemaDoc {
                    qualified: "hostile/Small".to_string(),
                    encoding: cerulion_core::SchemaEncoding::Msg,
                    text: "uint64 v\n".to_string(),
                    deps: vec![],
                },
                cerulion_core::SchemaDoc {
                    qualified: "hostile/WrapOver".to_string(),
                    encoding: cerulion_core::SchemaEncoding::Msg,
                    text: format!("uint8[{}] a\nSmall s\n", AT - 8),
                    deps: vec!["hostile/Small".to_string()],
                },
                cerulion_core::SchemaDoc {
                    qualified: "hostile/WrapAt".to_string(),
                    encoding: cerulion_core::SchemaEncoding::Msg,
                    text: format!("uint8[{}] a\nSmall s\n", AT - 16),
                    deps: vec!["hostile/Small".to_string()],
                },
            ]
        };
        let out = render_remote_schema(&cerulion_core::SchemaReply::found(
            "go2",
            "hostile/WrapOver",
            docs(),
        ));
        assert!(
            out.contains("showing it verbatim below"),
            "a root composed past the ceiling takes the verbatim fallback; got:\n{out}"
        );
        assert!(
            !out.contains("wire fixed size: 4294967264 bytes"),
            "never a size no frame can carry; got:\n{out}"
        );
        let out = render_remote_schema(&cerulion_core::SchemaReply::found(
            "go2",
            "hostile/WrapAt",
            docs(),
        ));
        assert!(
            out.contains("wire fixed size: 4294967256 bytes"),
            "one byte under (via the same inlining) renders its exact size; got:\n{out}"
        );
    }

    #[test]
    fn render_remote_schema_renders_unified_tree_with_provenance() {
        // The requested-root `.msg` text — reused as the independent oracle.
        const WIDGET_MSG: &str = "std_msgs/Header header\nacme/Gadget gadget\nfloat64 x\n";
        let reply = cerulion_core::SchemaReply::found(
            "go2",
            "acme/Widget",
            vec![
                cerulion_core::SchemaDoc {
                    qualified: "acme/Widget".to_string(),
                    encoding: cerulion_core::SchemaEncoding::Msg,
                    text: WIDGET_MSG.to_string(),
                    deps: vec!["acme/Gadget".to_string()],
                },
                cerulion_core::SchemaDoc {
                    qualified: "acme/Gadget".to_string(),
                    encoding: cerulion_core::SchemaEncoding::Msg,
                    // Variable (has a `string`), so the root's RESOLVED hash +
                    // wire fixed size equal the UNRESOLVED single-file parse
                    // (no fixed-nested folding) — making an independent
                    // `parse_rosmsg` oracle exact.
                    text: "int32 n\nstring label\n".to_string(),
                    deps: vec![],
                },
            ],
        );
        let out = render_remote_schema(&reply);

        // Header block: unified `schema:` + the network `source:` provenance,
        // plus the locally-computed wire identity — anchored to an INDEPENDENT
        // `parse_rosmsg` oracle over the fabricated closure (not presence-only).
        let widget_oracle = parse_rosmsg(WIDGET_MSG, "Widget", Some("acme")).unwrap();
        assert!(out.starts_with("schema: acme/Widget\n"), "GOT:\n{out}");
        assert!(
            out.contains(
                "source: robot 'go2' over the network \
                 (not compiled locally; 2 doc(s) in its closure)\n"
            ),
            "provenance line:\n{out}"
        );
        assert!(
            out.contains(&format!("\nhash: 0x{:016x}\n", widget_oracle.schema_hash())),
            "hash must equal the independent parse oracle:\n{out}"
        );
        assert!(
            out.contains(&format!(
                "\nwire fixed size: {} bytes\n",
                widget_oracle.wire_fixed_size()
            )),
            "wire fixed size must equal the independent parse oracle:\n{out}"
        );
        assert!(out.contains("\nfields: 3\n"), "field count:\n{out}");

        // The built-in nested `std_msgs/Header` expands (Time → sec/nanosec).
        assert!(
            out.contains(
                "  header: std_msgs/Header (variable)\n    \
                 stamp: builtin_interfaces/Time (fixed)\n      sec: int32 (fixed)\n"
            ),
            "built-in nested must expand inline:\n{out}"
        );
        // The CUSTOM nested dep `acme/Gadget` expands too.
        assert!(
            out.contains(
                "  gadget: acme/Gadget (variable)\n    n: int32 (fixed)\n    \
                 label: string (variable)\n"
            ),
            "custom nested dep must expand inline:\n{out}"
        );
        assert!(out.contains("  x: float64 (fixed)\n"), "GOT:\n{out}");

        // The OLD flat verbatim format is gone.
        assert!(
            !out.contains("--- acme/Widget (encoding: msg) ---"),
            "the flat `--- ... ---` blocks must be replaced by the tree:\n{out}"
        );
        assert!(
            !out.contains("OVER THE NETWORK"),
            "old wording gone:\n{out}"
        );
    }

    /// A YAML-encoded, PACKAGE-LESS served type (`schema create <Name>` served
    /// under its bare key) renders as the unified tree too — one renderer, both
    /// wire encodings, packaged or bare.
    #[test]
    fn render_remote_schema_handles_yaml_encoded_bare_type() {
        let reply = cerulion_core::SchemaReply::found(
            "ubuntu",
            "MyState",
            vec![cerulion_core::SchemaDoc {
                qualified: "MyState".to_string(),
                encoding: cerulion_core::SchemaEncoding::Yaml,
                text: "schemas:\n  MyState:\n    fields:\n      uint32 seq:\n      string label:\n"
                    .to_string(),
                deps: vec![],
            }],
        );
        let out = render_remote_schema(&reply);
        assert!(out.starts_with("schema: MyState\n"), "GOT:\n{out}");
        assert!(
            out.contains("source: robot 'ubuntu' over the network"),
            "GOT:\n{out}"
        );
        assert!(out.contains("  seq: uint32 (fixed)\n"), "GOT:\n{out}");
        assert!(out.contains("  label: string (variable)\n"), "GOT:\n{out}");
    }

    /// Robustness: if the requested ROOT doc cannot be parsed into IR, the
    /// renderer keeps the same provenance header but falls back to the VERBATIM
    /// served text — never losing the user's data (loud-over-silent).
    #[test]
    fn render_remote_schema_falls_back_to_verbatim_on_unparseable_root() {
        let reply = cerulion_core::SchemaReply::found(
            "go2",
            "acme/Broken",
            vec![cerulion_core::SchemaDoc {
                qualified: "acme/Broken".to_string(),
                encoding: cerulion_core::SchemaEncoding::Msg,
                // An invalid array length — `FieldType::parse` (and thus
                // `parse_rosmsg`) rejects it, so the root has no IR/layout.
                text: "float64[notanumber] data\n".to_string(),
                deps: vec![],
            }],
        );
        let out = render_remote_schema(&reply);
        assert!(out.starts_with("schema: acme/Broken\n"), "GOT:\n{out}");
        assert!(
            out.contains("source: robot 'go2' over the network"),
            "GOT:\n{out}"
        );
        assert!(
            out.contains("showing it verbatim"),
            "must announce the verbatim fallback:\n{out}"
        );
        assert!(
            out.contains("--- acme/Broken (encoding: msg) ---"),
            "verbatim block must be present:\n{out}"
        );
        assert!(
            out.contains("float64[notanumber] data"),
            "the served text must survive verbatim:\n{out}"
        );
    }

    /// Resolution-ladder pin: workspace WINS over built-in for a name defined in
    /// BOTH (`schema_info_unified` returns the workspace source); a bare unknown
    /// name errors `SchemaNotFound` (the point where the CLI's remote tier kicks
    /// in) and is NOT a remote candidate; a qualified built-in resolves LOCALLY
    /// (no remote attempt). This pins the local half of the ladder + the exact
    /// boundary where remote takes over.
    #[test]
    fn schema_info_local_ladder_before_remote() {
        let tmp = tempfile::tempdir().unwrap();
        // A workspace YAML schema named `Image` shadows the built-in
        // `sensor_msgs/Image`.
        std::fs::write(
            tmp.path().join("image.yaml"),
            "schemas:\n  Image:\n    fields:\n      \"uint32 w\":\n      \"uint32 h\":\n",
        )
        .unwrap();
        // Workspace wins for the bare `Image`.
        let ws = schema_info_unified(tmp.path(), "Image").unwrap();
        assert!(matches!(ws.source, SchemaSource::Workspace));
        // A qualified BUILT-IN resolves locally (source Builtin) — never remote.
        let bi = schema_info_unified(tmp.path(), "geometry_msgs/Vector3").unwrap();
        assert!(matches!(bi.source, SchemaSource::Builtin));
        assert!(remote_fetch_target("geometry_msgs/Vector3").is_some()); // is a *shape* candidate,
                                                                         // but the CLI only reaches remote AFTER a local SchemaNotFound — and this
                                                                         // one resolved locally, so remote is never attempted.
                                                                         // An unknown QUALIFIED name errors SchemaNotFound (the remote-tier boundary).
        let err = schema_info_unified(tmp.path(), "vendor_msgs/Nope").unwrap_err();
        assert!(matches!(err, CliError::SchemaNotFound { .. }));
        assert_eq!(
            remote_fetch_target("vendor_msgs/Nope").as_deref(),
            Some("vendor_msgs/Nope")
        );
    }

    /// `schema info` runs WITHOUT a workspace (brand-new
    /// laptop resolving a robot's types with zero setup). `None` schemas_dir ⇒
    /// the workspace + `.msg`-store tiers are SKIPPED and a builtin resolves
    /// LOCALLY; an unknown name errors `SchemaNotFound` — the EXACT boundary
    /// where the CLI's remote tier takes over (and the name IS a remote-fetch
    /// candidate). `Some(dir)` still lets the workspace tier WIN.
    #[test]
    fn schema_info_unified_opt_is_workspace_optional() {
        // No workspace: a built-in resolves LOCALLY (source Builtin).
        let bi = schema_info_unified_opt(None, "sensor_msgs/Image").unwrap();
        assert!(matches!(bi.source, SchemaSource::Builtin));
        assert_eq!(bi.result.entries[0].name, "sensor_msgs/Image");

        // No workspace + an unknown qualified name ⇒ SchemaNotFound (the remote
        // boundary), AND the name is a remote-fetch candidate (the CLI reaches
        // its remote tier exactly here — hermetic: no session opened in the
        // engine, the shape check is pure).
        let err = schema_info_unified_opt(None, "vendor_msgs/Custom").unwrap_err();
        assert!(matches!(err, CliError::SchemaNotFound { .. }));
        assert_eq!(
            remote_fetch_target("vendor_msgs/Custom").as_deref(),
            Some("vendor_msgs/Custom")
        );

        // A bare unknown name (no workspace) ⇒ an Err — the workspace tier is
        // genuinely skipped (a `Some(dir)` with that entry would win). A bare,
        // non-`pkg/Type` name is a `Validation` error from the built-in tier
        // (not a qualified name), distinct from the qualified `SchemaNotFound`
        // above — both mean "not resolved locally".
        assert!(matches!(
            schema_info_unified_opt(None, "MyState").unwrap_err(),
            CliError::Validation(_)
        ));

        // WITH a workspace, the workspace tier still WINS for a colliding name.
        let tmp = tempfile::tempdir().unwrap();
        std::fs::write(
            tmp.path().join("image.yaml"),
            "schemas:\n  Image:\n    fields:\n      \"uint32 w\":\n      \"uint32 h\":\n",
        )
        .unwrap();
        let ws = schema_info_unified_opt(Some(tmp.path()), "Image").unwrap();
        assert!(matches!(ws.source, SchemaSource::Workspace));
    }

    /// `schema list` is workspace-optional too — `None`
    /// yields the built-in registry alone (empty workspace + store), never
    /// reading a stray non-workspace `schemas/` from the cwd.
    #[test]
    fn schema_list_opt_none_lists_builtins_only() {
        let none = schema_list_opt(None);
        assert!(none.workspace.is_empty());
        assert!(none.store.is_empty());
        assert_eq!(none.builtin_total, native_ros2_messages::BUILTIN_MSGS.len());
        assert!(!none.builtin_packages.is_empty());
        // No workspace shadows anything (empty workspace + store).
        assert!(none
            .builtin_packages
            .iter()
            .flat_map(|g| &g.types)
            .all(|t| t.shadowed_by.is_none() && t.shadowed_by_store.is_none()));
    }

    #[test]
    fn test_schema_create() {
        let tmp = tempfile::tempdir().unwrap();
        schema_create(tmp.path(), "laser_scan").unwrap();

        let path = tmp.path().join("laser_scan.yaml");
        assert!(path.exists());

        let content = std::fs::read_to_string(path).unwrap();
        assert!(content.contains("LaserScan"));
        assert!(content.contains("schemas:"));
    }

    #[test]
    fn test_schema_create_already_exists() {
        let tmp = tempfile::tempdir().unwrap();
        schema_create(tmp.path(), "test").unwrap();
        let result = schema_create(tmp.path(), "test");
        assert!(result.is_err());
        assert!(result.unwrap_err().to_string().contains("already exists"));
    }

    #[test]
    fn test_schema_delete() {
        let tmp = tempfile::tempdir().unwrap();
        schema_create(tmp.path(), "test").unwrap();
        assert!(tmp.path().join("test.yaml").exists());

        schema_delete(tmp.path(), "test").unwrap();
        assert!(!tmp.path().join("test.yaml").exists());
    }

    #[test]
    fn test_schema_delete_not_found() {
        let tmp = tempfile::tempdir().unwrap();
        let result = schema_delete(tmp.path(), "nonexistent");
        assert!(result.is_err());
        let msg = result.unwrap_err().to_string();
        assert!(msg.contains("not found"), "{msg}");
        // Delete-context remedy: must NOT suggest
        // built-in lookup — built-ins cannot be deleted — and points at
        // the discovery command.
        assert!(
            msg.contains("only workspace schemas can be deleted"),
            "delete remedy must state the workspace-only contract: {msg}"
        );
        assert!(
            msg.contains("cerulion schema list"),
            "delete remedy must name the discovery command: {msg}"
        );
        assert!(
            !msg.contains("qualified 'pkg/Type'"),
            "delete remedy must not carry the info-context built-in guidance: {msg}"
        );
    }

    #[test]
    fn test_schema_info() {
        let tmp = tempfile::tempdir().unwrap();
        // Write a schema with actual fields
        let content = r#"schemas:
  Image:
    description: "Camera image"
    fields:
      uint32 height:
      uint32 width:
      uint8[] data:
"#;
        std::fs::write(tmp.path().join("image.yaml"), content).unwrap();

        let info = schema_info(tmp.path(), "image").unwrap();
        assert_eq!(info.entries.len(), 1);
        assert_eq!(info.entries[0].name, "Image");
        assert_eq!(info.entries[0].description, "Camera image");
        assert_eq!(info.entries[0].field_count, 3);
        assert!(info.entries[0].schema_hash.is_some_and(|h| h != 0));
    }

    #[test]
    fn test_schema_info_not_found() {
        let tmp = tempfile::tempdir().unwrap();
        let result = schema_info(tmp.path(), "nonexistent");
        assert!(result.is_err());
    }

    /// Parser hardening (b): field keys without exactly 2 whitespace
    /// tokens are rejected loudly, naming the key and the expected form.
    #[test]
    fn test_schema_info_rejects_one_token_field_key() {
        let tmp = tempfile::tempdir().unwrap();
        let content = "schemas:\n  Broken:\n    fields:\n      height:\n";
        std::fs::write(tmp.path().join("broken.yaml"), content).unwrap();

        let err = schema_info(tmp.path(), "broken").unwrap_err();
        let msg = err.to_string();
        assert!(msg.contains("height"), "must name the offending key: {msg}");
        assert!(
            msg.contains("<type> <name>"),
            "must state the expected form: {msg}"
        );
        assert!(
            msg.contains("1 token"),
            "must report the token count: {msg}"
        );
    }

    /// Parser hardening (b): three-token keys are rejected too — strict
    /// `<type> <name>`, nothing else.
    #[test]
    fn test_schema_info_rejects_three_token_field_key() {
        let tmp = tempfile::tempdir().unwrap();
        let content = "schemas:\n  Broken:\n    fields:\n      uint32 height extra:\n";
        std::fs::write(tmp.path().join("broken.yaml"), content).unwrap();

        let err = schema_info(tmp.path(), "broken").unwrap_err();
        let msg = err.to_string();
        assert!(
            msg.contains("uint32 height extra"),
            "must name the offending key: {msg}"
        );
        assert!(
            msg.contains("<type> <name>"),
            "must state the expected form: {msg}"
        );
    }

    /// Parser hardening (b): an unknown type identifier (including a
    /// typo'd primitive like `unit32`) is KEPT as a nested schema
    /// reference (`is_nested == true`) with a CLEAN canonical `type_display`.
    /// The loud "not a primitive" signal now lives in the unified renderer,
    /// which flags an unresolvable reference on its field line as
    /// `(… unknown type — no schema found)` — pinned by
    /// `schema_builtin_test::unified_unknown_nested_ref_is_flagged_loudly_on_the_field_line`.
    #[test]
    fn test_schema_info_marks_nested_types_loudly() {
        let tmp = tempfile::tempdir().unwrap();
        let content = "schemas:\n  Pose:\n    fields:\n      Position3D position:\n      unit32 typo_field:\n      uint32 real_field:\n";
        std::fs::write(tmp.path().join("pose.yaml"), content).unwrap();

        let info = schema_info(tmp.path(), "pose").unwrap();
        let fields = &info.entries[0].fields;
        assert_eq!(fields.len(), 3);

        // Nested reference: flagged nested, clean canonical string.
        assert!(fields[0].is_nested);
        assert_eq!(fields[0].type_display, "Position3D");
        // The typo'd primitive parses as Nested (so a later render flags it),
        // NOT as a silent primitive.
        assert!(fields[1].is_nested);
        assert_eq!(fields[1].type_display, "unit32");
        // A real primitive is not nested.
        assert!(!fields[2].is_nested);
        assert_eq!(fields[2].type_display, "uint32");
    }

    /// Hardening: an ARRAY of a nested reference (`Header[]` or
    /// `Header[3]`) must ALSO be flagged nested (`is_nested == true`) so the
    /// unified renderer expands its element type / flags it if unresolvable —
    /// `is_nested = matches!(.., Nested { .. })` (top-level only) would miss
    /// `DynamicArray`/`FixedArray` of `Nested` (including a typo like
    /// `Heaer[]`). `type_display` is the clean canonical array string.
    #[test]
    fn test_schema_info_marks_array_of_nested_loudly() {
        let tmp = tempfile::tempdir().unwrap();
        let content = "schemas:\n  Scan:\n    fields:\n      Header[] dyn_headers:\n      \
                       Header[3] fixed_headers:\n      Heaer[] typo_headers:\n      \
                       uint32[] real_array:\n";
        std::fs::write(tmp.path().join("scan.yaml"), content).unwrap();

        let info = schema_info(tmp.path(), "scan").unwrap();
        let fields = &info.entries[0].fields;
        assert_eq!(fields.len(), 4);

        // DynamicArray-of-nested: flagged, clean canonical string.
        assert!(fields[0].is_nested, "Header[] must be flagged nested");
        assert_eq!(fields[0].type_display, "Header[]");
        // FixedArray-of-nested: flagged.
        assert!(fields[1].is_nested, "Header[3] must be flagged nested");
        assert_eq!(fields[1].type_display, "Header[3]");
        // Typo'd element type parses as array-of-nested: flagged (so a render
        // marks it as an unknown type, not a silent primitive array).
        assert!(fields[2].is_nested, "Heaer[] typo must be flagged nested");
        assert_eq!(fields[2].type_display, "Heaer[]");
        // A real primitive array is not nested.
        assert!(!fields[3].is_nested, "uint32[] is not nested");
        assert_eq!(fields[3].type_display, "uint32[]");
    }

    /// Parser hardening (c): FixedArray lengths above MAX_FIXED_ARRAY_LEN
    /// are rejected with a CliError so the CLI never reaches the
    /// codegen-time `wire_fixed_size()` overflow panic.
    #[test]
    fn test_schema_info_rejects_oversized_fixed_array() {
        let tmp = tempfile::tempdir().unwrap();
        let content = "schemas:\n  Huge:\n    fields:\n      float64[1048577] data:\n";
        std::fs::write(tmp.path().join("huge.yaml"), content).unwrap();

        let err = schema_info(tmp.path(), "huge").unwrap_err();
        let msg = err.to_string();
        assert!(msg.contains("1048577"), "must name the length: {msg}");
        assert!(
            msg.contains("1048576"),
            "must state the maximum supported length: {msg}"
        );
    }

    /// Boundary: a FixedArray exactly at MAX_FIXED_ARRAY_LEN is accepted.
    #[test]
    fn test_schema_info_accepts_fixed_array_at_limit() {
        let tmp = tempfile::tempdir().unwrap();
        let content = "schemas:\n  Big:\n    fields:\n      uint8[1048576] data:\n";
        std::fs::write(tmp.path().join("big.yaml"), content).unwrap();

        let info = schema_info(tmp.path(), "big").unwrap();
        assert_eq!(info.entries[0].field_count, 1);
    }

    /// Parser hardening: a `StringFixed` with an
    /// absurd inline length is rejected with a `CliError`, not silently
    /// accepted. A `_ => Ok(())` fallthrough would let
    /// `StringFixed(usize::MAX)` reach `wire_fixed_size()`, where
    /// `offset.checked_add(usize::MAX)` panics instead of surfacing a
    /// user-friendly error.
    #[test]
    fn test_schema_info_rejects_oversized_string_fixed() {
        let tmp = tempfile::tempdir().unwrap();
        let content =
            "schemas:\n  Huge:\n    fields:\n      string_fixed[18446744073709551615] data:\n";
        std::fs::write(tmp.path().join("huge.yaml"), content).unwrap();

        let err = schema_info(tmp.path(), "huge").unwrap_err();
        let msg = err.to_string();
        assert!(
            msg.contains("18446744073709551615"),
            "must name the length: {msg}"
        );
        assert!(
            msg.contains("1048576"),
            "must state the maximum supported length: {msg}"
        );
        assert!(
            msg.contains("StringFixed"),
            "must name the offending variant: {msg}"
        );
    }

    /// Boundary: a `StringFixed` exactly at MAX_FIXED_ARRAY_LEN is accepted.
    #[test]
    fn test_schema_info_accepts_string_fixed_at_limit() {
        let tmp = tempfile::tempdir().unwrap();
        let content = "schemas:\n  Big:\n    fields:\n      string_fixed[1048576] data:\n";
        std::fs::write(tmp.path().join("big.yaml"), content).unwrap();

        let info = schema_info(tmp.path(), "big").unwrap();
        assert_eq!(info.entries[0].field_count, 1);
    }

    /// The displayed hash is layout-sensitive — same schema
    /// name, different fields → different hashes.
    #[test]
    fn test_schema_info_hash_is_layout_sensitive() {
        let tmp = tempfile::tempdir().unwrap();
        std::fs::write(
            tmp.path().join("v1.yaml"),
            "schemas:\n  Image:\n    fields:\n      uint32 width:\n",
        )
        .unwrap();
        std::fs::write(
            tmp.path().join("v2.yaml"),
            "schemas:\n  Image:\n    fields:\n      uint32 width:\n      uint32 height:\n",
        )
        .unwrap();

        let h1 = schema_info(tmp.path(), "v1").unwrap().entries[0].schema_hash;
        let h2 = schema_info(tmp.path(), "v2").unwrap().entries[0].schema_hash;
        assert!(
            h1.is_some() && h2.is_some(),
            "both representable entries hash"
        );
        assert_ne!(
            h1, h2,
            "same-name schemas with different layouts must display different hashes"
        );
    }

    /// A NON-STRING schema-name key (a YAML int here)
    /// must be REJECTED loudly by `parse_message_schemas`, not silently
    /// coerced to "unknown" (which would collide other coerced names under
    /// one bucket and never match a real `schema:`). The caller downgrades
    /// this to a warn + skip, so it stays non-fatal but visible.
    #[test]
    fn test_parse_message_schemas_rejects_non_string_schema_name() {
        // `123:` is an integer YAML key, not a string.
        let content = "schemas:\n  123:\n    fields:\n      uint32 a:\n";
        let err = parse_message_schemas(content).unwrap_err();
        let msg = err.to_string();
        assert!(
            msg.contains("schema name") && msg.contains("not a string"),
            "must reject a non-string schema name loudly: {msg}"
        );
    }

    #[test]
    fn test_parse_message_schemas_keeps_malformed_yaml_as_yaml_error() {
        let err = parse_message_schemas("schemas: [unclosed").expect_err("malformed YAML");
        assert!(matches!(err, CliError::Yaml(_)));
        assert!(err.to_string().starts_with("YAML error:"));
    }

    #[test]
    fn test_parse_message_schemas_rejects_scalar_schema_definition() {
        let err = parse_message_schemas("schemas:\n  Foo: 3\n").expect_err("scalar schema");
        assert!(
            matches!(err, CliError::Validation(message) if message.contains("must be a mapping"))
        );
    }

    #[test]
    fn test_parse_message_schemas_rejects_invalid_fields_shape() {
        let err = parse_message_schemas("schemas:\n  Foo:\n    fields: bad\n")
            .expect_err("scalar fields");
        assert!(matches!(err, CliError::Validation(message) if message.contains("fields")));
    }

    /// A well-formed workspace schema parses to a single `MessageSchema`
    /// keyed by its bare (package-less) qualified name — the shape
    /// `build_workspace_schema_hashes` relies on.
    #[test]
    fn test_parse_message_schemas_happy_path() {
        let content = "schemas:\n  Foo:\n    fields:\n      uint32 a:\n      float64 b:\n";
        let schemas = parse_message_schemas(content).unwrap();
        assert_eq!(schemas.len(), 1);
        assert_eq!(schemas[0].qualified_name(), "Foo");
        assert_eq!(schemas[0].fields.len(), 2);
    }

    // ---- resolve_port_schema (bare-name port-schema resolver) ----

    /// A unique bare built-in name (`Vector3` lives ONLY in
    /// geometry_msgs) with no workspace schema present resolves to that
    /// one qualified built-in.
    #[test]
    fn test_resolve_port_schema_unique_bare_builtin() {
        let tmp = tempfile::tempdir().unwrap();
        let resolved = resolve_port_schema(tmp.path(), "Vector3").unwrap();
        assert_eq!(resolved.schema, "geometry_msgs/Vector3");
        assert_eq!(
            resolved.provenance,
            PortSchemaProvenance::ResolvedBuiltin {
                qualified: "geometry_msgs/Vector3".to_string()
            }
        );
    }

    /// A name that is already qualified — in EITHER separator form —
    /// passes through as `Qualified`, normalized to the `/` form.
    #[test]
    fn test_resolve_port_schema_qualified_passthrough() {
        let tmp = tempfile::tempdir().unwrap();
        for raw in ["geometry_msgs/Vector3", "geometry_msgs::Vector3"] {
            let resolved = resolve_port_schema(tmp.path(), raw).unwrap();
            assert_eq!(resolved.schema, "geometry_msgs/Vector3", "raw={raw}");
            assert_eq!(
                resolved.provenance,
                PortSchemaProvenance::Qualified,
                "raw={raw}"
            );
        }
    }

    /// Idempotence: feeding a prior resolver output (a qualified
    /// `pkg/Name`) back in is a no-op — it lands on the `Qualified` arm,
    /// schema unchanged.
    #[test]
    fn test_resolve_port_schema_is_idempotent() {
        let tmp = tempfile::tempdir().unwrap();
        let first = resolve_port_schema(tmp.path(), "Vector3").unwrap();
        assert_eq!(first.schema, "geometry_msgs/Vector3");
        let second = resolve_port_schema(tmp.path(), &first.schema).unwrap();
        assert_eq!(second.schema, "geometry_msgs/Vector3");
        assert_eq!(second.provenance, PortSchemaProvenance::Qualified);
    }

    /// A bare name present in MORE THAN ONE built-in package (`Pose2D`
    /// lives in both geometry_msgs and vision_msgs) is a loud
    /// `Validation` error naming BOTH candidates, package-ascending
    /// (geometry_msgs before vision_msgs).
    #[test]
    fn test_resolve_port_schema_ambiguous_names_both_packages() {
        let tmp = tempfile::tempdir().unwrap();
        let err = resolve_port_schema(tmp.path(), "Pose2D").unwrap_err();
        assert!(
            matches!(err, CliError::Validation(_)),
            "expected Validation, got {err:?}"
        );
        let msg = err.to_string();
        assert!(msg.contains("ambiguous"), "must state ambiguity: {msg}");
        let g = msg
            .find("geometry_msgs/Pose2D")
            .expect("must name geometry_msgs/Pose2D");
        let v = msg
            .find("vision_msgs/Pose2D")
            .expect("must name vision_msgs/Pose2D");
        assert!(g < v, "candidates must be listed package-ascending: {msg}");
        // The example-qualification hint points at the first (ascending) candidate.
        assert!(
            msg.contains("e.g. 'geometry_msgs/Pose2D'"),
            "must suggest qualifying with the first candidate: {msg}"
        );
    }

    /// A bare name matching no built-in — and the empty/garbage inputs
    /// that are bare with no `/` — all land on the 0-candidate
    /// `SchemaNotFound` arm carrying the port-context remedy.
    #[test]
    fn test_resolve_port_schema_unknown_carries_remedy() {
        let tmp = tempfile::tempdir().unwrap();
        for raw in ["Vectorr3", "", "!!!"] {
            let err = resolve_port_schema(tmp.path(), raw).unwrap_err();
            match err {
                CliError::SchemaNotFound { remedy, .. } => {
                    assert_eq!(
                        remedy, PORT_SCHEMA_REMEDY,
                        "raw={raw:?} must carry the port-context remedy"
                    );
                }
                other => panic!("expected SchemaNotFound for {raw:?}, got {other:?}"),
            }
        }
        // Hand-pasted oracle substrings — the remedy names both sources
        // and the qualification escape hatch.
        let msg = resolve_port_schema(tmp.path(), "Vectorr3")
            .unwrap_err()
            .to_string();
        assert!(
            msg.contains("Run 'cerulion schema list'"),
            "remedy must name the discovery command: {msg}"
        );
        assert!(
            msg.contains("qualify as 'pkg/Type'"),
            "remedy must name the qualification escape hatch: {msg}"
        );
    }

    /// A workspace schema matched by FILE STEM wins; the bare name stays
    /// bare and, with no same-named built-in, shadows nothing.
    #[test]
    fn test_resolve_port_schema_workspace_wins_no_shadow() {
        let tmp = tempfile::tempdir().unwrap();
        std::fs::write(
            tmp.path().join("Foo.yaml"),
            "schemas:\n  Foo:\n    fields:\n      uint32 a:\n",
        )
        .unwrap();
        let resolved = resolve_port_schema(tmp.path(), "Foo").unwrap();
        assert_eq!(resolved.schema, "Foo");
        assert_eq!(
            resolved.provenance,
            PortSchemaProvenance::Workspace { shadowed: None }
        );
    }

    /// A workspace schema named `Vector3` WINS over the built-in
    /// geometry_msgs/Vector3: the bare name is kept unchanged (NOT
    /// rewritten to the built-in path) and the shadow is surfaced.
    #[test]
    fn test_resolve_port_schema_workspace_shadows_builtin() {
        let tmp = tempfile::tempdir().unwrap();
        std::fs::write(
            tmp.path().join("Vector3.yaml"),
            "schemas:\n  Vector3:\n    fields:\n      float64 x:\n",
        )
        .unwrap();
        let resolved = resolve_port_schema(tmp.path(), "Vector3").unwrap();
        assert_eq!(resolved.schema, "Vector3");
        match resolved.provenance {
            PortSchemaProvenance::Workspace { shadowed: Some(s) } => {
                assert!(
                    s.contains("geometry_msgs/Vector3"),
                    "shadow must name the shadowed built-in: {s}"
                );
            }
            other => panic!("expected Workspace {{ shadowed: Some(..) }}, got {other:?}"),
        }
    }

    // ---- msg-store parity, surface 4: the `.msg` store tier ----

    /// Write one `.msg` store file under `<schemas_dir>/<pkg>/msg/`.
    fn write_store_msg_s4(schemas_dir: &Path, pkg: &str, type_name: &str, text: &str) {
        let dir = schemas_dir.join(pkg).join("msg");
        std::fs::create_dir_all(&dir).unwrap();
        std::fs::write(dir.join(format!("{type_name}.msg")), text).unwrap();
    }

    /// THE resolver parity pin (a resolver consulting no store tier at all
    /// fails it with `SchemaNotFound`): a bare name uniquely in
    /// the `.msg` store resolves to its package-qualified form.
    #[test]
    fn test_resolve_port_schema_bare_store_name_resolves_qualified() {
        let tmp = tempfile::tempdir().unwrap();
        write_store_msg_s4(tmp.path(), "acme", "WidgetState", "uint32 a\n");
        let resolved = resolve_port_schema(tmp.path(), "WidgetState").unwrap();
        assert_eq!(resolved.schema, "acme/WidgetState");
        assert_eq!(
            resolved.provenance,
            PortSchemaProvenance::ResolvedStore {
                qualified: "acme/WidgetState".to_string(),
                shadowed: None,
            }
        );
    }

    /// A shared bare name is not WON by a tier, it is REFUSED: a workspace
    /// YAML definition the `.msg` store also spells is the twin, and no
    /// precedence picks between them (ambiguity-refusal rule, applied by
    /// the one workspace lookup). The control below is the same fixture
    /// without the twin, where the workspace definition binds and stays bare.
    #[test]
    fn test_resolve_port_schema_refuses_a_bare_name_both_tiers_define() {
        let tmp = tempfile::tempdir().unwrap();
        std::fs::write(
            tmp.path().join("Widget.yaml"),
            "schemas:\n  Widget:\n    fields:\n      uint32 a:\n",
        )
        .unwrap();
        write_store_msg_s4(tmp.path(), "acme", "Widget", "float64 x\n");
        let err = resolve_port_schema(tmp.path(), "Widget")
            .expect_err("both tiers define it")
            .to_string();
        assert!(
            err.contains("'Widget' is ambiguous in this workspace — defined by:")
                && err.contains("schemas/Widget.yaml (entry Widget)")
                && err.contains("schemas/acme/msg/Widget.msg (store)"),
            "got: {err}"
        );

        // CONTROL: without the store twin the workspace definition binds and
        // the name stays bare.
        let ctl = tempfile::tempdir().unwrap();
        std::fs::write(
            ctl.path().join("Widget.yaml"),
            "schemas:\n  Widget:\n    fields:\n      uint32 a:\n",
        )
        .unwrap();
        let resolved = resolve_port_schema(ctl.path(), "Widget").unwrap();
        assert_eq!(resolved.schema, "Widget", "stays bare");
        assert_eq!(
            resolved.provenance,
            PortSchemaProvenance::Workspace { shadowed: None }
        );
    }

    /// The store tier OUTRANKS built-ins (a built-ins-first resolver fails this with `ResolvedBuiltin`),
    /// and the outranked built-in is surfaced as a shadow for the caller's
    /// shadow WARNING.
    #[test]
    fn test_resolve_port_schema_store_wins_over_builtin_with_shadow() {
        let tmp = tempfile::tempdir().unwrap();
        write_store_msg_s4(tmp.path(), "acme", "Vector3", "float64 x\nfloat64 y\n");
        let resolved = resolve_port_schema(tmp.path(), "Vector3").unwrap();
        assert_eq!(resolved.schema, "acme/Vector3");
        match resolved.provenance {
            PortSchemaProvenance::ResolvedStore {
                qualified,
                shadowed: Some(s),
            } => {
                assert_eq!(qualified, "acme/Vector3");
                assert!(
                    s.contains("geometry_msgs/Vector3"),
                    "shadow must name the outranked built-in: {s}"
                );
            }
            other => panic!("expected ResolvedStore {{ shadowed: Some(..) }}, got {other:?}"),
        }
    }

    /// A bare name defined by TWO store packages errors LOUDLY, listing
    /// both candidates — never a silent pick, never a fall-through to the
    /// built-in tier.
    #[test]
    fn test_resolve_port_schema_ambiguous_store_bare_name_errors() {
        let tmp = tempfile::tempdir().unwrap();
        write_store_msg_s4(tmp.path(), "pkga", "Goal", "float64 x\n");
        write_store_msg_s4(tmp.path(), "pkgb", "Goal", "uint32 a\n");
        let err = resolve_port_schema(tmp.path(), "Goal").unwrap_err();
        let msg = err.to_string();
        assert!(
            msg.contains("pkga/Goal") && msg.contains("pkgb/Goal") && msg.contains("ambiguous"),
            "ambiguity error must list both store candidates: {msg}"
        );
    }

    /// The Qualified arm VALIDATES against the store: a `pkg/Type` (either
    /// spelling) naming a store entry reports `QualifiedStore` with the
    /// string unchanged, while a fully-unknown qualified name keeps the
    /// deliberate blind pass-through (`Qualified`) — existence gating is
    /// `port_schema_exists`'s job.
    #[test]
    fn test_resolve_port_schema_qualified_store_name_is_validated() {
        let tmp = tempfile::tempdir().unwrap();
        write_store_msg_s4(tmp.path(), "acme", "WidgetState", "uint32 a\n");
        for spelling in ["acme/WidgetState", "acme::WidgetState"] {
            let resolved = resolve_port_schema(tmp.path(), spelling).unwrap();
            assert_eq!(resolved.schema, "acme/WidgetState");
            assert_eq!(
                resolved.provenance,
                PortSchemaProvenance::QualifiedStore,
                "'{spelling}' names a store entry — validated, not blind"
            );
        }
        let unknown = resolve_port_schema(tmp.path(), "made/up").unwrap();
        assert_eq!(unknown.schema, "made/up");
        assert_eq!(unknown.provenance, PortSchemaProvenance::Qualified);
    }

    /// Idempotence across the store tier: feeding a bare-store resolution's
    /// output back in returns the SAME string (now via the validated
    /// qualified arm).
    #[test]
    fn test_resolve_port_schema_store_resolution_is_idempotent() {
        let tmp = tempfile::tempdir().unwrap();
        write_store_msg_s4(tmp.path(), "acme", "WidgetState", "uint32 a\n");
        let first = resolve_port_schema(tmp.path(), "WidgetState").unwrap();
        let second = resolve_port_schema(tmp.path(), &first.schema).unwrap();
        assert_eq!(second.schema, first.schema);
        assert_eq!(second.provenance, PortSchemaProvenance::QualifiedStore);
    }

    /// The SECOND workspace tier (entry name): a file whose STEM
    /// (`my_foo`) differs from its schema entry name (`Foo`). `Foo.yaml`
    /// does not exist, so the file-stem tier misses and the entry-name
    /// scan resolves it — proving the two-tier lookup is wired.
    #[test]
    fn test_resolve_port_schema_entry_name_tier() {
        let tmp = tempfile::tempdir().unwrap();
        std::fs::write(
            tmp.path().join("my_foo.yaml"),
            "schemas:\n  Foo:\n    fields:\n      uint32 a:\n",
        )
        .unwrap();
        assert!(
            !tmp.path().join("Foo.yaml").exists(),
            "file-stem tier must miss so the entry-name tier is exercised"
        );
        let resolved = resolve_port_schema(tmp.path(), "Foo").unwrap();
        assert_eq!(resolved.schema, "Foo");
        assert_eq!(
            resolved.provenance,
            PortSchemaProvenance::Workspace { shadowed: None }
        );
    }

    /// `port_schema_exists` mirrors
    /// `schema_info_unified`'s FIRST tier — the workspace FILE STEM — for a
    /// QUALIFIED name too, not only a bare one.
    ///
    /// Its own doc claims the two resolve identically ("a value this accepts
    /// is one `cerulion schema info` can show"), and that was FALSE: anything
    /// containing a `/` skipped straight to the entry-name tier, so a
    /// workspace schema at `schemas/my_pkg/Foo.yaml` was DISPLAYED by `schema
    /// info` and REFUSED by `graph validate` — two surfaces disagreeing about
    /// the same workspace, which is exactly the class the mirror claim exists
    /// to rule out.
    ///
    /// The fixture is built so the file-stem tier is the ONLY one that can
    /// answer, which is what makes the arm load-bearing: the entry name
    /// inside the YAML is `Foo` (not `my_pkg/Foo`), and `workspace_yaml_files`
    /// reads the top level of `schemas/` ONLY — it never descends into
    /// `my_pkg/` — so the entry-name scan cannot see the file at all.
    #[test]
    fn a_qualified_workspace_file_stem_resolves_the_way_schema_info_shows_it() {
        let tmp = tempfile::tempdir().unwrap();
        std::fs::create_dir_all(tmp.path().join("my_pkg")).unwrap();
        std::fs::write(
            tmp.path().join("my_pkg").join("Foo.yaml"),
            "schemas:\n  Foo:\n    fields:\n      uint32 a:\n",
        )
        .unwrap();

        // THE CONTRACT, asserted from both ends in one body: `schema info`
        // shows it as a workspace schema …
        let shown = schema_info_unified(tmp.path(), "my_pkg/Foo")
            .expect("`cerulion schema info my_pkg/Foo` must resolve it");
        assert_eq!(shown.source, SchemaSource::Workspace);
        // … so the graph-validation gate must accept it.
        assert!(
            port_schema_exists(tmp.path(), "my_pkg/Foo").unwrap(),
            "a schema `schema info` displays must not be refused by `graph validate`"
        );

        // ANTI-TAUTOLOGY. Without these, "a qualified name resolves" is
        // satisfied by a check that accepts every value with a slash in it —
        // which is the original defect this whole function was added to close.
        assert!(
            !port_schema_exists(tmp.path(), "my_pkg/Missing").unwrap(),
            "a sibling name with no file must still be refused"
        );
        assert!(
            !port_schema_exists(tmp.path(), "made/up").unwrap(),
            "an invented qualified name must still be refused"
        );
        // And the tiers BELOW the new one are untouched.
        assert!(
            port_schema_exists(tmp.path(), "sensor_msgs/Image").unwrap(),
            "a real built-in still resolves"
        );
    }

    /// A uniquely-named `.msg` STORE schema resolves
    /// for a BARE name too.
    ///
    /// `resolve_port_schema` — which `port_schema_exists` delegated every bare
    /// name to — consults the workspace and then the built-in registry, never
    /// the store. So a schema acquired by `ros2 attach`'s acquisition ladder into
    /// `schemas/<pkg>/msg/<Type>.msg` was DISPLAYED by `schema info` and
    /// REFUSED by `graph validate`: the third tier of this ladder disagreeing,
    /// after the qualified and bare file-stem ones.
    #[test]
    fn a_bare_msg_store_schema_resolves_the_way_schema_info_shows_it() {
        let tmp = tempfile::tempdir().unwrap();
        let msg_dir = tmp.path().join("acme").join("msg");
        std::fs::create_dir_all(&msg_dir).unwrap();
        std::fs::write(msg_dir.join("WidgetState.msg"), "uint32 a\n").unwrap();

        // THE CONTRACT, both ends in one body: `schema info` shows it …
        let shown = schema_info_unified(tmp.path(), "WidgetState")
            .expect("`schema info WidgetState` must resolve it from the .msg store");
        assert_eq!(shown.source, SchemaSource::MsgStore);
        // … so the validation gate must accept it, bare AND qualified.
        assert!(
            port_schema_exists(tmp.path(), "WidgetState").unwrap(),
            "a bare .msg store schema `schema info` displays must not be refused"
        );
        assert!(
            port_schema_exists(tmp.path(), "acme/WidgetState").unwrap(),
            "the qualified spelling already worked and must keep working"
        );

        // ANTI-TAUTOLOGY: a bare name in NO tier is still refused, and a real
        // built-in still resolves — this adds a tier, it does not open a hole.
        assert!(!port_schema_exists(tmp.path(), "NotAThing").unwrap());
        assert!(port_schema_exists(tmp.path(), "Vector3").unwrap());
    }

    /// `schema info` and the graph gate agree on the `pkg::Type`
    /// spelling of a NESTED workspace schema.
    ///
    /// `schema_info_unified`'s file-stem tier joined the RAW name, so
    /// `acme::Widget` probed `schemas/acme::Widget.yaml` and missed, while
    /// `port_schema_exists` normalizes first and found
    /// `schemas/acme/Widget.yaml`. The gate accepted what `schema info`
    /// rejected — the same ladder-disagreement class as the other three tiers,
    /// with the sign flipped. `::` is a documented spelling of the qualified
    /// name, so normalizing is what the verb already promised.
    #[test]
    fn the_colon_colon_spelling_resolves_on_both_surfaces() {
        let tmp = tempfile::tempdir().unwrap();
        std::fs::create_dir_all(tmp.path().join("acme")).unwrap();
        std::fs::write(
            tmp.path().join("acme").join("Widget.yaml"),
            "schemas:\n  Widget:\n    fields:\n      uint32 a:\n",
        )
        .unwrap();

        for spelling in ["acme/Widget", "acme::Widget"] {
            assert!(
                schema_info_unified(tmp.path(), spelling).is_ok(),
                "`schema info {spelling}` must resolve the nested workspace schema"
            );
            assert!(
                port_schema_exists(tmp.path(), spelling).unwrap(),
                "and the graph gate must agree about `{spelling}`"
            );
        }

        // ANTI-TAUTOLOGY: normalizing does not make everything resolve.
        assert!(schema_info_unified(tmp.path(), "acme::Missing").is_err());
        assert!(!port_schema_exists(tmp.path(), "acme::Missing").unwrap());
    }

    /// The existence check must not MATERIALIZE a store schema's
    /// layout — a hostile-but-parseable `.msg` crashed `graph validate`.
    ///
    /// `store_schema_lookup` builds a full `SchemaEntry`, which resolves the
    /// layout, and layout resolution PANICS on fixed-array size overflow. The
    /// `.msg`-store tier added one commit earlier called it, so a single
    /// `float64[18446744073709551615] a` anywhere in `schemas/` panicked
    /// `cerulion graph validate` and `graph run` on the real
    /// binary (`fixed-array size overflow (8 x 18446744073709551615)`), and
    /// attributed by reverting just that tier, which made the same workspace
    /// refuse cleanly instead.
    ///
    /// A panicking `#[test]` fails, so reaching the assertion IS the oracle;
    /// the value is asserted too so the arm cannot pass by refusing everything.
    #[test]
    fn a_hostile_store_schema_does_not_crash_the_existence_check() {
        let tmp = tempfile::tempdir().unwrap();
        let msg_dir = tmp.path().join("acme").join("msg");
        std::fs::create_dir_all(&msg_dir).unwrap();
        std::fs::write(
            msg_dir.join("Hostile.msg"),
            "float64[18446744073709551615] a\n",
        )
        .unwrap();

        assert!(
            port_schema_exists(tmp.path(), "Hostile").unwrap(),
            "the store entry EXISTS — the check answers that without resolving \
             a layout it was never asked for"
        );
        assert!(
            port_schema_exists(tmp.path(), "acme/Hostile").unwrap(),
            "and the qualified spelling likewise"
        );
        // ANTI-TAUTOLOGY: a name in no tier is still refused, so this is an
        // existence probe that survives the hostile input, not one that says
        // yes to everything.
        assert!(!port_schema_exists(tmp.path(), "NotAThing").unwrap());
    }

    /// The BARE file-stem tier parses
    /// too — the qualified fix left the sibling tier probing with `exists()`.
    ///
    /// `resolve_port_schema` (which `port_schema_exists` delegates every BARE
    /// name to) resolved a workspace schema through a presence check that
    /// accepted any existing `schemas/<name>.yaml` path. A directory or
    /// a malformed file therefore passed `graph validate`/`graph run` while
    /// `schema info` rejected it — the same disagreement as the qualified
    /// tier, on the path most graphs actually take.
    #[test]
    fn a_bare_stem_that_is_a_directory_or_malformed_is_not_accepted() {
        let tmp = tempfile::tempdir().unwrap();

        // (1) A DIRECTORY named `<Name>.yaml`.
        std::fs::create_dir_all(tmp.path().join("DirSchema.yaml")).unwrap();
        assert!(
            schema_info_unified(tmp.path(), "DirSchema").is_err(),
            "precondition: `schema info` cannot show a directory"
        );
        assert!(
            port_schema_exists(tmp.path(), "DirSchema").is_err(),
            "and the validation gate must not accept it either"
        );

        // (2) MALFORMED YAML.
        std::fs::write(
            tmp.path().join("BadSchema.yaml"),
            "schemas: [not a mapping\n",
        )
        .unwrap();
        assert!(
            schema_info_unified(tmp.path(), "BadSchema").is_err(),
            "precondition: `schema info` refuses malformed YAML"
        );
        assert!(
            port_schema_exists(tmp.path(), "BadSchema").is_err(),
            "and so must the validation gate"
        );

        // ANTI-TAUTOLOGY: a well-formed bare workspace schema still resolves,
        // and so does a built-in — the tiers below are untouched.
        std::fs::write(
            tmp.path().join("GoodSchema.yaml"),
            "schemas:\n  GoodSchema:\n    fields:\n      uint32 a:\n",
        )
        .unwrap();
        assert!(port_schema_exists(tmp.path(), "GoodSchema").unwrap());
        assert!(port_schema_exists(tmp.path(), "Vector3").unwrap());
    }

    /// The qualified file-stem tier
    /// PARSES what it finds — `exists()` alone broke the mirror in the other
    /// direction.
    ///
    /// `schema_info_unified`'s tier 1 hands the path to `schema_info`, which
    /// reads and parses it, so a DIRECTORY at `schemas/<pkg>/<Type>.yaml` or a
    /// file of malformed YAML fails `schema info` loudly. Stopping at
    /// `exists()` made `graph validate` ACCEPT both — the same two surfaces
    /// disagreeing about one workspace as before, with the sign flipped, and
    /// the more dangerous direction: a validation gate that passes a schema
    /// nothing can read.
    #[test]
    fn a_qualified_stem_that_is_a_directory_or_malformed_is_not_accepted() {
        let tmp = tempfile::tempdir().unwrap();
        std::fs::create_dir_all(tmp.path().join("dirpkg")).unwrap();

        // (1) A DIRECTORY that happens to be named `<Type>.yaml`.
        std::fs::create_dir_all(tmp.path().join("dirpkg").join("Foo.yaml")).unwrap();
        assert!(
            schema_info_unified(tmp.path(), "dirpkg/Foo").is_err(),
            "precondition: `schema info` cannot show a directory"
        );
        assert!(
            port_schema_exists(tmp.path(), "dirpkg/Foo").is_err(),
            "and the validation gate must not accept it either"
        );

        // (2) A real file of MALFORMED YAML.
        std::fs::create_dir_all(tmp.path().join("badpkg")).unwrap();
        std::fs::write(
            tmp.path().join("badpkg").join("Bar.yaml"),
            "schemas: [this is not a mapping\n",
        )
        .unwrap();
        assert!(
            schema_info_unified(tmp.path(), "badpkg/Bar").is_err(),
            "precondition: `schema info` refuses malformed YAML"
        );
        assert!(
            port_schema_exists(tmp.path(), "badpkg/Bar").is_err(),
            "and so must the validation gate"
        );

        // ANTI-TAUTOLOGY: a WELL-FORMED file at the same kind of path still
        // resolves, so this is a parse gate and not a rejection of nested
        // stems (which is what the sibling arm above exists for).
        std::fs::create_dir_all(tmp.path().join("okpkg")).unwrap();
        std::fs::write(
            tmp.path().join("okpkg").join("Baz.yaml"),
            "schemas:\n  Baz:\n    fields:\n      uint32 a:\n",
        )
        .unwrap();
        assert!(
            port_schema_exists(tmp.path(), "okpkg/Baz").unwrap(),
            "a parseable nested workspace schema must still resolve"
        );
    }

    #[test]
    fn test_normalize_schema() {
        assert_eq!(
            normalize_schema("sensor_msgs::LaserScan"),
            "sensor_msgs/LaserScan"
        );
        assert_eq!(
            normalize_schema("sensor_msgs/LaserScan"),
            "sensor_msgs/LaserScan"
        );
        assert_eq!(normalize_schema("std_msgs::Header"), "std_msgs/Header");
    }

    #[test]
    fn test_display_schema() {
        assert_eq!(
            display_schema("sensor_msgs/LaserScan"),
            "sensor_msgs::LaserScan"
        );
        assert_eq!(display_schema("std_msgs/Header"), "std_msgs::Header");
    }

    /// A `.msg` store definition and a workspace YAML entry of the
    /// SAME qualified name. Naming that spelling is REFUSED
    /// (`workspace_lookup`); a NESTED reference to it from another schema's
    /// field is not a spelling anyone chose, and the tree beneath it must
    /// render the WORKSPACE definition — the tier the verb itself selects —
    /// not the store's. The resolution context is last-wins per qualified
    /// name, so the order `resolution_schema_set` assembles the tiers in IS
    /// this precedence: store LAST would let `nav/msg/Bar.msg`
    /// overwrite the YAML `nav/Bar` beneath `Foo`, and `inner` would render the
    /// store's field. Both sides: with the YAML gone the store answers the
    /// same reference. Swapping the tiers back fails the tree arm
    /// (the workspace-wins assertion beneath `Foo`; the spelling's own
    /// refusal is untouched by it).
    #[test]
    fn a_nested_reference_to_a_yaml_store_twin_renders_the_workspace_definition() {
        let tmp = tempfile::tempdir().unwrap();
        let schemas = tmp.path().join("schemas");
        std::fs::create_dir_all(schemas.join("nav").join("msg")).unwrap();
        std::fs::write(
            schemas.join("Foo.yaml"),
            "schemas:\n  Foo:\n    fields:\n      \"nav/Bar inner\":\n      \"uint32 x\":\n",
        )
        .unwrap();
        std::fs::write(
            schemas.join("nav").join("msg").join("Bar.msg"),
            "uint32 from_store\n",
        )
        .unwrap();
        let twin = schemas.join("bar.yaml");
        std::fs::write(
            &twin,
            "schemas:\n  nav/Bar:\n    fields:\n      \"uint32 from_yaml\":\n",
        )
        .unwrap();

        // The spelling itself names two definitions: refused, both named.
        let reason = match workspace_lookup(&schemas, "nav/Bar") {
            Err(err) => err.to_string(),
            Ok(_) => panic!("a qualified YAML/store twin is refused when spelled"),
        };
        assert!(
            reason.contains("'nav/Bar' is ambiguous in this workspace")
                && reason.contains("schemas/bar.yaml (entry nav/Bar)")
                && reason.contains("schemas/nav/msg/Bar.msg (store)"),
            "{reason}"
        );

        // Beneath `Foo`, the nested reference renders the WORKSPACE definition.
        let out = schema_info_unified(&schemas, "Foo").unwrap().to_string();
        assert!(
            out.contains("\n  inner: nav/Bar (")
                && out.contains("\n    from_yaml: uint32 (fixed)\n"),
            "the workspace YAML definition must win beneath Foo:\n{out}"
        );
        assert!(
            !out.contains("from_store"),
            "the store twin must not overwrite the YAML definition in the tree:\n{out}"
        );

        // Both sides: with the YAML gone, the store answers the reference.
        std::fs::remove_file(&twin).unwrap();
        let out = schema_info_unified(&schemas, "Foo").unwrap().to_string();
        assert!(
            out.contains("\n  inner: nav/Bar (")
                && out.contains("\n    from_store: uint32 (fixed)\n")
                && !out.contains("from_yaml"),
            "with the YAML twin gone the store definition renders:\n{out}"
        );
    }
}
