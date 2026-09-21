// SPDX-License-Identifier: AGPL-3.0-only
//! The by-name FIELD REGISTRY + read-only variable-section
//! decoder that the tolerance validator and the tolerant diff share.
//!
//! # What it does
//!
//! Given the bag's embedded graph + the CURRENT workspace's `schemas/` set (and
//! the built-in ROS 2 registry), it resolves:
//!
//! - **topic → schema**: a graph-produced topic's schema is the producing
//!   output's `schema:` (directly in the graph YAML); the combined
//!   workspace + built-in schema set resolves that name to a
//!   [`cerulion_core::codegen::MessageSchema`] and its
//!   [`WireLayout`].
//! - **topic + dotted field PATH → a field**: a pure schema-tree walk that
//!   descends through nested schemas — INCLUDING variable sequences of nested
//!   schemas (`detections.bbox` addresses `bbox` on EACH `detections` element).
//!   This powers the tolerance validator's exit-4 typo detection (via the
//!   [`crate::tolerance::FieldResolver`] impl) and the per-field
//!   metric application.
//! - **a recorded frame's fields BY NAME → bytes/typed values**: the read-only
//!   [`FrameDecoder`], which locates a fixed-section field (primitive /
//!   `StringFixed` / fixed-nested leaf, at its computed offset) or a variable
//!   field (String / Bytes / `DynamicArray<primitive>`, via the offset table)
//!   within raw frame bytes — bounds-checking EVERYTHING so a corrupt bag
//!   produces a loud [`DecodeError`], never UB.
//!
//! # Placement (why cli_engine, not core)
//!
//! topic→schema resolution is inherently a graph + workspace concern — the
//! sibling `graph_cmd::build_workspace_schema_hashes` /
//! `build_workspace_schema_wire_sizes` already live here doing exactly this
//! kind of workspace-schema resolution, and `schema_cmd::builtin_layout_resolver`
//! is the built-in-registry precedent. The byte-level primitives the decoder
//! needs ([`WireLayout`], [`LayoutResolver`],
//! [`cerulion_core::shm_runtime::read_offset_entry`],
//! [`WireHeader`]) are already `pub` in core, so this needs
//! no new core surface. It sits next to `replay_engine` because it is a
//! replay-specific registry.
//!
//! # Decode scope
//!
//! The framework treats a `DynamicArray<Nested>` (e.g. a detection array) as
//! OPAQUE publisher-written bytes — there is NO framework-defined per-element
//! sub-encoding (native macro nodes write it via `set_<f>_bytes(&[u8])`), so a
//! generic per-element decode INTO a variable nested array is not possible and
//! is NOT faked here (see [`DecodeError::UndecodableVariableNested`]). A
//! non-`bit_exact` metric on such a field is refused LOUDLY at validation
//! ([`FieldRegistry::check_field_metric`], exit 4); [`FrameDecoder::decode_field_f64`]
//! decodes only metric-decodable fields (a top-level numeric scalar or a
//! `float64`-class array). Field-path RESOLUTION (typo detection) still walks
//! the full schema tree including opaque paths — resolution is a pure
//! schema-tree property, decode is not.

use std::collections::HashMap;

use cerulion_core::codegen::layout::{LayoutResolver, WireLayout};
use cerulion_core::codegen::{FieldType, MessageSchema};
use cerulion_core::graph::GraphConfig;
use cerulion_core::wire::WireHeader;

use crate::tolerance::{FieldResolveError, FieldResolver, MetricKind};

/// A field's classification within a topic's root schema, resolved by name.
#[derive(Debug, Clone, PartialEq)]
pub enum TopicClass {
    /// The topic is produced by an in-graph node output — its schema is known.
    Produced,
    /// The topic is consumed but not produced in-graph (the absolute `source:`
    /// class) — its schema is NOT carried in the graph YAML, so field-level
    /// validation is not available for it.
    External,
}

/// A replay topic and (when known) its root schema's qualified name.
struct TopicEntry {
    class: TopicClass,
    /// The producing output's `schema:` resolved to a qualified name in the
    /// combined schema set, or `None` when the topic's schema is unavailable
    /// (external topics, or a produced topic whose schema is not in the set —
    /// e.g. a schema referencing a type absent from both workspace + built-ins).
    schema_qname: Option<String>,
    /// The recipe-3 `schema_hash` of the resolved layout for `schema_qname`,
    /// computed once over the combined (workspace + built-in) set exactly as
    /// codegen does (`resolve_fixed_nested` then `MessageSchema::schema_hash`).
    /// The [`FrameDecoder`] compares each frame's `WireHeader.schema_hash`
    /// against this to reject a compatible-size layout drift. `None` when the
    /// schema is unavailable/unresolvable (the hash check is then skipped).
    expected_schema_hash: Option<u64>,
}

/// The by-name field registry for one replay.
pub struct FieldRegistry {
    /// Combined workspace + built-in schema set, resolved once
    /// (`resolve_fixed_nested` runs inside every [`LayoutResolver::new`] built
    /// from it). Retained (not just folded into a `LayoutResolver`) because the
    /// `&self` [`FieldResolver::resolve_field`] path needs to build a scratch
    /// resolver (`layout_of` is `&mut self`) and [`FrameDecoder`] owns its own.
    schemas: Vec<MessageSchema>,
    /// The identity-resolution indexes over `schemas` (the ONE
    /// binding rule for nested references), built by
    /// [`build_identity_indexes`] so the production and hermetic-test
    /// constructors cannot drift. See that fn's doc for the three maps'
    /// contracts.
    identity: IdentityIndexes,
    /// Replay topic → entry, in a stable (graph) order.
    topics: Vec<(String, TopicEntry)>,
    /// Canonical identity (`pkg/Type`) → the DECLARED spelling the retained
    /// [`MessageSchema`] carries. A topic's `schema_qname` is
    /// canonical (what a normalized `schema:` looks up), but
    /// [`LayoutResolver::new`] keys `by_qualified` by the declared
    /// `qualified_name()` (the hash identity — a `pkg::Type` YAML entry stays
    /// `pkg::Type` there), so every string-addressed layout lookup translates
    /// through here first. Last-wins over the same ladder order as
    /// `by_qualified`, so the two agree on which identity a string names.
    /// Without it a `::` entry resolved its HASH and then reported
    /// `SchemaUnavailable` from the very next layout lookup, silently
    /// skipping field-level tolerance validation.
    layout_keys: HashMap<String, String>,
}

/// The identity-resolution indexes a [`FieldRegistry`] binds nested
/// references through (the fifth and final site of the
/// identity-collapse class: every ref, bare or qualified, resolves
/// through core's `(package, name)` identity rule, never by comparing
/// qualified-name STRINGS).
struct IdentityIndexes {
    /// Resolver key `(package, name)` → index into `schemas`, later-wins —
    /// exactly `Resolver::new`'s `by_key`. A QUALIFIED ref is an exact
    /// lookup here (core does NOT fall through on a miss); an unqualified
    /// ref goes same-package → bare `Header` → unambiguous global bare.
    by_key: HashMap<(Option<String>, String), usize>,
    /// Qualified-name STRING → the index whose layout every string-keyed
    /// consumer serves for it (later-wins over the vec — the same rule as
    /// `LayoutResolver::new`'s `by_qualified` and the hash collect). The
    /// MATERIALIZATION gate: `layout_of` is string-addressed, so a bound
    /// identity can only be walked when it IS its own string's winner —
    /// anything else would descend a same-string different-identity
    /// layout core never bound.
    string_winner: HashMap<String, usize>,
    /// Bare name → the ONE identity bearing it when exactly one does
    /// (computed over the `by_key`-deduped identities — the
    /// FLAT global-bare rule `resolve_fixed_nested` applies, tier-free,
    /// one identity counted once). `None` = ambiguous, never guessed.
    nested_bare_index: HashMap<String, Option<usize>>,
}

/// Build the [`IdentityIndexes`] over a FINAL schema vec — the one seam
/// both `from_graph` and the hermetic test constructor go through.
fn build_identity_indexes(schemas: &[MessageSchema]) -> IdentityIndexes {
    let mut by_key: HashMap<(Option<String>, String), usize> = HashMap::new();
    let mut string_winner: HashMap<String, usize> = HashMap::new();
    for (idx, s) in schemas.iter().enumerate() {
        by_key.insert((s.package.clone(), s.name.clone()), idx);
        string_winner.insert(s.qualified_name(), idx);
    }
    let mut bare_counts: HashMap<&str, Vec<usize>> = HashMap::new();
    for ((_, name), &idx) in &by_key {
        bare_counts.entry(name.as_str()).or_default().push(idx);
    }
    let nested_bare_index: HashMap<String, Option<usize>> = bare_counts
        .into_iter()
        .map(|(bare, idxs)| {
            let resolved = if let [one] = idxs.as_slice() {
                Some(*one)
            } else {
                None
            };
            (bare.to_string(), resolved)
        })
        .collect();
    IdentityIndexes {
        by_key,
        string_winner,
        nested_bare_index,
    }
}

impl FieldRegistry {
    /// Build the registry from the bag's graph + the workspace root.
    ///
    /// Produced topics + their output schemas come from the graph; external
    /// (consumed-only) topics are recorded as known-but-schema-unavailable. The
    /// schema set combines the workspace `schemas/*.yaml`, the workspace
    /// `.msg` store (`schemas/<pkg>/msg/<Type>.msg`, via the shared
    /// [`crate::schema_store::SchemaStore`] seam — msg-store parity: a
    /// store-typed topic must not
    /// degrade to schema-unavailable, silently skipping its field
    /// validation), and the built-in ROS 2 registry, resolved together so a
    /// workspace schema nesting a ROS 2 `Header` resolves. Cross-tier
    /// precedence (the shadow semantics of the documented ladder) is
    /// expressed by ORDER, never by deletion: the set is
    /// assembled ASCENDING — built-ins, store, workspace YAML — so every
    /// last-wins string map (the hash index, `LayoutResolver`'s
    /// `by_qualified`) hands a claimed NAME to the highest tier, while
    /// every resolver identity `(package, name)` stays in the set for core
    /// to bind references against (a string-equality filter would
    /// delete a store twin core still binds — see the precedence comment
    /// in the body). Never fails: a schema-set problem degrades the
    /// affected topic to schema-unavailable (validation of OTHER topics
    /// still works), matching the loud-but-non-fatal norm of the sibling
    /// builders.
    pub fn from_graph(config: &GraphConfig, workspace_root: &std::path::Path) -> Self {
        // The three tiers come from the ONE shared assembly
        // (`schema_cmd::ResolutionTiers` — the same enumeration `schema
        // info`, the recording fold and `topic echo`'s walker resolve
        // against), so no consumer can disagree on membership or on which
        // duplicate wins.
        let schemas_dir = workspace_root.join("schemas");
        let tiers = crate::schema_cmd::ResolutionTiers::assemble(Some(&schemas_dir));
        // The YAML tier's bare-name claims — every ENTRY name and
        // every file STEM (the file-stem rule), taken over the PARSED tier
        // before any scrub (a claim whose definition a scrub drops
        // still MASKS the lower tiers — see the bare index below) and keyed
        // by the assembly's own files, so a stem's OWNER is a file this
        // registry holds the definitions of.
        let yaml_claims = crate::schema_cmd::workspace_yaml_bare_claims_of(&tiers.yaml_by_file);
        let crate::schema_cmd::ResolutionTiers {
            builtin: mut builtin_schemas,
            store: mut store_schemas,
            mut yaml_by_file,
        } = tiers;
        // Port resolution's bare-name claim over the
        // PARSED store — taken BEFORE the sizing retains below, because a
        // bearer those retains drop still exists on disk and still makes the
        // name ambiguous at validation (`unique_bare_store_names`, the ONE
        // rule the fold, the hash map and the gateway derive theirs from).
        let unique_bare_store =
            crate::schema_cmd::unique_bare_store_names(&store_schemas, &yaml_claims);
        let parsed_store_bare_names: std::collections::HashSet<String> =
            store_schemas.iter().map(|s| s.name.clone()).collect();
        // A hostile-but-parseable workspace declaration must not panic replay
        // (`schema_hash`/layout materialization panic on FixedArray
        // overflow); built-ins are the compiled corpus and need no probe.
        for (_, parsed) in &mut yaml_by_file {
            crate::schema_cmd::retain_sizable_schemas(parsed, "replay field registry");
        }
        crate::schema_cmd::retain_sizable_schemas(&mut store_schemas, "replay field registry");
        // The wire ceiling is judged at the RESOLVED moment only (below),
        // after composition — a declared-moment retain here would drop an
        // over-ceiling nested TARGET before the composed scrub and leave
        // its parents unresolved.
        // Precedence by order, never by deletion. Two
        // rules coexist here and they key on DIFFERENT things:
        //
        // - the LADDER (workspace YAML → store → built-ins)
        //   governs NAME-STRING claims — the graph's `schema:` strings and
        //   every string-keyed winner map (`compute_schema_hashes`'
        //   collect, `LayoutResolver::new`'s `by_qualified`). Both maps
        //   pick the LATER definition, so ordering every set ASCENDING in
        //   precedence — built-ins first, store, workspace YAML LAST (the
        //   renderer contract) — makes last-wins express the
        //   ladder with no definition removed.
        //
        // - core's RESOLVER IDENTITIES `(package, name)` govern REFERENCE
        //   binding: a store `acme/Widget` is `(Some("acme"), "Widget")`
        //   while a slash-named YAML entry is `(None, "acme/Widget")` —
        //   TWO entries `resolve_fixed_nested` keeps apart, and a
        //   QUALIFIED nested ref `acme/Widget` binds the STORE one. A
        //   filter that deletes the store copy on raw qualified-name
        //   equality leaves that ref UNRESOLVED here while codegen, the
        //   robot's producer, and the graph_cmd folds all bind
        //   and inline the store definition — the expected hash diverges
        //   from the wire for every schema nesting the collided name.
        //
        // So: nothing is filtered; same-KEY duplicates (a store type
        // shadowing a real built-in `(Some(pkg), Type)`) resolve later-wins
        // inside core exactly as its `by_key` does, and distinct-key
        // same-STRING pairs coexist — the string maps serve the YAML
        // winner, core binding serves whichever identity a reference names.

        // Resolve fixed-nested over a clone of the FULL combined set (exactly
        // the phase codegen runs before computing `ShmMessage::SCHEMA_HASH`),
        // then index each schema's recipe-3 `schema_hash`. Because the set is
        // workspace + ALL built-ins, a workspace schema nesting a ROS 2 type
        // resolves here just as codegen resolves it — so the resulting hash is
        // codegen-faithful (the `build_workspace_schema_hashes` closure-escape
        // caveat does not apply: nothing escapes the combined set).
        //
        // A schema whose composed fixed section overflows (individually
        // sizable, the overflow appearing only through fixed-nested
        // inlining) is scrubbed EVERYWHERE — from the TIER vecs, before
        // anything below is built over them: the schema set (which
        // `decoder_for` / `resolve_field` materialize layouts from), the
        // bare index, and the qualified set — never just a local hash
        // clone. Leaving it reachable anywhere lets a tolerance run
        // materialize its layout later and PANIC instead of degrading that
        // topic to schema-unavailable.
        // Scrubbing BEFORE the bare-index tier counting also keeps
        // bare-name uniqueness judged over the survivors (the same
        // principle as the wire-size map).
        //
        // This loop closes two defects:
        //
        // - PREFLIGHT: a composed-overflow schema used as a nested
        //   TARGET panics inside `resolve_fixed_nested` ITSELF
        //   (`resolved_fixed_of` sizes each fixed target via the panicking
        //   `wire_fixed_size`), before `compute_schema_hashes`' post-resolve
        //   retain can drop anything — so every pass runs
        //   `retain_composed_sizable_schemas` over the combined set FIRST.
        // - FIXPOINT: a drop can flip a bare reference's resolution
        //   (two global bare candidates → one; ambiguous → resolved), which
        //   changes a SURVIVOR's resolved layout. Hashing once and then
        //   scrubbing left the survivor's expected hash computed under the
        //   PRE-scrub ambiguity while `decoder_for`'s `LayoutResolver` sees
        //   the post-scrub set and inlines — expected hash passing while
        //   replay decodes different offsets. So the hashes are recomputed
        //   over the post-scrub set until nothing more drops: the hash map
        //   and every layout materialized below derive from the SAME set.
        //
        // Terminates: a pass either breaks or strictly shrinks the tiers
        // (every dropped qname is present in the pass's combined set).
        let hash_by_qname = loop {
            // ASCENDING precedence (built-ins → store → YAML) so every
            // last-wins string map expresses the ladder — see the
            // precedence comment above the fixpoint.
            let mut combined: Vec<MessageSchema> = builtin_schemas
                .iter()
                .chain(store_schemas.iter())
                .chain(yaml_by_file.iter().flat_map(|(_, parsed)| parsed.iter()))
                .cloned()
                .collect();
            let mut dropped = crate::schema_cmd::retain_composed_sizable_schemas(
                &mut combined,
                "replay field registry (composed preflight)",
            );
            let (hashes, composed_unsizable) = compute_schema_hashes(&combined);
            dropped.merge(composed_unsizable);
            if dropped.is_empty() {
                break hashes;
            }
            // Scrub tiers via the TWO-part removal contract:
            // fully-removed NAMES plus removed IDENTITIES — a
            // failing store definition whose flattened name survives via a
            // slash-named YAML twin is a PARTIAL removal, and a name-only
            // scrub left it in the tiers for `LayoutResolver::new` to
            // panic on when a qualified nested ref bound the store
            // identity (the precedence-by-order coexistence made the shape reachable here).
            let alive = |s: &MessageSchema| !dropped.removes(s);
            let before: usize = yaml_by_file.iter().map(|(_, p)| p.len()).sum::<usize>()
                + store_schemas.len()
                + builtin_schemas.len();
            for (_, parsed) in &mut yaml_by_file {
                parsed.retain(alive);
            }
            store_schemas.retain(alive);
            builtin_schemas.retain(alive);
            let after: usize = yaml_by_file.iter().map(|(_, p)| p.len()).sum::<usize>()
                + store_schemas.len()
                + builtin_schemas.len();
            // NO-PROGRESS FALLBACK. Termination must not rest on "every dropped
            // definition is matched by `removes`"; when that fails (the
            // identity-state bug) the loop rebuilds an identical set forever.
            // Breaking out instead would be WORSE than the hang: this scrub IS
            // the preflight — `resolve_fixed_nested` runs on the very next
            // statement and PANICS materializing a composed-overflow target —
            // so a set handed on unscrubbed aborts the verb.
            //
            // So a no-progress pass FORCE-SCRUBS by identity alone
            // (`removes_identity`, the hash conjunct dropped) and loops again.
            // Every recorded identity came from a clone of this very set, so
            // that predicate always matches: the set strictly shrinks and the
            // loop terminates unconditionally. A healthy same-identity sibling
            // may go with the offender — a loud schema-unavailable degrade,
            // never a panic.
            //
            // (Here the unscrubbed tiers become `FieldRegistry.schemas`, and
            // every materializing seam — `decoder_for`, `resolve_field`,
            // `topic_field_names` — builds a `LayoutResolver` over them with
            // no filter of its own.)
            if after == before {
                tracing::warn!(
                    dropped = dropped.removed_definitions.len(),
                    context = "replay field registry",
                    "a preflight removal matched no schema in the tiers — \
                     force-scrubbing by identity alone so the fixpoint makes \
                     progress; a healthy same-identity sibling may be dropped \
                     with the offender"
                );
                let by_identity = |s: &MessageSchema| !dropped.removes_identity(s);
                for (_, parsed) in &mut yaml_by_file {
                    parsed.retain(by_identity);
                }
                store_schemas.retain(by_identity);
                builtin_schemas.retain(by_identity);
            }
        };

        // Bare index: TIERED, mirroring the resolution ladder
        // (workspace YAML → `.msg` store → built-ins). Lower tiers are
        // written first and higher tiers overwrite, so a bare name claimed
        // by a higher tier resolves there — and an AMBIGUOUS claim within a
        // tier maps to `None` (never guess), masking lower tiers exactly as
        // the ladder's loud-ambiguity arm refuses to fall through.
        let mut bare_index: HashMap<String, Option<String>> = HashMap::new();
        // Built-in tier: a unique short name binds; ambiguous ⇒ `None`.
        {
            let mut counts: HashMap<String, Vec<String>> = HashMap::new();
            for s in &builtin_schemas {
                counts
                    .entry(s.name.clone())
                    .or_default()
                    .push(s.qualified_name());
            }
            for (bare, qs) in counts {
                let resolved = if qs.len() == 1 {
                    Some(qs[0].clone())
                } else {
                    None
                };
                bare_index.insert(bare, resolved);
            }
        }
        // Store tier — port resolution's rule over the PARSED store
        // (counting post-scrub survivors let a hostile
        // `p1/Type` beside a valid `p2/Type` resolve bare `Type` to `p2/Type`
        // at replay while validation REFUSES it as ambiguous — a sibling
        // pick): exactly one parsed bearer that survived the scrubs binds;
        // two parsed bearers ⇒ refused (`None`, masking the built-in tier
        // exactly as validation refuses to fall through); a unique bearer
        // the sizing scrubs removed ⇒ `None` (schema-unavailable, as its own
        // qualified name would be).
        for bare in &parsed_store_bare_names {
            let bound = match unique_bare_store.get(bare) {
                Some(q) if store_schemas.iter().any(|s| s.qualified_name() == *q) => {
                    Some(q.clone())
                }
                _ => None,
            };
            bare_index.insert(bare.clone(), bound);
        }
        // Workspace-YAML tier — every bare name the YAML tier CLAIMS at port
        // resolution (`yaml_claims`, over the PARSED tier), each mapped to
        // what the SURVIVING definitions can serve for it. An ENTRY claim
        // binds its own name string while any definition bearing it
        // survives — a duplicate entry name across files serves the string
        // maps' later-wins winner, the decision both this registry and
        // the graph-hash fold pin (`r8_duplicate_yaml_names_pick_one_winner_
        // on_both_surfaces_regardless_of_file_order`) — and a claim whose
        // EVERY bearer a scrub removed is an explicit `None`:
        // a scrubbed YAML claim still masks the store
        // and built-in tiers, because validation binds the name to the YAML
        // tier (the file parses; only its SIZING failed), so replaying
        // against a lower tier's definition would validate a bag against a
        // document the resolver never bound. Counting the tier over
        // the survivors alone would leave the store alias in place for
        // a scrubbed entry.
        let mut yaml_bearers: HashMap<&str, Vec<usize>> = HashMap::new();
        for (file, (_, parsed)) in yaml_by_file.iter().enumerate() {
            for s in parsed {
                yaml_bearers.entry(s.name.as_str()).or_default().push(file);
            }
        }
        // (A QUALIFIED string the YAML tier claims — a slash-named workspace
        // entry `pkg/Type` beside a store `pkg/Type` — needs no masking of
        // its own here: the YAML definition is always that string's LAST
        // bearer, so when its sizing fails the winner sweep in
        // `compute_schema_hashes` removes EVERY bearer of the string, the
        // store twin included, and the flat name set no longer carries it.
        // Pinned by `r24_a_scrubbed_qualified_yaml_claim_masks_the_store_
        // twin_at_replay`; a separate masking set for it would be
        // inert.)
        for (bare, claim) in &yaml_claims {
            match claim {
                crate::schema_cmd::YamlClaim::Entry { .. } => {
                    let survives = yaml_bearers.contains_key(bare.as_str());
                    bare_index.insert(bare.clone(), survives.then(|| bare.clone()));
                }
                // An ambiguous spelling is refused
                // on every surface — schema-unavailable here, masking the lower
                // tiers exactly as validation refuses the name; said once.
                crate::schema_cmd::YamlClaim::Refused(ambiguous) => {
                    ambiguous.warn("replay field registry");
                    bare_index.insert(bare.clone(), None);
                }
                crate::schema_cmd::YamlClaim::Stem { .. } => {}
            }
        }
        // File-STEM claims (owner-qualified): a bare name
        // the YAML tier claims by file STEM binds an entry OF THAT FILE
        // (`workspace_yaml_bare_claims_of` — the same rule the gateway binds
        // by, the hash map checks by and the fold records by), so a bag whose
        // channel is labelled `Goal` beside a `Goal.yaml` declaring only
        // `Other` replays with `Other`'s schema instead of schema-unavailable.
        // A stem beside another file's entry of
        // its name, a multi-entry stem, or a stem bound to an ambiguous entry
        // is a REFUSED claim (handled in the loop above), so a bound stem's
        // entry has ONE declarer — the owner. This registry addresses a schema
        // by its NAME STRING (`hash_by_qname`, the `LayoutResolver`'s
        // `by_qualified`), so the owner's definition is selectable whenever
        // it survived the scrubs; the one remaining `None` is an owner the
        // sizing scrubs removed (schema-unavailable, as its own name would
        // be), said loudly.
        for binding in crate::schema_cmd::stem_bindings(&yaml_claims) {
            let owner = yaml_by_file
                .iter()
                .position(|(path, _)| path.as_path() == binding.file);
            let bearers = yaml_bearers
                .get(binding.entry)
                .map(Vec::as_slice)
                .unwrap_or(&[]);
            // A bound stem's entry has ONE declarer (every other shape is a
            // refused claim), so the owner's definition is selectable by name
            // whenever it survived the scrubs; scrubbed ⇒ an explicit `None`,
            // said loudly — never a lower tier's definition of the same name.
            let bound = match owner {
                // The bound entry is the DECLARED spelling; the hash map is
                // keyed canonically.
                Some(owner) if bearers == [owner] => {
                    Some(crate::schema_cmd::normalize_schema(binding.entry))
                }
                _ => {
                    tracing::warn!(
                        schema = %binding.stem,
                        entry = %binding.entry,
                        file = %binding.file.display(),
                        context = "replay field registry",
                        "the workspace YAML file claiming this bare name by its file \
                         stem binds an entry the resolution preflight rejected — the \
                         topic replays schema-unavailable (never a lower tier's \
                         definition of the same name)"
                    );
                    None
                }
            };
            bare_index.insert(binding.stem.to_string(), bound);
        }

        // The combined set, in the SAME ascending-precedence order as the
        // fixpoint's `combined` (built-ins → store → YAML) — the string
        // maps built over it are last-wins, so the order IS the ladder.
        let mut schemas = builtin_schemas;
        schemas.extend(store_schemas);
        schemas.extend(yaml_by_file.into_iter().flat_map(|(_, parsed)| parsed));
        // The identity-resolution indexes (the ONE
        // binding rule for nested references; see `IdentityIndexes`).
        let identity = build_identity_indexes(&schemas);
        // Canonical identities: a recorded `schema:` is normalized
        // before the lookup, so a `pkg::Type` entry must be findable by
        // `pkg/Type`; its hash stays the declared name's.
        let schema_names: Vec<String> = schemas
            .iter()
            .map(|s| crate::schema_cmd::normalize_schema(&s.qualified_name()))
            .collect();
        // The declared spelling behind each canonical identity —
        // the key `LayoutResolver` answers to (last-wins, the same ladder).
        let layout_keys: HashMap<String, String> = schemas
            .iter()
            .map(|s| {
                let declared = s.qualified_name();
                (crate::schema_cmd::normalize_schema(&declared), declared)
            })
            .collect();
        let refused: std::collections::HashSet<String> =
            crate::schema_cmd::refused_claims(&yaml_claims)
                .map(|a| a.name.clone())
                .collect();
        let qualified_set: std::collections::HashSet<&str> =
            schema_names.iter().map(String::as_str).collect();

        // Map produced topics → resolved schema qname; collect external topics.
        let mut topics: Vec<(String, TopicEntry)> = Vec::new();
        let mut seen: std::collections::HashSet<String> = std::collections::HashSet::new();
        for node in &config.nodes {
            for output in &node.outputs {
                let topic =
                    cerulion_core::graph::resolve_output_topic(&config.prefix, &node.id, output);
                if !seen.insert(topic.clone()) {
                    continue;
                }
                let schema_qname =
                    resolve_schema_name(&output.schema, &qualified_set, &bare_index, &refused);
                let expected_schema_hash = schema_qname
                    .as_ref()
                    .and_then(|q| hash_by_qname.get(q).copied());
                topics.push((
                    topic,
                    TopicEntry {
                        class: TopicClass::Produced,
                        schema_qname,
                        expected_schema_hash,
                    },
                ));
            }
        }
        for node in &config.nodes {
            for input in &node.inputs {
                let topic = cerulion_core::graph::resolve_source(&config.prefix, &input.source);
                if seen.contains(&topic) {
                    continue; // produced (or already added external)
                }
                if !seen.insert(topic.clone()) {
                    continue;
                }
                topics.push((
                    topic,
                    TopicEntry {
                        class: TopicClass::External,
                        // Inputs carry no schema in the graph YAML.
                        schema_qname: None,
                        expected_schema_hash: None,
                    },
                ));
            }
        }

        Self {
            schemas,
            identity,
            topics,
            layout_keys,
        }
    }

    /// The key the [`LayoutResolver`] built over `self.schemas` answers to for
    /// a canonical root: the retained schema's DECLARED spelling.
    /// A name with no declared twin (a built-in, a `.msg` store type — both
    /// already `pkg/Type`) is its own key.
    fn layout_key(&self, canonical: &str) -> String {
        self.layout_keys
            .get(canonical)
            .cloned()
            .unwrap_or_else(|| canonical.to_string())
    }

    /// The CURRENT-workspace recipe-3 `schema_hash` this replay
    /// expects for `topic`'s root schema, or `None` when the schema is
    /// unavailable (an external/consumed-only topic, or a produced topic whose
    /// schema does not resolve in the combined set). The schema-drift preflight
    /// (`replay_cmd::detect_schema_drift`) compares this against the recorded
    /// frame's `WireHeader.schema_hash` to refuse a since-recording layout drift
    /// with exit 2 (rather than an unexplained per-frame byte mismatch, exit 1).
    pub fn expected_schema_hash(&self, topic: &str) -> Option<u64> {
        self.topics
            .iter()
            .find(|(t, _)| t == topic)
            .and_then(|(_, e)| e.expected_schema_hash)
    }

    /// The topic's [`TopicClass`], or `None` if the topic is unknown.
    pub fn topic_class(&self, topic: &str) -> Option<&TopicClass> {
        self.topics
            .iter()
            .find(|(t, _)| t == topic)
            .map(|(_, e)| &e.class)
    }

    /// A [`FrameDecoder`] for `topic`'s root schema, or `None` when the schema
    /// is unavailable (external / unresolvable). The decoder owns its own
    /// [`LayoutResolver`] (built from the shared schema set), so this is `&self`.
    pub fn decoder_for(&self, topic: &str) -> Option<FrameDecoder> {
        let entry = self
            .topics
            .iter()
            .find(|(t, _)| t == topic)
            .map(|(_, e)| e)?;
        let qname = entry.schema_qname.clone()?;
        let (layouts, _) = LayoutResolver::new(self.schemas.clone());
        Some(FrameDecoder {
            layouts,
            // The decoder addresses layouts by the DECLARED spelling.
            root_qname: self.layout_key(&qname),
            expected_schema_hash: entry.expected_schema_hash,
        })
    }

    /// Resolve a [`FieldType::Nested`] (possibly wrapped in arrays) to the
    /// qualified-name string the walk may DESCEND under, or `None`.
    ///
    /// TWO steps, each identity-faithful (the last site of the
    /// identity-collapse class):
    ///
    /// 1. **Bind by IDENTITY, exactly as `resolve_fixed_nested` does** — a
    ///    QUALIFIED ref is an exact `by_key` lookup with NO fallthrough on
    ///    a miss (a walk that matched the flattened STRING would take a
    ///    qualified ref core leaves UNRESOLVED — e.g. `pkg/Type` where
    ///    only a slash-named YAML `(None, "pkg/Type")` exists — and happily
    ///    descend that same-string different-identity layout during
    ///    tolerance path walking); an unqualified ref goes same-package
    ///    (the parent's package, read off the parent STRING's winner —
    ///    the only identity a string-addressed walk can be standing in) →
    ///    bare `Header` → unambiguous global bare.
    ///
    /// 2. **Materialize through the string-winner gate** — `layout_of` is
    ///    string-addressed and serves each string's later-wins winner, so
    ///    the bound identity is walkable IFF it IS its own string's
    ///    winner; otherwise refuse (`None`): serving any layout for a ref
    ///    bound to a different identity would validate paths against
    ///    bytes the producer never wrote. This is stronger than a
    ///    count-based `addressable` gate: a same-KEY duplicate's winner
    ///    passes (core and the string maps agree on it), a cross-key
    ///    same-string pair refuses whichever side is bound.
    fn nested_element_qname(&self, ft: &FieldType, parent_qname: &str) -> Option<String> {
        let (schema_name, package) = innermost_nested(ft)?;
        let bound: Option<usize> = if let Some(pkg) = package {
            // Qualified: exact lookup ONLY (core's rule — no fallthrough).
            self.identity
                .by_key
                .get(&(Some(pkg.to_string()), schema_name.to_string()))
                .copied()
        } else {
            // Same package as the parent — the parent's IDENTITY is its
            // string's winner (a string-addressed walk descended into it
            // through that winner's layout).
            let parent_package: Option<Option<String>> = self
                .identity
                .string_winner
                .get(parent_qname)
                .map(|&pi| self.schemas[pi].package.clone());
            parent_package
                .and_then(|ppkg| {
                    self.identity
                        .by_key
                        .get(&(ppkg, schema_name.to_string()))
                        .copied()
                })
                .or_else(|| {
                    // Bare Header → std_msgs (rosidl legacy special case).
                    (schema_name == "Header")
                        .then(|| {
                            self.identity
                                .by_key
                                .get(&(Some("std_msgs".to_string()), "Header".to_string()))
                                .copied()
                        })
                        .flatten()
                })
                .or_else(|| {
                    // Unambiguous global bare — the FLAT, key-deduped rule
                    // (rounds 8+10): one identity is one candidate; a name
                    // borne by two identities is AMBIGUOUS, never guessed.
                    self.identity
                        .nested_bare_index
                        .get(schema_name)
                        .copied()
                        .flatten()
                })
        };
        let idx = bound?;
        let qname = self.schemas[idx].qualified_name();
        (self.identity.string_winner.get(&qname) == Some(&idx)).then_some(qname)
    }
}

impl FieldResolver for FieldRegistry {
    fn topics(&self) -> Vec<String> {
        self.topics.iter().map(|(t, _)| t.clone()).collect()
    }

    fn resolve_field(&self, topic: &str, path: &str) -> Result<(), FieldResolveError> {
        let qname = self
            .topics
            .iter()
            .find(|(t, _)| t == topic)
            .and_then(|(_, e)| e.schema_qname.clone());
        let root = match qname {
            Some(q) => q,
            None => {
                return Err(FieldResolveError {
                    segment: String::new(),
                    candidates: vec![],
                    schema_unavailable: true,
                })
            }
        };
        // `LayoutResolver::layout_of` is `&mut self` (it memoizes); the trait
        // method is `&self`, so the walk builds a scratch resolver from the
        // shared schema set. Cold path (validation runs once at launch over a
        // handful of paths).
        self.resolve_path_immut(&self.layout_key(&root), path)
    }

    /// The top-level field names of `topic`'s root schema (fixed ∪
    /// variable, declaration order) — the domain a topic-wide/default metric
    /// expands over. `None` when the topic is external or its schema is
    /// unavailable (the validator treats an EXPLICIT non-`bit_exact` topic-wide
    /// metric on such a topic as exit-4; a global default just leaves it
    /// byte-exact). Cold path (validation only).
    fn topic_field_names(&self, topic: &str) -> Option<Vec<String>> {
        let qname = self
            .topics
            .iter()
            .find(|(t, _)| t == topic)
            .and_then(|(_, e)| e.schema_qname.clone())?;
        let (mut resolver, _) = LayoutResolver::new(self.schemas.clone());
        let layout = resolver.layout_of(&self.layout_key(&qname))?;
        Some(field_names(&layout))
    }

    fn check_field_metric(
        &self,
        topic: &str,
        path: &str,
        metric: &MetricKind,
    ) -> Result<(), String> {
        // `bit_exact` is legal on ANY resolvable field — byte compare needs no
        // decode, and the diff engine folds it into the byte-exact remainder.
        if metric.is_bit_exact() {
            return Ok(());
        }
        // A DOTTED (nested / per-element) path has no individually-addressable
        // wire span in the metric decoder — its elements are publisher-opaque
        // bytes (a `DynamicArray<Nested>` carries no framework per-element
        // encoding). Only a top-level decodable field carries a numeric metric.
        if path.contains('.') {
            return Err(format!(
                "the metric '{}' needs a metric-decodable field, but '{path}' is a NESTED / \
                 per-element path whose elements are publisher-opaque bytes; only bit_exact \
                 applies — target a top-level decodable field (e.g. a parallel float64[] array) \
                 or use bit_exact",
                metric.label()
            ));
        }
        // Single top-level segment: classify its wire field type.
        let ft = self.top_level_field_type(topic, path).ok_or_else(|| {
            // resolve_field already succeeded, so this is defensive only.
            format!("field '{path}' resolved but its wire layout could not be classified")
        })?;
        let opaque_reason = || {
            format!(
                "the metric '{}' needs a metric-decodable field, but '{path}' resolves to a \
                 publisher-opaque / non-numeric field ({}); only bit_exact applies — use a \
                 decodable layout (e.g. parallel float64 arrays) or bit_exact",
                metric.label(),
                describe_field_type(&ft)
            )
        };
        match metric {
            MetricKind::BboxIou { .. } => {
                // bbox needs a flattened [x,y,w,h]*N float64 SEQUENCE (an array),
                // never a lone scalar.
                if is_f64_array(&ft) {
                    Ok(())
                } else {
                    Err(format!(
                        "the metric 'bbox_iou' needs a float64[] field of flattened [x,y,w,h] \
                         boxes, but '{path}' is {}; use a float64[] layout or bit_exact",
                        describe_field_type(&ft)
                    ))
                }
            }
            // max_rel / rmse on a 64-bit INTEGER field (I64/U64) is refused
            // (exit 4). These metrics compute in the f64 domain, where a value
            // above 2^53 (e.g. a nanosecond timestamp) loses its low bits — a real
            // divergence can collapse to 0.0 and FALSE-PASS. max_abs (exact integer
            // domain) and bit_exact stay available.
            MetricKind::MaxRel { .. } | MetricKind::Rmse { .. } if is_64bit_int(&ft) => Err(format!(
                "the metric '{}' is precision-lossy on 64-bit integers (field '{path}' is {}); a \
                 value above 2^53 cannot be represented exactly in the f64 metric domain, so a real \
                 divergence can silently collapse to zero — use max_abs (compared in exact integer \
                 domain) or bit_exact",
                metric.label(),
                describe_field_type(&ft)
            )),
            // max_abs / max_rel / rmse / set_* / ordered_list_equal — any
            // metric-decodable numeric field (fixed scalar OR float64 array).
            _ => {
                if is_numeric_decodable(&ft) {
                    Ok(())
                } else {
                    Err(opaque_reason())
                }
            }
        }
    }
}

impl FieldRegistry {
    /// The wire [`FieldType`] of a TOP-LEVEL field `seg` in `topic`'s root
    /// schema (fixed OR variable), or `None` when the topic's schema is
    /// unavailable or the field is absent (a caller that already resolved the
    /// path treats `None` as defensive). Cold path (validation only).
    fn top_level_field_type(&self, topic: &str, seg: &str) -> Option<FieldType> {
        let qname = self
            .topics
            .iter()
            .find(|(t, _)| t == topic)
            .and_then(|(_, e)| e.schema_qname.clone())?;
        let (mut resolver, _) = LayoutResolver::new(self.schemas.clone());
        let layout = resolver.layout_of(&self.layout_key(&qname))?;
        field_type_of(&layout, seg)
    }
}

impl FieldRegistry {
    /// `&self` schema-tree walk for [`FieldResolver::resolve_field`]: builds a
    /// scratch [`LayoutResolver`] from the shared set (whose `layout_of` needs
    /// `&mut`) and descends segment by segment through nested schemas. Cold
    /// path, so the per-call resolver build is negligible.
    fn resolve_path_immut(
        &self,
        root_qname: &str,
        field_path: &str,
    ) -> Result<(), FieldResolveError> {
        let segments: Vec<&str> = field_path.split('.').collect();
        let mut current_qname = root_qname.to_string();
        let (mut resolver, _) = LayoutResolver::new(self.schemas.clone());
        for (i, seg) in segments.iter().enumerate() {
            let layout = match resolver.layout_of(&current_qname) {
                Some(l) => l,
                None => {
                    return Err(FieldResolveError {
                        segment: seg.to_string(),
                        candidates: vec![],
                        schema_unavailable: true,
                    })
                }
            };
            let field_type = match field_type_of(&layout, seg) {
                Some(ft) => ft,
                None => {
                    return Err(FieldResolveError {
                        segment: seg.to_string(),
                        candidates: field_names(&layout),
                        schema_unavailable: false,
                    })
                }
            };
            if i + 1 == segments.len() {
                return Ok(());
            }
            let parent_qname = current_qname.clone();
            match self.nested_element_qname(&field_type, &parent_qname) {
                Some(q) => current_qname = q,
                None => {
                    return Err(FieldResolveError {
                        segment: segments[i + 1].to_string(),
                        candidates: vec![],
                        schema_unavailable: false,
                    })
                }
            }
        }
        Ok(())
    }
}

/// The read-only decoder over one topic's root schema. Owns its
/// [`LayoutResolver`] (a clone of the registry's schema set) so decode is
/// self-contained (`&self` on [`FieldRegistry::decoder_for`]).
pub struct FrameDecoder {
    layouts: LayoutResolver,
    root_qname: String,
    /// The expected recipe-3 `schema_hash` of `root_qname`'s resolved layout.
    /// Every [`FrameDecoder::locate`] gates the frame's
    /// `WireHeader.schema_hash` against this (mismatch =
    /// [`DecodeError::SchemaHashMismatch`]). `None` when the topic's schema
    /// hash could not be computed — the gate is then skipped.
    expected_schema_hash: Option<u64>,
}

/// A located field within a frame (the metric decoder interprets the bytes per its metric).
#[derive(Debug, Clone, PartialEq)]
pub struct LocatedField<'f> {
    /// The raw bytes of the field within the frame.
    pub bytes: &'f [u8],
    /// The field's schema type (drives numeric interpretation).
    pub field_type: FieldType,
    /// `true` for a fixed-section field, `false` for a variable (offset-table)
    /// field.
    pub is_fixed: bool,
}

/// Why a frame field could not be decoded (a corrupt bag → loud error, never UB).
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum DecodeError {
    /// The frame is shorter than the 32-byte wire header.
    #[error("frame is {len} bytes, shorter than the 32-byte wire header")]
    FrameTooShort {
        /// The frame length.
        len: usize,
    },
    /// The top-level field name is not in the root schema.
    #[error("field '{field}' is not in schema '{schema}'")]
    FieldNotFound {
        /// The offending field name.
        field: String,
        /// The schema searched.
        schema: String,
    },
    /// A fixed-section field's `[offset, offset+size)` span exceeds the payload.
    #[error(
        "fixed field '{field}' span [{offset}, {end}) exceeds the {payload_len}-byte payload \
         (corrupt frame)"
    )]
    FixedOutOfBounds {
        /// The field name.
        field: String,
        /// The field's byte offset in the payload.
        offset: usize,
        /// The field's end offset (`offset + size`).
        end: usize,
        /// The payload length.
        payload_len: usize,
    },
    /// A variable field's offset-table entry points outside the payload.
    #[error(
        "variable field '{field}' offset-table entry [{offset}, {end}) exceeds the \
         {payload_len}-byte payload (corrupt offset table)"
    )]
    VariableOutOfBounds {
        /// The field name.
        field: String,
        /// The entry's byte offset in the payload.
        offset: usize,
        /// The entry's end offset (`offset + length`).
        end: usize,
        /// The payload length.
        payload_len: usize,
    },
    /// A variable field's 8-byte offset-table entry is not fully present in the
    /// payload (the frame is truncated at/within the offset table). Distinct
    /// from [`Self::VariableOutOfBounds`], where the entry IS present but points
    /// past the payload: here the entry SPAN itself is missing, so
    /// [`cerulion_core::shm_runtime::read_offset_entry`] would silently return
    /// the `(0, 0)` sentinel and a naive `end > payload.len()` check would pass
    /// vacuously — decoding the field as EMPTY instead of erroring.
    #[error(
        "variable field '{field}' offset-table entry (index {idx}) needs the payload to be at \
         least {needed} bytes but it is only {payload_len} — the frame is truncated within the \
         offset table (corrupt frame)"
    )]
    OffsetTableTruncated {
        /// The field name.
        field: String,
        /// The variable-field index whose 8-byte entry is truncated.
        idx: usize,
        /// The payload length required for the entry to be fully present
        /// (`fixed_size + 8 * (idx + 1)`).
        needed: usize,
        /// The actual payload length.
        payload_len: usize,
    },
    /// The recorded frame's [`WireHeader::schema_hash`] does not match the hash
    /// of the layout the registry resolved for its topic. Applying the current
    /// workspace layout to bytes recorded under a different (even
    /// compatible-size) schema would silently misread fields, so decode refuses
    /// loudly.
    #[error(
        "schema-hash mismatch: frame stamped 0x{actual:016x} but the topic's resolved layout \
         hashes 0x{expected:016x} (the recorded schema differs from the current workspace \
         schema — decode refused to avoid a silent misread)"
    )]
    SchemaHashMismatch {
        /// The layout hash the registry resolved for the topic's schema.
        expected: u64,
        /// The `schema_hash` stamped in the recorded frame's wire header.
        actual: u64,
    },
    /// The requested schema layout is not resolvable.
    #[error("schema '{schema}' is not resolvable for decode")]
    SchemaUnavailable {
        /// The schema name.
        schema: String,
    },
    /// A variable sequence-of-nested field carries publisher-defined opaque
    /// bytes with no framework per-element encoding — per-element decode is
    /// not implemented. The field's RAW bytes are still locatable via
    /// [`FrameDecoder::locate`].
    #[error(
        "variable nested field '{field}' has no framework-defined per-element encoding \
         (opaque publisher bytes); per-element decode is not implemented"
    )]
    UndecodableVariableNested {
        /// The field name.
        field: String,
    },
}

impl FrameDecoder {
    /// Locate a TOP-LEVEL field by name within `frame` (`[WireHeader][payload]`),
    /// bounds-checking every span. Fixed fields resolve via the computed layout
    /// offset; variable fields via the offset table. Nested-leaf paths and
    /// per-element array decode are not supported (this locates top-level fields only — the
    /// primitive/`StringFixed`/`String`/`Bytes`/`DynamicArray<primitive>` set,
    /// plus the raw bytes of a complex variable field).
    pub fn locate<'f>(
        &mut self,
        frame: &'f [u8],
        field: &str,
    ) -> Result<LocatedField<'f>, DecodeError> {
        if frame.len() < WireHeader::SIZE {
            return Err(DecodeError::FrameTooShort { len: frame.len() });
        }
        // Gate the recorded frame's schema_hash against the
        // layout the registry resolved for this topic BEFORE trusting any
        // offset. A compatible-size layout drift (same wire size, reordered /
        // retyped fields) would otherwise be silently misread as the current
        // schema. `read_from_buf` cannot fail here (length checked above).
        if let Some(expected) = self.expected_schema_hash {
            let header = WireHeader::read_from_buf(frame)
                .expect("frame.len() >= WireHeader::SIZE checked immediately above");
            if header.schema_hash != expected {
                return Err(DecodeError::SchemaHashMismatch {
                    expected,
                    actual: header.schema_hash,
                });
            }
        }
        let payload = &frame[WireHeader::SIZE..];
        let layout = self.layouts.layout_of(&self.root_qname).ok_or_else(|| {
            DecodeError::SchemaUnavailable {
                schema: self.root_qname.clone(),
            }
        })?;

        // Fixed field?
        if let Some(fl) = layout.fixed_fields.iter().find(|f| f.name == field) {
            let end = fl.offset + fl.size;
            if end > payload.len() {
                return Err(DecodeError::FixedOutOfBounds {
                    field: field.to_string(),
                    offset: fl.offset,
                    end,
                    payload_len: payload.len(),
                });
            }
            return Ok(LocatedField {
                bytes: &payload[fl.offset..end],
                field_type: fl.field_type.clone(),
                is_fixed: true,
            });
        }

        // Variable field?
        if let Some(idx) = layout.variable_fields.iter().position(|f| f.name == field) {
            // The 8-byte offset-table entry must be FULLY
            // present before we trust it. `read_offset_entry` returns the
            // `(0, 0)` sentinel when the entry span is truncated, and the
            // `end > payload.len()` check below would then pass vacuously
            // (`0 > len` is false) — silently decoding the field as EMPTY. An
            // entry that IS present but reads `(0, 0)` is a GENUINELY-empty
            // variable field (falls through to an empty slice, `Ok`); the
            // discriminator is exactly whether the entry SPAN fits.
            let needed = layout.fixed_size + 8 * (idx + 1);
            if payload.len() < needed {
                return Err(DecodeError::OffsetTableTruncated {
                    field: field.to_string(),
                    idx,
                    needed,
                    payload_len: payload.len(),
                });
            }
            let (off, len) =
                cerulion_core::shm_runtime::read_offset_entry(payload, layout.fixed_size, idx);
            let (off, len) = (off as usize, len as usize);
            let end = off.saturating_add(len);
            if end > payload.len() {
                return Err(DecodeError::VariableOutOfBounds {
                    field: field.to_string(),
                    offset: off,
                    end,
                    payload_len: payload.len(),
                });
            }
            return Ok(LocatedField {
                bytes: &payload[off..end],
                field_type: layout.variable_fields[idx].field_type.clone(),
                is_fixed: false,
            });
        }

        Err(DecodeError::FieldNotFound {
            field: field.to_string(),
            schema: self.root_qname.clone(),
        })
    }

    /// Decode a top-level `DynamicArray<f64>` variable field into its element
    /// values (per-element typed decode). Errors if the field is not an
    /// `f64` sequence or the bytes are not a whole multiple of 8. A concrete
    /// per-element decoder demonstrating the typed path the metric decoder generalizes.
    pub fn decode_f64_sequence(
        &mut self,
        frame: &[u8],
        field: &str,
    ) -> Result<Vec<f64>, DecodeError> {
        let located = self.locate(frame, field)?;
        let is_f64_seq = matches!(
            &located.field_type,
            FieldType::DynamicArray { element_type } if **element_type == FieldType::F64
        );
        if !is_f64_seq {
            return Err(DecodeError::UndecodableVariableNested {
                field: field.to_string(),
            });
        }
        // A partial-write / corrupt frame with a non-multiple-of-8 length is a
        // loud error, not a silent truncation.
        if !located.bytes.len().is_multiple_of(8) {
            return Err(DecodeError::VariableOutOfBounds {
                field: field.to_string(),
                offset: 0,
                end: located.bytes.len(),
                payload_len: located.bytes.len(),
            });
        }
        Ok(located
            .bytes
            .as_chunks::<8>()
            .0
            .iter()
            .map(|c| f64::from_le_bytes(*c))
            .collect())
    }

    /// The resolved root [`WireLayout`] for this decoder's topic, or `None` when
    /// the schema is unavailable. The tolerant diff uses it to compute the
    /// byte-exact REMAINDER (the fixed-section spans of the overridden fields it
    /// must EXCLUDE from the byte compare). `&mut self` — `layout_of` memoizes.
    pub fn root_layout(&mut self) -> Option<WireLayout> {
        self.layouts.layout_of(&self.root_qname)
    }

    /// Tolerant diff: decode a metric-decodable TOP-LEVEL field into its `f64`
    /// element sequence — a fixed numeric scalar decodes to a ONE-element
    /// sequence (its value widened to `f64`), a `float64[]` / `float64[N]` array
    /// to its elements. Errors ([`DecodeError::UndecodableVariableNested`]) if
    /// the field is not metric-decodable (String/Bytes/nested/non-f64 array) —
    /// but validation ([`FieldRegistry::check_field_metric`]) already refused a
    /// non-`bit_exact` metric on such a field at exit 4, so a live call here is
    /// always on a decodable field. Every span is bounds-checked via
    /// [`Self::locate`] (a corrupt frame → a loud [`DecodeError`], never UB).
    pub fn decode_field_f64(&mut self, frame: &[u8], field: &str) -> Result<Vec<f64>, DecodeError> {
        let located = self.locate(frame, field)?;
        if is_f64_array(&located.field_type) {
            // f64 sequence (fixed inline array OR variable float64[]): the bytes
            // must be a whole multiple of 8 (a partial write is loud, not
            // silently truncated).
            if !located.bytes.len().is_multiple_of(8) {
                return Err(DecodeError::VariableOutOfBounds {
                    field: field.to_string(),
                    offset: 0,
                    end: located.bytes.len(),
                    payload_len: located.bytes.len(),
                });
            }
            return Ok(located
                .bytes
                .as_chunks::<8>()
                .0
                .iter()
                .map(|c| f64::from_le_bytes(*c))
                .collect());
        }
        if let Some(v) = decode_numeric_scalar(&located.field_type, located.bytes) {
            return Ok(vec![v]);
        }
        Err(DecodeError::UndecodableVariableNested {
            field: field.to_string(),
        })
    }

    /// Decode an `I64`/`U64` SCALAR field into a one-element `i128`
    /// sequence — the EXACT integer domain the tolerance engine compares 64-bit
    /// integer fields in (never the lossy `f64` widening of [`Self::decode_field_f64`],
    /// which collapses two distinct values above 2^53 — nanosecond timestamps —
    /// into one). Errors ([`DecodeError::UndecodableVariableNested`]) if the field
    /// is not an `I64`/`U64` scalar; the engine only routes here for fields the
    /// plan classified 64-bit-int (max_rel/rmse were already refused at exit 4),
    /// so a mismatch is defensive. Bounds-checked via [`Self::locate`].
    pub fn decode_field_i128(
        &mut self,
        frame: &[u8],
        field: &str,
    ) -> Result<Vec<i128>, DecodeError> {
        let located = self.locate(frame, field)?;
        let bytes = located.bytes;
        let val: Option<i128> = match &located.field_type {
            FieldType::I64 => bytes
                .get(..8)
                .map(|b| i64::from_le_bytes(b.try_into().unwrap()) as i128),
            FieldType::U64 => bytes
                .get(..8)
                .map(|b| u64::from_le_bytes(b.try_into().unwrap()) as i128),
            _ => None,
        };
        val.map(|v| vec![v])
            .ok_or(DecodeError::UndecodableVariableNested {
                field: field.to_string(),
            })
    }
}

/// Decode a fixed numeric scalar field's bytes into an `f64` (widening every
/// integer/bool/float primitive). `None` for a non-scalar type or a
/// wrong-length span. LE, matching the wire encoding.
fn decode_numeric_scalar(ft: &FieldType, bytes: &[u8]) -> Option<f64> {
    fn arr<const N: usize>(b: &[u8]) -> Option<[u8; N]> {
        b.get(..N)?.try_into().ok()
    }
    match ft {
        FieldType::Bool => bytes.first().map(|&b| if b != 0 { 1.0 } else { 0.0 }),
        FieldType::I8 => bytes.first().map(|&b| b as i8 as f64),
        FieldType::U8 => bytes.first().map(|&b| b as f64),
        FieldType::I16 => arr::<2>(bytes).map(|a| i16::from_le_bytes(a) as f64),
        FieldType::U16 => arr::<2>(bytes).map(|a| u16::from_le_bytes(a) as f64),
        FieldType::I32 => arr::<4>(bytes).map(|a| i32::from_le_bytes(a) as f64),
        FieldType::U32 => arr::<4>(bytes).map(|a| u32::from_le_bytes(a) as f64),
        FieldType::I64 => arr::<8>(bytes).map(|a| i64::from_le_bytes(a) as f64),
        FieldType::U64 => arr::<8>(bytes).map(|a| u64::from_le_bytes(a) as f64),
        FieldType::F32 => arr::<4>(bytes).map(|a| f32::from_le_bytes(a) as f64),
        FieldType::F64 => arr::<8>(bytes).map(f64::from_le_bytes),
        _ => None,
    }
}

/// `true` if `ft` is a fixed numeric scalar (decodes to a one-element `f64`
/// sequence): a bool/int/float primitive.
fn is_numeric_scalar(ft: &FieldType) -> bool {
    matches!(
        ft,
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

/// `true` if `ft` is a 64-bit INTEGER scalar (`I64`/`U64`) — the fields the
/// tolerance engine compares in EXACT integer domain, and on which
/// `max_rel`/`rmse` are refused as precision-lossy above 2^53.
fn is_64bit_int(ft: &FieldType) -> bool {
    matches!(ft, FieldType::I64 | FieldType::U64)
}

/// `true` if `ft` is a `float64[]` / `float64[N]` array (the metric-decodable
/// sequence form — the metric decoder decodes 8-byte `f64` chunks).
fn is_f64_array(ft: &FieldType) -> bool {
    matches!(
        ft,
        FieldType::DynamicArray { element_type } | FieldType::FixedArray { element_type, .. }
            if **element_type == FieldType::F64
    )
}

/// `true` if `ft` is metric-decodable to an `f64` sequence: a fixed numeric
/// scalar OR a `float64` array. Everything else (String/Bytes/StringFixed,
/// nested, array-of-nested, array-of-non-f64) is publisher-opaque for the
/// metric decoder and takes only `bit_exact`.
fn is_numeric_decodable(ft: &FieldType) -> bool {
    is_numeric_scalar(ft) || is_f64_array(ft)
}

/// A short human label for a [`FieldType`] used in the exit-4 classification
/// message (never `Debug`, which is not a stability contract).
fn describe_field_type(ft: &FieldType) -> String {
    match ft {
        FieldType::String => "a variable-length string".to_string(),
        FieldType::Bytes => "an opaque byte array".to_string(),
        FieldType::StringFixed(_) => "a fixed-length string".to_string(),
        FieldType::Nested { .. } => "a nested message".to_string(),
        FieldType::FixedArray { element_type, .. } | FieldType::DynamicArray { element_type } => {
            format!("an array of {}", describe_field_type(element_type))
        }
        other if is_numeric_scalar(other) => "a numeric scalar".to_string(),
        _ => "a non-numeric field".to_string(),
    }
}

/// A field type looked up by name in a [`WireLayout`] (fixed OR variable).
fn field_type_of(layout: &WireLayout, name: &str) -> Option<FieldType> {
    layout
        .fixed_fields
        .iter()
        .find(|f| f.name == name)
        .map(|f| f.field_type.clone())
        .or_else(|| {
            layout
                .variable_fields
                .iter()
                .find(|f| f.name == name)
                .map(|f| f.field_type.clone())
        })
}

/// Every field name in a [`WireLayout`] (fixed then variable, declaration
/// order) — suggestion candidates.
fn field_names(layout: &WireLayout) -> Vec<String> {
    layout
        .fixed_fields
        .iter()
        .map(|f| f.name.clone())
        .chain(layout.variable_fields.iter().map(|f| f.name.clone()))
        .collect()
}

/// Peel `FixedArray`/`DynamicArray` wrappers off `ft` to its innermost
/// `Nested { schema_name, package }`, or `None` if the innermost is not nested.
fn innermost_nested(ft: &FieldType) -> Option<(&str, Option<&str>)> {
    match ft {
        FieldType::Nested {
            schema_name,
            package,
            ..
        } => Some((schema_name.as_str(), package.as_deref())),
        FieldType::FixedArray { element_type, .. } | FieldType::DynamicArray { element_type } => {
            innermost_nested(element_type)
        }
        _ => None,
    }
}

/// Resolve an output `schema:` string to a qualified name in the combined set.
/// The graph normalizes `schema:` to `/`-separated (`sensor_msgs/Image`); a
/// package-less workspace schema is a bare name. Returns `None` if not present.
///
/// A QUALIFIED name (one carrying `/`) is looked up in the combined set's
/// name strings; a BARE name is answered by the tiered `bare_index` ALONE:
/// the index is total over every bare name the workspace can
/// claim (built-in short names, parsed store bare names, every YAML entry
/// name and every file stem), and it is where the ladder's verdicts live —
/// the stem's authority over a same-named entry from another file, the
/// ambiguity refusals, the masking of a scrubbed claim. Consulting the flat
/// name set first bypassed all of that for any bare name some YAML entry
/// happened to bear: a port `Goal` beside `Goal.yaml` (declaring only
/// `Other`) and an unrelated file declaring an entry `Goal` replayed with
/// the UNRELATED entry, the exact document the resolver's stem-first
/// precedence rejects.
fn resolve_schema_name(
    schema: &str,
    qualified_set: &std::collections::HashSet<&str>,
    bare_index: &HashMap<String, Option<String>>,
    refused: &std::collections::HashSet<String>,
) -> Option<String> {
    if schema.is_empty() {
        return None;
    }
    // NORMALIZE first (the same normalization serving applies):
    // a recorded `pkg::Type` is the documented alternate
    // spelling; the refused set, the qualified set and every served key are
    // spelled `pkg/Type`.
    let schema = crate::schema_cmd::normalize_schema(schema);
    let schema = schema.as_str();
    // A spelling the workspace tier refuses —
    // either spelling (a slash-named entry declared by several files is
    // refused too) — resolves NOTHING: never the string maps' last-wins
    // winner.
    if refused.contains(schema) {
        return None;
    }
    if schema.contains('/') {
        return qualified_set.contains(schema).then(|| schema.to_string());
    }
    // Bare name → the ladder's verdict: an unambiguous binding, or nothing.
    match bare_index.get(schema) {
        Some(Some(q)) => Some(q.clone()),
        _ => None,
    }
}

/// Index every schema's codegen-faithful recipe-3 `schema_hash` by qualified
/// name. Clones the set, runs `resolve_fixed_nested` (the same phase codegen
/// runs before computing `ShmMessage::SCHEMA_HASH`), then hashes each resolved
/// schema. The input MUST be the FULL combined workspace + built-in set so
/// nested references resolve exactly as codegen resolves them (mirrors
/// `graph_cmd::build_workspace_schema_hashes`, minus the closure-escape skip
/// that only applies to a workspace-only set).
///
/// Also returns a [`crate::schema_cmd::RemovedDefinitions`] for the
/// schemas whose POST-RESOLVE fixed section overflows (dropped from the
/// hash index, loudly warned), under the shared rounds-5/6/10 removal
/// policy ([`crate::schema_cmd::remove_failing_definitions`]):
/// index-precise; a failing later-wins WINNER sweeps every bearer of its
/// name (returned as a fully-removed NAME); a failing definition whose
/// flattened name survives via a different-identity winner is a PARTIAL
/// removal returned by resolver KEY. The caller's tier scrub
/// applies BOTH halves via `RemovedDefinitions::removes`, so it can
/// neither evict a valid surviving winner nor leave a rejected
/// definition behind in the registry's schema set for resolution to
/// panic on. The
/// caller MUST scrub those from every sibling structure built over the same
/// set AND recompute the hashes over the scrubbed set (a drop can flip a
/// bare reference's resolution, changing a survivor's layout — `from_graph`'s
/// fixpoint loop) — a composed-overflow schema left reachable through the
/// registry's schema vec or bare index panics later when
/// `decoder_for`/`resolve_field` materialize its layout.
///
/// PRECONDITION: the input has been through
/// [`crate::schema_cmd::retain_composed_sizable_schemas`] — a recursively-
/// FIXED composed-overflow schema still in the set panics inside
/// `resolve_fixed_nested` itself when anything references it (the
/// preflight is what makes the call below safe).
fn compute_schema_hashes(
    schemas: &[MessageSchema],
) -> (HashMap<String, u64>, crate::schema_cmd::RemovedDefinitions) {
    let mut resolved = schemas.to_vec();
    // Resolution warnings are advisory (unresolvable/ambiguous/cyclic refs);
    // the layout + hash still reflect what codegen would emit for the frame's
    // producing schema.
    let _ = cerulion_core::codegen::resolve_fixed_nested(&mut resolved);
    // Individually-sizable schemas can compose an overflow through
    // fixed-nested inlining, and `schema_hash` panics on one — collect the
    // failing definitions INDEX-precisely, loudly. The ONE u32
    // wire ceiling is the second arm of the same verdict — a resolved
    // fixed section past `u32::MAX` (a `Huge` root at 2^64 − 8 bytes, or a
    // representable set composed past it) can never be a real frame, and
    // `layout_of`'s UNCHECKED header/offset-table terms overflow on the
    // declared-huge shape, so a `Huge`-typed produced topic's
    // `decoder_for` + `root_layout` would reach a PANIC (debug) or a
    // silent wrap (release) instead of degrading to schema-unavailable.
    // Same removal policy, same winner sweep, same fixpoint.
    // Both verdicts are the shared predicates' (ONE message site each);
    // this leg only collects the failing indices. The ceiling is judged on
    // the RESOLVED schema, so its offset table is counted exactly (the
    // header and the table are part of the frame the ceiling bounds).
    const CONTEXT: &str = "replay field registry (resolved)";
    let failing: std::collections::BTreeSet<usize> = resolved
        .iter()
        .enumerate()
        .filter_map(|(idx, s)| {
            let representable = crate::schema_cmd::sizable_or_warn(s, CONTEXT)
                && crate::schema_cmd::wire_representable_or_warn(
                    s,
                    CONTEXT,
                    crate::schema_cmd::SizeMoment::Resolved,
                );
            (!representable).then_some(idx)
        })
        .collect();
    // The duplicate-qualified-name eviction/panic class: the
    // ONE shared removal policy — index-precise, hostile-WINNER sweeps
    // every bearer, and the returned names are exactly those FULLY removed.
    // Suppressing any name with a surviving bearer is
    // right for a dropped shadowed LOSER (the valid winner keeps serving)
    // but MASKS a dropped later-wins WINNER whose earlier bearer
    // survives: `from_graph` never scrubs the name, the registry's own
    // schema set keeps the failing winner, and `decoder_for`'s `layout_of`
    // later materializes it and PANICS instead of degrading.
    // The winner sweep makes the name fully absent
    // instead — and keeps the local hash collect from serving the shadowed
    // loser's hash under a name whose real binding was rejected.
    // The identity source is the PRE-resolution set (`schemas`), not
    // `resolved`: `from_graph`'s fixpoint applies `RemovedDefinitions::removes`
    // to its UNRESOLVED tiers, and a definition carrying a fixed-nested
    // reference hashes differently once `resolve_fixed_nested` has inlined it
    // — recording the resolved hash would make the definition arm compare two
    // different states of one schema and match NOTHING. With the two other
    // arms missing as well (a store definition sharing a `(package, name)`
    // key with a built-in while a slash-named YAML twin holds the same
    // qualified string is neither fully removed nor key-unique), the scrub
    // would remove nothing, the next pass rebuild an identical set, and the
    // fixpoint SPIN FOREVER on a workspace holding one hostile-but-parseable
    // definition — reachable from a `.msg` the ladder acquired over
    // the wire. `resolve_fixed_nested` mutates in place without reordering,
    // so index `idx` is the same definition in both.
    let unsizable = crate::schema_cmd::remove_failing_definitions(
        &mut resolved,
        Some(schemas),
        &failing,
        "replay field registry (resolved)",
    );
    let hashes = resolved
        .iter()
        .map(|s| {
            (
                crate::schema_cmd::normalize_schema(&s.qualified_name()),
                s.schema_hash(),
            )
        })
        .collect();
    (hashes, unsizable)
}

#[cfg(test)]
mod tests {
    use super::*;
    use cerulion_core::codegen::FieldDef;

    // Hand-built schema sets + frames (never a self-compare — every assertion
    // is against a hand-computed offset/byte oracle).

    /// A registry over a hand-built schema set, one produced topic → root
    /// schema. Bypasses `from_graph` (no workspace/graph needed) so the decode +
    /// resolution logic is tested hermetically.
    fn registry_over(schemas: Vec<MessageSchema>, topic: &str, root_qname: &str) -> FieldRegistry {
        // The ONE identity-index builder `from_graph` uses —
        // the hermetic constructor cannot drift from production.
        let identity = build_identity_indexes(&schemas);
        // Compute the codegen-faithful hash the same way `from_graph` does, so
        // the decode-path tests must stamp a realistic wire header (the
        // schema-hash gate is exercised by every `locate`).
        let (hash_by_qname, _unsizable) = compute_schema_hashes(&schemas);
        let expected_schema_hash = hash_by_qname.get(root_qname).copied();
        let layout_keys = schemas
            .iter()
            .map(|s| {
                let declared = s.qualified_name();
                (crate::schema_cmd::normalize_schema(&declared), declared)
            })
            .collect();
        FieldRegistry {
            schemas,
            identity,
            layout_keys,
            topics: vec![(
                topic.to_string(),
                TopicEntry {
                    class: TopicClass::Produced,
                    schema_qname: Some(root_qname.to_string()),
                    expected_schema_hash,
                },
            )],
        }
    }

    /// Stamp `frame`'s wire header with `schema_qname`'s codegen-faithful hash
    /// so the schema-hash gate passes (the decode-path tests build frames
    /// with an otherwise-zeroed header). `frame` must be at least
    /// `WireHeader::SIZE` bytes.
    fn stamp_hash(frame: &mut [u8], schemas: &[MessageSchema], schema_qname: &str) {
        let hash = compute_schema_hashes(schemas)
            .0
            .get(schema_qname)
            .copied()
            .expect("schema present in set");
        WireHeader::with_schema(hash).write_to_buf(&mut frame[..WireHeader::SIZE]);
    }

    /// The post-resolve arm of the shadowed-duplicate eviction
    /// class: `compute_schema_hashes` returns only names with NO surviving
    /// bearer. One qualified name borne twice (two workspace YAML files
    /// declaring the same entry name — same resolver key, later-wins) with
    /// the SHADOWED loser composed-overflowing: the loser survives the
    /// preflight (it is not an active lookup target), gets stamped during
    /// resolution, and is dropped by the post-resolve retain — but its name
    /// must NOT reach the caller, whose NAME-keyed tier scrub would evict
    /// the valid later-wins winner from every sibling structure. A lone
    /// hostile with no same-name survivor IS returned (the control that
    /// keeps the filter from going inert).
    #[test]
    fn r5_a_dropped_shadowed_duplicate_is_not_returned_while_its_winner_survives() {
        let filler = {
            let mut s = MessageSchema::new("Filler3");
            s.add_field(FieldDef::new(
                "v",
                FieldType::FixedArray {
                    element_type: Box::new(FieldType::F64),
                    length: 2305843009213693951,
                },
            ));
            s
        };
        let hostile_dup_fields = |name: &str| {
            let mut s = MessageSchema::new(name);
            for f in ["a", "b"] {
                s.add_field(FieldDef::new(
                    f,
                    FieldType::Nested {
                        schema_name: "Filler3".to_string(),
                        package: None,
                        fixed: None,
                    },
                ));
            }
            s
        };
        // Shadowed hostile loser FIRST, valid winner SECOND (later-wins).
        let loser = hostile_dup_fields("Dup");
        let winner = {
            let mut s = MessageSchema::new("Dup");
            s.add_field(FieldDef::new("a", FieldType::U32));
            s
        };
        // A lone hostile (no same-name survivor) — the anti-inert control.
        // VARIABLE (trailing string), faithful to the real pipeline: the
        // composed preflight drops ACTIVE fixed overflows before this fn
        // ever runs, so the post-resolve retain's genuine catch is the
        // variable-parent composed overflow (its fixed section still
        // overflows through the two inlined Fillers).
        let bomb = {
            let mut s = hostile_dup_fields("Bomb");
            s.add_field(FieldDef::new("s", FieldType::String));
            s
        };

        let (hashes, unsizable) = compute_schema_hashes(&[filler, loser, winner.clone(), bomb]);

        assert!(
            !unsizable.fully_removed_names.contains(&"Dup".to_string()),
            "a name with a SURVIVING later-wins bearer must not be returned \
             — the caller's name-keyed tier scrub would evict the valid \
             winner; got {unsizable:?}"
        );
        assert!(
            unsizable.fully_removed_names.contains(&"Bomb".to_string()),
            "a fully-removed hostile IS returned (the filter must not go \
             inert); got {unsizable:?}"
        );
        // The winner serves the name with ITS hash (hand oracle: a lone
        // resolved clone of the winner).
        let mut winner_clone = vec![winner];
        let _ = cerulion_core::codegen::resolve_fixed_nested(&mut winner_clone);
        assert_eq!(
            hashes.get("Dup").copied(),
            Some(winner_clone[0].schema_hash()),
            "the surviving winner's hash serves the name"
        );
        assert!(
            !hashes.contains_key("Bomb"),
            "the lone hostile is not hashed"
        );
    }

    /// The mirror arm of the unit test above: when the
    /// FAILING bearer is the name's later-wins WINNER (here a VARIABLE
    /// duplicate whose resolved fixed section overflows) while an EARLIER
    /// valid bearer survives, the name must be swept FULLY and RETURNED —
    /// a suppress-any-survivor filter hides exactly this shape from
    /// `from_graph`'s tier scrub, leaving the rejected winner in the
    /// registry's schema set for `layout_of` to panic on, and serves the
    /// shadowed loser's hash under the name.
    #[test]
    fn r6_a_failing_winner_sweeps_its_name_fully_at_the_post_resolve_leg() {
        let filler = {
            let mut s = MessageSchema::new("Filler3");
            s.add_field(FieldDef::new(
                "v",
                FieldType::FixedArray {
                    element_type: Box::new(FieldType::F64),
                    length: 2305843009213693951,
                },
            ));
            s
        };
        // Earlier bearer: valid.
        let loser = {
            let mut s = MessageSchema::new("Dup2");
            s.add_field(FieldDef::new("a", FieldType::U32));
            s
        };
        // Later-wins winner: VARIABLE (string), fixed section overflowing
        // after resolution inlines two Filler3s.
        let winner = {
            let mut s = MessageSchema::new("Dup2");
            for f in ["c", "d"] {
                s.add_field(FieldDef::new(
                    f,
                    FieldType::Nested {
                        schema_name: "Filler3".to_string(),
                        package: None,
                        fixed: None,
                    },
                ));
            }
            s.add_field(FieldDef::new("s", FieldType::String));
            s
        };

        let (hashes, unsizable) = compute_schema_hashes(&[filler, loser, winner]);

        assert!(
            unsizable.fully_removed_names.contains(&"Dup2".to_string()),
            "a FAILING later-wins winner sweeps its name fully and the name \
             IS returned for the caller's tier scrub; got {unsizable:?}"
        );
        assert!(
            !hashes.contains_key("Dup2"),
            "the earlier valid bearer is not resurrected under the name; \
             got {:?}",
            hashes.keys().collect::<Vec<_>>()
        );
    }

    /// Image-like schema: fixed {height@0 u32, width@4 u32, is_bigendian@8 u8,
    /// step@12 u32} (fixed_size 16); variable [encoding: string, data:
    /// bytes]. Matches the layout.rs oracle.
    fn image_schema() -> MessageSchema {
        let mut s = MessageSchema::new_in_package("Img", "test_msgs");
        s.add_field(FieldDef::new("height", FieldType::U32));
        s.add_field(FieldDef::new("width", FieldType::U32));
        s.add_field(FieldDef::new("encoding", FieldType::String));
        s.add_field(FieldDef::new("is_bigendian", FieldType::U8));
        s.add_field(FieldDef::new("step", FieldType::U32));
        s.add_field(FieldDef::new("data", FieldType::Bytes));
        s
    }

    // ── Fixed-field resolution (offset/type against a hand-built schema) ────

    #[test]
    fn fixed_field_resolves_to_offset_and_type() {
        let reg = registry_over(vec![image_schema()], "/cam", "test_msgs/Img");
        // Build a frame: 32-byte header + fixed section (16 B) + offset table
        // (2 × 8 = 16) + variable payload.
        let mut frame = vec![0u8; WireHeader::SIZE];
        // fixed section
        frame.extend_from_slice(&7u32.to_le_bytes()); // height@0
        frame.extend_from_slice(&5u32.to_le_bytes()); // width@4
        frame.push(1u8); // is_bigendian@8
        frame.extend_from_slice(&[0u8; 3]); // pad @9..12
        frame.extend_from_slice(&99u32.to_le_bytes()); // step@12
                                                       // offset table (2 entries) — all zero (no variable data for this test)
        frame.extend_from_slice(&[0u8; 16]);
        stamp_hash(&mut frame, &[image_schema()], "test_msgs/Img");

        let mut dec = reg.decoder_for("/cam").unwrap();
        let h = dec.locate(&frame, "height").unwrap();
        assert!(h.is_fixed);
        assert_eq!(h.field_type, FieldType::U32);
        assert_eq!(u32::from_le_bytes(h.bytes.try_into().unwrap()), 7);
        let step = dec.locate(&frame, "step").unwrap();
        assert_eq!(u32::from_le_bytes(step.bytes.try_into().unwrap()), 99);
        let big = dec.locate(&frame, "is_bigendian").unwrap();
        assert_eq!(big.bytes, &[1u8]);
    }

    #[test]
    fn unknown_field_is_field_not_found() {
        let reg = registry_over(vec![image_schema()], "/cam", "test_msgs/Img");
        let mut frame = vec![0u8; WireHeader::SIZE + 32];
        stamp_hash(&mut frame, &[image_schema()], "test_msgs/Img");
        let mut dec = reg.decoder_for("/cam").unwrap();
        let err = dec.locate(&frame, "nope").unwrap_err();
        assert!(matches!(err, DecodeError::FieldNotFound { .. }), "{err:?}");
        // Keep `frame` mutable-used to silence unused-mut on some toolchains.
        frame[0] = 0;
    }

    // ── Variable-field per-element decode (hand-oracle bytes) ───────────────

    #[test]
    fn variable_bytes_field_decodes_span() {
        // Schema: one Bytes field `data` (all-variable → fixed_size 0, one
        // offset entry).
        let mut s = MessageSchema::new_in_package("Blob", "test_msgs");
        s.add_field(FieldDef::new("data", FieldType::Bytes));
        let schemas = vec![s];
        let reg = registry_over(schemas.clone(), "/blob", "test_msgs/Blob");

        // payload = [offset table (1×8)][variable payload]. The variable data
        // starts at payload offset 8; write 4 bytes there.
        let payload_var: [u8; 4] = [0xDE, 0xAD, 0xBE, 0xEF];
        let mut frame = vec![0u8; WireHeader::SIZE];
        // offset entry 0: offset=8 (payload-relative), length=4
        frame.extend_from_slice(&8u32.to_le_bytes());
        frame.extend_from_slice(&4u32.to_le_bytes());
        frame.extend_from_slice(&payload_var);
        stamp_hash(&mut frame, &schemas, "test_msgs/Blob");

        let mut dec = reg.decoder_for("/blob").unwrap();
        let d = dec.locate(&frame, "data").unwrap();
        assert!(!d.is_fixed);
        assert_eq!(d.bytes, &payload_var);
    }

    #[test]
    fn dynamic_f64_sequence_decodes_per_element() {
        // Schema: one `float64[]` field `vals`.
        let mut s = MessageSchema::new_in_package("Vec", "test_msgs");
        s.add_field(FieldDef::new(
            "vals",
            FieldType::DynamicArray {
                element_type: Box::new(FieldType::F64),
            },
        ));
        let schemas = vec![s];
        let reg = registry_over(schemas.clone(), "/v", "test_msgs/Vec");

        let elems = [1.5f64, -2.0, 3.25];
        let mut var_bytes = Vec::new();
        for e in &elems {
            var_bytes.extend_from_slice(&e.to_le_bytes());
        }
        let mut frame = vec![0u8; WireHeader::SIZE];
        // offset entry 0: offset=8, length=24
        frame.extend_from_slice(&8u32.to_le_bytes());
        frame.extend_from_slice(&(var_bytes.len() as u32).to_le_bytes());
        frame.extend_from_slice(&var_bytes);
        stamp_hash(&mut frame, &schemas, "test_msgs/Vec");

        let mut dec = reg.decoder_for("/v").unwrap();
        let got = dec.decode_f64_sequence(&frame, "vals").unwrap();
        assert_eq!(got, vec![1.5, -2.0, 3.25]);
    }

    // ── Malformed offset-table rejection (crafted corrupt frames) ──────────

    #[test]
    fn variable_offset_out_of_bounds_is_loud_error() {
        let mut s = MessageSchema::new_in_package("Blob", "test_msgs");
        s.add_field(FieldDef::new("data", FieldType::Bytes));
        let schemas = vec![s];
        let reg = registry_over(schemas.clone(), "/blob", "test_msgs/Blob");

        let mut frame = vec![0u8; WireHeader::SIZE];
        // offset entry 0: offset=8, length=9999 (way past the payload)
        frame.extend_from_slice(&8u32.to_le_bytes());
        frame.extend_from_slice(&9999u32.to_le_bytes());
        frame.extend_from_slice(&[0u8; 4]);
        stamp_hash(&mut frame, &schemas, "test_msgs/Blob");

        let mut dec = reg.decoder_for("/blob").unwrap();
        let err = dec.locate(&frame, "data").unwrap_err();
        assert!(
            matches!(err, DecodeError::VariableOutOfBounds { .. }),
            "{err:?}"
        );
    }

    // ── Offset-table truncation is loud, empty field is distinguished ──

    #[test]
    fn frame_truncated_inside_offset_table_is_loud_error_not_empty() {
        // Blob (fixed_size 0, one variable field `data`): the 8-byte offset
        // entry needs payload >= 8. A frame carrying only 4 payload bytes
        // truncates the entry — `read_offset_entry` would return the (0,0)
        // sentinel and a naive bounds check would decode the field as EMPTY.
        // The entry-span check makes this a loud `OffsetTableTruncated` instead.
        let mut s = MessageSchema::new_in_package("Blob", "test_msgs");
        s.add_field(FieldDef::new("data", FieldType::Bytes));
        let schemas = vec![s];
        let reg = registry_over(schemas.clone(), "/blob", "test_msgs/Blob");

        let mut frame = vec![0u8; WireHeader::SIZE];
        frame.extend_from_slice(&[0u8; 4]); // only 4 payload bytes — entry truncated
        stamp_hash(&mut frame, &schemas, "test_msgs/Blob");

        let mut dec = reg.decoder_for("/blob").unwrap();
        let err = dec.locate(&frame, "data").unwrap_err();
        match err {
            DecodeError::OffsetTableTruncated {
                field,
                idx,
                needed,
                payload_len,
            } => {
                assert_eq!(field, "data");
                assert_eq!(idx, 0);
                assert_eq!(needed, 8); // fixed_size(0) + 8*(0+1)
                assert_eq!(payload_len, 4);
            }
            other => panic!("expected OffsetTableTruncated, got {other:?}"),
        }
    }

    #[test]
    fn genuine_empty_variable_field_decodes_to_empty_slice_not_error() {
        // The DISCRIMINATOR arm: a legitimately-empty variable field also
        // encodes as off=0/len=0, but its 8-byte entry span IS present in the
        // payload. The bounds check must let this through as an empty
        // slice, NOT an error — the entry-span-present + (0,0) case.
        let mut s = MessageSchema::new_in_package("Blob", "test_msgs");
        s.add_field(FieldDef::new("data", FieldType::Bytes));
        let schemas = vec![s];
        let reg = registry_over(schemas.clone(), "/blob", "test_msgs/Blob");

        let mut frame = vec![0u8; WireHeader::SIZE];
        // full offset entry 0 present, all zero (off=0, len=0) — genuinely empty
        frame.extend_from_slice(&[0u8; 8]);
        stamp_hash(&mut frame, &schemas, "test_msgs/Blob");

        let mut dec = reg.decoder_for("/blob").unwrap();
        let located = dec.locate(&frame, "data").unwrap();
        assert!(!located.is_fixed);
        assert!(
            located.bytes.is_empty(),
            "genuine empty field → empty slice"
        );
    }

    // ── Schema-hash gate rejects a recorded-vs-current layout drift ──

    #[test]
    fn wrong_schema_hash_is_rejected_and_matching_hash_decodes() {
        let reg = registry_over(vec![image_schema()], "/cam", "test_msgs/Img");

        // A well-formed frame in every OTHER respect.
        let mut frame = vec![0u8; WireHeader::SIZE];
        frame.extend_from_slice(&7u32.to_le_bytes()); // height@0
        frame.extend_from_slice(&5u32.to_le_bytes()); // width@4
        frame.push(1u8); // is_bigendian@8
        frame.extend_from_slice(&[0u8; 3]); // pad
        frame.extend_from_slice(&99u32.to_le_bytes()); // step@12
        frame.extend_from_slice(&[0u8; 16]); // offset table

        // Stamp a WRONG hash: the current layout must refuse to misread it.
        let good_hash = compute_schema_hashes(&[image_schema()])
            .0
            .get("test_msgs/Img")
            .copied()
            .unwrap();
        WireHeader::with_schema(good_hash ^ 0xDEAD_BEEF)
            .write_to_buf(&mut frame[..WireHeader::SIZE]);

        let mut dec = reg.decoder_for("/cam").unwrap();
        let err = dec.locate(&frame, "height").unwrap_err();
        match err {
            DecodeError::SchemaHashMismatch { expected, actual } => {
                assert_eq!(expected, good_hash);
                assert_eq!(actual, good_hash ^ 0xDEAD_BEEF);
            }
            other => panic!("expected SchemaHashMismatch, got {other:?}"),
        }

        // Re-stamp the CORRECT hash: the same frame now decodes.
        stamp_hash(&mut frame, &[image_schema()], "test_msgs/Img");
        let h = dec.locate(&frame, "height").unwrap();
        assert_eq!(u32::from_le_bytes(h.bytes.try_into().unwrap()), 7);
    }

    #[test]
    fn frame_shorter_than_header_is_loud_error() {
        let reg = registry_over(vec![image_schema()], "/cam", "test_msgs/Img");
        let mut dec = reg.decoder_for("/cam").unwrap();
        let err = dec.locate(&[0u8; 10], "height").unwrap_err();
        assert!(
            matches!(err, DecodeError::FrameTooShort { len: 10 }),
            "{err:?}"
        );
    }

    #[test]
    fn fixed_field_out_of_bounds_is_loud_error() {
        // A frame truncated inside the fixed section: header + only 4 fixed
        // bytes (step@12 needs 12..16).
        let reg = registry_over(vec![image_schema()], "/cam", "test_msgs/Img");
        let mut frame = vec![0u8; WireHeader::SIZE + 4];
        frame[WireHeader::SIZE..].copy_from_slice(&7u32.to_le_bytes());
        stamp_hash(&mut frame, &[image_schema()], "test_msgs/Img");
        let mut dec = reg.decoder_for("/cam").unwrap();
        let err = dec.locate(&frame, "step").unwrap_err();
        assert!(
            matches!(err, DecodeError::FixedOutOfBounds { .. }),
            "{err:?}"
        );
    }

    #[test]
    fn dynamic_f64_sequence_rejects_ragged_length() {
        let mut s = MessageSchema::new_in_package("Vec", "test_msgs");
        s.add_field(FieldDef::new(
            "vals",
            FieldType::DynamicArray {
                element_type: Box::new(FieldType::F64),
            },
        ));
        let schemas = vec![s];
        let reg = registry_over(schemas.clone(), "/v", "test_msgs/Vec");
        let mut frame = vec![0u8; WireHeader::SIZE];
        // length 5 (not a multiple of 8)
        frame.extend_from_slice(&8u32.to_le_bytes());
        frame.extend_from_slice(&5u32.to_le_bytes());
        frame.extend_from_slice(&[0u8; 5]);
        stamp_hash(&mut frame, &schemas, "test_msgs/Vec");
        let mut dec = reg.decoder_for("/v").unwrap();
        assert!(dec.decode_f64_sequence(&frame, "vals").is_err());
    }

    // ── Nested-path resolution (through nested + variable sequences) ───────

    /// A schema set with a variable sequence of nested schemas:
    /// `Detection2DArray { detections: Detection2D[] }`,
    /// `Detection2D { bbox: BBox, score: float64 }` (bbox nested fixed),
    /// `BBox { x: float64, y: float64 }`.
    fn detection_set() -> Vec<MessageSchema> {
        let mut bbox = MessageSchema::new_in_package("BBox", "vision_msgs");
        bbox.add_field(FieldDef::new("x", FieldType::F64));
        bbox.add_field(FieldDef::new("y", FieldType::F64));
        let mut det = MessageSchema::new_in_package("Detection2D", "vision_msgs");
        det.add_field(FieldDef::new(
            "bbox",
            FieldType::Nested {
                schema_name: "BBox".into(),
                package: None,
                fixed: None,
            },
        ));
        det.add_field(FieldDef::new("score", FieldType::F64));
        let mut arr = MessageSchema::new_in_package("Detection2DArray", "vision_msgs");
        arr.add_field(FieldDef::new(
            "detections",
            FieldType::DynamicArray {
                element_type: Box::new(FieldType::Nested {
                    schema_name: "Detection2D".into(),
                    package: None,
                    fixed: None,
                }),
            },
        ));
        vec![bbox, det, arr]
    }

    fn detection_registry() -> FieldRegistry {
        registry_over(detection_set(), "/det", "vision_msgs/Detection2DArray")
    }

    #[test]
    fn nested_variable_path_resolves() {
        let reg = detection_registry();
        // detections (seq of Detection2D) → bbox (nested BBox) — a valid path.
        assert!(reg.resolve_field("/det", "detections.bbox").is_ok());
        // detections → score (a leaf f64 on each element)
        assert!(reg.resolve_field("/det", "detections.score").is_ok());
        // deep: detections.bbox.x
        assert!(reg.resolve_field("/det", "detections.bbox.x").is_ok());
    }

    #[test]
    fn nested_path_typo_is_reported_with_candidates() {
        let reg = detection_registry();
        // 'detections.bbx' — 'bbx' is a typo of 'bbox' on Detection2D.
        let err = reg.resolve_field("/det", "detections.bbx").unwrap_err();
        assert_eq!(err.segment, "bbx");
        assert!(
            err.candidates.contains(&"bbox".to_string()),
            "{:?}",
            err.candidates
        );
        assert!(err.candidates.contains(&"score".to_string()));
        assert!(!err.schema_unavailable);
    }

    #[test]
    fn top_level_field_typo_reports_root_candidates() {
        let reg = detection_registry();
        let err = reg.resolve_field("/det", "detectons").unwrap_err();
        assert_eq!(err.segment, "detectons");
        assert!(err.candidates.contains(&"detections".to_string()));
    }

    #[test]
    fn descending_into_a_leaf_fails() {
        let reg = detection_registry();
        // detections.score is an f64 leaf — 'score.foo' cannot descend.
        let err = reg
            .resolve_field("/det", "detections.score.foo")
            .unwrap_err();
        assert_eq!(err.segment, "foo");
        assert!(err.candidates.is_empty());
    }

    #[test]
    fn external_topic_field_is_schema_unavailable() {
        let mut reg = detection_registry();
        reg.topics.push((
            "/ext".to_string(),
            TopicEntry {
                class: TopicClass::External,
                schema_qname: None,
                expected_schema_hash: None,
            },
        ));
        let err = reg.resolve_field("/ext", "anything").unwrap_err();
        assert!(err.schema_unavailable);
    }

    #[test]
    fn topics_list_and_class() {
        let mut reg = detection_registry();
        reg.topics.push((
            "/ext".to_string(),
            TopicEntry {
                class: TopicClass::External,
                schema_qname: None,
                expected_schema_hash: None,
            },
        ));
        let topics = FieldResolver::topics(&reg);
        assert_eq!(topics, vec!["/det".to_string(), "/ext".to_string()]);
        assert_eq!(reg.topic_class("/det"), Some(&TopicClass::Produced));
        assert_eq!(reg.topic_class("/ext"), Some(&TopicClass::External));
        assert_eq!(reg.topic_class("/missing"), None);
    }

    // ── Metric decodability classification (check_field_metric) ─────────────

    /// A schema with a `float64[]` field (metric-decodable sequence).
    fn f64_seq_registry() -> FieldRegistry {
        let mut s = MessageSchema::new_in_package("Vec", "test_msgs");
        s.add_field(FieldDef::new(
            "vals",
            FieldType::DynamicArray {
                element_type: Box::new(FieldType::F64),
            },
        ));
        registry_over(vec![s], "/v", "test_msgs/Vec")
    }

    #[test]
    fn bit_exact_metric_is_legal_on_any_resolvable_field() {
        let reg = detection_registry();
        // Opaque top-level array-of-nested, a dotted per-element path, and a
        // leaf all accept bit_exact (byte compare needs no decode).
        for path in ["detections", "detections.bbox", "detections.score"] {
            assert!(
                reg.check_field_metric("/det", path, &MetricKind::BitExact {})
                    .is_ok(),
                "bit_exact rejected on {path}"
            );
        }
    }

    #[test]
    fn numeric_metric_on_opaque_top_level_field_is_rejected() {
        let reg = detection_registry();
        // `detections` is a DynamicArray<Nested> — publisher-opaque bytes.
        let err = reg
            .check_field_metric("/det", "detections", &MetricKind::MaxAbs { threshold: 0.1 })
            .unwrap_err();
        assert!(
            err.contains("publisher-opaque") && err.contains("bit_exact"),
            "got: {err}"
        );
    }

    #[test]
    fn numeric_metric_on_dotted_nested_path_is_rejected() {
        let reg = detection_registry();
        // A nested per-element path has no addressable wire span in the metric decoder.
        let err = reg
            .check_field_metric(
                "/det",
                "detections.score",
                &MetricKind::MaxAbs { threshold: 0.1 },
            )
            .unwrap_err();
        assert!(
            err.contains("NESTED") && err.contains("bit_exact"),
            "got: {err}"
        );
    }

    #[test]
    fn numeric_metric_on_fixed_scalar_is_accepted() {
        // image_schema.height is a U32 fixed scalar → decodable.
        let reg = registry_over(vec![image_schema()], "/cam", "test_msgs/Img");
        for m in [
            MetricKind::MaxAbs { threshold: 0.1 },
            MetricKind::MaxRel { threshold: 0.1 },
            MetricKind::Rmse { threshold: 0.1 },
            MetricKind::SetEqual {},
            MetricKind::OrderedListEqual {},
        ] {
            assert!(
                reg.check_field_metric("/cam", "height", &m).is_ok(),
                "{m:?} rejected on a numeric scalar"
            );
        }
    }

    /// A schema with two 64-bit integer scalar fields (a `U64` timestamp + an
    /// `I64` sequence) for the integer-domain pins.
    fn stamp_schema() -> MessageSchema {
        let mut s = MessageSchema::new_in_package("Stamp", "test_msgs");
        s.add_field(FieldDef::new("t_ns", FieldType::U64));
        s.add_field(FieldDef::new("seq", FieldType::I64));
        s
    }

    #[test]
    fn max_rel_and_rmse_rejected_on_64bit_integer_fields() {
        // max_rel/rmse are precision-lossy on I64/U64 (f64 domain loses
        // bits above 2^53) → exit-4 refused. max_abs/set/ordered/bit_exact stay.
        let reg = registry_over(vec![stamp_schema()], "/st", "test_msgs/Stamp");
        for field in ["t_ns", "seq"] {
            for m in [
                MetricKind::MaxRel { threshold: 0.1 },
                MetricKind::Rmse { threshold: 0.1 },
            ] {
                let err = reg.check_field_metric("/st", field, &m).unwrap_err();
                assert!(
                    err.contains("precision-lossy") && err.contains("max_abs"),
                    "field {field} metric {m:?}: {err}"
                );
            }
            for m in [
                MetricKind::MaxAbs { threshold: 1.0 },
                MetricKind::SetEqual {},
                MetricKind::OrderedListEqual {},
                MetricKind::BitExact {},
            ] {
                assert!(
                    reg.check_field_metric("/st", field, &m).is_ok(),
                    "{m:?} must stay legal on 64-bit int field {field}"
                );
            }
        }
    }

    #[test]
    fn decode_i64_u64_scalar_to_i128_preserves_full_precision() {
        let schemas = vec![stamp_schema()];
        let reg = registry_over(schemas.clone(), "/st", "test_msgs/Stamp");
        let t: u64 = 1_700_000_000_000_000_001; // above 2^53 → f64-lossy
        let sq: i64 = -5;
        let mut frame = vec![0u8; WireHeader::SIZE];
        frame.extend_from_slice(&t.to_le_bytes()); // t_ns @0
        frame.extend_from_slice(&sq.to_le_bytes()); // seq  @8
        stamp_hash(&mut frame, &schemas, "test_msgs/Stamp");
        let mut dec = reg.decoder_for("/st").unwrap();
        assert_eq!(
            dec.decode_field_i128(&frame, "t_ns").unwrap(),
            vec![t as i128]
        );
        assert_eq!(
            dec.decode_field_i128(&frame, "seq").unwrap(),
            vec![sq as i128]
        );
        // The exact i128 value survives; the f64 path would have lost the low bit.
        assert_eq!(dec.decode_field_i128(&frame, "t_ns").unwrap()[0], t as i128);
        assert_ne!(t as i128, (t as f64) as i128 - 1); // (documents the f64 blindness)
    }

    #[test]
    fn string_and_bytes_fields_reject_numeric_metrics() {
        let reg = registry_over(vec![image_schema()], "/cam", "test_msgs/Img");
        // `encoding` is a String, `data` is Bytes — both opaque for the metric
        // decoder (only bit_exact applies).
        for field in ["encoding", "data"] {
            let err = reg
                .check_field_metric("/cam", field, &MetricKind::Rmse { threshold: 0.1 })
                .unwrap_err();
            assert!(err.contains("bit_exact"), "field {field}: {err}");
        }
    }

    #[test]
    fn bbox_iou_requires_an_f64_array_not_a_scalar() {
        let reg = registry_over(vec![image_schema()], "/cam", "test_msgs/Img");
        // A scalar cannot be a flattened [x,y,w,h]*N sequence.
        let err = reg
            .check_field_metric("/cam", "height", &MetricKind::BboxIou { min_iou: 0.5 })
            .unwrap_err();
        assert!(
            err.contains("bbox_iou") && err.contains("float64[]"),
            "got: {err}"
        );
        // A float64[] field accepts bbox_iou (and every other numeric metric).
        let vreg = f64_seq_registry();
        assert!(vreg
            .check_field_metric("/v", "vals", &MetricKind::BboxIou { min_iou: 0.5 })
            .is_ok());
        assert!(vreg
            .check_field_metric("/v", "vals", &MetricKind::MaxAbs { threshold: 0.1 })
            .is_ok());
    }

    // ── decode_field_f64 (scalar + sequence, hand oracle) ───────────────────

    #[test]
    fn decode_fixed_scalar_to_single_f64() {
        let reg = registry_over(vec![image_schema()], "/cam", "test_msgs/Img");
        let mut frame = vec![0u8; WireHeader::SIZE];
        frame.extend_from_slice(&7u32.to_le_bytes()); // height@0
        frame.extend_from_slice(&5u32.to_le_bytes()); // width@4
        frame.push(1u8); // is_bigendian@8
        frame.extend_from_slice(&[0u8; 3]); // pad
        frame.extend_from_slice(&99u32.to_le_bytes()); // step@12
        frame.extend_from_slice(&[0u8; 16]); // offset table
        stamp_hash(&mut frame, &[image_schema()], "test_msgs/Img");
        let mut dec = reg.decoder_for("/cam").unwrap();
        assert_eq!(dec.decode_field_f64(&frame, "height").unwrap(), vec![7.0]);
        assert_eq!(dec.decode_field_f64(&frame, "step").unwrap(), vec![99.0]);
        // A non-numeric field (Bytes) is not decodable to an f64 sequence.
        assert!(dec.decode_field_f64(&frame, "data").is_err());
    }

    #[test]
    fn decode_f64_array_field_to_sequence() {
        let reg = f64_seq_registry();
        let elems = [1.5f64, -2.0, 3.25];
        let mut var_bytes = Vec::new();
        for e in &elems {
            var_bytes.extend_from_slice(&e.to_le_bytes());
        }
        let mut frame = vec![0u8; WireHeader::SIZE];
        frame.extend_from_slice(&8u32.to_le_bytes()); // offset entry: off=8
        frame.extend_from_slice(&(var_bytes.len() as u32).to_le_bytes());
        frame.extend_from_slice(&var_bytes);
        let schemas = {
            let mut s = MessageSchema::new_in_package("Vec", "test_msgs");
            s.add_field(FieldDef::new(
                "vals",
                FieldType::DynamicArray {
                    element_type: Box::new(FieldType::F64),
                },
            ));
            vec![s]
        };
        stamp_hash(&mut frame, &schemas, "test_msgs/Vec");
        let mut dec = reg.decoder_for("/v").unwrap();
        assert_eq!(
            dec.decode_field_f64(&frame, "vals").unwrap(),
            vec![1.5, -2.0, 3.25]
        );
    }

    #[test]
    fn root_layout_exposes_fixed_and_variable_fields() {
        let reg = registry_over(vec![image_schema()], "/cam", "test_msgs/Img");
        let mut dec = reg.decoder_for("/cam").unwrap();
        let layout = dec.root_layout().expect("layout resolves");
        // height/width/is_bigendian/step are fixed; encoding/data variable.
        assert!(layout.fixed_fields.iter().any(|f| f.name == "height"));
        assert!(layout.variable_fields.iter().any(|f| f.name == "data"));
    }
}
