// SPDX-License-Identifier: AGPL-3.0-only
//! Build the gateway's [`SchemaServing`] from the workspace.
//!
//! A network-free `cerulion_core` gateway process cannot reach the workspace
//! `.msg` store, the workspace YAML schemas, or the built-in ROS 2 corpus — so
//! the CLI (which can) computes the served schema docs + the topic→name catalog
//! bindings HERE and hands them across the process boundary in the gateway plan.
//!
//! # What is served (the automagic bar: "NOBODY ever compiles schemas")
//!
//! The robot serves the CUSTOM types it actually has — every `schemas/<pkg>/msg/
//! <Type>.msg` store entry (encoding `msg`) and every `schemas/*.yaml`
//! Cerulion-native schema (encoding `yaml`) — so a desk with ZERO local knowledge
//! can fetch a type's verbatim text + nested-custom closure and decode its frames.
//! BUILT-IN types (`std_msgs`, `geometry_msgs`, …) are OMITTED from the served
//! DOCS: every desk already compiled them into `native_ros2_messages`, so
//! re-shipping them is waste (mirrors the acquirer's "served from the
//! corpus, never re-materialized" rule). The served set IS the built-in
//! classifier: a nested reference that does not resolve to a served custom type
//! is either a built-in the desk has or a type nobody has — either way there is
//! nothing to serve, so it is not a dep. Their hash→NAME bindings ARE handed
//! across, so the gateway can catalog a runtime-registered
//! built-in-typed topic by name — the desk then decodes it locally.
//!
//! # `deps` (the closure)
//!
//! Each served doc carries the qualified names of the OTHER served (custom) types
//! it directly references; the desk walks them transitively
//! ([`cerulion_core::transport::cerulion_q::collect_schema_closure`]). Since the
//! served set contains every custom type, the walk is closure-complete and never
//! dangles.
//!
//! # The bridge fold
//!
//! On a `cerulion ros2 attach` robot the topic→type table lives NOT in the graph
//! (its nodes declare only the bridge's four FIXED typed ports) but in the
//! BRIDGE's own config `graphs/<name>.bridge.yaml` — the generic `dds_bridge`
//! node publishes each RAW mapping on its authoritative `cerulion_topic` with no
//! graph port, so those topics never reach the node-output catalog below. When a
//! bridge config is discovered ([`resolve_bridge_config_path`]: the
//! `DDS_BRIDGE_CONFIG` env WINS, else the sibling convention), its `mappings:`
//! table is FOLDED into the catalog (each `cerulion_topic → ros_type`; the GRAPH
//! wins a topic collision) and its `msg_dirs` store is loaded so the mapped
//! custom types are servable. A robot with no bridge config is byte-identical to
//! the serving without the fold.

use std::collections::{BTreeMap, BTreeSet, HashSet};
use std::ffi::OsStr;
use std::path::{Path, PathBuf};

use cerulion_core::codegen::{resolve_fixed_nested, FieldType, MessageSchema};
use cerulion_core::graph::config::GraphConfig;
use cerulion_core::graph::resolve_output_topic;
use cerulion_core::{SchemaDoc, SchemaEncoding, SchemaHashName, SchemaServing, TopicSchema};
use serde::Deserialize;

use crate::schema_cmd::{parse_builtin_schemas, parse_message_schemas, workspace_yaml_files};
use crate::schema_store::SchemaStore;

/// Recursively collect the nested references of a field type (unwrapping fixed +
/// dynamic arrays), pushing each as `(package, schema_name)`. Pure.
fn collect_nested(ft: &FieldType, out: &mut Vec<(Option<String>, String)>) {
    match ft {
        FieldType::FixedArray { element_type, .. } | FieldType::DynamicArray { element_type } => {
            collect_nested(element_type, out)
        }
        FieldType::Nested {
            schema_name,
            package,
            ..
        } => out.push((package.clone(), schema_name.clone())),
        _ => {}
    }
}

/// Every nested reference in a schema's fields, as `(package, schema_name)`. Pure.
fn nested_refs(schema: &MessageSchema) -> Vec<(Option<String>, String)> {
    let mut out = Vec::new();
    for f in &schema.fields {
        collect_nested(&f.field_type, &mut out);
    }
    out
}

/// Resolve ONE nested reference to a SERVED qualified name (a dep), or `None` when
/// it is not a served custom type (a built-in the desk has, or an absent type).
///
/// - A QUALIFIED ref (`geometry_msgs/Pose`) resolves iff that exact key is served.
/// - A BARE ref (`Header`) tries the parent's package first (ROS same-package
///   semantics), then a UNIQUE bare-name match across the served set. A built-in
///   like bare `Header`/`Time` never has a served entry, so it correctly yields
///   `None` (omitted from `deps`).
///
/// Pure — oracle-tested.
fn resolve_dep(
    pkg: Option<&str>,
    name: &str,
    parent_pkg: Option<&str>,
    served: &BTreeSet<String>,
) -> Option<String> {
    if let Some(p) = pkg {
        let q = format!("{p}/{name}");
        return served.contains(&q).then_some(q);
    }
    // Bare: same-package first.
    if let Some(pp) = parent_pkg {
        let q = format!("{pp}/{name}");
        if served.contains(&q) {
            return Some(q);
        }
    }
    // Then a UNIQUE bare-name match (either a bare served name, or a `pkg/name`
    // whose type part is `name`). Ambiguous or absent ⇒ not a dep.
    let mut matches = served
        .iter()
        .filter(|q| q.as_str() == name || q.rsplit_once('/').is_some_and(|(_, t)| t == name));
    match (matches.next(), matches.next()) {
        (Some(only), None) => Some(only.clone()),
        _ => None,
    }
}

/// The served custom-type deps a schema DIRECTLY references (deduped, self
/// excluded). Pure.
fn deps_of(schema: &MessageSchema, served: &BTreeSet<String>) -> Vec<String> {
    let parent_pkg = schema.package.clone();
    // The served set is keyed canonically, so the self key must
    // be too: a `pkg::Type` document referencing `pkg/Type`
    // otherwise listed ITSELF as a dependency.
    let self_q = crate::schema_cmd::normalize_schema(&schema.qualified_name());
    let mut deps: Vec<String> = Vec::new();
    for (pkg, name) in nested_refs(schema) {
        if let Some(q) = resolve_dep(pkg.as_deref(), &name, parent_pkg.as_deref(), served) {
            if q != self_q && !deps.contains(&q) {
                deps.push(q);
            }
        }
    }
    deps
}

/// One custom-type source before dep resolution: encoding + verbatim text +
/// parsed IR.
struct DocSrc {
    encoding: SchemaEncoding,
    text: String,
    schema: MessageSchema,
}

// ───────────────────────── Bridge-config fold ──────────────────────
//
// On a `cerulion ros2 attach` robot the topic→type table lives NOT in the graph
// (its nodes only declare the bridge's four FIXED typed ports) but in the
// BRIDGE's own config `graphs/<name>.bridge.yaml` — a `mappings:` list where
// every entry carries `{cerulion_topic, ros_type}` (a Unitree Go2 maps all 85
// topics). The generic `dds_bridge` node publishes each RAW mapping on its
// AUTHORITATIVE `cerulion_topic` via a dynamically-created ingress publisher
// (no graph port), so those topics never reach `build_schema_serving`'s
// node-output catalog and the served catalog names none of them — the exact
// live gap (`no ROS type for topic '/utlidar/robot_odom'`). This module folds
// the bridge's mapping table INTO the catalog so every bridged topic is named,
// with ZERO robot-specific logic: any robot with a bridge config gets it, and a
// robot with none is byte-identical to the serving without the fold.

/// The env var naming the bridge config the `dds_bridge` node loads
/// (`examples/go2/nodes/dds_bridge` `CONFIG_ENV`). It is the AUTHORITATIVE pointer:
/// the bridge itself reads THIS file, so when it is set the served catalog must
/// match it (never a stale sibling). Set absolute at the `ros2 attach` auto-run
/// handoff (`crates/cerulion_cli/src/main.rs`); a manual `graph run` inherits whatever
/// the shell exported.
pub const BRIDGE_CONFIG_ENV: &str = "DDS_BRIDGE_CONFIG";

/// A LENIENT view of a bridge config yaml — only the two fields the catalog fold
/// needs. Deliberately NOT the real `examples/go2` `BridgeConfig` (that crate is a
/// demo cdylib outside this workspace): a lightweight local mirror avoids the
/// dependency and, crucially, does NOT set `deny_unknown_fields`, so the real
/// config's `domain_id`/`only_networks`/`qos`/`route`/`max_slice_len` keys are
/// ignored rather than rejected. A mapping MISSING `cerulion_topic`/`ros_type`
/// still fails the parse (loud skip — the whole file degrades, never a
/// half-read table).
#[derive(Debug, Deserialize)]
struct BridgeYamlLite {
    /// Workspace `.msg` store dirs the bridge's generic codec loads — the SAME
    /// key `schema serving` already loads from `schemas/`, folded here for the
    /// GENERAL case (a bridge pointing its store elsewhere). Relative entries
    /// resolve against the config file's own dir at [`bridge_msg_store_dirs`].
    #[serde(default)]
    msg_dirs: Option<Vec<PathBuf>>,
    /// The DDS→Cerulion mappings — the topic→type table.
    #[serde(default)]
    mappings: Vec<BridgeMappingLite>,
}

/// One bridge mapping — the authoritative `cerulion_topic → ros_type` binding.
/// Extra fields (`dds_topic`, `qos`, `route`, `max_slice_len`) are ignored.
#[derive(Debug, Deserialize)]
struct BridgeMappingLite {
    /// The Cerulion-side topic. For RAW mappings this is AUTHORITATIVE (the real
    /// ingress-publisher topic); for TYPED mappings it is the graph port's
    /// `topic:` override verbatim (so it collides with — and loses to — the
    /// graph-derived binding).
    cerulion_topic: String,
    /// The ROS message type as `pkg/Type`.
    ros_type: String,
}

/// Discover the bridge config path for a run: the [`BRIDGE_CONFIG_ENV`] env var
/// wins (verbatim — it is what the bridge node itself loads), else the sibling
/// convention `graphs/<graph_name>.bridge.yaml` IF it exists. `None` ⇒ no bridge
/// (a normal graph — the serving is unchanged). The env branch is not
/// existence-gated (an explicit env pointing at a missing file surfaces a loud
/// read warn at fold time); the sibling branch IS existence-gated so a normal
/// graph run stays silent.
pub fn resolve_bridge_config_path(graphs_dir: &Path, graph_name: &str) -> Option<PathBuf> {
    let env = std::env::var_os(BRIDGE_CONFIG_ENV);
    let sibling = graphs_dir.join(format!("{graph_name}.bridge.yaml"));
    choose_bridge_config(env.as_deref(), sibling, |p| p.exists())
}

/// [`BRIDGE_CONFIG_ENV`] as a bridge-config path, with NO sibling candidate —
/// for a caller that has no graph to derive one from (`cerulion bag record`).
///
/// Exists so that caller cannot re-implement the env read and drift:
/// a hand-rolled `std::env::var(..).ok().map(PathBuf::from)`
/// turns `DDS_BRIDGE_CONFIG=""` into `Some("")` — a path that then fails to
/// read, warning about a bridge config the operator never set — and drops a
/// non-UTF-8 path silently, where `var_os` keeps it.
pub(crate) fn bridge_config_from_env() -> Option<PathBuf> {
    choose_bridge_config(
        std::env::var_os(BRIDGE_CONFIG_ENV).as_deref(),
        PathBuf::new(),
        |_| false,
    )
}

/// PURE discovery decision (oracle-tested): [`BRIDGE_CONFIG_ENV`] wins verbatim
/// when set + non-empty; else the `sibling` candidate iff `sibling_exists`
/// reports it present. Split from [`resolve_bridge_config_path`] so the env-wins
/// / empty-env-ignored / sibling-only-if-exists / none matrix is testable
/// without touching process env or the filesystem.
fn choose_bridge_config(
    env: Option<&OsStr>,
    sibling: PathBuf,
    sibling_exists: impl Fn(&Path) -> bool,
) -> Option<PathBuf> {
    if let Some(p) = env.filter(|p| !p.is_empty()) {
        return Some(PathBuf::from(p));
    }
    sibling_exists(&sibling).then_some(sibling)
}

/// Parse a bridge config yaml, BEST-EFFORT: an unreadable file or malformed YAML
/// is a loud `warn!` + `None` (the serving degrades to graph-only — never a
/// crash, never a silent skip — mirroring the store-read robustness and
/// `cerulion_viz`'s `bridge_config_msg_dirs`). `Some` iff the file read AND the
/// lenient parse both succeed.
fn parse_bridge_config(path: &Path) -> Option<BridgeYamlLite> {
    let text = match std::fs::read_to_string(path) {
        Ok(t) => t,
        Err(e) => {
            tracing::warn!(
                file = %path.display(), error = %e,
                "schema-serving: could not read the bridge config — the catalog will \
                 name only graph-declared topics (bridged topics stay unnamed)"
            );
            return None;
        }
    };
    match serde_yaml::from_str::<BridgeYamlLite>(&text) {
        Ok(cfg) => Some(cfg),
        Err(e) => {
            tracing::warn!(
                file = %path.display(), error = %e,
                "schema-serving: bridge config is not parseable as a mapping table — the \
                 catalog will name only graph-declared topics"
            );
            None
        }
    }
}

/// The bridge config's `.msg` store dirs, RELATIVE entries resolved against the
/// config file's OWN directory (the resolution contract the bridge applies at
/// load — `examples/go2` `BridgeConfig::from_yaml` supplement A; a config in
/// `graphs/` reaches the workspace store via `../schemas`). Absolute entries are
/// used verbatim.
fn bridge_msg_store_dirs(config_path: &Path, cfg: &BridgeYamlLite) -> Vec<PathBuf> {
    let base = config_path.parent().unwrap_or(Path::new(""));
    cfg.msg_dirs
        .iter()
        .flatten()
        .map(|d| {
            if d.is_relative() && !base.as_os_str().is_empty() {
                base.join(d)
            } else {
                d.clone()
            }
        })
        .collect()
}

/// A canonical dedup key for topic-collision detection: strip a single leading
/// `/`. Graph-derived topics ([`resolve_output_topic`]) and bridge
/// `cerulion_topic`s are both absolute (`/`-prefixed), and a typed mapping's
/// `cerulion_topic` is the graph port's `topic:` override VERBATIM, so on the
/// generated attach graph a typed route's graph binding and its bridge binding
/// share this key exactly (the graph wins the dedup — no phantom).
fn topic_dedup_key(topic: &str) -> &str {
    topic.strip_prefix('/').unwrap_or(topic)
}

/// Load every `<store_dir>/<pkg>/msg/<Type>.msg` into `docs_src` as `Msg` docs,
/// re-reading each file's verbatim text (the store drops it after parsing). When
/// `overwrite` is true the PRIMARY workspace store wins collisions
/// (authoritative); when false a folded bridge `msg_dir` only FILLS a gap (the
/// primary store wins). An unreadable file is skipped with a `warn!` (the rest
/// still serve).
fn load_msg_store(docs_src: &mut BTreeMap<String, DocSrc>, store_dir: &Path, overwrite: bool) {
    let store = SchemaStore::load(store_dir);
    for (qualified, stored) in store.iter() {
        if !overwrite && docs_src.contains_key(qualified) {
            continue;
        }
        // StoredSchema drops the raw text after parsing — re-read it (a cold,
        // one-time boot op) so the desk gets the byte-identical original.
        let path = store_dir.join(&stored.relative_path);
        match std::fs::read_to_string(&path) {
            Ok(text) => {
                docs_src.insert(
                    qualified.to_string(),
                    DocSrc {
                        encoding: SchemaEncoding::Msg,
                        text,
                        schema: stored.schema.clone(),
                    },
                );
            }
            Err(e) => tracing::warn!(
                file = %path.display(), error = %e,
                "schema-serving: could not re-read a .msg store file — that type will \
                 not be served (the rest still serve)"
            ),
        }
    }
}

/// Build the [`SchemaServing`] for a graph — the topic→schema-name
/// catalog bindings + the served custom-type docs (`.msg` store + workspace YAML,
/// built-ins omitted), each doc carrying its nested-custom closure deps. Never
/// fails: an unreadable file is skipped with a `warn!` (the rest still serve).
///
/// `bridge_config_path` (from [`resolve_bridge_config_path`]) FOLDS a
/// `cerulion ros2 attach` bridge's `mappings:` table into the catalog. On such a
/// robot the topic→type truth lives in `graphs/<name>.bridge.yaml`, not the
/// graph (whose nodes declare only the bridge's four fixed typed ports), so each
/// mapping's `cerulion_topic → ros_type` is registered here and the bridge's
/// `msg_dirs` store is folded in so those types are servable. `None` (a normal
/// graph) is byte-identical to the serving without the fold. Precedence: a
/// GRAPH-declared topic binding WINS a bridge collision (Principle #5 — the
/// graph is the source of truth for its ports); a bridge mapping only ADDS a
/// topic the graph never declared (the raw ingress routes — the live gap).
pub fn build_schema_serving(
    config: &GraphConfig,
    schemas_dir: &Path,
    bridge_config_path: Option<&Path>,
) -> SchemaServing {
    // Parse the bridge config once (best-effort — a malformed file is a
    // loud warn + None, so the serving degrades to graph-only). Its mappings
    // enrich the catalog; its `msg_dirs` store makes the mapped custom types
    // servable.
    let bridge_cfg = bridge_config_path.and_then(parse_bridge_config);

    let (schema_docs, mut schema_hashes) = build_schema_docs(schemas_dir, bridge_config_path);

    // Hand across the BUILT-IN corpus's hash→name bindings too.
    // The gateway names a RUNTIME-registered topic (an rmw publisher, a raw
    // route — reg-channel `(topic, schema_hash)`, no name) ONLY through this
    // map; with custom types only in it, a `std_msgs/String`
    // rmw topic would catalogue `schema_name: None` and every desk consumer would
    // refuse it (`SchemaUnavailable`). The desk resolves a NAMED built-in from its own
    // compiled corpus with no served doc (the local-first rule), so the
    // NAME is all that is missing — `schema_docs` still omits built-ins.
    //
    // ORDER: customs FIRST, built-ins appended. `hash_for_topic` (netd's egress
    // plane) resolves name→hash by FIRST match, so a workspace type that shadows
    // a built-in's name keeps resolving to ITS OWN hash; the gateway's hash→name
    // map is last-wins, where a duplicate hash implies a duplicate name (the
    // qualified name folds into the digest), so the order there is inert.
    schema_hashes.extend(builtin_hash_bindings());

    // 3. topic→schema-name bindings — every produced topic's declared schema (its
    //    resolved absolute topic name → the qualified `pkg/Type`). Built-in-typed
    //    topics are included too (the desk uses the name to decode LOCALLY without
    //    a fetch); an empty schema declaration is skipped.
    //    A graph output's `schema:` may be a
    //    BARE name that port resolution binds to a STORE definition (a unique
    //    bare store name) — but the served docs and hash bindings are keyed
    //    `pkg/Type`, so a binding carrying `Goal` verbatim would make the gateway
    //    look `Goal` up in qualified-only docs and serve nothing. The binding
    //    is resolved with the SAME rule as validation and the graph-hash map:
    //    a name the workspace-YAML tier claims binds the DOCUMENT that claim
    //    names (an entry claim itself; a file-stem claim the file's sole
    //    entry), and only a name no YAML claims, borne by exactly
    //    one parsed store schema, is qualified.
    //    A bound stem's entry has ONE
    //    declarer (the owner), which is what the served set keys the document
    //    under; every ambiguous claim is REFUSED and served by nothing.
    let yaml_files = crate::schema_cmd::parsed_workspace_yaml(schemas_dir);
    let yaml_claims = crate::schema_cmd::workspace_yaml_bare_claims_of(&yaml_files);
    let unique_bare_store = crate::schema_cmd::unique_bare_store_names(
        &SchemaStore::load(schemas_dir).schemas(),
        &yaml_claims,
    );
    let mut topic_schemas: Vec<TopicSchema> = Vec::new();
    let mut seen_topics: HashSet<String> = HashSet::new();
    for node in &config.nodes {
        for output in &node.outputs {
            if output.schema.is_empty() {
                continue;
            }
            let topic = resolve_output_topic(&config.prefix, &node.id, output);
            seen_topics.insert(topic_dedup_key(&topic).to_string());
            topic_schemas.push(TopicSchema {
                topic,
                schema_name: bind_output_schema_name(
                    &output.schema,
                    &yaml_claims,
                    &unique_bare_store,
                ),
            });
        }
    }

    // Fold the bridge's mapping table — each `cerulion_topic → ros_type`
    // the graph did NOT already declare (the raw ingress routes the bridge
    // publishes with no graph port). The GRAPH wins a collision (a typed
    // mapping's `cerulion_topic` IS its graph port's `topic:` override, so it is
    // already bound above and skipped here — no phantom, no override of the
    // authoritative graph binding). Order-stable: mappings fold in declaration
    // order after the graph bindings.
    //
    // A hand-edited mapping with an EMPTY (post-trim)
    // `ros_type` or `cerulion_topic` would fold a phantom empty-named binding the
    // graph path (which skips an empty `output.schema`) would never emit — the
    // desk can't decode "" and it pollutes the catalog. Skip it with a loud
    // `warn!` naming the mapping index + the file (being robust to hand-edited
    // configs is exactly why the guard must exist).
    if let Some(cfg) = &bridge_cfg {
        for (index, m) in cfg.mappings.iter().enumerate() {
            if m.cerulion_topic.trim().is_empty() || m.ros_type.trim().is_empty() {
                tracing::warn!(
                    file = %bridge_config_path.map(|p| p.display().to_string()).unwrap_or_default(),
                    mapping_index = index,
                    cerulion_topic = %m.cerulion_topic,
                    ros_type = %m.ros_type,
                    "schema-serving: bridge mapping has an empty cerulion_topic or \
                     ros_type — skipped (an empty-named catalog binding the desk cannot decode)"
                );
                continue;
            }
            if seen_topics.insert(topic_dedup_key(&m.cerulion_topic).to_string()) {
                topic_schemas.push(TopicSchema {
                    topic: m.cerulion_topic.clone(),
                    schema_name: m.ros_type.clone(),
                });
            }
        }
    }

    SchemaServing {
        topic_schemas,
        schema_docs,
        schema_hashes,
    }
}

/// The `schema_hash` → qualified-name binding for every BUILT-IN ROS 2
/// type, so a recorder can NAME a channel whose type every reader already has
/// compiled in, and also so a GATEWAY can name a
/// runtime-registered topic of a built-in type (see [`build_schema_serving`]).
///
/// Deliberately separate from [`build_schema_docs`], which returns the CUSTOM
/// bindings only (the slice a workspace shadow test isolates). Resolving the
/// built-in corpus against ITSELF is sufficient and not a shortcut: a built-in
/// type never references a custom one, so no custom schema can change a
/// built-in's fixed-nested resolution and therefore none can change its hash.
///
/// The ONE implementation lives beside the corpus —
/// [`native_ros2_messages::builtin_hash_bindings`] — because `cerulion-netd`
/// needs the same bindings for its start-booted standing gateway and cannot
/// depend on this crate (cyclic). The cost is one parse of the vendored corpus
/// per call, paid once at recorder / gateway startup — never per frame, and
/// never on a latency-bounded path.
pub fn builtin_hash_bindings() -> Vec<SchemaHashName> {
    native_ros2_messages::builtin_hash_bindings()
}

/// The GRAPH-FREE half of [`build_schema_serving`]: the workspace's custom-type
/// docs (`.msg` store + workspace YAML, built-ins omitted, each carrying its
/// nested-custom `deps`) plus the `schema_hash` → qualified-name reverse
/// bindings for the SAME set.
///
/// Split out because `cerulion bag record` has no graph — it taps whatever
/// is live — but needs exactly this half to stamp a recording's schema
/// provenance. Everything a graph contributes (the topic→name catalog bindings,
/// the bridge mapping fold) stays in [`build_schema_serving`], so the two
/// callers cannot drift on WHAT a robot knows about its own types.
///
/// Never fails: an unreadable file is skipped with a `warn!` (the rest still
/// resolve).
pub fn build_schema_docs(
    schemas_dir: &Path,
    bridge_config_path: Option<&Path>,
) -> (Vec<SchemaDoc>, Vec<SchemaHashName>) {
    let bridge_cfg = bridge_config_path.and_then(parse_bridge_config);

    // 1. Collect every custom type the robot has (the served set) with its
    //    verbatim source text + parsed IR. The `.msg` store loads FIRST (the
    //    acquired-truth store), then workspace YAML — which REPLACES a
    //    store definition of the same qualified name (the desk resolves
    //    built-ins → store → YAML with YAML winning the string, so a served
    //    collision that kept the store definition disagreed with every local
    //    surface and could hand the desk the wrong schema).
    let mut docs_src: BTreeMap<String, DocSrc> = BTreeMap::new();

    load_msg_store(&mut docs_src, schemas_dir, true);

    // Fold the bridge's own `.msg` store dirs (GENERAL — a bridge may
    // point its `msg_dirs` at a store other than `schemas/`). In the common
    // `../schemas` case this resolves to `schemas_dir` and every type is already
    // loaded, so it is a no-op; the workspace store WINS a collision (`overwrite
    // = false`).
    if let (Some(cfg), Some(path)) = (&bridge_cfg, bridge_config_path) {
        for dir in bridge_msg_store_dirs(path, cfg) {
            load_msg_store(&mut docs_src, &dir, false);
        }
    }

    // An AMBIGUOUS workspace spelling — declared
    // by several files, or colliding with a stem file — is served by NOTHING
    // (no document under that name, no hash binding), said once per name; a
    // last-wins pick would hand the desk a document validation never bound.
    let yaml_claims = crate::schema_cmd::workspace_yaml_bare_claims_of(
        &crate::schema_cmd::parsed_workspace_yaml(schemas_dir),
    );
    for ambiguous in crate::schema_cmd::refused_claims(&yaml_claims) {
        ambiguous.warn("schema serving");
    }
    let refused: std::collections::BTreeSet<&str> = crate::schema_cmd::refused_claims(&yaml_claims)
        .map(|a| a.name.as_str())
        .collect();
    for path in workspace_yaml_files(schemas_dir) {
        let text = match std::fs::read_to_string(&path) {
            Ok(text) => text,
            Err(e) => {
                tracing::warn!(file = %path.display(), error = %e,
                    "schema-serving: could not read a workspace YAML schema — skipped");
                continue;
            }
        };
        let schemas = match parse_message_schemas(&text) {
            Ok(schemas) => schemas,
            Err(e) => {
                tracing::warn!(file = %path.display(), error = %e,
                    "schema-serving: could not parse a workspace YAML schema — skipped \
                     (the claims walk above already warned)");
                continue;
            }
        };
        for schema in schemas {
            // The refused set is keyed canonically; the entry keeps its
            // declared spelling.
            if refused.contains(crate::schema_cmd::normalize_schema(&schema.name).as_str()) {
                continue;
            }
            // Keyed by the ONE canonical identity — a YAML
            // entry spelled `pkg::Type` is served under `pkg/Type`, the key
            // `bind_output_schema_name` normalizes a declaration to; the raw
            // text is still the doc.
            let q = crate::schema_cmd::normalize_schema(&schema.qualified_name());
            // Workspace YAML wins a qualified-name collision (the
            // same precedence as `ResolutionTiers`, built-ins → store → YAML);
            // a YAML file may hold several schemas, each served under its own
            // qualified name (the whole file's text is the doc — the desk's
            // YAML parser dedups on re-parse).
            docs_src.insert(
                q,
                DocSrc {
                    encoding: SchemaEncoding::Yaml,
                    text: text.clone(),
                    schema,
                },
            );
        }
    }

    // A refused QUALIFIED string's `.msg` store twin stays SERVED: the served
    // set is also the dependency closure other types nest through, and
    // validation already refuses every port that could bind the string, so
    // the store document names nothing a port can reach yet still decodes a
    // nested field of a sibling type. Only the YAML tier's documents of a
    // refused name are withheld (above).

    // 2. The served set = the qualified names; deps resolve ONLY against it.
    let served: BTreeSet<String> = docs_src.keys().cloned().collect();
    let schema_docs: Vec<SchemaDoc> = docs_src
        .iter()
        .map(|(qualified, src)| SchemaDoc {
            qualified: qualified.clone(),
            encoding: src.encoding,
            text: src.text.clone(),
            deps: deps_of(&src.schema, &served),
        })
        .collect();

    // 2b. The hash→qualified-name reverse bindings — the
    //     gateway names a RUNTIME-registered topic (a `ros2 attach` raw route,
    //     absent from the node-output `topic_schemas` below) by resolving ITS
    //     reg-channel `schema_hash` through this map. Compute each served type's
    //     recipe-3 `schema_hash` the SAME way the wire does
    //     (`MessageSchema::schema_hash`), resolving nested FIXED targets against
    //     the built-in corpus first (so a custom type nesting e.g. `std_msgs/Header`
    //     folds the same target hash the producer's generated type stamped — an
    //     unresolved fold would mis-hash and the topic would stay unnamed). The
    //     built-ins are hashed here only for that resolution; THIS fn returns the
    //     CUSTOM slice (the recorder merges `builtin_hash_bindings()` itself, and
    //     a workspace-shadow test isolates the custom hash through this seam),
    //     while `build_schema_serving` appends the built-in bindings for the
    //     gateway. A hash collision would just prefer the last
    //     binding (never wrong-served — the desk re-parses the fetched closure
    //     regardless).
    //
    //     The served docs come from the
    //     workspace `.msg` store — untrusted input — and `resolve_fixed_nested`
    //     PANICS materializing a composed-overflow doc referenced as a fixed
    //     target, while `schema_hash` panics on a declared overflow. The
    //     sizable retain thins the CUSTOM tier (built-ins pass trivially) and
    //     the composed probe runs over the COMBINED set with the tiers scrubbed
    //     by the shared `RemovedDefinitions` predicate until nothing more
    //     drops — so the rebuilt set is EXACTLY `builtins ++ customs` and the
    //     custom slice below stays index-exact (a hostile custom shadowing a
    //     built-in sweeps its twin out of BOTH tiers).
    let mut builtins: Vec<MessageSchema> = parse_builtin_schemas();
    let mut customs: Vec<MessageSchema> = docs_src.values().map(|src| src.schema.clone()).collect();
    //     The wire CEILING is judged ONCE here, on the RESOLVED
    //     bindings below — a declared retain run before the
    //     probe would drop an over-ceiling nested TARGET before composition, so
    //     its parent would resolve with that reference left variable and BIND a
    //     hash codegen never emits.
    //     Resolution only grows a fixed section, so a declared-over doc is
    //     resolved-over and leaves at the resolved filter with its parents.
    crate::schema_cmd::retain_sizable_schemas(&mut customs, "schema serving hash bindings");
    let mut hash_schemas: Vec<MessageSchema> = loop {
        let mut combined: Vec<MessageSchema> =
            builtins.iter().chain(customs.iter()).cloned().collect();
        let removed = crate::schema_cmd::retain_composed_sizable_schemas(
            &mut combined,
            "schema serving hash bindings (composed preflight)",
        );
        if removed.is_empty() {
            break combined;
        }
        let alive = |s: &MessageSchema| !removed.removes(s);
        let before = builtins.len() + customs.len();
        builtins.retain(alive);
        customs.retain(alive);
        // NO-PROGRESS FALLBACK. Termination cannot rest on "every dropped
        // definition is matched by `removes`": if that ever fails,
        // the loop rebuilds an identical set forever.
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
        // (Breaking here would ALSO have broken this loop's own index
        // contract — `custom_start` below is derived from `builtins.len()`,
        // which a break with the scrubbed `combined` would leave stale, so
        // the custom slice would mis-align or panic. Looping keeps the one
        // exit that maintains it.)
        if builtins.len() + customs.len() == before {
            tracing::warn!(
                dropped = removed.removed_definitions.len(),
                "a composed-overflow removal matched no schema in the tiers — \
                 force-scrubbing by identity alone so the fixpoint makes \
                 progress; a healthy same-identity sibling may be dropped with \
                 the offender"
            );
            let by_identity = |s: &MessageSchema| !removed.removes_identity(s);
            builtins.retain(by_identity);
            customs.retain(by_identity);
        }
    };
    let custom_start = builtins.len();
    let _ = resolve_fixed_nested(&mut hash_schemas);
    // The RESOLVED-moment ceiling on the bindings too — a custom
    // doc that crosses the wire only through fixed-nested inlining (or a
    // still-variable nested reference's offset entry) passed the declared
    // retain above and must not bind a hash the gateway would then NAME a
    // topic by; the served TEXT still carries every file (the desk
    // re-parses it and runs its own walker guards).
    let schema_hashes: Vec<SchemaHashName> = hash_schemas[custom_start..]
        .iter()
        .filter(|s| {
            crate::schema_cmd::wire_representable_or_warn(
                s,
                "schema serving hash bindings (resolved)",
                crate::schema_cmd::SizeMoment::Resolved,
            )
        })
        .map(|s| SchemaHashName {
            schema_hash: s.schema_hash(),
            // The same canonical identity the docs are keyed by.
            qualified: crate::schema_cmd::normalize_schema(&s.qualified_name()),
        })
        .collect();

    (schema_docs, schema_hashes)
}

/// The served-doc key a graph output's `schema:` binds to
/// (the ambiguity-refusal rule): a qualified name
/// passes through; a bare name the workspace-YAML tier claims binds the
/// DOCUMENT that claim names — an entry claim binds itself, a file-STEM
/// claim binds the sole entry OF THAT FILE (`Goal.yaml` declaring only
/// `Other` serves `Other`; keeping `Goal` would bind a key no served document
/// carries, so the gateway could not serve or decode a port validation
/// accepts). A bound stem's entry has ONE declarer — the owner — because
/// every other shape (a same-named entry in another file, a multi-entry
/// file, a sole entry whose own name is ambiguous) is a REFUSED claim: the
/// declared name then passes through UNSERVEABLE, the refusal having been
/// said when `build_schema_docs` built the served set (this binding runs
/// after it in `build_schema_serving`; a caller using the binding half
/// alone would bind silently — keep the order). A bare name exactly one
/// parsed store schema bears becomes that definition's `pkg/Type` — the key
/// `build_schema_docs` serves it under. Anything else (an unknown bare name,
/// which validation refuses) passes through.
///
/// The declared name is normalized first:
/// `pkg::Type` is a documented spelling `resolve_port_schema`
/// accepts, and the served docs and hash bindings are keyed `pkg/Type` —
/// a `::` spelling detected as bare and returned verbatim is a binding no
/// lookup can hit. `parse_graph` already canonicalizes `schema:` on
/// deserialization, so this reaches a programmatically-built config only;
/// the ONE normalizer (`schema_cmd::normalize_schema`) runs here regardless,
/// for detection, lookup and the returned binding alike.
fn bind_output_schema_name(
    declared: &str,
    yaml_claims: &BTreeMap<String, crate::schema_cmd::YamlClaim>,
    unique_bare_store: &BTreeMap<String, String>,
) -> String {
    use crate::schema_cmd::YamlClaim;
    let declared = crate::schema_cmd::normalize_schema(declared);
    if declared.contains('/') {
        return declared;
    }
    match yaml_claims.get(&declared) {
        Some(YamlClaim::Entry { .. }) => declared,
        // A bound stem's entry has ONE declarer — the owner — which is what
        // the served set keys the document under (every other shape is a
        // refused claim). NORMALIZED, like every other arm and like the
        // replay registry's twin (`normalize_schema(binding.entry)`): a claim
        // keeps the entry's DECLARED spelling, while the served
        // documents and hash bindings a few lines above are keyed
        // `normalize_schema(&schema.qualified_name())` — so returning a
        // `pkg::Type` entry verbatim bound the topic to a name the gateway
        // has no document and no hash for, and nothing on the desk could
        // decode it.
        Some(YamlClaim::Stem { entry, .. }) => crate::schema_cmd::normalize_schema(entry),
        // Refused on every surface — the
        // declared name passes through, served by nothing (the refusal was
        // said when the served set was built).
        Some(YamlClaim::Refused(_)) => declared,
        None => unique_bare_store
            .get(&declared)
            .cloned()
            .unwrap_or(declared),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use cerulion_core::codegen::parse_rosmsg;

    /// A graph output declared
    /// `pkg::Type` — the documented alternate spelling `resolve_port_schema`
    /// accepts — must not be detected as BARE by the `/` probe and returned verbatim,
    /// a binding no served doc or hash binding (keyed `pkg/Type`) could hit.
    /// `parse_graph` canonicalizes `schema:` on deserialization, so the shape
    /// reaches this seam only through a programmatically-built config — which
    /// is exactly how the test builds it (the parsed config is mutated); the
    /// ONE normalizer runs at the top for detection, lookup and the
    /// returned binding. Pinned for a store-typed and a YAML-slash-typed
    /// output; the served doc under the canonical key exists for both.
    #[test]
    fn a_double_colon_spelled_output_binds_its_canonical_served_key() {
        let tmp = tempfile::tempdir().unwrap();
        let msg_dir = tmp.path().join("pkg").join("msg");
        std::fs::create_dir_all(&msg_dir).unwrap();
        std::fs::write(msg_dir.join("Type.msg"), "uint32 a\n").unwrap();
        std::fs::write(
            tmp.path().join("ws.yaml"),
            "schemas:\n  acme/Widget:\n    fields:\n      uint32 w:\n",
        )
        .unwrap();
        let mut config = cerulion_core::graph::parse_graph(
            "name: s24\nprefix: s24\nnodes:\n  - id: n\n    type: n\n    outputs:\n      - name: t\n        schema: pkg/Type\n      - name: w\n        schema: acme/Widget\n",
        )
        .unwrap();
        // The parser already canonicalized; re-introduce the alternate
        // spelling the way a programmatic builder could.
        config.nodes[0].outputs[0].schema = "pkg::Type".to_string();
        config.nodes[0].outputs[1].schema = "acme::Widget".to_string();
        let serving = build_schema_serving(&config, tmp.path(), None);
        let binding = |topic: &str| {
            serving
                .topic_schemas
                .iter()
                .find(|t| t.topic == topic)
                .unwrap_or_else(|| panic!("binding for {topic}"))
                .schema_name
                .clone()
        };
        assert_eq!(
            binding("/s24/n/t"),
            "pkg/Type",
            "canonical, never `pkg::Type`"
        );
        assert_eq!(binding("/s24/n/w"), "acme/Widget");
        assert!(serving
            .schema_docs
            .iter()
            .any(|d| d.qualified == "pkg/Type"));
        assert!(serving
            .schema_docs
            .iter()
            .any(|d| d.qualified == "acme/Widget"));
    }

    /// Schema provenance on the serving surface: `Goal.yaml`'s sole
    /// entry `Other` is also declared by a lexically-LATER file, and the served
    /// set keys YAML documents by entry name with the later file winning — so
    /// binding the stem to `Other` would hand the desk the OTHER file's
    /// document for a port validation binds to `Goal.yaml`. The port passes
    /// through UNSERVED instead, loudly; the mirror-image workspace (the owner
    /// is the later file, so it IS the served winner) binds `Other` — the
    /// same rule, both outcomes pinned.
    #[test]
    #[tracing_test::traced_test]
    fn a_stem_whose_entry_a_later_file_also_declares_passes_through_unserved() {
        let tmp = tempfile::tempdir().unwrap();
        std::fs::write(
            tmp.path().join("Goal.yaml"),
            "schemas:\n  Other:\n    fields:\n      uint32 a:\n",
        )
        .unwrap();
        std::fs::write(
            tmp.path().join("zz_other.yaml"),
            "schemas:\n  Other:\n    fields:\n      uint32 z:\n      uint32 y:\n",
        )
        .unwrap();
        let config = cerulion_core::graph::parse_graph(
            "name: s24\nprefix: s24\nnodes:\n  - id: n\n    type: n\n    outputs:\n      - name: g\n        schema: Goal\n",
        )
        .unwrap();
        // `Goal.yaml`'s sole entry `Other` is
        // declared by another file — `Goal` is REFUSED and served by nothing,
        // and the duplicate `Other` is served by nothing either.
        let serving = build_schema_serving(&config, tmp.path(), None);
        assert_eq!(
            serving.topic_schemas[0].schema_name, "Goal",
            "unserved pass-through — never either file's `Other`"
        );
        assert!(!serving.schema_docs.iter().any(|d| d.qualified == "Goal"));
        assert!(
            !serving.schema_docs.iter().any(|d| d.qualified == "Other"),
            "a duplicate entry name is served by nothing"
        );
        assert!(
            logs_contain("claims this name ambiguously"),
            "said loudly, with the ONE message every surface uses"
        );

        // Mirror image (the owner is the later file): the same refusal — no
        // precedence rescues a duplicate.
        let tmp2 = tempfile::tempdir().unwrap();
        std::fs::write(
            tmp2.path().join("AA_other.yaml"),
            "schemas:\n  Other:\n    fields:\n      uint32 z:\n      uint32 y:\n",
        )
        .unwrap();
        std::fs::write(
            tmp2.path().join("Goal.yaml"),
            "schemas:\n  Other:\n    fields:\n      uint32 a:\n",
        )
        .unwrap();
        let serving = build_schema_serving(&config, tmp2.path(), None);
        assert_eq!(
            serving.topic_schemas[0].schema_name, "Goal",
            "refused whichever file is later"
        );
        assert!(!serving.schema_docs.iter().any(|d| d.qualified == "Other"));
    }

    /// Two shapes (A and B), both pinned under the ambiguity-refusal
    /// rule. (A) With `Goal.yaml` declaring only `Other` plus an
    /// unrelated `other.yaml` declaring an entry named `Goal`, the stem
    /// is NOT authoritative; the name is refused on every
    /// surface, both sources named — the resolver and `schema info` err, the
    /// binding passes through unserved and no document is minted under it
    /// (pinned here on the resolver, `schema info`, serving and the hash
    /// map). (B) a parseable `Foo.yaml` whose sole entry is keyed `""` would bind
    /// a port `Foo` to the EMPTY schema key; the empty key is refused at
    /// the ONE parse gate, so the file is unparseable everywhere — validation
    /// refuses `Foo`, no claim exists, and a port that names it passes
    /// through unbound (never `""`).
    #[test]
    fn a_stem_beside_another_files_entry_is_refused_everywhere_and_an_empty_key_is_refused() {
        let tmp = tempfile::tempdir().unwrap();
        std::fs::write(
            tmp.path().join("Goal.yaml"),
            "schemas:\n  Other:\n    fields:\n      uint32 a:\n",
        )
        .unwrap();
        std::fs::write(
            tmp.path().join("other.yaml"),
            "schemas:\n  Goal:\n    fields:\n      uint32 z:\n      uint32 y:\n",
        )
        .unwrap();
        std::fs::write(
            tmp.path().join("Foo.yaml"),
            "schemas:\n  \"\":\n    fields:\n      uint32 a:\n",
        )
        .unwrap();
        // (A) `Goal` names both a schema ENTRY
        // (other.yaml) and a FILE of that stem (Goal.yaml) — the resolver and
        // `schema info` REFUSE it with one message naming both; the earlier
        // stem-authoritative pick is gone.
        let err = crate::schema_cmd::resolve_port_schema(tmp.path(), "Goal")
            .expect_err("refused")
            .to_string();
        assert!(
            err.contains("'Goal' is ambiguous in this workspace — defined by:")
                && err.contains(
                    "schemas/other.yaml (entry Goal), schemas/Goal.yaml (file stem; entries: Other)"
                ),
            "got: {err}"
        );
        assert_eq!(
            crate::schema_cmd::schema_info_unified(tmp.path(), "Goal")
                .expect_err("schema info prints the refusal")
                .to_string(),
            err
        );
        // (B) The empty key is refused at the parse gate → the stem file is
        // unparseable → validation refuses `Foo`.
        let err = crate::schema_cmd::parse_message_schemas(
            &std::fs::read_to_string(tmp.path().join("Foo.yaml")).unwrap(),
        )
        .expect_err("an empty entry key is refused");
        assert!(err.to_string().contains("non-empty"), "got: {err}");
        assert!(crate::schema_cmd::port_schema_exists(tmp.path(), "Foo").is_err());

        let config = cerulion_core::graph::parse_graph(
            "name: s23\nprefix: s23\nnodes:\n  - id: n\n    type: n\n    outputs:\n      - name: g\n        schema: Goal\n      - name: f\n        schema: Foo\n",
        )
        .unwrap();
        let serving = build_schema_serving(&config, tmp.path(), None);
        let binding = |topic: &str| {
            serving
                .topic_schemas
                .iter()
                .find(|t| t.topic == topic)
                .unwrap_or_else(|| panic!("binding for {topic}"))
                .schema_name
                .clone()
        };
        assert_eq!(
            binding("/s23/n/g"),
            "Goal",
            "refused — unserved pass-through, never either file's document"
        );
        assert!(
            serving.schema_docs.iter().any(|d| d.qualified == "Other"),
            "the file's own entry, declared once, is still served"
        );
        assert!(
            !serving.schema_docs.iter().any(|d| d.qualified == "Goal"),
            "the duplicated/colliding name is served by nothing"
        );
        assert_eq!(
            binding("/s23/n/f"),
            "Foo",
            "unbound pass-through — never the empty key"
        );
        assert!(!serving.schema_docs.iter().any(|d| d.qualified.is_empty()));
    }

    /// Keeping a bare `Goal` binding when
    /// `Goal.yaml` parses but declares only `Other` is wrong — validation accepts the
    /// port through the file-STEM tier, but `build_schema_docs` keys YAML
    /// documents by ENTRY name, so no `Goal` document exists and the gateway
    /// could neither serve nor decode a valid port. The binding follows
    /// the claim to the document it names: the stem's sole entry (`Other`),
    /// asserted to hit a served document AND a hash binding (the YAML
    /// definition's hash, hand oracle) — the choice consistent with
    /// `ResolutionTiers`, which indexes YAML by entry name and has no stem
    /// identity (minting a document under the stem would create a second
    /// identity for one schema). A stem whose file declares SEVERAL entries
    /// binds nothing single: the declared name passes through unserveable,
    /// loudly — and, under the ambiguity-refusal rule, REFUSED at validation
    /// as well (pinned: the resolver errs, the binding passes through).
    #[test]
    #[tracing_test::traced_test]
    fn a_file_stem_claimed_port_binds_the_files_sole_entry_document() {
        let tmp = tempfile::tempdir().unwrap();
        let msg_dir = tmp.path().join("nav22").join("msg");
        std::fs::create_dir_all(&msg_dir).unwrap();
        std::fs::write(msg_dir.join("Goal.msg"), "float64 x\nfloat64 y\n").unwrap();
        std::fs::write(
            tmp.path().join("Goal.yaml"),
            "schemas:\n  Other:\n    fields:\n      uint32 a:\n",
        )
        .unwrap();
        std::fs::write(
            tmp.path().join("Multi.yaml"),
            "schemas:\n  A:\n    fields:\n      uint32 a:\n  B:\n    fields:\n      uint32 b:\n",
        )
        .unwrap();
        // PREMISE: `Goal.yaml` and the `.msg` store both spell `Goal`, so the
        // SPELLING is refused as the twin — while `Multi` names one
        // FILE and resolves there. What no map can do for either is pick a
        // document, which is what the serving assertions below pin.
        let twin = crate::schema_cmd::resolve_port_schema(tmp.path(), "Goal")
            .expect_err("Goal is the YAML/store twin")
            .to_string();
        assert!(
            twin.contains("'Goal' is ambiguous in this workspace — defined by:")
                && twin.contains("schemas/Goal.yaml (file stem; entries: Other)")
                && twin.contains("schemas/nav22/msg/Goal.msg (store)"),
            "got: {twin}"
        );
        assert!(matches!(
            crate::schema_cmd::resolve_port_schema(tmp.path(), "Multi")
                .expect("a multi-entry stem names ONE file")
                .provenance,
            crate::schema_cmd::PortSchemaProvenance::Workspace { .. }
        ));
        let config = cerulion_core::graph::parse_graph(
            "name: s22\nprefix: s22\nnodes:\n  - id: n\n    type: n\n    outputs:\n      - name: g\n        schema: Goal\n      - name: m\n        schema: Multi\n",
        )
        .unwrap();

        let serving = build_schema_serving(&config, tmp.path(), None);
        let binding = |topic: &str| {
            serving
                .topic_schemas
                .iter()
                .find(|t| t.topic == topic)
                .unwrap_or_else(|| panic!("binding for {topic}"))
                .schema_name
                .clone()
        };
        // Never "Goal": no served document carries that key.
        let goal = binding("/s22/n/g");
        assert_eq!(
            goal, "Other",
            "the stem-claimed port binds the file's sole entry"
        );
        assert!(
            serving.schema_docs.iter().any(|d| d.qualified == goal),
            "the resulting binding has a matching served document"
        );
        let other_hash = {
            let mut s = MessageSchema::new("Other");
            s.add_field(cerulion_core::codegen::FieldDef::new(
                "a",
                cerulion_core::codegen::FieldType::U32,
            ));
            s.schema_hash()
        };
        assert!(
            serving
                .schema_hashes
                .iter()
                .any(|h| h.qualified == goal && h.schema_hash == other_hash),
            "the hash lookup hits the bound document with the YAML definition's hash"
        );
        assert!(
            !serving.schema_docs.iter().any(|d| d.qualified == "Goal"),
            "no document is minted under the stem — the stem is not an identity"
        );
        // The multi-entry stem: accepted by validation, unserveable — the
        // refused at validation (ambiguity-refusal rule) — bound to the
        // declared name, no document.
        let multi = binding("/s22/n/m");
        assert_eq!(multi, "Multi");
        assert!(!serving.schema_docs.iter().any(|d| d.qualified == "Multi"));
        // The two assertions above hold even for a silent pass-through (a `None` arm
        // that returns the declared name and mints nothing); the serving surface
        // SAYING the refusal is what the next assertion pins.
        assert!(
            logs_contain("claims this name ambiguously") && logs_contain("schema serving"),
            "the served set's build says the refusal, in its own context"
        );
    }

    /// A stem-claimed
    /// port whose bound entry is declared with the `::` spelling binds the canonical name. A claim
    /// keeps the entry's DECLARED name while the served documents
    /// and hash bindings are keyed canonically, so returning it verbatim
    /// would bind the topic to a name the gateway serves NO document and NO hash
    /// for — nothing on the desk could decode that topic. The binding is
    /// normalized like every sibling seam (`replay_field_registry`
    /// does `normalize_schema(binding.entry)`).
    #[test]
    fn a_stem_claimed_port_whose_entry_is_double_colon_binds_the_canonical_name() {
        let tmp = tempfile::tempdir().unwrap();
        std::fs::write(
            tmp.path().join("Goal.yaml"),
            "schemas:\n  acme::Other:\n    fields:\n      uint32 a:\n",
        )
        .unwrap();
        let config = cerulion_core::graph::parse_graph(
            "name: s41\nprefix: s41\nnodes:\n  - id: n\n    type: n\n    outputs:\n      - name: g\n        schema: Goal\n",
        )
        .unwrap();

        let serving = build_schema_serving(&config, tmp.path(), None);
        let bound = serving
            .topic_schemas
            .iter()
            .find(|t| t.topic == "/s41/n/g")
            .expect("a binding for the stem-claimed port")
            .schema_name
            .clone();
        assert_eq!(
            bound, "acme/Other",
            "the binding is the CANONICAL identity, not the declared `::` spelling"
        );
        // THE PIN: the name the topic is bound to is one the gateway can
        // actually answer for — a served document AND a hash binding.
        assert!(
            serving.schema_docs.iter().any(|d| d.qualified == bound),
            "a served document carries the bound name; got {:?}",
            serving
                .schema_docs
                .iter()
                .map(|d| d.qualified.as_str())
                .collect::<Vec<_>>()
        );
        assert!(
            serving.schema_hashes.iter().any(|h| h.qualified == bound),
            "and a hash binding does too; got {:?}",
            serving
                .schema_hashes
                .iter()
                .map(|h| h.qualified.as_str())
                .collect::<Vec<_>>()
        );
    }

    /// A graph output declared as a unique bare
    /// store name (`Goal`) must not be bound verbatim: the served docs and hash
    /// bindings are keyed `nav21/Goal`, so the gateway could neither serve the
    /// doc nor hit the hash for that topic. The binding carries the
    /// qualified name — asserted to hit BOTH a served doc and a hash binding
    /// (the store definition's hash, hand oracle) — while a bare YAML-typed
    /// output stays bare and hits its own doc (the control).
    #[test]
    fn a_bare_store_typed_output_binds_its_qualified_name_so_the_gateway_can_serve_it() {
        let tmp = tempfile::tempdir().unwrap();
        let msg_dir = tmp.path().join("nav21").join("msg");
        std::fs::create_dir_all(&msg_dir).unwrap();
        std::fs::write(msg_dir.join("Goal.msg"), "float64 x\nfloat64 y\n").unwrap();
        std::fs::write(
            tmp.path().join("plain.yaml"),
            "schemas:\n  Plain:\n    fields:\n      uint32 a:\n",
        )
        .unwrap();
        let config = cerulion_core::graph::parse_graph(
            "name: s21\nprefix: s21\nnodes:\n  - id: n\n    type: n\n    outputs:\n      - name: g\n        schema: Goal\n      - name: p\n        schema: Plain\n",
        )
        .unwrap();

        let serving = build_schema_serving(&config, tmp.path(), None);
        let binding = |topic: &str| {
            serving
                .topic_schemas
                .iter()
                .find(|t| t.topic == topic)
                .unwrap_or_else(|| panic!("binding for {topic}"))
                .schema_name
                .clone()
        };
        // Never "Goal": a key no served doc or hash binding carries.
        let goal = binding("/s21/n/g");
        assert_eq!(
            goal, "nav21/Goal",
            "the bare store name binds its qualified key"
        );
        assert!(
            serving.schema_docs.iter().any(|d| d.qualified == goal),
            "the gateway can serve the doc under the bound name"
        );
        let goal_hash = parse_rosmsg("float64 x\nfloat64 y\n", "Goal", Some("nav21"))
            .unwrap()
            .schema_hash();
        assert!(
            serving
                .schema_hashes
                .iter()
                .any(|h| h.qualified == goal && h.schema_hash == goal_hash),
            "the hash lookup hits the bound name with the store definition's hash"
        );
        // CONTROL: the YAML-typed output stays bare and hits its own doc.
        let plain = binding("/s21/n/p");
        assert_eq!(plain, "Plain");
        assert!(serving.schema_docs.iter().any(|d| d.qualified == "Plain"));
    }

    /// Preloading the store and then
    /// `or_insert`ing workspace YAML would make a same-qualified collision serve the
    /// STORE definition while every local surface (built-ins → store → YAML)
    /// serves the YAML one — the desk could fetch a schema the robot itself
    /// never resolves to. So YAML replaces the store entry: the served text
    /// and the hash binding are the YAML definition's; the store-only
    /// control keeps its `.msg` doc.
    #[test]
    fn a_yaml_entry_replaces_a_same_qualified_store_definition_in_the_served_set() {
        let tmp = tempfile::tempdir().unwrap();
        let msg_dir = tmp.path().join("pkg").join("msg");
        std::fs::create_dir_all(&msg_dir).unwrap();
        std::fs::write(msg_dir.join("Type.msg"), "uint32 a\n").unwrap();
        std::fs::write(msg_dir.join("Other.msg"), "float64 x\n").unwrap();
        let yaml_text = "schemas:\n  pkg/Type:\n    fields:\n      uint32 b:\n      uint32 c:\n";
        std::fs::write(tmp.path().join("ptype.yaml"), yaml_text).unwrap();

        let (docs, hashes) = build_schema_docs(tmp.path(), None);
        let doc = |name: &str| docs.iter().find(|d| d.qualified == name).unwrap();
        assert_eq!(
            doc("pkg/Type").encoding,
            SchemaEncoding::Yaml,
            "YAML wins the collision"
        );
        assert_eq!(
            doc("pkg/Type").text,
            yaml_text,
            "the served bytes are the YAML definition"
        );
        assert_eq!(
            doc("pkg/Other").encoding,
            SchemaEncoding::Msg,
            "control: the store-only doc"
        );
        let yaml_hash = {
            let mut s = MessageSchema::new("pkg/Type");
            s.add_field(cerulion_core::codegen::FieldDef::new(
                "b",
                cerulion_core::codegen::FieldType::U32,
            ));
            s.add_field(cerulion_core::codegen::FieldDef::new(
                "c",
                cerulion_core::codegen::FieldType::U32,
            ));
            s.schema_hash()
        };
        let binding = |name: &str| {
            hashes
                .iter()
                .find(|h| h.qualified == name)
                .map(|h| h.schema_hash)
        };
        assert_eq!(
            binding("pkg/Type"),
            Some(yaml_hash),
            "the hash binding is the YAML definition's"
        );
        assert_eq!(docs.iter().filter(|d| d.qualified == "pkg/Type").count(), 1);
    }

    /// The robot-side twin of the overflow preflight: if the hash
    /// bindings resolved the workspace store's docs with no preflight, one
    /// hostile `.msg` in `schemas/<pkg>/msg/` (the composed-overflow shape)
    /// would PANIC schema serving at gateway boot. Pinned: no panic; `Outer` is
    /// dropped and gets NO binding; `Wrapper` survives with the ref variable
    /// (hash oracle = the definition hashed alone); the sane doc binds its
    /// resolved hash (nested-free, so equal to its single-file hash).
    #[test]
    fn hash_bindings_survive_a_hostile_composed_store_shape() {
        let tmp = tempfile::tempdir().unwrap();
        let msg_dir = tmp.path().join("hostile").join("msg");
        std::fs::create_dir_all(&msg_dir).unwrap();
        std::fs::write(msg_dir.join("Inner.msg"), "float64[536870907] v\n").unwrap();
        std::fs::write(msg_dir.join("Outer.msg"), "Inner[8589934592] arr\n").unwrap();
        std::fs::write(msg_dir.join("Wrapper.msg"), "Outer o\nfloat64 w\n").unwrap();
        std::fs::write(msg_dir.join("Sane.msg"), "float64 x\nfloat64 y\n").unwrap();

        // Without the preflight this PANICS inside resolve_fixed_nested.
        let (docs, hashes) = build_schema_docs(tmp.path(), None);
        assert!(
            docs.iter().any(|d| d.qualified == "hostile/Outer"),
            "the served TEXT still carries every file (the desk re-parses it)"
        );
        let binding = |name: &str| {
            hashes
                .iter()
                .find(|h| h.qualified == name)
                .map(|h| h.schema_hash)
        };
        assert_eq!(
            binding("hostile/Outer"),
            None,
            "a composed overflow gets no hash binding"
        );
        let wrapper_alone =
            parse_rosmsg("Outer o\nfloat64 w\n", "Wrapper", Some("hostile")).unwrap();
        assert_eq!(
            binding("hostile/Wrapper"),
            Some(wrapper_alone.schema_hash()),
            "the referencing doc binds with its ref variable"
        );
        let sane = parse_rosmsg("float64 x\nfloat64 y\n", "Sane", Some("hostile")).unwrap();
        assert_eq!(binding("hostile/Sane"), Some(sane.schema_hash()));
        assert!(
            !hashes.iter().any(|h| h.qualified.starts_with("std_msgs/")),
            "the custom slice stays index-exact — no built-in leaks into the served bindings"
        );
    }

    fn served_set(names: &[&str]) -> BTreeSet<String> {
        names.iter().map(|s| s.to_string()).collect()
    }

    /// A qualified nested ref resolves iff served; a built-in-qualified ref
    /// (`geometry_msgs/Point`) not in the served set is NOT a dep.
    #[test]
    fn resolve_dep_qualified() {
        let served = served_set(&["acme/Gadget", "acme/Gizmo"]);
        assert_eq!(
            resolve_dep(Some("acme"), "Gadget", Some("acme"), &served).as_deref(),
            Some("acme/Gadget")
        );
        // A qualified built-in ref (not served) → None.
        assert_eq!(
            resolve_dep(Some("geometry_msgs"), "Point", Some("acme"), &served),
            None
        );
    }

    /// A BARE ref resolves same-package first, then a unique bare match; a built-in
    /// bare `Header` (never served) → None (correctly omitted).
    #[test]
    fn resolve_dep_bare() {
        let served = served_set(&["acme/Gadget", "vendor/Thing"]);
        // Same-package bare `Gadget` in parent `acme`.
        assert_eq!(
            resolve_dep(None, "Gadget", Some("acme"), &served).as_deref(),
            Some("acme/Gadget")
        );
        // Cross-package UNIQUE bare `Thing` (not in parent pkg, unique elsewhere).
        assert_eq!(
            resolve_dep(None, "Thing", Some("acme"), &served).as_deref(),
            Some("vendor/Thing")
        );
        // Built-in bare `Header` — no served entry → None.
        assert_eq!(resolve_dep(None, "Header", Some("acme"), &served), None);
    }

    /// An AMBIGUOUS bare name (two packages define it) is NOT a dep — we refuse to
    /// guess (it would mis-decode). It falls through to hash-only, which claims nothing more.
    #[test]
    fn resolve_dep_ambiguous_bare_is_none() {
        let served = served_set(&["a/Pose2D", "b/Pose2D"]);
        assert_eq!(resolve_dep(None, "Pose2D", Some("c"), &served), None);
    }

    /// `deps_of` extracts a `.msg`'s nested-custom refs, omitting the built-in
    /// `std_msgs/Header` (not served) and deduping. Hand oracle.
    #[test]
    fn deps_of_msg_omits_builtin_and_dedups() {
        // `acme/Widget` nests two `acme/Gadget`s, one `acme/Gizmo`, one
        // `std_msgs/Header` (a built-in).
        let text = "\
std_msgs/Header header
acme/Gadget primary
acme/Gadget secondary
acme/Gizmo gizmo
";
        let schema = parse_rosmsg(text, "Widget", Some("acme")).expect("parse");
        let served = served_set(&["acme/Widget", "acme/Gadget", "acme/Gizmo"]);
        let deps = deps_of(&schema, &served);
        // Gadget appears twice in the .msg but is deduped; Header (builtin) omitted.
        assert_eq!(
            deps,
            vec!["acme/Gadget".to_string(), "acme/Gizmo".to_string()]
        );
    }

    /// The self-exclusion compares CANONICAL keys — a `::`-declared
    /// document whose field references its own canonical name (`acme/Node[]
    /// children` on `acme::Node`) lists no dependency, never itself.
    #[test]
    fn deps_of_excludes_self_under_the_canonical_key() {
        let mut schema = MessageSchema::new("acme::Node");
        schema.add_field(cerulion_core::codegen::FieldDef::new(
            "children",
            FieldType::DynamicArray {
                element_type: Box::new(FieldType::Nested {
                    schema_name: "Node".to_string(),
                    package: Some("acme".to_string()),
                    fixed: None,
                }),
            },
        ));
        let served = served_set(&["acme/Node", "acme/Other"]);
        assert_eq!(deps_of(&schema, &served), Vec::<String>::new());
        // The control: the SAME field on a document declared `acme/Other`
        // IS a dependency (it names another served doc).
        let mut other = MessageSchema::new_in_package("Other", "acme");
        other.add_field(cerulion_core::codegen::FieldDef::new(
            "child",
            FieldType::Nested {
                schema_name: "Node".to_string(),
                package: Some("acme".to_string()),
                fixed: None,
            },
        ));
        assert_eq!(deps_of(&other, &served), vec!["acme/Node".to_string()]);
    }

    /// A nested-CUSTOM inside an ARRAY field is still a dep (array unwrapping).
    #[test]
    fn deps_of_unwraps_arrays() {
        let text = "acme/Transform[] transforms\n";
        let schema = parse_rosmsg(text, "TFMessage", Some("acme")).expect("parse");
        let served = served_set(&["acme/TFMessage", "acme/Transform"]);
        assert_eq!(
            deps_of(&schema, &served),
            vec!["acme/Transform".to_string()]
        );
    }

    // ─────────────────────── Bridge discovery (pure) ───────────────────

    /// The env value WINS verbatim over the sibling when set + non-empty — even
    /// when the sibling exists (the env is what the bridge node itself loads).
    #[test]
    fn choose_bridge_config_env_wins_verbatim_over_existing_sibling() {
        let sibling = PathBuf::from("/ws/graphs/attach.bridge.yaml");
        let got = choose_bridge_config(
            Some(OsStr::new("/other/go2.bridge.yaml")),
            sibling,
            |_| true, // sibling exists — env still wins
        );
        assert_eq!(got, Some(PathBuf::from("/other/go2.bridge.yaml")));
    }

    /// An EMPTY env value is ignored (falls through to the sibling) — an
    /// accidentally-cleared var must not silently select "" as the config.
    #[test]
    fn choose_bridge_config_empty_env_falls_through_to_sibling() {
        let sibling = PathBuf::from("/ws/graphs/attach.bridge.yaml");
        let got = choose_bridge_config(Some(OsStr::new("")), sibling.clone(), |_| true);
        assert_eq!(got, Some(sibling));
    }

    /// With no env, the sibling is chosen ONLY when it exists.
    #[test]
    fn choose_bridge_config_sibling_only_when_it_exists() {
        let sibling = PathBuf::from("/ws/graphs/attach.bridge.yaml");
        assert_eq!(
            choose_bridge_config(None, sibling.clone(), |_| true),
            Some(sibling.clone())
        );
        // A normal graph (no bridge sibling) ⇒ None ⇒ the serving without the fold.
        assert_eq!(choose_bridge_config(None, sibling, |_| false), None);
    }

    /// [`bridge_config_from_env`] over the REAL process env — the whole
    /// point of the helper is that `cerulion bag record` does not hand-roll
    /// this read, so the pure `choose_bridge_config` arms above cannot cover it.
    ///
    /// The gap this closes is specific: the helper passes a NEVER-EXISTS sibling
    /// predicate (`|_| false`), and mutating that to `|_| true` returns
    /// `Some(PathBuf::new())` for an UNSET env — a bare `""` path that then fails
    /// to read, warning "could not read the bridge config" on EVERY recording
    /// against a variable the operator never set. No other arm sees it.
    ///
    /// `""` is the second arm because that is the shape the hand-rolled read got
    /// wrong (`std::env::var(..).ok().map(PathBuf::from)` yields `Some("")`).
    #[test]
    fn bridge_config_from_env_reads_only_a_set_non_empty_value() {
        let _lk = crate::test_env::env_lock();
        let restore = std::env::var_os(BRIDGE_CONFIG_ENV);

        std::env::remove_var(BRIDGE_CONFIG_ENV);
        assert_eq!(
            bridge_config_from_env(),
            None,
            "an UNSET env must yield no path — a `Some(\"\")` here warns about a bridge \
             config the operator never set, on every recording"
        );

        std::env::set_var(BRIDGE_CONFIG_ENV, "");
        assert_eq!(
            bridge_config_from_env(),
            None,
            "an EMPTY env is not a path (this is what the hand-rolled read got wrong)"
        );

        std::env::set_var(BRIDGE_CONFIG_ENV, "/go2/attach.bridge.yaml");
        assert_eq!(
            bridge_config_from_env(),
            Some(PathBuf::from("/go2/attach.bridge.yaml")),
            "a SET value is taken verbatim — the anti-tautology arm: without it every \
             assertion above is satisfied by a helper that always returns None"
        );

        match restore {
            Some(v) => std::env::set_var(BRIDGE_CONFIG_ENV, v),
            None => std::env::remove_var(BRIDGE_CONFIG_ENV),
        }
    }

    /// `topic_dedup_key` strips a single leading `/` so an absolute graph binding
    /// and an absolute bridge `cerulion_topic` collide on the same key (graph
    /// wins), and defends the degenerate no-slash form.
    #[test]
    fn topic_dedup_key_strips_one_leading_slash() {
        assert_eq!(topic_dedup_key("/go2/odom"), "go2/odom");
        assert_eq!(topic_dedup_key("go2/odom"), "go2/odom");
        assert_eq!(topic_dedup_key("/"), "");
    }

    /// A bridge config's RELATIVE `msg_dirs` entries resolve against the config
    /// file's OWN dir (the bridge's supplement-A contract); absolute entries are
    /// verbatim; an absent key yields no dirs.
    #[test]
    fn bridge_msg_store_dirs_resolves_relative_against_config_dir() {
        let cfg: BridgeYamlLite =
            serde_yaml::from_str("msg_dirs: [../schemas, /abs/store]\nmappings: []\n").unwrap();
        let dirs = bridge_msg_store_dirs(Path::new("/ws/graphs/attach.bridge.yaml"), &cfg);
        assert_eq!(
            dirs,
            vec![
                PathBuf::from("/ws/graphs/../schemas"),
                PathBuf::from("/abs/store"),
            ]
        );
        // Absent msg_dirs ⇒ empty.
        let none: BridgeYamlLite = serde_yaml::from_str("mappings: []\n").unwrap();
        assert!(bridge_msg_store_dirs(Path::new("/ws/graphs/x.bridge.yaml"), &none).is_empty());
    }

    /// The lenient parse tolerates the real config's extra keys (`domain_id`,
    /// `qos`, `route`, `max_slice_len`, `dds_topic`) and reads only the two the
    /// fold needs.
    #[test]
    fn parse_bridge_config_tolerates_real_config_extra_keys() {
        let tmp = tempfile::tempdir().unwrap();
        let path = tmp.path().join("go2.bridge.yaml");
        std::fs::write(
            &path,
            "domain_id: 0\nonly_networks: []\nmsg_dirs: [../schemas]\nmappings:\n  \
             - dds_topic: /utlidar/robot_odom\n    ros_type: unitree_go/SportModeState\n    \
             cerulion_topic: /go2/utlidar/robot_odom\n    qos: best_effort\n    route: raw\n",
        )
        .unwrap();
        let cfg = parse_bridge_config(&path).expect("lenient parse ignores extra keys");
        assert_eq!(cfg.mappings.len(), 1);
        assert_eq!(cfg.mappings[0].cerulion_topic, "/go2/utlidar/robot_odom");
        assert_eq!(cfg.mappings[0].ros_type, "unitree_go/SportModeState");
    }

    /// A malformed / unreadable bridge config is a loud (best-effort) `None` —
    /// never a panic, never a half-read mapping table.
    #[test]
    fn parse_bridge_config_malformed_or_missing_is_none() {
        // Unreadable (nonexistent) path.
        assert!(parse_bridge_config(Path::new("/definitely/not/a/real/bridge.yaml")).is_none());
        // Malformed YAML.
        let tmp = tempfile::tempdir().unwrap();
        let bad = tmp.path().join("bad.bridge.yaml");
        std::fs::write(&bad, "mappings: [ this is : not valid : yaml\n").unwrap();
        assert!(parse_bridge_config(&bad).is_none());
        // A mapping MISSING `cerulion_topic` fails the whole parse (loud skip).
        let missing = tmp.path().join("missing.bridge.yaml");
        std::fs::write(
            &missing,
            "mappings:\n  - dds_topic: /x\n    ros_type: geometry_msgs/Twist\n",
        )
        .unwrap();
        assert!(parse_bridge_config(&missing).is_none());
    }
}
