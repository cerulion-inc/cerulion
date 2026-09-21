// SPDX-License-Identifier: AGPL-3.0-only
//! Graph validation for correctness checking.
//!
//! Validates topology, references, and slice lengths before a graph
//! is built into a runtime.
//!
//! Trigger-policy bound checks (period > 0, deadline > 0,
//! sync window > 0, dangling DataTrigger topic, self-referencing
//! trigger) used to live here when graph YAML carried a `policy:`
//! block per node. Trigger policy now lives only on the
//! macro side; the analogous checks now live in `cerulion_macros`
//! (compile-time enforcement of the macro attributes) and in
//! `runtime.rs::validate_no_silent_data_trigger` (the
//! macro-declared-trigger / FFI plumbing regression guard).
//! See `super::config::NodeDef` for the architectural rationale.
use std::collections::HashSet;

use indexmap::IndexMap;

use crate::error::{TransportError, TransportResult};
use crate::wire::WireHeader;

use super::config::{GraphConfig, NetworkMode};
use super::topology::MAX_CONSUMER_DEPTH;
use super::{malformed_absolute_name, resolve_output_topic, resolve_source};

/// Diagnostic message fragment emitted when `max_slice_len` exceeds
/// `u32::MAX`. Hoisted into a `pub(crate) const` so the
/// `format!` site and the test's `err.contains(...)` assertion
/// reference the same string — a future refactor that rewords the
/// diagnostic must update both, preventing the test silently passing
/// while the user-visible message drifts.
///
/// Note: `runtime.rs::CLAMP_MESSAGE_FRAGMENT` is
/// gone (the runtime no longer clamps; tier-1 oversize falls through
/// to tier-3 with a `tracing::error!` — OOM/typo on the
/// unreachable-via-validation-gate path is unrecoverable, not
/// informational). This validation fragment is now the sole
/// user-visible string for the oversize case at graph-load.
pub(crate) const VALIDATION_OVERSIZED_FRAGMENT: &str = "exceeds wire format ceiling u32::MAX";

/// Is this `schema:` value a well-formed schema NAME?
///
/// PURE, and deliberately about SHAPE rather than about existence — see the
/// call site in [`validate_graph`] for why `cerulion_core` cannot ask the
/// second question. Returns the offending property (a sentence fragment that
/// completes "`schema: X` …") so the caller owns the surrounding wording.
///
/// Two spellings are legal, and BOTH are accepted here:
///
/// * qualified — `pkg/Type`, or `pkg::Type` (`OutputDef`'s
///   `deserialize_with = "normalize_schema_separator"` rewrites `::` to `/`
///   on the YAML read path, but a `GraphConfig` can also be built IN MEMORY
///   — `ros2 attach`'s generator, the CLI's `build_node_def`, every test that
///   constructs the struct directly — where no normalization runs, so a
///   checker that only knew `/` would refuse a value the parser accepts);
/// * bare — a single segment naming a schema in this workspace's
///   `schemas/`. `schema_cmd::resolve_port_schema` documents the bare form
///   as tier 3 of its ladder ("a bare workspace name is never silently
///   hijacked to a same-named built-in"), and two shipped graphs use it, so
///   requiring a `/` here would be a FALSE refusal of a documented spelling.
///
/// What it rejects is what no legal spelling can carry: emptiness,
/// whitespace, control characters, the zenoh-reserved set (a schema name
/// reaches Studio decoders and bag channel names), an empty path segment
/// (`/Image`, `sensor_msgs/`, `a//b`), a stray `:` left by a half-typed
/// `::`, and more than one separator.
pub(crate) fn validate_schema_value(schema: &str) -> Result<(), String> {
    if schema.is_empty() {
        return Err("is missing".to_string());
    }
    if schema.trim().is_empty() {
        return Err("is blank".to_string());
    }
    if schema.chars().any(char::is_whitespace) {
        return Err("contains whitespace".to_string());
    }
    if schema.chars().any(char::is_control) {
        return Err("contains a control character".to_string());
    }
    if schema.contains(['*', '?', '#', '$', '@']) {
        return Err(
            "contains a reserved character ('*', '?', '#', '$', '@') — a schema name reaches \
             zenoh key expressions, bag channel names and Studio decoders"
                .to_string(),
        );
    }
    // Fold the alternate `::` spelling before segmenting, so `a::b` and `a/b`
    // are judged identically (see the doc above on in-memory configs).
    let normalized = schema.replace("::", "/");
    if normalized.contains(':') {
        return Err("contains a stray ':' — the package separator is '/' (or '::')".to_string());
    }
    let segments: Vec<&str> = normalized.split('/').collect();
    if segments.iter().any(|s| s.is_empty()) {
        return Err(
            "has an empty segment — expected 'pkg/Type' (or a bare workspace schema name)"
                .to_string(),
        );
    }
    if segments.len() > 2 {
        return Err(format!(
            "has {} package separators — expected 'pkg/Type' (or a bare workspace schema name)",
            segments.len() - 1
        ));
    }
    Ok(())
}

/// Render the `schema:` CLAUSE of a diagnostic, so an absent value does not
/// print as a bare trailing space. An empty value is the COMMON case here
/// (`OutputDef.schema` is `#[serde(default)]`, so an omitted key yields
/// `""`), and `` `schema: ` is missing `` reads like a formatting bug rather
/// than like a message — so the absent case names the KEY and the present
/// case quotes the VALUE.
fn render_schema_for_diagnostic(schema: &str) -> String {
    if schema.is_empty() {
        "`schema:`".to_string()
    } else {
        format!("`schema: {schema}`")
    }
}

/// Which checks [`validate_graph_with`] runs. Every field defaults to the
/// STRICTEST setting, so [`validate_graph`] — and therefore
/// `GraphRuntime::build`, `graph run` and `graph validate` — is unaffected by
/// this type's existence.
#[derive(Debug, Clone, Copy)]
pub struct ValidationOptions<'a> {
    /// Require every output to declare a well-formed
    /// `schema:`. `true` everywhere a graph is RUN or VALIDATED.
    ///
    /// `false` has exactly ONE caller — `cerulion node stage` — and the reason
    /// is a format limitation rather than a preference. The CLI fills a staged
    /// output's schema from `node_metadata::PortDef::schema`, which is `None`
    /// for every RAW-FFI node: the `CERULION:INFO_START` document can express
    /// a port's `schema_hash` but has NO field for a schema NAME (the info-JSON
    /// port types in `super::node` are `deny_unknown_fields` over
    /// `name` / `schema_hash` / `max_slice_len_default` / `promise_within_ms`),
    /// and `cerulion node create --raw-ffi` scaffolds outputs as bare name
    /// strings carrying neither. Refusing at stage time would therefore brick
    /// an entire node class that our own scaffolding generates.
    ///
    /// Deriving the name from the hash was the preferred remedy and does not
    /// work HERE: `LayoutResolver::schema_name_for_hash` exists and would do
    /// it, but `node stage` reads SOURCE, and the source of a scaffolded
    /// raw-FFI node carries no hash to look up — the hash lives in the
    /// COMPILED cdylib's `OutputMeta`, which staging (normally run before
    /// `node build`) has not got.
    ///
    /// So staging WARNS and writes the entry, and the graph is refused when it
    /// is RUN — loud at both ends, and never a silent default.
    pub require_output_schema: bool,
    /// The absolute topics a SIBLING process group of the same graph produces
    /// for this one. `None` everywhere except a multi-process worker.
    ///
    /// A worker validates only its own group's slice of the graph, in which a
    /// cross-group input has been rewritten to the absolute topic its producer
    /// publishes. That producer is not in the slice, so the source reads as an
    /// in-prefix absolute miss and would draw the "is this a typo?" warn, on a
    /// graph the full validation had just passed. The supervisor, which holds
    /// the whole graph, names those topics here. Anything NOT in the set still
    /// warns, so a genuine unknown in-prefix source stays loud in a worker.
    pub sibling_topics: Option<&'a std::collections::BTreeSet<String>>,
}

impl Default for ValidationOptions<'_> {
    fn default() -> Self {
        Self {
            require_output_schema: true,
            sibling_topics: None,
        }
    }
}

/// Validate graph configuration for correctness.
pub fn validate_graph(config: &GraphConfig) -> TransportResult<()> {
    validate_graph_with(config, ValidationOptions::default())
}

/// [`validate_graph`] with individual checks selectable. See
/// [`ValidationOptions`] for the one relaxable check and who relaxes it, and
/// for the one fact a multi-process worker is told about its sibling groups.
pub fn validate_graph_with(
    config: &GraphConfig,
    options: ValidationOptions<'_>,
) -> TransportResult<()> {
    if config.nodes.is_empty() {
        return Err(TransportError::GraphError {
            reason: "graph must have at least one node".to_string(),
        });
    }

    // Canonical topic names are /{prefix}/... — a prefix
    // that itself starts or ends with '/' (or is empty) would produce
    // empty path segments ("//"), which are invalid in iceoryx2 service
    // names' canonical shape and in zenoh key expressions.
    if config.prefix.is_empty()
        || config.prefix.starts_with('/')
        || config.prefix.ends_with('/')
        || config.prefix.contains("//")
    {
        return Err(TransportError::GraphError {
            reason: format!(
                "graph prefix '{}' must be non-empty and must not start/end \
                 with '/' or contain empty segments (canonical topics are \
                 '/{{prefix}}/{{node}}/{{output}}')",
                config.prefix
            ),
        });
    }

    // The prefix becomes part of EVERY derived topic name, so it must also
    // reject zenoh-reserved characters — exactly as `malformed_absolute_name`
    // (graph/mod.rs) does for absolute `topic:`/`source:` names. Kept a
    // SEPARATE check (not folded into the structural `||` above) so the message
    // names the real problem: a prefix like `cam*` is a reserved-char issue,
    // not a leading/trailing-slash one, and would otherwise fail opaquely at
    // runtime (iceoryx2/zenoh) instead of loudly at graph-load.
    if config.prefix.contains(['*', '?', '#', '$', '@']) {
        return Err(TransportError::GraphError {
            reason: format!(
                "graph prefix '{}' contains a zenoh-reserved character \
                 ('*', '?', '#', '$', '@') — the prefix is part of every \
                 derived topic name, which must be a valid zenoh key",
                config.prefix
            ),
        });
    }

    // Check for duplicate node IDs
    let mut seen_ids = HashSet::new();
    for node in &config.nodes {
        if !seen_ids.insert(&node.id) {
            return Err(TransportError::GraphError {
                reason: format!("duplicate node ID: '{}'", node.id),
            });
        }
    }

    // `ros2:` entries: the shape rules for the mixed-stack graph, and the
    // `type:` XOR `ros2:` contract for every entry. Runs BEFORE the port /
    // topology checks below so a ros2 entry with ports is refused by name
    // (with a suggested fix) rather than by a downstream "unknown output" message.
    validate_ros2_entries(config)?;

    // `multi_publisher_topics` entries must be ABSOLUTE,
    // well-formed, and unique. Derived names embed the node id and cannot
    // be shared by construction, so a relative entry is always a mistake
    // — reject with the did-you-mean rather than silently never matching
    // any resolved topic.
    let mut seen_multi = HashSet::new();
    for t in &config.multi_publisher_topics {
        if !t.starts_with('/') {
            return Err(TransportError::GraphError {
                reason: format!(
                    "multi_publisher_topics entry '{t}' must be ABSOLUTE — did \
                     you mean '/{t}'? (derived names embed the node id and \
                     cannot be shared; the opt-in only applies to absolute \
                     topics like /tf)"
                ),
            });
        }
        if let Some(why) = malformed_absolute_name(t) {
            return Err(TransportError::GraphError {
                reason: format!("multi_publisher_topics entry '{t}' is malformed — {why}"),
            });
        }
        if !seen_multi.insert(t.as_str()) {
            return Err(TransportError::GraphError {
                reason: format!("multi_publisher_topics lists '{t}' more than once"),
            });
        }
    }

    // Build a set of all declared output topics.
    // Resolution routes through `resolve_output_topic`, so a `topic:`
    // override lands in the same set — override-vs-override and
    // override-vs-derived collisions are caught by the same
    // duplicate-topic check.
    let mut output_topics = HashSet::new();
    let mut output_topic_map: IndexMap<String, String> = IndexMap::new();
    for node in &config.nodes {
        for output in &node.outputs {
            // `schema:` is validated as a VALUE, not merely
            // as a key. `OutputDef.schema` is `#[serde(default)]`, so an
            // omitted key yields `""` — and nothing anywhere errored on it:
            // all four `schema.is_empty()` sites in the tree SKIP or DEGRADE.
            // The wire layout comes from the node's Rust type, so a wrong or
            // absent value cannot break publishing; the bill arrives later,
            // in three places a user will never connect back to this line —
            // a bag channel whose label resolves to nothing or to the wrong
            // definition (a `#[cerulion_node]` producer now
            // supplies both the hash and the fixed size, so on those the
            // NAME is the only wrong part; a node carrying no size still
            // takes the workspace's, or 0), a wrong Studio
            // decoder name, and a
            // FALSE exit-2 `SchemaDrift` refusal of a healthy bag when the
            // typo happens to resolve to a real-but-wrong built-in.
            //
            // This lives HERE, in `validate_graph`, rather than only in the
            // CLI's report, because `GraphRuntime::build` calls this function
            // — so the check is inherited by every run path and cannot be
            // downgraded to a log line (which is exactly what happened to the
            // report's own schema-match check before the run gate).
            //
            // SCOPE, and it is a layering fact rather than a choice:
            // `cerulion_core` cannot see the schema universe. Built-ins live
            // in `native_ros2_messages`, which depends on `cerulion_core`
            // (so it is a DEV-dependency here, never a real one), and
            // workspace schemas live under a workspace root this crate is
            // never told about. So core asserts what core can prove — the
            // value is present and is a well-formed NAME — and full
            // RESOLVABILITY stays in `cerulion_cli_engine`'s report check,
            // which the `graph run` gate makes fatal.
            // The relaxed arm skips ONLY the ABSENT case (see
            // `ValidationOptions::require_output_schema`). A value that is
            // PRESENT and malformed is a typo, which staging never produces
            // and which stays refused everywhere, unconditionally.
            let schema_check = if !options.require_output_schema && output.schema.is_empty() {
                Ok(())
            } else {
                validate_schema_value(&output.schema)
            };
            if let Err(why) = schema_check {
                return Err(TransportError::GraphError {
                    reason: format!(
                        "node '{}' output '{}': {} {} — every output must \
                         declare the message type it publishes (e.g. \
                         `schema: sensor_msgs/Image`, or a bare name for a \
                         schema in this workspace's `schemas/`)",
                        node.id,
                        output.name,
                        render_schema_for_diagnostic(&output.schema),
                        why
                    ),
                });
            }
            if let Some(t) = &output.topic {
                // The override MUST be absolute — `topic:` is not a
                // second relative-naming mechanism.
                if !t.starts_with('/') {
                    return Err(TransportError::GraphError {
                        reason: format!(
                            "node '{}' output '{}': `topic: {}` must be \
                             ABSOLUTE — did you mean `topic: /{}`? (relative \
                             topics are derived from the output name; the \
                             override exists for externally-fixed global \
                             names like /tf)",
                            node.id, output.name, t, t
                        ),
                    });
                }
                if let Some(why) = malformed_absolute_name(t) {
                    return Err(TransportError::GraphError {
                        reason: format!(
                            "node '{}' output '{}': `topic: {}` is malformed — {}",
                            node.id, output.name, t, why
                        ),
                    });
                }
            }
            let topic = resolve_output_topic(&config.prefix, &node.id, output);
            // A topic listed in `multi_publisher_topics`
            // legally carries multiple in-graph producers (e.g. two tf
            // broadcasters overriding to /tf) — the duplicate rejection
            // applies only to UNLISTED topics, where a second producer is
            // ambiguous.
            if !output_topics.insert(topic.clone()) && !config.is_multi_publisher(&topic) {
                return Err(TransportError::GraphError {
                    reason: format!(
                        "duplicate output topic: '{}' (if multiple publishers \
                         are intentional, list the topic in \
                         `multi_publisher_topics:`)",
                        topic
                    ),
                });
            }
            // Map short reference (node_id/output_name) to full topic
            let short_ref = format!("{}/{}", node.id, output.name);
            output_topic_map.insert(short_ref, topic);
        }
    }

    // Check for duplicate input names per node.
    // Two same-named inputs collide on every (node_id, input)
    // wiring key downstream: the second silently overwrites the first in
    // the runtime's subscriber IndexMap AND in the per-input backpressure
    // maps. For `block` that orphans the first edge's `outstanding` mirror
    // — the producer keeps incrementing a counter no drain ever decrements
    // and defers forever in release builds (debug builds previously tripped
    // the scheduler-side counter-registration debug_assert instead) — all
    // underneath the subscriber-level at-most-one-probe assert, because
    // each registration lands on a FRESH subscriber instance. The colliding
    // keys come from graph-YAML `InputDef`s, so EVERY node form is exposed
    // (macro, raw-FFI, closure): the macro's port DECLARATIONS can't
    // contain duplicates (Rust rejects duplicate struct fields), but
    // hand-edited YAML wiring a macro node can. Duplicate OUTPUT names need
    // no twin check: same node + same output name resolve to the same
    // topic, which the duplicate-output-topic check above already rejects.
    for node in &config.nodes {
        let mut seen_inputs = HashSet::new();
        for input in &node.inputs {
            if !seen_inputs.insert(&input.name) {
                return Err(TransportError::GraphError {
                    reason: format!(
                        "node '{}' declares duplicate input name '{}'",
                        node.id, input.name
                    ),
                });
            }
        }
    }

    // Validate input sources resolve to existing outputs.
    // An ABSOLUTE source (leading '/') that matches no in-graph output
    // is an EXTERNAL topic — legal for drop_oldest/sample consumers
    // (`GraphTopology::validate` rejects `block` on producer-less topics;
    // the runtime logs the External provisioning decision loudly). A
    // RELATIVE miss is still a hard error: relative references promise an
    // in-graph producer.
    for node in &config.nodes {
        for input in &node.inputs {
            if input.source.starts_with('/') {
                if let Some(why) = malformed_absolute_name(&input.source) {
                    return Err(TransportError::GraphError {
                        reason: format!(
                            "node '{}' input '{}': absolute source '{}' is malformed — {}",
                            node.id, input.name, input.source, why
                        ),
                    });
                }
                if output_topics.contains(&input.source) {
                    // Wired to an in-graph output (derived or overridden).
                    continue;
                }
                // This shipped as a hard REJECTION and was flipped to a
                // loud warn: two same-host graphs legitimately
                // share the hostname-default prefix — prefix is a
                // NAMESPACE, not a graph identity — so an in-prefix
                // absolute miss may be another graph's topic, not a
                // typo. The typo shape is still the common case, so the
                // warn keeps the full diagnosis + did-you-mean; the
                // topic provisions External like any other absolute
                // miss. The data-integrity half stays HARD: when two
                // graphs actually PUBLISH the same topic, the
                // single-writer pre-check at publisher creation errors
                // (see `create_publisher_with_topic_config`).
                // A source is "in this graph's prefix namespace" iff it
                // begins with the full `/{prefix}/` boundary. Match the
                // whole boundary, NOT just the first `/`-segment: the
                // prefix validator (above) permits internal single slashes,
                // so a multi-segment prefix like `my/robot` would make a
                // first-segment compare (`"my" == "my/robot"`) spuriously
                // false and silently suppress the in-prefix typo diagnosis.
                let in_prefix = input.source.starts_with(&format!("/{}/", config.prefix));
                if in_prefix {
                    // A worker's cross-group edge: the producer is real, it
                    // just lives in a sibling group's process (see
                    // `ValidationOptions::sibling_topics`). Not a typo.
                    if options
                        .sibling_topics
                        .is_some_and(|topics| topics.contains(&input.source))
                    {
                        continue;
                    }
                    // Did-you-mean: the DERIVED spelling of an output
                    // whose topic is overridden.
                    let mut hint = String::new();
                    for n in &config.nodes {
                        for o in &n.outputs {
                            if let Some(actual) = &o.topic {
                                let derived = format!("/{}/{}/{}", config.prefix, n.id, o.name);
                                if derived == input.source {
                                    hint = format!(
                                        " — output '{}/{}' publishes the absolute \
                                         topic '{}'; reference it as `source: {}`",
                                        n.id, o.name, actual, actual
                                    );
                                }
                            }
                        }
                    }
                    tracing::warn!(
                        node_id = %node.id,
                        input = %input.name,
                        source = %input.source,
                        prefix = %config.prefix,
                        hint = %hint,
                        "absolute source is inside this graph's own prefix \
                         namespace but matches no declared output — treating it \
                         as an EXTERNAL topic (another same-prefix graph may \
                         produce it); if this is a typo'd in-graph reference, \
                         fix the spelling"
                    );
                    continue;
                }
                // Out-of-prefix absolute miss: a genuinely EXTERNAL topic.
                continue;
            }
            let full_source = resolve_source(&config.prefix, &input.source);
            if !output_topics.contains(&full_source) {
                // A relative short-ref naming an output
                // whose topic is OVERRIDDEN misses the set (the derived
                // name no longer exists) — point at the absolute name
                // instead of claiming the output doesn't exist.
                if let Some(actual) = output_topic_map.get(&input.source) {
                    return Err(TransportError::GraphError {
                        reason: format!(
                            "node '{}' input '{}': source '{}' names an \
                             output that publishes the absolute topic '{}' — \
                             reference it as `source: {}`",
                            node.id, input.name, input.source, actual, actual
                        ),
                    });
                }
                return Err(TransportError::GraphError {
                    reason: format!(
                        "node '{}' input '{}' references non-existent source '{}'",
                        node.id, input.name, input.source
                    ),
                });
            }
        }
    }

    // Validate max_slice_len is within the wire-format-representable range.
    //
    // Lower bound: `>= WireHeader::SIZE` (the wire frame requires at least the
    // 32-byte header — a header-only payload like `std_msgs::Empty` whose
    // `WIRE_FIXED_SIZE = 0` is valid at exactly 32 bytes). Upper bound:
    // `<= u32::MAX` (the wire format's `WireHeader::total_size`
    // field is `u32`, so slot sizes above 4 GiB cannot be represented).
    // Failing graph-load surfaces the misconfig with an actionable error
    // message before any publisher is built — better than letting the
    // runtime fall through to `DEFAULT_MAX_SLICE_LEN` (128 MiB), which
    // continues with a slot the user did not request and only surfaces as
    // a confusing `BufferTooSmall` at first publish.
    //
    // The bound matches `MaxSliceLen::try_new` (defined in `wire.rs`) so
    // graph-load and direct `MaxSliceLen` construction agree on the same
    // contract — no two-gates / two-semantics divergence.
    //
    // This function is called at TWO entry points so the gate is unskippable:
    // (a) the CLI's `cerulion graph run` path; (b) `GraphRuntime::build()`
    // at the runtime API boundary. Programmatic
    // library callers therefore get the same fail-fast behavior as CLI users.
    for node in &config.nodes {
        for output in &node.outputs {
            if let Some(max_slice_len) = output.max_slice_len {
                if max_slice_len < WireHeader::SIZE {
                    return Err(TransportError::GraphError {
                        reason: format!(
                            "node '{}' output '{}' max_slice_len ({}) must be >= WireHeader::SIZE ({})",
                            node.id,
                            output.name,
                            max_slice_len,
                            WireHeader::SIZE
                        ),
                    });
                }
                if max_slice_len > u32::MAX as usize {
                    return Err(TransportError::GraphError {
                        reason: format!(
                            "node '{node_id}' output '{out_name}' max_slice_len ({max_slice_len}) \
                             {VALIDATION_OVERSIZED_FRAGMENT} ({ceiling} bytes / 4 GiB) — \
                             WireHeader::total_size is u32.",
                            node_id = node.id,
                            out_name = output.name,
                            max_slice_len = max_slice_len,
                            ceiling = u32::MAX,
                        ),
                    });
                }
            }

            // History replays through the consumer
            // queues, whose depth is capped at MAX_CONSUMER_DEPTH — a
            // larger history could never be delivered in full to any
            // subscriber, so reject it at graph-load instead of
            // silently truncating the replay at join time.
            if output.history_size > MAX_CONSUMER_DEPTH {
                return Err(TransportError::GraphError {
                    reason: format!(
                        "node '{}' output '{}' history_size ({}) exceeds the maximum of \
                         {MAX_CONSUMER_DEPTH}; history is delivered through consumer queues \
                         capped at depth {MAX_CONSUMER_DEPTH}, so a larger history can never \
                         be replayed in full",
                        node.id, output.name, output.history_size
                    ),
                });
            }
        }
    }

    // Loud over silent: a listed topic that nothing in
    // the graph references is almost certainly a typo in the list — warn
    // rather than silently carrying a no-op entry. (A listed topic that is
    // CONSUMED but not produced gets its own "listing has no effect" warn
    // at provisioning time — external topics already admit multiple
    // publishers.)
    //
    // The input side is resolution-aware like the
    // output side (`output_topics` is built via `resolve_output_topic`),
    // not a raw string compare — today only a verbatim absolute
    // `source:` can match a `/`-shaped list entry (relative sources
    // resolve under the prefix, and short-refs to overridden outputs are
    // rejected upstream), but keying the predicate on RESOLVED names
    // removes the hidden coupling to that upstream rejection.
    for t in &config.multi_publisher_topics {
        let referenced = output_topics.contains(t)
            || config.nodes.iter().any(|n| {
                n.inputs.iter().any(|i| {
                    &resolve_source(&config.prefix, &i.source) == t
                        || output_topic_map
                            .get(&i.source)
                            .is_some_and(|resolved| resolved == t)
                })
            });
        if !referenced {
            tracing::warn!(
                topic = %t,
                "multi_publisher_topics lists a topic no output publishes and \
                 no input sources — the entry has no effect (check for a typo)"
            );
        }
    }

    // Cross-process partition declaration. When
    // `process_groups:` is present, every node must appear in EXACTLY one
    // group (no orphans, no double-assignment, no dangling references) — fail
    // loudly at graph-load before the runtime/barrier wiring (a later PR)
    // consumes it.
    super::partition::validate_process_groups(config)?;

    // `level_assignments:` — the CONFIG-ONLY structural
    // half (every key names a node; EVERY node covered) gates early at
    // graph-load, mirroring the `validate_process_groups` boundary. The
    // trigger-edge + contiguity invariants need node metadata (trigger
    // edges), so they are enforced at build by
    // `GraphTopology::levels_from_assignments` (same shared text —
    // defense-in-depth, one voice).
    if let Some(assignments) = &config.level_assignments {
        // A `ros2:` entry has no DAG level (it is spawned, not scheduled), so
        // it is neither required nor allowed in the block — name the fix
        // here rather than letting the coverage check call it "unknown".
        for node in config.ros2_nodes() {
            if assignments.contains_key(&node.id) {
                return Err(TransportError::GraphError {
                    reason: format!(
                        "graph '{}': `level_assignments:` names '{}', which is a `ros2:` entry — \
                         ROS 2 processes are spawned by `cerulion graph run`, not scheduled into \
                         a DAG level; remove it from the block",
                        config.identity(),
                        node.id
                    ),
                });
            }
        }
        let ids: Vec<&str> = config.native_nodes().map(|n| n.id.as_str()).collect();
        if let Some(violation) =
            super::topology::level_assignment_coverage_violation(&ids, assignments)
        {
            return Err(TransportError::GraphError {
                reason: format!("graph '{}': {violation}", config.identity()),
            });
        }
    }

    // The optional `network:` block. Runs LAST so core
    // topology/reference errors surface first (a graph must be structurally
    // sound before its network surface is meaningful). Absent block ⇒ no-op,
    // byte-identical to the pre-`network:` behavior. Called inside `validate_graph`, so BOTH
    // programmatic callers (`GraphRuntime::build`) and the CLI
    // (`cerulion graph validate` / `graph run`) inherit the checks.
    validate_network_block(config)?;

    Ok(())
}

/// The mixed-stack shape rules for `ros2:` entries (v1: spawn + supervise
/// ONLY — no scheduling, no trigger policy, no determinism claim). Every
/// rejection names the entry AND the fix:
///
/// 1. Every entry declares EXACTLY ONE of `type:` (native) / `ros2:`.
/// 2. A ros2 entry carries NO `inputs:` / `outputs:` — its topics meet native
///    nodes on the shared transport BY NAME; the wiring is not modelled.
/// 3. A ros2 block is EITHER `package` + `executable` OR `launch` — never a
///    mix, never neither; `params` / `params_file` ride the `package` form
///    only (a launch file carries its own parameters).
/// 4. Every `params:` value is a scalar (string / number / bool).
/// 5. A graph of ONLY ros2 entries is refused — the runtime has nothing to
///    schedule; `cerulion ros2 run <launch-file>` is the tool for that.
///
/// `process_groups:` membership is refused in `validate_process_groups` and
/// `level_assignments:` membership in `validate_graph_with` (each beside the
/// check it exempts the entry from).
fn validate_ros2_entries(config: &GraphConfig) -> TransportResult<()> {
    let reject = |reason: String| Err(TransportError::GraphError { reason });
    for node in &config.nodes {
        let has_type = !node.node_type.is_empty();
        let Some(ros2) = &node.ros2 else {
            if !has_type {
                return reject(format!(
                    "node '{}': missing `type:` — declare the native node type (`type: \
                     <nodes/<type>/>`), or a `ros2:` block to spawn a ROS 2 process",
                    node.id
                ));
            }
            continue;
        };
        if has_type {
            return reject(format!(
                "node '{}': declares BOTH `type:` and `ros2:` — an entry is either a native \
                 node (`type:`) or a ROS 2 process (`ros2:`); remove one",
                node.id
            ));
        }
        if !node.inputs.is_empty() || !node.outputs.is_empty() {
            return reject(format!(
                "node '{}': a `ros2:` entry cannot declare `inputs:` / `outputs:` — ROS 2 \
                 processes are opaque; their topics meet native nodes on the shared transport \
                 by NAME (a native `#[input]` on `/tf` reads a ROS 2 `/tf` publisher directly), \
                 so remove the port declarations",
                node.id
            ));
        }
        let run_form = ros2.package.is_some() || ros2.executable.is_some();
        match (&ros2.launch, run_form) {
            (Some(_), true) => {
                return reject(format!(
                    "node '{}': `ros2:` declares `launch:` together with `package:` / \
                     `executable:` — use ONE shape: `package` + `executable` (ros2 run) or \
                     `launch` (ros2 launch)",
                    node.id
                ));
            }
            (Some(_), false) => {
                if ros2.params_file.is_some() || !ros2.params.is_empty() {
                    return reject(format!(
                        "node '{}': `ros2:` with `launch:` cannot carry `params:` / \
                         `params_file:` — a launch file declares its own parameters; pass launch \
                         arguments (`name:=value`) through `args:` instead",
                        node.id
                    ));
                }
            }
            (None, true) => {
                if ros2.package.is_none() || ros2.executable.is_none() {
                    return reject(format!(
                        "node '{}': `ros2:` needs BOTH `package:` and `executable:` (ros2 run \
                         <package> <executable>) — or a `launch:` file instead",
                        node.id
                    ));
                }
            }
            (None, false) => {
                return reject(format!(
                    "node '{}': `ros2:` declares nothing to run — give it `package:` + \
                     `executable:`, or `launch:`",
                    node.id
                ));
            }
        }
        for (key, value) in &ros2.params {
            if super::config::scalar_param_value(value).is_none() {
                return reject(format!(
                    "node '{}': `ros2:` param '{key}' is not a scalar — `params:` values must \
                     be a string, number or bool (one `-p {key}:=<value>` each); put nested \
                     parameter trees in a `params_file:` instead",
                    node.id
                ));
            }
        }
    }
    if config.has_ros2_nodes() && config.native_nodes().next().is_none() {
        return reject(
            "graph declares only `ros2:` entries and no native node — there is nothing for \
             the runtime to schedule; to run a bare launch file on Cerulion transport use \
             `cerulion ros2 run <launch-file>`, or add a native node to the graph"
                .to_string(),
        );
    }
    Ok(())
}

/// Validate a graph's optional `network:` block. Absent ⇒
/// `Ok(())` (no network surface to check). Every rejection names the
/// offending topic AND states the fix (user-facing contract). The checks:
///
/// 1. **Disabled-with-lists** (rule 5): `mode: disabled` (the default when
///    omitted) MUST NOT carry `egress`/`ingress` — the lists are inert until
///    the network is enabled. Empty lists under `disabled` are valid (an
///    entirely inert block).
/// 2. **Canonical absolute** (rule 4): every `egress`/`ingress` topic must
///    start with `/`. The network key-space is canonical; a bare name is
///    rejected with the expected `/`-prefixed form (never silently
///    canonicalized). Malformed absolute names (trailing `/`, `//`,
///    zenoh-reserved chars) are rejected too — these become zenoh keys.
/// 3. **No intra-list duplicates** (rule 6): catches config typos early.
/// 4. **No egress∩ingress** (rule 1): a topic cannot be both exported and
///    imported, which would loop it back to itself (the loop-safety
///    contract).
/// 5. **Egress is graph-owned** (rule 2): every `egress` topic must be
///    produced by an in-graph node (resolution is override-aware — a
///    `topic:` override or an absolute derived name both count).
/// 6. **Ingress is external** (rule 3): every `ingress` topic must NOT have
///    an in-graph producer — ingress topics are external sources.
fn validate_network_block(config: &GraphConfig) -> TransportResult<()> {
    let Some(net) = &config.network else {
        return Ok(());
    };

    // Rule 5: a disabled (or mode-omitted ⇒ disabled) block that declares
    // topic lists is a misconfiguration — the lists do nothing until the
    // network is enabled. Checked FIRST: when the block is inert the topic
    // lists are moot, so pointing at the mode is the single useful fix.
    if net.mode == NetworkMode::Disabled && !(net.egress.is_empty() && net.ingress.is_empty()) {
        return Err(TransportError::GraphError {
            reason: "network mode is disabled but egress/ingress topics are declared — set \
                     `mode: peer` or `mode: client` to enable the network, or remove the \
                     egress/ingress lists"
                .to_string(),
        });
    }

    // Rules 4 + 6, per list: canonical-absolute shape + no intra-list dups.
    validate_network_topic_list("egress", &net.egress)?;
    validate_network_topic_list("ingress", &net.ingress)?;

    // Rule 1: a topic cannot be both exported and imported.
    for t in &net.egress {
        if net.ingress.contains(t) {
            return Err(TransportError::GraphError {
                reason: format!(
                    "network topic '{t}' is declared in BOTH egress and ingress — a topic \
                     cannot be both exported and imported (that would loop it back to \
                     itself); remove it from one list"
                ),
            });
        }
    }

    // Resolved produced-topic → first in-graph producer node id (override-
    // aware via `resolve_output_topic`, exactly as `validate_graph`'s
    // duplicate-output-topic check derives producer names). Rebuilt here so
    // the helper is self-contained and independently testable (parallels
    // `partition::validate_process_groups`).
    let mut produced: IndexMap<String, String> = IndexMap::new();
    for node in &config.nodes {
        for output in &node.outputs {
            let topic = resolve_output_topic(&config.prefix, &node.id, output);
            produced.entry(topic).or_insert_with(|| node.id.clone());
        }
    }

    // Rule 2: egress topics must be produced by an in-graph node.
    for t in &net.egress {
        if !produced.contains_key(t) {
            return Err(TransportError::GraphError {
                reason: format!(
                    "network egress topic '{t}' is not produced by any node in this graph — \
                     egress topics must be produced by a node in this graph (the name must \
                     match the publisher's resolved topic, e.g. '/{{prefix}}/{{node}}/{{output}}' \
                     or an output's `topic:` override)"
                ),
            });
        }
    }

    // Rule 3: ingress topics must NOT have an in-graph producer — they are
    // external sources arriving from the network.
    for t in &net.ingress {
        if let Some(producer) = produced.get(t) {
            return Err(TransportError::GraphError {
                reason: format!(
                    "network ingress topic '{t}' is produced by node '{producer}' in this \
                     graph — ingress topics are EXTERNAL inputs and must not have an in-graph \
                     producer; remove the ingress entry or the producing node"
                ),
            });
        }
    }

    Ok(())
}

/// Shape-check one network topic list (`egress` or
/// `ingress`) — every entry canonical-absolute + well-formed, no intra-list
/// duplicates. `list` names the list in every diagnostic so the user knows
/// which one to fix.
fn validate_network_topic_list(list: &str, topics: &[String]) -> TransportResult<()> {
    let mut seen = HashSet::new();
    for t in topics {
        // Rule 4: canonical absolute. Reject bare names loudly with the
        // expected form — never silently canonicalize (the YAML surface is
        // authored by hand; a silent `/`-insert would hide typos).
        if !t.starts_with('/') {
            return Err(TransportError::GraphError {
                reason: format!(
                    "network {list} topic '{t}' must be a canonical absolute name (leading \
                     '/') — did you mean '/{t}'? (network topic names are canonical; declare \
                     them exactly as the graph publishes them)"
                ),
            });
        }
        if let Some(why) = malformed_absolute_name(t) {
            return Err(TransportError::GraphError {
                reason: format!("network {list} topic '{t}' is malformed — {why}"),
            });
        }
        // Rule 6: no intra-list duplicates.
        if !seen.insert(t.as_str()) {
            return Err(TransportError::GraphError {
                reason: format!(
                    "network {list} lists topic '{t}' more than once — remove the duplicate"
                ),
            });
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::graph::parse_graph;

    /// The PURE half of the output-schema check: which `schema:`
    /// values are well-formed NAMES.
    ///
    /// Hand vectors on BOTH sides, because the interesting failure is a
    /// checker that is too STRICT — the two legal spellings (`pkg/Type` and a
    /// bare workspace name) plus the `::` separator a programmatically-built
    /// `GraphConfig` can still carry are each a false-refusal waiting to
    /// happen, and a false refusal at graph-load is worse than the silence
    /// this replaces.
    #[test]
    fn a_schema_value_is_judged_as_a_name_not_merely_as_present() {
        // ACCEPTED — every spelling the resolver documents.
        for ok in [
            "sensor_msgs/Image",
            // `::` is the documented alternate separator. Graph YAML folds it
            // at parse (`normalize_schema_separator`), but an IN-MEMORY config
            // — `ros2 attach`'s generator, `build_node_def`, any test building
            // the struct — never goes through serde, so a checker that only
            // knew `/` would refuse a value the parser accepts.
            "sensor_msgs::Image",
            // A BARE workspace name: `schema_cmd::resolve_port_schema` tier 3,
            // and two shipped graphs use it. Requiring a `/` here would be a
            // false refusal of a documented spelling.
            "DetectionArray",
            "std_msgs/UInt8MultiArray",
            "autoware_perception_msgs/PredictedObjects",
        ] {
            assert!(
                validate_schema_value(ok).is_ok(),
                "`{ok}` is a legal spelling and must be accepted"
            );
        }

        // REFUSED, each naming its own property — an operator reading
        // "contains whitespace" fixes something different from one reading
        // "has an empty segment".
        for (bad, needle) in [
            ("", "is missing"),
            ("   ", "is blank"),
            ("sensor_msgs/ Image", "whitespace"),
            ("sensor msgs/Image", "whitespace"),
            ("sensor_msgs/Image\n", "whitespace"),
            ("/Image", "empty segment"),
            ("sensor_msgs/", "empty segment"),
            ("sensor_msgs//Image", "empty segment"),
            ("sensor_msgs:Image", "stray ':'"),
            ("sensor_msgs/msg/Image", "package separators"),
            ("sensor_msgs/*", "reserved character"),
            ("sensor_msgs/Image#1", "reserved character"),
        ] {
            let why = validate_schema_value(bad).expect_err(&format!("`{bad}` must be refused"));
            assert!(
                why.contains(needle),
                "`{bad}` must be refused for {needle:?}, got: {why}"
            );
        }
    }

    /// The GRAPH-level arm: the refusal names the node, the output and the
    /// offending value, so an operator can find the line without grepping.
    #[test]
    fn an_output_without_a_schema_is_refused_naming_node_and_output() {
        // ABSENT — the common case, since `OutputDef.schema` is
        // `#[serde(default)]` and `docs/user-api.md` calls the key required.
        let yaml = r#"
prefix: percep
nodes:
  - id: camera
    type: camera
    outputs:
      - name: image_raw
"#;
        let config = parse_graph(yaml).unwrap();
        let err = validate_graph(&config).unwrap_err().to_string();
        for needle in ["camera", "image_raw", "`schema:`", "is missing"] {
            assert!(err.contains(needle), "refusal must name {needle:?}: {err}");
        }

        // PRESENT but malformed — the value is QUOTED, so the operator sees
        // what they actually typed.
        let yaml = r#"
prefix: percep
nodes:
  - id: camera
    type: camera
    outputs:
      - name: image_raw
        schema: "sensor_msgs/ Image"
"#;
        let config = parse_graph(yaml).unwrap();
        let err = validate_graph(&config).unwrap_err().to_string();
        for needle in ["camera", "image_raw", "sensor_msgs/ Image", "whitespace"] {
            assert!(err.contains(needle), "refusal must name {needle:?}: {err}");
        }
    }

    /// The RELAXED arm skips ONLY the absent case.
    ///
    /// `ValidationOptions::require_output_schema = false` exists so `node
    /// stage` can write an entry whose schema the CLI could not derive (see
    /// that field's doc). It must NOT become "skip the schema check", because
    /// staging never produces a MALFORMED value — every value it writes comes
    /// from `node_metadata`, so a malformed one can only be a hand-edit typo,
    /// and typos are what this check exists to catch.
    ///
    /// Written because widening this arm's scope survived every other
    /// schema-check arm in the tree: they all call `validate_graph`, which is
    /// strict, so the relaxed arm's SCOPE was pinned by nothing at all.
    #[test]
    fn the_relaxed_arm_skips_an_absent_schema_but_never_a_malformed_one() {
        let relaxed = ValidationOptions {
            require_output_schema: false,
            ..ValidationOptions::default()
        };

        // ABSENT — what staging legitimately writes. Relaxed accepts it;
        // strict (every run path) still refuses.
        let absent = parse_graph(
            r#"
prefix: percep
nodes:
  - id: camera
    type: camera
    outputs:
      - name: image_raw
"#,
        )
        .unwrap();
        assert!(
            validate_graph_with(&absent, relaxed).is_ok(),
            "staging must be able to write an entry whose schema it cannot derive"
        );
        assert!(
            validate_graph(&absent).is_err(),
            "and every RUN path must still refuse it — the other end of the contract"
        );

        // PRESENT but malformed — a hand-edit typo, refused under BOTH
        // postures. Each spelling names its own property, so a relaxed arm
        // that swallowed the class would be caught whichever one is written.
        for (bad, needle) in [
            ("sensor_msgs/ Image", "whitespace"),
            ("/Image", "empty segment"),
            ("sensor_msgs/msg/Image", "package separators"),
            ("sensor_msgs/*", "reserved character"),
        ] {
            let yaml = format!(
                r#"
prefix: percep
nodes:
  - id: camera
    type: camera
    outputs:
      - name: image_raw
        schema: "{bad}"
"#
            );
            let config = parse_graph(&yaml).unwrap();
            let err = validate_graph_with(&config, relaxed)
                .unwrap_err()
                .to_string();
            assert!(
                err.contains(needle) && err.contains("image_raw"),
                "`{bad}` must be refused even under the relaxed posture, got: {err}"
            );
        }
    }

    /// ANTI-TAUTOLOGY. Without this the arm above passes against a validator
    /// that refuses every graph — and BOTH separator spellings must survive,
    /// which is the pin on `normalize_schema_separator` still folding `::` at
    /// the parse seam.
    #[test]
    fn a_correctly_declared_schema_passes_in_both_spellings() {
        for spelling in ["sensor_msgs/Image", "sensor_msgs::Image"] {
            let yaml = format!(
                r#"
prefix: percep
nodes:
  - id: camera
    type: camera
    outputs:
      - name: image_raw
        schema: {spelling}
"#
            );
            let config = parse_graph(&yaml).unwrap();
            assert!(
                validate_graph(&config).is_ok(),
                "`schema: {spelling}` must pass"
            );
            // The parse seam normalizes, so both reach the runtime identically.
            assert_eq!(config.nodes[0].outputs[0].schema, "sensor_msgs/Image");
        }
    }

    #[test]
    fn test_validate_empty_graph_rejected() {
        let yaml = r#"
name: empty
nodes: []
"#;
        let config = parse_graph(yaml).unwrap();
        let result = validate_graph(&config);
        assert!(result.is_err());
        let err = result.unwrap_err().to_string();
        assert!(err.contains("at least one node"));
    }

    #[test]
    fn test_validate_duplicate_node_ids_rejected() {
        let yaml = r#"
name: test
nodes:
  - id: dup
    type: a
  - id: dup
    type: b
"#;
        let config = parse_graph(yaml).unwrap();
        let result = validate_graph(&config);
        assert!(result.is_err());
        let err = result.unwrap_err().to_string();
        assert!(err.contains("duplicate node ID"));
    }

    #[test]
    fn test_validate_dangling_source_rejected() {
        let yaml = r#"
name: test
nodes:
  - id: sub1
    type: consumer
    inputs:
      - name: data
        source: ghost/output
"#;
        let config = parse_graph(yaml).unwrap();
        let result = validate_graph(&config);
        assert!(result.is_err());
        let err = result.unwrap_err().to_string();
        assert!(err.contains("non-existent source"));
    }

    #[test]
    fn test_validate_small_max_slice_len_rejected() {
        let yaml = r#"
name: test
nodes:
  - id: pub1
    type: pub
    outputs:
      - name: data
        schema: test/Data
        max_slice_len: 10
"#;
        let config = parse_graph(yaml).unwrap();
        let result = validate_graph(&config);
        assert!(result.is_err());
        let err = result.unwrap_err().to_string();
        assert!(err.contains("max_slice_len"));
    }

    /// Pin the exact
    /// boundary at the floor — `max_slice_len: 31` (one below
    /// `WireHeader::SIZE`) is rejected. Mirror of
    /// `try_new_rejects_just_below_wire_header_size` in
    /// `max_slice_len_newtype_test.rs`. Without this, a one-char
    /// regression (`<` → `<=` in the validation check) would silently
    /// accept undersized payloads.
    #[test]
    fn test_validate_just_below_wire_header_size_rejected() {
        let yaml = r#"
name: test
nodes:
  - id: pub1
    type: pub
    outputs:
      - name: data
        schema: test/Data
        max_slice_len: 31
"#;
        let config = parse_graph(yaml).unwrap();
        let result = validate_graph(&config);
        assert!(
            result.is_err(),
            "max_slice_len: 31 (< WireHeader::SIZE) must fail graph-load"
        );
        let err = result.unwrap_err().to_string();
        assert!(
            err.contains("WireHeader::SIZE"),
            "diagnostic must reference WireHeader::SIZE; got: {err}"
        );
    }

    /// Pin the exact
    /// floor — `max_slice_len: 32` (exactly `WireHeader::SIZE`) is
    /// accepted. Mirror of `try_new_accepts_exactly_wire_header_size`
    /// in `max_slice_len_newtype_test.rs`. This is the boundary
    /// (`<`, not `<=`); a future
    /// re-tightening to `<=` would silently reject every header-only
    /// payload like `std_msgs::Empty` and break the contract this test
    /// pins.
    #[test]
    fn test_validate_exactly_wire_header_size_accepted() {
        let yaml = r#"
name: test
nodes:
  - id: pub1
    type: pub
    outputs:
      - name: data
        schema: test/Data
        max_slice_len: 32
"#;
        let config = parse_graph(yaml).unwrap();
        let result = validate_graph(&config);
        assert!(
            result.is_ok(),
            "max_slice_len: 32 (== WireHeader::SIZE) must be accepted (header-only payload). \
             Error was: {:?}",
            result.err()
        );
    }

    /// `max_slice_len` above `u32::MAX` fails graph-load with a
    /// clear actionable error, instead of clamping silently at the resolver.
    /// 64-bit-only because `(u32::MAX as usize) + 1` overflows on 32-bit.
    #[cfg(target_pointer_width = "64")]
    #[test]
    fn test_validate_oversized_max_slice_len_rejected() {
        let oversize = (u32::MAX as usize) + 1;
        let yaml = format!(
            r#"
name: test
nodes:
  - id: pub1
    type: pub
    outputs:
      - name: data
        schema: test/Data
        max_slice_len: {oversize}
"#,
        );
        let config = parse_graph(&yaml).unwrap();
        let result = validate_graph(&config);
        assert!(
            result.is_err(),
            "oversize max_slice_len must fail graph-load"
        );
        let err = result.unwrap_err().to_string();
        // Reference the same const the format! site uses — single
        // source of truth for the diagnostic surface.
        assert!(
            err.contains(VALIDATION_OVERSIZED_FRAGMENT),
            "diagnostic must contain VALIDATION_OVERSIZED_FRAGMENT; got: {err}",
        );
        assert!(
            err.contains("4 GiB"),
            "diagnostic must include the human-readable 4 GiB ceiling; got: {err}",
        );
    }

    /// Pin that `max_slice_len ==
    /// u32::MAX` (exactly the ceiling) PASSES validation, not just
    /// that `(u32::MAX) + 1` is rejected. Guards against an off-by-one
    /// regression that flips `>` to `>=` in the upper-bound check.
    #[test]
    fn test_validate_u32_max_max_slice_len_accepted() {
        let ceiling = u32::MAX as usize;
        let yaml = format!(
            r#"
name: test
nodes:
  - id: pub1
    type: pub
    outputs:
      - name: data
        schema: test/Data
        max_slice_len: {ceiling}
"#,
        );
        let config = parse_graph(&yaml).unwrap();
        assert!(
            validate_graph(&config).is_ok(),
            "exactly u32::MAX must pass validation; the wire format \
             can represent total_size == u32::MAX",
        );
    }

    #[test]
    fn test_validate_single_node_no_connections() {
        let yaml = r#"
name: test
nodes:
  - id: standalone
    type: publisher_only
    outputs:
      - name: data
        schema: test/Data
        max_slice_len: 1024
"#;
        let config = parse_graph(yaml).unwrap();
        assert!(validate_graph(&config).is_ok());
    }

    #[test]
    fn test_validate_two_node_pipeline() {
        let yaml = r#"
name: test
nodes:
  - id: pub1
    type: publisher
    outputs:
      - name: data
        schema: test/Data
        max_slice_len: 1024
  - id: sub1
    type: consumer
    inputs:
      - name: data
        source: pub1/data
"#;
        let config = parse_graph(yaml).unwrap();
        assert!(validate_graph(&config).is_ok());
    }

    #[test]
    fn test_validate_duplicate_output_topics_rejected() {
        let yaml = r#"
name: test
nodes:
  - id: a
    type: pub
    outputs:
      - name: data
        max_slice_len: 1024
  - id: a
    type: pub2
    outputs:
      - name: data
        max_slice_len: 1024
"#;
        let config = parse_graph(yaml).unwrap();
        let result = validate_graph(&config);
        assert!(result.is_err());
    }

    /// `history_size` exactly at [`MAX_CONSUMER_DEPTH`]
    /// (the boundary) passes — the full history fits a max-depth queue.
    #[test]
    fn test_validate_history_size_at_max_accepted() {
        let yaml = format!(
            r#"
name: test
nodes:
  - id: pub1
    type: pub
    outputs:
      - name: data
        schema: test/Data
        max_slice_len: 1024
        history_size: {MAX_CONSUMER_DEPTH}
"#,
        );
        let config = parse_graph(&yaml).unwrap();
        assert!(
            validate_graph(&config).is_ok(),
            "history_size == MAX_CONSUMER_DEPTH (boundary) must be accepted"
        );
    }

    /// `history_size` above [`MAX_CONSUMER_DEPTH`] is
    /// rejected at graph-load — history replays through consumer queues
    /// capped at that depth, so a larger history can never be delivered
    /// in full.
    #[test]
    fn test_validate_history_size_above_max_rejected() {
        let above = MAX_CONSUMER_DEPTH + 1;
        let yaml = format!(
            r#"
name: test
nodes:
  - id: pub1
    type: pub
    outputs:
      - name: data
        schema: test/Data
        max_slice_len: 1024
        history_size: {above}
"#,
        );
        let config = parse_graph(&yaml).unwrap();
        let result = validate_graph(&config);
        assert!(
            result.is_err(),
            "history_size = MAX_CONSUMER_DEPTH + 1 must fail graph-load"
        );
        let err = result.unwrap_err().to_string();
        assert!(
            err.contains("history_size") && err.contains(&MAX_CONSUMER_DEPTH.to_string()),
            "diagnostic must name history_size and the cap; got: {err}"
        );
    }

    // ─── `network:` block validation ────────────────
    //
    // Oracle-vector tests (hand-built expected error substrings, never a
    // self-compare). The base graph PRODUCES `/robo/cam/cloud` (derived:
    // prefix `robo` + node `cam` + output `cloud`) so egress/ingress
    // ownership can be exercised against a real in-graph producer.

    /// Base fixture body: a single node producing `/robo/cam/cloud`. The
    /// caller appends a `network:` block.
    fn graph_with_network(network_block: &str) -> String {
        format!(
            r#"
name: net
prefix: robo
nodes:
  - id: cam
    type: camera
    outputs:
      - name: cloud
        schema: sensor_msgs/PointCloud2
        max_slice_len: 1024
{network_block}"#
        )
    }

    /// Happy path: `mode: peer`, egress references a produced topic, ingress
    /// references an external (unproduced) topic. Valid, no error.
    #[test]
    fn test_network_full_valid_block_accepted() {
        let yaml = graph_with_network(
            r#"network:
  mode: peer
  connect:
    - tcp/192.168.123.99:7447
  listen:
    - tcp/0.0.0.0:7447
  egress:
    - /robo/cam/cloud
  ingress:
    - /external/cmd_vel
"#,
        );
        let config = parse_graph(&yaml).unwrap();
        assert!(
            validate_graph(&config).is_ok(),
            "valid network block must pass: {:?}",
            validate_graph(&config).err()
        );
    }

    /// Empty egress/ingress lists under `peer` mode is VALID (a
    /// discovery-only / future-topics graph) — no diagnostic.
    #[test]
    fn test_network_empty_lists_peer_mode_accepted() {
        let yaml = graph_with_network(
            r#"network:
  mode: peer
  listen:
    - tcp/0.0.0.0:7447
"#,
        );
        let config = parse_graph(&yaml).unwrap();
        assert!(validate_graph(&config).is_ok());
    }

    /// A `disabled` block with EMPTY lists is a valid (inert) block — the
    /// boundary of rule 5.
    #[test]
    fn test_network_disabled_empty_lists_accepted() {
        let yaml = graph_with_network(
            r#"network:
  mode: disabled
"#,
        );
        let config = parse_graph(&yaml).unwrap();
        assert!(validate_graph(&config).is_ok());
    }

    /// Rule 1: a topic in BOTH egress and ingress is rejected, naming the
    /// topic + the fix.
    #[test]
    fn test_network_topic_in_both_egress_and_ingress_rejected() {
        let yaml = graph_with_network(
            r#"network:
  mode: peer
  egress:
    - /robo/cam/cloud
  ingress:
    - /robo/cam/cloud
"#,
        );
        let config = parse_graph(&yaml).unwrap();
        let err = validate_graph(&config).unwrap_err().to_string();
        assert!(
            err.contains("/robo/cam/cloud")
                && err.contains("BOTH egress and ingress")
                && err.contains("remove it from one list"),
            "rule 1 must name the topic and the fix; got: {err}"
        );
    }

    /// Rule 2: an egress topic with no in-graph producer is rejected.
    #[test]
    fn test_network_egress_not_produced_rejected() {
        let yaml = graph_with_network(
            r#"network:
  mode: peer
  egress:
    - /robo/cam/ghost
"#,
        );
        let config = parse_graph(&yaml).unwrap();
        let err = validate_graph(&config).unwrap_err().to_string();
        assert!(
            err.contains("/robo/cam/ghost")
                && err.contains("must be produced by a node in this graph"),
            "rule 2 must name the topic and the fix; got: {err}"
        );
    }

    /// Rule 3: an ingress topic that an in-graph node produces is rejected,
    /// naming the producing node + the fix.
    #[test]
    fn test_network_ingress_with_producer_rejected() {
        let yaml = graph_with_network(
            r#"network:
  mode: peer
  ingress:
    - /robo/cam/cloud
"#,
        );
        let config = parse_graph(&yaml).unwrap();
        let err = validate_graph(&config).unwrap_err().to_string();
        assert!(
            err.contains("/robo/cam/cloud")
                && err.contains("node 'cam'")
                && err.contains("remove the ingress entry or the producing node"),
            "rule 3 must name the topic, the producer, and the fix; got: {err}"
        );
    }

    /// Rule 4: a bare (non-absolute) egress topic is rejected with the
    /// expected `/`-prefixed form — never silently canonicalized.
    #[test]
    fn test_network_bare_egress_topic_rejected() {
        let yaml = graph_with_network(
            r#"network:
  mode: peer
  egress:
    - cam/cloud
"#,
        );
        let config = parse_graph(&yaml).unwrap();
        let err = validate_graph(&config).unwrap_err().to_string();
        assert!(
            err.contains("cam/cloud")
                && err.contains("canonical absolute name")
                && err.contains("did you mean '/cam/cloud'"),
            "rule 4 must name the topic and the expected form; got: {err}"
        );
    }

    /// Rule 4 (second half): a malformed absolute egress topic (trailing
    /// '/') is rejected as malformed.
    #[test]
    fn test_network_malformed_absolute_egress_rejected() {
        let yaml = graph_with_network(
            r#"network:
  mode: peer
  egress:
    - /robo/cam/cloud/
"#,
        );
        let config = parse_graph(&yaml).unwrap();
        let err = validate_graph(&config).unwrap_err().to_string();
        assert!(
            err.contains("/robo/cam/cloud/")
                && err.contains("malformed")
                && err.contains("trailing"),
            "malformed absolute name must be rejected; got: {err}"
        );
    }

    /// Rule 5: `mode: disabled` with a non-empty egress list is rejected —
    /// enable the network or remove the lists.
    #[test]
    fn test_network_disabled_with_lists_rejected() {
        let yaml = graph_with_network(
            r#"network:
  mode: disabled
  egress:
    - /robo/cam/cloud
"#,
        );
        let config = parse_graph(&yaml).unwrap();
        let err = validate_graph(&config).unwrap_err().to_string();
        assert!(
            err.contains("disabled")
                && err.contains("`mode: peer`")
                && err.contains("`mode: client`"),
            "rule 5 must name the disabled state and the enable fix; got: {err}"
        );
    }

    /// Rule 5 also fires when only `ingress` is declared under `disabled`.
    #[test]
    fn test_network_disabled_with_ingress_only_rejected() {
        let yaml = graph_with_network(
            r#"network:
  mode: disabled
  ingress:
    - /external/cmd
"#,
        );
        let config = parse_graph(&yaml).unwrap();
        let err = validate_graph(&config).unwrap_err().to_string();
        assert!(
            err.contains("disabled") && err.contains("egress/ingress"),
            "rule 5 must fire for an ingress-only disabled block; got: {err}"
        );
    }

    /// Rule 6: a duplicated topic within one list is rejected.
    #[test]
    fn test_network_duplicate_topic_in_list_rejected() {
        let yaml = graph_with_network(
            r#"network:
  mode: peer
  egress:
    - /robo/cam/cloud
    - /robo/cam/cloud
"#,
        );
        let config = parse_graph(&yaml).unwrap();
        let err = validate_graph(&config).unwrap_err().to_string();
        assert!(
            err.contains("/robo/cam/cloud")
                && err.contains("egress")
                && err.contains("more than once"),
            "rule 6 must name the topic + list + the fix; got: {err}"
        );
    }

    /// An absent `network:` block is a no-op — the graph validates exactly
    /// as it would without the block (byte-identical-when-absent contract).
    #[test]
    fn test_network_absent_block_is_noop() {
        let yaml = graph_with_network("");
        let config = parse_graph(&yaml).unwrap();
        assert!(config.network.is_none());
        assert!(validate_graph(&config).is_ok());
    }
}
