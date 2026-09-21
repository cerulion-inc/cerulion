// SPDX-License-Identifier: AGPL-3.0-only
//! Node metadata derived from source code.
//!
//! `nodes/<type>/src/lib.rs` is the single source of truth for
//! port and trigger metadata. `parse_node_metadata` walks the
//! source via `syn` for macro nodes (extracting `#[input]` /
//! `#[output]` field attrs and the node-level
//! `#[cerulion_node(...)]` policy) or via the
//! `// CERULION:INFO_START` JSON marker for raw-FFI nodes
//! (extracting `inputs`, `outputs`, and an optional `policy`
//! field with the same shape the cdylib FFI uses).

use std::path::Path;

use cerulion_core::graph::node::BackpressurePolicy;
use cerulion_core::{MacroPolicy, PolicyJson};

use crate::error::{CliError, CliResult};

/// Node metadata derived from `nodes/<type>/src/lib.rs`. Carries
/// the node type, trigger policy, and per-port name/schema lists.
///
/// `policy` reuses [`cerulion_core::MacroPolicy`] so the CLI and
/// runtime sides agree on a single enum (no parallel definition,
/// no manual conversion at the boundary).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct NodeMetadata {
    pub node_type: String,
    pub policy: Option<MacroPolicy>,
    pub inputs: Vec<PortDef>,
    pub outputs: Vec<PortDef>,
}

/// Port definition with optional schema reference. Carries the schema
/// NAME the port's Rust type resolves to: `pkg/Type` for a type imported
/// from an external message crate (`sensor_msgs/Image`), the BARE entry
/// name for a workspace schema reached through the node's own module
/// (`DetectionArray`), and `None` when the source gives no `use` to derive
/// it from (a raw-FFI node, or a type declared inline).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PortDef {
    pub name: String,
    pub schema: Option<String>,
    /// The OTHER schema names this port's Rust type carries when a `#[cfg]`
    /// decides it — a `#[cfg]`-gated same-scope module beside a same-named
    /// extern crate, cfg-exclusive imports of one leaf (every further one
    /// adds a name: a graph written for the THIRD configuration must not be
    /// refused), or a conditional import shadowing an ancestor scope's
    /// (measured against rustc — see `collect_scope_imports`, and `CfgShape`
    /// for the three). Empty for every unambiguous declaration. `graph
    /// validate` accepts a `schema:` matching `schema` or any of these for
    /// such a port, as a pass with a warning, because the parser cannot see
    /// which build the graph was written for.
    pub schema_alternatives: Vec<String>,
    /// Whether this input carries the `#[input(trigger)]` mark.
    /// Always `false` for outputs (a trigger is an input-only concept) and
    /// for scaffolding sites (`node create`/`node modify`) where trigger is
    /// added later by editing source. Populated for real by the macro-node
    /// source parse and the raw-FFI INFO_BYTES parse so the CLI's
    /// `source_entry_infos` feeds `build_trigger_edges` the same per-input
    /// truth the runtime sees across the cdylib FFI (ABI v9).
    pub trigger: bool,
    /// The declared `#[input(backpressure = …)]` policy.
    ///
    /// Always [`BackpressurePolicy::DropOldest`] (the macro's own default)
    /// for outputs — backpressure is an input-only concept — and for the
    /// scaffolding sites (`node create` / `node modify`), which emit a bare
    /// `#[input]` and leave any QoS to a later source edit.
    ///
    /// Load-bearing rather than informational: the auto-partitioner
    /// derives the multi-process grouping from SOURCE metadata, well before
    /// the first cdylib load, and a `block` edge is a hard CO-LOCATION
    /// constraint (`cerulion_core::graph::partition`) — the scheduler's defer
    /// mirror is a process-local `Arc<AtomicU64>`, so a `block` producer and
    /// its consumer in two OS processes cannot see each other. Dropping the
    /// value here (as this parser once did: it read it and threw
    /// it away) made every derived partition split every `block` edge, and
    /// the consumer's worker then died at `GraphTopology::validate` blaming
    /// an external publisher the user does not have.
    pub backpressure: BackpressurePolicy,
}

impl NodeMetadata {
    /// Construct an empty metadata record. Callers that build the
    /// metadata in pieces (e.g. `node_create` collecting CLI args
    /// before generating source) start here and push to
    /// `inputs` / `outputs`.
    pub fn new(node_type: &str, policy: Option<MacroPolicy>) -> Self {
        Self {
            node_type: node_type.to_string(),
            policy,
            inputs: vec![],
            outputs: vec![],
        }
    }

    /// Check if an input port with the given name already exists.
    pub fn has_input(&self, name: &str) -> bool {
        self.inputs.iter().any(|p| p.name == name)
    }

    /// Check if an output port with the given name already exists.
    pub fn has_output(&self, name: &str) -> bool {
        self.outputs.iter().any(|p| p.name == name)
    }
}

/// Parse `nodes/<type>/src/lib.rs` and return its node metadata.
///
/// `node_dir` is the node crate root (i.e.
/// `nodes_dir.join(node_type)`). The function reads
/// `node_dir.join("src/lib.rs")`, parses it via `syn::parse_file`,
/// and extracts:
///
/// - `node_type` — the folder name (the type is not stored in source)
/// - For macro nodes (struct annotated with `#[cerulion_node]`):
///   - `policy` — `Period` / `Sync` / `External` from
///     the node-level `#[cerulion_node(...)]` arg, OR `DataTrigger`
///     synthesized from a single `#[input(trigger)]` field
///   - `inputs` / `outputs` — fields annotated with `#[input]` /
///     `#[input(trigger)]` / `#[output]`. `name` = field ident,
///     `schema` = canonical slash-form recovered by walking
///     `use` statements (a `use native_ros2_messages::pkg::Type;`
///     anchors `Type` → `Some("pkg/Type")`)
/// - For raw-FFI nodes (has `// CERULION:INFO_START` markers):
///   - `policy` — read from the optional `policy` field of the
///     `INFO_BYTES` JSON; same shape as the cdylib FFI emits
///   - `inputs` / `outputs` — names parsed from the `INFO_BYTES`
///     JSON literal (`schema` is `None` because the JSON carries
///     names only)
///
/// Returns `CliError::NodeNotFound` if `lib.rs` is missing,
/// `CliError::Validation` if parsing fails or no recognisable
/// node shape is found.
pub fn parse_node_metadata(node_dir: &Path) -> CliResult<NodeMetadata> {
    let node_type = node_dir
        .file_name()
        .and_then(|n| n.to_str())
        .ok_or_else(|| {
            CliError::Validation(format!(
                "node directory '{}' has no readable file name",
                node_dir.display()
            ))
        })?
        .to_string();

    let lib_path = node_dir.join("src/lib.rs");
    if !lib_path.exists() {
        return Err(CliError::NodeNotFound {
            node_type: node_type.clone(),
        });
    }

    let source = std::fs::read_to_string(&lib_path)?;

    // Try macro form first; fall back to raw-FFI INFO marker if
    // no `#[cerulion_node]` struct is found OR if syn fails to
    // parse the source. The raw-FFI path uses only `find` + slice
    // operations and doesn't need valid Rust syntax — a raw-FFI
    // node with a syntax error elsewhere should still surface
    // its INFO_BYTES port list rather than fail entirely.
    let macro_result = try_parse_macro_node(&source, &node_type);
    if let Ok(Some(metadata)) = &macro_result {
        return Ok(metadata.clone());
    }
    if let Some(metadata) = try_parse_raw_ffi_node(&source, &node_type)? {
        return Ok(metadata);
    }
    // Neither path succeeded. If the macro-side errored, surface
    // its diagnostic (parse failures); otherwise emit the
    // "no recognisable shape" error.
    match macro_result {
        Ok(_) => Err(CliError::Validation(format!(
            "node '{}' source has no `#[cerulion_node]` struct and no \
             `// CERULION:INFO_START` markers",
            node_type
        ))),
        Err(e) => Err(e),
    }
}

/// Try to parse `source` as a macro-based node. Returns
/// `Ok(Some(metadata))` if a `#[cerulion_node]` struct is found,
/// `Ok(None)` if not, `Err` if syn-parse fails.
fn try_parse_macro_node(source: &str, node_type: &str) -> CliResult<Option<NodeMetadata>> {
    let file = match syn::parse_file(source) {
        Ok(f) => f,
        Err(e) => {
            return Err(CliError::Validation(format!(
                "node '{}' source did not parse as Rust: {}",
                node_type, e
            )));
        }
    };

    // Each port's type ident is resolved in the node struct's scope chain
    // (`resolve_port_type`): `use native_ros2_messages::pkg::Type;` names
    // `pkg/Type`, a local codegen module's type its bare entry name, a
    // local re-export its target; renames are keyed by the name the field
    // sees. A type no explicit `use` names while a glob import is in scope
    // is undecidable, and said so.
    let scope_chain = node_scope_chain(&file.items).unwrap_or_else(|| vec![&file.items]);

    // Walk every reachable struct (descending into modules to
    // mirror `is_macro_based`'s behaviour) and pick the first one
    // carrying `#[cerulion_node]`.
    let mut found: Option<&syn::ItemStruct> = None;
    let mut macro_attr: Option<&syn::Attribute> = None;
    walk_items(&file.items, &mut |item| {
        if found.is_some() {
            return;
        }
        if let syn::Item::Struct(item_struct) = item {
            for attr in &item_struct.attrs {
                if attr.path().is_ident("cerulion_node") {
                    found = Some(item_struct);
                    macro_attr = Some(attr);
                    return;
                }
            }
        }
    });

    let Some(item_struct) = found else {
        return Ok(None);
    };

    let mut inputs = Vec::new();
    let mut outputs = Vec::new();
    let mut single_trigger_input: Option<String> = None;
    let mut multiple_triggers = false;
    for field in &item_struct.fields {
        let Some(name_ident) = field.ident.as_ref() else {
            // Tuple-struct field; the macro rejects tuple structs
            // anyway, but be tolerant here.
            continue;
        };
        let port_name = name_ident.to_string();
        let mut is_input = false;
        let mut is_output = false;
        let mut is_trigger = false;
        // The declared backpressure policy. The macro's own default
        // when the attribute is absent, so an undeclared input reads exactly
        // as `BackpressurePolicy::default()` did before this field existed.
        let mut backpressure = BackpressurePolicy::DropOldest;
        for attr in &field.attrs {
            if attr.path().is_ident("input") {
                is_input = true;
                // `#[input(trigger)]` — record so we can synthesize
                // `MacroPolicy::DataTrigger` when there's exactly one.
                //
                // CLI/engine invariant: the CLI metadata parser must
                // not ACCEPT attr surface the macro REJECTS. Idents are
                // checked POSITIONALLY: top level accepts exactly the
                // macro's set (`trigger`, `depth`, `backpressure`,
                // `expect_within_ms`); the backpressure VALUE idents
                // (`drop_oldest` / `block` / `sample`) are legal ONLY in
                // the `backpressure = <value>` position — a bare
                // `#[input(block)]` is macro-invalid and must be a loud
                // `CliError` here too (a flat whitelist false-greened it).
                if let syn::Meta::List(meta_list) = &attr.meta {
                    use proc_macro2::TokenTree;
                    // Tiny positional state machine over the flat token
                    // stream: `saw_backpressure` = the previous ident was
                    // `backpressure`; `expect_bp_value` = we just consumed
                    // its `=` and the next token must be a policy value
                    // IDENT; `expect_sample_group` = we just accepted
                    // `sample` and the next token must be its `(N)` group
                    // (the macro requires the parens: without this state,
                    // bare `sample` and non-ident values
                    // false-green here while the macro rejects them).
                    let mut saw_backpressure = false;
                    let mut expect_bp_value = false;
                    let mut expect_sample_group = false;
                    for tt in meta_list.tokens.clone() {
                        match tt {
                            TokenTree::Group(g) if expect_sample_group => {
                                if g.delimiter() != proc_macro2::Delimiter::Parenthesis {
                                    return Err(CliError::Validation(format!(
                                        "node `{node_type}` field `{port_name}`: `sample` \
                                         requires a parenthesized count — `backpressure = \
                                         sample(N)`."
                                    )));
                                }
                                expect_sample_group = false;
                                // RECORD the count, parsed EXACTLY
                                // the way the macro parses it — a `LitInt` then
                                // `base10_parse::<u64>` (`cerulion_macros::parse`).
                                //
                                // The CLI/engine invariant runs BOTH ways, and this
                                // seam can break in each direction:
                                //
                                //  - never ACCEPT surface the macro REFUSES:
                                //    if the group's CONTENTS go unread,
                                //    `sample(abc)` false-greens here while the
                                //    macro refuses to compile it.
                                //  - never REJECT surface the macro ACCEPTS:
                                //    a plain `str::parse::<u64>` takes only bare
                                //    ASCII decimal, so `sample(1_000)`,
                                //    `sample(50u64)`, `sample(0x10)`, `0b`/`0o`
                                //    — all of which `base10_parse` accepts and
                                //    the macro compiles — hard-errored here, and
                                //    that error `?`-propagates through
                                //    `source_entry_infos` into the `graph run`
                                //    default-partition preflight, `graph levels`
                                //    and `graph partition`, so a node whose
                                //    source BUILDS could not be run.
                                //
                                // Sharing the macro's own parser is what keeps
                                // the two sides from drifting again; the syn
                                // error is carried through so an out-of-range
                                // literal (which IS an integer) does not report
                                // the wrong condition.
                                let inner = g.stream();
                                let parsed: Result<u64, String> =
                                    syn::parse2::<syn::LitInt>(inner.clone())
                                        .and_then(|lit| lit.base10_parse::<u64>())
                                        .map_err(|e| e.to_string());
                                let n = match parsed {
                                    Ok(n) => n,
                                    Err(err) => {
                                        return Err(CliError::Validation(format!(
                                            "node `{node_type}` field `{port_name}`: `sample` \
                                             requires an integer count that fits in a `u64` — \
                                             `backpressure = sample(N)` (got `sample({inner})`: \
                                             {err})."
                                        )));
                                    }
                                };
                                backpressure = BackpressurePolicy::Sample(n);
                            }
                            _ if expect_sample_group => {
                                return Err(CliError::Validation(format!(
                                    "node `{node_type}` field `{port_name}`: `sample` \
                                     requires a parenthesized count — `backpressure = \
                                     sample(N)`."
                                )));
                            }
                            TokenTree::Ident(ident) => {
                                if expect_bp_value {
                                    expect_bp_value = false;
                                    match ident.to_string().as_str() {
                                        // RECORD the value instead of
                                        // discarding it — `PortDef.backpressure`
                                        // feeds `source_entry_infos`, which is
                                        // what the auto-partitioner
                                        // reads at derive time.
                                        "drop_oldest" => {
                                            backpressure = BackpressurePolicy::DropOldest;
                                        }
                                        "block" => backpressure = BackpressurePolicy::Block,
                                        "sample" => expect_sample_group = true,
                                        _ => {
                                            return Err(CliError::Validation(format!(
                                                "node `{node_type}` field `{port_name}`: unknown \
                                                 backpressure policy `{ident}`. Supported: \
                                                 `drop_oldest`, `block`, `sample(N)`."
                                            )));
                                        }
                                    }
                                } else if ident == "trigger" {
                                    is_trigger = true;
                                } else if ident == "backpressure" {
                                    saw_backpressure = true;
                                } else if !matches!(
                                    ident.to_string().as_str(),
                                    "depth" | "expect_within_ms"
                                ) {
                                    return Err(CliError::Validation(format!(
                                        "node `{node_type}` field `{port_name}`: unknown input \
                                         attribute `{ident}`. Supported: `trigger`, `depth`, \
                                         `backpressure`, `expect_within_ms`."
                                    )));
                                }
                            }
                            TokenTree::Punct(p) if p.as_char() == '=' && saw_backpressure => {
                                saw_backpressure = false;
                                expect_bp_value = true;
                            }
                            other => {
                                // A non-ident in the VALUE position (e.g.
                                // `backpressure = 5`) — the macro parses the
                                // value as an Ident and rejects this; mirror
                                // it (and clear the state so a later ident
                                // is not misread as the value).
                                if expect_bp_value {
                                    return Err(CliError::Validation(format!(
                                        "node `{node_type}` field `{port_name}`: unknown \
                                         backpressure policy `{other}`. Supported: \
                                         `drop_oldest`, `block`, `sample(N)`."
                                    )));
                                }
                                // A comma (or anything else) after a bare
                                // `backpressure` means it had no `= value`
                                // — the macro rejects that; mirror it.
                                if saw_backpressure {
                                    return Err(CliError::Validation(format!(
                                        "node `{node_type}` field `{port_name}`: `backpressure` \
                                         requires a value — `backpressure = drop_oldest | block \
                                         | sample(N)`."
                                    )));
                                }
                            }
                        }
                    }
                    if saw_backpressure || expect_bp_value {
                        return Err(CliError::Validation(format!(
                            "node `{node_type}` field `{port_name}`: `backpressure` \
                             requires a value — `backpressure = drop_oldest | block \
                             | sample(N)`."
                        )));
                    }
                    if expect_sample_group {
                        return Err(CliError::Validation(format!(
                            "node `{node_type}` field `{port_name}`: `sample` requires a \
                             parenthesized count — `backpressure = sample(N)`."
                        )));
                    }
                }
            } else if attr.path().is_ident("output") {
                is_output = true;
                // `#[output(...)]` accepts ONLY `promise_within_ms = N`
                // — the variable-field lists / `complex(...)` were removed.
                // Mirror the macro's rejection so `cerulion node info` never
                // reports metadata for source the macro won't compile.
                // Positional: a bare `promise_within_ms`
                // with no `= N` is macro-REJECTED too — the ident alone must
                // not pass. `saw_promise` = just saw the ident (need `=`);
                // `expect_promise_value` = consumed the `=` (need a literal).
                if let syn::Meta::List(meta_list) = &attr.meta {
                    use proc_macro2::TokenTree;
                    let mut saw_promise = false;
                    let mut expect_promise_value = false;
                    let mut promise_count: u32 = 0;
                    let bare_promise_err = || {
                        CliError::Validation(format!(
                            "node `{node_type}` field `{port_name}`: \
                             `promise_within_ms` requires a value — \
                             `#[output(promise_within_ms = N)]`."
                        ))
                    };
                    for tt in meta_list.tokens.clone() {
                        match tt {
                            TokenTree::Literal(_) if expect_promise_value => {
                                expect_promise_value = false;
                            }
                            _ if expect_promise_value => {
                                return Err(bare_promise_err());
                            }
                            TokenTree::Punct(p) if p.as_char() == '=' && saw_promise => {
                                saw_promise = false;
                                expect_promise_value = true;
                            }
                            _ if saw_promise => {
                                return Err(bare_promise_err());
                            }
                            TokenTree::Ident(ident) => {
                                if ident == "promise_within_ms" {
                                    saw_promise = true;
                                    promise_count += 1;
                                } else {
                                    return Err(CliError::Validation(format!(
                                        "node `{node_type}` field `{port_name}`: unknown \
                                         output attribute `{ident}`; #[output] takes no field \
                                         list. Write self.<port>.<field> = expr in the node \
                                         body instead — the macro resolves fixed vs variable \
                                         fields at compile time. #[output] accepts only the \
                                         bare form or `promise_within_ms = N`."
                                    )));
                                }
                            }
                            _ => {}
                        }
                    }
                    if saw_promise || expect_promise_value {
                        return Err(bare_promise_err());
                    }
                    // Macro parity: `parse_output_attr` rejects a duplicate
                    // `promise_within_ms` — mirror it.
                    if promise_count > 1 {
                        return Err(CliError::Validation(format!(
                            "node `{node_type}` field `{port_name}`: duplicate \
                             `promise_within_ms` — declare it at most once."
                        )));
                    }
                }
            }
        }
        if !is_input && !is_output {
            // Plain field (e.g. `tick_count: u32`); not a port.
            continue;
        }
        if is_trigger {
            if single_trigger_input.is_some() {
                multiple_triggers = true;
            } else {
                single_trigger_input = Some(port_name.clone());
            }
        }
        let type_ident = field_type_ident(&field.ty);
        let imported: Option<ImportSchema> = match type_ident.as_deref() {
            None => None,
            Some(leaf) => match resolve_port_type(&scope_chain, leaf) {
                LeafOutcome::Bound(schema) => Some(schema),
                // Said where it was found (a glob re-export, a cycle).
                LeafOutcome::Undecidable => None,
                LeafOutcome::Unbound { globs } => {
                    if !globs.is_empty() {
                        // UNDECIDABLE, loudly: no explicit `use` names this
                        // type and a glob import may bring it in, so no
                        // schema name can be derived — never a bare guess
                        // (`graph validate` then reports the port as
                        // declaring none).
                        tracing::warn!(
                            node_type = %node_type,
                            port = %port_name,
                            r#type = %leaf,
                            globs = ?globs,
                            "port type is named by no explicit `use` while a glob import is in \
                             scope — its schema name cannot be derived; spell `use \
                             <path>::<Type>;` for it"
                        );
                    }
                    None
                }
            },
        };
        let imported = imported.as_ref();
        let schema = imported.map(|i| i.name.clone());
        // A cfg-decided leaf is said HERE, for the port that binds it — the
        // one site that knows a port exists — once per other name it carries.
        let schema_alternatives: Vec<String> = match (type_ident.as_deref(), imported) {
            (Some(leaf), Some(import)) => import
                .cfg_alternatives
                .iter()
                .map(|alt| {
                    warn_cfg_decided_port(node_type, &port_name, leaf, &import.name, alt);
                    alt.name.clone()
                })
                .collect(),
            _ => Vec::new(),
        };
        // A `#[cfg]`-gated glob that is
        // the only path the parser sees to the type is said here too, for
        // the port — a fact about one build, not an alternative.
        if let Some(import) = imported {
            if let Some(glob) = &import.only_path_glob {
                tracing::warn!(
                    node_type = %node_type,
                    port = %port_name,
                    glob = %format!("use {glob}::*;"),
                    schema = %import.name,
                    "a `#[cfg]`-gated glob is the only path the parser sees to this port's \
                     type: the schema reported is the configuration in which that glob \
                     exists, and no alternative is recorded — the parser sees no other \
                     path (with the cfg off the node does not build unless an item the \
                     parser cannot read, an `include!`, binds the type)"
                );
            }
        }
        // Outputs are never triggers, so `is_trigger` (only set from
        // an `#[input(trigger)]` mark) is always false for the output branch.
        // `backpressure` is only ever set from an `#[input(...)]`
        // attr walk, so the output branch always carries the default — the
        // same asymmetry `is_trigger` documents above.
        let port = PortDef {
            name: port_name,
            schema,
            schema_alternatives,
            trigger: is_trigger,
            backpressure,
        };
        if is_input {
            inputs.push(port);
        } else if is_output {
            outputs.push(port);
        }
    }

    // Mirror `cerulion_macros::codegen`'s policy precedence: the
    // node-level attribute wins; otherwise a SINGLE
    // `#[input(trigger)]` field synthesizes `DataTrigger`. Two or
    // more trigger fields without a `sync_window_ms` attr is a
    // macro-time error, but here we just skip the synthesis to
    // mirror the macro's runtime: it'd produce no policy at all.
    let policy = macro_attr.and_then(macro_attr_to_policy).or_else(|| {
        if multiple_triggers {
            None
        } else {
            single_trigger_input.map(|input_name| MacroPolicy::DataTrigger { input_name })
        }
    });

    Ok(Some(NodeMetadata {
        node_type: node_type.to_string(),
        policy,
        inputs,
        outputs,
    }))
}

/// Try to parse `source` as a legacy raw-FFI node by reading the
/// `// CERULION:INFO_START` … `// CERULION:INFO_END` block and
/// extracting port names from the embedded JSON literal.
///
/// Returns `Ok(None)` if no markers are found.
fn try_parse_raw_ffi_node(source: &str, node_type: &str) -> CliResult<Option<NodeMetadata>> {
    let start_marker = "// CERULION:INFO_START";
    let end_marker = "// CERULION:INFO_END";
    let Some(start) = source.find(start_marker) else {
        return Ok(None);
    };
    let Some(end) = source.find(end_marker) else {
        return Ok(None);
    };
    if end < start {
        return Err(CliError::Validation(format!(
            "node '{}' has CERULION:INFO_END before CERULION:INFO_START",
            node_type
        )));
    }
    let block = &source[start..end];

    // The block embeds a static byte string like:
    //   static INFO_BYTES: &[u8] = b"{\"inputs\":[\"a\"],\"outputs\":[]}\0";
    // We extract the bytes between `b"` and `\0"` then defer the
    // un-escape to syn's byte-string literal parser (which handles
    // every Rust escape sequence — `\"`, `\\`, `\n`, `\xNN`, etc.)
    // and the JSON parse to `serde_json` (which handles every
    // JSON escape — distinguishing the two is correct: the bytes
    // are Rust-escaped in the source, JSON-escaped in the literal
    // value).
    let bytes_marker = "static INFO_BYTES: &[u8] = b\"";
    let Some(bytes_start) = block.find(bytes_marker) else {
        return Err(CliError::Validation(format!(
            "node '{}' INFO marker block is missing the INFO_BYTES literal",
            node_type
        )));
    };
    let after_open = bytes_start + bytes_marker.len();
    let Some(close_offset) = block[after_open..].find("\\0\"") else {
        return Err(CliError::Validation(format!(
            "node '{}' INFO_BYTES literal is missing its terminating `\\0\";`",
            node_type
        )));
    };
    let rust_escaped = &block[after_open..after_open + close_offset];

    // Step 1: Rust-source un-escape via `syn::LitByteStr`.
    // Reconstruct a self-contained byte-string literal and let
    // syn's lexer resolve every escape sequence. A
    // hand-rolled `replace("\\\"", "\"").replace(
    // "\\\\", "\\")` handles only two escapes — fragile
    // if `templates::json_escape` ever broadens.
    let reconstructed = format!("b\"{}\"", rust_escaped);
    let bytes_lit: syn::LitByteStr = syn::parse_str(&reconstructed).map_err(|e| {
        CliError::Validation(format!(
            "node '{}' INFO_BYTES literal failed to parse as a Rust byte string: {}",
            node_type, e
        ))
    })?;
    let bytes = bytes_lit.value();
    let json = std::str::from_utf8(&bytes).map_err(|e| {
        CliError::Validation(format!(
            "node '{}' INFO_BYTES content is not valid UTF-8: {}",
            node_type, e
        ))
    })?;

    // Step 2: JSON parse via `serde_json` instead of hand-rolled
    // string searching. Handles arbitrary JSON-escape sequences
    // and validates structure.
    //
    // `policy` is optional and uses the same
    // flat-fields shape as the cdylib FFI's PolicyJson — see
    // `PolicyJson` for the wire-shape contract.
    // As of ABI v9, a raw-FFI `inputs` entry is EITHER a bare string
    // (the shape `generate_info_fn` emits today — trigger membership rides
    // `policy`) OR an object `{"name":..,"trigger":..}` that a hand-written
    // raw-FFI info block (or a future template) may carry. `serde(untagged)`
    // tries the variants in order, so a JSON string matches `Name` and a JSON
    // object matches `Full`; extra QoS keys on the object (`depth` etc.) are
    // ignored here — this parser only needs the name + trigger mark. Outputs
    // never carry a trigger, so they stay bare strings.
    //
    // `backpressure` IS read. A raw-FFI node declaring a `block`
    // input is a co-location constraint for the auto-partitioner
    // exactly like a macro node's, and ignoring the key here would leave that
    // whole class reporting `drop_oldest` at derive time — the partition would
    // split the edge and the consumer's worker would then die at
    // `GraphTopology::validate` blaming an external publisher the user does
    // not have. The wire shape is the ABI-v8 one `cerulion_core::graph::node`'s
    // loader parses: serde EXTERNAL tagging, so `"drop_oldest"` / `"block"`
    // are unit variants from plain strings and `{"sample":N}` is a newtype
    // variant from a single-key map.
    #[derive(serde::Deserialize)]
    #[serde(rename_all = "snake_case")]
    enum RawBackpressure {
        DropOldest,
        Block,
        Sample(u64),
    }
    impl RawBackpressure {
        fn into_policy(self) -> BackpressurePolicy {
            match self {
                Self::DropOldest => BackpressurePolicy::DropOldest,
                Self::Block => BackpressurePolicy::Block,
                Self::Sample(n) => BackpressurePolicy::Sample(n),
            }
        }
    }
    #[derive(serde::Deserialize)]
    #[serde(untagged)]
    enum RawInput {
        Name(String),
        Full {
            name: String,
            #[serde(default)]
            trigger: bool,
            #[serde(default)]
            backpressure: Option<RawBackpressure>,
        },
    }
    #[derive(serde::Deserialize)]
    struct InfoJson {
        #[serde(default)]
        inputs: Vec<RawInput>,
        #[serde(default)]
        outputs: Vec<String>,
        #[serde(default)]
        policy: Option<PolicyJson>,
    }
    // The SECOND host parser of this document. `graph::node`'s
    // parser reports an unknown envelope key; this one reports it too, or
    // which consumer read your info block would decide whether a typo is
    // visible at all. Same `unknown_key=` field name, so ONE grep finds both.
    //
    // The legal set is the DOCUMENT's, not this parser's: `tick_within_ms`
    // and `throttle_ms` are read by the runtime and merely not this
    // consumer's business, so warning about them here would be wrong.
    const LEGAL_ENVELOPE_KEYS: &[&str] = &[
        "inputs",
        "outputs",
        "policy",
        "tick_within_ms",
        "throttle_ms",
    ];
    if let Ok(raw) = serde_json::from_str::<serde_json::Value>(json) {
        if let Some(obj) = raw.as_object() {
            for key in obj.keys() {
                if !LEGAL_ENVELOPE_KEYS.contains(&key.as_str()) {
                    tracing::warn!(
                        node_type = %node_type,
                        section = "<top-level>",
                        unknown_key = %key,
                        "unknown key in INFO_BYTES JSON — ignored; the node loads with the \
                         default for the intended field (accepted: inputs, outputs, policy, \
                         tick_within_ms, throttle_ms)"
                    );
                }
            }
        }
    }
    let info: InfoJson = serde_json::from_str(json).map_err(|e| {
        CliError::Validation(format!(
            "node '{}' INFO_BYTES JSON failed to parse: {}",
            node_type, e
        ))
    })?;
    let policy = info.policy.and_then(PolicyJson::into_macro_policy);
    let (inputs, outputs) = (info.inputs, info.outputs);

    Ok(Some(NodeMetadata {
        node_type: node_type.to_string(),
        policy,
        inputs: inputs
            .into_iter()
            .map(|entry| match entry {
                RawInput::Name(name) => PortDef {
                    name,
                    schema: None,
                    schema_alternatives: Vec::new(),
                    trigger: false,
                    backpressure: BackpressurePolicy::DropOldest,
                },
                RawInput::Full {
                    name,
                    trigger,
                    backpressure,
                } => PortDef {
                    name,
                    schema: None,
                    schema_alternatives: Vec::new(),
                    trigger,
                    backpressure: backpressure
                        .map(RawBackpressure::into_policy)
                        .unwrap_or_default(),
                },
            })
            .collect(),
        outputs: outputs
            .into_iter()
            .map(|name| PortDef {
                name,
                schema: None,
                schema_alternatives: Vec::new(),
                trigger: false,
                backpressure: BackpressurePolicy::DropOldest,
            })
            .collect(),
    }))
}

/// Walk every reachable item, calling `cb` on each (recursing
/// into inline modules so module-nested macro nodes work the same
/// as in `is_macro_based`).
fn walk_items<'a>(items: &'a [syn::Item], cb: &mut dyn FnMut(&'a syn::Item)) {
    for item in items {
        cb(item);
        if let syn::Item::Mod(item_mod) = item {
            if let Some((_, nested)) = &item_mod.content {
                walk_items(nested, cb);
            }
        }
    }
}

/// The schema a port's TYPE ident names in the node's scope — resolved
/// per LEAF, lazily, over the node struct's scope CHAIN (root first):
/// the scope's own `use` items binding the ident first (nearer wins), then
/// what its DIRECT globs make visible from above. See [`resolve_leaf`] for
/// the state space.
fn resolve_port_type(chain: &[&[syn::Item]], type_ident: &str) -> LeafOutcome {
    let mut guard = ResolutionGuard::default();
    resolve_leaf(chain, type_ident, false, &mut guard)
}

/// What a scope has for one leaf.
enum LeafOutcome {
    /// Bound by a `use` in the scope, or one a direct glob makes visible.
    Bound(ImportSchema),
    /// Bound to nothing nameable: a glob re-export, a cyclic re-export
    /// chain, or an undecidable name behind either — said where it was
    /// found, propagated to whoever imports it.
    Undecidable,
    /// No `use` binds it. `globs` are the scope's glob imports (visible
    /// ones when only re-exports were asked for), any of which might supply
    /// it — undecidable for a port, "re-exported through a glob" for a
    /// module.
    Unbound { globs: Vec<String> },
}

/// `(scope, public-only, leaf)` resolutions in progress: a re-export chain
/// that returns to one of them is a cycle. Keyed per LEAF, so resolving
/// `Img` at the root through a module whose `pub use super::*` reaches the
/// root's `Image` is not a cycle (it is not the same leaf).
type ResolutionGuard = std::collections::HashSet<(usize, bool, String)>;

/// Resolve `leaf` — a field's type ident, or the KEY a `use` binds (its
/// leaf, or its `as` rename) — in the scope at the end of `chain`. The
/// state space, in the order it is decided:
///
/// 1. The scope's OWN `use` items binding the leaf, in source order (every
///    one, so a cfg-exclusive pair — two imports of one leaf in one scope,
///    at least one behind a `#[cfg]` — carries both names: the first in
///    source order is the name, every other an alternative; two PLAIN
///    imports of one leaf do not compile, the later shadows). Each is
///    resolved by [`resolve_use_path`]. With `public_only` (asked of a
///    module for what it RE-EXPORTS) only `pub use` items count. An
///    undecidable half makes the leaf undecidable.
/// 2. If the first binding is CONDITIONAL, the name in force with its cfg
///    OFF — an ANCESTOR's binding reached through this scope's DIRECT
///    globs (`use super::*` the parent's, `use super::super::*` the
///    grandparent's, `use crate::*` the root's) — is recorded as its
///    `AncestorShadow` alternative. A narrow glob (`use super::helpers::*`)
///    reaches no ancestor; without a reaching glob the cfg-off
///    configuration has no such name in scope (rustc: E0425) and nothing
///    is recorded.
/// 3. No own binding: the direct globs, in source order, resolve the leaf
///    in the ancestor they reach (an ancestor's PRIVATE imports are visible
///    to its descendants). Bound wins; an undecidable one is undecidable;
///    otherwise the leaf is unbound, with every (visible) glob listed.
///
/// A scope revisited for the SAME leaf while it is being resolved is a
/// cycle: undecidable.
fn resolve_leaf(
    chain: &[&[syn::Item]],
    leaf: &str,
    public_only: bool,
    guard: &mut ResolutionGuard,
) -> LeafOutcome {
    let Some(items) = chain.last().copied() else {
        return LeafOutcome::Unbound { globs: Vec::new() };
    };
    // The key's shape has ONE reader besides its insert/remove pairs (this
    // one, and the module splice's — `resolve_through_module_import` keys a
    // path segment as `mod:<name>`, a spelling no ident can be, so the two
    // namespaces never meet): `resolve_use_path`'s current-scope arm asks
    // whether THIS scope is already resolving `leaf` (under either
    // visibility) to tell the codegen self-reference from an alias of
    // another binding — change it in both. An EMPTY module body is an empty
    // `Vec`, whose pointer is the dangling sentinel every empty body shares;
    // nothing walks INTO an empty body, so no two are ever live at once.
    let key = (items.as_ptr() as usize, public_only, leaf.to_string());
    if !guard.insert(key.clone()) {
        return LeafOutcome::Undecidable;
    }
    let scope = scope_mods_of(items);
    let mut bindings: Vec<(ModPresence, Option<ImportSchema>)> = Vec::new();
    for item in items {
        let syn::Item::Use(item_use) = item else {
            continue;
        };
        if public_only && matches!(item_use.vis, syn::Visibility::Inherited) {
            continue;
        }
        // A `cfg` on the `use` itself (or on a `pub use` re-export) is
        // decided like one on a `mod`: never present ⇒ it imports nothing;
        // always present ⇒ an ordinary import; conditional ⇒ an import that
        // may or may not be in force.
        let presence = mod_presence(&item_use.attrs);
        if presence == ModPresence::Never {
            continue;
        }
        // A LEADING `::` names the external prelude explicitly, so the path
        // cannot be crate-internal no matter what this scope declares —
        // `mod serde { … } use ::serde::Serialize;` compiles and reaches the
        // CRATE (measured).
        let external_root = item_use.leading_colon.is_some();
        for (prefix, leaf_ident) in leaves_named(&item_use.tree, leaf) {
            let resolved = resolve_use_path(
                &prefix,
                &leaf_ident,
                chain,
                &scope,
                external_root,
                guard,
                false,
            );
            bindings.push((presence, resolved));
        }
    }
    let outcome = if bindings.is_empty() {
        resolve_via_globs(chain, &scope, leaf, public_only, guard)
    } else {
        combine_bindings(bindings, chain, &scope, leaf, guard)
    };
    guard.remove(&key);
    outcome
}

/// Step 3 of [`resolve_leaf`]: the leaf through the scope's globs.
fn resolve_via_globs(
    chain: &[&[syn::Item]],
    scope: &ScopeMods<'_>,
    leaf: &str,
    public_only: bool,
    guard: &mut ResolutionGuard,
) -> LeafOutcome {
    let mut globs = Vec::new();
    let mut undecidable = false;
    // Every DIRECT glob's binding, in source order, with the glob's
    // presence. A glob's binding is in force only in the configurations
    // its `use` item exists in (dropping the presence would let
    // `#[cfg(a)] use super::*;` name the first branch's schema in the
    // second build too).
    let mut bound: Vec<(ModPresence, ImportSchema)> = Vec::new();
    let mut unknowable_reach = false;
    let mut conditional_path: Option<String> = None;
    for glob in &scope.globs {
        if public_only && !glob.public {
            continue;
        }
        globs.push(glob.path.clone());
        let Some(depth) = glob_ancestor_depth(&glob.path, chain.len()) else {
            // A glob of something the parser cannot read into — a crate, a
            // narrow module path: it might supply the leaf.
            unknowable_reach = true;
            continue;
        };
        match resolve_leaf(&chain[..=depth], leaf, false, guard) {
            LeafOutcome::Bound(schema) => {
                if glob.presence == ModPresence::Conditional {
                    conditional_path.get_or_insert_with(|| glob.path.clone());
                }
                bound.push((glob.presence, schema));
            }
            LeafOutcome::Undecidable => undecidable = true,
            LeafOutcome::Unbound { .. } => {}
        }
    }
    if undecidable {
        return LeafOutcome::Undecidable;
    }
    // A cfg-gated glob that is the ONLY path to the leaf:
    // no other configuration the parser can see publishes a name, so no
    // alternative is recorded; but the name is a fact about ONE build, so the
    // glob is NOTED for the port that binds the leaf to say — below the
    // unknowable-reach check, which makes such a leaf undecidable instead.
    let only_path_glob = (bound.len() == 1 && bound[0].0 == ModPresence::Conditional)
        .then(|| conditional_path.clone())
        .flatten();
    let mut bound = bound.into_iter();
    let Some((first_presence, mut schema)) = bound.next() else {
        return LeafOutcome::Unbound { globs };
    };
    if first_presence == ModPresence::Conditional && unknowable_reach {
        // With this glob's cfg OFF the leaf comes from a glob the parser
        // cannot read into: the name in the other configuration is
        // unknowable — undecidable, loudly, never the first branch's name.
        tracing::warn!(
            leaf = %leaf,
            globs = ?globs,
            "a conditional glob binds this name in one configuration and another glob the \
             parser cannot read into is in scope for the other — the schema name cannot be \
             derived; spell `use <path>::<Type>;` under each cfg"
        );
        return LeafOutcome::Undecidable;
    }
    if only_path_glob.is_some() {
        schema.only_path_glob = only_path_glob;
    }
    // Further direct globs binding the leaf DIFFERENTLY: with a cfg on
    // either, the other configuration's name — an alternative, like a
    // cfg-exclusive pair of explicit imports; two unconditional globs that
    // disagree do not compile on use (E0659), the first stands.
    let mut any_conditional = first_presence == ModPresence::Conditional;
    for (presence, other) in bound {
        any_conditional |= presence == ModPresence::Conditional;
        if !any_conditional || other.name == schema.name {
            continue;
        }
        let ImportSchema {
            name,
            cfg_alternatives,
            ..
        } = other;
        record_alternative(
            &mut schema,
            CfgAlternative {
                name,
                shape: CfgShape::ExclusivePair,
            },
        );
        for alt in cfg_alternatives {
            record_alternative(&mut schema, alt);
        }
    }
    LeafOutcome::Bound(schema)
}

/// Steps 1–2 of [`resolve_leaf`] over the scope's own bindings of the leaf.
fn combine_bindings(
    bindings: Vec<(ModPresence, Option<ImportSchema>)>,
    chain: &[&[syn::Item]],
    scope: &ScopeMods<'_>,
    leaf: &str,
    guard: &mut ResolutionGuard,
) -> LeafOutcome {
    let mut bindings = bindings.into_iter();
    let (first_presence, first) = bindings.next().expect("non-empty");
    let Some(mut schema) = first else {
        return LeafOutcome::Undecidable;
    };
    let mut any_conditional = first_presence == ModPresence::Conditional;
    for (presence, other) in bindings {
        let Some(other) = other else {
            return LeafOutcome::Undecidable;
        };
        any_conditional |= presence == ModPresence::Conditional;
        if !any_conditional {
            // Two plain imports of one leaf in one scope: E0252 — the later
            // shadows, as a map insert would.
            schema = other;
            continue;
        }
        // APPENDED, never overwritten: a third cfg-exclusive import is a
        // third name, and a graph written for that configuration must not
        // be refused. Said by the port that binds this leaf, if any.
        let ImportSchema {
            name,
            cfg_alternatives,
            ..
        } = other;
        record_alternative(
            &mut schema,
            CfgAlternative {
                name,
                shape: CfgShape::ExclusivePair,
            },
        );
        for alt in cfg_alternatives {
            record_alternative(&mut schema, alt);
        }
    }
    if first_presence == ModPresence::Conditional {
        if let LeafOutcome::Bound(ancestor) = resolve_via_globs(chain, scope, leaf, false, guard) {
            record_alternative(
                &mut schema,
                CfgAlternative {
                    name: ancestor.name,
                    shape: CfgShape::AncestorShadow,
                },
            );
        }
    }
    LeafOutcome::Bound(schema)
}

/// Append `alt` to `schema`'s alternatives unless it names the schema itself
/// or an alternative already recorded — appended, never overwritten (a
/// third cfg-exclusive import is a third name).
fn record_alternative(schema: &mut ImportSchema, alt: CfgAlternative) {
    if alt.name != schema.name && !schema.cfg_alternatives.iter().any(|a| a.name == alt.name) {
        schema.cfg_alternatives.push(alt);
    }
}

/// Every complete path in a `use` tree whose KEY (the leaf, or its `as`
/// rename) is `key`, as `(prefix, leaf ident)`. `{self}` and `{self as key}`
/// name the tree's PREFIX item itself: `use a::b::{self as c};` binds `c`
/// to `a::b`, so the pair is `(["a"], "b")`.
fn leaves_named(tree: &syn::UseTree, key: &str) -> Vec<(Vec<String>, String)> {
    fn walk(
        tree: &syn::UseTree,
        prefix: &mut Vec<String>,
        key: &str,
        out: &mut Vec<(Vec<String>, String)>,
    ) {
        match tree {
            syn::UseTree::Path(path) => {
                prefix.push(path.ident.to_string());
                walk(&path.tree, prefix, key, out);
                prefix.pop();
            }
            syn::UseTree::Group(group) => {
                for item in &group.items {
                    walk(item, prefix, key, out);
                }
            }
            // Read as an ident, `self` would match no module
            // key (`use crate::helpers::{self, Thing}; use helpers::T;` would fall
            // to the extern prelude) and a rename of it would splice a literal
            // `self` segment into the path (`vendor/helpers/self/T`).
            syn::UseTree::Name(name) if name.ident == "self" => {
                if prefix.last().is_some_and(|last| last.as_str() == key) {
                    out.push((prefix[..prefix.len() - 1].to_vec(), key.to_string()));
                }
            }
            syn::UseTree::Rename(rename) if rename.ident == "self" => {
                if rename.rename == key && !prefix.is_empty() {
                    out.push((
                        prefix[..prefix.len() - 1].to_vec(),
                        prefix[prefix.len() - 1].clone(),
                    ));
                }
            }
            syn::UseTree::Name(name) if name.ident == key => {
                out.push((prefix.clone(), name.ident.to_string()));
            }
            syn::UseTree::Rename(rename) if rename.rename == key => {
                out.push((prefix.clone(), rename.ident.to_string()));
            }
            syn::UseTree::Name(_) | syn::UseTree::Rename(_) | syn::UseTree::Glob(_) => {}
        }
    }
    let mut out = Vec::new();
    walk(tree, &mut Vec::new(), key, &mut out);
    out
}

/// The chain depth a glob's PATH reaches, or `None` for a glob that reaches
/// no ancestor scope: `super` (× k) is the k-th ancestor, `crate` the root;
/// anything else — `super::helpers::*`, `crate::x::*`, `vendor::*` — globs
/// a module's items, not the ancestor's own.
fn glob_ancestor_depth(path: &str, chain_len: usize) -> Option<usize> {
    if path == "crate" {
        return Some(0);
    }
    let segments: Vec<&str> = path.split("::").collect();
    if segments.iter().all(|seg| *seg == "super") {
        return (chain_len - 1).checked_sub(segments.len());
    }
    None
}

/// What ONE `use` leaf contributes: the schema NAME, plus — when a `#[cfg]`
/// decided it — every OTHER name the SAME leaf carries in another build
/// configuration, each with the shape that produced it ([`CfgAlternative`]).
/// ONE value per ident, on purpose: the alternatives are scoped to the
/// import that produced them, so a nearer UNCONDITIONAL import shadowing
/// the same leaf replaces the name AND its alternatives at once — an
/// ancestor's alternative cannot outlive the import it belonged to and let
/// `graph validate` accept a mismatched graph through it (two side-by-side
/// maps let exactly that happen). A nearer CONDITIONAL import is the
/// two-configuration shape instead, and keeps the ancestor's name as an
/// alternative. A list, not one slot: cfg-exclusive imports of one leaf can
/// be three, and the third's name silently dropped was a graph written for
/// that configuration refused.
#[derive(Debug, Clone, PartialEq, Eq)]
struct ImportSchema {
    name: String,
    cfg_alternatives: Vec<CfgAlternative>,
    /// The `#[cfg]`-gated glob that is
    /// the ONLY path the parser sees to this name — no own binding, one
    /// direct glob binds it and that glob is conditional. Not an
    /// alternative (no other configuration the parser can see publishes a
    /// name) but a fact about ONE build, said for the port that binds the
    /// leaf — like the alternatives, and for the same reason (said from the
    /// glob walk it would name no port, and said from the ancestor-shadow
    /// probe its claim would be false).
    only_path_glob: Option<String>,
}

/// The other name a cfg-decided leaf carries, and WHY it carries two. Kept
/// in the map rather than said at import time, so the advisory is emitted by
/// the one site that knows a PORT binds the leaf
/// ([`warn_cfg_decided_port`], from the field walk): said while walking the
/// `use` items it was a confident claim about "this port's schema name" for
/// an import no port used — and on a raw-FFI node, whose map nothing reads,
/// for ports that do not exist.
#[derive(Debug, Clone, PartialEq, Eq)]
struct CfgAlternative {
    name: String,
    shape: CfgShape,
}

/// The five shapes that give one leaf two names. Only the gated-module,
/// gated-glob and gated-import shapes have an import spelling that ends the
/// ambiguity, which is why the advisory is per shape and the validator's
/// own line stays shape-neutral.
#[derive(Debug, Clone, PartialEq, Eq)]
enum CfgShape {
    /// A `#[cfg]`-gated module in the `use`'s own scope beside a same-named
    /// crate: `module` is that module, `import` the full `use` path — the
    /// `::` / `self::` spellings are built from it.
    GatedModule { module: String, import: String },
    /// A module an ANCESTOR declares, reached from the `use`'s scope through
    /// a glob, where the glob (`#[cfg(feature = "x")] use super::*;`) or the
    /// module itself (`#[cfg(feature = "x")] mod m`) is `#[cfg]`-gated:
    /// `glob` is the reaching glob's path, `module` the module, `import` the
    /// full `use` path. With the cfg OFF the path resolves to an extern crate
    /// of the module's name (dropping the glob's presence would
    /// make a module reached only through a gated glob read as
    /// visible in every build).
    GatedGlob {
        glob: String,
        module: String,
        import: String,
        /// Whether the GLOB carries the cfg (`#[cfg] use super::*`) or the
        /// ancestor MODULE does (`#[cfg] mod m` reached by a plain glob).
        glob_gated: bool,
    },
    /// A `#[cfg]`-gated `use` binding the MODULE the path starts with
    /// (`#[cfg(feature = "v")] use vendor::foo; use foo::Type;` — spliced
    /// as if unconditional, the cfg-off name would be dropped —
    /// the gated-module defect one binding smaller): `module` is that name,
    /// `import` the full `use` path. With every such import off the path
    /// names what it names WITHOUT them — an ancestor's module a glob
    /// reaches, else an extern crate of the module's name.
    GatedImport { module: String, import: String },
    /// Two (or more) imports of the leaf — or of the MODULE its path starts
    /// with — in one scope, at least one behind a `#[cfg]`: cfg-exclusive,
    /// or a plain one beside a conditional one
    /// (one buildable configuration plus one E0252; the parser does not pick
    /// the buildable half — source order does).
    ExclusivePair,
    /// A conditional import in the node's scope shadowing an ancestor
    /// scope's plain import of the leaf.
    AncestorShadow,
}

/// Say, for the PORT that binds it, why a leaf carries two names — one
/// constant message per shape, every value an operator acts on a field: the
/// port, the name reported, the name the OTHER configuration publishes, and
/// (for the two shapes that have them) the two spellings that end the
/// ambiguity.
fn warn_cfg_decided_port(
    node_type: &str,
    port: &str,
    leaf: &str,
    reported: &str,
    alternative: &CfgAlternative,
) {
    match &alternative.shape {
        CfgShape::GatedModule { module, import } => tracing::warn!(
            node_type = %node_type,
            port = %port,
            module = %module,
            import = %import,
            reported = %reported,
            alternative = %alternative.name,
            crate_spelling = %format!("use ::{import};"),
            module_spelling = %format!("use self::{import};"),
            "a `#[cfg]`-gated module in the node's own scope decides this port's \
             schema name: reported as the BARE workspace-schema name (the \
             configuration in which the module exists). With that cfg OFF at build \
             time and an external crate of the module's name providing the type, \
             the node publishes the alternative instead — `graph validate` accepts \
             either spelling for this port, with a warning. Make it unambiguous: \
             the crate spelling for the crate, the module spelling for the module"
        ),
        CfgShape::GatedGlob {
            glob,
            module,
            import,
            glob_gated,
        } => tracing::warn!(
            node_type = %node_type,
            port = %port,
            glob = %format!("use {glob}::*;"),
            module = %module,
            gated = %if *glob_gated { "the glob" } else { "the module" },
            import = %import,
            reported = %reported,
            alternative = %alternative.name,
            module_spelling = %format!("use {glob}::{import};"),
            crate_spelling = %format!("use ::{import};"),
            "a `#[cfg]` decides this port's schema name: the module the path starts \
             with is an ancestor's, reached from this scope through a glob, and either \
             that glob or the module itself is cfg-gated (`gated` says which), so it is \
             reported as the BARE workspace-schema name (the configuration in which the \
             reach exists). With that cfg OFF at build time the path resolves to an \
             external crate of the module's name and the node publishes the alternative \
             instead — `graph validate` accepts either spelling for this port, with a \
             warning. Make it unambiguous: the module spelling names the ancestor's \
             module by path, the crate spelling the crate"
        ),
        CfgShape::GatedImport { module, import } => tracing::warn!(
            node_type = %node_type,
            port = %port,
            module = %module,
            import = %import,
            reported = %reported,
            alternative = %alternative.name,
            crate_spelling = %format!("use ::{import};"),
            "a `#[cfg]`-gated `use` binds the module this port's type path starts with \
             and decides its schema name: reported as the name that import gives (the \
             configuration in which the import exists). With every such cfg OFF at build \
             time the path starts at what is in scope without it — an ancestor's module a \
             glob reaches, else an extern crate of the module's name — and the node \
             publishes the alternative instead; `graph validate` accepts either spelling \
             for this port, with a warning. Make it unambiguous: spell the type by its \
             crate under each cfg, or the crate spelling for the crate"
        ),
        CfgShape::ExclusivePair => tracing::warn!(
            node_type = %node_type,
            port = %port,
            leaf = %leaf,
            first = %reported,
            other = %alternative.name,
            "two imports of one name — the type, or the module its path starts with — in \
             the node's own scope, at least one behind a \
             `#[cfg]`, decide this port's schema name: the first in source order is \
             reported as the port's schema name and each other as a cfg alternative — \
             `graph validate` accepts any of them, with a warning; the parser cannot see \
             which build the graph is for"
        ),
        CfgShape::AncestorShadow => tracing::warn!(
            node_type = %node_type,
            port = %port,
            leaf = %leaf,
            first = %reported,
            second = %alternative.name,
            "a cfg-gated import in the node's own scope shadows an ancestor scope's \
             import of the same type and decides this port's schema name: the nearer one \
             is reported as the port's schema name and the ancestor's as its cfg \
             alternative — `graph validate` accepts either, with a warning"
        ),
    }
}

/// The chain of module scopes from the crate root down to and including the
/// one declaring the FIRST `#[cerulion_node]` struct — the same struct, found
/// in the same order, that the caller's `walk_items` pass picks.
///
/// `None` when no macro struct is present (a raw-FFI node, or a file that
/// declares none); the caller then falls back to the root scope, whose map
/// nothing reads on those paths.
fn node_scope_chain(items: &[syn::Item]) -> Option<Vec<&[syn::Item]>> {
    for item in items {
        if let syn::Item::Struct(item_struct) = item {
            if item_struct
                .attrs
                .iter()
                .any(|attr| attr.path().is_ident("cerulion_node"))
            {
                return Some(vec![items]);
            }
        }
        if let syn::Item::Mod(item_mod) = item {
            if let Some((_, nested)) = &item_mod.content {
                if let Some(mut chain) = node_scope_chain(nested) {
                    chain.insert(0, items);
                    return Some(chain);
                }
            }
        }
    }
    None
}

/// Whether an ITEM — a module, or a `use` — exists in the compiled node,
/// as far as its `cfg` attributes can be decided without a cfg set. The
/// variants say "module" for brevity; a `use` is classified identically.
/// `Conditional` is the PERMISSIVE tier: it is where every undecidable and
/// every unreadable attribute lands, and it widens what `graph validate`
/// accepts (to a pass with a warning) rather than narrowing it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ModPresence {
    /// No `cfg` constrains it, or every constraint is statically true: the
    /// module is always there — a plain local module.
    Always,
    /// Some `cfg` is statically FALSE: the module can never exist, so it
    /// declares nothing — a same-named `use` reaches the external crate,
    /// unambiguously (the fixture: `#[cfg(any())] mod x`).
    Never,
    /// A constraint depends on a feature, target or custom key the parser
    /// cannot see: the module may or may not exist — the GATE, which keeps
    /// the original bare-plus-alternative contract.
    Conditional,
}

/// Fold a module's attributes into its [`ModPresence`]: `Never` if any
/// attribute makes it impossible, else `Conditional` if any leaves it
/// undecidable, else `Always`.
fn mod_presence(attrs: &[syn::Attribute]) -> ModPresence {
    fold_presence(attrs.iter().map(|attr| attr_presence(&attr.meta)))
}

fn fold_presence(parts: impl Iterator<Item = ModPresence>) -> ModPresence {
    let mut out = ModPresence::Always;
    for part in parts {
        match part {
            ModPresence::Never => return ModPresence::Never,
            ModPresence::Conditional => out = ModPresence::Conditional,
            ModPresence::Always => {}
        }
    }
    out
}

/// One attribute's contribution to a module's presence. A direct `cfg(p)`
/// is decided by [`static_cfg_value`]: false ⇒ `Never`, true ⇒ `Always`,
/// undecidable ⇒ `Conditional`. A `cfg_attr(p, …)` whose predicate is
/// statically false never emits its payload (no constraint); one whose
/// predicate is statically true emits it as if written directly; one whose
/// predicate is undecidable constrains the module only if its payload
/// carries a real constraint — an emitted `cfg(all())` under an unknown
/// feature still means "always there". Anything else is not a `cfg`. A
/// `cfg` or `cfg_attr` whose arguments the parser cannot READ — an
/// unparseable list, or a payload that is a bare literal rather than an
/// attribute — is undecidable — `Conditional`, the tier that decides nothing — never a
/// decided answer (reading an unreadable `cfg_attr` as `Always` would be a
/// confident choice on an attribute nobody has parsed).
fn attr_presence(meta: &syn::Meta) -> ModPresence {
    if meta.path().is_ident("cfg") {
        let predicate = match meta {
            syn::Meta::List(list) => list.parse_args::<CfgTerm>().ok(),
            _ => None,
        };
        return match predicate.as_ref().and_then(static_cfg_value) {
            Some(false) => ModPresence::Never,
            Some(true) => ModPresence::Always,
            None => ModPresence::Conditional,
        };
    }
    if !meta.path().is_ident("cfg_attr") {
        return ModPresence::Always;
    }
    let syn::Meta::List(list) = meta else {
        return ModPresence::Conditional;
    };
    let Some(items) = cfg_terms(list) else {
        return ModPresence::Conditional;
    };
    let mut items = items.iter();
    let predicate = items.next().and_then(static_cfg_value);
    let emitted = fold_presence(items.map(|term| match term {
        CfgTerm::Meta(meta) => attr_presence(meta),
        // A bare literal is not an attribute the parser can read (rustc
        // rejects it): undecidable, like any other unreadable payload.
        CfgTerm::Bool(_) => ModPresence::Conditional,
    }));
    match predicate {
        Some(false) => ModPresence::Always,
        Some(true) => emitted,
        None => match emitted {
            ModPresence::Always => ModPresence::Always,
            _ => ModPresence::Conditional,
        },
    }
}

/// One term of a cfg predicate list: `syn::Meta` cannot parse the bare
/// literals `true` / `false` (they are `Lit`s, not paths), so a predicate
/// term is either of the two.
enum CfgTerm {
    Bool(syn::LitBool),
    /// Boxed: a `Meta` is ~300 bytes against the literal's 16.
    Meta(Box<syn::Meta>),
}

impl syn::parse::Parse for CfgTerm {
    fn parse(input: syn::parse::ParseStream) -> syn::Result<Self> {
        if input.peek(syn::LitBool) {
            Ok(Self::Bool(input.parse()?))
        } else {
            Ok(Self::Meta(Box::new(input.parse()?)))
        }
    }
}

/// The comma-separated terms of a `cfg_attr(…)` / `all(…)` / `any(…)` /
/// `not(…)` list, or `None` when they do not parse.
fn cfg_terms(list: &syn::MetaList) -> Option<syn::punctuated::Punctuated<CfgTerm, syn::Token![,]>> {
    list.parse_args_with(syn::punctuated::Punctuated::<CfgTerm, syn::Token![,]>::parse_terminated)
        .ok()
}

/// The value of a cfg predicate when it is decidable WITHOUT a cfg set:
/// `true` / `false`, `all()` (true) / `any()` (false), `not(p)`, and
/// `all(...)` / `any(...)` decided as soon as one term decides them (an
/// undecidable sibling does not make a decided `all` or `any` undecidable —
/// every term is evaluated, the decision is what short-circuits). `None`
/// for anything that depends on the build — a feature, a target, a custom
/// key — which the parser must not guess.
fn static_cfg_value(term: &CfgTerm) -> Option<bool> {
    let meta = match term {
        CfgTerm::Bool(lit) => return Some(lit.value),
        CfgTerm::Meta(meta) => meta.as_ref(),
    };
    match meta {
        syn::Meta::List(list) => {
            let ident = list.path.get_ident()?.to_string();
            let items = cfg_terms(list)?;
            let values: Vec<Option<bool>> = items.iter().map(static_cfg_value).collect();
            match ident.as_str() {
                "all" => {
                    if values.contains(&Some(false)) {
                        Some(false)
                    } else if values.iter().all(|v| *v == Some(true)) {
                        Some(true)
                    } else {
                        None
                    }
                }
                "any" => {
                    if values.contains(&Some(true)) {
                        Some(true)
                    } else if values.iter().all(|v| *v == Some(false)) {
                        Some(false)
                    } else {
                        None
                    }
                }
                "not" => match values.as_slice() {
                    [Some(v)] => Some(!v),
                    _ => None,
                },
                _ => None,
            }
        }
        _ => None,
    }
}

/// The modules ONE scope declares, as the same-scope `use` rule sees them,
/// and what the scope's glob imports let it reach.
#[derive(Default)]
struct ScopeMods<'a> {
    /// Every `mod` declared directly in the scope that CAN exist in some
    /// build — a statically-impossible one (`#[cfg(any())]`) declares
    /// nothing and is left out.
    declared: std::collections::HashSet<String>,
    /// The subset whose existence the parser cannot decide
    /// ([`ModPresence::Conditional`]) — present in the build only when an
    /// undecidable cfg holds. A statically-decidable cfg is resolved
    /// instead: an always-present module is a plain local, a never-present
    /// one is not in `declared` at all.
    cfg_gated: std::collections::HashSet<String>,
    /// The bodies of the declared modules that are INLINE (`mod m { … }`) —
    /// what a `pub use` re-export can be read from. An out-of-line
    /// `mod m;` has no body here and reads as the codegen shape.
    inline: std::collections::HashMap<String, &'a [syn::Item]>,
    /// Every glob import in the scope, as its path (`super`, `crate::x`,
    /// `cerulion_core::prelude`) and whether it is a `pub use` (re-exported).
    /// A DIRECT ancestor glob — exactly `super` (× k) or `crate` — is what
    /// makes that ancestor's items (its imports, its modules) reachable by
    /// bare name here; a narrow one (`super::helpers::*`) reaches only the
    /// module it names. Without a reaching glob an ancestor's import is NOT
    /// in this scope (rustc: E0425 with a nearer conditional import off),
    /// so it can neither be a cfg alternative of a nearer one nor name a
    /// module. Each carries the `use` item's own presence: a CONDITIONAL
    /// glob binds a leaf in one configuration only.
    globs: Vec<Glob>,
}

/// One glob import: the path it globs from, whether it is a `pub use`, and
/// whether the `use` item is conditional.
struct Glob {
    path: String,
    public: bool,
    presence: ModPresence,
}

/// [`ScopeMods`] for one scope's items.
fn scope_mods_of(items: &[syn::Item]) -> ScopeMods<'_> {
    let mut scope = ScopeMods::default();
    for item in items {
        match item {
            syn::Item::Mod(item_mod) => {
                let name = item_mod.ident.to_string();
                match mod_presence(&item_mod.attrs) {
                    // A module that can never exist declares nothing: the
                    // same-named `use` reaches the external crate.
                    ModPresence::Never => continue,
                    ModPresence::Always => {}
                    ModPresence::Conditional => {
                        scope.cfg_gated.insert(name.clone());
                    }
                }
                if let Some((_, content)) = &item_mod.content {
                    scope.inline.insert(name.clone(), content.as_slice());
                }
                scope.declared.insert(name);
            }
            syn::Item::Use(item_use) => {
                let presence = mod_presence(&item_use.attrs);
                if presence == ModPresence::Never {
                    continue;
                }
                let public = !matches!(item_use.vis, syn::Visibility::Inherited);
                collect_globs(
                    &item_use.tree,
                    &mut Vec::new(),
                    public,
                    presence,
                    &mut scope.globs,
                );
            }
            _ => {}
        }
    }
    scope
}

/// Every glob (`…::*`) in a `use` tree, as the path it globs from, with
/// the item's visibility and presence.
fn collect_globs(
    tree: &syn::UseTree,
    prefix: &mut Vec<String>,
    public: bool,
    presence: ModPresence,
    out: &mut Vec<Glob>,
) {
    match tree {
        syn::UseTree::Path(path) => {
            prefix.push(path.ident.to_string());
            collect_globs(&path.tree, prefix, public, presence, out);
            prefix.pop();
        }
        syn::UseTree::Group(group) => {
            for item in &group.items {
                collect_globs(item, prefix, public, presence, out);
            }
        }
        syn::UseTree::Glob(_) => out.push(Glob {
            path: prefix.join("::"),
            public,
            presence,
        }),
        syn::UseTree::Name(_) | syn::UseTree::Rename(_) => {}
    }
}

/// One `use` path — `prefix::leaf` — resolved over the scope's CHAIN into
/// the schema it names, or `None` when it is UNDECIDABLE (said here). The
/// state space, in the order it is decided:
///
/// | the path's ROOT                                        | reaches                                   |
/// |---|---|
/// | `::x`                                                  | the external crate `x`                    |
/// | `crate`                                                | the crate root scope                      |
/// | `self`                                                 | this scope                                |
/// | `super` × k                                            | the k-th ancestor (above the root: bare — crate-internal, unwalkable) |
/// | a module THIS scope declares, inline                   | that module's body (cfg-gated ⇒ the crate spelling is an alternative) |
/// | a module THIS scope declares, out-of-line              | no body to read: the codegen shape, bare  |
/// | a module an ANCESTOR declares, reached by a DIRECT glob (`use super::*`, `use crate::*`) | as above, inline or out-of-line — gated when the GLOB is (`#[cfg] use super::*` reaches it in one build only; a glob that can never exist reaches nothing) |
/// | anything else                                          | the external crate of that name           |
///
/// The remaining segments walk INLINE sub-modules (an out-of-line or
/// never-present one: the codegen shape, bare). The leaf is then resolved
/// in the reached scope by [`resolve_leaf`] — its FULL bindings when that
/// scope is this one or an ancestor (a descendant sees an ancestor's
/// private imports), its PUBLIC re-exports otherwise (E0603) — so a
/// re-export chain through sibling modules (`mod first { pub use …::Image;
/// } mod aliases { pub use super::first::Image; }`) resolves to its target,
/// and a cycle is caught by the per-leaf guard.
///
/// | the leaf in the reached scope      | verdict |
/// |---|---|
/// | bound                              | that binding (+ the gated alternative) |
/// | unbound, the scope has a glob      | UNDECIDABLE: "re-exported through a glob", loud |
/// | undecidable there (glob, cycle)    | UNDECIDABLE, propagated: loud |
/// | unbound, no glob                   | the codegen shape: bare (+ the gated alternative) |
///
/// Every row of both tables is a row of `the_use_walk_state_space`.
///
/// Schema-name convention for what is reached:
/// - `use native_ros2_messages::sensor_msgs::Image;` → `"sensor_msgs/Image"`
///   (the codegen target crate is not part of the schema namespace);
/// - `use unitree_go::Go2FrontVideoData;` → `"unitree_go/Go2FrontVideoData"`
///   (an external message crate carries its package as its crate name —
///   what the `.msg` store and the graph YAML both spell);
/// - the codegen shape (`mod detections { include!(…) } use
///   detections::DetectionArray;`) → `"DetectionArray"`, BARE: a workspace
///   schema's canonical name is its entry name, and qualifying it with a
///   private module invents a package no workspace can resolve (it would
///   refuse the `examples/perception` graphs);
/// - `use std::collections::HashMap;` → `"std/collections/HashMap"` (not a
///   Cerulion schema in practice; recorded consistently);
/// - `use Image;` (single segment) → nothing to qualify: unbound.
fn resolve_use_path(
    prefix: &[String],
    leaf: &str,
    chain: &[&[syn::Item]],
    scope: &ScopeMods<'_>,
    external_root: bool,
    guard: &mut ResolutionGuard,
    skip_root_import: bool,
) -> Option<ImportSchema> {
    if prefix.is_empty() {
        // Root-level `use Image;` — nothing to qualify.
        return None;
    }
    if external_root {
        return Some(external_import(prefix, leaf));
    }
    // The crate spelling a gated module's OTHER configuration publishes
    // (measured — see `resolve_leaf`), spelled by the external arm's rule
    // with every segment of the original path.
    let import = || format!("{}::{leaf}", prefix.join("::"));
    let gated_module = || CfgShape::GatedModule {
        module: prefix[0].clone(),
        import: import(),
    };
    let gated_alternative = |shape: CfgShape| CfgAlternative {
        name: external_schema_name(prefix, leaf),
        shape,
    };
    let bare = |gated: Option<CfgShape>| {
        // The codegen shape: report the leaf alone and let the caller's
        // resolver decide what that bare name means (a workspace schema
        // wins there, exactly as it does everywhere else). A gated module
        // decides the name only while its cfg holds; with it off a
        // same-named extern crate takes the import, so the other name is
        // RECORDED for the validator and said by the port that binds it.
        let mut cfg_alternatives = Vec::new();
        if let Some(shape) = gated {
            let alt = gated_alternative(shape);
            if alt.name != leaf {
                cfg_alternatives.push(alt);
            }
        }
        Some(ImportSchema {
            name: leaf.to_string(),
            cfg_alternatives,
            only_path_glob: None,
        })
    };
    // Where the path STARTS.
    let (mut target, rest, gated): (Vec<&[syn::Item]>, &[String], Option<CfgShape>) = match prefix
        [0]
    .as_str()
    {
        "crate" => (vec![chain[0]], &prefix[1..], None),
        "self" => (chain.to_vec(), &prefix[1..], None),
        "super" => {
            let k = prefix.iter().take_while(|seg| *seg == "super").count();
            if k >= chain.len() {
                // Above the crate root: crate-internal by keyword, with
                // nothing the parser can walk to (rustc rejects it, E0433;
                // a `lib.rs` the parser reads on its own may sit below a
                // parent it cannot see) — the codegen shape, bare.
                return bare(None);
            }
            (chain[..chain.len() - k].to_vec(), &prefix[k..], None)
        }
        module if scope.declared.contains(module) => {
            let gated = scope.cfg_gated.contains(module).then(gated_module);
            match scope.inline.get(module) {
                Some(body) => {
                    let mut target = chain.to_vec();
                    target.push(body);
                    (target, &prefix[1..], gated)
                }
                None => return bare(gated),
            }
        }
        module => {
            // Not declared here: a `use path::module;` of THIS scope binding
            // the name continues the path through it (`use other::foo; use
            // foo::Type;` names `other::foo::Type`) — an explicit import
            // shadows what a glob reaches (consulted BEFORE the ancestor
            // reach, as rustc does) — before an
            // ancestor's module the scope's globs reach, before the extern
            // prelude is assumed. `skip_root_import` is the recursive
            // "what does the path name WITHOUT that import" question.
            if !skip_root_import {
                if let Some((out, every_conditional)) =
                    resolve_through_module_import(module, &prefix[1..], leaf, chain, false, guard)
                {
                    let mut schema = out?;
                    if every_conditional {
                        // With every such import off the path names what the
                        // rest of this arm says it names — recorded with its
                        // own alternatives, said by the port that binds the
                        // leaf.
                        if let Some(off) =
                            resolve_use_path(prefix, leaf, chain, scope, false, guard, true)
                        {
                            let shape = CfgShape::GatedImport {
                                module: module.to_string(),
                                import: import(),
                            };
                            let ImportSchema {
                                name,
                                cfg_alternatives,
                                ..
                            } = off;
                            record_alternative(&mut schema, CfgAlternative { name, shape });
                            for alt in cfg_alternatives {
                                record_alternative(&mut schema, alt);
                            }
                        }
                    }
                    return Some(schema);
                }
            }
            match ancestor_module_via_globs(module, chain, scope) {
                // Neither declared, imported, nor an ancestor's reached
                // through a glob: the extern prelude.
                None => return Some(external_import(prefix, leaf)),
                Some((depth, body, gate)) => {
                    let gated = match gate {
                        AncestorGate::Open => None,
                        AncestorGate::ViaGlob { glob, glob_gated } => Some(CfgShape::GatedGlob {
                            glob,
                            module: module.to_string(),
                            import: import(),
                            glob_gated,
                        }),
                    };
                    match body {
                        Some(body) => {
                            let mut target = chain[..=depth].to_vec();
                            target.push(body);
                            (target, &prefix[1..], gated)
                        }
                        // An out-of-line ancestor module the glob makes
                        // visible: crate-internal, with no body to read —
                        // the codegen shape.
                        None => return bare(gated),
                    }
                }
            }
        }
    };
    // The remaining segments walk inline sub-modules — or a `use` of the
    // reached scope that binds the segment as a MODULE name (the merged
    // fixtures: `use external_crate::foo;` at the root, then
    // `use crate::foo::Type;` in a nested scope names `external_crate::foo::
    // Type`), through which the path continues in that scope's own context.
    // A scope off the node's chain is asked what it RE-EXPORTS: its private
    // `use` is no re-export (E0603) — bare, exactly as for a leaf.
    // Every such import conditional and off: the path does not resolve at
    // all (E0432), so no other build publishes a name — nothing recorded.
    for (i, segment) in rest.iter().enumerate() {
        let reached_items = *target.last().expect("non-empty");
        if let Some(body) = inline_module(reached_items, segment) {
            target.push(body);
            continue;
        }
        let public_only = !chain
            .iter()
            .any(|items| std::ptr::eq(*items, reached_items));
        if let Some((out, _every_conditional)) = resolve_through_module_import(
            segment,
            &rest[i + 1..],
            leaf,
            &target,
            public_only,
            guard,
        ) {
            return out;
        }
        return bare(gated);
    }
    // The leaf, in the reached scope.
    let reached = *target.last().expect("non-empty");
    let is_own_scope = std::ptr::eq(reached, *chain.last().expect("non-empty"));
    if is_own_scope {
        // `use self::Leaf [as Alias];` (or `crate::` at the root) names an
        // item of THIS scope. When the walk arrived here FROM `Leaf` itself —
        // the guard holds this scope's key for it — the `use` IS the binding
        // under resolution (`pub use self::Image;` beside the generated
        // struct: the codegen shape), there is nothing further to follow, and
        // it stays bare. Otherwise the leaf is ANOTHER binding of this scope
        // (`pub use sensor_msgs::Image; pub use self::Image as Frame;`) and
        // `Frame` resolves through it exactly as a sibling module's re-export
        // would — returning the bare leaf here would report `Frame` as
        // a workspace `Image` while the binding it aliases says
        // `sensor_msgs/Image`. A leaf no `use` of this scope binds then
        // answers exactly as the port's own type would: bare with no glob in
        // scope (the codegen shape), undecidable under one.
        // Both visibilities: this scope is asked what it RE-EXPORTS
        // (`public_only == true`) when another module's `use` reaches into
        // it, and what it BINDS (`false`) on its own chain — the key under
        // resolution carries whichever question was asked, and either one
        // means a frame is resolving THIS leaf in THIS scope.
        let own = reached.as_ptr() as usize;
        if [false, true]
            .into_iter()
            .any(|public_only| guard.contains(&(own, public_only, leaf.to_string())))
        {
            return bare(gated);
        }
    }
    let public_only = !chain.iter().any(|items| std::ptr::eq(*items, reached));
    match resolve_leaf(&target, leaf, public_only, guard) {
        LeafOutcome::Bound(mut schema) => {
            if let Some(shape) = gated {
                record_alternative(&mut schema, gated_alternative(shape));
            }
            Some(schema)
        }
        LeafOutcome::Unbound { globs } if is_own_scope && !globs.is_empty() => {
            // The alias names an item of THIS scope that no explicit `use`
            // binds while a glob is in scope — the port walk's own verdict
            // for such a type, said with its remedy (there is no module to
            // re-export it from).
            tracing::warn!(
                leaf = %leaf,
                path = %format!("{}::{leaf}", prefix.join("::")),
                globs = ?globs,
                "the alias names an item of this scope that no explicit `use` binds while \
                 a glob import is in scope — its schema name cannot be derived; declare the \
                 type in a module and import it by name (`mod m {{ include!(…); }} use \
                 m::<Type>;`), or drop the glob"
            );
            None
        }
        LeafOutcome::Unbound { globs } if !globs.is_empty() => {
            // UNDECIDABLE, loudly: the module re-exports an unknowable set,
            // so the leaf's schema cannot be derived — never a bare guess,
            // and never the older binding a glob made visible.
            tracing::warn!(
                leaf = %leaf,
                module = %prefix.join("::"),
                globs = ?globs,
                "a local module re-exports through a glob — the schema name of a type \
                 imported from it cannot be derived; re-export it by name (`pub use \
                 <path>::<Type>;`) or import it from its crate"
            );
            None
        }
        LeafOutcome::Undecidable => {
            tracing::warn!(
                leaf = %leaf,
                module = %prefix.join("::"),
                "a local module could not decide this name — a glob re-export or a cyclic \
                 re-export chain behind it; the schema name cannot be derived. Import the \
                 type from its crate"
            );
            None
        }
        LeafOutcome::Unbound { .. } => bare(gated),
    }
}

/// A type imported from OUTSIDE the node's crate, by its `use` path.
fn external_import(prefix: &[String], leaf: &str) -> ImportSchema {
    ImportSchema {
        name: external_schema_name(prefix, leaf),
        cfg_alternatives: Vec::new(),
        only_path_glob: None,
    }
}

/// A `use` binding a name as a path ROOT — the module a path's segment
/// continues through.
struct ModuleImport {
    presence: ModPresence,
    /// The path in front of the bound item, and the item's own ident:
    /// `use a::b as name;` → `["a"]`, `"b"`; `use a::b::{self as name};` →
    /// `["a"]`, `"b"`.
    prefix: Vec<String>,
    ident: String,
    /// Anchored at the extern prelude (`use ::…`).
    external_root: bool,
}

/// Every `use path::name;` / `use path::x as name;` / `use path::name::
/// {self};` among `items` that CAN exist, in source order, each with its
/// presence — ALL of them, because a cfg-exclusive pair binds one name under
/// two cfgs (taking the first as if unconditional would rest on
/// the premise that a scope binds a name once — true of two PLAIN imports
/// only, E0252). `public_only` (a scope off the node's chain, asked what it
/// RE-EXPORTS) skips private ones. `use name;` of the crate itself binds
/// nothing new and is not one.
fn module_imports_in(items: &[syn::Item], name: &str, public_only: bool) -> Vec<ModuleImport> {
    let mut out = Vec::new();
    for item in items {
        let syn::Item::Use(item_use) = item else {
            continue;
        };
        if public_only && matches!(item_use.vis, syn::Visibility::Inherited) {
            continue;
        }
        let presence = mod_presence(&item_use.attrs);
        if presence == ModPresence::Never {
            continue;
        }
        for (prefix, ident) in leaves_named(&item_use.tree, name) {
            if prefix.is_empty() && ident == name {
                continue;
            }
            out.push(ModuleImport {
                presence,
                prefix,
                ident,
                external_root: item_use.leading_colon.is_some(),
            });
        }
    }
    out
}

/// Continue a path whose segment `module` a `use` of the reached scope (the
/// end of `chain`) binds as a MODULE name rather than declaring (the
/// merged fixtures: `use external_crate::foo;` at the root, then
/// `use crate::foo::Type;` below names `external_crate::foo::Type`): each
/// such import, in source order, is spliced in front of `tail` and resolved
/// in that scope's own context; the first is the name and every other a
/// cfg alternative, exactly as [`combine_bindings`] folds a scope's imports
/// of a LEAF (two plain imports of one name do not compile, E0252 — the
/// later shadows; a cfg-exclusive pair carries both). `public_only` skips
/// the scope's private `use` items.
///
/// `None`: no such import. `Some((None, _))`: undecidable — a knot of module
/// imports (`use self::b as a; use self::a as b;`), or one half undecidable
/// — said where it was found. The flag says whether EVERY import found is
/// `#[cfg]`-conditional, for the caller that knows what the path names with
/// all of them off (the root segment: an ancestor's module or the extern
/// prelude; a later segment: nothing — the path does not resolve).
fn resolve_through_module_import(
    module: &str,
    tail: &[String],
    leaf: &str,
    chain: &[&[syn::Item]],
    public_only: bool,
    guard: &mut ResolutionGuard,
) -> Option<(Option<ImportSchema>, bool)> {
    let reached = *chain.last().expect("non-empty");
    let imports = module_imports_in(reached, module, public_only);
    if imports.is_empty() {
        return None;
    }
    // Guarded like a leaf, in a namespace of its own (`mod:` is no ident):
    // `use self::a as b; use self::b as a;`-style knots would otherwise
    // splice forever.
    let key = (reached.as_ptr() as usize, false, format!("mod:{module}"));
    if !guard.insert(key.clone()) {
        tracing::warn!(
            leaf = %leaf,
            module = %module,
            "a cyclic chain of module imports binds this path's module — the schema name \
             cannot be derived; import the type from its crate"
        );
        return Some((None, false));
    }
    let scope = scope_mods_of(reached);
    let mut resolved: Vec<(ModPresence, Option<ImportSchema>)> = Vec::new();
    for import in imports {
        let mut spliced = import.prefix;
        spliced.push(import.ident);
        spliced.extend(tail.iter().cloned());
        resolved.push((
            import.presence,
            resolve_use_path(
                &spliced,
                leaf,
                chain,
                &scope,
                import.external_root,
                guard,
                false,
            ),
        ));
    }
    guard.remove(&key);
    let mut resolved = resolved.into_iter();
    let (first_presence, first) = resolved.next().expect("non-empty");
    let Some(mut schema) = first else {
        return Some((None, false));
    };
    let mut every_conditional = first_presence == ModPresence::Conditional;
    let mut any_conditional = every_conditional;
    for (presence, other) in resolved {
        let Some(other) = other else {
            return Some((None, false));
        };
        let conditional = presence == ModPresence::Conditional;
        every_conditional &= conditional;
        any_conditional |= conditional;
        if !any_conditional {
            schema = other;
            continue;
        }
        let ImportSchema {
            name,
            cfg_alternatives,
            ..
        } = other;
        record_alternative(
            &mut schema,
            CfgAlternative {
                name,
                shape: CfgShape::ExclusivePair,
            },
        );
        for alt in cfg_alternatives {
            record_alternative(&mut schema, alt);
        }
    }
    Some((Some(schema), every_conditional))
}

/// The body of an inline `mod name { … }` among `items` that CAN exist.
fn inline_module<'a>(items: &'a [syn::Item], name: &str) -> Option<&'a [syn::Item]> {
    items.iter().find_map(|item| match item {
        syn::Item::Mod(m) if m.ident == name && mod_presence(&m.attrs) != ModPresence::Never => {
            m.content.as_ref().map(|(_, content)| content.as_slice())
        }
        _ => None,
    })
}

/// How a module an ANCESTOR declares, reached from this scope through its
/// globs, is gated for the leaf that names it (the glob's
/// presence must not be dropped, or a module reached only through
/// `#[cfg] use super::*` reads as available in every build, and validation
/// accepts a label the cfg-off node does not publish).
enum AncestorGate {
    /// Reachable in every build the parser can see: an unconditional glob
    /// reaches an unconditional module, or a FARTHER reach also declares the
    /// module (with the near glob off the farther module is what the path
    /// names — assuming the reaches' cfgs cover every build, which the parser
    /// cannot evaluate; with all of them off the path names an extern crate).
    Open,
    /// Visible in one configuration only — `glob_gated` says whether the
    /// GLOB (`#[cfg] use super::*`) or the MODULE (`#[cfg] mod m`) carries the
    /// cfg. With it off the path resolves to an extern crate of the module's
    /// name — the same answer the no-reach arm gives a module no glob reaches,
    /// whatever other globs the scope holds (a glob the parser cannot read
    /// into is never assumed to bind a MODULE name; it is assumed to bind a
    /// LEAF only where `resolve_via_globs` says so — an asymmetry CHOSEN, not
    /// incidental: a wrong guess here costs a permissive extra alternative,
    /// never a refusal); `glob` is the reaching
    /// glob's path, the one to spell the module through instead
    /// (`use <glob>::<module>::<Leaf>;`).
    ViaGlob { glob: String, glob_gated: bool },
}

/// A module named `name` declared in an ANCESTOR scope this scope's DIRECT
/// globs reach (nearest reached ancestor first), with its body when inline
/// and how the reach is gated — the module's own presence INTERSECTED with
/// the glob's, because the glob is a `use` edge. Per reached depth the BEST
/// presence among the globs reaching it counts (an unconditional glob makes
/// the ancestor visible in every build; only conditional ones make it
/// conditional; a glob that can never exist is not collected at all —
/// `scope_mods_of`). Two conditional globs at one depth read as conditional
/// even when their cfgs are complementary: the parser evaluates no
/// predicate, so it cannot know they cover every build. A FARTHER reach that
/// also declares the module makes the name crate-internal in every build
/// (with the near glob off, the farther module is what the path names), so
/// no alternative is reported (assuming those reaches cover every build — the
/// parser evaluates no cfg predicate). Out-of-line modules
/// count: `mod schema_types;` in the parent is visible to a `use super::*`
/// exactly as an inline one.
fn ancestor_module_via_globs<'a>(
    name: &str,
    chain: &[&'a [syn::Item]],
    scope: &ScopeMods<'_>,
) -> Option<(usize, Option<&'a [syn::Item]>, AncestorGate)> {
    // (depth, the best presence among the globs reaching it, that glob's path)
    let mut reach: Vec<(usize, ModPresence, String)> = Vec::new();
    for glob in &scope.globs {
        // `scope_mods_of` never collects a glob that can never exist.
        let Some(depth) = glob_ancestor_depth(&glob.path, chain.len()) else {
            continue;
        };
        match reach.iter_mut().find(|(d, _, _)| *d == depth) {
            Some((_, presence, path)) => {
                if glob.presence == ModPresence::Always && *presence != ModPresence::Always {
                    *presence = ModPresence::Always;
                    *path = glob.path.clone();
                }
            }
            None => reach.push((depth, glob.presence, glob.path.clone())),
        }
    }
    reach.sort_unstable_by_key(|(depth, _, _)| std::cmp::Reverse(*depth));
    let module_in = |items: &'a [syn::Item]| {
        items.iter().find_map(|item| match item {
            syn::Item::Mod(m) if m.ident == name => {
                let presence = mod_presence(&m.attrs);
                (presence != ModPresence::Never).then(|| {
                    (
                        m.content.as_ref().map(|(_, content)| content.as_slice()),
                        presence,
                    )
                })
            }
            _ => None,
        })
    };
    let mut reaches = reach
        .into_iter()
        .filter_map(|(depth, glob_presence, glob_path)| {
            module_in(chain[depth])
                .map(|(body, presence)| (depth, body, presence, glob_presence, glob_path))
        });
    let (depth, body, module_presence, glob_presence, glob_path) = reaches.next()?;
    let elsewhere = reaches.next().is_some();
    let gate = match (glob_presence, module_presence) {
        (ModPresence::Conditional, _) if elsewhere => AncestorGate::Open,
        (ModPresence::Conditional, _) => AncestorGate::ViaGlob {
            glob: glob_path,
            glob_gated: true,
        },
        (_, ModPresence::Conditional) => AncestorGate::ViaGlob {
            glob: glob_path,
            glob_gated: false,
        },
        _ => AncestorGate::Open,
    };
    Some((depth, body, gate))
}

/// The schema name of a type imported from OUTSIDE the node's crate, by
/// its `use` path. For the `native_ros2_messages` anchor the crate prefix
/// is dropped so ROS 2 schemas appear as `pkg/Type` (and a type at that
/// crate's root — `use native_ros2_messages::Image;` — is just the leaf);
/// everything else keeps the full crate-qualified path, so a user-defined
/// schema cannot collide with a ROS 2 schema of the same leaf name.
/// `prefix` is never empty: both callers sit below the empty-prefix return
/// (a `use ::T;` has no external root to name).
fn external_schema_name(prefix: &[String], leaf: &str) -> String {
    debug_assert!(!prefix.is_empty(), "an external import has a crate root");
    let pkg_segments: Vec<&str> = if prefix[0] == "native_ros2_messages" {
        prefix[1..].iter().map(|s| s.as_str()).collect()
    } else {
        prefix.iter().map(|s| s.as_str()).collect()
    };
    if pkg_segments.is_empty() {
        leaf.to_string()
    } else {
        format!("{}/{}", pkg_segments.join("/"), leaf)
    }
}

/// Extract the bare type ident from a field type. Supports
/// simple paths like `Image` and `sensor_msgs::Image`. Returns
/// `None` for tuples, references, generics, function pointers,
/// etc.
/// Parse a Rust integer literal as printed by
/// [`proc_macro2::Literal`]'s `to_string` (its `Display`) into a `u64`.
///
/// Rust accepts visual underscores inside numeric literals
/// (`100_000`) and optional type suffixes (`100u64`, `0x1A_u32`,
/// etc.). `Literal::to_string()` preserves the source formatting
/// verbatim, so a naive `parse::<u64>()` rejects both shapes. This
/// helper:
///
/// 1. Strips a trailing type suffix (any non-digit tail starting
///    with a letter).
/// 2. Removes underscores.
/// 3. Detects hex / oct / bin prefixes and parses with the right
///    radix.
///
/// Returns `None` for negatives, floats, byte/char/string
/// literals, or anything else that doesn't fit a non-negative
/// integer.
fn parse_int_literal(raw: &str) -> Option<u64> {
    let s = raw.trim();
    // Reject anything that obviously isn't an integer literal.
    if s.is_empty() || s.starts_with('"') || s.starts_with('\'') || s.starts_with('b') {
        // Note: a Rust integer literal can technically begin with
        // `0b` (binary) — that's caught by the radix branch below;
        // a bare leading `b` indicates a byte/string literal.
        if !(s.starts_with("0b") || s.starts_with("0B")) {
            return None;
        }
    }
    if s.contains('.') {
        return None;
    }
    // Strip optional type suffix. Suffix starts at the first
    // letter that isn't part of the radix prefix.
    let (digits_with_underscores, _suffix) = match s.as_bytes() {
        [b'0', b'x' | b'X', ..] | [b'0', b'o' | b'O', ..] | [b'0', b'b' | b'B', ..] => {
            // Skip radix prefix (2 chars), then find first
            // non-hex/digit char as suffix start.
            let prefix = &s[..2];
            let rest = &s[2..];
            let suffix_start = rest.find(|c: char| !c.is_ascii_hexdigit() && c != '_');
            match suffix_start {
                Some(idx) => (format!("{}{}", prefix, &rest[..idx]), &rest[idx..]),
                None => (s.to_string(), ""),
            }
        }
        _ => {
            let suffix_start = s.find(|c: char| !c.is_ascii_digit() && c != '_');
            match suffix_start {
                Some(idx) => (s[..idx].to_string(), &s[idx..]),
                None => (s.to_string(), ""),
            }
        }
    };
    let no_underscores: String = digits_with_underscores
        .chars()
        .filter(|c| *c != '_')
        .collect();
    let (radix, digits) = match no_underscores.as_bytes() {
        [b'0', b'x' | b'X', ..] => (16u32, &no_underscores[2..]),
        [b'0', b'o' | b'O', ..] => (8u32, &no_underscores[2..]),
        [b'0', b'b' | b'B', ..] => (2u32, &no_underscores[2..]),
        _ => (10u32, no_underscores.as_str()),
    };
    if digits.is_empty() {
        return None;
    }
    u64::from_str_radix(digits, radix).ok()
}

fn field_type_ident(ty: &syn::Type) -> Option<String> {
    let syn::Type::Path(type_path) = ty else {
        return None;
    };
    type_path.path.segments.last().map(|s| s.ident.to_string())
}

/// Translate the macro attribute `#[cerulion_node(...)]` arg list
/// into a `MacroPolicy`. Returns `None` for the bare
/// `#[cerulion_node]` form (no parens) — the macro's only
/// remaining policy signal in that case is a `#[input(trigger)]`
/// field, which the caller threads in separately.
///
/// The token-level scan walks the attribute's args looking for
/// recognizable shapes:
///
/// - `period_ms = N`     → `Period { period_ms: N }`
/// - `sync_window_ms = N`→ `Sync { window_ms: N }`
/// - `external`          → `External`
///
/// Mixed args (e.g. `period_ms = 10, external`) are macro-time
/// errors; we mirror the macro's precedence (period > sync >
/// external) and pick the first match.
///
/// Uses `TokenTree` directly rather than `parse_nested_meta`
/// because the latter requires the callback to consume the value
/// of name-value args; incorrect handling makes `parse_nested_meta`
/// error out before reaching the following bare ident. Token-level
/// scan is simpler and tolerant of mixed shapes.
fn macro_attr_to_policy(attr: &syn::Attribute) -> Option<MacroPolicy> {
    let syn::Meta::List(meta_list) = &attr.meta else {
        return None;
    };
    use proc_macro2::TokenTree;
    let tokens: Vec<TokenTree> = meta_list.tokens.clone().into_iter().collect();
    let mut i = 0;
    while i < tokens.len() {
        if let TokenTree::Ident(ident) = &tokens[i] {
            let name = ident.to_string();
            // Look ahead for `= <literal>` to extract the value.
            // `proc_macro2::Literal::to_string()` preserves
            // source-level formatting — `100_000` returns
            // `"100_000"`, `100u64` returns `"100u64"`. Strip
            // underscores (which Rust accepts in numeric literals as
            // visual separators) and any trailing type suffix before
            // parsing.
            let value: Option<u64> = if i + 2 < tokens.len() {
                if let (TokenTree::Punct(p), TokenTree::Literal(lit)) =
                    (&tokens[i + 1], &tokens[i + 2])
                {
                    if p.as_char() == '=' {
                        parse_int_literal(&lit.to_string())
                    } else {
                        None
                    }
                } else {
                    None
                }
            } else {
                None
            };
            match name.as_str() {
                "period_ms" => {
                    if let Some(period_ms) = value {
                        return Some(MacroPolicy::Period { period_ms });
                    }
                }
                "sync_window_ms" => {
                    if let Some(window_ms) = value {
                        return Some(MacroPolicy::Sync { window_ms });
                    }
                }
                "unbounded_sync" => {
                    return Some(MacroPolicy::UnboundedSync);
                }
                "external" => {
                    return Some(MacroPolicy::External);
                }
                _ => {}
            }
        }
        i += 1;
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::fs;
    use tempfile::TempDir;

    fn write_node(tmp: &TempDir, node_type: &str, lib_rs: &str) -> std::path::PathBuf {
        let node_dir = tmp.path().join(node_type);
        let src_dir = node_dir.join("src");
        fs::create_dir_all(&src_dir).unwrap();
        fs::write(src_dir.join("lib.rs"), lib_rs).unwrap();
        node_dir
    }

    #[test]
    fn parse_macro_node_with_input_and_output() {
        let tmp = TempDir::new().unwrap();
        let src = r#"
use cerulion_core::prelude::*;
use native_ros2_messages::sensor_msgs::Image;

#[cerulion_node(period_ms = 100)]
#[derive(Default)]
struct CameraNode {
    #[input]
    raw_image: Image,
    #[output]
    processed: Image,
    tick_count: u32,
}
"#;
        let node_dir = write_node(&tmp, "camera", src);
        let metadata = parse_node_metadata(&node_dir).unwrap();
        assert_eq!(metadata.node_type, "camera");
        // The macro on this fixture carries `period_ms = 100`.
        assert_eq!(
            metadata.policy,
            Some(MacroPolicy::Period { period_ms: 100 })
        );
        assert_eq!(metadata.inputs.len(), 1);
        assert_eq!(metadata.inputs[0].name, "raw_image");
        assert_eq!(
            metadata.inputs[0].schema.as_deref(),
            Some("sensor_msgs/Image")
        );
        assert_eq!(metadata.outputs.len(), 1);
        assert_eq!(metadata.outputs[0].name, "processed");
        assert_eq!(
            metadata.outputs[0].schema.as_deref(),
            Some("sensor_msgs/Image")
        );
    }

    // ======================================================================
    // The declared backpressure policy is RECORDED, not discarded.
    //
    // `PortDef.backpressure` is what `source_entry_infos` feeds
    // the auto-partitioner, and the partitioner treats a `block` edge as a hard
    // co-location constraint — so a policy this parser drops is a partition
    // that splits a `block` edge and a worker that dies at graph build.
    // ======================================================================

    #[test]
    fn a_macro_nodes_declared_backpressure_lands_on_its_port() {
        let tmp = TempDir::new().unwrap();
        let src = r#"
use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Vector3;

#[cerulion_node(period_ms = 10)]
#[derive(Default)]
struct MixedNode {
    #[input(trigger, backpressure = block, depth = 4)]
    blocking: Vector3,
    #[input(backpressure = drop_oldest)]
    lossy: Vector3,
    #[input(backpressure = sample(50))]
    decimated: Vector3,
    #[input]
    plain: Vector3,
    #[output]
    out: Vector3,
}
"#;
        let node_dir = write_node(&tmp, "mixed", src);
        let metadata = parse_node_metadata(&node_dir).unwrap();
        // Hand oracle: every declared policy, in declaration order, plus the
        // macro's own default for the undeclared one.
        let got: Vec<(&str, BackpressurePolicy)> = metadata
            .inputs
            .iter()
            .map(|p| (p.name.as_str(), p.backpressure))
            .collect();
        assert_eq!(
            got,
            vec![
                ("blocking", BackpressurePolicy::Block),
                ("lossy", BackpressurePolicy::DropOldest),
                ("decimated", BackpressurePolicy::Sample(50)),
                ("plain", BackpressurePolicy::DropOldest),
            ]
        );
        // Backpressure is an INPUT-only concept — an output must never carry a
        // policy read off some neighbouring input's attr walk.
        assert_eq!(
            metadata.outputs[0].backpressure,
            BackpressurePolicy::DropOldest
        );
    }

    /// The macro parses `sample(N)`'s count as a `LitInt` and `base10_parse`s
    /// it to a `u64`, so anything else is macro-REJECTED. A parser that never
    /// looks at the group's CONTENTS lets `sample(abc)` false-green
    /// here while the macro refuses to compile it (the CLI/engine invariant).
    #[test]
    fn a_non_integer_sample_count_is_rejected_like_the_macro_rejects_it() {
        let tmp = TempDir::new().unwrap();
        let src = r#"
use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Vector3;

#[cerulion_node(period_ms = 10)]
#[derive(Default)]
struct BadNode {
    #[input(backpressure = sample(abc))]
    inp: Vector3,
}
"#;
        let node_dir = write_node(&tmp, "bad", src);
        let err = parse_node_metadata(&node_dir).unwrap_err();
        let msg = err.to_string();
        assert!(
            msg.contains("`sample` requires an integer count"),
            "message must name the condition; got: {msg}"
        );
    }

    /// The OTHER half of the CLI/engine invariant, which had no coverage at all: this
    /// parser must never REJECT surface the macro ACCEPTS.
    ///
    /// The macro reads the count as a `syn::LitInt` and `base10_parse::<u64>`s
    /// it, so underscores, integer suffixes and `0x`/`0b`/`0o` radix prefixes
    /// all compile. A `str::parse::<u64>` takes only bare ASCII decimal, so each
    /// of these hard-errored here — and that error `?`-propagates through
    /// `source_entry_infos` into the `graph run` default-partition preflight,
    /// `graph levels` and `graph partition`, i.e. a node whose source BUILDS
    /// could not be run.
    ///
    /// The oracle is the VALUE, not merely that parsing succeeded: a resolver
    /// that accepted the spelling and then recorded the wrong number (a radix
    /// read as decimal, a suffix taken as digits) would pass an `is_ok()` check
    /// while silently mis-declaring the decimation rate.
    #[test]
    fn every_integer_literal_spelling_the_macro_accepts_parses_to_its_value() {
        // (declared spelling, the u64 the macro's `base10_parse` yields)
        let cases: &[(&str, u64)] = &[
            ("1_000", 1000),
            ("50u64", 50),
            ("0x10", 16),
            ("0b1010", 10),
            ("0o17", 15),
        ];

        for (spelling, expected) in cases {
            let tmp = TempDir::new().unwrap();
            let src = format!(
                r#"
use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Vector3;

#[cerulion_node(period_ms = 10)]
#[derive(Default)]
struct SpellingNode {{
    #[input(backpressure = sample({spelling}))]
    decimated: Vector3,
}}
"#
            );
            let node_dir = write_node(&tmp, "spelling", &src);
            let metadata = parse_node_metadata(&node_dir)
                .unwrap_or_else(|e| panic!("`sample({spelling})` compiles; parser said: {e}"));
            assert_eq!(
                metadata
                    .inputs
                    .iter()
                    .map(|p| (p.name.as_str(), p.backpressure))
                    .collect::<Vec<_>>(),
                vec![("decimated", BackpressurePolicy::Sample(*expected))],
                "`sample({spelling})` must record {expected}"
            );
        }
    }

    /// The REJECT direction, widened past `sample(abc)`: every spelling here is
    /// one the macro's `LitInt` + `base10_parse::<u64>` refuses, so accepting
    /// any of them would be this parser claiming a declaration the node cannot
    /// actually compile.
    ///
    /// The out-of-range case is why the message does not stop at "requires an
    /// integer count" — that value IS an integer, and reporting the wrong
    /// condition sends an operator hunting a typo that is not there.
    #[test]
    fn every_count_the_macro_rejects_is_still_refused_loudly() {
        let spellings = [
            "abc",                     // not a literal at all
            "\"50\"",                  // a string literal
            "1.5",                     // a float literal
            "",                        // `sample()` — no count at all
            "99999999999999999999999", // an integer, but past `u64`
        ];

        for spelling in spellings {
            let tmp = TempDir::new().unwrap();
            let src = format!(
                r#"
use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Vector3;

#[cerulion_node(period_ms = 10)]
#[derive(Default)]
struct BadNode {{
    #[input(backpressure = sample({spelling}))]
    inp: Vector3,
}}
"#
            );
            let node_dir = write_node(&tmp, "bad", &src);
            let err = parse_node_metadata(&node_dir)
                .err()
                .unwrap_or_else(|| panic!("`sample({spelling})` is macro-invalid and must error"));
            let msg = err.to_string();
            assert!(
                msg.contains("`sample` requires an integer count"),
                "`sample({spelling})` must name the condition; got: {msg}"
            );
        }
    }

    /// The raw-FFI half. A hand-written `// CERULION:INFO_START`
    /// block declaring `block` is the SAME co-location constraint as a macro
    /// node's — before this the key was ignored and the whole raw-FFI class
    /// reported `drop_oldest`, silently missing co-location.
    ///
    /// Drives the exact ABI-v8 wire shape `cerulion_core::graph::node`'s
    /// loader parses: `"drop_oldest"` / `"block"` as plain strings and
    /// `{"sample":N}` as a single-key map.
    #[test]
    fn a_raw_ffi_info_blocks_backpressure_lands_on_its_port() {
        let tmp = TempDir::new().unwrap();
        let src = r#"
// CERULION:INFO_START
static INFO_BYTES: &[u8] = b"{\"inputs\":[{\"name\":\"blocking\",\"trigger\":true,\"depth\":4,\"backpressure\":\"block\"},{\"name\":\"lossy\",\"backpressure\":\"drop_oldest\"},{\"name\":\"decimated\",\"backpressure\":{\"sample\":25}},{\"name\":\"plain\"},\"bare\"],\"outputs\":[\"out\"]}\0";

#[no_mangle]
pub extern "C" fn cerulion_node_info() -> *const std::ffi::c_char {
    INFO_BYTES.as_ptr() as *const std::ffi::c_char
}
// CERULION:INFO_END
"#;
        let node_dir = write_node(&tmp, "rawffi", src);
        let metadata = parse_node_metadata(&node_dir).unwrap();
        let got: Vec<(&str, bool, BackpressurePolicy)> = metadata
            .inputs
            .iter()
            .map(|p| (p.name.as_str(), p.trigger, p.backpressure))
            .collect();
        assert_eq!(
            got,
            vec![
                ("blocking", true, BackpressurePolicy::Block),
                ("lossy", false, BackpressurePolicy::DropOldest),
                ("decimated", false, BackpressurePolicy::Sample(25)),
                ("plain", false, BackpressurePolicy::DropOldest),
                ("bare", false, BackpressurePolicy::DropOldest),
            ]
        );
        assert_eq!(
            metadata.outputs[0].backpressure,
            BackpressurePolicy::DropOldest
        );
    }

    /// CLI/engine invariant: the metadata parser must REJECT attr surface
    /// the macro rejects — otherwise `cerulion node info` would happily
    /// report metadata for source that cannot compile.
    #[test]
    fn parse_rejects_an_unknown_input_attr_the_same_way_the_macro_does() {
        let tmp = TempDir::new().unwrap();
        let src = r#"
use cerulion_core::prelude::*;

#[cerulion_node]
#[derive(Default)]
struct BadNode {
    #[input(trigger, newest_first, depth = 1)]
    data: u8,
}
"#;
        let node_dir = write_node(&tmp, "bad_input_attr", src);
        let err = parse_node_metadata(&node_dir).expect_err("an unknown attr must be rejected");
        let msg = err.to_string();
        assert!(
            msg.contains("unknown input attribute `newest_first`")
                && msg
                    .contains("Supported: `trigger`, `depth`, `backpressure`, `expect_within_ms`."),
            "must mirror the macro's diagnostic — offending ident + accepted set; got: {msg}"
        );
    }

    #[test]
    fn parse_rejects_an_output_field_list_the_same_way_the_macro_does() {
        let tmp = TempDir::new().unwrap();
        let src = r#"
use cerulion_core::prelude::*;
use native_ros2_messages::sensor_msgs::Image;

#[cerulion_node(period_ms = 16)]
#[derive(Default)]
struct BadNode {
    #[output(data, encoding)]
    image: Image,
}
"#;
        let node_dir = write_node(&tmp, "bad_list", src);
        let err = parse_node_metadata(&node_dir).expect_err("output list must be rejected");
        let msg = err.to_string();
        assert!(
            msg.contains("unknown output attribute `data`")
                && msg.contains("takes no field list")
                && msg.contains("bare form"),
            "must mirror the macro's diagnostic; got: {msg}"
        );
    }

    #[test]
    fn parse_accepts_surviving_input_and_output_surface() {
        // The COMPLETE accepted surface must keep parsing: trigger, depth,
        // backpressure (all three values), expect_within_ms; bare #[output]
        // and promise_within_ms.
        let tmp = TempDir::new().unwrap();
        let src = r#"
use cerulion_core::prelude::*;
use native_ros2_messages::sensor_msgs::Image;

#[cerulion_node]
#[derive(Default)]
struct GoodNode {
    #[input(trigger, depth = 4, backpressure = sample(50), expect_within_ms = 100)]
    raw: Image,
    #[input(backpressure = drop_oldest)]
    aux: Image,
    #[input(backpressure = block)]
    aux2: Image,
    #[output(promise_within_ms = 10)]
    processed: Image,
}
"#;
        let node_dir = write_node(&tmp, "good_surface", src);
        let metadata = parse_node_metadata(&node_dir).unwrap();
        assert_eq!(metadata.inputs.len(), 3);
        assert_eq!(metadata.outputs.len(), 1);
        assert_eq!(
            metadata.policy,
            Some(MacroPolicy::DataTrigger {
                input_name: "raw".to_string()
            })
        );
    }

    /// The backpressure VALUE idents are legal ONLY in the
    /// `backpressure = <value>` position. A flat whitelist false-greened
    /// `#[input(block)]` — source the macro rejects.
    #[test]
    fn parse_rejects_bare_backpressure_value_ident_at_top_level() {
        let tmp = TempDir::new().unwrap();
        for bad in ["block", "drop_oldest", "sample"] {
            let src = format!(
                r#"
use cerulion_core::prelude::*;

#[cerulion_node]
#[derive(Default)]
struct BadNode {{
    #[input(trigger, {bad})]
    data: u8,
}}
"#
            );
            let node_dir = write_node(&tmp, &format!("bad_bare_{bad}"), &src);
            let err = parse_node_metadata(&node_dir)
                .expect_err("bare backpressure value ident must be rejected");
            let msg = err.to_string();
            assert!(
                msg.contains(&format!("unknown input attribute `{bad}`")),
                "bare `{bad}` must hit the unknown-attribute error; got: {msg}"
            );
        }
    }

    /// `backpressure` with no `= value` is macro-invalid;
    /// the CLI mirrors it (both mid-list and list-final positions).
    #[test]
    fn parse_rejects_backpressure_without_value() {
        let tmp = TempDir::new().unwrap();
        for (name, attr) in [
            ("bp_bare_final", "#[input(trigger, backpressure)]"),
            ("bp_bare_mid", "#[input(backpressure, trigger)]"),
        ] {
            let src = format!(
                r#"
use cerulion_core::prelude::*;

#[cerulion_node]
#[derive(Default)]
struct BadNode {{
    {attr}
    data: u8,
}}
"#
            );
            let node_dir = write_node(&tmp, name, &src);
            let err = parse_node_metadata(&node_dir)
                .expect_err("valueless backpressure must be rejected");
            let msg = err.to_string();
            assert!(
                msg.contains("`backpressure` requires a value"),
                "got: {msg}"
            );
        }
    }

    /// An unknown ident in the VALUE position gets the
    /// policy-specific error (not the generic unknown-attribute one).
    #[test]
    fn parse_rejects_unknown_backpressure_policy_value() {
        let tmp = TempDir::new().unwrap();
        let src = r#"
use cerulion_core::prelude::*;

#[cerulion_node]
#[derive(Default)]
struct BadNode {
    #[input(trigger, backpressure = newest_wins)]
    data: u8,
}
"#;
        let node_dir = write_node(&tmp, "bad_bp_value", src);
        let err = parse_node_metadata(&node_dir)
            .expect_err("unknown backpressure policy must be rejected");
        let msg = err.to_string();
        assert!(
            msg.contains("unknown backpressure policy `newest_wins`")
                && msg.contains("`sample(N)`"),
            "got: {msg}"
        );
    }

    /// `sample` must carry its parenthesized count
    /// (the macro requires `sample(N)`); bare `sample` in the value
    /// position — mid-list AND list-final — is rejected, with no
    /// misclassification of a following attr.
    #[test]
    fn parse_rejects_sample_without_count() {
        let tmp = TempDir::new().unwrap();
        for (name, attr) in [
            (
                "bp_sample_final",
                "#[input(trigger, backpressure = sample)]",
            ),
            ("bp_sample_mid", "#[input(backpressure = sample, trigger)]"),
        ] {
            let src = format!(
                r#"
use cerulion_core::prelude::*;

#[cerulion_node]
#[derive(Default)]
struct BadNode {{
    {attr}
    data: u8,
}}
"#
            );
            let node_dir = write_node(&tmp, name, &src);
            let err = parse_node_metadata(&node_dir).expect_err("bare sample must be rejected");
            let msg = err.to_string();
            assert!(
                msg.contains("`sample` requires a parenthesized count"),
                "got: {msg}"
            );
        }
    }

    /// A NON-IDENT in the value position
    /// (`backpressure = 5`) errors immediately with the policy message —
    /// and a following `trigger` is NOT misread as the value (the error
    /// names the literal, not the next ident).
    #[test]
    fn parse_rejects_non_ident_backpressure_value_without_misclassifying() {
        let tmp = TempDir::new().unwrap();
        let src = r#"
use cerulion_core::prelude::*;

#[cerulion_node]
#[derive(Default)]
struct BadNode {
    #[input(backpressure = 5, trigger)]
    data: u8,
}
"#;
        let node_dir = write_node(&tmp, "bad_bp_literal", src);
        let err = parse_node_metadata(&node_dir)
            .expect_err("literal backpressure value must be rejected");
        let msg = err.to_string();
        assert!(
            msg.contains("unknown backpressure policy `5`"),
            "must name the literal, not a later ident; got: {msg}"
        );
    }

    /// A bare `promise_within_ms` (no `= N`) is
    /// macro-rejected and must not pass the CLI validator — mid-list,
    /// list-final, and non-literal-value forms all error.
    #[test]
    fn parse_rejects_bare_promise_within_ms() {
        let tmp = TempDir::new().unwrap();
        for (name, attr) in [
            ("promise_bare", "#[output(promise_within_ms)]"),
            ("promise_no_value", "#[output(promise_within_ms = )]"),
            ("promise_ident_value", "#[output(promise_within_ms = fast)]"),
        ] {
            let src = format!(
                r#"
use cerulion_core::prelude::*;
use native_ros2_messages::sensor_msgs::Image;

#[cerulion_node(period_ms = 16)]
#[derive(Default)]
struct BadNode {{
    {attr}
    image: Image,
}}
"#
            );
            let node_dir = write_node(&tmp, name, &src);
            let err = parse_node_metadata(&node_dir)
                .expect_err("valueless promise_within_ms must be rejected");
            let msg = err.to_string();
            assert!(
                msg.contains("`promise_within_ms` requires a value"),
                "case {name}: got: {msg}"
            );
        }
    }

    /// Macro parity: duplicate `promise_within_ms` is rejected by
    /// `parse_output_attr` — the CLI mirrors it.
    #[test]
    fn parse_rejects_duplicate_promise_within_ms() {
        let tmp = TempDir::new().unwrap();
        let src = r#"
use cerulion_core::prelude::*;
use native_ros2_messages::sensor_msgs::Image;

#[cerulion_node(period_ms = 16)]
#[derive(Default)]
struct BadNode {
    #[output(promise_within_ms = 5, promise_within_ms = 6)]
    image: Image,
}
"#;
        let node_dir = write_node(&tmp, "promise_dup", src);
        let err = parse_node_metadata(&node_dir)
            .expect_err("duplicate promise_within_ms must be rejected");
        assert!(err.to_string().contains("duplicate `promise_within_ms`"));
    }

    #[test]
    fn parse_macro_node_external_flag() {
        let tmp = TempDir::new().unwrap();
        let src = r#"
use cerulion_core::prelude::*;

#[cerulion_node(external)]
#[derive(Default)]
struct ButtonNode {
    #[output]
    pressed: u8,
}
"#;
        let node_dir = write_node(&tmp, "button", src);
        let metadata = parse_node_metadata(&node_dir).unwrap();
        assert_eq!(
            metadata.policy,
            Some(MacroPolicy::External),
            "`#[cerulion_node(external)]` must set policy = External"
        );
    }

    #[test]
    fn parse_macro_node_period_attr() {
        let tmp = TempDir::new().unwrap();
        let src = r#"
use cerulion_core::prelude::*;

#[cerulion_node(period_ms = 33)]
#[derive(Default)]
struct CamNode {
    #[output]
    image: u8,
}
"#;
        let node_dir = write_node(&tmp, "cam", src);
        let metadata = parse_node_metadata(&node_dir).unwrap();
        assert_eq!(
            metadata.policy,
            Some(MacroPolicy::Period { period_ms: 33 }),
            "`#[cerulion_node(period_ms = N)]` must set policy = Period {{ period_ms: N }}"
        );
    }

    #[test]
    fn parse_macro_node_period_attr_with_underscore_literal() {
        // `proc_macro2::Literal::to_string()` preserves the
        // underscore separator, so `100_000` arrives at our parser
        // as the string "100_000". Naive `parse::<u64>()` rejects
        // it; `parse_int_literal` strips the underscores.
        let tmp = TempDir::new().unwrap();
        let src = r#"
use cerulion_core::prelude::*;

#[cerulion_node(period_ms = 100_000)]
#[derive(Default)]
struct CamNode {
    #[output]
    image: u8,
}
"#;
        let node_dir = write_node(&tmp, "cam", src);
        let metadata = parse_node_metadata(&node_dir).unwrap();
        assert_eq!(
            metadata.policy,
            Some(MacroPolicy::Period { period_ms: 100_000 }),
            "underscore-separated literals must round-trip"
        );
    }

    #[test]
    fn parse_macro_node_period_attr_with_type_suffix() {
        // `Literal::to_string()` also preserves type suffixes
        // (`100u64`). `parse_int_literal` strips them.
        let tmp = TempDir::new().unwrap();
        let src = r#"
use cerulion_core::prelude::*;

#[cerulion_node(period_ms = 50u64)]
#[derive(Default)]
struct CamNode {
    #[output]
    image: u8,
}
"#;
        let node_dir = write_node(&tmp, "cam", src);
        let metadata = parse_node_metadata(&node_dir).unwrap();
        assert_eq!(
            metadata.policy,
            Some(MacroPolicy::Period { period_ms: 50 }),
            "type-suffixed literals must round-trip"
        );
    }

    // Direct unit tests for `parse_int_literal` — exercise the
    // shapes `proc_macro2::Literal::to_string()` actually produces.

    #[test]
    fn parse_int_literal_plain_decimal() {
        assert_eq!(super::parse_int_literal("100"), Some(100));
        assert_eq!(super::parse_int_literal("0"), Some(0));
    }

    #[test]
    fn parse_int_literal_underscores() {
        assert_eq!(super::parse_int_literal("100_000"), Some(100_000));
        assert_eq!(super::parse_int_literal("1_000_000"), Some(1_000_000));
    }

    #[test]
    fn parse_int_literal_type_suffixes() {
        assert_eq!(super::parse_int_literal("100u64"), Some(100));
        assert_eq!(super::parse_int_literal("50_000u32"), Some(50_000));
        assert_eq!(super::parse_int_literal("42usize"), Some(42));
    }

    #[test]
    fn parse_int_literal_radix_prefixes() {
        assert_eq!(super::parse_int_literal("0x1A"), Some(0x1A));
        assert_eq!(super::parse_int_literal("0x1A_u32"), Some(0x1A));
        assert_eq!(super::parse_int_literal("0b1010"), Some(0b1010));
        assert_eq!(super::parse_int_literal("0o777"), Some(0o777));
    }

    #[test]
    fn parse_int_literal_rejects_non_integers() {
        assert_eq!(super::parse_int_literal("1.5"), None);
        assert_eq!(super::parse_int_literal("\"100\""), None);
        assert_eq!(super::parse_int_literal("'x'"), None);
        assert_eq!(super::parse_int_literal(""), None);
        assert_eq!(super::parse_int_literal("0x"), None);
    }

    #[test]
    fn parse_macro_node_data_trigger_from_field_attr() {
        // Single `#[input(trigger)]` field with no node-level attr →
        // `policy` synthesizes to `DataTrigger { input_name }`.
        let tmp = TempDir::new().unwrap();
        let src = r#"
use cerulion_core::prelude::*;

#[cerulion_node]
#[derive(Default)]
struct ConsumerNode {
    #[input(trigger)]
    count: u32,
}
"#;
        let node_dir = write_node(&tmp, "consumer", src);
        let metadata = parse_node_metadata(&node_dir).unwrap();
        assert_eq!(
            metadata.policy,
            Some(MacroPolicy::DataTrigger {
                input_name: "count".to_string(),
            }),
            "single `#[input(trigger)]` must set policy = DataTrigger"
        );
    }

    #[test]
    fn parse_macro_node_input_with_trigger_attr() {
        let tmp = TempDir::new().unwrap();
        let src = r#"
use cerulion_core::prelude::*;
use native_ros2_messages::sensor_msgs::Image;

#[cerulion_node]
#[derive(Default)]
struct DetectorNode {
    #[input(trigger)]
    image: Image,
}
"#;
        let node_dir = write_node(&tmp, "detector", src);
        let metadata = parse_node_metadata(&node_dir).unwrap();
        assert_eq!(metadata.inputs.len(), 1);
        assert_eq!(metadata.inputs[0].name, "image");
        assert_eq!(
            metadata.inputs[0].schema.as_deref(),
            Some("sensor_msgs/Image")
        );
        // The `#[input(trigger)]` mark lands on `PortDef.trigger`.
        assert!(
            metadata.inputs[0].trigger,
            "an `#[input(trigger)]` field must parse trigger = true"
        );
    }

    #[test]
    fn parse_macro_node_input_trigger_flag_per_input() {
        // `PortDef.trigger` mirrors the `#[input(trigger)]` mark
        // per-input. A sync node with TWO trigger inputs and one plain
        // latest-value input — the trigger-scoped sync shape — must parse
        // trigger=true on the marked inputs and false on the plain one
        // (hand oracle, order preserved). The node-level `sync_window_ms`
        // owns the policy, so trigger flags are independent of it.
        let tmp = TempDir::new().unwrap();
        let src = r#"
use cerulion_core::prelude::*;

#[cerulion_node(sync_window_ms = 10)]
#[derive(Default)]
struct FusionNode {
    #[input(trigger)]
    cam: u32,
    #[input(trigger)]
    imu: u32,
    #[input]
    calib: u32,
    #[output]
    fused: u32,
}
"#;
        let node_dir = write_node(&tmp, "fusion", src);
        let metadata = parse_node_metadata(&node_dir).unwrap();
        assert_eq!(metadata.policy, Some(MacroPolicy::Sync { window_ms: 10 }));
        assert_eq!(metadata.inputs.len(), 3);
        assert_eq!(metadata.inputs[0].name, "cam");
        assert!(metadata.inputs[0].trigger, "cam is `#[input(trigger)]`");
        assert_eq!(metadata.inputs[1].name, "imu");
        assert!(metadata.inputs[1].trigger, "imu is `#[input(trigger)]`");
        assert_eq!(metadata.inputs[2].name, "calib");
        assert!(
            !metadata.inputs[2].trigger,
            "calib is a plain `#[input]` — not a trigger"
        );
        // Outputs are never triggers.
        assert_eq!(metadata.outputs.len(), 1);
        assert!(!metadata.outputs[0].trigger, "outputs are never triggers");
    }

    #[test]
    fn parse_macro_node_unqualified_field_type() {
        // A field type with no `use native_ros2_messages::...`
        // anchor produces `schema: None`.
        let tmp = TempDir::new().unwrap();
        let src = r#"
use cerulion_core::prelude::*;

#[cerulion_node(period_ms = 100)]
#[derive(Default)]
struct CounterNode {
    #[output]
    count: u32,
}
"#;
        let node_dir = write_node(&tmp, "counter", src);
        let metadata = parse_node_metadata(&node_dir).unwrap();
        assert_eq!(metadata.outputs[0].name, "count");
        assert_eq!(metadata.outputs[0].schema, None);
    }

    #[test]
    fn parse_raw_ffi_node_via_info_markers() {
        let tmp = TempDir::new().unwrap();
        let src = r#"
use cerulion_core::prelude::*;

struct CameraState { tick_count: u64 }

// CERULION:INFO_START
static INFO_BYTES: &[u8] = b"{\"inputs\":[\"raw\"],\"outputs\":[\"processed\",\"meta\"]}\0";

#[no_mangle]
pub extern "C" fn cerulion_node_info() -> *const std::ffi::c_char {
    INFO_BYTES.as_ptr() as *const std::ffi::c_char
}
// CERULION:INFO_END
"#;
        let node_dir = write_node(&tmp, "raw_camera", src);
        let metadata = parse_node_metadata(&node_dir).unwrap();
        assert_eq!(metadata.node_type, "raw_camera");
        assert!(
            metadata.policy.is_none(),
            "raw-FFI without `policy:` JSON must report None"
        );
        assert_eq!(metadata.inputs.len(), 1);
        assert_eq!(metadata.inputs[0].name, "raw");
        assert_eq!(metadata.inputs[0].schema, None);
        // A legacy bare-string raw-FFI input carries no trigger mark
        // (its trigger membership rides `policy`) → trigger = false.
        assert!(!metadata.inputs[0].trigger);
        assert_eq!(metadata.outputs.len(), 2);
        assert_eq!(metadata.outputs[0].name, "processed");
        assert_eq!(metadata.outputs[1].name, "meta");
    }

    #[test]
    fn parse_raw_ffi_node_object_input_carries_trigger() {
        // A raw-FFI `inputs` entry may be an OBJECT carrying
        // `"trigger":true` (a hand-written info block, or a future template).
        // The tolerant untagged parse reads it; a plain object (no key) and a
        // bare string both default to false. Hand oracle over the three forms.
        let tmp = TempDir::new().unwrap();
        let src = r#"
// CERULION:INFO_START
static INFO_BYTES: &[u8] = b"{\"inputs\":[{\"name\":\"fire\",\"trigger\":true},{\"name\":\"ctx\"},\"legacy\"],\"outputs\":[]}\0";

#[no_mangle]
pub extern "C" fn cerulion_node_info() -> *const std::ffi::c_char {
    INFO_BYTES.as_ptr() as *const std::ffi::c_char
}
// CERULION:INFO_END
"#;
        let node_dir = write_node(&tmp, "raw_trigger", src);
        let metadata = parse_node_metadata(&node_dir).unwrap();
        assert_eq!(metadata.inputs.len(), 3);
        assert_eq!(metadata.inputs[0].name, "fire");
        assert!(
            metadata.inputs[0].trigger,
            "object input with `trigger:true` must parse trigger = true"
        );
        assert_eq!(metadata.inputs[1].name, "ctx");
        assert!(
            !metadata.inputs[1].trigger,
            "object input with no trigger key defaults to false"
        );
        assert_eq!(metadata.inputs[2].name, "legacy");
        assert!(
            !metadata.inputs[2].trigger,
            "bare-string input defaults to false"
        );
    }

    #[test]
    fn parse_raw_ffi_node_empty_arrays() {
        let tmp = TempDir::new().unwrap();
        let src = r#"
// CERULION:INFO_START
static INFO_BYTES: &[u8] = b"{\"inputs\":[],\"outputs\":[]}\0";

#[no_mangle]
pub extern "C" fn cerulion_node_info() -> *const std::ffi::c_char {
    INFO_BYTES.as_ptr() as *const std::ffi::c_char
}
// CERULION:INFO_END
"#;
        let node_dir = write_node(&tmp, "empty_raw", src);
        let metadata = parse_node_metadata(&node_dir).unwrap();
        assert_eq!(metadata.inputs.len(), 0);
        assert_eq!(metadata.outputs.len(), 0);
    }

    #[test]
    fn parse_node_metadata_missing_lib_rs_returns_node_not_found() {
        let tmp = TempDir::new().unwrap();
        let node_dir = tmp.path().join("ghost");
        fs::create_dir_all(&node_dir).unwrap();
        let err = parse_node_metadata(&node_dir).unwrap_err();
        assert!(
            matches!(err, CliError::NodeNotFound { .. }),
            "missing lib.rs must return NodeNotFound, got: {:?}",
            err
        );
    }

    #[test]
    fn parse_node_metadata_unparseable_source_returns_validation_error() {
        let tmp = TempDir::new().unwrap();
        let src = "fn foo( {"; // syntactically broken
        let node_dir = write_node(&tmp, "broken", src);
        let err = parse_node_metadata(&node_dir).unwrap_err();
        assert!(
            matches!(err, CliError::Validation(_)),
            "unparseable source must return Validation error"
        );
    }

    #[test]
    fn parse_node_metadata_no_recognisable_shape_errors() {
        // No macro struct, no INFO markers — neither path
        // recognises the source, so we emit a clear Validation
        // error so the user knows what's missing.
        let tmp = TempDir::new().unwrap();
        let src = "fn main() {}\n";
        let node_dir = write_node(&tmp, "blank", src);
        let err = parse_node_metadata(&node_dir).unwrap_err();
        assert!(matches!(err, CliError::Validation(_)));
    }

    #[test]
    fn parse_macro_node_module_nested() {
        // Module-nested macro nodes are detected by
        // `is_macro_based`'s recursion through `Item::Mod`, and so
        // should be detected here too.
        let tmp = TempDir::new().unwrap();
        let src = r#"
use cerulion_core::prelude::*;
use native_ros2_messages::std_msgs::String;

mod inner {
    use super::*;
    #[cerulion_node(period_ms = 50)]
    #[derive(Default)]
    pub struct WrappedNode {
        #[output]
        msg: String,
    }
}
"#;
        let node_dir = write_node(&tmp, "wrapped", src);
        let metadata = parse_node_metadata(&node_dir).unwrap();
        assert_eq!(metadata.outputs.len(), 1);
        assert_eq!(metadata.outputs[0].name, "msg");
        // The schema MUST round-trip even though the `use` lives
        // outside the `mod inner` block. Imports at file root
        // resolve even under a shallow walk; this
        // assertion is the canary for `build_import_map` going
        // shallow.
        assert_eq!(
            metadata.outputs[0].schema.as_deref(),
            Some("std_msgs/String"),
            "module-nested node must recover its file-root import"
        );
    }

    #[test]
    fn parse_macro_node_module_nested_imports_inside_module() {
        // Adversarial: both the `#[cerulion_node]` struct AND its
        // `use` statements live INSIDE the module. `build_import_map`
        // must descend into `Item::Mod` to find them, otherwise the
        // schema collapses silently to `None`.
        let tmp = TempDir::new().unwrap();
        let src = r#"
use cerulion_core::prelude::*;

mod inner {
    use super::*;
    use native_ros2_messages::sensor_msgs::Image;

    #[cerulion_node(period_ms = 50)]
    #[derive(Default)]
    pub struct CameraNode {
        #[output]
        image: Image,
    }
}
"#;
        let node_dir = write_node(&tmp, "camera_nested", src);
        let metadata = parse_node_metadata(&node_dir).unwrap();
        assert_eq!(metadata.outputs.len(), 1);
        assert_eq!(metadata.outputs[0].name, "image");
        assert_eq!(
            metadata.outputs[0].schema.as_deref(),
            Some("sensor_msgs/Image"),
            "imports inside the same module as the macro struct must be found"
        );
    }

    #[test]
    fn parse_macro_node_user_defined_schema_distinct_from_ros2() {
        // Two nodes, each with a field type named `Image` but from
        // different crates. The schema names must be distinct so
        // graph_validate can cross-check correctly. An
        // anchor-only walker returns `schema: None` for the
        // user-defined case, which silently loses the type info.
        let tmp = TempDir::new().unwrap();

        // Node A: ROS2 schema.
        let ros_src = r#"
use cerulion_core::prelude::*;
use native_ros2_messages::sensor_msgs::Image;

#[cerulion_node(period_ms = 100)]
#[derive(Default)]
struct CameraNode {
    #[output]
    img: Image,
}
"#;
        let node_dir_a = write_node(&tmp, "ros_camera", ros_src);
        let metadata_a = parse_node_metadata(&node_dir_a).unwrap();
        assert_eq!(
            metadata_a.outputs[0].schema.as_deref(),
            Some("sensor_msgs/Image"),
            "ROS2 schema must use `sensor_msgs/Image` (anchor stripped)"
        );

        // Node B: user-defined schema with the same leaf type
        // name but a different crate.
        let user_src = r#"
use cerulion_core::prelude::*;
use user_msgs::Image;

#[cerulion_node(period_ms = 100)]
#[derive(Default)]
struct UserCameraNode {
    #[output]
    img: Image,
}
"#;
        let node_dir_b = write_node(&tmp, "user_camera", user_src);
        let metadata_b = parse_node_metadata(&node_dir_b).unwrap();
        assert_eq!(
            metadata_b.outputs[0].schema.as_deref(),
            Some("user_msgs/Image"),
            "user-defined schema must use `user_msgs/Image` (full crate path retained), \
             not `sensor_msgs/Image` (ROS2 collision) and not `None` (anchor-only walker behaviour)"
        );

        // The two are distinct schemas at the framework level.
        assert_ne!(metadata_a.outputs[0].schema, metadata_b.outputs[0].schema);
    }

    #[test]
    fn parse_macro_node_user_schema_with_subpkg_path() {
        // `use my_workspace::messages::Detection;` should produce
        // `my_workspace/messages/Detection` — full crate-qualified
        // path retained for non-ROS2 imports.
        let tmp = TempDir::new().unwrap();
        let src = r#"
use cerulion_core::prelude::*;
use my_workspace::messages::Detection;

#[cerulion_node(period_ms = 50)]
#[derive(Default)]
struct DetectorNode {
    #[output]
    out: Detection,
}
"#;
        let node_dir = write_node(&tmp, "detector", src);
        let metadata = parse_node_metadata(&node_dir).unwrap();
        assert_eq!(
            metadata.outputs[0].schema.as_deref(),
            Some("my_workspace/messages/Detection"),
        );
    }

    #[test]
    fn helpers_round_trip() {
        let mut metadata = NodeMetadata::new("camera", None);
        assert_eq!(metadata.node_type, "camera");
        assert!(metadata.policy.is_none());
        metadata.inputs.push(PortDef {
            name: "in".to_string(),
            schema: None,
            schema_alternatives: Vec::new(),
            trigger: false,
            backpressure: BackpressurePolicy::DropOldest,
        });
        metadata.outputs.push(PortDef {
            name: "out".to_string(),
            schema: Some("sensor_msgs/Image".to_string()),
            schema_alternatives: Vec::new(),
            trigger: false,
            backpressure: BackpressurePolicy::DropOldest,
        });
        assert!(metadata.has_input("in"));
        assert!(!metadata.has_input("out"));
        assert!(metadata.has_output("out"));
        assert!(!metadata.has_output("in"));
    }
    /// A port type that comes from a module of the node's OWN
    /// crate is reported by its BARE name — the schema namespace's name for
    /// a workspace schema — never qualified with the Rust module it happens
    /// to live in.
    ///
    /// This is the SHIPPED shape of a workspace schema and the only one it
    /// can take: `schemas/<file>.yaml` becomes a Rust type by way of the
    /// node's `build.rs`, which writes it into `OUT_DIR` for the node to
    /// `include!` into a private module. `examples/perception` does exactly
    /// this, and the fabricated `detections/DetectionArray` a naive walk
    /// produces is what `graph validate` would report as the node's own
    /// declaration — refusing both demo graphs, whose BARE
    /// `schema: DetectionArray` is the correct spelling (`docs/user-api.md` teaches
    /// that spelling, and `schema_cmd::resolve_port_schema`'s `Workspace`
    /// arm returns it unchanged).
    ///
    /// The oracle is written out per arm rather than derived, and the
    /// EXTERNAL-crate arm is the anti-tautology half: without it, "report
    /// the bare leaf" is satisfied by deleting the qualification entirely,
    /// which would lose the `.msg`-store package a vendor crate really does
    /// name (`examples/go2`'s `unitree_go/Go2FrontVideoData`).
    #[test]
    fn a_type_from_a_local_module_is_reported_bare_not_module_qualified() {
        let tmp = TempDir::new().unwrap();

        // The example's exact shape: an inline `mod` holding the codegen'd
        // type, plus a `use` rooted at it.
        let local_mod_src = r#"
use cerulion_core::prelude::*;
use native_ros2_messages::sensor_msgs::Image;

mod detections {
    include!(concat!(env!("OUT_DIR"), "/detections_schema.rs"));
}
use detections::DetectionArray;

#[cerulion_node]
#[derive(Default)]
struct DetectorNode {
    #[input(trigger)]
    image_raw: Image,
    #[output]
    detections: DetectionArray,
}
"#;
        let node_dir = write_node(&tmp, "detector", local_mod_src);
        let metadata = parse_node_metadata(&node_dir).unwrap();
        assert_eq!(
            metadata.outputs[0].schema.as_deref(),
            Some("DetectionArray"),
            "a type from a private module of this crate must be reported by its bare \
             schema name, not qualified with the module (`detections/DetectionArray` \
             resolves to no schema in any workspace)"
        );
        // The sibling ROS 2 port is untouched — the bare-name rule is scoped to the
        // crate-internal root, not to "everything but the anchor".
        assert_eq!(
            metadata.inputs[0].schema.as_deref(),
            Some("sensor_msgs/Image"),
            "the built-in anchor must still qualify"
        );

        // ANTI-TAUTOLOGY: an EXTERNAL message crate is its own schema
        // package (`schemas/<pkg>/msg/<Type>.msg`) and keeps the
        // qualification. Same file, same `use` SHAPE — the only difference
        // is that nothing declares `mod unitree_go` here.
        let external_crate_src = r#"
use cerulion_core::prelude::*;
use unitree_go::Go2FrontVideoData;

#[cerulion_node]
#[derive(Default)]
struct TranscodeNode {
    #[input(trigger)]
    h264: Go2FrontVideoData,
}
"#;
        let node_dir = write_node(&tmp, "camera_jpeg", external_crate_src);
        let metadata = parse_node_metadata(&node_dir).unwrap();
        assert_eq!(
            metadata.inputs[0].schema.as_deref(),
            Some("unitree_go/Go2FrontVideoData"),
            "an external message crate names a real schema package and must stay \
             qualified — collapsing it would lose what the `.msg` store spells"
        );

        // A module NESTED inside another module does NOT shadow a crate
        // name — a keyword-less `use` path resolves its first segment
        // against external crates or a module declared in the SAME scope as
        // the `use` (the rule `collect_scope_imports` implements; this
        // import sits at the crate root, whose scope has no `mod
        // unitree_go`). So an unrelated inner `mod unitree_go` must not
        // reclassify the top-level import and strip a real package
        // qualifier.
        let nested_shadow_src = r#"
use cerulion_core::prelude::*;
use unitree_go::Go2FrontVideoData;

mod capture {
    pub mod unitree_go {
        pub struct Unrelated;
    }
}

#[cerulion_node]
#[derive(Default)]
struct TranscodeNode {
    #[input(trigger)]
    h264: Go2FrontVideoData,
}
"#;
        let node_dir = write_node(&tmp, "nested_shadow", nested_shadow_src);
        let metadata = parse_node_metadata(&node_dir).unwrap();
        assert_eq!(
            metadata.inputs[0].schema.as_deref(),
            Some("unitree_go/Go2FrontVideoData"),
            "a module nested inside another module cannot shadow a crate name, so the \
             top-level import must keep its package qualifier"
        );

        // A node whose whole struct lives INSIDE a module, importing a
        // sibling schema module by a bare path. 2018 uniform paths resolve
        // that against the enclosing module's own children, so this is
        // valid Rust and must reach the same bare answer — a crate-root-only
        // module set would qualify the name with the module one scope down.
        let module_nested_src = r#"
mod inner {
    use cerulion_core::prelude::*;
    use native_ros2_messages::sensor_msgs::Image;

    mod detections {
        include!(concat!(env!("OUT_DIR"), "/detections_schema.rs"));
    }
    use detections::DetectionArray;

    #[cerulion_node]
    #[derive(Default)]
    struct DetectorNode {
        #[input(trigger)]
        image_raw: Image,
        #[output]
        detections: DetectionArray,
    }
}
"#;
        let node_dir = write_node(&tmp, "module_nested", module_nested_src);
        let metadata = parse_node_metadata(&node_dir).unwrap();
        assert_eq!(
            metadata.outputs[0].schema.as_deref(),
            Some("DetectionArray"),
            "a node nested in a module must resolve its sibling schema module the same \
             way — uniform paths make the bare import valid there"
        );
        assert_eq!(
            metadata.inputs[0].schema.as_deref(),
            Some("sensor_msgs/Image"),
            "the built-in anchor must still qualify inside a nested scope"
        );

        // An OUT-OF-LINE `mod` is the same claim: the declaration, not the
        // brace, is what makes the root crate-internal.
        let out_of_line_src = r#"
use cerulion_core::prelude::*;

mod schema_types;
use schema_types::TrackArray;

#[cerulion_node(period_ms = 10)]
#[derive(Default)]
struct TrackerNode {
    #[output]
    tracks: TrackArray,
}
"#;
        let node_dir = write_node(&tmp, "tracker", out_of_line_src);
        let metadata = parse_node_metadata(&node_dir).unwrap();
        assert_eq!(
            metadata.outputs[0].schema.as_deref(),
            Some("TrackArray"),
            "an out-of-line `mod` is crate-internal too"
        );
    }

    /// A LEADING `::` names the external prelude, so a same-scope
    /// module of the same name cannot capture it.
    ///
    /// `mod serde { … } use ::serde::Serialize;` compiles and reaches the
    /// CRATE — measured against rustc — so the same-scope rule must not
    /// steal such a path and strip its package qualifier. The colon-free
    /// twin is the control: it IS captured, which is what makes the pair
    /// prove the colon is what decides rather than some other difference.
    #[test]
    fn a_leading_colon_import_is_always_external() {
        let tmp = TempDir::new().unwrap();
        for (leading, want, why) in [
            (
                "::",
                "unitree_go/Go2FrontVideoData",
                "a leading `::` is the external prelude",
            ),
            (
                "",
                "Go2FrontVideoData",
                "without it the same-scope module captures the path",
            ),
        ] {
            let src = format!(
                r#"
use cerulion_core::prelude::*;
use {leading}unitree_go::Go2FrontVideoData;

mod unitree_go {{
    include!(concat!(env!("OUT_DIR"), "/shadow.rs"));
}}

#[cerulion_node]
#[derive(Default)]
struct TranscodeNode {{
    #[input(trigger)]
    h264: Go2FrontVideoData,
}}
"#
            );
            let node_dir = write_node(&tmp, "colon", &src);
            let metadata = parse_node_metadata(&node_dir).unwrap();
            assert_eq!(metadata.inputs[0].schema.as_deref(), Some(want), "{why}");
        }
    }

    /// An UNRELATED sibling module cannot decide a port's schema.
    ///
    /// The import map is keyed by leaf ident. Without the ancestor-chain scoping
    /// every `use` in the file merges into one map, so a sibling module
    /// importing a colliding name overwrites the node's own import and the
    /// winner is decided by DECLARATION ORDER — a fabricated schema name
    /// refusing a valid graph.
    ///
    /// Driven BOTH orders against one hand oracle, because a declaration-order
    /// parser gets the right answer in one of them by luck: with the sibling
    /// module ABOVE the node it answers `sensor_msgs/Image`, and only the
    /// BELOW ordering exposes it (measured, both ways, on such a
    /// parser). A single-ordering test would pass the bug.
    #[test]
    fn an_unrelated_sibling_module_cannot_decide_a_ports_schema() {
        let tmp = TempDir::new().unwrap();
        let node = r#"
use cerulion_core::prelude::*;
use native_ros2_messages::sensor_msgs::Image;

#[cerulion_node(period_ms = 10)]
#[derive(Default)]
struct CamNode {
    #[output]
    img: Image,
}
"#;
        let sibling = r#"
mod other {
    use vendor_msgs::Image;
    pub fn f() -> Image { todo!() }
}
"#;
        for (order, src) in [
            (
                "sibling BELOW the node",
                format!(
                    "{node}
{sibling}"
                ),
            ),
            (
                "sibling ABOVE the node",
                format!(
                    "{sibling}
{node}"
                ),
            ),
        ] {
            let node_dir = write_node(&tmp, "cam", &src);
            let metadata = parse_node_metadata(&node_dir).unwrap();
            assert_eq!(
                metadata.outputs[0].schema.as_deref(),
                Some("sensor_msgs/Image"),
                "the node's OWN import must decide its port schema regardless of where an \
                 unrelated sibling module sits ({order})"
            );
        }
    }

    /// The `crate` / `self` / `super` keyword roots are
    /// crate-internal by construction — no `mod` declaration needed, and
    /// none of them is a schema package.
    ///
    /// Hand-written oracle per root, driven through the real parser. A parser
    /// that treats them as packages yields `crate/schema_types/TrackArray` and friends.
    #[test]
    fn the_keyword_roots_are_crate_internal() {
        let tmp = TempDir::new().unwrap();
        for (root, node_type) in [
            ("crate::schema_types", "kw_crate"),
            ("self::schema_types", "kw_self"),
            ("super::schema_types", "kw_super"),
        ] {
            let src = format!(
                r#"
use cerulion_core::prelude::*;
use {root}::TrackArray;

#[cerulion_node(period_ms = 10)]
#[derive(Default)]
struct TrackerNode {{
    #[output]
    tracks: TrackArray,
}}
"#
            );
            let node_dir = write_node(&tmp, node_type, &src);
            let metadata = parse_node_metadata(&node_dir).unwrap();
            assert_eq!(
                metadata.outputs[0].schema.as_deref(),
                Some("TrackArray"),
                "`use {root}::TrackArray;` names a path inside this crate, so the schema \
                 name is the bare leaf"
            );
        }
    }

    /// The cfg ALTERNATIVE is scoped to the
    /// import that produced it — a nearer import shadowing the same leaf
    /// replaces BOTH names, never one. Two shapes: (1) an ancestor scope's
    /// cfg pair (`#[cfg] mod unitree_go` + `use unitree_go::Image`) shadowed
    /// by the node scope's plain `use native_ros2_messages::sensor_msgs::Image`
    /// — the name is `sensor_msgs/Image` and the alternative is GONE (with
    /// two side-by-side maps it survived, and `graph validate` accepted a
    /// `schema: unitree_go/Image` that belonged to an import the node never
    /// used); (2) the mirror — an outer plain import shadowed by the node
    /// scope's own cfg pair — carries BOTH names, since that pair is the
    /// import in force.
    #[test]
    fn a_nearer_import_replaces_both_names_never_one() {
        let tmp = TempDir::new().unwrap();
        // (1) ancestor cfg pair, shadowed by a plain import in the node's scope.
        let node_dir = write_node(
            &tmp,
            "shadowed",
            r#"
use cerulion_core::prelude::*;
#[cfg(feature = "vendored")]
mod unitree_go {
    include!(concat!(env!("OUT_DIR"), "/vendored.rs"));
}
use unitree_go::Image;

mod inner {
    use super::*;
    use native_ros2_messages::sensor_msgs::Image;

    #[cerulion_node]
    #[derive(Default)]
    struct SinkNode {
        #[input(trigger)]
        frame: Image,
    }
}
"#,
        );
        let port = parse_node_metadata(&node_dir).unwrap().inputs.remove(0);
        assert_eq!(port.schema.as_deref(), Some("sensor_msgs/Image"));
        assert!(
            port.schema_alternatives.is_empty(),
            "the ancestor's cfg alternative must not outlive the import it belonged to"
        );

        // (2) the mirror: an outer plain import, shadowed by the node scope's
        // own cfg pair — the pair in force carries both names.
        let node_dir = write_node(
            &tmp,
            "shadowing",
            r#"
use cerulion_core::prelude::*;
use native_ros2_messages::sensor_msgs::Image;

mod inner {
    use super::*;
    #[cfg(feature = "vendored")]
    mod unitree_go {
        include!(concat!(env!("OUT_DIR"), "/vendored.rs"));
    }
    use unitree_go::Image;

    #[cerulion_node]
    #[derive(Default)]
    struct SinkNode {
        #[input(trigger)]
        frame: Image,
    }
}
"#,
        );
        let port = parse_node_metadata(&node_dir).unwrap().inputs.remove(0);
        assert_eq!(port.schema.as_deref(), Some("Image"));
        assert_eq!(
            port.schema_alternatives,
            ["unitree_go/Image"],
            "the pair in force carries both names"
        );
    }

    /// A `#[cfg]`-gated same-scope module resolves a bare
    /// `use` DIFFERENTLY per configuration, and the parser cannot see the
    /// configuration. MEASURED against rustc: `mod core {…}` +
    /// `use core::option::Option` is E0432 (while it exists the module
    /// shadows the crate — bare is right), and `#[cfg(any())] mod core {…}`
    /// + `use core::Probe` is E0432 on `core::Probe` (the inactive module is
    /// invisible and the import reaches the crate — qualified is right). So
    /// the parser keeps the bare answer, RECORDS the crate spelling as the
    /// port's `schema_alternative` (for the validator to accept), and says
    /// so LOUDLY with both unambiguous spellings — every path segment kept.
    /// A `cfg_attr`-emitted `cfg` is a gate too. The two controls run
    /// FIRST and must stay silent with no alternative: an un-gated module,
    /// and a leading-colon import beside the gated module.
    #[tracing_test::traced_test]
    #[test]
    fn a_cfg_gated_module_decides_a_ports_schema_loudly() {
        let tmp = TempDir::new().unwrap();
        let src = |gate: &str, import: &str| {
            format!(
                r#"
use cerulion_core::prelude::*;
use {import};

{gate}
mod unitree_go {{
    include!(concat!(env!("OUT_DIR"), "/vendored.rs"));
}}

#[cerulion_node]
#[derive(Default)]
struct TranscodeNode {{
    #[input(trigger)]
    h264: Go2FrontVideoData,
}}
"#
            )
        };
        let parse = |name: &str, gate: &str, import: &str| {
            let node_dir = write_node(&tmp, name, &src(gate, import));
            parse_node_metadata(&node_dir).unwrap().inputs.remove(0)
        };
        let warns_so_far = |expected: usize| {
            logs_assert(|lines: &[&str]| {
                let n = lines
                    .iter()
                    .filter(|l| l.contains("decides this port's schema name"))
                    .count();
                if n == expected {
                    Ok(())
                } else {
                    Err(format!("expected {expected} cfg warn(s), saw {n}"))
                }
            });
        };
        const GATE: &str = "#[cfg(feature = \"vendored\")]";

        // Control 1: an un-gated module decides silently, no alternative.
        let port = parse("ungated", "", "unitree_go::Go2FrontVideoData");
        assert_eq!(port.schema.as_deref(), Some("Go2FrontVideoData"));
        assert!(port.schema_alternatives.is_empty());
        warns_so_far(0);

        // Control 2: a leading `::` beside the gated module is the crate,
        // unconditionally — no alternative, nothing to warn about.
        let port = parse("colon", GATE, "::unitree_go::Go2FrontVideoData");
        assert_eq!(port.schema.as_deref(), Some("unitree_go/Go2FrontVideoData"));
        assert!(port.schema_alternatives.is_empty());
        warns_so_far(0);

        // The arm: gated module + bare import ⇒ bare, the crate spelling
        // recorded as the alternative, and EXACTLY one warn carrying both
        // spellings.
        let port = parse("gated", GATE, "unitree_go::Go2FrontVideoData");
        assert_eq!(
            port.schema.as_deref(),
            Some("Go2FrontVideoData"),
            "the module the author declared is the more specific evidence: bare"
        );
        assert_eq!(
            port.schema_alternatives,
            ["unitree_go/Go2FrontVideoData"],
            "the crate spelling is what the OTHER configuration publishes"
        );
        warns_so_far(1);
        assert!(
            logs_contain("use ::unitree_go::Go2FrontVideoData;")
                && logs_contain("use self::unitree_go::Go2FrontVideoData;")
                && logs_contain("alternative=unitree_go/Go2FrontVideoData"),
            "both unambiguous spellings and the alternative must be offered, as fields"
        );

        // A `cfg_attr`-emitted `cfg` gates the module just the same — when
        // its predicate can hold AND the payload is a real constraint.
        let port = parse(
            "cfg_attr",
            "#[cfg_attr(feature = \"vendored\", cfg(feature = \"other\"))]",
            "unitree_go::Go2FrontVideoData",
        );
        assert_eq!(
            port.schema_alternatives,
            ["unitree_go/Go2FrontVideoData"],
            "a cfg_attr-emitted undecidable cfg is a gate"
        );
        warns_so_far(2);

        // ALWAYS present — a plain local module, bare, no alternative, no
        // warn: no cfg constraint survives static evaluation.
        for gate in [
            "#[cfg(all())]",
            "#[cfg(true)]",
            "#[cfg_attr(any(), cfg(any()))]",
            "#[cfg_attr(false, cfg(all()))]",
            "#[cfg_attr(not(all()), cfg(all()))]",
            "#[cfg_attr(all(any(), feature = \"vendored\"), cfg(all()))]",
            "#[cfg_attr(all(), cfg(all()))]",
            "#[cfg_attr(feature = \"vendored\", cfg(all()))]",
        ] {
            let port = parse("always", gate, "unitree_go::Go2FrontVideoData");
            assert_eq!(port.schema.as_deref(), Some("Go2FrontVideoData"), "{gate}");
            assert!(
                port.schema_alternatives.is_empty(),
                "{gate}: a module that is always present gates nothing"
            );
            warns_so_far(2);
        }

        // NEVER present — the module declares nothing, so the import IS the
        // external crate: qualified, no alternative, no warn. This is
        // the one place the gating contract
        // differs: a statically-false cfg is resolved, not gated.
        for gate in [
            "#[cfg(any())]",
            "#[cfg(false)]",
            "#[cfg(not(all()))]",
            "#[cfg_attr(all(), cfg(any()))]",
        ] {
            let port = parse("never", gate, "unitree_go::Go2FrontVideoData");
            assert_eq!(
                port.schema.as_deref(),
                Some("unitree_go/Go2FrontVideoData"),
                "{gate}: a module that can never exist leaves the import to the crate"
            );
            assert!(
                port.schema_alternatives.is_empty(),
                "{gate}: nothing to be ambiguous about"
            );
            warns_so_far(2);
        }

        // UNDECIDABLE — still a gate: bare, the crate alternative, a warn —
        // counted per row, so a row that gates silently fails on its own.
        for (i, gate) in [
            "#[cfg(feature = \"vendored\")]",
            "#[cfg(unix)]",
            "#[cfg(any(feature = \"vendored\", any()))]",
            "#[cfg_attr(any(feature = \"vendored\", any()), cfg(feature = \"other\"))]",
        ]
        .into_iter()
        .enumerate()
        {
            let port = parse("undecidable", gate, "unitree_go::Go2FrontVideoData");
            assert_eq!(port.schema.as_deref(), Some("Go2FrontVideoData"), "{gate}");
            assert_eq!(
                port.schema_alternatives,
                ["unitree_go/Go2FrontVideoData"],
                "{gate}: an undecidable cfg is a gate"
            );
            warns_so_far(3 + i);
        }

        // Nested segments survive: in the alternative and in both spellings.
        let port = parse("nested", GATE, "unitree_go::msg::Go2FrontVideoData");
        assert_eq!(port.schema.as_deref(), Some("Go2FrontVideoData"));
        assert_eq!(
            port.schema_alternatives,
            ["unitree_go/msg/Go2FrontVideoData"]
        );
        warns_so_far(7);
        assert!(
            logs_contain("use ::unitree_go::msg::Go2FrontVideoData;")
                && logs_contain("use self::unitree_go::msg::Go2FrontVideoData;"),
            "the spellings must keep every segment of the original path"
        );
    }

    /// Module presence is decided by ONE table over every predicate shape a
    /// `cfg` can carry.
    /// Statically decidable ⇒ `Always` / `Never`; anything that depends on
    /// the build (a feature, a target, `test`, `debug_assertions`, a custom
    /// key) ⇒ `Conditional`, the gate. `cfg_attr` folds through its
    /// predicate (false ⇒ never emitted; true ⇒ as if direct; undecidable
    /// ⇒ gates only if the payload constrains); stacked attributes fold
    /// (any `Never` wins, else any `Conditional`); inline and file modules
    /// classify alike (the attributes are the whole input).
    #[test]
    fn module_presence_is_decided_by_the_full_predicate_table() {
        use ModPresence::*;
        let table: &[(&str, ModPresence)] = &[
            // no constraint
            ("", Always),
            ("#[allow(dead_code)]", Always),
            // literals and empty combinators
            ("#[cfg(true)]", Always),
            ("#[cfg(false)]", Never),
            ("#[cfg(all())]", Always),
            ("#[cfg(any())]", Never),
            // not
            ("#[cfg(not(all()))]", Never),
            ("#[cfg(not(any()))]", Always),
            ("#[cfg(not(false))]", Always),
            ("#[cfg(not(feature = \"x\"))]", Conditional),
            ("#[cfg(not(not(any())))]", Never),
            // nested combinators, decidable
            ("#[cfg(any(all(), any()))]", Always),
            ("#[cfg(all(any(), all()))]", Never),
            ("#[cfg(all(not(any()), true))]", Always),
            ("#[cfg(any(not(all()), false))]", Never),
            // short-circuiting against an undecidable term
            ("#[cfg(any(feature = \"x\", true))]", Always),
            ("#[cfg(any(feature = \"x\", all()))]", Always),
            ("#[cfg(all(feature = \"x\", false))]", Never),
            ("#[cfg(all(feature = \"x\", any()))]", Never),
            ("#[cfg(all(feature = \"x\", true))]", Conditional),
            ("#[cfg(any(feature = \"x\", any()))]", Conditional),
            ("#[cfg(all(unix, any()))]", Never),
            ("#[cfg(any(windows, all()))]", Always),
            // undecidable keys of every kind
            ("#[cfg(feature = \"x\")]", Conditional),
            ("#[cfg(target_os = \"linux\")]", Conditional),
            ("#[cfg(target_arch = \"aarch64\")]", Conditional),
            ("#[cfg(unix)]", Conditional),
            ("#[cfg(windows)]", Conditional),
            ("#[cfg(test)]", Conditional),
            ("#[cfg(debug_assertions)]", Conditional),
            ("#[cfg(my_custom_key)]", Conditional),
            ("#[cfg(my_key = \"v\")]", Conditional),
            ("#[cfg(version(\"1.80\"))]", Conditional),
            ("#[cfg(not(any(feature = \"x\", true)))]", Never),
            // cfg_attr: predicate decides whether the payload exists
            ("#[cfg_attr(any(), cfg(any()))]", Always),
            ("#[cfg_attr(false, cfg(all()))]", Always),
            ("#[cfg_attr(all(), cfg(any()))]", Never),
            ("#[cfg_attr(true, cfg(all()))]", Always),
            ("#[cfg_attr(all(), cfg(feature = \"x\"))]", Conditional),
            ("#[cfg_attr(feature = \"x\", cfg(all()))]", Always),
            ("#[cfg_attr(feature = \"x\", cfg(any()))]", Conditional),
            (
                "#[cfg_attr(feature = \"x\", cfg(feature = \"y\"))]",
                Conditional,
            ),
            ("#[cfg_attr(feature = \"x\", derive(Debug))]", Always),
            // multi-payload and nested cfg_attr
            (
                "#[cfg_attr(feature = \"x\", derive(Debug), cfg(any()))]",
                Conditional,
            ),
            ("#[cfg_attr(all(), derive(Debug), cfg(any()))]", Never),
            ("#[cfg_attr(all(), cfg_attr(all(), cfg(any())))]", Never),
            ("#[cfg_attr(all(), cfg_attr(any(), cfg(any())))]", Always),
            (
                "#[cfg_attr(feature = \"x\", cfg_attr(all(), cfg(all())))]",
                Always,
            ),
            (
                "#[cfg_attr(feature = \"x\", cfg_attr(all(), cfg(unix)))]",
                Conditional,
            ),
            // `not` takes exactly one term; any other arity is undecidable
            ("#[cfg(not())]", Conditional),
            ("#[cfg(not(all(), any()))]", Conditional),
            // an attribute the parser cannot READ is undecidable, never decided
            // (a `cfg_attr` nobody had parsed once read as `Always`); a bare
            // literal payload is not an attribute either
            ("#[cfg_attr(feature = \"x\", 123)]", Conditional),
            ("#[cfg_attr(feature = \"x\", true)]", Conditional),
            ("#[cfg_attr]", Conditional),
            ("#[cfg]", Conditional),
            // stacked attributes fold
            ("#[cfg(feature = \"x\")] #[cfg(any())]", Never),
            ("#[cfg(any())] #[cfg(feature = \"x\")]", Never),
            ("#[cfg(all())] #[cfg(feature = \"x\")]", Conditional),
            ("#[cfg(unix)] #[cfg(windows)]", Conditional),
            ("#[cfg(all())] #[cfg(true)]", Always),
            ("#[allow(dead_code)] #[cfg(any())]", Never),
        ];
        for (attrs, expected) in table {
            for shape in ["mod probe {}", "mod probe;"] {
                let item: syn::ItemMod = syn::parse_str(&format!("{attrs} {shape}"))
                    .unwrap_or_else(|e| panic!("{attrs} {shape}: {e}"));
                assert_eq!(mod_presence(&item.attrs), *expected, "{attrs} ({shape})");
            }
        }
    }

    /// A `cfg` on a `use` (or on a
    /// `pub use` re-export) is decided like one on a `mod`. A never-present
    /// import imports nothing (the other import of that leaf stands alone,
    /// no alternative, no warn); a cfg-EXCLUSIVE pair of imports of one leaf
    /// in one scope is the first as the schema name and the second as the
    /// cfg alternative, said loudly; an always-present import is ordinary.
    #[tracing_test::traced_test]
    #[test]
    fn a_cfg_on_a_use_is_decided_like_a_cfg_on_a_mod() {
        let tmp = TempDir::new().unwrap();
        let src = |imports: &str| {
            format!(
                r#"
use cerulion_core::prelude::*;
{imports}

#[cerulion_node]
#[derive(Default)]
struct SinkNode {{
    #[input(trigger)]
    frame: Frame,
}}
"#
            )
        };
        let parse = |name: &str, imports: &str| {
            let node_dir = write_node(&tmp, name, &src(imports));
            parse_node_metadata(&node_dir).unwrap().inputs.remove(0)
        };
        // The two shapes are counted SEPARATELY, so a mis-routed shape (an
        // `AncestorShadow` where an `ExclusivePair` belongs) cannot keep the
        // total right; every line must also carry the node type, which is
        // how the validator's advisory points an operator at it.
        let pair_warns = |exclusive: usize, ancestor: usize| {
            logs_assert(|lines: &[&str]| {
                let count = |needle: &str| {
                    lines
                        .iter()
                        .filter(|l| l.contains(needle) && l.contains("node_type="))
                        .count()
                };
                let e = count("two imports of one name");
                let a = count("shadows an ancestor scope's import of the same type");
                if (e, a) == (exclusive, ancestor) {
                    Ok(())
                } else {
                    Err(format!(
                        "expected {exclusive} exclusive-pair + {ancestor} ancestor warn(s), saw \
                         {e} + {a}"
                    ))
                }
            });
        };

        // A never-present import is not an import.
        let port = parse(
            "never",
            "#[cfg(any())]\nuse never_crate::Frame;\nuse real_crate::Frame;",
        );
        assert_eq!(port.schema.as_deref(), Some("real_crate/Frame"));
        assert!(port.schema_alternatives.is_empty());
        pair_warns(0, 0);

        // An always-present import is ordinary.
        let port = parse("always", "#[cfg(all())]\npub use always_crate::Frame;");
        assert_eq!(port.schema.as_deref(), Some("always_crate/Frame"));
        assert!(port.schema_alternatives.is_empty());
        pair_warns(0, 0);

        // A cfg-exclusive pair: first is the name, second the alternative.
        let port = parse(
            "pair",
            "#[cfg(feature = \"a\")]\nuse crate_a::Frame;\n#[cfg(not(feature = \"a\"))]\npub use crate_b::Frame;",
        );
        assert_eq!(port.schema.as_deref(), Some("crate_a/Frame"));
        assert_eq!(port.schema_alternatives, ["crate_b/Frame"]);
        pair_warns(1, 0);

        // A PLAIN import beside a conditional one of the same leaf, either
        // order: one buildable configuration (the conditional import off)
        // plus one E0252. The parser does NOT try to pick the buildable
        // half — it reports the FIRST import in source order as the name and
        // the second as the alternative, and `graph validate` accepts
        // either; so the order here is arbitrary, not a claim about which
        // build wins.
        let port = parse(
            "plain_then_cond",
            "use crate_a::Frame;\n#[cfg(feature = \"a\")]\nuse crate_b::Frame;",
        );
        assert_eq!(port.schema.as_deref(), Some("crate_a/Frame"));
        assert_eq!(port.schema_alternatives, ["crate_b/Frame"]);
        pair_warns(2, 0);
        let port = parse(
            "cond_then_plain",
            "#[cfg(feature = \"a\")]\nuse crate_a::Frame;\nuse crate_b::Frame;",
        );
        assert_eq!(port.schema.as_deref(), Some("crate_a/Frame"));
        assert_eq!(port.schema_alternatives, ["crate_b/Frame"]);
        pair_warns(3, 0);

        // THREE cfg-exclusive imports (the vendor-selection shape: `a`, `b`,
        // and `not(any(a, b))`) are three names. A one-slot alternative
        // dropped the second, and a graph written for `feature = "b"` was
        // REFUSED under the fail-closed run — the very class this parser
        // exists to end, one import further along.
        let port = parse(
            "three_way",
            "#[cfg(feature = \"a\")]\nuse crate_a::Frame;\n#[cfg(feature = \"b\")]\nuse \
             crate_b::Frame;\n#[cfg(not(any(feature = \"a\", feature = \"b\")))]\nuse \
             crate_c::Frame;",
        );
        assert_eq!(port.schema.as_deref(), Some("crate_a/Frame"));
        assert_eq!(port.schema_alternatives, ["crate_b/Frame", "crate_c/Frame"]);
        pair_warns(5, 0);

        // A conditional import shadowing an OUTER scope's plain import is a
        // two-configuration shape as well: with the cfg off, the ancestor's
        // import is in force — the nearer is the name, the ancestor's the
        // alternative, warned.
        let node_dir = write_node(
            &tmp,
            "nested",
            r#"
use cerulion_core::prelude::*;
use outer_crate::Frame;

mod inner {
    use super::*;
    #[cfg(feature = "a")]
    use inner_crate::Frame;

    #[cerulion_node]
    #[derive(Default)]
    struct SinkNode {
        #[input(trigger)]
        frame: Frame,
    }
}
"#,
        );
        let port = parse_node_metadata(&node_dir).unwrap().inputs.remove(0);
        assert_eq!(port.schema.as_deref(), Some("inner_crate/Frame"));
        assert_eq!(port.schema_alternatives, ["outer_crate/Frame"]);
        pair_warns(5, 1);
        assert!(
            logs_contain("shadows an ancestor scope's import of the same type"),
            "the ancestor pair is said in its own words"
        );
    }

    /// The advisory is said by the PORT that binds a cfg-decided leaf, not by
    /// the `use` that imported it (a fixture binding its gated leaf to
    /// exactly one port cannot tell the two sites apart, so nothing else
    /// pins where the warn is said). A gated import NO port binds says
    /// nothing; a leaf bound by an input AND an output is said twice, once
    /// per port by name, each line carrying the node type the validator's
    /// advisory points at.
    #[tracing_test::traced_test]
    #[test]
    fn the_cfg_advisory_is_said_once_per_port_that_binds_the_leaf() {
        let tmp = TempDir::new().unwrap();
        let count = |needle: &str| {
            let n = std::cell::Cell::new(0);
            logs_assert(|lines: &[&str]| {
                n.set(lines.iter().filter(|l| l.contains(needle)).count());
                Ok(())
            });
            n.get()
        };
        const GATED: &str = r#"
use cerulion_core::prelude::*;
use native_ros2_messages::sensor_msgs::Image;
use unitree_go::Go2FrontVideoData;

#[cfg(feature = "vendored")]
mod unitree_go {
    include!(concat!(env!("OUT_DIR"), "/vendored.rs"));
}
"#;
        // No port binds the gated leaf: nothing to say.
        let unused = write_node(
            &tmp,
            "unused",
            &format!(
                "{GATED}
#[cerulion_node]
#[derive(Default)]
struct OnlyImages {{
    #[input(trigger)]
    frame: Image,
}}
"
            ),
        );
        let md = parse_node_metadata(&unused).unwrap();
        assert_eq!(md.inputs[0].schema.as_deref(), Some("sensor_msgs/Image"));
        assert_eq!(
            count("decides this port's schema name"),
            0,
            "a cfg-decided import no port binds is not a claim about any port"
        );

        // Two ports bind it: two lines, one per port, each naming the node.
        let both = write_node(
            &tmp,
            "both",
            &format!(
                "{GATED}
#[cerulion_node]
#[derive(Default)]
struct Transcode {{
    #[input(trigger)]
    h264_in: Go2FrontVideoData,
    #[output]
    h264_out: Go2FrontVideoData,
}}
"
            ),
        );
        let md = parse_node_metadata(&both).unwrap();
        assert_eq!(
            md.inputs[0].schema_alternatives,
            ["unitree_go/Go2FrontVideoData"]
        );
        assert_eq!(
            md.outputs[0].schema_alternatives,
            ["unitree_go/Go2FrontVideoData"]
        );
        assert_eq!(count("decides this port's schema name"), 2);
        assert_eq!(count("port=h264_in"), 1, "said for the input, by name");
        assert_eq!(count("port=h264_out"), 1, "…and for the output, by name");
        assert_eq!(
            count("node_type=both"),
            2,
            "every line names the node type the validator's advisory points at"
        );
    }

    /// A `#[cfg]`-gated glob that is
    /// the ONLY path to a port's type — the root's `use old_msgs::Leaf;`
    /// reached solely through `#[cfg(feature = "n")] use super::*` — reports
    /// `old_msgs/Leaf` with no alternative (no other configuration the
    /// parser can see publishes anything) AND says so, for the PORT (said
    /// from the glob walk it would name no node and no port), on ONE line:
    /// the label is a fact about one build. Controls: a plain glob says
    /// nothing; two cfg-exclusive globs binding differently are NOT an only
    /// path (both names are carried — the `bound.len() == 1`
    /// guard); a conditional glob beside one the parser cannot read
    /// into is undecidable, with no schema to claim anything about. What
    /// fails it: the note never set → the first arm; `bound.len() >= 1` → the
    /// pair arm.
    #[tracing_test::traced_test]
    #[test]
    fn a_cfg_gated_glob_that_is_the_only_path_to_a_type_is_said() {
        let tmp = TempDir::new().unwrap();
        let port = |name: &str, inner: &str| {
            let src = format!(
                "use cerulion_core::prelude::*;\nuse old_msgs::Leaf;\n\nmod inner {{\n    {inner}\n\n    \
                 #[cerulion_node]\n    #[derive(Default)]\n    struct SinkNode {{\n        \
                 #[input(trigger)]\n        frame: Leaf,\n    }}\n}}\n"
            );
            parse_node_metadata(&write_node(&tmp, name, &src))
                .unwrap()
                .inputs
                .remove(0)
        };
        let plain = port("plain", "use super::*;");
        assert_eq!(plain.schema.as_deref(), Some("old_msgs/Leaf"));
        assert!(
            !logs_contain("is the only path to this port's type"),
            "a plain glob is every build's: nothing to say"
        );
        let gated = port("gated", "#[cfg(feature = \"n\")]\n    use super::*;");
        assert_eq!(gated.schema.as_deref(), Some("old_msgs/Leaf"));
        assert!(
            gated.schema_alternatives.is_empty(),
            "no other build publishes anything"
        );
        let only_path_lines = |lines: &[&str]| {
            lines
                .iter()
                .filter(|l| l.contains("is the only path the parser sees to this port's type"))
                .count()
        };
        logs_assert(|lines: &[&str]| {
            let complete = lines.iter().any(|l| {
                l.contains("is the only path the parser sees to this port's type")
                    && l.contains("node_type=gated")
                    && l.contains("port=frame")
                    && l.contains("glob=use super::*;")
                    && l.contains("schema=old_msgs/Leaf")
            });
            match (only_path_lines(lines), complete) {
                (1, true) => Ok(()),
                (n, complete) => Err(format!(
                    "the cfg-gated only path is said ONCE, for the port, naming the node type, \
                     the port, the glob and the schema on one line — said {n} time(s), one \
                     complete line: {complete}"
                )),
            }
        });
        // Not an only path: two cfg-exclusive globs binding differently
        // carry BOTH names…
        let pair_src = "use cerulion_core::prelude::*;\nuse old_msgs::Leaf;\nmod mid {\n    use \
                        mid_msgs::Leaf;\n    mod inner {\n        #[cfg(feature = \"a\")]\n        \
                        use super::*;\n        #[cfg(not(feature = \"a\"))]\n        use \
                        super::super::*;\n\n        #[cerulion_node]\n        #[derive(Default)]\n        \
                        struct SinkNode {\n            #[input(trigger)]\n            frame: Leaf,\n        \
                        }\n    }\n}\n";
        let pair = parse_node_metadata(&write_node(&tmp, "pair", pair_src))
            .unwrap()
            .inputs
            .remove(0);
        assert_eq!(pair.schema.as_deref(), Some("mid_msgs/Leaf"));
        assert_eq!(pair.schema_alternatives, vec!["old_msgs/Leaf"]);
        // …and a conditional glob beside one the parser cannot read into is
        // undecidable.
        let unknowable = port(
            "unknowable",
            "#[cfg(feature = \"a\")]\n    use super::*;\n    #[cfg(not(feature = \"a\"))]\n    \
             use new_msgs::*;",
        );
        assert_eq!(unknowable.schema, None);
        logs_assert(|lines: &[&str]| match only_path_lines(lines) {
            1 => Ok(()),
            n => Err(format!(
                "neither a cfg-exclusive pair nor an undecidable leaf is an only path — {n} \
                 line(s), expected the gated arm's one"
            )),
        });
    }

    /// A `#[cfg]`-gated `use` binding the MODULE a
    /// port's type path starts with is the gated-module defect one binding
    /// smaller — with the cfg off the path starts at the extern crate — so
    /// the port carries BOTH names and the advisory names the port, the
    /// import and the crate spelling; a cfg-exclusive PAIR of module imports
    /// carries every name plus the prelude's, said as a pair AND as a gated
    /// import; a plain import carries one name and says nothing. What fails it:
    /// the first import taken as unconditional → the lone arm has no
    /// alternative; the pair's second import dropped → the pair arm.
    #[tracing_test::traced_test]
    #[test]
    fn a_gated_module_import_is_said_for_the_port_that_binds_it() {
        let tmp = TempDir::new().unwrap();
        let port = |name: &str, imports: &str| {
            let src = format!(
                "use cerulion_core::prelude::*;\n{imports}\n\n#[cerulion_node]\n#[derive(Default)]\n\
                 struct SinkNode {{\n    #[input(trigger)]\n    frame: Type,\n}}\n"
            );
            parse_node_metadata(&write_node(&tmp, name, &src))
                .unwrap()
                .inputs
                .remove(0)
        };
        let line_with = |lines: &[&str], needles: &[&str]| {
            lines
                .iter()
                .any(|l| needles.iter().all(|needle| l.contains(needle)))
        };
        let lone = port(
            "lone",
            "#[cfg(feature = \"v\")]\nuse vendor::foo;\nuse foo::Type;",
        );
        assert_eq!(lone.schema.as_deref(), Some("vendor/foo/Type"));
        assert_eq!(lone.schema_alternatives, vec!["foo/Type"]);
        logs_assert(|lines: &[&str]| {
            if line_with(
                lines,
                &[
                    "binds the module this port's type path starts with",
                    "node_type=lone",
                    "port=frame",
                    "module=foo",
                    "import=foo::Type",
                    "reported=vendor/foo/Type",
                    "alternative=foo/Type",
                    "crate_spelling=use ::foo::Type;",
                ],
            ) {
                Ok(())
            } else {
                Err("the gated import is said for the port, with every field".to_string())
            }
        });
        let pair = port(
            "pair",
            "#[cfg(feature = \"a\")]\nuse vendor_a::foo;\n#[cfg(not(feature = \"a\"))]\nuse \
             vendor_b::foo;\nuse foo::Type;",
        );
        assert_eq!(pair.schema.as_deref(), Some("vendor_a/foo/Type"));
        assert_eq!(
            pair.schema_alternatives,
            vec!["vendor_b/foo/Type", "foo/Type"],
            "every import's name, then the prelude's"
        );
        logs_assert(|lines: &[&str]| {
            let as_pair = line_with(
                lines,
                &[
                    "two imports of one name",
                    "node_type=pair",
                    "first=vendor_a/foo/Type",
                    "other=vendor_b/foo/Type",
                ],
            );
            let as_gated = line_with(
                lines,
                &[
                    "binds the module this port's type path starts with",
                    "node_type=pair",
                    "alternative=foo/Type",
                ],
            );
            if as_pair && as_gated {
                Ok(())
            } else {
                Err(format!(
                    "pair said as a pair: {as_pair}, prelude as a gated import: {as_gated}"
                ))
            }
        });
        let plain = port("plain", "use vendor::foo;\nuse foo::Type;");
        assert_eq!(plain.schema.as_deref(), Some("vendor/foo/Type"));
        assert!(plain.schema_alternatives.is_empty());
        assert!(
            !logs_contain("node_type=plain"),
            "one plain import: one name, nothing to say"
        );
    }

    /// The `GatedGlob` advisory names the remedies that fit the
    /// shape — the ancestor's module BY PATH through the reaching glob
    /// (`use super::detections::TrackArray;`) and the crate (`use
    /// ::detections::TrackArray;`) — never the same-scope `self::` spelling,
    /// which does not exist for an ancestor's module (the shape it replaced
    /// said exactly that). Both gated sides: the glob, and the module.
    #[tracing_test::traced_test]
    #[test]
    fn a_gated_ancestor_glob_advisory_names_the_path_through_the_glob() {
        let tmp = TempDir::new().unwrap();
        for (i, (outer, inner)) in [
            (
                "mod detections { include!(concat!(env!(\"OUT_DIR\"), \"/d.rs\")); }",
                "#[cfg(feature = \"n\")]\n    use super::*;\n    use detections::TrackArray;",
            ),
            (
                "#[cfg(feature = \"v\")]\nmod detections { include!(concat!(env!(\"OUT_DIR\"), \"/d.rs\")); }",
                "use super::*;\n    use detections::TrackArray;",
            ),
            // The reaching glob's PATH is the remedy's, not `super` (with
            // both arms reaching through `super::*`, a hardcoded `super`
            // would pass).
            (
                "mod detections { include!(concat!(env!(\"OUT_DIR\"), \"/d.rs\")); }",
                "#[cfg(feature = \"n\")]\n    use crate::*;\n    use detections::TrackArray;",
            ),
        ]
        .iter()
        .enumerate()
        {
            let src = format!(
                "use cerulion_core::prelude::*;\n{outer}\n\nmod inner {{\n    {inner}\n\n    \
                 #[cerulion_node]\n    #[derive(Default)]\n    struct SinkNode {{\n        \
                 #[input(trigger)]\n        frame: TrackArray,\n    }}\n}}\n"
            );
            let node_dir = write_node(&tmp, &format!("gated_glob_{i}"), &src);
            let port = parse_node_metadata(&node_dir).unwrap().inputs.remove(0);
            assert_eq!(port.schema.as_deref(), Some("TrackArray"));
            assert_eq!(port.schema_alternatives, vec!["detections/TrackArray"]);
        }
        assert!(
            logs_contain("use super::detections::TrackArray;")
                && logs_contain("use crate::detections::TrackArray;")
                && logs_contain("use ::detections::TrackArray;")
                && !logs_contain("use self::detections::TrackArray;"),
            "the advisory names the path through the glob (its own path) and the crate, \
             never `self::`"
        );
        assert!(
            logs_contain("gated=the glob") && logs_contain("gated=the module"),
            "each shape says which side carries the cfg"
        );
    }

    /// The use-walk STATE SPACE as one table — import shape × ancestor
    /// reach × cfg × module body → the port's verdict — each row a fixture
    /// through the real parser against a hand oracle (schema, alternatives,
    /// and whether an undecidable-loud warn was said). This is the table
    /// `resolve_leaf` documents, pinned row by row; a new edge is a new
    /// row here, not a new patch.
    #[tracing_test::traced_test]
    #[test]
    fn the_use_walk_state_space() {
        let tmp = TempDir::new().unwrap();
        const NRM: &str = "native_ros2_messages::sensor_msgs::Image";
        // A root-level node with `imports` above it.
        let root = |imports: &str, ty: &str| {
            format!(
                "use cerulion_core::prelude::*;\n{imports}\n\n#[cerulion_node]\n#[derive(Default)]\n\
                 struct SinkNode {{\n    #[input(trigger)]\n    frame: {ty},\n}}\n"
            )
        };
        // A node nested in `mod inner`, with `outer` at the root and `inner_uses` inside.
        let nested = |outer: &str, inner_uses: &str, ty: &str| {
            format!(
                "use cerulion_core::prelude::*;\n{outer}\n\nmod inner {{\n    {inner_uses}\n\n    \
                 #[cerulion_node]\n    #[derive(Default)]\n    struct SinkNode {{\n        \
                 #[input(trigger)]\n        frame: {ty},\n    }}\n}}\n"
            )
        };
        // (row, source, expected schema, expected alternatives, expected undecidable warn)
        type Row<'a> = (
            &'a str,
            String,
            Option<&'a str>,
            Vec<&'a str>,
            Option<&'a str>,
        );
        let rows: Vec<Row<'_>> = vec![
            // ---- the path's ROOT ----
            ("leading colon is the external crate",
             root("mod vendor { pub use native_ros2_messages::sensor_msgs::Image as Frame; }\nuse ::vendor::Frame;", "Frame"),
             Some("vendor/Frame"), vec![], None),
            ("crate:: to a declared inline codegen module is bare",
             root("mod detections { include!(concat!(env!(\"OUT_DIR\"), \"/d.rs\")); }\nuse crate::detections::TrackArray;", "TrackArray"),
             Some("TrackArray"), vec![], None),
            ("self:: to a declared module is bare",
             root("mod detections { include!(concat!(env!(\"OUT_DIR\"), \"/d.rs\")); }\nuse self::detections::TrackArray;", "TrackArray"),
             Some("TrackArray"), vec![], None),
            ("super:: above the crate root is bare (crate-internal, unwalkable)",
             root("use super::schema_types::TrackArray;", "TrackArray"),
             Some("TrackArray"), vec![], None),
            ("super::Leaf from a nested scope is the parent's binding",
             nested(&format!("use {NRM};"), "use super::Image;", "Image"),
             Some("sensor_msgs/Image"), vec![], None),
            ("crate::module::Leaf re-export resolves from a nested scope",
             nested(&format!("mod aliases {{ pub use {NRM}; }}"), "use crate::aliases::Image;", "Image"),
             Some("sensor_msgs/Image"), vec![], None),
            ("an undeclared first segment is the external crate",
             root("use vendor_msgs::Frame;", "Frame"),
             Some("vendor_msgs/Frame"), vec![], None),
            // ---- a module THIS scope declares ----
            ("inline module re-exporting a built-in",
             root(&format!("mod aliases {{ pub use {NRM}; }}\nuse aliases::Image;"), "Image"),
             Some("sensor_msgs/Image"), vec![], None),
            ("inline module re-exporting a vendor type keeps its package",
             root("mod aliases { pub use unitree_go::Go2FrontVideoData; }\nuse aliases::Go2FrontVideoData;", "Go2FrontVideoData"),
             Some("unitree_go/Go2FrontVideoData"), vec![], None),
            ("inline module with no pub use of the leaf is the codegen shape",
             root("mod detections { include!(concat!(env!(\"OUT_DIR\"), \"/d.rs\")); }\nuse detections::TrackArray;", "TrackArray"),
             Some("TrackArray"), vec![], None),
            ("a PRIVATE use inside the module is not a re-export",
             root(&format!("mod aliases {{ use {NRM}; }}\nuse aliases::Image;"), "Image"),
             Some("Image"), vec![], None),
            ("out-of-line same-scope module is the codegen shape",
             root("mod detections;\nuse detections::TrackArray;", "TrackArray"),
             Some("TrackArray"), vec![], None),
            ("cfg-gated inline codegen module: bare plus the crate spelling",
             root("#[cfg(feature = \"v\")]\nmod unitree_go { include!(concat!(env!(\"OUT_DIR\"), \"/v.rs\")); }\nuse unitree_go::Go2FrontVideoData;", "Go2FrontVideoData"),
             Some("Go2FrontVideoData"), vec!["unitree_go/Go2FrontVideoData"], None),
            ("cfg-gated inline re-export module: the target plus the crate spelling",
             root(&format!("#[cfg(feature = \"v\")]\nmod aliases {{ pub use {NRM}; }}\nuse aliases::Image;"), "Image"),
             Some("sensor_msgs/Image"), vec!["aliases/Image"], None),
            ("never-present module leaves the import to the crate",
             root(&format!("#[cfg(any())]\nmod aliases {{ pub use {NRM}; }}\nuse aliases::Image;"), "Image"),
             Some("aliases/Image"), vec![], None),
            ("nested path through inline sub-modules",
             root(&format!("mod aliases {{ pub mod deep {{ pub use {NRM}; }} }}\nuse aliases::deep::Image;"), "Image"),
             Some("sensor_msgs/Image"), vec![], None),
            ("nested path through an out-of-line sub-module is the codegen shape",
             root("mod aliases { pub mod deep; }\nuse aliases::deep::Image;", "Image"),
             Some("Image"), vec![], None),
            // ---- renames ----
            ("rename at the node is keyed by the field's name",
             root(&format!("use {NRM} as Img;"), "Img"),
             Some("sensor_msgs/Image"), vec![], None),
            ("rename inside the module",
             root(&format!("mod aliases {{ pub use {NRM} as Img; }}\nuse aliases::Img;"), "Img"),
             Some("sensor_msgs/Image"), vec![], None),
            // ---- undecidable: globs and cycles CLEAR ----
            ("a glob re-export is undecidable, loud",
             root("mod aliases { pub use native_ros2_messages::sensor_msgs::*; }\nuse aliases::Image;", "Image"),
             None, vec![], Some("re-exports through a glob")),
            ("a glob re-export CLEARS an older glob-visible binding of the leaf",
             nested("use vendor_msgs::Image;\nmod aliases { pub use native_ros2_messages::sensor_msgs::*; }", "use super::*;\n    use aliases::Image;", "Image"),
             None, vec![], Some("re-exports through a glob")),
            (
                "a re-export cycle is undecidable, loud, and stays so across the hop",
                root(
                    "mod a { pub use super::b::Image; }
mod b { pub use super::a::Image; }
use a::Image;",
                    "Image",
                ),
                None,
                vec![],
                Some("could not decide"),
            ),
            (
                "a chain ending in a glob is undecidable across the hop",
                root(
                    "mod first { pub use native_ros2_messages::sensor_msgs::*; }
mod aliases { pub use super::first::Image; }
use aliases::Image;",
                    "Image",
                ),
                None,
                vec![],
                Some("could not decide"),
            ),
            ("a node-scope glob with an unnamed type is undecidable, loud",
             root("use vendor_msgs::*;", "Frame"),
             None, vec![], Some("while a glob import is in scope")),
            ("an explicit import beside a node-scope glob wins",
             root("use vendor_msgs::*;\nuse other::Frame;", "Frame"),
             Some("other/Frame"), vec![], None),
            // ---- chains through sibling modules ----
            ("a chain through a sibling module resolves to its target",
             root(&format!("mod first {{ pub use {NRM}; }}\nmod aliases {{ pub use super::first::Image; }}\nuse aliases::Image;"), "Image"),
             Some("sensor_msgs/Image"), vec![], None),
            ("a chain through a sibling reached by the module's own super glob",
             root(&format!("mod first {{ pub use {NRM}; }}\nmod aliases {{ use super::*; pub use first::Image; }}\nuse aliases::Image;"), "Image"),
             Some("sensor_msgs/Image"), vec![], None),
            ("a module's pub super glob re-exports the parent's binding",
             root(&format!("use {NRM};\nmod aliases {{ pub use super::*; }}\nmod other {{ pub use super::aliases::Image; }}\nuse other::Image as Img;"), "Img"),
             Some("sensor_msgs/Image"), vec![], None),
            // ---- ancestor reach: direct globs only ----
            ("an ancestor's inline module through use super::*",
             nested(&format!("mod aliases {{ pub use {NRM}; }}"), "use super::*;\n    use aliases::Image;", "Image"),
             Some("sensor_msgs/Image"), vec![], None),
            ("an ancestor's OUT-OF-LINE module through use super::* is crate-internal, bare",
             nested("mod schema_types;", "use super::*;\n    use schema_types::TrackArray;", "TrackArray"),
             Some("TrackArray"), vec![], None),
            ("an ancestor's module without a glob is the external crate",
             nested("mod schema_types;", "use schema_types::TrackArray;", "TrackArray"),
             Some("schema_types/TrackArray"), vec![], None),
            ("a NARROW glob (use super::helpers::*) reaches no ancestor",
             nested("mod schema_types;\nmod helpers {}", "use super::helpers::*;\n    use schema_types::TrackArray;", "TrackArray"),
             Some("schema_types/TrackArray"), vec![], None),
            ("use crate::* reaches the root from depth one",
             nested(&format!("mod aliases {{ pub use {NRM}; }}"), "use crate::*;\n    use aliases::Image;", "Image"),
             Some("sensor_msgs/Image"), vec![], None),
            // ---- ancestor imports as cfg alternatives ----
            ("an ancestor import is a cfg alternative through use super::*",
             nested("use old_msgs::Leaf;", "use super::*;\n    #[cfg(feature = \"n\")]\n    use new_msgs::Leaf;", "Leaf"),
             Some("new_msgs/Leaf"), vec!["old_msgs/Leaf"], None),
            ("…not without a glob",
             nested("use old_msgs::Leaf;", "#[cfg(feature = \"n\")]\n    use new_msgs::Leaf;", "Leaf"),
             Some("new_msgs/Leaf"), vec![], None),
            ("…not through a narrow glob (use super::helpers::*)",
             nested("use old_msgs::Leaf;\nmod helpers {}", "use super::helpers::*;\n    #[cfg(feature = \"n\")]\n    use new_msgs::Leaf;", "Leaf"),
             Some("new_msgs/Leaf"), vec![], None),
            ("…not through a narrow crate glob (use crate::helpers::*)",
             nested("use old_msgs::Leaf;\nmod helpers {}", "use crate::helpers::*;\n    #[cfg(feature = \"n\")]\n    use new_msgs::Leaf;", "Leaf"),
             Some("new_msgs/Leaf"), vec![], None),
            ("a conditional direct glob beside a glob the parser cannot read into: undecidable",
             nested("use old_msgs::Leaf;", "#[cfg(feature = \"a\")]\n    use super::*;\n    #[cfg(not(feature = \"a\"))]\n    use new_msgs::*;", "Leaf"),
             None, vec![], Some("a conditional glob binds this name")),
            ("a conditional direct glob alone names the one configuration that has the leaf",
             nested("use old_msgs::Leaf;", "#[cfg(feature = \"a\")]\n    use super::*;", "Leaf"),
             Some("old_msgs/Leaf"), vec![], None),
            ("two cfg-exclusive direct globs binding differently carry both names",
             "use cerulion_core::prelude::*;\nuse old_msgs::Leaf;\nmod mid {\n    use mid_msgs::Leaf;\n    mod inner {\n        #[cfg(feature = \"a\")]\n        use super::*;\n        #[cfg(not(feature = \"a\"))]\n        use super::super::*;\n\n        #[cerulion_node]\n        #[derive(Default)]\n        struct SinkNode {\n            #[input(trigger)]\n            frame: Leaf,\n        }\n    }\n}\n".to_string(),
             Some("mid_msgs/Leaf"), vec!["old_msgs/Leaf"], None),
            ("…and through use crate::*",
             nested("use old_msgs::Leaf;", "use crate::*;\n    #[cfg(feature = \"n\")]\n    use new_msgs::Leaf;", "Leaf"),
             Some("new_msgs/Leaf"), vec!["old_msgs/Leaf"], None),
            // ---- the CURRENT scope: `self::` / `crate::` at the root ----
            ("use self::Leaf of this scope's own item is bare — the codegen shape",
             root("pub use self::TrackArray;", "TrackArray"),
             Some("TrackArray"), vec![], None),
            ("a current-scope alias of an existing binding resolves through it",
             root(&format!("pub use {NRM};\npub use self::Image as Frame;"), "Frame"),
             Some("sensor_msgs/Image"), vec![], None),
            ("…and by the crate-root spelling",
             root(&format!("pub use {NRM};\npub use crate::Image as Frame;"), "Frame"),
             Some("sensor_msgs/Image"), vec![], None),
            ("a current-scope alias of an item no `use` binds, under the prelude glob, is undecidable — as the port's own type would be",
             root("pub use self::TrackArray as Frame;", "Frame"),
             None, vec![], Some("names an item of this scope that no explicit `use` binds while a glob import is in scope — its schema name cannot be derived; declare the type in a module and import it by name")),
            ("…and with no glob in scope it is the codegen shape, bare",
             "pub use self::TrackArray as Frame;\n\n#[cerulion_node]\n#[derive(Default)]\nstruct SinkNode {\n    #[input(trigger)]\n    frame: Frame,\n}\n".to_string(),
             Some("TrackArray"), vec![], None),
            ("a mutual pair of current-scope aliases terminates, bare",
             root("pub use self::Frame as Image;\npub use self::Image as Frame;", "Image"),
             Some("Image"), vec![], None),
            // ---- an ancestor module reached through a #[cfg]-gated glob ----
            ("an ancestor module reached only through a #[cfg] glob is decided by that cfg",
             nested("mod detections { include!(concat!(env!(\"OUT_DIR\"), \"/d.rs\")); }", "#[cfg(feature = \"n\")]\n    use super::*;\n    use detections::TrackArray;", "TrackArray"),
             Some("TrackArray"), vec!["detections/TrackArray"], None),
            ("…a glob that can never exist reaches nothing: the path is the extern crate",
             nested("mod detections { include!(concat!(env!(\"OUT_DIR\"), \"/d.rs\")); }", "#[cfg(any())]\n    use super::*;\n    use detections::TrackArray;", "TrackArray"),
             Some("detections/TrackArray"), vec![], None),
            ("…a conditional glob FIRST, an unconditional one after it: still every build",
             nested("mod detections { include!(concat!(env!(\"OUT_DIR\"), \"/d.rs\")); }", "#[cfg(feature = \"n\")]\n    use super::*;\n    use super::*;\n    use detections::TrackArray;", "TrackArray"),
             Some("TrackArray"), vec![], None),
            ("…an unconditional glob beside the conditional one reaches it in every build",
             nested("mod detections { include!(concat!(env!(\"OUT_DIR\"), \"/d.rs\")); }", "use super::*;\n    #[cfg(feature = \"n\")]\n    use super::*;\n    use detections::TrackArray;", "TrackArray"),
             Some("TrackArray"), vec![], None),
            ("…the same through crate::* and an out-of-line ancestor module",
             nested("mod schema_types;", "#[cfg(feature = \"n\")]\n    use crate::*;\n    use schema_types::TrackArray;", "TrackArray"),
             Some("TrackArray"), vec!["schema_types/TrackArray"], None),
            ("…a cfg-gated ancestor MODULE reached through a plain glob is decided by the module's cfg",
             nested("#[cfg(feature = \"v\")]\nmod detections { include!(concat!(env!(\"OUT_DIR\"), \"/d.rs\")); }", "use super::*;\n    use detections::TrackArray;", "TrackArray"),
             Some("TrackArray"), vec!["detections/TrackArray"], None),
            ("…a farther reach that also declares the module makes the name crate-internal in every build",
             "use cerulion_core::prelude::*;\nmod detections { include!(concat!(env!(\"OUT_DIR\"), \"/d.rs\")); }\nmod mid {\n    mod detections { include!(concat!(env!(\"OUT_DIR\"), \"/m.rs\")); }\n    mod inner {\n        #[cfg(feature = \"a\")]\n        use super::*;\n        #[cfg(not(feature = \"a\"))]\n        use super::super::*;\n        use detections::TrackArray;\n\n        #[cerulion_node]\n        #[derive(Default)]\n        struct SinkNode {\n            #[input(trigger)]\n            frame: TrackArray,\n        }\n    }\n}\n".to_string(),
             Some("TrackArray"), vec![], None),
            ("…a glob the parser cannot read into never binds a MODULE name: the cfg-off answer stays the extern crate",
             nested("mod detections { include!(concat!(env!(\"OUT_DIR\"), \"/d.rs\")); }", "use vendor::prelude::*;\n    #[cfg(feature = \"n\")]\n    use super::*;\n    use detections::TrackArray;", "TrackArray"),
             Some("TrackArray"), vec!["detections/TrackArray"], None),
            // ---- a `use` binding a MODULE name as a path root (the merged fixtures) ----
            ("a root `use crate_::foo;` then `use crate::foo::Type;` below names the crate's module",
             nested("use external_crate::foo;", "use crate::foo::Type;", "Type"),
             Some("external_crate/foo/Type"), vec![], None),
            ("…and through `super::`",
             nested("use external_crate::foo;", "mod foo {}\n    use super::foo::Type;", "Type"),
             Some("external_crate/foo/Type"), vec![], None),
            ("…and in the same scope (`use other::foo; use foo::Type;`)",
             root("use other::foo;\nuse foo::Type;", "Type"),
             Some("other/foo/Type"), vec![], None),
            ("…a module import of a LOCAL module walks into it",
             nested("mod detections { include!(concat!(env!(\"OUT_DIR\"), \"/d.rs\")); }", "use super::detections;\n    use detections::TrackArray;", "TrackArray"),
             Some("TrackArray"), vec![], None),
            ("…a knot of module imports is undecidable, loudly",
             root("use self::b as a;\nuse self::a as b;\nuse a::Type;", "Type"),
             None, vec![], Some("a cyclic chain of module imports")),
            // ---- module imports: cfg, visibility, `{self}`, globs ----
            ("…an explicit module import shadows a glob-reached ancestor module, as in rustc",
             nested("mod detections { include!(concat!(env!(\"OUT_DIR\"), \"/d.rs\")); }", "use super::*;\n    use other::detections;\n    use detections::TrackArray;", "TrackArray"),
             Some("other/detections/TrackArray"), vec![], None),
            ("…a #[cfg]-gated module import beside the extern prelude carries both names",
             root("#[cfg(feature = \"v\")]\nuse vendor::foo;\nuse foo::Type;", "Type"),
             Some("vendor/foo/Type"), vec!["foo/Type"], None),
            ("…a cfg-exclusive pair of module imports carries every name, and the prelude's",
             root("#[cfg(feature = \"a\")]\nuse vendor_a::foo;\n#[cfg(not(feature = \"a\"))]\nuse vendor_b::foo;\nuse foo::Type;", "Type"),
             Some("vendor_a/foo/Type"), vec!["vendor_b/foo/Type", "foo/Type"], None),
            ("…a plain module import beside a gated one: the first stands, the other is an alternative",
             root("#[cfg(feature = \"a\")]\nuse vendor_a::foo;\nuse vendor_b::foo;\nuse foo::Type;", "Type"),
             Some("vendor_a/foo/Type"), vec!["vendor_b/foo/Type"], None),
            ("…a gated module import beside a glob-reached ancestor module: the cfg-off name is the ancestor's, not the prelude's",
             nested("mod detections { include!(concat!(env!(\"OUT_DIR\"), \"/d.rs\")); }", "use super::*;\n    #[cfg(feature = \"v\")]\n    use vendor::detections;\n    use detections::TrackArray;", "TrackArray"),
             Some("vendor/detections/TrackArray"), vec!["TrackArray"], None),
            ("…a PRIVATE module import in a module off the chain is no re-export (E0603): bare",
             root("mod m { use vendor::foo; }\nuse crate::m::foo::Type;", "Type"),
             Some("Type"), vec![], None),
            ("…and a `pub use` of the module is",
             root("mod m { pub use vendor::foo; }\nuse crate::m::foo::Type;", "Type"),
             Some("vendor/foo/Type"), vec![], None),
            ("…`{self as x}` names the module itself",
             root("use vendor::helpers::{self as h};\nuse h::Type;", "Type"),
             Some("vendor/helpers/Type"), vec![], None),
            ("…and `{self, Thing}` binds the module beside its item",
             nested("mod helpers { include!(concat!(env!(\"OUT_DIR\"), \"/h.rs\")); }", "use crate::helpers::{self, Thing};\n    use helpers::Type;", "Type"),
             Some("Type"), vec![], None),
            ("…`use foo;` of the crate itself binds nothing new: the extern crate",
             root("use foo;\nuse foo::Type;", "Type"),
             Some("foo/Type"), vec![], None),
        ];
        let undecidable = |lines: &[&str]| {
            lines
                .iter()
                .filter(|l| {
                    l.contains("re-exports through a glob")
                        || l.contains("cyclic")
                        || l.contains("could not decide")
                        || l.contains("while a glob import is in scope")
                        || l.contains("a conditional glob binds this name")
                        || l.contains("a cyclic chain of module imports")
                })
                .count()
        };
        for (i, (row, src, schema, alternatives, warn)) in rows.iter().enumerate() {
            let before = std::cell::Cell::new(0);
            logs_assert(|lines: &[&str]| {
                before.set(undecidable(lines));
                Ok(())
            });
            let node_dir = write_node(&tmp, &format!("row{i}"), src);
            let port = parse_node_metadata(&node_dir)
                .unwrap_or_else(|e| panic!("row {i} ({row}): {e}"))
                .inputs
                .remove(0);
            assert_eq!(port.schema.as_deref(), *schema, "row {i}: {row}");
            assert_eq!(port.schema_alternatives, *alternatives, "row {i}: {row}");
            let after = std::cell::Cell::new(0);
            let said = std::cell::Cell::new(false);
            logs_assert(|lines: &[&str]| {
                after.set(undecidable(lines));
                if let Some(needle) = warn {
                    said.set(
                        lines[lines.len().saturating_sub(8)..]
                            .iter()
                            .any(|l| l.contains(needle)),
                    );
                }
                Ok(())
            });
            match warn {
                Some(needle) => assert!(
                    after.get() > before.get() && said.get(),
                    "row {i}: {row} — expected an undecidable warn containing {needle:?}"
                ),
                None => assert_eq!(
                    after.get(),
                    before.get(),
                    "row {i}: {row} — no undecidable warn expected"
                ),
            }
        }
    }

    /// A port imported through a LOCAL RE-EXPORT carries the re-export's
    /// TARGET (without following the re-export, `mod aliases { pub
    /// use native_ros2_messages::sensor_msgs::Image; }` + `use
    /// aliases::Image` reports as the bare `Image` — a workspace
    /// `Image` where one exists, no schema at all for a vendor crate's type).
    /// Every shape, each against a hand oracle: the plain re-export; a
    /// rename INSIDE the module; a rename at the node; a vendor crate's type
    /// (keeps its package); a nested path through inline sub-modules; a
    /// cfg-gated re-export module (target + the crate spelling as the gated
    /// alternative); a module reached from a NESTED node through `use
    /// super::*` — and without the glob, the same spelling is the external
    /// crate, as rustc would resolve it. And the two shapes that stay as
    /// they were: a module with NO `pub use` of the leaf is the codegen
    /// shape (bare), and a PRIVATE `use` inside the module is not a
    /// re-export (E0603) — bare too.
    #[tracing_test::traced_test]
    #[test]
    fn a_local_re_export_carries_its_targets_schema_name() {
        let tmp = TempDir::new().unwrap();
        let parse = |name: &str, body: &str, field_ty: &str| {
            let src = format!(
                "use cerulion_core::prelude::*;\n{body}\n\n#[cerulion_node]\n#[derive(Default)]\n\
                 struct SinkNode {{\n    #[input(trigger)]\n    frame: {field_ty},\n}}\n"
            );
            parse_node_metadata(&write_node(&tmp, name, &src))
                .unwrap()
                .inputs
                .remove(0)
        };
        let port = parse(
            "plain",
            "mod aliases { pub use native_ros2_messages::sensor_msgs::Image; }\nuse aliases::Image;",
            "Image",
        );
        assert_eq!(port.schema.as_deref(), Some("sensor_msgs/Image"));
        assert!(port.schema_alternatives.is_empty());

        let port = parse(
            "renamed_inside",
            "mod aliases { pub use native_ros2_messages::sensor_msgs::Image as Img; }\nuse aliases::Img;",
            "Img",
        );
        assert_eq!(port.schema.as_deref(), Some("sensor_msgs/Image"));

        let port = parse(
            "renamed_at_node",
            "mod aliases { pub use native_ros2_messages::sensor_msgs::Image; }\nuse aliases::Image as Frame;",
            "Frame",
        );
        assert_eq!(port.schema.as_deref(), Some("sensor_msgs/Image"));

        let port = parse(
            "vendor",
            "mod aliases { pub use unitree_go::Go2FrontVideoData; }\nuse aliases::Go2FrontVideoData;",
            "Go2FrontVideoData",
        );
        assert_eq!(
            port.schema.as_deref(),
            Some("unitree_go/Go2FrontVideoData"),
            "a vendor crate's type keeps its package through the re-export"
        );

        let port = parse(
            "nested_path",
            "mod aliases { pub mod deep { pub use native_ros2_messages::sensor_msgs::Image; } }\n\
             use aliases::deep::Image;",
            "Image",
        );
        assert_eq!(port.schema.as_deref(), Some("sensor_msgs/Image"));

        let port = parse(
            "gated",
            "#[cfg(feature = \"vendored\")]\nmod aliases { pub use native_ros2_messages::sensor_msgs::Image; }\n\
             use aliases::Image;",
            "Image",
        );
        assert_eq!(port.schema.as_deref(), Some("sensor_msgs/Image"));
        assert_eq!(
            port.schema_alternatives,
            ["aliases/Image"],
            "a gated re-export module keeps the crate spelling as its other configuration"
        );

        // A nested node reaches a ROOT module through `use super::*` …
        let nested = |name: &str, glob: &str| {
            let src = format!(
                "use cerulion_core::prelude::*;\nmod aliases {{ pub use native_ros2_messages::sensor_msgs::Image; }}\n\
                 mod inner {{\n    {glob}\n    use aliases::Image;\n\n    #[cerulion_node]\n    #[derive(Default)]\n\
                 struct SinkNode {{\n        #[input(trigger)]\n        frame: Image,\n    }}\n}}\n"
            );
            parse_node_metadata(&write_node(&tmp, name, &src))
                .unwrap()
                .inputs
                .remove(0)
        };
        assert_eq!(
            nested("ancestor_via_glob", "use super::*;")
                .schema
                .as_deref(),
            Some("sensor_msgs/Image"),
            "`use super::*` brings the root's `mod aliases` into the nested scope"
        );
        // … and without the glob the same spelling is the external crate
        // `aliases` (the root module is not in the nested scope).
        assert_eq!(
            nested("ancestor_without_glob", "").schema.as_deref(),
            Some("aliases/Image")
        );

        // The codegen shape, unchanged: no `pub use` of the leaf ⇒ bare.
        let port = parse(
            "codegen",
            "mod detections { include!(concat!(env!(\"OUT_DIR\"), \"/detections.rs\")); }\n\
             use detections::DetectionArray;",
            "DetectionArray",
        );
        assert_eq!(port.schema.as_deref(), Some("DetectionArray"));
        // A PRIVATE `use` inside the module is not a re-export.
        let port = parse(
            "private_use",
            "mod aliases { use native_ros2_messages::sensor_msgs::Image; }\nuse aliases::Image;",
            "Image",
        );
        assert_eq!(port.schema.as_deref(), Some("Image"));
        assert!(
            !logs_contain("re-exports through a glob"),
            "no shape above is a glob re-export"
        );
    }

    /// A GLOB re-export is UNDECIDABLE — said loudly, no schema recorded,
    /// never a bare guess; and a port type no explicit `use` names while a
    /// glob import is in scope at the node is reported the same way (a bare
    /// `use foo::*;` must not leave such a port silently unnamed).
    #[tracing_test::traced_test]
    #[test]
    fn a_glob_re_export_or_import_is_undecidable_and_said() {
        let tmp = TempDir::new().unwrap();
        let src = "use cerulion_core::prelude::*;\nmod aliases { pub use native_ros2_messages::sensor_msgs::*; }\n\
                   use aliases::Image;\n\n#[cerulion_node]\n#[derive(Default)]\nstruct SinkNode {\n    #[input(trigger)]\n    frame: Image,\n}\n";
        let port = parse_node_metadata(&write_node(&tmp, "glob_reexport", src))
            .unwrap()
            .inputs
            .remove(0);
        assert_eq!(port.schema, None, "a glob re-export decides nothing");
        assert!(
            logs_contain("a local module re-exports through a glob")
                && logs_contain("module=aliases"),
            "…and says so, naming the module"
        );

        let src = "use cerulion_core::prelude::*;\nuse vendor_msgs::*;\n\n#[cerulion_node]\n#[derive(Default)]\n\
                   struct SinkNode {\n    #[input(trigger)]\n    frame: VendorFrame,\n}\n";
        let port = parse_node_metadata(&write_node(&tmp, "glob_import", src))
            .unwrap()
            .inputs
            .remove(0);
        assert_eq!(port.schema, None);
        assert!(
            logs_contain("while a glob import is in scope")
                && logs_contain("port=frame")
                && logs_contain("vendor_msgs"),
            "an unnamed port type under a glob import is said, naming the port and the glob"
        );
    }

    /// A rename at the node's own scope is keyed by the name the field sees.
    #[test]
    fn a_renamed_import_is_keyed_by_the_name_the_field_sees() {
        let tmp = TempDir::new().unwrap();
        let src = "use cerulion_core::prelude::*;\nuse native_ros2_messages::sensor_msgs::Image as Img;\n\n\
                   #[cerulion_node]\n#[derive(Default)]\nstruct SinkNode {\n    #[input(trigger)]\n    frame: Img,\n}\n";
        let port = parse_node_metadata(&write_node(&tmp, "renamed", src))
            .unwrap()
            .inputs
            .remove(0);
        assert_eq!(port.schema.as_deref(), Some("sensor_msgs/Image"));
    }

    /// An ANCESTOR scope's import is a cfg alternative of a nearer
    /// conditional import ONLY through a glob that reaches it: without
    /// `use super::*` the cfg-off configuration has no
    /// `Leaf` in the nested scope at all (rustc: E0425), so `old_msgs/Leaf`
    /// is a name the built node can never carry and `graph validate` must
    /// not accept it. With the glob, the ancestor's import IS in force with
    /// the cfg off, and the pair carries both names. Dropping the
    /// glob's reach (a narrow glob made to reach the parent) fails the first arm.
    #[tracing_test::traced_test]
    #[test]
    fn an_ancestor_import_is_a_cfg_alternative_only_through_a_glob() {
        let tmp = TempDir::new().unwrap();
        let parse = |name: &str, glob: &str| {
            let src = format!(
                "use cerulion_core::prelude::*;\nuse old_msgs::Leaf;\n\nmod inner {{\n    {glob}\n    \
                 #[cfg(feature = \"new\")]\n    use new_msgs::Leaf;\n\n    #[cerulion_node]\n    #[derive(Default)]\n    \
                 struct SinkNode {{\n        #[input(trigger)]\n        leaf: Leaf,\n    }}\n}}\n"
            );
            parse_node_metadata(&write_node(&tmp, name, &src))
                .unwrap()
                .inputs
                .remove(0)
        };
        let port = parse("no_glob", "");
        assert_eq!(port.schema.as_deref(), Some("new_msgs/Leaf"));
        assert!(
            port.schema_alternatives.is_empty(),
            "an ancestor import the nested scope cannot reach is not an alternative"
        );
        assert!(!logs_contain("shadows an ancestor scope's import"));

        let port = parse("with_glob", "use super::*;");
        assert_eq!(port.schema.as_deref(), Some("new_msgs/Leaf"));
        assert_eq!(port.schema_alternatives, ["old_msgs/Leaf"]);
        assert!(logs_contain("shadows an ancestor scope's import"));
    }

    /// A gated module named after the codegen
    /// ANCHOR (`mod native_ros2_messages`) shadowing a root-level type makes
    /// both configurations publish the same bare name — nothing is
    /// ambiguous, so no alternative is recorded and nothing is said.
    #[tracing_test::traced_test]
    #[test]
    fn a_gated_module_whose_alternative_is_its_own_name_is_not_ambiguous() {
        let tmp = TempDir::new().unwrap();
        let node_dir = write_node(
            &tmp,
            "anchor",
            r#"
use cerulion_core::prelude::*;
#[cfg(feature = "vendored")]
mod native_ros2_messages {
    include!(concat!(env!("OUT_DIR"), "/vendored.rs"));
}
use native_ros2_messages::Image;

#[cerulion_node]
#[derive(Default)]
struct SinkNode {
    #[input(trigger)]
    frame: Image,
}
"#,
        );
        let port = parse_node_metadata(&node_dir).unwrap().inputs.remove(0);
        assert_eq!(port.schema.as_deref(), Some("Image"));
        assert!(
            port.schema_alternatives.is_empty(),
            "the other configuration says the same name"
        );
        assert!(
            !logs_contain("decides this port's schema name"),
            "nothing ambiguous, nothing to warn about"
        );
    }

    // ---- a same-file module is not a schema package qualifier: these tests run
    // the per-leaf walk over paths through same-file modules. ELEVEN carry an
    // assertion message that explains why the expected value is what it
    // is — see each message. Seven because the fixture is a
    // program rustc rejects (E0432: an extern crate or an out-of-line module would
    // have to exist, or a `use` names its own item) and the walk reports what a
    // well-formed reading of the path names; four because a `use` binding a MODULE
    // name continues the path through it and a non-ROS external type is
    // spelled with its full crate path: `child_import_shadows_ancestor_local_module`,
    // its `_regardless_of_order` twin, and the two
    // `*_anchor_uses_target_scope_import_for_locality`. The two `child_import_
    // shadows_*` NAMES are loose — nothing is shadowed, the child's import
    // binds the name outright, as their messages say. ----

    #[test]
    fn parse_macro_node_local_inline_module_import_is_bare() {
        let tmp = TempDir::new().unwrap();
        let src = r#"
use cerulion_core::prelude::*;

mod detections {
    // The generated package-less workspace schema is included here.
}
use detections::DetectionArray;

#[cerulion_node(period_ms = 50)]
#[derive(Default)]
struct DetectorNode {
    #[output]
    detections: DetectionArray,
}
"#;
        let node_dir = write_node(&tmp, "detector", src);
        let metadata = parse_node_metadata(&node_dir).unwrap();
        assert_eq!(
            metadata.outputs[0].schema.as_deref(),
            Some("DetectionArray"),
            "a local module is not a schema package"
        );
    }

    #[test]
    fn parse_macro_node_local_external_module_import_is_bare() {
        let tmp = TempDir::new().unwrap();
        let src = r#"
use cerulion_core::prelude::*;

mod detections;
use detections::DetectionArray;

#[cerulion_node(period_ms = 50)]
#[derive(Default)]
struct DetectorNode {
    #[output]
    detections: DetectionArray,
}
"#;
        let node_dir = write_node(&tmp, "detector", src);
        let metadata = parse_node_metadata(&node_dir).unwrap();
        assert_eq!(
            metadata.outputs[0].schema.as_deref(),
            Some("DetectionArray"),
            "an external local module declaration is not a schema package"
        );
    }

    #[test]
    fn parse_macro_node_local_module_wins_package_name_collision() {
        let tmp = TempDir::new().unwrap();
        let src = r#"
use cerulion_core::prelude::*;

mod user_msgs {}
use user_msgs::Image;

#[cerulion_node(period_ms = 50)]
#[derive(Default)]
struct CameraNode {
    #[output]
    image: Image,
}
"#;
        let node_dir = write_node(&tmp, "camera", src);
        let metadata = parse_node_metadata(&node_dir).unwrap();
        assert_eq!(
            metadata.outputs[0].schema.as_deref(),
            Some("Image"),
            "a local module must win over package-name interpretation"
        );
    }

    #[test]
    fn parse_macro_node_nested_module_does_not_hide_root_package_qualifier() {
        let tmp = TempDir::new().unwrap();
        let src = r#"
use cerulion_core::prelude::*;

mod outer {
    mod vendor {}
}
use vendor::Image;

#[cerulion_node(period_ms = 50)]
#[derive(Default)]
struct CameraNode {
    #[output]
    image: Image,
}
"#;
        let node_dir = write_node(&tmp, "camera", src);
        let metadata = parse_node_metadata(&node_dir).unwrap();
        assert_eq!(
            metadata.outputs[0].schema.as_deref(),
            Some("vendor/Image"),
            "a module nested under another module is not a root-level local module"
        );
    }

    #[test]
    fn parse_macro_node_local_sibling_module_import_is_bare() {
        let tmp = TempDir::new().unwrap();
        let src = r#"
use cerulion_core::prelude::*;

mod messages {
    mod detections {}
    use detections::DetectionArray;

    #[cerulion_node(period_ms = 50)]
    #[derive(Default)]
    pub struct DetectorNode {
        #[output]
        detections: DetectionArray,
    }
}
"#;
        let node_dir = write_node(&tmp, "detector", src);
        let metadata = parse_node_metadata(&node_dir).unwrap();
        assert_eq!(
            metadata.outputs[0].schema.as_deref(),
            Some("DetectionArray"),
            "a direct sibling module is a local package-less schema namespace"
        );
    }

    #[test]
    fn parse_macro_node_root_import_is_not_overwritten_by_unrelated_module() {
        let tmp = TempDir::new().unwrap();
        let src = r#"
use cerulion_core::prelude::*;
use native_ros2_messages::sensor_msgs::Image;

#[cerulion_node(period_ms = 50)]
#[derive(Default)]
struct CameraNode {
    #[output]
    image: Image,
}

mod other {
    use user_msgs::Image;
}
"#;
        let node_dir = write_node(&tmp, "root_camera", src);
        let metadata = parse_node_metadata(&node_dir).unwrap();
        assert_eq!(
            metadata.outputs[0].schema.as_deref(),
            Some("sensor_msgs/Image"),
            "an unrelated module import must not overwrite the root node's schema"
        );
    }

    #[test]
    fn parse_macro_node_local_module_named_like_ros_package_does_not_hide_ros_import() {
        let tmp = TempDir::new().unwrap();
        let src = r#"
use cerulion_core::prelude::*;
use native_ros2_messages::sensor_msgs::Image;

mod sensor_msgs {}

#[cerulion_node(period_ms = 50)]
#[derive(Default)]
struct CameraNode {
    #[output]
    image: Image,
}
"#;
        let node_dir = write_node(&tmp, "sensor_camera", src);
        let metadata = parse_node_metadata(&node_dir).unwrap();
        assert_eq!(
            metadata.outputs[0].schema.as_deref(),
            Some("sensor_msgs/Image"),
            "a local module named like a ROS package must not hide the crate qualifier"
        );
    }

    #[test]
    fn parse_macro_node_inner_import_overrides_root_import() {
        let tmp = TempDir::new().unwrap();
        let src = r#"
use cerulion_core::prelude::*;
use user_msgs::Image;

mod inner {
    use super::*;
    use native_ros2_messages::sensor_msgs::Image;

    #[cerulion_node(period_ms = 50)]
    #[derive(Default)]
    struct CameraNode {
        #[output]
        image: Image,
    }
}
"#;
        let node_dir = write_node(&tmp, "inner_camera", src);
        let metadata = parse_node_metadata(&node_dir).unwrap();
        assert_eq!(
            metadata.outputs[0].schema.as_deref(),
            Some("sensor_msgs/Image"),
            "the selected node's innermost import must win"
        );
    }

    #[test]
    fn parse_macro_node_ancestor_module_is_local_in_nested_scope() {
        let tmp = TempDir::new().unwrap();
        let src = r#"
use cerulion_core::prelude::*;

mod detections {
    pub struct DetectionArray;
}

mod inner {
    use detections::DetectionArray;

    #[cerulion_node(period_ms = 50)]
    #[derive(Default)]
    struct DetectorNode {
        #[output]
        detections: DetectionArray,
    }
}
"#;
        let node_dir = write_node(&tmp, "nested_detector", src);
        let metadata = parse_node_metadata(&node_dir).unwrap();
        assert_eq!(
            metadata.outputs[0].schema.as_deref(),
            Some("detections/DetectionArray"),
            "with no glob, `use detections::…` inside `mod inner` \
             names no item of `inner` — rustc (2018 uniform paths) resolves the first segment in the \
             current module and the extern prelude, so this program is E0432 unless an extern \
             crate `detections` exists, and THAT is what the port reports; an import map \
             that never modelled scopes would answer bare"
        );
    }

    #[test]
    fn parse_macro_node_child_import_shadows_ancestor_local_module() {
        let tmp = TempDir::new().unwrap();
        let src = r#"
use cerulion_core::prelude::*;

mod user_msgs {}

mod inner {
    use other_msgs::user_msgs;
    use user_msgs::Image;

    #[cerulion_node(period_ms = 50)]
    #[derive(Default)]
    struct CameraNode {
        #[output]
        image: Image,
    }
}
"#;
        let node_dir = write_node(&tmp, "child_import_camera", src);
        let metadata = parse_node_metadata(&node_dir).unwrap();
        assert_eq!(
            metadata.outputs[0].schema.as_deref(),
            Some("other_msgs/user_msgs/Image"),
            "the child's `use other_msgs::user_msgs;` binds the name, so \
             `use user_msgs::Image` names `other_msgs::user_msgs::Image` — spelled with its full \
             crate path, the rule for every non-ROS external type"
        );
    }

    #[test]
    fn parse_macro_node_child_import_shadows_ancestor_local_module_regardless_of_order() {
        let tmp = TempDir::new().unwrap();
        let src = r#"
use cerulion_core::prelude::*;

mod user_msgs {}

mod inner {
    use user_msgs::Image;
    use other_msgs::user_msgs;

    #[cerulion_node(period_ms = 50)]
    #[derive(Default)]
    struct CameraNode {
        #[output]
        image: Image,
    }
}
"#;
        let node_dir = write_node(&tmp, "reverse_child_import_camera", src);
        let metadata = parse_node_metadata(&node_dir).unwrap();
        assert_eq!(
            metadata.outputs[0].schema.as_deref(),
            Some("other_msgs/user_msgs/Image"),
            "whichever `use` comes first, the child's `use \
             other_msgs::user_msgs;` binds the name and `use user_msgs::Image` names \
             `other_msgs::user_msgs::Image`, spelled with its full crate path"
        );
    }

    #[test]
    fn parse_macro_node_sibling_scope_retains_ancestor_local_module() {
        let tmp = TempDir::new().unwrap();
        let src = r#"
use cerulion_core::prelude::*;

mod user_msgs {}

mod child_with_import {
    use other_msgs::user_msgs;
}

mod sibling {
    use user_msgs::Image;

    #[cerulion_node(period_ms = 50)]
    #[derive(Default)]
    struct CameraNode {
        #[output]
        image: Image,
    }
}
"#;
        let node_dir = write_node(&tmp, "sibling_camera", src);
        let metadata = parse_node_metadata(&node_dir).unwrap();
        assert_eq!(
            metadata.outputs[0].schema.as_deref(),
            Some("user_msgs/Image"),
            "`mod sibling` reaches the root's `user_msgs` through \
             no glob and no `super::`/`crate::` — rustc resolves the first segment in the current \
             module and the extern prelude (E0432 otherwise), so the spelling names an extern crate \
             `user_msgs`, which is what the port reports"
        );
    }

    #[test]
    fn parse_macro_node_ancestor_import_does_not_shadow_root_local_module() {
        let tmp = TempDir::new().unwrap();
        let src = r#"
use cerulion_core::prelude::*;

mod user_msgs {}

mod outer {
    use other_msgs::user_msgs;

    mod inner {
        use user_msgs::Image;

        #[cerulion_node(period_ms = 50)]
        #[derive(Default)]
        struct CameraNode {
            #[output]
            image: Image,
        }
    }
}
"#;
        let node_dir = write_node(&tmp, "ancestor_import_camera", src);
        let metadata = parse_node_metadata(&node_dir).unwrap();
        assert_eq!(
            metadata.outputs[0].schema.as_deref(),
            Some("user_msgs/Image"),
            "`inner` reaches neither the root's `mod user_msgs` nor \
             `outer`'s import of that name — no glob, no `super::` — so rustc resolves `user_msgs` in \
             the extern prelude (E0432 otherwise), and that is what the port reports"
        );
    }

    #[test]
    fn parse_macro_node_super_local_module_reimport_remains_bare() {
        let tmp = TempDir::new().unwrap();
        let src = r#"
use cerulion_core::prelude::*;

mod user_msgs {}

mod outer {
    mod inner {
        use super::user_msgs;
        use user_msgs::Image;

        #[cerulion_node(period_ms = 50)]
        #[derive(Default)]
        struct CameraNode {
            #[output]
            image: Image,
        }
    }
}
"#;
        let node_dir = write_node(&tmp, "super_local_reimport_camera", src);
        let metadata = parse_node_metadata(&node_dir).unwrap();
        assert_eq!(
            metadata.outputs[0].schema.as_deref(),
            Some("Image"),
            "`use super::user_msgs;` binds the name, and the path \
             continues through it to `super::user_msgs::Image` — a sub-module `outer` does not \
             declare inline (rustc: E0432 unless `mod user_msgs;` exists out of line), whose body \
             the walk cannot read: the leaf is the codegen shape, bare"
        );
    }

    #[test]
    fn parse_macro_node_crate_anchored_local_module_import_is_bare() {
        let tmp = TempDir::new().unwrap();
        let src = r#"
use cerulion_core::prelude::*;

mod detections {}
use crate::detections::DetectionArray;

#[cerulion_node(period_ms = 50)]
#[derive(Default)]
struct DetectorNode {
    #[output]
    detections: DetectionArray,
}
"#;
        let node_dir = write_node(&tmp, "crate_detector", src);
        let metadata = parse_node_metadata(&node_dir).unwrap();
        assert_eq!(
            metadata.outputs[0].schema.as_deref(),
            Some("DetectionArray"),
            "crate-anchored local modules are package-less schemas"
        );
    }

    #[test]
    fn parse_macro_node_self_anchored_local_module_import_is_bare() {
        let tmp = TempDir::new().unwrap();
        let src = r#"
use cerulion_core::prelude::*;

mod detections {}
use self::detections::DetectionArray;

#[cerulion_node(period_ms = 50)]
#[derive(Default)]
struct DetectorNode {
    #[output]
    detections: DetectionArray,
}
"#;
        let node_dir = write_node(&tmp, "self_detector", src);
        let metadata = parse_node_metadata(&node_dir).unwrap();
        assert_eq!(
            metadata.outputs[0].schema.as_deref(),
            Some("DetectionArray"),
            "self-anchored local modules are package-less schemas"
        );
    }

    #[test]
    fn parse_macro_node_super_anchored_ancestor_module_import_is_bare() {
        let tmp = TempDir::new().unwrap();
        let src = r#"
use cerulion_core::prelude::*;

mod detections {}

mod inner {
    use super::detections::DetectionArray;

    #[cerulion_node(period_ms = 50)]
    #[derive(Default)]
    struct DetectorNode {
        #[output]
        detections: DetectionArray,
    }
}
"#;
        let node_dir = write_node(&tmp, "super_detector", src);
        let metadata = parse_node_metadata(&node_dir).unwrap();
        assert_eq!(
            metadata.outputs[0].schema.as_deref(),
            Some("DetectionArray"),
            "super-anchored ancestor modules are package-less schemas"
        );
    }

    #[test]
    fn parse_macro_node_super_anchor_uses_target_scope_import_for_locality() {
        let tmp = TempDir::new().unwrap();
        let src = r#"
use cerulion_core::prelude::*;
use external_crate::foo;

mod inner {
    mod foo {}
    use super::foo::Type;

    #[cerulion_node(period_ms = 50)]
    #[derive(Default)]
    struct CameraNode {
        #[output]
        image: Type,
    }
}
"#;
        let node_dir = write_node(&tmp, "super_external_anchor", src);
        let metadata = parse_node_metadata(&node_dir).unwrap();
        assert_eq!(
            metadata.outputs[0].schema.as_deref(),
            Some("external_crate/foo/Type"),
            "`super::foo` from `inner` is the root's `use \
             external_crate::foo;` (its own `mod foo` is not `super`'s), so the type is \
             `external_crate::foo::Type`, spelled with its full crate path"
        );
    }

    #[test]
    fn parse_macro_node_super_anchor_uses_target_scope_module_for_locality() {
        let tmp = TempDir::new().unwrap();
        let src = r#"
use cerulion_core::prelude::*;

mod foo {}

mod inner {
    mod foo {}
    use super::foo::Type;

    #[cerulion_node(period_ms = 50)]
    #[derive(Default)]
    struct CameraNode {
        #[output]
        image: Type,
    }
}
"#;
        let node_dir = write_node(&tmp, "super_local_anchor", src);
        let metadata = parse_node_metadata(&node_dir).unwrap();
        assert_eq!(
            metadata.outputs[0].schema.as_deref(),
            Some("Type"),
            "super locality must use the selected parent scope's module set"
        );
    }

    #[test]
    fn parse_macro_node_crate_anchor_uses_target_scope_import_for_locality() {
        let tmp = TempDir::new().unwrap();
        let src = r#"
use cerulion_core::prelude::*;
use external_crate::foo;

mod inner {
    mod foo {}
    use crate::foo::Type;

    #[cerulion_node(period_ms = 50)]
    #[derive(Default)]
    struct CameraNode {
        #[output]
        image: Type,
    }
}
"#;
        let node_dir = write_node(&tmp, "crate_external_anchor", src);
        let metadata = parse_node_metadata(&node_dir).unwrap();
        assert_eq!(
            metadata.outputs[0].schema.as_deref(),
            Some("external_crate/foo/Type"),
            "`crate::foo` is the root's `use external_crate::foo;`, \
             so the type is `external_crate::foo::Type` — spelled with its full crate path, the \
             rule for every non-ROS external type (a user type never collides with a ROS 2 \
             schema of the same leaf)"
        );
    }

    #[test]
    fn parse_macro_node_crate_anchor_uses_target_scope_module_for_locality() {
        let tmp = TempDir::new().unwrap();
        let src = r#"
use cerulion_core::prelude::*;

mod foo {}

mod inner {
    mod foo {}
    use crate::foo::Type;

    #[cerulion_node(period_ms = 50)]
    #[derive(Default)]
    struct CameraNode {
        #[output]
        image: Type,
    }
}
"#;
        let node_dir = write_node(&tmp, "crate_local_anchor", src);
        let metadata = parse_node_metadata(&node_dir).unwrap();
        assert_eq!(
            metadata.outputs[0].schema.as_deref(),
            Some("Type"),
            "crate locality must use the root scope's module set"
        );
    }

    #[test]
    fn parse_macro_node_super_anchor_uses_target_scope_local_reimport_for_locality() {
        let tmp = TempDir::new().unwrap();
        let src = r#"
use cerulion_core::prelude::*;

mod user_msgs {}

mod outer {
    use super::user_msgs;

    mod inner {
        use super::user_msgs::Image;

        #[cerulion_node(period_ms = 50)]
        #[derive(Default)]
        struct CameraNode {
            #[output]
            image: Image,
        }
    }
}
"#;
        let node_dir = write_node(&tmp, "super_local_reimport", src);
        let metadata = parse_node_metadata(&node_dir).unwrap();
        assert_eq!(
            metadata.outputs[0].schema.as_deref(),
            Some("Image"),
            "a super anchor must follow the target scope's local re-import"
        );
    }

    #[test]
    fn parse_macro_node_crate_anchor_uses_target_scope_local_reimport_for_locality() {
        let tmp = TempDir::new().unwrap();
        let src = r#"
use cerulion_core::prelude::*;

mod user_msgs {}
use self::user_msgs;

mod outer {
    use crate::user_msgs::Image;

    #[cerulion_node(period_ms = 50)]
    #[derive(Default)]
    struct CameraNode {
        #[output]
        image: Image,
    }
}
"#;
        let node_dir = write_node(&tmp, "crate_local_reimport", src);
        let metadata = parse_node_metadata(&node_dir).unwrap();
        assert_eq!(
            metadata.outputs[0].schema.as_deref(),
            Some("Image"),
            "a crate anchor must follow the target scope's local re-import"
        );
    }

    #[test]
    fn parse_macro_node_super_anchored_reimport_preserves_ancestor_qualifier() {
        let tmp = TempDir::new().unwrap();
        let src = r#"
use cerulion_core::prelude::*;
use sensor_msgs::Image;

mod inner {
    use super::Image;

    #[cerulion_node(period_ms = 50)]
    #[derive(Default)]
    struct CameraNode {
        #[output]
        image: Image,
    }
}
"#;
        let node_dir = write_node(&tmp, "super_reimport_camera", src);
        let metadata = parse_node_metadata(&node_dir).unwrap();
        assert_eq!(
            metadata.outputs[0].schema.as_deref(),
            Some("sensor_msgs/Image"),
            "super-anchored re-imports preserve an ancestor qualifier"
        );
    }

    #[test]
    fn parse_macro_node_multi_super_reimport_uses_target_scope() {
        let tmp = TempDir::new().unwrap();
        let src = r#"
use cerulion_core::prelude::*;
use sensor_msgs::Image;

mod middle {
    use user_msgs::Image;

    mod inner {
        use super::super::Image;

        #[cerulion_node(period_ms = 50)]
        #[derive(Default)]
        struct CameraNode {
            #[output]
            image: Image,
        }
    }
}
"#;
        let node_dir = write_node(&tmp, "multi_super_reimport_camera", src);
        let metadata = parse_node_metadata(&node_dir).unwrap();
        assert_eq!(
            metadata.outputs[0].schema.as_deref(),
            Some("sensor_msgs/Image"),
            "a multi-super re-import resolves against its exact target scope"
        );
    }

    #[test]
    fn parse_macro_node_single_super_reimport_uses_immediate_parent() {
        let tmp = TempDir::new().unwrap();
        let src = r#"
use cerulion_core::prelude::*;
use sensor_msgs::Image;

mod middle {
    use user_msgs::Image;

    mod inner {
        use super::Image;

        #[cerulion_node(period_ms = 50)]
        #[derive(Default)]
        struct CameraNode {
            #[output]
            image: Image,
        }
    }
}
"#;
        let node_dir = write_node(&tmp, "single_super_reimport_camera", src);
        let metadata = parse_node_metadata(&node_dir).unwrap();
        assert_eq!(
            metadata.outputs[0].schema.as_deref(),
            Some("user_msgs/Image"),
            "a single-super re-import resolves against its immediate parent"
        );
    }

    #[test]
    fn parse_macro_node_self_anchored_reimport_preserves_ancestor_qualifier() {
        let tmp = TempDir::new().unwrap();
        let src = r#"
use cerulion_core::prelude::*;
use sensor_msgs::Image;

mod inner {
    use self::Image;

    #[cerulion_node(period_ms = 50)]
    #[derive(Default)]
    struct CameraNode {
        #[output]
        image: Image,
    }
}
"#;
        let node_dir = write_node(&tmp, "self_reimport_camera", src);
        let metadata = parse_node_metadata(&node_dir).unwrap();
        assert_eq!(
            metadata.outputs[0].schema.as_deref(),
            Some("Image"),
            "`use self::Image;` inside `inner` names an item of `inner` \
             itself, and the only such item is this very `use` — rustc rejects the self-reference \
             (E0432); the walk treats `use self::Leaf;` of the scope's own item as the codegen shape \
             and reports the bare leaf, never the ancestor's binding it does not name"
        );
    }

    #[test]
    fn parse_macro_node_self_anchored_local_reimport_is_bare() {
        let tmp = TempDir::new().unwrap();
        let src = r#"
use cerulion_core::prelude::*;

mod user_msgs {}

mod outer {
    use super::user_msgs;
    use self::user_msgs::Image;

    #[cerulion_node(period_ms = 50)]
    #[derive(Default)]
    struct CameraNode {
        #[output]
        image: Image,
    }
}
"#;
        let node_dir = write_node(&tmp, "self_local_reimport", src);
        let metadata = parse_node_metadata(&node_dir).unwrap();
        assert_eq!(
            metadata.outputs[0].schema.as_deref(),
            Some("Image"),
            "self-anchored local re-imports must remain package-less"
        );
    }

    #[test]
    fn parse_macro_node_self_anchored_external_path_keeps_qualifier() {
        let tmp = TempDir::new().unwrap();
        let src = r#"
use cerulion_core::prelude::*;

mod outer {
    use self::external_msgs::Image;

    #[cerulion_node(period_ms = 50)]
    #[derive(Default)]
    struct CameraNode {
        #[output]
        image: Image,
    }
}
"#;
        let node_dir = write_node(&tmp, "self_external_reimport", src);
        let metadata = parse_node_metadata(&node_dir).unwrap();
        assert_eq!(
            metadata.outputs[0].schema.as_deref(),
            Some("Image"),
            "`self::external_msgs` names a module OF `outer`, never \
             an extern crate (rustc: E0432 unless `mod external_msgs;` exists out of line) — the walk \
             cannot read an out-of-line module's body, so the leaf is the codegen shape, bare"
        );
    }

    #[test]
    fn parse_macro_node_crate_anchored_reimport_preserves_ancestor_qualifier() {
        let tmp = TempDir::new().unwrap();
        let src = r#"
use cerulion_core::prelude::*;
use sensor_msgs::Image;

mod inner {
    use crate::Image;

    #[cerulion_node(period_ms = 50)]
    #[derive(Default)]
    struct CameraNode {
        #[output]
        image: Image,
    }
}
"#;
        let node_dir = write_node(&tmp, "crate_reimport_camera", src);
        let metadata = parse_node_metadata(&node_dir).unwrap();
        assert_eq!(
            metadata.outputs[0].schema.as_deref(),
            Some("sensor_msgs/Image"),
            "crate-anchored re-imports preserve an ancestor qualifier"
        );
    }

    #[test]
    fn parse_macro_node_crate_reimport_uses_file_root_scope() {
        let tmp = TempDir::new().unwrap();
        let src = r#"
use cerulion_core::prelude::*;
use sensor_msgs::Image;

mod middle {
    use user_msgs::Image;

    mod inner {
        use crate::Image;

        #[cerulion_node(period_ms = 50)]
        #[derive(Default)]
        struct CameraNode {
            #[output]
            image: Image,
        }
    }
}
"#;
        let node_dir = write_node(&tmp, "crate_root_reimport_camera", src);
        let metadata = parse_node_metadata(&node_dir).unwrap();
        assert_eq!(
            metadata.outputs[0].schema.as_deref(),
            Some("sensor_msgs/Image"),
            "crate re-imports resolve against the file-root scope"
        );
    }

    #[test]
    fn parse_macro_node_super_anchored_reimport_without_ancestor_is_bare() {
        let tmp = TempDir::new().unwrap();
        let src = r#"
use cerulion_core::prelude::*;

mod inner {
    use super::Image;

    #[cerulion_node(period_ms = 50)]
    #[derive(Default)]
    struct CameraNode {
        #[output]
        image: Image,
    }
}
"#;
        let node_dir = write_node(&tmp, "bare_reimport_camera", src);
        let metadata = parse_node_metadata(&node_dir).unwrap();
        assert_eq!(
            metadata.outputs[0].schema.as_deref(),
            None,
            "the root declares no `Image` but holds a glob the walk \
             cannot read into (`use cerulion_core::prelude::*;`), which may bind it — undecidable, said \
             loudly, never a bare guess (rustc: E0432 if the glob does not)"
        );
    }

    #[test]
    fn parse_macro_node_super_reimport_beyond_nesting_is_bare() {
        let tmp = TempDir::new().unwrap();
        let src = r#"
use cerulion_core::prelude::*;
use sensor_msgs::Image;

mod middle {
    mod inner {
        use super::super::super::Image;

        #[cerulion_node(period_ms = 50)]
        #[derive(Default)]
        struct CameraNode {
            #[output]
            image: Image,
        }
    }
}
"#;
        let node_dir = write_node(&tmp, "too_deep_reimport_camera", src);
        let metadata = parse_node_metadata(&node_dir).unwrap();
        assert_eq!(
            metadata.outputs[0].schema.as_deref(),
            Some("Image"),
            "a super-chain beyond the crate nesting falls back to the bare name"
        );
    }

    #[test]
    fn parse_macro_node_global_import_keeps_external_package_qualifier() {
        let tmp = TempDir::new().unwrap();
        let src = r#"
use cerulion_core::prelude::*;

mod user_msgs {}
use ::user_msgs::Image;

#[cerulion_node(period_ms = 50)]
#[derive(Default)]
struct CameraNode {
    #[output]
    image: Image,
}
"#;
        let node_dir = write_node(&tmp, "global_camera", src);
        let metadata = parse_node_metadata(&node_dir).unwrap();
        assert_eq!(
            metadata.outputs[0].schema.as_deref(),
            Some("user_msgs/Image"),
            "a globally anchored external crate must not be treated as local"
        );
    }

    #[test]
    fn parse_macro_node_crate_anchor_without_module_is_unqualified() {
        let tmp = TempDir::new().unwrap();
        let src = r#"
use cerulion_core::prelude::*;
use crate::DetectionArray;

#[cerulion_node(period_ms = 50)]
#[derive(Default)]
struct DetectorNode {
    #[output]
    detections: DetectionArray,
}
"#;
        let node_dir = write_node(&tmp, "crate_root_detector", src);
        let metadata = parse_node_metadata(&node_dir).unwrap();
        assert_eq!(
            metadata.outputs[0].schema.as_deref(),
            Some("DetectionArray"),
            "a crate anchor without a module leaves the schema unqualified"
        );
    }
}
