// SPDX-License-Identifier: AGPL-3.0-only
//! Node commands: create, delete, modify, build, stage, run, list, info.

use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicU64, Ordering};

use cerulion_core::graph::node::{BackpressurePolicy, MacroPolicy};

use crate::error::{CliError, CliResult};
use crate::node_metadata::{parse_node_metadata, NodeMetadata, PortDef, PORT_SCHEMAS_MARKER};
use crate::schema_cmd;
use crate::system_deps;
use crate::templates;
use crate::utils;
use crate::workspace_lock::WorkspaceLock;

fn acquire_workspace_lock(dir: &Path) -> CliResult<(WorkspaceLock, std::path::PathBuf)> {
    let root = dir.parent().ok_or_else(|| {
        CliError::Validation(format!(
            "workspace directory has no parent: {}",
            dir.display()
        ))
    })?;
    let lock = WorkspaceLock::acquire_and_track_gitignore(root)?;
    let canonical_dir = match dir.canonicalize() {
        Ok(path) => path,
        Err(error) if error.kind() == std::io::ErrorKind::NotFound => {
            let parent = dir.parent().ok_or_else(|| {
                CliError::Validation(format!(
                    "workspace directory has no parent: {}",
                    dir.display()
                ))
            })?;
            let file_name = dir.file_name().ok_or_else(|| {
                CliError::Validation(format!(
                    "workspace directory has no name: {}",
                    dir.display()
                ))
            })?;
            parent.canonicalize()?.join(file_name)
        }
        Err(error) => return Err(error.into()),
    };
    Ok((lock, canonical_dir))
}

/// Inputs the scaffold marks `trigger=True`: the data-trigger input, or every
/// input of a sync node (the aligned set).
fn python_trigger_inputs(policy: Option<&MacroPolicy>, inputs: &[(String, String)]) -> Vec<String> {
    match policy {
        Some(MacroPolicy::DataTrigger { input_name }) => vec![input_name.clone()],
        Some(MacroPolicy::Sync { .. }) => inputs.iter().map(|(name, _)| name.clone()).collect(),
        _ => Vec::new(),
    }
}

fn python_policy(policy: Option<&MacroPolicy>) -> CliResult<(serde_json::Value, String)> {
    match policy {
        Some(MacroPolicy::Period { period_ms }) => Ok((
            serde_json::json!({ "period_ms": period_ms }),
            format!("period_ms={period_ms}"),
        )),
        Some(MacroPolicy::DataTrigger { input_name }) => Ok((
            serde_json::json!({ "data_trigger": { "input_name": input_name } }),
            format!("trigger={input_name:?}"),
        )),
        Some(MacroPolicy::Sync { window_ms }) => Ok((
            serde_json::json!({ "sync_window_ms": window_ms }),
            format!("sync_window_ms={window_ms}"),
        )),
        Some(MacroPolicy::External) | Some(MacroPolicy::UnboundedSync) => {
            Err(CliError::Validation(
                "Python node policy is not supported by the Python decorator".to_string(),
            ))
        }
        None => Err(CliError::Validation(
            "Python nodes need a trigger policy: pass -T SCHEMA NAME or --policy period_ms=N (Python nodes cannot be modified after creation)"
                .to_string(),
        )),
    }
}

/// Options for `node_create` controlling port declarations and template style.
#[derive(Debug, Default)]
pub struct NodeCreateOptions {
    /// Output ports: `(schema, name)` pairs (e.g. `("sensor_msgs::Image", "image")`).
    /// Schemas may be qualified (`pkg/Type` or `pkg::Type`) or BARE
    /// (`Vector3`) — bare names are resolved at create time via
    /// [`schema_cmd::resolve_port_schema`]: workspace
    /// schemas win, a unique built-in short name qualifies, and
    /// ambiguous/unknown names error before anything is created.
    pub outputs: Vec<(String, String)>,
    /// Input ports: `(schema, name)` pairs (same schema resolution
    /// as `outputs`).
    pub inputs: Vec<(String, String)>,
    /// Trigger port name (must match one of the input port names).
    pub trigger: Option<String>,
    /// Use the legacy raw FFI template instead of `#[cerulion_node]` macro.
    pub raw_ffi: bool,
    /// Authoring language for the generated node.
    pub language: NodeLanguage,
}

/// Supported node authoring languages.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum NodeLanguage {
    /// Rust macro node.
    #[default]
    Rust,
    /// Embedded-CPython node.
    Python,
}

/// Create a new node type in the workspace — convenience wrapper.
///
/// This convenience function exists so callers that just want a
/// node *skeleton* (typically tests, programmatic scaffolding, or
/// the TUI's "new node" affordance) can create one without having
/// to construct a [`NodeCreateOptions`] or pick a starter policy.
///
/// **Bootstrap default**: when `policy` is `None`, this function
/// supplies `Some(MacroPolicy::Period { period_ms: 100 })` before
/// dispatching to [`node_create_with_options`]. The default is
/// emitted explicitly here (rather than buried as a fallback inside
/// the template generator) so the convenience is documented at the
/// API boundary, not hidden. Callers who want strict
/// "0 inputs → error" semantics (matching the CLI's
/// `resolve_create_policy`) should use [`node_create_with_options`]
/// directly and pass `policy = None` themselves.
pub fn node_create(
    nodes_dir: &Path,
    workspace_cargo_toml: &Path,
    node_type: &str,
    policy: Option<cerulion_core::MacroPolicy>,
) -> CliResult<()> {
    let (_lock, nodes_dir) = acquire_workspace_lock(nodes_dir)?;
    let bootstrap_policy = policy.or(Some(cerulion_core::MacroPolicy::Period { period_ms: 100 }));
    node_create_with_options(
        &nodes_dir,
        workspace_cargo_toml,
        node_type,
        bootstrap_policy,
        &NodeCreateOptions::default(),
    )
}

/// Create a new node type with full port and template options —
/// strict variant.
///
/// Unlike [`node_create`], this function rejects the shape
/// `policy = None && options.inputs.is_empty() && options.trigger.is_none()`
/// at the API boundary: such a node would have no firing rule
/// (the macro would expand to `#[cerulion_node]` with no inputs
/// and no policy attribute, failing
/// `validate_trigger_inference`). The CLI's `resolve_create_policy`
/// already errors on this shape at the user surface; mirroring the
/// check here keeps the engine API and CLI contracts aligned so
/// future direct-engine callers (other tooling, plugins) cannot
/// accidentally create an unfirable node.
///
/// Port schemas in `options` may be bare (`Vector3`) — they are
/// resolved via [`schema_cmd::resolve_port_schema`] BEFORE any
/// filesystem mutation, so an ambiguous/unknown name
/// errors with no node directory created. See
/// [`NodeCreateOptions::outputs`] for the resolution rules.
pub fn node_create_with_options(
    nodes_dir: &Path,
    workspace_cargo_toml: &Path,
    node_type: &str,
    policy: Option<cerulion_core::MacroPolicy>,
    options: &NodeCreateOptions,
) -> CliResult<()> {
    let (_lock, nodes_dir) = acquire_workspace_lock(nodes_dir)?;
    let canonical_workspace_cargo_toml = workspace_cargo_toml.canonicalize()?;
    utils::validate_node_type(node_type)?;
    if options.language == NodeLanguage::Python && options.raw_ffi {
        return Err(CliError::Validation(
            "--raw-ffi applies to Rust nodes only; Python nodes always use the embedded-CPython template"
                .to_string(),
        ));
    }
    let policy = if options.language == NodeLanguage::Python && policy.is_none() {
        match options.trigger.as_deref() {
            Some(input_name) => Some(cerulion_core::MacroPolicy::DataTrigger {
                input_name: input_name.to_string(),
            }),
            None => {
                return Err(CliError::Validation(
                    "Python nodes need a trigger policy: pass -T SCHEMA NAME or --policy period_ms=N (Python nodes cannot be modified after creation)"
                        .to_string(),
                ));
            }
        }
    } else {
        policy
    };

    if policy.is_none() && options.inputs.is_empty() && options.trigger.is_none() {
        return Err(CliError::Validation(
            "node_create_with_options requires a trigger policy when no inputs are \
             declared: pass Some(MacroPolicy::Period { period_ms: N }), \
             Some(MacroPolicy::External), or declare at least one input via \
             `NodeCreateOptions::inputs`. For a quick skeleton, use `node_create` \
             which supplies a 100ms period default."
                .to_string(),
        ));
    }

    // Reject duplicate input port names. Two entries in
    // `options.inputs` with the same name would generate a `lib.rs`
    // declaring the field twice, which fails `cargo build` with an
    // opaque "duplicate field" error. Catch it here for a clear
    // diagnostic. The CLI's `NodeAction::Create` handler also
    // guards against `-T` and `-i` colliding on the same name —
    // mirror that gate at the engine API boundary per the
    // CLI-vs-engine contract alignment rule, so future direct
    // engine callers don't bypass it.
    {
        let mut seen: std::collections::HashSet<&str> = std::collections::HashSet::new();
        for (_, name) in &options.inputs {
            if !seen.insert(name.as_str()) {
                return Err(CliError::Validation(format!(
                    "duplicate input port name '{name}' in NodeCreateOptions::inputs. \
                     Each input port name must be unique."
                )));
            }
        }
    }
    if options.language == NodeLanguage::Python {
        for (_, name) in options.inputs.iter().chain(options.outputs.iter()) {
            utils::validate_python_identifier(name)?;
        }
        if let Some(name) = &options.trigger {
            utils::validate_python_identifier(name)?;
            if !options.inputs.iter().any(|(_, input)| input == name) {
                return Err(CliError::Validation(format!(
                    "trigger '{name}' does not name an input port"
                )));
            }
        }
        let mut seen: std::collections::HashSet<&str> = std::collections::HashSet::new();
        for (_, name) in options.inputs.iter().chain(options.outputs.iter()) {
            if !seen.insert(name.as_str()) {
                return Err(CliError::Validation(format!(
                    "duplicate port name '{name}': a Python node's inputs and outputs share one namespace"
                )));
            }
        }
    }

    let node_dir = nodes_dir.join(node_type);
    if node_dir.exists() {
        return Err(CliError::NodeExists {
            node_type: node_type.to_string(),
        });
    }
    let python_paths = if options.language == NodeLanguage::Python {
        Some(python_paths_for_new_node(&node_dir, node_type)?)
    } else {
        None
    };

    let inputs = options.inputs.clone();
    let outputs = options.outputs.clone();

    // Resolve every port schema BEFORE any filesystem
    // mutation, so a bare built-in name (`Vector3`) scaffolds a
    // resolvable import (`geometry_msgs/Vector3`) instead of the
    // E0432 `use Vector3;`, and an ambiguous/unknown name errors
    // with NO node directory created. Workspace schemas win (bare
    // workspace names pass through unchanged) and the resolver is
    // idempotent, so callers already passing qualified names see
    // byte-identical output. The CLI resolves the same args
    // pre-call for its user-facing breadcrumbs; this is the
    // engine-boundary enforcement backstop (per the CLI-vs-engine
    // contract alignment rule), so direct engine callers get the
    // same contract.
    let schemas_dir = workspace_schemas_dir(&nodes_dir);
    let mut metadata = NodeMetadata::new(node_type, policy.clone());
    for (schema, name) in &outputs {
        let resolved = schema_cmd::resolve_port_schema(&schemas_dir, schema)?;
        // A `.msg` store type has no generated Rust type, so a port
        // scaffolded from it could never compile — refuse loudly BEFORE any
        // filesystem mutation (see `refuse_store_port_scaffold`).
        schema_cmd::refuse_store_port_scaffold(&resolved, schema)?;
        metadata.outputs.push(PortDef {
            name: name.clone(),
            schema: Some(resolved.schema),
            schema_alternatives: Vec::new(),
            trigger: false,
            backpressure: BackpressurePolicy::DropOldest,
        });
    }
    for (schema, name) in &inputs {
        let resolved = schema_cmd::resolve_port_schema(&schemas_dir, schema)?;
        schema_cmd::refuse_store_port_scaffold(&resolved, schema)?;
        // `node create` scaffolds plain (non-trigger) inputs. A
        // trigger input is ADDED later via `node modify -T/--trigger-input
        // SCHEMA NAME` (a new `#[input(trigger)]` field); promoting an
        // EXISTING plain input is a source edit (or `--policy
        // data_trigger=NAME`).
        metadata.inputs.push(PortDef {
            name: name.clone(),
            schema: Some(resolved.schema),
            schema_alternatives: Vec::new(),
            trigger: false,
            backpressure: BackpressurePolicy::DropOldest,
        });
    }

    let python = options.language == NodeLanguage::Python;
    let python_policy = if python {
        Some(python_policy(policy.as_ref())?)
    } else {
        None
    };
    let python_inputs = metadata
        .inputs
        .iter()
        .map(|port| {
            let schema = port.schema.clone().ok_or_else(|| {
                CliError::Validation(format!(
                    "Python nodes require a schema for port '{}'",
                    port.name
                ))
            })?;
            Ok((port.name.clone(), schema))
        })
        .collect::<CliResult<Vec<(String, String)>>>()?;
    let python_outputs = metadata
        .outputs
        .iter()
        .map(|port| {
            let schema = port.schema.clone().ok_or_else(|| {
                CliError::Validation(format!(
                    "Python nodes require a schema for port '{}'",
                    port.name
                ))
            })?;
            Ok((port.name.clone(), schema))
        })
        .collect::<CliResult<Vec<(String, String)>>>()?;
    let python_pynode_path = if python {
        let base = crate::workspace::find_cerulion_base().ok_or_else(|| {
            CliError::Validation(
                "Python nodes need a Cerulion checkout: cerulion_pynode is not published to a registry"
                    .to_string(),
            )
        })?;
        let path = base.join("crates/cerulion_py/crates/cerulion_pynode");
        if !path.is_dir() {
            return Err(CliError::Validation(format!(
                "Python nodes need cerulion_pynode at {}",
                path.display()
            )));
        }
        Some(path)
    } else {
        None
    };

    let src_dir = node_dir.join("src");
    std::fs::create_dir_all(&src_dir)?;

    // Generate Cargo.toml
    let python_node_dir = node_dir
        .canonicalize()
        .unwrap_or_else(|_| node_dir.clone())
        .display()
        .to_string();
    std::fs::write(
        node_dir.join("Cargo.toml"),
        if python {
            templates::generate_python_cargo_toml(
                node_type,
                &python_pynode_path
                    .as_ref()
                    .expect("python path validated above")
                    .display()
                    .to_string(),
            )
        } else {
            templates::generate_cargo_toml(node_type)
        },
    )?;

    // Generate lib.rs — macro template by default, raw FFI with --raw-ffi
    let lib_source = if python {
        let (python_policy_json, _) = python_policy.as_ref().ok_or_else(|| {
            CliError::Validation("Python node policy was not prepared".to_string())
        })?;
        templates::generate_python_lib_rs(
            &python_node_dir,
            &python_inputs,
            &python_outputs,
            python_policy_json,
            &python_paths
                .as_ref()
                .expect("Python paths prepared above")
                .site_paths,
        )
        .map_err(|error| {
            CliError::Validation(format!("failed to serialize Python metadata: {error}"))
        })?
    } else if options.raw_ffi {
        templates::generate_lib_rs(&metadata)
    } else {
        templates::generate_macro_lib_rs(&metadata, options.trigger.as_deref())
    };
    std::fs::write(src_dir.join("lib.rs"), lib_source)?;
    if python {
        let (_, python_policy_expr) = python_policy.as_ref().ok_or_else(|| {
            CliError::Validation("Python node policy was not prepared".to_string())
        })?;
        std::fs::write(
            node_dir.join("build.rs"),
            templates::generate_python_build_rs(
                &python_paths
                    .as_ref()
                    .expect("Python paths prepared above")
                    .libdir,
            ),
        )?;
        std::fs::write(
            node_dir.join("node.py"),
            templates::generate_python_node_py(
                &python_inputs,
                &python_outputs,
                python_policy_expr,
                &python_trigger_inputs(policy.as_ref(), &python_inputs),
            ),
        )?;
    }

    // Update workspace Cargo.toml members
    add_workspace_member(&canonical_workspace_cargo_toml, node_type)?;

    tracing::info!(node_type = %node_type, "node created");
    Ok(())
}

/// Delete a node type from the workspace.
pub fn node_delete(
    nodes_dir: &Path,
    workspace_cargo_toml: &Path,
    node_type: &str,
) -> CliResult<()> {
    let (_lock, nodes_dir) = acquire_workspace_lock(nodes_dir)?;
    let workspace_cargo_toml = workspace_cargo_toml.canonicalize()?;
    let node_dir = nodes_dir.join(node_type);
    if !node_dir.exists() {
        return Err(CliError::NodeNotFound {
            node_type: node_type.to_string(),
        });
    }

    std::fs::remove_dir_all(&node_dir)?;
    remove_workspace_member(&workspace_cargo_toml, node_type)?;

    tracing::info!(node_type = %node_type, "node deleted");
    Ok(())
}

fn refuse_python_node_modify(node_dir: &Path, node_type: &str) -> CliResult<()> {
    if node_dir.join("node.py").is_file() {
        return Err(CliError::Validation(format!(
            "node '{node_type}' is a Python node; edit nodes/{node_type}/node.py and run `cerulion node build {node_type}`"
        )));
    }
    Ok(())
}

/// Add input or output port to a node.
///
/// Reads the current ports from `nodes/<type>/src/lib.rs` via
/// `parse_node_metadata`, checks for duplicates, then mutates the
/// source: for macro nodes splices a new `#[input]`/`#[output]`
/// field into the struct via `templates::inject_macro_field`; for
/// raw-FFI nodes regenerates the `INFO_BYTES` JSON. The user's
/// hand-edited tick body, comments, and other struct fields
/// (e.g. `tick_count: u32`) are preserved untouched.
///
/// `is_trigger` (when adding an input) designates the new field
/// as the data trigger (`#[input(trigger)]`) AND clears any
/// conflicting node-level policy args (`period_ms`,
/// `sync_window_ms`, `external`) from the
/// `#[cerulion_node(...)]` line. Ignored for output ports
/// (`is_trigger=true && is_output=true` returns an error).
///
/// `schema` may be qualified (`pkg/Type` or `pkg::Type`) or BARE
/// (`Vector3`) — bare names are resolved before any source edit
/// via [`schema_cmd::resolve_port_schema`]: workspace
/// schemas win, a unique built-in short name qualifies, and
/// ambiguous/unknown names error with the source untouched.
pub fn node_modify_add_port(
    nodes_dir: &Path,
    node_type: &str,
    port_name: &str,
    schema: Option<&str>,
    is_output: bool,
    is_trigger: bool,
) -> CliResult<()> {
    let (_lock, nodes_dir) = acquire_workspace_lock(nodes_dir)?;
    if is_trigger && is_output {
        return Err(CliError::Validation(
            "-T/--trigger-input applies to input ports only; output ports cannot be triggers"
                .to_string(),
        ));
    }
    let node_dir = nodes_dir.join(node_type);
    if !node_dir.exists() {
        return Err(CliError::NodeNotFound {
            node_type: node_type.to_string(),
        });
    }
    refuse_python_node_modify(&node_dir, node_type)?;

    // Resolve the schema argument BEFORE reading or writing
    // any source, so a bare built-in name (`Vector3`) splices a
    // resolvable import instead of `use Vector3;` (E0432), and an
    // ambiguous/unknown name errors with lib.rs byte-untouched.
    // Workspace schemas win (bare workspace names pass through
    // unchanged); the resolver is idempotent, so already-qualified
    // callers are byte-identical no-ops. Shadowing the parameter
    // routes the resolved value through every downstream use
    // (`schema_to_ident`, `add_import_if_needed`, the raw-FFI
    // PortDef) with no site left behind. The engine-boundary
    // backstop for the CLI's own pre-call resolution.
    let resolved_schema = schema
        .map(|raw| {
            let resolved =
                schema_cmd::resolve_port_schema(&workspace_schemas_dir(&nodes_dir), raw)?;
            // A `.msg` store type has no generated Rust type — refuse loudly
            // with lib.rs byte-untouched (see `refuse_store_port_scaffold`).
            schema_cmd::refuse_store_port_scaffold(&resolved, raw)?;
            Ok::<String, crate::error::CliError>(resolved.schema)
        })
        .transpose()?;
    let schema = resolved_schema.as_deref();

    // Read current metadata from source to detect duplicate ports
    // before we edit anything.
    let metadata = parse_node_metadata(&node_dir)?;
    if is_output && metadata.has_output(port_name) {
        return Err(CliError::Validation(format!(
            "output port '{}' already exists on node '{}'",
            port_name, node_type
        )));
    }
    if !is_output && metadata.has_input(port_name) {
        return Err(CliError::Validation(format!(
            "input port '{}' already exists on node '{}'",
            port_name, node_type
        )));
    }

    // Warn when adding a regular (non-trigger) input to a node
    // whose existing source already declares a data-trigger
    // (`#[input(trigger)] foo: T`). The newly added input will
    // NOT participate in firing — only the existing trigger
    // input fires the node. This silent semantic gap was the
    // motivating bug for this warn: a user adding a
    // second input naturally expects it to fan in or replace the
    // trigger, but the runtime keeps firing on the original
    // trigger only and the new input is read-side only.
    //
    // Skip the warn when:
    //   - adding an output (no firing-rule interaction)
    //   - the new input IS being marked as trigger (`-T`) — that
    //     replaces the existing trigger and the engine clears the
    //     conflicting macro args
    //   - the node has no existing data_trigger declaration
    if !is_output && !is_trigger {
        if let Some(cerulion_core::MacroPolicy::DataTrigger { ref input_name }) = metadata.policy {
            tracing::warn!(
                node_type = %node_type,
                added_input = %port_name,
                existing_trigger = %input_name,
                "adding non-trigger input to a data-triggered node: the new input \
                 '{port_name}' will NOT fire the node (only '{input_name}' fires it). \
                 If you want fan-in firing, use `--policy sync_window_ms=N` to switch \
                 to a sync trigger. To make '{port_name}' the new trigger instead, \
                 re-add it with `-T <schema> {port_name}` (replaces the existing \
                 trigger). If the new input is intentionally read-only, ignore this \
                 warning."
            );
        }
    }

    let lib_path = node_dir.join("src/lib.rs");
    let source = std::fs::read_to_string(&lib_path)?;

    let updated = if templates::is_macro_based(&source) {
        // Macro path: splice the new field into the struct body.
        // Field type is the bare ident derived from the schema
        // (canonicalised slash form by `parse_port_args`); for
        // unqualified or no-schema ports, default to `()` (the
        // same placeholder `generate_macro_lib_rs` uses).
        let field_type = schema
            .map(|s| templates::schema_to_ident(s).to_string())
            .unwrap_or_else(|| "()".to_string());
        // Clear any existing trigger-policy macro args BEFORE
        // injecting the new trigger field — otherwise
        // `period_ms = 100` (or `external` etc.) would compete
        // with the new `#[input(trigger)]` and the macro would
        // reject the conflicting hints.
        let policy_cleared = if is_trigger {
            templates::clear_macro_policy_args(&source).map_err(CliError::Validation)?
        } else {
            source.clone()
        };
        let injected = templates::inject_macro_field(
            &policy_cleared,
            port_name,
            &field_type,
            is_output,
            is_trigger,
        )
        .map_err(CliError::Validation)?;
        if let Some(schema_ref) = schema {
            add_import_if_needed(&injected, schema_ref)
        } else {
            injected
        }
    } else if is_trigger {
        return Err(CliError::Validation(format!(
            "node '{}' is a raw-FFI node; `-T/--trigger-input` is only supported for \
             `#[cerulion_node]` macro nodes",
            node_type
        )));
    } else {
        // Raw-FFI path: append the new port to the in-memory
        // metadata and regenerate the INFO_BYTES JSON.
        let mut updated_metadata = metadata.clone();
        // The raw-FFI branch is only reached when `is_trigger` is
        // false (the `else if is_trigger` arm above rejects `-T/--trigger-input`
        // on a raw-FFI node), so a newly-added raw-FFI port is never a trigger.
        let new_port = PortDef {
            name: port_name.to_string(),
            schema: schema.map(|s| s.to_string()),
            schema_alternatives: Vec::new(),
            trigger: false,
            backpressure: BackpressurePolicy::DropOldest,
        };
        if is_output {
            updated_metadata.outputs.push(new_port);
        } else {
            updated_metadata.inputs.push(new_port);
        }
        let updated = update_info_fn(&source, &updated_metadata)?;
        if let Some(schema_ref) = schema {
            add_import_if_needed(&updated, schema_ref)
        } else {
            updated
        }
    };

    std::fs::write(&lib_path, updated)?;

    tracing::info!(
        node_type = %node_type,
        port_name = %port_name,
        direction = if is_output { "output" } else { "input" },
        "port added"
    );
    Ok(())
}

/// Clear the data trigger on a macro node.
///
/// Removes `#[input(trigger)]` from any input field (the trigger
/// field becomes a regular `#[input]`) AND inserts
/// `period_ms = 100` into the `#[cerulion_node(...)]` macro args
/// so the node has a valid trigger-policy hint after the change.
/// Raw-FFI nodes don't have a `trigger` concept; calling on a
/// raw-FFI source returns a descriptive error.
pub fn node_modify_clear_trigger(nodes_dir: &Path, node_type: &str) -> CliResult<()> {
    let (_lock, nodes_dir) = acquire_workspace_lock(nodes_dir)?;
    let node_dir = nodes_dir.join(node_type);
    if !node_dir.exists() {
        return Err(CliError::NodeNotFound {
            node_type: node_type.to_string(),
        });
    }
    refuse_python_node_modify(&node_dir, node_type)?;
    let lib_path = node_dir.join("src/lib.rs");
    let source = std::fs::read_to_string(&lib_path)?;
    if !templates::is_macro_based(&source) {
        return Err(CliError::Validation(format!(
            "node '{}' is a raw-FFI node; `--no-trigger` is only supported for `#[cerulion_node]` macro nodes",
            node_type
        )));
    }
    let updated = templates::clear_input_trigger_attr(&source).map_err(CliError::Validation)?;
    let updated = templates::ensure_macro_policy_hint(&updated).map_err(CliError::Validation)?;
    std::fs::write(&lib_path, updated)?;
    tracing::info!(node_type = %node_type, "trigger cleared; falling back to period_ms = 100");
    Ok(())
}

/// Enable/disable the `external` (self-triggering ingress/driver)
/// policy on a node.
///
/// For macro nodes this toggles the `external` token in the
/// `#[cerulion_node(...)]` attribute via
/// `templates::toggle_macro_external_flag`. Raw-FFI nodes have no
/// equivalent macro arg; calling on a raw-FFI source returns a
/// descriptive error rather than silently no-op'ing.
///
/// Enabling `external` here (i.e. `node modify --policy external`)
/// only flips the macro token — it deliberately does NOT inject the
/// required `external_source()` method (unlike `node create`, which
/// scaffolds a `HostDriven` stub). The macro then emits a hard
/// compile error naming exactly what to add, so the user chooses the
/// real source (`ExternalSource::Fd`/`Blocking`/`HostDriven`).
pub fn node_modify_ext_trigger(nodes_dir: &Path, node_type: &str, ext: bool) -> CliResult<()> {
    let (_lock, nodes_dir) = acquire_workspace_lock(nodes_dir)?;
    let node_dir = nodes_dir.join(node_type);
    if !node_dir.exists() {
        return Err(CliError::NodeNotFound {
            node_type: node_type.to_string(),
        });
    }
    refuse_python_node_modify(&node_dir, node_type)?;

    let lib_path = node_dir.join("src/lib.rs");
    let source = std::fs::read_to_string(&lib_path)?;

    if !templates::is_macro_based(&source) {
        return Err(CliError::Validation(format!(
            "node '{}' is a raw-FFI node; the `external` policy (`--policy external`) is only supported for `#[cerulion_node]` macro nodes. Migrate to the macro form (drop `--raw-ffi` from `node create`) to use the external ingress policy, or set the trigger by hand at the FFI surface.",
            node_type
        )));
    }

    let updated =
        templates::toggle_macro_external_flag(&source, ext).map_err(CliError::Validation)?;
    std::fs::write(&lib_path, updated)?;

    tracing::info!(node_type = %node_type, ext_trigger = ext, "ext-trigger updated");
    Ok(())
}

/// Set the trigger policy on a macro node by mutating the
/// `#[cerulion_node(...)]` argument list. The supplied `policy`
/// determines the resulting macro args:
///
/// - `Period { period_ms }` → `period_ms = N` (strips other policy keys)
/// - `Sync { window_ms }` → `sync_window_ms = N` (strips other policy keys)
/// - `External` → `external` (strips other policy keys)
/// - `DataTrigger { input_name }` is rejected here — DataTrigger is
///   carried by `#[input(trigger)]` on the input field, not by a
///   node-level macro arg. Callers handle DataTrigger at the
///   `node_modify_add_port(... is_trigger=true)` site.
///
/// Raw-FFI nodes are not supported; the caller errors with a
/// descriptive message when this is invoked on raw-FFI source.
pub fn node_modify_set_policy(
    nodes_dir: &Path,
    node_type: &str,
    policy: &cerulion_core::MacroPolicy,
) -> CliResult<()> {
    let (_lock, nodes_dir) = acquire_workspace_lock(nodes_dir)?;
    use cerulion_core::MacroPolicy;
    let node_dir = nodes_dir.join(node_type);
    if !node_dir.exists() {
        return Err(CliError::NodeNotFound {
            node_type: node_type.to_string(),
        });
    }
    refuse_python_node_modify(&node_dir, node_type)?;
    let lib_path = node_dir.join("src/lib.rs");
    let source = std::fs::read_to_string(&lib_path)?;
    if !templates::is_macro_based(&source) {
        return Err(CliError::Validation(format!(
            "node '{}' is a raw-FFI node; `--policy` is only supported for `#[cerulion_node]` \
             macro nodes. Migrate to the macro form (drop `--raw-ffi` from `node create`) to \
             use the `--policy` flag, or set the trigger by hand at the FFI surface.",
            node_type
        )));
    }
    let arg = match policy {
        MacroPolicy::Period { period_ms } => format!("period_ms = {period_ms}"),
        MacroPolicy::Sync { window_ms } => format!("sync_window_ms = {window_ms}"),
        MacroPolicy::UnboundedSync => "unbounded_sync".to_string(),
        MacroPolicy::External => "external".to_string(),
        MacroPolicy::DataTrigger { .. } => {
            return Err(CliError::Validation(
                "`DataTrigger` is set via `#[input(trigger)]` on the input field, not via a \
                 node-level policy arg. Add the input with `-i SCHEMA NAME` and set \
                 `--policy data_trigger=NAME`."
                    .to_string(),
            ));
        }
    };
    let updated = templates::set_macro_policy_arg(&source, &arg).map_err(CliError::Validation)?;
    std::fs::write(&lib_path, updated)?;
    tracing::info!(node_type = %node_type, policy = ?policy, "policy updated");
    Ok(())
}

/// Convenience wrapper around `node_modify_set_policy` for `Period`.
pub fn node_modify_set_period(nodes_dir: &Path, node_type: &str, period_ms: u64) -> CliResult<()> {
    let (_lock, nodes_dir) = acquire_workspace_lock(nodes_dir)?;
    node_modify_set_policy(
        &nodes_dir,
        node_type,
        &cerulion_core::MacroPolicy::Period { period_ms },
    )
}

/// Convenience wrapper around `node_modify_set_policy` for `Sync`.
pub fn node_modify_set_sync(nodes_dir: &Path, node_type: &str, window_ms: u64) -> CliResult<()> {
    let (_lock, nodes_dir) = acquire_workspace_lock(nodes_dir)?;
    node_modify_set_policy(
        &nodes_dir,
        node_type,
        &cerulion_core::MacroPolicy::Sync { window_ms },
    )
}

/// Promote an existing `#[input] NAME: T` field on the node to
/// `#[input(trigger)] NAME: T`, AND clear any conflicting
/// node-level policy args. The input must already exist.
///
/// Errors if the named input doesn't exist or if the node is
/// raw-FFI (no field-attribute path).
pub fn node_modify_promote_input_to_trigger(
    nodes_dir: &Path,
    node_type: &str,
    input_name: &str,
) -> CliResult<()> {
    let (_lock, nodes_dir) = acquire_workspace_lock(nodes_dir)?;
    let node_dir = nodes_dir.join(node_type);
    if !node_dir.exists() {
        return Err(CliError::NodeNotFound {
            node_type: node_type.to_string(),
        });
    }
    refuse_python_node_modify(&node_dir, node_type)?;
    let metadata = parse_node_metadata(&node_dir)?;
    let input_exists = metadata.inputs.iter().any(|p| p.name == input_name);
    if !input_exists {
        return Err(CliError::Validation(format!(
            "input '{input_name}' does not exist on node '{node_type}'; declare it via \
             `cerulion node modify {node_type} -i SCHEMA {input_name}` first"
        )));
    }
    let lib_path = node_dir.join("src/lib.rs");
    let source = std::fs::read_to_string(&lib_path)?;
    if !templates::is_macro_based(&source) {
        return Err(CliError::Validation(format!(
            "node '{node_type}' is a raw-FFI node; `--policy data_trigger=NAME` is only \
             supported for `#[cerulion_node]` macro nodes"
        )));
    }
    // 1. Drop `(trigger)` from any existing trigger field.
    let cleared = templates::clear_input_trigger_attr(&source).map_err(CliError::Validation)?;
    // 2. Mark the named field as the trigger.
    let promoted = templates::promote_input_field_to_trigger(&cleared, input_name)
        .map_err(CliError::Validation)?;
    // 3. Strip conflicting node-level policy args.
    let final_source =
        templates::clear_macro_policy_args(&promoted).map_err(CliError::Validation)?;
    std::fs::write(&lib_path, final_source)?;
    tracing::info!(
        node_type = %node_type,
        input = %input_name,
        "promoted input to data trigger"
    );
    Ok(())
}

/// List all node types in the workspace.
///
/// Walks each `nodes/<type>/` subdirectory and runs
/// `parse_node_metadata` against its `src/lib.rs`. Node directories
/// that don't contain a recognisable Cerulion node (no
/// `#[cerulion_node]` struct, no `// CERULION:INFO_START`
/// markers) are silently skipped — they may be in-progress
/// scaffolds or non-Cerulion crates that happen to live under
/// `nodes/`.
pub fn node_list(nodes_dir: &Path) -> CliResult<Vec<NodeMetadata>> {
    let mut result = vec![];
    if !nodes_dir.is_dir() {
        return Ok(result);
    }

    let mut entries: Vec<_> = std::fs::read_dir(nodes_dir)?
        .filter_map(|e| e.ok())
        .filter(|e| e.path().is_dir())
        .collect();
    entries.sort_by_key(|e| e.file_name());

    for entry in entries {
        // Best-effort: if a directory's lib.rs is unparseable or
        // missing markers, skip it rather than fail the whole list.
        if let Ok(metadata) = parse_node_metadata(&entry.path()) {
            result.push(metadata);
        }
    }
    Ok(result)
}

/// Get info about a specific node type.
pub fn node_info(nodes_dir: &Path, node_type: &str) -> CliResult<NodeMetadata> {
    let node_dir = nodes_dir.join(node_type);
    let mut metadata = parse_node_metadata(&node_dir)?;
    let source = std::fs::read_to_string(node_dir.join("src/lib.rs"))?;
    let Some(start) = source.find("static INFO_BYTES: &[u8] = b\"") else {
        return Ok(metadata);
    };
    let after_open = start + "static INFO_BYTES: &[u8] = b\"".len();
    let Some(close) = source[after_open..].find("\\0\"") else {
        return Ok(metadata);
    };
    let literal = syn::parse_str::<syn::LitByteStr>(&format!(
        "b\"{}\"",
        &source[after_open..after_open + close]
    ))
    .map_err(|error| {
        CliError::Validation(format!(
            "node '{}' INFO_BYTES literal failed to parse: {}",
            node_type, error
        ))
    })?;
    let info: serde_json::Value = serde_json::from_slice(&literal.value()).map_err(|error| {
        CliError::Validation(format!(
            "node '{}' INFO_BYTES JSON failed to parse: {}",
            node_type, error
        ))
    })?;
    let schemas =
        crate::graph_cmd::build_workspace_schema_hashes(nodes_dir.parent().unwrap_or(nodes_dir));
    let schema_name = |hash: Option<u64>| {
        hash.and_then(|hash| {
            schemas
                .iter()
                .filter(|(name, candidate)| **candidate == hash && name.contains('/'))
                .map(|(name, _)| name.clone())
                .next()
                .or_else(|| {
                    schemas
                        .iter()
                        .find_map(|(name, candidate)| (*candidate == hash).then(|| name.clone()))
                })
        })
    };
    for (port, value) in metadata.inputs.iter_mut().zip(
        info.get("inputs")
            .and_then(serde_json::Value::as_array)
            .into_iter()
            .flatten(),
    ) {
        if port.schema.is_none() {
            port.schema = schema_name(value.get("schema_hash").and_then(|v| v.as_u64()));
        }
    }
    for (port, value) in metadata.outputs.iter_mut().zip(
        info.get("outputs")
            .and_then(serde_json::Value::as_array)
            .into_iter()
            .flatten(),
    ) {
        if port.schema.is_none() {
            port.schema = schema_name(value.get("schema_hash").and_then(|v| v.as_u64()));
        }
    }
    Ok(metadata)
}

/// What a build did about the node's optional SYSTEM dependencies,
/// alongside cargo's own output.
///
/// [`Self::notice`] has ALREADY been handed to `node_build_with_reporter`'s
/// sink by the time this is returned (before cargo ran — see that function).
/// It is repeated here for programmatic consumers; a caller that also used the
/// sink must not print it twice.
#[derive(Debug)]
pub struct NodeBuildOutcome {
    /// Cargo's captured output.
    pub output: std::process::Output,
    /// The per-dep verdicts (empty for a node that declares none).
    pub system_deps: system_deps::SystemDepReport,
    /// The rendered notice, or `None` when the node declares no optional
    /// system deps and there is correctly nothing to say.
    pub notice: Option<String>,
}

/// The exact `cargo` argv `node_build` runs, given the resolved feature set.
///
/// Extracted so the ONE thing the whole optional-system-deps mechanism exists
/// to do — get `--features <resolved>` onto the cargo invocation — is
/// assertable without spawning cargo and without a machine that happens to
/// have the library. Without this seam, deleting the `--features` append leaves the entire
/// suite green while producing a capability-less artifact.
pub fn cargo_build_args(node_type: &str, release: bool, features: &[String]) -> Vec<String> {
    let mut args = vec!["build".to_string(), "-p".to_string(), node_type.to_string()];
    if release {
        args.push("--release".to_string());
    }
    if !features.is_empty() {
        args.push("--features".to_string());
        args.push(features.join(","));
    }
    args
}

/// Build a node crate, enabling any optional SYSTEM-dependency feature whose
/// library is actually present on this machine.
///
/// Convenience wrapper over [`node_build_with_reporter`] that DISCARDS the
/// pre-build notice. `cerulion node build` uses [`node_build_with_progress`];
/// this one exists for callers that only want the artifact (the multi-process
/// box harnesses, which build an unrelated fixture node).
pub fn node_build(
    workspace_root: &Path,
    node_type: &str,
    release: bool,
) -> CliResult<NodeBuildOutcome> {
    node_build_with_reporter(workspace_root, node_type, release, &mut |_| {})
}

/// Build a node crate, reporting the optional-system-dependency decision
/// through `on_report` BEFORE cargo is invoked.
///
/// A node declares its optional system deps in its own manifest under
/// `[package.metadata.cerulion.optional-system-deps]`; see [`system_deps`] for
/// the format and for why this probe cannot live in a `build.rs`. The build
/// ALWAYS succeeds when the code compiles: a missing system library leaves its
/// feature off and produces a loud, actionable notice rather than a failure.
/// What it must never do is leave the feature off SILENTLY, which is why a
/// malformed metadata block is a hard error.
///
/// # Why the report goes out BEFORE cargo
///
/// This function decides, on its own, to append `--features` to the cargo
/// invocation. When that build then FAILS, the user sees cargo's stderr and
/// nothing else — so the one decision this module exists to make is invisible
/// in the failure it caused, and the plain `cargo build -p <node>` they try
/// next SUCCEEDS (the feature is off by default), which reads as "the CLI is
/// broken" rather than "the featured build does not link here". Reporting
/// first costs nothing and makes the failure self-describing.
///
/// `on_report` receives the rendered notice verbatim, exactly once, and only
/// when the node declares at least one optional system dep.
pub fn node_build_with_reporter(
    workspace_root: &Path,
    node_type: &str,
    release: bool,
    on_report: &mut dyn FnMut(&str),
) -> CliResult<NodeBuildOutcome> {
    node_build_with_progress(workspace_root, node_type, release, on_report, &mut |_| {})
}

/// The one line a user sees between typing `cerulion node build` and the
/// build finishing.
///
/// Cargo's output is captured and shown only when the build fails, so without
/// this the command prints nothing at all while cargo runs: seconds on a warm
/// tree, minutes for the first build of a workspace, which compiles the
/// Cerulion runtime from source as well as the node. A terminal that stays
/// silent that long reads as a hang. No trailing newline: the caller owns the
/// stream it goes to.
fn node_build_progress_line(node_type: &str) -> String {
    format!(
        "Building '{node_type}'. The first build of a workspace also compiles the Cerulion \
         runtime and can take a few minutes; nothing more is printed until the build finishes."
    )
}

/// [`node_build_with_reporter`], plus a progress line handed to
/// `on_cargo_start` immediately before cargo is spawned.
///
/// The line goes out only once every up-front refusal has passed (unknown
/// node, malformed metadata), so a build that was refused before reaching
/// cargo never claims to be building, and after the system-dep notice, so it
/// is the last thing printed before the silence it explains. It is a separate
/// sink from `on_report` because that one is silent for a node that declares
/// nothing, and this one speaks on every build.
pub fn node_build_with_progress(
    workspace_root: &Path,
    node_type: &str,
    release: bool,
    on_report: &mut dyn FnMut(&str),
    on_cargo_start: &mut dyn FnMut(&str),
) -> CliResult<NodeBuildOutcome> {
    let nodes_dir = workspace_root.join("nodes");
    let node_dir = nodes_dir.join(node_type);
    if !node_dir.exists() {
        return Err(CliError::NodeNotFound {
            node_type: node_type.to_string(),
        });
    }

    if node_dir.join("node.py").is_file() {
        let python = resolve_python(&node_dir);
        regenerate_python_info_with(&node_dir, node_type, &python)?;
        let paths = query_python_paths(&python).map_err(|reason| CliError::BuildFailed {
            target: node_type.to_string(),
            reason: format!(
                "Python interpreter {} path query failed: {reason}",
                python.display()
            ),
        })?;
        regenerate_python_build_rs_with(&node_dir, node_type, &python, &paths)?;
        regenerate_python_sys_path_with(&node_dir, node_type, &paths.site_paths)?;
    }

    // Probe BEFORE invoking cargo: a malformed declaration must stop the build
    // rather than silently produce a feature-less artifact.
    let declared = system_deps::optional_system_deps_of_node(&node_dir).map_err(|e| {
        CliError::BuildFailed {
            target: node_type.to_string(),
            reason: e.to_string(),
        }
    })?;

    // The contract's central rule, enforced for EVERY node and not just the
    // ones in this repo (the manifest walk test can only see a cerulion
    // checkout). A gated feature left in `default` makes the system library a
    // hard prerequisite for the plain `cargo build` again — the exact breakage
    // the mechanism removes — and the probe then buys nothing. Warn, don't
    // refuse: the artifact is still correct, and a manifest smell is not
    // grounds for failing a user's build.
    warn_on_gated_default_features(&node_dir, node_type, &declared);

    // ONE tool probe before the module loop. Without it a machine that has the
    // library but not `pkg-config` is told the library is missing, and handed
    // an install command it has already run.
    let tool = system_deps::pkg_config_tool();
    let report = if tool.can_probe() {
        system_deps::evaluate(declared, &system_deps::pkg_config_probe)
    } else {
        // Nothing can resolve without the tool; spawning N doomed `pkg-config`
        // processes to learn that would only slow the build down.
        system_deps::evaluate(declared, &|_m| None)
    };
    let features = report.features_to_enable();
    let notice = system_deps::render_report(
        node_type,
        &report,
        &tool,
        system_deps::preferred_install_keys(),
    );

    // Log the enabled set + any shortfall BEFORE the build, beside the printed
    // notice, so a cargo failure cannot swallow the attribution.
    if let Some(text) = &notice {
        on_report(text);
    }
    if !features.is_empty() {
        tracing::info!(
            node_type = %node_type,
            features = %features.join(","),
            "optional system dependencies found — building with their features"
        );
    }
    if report.has_missing() {
        warn_unsatisfied_deps(node_type, &tool);
    }

    // PATH's rustc is only a hint: Cargo may use RUSTC, build.rustc, or a
    // wrapper instead. Never reject a build based on this probe. The loader
    // checks the built cdylib's full fingerprint before node initialization.
    if let Some(detected_release) = detect_rustc_release(workspace_root) {
        if let Some(warning) = rustc_release_mismatch_warning(
            node_type,
            &detected_release,
            cerulion_core::RUSTC_RELEASE,
        ) {
            tracing::warn!(
                node_type = %node_type,
                path_rustc_release = %detected_release,
                host_rustc_release = cerulion_core::RUSTC_RELEASE,
                warning = %warning,
                "PATH compiler differs from Cerulion; continuing with Cargo's compiler selection"
            );
        }
    }

    let mut cmd = std::process::Command::new("cargo");
    cmd.args(cargo_build_args(node_type, release, &features));
    cmd.current_dir(workspace_root);

    on_cargo_start(&node_build_progress_line(node_type));
    let output = cmd.output().map_err(|e| CliError::BuildFailed {
        target: node_type.to_string(),
        reason: cargo_spawn_failure_reason(&e, workspace_root),
    })?;

    if !output.status.success() {
        return Err(CliError::BuildFailed {
            target: node_type.to_string(),
            reason: String::from_utf8_lossy(&output.stderr).to_string(),
        });
    }

    tracing::info!(node_type = %node_type, release, "build succeeded");
    Ok(NodeBuildOutcome {
        output,
        system_deps: report,
        notice,
    })
}

/// Probe PATH's `rustc -vV` in the workspace directory. Rustup's directory
/// selection is honored, but Cargo-specific compiler overrides need not match
/// this executable. This is advisory only; missing or unreadable output yields
/// no warning. The loader validates the built node's full fingerprint.
fn detect_rustc_release(workspace_root: &Path) -> Option<String> {
    let output = std::process::Command::new("rustc")
        .arg("-vV")
        .current_dir(workspace_root)
        .output()
        .ok()?;
    if !output.status.success() {
        return None;
    }
    let text = String::from_utf8(output.stdout).ok()?;
    let release = text
        .lines()
        .find_map(|line| line.strip_prefix("release: "))?;
    let release = release.trim();
    if release.is_empty() {
        None
    } else {
        Some(release.to_string())
    }
}

/// Describe a possible PATH compiler mismatch without claiming to know which
/// compiler Cargo will select. An absent or matching release needs no warning.
fn rustc_release_mismatch_warning(
    node_type: &str,
    detected_release: &str,
    host_release: &str,
) -> Option<String> {
    if detected_release.is_empty() || detected_release == host_release {
        return None;
    }
    Some(format!(
        "PATH rustc reports {detected_release}, while the `cerulion` host was built by \
         rustc {host_release}. Cargo may select a different compiler through an override; \
         the build will continue. The built node's full compiler fingerprint must match \
         the host at load time. If it does not, select the compiler that built the host \
         (for an official stable compiler: `rustup toolchain install {host_release}`, then \
         `RUSTUP_TOOLCHAIN={host_release} cerulion node build {node_type}`, ensuring any \
         Cargo compiler overrides agree), or rebuild the CLI and node with the same compiler."
    ))
}

/// Map a `cargo` spawn failure to the CLI's `BuildFailed` `reason` text.
///
/// A missing toolchain is the single most common way this spawn fails on a
/// fresh machine (`cerulion node build` spawns the user's own `cargo`; the
/// CLI ships no toolchain), and the raw [`std::io::Error`] text
/// ("No such file or directory (os error 2)") names none of that: it reads
/// like a build artifact went missing, not the toolchain that would have
/// produced it. Once the `node_dir.exists()` guard above has run (it lies
/// under `workspace_root`, the spawn's `current_dir`), an
/// `ErrorKind::NotFound` from `Command::output()` means the executable was
/// not located on `PATH` — BUT std reports a missing `current_dir` through
/// the same `NotFound` kind, and the `node_dir.exists()` guard ran earlier,
/// so a workspace removed between that check and this spawn (a TOCTOU race)
/// would otherwise be blamed on a missing toolchain. Guard against it here:
/// the dedicated toolchain message is used only when `workspace_root` still
/// resolves to a directory; if it has vanished, the raw `io::Error` is
/// preserved so the user sees the filesystem failure, not an irrelevant
/// reinstall remedy. ENOENT for a missing interpreter/loader still lands in
/// the toolchain arm (the workspace is intact there), where installing the
/// toolchain is the fix. Every other spawn error (permissions, resource
/// limits, …) keeps today's `io::Error` text verbatim, since those are not
/// toolchain-shaped problems.
/// The version floor is read off the manifest's `rust-version`
/// (`CARGO_PKG_RUST_VERSION`, workspace-inherited) so an MSRV bump cannot
/// leave this text advertising a toolchain that is too old; the unit
/// test pins the literal floor separately (not re-derived from the same
/// macro), so a bump trips it and the user-facing number is re-confirmed by
/// hand rather than tracking through unread. Pure and side-effect-free
/// so the NotFound/other split is a plain unit test, never
/// a process spawn.
fn cargo_spawn_failure_reason(err: &std::io::Error, workspace_root: &Path) -> String {
    if err.kind() == std::io::ErrorKind::NotFound && workspace_root.is_dir() {
        concat!(
            "`cargo` was not found on PATH. `cerulion node build` compiles the node with the ",
            "Rust toolchain: install Rust ",
            env!("CARGO_PKG_RUST_VERSION"),
            "+ from https://rustup.rs (plus a C linker: build-essential on Ubuntu/Debian, ",
            "Xcode Command Line Tools on macOS) and retry"
        )
        .to_string()
    } else {
        err.to_string()
    }
}

/// Warn that at least one declared dep did not resolve — with the SAME
/// diagnosis the printed notice carries.
///
/// The split is the whole point. When `pkg-config` could not run, NOTHING was
/// probed, so "the dependencies are missing" is a claim about libraries this
/// build never looked at. The notice already says so verbatim ("NOT PROVEN …
/// this is not evidence that the library is missing"); a structured log
/// contradicting it one line later is worse than no log at all — it is exactly
/// the inverted diagnosis [`system_deps::PkgConfigTool`] exists to remove,
/// re-emitted at the surface tooling greps. It also matters MORE than the
/// notice for a scripted run: `node build` is a one-shot verb, so the CLI's
/// default filter is `cerulion=warn`, and this line can be the only record the
/// run leaves in a log.
fn warn_unsatisfied_deps(node_type: &str, tool: &system_deps::PkgConfigTool) {
    if tool.can_probe() {
        tracing::warn!(
            node_type = %node_type,
            "one or more optional system dependencies are missing — the node is \
             being built without them (see the printed notice for the install command)"
        );
    } else {
        tracing::warn!(
            node_type = %node_type,
            "`pkg-config` is not usable on this machine, so one or more optional \
             system dependencies are UNPROVEN — the node is being built without them, \
             but this is NOT evidence that those libraries are absent. Install \
             pkg-config first (the printed notice carries the command for this \
             platform), then re-run"
        );
    }
}

/// Warn once per offender when a gated feature is ALSO in the crate's
/// `default` list. A manifest that cannot be re-read is not an error here — the
/// parse that produced `declared` already succeeded, so an unreadable file at
/// this point is a race, and the guard is advisory.
fn warn_on_gated_default_features(
    node_dir: &Path,
    node_type: &str,
    declared: &[system_deps::OptionalSystemDep],
) {
    if declared.is_empty() {
        return;
    }
    let manifest = node_dir.join("Cargo.toml");
    let Ok(text) = std::fs::read_to_string(&manifest) else {
        return;
    };
    let Ok(defaults) =
        system_deps::declared_default_features(&manifest.display().to_string(), &text)
    else {
        return;
    };
    for feature in system_deps::gated_features_in_default(declared, &defaults) {
        tracing::warn!(
            node_type = %node_type,
            feature = %feature,
            manifest = %manifest.display(),
            "`{feature}` is declared as an optional SYSTEM dependency but is also in \
             this crate's `default` features — so a plain `cargo build` still REQUIRES \
             the system library on every machine, which is the breakage the declaration \
             exists to remove. Remove `{feature}` from `default`; `cerulion node build` \
             turns it on whenever the library is present."
        );
    }
}

// ─── Internal helpers ────────────────────────────────────────

/// Derive the workspace `schemas/` directory from `nodes_dir`.
///
/// `workspace_create` lays out `nodes/` and `schemas/` as siblings
/// under the workspace root, so port-schema resolution reads
/// `nodes_dir.parent()/schemas`. A missing directory is fine —
/// `resolve_port_schema` treats it as an empty workspace and
/// resolves against built-ins only. The degenerate no-parent case
/// (`nodes_dir` at the filesystem root) falls back to a child path
/// no workspace tooling ever creates, which resolves the same way.
fn workspace_schemas_dir(nodes_dir: &Path) -> std::path::PathBuf {
    match nodes_dir.parent() {
        Some(parent) => parent.join("schemas"),
        None => nodes_dir.join("schemas"),
    }
}

/// Replace the `cerulion_node_info()` block between marker comments.
fn update_info_fn(source: &str, metadata: &NodeMetadata) -> CliResult<String> {
    let start_marker = "// CERULION:INFO_START";
    let end_marker = "// CERULION:INFO_END";

    if let (Some(start), Some(end)) = (source.find(start_marker), source.find(end_marker)) {
        let end = end + end_marker.len();
        let mut result = String::with_capacity(source.len());
        result.push_str(&source[..start]);
        result.push_str(&templates::generate_info_fn(metadata));
        // Skip trailing newline from marker if present
        let rest_start = if source[end..].starts_with('\n') {
            end + 1
        } else {
            end
        };
        result.push_str(&source[rest_start..]);
        Ok(result)
    } else {
        // Fallback: replace using regex on the info function pattern
        let re = regex::Regex::new(
            r#"(?s)(static INFO_BYTES:.*?\n\n)?#\[no_mangle\]\s*pub extern "C" fn cerulion_node_info\(\).*?\}\n"#,
        )
        .map_err(|e| CliError::Validation(format!("regex error: {}", e)))?;

        if re.is_match(source) {
            let replaced = re.replace(source, templates::generate_info_fn(metadata).as_str());
            Ok(replaced.to_string())
        } else {
            Err(CliError::Validation(
                "could not find cerulion_node_info() function in lib.rs".to_string(),
            ))
        }
    }
}

/// Replace the embedded Python INFO_BYTES marker block with freshly generated
/// declaration metadata.
pub fn regenerate_info_block(source: &str, json: &str) -> CliResult<String> {
    regenerate_info_block_with_port_schemas(source, json, None)
}

fn regenerate_info_block_with_port_schemas(
    source: &str,
    json: &str,
    port_schemas: Option<&str>,
) -> CliResult<String> {
    let start_marker = "// CERULION:INFO_START";
    let end_marker = "// CERULION:INFO_END";
    let start = source
        .find(start_marker)
        .ok_or_else(|| CliError::Validation("missing CERULION:INFO_START marker".to_string()))?;
    let end_rel = source[start..]
        .find(end_marker)
        .ok_or_else(|| CliError::Validation("missing CERULION:INFO_END marker".to_string()))?;
    let end = start + end_rel + end_marker.len();
    let escaped = json.replace('\\', "\\\\").replace('"', "\\\"");
    let schemas_line = port_schemas
        .map(|schemas| format!("{PORT_SCHEMAS_MARKER}{schemas}\n"))
        .unwrap_or_default();
    let block = format!(
        "// CERULION:INFO_START\nstatic INFO_BYTES: &[u8] = b\"{escaped}\\0\";\n{schemas_line}// CERULION:INFO_END"
    );
    let mut result = String::with_capacity(source.len() + block.len());
    result.push_str(&source[..start]);
    result.push_str(&block);
    result.push_str(&source[end..]);
    Ok(result)
}

/// Moves each port's `schema` name out of the Python info document: the
/// runtime reads the rest as INFO_BYTES, `node_metadata` reads the names from
/// the `CERULION:PORT_SCHEMAS` line.
fn split_port_schemas(mut value: serde_json::Value) -> Result<(String, Option<String>), String> {
    let mut schemas = serde_json::Map::new();
    for section in ["inputs", "outputs"] {
        let mut names = serde_json::Map::new();
        if let Some(ports) = value
            .get_mut(section)
            .and_then(serde_json::Value::as_array_mut)
        {
            for port in ports {
                let Some(object) = port.as_object_mut() else {
                    continue;
                };
                let Some(schema) = object.remove("schema") else {
                    continue;
                };
                let name = object
                    .get("name")
                    .and_then(serde_json::Value::as_str)
                    .ok_or_else(|| format!("{section} entry with a schema has no string name"))?;
                if !schema.is_string() {
                    return Err(format!("{section} port '{name}' schema is not a string"));
                }
                if names.insert(name.to_string(), schema).is_some() {
                    return Err(format!("{section} port '{name}' is declared twice"));
                }
            }
        }
        if !names.is_empty() {
            schemas.insert(section.to_string(), serde_json::Value::Object(names));
        }
    }
    let info = serde_json::to_string(&value).map_err(|error| error.to_string())?;
    if schemas.is_empty() {
        return Ok((info, None));
    }
    let schemas = serde_json::to_string(&serde_json::Value::Object(schemas))
        .map_err(|error| error.to_string())?;
    Ok((info, Some(schemas)))
}

struct RemoveOnDrop(PathBuf);

impl Drop for RemoveOnDrop {
    fn drop(&mut self) {
        let _ = std::fs::remove_file(&self.0);
    }
}

#[derive(Debug)]
struct PythonInterpreterPaths {
    libdir: String,
    site_paths: Vec<String>,
}

fn query_python_paths(python: &Path) -> Result<PythonInterpreterPaths, String> {
    let output = std::process::Command::new(python)
        .args([
            "-c",
            "import json, sysconfig; print(json.dumps({'libdir': sysconfig.get_config_var('LIBDIR') or '', 'purelib': sysconfig.get_path('purelib') or '', 'platlib': sysconfig.get_path('platlib') or ''}))",
        ])
        .output()
        .map_err(|error| format!("interpreter query failed: {error}"))?;
    if !output.status.success() {
        let stdout = String::from_utf8_lossy(&output.stdout);
        let stderr = String::from_utf8_lossy(&output.stderr);
        return Err(format!("interpreter query failed: {stderr}{stdout}"));
    }
    let paths: serde_json::Value = serde_json::from_slice(&output.stdout)
        .map_err(|error| format!("interpreter query returned invalid JSON: {error}"))?;
    let libdir = paths
        .get("libdir")
        .and_then(serde_json::Value::as_str)
        .unwrap_or_default()
        .to_string();
    let mut site_paths = Vec::new();
    for key in ["purelib", "platlib"] {
        let Some(path) = paths.get(key).and_then(serde_json::Value::as_str) else {
            continue;
        };
        if path.is_empty() || !Path::new(path).is_dir() || site_paths.iter().any(|p| p == path) {
            continue;
        }
        site_paths.push(path.to_string());
    }
    Ok(PythonInterpreterPaths { libdir, site_paths })
}

fn resolve_python(node_dir: &Path) -> PathBuf {
    std::env::var_os("CERULION_PYTHON")
        .map(std::path::PathBuf::from)
        .filter(|path| path.is_file())
        .or_else(|| {
            let path = node_dir
                .ancestors()
                .nth(2)
                .map(|root| root.join(".venv/bin/python"))?;
            path.is_file().then_some(path)
        })
        .unwrap_or_else(|| std::path::PathBuf::from("python3"))
}

fn python_paths_for_new_node(
    node_dir: &Path,
    node_type: &str,
) -> CliResult<PythonInterpreterPaths> {
    let python = resolve_python(node_dir);
    query_python_paths(&python).map_err(|reason| {
        CliError::Validation(format!(
            "Python interpreter {} path query failed for node '{}': {reason}",
            python.display(),
            node_type
        ))
    })
}

fn regenerate_python_info_with(node_dir: &Path, node_type: &str, python: &Path) -> CliResult<()> {
    static COUNTER: AtomicU64 = AtomicU64::new(0);
    let info_path = node_dir.join(format!(
        ".cerulion_info.{}.{}.json",
        std::process::id(),
        COUNTER.fetch_add(1, Ordering::Relaxed)
    ));
    let file = std::fs::OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(&info_path)
        .map_err(|error| CliError::BuildFailed {
            target: node_type.to_string(),
            reason: format!("python metadata temp file {}: {error}", info_path.display()),
        })?;
    drop(file);
    let script = r#"
import importlib.util
import pathlib
import sys
root = pathlib.Path.cwd()
spec = importlib.util.spec_from_file_location("node", root / "node.py")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
classes = [value for value in vars(module).values()
           if isinstance(value, type) and hasattr(value, "__cerulion_info__")]
if len(classes) != 1:
    raise RuntimeError("expected exactly one decorated node class")
pathlib.Path(sys.argv[1]).write_text(classes[0].__cerulion_info__(), encoding="utf-8")
"#;
    let _remove_info = RemoveOnDrop(info_path.clone());
    let output = std::process::Command::new(python)
        .arg("-c")
        .arg(script)
        .arg(&info_path)
        .current_dir(node_dir)
        .env(
            "CERULION_WORKSPACE",
            node_dir.parent().and_then(Path::parent).unwrap_or(node_dir),
        )
        .output()
        .map_err(|error| CliError::BuildFailed {
            target: node_type.to_string(),
            reason: format!("python metadata generation failed: {error}"),
        })?;
    if !output.status.success() {
        let stdout = String::from_utf8_lossy(&output.stdout);
        let stderr = String::from_utf8_lossy(&output.stderr);
        let reason = if stderr.contains("No module named 'cerulion'") {
            format!(
                "{stderr}{stdout}Python interpreter {} cannot import cerulion; install it with `{} -m pip install cerulion`",
                python.display(),
                python.display()
            )
        } else {
            format!("{stderr}{stdout}")
        };
        return Err(CliError::BuildFailed {
            target: node_type.to_string(),
            reason: format!("python metadata generation failed: {reason}"),
        });
    }
    let json = std::fs::read_to_string(&info_path).map_err(|error| CliError::BuildFailed {
        target: node_type.to_string(),
        reason: format!("python metadata generation produced invalid JSON: {error}"),
    })?;
    let mut documents = serde_json::Deserializer::from_str(&json).into_iter::<serde_json::Value>();
    let first = documents
        .next()
        .transpose()
        .map_err(|error| CliError::BuildFailed {
            target: node_type.to_string(),
            reason: format!("python metadata generation produced invalid JSON: {error}"),
        })?;
    let value = match (first, documents.next()) {
        (Some(value), None) => value,
        _ => {
            return Err(CliError::BuildFailed {
                target: node_type.to_string(),
                reason:
                    "python metadata generation produced invalid JSON: expected exactly one document"
                        .to_string(),
            });
        }
    };
    if !value.is_object()
        || !value.get("inputs").is_some_and(serde_json::Value::is_array)
        || !value
            .get("outputs")
            .is_some_and(serde_json::Value::is_array)
    {
        return Err(CliError::BuildFailed {
            target: node_type.to_string(),
            reason: "python metadata generation produced invalid JSON: expected object with inputs and outputs arrays"
                .to_string(),
        });
    }
    let (info_json, port_schemas) =
        split_port_schemas(value).map_err(|reason| CliError::BuildFailed {
            target: node_type.to_string(),
            reason: format!("python metadata generation produced invalid JSON: {reason}"),
        })?;
    let lib_path = node_dir.join("src/lib.rs");
    let source = std::fs::read_to_string(&lib_path)?;
    let updated =
        regenerate_info_block_with_port_schemas(&source, &info_json, port_schemas.as_deref())?;
    std::fs::write(lib_path, updated)?;
    Ok(())
}

fn regenerate_python_build_rs_with(
    node_dir: &Path,
    node_type: &str,
    python: &Path,
    paths: &PythonInterpreterPaths,
) -> CliResult<()> {
    let libdir = paths.libdir.as_str();
    if libdir.is_empty() {
        tracing::warn!(
            python = %python.display(),
            "Python interpreter LIBDIR is empty; the node cdylib will not carry a libpython rpath"
        );
    }

    let build_path = node_dir.join("build.rs");
    let source = std::fs::read_to_string(&build_path).unwrap_or_default();
    let start_marker = "// CERULION:LIBDIR_START";
    let end_marker = "// CERULION:LIBDIR_END";
    let updated = if let Some(start) = source.find(start_marker) {
        let end_rel = source[start..].find(end_marker).ok_or_else(|| {
            CliError::Validation("missing CERULION:LIBDIR_END marker".to_string())
        })?;
        let end = start + end_rel + end_marker.len();
        let replacement = templates::generate_python_build_rs(libdir);
        let replacement_start = replacement
            .find(start_marker)
            .expect("Python build template must have a LIBDIR start marker");
        let replacement_end = replacement
            .find(end_marker)
            .expect("Python build template must have a LIBDIR end marker")
            + end_marker.len();
        let mut result = String::with_capacity(source.len() + replacement.len());
        result.push_str(&source[..start]);
        result.push_str(&replacement[replacement_start..replacement_end]);
        result.push_str(&source[end..]);
        result
    } else {
        templates::generate_python_build_rs(libdir)
    };
    std::fs::write(build_path, updated).map_err(|error| CliError::BuildFailed {
        target: node_type.to_string(),
        reason: format!("Python build script generation failed: {error}"),
    })?;
    Ok(())
}

fn regenerate_python_sys_path_with(
    node_dir: &Path,
    node_type: &str,
    site_paths: &[String],
) -> CliResult<()> {
    let lib_path = node_dir.join("src/lib.rs");
    let source = std::fs::read_to_string(&lib_path)?;
    let block =
        templates::generate_python_sys_path_block(&node_dir.display().to_string(), site_paths);
    let updated = if let Some(start) = source.find("// CERULION:SYSPATH_START") {
        let end_rel = source[start..]
            .find("// CERULION:SYSPATH_END")
            .ok_or_else(|| {
                CliError::Validation("missing CERULION:SYSPATH_END marker".to_string())
            })?;
        let end = start + end_rel + "// CERULION:SYSPATH_END".len();
        let block_start = block
            .find("// CERULION:SYSPATH_START")
            .expect("Python sys.path template must have a start marker");
        let block_end = block
            .find("// CERULION:SYSPATH_END")
            .expect("Python sys.path template must have an end marker")
            + "// CERULION:SYSPATH_END".len();
        let mut result = String::with_capacity(source.len() + block.len());
        result.push_str(&source[..start]);
        result.push_str(&block[block_start..block_end]);
        result.push_str(&source[end..]);
        result
    } else {
        let start = source
            .find("    sys_path: [")
            .ok_or_else(|| CliError::Validation("missing Python sys_path list".to_string()))?;
        let end = start
            + source[start..].find("    ],").ok_or_else(|| {
                CliError::Validation("missing Python sys_path list end".to_string())
            })?
            + "    ],".len();
        let replacement = format!("{block},");
        let mut result = String::with_capacity(source.len() + replacement.len());
        result.push_str(&source[..start]);
        result.push_str(&replacement);
        result.push_str(&source[end..]);
        result
    };
    std::fs::write(&lib_path, updated).map_err(|error| CliError::BuildFailed {
        target: node_type.to_string(),
        reason: format!("Python sys.path generation failed: {error}"),
    })?;
    Ok(())
}

/// Add a `use` import for a schema type if not already present.
fn add_import_if_needed(source: &str, schema: &str) -> String {
    let import = templates::schema_to_import(schema);
    let use_line = format!("use {};", import);

    if source.contains(&use_line) {
        return source.to_string();
    }

    // Insert after the last `use` line
    let mut lines: Vec<&str> = source.lines().collect();
    let mut last_use_idx = None;
    for (i, line) in lines.iter().enumerate() {
        if line.starts_with("use ") {
            last_use_idx = Some(i);
        }
    }

    if let Some(idx) = last_use_idx {
        lines.insert(idx + 1, &use_line);
    } else {
        lines.insert(0, &use_line);
    }

    let mut result = lines.join("\n");
    if source.ends_with('\n') {
        result.push('\n');
    }
    result
}

/// Add a node crate to workspace `Cargo.toml` members.
pub(crate) fn add_workspace_member(cargo_toml: &Path, node_type: &str) -> CliResult<()> {
    let content = std::fs::read_to_string(cargo_toml)?;
    let member_entry = format!("\"nodes/{}\"", node_type);

    if content.contains(&member_entry) {
        return Ok(());
    }

    // Replace `members = [...]` with updated list
    let re = regex::Regex::new(r#"members\s*=\s*\[([^\]]*)\]"#)
        .map_err(|e| CliError::Validation(format!("regex error: {}", e)))?;

    let updated = if let Some(captures) = re.captures(&content) {
        let existing = captures.get(1).unwrap().as_str().trim();
        let new_members = if existing.is_empty() || existing == "\"nodes/*\"" {
            // Glob pattern or empty — keep as-is since we use glob
            return Ok(());
        } else {
            format!("{}, {}", existing.trim_end_matches(','), member_entry)
        };
        re.replace(&content, format!("members = [{}]", new_members))
            .to_string()
    } else {
        content
    };

    std::fs::write(cargo_toml, updated)?;
    Ok(())
}

/// Remove a node crate from workspace `Cargo.toml` members.
fn remove_workspace_member(cargo_toml: &Path, node_type: &str) -> CliResult<()> {
    let content = std::fs::read_to_string(cargo_toml)?;
    let member_entry = format!("\"nodes/{}\"", node_type);

    if !content.contains(&member_entry) {
        return Ok(());
    }

    // Remove the specific member entry (with optional trailing comma/whitespace)
    let pattern = format!(r#",?\s*"nodes/{}"\s*,?"#, regex::escape(node_type));
    let re = regex::Regex::new(&pattern)
        .map_err(|e| CliError::Validation(format!("regex error: {}", e)))?;
    let updated = re.replace(&content, "").to_string();

    std::fs::write(cargo_toml, updated)?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn setup_workspace() -> (tempfile::TempDir, std::path::PathBuf, std::path::PathBuf) {
        let tmp = tempfile::tempdir().unwrap();
        let ws = crate::workspace::workspace_create(tmp.path(), "test_ws").unwrap();
        let nodes_dir = ws.nodes_dir.clone();
        let cargo_toml = ws.root.join("Cargo.toml");
        (tmp, nodes_dir, cargo_toml)
    }

    #[test]
    fn test_node_create_source_has_prelude() {
        let (_tmp, nodes_dir, cargo_toml) = setup_workspace();
        node_create(&nodes_dir, &cargo_toml, "camera", None).unwrap();

        let src = std::fs::read_to_string(nodes_dir.join("camera/src/lib.rs")).unwrap();
        assert!(src.contains("use cerulion_core::prelude::*;"));
        // macro is bare or carries trigger hints; the
        // node type comes from the folder name, not from `type_name = ...`.
        assert!(src.contains("#[cerulion_node"));
        assert!(src.contains("#[cerulion_node_impl]"));
        assert!(src.contains("fn tick("));
        assert!(!src.contains("type_name"));
    }

    #[test]
    fn test_node_create_files_exist() {
        let (_tmp, nodes_dir, cargo_toml) = setup_workspace();
        node_create(&nodes_dir, &cargo_toml, "camera", None).unwrap();

        assert!(nodes_dir.join("camera/Cargo.toml").exists());
        assert!(nodes_dir.join("camera/src/lib.rs").exists());
    }

    #[test]
    fn python_port_schemas_move_out_of_info_bytes_and_round_trip_through_metadata() {
        let value: serde_json::Value = serde_json::from_str(
            r#"{"inputs":[{"name":"inp","schema":"std_msgs/Float64","schema_hash":7}],"outputs":[{"name":"out","schema":"geometry_msgs/Vector3","schema_hash":15293913555552287199}],"policy":{"period_ms":5}}"#,
        )
        .unwrap();
        let (info, schemas) = split_port_schemas(value).unwrap();
        assert!(!info.contains("\"schema\""), "{info}");
        assert!(info.contains("15293913555552287199"), "{info}");
        let schemas = schemas.unwrap();
        assert_eq!(
            schemas,
            r#"{"inputs":{"inp":"std_msgs/Float64"},"outputs":{"out":"geometry_msgs/Vector3"}}"#
        );
        let source = "// CERULION:INFO_START\nstatic INFO_BYTES: &[u8] = b\"old\\0\";\n// CERULION:INFO_END\n";
        let block = regenerate_info_block_with_port_schemas(source, &info, Some(&schemas)).unwrap();
        assert_eq!(
            regenerate_info_block_with_port_schemas(&block, &info, Some(&schemas)).unwrap(),
            block
        );
        let tmp = tempfile::TempDir::new().unwrap();
        let node_dir = tmp.path().join("pyecho");
        std::fs::create_dir_all(node_dir.join("src")).unwrap();
        std::fs::write(node_dir.join("src/lib.rs"), &block).unwrap();
        let metadata = parse_node_metadata(&node_dir).unwrap();
        assert_eq!(
            metadata.inputs[0].schema.as_deref(),
            Some("std_msgs/Float64")
        );
        assert_eq!(
            metadata.outputs[0].schema.as_deref(),
            Some("geometry_msgs/Vector3")
        );
    }

    #[test]
    fn python_port_schemas_reject_non_string_and_duplicate_ports() {
        let bad: serde_json::Value =
            serde_json::from_str(r#"{"inputs":[{"name":"inp","schema":3}],"outputs":[]}"#).unwrap();
        assert!(split_port_schemas(bad)
            .unwrap_err()
            .contains("not a string"));
        let dup: serde_json::Value = serde_json::from_str(
            r#"{"inputs":[],"outputs":[{"name":"o","schema":"a/B"},{"name":"o","schema":"a/B"}]}"#,
        )
        .unwrap();
        assert!(split_port_schemas(dup)
            .unwrap_err()
            .contains("declared twice"));
        let none: serde_json::Value =
            serde_json::from_str(r#"{"inputs":["inp"],"outputs":[]}"#).unwrap();
        assert_eq!(split_port_schemas(none).unwrap().1, None);
    }

    #[test]
    fn python_info_block_replacement_is_idempotent_and_requires_markers() {
        let source = "before\n// CERULION:INFO_START\nstatic INFO_BYTES: &[u8] = b\"old\\0\";\n// CERULION:INFO_END\nafter\n";
        let json = r#"{"inputs":[],"outputs":[],"policy":{"period_ms":1}}"#;
        let replaced = regenerate_info_block(source, json).unwrap();
        assert!(replaced.contains(r#"static INFO_BYTES: &[u8] = b"{\"inputs\":[],\"outputs\":[],\"policy\":{\"period_ms\":1}}\0";"#));
        assert_eq!(regenerate_info_block(&replaced, json).unwrap(), replaced);
        assert!(regenerate_info_block("no markers", json).is_err());
    }

    #[test]
    fn python_node_create_rejects_unusable_port_sets_before_writing() {
        let (_tmp, nodes_dir, cargo_toml) = setup_workspace();
        let port = |name: &str| ("geometry_msgs/Vector3".to_string(), name.to_string());
        let cases = [
            (
                NodeCreateOptions {
                    inputs: vec![port("inp")],
                    trigger: Some("missing".to_string()),
                    language: NodeLanguage::Python,
                    ..NodeCreateOptions::default()
                },
                None,
                "trigger 'missing' does not name an input port",
            ),
            (
                NodeCreateOptions {
                    inputs: vec![port("same")],
                    outputs: vec![port("same")],
                    language: NodeLanguage::Python,
                    ..NodeCreateOptions::default()
                },
                Some(cerulion_core::MacroPolicy::Period { period_ms: 10 }),
                "duplicate port name 'same'",
            ),
        ];
        for (options, policy, expected) in cases {
            let err = node_create_with_options(&nodes_dir, &cargo_toml, "bad", policy, &options)
                .unwrap_err();
            assert!(err.to_string().contains(expected), "{err}");
            assert!(!nodes_dir.join("bad").exists());
        }
    }

    #[test]
    fn python_node_create_writes_scaffold() {
        let (_tmp, nodes_dir, cargo_toml) = setup_workspace();
        let options = NodeCreateOptions {
            outputs: vec![("geometry_msgs/Vector3".to_string(), "out".to_string())],
            inputs: vec![("geometry_msgs/Vector3".to_string(), "inp".to_string())],
            language: NodeLanguage::Python,
            ..NodeCreateOptions::default()
        };
        node_create_with_options(
            &nodes_dir,
            &cargo_toml,
            "camera",
            Some(cerulion_core::MacroPolicy::Period { period_ms: 100 }),
            &options,
        )
        .unwrap();
        let root = nodes_dir.join("camera");
        assert!(std::fs::read_to_string(root.join("Cargo.toml"))
            .unwrap()
            .contains("cerulion_pynode"));
        assert!(std::fs::read_to_string(root.join("src/lib.rs"))
            .unwrap()
            .contains("export_node!"));
        let build_rs = std::fs::read_to_string(root.join("build.rs")).unwrap();
        assert!(build_rs.contains("// CERULION:LIBDIR_START"));
        assert!(build_rs.contains("cargo:rustc-link-arg=-Wl,-rpath,"));
        assert!(build_rs.contains("cargo:rerun-if-changed=build.rs"));
        assert!(std::fs::read_to_string(root.join("node.py"))
            .unwrap()
            .contains("cerulion"));
    }

    #[test]
    fn python_node_without_policy_or_trigger_is_rejected_before_write() {
        let (_tmp, nodes_dir, cargo_toml) = setup_workspace();
        let options = NodeCreateOptions {
            inputs: vec![("geometry_msgs/Vector3".to_string(), "inp".to_string())],
            outputs: vec![("geometry_msgs/Vector3".to_string(), "out".to_string())],
            language: NodeLanguage::Python,
            ..NodeCreateOptions::default()
        };
        let err =
            node_create_with_options(&nodes_dir, &cargo_toml, "missing_policy", None, &options)
                .unwrap_err();
        assert_eq!(
            err.to_string(),
            "Python nodes need a trigger policy: pass -T SCHEMA NAME or --policy period_ms=N (Python nodes cannot be modified after creation)"
        );
        assert!(!nodes_dir.join("missing_policy").exists());
    }

    #[test]
    fn python_node_trigger_without_policy_emits_data_trigger() {
        let (_tmp, nodes_dir, cargo_toml) = setup_workspace();
        let options = NodeCreateOptions {
            inputs: vec![("geometry_msgs/Vector3".to_string(), "inp".to_string())],
            outputs: vec![("geometry_msgs/Vector3".to_string(), "out".to_string())],
            trigger: Some("inp".to_string()),
            language: NodeLanguage::Python,
            ..NodeCreateOptions::default()
        };
        node_create_with_options(&nodes_dir, &cargo_toml, "triggered", None, &options).unwrap();
        let root = nodes_dir.join("triggered");
        let lib = std::fs::read_to_string(root.join("src/lib.rs")).unwrap();
        let source = std::fs::read_to_string(root.join("node.py")).unwrap();
        assert!(lib.contains(r#"\"data_trigger\":{\"input_name\":\"inp\"}"#));
        assert!(source.contains(r#"@cer.node(trigger="inp")"#));
    }

    #[test]
    fn python_node_rejects_raw_ffi_before_any_write() {
        let (_tmp, nodes_dir, cargo_toml) = setup_workspace();
        let options = NodeCreateOptions {
            language: NodeLanguage::Python,
            raw_ffi: true,
            ..NodeCreateOptions::default()
        };
        let err = node_create_with_options(
            &nodes_dir,
            &cargo_toml,
            "python_raw",
            Some(MacroPolicy::Period { period_ms: 1 }),
            &options,
        )
        .unwrap_err();
        assert_eq!(
            err.to_string(),
            "--raw-ffi applies to Rust nodes only; Python nodes always use the embedded-CPython template"
        );
        assert!(!nodes_dir.join("python_raw").exists());
    }

    #[test]
    fn python_port_names_are_validated_before_any_write() {
        let cases = [
            ("1abc", "is not a valid Python identifier"),
            ("with-dash", "is not a valid Python identifier"),
            ("class", "is a Python keyword"),
            ("None", "is a Python keyword"),
            ("loan", "is reserved by the Python node runtime"),
            ("tick", "is reserved by the Python node runtime"),
        ];
        for (index, (name, expected)) in cases.into_iter().enumerate() {
            for position in ["input", "output", "trigger"] {
                let (_tmp, nodes_dir, cargo_toml) = setup_workspace();
                let mut options = NodeCreateOptions {
                    language: NodeLanguage::Python,
                    ..NodeCreateOptions::default()
                };
                match position {
                    "input" => options
                        .inputs
                        .push(("geometry_msgs/Vector3".to_string(), name.to_string())),
                    "output" => options
                        .outputs
                        .push(("geometry_msgs/Vector3".to_string(), name.to_string())),
                    "trigger" => options.trigger = Some(name.to_string()),
                    _ => unreachable!(),
                }
                let node_type = format!("bad_{index}_{position}");
                let err = node_create_with_options(
                    &nodes_dir,
                    &cargo_toml,
                    &node_type,
                    Some(MacroPolicy::Period { period_ms: 1 }),
                    &options,
                )
                .unwrap_err();
                assert!(
                    err.to_string().contains(expected),
                    "{name} ({position}) produced: {err}"
                );
                assert!(!nodes_dir.join(node_type).exists());
            }
        }
    }

    #[test]
    fn python_prerequisite_failure_creates_no_directory() {
        let (_tmp, nodes_dir, cargo_toml) = setup_workspace();
        let options = NodeCreateOptions {
            language: NodeLanguage::Python,
            ..NodeCreateOptions::default()
        };
        let err = node_create_with_options(
            &nodes_dir,
            &cargo_toml,
            "unsupported",
            Some(MacroPolicy::External),
            &options,
        )
        .unwrap_err();
        assert!(err
            .to_string()
            .contains("Python node policy is not supported by the Python decorator"));
        assert!(!nodes_dir.join("unsupported").exists());
    }

    #[test]
    fn python_node_with_no_ports_and_period_policy_scaffolds_empty_class() {
        let (_tmp, nodes_dir, cargo_toml) = setup_workspace();
        let options = NodeCreateOptions {
            language: NodeLanguage::Python,
            ..NodeCreateOptions::default()
        };
        node_create_with_options(
            &nodes_dir,
            &cargo_toml,
            "empty_python",
            Some(MacroPolicy::Period { period_ms: 1 }),
            &options,
        )
        .unwrap();
        let source = std::fs::read_to_string(nodes_dir.join("empty_python/node.py")).unwrap();
        assert!(source.contains("class Node:"));
        assert!(source.contains("        return\n"));
        assert!(!source.contains("Vector3"));
    }

    #[test]
    fn python_node_refuses_every_modify_verb() {
        let (_tmp, nodes_dir, cargo_toml) = setup_workspace();
        let options = NodeCreateOptions {
            language: NodeLanguage::Python,
            inputs: vec![("geometry_msgs/Vector3".to_string(), "inp".to_string())],
            outputs: vec![("geometry_msgs/Vector3".to_string(), "out".to_string())],
            ..NodeCreateOptions::default()
        };
        node_create_with_options(
            &nodes_dir,
            &cargo_toml,
            "python_node",
            Some(MacroPolicy::Period { period_ms: 1 }),
            &options,
        )
        .unwrap();
        let lib_path = nodes_dir.join("python_node/src/lib.rs");
        let py_path = nodes_dir.join("python_node/node.py");
        let expected = "node 'python_node' is a Python node; edit nodes/python_node/node.py and run `cerulion node build python_node`";
        let verbs: [Box<dyn Fn() -> CliResult<()> + '_>; 7] = [
            Box::new(|| {
                node_modify_add_port(
                    &nodes_dir,
                    "python_node",
                    "extra",
                    Some("geometry_msgs/Vector3"),
                    true,
                    false,
                )
            }),
            Box::new(|| node_modify_clear_trigger(&nodes_dir, "python_node")),
            Box::new(|| node_modify_ext_trigger(&nodes_dir, "python_node", true)),
            Box::new(|| {
                node_modify_set_policy(
                    &nodes_dir,
                    "python_node",
                    &MacroPolicy::Period { period_ms: 2 },
                )
            }),
            Box::new(|| node_modify_set_period(&nodes_dir, "python_node", 2)),
            Box::new(|| node_modify_set_sync(&nodes_dir, "python_node", 2)),
            Box::new(|| node_modify_promote_input_to_trigger(&nodes_dir, "python_node", "inp")),
        ];
        for modify in verbs {
            let lib_before = std::fs::read(&lib_path).unwrap();
            let py_before = std::fs::read(&py_path).unwrap();
            let err = modify().unwrap_err();
            assert_eq!(err.to_string(), expected);
            assert_eq!(std::fs::read(&lib_path).unwrap(), lib_before);
            assert_eq!(std::fs::read(&py_path).unwrap(), py_before);
        }
    }

    #[cfg(unix)]
    #[test]
    fn regenerate_python_info_cleans_temp_file_on_every_result() {
        use std::os::unix::fs::PermissionsExt;

        fn make_stub(dir: &Path, name: &str, body: &str) -> PathBuf {
            let path = dir.join(name);
            std::fs::write(&path, format!("#!/bin/sh\n{body}\n")).unwrap();
            let mut permissions = std::fs::metadata(&path).unwrap().permissions();
            permissions.set_mode(0o755);
            std::fs::set_permissions(&path, permissions).unwrap();
            path
        }

        let (_tmp, nodes_dir, cargo_toml) = setup_workspace();
        let stub_dir = tempfile::tempdir().unwrap();
        let cases = [
            (
                "valid",
                r#"printf '%s' '{"inputs":[],"outputs":[]}' > "$3"; printf 'noise'"#,
                Some(r#"static INFO_BYTES: &[u8] = b"{\"inputs\":[],\"outputs\":[]}\0";"#),
            ),
            (
                "two",
                r#"printf '%s%s' '{"inputs":[],"outputs":[]}' '{"inputs":[],"outputs":[]}' > "$3""#,
                None,
            ),
            ("bad", r#"printf '%s' 'not json' > "$3""#, None),
            ("array", r#"printf '%s' '[]' > "$3""#, None),
            ("stderr", r#"printf '%s' 'stub stderr' >&2; exit 1"#, None),
        ];
        for (name, body, oracle) in cases {
            let node_type = format!("metadata_{name}");
            let options = NodeCreateOptions {
                language: NodeLanguage::Python,
                ..NodeCreateOptions::default()
            };
            node_create_with_options(
                &nodes_dir,
                &cargo_toml,
                &node_type,
                Some(MacroPolicy::Period { period_ms: 1 }),
                &options,
            )
            .unwrap();
            let node_dir = nodes_dir.join(&node_type);
            let stub = make_stub(stub_dir.path(), name, body);
            let result = regenerate_python_info_with(&node_dir, &node_type, &stub);
            match (name, result, oracle) {
                ("valid", Ok(()), Some(expected)) => {
                    let source = std::fs::read_to_string(node_dir.join("src/lib.rs")).unwrap();
                    assert!(source.contains(expected));
                }
                ("two", Err(err), None) => {
                    assert!(err.to_string().contains("expected exactly one document"));
                }
                ("bad", Err(err), None) => {
                    assert!(err.to_string().contains("produced invalid JSON"));
                }
                ("array", Err(err), None) => {
                    assert!(err
                        .to_string()
                        .contains("expected object with inputs and outputs arrays"));
                }
                ("stderr", Err(err), None) => {
                    assert!(err.to_string().contains("stub stderr"));
                }
                other => panic!("unexpected metadata result: {other:?}"),
            }
            assert!(!node_dir.join(".cerulion_info.json").exists());
            let leftovers = std::fs::read_dir(&node_dir)
                .unwrap()
                .filter_map(Result::ok)
                .map(|entry| entry.file_name())
                .filter(|name| {
                    let name = name.to_string_lossy();
                    name.starts_with(".cerulion_info.") && name.ends_with(".json")
                })
                .collect::<Vec<_>>();
            assert!(
                leftovers.is_empty(),
                "metadata temp files remain: {leftovers:?}"
            );
        }

        let node_type = "metadata_unique";
        let options = NodeCreateOptions {
            language: NodeLanguage::Python,
            ..NodeCreateOptions::default()
        };
        node_create_with_options(
            &nodes_dir,
            &cargo_toml,
            node_type,
            Some(MacroPolicy::Period { period_ms: 1 }),
            &options,
        )
        .unwrap();
        let node_dir = nodes_dir.join(node_type);
        std::fs::write(node_dir.join(".cerulion_info.json"), "preserve me").unwrap();
        let stub = make_stub(
            stub_dir.path(),
            "unique",
            r#"printf '%s\n' "$3" >> "$(dirname "$0")/paths.log"; printf '%s' '{"inputs":[],"outputs":[]}' > "$3""#,
        );
        regenerate_python_info_with(&node_dir, node_type, &stub).unwrap();
        regenerate_python_info_with(&node_dir, node_type, &stub).unwrap();
        assert_eq!(
            std::fs::read_to_string(node_dir.join(".cerulion_info.json")).unwrap(),
            "preserve me"
        );
        let paths = std::fs::read_to_string(stub_dir.path().join("paths.log")).unwrap();
        let paths = paths.lines().collect::<Vec<_>>();
        assert_eq!(paths.len(), 2);
        assert_ne!(paths[0], paths[1]);
    }

    #[cfg(unix)]
    #[test]
    fn regenerate_python_build_rs_bakes_interpreter_libdir() {
        use std::os::unix::fs::PermissionsExt;

        let (_tmp, nodes_dir, cargo_toml) = setup_workspace();
        let stub_dir = tempfile::tempdir().unwrap();
        let options = NodeCreateOptions {
            language: NodeLanguage::Python,
            ..NodeCreateOptions::default()
        };
        node_create_with_options(
            &nodes_dir,
            &cargo_toml,
            "rpath_node",
            Some(MacroPolicy::Period { period_ms: 1 }),
            &options,
        )
        .unwrap();
        let stub = nodes_dir.join("python-stub");
        let site = stub_dir.path().join("site");
        std::fs::create_dir(&site).unwrap();
        std::fs::write(
            &stub,
            format!(
                "#!/bin/sh\nif [ \"$1\" = \"-c\" ]; then printf '%s\\n' '{{\"libdir\":\"/opt/python/lib\",\"purelib\":\"{}\",\"platlib\":\"{}\"}}'; fi\n",
                site.display(),
                site.display()
            ),
        )
        .unwrap();
        let mut permissions = std::fs::metadata(&stub).unwrap().permissions();
        permissions.set_mode(0o755);
        std::fs::set_permissions(&stub, permissions).unwrap();

        let node_dir = nodes_dir.join("rpath_node");
        let paths = query_python_paths(&stub).unwrap();
        regenerate_python_build_rs_with(&node_dir, "rpath_node", &stub, &paths).unwrap();
        regenerate_python_sys_path_with(&node_dir, "rpath_node", &paths.site_paths).unwrap();
        let source = std::fs::read_to_string(node_dir.join("build.rs")).unwrap();
        assert!(source.contains("// CERULION:LIBDIR_START"));
        assert!(source.contains("const PYTHON_LIBDIR: &str = \"/opt/python/lib\";"));
        assert!(source.contains("cargo:rustc-link-arg=-Wl,-rpath,"));
        assert!(source.contains("cargo:rerun-if-changed=build.rs"));
        let lib_source = std::fs::read_to_string(node_dir.join("src/lib.rs")).unwrap();
        assert!(lib_source.contains("// CERULION:SYSPATH_START"));
        assert!(lib_source.contains(&format!("{:?}", site.display().to_string())));
        assert_eq!(
            lib_source
                .matches(&format!("{:?}", site.display().to_string()))
                .count(),
            1
        );
    }

    #[test]
    fn python_templates_match_exact_snapshot() {
        let inputs = vec![("inp".to_string(), "geometry_msgs/Vector3".to_string())];
        let outputs = vec![("out".to_string(), "geometry_msgs/Vector3".to_string())];
        assert_eq!(
            templates::generate_python_cargo_toml("echo", "/checkout/cerulion_pynode"),
            "[package]\nname = \"echo\"\nversion = \"0.1.0\"\nedition = \"2021\"\n\n[lib]\ncrate-type = [\"cdylib\"]\n\n[dependencies]\ncerulion_pynode = { path = \"/checkout/cerulion_pynode\" }\n\n[profile.release]\nstrip = true\n"
        );
        assert_eq!(
            templates::generate_python_lib_rs(
                "/workspace/nodes/echo",
                &inputs,
                &outputs,
                &serde_json::json!({"period_ms": 100}),
                &[],
            )
            .unwrap(),
            "// SPDX-License-Identifier: AGPL-3.0-only\n// CERULION:INFO_START\nstatic INFO_BYTES: &[u8] = b\"{\\\"inputs\\\":[{\\\"name\\\":\\\"inp\\\",\\\"schema_hash\\\":0}],\\\"outputs\\\":[{\\\"max_slice_len_default\\\":null,\\\"name\\\":\\\"out\\\",\\\"promise_within_ms\\\":null,\\\"schema_hash\\\":0,\\\"wire_fixed_size\\\":null}],\\\"policy\\\":{\\\"period_ms\\\":100}}\\0\";\n// CERULION:INFO_END\n\ncerulion_pynode::export_node! {\n    module: \"node\",\n    sys_path: [\n// CERULION:SYSPATH_START\n    \"/workspace/nodes/echo\",\n// CERULION:SYSPATH_END\n    ],\n    info: INFO_BYTES\n}\n"
        );
        assert_eq!(
            templates::generate_python_node_py(&inputs, &outputs, "period_ms=100", &[]),
            "import cerulion as cer\n\n\n@cer.node(period_ms=100)\nclass Node:\n    inp = cer.input(\"geometry_msgs/Vector3\")\n    out = cer.output(\"geometry_msgs/Vector3\")\n\n    def tick(self):\n        msg = self.inp\n        if msg is None:  # no frame received yet\n            return\n        out = self.out  # first touch loans the output; it is committed at tick end\n        # copy fields here, e.g. out.x = msg.x\n"
        );
    }

    #[test]
    fn python_cargo_toml_escapes_the_pynode_path() {
        let manifest =
            templates::generate_python_cargo_toml("echo", "/check\"out\\x/cerulion_pynode");
        let parsed: toml::Value = toml::from_str(&manifest).expect("manifest parses");
        assert_eq!(
            parsed["dependencies"]["cerulion_pynode"]["path"].as_str(),
            Some("/check\"out\\x/cerulion_pynode")
        );
    }

    #[test]
    fn python_sync_scaffold_marks_every_input_as_a_trigger() {
        let inputs = vec![
            ("left".to_string(), "Probe".to_string()),
            ("right".to_string(), "Probe".to_string()),
        ];
        let triggers = python_trigger_inputs(Some(&MacroPolicy::Sync { window_ms: 10 }), &inputs);
        assert_eq!(triggers, vec!["left".to_string(), "right".to_string()]);
        let source =
            templates::generate_python_node_py(&inputs, &[], "sync_window_ms=10", &triggers);
        assert!(source.contains("left = cer.input(\"Probe\", trigger=True)"));
        assert!(source.contains("right = cer.input(\"Probe\", trigger=True)"));
        assert_eq!(
            python_trigger_inputs(Some(&MacroPolicy::Period { period_ms: 5 }), &inputs),
            Vec::<String>::new()
        );
    }

    #[test]
    fn python_template_covers_all_ports_and_empty_sides() {
        let inputs = vec![
            ("left".to_string(), "Probe".to_string()),
            ("triggered".to_string(), "Probe".to_string()),
        ];
        let outputs = vec![
            ("first".to_string(), "Probe".to_string()),
            ("second".to_string(), "Probe".to_string()),
        ];
        let source = templates::generate_python_node_py(
            &inputs,
            &outputs,
            "trigger=\"triggered\"",
            &["triggered".to_string()],
        );
        assert!(source.contains("left = cer.input(\"Probe\")"));
        assert!(source.contains("triggered = cer.input(\"Probe\", trigger=True)"));
        assert!(source.contains("first = cer.output(\"Probe\")"));
        assert!(source.contains("second = cer.output(\"Probe\")"));
        assert!(
            templates::generate_python_node_py(&[], &[], "period_ms=1", &[])
                .contains("        return\n")
        );
        assert!(
            templates::generate_python_node_py(&inputs, &[], "period_ms=1", &[])
                .contains("msg = self.left")
        );
        assert!(
            templates::generate_python_node_py(&[], &outputs, "period_ms=1", &[])
                .contains("out = self.first")
        );
    }

    #[test]
    fn test_node_create_already_exists() {
        let (_tmp, nodes_dir, cargo_toml) = setup_workspace();
        node_create(&nodes_dir, &cargo_toml, "camera", None).unwrap();
        let result = node_create(&nodes_dir, &cargo_toml, "camera", None);
        assert!(result.is_err());
        assert!(result.unwrap_err().to_string().contains("already exists"));
    }

    #[test]
    fn test_node_modify_add_output() {
        let (_tmp, nodes_dir, cargo_toml) = setup_workspace();
        node_create(&nodes_dir, &cargo_toml, "camera", None).unwrap();
        node_modify_add_port(
            &nodes_dir,
            "camera",
            "image",
            Some("sensor_msgs::Image"),
            true,
            false,
        )
        .unwrap();

        let metadata =
            crate::node_metadata::parse_node_metadata(&nodes_dir.join("camera")).unwrap();
        assert_eq!(metadata.outputs.len(), 1);
        assert_eq!(metadata.outputs[0].name, "image");

        let src = std::fs::read_to_string(nodes_dir.join("camera/src/lib.rs")).unwrap();
        // The macro line and tick body are preserved; only the schema
        // import is added so the user can wire the matching
        // `#[output]` field manually.
        assert!(src.contains("use native_ros2_messages::sensor_msgs::Image;"));
    }

    #[test]
    fn test_node_modify_add_input() {
        let (_tmp, nodes_dir, cargo_toml) = setup_workspace();
        node_create(&nodes_dir, &cargo_toml, "detector", None).unwrap();
        node_modify_add_port(
            &nodes_dir,
            "detector",
            "image",
            Some("sensor_msgs::Image"),
            false,
            false,
        )
        .unwrap();

        let metadata =
            crate::node_metadata::parse_node_metadata(&nodes_dir.join("detector")).unwrap();
        assert_eq!(metadata.inputs.len(), 1);
        assert_eq!(metadata.inputs[0].name, "image");

        let src = std::fs::read_to_string(nodes_dir.join("detector/src/lib.rs")).unwrap();
        // Schema import still added; legacy `inputs(image)` arg is gone.
        assert!(src.contains("use native_ros2_messages::sensor_msgs::Image;"));
    }

    #[test]
    fn test_node_modify_duplicate_port_rejected() {
        let (_tmp, nodes_dir, cargo_toml) = setup_workspace();
        node_create(&nodes_dir, &cargo_toml, "camera", None).unwrap();
        node_modify_add_port(&nodes_dir, "camera", "image", None, true, false).unwrap();
        let result = node_modify_add_port(&nodes_dir, "camera", "image", None, true, false);
        assert!(result.is_err());
        assert!(result.unwrap_err().to_string().contains("already exists"));
    }

    #[test]
    fn test_node_delete_removes_type() {
        let (_tmp, nodes_dir, cargo_toml) = setup_workspace();
        node_create(&nodes_dir, &cargo_toml, "camera", None).unwrap();
        assert!(nodes_dir.join("camera").exists());

        node_delete(&nodes_dir, &cargo_toml, "camera").unwrap();
        assert!(!nodes_dir.join("camera").exists());
    }

    #[test]
    fn test_node_delete_not_found() {
        let (_tmp, nodes_dir, cargo_toml) = setup_workspace();
        let result = node_delete(&nodes_dir, &cargo_toml, "nonexistent");
        assert!(result.is_err());
        assert!(result.unwrap_err().to_string().contains("not found"));
    }

    #[test]
    fn test_node_list() {
        let (_tmp, nodes_dir, cargo_toml) = setup_workspace();
        node_create(&nodes_dir, &cargo_toml, "camera", None).unwrap();
        node_create(&nodes_dir, &cargo_toml, "detector", None).unwrap();

        let nodes = node_list(&nodes_dir).unwrap();
        assert_eq!(nodes.len(), 2);
        let types: Vec<&str> = nodes.iter().map(|n| n.node_type.as_str()).collect();
        assert!(types.contains(&"camera"));
        assert!(types.contains(&"detector"));
    }

    #[test]
    fn test_node_info() {
        let (_tmp, nodes_dir, cargo_toml) = setup_workspace();
        node_create(
            &nodes_dir,
            &cargo_toml,
            "camera",
            Some(cerulion_core::MacroPolicy::External),
        )
        .unwrap();

        let info = node_info(&nodes_dir, "camera").unwrap();
        assert_eq!(info.node_type, "camera");
        assert_eq!(info.policy, Some(cerulion_core::MacroPolicy::External));
    }

    #[test]
    fn test_node_info_not_found() {
        let (_tmp, nodes_dir, _cargo_toml) = setup_workspace();
        let result = node_info(&nodes_dir, "nonexistent");
        assert!(result.is_err());
    }

    // ─── `node_modify_ext_trigger` round-trip coverage ───────────

    #[test]
    fn test_node_modify_ext_trigger_enables_on_macro_node() {
        let (_tmp, nodes_dir, cargo_toml) = setup_workspace();
        node_create(&nodes_dir, &cargo_toml, "camera", None).unwrap();
        node_modify_ext_trigger(&nodes_dir, "camera", true).unwrap();
        // Round-trip oracle: parse_node_metadata sees policy = External.
        let info = node_info(&nodes_dir, "camera").unwrap();
        assert_eq!(info.policy, Some(cerulion_core::MacroPolicy::External));
        // Source-level oracle: the macro line carries `external`.
        let src = std::fs::read_to_string(nodes_dir.join("camera/src/lib.rs")).unwrap();
        assert!(
            src.contains("external"),
            "source must carry the `external` token, got:\n{}",
            src
        );
    }

    #[test]
    fn test_node_modify_ext_trigger_disables_on_macro_node() {
        // Disable the sole `external` arg on a node with no
        // `#[input(trigger)]` field. Note
        // that pure-bare `#[cerulion_node]` would fail macro
        // expansion (no trigger policy hint); the fix-up falls
        // back to `period_ms = 100` so the source stays buildable.
        let (_tmp, nodes_dir, cargo_toml) = setup_workspace();
        node_create(
            &nodes_dir,
            &cargo_toml,
            "camera",
            Some(cerulion_core::MacroPolicy::External),
        )
        .unwrap();
        node_modify_ext_trigger(&nodes_dir, "camera", false).unwrap();
        let info = node_info(&nodes_dir, "camera").unwrap();
        assert!(
            !matches!(info.policy, Some(cerulion_core::MacroPolicy::External)),
            "after disabling ext_trigger, policy must not be External; got {:?}",
            info.policy
        );
        let src = std::fs::read_to_string(nodes_dir.join("camera/src/lib.rs")).unwrap();
        assert!(
            !src.contains("#[cerulion_node()]"),
            "must NOT emit empty-parens form; got:\n{}",
            src
        );
        assert!(
            src.contains("#[cerulion_node(period_ms = 100)]"),
            "must fall back to period_ms = 100 when no trigger field exists; got:\n{}",
            src
        );
        syn::parse_file(&src).expect("source must parse after disable");
    }

    #[test]
    fn test_node_modify_ext_trigger_errors_on_raw_ffi_node() {
        // Raw-FFI nodes have no `#[cerulion_node]` macro to mutate;
        // the function must error rather than silently no-op.
        let (_tmp, nodes_dir, cargo_toml) = setup_workspace();
        let opts = NodeCreateOptions {
            raw_ffi: true,
            ..Default::default()
        };
        node_create_with_options(
            &nodes_dir,
            &cargo_toml,
            "raw_cam",
            Some(cerulion_core::MacroPolicy::Period { period_ms: 100 }),
            &opts,
        )
        .unwrap();
        let err = node_modify_ext_trigger(&nodes_dir, "raw_cam", true).unwrap_err();
        let msg = err.to_string();
        assert!(
            msg.contains("raw-FFI"),
            "error must mention raw-FFI; got: {}",
            msg
        );
        assert!(
            msg.contains("macro"),
            "error must point to the macro form; got: {}",
            msg
        );
    }

    #[test]
    fn test_node_modify_ext_trigger_errors_on_missing_node() {
        let (_tmp, nodes_dir, _cargo_toml) = setup_workspace();
        let err = node_modify_ext_trigger(&nodes_dir, "ghost", true).unwrap_err();
        assert!(matches!(err, CliError::NodeNotFound { .. }));
    }

    // ─── `-T/--trigger-input` flag coverage ──────────────────────

    #[test]
    fn test_node_modify_add_input_with_trigger_clears_period_ms_and_marks_field() {
        // Fresh source node (period_ms = 100) → add an input
        // designated as the trigger. Result: macro args lose
        // period_ms, the new field carries `#[input(trigger)]`.
        let (_tmp, nodes_dir, cargo_toml) = setup_workspace();
        node_create(&nodes_dir, &cargo_toml, "detector", None).unwrap();
        node_modify_add_port(
            &nodes_dir,
            "detector",
            "image",
            Some("sensor_msgs::Image"),
            false, // is_output
            true,  // is_trigger
        )
        .unwrap();
        let src = std::fs::read_to_string(nodes_dir.join("detector/src/lib.rs")).unwrap();
        assert!(
            !src.contains("period_ms"),
            "period_ms must be cleared; got:\n{}",
            src
        );
        assert!(
            src.contains("#[input(trigger)]"),
            "new field must carry `#[input(trigger)]`; got:\n{}",
            src
        );
        assert!(src.contains("image: Image,"));
        assert!(src.contains("use native_ros2_messages::sensor_msgs::Image;"));
        // Source must parse as Rust.
        syn::parse_file(&src).expect("post-trigger source must parse");
    }

    #[test]
    #[tracing_test::traced_test]
    fn test_node_modify_add_input_warns_when_existing_trigger_present() {
        // Scenario: node already has `#[input(trigger)] foo: T`
        // (its policy is `MacroPolicy::DataTrigger { input_name:
        // "foo" }`). The user runs `cerulion node modify -i
        // bar_schema bar` — adds a REGULAR input. The new input
        // does NOT fire the node; only the existing trigger does.
        // `node_modify_add_port` must emit a `tracing::warn!`
        // surfacing this semantic gap so the user notices.
        let (_tmp, nodes_dir, cargo_toml) = setup_workspace();
        node_create(&nodes_dir, &cargo_toml, "consumer", None).unwrap();
        // Establish the trigger first.
        node_modify_add_port(
            &nodes_dir,
            "consumer",
            "primary",
            Some("sensor_msgs::Image"),
            false, // is_output
            true,  // is_trigger
        )
        .unwrap();
        // Sanity: the node's policy is now DataTrigger.
        let md = parse_node_metadata(&nodes_dir.join("consumer")).unwrap();
        assert_eq!(
            md.policy,
            Some(cerulion_core::MacroPolicy::DataTrigger {
                input_name: "primary".to_string(),
            }),
        );
        // Add a regular (non-trigger) input to a node that already
        // has a trigger — the warn condition.
        node_modify_add_port(
            &nodes_dir,
            "consumer",
            "secondary",
            Some("sensor_msgs::Imu"),
            false, // is_output
            false, // is_trigger
        )
        .unwrap();
        // The warn must mention both the new input and the
        // existing trigger by name so the user can disambiguate.
        assert!(
            logs_contain("will NOT fire the node"),
            "expected warn about non-firing input; tracing logs did not match"
        );
        assert!(logs_contain("secondary"), "warn must name new input");
        assert!(logs_contain("primary"), "warn must name existing trigger");
        // The modify still succeeds — this is a warn, not an error.
        let src = std::fs::read_to_string(nodes_dir.join("consumer/src/lib.rs")).unwrap();
        assert!(src.contains("#[input(trigger)]"));
        assert!(src.contains("primary: Image"));
        assert!(src.contains("secondary: Imu"));
    }

    #[test]
    #[tracing_test::traced_test]
    fn test_node_modify_add_input_no_warn_when_no_existing_trigger() {
        // Inverse: a node with no `#[input(trigger)]` field gets
        // a new regular input. No warn — there's no semantic gap
        // to flag.
        let (_tmp, nodes_dir, cargo_toml) = setup_workspace();
        node_create(&nodes_dir, &cargo_toml, "consumer", None).unwrap();
        node_modify_add_port(
            &nodes_dir,
            "consumer",
            "first",
            Some("sensor_msgs::Image"),
            false,
            false,
        )
        .unwrap();
        assert!(
            !logs_contain("will NOT fire the node"),
            "must NOT warn when node has no existing trigger"
        );
    }

    #[test]
    #[tracing_test::traced_test]
    fn test_node_modify_add_input_no_warn_when_new_input_is_trigger() {
        // When the new input is itself marked `is_trigger=true`,
        // it replaces the existing trigger — the firing rule
        // changes but explicitly, so no warn is needed.
        let (_tmp, nodes_dir, cargo_toml) = setup_workspace();
        node_create(&nodes_dir, &cargo_toml, "consumer", None).unwrap();
        node_modify_add_port(
            &nodes_dir,
            "consumer",
            "primary",
            Some("sensor_msgs::Image"),
            false,
            true,
        )
        .unwrap();
        // Replace the trigger with a second `-T`-equivalent call.
        node_modify_add_port(
            &nodes_dir,
            "consumer",
            "replacement",
            Some("sensor_msgs::Imu"),
            false,
            true, // new input IS the trigger
        )
        .unwrap();
        assert!(
            !logs_contain("will NOT fire the node"),
            "must NOT warn when the new input is itself the trigger"
        );
    }

    #[test]
    fn test_node_modify_add_output_with_trigger_errors() {
        // `-T/--trigger-input` only applies to inputs.
        let (_tmp, nodes_dir, cargo_toml) = setup_workspace();
        node_create(&nodes_dir, &cargo_toml, "camera", None).unwrap();
        let err = node_modify_add_port(
            &nodes_dir,
            "camera",
            "image",
            Some("sensor_msgs::Image"),
            true, // is_output
            true, // is_trigger — invalid combination
        )
        .unwrap_err();
        let msg = err.to_string();
        assert!(
            msg.contains("input") && msg.contains("trigger"),
            "error must explain the constraint; got: {}",
            msg
        );
    }

    #[test]
    fn test_node_modify_add_input_with_trigger_on_raw_ffi_errors() {
        let (_tmp, nodes_dir, cargo_toml) = setup_workspace();
        let opts = NodeCreateOptions {
            raw_ffi: true,
            ..Default::default()
        };
        node_create_with_options(
            &nodes_dir,
            &cargo_toml,
            "raw_camera",
            Some(cerulion_core::MacroPolicy::Period { period_ms: 100 }),
            &opts,
        )
        .unwrap();
        let err = node_modify_add_port(
            &nodes_dir,
            "raw_camera",
            "image",
            Some("sensor_msgs::Image"),
            false,
            true,
        )
        .unwrap_err();
        let msg = err.to_string();
        assert!(
            msg.contains("raw-FFI"),
            "error must explain raw-FFI limitation; got: {}",
            msg
        );
    }

    #[test]
    fn test_node_modify_clear_trigger_demotes_field_and_inserts_period_ms() {
        // Node with `#[input(trigger)]` → clear-trigger →
        // `#[input]` (no trigger) AND macro args carry
        // `period_ms = 100`.
        let (_tmp, nodes_dir, cargo_toml) = setup_workspace();
        node_create(&nodes_dir, &cargo_toml, "detector", None).unwrap();
        node_modify_add_port(
            &nodes_dir,
            "detector",
            "image",
            Some("sensor_msgs::Image"),
            false,
            true,
        )
        .unwrap();
        // Sanity: pre-clear has no period_ms and the trigger attr.
        let src = std::fs::read_to_string(nodes_dir.join("detector/src/lib.rs")).unwrap();
        assert!(src.contains("#[input(trigger)]"));
        assert!(!src.contains("period_ms"));

        node_modify_clear_trigger(&nodes_dir, "detector").unwrap();
        let src = std::fs::read_to_string(nodes_dir.join("detector/src/lib.rs")).unwrap();
        assert!(
            !src.contains("#[input(trigger)]"),
            "trigger attr must be removed; got:\n{}",
            src
        );
        assert!(
            src.contains("#[input]\n    image: Image"),
            "field must remain as a regular #[input]; got:\n{}",
            src
        );
        assert!(
            src.contains("period_ms = 100"),
            "macro args must carry the period_ms fallback; got:\n{}",
            src
        );
        syn::parse_file(&src).expect("post-clear source must parse");
    }

    #[test]
    fn test_node_modify_clear_trigger_errors_on_raw_ffi() {
        let (_tmp, nodes_dir, cargo_toml) = setup_workspace();
        let opts = NodeCreateOptions {
            raw_ffi: true,
            ..Default::default()
        };
        node_create_with_options(
            &nodes_dir,
            &cargo_toml,
            "raw_camera",
            Some(cerulion_core::MacroPolicy::Period { period_ms: 100 }),
            &opts,
        )
        .unwrap();
        let err = node_modify_clear_trigger(&nodes_dir, "raw_camera").unwrap_err();
        assert!(err.to_string().contains("raw-FFI"));
    }

    #[test]
    fn test_node_modify_clear_trigger_errors_on_missing_node() {
        let (_tmp, nodes_dir, _) = setup_workspace();
        let err = node_modify_clear_trigger(&nodes_dir, "ghost").unwrap_err();
        assert!(matches!(err, CliError::NodeNotFound { .. }));
    }

    /// Create a macro-based node for testing (writes macro-style lib.rs).
    fn create_macro_node(nodes_dir: &Path, cargo_toml: &Path, node_type: &str) {
        // Create the node normally first (sets up dirs + Cargo.toml +
        // workspace member). The default `node_create` already
        // produces a macro-style lib.rs; we re-emit it here so any
        // future template change is explicit at the call site.
        node_create(nodes_dir, cargo_toml, node_type, None).unwrap();
        let metadata = parse_node_metadata(&nodes_dir.join(node_type)).unwrap();
        let macro_src = templates::generate_macro_lib_rs(&metadata, None);
        std::fs::write(nodes_dir.join(node_type).join("src/lib.rs"), macro_src).unwrap();
    }

    #[test]
    fn test_macro_node_modify_add_output() {
        let (_tmp, nodes_dir, cargo_toml) = setup_workspace();
        create_macro_node(&nodes_dir, &cargo_toml, "camera");

        node_modify_add_port(
            &nodes_dir,
            "camera",
            "image",
            Some("sensor_msgs::Image"),
            true,
            false,
        )
        .unwrap();

        // Metadata reflects the new port
        let metadata =
            crate::node_metadata::parse_node_metadata(&nodes_dir.join("camera")).unwrap();
        assert_eq!(metadata.outputs.len(), 1);
        assert_eq!(metadata.outputs[0].name, "image");
        assert_eq!(
            metadata.outputs[0].schema.as_deref(),
            Some("sensor_msgs/Image")
        );

        // The macro line stays bare apart from any policy attribute;
        // ports are declared by `#[input]` / `#[output]` field attrs.
        let src = std::fs::read_to_string(nodes_dir.join("camera/src/lib.rs")).unwrap();
        assert!(src.contains("#[cerulion_node"));
        assert!(src.contains("use native_ros2_messages::sensor_msgs::Image;"));
        // User code preserved
        assert!(src.contains("fn tick(&mut self"));
        assert!(src.contains("self.tick_count += 1"));
    }

    #[test]
    fn test_macro_node_modify_add_input() {
        let (_tmp, nodes_dir, cargo_toml) = setup_workspace();
        create_macro_node(&nodes_dir, &cargo_toml, "detector");

        node_modify_add_port(
            &nodes_dir,
            "detector",
            "image",
            Some("sensor_msgs::Image"),
            false,
            false,
        )
        .unwrap();

        // Metadata reflects the new port
        let metadata =
            crate::node_metadata::parse_node_metadata(&nodes_dir.join("detector")).unwrap();
        assert_eq!(metadata.inputs.len(), 1);
        assert_eq!(metadata.inputs[0].name, "image");

        // Source has the schema import; macro args
        // no longer enumerate ports.
        let src = std::fs::read_to_string(nodes_dir.join("detector/src/lib.rs")).unwrap();
        assert!(src.contains("use native_ros2_messages::sensor_msgs::Image;"));
    }

    #[test]
    fn test_macro_node_modify_add_multiple_ports() {
        let (_tmp, nodes_dir, cargo_toml) = setup_workspace();
        create_macro_node(&nodes_dir, &cargo_toml, "fusion");

        node_modify_add_port(&nodes_dir, "fusion", "lidar", None, false, false).unwrap();
        node_modify_add_port(&nodes_dir, "fusion", "camera", None, false, false).unwrap();
        node_modify_add_port(&nodes_dir, "fusion", "fused", None, true, false).unwrap();

        let metadata =
            crate::node_metadata::parse_node_metadata(&nodes_dir.join("fusion")).unwrap();
        assert_eq!(metadata.inputs.len(), 2);
        assert_eq!(metadata.outputs.len(), 1);

        // Sidecar is the source of truth for ports; the macro line stays
        // unchanged across `node modify` calls.
    }

    #[test]
    fn test_macro_node_modify_preserves_user_code() {
        let (_tmp, nodes_dir, cargo_toml) = setup_workspace();
        create_macro_node(&nodes_dir, &cargo_toml, "camera");

        // Add custom user code after the generated source
        let lib_path = nodes_dir.join("camera/src/lib.rs");
        let mut src = std::fs::read_to_string(&lib_path).unwrap();
        src.push_str("\n// USER: custom helper\nfn my_helper() -> u32 { 42 }\n");
        std::fs::write(&lib_path, src).unwrap();

        // Modify should preserve the custom code
        node_modify_add_port(&nodes_dir, "camera", "image", None, true, false).unwrap();

        let src = std::fs::read_to_string(&lib_path).unwrap();
        // macro line is preserved verbatim; user code
        // appended after the macro stays put.
        assert!(src.contains("// USER: custom helper"));
        assert!(src.contains("fn my_helper() -> u32 { 42 }"));
    }

    #[test]
    fn test_ffi_node_modify_still_works() {
        // Ensure the existing FFI path is not broken
        let (_tmp, nodes_dir, cargo_toml) = setup_workspace();
        let opts = NodeCreateOptions {
            raw_ffi: true,
            ..Default::default()
        };
        node_create_with_options(
            &nodes_dir,
            &cargo_toml,
            "camera",
            Some(cerulion_core::MacroPolicy::Period { period_ms: 100 }),
            &opts,
        )
        .unwrap();

        // Verify it's FFI-based (has markers, no macro)
        let src = std::fs::read_to_string(nodes_dir.join("camera/src/lib.rs")).unwrap();
        assert!(src.contains("// CERULION:INFO_START"));
        assert!(!src.contains("#[cerulion_node("));

        // Modify
        node_modify_add_port(
            &nodes_dir,
            "camera",
            "image",
            Some("sensor_msgs::Image"),
            true,
            false,
        )
        .unwrap();

        let src = std::fs::read_to_string(nodes_dir.join("camera/src/lib.rs")).unwrap();
        assert!(src.contains("\\\"image\\\""));
        assert!(src.contains("use native_ros2_messages::sensor_msgs::Image;"));

        let metadata =
            crate::node_metadata::parse_node_metadata(&nodes_dir.join("camera")).unwrap();
        assert_eq!(metadata.outputs.len(), 1);
        assert_eq!(metadata.outputs[0].name, "image");
    }

    #[test]
    fn test_macro_node_modify_metadata_updated() {
        let (_tmp, nodes_dir, cargo_toml) = setup_workspace();
        create_macro_node(&nodes_dir, &cargo_toml, "sensor");

        node_modify_add_port(
            &nodes_dir,
            "sensor",
            "data",
            Some("sensor_msgs::Image"),
            true,
            false,
        )
        .unwrap();
        node_modify_add_port(&nodes_dir, "sensor", "cmd", None, false, false).unwrap();

        let metadata =
            crate::node_metadata::parse_node_metadata(&nodes_dir.join("sensor")).unwrap();
        assert_eq!(metadata.node_type, "sensor");
        assert_eq!(metadata.outputs.len(), 1);
        assert_eq!(metadata.outputs[0].name, "data");
        assert_eq!(
            metadata.outputs[0].schema.as_deref(),
            Some("sensor_msgs/Image")
        );
        assert_eq!(metadata.inputs.len(), 1);
        assert_eq!(metadata.inputs[0].name, "cmd");
        assert!(metadata.inputs[0].schema.is_none());
    }

    #[test]
    fn test_node_create_invalid_name_rejected() {
        let (_tmp, nodes_dir, cargo_toml) = setup_workspace();

        let result = node_create(&nodes_dir, &cargo_toml, "my-node", None);
        assert!(result.is_err());
        assert!(result.unwrap_err().to_string().contains("alphanumeric"));

        let result = node_create(&nodes_dir, &cargo_toml, "2camera", None);
        assert!(result.is_err());
        assert!(result.unwrap_err().to_string().contains("digit"));

        let result = node_create(&nodes_dir, &cargo_toml, "", None);
        assert!(result.is_err());
        assert!(result.unwrap_err().to_string().contains("empty"));
    }

    // ── node_create_with_options tests ─────────────────────────

    #[test]
    fn test_node_create_macro_template_default() {
        let (_tmp, nodes_dir, cargo_toml) = setup_workspace();
        let opts = NodeCreateOptions::default();
        // `node_create_with_options` is the STRICT variant — it
        // requires an explicit policy when no inputs are declared.
        // Pass a 100ms period to exercise the macro template path.
        node_create_with_options(
            &nodes_dir,
            &cargo_toml,
            "camera",
            Some(cerulion_core::MacroPolicy::Period { period_ms: 100 }),
            &opts,
        )
        .unwrap();

        let src = std::fs::read_to_string(nodes_dir.join("camera/src/lib.rs")).unwrap();
        // macro carries period_ms hint (no type_name).
        assert!(src.contains("#[cerulion_node"));
        assert!(src.contains("#[cerulion_node_impl]"));
        assert!(!src.contains("extern \"C\""));
        assert!(!src.contains("type_name"));
    }

    #[test]
    fn test_node_create_with_options_rejects_unfirable_shape() {
        // The strict invariant: policy=None + no inputs + no
        // trigger → error. Matches the CLI's resolve_create_policy
        // 0-inputs-no-policy gate so engine + CLI contracts stay
        // aligned.
        let (_tmp, nodes_dir, cargo_toml) = setup_workspace();
        let opts = NodeCreateOptions::default();
        let err = node_create_with_options(&nodes_dir, &cargo_toml, "no_trigger", None, &opts)
            .expect_err("must reject unfirable shape");
        assert!(
            matches!(err, CliError::Validation(_)),
            "expected Validation error, got {err:?}"
        );
        assert!(
            err.to_string().contains("trigger policy"),
            "error must explain the missing trigger; got: {err}"
        );
    }

    #[test]
    fn test_node_create_rejects_duplicate_input_port_names() {
        // Engine API boundary mirror of the CLI's `-T` + `-i`
        // collision guard. Two inputs with the same `name` in
        // `NodeCreateOptions::inputs` would generate a `lib.rs`
        // declaring the field twice — invalid Rust. Reject at the
        // engine API for a clear diagnostic instead of a cargo
        // build failure on duplicate struct fields.
        let (_tmp, nodes_dir, cargo_toml) = setup_workspace();
        let opts = NodeCreateOptions {
            inputs: vec![
                ("sensor_msgs/Image".to_string(), "foo".to_string()),
                ("sensor_msgs/Imu".to_string(), "foo".to_string()),
            ],
            ..Default::default()
        };
        let err = node_create_with_options(&nodes_dir, &cargo_toml, "dup", None, &opts)
            .expect_err("must reject duplicate input port names");
        assert!(
            matches!(err, CliError::Validation(_)),
            "expected Validation error, got {err:?}"
        );
        assert!(
            err.to_string().contains("duplicate input port name 'foo'"),
            "error must name the duplicate port; got: {err}"
        );
    }

    #[test]
    fn test_node_create_convenience_supplies_bootstrap_default() {
        // `node_create` (the convenience wrapper) explicitly
        // supplies `Some(Period { 100 })` when called with
        // `policy = None`. This is documented at the function
        // boundary, not hidden inside the template generator.
        let (_tmp, nodes_dir, cargo_toml) = setup_workspace();
        node_create(&nodes_dir, &cargo_toml, "skeleton", None).unwrap();
        let src = std::fs::read_to_string(nodes_dir.join("skeleton/src/lib.rs")).unwrap();
        assert!(
            src.contains("#[cerulion_node(period_ms = 100)]"),
            "convenience `node_create(None)` must produce period_ms=100; got:\n{src}",
        );
    }

    #[test]
    fn test_node_create_raw_ffi_flag() {
        let (_tmp, nodes_dir, cargo_toml) = setup_workspace();
        let opts = NodeCreateOptions {
            raw_ffi: true,
            ..Default::default()
        };
        node_create_with_options(
            &nodes_dir,
            &cargo_toml,
            "camera",
            Some(cerulion_core::MacroPolicy::Period { period_ms: 100 }),
            &opts,
        )
        .unwrap();

        let src = std::fs::read_to_string(nodes_dir.join("camera/src/lib.rs")).unwrap();
        assert!(src.contains("extern \"C\""));
        assert!(src.contains("cerulion_node_info"));
        assert!(!src.contains("#[cerulion_node("));
    }

    #[test]
    fn test_node_create_with_output_port() {
        let (_tmp, nodes_dir, cargo_toml) = setup_workspace();
        let opts = NodeCreateOptions {
            outputs: vec![("sensor_msgs::Image".to_string(), "image".to_string())],
            ..Default::default()
        };
        // Output-only node has no inputs and no trigger; the
        // strict engine API requires an explicit policy.
        node_create_with_options(
            &nodes_dir,
            &cargo_toml,
            "camera",
            Some(cerulion_core::MacroPolicy::Period { period_ms: 100 }),
            &opts,
        )
        .unwrap();

        let src = std::fs::read_to_string(nodes_dir.join("camera/src/lib.rs")).unwrap();
        // outputs declared as struct fields with #[output].
        assert!(src.contains("#[output]"));
        assert!(src.contains("image: Image,"));
        assert!(src.contains("use native_ros2_messages::sensor_msgs::Image;"));

        let metadata =
            crate::node_metadata::parse_node_metadata(&nodes_dir.join("camera")).unwrap();
        assert_eq!(metadata.outputs.len(), 1);
        assert_eq!(metadata.outputs[0].name, "image");
        assert_eq!(
            metadata.outputs[0].schema.as_deref(),
            Some("sensor_msgs/Image")
        );
    }

    #[test]
    fn test_node_create_with_input_port() {
        let (_tmp, nodes_dir, cargo_toml) = setup_workspace();
        let opts = NodeCreateOptions {
            inputs: vec![("sensor_msgs::Image".to_string(), "image".to_string())],
            ..Default::default()
        };
        node_create_with_options(&nodes_dir, &cargo_toml, "detector", None, &opts).unwrap();

        let src = std::fs::read_to_string(nodes_dir.join("detector/src/lib.rs")).unwrap();
        // inputs declared as struct fields with #[input].
        assert!(src.contains("#[input]"));
        assert!(src.contains("image: Image,"));
        assert!(src.contains("use native_ros2_messages::sensor_msgs::Image;"));

        let metadata =
            crate::node_metadata::parse_node_metadata(&nodes_dir.join("detector")).unwrap();
        assert_eq!(metadata.inputs.len(), 1);
        assert_eq!(metadata.inputs[0].name, "image");
    }

    #[test]
    fn test_node_create_with_trigger() {
        let (_tmp, nodes_dir, cargo_toml) = setup_workspace();
        let opts = NodeCreateOptions {
            inputs: vec![("sensor_msgs::LaserScan".to_string(), "scan".to_string())],
            trigger: Some("scan".to_string()),
            ..Default::default()
        };
        node_create_with_options(&nodes_dir, &cargo_toml, "mapper", None, &opts).unwrap();

        let src = std::fs::read_to_string(nodes_dir.join("mapper/src/lib.rs")).unwrap();
        // trigger input emits #[input(trigger)] field.
        assert!(src.contains("#[input(trigger)]"));
        assert!(src.contains("scan: LaserScan,"));
    }

    // ─── Bare-schema resolution at the engine boundary ──

    /// Bare-schema resolution at `node create`: a BARE built-in name
    /// (`Vector3`, unique to geometry_msgs) supplied via output, input,
    /// AND trigger arms scaffolds the qualified import — never the
    /// bare `use Vector3;` (E0432).
    #[test]
    fn test_node_create_bare_builtin_schema_resolves_all_port_arms() {
        let (_tmp, nodes_dir, cargo_toml) = setup_workspace();
        let opts = NodeCreateOptions {
            outputs: vec![("Vector3".to_string(), "vec_out".to_string())],
            inputs: vec![("Vector3".to_string(), "vec_in".to_string())],
            trigger: Some("vec_in".to_string()),
            ..Default::default()
        };
        node_create_with_options(&nodes_dir, &cargo_toml, "vfilter", None, &opts).unwrap();

        let src = std::fs::read_to_string(nodes_dir.join("vfilter/src/lib.rs")).unwrap();
        assert!(
            src.contains("use native_ros2_messages::geometry_msgs::Vector3;"),
            "bare Vector3 must scaffold the qualified import; got:\n{src}"
        );
        assert!(
            !src.contains("use Vector3;"),
            "the E0432 bare import must NOT be emitted; got:\n{src}"
        );
        assert!(src.contains("#[input(trigger)]"));
        assert!(src.contains("vec_in: Vector3,"));
        assert!(src.contains("#[output]"));
        assert!(src.contains("vec_out: Vector3,"));
        syn::parse_file(&src).expect("generated source must parse");

        // Round-trip oracle: the stored schema is the QUALIFIED name.
        let metadata = parse_node_metadata(&nodes_dir.join("vfilter")).unwrap();
        assert_eq!(
            metadata.outputs[0].schema.as_deref(),
            Some("geometry_msgs/Vector3")
        );
        assert_eq!(
            metadata.inputs[0].schema.as_deref(),
            Some("geometry_msgs/Vector3")
        );
    }

    /// The bare-schema fix at `node modify`: bare `Vector3` through the
    /// output (-o), plain input (-i), and trigger input (-T) arms all
    /// splice the qualified import into an existing node's source.
    #[test]
    fn test_node_modify_bare_builtin_schema_resolves_all_port_arms() {
        let (_tmp, nodes_dir, cargo_toml) = setup_workspace();
        node_create(&nodes_dir, &cargo_toml, "vfilter", None).unwrap();
        node_modify_add_port(
            &nodes_dir,
            "vfilter",
            "vec_out",
            Some("Vector3"),
            true,
            false,
        )
        .unwrap();
        node_modify_add_port(
            &nodes_dir,
            "vfilter",
            "vec_in",
            Some("Vector3"),
            false,
            false,
        )
        .unwrap();
        node_modify_add_port(
            &nodes_dir,
            "vfilter",
            "vec_trig",
            Some("Vector3"),
            false,
            true,
        )
        .unwrap();

        let src = std::fs::read_to_string(nodes_dir.join("vfilter/src/lib.rs")).unwrap();
        assert!(
            src.contains("use native_ros2_messages::geometry_msgs::Vector3;"),
            "bare Vector3 must splice the qualified import; got:\n{src}"
        );
        assert!(
            !src.contains("use Vector3;"),
            "the E0432 bare import must NOT be spliced; got:\n{src}"
        );
        assert!(src.contains("vec_out: Vector3,"));
        assert!(src.contains("vec_in: Vector3,"));
        assert!(src.contains("vec_trig: Vector3,"));
        assert!(src.contains("#[input(trigger)]"));
        syn::parse_file(&src).expect("post-modify source must parse");

        let metadata = parse_node_metadata(&nodes_dir.join("vfilter")).unwrap();
        assert_eq!(
            metadata.outputs[0].schema.as_deref(),
            Some("geometry_msgs/Vector3")
        );
    }

    /// Fail-BEFORE-mutation (create): an ambiguous bare name (`Pose2D`
    /// lives in geometry_msgs AND vision_msgs) errors naming both
    /// candidates, and NO node directory is created.
    #[test]
    fn test_node_create_ambiguous_bare_schema_fails_before_mkdir() {
        let (_tmp, nodes_dir, cargo_toml) = setup_workspace();
        let opts = NodeCreateOptions {
            outputs: vec![("Pose2D".to_string(), "pose".to_string())],
            ..Default::default()
        };
        let err = node_create_with_options(
            &nodes_dir,
            &cargo_toml,
            "ambig",
            Some(cerulion_core::MacroPolicy::Period { period_ms: 100 }),
            &opts,
        )
        .expect_err("ambiguous bare schema must be rejected");
        assert!(
            matches!(err, CliError::Validation(_)),
            "expected Validation error, got {err:?}"
        );
        let msg = err.to_string();
        assert!(
            msg.contains("geometry_msgs/Pose2D") && msg.contains("vision_msgs/Pose2D"),
            "error must name both candidate packages: {msg}"
        );
        assert!(
            !nodes_dir.join("ambig").exists(),
            "a failed create must not leave a node directory behind"
        );
    }

    /// Fail-BEFORE-mutation (modify): ambiguous (`Pose2D`) and unknown
    /// (`Vectorr3`) bare names error with lib.rs BYTE-unchanged.
    #[test]
    fn test_node_modify_bad_bare_schema_leaves_source_untouched() {
        let (_tmp, nodes_dir, cargo_toml) = setup_workspace();
        node_create(&nodes_dir, &cargo_toml, "camera", None).unwrap();
        let lib_path = nodes_dir.join("camera/src/lib.rs");
        let before = std::fs::read_to_string(&lib_path).unwrap();

        let err = node_modify_add_port(&nodes_dir, "camera", "pose", Some("Pose2D"), true, false)
            .expect_err("ambiguous bare schema must be rejected");
        assert!(
            matches!(err, CliError::Validation(_)),
            "expected Validation error, got {err:?}"
        );
        let msg = err.to_string();
        assert!(
            msg.contains("geometry_msgs/Pose2D") && msg.contains("vision_msgs/Pose2D"),
            "error must name both candidate packages: {msg}"
        );

        let err = node_modify_add_port(&nodes_dir, "camera", "vec", Some("Vectorr3"), true, false)
            .expect_err("unknown bare schema must be rejected");
        assert!(
            matches!(err, CliError::SchemaNotFound { .. }),
            "expected SchemaNotFound error, got {err:?}"
        );

        let after = std::fs::read_to_string(&lib_path).unwrap();
        assert_eq!(
            before, after,
            "lib.rs must be byte-unchanged after failed modifies"
        );
    }

    /// Workspace schemas WIN over built-ins and stay BARE: a workspace
    /// `schemas/Foo.yaml` supplied as `-o Foo` passes through unchanged
    /// (`use Foo;`), never hijacked to a built-in path.
    #[test]
    fn test_node_create_workspace_bare_schema_passes_through() {
        let (_tmp, nodes_dir, cargo_toml) = setup_workspace();
        // `workspace_create` lays `schemas/` out as `nodes/`'s sibling.
        let schemas_dir = nodes_dir.parent().unwrap().join("schemas");
        std::fs::write(
            schemas_dir.join("Foo.yaml"),
            "schemas:\n  Foo:\n    fields:\n      uint32 a:\n",
        )
        .unwrap();
        let opts = NodeCreateOptions {
            outputs: vec![("Foo".to_string(), "foo_out".to_string())],
            ..Default::default()
        };
        node_create_with_options(
            &nodes_dir,
            &cargo_toml,
            "wsnode",
            Some(cerulion_core::MacroPolicy::Period { period_ms: 100 }),
            &opts,
        )
        .unwrap();

        let src = std::fs::read_to_string(nodes_dir.join("wsnode/src/lib.rs")).unwrap();
        assert!(
            src.contains("use Foo;"),
            "workspace bare name must pass through unchanged; got:\n{src}"
        );
        assert!(src.contains("foo_out: Foo,"));
        assert!(
            !src.contains("native_ros2_messages::Foo"),
            "workspace name must not be rewritten to a built-in path; got:\n{src}"
        );
    }

    /// Raw-FFI arm: bare `Vector3` resolves on BOTH raw-FFI paths.
    /// The INFO JSON carries port NAMES only (schema strings never
    /// appear in it — see `generate_info_fn` /
    /// `try_parse_raw_ffi_node`, which round-trips every port with
    /// `schema: None`), so the observable schema string on this path
    /// is the import `node modify` splices — it must be the QUALIFIED
    /// one. Resolution also gates the raw-FFI create path: an
    /// ambiguous bare name fails with no directory created.
    #[test]
    fn test_raw_ffi_bare_schema_resolves_qualified_import() {
        let (_tmp, nodes_dir, cargo_toml) = setup_workspace();
        let opts = NodeCreateOptions {
            outputs: vec![("Vector3".to_string(), "vec_out".to_string())],
            raw_ffi: true,
            ..Default::default()
        };
        node_create_with_options(
            &nodes_dir,
            &cargo_toml,
            "raw_vec",
            Some(cerulion_core::MacroPolicy::Period { period_ms: 100 }),
            &opts,
        )
        .unwrap();
        // Create succeeded — the bare name resolved (the gate arm is
        // pinned below). The INFO JSON carries the port name.
        let metadata = parse_node_metadata(&nodes_dir.join("raw_vec")).unwrap();
        assert_eq!(metadata.outputs[0].name, "vec_out");

        // Modify: the spliced import is the QUALIFIED schema string —
        // the raw-FFI path's observable resolution product.
        node_modify_add_port(
            &nodes_dir,
            "raw_vec",
            "vec_in",
            Some("Vector3"),
            false,
            false,
        )
        .unwrap();
        let src = std::fs::read_to_string(nodes_dir.join("raw_vec/src/lib.rs")).unwrap();
        assert!(
            src.contains("use native_ros2_messages::geometry_msgs::Vector3;"),
            "raw-FFI modify must splice the qualified import; got:\n{src}"
        );
        assert!(
            !src.contains("use Vector3;"),
            "the E0432 bare import must NOT appear; got:\n{src}"
        );
        let metadata = parse_node_metadata(&nodes_dir.join("raw_vec")).unwrap();
        assert_eq!(metadata.inputs[0].name, "vec_in");

        // The resolution gate applies to raw-FFI create too:
        // ambiguous bare name → Err, no node directory created.
        let opts = NodeCreateOptions {
            outputs: vec![("Pose2D".to_string(), "pose".to_string())],
            raw_ffi: true,
            ..Default::default()
        };
        let err = node_create_with_options(
            &nodes_dir,
            &cargo_toml,
            "raw_ambig",
            Some(cerulion_core::MacroPolicy::Period { period_ms: 100 }),
            &opts,
        )
        .expect_err("ambiguous bare schema must be rejected on the raw-FFI path");
        assert!(
            matches!(err, CliError::Validation(_)),
            "expected Validation error, got {err:?}"
        );
        assert!(
            !nodes_dir.join("raw_ambig").exists(),
            "a failed raw-FFI create must not leave a node directory behind"
        );
    }

    // ── The unsatisfied-dep WARN must match the printed notice ──

    /// A PROBED shortfall says "missing"; an UNPROBED one must not. Both arms
    /// are driven through the production `tracing::warn!` site, so this pins
    /// the emission and not merely a string constant.
    ///
    /// Split into two `#[traced_test]` bodies because the captured log is
    /// per-test: asserting the ABSENCE of the other arm's wording is what makes
    /// each half a real discriminator, and that is only possible when one arm
    /// ran alone.
    #[tracing_test::traced_test]
    #[test]
    fn a_probed_shortfall_warns_that_the_dependency_is_missing() {
        warn_unsatisfied_deps("camera_jpeg", &system_deps::PkgConfigTool::Present);
        assert!(
            logs_contain("optional system dependencies are missing"),
            "a real probe that came back empty IS a missing library"
        );
        assert!(
            !logs_contain("UNPROVEN"),
            "the probed arm must not hedge — the modules really were looked at"
        );
        assert!(logs_contain("camera_jpeg"), "the node must be named");
    }

    /// THE contradiction pin: with `pkg-config` unusable nothing was probed, so
    /// the warn must NOT assert the libraries are missing — the notice printed
    /// microseconds earlier says, verbatim, that this is not evidence of that.
    #[tracing_test::traced_test]
    #[test]
    fn an_unprobed_shortfall_warns_about_the_tool_not_the_library() {
        warn_unsatisfied_deps("camera_jpeg", &system_deps::PkgConfigTool::NotInstalled);
        assert!(
            logs_contain("`pkg-config` is not usable"),
            "the TOOL is the diagnosis when nothing could be probed"
        );
        assert!(
            logs_contain("UNPROVEN"),
            "and the deps are unproven, not absent"
        );
        assert!(
            logs_contain("NOT evidence that those libraries are absent"),
            "the warn must carry the same disclaimer as the notice"
        );
        assert!(
            !logs_contain("dependencies are missing"),
            "the warn must NOT contradict the notice it accompanies"
        );
    }

    /// The third tool state takes the same unprobed arm — an executable that
    /// cannot run probed exactly as much as one that is absent: nothing.
    #[tracing_test::traced_test]
    #[test]
    fn an_unusable_tool_takes_the_unprobed_warn_arm() {
        warn_unsatisfied_deps(
            "camera_jpeg",
            &system_deps::PkgConfigTool::Unusable("permission denied".to_string()),
        );
        assert!(logs_contain("UNPROVEN"));
        assert!(!logs_contain("dependencies are missing"));
    }

    /// The one spawn error worth a dedicated message: `cargo` itself is not
    /// on `PATH`, so the reason must name the toolchain and the fix rather
    /// than repeat the raw io::Error text ("No such file or directory (os
    /// error 2)"), which names a missing FILE, not a missing COMPILER. Pinned
    /// to the exact contract text (user-facing error text is a contract) with
    /// a HAND-WRITTEN version floor — never `env!`-derived, which is the same
    /// source the code under test reads — so an MSRV bump fails this on
    /// purpose and the new floor in the user-facing text is re-pinned
    /// consciously rather than drifting through unread.
    #[test]
    fn cargo_spawn_failure_reason_names_the_missing_toolchain_on_not_found() {
        // The workspace still exists, so a NotFound is a missing `cargo`.
        let ws = tempfile::tempdir().unwrap();
        let err = std::io::Error::from(std::io::ErrorKind::NotFound);
        let reason = cargo_spawn_failure_reason(&err, ws.path());
        assert_eq!(
            reason,
            "`cargo` was not found on PATH. `cerulion node build` compiles the node with \
             the Rust toolchain: install Rust 1.93+ from https://rustup.rs (plus a C \
             linker: build-essential on Ubuntu/Debian, Xcode Command Line Tools on macOS) \
             and retry"
        );
        // The KIND decides, not the message: a NotFound carrying custom text
        // still names the toolchain.
        let custom = std::io::Error::new(std::io::ErrorKind::NotFound, "custom spawn text");
        assert_eq!(cargo_spawn_failure_reason(&custom, ws.path()), reason);
    }

    /// A NotFound whose `current_dir` (the workspace) vanished after
    /// validation must NOT be blamed on a missing toolchain — std raises the
    /// same `ErrorKind::NotFound` for an absent working directory as for an
    /// absent executable, so the guard preserves the raw io text and never
    /// advertises a reinstall the user does not need.
    #[test]
    fn cargo_spawn_failure_reason_preserves_io_error_when_workspace_vanished() {
        let ws = tempfile::tempdir().unwrap();
        let gone = ws.path().join("removed");
        // `gone` never existed, so it stands in for a workspace removed
        // between the node_dir check and the spawn.
        let err = std::io::Error::new(
            std::io::ErrorKind::NotFound,
            "No such file or directory (os error 2)",
        );
        let reason = cargo_spawn_failure_reason(&err, &gone);
        assert_eq!(reason, err.to_string());
        assert!(
            !reason.contains("rustup.rs"),
            "vanished workspace must not advertise a toolchain reinstall: {reason}"
        );
    }

    /// Every OTHER spawn error (permissions, resource limits, ...) is not a
    /// toolchain problem — it keeps its io::Error text verbatim, whatever
    /// that text is. A table rather than one kind: a `!= PermissionDenied`
    /// special case passes a single-kind check while showing the rustup text
    /// for every other fork/exec failure, and a kind's default Display text
    /// says nothing about a real spawn failure's message surviving.
    #[test]
    fn cargo_spawn_failure_reason_keeps_the_io_error_text_for_other_kinds() {
        use std::io::{Error, ErrorKind};
        let errors = [
            Error::from(ErrorKind::PermissionDenied),
            Error::from(ErrorKind::WouldBlock),
            Error::from(ErrorKind::OutOfMemory),
            Error::other("fork: Resource temporarily unavailable (os error 35)"),
        ];
        let ws = tempfile::tempdir().unwrap();
        for err in &errors {
            let reason = cargo_spawn_failure_reason(err, ws.path());
            assert_eq!(
                reason,
                err.to_string(),
                "{:?} must pass through verbatim",
                err.kind()
            );
            assert!(!reason.contains("rustup.rs"), "{reason}");
        }
    }

    /// The advisory warning names the PATH probe, Cargo override uncertainty,
    /// both releases, and the stable-toolchain remedy.
    #[test]
    fn rustc_release_mismatch_warning_names_both_releases_and_the_fix() {
        let reason = rustc_release_mismatch_warning("lidar_sensor", "1.97.1", "1.93.0")
            .expect("differing releases must be reported");
        assert!(reason.contains("1.97.1"), "{reason}");
        assert!(reason.contains("1.93.0"), "{reason}");
        assert!(reason.contains("PATH rustc"), "{reason}");
        assert!(
            reason.contains("Cargo may select a different compiler"),
            "{reason}"
        );
        assert!(reason.contains("the build will continue"), "{reason}");
        assert!(
            reason.contains("rustup toolchain install 1.93.0"),
            "{reason}"
        );
        assert!(
            reason.contains("RUSTUP_TOOLCHAIN=1.93.0 cerulion node build lidar_sensor"),
            "{reason}"
        );
    }

    /// A matching release is not an error: the overwhelmingly common case (one
    /// ambient toolchain) must build exactly as it did before this check
    /// existed.
    #[test]
    fn rustc_release_mismatch_warning_is_none_on_a_match() {
        assert_eq!(
            rustc_release_mismatch_warning("lidar_sensor", "1.93.0", "1.93.0"),
            None
        );
    }

    /// An UNDETERMINED release (the pre-flight `rustc -vV` spawn or parse
    /// failed) must never itself refuse a build this check cannot evaluate:
    /// the probe is advisory and the load-time guard remains authoritative.
    #[test]
    fn rustc_release_mismatch_warning_is_none_when_detection_is_empty() {
        assert_eq!(
            rustc_release_mismatch_warning("lidar_sensor", "", "1.93.0"),
            None
        );
    }

    /// `detect_rustc_release` parses the real `rustc -vV` on this machine (no
    /// fixture needed) and must return a non-empty `release:` field, never
    /// `None`, when `rustc` is on `PATH` (true in every CI or dev environment
    /// that can run this suite at all).
    #[test]
    fn detect_rustc_release_reads_a_real_release_field() {
        let ws = tempfile::tempdir().unwrap();
        let detected =
            detect_rustc_release(ws.path()).expect("rustc -vV must be parseable in this suite");
        assert!(!detected.is_empty());
        // `release:` is always `<major>.<minor>.<patch>` optionally suffixed
        // (`-nightly`, `-beta.N`): never empty, never containing a space.
        assert!(!detected.contains(' '), "{detected}");
    }
}
