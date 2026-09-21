//! Versioned NDJSON protocol and command adapters.
//!
//! One JSON object per line in each direction. The daemon greets every
//! connection with [`hello_line`] (`{"hello":"cerulion-wsd","protocol":1}`);
//! a client that reads a `protocol` it does not know must disconnect. Every
//! request carries a numeric `id`, a `verb` from [`VERBS`] and an absolute
//! workspace `root`; the response echoes the `id` (`null` when the line could
//! not be parsed far enough to find one). Unknown fields are REJECTED
//! (`bad_request`) so a client cannot silently rely on a knob this version
//! ignores.
//!
//! Error codes, and what each obliges a client to do:
//! * `bad_request` — the line is not a request of this protocol version
//!   (malformed JSON, missing/unknown field, unsafe name, over-long line).
//!   Fix the client; do not retry.
//! * `unknown_verb` — the verb is not in [`VERBS`]. Negotiate per verb: a
//!   newer client probes, an older daemon answers this.
//! * `workspace_not_found` — `root` is not an absolute path to a Cerulion
//!   workspace root.
//! * `not_found` — the named graph, node type or schema does not exist in the
//!   workspace. The request was well-formed.
//! * `invalid_request` — the engine refused the operation (a duplicate port,
//!   a policy that needs an input the node lacks, ...); the message is the
//!   engine's own explanation, the same text the CLI prints.
//! * `version_conflict` — `expect_version` did not match the file's current
//!   SHA-256; nothing was written. Re-read and retry.
//! * `engine_error` — anything else the engine failed on (I/O, YAML, a build).

use cerulion_cli_engine::graph_cmd::{self, GraphLevelsReport};
use cerulion_cli_engine::node_cmd;
use cerulion_cli_engine::node_metadata::{NodeMetadata, PortDef};
use cerulion_cli_engine::workspace::CerulionWorkspace;
use serde::{Deserialize, Serialize};
use serde_json::{json, Value};
use sha2::{Digest, Sha256};
use std::path::Path;

use crate::{discover_workspace, WsdError};
use cerulion_cli_engine::error::CliError;
use cerulion_cli_engine::node_inspector::NodeInspector;
use cerulion_cli_engine::workspace_lock::{WorkspaceLock, WorkspaceReadLock};

/// The protocol version in the hello line. Bump it for any change a v1
/// client could misread — a renamed field, a changed response shape, a
/// verb whose meaning changed. ADDING a verb does not bump it: clients
/// discover verbs per request (`unknown_verb`).
pub const PROTOCOL_VERSION: u32 = 1;
/// The `hello` marker every connection is greeted with (parity with
/// `cerulion-netd`'s `"cerulion-netd"`).
pub const HELLO_MARKER: &str = "cerulion-wsd";
/// Every verb this version dispatches — the one list [`handle_line`] gates on
/// and the one the tests hold equal to the [`Request`] variants.
pub const VERBS: &[&str] = &[
    "workspace.info",
    "graph.read",
    "graph.validate",
    "graph.levels",
    "node.list",
    "node.info",
    "graph.stage_node",
    "node.modify",
];

/// One request line, tagged by `verb`. Every variant carries the correlation
/// `id` and the absolute workspace `root`; unknown fields are rejected.
#[derive(Debug, Deserialize)]
#[serde(tag = "verb", deny_unknown_fields)]
pub enum Request {
    /// Graph, node-type and workspace-schema names of the workspace.
    #[serde(rename = "workspace.info")]
    WorkspaceInfo { id: u64, root: String },
    /// `graphs/<graph>.yaml`: `raw` bytes, `parsed` config, file `version`.
    #[serde(rename = "graph.read")]
    GraphRead {
        id: u64,
        root: String,
        graph: String,
    },
    /// The engine's `graph validate` checks (`checks[{passed,label,detail,warning}]`).
    #[serde(rename = "graph.validate")]
    GraphValidate {
        id: u64,
        root: String,
        graph: String,
        #[serde(default)]
        prefer_release: bool,
    },
    /// The `graph levels` rendering plus any partition error.
    #[serde(rename = "graph.levels")]
    GraphLevels {
        id: u64,
        root: String,
        graph: String,
    },
    /// Every node type's metadata (ports, policy).
    #[serde(rename = "node.list")]
    NodeList { id: u64, root: String },
    /// One node type's metadata plus the `version` of `nodes/<type>/src/lib.rs`.
    #[serde(rename = "node.info")]
    NodeInfo {
        id: u64,
        root: String,
        node_type: String,
    },
    /// Append a node to `graphs/<graph>.yaml`. Its `outputs:` come from the
    /// node type's DECLARED ports (the same rule as `cerulion node stage`);
    /// only the input bindings are supplied here. Returns the new `raw` YAML
    /// and `version`. Optional `expect_version` makes it a compare-and-swap.
    #[serde(rename = "graph.stage_node")]
    GraphStageNode {
        id: u64,
        root: String,
        graph: String,
        node_type: String,
        #[serde(default)]
        node_id: Option<String>,
        #[serde(default)]
        inputs: Vec<StageInput>,
        #[serde(default)]
        expect_version: Option<String>,
    },
    /// Rewrite `nodes/<type>/src/lib.rs` with one [`ModifyOperation`] and
    /// return the node's new metadata plus `version`. Optional
    /// `expect_version` makes it a compare-and-swap.
    #[serde(rename = "node.modify")]
    NodeModify {
        id: u64,
        root: String,
        node_type: String,
        op: ModifyOperation,
        #[serde(default)]
        expect_version: Option<String>,
    },
}

/// One input binding for `graph.stage_node`: the node's input `name` wired to
/// a `source` (`[node_id,output]` or an absolute `/topic`).
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct StageInput {
    pub name: String,
    pub source: String,
}

/// The source edits `node.modify` performs, tagged by `op`. Each maps onto
/// one `cerulion node modify` gesture; trigger policy is edited ONLY through
/// `set_policy` (one operation, one encoding).
#[derive(Debug, Deserialize)]
#[serde(tag = "op", deny_unknown_fields)]
pub enum ModifyOperation {
    /// Declare a port (`cerulion node modify -i/-o SCHEMA NAME`).
    #[serde(rename = "add_port")]
    AddPort {
        port_name: String,
        #[serde(default)]
        schema: Option<String>,
        #[serde(default)]
        is_output: bool,
        #[serde(default)]
        is_trigger: bool,
    },
    /// Remove every `#[input(trigger)]` mark.
    #[serde(rename = "clear_trigger")]
    ClearTrigger,
    /// Set or clear the `external` trigger attribute.
    #[serde(rename = "ext_trigger")]
    ExtTrigger { ext: bool },
    /// Set the node's trigger policy (see [`PolicyRequest`]).
    #[serde(rename = "set_policy")]
    SetPolicy { policy: PolicyRequest },
}

/// A trigger policy, tagged by `kind` — the same vocabulary `node.info`
/// reports under `policy`.
#[derive(Debug, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum PolicyRequest {
    Period { period_ms: u64 },
    Sync { window_ms: u64 },
    UnboundedSync,
    External,
    DataTrigger { input_name: String },
}

/// One response line. `id` echoes the request's, or is `null` when the line
/// could not be parsed far enough to find one (so a legal `"id": 0` is never
/// ambiguous). Exactly one of `result` / `error` is present.
#[derive(Debug, Serialize)]
pub struct Response {
    pub id: Option<u64>,
    pub ok: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub result: Option<Value>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub error: Option<ResponseError>,
}

/// The `error` half of a failed response: a stable `code` (see the module
/// docs) and a human-readable `message`.
#[derive(Debug, Serialize)]
pub struct ResponseError {
    pub code: &'static str,
    pub message: String,
}

impl Response {
    /// A successful response carrying `result`.
    pub fn success(id: u64, result: Value) -> Self {
        Self {
            id: Some(id),
            ok: true,
            result: Some(result),
            error: None,
        }
    }

    /// A failed response; `id` is `None` when the request line had no usable id.
    pub fn failure(id: Option<u64>, code: &'static str, message: impl Into<String>) -> Self {
        Self {
            id,
            ok: false,
            result: None,
            error: Some(ResponseError {
                code,
                message: message.into(),
            }),
        }
    }
}

/// The greeting every connection receives first:
/// `{"hello":"cerulion-wsd","protocol":<PROTOCOL_VERSION>}`.
pub fn hello_line() -> String {
    serde_json::to_string(&json!({"hello": HELLO_MARKER, "protocol": PROTOCOL_VERSION}))
        .expect("hello JSON is infallible")
}

/// Decode and dispatch one request line. `inspector` is how `graph.validate`
/// reads a node library's info — the daemon passes a child-process inspector
/// so a library that aborts on load cannot take the daemon down.
pub fn handle_line(line: &str, inspector: &dyn NodeInspector) -> Response {
    let value: Value = match serde_json::from_slice(line.as_bytes()) {
        Ok(value) => value,
        Err(error) => return Response::failure(None, "bad_request", error.to_string()),
    };
    let id = value.get("id").and_then(Value::as_u64);
    match value.get("verb").and_then(Value::as_str) {
        Some(verb) if VERBS.contains(&verb) => {}
        Some(_) => return Response::failure(id, "unknown_verb", "unknown request verb"),
        None => return Response::failure(id, "bad_request", "request is missing string verb"),
    }
    let request: Request = match serde_json::from_value(value) {
        Ok(request) => request,
        Err(error) => return Response::failure(id, "bad_request", error.to_string()),
    };
    match dispatch(request, inspector) {
        Ok((id, result)) => Response::success(id, result),
        Err((id, code, message)) => Response::failure(Some(id), code, message),
    }
}

type DispatchError = (u64, &'static str, String);
type OperationError = (&'static str, String);

enum DispatchLock {
    Read(WorkspaceReadLock),
    Write(WorkspaceLock),
}

impl DispatchLock {
    fn root(&self) -> &Path {
        match self {
            Self::Read(lock) => lock.root(),
            Self::Write(lock) => lock.root(),
        }
    }
}

fn acquire_dispatch_lock(
    root: &str,
    id: u64,
    mutating: bool,
) -> Result<DispatchLock, DispatchError> {
    let lock = if mutating {
        WorkspaceLock::acquire_and_track_gitignore(Path::new(root))
            .map(DispatchLock::Write)
            .map_err(|error| lock_error(id, error))?
    } else {
        WorkspaceLock::acquire_read(Path::new(root))
            .map(DispatchLock::Read)
            .map_err(|error| lock_error(id, error))?
    };
    Ok(lock)
}

fn validate_workspace_candidate(root: &str, id: u64) -> Result<(), DispatchError> {
    let path = Path::new(root);
    let cargo_toml = path.join("Cargo.toml");
    if !path.is_absolute()
        || !path.is_dir()
        || !cargo_toml.is_file()
        || !path.join("graphs").is_dir()
    {
        return Err((
            id,
            "workspace_not_found",
            "workspace root is not a Cerulion workspace".to_string(),
        ));
    }
    let content = std::fs::read_to_string(cargo_toml).map_err(|error| {
        (
            id,
            "workspace_not_found",
            format!("workspace root is not a Cerulion workspace: {error}"),
        )
    })?;
    if !content.contains("[workspace]") {
        return Err((
            id,
            "workspace_not_found",
            "workspace root is not a Cerulion workspace".to_string(),
        ));
    }
    Ok(())
}

fn lock_error(id: u64, error: CliError) -> DispatchError {
    let code = if matches!(error, CliError::Io(ref error) if error.kind() == std::io::ErrorKind::NotFound)
    {
        "workspace_not_found"
    } else {
        "engine_error"
    };
    (id, code, error.to_string())
}

/// Map an engine failure onto the protocol's code set: a missing graph, node
/// type or schema is `not_found`; an engine refusal (`CliError::Validation`,
/// the text the CLI would print) is `invalid_request`; everything else is
/// `engine_error`.
fn engine_failure(error: CliError) -> OperationError {
    let code = match &error {
        CliError::GraphNotFound { .. }
        | CliError::NodeNotFound { .. }
        | CliError::SchemaNotFound { .. } => "not_found",
        CliError::Validation(_) => "invalid_request",
        _ => "engine_error",
    };
    (code, error.to_string())
}

fn dispatch(
    request: Request,
    inspector: &dyn NodeInspector,
) -> Result<(u64, Value), DispatchError> {
    let id = request_id(&request);
    if let Some(graph) = request_graph(&request) {
        if !is_safe_component(graph) {
            return Err((id, "bad_request", "invalid graph name".to_string()));
        }
    }
    if let Some(node_type) = request_node_type(&request) {
        if !is_safe_component(node_type) {
            return Err((id, "bad_request", "invalid node type".to_string()));
        }
    }
    let root = request_root(&request);
    validate_workspace_candidate(root, id)?;
    let lock = acquire_dispatch_lock(
        root,
        id,
        matches!(
            &request,
            Request::GraphStageNode { .. } | Request::NodeModify { .. }
        ),
    )?;
    let workspace = discover_workspace(lock.root()).map_err(|error| {
        let code = if matches!(
            error,
            WsdError::WorkspaceNotFound | WsdError::RelativeWorkspace
        ) {
            "workspace_not_found"
        } else {
            "engine_error"
        };
        (id, code, error.to_string())
    })?;
    let result = match request {
        Request::WorkspaceInfo { .. } => workspace_info(&workspace),
        Request::GraphRead { graph, .. } => graph_read(&workspace, &graph),
        Request::GraphValidate {
            graph,
            prefer_release,
            ..
        } => graph_validate(&workspace, &graph, prefer_release, inspector),
        Request::GraphLevels { graph, .. } => graph_levels(&workspace, &graph),
        Request::NodeList { .. } => node_list(&workspace),
        Request::NodeInfo { node_type, .. } => node_info(&workspace, &node_type),
        Request::GraphStageNode {
            graph,
            node_type,
            node_id,
            inputs,
            expect_version,
            ..
        } => graph_stage_node(
            &workspace,
            &graph,
            &node_type,
            node_id.as_deref(),
            &inputs,
            expect_version.as_deref(),
        ),
        Request::NodeModify {
            node_type,
            op,
            expect_version,
            ..
        } => node_modify(&workspace, &node_type, op, expect_version.as_deref()),
    };
    result
        .map(|value| (id, value))
        .map_err(|(code, message)| (id, code, message))
}

fn engine_error(error: String) -> OperationError {
    ("engine_error", error)
}

fn file_version(path: &Path) -> Result<String, OperationError> {
    let bytes = std::fs::read(path).map_err(|error| engine_failure(error.into()))?;
    Ok(format!("{:x}", Sha256::digest(bytes)))
}

fn check_expected_version(path: &Path, expected: Option<&str>) -> Result<(), OperationError> {
    let actual = file_version(path)?;
    if let Some(expected) = expected {
        if expected != actual {
            return Err((
                "version_conflict",
                format!("expected version {expected}, current version {actual}"),
            ));
        }
    }
    Ok(())
}

fn with_version(mut value: Value, version: String) -> Result<Value, OperationError> {
    value
        .as_object_mut()
        .ok_or_else(|| engine_error("metadata response is not an object".to_string()))?
        .insert("version".to_string(), Value::String(version));
    Ok(value)
}

fn request_graph(request: &Request) -> Option<&str> {
    match request {
        Request::GraphRead { graph, .. }
        | Request::GraphValidate { graph, .. }
        | Request::GraphLevels { graph, .. }
        | Request::GraphStageNode { graph, .. } => Some(graph),
        Request::WorkspaceInfo { .. }
        | Request::NodeList { .. }
        | Request::NodeInfo { .. }
        | Request::NodeModify { .. } => None,
    }
}

fn request_node_type(request: &Request) -> Option<&str> {
    match request {
        Request::NodeInfo { node_type, .. }
        | Request::GraphStageNode { node_type, .. }
        | Request::NodeModify { node_type, .. } => Some(node_type),
        Request::WorkspaceInfo { .. }
        | Request::GraphRead { .. }
        | Request::GraphValidate { .. }
        | Request::GraphLevels { .. }
        | Request::NodeList { .. } => None,
    }
}

fn is_safe_component(value: &str) -> bool {
    !value.is_empty()
        && value != "."
        && value != ".."
        && !value.contains('/')
        && !value.contains('\\')
        && !value.contains('\0')
}

fn request_id(request: &Request) -> u64 {
    match request {
        Request::WorkspaceInfo { id, .. }
        | Request::GraphRead { id, .. }
        | Request::GraphValidate { id, .. }
        | Request::GraphLevels { id, .. }
        | Request::NodeList { id, .. }
        | Request::NodeInfo { id, .. }
        | Request::GraphStageNode { id, .. }
        | Request::NodeModify { id, .. } => *id,
    }
}

fn request_root(request: &Request) -> &str {
    match request {
        Request::WorkspaceInfo { root, .. }
        | Request::GraphRead { root, .. }
        | Request::GraphValidate { root, .. }
        | Request::GraphLevels { root, .. }
        | Request::NodeList { root, .. }
        | Request::NodeInfo { root, .. }
        | Request::GraphStageNode { root, .. }
        | Request::NodeModify { root, .. } => root,
    }
}

fn workspace_info(workspace: &CerulionWorkspace) -> Result<Value, OperationError> {
    let graphs = graph_cmd::graph_list(&workspace.graphs_dir).map_err(engine_failure)?;
    let nodes = node_cmd::node_list(&workspace.nodes_dir)
        .map_err(engine_failure)?
        .into_iter()
        .map(|node| node.node_type)
        .collect::<Vec<_>>();
    let schemas = cerulion_cli_engine::schema_cmd::schema_list(&workspace.schemas_dir)
        .workspace
        .into_iter()
        .map(|schema| schema.name)
        .collect::<Vec<_>>();
    Ok(json!({"root": workspace.root, "graphs": graphs, "nodes": nodes, "schemas": schemas}))
}

fn graph_read(workspace: &CerulionWorkspace, graph: &str) -> Result<Value, OperationError> {
    let (config, raw) =
        graph_cmd::graph_read_raw(&workspace.graphs_dir, graph).map_err(engine_failure)?;
    let version = file_version(&workspace.graphs_dir.join(format!("{graph}.yaml")))?;
    Ok(json!({
        "raw": raw,
        "parsed": serde_json::to_value(config).map_err(|e| engine_error(e.to_string()))?,
        "version": version,
    }))
}

fn graph_validate(
    workspace: &CerulionWorkspace,
    graph: &str,
    prefer_release: bool,
    inspector: &dyn NodeInspector,
) -> Result<Value, OperationError> {
    // Never `graph_cmd::graph_validate`: that loads node libraries into THIS
    // process, and a library that aborts on load would kill the daemon.
    let report =
        graph_cmd::graph_validate_with_inspector(&workspace.root, graph, prefer_release, inspector)
            .map_err(engine_failure)?;
    Ok(json!({
        "graph_name": report.graph_name,
        "all_passed": report.all_passed(),
        "checks": report.checks.into_iter().map(|check| json!({
            "passed": check.passed, "label": check.label, "detail": check.detail, "warning": check.warning
        })).collect::<Vec<_>>()
    }))
}

fn graph_levels(workspace: &CerulionWorkspace, graph: &str) -> Result<Value, OperationError> {
    let GraphLevelsReport {
        rendered,
        partition_error,
    } = graph_cmd::graph_levels(&workspace.root, graph).map_err(engine_failure)?;
    Ok(json!({"rendered": rendered, "partition_error": partition_error}))
}

fn node_list(workspace: &CerulionWorkspace) -> Result<Value, OperationError> {
    let nodes = node_cmd::node_list(&workspace.nodes_dir)
        .map_err(engine_failure)?
        .into_iter()
        .map(node_metadata_value)
        .collect::<Result<Vec<_>, _>>()?;
    Ok(Value::Array(nodes))
}

fn node_info(workspace: &CerulionWorkspace, node_type: &str) -> Result<Value, OperationError> {
    let value = node_cmd::node_info(&workspace.nodes_dir, node_type)
        .map(node_metadata_value)
        .map_err(engine_failure)??;
    let version = file_version(
        &workspace
            .nodes_dir
            .join(node_type)
            .join("src")
            .join("lib.rs"),
    )?;
    with_version(value, version)
}

fn graph_stage_node(
    workspace: &CerulionWorkspace,
    graph: &str,
    node_type: &str,
    node_id: Option<&str>,
    inputs: &[StageInput],
    expect_version: Option<&str>,
) -> Result<Value, OperationError> {
    let path = workspace.graphs_dir.join(format!("{graph}.yaml"));
    check_expected_version(&path, expect_version)?;
    let input_defs = inputs
        .iter()
        .map(|input| (input.name.clone(), input.source.clone()))
        .collect::<Vec<_>>();
    // Outputs are the node type's DECLARED ports — the same engine fn
    // `cerulion node stage` uses, so the YAML can never disagree with source.
    graph_cmd::stage_declared_node(
        &workspace.nodes_dir,
        &workspace.graphs_dir,
        graph,
        node_type,
        node_id,
        &input_defs,
    )
    .map_err(engine_failure)?;
    let raw = std::fs::read_to_string(&path).map_err(|e| engine_failure(e.into()))?;
    let version = file_version(&path)?;
    Ok(json!({"raw": raw, "version": version}))
}

fn node_modify(
    workspace: &CerulionWorkspace,
    node_type: &str,
    operation: ModifyOperation,
    expect_version: Option<&str>,
) -> Result<Value, OperationError> {
    let path = workspace
        .nodes_dir
        .join(node_type)
        .join("src")
        .join("lib.rs");
    check_expected_version(&path, expect_version)?;
    match operation {
        ModifyOperation::AddPort {
            port_name,
            schema,
            is_output,
            is_trigger,
        } => node_cmd::node_modify_add_port(
            &workspace.nodes_dir,
            node_type,
            &port_name,
            schema.as_deref(),
            is_output,
            is_trigger,
        ),
        ModifyOperation::ClearTrigger => {
            node_cmd::node_modify_clear_trigger(&workspace.nodes_dir, node_type)
        }
        ModifyOperation::ExtTrigger { ext } => {
            node_cmd::node_modify_ext_trigger(&workspace.nodes_dir, node_type, ext)
        }
        ModifyOperation::SetPolicy {
            policy: PolicyRequest::DataTrigger { input_name },
        } => node_cmd::node_modify_promote_input_to_trigger(
            &workspace.nodes_dir,
            node_type,
            &input_name,
        ),
        ModifyOperation::SetPolicy { policy } => {
            node_cmd::node_modify_set_policy(&workspace.nodes_dir, node_type, &policy.into_core())
        }
    }
    .map_err(engine_failure)?;
    let value = node_cmd::node_info(&workspace.nodes_dir, node_type)
        .map(node_metadata_value)
        .map_err(engine_failure)??;
    let version = file_version(&path)?;
    with_version(value, version)
}

impl PolicyRequest {
    fn into_core(self) -> cerulion_core::MacroPolicy {
        match self {
            Self::Period { period_ms } => cerulion_core::MacroPolicy::Period { period_ms },
            Self::Sync { window_ms } => cerulion_core::MacroPolicy::Sync { window_ms },
            Self::UnboundedSync => cerulion_core::MacroPolicy::UnboundedSync,
            Self::External => cerulion_core::MacroPolicy::External,
            Self::DataTrigger { input_name } => {
                cerulion_core::MacroPolicy::DataTrigger { input_name }
            }
        }
    }
}

fn node_metadata_value(metadata: NodeMetadata) -> Result<Value, OperationError> {
    Ok(json!({
        "node_type": metadata.node_type,
        "policy": metadata.policy.map(policy_value),
        "inputs": metadata.inputs.into_iter().map(port_value).collect::<Result<Vec<_>, _>>()?,
        "outputs": metadata.outputs.into_iter().map(port_value).collect::<Result<Vec<_>, _>>()?,
    }))
}

fn policy_value(policy: cerulion_core::MacroPolicy) -> Value {
    match policy {
        cerulion_core::MacroPolicy::Period { period_ms } => {
            json!({"kind": "period", "period_ms": period_ms})
        }
        cerulion_core::MacroPolicy::Sync { window_ms } => {
            json!({"kind": "sync", "window_ms": window_ms})
        }
        cerulion_core::MacroPolicy::UnboundedSync => json!({"kind": "unbounded_sync"}),
        cerulion_core::MacroPolicy::External => json!({"kind": "external"}),
        cerulion_core::MacroPolicy::DataTrigger { input_name } => {
            json!({"kind": "data_trigger", "input_name": input_name})
        }
    }
}

fn port_value(port: PortDef) -> Result<Value, OperationError> {
    let backpressure = match port.backpressure {
        cerulion_core::graph::node::BackpressurePolicy::DropOldest => {
            json!({"policy": "drop_oldest"})
        }
        cerulion_core::graph::node::BackpressurePolicy::Block => json!({"policy": "block"}),
        cerulion_core::graph::node::BackpressurePolicy::Sample(n) => {
            json!({"policy": "sample", "n": n})
        }
    };
    Ok(json!({
        "name": port.name,
        "schema": port.schema,
        "trigger": port.trigger,
        "backpressure": backpressure,
    }))
}

#[cfg(test)]
mod tests {
    use super::*;
    use cerulion_cli_engine::node_inspector::InProcessInspector;

    /// A scratch directory following this crate's own fixture convention:
    /// `std::env::temp_dir()` + pid, removed on drop. `cerulion_wsd` carries no
    /// `[dev-dependencies]`, so nothing here reaches for `tempfile`.
    ///
    /// The name also carries a per-process COUNTER, because libtest runs tests
    /// in parallel within one pid and `new` opens with `remove_dir_all`: two
    /// tests that happened to pick the same tag would delete each other's
    /// fixture out from under them.
    #[cfg(unix)]
    struct Scratch(std::path::PathBuf);

    #[cfg(unix)]
    impl Scratch {
        fn new(tag: &str) -> Self {
            static NEXT: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
            let nth = NEXT.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
            let dir = std::env::temp_dir().join(format!(
                "cerulion-wsd-lock-{tag}-{}-{nth}",
                std::process::id()
            ));
            let _ = std::fs::remove_dir_all(&dir);
            std::fs::create_dir_all(&dir).expect("scratch dir");
            Self(dir)
        }
        fn path(&self) -> &std::path::Path {
            &self.0
        }
    }

    #[cfg(unix)]
    impl Drop for Scratch {
        fn drop(&mut self) {
            let _ = std::fs::remove_dir_all(&self.0);
        }
    }

    /// The READ dispatch is the ONE production caller of
    /// `WorkspaceLock::acquire_read`, so it is where the new refusal has to be
    /// shown to be LOUD rather than a silent success. A symlinked `.cerulion`
    /// means the shared lock would be taken on another tree's file while this
    /// workspace was reported readable; the dispatch must refuse instead.
    ///
    /// The anti-tautology half matters here: without it the assertion would
    /// pass on a fixture the dispatch rejected for some earlier reason (a
    /// missing `graphs/`, a `Cargo.toml` without `[workspace]`), which is the
    /// easy way to write a green test that proves nothing about the lock.
    #[cfg(unix)]
    #[test]
    fn a_read_dispatch_refuses_a_symlinked_lock_directory() {
        fn workspace(dir: &std::path::Path) {
            std::fs::write(dir.join("Cargo.toml"), "[workspace]\nmembers = []\n").unwrap();
            std::fs::create_dir_all(dir.join("graphs")).unwrap();
        }
        let request = |root: &std::path::Path| Request::WorkspaceInfo {
            id: 1,
            root: root.to_string_lossy().into_owned(),
        };
        let inspector = InProcessInspector;

        // ANTI-TAUTOLOGY: the same fixture WITHOUT the link gets past the lock.
        // It may still fail further in (this is not a full workspace tree), but
        // it must not fail with the lock's refusal.
        let healthy = Scratch::new("healthy");
        workspace(healthy.path());
        if let Err((_, _, message)) = dispatch(request(healthy.path()), &inspector) {
            assert!(
                !message.contains("SYMLINK"),
                "the control fixture was refused BY THE LOCK: {message}"
            );
        }

        let planted = Scratch::new("planted");
        workspace(planted.path());
        let outside = Scratch::new("outside");
        // The link's target holds a REAL lock file. Asserting the target stays
        // EMPTY could not fail: the read path creates nothing on any path, so
        // a walk that followed the link left an empty target just as empty.
        // With the file there, following the link would take `LOCK_SH` on it,
        // which a foreign `LOCK_EX` can see.
        std::fs::write(outside.path().join("workspace.lock"), b"").unwrap();
        std::os::unix::fs::symlink(outside.path(), planted.path().join(".cerulion")).unwrap();

        let (_, code, message) = dispatch(request(planted.path()), &inspector)
            .expect_err("a symlinked .cerulion must refuse the read dispatch");
        assert!(
            message.contains("SYMLINK"),
            "the refusal must name the condition; got [{code}] {message}"
        );
        let outsider = std::fs::OpenOptions::new()
            .read(true)
            .write(true)
            .open(outside.path().join("workspace.lock"))
            .unwrap();
        // SAFETY: `outsider` is a live open file owned by this call.
        let took = unsafe {
            libc::flock(
                std::os::fd::AsRawFd::as_raw_fd(&outsider),
                libc::LOCK_EX | libc::LOCK_NB,
            )
        };
        assert_eq!(
            took, 0,
            "the read dispatch took a shared lock on a file OUTSIDE this workspace"
        );
        // SAFETY: as above.
        unsafe { libc::flock(std::os::fd::AsRawFd::as_raw_fd(&outsider), libc::LOCK_UN) };
    }

    #[test]
    fn every_declared_verb_deserializes() {
        for verb in VERBS {
            let mut request = json!({"id": 1, "verb": verb, "root": "/workspace"});
            match *verb {
                "graph.read" | "graph.validate" | "graph.levels" => {
                    request["graph"] = json!("main");
                }
                "node.info" => request["node_type"] = json!("camera"),
                "graph.stage_node" => {
                    request["graph"] = json!("main");
                    request["node_type"] = json!("camera");
                }
                "node.modify" => {
                    request["node_type"] = json!("camera");
                    request["op"] = json!({"op": "clear_trigger"});
                }
                _ => {}
            }
            serde_json::from_value::<Request>(request).expect(verb);
        }
    }

    /// The `VERBS` gate and the `Request` enum must describe the SAME set:
    /// a variant missing from `VERBS` is unreachable (every test still
    /// passes), a `VERBS` entry with no variant is a promise nothing keeps.
    #[test]
    fn verbs_list_and_request_variants_are_one_set() {
        for verb in VERBS {
            let request = json!({"id": 1, "verb": verb, "root": "/workspace"});
            let error = serde_json::from_value::<Request>(request)
                .err()
                .map(|e| e.to_string())
                .unwrap_or_default();
            assert!(
                !error.contains("unknown variant"),
                "{verb} is in VERBS but has no Request variant: {error}"
            );
        }
        // Serde names the variants in its own error for an unknown tag.
        let error = serde_json::from_value::<Request>(json!({"verb": "nope", "id": 1}))
            .expect_err("unknown verb must fail")
            .to_string();
        let mut named: Vec<&str> = error.split('`').filter(|s| s.contains('.')).collect();
        named.sort_unstable();
        let mut listed: Vec<&str> = VERBS.to_vec();
        listed.sort_unstable();
        assert_eq!(named, listed, "serde knows a different verb set than VERBS");
    }

    #[test]
    fn unknown_fields_are_rejected_not_ignored() {
        let response = handle_line(
            r#"{"id": 7, "verb": "graph.stage_node", "root": "/workspace", "graph": "main", "node_type": "camera", "outputs": []}"#,
            &InProcessInspector,
        );
        let error = response.error.expect("error");
        assert_eq!(response.id, Some(7));
        assert_eq!(error.code, "bad_request");
        assert!(
            error.message.contains("unknown field `outputs`"),
            "{}",
            error.message
        );
    }

    #[test]
    fn an_unparseable_line_answers_with_a_null_id_and_a_legal_zero_id_is_echoed() {
        let unparseable = handle_line("{not json", &InProcessInspector);
        assert_eq!(unparseable.id, None);
        assert_eq!(unparseable.error.expect("error").code, "bad_request");
        assert_eq!(
            serde_json::to_value(handle_line("{not json", &InProcessInspector)).unwrap()["id"],
            Value::Null
        );
        let zero = handle_line(
            r#"{"id": 0, "verb": "workspace.nope", "root": "/w"}"#,
            &InProcessInspector,
        );
        assert_eq!(zero.id, Some(0));
    }

    #[test]
    fn engine_failures_map_onto_stable_codes() {
        assert_eq!(
            engine_failure(CliError::GraphNotFound {
                name: "main".into()
            })
            .0,
            "not_found"
        );
        assert_eq!(
            engine_failure(CliError::NodeNotFound {
                node_type: "cam".into()
            })
            .0,
            "not_found"
        );
        assert_eq!(
            engine_failure(CliError::Validation("dup".into())).0,
            "invalid_request"
        );
        assert_eq!(
            engine_failure(CliError::Io(std::io::Error::other("disk"))).0,
            "engine_error"
        );
    }

    #[test]
    fn known_and_unknown_verbs_have_distinct_parse_errors() {
        let unknown = handle_line(
            r#"{"id": 1, "verb": "workspace.nope", "root": "/workspace"}"#,
            &InProcessInspector,
        );
        assert_eq!(unknown.error.expect("error").code, "unknown_verb");

        let missing_fields = handle_line(r#"{"id": 1, "verb": "graph.read"}"#, &InProcessInspector);
        assert_eq!(missing_fields.error.expect("error").code, "bad_request");
    }

    #[test]
    fn graph_names_cannot_escape_the_workspace() {
        let response = handle_line(
            r#"{"id": 1, "verb": "graph.read", "root": "/workspace", "graph": "../secret"}"#,
            &InProcessInspector,
        );
        let error = response.error.expect("error");
        assert_eq!(error.code, "bad_request");
        assert_eq!(error.message, "invalid graph name");
    }

    #[test]
    fn node_types_cannot_escape_the_workspace() {
        let response = handle_line(
            r#"{"id": 1, "verb": "node.info", "root": "/workspace", "node_type": "../secret"}"#,
            &InProcessInspector,
        );
        let error = response.error.expect("error");
        assert_eq!(error.code, "bad_request");
        assert_eq!(error.message, "invalid node type");
    }
}
