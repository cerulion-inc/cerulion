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
//!
//! Every verb answers with exactly one response line except `node.build`,
//! which streams: zero or more `{"id", "event": "diagnostic", ...}` lines
//! while cargo compiles, then one `{"id", "event": "done", "ok"}` line. A
//! `node.build` the daemon refuses before cargo starts (unknown node, no such
//! workspace) is answered with the ordinary error response instead, so a
//! client reads lines for its `id` until one carries `error` or
//! `event: "done"`. Closing the connection cancels a build in progress.
//!
//! The graph edit verbs (`graph.wire`, `graph.unwire`, `graph.unstage`) add two
//! refusal codes, only ever sent in answer to those verbs, and an optional
//! `error.data` object that carries the structured detail of a refusal:
//! * `schema_mismatch`: the output and the input name different schemas;
//!   `data` is `{"expected": <input schema>, "found": <output schema>}`.
//! * `would_break`: `graph.unstage` of a node other nodes read from, without
//!   `force`; `data` is `{"wires": [{"from": {node, port}, "to": {node, port}}]}`.
//!   Nothing was written; resend with `force: true` to remove the node and
//!   those wires together.

use cerulion_cli_engine::graph_cmd::{self, GraphLevelsReport};
use cerulion_cli_engine::graph_edit::{self, EditOutcome, GraphEditError, PortRef, Wire};
use cerulion_cli_engine::node_cmd;
use cerulion_cli_engine::node_metadata::{NodeMetadata, PortDef};
use cerulion_cli_engine::workspace::CerulionWorkspace;
use serde::{Deserialize, Serialize};
use serde_json::{json, Value};
use sha2::{Digest, Sha256};
use std::path::Path;
use std::sync::atomic::{AtomicBool, Ordering};

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
    "graph.create",
    "node.create",
    "schema.create",
    "node.build",
    "graph.wire",
    "graph.unwire",
    "graph.unstage",
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
    /// Create `graphs/<name>.yaml` (`cerulion graph create NAME [-n PREFIX]`).
    /// Returns the new file's `version`.
    #[serde(rename = "graph.create")]
    GraphCreate {
        id: u64,
        root: String,
        name: String,
        #[serde(default)]
        prefix: Option<String>,
    },
    /// Scaffold `nodes/<node_type>/` (`cerulion node create`). Returns the
    /// `version` of its `src/lib.rs`.
    #[serde(rename = "node.create")]
    NodeCreate {
        id: u64,
        root: String,
        spec: NodeCreateSpec,
    },
    /// Create `schemas/<name>.yaml` (`cerulion schema create NAME`). Returns
    /// the new file's `version`.
    #[serde(rename = "schema.create")]
    SchemaCreate {
        id: u64,
        root: String,
        spec: SchemaCreateSpec,
    },
    /// Build `nodes/<node_type>` (`cerulion node build`) and STREAM cargo's
    /// compiler messages as `diagnostic` events, then a `done` event. See the
    /// module docs for the line sequence.
    #[serde(rename = "node.build")]
    NodeBuild {
        id: u64,
        root: String,
        node_type: String,
        #[serde(default)]
        release: bool,
    },
    /// Wire the `from` output to the `to` input: one `inputs:` entry is added to
    /// the consuming node, every other byte of the graph file is kept. Refused
    /// with `schema_mismatch` when the two ends name different schemas. Returns
    /// the new `raw` YAML and `version`. Optional `expect_version` makes it a
    /// compare-and-swap.
    #[serde(rename = "graph.wire")]
    GraphWire {
        id: u64,
        root: String,
        graph: String,
        from: WirePort,
        to: WirePort,
        #[serde(default)]
        expect_version: Option<String>,
    },
    /// Remove the wire from `from` to `to`: the matching `inputs:` entry of the
    /// consuming node is deleted. Returns the new `raw` YAML and `version`.
    /// Optional `expect_version` makes it a compare-and-swap.
    #[serde(rename = "graph.unwire")]
    GraphUnwire {
        id: u64,
        root: String,
        graph: String,
        from: WirePort,
        to: WirePort,
        #[serde(default)]
        expect_version: Option<String>,
    },
    /// Remove the node `node` (a node id in the graph) from
    /// `graphs/<graph>.yaml`. Refused with `would_break` and the list of wires
    /// when other nodes read from it, unless `force` is true, which removes
    /// those wires too. Returns the new `raw` YAML, `version` and the
    /// `removed_wires`. Optional `expect_version` makes it a compare-and-swap.
    #[serde(rename = "graph.unstage")]
    GraphUnstage {
        id: u64,
        root: String,
        graph: String,
        node: String,
        #[serde(default)]
        force: bool,
        #[serde(default)]
        expect_version: Option<String>,
    },
}

/// What `node.create` scaffolds, the flags of `cerulion node create` as data:
/// `inputs` are `-i`, `trigger_input` is `-T`, `outputs` are `-o`, `policy`
/// is `--policy` (same vocabulary as `node.modify`'s `set_policy`) and
/// `raw_ffi` is `--raw-ffi`. The engine applies the CLI's own defaulting rules
/// (a node with no input needs a `policy`; `trigger_input` implies a
/// data-trigger policy) and its own refusals.
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct NodeCreateSpec {
    pub node_type: String,
    #[serde(default)]
    pub inputs: Vec<PortSpec>,
    #[serde(default)]
    pub outputs: Vec<PortSpec>,
    #[serde(default)]
    pub trigger_input: Option<PortSpec>,
    #[serde(default)]
    pub policy: Option<PolicyRequest>,
    #[serde(default)]
    pub raw_ffi: bool,
}

/// One declared port: a `schema` (bare names resolve as `cerulion node create`
/// resolves them) and the port `name`.
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PortSpec {
    pub schema: String,
    pub name: String,
}

/// What `schema.create` creates.
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SchemaCreateSpec {
    pub name: String,
}

/// One end of a wire: a node id of the graph and a port name on it.
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct WirePort {
    pub node: String,
    pub port: String,
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
    /// Structured detail of a refusal (see the module docs), absent for every
    /// refusal that has none.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub data: Option<Value>,
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
                data: None,
            }),
        }
    }

    /// A failed response whose refusal carries structured `data`.
    pub fn failure_with_data(
        id: u64,
        code: &'static str,
        message: impl Into<String>,
        data: Value,
    ) -> Self {
        let mut response = Self::failure(Some(id), code, message);
        if let Some(error) = response.error.as_mut() {
            error.data = Some(data);
        }
        response
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
///
/// A single-response entry point: it cannot carry `node.build`'s stream, so
/// that verb is refused here. The daemon serves every verb through
/// [`handle_line_streaming`].
pub fn handle_line(line: &str, inspector: &dyn NodeInspector) -> Response {
    match parse_request(line) {
        Ok(Request::NodeBuild { id, .. }) => Response::failure(
            Some(id),
            "bad_request",
            "node.build streams events; it is served by handle_line_streaming",
        ),
        Ok(request) => respond(request, inspector),
        Err(response) => *response,
    }
}

/// Like [`handle_line`], but every line the daemon should send for this
/// request is handed to `emit` as it is produced: one response line for
/// every verb except `node.build`, which emits its `diagnostic` events live
/// and finishes with a `done` event (module docs). Setting `cancel` (the
/// client is gone) kills a build in progress.
pub fn handle_line_streaming(
    line: &str,
    inspector: &dyn NodeInspector,
    emit: &mut dyn FnMut(Value),
    cancel: &AtomicBool,
) {
    match parse_request(line) {
        Ok(Request::NodeBuild {
            id,
            root,
            node_type,
            release,
        }) => node_build(id, &root, &node_type, release, emit, cancel),
        Ok(request) => emit(response_value(&respond(request, inspector))),
        Err(response) => emit(response_value(&response)),
    }
}

/// Does this request line ask for `node.build`? That verb streams for as long
/// as cargo runs, so the daemon reads the client closing its end as a cancel.
pub fn cancels_on_hangup(line: &str) -> bool {
    line.contains("node.build")
        && serde_json::from_slice::<Value>(line.as_bytes())
            .is_ok_and(|value| value.get("verb").and_then(Value::as_str) == Some("node.build"))
}

fn response_value(response: &Response) -> Value {
    serde_json::to_value(response).unwrap_or_else(|error| {
        json!({"id": response.id, "ok": false,
               "error": {"code": "engine_error", "message": error.to_string()}})
    })
}

/// Decode one request line; a line that is not a request of this protocol
/// version comes back as the ready-made error response (boxed: a response
/// that can carry `error.data` is too large to return by value in `Err`).
fn parse_request(line: &str) -> Result<Request, Box<Response>> {
    let refuse = |id, code, message: String| Box::new(Response::failure(id, code, message));
    let value: Value = serde_json::from_slice(line.as_bytes())
        .map_err(|error| refuse(None, "bad_request", error.to_string()))?;
    let id = value.get("id").and_then(Value::as_u64);
    match value.get("verb").and_then(Value::as_str) {
        Some(verb) if VERBS.contains(&verb) => {}
        Some(_) => return Err(refuse(id, "unknown_verb", "unknown request verb".into())),
        None => {
            return Err(refuse(
                id,
                "bad_request",
                "request is missing string verb".into(),
            ))
        }
    }
    serde_json::from_value(value).map_err(|error| refuse(id, "bad_request", error.to_string()))
}

fn respond(request: Request, inspector: &dyn NodeInspector) -> Response {
    let mut data = None;
    match dispatch_with_data(request, inspector, &mut data) {
        Ok((id, result)) => Response::success(id, result),
        Err((id, code, message)) => match data {
            Some(data) => Response::failure_with_data(id, code, message, data),
            None => Response::failure(Some(id), code, message),
        },
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
/// or creating a graph or node type that already exists; the text the CLI
/// would print) is `invalid_request`; everything else is
/// `engine_error`.
fn engine_failure(error: CliError) -> OperationError {
    let code = match &error {
        CliError::GraphNotFound { .. }
        | CliError::NodeNotFound { .. }
        | CliError::SchemaNotFound { .. } => "not_found",
        CliError::Validation(_) | CliError::GraphExists { .. } | CliError::NodeExists { .. } => {
            "invalid_request"
        }
        _ => "engine_error",
    };
    (code, error.to_string())
}

#[cfg(test)]
fn dispatch(
    request: Request,
    inspector: &dyn NodeInspector,
) -> Result<(u64, Value), DispatchError> {
    dispatch_with_data(request, inspector, &mut None)
}

/// [`dispatch`], plus `data`: the structured detail of a refusal that has some
/// (`schema_mismatch`, `would_break`), left `None` otherwise.
fn dispatch_with_data(
    request: Request,
    inspector: &dyn NodeInspector,
    data: &mut Option<Value>,
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
    if let Request::SchemaCreate { spec, .. } = &request {
        if !is_schema_name(&spec.name)
            || !cerulion_cli_engine::schema_cmd::schema_name_makes_a_string_key(&spec.name)
        {
            return Err((id, "bad_request", "invalid schema name".to_string()));
        }
    }
    let root = request_root(&request);
    validate_workspace_candidate(root, id)?;
    let lock = acquire_dispatch_lock(
        root,
        id,
        matches!(
            &request,
            Request::GraphStageNode { .. }
                | Request::NodeModify { .. }
                | Request::GraphCreate { .. }
                | Request::NodeCreate { .. }
                | Request::SchemaCreate { .. }
                | Request::GraphWire { .. }
                | Request::GraphUnwire { .. }
                | Request::GraphUnstage { .. }
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
        Request::GraphCreate { name, prefix, .. } => {
            graph_create(&workspace, &name, prefix.as_deref())
        }
        Request::NodeCreate { spec, .. } => node_create(&workspace, spec),
        Request::SchemaCreate { spec, .. } => schema_create(&workspace, &spec.name),
        Request::NodeBuild { .. } => Err((
            "bad_request",
            "node.build streams events; it is served by handle_line_streaming".to_string(),
        )),
        Request::GraphWire {
            graph,
            from,
            to,
            expect_version,
            ..
        } => graph_edit_call(&workspace, &graph, expect_version.as_deref(), data, |ws| {
            graph_edit::graph_wire(ws, &graph, &from.into(), &to.into())
        }),
        Request::GraphUnwire {
            graph,
            from,
            to,
            expect_version,
            ..
        } => graph_edit_call(&workspace, &graph, expect_version.as_deref(), data, |ws| {
            graph_edit::graph_unwire(ws, &graph, &from.into(), &to.into())
        }),
        Request::GraphUnstage {
            graph,
            node,
            force,
            expect_version,
            ..
        } => graph_edit_call(&workspace, &graph, expect_version.as_deref(), data, |ws| {
            graph_edit::graph_unstage(ws, &graph, &node, force)
        }),
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
        | Request::GraphStageNode { graph, .. }
        | Request::GraphWire { graph, .. }
        | Request::GraphUnwire { graph, .. }
        | Request::GraphUnstage { graph, .. } => Some(graph),
        Request::GraphCreate { name, .. } => Some(name),
        Request::WorkspaceInfo { .. }
        | Request::NodeList { .. }
        | Request::NodeInfo { .. }
        | Request::NodeModify { .. }
        | Request::NodeCreate { .. }
        | Request::SchemaCreate { .. }
        | Request::NodeBuild { .. } => None,
    }
}

fn request_node_type(request: &Request) -> Option<&str> {
    match request {
        Request::NodeInfo { node_type, .. }
        | Request::GraphStageNode { node_type, .. }
        | Request::NodeModify { node_type, .. }
        | Request::NodeBuild { node_type, .. } => Some(node_type),
        Request::NodeCreate { spec, .. } => Some(&spec.node_type),
        Request::WorkspaceInfo { .. }
        | Request::GraphRead { .. }
        | Request::GraphValidate { .. }
        | Request::GraphLevels { .. }
        | Request::NodeList { .. }
        | Request::GraphCreate { .. }
        | Request::SchemaCreate { .. }
        | Request::GraphWire { .. }
        | Request::GraphUnwire { .. }
        | Request::GraphUnstage { .. } => None,
    }
}

/// A schema name is the file stem and, PascalCased, the YAML key of the new
/// file. It takes what `cerulion schema create` takes in practice: ASCII
/// letters, digits, `_` and `-`, in any order. Anything else (a colon, a
/// newline, a path separator) would write a key that cannot be read back, and
/// so would a name that PascalCases to a number or a boolean (`123`, `true`),
/// which the engine's `schema_name_makes_a_string_key` refuses.
fn is_schema_name(value: &str) -> bool {
    !value.is_empty()
        && value
            .chars()
            .all(|c| c.is_ascii_alphanumeric() || c == '_' || c == '-')
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
        | Request::NodeModify { id, .. }
        | Request::GraphCreate { id, .. }
        | Request::NodeCreate { id, .. }
        | Request::SchemaCreate { id, .. }
        | Request::NodeBuild { id, .. }
        | Request::GraphWire { id, .. }
        | Request::GraphUnwire { id, .. }
        | Request::GraphUnstage { id, .. } => *id,
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
        | Request::NodeModify { root, .. }
        | Request::GraphCreate { root, .. }
        | Request::NodeCreate { root, .. }
        | Request::SchemaCreate { root, .. }
        | Request::NodeBuild { root, .. }
        | Request::GraphWire { root, .. }
        | Request::GraphUnwire { root, .. }
        | Request::GraphUnstage { root, .. } => root,
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

fn wire_value(wire: &Wire) -> Value {
    json!({
        "from": {"node": wire.from.node, "port": wire.from.port},
        "to": {"node": wire.to.node, "port": wire.to.port},
    })
}

impl From<WirePort> for PortRef {
    fn from(port: WirePort) -> Self {
        Self {
            node: port.node,
            port: port.port,
        }
    }
}

/// Run one graph edit under the version check and answer with the new `raw`
/// YAML, `version` and `removed_wires` (the wires an unstage took with the
/// node; empty for every other edit). A refusal keeps its typed code, and the
/// structured detail goes to `data`.
fn graph_edit_call(
    workspace: &CerulionWorkspace,
    graph: &str,
    expect_version: Option<&str>,
    data: &mut Option<Value>,
    edit: impl FnOnce(&CerulionWorkspace) -> Result<EditOutcome, GraphEditError>,
) -> Result<Value, OperationError> {
    let path = workspace.graphs_dir.join(format!("{graph}.yaml"));
    check_expected_version(&path, expect_version)?;
    match edit(workspace) {
        Ok(EditOutcome { raw, removed_wires }) => {
            let version = file_version(&path)?;
            Ok(json!({
                "raw": raw,
                "version": version,
                "removed_wires": removed_wires.iter().map(wire_value).collect::<Vec<_>>(),
            }))
        }
        Err(GraphEditError::SchemaMismatch {
            expected,
            found,
            detail,
        }) => {
            *data = Some(json!({"expected": expected, "found": found}));
            Err(("schema_mismatch", detail))
        }
        Err(GraphEditError::WouldBreak { wires }) => {
            let message = GraphEditError::WouldBreak {
                wires: wires.clone(),
            }
            .to_string();
            *data = Some(json!({"wires": wires.iter().map(wire_value).collect::<Vec<_>>()}));
            Err(("would_break", message))
        }
        Err(GraphEditError::Cli(error)) => Err(engine_failure(error)),
    }
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
    if let ModifyOperation::SetPolicy { policy } = &operation {
        check_positive_durations(Some(policy))?;
    }
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

fn graph_create(
    workspace: &CerulionWorkspace,
    name: &str,
    prefix: Option<&str>,
) -> Result<Value, OperationError> {
    graph_cmd::graph_create(&workspace.graphs_dir, name, prefix).map_err(engine_failure)?;
    let version = file_version(&workspace.graphs_dir.join(format!("{name}.yaml")))?;
    Ok(json!({"version": version}))
}

fn schema_create(workspace: &CerulionWorkspace, name: &str) -> Result<Value, OperationError> {
    cerulion_cli_engine::schema_cmd::schema_create(&workspace.schemas_dir, name)
        .map_err(engine_failure)?;
    let version = file_version(&workspace.schemas_dir.join(format!("{name}.yaml")))?;
    Ok(json!({"version": version}))
}

/// The CLI's `--policy` parser refuses a zero duration (it would spin-loop the
/// scheduler); the wire form must not be a way around that, on `node.create`
/// or `node.modify`.
fn check_positive_durations(policy: Option<&PolicyRequest>) -> Result<(), OperationError> {
    match policy {
        Some(PolicyRequest::Period { period_ms: 0 }) => Err(engine_failure(CliError::Validation(
            "`period_ms` must be > 0 (a zero-duration period would spin-loop the scheduler)"
                .to_string(),
        ))),
        Some(PolicyRequest::Sync { window_ms: 0 }) => Err(engine_failure(CliError::Validation(
            "`window_ms` must be > 0 (a zero-duration window would spin-loop the scheduler)"
                .to_string(),
        ))),
        _ => Ok(()),
    }
}

/// `cerulion node create` with its flags as data. The policy defaulting and
/// the refusals are the engine's (`resolve_create_policy`,
/// `node_create_with_options`), the same functions the CLI calls.
fn node_create(
    workspace: &CerulionWorkspace,
    spec: NodeCreateSpec,
) -> Result<Value, OperationError> {
    check_positive_durations(spec.policy.as_ref())?;
    let pair = |port: PortSpec| (port.schema, port.name);
    let trigger_input = spec.trigger_input.map(pair);
    let regular_inputs: Vec<(String, String)> = spec.inputs.into_iter().map(pair).collect();
    let outputs: Vec<(String, String)> = spec.outputs.into_iter().map(pair).collect();
    let explicit_policy = spec.policy.map(PolicyRequest::into_core);
    let policy = node_cmd::resolve_create_policy(
        explicit_policy.as_ref(),
        trigger_input.as_ref(),
        &regular_inputs,
    )
    .map_err(engine_failure)?;
    let trigger = trigger_input
        .as_ref()
        .map(|(_, name)| name.clone())
        .or_else(|| match policy.as_ref() {
            Some(cerulion_core::MacroPolicy::DataTrigger { input_name }) => {
                Some(input_name.clone())
            }
            _ => None,
        });
    // Trigger input first, as the CLI orders them.
    let mut inputs: Vec<(String, String)> = trigger_input.into_iter().collect();
    inputs.extend(regular_inputs);
    let options = node_cmd::NodeCreateOptions {
        outputs,
        inputs,
        trigger,
        raw_ffi: spec.raw_ffi,
    };
    node_cmd::node_create_with_options(
        &workspace.nodes_dir,
        &workspace.root.join("Cargo.toml"),
        &spec.node_type,
        policy,
        &options,
    )
    .map_err(engine_failure)?;
    let version = file_version(
        &workspace
            .nodes_dir
            .join(&spec.node_type)
            .join("src")
            .join("lib.rs"),
    )?;
    Ok(json!({"version": version}))
}

/// Serve `node.build`: validate, then run the engine's build with cargo's
/// JSON messages turned into `diagnostic` events as they arrive.
fn node_build(
    id: u64,
    root: &str,
    node_type: &str,
    release: bool,
    emit: &mut dyn FnMut(Value),
    cancel: &AtomicBool,
) {
    let refuse = |emit: &mut dyn FnMut(Value), code: &'static str, message: String| {
        emit(response_value(&Response::failure(Some(id), code, message)));
    };
    if !is_safe_component(node_type) {
        return refuse(emit, "bad_request", "invalid node type".to_string());
    }
    if let Err((_, code, message)) = validate_workspace_candidate(root, id) {
        return refuse(emit, code, message);
    }
    // The shared lock only covers discovering the workspace: a build can run
    // for minutes and must not hold off every edit meanwhile. As with
    // `cerulion node build`, the sources are read by cargo when it gets to them.
    let workspace = match acquire_dispatch_lock(root, id, false).and_then(|lock| {
        discover_workspace(lock.root()).map_err(|error| {
            let code = if matches!(
                error,
                WsdError::WorkspaceNotFound | WsdError::RelativeWorkspace
            ) {
                "workspace_not_found"
            } else {
                "engine_error"
            };
            (id, code, error.to_string())
        })
    }) {
        Ok(workspace) => workspace,
        Err((_, code, message)) => return refuse(emit, code, message),
    };

    let emit = std::cell::RefCell::new(emit);
    let started = std::cell::Cell::new(false);
    let errors_sent = std::cell::Cell::new(false);
    let send = |event: Value| (emit.borrow_mut())(event);
    let result = node_cmd::node_build_streaming(
        &workspace.root,
        node_type,
        release,
        // The optional-system-dependency notice explains a failure that
        // cargo's own output never will, so it travels as a note.
        &mut |notice| send(diagnostic_event(id, "note", notice.trim(), None, None)),
        &mut |_| started.set(true),
        &mut |line| {
            if let Some(event) = compiler_message_event(id, line) {
                if event["level"] == "error" {
                    errors_sent.set(true);
                }
                send(event);
            }
        },
        cancel,
    );
    if cancel.load(Ordering::Acquire) {
        // The client is gone; there is nobody to tell.
        return;
    }
    match result {
        Ok(_) => send(done_event(id, true)),
        Err(error) if !started.get() => {
            let (code, message) = engine_failure(error);
            send(response_value(&Response::failure(Some(id), code, message)));
        }
        Err(error) => {
            // Cargo ran and failed. Compiler errors already went out as
            // diagnostics; anything else (a resolver error, a missing
            // toolchain) is only in cargo's stderr, so it goes out as one.
            if !errors_sent.get() {
                let reason = match &error {
                    CliError::BuildFailed { reason, .. } => reason.trim().to_string(),
                    other => other.to_string(),
                };
                send(diagnostic_event(id, "error", &reason, None, Some(&reason)));
            }
            send(done_event(id, false));
        }
    }
}

fn done_event(id: u64, ok: bool) -> Value {
    json!({"id": id, "event": "done", "ok": ok})
}

fn diagnostic_event(
    id: u64,
    level: &str,
    message: &str,
    span: Option<&Value>,
    rendered: Option<&str>,
) -> Value {
    let field = |name: &str| span.map_or(Value::Null, |span| span[name].clone());
    json!({
        "id": id,
        "event": "diagnostic",
        "file": field("file_name"),
        "line": field("line_start"),
        "col": field("column_start"),
        "end_line": field("line_end"),
        "end_col": field("column_end"),
        "level": level,
        "message": message,
        "code": Value::Null,
        "rendered": rendered,
    })
}

/// One line of `cargo build --message-format=json` as a `diagnostic` event, or
/// `None` for every line that is not a compiler message (artifacts, build
/// scripts, the finish marker) and for rustc's two location-less summaries
/// ("aborting due to ...", "N warnings emitted"), which restate what the
/// events already say. Position fields are rustc's own: 1-based, the primary
/// span, relative to the workspace root for workspace members.
fn compiler_message_event(id: u64, line: &str) -> Option<Value> {
    let value: Value = serde_json::from_slice(line.as_bytes()).ok()?;
    if value["reason"] != "compiler-message" {
        return None;
    }
    let message = &value["message"];
    let text = message["message"].as_str()?;
    let level = message["level"].as_str()?;
    let spans = message["spans"]
        .as_array()
        .map(Vec::as_slice)
        .unwrap_or(&[]);
    let span = spans
        .iter()
        .find(|span| span["is_primary"] == true)
        .or_else(|| spans.first());
    if span.is_none() && is_summary_message(text) {
        return None;
    }
    let mut event = diagnostic_event(id, level, text, span, message["rendered"].as_str());
    event["code"] = message["code"]["code"].clone();
    Some(event)
}

fn is_summary_message(text: &str) -> bool {
    text.starts_with("aborting due to")
        || text
            .strip_suffix(" warnings emitted")
            .or_else(|| text.strip_suffix(" warning emitted"))
            .is_some_and(|count| !count.is_empty() && count.bytes().all(|b| b.is_ascii_digit()))
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
                "graph.create" => request["name"] = json!("main"),
                "node.create" => request["spec"] = json!({"node_type": "camera"}),
                "schema.create" => request["spec"] = json!({"name": "scan"}),
                "node.build" => request["node_type"] = json!("camera"),
                "graph.wire" | "graph.unwire" => {
                    request["graph"] = json!("main");
                    request["from"] = json!({"node": "camera", "port": "image"});
                    request["to"] = json!({"node": "detector", "port": "image"});
                }
                "graph.unstage" => {
                    request["graph"] = json!("main");
                    request["node"] = json!("camera");
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

    /// The three edit verbs are additive: `PROTOCOL_VERSION` stays 1, and an
    /// edit request with a field this version does not know is refused, as
    /// every request is.
    #[test]
    fn the_edit_verbs_keep_protocol_one_and_reject_unknown_fields() {
        assert_eq!(PROTOCOL_VERSION, 1);
        for line in [
            r#"{"id":1,"verb":"graph.wire","root":"/w","graph":"main","from":{"node":"a","port":"o","x":1},"to":{"node":"b","port":"i"}}"#,
            r#"{"id":1,"verb":"graph.unwire","root":"/w","graph":"main","from":{"node":"a","port":"o"},"to":{"node":"b","port":"i"},"force":true}"#,
            r#"{"id":1,"verb":"graph.unstage","root":"/w","graph":"main","node":"a","wires":[]}"#,
        ] {
            let response = handle_line(line, &InProcessInspector);
            assert_eq!(response.error.expect("error").code, "bad_request", "{line}");
        }
    }

    #[test]
    fn a_refusal_without_detail_serializes_without_a_data_key() {
        let plain = serde_json::to_value(Response::failure(Some(1), "not_found", "x")).unwrap();
        assert!(plain["error"].get("data").is_none(), "{plain}");
        let detailed = serde_json::to_value(Response::failure_with_data(
            1,
            "would_break",
            "x",
            json!({"wires": []}),
        ))
        .unwrap();
        assert_eq!(detailed["error"]["data"], json!({"wires": []}));
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
        // Creating something that already exists is an engine refusal.
        assert_eq!(
            engine_failure(CliError::GraphExists {
                name: "main".into()
            })
            .0,
            "invalid_request"
        );
        assert_eq!(
            engine_failure(CliError::NodeExists {
                node_type: "cam".into()
            })
            .0,
            "invalid_request"
        );
    }

    #[test]
    fn node_build_needs_the_streaming_entry_point() {
        let line =
            r#"{"id": 3, "verb": "node.build", "root": "/workspace", "node_type": "camera"}"#;
        let response = handle_line(line, &InProcessInspector);
        assert_eq!(response.id, Some(3));
        assert_eq!(response.error.expect("error").code, "bad_request");
        assert!(cancels_on_hangup(line));
        assert!(!cancels_on_hangup(
            r#"{"id": 3, "verb": "node.info", "root": "/workspace", "node_type": "node.build"}"#
        ));
        assert!(!cancels_on_hangup(r#"{"id": 1, "verb": "workspace.info"}"#));
    }

    #[test]
    fn a_streaming_refusal_is_one_ordinary_response_line() {
        let mut lines = Vec::new();
        handle_line_streaming(
            r#"{"id": 4, "verb": "node.build", "root": "/not/a/workspace", "node_type": "camera"}"#,
            &InProcessInspector,
            &mut |value| lines.push(value),
            &AtomicBool::new(false),
        );
        assert_eq!(lines.len(), 1, "{lines:?}");
        assert_eq!(lines[0]["id"], 4);
        assert_eq!(lines[0]["ok"], false);
        assert_eq!(lines[0]["error"]["code"], "workspace_not_found");
    }

    /// The daemon serves every verb through `handle_line_streaming`, so the
    /// structured detail of a graph edit refusal has to survive THAT entry
    /// point, and `handle_line` too. A `respond` that dispatched without the
    /// data slot would still answer the right code and drop `error.data`.
    #[cfg(unix)]
    #[test]
    fn both_entry_points_keep_the_detail_of_a_graph_edit_refusal() {
        let scratch = Scratch::new("refusal-data");
        let root = cerulion_cli_engine::workspace::workspace_create(scratch.path(), "test")
            .expect("workspace")
            .root;
        for (node, port, schema, is_output) in [
            ("camera", "image", "sensor_msgs/Image", true),
            ("detector", "image", "sensor_msgs/Image", false),
            ("logger", "data", "geometry_msgs/Vector3", false),
        ] {
            node_cmd::node_create(&root.join("nodes"), &root.join("Cargo.toml"), node, None)
                .expect("node create");
            node_cmd::node_modify_add_port(
                &root.join("nodes"),
                node,
                port,
                Some(schema),
                is_output,
                false,
            )
            .expect("declare port");
        }
        let graph = root.join("graphs").join("main.yaml");
        std::fs::write(
            &graph,
            "prefix: test\nnodes:\n  - id: cam\n    type: camera\n    outputs:\n      - name: image\n        schema: sensor_msgs/Image\n  - id: det\n    type: detector\n  - id: log\n    type: logger\n",
        )
        .expect("graph yaml");
        let root = serde_json::to_string(root.to_str().expect("utf8")).unwrap();
        let wired = handle_line(
            &format!(
                r#"{{"id":4,"verb":"graph.wire","root":{root},"graph":"main","from":{{"node":"cam","port":"image"}},"to":{{"node":"det","port":"image"}}}}"#
            ),
            &InProcessInspector,
        );
        assert!(wired.error.is_none(), "{:?}", response_value(&wired));
        let raw = std::fs::read_to_string(&graph).unwrap();
        let mismatch = format!(
            r#"{{"id":5,"verb":"graph.wire","root":{root},"graph":"main","from":{{"node":"cam","port":"image"}},"to":{{"node":"log","port":"data"}}}}"#
        );
        let unstage = format!(
            r#"{{"id":6,"verb":"graph.unstage","root":{root},"graph":"main","node":"cam"}}"#
        );

        let streamed = |line: &str| {
            let mut lines = Vec::new();
            handle_line_streaming(
                line,
                &InProcessInspector,
                &mut |value| lines.push(value),
                &AtomicBool::new(false),
            );
            assert_eq!(lines.len(), 1, "{lines:?}");
            lines.remove(0)
        };
        let single = |line: &str| response_value(&handle_line(line, &InProcessInspector));

        for answer in [streamed(&mismatch), single(&mismatch)] {
            assert_eq!(answer["error"]["code"], "schema_mismatch", "{answer}");
            assert_eq!(
                answer["error"]["data"],
                json!({"expected": "geometry_msgs/Vector3", "found": "sensor_msgs/Image"}),
                "{answer}"
            );
        }
        for answer in [streamed(&unstage), single(&unstage)] {
            assert_eq!(answer["error"]["code"], "would_break", "{answer}");
            assert_eq!(
                answer["error"]["data"]["wires"],
                json!([{"from": {"node": "cam", "port": "image"},
                        "to": {"node": "det", "port": "image"}}]),
                "{answer}"
            );
        }
        assert_eq!(std::fs::read_to_string(&graph).unwrap(), raw);
    }

    #[test]
    fn the_stand_in_for_an_over_long_cargo_line_reaches_the_client_as_a_diagnostic() {
        let event = compiler_message_event(3, node_cmd::DROPPED_MESSAGE_LINE).unwrap();
        assert_eq!(event["event"], "diagnostic");
        assert_eq!(event["level"], "warning");
        assert!(event["message"].as_str().unwrap().contains("1 MiB"));
        assert_eq!(event["file"], Value::Null);
    }

    #[test]
    fn compiler_messages_become_diagnostic_events_and_everything_else_is_dropped() {
        let error = json!({"reason": "compiler-message", "package_id": "x", "message": {
        "message": "mismatched types", "level": "error",
        "code": {"code": "E0308", "explanation": null},
        "rendered": "error[E0308]: mismatched types\n",
        "spans": [
            {"file_name": "nodes/a/src/lib.rs", "line_start": 7, "line_end": 7,
             "column_start": 5, "column_end": 9, "is_primary": false},
            {"file_name": "nodes/a/src/lib.rs", "line_start": 9, "line_end": 10,
             "column_start": 18, "column_end": 3, "is_primary": true}
        ]}});
        let event = compiler_message_event(5, &error.to_string()).expect("a diagnostic");
        assert_eq!(
            event,
            json!({
                "id": 5, "event": "diagnostic", "file": "nodes/a/src/lib.rs",
                "line": 9, "col": 18, "end_line": 10, "end_col": 3,
                "level": "error", "message": "mismatched types", "code": "E0308",
                "rendered": "error[E0308]: mismatched types\n",
            }),
            "the PRIMARY span is the location"
        );

        // No span (a linker failure): still reported, with null positions.
        let linker = json!({"reason": "compiler-message", "message": {
            "message": "linking with `cc` failed", "level": "error", "code": null,
            "rendered": "error: linking with `cc` failed\n", "spans": []}});
        let event = compiler_message_event(5, &linker.to_string()).expect("a diagnostic");
        assert_eq!(event["file"], Value::Null);
        assert_eq!(event["line"], Value::Null);
        assert_eq!(event["code"], Value::Null);
        assert_eq!(event["message"], "linking with `cc` failed");

        for summary in [
            "aborting due to 2 previous errors",
            "1 warning emitted",
            "3 warnings emitted",
        ] {
            let line = json!({"reason": "compiler-message", "message": {
                "message": summary, "level": "error", "code": null, "rendered": "", "spans": []}});
            assert_eq!(
                compiler_message_event(5, &line.to_string()),
                None,
                "{summary}"
            );
        }
        // A located message that merely starts like a summary is kept.
        let located = json!({"reason": "compiler-message", "message": {
            "message": "aborting due to a typo", "level": "warning", "code": null,
            "rendered": "", "spans": [{"file_name": "a.rs", "line_start": 1, "line_end": 1,
            "column_start": 1, "column_end": 2, "is_primary": true}]}});
        assert!(compiler_message_event(5, &located.to_string()).is_some());

        for other in [
            r#"{"reason":"compiler-artifact","package_id":"x"}"#,
            r#"{"reason":"build-finished","success":false}"#,
            "not json at all",
        ] {
            assert_eq!(compiler_message_event(5, other), None, "{other}");
        }
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
