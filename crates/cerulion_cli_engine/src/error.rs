// SPDX-License-Identifier: AGPL-3.0-only
//! CLI error types.

/// CLI engine errors.
#[derive(Debug, thiserror::Error)]
pub enum CliError {
    #[error("I/O error: {0}")]
    Io(#[from] std::io::Error),
    #[error("YAML error: {0}")]
    Yaml(#[from] serde_yaml::Error),
    // Rendered verbatim: every `TransportError` message names its own failure
    // ("Graph validation error: ...", "Failed to publish on topic ..."), and a
    // "Transport error:" prefix mislabelled a graph validation or launch
    // refusal as a transport fault in the one line a user reads.
    #[error("{0}")]
    Transport(#[from] cerulion_core::TransportError),
    #[error("Workspace not found (searched from '{start}' to filesystem root). Create one with: cerulion workspace create <name>")]
    WorkspaceNotFound { start: String },
    #[error("Workspace already exists at '{path}'. Use a different name or delete the existing workspace first.")]
    WorkspaceExists { path: String },
    #[error("Node type '{node_type}' already exists in this workspace. Choose a different name or delete the existing node with: cerulion node delete {node_type}")]
    NodeExists { node_type: String },
    #[error("Node type '{node_type}' not found in nodes/. List available nodes with: cerulion node list")]
    NodeNotFound { node_type: String },
    #[error(
        "Graph '{name}' not found in graphs/. List available graphs with: cerulion graph list"
    )]
    GraphNotFound { name: String },
    #[error("Graph '{name}' already exists. Choose a different name or delete the existing graph file in graphs/.")]
    GraphExists { name: String },
    #[error("Schema '{name}' not found. {remedy}")]
    SchemaNotFound {
        name: String,
        /// Context-specific remedy supplied by the call
        /// site: the `schema info` contexts name BOTH lookup
        /// sources, while `schema delete` must not suggest built-in
        /// lookup (a built-in ROS 2 message cannot be deleted).
        remedy: &'static str,
    },
    #[error("Build failed for '{target}': {reason}")]
    BuildFailed { target: String, reason: String },
    #[error("cdylib for node '{node_type}' not found: {reason}")]
    CdylibNotFound { node_type: String, reason: String },
    #[error("{0}")]
    Validation(String),
    /// The workspace lookup could not CHECK a spelling — a
    /// `schemas/…yaml` it could not read or parse, named in the message
    /// (`schemas/Foo.yaml (file stem) could not be parsed (YAML error: …)`).
    /// Distinct from [`CliError::Validation`], which is a REFUSAL (the
    /// spelling names more than one definition), so every consumer can frame
    /// this once as `'<spelling>' could not be checked against this workspace
    /// (…)` while rendering a refusal verbatim — and no consumer inherits an
    /// `I/O error:` / `YAML error:` prefix in a user-facing sentence.
    #[error("{0}")]
    SchemaUnchecked(String),
    /// A login / account-service failure. The message names the fix
    /// (`cerulion login`) where the failure is a missing/expired identity.
    #[error("{0}")]
    Login(String),
    /// One or more nodes returned `NodeError::Fatal` during the run and were
    /// stopped: `(node id, the fatal error's message)` in graph order. The run
    /// itself completed; the process exits with [`EXIT_NODE_FATAL`].
    #[error("{}", render_node_fatal(.nodes))]
    NodeFatal { nodes: Vec<(String, String)> },
    /// A multi-process run in which the listed worker groups each stopped a
    /// node on `NodeError::Fatal` (each worker exited [`EXIT_NODE_FATAL`]). The
    /// worker's own `Error:` line, above in the same terminal, names the node
    /// and the message; the supervisor only sees the exit status.
    #[error(
        "{}", .groups.iter().map(|g| format!(
            "a node in worker group '{g}' stopped on a fatal error (that worker's \
             `Error:` line above names the node and the error)"
        )).collect::<Vec<_>>().join(VERDICT_LINE_SEP)
    )]
    WorkerNodeFatal { groups: Vec<String> },
}

/// Exit status of a `graph run` in which a node was stopped by a fatal error.
/// The same number as `bag play --resim --verify`'s node-failure exit.
pub const EXIT_NODE_FATAL: u8 = 3;

impl CliError {
    /// The process exit status for this error: [`EXIT_NODE_FATAL`] for
    /// [`CliError::NodeFatal`] and [`CliError::WorkerNodeFatal`], 1 for
    /// everything else.
    pub fn exit_code(&self) -> u8 {
        match self {
            Self::NodeFatal { .. } | Self::WorkerNodeFatal { .. } => EXIT_NODE_FATAL,
            _ => 1,
        }
    }
}

/// Between two entries of one fatal verdict. `main` prints the error after
/// one `Error: ` prefix, so this gives every stopped node (or worker group)
/// its own `Error:` line, as `docs/user-api.md` promises.
const VERDICT_LINE_SEP: &str = "\nError: ";

fn render_node_fatal(nodes: &[(String, String)]) -> String {
    let parts: Vec<String> = nodes
        .iter()
        .map(|(node, message)| format!("node '{node}' stopped on a fatal error: {message}"))
        .collect();
    parts.join(VERDICT_LINE_SEP)
}

/// The verdict for a finished run: `Err(NodeFatal)` when any node was
/// stopped by a fatal error, else `Ok`.
pub fn node_fatal_verdict(nodes: Vec<(String, String)>) -> CliResult<()> {
    if nodes.is_empty() {
        Ok(())
    } else {
        Err(CliError::NodeFatal { nodes })
    }
}

/// Result type alias for CLI operations.
pub type CliResult<T> = Result<T, CliError>;

#[cfg(target_os = "macos")]
const LIBRARY_PATH_VAR: &str = "DYLD_LIBRARY_PATH";
#[cfg(not(target_os = "macos"))]
const LIBRARY_PATH_VAR: &str = "LD_LIBRARY_PATH";

/// Render a CLI error with an actionable remedy for missing embedded-Python
/// runtime libraries.
pub fn render_user_error(error: &CliError) -> String {
    let rendered = error.to_string();
    if let CliError::Transport(
        cerulion_core::TransportError::NodeError { reason, .. }
        | cerulion_core::TransportError::NodeFailed { reason, .. },
    ) = error
    {
        // The first two are the Linux and macOS dynamic-loader messages for
        // a cdylib whose libpython dependency is missing; the third is the
        // embedded host's own report when its explicit libpython load fails.
        let loader_failure = [
            "cannot open shared object file",
            "Library not loaded",
            "unable to load",
        ]
        .iter()
        .any(|marker| reason.contains(marker));
        if loader_failure && reason.contains("libpython") {
            return format!(
                "{rendered}\nthe node cdylib could not find libpython; for a Python node \
                 (`node.py`), rebuild with `cerulion node build <type>` (bakes the \
                 interpreter's LIBDIR rpath); otherwise set {LIBRARY_PATH_VAR}"
            );
        }
    }
    rendered
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_run_with_no_fatal_node_is_ok() {
        assert!(node_fatal_verdict(Vec::new()).is_ok());
    }

    #[test]
    fn a_fatal_node_exits_3_and_names_every_node_and_message() {
        let err = node_fatal_verdict(vec![
            ("lidar".to_string(), "Fatal error: lost".to_string()),
            ("imu".to_string(), "Fatal error: bus off".to_string()),
        ])
        .expect_err("a fatal node fails the run");
        assert_eq!(err.exit_code(), 3);
        assert_eq!(
            err.to_string(),
            "node 'lidar' stopped on a fatal error: Fatal error: lost\n\
             Error: node 'imu' stopped on a fatal error: Fatal error: bus off"
        );
    }

    #[test]
    fn a_worker_group_fatal_exits_3_and_names_every_group() {
        let err = CliError::WorkerNodeFatal {
            groups: vec!["g0".to_string(), "g2".to_string()],
        };
        assert_eq!(err.exit_code(), 3);
        assert_eq!(
            err.to_string(),
            "a node in worker group 'g0' stopped on a fatal error (that worker's \
             `Error:` line above names the node and the error)\n\
             Error: a node in worker group 'g2' stopped on a fatal error (that worker's \
             `Error:` line above names the node and the error)"
        );
    }

    #[test]
    fn every_other_error_keeps_exit_1() {
        assert_eq!(CliError::Validation("x".to_string()).exit_code(), 1);
    }

    #[test]
    fn missing_libpython_remedy_also_covers_a_typed_node_error() {
        // A node's own init error now arrives typed (`NodeFailed`); the
        // embedded host's libpython report must still get the remedy.
        let error = CliError::Transport(cerulion_core::TransportError::NodeFailed {
            node_id: "echo".to_string(),
            kind: cerulion_core::error::NodeErrorKind::Logic,
            reason: "init failed: unable to load libpython3.12.so".to_string(),
        });
        let rendered = render_user_error(&error);
        assert!(
            rendered.contains("could not find libpython"),
            "got: {rendered}"
        );
    }

    #[test]
    fn missing_libpython_error_includes_python_node_remedy() {
        let error = CliError::Transport(cerulion_core::TransportError::NodeError {
            node_id: "echo".to_string(),
            reason: "failed to load library: libpython3.12.so.1.0: cannot open shared object file"
                .to_string(),
        });
        assert_eq!(
            render_user_error(&error),
            format!(
                "Node 'echo' error: failed to load library: \
                 libpython3.12.so.1.0: cannot open shared object file\nthe node cdylib could \
                 not find libpython; for a Python node (`node.py`), rebuild with `cerulion node \
                 build <type>` (bakes the interpreter's LIBDIR rpath); otherwise set {}",
                if cfg!(target_os = "macos") {
                    "DYLD_LIBRARY_PATH"
                } else {
                    "LD_LIBRARY_PATH"
                }
            )
        );
    }

    #[test]
    fn embedded_host_libpython_load_failure_includes_python_node_remedy() {
        // The embedded host loads libpython itself and reports a failure as
        // `unable to load <soname> or <fallback>`; that is a loader failure
        // too, so the remedy must follow.
        let error = CliError::Transport(cerulion_core::TransportError::NodeError {
            node_id: "echo".to_string(),
            reason: "init failed: unable to load libpython3.12.so.1.0 or libpython3.12.so"
                .to_string(),
        });
        let rendered = render_user_error(&error);
        assert!(
            rendered.ends_with(&format!(
                "the node cdylib could not find libpython; for a Python node (`node.py`), \
                 rebuild with `cerulion node build <type>` (bakes the interpreter's LIBDIR \
                 rpath); otherwise set {LIBRARY_PATH_VAR}"
            )),
            "{rendered}"
        );
    }

    #[test]
    fn python_exception_mentioning_libpython_gets_no_loader_remedy() {
        let error = CliError::Transport(cerulion_core::TransportError::NodeError {
            node_id: "echo".to_string(),
            reason: "tick raised ValueError: bad path /opt/libpython-tools".to_string(),
        });
        assert_eq!(
            render_user_error(&error),
            "Node 'echo' error: tick raised ValueError: bad path /opt/libpython-tools"
        );
    }

    #[test]
    fn schema_not_found_display_renders_name_and_callsite_remedy() {
        // Oracle pin (HAND-WRITTEN expected string — never derived from the
        // same #[error] attribute): the template renders
        // "Schema '<name>' not found. <remedy>" with the remedy supplied
        // PER CONTEXT by the call site. The
        // per-context remedy TEXTS are pinned where they fire: the info
        // contexts in `schema_builtin_test.rs` / `schema_cli_test.rs`,
        // the delete context in `schema_cmd.rs`'s own tests.
        let err = CliError::SchemaNotFound {
            name: "sensor_msgs::LaserScan".to_string(),
            remedy: "Run `cerulion schema list` to see every available schema.",
        };
        assert_eq!(
            err.to_string(),
            "Schema 'sensor_msgs::LaserScan' not found. Run `cerulion schema list` to see \
             every available schema."
        );
    }
}
