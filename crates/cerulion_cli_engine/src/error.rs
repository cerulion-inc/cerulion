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
    if let CliError::Transport(cerulion_core::TransportError::NodeError { reason, .. }) = error {
        let loader_failure = ["cannot open shared object file", "Library not loaded"]
            .iter()
            .any(|marker| reason.contains(marker));
        if loader_failure && reason.contains("libpython") {
            return format!(
                "{rendered}\nPython node cdylib could not find libpython; rebuild with \
                 `cerulion node build <type>` (bakes the interpreter's LIBDIR rpath) or set \
                 {LIBRARY_PATH_VAR}"
            );
        }
    }
    rendered
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn missing_libpython_error_includes_python_node_remedy() {
        let error = CliError::Transport(cerulion_core::TransportError::NodeError {
            node_id: "echo".to_string(),
            reason: "failed to load library: libpython3.12.so.1.0: cannot open shared object file"
                .to_string(),
        });
        assert_eq!(
            render_user_error(&error),
            "Node 'echo' error: failed to load library: \
             libpython3.12.so.1.0: cannot open shared object file\nPython node cdylib could not \
             find libpython; rebuild with `cerulion node build <type>` (bakes the interpreter's \
             LIBDIR rpath) or set LD_LIBRARY_PATH"
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
