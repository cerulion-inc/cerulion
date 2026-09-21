//! `cerulion-wsd`: the local workspace-engine daemon that Cerulion Studio talks to.
//!
//! The daemon serves workspace, graph and node inspection plus versioned node and
//! graph edits over a private Unix-socket protocol (one JSON object per line).
//! Every request runs through `cerulion_cli_engine`, the same code the `cerulion`
//! CLI runs, so a GUI never re-implements the engine's rules, and edits serialize
//! with the CLI on the workspace lock.
//!
//! Studio bundles and starts the daemon itself, so a user never runs it by hand;
//! `cerulion-wsd --help` documents the socket path and the environment variables for
//! anyone writing another client. Every connection is greeted with a hello line
//! that names the protocol version, and a client that reads a version it does not
//! know must disconnect. The protocol and its error codes are described under
//! "Desk daemons" in `docs/user-api.md`.
//!
//! - `daemon`: the accept loop and request dispatch (`daemon::WsdConfig`).
//! - `protocol`: the request and response types.
//! - `inspect`: loads a node library in a child process to read its metadata.
//! - `hygiene`: the socket path and the single-daemon lock, shared through
//!   `cerulion_hygiene`.
//!
//! Unix only.

#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]

#[cfg(unix)]
pub mod daemon;
#[cfg(unix)]
pub mod hygiene;
#[cfg(unix)]
pub mod inspect;
#[cfg(unix)]
pub mod protocol;

use std::path::Path;

use cerulion_cli_engine::workspace::CerulionWorkspace;
use thiserror::Error;

/// Errors returned by the workspace daemon.
#[derive(Debug, Error)]
pub enum WsdError {
    #[error("I/O error: {0}")]
    Io(#[from] std::io::Error),
    #[error("JSON error: {0}")]
    Json(#[from] serde_json::Error),
    #[error("engine error: {0}")]
    Engine(#[from] cerulion_cli_engine::error::CliError),
    #[error("workspace root must be absolute")]
    RelativeWorkspace,
    #[error("workspace root is not a Cerulion workspace")]
    WorkspaceNotFound,
}

/// Discover and validate an absolute workspace root.
pub(crate) fn discover_workspace(path: &Path) -> Result<CerulionWorkspace, WsdError> {
    if !path.is_absolute() {
        return Err(WsdError::RelativeWorkspace);
    }
    let workspace = CerulionWorkspace::discover(path).map_err(|_| WsdError::WorkspaceNotFound)?;
    if workspace.root != path {
        return Err(WsdError::WorkspaceNotFound);
    }
    Ok(workspace)
}
