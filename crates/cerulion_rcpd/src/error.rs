// SPDX-License-Identifier: AGPL-3.0-only
//! The crate's one error enum.

use std::net::SocketAddr;
use std::path::PathBuf;

use cerulion_core::TransportError;

/// Everything that can stop `cerulion-rcpd` from serving.
#[derive(Debug, thiserror::Error)]
pub enum RcpdError {
    /// The manifest file could not be read.
    #[error("cannot read manifest {}: {source}", path.display())]
    ManifestRead {
        /// The manifest path as given.
        path: PathBuf,
        /// The underlying I/O error.
        source: std::io::Error,
    },
    /// The manifest is not valid `rcp.yaml`.
    #[error("invalid manifest {}: {reason}", path.display())]
    ManifestParse {
        /// The manifest path as given.
        path: PathBuf,
        /// The parser's explanation.
        reason: String,
    },
    /// The graph YAML named by the manifest could not be read.
    #[error("cannot read graph {}: {source}", path.display())]
    GraphRead {
        /// The resolved graph path.
        path: PathBuf,
        /// The underlying I/O error.
        source: std::io::Error,
    },
    /// The graph YAML named by the manifest does not parse.
    #[error("invalid graph {}: {reason}", path.display())]
    GraphParse {
        /// The resolved graph path.
        path: PathBuf,
        /// The parser's explanation.
        reason: String,
    },
    /// `--http` was asked for without a bearer token.
    #[error("the HTTP transport needs a bearer token: set {}", crate::TOKEN_ENV)]
    MissingToken,
    /// The transport manager could not be opened.
    #[error("transport: {0}")]
    Transport(#[from] TransportError),
    /// The HTTP listener could not bind.
    #[error("cannot bind {addr}: {source}")]
    Bind {
        /// The requested address.
        addr: SocketAddr,
        /// The underlying I/O error.
        source: std::io::Error,
    },
    /// The MCP session could not start or ended abnormally.
    #[error("MCP session failed: {reason}")]
    Session {
        /// What failed.
        reason: String,
    },
    /// The receipt log could not be opened.
    #[error("cannot open receipt log {}: {reason}", path.display())]
    Receipts {
        /// The receipt log path.
        path: PathBuf,
        /// What failed.
        reason: String,
    },
    /// The graph observer thread could not be spawned.
    #[error("cannot start the graph observer thread: {0}")]
    ObserverSpawn(std::io::Error),
}
