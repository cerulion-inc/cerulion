// SPDX-License-Identifier: AGPL-3.0-only
use std::path::PathBuf;

/// The crate's one error enum. Only consent file operations can fail;
/// recording an event never does (a bad event is dropped and counted).
#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error("no home directory to place telemetry.json (set CERULION_HOME)")]
    NoHome,
    #[error("telemetry file I/O at {path}: {source}")]
    Io {
        path: PathBuf,
        #[source]
        source: std::io::Error,
    },
    #[error("telemetry.json does not parse: {0}")]
    Json(String),
}
