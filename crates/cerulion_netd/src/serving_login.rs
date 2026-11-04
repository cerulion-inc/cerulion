// SPDX-License-Identifier: AGPL-3.0-only
//! Shared local-only prerequisite for serving a network plane.
//!
//! A persisted prior login is sufficient even after token expiry. This reader
//! never refreshes a session, prompts, creates a key, or writes account state.

use std::fs::OpenOptions;
use std::io::Read;
use std::path::{Path, PathBuf};

use serde::Deserialize;

/// The same actionable refusal for every network-serving entry point.
pub const REFUSAL: &str = "serving the network requires a prior login; run `cerulion login`";
const MAX_AUTH_BYTES: u64 = 64 * 1024;

/// A reason that contains no authentication tokens or untrusted JSON text.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LoginProblem {
    /// No login configuration home can be resolved.
    NoConfigHome,
    /// No prior login file exists.
    Missing,
    /// The file could not be opened or read.
    Unreadable,
    /// The opened descriptor is not a regular file.
    NotRegular,
    /// The file exceeds the bounded local-state size.
    TooLarge,
    /// Required durable login fields are absent or malformed.
    Malformed,
    /// A valid record explicitly says no login has completed.
    NeverLoggedIn,
}

/// Expiry is deliberately absent from the serving decision.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ServingLogin {
    /// The durable prior-login marker allows offline serving.
    Allowed,
    /// Serving must stop before opening a network plane.
    Refused(LoginProblem),
}

/// Local startup policy selected by the daemon embedding the egress dispatcher.
/// Injected library daemons stay independent of ambient account state.
#[derive(Clone, Debug, Default)]
pub enum EgressLoginPolicy {
    /// No login prerequisite, including network-off production operation.
    #[default]
    Unrestricted,
    /// Re-read this local login before each egress registration. No resolved home
    /// is a refusal, rather than an implicit exemption.
    PriorLogin {
        /// The configured login file, resolved once by the embedding process.
        auth_path: Option<PathBuf>,
    },
}

impl EgressLoginPolicy {
    /// Resolve the production policy from whether a real network is configured.
    pub fn for_network(network_enabled: bool) -> Self {
        if network_enabled {
            Self::PriorLogin {
                auth_path: cerulion_discovery::robot_state::config_dir()
                    .map(|dir| dir.join("auth.json")),
            }
        } else {
            Self::Unrestricted
        }
    }

    pub(crate) fn check(&self) -> Result<(), &'static str> {
        match self {
            Self::Unrestricted => Ok(()),
            Self::PriorLogin { auth_path } => {
                let decision = auth_path
                    .as_deref()
                    .map(read_at)
                    .unwrap_or(ServingLogin::Refused(LoginProblem::NoConfigHome));
                match decision {
                    ServingLogin::Allowed => Ok(()),
                    ServingLogin::Refused(_) => Err(REFUSAL),
                }
            }
        }
    }
}

// Mirror only the mandatory AuthState fields. Unknown fields (including future
// role values) remain compatible. No Debug implementation may expose tokens.
#[derive(Deserialize)]
struct LoginRecord {
    account_id: String,
    #[serde(rename = "session_token")]
    _session_token: String,
    #[serde(rename = "refresh_token")]
    _refresh_token: String,
    #[serde(rename = "expires_at_ns")]
    _expires_at_ns: u64,
    logged_in_ever: bool,
    #[serde(default, rename = "role")]
    _role: Option<serde::de::IgnoredAny>,
}

fn identity(bytes: &[u8]) -> Result<String, LoginProblem> {
    let record: LoginRecord = serde_json::from_slice(bytes).map_err(|_| LoginProblem::Malformed)?;
    if record.account_id.trim().is_empty() {
        return Err(LoginProblem::Malformed);
    }
    if !record.logged_in_ever {
        return Err(LoginProblem::NeverLoggedIn);
    }
    Ok(record.account_id)
}

fn classify(bytes: &[u8]) -> ServingLogin {
    match identity(bytes) {
        Ok(_) => ServingLogin::Allowed,
        Err(reason) => ServingLogin::Refused(reason),
    }
}

/// Read a regular file with a size bound and without blocking on a FIFO.
/// Symlinks to regular files retain the existing auth reader's behavior.
/// Callers must independently verify multi-file snapshot consistency.
#[doc(hidden)]
pub fn read_regular_bounded(path: &Path, max_bytes: u64) -> Result<Vec<u8>, LoginProblem> {
    let limit = max_bytes.checked_add(1).ok_or(LoginProblem::TooLarge)?;
    let mut options = OpenOptions::new();
    options.read(true);
    #[cfg(unix)]
    {
        use std::os::unix::fs::OpenOptionsExt;
        options.custom_flags(libc::O_NONBLOCK);
    }
    let file = options.open(path).map_err(|error| {
        if error.kind() == std::io::ErrorKind::NotFound {
            LoginProblem::Missing
        } else {
            LoginProblem::Unreadable
        }
    })?;
    if !file
        .metadata()
        .map_err(|_| LoginProblem::Unreadable)?
        .is_file()
    {
        return Err(LoginProblem::NotRegular);
    }
    let mut bytes = Vec::new();
    file.take(limit)
        .read_to_end(&mut bytes)
        .map_err(|_| LoginProblem::Unreadable)?;
    if bytes.len() as u64 > max_bytes {
        return Err(LoginProblem::TooLarge);
    }
    Ok(bytes)
}

/// Read a durable prior-login identity without HTTP, key creation or token output.
/// This is one file snapshot, not an atomic transaction with any key or certificate.
pub fn read_identity_at(path: &Path) -> Result<String, LoginProblem> {
    identity(&read_regular_bounded(path, MAX_AUTH_BYTES)?)
}

/// Read an explicit login file without following a special-file blocking path.
pub fn read_at(path: &Path) -> ServingLogin {
    match read_regular_bounded(path, MAX_AUTH_BYTES) {
        Ok(bytes) => classify(&bytes),
        Err(reason) => ServingLogin::Refused(reason),
    }
}

/// Enforce the one local prerequisite without consulting staged desk-gate flags.
pub fn require_prior_login() -> Result<(), &'static str> {
    let decision = cerulion_discovery::robot_state::config_dir()
        .map(|dir| read_at(&dir.join("auth.json")))
        .unwrap_or(ServingLogin::Refused(LoginProblem::NoConfigHome));
    require_allowed(decision)
}

fn require_allowed(decision: ServingLogin) -> Result<(), &'static str> {
    match decision {
        ServingLogin::Allowed => Ok(()),
        ServingLogin::Refused(reason) => {
            tracing::warn!(reason = ?reason, "network serving refused before startup");
            Err(REFUSAL)
        }
    }
}

#[cfg(test)]
mod tests;
