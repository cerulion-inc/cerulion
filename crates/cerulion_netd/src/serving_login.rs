// SPDX-License-Identifier: AGPL-3.0-only
//! Shared local-only prerequisite for serving a network plane.
//!
//! A persisted prior login is sufficient even after token expiry. WAN egress
//! also accepts authenticated persistent robot pairing. These readers never
//! refresh a session, prompt, create a key, or write account state.

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
    /// Durable pairing can serve without account enrollment. The remote daemon
    /// verifies the persisted device index before binding its endpoint.
    PriorLoginOrPairedRobot {
        /// Saved login location, resolved once.
        auth_path: Option<PathBuf>,
        /// Existing remote-plane state, never created by this reader.
        state_root: Option<PathBuf>,
    },
}

impl EgressLoginPolicy {
    /// Resolve the production policy from whether a real network is configured.
    pub fn for_network(network_enabled: bool) -> Self {
        if network_enabled {
            Self::PriorLoginOrPairedRobot {
                auth_path: cerulion_discovery::robot_state::config_dir()
                    .map(|dir| dir.join("auth.json")),
                state_root: cerulion_discovery::robot_state::resolve(),
            }
        } else {
            Self::Unrestricted
        }
    }

    pub(crate) fn check(&self) -> Result<(), &'static str> {
        match self {
            Self::Unrestricted => Ok(()),
            Self::PriorLoginOrPairedRobot {
                auth_path,
                state_root,
            } => {
                if state_root
                    .as_deref()
                    .is_some_and(|root| saved_robot(root).is_ok())
                {
                    return Ok(());
                }
                Self::PriorLogin {
                    auth_path: auth_path.clone(),
                }
                .check()
            }
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

/// Public identity read from authenticated persistent robot state. Login expiry
/// does not erase an established pairing. No secret is retained in the result.
pub(crate) struct SavedRobot {
    pub robot_id: String,
    pub endpoint_id: String,
    pub owner_account: String,
}

pub(crate) fn saved_robot(root: &Path) -> Result<SavedRobot, LoginProblem> {
    use cerulion_pairing::client::DeviceIdentity;
    use cerulion_pairing::verify::TrustStore;
    if !root.is_absolute()
        || root.components().any(|part| {
            matches!(
                part,
                std::path::Component::ParentDir | std::path::Component::CurDir
            )
        })
    {
        return Err(LoginProblem::Malformed);
    }
    let dir = root.join("remoted");
    let seed: [u8; 32] = read_regular_bounded(&dir.join("device_key"), 32)?
        .try_into()
        .map_err(|_| LoginProblem::Malformed)?;
    let mac = read_regular_bounded(&dir.join("trust_store.mac_key"), MAX_AUTH_BYTES)?;
    if mac.is_empty() {
        return Err(LoginProblem::Malformed);
    }
    let store_path = dir.join("trust_store");
    read_regular_bounded(&store_path, 16 * 1024 * 1024)?;
    let store = TrustStore::load(&store_path, &mac).map_err(|_| LoginProblem::Malformed)?;
    let key = DeviceIdentity::from_seed(&seed).public_key();
    if key != store.robot_transport_key() {
        return Err(LoginProblem::Malformed);
    }
    let owner = store.owner().ok_or(LoginProblem::Malformed)?;
    let index = read_regular_bounded(&dir.join("device_index.json"), 16 * 1024 * 1024)?;
    verify_index(&index, &mac)?;
    let encode =
        |bytes: &[u8; 32]| -> String { bytes.iter().map(|byte| format!("{byte:02x}")).collect() };
    Ok(SavedRobot {
        robot_id: encode(&store.robot_id().0),
        endpoint_id: encode(&key.0),
        owner_account: encode(&owner.0),
    })
}

fn verify_index(bytes: &[u8], mac: &[u8]) -> Result<(), LoginProblem> {
    #[derive(Deserialize)]
    struct Index {
        version: u16,
        bindings: Vec<Binding>,
    }
    #[derive(Deserialize)]
    struct Binding {
        device_key: String,
        account: String,
    }
    if bytes.len() < 14 + 32 {
        return Err(LoginProblem::Malformed);
    }
    let (frame, tag) = bytes.split_at(bytes.len() - 32);
    if !cerulion_pairing::verify::verify_persisted_mac(mac, frame, tag)
        || &frame[..8] != b"CERIDX01"
        || frame[8..10] != 1u16.to_le_bytes()
        || u32::from_le_bytes(
            frame[10..14]
                .try_into()
                .map_err(|_| LoginProblem::Malformed)?,
        ) as usize
            != frame.len() - 14
    {
        return Err(LoginProblem::Malformed);
    }
    let index: Index = serde_json::from_slice(&frame[14..]).map_err(|_| LoginProblem::Malformed)?;
    let identity =
        |value: &str| value.len() == 64 && value.bytes().all(|byte| byte.is_ascii_hexdigit());
    if index.version != 1
        || index
            .bindings
            .iter()
            .any(|binding| !identity(&binding.device_key) || !identity(&binding.account))
    {
        return Err(LoginProblem::Malformed);
    }
    Ok(())
}

/// Local serving authority shared by frontend and graph startup gates. A valid
/// persisted claimed robot key, trust and authenticated device index suffice
/// without auth.json; otherwise the saved prior-login rule applies. Peer pins
/// in robots.toml never authorize this machine to serve. Performs no enrollment.
pub fn require_serving_authority() -> Result<(), &'static str> {
    EgressLoginPolicy::for_network(true).check()
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
