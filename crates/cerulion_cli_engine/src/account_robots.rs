// SPDX-License-Identifier: AGPL-3.0-only
//! Bounded, account-bound listing of the current login's owned robots.
//!
//! Listing never establishes presence, opens a robot connection or writes a cache.
//! A caller retaining a result must revalidate its login before using or caching it.

use std::collections::BTreeSet;
use std::fs::OpenOptions;
use std::io::Read;
use std::path::{Path, PathBuf};
use std::time::Duration;

use base64::engine::general_purpose::URL_SAFE_NO_PAD;
use base64::Engine;
use cerulion_pairing::format::{AccountId, PublicKey, RobotId};
use serde::Deserialize;

use crate::auth::{self, AuthState};
use crate::error::{CliError, CliResult};
use crate::{account_cmd, account_identity, login_cmd};

const HTTP_TIMEOUT: Duration = Duration::from_secs(15);
const MAX_BODY_BYTES: usize = 1024 * 1024;
const MAX_ROBOTS: usize = 4096;
const MAX_HOSTNAME_BYTES: usize = 253;

/// Endpoint identity, independent of whether a robot is reachable.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum RobotEndpoint {
    /// Registration proved possession of this key. Presence is still untested.
    Known(PublicKey),
    /// The service omitted the endpoint key; presence cannot be checked.
    Unknown {
        /// A local explanation, never server-supplied prose.
        reason: &'static str,
    },
}

/// One owned robot; its opaque identifier distinguishes duplicate display names.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct AccountRobot {
    /// Account-service robot identifier.
    pub robot_id: RobotId,
    /// Bounded runtime display label.
    pub hostname: String,
    /// Authenticated endpoint key or an explicit reason it is unavailable.
    pub endpoint: RobotEndpoint,
}

/// An owner-only catalog tied to the login and device key used to fetch it.
///
/// No presence result is implied by an entry, including one with a known key.
/// Listing works without a cached certificate chain; later WAN admission verifies
/// that chain separately. A failed fetch must not replace LAN discovery results.
#[derive(Clone)]
pub struct AccountRobotDirectory {
    /// Mapped pairing account of the current login.
    pub account_id: AccountId,
    /// Desk transport key from the same login snapshot.
    pub desk_key: PublicKey,
    /// Owned robots, sorted by hostname then robot identifier.
    pub robots: Vec<AccountRobot>,
    login: LoginSnapshot,
}

impl std::fmt::Debug for AccountRobotDirectory {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("AccountRobotDirectory")
            .field("account_id", &self.account_id)
            .field("desk_key", &self.desk_key)
            .field("robots", &self.robots)
            .finish_non_exhaustive()
    }
}

impl AccountRobotDirectory {
    /// Build the public daemon message under one auth-store transaction lock.
    ///
    /// The HTTP directory remains bound to its original login/session/account/key.
    /// The certificate is read now: same-identity proof refresh is allowed, and the
    /// daemon independently compares the supplied proof with its current local one.
    /// No token or private key is serialized; no session is refreshed here.
    #[cfg(unix)]
    pub fn daemon_snapshot(&self) -> CliResult<cerulion_netd::account_access::AccountSnapshot> {
        auth::with_store_lock(&self.login.auth_path, || Ok(self.daemon_snapshot_locked()))
            .map_err(|_| invalid("the account store could not be locked"))?
    }

    #[cfg(unix)]
    fn daemon_snapshot_locked(&self) -> CliResult<cerulion_netd::account_access::AccountSnapshot> {
        let current = snapshot_locked(&self.login.auth_path)?;
        ensure_unchanged(&self.login, &current)?;
        if self.account_id != current.account || self.desk_key != current.key {
            return Err(changed_login());
        }
        if !current.state.session_is_valid(auth::now_unix_ns()) {
            return Err(invalid(
                "the session expired after fetching the robot catalog",
            ));
        }
        let dir = current
            .auth_path
            .parent()
            .ok_or_else(|| invalid("the account store has no parent directory"))?;
        let owner_chain = crate::owner_certificate::load_optional_locked(
            dir,
            &current.auth_path,
            current.account,
            current.key,
        )?
        .map(|proof| {
            proof
                .to_postcard()
                .map_err(|_| invalid("the owner certificate could not be encoded"))
        })
        .transpose()?;
        let snapshot = cerulion_netd::account_access::AccountSnapshot {
            auth_account_id: current.state.account_id,
            pairing_account_id: current.account.0,
            device_key: current.key.0,
            owner_chain,
            robots: self
                .robots
                .iter()
                .map(|robot| cerulion_netd::account_access::AccountRobot {
                    robot_id: robot.robot_id.0,
                    hostname: robot.hostname.clone(),
                    endpoint_key: match &robot.endpoint {
                        RobotEndpoint::Known(key) => Some(key.0),
                        RobotEndpoint::Unknown { .. } => None,
                    },
                })
                .collect(),
        };
        snapshot
            .validate()
            .map_err(|_| invalid("the account directory cannot form a valid daemon snapshot"))?;
        Ok(snapshot)
    }

    /// Refuse a stale result after logout, account/key change or session refresh.
    /// Call immediately before caching or installing this result into another service.
    pub fn validate_current_login(&self) -> CliResult<()> {
        ensure_unchanged(&self.login, &snapshot(&self.login.auth_path)?)?;
        if !self.login.state.session_is_valid(auth::now_unix_ns()) {
            return Err(invalid(
                "the session expired after fetching the robot catalog",
            ));
        }
        Ok(())
    }
}

#[derive(Deserialize)]
struct RobotsResponse {
    robots: Vec<RobotResponse>,
}

#[derive(Deserialize)]
struct RobotResponse {
    robot_id: String,
    hostname: String,
    owner_account_id: String,
    // Omission is backward compatible; present null or a wrong type is malformed.
    #[serde(default, deserialize_with = "present_endpoint")]
    robot_transport_key: Option<String>,
}

fn present_endpoint<'de, D: serde::Deserializer<'de>>(d: D) -> Result<Option<String>, D::Error> {
    String::deserialize(d).map(Some)
}

// No Debug: AuthState contains bearer secrets.
#[derive(Clone, PartialEq, Eq)]
struct LoginSnapshot {
    auth_path: PathBuf,
    state: AuthState,
    account: AccountId,
    key: PublicKey,
}

/// Fetch the current account's robots without prompting, probing or writing a cache.
///
/// Uses the existing session refresh flow, then bounds the listing request to 15
/// seconds and one MiB. A changed login or key discards the response. This must not
/// run from shell completion. An unsupported endpoint is an error, not an empty list.
pub fn fetch() -> CliResult<AccountRobotDirectory> {
    let auth_path =
        auth::auth_json_path().ok_or_else(|| invalid("no account directory is configured"))?;
    let initial = snapshot(&auth_path)?;
    let endpoint = service_url(&login_cmd::account_service_base())?;
    let session = account_cmd::require_session()
        .map_err(|_| invalid("a current session is required; run `cerulion login`"))?;
    let before = snapshot(&auth_path)?;
    if initial.account != before.account
        || initial.key != before.key
        || initial.state.account_id != before.state.account_id
        || session != before.state.session_token
    {
        return Err(changed_login());
    }
    let response = login_cmd::http_client_without_redirects()
        .map_err(|_| invalid("HTTP client could not be initialized"))?
        .get(endpoint.clone())
        .bearer_auth(&session)
        .timeout(HTTP_TIMEOUT)
        .send()
        .map_err(|_| {
            invalid("the account service could not be reached within the request deadline")
        })?;
    if response.url() != &endpoint {
        return Err(invalid(
            "the account service redirected the robot catalog request",
        ));
    }
    check_status(response.status())?;
    if response
        .content_length()
        .is_some_and(|len| len > MAX_BODY_BYTES as u64)
    {
        return Err(invalid("the robot catalog exceeds the response size limit"));
    }
    finish(before, &read_body(response)?)
}

fn finish(before: LoginSnapshot, bytes: &[u8]) -> CliResult<AccountRobotDirectory> {
    let robots = parse_robots(bytes, before.account)?;
    let after = snapshot(&before.auth_path)?;
    ensure_unchanged(&before, &after)?;
    if !after.state.session_is_valid(auth::now_unix_ns()) {
        return Err(invalid(
            "the session expired while fetching the robot catalog",
        ));
    }
    Ok(AccountRobotDirectory {
        account_id: before.account,
        desk_key: before.key,
        robots,
        login: before,
    })
}

fn check_status(status: reqwest::StatusCode) -> CliResult<()> {
    if status.is_success() {
        return Ok(());
    }
    Err(match status {
        reqwest::StatusCode::NOT_FOUND => invalid("the account service does not provide a robot catalog; account robot presence is unknown"),
        reqwest::StatusCode::UNAUTHORIZED => invalid("the account service refused the session; run `cerulion login`"),
        _ => invalid(&format!("the account service refused the catalog request (HTTP {})", status.as_u16())),
    })
}

fn service_url(base: &str) -> CliResult<reqwest::Url> {
    let url = reqwest::Url::parse(&format!("{}/v1/robots", base.trim_end_matches('/')))
        .map_err(|_| invalid("the account service URL is invalid"))?;
    let loopback = url
        .host_str()
        .and_then(|host| {
            host.trim_matches(['[', ']'])
                .parse::<std::net::IpAddr>()
                .ok()
        })
        .is_some_and(|ip| ip.is_loopback());
    if (url.scheme() != "https" && !(url.scheme() == "http" && loopback))
        || url.host_str().is_none()
        || !url.username().is_empty()
        || url.password().is_some()
        || url.query().is_some()
        || url.fragment().is_some()
    {
        return Err(invalid("the account service requires HTTPS or loopback HTTP without URL credentials, query or fragment"));
    }
    Ok(url)
}

fn snapshot(auth_path: &Path) -> CliResult<LoginSnapshot> {
    auth::with_store_lock(auth_path, || Ok(snapshot_locked(auth_path)))
        .map_err(|_| invalid("the account store could not be locked"))?
}

// Caller holds this auth store's transaction lock across every identity read.
fn snapshot_locked(auth_path: &Path) -> CliResult<LoginSnapshot> {
    if auth::auth_json_path().as_deref() != Some(auth_path) {
        return Err(changed_login());
    }
    let loaded = auth::load_from(auth_path);
    let state = loaded
        .state()
        .filter(|state| state.logged_in_ever && !state.session_token.is_empty())
        .cloned()
        .ok_or_else(|| invalid("a login session is required; run `cerulion login`"))?;
    let account = account_identity::pairing_account_id(&state.account_id)
        .map_err(|_| invalid("the login has no valid pairing account identity"))?;
    let key_path =
        auth::device_key_path().ok_or_else(|| invalid("the login key path is unavailable"))?;
    let mut options = OpenOptions::new();
    options.read(true);
    #[cfg(unix)]
    {
        use std::os::unix::fs::OpenOptionsExt;
        options.custom_flags(libc::O_NONBLOCK);
    }
    let file = options
        .open(key_path)
        .map_err(|_| invalid("the login device key could not be read"))?;
    let metadata = file
        .metadata()
        .map_err(|_| invalid("the login device key could not be inspected"))?;
    if !metadata.is_file() || metadata.len() != 32 {
        return Err(invalid(
            "the login device key must be a regular file containing exactly 32 bytes",
        ));
    }
    let mut bytes = Vec::new();
    file.take(33)
        .read_to_end(&mut bytes)
        .map_err(|_| invalid("the login device key could not be read"))?;
    let seed: [u8; 32] = bytes
        .try_into()
        .map_err(|_| invalid("the login device key changed while reading it"))?;
    let key = PublicKey(
        ed25519_dalek::SigningKey::from_bytes(&seed)
            .verifying_key()
            .to_bytes(),
    );
    Ok(LoginSnapshot {
        auth_path: auth_path.to_path_buf(),
        state,
        account,
        key,
    })
}

fn ensure_unchanged(before: &LoginSnapshot, after: &LoginSnapshot) -> CliResult<()> {
    if before != after {
        return Err(changed_login());
    }
    Ok(())
}

fn changed_login() -> CliError {
    invalid("the login changed while fetching the robot catalog; retry with the current account")
}

fn read_body(reader: impl Read) -> CliResult<Vec<u8>> {
    let mut bytes = Vec::new();
    reader
        .take(MAX_BODY_BYTES as u64 + 1)
        .read_to_end(&mut bytes)
        .map_err(|_| {
            invalid("the robot catalog response could not be read within the request deadline")
        })?;
    if bytes.len() > MAX_BODY_BYTES {
        return Err(invalid("the robot catalog exceeds the response size limit"));
    }
    Ok(bytes)
}

fn parse_id(encoded: &str) -> CliResult<[u8; 32]> {
    URL_SAFE_NO_PAD
        .decode(encoded)
        .ok()
        .and_then(|bytes| bytes.try_into().ok())
        .ok_or_else(|| invalid("the robot catalog contains an invalid account or robot identifier"))
}

fn parse_robots(bytes: &[u8], owner: AccountId) -> CliResult<Vec<AccountRobot>> {
    if bytes.len() > MAX_BODY_BYTES {
        return Err(invalid("the robot catalog exceeds the response size limit"));
    }
    let response: RobotsResponse = serde_json::from_slice(bytes)
        .map_err(|_| invalid("the account service returned a malformed robot catalog"))?;
    if response.robots.len() > MAX_ROBOTS {
        return Err(invalid("the robot catalog exceeds the robot count limit"));
    }
    let mut ids = BTreeSet::new();
    let mut robots = Vec::with_capacity(response.robots.len());
    for robot in response.robots {
        if AccountId(parse_id(&robot.owner_account_id)?) != owner {
            return Err(invalid(
                "the robot catalog contains a robot owned by another account",
            ));
        }
        let robot_id = RobotId(parse_id(&robot.robot_id)?);
        if !ids.insert(robot_id.0) {
            return Err(invalid("the robot catalog repeats a robot identifier"));
        }
        if robot.hostname.is_empty()
            || robot.hostname.len() > MAX_HOSTNAME_BYTES
            || robot.hostname.starts_with('-')
            || !robot
                .hostname
                .bytes()
                .all(|b| b.is_ascii_alphanumeric() || matches!(b, b'_' | b'.' | b'-'))
        {
            return Err(invalid("the robot catalog contains an invalid hostname"));
        }
        let endpoint = match robot.robot_transport_key {
            None => RobotEndpoint::Unknown { reason: "the account service did not provide this robot's endpoint key; presence is unknown" },
            Some(encoded) => {
                if encoded.len() != 64 || !encoded.bytes().all(|b| b.is_ascii_hexdigit()) {
                    return Err(invalid("the robot catalog contains an invalid endpoint key"));
                }
                let bytes: [u8; 32] = hex::decode(encoded).ok().and_then(|bytes| bytes.try_into().ok())
                    .ok_or_else(|| invalid("the robot catalog contains an invalid endpoint key"))?;
                let key = ed25519_dalek::VerifyingKey::from_bytes(&bytes)
                    .map_err(|_| invalid("the robot catalog contains an invalid endpoint key"))?;
                if key.is_weak() { return Err(invalid("the robot catalog contains a weak endpoint key")); }
                RobotEndpoint::Known(PublicKey(bytes))
            }
        };
        robots.push(AccountRobot {
            robot_id,
            hostname: robot.hostname,
            endpoint,
        });
    }
    robots.sort_by(|a, b| {
        a.hostname
            .cmp(&b.hostname)
            .then(a.robot_id.0.cmp(&b.robot_id.0))
    });
    Ok(robots)
}

fn invalid(reason: &str) -> CliError {
    CliError::Login(format!("account robot catalog unavailable: {reason}"))
}

#[cfg(test)]
mod tests;
