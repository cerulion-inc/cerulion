// SPDX-License-Identifier: AGPL-3.0-only
//! Bounded regular-file reads and durable prior-login decoding.

use serde::Deserialize;
use std::fs::File;
use std::io::{self, Read};
use std::path::Path;

const MAX_AUTH_BYTES: u64 = 64 * 1024;

/// A reason containing no authentication tokens or untrusted file text.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LoginProblem {
    /// No login configuration home can be resolved by the caller.
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

// Unknown fields remain compatible, including future role values. No Debug.
#[derive(Deserialize)]
struct LoginRecord {
    account_id: String,
    session_token: String,
    #[serde(rename = "refresh_token")]
    _refresh_token: String,
    #[serde(rename = "expires_at_ns")]
    _expires_at_ns: u64,
    logged_in_ever: bool,
    #[serde(default, rename = "role")]
    _role: Option<serde::de::IgnoredAny>,
}

/// Decode the mandatory durable-login fields without considering token expiry.
pub fn decode_login_identity(bytes: &[u8]) -> Result<String, LoginProblem> {
    decode_login_state(bytes).map(|(account, _)| account)
}

fn decode_login_state(bytes: &[u8]) -> Result<(String, bool), LoginProblem> {
    let record: LoginRecord = serde_json::from_slice(bytes).map_err(|_| LoginProblem::Malformed)?;
    if record.account_id.trim().is_empty() {
        return Err(LoginProblem::Malformed);
    }
    if !record.logged_in_ever {
        return Err(LoginProblem::NeverLoggedIn);
    }
    Ok((record.account_id, !record.session_token.is_empty()))
}

/// Read a regular file through the caller's read-only, nonblocking opener.
///
/// Validates the opened descriptor before reading and checks the byte bound.
/// Symlinks to regular files remain supported. Multi-file consistency requires
/// the caller to hold its existing transaction lock.
pub fn read_regular_bounded(
    path: &Path,
    max_bytes: u64,
    open: &impl Fn(&Path) -> io::Result<File>,
) -> Result<Vec<u8>, LoginProblem> {
    let limit = max_bytes.checked_add(1).ok_or(LoginProblem::TooLarge)?;
    let file = open(path).map_err(|error| {
        if error.kind() == io::ErrorKind::NotFound {
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
pub fn read_identity_at(
    path: &Path,
    open: &impl Fn(&Path) -> io::Result<File>,
) -> Result<String, LoginProblem> {
    decode_login_identity(&read_regular_bounded(path, MAX_AUTH_BYTES, open)?)
}

pub(super) fn read_login_state(
    path: &Path,
    open: &impl Fn(&Path) -> io::Result<File>,
) -> Result<(String, bool), LoginProblem> {
    decode_login_state(&read_regular_bounded(path, MAX_AUTH_BYTES, open)?)
}
