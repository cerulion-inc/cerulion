// SPDX-License-Identifier: AGPL-3.0-only
//! Convert account-service identities into the pairing protocol's 32-byte id.
//!
//! Authentication retains the service's original account id. UUID-backed accounts
//! use a domain-separated mapping; legacy unpadded base64url ids retain their bytes.

use base64::engine::general_purpose::URL_SAFE_NO_PAD;
use base64::Engine;
use cerulion_pairing::format::AccountId;
pub(crate) use cerulion_pairing::identity::uuid_bytes;

use crate::error::{CliError, CliResult};

/// Resolve a UUID or an existing base64url pairing id without changing auth state.
/// Opaque legacy authentication labels are not pairing identities.
pub fn pairing_account_id(account: &str) -> CliResult<AccountId> {
    if let Some(account) = cerulion_pairing::identity::account_from_uuid(account) {
        return Ok(account);
    }
    let bytes = URL_SAFE_NO_PAD
        .decode(account)
        .ok()
        .and_then(|bytes| bytes.try_into().ok());
    bytes.map(AccountId).ok_or_else(|| {
        CliError::Login(
            "the login account is neither a UUID nor a 32-byte base64url pairing identity".into(),
        )
    })
}
