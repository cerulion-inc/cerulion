// SPDX-License-Identifier: AGPL-3.0-only
//! Pure decoding of a local login's public certificate material.
//!
//! This checks representation and identity consistency. The robot remains the
//! trust anchor: it verifies signatures, validity, ownership and revocation.

use crate::{ConnectError, ConnectResult};
use base64::engine::general_purpose::URL_SAFE_NO_PAD;
use base64::Engine;
use cerulion_pairing::format::{
    AccountId, PublicKey, SignedDeviceCert, SignedIntermediateCert, FORMAT_VERSION,
};
use cerulion_pairing::verify::OwnerCertificatePresentationWire;
use serde::Deserialize;

/// Resolve hosted UUIDs or legacy unpadded base64url pairing identities.
pub fn pairing_account_id(value: &str) -> ConnectResult<AccountId> {
    if let Some(account) = cerulion_pairing::identity::account_from_uuid(value) {
        return Ok(account);
    }
    URL_SAFE_NO_PAD
        .decode(value)
        .ok()
        .and_then(|bytes| bytes.try_into().ok())
        .map(AccountId)
        .ok_or_else(|| invalid("login account is not a pairing identity"))
}

/// A decoded owner envelope and its canonical public postcard representation.
/// The bytes match the CLI's `OwnerCertificatePresentationWire::to_postcard`.
pub struct CachedOwnerChain {
    /// Public certificates to carry for a new owner enrollment.
    pub presentation: OwnerCertificatePresentationWire,
    /// Versioned envelope bytes; no bearer token or private key.
    pub postcard: Vec<u8>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct CachedChain {
    device_cert: String,
    intermediate: String,
}

/// Validate a present leaf even when its legacy login has no complete chain.
/// Expiry is deliberately not checked: elapsed time must not change local identity
/// or evict an established durable pairing. New enrollment is robot-verified.
pub fn decode_owner_chain(
    leaf: &[u8],
    cache: Option<&[u8]>,
    account: AccountId,
    key: PublicKey,
) -> ConnectResult<Option<CachedOwnerChain>> {
    let text = std::str::from_utf8(leaf).map_err(|_| invalid("leaf is not UTF-8"))?;
    let cert: SignedDeviceCert = decode(text.trim())?;
    if cert.cert.version != FORMAT_VERSION
        || cert.cert.validity.not_before_ns >= cert.cert.validity.not_after_ns
    {
        return Err(invalid("leaf has an invalid format or validity interval"));
    }
    if cert.cert.account != account || cert.cert.device_key != key {
        return Err(invalid(
            "leaf does not match the current login and device key",
        ));
    }
    let Some(cache) = cache else {
        return Ok(None);
    };
    let cached: CachedChain =
        serde_json::from_slice(cache).map_err(|_| invalid("chain cache is malformed"))?;
    if cached.device_cert != text.trim() {
        return Err(invalid("chain cache does not match the current leaf"));
    }
    let intermediate: SignedIntermediateCert = decode(&cached.intermediate)?;
    if intermediate.cert.version != FORMAT_VERSION
        || intermediate.cert.validity.not_before_ns >= intermediate.cert.validity.not_after_ns
        || intermediate.signatures.is_empty()
    {
        return Err(invalid(
            "intermediate has an invalid format or validity interval",
        ));
    }
    if cert.cert.issuer_key != intermediate.cert.intermediate_key {
        return Err(invalid("leaf and intermediate name different issuers"));
    }
    let presentation = OwnerCertificatePresentationWire::new(intermediate, cert);
    let postcard = presentation
        .to_postcard()
        .map_err(|_| invalid("owner envelope could not be encoded"))?;
    Ok(Some(CachedOwnerChain {
        presentation,
        postcard,
    }))
}

fn decode<T: serde::de::DeserializeOwned>(value: &str) -> ConnectResult<T> {
    let bytes = URL_SAFE_NO_PAD
        .decode(value)
        .map_err(|_| invalid("certificate is not base64url"))?;
    let (value, remainder) = postcard::take_from_bytes(&bytes)
        .map_err(|_| invalid("certificate postcard is malformed"))?;
    if !remainder.is_empty() {
        return Err(invalid("certificate postcard has trailing bytes"));
    }
    Ok(value)
}

fn invalid(reason: &'static str) -> ConnectError {
    ConnectError::DeviceCert(reason.into())
}
