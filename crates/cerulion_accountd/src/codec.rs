// SPDX-License-Identifier: AGPL-3.0-only
//! Byte-deterministic transport envelope for pairing artifacts.
//!
//! A `SignedDeviceCert` / `SignedGrant` / `RootSet` crosses the HTTP surface as
//! `base64url(postcard(value))` inside a JSON field. postcard is exactly the
//! container format `cerulion_pairing` itself uses on disk / on the wire, so the
//! encoding is deterministic and round-trips byte-for-byte through the shipped
//! verifier — the cross-check the acceptance test pins.

use base64::engine::general_purpose::URL_SAFE_NO_PAD;
use base64::Engine;
use serde::{de::DeserializeOwned, Serialize};

use crate::error::{AccountdError, Result};

/// Serialize `value` to postcard, then base64url (no padding).
pub fn encode_b64<T: Serialize>(value: &T) -> Result<String> {
    let bytes =
        postcard::to_stdvec(value).map_err(|e| AccountdError::Serialization(e.to_string()))?;
    Ok(URL_SAFE_NO_PAD.encode(bytes))
}

/// Inverse of [`encode_b64`].
pub fn decode_b64<T: DeserializeOwned>(s: &str) -> Result<T> {
    let bytes = URL_SAFE_NO_PAD
        .decode(s.as_bytes())
        .map_err(|e| AccountdError::Serialization(format!("base64: {e}")))?;
    postcard::from_bytes(&bytes).map_err(|e| AccountdError::Serialization(e.to_string()))
}
