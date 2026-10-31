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

/// Encode a transport public key as lowercase hexadecimal for the robot catalog.
pub fn encode_transport_key_hex(key: &[u8; 32]) -> String {
    const HEX: &[u8; 16] = b"0123456789abcdef";
    let mut result = String::with_capacity(64);
    for byte in key {
        result.push(HEX[usize::from(byte >> 4)] as char);
        result.push(HEX[usize::from(byte & 0x0f)] as char);
    }
    result
}

#[cfg(test)]
mod transport_key_tests {
    #[test]
    fn catalog_key_uses_lowercase_hex_in_byte_order() {
        let key = [
            0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d,
            0x0e, 0x0f, 0xf0, 0xe1, 0xd2, 0xc3, 0xb4, 0xa5, 0x96, 0x87, 0x78, 0x69, 0x5a, 0x4b,
            0x3c, 0x2d, 0x1e, 0xff,
        ];
        assert_eq!(
            super::encode_transport_key_hex(&key),
            "000102030405060708090a0b0c0d0e0ff0e1d2c3b4a5968778695a4b3c2d1eff"
        );
    }
}
