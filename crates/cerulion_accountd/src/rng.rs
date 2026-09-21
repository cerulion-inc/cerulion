// SPDX-License-Identifier: AGPL-3.0-only
//! Cryptographic randomness helpers over the platform RNG (`getrandom`).
//!
//! Every value the service mints from entropy comes through here: ed25519 key
//! seeds (dev provisioning), opaque session / refresh / device / magic-link
//! tokens, the RFC 8628 human `user_code`, and stable 32-byte account / robot ids.

use base64::engine::general_purpose::URL_SAFE_NO_PAD;
use base64::Engine;

use crate::error::{AccountdError, Result};

/// Fill an `N`-byte array with cryptographic entropy.
pub fn fill<const N: usize>() -> Result<[u8; N]> {
    let mut buf = [0u8; N];
    getrandom::fill(&mut buf).map_err(|e| AccountdError::Random(e.to_string()))?;
    Ok(buf)
}

/// A stable 32-byte identifier (an `AccountId` / `RobotId` payload — distinct from
/// any signing key, so key rotation never strands the identifier).
pub fn id_32() -> Result<[u8; 32]> {
    fill::<32>()
}

/// An opaque high-entropy token (32 bytes → base64url, no padding). Used for
/// session tokens, refresh tokens, device codes, and magic-link tokens. We store
/// only the SHA-256 hash of the returned string — never the plaintext.
pub fn opaque_token() -> Result<String> {
    Ok(URL_SAFE_NO_PAD.encode(fill::<32>()?))
}

/// The RFC 8628 human `user_code`: 8 characters from an unambiguous base-20
/// alphabet (no vowels, no easily-confused glyphs), grouped as `XXXX-XXXX`.
pub fn user_code() -> Result<String> {
    // 20 unambiguous consonants (no vowels → no accidental words; no 0/O/1/I/L).
    const ALPHABET: &[u8; 20] = b"BCDFGHJKLMNPQRSTVWXZ";
    let mut out = String::with_capacity(9);
    let mut drawn = 0usize;
    while drawn < 8 {
        // Rejection-sample to avoid modulo bias (240 = 20 * 12 is the largest
        // multiple of 20 that fits in a byte).
        let b = fill::<1>()?[0];
        if b >= 240 {
            continue;
        }
        if drawn == 4 {
            out.push('-');
        }
        out.push(ALPHABET[(b % 20) as usize] as char);
        drawn += 1;
    }
    Ok(out)
}
