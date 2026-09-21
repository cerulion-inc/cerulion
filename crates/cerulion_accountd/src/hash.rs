// SPDX-License-Identifier: AGPL-3.0-only
//! Token hashing. We persist only the SHA-256 hash of every opaque token
//! (session / refresh / device / magic-link), never the plaintext — a database
//! read never yields a usable credential.

use sha2::{Digest, Sha256};

/// Lowercase-hex SHA-256 of an opaque token string. Deterministic, so a presented
/// token is looked up by hashing it and matching the stored hash.
pub fn hash_token(token: &str) -> String {
    let digest = Sha256::digest(token.as_bytes());
    let mut s = String::with_capacity(64);
    for b in digest {
        s.push_str(&format!("{b:02x}"));
    }
    s
}
