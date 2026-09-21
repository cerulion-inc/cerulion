// SPDX-License-Identifier: MIT OR Apache-2.0
//! Thin, crate-internal wrappers over the vetted primitive crates. This module
//! never hand-rolls curve/hash arithmetic — it only composes ed25519-dalek, sha2, hmac, and
//! subtle. Nothing here is public API.

use ed25519_dalek::{Signer, SigningKey, VerifyingKey};
use hmac::{Hmac, Mac};
use sha2::{Digest, Sha256};
use subtle::ConstantTimeEq;

use crate::format::{PublicKey, Signature};

/// Outcome of an ed25519 verification, distinguishing a structurally invalid key
/// from a bad signature so callers can surface the precise error.
pub(crate) enum VerifyResult {
    Ok,
    BadKey,
    BadSig,
}

/// Strictly verify an ed25519 signature. `verify_strict` rejects non-canonical
/// encodings and small-order public keys.
pub(crate) fn ed25519_verify(key: &PublicKey, msg: &[u8], sig: &Signature) -> VerifyResult {
    let vk = match VerifyingKey::from_bytes(&key.0) {
        Ok(vk) => vk,
        Err(_) => return VerifyResult::BadKey,
    };
    let s = ed25519_dalek::Signature::from_bytes(&sig.0);
    match vk.verify_strict(msg, &s) {
        Ok(()) => VerifyResult::Ok,
        Err(_) => VerifyResult::BadSig,
    }
}

/// Sign `msg` with an ed25519 signing key.
pub(crate) fn ed25519_sign(key: &SigningKey, msg: &[u8]) -> Signature {
    Signature(key.sign(msg).to_bytes())
}

/// HMAC-SHA256 over `data` with `mac_key`.
pub(crate) fn hmac_sha256(mac_key: &[u8], data: &[u8]) -> [u8; 32] {
    let mut mac =
        <Hmac<Sha256> as Mac>::new_from_slice(mac_key).expect("HMAC accepts a key of any length");
    mac.update(data);
    mac.finalize().into_bytes().into()
}

/// SHA-256 of `domain || data` — used to store a hash of the chassis secret
/// (never the raw secret).
pub(crate) fn sha256_domain(domain: &[u8], data: &[u8]) -> [u8; 32] {
    let mut h = Sha256::new();
    h.update(domain);
    h.update(data);
    h.finalize().into()
}

/// Constant-time equality. Returns `false` for unequal lengths (without
/// branching on the contents of equal-length inputs).
pub(crate) fn ct_eq(a: &[u8], b: &[u8]) -> bool {
    if a.len() != b.len() {
        return false;
    }
    a.ct_eq(b).into()
}

/// Lowercase hex encoding (no external crate).
pub(crate) fn to_hex(bytes: &[u8]) -> String {
    const HEX: &[u8; 16] = b"0123456789abcdef";
    let mut s = String::with_capacity(bytes.len() * 2);
    for &b in bytes {
        s.push(HEX[(b >> 4) as usize] as char);
        s.push(HEX[(b & 0x0f) as usize] as char);
    }
    s
}
