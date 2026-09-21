// SPDX-License-Identifier: AGPL-3.0-only
//! The `device_key → account` side-map (R1).
//!
//! A `cerulion_pairing` `AccessRow` stores the *account*, not the device key, so
//! the accept gate cannot map a TLS-authenticated `remote_id` (a device key) to
//! an account from the trust store alone. `remoted` owns this small persisted
//! index — written at pairing time — and reads it at every accept.
//!
//! ## On-disk format (MAC-authenticated)
//!
//! The index is the SOLE `device_key → account` binding the accept gate trusts
//! ([`crate::authorizer::PairingAuthorizer`] maps a TLS-authed key → account →
//! access row), so it MUST be tamper-evident — a disk-tamper adversary who could
//! write a `{their key → owner account}` binding would otherwise escalate to
//! OWNER, bypassing the trust store's HMAC entirely. The file is therefore
//! **MAC-authenticated with the SAME firmware secure-storage MAC key as the
//! trust store**, mirroring `cerulion_pairing`'s store framing:
//!
//! ```text
//! magic "CERIDX01" (8) | format_version u16-LE (2) | body_len u32-LE (4)
//!   | body = pretty-JSON({version, bindings}) | HMAC-SHA256 tag (32)
//! ```
//!
//! The HMAC covers the header + body; on load the tag is verified FIRST (in
//! constant time) before any field is structurally trusted. Writes are atomic
//! (`.tmp` then `rename`). The JSON body stays human-readable (inspectable in a
//! text/hex editor); a device key can bind to exactly one account, but one
//! account may have many device keys (many bindings).
//!
//! ## Fail-closed on corruption / tampering
//!
//! An **absent** file is a fresh, empty index (no device paired yet — every
//! unpaired key then correctly falls to the bootstrap surface). A **present**
//! file with a bad/missing MAC, bad magic/version/length, or corrupt inner JSON
//! is a LOUD error, never silently reset to empty: silently dropping every
//! binding would deny-by-default every previously-paired device (a mass
//! availability failure) AND — critically — would let a tamper adversary DELETE
//! or truncate the MAC to reset the index. The daemon refuses to start rather
//! than proceed on a corrupt/tampered index.

use std::collections::HashMap;
use std::path::{Path, PathBuf};

use cerulion_pairing::format::{AccountId, PublicKey};
use serde::{Deserialize, Serialize};

use crate::error::RemotedError;

/// The on-disk format version (bumped on a breaking layout change).
const INDEX_FORMAT_VERSION: u16 = 1;
/// On-disk framing magic (8 bytes) — mirrors `cerulion_pairing`'s trust-store
/// `CERPAIR\x01` convention.
const INDEX_MAGIC: [u8; 8] = *b"CERIDX01";
/// HMAC-SHA256 tag length appended after the body.
const INDEX_TAG_LEN: usize = 32;
/// Header length: magic (8) + version (2) + body_len (4).
const INDEX_HEADER_LEN: usize = 8 + 2 + 4;

/// The `device_key → account` side-map. Reads at accept; the daemon writes it at
/// pairing time.
#[derive(Debug, Clone, Default)]
pub struct DeviceAccountIndex {
    map: HashMap<PublicKey, AccountId>,
    path: Option<PathBuf>,
}

#[derive(Debug, Serialize, Deserialize)]
struct OnDisk {
    version: u16,
    bindings: Vec<OnDiskBinding>,
}

#[derive(Debug, Serialize, Deserialize)]
struct OnDiskBinding {
    /// Hex-encoded 32-byte device (transport) public key.
    device_key: String,
    /// Hex-encoded 32-byte account id.
    account: String,
}

impl DeviceAccountIndex {
    /// An empty in-memory index with no backing path.
    pub fn new() -> Self {
        DeviceAccountIndex::default()
    }

    /// Attach a backing file path (does not write).
    pub fn with_path(mut self, path: impl Into<PathBuf>) -> Self {
        self.path = Some(path.into());
        self
    }

    /// Load and MAC-verify the index from disk, authenticating with `mac_key`
    /// (the SAME firmware secure-storage key the trust store uses).
    ///
    /// - **Absent file** → a fresh, empty index (bound to `path`). Benign: an
    ///   empty index maps no key to an account.
    /// - **Present** but with a bad/missing MAC, bad magic/version/length, or
    ///   corrupt inner JSON/hex → a LOUD [`RemotedError::Index`] (fail-closed —
    ///   never silently reset to empty; see the module docs). The MAC is verified
    ///   in constant time BEFORE any field is structurally trusted, so a
    ///   disk-tamper adversary cannot forge a `{their key → owner account}`
    ///   binding (nor delete the MAC to reset the index — a present file with a
    ///   missing/short frame is refused, only a fully ABSENT file is empty).
    pub fn load(path: impl AsRef<Path>, mac_key: &[u8]) -> Result<Self, RemotedError> {
        let path = path.as_ref();
        match std::fs::read(path) {
            Ok(bytes) => {
                let on_disk = decode_and_verify(&bytes, mac_key, &path.display().to_string())?;
                let mut map = HashMap::with_capacity(on_disk.bindings.len());
                for b in on_disk.bindings {
                    let key = parse_key32(&b.device_key).map_err(|e| {
                        RemotedError::Index(format!("{}: bad device_key: {e}", path.display()))
                    })?;
                    let account = parse_key32(&b.account).map_err(|e| {
                        RemotedError::Index(format!("{}: bad account: {e}", path.display()))
                    })?;
                    map.insert(PublicKey(key), AccountId(account));
                }
                Ok(DeviceAccountIndex {
                    map,
                    path: Some(path.to_path_buf()),
                })
            }
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => Ok(DeviceAccountIndex {
                map: HashMap::new(),
                path: Some(path.to_path_buf()),
            }),
            Err(e) => Err(RemotedError::Io(e)),
        }
    }

    /// Atomically write the index to its backing path (`.tmp` then `rename`),
    /// authenticated with `mac_key` (the SAME firmware secure-storage key the
    /// trust store uses). Bindings are sorted by device-key hex for a
    /// deterministic on-disk file.
    pub fn save(&self, mac_key: &[u8]) -> Result<(), RemotedError> {
        let path = self.path.as_ref().ok_or_else(|| {
            RemotedError::Index("no backing path set for the device index".into())
        })?;
        let mut bindings: Vec<OnDiskBinding> = self
            .map
            .iter()
            .map(|(k, a)| OnDiskBinding {
                device_key: hex::encode(k.0),
                account: hex::encode(a.0),
            })
            .collect();
        bindings.sort_by(|a, b| a.device_key.cmp(&b.device_key));
        let on_disk = OnDisk {
            version: INDEX_FORMAT_VERSION,
            bindings,
        };
        let framed = encode_and_mac(&on_disk, mac_key)?;
        let tmp = tmp_path(path);
        // A save-side filesystem failure is a device-index *persistence* failure —
        // surface it as `Index` WITH the path + failed operation, uniform with the
        // `encode_and_mac` sibling above and every other `Index` error in this
        // module (all carry `path.display()`), so a caller handles one save-failure
        // class and the operator sees which file/op failed rather than a bare
        // context-free `Io`.
        std::fs::write(&tmp, &framed).map_err(|e| {
            RemotedError::Index(format!(
                "{}: writing the device index failed: {e}",
                tmp.display()
            ))
        })?;
        std::fs::rename(&tmp, path).map_err(|e| {
            RemotedError::Index(format!(
                "{}: renaming the device index into place failed: {e}",
                path.display()
            ))
        })?;
        Ok(())
    }

    /// Bind a device key to an account (written at pairing time). A device key
    /// binds to exactly one account (a re-bind replaces it); one account may
    /// have many device keys.
    pub fn bind(&mut self, device_key: PublicKey, account: AccountId) {
        self.map.insert(device_key, account);
    }

    /// The account a device key is bound to, if any.
    pub fn account_for(&self, device_key: &PublicKey) -> Option<AccountId> {
        self.map.get(device_key).copied()
    }

    /// The number of bound device keys.
    pub fn len(&self) -> usize {
        self.map.len()
    }

    /// Whether the index is empty.
    pub fn is_empty(&self) -> bool {
        self.map.is_empty()
    }
}

/// Frame + MAC the JSON body: `magic | version | body_len | body | tag`, with
/// `tag = HMAC-SHA256(mac_key, magic|version|body_len|body)`.
fn encode_and_mac(on_disk: &OnDisk, mac_key: &[u8]) -> Result<Vec<u8>, RemotedError> {
    let body = serde_json::to_vec_pretty(on_disk)
        .map_err(|e| RemotedError::Index(format!("serialize: {e}")))?;
    let body_len = u32::try_from(body.len()).map_err(|_| {
        RemotedError::Index(
            "device index body exceeds u32::MAX (would truncate the length prefix)".into(),
        )
    })?;
    let mut framed = Vec::with_capacity(INDEX_HEADER_LEN + body.len() + INDEX_TAG_LEN);
    framed.extend_from_slice(&INDEX_MAGIC);
    framed.extend_from_slice(&INDEX_FORMAT_VERSION.to_le_bytes());
    framed.extend_from_slice(&body_len.to_le_bytes());
    framed.extend_from_slice(&body);
    let tag = hmac_sha256(mac_key, &framed);
    framed.extend_from_slice(&tag);
    Ok(framed)
}

/// Verify the MAC (constant time, FIRST) then the structural frame, returning the
/// decoded [`OnDisk`] body. A tampered / mis-keyed / structurally-broken file is a
/// LOUD [`RemotedError::Index`] (fail-closed) — never a silent empty index.
fn decode_and_verify(bytes: &[u8], mac_key: &[u8], path: &str) -> Result<OnDisk, RemotedError> {
    if bytes.len() < INDEX_HEADER_LEN + INDEX_TAG_LEN {
        return Err(RemotedError::Index(format!(
            "{path} is shorter than the device-index header + MAC tag \
             (present-but-truncated — refusing to start on a corrupt/tampered index rather \
             than silently dropping every pairing; fail-closed)"
        )));
    }
    let (framed, tag) = bytes.split_at(bytes.len() - INDEX_TAG_LEN);
    // Authenticity BEFORE structurally trusting any field (constant-time).
    let expected = hmac_sha256(mac_key, framed);
    if !ct_eq(&expected, tag) {
        return Err(RemotedError::Index(format!(
            "{path} MAC verification FAILED — the device index was tampered with, or the \
             secure-storage MAC key does not match. The device→account side-map is the sole \
             binding the accept gate trusts, so it is refused rather than silently reset \
             (which would let an attacker forge or delete a binding); fail-closed"
        )));
    }
    if framed[..8] != INDEX_MAGIC {
        return Err(RemotedError::Index(format!(
            "{path} has a bad device-index magic (not a Cerulion device index)"
        )));
    }
    let ver = u16::from_le_bytes([framed[8], framed[9]]);
    if ver != INDEX_FORMAT_VERSION {
        return Err(RemotedError::Index(format!(
            "{path} has unsupported device-index version {ver} (expected {INDEX_FORMAT_VERSION})"
        )));
    }
    let body_len = u32::from_le_bytes([framed[10], framed[11], framed[12], framed[13]]) as usize;
    let body = &framed[INDEX_HEADER_LEN..];
    if body.len() != body_len {
        return Err(RemotedError::Index(format!(
            "{path} device-index body length mismatch (declared {body_len}, got {})",
            body.len()
        )));
    }
    let on_disk: OnDisk = serde_json::from_slice(body).map_err(|e| {
        RemotedError::Index(format!(
            "{path} is not valid device-index JSON: {e} \
             (refusing to start on a corrupt index rather than silently dropping every \
             pairing — fail-closed)"
        ))
    })?;
    if on_disk.version != INDEX_FORMAT_VERSION {
        return Err(RemotedError::Index(format!(
            "{path} has unsupported inner device-index version {} (expected {INDEX_FORMAT_VERSION})",
            on_disk.version
        )));
    }
    Ok(on_disk)
}

/// HMAC-SHA256 over `data` with `mac_key` (mirrors `cerulion_pairing::crypto`,
/// which is crate-private there — we compose the same vetted primitives here).
fn hmac_sha256(mac_key: &[u8], data: &[u8]) -> [u8; 32] {
    use hmac::{Hmac, Mac};
    use sha2::Sha256;
    let mut mac =
        <Hmac<Sha256> as Mac>::new_from_slice(mac_key).expect("HMAC accepts a key of any length");
    mac.update(data);
    mac.finalize().into_bytes().into()
}

/// Constant-time equality (never a data-dependent `==` on a MAC tag). Returns
/// `false` for unequal lengths without branching on equal-length contents.
fn ct_eq(a: &[u8], b: &[u8]) -> bool {
    use subtle::ConstantTimeEq;
    if a.len() != b.len() {
        return false;
    }
    a.ct_eq(b).into()
}

/// Sibling temp path for atomic writes: append `.tmp` to the file name.
fn tmp_path(path: &Path) -> PathBuf {
    let mut name = path.file_name().unwrap_or_default().to_os_string();
    name.push(".tmp");
    path.with_file_name(name)
}

/// Decode a 32-byte value from a hex string, erroring loudly on bad hex or the
/// wrong length.
fn parse_key32(s: &str) -> Result<[u8; 32], String> {
    let bytes = hex::decode(s).map_err(|e| format!("invalid hex: {e}"))?;
    bytes
        .try_into()
        .map_err(|v: Vec<u8>| format!("expected 32 bytes, got {}", v.len()))
}

#[cfg(test)]
mod tests {
    use super::*;

    const MAC_KEY: &[u8] = b"device-index-unit-mac-key-32bytes!";

    /// A valid MAC-framed file with an arbitrary JSON body — the ONLY way
    /// to exercise the inner-content parse (bad hex / wrong length / version) now
    /// that the MAC gates before the JSON. An attacker cannot produce this
    /// without the key, so these are defense-in-depth against a writer bug.
    fn frame_body(body: &[u8], mac_key: &[u8]) -> Vec<u8> {
        let mut framed = Vec::new();
        framed.extend_from_slice(&INDEX_MAGIC);
        framed.extend_from_slice(&INDEX_FORMAT_VERSION.to_le_bytes());
        framed.extend_from_slice(&(body.len() as u32).to_le_bytes());
        framed.extend_from_slice(body);
        let tag = hmac_sha256(mac_key, &framed);
        framed.extend_from_slice(&tag);
        framed
    }

    #[test]
    fn encode_then_decode_round_trips_the_body() {
        let on_disk = OnDisk {
            version: INDEX_FORMAT_VERSION,
            bindings: vec![OnDiskBinding {
                device_key: hex::encode([1u8; 32]),
                account: hex::encode([10u8; 32]),
            }],
        };
        let framed = encode_and_mac(&on_disk, MAC_KEY).unwrap();
        let decoded = decode_and_verify(&framed, MAC_KEY, "t").unwrap();
        assert_eq!(decoded.bindings.len(), 1);
        assert_eq!(decoded.bindings[0].device_key, hex::encode([1u8; 32]));
    }

    #[test]
    fn a_flipped_tag_byte_fails_the_mac() {
        let on_disk = OnDisk {
            version: INDEX_FORMAT_VERSION,
            bindings: vec![],
        };
        let mut framed = encode_and_mac(&on_disk, MAC_KEY).unwrap();
        let last = framed.len() - 1;
        framed[last] ^= 0x01; // flip one tag bit
        let err = decode_and_verify(&framed, MAC_KEY, "t")
            .unwrap_err()
            .to_string();
        assert!(err.contains("MAC verification FAILED"), "err: {err}");
    }

    #[test]
    fn a_flipped_body_byte_fails_the_mac() {
        let on_disk = OnDisk {
            version: INDEX_FORMAT_VERSION,
            bindings: vec![OnDiskBinding {
                device_key: hex::encode([7u8; 32]),
                account: hex::encode([9u8; 32]),
            }],
        };
        let mut framed = encode_and_mac(&on_disk, MAC_KEY).unwrap();
        // Flip a byte inside the JSON body (after the 14-byte header).
        framed[INDEX_HEADER_LEN + 1] ^= 0x20;
        let err = decode_and_verify(&framed, MAC_KEY, "t")
            .unwrap_err()
            .to_string();
        assert!(err.contains("MAC verification FAILED"), "err: {err}");
    }

    #[test]
    fn the_wrong_mac_key_is_refused() {
        let on_disk = OnDisk {
            version: INDEX_FORMAT_VERSION,
            bindings: vec![],
        };
        let framed = encode_and_mac(&on_disk, MAC_KEY).unwrap();
        let err = decode_and_verify(&framed, b"a-different-key", "t")
            .unwrap_err()
            .to_string();
        assert!(err.contains("MAC verification FAILED"), "err: {err}");
    }

    #[test]
    fn a_valid_mac_over_bad_inner_hex_is_a_loud_error_through_load() {
        // A validly-MAC'd body whose device_key hex is not 32 bytes. The MAC and
        // frame pass; the inner hex parse in `load` surfaces the loud error (the
        // second defense-in-depth layer, only reachable behind a valid MAC).
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("badhex.idx");
        let bad = br#"{"version":1,"bindings":[{"device_key":"zz","account":"00"}]}"#;
        std::fs::write(&path, frame_body(bad, MAC_KEY)).unwrap();
        let err = DeviceAccountIndex::load(&path, MAC_KEY)
            .unwrap_err()
            .to_string();
        assert!(err.contains("bad device_key"), "err: {err}");
    }

    #[test]
    fn a_valid_mac_over_wrong_length_key_is_a_loud_error_through_load() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("shortkey.idx");
        let short = hex::encode([1u8]); // 1 byte where 32 are required
        let full = hex::encode([2u8; 32]);
        let body = format!(
            r#"{{"version":1,"bindings":[{{"device_key":"{short}","account":"{full}"}}]}}"#
        );
        std::fs::write(&path, frame_body(body.as_bytes(), MAC_KEY)).unwrap();
        let err = DeviceAccountIndex::load(&path, MAC_KEY)
            .unwrap_err()
            .to_string();
        assert!(err.contains("bad device_key"), "err: {err}");
        assert!(err.contains("expected 32 bytes"), "err: {err}");
    }

    #[test]
    fn a_valid_mac_over_bad_magic_is_refused() {
        let on_disk = OnDisk {
            version: INDEX_FORMAT_VERSION,
            bindings: vec![],
        };
        let mut framed = encode_and_mac(&on_disk, MAC_KEY).unwrap();
        framed[0] ^= 0xff; // corrupt magic, then re-MAC so the tag is valid
        let reframed = {
            let (body, _tag) = framed.split_at(framed.len() - INDEX_TAG_LEN);
            let mut v = body.to_vec();
            v.extend_from_slice(&hmac_sha256(MAC_KEY, body));
            v
        };
        let err = decode_and_verify(&reframed, MAC_KEY, "t")
            .unwrap_err()
            .to_string();
        assert!(err.contains("bad device-index magic"), "err: {err}");
    }

    #[test]
    fn a_valid_mac_over_unsupported_version_is_refused() {
        // Craft magic + version=9999 + a matching JSON body, then MAC it.
        let body = br#"{"version":9999,"bindings":[]}"#;
        let mut framed = Vec::new();
        framed.extend_from_slice(&INDEX_MAGIC);
        framed.extend_from_slice(&9999u16.to_le_bytes()); // frame version 9999
        framed.extend_from_slice(&(body.len() as u32).to_le_bytes());
        framed.extend_from_slice(body);
        let tag = hmac_sha256(MAC_KEY, &framed);
        framed.extend_from_slice(&tag);
        let err = decode_and_verify(&framed, MAC_KEY, "t")
            .unwrap_err()
            .to_string();
        assert!(
            err.contains("unsupported device-index version 9999"),
            "err: {err}"
        );
    }

    #[test]
    fn a_truncated_file_is_refused_not_treated_as_empty() {
        // A present-but-too-short file (e.g. an attacker deleted the MAC to reset).
        let err = decode_and_verify(b"tiny", MAC_KEY, "t")
            .unwrap_err()
            .to_string();
        assert!(err.contains("shorter than"), "err: {err}");
    }
}
