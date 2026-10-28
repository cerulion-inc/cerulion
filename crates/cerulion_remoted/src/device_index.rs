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
    ///
    /// On Unix, the group and other permission bits of a regular index file are
    /// cleared before it is read.
    pub fn load(path: impl AsRef<Path>, mac_key: &[u8]) -> Result<Self, RemotedError> {
        let path = path.as_ref();
        match read_private(path) {
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

    /// Atomically and durably write the index to its backing path,
    /// authenticated with `mac_key` (the SAME firmware secure-storage key the
    /// trust store uses): create a fresh sibling temporary file (exclusively, mode
    /// 0600 on Unix), write and fsync it, rename it over the index, then fsync the
    /// directory. The index takes the temporary file's mode at the rename: 0600,
    /// less any bits the umask clears. Bindings are sorted by device-key hex for a
    /// deterministic on-disk file.
    ///
    /// A failure up to and including the rename leaves the old index in place and
    /// is returned. A failed directory fsync after the rename is logged at `warn`
    /// and the save returns `Ok`, since the new index is already in place; if that
    /// save removed a device's binding, a power loss can then bring back the
    /// previous index with the binding, and the `warn` line is the only trace of
    /// it.
    ///
    /// Each save uses its own temporary file, so concurrent saves never share one
    /// and the last rename wins. The caller in this crate serializes its saves
    /// under the trust handle's writer lock.
    pub fn save(&self, mac_key: &[u8]) -> Result<(), RemotedError> {
        self.save_with(mac_key, &mut StdSaveFs)
    }

    /// [`Self::save`] over an injected [`SaveFs`].
    fn save_with(&self, mac_key: &[u8], fs: &mut impl SaveFs) -> Result<(), RemotedError> {
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
        let tmp = unique_tmp_path(path);
        // A save-side filesystem failure is a device-index *persistence* failure —
        // surface it as `Index` WITH the path + failed operation, uniform with the
        // `encode_and_mac` sibling above and every other `Index` error in this
        // module (all carry `path.display()`), so a caller handles one save-failure
        // class and the operator sees which file/op failed rather than a bare
        // context-free `Io`.
        replace_durably(fs, &tmp, path, &framed).map_err(|(step, e)| {
            let (at, op) = match step {
                SaveStep::Write => (&tmp, "writing the device index"),
                SaveStep::SyncFile => (&tmp, "syncing the device index"),
                SaveStep::Rename => (path, "renaming the device index into place"),
            };
            RemotedError::Index(format!("{}: {op} failed: {e}", at.display()))
        })
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

/// A fresh sibling name for one save's temporary file, `.<name>.<16 hex>.tmp` in
/// the index's directory. The hex hashes the process id and a per-process
/// counter under std's OS-seeded `RandomState`, so two saves (in one process or
/// two) and a crashed save's leftover do not share a name. The name is unique,
/// not secret: the exclusive create in [`StdSaveFs`] is what refuses a path that
/// is already there.
fn unique_tmp_path(path: &Path) -> PathBuf {
    use std::hash::{BuildHasher, Hasher};
    use std::sync::atomic::{AtomicU64, Ordering};
    static NEXT: AtomicU64 = AtomicU64::new(0);
    let mut hasher = std::collections::hash_map::RandomState::new().build_hasher();
    hasher.write_u32(std::process::id());
    hasher.write_u64(NEXT.fetch_add(1, Ordering::Relaxed));
    let mut name = std::ffi::OsString::from(".");
    name.push(path.file_name().unwrap_or_default());
    name.push(format!(".{:016x}.tmp", hasher.finish()));
    path.with_file_name(name)
}

/// Read the index file at `path`. On Unix, when the file is a regular file that
/// group or other users can access, its group and other permission bits are
/// cleared through the open handle before it is read, logged at `warn`. When
/// clearing them fails, that is logged at `warn` and the read goes on.
fn read_private(path: &Path) -> std::io::Result<Vec<u8>> {
    use std::io::Read as _;
    let mut file = std::fs::File::open(path)?;
    #[cfg(unix)]
    narrow_to_owner(&file, path);
    let mut bytes = Vec::new();
    file.read_to_end(&mut bytes)?;
    Ok(bytes)
}

/// Clear the group and other permission bits of the open index `file`, when it
/// is a regular file.
#[cfg(unix)]
fn narrow_to_owner(file: &std::fs::File, path: &Path) {
    use std::os::unix::fs::PermissionsExt as _;
    let mode = match file.metadata() {
        Ok(meta) if meta.is_file() => meta.permissions().mode() & 0o7777,
        Ok(_) => return,
        Err(e) => {
            tracing::warn!(
                path = %path.display(),
                error = %e,
                "could not read the device index's mode"
            );
            return;
        }
    };
    if mode & 0o077 == 0 {
        return;
    }
    let narrowed = mode & !0o077;
    match file.set_permissions(std::fs::Permissions::from_mode(narrowed)) {
        Ok(()) => tracing::warn!(
            path = %path.display(),
            mode = format_args!("{mode:o}"),
            narrowed = format_args!("{narrowed:o}"),
            "the device index was accessible to other users; cleared its group and other bits"
        ),
        Err(e) => tracing::warn!(
            path = %path.display(),
            mode = format_args!("{mode:o}"),
            error = %e,
            "the device index is accessible to other users and its group and other bits \
             could not be cleared"
        ),
    }
}

/// The step of [`replace_durably`] that failed, for the error text.
#[derive(Debug, PartialEq, Eq)]
enum SaveStep {
    Write,
    SyncFile,
    Rename,
}

/// The file-system steps [`replace_durably`] runs, one method per step, so a
/// test can record their order. [`StdSaveFs`] is the production implementation.
trait SaveFs {
    /// Create `path` for writing; it must not exist.
    fn create(&mut self, path: &Path) -> std::io::Result<std::fs::File>;
    /// Write all of `bytes` to `file`, which was created at `path`.
    fn write(&mut self, file: &mut std::fs::File, path: &Path, bytes: &[u8])
        -> std::io::Result<()>;
    /// Flush `file` (created at `path`) and its metadata to stable storage.
    fn sync_file(&mut self, file: &std::fs::File, path: &Path) -> std::io::Result<()>;
    /// Rename `from` over `to`.
    fn rename(&mut self, from: &Path, to: &Path) -> std::io::Result<()>;
    /// Flush the directory `dir`, so a rename into it is on stable storage.
    fn sync_dir(&mut self, dir: &Path) -> std::io::Result<()>;
}

/// [`SaveFs`] over `std::fs`.
struct StdSaveFs;

impl SaveFs for StdSaveFs {
    /// An exclusive create (`O_CREAT|O_EXCL`): it fails with `AlreadyExists` on
    /// any existing path, a symbolic link included, without following it (POSIX
    /// `open()` specifies `EEXIST` when `O_CREAT|O_EXCL` names a symbolic link,
    /// dangling or not; Linux and macOS do so). On Unix the file is created with
    /// mode 0600, and the rename gives the index that mode.
    fn create(&mut self, path: &Path) -> std::io::Result<std::fs::File> {
        let mut opts = std::fs::OpenOptions::new();
        opts.write(true).create_new(true);
        #[cfg(unix)]
        {
            use std::os::unix::fs::OpenOptionsExt as _;
            opts.mode(0o600);
        }
        opts.open(path)
    }
    fn write(
        &mut self,
        file: &mut std::fs::File,
        _path: &Path,
        bytes: &[u8],
    ) -> std::io::Result<()> {
        std::io::Write::write_all(file, bytes)
    }
    fn sync_file(&mut self, file: &std::fs::File, _path: &Path) -> std::io::Result<()> {
        file.sync_all()
    }
    fn rename(&mut self, from: &Path, to: &Path) -> std::io::Result<()> {
        std::fs::rename(from, to)
    }
    /// Unix opens the directory and fsyncs it. Off Unix this is a no-op.
    fn sync_dir(&mut self, dir: &Path) -> std::io::Result<()> {
        #[cfg(unix)]
        {
            std::fs::File::open(dir)?.sync_all()
        }
        #[cfg(not(unix))]
        {
            let _ = dir;
            Ok(())
        }
    }
}

/// Replace `path` with `bytes` so that a power loss at any point leaves either
/// the old file or all of the new one: write `tmp`, fsync it, rename it over
/// `path`, then fsync the directory holding `path` (the current directory for a
/// bare file name). Without the first fsync the rename can reach the disk
/// before the data does; without the second the rename itself can be lost.
///
/// A failure up to and including the rename is returned with the step that failed
/// and `path` unchanged; `tmp` is removed when this call created it. The rename is
/// the commit point: a failed directory fsync after it is logged at `warn` and the
/// replace returns `Ok`, because the new file is in place and a power loss can only
/// bring back the old one.
fn replace_durably(
    fs: &mut impl SaveFs,
    tmp: &Path,
    path: &Path,
    bytes: &[u8],
) -> Result<(), (SaveStep, std::io::Error)> {
    // A failed create made nothing, so whatever is at `tmp` is not this save's
    // and is left alone.
    let mut file = fs.create(tmp).map_err(|e| (SaveStep::Write, e))?;
    let staged = fs
        .write(&mut file, tmp, bytes)
        .map_err(|e| (SaveStep::Write, e))
        .and_then(|()| {
            fs.sync_file(&file, tmp)
                .map_err(|e| (SaveStep::SyncFile, e))
        });
    drop(file);
    if let Err(failed) =
        staged.and_then(|()| fs.rename(tmp, path).map_err(|e| (SaveStep::Rename, e)))
    {
        let _ = std::fs::remove_file(tmp);
        return Err(failed);
    }
    let dir = match path.parent() {
        Some(dir) if !dir.as_os_str().is_empty() => dir,
        _ => Path::new("."),
    };
    if let Err(e) = fs.sync_dir(dir) {
        tracing::warn!(
            dir = %dir.display(),
            error = %e,
            "the device index's directory could not be flushed after the rename"
        );
    }
    Ok(())
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

    /// One recorded [`SaveFs`] step.
    #[derive(Debug, Clone, PartialEq, Eq)]
    enum Step {
        Create(PathBuf),
        Write(PathBuf),
        SyncFile(PathBuf),
        Rename(PathBuf, PathBuf),
        SyncDir(PathBuf),
    }

    /// Records each step, then performs it through [`StdSaveFs`]; `fail_at`
    /// makes the step at that index fail instead.
    #[derive(Default)]
    struct Recorder {
        steps: Vec<Step>,
        fail_at: Option<usize>,
    }

    impl Recorder {
        fn note(&mut self, step: Step) -> std::io::Result<()> {
            self.steps.push(step);
            if self.fail_at == Some(self.steps.len() - 1) {
                return Err(std::io::Error::other("injected"));
            }
            Ok(())
        }
    }

    impl SaveFs for Recorder {
        fn create(&mut self, path: &Path) -> std::io::Result<std::fs::File> {
            self.note(Step::Create(path.to_path_buf()))?;
            StdSaveFs.create(path)
        }
        fn write(
            &mut self,
            file: &mut std::fs::File,
            path: &Path,
            bytes: &[u8],
        ) -> std::io::Result<()> {
            self.note(Step::Write(path.to_path_buf()))?;
            StdSaveFs.write(file, path, bytes)
        }
        fn sync_file(&mut self, file: &std::fs::File, path: &Path) -> std::io::Result<()> {
            self.note(Step::SyncFile(path.to_path_buf()))?;
            StdSaveFs.sync_file(file, path)
        }
        fn rename(&mut self, from: &Path, to: &Path) -> std::io::Result<()> {
            self.note(Step::Rename(from.to_path_buf(), to.to_path_buf()))?;
            StdSaveFs.rename(from, to)
        }
        fn sync_dir(&mut self, dir: &Path) -> std::io::Result<()> {
            self.note(Step::SyncDir(dir.to_path_buf()))?;
            StdSaveFs.sync_dir(dir)
        }
    }

    #[test]
    fn a_durable_replace_syncs_the_file_before_the_rename_and_the_directory_after() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("device-index");
        let tmp = unique_tmp_path(&path);
        std::fs::write(&path, b"old").unwrap();
        let mut fs = Recorder::default();
        replace_durably(&mut fs, &tmp, &path, b"new bytes").unwrap();
        assert_eq!(
            fs.steps,
            vec![
                Step::Create(tmp.clone()),
                Step::Write(tmp.clone()),
                Step::SyncFile(tmp.clone()),
                Step::Rename(tmp.clone(), path.clone()),
                Step::SyncDir(dir.path().to_path_buf()),
            ]
        );
        assert_eq!(std::fs::read(&path).unwrap(), b"new bytes");
        assert!(!tmp.exists());
    }

    #[test]
    fn each_failed_step_is_named_and_stops_the_replace() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("device-index");
        let tmp = unique_tmp_path(&path);
        std::fs::write(&path, b"old").unwrap();
        let expected = [
            SaveStep::Write,
            SaveStep::Write,
            SaveStep::SyncFile,
            SaveStep::Rename,
        ];
        for (i, want) in expected.into_iter().enumerate() {
            let mut fs = Recorder {
                fail_at: Some(i),
                ..Recorder::default()
            };
            let (step, _) = replace_durably(&mut fs, &tmp, &path, b"x").unwrap_err();
            assert_eq!(step, want, "failure injected at step {i}");
            assert_eq!(fs.steps.len(), i + 1, "nothing runs after the failed step");
            assert_eq!(
                std::fs::read(&path).unwrap(),
                b"old",
                "step {i}: old index kept"
            );
            assert!(!tmp.exists(), "step {i}: the tmp is removed");
        }
        // The directory sync follows the commit point: its failure is logged and
        // the new index stands.
        let mut fs = Recorder {
            fail_at: Some(4),
            ..Recorder::default()
        };
        replace_durably(&mut fs, &tmp, &path, b"new").unwrap();
        assert_eq!(fs.steps.len(), 5);
        assert_eq!(std::fs::read(&path).unwrap(), b"new");
    }

    #[test]
    fn save_names_the_path_and_the_step_that_failed() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("device-index");
        let index = DeviceAccountIndex::new().with_path(&path);
        let cases = [
            (1, "writing the device index failed: injected", false),
            (2, "syncing the device index failed: injected", false),
            (
                3,
                "renaming the device index into place failed: injected",
                true,
            ),
        ];
        for (fail_at, op, names_index) in cases {
            let mut fs = Recorder {
                fail_at: Some(fail_at),
                ..Recorder::default()
            };
            let err = index.save_with(MAC_KEY, &mut fs).unwrap_err();
            let Some(Step::Create(tmp)) = fs.steps.first().cloned() else {
                panic!("the first step is the create: {:?}", fs.steps);
            };
            let at = if names_index { &path } else { &tmp };
            let want = format!("{}: {op}", at.display());
            assert!(matches!(err, RemotedError::Index(_)), "{err:?}");
            assert!(err.to_string().contains(&want), "{err} vs {want}");
        }
        // The production path runs the same steps and the index loads back.
        let mut fs = Recorder::default();
        index.save_with(MAC_KEY, &mut fs).unwrap();
        let Some(Step::Create(tmp)) = fs.steps.first().cloned() else {
            panic!("the first step is the create: {:?}", fs.steps);
        };
        assert_tmp_name(&tmp, &path);
        assert_eq!(
            fs.steps,
            vec![
                Step::Create(tmp.clone()),
                Step::Write(tmp.clone()),
                Step::SyncFile(tmp.clone()),
                Step::Rename(tmp, path.clone()),
                Step::SyncDir(dir.path().to_path_buf()),
            ]
        );
        DeviceAccountIndex::load(&path, MAC_KEY).expect("the saved index loads");
    }

    /// `tmp` is `.<index name>.<16 lowercase hex>.tmp` beside `index`.
    fn assert_tmp_name(tmp: &Path, index: &Path) {
        assert_eq!(tmp.parent(), index.parent(), "{}", tmp.display());
        let name = tmp.file_name().unwrap().to_str().unwrap();
        let prefix = format!(".{}.", index.file_name().unwrap().to_str().unwrap());
        let hex = name
            .strip_prefix(&prefix)
            .and_then(|rest| rest.strip_suffix(".tmp"))
            .unwrap_or_else(|| panic!("unexpected tmp name {name}"));
        assert_eq!(hex.len(), 16, "{name}");
        assert!(
            hex.bytes()
                .all(|b| b.is_ascii_digit() || (b'a'..=b'f').contains(&b)),
            "{name}"
        );
    }

    #[test]
    fn each_save_gets_its_own_temporary_name() {
        let path = Path::new("/r/device-index");
        let a = unique_tmp_path(path);
        let b = unique_tmp_path(path);
        assert_tmp_name(&a, path);
        assert_tmp_name(&b, path);
        assert_ne!(a, b);
    }

    #[cfg(unix)]
    fn mode_of(path: &Path) -> u32 {
        use std::os::unix::fs::PermissionsExt as _;
        std::fs::metadata(path).unwrap().permissions().mode() & 0o7777
    }

    #[cfg(unix)]
    #[test]
    fn a_save_over_a_0644_index_leaves_it_0600() {
        use std::os::unix::fs::PermissionsExt as _;
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("device-index");
        std::fs::write(&path, b"old").unwrap();
        std::fs::set_permissions(&path, std::fs::Permissions::from_mode(0o644)).unwrap();
        // Control: a file created without an explicit mode keeps group or other
        // bits under this umask, so the 0600 below comes from the save.
        let probe = dir.path().join("probe");
        std::fs::File::create(&probe).unwrap();
        assert_ne!(
            mode_of(&probe) & 0o077,
            0,
            "the umask clears group and other bits, which would hide a dropped mode"
        );
        std::fs::remove_file(&probe).unwrap();
        DeviceAccountIndex::new()
            .with_path(&path)
            .save(MAC_KEY)
            .unwrap();
        assert_eq!(mode_of(&path), 0o600);
    }

    #[cfg(unix)]
    #[test]
    fn loading_a_0644_index_narrows_it_to_0600() {
        use std::os::unix::fs::PermissionsExt as _;
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("device-index");
        DeviceAccountIndex::new()
            .with_path(&path)
            .save(MAC_KEY)
            .unwrap();
        std::fs::set_permissions(&path, std::fs::Permissions::from_mode(0o644)).unwrap();
        DeviceAccountIndex::load(&path, MAC_KEY).expect("a narrowed index still loads");
        assert_eq!(mode_of(&path), 0o600);
    }

    #[cfg(unix)]
    #[test]
    fn a_planted_symlink_at_a_temporary_name_is_not_followed() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("device-index");
        let victim = dir.path().join("victim");
        std::fs::write(&victim, b"keep").unwrap();
        // The fixed name saves used before, and a name shaped like a fresh one.
        let fixed = dir.path().join("device-index.tmp");
        let shaped = dir.path().join(".device-index.0123456789abcdef.tmp");
        for link in [&fixed, &shaped] {
            std::os::unix::fs::symlink(&victim, link).unwrap();
            let err = StdSaveFs
                .create(link)
                .expect_err("an existing path is refused");
            assert_eq!(err.kind(), std::io::ErrorKind::AlreadyExists, "{err}");
            let (step, err) = replace_durably(&mut StdSaveFs, link, &path, b"new").unwrap_err();
            assert_eq!(step, SaveStep::Write);
            assert_eq!(err.kind(), std::io::ErrorKind::AlreadyExists, "{err}");
            assert!(
                link.symlink_metadata().is_ok(),
                "the planted path is left in place"
            );
        }
        assert_eq!(
            std::fs::read(&victim).unwrap(),
            b"keep",
            "the target is unchanged"
        );
        assert!(!path.exists(), "nothing was renamed into place");
        DeviceAccountIndex::new()
            .with_path(&path)
            .save(MAC_KEY)
            .unwrap();
        DeviceAccountIndex::load(&path, MAC_KEY).expect("the saved index loads");
        assert_eq!(std::fs::read(&victim).unwrap(), b"keep");
    }

    #[test]
    fn concurrent_saves_both_succeed_and_leave_a_valid_index() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("device-index");
        let mut index = DeviceAccountIndex::new().with_path(&path);
        index.bind(PublicKey([7; 32]), AccountId([9; 32]));
        let index = std::sync::Arc::new(index);
        let threads: Vec<_> = (0..2)
            .map(|_| {
                let index = std::sync::Arc::clone(&index);
                std::thread::spawn(move || {
                    for _ in 0..10 {
                        index.save(MAC_KEY).expect("a concurrent save succeeds");
                    }
                })
            })
            .collect();
        for t in threads {
            t.join().unwrap();
        }
        let loaded =
            DeviceAccountIndex::load(&path, MAC_KEY).expect("the final index has a valid MAC");
        assert_eq!(
            loaded.account_for(&PublicKey([7; 32])),
            Some(AccountId([9; 32]))
        );
        let leftovers: Vec<_> = std::fs::read_dir(dir.path())
            .unwrap()
            .map(|e| e.unwrap().file_name())
            .filter(|n| n != "device-index")
            .collect();
        assert!(
            leftovers.is_empty(),
            "no temporary file is left: {leftovers:?}"
        );
    }
}
