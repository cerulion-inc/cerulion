// SPDX-License-Identifier: AGPL-3.0-only
//! The append-only, hash-chained on-robot receipt audit log.
//!
//! Every verb invocation is recorded: who called, which verb, a digest of the
//! args, a timestamp, and the outcome. Each entry carries the previous entry's
//! hash, so the log is a tamper-evident chain — altering any past field breaks
//! every subsequent link, detectable by [`verify_chain`].
//!
//! # Durability & crash safety
//!
//! - Every append `sync_data()`s BEFORE it returns, so a recorded outcome is on
//!   disk before the caller's response is acknowledged.
//! - Opening a log tolerates exactly ONE crash-torn final line: it truncates
//!   the file back to the last intact newline (a loud `warn!`) and continues
//!   the chain. A non-final unparseable line stays a HARD error (tamper
//!   evidence). Recovery reads only a bounded tail — never the whole file.
//! - Size-based rotation: when the current file exceeds a cap it becomes
//!   `<log>.1` (shifting `.1`→`.2`, keeping N), and the fresh file opens with a
//!   `__rotation__` anchor whose `prev_hash` links across the boundary — so the
//!   chain is continuous and rotation is itself receipted + tamper-evident.
//!
//! The on-disk format is JSON-lines (one [`Receipt`] per line), append-only.

use std::fs::{File, OpenOptions};
use std::io::{Read, Seek, SeekFrom, Write};
use std::path::{Path, PathBuf};

use serde::{Deserialize, Serialize};

use crate::error::{CerudError, CerudResult};
use crate::hash::{canonical_json_bytes, sha256_hex};

/// The genesis previous-hash for the first entry (64 hex zeros).
pub const GENESIS_HASH: &str = "0000000000000000000000000000000000000000000000000000000000000000";

/// The synthetic verb recorded as the first entry of a freshly-rotated log.
/// It links the chain across the rotation boundary (tamper-evident) and makes
/// the rotation itself an auditable, chained event.
pub const ROTATION_VERB: &str = "__rotation__";

/// Default rotation size cap (8 MiB) — a full file becomes `<log>.1`.
pub const DEFAULT_MAX_LOG_BYTES: u64 = 8 * 1024 * 1024;

/// Default number of rotated files retained (`<log>.1` .. `<log>.N`).
pub const DEFAULT_MAX_ROTATIONS: usize = 4;

/// Bytes read from the END of the file on open to recover the chain tail. The
/// tail holds the last entry (its hash + seq); the reader never slurps the whole log.
const OPEN_TAIL_BUDGET: u64 = 256 * 1024;

/// The outcome of a recorded verb invocation.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(tag = "outcome", content = "detail", rename_all = "snake_case")]
pub enum ReceiptOutcome {
    /// The verb ran and succeeded.
    Ok,
    /// The authorizer denied the call.
    Denied,
    /// A mutating verb is ABOUT to be dispatched (written before the side
    /// effect, so a crash after the effect still has an audit trail).
    Intent,
    /// The verb failed (or was unknown); `detail` carries the error class.
    Error(String),
}

impl ReceiptOutcome {
    /// A stable tag folded into the entry hash. Distinct per variant so a
    /// tampered outcome changes the hash.
    fn hash_tag(&self) -> String {
        match self {
            ReceiptOutcome::Ok => "ok".to_string(),
            ReceiptOutcome::Denied => "denied".to_string(),
            ReceiptOutcome::Intent => "intent".to_string(),
            ReceiptOutcome::Error(detail) => format!("error:{detail}"),
        }
    }
}

/// One recorded, hash-chained receipt entry.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Receipt {
    /// Monotonic 0-based sequence number (continuous across rotations).
    pub seq: u64,
    /// Nanoseconds since the Unix epoch (fits u64 until ~2554).
    pub timestamp_ns: u64,
    /// The caller's stable identity.
    pub caller: String,
    /// The verb invoked.
    pub verb: String,
    /// Lowercase-hex SHA-256 of the canonical args JSON.
    pub args_digest: String,
    /// The invocation outcome.
    pub outcome: ReceiptOutcome,
    /// The previous entry's `entry_hash` (or [`GENESIS_HASH`] for seq 0).
    pub prev_hash: String,
    /// This entry's hash over all the fields above.
    pub entry_hash: String,
}

/// A tamper-detection failure returned by [`verify_chain`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ChainError {
    /// Entry `index`'s `prev_hash` does not match the prior entry's hash
    /// (a link was cut, reordered, or an entry inserted/removed).
    BrokenLink {
        index: usize,
        expected_prev: String,
        found_prev: String,
    },
    /// Entry `index`'s stored `entry_hash` does not match a recomputation
    /// from its fields (a field was altered in place).
    Tampered { index: usize },
}

impl std::fmt::Display for ChainError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ChainError::BrokenLink {
                index,
                expected_prev,
                found_prev,
            } => write!(
                f,
                "receipt chain broken at entry {index}: expected prev_hash {expected_prev}, \
                 found {found_prev}"
            ),
            ChainError::Tampered { index } => {
                write!(f, "receipt entry {index} was tampered (hash mismatch)")
            }
        }
    }
}

impl std::error::Error for ChainError {}

/// Lowercase-hex SHA-256 digest of a verb's canonical args JSON.
pub fn args_digest(args: &serde_json::Value) -> String {
    sha256_hex(&canonical_json_bytes(args))
}

/// Compute the entry hash over an entry's fields. Pure and injection-safe: the
/// preimage is canonical JSON of a struct, not a delimited string, so a `|`
/// (or any byte) inside `caller`/`verb` cannot forge a different field split.
pub fn compute_entry_hash(
    prev_hash: &str,
    seq: u64,
    timestamp_ns: u64,
    caller: &str,
    verb: &str,
    args_digest: &str,
    outcome: &ReceiptOutcome,
) -> String {
    let preimage = serde_json::json!({
        "prev_hash": prev_hash,
        "seq": seq,
        "timestamp_ns": timestamp_ns,
        "caller": caller,
        "verb": verb,
        "args_digest": args_digest,
        "outcome": outcome.hash_tag(),
    });
    sha256_hex(&canonical_json_bytes(&preimage))
}

/// Verify a chain of receipts: each `prev_hash` links to the prior entry and
/// each `entry_hash` recomputes from the entry's fields. Returns the first
/// [`ChainError`] found, or `Ok(())` for an intact chain (empty is intact).
///
/// A boundary-truncated log (a valid PREFIX of a longer chain, cut at a line
/// boundary) verifies as that prefix — the returned `Ok(())` covers exactly the
/// entries passed, so a shorter slice is a shorter-but-valid chain, not a
/// tamper. Callers that need the full history across rotations concatenate the
/// rotated files' entries before verifying.
pub fn verify_chain(entries: &[Receipt]) -> Result<(), ChainError> {
    let mut expected_prev = GENESIS_HASH.to_string();
    for (index, r) in entries.iter().enumerate() {
        if r.prev_hash != expected_prev {
            return Err(ChainError::BrokenLink {
                index,
                expected_prev,
                found_prev: r.prev_hash.clone(),
            });
        }
        let recomputed = compute_entry_hash(
            &r.prev_hash,
            r.seq,
            r.timestamp_ns,
            &r.caller,
            &r.verb,
            &r.args_digest,
            &r.outcome,
        );
        if recomputed != r.entry_hash {
            return Err(ChainError::Tampered { index });
        }
        expected_prev = r.entry_hash.clone();
    }
    Ok(())
}

/// Parse every entry from an on-disk receipt log (the AUDIT reader — reads the
/// whole file, one file, for full-chain verification; this is NOT the hot
/// `open` path, which reads a bounded tail).
///
/// Tolerates exactly ONE crash-torn final line (a last line with no trailing
/// newline that fails to parse): it is skipped with a `warn!`. Any OTHER
/// unparseable line is a HARD error — tamper evidence.
pub fn read_all(path: &Path) -> CerudResult<Vec<Receipt>> {
    if !path.exists() {
        return Ok(Vec::new());
    }
    let content = std::fs::read_to_string(path)?;
    let ends_with_newline = content.ends_with('\n');
    let lines: Vec<&str> = content.lines().collect();
    let line_count = lines.len();
    let mut out = Vec::new();
    for (i, line) in lines.iter().enumerate() {
        if line.trim().is_empty() {
            continue;
        }
        match serde_json::from_str::<Receipt>(line) {
            Ok(r) => out.push(r),
            Err(e) => {
                let is_torn_final = i + 1 == line_count && !ends_with_newline;
                if is_torn_final {
                    tracing::warn!(
                        path = %path.display(),
                        "skipping a crash-torn final receipt line during read_all"
                    );
                } else {
                    return Err(CerudError::Receipt(format!(
                        "corrupt receipt log line {} in '{}': {e}",
                        i + 1,
                        path.display()
                    )));
                }
            }
        }
    }
    Ok(out)
}

/// The path of the `i`-th rotated file (`<log>.i`, 1-based).
pub fn rotated_path(base: &Path, i: usize) -> PathBuf {
    let mut os = base.as_os_str().to_owned();
    os.push(format!(".{i}"));
    PathBuf::from(os)
}

/// An open, append-only receipt log with crash-tolerant recovery + rotation.
pub struct ReceiptLog {
    path: PathBuf,
    file: File,
    next_seq: u64,
    prev_hash: String,
    current_bytes: u64,
    max_bytes: u64,
    max_rotations: usize,
    /// Test seam: when set, the next `record_at` fails before writing anything.
    /// Always present (a plain bool), but only SETTABLE via the feature-gated
    /// [`ReceiptLog::fail_next_record_for_test`], so production code can never
    /// arm it (it stays `false`).
    fail_next_record: bool,
    /// Test seam: when set, the next append succeeds its `write_all` then
    /// simulates a `sync_data` failure (to exercise the write→sync rollback).
    /// Same production-inert guarantee as `fail_next_record`.
    fail_next_sync: bool,
}

impl ReceiptLog {
    /// Open (creating if needed) the log at `path` with default rotation limits.
    pub fn open(path: &Path) -> CerudResult<Self> {
        Self::open_with_limits(path, DEFAULT_MAX_LOG_BYTES, DEFAULT_MAX_ROTATIONS)
    }

    /// Open with explicit rotation limits. `max_bytes == 0` disables rotation.
    pub fn open_with_limits(
        path: &Path,
        max_bytes: u64,
        max_rotations: usize,
    ) -> CerudResult<Self> {
        let path = path.to_path_buf();
        // Bounded tail recovery: crash-torn final line is truncated, the chain
        // tail (next seq + prev hash) recovered — never a whole-file slurp.
        // fsync the parent dir ONLY when we just created the file, so the new
        // directory entry is durable across a crash (a reopen of an existing
        // file needs no dir fsync — the entry is already durable).
        let existed = path.exists();
        let (next_seq, prev_hash) = recover_chain_tail(&path)?;
        let file = OpenOptions::new().create(true).append(true).open(&path)?;
        set_owner_only(&path);
        if !existed {
            let _ = sync_parent_dir(&path);
        }
        let current_bytes = file.metadata()?.len();
        Ok(ReceiptLog {
            path,
            file,
            next_seq,
            prev_hash,
            current_bytes,
            max_bytes,
            max_rotations,
            fail_next_record: false,
            fail_next_sync: false,
        })
    }

    /// Test seam: arm the next [`ReceiptLog::record`]/`record_at` to fail with a
    /// [`CerudError::Receipt`] WITHOUT writing anything (models a disk/durability
    /// failure). Used to exercise the server's fail-closed audit path.
    ///
    /// Gated behind `cfg(any(test, feature = "test-seam"))` so it is NOT
    /// exposed in the production AGPL binary. The integration tests enable the
    /// `test-seam` feature via a self dev-dependency.
    #[doc(hidden)]
    #[cfg(any(test, feature = "test-seam"))]
    pub fn fail_next_record_for_test(&mut self) {
        self.fail_next_record = true;
    }

    /// Test seam: arm the next append to fail at the `sync_data` step (AFTER a
    /// successful `write_all`), exercising the write→sync rollback that keeps
    /// on-disk bytes and in-memory `next_seq`/`prev_hash` consistent. Same
    /// production-inert gating as [`ReceiptLog::fail_next_record_for_test`].
    #[doc(hidden)]
    #[cfg(any(test, feature = "test-seam"))]
    pub fn fail_next_sync_for_test(&mut self) {
        self.fail_next_sync = true;
    }

    /// Append a receipt for `(caller, verb, args_digest, outcome)`, stamping
    /// the current wall-clock time. Durable (`sync_data`) before returning.
    pub fn record(
        &mut self,
        caller: &str,
        verb: &str,
        args_digest: &str,
        outcome: ReceiptOutcome,
    ) -> CerudResult<Receipt> {
        self.record_at(caller, verb, args_digest, outcome, now_ns())
    }

    /// Append with an explicit timestamp (deterministic; used by tests).
    pub fn record_at(
        &mut self,
        caller: &str,
        verb: &str,
        args_digest: &str,
        outcome: ReceiptOutcome,
        timestamp_ns: u64,
    ) -> CerudResult<Receipt> {
        if self.fail_next_record {
            self.fail_next_record = false;
            return Err(CerudError::Receipt(
                "injected receipt-write failure (test seam)".to_string(),
            ));
        }
        self.maybe_rotate()?; // may append an anchor first, updating prev/seq
        let entry = self.build_entry(caller, verb, args_digest, outcome, timestamp_ns);
        self.write_entry(&entry)?;
        Ok(entry)
    }

    /// The sequence number the next appended entry will carry.
    pub fn next_seq(&self) -> u64 {
        self.next_seq
    }

    fn build_entry(
        &self,
        caller: &str,
        verb: &str,
        args_digest: &str,
        outcome: ReceiptOutcome,
        timestamp_ns: u64,
    ) -> Receipt {
        let seq = self.next_seq;
        let entry_hash = compute_entry_hash(
            &self.prev_hash,
            seq,
            timestamp_ns,
            caller,
            verb,
            args_digest,
            &outcome,
        );
        Receipt {
            seq,
            timestamp_ns,
            caller: caller.to_string(),
            verb: verb.to_string(),
            args_digest: args_digest.to_string(),
            outcome,
            prev_hash: self.prev_hash.clone(),
            entry_hash,
        }
    }

    fn write_entry(&mut self, entry: &Receipt) -> CerudResult<()> {
        let mut line =
            serde_json::to_string(entry).map_err(|e| CerudError::Receipt(e.to_string()))?;
        line.push('\n');
        let bytes = line.as_bytes();

        // The tracked in-memory length equals the on-disk length (append-only,
        // we control every write). Capture it so we can roll the file back to
        // it if the durability sync fails — keeping bytes-on-disk and
        // next_seq/prev_hash consistent so a retry cleanly reuses this seq.
        let pre_len = self.current_bytes;

        // A partial/failed write leaves the file longer than pre_len; truncate
        // back so the failed append never becomes a phantom (torn) entry.
        if let Err(e) = self.file.write_all(bytes) {
            let _ = self.file.set_len(pre_len);
            let _ = self.file.sync_data();
            return Err(e.into());
        }

        // Test seam: simulate a sync failure AFTER the bytes are written.
        let sync_result = if self.fail_next_sync {
            self.fail_next_sync = false;
            Err(CerudError::Receipt(
                "injected sync_data failure (test seam)".to_string(),
            ))
        } else {
            // sync_data() (NOT flush — File::flush is a no-op) so the entry is
            // durable on disk before the caller's response is acknowledged.
            self.file.sync_data().map_err(CerudError::from)
        };
        if let Err(e) = sync_result {
            // Undo the just-appended bytes so on-disk state matches the
            // un-advanced in-memory state (next_seq/prev_hash unchanged).
            let _ = self.file.set_len(pre_len);
            let _ = self.file.sync_data();
            return Err(e);
        }

        self.prev_hash = entry.entry_hash.clone();
        self.next_seq += 1;
        self.current_bytes += bytes.len() as u64;
        Ok(())
    }

    fn maybe_rotate(&mut self) -> CerudResult<()> {
        if self.max_bytes == 0 || self.current_bytes < self.max_bytes {
            return Ok(());
        }
        self.rotate()
    }

    fn rotate(&mut self) -> CerudResult<()> {
        self.file.sync_data()?;
        // Delete the oldest, shift the rest up, then move current -> .1.
        let oldest = rotated_path(&self.path, self.max_rotations);
        if oldest.exists() {
            std::fs::remove_file(&oldest)?;
        }
        for i in (1..self.max_rotations).rev() {
            let from = rotated_path(&self.path, i);
            if from.exists() {
                std::fs::rename(&from, rotated_path(&self.path, i + 1))?;
            }
        }
        std::fs::rename(&self.path, rotated_path(&self.path, 1))?;
        // Fresh current file; the old handle (now the .1 inode) is dropped.
        self.file = OpenOptions::new()
            .create(true)
            .append(true)
            .open(&self.path)?;
        set_owner_only(&self.path);
        // fsync the parent dir so the renames + the new file's directory entry
        // are durable across a crash (dir entries are otherwise not implied
        // durable by an fsync of the file's data).
        let _ = sync_parent_dir(&self.path);
        self.current_bytes = 0;
        // Anchor: continues the chain across the boundary (tamper-evident) and
        // receipts the rotation itself.
        let anchor_digest = args_digest(&serde_json::json!({
            "rotated_to": rotated_path(&self.path, 1)
                .file_name()
                .map(|s| s.to_string_lossy().into_owned())
                .unwrap_or_default(),
        }));
        let anchor = self.build_entry(
            "cerud",
            ROTATION_VERB,
            &anchor_digest,
            ReceiptOutcome::Ok,
            now_ns(),
        );
        self.write_entry(&anchor)?;
        tracing::info!(
            path = %self.path.display(),
            "rotated receipt log; chain continues via anchor"
        );
        Ok(())
    }
}

/// Recover `(next_seq, prev_hash)` from the tail of an existing log, reading at
/// most [`OPEN_TAIL_BUDGET`] bytes from the end.
///
/// A file NOT ending in a newline has a trailing fragment that is EITHER a
/// genuine crash-torn partial OR a TAMPER — normal operation always terminates
/// each entry with a `sync_data`'d `\n`, so a complete, chain-valid entry whose
/// only defect is a missing newline means its terminator byte was deleted to
/// erase the newest record. The fragment is discriminated: if it parses AND its
/// `entry_hash` recomputes AND its `prev_hash` links to the prior entry, it is a
/// COMPLETE entry → hard error (never silently truncated). Anything that fails
/// to parse/verify is crash-torn → truncated with a loud `warn!`. Any COMPLETE
/// (newline-terminated) line that fails to parse is likewise a hard error
/// (tamper evidence). A fresh/missing file starts at genesis.
fn recover_chain_tail(path: &Path) -> CerudResult<(u64, String)> {
    if !path.exists() {
        return Ok((0, GENESIS_HASH.to_string()));
    }
    // read+write so a torn tail can be truncated in place.
    let mut file = OpenOptions::new().read(true).write(true).open(path)?;
    let len = file.metadata()?.len();
    if len == 0 {
        return Ok((0, GENESIS_HASH.to_string()));
    }
    let start = len.saturating_sub(OPEN_TAIL_BUDGET);
    file.seek(SeekFrom::Start(start))?;
    let mut buf = Vec::new();
    file.read_to_end(&mut buf)?;

    // Split the window into a COMPLETE region (up to and including the last
    // newline) and a trailing FRAGMENT (bytes after it — empty if the file ends
    // with a newline).
    let last_nl = buf.iter().rposition(|&b| b == b'\n');
    let complete_end = last_nl.map(|p| p + 1).unwrap_or(0);
    let fragment_present = complete_end < buf.len();

    // A window with no line boundary that we seeked INTO (start > 0) cannot be
    // recovered from the tail alone.
    if last_nl.is_none() && start > 0 {
        return Err(CerudError::Receipt(format!(
            "receipt log '{}' tail window ({OPEN_TAIL_BUDGET} bytes) contains no line \
             boundary; cannot safely recover the chain tail",
            path.display()
        )));
    }

    // Skip the partial LEADING fragment if we seeked into the middle of a line.
    let scan_from = if start == 0 {
        0
    } else {
        buf[..complete_end]
            .iter()
            .position(|&b| b == b'\n')
            .map(|p| p + 1)
            .unwrap_or(complete_end)
    };

    // Parse every COMPLETE line; any parse failure here is a hard error
    // (tamper/corruption). `prior` is the last complete entry.
    let mut prior: Option<Receipt> = None;
    for line in buf[scan_from..complete_end].split(|&b| b == b'\n') {
        if line.is_empty() {
            continue;
        }
        let text = std::str::from_utf8(line).map_err(|e| {
            CerudError::Receipt(format!(
                "receipt log '{}' has non-UTF8 content near the tail: {e}",
                path.display()
            ))
        })?;
        if text.trim().is_empty() {
            continue;
        }
        let r: Receipt = serde_json::from_str(text).map_err(|e| {
            CerudError::Receipt(format!(
                "corrupt receipt log line in '{}': {e}",
                path.display()
            ))
        })?;
        prior = Some(r);
    }

    // Discriminate a trailing fragment: sealed entry (missing newline) = tamper;
    // otherwise crash-torn = truncate + warn.
    if fragment_present {
        let fragment = &buf[complete_end..];
        let prior_hash = prior
            .as_ref()
            .map(|r| r.entry_hash.as_str())
            .unwrap_or(GENESIS_HASH);
        if fragment_is_sealed_entry(fragment, prior_hash) {
            return Err(CerudError::Receipt(format!(
                "receipt log '{}': the final entry parses AND chain-verifies but is missing its \
                 terminating newline — its terminator byte was deleted (tamper). A complete audit \
                 record must not be silently dropped; refusing to recover.",
                path.display()
            )));
        }
        // Genuine crash-torn partial: truncate it away.
        let keep = start + complete_end as u64;
        file.set_len(keep)?;
        file.sync_data()?;
        if keep == 0 {
            tracing::warn!(
                path = %path.display(),
                "receipt log had no complete entry; reset to genesis"
            );
        } else {
            tracing::warn!(
                path = %path.display(),
                dropped_bytes = len - keep,
                "recovered a crash-torn final receipt line by truncating to the last intact entry"
            );
        }
    }

    match prior {
        Some(r) => Ok((r.seq + 1, r.entry_hash)),
        None => Ok((0, GENESIS_HASH.to_string())),
    }
}

/// Whether a trailing fragment is actually a COMPLETE, chain-valid entry whose
/// terminating newline was removed (a tamper), rather than a crash-torn partial.
/// True iff it parses as a `Receipt`, its `entry_hash` recomputes from its own
/// fields, AND its `prev_hash` links to `prior_hash`.
fn fragment_is_sealed_entry(fragment: &[u8], prior_hash: &str) -> bool {
    let Ok(text) = std::str::from_utf8(fragment) else {
        return false;
    };
    let text = text.trim();
    if text.is_empty() {
        return false;
    }
    let Ok(r) = serde_json::from_str::<Receipt>(text) else {
        return false;
    };
    let recomputed = compute_entry_hash(
        &r.prev_hash,
        r.seq,
        r.timestamp_ns,
        &r.caller,
        &r.verb,
        &r.args_digest,
        &r.outcome,
    );
    recomputed == r.entry_hash && r.prev_hash == prior_hash
}

/// Best-effort tighten a file to owner-only (0o600) on Unix; no-op elsewhere.
/// The receipt log is a sensitive audit artifact.
#[cfg(unix)]
fn set_owner_only(path: &Path) {
    use std::os::unix::fs::PermissionsExt;
    if let Err(e) = std::fs::set_permissions(path, std::fs::Permissions::from_mode(0o600)) {
        tracing::warn!(
            path = %path.display(),
            error = %e,
            "could not set receipt-log permissions to 0o600"
        );
    }
}

#[cfg(not(unix))]
fn set_owner_only(_path: &Path) {}

/// Best-effort fsync of a file's PARENT directory, so the directory entry
/// (file creation, or a rename during rotation) is durable across a crash.
/// A data `sync_data()` on the file does NOT imply the directory entry is
/// durable. Called only on create + rotation — not per append. Returns `Ok`
/// (a fsync failure warns, never fails the append).
pub fn sync_parent_dir(path: &Path) -> CerudResult<()> {
    let parent = path.parent().unwrap_or_else(|| Path::new("."));
    let dir = if parent.as_os_str().is_empty() {
        Path::new(".")
    } else {
        parent
    };
    match File::open(dir) {
        Ok(f) => {
            if let Err(e) = f.sync_all() {
                tracing::warn!(dir = %dir.display(), error = %e, "could not fsync receipt-log parent dir");
            }
        }
        Err(e) => {
            tracing::warn!(dir = %dir.display(), error = %e, "could not open receipt-log parent dir to fsync");
        }
    }
    Ok(())
}

/// Nanoseconds since the Unix epoch (saturating; a pre-epoch clock reads 0).
fn now_ns() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_nanos() as u64)
        .unwrap_or(0)
}
