// SPDX-License-Identifier: AGPL-3.0-only
//! Checking a receipt log's hash chain, rotated files included.
//!
//! The log rotates by size: the full file becomes `<log>.1`, older ones shift
//! to `.2`, `.3`, ..., and the oldest beyond the retention count is deleted.
//! Each file after a rotation begins with a rotation anchor whose `prev_hash`
//! links to the previous file. [`verify_log`] reads every file still present,
//! oldest first, and checks that each entry links to the one before it and
//! that its hash recomputes from its fields.
//!
//! The chain must begin either at the genesis hash or at a rotation anchor
//! (older files deleted by retention). A first entry that is neither means
//! lines were cut from the head, and a missing file between rotated files
//! means one was deleted; both are reported as broken.
//!
//! What a valid chain does NOT show: that no entries were removed from the
//! END of the newest file. A log cut at a line boundary is a shorter valid
//! chain. Compare [`AuditReport::last_hash`] with a copy kept elsewhere to
//! detect that.

use std::path::{Path, PathBuf};

use cerud::receipt::{
    compute_entry_hash, read_all, rotated_path, Receipt, GENESIS_HASH, ROTATION_VERB,
};

/// How far past a missing rotated file to look for a later one (a gap).
const ROTATION_SCAN: usize = 64;

/// What [`verify_log`] checked.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct AuditReport {
    /// The files read, oldest first.
    pub files: Vec<PathBuf>,
    /// How many entries were checked.
    pub entries: usize,
    /// The `seq` of the first entry checked.
    pub first_seq: u64,
    /// Whether the first entry links to the genesis hash.
    pub from_genesis: bool,
    /// The newest entry's hash.
    pub last_hash: String,
}

/// Check the hash chain of the receipt log at `path` and its rotated files.
///
/// # Errors
///
/// A description of the first problem: a missing or empty log, an unreadable
/// file, a broken link or an entry whose hash does not recompute.
pub fn verify_log(path: &Path) -> Result<AuditReport, String> {
    if !path.exists() {
        return Err(format!("no receipt log at {}", path.display()));
    }
    let mut files: Vec<PathBuf> = (1..)
        .map(|i| rotated_path(path, i))
        .take_while(|p| p.exists())
        .collect();
    let next = files.len() + 1;
    if let Some(later) = (next + 1..=next + ROTATION_SCAN)
        .map(|i| rotated_path(path, i))
        .find(|p| p.exists())
    {
        return Err(format!(
            "rotated file {} is missing but {} exists",
            rotated_path(path, next).display(),
            later.display()
        ));
    }
    files.reverse();
    files.push(path.to_path_buf());
    let mut entries: Vec<Receipt> = Vec::new();
    for file in &files {
        let read = read_all(file).map_err(|e| format!("cannot read {}: {e}", file.display()))?;
        entries.extend(read);
    }
    let Some(first) = entries.first() else {
        return Err(format!("the receipt log at {} is empty", path.display()));
    };
    let from_genesis = first.prev_hash == GENESIS_HASH;
    if !from_genesis && first.verb != ROTATION_VERB {
        return Err(format!(
            "the oldest entry (seq {}) neither starts the chain nor follows a rotation: \
             lines were removed from the head",
            first.seq
        ));
    }
    let first_seq = first.seq;
    let mut expected_prev = first.prev_hash.clone();
    for (index, r) in entries.iter().enumerate() {
        if r.prev_hash != expected_prev {
            return Err(format!(
                "broken link at entry {index} (seq {}): prev_hash {} does not match the previous entry's hash {expected_prev}",
                r.seq, r.prev_hash
            ));
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
            return Err(format!(
                "entry {index} (seq {}) was altered: its hash does not recompute",
                r.seq
            ));
        }
        expected_prev.clone_from(&r.entry_hash);
    }
    Ok(AuditReport {
        files,
        entries: entries.len(),
        first_seq,
        from_genesis,
        last_hash: expected_prev,
    })
}
