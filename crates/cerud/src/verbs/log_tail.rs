// SPDX-License-Identifier: AGPL-3.0-only
//! The `log-tail` verb: return the tail of a named log file under an
//! allow-listed root. No arbitrary path reads — the path is hardened against
//! traversal both lexically (reject `..`/absolute) and, at read time, against
//! symlink escapes.
//!
//! Args: `{ "name": "<relative log path>", "lines": <optional u64> }`.
//! Result: `{ "name", "path", "lines": [...], "returned", "truncated",
//! "bytes_scanned" }`.
//!
//! The read is BOUNDED — only the last [`TAIL_BYTE_BUDGET`] bytes are read from
//! disk, so a huge log never loads whole into memory.
//!
//! (Genuine line-by-line *streaming*/follow is not implemented; this returns the
//! last N lines in one response over the request/response protocol.)

use std::fs::File;
use std::io::{Read, Seek, SeekFrom};
use std::path::{Component, Path, PathBuf};

use serde::Deserialize;

use crate::error::{CerudError, CerudResult};
use crate::verbs::VerbHandler;

/// Default number of tail lines when the request omits `lines`.
pub const DEFAULT_TAIL_LINES: usize = 200;

/// Hard cap on returned lines (a request asking for more is clamped).
pub const MAX_TAIL_LINES: usize = 10_000;

/// Byte budget for a tail read: the verb seeks to `len - TAIL_BYTE_BUDGET` and
/// reads only that suffix, so a multi-gigabyte log never loads into memory. A
/// 4 MiB backstop comfortably covers [`MAX_TAIL_LINES`] lines of a few hundred
/// bytes each; a request whose tail lines exceed the budget gets the last
/// budget-worth of lines with `truncated = true`.
pub const TAIL_BYTE_BUDGET: u64 = 4 * 1024 * 1024;

/// The `log-tail` verb, bound to one allow-listed root directory.
#[derive(Debug, Clone)]
pub struct LogTailVerb {
    allowed_root: PathBuf,
}

impl LogTailVerb {
    /// A verb serving log files under `allowed_root` (and nowhere else).
    pub fn new(allowed_root: impl Into<PathBuf>) -> Self {
        LogTailVerb {
            allowed_root: allowed_root.into(),
        }
    }
}

#[derive(Debug, Deserialize)]
struct LogTailArgs {
    name: String,
    #[serde(default)]
    lines: Option<u64>,
}

impl VerbHandler for LogTailVerb {
    fn name(&self) -> &'static str {
        "log-tail"
    }

    fn is_mutating(&self) -> bool {
        false // read-only tail of an allow-listed log
    }

    fn execute(&self, args: &serde_json::Value) -> CerudResult<serde_json::Value> {
        let args: LogTailArgs = serde_json::from_value(args.clone())
            .map_err(|e| CerudError::Verb(format!("invalid log-tail args: {e}")))?;

        let requested = args
            .lines
            .map(|n| (n as usize).min(MAX_TAIL_LINES))
            .unwrap_or(DEFAULT_TAIL_LINES);

        // 1. Lexical traversal guard (no fs access — cannot be tricked).
        let resolved = resolve_under_root(&self.allowed_root, &args.name)?;
        // 2. Symlink-escape guard, canonicalizing BEFORE reading so a symlink
        //    escaping the root is refused without ever reading its target.
        let canonical = canonical_within_root(&self.allowed_root, &resolved, &args.name)?;
        // 3. BOUNDED tail read — only the last TAIL_BYTE_BUDGET bytes are read,
        //    so a huge log never loads whole into memory.
        let outcome = tail_file(&canonical, requested, TAIL_BYTE_BUDGET)?;

        Ok(serde_json::json!({
            "name": args.name,
            "path": canonical.display().to_string(),
            "returned": outcome.lines.len(),
            "truncated": outcome.truncated,
            "bytes_scanned": outcome.bytes_scanned,
            "lines": outcome.lines,
        }))
    }
}

/// Lexically resolve `name` under `root`, refusing anything that escapes.
///
/// Pure (no filesystem access) and oracle-testable: rejects an absolute path,
/// an empty name, and any `..`/root/prefix component. A bare `.` is harmless
/// and allowed. This is the first, un-trickable line of defense; the symlink
/// guard ([`canonical_within_root`]) is the second.
pub fn resolve_under_root(root: &Path, name: &str) -> CerudResult<PathBuf> {
    let traversal = || CerudError::PathTraversal {
        requested: name.to_string(),
        root: root.display().to_string(),
    };
    if name.is_empty() {
        return Err(traversal());
    }
    let rel = Path::new(name);
    if rel.is_absolute() {
        return Err(traversal());
    }
    for component in rel.components() {
        match component {
            Component::Normal(_) | Component::CurDir => {}
            // ParentDir (`..`), RootDir (`/`), Prefix (`C:\`) all escape.
            _ => return Err(traversal()),
        }
    }
    Ok(root.join(rel))
}

/// Canonicalize `resolved` and confirm it stays under `root` (defeats a
/// symlink under the allow-listed root pointing outside it), returning the
/// canonical path. A missing file yields a clean [`CerudError::Verb`] rather
/// than leaking the raw I/O error.
pub fn canonical_within_root(root: &Path, resolved: &Path, name: &str) -> CerudResult<PathBuf> {
    let canon_root = root.canonicalize().map_err(|e| {
        CerudError::Verb(format!(
            "log-tail allow-list root '{}' is not accessible: {e}",
            root.display()
        ))
    })?;
    let canon_file = match resolved.canonicalize() {
        Ok(p) => p,
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => {
            return Err(CerudError::Verb(format!("no such log '{name}'")));
        }
        Err(e) => return Err(e.into()),
    };
    if !canon_file.starts_with(&canon_root) {
        return Err(CerudError::PathTraversal {
            requested: name.to_string(),
            root: root.display().to_string(),
        });
    }
    Ok(canon_file)
}

/// Return the last `n` lines of `content` and whether more lines existed.
///
/// Pure and oracle-testable. A trailing newline does not count as an extra
/// empty line (`"a\nb\n"` has two lines).
pub fn tail_lines(content: &str, n: usize) -> (Vec<String>, bool) {
    let all: Vec<&str> = content.lines().collect();
    let total = all.len();
    if n >= total {
        (all.into_iter().map(|s| s.to_string()).collect(), false)
    } else {
        let start = total - n;
        (all[start..].iter().map(|s| s.to_string()).collect(), true)
    }
}

/// The result of a bounded tail read.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TailOutcome {
    /// The tail lines (up to `lines_wanted`).
    pub lines: Vec<String>,
    /// Whether more lines existed than were returned (byte-budget-clipped OR
    /// line-count-clipped).
    pub truncated: bool,
    /// How many bytes were actually read from disk — always `<= byte_budget`.
    /// Exposed so callers/tests can prove the read stayed bounded (memory does
    /// not scale with file size).
    pub bytes_scanned: u64,
}

/// Read the tail of a file, bounded to the last `byte_budget` bytes.
///
/// Seeks to `max(0, len - byte_budget)`, reads only that suffix (so a
/// multi-gigabyte log never loads whole into memory), lossy-decodes it (a
/// non-UTF8 log still tails, with replacement chars), drops any partial leading
/// line created by the seek, and returns the last `lines_wanted` lines. The
/// `truncated` flag is true if the seek clipped the head OR more lines existed
/// within the window than were returned.
pub fn tail_file(path: &Path, lines_wanted: usize, byte_budget: u64) -> CerudResult<TailOutcome> {
    let mut file = File::open(path)
        .map_err(|e| CerudError::Verb(format!("cannot open log '{}': {e}", path.display())))?;
    let len = file.metadata()?.len();
    let start = len.saturating_sub(byte_budget);
    file.seek(SeekFrom::Start(start))?;
    let mut buf = Vec::new();
    file.read_to_end(&mut buf)?;
    let bytes_scanned = buf.len() as u64;

    // If we seeked into the middle of a line, drop the partial leading fragment.
    let head_clipped = start > 0;
    let slice: &[u8] = if head_clipped {
        match buf.iter().position(|&b| b == b'\n') {
            Some(p) => &buf[p + 1..],
            None => &buf[..], // a single line longer than the budget: return it lossy
        }
    } else {
        &buf[..]
    };

    let text = String::from_utf8_lossy(slice);
    let (lines, line_clipped) = tail_lines(&text, lines_wanted);
    Ok(TailOutcome {
        lines,
        truncated: head_clipped || line_clipped,
        bytes_scanned,
    })
}
