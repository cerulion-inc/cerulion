// SPDX-License-Identifier: AGPL-3.0-only
//! B2: the `runs` verb's RUN-DIRECTORY half — reading a live run's two
//! self-describing artifacts off disk and folding them, with the run registry's
//! own record, into the [`RunEntry`] values the verb serves.
//!
//! # Why this is its own module
//!
//! The `runs` verb's serve side is two unrelated I/O planes stitched together: a
//! windowed SHM gather (`run_registry`) and a filesystem read (here). Everything
//! else in [`network`](super::network) is zenoh; nothing else in it touches the
//! filesystem at all. Keeping the directory half separate is what lets it be
//! oracle-tested against a `tempdir` with no session, no node and no gateway —
//! which is the whole failure surface that matters, since a run directory is the
//! one input to this verb that an operator, a full disk, or a hostile process can
//! shape.
//!
//! # The rule the whole module exists to enforce
//!
//! **A run whose artifacts cannot be read is SKIPPED, never served empty.**
//! The effective `graph.yaml` is served VERBATIM as the run's
//! authoritative self-description, and the desk re-parses it with the same
//! `GraphConfig` parser the robot used — so an entry carrying an empty document
//! renders as *a graph with no nodes*, which is a confident lie about a running
//! robot rather than an absence of information. [`RunEntry::new`] refuses the
//! empty shapes for exactly this reason; this module's job is to make the skip
//! CARRY A NAMED REASON so an operator can tell "the run directory was deleted"
//! from "the disk is full" from "somebody put a 4 GiB file there".
//!
//! # Bounded by construction
//!
//! Every read is capped at [`MAX_RUN_ARTIFACT_LEN`] + 1 bytes *before* any
//! allocation proportional to the file, so a pathological run directory cannot
//! bloat a serve — the cap is checked twice (an `fstat`, then the read itself),
//! because a file can grow between the two and only the second bound is a
//! guarantee.
//!
//! Non-regular files are refused without ever BLOCKING on them: `open(fifo,
//! O_RDONLY)` waits for a writer, so a FIFO at `graph.yaml` would wedge the
//! shared zenoh callback thread — every verb on that surface, not just this one
//! — rather than skip one run. On Unix the open is `O_NONBLOCK | O_NOFOLLOW` and
//! the type is settled by an `fstat` on the resulting descriptor; elsewhere a
//! non-following pre-stat refuses it before the open happens at all.
//!
//! # The path is walked, never re-resolved (Unix)
//!
//! A run directory is bounded by canonicalisation + prefix containment, and then
//! opened by WALKING from the runs root with one `openat(.., O_NOFOLLOW)` per
//! component — so the bytes read come from the directory the containment verdict
//! was about, not from whatever that path names a moment later. See
//! `open_artifact_within`.

use std::fs::File;
use std::io::Read;
use std::path::{Path, PathBuf};

use super::cerulion_q::{format_run_id, RunEntry, UndescribableRun, MAX_RUN_ARTIFACT_LEN};
use super::run_registry::{run_dir_root, RunRecord};

/// The run directory's effective-graph artifact.
pub const GRAPH_YAML_FILE: &str = "graph.yaml";

/// The run directory's manifest artifact.
pub const RUN_JSON_FILE: &str = "run.json";

/// Why ONE run-directory artifact could not be served. Each variant is a distinct
/// operator-actionable condition — the reason that rides the serve side's skip
/// log, so "this run is missing from the answer" is never unexplained.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum RunArtifactError {
    /// The file could not be opened or read (missing, permission denied, I/O
    /// error) — carries the OS error text.
    Unreadable {
        /// Which artifact (`graph.yaml` / `run.json`).
        file: &'static str,
        /// The underlying reason, verbatim.
        reason: String,
    },
    /// The path exists but is not a REGULAR file (a directory, a FIFO, a socket).
    /// Refused BEFORE any read, because reading a FIFO would block the serving
    /// thread indefinitely — a wedge, not a slow answer.
    NotAFile {
        /// Which artifact.
        file: &'static str,
    },
    /// The file is longer than [`MAX_RUN_ARTIFACT_LEN`]. `len` is the size the
    /// filesystem reported when it could be read, else the capped read's own
    /// bound — either way a real number this serve refused, never a guess.
    TooLong {
        /// Which artifact.
        file: &'static str,
        /// Observed length in bytes (at least `MAX_RUN_ARTIFACT_LEN + 1`).
        len: u64,
    },
    /// The bytes are not valid UTF-8 — the wire carries TEXT (a YAML document and
    /// a JSON document), so a binary blob is refused rather than lossily
    /// transcoded into something the desk would try to parse.
    NotUtf8 {
        /// Which artifact.
        file: &'static str,
    },
}

impl std::fmt::Display for RunArtifactError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            RunArtifactError::Unreadable { file, reason } => {
                write!(f, "{file} could not be read ({reason})")
            }
            RunArtifactError::NotAFile { file } => {
                write!(f, "{file} is not a regular file")
            }
            RunArtifactError::TooLong { file, len } => write!(
                f,
                "{file} is {len} bytes, above the {MAX_RUN_ARTIFACT_LEN}-byte served cap"
            ),
            RunArtifactError::NotUtf8 { file } => write!(f, "{file} is not valid UTF-8"),
        }
    }
}

/// Why a gathered `run_dir` was refused before anything under it was opened.
///
/// Deliberately carries NO PATH: the reason travels to the DESK on the wire (as
/// an [`UndescribableRun`] reason), the path is a filesystem location on the
/// robot, and here it is a location an attacker CHOSE — so echoing it would let a
/// local publisher use an authorized remote `runs` GET as a channel for arbitrary
/// text of its own composition. The path is logged LOCALLY beside every skip,
/// where the operator who can act on it is reading.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RunDirRefusal {
    /// Neither `CERULION_HOME` nor a home directory resolves, so no run
    /// directory is legitimately reachable. Refuses everything rather than
    /// falling back to reading anything.
    NoRunRoot,
    /// The path does not resolve (missing, a broken symlink, a permission wall
    /// on a parent) — indistinguishable from a run that has just exited, which
    /// is why it is a skip and not an alarm.
    Unresolvable,
    /// The path RESOLVES, and resolves OUTSIDE the canonical runs root. This is
    /// the traversal refusal: after canonicalisation a symlink escaping the root
    /// lands here exactly as a literal `/etc` would.
    OutsideRunRoot,
}

impl std::fmt::Display for RunDirRefusal {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            RunDirRefusal::NoRunRoot => write!(
                f,
                "no run directory root resolves on this machine (neither CERULION_HOME nor a \
                 home directory), so no run directory can be read"
            ),
            RunDirRefusal::Unresolvable => write!(
                f,
                "the run directory does not resolve (it may have exited, or the path is not \
                 readable)"
            ),
            RunDirRefusal::OutsideRunRoot => write!(
                f,
                "the run directory resolves outside the canonical runs root and is refused"
            ),
        }
    }
}

/// Resolve a gathered `run_dir` and REFUSE anything that is not inside `root`.
///
/// # Why this exists at all
///
/// `RunRecord::run_dir` is a PATH supplied by whatever published the registry
/// record. A run publishes its own, but the registry is a machine-wide SHM
/// service and its writer is any local process: a malicious or merely broken one
/// can point `run_dir` anywhere the SERVING process can read, and an AUTHORIZED
/// remote `runs` GET then returns those bytes verbatim as a run's `graph.yaml`.
/// That turns the verb into an arbitrary-file-read channel — the machine's own
/// access control does not help, because the gateway is the one reading.
///
/// # Both sides are canonicalised, and that is the point
///
/// Comparing the RAW path against the root catches only a literal `/etc/…`; the
/// interesting shape is a directory INSIDE the root that is a symlink pointing
/// out of it, which a textual prefix test admits. `canonicalize` resolves every
/// symlink and every `..` on BOTH sides, so containment is decided on where the
/// path REALLY lands. `Path::starts_with` compares whole COMPONENTS, so a sibling
/// named `runs-evil` is not a prefix of `runs` — no string-prefix hazard.
///
/// # What it does NOT close
///
/// A directory component swapped between this resolution and the subsequent open
/// (the classic TOCTOU). It COMPOSES with the file-level guards rather than
/// replacing them — the artifact open is `O_NOFOLLOW` + non-blocking with a
/// post-open `fstat` — so what remains needs write access INSIDE a `0700`
/// owner-only tree, and closing it properly means `openat` walking every
/// component with `O_NOFOLLOW`, which is a different change.
fn resolve_contained_run_dir(run_dir: &str, root: &Path) -> Result<PathBuf, RunDirRefusal> {
    let resolved = std::fs::canonicalize(run_dir).map_err(|_| RunDirRefusal::Unresolvable)?;
    if !resolved.starts_with(root) {
        return Err(RunDirRefusal::OutsideRunRoot);
    }
    Ok(resolved)
}

/// B2: open `file` under `run_dir` by WALKING from
/// `root`, one `openat` per component, so no path is ever re-resolved.
///
/// # The window this closes
///
/// [`resolve_contained_run_dir`] canonicalises and bounds a path; the read then
/// re-resolved that path from its TEXT, so a component swapped in between sent
/// the open somewhere else entirely and the containment verdict described a
/// directory that was no longer there. Every open below is relative to a
/// descriptor ALREADY HELD — a descriptor names an inode, not a name — so there
/// is no second resolution to race. `O_NOFOLLOW` on each step means a component
/// REPLACED by a symlink is refused (`ELOOP`) rather than followed.
///
/// # Threat model, stated at its real strength
///
/// The swap needs write access INSIDE the runs tree: each run directory is
/// created `0700` and its artifacts `0600` (`cerulion_cli_engine::run_dir`), and
/// the `runs/` root is owner-write in either of its two shapes — best-effort
/// tightened to `0700`, or left `0755` on a pre-existing tree (that wider mode
/// exposes the directory LISTING, never write). So the party who could win this
/// race is the OWNER, who can already read anything the gateway can. This closes
/// a privilege-ESCALATION-free window, and it is done because the cost is
/// bounded — not because the residual was exploitable across users.
///
/// # Errors
///
/// A component that vanished, stopped being a directory, or became a symlink is
/// an ordinary [`RunArtifactError::Unreadable`] carrying the OS error text —
/// which distinguishes the shapes for an operator (`No such file or directory`
/// for a run that exited, `Too many levels of symbolic links` for a swap) while
/// carrying NO PATH onto the wire.
#[cfg(unix)]
fn open_artifact_within(
    root: &Path,
    run_dir: &Path,
    file: &'static str,
) -> Result<File, RunArtifactError> {
    use std::ffi::CString;
    use std::os::fd::{FromRawFd, OwnedFd};
    use std::os::unix::ffi::OsStrExt;

    // hot-path-alloc-ok (this fn): cold — one walk per served run on a
    // control-plane GET, never per frame.
    let unreadable = |e: std::io::Error| RunArtifactError::Unreadable {
        file,
        reason: e.to_string(),
    };
    let cstr = |bytes: &[u8]| {
        CString::new(bytes).map_err(|_| RunArtifactError::Unreadable {
            file,
            reason: "path component contains an interior NUL".to_string(),
        })
    };

    // The ROOT is opened by full path — it is this machine's own derived
    // location and is already canonical, so its final component cannot be a
    // symlink; `O_NOFOLLOW` is kept so a root REPLACED by one is refused too.
    let root_c = cstr(root.as_os_str().as_bytes())?;
    // SAFETY: `root_c` is a valid NUL-terminated path; `open` returns a fd or -1.
    let fd = unsafe {
        libc::open(
            root_c.as_ptr(),
            libc::O_RDONLY | libc::O_DIRECTORY | libc::O_NOFOLLOW | libc::O_CLOEXEC,
        )
    };
    if fd < 0 {
        return Err(unreadable(std::io::Error::last_os_error()));
    }
    // SAFETY: `fd` is a fresh, owned, valid descriptor.
    let mut dir = unsafe { OwnedFd::from_raw_fd(fd) };

    // Walk the run directory's components RELATIVE to the root. Both paths are
    // canonical, so the strip cannot fail — but a failure here would mean the
    // containment verdict and this walk disagree, which must refuse rather than
    // fall back to a full-path open.
    let relative = run_dir
        .strip_prefix(root)
        .map_err(|_| RunArtifactError::Unreadable {
            file,
            reason: "run directory is not under the runs root".to_string(),
        })?;
    for component in relative.components() {
        let std::path::Component::Normal(name) = component else {
            // `canonicalize` leaves only normal components; anything else means
            // the path was not the canonical one this walk assumes.
            return Err(RunArtifactError::Unreadable {
                file,
                reason: "run directory path is not in canonical form".to_string(),
            });
        };
        let name_c = cstr(name.as_bytes())?;
        // SAFETY: `dir` is a live directory descriptor and `name_c` a valid
        // NUL-terminated component; `openat` returns a fd or -1.
        let next = unsafe {
            libc::openat(
                std::os::fd::AsRawFd::as_raw_fd(&dir),
                name_c.as_ptr(),
                libc::O_RDONLY | libc::O_DIRECTORY | libc::O_NOFOLLOW | libc::O_CLOEXEC,
            )
        };
        if next < 0 {
            return Err(unreadable(std::io::Error::last_os_error()));
        }
        // SAFETY: `next` is a fresh, owned, valid descriptor. The previous `dir`
        // is dropped (closed) by the assignment.
        dir = unsafe { OwnedFd::from_raw_fd(next) };
    }

    // The ARTIFACT itself: no follow (a symlinked artifact is refused) and
    // non-blocking (a FIFO must not wedge the serving thread — a review
    // fix, now enforced at this final `openat`).
    let file_c = cstr(file.as_bytes())?;
    // SAFETY: as above; `dir` is the run directory's own descriptor.
    let opened = unsafe {
        libc::openat(
            std::os::fd::AsRawFd::as_raw_fd(&dir),
            file_c.as_ptr(),
            libc::O_RDONLY | libc::O_NOFOLLOW | libc::O_NONBLOCK | libc::O_CLOEXEC,
        )
    };
    if opened < 0 {
        let err = std::io::Error::last_os_error();
        // `O_NOFOLLOW` refusing the FINAL component has exactly ONE meaning: the
        // artifact is a symlink. Reported as the classification an operator can
        // act on rather than as an errno — `graph.yaml is not a regular file`
        // says what is wrong with the run directory; `Too many levels of
        // symbolic links` reads like a filesystem fault. (POSIX leaves the errno
        // to the implementation: `ELOOP` on Linux and macOS, `EMLINK` on some
        // BSDs.)
        let raw = err.raw_os_error();
        if raw == Some(libc::ELOOP) || raw == Some(libc::EMLINK) {
            return Err(RunArtifactError::NotAFile { file });
        }
        return Err(unreadable(err));
    }
    // SAFETY: `opened` is a fresh, owned, valid descriptor.
    Ok(unsafe { File::from_raw_fd(opened) })
}

/// NON-UNIX ONLY: open ONE run-directory artifact by PATH, refusing anything
/// that is not a REGULAR file.
///
/// On Unix this is superseded by [`open_artifact_within`]'s descriptor walk;
/// here the path is re-resolved by the kernel on every call, so the TOCTOU
/// window [`resolve_contained_run_dir`] documents REMAINS on this platform. It is
/// left stated rather than closed: `openat`-per-component has no portable
/// equivalent, the shipping robot is Unix, and the ordinary refusals below are
/// unchanged.
///
/// The pre-stat is the ordinary refusal and the reason this function exists at
/// all: `File::open` on a FIFO BLOCKS, so a check that runs after the open has
/// already lost. It uses `symlink_metadata`, which does NOT follow, so a
/// SYMLINKED artifact is refused as `NotAFile`.
#[cfg(not(unix))]
fn open_artifact_by_path(path: &Path, file: &'static str) -> Result<File, RunArtifactError> {
    let declared = std::fs::symlink_metadata(path).map_err(|e| RunArtifactError::Unreadable {
        file,
        // hot-path-alloc-ok: cold — one per skipped run on a control-plane GET.
        reason: e.to_string(),
    })?;
    if !declared.is_file() {
        return Err(RunArtifactError::NotAFile { file });
    }
    File::open(path).map_err(|e| RunArtifactError::Unreadable {
        file,
        // hot-path-alloc-ok: cold — control-plane skip diagnostic.
        reason: e.to_string(),
    })
}

/// The metadata of an opened artifact, refusing anything that is not a REGULAR
/// file.
///
/// The `fstat` is on the DESCRIPTOR, never a second `stat` of the path — on Unix
/// that descriptor came from a walk in which nothing was resolved by name, so
/// this is asking about the exact inode the read will consume; on other
/// platforms it is what makes a path re-resolved between check and open still
/// refuse a non-regular target.
///
/// Every length decision downstream reads THIS metadata, for the same reason.
fn regular_file_metadata(
    handle: &File,
    file: &'static str,
) -> Result<std::fs::Metadata, RunArtifactError> {
    let opened = handle
        .metadata()
        .map_err(|e| RunArtifactError::Unreadable {
            file,
            // hot-path-alloc-ok: cold — control-plane skip diagnostic.
            reason: e.to_string(),
        })?;
    if !opened.is_file() {
        return Err(RunArtifactError::NotAFile { file });
    }
    Ok(opened)
}

/// Read ONE run-directory artifact, bounded.
///
/// The cap is applied TWICE on purpose: the handle's `fstat` gives an exact
/// length for the diagnostic, and the capped read is the actual guarantee (a
/// file that grows between the two still cannot allocate past the bound).
fn read_artifact(
    root: &Path,
    run_dir: &Path,
    file: &'static str,
) -> Result<String, RunArtifactError> {
    #[cfg(unix)]
    let handle = open_artifact_within(root, run_dir, file)?;
    #[cfg(not(unix))]
    let handle = {
        let _ = root;
        open_artifact_by_path(&run_dir.join(file), file)?
    };
    let meta = regular_file_metadata(&handle, file)?;
    if meta.len() > MAX_RUN_ARTIFACT_LEN as u64 {
        return Err(RunArtifactError::TooLong {
            file,
            len: meta.len(),
        });
    }
    // Read at most CAP + 1 so "is it over the cap?" is answerable WITHOUT ever
    // holding more than that. Bytes rather than `read_to_string`, because a read
    // truncated at the cap can split a multi-byte character and would then be
    // reported as invalid UTF-8 — the wrong reason for a file whose real problem
    // is its size.
    let mut bytes = Vec::new();
    handle
        .take(MAX_RUN_ARTIFACT_LEN as u64 + 1)
        .read_to_end(&mut bytes)
        .map_err(|e| RunArtifactError::Unreadable {
            file,
            // hot-path-alloc-ok: cold — control-plane skip diagnostic.
            reason: e.to_string(),
        })?;
    if bytes.len() > MAX_RUN_ARTIFACT_LEN {
        return Err(RunArtifactError::TooLong {
            file,
            len: bytes.len() as u64,
        });
    }
    String::from_utf8(bytes).map_err(|_| RunArtifactError::NotUtf8 { file })
}

/// A run the serve side gathered but could NOT describe, and why — the explicit
/// complement to a served [`RunEntry`].
///
/// Carries the run's identity and directory so the reason is actionable: an
/// operator reading the serve log gets the run it is about and the path to look
/// at, not merely a count.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SkippedRun {
    /// The gathered run's 128-bit identity (raw, not the wire text — this value
    /// never reaches the wire).
    pub run_id: u128,
    /// The run directory the registry record pointed at.
    pub run_dir: String,
    /// Why the run could not be served, in operator-readable text.
    pub reason: String,
}

impl SkippedRun {
    /// The WIRE form — identity plus reason, with the local-only `run_dir`
    /// dropped.
    ///
    /// The path is deliberately NOT carried: it is a filesystem location on
    /// ANOTHER machine, useless to the desk and a small disclosure of the robot's
    /// layout to whoever asked. It stays in the serve-side log, where the person
    /// who can act on it is reading.
    #[must_use]
    pub fn to_wire(&self) -> UndescribableRun {
        UndescribableRun {
            // hot-path-alloc-ok: cold — one per skipped run on a control-plane GET.
            run_id: format_run_id(self.run_id),
            reason: self.reason.clone(),
        }
    }
}

/// The outcome of folding gathered records + their directories into served
/// entries: what CAN be described, and what could not.
#[derive(Debug, Default, Clone, PartialEq, Eq)]
pub struct RunFold {
    /// Entries ready for [`build_runs_reply`](super::cerulion_q::build_runs_reply).
    pub entries: Vec<RunEntry>,
    /// Runs skipped, each with its named reason.
    pub skipped: Vec<SkippedRun>,
}

/// Fold gathered run records into served entries by reading each run's directory.
///
/// ONE ENTRY PER IDENTITY holds by construction rather than by a second dedup
/// here: [`RunGather::records`](super::run_registry::RunGather) is unioned through
/// a `BTreeMap<u128, RunRecord>`, so the gather already collapses a run's repeated
/// republishes (last-write-wins, so a `Live` → `Ending` flip converges) before
/// this loop ever sees them. `build_runs_reply`'s dedup is the reply-side
/// BACKSTOP for that guarantee, not a licence to hand it duplicates — folding a
/// second dedup in here would be a second copy of one policy, free to disagree
/// with the first.
///
/// A per-run failure is a SKIP with a named reason, never an error return: one
/// unreadable run directory must not cost a robot every OTHER run it is executing
/// (and on the shipping path a run that exits mid-gather deletes its directory,
/// so an ordinary race lands here).
/// # Every directory is BOUNDED first
///
/// The runs root is canonicalised ONCE per fold, not once per record — it is the
/// same answer for every run, and resolving it per record would pay a syscall per
/// row for nothing. A root that does not resolve refuses EVERY record (see
/// [`RunDirRefusal::NoRunRoot`]): with no legitimate root there is no bounded set
/// to read from, and falling back to "read whatever the record says" is exactly
/// the traversal this closes.
pub fn collect_run_entries(records: &[RunRecord]) -> RunFold {
    let root = run_dir_root().and_then(|r| std::fs::canonicalize(r).ok());
    let mut fold = RunFold::default();
    for record in records {
        match describe_run(record, root.as_deref()) {
            Ok(entry) => fold.entries.push(entry),
            Err(reason) => fold.skipped.push(SkippedRun {
                run_id: record.run_id,
                run_dir: record.run_dir.clone(),
                reason,
            }),
        }
    }
    fold
}

/// Read one run's directory and mint its served entry, or report WHY not.
///
/// Artifact errors, containment refusals and [`RunEntry::new`]'s own refusals
/// collapse into one reason STRING here, deliberately: the caller's only use for
/// it is the skip log plus the operator reading it, and preserving three error
/// enums through the fold would buy a discrimination nothing acts on. **No
/// variant of any of them renders a PATH** — the reason crosses the wire, and the
/// one attacker-controlled input to this function is precisely that path.
fn describe_run(record: &RunRecord, root: Option<&Path>) -> Result<RunEntry, String> {
    // hot-path-alloc-ok (this fn): cold — one per gathered run on a control-plane
    // runs GET, never per frame.
    let Some(root) = root else {
        return Err(RunDirRefusal::NoRunRoot.to_string());
    };
    let dir = resolve_contained_run_dir(&record.run_dir, root).map_err(|e| e.to_string())?;
    let dir = dir.as_path();
    let graph_yaml = read_artifact(root, dir, GRAPH_YAML_FILE).map_err(|e| e.to_string())?;
    let run_json = read_artifact(root, dir, RUN_JSON_FILE).map_err(|e| e.to_string())?;
    RunEntry::new(
        record.run_id,
        record.graph_name.clone(),
        record.run_started_at_ns,
        record.state,
        graph_yaml,
        run_json,
    )
    .map_err(|e| e.to_string())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::transport::cerulion_q::format_run_id;
    use crate::transport::run_registry::RunState;
    #[cfg(unix)]
    use std::os::unix::fs::FileTypeExt;
    use std::sync::atomic::{AtomicU64, Ordering};
    use std::sync::OnceLock;
    use std::time::Duration;

    /// The RUNS ROOT every fixture in this module lives under, with
    /// `CERULION_HOME` pointed at it.
    ///
    /// Fixtures used to be bare `tempfile::tempdir`s, which the B2
    /// containment check now correctly REFUSES — a run directory outside the
    /// canonical root is exactly the traversal that check exists to stop. Putting
    /// them under a redirected root is strictly MORE faithful: it is where a real
    /// `graph run` writes.
    ///
    /// The `set_var` happens exactly once, inside `get_or_init`, and every test
    /// calls this as its FIRST action — so the single write completes before any
    /// test has a fixture to read the env for, and no reader can race it. The
    /// `TempDir` is held for the process lifetime (never dropped, so never
    /// removed): one directory per test binary run, which the OS reaps.
    fn runs_root() -> PathBuf {
        static ROOT: OnceLock<tempfile::TempDir> = OnceLock::new();
        let home = ROOT.get_or_init(|| {
            let dir = tempfile::tempdir().expect("runs root");
            std::env::set_var("CERULION_HOME", dir.path());
            std::fs::create_dir_all(dir.path().join("runs")).expect("runs dir");
            dir
        });
        home.path().join("runs")
    }

    /// A fresh, EXISTING run directory inside the canonical root.
    fn run_dir_in_root(tag: &str) -> PathBuf {
        static SEQ: AtomicU64 = AtomicU64::new(0);
        let dir = runs_root().join(format!("{tag}-{}", SEQ.fetch_add(1, Ordering::Relaxed)));
        std::fs::create_dir_all(&dir).expect("run dir");
        dir
    }

    /// A path inside the canonical root that does NOT exist.
    fn absent_dir_in_root(tag: &str) -> PathBuf {
        runs_root().join(format!("{tag}-absent"))
    }

    /// A record pointing at `dir`. Every field is hand-set so an oracle can name
    /// the exact value it expects back.
    fn record_at(run_id: u128, dir: &Path) -> RunRecord {
        RunRecord {
            run_id,
            supervisor_pid: 4242,
            run_started_at_ns: 1_700_000_000_000_000_000,
            state: RunState::Live,
            graph_name: "perception".to_string(),
            run_dir: dir.to_string_lossy().into_owned(),
        }
    }

    /// Write both artifacts into `dir` with the given bodies.
    fn write_run_dir(dir: &Path, graph_yaml: &str, run_json: &str) {
        std::fs::write(dir.join(GRAPH_YAML_FILE), graph_yaml).expect("write graph.yaml");
        std::fs::write(dir.join(RUN_JSON_FILE), run_json).expect("write run.json");
    }

    #[test]
    fn a_complete_run_directory_folds_into_an_entry_carrying_both_documents_verbatim() {
        let tmp = run_dir_in_root("a_complete_run_directory");
        // Deliberately awkward bodies: trailing newline, interior blank line, a
        // UTF-8 multi-byte char. VERBATIM means verbatim.
        let yaml = "name: perception\nprefix: /percep\n\nnodes:\n  - id: camera  # \u{00b5}s\n";
        let json = "{\"run_id\":\"0x0000000000000000000000000000002a\"}\n";
        write_run_dir(tmp.as_path(), yaml, json);

        let fold = collect_run_entries(&[record_at(42, tmp.as_path())]);

        assert!(fold.skipped.is_empty(), "skipped: {:?}", fold.skipped);
        assert_eq!(fold.entries.len(), 1);
        let entry = &fold.entries[0];
        assert_eq!(entry.run_id, format_run_id(42));
        assert_eq!(entry.graph_name, "perception");
        assert_eq!(entry.run_started_at_ns, 1_700_000_000_000_000_000);
        assert_eq!(entry.graph_yaml, yaml, "graph.yaml must be VERBATIM");
        assert_eq!(entry.run_json, json, "run.json must be VERBATIM");
    }

    #[test]
    fn a_run_directory_that_vanished_is_skipped_by_name_never_served_empty() {
        // Inside the canonical root, but gone — the run-exited-mid-gather shape.
        let gone = absent_dir_in_root("vanished");

        let fold = collect_run_entries(&[record_at(7, &gone)]);

        assert!(fold.entries.is_empty(), "an unreadable run is NEVER served");
        assert_eq!(fold.skipped.len(), 1);
        let skip = &fold.skipped[0];
        assert_eq!(skip.run_id, 7);
        assert_eq!(
            skip.run_dir,
            gone.to_string_lossy(),
            "the LOCAL skip keeps the path, for the operator's log"
        );
        // This is the CONTAINMENT layer's refusal
        // (`canonicalize` cannot resolve a directory that is not there), which is
        // reached BEFORE any artifact is named — a correct widening, since the
        // condition is about the DIRECTORY rather than about either file.
        assert!(
            skip.reason.contains("does not resolve"),
            "the skip must name the condition: {}",
            skip.reason
        );
        assert!(
            !skip.reason.contains(&gone.to_string_lossy().to_string()),
            "and must NOT echo the path — the reason crosses the wire: {}",
            skip.reason
        );
    }

    #[test]
    fn a_run_missing_only_its_run_json_is_skipped_naming_that_file() {
        let tmp = run_dir_in_root("a_run_missing_only_its_r");
        std::fs::write(tmp.as_path().join(GRAPH_YAML_FILE), "name: g\n").expect("write");

        let fold = collect_run_entries(&[record_at(9, tmp.as_path())]);

        assert!(fold.entries.is_empty());
        assert_eq!(fold.skipped.len(), 1);
        let reason = &fold.skipped[0].reason;
        assert!(
            reason.contains(RUN_JSON_FILE) && !reason.contains(GRAPH_YAML_FILE),
            "the reason must name run.json alone: {reason}"
        );
    }

    #[test]
    fn an_empty_graph_yaml_is_a_skip_not_an_entry_with_an_empty_document() {
        // THE headline rule: an empty document renders desk-side as a
        // graph with no nodes — a confident lie about a running robot.
        let tmp = run_dir_in_root("an_empty_graph_yaml_is_a");
        write_run_dir(tmp.as_path(), "", "{}\n");

        let fold = collect_run_entries(&[record_at(11, tmp.as_path())]);

        assert!(
            fold.entries.is_empty(),
            "an empty graph.yaml is NEVER served"
        );
        assert_eq!(fold.skipped.len(), 1);
        assert!(
            fold.skipped[0].reason.contains("graph.yaml"),
            "reason: {}",
            fold.skipped[0].reason
        );
    }

    #[test]
    fn an_oversized_artifact_is_refused_with_its_real_length_and_never_read_whole() {
        let tmp = run_dir_in_root("an_oversized_artifact_is");
        let over = MAX_RUN_ARTIFACT_LEN + 1;
        write_run_dir(tmp.as_path(), &"y".repeat(over), "{}\n");

        let fold = collect_run_entries(&[record_at(13, tmp.as_path())]);

        assert!(fold.entries.is_empty());
        assert_eq!(fold.skipped.len(), 1);
        let reason = &fold.skipped[0].reason;
        assert!(
            reason.contains(&over.to_string())
                && reason.contains(&MAX_RUN_ARTIFACT_LEN.to_string()),
            "the refusal must name BOTH the observed length and the cap: {reason}"
        );
    }

    #[test]
    fn an_artifact_exactly_at_the_cap_is_served_so_the_boundary_is_pinned_on_both_sides() {
        // The ANTI-TAUTOLOGY half of the arm above: a refusal test alone passes an
        // implementation that refuses everything.
        let tmp = run_dir_in_root("an_artifact_exactly_at_t");
        let at_cap = "y".repeat(MAX_RUN_ARTIFACT_LEN);
        write_run_dir(tmp.as_path(), &at_cap, "{}\n");

        let fold = collect_run_entries(&[record_at(17, tmp.as_path())]);

        assert!(fold.skipped.is_empty(), "skipped: {:?}", fold.skipped);
        assert_eq!(fold.entries.len(), 1);
        assert_eq!(fold.entries[0].graph_yaml.len(), MAX_RUN_ARTIFACT_LEN);
    }

    #[test]
    fn a_non_utf8_artifact_is_refused_rather_than_lossily_transcoded() {
        let tmp = run_dir_in_root("a_non_utf8_artifact_is_r");
        std::fs::write(tmp.as_path().join(GRAPH_YAML_FILE), [0xffu8, 0xfe, 0xfd])
            .expect("write binary");
        std::fs::write(tmp.as_path().join(RUN_JSON_FILE), "{}\n").expect("write");

        let fold = collect_run_entries(&[record_at(19, tmp.as_path())]);

        assert!(fold.entries.is_empty());
        assert!(
            fold.skipped[0].reason.contains("UTF-8"),
            "reason: {}",
            fold.skipped[0].reason
        );
    }

    /// A WRITER-LESS FIFO at `graph.yaml` is refused, and refused PROMPTLY.
    ///
    /// The member of the non-regular-file class that actually bites: `open`ing
    /// one read-only BLOCKS until a writer appears, on the shared zenoh callback
    /// thread that also serves `demand`/`catalog`/`schema` — so a regression here
    /// is not a wrong answer, it is a wedged surface.
    ///
    /// # Why the fold runs on a HELPER thread
    ///
    /// A regression makes this call NEVER RETURN, and a test that simply calls it
    /// would hang the binary rather than fail — the unbounded-cross-thread-wait
    /// class `shm_ring_test` closed, which on CI burns to the job timeout with no
    /// attributable red. The bound is a generous LIVENESS ceiling in seconds
    /// against microseconds of work, so load can only delay it.
    ///
    /// The thread is deliberately NOT joined on the timeout path: under a
    /// regression it is parked in `open` forever, and joining it would reproduce
    /// the very hang the bound exists to avoid. It is left detached (the process
    /// is about to die on the panic) and the FIFO is unlinked by the tempdir.
    #[cfg(unix)]
    #[test]
    fn a_writer_less_fifo_is_refused_promptly_rather_than_wedging_the_serving_thread() {
        use std::sync::mpsc;

        let tmp = run_dir_in_root("a_writer_less_fifo_is_re");
        let fifo = tmp.as_path().join(GRAPH_YAML_FILE);
        let c_path = std::ffi::CString::new(fifo.as_os_str().as_encoded_bytes()).expect("cstring");
        // SAFETY: `c_path` is a valid NUL-terminated path in a directory this
        // test owns; `mkfifo` writes nothing through the pointer.
        let rc = unsafe { libc::mkfifo(c_path.as_ptr(), 0o600) };
        assert_eq!(rc, 0, "mkfifo failed: {}", std::io::Error::last_os_error());
        std::fs::write(tmp.as_path().join(RUN_JSON_FILE), "{}\n").expect("write");
        // PRECONDITION: it really is a FIFO (so the arm cannot pass by having
        // created an ordinary file).
        assert!(
            std::fs::metadata(&fifo)
                .expect("stat fifo")
                .file_type()
                .is_fifo(),
            "the fixture must be a real FIFO"
        );

        let record = record_at(31, tmp.as_path());
        let (tx, rx) = mpsc::channel();
        std::thread::spawn(move || {
            let _ = tx.send(collect_run_entries(&[record]));
        });
        let fold = rx
            .recv_timeout(Duration::from_secs(20))
            .expect("a writer-less FIFO must be REFUSED, not opened — the fold never returned");

        assert!(fold.entries.is_empty());
        assert_eq!(fold.skipped.len(), 1);
        assert!(
            fold.skipped[0].reason.contains("not a regular file"),
            "reason: {}",
            fold.skipped[0].reason
        );
    }

    #[test]
    fn a_directory_where_an_artifact_should_be_is_refused_before_any_read() {
        // The portable, hermetic member of the non-regular-file class; its
        // blocking sibling is the FIFO arm above.
        let tmp = run_dir_in_root("a_directory_where_an_art");
        std::fs::create_dir(tmp.as_path().join(GRAPH_YAML_FILE)).expect("mkdir");
        std::fs::write(tmp.as_path().join(RUN_JSON_FILE), "{}\n").expect("write");

        let fold = collect_run_entries(&[record_at(23, tmp.as_path())]);

        assert!(fold.entries.is_empty());
        assert!(
            fold.skipped[0].reason.contains("not a regular file"),
            "reason: {}",
            fold.skipped[0].reason
        );
    }

    #[test]
    fn one_unreadable_run_costs_only_itself_the_others_are_still_served() {
        // The reason a per-run failure is a SKIP and not an error return.
        let good_a = run_dir_in_root("good_a");
        let bad = run_dir_in_root("bad");
        let good_b = run_dir_in_root("good_b");
        write_run_dir(good_a.as_path(), "name: a\n", "{\"a\":1}\n");
        // `bad` gets NO artifacts — the run-exited-mid-gather shape.
        write_run_dir(good_b.as_path(), "name: b\n", "{\"b\":1}\n");

        let fold = collect_run_entries(&[
            record_at(1, good_a.as_path()),
            record_at(2, bad.as_path()),
            record_at(3, good_b.as_path()),
        ]);

        let served: Vec<&str> = fold.entries.iter().map(|e| e.run_id.as_str()).collect();
        assert_eq!(served, vec![format_run_id(1), format_run_id(3)]);
        assert_eq!(fold.skipped.len(), 1);
        assert_eq!(fold.skipped[0].run_id, 2);
    }

    #[test]
    fn the_gathered_state_reaches_the_served_entry() {
        // `Ending` is the registry's LAST WORD — a desk that renders it
        // as `Live` would show a shutting-down run as healthy.
        let tmp = run_dir_in_root("the_gathered_state_reach");
        write_run_dir(tmp.as_path(), "name: g\n", "{}\n");
        let mut record = record_at(29, tmp.as_path());
        record.state = RunState::Ending;

        let fold = collect_run_entries(&[record]);

        assert_eq!(
            fold.entries[0].state,
            crate::transport::cerulion_q::RunEntryState::Ending
        );
    }

    /// THE traversal refusal: a `run_dir` pointing OUTSIDE the canonical runs
    /// root is refused, and the artifacts under it are never opened.
    ///
    /// `RunRecord::run_dir` is a PATH supplied by whatever published the registry
    /// record — a machine-wide SHM service any local process can write — so
    /// without this an AUTHORIZED remote `runs` GET returns arbitrary readable
    /// bytes as a run's `graph.yaml`.
    ///
    /// The fixture is a REAL readable file under a REAL directory outside the
    /// root, so a missing containment check would genuinely serve it (the arm
    /// cannot pass merely because the target does not exist), and the assertion is
    /// on the SERVED CONTENT as well as the skip.
    #[test]
    fn a_run_dir_outside_the_canonical_root_is_refused_and_its_bytes_never_served() {
        // ESTABLISH THE ROOT FIRST, even though every fixture below is
        // deliberately OUTSIDE it.
        //
        // `runs_root()` is what points `CERULION_HOME` at a temp directory, and
        // its doc says every test calls it as its first action. This one did
        // not — it needs no fixture inside the root — so it was relying on some
        // SIBLING test having called it earlier in the same process.
        //
        // That held under `cargo test -- --test-threads=1`, where the whole lib
        // suite shares one process. It does not hold under nextest, which gives
        // each test its OWN process: nothing sets `CERULION_HOME`, resolution
        // falls back to the home directory, and on a machine with no
        // `~/.cerulion` the fold refuses with `NoRunRoot` — a DIFFERENT refusal
        // than the traversal one this test is about, so the assertion below
        // fails on its message.
        //
        // It passed on developer desks (which have a real `~/.cerulion` from
        // running the tooling) and failed on fresh CI runners, which is the
        // worst shape for a latent dependency. Reproduced locally by scrubbing
        // HOME for the test process alone.
        let _root = runs_root();

        // Sibling of the runs root in the system temp dir, so genuinely outside
        // it — which is the whole point of the fixture.
        let outside = tempfile::tempdir().expect("outside");
        let secret = "SECRET: this must never reach the wire\n";
        write_run_dir(outside.path(), secret, "{\"k\":1}\n");

        let fold = collect_run_entries(&[record_at(0x0e_51, outside.path())]);

        assert!(
            fold.entries.is_empty(),
            "a run directory outside the root is NEVER served: {:?}",
            fold.entries
        );
        assert_eq!(fold.skipped.len(), 1);
        let reason = &fold.skipped[0].reason;
        assert!(
            reason.contains("outside the canonical runs root"),
            "the refusal must name the condition: {reason}"
        );
        assert!(
            !reason.contains(&outside.path().to_string_lossy().to_string())
                && !reason.contains(secret),
            "and must echo NEITHER the attacker-chosen path NOR the file: {reason}"
        );
        assert_eq!(
            fold.skipped[0].run_dir,
            outside.path().to_string_lossy(),
            "the LOCAL skip keeps the path — that is the operator's log, not the wire"
        );
    }

    /// A SYMLINK inside the root pointing OUT of it is refused — the shape a
    /// textual prefix test admits and canonicalisation catches.
    #[cfg(unix)]
    #[test]
    fn a_symlinked_run_dir_escaping_the_root_is_refused() {
        let outside = tempfile::tempdir().expect("outside");
        write_run_dir(outside.path(), "SECRET\n", "{\"k\":1}\n");
        let link = runs_root().join("escaping-link");
        let _ = std::fs::remove_file(&link);
        std::os::unix::fs::symlink(outside.path(), &link).expect("symlink");

        let fold = collect_run_entries(&[record_at(0x5111, &link)]);

        assert!(
            fold.entries.is_empty(),
            "a symlink escaping the root is NEVER served: {:?}",
            fold.entries
        );
        assert!(
            fold.skipped[0]
                .reason
                .contains("outside the canonical runs root"),
            "reason: {}",
            fold.skipped[0].reason
        );
    }

    /// A SYMLINKED ARTIFACT is refused even when its directory is perfectly
    /// contained — the containment check bounds the DIRECTORY and says nothing
    /// about the final component, so this is the other half of the same hazard.
    #[cfg(unix)]
    #[test]
    fn a_symlinked_artifact_inside_a_contained_run_dir_is_refused() {
        let outside = tempfile::tempdir().expect("outside");
        let secret = outside.path().join("secret.txt");
        std::fs::write(&secret, "SECRET: never on the wire\n").expect("write secret");

        let dir = run_dir_in_root("symlinked_artifact");
        std::os::unix::fs::symlink(&secret, dir.join(GRAPH_YAML_FILE)).expect("symlink");
        std::fs::write(dir.join(RUN_JSON_FILE), "{\"k\":1}\n").expect("write");

        let fold = collect_run_entries(&[record_at(0x5112, dir.as_path())]);

        assert!(
            fold.entries.is_empty(),
            "a symlinked artifact is NEVER served: {:?}",
            fold.entries
        );
        assert!(
            fold.skipped[0].reason.contains("not a regular file"),
            "reason: {}",
            fold.skipped[0].reason
        );
    }

    /// A SYMLINKED INTERMEDIATE COMPONENT is refused BY THE WALK — the TOCTOU
    /// window itself, driven at the one seam where it is observable.
    ///
    /// # Why this drives `open_artifact_within` directly
    ///
    /// The window is between `resolve_contained_run_dir` and the read, and
    /// `collect_run_entries` closes over both: it re-canonicalises on every call,
    /// so a swap performed before it runs is caught by CONTAINMENT and the walk is
    /// never reached. **Measured, not assumed** — the first version of this arm
    /// went through the fold, passed, and still passed when reverted to
    /// path-open, because containment was doing all the work.
    ///
    /// So the walk is handed exactly what a stale verdict hands it: a root and a
    /// run directory that WERE contained, whose components have since changed.
    /// That is not a synthetic shape — it is the state of the arguments at the
    /// instant the race is lost.
    ///
    /// The PRE-SWAP call is asserted first, so "it refuses" cannot be satisfied by
    /// a walk that refuses everything; and the swap is PROVEN to redirect an
    /// ordinary path-resolving read, so the refusal cannot be a broken fixture.
    #[cfg(unix)]
    #[test]
    fn a_component_swapped_for_a_symlink_after_containment_is_refused_by_the_walk() {
        let outside = tempfile::tempdir().expect("outside");
        let secret = "SECRET: reached through a swapped component\n";

        // A genuine, fully-real run directory two levels under the root.
        let parent = runs_root().join("swap-parent");
        std::fs::create_dir_all(&parent).expect("parent");
        let dir = parent.join("run");
        std::fs::create_dir_all(&dir).expect("run dir");
        write_run_dir(dir.as_path(), "name: real\n", "{\"k\":1}\n");

        // The arguments a successful containment check produces.
        let root = std::fs::canonicalize(runs_root()).expect("canonical root");
        let contained = resolve_contained_run_dir(&dir.to_string_lossy(), &root)
            .expect("the un-swapped path must be contained");

        // PRECONDITION: with nothing swapped, the walk serves the real file.
        let handle = open_artifact_within(&root, &contained, GRAPH_YAML_FILE)
            .expect("the un-swapped walk must open the artifact");
        drop(handle);

        // THE SWAP, performed on the very path that verdict was about: replace
        // the intermediate component with a symlink pointing out of the root,
        // holding a directory of the same shape.
        let decoy = outside.path().join("run");
        std::fs::create_dir_all(&decoy).expect("decoy");
        write_run_dir(&decoy, secret, "{\"k\":2}\n");
        std::fs::remove_dir_all(&parent).expect("remove parent");
        std::os::unix::fs::symlink(outside.path(), &parent).expect("symlink the component");

        // The swap is REAL: an ordinary path-resolving read now returns the
        // decoy, which is exactly what the walk must not do.
        assert_eq!(
            std::fs::read_to_string(dir.join(GRAPH_YAML_FILE)).expect("the swap resolves"),
            secret,
            "the fixture must really redirect a path-resolving read"
        );

        // THE PIN: the same arguments, after the swap.
        let err = open_artifact_within(&root, &contained, GRAPH_YAML_FILE)
            .expect_err("a swapped component is NEVER followed");
        let reason = err.to_string();
        assert!(
            !reason.contains(secret)
                && !reason.contains(&outside.path().to_string_lossy().to_string()),
            "and the refusal carries neither the file nor the attacker-chosen path: {reason}"
        );

        // END TO END, for completeness: the fold refuses it too — here through
        // CONTAINMENT, which re-canonicalises and sees the escape. Both layers
        // hold; only the assertion above is about the walk.
        let fold = collect_run_entries(&[record_at(0x_5_a_1, dir.as_path())]);
        assert!(fold.entries.is_empty(), "entries: {:?}", fold.entries);
        assert!(
            fold.skipped[0]
                .reason
                .contains("outside the canonical runs root"),
            "reason: {}",
            fold.skipped[0].reason
        );
    }

    /// ANTI-TAUTOLOGY for the whole containment layer: a LEGITIMATE run under the
    /// root still serves.
    ///
    /// Without it, every refusal arm above is satisfied by an implementation that
    /// refuses everything — which would take the verb offline rather than secure
    /// it. Deliberately a SEPARATE arm from the verbatim-documents one so a
    /// containment regression cannot be mistaken for a reading regression.
    #[test]
    fn a_run_under_the_canonical_root_is_still_served() {
        let dir = run_dir_in_root("contained_happy_path");
        write_run_dir(dir.as_path(), "name: contained\n", "{\"k\":1}\n");

        let fold = collect_run_entries(&[record_at(0x600d, dir.as_path())]);

        assert!(fold.skipped.is_empty(), "skipped: {:?}", fold.skipped);
        assert_eq!(fold.entries.len(), 1);
        assert_eq!(fold.entries[0].graph_yaml, "name: contained\n");
    }

    /// A ROOT-LOOKALIKE sibling is refused — `Path::starts_with` compares whole
    /// COMPONENTS, so `…/runs-evil` is not inside `…/runs`.
    ///
    /// Pins the property rather than trusting it: a containment check written as a
    /// string prefix admits this, and it is the classic way one is got wrong.
    #[test]
    fn a_directory_whose_name_merely_starts_with_the_root_is_refused() {
        let root = runs_root();
        let sibling = root.parent().expect("root has a parent").join(format!(
            "{}-evil",
            root.file_name().unwrap().to_string_lossy()
        ));
        std::fs::create_dir_all(&sibling).expect("sibling");
        write_run_dir(&sibling, "SECRET\n", "{\"k\":1}\n");

        let fold = collect_run_entries(&[record_at(0x5163, &sibling)]);

        assert!(fold.entries.is_empty(), "entries: {:?}", fold.entries);
        assert!(
            fold.skipped[0]
                .reason
                .contains("outside the canonical runs root"),
            "reason: {}",
            fold.skipped[0].reason
        );
    }

    #[test]
    fn an_empty_record_set_folds_to_an_empty_answer_with_nothing_skipped() {
        let fold = collect_run_entries(&[]);
        assert_eq!(fold, RunFold::default());
    }
}
