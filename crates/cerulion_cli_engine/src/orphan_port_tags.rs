// SPDX-License-Identifier: AGPL-3.0-only
//! The orphan port-tag reclaim: the ONE dead-node shape iceoryx2's sweep can
//! never clear, selected from `cerulion clean`'s attributed refusals and
//! healed by descriptor-pinned removal of the tags.
//!
//! Its own module, apart from [`crate::ipc_cleanup`], on purpose: that module
//! WIRES the sweeps and the exit pass and performs no removal itself (the
//! `the_exit_path_reclaims_through_the_shared_machinery` guard in
//! `shm_state` walks it for `remove_file(` / `read_dir(`), and this one is
//! where the destructive primitive of the orphan-tag reclaim lives. The
//! liveness evidence it needs is NOT its own: it asks
//! [`crate::shm_state::creator_verdict`], the one predicate the `.shm_state`
//! reclamation trusts (the
//! `no_second_copy_of_the_liveness_evidence_predicate_exists` guard
//! forbids a second spelling anywhere in the engine or the CLI).

use std::path::{Path, PathBuf};

use iceoryx2::config::Config;
use iceoryx2::prelude::SemanticString;

use crate::ipc_cleanup::FailedNodeCleanup;
use crate::shm_state::CreatorVerdict;

/// iceoryx2's `Config`, re-exported for the one CLI caller that must hand the
/// GLOBAL config to [`orphan_port_tag_candidates`] and
/// [`reclaim_orphan_port_tags`] without taking a direct `iceoryx2` dependency
/// (the binary crate deliberately has none — every iceoryx2 concern lives in
/// this engine).
pub use iceoryx2::config::Config as Iceoryx2Config;

// ── Orphan port tags: the ONE dead-node shape the sweep can never clear ──────
//
// A publisher destroyed while one of its loaned samples had been leaked
// (`mem::forget` of a loan, the rmw destroy path that has since been fixed at
// the source) deregisters its port but leaves the port's on-disk tag
// behind: the tag is owned by the publisher's shared state, which every
// forgotten sample keeps alive until the process dies, and a dead process
// removes nothing. iceoryx2's sweep (`iceoryx2-0.9.1/src/node/mod.rs:668-730`)
// then runs, in order: the service-tags pass (deregisters the node's REGISTERED
// ports and removes THEIR tags), the port-tags pass (`remove_stale_port_resources`
// per remaining tag — reclaims the port's data segment and connections but
// never deletes the tag itself), and `remove_node` (removes the `.details`
// storage, then `rmdir`s the node directory). That final `rmdir` fails
// `ENOTEMPTY` on the orphan tag, on EVERY sweep, forever — `failed_cleanups`
// never reaches 0, so the `.shm_state` reclamation stands down for
// good and the desk fills up (13,647 stale mappings behind eleven such
// directories on a development machine).
//
// WHY REMOVING THE TAGS IS SAFE — the argument the reclaimer below relies on,
// and the reason it refuses anything that does not fit it exactly. The `rmdir`
// failure is reachable ONLY after both tag passes succeeded on THIS sweep
// (each is followed by `cleanup_failure?`), so every tag still in the directory
// has already had its port resources reclaimed and belongs to a port no
// service registers; the node itself is dead (iceoryx2 established that — its
// monitor lock is free — and the reclaimer re-proves the process is gone). The
// tags are pure residue. Any OTHER residue — an entry that is not
// `<prefix><port id><port-tag suffix>`, a sub-cause other than the four lines
// below, a variant other than `InternalError`, a live or unprovable pid —
// means the argument does not apply, and the reclaimer REFUSES and reports
// rather than guessing. The reclaimer only heals a robot that already carries
// the shape; the source fix is what stops it being minted.

/// The four sub-cause lines iceoryx2 0.9.1 logs, in this order, on the way to
/// refusing a dead node whose directory holds only orphan port tags:
///
/// 1. `Unable to remove empty directory "<dir>" since the directory is not
///    empty or there are hard links pointing to the directory.`
///    (`iceoryx2-bb-posix` `Directory::remove_empty()`, `ENOTEMPTY` arm — the
///    ONLY line that names the directory, which is where the reclaimer reads it)
/// 2. `Unable to remove path hint due to an internal error
///    (NotEmptyOrHardLinksPointingToTheDirectory).` (`iceoryx2-cal`
///    `remove_path_hint`)
/// 3. `Unable to remove node details directory due to an internal error.`
///    (`remove_node_details_directory`, `mod.rs:826`)
/// 4. `Unable to remove stale resources since the node itself could not be
///    removed.` (`remove_stale_resources_impl`, `mod.rs:727`)
///
/// followed by the refusal line itself, `Unable to remove dead node … (InternalError).`
///
/// PRECEDED, on a real sweep, by a run of PROBE lines that say nothing about
/// why the node was refused — three shapes, every one MEASURED on a real
/// sweep and cited to the line that emits it:
///
/// * `Unable to open file due to insufficient permissions.` —
///   `FileBuilder::open_existing`'s `EACCES` arm
///   (`iceoryx2-bb-posix-0.9.1/src/file.rs:589-614`), logged just before…
/// * `Unable to open ProcessMonitor state file "<…>_context" with access mode
///   Write due to insufficient permissions.` — `ProcessMonitor::open_file`
///   (`process_state.rs:1021-1035`), called by `ProcessMonitor::state()`
///   (`:909-931`), which opens the node's monitor context file with
///   `AccessMode::Write` FIRST: a DESIGNED `EACCES`, because the context file
///   is made read-only once the node is initialised, and `state()` swallows
///   it. The sweep probes a dead node twice (classifying it in `Node::list`,
///   then acquiring its cleaner), so this pair normally appears twice.
/// * `Unable to open static storage due to a failure while opening the file.`
///   — `static_storage::File::Builder::open()`
///   (`iceoryx2-cal-0.9.1/src/static_storage/file.rs:443-449`): the node's
///   `.details` storage no longer exists, because the PREVIOUS sweep's
///   `remove_node` removed it before its `rmdir` failed. It is absent on the
///   first sweep that refuses a node and present on every later one — i.e.
///   on every `cerulion clean` a robot carrying the shape runs after the
///   first, which is the run that has to heal it (MEASURED: the first
///   selector, written to the first-sweep shape, healed nothing on a
///   re-sweep).
///
/// * `Unable to open file since it does not exist.` — `FileBuilder::open_existing`'s
///   `ENOENT` arm (`iceoryx2-bb-posix-0.9.1/src/file.rs:621`), the line the
///   static-storage open above logs FIRST when the `.details` file is gone.
///   It is `since it …`, not `since the …`, so the original sub-cause
///   predicate never attributed it; since the attribution recognises every
///   0.9.1 explanation shape it lands in the chain on every re-sweep
///   (measured: a real orphan-tag node, pid gone, directory
///   holding one tag, refused by the selector for exactly this line —
///   `cerulion clean`'s convergence e2e caught it).
///
/// The probe pair also fires for every ALIVE node the walk lists, and those
/// lines carry no node token, so the attribution's adjacency arm may hand a
/// NEIGHBOUR's probe lines to a refused node — noise about the monitor, not
/// evidence about this node, so a probe line naming another node's context
/// file is tolerated. The lead may hold NOTHING but these four shapes; any
/// other line — and in particular any line carrying one of
/// [`ORPHAN_TAG_DISQUALIFIERS`], or a permission refusal on anything but the
/// probe shapes — refuses.
const MONITOR_PROBE_OPEN_LINE: &str = "Unable to open file due to insufficient permissions.";
const FILE_MISSING_OPEN_LINE: &str = "Unable to open file since it does not exist.";
const MONITOR_PROBE_STATE_PREFIX: &str = "Unable to open ProcessMonitor state file \"";
const MONITOR_PROBE_STATE_SUFFIX: &str =
    "\" with access mode Write due to insufficient permissions.";
const STATIC_STORAGE_OPEN_LINE: &str =
    "Unable to open static storage due to a failure while opening the file.";
/// `ProcessMonitor`'s context file is `<state path>_context`
/// (`process_state.rs:166`); the state path carries the config's
/// `monitor_suffix`.
const MONITOR_CONTEXT_TAIL: &str = "_context";
/// Fragments that mark a DIFFERENT refusal — a port whose resources could not
/// be reclaimed, a corrupted or version-skewed service, unreadable tags, a
/// contended cleaner lock. The allow-list above already refuses every line
/// that is not one of the three probe shapes; this set is the belt to that
/// brace, checked over EVERY line of the chain, and each entry is pinned by
/// its own oracle arm so a future widening of the allow-list cannot quietly
/// admit one of them.
pub const ORPHAN_TAG_DISQUALIFIERS: [&str; 7] = [
    "stale resources of the port",
    "dynamic service segment is missing",
    "service tags could not be read",
    "port tags could not be read",
    "monitor cleaner lock",
    "corrupted",
    "different iceoryx2 version",
];
const ORPHAN_TAG_RMDIR_PREFIX: &str = "Unable to remove empty directory \"";
const ORPHAN_TAG_RMDIR_SUFFIX: &str =
    "\" since the directory is not empty or there are hard links pointing to the directory.";
const ORPHAN_TAG_PATH_HINT_LINE: &str =
    "Unable to remove path hint due to an internal error (NotEmptyOrHardLinksPointingToTheDirectory).";
const ORPHAN_TAG_DETAILS_DIR_LINE: &str =
    "Unable to remove node details directory due to an internal error.";
const ORPHAN_TAG_NODE_ITSELF_LINE: &str =
    "Unable to remove stale resources since the node itself could not be removed.";
/// The `NodeCleanupFailure` variant the chain above ends in. Any other variant
/// with the same lines is a shape this module has never seen and does not claim.
const ORPHAN_TAG_VARIANT: &str = "InternalError";

/// A dead node whose refusal is EXACTLY the orphan-port-tag chain — selected
/// by [`orphan_port_tag_candidates`], acted on by [`reclaim_orphan_port_tags`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct OrphanTagNode {
    /// The node identity verbatim, as [`FailedNodeCleanup::node`] carries it.
    pub node: String,
    /// The `value:` of the node's `UniqueSystemId` — also the name of its
    /// directory under the registry (`UniqueNodeId::as_file_name` renders the
    /// value in decimal).
    pub node_id: u128,
    /// The `pid:` of the node's `UniqueSystemId` — the process that minted it.
    pub pid: u32,
    /// The directory the `rmdir` failed on, exactly as line 1 quoted it. Already
    /// cross-checked against `<config.global.node_dir()>/<node_id>` at selection.
    pub dir: PathBuf,
}

/// What [`reclaim_orphan_port_tags`] did — or refused to do — for one candidate.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct OrphanTagReclaim {
    /// See [`OrphanTagNode::node`].
    pub node: String,
    /// See [`OrphanTagNode::node_id`].
    pub node_id: u128,
    /// See [`OrphanTagNode::pid`].
    pub pid: u32,
    /// See [`OrphanTagNode::dir`].
    pub dir: PathBuf,
    /// The port ids whose tags were removed — or, under `dry_run`, WOULD have
    /// been. On a refusal this holds whatever was removed before the refusal
    /// (a mid-way I/O error), never a projection.
    pub removed: Vec<u128>,
    /// What became of the directory — see [`ReclaimVerdict`].
    pub verdict: ReclaimVerdict,
}

impl OrphanTagReclaim {
    /// Why the directory was left alone, if it was — the reason of a
    /// [`ReclaimVerdict::Refused`]; `None` for the two outcomes that leave
    /// nothing to refuse.
    #[must_use]
    pub fn refused(&self) -> Option<&str> {
        match &self.verdict {
            ReclaimVerdict::Refused(reason) => Some(reason.as_str()),
            ReclaimVerdict::Reclaimed | ReclaimVerdict::AlreadyEmpty => None,
        }
    }
}

/// The outcome of [`reclaim_orphan_port_tags`] for one candidate — three
/// outcomes, because "nothing was removed" covers two that the next sweep
/// treats OPPOSITELY.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ReclaimVerdict {
    /// Every entry was an orphan tag of a provably-dead node and (unless
    /// `dry_run`) all of them are gone; [`OrphanTagReclaim::removed`] lists
    /// them.
    Reclaimed,
    /// The directory held NO entry at all when it was listed — a concurrent
    /// session's or an earlier, interrupted `cerulion clean`'s reclaim had
    /// already emptied it. Nothing to remove, nothing to refuse: the node is
    /// CONVERGED PENDING SWEEP, since the next sweep's `remove_node` is what
    /// removes an empty directory. It is not a refusal, and a caller that
    /// gated its second sweep on "a tag was removed" would leave exactly this
    /// node standing for one more run.
    AlreadyEmpty,
    /// The directory was left alone, for the reason given. A mid-way removal
    /// failure names the entry and the error only — the count of what was
    /// removed before it is `removed.len()`, stated once by the renderer.
    Refused(String),
}

/// Parse `value: <u128>` and `pid: <u32>` out of a node token
/// (`UniqueNodeId(UniqueSystemId { value: …, pid: …, creation_time: … })`).
/// Any other rendering yields `None` — a node whose identity cannot be read
/// is never a candidate.
fn parse_node_identity(token: &str) -> Option<(u128, u32)> {
    fn field<T: std::str::FromStr>(token: &str, key: &str) -> Option<T> {
        let at = token.find(key)? + key.len();
        let digits: String = token[at..]
            .trim_start()
            .chars()
            .take_while(char::is_ascii_digit)
            .collect();
        digits.parse().ok()
    }
    Some((field(token, "value:")?, field(token, "pid:")?))
}

/// The `creation_time` seconds of a node token, ONLY when its clock is
/// `Realtime` (Unix seconds) — a `Monotonic` stamp (seconds since boot) cannot
/// be compared to wall time and yields `None`, as does any other rendering.
fn parse_creation_unix_s(token: &str) -> Option<u64> {
    let at = token.find("creation_time:")?;
    let rest = &token[at..];
    if !rest.contains("clock_type: Realtime") {
        return None;
    }
    let secs = rest.find("seconds:")? + "seconds:".len();
    let digits: String = rest[secs..]
        .trim_start()
        .chars()
        .take_while(char::is_ascii_digit)
        .collect();
    digits.parse().ok()
}

/// The directory iceoryx2 keeps a node's details in:
/// `<config.global.node_dir()>/<node id in decimal>` (`node_details_path`,
/// `iceoryx2-0.9.1/src/service/config_scheme.rs:89`).
fn node_details_dir(config: &Config, node_id: u128) -> PathBuf {
    PathBuf::from(String::from(&config.global.node_dir())).join(node_id.to_string())
}

/// Path equality by COMPONENT, so a trailing slash or a doubled separator in
/// either rendering cannot make the same directory look like a different one.
fn same_path(a: &Path, b: &Path) -> bool {
    a.components().eq(b.components())
}

/// Is this one of the four probe shapes documented at
/// [`MONITOR_PROBE_OPEN_LINE`]? The state-file shape must quote a
/// `<monitor_suffix>_context` path — any other quoted path is a different
/// open, and refuses.
fn is_benign_probe_line(cause: &str, monitor_context_suffix: &str) -> bool {
    cause == MONITOR_PROBE_OPEN_LINE
        || cause == FILE_MISSING_OPEN_LINE
        || cause == STATIC_STORAGE_OPEN_LINE
        || cause
            .strip_prefix(MONITOR_PROBE_STATE_PREFIX)
            .and_then(|rest| rest.strip_suffix(MONITOR_PROBE_STATE_SUFFIX))
            .is_some_and(|path| path.ends_with(monitor_context_suffix))
}

/// Select, from the attributed refusals, exactly the dead nodes whose refusal
/// is the orphan-port-tag chain: variant `InternalError`, no line carrying a
/// [`ORPHAN_TAG_DISQUALIFIERS`] fragment, sub-causes EXACTLY a (possibly
/// empty) leading run of the three probe shapes followed by the four lines
/// above in order and nothing after, the directory line 1 quotes equal (by
/// component) to `<config.global.node_dir()>/<node id>`, and a token that
/// carries both `value:` and `pid:`. Anything else is not a candidate — an
/// extra line, a missing line, a `stale resources of the port …` failure, a
/// probe line after the chain, a directory outside the configured registry —
/// because the safety argument in this section's header is stated for this
/// chain and no other.
///
/// PURE: no filesystem, no registry, no clock — hand-built captures pin it.
#[must_use]
pub fn orphan_port_tag_candidates(
    failures: &[FailedNodeCleanup],
    config: &Config,
) -> Vec<OrphanTagNode> {
    let monitor_context_suffix = format!(
        "{}{MONITOR_CONTEXT_TAIL}",
        String::from_utf8_lossy(config.global.node.monitor_suffix.as_bytes())
    );
    failures
        .iter()
        .filter_map(|failure| {
            if failure.variant != ORPHAN_TAG_VARIANT {
                return None;
            }
            if failure.causes.iter().any(|cause| {
                ORPHAN_TAG_DISQUALIFIERS
                    .iter()
                    .any(|fragment| cause.contains(fragment))
            }) {
                return None;
            }
            let probe_run = failure
                .causes
                .iter()
                .take_while(|cause| is_benign_probe_line(cause, &monitor_context_suffix))
                .count();
            let [rmdir, path_hint, details_dir, node_itself] = &failure.causes[probe_run..] else {
                return None;
            };
            if path_hint != ORPHAN_TAG_PATH_HINT_LINE
                || details_dir != ORPHAN_TAG_DETAILS_DIR_LINE
                || node_itself != ORPHAN_TAG_NODE_ITSELF_LINE
            {
                return None;
            }
            let quoted = rmdir
                .strip_prefix(ORPHAN_TAG_RMDIR_PREFIX)?
                .strip_suffix(ORPHAN_TAG_RMDIR_SUFFIX)?;
            let (node_id, pid) = parse_node_identity(&failure.node)?;
            let dir = PathBuf::from(quoted);
            if !same_path(&dir, &node_details_dir(config, node_id)) {
                return None;
            }
            Some(OrphanTagNode {
                node: failure.node.clone(),
                node_id,
                pid,
                dir,
            })
        })
        .collect()
}

// ── Descriptor-pinned directory access ──────────────────────────────────────
//
// The registry check on a candidate is LEXICAL (`same_path` on the string the
// sweep quoted), and a path does not bind an identity: between that check and
// a delete, the node directory can be replaced by a symbolic link (or another
// directory) pointing OUTSIDE the registry, and a path-driven `read_dir` +
// `remove_file` would follow it and delete unrelated files that happen to be
// named like tags. So every filesystem operation below runs on DESCRIPTORS,
// never on paths: the registry root is reached DIRECTORY BY DIRECTORY — every
// component `openat`ed from the previous one `O_DIRECTORY|O_NOFOLLOW`, a link
// followed only where root planted it (`open_root`) — the node directory is
// `openat`ed FROM that root by its bare NAME (no separators, so it cannot
// escape) with the same flags — a symbolic link fails `ELOOP`, a
// non-directory `ENOTDIR`, both REFUSED and named — entries are listed through
// the pinned descriptor (`fdopendir` on a `dup`), each is checked with
// `fstatat(dirfd, name, AT_SYMLINK_NOFOLLOW)` to be a REGULAR file (a symlink
// named like a tag is an offender), and removal is `unlinkat(dirfd, name, 0)`,
// which removes the ENTRY in the pinned directory and follows nothing. A swap
// of the directory after it is pinned changes nothing the descriptor sees; a
// swap of an ENTRY after its check can at most make `unlinkat` remove the
// swapped-in link itself, inside the pinned directory.

#[cfg(unix)]
mod pinned {
    use std::ffi::{CStr, CString};
    use std::os::fd::{AsRawFd, FromRawFd, OwnedFd};

    const DIR_FLAGS: libc::c_int =
        libc::O_RDONLY | libc::O_DIRECTORY | libc::O_NOFOLLOW | libc::O_CLOEXEC;

    fn cstring(name: &str) -> Result<CString, String> {
        CString::new(name).map_err(|_| format!("`{name}` contains an interior NUL"))
    }

    /// Open the registry root DIRECTORY BY DIRECTORY, following no link that
    /// root did not plant.
    ///
    /// A single `open(path, O_NOFOLLOW)` refuses a link at the LAST component
    /// only — every earlier component is resolved by the kernel, links
    /// included — so a registry path that passed THROUGH a planted link could
    /// still aim the reclaim outside the registry. Here each component is
    /// `openat`ed from the previous descriptor with `O_DIRECTORY|O_NOFOLLOW`,
    /// starting at `/` (or the working directory for a relative root). A
    /// component that is a symbolic link — `ELOOP` on Linux, but `ENOTDIR` on
    /// macOS, whose `O_DIRECTORY` check answers first (MEASURED: `/var` refused
    /// as "not a directory" on the first run), so both errnos are settled by an
    /// `fstatat` of the component — is followed ONLY when root
    /// owns the link: `/tmp → private/tmp` and `/var → private/var` on macOS
    /// are root-owned links in the OS layout, and the default registry root
    /// lives under the first — an OS-planted link is trusted exactly as far as
    /// the OS is. Any other owner refuses, naming the component and its uid,
    /// and that includes the operator's own link: the walk cannot tell it from
    /// one planted by a process running as the operator, and the remedy is in
    /// the refusal (point `root_path` at the resolved directory). `..` refuses
    /// too — from a followed link it is an escape.
    pub(super) fn open_root(path: &str) -> Result<OwnedFd, String> {
        let (start, rest) = match path.strip_prefix('/') {
            Some(rest) => ("/", rest),
            None => (".", path),
        };
        let c = cstring(start)?;
        // SAFETY: `c` is a valid NUL-terminated string; the flags open for
        // reading only and create nothing. Neither `/` nor `.` is a link.
        let fd = unsafe { libc::open(c.as_ptr(), DIR_FLAGS) };
        if fd < 0 {
            return Err(format!(
                "the registry directory `{path}` could not be opened at `{start}`: {}",
                std::io::Error::last_os_error()
            ));
        }
        // SAFETY: `fd` is a freshly opened descriptor this process owns.
        let mut dir = unsafe { OwnedFd::from_raw_fd(fd) };
        let mut so_far = String::from(start.trim_end_matches('/'));
        for name in rest.split('/').filter(|c| !c.is_empty() && *c != ".") {
            if name == ".." {
                return Err(format!(
                    "the registry path `{path}` steps upward (`..`) — refusing to walk it; \
                     nothing is removed"
                ));
            }
            so_far.push('/');
            so_far.push_str(name);
            let c = cstring(name)?;
            // SAFETY: `dir` is an open directory descriptor and `c` a valid
            // NUL-terminated name without separators; the flags open for
            // reading only and follow nothing.
            let next = unsafe { libc::openat(dir.as_raw_fd(), c.as_ptr(), DIR_FLAGS) };
            if next >= 0 {
                // SAFETY: freshly opened, owned here.
                dir = unsafe { OwnedFd::from_raw_fd(next) };
                continue;
            }
            let err = std::io::Error::last_os_error();
            if !matches!(err.raw_os_error(), Some(libc::ELOOP) | Some(libc::ENOTDIR)) {
                return Err(format!(
                    "the registry directory `{path}` could not be opened at `{so_far}`: {err}"
                ));
            }
            // Possibly a symbolic link (`ELOOP` on Linux, `ENOTDIR` on macOS
            // under `O_DIRECTORY`); the link ITSELF is stat'ed to settle it.
            let st = lstat_at(&dir, name).map_err(|e| {
                format!("`{so_far}` in the registry path `{path}` could not be examined: {e}")
            })?;
            if (st.st_mode & libc::S_IFMT) != libc::S_IFLNK {
                return Err(format!(
                    "`{so_far}` in the registry path `{path}` is not a directory ({err});                      nothing is removed"
                ));
            }
            // A symbolic link. Root's own — the OS layout — is followed, this
            // one link only; anyone else's refuses.
            let uid = st.st_uid;
            if uid != 0 {
                return Err(format!(
                    "the registry path `{path}` passes through `{so_far}`, a symbolic link owned \
                     by uid {uid}, not by root — the reclaim walks the registry directory by \
                     directory and follows only links the system itself planted; point \
                     `root_path` at the resolved directory. Nothing is removed"
                ));
            }
            // SAFETY: as above, minus `O_NOFOLLOW` — following exactly this
            // root-owned link.
            let followed = unsafe {
                libc::openat(
                    dir.as_raw_fd(),
                    c.as_ptr(),
                    libc::O_RDONLY | libc::O_DIRECTORY | libc::O_CLOEXEC,
                )
            };
            if followed < 0 {
                return Err(format!(
                    "the registry directory `{path}` could not be opened at `{so_far}` (a \
                     root-owned link): {}",
                    std::io::Error::last_os_error()
                ));
            }
            // SAFETY: freshly opened, owned here.
            dir = unsafe { OwnedFd::from_raw_fd(followed) };
        }
        Ok(dir)
    }

    /// `stat` of the entry `name` in the pinned directory — the entry ITSELF
    /// (`AT_SYMLINK_NOFOLLOW`), never a link's target: its type says whether it
    /// is a link, its owner whether root planted it.
    fn lstat_at(dir: &OwnedFd, name: &str) -> Result<libc::stat, std::io::Error> {
        let c =
            cstring(name).map_err(|e| std::io::Error::new(std::io::ErrorKind::InvalidInput, e))?;
        // SAFETY: zeroed `stat` is a valid out-parameter for `fstatat`.
        let mut st: libc::stat = unsafe { std::mem::zeroed() };
        // SAFETY: `dir` is an open directory descriptor, `c` a valid name,
        // `st` a valid out-parameter; `AT_SYMLINK_NOFOLLOW` stats the link.
        let rc = unsafe {
            libc::fstatat(
                dir.as_raw_fd(),
                c.as_ptr(),
                &mut st,
                libc::AT_SYMLINK_NOFOLLOW,
            )
        };
        if rc != 0 {
            return Err(std::io::Error::last_os_error());
        }
        Ok(st)
    }

    /// What `openat` said about the node directory NAME under the root.
    pub(super) enum PinError {
        /// `ENOENT` — the entry is gone.
        Vanished,
        /// `ELOOP` / `ENOTDIR` — a symbolic link, or not a directory.
        NotADirectory,
        Other(std::io::Error),
    }

    /// Pin the node directory by NAME from the root descriptor.
    pub(super) fn open_node_dir(root: &OwnedFd, name: &str) -> Result<OwnedFd, PinError> {
        debug_assert!(
            !name.contains('/'),
            "a node directory name never carries a separator"
        );
        let c = cstring(name).map_err(|e| {
            PinError::Other(std::io::Error::new(std::io::ErrorKind::InvalidInput, e))
        })?;
        // SAFETY: `root` is an open directory descriptor and `c` a valid
        // NUL-terminated name; the flags open for reading only.
        let fd = unsafe { libc::openat(root.as_raw_fd(), c.as_ptr(), DIR_FLAGS) };
        if fd >= 0 {
            // SAFETY: freshly opened, owned here.
            return Ok(unsafe { OwnedFd::from_raw_fd(fd) });
        }
        let err = std::io::Error::last_os_error();
        Err(match err.raw_os_error() {
            Some(libc::ENOENT) => PinError::Vanished,
            Some(libc::ELOOP) | Some(libc::ENOTDIR) => PinError::NotADirectory,
            _ => PinError::Other(err),
        })
    }

    /// Every entry name in the pinned directory (`.` and `..` excluded),
    /// read through a `dup` of the descriptor so the caller keeps its own.
    pub(super) fn list(dir: &OwnedFd) -> Result<Vec<String>, String> {
        // SAFETY: `dir` is an open descriptor; `dup` returns a new one we own.
        let dup = unsafe { libc::dup(dir.as_raw_fd()) };
        if dup < 0 {
            return Err(format!(
                "the node directory could not be listed: {}",
                std::io::Error::last_os_error()
            ));
        }
        // SAFETY: `dup` is an open directory descriptor; on success
        // `fdopendir` takes ownership of it and `closedir` releases it.
        let stream = unsafe { libc::fdopendir(dup) };
        if stream.is_null() {
            let err = std::io::Error::last_os_error();
            // SAFETY: `fdopendir` failed, so `dup` is still ours to close.
            unsafe { libc::close(dup) };
            return Err(format!("the node directory could not be listed: {err}"));
        }
        let mut names = Vec::new();
        loop {
            // `readdir` reports BOTH end-of-directory and an I/O error as
            // NULL and distinguishes them only through `errno`, which it
            // leaves untouched at EOF — so `errno` is cleared first (a stale
            // value from an earlier syscall, `kill(pid, 0)`'s `ESRCH` from the
            // death guard included, would otherwise read as an error or mask
            // one) and read back after a NULL.
            clear_errno();
            // SAFETY: `stream` is a valid DIR* until `closedir` below.
            let entry = unsafe { libc::readdir(stream) };
            if entry.is_null() {
                match classify_readdir_end(
                    std::io::Error::last_os_error().raw_os_error().unwrap_or(0),
                ) {
                    ReaddirEnd::Eof => break,
                    ReaddirEnd::Error(errno) => {
                        // SAFETY: `stream` came from `fdopendir` and is closed
                        // exactly once, here.
                        unsafe { libc::closedir(stream) };
                        // A listing that stopped short is NOT a listing: the
                        // proof "every entry is a validated orphan tag" needs
                        // every entry, so nothing is unlinked.
                        return Err(format!(
                            "the node directory could not be listed completely ({}) — refusing \
                             to touch it",
                            std::io::Error::from_raw_os_error(errno)
                        ));
                    }
                }
            }
            // SAFETY: `readdir` returned a valid entry whose `d_name` is
            // NUL-terminated.
            let name = unsafe { CStr::from_ptr((*entry).d_name.as_ptr()) }
                .to_string_lossy()
                .into_owned();
            if name != "." && name != ".." {
                names.push(name);
            }
        }
        // SAFETY: `stream` came from `fdopendir` and is closed exactly once.
        unsafe { libc::closedir(stream) };
        Ok(names)
    }

    /// How a NULL from `readdir` reads once `errno` has been cleared before
    /// the call: `0` is end-of-directory, anything else an I/O error that
    /// means the listing is INCOMPLETE.
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub(super) enum ReaddirEnd {
        Eof,
        Error(i32),
    }

    /// PURE: the one decision the listing loop makes on a NULL entry.
    pub(super) fn classify_readdir_end(errno: i32) -> ReaddirEnd {
        if errno == 0 {
            ReaddirEnd::Eof
        } else {
            ReaddirEnd::Error(errno)
        }
    }

    /// Set `errno` to 0 so a following `readdir` NULL can be classified.
    /// Per-platform, because libc exposes the thread's `errno` location under
    /// different names.
    fn clear_errno() {
        #[cfg(any(target_os = "macos", target_os = "ios", target_os = "freebsd"))]
        // SAFETY: `__error()` returns the calling thread's errno location.
        unsafe {
            *libc::__error() = 0;
        }
        #[cfg(any(target_os = "linux", target_os = "android"))]
        // SAFETY: `__errno_location()` returns the calling thread's errno location.
        unsafe {
            *libc::__errno_location() = 0;
        }
    }

    /// Is `name`, looked up in the pinned directory WITHOUT following links,
    /// a regular file?
    pub(super) fn is_regular_file(dir: &OwnedFd, name: &str) -> bool {
        let Ok(c) = cstring(name) else {
            return false;
        };
        // SAFETY: zeroed `stat` is a valid out-parameter for `fstatat`.
        let mut st: libc::stat = unsafe { std::mem::zeroed() };
        // SAFETY: `dir` is an open directory descriptor, `c` a valid name,
        // `st` a valid out-parameter; `AT_SYMLINK_NOFOLLOW` stats the link
        // itself.
        let rc = unsafe {
            libc::fstatat(
                dir.as_raw_fd(),
                c.as_ptr(),
                &mut st,
                libc::AT_SYMLINK_NOFOLLOW,
            )
        };
        rc == 0 && (st.st_mode & libc::S_IFMT) == libc::S_IFREG
    }

    /// Remove the ENTRY `name` from the pinned directory — never a path.
    pub(super) fn unlink(dir: &OwnedFd, name: &str) -> Result<(), std::io::Error> {
        let c =
            cstring(name).map_err(|e| std::io::Error::new(std::io::ErrorKind::InvalidInput, e))?;
        // SAFETY: `dir` is an open directory descriptor and `c` a valid name;
        // flags 0 removes a non-directory entry and follows nothing.
        if unsafe { libc::unlinkat(dir.as_raw_fd(), c.as_ptr(), 0) } == 0 {
            Ok(())
        } else {
            Err(std::io::Error::last_os_error())
        }
    }
}

/// List the pinned node directory and prove EVERY entry is an orphan port
/// tag: a regular file (never a symlink — `fstatat` with
/// `AT_SYMLINK_NOFOLLOW`, so nothing is followed) named
/// `<prefix><decimal port id><suffix>`. Returns the port ids with their
/// entry names, or the offending names.
#[cfg(unix)]
fn orphan_tags_in(
    dir: &std::os::fd::OwnedFd,
    prefix: &str,
    suffix: &str,
) -> Result<Vec<(u128, String)>, String> {
    let mut tags = Vec::new();
    let mut offenders = Vec::new();
    for name in pinned::list(dir)? {
        let is_regular_file = pinned::is_regular_file(dir, &name);
        let port_id = name
            .strip_prefix(prefix)
            .and_then(|rest| rest.strip_suffix(suffix))
            .filter(|digits| !digits.is_empty() && digits.chars().all(|c| c.is_ascii_digit()))
            .and_then(|digits| digits.parse::<u128>().ok());
        match port_id {
            Some(port_id) if is_regular_file => tags.push((port_id, name)),
            _ => offenders.push(name),
        }
    }
    if !offenders.is_empty() {
        offenders.sort();
        return Err(format!(
            "the node directory holds {} entr{} that {} not orphan port tag{} — refusing to touch \
             it: {}",
            offenders.len(),
            if offenders.len() == 1 { "y" } else { "ies" },
            if offenders.len() == 1 { "is" } else { "are" },
            if offenders.len() == 1 { "" } else { "s" },
            offenders
                .iter()
                .map(|n| format!("`{n}`"))
                .collect::<Vec<_>>()
                .join(", ")
        ));
    }
    tags.sort();
    Ok(tags)
}

/// Remove the orphan port tags of each candidate — or, under `dry_run`, only
/// say which would be removed — and return one [`OrphanTagReclaim`] per
/// candidate, in order.
///
/// Three guards, each a refusal that is REPORTED, never silent:
///
/// * **The process is gone.** `verdict(pid, created_at)` must answer
///   [`CreatorVerdict::Gone`] — the ONE liveness predicate the `.shm_state`
///   reclamation trusts, reached through `shm_state::creator_verdict`
///   (`kill(pid, 0)` → `ESRCH`; a creation stamp later than now is not
///   evidence). `Alive` is refused (a pid reused by an unrelated process is
///   refused too — the safe direction), and so is `Unknown`.
/// * **The directory is where the registry says it is.** Re-checked here,
///   not only at selection, so a hand-built candidate cannot aim the
///   reclaimer outside `<config.global.node_dir()>`.
/// * **Every entry is an orphan tag, RIGHT NOW, in THAT directory.** The
///   directory is pinned by DESCRIPTOR (the registry root reached directory
///   by directory, each component `O_DIRECTORY|O_NOFOLLOW` and a link
///   followed only where root planted it; the node directory `openat`ed from
///   it by bare name — a symbolic link or a non-directory in its place is
///   refused and named), listed through that descriptor immediately before anything is
///   removed, never from the sweep's memory of it, and every removal is an
///   `unlinkat` on it — a path check cannot bind an identity, and a candidate
///   can appear (and its directory change) between the sweep that classified
///   it and this call (measured on a development machine, where a fresh
///   orphan-tag directory was minted by an unrelated session while this was
///   being written). Anything that is not a regular file (links included —
///   `fstatat` with `AT_SYMLINK_NOFOLLOW`) named `<prefix><port id><suffix>`
///   refuses the WHOLE directory and is named.
///
/// The directory itself is left standing: the next sweep's `remove_node` is
/// what removes it, and its `cleanups == 1` is the proof the shape is healed.
/// Removal stops at the first I/O error and reports it with what was removed
/// before it. A directory that is ALREADY empty when listed is
/// [`ReclaimVerdict::AlreadyEmpty`] — nothing to remove, nothing to refuse,
/// converged pending that same sweep — which is why a caller must run its
/// subsequent sweep whenever candidates EXISTED, not only when a tag came off.
pub fn reclaim_orphan_port_tags(
    candidates: &[OrphanTagNode],
    config: &Config,
    dry_run: bool,
    verdict: &dyn Fn(u32, Option<u64>) -> CreatorVerdict,
) -> Vec<OrphanTagReclaim> {
    let prefix = String::from_utf8_lossy(config.global.prefix.as_bytes()).into_owned();
    let suffix =
        String::from_utf8_lossy(config.global.node.port_tag_suffix.as_bytes()).into_owned();
    candidates
        .iter()
        .map(|candidate| {
            let mut outcome = OrphanTagReclaim {
                node: candidate.node.clone(),
                node_id: candidate.node_id,
                pid: candidate.pid,
                dir: candidate.dir.clone(),
                removed: Vec::new(),
                verdict: ReclaimVerdict::Reclaimed,
            };
            match verdict(candidate.pid, parse_creation_unix_s(&candidate.node)) {
                CreatorVerdict::Gone => {}
                CreatorVerdict::Alive => {
                    outcome.verdict = ReclaimVerdict::Refused(format!(
                        "process {} is still alive — a live process's tags are never touched",
                        candidate.pid
                    ));
                    return outcome;
                }
                CreatorVerdict::Unknown => {
                    outcome.verdict = ReclaimVerdict::Refused(format!(
                        "could not determine whether process {} is alive — nothing is removed on \
                         an unproven death",
                        candidate.pid
                    ));
                    return outcome;
                }
            }
            if !same_path(&candidate.dir, &node_details_dir(config, candidate.node_id)) {
                outcome.verdict = ReclaimVerdict::Refused(format!(
                    "`{}` is not this node's directory under the configured registry ({})",
                    candidate.dir.display(),
                    node_details_dir(config, candidate.node_id).display()
                ));
                return outcome;
            }
            #[cfg(not(unix))]
            {
                let _ = (&prefix, &suffix, dry_run);
                outcome.verdict = ReclaimVerdict::Refused(
                    "the orphan port-tag reclaim pins directories by descriptor and needs a Unix \
                     registry — nothing is removed on this platform"
                        .to_string(),
                );
                outcome
            }
            #[cfg(unix)]
            {
                // IDENTITY, not path: the root is opened without following
                // links, the node directory is opened FROM it by bare name,
                // and every later operation runs on that descriptor.
                let root = match pinned::open_root(&String::from(&config.global.node_dir())) {
                    Ok(root) => root,
                    Err(reason) => {
                        outcome.verdict = ReclaimVerdict::Refused(reason);
                        return outcome;
                    }
                };
                let dir = match pinned::open_node_dir(&root, &candidate.node_id.to_string()) {
                    Ok(dir) => dir,
                    Err(pinned::PinError::Vanished) => {
                        outcome.verdict = ReclaimVerdict::Refused(
                            "the node directory no longer exists (another sweep reclaimed it, or \
                             it was removed by hand)"
                                .to_string(),
                        );
                        return outcome;
                    }
                    Err(pinned::PinError::NotADirectory) => {
                        outcome.verdict = ReclaimVerdict::Refused(format!(
                            "`{}` is not a directory of the registry any more (a symbolic link, or \
                             replaced by something that is not a directory) — refusing to touch it",
                            candidate.dir.display()
                        ));
                        return outcome;
                    }
                    Err(pinned::PinError::Other(err)) => {
                        outcome.verdict = ReclaimVerdict::Refused(format!(
                            "the node directory could not be listed: {err}"
                        ));
                        return outcome;
                    }
                };
                let tags = match orphan_tags_in(&dir, &prefix, &suffix) {
                    Ok(tags) => tags,
                    Err(reason) => {
                        outcome.verdict = ReclaimVerdict::Refused(reason);
                        return outcome;
                    }
                };
                if tags.is_empty() {
                    // Not a refusal: nothing to remove and nothing to refuse.
                    // The next sweep's `remove_node` finishes it.
                    outcome.verdict = ReclaimVerdict::AlreadyEmpty;
                    return outcome;
                }
                for (port_id, name) in tags {
                    if !dry_run {
                        if let Err(err) = pinned::unlink(&dir, &name) {
                            outcome.verdict = ReclaimVerdict::Refused(format!(
                                "removing `{}` failed: {err}",
                                candidate.dir.join(&name).display()
                            ));
                            return outcome;
                        }
                    }
                    outcome.removed.push(port_id);
                }
                outcome
            }
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ipc_cleanup::classify_cleanup_failures;
    use crate::ipc_cleanup::tests::{detected, line, node_token, refusal, sub_cause, view_origin};
    use cerulion_core::iceoryx_logger::CapturedLog;
    use iceoryx2::prelude::{FileName, LogLevel, Path as IoxPath};

    // ── Orphan port tags: candidate selection over hand-built captures, and the
    //    reclaimer over a tempdir with an injected liveness oracle ─────────────

    /// A config rooted at `root` with the given file prefix — what the pin's
    /// child and parent build too, so the classifier is exercised against a
    /// `node_dir()` iceoryx2 itself derives rather than a string the test
    /// happens to agree with.
    fn rooted_config(root: &str, prefix: &str) -> Config {
        let mut cfg = Config::default();
        cfg.global
            .set_root_path(&IoxPath::new(root.as_bytes()).expect("iceoryx2 root path"));
        cfg.global.prefix = FileName::new(prefix.as_bytes()).expect("iceoryx2 prefix");
        cfg
    }

    fn node_dir_string(cfg: &Config, node_id: u128) -> String {
        node_details_dir(cfg, node_id).display().to_string()
    }

    /// Line 1 of the chain — the only line that carries the directory, from a
    /// string origin with NO node token (attributed by adjacency).
    fn rmdir_line(dir: &str) -> CapturedLog {
        line(
            LogLevel::Debug,
            "Directory::remove_empty()",
            format!("{ORPHAN_TAG_RMDIR_PREFIX}{dir}{ORPHAN_TAG_RMDIR_SUFFIX}"),
        )
    }

    /// Line 2 — `remove_path_hint`'s origin renders the path, never the token.
    fn path_hint_line(dir: &str) -> CapturedLog {
        line(
            LogLevel::Debug,
            format!("remove_path_hint(Path {{ value: \"{dir}\" }})"),
            ORPHAN_TAG_PATH_HINT_LINE,
        )
    }

    /// Line 3 — a string origin that DOES embed the node token.
    fn details_dir_line(node: &str) -> CapturedLog {
        line(
            LogLevel::Debug,
            format!("remove_node_details_directory(Config {{ .. }}, {node})"),
            ORPHAN_TAG_DETAILS_DIR_LINE,
        )
    }

    /// Line 4 — `fail!(from self, ..)` on the `DeadNodeView`.
    fn node_itself_line(node: &str) -> CapturedLog {
        line(
            LogLevel::Debug,
            view_origin(node),
            ORPHAN_TAG_NODE_ITSELF_LINE,
        )
    }

    /// The context file `ProcessMonitor::state()` probes for a node —
    /// `<node_dir>/<prefix><value>.node_monitor_context` under the default
    /// suffix.
    fn monitor_context_path(cfg: &Config, node_id: u128) -> String {
        let prefix = String::from_utf8_lossy(cfg.global.prefix.as_bytes()).into_owned();
        let suffix =
            String::from_utf8_lossy(cfg.global.node.monitor_suffix.as_bytes()).into_owned();
        format!(
            "{}/{prefix}{node_id}{suffix}_context",
            String::from(&cfg.global.node_dir())
        )
    }

    /// The two probe lines `ProcessMonitor::state()` logs per probe — string
    /// origins, NO node token (attributed by adjacency, as on a real sweep).
    fn monitor_probe_lines(cfg: &Config, node_id: u128) -> Vec<CapturedLog> {
        let path = monitor_context_path(cfg, node_id);
        vec![
            line(
                LogLevel::Debug,
                "FileBuilder::open_existing()",
                MONITOR_PROBE_OPEN_LINE,
            ),
            line(
                LogLevel::Debug,
                format!("ProcessMonitor {{ context_path: \"{path}\" }}"),
                format!("{MONITOR_PROBE_STATE_PREFIX}{path}{MONITOR_PROBE_STATE_SUFFIX}"),
            ),
        ]
    }

    /// The whole chain for one node exactly as the real sweep emits it
    /// (MEASURED, isolated root, macOS): the probe pair from `Node::list`'s
    /// classification, the detected line, the probe pair again from the
    /// cleaner acquisition, the four `rmdir` lines, the refusal.
    fn orphan_tag_chain(cfg: &Config, node_id: u128) -> Vec<CapturedLog> {
        let node = node_token(node_id);
        let dir = node_dir_string(cfg, node_id);
        let mut captured = monitor_probe_lines(cfg, node_id);
        captured.push(detected(&node));
        captured.extend(monitor_probe_lines(cfg, node_id));
        captured.extend([
            rmdir_line(&dir),
            path_hint_line(&dir),
            details_dir_line(&node),
            node_itself_line(&node),
            refusal(&node, "InternalError"),
        ]);
        captured
    }

    /// The chain as a RE-SWEEP emits it — the `.details` storage is already
    /// gone, so `Node::list`'s details open logs the static-storage line first
    /// (MEASURED, verbatim from the pin's dry-run arm on a second sweep).
    fn resweep_orphan_tag_chain(cfg: &Config, node_id: u128) -> Vec<CapturedLog> {
        let node = node_token(node_id);
        let dir = node_dir_string(cfg, node_id);
        let mut captured = vec![
            line(
                LogLevel::Debug,
                "FileBuilder::open_existing()",
                FILE_MISSING_OPEN_LINE,
            ),
            line(
                LogLevel::Debug,
                "static_storage::File::Builder::open()",
                STATIC_STORAGE_OPEN_LINE,
            ),
        ];
        captured.extend(monitor_probe_lines(cfg, node_id));
        captured.push(detected(&node));
        captured.extend(monitor_probe_lines(cfg, node_id));
        captured.extend([
            rmdir_line(&dir),
            path_hint_line(&dir),
            details_dir_line(&node),
            node_itself_line(&node),
            refusal(&node, "InternalError"),
        ]);
        captured
    }

    /// The chain WITHOUT the probe lines — what a future iceoryx2 that stops
    /// logging them (or a capture that missed them) would yield.
    fn bare_orphan_tag_chain(cfg: &Config, node_id: u128) -> Vec<CapturedLog> {
        let node = node_token(node_id);
        let dir = node_dir_string(cfg, node_id);
        vec![
            detected(&node),
            rmdir_line(&dir),
            path_hint_line(&dir),
            details_dir_line(&node),
            node_itself_line(&node),
            refusal(&node, "InternalError"),
        ]
    }

    fn candidates_of(captured: &[CapturedLog], cfg: &Config) -> Vec<OrphanTagNode> {
        orphan_port_tag_candidates(&classify_cleanup_failures(captured).failures, cfg)
    }

    /// The exact chain, through the REAL attribution (`classify_cleanup_failures`
    /// — the probe lines and lines 1 and 2 arrive by adjacency, 3 and 4 by
    /// id) and then the selector: exactly one candidate, carrying the node's
    /// value, pid and the directory line 1 quoted. The measured shape carries
    /// the probe pair twice; the bare chain (no probe lines at all) must
    /// select too.
    #[test]
    fn the_exact_orphan_tag_chain_selects_the_node() {
        let cfg = rooted_config("/tmp/iceoryx2/orphan_unit/", "orphan_");
        let expected = vec![OrphanTagNode {
            node: node_token(4242),
            node_id: 4242,
            pid: 77,
            dir: PathBuf::from(node_dir_string(&cfg, 4242)),
        }];

        let captured = orphan_tag_chain(&cfg, 4242);
        let parts = classify_cleanup_failures(&captured);
        assert_eq!(
            parts.failures[0].causes.len(),
            8,
            "the attribution hands the node its four probe lines and the four rmdir lines: {:?}",
            parts.failures
        );
        assert_eq!(
            candidates_of(&captured, &cfg),
            expected,
            "the measured shape"
        );
        assert_eq!(
            candidates_of(&bare_orphan_tag_chain(&cfg, 4242), &cfg),
            expected,
            "the bare chain"
        );
    }

    /// The RE-SWEEP shape, replayed VERBATIM from the pin's dry-run arm on a
    /// second sweep — the two dead nodes that were under that root, in the
    /// captured order and with the captured messages — must yield BOTH as
    /// candidates; the same capture with a `stale resources of the port` line
    /// inserted into the first node's attempt must yield only the second.
    /// This is the shape EVERY `cerulion clean` after the first sees on a
    /// robot carrying the defect: a selector written to the first-sweep shape
    /// alone heals nothing.
    #[test]
    fn the_verbatim_resweep_capture_selects_both_nodes() {
        let cfg = rooted_config(
            "/tmp/iceoryx2/orphan_6507_1788926827433141000/",
            "orphan_6507_",
        );
        let (a, b) = (
            (14857966915340887985112488300u128, 6508u32),
            (244818821161215785235151591808u128, 6528u32),
        );
        let mut captured = resweep_orphan_tag_chain(&cfg, a.0);
        captured.extend(resweep_orphan_tag_chain(&cfg, b.0));
        // The messages are the captured ones, byte for byte.
        let parts = classify_cleanup_failures(&captured);
        assert_eq!(parts.failures.len(), 2, "{:?}", parts.failures);
        assert_eq!(
            parts.failures[0].causes,
            vec![
                "Unable to open file since it does not exist.".to_string(),
                "Unable to open static storage due to a failure while opening the file.".to_string(),
                "Unable to open file due to insufficient permissions.".to_string(),
                "Unable to open ProcessMonitor state file \"/tmp/iceoryx2/orphan_6507_1788926827433141000/nodes/orphan_6507_14857966915340887985112488300.node_monitor_context\" with access mode Write due to insufficient permissions.".to_string(),
                "Unable to open file due to insufficient permissions.".to_string(),
                "Unable to open ProcessMonitor state file \"/tmp/iceoryx2/orphan_6507_1788926827433141000/nodes/orphan_6507_14857966915340887985112488300.node_monitor_context\" with access mode Write due to insufficient permissions.".to_string(),
                "Unable to remove empty directory \"/tmp/iceoryx2/orphan_6507_1788926827433141000/nodes/14857966915340887985112488300\" since the directory is not empty or there are hard links pointing to the directory.".to_string(),
                "Unable to remove path hint due to an internal error (NotEmptyOrHardLinksPointingToTheDirectory).".to_string(),
                "Unable to remove node details directory due to an internal error.".to_string(),
                "Unable to remove stale resources since the node itself could not be removed.".to_string(),
            ],
            "the replay must reproduce the captured chain byte for byte"
        );

        let candidates = orphan_port_tag_candidates(&parts.failures, &cfg);
        assert_eq!(
            candidates
                .iter()
                .map(|c| (c.node_id, c.pid))
                .collect::<Vec<_>>(),
            vec![a, b],
            "{candidates:?}"
        );

        // The same capture, with a port-resource failure inside node A's
        // attempt: A is refused, B still selects.
        let port_line = sub_cause(
            &node_token(a.0),
            "stale resources of the port PortId(9) could not be removed due to an internal failure.",
        );
        let mut poisoned = captured.clone();
        poisoned.insert(7, port_line);
        let parts = classify_cleanup_failures(&poisoned);
        let candidates = orphan_port_tag_candidates(&parts.failures, &cfg);
        assert_eq!(
            candidates.iter().map(|c| c.pid).collect::<Vec<_>>(),
            vec![b.1],
            "{candidates:?}"
        );
    }

    /// The desk capture that caught the fourth shape, replayed VERBATIM: a
    /// real orphan-tag node (pid gone, one tag in the directory) refused on a
    /// re-sweep under the attribution that recognises `since it …` lines —
    /// the ENOENT open of the missing `.details` file leads the chain. The
    /// convergence e2e (`clean_happy_path_reports_nothing_to_clean`) failed on
    /// exactly this: the reclaim never ran, the second `cerulion clean` still
    /// listed the node. MUST select.
    #[test]
    fn the_verbatim_desk_capture_with_the_missing_details_file_line_selects() {
        let cfg = rooted_config("/tmp/iceoryx2/", "iox2_");
        let node = "UniqueNodeId(UniqueSystemId { value: 3490140064757085183321206494843, pid: 71291, creation_time: Time { clock_type: Realtime, seconds: 1789043908, nanoseconds: 222311000 } })";
        let l = |msg: &str| line(LogLevel::Debug, "probe", msg.to_string());
        let captured = vec![
            l("Unable to open file since it does not exist."),
            l("Unable to open static storage due to a failure while opening the file."),
            l("Unable to open file due to insufficient permissions."),
            l("Unable to open ProcessMonitor state file \"/tmp/iceoryx2/nodes/iox2_3490140064757085183321206494843.node_monitor_context\" with access mode Write due to insufficient permissions."),
            detected(node),
            l("Unable to open file due to insufficient permissions."),
            l("Unable to open ProcessMonitor state file \"/tmp/iceoryx2/nodes/iox2_3490140064757085183321206494843.node_monitor_context\" with access mode Write due to insufficient permissions."),
            l("Unable to remove empty directory \"/tmp/iceoryx2/nodes/3490140064757085183321206494843\" since the directory is not empty or there are hard links pointing to the directory."),
            l("Unable to remove path hint due to an internal error (NotEmptyOrHardLinksPointingToTheDirectory)."),
            line(
                LogLevel::Debug,
                format!("remove_node_details_directory(Config {{ .. }}, {node})"),
                "Unable to remove node details directory due to an internal error.",
            ),
            line(
                LogLevel::Debug,
                view_origin(node),
                "Unable to remove stale resources since the node itself could not be removed.",
            ),
            refusal(node, "InternalError"),
        ];
        let parts = classify_cleanup_failures(&captured);
        assert_eq!(parts.failures.len(), 1, "{:?}", parts.failures);
        assert_eq!(
            parts.failures[0].causes.len(),
            10,
            "the attribution hands the node all ten lines: {:?}",
            parts.failures[0].causes
        );
        let candidates = orphan_port_tag_candidates(&parts.failures, &cfg);
        assert_eq!(
            candidates
                .iter()
                .map(|c| (c.node_id, c.pid))
                .collect::<Vec<_>>(),
            vec![(3490140064757085183321206494843u128, 71291u32)],
            "{candidates:?}"
        );
        assert!(candidates[0]
            .dir
            .components()
            .eq(Path::new("/tmp/iceoryx2/nodes/3490140064757085183321206494843").components()));
    }

    /// Every disqualifier fragment, inserted into an otherwise exact chain,
    /// refuses — pinned one by one, so a widening of the allow-list cannot
    /// quietly admit one.
    #[test]
    fn every_disqualifier_refuses_an_otherwise_exact_chain() {
        let cfg = rooted_config("/tmp/iceoryx2/orphan_unit/", "orphan_");
        let node = node_token(4242);
        for fragment in ORPHAN_TAG_DISQUALIFIERS {
            let mut captured = resweep_orphan_tag_chain(&cfg, 4242);
            captured.insert(
                7,
                sub_cause(&node, &format!("something about the {fragment} happened.")),
            );
            assert!(
                candidates_of(&captured, &cfg).is_empty(),
                "`{fragment}` must disqualify"
            );
        }
        // The control: the unpoisoned re-sweep chain selects.
        assert_eq!(
            candidates_of(&resweep_orphan_tag_chain(&cfg, 4242), &cfg).len(),
            1
        );
    }

    /// The probe run is bounded to EXACTLY the three probe shapes, in the lead:
    /// a neighbour's probe (foreign context path) is tolerated as the noise it
    /// is; a probe line AFTER the chain, a probe with another access mode, and
    /// a probe of a file that is not a monitor context all refuse.
    #[test]
    fn the_monitor_probe_run_admits_only_the_two_probe_shapes_before_the_chain() {
        let cfg = rooted_config("/tmp/iceoryx2/orphan_unit/", "orphan_");
        let node = node_token(4242);
        let dir = node_dir_string(&cfg, 4242);
        let chain = |lead: Vec<CapturedLog>, tail: Vec<CapturedLog>| {
            let mut c = lead;
            c.push(detected(&node));
            c.extend([
                rmdir_line(&dir),
                path_hint_line(&dir),
                details_dir_line(&node),
                node_itself_line(&node),
            ]);
            c.extend(tail);
            c.push(refusal(&node, "InternalError"));
            c
        };

        // A neighbour's probe pair (another node's context file) in the lead.
        let foreign = chain(monitor_probe_lines(&cfg, 999), vec![]);
        assert_eq!(
            candidates_of(&foreign, &cfg).len(),
            1,
            "a neighbour's probe is noise"
        );

        // The probe pair AFTER the chain: the tail is not the four lines.
        let after = chain(vec![], monitor_probe_lines(&cfg, 4242));
        assert!(
            candidates_of(&after, &cfg).is_empty(),
            "a probe after the chain"
        );

        // A probe with another access mode is a REAL permission problem.
        let path = monitor_context_path(&cfg, 4242);
        let read_probe = line(
            LogLevel::Debug,
            "ProcessMonitor",
            format!(
                "Unable to open ProcessMonitor state file \"{path}\" with access mode Read due to insufficient permissions."
            ),
        );
        let wrong_mode = chain(vec![read_probe], vec![]);
        assert!(
            candidates_of(&wrong_mode, &cfg).is_empty(),
            "another access mode"
        );

        // A Write probe of something that is not a monitor context file.
        let not_context = line(
            LogLevel::Debug,
            "ProcessMonitor",
            format!(
                "{MONITOR_PROBE_STATE_PREFIX}{dir}/orphan_4242.details{MONITOR_PROBE_STATE_SUFFIX}"
            ),
        );
        let wrong_file = chain(vec![not_context], vec![]);
        assert!(
            candidates_of(&wrong_file, &cfg).is_empty(),
            "not a monitor context file"
        );

        // Any other "insufficient permissions" line in the lead is a different
        // shape (a real permission fault), never tolerated.
        let other = line(
            LogLevel::Debug,
            "Node::open_node_storage()",
            "Unable to open node storage due to insufficient permissions.",
        );
        let other_lead = chain(vec![other], vec![]);
        assert!(
            candidates_of(&other_lead, &cfg).is_empty(),
            "an unrelated permission line"
        );

        // The control: the measured shape selects.
        assert_eq!(
            candidates_of(&orphan_tag_chain(&cfg, 4242), &cfg).len(),
            1
        );
    }

    /// A refusal that ALSO failed a port's stale-resource reclaim is a
    /// different shape: the safety argument (both tag passes succeeded) does
    /// not hold, so it must not be a candidate even though the `rmdir` lines
    /// are all there.
    #[test]
    fn a_port_resource_failure_in_the_chain_is_not_a_candidate() {
        let cfg = rooted_config("/tmp/iceoryx2/orphan_unit/", "orphan_");
        let mut captured = orphan_tag_chain(&cfg, 4242);
        let port_line = sub_cause(
            &node_token(4242),
            "stale resources of the port PortId(9) could not be removed due to an internal failure.",
        );
        captured.insert(1, port_line);

        assert!(
            candidates_of(&captured, &cfg).is_empty(),
            "a chain carrying a port-resource failure must not be selected"
        );
    }

    /// Two nodes INTERLEAVED (the discriminating shape for the id arm): the
    /// one with the exact chain is selected, the one refused for permissions
    /// is not, and the selected one's directory is its OWN.
    #[test]
    fn two_interleaved_nodes_select_only_the_one_with_the_exact_chain() {
        let cfg = rooted_config("/tmp/iceoryx2/orphan_unit/", "orphan_");
        let (a, b) = (node_token(1), node_token(2));
        let dir_a = node_dir_string(&cfg, 1);
        let captured = vec![
            detected(&a),
            detected(&b),
            rmdir_line(&dir_a),
            sub_cause(
                &b,
                "stale resources of the port PortId(3) could not be removed due to insufficient permissions.",
            ),
            path_hint_line(&dir_a),
            details_dir_line(&a),
            node_itself_line(&a),
            refusal(&a, "InternalError"),
            refusal(&b, "InsufficientPermissions"),
        ];

        let candidates = candidates_of(&captured, &cfg);

        assert_eq!(candidates.len(), 1, "{candidates:?}");
        assert_eq!(candidates[0].node_id, 1);
        assert_eq!(candidates[0].pid, 10);
        assert!(same_path(&candidates[0].dir, Path::new(&dir_a)));
    }

    /// A directory that is not `<config.global.node_dir()>/<id>` is not this
    /// registry's node — whatever the lines say, the reclaimer must never be
    /// pointed outside the configured root.
    #[test]
    fn a_directory_outside_the_configured_registry_is_not_a_candidate() {
        let cfg = rooted_config("/tmp/iceoryx2/orphan_unit/", "orphan_");
        let node = node_token(4242);
        let foreign = "/somewhere/else/nodes/4242";
        let captured = vec![
            detected(&node),
            rmdir_line(foreign),
            path_hint_line(foreign),
            details_dir_line(&node),
            node_itself_line(&node),
            refusal(&node, "InternalError"),
        ];
        assert!(candidates_of(&captured, &cfg).is_empty());

        // Same directory NAME under a different root — the id matches, the
        // root does not.
        let other_root = rooted_config("/tmp/iceoryx2/orphan_other/", "orphan_");
        let captured = orphan_tag_chain(&other_root, 4242);
        assert!(
            candidates_of(&captured, &cfg).is_empty(),
            "a node under another root must not be selected against this config"
        );
        // …and IS a candidate against its own root (the control).
        assert_eq!(candidates_of(&captured, &other_root).len(), 1);
    }

    /// The selector is EXACT: a missing line, an extra benign line, another
    /// variant on the same lines, or a token without a pid all refuse.
    #[test]
    fn anything_but_the_exact_chain_is_not_a_candidate() {
        let cfg = rooted_config("/tmp/iceoryx2/orphan_unit/", "orphan_");
        let node = node_token(4242);
        let dir = node_dir_string(&cfg, 4242);

        // Three of the four lines.
        let short = vec![
            detected(&node),
            rmdir_line(&dir),
            path_hint_line(&dir),
            node_itself_line(&node),
            refusal(&node, "InternalError"),
        ];
        assert!(candidates_of(&short, &cfg).is_empty(), "a missing line");

        // The four lines plus an extra sub-cause about the service.
        let mut long = orphan_tag_chain(&cfg, 4242);
        long.insert(
            1,
            sub_cause(
                &node,
                "service itself is corrupted. Trying to remove the corrupted remainders of the service.",
            ),
        );
        assert!(candidates_of(&long, &cfg).is_empty(), "an extra line");

        // The same lines under another variant.
        let mut other_variant = orphan_tag_chain(&cfg, 4242);
        other_variant.pop();
        other_variant.push(refusal(&node, "InsufficientPermissions"));
        assert!(
            candidates_of(&other_variant, &cfg).is_empty(),
            "another variant"
        );

        // A token the identity parser cannot read (no pid).
        let bare = "UniqueNodeId(4242)";
        let unreadable = vec![
            rmdir_line(&dir),
            path_hint_line(&dir),
            line(
                LogLevel::Debug,
                format!("remove_node_details_directory(Config {{ .. }}, {bare})"),
                ORPHAN_TAG_DETAILS_DIR_LINE,
            ),
            line(
                LogLevel::Debug,
                view_origin(bare),
                ORPHAN_TAG_NODE_ITSELF_LINE,
            ),
            refusal(bare, "InternalError"),
        ];
        assert!(
            candidates_of(&unreadable, &cfg).is_empty(),
            "an unreadable identity"
        );

        // The control: the exact chain still selects.
        assert_eq!(
            candidates_of(&orphan_tag_chain(&cfg, 4242), &cfg).len(),
            1
        );
    }

    #[test]
    fn the_creation_stamp_parser_reads_realtime_seconds_and_nothing_else() {
        assert_eq!(
            parse_creation_unix_s(
                "UniqueNodeId(UniqueSystemId { value: 1, pid: 2, creation_time: Time { clock_type: Realtime, seconds: 1788909140, nanoseconds: 473229000 } })"
            ),
            Some(1788909140)
        );
        assert_eq!(
            parse_creation_unix_s(&node_token(1)),
            None,
            "a Monotonic stamp cannot be compared to wall time"
        );
        assert_eq!(parse_creation_unix_s("UniqueNodeId(4242)"), None);
        assert_eq!(
            parse_creation_unix_s("UniqueNodeId(UniqueSystemId { value: 1, pid: 2, creation_time: Time { clock_type: Realtime, seconds: x } })"),
            None
        );
    }

    #[test]
    fn the_node_identity_parser_reads_value_and_pid_and_nothing_else() {
        assert_eq!(parse_node_identity(&node_token(4242)), Some((4242, 77)));
        assert_eq!(
            parse_node_identity(
                "UniqueNodeId(UniqueSystemId { value: 8517378255348516775436287364, pid: 64900, creation_time: Time { clock_type: Monotonic, seconds: 1, nanoseconds: 2 } })"
            ),
            Some((8517378255348516775436287364, 64900))
        );
        assert_eq!(parse_node_identity("UniqueNodeId(4242)"), None);
        assert_eq!(
            parse_node_identity("UniqueNodeId(UniqueSystemId { value: x, pid: 1 })"),
            None
        );
        assert_eq!(
            parse_node_identity("UniqueNodeId(UniqueSystemId { value: 1 })"),
            None,
            "no pid"
        );
    }

    // ── The reclaimer ──

    /// A tempdir registry: `<root>/nodes/<id>/` with the given entries planted.
    struct PlantedNode {
        _root: tempfile::TempDir,
        cfg: Config,
        candidate: OrphanTagNode,
    }

    const UNIT_PREFIX: &str = "orphan_";

    fn plant(node_id: u128, pid: u32, tags: &[u128]) -> PlantedNode {
        let root = tempfile::tempdir().expect("tempdir");
        let cfg = rooted_config(&format!("{}/", root.path().display()), UNIT_PREFIX);
        let dir = node_details_dir(&cfg, node_id);
        std::fs::create_dir_all(&dir).expect("node dir");
        for tag in tags {
            std::fs::write(dir.join(format!("{UNIT_PREFIX}{tag}.port_tag")), b"").expect("tag");
        }
        PlantedNode {
            _root: root,
            cfg,
            candidate: OrphanTagNode {
                node: node_token(node_id),
                node_id,
                pid,
                dir,
            },
        }
    }

    fn names_in(dir: &Path) -> Vec<String> {
        let mut names: Vec<String> = std::fs::read_dir(dir)
            .map(|entries| {
                entries
                    .flatten()
                    .map(|e| e.file_name().to_string_lossy().into_owned())
                    .collect()
            })
            .unwrap_or_default();
        names.sort();
        names
    }

    fn gone(_pid: u32, _created_at: Option<u64>) -> CreatorVerdict {
        CreatorVerdict::Gone
    }

    /// A directory holding NOTHING but well-formed tags of a gone process:
    /// every tag removed, ids reported in order, the directory itself left for
    /// the next sweep's `remove_node`.
    #[test]
    fn reclaim_removes_every_tag_of_a_directory_holding_only_tags() {
        let planted = plant(4242, 77, &[22, 1, 300]);
        let out = reclaim_orphan_port_tags(
            std::slice::from_ref(&planted.candidate),
            &planted.cfg,
            false,
            &gone,
        );

        assert_eq!(out.len(), 1);
        assert_eq!(out[0].refused(), None, "{out:?}");
        assert_eq!(out[0].removed, vec![1, 22, 300]);
        assert_eq!((out[0].node_id, out[0].pid), (4242, 77));
        assert!(
            planted.candidate.dir.is_dir(),
            "the directory is left standing"
        );
        assert!(
            names_in(&planted.candidate.dir).is_empty(),
            "every tag removed"
        );
    }

    /// ANY other entry refuses the WHOLE directory, names every offender, and
    /// removes nothing — not even the genuine tags beside them. A symlink named
    /// exactly like a tag is an offender too (nothing is followed).
    #[cfg(unix)]
    #[test]
    fn reclaim_refuses_a_directory_holding_anything_else_and_names_every_offender() {
        let planted = plant(4242, 77, &[5]);
        let dir = &planted.candidate.dir;
        std::fs::write(dir.join("stray.txt"), b"x").expect("stray");
        std::fs::create_dir(dir.join("sub")).expect("subdir");
        std::os::unix::fs::symlink(
            dir.join("stray.txt"),
            dir.join(format!("{UNIT_PREFIX}9.port_tag")),
        )
        .expect("symlink");
        let before = names_in(dir);

        let out = reclaim_orphan_port_tags(
            std::slice::from_ref(&planted.candidate),
            &planted.cfg,
            false,
            &gone,
        );

        let refused = out[0].refused().expect("must refuse");
        for offender in [
            "`stray.txt`",
            "`sub`",
            &format!("`{UNIT_PREFIX}9.port_tag`"),
        ] {
            assert!(refused.contains(offender), "{refused}");
        }
        assert!(
            !refused.contains(&format!("`{UNIT_PREFIX}5.port_tag`")),
            "the genuine tag is not an offender: {refused}"
        );
        assert!(out[0].removed.is_empty());
        assert_eq!(names_in(dir), before, "nothing removed");
    }

    /// IDENTITY, not path: the node directory swapped for a SYMBOLIC LINK to a
    /// directory OUTSIDE the registry that holds a perfectly shaped tag — a
    /// path-driven reclaim would follow it and delete that file. Refused,
    /// naming the link; the outside tag survives; the link is left alone.
    #[cfg(unix)]
    #[test]
    fn a_node_directory_swapped_for_a_symlink_is_refused_and_the_outside_tag_survives() {
        let planted = plant(4242, 77, &[8]);
        let outside = tempfile::tempdir().expect("outside");
        let outside_dir = outside.path().join("elsewhere");
        std::fs::create_dir_all(&outside_dir).expect("outside dir");
        let outside_tag = outside_dir.join(format!("{UNIT_PREFIX}8.port_tag"));
        std::fs::write(&outside_tag, b"").expect("outside tag");
        // The swap: the real node directory is gone, a link stands in its place.
        std::fs::remove_dir_all(&planted.candidate.dir).expect("remove the real dir");
        std::os::unix::fs::symlink(&outside_dir, &planted.candidate.dir).expect("symlink");

        let out = reclaim_orphan_port_tags(
            std::slice::from_ref(&planted.candidate),
            &planted.cfg,
            false,
            &gone,
        );

        let refused = out[0].refused().expect("a swapped directory must refuse");
        assert!(
            refused.contains("symbolic link")
                && refused.contains(&planted.candidate.dir.display().to_string()),
            "{refused}"
        );
        assert!(out[0].removed.is_empty(), "{out:?}");
        assert!(outside_tag.is_file(), "the outside tag must survive");
        assert!(
            std::fs::symlink_metadata(&planted.candidate.dir)
                .map(|m| m.file_type().is_symlink())
                .unwrap_or(false),
            "the link itself is left alone"
        );
    }

    /// The registry PATH is walked directory by directory. A symbolic link at
    /// an INTERMEDIATE component — which a single `O_NOFOLLOW` `open()` on
    /// the whole path follows silently, refusing only a link at the LAST one
    /// — is refused unless root planted it. A user-owned link standing in for
    /// the registry's parent, aimed at a real registry elsewhere: the reclaim
    /// must refuse naming that link, remove nothing, and the tag out there
    /// must survive; the same registry by its real path is the control.
    /// (Every tempdir this file uses sits under macOS's root-owned
    /// `/var → private/var` link, so the follow-root's-links half is
    /// exercised by every reclaim test in it.)
    #[cfg(unix)]
    #[test]
    fn a_user_owned_symlink_at_an_intermediate_component_of_the_registry_path_is_refused() {
        let base = tempfile::tempdir().expect("tempdir");
        let real = base.path().join("real");
        let link = base.path().join("link");
        std::fs::create_dir_all(&real).expect("the real root");
        std::os::unix::fs::symlink(&real, &link).expect("plant the link");
        // A config rooted THROUGH the link; the node directory and its tag
        // are created through it too, so they really live under `real`.
        let cfg = rooted_config(&format!("{}/", link.display()), UNIT_PREFIX);
        let node_dir = node_details_dir(&cfg, 4242);
        std::fs::create_dir_all(&node_dir).expect("node dir through the link");
        std::fs::write(node_dir.join(format!("{UNIT_PREFIX}8.port_tag")), b"").expect("tag");
        let real_tag = real
            .join("nodes")
            .join("4242")
            .join(format!("{UNIT_PREFIX}8.port_tag"));
        assert!(
            real_tag.is_file(),
            "precondition: the tag lives under the real root"
        );
        let candidate = OrphanTagNode {
            node: node_token(4242),
            node_id: 4242,
            pid: 77,
            dir: node_dir,
        };

        let out = reclaim_orphan_port_tags(std::slice::from_ref(&candidate), &cfg, false, &gone);

        let refused = out[0]
            .refused()
            .expect("a user-owned link in the registry path must refuse");
        assert!(
            refused.contains("symbolic link") && refused.contains(&link.display().to_string()),
            "the refusal names the link component: {refused}"
        );
        assert!(refused.contains("not by root"), "{refused}");
        assert!(out[0].removed.is_empty(), "{out:?}");
        assert!(real_tag.is_file(), "the tag under the real root survives");

        // The control: the same registry by its real path reclaims.
        let cfg = rooted_config(&format!("{}/", real.display()), UNIT_PREFIX);
        let candidate = OrphanTagNode {
            dir: node_details_dir(&cfg, 4242),
            ..candidate
        };
        let out = reclaim_orphan_port_tags(std::slice::from_ref(&candidate), &cfg, false, &gone);
        assert_eq!(out[0].refused(), None, "{out:?}");
        assert_eq!(out[0].removed, vec![8]);
        assert!(!real_tag.exists(), "reclaimed by the real path");
    }

    /// A tag ENTRY that is a symbolic link — even one named exactly like a
    /// tag and pointing at a real tag — is an offender: nothing is followed,
    /// the whole directory is refused, and both the real tag and the link's
    /// target survive.
    #[cfg(unix)]
    #[test]
    fn a_tag_entry_that_is_a_symlink_is_an_offender_and_nothing_is_followed() {
        let planted = plant(4242, 77, &[8]);
        let dir = &planted.candidate.dir;
        let target = tempfile::tempdir().expect("target");
        let target_file = target.path().join(format!("{UNIT_PREFIX}9.port_tag"));
        std::fs::write(&target_file, b"").expect("target file");
        std::os::unix::fs::symlink(&target_file, dir.join(format!("{UNIT_PREFIX}9.port_tag")))
            .expect("symlink");
        let before = names_in(dir);

        let out = reclaim_orphan_port_tags(
            std::slice::from_ref(&planted.candidate),
            &planted.cfg,
            false,
            &gone,
        );

        let refused = out[0].refused().expect("must refuse");
        assert!(
            refused.contains(&format!("`{UNIT_PREFIX}9.port_tag`")),
            "{refused}"
        );
        assert!(
            !refused.contains(&format!("`{UNIT_PREFIX}8.port_tag`")),
            "{refused}"
        );
        assert!(out[0].removed.is_empty());
        assert_eq!(names_in(dir), before, "nothing removed");
        assert!(target_file.is_file(), "the link's target must survive");
    }

    /// The listing loop's one decision on a NULL `readdir`: `errno == 0` is
    /// end-of-directory, anything else an incomplete listing — pinned on the
    /// pure helper the loop calls, and on a REAL directory listed right after
    /// a syscall left `errno` stale (the death guard's `kill(pid, 0)` leaves
    /// `ESRCH` behind on every reclaim): without clearing `errno` before each
    /// `readdir`, the end of a perfectly healthy directory would read as that
    /// stale error and every reclaim would refuse.
    #[cfg(unix)]
    #[test]
    fn a_null_readdir_is_eof_only_when_errno_is_zero_and_a_stale_errno_is_cleared_first() {
        use super::pinned::{classify_readdir_end, ReaddirEnd};
        assert_eq!(classify_readdir_end(0), ReaddirEnd::Eof);
        assert_eq!(
            classify_readdir_end(libc::EBADF),
            ReaddirEnd::Error(libc::EBADF)
        );
        assert_eq!(
            classify_readdir_end(libc::EIO),
            ReaddirEnd::Error(libc::EIO)
        );

        let planted = plant(4242, 77, &[8, 9]);
        // Leave a STALE errno behind (ENOENT), the way the death guard leaves
        // ESRCH, immediately before the listing.
        let _ = std::fs::metadata("/nonexistent/orphan/stale-errno");
        assert_eq!(
            std::io::Error::last_os_error().raw_os_error(),
            Some(libc::ENOENT),
            "precondition: errno really is stale before the listing"
        );
        let out = reclaim_orphan_port_tags(
            std::slice::from_ref(&planted.candidate),
            &planted.cfg,
            true,
            &gone,
        );
        assert_eq!(
            out[0].refused(),
            None,
            "a healthy directory lists completely: {out:?}"
        );
        assert_eq!(out[0].removed, vec![8, 9]);
    }

    /// Tag names that are not `<prefix><digits><suffix>` are offenders: a
    /// foreign prefix, a non-numeric id, an empty id.
    #[test]
    fn reclaim_refuses_tags_that_are_not_prefix_digits_suffix() {
        for bad in [
            "other_1.port_tag",
            "orphan_abc.port_tag",
            "orphan_.port_tag",
            "orphan_7.service_tag",
        ] {
            let planted = plant(1, 2, &[3]);
            std::fs::write(planted.candidate.dir.join(bad), b"").expect("bad");
            let before = names_in(&planted.candidate.dir);
            let out = reclaim_orphan_port_tags(
                std::slice::from_ref(&planted.candidate),
                &planted.cfg,
                false,
                &gone,
            );
            let refused = out[0]
                .refused()
                .unwrap_or_else(|| panic!("`{bad}` must refuse"));
            assert!(refused.contains(&format!("`{bad}`")), "{refused}");
            assert_eq!(
                names_in(&planted.candidate.dir),
                before,
                "`{bad}`: nothing removed"
            );
        }
    }

    #[test]
    fn dry_run_lists_what_would_be_removed_and_removes_nothing() {
        let planted = plant(4242, 77, &[8, 9]);
        let before = names_in(&planted.candidate.dir);

        let out = reclaim_orphan_port_tags(
            std::slice::from_ref(&planted.candidate),
            &planted.cfg,
            true,
            &gone,
        );

        assert_eq!(out[0].refused(), None);
        assert_eq!(out[0].removed, vec![8, 9], "listed, not removed");
        assert_eq!(names_in(&planted.candidate.dir), before);
    }

    /// The death guard: a live pid and an unprovable one BOTH refuse, before
    /// the directory is even listed — the tags survive untouched.
    #[test]
    fn an_alive_or_unproven_process_refuses_the_reclaim_before_anything_is_touched() {
        for (verdict, word) in [
            (CreatorVerdict::Alive, "still alive"),
            (CreatorVerdict::Unknown, "could not determine"),
        ] {
            let planted = plant(4242, 77, &[8]);
            let before = names_in(&planted.candidate.dir);
            let out = reclaim_orphan_port_tags(
                std::slice::from_ref(&planted.candidate),
                &planted.cfg,
                false,
                &|_pid, _created| verdict,
            );
            let refused = out[0].refused().expect("must refuse");
            assert!(
                refused.contains(word) && refused.contains("77"),
                "{refused}"
            );
            assert!(out[0].removed.is_empty());
            assert_eq!(names_in(&planted.candidate.dir), before);
        }
    }

    /// A candidate whose directory vanished between the sweep and the reclaim
    /// is REFUSED (reported, not guessed); an EMPTY one is `AlreadyEmpty` —
    /// not a refusal, nothing removed, the directory left for the next
    /// sweep's `remove_node`. The two are pinned apart because a caller that
    /// read "already empty" as a refusal would skip the sweep that converges
    /// it.
    #[test]
    fn a_vanished_or_empty_directory_is_reported_not_guessed() {
        let planted = plant(4242, 77, &[8]);
        std::fs::remove_dir_all(&planted.candidate.dir).expect("vanish");
        let out = reclaim_orphan_port_tags(
            std::slice::from_ref(&planted.candidate),
            &planted.cfg,
            false,
            &gone,
        );
        assert!(
            out[0]
                .refused()
                .is_some_and(|r| r.contains("no longer exists")),
            "{out:?}"
        );

        let planted = plant(4242, 77, &[]);
        for dry_run in [false, true] {
            let out = reclaim_orphan_port_tags(
                std::slice::from_ref(&planted.candidate),
                &planted.cfg,
                dry_run,
                &gone,
            );
            assert_eq!(out[0].verdict, ReclaimVerdict::AlreadyEmpty, "{out:?}");
            assert_eq!(out[0].refused(), None, "not a refusal: {out:?}");
            assert!(out[0].removed.is_empty(), "{out:?}");
            assert!(
                planted.candidate.dir.is_dir(),
                "left for the next sweep's `remove_node`"
            );
        }
    }

    /// A hand-built candidate aimed OUTSIDE `<config.global.node_dir()>` is
    /// refused without listing anything — the registry check is re-done at
    /// reclaim time, not trusted from selection.
    #[test]
    fn a_candidate_outside_the_configured_registry_is_refused_at_reclaim_time() {
        let planted = plant(4242, 77, &[8]);
        let elsewhere = tempfile::tempdir().expect("elsewhere");
        let decoy = elsewhere.path().join("4242");
        std::fs::create_dir_all(&decoy).expect("decoy");
        std::fs::write(decoy.join(format!("{UNIT_PREFIX}8.port_tag")), b"").expect("decoy tag");
        let candidate = OrphanTagNode {
            dir: decoy.clone(),
            ..planted.candidate.clone()
        };

        let out = reclaim_orphan_port_tags(&[candidate], &planted.cfg, false, &gone);

        assert!(
            out[0]
                .refused()
                .is_some_and(|r| r.contains("not this node's directory")),
            "{out:?}"
        );
        assert_eq!(names_in(&decoy), vec![format!("{UNIT_PREFIX}8.port_tag")]);
    }

    /// Several candidates are handled independently, in order, each with its
    /// own verdict.
    #[test]
    fn candidates_are_reclaimed_independently_and_in_order() {
        let root = tempfile::tempdir().expect("tempdir");
        let cfg = rooted_config(&format!("{}/", root.path().display()), UNIT_PREFIX);
        let mut candidates = Vec::new();
        for (id, tags, stray) in [
            (1u128, vec![10u128], false),
            (2, vec![20, 21], true),
            (3, vec![30], false),
        ] {
            let dir = node_details_dir(&cfg, id);
            std::fs::create_dir_all(&dir).expect("dir");
            for tag in &tags {
                std::fs::write(dir.join(format!("{UNIT_PREFIX}{tag}.port_tag")), b"").expect("tag");
            }
            if stray {
                std::fs::write(dir.join("stray"), b"").expect("stray");
            }
            candidates.push(OrphanTagNode {
                node: node_token(id),
                node_id: id,
                pid: 1,
                dir,
            });
        }

        let out = reclaim_orphan_port_tags(&candidates, &cfg, false, &gone);

        assert_eq!(
            out.iter().map(|o| o.node_id).collect::<Vec<_>>(),
            vec![1, 2, 3]
        );
        assert_eq!(
            (out[0].removed.clone(), out[0].refused().is_none()),
            (vec![10], true)
        );
        assert_eq!(
            (out[1].removed.clone(), out[1].refused().is_some()),
            (vec![], true)
        );
        assert_eq!(
            (out[2].removed.clone(), out[2].refused().is_none()),
            (vec![30], true)
        );
        assert_eq!(
            names_in(&candidates[1].dir),
            vec![
                format!("{UNIT_PREFIX}20.port_tag"),
                format!("{UNIT_PREFIX}21.port_tag"),
                "stray".to_string()
            ]
        );
    }
}
