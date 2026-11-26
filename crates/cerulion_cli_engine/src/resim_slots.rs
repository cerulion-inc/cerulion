// SPDX-License-Identifier: AGPL-3.0-only
//! Private shared-memory namespace slots for `cerulion bag play --resim`.
//!
//! Each resim runs on one of [`RESIM_NAMESPACE_SLOTS`] fixed-width hex
//! prefixes (`cer_resim_<slot:08x>_`), serialized by an exclusive `flock` on
//! `cer_resim_<slot:08x>.lock` in [`resim_slot_lock_dir`]. Keeping the slot
//! constants, lock verification, and sweep logic here lets `replay_engine`,
//! `cerulion clean`, `cerulion doctor`, and `graph run` exit hygiene share one
//! definition of the namespace layout and lock contract.

use std::path::{Path, PathBuf};
use std::time::Duration;

#[cfg(unix)]
use std::collections::BTreeSet;
#[cfg(unix)]
use std::time::Instant;

use iceoryx2::config::Config;

use crate::ipc_cleanup::SweepMode;

#[cfg(unix)]
use iceoryx2::node::{Node, NodeState};
#[cfg(unix)]
use iceoryx2::prelude::CallbackProgression;
#[cfg(unix)]
use iceoryx2::service::ipc_threadsafe::Service as CerService;

/// The fixed head of every resim's PRIVATE iceoryx2 prefix
/// (`{RESIM_NAMESPACE_MARKER}{slot:08x}_`, see [`resim_namespace_prefix`]).
///
/// A resim publishes the frames it re-executes and the recorded external
/// inputs it re-injects under the recording's own topic names, and its
/// transport runs under this prefix. iceoryx2 names every node, service and
/// shared-memory segment `{prefix}{name}` and opens only files carrying its own
/// prefix, so a process on the shared namespace (iceoryx2's default `iox2_`,
/// the one `graph run`, `cerulion-netd`, `bagd` and `topic echo` use) cannot
/// open a service a resim creates, and a resim cannot open theirs.
///
/// It diverges from `iox2_` at its first character and from the multi-process
/// planning namespace's `cer_p_` at its fifth, so iceoryx2's string-prefix
/// listing of one namespace never reaches another's files.
pub const RESIM_NAMESPACE_MARKER: &str = "cer_resim_";

/// How many resim namespaces a machine has. A resim runs on the slot it holds,
/// so at most this many resims run at once, and the files resims leave behind
/// belong to at most this many namespaces.
pub const RESIM_NAMESPACE_SLOTS: u32 = 256;

/// The private iceoryx2 prefix of resim namespace slot `slot`.
///
/// The slot is rendered as exactly eight lowercase hex digits (every `u32`
/// fits), so every resim prefix has the same length and none is a strict
/// prefix of another: iceoryx2 lists a namespace's nodes by string-prefix
/// match, and prefixes of different lengths would let one slot's listing reach
/// another's files. One process at a time holds a slot, so two resims running
/// at once never share a namespace.
///
/// Nothing the run records or reports carries this prefix: it names files, and
/// a topic's iceoryx2 service identity is derived from the topic alone.
#[must_use]
pub fn resim_namespace_prefix(slot: u32) -> String {
    format!("{RESIM_NAMESPACE_MARKER}{slot:08x}_")
}

/// The resim namespace slot whose [`resim_namespace_prefix`] `file_name`
/// begins with.
///
/// `None` for every other name: other namespaces' files, and a slot's lock
/// file (`cer_resim_{slot:08x}.lock`, no `_` after the digits).
#[must_use]
pub fn resim_namespace_slot(file_name: &str) -> Option<u32> {
    let rest = file_name.strip_prefix(RESIM_NAMESPACE_MARKER)?;
    let hex = rest.get(..8)?;
    if !rest.get(8..)?.starts_with('_')
        || !hex.bytes().all(|b| matches!(b, b'0'..=b'9' | b'a'..=b'f'))
    {
        return None;
    }
    u32::from_str_radix(hex, 16).ok()
}

/// The iceoryx2 config of resim namespace slot `slot`: `base` with the private
/// [`resim_namespace_prefix`]. A resim passes the host's RESOLVED config
/// (`Config::global_config()`), so an operator's iceoryx2 config file, its root
/// path included, applies exactly as it does to a live run.
pub(crate) fn resim_ix_config(base: &Config, slot: u32) -> Result<Config, String> {
    use iceoryx2::prelude::{FileName, SemanticString};
    let prefix = resim_namespace_prefix(slot);
    let file_name = FileName::new(prefix.as_bytes()).map_err(|e| {
        format!(
            "cannot build the resim's private iceoryx2 prefix '{prefix}': iceoryx2 rejects it \
             ({e:?})"
        )
    })?;
    let mut ix_config = base.clone();
    ix_config.global.prefix = file_name;
    Ok(ix_config)
}

/// The directory that holds this machine's shared-memory segment names:
/// `/dev/shm` on Linux, and on macOS the directory of the state files iceoryx2
/// keeps one per segment. Segment names do not depend on the iceoryx2 root, so
/// every namespace with one prefix shares its segment names here, whatever root
/// its config names. `None` where a resim does not walk segment names.
#[cfg(target_os = "linux")]
pub(crate) const SEGMENT_NAME_DIR: Option<&str> = Some("/dev/shm/");
#[cfg(target_os = "macos")]
pub(crate) const SEGMENT_NAME_DIR: Option<&str> = Some(crate::shm_state::SHM_STATE_DIRECTORY);
#[cfg(not(any(target_os = "linux", target_os = "macos")))]
pub(crate) const SEGMENT_NAME_DIR: Option<&str> = None;

/// The directory that holds this machine's shared-memory segment names, if any.
#[must_use]
pub fn resim_segment_dir() -> Option<&'static Path> {
    SEGMENT_NAME_DIR.map(Path::new)
}

/// The directory a resim takes its slot lock in: [`resim_segment_dir`], or
/// `base`'s iceoryx2 root where there is none.
///
/// The lock sits where a slot's segment names collide, so its exclusion covers
/// every process that could share them: two resims whose iceoryx2 configs name
/// different roots, and two containers that share `/dev/shm` but not `/tmp`,
/// still take different slots, and a slot's reclaim never removes a segment a
/// running resim uses.
#[must_use]
pub fn resim_slot_lock_dir(base: &Config) -> PathBuf {
    SEGMENT_NAME_DIR.map_or_else(
        || PathBuf::from(String::from(base.global.root_path())),
        PathBuf::from,
    )
}

/// What a sweep across lockable resim namespace slots found and did.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct ResimSweepReport {
    /// Slots whose lock file existed and was locked for inspection or cleanup.
    pub slots_checked: u64,
    /// Slots skipped because a running resim holds the lock (`WOULDBLOCK`).
    pub slots_in_use: u64,
    /// Dead nodes classified across lockable resim slots.
    pub dead_nodes: u64,
    /// Dead nodes successfully reaped (`SweepMode::Remove` only).
    pub cleaned_nodes: u64,
    /// Dead-node cleanup attempts that failed (`SweepMode::Remove` only).
    pub failed_node_cleanups: u64,
    /// Dead nodes deferred when a wall budget ran out (`SweepMode::Remove` with budget).
    pub deferred_nodes: u64,
    /// Registry-listing failures encountered across lockable resim slots.
    pub registry_errors: u64,
    /// Leftover slot segments seen on lockable slots with no live or unknown nodes.
    pub leftover_segments: u64,
    /// Leftover slot segments removed (`SweepMode::Remove` only).
    pub segments_removed: u64,
    /// Leftover slot segments that could not be removed (`SweepMode::Remove` only).
    pub segments_failed: u64,
    /// Errors encountered while locking slots, listing slot registries,
    /// listing the segment directory, or removing segments.
    pub errors: Vec<String>,
    /// Whether a wall-clock budget cut the sweep short.
    pub truncated: bool,
}

impl ResimSweepReport {
    /// Did every lockable resim slot's node registry converge with no refused,
    /// deferred, or unlistable dead nodes?
    ///
    /// Used by `cerulion clean` and graceful exit hygiene to gate macOS
    /// `.shm_state` reclamation alongside the default-namespace convergence check.
    #[must_use]
    pub fn nodes_converged(&self) -> bool {
        self.failed_node_cleanups == 0
            && self.deferred_nodes == 0
            && self.registry_errors == 0
            && !self.truncated
    }

    /// Total failure count across node cleanups, segment removals, and
    /// lock/registry/directory errors, without double-counting segment
    /// failures that are also recorded in [`Self::errors`].
    #[must_use]
    pub fn total_failures(&self) -> u64 {
        self.failed_node_cleanups
            .saturating_add(self.segments_failed.max(self.errors.len() as u64))
    }
}

/// A resim namespace slot this process holds: its lock file stays open, and
/// its exclusive `flock` held, until the value is dropped or the process
/// ends, a crash included (the kernel releases the lock with the descriptor).
#[cfg(unix)]
#[derive(Debug)]
pub(crate) struct ResimSlot {
    pub(crate) slot: u32,
    /// Kept only to hold the lock.
    pub(crate) _lock: SlotLock,
}

/// A slot's lock file with its exclusive `flock` held, released on drop.
///
/// A `flock` lock belongs to the open file description, not to one
/// descriptor. A child this process forks holds a copy of every descriptor
/// until it execs, so closing the file alone would leave the lock held for as
/// long as such a copy lives. Drop therefore unlocks the description first,
/// which releases the lock whatever copies exist, and then closes the file.
#[cfg(unix)]
#[derive(Debug)]
pub(crate) struct SlotLock(pub(crate) std::fs::File);

#[cfg(unix)]
impl Drop for SlotLock {
    fn drop(&mut self) {
        if let Err(error) = rustix::fs::flock(&self.0, rustix::fs::FlockOperation::Unlock) {
            tracing::warn!(
                error = %error,
                "replay: could not unlock a resim namespace slot before closing its lock file; \
                 the slot stays held until every copy of the descriptor is closed"
            );
        }
    }
}

/// Why a claim passed over a slot whose lock a running resim does not hold.
#[cfg(unix)]
#[derive(Debug)]
pub(crate) enum SlotPassedOver {
    /// Its lock file could not be opened, verified (for example a symlink, a
    /// non-regular file, or a file owned by another user), or locked: the path
    /// and the error.
    Lock(String),
    /// The slot's private iceoryx2 config could not be built.
    Config(String),
    /// Its namespace did not reclaim clean: what the reclaim found, and
    /// [`SlotReclaim::refusal`].
    Reclaim {
        reclaim: SlotReclaim,
        reason: String,
    },
}

#[cfg(unix)]
impl std::fmt::Display for SlotPassedOver {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Lock(failure) => {
                write!(f, "its lock file could not be opened or locked: {failure}")
            }
            Self::Config(reason) | Self::Reclaim { reason, .. } => f.write_str(reason),
        }
    }
}

/// Refuse a slot lock file whose `fstat` owner uid does not match
/// `expected_euid`.
#[cfg(unix)]
pub(crate) fn check_slot_lock_owner(
    path: &Path,
    file_uid: u32,
    expected_euid: u32,
) -> Result<(), String> {
    if file_uid == expected_euid {
        Ok(())
    } else {
        Err(format!(
            "{}: lock file is owned by uid {file_uid}, not current uid {expected_euid}",
            path.display()
        ))
    }
}

/// Outcome of trying to lock an existing slot lock file without creating it.
#[cfg(unix)]
enum ExistingLockAttempt {
    Acquired(SlotLock),
    Absent,
    HeldByLiveResim,
}

/// Open an existing slot lock file read-only with `O_NOFOLLOW | O_NONBLOCK`,
/// verify it is a regular file owned by `expected_euid`, and try a non-blocking
/// exclusive `flock`. `O_NONBLOCK` prevents an attacker-planted FIFO from
/// blocking `open(2)` before `fstat` can reject it.
#[cfg(unix)]
fn try_lock_existing_slot(path: &Path, expected_euid: u32) -> Result<ExistingLockAttempt, String> {
    use std::os::unix::fs::{MetadataExt, OpenOptionsExt};
    let lock = match std::fs::OpenOptions::new()
        .read(true)
        .write(false)
        .create(false)
        .custom_flags(libc::O_NOFOLLOW | libc::O_NONBLOCK)
        .open(path)
    {
        Ok(file) => file,
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => {
            return Ok(ExistingLockAttempt::Absent);
        }
        Err(e) => return Err(format!("{}: {e}", path.display())),
    };
    let meta = lock
        .metadata()
        .map_err(|e| format!("{}: {e}", path.display()))?;
    if !meta.file_type().is_file() {
        return Err(format!(
            "{}: lock file is not a regular file",
            path.display()
        ));
    }
    check_slot_lock_owner(path, meta.uid(), expected_euid)?;
    match rustix::fs::flock(&lock, rustix::fs::FlockOperation::NonBlockingLockExclusive) {
        Ok(()) => Ok(ExistingLockAttempt::Acquired(SlotLock(lock))),
        Err(e) if e == rustix::io::Errno::WOULDBLOCK => Ok(ExistingLockAttempt::HeldByLiveResim),
        Err(e) => Err(format!("{}: {e}", path.display())),
    }
}

/// Log one slot a claim passed over, through the claim's `latch`: the first
/// at warn, the repeats at debug, and the slot that takes the count across a
/// power of ten at warn again.
#[cfg(unix)]
fn log_passed_over_slot(
    latch: &mut cerulion_core::transport::failure_regime_latch::FailureRegimeLatch,
    slot: u32,
    passed: &SlotPassedOver,
) {
    use cerulion_core::transport::failure_regime_latch::RegimeDecision;
    let reclaim = match passed {
        SlotPassedOver::Reclaim { reclaim, .. } => Some(reclaim),
        SlotPassedOver::Lock(_) | SlotPassedOver::Config(_) => None,
    };
    let nodes_left = reclaim.and_then(|r| r.nodes_left.as_ref().ok().copied());
    let cleanups = reclaim.map(|r| r.cleaned.cleanups);
    let failed_cleanups = reclaim.map(|r| r.cleaned.failed_cleanups);
    let deferred = reclaim.map(|r| r.cleaned.deferred);
    let segments_failed = reclaim.map(|r| r.segments_failed);
    match latch.on_failure() {
        RegimeDecision::Loud => tracing::warn!(
            slot,
            nodes_left,
            cleanups,
            failed_cleanups,
            deferred,
            segments_failed,
            error = %passed,
            "replay: passed over a resim namespace slot whose lock could not be taken or whose \
             namespace did not reclaim clean; the slot keeps what could not be removed and the \
             resim tries the next one"
        ),
        RegimeDecision::StillFailing { total, suppressed } => tracing::warn!(
            slot,
            nodes_left,
            cleanups,
            failed_cleanups,
            deferred,
            segments_failed,
            total_failures = total,
            suppressed,
            error = %passed,
            "replay: still passing over resim namespace slots"
        ),
        RegimeDecision::Suppressed { suppressed } => tracing::debug!(
            slot,
            nodes_left,
            cleanups,
            failed_cleanups,
            deferred,
            segments_failed,
            suppressed,
            error = %passed,
            "replay: passed over another resim namespace slot (suppressed)"
        ),
    }
}

/// Claim the lowest slot below `slots` whose lock file in `lock_dir`
/// (`cer_resim_{slot:08x}.lock`, created with mode `0600`) this process can
/// lock with an exclusive non-blocking `flock`, and that `ready` accepts once
/// the lock is held.
#[cfg(unix)]
pub(crate) fn claim_resim_slot(
    lock_dir: &Path,
    slots: u32,
    mut ready: impl FnMut(u32) -> Result<(), SlotPassedOver>,
) -> Result<ResimSlot, String> {
    use std::os::unix::fs::{MetadataExt, OpenOptionsExt};
    let expected_euid = cerulion_hygiene::euid();
    let mut latch = cerulion_core::transport::failure_regime_latch::FailureRegimeLatch::new();
    let mut first_failure: Option<String> = None;
    let mut last_refusal: Option<(u32, SlotPassedOver)> = None;
    for slot in 0..slots {
        let path = lock_dir.join(format!("{RESIM_NAMESPACE_MARKER}{slot:08x}.lock"));
        let opened = std::fs::OpenOptions::new()
            .read(true)
            .write(true)
            .create(true)
            .truncate(false)
            .mode(0o600)
            .custom_flags(libc::O_NOFOLLOW | libc::O_NONBLOCK)
            .open(&path)
            .map_err(|e| format!("{}: {e}", path.display()))
            .and_then(|lock| {
                let meta = lock
                    .metadata()
                    .map_err(|e| format!("{}: {e}", path.display()))?;
                if !meta.file_type().is_file() {
                    return Err(format!(
                        "{}: lock file is not a regular file",
                        path.display()
                    ));
                }
                check_slot_lock_owner(&path, meta.uid(), expected_euid)?;
                match rustix::fs::flock(&lock, rustix::fs::FlockOperation::NonBlockingLockExclusive)
                {
                    Ok(()) => Ok(Some(SlotLock(lock))),
                    Err(e) if e == rustix::io::Errno::WOULDBLOCK => Ok(None),
                    Err(e) => Err(format!("{}: {e}", path.display())),
                }
            });
        let lock = match opened {
            Ok(Some(lock)) => lock,
            // Held by a running resim.
            Ok(None) => continue,
            Err(failure) => {
                first_failure.get_or_insert_with(|| failure.clone());
                log_passed_over_slot(&mut latch, slot, &SlotPassedOver::Lock(failure));
                continue;
            }
        };
        match ready(slot) {
            Ok(()) => {
                if let Some(suppressed) = latch.on_success() {
                    tracing::info!(
                        slot,
                        suppressed,
                        total_failures = latch.total_failures(),
                        "replay: claimed a resim namespace slot after passing over others"
                    );
                }
                return Ok(ResimSlot { slot, _lock: lock });
            }
            Err(passed) => {
                log_passed_over_slot(&mut latch, slot, &passed);
                last_refusal = Some((slot, passed));
            }
        }
    }
    Err(format!(
        "no resim namespace slot is free: none of the {slots} slot locks in '{}' could be \
         taken by this process (each is held by a running resim, could not be opened or \
         locked, or its namespace did not reclaim clean){}{}",
        lock_dir.display(),
        first_failure
            .map(|failure| format!("; the first that could not be opened or locked: {failure}"))
            .unwrap_or_default(),
        last_refusal
            .map(|(slot, passed)| format!(
                "; the last whose namespace was refused: slot {slot}, {passed}"
            ))
            .unwrap_or_default()
    ))
}

/// What reclaiming one slot's namespace found and did.
#[cfg(unix)]
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct SlotReclaim {
    /// The dead-node sweep's counts.
    pub(crate) cleaned: cerulion_core::transport::dead_node_sweep::BoundedCleanup,
    /// Nodes still registered after the sweep, or why the registry could not
    /// be listed.
    pub(crate) nodes_left: Result<u64, String>,
    /// Leftover segments removed.
    pub(crate) segments_removed: u64,
    /// Leftover segments that could not be removed.
    pub(crate) segments_failed: u64,
    /// Error text from the first leftover segment that could not be removed.
    pub(crate) first_segment_error: Option<String>,
}

#[cfg(unix)]
impl SlotReclaim {
    /// Why a resim may not start on the slot, or `None` when it may: its
    /// namespace holds no node, and every leftover segment the walk reached
    /// was removed.
    pub(crate) fn refusal(&self) -> Option<String> {
        let cleaned = &self.cleaned;
        match &self.nodes_left {
            Err(error) => Some(format!(
                "its namespace's node registry could not be listed after the dead-node sweep \
                 ({error})"
            )),
            Ok(0) if self.segments_failed == 0 => None,
            Ok(0) => Some(match &self.first_segment_error {
                Some(first) => format!(
                    "leftover segments of its namespace that could not be removed: {} (first \
                     error: {first})",
                    self.segments_failed
                ),
                None => format!(
                    "leftover segments of its namespace that could not be removed: {}",
                    self.segments_failed
                ),
            }),
            Ok(left) => Some(format!(
                "nodes still registered in its namespace after the dead-node sweep: {left} (the \
                 sweep removed {}, failed on {} and deferred {}; it removes only the nodes of \
                 processes that have ended and that this user can inspect)",
                cleaned.cleanups, cleaned.failed_cleanups, cleaned.deferred
            )),
        }
    }
}

/// Whether `path` is a segment file that belongs to a reclaimed namespace
/// (a regular file, and on macOS a `.shm_state` file).
#[cfg(unix)]
fn is_segment_file(path: &Path) -> bool {
    let is_file = std::fs::symlink_metadata(path).is_ok_and(|meta| meta.file_type().is_file());
    #[cfg(target_os = "macos")]
    let is_segment = is_file
        && path
            .file_name()
            .and_then(|name| name.to_str())
            .is_some_and(|name| name.ends_with(crate::shm_state::SHM_STATE_SUFFIX));
    #[cfg(not(target_os = "macos"))]
    let is_segment = is_file;
    is_segment
}

/// What [`remove_leftover_segment`] did with one file.
#[cfg(unix)]
#[derive(Debug, Clone, PartialEq, Eq)]
enum SegmentRemoval {
    Removed,
    /// Not a segment (a directory or a link, or on macOS anything but a state
    /// file), so it was left alone.
    NotASegment,
    /// Removal failed, with the underlying error message.
    Failed(String),
}

/// Remove one leftover segment of a reclaimed namespace: a regular file, which
/// on macOS is a state file and goes with the object it names
/// ([`crate::shm_state::remove_unused_state_file`]).
#[cfg(unix)]
fn remove_leftover_segment(path: &Path) -> SegmentRemoval {
    if !is_segment_file(path) {
        return SegmentRemoval::NotASegment;
    }
    #[cfg(target_os = "macos")]
    let result = crate::shm_state::remove_unused_state_file(path, &crate::shm_state::LibcProbe);
    #[cfg(not(target_os = "macos"))]
    let result = std::fs::remove_file(path).map_err(|e| format!("{}: {e}", path.display()));
    match result {
        Ok(()) => SegmentRemoval::Removed,
        Err(error) => SegmentRemoval::Failed(error),
    }
}

/// Reclaim the namespace of a slot this process holds, within `budget`,
/// before a resim starts on it.
#[cfg(unix)]
pub(crate) fn reclaim_resim_slot(
    ix_config: &Config,
    slot: u32,
    segment_dir: Option<&Path>,
    budget: Duration,
) -> SlotReclaim {
    let started = Instant::now();
    let cleaned =
        cerulion_core::transport::dead_node_sweep::cleanup_dead_nodes_bounded(ix_config, budget);
    let mut nodes = 0u64;
    let listed = Node::<CerService>::list(ix_config, |_| {
        nodes += 1;
        CallbackProgression::Continue
    });
    let mut reclaim = SlotReclaim {
        cleaned,
        nodes_left: listed.map(|()| nodes).map_err(|e| e.to_string()),
        segments_removed: 0,
        segments_failed: 0,
        first_segment_error: None,
    };
    if reclaim.nodes_left != Ok(0) {
        return reclaim;
    }
    let Some(dir) = segment_dir else {
        return reclaim;
    };
    let entries = match std::fs::read_dir(dir) {
        Ok(entries) => entries,
        Err(error) => {
            tracing::warn!(
                slot,
                error = %error,
                "replay: could not list the shared-memory segment names to reclaim a resim \
                 namespace slot; its leftover segments stay until a later resim on the slot \
                 lists them"
            );
            return reclaim;
        }
    };
    for entry in entries {
        if started.elapsed() >= budget {
            break;
        }
        let entry = match entry {
            Ok(entry) => entry,
            Err(error) => {
                reclaim.segments_failed += 1;
                reclaim
                    .first_segment_error
                    .get_or_insert_with(|| format!("{}: {error}", dir.display()));
                continue;
            }
        };
        if entry.file_name().to_str().and_then(resim_namespace_slot) != Some(slot) {
            continue;
        }
        match remove_leftover_segment(&entry.path()) {
            SegmentRemoval::Removed => reclaim.segments_removed += 1,
            SegmentRemoval::NotASegment => {}
            SegmentRemoval::Failed(error) => {
                reclaim.segments_failed += 1;
                reclaim.first_segment_error.get_or_insert(error);
            }
        }
    }
    reclaim
}

/// Claim and reclaim a resim namespace slot against explicit `lock_dir` and
/// `segment_dir` paths, bounded by `budget`.
#[cfg(unix)]
pub(crate) fn claim_and_reclaim_resim_slot(
    base: &Config,
    lock_dir: &Path,
    segment_dir: Option<&Path>,
    slots: u32,
    budget: Duration,
) -> Result<ResimSlot, String> {
    let started = Instant::now();
    let _ = std::fs::create_dir_all(lock_dir);
    let (mut cleanups, mut deferred, mut segments, mut total_failures) = (0u64, 0u64, 0u64, 0u64);
    let claimed = claim_resim_slot(lock_dir, slots, |slot| {
        let ix_config = resim_ix_config(base, slot).map_err(SlotPassedOver::Config)?;
        let reclaim = reclaim_resim_slot(
            &ix_config,
            slot,
            segment_dir,
            budget.saturating_sub(started.elapsed()),
        );
        cleanups += reclaim.cleaned.cleanups;
        deferred += reclaim.cleaned.deferred;
        segments += reclaim.segments_removed;
        total_failures += reclaim.cleaned.failed_cleanups + reclaim.segments_failed;
        match reclaim.refusal() {
            None => Ok(()),
            Some(reason) => Err(SlotPassedOver::Reclaim { reclaim, reason }),
        }
    })?;
    tracing::debug!(
        slot = claimed.slot,
        cleanups,
        deferred,
        segments,
        total_failures,
        "replay: claimed and reclaimed a resim namespace slot"
    );
    Ok(claimed)
}

/// Flush one batch of locked slots against `segment_dir` in a single `read_dir`.
/// Holding every slot's `SlotLock` across the walk prevents a racing resim
/// from claiming a slot while its segments are inspected or removed, while
/// keeping `read_dir` calls O(1) per batch even when `/tmp` is large on macOS.
#[cfg(unix)]
fn process_segment_batch(
    batch: &mut Vec<(u32, SlotLock)>,
    segment_dir: Option<&Path>,
    mode: SweepMode,
    started: Instant,
    budget: Option<Duration>,
    report: &mut ResimSweepReport,
) {
    if batch.is_empty() {
        return;
    }
    let Some(dir) = segment_dir else {
        batch.clear();
        return;
    };
    if budget.is_some_and(|b| started.elapsed() >= b) {
        report.truncated = true;
        batch.clear();
        return;
    }
    let batch_slots: BTreeSet<u32> = batch.iter().map(|(slot, _)| *slot).collect();
    match std::fs::read_dir(dir) {
        Ok(entries) => {
            for entry in entries {
                if budget.is_some_and(|b| started.elapsed() >= b) {
                    report.truncated = true;
                    break;
                }
                let entry = match entry {
                    Ok(entry) => entry,
                    Err(error) => {
                        report.errors.push(format!("{}: {error}", dir.display()));
                        continue;
                    }
                };
                let Some(slot) = entry.file_name().to_str().and_then(resim_namespace_slot) else {
                    continue;
                };
                if !batch_slots.contains(&slot) {
                    continue;
                }
                let path = entry.path();
                match mode {
                    SweepMode::ReportOnly => {
                        if is_segment_file(&path) {
                            report.leftover_segments += 1;
                        }
                    }
                    SweepMode::Remove => match remove_leftover_segment(&path) {
                        SegmentRemoval::Removed => {
                            report.leftover_segments += 1;
                            report.segments_removed += 1;
                        }
                        SegmentRemoval::NotASegment => {}
                        SegmentRemoval::Failed(error) => {
                            report.leftover_segments += 1;
                            report.segments_failed += 1;
                            report.errors.push(error);
                        }
                    },
                }
            }
        }
        Err(error) if error.kind() == std::io::ErrorKind::NotFound => {}
        Err(error) => {
            report.errors.push(format!("{}: {error}", dir.display()));
        }
    }
    batch.clear();
}

/// Sweep every lockable resim namespace slot (`0..RESIM_NAMESPACE_SLOTS`)
/// against `base`, `lock_dir`, and `segment_dir`.
///
/// Slots whose lock file does not exist are skipped without creating files.
/// Slots whose lock is held by a running resim (`WOULDBLOCK`) are skipped and
/// counted in [`ResimSweepReport::slots_in_use`]. Symlink, non-regular, and
/// foreign-owned lock files are refused and recorded in
/// [`ResimSweepReport::errors`].
///
/// In [`SweepMode::ReportOnly`], nothing on disk is created or removed: dead
/// nodes and reclaimable leftover segments are counted only. In
/// [`SweepMode::Remove`], dead nodes in each lockable slot are reaped and,
/// once `nodes_left == Ok(0)`, leftover segments carrying that slot's prefix
/// (including `.global_mgmt`) are removed.
#[must_use]
pub fn sweep_resim_slots(
    base: &Config,
    lock_dir: &Path,
    segment_dir: Option<&Path>,
    mode: SweepMode,
    budget: Option<Duration>,
) -> ResimSweepReport {
    #[cfg(not(unix))]
    {
        let _ = (base, lock_dir, segment_dir, mode, budget);
        ResimSweepReport::default()
    }
    #[cfg(unix)]
    {
        const SEGMENT_BATCH_LIMIT: usize = 32;
        let started = Instant::now();
        let mut report = ResimSweepReport::default();
        if budget.is_some_and(|b| b.is_zero()) {
            report.truncated = true;
            return report;
        }
        let expected_euid = cerulion_hygiene::euid();
        let mut segment_batch: Vec<(u32, SlotLock)> = Vec::new();

        for slot in 0..RESIM_NAMESPACE_SLOTS {
            if budget.is_some_and(|b| started.elapsed() >= b) {
                report.truncated = true;
                break;
            }
            let lock_path = lock_dir.join(format!("{RESIM_NAMESPACE_MARKER}{slot:08x}.lock"));
            let lock = match try_lock_existing_slot(&lock_path, expected_euid) {
                Ok(ExistingLockAttempt::Absent) => continue,
                Ok(ExistingLockAttempt::HeldByLiveResim) => {
                    report.slots_in_use += 1;
                    continue;
                }
                Ok(ExistingLockAttempt::Acquired(lock)) => lock,
                Err(error) => {
                    report.errors.push(error);
                    continue;
                }
            };
            report.slots_checked += 1;

            let ix_config = match resim_ix_config(base, slot) {
                Ok(cfg) => cfg,
                Err(error) => {
                    report.errors.push(error);
                    continue;
                }
            };

            match mode {
                SweepMode::ReportOnly => {
                    let mut dead = 0u64;
                    let mut non_dead = 0u64;
                    let listed = Node::<CerService>::list(&ix_config, |state| {
                        match state {
                            NodeState::Dead(_) => dead += 1,
                            NodeState::Alive(_)
                            | NodeState::Inaccessible(_)
                            | NodeState::Undefined(_) => non_dead += 1,
                        }
                        CallbackProgression::Continue
                    });
                    match listed {
                        Ok(()) => {
                            report.dead_nodes += dead;
                            if non_dead == 0 {
                                segment_batch.push((slot, lock));
                            }
                        }
                        Err(error) => {
                            report.registry_errors += 1;
                            report
                                .errors
                                .push(format!("slot {slot}: could not list nodes ({error:?})"));
                        }
                    }
                }
                SweepMode::Remove => {
                    if let Some(total_budget) = budget {
                        let remaining = total_budget.saturating_sub(started.elapsed());
                        let cleaned =
                            cerulion_core::transport::dead_node_sweep::cleanup_dead_nodes_bounded(
                                &ix_config, remaining,
                            );
                        report.dead_nodes +=
                            cleaned.cleanups + cleaned.failed_cleanups + cleaned.deferred;
                        report.cleaned_nodes += cleaned.cleanups;
                        report.failed_node_cleanups += cleaned.failed_cleanups;
                        report.deferred_nodes += cleaned.deferred;
                        if cleaned.deferred > 0 {
                            report.truncated = true;
                        }
                    } else {
                        let swept = crate::ipc_cleanup::sweep_dead_nodes_with_config(
                            &ix_config,
                            SweepMode::Remove,
                        );
                        report.dead_nodes += swept.dead_nodes.len() as u64;
                        report.cleaned_nodes += swept.cleanups;
                        report.failed_node_cleanups += swept.failed_cleanups;
                        for err in swept.registry_errors {
                            report.registry_errors += 1;
                            report.errors.push(format!("slot {slot}: {err}"));
                        }
                    }
                    let mut nodes_left = 0u64;
                    let listed = Node::<CerService>::list(&ix_config, |_| {
                        nodes_left += 1;
                        CallbackProgression::Continue
                    });
                    match listed {
                        Ok(()) if nodes_left == 0 => {
                            segment_batch.push((slot, lock));
                        }
                        Ok(()) => {}
                        Err(error) => {
                            report.registry_errors += 1;
                            report.errors.push(format!(
                                "slot {slot}: could not list nodes after sweep ({error:?})"
                            ));
                        }
                    }
                }
            }

            if segment_batch.len() >= SEGMENT_BATCH_LIMIT {
                process_segment_batch(
                    &mut segment_batch,
                    segment_dir,
                    mode,
                    started,
                    budget,
                    &mut report,
                );
            }
        }

        process_segment_batch(
            &mut segment_batch,
            segment_dir,
            mode,
            started,
            budget,
            &mut report,
        );
        report
    }
}

/// Sweep every lockable resim namespace slot on the host's resolved global
/// iceoryx2 config.
#[must_use]
pub fn sweep_resim_slots_global(mode: SweepMode, budget: Option<Duration>) -> ResimSweepReport {
    let base = Config::global_config();
    let lock_dir = resim_slot_lock_dir(base);
    sweep_resim_slots(base, &lock_dir, resim_segment_dir(), mode, budget)
}

#[cfg(all(test, unix))]
mod tests {
    use super::*;
    use cerulion_core::testing::IsolatedRoot;

    #[test]
    fn sweep_resim_slots_refuses_symlink_lock_and_respects_zero_budget() {
        let root = IsolatedRoot::mint("rslotsweepunit");
        let locks = tempfile::tempdir().expect("lock dir");
        let segments = tempfile::tempdir().expect("segment dir");

        // Zero budget truncates immediately without touching anything.
        let zero = sweep_resim_slots(
            &root.config(),
            locks.path(),
            Some(segments.path()),
            SweepMode::ReportOnly,
            Some(Duration::ZERO),
        );
        assert!(zero.truncated);
        assert!(!zero.nodes_converged());
        assert_eq!(zero.slots_checked, 0);

        // Empty lock dir reports clean and creates zero lock files.
        let empty = sweep_resim_slots(
            &root.config(),
            locks.path(),
            Some(segments.path()),
            SweepMode::ReportOnly,
            None,
        );
        assert_eq!(empty, ResimSweepReport::default());
        assert!(empty.nodes_converged());
        assert_eq!(
            std::fs::read_dir(locks.path()).expect("read locks").count(),
            0,
            "ReportOnly must create no lock files"
        );

        // Adversarial: symlink lock file at slot 0 is refused and recorded as an error.
        let target = locks.path().join("victim");
        std::fs::write(&target, b"keep").expect("write victim");
        let symlink = locks.path().join("cer_resim_00000000.lock");
        std::os::unix::fs::symlink(&target, &symlink).expect("create symlink");
        let report = sweep_resim_slots(
            &root.config(),
            locks.path(),
            Some(segments.path()),
            SweepMode::Remove,
            None,
        );
        assert_eq!(report.slots_checked, 0);
        assert_eq!(report.errors.len(), 1, "{report:?}");
        assert!(
            report.errors[0].contains(&symlink.display().to_string()),
            "{:?}",
            report.errors
        );
        assert_eq!(std::fs::read(&target).expect("victim intact"), b"keep");
    }

    #[test]
    fn fifo_lock_file_is_refused_without_blocking_in_claim_and_sweep() {
        let root = IsolatedRoot::mint("rslotfifo");
        let locks = tempfile::tempdir().expect("lock dir");
        let segments = tempfile::tempdir().expect("segment dir");
        let fifo = locks.path().join("cer_resim_00000000.lock");
        let status = std::process::Command::new("mkfifo")
            .arg(&fifo)
            .status()
            .expect("spawn mkfifo");
        assert!(status.success(), "mkfifo failed for {}", fifo.display());

        // Both sweep and claim must return immediately and refuse the FIFO.
        let (tx, rx) = std::sync::mpsc::channel();
        let cfg = root.config();
        let locks_path = locks.path().to_path_buf();
        let segs_path = segments.path().to_path_buf();
        let handle = std::thread::spawn(move || {
            let sweep = sweep_resim_slots(
                &cfg,
                &locks_path,
                Some(&segs_path),
                SweepMode::ReportOnly,
                None,
            );
            let claimed = claim_resim_slot(&locks_path, 2, |_| Ok(())).map(|s| s.slot);
            let _ = tx.send((sweep, claimed));
        });
        let (sweep, claimed) = rx
            .recv_timeout(Duration::from_secs(10))
            .expect("FIFO lock file must not block sweep or claim");
        handle.join().expect("fifo worker thread panicked");
        assert_eq!(sweep.slots_checked, 0);
        assert_eq!(sweep.errors.len(), 1, "{sweep:?}");
        assert!(
            sweep.errors[0].contains("not a regular file"),
            "{:?}",
            sweep.errors
        );
        assert_eq!(
            claimed,
            Ok(1),
            "claim must pass over the FIFO at slot 0 and take slot 1"
        );
    }

    #[test]
    fn total_failures_does_not_double_count_segment_errors() {
        let seg_only = ResimSweepReport {
            segments_failed: 1,
            errors: vec!["seg: EACCES".to_string()],
            ..ResimSweepReport::default()
        };
        assert_eq!(seg_only.total_failures(), 1);

        let mixed = ResimSweepReport {
            failed_node_cleanups: 2,
            segments_failed: 1,
            errors: vec!["seg: EACCES".to_string(), "lock: ELOOP".to_string()],
            ..ResimSweepReport::default()
        };
        assert_eq!(mixed.total_failures(), 4);

        let synthetic_no_errors = ResimSweepReport {
            failed_node_cleanups: 1,
            segments_failed: 2,
            ..ResimSweepReport::default()
        };
        assert_eq!(synthetic_no_errors.total_failures(), 3);
    }
}
