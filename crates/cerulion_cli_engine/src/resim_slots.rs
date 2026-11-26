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
use std::collections::{BTreeMap, BTreeSet};
#[cfg(unix)]
use std::io::{Read, Seek, SeekFrom, Write};
#[cfg(unix)]
use std::ops::Range;
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

/// Schema version for the owning-registry metadata stored inside a slot lock file.
#[cfg(unix)]
const SLOT_OWNER_RECORD_VERSION: u32 = 1;

/// Maximum lock-file metadata size read when identifying a slot's owning registry.
#[cfg(unix)]
const MAX_SLOT_LOCK_BYTES: u64 = 16 * 1024;

/// Owning iceoryx2 registry metadata persisted inside `cer_resim_<slot:08x>.lock`
/// while holding its exclusive `flock`.
///
/// Lock files and shared-memory segment names live in host-wide directories
/// (`/dev/shm` on Linux, `/tmp` on macOS) even when two processes use different
/// iceoryx2 registry roots. Recording the claiming process's registry paths lets
/// a later sweep or claim from another root inspect and reap that owning registry
/// before unlinking any segment, so dead nodes in the original root are never
/// stranded without their name mappings.
#[cfg(unix)]
#[derive(Debug, Clone, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
#[serde(deny_unknown_fields)]
struct SlotOwnerRecord {
    version: u32,
    root_path: String,
    node_dir: String,
    service_dir: String,
}

/// Resolution of a recorded owning registry on disk.
#[cfg(unix)]
enum OwnerRegistry {
    /// The recorded `root_path` no longer exists on disk, so it holds no nodes.
    Absent,
    /// Concrete iceoryx2 config targeting the recorded registry for one slot.
    Config(Box<Config>),
}

#[cfg(unix)]
impl SlotOwnerRecord {
    fn from_config(base: &Config) -> Self {
        Self {
            version: SLOT_OWNER_RECORD_VERSION,
            root_path: String::from(base.global.root_path()),
            node_dir: String::from(&base.global.node.directory),
            service_dir: String::from(&base.global.service.directory),
        }
    }

    fn same_registry(&self, other: &Self) -> bool {
        Path::new(&self.root_path)
            .components()
            .eq(Path::new(&other.root_path).components())
            && Path::new(&self.node_dir)
                .components()
                .eq(Path::new(&other.node_dir).components())
            && Path::new(&self.service_dir)
                .components()
                .eq(Path::new(&other.service_dir).components())
    }

    fn resolve_owner_registry(&self, base: &Config, slot: u32) -> Result<OwnerRegistry, String> {
        use iceoryx2::prelude::{Path as IoxPath, SemanticString};
        let root_path = Path::new(&self.root_path);
        match std::fs::symlink_metadata(root_path) {
            Ok(meta) if meta.file_type().is_dir() => {}
            Ok(_) => {
                return Err(format!(
                    "recorded slot owner root '{}' is not a real directory",
                    self.root_path
                ));
            }
            Err(err) if err.kind() == std::io::ErrorKind::NotFound => {
                return Ok(OwnerRegistry::Absent);
            }
            Err(err) => {
                return Err(format!(
                    "could not inspect recorded slot owner root '{}': {err}",
                    self.root_path
                ));
            }
        }
        let node_dir_path = root_path.join(&self.node_dir);
        match std::fs::symlink_metadata(&node_dir_path) {
            Ok(meta) if meta.file_type().is_dir() => {}
            Ok(_) => {
                return Err(format!(
                    "recorded slot owner node directory '{}' is not a real directory",
                    node_dir_path.display()
                ));
            }
            Err(err) if err.kind() == std::io::ErrorKind::NotFound => {}
            Err(err) => {
                return Err(format!(
                    "could not inspect recorded slot owner node directory '{}': {err}",
                    node_dir_path.display()
                ));
            }
        }
        let mut cfg = resim_ix_config(base, slot)?;
        let iox_root = IoxPath::new(self.root_path.as_bytes()).map_err(|e| {
            format!(
                "invalid recorded slot owner root '{}': {e:?}",
                self.root_path
            )
        })?;
        let iox_node = IoxPath::new(self.node_dir.as_bytes()).map_err(|e| {
            format!(
                "invalid recorded slot owner node_dir '{}': {e:?}",
                self.node_dir
            )
        })?;
        let iox_service = IoxPath::new(self.service_dir.as_bytes()).map_err(|e| {
            format!(
                "invalid recorded slot owner service_dir '{}': {e:?}",
                self.service_dir
            )
        })?;
        cfg.global.set_root_path(&iox_root);
        cfg.global.node.directory = iox_node;
        cfg.global.service.directory = iox_service;
        Ok(OwnerRegistry::Config(Box::new(cfg)))
    }
}

/// Ownership metadata parsed from an already-locked slot lock descriptor.
#[cfg(unix)]
#[derive(Debug, Clone, PartialEq, Eq)]
enum SlotLockOwner {
    /// Empty 0-byte lock file (newly created or created by an older binary).
    LegacyEmpty,
    /// Valid v1 owning-registry record.
    Recorded(SlotOwnerRecord),
    /// Non-empty lock file that could not be parsed or uses an unknown schema.
    Unrecognized(String),
}

#[cfg(unix)]
impl SlotLock {
    fn read_owner(&self) -> SlotLockOwner {
        let mut file = &self.0;
        if let Err(err) = file.seek(SeekFrom::Start(0)) {
            return SlotLockOwner::Unrecognized(format!("could not seek slot lock file: {err}"));
        }
        let mut buf = Vec::new();
        if let Err(err) = file
            .take(MAX_SLOT_LOCK_BYTES.saturating_add(1))
            .read_to_end(&mut buf)
        {
            return SlotLockOwner::Unrecognized(format!("could not read slot lock file: {err}"));
        }
        if buf.is_empty() {
            return SlotLockOwner::LegacyEmpty;
        }
        if buf.len() as u64 > MAX_SLOT_LOCK_BYTES {
            return SlotLockOwner::Unrecognized(format!(
                "slot lock metadata exceeds {MAX_SLOT_LOCK_BYTES} bytes"
            ));
        }
        match serde_json::from_slice::<SlotOwnerRecord>(&buf) {
            Ok(record) if record.version == SLOT_OWNER_RECORD_VERSION => {
                SlotLockOwner::Recorded(record)
            }
            Ok(record) => SlotLockOwner::Unrecognized(format!(
                "unsupported slot lock metadata version {}",
                record.version
            )),
            Err(err) => {
                SlotLockOwner::Unrecognized(format!("could not parse slot lock metadata: {err}"))
            }
        }
    }

    fn write_owner(&self, owner: &SlotOwnerRecord) -> Result<(), String> {
        let bytes = serde_json::to_vec(owner)
            .map_err(|err| format!("could not encode slot lock metadata: {err}"))?;
        let mut file = &self.0;
        file.seek(SeekFrom::Start(0))
            .map_err(|err| format!("could not seek slot lock file for write: {err}"))?;
        self.0
            .set_len(0)
            .map_err(|err| format!("could not truncate slot lock file: {err}"))?;
        file.write_all(&bytes)
            .map_err(|err| format!("could not write slot lock metadata: {err}"))?;
        file.flush()
            .map_err(|err| format!("could not flush slot lock metadata: {err}"))?;
        Ok(())
    }
}

/// Extract a readable message from a caught unwind payload.
#[cfg(unix)]
fn panic_payload_message(payload: Box<dyn std::any::Any + Send>) -> String {
    payload
        .downcast_ref::<&str>()
        .map(|s| (*s).to_string())
        .or_else(|| payload.downcast_ref::<String>().cloned())
        .unwrap_or_else(|| "<non-string panic payload>".to_string())
}

/// Run an iceoryx2 registry operation inside a panic boundary so a stray or
/// malformed registry entry turns into a reported error rather than crashing
/// a cleanup pass or `ExitHygieneGuard::drop`.
#[cfg(unix)]
fn run_guarded_registry_walk<T>(walk: impl FnOnce() -> Result<T, String>) -> Result<T, String> {
    match std::panic::catch_unwind(std::panic::AssertUnwindSafe(walk)) {
        Ok(result) => result,
        Err(payload) => Err(format!(
            "iceoryx2 node registry walk panicked: {}",
            panic_payload_message(payload)
        )),
    }
}

/// The single panic-guarded `Node::list` entry point for resim slot operations.
#[cfg(unix)]
fn list_nodes_guarded(
    ix_config: &Config,
    mut on_node: impl FnMut(NodeState<CerService>),
) -> Result<(), String> {
    run_guarded_registry_walk(|| {
        Node::<CerService>::list(ix_config, |state| {
            on_node(state);
            CallbackProgression::Continue
        })
        .map_err(|error| format!("{error:?}"))
    })
}

/// Panic-guarded wrapper around the unbounded dead-node sweep for one slot.
#[cfg(unix)]
fn sweep_dead_nodes_guarded(
    ix_config: &Config,
) -> Result<crate::ipc_cleanup::CleanupReport, String> {
    run_guarded_registry_walk(|| {
        Ok(crate::ipc_cleanup::sweep_dead_nodes_with_config(
            ix_config,
            SweepMode::Remove,
        ))
    })
}

/// Outcome of inspecting one slot's node registry in report-only mode.
#[cfg(unix)]
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
struct NodeCounts {
    dead: u64,
    non_dead: u64,
}

/// Injectable registry operations so tests can drive panic-containment paths
/// through the real `reclaim_resim_slot` and `sweep_resim_slots` logic.
#[cfg(unix)]
trait RegistryWalkOps {
    fn count_nodes(&self, ix_config: &Config) -> Result<NodeCounts, String>;
    fn list_node_count(&self, ix_config: &Config) -> Result<u64, String>;
    fn sweep_bounded(
        &self,
        ix_config: &Config,
        budget: Duration,
    ) -> cerulion_core::transport::dead_node_sweep::BoundedCleanup;
    fn sweep_unbounded(
        &self,
        ix_config: &Config,
    ) -> Result<crate::ipc_cleanup::CleanupReport, String>;
}

#[cfg(unix)]
struct RealRegistryWalkOps;

#[cfg(unix)]
impl RegistryWalkOps for RealRegistryWalkOps {
    fn count_nodes(&self, ix_config: &Config) -> Result<NodeCounts, String> {
        let mut counts = NodeCounts::default();
        list_nodes_guarded(ix_config, |state| match state {
            NodeState::Dead(_) => counts.dead += 1,
            NodeState::Alive(_) | NodeState::Inaccessible(_) | NodeState::Undefined(_) => {
                counts.non_dead += 1;
            }
        })?;
        Ok(counts)
    }

    fn list_node_count(&self, ix_config: &Config) -> Result<u64, String> {
        let mut nodes = 0u64;
        list_nodes_guarded(ix_config, |_| {
            nodes += 1;
        })?;
        Ok(nodes)
    }

    fn sweep_bounded(
        &self,
        ix_config: &Config,
        budget: Duration,
    ) -> cerulion_core::transport::dead_node_sweep::BoundedCleanup {
        cerulion_core::transport::dead_node_sweep::cleanup_dead_nodes_bounded(ix_config, budget)
    }

    fn sweep_unbounded(
        &self,
        ix_config: &Config,
    ) -> Result<crate::ipc_cleanup::CleanupReport, String> {
        sweep_dead_nodes_guarded(ix_config)
    }
}

/// Claim the lowest slot below `slots` whose lock file in `lock_dir`
/// (`cer_resim_{slot:08x}.lock`, created with mode `0600`) this process can
/// lock with an exclusive non-blocking `flock`, and that `ready` accepts once
/// the lock is held.
#[cfg(all(test, unix))]
pub(crate) fn claim_resim_slot(
    lock_dir: &Path,
    slots: u32,
    mut ready: impl FnMut(u32) -> Result<(), SlotPassedOver>,
) -> Result<ResimSlot, String> {
    claim_resim_slot_in_range(lock_dir, 0..slots, |slot, _lock, _created_fresh| {
        ready(slot)
    })
}

/// Open or create a slot lock file without following symlinks or blocking on a
/// FIFO, verify ownership, and take a non-blocking exclusive `flock`.
/// Returns `(Option<SlotLock>, created_fresh)` where `None` means `WOULDBLOCK`.
#[cfg(unix)]
fn try_claim_slot_lock(
    path: &Path,
    expected_euid: u32,
) -> Result<(Option<SlotLock>, bool), String> {
    use std::os::unix::fs::{MetadataExt, OpenOptionsExt};
    let (lock, created_fresh) = match std::fs::OpenOptions::new()
        .read(true)
        .write(true)
        .create_new(true)
        .mode(0o600)
        .custom_flags(libc::O_NOFOLLOW | libc::O_NONBLOCK)
        .open(path)
    {
        Ok(file) => (file, true),
        Err(e) if e.kind() == std::io::ErrorKind::AlreadyExists => {
            let file = std::fs::OpenOptions::new()
                .read(true)
                .write(true)
                .create(false)
                .truncate(false)
                .custom_flags(libc::O_NOFOLLOW | libc::O_NONBLOCK)
                .open(path)
                .map_err(|err| format!("{}: {err}", path.display()))?;
            (file, false)
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
        Ok(()) => Ok((Some(SlotLock(lock)), created_fresh)),
        Err(e) if e == rustix::io::Errno::WOULDBLOCK => Ok((None, created_fresh)),
        Err(e) => Err(format!("{}: {e}", path.display())),
    }
}

#[cfg(unix)]
fn claim_resim_slot_in_range(
    lock_dir: &Path,
    slots: Range<u32>,
    mut ready: impl FnMut(u32, &SlotLock, bool) -> Result<(), SlotPassedOver>,
) -> Result<ResimSlot, String> {
    let expected_euid = cerulion_hygiene::euid();
    let slot_count = slots.end.saturating_sub(slots.start);
    let mut latch = cerulion_core::transport::failure_regime_latch::FailureRegimeLatch::new();
    let mut first_failure: Option<String> = None;
    let mut last_refusal: Option<(u32, SlotPassedOver)> = None;
    for slot in slots {
        let path = lock_dir.join(format!("{RESIM_NAMESPACE_MARKER}{slot:08x}.lock"));
        let (lock, created_fresh) = match try_claim_slot_lock(&path, expected_euid) {
            Ok((Some(lock), created_fresh)) => (lock, created_fresh),
            // Held by a running resim.
            Ok((None, _)) => continue,
            Err(failure) => {
                first_failure.get_or_insert_with(|| failure.clone());
                log_passed_over_slot(&mut latch, slot, &SlotPassedOver::Lock(failure));
                continue;
            }
        };
        match ready(slot, &lock, created_fresh) {
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
        "no resim namespace slot is free: none of the {slot_count} slot locks in '{}' could be \
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
    reclaim_resim_slot_with_ops(ix_config, slot, segment_dir, budget, &RealRegistryWalkOps)
}

#[cfg(unix)]
fn reclaim_resim_slot_with_ops(
    ix_config: &Config,
    slot: u32,
    segment_dir: Option<&Path>,
    budget: Duration,
    ops: &dyn RegistryWalkOps,
) -> SlotReclaim {
    let started = Instant::now();
    let cleaned = ops.sweep_bounded(ix_config, budget);
    let nodes_left = ops.list_node_count(ix_config);
    let mut reclaim = SlotReclaim {
        cleaned,
        nodes_left,
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
    if started.elapsed() >= budget {
        return reclaim;
    }
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

/// Why a bounded segment-directory scan for legacy empty slot locks could not
/// finish.
#[cfg(unix)]
#[derive(Debug, Clone, PartialEq, Eq)]
enum SegmentScanError {
    /// The shared startup cleanup budget expired before the scan finished.
    BudgetExhausted,
    /// Opening or reading `segment_dir` failed.
    Io(String),
}

/// Scan `segment_dir` once and count segment files per resim slot without
/// mutating them, stopping with [`SegmentScanError::BudgetExhausted`] as soon
/// as `budget_expired()` returns `true` (checked before `read_dir` and before
/// every entry).
#[cfg(unix)]
fn scan_all_slot_segments_bounded(
    segment_dir: Option<&Path>,
    mut budget_expired: impl FnMut() -> bool,
) -> Result<BTreeMap<u32, u64>, SegmentScanError> {
    let Some(dir) = segment_dir else {
        return Ok(BTreeMap::new());
    };
    if budget_expired() {
        return Err(SegmentScanError::BudgetExhausted);
    }
    let entries = match std::fs::read_dir(dir) {
        Ok(entries) => entries,
        Err(error) if error.kind() == std::io::ErrorKind::NotFound => return Ok(BTreeMap::new()),
        Err(error) => return Err(SegmentScanError::Io(format!("{}: {error}", dir.display()))),
    };
    let mut counts = BTreeMap::new();
    for entry in entries {
        if budget_expired() {
            return Err(SegmentScanError::BudgetExhausted);
        }
        let entry =
            entry.map_err(|error| SegmentScanError::Io(format!("{}: {error}", dir.display())))?;
        let Some(slot) = entry.file_name().to_str().and_then(resim_namespace_slot) else {
            continue;
        };
        if is_segment_file(&entry.path()) {
            *counts.entry(slot).or_insert(0) += 1;
        }
    }
    Ok(counts)
}

/// Count segment files belonging to `slot` in `segment_dir` without mutating
/// them, stopping with [`SegmentScanError::BudgetExhausted`] as soon as
/// `budget_expired()` returns `true` (checked before `read_dir` and before
/// every entry).
#[cfg(unix)]
fn count_slot_segments_bounded(
    segment_dir: Option<&Path>,
    slot: u32,
    mut budget_expired: impl FnMut() -> bool,
) -> Result<u64, SegmentScanError> {
    let Some(dir) = segment_dir else {
        return Ok(0);
    };
    if budget_expired() {
        return Err(SegmentScanError::BudgetExhausted);
    }
    let entries = match std::fs::read_dir(dir) {
        Ok(entries) => entries,
        Err(error) if error.kind() == std::io::ErrorKind::NotFound => return Ok(0),
        Err(error) => return Err(SegmentScanError::Io(format!("{}: {error}", dir.display()))),
    };
    let mut count = 0u64;
    for entry in entries {
        if budget_expired() {
            return Err(SegmentScanError::BudgetExhausted);
        }
        let entry =
            entry.map_err(|error| SegmentScanError::Io(format!("{}: {error}", dir.display())))?;
        if entry.file_name().to_str().and_then(resim_namespace_slot) != Some(slot) {
            continue;
        }
        if is_segment_file(&entry.path()) {
            count += 1;
        }
    }
    Ok(count)
}

/// Injectable legacy segment scan seam so tests can verify scan counts and
/// race behavior through the real `claim_and_reclaim_resim_slot_in_range`
/// logic without sleeps.
#[cfg(unix)]
trait LegacySegmentScanOps {
    fn scan_all(
        &self,
        segment_dir: Option<&Path>,
        started: Instant,
        budget: Duration,
    ) -> Result<BTreeMap<u32, u64>, SegmentScanError>;

    fn count_slot(
        &self,
        segment_dir: Option<&Path>,
        slot: u32,
        started: Instant,
        budget: Duration,
    ) -> Result<u64, SegmentScanError>;
}

#[cfg(unix)]
struct RealLegacySegmentScanOps;

#[cfg(unix)]
impl LegacySegmentScanOps for RealLegacySegmentScanOps {
    fn scan_all(
        &self,
        segment_dir: Option<&Path>,
        started: Instant,
        budget: Duration,
    ) -> Result<BTreeMap<u32, u64>, SegmentScanError> {
        scan_all_slot_segments_bounded(segment_dir, || started.elapsed() >= budget)
    }

    fn count_slot(
        &self,
        segment_dir: Option<&Path>,
        slot: u32,
        started: Instant,
        budget: Duration,
    ) -> Result<u64, SegmentScanError> {
        count_slot_segments_bounded(segment_dir, slot, || started.elapsed() >= budget)
    }
}

#[cfg(unix)]
fn legacy_budget_exhausted_refusal(reclaim: SlotReclaim) -> SlotPassedOver {
    SlotPassedOver::Reclaim {
        reclaim,
        reason: "legacy slot lock records no owning registry and the startup cleanup budget \
                 expired before the segment directory scan finished"
            .to_string(),
    }
}

#[cfg(unix)]
fn legacy_leftover_segments_refusal(reclaim: SlotReclaim, leftover: u64) -> SlotPassedOver {
    SlotPassedOver::Reclaim {
        reclaim,
        reason: format!(
            "legacy slot lock records no owning registry and {leftover} leftover segment(s) \
             remain; preserving mappings to avoid stranding another registry"
        ),
    }
}

#[cfg(unix)]
fn legacy_scan_io_refusal(reclaim: SlotReclaim, err: &str) -> SlotPassedOver {
    SlotPassedOver::Reclaim {
        reclaim,
        reason: format!(
            "legacy slot lock records no owning registry and segment directory could not be \
             inspected ({err})"
        ),
    }
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
    claim_and_reclaim_resim_slot_in_range(base, lock_dir, segment_dir, 0..slots, budget)
}

#[cfg(unix)]
fn claim_and_reclaim_resim_slot_in_range(
    base: &Config,
    lock_dir: &Path,
    segment_dir: Option<&Path>,
    slots: Range<u32>,
    budget: Duration,
) -> Result<ResimSlot, String> {
    claim_and_reclaim_resim_slot_with_scanner(
        base,
        lock_dir,
        segment_dir,
        slots,
        budget,
        &RealLegacySegmentScanOps,
    )
}

#[cfg(unix)]
fn claim_and_reclaim_resim_slot_with_scanner(
    base: &Config,
    lock_dir: &Path,
    segment_dir: Option<&Path>,
    slots: Range<u32>,
    budget: Duration,
    scanner: &dyn LegacySegmentScanOps,
) -> Result<ResimSlot, String> {
    let started = Instant::now();
    let _ = std::fs::create_dir_all(lock_dir);
    let base_owner = SlotOwnerRecord::from_config(base);
    let (mut cleanups, mut deferred, mut segments, mut total_failures) = (0u64, 0u64, 0u64, 0u64);
    // Lazily populated single bounded scan of `segment_dir` shared across all
    // legacy 0-byte locks encountered during this claim pass.
    let mut legacy_segment_scan: Option<Result<BTreeMap<u32, u64>, SegmentScanError>> = None;
    let claimed = claim_resim_slot_in_range(lock_dir, slots, |slot, lock, created_fresh| {
        let ix_config = resim_ix_config(base, slot).map_err(SlotPassedOver::Config)?;
        let prior_owner = if created_fresh {
            SlotLockOwner::LegacyEmpty
        } else {
            lock.read_owner()
        };

        let mut reclaim_segments = true;
        match &prior_owner {
            SlotLockOwner::Unrecognized(error) => {
                return Err(SlotPassedOver::Lock(error.clone()));
            }
            SlotLockOwner::Recorded(owner_rec) if !owner_rec.same_registry(&base_owner) => {
                let owner_registry = owner_rec
                    .resolve_owner_registry(base, slot)
                    .map_err(SlotPassedOver::Config)?;
                if let OwnerRegistry::Config(owner_ix) = owner_registry {
                    let owner_reclaim = reclaim_resim_slot(
                        &owner_ix,
                        slot,
                        None,
                        budget.saturating_sub(started.elapsed()),
                    );
                    cleanups += owner_reclaim.cleaned.cleanups;
                    deferred += owner_reclaim.cleaned.deferred;
                    total_failures += owner_reclaim.cleaned.failed_cleanups;
                    if let Some(reason) = owner_reclaim.refusal() {
                        return Err(SlotPassedOver::Reclaim {
                            reclaim: owner_reclaim,
                            reason,
                        });
                    }
                }
            }
            SlotLockOwner::Recorded(_) => {}
            SlotLockOwner::LegacyEmpty if !created_fresh => {
                // Existing 0-byte lock file without owner metadata: if the
                // shared startup budget is already spent, refuse immediately
                // without walking the registry or segment directory.
                if segment_dir.is_some()
                    && (started.elapsed() >= budget
                        || matches!(
                            legacy_segment_scan,
                            Some(Err(SegmentScanError::BudgetExhausted))
                        ))
                {
                    let skipped = SlotReclaim {
                        cleaned: cerulion_core::transport::dead_node_sweep::BoundedCleanup::default(
                        ),
                        nodes_left: Err("startup cleanup budget expired".to_string()),
                        segments_removed: 0,
                        segments_failed: 0,
                        first_segment_error: None,
                    };
                    return Err(legacy_budget_exhausted_refusal(skipped));
                }
                // Sweep our own registry first without unlinking segments. If
                // our registry had no dead nodes here and segments exist, we
                // cannot prove another registry does not still need those
                // mappings, so pass over the slot.
                reclaim_segments = false;
            }
            SlotLockOwner::LegacyEmpty => {}
        }

        let target_segment_dir = if reclaim_segments { segment_dir } else { None };
        let mut reclaim = reclaim_resim_slot(
            &ix_config,
            slot,
            target_segment_dir,
            budget.saturating_sub(started.elapsed()),
        );
        if !reclaim_segments && reclaim.refusal().is_none() {
            if reclaim.cleaned.cleanups > 0 {
                let seg_reclaim = reclaim_resim_slot(
                    &ix_config,
                    slot,
                    segment_dir,
                    budget.saturating_sub(started.elapsed()),
                );
                reclaim.segments_removed += seg_reclaim.segments_removed;
                reclaim.segments_failed += seg_reclaim.segments_failed;
                if reclaim.first_segment_error.is_none() {
                    reclaim.first_segment_error = seg_reclaim.first_segment_error;
                }
                reclaim.nodes_left = seg_reclaim.nodes_left;
            } else if segment_dir.is_some() {
                let fresh_scan = legacy_segment_scan.is_none();
                if fresh_scan {
                    legacy_segment_scan = Some(scanner.scan_all(segment_dir, started, budget));
                }
                let cached = match legacy_segment_scan.as_ref() {
                    Some(Ok(counts)) => counts.get(&slot).copied().unwrap_or(0),
                    Some(Err(SegmentScanError::BudgetExhausted)) => {
                        return Err(legacy_budget_exhausted_refusal(reclaim));
                    }
                    Some(Err(SegmentScanError::Io(err))) => {
                        return Err(legacy_scan_io_refusal(reclaim, err));
                    }
                    None => unreachable!("legacy_segment_scan populated above"),
                };
                if cached > 0 {
                    return Err(legacy_leftover_segments_refusal(reclaim, cached));
                }
                if !fresh_scan {
                    // This slot was not locked during the earlier shared scan;
                    // confirm while holding its lock so a racing legacy binary
                    // cannot hide newly created segments.
                    match scanner.count_slot(segment_dir, slot, started, budget) {
                        Ok(0) => {}
                        Ok(leftover) => {
                            if let Some(Ok(counts)) = legacy_segment_scan.as_mut() {
                                counts.insert(slot, leftover);
                            }
                            return Err(legacy_leftover_segments_refusal(reclaim, leftover));
                        }
                        Err(SegmentScanError::BudgetExhausted) => {
                            legacy_segment_scan = Some(Err(SegmentScanError::BudgetExhausted));
                            return Err(legacy_budget_exhausted_refusal(reclaim));
                        }
                        Err(SegmentScanError::Io(err)) => {
                            return Err(legacy_scan_io_refusal(reclaim, &err));
                        }
                    }
                }
            }
        }
        cleanups += reclaim.cleaned.cleanups;
        deferred += reclaim.cleaned.deferred;
        segments += reclaim.segments_removed;
        total_failures += reclaim.cleaned.failed_cleanups + reclaim.segments_failed;
        if let Some(reason) = reclaim.refusal() {
            return Err(SlotPassedOver::Reclaim { reclaim, reason });
        }
        lock.write_owner(&base_owner)
            .map_err(SlotPassedOver::Lock)?;
        Ok(())
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
        sweep_resim_slot_range_with_ops(
            base,
            lock_dir,
            segment_dir,
            0..RESIM_NAMESPACE_SLOTS,
            mode,
            budget,
            &RealRegistryWalkOps,
        )
    }
}

#[cfg(unix)]
fn sweep_resim_slot_range_with_ops(
    base: &Config,
    lock_dir: &Path,
    segment_dir: Option<&Path>,
    slots: Range<u32>,
    mode: SweepMode,
    budget: Option<Duration>,
    ops: &dyn RegistryWalkOps,
) -> ResimSweepReport {
    const SEGMENT_BATCH_LIMIT: usize = 32;
    let started = Instant::now();
    let mut report = ResimSweepReport::default();
    if budget.is_some_and(|b| b.is_zero()) {
        report.truncated = true;
        return report;
    }
    let expected_euid = cerulion_hygiene::euid();
    let base_owner = SlotOwnerRecord::from_config(base);
    let mut segment_batch: Vec<(u32, SlotLock)> = Vec::new();

    for slot in slots {
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

        let base_ix_config = match resim_ix_config(base, slot) {
            Ok(cfg) => cfg,
            Err(error) => {
                report.errors.push(error);
                continue;
            }
        };

        let owner = lock.read_owner();
        let (configs_to_check, require_base_dead_for_segments) = match &owner {
            SlotLockOwner::Unrecognized(error) => {
                report.registry_errors += 1;
                report.errors.push(format!("slot {slot}: {error}"));
                continue;
            }
            SlotLockOwner::Recorded(owner_rec) => {
                let mut configs = Vec::with_capacity(2);
                if !owner_rec.same_registry(&base_owner) {
                    match owner_rec.resolve_owner_registry(base, slot) {
                        Ok(OwnerRegistry::Config(owner_ix)) => configs.push(*owner_ix),
                        Ok(OwnerRegistry::Absent) => {}
                        Err(error) => {
                            report.registry_errors += 1;
                            report.errors.push(format!("slot {slot}: {error}"));
                            continue;
                        }
                    }
                }
                configs.push(base_ix_config);
                (configs, false)
            }
            SlotLockOwner::LegacyEmpty => (vec![base_ix_config], true),
        };

        match mode {
            SweepMode::ReportOnly => {
                let mut dead = 0u64;
                let mut non_dead = 0u64;
                let mut all_ok = true;
                for cfg in &configs_to_check {
                    match ops.count_nodes(cfg) {
                        Ok(counts) => {
                            dead += counts.dead;
                            non_dead += counts.non_dead;
                        }
                        Err(error) => {
                            all_ok = false;
                            report.registry_errors += 1;
                            report
                                .errors
                                .push(format!("slot {slot}: could not list nodes ({error})"));
                        }
                    }
                }
                report.dead_nodes += dead;
                let owner_verified = !require_base_dead_for_segments || dead > 0;
                if all_ok && non_dead == 0 && owner_verified {
                    segment_batch.push((slot, lock));
                }
            }
            SweepMode::Remove => {
                let mut saw_dead = 0u64;
                let mut all_listed_ok = true;
                let mut total_nodes_left = 0u64;
                for cfg in &configs_to_check {
                    if let Some(total_budget) = budget {
                        let remaining = total_budget.saturating_sub(started.elapsed());
                        let cleaned = ops.sweep_bounded(cfg, remaining);
                        let cfg_dead =
                            cleaned.cleanups + cleaned.failed_cleanups + cleaned.deferred;
                        saw_dead += cfg_dead;
                        report.dead_nodes += cfg_dead;
                        report.cleaned_nodes += cleaned.cleanups;
                        report.failed_node_cleanups += cleaned.failed_cleanups;
                        report.deferred_nodes += cleaned.deferred;
                        if cleaned.deferred > 0 {
                            report.truncated = true;
                        }
                    } else {
                        match ops.sweep_unbounded(cfg) {
                            Ok(swept) => {
                                let cfg_dead = swept.dead_nodes.len() as u64;
                                saw_dead += cfg_dead;
                                report.dead_nodes += cfg_dead;
                                report.cleaned_nodes += swept.cleanups;
                                report.failed_node_cleanups += swept.failed_cleanups;
                                for err in swept.registry_errors {
                                    all_listed_ok = false;
                                    report.registry_errors += 1;
                                    report.errors.push(format!("slot {slot}: {err}"));
                                }
                            }
                            Err(error) => {
                                all_listed_ok = false;
                                report.registry_errors += 1;
                                report.errors.push(format!(
                                    "slot {slot}: could not sweep dead nodes ({error})"
                                ));
                            }
                        }
                    }
                    match ops.list_node_count(cfg) {
                        Ok(nodes_left) => {
                            total_nodes_left += nodes_left;
                        }
                        Err(error) => {
                            all_listed_ok = false;
                            report.registry_errors += 1;
                            report.errors.push(format!(
                                "slot {slot}: could not list nodes after sweep ({error})"
                            ));
                        }
                    }
                }
                let owner_verified = !require_base_dead_for_segments || saw_dead > 0;
                if all_listed_ok && total_nodes_left == 0 && owner_verified {
                    segment_batch.push((slot, lock));
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

/// Sweep every lockable resim namespace slot on the host's resolved global
/// iceoryx2 config.
#[must_use]
pub fn sweep_resim_slots_global(mode: SweepMode, budget: Option<Duration>) -> ResimSweepReport {
    #[cfg(not(unix))]
    {
        let _ = (mode, budget);
        ResimSweepReport::default()
    }
    #[cfg(unix)]
    {
        match std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            let base = Config::global_config();
            let lock_dir = resim_slot_lock_dir(base);
            sweep_resim_slots(base, &lock_dir, resim_segment_dir(), mode, budget)
        })) {
            Ok(report) => report,
            Err(payload) => {
                let msg = format!(
                    "global resim slot sweep panicked: {}",
                    panic_payload_message(payload)
                );
                tracing::warn!(
                    error = %msg,
                    "resim slot sweep panicked while resolving global config or walking registry"
                );
                ResimSweepReport {
                    registry_errors: 1,
                    errors: vec![msg],
                    ..ResimSweepReport::default()
                }
            }
        }
    }
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

    /// Test-only slot outside `0..RESIM_NAMESPACE_SLOTS`, distinct per process
    /// and `offset`, so unit tests that mint real iceoryx2 nodes never collide
    /// with production resim slots or concurrent test processes on the host.
    fn test_only_slot(offset: u32) -> u32 {
        RESIM_NAMESPACE_SLOTS + (std::process::id() & 0x00ff_ffff) * 8 + offset
    }

    /// Clean up any host `.global_mgmt` segment left by a unit test's real node
    /// on a test-only slot outside `0..RESIM_NAMESPACE_SLOTS`.
    fn remove_test_only_host_segments(slot: u32) {
        assert!(
            slot >= RESIM_NAMESPACE_SLOTS,
            "only test-only slots may be cleaned up without a host lock"
        );
        let Some(dir) = resim_segment_dir() else {
            return;
        };
        let Ok(entries) = std::fs::read_dir(dir) else {
            return;
        };
        for entry in entries.flatten() {
            if entry.file_name().to_str().and_then(resim_namespace_slot) == Some(slot) {
                let _ = remove_leftover_segment(&entry.path());
            }
        }
    }

    #[test]
    fn sweep_on_different_root_preserves_segments_for_owning_registry() {
        let root_a = IsolatedRoot::mint("rslotcrossa");
        let root_b = IsolatedRoot::mint("rslotcrossb");
        let locks = tempfile::tempdir().expect("lock dir");
        let segments = tempfile::tempdir().expect("segment dir");
        let slot = test_only_slot(0);

        // Root A claims the slot and registers a node on it.
        let claimed = claim_and_reclaim_resim_slot_in_range(
            &root_a.config(),
            locks.path(),
            Some(segments.path()),
            slot..slot + 2,
            Duration::from_secs(10),
        )
        .expect("root A claims test slot");
        assert_eq!(claimed.slot, slot);

        let ix_a = resim_ix_config(&root_a.config(), slot).expect("root A slot config");
        let node_a = iceoryx2::node::NodeBuilder::new()
            .config(&ix_a)
            .create::<CerService>()
            .expect("register node on root A slot");

        let seg = segments
            .path()
            .join(format!("{}node.shm_state", resim_namespace_prefix(slot)));
        std::fs::write(&seg, b"").expect("plant slot segment");

        // Release the lock while the node is still registered in root A.
        drop(claimed);

        // Sweeping from a DIFFERENT, empty root B must inspect root A and
        // preserve the segment because root A still has a registered node.
        let swept = sweep_resim_slot_range_with_ops(
            &root_b.config(),
            locks.path(),
            Some(segments.path()),
            slot..slot + 1,
            SweepMode::Remove,
            None,
            &RealRegistryWalkOps,
        );
        assert!(
            seg.exists(),
            "root B must not delete slot's segment while root A still has a registered node: \
             {swept:?}"
        );
        assert_eq!(swept.segments_removed, 0, "{swept:?}");

        // Claiming from root B must pass over this slot (preserving root A's
        // segment and lock record) and take the next slot instead.
        let claimed_b = claim_and_reclaim_resim_slot_in_range(
            &root_b.config(),
            locks.path(),
            Some(segments.path()),
            slot..slot + 2,
            Duration::from_secs(10),
        )
        .expect("root B passes over occupied slot and claims next");
        assert_eq!(claimed_b.slot, slot + 1);
        assert!(
            seg.exists(),
            "passing over the slot on claim must leave root A's segment intact"
        );
        drop(claimed_b);

        // Once root A's node is gone, root B can safely remove the leftover segment.
        drop(node_a);
        let cleaned = sweep_resim_slot_range_with_ops(
            &root_b.config(),
            locks.path(),
            Some(segments.path()),
            slot..slot + 1,
            SweepMode::Remove,
            None,
            &RealRegistryWalkOps,
        );
        assert!(
            !seg.exists(),
            "once root A is empty, root B removes the leftover segment: {cleaned:?}"
        );
        assert_eq!(cleaned.segments_removed, 1, "{cleaned:?}");
        remove_test_only_host_segments(slot);
    }

    #[test]
    fn sweep_on_different_root_preserves_segments_for_legacy_empty_lock_file() {
        let root_a = IsolatedRoot::mint("rslotlega");
        let root_b = IsolatedRoot::mint("rslotlegb");
        let locks = tempfile::tempdir().expect("lock dir");
        let segments = tempfile::tempdir().expect("segment dir");
        let slot = test_only_slot(2);

        // Legacy 0-byte lock file without owner metadata, while root A holds a node.
        let lock_path = locks
            .path()
            .join(format!("{RESIM_NAMESPACE_MARKER}{slot:08x}.lock"));
        std::fs::write(&lock_path, b"").expect("write legacy empty lock file");
        let ix_a = resim_ix_config(&root_a.config(), slot).expect("root A slot config");
        let node_a = iceoryx2::node::NodeBuilder::new()
            .config(&ix_a)
            .create::<CerService>()
            .expect("register node on root A slot");

        let seg = segments
            .path()
            .join(format!("{}legacy.shm_state", resim_namespace_prefix(slot)));
        std::fs::write(&seg, b"").expect("plant legacy slot segment");

        // Root B sees 0 nodes in root B and no owner record in the lock file, so
        // it must preserve the segment instead of stranding root A.
        let swept = sweep_resim_slot_range_with_ops(
            &root_b.config(),
            locks.path(),
            Some(segments.path()),
            slot..slot + 1,
            SweepMode::Remove,
            None,
            &RealRegistryWalkOps,
        );
        assert!(
            seg.exists(),
            "legacy 0-byte lock with empty base root must preserve mappings: {swept:?}"
        );
        assert_eq!(swept.segments_removed, 0, "{swept:?}");

        // Claiming from root B also refuses the legacy slot with leftover
        // segments and moves to the next slot instead of unlinking them.
        let claimed_b = claim_and_reclaim_resim_slot_in_range(
            &root_b.config(),
            locks.path(),
            Some(segments.path()),
            slot..slot + 2,
            Duration::from_secs(10),
        )
        .expect("root B skips legacy slot with leftover segments");
        assert_eq!(claimed_b.slot, slot + 1);
        assert!(
            seg.exists(),
            "legacy slot segment must stay intact on claim"
        );
        drop(claimed_b);
        drop(node_a);
        remove_test_only_host_segments(slot);
    }

    #[test]
    fn corrupt_or_unknown_lock_metadata_fails_closed_without_removing_segments() {
        let root = IsolatedRoot::mint("rslotcorrupt");
        let locks = tempfile::tempdir().expect("lock dir");
        let segments = tempfile::tempdir().expect("segment dir");
        let lock_path = locks.path().join("cer_resim_00000000.lock");
        std::fs::write(&lock_path, b"not-json-metadata").expect("write corrupt metadata");
        let seg = segments
            .path()
            .join(format!("{}seg.shm_state", resim_namespace_prefix(0)));
        std::fs::write(&seg, b"").expect("plant segment");

        let swept = sweep_resim_slots(
            &root.config(),
            locks.path(),
            Some(segments.path()),
            SweepMode::Remove,
            None,
        );
        assert_eq!(swept.registry_errors, 1, "{swept:?}");
        assert!(!swept.nodes_converged(), "{swept:?}");
        assert!(seg.exists(), "corrupt lock metadata must preserve segments");
    }

    #[test]
    fn unknown_key_in_slot_lock_metadata_fails_closed_without_removing_segments() {
        let root = IsolatedRoot::mint("rslotunkkey");
        let locks = tempfile::tempdir().expect("lock dir");
        let segments = tempfile::tempdir().expect("segment dir");
        let owner = SlotOwnerRecord::from_config(&root.config());
        let raw_with_unknown = format!(
            "{{\"version\":{},\"root_path\":{},\"node_dir\":{},\"service_dir\":{},\"extra_key\":\"unexpected\"}}",
            owner.version,
            serde_json::to_string(&owner.root_path).expect("quote root_path"),
            serde_json::to_string(&owner.node_dir).expect("quote node_dir"),
            serde_json::to_string(&owner.service_dir).expect("quote service_dir"),
        );
        let lock_path = locks.path().join("cer_resim_00000000.lock");
        std::fs::write(&lock_path, raw_with_unknown.as_bytes()).expect("write lock with extra key");
        let seg = segments
            .path()
            .join(format!("{}seg.shm_state", resim_namespace_prefix(0)));
        std::fs::write(&seg, b"").expect("plant segment");

        let report_only = sweep_resim_slots(
            &root.config(),
            locks.path(),
            Some(segments.path()),
            SweepMode::ReportOnly,
            None,
        );
        assert_eq!(report_only.registry_errors, 1, "{report_only:?}");
        assert!(!report_only.nodes_converged(), "{report_only:?}");
        assert!(
            report_only
                .errors
                .iter()
                .any(|e| e.contains("unknown field")),
            "expected unknown field error, got: {:?}",
            report_only.errors
        );
        assert!(seg.exists(), "ReportOnly must preserve segments");

        let removed = sweep_resim_slots(
            &root.config(),
            locks.path(),
            Some(segments.path()),
            SweepMode::Remove,
            None,
        );
        assert_eq!(removed.registry_errors, 1, "{removed:?}");
        assert_eq!(removed.segments_removed, 0, "{removed:?}");
        assert!(!removed.nodes_converged(), "{removed:?}");
        assert!(
            seg.exists(),
            "unknown key in lock metadata must fail closed without removing segments"
        );
        // Determinism: repeated sweeps return identical error reports.
        let removed_again = sweep_resim_slots(
            &root.config(),
            locks.path(),
            Some(segments.path()),
            SweepMode::Remove,
            None,
        );
        assert_eq!(removed, removed_again);

        // Claiming must pass over slot 0 (leaving its lock and segment untouched) and take slot 1.
        let claimed = claim_and_reclaim_resim_slot_in_range(
            &root.config(),
            locks.path(),
            Some(segments.path()),
            0..2,
            Duration::from_secs(10),
        )
        .expect("claim skips slot with unknown lock key and takes slot 1");
        assert_eq!(claimed.slot, 1);
        assert!(seg.exists(), "claim must preserve slot 0 segment");
        assert_eq!(
            std::fs::read_to_string(&lock_path).expect("read slot 0 lock"),
            raw_with_unknown,
            "passing over slot 0 must not overwrite its lock file"
        );
        drop(claimed);

        // With only slot 0 in range, claim refuses naming the unknown field.
        let err = claim_and_reclaim_resim_slot_in_range(
            &root.config(),
            locks.path(),
            Some(segments.path()),
            0..1,
            Duration::from_secs(10),
        )
        .expect_err("slot 0 with unknown key must be refused");
        assert!(
            err.contains("unknown field"),
            "claim refusal must name unknown field: {err}"
        );
        assert!(seg.exists(), "refused claim must preserve slot 0 segment");
    }

    #[test]
    fn legacy_empty_slot_locks_respect_startup_budget_without_removing_segments() {
        let root = IsolatedRoot::mint("rslotbudget");
        let locks = tempfile::tempdir().expect("lock dir");
        let segments = tempfile::tempdir().expect("segment dir");
        let legacy_slots = 16u32;
        for slot in 0..legacy_slots {
            let lock_path = locks
                .path()
                .join(format!("{RESIM_NAMESPACE_MARKER}{slot:08x}.lock"));
            std::fs::write(&lock_path, b"").expect("write legacy empty lock");
        }

        // Case 1: zero budget with legacy empty locks and NO leftover segments must still
        // refuse because the segment directory scan cannot complete within the budget.
        let err_empty = claim_and_reclaim_resim_slot_in_range(
            &root.config(),
            locks.path(),
            Some(segments.path()),
            0..legacy_slots,
            Duration::ZERO,
        )
        .expect_err("zero startup budget must refuse legacy empty locks");
        assert!(
            err_empty.contains("budget"),
            "refusal must explain startup budget expiry: {err_empty}"
        );
        for slot in 0..legacy_slots {
            let lock_path = locks
                .path()
                .join(format!("{RESIM_NAMESPACE_MARKER}{slot:08x}.lock"));
            assert_eq!(
                std::fs::read(&lock_path).expect("read legacy lock"),
                b"",
                "refused legacy lock must not be overwritten"
            );
        }

        // Case 2: zero budget with legacy empty locks AND leftover segments must refuse due
        // to budget expiry without removing any segment.
        let mut planted = Vec::new();
        for slot in 0..legacy_slots {
            let seg = segments
                .path()
                .join(format!("{}seg.shm_state", resim_namespace_prefix(slot)));
            std::fs::write(&seg, b"").expect("plant legacy segment");
            planted.push(seg);
        }
        let err_with_segs = claim_and_reclaim_resim_slot_in_range(
            &root.config(),
            locks.path(),
            Some(segments.path()),
            0..legacy_slots,
            Duration::ZERO,
        )
        .expect_err("zero startup budget must refuse legacy empty locks with segments");
        assert!(
            err_with_segs.contains("budget"),
            "refusal must explain startup budget expiry: {err_with_segs}"
        );
        for seg in &planted {
            assert!(seg.exists(), "budget-refused slot must preserve segment");
        }
    }

    #[test]
    fn legacy_empty_locks_share_one_bounded_segment_scan() {
        use std::cell::Cell;

        struct CountingScanner {
            scan_all_calls: Cell<usize>,
            count_slot_calls: Cell<usize>,
            plant_on_confirm: Option<(u32, PathBuf)>,
            force_budget_exhausted: bool,
        }

        impl LegacySegmentScanOps for CountingScanner {
            fn scan_all(
                &self,
                segment_dir: Option<&Path>,
                started: Instant,
                budget: Duration,
            ) -> Result<BTreeMap<u32, u64>, SegmentScanError> {
                self.scan_all_calls.set(self.scan_all_calls.get() + 1);
                if self.force_budget_exhausted {
                    return Err(SegmentScanError::BudgetExhausted);
                }
                RealLegacySegmentScanOps.scan_all(segment_dir, started, budget)
            }

            fn count_slot(
                &self,
                segment_dir: Option<&Path>,
                slot: u32,
                started: Instant,
                budget: Duration,
            ) -> Result<u64, SegmentScanError> {
                self.count_slot_calls.set(self.count_slot_calls.get() + 1);
                if let Some((target_slot, ref seg_path)) = self.plant_on_confirm {
                    if slot == target_slot {
                        std::fs::write(seg_path, b"").expect("plant racing segment");
                    }
                }
                RealLegacySegmentScanOps.count_slot(segment_dir, slot, started, budget)
            }
        }

        let root = IsolatedRoot::mint("rslotshare");

        // Arm A: 20 legacy empty locks with leftover segments share ONE scan,
        // preserve every segment, and claim fresh slot 20.
        let locks = tempfile::tempdir().expect("lock dir");
        let segments = tempfile::tempdir().expect("segment dir");
        let mut planted = Vec::new();
        for slot in 0..20u32 {
            let lock_path = locks
                .path()
                .join(format!("{RESIM_NAMESPACE_MARKER}{slot:08x}.lock"));
            std::fs::write(&lock_path, b"").expect("write legacy lock");
            let seg = segments
                .path()
                .join(format!("{}seg.shm_state", resim_namespace_prefix(slot)));
            std::fs::write(&seg, b"").expect("plant segment");
            planted.push(seg);
        }
        let scanner = CountingScanner {
            scan_all_calls: Cell::new(0),
            count_slot_calls: Cell::new(0),
            plant_on_confirm: None,
            force_budget_exhausted: false,
        };
        let claimed = claim_and_reclaim_resim_slot_with_scanner(
            &root.config(),
            locks.path(),
            Some(segments.path()),
            0..21,
            Duration::from_secs(10),
            &scanner,
        )
        .expect("fresh slot 20 claimed after skipping 20 legacy locks");
        assert_eq!(claimed.slot, 20);
        assert_eq!(
            scanner.scan_all_calls.get(),
            1,
            "all 20 legacy empty locks must share one segment directory scan"
        );
        assert_eq!(scanner.count_slot_calls.get(), 0);
        for seg in &planted {
            assert!(seg.exists(), "legacy segment must remain intact");
        }
        drop(claimed);

        // Arm B: legacy slot 0 has a segment and legacy slot 1 has none ->
        // one shared scan plus one locked confirmation on slot 1, then slot 1 is claimed.
        let locks_b = tempfile::tempdir().expect("lock dir b");
        let segments_b = tempfile::tempdir().expect("segment dir b");
        for slot in 0..2u32 {
            std::fs::write(
                locks_b
                    .path()
                    .join(format!("{RESIM_NAMESPACE_MARKER}{slot:08x}.lock")),
                b"",
            )
            .expect("write legacy lock");
        }
        let seg0 = segments_b
            .path()
            .join(format!("{}seg.shm_state", resim_namespace_prefix(0)));
        std::fs::write(&seg0, b"").expect("plant slot 0 segment");
        let scanner_b = CountingScanner {
            scan_all_calls: Cell::new(0),
            count_slot_calls: Cell::new(0),
            plant_on_confirm: None,
            force_budget_exhausted: false,
        };
        let claimed_b = claim_and_reclaim_resim_slot_with_scanner(
            &root.config(),
            locks_b.path(),
            Some(segments_b.path()),
            0..2,
            Duration::from_secs(10),
            &scanner_b,
        )
        .expect("clean legacy slot 1 claimed");
        assert_eq!(claimed_b.slot, 1);
        assert_eq!(scanner_b.scan_all_calls.get(), 1);
        assert_eq!(scanner_b.count_slot_calls.get(), 1);
        assert!(seg0.exists(), "slot 0 segment must remain intact");
        drop(claimed_b);

        // Arm C: legacy slot 0 itself has 0 segments -> one scan while holding
        // slot 0's lock is sufficient (zero extra confirmation scans).
        let locks_c = tempfile::tempdir().expect("lock dir c");
        let segments_c = tempfile::tempdir().expect("segment dir c");
        std::fs::write(locks_c.path().join("cer_resim_00000000.lock"), b"")
            .expect("write legacy lock 0");
        let scanner_c = CountingScanner {
            scan_all_calls: Cell::new(0),
            count_slot_calls: Cell::new(0),
            plant_on_confirm: None,
            force_budget_exhausted: false,
        };
        let claimed_c = claim_and_reclaim_resim_slot_with_scanner(
            &root.config(),
            locks_c.path(),
            Some(segments_c.path()),
            0..2,
            Duration::from_secs(10),
            &scanner_c,
        )
        .expect("clean legacy slot 0 claimed");
        assert_eq!(claimed_c.slot, 0);
        assert_eq!(scanner_c.scan_all_calls.get(), 1);
        assert_eq!(scanner_c.count_slot_calls.get(), 0);
        drop(claimed_c);

        // Arm D (adversarial race): segment appears on slot 1 between the shared
        // scan and locking slot 1 -> locked confirmation catches it, preserves it,
        // and claims slot 2.
        let locks_d = tempfile::tempdir().expect("lock dir d");
        let segments_d = tempfile::tempdir().expect("segment dir d");
        for slot in 0..2u32 {
            std::fs::write(
                locks_d
                    .path()
                    .join(format!("{RESIM_NAMESPACE_MARKER}{slot:08x}.lock")),
                b"",
            )
            .expect("write legacy lock");
        }
        let seg_d0 = segments_d
            .path()
            .join(format!("{}seg.shm_state", resim_namespace_prefix(0)));
        std::fs::write(&seg_d0, b"").expect("plant slot 0 segment");
        let seg_d1 = segments_d
            .path()
            .join(format!("{}race.shm_state", resim_namespace_prefix(1)));
        let scanner_d = CountingScanner {
            scan_all_calls: Cell::new(0),
            count_slot_calls: Cell::new(0),
            plant_on_confirm: Some((1, seg_d1.clone())),
            force_budget_exhausted: false,
        };
        let claimed_d = claim_and_reclaim_resim_slot_with_scanner(
            &root.config(),
            locks_d.path(),
            Some(segments_d.path()),
            0..3,
            Duration::from_secs(10),
            &scanner_d,
        )
        .expect("raced slot 1 skipped and slot 2 claimed");
        assert_eq!(claimed_d.slot, 2);
        assert_eq!(scanner_d.scan_all_calls.get(), 1);
        assert_eq!(scanner_d.count_slot_calls.get(), 1);
        assert!(seg_d0.exists());
        assert!(seg_d1.exists(), "racing legacy segment must be preserved");
        drop(claimed_d);

        // Arm E (error arm): unreadable/non-dir segment_dir caches Io error across
        // legacy locks and fails closed on one scan.
        let not_a_dir = locks_d.path().join("regular_file_not_a_dir");
        std::fs::write(&not_a_dir, b"x").expect("write non-dir");
        let scanner_e = CountingScanner {
            scan_all_calls: Cell::new(0),
            count_slot_calls: Cell::new(0),
            plant_on_confirm: None,
            force_budget_exhausted: false,
        };
        let err_e = claim_and_reclaim_resim_slot_with_scanner(
            &root.config(),
            locks_d.path(),
            Some(&not_a_dir),
            0..2,
            Duration::from_secs(10),
            &scanner_e,
        )
        .expect_err("non-directory segment_dir must fail closed");
        assert!(
            err_e.contains("segment directory could not be inspected"),
            "{err_e}"
        );
        assert_eq!(scanner_e.scan_all_calls.get(), 1);

        // Arm F (budget expiry cached across remaining legacy locks): once
        // scan_all reports BudgetExhausted on slot 0, remaining legacy locks
        // short-circuit without rescanning.
        let scanner_f = CountingScanner {
            scan_all_calls: Cell::new(0),
            count_slot_calls: Cell::new(0),
            plant_on_confirm: None,
            force_budget_exhausted: true,
        };
        let err_f = claim_and_reclaim_resim_slot_with_scanner(
            &root.config(),
            locks.path(),
            Some(segments.path()),
            0..20,
            Duration::from_secs(10),
            &scanner_f,
        )
        .expect_err("budget exhaustion must refuse all legacy locks");
        assert!(err_f.contains("budget"), "{err_f}");
        assert_eq!(scanner_f.scan_all_calls.get(), 1);
        assert_eq!(scanner_f.count_slot_calls.get(), 0);
    }

    #[test]
    fn bounded_segment_scan_stops_early_when_budget_expires_mid_directory() {
        let dir = tempfile::tempdir().expect("segment dir");
        for i in 0..64u32 {
            let name = format!("unrelated_entry_{i:03}.tmp");
            std::fs::write(dir.path().join(name), b"").expect("write entry");
        }
        std::fs::write(
            dir.path()
                .join(format!("{}seg.shm_state", resim_namespace_prefix(0))),
            b"",
        )
        .expect("write slot segment");

        // Mid-directory expiry: stops after 4 entries instead of scanning all 65.
        let mut checks_all = 0usize;
        let res_all = scan_all_slot_segments_bounded(Some(dir.path()), || {
            checks_all += 1;
            checks_all >= 5
        });
        assert_eq!(res_all, Err(SegmentScanError::BudgetExhausted));
        assert_eq!(
            checks_all, 5,
            "scan_all must stop immediately on budget expiry"
        );

        let mut checks_one = 0usize;
        let res_one = count_slot_segments_bounded(Some(dir.path()), 0, || {
            checks_one += 1;
            checks_one >= 5
        });
        assert_eq!(res_one, Err(SegmentScanError::BudgetExhausted));
        assert_eq!(
            checks_one, 5,
            "count_slot must stop immediately on budget expiry"
        );

        // Pre-open expiry: refuses before opening read_dir even on a non-dir path.
        let not_a_dir = dir.path().join("unrelated_entry_000.tmp");
        let mut pre_checks = 0usize;
        let res_pre = scan_all_slot_segments_bounded(Some(&not_a_dir), || {
            pre_checks += 1;
            true
        });
        assert_eq!(res_pre, Err(SegmentScanError::BudgetExhausted));
        assert_eq!(pre_checks, 1);

        // Unexpired scan sees exact hand-written count.
        let full = scan_all_slot_segments_bounded(Some(dir.path()), || false)
            .expect("unexpired scan succeeds");
        assert_eq!(full.get(&0), Some(&1));
        assert_eq!(full.len(), 1);
    }

    #[test]
    fn registry_walk_panic_is_contained_in_report_only_remove_and_reclaim() {
        struct PanickingOps;
        impl RegistryWalkOps for PanickingOps {
            fn count_nodes(&self, _ix_config: &Config) -> Result<NodeCounts, String> {
                run_guarded_registry_walk(|| -> Result<NodeCounts, String> {
                    panic!("stray filename in report-only walk");
                })
            }

            fn list_node_count(&self, _ix_config: &Config) -> Result<u64, String> {
                run_guarded_registry_walk(|| -> Result<u64, String> {
                    panic!("stray filename in post-sweep walk");
                })
            }

            fn sweep_bounded(
                &self,
                _ix_config: &Config,
                _budget: Duration,
            ) -> cerulion_core::transport::dead_node_sweep::BoundedCleanup {
                cerulion_core::transport::dead_node_sweep::BoundedCleanup::default()
            }

            fn sweep_unbounded(
                &self,
                _ix_config: &Config,
            ) -> Result<crate::ipc_cleanup::CleanupReport, String> {
                run_guarded_registry_walk(
                    || -> Result<crate::ipc_cleanup::CleanupReport, String> {
                        panic!("stray filename in unbounded sweep");
                    },
                )
            }
        }

        let root = IsolatedRoot::mint("rslotpanic");
        let locks = tempfile::tempdir().expect("lock dir");
        let segments = tempfile::tempdir().expect("segment dir");
        let lock_path = locks.path().join("cer_resim_00000000.lock");
        let owner = SlotOwnerRecord::from_config(&root.config());
        std::fs::write(
            &lock_path,
            serde_json::to_vec(&owner).expect("encode owner"),
        )
        .expect("write owner lock");
        let seg = segments
            .path()
            .join(format!("{}seg.shm_state", resim_namespace_prefix(0)));
        std::fs::write(&seg, b"").expect("plant segment");

        // ReportOnly catches the panic, records a registry error, and touches nothing.
        let report_only = sweep_resim_slot_range_with_ops(
            &root.config(),
            locks.path(),
            Some(segments.path()),
            0..1,
            SweepMode::ReportOnly,
            None,
            &PanickingOps,
        );
        assert_eq!(report_only.registry_errors, 1, "{report_only:?}");
        assert!(!report_only.nodes_converged());
        assert!(
            report_only
                .errors
                .iter()
                .any(|e| e.contains("stray filename in report-only walk")),
            "{:?}",
            report_only.errors
        );
        assert!(seg.exists());

        // Bounded Remove (ExitHygieneGuard path) catches the post-sweep walk panic.
        let bounded_remove = sweep_resim_slot_range_with_ops(
            &root.config(),
            locks.path(),
            Some(segments.path()),
            0..1,
            SweepMode::Remove,
            Some(Duration::from_secs(5)),
            &PanickingOps,
        );
        assert_eq!(bounded_remove.registry_errors, 1, "{bounded_remove:?}");
        assert!(!bounded_remove.nodes_converged());
        assert!(
            bounded_remove
                .errors
                .iter()
                .any(|e| e.contains("stray filename in post-sweep walk")),
            "{:?}",
            bounded_remove.errors
        );
        assert!(seg.exists(), "panicked walk must not remove segments");

        // Unbounded Remove (cerulion clean path) catches both sweep and post-sweep panics.
        let unbounded_remove = sweep_resim_slot_range_with_ops(
            &root.config(),
            locks.path(),
            Some(segments.path()),
            0..1,
            SweepMode::Remove,
            None,
            &PanickingOps,
        );
        assert_eq!(unbounded_remove.registry_errors, 2, "{unbounded_remove:?}");
        assert!(!unbounded_remove.nodes_converged());
        assert!(
            unbounded_remove
                .errors
                .iter()
                .any(|e| e.contains("stray filename in unbounded sweep")),
            "{:?}",
            unbounded_remove.errors
        );
        assert!(
            seg.exists(),
            "panicked unbounded sweep must not remove segments"
        );

        // Reclaim catches the post-sweep walk panic and refuses the slot cleanly.
        let ix = resim_ix_config(&root.config(), 0).expect("slot 0 config");
        let reclaim = reclaim_resim_slot_with_ops(
            &ix,
            0,
            Some(segments.path()),
            Duration::from_secs(5),
            &PanickingOps,
        );
        let refusal = reclaim.refusal().expect("panicked reclaim refuses slot");
        assert!(
            refusal.contains("stray filename in post-sweep walk"),
            "{refusal}"
        );
        assert!(seg.exists(), "panicked reclaim must not remove segments");
    }

    #[test]
    fn every_node_list_call_in_resim_slots_goes_through_guarded_helper() {
        let src: String = include_str!("resim_slots.rs")
            .lines()
            .map(|l| match l.find("//") {
                Some(i) => &l[..i],
                None => l,
            })
            .collect::<Vec<_>>()
            .join("\n");
        let production_src = src
            .split("mod tests {")
            .next()
            .expect("resim_slots production source");
        assert_eq!(
            production_src.matches("Node::<CerService>::list(").count(),
            1,
            "all Node::list walks in resim_slots.rs must route through the single guarded helper"
        );
        assert_eq!(
            production_src
                .matches("sweep_dead_nodes_with_config(")
                .count(),
            1,
            "all unbounded dead-node sweeps in resim_slots.rs must route through the guarded helper"
        );
        assert!(
            production_src.contains("fn list_nodes_guarded("),
            "resim_slots.rs must define list_nodes_guarded"
        );
        assert!(
            production_src.contains("fn sweep_dead_nodes_guarded("),
            "resim_slots.rs must define sweep_dead_nodes_guarded"
        );
        let global_fn = production_src
            .split("pub fn sweep_resim_slots_global(")
            .nth(1)
            .expect("sweep_resim_slots_global exists");
        let catch_pos = global_fn
            .find("catch_unwind(")
            .expect("sweep_resim_slots_global wraps in catch_unwind");
        let config_pos = global_fn
            .find("Config::global_config()")
            .expect("sweep_resim_slots_global resolves global config");
        assert!(
            catch_pos < config_pos,
            "Config::global_config() must stay inside catch_unwind"
        );
    }
}
